/*
 * NTPv4 服务器实现
 */
#include "ntp.h"
#include "config.h"
#include "discipline.h"
#include "eth_if.h"

#include <string.h>
#include <errno.h>
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ntp";

static ntp_stats_t s_stats;

/* 令牌桶 */
static uint32_t s_tokens  = CFG_NTP_RATE_BURST;
static uint64_t s_tb_last = 0;
static uint64_t s_tb_rem  = 0;

static inline void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);  p[3] = (uint8_t)(v);
}

static inline uint32_t get_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

/* NTP 短格式（16.16）时间戳 */
static void put_short_ts(uint8_t *p, uint32_t sec, uint32_t frac16)
{
    p[0] = (uint8_t)(sec >> 8); p[1] = (uint8_t)sec;
    p[2] = (uint8_t)(frac16 >> 8); p[3] = (uint8_t)frac16;
}

static void put_ts(uint8_t *p, uint32_t sec, uint32_t frac32)
{
    put_u32(p, sec);
    put_u32(p + 4, frac32);
}

/* 微秒 -> NTP 短格式小数（2^-16 s） */
static inline uint32_t us_to_frac16(uint32_t us)
{
    return (uint32_t)(((uint64_t)us * 65536ULL) / 1000000ULL);
}

static bool rate_allow(uint64_t now)
{
    uint64_t dt = (s_tb_last == 0) ? 0 : (now - s_tb_last);
    if (dt > 1000000ULL) {
        dt = 1000000ULL;
    }
    uint64_t num = dt * (uint64_t)CFG_NTP_RATE_PPS + s_tb_rem;
    s_tokens  += (uint32_t)(num / 1000000ULL);
    s_tb_rem   = num % 1000000ULL;
    s_tb_last  = now;
    if (s_tokens > (uint32_t)CFG_NTP_RATE_BURST) {
        s_tokens = CFG_NTP_RATE_BURST;
    }
    if (s_tokens == 0) {
        return false;
    }
    s_tokens--;
    return true;
}

/* ------------------------------------------------------------------ */
static void handle_request(int sock, const uint8_t *req, int len,
                           const struct sockaddr_in *from, socklen_t fromlen,
                           uint64_t rx_tb)
{
    uint8_t resp[48];

    if (len < 48) {
        s_stats.bad++;
        return;
    }
    uint8_t li_mode = req[0];
    uint8_t mode    = li_mode & 0x07;
    uint8_t vn      = (li_mode >> 3) & 0x07;
    /* mode 3 = client, 4 = server(对称模式)。其余（含 6/7 控制/私有）一律不响应，
     * 避免被当作放大反射源 */
    if (mode != 3 && mode != 4) {
        s_stats.bad++;
        return;
    }
    if (vn == 0 || vn > 4) {
        s_stats.bad++;
        return;
    }

    uint64_t now = tb_now_us();
    if (!rate_allow(now)) {
        s_stats.dropped++;
        return;
    }
    s_stats.requests++;

    disc_status_t st;
    discipline_get_status(&st);

    /* 入站时戳：优先用以太网驱动层的帧到达时刻 */
    uint64_t hw_rx;
    if (eth_if_take_rx_ts(&hw_rx, now)) {
        rx_tb = hw_rx;
        s_stats.rx_ts_used++;
    } else {
        s_stats.rx_ts_miss++;
    }
    if (rx_tb > (uint64_t)CFG_NTP_RX_CORR_US) {
        rx_tb -= CFG_NTP_RX_CORR_US;
    }

    memset(resp, 0, sizeof(resp));

    uint8_t out_vn = (vn < 4) ? vn : 4;
    resp[0] = (uint8_t)((st.li << 6) | (out_vn << 3) | 4);   /* LI + VN + Mode=4(server) */
    resp[1] = (uint8_t)st.stratum;

    uint8_t poll = req[2];
    if (poll < 4 || poll > 17) {
        poll = 6;
    }
    resp[2] = poll;
    resp[3] = (uint8_t)(int8_t)CFG_NTP_PRECISION;            /* 2^-20 s ≈ 0.95 µs */

    /* Root Delay = 0（本服务器就是参考源） */
    put_short_ts(&resp[4], 0, 0);
    /* Root Dispersion：基准 + 抖动 + 守时漂移 */
    put_short_ts(&resp[8], (uint32_t)(st.root_disp_us / 1000000u),
                 us_to_frac16((uint32_t)(st.root_disp_us % 1000000u)));

    if (st.stratum <= 2) {
        resp[12] = 'G'; resp[13] = 'P'; resp[14] = 'S'; resp[15] = 0;
    } else {
        resp[12] = 0; resp[13] = 0; resp[14] = 0; resp[15] = 0;
    }

    if (st.stratum <= 2) {
        put_ts(&resp[16], discipline_ref_sec(), 0);          /* Reference Timestamp */
    }
    memcpy(&resp[24], &req[40], 8);                          /* Originate = 客户端 Transmit */

    uint32_t rs, rf;
    if (st.anchored && discipline_get_utc(rx_tb, &rs, &rf)) {
        put_ts(&resp[32], rs + (uint32_t)NTP_EPOCH_OFFSET, rf);
    }

    /* 发送时戳：紧贴 sendto 之前取，尽量贴近实际上线路时刻 */
    uint64_t xmit_tb = tb_now_us() + (uint64_t)CFG_NTP_TX_CORR_US;
    uint32_t xs, xf;
    if (st.anchored && discipline_get_utc(xmit_tb, &xs, &xf)) {
        put_ts(&resp[40], xs + (uint32_t)NTP_EPOCH_OFFSET, xf);
    }

    if (sendto(sock, resp, sizeof(resp), 0,
               (const struct sockaddr *)from, fromlen) < 0) {
        s_stats.send_fail++;
    } else {
        s_stats.responses++;
    }
}

static void ntp_task(void *arg)
{
    (void)arg;
    if (esp_task_wdt_add(NULL) != ESP_OK) {
        ESP_LOGW(TAG, "TWDT 订阅失败（任务看门狗未启用？）");
    }

    int sock = -1;
    while (1) {
        sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (sock < 0) {
            ESP_LOGE(TAG, "socket 创建失败，1s 后重试");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_task_wdt_reset();
            continue;
        }

        /* 增大接收缓冲：突发请求先在协议栈里排队，而不是压垮主循环。
         * 需要 CONFIG_LWIP_SO_RCVBUF=y（见 sdkconfig.defaults） */
#if LWIP_SO_RCVBUF
        int rcvbuf = CFG_NTP_SO_RCVBUF;
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));
#endif
#if LWIP_SO_RCVTIMEO
        struct timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = 500000;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

        struct sockaddr_in sa;
        memset(&sa, 0, sizeof(sa));
        sa.sin_family      = AF_INET;
        sa.sin_port        = htons((uint16_t)CFG_NTP_PORT);
        sa.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(sock, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
            ESP_LOGE(TAG, "bind %d 失败，1s 后重试", CFG_NTP_PORT);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_task_wdt_reset();
            continue;
        }
        ESP_LOGI(TAG, "NTP 服务器已启动 UDP/%d (SO_RCVBUF=%d)", CFG_NTP_PORT, rcvbuf);
        break;
    }

    uint8_t buf[128];
    while (1) {
        esp_task_wdt_reset();

        struct sockaddr_in from;
        socklen_t fromlen = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &fromlen);
        if (n < 0) {
            continue;              /* 超时，继续循环喂狗 */
        }
        uint64_t rx_tb = tb_now_us();   /* 兜底的入站时戳 */
        handle_request(sock, buf, n, &from, fromlen, rx_tb);
    }
}

void ntp_server_start(void)
{
    /* 网络侧（lwIP tcpip 线程也固定在 CORE_NET），与时间链路隔核 */
    xTaskCreatePinnedToCore(ntp_task, "ntp", CFG_NTP_TASK_STACK, NULL,
                            CFG_NTP_TASK_PRIO, NULL, CFG_CORE_NET);
}

void ntp_get_stats(ntp_stats_t *st)
{
    *st = s_stats;
}
