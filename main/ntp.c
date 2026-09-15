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
/* s_stats 只由 ntp 任务更新；其他任务（HTTP 面板、串口诊断）读 s_stats_pub
 * 快照，避免看到"请求已加、应答还没加"这类中间态。 */
static portMUX_TYPE  s_stats_lock = portMUX_INITIALIZER_UNLOCKED;
static ntp_stats_t   s_stats_pub;

static void publish_stats(void)
{
    portENTER_CRITICAL(&s_stats_lock);
    s_stats_pub = s_stats;
    portEXIT_CRITICAL(&s_stats_lock);
}

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
/* have_hw_ts / hw_rx 由 ntp_task 为"本包"弹出后传入（见那里的说明） */
static void handle_request(int sock, const uint8_t *req, int len,
                           const struct sockaddr_in *from, socklen_t fromlen,
                           uint64_t rx_tb, bool have_hw_ts, uint64_t hw_rx)
{
    uint8_t resp[48];

    if (len < 48) {
        s_stats.bad++;
        return;
    }
    /* 源地址过滤：只接受普通单播客户端。
     * sin_addr 在内存里就是网络序的四字节 [a.b.c.d]，按字节判断不依赖主机
     * 字节序。0.0.0.0/8（未配置/伪造）、224.0.0.0/4（组播）、240.0.0.0/4
     * （保留，含 255.255.255.255 受限广播）都不该成为 NTP 请求的源，
     * 放行它们只会扩大被当作放大反射源的面。 */
    const uint8_t *src = (const uint8_t *)(const void *)&from->sin_addr.s_addr;
    if (src[0] == 0 || src[0] >= 224) {
        s_stats.bad++;
        return;
    }

    uint8_t li_mode = req[0];
    uint8_t mode    = li_mode & 0x07;
    uint8_t vn      = (li_mode >> 3) & 0x07;
    /* 只响应 mode 3（client）。mode 4 是服务器自身的模式，响应它没有意义；
     * mode 1/2 是对称模式，本服务器不参与；其余（5 保留、6/7 控制/私有）
     * 更不该响应。少一类可被利用的报文，反射/环路面就小一分。 */
    if (mode != 3) {
        s_stats.bad++;
        return;
    }
    /* 只接受 3/4：NTPv1/v2 的报文语义与 v3/v4 有差异，混着回容易出错，
     * 现代实现全都是 3/4，其余按非法报文计数。 */
    if (vn < 3 || vn > 4) {
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

    /* 尚未建立绝对时间基准（从未与 RMC/PPS 对齐过）时，本地钟面与 UTC 无关，
     * 填不出 RFC 5905 要求的 Transmit/Receive Timestamp。以前这里会把 0 直接
     * 填进响应，chrony/ntpd/sntp 会判为 bogus 丢弃，或算出无意义的 offset。
     * 改为静默不回包：既符合协议，也避免被当成放大反射源。
     * （本包的帧时戳已在 ntp_task 里弹出，此处直接返回不会留下残项。） */
    if (!st.anchored) {
        s_stats.unsynced++;
        return;
    }

    /* 入站时戳：优先用以太网驱动层的帧到达时刻（由 ntp_task 为每个包弹出） */
    if (have_hw_ts) {
        rx_tb = hw_rx;
        s_stats.rx_ts_used++;
    } else {
        s_stats.rx_ts_miss++;
    }
    if (rx_tb > (uint64_t)CFG_NTP_RX_CORR_US) {
        rx_tb -= CFG_NTP_RX_CORR_US;
    }

    memset(resp, 0, sizeof(resp));

    resp[0] = (uint8_t)((st.li << 6) | (vn << 3) | 4);       /* LI + VN(回显 3/4) + Mode=4 */
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
         * 需要 CONFIG_LWIP_SO_RCVBUF=y（见 sdkconfig.defaults）。
         * 注意 rcvbuf 必须声明在条件编译之外：LWIP_SO_RCVBUF 未开启时会被
         * lwipopts.h 显式定义为 0（不是"未定义标识符"），若把声明也放进
         * #if 里，下面日志引用它就会直接编译失败。 */
        int rcvbuf = CFG_NTP_SO_RCVBUF;
#if LWIP_SO_RCVBUF
        if (setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) != 0) {
            ESP_LOGW(TAG, "SO_RCVBUF=%d 设置失败，沿用协议栈默认值", rcvbuf);
        }
#else
        ESP_LOGW(TAG, "本次构建未开启 LWIP_SO_RCVBUF，接收缓冲为协议栈默认值"
                      "（建议在 sdkconfig 中打开 CONFIG_LWIP_SO_RCVBUF）");
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

        /* 【关键】无论这个包最终是否被应答，都必须为它弹出一条帧时戳：
         * 驱动侧是按"每个目的端口为 123 的帧"入队的，若只在合法请求时才弹，
         * 被丢弃（畸形 / 令牌桶）的包就会在缓冲里留下时戳，后面的请求便会
         * 取到属于别人的时戳（在 200ms 窗口内会被采信），给 Receive
         * Timestamp 注入最多 200ms 的误差。 */
        uint64_t hw_rx   = 0;
        bool     have_hw = eth_if_take_rx_ts(&hw_rx, rx_tb);

        handle_request(sock, buf, n, &from, fromlen, rx_tb, have_hw, hw_rx);

        publish_stats();
    }
}

void ntp_server_start(void)
{
    /* 网络侧（lwIP tcpip 线程也固定在 CORE_NET），与时间链路隔核。
     * 优先级见 CFG_NTP_TASK_PRIO：必须低于 tcpip 线程，否则会优先级反转
     * —— 本任务的包是由 tcpip 线程投递到 socket 的。 */
    if (xTaskCreatePinnedToCore(ntp_task, "ntp", CFG_NTP_TASK_STACK, NULL,
                                CFG_NTP_TASK_PRIO, NULL, CFG_CORE_NET) != pdPASS) {
        ESP_LOGE(TAG, "ntp 任务创建失败（内存不足？），NTP 服务不可用");
    }
}

void ntp_get_stats(ntp_stats_t *st)
{
    portENTER_CRITICAL(&s_stats_lock);
    *st = s_stats_pub;
    portEXIT_CRITICAL(&s_stats_lock);
}
