/*
 * GNSS 原始报文 TCP 转发实现（远程"串口调试助手"）
 *
 * 目的：把 GNSS 模块吐在串口上的原始字节原样转发到 TCP 端口（默认 8880），
 * 于是不必抱着笔记本蹲在设备旁接串口线 —— 远程用串口调试助手 / telnet / nc
 * 连上来，看到的就是模块真实的输出：NMEA 文本行、模块对 UBX 查询的二进制
 * 应答、校验失败的坏帧，乃至波特率探测期间收到的乱码。
 * 面板上的统计是"结论"，这里是"一手证据"，两者合起来才好判断模块的健康状况
 * （典型用法：GGA 里 sats=0、RMC 里 status='V'，同时报文流却一直很流畅 →
 *   模块供电与串口都没问题，纯粹是天线/视野拿不到定位）。
 *
 * 数据路径：gps.c 读完 UART、跑完 NMEA 解析之后调用 gnss_tcp_feed()，那里只做
 * 一次无锁 memcpy（单生产者单消费者环形缓冲），不取信号量、不阻塞、不碰网络 ——
 * PPS/NMEA 的时标与解析顺序完全不受影响。真正的 socket 收发全在 CORE_NET 上的
 * 独立任务里做，与 lwIP tcpip 线程同核。
 *
 * 为什么用"非阻塞 + 轮询"而不是 select()：本服务只出不进，每 CFG_GNSS_TCP_POLL_MS
 * 醒一次即可；这样不必关心 lwip 的 FD_SETSIZE 与 socket fd 编号范围（FD_SET
 * 用的是 fd 位图，fd 超出 FD_SETSIZE 会越界写），代码也短得多。
 */
#include "gnss_tcp.h"

#if CFG_GNSS_TCP_ENABLE

#include "gps.h"
#include "discipline.h"

#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <time.h>

#include "lwip/sockets.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "gnss_tcp";

_Static_assert((CFG_GNSS_TCP_RING & (CFG_GNSS_TCP_RING - 1)) == 0,
               "CFG_GNSS_TCP_RING 必须是 2 的幂");
#define RING_MASK ((uint32_t)CFG_GNSS_TCP_RING - 1u)

/* ====================== 环形缓冲（写侧 / 读侧各一个任务）============
 * head / tail 都是"只增计数器"，已用字节数用无符号差表示，回绕天然正确；
 * 写侧（GPS 任务）只写 head，读侧（本模块的 TCP 任务）只写 tail，所以不需要
 * 任何锁 —— 这点很关键：写侧在 CORE_TIME 上跑，绝不能因为等信号量被推迟，
 * 否则会污染 NMEA 到达时刻的估计，直接变成授时误差。 */
static uint8_t           s_ring[CFG_GNSS_TCP_RING];
static volatile uint32_t s_head;
static volatile uint32_t s_tail;
static volatile uint32_t s_drop_bytes;    /* 写侧累加：缓冲放不下而丢弃的字节 */
static volatile uint32_t s_sent_bytes;    /* 发侧累加：已成功发出的字节       */
static volatile bool     s_started;       /* start 之后写侧才开始投递         */

/* ---- 统计 ---- */
static volatile uint32_t s_clients;        /* 当前连接数                 */
static volatile uint32_t s_clients_total;  /* 累计接受过的连接数         */
static volatile uint32_t s_rejected;       /* 因达上限被拒绝的连接数     */

typedef struct {
    int  fd;                 /* -1 = 空槽位 */
    char ip[16];
} gnss_client_t;

static gnss_client_t s_cl[CFG_GNSS_TCP_MAX_CLIENTS];

/* ====================== 写侧（GPS 任务） ==========================
 * 只在 gps.c 里、读完 UART 并跑完 NMEA 解析之后调用。必须保持"几微秒内返回、
 * 绝不休眠"：它是时基链路的一部分。 */
void gnss_tcp_feed(const uint8_t *data, size_t len)
{
    uint32_t h, t, room, idx, first;

    if (!s_started || len == 0) {
        return;
    }
    h    = s_head;
    t    = __atomic_load_n(&s_tail, __ATOMIC_ACQUIRE);
    room = (uint32_t)CFG_GNSS_TCP_RING - (h - t);
    if (len > room) {
        /* 缓冲不足：丢掉这批里放不下的尾部并计数。
         * 为什么不覆盖还没被读走的旧数据：覆盖会让客户端收到的字节流中间
         * "缺一截"，排障时比丢数据本身更误导（NMEA 行被咬掉半句会被误判成
         * 串口丢字节）。正常情况下客户端是跟得上的（115200 ≈ 11.5 KB/s，
         * 4096 B 缓冲足够吸收一整秒的突发）。 */
        __atomic_add_fetch(&s_drop_bytes, (uint32_t)(len - room), __ATOMIC_RELAXED);
        len = room;
    }
    if (len == 0) {
        return;
    }
    idx   = h & RING_MASK;
    first = (uint32_t)CFG_GNSS_TCP_RING - idx;
    if (first > (uint32_t)len) {
        first = (uint32_t)len;
    }
    memcpy(&s_ring[idx], data, first);
    if ((uint32_t)len > first) {
        memcpy(&s_ring[0], data + first, (uint32_t)len - first);
    }
    __atomic_store_n(&s_head, h + (uint32_t)len, __ATOMIC_RELEASE);
}

/* ====================== 读侧（TCP 任务） ========================== */
static uint32_t ring_read(uint8_t *dst, uint32_t max)
{
    uint32_t t = s_tail;
    uint32_t h = __atomic_load_n(&s_head, __ATOMIC_ACQUIRE);
    uint32_t used = h - t;
    uint32_t idx, first;

    if (used == 0) {
        return 0;
    }
    if (used > max) {
        used = max;
    }
    idx   = t & RING_MASK;
    first = (uint32_t)CFG_GNSS_TCP_RING - idx;
    if (first > used) {
        first = used;
    }
    memcpy(dst, &s_ring[idx], first);
    if (used > first) {
        memcpy(dst + first, &s_ring[0], used - first);
    }
    __atomic_store_n(&s_tail, t + used, __ATOMIC_RELEASE);
    return used;
}

/* 没有任何客户端时清空缓冲：不这样做的话，缓冲会一直顶满、drop 计数一直涨，
 * 面板上看起来像故障，其实只是没人接。 */
static void ring_drop_all(void)
{
    __atomic_store_n(&s_tail, __atomic_load_n(&s_head, __ATOMIC_ACQUIRE),
                     __ATOMIC_RELEASE);
}

static int sock_set_nonblock(int fd)
{
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl < 0) {
        fl = 0;
    }
    return fcntl(fd, F_SETFL, fl | O_NONBLOCK);
}

/* 非阻塞发送：一次尽量发完，出现 EAGAIN 就返回 -1 让调用方断开该客户端。
 * 为什么不排队重试：诊断用途下"这个客户端少收一段"远比"拖住整个任务、把
 * 其他客户端的实时性也一起拖坏"更可接受 —— 慢客户端断开后重连即可，
 * 重连看到的本来就是最新的真实报文流。 */
static int send_all(int fd, const uint8_t *p, uint32_t n)
{
    uint32_t off = 0;

    while (off < n) {
        int k = send(fd, p + off, (size_t)(n - off), 0);
        if (k > 0) {
            off += (uint32_t)k;
            continue;
        }
        if (k < 0 && errno == EINTR) {
            continue;
        }
        if (off > 0) {
            __atomic_add_fetch(&s_sent_bytes, off, __ATOMIC_RELAXED);
        }
        return -1;
    }
    __atomic_add_fetch(&s_sent_bytes, off, __ATOMIC_RELAXED);
    return 0;
}

/* 把 Unix 秒格式化成 "YYYY-MM-DD hh:mm:ss UTC"。
 * 目标缓冲大小以参数传入（**不要**写成 snprintf(x, sizeof(x), ...)）：
 * 本工程 cflags 带 -Werror，-Wformat-truncation 会按"%04d 的每个 int 都取
 * -2147483648"这种最坏情形判定截断，6 个字段合计 75 字节，从而直接把
 * snprintf 判成编译错误；缓冲大小走参数后编译器不再做该推断（monitor.c 的
 * fmt_utc 一直是这个写法，原因相同）。 */
static void fmt_utc_line(uint32_t sec, char *out, size_t n)
{
    time_t    t = (time_t)sec;
    struct tm tm = { 0 };

    if (gmtime_r(&t, &tm) == NULL) {
        return;
    }
    snprintf(out, n, "%04d-%02d-%02d %02d:%02d:%02d UTC",
             tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
             tm.tm_hour, tm.tm_min, tm.tm_sec);
}

/* 连接时的一次性提示。以 '#' 开头，任何 NMEA 解析器都会忽略；但人一眼就能
 * 确认"确实连上了、当前波特率是多少、时基是否已锁定"，不必先干等一个 1 Hz
 * 的报文突发（模块不输出语句时，屏幕上会一直什么都没有）。 */
static void send_banner(int fd)
{
    gps_status_t g;
    char         stamp[40] = "unsynced (no UTC yet)";   /* 最长 24 字节："2026-09-15 04:00:31 UTC" */
    /* 三行合计实际最长约 214 字节。这里仍留到 320：如果编译器把 stamp 的数组
     * 大小（39）当成 %s 的长度上界来算最坏情形，合计是 258 —— 缓冲取 256 就
     * 正好卡在边界上，多留一点余量免得哪天改个常量就撞上 -Wformat-truncation。 */
    char         buf[320];
    uint32_t     sec = 0, frac = 0;
    int          n;

    gps_get_status(&g);
    if (discipline_get_utc(tb_now_us(), &sec, &frac)) {
        fmt_utc_line(sec, stamp, sizeof(stamp));
    }

    n = snprintf(buf, sizeof(buf),
                 "# GNSS raw TCP feed - the bytes below come from the GNSS UART as-is\r\n"
                 "# %s | baud=%u fixq=%u sats=%u/%u %s\r\n"
                 "# Lines starting with '#' are added by this server, not by the GNSS module.\r\n",
                 stamp, (unsigned)g.baud, (unsigned)g.fix_quality,
                 (unsigned)g.sats_used, (unsigned)g.sats_view,
                 g.fix_valid ? "FIX" : "NOFIX");
    if (n > 0) {
        /* snprintf 返回的是"本该写入"的长度：真被截断时必须按实际容量发送，
         * 否则会把缓冲之外的字节一起发出去。 */
        size_t len = ((size_t)n < sizeof(buf)) ? (size_t)n : sizeof(buf) - 1u;
        (void)send_all(fd, (const uint8_t *)buf, (uint32_t)len);
    }
}

static void client_close(int i, const char *why)
{
    char     ip[16];
    uint32_t left;

    if (s_cl[i].fd < 0) {
        return;
    }
    memcpy(ip, s_cl[i].ip, sizeof(ip));
    close(s_cl[i].fd);
    s_cl[i].fd    = -1;
    s_cl[i].ip[0] = 0;
    left = __atomic_sub_fetch(&s_clients, 1, __ATOMIC_RELAXED);
    ESP_LOGW(TAG, "客户端 %s 断开（%s），剩余 %lu 个", ip, why, (unsigned long)left);
}

static void accept_new_clients(int srv)
{
    for (;;) {
        struct sockaddr_in from;
        socklen_t          fromlen = sizeof(from);
        int                fd = accept(srv, (struct sockaddr *)&from, &fromlen);
        int                slot = -1;

        if (fd < 0) {
            return;                 /* EAGAIN：待处理的连接已全部接纳 */
        }
        for (int i = 0; i < CFG_GNSS_TCP_MAX_CLIENTS; i++) {
            if (s_cl[i].fd < 0) {
                slot = i;
                break;
            }
        }
        if (slot < 0) {
            /* 上限保护：拒绝了也要说清楚为什么，否则用户看到的是"连上了却
             * 一个字节都没有"，全靠猜。先切成非阻塞再发，避免这个提示把
             * 整个转发任务卡在 send 上（对端不读时是可能的）。 */
            static const char busy[] = "# GNSS TCP feed: too many clients, try later\r\n";
            (void)sock_set_nonblock(fd);
            (void)send(fd, busy, sizeof(busy) - 1, 0);
            close(fd);
            __atomic_add_fetch(&s_rejected, 1, __ATOMIC_RELAXED);
            ESP_LOGW(TAG, "客户端数已达上限 %d，拒绝新连接", CFG_GNSS_TCP_MAX_CLIENTS);
            continue;
        }
        if (sock_set_nonblock(fd) != 0) {
            close(fd);
            continue;
        }

        s_cl[slot].fd = fd;
        {
            /* sockaddr_in.sin_addr 与 esp_ip4_addr_t 内存布局一致（都是网络序
             * 四字节），直接转换即可，不依赖主机字节序 */
            esp_ip4_addr_t a;
            a.addr = from.sin_addr.s_addr;
            esp_ip4addr_ntoa(&a, s_cl[slot].ip, (int)sizeof(s_cl[slot].ip));
        }
        ESP_LOGI(TAG, "客户端 %s 已连接（%lu/%d）", s_cl[slot].ip,
                 (unsigned long)__atomic_add_fetch(&s_clients, 1, __ATOMIC_RELAXED),
                 CFG_GNSS_TCP_MAX_CLIENTS);
        __atomic_add_fetch(&s_clients_total, 1, __ATOMIC_RELAXED);
        send_banner(fd);
    }
}

/* 探测客户端是否还在（串口调试助手不会向我们发数据，收到的字节一律丢弃；
 * recv 返回 0 表示对端正常关闭）。 */
static void reap_disconnected(void)
{
    for (int i = 0; i < CFG_GNSS_TCP_MAX_CLIENTS; i++) {
        uint8_t junk[64];
        int     k;

        if (s_cl[i].fd < 0) {
            continue;
        }
        k = recv(s_cl[i].fd, junk, sizeof(junk), MSG_DONTWAIT);
        if (k == 0) {
            client_close(i, "对端关闭");
        } else if (k < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            client_close(i, "读错误");
        }
    }
}

static void broadcast_pending(uint8_t *buf, uint32_t cap)
{
    uint32_t n;
    bool     any = false;

    for (int i = 0; i < CFG_GNSS_TCP_MAX_CLIENTS; i++) {
        if (s_cl[i].fd >= 0) {
            any = true;
            break;
        }
    }
    if (!any) {
        ring_drop_all();
        return;
    }
    while ((n = ring_read(buf, cap)) > 0) {
        for (int i = 0; i < CFG_GNSS_TCP_MAX_CLIENTS; i++) {
            if (s_cl[i].fd < 0) {
                continue;
            }
            if (send_all(s_cl[i].fd, buf, n) != 0) {
                client_close(i, "发送跟不上/失败");
            }
        }
    }
}

/* ====================== 任务 ====================================== */
static void gnss_tcp_task(void *arg)
{
    int     srv = -1;
    uint8_t buf[CFG_GNSS_TCP_TX_CHUNK];

    (void)arg;
    if (esp_task_wdt_add(NULL) != ESP_OK) {
        ESP_LOGW(TAG, "TWDT 订阅失败（任务看门狗未启用？）");
    }

    /* 建监听 socket。网络还没起来（DHCP 未完成）也能 bind：这里绑的是
     * INADDR_ANY，不依赖本机地址。 */
    while (srv < 0) {
        int                one = 1;
        struct sockaddr_in sa;

        srv = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (srv < 0) {
            ESP_LOGE(TAG, "socket 创建失败，1s 后重试");
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_task_wdt_reset();
            continue;
        }
        (void)setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

        memset(&sa, 0, sizeof(sa));
        sa.sin_family      = AF_INET;
        sa.sin_port        = htons((uint16_t)CFG_GNSS_TCP_PORT);
        sa.sin_addr.s_addr = htonl(INADDR_ANY);
        if (bind(srv, (struct sockaddr *)&sa, sizeof(sa)) < 0) {
            ESP_LOGE(TAG, "bind TCP/%d 失败（errno=%d），1s 后重试",
                     CFG_GNSS_TCP_PORT, errno);
            close(srv);
            srv = -1;
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_task_wdt_reset();
            continue;
        }
        if (listen(srv, CFG_GNSS_TCP_MAX_CLIENTS) < 0 || sock_set_nonblock(srv) != 0) {
            ESP_LOGE(TAG, "listen(%d) 或非阻塞设置失败，1s 后重试", CFG_GNSS_TCP_PORT);
            close(srv);
            srv = -1;
            vTaskDelay(pdMS_TO_TICKS(1000));
            esp_task_wdt_reset();
            continue;
        }
        ESP_LOGI(TAG, "GNSS 原始报文转发已启动: TCP/%d（最多 %d 个客户端，缓冲 %d B，"
                      "轮询 %d ms）", CFG_GNSS_TCP_PORT, CFG_GNSS_TCP_MAX_CLIENTS,
                 CFG_GNSS_TCP_RING, CFG_GNSS_TCP_POLL_MS);
    }

    while (1) {
        esp_task_wdt_reset();
        accept_new_clients(srv);
        reap_disconnected();
        broadcast_pending(buf, (uint32_t)sizeof(buf));
        vTaskDelay(pdMS_TO_TICKS(CFG_GNSS_TCP_POLL_MS));
    }
}

void gnss_tcp_start(void)
{
    for (int i = 0; i < CFG_GNSS_TCP_MAX_CLIENTS; i++) {
        s_cl[i].fd    = -1;
        s_cl[i].ip[0] = 0;
    }
    /* 先放通写侧：gps 任务可能已经跑起来了，任务创建失败也只是白存一屏缓冲，
     * 不会丢数据、更不会影响授时。 */
    s_started = true;

    /* 网络侧（与 lwIP tcpip 线程同核）。优先级见 CFG_GNSS_TCP_TASK_PRIO：
     * 与 NTP 任务同理，刻意低于 tcpip 线程，避免优先级反转。 */
    if (xTaskCreatePinnedToCore(gnss_tcp_task, "gnss_tcp", CFG_GNSS_TCP_TASK_STACK,
                                NULL, CFG_GNSS_TCP_TASK_PRIO, NULL,
                                CFG_CORE_NET) != pdPASS) {
        s_started = false;
        ESP_LOGE(TAG, "gnss_tcp 任务创建失败（内存不足？），远程查看原始报文不可用");
    }
}

#endif /* CFG_GNSS_TCP_ENABLE */

void gnss_tcp_get_stats(gnss_tcp_stats_t *st)
{
    memset(st, 0, sizeof(*st));
#if CFG_GNSS_TCP_ENABLE
    st->clients       = (uint8_t)__atomic_load_n(&s_clients, __ATOMIC_RELAXED);
    st->clients_total = __atomic_load_n(&s_clients_total, __ATOMIC_RELAXED);
    st->rejected      = __atomic_load_n(&s_rejected, __ATOMIC_RELAXED);
    st->sent_bytes    = __atomic_load_n(&s_sent_bytes, __ATOMIC_RELAXED);
    st->drop_bytes    = __atomic_load_n(&s_drop_bytes, __ATOMIC_RELAXED);
#endif
}
