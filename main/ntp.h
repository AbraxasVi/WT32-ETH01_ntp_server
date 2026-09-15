/*
 * NTPv4 服务器（RFC 5905）
 *  - 独立任务 + BSD socket，SO_RCVBUF 放大，令牌桶限速，突发流量不会阻塞
 *    lwIP tcpip 线程；任务优先级刻意低于 tcpip（见 CFG_NTP_TASK_PRIO），
 *    因为包是由 tcpip 线程投递到 socket 的，高过它会造成优先级反转
 *  - 只响应 mode 3（client），并过滤 0.0.0.0/8、组播、保留网段的源地址，
 *    缩小被当作放大反射源的面
 *  - 入站时戳优先取以太网驱动层的帧到达时刻，退化为 socket 唤醒时刻
 *  - Stratum / Root Dispersion / Precision / LI 全部按锁定状态动态填写；
 *    尚未建立绝对时间基准时**不回包**（不发出 transmit 时戳为 0 的非法响应）
 */
#ifndef NTP_H
#define NTP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t requests;      /* 收到的合法请求 */
    uint32_t responses;     /* 已应答 */
    uint32_t unsynced;      /* 合法请求，但本机尚无绝对时间基准 -> 按策略不回包 */
    uint32_t bad;           /* 非法/不支持的报文（含源地址被过滤、mode/vn 不符） */
    uint32_t dropped;       /* 被令牌桶丢弃 */
    uint32_t rx_ts_used;    /* 采用了驱动层帧时戳的请求数（只统计被采纳的合法请求） */
    uint32_t rx_ts_miss;    /* 退化为 socket 唤醒时刻的请求数（口径同上） */
    uint32_t send_fail;     /* sendto 失败的次数 */
} ntp_stats_t;

void ntp_server_start(void);
void ntp_get_stats(ntp_stats_t *st);

#ifdef __cplusplus
}
#endif

#endif /* NTP_H */
