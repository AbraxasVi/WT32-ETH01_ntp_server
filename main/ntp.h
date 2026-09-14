/*
 * NTPv4 服务器（RFC 5905）
 *  - 独立高优先级任务 + BSD socket，SO_RCVBUF 放大，令牌桶限速，
 *    突发流量不会阻塞 lwIP tcpip 线程
 *  - 入站时戳优先取以太网驱动层的帧到达时刻，退化为 socket 唤醒时刻
 *  - Stratum / Root Dispersion / Precision / LI 全部按锁定状态动态填写
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
    uint32_t bad;           /* 非法/不支持的报文 */
    uint32_t dropped;       /* 被令牌桶丢弃 */
    uint32_t rx_ts_used;    /* 使用了驱动层时戳的次数 */
    uint32_t rx_ts_miss;    /* 未取到驱动层时戳、退化处理的次数 */
    uint32_t send_fail;     /* 发送失败 */
} ntp_stats_t;

void ntp_server_start(void);
void ntp_get_stats(ntp_stats_t *st);

#ifdef __cplusplus
}
#endif

#endif /* NTP_H */
