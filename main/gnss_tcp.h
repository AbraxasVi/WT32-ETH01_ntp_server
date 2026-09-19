/*
 * GNSS 原始报文 TCP 转发（远程"串口调试助手"）
 *  - 把 GPS 串口上收到的原始字节原样推送到一个 TCP 端口（默认 8880）
 *  - 转发内容与串口完全一致：NMEA 文本行、模块的 UBX 二进制应答、
 *    校验失败的坏帧、波特率探测期间的乱码
 *  - 客户端连上时先发 2~3 行以 '#' 开头的提示（波特率 / fix / UTC），
 *    便于一眼确认连接是否成功
 *  - 对授时路径零影响：写侧只在 GPS 任务跑完解析后做一次无锁 memcpy，
 *    socket 收发全在 CORE_NET 的独立任务里（详见 gnss_tcp.c）
 *
 * CFG_GNSS_TCP_ENABLE=0 时本模块整体退化为空操作，调用点无需条件编译。
 */
#ifndef GNSS_TCP_H
#define GNSS_TCP_H

#include <stdint.h>
#include <stddef.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  clients;        /* 当前已连接的客户端数                 */
    uint32_t clients_total;  /* 累计接受过的连接数                   */
    uint32_t rejected;       /* 因连接数达上限被拒绝的连接数         */
    uint32_t sent_bytes;     /* 已成功发往客户端的字节数（含提示行） */
    uint32_t drop_bytes;     /* 环形缓冲放不下而丢弃的字节数         */
} gnss_tcp_stats_t;

#if CFG_GNSS_TCP_ENABLE

/* 启动转发服务：创建监听 socket 与后台任务（固定 CORE_NET） */
void gnss_tcp_start(void);

/* 投递刚读到的串口原始字节。**只能在 GPS 任务（CORE_TIME）上下文调用**：
 * 内部只有一次无锁 memcpy，不取信号量、不阻塞、不下发任何网络操作。 */
void gnss_tcp_feed(const uint8_t *data, size_t len);

#else  /* !CFG_GNSS_TCP_ENABLE */

static inline void gnss_tcp_start(void)
{
}

static inline void gnss_tcp_feed(const uint8_t *data, size_t len)
{
    (void)data;
    (void)len;
}

#endif /* CFG_GNSS_TCP_ENABLE */

/* 取统计快照（面板 / 串口诊断用；关闭时返回全零） */
void gnss_tcp_get_stats(gnss_tcp_stats_t *st);

#ifdef __cplusplus
}
#endif

#endif /* GNSS_TCP_H */
