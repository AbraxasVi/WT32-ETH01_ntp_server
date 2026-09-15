/*
 * 以太网接口层（WT32-ETH01 / LAN8720 RMII）
 *  - IP 地址由 DHCP 获取（不支持静态 IP，见 config.h 中的说明）
 *  - 通过自定义 input path 抓取 NTP 入站帧的到达时刻（比 lwIP 回调更接近线路）
 *  - 断链统计与强制重建
 */
#ifndef ETH_IF_H
#define ETH_IF_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 由 app_main 调用 */
void eth_if_init(void);

/* NTP 侧调用：弹出一条"入站帧到达时戳"（FIFO），没有则返回 false。
 * 调用约定：每收到一个包都必须调用一次（不论该包最终是否被应答），以保证与
 * 驱动侧"一帧一入队"严格一一对应——否则被丢弃的包会在缓冲里留下时戳，
 * 后面的请求就会取到属于别人的时戳。
 * 队列里过久（>200ms）的陈旧时戳会被丢弃并继续取，直到取到有效项或队列空。 */
bool eth_if_take_rx_ts(uint64_t *out_ts, uint64_t now);

bool     eth_if_link_up(void);
void     eth_if_get_ip(char *buf, size_t len);
/* 链路已断开的秒数；返回 0 表示"链路正常"或"尚未记录断开起点"（上电即无
 * 链路时，起点由 eth_if_periodic() 惰性建立，见那里的说明）。 */
uint32_t eth_if_link_down_seconds(void);
uint32_t eth_if_link_up_count(void);   /* 累计 link UP 次数，用于观察是否仍在翻动 */

/* 周期调用：断链超时后强制重建 */
void eth_if_periodic(void);

#ifdef __cplusplus
}
#endif

#endif /* ETH_IF_H */
