/*
 * 以太网接口层（WT32-ETH01 / LAN8720 RMII）
 *  - 静态 IP
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

/* NTP 侧调用：取出一个"入站帧到达时戳"；没有则返回 false */
bool eth_if_take_rx_ts(uint64_t *out_ts, uint64_t now);

bool     eth_if_link_up(void);
void     eth_if_get_ip(char *buf, size_t len);
uint32_t eth_if_link_down_seconds(void);
uint32_t eth_if_link_up_count(void);   /* 累计 link UP 次数，用于观察是否仍在翻动 */

/* 周期调用：断链超时后强制重建 */
void eth_if_periodic(void);

#ifdef __cplusplus
}
#endif

#endif /* ETH_IF_H */
