/*
 * 时基与驯服（discipline）
 *  - PPS 由 GPIO 硬件中断捕获，时标取自 esp_timer 的 64 位微秒计数（无 32 位回绕）
 *  - NMEA 只用于解析“绝对秒”，不参与相位测量
 *  - 锁定后每个 PPS 上升沿即是一个相位采样点，PLL(积分) + FLL(频差) 联合补偿晶振温漂
 *  - 失锁后进入 holdover，继续按最后估计的频率自由运行
 */
#ifndef DISCIPLINE_H
#define DISCIPLINE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NTP_EPOCH_OFFSET 2208988800ULL   /* 1900-01-01 <-> 1970-01-01 */

typedef struct {
    bool      anchored;      /* 已知绝对 UTC（至少对齐过一次）          */
    bool      pps_ok;        /* CFG_PPS_TIMEOUT_MS 内见过 PPS           */
    bool      nmea_ok;       /* CFG_FIX_TIMEOUT_MS 内见过有效 RMC       */
    bool      fix_ok;        /* CFG_FIX_TIMEOUT_MS 内见过合格定位(GGA)。
                              * 这是"原始探测结果"；只有确实收到过 GGA 时才
                              * 参与判据（见下面的 locked）                */
    bool      locked;        /* anchored && pps_ok && nmea_ok && fix_gate 同时成立。
                              * fix_gate = 收到过 GGA ? fix_ok : true
                              * （模块只输出 RMC 时不把 GGA 质量当硬条件）  */
    uint32_t  holdover_ms;   /* 距上一次有效 PPS 的时间；从未见过 PPS 时为 0 */
    bool      holdover_valid;/* false = 从未收到过 PPS，holdover_ms 无意义   */
    int64_t   offset_us;     /* 最近一次 PPS 相位误差（真值 - 本地估计）*/
    int64_t   jitter_us;     /* |offset| 的滑动平均                     */
    int32_t   ppb;           /* 当前频率修正（parts per billion）       */
    uint32_t  pps_total;     /* 累计 PPS 数                             */
    uint32_t  pps_missed;    /* 累计漏掉的 PPS（间隔推算）              */
    uint32_t  stratum;       /* RFC 5905 stratum                        */
    uint8_t   li;            /* RFC 5905 leap indicator                 */
    uint32_t  root_disp_us;  /* Root Dispersion（微秒）                 */
    /* 伺服事件计数（排障用：一直涨说明对时链路有问题） */
    uint32_t  step_count;    /* 相位误差过大直接阶跃                    */
    uint32_t  resync_count;  /* 用 RMC 重新对齐（残差超阈值）           */
    uint32_t  slip_count;    /* 检测到整秒滑移并纠正                    */
    uint32_t  mismatch_count;/* RMC 与 PPS 对不上（丢弃）               */
    uint32_t  late_count;    /* 语句传输跨秒、回退用上一条边沿（9600 下正常）*/
    uint32_t  lag_ms;        /* NMEA 语句起始相对其 PPS 边沿的滞后（ms） */
} disc_status_t;

/* 硬件微秒计时器。ISR 安全（底层在 IRAM）。 */
uint64_t tb_now_us(void);

/* 初始化时基与伺服任务。必须在 ethernet / gps 之前调用。 */
void discipline_init(void);

/* ---- PPS GPIO 中断里调用（IRAM） ---- */
void discipline_on_pps(uint64_t tb_us);

/* ---- GPS 任务里调用 ----
 * unix_sec    : 本条 RMC 对应的 UTC 整秒（Unix 秒）
 * est_start_tb: 该条 NMEA 语句"开始发送"时刻的估计（用于消除串口传输延迟）*/
void discipline_on_nmea_second(uint32_t unix_sec, uint64_t est_start_tb);

/* ---- GPS 任务里调用：刷新定位质量（GGA） ---- */
void discipline_on_fix(uint8_t fix_quality, uint8_t sats);

/* 取当前 UTC（整数运算，保留 µs）。未 anchored 返回 false。 */
bool discipline_get_utc(uint64_t tb_us, uint32_t *sec, uint32_t *frac32);

uint32_t discipline_ref_sec(void);      /* 参考时间戳（NTP 秒） */
void     discipline_get_status(disc_status_t *st);

#ifdef __cplusplus
}
#endif

#endif /* DISCIPLINE_H */
