/*
 * 时基 / PPS 驯服实现
 *
 * 设计要点
 * 1) 时标：esp_timer_get_time() 返回 64 位微秒，不存在 32 位计数器回绕问题；
 *    所有差值运算统一用 uint64/int64，配合 TB_DELTA 宏保证回绕安全。
 * 2) 相位检测：PPS 上升沿必须落在整数 UTC 秒上。锁定后每个 PPS 都是一次
 *    相位采样（offset = 最近整秒 - 本地估计），NMEA 仅负责解析"是第几秒"，
 *    不再参与相位测量，因此串口传输延迟不会污染精度。
 * 3) 伺服：PLL（对残余相位积分到频率修正）+ FLL（用相邻相位差估频差）
 *    联合工作，可跟随晶振温漂；失锁后按最后的频率估计进入 holdover。
 * 4) NMEA 跨秒：RMC 帧在低波特率下可能跨到下一个 PPS 之后才被解析完，
 *    这里用"语句起始时刻估计 + 逐条 PPS 边沿比对"挑选正确的秒首，
 *    并用当前钟面的预测值做 ±1 秒交叉校验。
 */
#include "discipline.h"
#include "config.h"

#include <string.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

static const char *TAG = "disc";

#define PPS_RING      8                       /* 必须是 2 的幂 */
#define PPS_RING_MASK (PPS_RING - 1)
_Static_assert((PPS_RING & PPS_RING_MASK) == 0, "PPS_RING 必须是 2 的幂");

/* 回绕安全的无符号差值（两个 uint64 时标之间） */
#define TB_DELTA(now, before) ((uint64_t)((now) - (before)))

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

/* ---- 受 s_lock 保护的状态 ---- */
static volatile uint64_t s_pps_ring[PPS_RING];
static volatile uint32_t s_pps_head;        /* 单调递增，最新边沿 = head-1 */
static volatile uint64_t s_last_pps_tb;
static volatile uint32_t s_pps_total;
static volatile uint32_t s_pps_missed;

static volatile bool     s_anchored;        /* 已知绝对 UTC */
static volatile uint64_t s_anchor_tb;
static volatile int64_t  s_anchor_utc_us;
static volatile int32_t  s_ppb;
static volatile uint64_t s_lock_since_tb;   /* 最近一次建立锚点的时刻（暖机计时） */

static volatile int64_t  s_prev_off;        /* 上一次的残余相位误差 */
static volatile bool     s_have_prev;
static volatile int64_t  s_offset_us;
static volatile int64_t  s_jitter_q3;       /* |offset| 的 EMA，放大 8 倍 */

static volatile uint64_t s_nmea_time_tb;    /* 最近一条有效 RMC 秒 */
static volatile uint64_t s_fix_tb;          /* 最近一次合格定位（GGA） */
static volatile uint32_t s_ref_sec;         /* 参考时间戳（NTP 秒） */

/* 供任务侧打印的计数（ISR 只自增） */
static volatile uint32_t s_cnt_step;
static volatile uint32_t s_cnt_resync;
static volatile uint32_t s_cnt_slip;
static volatile uint32_t s_cnt_mismatch;
static volatile uint32_t s_cnt_late;
static volatile uint32_t s_lag_us;      /* 最近一次配对的 NMEA 滞后（µs） */

/* ---- 任务侧派生状态 ---- */
/* 注意：s_seen_gga 由 GPS 任务在 discipline_on_fix() 里写、被其它任务经
 * refresh_flags() 读，因此与 s_fix_tb 一样需要 volatile（跨任务可见性）。 */
static bool     s_pps_ok;
static bool     s_nmea_ok;
static bool     s_fix_ok;
static bool     s_fix_gate;       /* 是否把 GGA 定位质量当作硬条件（见 refresh_flags） */
static volatile bool s_seen_gga;  /* 是否收到过 GGA（有些模块只输出 RMC） */
static bool     s_locked;
static uint32_t s_holdover_ms;
static bool     s_hold_valid;     /* 是否收到过 PPS */
static bool     s_was_locked;

uint64_t IRAM_ATTR tb_now_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

static inline int64_t i64abs(int64_t v)
{
    return (v < 0) ? -v : v;
}

/* ------------------------------------------------------------------ */
/* 持锁条件下：按当前锚点 + 频率修正推算 tb 时刻的 UTC（µs）           */
/* ------------------------------------------------------------------ */
static int64_t IRAM_ATTR utc_us_at_locked(uint64_t tb)
{
    int64_t dt   = (int64_t)TB_DELTA(tb, s_anchor_tb);
    int64_t corr = (int64_t)((dt * (int64_t)s_ppb) / 1000000000LL);
    return (int64_t)s_anchor_utc_us + dt + corr;
}

/* ============================ PPS 中断 ============================ */
void IRAM_ATTR discipline_on_pps(uint64_t tb)
{
    portENTER_CRITICAL_ISR(&s_lock);

    /* --- 间隔统计：识别漏掉的 PPS --- */
    if (s_last_pps_tb != 0) {
        uint64_t dt = TB_DELTA(tb, s_last_pps_tb);
        if (dt > 1500000ULL && dt < 60000000ULL) {
            s_pps_missed += (uint32_t)(dt / 1000000ULL) - 1U;
        }
    }
    s_last_pps_tb = tb;
    s_pps_total++;
    s_pps_ring[s_pps_head & PPS_RING_MASK] = tb;
    s_pps_head++;

    /* --- 相位检测 + 伺服 --- */
    if (s_anchored) {
        int64_t pred    = utc_us_at_locked(tb);
        int64_t nearest = ((pred + 500000LL) / 1000000LL) * 1000000LL;
        int64_t off     = nearest - pred;      /* >0 表示本地慢了 */

        s_offset_us = off;

        if (off > CFG_PLL_STEP_US || off < -CFG_PLL_STEP_US) {
            /* 偏差过大（首次对齐、长时间失锁恢复）：直接阶跃 */
            s_anchor_tb     = tb;
            s_anchor_utc_us = nearest;
            s_have_prev     = false;
            s_cnt_step++;
        } else {
            bool     warm      = TB_DELTA(tb, s_lock_since_tb) < ((uint64_t)CFG_PLL_WARMUP_SEC * 1000000ULL);
            int64_t  phase_div = warm ? CFG_PLL_WARMUP_PHASE_DIV : CFG_PLL_PHASE_DIV;
            int64_t  fll_pct   = warm ? CFG_PLL_WARMUP_FLL_PCT   : CFG_FLL_GAIN_PCT;

            /* FLL：相邻两次相位差 / 间隔 => 频差(ppb) */
            if (s_have_prev) {
                int64_t dt_us = (int64_t)TB_DELTA(tb, s_anchor_tb);
                if (dt_us > 0) {
                    int64_t d_ns  = (off - s_prev_off) * 1000LL;          /* 相位差 ns */
                    int64_t fppb  = (d_ns * 1000000LL) / dt_us;           /* ns/s == ppb */
                    s_ppb += (int32_t)((fppb * fll_pct) / 100);
                }
            }
            /* PLL：把残余相位误差慢慢积分进频率修正 */
            s_ppb += (int32_t)((off * CFG_PLL_FREQ_NUM) / 1000);
            if (s_ppb > CFG_PPB_LIMIT) {
                s_ppb = CFG_PPB_LIMIT;
            } else if (s_ppb < -CFG_PPB_LIMIT) {
                s_ppb = -CFG_PPB_LIMIT;
            }

            /* 相位项：每个 PPS 修正 1/phase_div */
            int64_t corr_us = off / phase_div;

            /* 【关键】重新基准化必须把流逝的 dt 一起搬进
             * anchor_utc_us。u(t) = anchor_utc_us + (t - anchor_tb) + 频偏修正，
             * 若只把 anchor_tb 推进到 tb 而不给 anchor_utc_us 加上 dt，
             * 等于每来一个 PPS 就把钟面拉回上一次的读数：
             *   - 串口诊断里的 UTC 会完全冻结；
             *   - 因为 pred 每次都被重置，off 恒为 0，伺服"看起来很稳"；
             *   - RMC 与 PPS 的差值被固定成初始误差，永远匹配不上。
             * 正确写法：新锚点的 UTC = 本次推算值 pred + 相位修正。 */
            s_anchor_utc_us = pred + corr_us;
            s_anchor_tb     = tb;
            s_prev_off      = off - corr_us;
            s_have_prev     = true;
        }

        /* 抖动滑动平均（|offset|，放大 8 倍保存） */
        int64_t a = off < 0 ? -off : off;
        s_jitter_q3 = s_jitter_q3 - (s_jitter_q3 / 8) + a * 8;
    }

    portEXIT_CRITICAL_ISR(&s_lock);
}

/* 第 k 新的 PPS 边沿（k = 0 为最新）。调用方需持锁且 k < pps_avail() */
static inline uint64_t pps_edge_at(int k)
{
    return s_pps_ring[(s_pps_head - 1u - (uint32_t)k) & PPS_RING_MASK];
}

static inline int pps_avail(void)
{
    return (s_pps_head < PPS_RING) ? (int)s_pps_head : PPS_RING;
}

/* ------------------------------------------------------------------
 * 找出"本条语句所属的那一次 PPS"的候选序号。
 *
 * 物理事实：模块在 PPS_N 之后才开始发送第 N 秒的 NMEA 突发。因此本条语句
 * 对应的边沿必然满足 "边沿时刻 <= 语句起始时刻 + 容差"，而且只可能是
 *   (a) 满足该条件的最新一条边沿（突发 < 1 s 的常规情况），或
 *   (b) 它的前一条（9600 波特 + GSV 全开时突发超过 1 s，尾部语句会拖到
 *       PPS_{N+1} 之后才发出来，此时 (a) 是 PPS_{N+1}，正确边沿是前一条）。
 *
 * 早先是"在最近 8 条边沿里取 |RMC秒 - 边沿推算秒| 最小的 那条"。由于 PPS 恰好
 *  1 s 一条，一旦时钟整体偏了 k 秒，第 k 条旧边沿就会
 * 给出 0 残差而被选中 —— 于是误差被判成"完全正常"，从此永远不再纠正。
 * 所以候选必须限制在 (a)(b) 两条，多一秒就必须是失配并强制重对齐。
 * ------------------------------------------------------------------ */
static int pick_candidate_index(uint64_t est_start_tb, int avail)
{
    uint64_t lim = est_start_tb + (uint64_t)CFG_NMEA_LAG_TOL_US;
    int      k   = 0;

    while (k < avail - 1 && pps_edge_at(k) > lim) {
        k++;
    }
    return k;
}

/* 把锚点重新对齐到 (边沿 p, 该边沿对应的 UTC 整秒 target) */
static void reanchor_locked(uint64_t p, int64_t target)
{
    s_anchor_tb     = p;
    s_anchor_utc_us = target;
    s_have_prev     = false;
}

/* 限流日志：同一条告警最多每 period_ms 打一次，避免串口被刷爆
 * （串口日志是同步输出，刷屏会直接拖慢 HTTP 与 NTP 响应）。 */
#define LOG_THROTTLE(period_ms, lvl, fmt, ...)          \
    do {                                                \
        static uint64_t _lt = 0;                        \
        uint64_t _n = tb_now_us();                       \
        if (_n - _lt > (uint64_t)(period_ms) * 1000ULL) { \
            _lt = _n;                                   \
            ESP_LOG##lvl(TAG, fmt, ##__VA_ARGS__);       \
        }                                               \
    } while (0)

/* ====================== NMEA：解析绝对秒 ========================== */
void discipline_on_nmea_second(uint32_t unix_sec, uint64_t est_start_tb)
{
    int64_t  target = (int64_t)unix_sec * 1000000LL;
    int      avail;
    int      k;
    uint64_t p0, best_p;
    int64_t  d0, best_d;
    bool     late = false;

    portENTER_CRITICAL(&s_lock);

    avail = pps_avail();
    if (avail == 0) {
        portEXIT_CRITICAL(&s_lock);
        return;
    }
    k  = pick_candidate_index(est_start_tb, avail);
    p0 = pps_edge_at(k);

    if (!s_anchored) {
        /* 首次对齐：直接用物理上正确的那条边沿。万一模块时序特殊导致差 1 秒，
         * 下面的"整秒滑移"会在随后 1~2 秒内自动修回来（候选只有两条，
         * 误差最多 ±1 秒）。 */
        reanchor_locked(p0, target);
        s_ppb           = 0;
        s_anchored      = true;
        s_lock_since_tb = p0;
        s_lag_us        = 0;
    } else {
        d0     = target - utc_us_at_locked(p0);
        best_d = d0;
        best_p = p0;

        if (k + 1 < avail) {
            uint64_t p1 = pps_edge_at(k + 1);
            int64_t  d1 = target - utc_us_at_locked(p1);
            if (i64abs(d1) < i64abs(d0)) {
                best_d = d1;
                best_p = p1;
                late   = true;              /* 突发超过 1 s，语句拖到了下一秒 */
            }
        }

        /* 语句起始相对所配边沿的滞后。必须用有符号算：理论值 >= 0，但候选边沿
         * 的选取允许语句起始落在边沿之前最多 CFG_NMEA_LAG_TOL_US（400ms），
         * 无符号减法一旦下溢就会变成 4e9 级别的"滞后"，既打印出荒谬数值，
         * 又会误触发下面的"滞后超上限 -> 强制重对齐"，把本来正确的锚点搬走。 */
        int64_t lag_us_s = (int64_t)(est_start_tb - best_p);
        s_lag_us = (lag_us_s > 0) ? (uint32_t)(uint64_t)lag_us_s : 0u;

        if (i64abs(best_d) <= (int64_t)CFG_NMEA_MATCH_US &&
            lag_us_s <= (int64_t)CFG_NMEA_MAX_LAG_US) {
            if (late) {
                /* 9600 波特 + GSV 全开时这是常态，不是故障：只记数，不刷日志 */
                s_cnt_late++;
            }
            if (i64abs(best_d) > (int64_t)CFG_NMEA_SNAP_US) {
                /* 残差偏大（>50 ms）：直接对齐，慢慢牵引没有意义 */
                reanchor_locked(best_p, target);
                s_cnt_resync++;
            }
        } else {
            /* 完全对不上。先判断是不是"整秒滑移"：残差去掉整数秒后，
             * 小数部分仍贴近 0（±100 ms）就说明只是差了整整 N 秒
             * （初次对齐选错边沿 / 长时间失锁后自由漂移），
             * 直接按 RMC 重新对齐，否则会卡在
             * "匹配不上 -> 不纠正 -> 继续匹配不上"的死循环里。
             * 注意用物理边沿 p0 而不是 best_p 作为新锚点的时基，
             * 这样即使 best_p 是滞后一秒的旧边沿也不会把误差固化。 */
            int64_t frac = best_d % 1000000LL;
            bool    is_slip;
            if (frac < 0) {
                frac += 1000000LL;
            }
            is_slip = (frac <= (int64_t)CFG_NMEA_SLIP_TOL_US ||
                       frac >= 1000000LL - (int64_t)CFG_NMEA_SLIP_TOL_US);

            if (is_slip) {
                int64_t slip = best_d / 1000000LL;
                if (best_d < 0 && (best_d % 1000000LL) != 0) {
                    slip -= 1;                  /* C 整除向零取整，修正成向下取整 */
                }
                /* 幅度合理性检查：滑移是给"我们自己选错边沿 / 失锁后晶振漂了整数秒"
                 * 用的，不该无条件接受任意整数秒。差得太多说明时间源本身不可信
                 * （实测：模块丢定位后 RTC 漂了 3 整秒，钟面就被拉走了 3 秒），
                 * 此时宁可丢弃本次对齐，也不能把钟面搬过去。 */
                if (slip > (int64_t)CFG_NMEA_SLIP_MAX_SEC ||
                    slip < -(int64_t)CFG_NMEA_SLIP_MAX_SEC) {
                    s_cnt_mismatch++;
                    portEXIT_CRITICAL(&s_lock);
                    LOG_THROTTLE(10000, W,
                                 "NMEA 与本地钟面相差 %lld 整秒，超出滑移上限 %d s，"
                                 "判定时间源不可信，丢弃本次对齐",
                                 (long long)slip, CFG_NMEA_SLIP_MAX_SEC);
                    return;
                }
                reanchor_locked(p0, target);
                s_cnt_slip++;
                s_nmea_time_tb = tb_now_us();
                s_ref_sec      = (uint32_t)(unix_sec + NTP_EPOCH_OFFSET);
                portEXIT_CRITICAL(&s_lock);
                LOG_THROTTLE(10000, W, "PPS 与 NMEA 相差 %lld 整秒（滑移），已按 NMEA 重新对齐",
                             (long long)slip);
                return;
            }

            s_cnt_mismatch++;
            if (lag_us_s > (int64_t)CFG_NMEA_MAX_LAG_US) {
                /* 配对到明显过期的边沿：强制重置 */
                reanchor_locked(p0, target);
                s_cnt_resync++;
                portEXIT_CRITICAL(&s_lock);
                LOG_THROTTLE(10000, W,
                             "NMEA 滞后 %lu ms 超上限，已强制按当前 PPS 边沿重对齐",
                             (unsigned long)(s_lag_us / 1000UL));
                return;
            }
            portEXIT_CRITICAL(&s_lock);
            LOG_THROTTLE(10000, W,
                         "RMC 秒 %u 与最近 PPS 均不匹配（残差 %lld us），丢弃本次对齐",
                         (unsigned)unix_sec, (long long)best_d);
            return;
        }
    }

    s_nmea_time_tb = tb_now_us();
    s_ref_sec      = (uint32_t)(unix_sec + NTP_EPOCH_OFFSET);

    portEXIT_CRITICAL(&s_lock);

    if (late) {
        ESP_LOGD(TAG, "NMEA 突发跨秒，本条用了前一条 PPS 边沿 (UTC %u, 滞后 %lu ms)",
                 (unsigned)unix_sec, (unsigned long)(s_lag_us / 1000UL));
    }
}

/* ====================== 定位质量刷新 =============================== */
void discipline_on_fix(uint8_t fix_quality, uint8_t sats)
{
    uint64_t now = tb_now_us();
    portENTER_CRITICAL(&s_lock);
    /* 见过 GGA 就置位：用于区分"模块根本不输出 GGA"和"输出但定位不合格"，
     * 前者不该把设备永久卡在 stratum 16（见 refresh_flags）。 */
    s_seen_gga = true;
    /* 与 gps.c 的 fix_valid 共用同一组宏：推算(6)/模拟(8)定位不算合格来源 */
    if (fix_quality >= CFG_GPS_FIXQ_MIN && fix_quality <= CFG_GPS_FIXQ_MAX &&
        sats >= CFG_GPS_MIN_SATS) {
        s_fix_tb = now;
    }
    portEXIT_CRITICAL(&s_lock);
}

/* =========================== 取当前 UTC ============================ */
bool discipline_get_utc(uint64_t tb_us, uint32_t *sec, uint32_t *frac32)
{
    int64_t u;
    bool    ok;

    portENTER_CRITICAL(&s_lock);
    ok = s_anchored;
    u  = ok ? utc_us_at_locked(tb_us) : 0;
    portEXIT_CRITICAL(&s_lock);

    if (!ok || u < 0) {
        return false;
    }
    *sec    = (uint32_t)(u / 1000000LL);
    *frac32 = (uint32_t)(((u % 1000000LL) * 4294967296LL) / 1000000LL);
    return true;
}

/* ============================ 状态查询 ============================= */
static void refresh_flags(uint64_t now)
{
    s_pps_ok  = (s_last_pps_tb != 0) &&
                (TB_DELTA(now, s_last_pps_tb) < (uint64_t)CFG_PPS_TIMEOUT_MS * 1000ULL);
    s_nmea_ok = (s_nmea_time_tb != 0) &&
                (TB_DELTA(now, s_nmea_time_tb) < (uint64_t)CFG_FIX_TIMEOUT_MS * 1000ULL);
    s_fix_ok  = (s_fix_tb != 0) &&
                (TB_DELTA(now, s_fix_tb) < (uint64_t)CFG_FIX_TIMEOUT_MS * 1000ULL);
    /* 有些模块被裁成只输出 RMC（GGA 关闭）——那种情况下 s_fix_ok 永远是 false，
     * 把它当硬条件会让设备永远停在 stratum 16。所以只有确实见过 GGA 时，
     * 才把"定位质量"当作判据。 */
    s_fix_gate = s_seen_gga ? s_fix_ok : true;
    s_locked   = s_anchored && s_pps_ok && s_nmea_ok && s_fix_gate;

    /* 从未收到过 PPS 时，holdover 没有意义：以前这里填 0xFFFFFFFF
     * （显示为 4294967295ms ≈ 49 天），既不是真值也容易误判成"失锁很久"，
     * 改成 0 + holdover_valid=false。 */
    s_hold_valid  = (s_last_pps_tb != 0);
    s_holdover_ms = s_hold_valid
                        ? (uint32_t)(TB_DELTA(now, s_last_pps_tb) / 1000ULL)
                        : 0u;
}

void discipline_get_status(disc_status_t *st)
{
    uint64_t now = tb_now_us();

    portENTER_CRITICAL(&s_lock);
    st->anchored     = s_anchored;
    st->ppb          = s_ppb;
    st->offset_us    = s_offset_us;
    st->jitter_us    = s_jitter_q3 / 8;
    st->pps_total    = s_pps_total;
    st->pps_missed   = s_pps_missed;
    st->step_count     = s_cnt_step;
    st->resync_count   = s_cnt_resync;
    st->slip_count     = s_cnt_slip;
    st->mismatch_count = s_cnt_mismatch;
    st->late_count     = s_cnt_late;
    st->lag_ms         = s_lag_us / 1000U;
    portEXIT_CRITICAL(&s_lock);

    refresh_flags(now);

    st->pps_ok       = s_pps_ok;
    st->nmea_ok      = s_nmea_ok;
    st->fix_ok       = s_fix_ok;
    st->locked       = s_locked;
    st->holdover_ms  = s_holdover_ms;
    st->holdover_valid = s_hold_valid;

    /* 没有绝对时间来源（没对齐过 / 长时间没拿到有效 RMC / 定位不合格）就不能
     * 自称 stratum 1。判据与 locked 保持一致，避免出现"面板显示 UNLOCKED、
     * 却对外宣告 stratum=1 / LI=0"这种自相矛盾的状态。 */
    if (!st->anchored || !st->nmea_ok || !s_fix_gate) {
        st->stratum      = 16;
        st->li           = 3;                       /* 未同步 */
        st->root_disp_us = 10000000;                /* 10 s */
        return;
    }
    if (st->pps_ok && st->holdover_ms < ((uint32_t)CFG_HOLD_STRATUM2_S * 1000u)) {
        st->stratum = 1;                            /* GPS/PPS 主时钟 */
        st->li      = 0;
    } else if (st->holdover_ms < ((uint32_t)CFG_HOLD_UNSYNC_S * 1000u)) {
        st->stratum = 2;                            /* Holdover 守时中 */
        st->li      = 0;
    } else {
        st->stratum = 16;                           /* 失锁过久，宣告未同步 */
        st->li      = 3;
    }

    uint32_t disp = CFG_NTP_BASE_DISP_US + (uint32_t)(st->jitter_us * 2);
    if (st->holdover_valid) {
        disp += st->holdover_ms / 10u;              /* 允许 100 ppb 的守时漂移 */
    } else {
        disp = 10000000u;                           /* 从未对过钟，保守给 10 s */
    }
    if (disp > 10000000u) {
        disp = 10000000u;
    }
    st->root_disp_us = disp;
}

uint32_t discipline_ref_sec(void)
{
    return (uint32_t)s_ref_sec;
}

/* ============================ 后台任务 ============================= */
static void discipline_task(void *arg)
{
    (void)arg;
    if (esp_task_wdt_add(NULL) != ESP_OK) {
        ESP_LOGW(TAG, "TWDT 订阅失败（任务看门狗未启用？）");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(250));
        esp_task_wdt_reset();

        disc_status_t st;
        discipline_get_status(&st);

        if (st.locked != s_was_locked) {
            if (st.locked) {
                ESP_LOGI(TAG, "GPS/PPS 已锁定 (UTC 锚点偏移 %lld us)", (long long)st.offset_us);
            } else {
                ESP_LOGW(TAG, "失去锁定: pps_ok=%d nmea_ok=%d fix_ok=%d holdover=%u ms",
                         (int)st.pps_ok, (int)st.nmea_ok, (int)st.fix_ok,
                         (unsigned)st.holdover_ms);
            }
            s_was_locked = st.locked;
        }
    }
}

void discipline_init(void)
{
    memset((void *)s_pps_ring, 0, sizeof(s_pps_ring));
    s_pps_head      = 0;
    s_last_pps_tb   = 0;
    s_anchored      = false;
    s_anchor_tb     = 0;
    s_anchor_utc_us = 0;
    s_ppb           = 0;
    s_prev_off      = 0;
    s_have_prev     = false;
    s_jitter_q3     = 0;
    s_ref_sec       = 0;
    s_was_locked    = false;
    s_cnt_step      = 0;
    s_cnt_resync    = 0;
    s_cnt_slip      = 0;
    s_cnt_mismatch  = 0;
    s_cnt_late      = 0;
    s_lag_us        = 0;
    s_seen_gga      = false;
    s_fix_gate      = true;

    /* 时基链路固定在 CORE_TIME：PPS 中断就在同一个核上，
     * 不会被 lwIP / HTTP 的长临界区推迟 */
    if (xTaskCreatePinnedToCore(discipline_task, "disc", 3072, NULL, 20, NULL,
                                CFG_CORE_TIME) != pdPASS) {
        ESP_LOGE(TAG, "disc 任务创建失败（内存不足？）");
    }
    ESP_LOGI(TAG, "时基初始化完成 (esp_timer 64bit us, PPS->IO%d)", CFG_PPS_GPIO);
}
