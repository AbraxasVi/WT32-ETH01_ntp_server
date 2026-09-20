/*
 * GPS 接入层实现
 */
#include "gps.h"
#include "config.h"
#include "discipline.h"
#include "gnss_tcp.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "gps";

static const uint32_t s_baud_list[] = CFG_GPS_BAUD_LIST;
/* 候选表长度以数组本身为准（下面统一用 GPS_BAUD_N），CFG_GPS_BAUD_COUNT
 * 只作编译期一致性检查，避免"少配一个波特率就静默只试前几个"。 */
#define GPS_BAUD_N (sizeof(s_baud_list) / sizeof(s_baud_list[0]))
_Static_assert(GPS_BAUD_N == CFG_GPS_BAUD_COUNT,
               "CFG_GPS_BAUD_COUNT 与 CFG_GPS_BAUD_LIST 的元素个数不一致");

/* ---------------- 运行状态 ---------------- */
/* s_st / s_baud 由 GPS 任务独占更新；其他任务（HTTP 面板、串口诊断）读
 * s_st_pub 快照，避免看到"半新半旧"的字段组合。发布点见 publish_status()。 */
static gps_status_t s_st;
static portMUX_TYPE s_st_lock = portMUX_INITIALIZER_UNLOCKED;
static gps_status_t s_st_pub;
static uint32_t     s_baud = 0;
static bool         s_baud_locked = false;
static uint32_t     s_byte_us_q16;         /* 每字节耗时(µs) 的 Q16 定点 */
static bool         s_probe_mode = false;  /* 波特率探测中：只判定，不解析 */
static bool         s_detect_hit = false;
static bool         s_got_ack = false;
static uint8_t      s_ack_cls = 0, s_ack_id = 0;

/* ---- 多 Hz 模块的"整段筛选" ----
 * 5/10/25 Hz 的模块每秒会吐出 N 组报文，其中只有 NMEA 时间戳小数部分为 0
 * （.00）的那一组与 PPS 整秒对齐、可以用于授时。其余各组的 hhmmss.ss 会被
 * 解析成同一个整秒，混进来只会让 RMC↔PPS 配对反复重对齐（表现为始终锁不上），
 * 同时白白增加解析负担。
 * 这里以每条 RMC 作为"段界标"：RMC 的时间戳小数不为 0 时，丢弃它以及紧随其
 * 后的这一整段（GGA/GSA/GSV/GLL/VTG...），直到下一条 RMC。
 * 1 Hz 模块的时间戳恒为 .00，因此行为与以前完全一致。
 *
 * 界标语句 = 带 hhmmss.ss 时间戳的 RMC 或 GGA，任一到达都重设本段标志。
 * 两者都用是因为某些模块 RMC 输出稀疏（甚至完全不输出），只认 RMC 会让标志
 * 停在旧值上把后续整批语句误丢（表现为 nmea 不再增长）。标志还带"保活"：
 * 超过 1.5 s 没见到界标语句就取消筛选，绝不让它把数据流永久掐断。 */
static bool         s_burst_keep = true;
static uint64_t     s_burst_mark_us;    /* 最近一次界标语句的时刻（µs） */

/* ---- 绝对秒的"日期记忆"+ GGA 兜底 ----
 * RMC 是首选来源（既给秒又给日期）。但实测某些模块的整数秒 RMC 会整段不可用
 * （status='V'、字段残缺），此时死等 RMC 就会一直掉锁。所以记住最近一次成功
 * 解析的 RMC 日期：当 RMC 连续失效 > 3 s 时，改用"记住的日期 + GGA 的
 * hhmmss"继续供秒，让时基不至于丢锁。 */
static int          s_utc_y, s_utc_mo, s_utc_d;   /* 最近一次 RMC 给出的 UTC 日期 */
static uint64_t     s_rmc_ok_tb;                  /* 最近一次 RMC 对时的时刻（µs） */

/* 模块"抢救"机制是否启用：见 config.h 的 CFG_GPS_RST_MODE / CFG_GPS_REFIX_* 说明。 */
#if CFG_GPS_RST_MODE > 0 && CFG_GPS_REFIX_SEC > 0
#define GPS_RECOVER_ENABLED 1
static uint64_t     s_last_fix_tb;                /* 最近一次"定位合格"的时刻（µs） */
static uint32_t     s_restart_cnt;                /* 连续恢复尝试次数 */
#else
#define GPS_RECOVER_ENABLED 0
#endif

/* ====================== 日期 / 闰秒工具 ============================ */
static int is_leap(int y)
{
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

static const int s_dim[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

static uint32_t to_unix(int y, int mo, int d, int h, int mi, int s)
{
    int days = 0;
    for (int yy = 1970; yy < y; yy++) {
        days += is_leap(yy) ? 366 : 365;
    }
    for (int m = 1; m < mo; m++) {
        /* 纵深防御：调用方必须先用 nmea_datetime_is_valid() 校验。这里再挡
         * 一次，因为 m 一旦超过 12 就会读出 s_dim[] 之外的内存（月份字段是
         * "%2d"，最大 99），那是未定义行为，不是"算错时间"这么简单。 */
        if (m > 12) {
            break;
        }
        days += s_dim[m - 1];
        if (m == 2 && is_leap(y)) {
            days += 1;
        }
    }
    days += d - 1;
    return (uint32_t)days * 86400u + (uint32_t)h * 3600u + (uint32_t)mi * 60u + (uint32_t)s;
}

/* NMEA 的时间/日期字段必须落在合法范围内才允许参与对时。
 * 只判长度 + sscanf 成功是不够的："%2d" 能把 "99" 读进来，月份越界会让
 * to_unix() 越界索引 s_dim[]，时分秒越界则会凭空算出一个时间戳。这类报文
 * 要么来自模块异常，要么是串口误码，都不该被当作时间来源。 */
static bool nmea_datetime_is_valid(int y, int mo, int d, int h, int mi, int s)
{
    if (y < 1970 || y > 2099 || mo < 1 || mo > 12) {
        return false;
    }
    int dim = s_dim[mo - 1] + ((mo == 2 && is_leap(y)) ? 1 : 0);
    return d >= 1 && d <= dim &&
           h >= 0 && h <= 23 &&
           mi >= 0 && mi <= 59 &&
           s >= 0 && s <= 60;          /* 60 = 闰秒 */
}

/* GPS-UTC 闰秒跳变表（生效日期 -> 之后的 GPS-UTC 差值）。
 * 最后一次是 2017-01-01 起 18 秒。 */
static const struct { int y, m, d; uint8_t off; } s_leap_table[] = {
    { 1981, 7, 1, 1 },  { 1982, 7, 1, 2 },  { 1983, 7, 1, 3 },  { 1985, 7, 1, 4 },
    { 1988, 1, 1, 5 },  { 1990, 1, 1, 6 },  { 1991, 1, 1, 7 },  { 1992, 7, 1, 8 },
    { 1993, 7, 1, 9 },  { 1994, 7, 1, 10 }, { 1996, 1, 1, 11 }, { 1997, 7, 1, 12 },
    { 1999, 1, 1, 13 }, { 2006, 1, 1, 14 }, { 2009, 1, 1, 15 }, { 2012, 7, 1, 16 },
    { 2015, 7, 1, 17 }, { 2017, 1, 1, 18 },
};

static uint8_t gps_utc_offset(uint32_t unix_sec)
{
    uint8_t off = 0;
    for (size_t i = 0; i < sizeof(s_leap_table) / sizeof(s_leap_table[0]); i++) {
        uint32_t t = to_unix(s_leap_table[i].y, s_leap_table[i].m, s_leap_table[i].d, 0, 0, 0);
        if (unix_sec >= t) {
            off = s_leap_table[i].off;
        }
    }
    return off;
}

/* ====================== 原始报文留痕（排障用） =====================
 * 保存最近 GPS_RAW_LINES 条 GGA/GSA/ZDA 原始语句，HTTP 面板可直接查看。
 * 只存不解析，单写单读，不做加锁（读到半截也无害，刷新即可）。 */
static char    s_raw[GPS_RAW_LINES][GPS_RAW_LINE_LEN];
static uint8_t s_raw_head;      /* 下一条写入位置 */
static uint8_t s_raw_count;     /* 已填充条数 */

static bool raw_is_wanted(const char *line)
{
    /* $GNGGA / $GPGSA / $GNZDA ... 第 3 个字符开始就是句子类型 */
    return (strncmp(line + 3, "GGA", 3) == 0) ||
           (strncmp(line + 3, "GSA", 3) == 0) ||
           (strncmp(line + 3, "ZDA", 3) == 0);
}

/* raw 缓冲的一致性保护（seqlock）：写侧在首尾各自增一次序号，读者两次读取
 * 序号相同且为偶数才算拿到稳定快照。这样 GPS 任务写缓冲时不必抢锁，
 * 不会给 NMEA 时戳引入额外抖动。 */
static volatile uint32_t s_raw_seq;

static void raw_store(const char *line)
{
    char  *dst = s_raw[s_raw_head];
    size_t i   = 0;

    s_raw_seq++;
    while (line[i] && i < GPS_RAW_LINE_LEN - 1U) {
        char c = line[i];
        if (c == '\r' || c == '\n') {
            break;
        }
        /* 面板走 JSON，这里顺手把会破坏 JSON 的字符换掉 */
        if (c < 0x20 || c > 0x7E || c == '"' || c == '\\') {
            c = '?';
        }
        dst[i] = c;
        i++;
    }
    dst[i] = 0;

    s_raw_head = (uint8_t)((s_raw_head + 1u) % GPS_RAW_LINES);
    if (s_raw_count < GPS_RAW_LINES) {
        s_raw_count++;
    }
    s_raw_seq++;
}

int gps_get_last_nmea(char dst[][GPS_RAW_LINE_LEN], int max)
{
    int start, n;

    for (int attempt = 0; attempt < 2; attempt++) {
        uint32_t seq0 = s_raw_seq;
        if (seq0 & 1u) {
            continue;                       /* GPS 任务正在写入，重试 */
        }
        start = (int)s_raw_head - (int)s_raw_count;
        n     = 0;
        if (start < 0) {
            start += GPS_RAW_LINES;
        }
        for (int i = 0; i < (int)s_raw_count && n < max; i++) {
            int idx = (start + i) % GPS_RAW_LINES;
            snprintf(dst[n], GPS_RAW_LINE_LEN, "%s", s_raw[idx]);
            n++;
        }
        if (s_raw_seq == seq0) {
            return n;                       /* 期间没有写入：快照一致 */
        }
    }
    return 0;   /* 连续两次都撞上写入：本次不返回数据，调用方下次再取 */
}

/* ====================== UBX 发送 / 接收 ============================ */
static void ubx_send(uint8_t cls, uint8_t id, const uint8_t *pl, uint16_t len)
{
    uint8_t buf[128];
    if ((size_t)len + 8 > sizeof(buf)) {
        return;
    }
    buf[0] = 0xB5;
    buf[1] = 0x62;
    buf[2] = cls;
    buf[3] = id;
    buf[4] = (uint8_t)(len & 0xFF);
    buf[5] = (uint8_t)(len >> 8);
    if (len) {
        memcpy(&buf[6], pl, len);
    }
    uint8_t a = 0, b = 0;
    for (int i = 2; i < 6 + (int)len; i++) {
        a = (uint8_t)(a + buf[i]);
        b = (uint8_t)(b + a);
    }
    buf[6 + len] = a;
    buf[7 + len] = b;
    uart_write_bytes(CFG_GPS_UART, buf, (size_t)len + 8);
}

/* ====================== NMEA 解析 ================================== */
static char  s_fbuf[256];
static char *s_f[32];

static int split_fields(const char *line)
{
    size_t n = strlen(line);
    if (n >= sizeof(s_fbuf)) {
        n = sizeof(s_fbuf) - 1;
    }
    memcpy(s_fbuf, line, n);
    s_fbuf[n] = 0;
    char *star = strchr(s_fbuf, '*');
    if (star) {
        *star = 0;
    }
    char *cr = strpbrk(s_fbuf, "\r\n");
    if (cr) {
        *cr = 0;
    }
    /* 【必须保留空字段】NMEA 语句里空字段很常见（静止时 RMC 的速度是 0.000
     * 而航向为空：",0.000,,170926,"）。以前这里用 strtok()，它会把连续分隔符
     * 当成一个并跳过空字段，于是其后所有字段索引整体前移 —— 结果把 date 当成了
     * mode indicator，RMC 被误判为"时间/日期字段长度异常"而整段拒收（实测 5 Hz
     * 模块静止时稳定踩中，表现为 rmc 的 ok 不再增长、反复掉锁）。
     * 改成逐段扫描：每个逗号都切一刀，空段就是空字符串。 */
    int nf = 0;
    char *p = s_fbuf;
    while (nf < 32) {
        char *comma = strchr(p, ',');
        if (comma) {
            *comma = 0;
        }
        s_f[nf++] = p;
        if (!comma) {
            break;
        }
        p = comma + 1;
    }
    return nf;
}

static bool nmea_checksum_ok(const char *line)
{
    if (line[0] != '$') {
        return false;
    }
    const char *star = strchr(line, '*');
    if (!star || strlen(star) < 3) {
        return false;
    }
    uint8_t c = 0;
    for (const char *p = line + 1; p < star; p++) {
        c ^= (uint8_t)(*p);
    }
    char chk[3] = { star[1], star[2], 0 };
    return c == (uint8_t)strtoul(chk, NULL, 16);
}

/* RMC / GGA 的第 2 个字段都是 "hhmmss.sss"。返回 false 表示不是整数秒
 * （多 Hz 模块的 .20/.40/.60/.80 等）。不带小数部分时按整秒处理。
 * 直接在字符串上找逗号/小数点，不必为每个句段都建字段表。 */
static bool ts_is_whole_second(const char *line)
{
    const char *ts = strchr(line, ',');
    if (!ts) {
        return false;
    }
    ts++;
    const char *end = strchr(ts, ',');
    if (!end) {
        return false;
    }
    const char *dot = memchr(ts, '.', (size_t)(end - ts));
    if (!dot) {
        return true;
    }
    for (const char *q = dot + 1; q < end; q++) {
        if (*q != '0') {
            return false;
        }
    }
    return true;
}

/* 被丢弃的 RMC：计数 + 限流打印（每 10 s 最多一条），直接给出原因与原文。
 * 否则"RMC 明明在流里、却始终锁不上"只能靠猜，而原因不同处置方向完全不同。 */
static void rmc_reject(const char *why, const char *line)
{
    static uint64_t s_last_log_us;
    uint64_t now = tb_now_us();

    s_st.rmc_bad++;
    if (now - s_last_log_us > 10000000ULL) {
        s_last_log_us = now;
        ESP_LOGW(TAG, "RMC 被丢弃（%s）: %s", why, line);
    }
}

/* 解析 RMC 并交给时基做"整秒对齐"；成功时记住 UTC 日期供 GGA 兜底。 */
static void parse_rmc(const char *line, uint64_t sentence_start_tb)
{
    int nf = split_fields(line);
    if (nf < 10) {
        rmc_reject("字段数不足", line);
        return;
    }
    if (strcmp(s_f[2], "A") != 0) {
        rmc_reject("status 不是 A（模块报告定位无效）", line);
        return;
    }
    const char *t = s_f[1];
    const char *d = s_f[9];
    if (strlen(t) < 6 || strlen(d) != 6) {
        rmc_reject("时间/日期字段长度异常", line);
        return;
    }
    int hh, mm, ss, DD, MO, YY;
    if (sscanf(t, "%2d%2d%2d", &hh, &mm, &ss) != 3) {
        rmc_reject("时间字段解析失败", line);
        return;
    }
    if (sscanf(d, "%2d%2d%2d", &DD, &MO, &YY) != 3) {
        rmc_reject("日期字段解析失败", line);
        return;
    }
    int y = (YY < 70) ? (2000 + YY) : (1900 + YY);
    /* 校验必须做在 to_unix() 之前：它内部要用月份索引 s_dim[12]，
     * 而 "%2d" 能读进 0~99，月份越界就是数组越界读。 */
    if (!nmea_datetime_is_valid(y, MO, DD, hh, mm, ss)) {
        rmc_reject("时间/日期字段超出合法范围", line);
        return;
    }
    uint32_t unix_sec = to_unix(y, MO, DD, hh, mm, ss);
    discipline_on_nmea_second(unix_sec, sentence_start_tb);
    s_st.rmc_ok++;

    s_utc_y     = y;
    s_utc_mo    = MO;
    s_utc_d     = DD;
    s_rmc_ok_tb = tb_now_us();
}

static void parse_gga(const char *line)
{
    int nf = split_fields(line);
    if (nf < 8) {
        return;
    }
    int fq   = atoi(s_f[6]);
    int sats = atoi(s_f[7]);
    float hdop = (nf > 8) ? atof(s_f[8]) : 0.0f;

    s_st.fix_quality = (uint8_t)(fq < 0 ? 0 : (fq > 255 ? 255 : fq));
    s_st.sats_used   = (uint8_t)(sats < 0 ? 0 : (sats > 255 ? 255 : sats));
    s_st.hdop_x10    = (uint16_t)(hdop * 10.0f);
    /* 与 discipline.c 的判据共用同一组宏：fix quality 必须落在
     * [CFG_GPS_FIXQ_MIN, CFG_GPS_FIXQ_MAX]（默认 1~5，排除 6=推算、8=模拟）
     * 且参与解算卫星数达标。本字段供面板/日志显示，那份用于 stratum/LI 判定
     * —— 两处都引用宏，不会再出现"只改了一边"的偏差。 */
    s_st.fix_valid   = (fq >= CFG_GPS_FIXQ_MIN && fq <= CFG_GPS_FIXQ_MAX &&
                        s_st.sats_used >= CFG_GPS_MIN_SATS);

    discipline_on_fix(s_st.fix_quality, s_st.sats_used);

    /* ---- 绝对秒兜底：RMC 失效时改用 GGA 的时间戳 ----
     * 前提：①曾经从 RMC 拿到过日期（s_rmc_ok_tb != 0）；②**当前定位合格**
     * （模块自己都报 status='V'/fixq 不合格时，它的时间同样不可信，不该拿来对时）；
     * ③与当前钟面相差 ≤1 s —— 记住的日期可能已跨日，绝不允许用不可靠的日期去
     * "重建"时间，只让它维持已建立的时基。 */
    if (s_rmc_ok_tb != 0 && s_st.fix_valid &&
        (tb_now_us() - s_rmc_ok_tb) > 3000000ULL) {
        int hh, mm, ss;
        if (strlen(s_f[1]) >= 6 && sscanf(s_f[1], "%2d%2d%2d", &hh, &mm, &ss) == 3 &&
            nmea_datetime_is_valid(s_utc_y, s_utc_mo, s_utc_d, hh, mm, ss)) {
            uint32_t sec = to_unix(s_utc_y, s_utc_mo, s_utc_d, hh, mm, ss);
            uint32_t now_sec, now_frac;
            if (discipline_get_utc(tb_now_us(), &now_sec, &now_frac) &&
                (int64_t)sec - (int64_t)now_sec <= 1 &&
                (int64_t)now_sec - (int64_t)sec <= 1) {
                /* 这里用"当前时刻"近似语句起始：它只用来挑 PPS 边沿
                 * （容差 CFG_NMEA_LAG_TOL_US = 400 ms），不参与相位测量，
                 * 几十毫秒的近似足够，也省得为兜底路径再多传一个参数。 */
                discipline_on_nmea_second(sec, tb_now_us());
                s_st.gga_ts_used++;
            }
        }
    }
}

/* ---- 可见卫星数：按星座汇总 ----
 * $GPGSV / $BDGSV（或 $GBGSV）/ $GAGSV / $GLGSV 各自上报的是"该星座的可见卫星
 * 数"，直接覆盖只会留下最后一条 —— 多星座模块上就会出现 sats_used > sats_view
 * 这种自相矛盾的显示（实测：GPS 10 颗 + 北斗 7 颗，面板却只显示 7）。
 * 这里按 talker 分别记录，并且只累加"最近仍在输出"的星座：某星座消失后不再
 * 发 GSV，它的旧值不能继续计入总数。 */
static uint8_t  s_view_cnt[4];
static uint64_t s_view_tb[4];
#define GSV_KEEP_US  3000000ULL

static int gsv_talker_slot(const char *line)
{
    if (line[1] == 'B' && line[2] == 'D') return 0;   /* 北斗 $BDGSV */
    if (line[1] == 'G' && line[2] == 'B') return 0;   /* 北斗 $GBGSV */
    if (line[1] == 'G' && line[2] == 'P') return 1;   /* GPS */
    if (line[1] == 'G' && line[2] == 'A') return 2;   /* Galileo */
    if (line[1] == 'G' && line[2] == 'L') return 3;   /* GLONASS */
    return 1;                                         /* 合并输出的 $GNGSV 等归 GPS 槽 */
}

static void parse_gsv(const char *line)
{
    int nf = split_fields(line);
    if (nf < 4) {
        return;
    }
    int view = atoi(s_f[3]);
    if (view < 0) {
        view = 0;
    } else if (view > 255) {
        view = 255;
    }

    int k = gsv_talker_slot(line);
    s_view_cnt[k] = (uint8_t)view;
    s_view_tb[k]  = tb_now_us();

    uint64_t now = tb_now_us();
    uint32_t sum = 0;
    for (int i = 0; i < 4; i++) {
        if (now - s_view_tb[i] < GSV_KEEP_US) {
            sum += s_view_cnt[i];
        }
    }
    s_st.sats_view = (uint8_t)(sum > 255 ? 255 : sum);
}

/* ====================== 字节流状态机 =============================== */
typedef enum { PS_SYNC, PS_NMEA, PS_UBX } pstate_t;

static pstate_t  s_ps = PS_SYNC;
static uint8_t   s_line[256];
static uint16_t  s_pos;
static uint16_t  s_ubx_len;
static uint64_t  s_sentence_start_tb;

static void reset_parser(void)
{
    s_ps  = PS_SYNC;
    s_pos = 0;
}

static void handle_nmea(const char *line)
{
    if (!nmea_checksum_ok(line)) {
        s_st.bad_count++;      /* 校验不过：多半是丢字节/截断（通道拥塞或信号差） */
        return;
    }
    /* 句子类型从第 3 个字符开始，短于 6 字节的畸形帧先挡掉：以前长度检查排在
     * raw_is_wanted() / strncmp() 之后，短句会先被按偏移读取（虽不越界，但读到
     * 的是 s_line 里的陈旧字节），判定结果不可信。 */
    if (strlen(line) < 6) {
        s_st.bad_count++;
        return;
    }
    /* 探测判据：只要收到校验正确的语句就算"有数据流"，必须放在整段筛选之前 ——
     * 否则 5 Hz 模块在探测窗口内可能只碰上非整数秒段而误判为"没数据"。 */
    s_detect_hit = true;

    /* ---- 多 Hz 模块：只保留整数秒那一整段（见 s_burst_keep 的说明） ---- */
    bool is_rmc = (strncmp(line + 3, "RMC", 3) == 0);
    if (is_rmc || strncmp(line + 3, "GGA", 3) == 0) {
        s_burst_keep    = ts_is_whole_second(line);
        s_burst_mark_us = tb_now_us();
    } else if (!s_burst_keep && (tb_now_us() - s_burst_mark_us) > 1500000ULL) {
        s_burst_keep = true;   /* 界标信号消失太久：取消筛选，避免把数据流掐断 */
    }
    if (is_rmc) {
        s_st.rmc_seen++;
    }
    if (!s_burst_keep) {
        if (is_rmc) {
            s_st.rmc_drop++;
        }
        return;
    }

    s_st.nmea_count++;
    if (raw_is_wanted(line)) {
        raw_store(line);
    }
    if (s_probe_mode) {
        return;
    }
    if (is_rmc) {
        parse_rmc(line, s_sentence_start_tb);
    } else if (strncmp(line + 3, "GGA", 3) == 0) {
        parse_gga(line);
    } else if (strncmp(line + 3, "GSV", 3) == 0) {
        parse_gsv(line);
    }
}

static void handle_ubx(const uint8_t *b, size_t len)
{
    if (len < 8) {
        return;
    }
    uint8_t  cls  = b[2];
    uint8_t  id   = b[3];
    uint16_t plen = (uint16_t)(b[4] | ((uint16_t)b[5] << 8));
    if (len < (size_t)plen + 8) {
        return;
    }
    uint8_t a = 0, c = 0;
    for (int i = 2; i < 6 + (int)plen; i++) {
        a = (uint8_t)(a + b[i]);
        c = (uint8_t)(c + a);
    }
    if (a != b[6 + plen] || c != b[7 + plen]) {
        s_st.bad_count++;
        return;
    }
    s_detect_hit = true;

    if (cls == 0x05 && id == 0x01 && plen >= 2) {
        s_got_ack = true;
        s_ack_cls = b[6];
        s_ack_id  = b[7];
        s_st.ubx_ack++;
    } else if (cls == 0x05 && id == 0x00) {
        s_st.ubx_nak++;
    } else if (cls == 0x01 && id == 0x20 && plen >= 16) {
        /* UBX-NAV-TIMEGPS: iTOW(0..3) fTOW(4..7) week(8..9) leapS(10) valid(11) tAcc(12..15) */
        s_st.leap_s       = (int8_t)b[6 + 10];
        s_st.leap_valid   = (b[6 + 11] & 0x04) != 0;   /* leapSValid */
    }
}

static void feed_byte(uint8_t c, uint64_t t_arrive)
{
    switch (s_ps) {
    case PS_SYNC:
        if (c == '$') {
            s_pos = 0;
            s_sentence_start_tb = t_arrive;
            s_line[s_pos++] = c;
            s_ps = PS_NMEA;
        } else if (c == 0xB5) {
            s_pos = 0;
            s_line[s_pos++] = c;
            s_ps = PS_UBX;
        }
        break;

    case PS_NMEA:
        if (s_pos < sizeof(s_line) - 1) {
            s_line[s_pos++] = c;
        }
        if (c == '\n') {
            s_line[s_pos] = 0;
            handle_nmea((const char *)s_line);
            reset_parser();
        } else if (s_pos >= sizeof(s_line) - 1) {
            s_st.bad_count++;                  /* 单条语句超长被截断 */
            reset_parser();
        }
        break;

    case PS_UBX:
        if (s_pos < sizeof(s_line)) {
            s_line[s_pos++] = c;
        }
        if (s_pos == 2 && s_line[1] != 0x62) {
            reset_parser();
            break;
        }
        if (s_pos >= 6) {
            s_ubx_len = (uint16_t)(s_line[4] | ((uint16_t)s_line[5] << 8));
            if (s_ubx_len + 8u > sizeof(s_line)) {
                s_st.bad_count++;
                reset_parser();
                break;
            }
            if (s_pos >= s_ubx_len + 8u) {
                handle_ubx(s_line, s_pos);
                reset_parser();
            }
        }
        break;
    }
}

/* ====================== 波特率自适应 =============================== */
static void update_byte_time(void)
{
    /* 8N1 = 10 bit/byte，µs 的 Q16 定点 */
    s_byte_us_q16 = (uint32_t)((10ULL * 1000000ULL * 65536ULL) / s_baud);
}

/* 在 ms 毫秒内持续读取串口并喂给状态机。
 * 探测模式下一旦收到有效语句就立刻返回，不用空等满整个窗口。 */
static void pump_uart(uint32_t ms, bool probe)
{
    uint8_t  b[64];
    int64_t  t0 = esp_timer_get_time();
    s_probe_mode = probe;
    while (esp_timer_get_time() - t0 < (int64_t)ms * 1000LL) {
        /* 探测窗口里可能连续几秒收不到任何字节：必须在这里喂狗。否则
         * "5 个候选 × CFG_GPS_BAUD_PROBE_MS" 会一路攒到接近 TWDT 超时（10s），
         * 没接 GPS 时表现为反复 panic 复位。 */
        esp_task_wdt_reset();
        int n = uart_read_bytes(CFG_GPS_UART, b, sizeof(b), pdMS_TO_TICKS(10));
        if (n > 0) {
            uint64_t t_end = tb_now_us();
            for (int i = 0; i < n; i++) {
                uint64_t ti = t_end - ((((uint64_t)(n - 1 - i)) * s_byte_us_q16) >> 16);
                feed_byte(b[i], ti);
            }
            /* 原始字节转发给 TCP（远程观察用）放在最后：授时的时标与解析先走完，
             * 转发多花的那点时间落不到任何时间量上（见 gnss_tcp.c 顶部说明）。
             * 探测阶段的乱码也照转 —— 那正是"模块到底在说什么"的一部分。 */
            gnss_tcp_feed(b, (size_t)n);
            if (probe && s_detect_hit) {
                break;
            }
        }
    }
    s_probe_mode = false;
}

static bool probe_baud(uint32_t baud)
{
    if (uart_set_baudrate(CFG_GPS_UART, baud) != ESP_OK) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    uart_flush_input(CFG_GPS_UART);
    reset_parser();
    s_detect_hit = false;
    s_baud = baud;
    update_byte_time();
    pump_uart(CFG_GPS_BAUD_PROBE_MS, true);
    return s_detect_hit;
}

/* 走一遍波特率候选，命中即锁定。若已知当前波特率，先试它 ——
 * "模块只是短暂静默"时能立刻恢复，不必逐个降速。启动阶段与运行中
 * 失联重探都走这里。全部失败时恢复原来的波特率设置（probe_baud 会把
 * s_baud 改成刚试过的那个），保持"当前波特率"这个概念的连贯。 */
static bool probe_baud_loop(void)
{
    uint32_t prev = s_baud;

    if (prev != 0 && probe_baud(prev)) {
        s_baud_locked = true;
        return true;
    }
    for (int i = 0; i < (int)GPS_BAUD_N; i++) {
        if ((uint32_t)s_baud_list[i] == prev) {
            continue;                       /* 上面已经试过 */
        }
        if (probe_baud(s_baud_list[i])) {
            s_baud_locked = true;
            return true;
        }
    }

    if (prev != 0) {
        uart_set_baudrate(CFG_GPS_UART, prev);
        s_baud = prev;
        update_byte_time();
    }
    s_baud_locked = false;
    return false;
}

#if CFG_GPS_SET_BAUD
/* 发一次 UBX-CFG-PRT，把模块切到 target；判据是"切过去后还能不能收到报文"。
 * 回退是安全的：模块若没理会这条指令，回退后数据流立刻恢复；模块若已经切了而
 * 本机新速率下收不到（线材/干扰），回退后同样收不到，会由 RELOCK 机制重新探测。 */
static bool try_switch_baud_once(uint32_t target)
{
    if (target == s_baud) {
        return true;
    }
    uint8_t p[20] = { 0 };
    p[0]  = 1;                                   /* portID = UART1 */
    p[1]  = 0;                                   /* reserved0      */
    p[2]  = 0; p[3] = 0;                         /* txReady        */
    p[4]  = 0xD0; p[5] = 0x08; p[6] = 0x00; p[7] = 0x00;   /* mode: 8N1 (0x000008D0) */
    p[8]  = (uint8_t)(target & 0xFF);
    p[9]  = (uint8_t)((target >> 8) & 0xFF);
    p[10] = (uint8_t)((target >> 16) & 0xFF);
    p[11] = (uint8_t)((target >> 24) & 0xFF);
    p[12] = 0x03; p[13] = 0x00;                  /* inProtoMask  = UBX|NMEA */
    p[14] = 0x03; p[15] = 0x00;                  /* outProtoMask = UBX|NMEA */
    p[16] = 0x00; p[17] = 0x00;                  /* flags */
    p[18] = 0x00; p[19] = 0x00;                  /* reserved1 */

    s_got_ack = false;
    s_ack_cls = 0;
    s_ack_id  = 0;
    ubx_send(0x06, 0x00, p, 20);
    s_st.cfg_sent++;
    pump_uart(400, false);                       /* 只为看一眼 ACK/NAK */
    int acked = (s_got_ack && s_ack_cls == 0x06 && s_ack_id == 0x00) ? 1 : 0;

    /* 本机切过去，看数据流还在不在 —— 这才是真判据 */
    uint32_t old = s_baud;
    uart_set_baudrate(CFG_GPS_UART, target);
    vTaskDelay(pdMS_TO_TICKS(50));               /* 等模块让新参数生效、本机稳定 */
    uart_flush_input(CFG_GPS_UART);
    reset_parser();
    s_baud = target;
    update_byte_time();

    s_detect_hit = false;
    pump_uart(2000, true);
    if (s_detect_hit) {
        ESP_LOGI(TAG, "GPS 波特率已切换到 %u%s（ack=%d nak=%lu）",
                 (unsigned)target, acked ? "" : "，模块未回 ACK 但数据流正常",
                 acked, (unsigned long)s_st.ubx_nak);
        return true;
    }

    /* 收不到 → 切回去，并确认回退后数据流仍在 */
    uart_set_baudrate(CFG_GPS_UART, old);
    vTaskDelay(pdMS_TO_TICKS(50));
    uart_flush_input(CFG_GPS_UART);
    reset_parser();
    s_baud = old;
    update_byte_time();
    s_detect_hit = false;
    pump_uart(500, true);
    ESP_LOGW(TAG, "%u 下收不到数据，回退到 %u（ack=%d nak=%lu）",
             (unsigned)target, (unsigned)old, acked, (unsigned long)s_st.ubx_nak);
    return false;
}

/* 上电握手成功 / 模块重启后调用：尽量把模块往高波特率上提（越高 lag 越小）。
 * 只往上试、不降速 —— 降速只会让滞后变大。从候选表里挑 "高于当前 且 不超过
 * cap" 的项，从高到低逐个尝试，第一个成功的就是可用范围内最高的速率；全都不
 * 成功就留在原波特率（每次失败都已在 try_switch_baud_once 里切回来了）。 */
static void try_switch_baud(uint32_t cap)
{
    if (cap == 0) {
        cap = (uint32_t)s_baud_list[0];          /* 0 = 不限，取表里最高 */
    }
    for (int i = 0; i < (int)GPS_BAUD_N; i++) {
        uint32_t cand = s_baud_list[i];          /* 表已按 高 → 低 排列 */
        if (cand <= s_baud) {
            break;                               /* 再往后只会更低，停 */
        }
        if (cand > cap) {
            continue;                            /* 超出上限，跳过 */
        }
        if (try_switch_baud_once(cand)) {
            return;
        }
    }
    ESP_LOGI(TAG, "保持当前波特率 %u（已是可用范围内最高）", (unsigned)s_baud);
}
#endif /* CFG_GPS_SET_BAUD */

#if CFG_GPS_SEND_UBX_CFG
/* ====================== UBX 配置下发 =============================== */
static void cfg_msg(uint8_t cls, uint8_t id, uint8_t rate)
{
    uint8_t p[3] = { cls, id, rate };
    ubx_send(0x06, 0x01, p, 3);
    s_st.cfg_sent++;
    pump_uart(120, false);
}

static void send_ubx_config(void)
{
    /* 1) 定位率 1 Hz，时间基准 UTC */
    {
        uint8_t p[6];
        p[0] = 0xE8; p[1] = 0x03;        /* measRate = 1000 ms */
        p[2] = 0x01; p[3] = 0x00;        /* navRate  = 1       */
        p[4] = 0x00; p[5] = 0x00;        /* timeRef  = UTC     */
        ubx_send(0x06, 0x08, p, 6);
        s_st.cfg_sent++;
        pump_uart(150, false);
    }

    /* 2) 导航模型 = 静止（固定站能显著提升授时精度） */
    {
        uint8_t p[36] = { 0 };
        p[0] = 0x01; p[1] = 0x00;        /* mask: 仅应用 dynModel */
        p[2] = 2;                        /* dynModel = stationary */
        p[3] = 3;                        /* fixMode  = auto 2D/3D */
        p[12] = 5;                       /* minElev  = 5 deg      */
        ubx_send(0x06, 0x24, p, 36);
        s_st.cfg_sent++;
        pump_uart(150, false);
    }

    /* 3) 裁剪 NMEA：只留 RMC / GGA，尽量缩短突发长度 */
    cfg_msg(0xF0, 0x04, 1);                              /* RMC */
    cfg_msg(0xF0, 0x00, 1);                              /* GGA */
    cfg_msg(0xF0, 0x01, 0);                              /* GLL */
    cfg_msg(0xF0, 0x02, 0);                              /* GSA */
    cfg_msg(0xF0, 0x03, (uint8_t)(CFG_GPS_KEEP_GSV ? 1 : 0)); /* GSV */
    cfg_msg(0xF0, 0x05, 0);                              /* VTG */
    cfg_msg(0xF0, 0x08, 0);                              /* ZDA */
    cfg_msg(0x01, 0x20, 1);                              /* UBX NAV-TIMEGPS（闰秒） */

    /* 4) 时间脉冲：1 Hz / UTC 网格 / 对齐秒首 / 加宽脉宽 / 锁定 GPS 频率 */
    {
        uint8_t p[32] = { 0 };
        uint32_t period = 1000000UL;
        uint32_t plen   = CFG_TP5_PULSE_LEN_US;
        uint32_t flags  = 0x01UL | 0x02UL | 0x04UL | 0x10UL | 0x20UL;   /* active/lockGnssFreq/lockedOtherSet/isLength/alignToTow */
#if CFG_TP5_POLARITY_RISING
        flags |= 0x40UL;                                                 /* polarity: 上升沿为脉冲起点 */
#endif
        int16_t cable = (int16_t)CFG_TP5_ANT_CABLE_DELAY_NS;

        p[0] = 0x00;                     /* tpIdx = TIMEPULSE */
        p[1] = 0x01;                     /* version           */
        p[2] = 0x00; p[3] = 0x00;        /* reserved1         */
        p[4] = (uint8_t)(cable & 0xFF); p[5] = (uint8_t)((cable >> 8) & 0xFF);
        p[6] = 0x00; p[7] = 0x00;        /* rfGroupDelay      */
        p[8]  = (uint8_t)(period & 0xFF);        p[9]  = (uint8_t)((period >> 8) & 0xFF);
        p[10] = (uint8_t)((period >> 16) & 0xFF); p[11] = (uint8_t)((period >> 24) & 0xFF);
        p[12] = p[8];  p[13] = p[9];  p[14] = p[10]; p[15] = p[11];      /* freqPeriodLock */
        p[16] = (uint8_t)(plen & 0xFF);        p[17] = (uint8_t)((plen >> 8) & 0xFF);
        p[18] = (uint8_t)((plen >> 16) & 0xFF); p[19] = (uint8_t)((plen >> 24) & 0xFF);
        p[20] = p[16]; p[21] = p[17]; p[22] = p[18]; p[23] = p[19];      /* pulseLenRatioLock */
        p[24] = p[25] = p[26] = p[27] = 0;                                /* userConfigDelay  */
        p[28] = (uint8_t)(flags & 0xFF);        p[29] = (uint8_t)((flags >> 8) & 0xFF);
        p[30] = (uint8_t)((flags >> 16) & 0xFF); p[31] = (uint8_t)((flags >> 24) & 0xFF);

        ubx_send(0x06, 0x31, p, 32);
        s_st.cfg_sent++;
        pump_uart(200, false);
    }

    ESP_LOGI(TAG, "UBX 配置已下发：1Hz / 静止模型 / RMC+GGA / PPS %u us 脉宽, flags=0x%02X",
             (unsigned)CFG_TP5_PULSE_LEN_US,
             (unsigned)(0x01 | 0x02 | 0x04 | 0x10 | 0x20 | (CFG_TP5_POLARITY_RISING ? 0x40 : 0)));
}
#endif /* CFG_GPS_SEND_UBX_CFG */

/* 把当前状态发布给其他任务读（只在 GPS 任务上下文调用）。
 * 整个结构体在临界区里拷贝，读者就不会看到"半新半旧"的字段组合；
 * 调用频率与主循环相同（约 10ms 一次），开销可忽略。 */
static void publish_status(void)
{
    portENTER_CRITICAL(&s_st_lock);
    s_st_pub = s_st;
    s_st_pub.baud        = s_baud;
    s_st_pub.baud_locked = s_baud_locked;
    portEXIT_CRITICAL(&s_st_lock);
}

/* ====================== 模块恢复 ===================================
 * 长期拿不到定位时用 UBX-CFG-RST 软复位"抢救"模块（CFG_GPS_RST_MODE，默认
 * 温启动）—— 纯软件，不需要任何额外硬件。详见 config.h 的说明。 */
#if GPS_RECOVER_ENABLED
#if CFG_GPS_RST_MODE == 1
#define GPS_RST_NAME "热"
#elif CFG_GPS_RST_MODE == 3
#define GPS_RST_NAME "冷"
#else
#define GPS_RST_NAME "温"
#endif

/* 发 UBX-CFG-RST 让模块自己复位。navBbrMask 决定清掉多少历史数据：
 *   0x0000 = 热启动（全保留，最快）
 *   0x0001 = 温启动（只清星历，保留历书/位置/时间 —— 丢定位后最常用）
 *   0xFFFF = 冷启动（全清，最慢但最彻底）
 * resetMode = 0x02 表示"只复位 GNSS 子系统"，**不动 UART/端口配置**，所以
 * 波特率保持不变、不必重新握手。复位会让模块重启，通常收不到 ACK，故不等 ACK。
 * 模块若不认这条指令（ROM 只读版 / 屏蔽了 CFG 写入），最多是没效果，无害。 */
static void gps_ubx_reset(void)
{
    uint16_t bbr;
    uint8_t  p[4];

#if CFG_GPS_RST_MODE == 1
    bbr = 0x0000;
#elif CFG_GPS_RST_MODE == 3
    bbr = 0xFFFF;
#else
    bbr = 0x0001;
#endif
    p[0] = (uint8_t)(bbr & 0xFF);
    p[1] = (uint8_t)(bbr >> 8);
    p[2] = 0x02;                        /* resetMode: GNSS-only controlled sw reset */
    p[3] = 0x00;                        /* reserved1 */
    ubx_send(0x06, 0x04, p, 4);
    s_st.cfg_sent++;
}

static void gps_recover(void)
{
#if CFG_GPS_RST_MODE > 0
    ESP_LOGW(TAG, "  → 发送 UBX-CFG-RST（%s启动）", GPS_RST_NAME);
    gps_ubx_reset();
    /* 等模块重启并重新吐数据（3 s），顺带喂狗 */
    for (int i = 0; i < 30; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_task_wdt_reset();
    }
#endif
}
#endif /* GPS_RECOVER_ENABLED */

/* ====================== 主任务 ===================================== */
static void gps_task(void *arg)
{
    (void)arg;
    if (esp_task_wdt_add(NULL) != ESP_OK) {
        ESP_LOGW(TAG, "TWDT 订阅失败（任务看门狗未启用？）");
    }

#if GPS_RECOVER_ENABLED
    s_last_fix_tb = tb_now_us();            /* 给模块留足首次定位的时间 */
#endif

    /* 串口驱动就装在本任务所在的核（CORE_TIME）上，UART 中断与 PPS 中断
     * 同核，NMEA 时戳估计受核间调度影响最小。 */
    uart_config_t cfg = {
        .baud_rate  = (int)s_baud_list[0],
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(CFG_GPS_UART, CFG_GPS_UART_RX_BUF,
                                        CFG_GPS_UART_TX_BUF, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CFG_GPS_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(CFG_GPS_UART, CFG_GPS_TX_GPIO, CFG_GPS_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART%d 已启动: TX=IO%d RX=IO%d，自动波特率 %u..%u",
             CFG_GPS_UART, CFG_GPS_TX_GPIO, CFG_GPS_RX_GPIO,
             (unsigned)s_baud_list[GPS_BAUD_N - 1],
             (unsigned)s_baud_list[0]);

    /* ---- 自动波特率 ---- */
    while (!s_baud_locked) {
        if (probe_baud_loop()) {
            ESP_LOGI(TAG, "GPS 波特率识别成功：%u baud", (unsigned)s_baud);
        } else {
            ESP_LOGW(TAG, "未识别到 GPS 数据流，3 秒后重试...");
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
    s_st.baud = s_baud;
    s_st.baud_locked = true;

#if CFG_GPS_SET_BAUD
    try_switch_baud(CFG_GPS_TARGET_BAUD);
#endif
    s_st.baud = s_baud;

#if CFG_GPS_SEND_UBX_CFG
    send_ubx_config();
#endif

    /* ---- 正常接收 ---- */
    uint8_t  buf[64];
    uint32_t last_count  = s_st.nmea_count;
    uint32_t last_active = (uint32_t)(tb_now_us() / 1000ULL);
    reset_parser();
    while (1) {
        int n = uart_read_bytes(CFG_GPS_UART, buf, sizeof(buf), pdMS_TO_TICKS(10));
        if (n > 0) {
            uint64_t t_end = tb_now_us();
            for (int i = 0; i < n; i++) {
                uint64_t ti = t_end - ((((uint64_t)(n - 1 - i)) * s_byte_us_q16) >> 16);
                feed_byte(buf[i], ti);
            }
            /* 原始字节转发给 TCP（远程观察用）。放在解析之后：时基与解析先走完，
             * 转发不会挤进授时路径（见 gnss_tcp.c 顶部说明）。 */
            gnss_tcp_feed(buf, (size_t)n);
        }
        esp_task_wdt_reset();

        /* 闰秒交叉校验（每秒一次足够） */
        static uint32_t s_last_leap_check = 0;
        static uint32_t s_last_leap_poll  = 0;
        uint32_t now_ms = (uint32_t)(tb_now_us() / 1000ULL);

        /* ---- 串口失联自愈 ----
         * 模块掉电重启（u-blox 会回到出厂 9600）、被换过、或厂商固件改了
         * 波特率之后，固件若一直停在旧波特率就会永远收不到 NMEA、永久停在
         * stratum 16 且无人值守时无法自愈。这里以"多久没有新增合法语句"
         * 为判据重新探测（CFG_GPS_RELOCK_SEC = 0 可关闭）。 */
        if (s_st.nmea_count != last_count) {
            last_count  = s_st.nmea_count;
            last_active = now_ms;
        } else if (CFG_GPS_RELOCK_SEC > 0 &&
                   (uint32_t)(now_ms - last_active) > (uint32_t)CFG_GPS_RELOCK_SEC * 1000u) {
            last_active = now_ms;
            ESP_LOGW(TAG, "%u 秒未收到合法 NMEA，重新探测波特率（当前 %u）",
                     (unsigned)CFG_GPS_RELOCK_SEC, (unsigned)s_baud);
            if (probe_baud_loop()) {
                ESP_LOGI(TAG, "GPS 波特率已恢复：%u baud", (unsigned)s_baud);
            } else {
                ESP_LOGW(TAG, "重新探测失败，保持 %u baud 继续监听", (unsigned)s_baud);
            }
            s_st.baud        = s_baud;
            s_st.baud_locked = s_baud_locked;
            reset_parser();
        }

#if CFG_GPS_POLL_NAV_TIMEGPS
        /* 只读查询，不是配置写入：模块不支持就收不到，不会有任何副作用 */
        if (now_ms - s_last_leap_poll > (uint32_t)CFG_GPS_LEAP_POLL_MS) {
            s_last_leap_poll = now_ms;
            ubx_send(0x01, 0x20, NULL, 0);      /* UBX-NAV-TIMEGPS poll */
        }
#endif

        if (now_ms - s_last_leap_check > 1000u) {
            s_last_leap_check = now_ms;
            uint32_t sec, frac;
            if (discipline_get_utc(tb_now_us(), &sec, &frac)) {
                uint8_t exp = gps_utc_offset(sec);
                s_st.leap_expected = (int8_t)exp;
                s_st.leap_mismatch = s_st.leap_valid && (s_st.leap_s != (int8_t)exp);
            }
        }

#if GPS_RECOVER_ENABLED
        /* ---- 长时间拿不到定位 → 软复位唤醒模块 ----
         * 放在主循环末尾：恢复流程会阻塞几秒，走完直接进入下一轮，不会再碰到
         * 上面的自愈 / 轮询逻辑。 */
        if (s_st.fix_valid) {
            s_last_fix_tb = tb_now_us();
            s_restart_cnt = 0;                 /* 恢复定位即清零连续恢复计数 */
        } else if ((tb_now_us() - s_last_fix_tb) > (uint64_t)CFG_GPS_REFIX_SEC * 1000000ULL) {
            if (CFG_GPS_REFIX_MAX > 0 && s_restart_cnt >= CFG_GPS_REFIX_MAX) {
                s_last_fix_tb = tb_now_us();   /* 已达上限：不再折腾模块，只推后检查 */
                ESP_LOGW(TAG, "连续 %u 次尝试恢复仍无合格定位，停止重试，"
                              "请检查天线/视野/模块供电", (unsigned)s_restart_cnt);
            } else {
                s_restart_cnt++;
                ESP_LOGW(TAG, "%d 秒无合格定位，第 %u 次尝试恢复模块",
                         CFG_GPS_REFIX_SEC, (unsigned)s_restart_cnt);
                gps_recover();
                reset_parser();
                last_count    = s_st.nmea_count;    /* 复位失联自愈判据 */
                last_active   = (uint32_t)(tb_now_us() / 1000ULL);
                s_last_fix_tb = tb_now_us();        /* 重新计时 */
            }
        }
#endif

        publish_status();
    }
}

void gps_init(void)
{
    s_st.baud = (uint32_t)s_baud_list[0];
    s_st.leap_s = 0;
    s_st.leap_valid = false;
    publish_status();          /* 先发布一次初值，面板不会看到全零状态 */

    /* 固定跑在 CORE_TIME（与 PPS 中断同核） */
    if (xTaskCreatePinnedToCore(gps_task, "gps", CFG_GPS_TASK_STACK, NULL,
                                CFG_GPS_TASK_PRIO, NULL, CFG_CORE_TIME) != pdPASS) {
        ESP_LOGE(TAG, "gps 任务创建失败（内存不足？），GPS 授时不可用");
    }
}

void gps_get_status(gps_status_t *st)
{
    /* 读发布快照（见 publish_status），不直接读 s_st */
    portENTER_CRITICAL(&s_st_lock);
    *st = s_st_pub;
    portEXIT_CRITICAL(&s_st_lock);
}
