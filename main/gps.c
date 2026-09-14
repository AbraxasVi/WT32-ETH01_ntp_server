/*
 * GPS 接入层实现
 */
#include "gps.h"
#include "config.h"
#include "discipline.h"

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

static const uint32_t s_baud_list[CFG_GPS_BAUD_COUNT] = CFG_GPS_BAUD_LIST;

/* ---------------- 运行状态 ---------------- */
static gps_status_t s_st;
static uint32_t     s_baud = 0;
static bool         s_baud_locked = false;
static uint32_t     s_byte_us_q16;         /* 每字节耗时(µs) 的 Q16 定点 */
static bool         s_probe_mode = false;  /* 波特率探测中：只判定，不解析 */
static bool         s_detect_hit = false;
static bool         s_got_ack = false;
static uint8_t      s_ack_cls = 0, s_ack_id = 0;

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
        days += s_dim[m - 1];
        if (m == 2 && is_leap(y)) {
            days += 1;
        }
    }
    days += d - 1;
    return (uint32_t)days * 86400u + (uint32_t)h * 3600u + (uint32_t)mi * 60u + (uint32_t)s;
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

static void raw_store(const char *line)
{
    char  *dst = s_raw[s_raw_head];
    size_t i   = 0;

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
}

int gps_get_last_nmea(char dst[][GPS_RAW_LINE_LEN], int max)
{
    int start = (int)s_raw_head - (int)s_raw_count;
    int n     = 0;

    if (start < 0) {
        start += GPS_RAW_LINES;
    }
    for (int i = 0; i < (int)s_raw_count && n < max; i++) {
        int idx = (start + i) % GPS_RAW_LINES;
        snprintf(dst[n], GPS_RAW_LINE_LEN, "%s", s_raw[idx]);
        n++;
    }
    return n;
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
    int nf = 0;
    char *p = strtok(s_fbuf, ",");
    while (p && nf < 32) {
        s_f[nf++] = p;
        p = strtok(NULL, ",");
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

static void parse_rmc(const char *line, uint64_t sentence_start_tb)
{
    int nf = split_fields(line);
    if (nf < 10) {
        return;
    }
    if (strcmp(s_f[2], "A") != 0) {
        return;                              /* 'V' = 定位无效 */
    }
    const char *t = s_f[1];
    const char *d = s_f[9];
    if (strlen(t) < 6 || strlen(d) != 6) {
        return;
    }
    int hh, mm, ss, DD, MO, YY;
    if (sscanf(t, "%2d%2d%2d", &hh, &mm, &ss) != 3) {
        return;
    }
    if (sscanf(d, "%2d%2d%2d", &DD, &MO, &YY) != 3) {
        return;
    }
    int y = (YY < 70) ? (2000 + YY) : (1900 + YY);
    uint32_t unix_sec = to_unix(y, MO, DD, hh, mm, ss);
    discipline_on_nmea_second(unix_sec, sentence_start_tb);
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
    s_st.fix_valid   = (fq > 0 && s_st.sats_used >= CFG_GPS_MIN_SATS);

    discipline_on_fix(s_st.fix_quality, s_st.sats_used);
}

static void parse_gsv(const char *line)
{
    int nf = split_fields(line);
    if (nf < 4) {
        return;
    }
    int view = atoi(s_f[3]);
    s_st.sats_view = (uint8_t)(view < 0 ? 0 : (view > 255 ? 255 : view));
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
        return;
    }
    s_detect_hit = true;
    s_st.nmea_count++;
    if (raw_is_wanted(line)) {
        raw_store(line);
    }
    if (s_probe_mode) {
        return;
    }
    if (strlen(line) < 6) {
        return;
    }
    if (strncmp(line + 3, "RMC", 3) == 0) {
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
        int n = uart_read_bytes(CFG_GPS_UART, b, sizeof(b), pdMS_TO_TICKS(10));
        if (n > 0) {
            uint64_t t_end = tb_now_us();
            for (int i = 0; i < n; i++) {
                uint64_t ti = t_end - ((((uint64_t)(n - 1 - i)) * s_byte_us_q16) >> 16);
                feed_byte(b[i], ti);
            }
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

#if CFG_GPS_SET_BAUD
/* 用 UBX-CFG-PRT 把模块切到目标波特率；失败自动回退。
 * 注意：这是写操作，默认关闭（CFG_GPS_SET_BAUD=0）。 */
static bool try_switch_baud(uint32_t target)
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
    pump_uart(500, false);

    if (!s_got_ack || s_ack_cls != 0x06 || s_ack_id != 0x00) {
        ESP_LOGW(TAG, "CFG-PRT -> %u 未收到 ACK，保持 %u", (unsigned)target, (unsigned)s_baud);
        return false;
    }

    uint32_t old = s_baud;
    uart_set_baudrate(CFG_GPS_UART, target);
    vTaskDelay(pdMS_TO_TICKS(20));
    uart_flush_input(CFG_GPS_UART);
    reset_parser();
    s_baud = target;
    update_byte_time();

    s_detect_hit = false;
    pump_uart(2000, true);
    if (s_detect_hit) {
        ESP_LOGI(TAG, "GPS 波特率已切换到 %u", (unsigned)target);
        return true;
    }

    /* 回退 */
    uart_set_baudrate(CFG_GPS_UART, old);
    vTaskDelay(pdMS_TO_TICKS(20));
    uart_flush_input(CFG_GPS_UART);
    reset_parser();
    s_baud = old;
    update_byte_time();
    ESP_LOGW(TAG, "新波特率 %u 下收不到数据，已回退到 %u", (unsigned)target, (unsigned)old);
    return false;
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

/* ====================== 主任务 ===================================== */
static void gps_task(void *arg)
{
    (void)arg;
    if (esp_task_wdt_add(NULL) != ESP_OK) {
        ESP_LOGW(TAG, "TWDT 订阅失败（任务看门狗未启用？）");
    }

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
    /* 显式把 UART2 从默认 IO16/IO17 改到 IO17(TX)/IO5(RX)，
     * 把 IO16 完全让给以太网 50MHz 振荡器使能，消除引脚冲突 */
    ESP_ERROR_CHECK(uart_set_pin(CFG_GPS_UART, CFG_GPS_TX_GPIO, CFG_GPS_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART%d 已启动: TX=IO%d RX=IO%d，自动波特率 %u..%u",
             CFG_GPS_UART, CFG_GPS_TX_GPIO, CFG_GPS_RX_GPIO,
             (unsigned)s_baud_list[CFG_GPS_BAUD_COUNT - 1],
             (unsigned)s_baud_list[0]);

    /* ---- 自动波特率 ---- */
    while (!s_baud_locked) {
        for (int i = 0; i < CFG_GPS_BAUD_COUNT && !s_baud_locked; i++) {
            if (probe_baud(s_baud_list[i])) {
                s_baud_locked = true;
                ESP_LOGI(TAG, "GPS 波特率识别成功：%u baud", (unsigned)s_baud);
            }
        }
        if (!s_baud_locked) {
            ESP_LOGW(TAG, "未识别到 GPS 数据流，3 秒后重试...");
            esp_task_wdt_reset();
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
    uint8_t buf[64];
    reset_parser();
    while (1) {
        int n = uart_read_bytes(CFG_GPS_UART, buf, sizeof(buf), pdMS_TO_TICKS(10));
        if (n > 0) {
            uint64_t t_end = tb_now_us();
            for (int i = 0; i < n; i++) {
                uint64_t ti = t_end - ((((uint64_t)(n - 1 - i)) * s_byte_us_q16) >> 16);
                feed_byte(buf[i], ti);
            }
        }
        esp_task_wdt_reset();

        /* 闰秒交叉校验（每秒一次足够） */
        static uint32_t s_last_leap_check = 0;
        static uint32_t s_last_leap_poll  = 0;
        uint32_t now_ms = (uint32_t)(tb_now_us() / 1000ULL);

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
    }
}

void gps_init(void)
{
    s_st.baud = (uint32_t)s_baud_list[0];
    s_st.leap_s = 0;
    s_st.leap_valid = false;

    /* 固定跑在 CORE_TIME（与 PPS 中断同核） */
    xTaskCreatePinnedToCore(gps_task, "gps", CFG_GPS_TASK_STACK, NULL,
                            CFG_GPS_TASK_PRIO, NULL, CFG_CORE_TIME);
}

void gps_get_status(gps_status_t *st)
{
    *st = s_st;
    st->baud = s_baud;
    st->baud_locked = s_baud_locked;
}
