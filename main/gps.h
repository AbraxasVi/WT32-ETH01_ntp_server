/*
 * GPS（u-blox M8N 系列，带 PPS）串口接入层
 *  - 9600 ~ 115200 波特率自动识别（被动探测，不改模块）
 *  - 默认不向模块写入任何配置（CFG_GPS_SET_BAUD=0 / CFG_GPS_SEND_UBX_CFG=0），
 *    兼容 ROM 只读、厂商屏蔽 CFG 写入的模块
 *  - 解析 RMC(UTC 秒) / GGA(Fix Quality + 参与解算卫星数) / GSV(可见卫星数)
 *  - 只读轮询 UBX-NAV-TIMEGPS 的 leapS，与内置闰秒表交叉校验
 */
#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 原始报文留痕（排障用） ---- */
#define GPS_RAW_LINES      15      /* 保留的条数 */
#define GPS_RAW_LINE_LEN   128     /* 单条最大长度（含尾零） */

typedef struct {
    uint32_t baud;            /* 当前串口波特率               */
    bool     baud_locked;     /* 是否已识别到波特率           */
    bool     fix_valid;       /* fix_quality>0 且 卫星数达标  */
    uint8_t  fix_quality;     /* GGA field6: 0无效 1单点 2DGPS ... */
    uint8_t  sats_used;       /* GGA field7: 参与解算卫星数   */
    uint8_t  sats_view;       /* GSV field3: 可见卫星数       */
    uint16_t hdop_x10;        /* GGA field8: HDOP * 10        */
    uint32_t nmea_count;      /* 累计解析出的合法 NMEA 语句   */
    uint32_t ubx_ack;         /* UBX ACK-ACK 计数             */
    uint32_t ubx_nak;         /* UBX ACK-NAK 计数             */
    uint32_t cfg_sent;        /* 已下发的 UBX 配置条数        */
    int8_t   leap_s;          /* 模块上报的 GPS-UTC 闰秒      */
    bool     leap_valid;      /* 模块 leapS 有效位            */
    int8_t   leap_expected;   /* 内置闰秒表推算值             */
    bool     leap_mismatch;   /* 两者不一致                   */
} gps_status_t;

void gps_init(void);
void gps_get_status(gps_status_t *st);

/* 取出最近若干条 GGA/GSA/ZDA 原始语句（从旧到新），返回实际条数。
 * dst 至少要有 max 行 * GPS_RAW_LINE_LEN 的空间。 */
int  gps_get_last_nmea(char dst[][GPS_RAW_LINE_LEN], int max);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
