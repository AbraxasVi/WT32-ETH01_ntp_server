/*
 * 全项目硬件 / 策略配置集中在这里。
 * 目标板：WT32-ETH01 (ESP32 + LAN8720 RMII) + u-blox M8N 系列（带 PPS）
 * 目标平台：ESP-IDF v6.1
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================== 以太网 / 网络 ==========================
 * IP 获取方式：只支持 DHCP 自动分配（地址由路由器分配，换网段即插即用）。
 * 说明：这里曾有一个 CFG_USE_DHCP 开关声称"0 = 静态 IP"，但
 * ESP_NETIF_DEFAULT_ETH() 生成的 netif 自带 ESP_NETIF_DHCP_CLIENT 标志，
 * 把它关掉既不会配置静态地址、也拦不住 dhcpc，属于死配置，已删除。
 * 若确实需要静态地址：自行在 eth_if_init() 里 esp_netif_dhcpc_stop() +
 * esp_netif_set_ip_info()，并把 sdkconfig 的 DHCP 相关选项一并考虑。
 * ------------------------------------------------------------------ */
#define CFG_NTP_PORT           123
#define CFG_HTTP_PORT          80

/* 链路 DOWN 超过该秒数后强制 stop/start 重建 PHY；0 = 不重建 */
#define CFG_ETH_RECOVER_SEC    30

/* ================== WT32-ETH01 以太网专用 GPIO（禁复用）==================
 * RMII 数据面 : IO21(TX_EN) IO19(TXD0) IO22(TXD1) IO27(CRS_DV)
 *               IO25(RXD0) IO26(RXD1) IO35(RX_ER)
 * SMI         : IO23(MDC)  IO18(MDIO)
 *
 * 【关键】WT32-ETH01 的 RMII 参考时钟不是 ESP32 产生的，而是板载 50MHz
 *         有源振荡器灌进 IO0 的：
 *           - IO16 = 该振荡器的使能脚，必须拉高，否则 PHY/MAC 无时钟；
 *                    官方资料里常被当成 "PHY power"，它同时也被许多例程
 *                    误当成 PHY nRST 使用——只要最终为高电平就恰好能用。
 *           - IO0  = 50MHz REF_CLK 输入 -> clock_mode 必须是 EMAC_CLK_EXT_IN。
 *             绝不能用 EMAC_CLK_OUT（ESP32 在 IO0 往外灌自己的 50MHz，
 *             会和板载振荡器在 IO0 上打架 -> RMII 数据错 -> link 反复翻动）。
 *           - PHY 的 nRST 在 WT32-ETH01 上没有接到任何 GPIO，
 *             所以 reset_gpio_num 必须是 -1；
 *             注意 ETH_PHY_DEFAULT_CONFIG() 默认 reset_gpio_num = 5，
 *             既违背硬件，又和 GPS 的 RXD(IO5) 冲突。
 *        同时 UART2 默认脚位是 IO16/IO17，已通过 uart_set_pin() 改到
 *        IO17/IO5，把 IO16 完全留给振荡器使能。
 * ------------------------------------------------------------------ */
#define CFG_ETH_MDC_GPIO       23
#define CFG_ETH_MDIO_GPIO      18
/* IO0: 外部 50MHz REF_CLK 输入 */
#define CFG_ETH_CLK_GPIO       0
#define CFG_ETH_CLK_MODE       EMAC_CLK_EXT_IN
/* IO16: 50MHz 振荡器使能（输出，恒高） */
#define CFG_ETH_OSC_EN_GPIO    16
/* PHY nRST 未接 GPIO */
#define CFG_ETH_PHY_RST_GPIO   (-1)
#define CFG_ETH_PHY_ADDR       1
/* 上电后使能振荡器到第一个 PHY 寄存器访问之间的等待（ms），启振 + 锁相 */
#define CFG_ETH_CLK_STABLE_MS  50

/* ================================ PPS ============================== */
#define CFG_PPS_GPIO           2          /* 硬件约束：PPS -> IO2 */
#define CFG_PPS_EDGE           GPIO_INTR_POSEDGE
#define CFG_PPS_ISR_FLAGS      (ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM)

/* 判定“PPS 有效”的最大间隔（ms），超过即认为失锁进入 holdover */
#define CFG_PPS_TIMEOUT_MS     2500
/* 判定“NMEA 定位有效”的最大间隔（ms） */
#define CFG_FIX_TIMEOUT_MS     5000

/* NMEA（RMC）与 PPS 边沿的配对策略
 *  MATCH : |RMC秒 - PPS推算秒| 在该窗口内即认定同一秒
 *  SNAP  : 命中但残差超过该值 -> 直接把锚点对齐到 RMC（残差太大，慢慢牵引没意义）
 *  SLIP  : 完全不匹配时，若残差的小数部分落在该容差内，判定为"整秒滑移"
 *          （初次对齐偏差 / 长时间失锁后晶振漂了整数秒），直接按 RMC 重新对齐，
 *          否则整机会卡死在"永远匹配不上、永远不纠正"的死循环里。 */
#define CFG_NMEA_MATCH_US      400000
#define CFG_NMEA_SNAP_US       50000
#define CFG_NMEA_SLIP_TOL_US   100000

/* NMEA 语句"开始发送"时刻相对其所属 PPS 边沿的滞后容差（µs）。
 * 模块在 PPS 之后才吐出该秒的 NMEA 突发，突发可能在 PPS 前一点就开始，
 * 也可能（9600 波特 + GSV 全开时突发超过 1 s）拖到下一个 PPS 之后，
 * 所以候选边沿取"不晚于 语句起始 + 该容差"的最新一条，以及它的前一条。 */
#define CFG_NMEA_LAG_TOL_US    400000
/* 滞后上限（µs）：超过它就说明配对到了明显过期的边沿（例如把 7 秒前的
 * PPS 当成本秒），强制按物理边沿重对齐。9600 下整个突发一般 < 1.5 s。 */
#define CFG_NMEA_MAX_LAG_US    2000000

/* ============================ GPS 串口 ============================== */
#define CFG_GPS_UART           UART_NUM_2
#define CFG_GPS_TX_GPIO        17         /* ESP TXD(IO17) -> GPS RX  */
#define CFG_GPS_RX_GPIO        5          /* ESP RXD(IO5)  <- GPS TX  */
#define CFG_GPS_UART_RX_BUF    4096
#define CFG_GPS_UART_TX_BUF    512
#define CFG_GPS_TASK_STACK     4096
#define CFG_GPS_TASK_PRIO      6

/* 自动适配的波特率候选（按探测顺序，从高到低）。
 * 先试 115200：高速率下 NMEA 突发最短，"RMC 跨秒"概率最低、授时最准；
 * 没收到任何有效语句再依次降速。多数模块出厂是 9600，探测几次就会命中。 */
#define CFG_GPS_BAUD_LIST      { 115200, 57600, 38400, 19200, 9600 }
/* 仅用于编译期一致性检查：实际遍历以数组长度为准（gps.c 的 GPS_BAUD_N）。
 * 改波特率列表时必须同步改这里，否则 _Static_assert 直接编译失败。 */
#define CFG_GPS_BAUD_COUNT     5
/* 每个波特率的探测窗口（ms），需 > 一个 NMEA 周期。
 * 命中即提前退出，不会每次都等满窗口。 */
#define CFG_GPS_BAUD_PROBE_MS  1500
/* 运行中"多久没有新增合法语句"就重新走一遍波特率探测（秒）。
 * 用于模块掉电重启（u-blox 回到出厂 9600）、被换过、或固件改了波特率
 * 之后的自愈；0 = 关闭该自愈。 */
#define CFG_GPS_RELOCK_SEC     15

/* ======= 是否改写 GPS 模块自身的配置（默认全部关闭）=======
 * 【已按需求改为 0】很多 GNSS 模块是 ROM 只读版（或厂商固件屏蔽了 CFG 写入），
 * 发 UBX-CFG-* 只会拿到 NAK；更糟的是 CFG-PRT 改波特率一旦生效而模块实际
 * 不支持，就会直接失联。因此默认"拿来就用，不改模块"：
 *   - 只做波特率自动探测（被动接收，不改任何东西）
 *   - 不下发 CFG-PRT / CFG-RATE / CFG-NAV5 / CFG-MSG / CFG-TP5
 * 需要精细调节时才把这两项打开（并且先确认模块枚举不是 ROM 版）。
 * ------------------------------------------------------------------ */

/* 是否用 UBX-CFG-PRT 把模块切到目标波特率（写操作） */
#define CFG_GPS_SET_BAUD       0
#define CFG_GPS_TARGET_BAUD    115200

/* 是否下发 UBX 配置报文 CFG-RATE / CFG-MSG / CFG-TP5 / CFG-NAV5（写操作） */
#define CFG_GPS_SEND_UBX_CFG   0

/* 是否周期性查询 UBX-NAV-TIMEGPS 取 leapS（只读轮询，不写配置，
 * 纯查询对 ROM 只读模块同样安全；模块不认识就收不到，不会报错） */
#define CFG_GPS_POLL_NAV_TIMEGPS   1
#define CFG_GPS_LEAP_POLL_MS       5000

/* 【生效条件】以下三项与 CFG_GPS_KEEP_GSV 都只在 CFG_GPS_SEND_UBX_CFG=1
 * 且模块支持 CFG 写入时才会下发给模块。默认 0 = 一个字节都不发，改这些值
 * 不会有任何效果（模块保持出厂配置），排障时先确认这一点。
 *
 * UBX-CFG-TP5：1 Hz、UTC 网格、上升沿为秒首、脉宽 200 ms。
 * 若发现 PPS 实际触发在下降沿（表现为恒定约 +200ms 偏差），
 * 把下面的 CFG_TP5_POLARITY_RISING 改成 0（需先打开 CFG_GPS_SEND_UBX_CFG）。 */
#define CFG_TP5_POLARITY_RISING    1
/* 天线馈线延迟（ns），可按实际馈线长度标定：约 5 ns/m（同上，需开启下发） */
#define CFG_TP5_ANT_CABLE_DELAY_NS 0
/* PPS 高电平脉宽（µs）。M8N 出厂 100000；加宽便于示波器/LED 观察，
 * 对上升沿时刻没有影响（同上，需开启下发）。 */
#define CFG_TP5_PULSE_LEN_US       200000

/* 是否保留 GSV（可见卫星数）。关掉可显著缩短 NMEA 突发长度，降低"RMC 跨秒"
 * 概率；卫星数量改用 GGA 的"参与解算卫星数"。
 * 【生效条件】本项通过 UBX-CFG-MSG 下发给模块，因此仅在 CFG_GPS_SEND_UBX_CFG=1
 * （且模块支持 CFG 写入）时才真正关闭 GSV；默认配置下模块仍按出厂设置输出
 * GSV，改这里没有效果。 */
#define CFG_GPS_KEEP_GSV       0

/* 判定“定位可用”的最小参与解算卫星数 */
#define CFG_GPS_MIN_SATS       4

/* ========================= 时基 / 伺服环 =========================== */
/* 每拍 PPS 相位修正比例（1/x）：越小越平滑、收敛越慢 */
#define CFG_PLL_PHASE_DIV      4
/* 锁相环积分增益：ppb += offset_us * CFG_PLL_FREQ_NUM / 1000 */
#define CFG_PLL_FREQ_NUM       4
/* 锁频环增益（%）：ppb += 频差估计 * CFG_FLL_GAIN_PCT / 100 */
#define CFG_FLL_GAIN_PCT       25
/* 捕获阶段（刚锁定的前 N 秒）使用更激进的参数 */
#define CFG_PLL_WARMUP_SEC     30
#define CFG_PLL_WARMUP_PHASE_DIV 1
#define CFG_PLL_WARMUP_FLL_PCT   50
/* 频率修正常数上限（ppb）。ESP32 无源晶振温漂一般 ±30 ppm */
#define CFG_PPB_LIMIT          100000
/* 相位误差超过该值（µs）直接阶跃，不再慢慢牵引 */
#define CFG_PLL_STEP_US        20000

/* ======================== NTP 报文固定修正 ========================= */
/* 入站：以太网帧交给驱动 -> 本程序打时戳 之间的固定偏差（µs），
 * 可用已知参考源标定后填入；0 = 不修正 */
#define CFG_NTP_RX_CORR_US     0
/* 出站：打时戳 -> 比特上线路 之间的固定偏差（µs） */
#define CFG_NTP_TX_CORR_US     0

/* 精度字段：esp_timer 分辨率为 1 µs -> 2^-20 s ≈ 0.95 µs */
#define CFG_NTP_PRECISION      (-20)

/* 未锁定 / 长期失锁时的 Root Dispersion 基准（µs） */
#define CFG_NTP_BASE_DISP_US   1000

/* Holdover 分级（秒） */
#define CFG_HOLD_STRATUM2_S    5      /* 超时 -> stratum 2  */
#define CFG_HOLD_UNSYNC_S      600    /* 超时 -> LI=3 + stratum 16 */

/* ===================== NTP 并发 / 抗突发 =========================== */
#define CFG_NTP_TASK_STACK     4096
/* NTP 任务的优先级必须低于 lwIP tcpip 线程（CONFIG_LWIP_TCPIP_TASK_PRIO，
 * 默认 18）：本任务靠 tcpip 线程把包投递到 socket，优先级反而更高会造成
 * 优先级反转 —— 收到包后先唤醒 NTP 任务、tcpip 线程被推迟，突发时收包更慢、
 * 抖动更大，正好伤害本项目最在意的指标。这里留 1 级余量。 */
#define CFG_NTP_TASK_PRIO      (CONFIG_LWIP_TCPIP_TASK_PRIO - 1)
/* SO_RCVBUF（字节），需要 CONFIG_LWIP_SO_RCVBUF=y */
#define CFG_NTP_SO_RCVBUF      16384
/* 令牌桶：整体限速，超出直接丢弃并计数，避免打爆 CPU */
#define CFG_NTP_RATE_PPS       400    /* 平均包/秒 */
#define CFG_NTP_RATE_BURST     800    /* 突发容量 */

/* 以太网入站时戳环形缓冲深度（NTP 帧）。
 * 每个入站 NTP 帧在驱动 hook 里入队一条时戳，消费侧严格按"每个包弹一条"
 * 与之对齐，所以深度至少要覆盖 lwIP 的 UDP 接收队列
 * （CONFIG_LWIP_UDP_RECVMBOX_SIZE）；太浅会在突发时挤出时戳（丢弃最旧），
 * 表现为 rx_ts_miss 增长。必须是 2 的幂。 */
#define CFG_RX_TS_RING         32

/* ====================== 双核分工（ESP32 双核）======================
 * CORE_TIME(CPU0 = PRO)：PPS 硬件中断、时基伺服、GPS 串口/UART 中断与 NMEA 解析。
 *     这一路全是"时间敏感"链路，必须独占一个核，避免被 lwIP/HTTP 的长临界区
 *     推迟中断响应（PPS 抖动直接变成 NTP 误差）。
 * CORE_NET (CPU1 = APP)：以太网 MAC 中断 + EMAC RX 任务、lwIP tcpip 线程、
 *     NTP socket 任务、HTTP 服务器。
 * 注意：中断归属由"安装它的那个任务所在核"决定，所以以太网初始化必须放在
 *     CORE_NET 上跑（见 main.c 的 eth_init_task），PPS/GPS 要放在 CORE_TIME 上跑。
 * ------------------------------------------------------------------ */
#ifndef CONFIG_FREERTOS_UNICORE
#define CFG_CORE_TIME          0
#define CFG_CORE_NET           1
#else
#define CFG_CORE_TIME          0
#define CFG_CORE_NET           0
#endif

/* ============================ 其它 ================================= */
#define CFG_LOG_INTERVAL_MS    5000   /* 串口诊断输出周期 */
#define CFG_WDT_TIMEOUT_MS     10000
#define CFG_MONITOR_STACK      6144   /* HTTP 服务器任务栈 */
#define CFG_MONITOR_PRIO       5

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
