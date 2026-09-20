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
 * 若确实需要静态地址：自行在 eth_if_init() 里 esp_netif_dhcpc_stop() +
 * esp_netif_set_ip_info()，并把 sdkconfig 的 DHCP 相关选项一并考虑。
 * ------------------------------------------------------------------ */
#define CFG_NTP_PORT           123
#define CFG_HTTP_PORT          80

/* 链路 DOWN 超过该秒数后强制 stop/start 重建 PHY；0 = 不重建 */
#define CFG_ETH_RECOVER_SEC    30

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
/* 滑移的最大幅度（秒）：超过它就不认"滑移"，而是判定时间源不可信、丢弃本次对齐。
 * 滑移是给"我们自己选错 ±1 秒边沿 / 失锁后晶振漂了整数秒"用的，不该无条件接受
 * 任意整数秒 —— 实测模块丢定位后时间漂了 3 整秒，钟面就被拉走了 3 秒。
 * 600 s 既挡得住"时间源完全乱掉"，又不会妨碍长时间失锁后的正常重新对齐。 */
#define CFG_NMEA_SLIP_MAX_SEC  600

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

/* ==================== GNSS 模块自愈（纯软件）========================
 * 用途：模块长时间拿不到定位时，发 UBX-CFG-RST 让它自己重启、重新搜星 ——
 * 丢定位或进了异常状态时，重启模块往往是唯一出路，比反复重探波特率彻底。 */

/* 复位方式：0=不用、1=热启动、2=温启动（默认）、3=冷启动。
 * 温启动只清星历、保留历书/位置/时间，重新定位比冷启动快得多；且只复位 GNSS
 * 子系统、不动 UART 配置（波特率保持），所以不需要重新握手。
 * 模块不支持这条指令（ROM 只读版 / 屏蔽 CFG 写入）时无副作用，只是没效果。 */
#define CFG_GPS_RST_MODE       2
/* 超过这么久仍拿不到合格定位（GGA fix quality ∈ [MIN,MAX] 且卫星数达标）→
 * 复位模块。单位秒，0 = 关闭该自愈。默认 10 分钟。 */
#define CFG_GPS_REFIX_SEC      600
/* 连续复位多少次仍无 fix 就停止（0 = 不限次数）。
 * 设上限是为了避免"天线坏 / 室内无信号"时无休止折腾模块 —— 反复复位/冷启动
 * 只会让定位更难。一旦重新拿到定位，计数清零，下次超时仍会重试。 */
#define CFG_GPS_REFIX_MAX      3

/* 上电握手成功后，是否用 UBX-CFG-PRT 把模块往高波特率上提（写操作）。
 * 做法：从 CFG_GPS_BAUD_LIST 里挑 "高于当前 且 不超过 CFG_GPS_TARGET_BAUD" 的
 * 波特率，从高到低逐个尝试；判据**不是**模块的 ACK（实测 AF68GBR 这类中科微系
 * 兼容模块在 38400 下根本回不上 ACK，但指令其实是生效的 —— 拿 ACK 当门槛会把
 * 能用的模块判死），而是"本机切过去之后还能不能收到有效报文"。第一个成功的就
 * 留在那里，全失败则保持原波特率（每次失败都会自动切回来）。
 *   为什么值得这么做：9600 下一个 NMEA 突发要传 0.6~0.7 s，RMC 相对 PPS 的滞后
 *   （日志里的 lag）就有 600+ ms；115200 下同样内容只需 ~50 ms，lag 直接降一个
 *   数量级，配对窗口与授时裕度都宽松得多。
 * 全程失败也是安全的：模块不认就不切，切了收不到就切回原速率。
 * 其余改写模块的项（CFG-RATE / CFG-MSG / CFG-TP5 / CFG-NAV5）仍默认关闭，
 * 见 CFG_GPS_SEND_UBX_CFG。 */
#define CFG_GPS_SET_BAUD       1          /* 是否尝试提速（写操作） */
/* 提速上限：只尝试比当前高、且不超过此值的波特率（0 = 不限，取表里最高） */
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

/* 判定"定位可用"的最小参与解算卫星数 */
#define CFG_GPS_MIN_SATS       4

/* 判定"定位可用"的 GGA fix quality 允许范围。
 * NMEA 定义：0=无效 1=SPS 2=DGPS 3=PPS 4=RTK固定 5=RTK浮动
 *            6=推算(estimated) 7=手动 8=模拟
 * 推算/模拟定位不具备授时资格（模块自己也会在 GLL 里用 'E' 标注），只认 1~5。
 * 注意：gps.c 的 fix_valid 与 discipline.c 的定位判据共用这两个宏，改一处即可。 */
#define CFG_GPS_FIXQ_MIN       1
#define CFG_GPS_FIXQ_MAX       5

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

/* ======================= Root Dispersion 构成 ======================
 * 对外宣告的 Root Dispersion（RFC 5905 的 ε）由三部分组成：
 *
 *   Root Dispersion = CFG_NTP_BASE_DISP_US        （打戳/路径的固定不确定度）
 *                   + 2 × jitter                  （PPS 相位残差 EMA，约 2σ）
 *                   + 守时漂移（仅 PPS 不新鲜时按 CFG_HOLD_DRIFT_PPM 累积）
 *
 * 它直接决定客户端看到的 Root Distance（≈ Root Delay/2 + Root Dispersion），
 * 而 Root Distance 是 clock_select 排序、剔除（ntpd 的 TEST11 / MAXDIST
 * = 1.5 s）与 chrony 选源（maxdistance，默认 3 s）的判据，调大它等于主动
 * 降低自己作为时间源的可用性，别为了"保守"随手加。
 * ------------------------------------------------------------------ */

/* 固定不确定度基准（µs）：esp_timer 分辨率、以太网驱动打戳、lwIP 路径、
 * PPS 边沿->整秒对齐合起来的量级，1 ms 是这一档实现的合理取值。 */
#define CFG_NTP_BASE_DISP_US   1000

/* Root Dispersion 封顶（µs）= 10 s。用于"从未建立绝对时间基准/长时间失锁"
 * 这类确实不可信的状态 —— 这个值本身就是"别选我"的信号，不要改小。 */
#define CFG_NTP_DISP_MAX_US    10000000u

/* 守时（holdover）漂移率，ppm：1 ppm == 1 µs/s。PPS 丢失后钟面按此速率
 * 累积误差，Root Dispersion 随之线性增长（1000 s -> 10 ms）。
 * ESP32 无源 40 MHz 晶振初始精度约 ±10 ppm、温漂约 ±20 ppm；进入 holdover
 * 时 s_ppb 已吸收了当时的频差，残余主要来自温度变化，10 ppm 属保守估计。
 * 调大 = 更早被判不可信；调小 = 更乐观。 */
#define CFG_HOLD_DRIFT_PPM     10

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

/* ================ GNSS 原始报文 TCP 转发（远程"串口调试助手"）==============
 * 把 GNSS 模块吐在串口上的原始字节流原样转发到一个 TCP 端口：串口调试助手 /
 * telnet / nc 远程连上来，看到的就是模块的真实输出 —— NMEA 文本行、模块对
 * UBX 查询的二进制应答、校验失败的坏帧，乃至波特率探测期间收到的乱码。
 * 作用：判断"模块的工作健康状况"。面板上的统计是结论，这里是一手证据；
 * 两者合看才能分清"模块拿不到定位"和"模块/串口根本没在工作"。
 *
 * 数据面全在 CORE_NET 上的独立任务里；GPS 任务侧只多一次无锁 memcpy（写环形
 * 缓冲），不阻塞、不取信号量，因此不影响 PPS/NMEA 时基（见 gnss_tcp.c）。
 * 服务只出不进：不解析客户端发来的任何字节（收到即丢弃，仅用于探测断开）。
 *
 * 注意：客户端数要占用 lwip socket，改大这里时留意 sdkconfig 的
 * CONFIG_LWIP_MAX_SOCKETS 是否还够（见 sdkconfig.defaults）。
 * ------------------------------------------------------------------ */
#define CFG_GNSS_TCP_ENABLE        1      /* 0 = 完全关闭（连字节投递都不做） */
#define CFG_GNSS_TCP_PORT          8880
#define CFG_GNSS_TCP_MAX_CLIENTS   3      /* 同时连接的客户端数上限 */
/* 环形缓冲字节数（必须是 2 的幂）：115200 下一个 NMEA 突发约 1.5 KB，4096 B
 * 足以吸收一次瞬时拥塞；再大只是白占 RAM。缓冲放不下时丢"最新"字节并计数。 */
#define CFG_GNSS_TCP_RING          4096
#define CFG_GNSS_TCP_TX_CHUNK      1024   /* 单次发送块大小（任务栈上的缓冲） */
#define CFG_GNSS_TCP_POLL_MS       20     /* 轮询周期：报文转发延迟的上限 */
/* 任务栈：含 CFG_GNSS_TCP_TX_CHUNK 的发送缓冲与连接提示，另留 lwIP 调用深度 */
#define CFG_GNSS_TCP_TASK_STACK    5120
/* 与 NTP 任务同理，刻意低于 lwIP tcpip 线程，避免优先级反转 */
#define CFG_GNSS_TCP_TASK_PRIO     (CONFIG_LWIP_TCPIP_TASK_PRIO - 1)

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
