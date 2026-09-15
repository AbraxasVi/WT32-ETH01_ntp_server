# WT32-ETH01 GPS/PPS NTP Server

一个基于**ESP-IDF v6.1**的GPS驯服（GPS-disciplined）NTP时间服务器固件，运行于**WT32-ETH01**（ESP32-D0WD 双核 + LAN8720 RMII 以太网）开发板，配合任意带**PPS**秒脉冲输出的GNSS模块（中科微AT6558、u-blox M7N / M8N 等）。将GNSS的PPS + NMEA作为时间基准，对外提供**Stratum 1**级NTPv4（RFC 5905）服务，并内置一个实时 Web 监控面板。

> 主打一个低成本，闲置设备的复用、拿来即用、不改模块。固件默认**不向GNSS写入任何配置**，纯被动接收NMEA / PPS，兼容ROM只读版与厂商屏蔽CFG写入的模块。
> 但也不要过于依赖这玩意儿的可靠性，毕竟便宜没好货，家用环境下用用么问题不大，大型网络环境中就别用这东西了，哪怕你找个老态龙钟的树莓派2来做chrony服务器，可靠性也要比ESP32好得多，理论上来说，ESP32C3或S3之类的无线模块也能用，改改网络接口就行了，但非常不建议，有线网络的延时尚且可控，无线网络太坑了。

---

## 功能特性

- **高精度时基**：PPS 由 GPIO 硬件中断（IRAM、`LEVEL3`）捕获，时标取自  `esp_timer` 64 位微秒计数，无 32 位回绕；“时间敏感链路”独占 CPU0。
- **PLL + FLL 联合驯服**：每拍 PPS 即一个相位采样点，积分环（PLL）补偿相位、频差环（FLL）跟踪晶振温漂，频率修正常数上限 ±100 ppm。
- **Holdover 分级**：PPS 失锁后按时间自动降级 —— 锁定 → Stratum 2 →  Stratum 16 / LI=3（不同步），期间按最后估计频率自由运行，不丢秒。
- **NMEA / PPS 自动配对**：内置“整秒滑移”自恢复，长失锁或初次对齐偏差后不会卡死在永不匹配死循环；支持 9600~115200 波特率下 RMC 跨秒、突发滞后等真实场景。
- **GNSS 波特率自动探测**：优先 115200（突发最短、授时最准），收不到有效语句再依次降到 57600 → 38400 → 19200 → 9600；纯被动监听，不写模块。
- **多源闰秒**：只读轮询 UBX-NAV-TIMEGPS 的 `leapS`，并与内置闰秒表交叉校验。
- **完整 NTPv4 报文**：Stratum / Root Dispersion / Precision / Leap Indicator 全部按锁定状态动态填写；独立任务 + BSD socket + `SO_RCVBUF` + 令牌桶限速，抗突发查询（任务优先级刻意低于 lwIP tcpip 线程，避免优先级反转）。
- **抗反射 / 抗误用**：只响应 mode 3（标准客户端）；源地址属于 `0.0.0.0/8`、组播或保留网段的请求直接丢弃；尚未建立绝对时间基准时**静默不回包**（不会发出 transmit 时戳为 0 的非法响应，该计数显示为 `unsync`）。
- **双核分工**：CPU0 = 时基 / PPS / GPS（时间敏感，独占）；CPU1 = 以太网 / lwIP / NTP / HTTP（与 lwIP tcpip 线程同核）。
- **以太网健壮性**：DHCP 获取地址；自定义 input path 抓 NTP 入站帧到达时刻；断链超时强制重建 PHY，暴露 link UP 计数以观察是否仍在翻动。
- **Web 监控面板**：实时显示锁定状态、相位误差、频率修正、卫星数、Holdover 等，并保留最近15 条 GNGGA / GNGSA / GNZDA 原始报文便于排障。
- **任务看门狗（TWDT）**：10s 无心跳即 panic 复位，防止静默卡死。

---

## 硬件连接（GND和VCC根据模块型号连接即可）

| 信号        | WT32-ETH01 脚位            | GNSS 模块 / 网络                | 备注                                   |
|-------------|----------------------------|--------------------------------|----------------------------------------|
| **PPS**     | IO2（输入，上升沿中断）     | GNSS `TIMEPULSE` / `PPS` 输出   | 硬件约束，固定接 IO2                    |
| **GPS TX**  | ESP `IO17`（TXD2）         | → GNSS `RX`                    | ESP 发往模块                            |
| **GPS RX**  | ESP `IO5`（RXD2）          | ← GNSS `TX`                    | ESP 接收模块                           |

> WT32-ETH01 的 RMII 参考时钟来自板载 50 MHz 振荡器灌入 IO0，因此 `clock_mode` 必须是 `EMAC_CLK_EXT_IN`。若误用 `EMAC_CLK_OUT`，ESP32 会在 IO0 上往外灌自己的 50 MHz，与板载振荡器打架 → RMII 数据错 → link 反复翻动。
> 另外 IO16 是振荡器使能脚（常被误当 PHY nRST），PHY 的 nRST 在板上并没接到任何 GPIO。

网络地址：**DHCP 自动获取**。

---

## 软件架构（双核分工）

```
                ESP32-D0WD (240 MHz, 双核)
 ┌──────────────────────────┐      ┌──────────────────────────┐
 │  CPU0 (CORE_TIME / PRO)  │      │  CPU1 (CORE_NET  / APP)  │
 │  • PPS GPIO ISR (IRAM)   │      │  • 以太网 EMAC 中断+RX任务│
 │  • 时基伺服 PLL/FLL       │      │  • lwIP tcpip 线程        │
 │  • GPS UART + NMEA 解析   │      │  • NTP socket 任务        │
 │                          │      │  • HTTP 监控服务器         │
 └──────────────────────────┘      └──────────────────────────┘
         ▲  PPS / NMEA                    ▲  ETH / NTP / HTTP
         │  (时间敏感，独占)              │  (与 tcpip 同核)
```

中断归属由"安装它的任务所在核"决定，因此以太网初始化放在 `CORE_NET` 上跑
（`main.c` 的 `eth_init_task`），PPS/GPS 放在 `CORE_TIME` 上跑。

模块划分（`main/`）：

| 文件           | 职责                                                         |
|----------------|--------------------------------------------------------------|
| `main.c`       | 入口、双核初始化编排、TWDT                                   |
| `config.h`     | 全部硬件 / 策略配置集中处                                    |
| `discipline.c` | 时基、PPS 捕获、PLL/FLL 伺服、Holdover、NTP 时间戳           |
| `gps.c`        | 串口接入、波特率探测、NMEA(RMC/GGA/GSV) 解析、闰秒轮询、原始报文留痕 |
| `ntp.c`        | NTPv4 服务器（RFC 5905）、令牌桶、入站时戳                   |
| `eth_if.c`     | LAN8720 RMII 驱动、DHCP、断链重建、入站帧时戳                |
| `monitor.c`    | Web 状态面板 + 串口诊断输出                                  |

---

## 构建与烧录

### 前置条件

- [ESP-IDF v6.1](https://docs.espressif.com/projects/esp-idf/zh_CN/v6.1/esp32/) 环境已安装并 `export` 到当前 shell。
- 目标芯片：`esp32`（WT32-ETH01 为 ESP32-D0WD）。
- 主要依赖组件已在 `main/CMakeLists.txt` 的 `REQUIRES` 中声明，IDF 自带： `esp_eth` `esp_netif` `esp_event` `esp_http_server` `esp_timer` `esp_driver_gpio` `esp_driver_uart` `lwip` 等。

### 步骤

```bash
# 1. 进入工程目录
cd <本仓库目录>

# 2. 设置目标芯片（首次需要）
idf.py set-target esp32

# 3. 配置（可选）：用 menuconfig 调整，或直接使用仓库内置 sdkconfig.defaults
idf.py build

# 4. 烧录并打开监视器
```

> 若修改了 `sdkconfig.defaults` 想让默认值重新生效，删除 `sdkconfig` 后重新 `idf.py set-target esp32 && idf.py build` 即可。

### 关键 sdkconfig 项（已写入 `sdkconfig.defaults`）

- `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y`：主频 240 MHz，降低中断抖动。
- `CONFIG_LWIP_TCPIP_CORE_LOCKING=y` / `CORE_LOCKING_INPUT=y`：NTP 任务可直接在自身
  上下文调用 lwIP，时戳更贴近实际上线路时刻。
- `CONFIG_LWIP_UDP_RECVMBOX_SIZE=32`：UDP 接收队列加深，避免突发丢包。
- `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU1=y`：lwIP tcpip 线程固定到 CPU1（网络核）。
- `CONFIG_LWIP_SO_RCVBUF=y`：允许 `setsockopt(SO_RCVBUF)`。
- `CONFIG_ESP_TASK_WDT_EN=y` / `INIT=n`：由 `app_main()` 显式 `esp_task_wdt_init()`。

---

## 配置项速查（`main/config.h`）

大多数行为可通过修改 `config.h` 调整，无需改代码逻辑：

| 宏                              | 默认   | 说明                                              |
|---------------------------------|--------|---------------------------------------------------|
| `CFG_NTP_PORT` / `CFG_HTTP_PORT`| `123`/`80` | NTP 与 Web 端口                                |
| `CFG_PPS_GPIO` / `CFG_PPS_EDGE` | `2`/上升沿 | PPS 输入脚与边沿                                |
| `CFG_GPS_TX_GPIO` / `RX_GPIO`   | `17`/`5` | GPS 串口脚位                                     |
| `CFG_GPS_BAUD_LIST`             | `115200,57600,38400,19200,9600` | 探测顺序（高→低）                     |
| `CFG_GPS_RELOCK_SEC`            | `15`   | 多久没有合法语句就重探波特率（秒，0=关闭）         |
| `CFG_GPS_SET_BAUD`              | `0`    | 是否用 UBX-CFG-PRT 改模块波特率（写）              |
| `CFG_GPS_SEND_UBX_CFG`          | `0`    | 是否下发 UBX 配置（写）                            |
| `CFG_GPS_POLL_NAV_TIMEGPS`      | `1`    | 只读轮询 UBX-NAV-TIMEGPS 取闰秒                    |
| `CFG_GPS_KEEP_GSV`              | `0`    | 是否保留 GSV（关掉可缩短突发、降低跨秒概率）       |
| `CFG_GPS_MIN_SATS`              | `4`    | 判定定位可用的最小参与解算卫星数                   |
| `CFG_PPS_TIMEOUT_MS`            | `2500` | 超此时间未见 PPS → 进入 holdover                   |
| `CFG_PLL_PHASE_DIV` / `FLL_GAIN_PCT` | `4`/`25` | 伺服环相位/频差增益                        |
| `CFG_PPB_LIMIT`                | `100000` | 频率修正常数上限（ppb，±100 ppm）               |
| `CFG_NTP_SO_RCVBUF`             | `16384` | NTP socket 接收缓冲                              |
| `CFG_NTP_RATE_PPS` / `BURST`    | `400`/`800` | 令牌桶平均/突发限速                          |
| `CFG_CORE_TIME` / `CFG_CORE_NET`| `0`/`1` | 双核分工（UNICORE 下均为 0）                    |

---

## Web 监控面板

设备通过 DHCP 拿到 IP 后，浏览器访问 `http://<设备IP>/` 即可。页面每 2 秒自动刷新，
包含：

- **锁定状态 / Stratum / Leap Indicator / Root Dispersion**
- **相位误差（offset）**、**抖动（jitter）**、**频率修正（ppb）**
- **PPS 计数 / 漏拍 / Holdover 时长**
- **GNSS 卫星数（参与解算 / 可见）**、Fix Quality、HDOP、当前波特率
- **伺服事件计数**：step / resync / slip / mismatch / late（排障用，持续上涨说明对时链路有问题）
- **最近 15 条原始报文**：GNGGA / GNGSA / GNZDA，便于核对定位与授时是否正常

状态数据同时以 JSON 提供：`http://<设备IP>/status.json`（供脚本/监控系统集成）。

---

## NTP 客户端配置示例

设备锁定后即为 **Stratum 1**。在 Linux 上把它加入 NTP 源：

```bash
# /etc/chrony/chrony.conf 或 /etc/ntpsec/ntp.conf
server <设备IP> iburst minpoll 3 maxpoll 4 prefer
```

OpenWRT路由器也可以直接在DHCP设置中添加Option 42来指定内网ntp服务器IP，大部分设备包括IoT设备都会自动识别并获取。

验证：

```bash
ntpq -p            # 应能看到本设备，stratum 1，offset 通常在几十 µs 内
# 或
chronyc sources -v
```

---

## 串口诊断输出

`idf.py monitor` 每 `CFG_LOG_INTERVAL_MS`（默认 5s）由 `monitor.c` 打印一行诊断。
下面是一台**已锁定**设备的真实样例（数值节选自实际运行日志）：

```
I (48026284) mon: LINK UP 192.168.6.201 | UTC 2026-09-15 04:00:31 | LOCKED  stratum=1 li=0 | off=0us jit=1us ppb=5004 | hold=956ms pps=189414/0 | sats=9/11 fixq=1 hdop=2.5 [FIX] | baud=9600 cfg=0 | ntp req=8 resp=8 unsync=0 bad=0 drop=0 txfail=0 | rxTs 8/0 | linkup=1 | sv 1/1/2/1/128 | lag=683ms | heap=217KB
```

逐字段含义：

| 片段 | 含义 |
|------|------|
| `LINK UP 192.168.6.201` | 以太网链路状态与 IP（DHCP 获取） |
| `UTC 2026-09-14 15:37:37` | 当前 UTC；未同步时显示 `未同步` |
| `LOCKED stratum=1 li=0` | 锁定状态 / NTP 层级 / 闰秒指示符（LI） |
| `off=0us jit=1us ppb=5004` | PPS 相位误差 / 抖动 / 频率修正（ppb） |
| `hold=956ms pps=189414/0` | Holdover 时长（未见过 PPS 时为 `n/a(无PPS)`）/ PPS 累计与丢失 |
| `sats=9/11 fixq=1 hdop=2.5 [FIX]` | 卫星（**参与解算 / 可见**）/ Fix Quality / HDOP / 定位可用 |
| `baud=9600 cfg=0` | 当前 GNSS 波特率（自动探测结果）/ 已下发的 UBX 配置条数（默认配置下恒为 0） |
| `ntp req=8 resp=8 unsync=0 bad=0 drop=0 txfail=0` | NTP 请求 / 应答 / **未同步未应答** / 非法报文 / 被令牌桶丢弃 / 发送失败 |
| `rxTs 8/0` | 使用了驱动层入站硬件时戳的次数 / 回退到 socket 时刻的次数 |
| `linkup=1` | 累计 link UP 次数（持续上涨说明链路仍在翻动） |
| `sv 1/1/2/1/128` | 伺服事件：阶跃 / 重对齐(RMC) / 整秒滑移 / RMC 失配 / 传输跨秒 |
| `lag=683ms` | NMEA 语句起始相对其 PPS 边沿的滞后 |
| `heap=217KB` | 空闲堆内存 |

> 说明：上述 `off=0us`、`ppb=5004`、相位抖动 `jit=1us`、PPS 稳定累计说明设备已处于健康锁定状态，授时精度在微秒级。
> `unsync` 是"尚未建立绝对时间基准、按 RFC 5905 不回包"的请求数（锁定后应恒为 0，只在冷启动未对齐期间增长）；
> `sv` 最后一位（`late`）在 9600 波特下缓慢增长属常态 —— 那是 NMEA 突发跨秒时的正常配对回退，不是故障。

客户机实测，精度还行

> ❯ sntp -d 192.168.6.201
> sntp 4.2.8p18@1.4062-o Thu Oct 23 00:08:33 UTC 2025 (1)
> kod_init_kod_db(): Cannot open KoD db file /var/db/ntp-kod: No such file or directory
> sntp auth_init: Couldn't open key file /etc/ntp.keys for reading!
> handle_lookup(192.168.6.201,0x2)
> move_fd: estimated max descriptors: 2048, initial socket boundary: 16
> generate_pkt: key_id -1, key pointer (nil)
> sntp sendpkt: Sending packet to 192.168.6.201:123 ...
> Packet sent.
> sock_cb: 192.168.6.201 192.168.6.201:123
> 2026-09-14 23:05:24.566922 (-0800) -0.041843 +/- 0.028978 192.168.6.201 s1 no-leap

---

## 排障小贴士

- **link 反复 UP/DOWN**：确认 IO16 已拉高（振荡器使能）、`EMAC_CLK_EXT_IN` 已选、 `reset_gpio_num = -1`（PHY nRST 未接 GPIO）。
- **相位误差恒为 ~+200 ms**：说明 PPS 实际在下降沿触发。把 `config.h` 的 `CFG_TP5_POLARITY_RISING` 改为 `0` —— **注意这需要 `CFG_GPS_SEND_UBX_CFG=1` 且模块支持 CFG 写入**；默认 0 时该值根本不会下发，改了也没有效果（见 `config.h` 里的生效条件说明）。
- **RMC 与 PPS 一直对不上**：检查 GNSS 波特率是否被识别（`baud_locked`）、 GSV 是否过长导致跨秒（关 `CFG_GPS_KEEP_GSV`，同样**仅在 `CFG_GPS_SEND_UBX_CFG=1` 时生效**），以及定位是否达标
  （`sats_used >= CFG_GPS_MIN_SATS`）。
- **串口偶发 `N 秒未收到合法 NMEA，重新探测波特率`**：这是模块掉电重启 / 被换 / 波特率被改之后的自愈动作（`CFG_GPS_RELOCK_SEC`，默认 15 s）。探测是只读的、失败会恢复原波特率；嫌频繁可调大该值，设 0 关闭。
- **HTTP 页面读取失败**：查看 `/status.json` 是否为合法 JSON，先用串口日志确认服务已起。

---

## 许可证

本项目仅用于学习与研究。固件依赖的 ESP-IDF 与第三方组件各自遵循其原有许可证。

---

## 致谢

- Espressif **ESP-IDF** 与 **LAN8720** RMII 驱动。
- NTP 协议参考 **RFC 5905**。
- GNSS 时间基准参考 u-blox NMEA / UBX 协议。

---
---

# WT32-ETH01 GPS/PPS NTP Server (English)

A **GPS-disciplined NTP time server** firmware built on **ESP-IDF v6.1**, running on the
**WT32-ETH01** board (ESP32-D0WD dual-core + LAN8720 RMII Ethernet) together with any GNSS
module that outputs a **PPS** (pulse-per-second) signal (e.g. u-blox M8 / M8N). It uses the
GNSS PPS + NMEA as the time reference and serves **Stratum 1** NTPv4 (RFC 5905) to the network,
with a built-in real-time Web monitoring panel.

> Design goal: plug-and-play, no module modification. By default the firmware **writes no
> configuration to the GNSS module** — it passively receives NMEA / PPS only, which keeps it
> compatible with ROM-read-only modules and modules whose CFG writes are blocked by the vendor.

---

## Features

- **High-precision time base**: PPS is captured by a GPIO hardware ISR (IRAM, `LEVEL3`); the
  timestamp comes from `esp_timer`'s 64-bit microsecond counter (no 32-bit wraparound); the
  "time-sensitive" path is pinned to CPU0 exclusively.
- **PLL + FLL joint discipline**: every PPS edge is a phase sample; the integral loop (PLL)
  corrects phase and the frequency-difference loop (FLL) tracks crystal temperature drift.
  Frequency correction clamp is ±100 ppm.
- **Graded Holdover**: when PPS is lost, the device degrades over time — Locked → Stratum 2 →
  Stratum 16 / LI=3 (unsynchronized) — and keeps free-running on the last estimated frequency
  without losing seconds.
- **Automatic NMEA / PPS pairing**: built-in "integer-second slip" self-recovery so the device
  won't deadlock in a never-match loop after long outages or large initial alignment errors;
  handles real-world cases like RMC crossing a second boundary and burst lag at 9600~115200 baud.
- **GNSS baud-rate auto-detection**: tries 115200 first (shortest burst, most accurate timing),
  then falls back to 57600 → 38400 → 19200 → 9600 if no valid sentence is seen; passive listening
  only, no writes to the module.
- **Multi-source leap second**: polls UBX-NAV-TIMEGPS `leapS` read-only and cross-checks it
  against a built-in leap-second table.
- **Full NTPv4 (RFC 5905)**: Stratum / Root Dispersion / Precision / Leap Indicator are filled
  dynamically from the lock state; a dedicated task + BSD socket + `SO_RCVBUF` + token-bucket rate
  limiting absorb burst queries (its priority is deliberately kept below the lwIP tcpip thread to
  avoid priority inversion).
- **Anti-reflection / misuse hardening**: answers mode 3 (standard clients) only; drops requests from
  `0.0.0.0/8`, multicast and reserved source ranges; stays **silent until an absolute time base
  exists** (never emits a response with a zero transmit timestamp — counted as `unsync`).
- **Dual-core split**: CPU0 = time base / PPS / GPS (time-sensitive, exclusive); CPU1 = Ethernet
  / lwIP / NTP / HTTP (same core as the lwIP tcpip thread).
- **Ethernet robustness**: DHCP address; custom input path captures the NTP inbound frame
  arrival time; forced PHY rebuild on link-down timeout, exposing a link-UP counter to spot
  flapping.
- **Web monitoring panel**: live lock state, phase error, frequency correction, satellite count,
  holdover, etc., plus the last **15 raw GNGGA / GNGSA / GNZDA sentences** for troubleshooting.
- **Task Watchdog (TWDT)**: panics and reboots after 10s without a heartbeat, preventing silent hangs.

---

## Hardware Connections

| Signal       | WT32-ETH01 pin            | GNSS module / network           | Notes                                  |
|--------------|---------------------------|---------------------------------|----------------------------------------|
| **PPS**      | IO2 (input, rising edge)  | GNSS `TIMEPULSE` / `PPS` output | Hardware constraint: fixed to IO2      |
| **GPS TX**   | ESP `IO17` (TXD2)         | → GNSS `RX`                     | ESP transmits to module                |
| **GPS RX**   | ESP `IO5` (RXD2)          | ← GNSS `TX`                     | ESP receives from module               |

> WT32-ETH01's RMII reference clock comes from the
> on-board 50 MHz oscillator fed into IO0, so `clock_mode` must be `EMAC_CLK_EXT_IN`. If you
> mistakenly use `EMAC_CLK_OUT`, the ESP32 drives its own 50 MHz onto IO0, fighting the on-board
> oscillator → corrupted RMII data → link flapping. Also, IO16 is the oscillator enable pin (often
> mistaken for PHY nRST); the PHY's nRST is not wired to any GPIO on this board.

Network address: **DHCP**.

---

## Software Architecture (Dual-Core)

```
                ESP32-D0WD (240 MHz, dual-core)
 ┌──────────────────────────┐      ┌──────────────────────────┐
 │  CPU0 (CORE_TIME / PRO)  │      │  CPU1 (CORE_NET  / APP)  │
 │  • PPS GPIO ISR (IRAM)   │      │  • Ethernet EMAC IRQ + RX task │
 │  • Time-base PLL/FLL     │      │  • lwIP tcpip thread      │
 │  • GPS UART + NMEA parse │      │  • NTP socket task        │
 │                          │      │  • HTTP monitor server    │
 └──────────────────────────┘      └──────────────────────────┘
         ▲  PPS / NMEA                    ▲  ETH / NTP / HTTP
         │  (time-sensitive, exclusive)  │  (same core as tcpip)
```

Interrupt affinity is decided by the core that installs it, so Ethernet init runs on `CORE_NET`
(`eth_init_task` in `main.c`) and PPS/GPS run on `CORE_TIME`.

Module layout (`main/`):

| File        | Responsibility                                                |
|-------------|---------------------------------------------------------------|
| `main.c`    | Entry point, dual-core init orchestration, TWDT               |
| `config.h`  | All hardware / policy configuration in one place             |
| `discipline.c` | Time base, PPS capture, PLL/FLL servo, Holdover, NTP timestamp |
| `gps.c`     | UART, baud detection, NMEA (RMC/GGA/GSV) parse, leap-second poll, raw sentence ring |
| `ntp.c`     | NTPv4 server (RFC 5905), token bucket, inbound timestamp      |
| `eth_if.c`  | LAN8720 RMII driver, DHCP, link-down rebuild, inbound frame timestamp |
| `monitor.c` | Web status panel + serial diagnostics                        |

---

## Build & Flash

### Prerequisites

- [ESP-IDF v6.1](https://docs.espressif.com/projects/esp-idf/en/v6.1/esp32/) installed and
  `export`ed into the current shell.
- Target chip: `esp32` (WT32-ETH01 uses ESP32-D0WD).
- Dependencies are declared in `main/CMakeLists.txt` `REQUIRES` and ship with IDF:
  `esp_eth` `esp_netif` `esp_event` `esp_http_server` `esp_timer`
  `esp_driver_gpio` `esp_driver_uart` `lwip`, etc.

### Steps

```bash
# 1. Enter the project directory
cd <this repo>

# 2. Set the target chip (first time only)
idf.py set-target esp32

# 3. Configure (optional): use menuconfig, or just rely on the bundled sdkconfig.defaults
idf.py build

# 4. Flash and open the monitor 
```

> If you changed `sdkconfig.defaults` and want the defaults to take effect, delete `sdkconfig`
> then run `idf.py set-target esp32 && idf.py build` again.

### Key sdkconfig items (in `sdkconfig.defaults`)

- `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y`: 240 MHz to reduce interrupt jitter.
- `CONFIG_LWIP_TCPIP_CORE_LOCKING=y` / `CORE_LOCKING_INPUT=y`: the NTP task can call lwIP in its
  own context, so timestamps stay closer to the real on-wire moment.
- `CONFIG_LWIP_UDP_RECVMBOX_SIZE=32`: deeper UDP receive queue to avoid burst packet loss.
- `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU1=y`: lwIP tcpip thread pinned to CPU1 (network core).
- `CONFIG_LWIP_SO_RCVBUF=y`: allow `setsockopt(SO_RCVBUF)`.
- `CONFIG_ESP_TASK_WDT_EN=y` / `INIT=n`: `app_main()` calls `esp_task_wdt_init()` explicitly.

---

## Configuration Quick Reference (`main/config.h`)

Most behavior can be tuned in `config.h` without touching logic:

| Macro                          | Default | Description                                    |
|--------------------------------|---------|------------------------------------------------|
| `CFG_NTP_PORT` / `CFG_HTTP_PORT` | `123`/`80` | NTP and Web ports                        |
| `CFG_PPS_GPIO` / `CFG_PPS_EDGE` | `2`/rising | PPS input pin and edge                      |
| `CFG_GPS_TX_GPIO` / `RX_GPIO`  | `17`/`5` | GPS UART pins                                |
| `CFG_GPS_BAUD_LIST`            | `115200,57600,38400,19200,9600` | Probe order (high→low)        |
| `CFG_GPS_RELOCK_SEC`           | `15`    | Re-probe baud after N s with no valid sentence (0=off) |
| `CFG_GPS_SET_BAUD`             | `0`     | Write module baud via UBX-CFG-PRT (write)      |
| `CFG_GPS_SEND_UBX_CFG`         | `0`     | Send UBX config (write)                        |
| `CFG_GPS_POLL_NAV_TIMEGPS`     | `1`     | Read-only poll UBX-NAV-TIMEGPS for leap second |
| `CFG_GPS_KEEP_GSV`             | `0`     | Keep GSV (off shortens burst, lowers cross-second risk) |
| `CFG_GPS_MIN_SATS`             | `4`     | Min satellites-in-solution for a valid fix      |
| `CFG_PPS_TIMEOUT_MS`           | `2500`  | No PPS beyond this → enter holdover            |
| `CFG_PLL_PHASE_DIV` / `FLL_GAIN_PCT` | `4`/`25` | Servo phase / frequency gain          |
| `CFG_PPB_LIMIT`                | `100000` | Frequency correction clamp (ppb, ±100 ppm)    |
| `CFG_NTP_SO_RCVBUF`            | `16384` | NTP socket receive buffer                      |
| `CFG_NTP_RATE_PPS` / `BURST`   | `400`/`800` | Token-bucket avg / burst rate              |
| `CFG_CORE_TIME` / `CFG_CORE_NET` | `0`/`1` | Dual-core split (both 0 under UNICORE)     |

---

## Web Monitoring Panel

Once the device gets an IP via DHCP, open `http://<device-ip>/` in a browser. The page auto-refreshes
every 2 seconds and shows:

- **Lock state / Stratum / Leap Indicator / Root Dispersion**
- **Phase error (offset)**, **jitter**, **frequency correction (ppb)**
- **PPS count / missed / Holdover duration**
- **GNSS satellites (in-solution / visible)**, Fix Quality, HDOP, current baud rate
- **Servo event counters**: step / resync / slip / mismatch / late (for troubleshooting; a steadily
  rising value means the timing link has a problem)
- **Last 15 raw sentences**: GNGGA / GNGSA / GNZDA, to verify fix and timing

The same data is available as JSON at `http://<device-ip>/status.json` (for scripts / monitoring).

---

## NTP Client Example

Once locked, the device is **Stratum 1**. Add it as an NTP source on Linux:

```bash
# /etc/chrony/chrony.conf or /etc/ntpsec/ntp.conf
server <device-ip> iburst minpoll 3 maxpoll 4 prefer
```

Verify:

```bash
ntpq -p            # you should see this device at stratum 1, offset usually within tens of µs
# or
chronyc sources -v
```

---

## Serial Diagnostics

`idf.py monitor` prints one diagnostic line every `CFG_LOG_INTERVAL_MS` (default 5s) from
`monitor.c`. Below is a **locked** device's real output (values excerpted from an actual run):

```
I (48026284) mon: LINK UP 192.168.6.201 | UTC 2026-09-15 04:00:31 | LOCKED  stratum=1 li=0 | off=0us jit=1us ppb=5004 | hold=956ms pps=189414/0 | sats=9/11 fixq=1 hdop=2.5 [FIX] | baud=9600 cfg=0 | ntp req=8 resp=8 unsync=0 bad=0 drop=0 txfail=0 | rxTs 8/0 | linkup=1 | sv 1/1/2/1/128 | lag=683ms | heap=217KB
```

Field-by-field meaning:

| Segment | Meaning |
|---------|---------|
| `LINK UP 192.168.6.201` | Ethernet link state and IP (DHCP) |
| `UTC 2026-09-14 15:37:37` | Current UTC; shows `未同步` (unsynchronized) when not anchored |
| `LOCKED stratum=1 li=0` | Lock state / NTP stratum / Leap Indicator (LI) |
| `off=0us jit=1us ppb=5004` | PPS phase error / jitter / frequency correction (ppb) |
| `hold=956ms pps=189414/0` | Holdover duration (`n/a(无PPS)` if PPS never seen) / PPS total and missed |
| `sats=9/11 fixq=1 hdop=2.5 [FIX]` | Satellites (**in-solution / visible**) / Fix Quality / HDOP / fix valid |
| `baud=9600 cfg=0` | Current GNSS baud rate (auto-detected) / UBX config messages sent (always 0 with defaults) |
| `ntp req=8 resp=8 unsync=0 bad=0 drop=0 txfail=0` | NTP requests / responses / **valid but not answered (no time base yet)** / invalid / dropped by token bucket / send failures |
| `rxTs 8/0` | Times the driver-layer inbound hardware timestamp was used / fell back to socket time |
| `linkup=1` | Cumulative link-UP count (keeps rising if the link is still flapping) |
| `sv 1/1/2/1/128` | Servo events: step / resync (RMC) / integer-second slip / RMC mismatch / cross-second |
| `lag=683ms` | Lag of the NMEA sentence start relative to its PPS edge |
| `heap=217KB` | Free heap memory |

> Note: `off=0us`, `ppb=5004`, jitter `jit=1us`, and steadily accumulating PPS show the device is
> in a healthy locked state with microsecond-level accuracy. `unsync` counts valid requests that were
> not answered because no absolute time base had been established yet (stays 0 once locked); the last
> `sv` value (`late`) growing slowly at 9600 baud is normal — it is the pairing fallback when an NMEA
> burst crosses a second boundary.

---

## Troubleshooting

- **Link keeps flapping UP/DOWN**: confirm IO16 is pulled high (oscillator enable),
  `EMAC_CLK_EXT_IN` is selected, and `reset_gpio_num = -1` (PHY nRST not wired to GPIO).
- **Phase error stuck at ~+200 ms**: the PPS actually fires on the falling edge. Set
  `CFG_TP5_POLARITY_RISING` to `0` in `config.h` — **this only takes effect when
  `CFG_GPS_SEND_UBX_CFG=1` and the module accepts CFG writes**; with the default 0 the value is
  never sent to the module at all.
- **RMC never matches PPS**: check the GNSS baud was detected (`baud_locked`), whether GSV is too
  long and causes cross-second (turn off `CFG_GPS_KEEP_GSV`, which likewise **requires
  `CFG_GPS_SEND_UBX_CFG=1`**), and whether the fix qualifies (`sats_used >= CFG_GPS_MIN_SATS`).
- **Occasional `N 秒未收到合法 NMEA，重新探测波特率` in the log**: that is the self-healing path
  (`CFG_GPS_RELOCK_SEC`, default 15 s) triggered when the module was power-cycled / swapped or its
  baud rate changed. Probing is read-only and restores the previous baud rate on failure.
- **HTTP page fails to load**: check whether `/status.json` is valid JSON; first confirm the service
  started via the serial log.

---

## License

This project is for study and research only. The ESP-IDF and third-party components it depends on
retain their respective original licenses.

---

## Acknowledgements

- Espressif **ESP-IDF** and the **LAN8720** RMII driver.
- NTP protocol per **RFC 5905**.
- GNSS time base per u-blox NMEA / UBX protocols.
