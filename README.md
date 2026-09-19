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
- **GNSS 波特率自动探测**：优先 115200（突发最短、授时最准），收不到有效语句再依次降到 57600 → 38400 → 19200 → 9600；探测本身纯被动监听。
- **上电主动提速**：握手成功后从高到低逐个尝试更高的波特率（115200 → 57600 → …），用 UBX-CFG-PRT 下发，并**以"切过去之后还能不能收到报文"为判据**（不用 ACK —— AF68GBR 这类中科微系模块不回 ACK 但会照做），留下第一个成功的那个：RMC 相对 PPS 的滞后（日志里的 `lag`）会从 9600 下的 600+ ms 压到 ~50 ms。全都失败则保持原波特率，不影响使用。
- **模块自愈：软复位（默认，无需任何硬件）**：模块超过 10 分钟仍拿不到合格定位时，发 **UBX-CFG-RST** 让它自己重启 —— 默认**温启动**（只清星历、保留历书/位置/时间，重新定位比冷启动快得多），而且只复位 GNSS 子系统、**不动 UART 配置**（波特率保持，不必重新握手）；`CFG_GPS_RST_MODE` 可切成热启动/冷启动。连续 3 次无效即停止并告警。
- **支持多 Hz 模块（5 / 10 / 25 Hz）**：只采纳 NMEA 时间戳为整数秒（`.00`）的那一整段报文与 PPS 对齐，同一秒内其余各段直接丢弃 —— 避免同一个整秒被反复投喂导致一直重对齐（表现为锁不上），也免去无谓的解析负担。
- **多源闰秒**：只读轮询 UBX-NAV-TIMEGPS 的 `leapS`，并与内置闰秒表交叉校验。
- **完整 NTPv4 报文**：Stratum / Root Dispersion / Precision / Leap Indicator 全部按锁定状态动态填写；独立任务 + BSD socket + `SO_RCVBUF` + 令牌桶限速，抗突发查询（任务优先级刻意低于 lwIP tcpip 线程，避免优先级反转）。
- **抗反射 / 抗误用**：只响应 mode 3（标准客户端）；源地址属于 `0.0.0.0/8`、组播或保留网段的请求直接丢弃；尚未建立绝对时间基准时**静默不回包**（不会发出 transmit 时戳为 0 的非法响应，该计数显示为 `unsync`）。
- **双核分工**：CPU0 = 时基 / PPS / GPS（时间敏感，独占）；CPU1 = 以太网 / lwIP / NTP / HTTP（与 lwIP tcpip 线程同核）。
- **以太网健壮性**：DHCP 获取地址；自定义 input path 抓 NTP 入站帧到达时刻；断链超时强制重建 PHY，暴露 link UP 计数以观察是否仍在翻动。
- **Web 监控面板**：实时显示锁定状态、相位误差、频率修正、卫星数、Holdover 等，并保留最近15 条 GNGGA / GNGSA / GNZDA 原始报文便于排障。
- **远程查看 GNSS 原始报文（排障利器）**：把 GNSS 串口上的**原始字节流**（NMEA 文本 + 模块的 UBX 二进制应答 + 校验失败的坏帧 + 波特率探测期间的乱码）原样转发到 **TCP/8880**，用串口调试助手 / telnet / nc 远程连上就能看到模块"到底在说什么"，不必抱着笔记本蹲在设备旁接串口线。判断模块健康状况时，面板上的统计是"结论"、这份原始流是"一手证据"。对授时链路零干扰：GPS 任务侧只多一次无锁 `memcpy`，socket 收发全在网络核的独立任务里（见 `gnss_tcp.c`）。
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
 │                          │      │  • GNSS 原始报文 TCP 转发  │
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
| `gnss_tcp.c`   | GNSS 串口原始字节流的 TCP 转发（TCP/8880，远程串口调试）     |

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
- `CONFIG_LWIP_MAX_SOCKETS=16`：socket 总数（HTTP 面板 5 + NTP 1 + GNSS 原始报文 TCP 转发 1 监听 + 客户端）。
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
| `CFG_GPS_SET_BAUD` / `TARGET_BAUD` | `1` / `115200` | 握手成功后从高到低尝试**高于当前**的候选波特率（上限 `TARGET_BAUD`），保留第一个仍能收到报文者；判据不是模块的 ACK（AF68GBR 不回 ACK 但会照做） |
| `CFG_GPS_SEND_UBX_CFG`          | `0`    | 是否下发 UBX 配置（写）                            |
| `CFG_GPS_POLL_NAV_TIMEGPS`      | `1`    | 只读轮询 UBX-NAV-TIMEGPS 取闰秒                    |
| `CFG_GPS_KEEP_GSV`              | `0`    | 是否保留 GSV（关掉可缩短突发、降低跨秒概率）       |
| `CFG_GPS_MIN_SATS` / `FIXQ_MIN` / `FIXQ_MAX` | `4`/`1`/`5` | 定位合格判据：参与解算卫星数 + fix quality 范围（6=推算、8=模拟不算合格） |
| `CFG_GPS_RST_MODE` / `REFIX_SEC` / `REFIX_MAX` | `2`/`600`/`3` | **模块自愈（默认无需硬件）**：10 分钟无 fix → 发 UBX-CFG-RST（`2`=温启动，可改 `1`=热启动 / `3`=冷启动）；连续 3 次无效即停止 |
| `CFG_PPS_TIMEOUT_MS`            | `2500` | 超此时间未见 PPS → 进入 holdover                   |
| `CFG_PLL_PHASE_DIV` / `FLL_GAIN_PCT` | `4`/`25` | 伺服环相位/频差增益                        |
| `CFG_PPB_LIMIT`                | `100000` | 频率修正常数上限（ppb，±100 ppm）               |
| `CFG_NTP_SO_RCVBUF`             | `16384` | NTP socket 接收缓冲                              |
| `CFG_NTP_RATE_PPS` / `BURST`    | `400`/`800` | 令牌桶平均/突发限速                          |
| `CFG_GNSS_TCP_ENABLE` / `PORT`  | `1`/`8880` | GNSS 串口原始字节流的 TCP 转发（远程串口调试；`0` 关闭） |
| `CFG_GNSS_TCP_MAX_CLIENTS` / `RING` | `3`/`4096` | 同时连接客户端数上限 / 环形缓冲字节数（2 的幂） |
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
- **NMEA TCP 客户端数**：当前有几个连接正在通过 TCP/8880 看原始字节流

状态数据同时以 JSON 提供：`http://<设备IP>/status.json`（供脚本/监控系统集成）。

---

## 远程查看 GNSS 原始报文（TCP/8880）

设备会把 GNSS 模块**吐在串口上的原始字节**（与串口线上出现的完全一致）转发到 **TCP 8880**，
于是坐在另一台机器上就能"接上"模块的串口，用来判断模块到底在工作还是在装死：

```bash
# Linux / macOS / Windows(WSL) 都行；Windows 也可以直接用串口调试助手的 TCP Client 模式
nc 192.168.1.50 8880
# 或者
telnet 192.168.1.50 8880
```

连上后先收到 2~3 行以 `#` 开头的提示（当前波特率、fix 状态、时基是否已锁定），
随后就是模块输出的原始字节：

```
# GNSS raw TCP feed - the bytes below come from the GNSS UART as-is
# 2026-09-15 04:00:31 UTC | baud=115200 fixq=1 sats=9/11 FIX
# Lines starting with '#' are added by this server, not by the GNSS module.
$GNRMC,040031.00,A,3959.12345,N,11618.54321,E,0.021,,150926,,,A,V*33
$GNGGA,040031.00,3959.12345,N,11618.54321,E,1,09,1.02,45.3,M,-8.4,M,,*6A
...
```

几点说明：

- **内容与串口完全一致**：除了 NMEA 文本行，还会看到模块对 UBX 查询/配置的二进制应答
  （例如上电提速时的 `CFG-PRT` ACK/NAK）、校验失败的坏帧，以及波特率探测期间收到的乱码 ——
  排障时这些恰恰是最有用的信息。
- **只出不进**：服务器不解析客户端发来的任何内容（收到即丢弃，仅用于探测断开）。
- **不影响授时**：GPS 任务侧只多做一次无锁 `memcpy`，socket 收发全在 CPU1 的独立任务里；
  面板上的 `NMEA TCP 端口/连接/已发/丢弃` 可以确认转发是否正常。
- 客户端最多 `CFG_GNSS_TCP_MAX_CLIENTS`（默认 3）个；某个客户端读得太慢会被直接断开
  （它拖慢不了别人，重连即可）。没有任何客户端时不占用缓冲，也不累计丢弃计数。
- 不想暴露：把 `CFG_GNSS_TCP_ENABLE` 设为 `0`（连字节投递都省掉）。

**典型用法** —— 把三种"看起来都像坏了"的情形分开：

| 现象 | 结论 |
|------|------|
| 有完整且校验正确的 NMEA 流水，但 `sats=0`、RMC 的 `status='V'` | 模块供电 / 串口 / 波特率都正常，**纯粹是拿不到定位**（天线、视野、干扰） |
| 一个字节都没有，或只有乱码 | 波特率不对、串口线/焊点有问题、模块没上电 —— 与定位无关 |
| 只有零星字节、NMEA 行经常截断，且面板 `nmea_bad` 持续增长 | 串口物理带宽不够（例如 5 Hz 全语句 @9600），需要提速或裁剪输出 |

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
I (48026284) mon: LINK UP  192.168.6.201 | UTC 2026-09-15 04:00:31 | LOCKED   stratum=1 li=0 | off=0us jit=1us ppb=5004 | hold=956ms pps=189414/0 | sats=9/11 fixq=1 hdop=2.5 [FIX] | baud=115200 cfg=1 nmea=1234/0 rmc=1180/1170/0/5 gga=0 | ntp req=8 resp=8 unsync=0 bad=0 drop=0 txfail=0 | rxTs 8/0 | linkup=1 | nmeaTcp 1/1/0 | sv 1/1/2/1/128 | lag=48ms | heap=217KB
```

逐字段含义：

| 片段 | 含义 |
|------|------|
| `LINK UP 192.168.6.201` | 以太网链路状态与 IP（DHCP 获取） |
| `UTC 2026-09-15 04:00:31` | 当前 UTC；未同步时显示 `未同步` |
| `LOCKED stratum=1 li=0` | 锁定状态 / NTP 层级 / 闰秒指示符（LI） |
| `off=0us jit=1us ppb=5004` | PPS 相位误差 / 抖动 / 频率修正（ppb） |
| `hold=956ms pps=189414/0` | Holdover 时长（未见过 PPS 时为 `n/a(无PPS)`）/ PPS 累计与丢失 |
| `sats=9/11 fixq=1 hdop=2.5 [FIX]` | 卫星（**参与解算 / 多星座可见数之和**）/ Fix Quality / HDOP / 定位可用。`定位可用` 要求 fix quality ∈ [`CFG_GPS_FIXQ_MIN`, `CFG_GPS_FIXQ_MAX`]（默认 **1~5**，即排除 6=推算、8=模拟）**且**参与解算卫星数 ≥ `CFG_GPS_MIN_SATS` |
| `baud=115200 cfg=1 nmea=1234/0` | 当前 GNSS 波特率（自动探测后由上电提速抬高）/ 已下发的 UBX 报文条数（每次 `CFG-PRT` 提速尝试、每次模块软复位都会 +1；配置报文只在 `CFG_GPS_SEND_UBX_CFG=1` 时计数）/ **采纳的 NMEA 语句数 / 坏帧数**（坏帧持续增长 = 串口在丢字节，多半是带宽不足或信号差） |
| `rmc=1/1/0/0 gga=0` | RMC 分项计数：**识别到 / 成功对时 / 内容不可用 / 被整段筛选丢弃**，以及 **`gga`＝RMC 失效期间改用 GGA 时间戳兜底供秒的次数**。`识别到`不涨 = 模块没输出 RMC（没有绝对秒，永远锁不上）；`识别到`涨但`对时`不涨 = 模块的 RMC 自身不可用（status='V' 或字段残缺）—— 此时固件会自动切到 GGA 兜底（看到 `gga` 增长即是在兜底），并每 10 s 打印一条 `RMC 被丢弃（原因）: 原文` 供定位 |
| `ntp req=8 resp=8 unsync=0 bad=0 drop=0 txfail=0` | NTP 请求 / 应答 / **未同步未应答** / 非法报文 / 被令牌桶丢弃 / 发送失败 |
| `rxTs 8/0` | 使用了驱动层入站硬件时戳的次数 / 回退到 socket 时刻的次数 |
| `linkup=1` | 累计 link UP 次数（持续上涨说明链路仍在翻动） |
| `nmeaTcp 1/1/0` | GNSS 原始报文 TCP 转发（`CFG_GNSS_TCP_*`）：**当前客户端数 / 累计连接数 / 缓冲丢弃字节**。丢弃持续增长说明某个客户端读得太慢（会被断开） |
| `sv 1/1/2/1/128` | 伺服事件：阶跃 / 重对齐(RMC) / 整秒滑移 / RMC 失配 / 传输跨秒 |
| `lag=48ms` | NMEA 语句起始相对其 PPS 边沿的滞后 |
| `heap=217KB` | 空闲堆内存 |

> 说明：上述 `off=0us`、`ppb=5004`、相位抖动 `jit=1us`、PPS 稳定累计说明设备已处于健康锁定状态，授时精度在微秒级。
> `unsync` 是"尚未建立绝对时间基准、按 RFC 5905 不回包"的请求数（锁定后应恒为 0，只在冷启动未对齐期间增长）；
> `sv` 最后一位（`late`）在较低波特率下缓慢增长属常态 —— 那是 NMEA 突发跨秒时的正常配对回退，不是故障（115200 下突发足够短，很少发生）。

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
- **接了 5 Hz / 10 Hz 模块后锁不上、或串口像"堵住"**：先看日志里的 `nmea=OK/BAD` 与 `baud=`。每秒只应采纳 1 组整数秒报文（约 10 条）；若 `OK` 每秒增长远少于这个数、而 `BAD` 持续增长，说明**串口物理带宽不够**（例：5 Hz 全语句 @ 9600 约 3 KB/s，而 9600 只有约 0.96 KB/s，必然丢字节），固件的整段筛选救不了 —— 需要把模块改到 115200，或用厂商工具 / UBX-CFG 把输出降到 1 Hz、裁掉 VTG/GSA/GLL/GSV。带宽足够时（例如 115200），整段筛选会正常完成对齐与锁定。
- **串口偶发 `N 秒未收到合法 NMEA，重新探测波特率`**：这是模块掉电重启 / 被换 / 波特率被改之后的自愈动作（`CFG_GPS_RELOCK_SEC`，默认 15 s）。探测是只读的、失败会恢复原波特率；嫌频繁可调大该值，设 0 关闭。
- **运行数小时后突然 `UNLOCKED`，日志反复出现 `RMC 被丢弃（status 不是 A（模块报告定位无效））`**：这是**模块侧**掉了定位，不是固件问题 —— 报文里 RMC 的 status 是 `V`、GGA 的 `fixq=0`/`sats=0`，有些模块还会连 PPS 一起停（表现为 `pps=` 不再增长、`hold` 一路涨大）。固件此时会**按设计拒绝采纳它的时间**：RMC 直接丢弃；GGA 兜底额外要求当前定位合格（`fix_ok`）且与钟面相差 ≤1 s；即使偏差恰好是整数秒，只要超过 `CFG_NMEA_SLIP_MAX_SEC`（默认 600 s）也不会"滑移"对齐，只丢弃并告警。模块重新拿到定位后会自动恢复。
- **想确认"模块到底在说什么"**：用 `nc <设备IP> 8880`（或串口调试助手的 TCP Client 模式）直接看 GNSS 串口的原始字节流，见上文"远程查看 GNSS 原始报文（TCP/8880）"——这是区分"模块拿不到定位"与"模块/串口根本没在工作"最快的手段。
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
  then falls back to 57600 → 38400 → 19200 → 9600 if no valid sentence is seen; the probing itself
  is passive listening only.
- **Proactive speed-up at boot**: after the handshake the firmware walks the candidate rates from
  high to low (115200 → 57600 → …) issuing UBX-CFG-PRT, and **judges success by "can we still
  receive sentences at the new rate"** rather than by the module's ACK (Zhongkewei-class modules such
  as the AF68GBR never ACK yet still obey the command), keeping the first rate that works — this cuts
  the RMC-to-PPS lag (`lag` in the log) from 600+ ms at 9600 to ~50 ms. If every attempt fails, the
  original baud rate is kept.
- **Module self-recovery: software reset (default, no extra hardware)**: if the module cannot get a
  valid fix for 10 minutes the firmware issues **UBX-CFG-RST** — by default a **warm start** (clears
  only the ephemeris, keeps almanac/position/time, so it re-fixes far faster than a cold start) and
  GNSS-only, which leaves the UART configuration untouched (baud rate preserved, no re-handshake
  needed). `CFG_GPS_RST_MODE` selects hot/warm/cold; it gives up with a warning after 3 tries.
- **Multi-rate GNSS support (5 / 10 / 25 Hz)**: only the burst whose NMEA timestamp is a whole
  second (`.00`) is accepted and aligned to PPS; the other bursts of the same second are dropped —
  otherwise the same whole second would be fed repeatedly (constant re-alignment, never locks) and
  the parser would waste effort on sentences that can never be used.
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
- **Remote access to the raw GNSS sentences (a troubleshooting lifeline)**: the **raw byte stream**
  from the GNSS UART (NMEA text + the module's binary UBX replies + sentences that fail their
  checksum + the garbage seen while probing baud rates) is forwarded verbatim to **TCP/8880**, so a
  serial assistant / `telnet` / `nc` anywhere on the LAN can "attach" to the module's serial port and
  see what it is really saying — no need to sit next to the device with a USB-TTL adapter. When you
  judge module health, the panel is the *conclusion* and this stream is the *primary evidence*. Zero
  impact on timing: the GPS task only adds one lock-free `memcpy`, all socket I/O happens in a
  dedicated task on the network core (see `gnss_tcp.c`).
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
 │                          │      │  • GNSS raw TCP forwarder │
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
| `gnss_tcp.c` | TCP forwarder for the raw GNSS UART stream (TCP/8880, remote serial debug) |

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
- `CONFIG_LWIP_MAX_SOCKETS=16`: total sockets (HTTP panel 5 + NTP 1 + GNSS raw TCP forwarder: 1 listener + clients).
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
| `CFG_GPS_SET_BAUD` / `TARGET_BAUD` | `1` / `115200` | After the boot handshake, try the candidate rates **above the current one** (highest first, capped by `TARGET_BAUD`) and keep the first that still delivers sentences; the module's ACK is *not* the criterion (AF68GBR never ACKs, yet obeys) |
| `CFG_GPS_SEND_UBX_CFG`         | `0`     | Send UBX config (write)                        |
| `CFG_GPS_POLL_NAV_TIMEGPS`     | `1`     | Read-only poll UBX-NAV-TIMEGPS for leap second |
| `CFG_GPS_KEEP_GSV`             | `0`     | Keep GSV (off shortens burst, lowers cross-second risk) |
| `CFG_GPS_MIN_SATS` / `FIXQ_MIN` / `FIXQ_MAX` | `4`/`1`/`5` | Fix-valid criteria: in-solution satellites + fix-quality range (6=estimated, 8=simulated rejected) |
| `CFG_GPS_RST_MODE` / `REFIX_SEC` / `REFIX_MAX` | `2`/`600`/`3` | **Module self-recovery (no hardware needed)**: no valid fix for 10 min → issue UBX-CFG-RST (`2`=warm, `1`=hot, `3`=cold start); stops after 3 tries |
| `CFG_PPS_TIMEOUT_MS`           | `2500`  | No PPS beyond this → enter holdover            |
| `CFG_PLL_PHASE_DIV` / `FLL_GAIN_PCT` | `4`/`25` | Servo phase / frequency gain          |
| `CFG_PPB_LIMIT`                | `100000` | Frequency correction clamp (ppb, ±100 ppm)    |
| `CFG_NTP_SO_RCVBUF`            | `16384` | NTP socket receive buffer                      |
| `CFG_NTP_RATE_PPS` / `BURST`   | `400`/`800` | Token-bucket avg / burst rate              |
| `CFG_GNSS_TCP_ENABLE` / `PORT` | `1`/`8880` | TCP forwarder for the raw GNSS UART stream (remote serial debug; `0` disables) |
| `CFG_GNSS_TCP_MAX_CLIENTS` / `RING` | `3`/`4096` | Max simultaneous clients / ring buffer size in bytes (power of two) |
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
- **NMEA TCP client count**: how many connections are watching the raw byte stream on TCP/8880

The same data is available as JSON at `http://<device-ip>/status.json` (for scripts / monitoring).

---

## Viewing the Raw GNSS Output Remotely (TCP/8880)

The device forwards the **raw bytes the GNSS module puts on its UART** (identical to what appears on
the wire) to **TCP 8880**, so from another machine you can "attach" to the module's serial port and
tell whether it is working or stone dead:

```bash
# Linux / macOS / Windows (WSL); on Windows you can also use a serial assistant in TCP Client mode
nc 192.168.1.50 8880
# or
telnet 192.168.1.50 8880
```

On connect you first get 2–3 `#`-prefixed lines (current baud rate, fix state, whether the time base
is locked), then the module's raw output:

```
# GNSS raw TCP feed - the bytes below come from the GNSS UART as-is
# 2026-09-15 04:00:31 UTC | baud=115200 fixq=1 sats=9/11 FIX
# Lines starting with '#' are added by this server, not by the GNSS module.
$GNRMC,040031.00,A,3959.12345,N,11618.54321,E,0.021,,150926,,,A,V*33
$GNGGA,040031.00,3959.12345,N,11618.54321,E,1,09,1.02,45.3,M,-8.4,M,,*6A
...
```

Notes:

- **Byte-for-byte identical to the serial port**: besides NMEA text you also see the module's binary
  UBX replies (e.g. the `CFG-PRT` ACK/NAK during the boot speed-up), sentences that fail their
  checksum, and the garbage received while probing baud rates — exactly the evidence you need when
  troubleshooting.
- **Outbound only**: the server never parses anything a client sends (bytes are discarded; reads are
  used only to detect a disconnect).
- **No impact on timing**: the GPS task only does one extra lock-free `memcpy`; all socket I/O lives in
  a dedicated task on CPU1. The panel's `NMEA TCP port/conns/sent/drop` row confirms the forwarding.
- At most `CFG_GNSS_TCP_MAX_CLIENTS` clients (default 3); a client that reads too slowly is simply
  disconnected (it cannot slow the others down — just reconnect). With no client connected the ring
  buffer is not occupied and no drops are counted.
- To disable: set `CFG_GNSS_TCP_ENABLE` to `0` (even the byte hand-off is compiled out).

**Typical use** — telling three conditions apart that all look like "it's broken":

| Observation | Conclusion |
|-------------|------------|
| A complete, checksum-valid NMEA stream, but `sats=0` and RMC `status='V'` | Supply / UART / baud rate are all fine — **it simply has no fix** (antenna, sky view, interference) |
| Not a single byte, or only garbage | Wrong baud rate, bad serial wiring/solder joint, or the module has no power — nothing to do with the fix |
| Only occasional bytes, NMEA lines often truncated, panel `nmea_bad` rising | The serial link is physically saturated (e.g. 5 Hz with all sentences @9600) — raise the baud rate or trim the output |

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
I (48026284) mon: LINK UP  192.168.6.201 | UTC 2026-09-15 04:00:31 | LOCKED   stratum=1 li=0 | off=0us jit=1us ppb=5004 | hold=956ms pps=189414/0 | sats=9/11 fixq=1 hdop=2.5 [FIX] | baud=115200 cfg=1 nmea=1234/0 rmc=1180/1170/0/5 gga=0 | ntp req=8 resp=8 unsync=0 bad=0 drop=0 txfail=0 | rxTs 8/0 | linkup=1 | nmeaTcp 1/1/0 | sv 1/1/2/1/128 | lag=48ms | heap=217KB
```

Field-by-field meaning:

| Segment | Meaning |
|---------|---------|
| `LINK UP 192.168.6.201` | Ethernet link state and IP (DHCP) |
| `UTC 2026-09-15 04:00:31` | Current UTC; shows `未同步` (unsynchronized) when not anchored |
| `LOCKED stratum=1 li=0` | Lock state / NTP stratum / Leap Indicator (LI) |
| `off=0us jit=1us ppb=5004` | PPS phase error / jitter / frequency correction (ppb) |
| `hold=956ms pps=189414/0` | Holdover duration (`n/a(无PPS)` if PPS never seen) / PPS total and missed |
| `sats=9/11 fixq=1 hdop=2.5 [FIX]` | Satellites (**in-solution / sum of all constellations' visible counts**) / Fix Quality / HDOP / fix valid. `fix valid` requires fix quality within [`CFG_GPS_FIXQ_MIN`, `CFG_GPS_FIXQ_MAX`] (default **1–5**, i.e. 6=estimated and 8=simulated are rejected) **and** in-solution satellites ≥ `CFG_GPS_MIN_SATS` |
| `baud=115200 cfg=1 nmea=1234/0` | Current GNSS baud rate (auto-detected, then raised by the boot speed-up) / UBX messages sent (incremented by each `CFG-PRT` speed-up attempt and each module soft reset; config messages are counted only when `CFG_GPS_SEND_UBX_CFG=1`) / **accepted NMEA sentences / bad frames** (bad frames growing = dropped bytes: not enough bandwidth or a bad signal) |
| `rmc=1/1/0/0 gga=0` | RMC breakdown: **seen / used for timing / unusable content / dropped by the whole-second filter**, plus **`gga` = times the GGA timestamp was used as a fallback while RMC was unusable**. `seen` flat = the module emits no RMC at all (no absolute second → it can never lock); `seen` rising but `used` flat = the RMC itself is unusable (status='V' or truncated fields) — the firmware then falls back to GGA automatically (watch `gga`) and logs one `RMC 被丢弃（reason）: <sentence>` every 10 s to pin down the cause |
| `ntp req=8 resp=8 unsync=0 bad=0 drop=0 txfail=0` | NTP requests / responses / **valid but not answered (no time base yet)** / invalid / dropped by token bucket / send failures |
| `rxTs 8/0` | Times the driver-layer inbound hardware timestamp was used / fell back to socket time |
| `linkup=1` | Cumulative link-UP count (keeps rising if the link is still flapping) |
| `nmeaTcp 1/1/0` | GNSS raw-sentence TCP forwarder (`CFG_GNSS_TCP_*`): **current clients / total connections / bytes dropped from the ring buffer**. A steadily rising drop count means a client is reading too slowly (it gets disconnected) |
| `sv 1/1/2/1/128` | Servo events: step / resync (RMC) / integer-second slip / RMC mismatch / cross-second |
| `lag=48ms` | Lag of the NMEA sentence start relative to its PPS edge |
| `heap=217KB` | Free heap memory |

> Note: `off=0us`, `ppb=5004`, jitter `jit=1us`, and steadily accumulating PPS show the device is
> in a healthy locked state with microsecond-level accuracy. `unsync` counts valid requests that were
> not answered because no absolute time base had been established yet (stays 0 once locked); the last
> `sv` value (`late`) growing slowly is normal at lower baud rates — it is the pairing fallback when an
> NMEA burst crosses a second boundary (at 115200 the burst is short enough that it rarely happens).

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
- **Drops to `UNLOCKED` after hours of running, with repeated `RMC 被丢弃（status 不是 A（模块报告定位无效））`**: that is the **module** losing its fix, not a firmware problem — the RMC `status` is `V`, GGA shows `fixq=0`/`sats=0`, and some modules even stop PPS (the log then shows `pps=` frozen and `hold` growing). The firmware deliberately refuses to trust such a time source: RMC is dropped, the GGA fallback additionally requires a valid fix (`fix_ok`) and ≤1 s agreement with the current clock face, and even an integer-second offset beyond `CFG_NMEA_SLIP_MAX_SEC` (default 600 s) is discarded with a warning instead of being "slipped" into the clock. It re-locks automatically once the module regains its fix.
- **Won't lock with a 5 Hz / 10 Hz module, serial looks "clogged"**: check `nmea=OK/BAD` and `baud=`
  in the log. Only one whole-second burst per second (~10 sentences) should be accepted; if `OK`
  grows far slower than that while `BAD` keeps rising, the **serial link is physically saturated**
  (e.g. 5 Hz with all sentences @ 9600 ≈ 3 KB/s against ~0.96 KB/s available) and no firmware filter
  can help — switch the module to 115200, or reduce it to 1 Hz / drop VTG/GSA/GLL/GSV with the
  vendor tool or UBX-CFG. With enough bandwidth (e.g. 115200) the whole-second filter aligns and
  locks normally.
- **Occasional `N 秒未收到合法 NMEA，重新探测波特率` in the log**: that is the self-healing path
  (`CFG_GPS_RELOCK_SEC`, default 15 s) triggered when the module was power-cycled / swapped or its
  baud rate changed. Probing is read-only and restores the previous baud rate on failure.
- **Want to see what the module is actually saying**: use `nc <device-ip> 8880` (or a serial
  assistant in TCP Client mode) to watch the raw GNSS UART byte stream — see
  [Viewing the Raw GNSS Output Remotely](#viewing-the-raw-gnss-output-remotely-tcp8880) above. It is
  the fastest way to separate "the module has no fix" from "the module/UART is not working at all".
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
