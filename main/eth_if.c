/*
 * 以太网接口层实现（ESP-IDF v6.1 API）
 */
#include "eth_if.h"
#include "config.h"
#include "discipline.h"

#include <string.h>
#include <stdio.h>
#include <assert.h>
#include "esp_log.h"
#include "esp_event.h"
#include "esp_eth.h"
#include "esp_eth_mac_esp.h"
#include "esp_eth_phy.h"
#include "esp_netif.h"
#include "esp_netif_defaults.h"
#include "esp_netif_ip_addr.h"
#include "lwip/ip4_addr.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "eth";

static esp_netif_t      *s_netif    = NULL;
static esp_eth_handle_t  s_handle   = NULL;
static volatile bool     s_link_up  = false;
static uint32_t          s_down_since_s = 0;
static uint32_t          s_up_count = 0;

/* -------------------- 入站 NTP 帧时戳环形缓冲 ---------------------- */
/* 生产者 = EMAC 接收任务，消费者 = NTP 任务，单进单出，用自旋锁保护 */
static portMUX_TYPE  s_ts_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint64_t s_ts[CFG_RX_TS_RING];
static volatile uint32_t s_ts_head;
static volatile uint32_t s_ts_tail;
#define TS_MASK (CFG_RX_TS_RING - 1)

static inline void ts_push(uint64_t t)
{
    portENTER_CRITICAL(&s_ts_lock);
    if (s_ts_head - s_ts_tail >= (uint32_t)CFG_RX_TS_RING) {
        s_ts_tail++;                      /* 缓冲满：丢弃最旧的一条，绝不阻塞驱动 */
    }
    s_ts[s_ts_head & TS_MASK] = t;
    s_ts_head++;
    portEXIT_CRITICAL(&s_ts_lock);
}

bool eth_if_take_rx_ts(uint64_t *out_ts, uint64_t now)
{
    bool ok = false;
    portENTER_CRITICAL(&s_ts_lock);
    while (s_ts_tail != s_ts_head) {
        uint64_t t = s_ts[s_ts_tail & TS_MASK];
        s_ts_tail++;
        if (now >= t && (now - t) < 200000ULL) {
            *out_ts = t;
            ok = true;
            break;
        }
    }
    portEXIT_CRITICAL(&s_ts_lock);
    return ok;
}

/* ------------------------- 帧识别 ---------------------------------- */
/* 只做最小解析：Ethernet -> IPv4 -> UDP/123 */
static bool frame_is_ntp(const uint8_t *f, uint32_t len)
{
    uint32_t off;
    uint16_t etype;

    if (len < 14 + 20 + 8) {
        return false;
    }
    etype = ((uint16_t)f[12] << 8) | f[13];
    off   = 14;
    if (etype == 0x8100U) {                       /* VLAN tag */
        if (len < 18 + 20 + 8) {
            return false;
        }
        etype = ((uint16_t)f[16] << 8) | f[17];
        off   = 18;
    }
    if (etype != 0x0800U) {                       /* 非 IPv4 */
        return false;
    }
    if ((f[off] >> 4) != 4) {
        return false;
    }
    uint32_t ihl = (uint32_t)(f[off] & 0x0FU) * 4U;
    if (ihl < 20U || (uint32_t)len < off + ihl + 8U) {
        return false;
    }
    if (f[off + 9] != 17U) {                      /* 非 UDP */
        return false;
    }
    uint32_t u = off + ihl;
    uint16_t dport = ((uint16_t)f[u + 2] << 8) | f[u + 3];
    return dport == (uint16_t)CFG_NTP_PORT;
}

/* 以太网输入钩子：先打时戳，再交给 esp-netif。
 * 该钩子运行在 EMAC 接收任务里，是软件可见的最早时刻。 */
static esp_err_t eth_rx_hook(esp_eth_handle_t hdl, uint8_t *buffer, uint32_t length, void *priv)
{
    (void)hdl;
    uint64_t now = tb_now_us();
    if (frame_is_ntp(buffer, length)) {
        ts_push(now);
    }
    return esp_netif_receive((esp_netif_t *)priv, buffer, length, NULL);
}

/* ------------------------- IP 地址（DHCP） --------------------------
 * 不再使用静态地址：地址由路由器分配，换网段即插即用。
 * 仅做兜底：如果 netif 上还没有拿到地址且 DHCP 没在跑，就把它拉起来。 */
static void ensure_dhcp(void)
{
    if (!s_netif) {
        return;
    }
    esp_netif_dhcp_status_t st = ESP_NETIF_DHCP_STOPPED;
    if (esp_netif_dhcpc_get_status(s_netif, &st) != ESP_OK) {
        st = ESP_NETIF_DHCP_STOPPED;
    }
    if (st == ESP_NETIF_DHCP_STARTED) {
        return;
    }
    esp_err_t err = esp_netif_dhcpc_start(s_netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STARTED) {
        ESP_LOGW(TAG, "启动 DHCP 客户端失败: %s", esp_err_to_name(err));
    }
}

/* -------------------- 50MHz 振荡器使能（WT32-ETH01 专用）------------- */
static void eth_clock_enable(void)
{
#if CFG_ETH_OSC_EN_GPIO >= 0
    gpio_config_t en = {
        .pin_bit_mask = (1ULL << CFG_ETH_OSC_EN_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&en));
    gpio_set_level(CFG_ETH_OSC_EN_GPIO, 1);          /* 拉高 -> 振荡器起振 */
    /* 等时钟稳定后再访问 PHY / 初始化 MAC，否则 SMI 可能读不到东西 */
    vTaskDelay(pdMS_TO_TICKS(CFG_ETH_CLK_STABLE_MS));
    ESP_LOGI(TAG, "IO%d 已拉高：板载 50MHz REF_CLK 振荡器使能", CFG_ETH_OSC_EN_GPIO);
#endif
}

/* ------------------------- PHY 自检 ---------------------------------
 * 读出 PHYID 并确认 SMI 通；顺带把 BMCR/BMSR 打出来方便排查地址插错。 */
static void phy_diag(void)
{
    uint32_t id1 = 0, id2 = 0, bmsr = 0;
    esp_eth_phy_reg_rw_data_t rw;
    uint32_t v;

    rw.reg_addr = 2;
    rw.reg_value_p = &id1;
    esp_err_t e1 = esp_eth_ioctl(s_handle, ETH_CMD_READ_PHY_REG, &rw);

    rw.reg_addr = 3;
    rw.reg_value_p = &id2;
    esp_err_t e2 = esp_eth_ioctl(s_handle, ETH_CMD_READ_PHY_REG, &rw);

    v = 0;
    rw.reg_addr = 1;                                  /* BMSR */
    rw.reg_value_p = &v;
    esp_eth_ioctl(s_handle, ETH_CMD_READ_PHY_REG, &rw);
    bmsr = v;

    if (e1 != ESP_OK || e2 != ESP_OK) {
        ESP_LOGE(TAG, "PHY 在 addr=%d 上无响应（SMI 读超时）。"
                      "请检查：IO%d 是否已拉高使能振荡器、"
                      "时钟是否为外部输入、addr 是否应为 0 或 31",
                 CFG_ETH_PHY_ADDR, CFG_ETH_OSC_EN_GPIO);
        return;
    }
    ESP_LOGI(TAG, "PHY addr=%d PHYID=0x%04X%04X BMSR=0x%04X link=%s ane=%s",
             CFG_ETH_PHY_ADDR,
             (unsigned)(id1 & 0xFFFFU), (unsigned)(id2 & 0xFFFFU),
             (unsigned)bmsr,
             (bmsr & (1U << 2)) ? "UP" : "DOWN",
             (bmsr & (1U << 5)) ? "done" : "nogo");
}

/* ------------------------- 事件处理 -------------------------------- */
static void eth_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)data;
    if (base == ETH_EVENT) {
        switch (id) {
        case ETHERNET_EVENT_START:
            ESP_LOGI(TAG, "ETH start");
            break;
        case ETHERNET_EVENT_STOP:
            ESP_LOGW(TAG, "ETH stop");
            s_link_up = false;
            break;
        case ETHERNET_EVENT_CONNECTED: {
            s_link_up = true;
            s_up_count++;
            ensure_dhcp();            /* 链路恢复后确保 DHCP 在跑（重获地址） */
            eth_speed_t  sp  = ETH_SPEED_10M;
            eth_duplex_t dpx = ETH_DUPLEX_HALF;
            esp_eth_ioctl(s_handle, ETH_CMD_G_SPEED, &sp);
            esp_eth_ioctl(s_handle, ETH_CMD_G_DUPLEX_MODE, &dpx);
            ESP_LOGI(TAG, "ETH link UP (第 %lu 次) %sM/%s",
                     (unsigned long)s_up_count,
                     (sp == ETH_SPEED_100M) ? "100" : "10",
                     (dpx == ETH_DUPLEX_FULL) ? "Full" : "Half");
            break;
        }
        case ETHERNET_EVENT_DISCONNECTED:
            s_link_up = false;
            s_down_since_s = (uint32_t)(tb_now_us() / 1000000ULL);
#if CFG_USE_DHCP
            /* 停掉 DHCP，等下次 link UP 时重新走一遍 discover，
             * 避免换网段后仍抱着旧地址不放 */
            esp_netif_dhcpc_stop(s_netif);
#endif
            ESP_LOGW(TAG, "ETH link DOWN");
            break;
        default:
            break;
        }
    }
}

static void ip_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg; (void)base; (void)id; (void)data;
    char ipstr[IP4ADDR_STRLEN_MAX];
    eth_if_get_ip(ipstr, sizeof(ipstr));
    ESP_LOGI(TAG, "IP 就绪(DHCP): %s", ipstr);
}

/* ------------------------- 初始化 ---------------------------------- */
void eth_if_init(void)
{
    /* 0) 先给 WT32-ETH01 的板载 50MHz 振荡器供电使能。
     *    IO0 上的 REF_CLK 来自它，而不是 ESP32 内部 PLL。
     *    必须在 install / MAC 初始化之前完成，否则 MAC 复位会一直失败。 */
    eth_clock_enable();

    /* 1) MAC */
    eth_mac_config_t mac_cfg = ETH_MAC_DEFAULT_CONFIG();
    mac_cfg.rx_task_prio = 15;
    mac_cfg.flags |= ETH_MAC_FLAG_PIN_TO_CORE;

    eth_esp32_emac_config_t emac_cfg = ETH_ESP32_EMAC_DEFAULT_CONFIG();
    emac_cfg.interface                      = EMAC_DATA_INTERFACE_RMII;
    emac_cfg.smi_gpio.mdc_num               = CFG_ETH_MDC_GPIO;
    emac_cfg.smi_gpio.mdio_num              = CFG_ETH_MDIO_GPIO;
    /* 外部 50MHz 灌进 IO0 —— 绝不能写成 EMAC_CLK_OUT，
     * 否则 ESP32 会在 IO0 上反向输出自己的 50MHz，和板载振荡器打架，
     * 表现为 link 每几百毫秒 UP/DOWN 反复翻动。 */
    emac_cfg.clock_config.rmii.clock_mode   = CFG_ETH_CLK_MODE;   /* EMAC_CLK_EXT_IN */
    emac_cfg.clock_config.rmii.clock_gpio   = CFG_ETH_CLK_GPIO;   /* IO0 */
    esp_eth_mac_t *mac = esp_eth_mac_new_esp32(&emac_cfg, &mac_cfg);

    /* 2) PHY
     *    - WT32-ETH01 的 PHY nRST 没有接到任何 GPIO -> reset_gpio_num = -1
     *      （默认值是 5，与 GPS 的 RXD 冲突且硬件上不存在，必须显式改掉）
     *    - 因此 PHY 不能靠 GPIO 复位，只能用 BMCR 软件复位（驱动内部已处理） */
    eth_phy_config_t phy_cfg = ETH_PHY_DEFAULT_CONFIG();
    phy_cfg.phy_addr       = CFG_ETH_PHY_ADDR;
    phy_cfg.reset_gpio_num = CFG_ETH_PHY_RST_GPIO;   /* = -1 */
    esp_eth_phy_t *phy = esp_eth_phy_new_generic(&phy_cfg);

    /* 3) 驱动 */
    esp_eth_config_t eth_cfg = ETH_DEFAULT_CONFIG(mac, phy);
    eth_cfg.check_link_period_ms = 500;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_cfg, &s_handle));

    /* 4) netif + glue */
    esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
    s_netif = esp_netif_new(&netif_cfg);
    assert(s_netif);
    esp_eth_netif_glue_handle_t glue = esp_eth_new_netif_glue(s_handle);
    ESP_ERROR_CHECK(esp_netif_attach(s_netif, glue));
    esp_netif_set_default_netif(s_netif);

    /* 5) 事件 */
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID,
                                               &eth_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                               &ip_event_handler, NULL));

    /* 6) IP 地址：交给 DHCP */
#if CFG_USE_DHCP
    ensure_dhcp();
#endif

    /* 7) 启动 */
    ESP_ERROR_CHECK(esp_eth_start(s_handle));

    /* 8) 接管输入路径，抓取 NTP 入站时戳（必须在 start 之后，覆盖 glue 的默认回调） */
    ESP_ERROR_CHECK(esp_eth_update_input_path(s_handle, eth_rx_hook, s_netif));

    /* 9) PHY 自检：确认 SMI 通路和 PHYID */
    phy_diag();

    ESP_LOGI(TAG, "以太网初始化完成：时钟=外部50MHz(IO%d) SMI MDC=IO%d MDIO=IO%d "
                  "PHY addr=%d reset_gpio=%d",
             CFG_ETH_CLK_GPIO, CFG_ETH_MDC_GPIO, CFG_ETH_MDIO_GPIO,
             CFG_ETH_PHY_ADDR, CFG_ETH_PHY_RST_GPIO);
}

/* ------------------------- 状态查询 -------------------------------- */
bool eth_if_link_up(void)
{
    return s_link_up;
}

void eth_if_get_ip(char *buf, size_t len)
{
    if (!buf || len == 0) {
        return;
    }
    if (!s_netif) {
        snprintf(buf, len, "0.0.0.0");
        return;
    }
    esp_netif_ip_info_t info;
    if (esp_netif_get_ip_info(s_netif, &info) != ESP_OK) {
        snprintf(buf, len, "0.0.0.0");
        return;
    }
    esp_ip4addr_ntoa(&info.ip, buf, (int)len);
}

uint32_t eth_if_link_up_count(void)
{
    return s_up_count;
}

uint32_t eth_if_link_down_seconds(void)
{
    if (s_link_up || s_down_since_s == 0) {
        return 0;
    }
    return (uint32_t)(tb_now_us() / 1000000ULL) - s_down_since_s;
}

/* ------------------------- 断链自愈 -------------------------------- */
void eth_if_periodic(void)
{
#if CFG_ETH_RECOVER_SEC
    if (s_link_up) {
        return;
    }
    uint32_t down = eth_if_link_down_seconds();
    if (down < (uint32_t)CFG_ETH_RECOVER_SEC) {
        return;
    }
    ESP_LOGW(TAG, "链路已断开 %u s，尝试重建 PHY", (unsigned)down);
    esp_eth_stop(s_handle);
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_err_t err = esp_eth_start(s_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_eth_start 失败: %s", esp_err_to_name(err));
    }
    s_down_since_s = (uint32_t)(tb_now_us() / 1000000ULL);
#endif
}
