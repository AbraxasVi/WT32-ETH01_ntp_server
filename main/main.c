/*
 * WT32-ETH01 + u-blox M8N(PPS) -> NTP 服务器
 * 目标平台：ESP-IDF v6.1
 *
 * 硬件连接
 *   - PPS        : GPS TIMEPULSE -> IO2
 *   - GPS 串口   : ESP TXD(IO17) -> GPS RX ；ESP RXD(IO5) <- GPS TX
 *   - 以太网     : 板载 LAN8720（IO16=50MHz 振荡器使能，IO0=外部 REF_CLK 输入，
 *                   IO23 MDC / IO18 MDIO，PHY nRST 未接 GPIO）
 *   - IP         : DHCP 自动获取（原 192.168.6.201 静态地址已改为 DHCP）
 *
 * 对外服务
 *   - NTP  : UDP/123
 *   - Web  : HTTP/80（状态面板）
 *   - 排障 : TCP/8880 —— GNSS 串口原始字节流（含 UBX 二进制）原样转发，
 *            远程用串口调试助手/telnet 连上来看模块真实输出，见 gnss_tcp.c
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_idf_version.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "driver/gpio.h"

#include "config.h"
#include "discipline.h"
#include "gps.h"
#include "gnss_tcp.h"
#include "ntp.h"
#include "eth_if.h"
#include "monitor.h"

static const char *TAG = "main";

/* ---------- PPS：硬件中断捕获，时标取 esp_timer 64 位微秒 ---------- */
static void IRAM_ATTR pps_isr(void *arg)
{
    (void)arg;
    discipline_on_pps(tb_now_us());
}

static void pps_gpio_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << CFG_PPS_GPIO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,      /* 空闲时保持低电平 */
        .intr_type    = CFG_PPS_EDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    /* 高优先级 + IRAM：尽量减少中断响应抖动 */
    ESP_ERROR_CHECK(gpio_install_isr_service(CFG_PPS_ISR_FLAGS));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CFG_PPS_GPIO, pps_isr, NULL));
    ESP_LOGI(TAG, "PPS 输入: IO%d（上升沿中断）", CFG_PPS_GPIO);
}

/* 中断归属由"安装它的那个任务所在的核"决定。
 * 这两段初始化各用一个一次性任务放到目标核上执行：
 *   - PPS 的 GPIO 中断 -> CORE_TIME
 *   - 以太网 EMAC 中断与 RX 任务 -> CORE_NET（与 lwIP tcpip 线程同核） */
static void pps_init_task(void *arg)
{
    (void)arg;
    pps_gpio_init();
    vTaskDelete(NULL);
}

static void eth_init_task(void *arg)
{
    (void)arg;
    eth_if_init();
    vTaskDelete(NULL);
}

/* ---------------------------- 看门狗 ------------------------------ */
static void wdt_init(void)
{
    esp_task_wdt_config_t cfg = {
        .timeout_ms     = CFG_WDT_TIMEOUT_MS,
        .idle_core_mask = (1u << 0) | (1u << 1),
        .trigger_panic  = true,
    };
    esp_err_t err = esp_task_wdt_init(&cfg);
    if (err == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "TWDT 已在启动阶段初始化，沿用 sdkconfig 配置");
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWDT 初始化失败: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "TWDT 已启用: %d ms, panic=%d", CFG_WDT_TIMEOUT_MS, 1);
    }
}

static void wdt_subscribe(void)
{
    esp_err_t err = esp_task_wdt_add(NULL);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TWDT 订阅失败: %s", esp_err_to_name(err));
    }
}

/* ---------------------------- app_main ---------------------------- */
void app_main(void)
{
    ESP_LOGI(TAG, "=== WT32-ETH01 GPS/PPS NTP Server (IDF %s) ===", IDF_VER);

    wdt_init();
    wdt_subscribe();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* 时基与伺服先起来，PPS 中断一挂上就能记录 */
    discipline_init();
    if (xTaskCreatePinnedToCore(pps_init_task, "pps_init", 2048, NULL,
                                configMAX_PRIORITIES - 5, NULL, CFG_CORE_TIME) != pdPASS) {
        ESP_LOGE(TAG, "pps_init 任务创建失败：PPS 中断不会安装，设备无法授时");
    }

    /* 以太网初始化放到 CORE_NET：EMAC 中断 + RX 任务就落在 CPU1，
     * 和 lwIP tcpip 线程同核，时间核保持干净 */
    if (xTaskCreatePinnedToCore(eth_init_task, "eth_init", 4096, NULL,
                                configMAX_PRIORITIES - 6, NULL, CFG_CORE_NET) != pdPASS) {
        ESP_LOGE(TAG, "eth_init 任务创建失败：以太网不会初始化");
    }

    gps_init();
    ntp_server_start();
    gnss_tcp_start();       /* GNSS 原始报文 TCP 转发（远程串口调试用） */
    monitor_start();

#if CFG_GNSS_TCP_ENABLE
    ESP_LOGI(TAG, "全部服务已启动: NTP/UDP%d, GNSS 原始报文 TCP/%d, "
                  "Web 面板 http://<DHCP 分配的 IP>/",
             CFG_NTP_PORT, CFG_GNSS_TCP_PORT);
#else
    ESP_LOGI(TAG, "全部服务已启动: NTP/UDP%d, Web 面板 http://<DHCP 分配的 IP>/",
             CFG_NTP_PORT);
#endif
    ESP_LOGI(TAG, "双核分工: CPU%d=时基/PPS/GPS, CPU%d=以太网/lwIP/NTP/HTTP/GNSS-TCP",
             CFG_CORE_TIME, CFG_CORE_NET);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_task_wdt_reset();
    }
}
