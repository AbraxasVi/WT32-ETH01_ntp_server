/*
 * 监控实现：HTTP 面板 (/ 和 /status.json) + 周期性串口诊断
 */
#include "monitor.h"
#include "config.h"
#include "discipline.h"
#include "gps.h"
#include "ntp.h"
#include "eth_if.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "mon";

/* /status.json 的静态缓冲互斥量（httpd 每连接一任务，需串行化） */
static SemaphoreHandle_t s_json_mux = NULL;

/* ==================== 通用格式化 ================================== */
static void fmt_utc(uint32_t sec, char *out, size_t n)
{
    static const int dim[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    uint32_t days = sec / 86400u;
    uint32_t rem  = sec % 86400u;
    int y = 1970;
    while (y < 2200) {
        uint32_t yd = (uint32_t)(((y % 4 == 0 && y % 100 != 0) || y % 400 == 0) ? 366 : 365);
        if (days < yd) {
            break;
        }
        days -= yd;
        y++;
    }
    int m = 0;
    while (m < 12) {
        uint32_t dl = (uint32_t)dim[m] +
                      ((m == 1 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0)) ? 1u : 0u);
        if (days < dl) {
            break;
        }
        days -= dl;
        m++;
    }
    snprintf(out, n, "%04d-%02d-%02d %02u:%02u:%02u", y, m + 1, (unsigned)days + 1,
             (unsigned)(rem / 3600u), (unsigned)((rem % 3600u) / 60u), (unsigned)(rem % 60u));
}

/* ==================== JSON 状态 =================================== */
/* 往 JSON 缓冲尾部追加内容并返回新的偏移量（溢出时自保护） */
static int json_append(char *buf, size_t cap, int off, const char *fmt, ...)
{
    int w;
    va_list ap;

    if (off < 0 || (size_t)off >= cap) {
        return off;
    }
    va_start(ap, fmt);
    w = vsnprintf(buf + off, cap - off, fmt, ap);
    va_end(ap);
    if (w < 0) {
        return off;
    }
    /* vsnprintf 返回的是"本该写入"的长度，不是实际写入长度。
     * 截断时必须把偏移顶到缓冲末尾，否则 off 会跑到 cap 之外，
     * 后面的 buf[p]=0 就会砍掉已经写好的内容，输出半截 JSON。 */
    if ((size_t)w >= cap - (size_t)off) {
        return (int)cap - 1;
    }
    return off + w;
}

static int build_json(char *buf, size_t cap)
{
    disc_status_t d;
    gps_status_t  g;
    ntp_stats_t   n;
    char ip[32];
    char utc[40];
    uint32_t sec = 0, frac = 0;
    static char lines[GPS_RAW_LINES][GPS_RAW_LINE_LEN];   /* ~1.9KB，放静态避免压 http 栈 */
    int raw_cnt, p;
    /* 预留 8 字节给收尾的 "]}"，保证无论内容多长输出的都是合法 JSON */
    size_t lim = (cap > 16u) ? (cap - 8u) : cap;

    discipline_get_status(&d);
    gps_get_status(&g);
    ntp_get_stats(&n);
    eth_if_get_ip(ip, sizeof(ip));

    bool have_utc = discipline_get_utc(tb_now_us(), &sec, &frac);
    if (have_utc) {
        fmt_utc(sec, utc, sizeof(utc));
    } else {
        snprintf(utc, sizeof(utc), "---------- --:--:--");
    }

    p = json_append(buf, lim, 0,
        "{"
        "\"ip\":\"%s\",\"link\":%s,"
        "\"utc\":\"%s\",\"unix\":%lu,\"frac\":%lu,"
        "\"locked\":%s,\"stratum\":%u,\"li\":%u,\"precision\":%d,"
        "\"holdover_ms\":%lu,\"offset_us\":%ld,\"jitter_us\":%ld,\"ppb\":%ld,"
        "\"pps_total\":%lu,\"pps_missed\":%lu,"
        "\"root_disp_us\":%lu,"
        "\"baud\":%lu,\"baud_locked\":%s,"
        "\"fix_valid\":%s,\"fix_quality\":%u,\"sats_used\":%u,\"sats_view\":%u,"
        "\"hdop\":%u.%u,"
        "\"leap_s\":%d,\"leap_expected\":%d,\"leap_ok\":%s,"
        "\"nmea\":%lu,\"ubx_ack\":%lu,\"ubx_nak\":%lu,"
        "\"holdover_valid\":%s,"
        "\"ntp_req\":%lu,\"ntp_resp\":%lu,\"ntp_bad\":%lu,\"ntp_drop\":%lu,"
        "\"rx_ts_used\":%lu,\"rx_ts_miss\":%lu,"
        "\"step\":%lu,\"resync\":%lu,\"slip\":%lu,\"mismatch\":%lu,\"late\":%lu,"
        "\"lag_ms\":%lu,"
        "\"heap\":%lu,"
        "\"uptime_s\":%lu",
        /* 注意：这里不能收尾 '}'——后面还要挂 "raw" 数组。
         * 曾经在这里就写了 '}'，结果拼出 {"..."},"raw":[...] ，
         * 前端 JSON.parse 直接报 "Unexpected non-whitespace character
         * after JSON"，页面永远停在 loading。闭合交给下面的 "raw" 段。 */
        ip, eth_if_link_up() ? "true" : "false",
        utc, (unsigned long)sec, (unsigned long)frac,
        d.locked ? "true" : "false", (unsigned)d.stratum, (unsigned)d.li, CFG_NTP_PRECISION,
        (unsigned long)d.holdover_ms, (long)d.offset_us, (long)d.jitter_us, (long)d.ppb,
        (unsigned long)d.pps_total, (unsigned long)d.pps_missed,
        (unsigned long)d.root_disp_us,
        (unsigned long)g.baud, g.baud_locked ? "true" : "false",
        g.fix_valid ? "true" : "false", (unsigned)g.fix_quality,
        (unsigned)g.sats_used, (unsigned)g.sats_view,
        (unsigned)(g.hdop_x10 / 10), (unsigned)(g.hdop_x10 % 10),
        (int)g.leap_s, (int)g.leap_expected, g.leap_mismatch ? "false" : "true",
        (unsigned long)g.nmea_count, (unsigned long)g.ubx_ack, (unsigned long)g.ubx_nak,
        d.holdover_valid ? "true" : "false",
        (unsigned long)n.requests, (unsigned long)n.responses,
        (unsigned long)n.bad, (unsigned long)n.dropped,
        (unsigned long)n.rx_ts_used, (unsigned long)n.rx_ts_miss,
        (unsigned long)d.step_count, (unsigned long)d.resync_count,
        (unsigned long)d.slip_count, (unsigned long)d.mismatch_count,
        (unsigned long)d.late_count,
        (unsigned long)d.lag_ms,
        (unsigned long)esp_get_free_heap_size(),
        (unsigned long)((uint32_t)(tb_now_us() / 1000000ULL)));

    /* 最近 GPS_RAW_LINES 条 GGA/GSA/ZDA 原始报文（排障用）。
     * 入库时已去掉 \r\n 并把控制字符/引号/反斜杠换成 '?'，
     * 这里再兜一层：非 '$' 开头的（空串、半截帧）直接不输出。 */
    raw_cnt = gps_get_last_nmea(lines, GPS_RAW_LINES);
    p = json_append(buf, lim, p, ",\"raw\":[");
    for (int i = 0, out = 0; i < raw_cnt; i++) {
        if (lines[i][0] != '$') {
            continue;
        }
        p = json_append(buf, lim, p, out ? ",\"%s\"" : "\"%s\"", lines[i]);
        out++;
    }
    /* 收尾用完整 cap，保证 "]}"" 一定有地方写 */
    p = json_append(buf, cap, p, "]}");

    if (p < 0) {
        p = 0;
    }
    if ((size_t)p >= cap) {
        p = (int)cap - 1;
    }
    buf[p] = 0;
    return p;
}

/* ==================== HTTP 处理 =================================== */
#define STR_HELPER(x) #x
#define STR(x)        STR_HELPER(x)

static const char *PAGE_HEAD =
    "<!DOCTYPE html><html lang=\"zh-CN\"><head><meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>GPS/PPS NTP Server</title>"
    "<style>"
    "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;"
    "background:#f5f6f8;color:#1f2328;margin:0;padding:24px}"
    "h1{font-size:20px;margin:0 0 4px}"
    "h2{font-size:14px;margin:20px 0 8px;color:#57606a}"
    ".sub{color:#57606a;font-size:13px;margin-bottom:16px}"
    ".grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(170px,1fr));gap:10px}"
    ".c{background:#fff;border:1px solid #d0d7de;border-radius:8px;padding:10px 12px}"
    ".k{font-size:11px;color:#57606a;letter-spacing:.03em}"
    ".v{font-size:16px;font-weight:600;margin-top:2px;word-break:break-all}"
    ".ok{color:#1a7f37}.warn{color:#9a6700}.bad{color:#cf222e}"
    ".raw{background:#fff;border:1px solid #d0d7de;border-radius:8px;padding:10px 12px;"
    "font:12px/1.55 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;"
    "color:#24292f;white-space:pre-wrap;word-break:break-all;max-height:340px;overflow:auto}"
    "table{border-collapse:collapse;width:100%;background:#fff;"
    "border:1px solid #d0d7de;border-radius:8px;overflow:hidden}"
    "td{padding:6px 10px;border-bottom:1px solid #eaeef2;font-size:13px}"
    "td:first-child{color:#57606a;width:40%}"
    "</style></head><body>"
    "<h1>GPS / PPS NTP Server</h1>"
    "<div class=\"sub\" id=\"sub\">loading...</div>"
    "<div class=\"grid\" id=\"g\"></div>"
    "<h2>明细</h2><table id=\"t\"></table>"
    "<h2>最近 NMEA 原始报文（GNGSA / GNGGA / GNZDA，共 " STR(GPS_RAW_LINES) " 条，最新在上）</h2>"
    "<pre id=\"raw\" class=\"raw\">loading...</pre>"
    "<script>"
    "const F=(v,f)=>{if(v===null||v===undefined)return '-';if(typeof v==='boolean')return v?'是':'否';"
    "return String(v)+(f?(' '+f):'');};"
    "function cls(v){if(v===true)return 'ok';if(v===false)return 'bad';return '';}"
    "async function load(){"
    " try{"
    "  const res=await fetch('/status.json',{cache:'no-store'});"
    "  const d=await res.json();"
    "  const items=["
    "   ['锁定',d.locked,''],"
    "   ['Stratum',d.stratum,''],"
    "   ['LI',d.li,''],"
    "   ['UTC',d.utc,''],"
    "   ['PPS 相位误差',d.offset_us,'us'],"
    "   ['抖动',d.jitter_us,'us'],"
    "   ['频率修正',d.ppb,'ppb'],"
    "   ['守时',d.holdover_ms,'ms'],"
    "   ['卫星(解算/可见)',d.sats_used+'/'+d.sats_view,''],"
    "   ['Fix Quality',d.fix_quality,''],"
    "   ['HDOP',d.hdop,''],"
    "   ['波特率',d.baud,''],"
    "   ['NTP 请求',d.ntp_req,''],"
    "   ['NTP 丢弃',d.ntp_drop,''],"
    "   ['入站硬件时戳',d.rx_ts_used,''],"
    "   ['闰秒(模块/期望)',d.leap_s+' / '+d.leap_expected,'']"
    "  ];"
    "  document.getElementById('g').innerHTML=items.map(([k,v,u])=>"
    "`<div class=\"c\"><div class=\"k\">${k}</div>"
    "<div class=\"v ${cls(v)}\">${F(v,u)}</div></div>`).join('');"
    "  const rows=[['IP',d.ip],['Link',d.link?'UP':'DOWN'],['Precision',d.precision],"
    "   ['Root Dispersion',d.root_disp_us+' us'],['PPS 累计/丢失',d.pps_total+' / '+d.pps_missed],"
    "   ['NMEA 语句',d.nmea],['UBX ACK/NAK',d.ubx_ack+' / '+d.ubx_nak],"
    "   ['NTP 响应/非法',d.ntp_resp+' / '+d.ntp_bad],"
    "   ['伺服 阶跃/重对齐',d.step+' / '+d.resync],"
    "   ['伺服 整秒滑移/失配',d.slip+' / '+d.mismatch],"
    "   ['NMEA 滞后/跨秒次数',d.lag_ms+' ms / '+d.late],"
    "   ['空闲堆',d.heap+' B'],"
    "   ['入站时戳回退',d.rx_ts_miss],['Uptime',d.uptime_s+' s']];"
    "  const tb=document.getElementById('t');"
    "  if(tb)tb.innerHTML=rows.map(([k,v])=>"
    "`<tr><td>${k}</td><td>${F(v)}</td></tr>`).join('');"
    "  const raw=(d.raw||[]).slice().reverse();"
    "  document.getElementById('raw').textContent="
    "raw.length?raw.join('\\n'):'(尚未收到 GGA/GSA/ZDA)';"
    "  document.getElementById('sub').textContent='自动刷新 · 2 s · '+new Date().toLocaleTimeString();"
    " }catch(e){"
    "  const s=document.getElementById('sub');"
    "  if(s)s.textContent='状态读取失败: '+e;"
    " }"
    "}"
    "load();setInterval(load,2000);"
    "</script></body></html>";

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, PAGE_HEAD, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t json_handler(httpd_req_t *req)
{
    static char json[6144];          /* 含最多 15 条原始报文，留足余量避免截断 */
    int n;

    /* 缓冲是静态的，而 httpd 每个连接一个任务：多个页面/tab 同时轮询会并发
     * 写同一块内存，拼出互相撕裂的半截 JSON。用互斥量串行化
     * （不能用临界区——httpd_resp_send 会在锁内阻塞在 socket 上）。 */
    if (s_json_mux) {
        xSemaphoreTake(s_json_mux, portMAX_DELAY);
    }
    n = build_json(json, sizeof(json));
    httpd_resp_set_type(req, "application/json");
    n = httpd_resp_send(req, json, n);
    if (s_json_mux) {
        xSemaphoreGive(s_json_mux);
    }
    return (esp_err_t)n;
}

/* 浏览器会自动请求 favicon，没有 404 噪声 */
static esp_err_t favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "204 No Content");
    return httpd_resp_send(req, NULL, 0);
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = CFG_HTTP_PORT;
    cfg.stack_size       = CFG_MONITOR_STACK;
    cfg.task_priority    = CFG_MONITOR_PRIO;
    cfg.max_open_sockets = 5;
    cfg.lru_purge_enable = true;
    cfg.uri_match_fn     = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP 服务器启动失败");
        return NULL;
    }
    httpd_uri_t root_uri = {
        .uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = NULL
    };
    httpd_uri_t json_uri = {
        .uri = "/status.json", .method = HTTP_GET, .handler = json_handler, .user_ctx = NULL
    };
    httpd_uri_t fav_uri = {
        .uri = "/favicon.ico", .method = HTTP_GET, .handler = favicon_handler, .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &root_uri);
    httpd_register_uri_handler(server, &json_uri);
    httpd_register_uri_handler(server, &fav_uri);
    ESP_LOGI(TAG, "Web 面板已启动，端口 %d", CFG_HTTP_PORT);
    return server;
}

/* ==================== 串口诊断 ==================================== */
static void log_diag(void)
{
    disc_status_t d;
    gps_status_t  g;
    ntp_stats_t   n;
    char ip[32], utc[40], hold[24];
    uint32_t sec = 0, frac = 0;

    discipline_get_status(&d);
    gps_get_status(&g);
    ntp_get_stats(&n);
    eth_if_get_ip(ip, sizeof(ip));

    if (discipline_get_utc(tb_now_us(), &sec, &frac)) {
        fmt_utc(sec, utc, sizeof(utc));
    } else {
        snprintf(utc, sizeof(utc), "未同步");
    }

    snprintf(hold, sizeof(hold), d.holdover_valid ? "%lums" : "n/a(无PPS)",
             (unsigned long)d.holdover_ms);

    ESP_LOGI(TAG,
             "%s %s | UTC %s | %s stratum=%u li=%u | off=%lldus jit=%lldus ppb=%ld | "
             "hold=%s pps=%lu/%lu | sats=%u/%u fixq=%u hdop=%u.%u %s | baud=%lu | "
             "ntp req=%lu resp=%lu bad=%lu drop=%lu | rxTs %lu/%lu | linkup=%lu | "
             "sv %lu/%lu/%lu/%lu/%lu | lag=%lums | heap=%luKB",
             eth_if_link_up() ? "LINK UP " : "LINK DOWN", ip, utc,
             d.locked ? "LOCKED  " : "UNLOCKED", (unsigned)d.stratum, (unsigned)d.li,
             (long long)d.offset_us, (long long)d.jitter_us, (long)d.ppb,
             hold, (unsigned long)d.pps_total, (unsigned long)d.pps_missed,
             (unsigned)g.sats_used, (unsigned)g.sats_view, (unsigned)g.fix_quality,
             (unsigned)(g.hdop_x10 / 10), (unsigned)(g.hdop_x10 % 10),
             g.fix_valid ? "[FIX]" : "[NOFIX]",
             (unsigned long)g.baud,
             (unsigned long)n.requests, (unsigned long)n.responses,
             (unsigned long)n.bad, (unsigned long)n.dropped,
             (unsigned long)n.rx_ts_used, (unsigned long)n.rx_ts_miss,
             (unsigned long)eth_if_link_up_count(),
             /* 阶跃 / 重对齐 / 整秒滑移 / 失配 / 传输跨秒 */
             (unsigned long)d.step_count, (unsigned long)d.resync_count,
             (unsigned long)d.slip_count, (unsigned long)d.mismatch_count,
             (unsigned long)d.late_count,
             (unsigned long)d.lag_ms,
             (unsigned long)(esp_get_free_heap_size() / 1024UL));

    if (g.leap_valid && g.leap_mismatch) {
        ESP_LOGW(TAG, "闰秒不一致：模块 leapS=%d，内置表期望 %d",
                 (int)g.leap_s, (int)g.leap_expected);
    }
}

/* ==================== 任务 ========================================= */
static void monitor_task(void *arg)
{
    (void)arg;
    if (esp_task_wdt_add(NULL) != ESP_OK) {
        ESP_LOGW(TAG, "TWDT 订阅失败（任务看门狗未启用？）");
    }
    start_webserver();

    while (1) {
        esp_task_wdt_reset();
        eth_if_periodic();
        log_diag();
        vTaskDelay(pdMS_TO_TICKS(CFG_LOG_INTERVAL_MS));
    }
}

void monitor_start(void)
{
    if (!s_json_mux) {
        s_json_mux = xSemaphoreCreateMutex();
    }
    /* HTTP / 诊断属于网络侧，固定到 CORE_NET，不打扰时间核 */
    xTaskCreatePinnedToCore(monitor_task, "monitor", CFG_MONITOR_STACK, NULL,
                            CFG_MONITOR_PRIO, NULL, CFG_CORE_NET);
}
