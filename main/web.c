/*
 * 网页配置后台（从 orbis-w518/main/wifi_manager.c 的配网部分移植）
 *
 *   http://<设备IP>/            配网表单（Wi-Fi / HA / MQTT / 设备名 / 登录口令）
 *   http://<设备IP>/save        POST：写 NVS 后重启
 *   http://<设备IP>/ota         网页 OTA 页面
 *   http://<设备IP>/ota/upload  POST：.bin 直接写进下一个 OTA 分区
 *
 * 全部页面要 HTTP Basic Auth，账号来自 app_settings（首次开机是 secrets.h 里的
 * APP_SECRET_WEB_USER / APP_SECRET_WEB_PASS）。
 *
 * 内存约束（内部 RAM 运行时只剩 50~90KB）：HTTP 任务栈 8KB、最多 4 个 socket、
 * 5 个 URI 槽位；页面缓冲优先放 PSRAM，OTA 接收缓冲放内部 RAM（写 flash 时不能用 PSRAM）。
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_system.h"

#include "mbedtls/base64.h"

#include "app_settings.h"
#include "net.h"
#include "ota.h"
#include "web.h"

static const char *TAG = "web";

#define WEB_TASK_STACK 6144    /* 旧工程实测值：默认 4KB 会在 /save 上栈溢出重启 */
#define WEB_PAGE_MAX     5120     /* 转义后最坏情况约 4KB，正常 2~3KB */
#define WEB_BODY_MAX     1024     /* /save 的 x-www-form-urlencoded 请求体上限 */
#define WEB_OTA_CHUNK    4096     /* 与 flash 扇区对齐 */
#define WEB_RETRY_US     (5LL * 1000 * 1000)

static httpd_handle_t s_httpd;
static int64_t s_last_fail_us;

/* 页面缓冲优先用 PSRAM：内部 RAM 要留给 LVGL / TLS / MQTT / Opus */
static char *page_alloc(size_t n)
{
    char *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == NULL) {
        p = malloc(n);
    }
    return p;
}

/* ------------------------------------------------------------------------- */
/* 小缓冲拼接：所有写 HTML 的动作都经过这里，天然带边界，不会有 snprintf 溢出    */
/* ------------------------------------------------------------------------- */

typedef struct {
    char *buf;
    size_t cap;
    size_t len;
    bool full;
} sb_t;

static void sb_init(sb_t *sb, char *buf, size_t cap)
{
    sb->buf = buf;
    sb->cap = cap;
    sb->len = 0;
    sb->full = false;
    if (cap > 0) {
        buf[0] = '\0';
    }
}

static void sb_putc(sb_t *sb, char c)
{
    if (sb->len + 1 >= sb->cap) {
        sb->full = true;
        return;
    }
    sb->buf[sb->len++] = c;
    sb->buf[sb->len] = '\0';
}

static void sb_puts(sb_t *sb, const char *s)
{
    if (s == NULL) {
        return;
    }
    while (*s) {
        sb_putc(sb, *s++);
    }
}

static void sb_vprintf(sb_t *sb, const char *fmt, va_list ap)
{
    if (sb->len + 1 >= sb->cap) {
        sb->full = true;
        return;
    }
    const int n = vsnprintf(sb->buf + sb->len, sb->cap - sb->len, fmt, ap);
    if (n < 0) {
        sb->full = true;
        return;
    }
    if ((size_t)n >= sb->cap - sb->len) {
        sb->len = sb->cap - 1;
        sb->full = true;
    } else {
        sb->len += (size_t)n;
    }
}

static void sb_printf(sb_t *sb, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    sb_vprintf(sb, fmt, ap);
    va_end(ap);
}

/* 表单回填：SSID / URL 里的 ' & < > " 必须转义，否则页面会被撑坏 */
static void sb_html(sb_t *sb, const char *s)
{
    if (s == NULL) {
        return;
    }
    for (; *s; s++) {
        switch (*s) {
        case '&': sb_puts(sb, "&amp;");  break;
        case '<': sb_puts(sb, "&lt;");   break;
        case '>': sb_puts(sb, "&gt;");   break;
        case '"': sb_puts(sb, "&quot;"); break;
        case '\'': sb_puts(sb, "&#39;"); break;
        default:  sb_putc(sb, *s);       break;
        }
    }
}

/* ------------------------------------------------------------------------- */
/* HTTP Basic Auth                                                           */
/* ------------------------------------------------------------------------- */

static esp_err_t send_401(httpd_req_t *req)
{
    httpd_resp_set_status(req, "401 Unauthorized");
    httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Orbis\", charset=\"UTF-8\"");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_send(req, "需要登录", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t send_err_zh(httpd_req_t *req, const char *status, const char *msg)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_send(req, msg, HTTPD_RESP_USE_STRLEN);
}

/** 校验 Authorization: Basic base64(user:pass)，账号口令来自 app_settings */
static bool http_auth_ok(httpd_req_t *req)
{
    char hdr[192];
    if (httpd_req_get_hdr_value_str(req, "Authorization", hdr, sizeof(hdr)) != ESP_OK) {
        return false;
    }

    const app_settings_t *cfg = app_settings_get();
    char cred[128];
    const int n = snprintf(cred, sizeof(cred), "%s:%s", cfg->web_user, cfg->web_pass);
    if (n <= 0 || n >= (int)sizeof(cred)) {
        return false;
    }

    unsigned char b64[192];
    size_t olen = 0;
    if (mbedtls_base64_encode(b64, sizeof(b64) - 1, &olen, (const unsigned char *)cred, (size_t)n) != 0) {
        return false;
    }
    b64[olen] = '\0';

    char expect[224];
    snprintf(expect, sizeof(expect), "Basic %s", (char *)b64);
    return strcmp(hdr, expect) == 0;
}

/* ------------------------------------------------------------------------- */
/* 表单解析                                                                   */
/* ------------------------------------------------------------------------- */

static int hex2int(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* 截断到合法 UTF-8 边界：中文显示名被切成半个字符会变成乱码 */
static void utf8_fix_tail(char *s, size_t len)
{
    if (len == 0) {
        return;
    }
    size_t i = len;
    /* 最多回退 3 个字节（UTF-8 一个字符最长 4 字节） */
    while (i > 0 && (len - i) < 4 && ((unsigned char)s[i - 1] & 0xC0) == 0x80) {
        i--;
    }
    if (i == 0) {
        return;
    }
    const unsigned char lead = (unsigned char)s[i - 1];
    size_t need = 0;
    if ((lead & 0x80) == 0x00) need = 1;
    else if ((lead & 0xE0) == 0xC0) need = 2;
    else if ((lead & 0xF0) == 0xE0) need = 3;
    else if ((lead & 0xF8) == 0xF0) need = 4;
    else need = 0;      /* 非法首字节，原样留着 */
    if (need == 0) {
        return;
    }
    if (i - 1 + need > len) {
        s[i - 1] = '\0';    /* 字符不完整，丢掉 */
    }
}

/* 边解码边写入定长缓冲：解码后只会更短，不会写爆 out */
static void form_decode(const char *src, size_t len, char *out, size_t out_len)
{
    size_t o = 0;
    for (size_t i = 0; i < len && (o + 1) < out_len; i++) {
        const char c = src[i];
        if (c == '+') {
            out[o++] = ' ';
        } else if (c == '%' && (i + 2) < len) {
            const int hi = hex2int(src[i + 1]);
            const int lo = hex2int(src[i + 2]);
            if (hi >= 0 && lo >= 0) {
                out[o++] = (char)(hi * 16 + lo);
                i += 2;
            } else {
                out[o++] = c;
            }
        } else {
            out[o++] = c;
        }
    }
    out[o] = '\0';
    if (o + 1 == out_len) {
        utf8_fix_tail(out, o);
    }
}

/*
 * 从 urlencoded body 取一个字段。
 * 两个坑：1) 不能就地改 body（后面的字段会找不到）；2) 必须按字段边界匹配，
 * 否则找 "pass=" 会先命中 "mqtt_pass="。
 */
static bool form_get(const char *body, const char *key, char *out, size_t out_len)
{
    if (out == NULL || out_len == 0) {
        return false;
    }
    out[0] = '\0';

    const size_t klen = strlen(key);
    const char *p = body;
    while ((p = strstr(p, key)) != NULL) {
        const bool boundary = (p == body) || (p[-1] == '&');
        if (boundary && p[klen] == '=') {
            break;
        }
        p += klen;
    }
    if (p == NULL) {
        return false;
    }

    p += klen + 1;
    const char *end = strchr(p, '&');
    const size_t len = end ? (size_t)(end - p) : strlen(p);
    form_decode(p, len, out, out_len);
    return true;
}

/** 设备 ID 会进 MQTT 主题，只允许 a-z 0-9 - _ */
static bool device_id_ok(const char *s)
{
    if (s == NULL || s[0] == '\0') {
        return false;
    }
    size_t n = 0;
    for (const char *p = s; *p; p++, n++) {
        const char c = *p;
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' || c == '_')) {
            return false;
        }
    }
    return n < APP_DEVICE_ID_MAX;
}

/* ------------------------------------------------------------------------- */
/* 页面                                                                       */
/* ------------------------------------------------------------------------- */

static const char PAGE_CSS[] =
    "<style>"
    "*{box-sizing:border-box}"
    "body{font-family:system-ui,-apple-system,'PingFang SC','Microsoft YaHei',sans-serif;"
    "margin:0 auto;padding:18px 16px 40px;max-width:560px;background:#111;color:#eee;"
    "font-size:15px;line-height:1.5;-webkit-text-size-adjust:100%}"
    "h1,h2{font-size:20px;margin:0 0 8px}"
    "p{color:#9aa0a6;font-size:13px;margin:0 0 16px}"
    "label{display:block;margin:16px 0 6px;font-size:13px;color:#9aa0a6}"
    "input{width:100%;padding:12px;border-radius:10px;border:1px solid #333;"
    "background:#1b1b1b;color:#eee;font-size:16px}"
    "input:focus{outline:none;border-color:#3b82f6}"
    "button{margin-top:24px;width:100%;padding:14px;border:0;border-radius:10px;"
    "background:#3b82f6;color:#fff;font-size:16px;font-weight:600}"
    "button:disabled{opacity:.5}"
    "pre{background:#000;color:#0f0;padding:10px;border-radius:10px;margin-top:14px;"
    "white-space:pre-wrap;word-break:break-all;font-size:12px}"
    ".hint{font-size:12px;color:#777;margin-top:14px}"
    "a{color:#3b82f6}"
    "</style>";

static void page_head(sb_t *sb, const char *title)
{
    sb_puts(sb, "<!DOCTYPE html><html lang='zh-CN'><head><meta charset='utf-8'>"
                "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                "<title>");
    sb_html(sb, title);
    sb_puts(sb, "</title>");
    sb_puts(sb, PAGE_CSS);
    sb_puts(sb, "</head><body>");
}

static void form_input(sb_t *sb, const char *label, const char *name, const char *value,
                       const char *type, size_t maxlen, const char *placeholder)
{
    sb_puts(sb, "<label for='");
    sb_puts(sb, name);
    sb_puts(sb, "'>");
    sb_html(sb, label);
    sb_puts(sb, "</label><input id='");
    sb_puts(sb, name);
    sb_puts(sb, "' name='");
    sb_puts(sb, name);
    sb_puts(sb, "' type='");
    sb_puts(sb, type);
    sb_puts(sb, "'");
    if (strcmp(type, "password") == 0) {
        /* 密码框不回填，也别让浏览器拿登录密码来填 Wi-Fi 密码 */
        sb_puts(sb, " autocomplete='new-password'");
    }
    if (maxlen > 0) {
        sb_printf(sb, " maxlength='%u'", (unsigned)maxlen);
    }
    if (placeholder != NULL) {
        sb_puts(sb, " placeholder='");
        sb_html(sb, placeholder);
        sb_puts(sb, "'");
    }
    sb_puts(sb, " value='");
    sb_html(sb, value);
    sb_puts(sb, "'>");
}

static esp_err_t root_get(httpd_req_t *req)
{
    if (!http_auth_ok(req)) {
        return send_401(req);
    }

    const app_settings_t *cfg = app_settings_get();
    char *page = page_alloc(WEB_PAGE_MAX);
    if (page == NULL) {
        return httpd_resp_send_500(req);
    }

    sb_t sb;
    sb_init(&sb, page, WEB_PAGE_MAX);
    page_head(&sb, "圆屏设置");
    sb_puts(&sb, "<h1>圆屏设置</h1>"
                 "<p>填好 Wi-Fi 和 Home Assistant 的连接信息，点保存后设备自动重启。"
                 "密码和令牌留空表示保持原值。</p>"
                 "<form method='post' action='/save' autocomplete='off'>");
    form_input(&sb, "Wi-Fi 名称（SSID）", "ssid", cfg->wifi_ssid, "text", APP_WIFI_SSID_MAX - 1, NULL);
    form_input(&sb, "Wi-Fi 密码", "pass", "", "password", APP_WIFI_PASS_MAX - 1, "已保存；留空表示不修改");
    form_input(&sb, "Home Assistant 地址", "url", cfg->ha_url, "text", APP_HA_URL_MAX - 1,
               "https://ha.example.com");
    form_input(&sb, "HA 长期访问令牌", "token", "", "password", APP_HA_TOKEN_MAX - 1,
               "已保存；留空表示不修改");
    form_input(&sb, "MQTT 服务器（留空 = 不启用）", "mqtt_host", cfg->mqtt_host, "text",
               APP_MQTT_HOST_MAX - 1, "192.168.1.10");
    sb_printf(&sb,
              "<label for='mqtt_port'>MQTT 端口</label>"
              "<input id='mqtt_port' name='mqtt_port' type='number' inputmode='numeric'"
              " min='1' max='65535' value='%d'>", cfg->mqtt_port);
    form_input(&sb, "MQTT 用户名", "mqtt_user", cfg->mqtt_user, "text", APP_MQTT_USER_MAX - 1, NULL);
    form_input(&sb, "MQTT 密码", "mqtt_pass", "", "password", APP_MQTT_PASS_MAX - 1,
               "已保存；留空表示不修改");
    form_input(&sb, "设备 ID（MQTT 主题里的名字）", "device_id", cfg->device_id, "text",
               APP_DEVICE_ID_MAX - 1, "orbis-f118");
    form_input(&sb, "显示名（HA 里看到的名字）", "device_name", cfg->device_name, "text",
               APP_DEVICE_NAME_MAX - 1, NULL);
    form_input(&sb, "设置页用户名", "web_user", cfg->web_user, "text", APP_WEB_USER_MAX - 1, NULL);
    form_input(&sb, "设置页密码", "web_pass", "", "password", APP_WEB_PASS_MAX - 1,
               "已保存；留空表示不修改");
    sb_puts(&sb, "<button type='submit'>保存并重启</button></form>"
                 "<p class='hint'>保存后设备立即重启。换过 Wi-Fi 之后 IP 可能变，"
                 "请到路由器客户端列表里按设备名找它。</p>"
                 "<p><a href='/ota'>固件升级（网页 OTA）→</a></p>"
                 "</body></html>");

    if (sb.full) {
        ESP_LOGW(TAG, "配置页被截断（%u/%u 字节），说明某项配置格外长", (unsigned)sb.len,
                 (unsigned)WEB_PAGE_MAX);
    }

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    const esp_err_t err = httpd_resp_send(req, page, (ssize_t)sb.len);
    free(page);
    return err;
}

static esp_err_t save_post(httpd_req_t *req)
{
    if (!http_auth_ok(req)) {
        return send_401(req);
    }

    const int total = req->content_len;
    if (total <= 0) {
        return send_err_zh(req, "400 Bad Request", "没有收到表单内容");
    }
    if (total >= (int)WEB_BODY_MAX) {
        return send_err_zh(req, "400 Bad Request", "表单内容过大");
    }

    char body[WEB_BODY_MAX];
    int received = 0;
    int timeouts = 0;
    while (received < total) {
        const int r = httpd_req_recv(req, body + received, (size_t)(total - received));
        if (r == HTTPD_SOCK_ERR_TIMEOUT && ++timeouts <= 3) {
            continue;
        }
        if (r <= 0) {
            return send_err_zh(req, "400 Bad Request", "读取表单内容失败");
        }
        received += r;
        timeouts = 0;
    }
    body[received] = '\0';

    char ssid[APP_WIFI_SSID_MAX] = {0};
    char pass[APP_WIFI_PASS_MAX] = {0};
    char url[APP_HA_URL_MAX] = {0};
    char token[APP_HA_TOKEN_MAX] = {0};
    char mqtt_host[APP_MQTT_HOST_MAX] = {0};
    char mqtt_port[8] = {0};
    char mqtt_user[APP_MQTT_USER_MAX] = {0};
    char mqtt_pass[APP_MQTT_PASS_MAX] = {0};
    char device_id[APP_DEVICE_ID_MAX] = {0};
    char device_name[APP_DEVICE_NAME_MAX] = {0};
    char web_user[APP_WEB_USER_MAX] = {0};
    char web_pass[APP_WEB_PASS_MAX] = {0};

    form_get(body, "ssid", ssid, sizeof(ssid));
    form_get(body, "pass", pass, sizeof(pass));
    form_get(body, "url", url, sizeof(url));
    form_get(body, "token", token, sizeof(token));
    /* MQTT 服务器允许清空（页面上写着"留空 = 不启用"），所以要知道字段在不在 */
    const bool mqtt_host_given = form_get(body, "mqtt_host", mqtt_host, sizeof(mqtt_host));
    form_get(body, "mqtt_port", mqtt_port, sizeof(mqtt_port));
    form_get(body, "mqtt_user", mqtt_user, sizeof(mqtt_user));
    form_get(body, "mqtt_pass", mqtt_pass, sizeof(mqtt_pass));
    form_get(body, "device_id", device_id, sizeof(device_id));
    form_get(body, "device_name", device_name, sizeof(device_name));
    form_get(body, "web_user", web_user, sizeof(web_user));
    form_get(body, "web_pass", web_pass, sizeof(web_pass));

    if (ssid[0] == '\0') {
        return send_err_zh(req, "400 Bad Request", "Wi-Fi 名称（SSID）不能为空");
    }
    if (device_id[0] != '\0' && !device_id_ok(device_id)) {
        return send_err_zh(req, "400 Bad Request", "设备 ID 只能用 a-z 0-9 - _（最长 31 字符）");
    }
    if (web_user[0] != '\0' && strchr(web_user, ':') != NULL) {
        return send_err_zh(req, "400 Bad Request", "设置页用户名里不能有冒号");
    }

    const int port = atoi(mqtt_port);
    if (port < 0 || port > 65535) {
        return send_err_zh(req, "400 Bad Request", "MQTT 端口必须是 1~65535");
    }

    ESP_LOGI(TAG, "保存配置: ssid='%s' ha='%s' mqtt='%s:%d' 设备 id '%s'", ssid, url, mqtt_host,
             port, device_id);

    /* 空密码 / 空令牌 / 空用户名 = 不改（密码框不回填，留空就是"不动"） */
    esp_err_t err = app_settings_update(ssid, pass[0] ? pass : NULL, url[0] ? url : NULL,
                                        token[0] ? token : NULL);
    if (err == ESP_OK) {
        err = app_settings_update_name(device_name[0] ? device_name : NULL);
    }
    if (err == ESP_OK) {
        err = app_settings_update_web(web_user[0] ? web_user : NULL, web_pass[0] ? web_pass : NULL);
    }
    if (err == ESP_OK) {
        err = app_settings_update_mqtt(mqtt_host_given ? mqtt_host : NULL, port,
                                       mqtt_user[0] ? mqtt_user : NULL, mqtt_pass[0] ? mqtt_pass : NULL,
                                       device_id[0] ? device_id : NULL);
    }

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    if (err != ESP_OK) {
        httpd_resp_sendstr(req, "<meta charset='utf-8'><h3>保存失败：NVS 写入错误，配置没生效</h3>"
                                "<p><a href='/'>返回设置页</a></p>");
        ESP_LOGE(TAG, "保存失败（%s），未重启", esp_err_to_name(err));
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "<meta charset='utf-8'><h3>已保存，正在重启…</h3>");
    vTaskDelay(pdMS_TO_TICKS(1200));   /* 等响应发出去再重启 */
    esp_restart();
    return ESP_OK;
}

/* ------------------------------------------------------------------------- */
/* 网页 OTA                                                                   */
/* ------------------------------------------------------------------------- */

static esp_err_t ota_page_get(httpd_req_t *req)
{
    if (!http_auth_ok(req)) {
        return send_401(req);
    }

    const esp_partition_t *run = esp_ota_get_running_partition();
    char *page = page_alloc(WEB_PAGE_MAX);
    if (page == NULL) {
        return httpd_resp_send_500(req);
    }

    sb_t sb;
    sb_init(&sb, page, WEB_PAGE_MAX);
    page_head(&sb, "固件升级");
    sb_puts(&sb, "<h1>固件升级</h1>");
    sb_printf(&sb, "<p>当前版本 <b>%s</b> · 分区 %s · 空闲 %d KB</p>", ota_current_version(),
              run ? run->label : "?", (int)(esp_get_free_heap_size() / 1024));
    sb_puts(&sb, "<input type='file' id='f' accept='.bin'><br><br>"
                 "<button id='b'>上传并升级</button>"
                 "<pre id='s'>选择 .bin 文件后点上传</pre>"
                 "<p><a href='/'>← 返回设置页</a></p>"
                 "<script>"
                 "var f=document.getElementById('f'),b=document.getElementById('b'),"
                 "s=document.getElementById('s');"
                 "b.onclick=function(){var file=f.files[0];if(!file){s.textContent='先选择文件';return;}"
                 "b.disabled=true;s.textContent='上传中…';var x=new XMLHttpRequest();"
                 "x.open('POST','/ota/upload');"
                 "x.setRequestHeader('Content-Type','application/octet-stream');"
                 "x.upload.onprogress=function(e){s.textContent='上传 '+Math.round(e.loaded/1024)+"
                 "' / '+Math.round(e.total/1024)+' KB';};"
                 "x.onload=function(){s.textContent=x.responseText;b.disabled=false;};"
                 "x.onerror=function(){s.textContent='上传失败（连接中断）';b.disabled=false;};"
                 "x.send(file);};"
                 "</script></body></html>");

    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    const esp_err_t err = httpd_resp_send(req, page, (ssize_t)sb.len);
    free(page);
    return err;
}

static esp_err_t ota_upload_post(httpd_req_t *req)
{
    if (!http_auth_ok(req)) {
        return send_401(req);
    }
    if (req->content_len <= 0) {
        return send_err_zh(req, "400 Bad Request", "没有收到固件数据");
    }
    if (ota_busy()) {
        return send_err_zh(req, "409 Conflict", "设备正在做 OTA，等它结束再试");
    }

    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (target == NULL) {
        return send_err_zh(req, "500 Internal Server Error", "没有可用的 OTA 分区");
    }
    if ((size_t)req->content_len > target->size) {
        return send_err_zh(req, "400 Bad Request", "固件超过分区容量");
    }
    ESP_LOGI(TAG, "网页 OTA：接收 %d 字节 -> %s", req->content_len, target->label);

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin 失败: %s", esp_err_to_name(err));
        return send_err_zh(req, "500 Internal Server Error", "初始化 OTA 失败");
    }

    /* 必须用内部 RAM：写 flash 时 cache 关闭，PSRAM 里的数据取不到 */
    char *buf = malloc(WEB_OTA_CHUNK);
    if (buf == NULL) {
        esp_ota_abort(handle);
        return send_err_zh(req, "500 Internal Server Error", "内存不足");
    }

    int remaining = req->content_len;
    int timeouts = 0;
    while (remaining > 0) {
        const int want = (remaining < WEB_OTA_CHUNK) ? remaining : WEB_OTA_CHUNK;
        const int got = httpd_req_recv(req, buf, (size_t)want);
        if (got == HTTPD_SOCK_ERR_TIMEOUT) {
            /* 客户端卡住不能无限等：超时太多次就放弃，别把 httpd 任务占死 */
            if (++timeouts > 30) {
                ESP_LOGE(TAG, "网页 OTA 接收超时，剩余 %d 字节", remaining);
                free(buf);
                esp_ota_abort(handle);
                return send_err_zh(req, "408 Request Timeout", "接收超时，升级已中止");
            }
            continue;
        }
        if (got <= 0) {
            ESP_LOGE(TAG, "网页 OTA 接收中断，剩余 %d 字节", remaining);
            free(buf);
            esp_ota_abort(handle);
            return send_err_zh(req, "400 Bad Request", "传输中断");
        }
        timeouts = 0;
        if (esp_ota_write(handle, buf, (size_t)got) != ESP_OK) {
            free(buf);
            esp_ota_abort(handle);
            return send_err_zh(req, "500 Internal Server Error", "写 flash 失败");
        }
        remaining -= got;
    }
    free(buf);

    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        /* 镜像不合法（不是本芯片的固件 / CRC 错）：原固件还能启动 */
        ESP_LOGE(TAG, "固件校验失败: %s", esp_err_to_name(err));
        return send_err_zh(req, "400 Bad Request", "固件校验失败，可能不是本设备的固件");
    }
    if (esp_ota_set_boot_partition(target) != ESP_OK) {
        return send_err_zh(req, "500 Internal Server Error", "设置启动分区失败");
    }

    ESP_LOGI(TAG, "网页 OTA 完成，1.5 秒后重启");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_sendstr(req, "升级成功，正在重启…");
    vTaskDelay(pdMS_TO_TICKS(1500));
    esp_restart();
    return ESP_OK;
}

/* ------------------------------------------------------------------------- */
/* 启动                                                                       */
/* ------------------------------------------------------------------------- */

esp_err_t web_start(void)
{
    if (s_httpd != NULL) {
        return ESP_OK;
    }

    /* 失败后静默 5 秒：主循环可以放心每轮都调用 web_start()，不会刷日志 */
    const int64_t now = esp_timer_get_time();
    if (s_last_fail_us != 0 && (now - s_last_fail_us) < WEB_RETRY_US) {
        return ESP_FAIL;
    }

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.stack_size = WEB_TASK_STACK;
    cfg.max_uri_handlers = 5;      /* 只用 4 个，留一个余量 */
    cfg.max_open_sockets = 2;      /* 每个 socket 都占 lwIP 缓冲：手机访问够用 */
    cfg.backlog_conn = 3;
    cfg.lru_purge_enable = true;   /* 浏览器并发连接时回收最久不用的那条 */
    cfg.recv_wait_timeout = 10;
    cfg.send_wait_timeout = 10;

    esp_err_t err = httpd_start(&s_httpd, &cfg);
    if (err != ESP_OK) {
        s_httpd = NULL;
        s_last_fail_us = now;
        ESP_LOGE(TAG, "配置页启动失败（%s），5 秒后自动重试", esp_err_to_name(err));
        return err;
    }

    httpd_uri_t uris[] = {
        {.uri = "/",           .method = HTTP_GET,  .handler = root_get},
        {.uri = "/save",       .method = HTTP_POST, .handler = save_post},
        {.uri = "/ota",        .method = HTTP_GET,  .handler = ota_page_get},
        {.uri = "/ota/upload", .method = HTTP_POST, .handler = ota_upload_post},
    };

    bool ok = true;
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        const esp_err_t rerr = httpd_register_uri_handler(s_httpd, &uris[i]);
        if (rerr != ESP_OK) {
            ESP_LOGE(TAG, "注册 %s 失败（%s）", uris[i].uri, esp_err_to_name(rerr));
            ok = false;
        }
    }
    if (!ok) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
        s_last_fail_us = now;
        return ESP_FAIL;
    }

    const app_settings_t *cfg2 = app_settings_get();
    ESP_LOGI(TAG, "配置页已启动: http://%s/ （用户 %s）", net_ip_string(), cfg2->web_user);
    if (!net_wifi_connected()) {
        ESP_LOGW(TAG, "WiFi 还没连上，等拿到 IP 后再访问");
    }
    return ESP_OK;
}

bool web_running(void)
{
    return s_httpd != NULL;
}
