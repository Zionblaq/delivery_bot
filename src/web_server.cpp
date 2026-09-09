#include "Bot_config.h"
#include "Bot_state.h"
#include "servo.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char *TAG = "WEBSERVER";
static const char *WIFI_TAG = "WIFI_AP";

static const char PAGE_HTML[] = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <title>Delivery Bot</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: sans-serif; text-align: center; background:#111; color:#eee; }
    button { font-size: 20px; margin: 6px; padding: 14px 22px; border-radius: 8px; border: none; user-select: none; }
    .nav { background:#2b6cb0; color:#fff; }
    .mode { background:#38a169; color:#fff; }
    .lock { background:#a03838; color:#fff; }
    .power-off { background:#c53030; color:#fff; }
    .power-on { background:#2f855a; color:#fff; }
    .grid { display:grid; grid-template-columns: 1fr 1fr 1fr; width: 260px; margin: 20px auto; }
    input, select { font-size: 20px; padding: 8px; width: 160px; text-align:center; }
    #pinDisplay { font-size: 28px; letter-spacing: 6px; margin: 10px; }
  </style>
</head>
<body>
  <h2>Delivery Bot Control</h2>
  <h3>Power</h3>
  <button class="power-off" onclick="setPower(false)">Power Off</button>
  <button class="power-on" onclick="setPower(true)">Power On</button>
  <div id="powerStatus"></div>
  <hr>
  <div class="grid">
    <div></div>
    <button class="nav" id="btnF">&#8593;</button>
    <div></div>
    <button class="nav" id="btnL">&#8592;</button>
    <button class="nav" id="btnS">&#9632;</button>
    <button class="nav" id="btnR">&#8594;</button>
    <div></div>
    <button class="nav" id="btnB">&#8595;</button>
    <div></div>
  </div>
  <p><button class="mode" onclick="toggleMode()">Switch Mode: <span id="modeLabel">Manual</span></button></p>
  <hr>
  <h3>Delivery Location</h3>
  <select id="locationSelect" onchange="setLocation()">
    <option value="A">Location A</option>
    <option value="B">Location B</option>
    <option value="C">Location C</option>
  </select>
  <div id="locationStatus"></div>
  <h3>Place Order</h3>
  <button onclick="placeOrder()">Generate Delivery PIN</button>
  <div id="pinDisplay"></div>
  <h3>Unlock Delivery</h3>
  <input id="pinInput" maxlength="4" placeholder="4-digit PIN">
  <button onclick="unlock()">Unlock</button>
  <div id="unlockStatus"></div>
  <hr>
  <h3>Manual Compartment Lock</h3>
  <button class="lock" onclick="manualLock(true)">Lock Now</button>
  <button class="lock" onclick="manualLock(false)">Unlock Now</button>
  <div id="lockStatus"></div>
<script>
let manual = true;
window.onload = function() {
  fetch('/status').then(r => r.text()).then(mode => {
    manual = (mode !== 'line');
    document.getElementById('modeLabel').innerText = manual ? 'Manual' : 'Line Follow';
  });
};
function sendCmd(dir) { fetch('/cmd?dir=' + dir); }
function bindHold(id, dir) {
  const el = document.getElementById(id);
  let holdInterval = null;
  const start = (e) => {
    e.preventDefault();
    sendCmd(dir);
    holdInterval = setInterval(() => sendCmd(dir), 200);
  };
  const stop = (e) => {
    e.preventDefault();
    if (holdInterval) { clearInterval(holdInterval); holdInterval = null; }
    sendCmd('stop');
  };
  el.addEventListener('mousedown', start);
  el.addEventListener('mouseup', stop);
  el.addEventListener('mouseleave', stop);
  el.addEventListener('touchstart', start);
  el.addEventListener('touchend', stop);
  el.addEventListener('touchcancel', stop);
}
bindHold('btnF', 'forward');
bindHold('btnB', 'backward');
bindHold('btnL', 'left');
bindHold('btnR', 'right');
document.getElementById('btnS').addEventListener('click', () => sendCmd('stop'));
function toggleMode() {
  manual = !manual;
  const value = manual ? 'manual' : 'line';
  document.getElementById('modeLabel').innerText = manual ? 'Manual' : 'Line Follow';
  fetch('/mode?value=' + value);
}
function setLocation() {
  const loc = document.getElementById('locationSelect').value;
  fetch('/location?loc=' + loc).then(r => r.text()).then(msg => {
    document.getElementById('locationStatus').innerText = 'Target: ' + loc;
  });
}
function placeOrder() {
  fetch('/order').then(r => r.text()).then(pin => {
    document.getElementById('pinDisplay').innerText = 'PIN: ' + pin;
  });
}
function unlock() {
  const pin = document.getElementById('pinInput').value;
  fetch('/unlock?pin=' + pin).then(r => r.text()).then(msg => {
    document.getElementById('unlockStatus').innerText = msg;
  });
}
function manualLock(lock) {
  fetch('/manual_lock?state=' + (lock ? 'lock' : 'unlock')).then(r => r.text()).then(msg => {
    document.getElementById('lockStatus').innerText = msg;
  });
}
function setPower(on) {
  fetch('/power?action=' + (on ? 'on' : 'off')).then(r => r.text()).then(msg => {
    document.getElementById('powerStatus').innerText = msg;
  });
}
</script>
</body>
</html>
)HTML";

void wifi_ap_start() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.ap.ssid, WIFI_AP_SSID, sizeof(wifi_config.ap.ssid));
    wifi_config.ap.ssid_len = strlen(WIFI_AP_SSID);
    strncpy((char *)wifi_config.ap.password, WIFI_AP_PASS, sizeof(wifi_config.ap.password));
    wifi_config.ap.channel = WIFI_AP_CHANNEL;
    wifi_config.ap.max_connection = WIFI_AP_MAX_CONN;
    wifi_config.ap.authmode = (strlen(WIFI_AP_PASS) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(WIFI_TAG, "AP started - SSID: %s, connect then browse to 192.168.4.1", WIFI_AP_SSID);
}

static esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, PAGE_HTML, HTTPD_RESP_USE_STRLEN);
}

static bool get_query_param(httpd_req_t *req, const char *key, char *out, size_t out_len) {
    char query[128];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
    return httpd_query_key_value(query, key, out, out_len) == ESP_OK;
}

static esp_err_t status_handler(httpd_req_t *req) {
    RobotMode mode = robot_state_get_mode();
    const char *resp = (mode == RobotMode::LINE_FOLLOW) ? "line" : "manual";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t cmd_handler(httpd_req_t *req) {
    char dir[16] = {0};
    if (!get_query_param(req, "dir", dir, sizeof(dir))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing dir");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Button press: dir=%s", dir);
    if (strcmp(dir, "forward") == 0)       robot_state_set_manual_cmd(ManualCmd::FORWARD);
    else if (strcmp(dir, "backward") == 0) robot_state_set_manual_cmd(ManualCmd::BACKWARD);
    else if (strcmp(dir, "left") == 0)     robot_state_set_manual_cmd(ManualCmd::LEFT);
    else if (strcmp(dir, "right") == 0)    robot_state_set_manual_cmd(ManualCmd::RIGHT);
    else                                   robot_state_set_manual_cmd(ManualCmd::STOP);
    httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t mode_handler(httpd_req_t *req) {
    char value[16] = {0};
    if (!get_query_param(req, "value", value, sizeof(value))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing value");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Mode button press: value=%s", value);
    if (strcmp(value, "line") == 0) {
        robot_state_set_mode(RobotMode::LINE_FOLLOW);
    } else {
        robot_state_set_mode(RobotMode::MANUAL);
        robot_state_set_manual_cmd(ManualCmd::STOP);
    }
    httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t order_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Generate Delivery PIN button press");
    std::string pin = robot_state_new_order();
    ESP_LOGI(TAG, "New PIN generated: %s", pin.c_str());
    httpd_resp_send(req, pin.c_str(), HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static void spawn_unlock_task() {
    xTaskCreate(servo_unlock_task, "servo_unlock_task", 3072, NULL, 5, NULL);
}

static esp_err_t unlock_handler(httpd_req_t *req) {
    char pin[8] = {0};
    if (!get_query_param(req, "pin", pin, sizeof(pin))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing pin");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Unlock button press: pin=%s", pin);
    if (robot_state_get_unlock_active()) {
        httpd_resp_send(req, "Already unlocking", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    if (robot_state_check_pin(std::string(pin))) {
        ESP_LOGI(TAG, "PIN correct - unlocking compartment");
        spawn_unlock_task();
        httpd_resp_send(req, "Unlocked - compartment open", HTTPD_RESP_USE_STRLEN);
    } else {
        ESP_LOGW(TAG, "PIN incorrect");
        httpd_resp_send(req, "Incorrect PIN", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

static esp_err_t manual_lock_handler(httpd_req_t *req) {
    char state[8] = {0};
    if (!get_query_param(req, "state", state, sizeof(state))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing state");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Manual lock button press: state=%s", state);
    if (robot_state_get_unlock_active()) {
        httpd_resp_send(req, "Busy - timed unlock in progress", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    if (strcmp(state, "lock") == 0) {
        servo_close();
        httpd_resp_send(req, "Compartment locked", HTTPD_RESP_USE_STRLEN);
    } else if (strcmp(state, "unlock") == 0) {
        servo_open();
        httpd_resp_send(req, "Compartment unlocked", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "state must be lock or unlock");
        return ESP_FAIL;
    }
    return ESP_OK;
}

static esp_err_t power_handler(httpd_req_t *req) {
    char action[16] = {0};
    if (!get_query_param(req, "action", action, sizeof(action))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing action");
        return ESP_FAIL;
    }
    bool off = (strcmp(action, "off") == 0);
    ESP_LOGI(TAG, "Power button press: %s", off ? "OFF" : "ON");
    robot_state_set_powered_off(off);
    httpd_resp_send(req, off ? "Powered off" : "Powered on", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t location_handler(httpd_req_t *req) {
    char loc[8] = {0};
    if (!get_query_param(req, "loc", loc, sizeof(loc))) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing loc");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Location button press: loc=%s", loc);
    DeliveryLocation target = DeliveryLocation::NONE;
    if (strcmp(loc, "A") == 0) target = DeliveryLocation::LOC_A;
    else if (strcmp(loc, "B") == 0) target = DeliveryLocation::LOC_B;
    else if (strcmp(loc, "C") == 0) target = DeliveryLocation::LOC_C;
    robot_state_set_target_location(target);
    httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void web_server_start() {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 12;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return;
    }
    httpd_uri_t root_uri        = { "/",            HTTP_GET, root_handler,        NULL };
    httpd_uri_t status_uri      = { "/status",       HTTP_GET, status_handler,      NULL };
    httpd_uri_t cmd_uri         = { "/cmd",         HTTP_GET, cmd_handler,         NULL };
    httpd_uri_t mode_uri        = { "/mode",        HTTP_GET, mode_handler,        NULL };
    httpd_uri_t order_uri       = { "/order",       HTTP_GET, order_handler,       NULL };
    httpd_uri_t unlock_uri      = { "/unlock",      HTTP_GET, unlock_handler,      NULL };
    httpd_uri_t manual_lock_uri = { "/manual_lock", HTTP_GET, manual_lock_handler, NULL };
    httpd_uri_t power_uri       = { "/power",       HTTP_GET, power_handler,       NULL };
    httpd_uri_t location_uri    = { "/location",    HTTP_GET, location_handler,    NULL };

    httpd_register_uri_handler(server, &root_uri);
    httpd_register_uri_handler(server, &status_uri);
    httpd_register_uri_handler(server, &cmd_uri);
    httpd_register_uri_handler(server, &mode_uri);
    httpd_register_uri_handler(server, &order_uri);
    httpd_register_uri_handler(server, &unlock_uri);
    httpd_register_uri_handler(server, &manual_lock_uri);
    httpd_register_uri_handler(server, &power_uri);
    httpd_register_uri_handler(server, &location_uri);
}