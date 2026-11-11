
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
#include <cstdio>

// Implemented in Task.cpp
const char *control_journey_status();
void control_request_play();
void control_request_pause();

static const char *TAG = "WEBSERVER";
static const char *WIFI_TAG = "WIFI_AP";

static const char PAGE_HTML[] = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Delivery Bot</title>
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">

<style>
:root {
  --bg1:#0b1020;
  --bg2:#151b34;
  --card:rgba(255,255,255,0.06);
  --line:rgba(255,255,255,0.10);
  --text:#e8ecf8;
  --muted:#8e98b8;
  --accent:#4f8cff;
  --accent2:#7c5cff;
  --ok:#2fd18b;
  --bad:#ff5c6c;
  --warn:#ffb84d;
}

* {
  box-sizing:border-box;
  -webkit-tap-highlight-color:transparent;
}

body {
  margin:0;
  min-height:100vh;
  text-align:center;
  color:var(--text);
  font-family:-apple-system,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
  background:
    radial-gradient(900px 500px at 50% -10%, #28357a 0%, transparent 65%),
    linear-gradient(160deg,var(--bg1),var(--bg2));
  user-select:none;
  touch-action:manipulation;
}

.wrap {
  max-width:440px;
  margin:0 auto;
  padding:16px 14px 40px;
}

header {
  display:flex;
  align-items:center;
  justify-content:space-between;
  margin:4px 2px 8px;
}

h1 {
  margin:0;
  font-size:21px;
  letter-spacing:.3px;
  text-align:left;
}

.sub {
  font-size:12px;
  color:var(--muted);
  text-align:left;
  margin-top:2px;
}

.power {
  display:flex;
  flex-direction:column;
  align-items:center;
  gap:4px;
}

#powerBtn {
  width:60px;
  height:60px;
  border-radius:50%;
  padding:0;
  border:0;
  color:#fff;
  cursor:pointer;
  display:flex;
  align-items:center;
  justify-content:center;
  background:linear-gradient(145deg,#ff6b78,#c2303f);
  box-shadow:0 0 0 4px rgba(255,92,108,.15),0 6px 16px rgba(0,0,0,.4);
}

#powerBtn.on {
  background:linear-gradient(145deg,#3be39b,#17a26a);
}

#powerLabel {
  font-size:11px;
  font-weight:700;
  letter-spacing:1.5px;
  color:var(--muted);
}

.card {
  background:var(--card);
  border:1px solid var(--line);
  border-radius:20px;
  padding:16px;
  margin:12px 0;
  box-shadow:0 8px 24px rgba(0,0,0,.25);
}

.card h3 {
  margin:0 0 12px;
  font-size:12px;
  text-transform:uppercase;
  letter-spacing:1.4px;
  color:var(--muted);
  font-weight:700;
  text-align:left;
}

button {
  font:inherit;
  color:#fff;
  border:0;
  border-radius:14px;
  padding:13px 16px;
  cursor:pointer;
}

button:active {
  transform:scale(.96);
}

.primary {
  background:linear-gradient(135deg,var(--accent),var(--accent2));
}

.ghost {
  background:rgba(255,255,255,.09);
  border:1px solid var(--line);
}

.danger {
  background:linear-gradient(135deg,#ff6b78,#c2303f);
}

.good {
  background:linear-gradient(135deg,#3be39b,#17a26a);
}

.row {
  display:flex;
  gap:10px;
}

.row > button {
  flex:1;
}

svg {
  display:block;
}

.seg {
  display:flex;
  background:rgba(255,255,255,.07);
  border-radius:14px;
  padding:4px;
}

.seg button {
  flex:1;
  background:transparent;
  color:var(--muted);
  padding:11px 0;
  font-weight:600;
}

.seg button.on {
  background:linear-gradient(135deg,var(--accent),var(--accent2));
  color:#fff;
}

.grid {
  display:grid;
  grid-template-columns:repeat(3,72px);
  gap:10px;
  justify-content:center;
}

.grid button {
  height:72px;
  padding:0;
  font-size:26px;
  border-radius:20px;
  background:rgba(255,255,255,.09);
  border:1px solid var(--line);
}

.grid .stop {
  background:linear-gradient(135deg,#ff6b78,#c2303f);
}

.status {
  display:flex;
  align-items:center;
  justify-content:center;
  gap:10px;
  font-size:15px;
  margin-bottom:14px;
}

.dot {
  width:12px;
  height:12px;
  border-radius:50%;
  background:var(--muted);
}

.dot.running {
  background:var(--ok);
}

.dot.obstacle,.dot.stopped {
  background:var(--bad);
}

.dot.paused,.dot.waiting {
  background:var(--warn);
}

.dot.arrived {
  background:var(--accent);
}

.ctl {
  display:flex;
  align-items:center;
  justify-content:center;
  gap:8px;
  font-weight:600;
}

#picker {
  display:none;
  margin-top:12px;
}

.chips {
  display:flex;
  gap:10px;
}

.chips button {
  flex:1;
  font-size:20px;
  font-weight:700;
  padding:14px 0;
}

.chips button.sel {
  background:linear-gradient(135deg,var(--accent),var(--accent2));
}

.hint {
  font-size:12px;
  color:var(--muted);
  margin-top:10px;
}

#locationStatus {
  margin-top:12px;
  font-size:15px;
}

#pinDisplay {
  font-size:34px;
  letter-spacing:10px;
  font-weight:300;
  margin:2px 0 12px;
  font-variant-numeric:tabular-nums;
}

input {
  width:100%;
  font-size:20px;
  padding:12px;
  margin-bottom:12px;
  text-align:center;
  letter-spacing:8px;
  color:var(--text);
  background:rgba(255,255,255,.07);
  border:1px solid var(--line);
  border-radius:14px;
  outline:none;
}

.msg {
  font-size:13px;
  color:var(--muted);
  min-height:18px;
  margin-top:10px;
}

.alert {
  display:none;
  position:fixed;
  top:10px;
  left:10px;
  right:10px;
  z-index:10;
  max-width:420px;
  margin:0 auto;
  padding:14px 16px;
  border-radius:16px;
  font-weight:600;
  font-size:14px;
  line-height:1.35;
  box-shadow:0 10px 30px rgba(0,0,0,.5);
}

.alert.bad {
  background:linear-gradient(135deg,#ff5c6c,#c2303f);
}

.alert.warn {
  background:linear-gradient(135deg,#ffb84d,#e08a12);
  color:#1a1200;
}

.alert.ok {
  background:linear-gradient(135deg,#3be39b,#17a26a);
}

/* SENSOR DASHBOARD */

.sensor-grid {
  display:grid;
  grid-template-columns:repeat(2,1fr);
  gap:10px;
}

.sensor-box {
  background:rgba(255,255,255,.06);
  border:1px solid var(--line);
  border-radius:14px;
  padding:13px 8px;
}

.sensor-name {
  color:var(--muted);
  font-size:11px;
  font-weight:700;
  text-transform:uppercase;
}

.sensor-value {
  font-size:25px;
  font-weight:700;
  margin-top:7px;
  font-variant-numeric:tabular-nums;
}

.sensor-unit {
  color:var(--muted);
  font-size:11px;
}

.ir-row {
  display:flex;
  justify-content:space-between;
  align-items:center;
  gap:8px;
  margin:12px 0;
}

.ir-name {
  font-size:13px;
  color:var(--muted);
}

.ir-state {
  font-size:11px;
  font-weight:700;
  padding:7px 10px;
  border-radius:9px;
  background:rgba(255,255,255,.08);
}

.ir-state.detected {
  background:rgba(47,209,139,.18);
  color:var(--ok);
}

.ir-state.clear {
  background:rgba(255,255,255,.08);
  color:var(--muted);
}

.sensor-status {
  font-size:11px;
  color:var(--muted);
  margin-top:12px;
}

.sensor-status.error {
  color:var(--bad);
}

</style>
</head>

<body>

<div id="alertBar" class="alert"></div>

<div class="wrap">

<header>
  <div>
    <h1>Delivery Bot</h1>
    <div class="sub">192.168.4.1</div>
  </div>

  <div class="power">
    <button id="powerBtn" class="on" onclick="togglePower()">
      <svg viewBox="0 0 24 24" width="28" height="28"
        fill="none" stroke="currentColor" stroke-width="2.4"
        stroke-linecap="round">
        <path d="M12 3v8"/>
        <path d="M6.3 6.8a8 8 0 1 0 11.4 0"/>
      </svg>
    </button>
    <div id="powerLabel">ON</div>
  </div>
</header>

<div class="card">
  <h3>Mode</h3>
  <div class="seg">
    <button id="segManual" class="on" onclick="setMode('manual')">Manual</button>
    <button id="segLine" onclick="setMode('line')">Line Follow</button>
  </div>
</div>

<div class="card" id="manualCard">
  <h3>Drive</h3>
  <div class="grid">
    <div></div><button id="btnF">&#8593;</button><div></div>
    <button id="btnL">&#8592;</button>
    <button id="btnS" class="stop">&#9632;</button>
    <button id="btnR">&#8594;</button>
    <div></div><button id="btnB">&#8595;</button><div></div>
  </div>
</div>

<div class="card" id="journeyCard" style="display:none">
  <h3>Journey</h3>
  <div class="status">
    <span id="journeyDot" class="dot"></span>
    <span id="journeyText">Waiting for a location</span>
  </div>

  <div class="row">
    <button class="danger" onclick="journeyStop()">Stop</button>
    <button class="good" onclick="journeyPlay()">Play</button>
  </div>
</div>

<div class="card">
  <h3>Delivery Location</h3>
  <button class="primary" style="width:100%" onclick="togglePicker()">Pick a Location</button>

  <div id="picker">
    <div class="chips">
      <button id="chipA" class="ghost" onclick="pickLocation('A')">A</button>
      <button id="chipB" class="ghost" onclick="pickLocation('B')">B</button>
      <button id="chipC" class="ghost" onclick="pickLocation('C')">C</button>
    </div>

    <button class="ghost" style="width:100%;margin-top:10px"
      onclick="pickLocation('none')">Cancel pick</button>

    <div class="hint">A = straight · B = right · C = left</div>
  </div>

  <div id="locationStatus">No location picked</div>
</div>

<div class="card">
  <h3>Order &amp; Unlock</h3>
  <div id="pinDisplay">----</div>
  <input id="pinInput" maxlength="4" inputmode="numeric" placeholder="4-digit PIN">

  <div class="row">
    <button class="primary" onclick="placeOrder()">Place Order</button>
    <button class="good" onclick="unlock()">Unlock</button>
  </div>

  <div class="msg" id="unlockStatus"></div>
</div>

<div class="card">
  <h3>Compartment Lock</h3>

  <div class="row">
    <button class="danger" onclick="manualLock(true)">Lock Now</button>
    <button class="ghost" onclick="manualLock(false)">Unlock Now</button>
  </div>

  <div class="msg" id="lockStatus"></div>
</div>

<!-- NEW SENSOR MONITORING CARD -->

<div class="card">
  <h3>Sensor Monitoring</h3>

  <h3>Ultrasonic Distances</h3>

  <div class="sensor-grid">

    <div class="sensor-box">
      <div class="sensor-name">Front Left</div>
      <div class="sensor-value" id="usLeft">--</div>
      <div class="sensor-unit">centimetres</div>
    </div>

    <div class="sensor-box">
      <div class="sensor-name">Front Centre</div>
      <div class="sensor-value" id="usCenter">--</div>
      <div class="sensor-unit">centimetres</div>
    </div>

    <div class="sensor-box">
      <div class="sensor-name">Front Right</div>
      <div class="sensor-value" id="usRight">--</div>
      <div class="sensor-unit">centimetres</div>
    </div>

    <div class="sensor-box">
      <div class="sensor-name">Rear</div>
      <div class="sensor-value" id="usRear">--</div>
      <div class="sensor-unit">centimetres</div>
    </div>

  </div>

  <h3 style="margin-top:20px">Infrared Line Sensors</h3>

  <div class="ir-row">
    <span class="ir-name">Left IR</span>
    <span id="irLeft" class="ir-state">--</span>
  </div>

  <div class="ir-row">
    <span class="ir-name">Centre IR</span>
    <span id="irCenter" class="ir-state">--</span>
  </div>

  <div class="ir-row">
    <span class="ir-name">Right IR</span>
    <span id="irRight" class="ir-state">--</span>
  </div>

  <div id="sensorStatus" class="sensor-status">
    Waiting for sensor data...
  </div>
</div>

</div>

<script>

const $ = (id) => document.getElementById(id);

let mode = 'manual';
let powerOn = true;
let lastJourney = '';
let lastLocal = 0;

function touchLocal() {
  lastLocal = Date.now();
}

function recentLocal() {
  return Date.now() - lastLocal < 1500;
}

/* POWER */

function applyPower() {
  $('powerBtn').classList.toggle('on', powerOn);
  $('powerLabel').innerText = powerOn ? 'ON' : 'OFF';
}

function togglePower() {
  powerOn = !powerOn;
  touchLocal();
  applyPower();

  fetch('/power?action=' + (powerOn ? 'on' : 'off'));
}

/* MODE */

function applyMode(m) {
  mode = m;

  $('segManual').classList.toggle('on', m === 'manual');
  $('segLine').classList.toggle('on', m === 'line');

  $('manualCard').style.display = m === 'manual' ? 'block' : 'none';
  $('journeyCard').style.display = m === 'line' ? 'block' : 'none';

  if (m === 'manual') showJourney('idle');
}

function setMode(m) {
  touchLocal();
  applyMode(m);
  fetch('/mode?value=' + m);
}

/* MANUAL DRIVE */

function sendCmd(dir) {
  fetch('/cmd?dir=' + dir);
}

function bindHold(id, dir) {
  const el = $(id);
  let holdInterval = null;

  const start = (e) => {
    e.preventDefault();
    sendCmd(dir);

    if (holdInterval) clearInterval(holdInterval);

    holdInterval = setInterval(() => sendCmd(dir), 200);
  };

  const stop = (e) => {
    e.preventDefault();

    if (holdInterval) {
      clearInterval(holdInterval);
      holdInterval = null;
    }

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

$('btnS').addEventListener('click', () => sendCmd('stop'));

/* JOURNEY */

const JOURNEY_TEXT = {
  idle: 'Waiting for a location',
  waiting: 'At the junction: pick a location',
  running: 'On its way',
  obstacle: 'Obstacle ahead: stopped',
  paused: 'Paused',
  arrived: 'Arrived',
  stopped: 'Stopped: line lost'
};

function journeyStop() {
  fetch('/pause');
}

function journeyPlay() {
  fetch('/play');
}

function showJourney(j) {
  $('journeyText').innerText = JOURNEY_TEXT[j] || j;
  $('journeyDot').className = 'dot ' + j;

  let msg = '';
  let cls = '';

  if (j === 'obstacle') {
    msg = 'Obstacle detected. The robot has stopped.';
    cls = 'bad';
  } else if (j === 'paused') {
    msg = 'Journey paused. Press Play to continue.';
    cls = 'warn';
  } else if (j === 'arrived') {
    msg = 'Arrived. Enter the delivery PIN to unlock.';
    cls = 'ok';
  } else if (j === 'stopped') {
    msg = 'The robot stopped because it lost the line.';
    cls = 'bad';
  }

  const bar = $('alertBar');

  if (msg && mode === 'line') {
    bar.className = 'alert ' + cls;
    bar.innerText = msg;
    bar.style.display = 'block';
  } else {
    bar.style.display = 'none';
  }

  if (j === 'obstacle' && lastJourney !== 'obstacle' && navigator.vibrate) {
    navigator.vibrate([250, 100, 250]);
  }

  lastJourney = j;
}

/* LOCATION */

function togglePicker() {
  const p = $('picker');
  p.style.display = p.style.display === 'block' ? 'none' : 'block';
}

function showTarget(t) {
  const ok = (t === 'A' || t === 'B' || t === 'C');

  $('locationStatus').innerText =
    ok ? 'Picked: Location ' + t : 'No location picked';

  ['A', 'B', 'C'].forEach(x => {
    $('chip' + x).classList.toggle('sel', x === t);
  });
}

function pickLocation(loc) {
  fetch('/location?loc=' + loc).then(() => {
    showTarget(loc);
    $('picker').style.display = 'none';
  });
}

/* ORDER AND LOCK */

function placeOrder() {
  fetch('/order')
    .then(r => r.text())
    .then(pin => {
      $('pinDisplay').innerText = pin;
    });
}

function unlock() {
  const pin = $('pinInput').value;

  fetch('/unlock?pin=' + pin)
    .then(r => r.text())
    .then(msg => {
      $('unlockStatus').innerText = msg;
    });
}

function manualLock(lock) {
  fetch('/manual_lock?state=' + (lock ? 'lock' : 'unlock'))
    .then(r => r.text())
    .then(msg => {
      $('lockStatus').innerText = msg;
    });
}

/* EXISTING LIVE ROBOT STATUS */

function applyLive(txt) {
  const p = txt.trim().split(',');

  if (p.length < 4) return;

  if (!recentLocal()) {
    if (p[0] !== mode) applyMode(p[0]);

    const on = (p[1] === 'on');

    if (on !== powerOn) {
      powerOn = on;
      applyPower();
    }
  }

  showTarget(p[2]);
  showJourney(p[3]);
}

function poll() {
  fetch('/live')
    .then(r => r.text())
    .then(applyLive)
    .catch(() => {});
}

setInterval(poll, 1000);
poll();

/* NEW SENSOR MONITORING JAVASCRIPT */

function showDistance(id, value) {
  const number = Number(value);

  if (Number.isFinite(number) && number >= 0) {
    $(id).innerText = number.toFixed(1);
  } else {
    $(id).innerText = '--';
  }
}

function showIR(id, value) {
  const detected = Number(value) === 1;
  const element = $(id);

  element.innerText = detected ? 'LINE DETECTED' : 'NO LINE';
  element.className = 'ir-state ' + (detected ? 'detected' : 'clear');
}

function updateSensors(data) {

  showDistance('usLeft', data.front_left);
  showDistance('usCenter', data.front_center);
  showDistance('usRight', data.front_right);
  showDistance('usRear', data.rear);

  showIR('irLeft', data.ir_left);
  showIR('irCenter', data.ir_center);
  showIR('irRight', data.ir_right);

  $('sensorStatus').innerText = 'Sensor connection active';
  $('sensorStatus').className = 'sensor-status';
}

function pollSensors() {
  fetch('/sensors')
    .then(response => {
      if (!response.ok) {
        throw new Error('Sensor request failed');
      }

      return response.json();
    })
    .then(updateSensors)
    .catch(() => {
      $('sensorStatus').innerText = 'Sensor connection error';
      $('sensorStatus').className = 'sensor-status error';
    });
}

setInterval(pollSensors, 500);
pollSensors();

</script>
</body>
</html>
)HTML";

/* WIFI ACCESS POINT */

void wifi_ap_start() {

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t wifi_config = {};

    strncpy(
        (char *)wifi_config.ap.ssid,
        WIFI_AP_SSID,
        sizeof(wifi_config.ap.ssid)
    );

    wifi_config.ap.ssid_len = strlen(WIFI_AP_SSID);

    strncpy(
        (char *)wifi_config.ap.password,
        WIFI_AP_PASS,
        sizeof(wifi_config.ap.password)
    );

    wifi_config.ap.channel = WIFI_AP_CHANNEL;
    wifi_config.ap.max_connection = WIFI_AP_MAX_CONN;

    wifi_config.ap.authmode =
        (strlen(WIFI_AP_PASS) == 0)
        ? WIFI_AUTH_OPEN
        : WIFI_AUTH_WPA2_PSK;

    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    ESP_LOGI(
        WIFI_TAG,
        "AP started - SSID: %s, browse to 192.168.4.1",
        WIFI_AP_SSID
    );
}

/* ROOT PAGE */

static esp_err_t root_handler(httpd_req_t *req) {

    httpd_resp_set_type(req, "text/html");

    return httpd_resp_send(
        req,
        PAGE_HTML,
        HTTPD_RESP_USE_STRLEN
    );
}

/* QUERY PARAMETER HELPER */

static bool get_query_param(
    httpd_req_t *req,
    const char *key,
    char *out,
    size_t out_len
) {

    char query[128];

    if (httpd_req_get_url_query_str(
        req,
        query,
        sizeof(query)
    ) != ESP_OK) {
        return false;
    }

    return httpd_query_key_value(
        query,
        key,
        out,
        out_len
    ) == ESP_OK;
}

/* STATUS */

static esp_err_t status_handler(httpd_req_t *req) {

    RobotMode mode = robot_state_get_mode();

    const char *resp =
        (mode == RobotMode::LINE_FOLLOW) ? "line" : "manual";

    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/* MANUAL MOVEMENT */

static esp_err_t cmd_handler(httpd_req_t *req) {

    char dir[16] = {0};

    if (!get_query_param(req, "dir", dir, sizeof(dir))) {
        httpd_resp_send_err(
            req,
            HTTPD_400_BAD_REQUEST,
            "missing dir"
        );

        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Button press: dir=%s", dir);

    if (strcmp(dir, "forward") == 0)
        robot_state_set_manual_cmd(ManualCmd::FORWARD);

    else if (strcmp(dir, "backward") == 0)
        robot_state_set_manual_cmd(ManualCmd::BACKWARD);

    else if (strcmp(dir, "left") == 0)
        robot_state_set_manual_cmd(ManualCmd::LEFT);

    else if (strcmp(dir, "right") == 0)
        robot_state_set_manual_cmd(ManualCmd::RIGHT);

    else
        robot_state_set_manual_cmd(ManualCmd::STOP);

    httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/* MODE */

static esp_err_t mode_handler(httpd_req_t *req) {

    char value[16] = {0};

    if (!get_query_param(req, "value", value, sizeof(value))) {
        httpd_resp_send_err(
            req,
            HTTPD_400_BAD_REQUEST,
            "missing value"
        );

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

/* ORDER */

static esp_err_t order_handler(httpd_req_t *req) {

    ESP_LOGI(TAG, "Generate Delivery PIN button press");

    std::string pin = robot_state_new_order();

    ESP_LOGI(TAG, "New PIN generated: %s", pin.c_str());

    httpd_resp_send(req, pin.c_str(), HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/* SERVO UNLOCK */

static void spawn_unlock_task() {
    xTaskCreate(
        servo_unlock_task,
        "servo_unlock_task",
        3072,
        NULL,
        5,
        NULL
    );
}

static esp_err_t unlock_handler(httpd_req_t *req) {

    char pin[8] = {0};

    if (!get_query_param(req, "pin", pin, sizeof(pin))) {
        httpd_resp_send_err(
            req,
            HTTPD_400_BAD_REQUEST,
            "missing pin"
        );

        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Unlock button press: pin=%s", pin);

    if (robot_state_get_unlock_active()) {
        httpd_resp_send(
            req,
            "Already unlocking",
            HTTPD_RESP_USE_STRLEN
        );

        return ESP_OK;
    }

    if (robot_state_check_pin(std::string(pin))) {

        ESP_LOGI(TAG, "PIN correct - unlocking compartment");

        spawn_unlock_task();

        httpd_resp_send(
            req,
            "Unlocked - compartment open",
            HTTPD_RESP_USE_STRLEN
        );

    } else {

        ESP_LOGW(TAG, "PIN incorrect");

        httpd_resp_send(
            req,
            "Incorrect PIN",
            HTTPD_RESP_USE_STRLEN
        );
    }

    return ESP_OK;
}

/* MANUAL LOCK */

static esp_err_t manual_lock_handler(httpd_req_t *req) {

    char state[8] = {0};

    if (!get_query_param(req, "state", state, sizeof(state))) {
        httpd_resp_send_err(
            req,
            HTTPD_400_BAD_REQUEST,
            "missing state"
        );

        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Manual lock button press: state=%s", state);

    if (robot_state_get_unlock_active()) {

        httpd_resp_send(
            req,
            "Busy - timed unlock in progress",
            HTTPD_RESP_USE_STRLEN
        );

        return ESP_OK;
    }

    if (strcmp(state, "lock") == 0) {

        servo_close();

        httpd_resp_send(
            req,
            "Compartment locked",
            HTTPD_RESP_USE_STRLEN
        );

    } else if (strcmp(state, "unlock") == 0) {

        servo_open();

        httpd_resp_send(
            req,
            "Compartment unlocked",
            HTTPD_RESP_USE_STRLEN
        );

    } else {

        httpd_resp_send_err(
            req,
            HTTPD_400_BAD_REQUEST,
            "state must be lock or unlock"
        );

        return ESP_FAIL;
    }

    return ESP_OK;
}

/* POWER */

static esp_err_t power_handler(httpd_req_t *req) {

    char action[16] = {0};

    if (!get_query_param(req, "action", action, sizeof(action))) {
        httpd_resp_send_err(
            req,
            HTTPD_400_BAD_REQUEST,
            "missing action"
        );

        return ESP_FAIL;
    }

    bool off = (strcmp(action, "off") == 0);

    ESP_LOGI(
        TAG,
        "Power button press: %s",
        off ? "OFF" : "ON"
    );

    robot_state_set_powered_off(off);

    httpd_resp_send(
        req,
        off ? "Powered off" : "Powered on",
        HTTPD_RESP_USE_STRLEN
    );

    return ESP_OK;
}

/* DELIVERY LOCATION */

static esp_err_t location_handler(httpd_req_t *req) {

    char loc[8] = {0};

    if (!get_query_param(req, "loc", loc, sizeof(loc))) {
        httpd_resp_send_err(
            req,
            HTTPD_400_BAD_REQUEST,
            "missing loc"
        );

        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Location button press: loc=%s", loc);

    DeliveryLocation target = DeliveryLocation::NONE;

    if (strcmp(loc, "A") == 0)
        target = DeliveryLocation::LOC_A;

    else if (strcmp(loc, "B") == 0)
        target = DeliveryLocation::LOC_B;

    else if (strcmp(loc, "C") == 0)
        target = DeliveryLocation::LOC_C;

    robot_state_set_target_location(target);

    httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/* EXISTING LIVE STATUS */

static esp_err_t live_handler(httpd_req_t *req) {

    const char *mode =
        (robot_state_get_mode() == RobotMode::LINE_FOLLOW)
        ? "line"
        : "manual";

    const char *power =
        robot_state_is_powered_off() ? "off" : "on";

    char loc = '-';

    switch (robot_state_get_target_location()) {

        case DeliveryLocation::LOC_A:
            loc = 'A';
            break;

        case DeliveryLocation::LOC_B:
            loc = 'B';
            break;

        case DeliveryLocation::LOC_C:
            loc = 'C';
            break;

        default:
            break;
    }

    char buf[64];

    snprintf(
        buf,
        sizeof(buf),
        "%s,%s,%c,%s",
        mode,
        power,
        loc,
        control_journey_status()
    );

    httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/* NEW SENSOR DATA ENDPOINT */

static esp_err_t sensors_handler(httpd_req_t *req) {

    float distances[4];
    int lineRaw[3];

    // Get the latest shared sensor readings.
    robot_state_get_distances(distances);
    robot_state_get_line_raw(lineRaw);

    char buf[256];

    int len = snprintf(
        buf,
        sizeof(buf),

        "{"
        "\"front_left\":%.1f,"
        "\"front_center\":%.1f,"
        "\"front_right\":%.1f,"
        "\"rear\":%.1f,"
        "\"ir_left\":%d,"
        "\"ir_center\":%d,"
        "\"ir_right\":%d"
        "}",

        distances[US_FRONT_LEFT],
        distances[US_FRONT_CENTER],
        distances[US_FRONT_RIGHT],
        distances[US_REAR],

        lineRaw[0],
        lineRaw[1],
        lineRaw[2]
    );

    if (len < 0 || len >= sizeof(buf)) {
        httpd_resp_send_err(
            req,
            HTTPD_500_INTERNAL_SERVER_ERROR,
            "Failed to create sensor response"
        );

        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "application/json");

    return httpd_resp_send(req, buf, len);
}

/* PLAY */

static esp_err_t play_handler(httpd_req_t *req) {

    ESP_LOGI(TAG, "Play button press");

    control_request_play();

    httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/* PAUSE */

static esp_err_t pause_handler(httpd_req_t *req) {

    ESP_LOGI(TAG, "Journey stop button press");

    control_request_pause();

    httpd_resp_send(req, "ok", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

/* START WEB SERVER */

void web_server_start() {

    httpd_handle_t server = NULL;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.max_uri_handlers = 16;

    if (httpd_start(&server, &config) != ESP_OK) {

        ESP_LOGE(TAG, "Failed to start HTTP server");

        return;
    }

    httpd_uri_t root_uri = {
        "/", HTTP_GET, root_handler, NULL
    };

    httpd_uri_t status_uri = {
        "/status", HTTP_GET, status_handler, NULL
    };

    httpd_uri_t cmd_uri = {
        "/cmd", HTTP_GET, cmd_handler, NULL
    };

    httpd_uri_t mode_uri = {
        "/mode", HTTP_GET, mode_handler, NULL
    };

    httpd_uri_t order_uri = {
        "/order", HTTP_GET, order_handler, NULL
    };

    httpd_uri_t unlock_uri = {
        "/unlock", HTTP_GET, unlock_handler, NULL
    };

    httpd_uri_t manual_lock_uri = {
        "/manual_lock", HTTP_GET, manual_lock_handler, NULL
    };

    httpd_uri_t power_uri = {
        "/power", HTTP_GET, power_handler, NULL
    };

    httpd_uri_t location_uri = {
        "/location", HTTP_GET, location_handler, NULL
    };

    httpd_uri_t live_uri = {
        "/live", HTTP_GET, live_handler, NULL
    };

    httpd_uri_t sensors_uri = {
        "/sensors", HTTP_GET, sensors_handler, NULL
    };

    httpd_uri_t play_uri = {
        "/play", HTTP_GET, play_handler, NULL
    };

    httpd_uri_t pause_uri = {
        "/pause", HTTP_GET, pause_handler, NULL
    };

    httpd_register_uri_handler(server, &root_uri);
    httpd_register_uri_handler(server, &status_uri);
    httpd_register_uri_handler(server, &cmd_uri);
    httpd_register_uri_handler(server, &mode_uri);
    httpd_register_uri_handler(server, &order_uri);
    httpd_register_uri_handler(server, &unlock_uri);
    httpd_register_uri_handler(server, &manual_lock_uri);
    httpd_register_uri_handler(server, &power_uri);
    httpd_register_uri_handler(server, &location_uri);
    httpd_register_uri_handler(server, &live_uri);

    // Register the new sensor endpoint.
    httpd_register_uri_handler(server, &sensors_uri);

    httpd_register_uri_handler(server, &play_uri);
    httpd_register_uri_handler(server, &pause_uri);

    ESP_LOGI(TAG, "Web server started successfully");
}
