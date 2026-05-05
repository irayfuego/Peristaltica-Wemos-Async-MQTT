#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <ESP_FlexyStepper.h>
#include <time.h>
extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/timers.h"
}
#include <AsyncMqttClient.h>

// ---- MQTT client ----
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// ---- MQTT config (loaded from Preferences) ----
#define MQTT_PORT_DEFAULT 1883
String mqttServer;
int    mqttPort = MQTT_PORT_DEFAULT;
String mqttUser;
String mqttPass;
bool   mqttEnabled = false;

// ---- Preferences ----
Preferences prefs;

void loadMqttConfig() {
    prefs.begin("peristaltica", true);
    mqttServer = prefs.getString("mqtt_srv", "");
    mqttPort   = prefs.getInt("mqtt_port", MQTT_PORT_DEFAULT);
    mqttUser   = prefs.getString("mqtt_usr", "");
    mqttPass   = prefs.getString("mqtt_pwd", "");
    prefs.end();
    mqttEnabled = mqttServer.length() > 0;
}

void saveMqttConfig(const String& srv, int port, const String& usr, const String& pwd) {
    prefs.begin("peristaltica", false);
    prefs.putString("mqtt_srv", srv);
    prefs.putInt("mqtt_port", port);
    prefs.putString("mqtt_usr", usr);
    prefs.putString("mqtt_pwd", pwd);
    prefs.end();
    mqttServer  = srv;
    mqttPort    = port;
    mqttUser    = usr;
    mqttPass    = pwd;
    mqttEnabled = srv.length() > 0;
}

// ---- Web server ----
AsyncWebServer webServer(80);

// ---- Pins ----
const int Dir1          = GPIO_NUM_16;
const int Dir2          = GPIO_NUM_27;
const int Dir3          = GPIO_NUM_14;
const int Step1         = GPIO_NUM_26;
const int Step2         = GPIO_NUM_25;
const int Step3         = GPIO_NUM_17;
const int EnableStepper = 12;

// ---- Motor state ----
long StepsPerMili1, StepsPerMili2, StepsPerMili3;
long StepsToMove1,  StepsToMove2,  StepsToMove3;
long SpeedToMove1,  SpeedToMove2,  SpeedToMove3;
long InitialSteps1, InitialSteps2, InitialSteps3;
long TargetSteps1,  TargetSteps2,  TargetSteps3;
int  StepperStopped;
bool Stepper1Running = false;
bool Stepper2Running = false;
bool Stepper3Running = false;
int  Progress1 = 0, Progress2 = 0, Progress3 = 0;

// ---- EEPROM ----
#define EEPROM_SIZE 512
const int EepromStepsPerMili1 = 0;
const int EepromStepsPerMili2 = 4;
const int EepromStepsPerMili3 = 8;

// ---- Speed defaults ----
const int SPEED_IN_STEPS_PER_SECOND        = 2000;
const int ACCELERATION_IN_STEPS_PER_SECOND = 800;
const int DECELERATION_IN_STEPS_PER_SECOND = 800;

ESP_FlexyStepper stepper1, stepper2, stepper3;

// ---- Progress timing ----
long ProgressPreviousMillis = 0;
const long ProgressInterval = 5000;

TaskHandle_t C0;

// =========================================================
// SCHEDULE
// =========================================================

#define MAX_SCHEDULES 16

struct Schedule {
    bool    enabled;
    uint8_t channel;    // 1-3
    uint8_t days;       // bitmask bit0=Mon … bit6=Sun
    uint8_t hour;
    uint8_t minute;
    float   volume;     // mL
    float   speed;      // mL/min
    char    direction[4]; // "cw" or "ccw"
};

Schedule schedules[MAX_SCHEDULES];

void saveSchedule(int idx) {
    char key[12]; sprintf(key, "sched_%d", idx);
    StaticJsonDocument<192> doc;
    doc["en"]  = schedules[idx].enabled;
    doc["ch"]  = schedules[idx].channel;
    doc["dy"]  = schedules[idx].days;
    doc["hr"]  = schedules[idx].hour;
    doc["mn"]  = schedules[idx].minute;
    doc["vol"] = schedules[idx].volume;
    doc["spd"] = schedules[idx].speed;
    doc["dir"] = schedules[idx].direction;
    char buf[192]; serializeJson(doc, buf);
    prefs.begin("peristaltica", false);
    prefs.putString(key, buf);
    prefs.end();
}

void loadSchedules() {
    for (int i = 0; i < MAX_SCHEDULES; i++) {
        char key[12]; sprintf(key, "sched_%d", i);
        prefs.begin("peristaltica", true);
        String s = prefs.getString(key, "");
        prefs.end();
        if (s.length() == 0) { schedules[i] = {false, 1, 0, 0, 0, 0.0f, 1.0f, "cw"}; continue; }
        StaticJsonDocument<192> doc;
        if (deserializeJson(doc, s)) continue;
        schedules[i].enabled   = doc["en"]  | false;
        schedules[i].channel   = doc["ch"]  | 1;
        schedules[i].days      = doc["dy"]  | 0;
        schedules[i].hour      = doc["hr"]  | 0;
        schedules[i].minute    = doc["mn"]  | 0;
        schedules[i].volume    = doc["vol"] | 0.0f;
        schedules[i].speed     = doc["spd"] | 1.0f;
        strlcpy(schedules[i].direction, doc["dir"] | "cw", 4);
    }
}

void deleteSchedule(int idx) {
    schedules[idx] = {false, 1, 0, 0, 0, 0.0f, 1.0f, "cw"};
    char key[12]; sprintf(key, "sched_%d", idx);
    prefs.begin("peristaltica", false);
    prefs.remove(key);
    prefs.end();
}

// =========================================================
// NTP
// =========================================================

void applyTimezone() {
    prefs.begin("peristaltica", true);
    String tz = prefs.getString("timezone", "CET-1CEST,M3.5.0,M10.5.0/3");
    prefs.end();
    setenv("TZ", tz.c_str(), 1);
    tzset();
}

void setupNTP() {
    applyTimezone();
    configTime(0, 0, "pool.ntp.org", "time.cloudflare.com");
    Serial.print("NTP sync");
    for (int i = 0; i < 20 && time(nullptr) < 100000; i++) { delay(500); Serial.print("."); }
    Serial.println(time(nullptr) > 100000 ? " OK" : " timeout");
}

void checkSchedules() {
    struct tm t;
    if (!getLocalTime(&t, 0)) return;

    // Only check once per minute
    static int lastCheckedMinute = -1;
    if (t.tm_min == lastCheckedMinute) return;
    lastCheckedMinute = t.tm_min;

    // tm_wday: 0=Sun…6=Sat  →  we use 0=Mon…6=Sun
    int dow    = (t.tm_wday == 0) ? 6 : t.tm_wday - 1;
    uint8_t db = 1 << dow;

    for (int i = 0; i < MAX_SCHEDULES; i++) {
        if (!schedules[i].enabled)               continue;
        if (!(schedules[i].days & db))            continue;
        if (schedules[i].hour   != t.tm_hour)     continue;
        if (schedules[i].minute != t.tm_min)      continue;
        // Forward declaration — defined below
        extern void doRun(int, float, float, const char*);
        doRun(schedules[i].channel, schedules[i].volume,
              schedules[i].speed,   schedules[i].direction);
    }
}

// =========================================================
// PROGMEM HTML
// =========================================================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Perist&aacute;ltica</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,sans-serif;background:#f0f2f5;color:#333}
header{background:#1565c0;color:#fff;padding:14px 20px;display:flex;justify-content:space-between;align-items:center}
header h1{font-size:1.1rem;font-weight:600}
.sb{font-size:.78rem;display:flex;gap:14px;align-items:center}
.dot{width:9px;height:9px;border-radius:50%;display:inline-block;margin-right:5px;vertical-align:middle}
.on{background:#69f0ae}.off{background:#ef5350}
main{max-width:980px;margin:20px auto;padding:0 14px;display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:14px}
.card{background:#fff;border-radius:10px;padding:18px;box-shadow:0 1px 5px rgba(0,0,0,.1)}
.card h2{font-size:.95rem;margin-bottom:14px;color:#1565c0;font-weight:600;border-bottom:1px solid #e3e8f0;padding-bottom:8px}
label{display:block;font-size:.76rem;color:#666;margin:9px 0 3px}
input,select{width:100%;padding:7px 9px;border:1px solid #ddd;border-radius:6px;font-size:.88rem;background:#fafafa}
input:focus,select:focus{outline:none;border-color:#1565c0;background:#fff}
.row{display:flex;gap:8px;margin-top:14px}
button{flex:1;padding:9px 6px;border:none;border-radius:6px;font-size:.88rem;cursor:pointer;font-weight:500;transition:opacity .15s,transform .1s}
button:active{opacity:.75;transform:scale(.97)}
.run{background:#1565c0;color:#fff}
.stp{background:#e53935;color:#fff}
.sav{background:#2e7d32;color:#fff}
.rst{background:#bf360c;color:#fff}
.pw{margin-top:12px}
.pl{font-size:.73rem;color:#888;display:flex;justify-content:space-between;margin-bottom:4px}
.pb{height:8px;background:#e0e0e0;border-radius:4px;overflow:hidden}
.pf{height:100%;background:#1565c0;border-radius:4px;transition:width .6s ease}
.full{grid-column:1/-1}
.two{display:grid;grid-template-columns:1fr 1fr;gap:8px}
.msg{font-size:.76rem;margin-top:8px;color:#2e7d32;min-height:1.1em}
/* schedule */
.schd-form{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:8px;margin-top:10px}
.days{display:flex;gap:5px;margin-top:4px;flex-wrap:wrap}
.db{width:30px;height:30px;border-radius:50%;padding:0;flex:none;background:#e8eaf6;color:#3949ab;font-size:.78rem;font-weight:700;border:2px solid transparent}
.db.sel{background:#1565c0;color:#fff;border-color:#0d47a1}
table{width:100%;border-collapse:collapse;font-size:.81rem;margin-top:8px}
th{color:#888;font-weight:500;text-align:left;padding:5px 4px;border-bottom:1px solid #e8eaf6}
td{padding:5px 4px;border-bottom:1px solid #f5f5f5;vertical-align:middle}
.tz-row{display:flex;gap:8px;align-items:flex-end;margin-bottom:12px;flex-wrap:wrap}
.tz-row input{flex:1;min-width:180px}
.tz-row button{flex:none;padding:7px 14px}
.time-badge{font-size:.95rem;font-weight:600;color:#1565c0;font-variant-numeric:tabular-nums}
</style>
</head>
<body>
<header>
  <h1>Bomba Perist&aacute;ltica</h1>
  <div class="sb">
    <span><span class="dot off" id="dW"></span><span id="lW">WiFi</span></span>
    <span><span class="dot off" id="dM"></span>MQTT</span>
    <span class="time-badge" id="clock">--:--:--</span>
  </div>
</header>
<main>

  <!-- Canales 1-3 -->
  <div class="card">
    <h2>Canal 1</h2>
    <label>Volumen (mL)</label><input type="number" id="v1" value="100" min="0.1" step="0.1">
    <label>Velocidad (mL/min)</label><input type="number" id="s1" value="10" min="0.1" step="0.1">
    <label>Direcci&oacute;n</label>
    <select id="d1"><option value="cw">CW &#8635;</option><option value="ccw">CCW &#8634;</option></select>
    <div class="row">
      <button class="run" onclick="run(1)">&#9654; Iniciar</button>
      <button class="stp" onclick="stp(1)">&#9632; Parar</button>
    </div>
    <div class="pw">
      <div class="pl"><span>Progreso</span><span id="p1t">0%</span></div>
      <div class="pb"><div class="pf" id="p1" style="width:0%"></div></div>
    </div>
  </div>

  <div class="card">
    <h2>Canal 2</h2>
    <label>Volumen (mL)</label><input type="number" id="v2" value="100" min="0.1" step="0.1">
    <label>Velocidad (mL/min)</label><input type="number" id="s2" value="10" min="0.1" step="0.1">
    <label>Direcci&oacute;n</label>
    <select id="d2"><option value="cw">CW &#8635;</option><option value="ccw">CCW &#8634;</option></select>
    <div class="row">
      <button class="run" onclick="run(2)">&#9654; Iniciar</button>
      <button class="stp" onclick="stp(2)">&#9632; Parar</button>
    </div>
    <div class="pw">
      <div class="pl"><span>Progreso</span><span id="p2t">0%</span></div>
      <div class="pb"><div class="pf" id="p2" style="width:0%"></div></div>
    </div>
  </div>

  <div class="card">
    <h2>Canal 3</h2>
    <label>Volumen (mL)</label><input type="number" id="v3" value="100" min="0.1" step="0.1">
    <label>Velocidad (mL/min)</label><input type="number" id="s3" value="10" min="0.1" step="0.1">
    <label>Direcci&oacute;n</label>
    <select id="d3"><option value="cw">CW &#8635;</option><option value="ccw">CCW &#8634;</option></select>
    <div class="row">
      <button class="run" onclick="run(3)">&#9654; Iniciar</button>
      <button class="stp" onclick="stp(3)">&#9632; Parar</button>
    </div>
    <div class="pw">
      <div class="pl"><span>Progreso</span><span id="p3t">0%</span></div>
      <div class="pb"><div class="pf" id="p3" style="width:0%"></div></div>
    </div>
  </div>

  <!-- Parar todo -->
  <div class="card full" style="display:flex;align-items:center;gap:20px;flex-wrap:wrap">
    <h2 style="border:none;padding:0;margin:0;flex:none">Control global</h2>
    <button class="stp" style="max-width:200px;flex:unset;padding:10px 24px" onclick="stp(0)">&#9632; Parar todo</button>
  </div>

  <!-- Programación semanal -->
  <div class="card full">
    <h2>&#128197; Programaci&oacute;n semanal</h2>

    <!-- Zona horaria -->
    <div class="tz-row">
      <div style="flex:1">
        <label>Zona horaria (POSIX)</label>
        <input id="tzIn" placeholder="CET-1CEST,M3.5.0,M10.5.0/3">
      </div>
      <button class="sav" onclick="saveTz()">Guardar TZ</button>
    </div>

    <!-- Tabla de horarios existentes -->
    <table>
      <thead>
        <tr>
          <th>Canal</th><th>D&iacute;as</th><th>Hora</th>
          <th>Volumen</th><th>Vel.</th><th>Dir.</th>
          <th>Activo</th><th></th>
        </tr>
      </thead>
      <tbody id="schedBody"></tbody>
    </table>

    <!-- Formulario añadir horario -->
    <h3 style="font-size:.88rem;color:#1565c0;margin:16px 0 6px">Nuevo horario</h3>
    <div class="schd-form">
      <div><label>Canal</label>
        <select id="nch"><option value="1">Canal 1</option><option value="2">Canal 2</option><option value="3">Canal 3</option></select>
      </div>
      <div><label>Hora</label><input type="time" id="nhm" value="08:00"></div>
      <div><label>Volumen (mL)</label><input type="number" id="nvol" value="100" min="1"></div>
      <div><label>Velocidad (mL/min)</label><input type="number" id="nspd" value="10" min="0.1" step="0.1"></div>
      <div><label>Direcci&oacute;n</label>
        <select id="ndir"><option value="cw">CW &#8635;</option><option value="ccw">CCW &#8634;</option></select>
      </div>
    </div>
    <div style="margin:10px 0">
      <label>D&iacute;as de la semana</label>
      <div class="days" id="dayPicker">
        <button class="db" data-d="0" onclick="toggleDay(0)">L</button>
        <button class="db" data-d="1" onclick="toggleDay(1)">M</button>
        <button class="db" data-d="2" onclick="toggleDay(2)">X</button>
        <button class="db" data-d="3" onclick="toggleDay(3)">J</button>
        <button class="db" data-d="4" onclick="toggleDay(4)">V</button>
        <button class="db" data-d="5" onclick="toggleDay(5)">S</button>
        <button class="db" data-d="6" onclick="toggleDay(6)">D</button>
      </div>
    </div>
    <div class="row" style="max-width:200px">
      <button class="sav" onclick="addSched()">+ A&ntilde;adir horario</button>
    </div>
    <div class="msg" id="schedMsg"></div>
  </div>

  <!-- Calibración -->
  <div class="card">
    <h2>Calibraci&oacute;n</h2>
    <label>Canal</label>
    <select id="cc"><option value="1">Canal 1</option><option value="2">Canal 2</option><option value="3">Canal 3</option></select>
    <label>Pasos / mL</label>
    <input type="number" id="cs" value="1600" min="1">
    <div class="row"><button class="sav" onclick="cal()">Guardar calibraci&oacute;n</button></div>
    <div class="msg" id="cm"></div>
  </div>

  <!-- Config MQTT -->
  <div class="card">
    <h2>Configuraci&oacute;n MQTT</h2>
    <label>Servidor</label><input type="text" id="ms" placeholder="192.168.1.100">
    <div class="two">
      <div><label>Puerto</label><input type="number" id="mp" value="1883"></div>
      <div><label>Usuario</label><input type="text" id="mu" autocomplete="off"></div>
    </div>
    <label>Contrase&ntilde;a</label><input type="password" id="mw" autocomplete="new-password">
    <div class="row">
      <button class="sav" onclick="saveCfg()">Guardar</button>
      <button class="rst" onclick="rstWifi()">Reset WiFi</button>
    </div>
    <div class="msg" id="mm"></div>
  </div>

</main>
<script>
const $ = id => document.getElementById(id);
const DAY_NAMES = ['L','M','X','J','V','S','D'];
let selDays = 0;

async function api(path, body, method) {
  try {
    const m = method || (body ? 'POST' : 'GET');
    const opts = body
      ? { method: m, headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }
      : { method: m };
    return (await fetch(path, opts)).json();
  } catch(e) { return null; }
}

// ---- Motor controls ----
function run(ch) {
  api('/api/run', { channel:ch, volume:+$('v'+ch).value, speed:+$('s'+ch).value, direction:$('d'+ch).value });
}
function stp(ch) { api('/api/stop', { channel:ch }); }

function cal() {
  api('/api/calibrate', { channel:+$('cc').value, stepsperml:+$('cs').value })
    .then(() => { $('cm').textContent='Guardado'; setTimeout(()=>$('cm').textContent='',2500); });
}

function saveCfg() {
  api('/api/config', { server:$('ms').value, port:+$('mp').value, user:$('mu').value, pass:$('mw').value })
    .then(d => { $('mm').textContent = d ? 'Guardado — reconectando...' : 'Error'; setTimeout(()=>$('mm').textContent='',3000); });
}
function rstWifi() {
  if (confirm('Se borrarán las credenciales WiFi y el dispositivo reiniciará.\n¿Continuar?'))
    api('/api/resetwifi', {});
}

// ---- Schedules ----
function toggleDay(d) {
  selDays ^= (1 << d);
  document.querySelectorAll('.db').forEach(b => b.classList.toggle('sel', !!(selDays & (1 << +b.dataset.d))));
}

function daysStr(mask) {
  return DAY_NAMES.filter((_,i) => mask & (1<<i)).join(' ') || '—';
}

function loadSchedules() {
  api('/api/schedules').then(data => {
    if (!data) return;
    const tb = $('schedBody');
    tb.innerHTML = '';
    data.forEach(s => {
      if (!s || s.days === 0) return;
      const tr = document.createElement('tr');
      tr.innerHTML =
        '<td>Ch.' + s.channel + '</td>' +
        '<td style="white-space:nowrap">' + daysStr(s.days) + '</td>' +
        '<td>' + String(s.hour).padStart(2,'0') + ':' + String(s.minute).padStart(2,'0') + '</td>' +
        '<td>' + s.volume + ' mL</td>' +
        '<td>' + s.speed + '</td>' +
        '<td>' + s.direction + '</td>' +
        '<td><input type="checkbox"' + (s.enabled ? ' checked' : '') + ' onchange="toggleSched(' + s.id + ',this.checked)"></td>' +
        '<td><button class="stp" style="padding:3px 10px;font-size:.78rem;flex:none" onclick="delSched(' + s.id + ')">&#10005;</button></td>';
      tb.appendChild(tr);
    });
  });
}

function addSched() {
  if (!selDays) { $('schedMsg').textContent = 'Selecciona al menos un día'; return; }
  const parts = $('nhm').value.split(':');
  api('/api/schedules', {
    channel:+$('nch').value, hour:+parts[0], minute:+parts[1],
    volume:+$('nvol').value, speed:+$('nspd').value,
    direction:$('ndir').value, days:selDays, enabled:true
  }).then(d => {
    if (!d || !d.ok) { $('schedMsg').textContent='Error al guardar'; return; }
    loadSchedules();
    $('schedMsg').textContent='Horario añadido';
    setTimeout(()=>$('schedMsg').textContent='',2500);
  });
}

function delSched(id) {
  if (!confirm('¿Eliminar este horario?')) return;
  api('/api/schedules?id='+id, null, 'DELETE').then(loadSchedules);
}

function toggleSched(id, en) {
  api('/api/schedules', { id:id, enabled:en });
}

function saveTz() {
  api('/api/timezone', { tz: $('tzIn').value })
    .then(() => { $('schedMsg').textContent='Zona horaria guardada'; setTimeout(()=>$('schedMsg').textContent='',2500); });
}

// ---- Status poll ----
function poll() {
  api('/api/status').then(d => {
    if (!d) return;
    $('dW').className = 'dot on';
    $('lW').textContent = d.ip || 'WiFi';
    $('dM').className = 'dot ' + (d.mqtt ? 'on' : 'off');
    for (let i=1; i<=3; i++) {
      const p = d.motors[i-1]?.progress ?? 0;
      $('p'+i).style.width = p+'%';
      $('p'+i+'t').textContent = p+'%';
    }
    if (d.time) $('clock').textContent = d.time;
  });
}

// ---- Init ----
api('/api/config').then(d => {
  if (!d) return;
  $('ms').value = d.server || ''; $('mp').value = d.port || 1883; $('mu').value = d.user || '';
});
api('/api/timezone').then(d => { if (d) $('tzIn').value = d.tz || ''; });
loadSchedules();
setInterval(poll, 1000);
poll();
</script>
</body>
</html>
)rawliteral";


// =========================================================
// MOTOR ACTION HELPERS
// =========================================================

void doRun(int ch, float vol, float spd, const char* dir) {
    bool ccw = String(dir) == "ccw";
    digitalWrite(EnableStepper, LOW);
    if (ch == 1) {
        float v = ccw ? -vol : vol;
        SpeedToMove1  = round(spd * StepsPerMili1 / 60.0f);
        StepsToMove1  = (long)(v * StepsPerMili1);
        InitialSteps1 = stepper1.getCurrentPositionInSteps();
        TargetSteps1  = InitialSteps1 + StepsToMove1;
        Stepper1Running = true;
        stepper1.setSpeedInStepsPerSecond(SpeedToMove1);
        stepper1.setTargetPositionRelativeInSteps(StepsToMove1);
    } else if (ch == 2) {
        float v = ccw ? -vol : vol;
        SpeedToMove2  = round(spd * StepsPerMili2 / 60.0f);
        StepsToMove2  = (long)(v * StepsPerMili2);
        InitialSteps2 = stepper2.getCurrentPositionInSteps();
        TargetSteps2  = InitialSteps2 + StepsToMove2;
        Stepper2Running = true;
        stepper2.setSpeedInStepsPerSecond(SpeedToMove2);
        stepper2.setTargetPositionRelativeInSteps(StepsToMove2);
    } else if (ch == 3) {
        float v = ccw ? -vol : vol;
        SpeedToMove3  = round(spd * StepsPerMili3 / 60.0f);
        StepsToMove3  = (long)(v * StepsPerMili3);
        InitialSteps3 = stepper3.getCurrentPositionInSteps();
        TargetSteps3  = InitialSteps3 + StepsToMove3;
        Stepper3Running = true;
        stepper3.setSpeedInStepsPerSecond(SpeedToMove3);
        stepper3.setTargetPositionRelativeInSteps(StepsToMove3);
    }
}

void doStop(int ch) {
    if (ch == 0) {
        StepperStopped  = 0;
        Stepper1Running = Stepper2Running = Stepper3Running = false;
        stepper1.emergencyStop(); stepper2.emergencyStop(); stepper3.emergencyStop();
        digitalWrite(EnableStepper, HIGH);
    } else if (ch == 1) { StepperStopped=1; Stepper1Running=false; stepper1.emergencyStop(); }
    else if (ch == 2)   { StepperStopped=2; Stepper2Running=false; stepper2.emergencyStop(); }
    else if (ch == 3)   { StepperStopped=3; Stepper3Running=false; stepper3.emergencyStop(); }
}

void doCalibrate(int ch, long spm) {
    EEPROM.begin(EEPROM_SIZE);
    if      (ch == 1) { StepsPerMili1 = spm; EEPROM.put(EepromStepsPerMili1, spm); }
    else if (ch == 2) { StepsPerMili2 = spm; EEPROM.put(EepromStepsPerMili2, spm); }
    else if (ch == 3) { StepsPerMili3 = spm; EEPROM.put(EepromStepsPerMili3, spm); }
    EEPROM.commit(); EEPROM.end();
}


// =========================================================
// STEPPER CALLBACKS
// =========================================================

void targetPositionReachedCallback(byte channel) {
    if (!Stepper1Running && !Stepper2Running && !Stepper3Running)
        digitalWrite(EnableStepper, HIGH);
    if (!mqttEnabled) return;
    char buf[128]; StaticJsonDocument<128> doc;
    doc["type"]="done"; doc["action"]="run"; doc["channel"]=channel; doc["progress"]=100;
    serializeJson(doc, buf);
    mqttClient.publish("peristaltica/status", 1, true, buf);
}

void targetPositionReachedCallbackStepper1(long) { Progress1=100; Stepper1Running=false; targetPositionReachedCallback(1); }
void targetPositionReachedCallbackStepper2(long) { Progress2=100; Stepper2Running=false; targetPositionReachedCallback(2); }
void targetPositionReachedCallbackStepper3(long) { Progress3=100; Stepper3Running=false; targetPositionReachedCallback(3); }

void emergencyStopTriggerdCallbackFunction() {
    if (!Stepper1Running && !Stepper2Running && !Stepper3Running)
        digitalWrite(EnableStepper, HIGH);
    int ps = 0;
    if      (StepperStopped==1 && TargetSteps1!=InitialSteps1) ps=Progress1=round(100*(stepper1.getCurrentPositionInSteps()-InitialSteps1)/(float)(TargetSteps1-InitialSteps1));
    else if (StepperStopped==2 && TargetSteps2!=InitialSteps2) ps=Progress2=round(100*(stepper2.getCurrentPositionInSteps()-InitialSteps2)/(float)(TargetSteps2-InitialSteps2));
    else if (StepperStopped==3 && TargetSteps3!=InitialSteps3) ps=Progress3=round(100*(stepper3.getCurrentPositionInSteps()-InitialSteps3)/(float)(TargetSteps3-InitialSteps3));
    if (!mqttEnabled) return;
    char buf[128]; StaticJsonDocument<128> doc;
    doc["type"]="done"; doc["action"]="stop"; doc["channel"]=StepperStopped; doc["progress"]=ps;
    serializeJson(doc, buf);
    mqttClient.publish("peristaltica/status", 1, true, buf);
}


// =========================================================
// PROGRESS (loop)
// =========================================================

void checkProgress() {
    long now = millis();
    if (now - ProgressPreviousMillis > ProgressInterval) {
        ProgressPreviousMillis = now;
        auto upd = [](bool run, ESP_FlexyStepper& st, long ini, long tgt, int& pct, int ch) {
            if (!run) return;
            if (tgt != ini) pct = round(100*(st.getCurrentPositionInSteps()-ini)/(float)(tgt-ini));
            if (!mqttEnabled) return;
            char buf[128]; StaticJsonDocument<128> doc;
            doc["type"]="running"; doc["action"]="run"; doc["channel"]=ch; doc["progress"]=pct;
            serializeJson(doc, buf);
            mqttClient.publish("peristaltica/status", 1, true, buf);
        };
        upd(Stepper1Running, stepper1, InitialSteps1, TargetSteps1, Progress1, 1);
        upd(Stepper2Running, stepper2, InitialSteps2, TargetSteps2, Progress2, 2);
        upd(Stepper3Running, stepper3, InitialSteps3, TargetSteps3, Progress3, 3);
    }
    if (!Stepper1Running && !Stepper2Running && !Stepper3Running)
        digitalWrite(EnableStepper, HIGH);
}


// =========================================================
// WEB SERVER
// =========================================================

// Helper: current local time as HH:MM:SS string, empty if NTP not ready
String currentTimeStr() {
    struct tm t;
    if (!getLocalTime(&t, 0)) return "";
    char buf[10]; strftime(buf, sizeof(buf), "%H:%M:%S", &t);
    return String(buf);
}

void setupWebServer() {

    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send_P(200, "text/html", index_html);
    });

    webServer.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req) {
        StaticJsonDocument<256> doc;
        doc["ip"]   = WiFi.localIP().toString();
        doc["mqtt"] = mqttClient.connected();
        doc["time"] = currentTimeStr();
        JsonArray m = doc.createNestedArray("motors");
        JsonObject m1=m.createNestedObject(); m1["running"]=Stepper1Running; m1["progress"]=Progress1;
        JsonObject m2=m.createNestedObject(); m2["running"]=Stepper2Running; m2["progress"]=Progress2;
        JsonObject m3=m.createNestedObject(); m3["running"]=Stepper3Running; m3["progress"]=Progress3;
        char buf[256]; serializeJson(doc, buf);
        req->send(200, "application/json", buf);
    });

    webServer.on("/api/params", HTTP_GET, [](AsyncWebServerRequest* req) {
        StaticJsonDocument<128> doc;
        doc["spm1"]=StepsPerMili1; doc["spm2"]=StepsPerMili2; doc["spm3"]=StepsPerMili3;
        char buf[128]; serializeJson(doc, buf);
        req->send(200, "application/json", buf);
    });

    webServer.on("/api/config", HTTP_GET, [](AsyncWebServerRequest* req) {
        StaticJsonDocument<192> doc;
        doc["server"]=mqttServer; doc["port"]=mqttPort; doc["user"]=mqttUser;
        char buf[192]; serializeJson(doc, buf);
        req->send(200, "application/json", buf);
    });

    // GET /api/schedules — returns array of all schedules with their id
    webServer.on("/api/schedules", HTTP_GET, [](AsyncWebServerRequest* req) {
        StaticJsonDocument<2048> doc;
        JsonArray arr = doc.to<JsonArray>();
        for (int i = 0; i < MAX_SCHEDULES; i++) {
            JsonObject o = arr.createNestedObject();
            o["id"]        = i;
            o["enabled"]   = schedules[i].enabled;
            o["channel"]   = schedules[i].channel;
            o["days"]      = schedules[i].days;
            o["hour"]      = schedules[i].hour;
            o["minute"]    = schedules[i].minute;
            o["volume"]    = schedules[i].volume;
            o["speed"]     = schedules[i].speed;
            o["direction"] = schedules[i].direction;
        }
        char buf[2048]; serializeJson(doc, buf);
        req->send(200, "application/json", buf);
    });

    // DELETE /api/schedules?id=N
    webServer.on("/api/schedules", HTTP_DELETE, [](AsyncWebServerRequest* req) {
        if (!req->hasParam("id")) { req->send(400); return; }
        int id = req->getParam("id")->value().toInt();
        if (id < 0 || id >= MAX_SCHEDULES) { req->send(400); return; }
        deleteSchedule(id);
        req->send(200, "application/json", "{\"ok\":true}");
    });

    webServer.on("/api/timezone", HTTP_GET, [](AsyncWebServerRequest* req) {
        prefs.begin("peristaltica", true);
        String tz = prefs.getString("timezone", "CET-1CEST,M3.5.0,M10.5.0/3");
        prefs.end();
        StaticJsonDocument<128> doc; doc["tz"] = tz;
        char buf[128]; serializeJson(doc, buf);
        req->send(200, "application/json", buf);
    });

    // Shared POST body handler
    auto bodyHandler = [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
        StaticJsonDocument<256> doc;
        if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"ok\":false}"); return; }
        String path = req->url();

        if (path == "/api/run") {
            doRun(doc["channel"]|0, doc["volume"]|0.0f, doc["speed"]|0.0f, doc["direction"]|"cw");
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/stop") {
            doStop(doc["channel"]|0);
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/calibrate") {
            doCalibrate(doc["channel"]|0, doc["stepsperml"]|1600L);
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/config") {
            saveMqttConfig(doc["server"]|"", doc["port"]|MQTT_PORT_DEFAULT,
                           doc["user"]|"",   doc["pass"]|"");
            if (mqttEnabled) {
                mqttClient.disconnect(); delay(200);
                mqttClient.setServer(mqttServer.c_str(), mqttPort);
                if (mqttUser.length()) mqttClient.setCredentials(mqttUser.c_str(), mqttPass.c_str());
                mqttClient.connect();
            }
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/schedules") {
            int id = doc["id"] | -1;
            if (id >= 0 && id < MAX_SCHEDULES) {
                // Partial update (e.g. toggle enabled)
                if (doc.containsKey("enabled"))   schedules[id].enabled  = doc["enabled"];
                if (doc.containsKey("days"))       schedules[id].days     = doc["days"];
                if (doc.containsKey("hour"))       schedules[id].hour     = doc["hour"];
                if (doc.containsKey("minute"))     schedules[id].minute   = doc["minute"];
                if (doc.containsKey("volume"))     schedules[id].volume   = doc["volume"];
                if (doc.containsKey("speed"))      schedules[id].speed    = doc["speed"];
                if (doc.containsKey("direction"))  strlcpy(schedules[id].direction, doc["direction"]|"cw", 4);
                if (doc.containsKey("channel"))    schedules[id].channel  = doc["channel"];
                saveSchedule(id);
            } else {
                // Find first free slot (days==0) and insert
                bool inserted = false;
                for (int i = 0; i < MAX_SCHEDULES && !inserted; i++) {
                    if (schedules[i].days != 0) continue;
                    schedules[i].enabled = doc["enabled"]|true;
                    schedules[i].channel = doc["channel"]|1;
                    schedules[i].days    = doc["days"]|0;
                    schedules[i].hour    = doc["hour"]|0;
                    schedules[i].minute  = doc["minute"]|0;
                    schedules[i].volume  = doc["volume"]|0.0f;
                    schedules[i].speed   = doc["speed"]|1.0f;
                    strlcpy(schedules[i].direction, doc["direction"]|"cw", 4);
                    saveSchedule(i);
                    inserted = true;
                }
                if (!inserted) { req->send(507, "application/json", "{\"ok\":false,\"error\":\"full\"}"); return; }
            }
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/timezone") {
            String tz = doc["tz"] | "CET-1CEST,M3.5.0,M10.5.0/3";
            prefs.begin("peristaltica", false);
            prefs.putString("timezone", tz);
            prefs.end();
            setenv("TZ", tz.c_str(), 1);
            tzset();
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/resetwifi") {
            req->send(200, "application/json", "{\"ok\":true}");
            delay(500); WiFiManager wm; wm.resetSettings(); ESP.restart();
        }
        else { req->send(404); }
    };

    webServer.on("/api/run",       HTTP_POST, [](AsyncWebServerRequest*){}, NULL, bodyHandler);
    webServer.on("/api/stop",      HTTP_POST, [](AsyncWebServerRequest*){}, NULL, bodyHandler);
    webServer.on("/api/calibrate", HTTP_POST, [](AsyncWebServerRequest*){}, NULL, bodyHandler);
    webServer.on("/api/config",    HTTP_POST, [](AsyncWebServerRequest*){}, NULL, bodyHandler);
    webServer.on("/api/schedules", HTTP_POST, [](AsyncWebServerRequest*){}, NULL, bodyHandler);
    webServer.on("/api/timezone",  HTTP_POST, [](AsyncWebServerRequest*){}, NULL, bodyHandler);
    webServer.on("/api/resetwifi", HTTP_POST, [](AsyncWebServerRequest*){}, NULL, bodyHandler);

    webServer.begin();
    Serial.println("Web server started");
}


// =========================================================
// MQTT
// =========================================================

void connectToMqtt() {
    if (!mqttEnabled) return;
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
}

void startMqtt() {
    if (!mqttEnabled) return;
    mqttClient.setServer(mqttServer.c_str(), mqttPort);
    if (mqttUser.length()) mqttClient.setCredentials(mqttUser.c_str(), mqttPass.c_str());
    mqttClient.connect();
}

void onMqttConnect(bool) {
    Serial.println("MQTT connected");
    mqttClient.subscribe("peristaltica/action", 1);
    mqttClient.publish("peristaltica/status", 1, true, "Connected");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason) {
    Serial.println("MQTT disconnected");
    if (WiFi.isConnected()) xTimerStart(mqttReconnectTimer, 0);
}

void onMqttMessage(char*, char* payload, AsyncMqttClientMessageProperties, size_t len, size_t, size_t) {
    StaticJsonDocument<256> doc;
    if (deserializeJson(doc, payload, len)) return;
    const char* action = doc["action"]|"";
    int ch = doc["channel"]|0;
    if      (String(action)=="run")       doRun(ch, doc["volume"]|0.0f, doc["speed"]|0.0f, doc["direction"]|"cw");
    else if (String(action)=="stop")      doStop(ch);
    else if (String(action)=="calibrate") doCalibrate(ch, doc["stepsperml"]|1600L);
    else if (String(action)=="params") {
        char buf[128]; StaticJsonDocument<128> r;
        r["action"]="params"; r["type"]="done";
        r["spm1"]=StepsPerMili1; r["spm2"]=StepsPerMili2; r["spm3"]=StepsPerMili3;
        serializeJson(r, buf); mqttClient.publish("peristaltica/status", 1, true, buf);
    }
}


// =========================================================
// WiFi EVENT
// =========================================================

void reconnectWifi() { WiFi.reconnect(); }

void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.print("WiFi OK — "); Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost");
        xTimerStop(mqttReconnectTimer, 0);
        xTimerStart(wifiReconnectTimer, 0);
        break;
    default: break;
    }
}


// =========================================================
// OTA (Core 0)
// =========================================================

void core0assignments(void*) {
    for (;;) { ArduinoOTA.handle(); vTaskDelay(1); }
}


// =========================================================
// STEPPER SETUP
// =========================================================

void StepperSetup() {
    pinMode(Dir1,OUTPUT);  pinMode(Step1,OUTPUT);
    pinMode(Dir2,OUTPUT);  pinMode(Step2,OUTPUT);
    pinMode(Dir3,OUTPUT);  pinMode(Step3,OUTPUT);
    pinMode(EnableStepper,OUTPUT); digitalWrite(EnableStepper,HIGH);

    stepper1.connectToPins(Step1,Dir1);
    stepper1.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
    stepper1.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
    stepper1.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
    stepper1.registerTargetPositionReachedCallback(targetPositionReachedCallbackStepper1);
    stepper1.registerEmergencyStopTriggeredCallback(emergencyStopTriggerdCallbackFunction);

    stepper2.connectToPins(Step2,Dir2);
    stepper2.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
    stepper2.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
    stepper2.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
    stepper2.registerTargetPositionReachedCallback(targetPositionReachedCallbackStepper2);
    stepper2.registerEmergencyStopTriggeredCallback(emergencyStopTriggerdCallbackFunction);

    stepper3.connectToPins(Step3,Dir3);
    stepper3.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
    stepper3.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
    stepper3.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
    stepper3.registerTargetPositionReachedCallback(targetPositionReachedCallbackStepper3);
    stepper3.registerEmergencyStopTriggeredCallback(emergencyStopTriggerdCallbackFunction);

    stepper1.startAsService(1);
    stepper2.startAsService(1);
    stepper3.startAsService(1);
}


// =========================================================
// EEPROM READ
// =========================================================

void EepromRead() {
    EEPROM.begin(EEPROM_SIZE);
    EEPROM.get(EepromStepsPerMili1, StepsPerMili1);
    EEPROM.get(EepromStepsPerMili2, StepsPerMili2);
    EEPROM.get(EepromStepsPerMili3, StepsPerMili3);
    EEPROM.end();
    if (StepsPerMili1 <= 0) StepsPerMili1 = 1600;
    if (StepsPerMili2 <= 0) StepsPerMili2 = 1600;
    if (StepsPerMili3 <= 0) StepsPerMili3 = 1600;
}


// =========================================================
// SETUP
// =========================================================

void setup() {
    Serial.begin(115200);

    disableCore0WDT();
    xTaskCreatePinnedToCore(core0assignments, "Core_0", 10000, NULL, 1, &C0, 0);

    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                      reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0,
                                      reinterpret_cast<TimerCallbackFunction_t>(reconnectWifi));

    WiFi.onEvent(WiFiEvent);

    WiFiManager wm;
    wm.setConfigPortalTimeout(180);
    wm.setConnectTimeout(30);
    wm.setHostname("Peristaltica");
    if (!wm.autoConnect("Peristaltica-Setup")) {
        Serial.println("WiFiManager timeout — restarting");
        ESP.restart();
    }

    // MQTT
    loadMqttConfig();
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);
    startMqtt();

    // NTP + schedules
    setupNTP();
    loadSchedules();

    // OTA
    ArduinoOTA.setHostname("Peristaltica");
    ArduinoOTA.begin();

    // Web server
    setupWebServer();

    // Steppers + calibration
    StepperSetup();
    EepromRead();

    Serial.print("Ready — http://");
    Serial.println(WiFi.localIP());
}


// =========================================================
// LOOP
// =========================================================

void loop() {
    checkProgress();
    checkSchedules();
    delay(1);
}
