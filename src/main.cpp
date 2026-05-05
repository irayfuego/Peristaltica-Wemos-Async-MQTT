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
const int EepromStepsPerMili1 = 0;   // addr 0, 4 bytes
const int EepromStepsPerMili2 = 4;   // addr 4, 4 bytes
const int EepromStepsPerMili3 = 8;   // addr 8, 4 bytes

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
.err{color:#c62828}
</style>
</head>
<body>
<header>
  <h1>Bomba Perist&aacute;ltica</h1>
  <div class="sb">
    <span><span class="dot off" id="dW"></span><span id="lW">WiFi</span></span>
    <span><span class="dot off" id="dM"></span>MQTT</span>
  </div>
</header>
<main>

  <!-- Canal 1 -->
  <div class="card">
    <h2>Canal 1</h2>
    <label>Volumen (mL)</label>
    <input type="number" id="v1" value="10" min="0.1" step="0.1">
    <label>Velocidad (mL/min)</label>
    <input type="number" id="s1" value="5" min="0.1" step="0.1">
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

  <!-- Canal 2 -->
  <div class="card">
    <h2>Canal 2</h2>
    <label>Volumen (mL)</label>
    <input type="number" id="v2" value="10" min="0.1" step="0.1">
    <label>Velocidad (mL/min)</label>
    <input type="number" id="s2" value="5" min="0.1" step="0.1">
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

  <!-- Canal 3 -->
  <div class="card">
    <h2>Canal 3</h2>
    <label>Volumen (mL)</label>
    <input type="number" id="v3" value="10" min="0.1" step="0.1">
    <label>Velocidad (mL/min)</label>
    <input type="number" id="s3" value="5" min="0.1" step="0.1">
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
    <h2 style="border:none;padding:0;margin:0">Control global</h2>
    <button class="stp" style="max-width:200px;flex:unset;padding:10px 24px" onclick="stp(0)">&#9632; Parar todo</button>
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
    <label>Servidor (IP o hostname)</label>
    <input type="text" id="ms" placeholder="192.168.1.100">
    <div class="two">
      <div><label>Puerto</label><input type="number" id="mp" value="1883"></div>
      <div><label>Usuario</label><input type="text" id="mu" autocomplete="off"></div>
    </div>
    <label>Contrase&ntilde;a</label>
    <input type="password" id="mw" autocomplete="new-password">
    <div class="row">
      <button class="sav" onclick="saveCfg()">Guardar</button>
      <button class="rst" onclick="rstWifi()">Reset WiFi</button>
    </div>
    <div class="msg" id="mm"></div>
  </div>

</main>
<script>
const $ = id => document.getElementById(id);

async function api(path, body) {
  try {
    const opts = body
      ? { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }
      : {};
    const r = await fetch(path, opts);
    return r.json();
  } catch (e) { return null; }
}

function run(ch) {
  api('/api/run', {
    channel: ch,
    volume:    +$('v' + ch).value,
    speed:     +$('s' + ch).value,
    direction:  $('d' + ch).value
  });
}
function stp(ch) { api('/api/stop', { channel: ch }); }

function cal() {
  api('/api/calibrate', { channel: +$('cc').value, stepsperml: +$('cs').value })
    .then(() => { $('cm').textContent = 'Guardado'; setTimeout(() => $('cm').textContent = '', 2500); });
}

function saveCfg() {
  api('/api/config', {
    server: $('ms').value, port: +$('mp').value,
    user:   $('mu').value, pass: $('mw').value
  }).then(d => {
    $('mm').textContent = d ? 'Guardado — reconectando MQTT...' : 'Error';
    setTimeout(() => $('mm').textContent = '', 3000);
  });
}

function rstWifi() {
  if (confirm('Se borrarán las credenciales WiFi y el dispositivo reiniciará.\n¿Continuar?'))
    api('/api/resetwifi', {});
}

function setBar(ch, pct) {
  $('p' + ch).style.width = pct + '%';
  $('p' + ch + 't').textContent = pct + '%';
}

function poll() {
  api('/api/status').then(d => {
    if (!d) return;
    $('dW').className = 'dot on';
    $('lW').textContent = d.ip || 'WiFi';
    $('dM').className = 'dot ' + (d.mqtt ? 'on' : 'off');
    for (let i = 1; i <= 3; i++) setBar(i, d.motors[i - 1]?.progress ?? 0);
  });
}

// Load MQTT config on startup
api('/api/config').then(d => {
  if (!d) return;
  $('ms').value = d.server || '';
  $('mp').value = d.port   || 1883;
  $('mu').value = d.user   || '';
});

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
        stepper1.emergencyStop();
        stepper2.emergencyStop();
        stepper3.emergencyStop();
        digitalWrite(EnableStepper, HIGH);
    } else if (ch == 1) {
        StepperStopped  = 1;
        Stepper1Running = false;
        stepper1.emergencyStop();
    } else if (ch == 2) {
        StepperStopped  = 2;
        Stepper2Running = false;
        stepper2.emergencyStop();
    } else if (ch == 3) {
        StepperStopped  = 3;
        Stepper3Running = false;
        stepper3.emergencyStop();
    }
}

void doCalibrate(int ch, long spm) {
    EEPROM.begin(EEPROM_SIZE);
    if (ch == 1) { StepsPerMili1 = spm; EEPROM.put(EepromStepsPerMili1, spm); }
    else if (ch == 2) { StepsPerMili2 = spm; EEPROM.put(EepromStepsPerMili2, spm); }
    else if (ch == 3) { StepsPerMili3 = spm; EEPROM.put(EepromStepsPerMili3, spm); }
    EEPROM.commit();
    EEPROM.end();
}


// =========================================================
// STEPPER CALLBACKS
// =========================================================

void targetPositionReachedCallback(byte channel) {
    if (!Stepper1Running && !Stepper2Running && !Stepper3Running)
        digitalWrite(EnableStepper, HIGH);

    if (!mqttEnabled) return;
    char buf[128];
    StaticJsonDocument<128> doc;
    doc["type"] = "done"; doc["action"] = "run";
    doc["channel"] = channel; doc["progress"] = 100;
    serializeJson(doc, buf);
    mqttClient.publish("peristaltica/status", 1, true, buf);
}

void targetPositionReachedCallbackStepper1(long) {
    Progress1 = 100;
    Stepper1Running = false;
    targetPositionReachedCallback(1);
}
void targetPositionReachedCallbackStepper2(long) {
    Progress2 = 100;
    Stepper2Running = false;
    targetPositionReachedCallback(2);
}
void targetPositionReachedCallbackStepper3(long) {
    Progress3 = 100;
    Stepper3Running = false;
    targetPositionReachedCallback(3);
}

void emergencyStopTriggerdCallbackFunction() {
    if (!Stepper1Running && !Stepper2Running && !Stepper3Running)
        digitalWrite(EnableStepper, HIGH);

    int progressStop = 0;
    if (StepperStopped == 1 && TargetSteps1 != InitialSteps1)
        progressStop = Progress1 = round(100 * (stepper1.getCurrentPositionInSteps() - InitialSteps1) / (float)(TargetSteps1 - InitialSteps1));
    else if (StepperStopped == 2 && TargetSteps2 != InitialSteps2)
        progressStop = Progress2 = round(100 * (stepper2.getCurrentPositionInSteps() - InitialSteps2) / (float)(TargetSteps2 - InitialSteps2));
    else if (StepperStopped == 3 && TargetSteps3 != InitialSteps3)
        progressStop = Progress3 = round(100 * (stepper3.getCurrentPositionInSteps() - InitialSteps3) / (float)(TargetSteps3 - InitialSteps3));

    if (!mqttEnabled) return;
    char buf[128];
    StaticJsonDocument<128> doc;
    doc["type"] = "done"; doc["action"] = "stop";
    doc["channel"] = StepperStopped; doc["progress"] = progressStop;
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

        auto update = [](bool running, ESP_FlexyStepper& st, long ini, long tgt, int& pct,
                         int ch) {
            if (!running) return;
            if (tgt != ini)
                pct = round(100 * (st.getCurrentPositionInSteps() - ini) / (float)(tgt - ini));
            if (!mqttEnabled) return;
            char buf[128];
            StaticJsonDocument<128> doc;
            doc["type"] = "running"; doc["action"] = "run";
            doc["channel"] = ch; doc["progress"] = pct;
            serializeJson(doc, buf);
            mqttClient.publish("peristaltica/status", 1, true, buf);
        };

        update(Stepper1Running, stepper1, InitialSteps1, TargetSteps1, Progress1, 1);
        update(Stepper2Running, stepper2, InitialSteps2, TargetSteps2, Progress2, 2);
        update(Stepper3Running, stepper3, InitialSteps3, TargetSteps3, Progress3, 3);
    }

    if (!Stepper1Running && !Stepper2Running && !Stepper3Running)
        digitalWrite(EnableStepper, HIGH);
}


// =========================================================
// WEB SERVER
// =========================================================

void setupWebServer() {
    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send_P(200, "text/html", index_html);
    });

    webServer.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req) {
        StaticJsonDocument<256> doc;
        doc["ip"]   = WiFi.localIP().toString();
        doc["mqtt"] = mqttClient.connected();
        JsonArray motors = doc.createNestedArray("motors");
        JsonObject m1 = motors.createNestedObject(); m1["running"] = Stepper1Running; m1["progress"] = Progress1;
        JsonObject m2 = motors.createNestedObject(); m2["running"] = Stepper2Running; m2["progress"] = Progress2;
        JsonObject m3 = motors.createNestedObject(); m3["running"] = Stepper3Running; m3["progress"] = Progress3;
        char buf[256]; serializeJson(doc, buf);
        req->send(200, "application/json", buf);
    });

    webServer.on("/api/params", HTTP_GET, [](AsyncWebServerRequest* req) {
        StaticJsonDocument<128> doc;
        doc["spm1"] = StepsPerMili1;
        doc["spm2"] = StepsPerMili2;
        doc["spm3"] = StepsPerMili3;
        char buf[128]; serializeJson(doc, buf);
        req->send(200, "application/json", buf);
    });

    webServer.on("/api/config", HTTP_GET, [](AsyncWebServerRequest* req) {
        StaticJsonDocument<256> doc;
        doc["server"] = mqttServer;
        doc["port"]   = mqttPort;
        doc["user"]   = mqttUser;
        // password not sent to client
        char buf[256]; serializeJson(doc, buf);
        req->send(200, "application/json", buf);
    });

    // Shared body handler — dispatches by URL
    auto bodyHandler = [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t, size_t) {
        StaticJsonDocument<256> doc;
        if (deserializeJson(doc, data, len)) { req->send(400, "application/json", "{\"ok\":false}"); return; }

        String path = req->url();

        if (path == "/api/run") {
            doRun(doc["channel"] | 0, doc["volume"] | 0.0f, doc["speed"] | 0.0f,
                  doc["direction"] | "cw");
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/stop") {
            doStop(doc["channel"] | 0);
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/calibrate") {
            doCalibrate(doc["channel"] | 0, doc["stepsperml"] | 1600L);
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/config") {
            saveMqttConfig(doc["server"] | "", doc["port"] | MQTT_PORT_DEFAULT,
                           doc["user"] | "", doc["pass"] | "");
            if (mqttEnabled) {
                mqttClient.disconnect();
                delay(200);
                mqttClient.setServer(mqttServer.c_str(), mqttPort);
                if (mqttUser.length())
                    mqttClient.setCredentials(mqttUser.c_str(), mqttPass.c_str());
                mqttClient.connect();
            }
            req->send(200, "application/json", "{\"ok\":true}");
        }
        else if (path == "/api/resetwifi") {
            req->send(200, "application/json", "{\"ok\":true}");
            delay(500);
            WiFiManager wm;
            wm.resetSettings();
            ESP.restart();
        }
        else {
            req->send(404);
        }
    };

    webServer.on("/api/run",       HTTP_POST, [](AsyncWebServerRequest*) {}, NULL, bodyHandler);
    webServer.on("/api/stop",      HTTP_POST, [](AsyncWebServerRequest*) {}, NULL, bodyHandler);
    webServer.on("/api/calibrate", HTTP_POST, [](AsyncWebServerRequest*) {}, NULL, bodyHandler);
    webServer.on("/api/config",    HTTP_POST, [](AsyncWebServerRequest*) {}, NULL, bodyHandler);
    webServer.on("/api/resetwifi", HTTP_POST, [](AsyncWebServerRequest*) {}, NULL, bodyHandler);

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
    if (mqttUser.length())
        mqttClient.setCredentials(mqttUser.c_str(), mqttPass.c_str());
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

    const char* action = doc["action"] | "";
    int ch = doc["channel"] | 0;

    if (String(action) == "run") {
        doRun(ch, doc["volume"] | 0.0f, doc["speed"] | 0.0f, doc["direction"] | "cw");
    } else if (String(action) == "stop") {
        doStop(ch);
    } else if (String(action) == "calibrate") {
        doCalibrate(ch, doc["stepsperml"] | 1600L);
    } else if (String(action) == "params") {
        char buf[128];
        StaticJsonDocument<128> resp;
        resp["action"] = "params"; resp["type"] = "done";
        resp["spm1"] = StepsPerMili1; resp["spm2"] = StepsPerMili2; resp["spm3"] = StepsPerMili3;
        serializeJson(resp, buf);
        mqttClient.publish("peristaltica/status", 1, true, buf);
    }
}


// =========================================================
// WiFi EVENT
// =========================================================

void reconnectWifi() { WiFi.reconnect(); }

void WiFiEvent(WiFiEvent_t event) {
    switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.print("WiFi OK, IP: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost");
        xTimerStop(mqttReconnectTimer, 0);
        xTimerStart(wifiReconnectTimer, 0);
        break;
    default:
        break;
    }
}


// =========================================================
// OTA (Core 0)
// =========================================================

void core0assignments(void*) {
    for (;;) {
        ArduinoOTA.handle();
        vTaskDelay(1);
    }
}


// =========================================================
// STEPPER SETUP
// =========================================================

void StepperSetup() {
    pinMode(Dir1,  OUTPUT); pinMode(Step1, OUTPUT);
    pinMode(Dir2,  OUTPUT); pinMode(Step2, OUTPUT);
    pinMode(Dir3,  OUTPUT); pinMode(Step3, OUTPUT);
    pinMode(EnableStepper, OUTPUT);
    digitalWrite(EnableStepper, HIGH);

    stepper1.connectToPins(Step1, Dir1);
    stepper1.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
    stepper1.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
    stepper1.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
    stepper1.registerTargetPositionReachedCallback(targetPositionReachedCallbackStepper1);
    stepper1.registerEmergencyStopTriggeredCallback(emergencyStopTriggerdCallbackFunction);

    stepper2.connectToPins(Step2, Dir2);
    stepper2.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
    stepper2.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
    stepper2.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
    stepper2.registerTargetPositionReachedCallback(targetPositionReachedCallbackStepper2);
    stepper2.registerEmergencyStopTriggeredCallback(emergencyStopTriggerdCallbackFunction);

    stepper3.connectToPins(Step3, Dir3);
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
    // Default for fresh flash (value 0 or negative means uninitialised)
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

    // WiFiManager — blocks until connected or timeout
    WiFiManager wm;
    wm.setConfigPortalTimeout(180);  // 3 min to configure, then restart
    wm.setConnectTimeout(30);
    wm.setHostname("Peristaltica");
    if (!wm.autoConnect("Peristaltica-Setup")) {
        Serial.println("WiFiManager timeout — restarting");
        ESP.restart();
    }

    // Load MQTT config from Preferences and connect
    loadMqttConfig();
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.onMessage(onMqttMessage);
    startMqtt();

    // OTA
    ArduinoOTA.setHostname("Peristaltica");
    ArduinoOTA.begin();

    // Web server
    setupWebServer();

    // Steppers + EEPROM calibration
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
    delay(1);
}
