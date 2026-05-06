#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <ESP_FlexyStepper.h>
#include <NimBLEDevice.h>
#include <time.h>
extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/timers.h"
    #include "freertos/semphr.h"
    #include "esp_task_wdt.h"
}
#include <AsyncMqttClient.h>

// ---- Mutex protecting global Preferences instance + sensorData/scanResults ----
SemaphoreHandle_t prefsMutex   = nullptr;
SemaphoreHandle_t sensorMutex  = nullptr;
#define PREFS_LOCK()    xSemaphoreTake(prefsMutex,  portMAX_DELAY)
#define PREFS_UNLOCK()  xSemaphoreGive(prefsMutex)
#define SENSOR_LOCK()   xSemaphoreTake(sensorMutex, portMAX_DELAY)
#define SENSOR_UNLOCK() xSemaphoreGive(sensorMutex)

// ---- MQTT client ----
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// ---- MQTT config ----
#define MQTT_PORT_DEFAULT 1883
String mqttServer;
int    mqttPort = MQTT_PORT_DEFAULT;
String mqttUser;
String mqttPass;
bool   mqttEnabled = false;

// ---- Preferences ----
Preferences prefs;

void loadMqttConfig() {
    PREFS_LOCK();
    prefs.begin("peristaltica", true);
    mqttServer = prefs.getString("mqtt_srv", "");
    mqttPort   = prefs.getInt("mqtt_port", MQTT_PORT_DEFAULT);
    mqttUser   = prefs.getString("mqtt_usr", "");
    mqttPass   = prefs.getString("mqtt_pwd", "");
    prefs.end();
    PREFS_UNLOCK();
    mqttEnabled = mqttServer.length() > 0;
}

void saveMqttConfig(const String& srv, int port, const String& usr, const String& pwd) {
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.putString("mqtt_srv", srv);
    prefs.putInt("mqtt_port", port);
    prefs.putString("mqtt_usr", usr);
    prefs.putString("mqtt_pwd", pwd);
    prefs.end();
    PREFS_UNLOCK();
    mqttServer  = srv;  mqttPort = port;
    mqttUser    = usr;  mqttPass = pwd;
    mqttEnabled = srv.length() > 0;
}

// ---- Web server ----
AsyncWebServer webServer(80);

// ---- Pins ----
const int Dir1 = GPIO_NUM_16, Dir2 = GPIO_NUM_27, Dir3 = GPIO_NUM_14;
const int Step1 = GPIO_NUM_26, Step2 = GPIO_NUM_25, Step3 = GPIO_NUM_17;
const int EnableStepper = 12;

// ---- Motor state ----
long StepsPerMili1, StepsPerMili2, StepsPerMili3;
long StepsToMove1,  StepsToMove2,  StepsToMove3;
long SpeedToMove1,  SpeedToMove2,  SpeedToMove3;
long InitialSteps1, InitialSteps2, InitialSteps3;
long TargetSteps1,  TargetSteps2,  TargetSteps3;
int  StepperStopped;
volatile bool Stepper1Running = false, Stepper2Running = false, Stepper3Running = false;
int  Progress1 = 0, Progress2 = 0, Progress3 = 0;

// MQTT publish deferred from stepper-task callbacks to loop()
struct PendingMqttEvent {
    bool     pending;
    char     event;     // 'D' = run done, 'S' = stop
    uint8_t  channel;
    int      progress;
};
volatile PendingMqttEvent pendingMqtt[3] = {{false,0,0,0},{false,0,0,0},{false,0,0,0}};

// ---- Calibration storage keys (in Preferences "peristaltica") ----
const char* CAL_KEY[3] = { "spm1", "spm2", "spm3" };

// ---- Speed defaults ----
const int SPEED_IN_STEPS_PER_SECOND        = 2000;
const int ACCELERATION_IN_STEPS_PER_SECOND = 800;
const int DECELERATION_IN_STEPS_PER_SECOND = 800;

ESP_FlexyStepper stepper1, stepper2, stepper3;

// Cached driver-enable line state. EN is active-LOW.
static int enableStepperState = HIGH;
static inline void setEnableStepper(int level) {
    if (level == enableStepperState) return;
    enableStepperState = level;
    digitalWrite(EnableStepper, level);
}

// ---- Progress timing ----
long ProgressPreviousMillis = 0;
const long ProgressInterval = 5000;

TaskHandle_t C0;


// =========================================================
// BLE SENSORS
// =========================================================

#define MAX_SCAN_RESULTS 20

struct SensorConfig {
    char    mac[18];     // "XX:XX:XX:XX:XX:XX\0"
    char    name[40];
    uint8_t threshold;   // moisture % — 0 = disabled
    bool    assigned;
};

struct SensorReading {
    float    temperature;
    uint8_t  moisture;
    uint32_t light;
    uint16_t conductivity;
    bool     valid;
    time_t   timestamp;
};

struct BLEScanResult {
    char addr[18];
    char name[48];
    int  rssi;
};

SensorConfig  sensorCfg[3];     // index = channel-1
SensorReading sensorData[3];

BLEScanResult scanResults[MAX_SCAN_RESULTS];
volatile int  scanResultCount = 0;
volatile bool bleScanning     = false;
volatile bool bleScanReq      = false;
volatile bool bleReadReq      = false;

// ---------- NVS helpers ----------

void saveSensorConfig(int ch) {   // ch = 0..2
    char key[12]; sprintf(key, "sensor_%d", ch);
    StaticJsonDocument<192> doc;
    doc["mac"]  = sensorCfg[ch].mac;
    doc["name"] = sensorCfg[ch].name;
    doc["thr"]  = sensorCfg[ch].threshold;
    doc["asgn"] = sensorCfg[ch].assigned;
    char buf[192]; serializeJson(doc, buf);
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.putString(key, buf);
    prefs.end();
    PREFS_UNLOCK();
}

void loadSensorConfigs() {
    PREFS_LOCK();
    prefs.begin("peristaltica", true);
    for (int i = 0; i < 3; i++) {
        sensorCfg[i]  = {{0}, {0}, 0, false};
        sensorData[i] = {0, 0, 0, 0, false, 0};
        char key[12]; sprintf(key, "sensor_%d", i);
        String s = prefs.getString(key, "");
        if (s.length() == 0) continue;
        StaticJsonDocument<192> doc;
        if (deserializeJson(doc, s)) continue;
        strlcpy(sensorCfg[i].mac,  doc["mac"]  | "", 18);
        strlcpy(sensorCfg[i].name, doc["name"] | "", 40);
        sensorCfg[i].threshold = doc["thr"]  | 0;
        sensorCfg[i].assigned  = doc["asgn"] | false;
    }
    prefs.end();
    PREFS_UNLOCK();
}

void deleteSensorConfig(int ch) {
    sensorCfg[ch] = {{0}, {0}, 0, false};
    char key[12]; sprintf(key, "sensor_%d", ch);
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.remove(key);
    prefs.end();
    PREFS_UNLOCK();
}

// ---------- Mi Flora BLE read ----------

bool readMiFlora(const char* macStr, SensorReading& r) {
    NimBLEAddress addr(macStr);
    NimBLEClient* client = NimBLEDevice::createClient();
    client->setConnectionParams(16, 16, 0, 60);

    if (!client->connect(addr)) {
        NimBLEDevice::deleteClient(client);
        return false;
    }

    // Service 00001204-0000-1000-8000-00805f9b34fb
    NimBLERemoteService* svc =
        client->getService("00001204-0000-1000-8000-00805f9b34fb");
    if (!svc) { client->disconnect(); NimBLEDevice::deleteClient(client); return false; }

    // Write 0xA0 0x1F to 0x1A00 to request real-time data
    NimBLERemoteCharacteristic* wc =
        svc->getCharacteristic("00001a00-0000-1000-8000-00805f9b34fb");
    if (wc && wc->canWrite()) {
        uint8_t cmd[] = {0xA0, 0x1F};
        wc->writeValue(cmd, 2, false);
        delay(200);
    }

    // Read sensor data from 0x1A01
    NimBLERemoteCharacteristic* rc =
        svc->getCharacteristic("00001a01-0000-1000-8000-00805f9b34fb");
    if (!rc || !rc->canRead()) {
        client->disconnect(); NimBLEDevice::deleteClient(client); return false;
    }

    std::string val = rc->readValue();
    if (val.length() < 8) {
        client->disconnect(); NimBLEDevice::deleteClient(client); return false;
    }

    const uint8_t* d = (const uint8_t*)val.data();
    // Byte layout: [0-1]=temp(int16LE/10°C) [2]=0 [3]=moisture% [4-5]=light(uint16LE lux) [6-7]=conductivity(uint16LE µS/cm)
    SENSOR_LOCK();
    r.temperature   = (int16_t)(d[0] | (d[1] << 8)) / 10.0f;
    r.moisture      = d[3];
    r.light         = (uint32_t)(d[4] | (d[5] << 8));
    r.conductivity  = (uint16_t)(d[6] | (d[7] << 8));
    r.valid         = true;
    r.timestamp     = time(nullptr);
    SENSOR_UNLOCK();

    client->disconnect();
    NimBLEDevice::deleteClient(client);
    return true;
}

// ---------- Scan ----------

void doBleScan() {
    scanResultCount = 0;
    bleScanning = true;

    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);

    NimBLEScanResults results = scan->start(10, false); // 10 seconds

    SENSOR_LOCK();
    int count = 0;
    for (int i = 0; i < results.getCount() && count < MAX_SCAN_RESULTS; i++) {
        NimBLEAdvertisedDevice dev = results.getDevice(i);
        std::string name = dev.getName();
        std::string addr = dev.getAddress().toString();
        strlcpy(scanResults[count].addr, addr.c_str(), 18);
        strlcpy(scanResults[count].name, name.length() ? name.c_str() : "?", 48);
        scanResults[count].rssi = dev.getRSSI();
        count++;
    }
    scanResultCount = count;
    SENSOR_UNLOCK();
    scan->clearResults();
    bleScanning = false;
}

// ---------- Read all assigned sensors ----------

void doReadAllSensors() {
    for (int i = 0; i < 3; i++) {
        if (!sensorCfg[i].assigned) continue;
        Serial.printf("Reading sensor ch%d (%s)...\n", i+1, sensorCfg[i].mac);
        if (!readMiFlora(sensorCfg[i].mac, sensorData[i])) {
            Serial.printf("  Failed\n");
        } else {
            Serial.printf("  moisture=%d%% temp=%.1f°C\n",
                          sensorData[i].moisture, sensorData[i].temperature);
        }
    }
}

// Returns true if at least one channel has an assigned sensor
bool anySensorAssigned() {
    for (int i = 0; i < 3; i++) if (sensorCfg[i].assigned) return true;
    return false;
}

// "XX:XX:XX:XX:XX:XX" — 17 chars, hex pairs separated by ':'
bool isValidMac(const char* m) {
    if (!m || strlen(m) != 17) return false;
    for (int i = 0; i < 17; i++) {
        char c = m[i];
        if ((i % 3) == 2) { if (c != ':') return false; }
        else { if (!isxdigit((int)c)) return false; }
    }
    return true;
}


// =========================================================
// SCHEDULE
// =========================================================

#define MAX_SCHEDULES 16

struct Schedule {
    bool    enabled;
    uint8_t channel;
    uint8_t days;       // bitmask bit0=Mon … bit6=Sun
    uint8_t hour;
    uint8_t minute;
    float   volume;
    float   speed;
    char    direction[4];
    uint8_t moistureThreshold; // 0 = ignore sensor; 1-100 = skip if moisture >= this
};

Schedule schedules[MAX_SCHEDULES];

static void resetSchedule(Schedule& s) {
    s.enabled = false; s.channel = 1; s.days = 0;
    s.hour = 0; s.minute = 0; s.volume = 0; s.speed = 10;
    strlcpy(s.direction, "cw", sizeof(s.direction));
    s.moistureThreshold = 0;
}

void saveSchedule(int idx) {
    char key[12]; sprintf(key, "sched_%d", idx);
    StaticJsonDocument<256> doc;
    doc["en"]  = schedules[idx].enabled;
    doc["ch"]  = schedules[idx].channel;
    doc["dy"]  = schedules[idx].days;
    doc["hr"]  = schedules[idx].hour;
    doc["mn"]  = schedules[idx].minute;
    doc["vol"] = schedules[idx].volume;
    doc["spd"] = schedules[idx].speed;
    doc["dir"] = schedules[idx].direction;
    doc["mth"] = schedules[idx].moistureThreshold;
    char buf[256]; serializeJson(doc, buf);
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.putString(key, buf);
    prefs.end();
    PREFS_UNLOCK();
}

void loadSchedules() {
    PREFS_LOCK();
    prefs.begin("peristaltica", true);
    for (int i = 0; i < MAX_SCHEDULES; i++) {
        // Reset slot first so corrupt JSON also yields a clean default
        resetSchedule(schedules[i]);
        char key[12]; sprintf(key, "sched_%d", i);
        String s = prefs.getString(key, "");
        if (s.length() == 0) continue;
        StaticJsonDocument<256> doc;
        if (deserializeJson(doc, s)) continue;
        schedules[i].enabled           = doc["en"]  | false;
        schedules[i].channel           = doc["ch"]  | 1;
        schedules[i].days              = doc["dy"]  | 0;
        schedules[i].hour              = doc["hr"]  | 0;
        schedules[i].minute            = doc["mn"]  | 0;
        schedules[i].volume            = doc["vol"] | 0.0f;
        schedules[i].speed             = doc["spd"] | 10.0f;
        schedules[i].moistureThreshold = doc["mth"] | 0;
        strlcpy(schedules[i].direction, doc["dir"] | "cw", 4);
    }
    prefs.end();
    PREFS_UNLOCK();
}

void deleteSchedule(int idx) {
    resetSchedule(schedules[idx]);
    char key[12]; sprintf(key, "sched_%d", idx);
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.remove(key);
    prefs.end();
    PREFS_UNLOCK();
}


// =========================================================
// NTP
// =========================================================

void applyTimezone() {
    PREFS_LOCK();
    prefs.begin("peristaltica", true);
    String tz = prefs.getString("timezone", "CET-1CEST,M3.5.0,M10.5.0/3");
    prefs.end();
    PREFS_UNLOCK();
    setenv("TZ", tz.c_str(), 1);
    tzset();
}

// Returns true when the system clock has been set from a valid source
// (year >= 2024). Avoids the magic 100000-seconds heuristic.
static bool ntpSynced() {
    struct tm t;
    return getLocalTime(&t, 0) && (t.tm_year + 1900) >= 2024;
}

void setupNTP() {
    applyTimezone();
    configTime(0, 0, "pool.ntp.org", "time.cloudflare.com");
}

// Optional: block briefly while waiting for first sync. Used by setup() if
// you prefer to log the result to serial; safe to skip — checkSchedules()
// already guards against pre-sync ticks.
static void waitForNTP(uint32_t timeoutMs = 10000) {
    Serial.print("NTP sync");
    uint32_t start = millis();
    while (!ntpSynced() && millis() - start < timeoutMs) {
        delay(500); Serial.print(".");
    }
    Serial.println(ntpSynced() ? " OK" : " timeout");
}

void checkSchedules() {
    struct tm t;
    if (!getLocalTime(&t, 0)) return;

    static int  lastCheckedMinute = -1;
    static bool firstTickConsumed = false;

    if (t.tm_min == lastCheckedMinute) return;
    lastCheckedMinute = t.tm_min;

    // First valid tick after boot: just record the minute, don't fire schedules.
    // Prevents accidentally re-running a schedule whose minute happens to match
    // when NTP first syncs.
    if (!firstTickConsumed) { firstTickConsumed = true; return; }

    int dow    = (t.tm_wday == 0) ? 6 : t.tm_wday - 1; // 0=Mon…6=Sun
    uint8_t db = 1 << dow;

    for (int i = 0; i < MAX_SCHEDULES; i++) {
        if (!schedules[i].enabled)              continue;
        if (!(schedules[i].days & db))           continue;
        if (schedules[i].hour   != t.tm_hour)    continue;
        if (schedules[i].minute != t.tm_min)     continue;

        // Moisture check: skip if soil is already moist enough
        if (schedules[i].moistureThreshold > 0) {
            int ch = schedules[i].channel - 1;
            SENSOR_LOCK();
            bool assigned = sensorCfg[ch].assigned;
            bool valid    = sensorData[ch].valid;
            uint8_t m     = sensorData[ch].moisture;
            SENSOR_UNLOCK();
            if (assigned && valid && m >= schedules[i].moistureThreshold) {
                Serial.printf("Schedule %d skipped: moisture %d%% >= threshold %d%%\n",
                              i, m, schedules[i].moistureThreshold);
                continue;
            }
        }

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
header{background:#1565c0;color:#fff;padding:14px 20px;display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px}
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
.run{background:#1565c0;color:#fff}.stp{background:#e53935;color:#fff}
.sav{background:#2e7d32;color:#fff}.rst{background:#bf360c;color:#fff}
.sec{background:#546e7a;color:#fff}
.pw{margin-top:12px}
.pl{font-size:.73rem;color:#888;display:flex;justify-content:space-between;margin-bottom:4px}
.pb{height:8px;background:#e0e0e0;border-radius:4px;overflow:hidden}
.pf{height:100%;background:#1565c0;border-radius:4px;transition:width .6s ease}
.full{grid-column:1/-1}.two{display:grid;grid-template-columns:1fr 1fr;gap:8px}
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
.time-badge{font-size:.95rem;font-weight:600;color:#fff;font-variant-numeric:tabular-nums}
/* sensors */
.slots{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:10px;margin-bottom:14px}
.slot{border:2px dashed #dce3f0;border-radius:8px;padding:12px;font-size:.82rem}
.slot.asgn{border:2px solid #1565c0;border-style:solid}
.slot-hd{font-weight:600;color:#1565c0;margin-bottom:6px;font-size:.88rem}
.reading{display:grid;grid-template-columns:1fr 1fr;gap:4px;margin:8px 0;font-size:.8rem}
.reading span{color:#666}
.reading b{color:#333}
.scan-item{display:flex;justify-content:space-between;align-items:center;padding:8px 4px;border-bottom:1px solid #f0f0f0;font-size:.82rem}
.scan-item .flora{color:#2e7d32;font-size:.72rem;font-weight:600;margin-left:4px}
.afrm{background:#f8f9ff;border:1px solid #c5cae9;border-radius:8px;padding:12px;margin-top:8px;display:none}
.afrm-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
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

  <!-- Canales -->
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
    <div class="pw"><div class="pl"><span>Progreso</span><span id="p1t">0%</span></div>
    <div class="pb"><div class="pf" id="p1" style="width:0%"></div></div></div>
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
    <div class="pw"><div class="pl"><span>Progreso</span><span id="p2t">0%</span></div>
    <div class="pb"><div class="pf" id="p2" style="width:0%"></div></div></div>
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
    <div class="pw"><div class="pl"><span>Progreso</span><span id="p3t">0%</span></div>
    <div class="pb"><div class="pf" id="p3" style="width:0%"></div></div></div>
  </div>

  <!-- Parar todo -->
  <div class="card full" style="display:flex;align-items:center;gap:20px;flex-wrap:wrap">
    <h2 style="border:none;padding:0;margin:0;flex:none">Control global</h2>
    <button class="stp" style="max-width:200px;flex:unset;padding:10px 24px" onclick="stp(0)">&#9632; Parar todo</button>
  </div>

  <!-- ========= SENSORES MI FLORA ========= -->
  <div class="card full">
    <h2>&#127807; Sensores Mi Flora</h2>

    <!-- Canal slots -->
    <div class="slots" id="sensorSlots"></div>

    <!-- Scan controls -->
    <div class="row" style="max-width:280px;margin-bottom:10px">
      <button class="sec" id="scanBtn" onclick="startScan()">&#128268; Buscar dispositivos BLE</button>
    </div>
    <div id="scanStatus" style="font-size:.8rem;color:#666;margin-bottom:8px"></div>

    <!-- Scan results -->
    <div id="scanList"></div>

    <!-- Assign form (hidden until user clicks Asignar) -->
    <div class="afrm" id="assignForm">
      <b style="font-size:.88rem">Asignar sensor</b>
      <div class="afrm-grid" style="margin-top:10px">
        <div><label>Canal</label>
          <select id="aCh"><option value="0">Canal 1</option><option value="1">Canal 2</option><option value="2">Canal 3</option></select>
        </div>
        <div><label>Umbral humedad (%)</label>
          <input type="number" id="aTh" value="40" min="0" max="100">
        </div>
      </div>
      <small style="color:#888;display:block;margin-top:4px">0 = ignorar sensor en schedules</small>
      <div class="row" style="margin-top:10px">
        <button class="sav" onclick="doAssign()">Confirmar</button>
        <button class="sec" onclick="hideAssign()">Cancelar</button>
      </div>
    </div>

    <div class="row" style="max-width:220px;margin-top:12px">
      <button class="sec" onclick="readNow()">&#8635; Leer sensores ahora</button>
    </div>
    <div class="msg" id="sensorMsg"></div>
  </div>

  <!-- ========= PROGRAMACIÓN SEMANAL ========= -->
  <div class="card full">
    <h2>&#128197; Programaci&oacute;n semanal</h2>
    <div class="tz-row">
      <div style="flex:1"><label>Zona horaria (POSIX)</label>
        <input id="tzIn" placeholder="CET-1CEST,M3.5.0,M10.5.0/3">
      </div>
      <button class="sav" onclick="saveTz()">Guardar TZ</button>
    </div>
    <table>
      <thead><tr>
        <th>Canal</th><th>D&iacute;as</th><th>Hora</th>
        <th>Volumen</th><th>Vel.</th><th>Dir.</th>
        <th>Umbral</th><th>Activo</th><th></th>
      </tr></thead>
      <tbody id="schedBody"></tbody>
    </table>
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
      <div><label>Umbral humedad (%)</label>
        <input type="number" id="nmth" value="0" min="0" max="100">
        <small style="color:#aaa">0=sin sensor</small>
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
    <label>Pasos / mL</label><input type="number" id="cs" value="1600" min="1">
    <div class="row"><button class="sav" onclick="cal()">Guardar</button></div>
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
let assigningMAC = '', assigningName = '';
let sensors = [{},{},{}]; // latest sensor data from server

async function api(path, body, method) {
  try {
    const m = method || (body ? 'POST' : 'GET');
    const opts = body
      ? {method:m, headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)}
      : {method:m};
    return (await fetch(path, opts)).json();
  } catch(e) { return null; }
}

// ---- Motor ----
function run(ch) {
  api('/api/run', {channel:ch, volume:+$('v'+ch).value, speed:+$('s'+ch).value, direction:$('d'+ch).value});
}
function stp(ch) { api('/api/stop', {channel:ch}); }
function cal() {
  api('/api/calibrate', {channel:+$('cc').value, stepsperml:+$('cs').value})
    .then(()=>{ $('cm').textContent='Guardado'; setTimeout(()=>$('cm').textContent='',2500); });
}
function saveCfg() {
  api('/api/config', {server:$('ms').value, port:+$('mp').value, user:$('mu').value, pass:$('mw').value})
    .then(d=>{ $('mm').textContent=d?'Guardado':'Error'; setTimeout(()=>$('mm').textContent='',3000); });
}
function rstWifi() {
  if(confirm('¿Borrar WiFi y reiniciar?')) api('/api/resetwifi', {});
}

// ---- Sensors ----
function renderSensorSlots() {
  const html = [0,1,2].map(i => {
    const cfg = sensors[i]?.cfg;
    const rd  = sensors[i]?.reading;
    if (!cfg?.assigned) return `
      <div class="slot">
        <div class="slot-hd">Canal ${i+1}</div>
        <div style="color:#aaa;font-size:.8rem">Sin sensor asignado</div>
      </div>`;
    const ago = rd?.valid && rd.timestamp ? relTime(rd.timestamp) : 'Sin lectura';
    return `
      <div class="slot asgn">
        <div class="slot-hd">Canal ${i+1} — ${cfg.name||cfg.mac}</div>
        <div style="color:#888;font-size:.75rem">${cfg.mac}</div>
        ${rd?.valid ? `
        <div class="reading">
          <span>Humedad</span><b>${rd.moisture}%</b>
          <span>Temperatura</span><b>${rd.temperature?.toFixed(1)}°C</b>
          <span>Luz</span><b>${rd.light} lux</b>
          <span>Fertilidad</span><b>${rd.conductivity} µS/cm</b>
        </div>` : '<div style="color:#aaa;margin:6px 0;font-size:.78rem">Sin lectura</div>'}
        <div style="font-size:.73rem;color:#999;margin-bottom:6px">${ago} · Umbral: ${cfg.threshold>0?cfg.threshold+'%':'sin umbral'}</div>
        <button class="stp" style="padding:4px 10px;font-size:.78rem;flex:none" onclick="removeSensor(${i})">Quitar</button>
      </div>`;
  }).join('');
  $('sensorSlots').innerHTML = html;
}

function relTime(ts) {
  const diff = Math.floor(Date.now()/1000) - ts;
  if (diff < 60) return 'Hace <1 min';
  if (diff < 3600) return `Hace ${Math.floor(diff/60)} min`;
  return `Hace ${Math.floor(diff/3600)} h`;
}

function loadSensors() {
  api('/api/sensors').then(d => {
    if (!d) return;
    sensors = d;
    renderSensorSlots();
  });
}

function removeSensor(ch) {
  if (!confirm(`¿Quitar sensor del Canal ${ch+1}?`)) return;
  api('/api/sensors?channel='+ch, null, 'DELETE').then(loadSensors);
}

function readNow() {
  $('sensorMsg').textContent = 'Leyendo sensores...';
  api('/api/sensors/read', {}).then(() => {
    setTimeout(() => { loadSensors(); $('sensorMsg').textContent=''; }, 20000);
  });
}

// ---- BLE Scan ----
function startScan() {
  $('scanBtn').disabled = true;
  $('scanStatus').textContent = '&#128268; Escaneando... (10 s)';
  $('scanList').innerHTML = '';
  hideAssign();
  api('/api/ble/scan', {}).then(() => {
    setTimeout(() => {
      api('/api/ble/results').then(renderScanResults);
      $('scanBtn').disabled = false;
      $('scanStatus').textContent = '';
    }, 12000);
  });
}

function renderScanResults(data) {
  if (!data || !data.length) { $('scanList').innerHTML='<div style="color:#999;font-size:.82rem">No se encontraron dispositivos.</div>'; return; }
  const items = data.map(d => {
    const isFlora = /flower|flora|hhcc/i.test(d.name);
    return `<div class="scan-item">
      <div>
        <b>${d.name}</b>${isFlora?'<span class="flora">&#127807; Mi Flora</span>':''}
        <div style="color:#999;font-size:.73rem">${d.addr} &nbsp;·&nbsp; ${d.rssi} dBm</div>
      </div>
      <button class="sec" style="flex:none;padding:5px 12px;font-size:.8rem" onclick="showAssign('${d.addr}','${d.name.replace(/'/g,"\\'")}')">Asignar</button>
    </div>`;
  }).join('');
  $('scanList').innerHTML = items;
}

function showAssign(mac, name) {
  assigningMAC = mac; assigningName = name;
  $('assignForm').style.display = 'block';
  $('assignForm').scrollIntoView({behavior:'smooth', block:'nearest'});
}
function hideAssign() { $('assignForm').style.display='none'; }

function doAssign() {
  api('/api/sensors', {channel:+$('aCh').value, mac:assigningMAC, name:assigningName, threshold:+$('aTh').value})
    .then(d => {
      if (!d?.ok) { $('sensorMsg').textContent='Error'; return; }
      hideAssign();
      $('scanList').innerHTML='';
      loadSensors();
      $('sensorMsg').textContent='Sensor asignado';
      setTimeout(()=>$('sensorMsg').textContent='',2500);
    });
}

// ---- Schedules ----
function toggleDay(d) {
  selDays ^= (1<<d);
  document.querySelectorAll('.db').forEach(b=>b.classList.toggle('sel',!!(selDays&(1<<+b.dataset.d))));
}
function daysStr(mask) { return DAY_NAMES.filter((_,i)=>mask&(1<<i)).join(' ')||'—'; }

function loadSchedules() {
  api('/api/schedules').then(data => {
    if (!data) return;
    const tb = $('schedBody');
    tb.innerHTML = '';
    data.forEach(s => {
      if (!s || s.days===0) return;
      const tr = document.createElement('tr');
      tr.innerHTML =
        '<td>Ch.'+s.channel+'</td>'+
        '<td style="white-space:nowrap">'+daysStr(s.days)+'</td>'+
        '<td>'+String(s.hour).padStart(2,'0')+':'+String(s.minute).padStart(2,'0')+'</td>'+
        '<td>'+s.volume+' mL</td>'+
        '<td>'+s.speed+'</td>'+
        '<td>'+s.direction+'</td>'+
        '<td>'+(s.moistureThreshold>0?s.moistureThreshold+'%':'—')+'</td>'+
        '<td><input type="checkbox"'+(s.enabled?' checked':'')+' onchange="toggleSched('+s.id+',this.checked)"></td>'+
        '<td><button class="stp" style="padding:3px 10px;font-size:.78rem;flex:none" onclick="delSched('+s.id+')">&#10005;</button></td>';
      tb.appendChild(tr);
    });
  });
}

function addSched() {
  if (!selDays) { $('schedMsg').textContent='Selecciona al menos un día'; return; }
  const parts = $('nhm').value.split(':');
  api('/api/schedules', {
    channel:+$('nch').value, hour:+parts[0], minute:+parts[1],
    volume:+$('nvol').value, speed:+$('nspd').value,
    direction:$('ndir').value, days:selDays, enabled:true,
    moistureThreshold:+$('nmth').value
  }).then(d=>{
    if(!d?.ok){$('schedMsg').textContent='Error';return;}
    loadSchedules();
    $('schedMsg').textContent='Horario añadido';
    setTimeout(()=>$('schedMsg').textContent='',2500);
  });
}
function delSched(id) { if(confirm('¿Eliminar?')) api('/api/schedules?id='+id,null,'DELETE').then(loadSchedules); }
function toggleSched(id,en) { api('/api/schedules',{id,enabled:en}); }
function saveTz() {
  api('/api/timezone',{tz:$('tzIn').value})
    .then(()=>{$('schedMsg').textContent='Zona horaria guardada';setTimeout(()=>$('schedMsg').textContent='',2500);});
}

// ---- Status poll ----
function poll() {
  api('/api/status').then(d=>{
    if(!d)return;
    $('dW').className='dot on'; $('lW').textContent=d.ip||'WiFi';
    $('dM').className='dot '+(d.mqtt?'on':'off');
    for(let i=1;i<=3;i++){const p=d.motors[i-1]?.progress??0;$('p'+i).style.width=p+'%';$('p'+i+'t').textContent=p+'%';}
    if(d.time)$('clock').textContent=d.time;
  });
}

// ---- Init ----
api('/api/config').then(d=>{if(!d)return;$('ms').value=d.server||'';$('mp').value=d.port||1883;$('mu').value=d.user||'';});
api('/api/timezone').then(d=>{if(d)$('tzIn').value=d.tz||'';});
loadSchedules();
loadSensors();
setInterval(poll,1000);
setInterval(loadSensors,30000);
poll();
</script>
</body>
</html>
)rawliteral";


// =========================================================
// MOTOR ACTION HELPERS
// =========================================================

void doRun(int ch, float vol, float spd, const char* dir) {
    if (vol <= 0 || spd <= 0) return; // never enable driver with zero step target
    bool ccw = !strcmp(dir, "ccw");
    setEnableStepper(LOW);
    if (ch == 1) {
        float v=ccw?-vol:vol; SpeedToMove1=round(spd*StepsPerMili1/60.0f); StepsToMove1=(long)(v*StepsPerMili1);
        InitialSteps1=stepper1.getCurrentPositionInSteps(); TargetSteps1=InitialSteps1+StepsToMove1;
        Progress1=0; Stepper1Running=true;
        stepper1.setSpeedInStepsPerSecond(SpeedToMove1); stepper1.setTargetPositionRelativeInSteps(StepsToMove1);
    } else if (ch == 2) {
        float v=ccw?-vol:vol; SpeedToMove2=round(spd*StepsPerMili2/60.0f); StepsToMove2=(long)(v*StepsPerMili2);
        InitialSteps2=stepper2.getCurrentPositionInSteps(); TargetSteps2=InitialSteps2+StepsToMove2;
        Progress2=0; Stepper2Running=true;
        stepper2.setSpeedInStepsPerSecond(SpeedToMove2); stepper2.setTargetPositionRelativeInSteps(StepsToMove2);
    } else if (ch == 3) {
        float v=ccw?-vol:vol; SpeedToMove3=round(spd*StepsPerMili3/60.0f); StepsToMove3=(long)(v*StepsPerMili3);
        InitialSteps3=stepper3.getCurrentPositionInSteps(); TargetSteps3=InitialSteps3+StepsToMove3;
        Progress3=0; Stepper3Running=true;
        stepper3.setSpeedInStepsPerSecond(SpeedToMove3); stepper3.setTargetPositionRelativeInSteps(StepsToMove3);
    }
}

void doStop(int ch) {
    if (ch==0) { StepperStopped=0; Stepper1Running=Stepper2Running=Stepper3Running=false;
        stepper1.emergencyStop(); stepper2.emergencyStop(); stepper3.emergencyStop(); setEnableStepper(HIGH);
    } else if(ch==1){StepperStopped=1;Stepper1Running=false;stepper1.emergencyStop();}
    else if(ch==2){StepperStopped=2;Stepper2Running=false;stepper2.emergencyStop();}
    else if(ch==3){StepperStopped=3;Stepper3Running=false;stepper3.emergencyStop();}
}

void doCalibrate(int ch, long spm) {
    if (ch < 1 || ch > 3 || spm <= 0) return;
    if (ch==1) StepsPerMili1=spm;
    else if (ch==2) StepsPerMili2=spm;
    else StepsPerMili3=spm;
    PREFS_LOCK();
    prefs.begin("peristaltica", false);
    prefs.putLong(CAL_KEY[ch-1], spm);
    prefs.end();
    PREFS_UNLOCK();
}


// =========================================================
// STEPPER CALLBACKS
// =========================================================

// Stepper callbacks run in the stepper-service task. Keep them tiny:
// just record state and queue an MQTT event; loop() publishes later.
static void queueEvent(int ch, char ev, int ps) {
    pendingMqtt[ch-1].channel  = ch;
    pendingMqtt[ch-1].event    = ev;
    pendingMqtt[ch-1].progress = ps;
    pendingMqtt[ch-1].pending  = true;
}

void targetPositionReachedCallbackStepper1(long){Progress1=100;Stepper1Running=false;queueEvent(1,'D',100);}
void targetPositionReachedCallbackStepper2(long){Progress2=100;Stepper2Running=false;queueEvent(2,'D',100);}
void targetPositionReachedCallbackStepper3(long){Progress3=100;Stepper3Running=false;queueEvent(3,'D',100);}

void emergencyStopCallback1() {
    int ps = (TargetSteps1!=InitialSteps1)
        ? Progress1=round(100*(stepper1.getCurrentPositionInSteps()-InitialSteps1)/(float)(TargetSteps1-InitialSteps1)) : 0;
    queueEvent(1, 'S', ps);
}
void emergencyStopCallback2() {
    int ps = (TargetSteps2!=InitialSteps2)
        ? Progress2=round(100*(stepper2.getCurrentPositionInSteps()-InitialSteps2)/(float)(TargetSteps2-InitialSteps2)) : 0;
    queueEvent(2, 'S', ps);
}
void emergencyStopCallback3() {
    int ps = (TargetSteps3!=InitialSteps3)
        ? Progress3=round(100*(stepper3.getCurrentPositionInSteps()-InitialSteps3)/(float)(TargetSteps3-InitialSteps3)) : 0;
    queueEvent(3, 'S', ps);
}

// Drain any queued MQTT events. Called from loop() (Core 1).
void publishPendingMqttEvents() {
    if(!Stepper1Running&&!Stepper2Running&&!Stepper3Running) setEnableStepper(HIGH);
    if(!mqttEnabled) {
        for (int i=0; i<3; i++) pendingMqtt[i].pending = false;
        return;
    }
    for (int i = 0; i < 3; i++) {
        if (!pendingMqtt[i].pending) continue;
        PendingMqttEvent ev = const_cast<PendingMqttEvent&>(pendingMqtt[i]);
        pendingMqtt[i].pending = false;
        char buf[128]; StaticJsonDocument<128> doc;
        doc["type"]   = "done";
        doc["action"] = (ev.event == 'D') ? "run" : "stop";
        doc["channel"] = ev.channel;
        doc["progress"] = ev.progress;
        serializeJson(doc, buf);
        mqttClient.publish("peristaltica/status", 1, true, buf);
    }
}


// =========================================================
// PROGRESS
// =========================================================

void checkProgress() {
    long now = millis();
    if (now - ProgressPreviousMillis > ProgressInterval) {
        ProgressPreviousMillis = now;
        auto upd=[](bool run,ESP_FlexyStepper& st,long ini,long tgt,int& pct,int ch){
            if(!run)return;
            if(tgt!=ini)pct=round(100*(st.getCurrentPositionInSteps()-ini)/(float)(tgt-ini));
            if(!mqttEnabled)return;
            char buf[128];StaticJsonDocument<128>doc;
            doc["type"]="running";doc["action"]="run";doc["channel"]=ch;doc["progress"]=pct;
            serializeJson(doc,buf);mqttClient.publish("peristaltica/status",1,true,buf);
        };
        upd(Stepper1Running,stepper1,InitialSteps1,TargetSteps1,Progress1,1);
        upd(Stepper2Running,stepper2,InitialSteps2,TargetSteps2,Progress2,2);
        upd(Stepper3Running,stepper3,InitialSteps3,TargetSteps3,Progress3,3);
    }
    if(!Stepper1Running&&!Stepper2Running&&!Stepper3Running) setEnableStepper(HIGH);
}

// Trigger sensor reads every 15 minutes
void checkSensorTimer() {
    static long lastRead = -(15L*60*1000); // trigger immediately at first call
    if (millis() - lastRead > 15L*60*1000) {
        lastRead = millis();
        if (anySensorAssigned()) bleReadReq = true;
    }
}


// =========================================================
// WEB SERVER
// =========================================================

String currentTimeStr() {
    struct tm t; if(!getLocalTime(&t,0)) return "";
    char buf[10]; strftime(buf,sizeof(buf),"%H:%M:%S",&t); return String(buf);
}

void setupWebServer() {

    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req){
        req->send_P(200,"text/html",index_html);
    });

    webServer.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req){
        StaticJsonDocument<256> doc;
        doc["ip"]=WiFi.localIP().toString(); doc["mqtt"]=mqttClient.connected(); doc["time"]=currentTimeStr();
        JsonArray m=doc.createNestedArray("motors");
        JsonObject m1=m.createNestedObject(); m1["running"]=Stepper1Running; m1["progress"]=Progress1;
        JsonObject m2=m.createNestedObject(); m2["running"]=Stepper2Running; m2["progress"]=Progress2;
        JsonObject m3=m.createNestedObject(); m3["running"]=Stepper3Running; m3["progress"]=Progress3;
        char buf[256]; serializeJson(doc,buf); req->send(200,"application/json",buf);
    });

    webServer.on("/api/params", HTTP_GET, [](AsyncWebServerRequest* req){
        StaticJsonDocument<128> doc;
        doc["spm1"]=StepsPerMili1; doc["spm2"]=StepsPerMili2; doc["spm3"]=StepsPerMili3;
        char buf[128]; serializeJson(doc,buf); req->send(200,"application/json",buf);
    });

    webServer.on("/api/config", HTTP_GET, [](AsyncWebServerRequest* req){
        StaticJsonDocument<192> doc;
        doc["server"]=mqttServer; doc["port"]=mqttPort; doc["user"]=mqttUser;
        char buf[192]; serializeJson(doc,buf); req->send(200,"application/json",buf);
    });

    webServer.on("/api/schedules", HTTP_GET, [](AsyncWebServerRequest* req){
        StaticJsonDocument<2048> doc; JsonArray arr=doc.to<JsonArray>();
        for(int i=0;i<MAX_SCHEDULES;i++){
            if (schedules[i].days == 0) continue; // skip empty slots
            JsonObject o=arr.createNestedObject();
            o["id"]=i; o["enabled"]=schedules[i].enabled; o["channel"]=schedules[i].channel;
            o["days"]=schedules[i].days; o["hour"]=schedules[i].hour; o["minute"]=schedules[i].minute;
            o["volume"]=schedules[i].volume; o["speed"]=schedules[i].speed;
            o["direction"]=schedules[i].direction; o["moistureThreshold"]=schedules[i].moistureThreshold;
        }
        char buf[2048]; serializeJson(doc,buf); req->send(200,"application/json",buf);
    });

    webServer.on("/api/schedules", HTTP_DELETE, [](AsyncWebServerRequest* req){
        if(!req->hasParam("id")){req->send(400);return;}
        int id=req->getParam("id")->value().toInt();
        if(id<0||id>=MAX_SCHEDULES){req->send(400);return;}
        deleteSchedule(id); req->send(200,"application/json","{\"ok\":true}");
    });

    // GET /api/sensors — config + latest readings for all 3 channels
    webServer.on("/api/sensors", HTTP_GET, [](AsyncWebServerRequest* req){
        StaticJsonDocument<1024> doc; JsonArray arr=doc.to<JsonArray>();
        SENSOR_LOCK();
        SensorConfig  cfgCopy[3];
        SensorReading dataCopy[3];
        memcpy(cfgCopy,  sensorCfg,  sizeof(cfgCopy));
        memcpy(dataCopy, sensorData, sizeof(dataCopy));
        SENSOR_UNLOCK();
        for(int i=0;i<3;i++){
            JsonObject o=arr.createNestedObject();
            JsonObject cfg=o.createNestedObject("cfg");
            cfg["assigned"]=cfgCopy[i].assigned; cfg["mac"]=cfgCopy[i].mac;
            cfg["name"]=cfgCopy[i].name; cfg["threshold"]=cfgCopy[i].threshold;
            JsonObject rd=o.createNestedObject("reading");
            rd["valid"]=dataCopy[i].valid; rd["moisture"]=dataCopy[i].moisture;
            rd["temperature"]=dataCopy[i].temperature; rd["light"]=dataCopy[i].light;
            rd["conductivity"]=dataCopy[i].conductivity; rd["timestamp"]=(uint32_t)dataCopy[i].timestamp;
        }
        char buf[1024]; serializeJson(doc,buf); req->send(200,"application/json",buf);
    });

    // DELETE /api/sensors?channel=N
    webServer.on("/api/sensors", HTTP_DELETE, [](AsyncWebServerRequest* req){
        if(!req->hasParam("channel")){req->send(400);return;}
        int ch=req->getParam("channel")->value().toInt();
        if(ch<0||ch>2){req->send(400);return;}
        deleteSensorConfig(ch); req->send(200,"application/json","{\"ok\":true}");
    });

    // GET /api/ble/results
    webServer.on("/api/ble/results", HTTP_GET, [](AsyncWebServerRequest* req){
        if(bleScanning){req->send(200,"application/json","{\"scanning\":true}");return;}
        StaticJsonDocument<2048> doc; JsonArray arr=doc.to<JsonArray>();
        SENSOR_LOCK();
        int n = scanResultCount;
        BLEScanResult copy[MAX_SCAN_RESULTS];
        memcpy(copy, scanResults, sizeof(BLEScanResult)*n);
        SENSOR_UNLOCK();
        for(int i=0;i<n;i++){
            JsonObject o=arr.createNestedObject();
            o["addr"]=copy[i].addr; o["name"]=copy[i].name; o["rssi"]=copy[i].rssi;
        }
        char buf[2048]; serializeJson(doc,buf); req->send(200,"application/json",buf);
    });

    webServer.on("/api/timezone", HTTP_GET, [](AsyncWebServerRequest* req){
        PREFS_LOCK();
        prefs.begin("peristaltica",true);
        String tz=prefs.getString("timezone","CET-1CEST,M3.5.0,M10.5.0/3");
        prefs.end();
        PREFS_UNLOCK();
        StaticJsonDocument<128> doc; doc["tz"]=tz;
        char buf[128]; serializeJson(doc,buf); req->send(200,"application/json",buf);
    });

    // POST body handler — accumulates chunks until the full body is received,
    // then parses once. AsyncWebServer can split bodies across multiple calls.
    auto bodyHandler=[](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total){
        const size_t MAX_BODY = 1024;
        if (total > MAX_BODY) { req->send(413,"application/json","{\"ok\":false,\"error\":\"too_large\"}"); return; }

        // Buffer is allocated on first chunk and stored in the request _tempObject.
        char* buf = (char*) req->_tempObject;
        if (index == 0) {
            if (buf) { free(buf); buf = nullptr; }
            buf = (char*) malloc(total + 1);
            if (!buf) { req->send(500); return; }
            req->_tempObject = buf;
        }
        if (!buf) return;
        memcpy(buf + index, data, len);
        if (index + len < total) return; // wait for more chunks
        buf[total] = 0;

        StaticJsonDocument<512> doc;
        DeserializationError err = deserializeJson(doc, buf, total);
        free(buf); req->_tempObject = nullptr;
        if (err) { req->send(400,"application/json","{\"ok\":false}"); return; }

        String path=req->url();

        if(path=="/api/run"){
            int ch=doc["channel"]|0;
            if(ch<1||ch>3){req->send(400,"application/json","{\"ok\":false,\"error\":\"bad_channel\"}");return;}
            doRun(ch,doc["volume"]|0.0f,doc["speed"]|0.0f,doc["direction"]|"cw");
            req->send(200,"application/json","{\"ok\":true}");
        }
        else if(path=="/api/stop"){
            int ch=doc["channel"]|0;
            if(ch<0||ch>3){req->send(400,"application/json","{\"ok\":false,\"error\":\"bad_channel\"}");return;}
            doStop(ch);
            req->send(200,"application/json","{\"ok\":true}");
        }
        else if(path=="/api/calibrate"){
            int ch=doc["channel"]|0;
            if(ch<1||ch>3){req->send(400,"application/json","{\"ok\":false,\"error\":\"bad_channel\"}");return;}
            doCalibrate(ch,doc["stepsperml"]|1600L);
            req->send(200,"application/json","{\"ok\":true}");
        }
        else if(path=="/api/config"){
            saveMqttConfig(doc["server"]|"",doc["port"]|MQTT_PORT_DEFAULT,doc["user"]|"",doc["pass"]|"");
            if(mqttEnabled){mqttClient.disconnect();delay(200);
                mqttClient.setServer(mqttServer.c_str(),mqttPort);
                if(mqttUser.length())mqttClient.setCredentials(mqttUser.c_str(),mqttPass.c_str());
                mqttClient.connect();}
            req->send(200,"application/json","{\"ok\":true}");
        }
        else if(path=="/api/schedules"){
            int id=doc["id"]|-1;
            if(doc.containsKey("channel")){
                int c=doc["channel"]; if(c<1||c>3){req->send(400,"application/json","{\"ok\":false,\"error\":\"bad_channel\"}");return;}
            }
            if(id>=0&&id<MAX_SCHEDULES){
                if(doc.containsKey("enabled"))  schedules[id].enabled=doc["enabled"];
                if(doc.containsKey("days"))     schedules[id].days=doc["days"];
                if(doc.containsKey("hour"))     schedules[id].hour=doc["hour"];
                if(doc.containsKey("minute"))   schedules[id].minute=doc["minute"];
                if(doc.containsKey("volume"))   schedules[id].volume=doc["volume"];
                if(doc.containsKey("speed"))    schedules[id].speed=doc["speed"];
                if(doc.containsKey("channel"))  schedules[id].channel=doc["channel"];
                if(doc.containsKey("direction"))strlcpy(schedules[id].direction,doc["direction"]|"cw",4);
                if(doc.containsKey("moistureThreshold"))schedules[id].moistureThreshold=doc["moistureThreshold"];
                saveSchedule(id);
            } else {
                bool ok=false;
                for(int i=0;i<MAX_SCHEDULES&&!ok;i++){
                    if(schedules[i].days!=0)continue;
                    schedules[i].enabled=doc["enabled"]|true; schedules[i].channel=doc["channel"]|1;
                    schedules[i].days=doc["days"]|0; schedules[i].hour=doc["hour"]|0; schedules[i].minute=doc["minute"]|0;
                    schedules[i].volume=doc["volume"]|0.0f; schedules[i].speed=doc["speed"]|10.0f;
                    schedules[i].moistureThreshold=doc["moistureThreshold"]|0;
                    strlcpy(schedules[i].direction,doc["direction"]|"cw",4);
                    saveSchedule(i); ok=true;
                }
                if(!ok){req->send(507,"application/json","{\"ok\":false,\"error\":\"full\"}");return;}
            }
            req->send(200,"application/json","{\"ok\":true}");
        }
        else if(path=="/api/sensors"){
            int ch=doc["channel"]|0;
            const char* mac = doc["mac"] | "";
            if(ch<0||ch>2){req->send(400);return;}
            if(!isValidMac(mac)){
                req->send(400,"application/json","{\"ok\":false,\"error\":\"bad_mac\"}");
                return;
            }
            SENSOR_LOCK();
            sensorCfg[ch].assigned=true;
            sensorCfg[ch].threshold=doc["threshold"]|0;
            strlcpy(sensorCfg[ch].mac,  mac, 18);
            strlcpy(sensorCfg[ch].name, doc["name"] |"", 40);
            SENSOR_UNLOCK();
            saveSensorConfig(ch);
            bleReadReq=true; // read new sensor immediately
            req->send(200,"application/json","{\"ok\":true}");
        }
        else if(path=="/api/sensors/read"){
            if(anySensorAssigned()) bleReadReq=true;
            req->send(200,"application/json","{\"ok\":true}");
        }
        else if(path=="/api/ble/scan"){
            if(!bleScanning) bleScanReq=true;
            req->send(200,"application/json","{\"ok\":true,\"scanning\":true}");
        }
        else if(path=="/api/timezone"){
            String tz=doc["tz"]|"CET-1CEST,M3.5.0,M10.5.0/3";
            PREFS_LOCK();
            prefs.begin("peristaltica",false);
            prefs.putString("timezone",tz);
            prefs.end();
            PREFS_UNLOCK();
            setenv("TZ",tz.c_str(),1); tzset();
            req->send(200,"application/json","{\"ok\":true}");
        }
        else if(path=="/api/resetwifi"){
            req->send(200,"application/json","{\"ok\":true}");
            delay(500);
            WiFi.disconnect(true, true); // erase NVS WiFi creds + AP config
            ESP.restart();
        }
        else { req->send(404); }
    };

    webServer.on("/api/run",          HTTP_POST,[](AsyncWebServerRequest*){},NULL,bodyHandler);
    webServer.on("/api/stop",         HTTP_POST,[](AsyncWebServerRequest*){},NULL,bodyHandler);
    webServer.on("/api/calibrate",    HTTP_POST,[](AsyncWebServerRequest*){},NULL,bodyHandler);
    webServer.on("/api/config",       HTTP_POST,[](AsyncWebServerRequest*){},NULL,bodyHandler);
    webServer.on("/api/schedules",    HTTP_POST,[](AsyncWebServerRequest*){},NULL,bodyHandler);
    webServer.on("/api/sensors",      HTTP_POST,[](AsyncWebServerRequest*){},NULL,bodyHandler);
    webServer.on("/api/sensors/read", HTTP_POST,[](AsyncWebServerRequest*){},NULL,bodyHandler);
    webServer.on("/api/ble/scan",     HTTP_POST,[](AsyncWebServerRequest*){},NULL,bodyHandler);
    webServer.on("/api/timezone",     HTTP_POST,[](AsyncWebServerRequest*){},NULL,bodyHandler);
    webServer.on("/api/resetwifi",    HTTP_POST,[](AsyncWebServerRequest*){},NULL,bodyHandler);

    webServer.begin();
    Serial.println("Web server started");
}


// =========================================================
// MQTT
// =========================================================

void connectToMqtt() {
    if(!mqttEnabled)return;
    if(mqttClient.connected())return;
    Serial.println("Connecting to MQTT..."); mqttClient.connect();
}
void startMqtt() {
    if(!mqttEnabled)return;
    mqttClient.setServer(mqttServer.c_str(),mqttPort);
    if(mqttUser.length())mqttClient.setCredentials(mqttUser.c_str(),mqttPass.c_str());
    mqttClient.connect();
}
void onMqttConnect(bool){
    Serial.println("MQTT connected");
    mqttClient.subscribe("peristaltica/action",1);
    mqttClient.publish("peristaltica/status",1,true,"Connected");
}
void onMqttDisconnect(AsyncMqttClientDisconnectReason){
    Serial.println("MQTT disconnected");
    if(WiFi.isConnected())xTimerStart(mqttReconnectTimer,0);
}
void onMqttMessage(char*,char* payload,AsyncMqttClientMessageProperties,size_t len,size_t,size_t){
    StaticJsonDocument<256> doc;
    if(deserializeJson(doc,payload,len))return;
    const char* action=doc["action"]|""; int ch=doc["channel"]|0;
    if(!strcmp(action,"run")) doRun(ch,doc["volume"]|0.0f,doc["speed"]|0.0f,doc["direction"]|"cw");
    else if(!strcmp(action,"stop")) doStop(ch);
    else if(!strcmp(action,"calibrate")) doCalibrate(ch,doc["stepsperml"]|1600L);
    else if(!strcmp(action,"params")){
        char buf[128];StaticJsonDocument<128>r;
        r["action"]="params";r["type"]="done";r["spm1"]=StepsPerMili1;r["spm2"]=StepsPerMili2;r["spm3"]=StepsPerMili3;
        serializeJson(r,buf);mqttClient.publish("peristaltica/status",1,true,buf);
    }
}


// =========================================================
// WiFi EVENT
// =========================================================

void reconnectWifi() { WiFi.reconnect(); }

// Proper TimerCallbackFunction_t wrappers (avoid reinterpret_cast UB)
static void mqttTimerCb(TimerHandle_t) { connectToMqtt(); }
static void wifiTimerCb(TimerHandle_t) { reconnectWifi(); }

void WiFiEvent(WiFiEvent_t event) {
    switch(event){
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.print("WiFi OK — "); Serial.println(WiFi.localIP()); connectToMqtt(); break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost"); xTimerStop(mqttReconnectTimer,0); xTimerStart(wifiReconnectTimer,0); break;
    default: break;
    }
}


// =========================================================
// CORE 0: OTA + BLE
// =========================================================

void core0assignments(void*) {
    // Detach this task from the IDLE-task watchdog: BLE scans and OTA can
    // exceed the 5s timeout on Core 0. (Replacement for deprecated
    // disableCore0WDT() which was removed in arduino-esp32 3.x.)
    esp_task_wdt_delete(NULL);
    for (;;) {
        ArduinoOTA.handle();
        if (bleScanReq) { bleScanReq=false; doBleScan(); }
        if (bleReadReq) { bleReadReq=false; doReadAllSensors(); }
        vTaskDelay(1);
    }
}


// =========================================================
// STEPPER SETUP
// =========================================================

void StepperSetup() {
    pinMode(Dir1,OUTPUT); pinMode(Step1,OUTPUT);
    pinMode(Dir2,OUTPUT); pinMode(Step2,OUTPUT);
    pinMode(Dir3,OUTPUT); pinMode(Step3,OUTPUT);
    pinMode(EnableStepper,OUTPUT); digitalWrite(EnableStepper,HIGH);

    stepper1.connectToPins(Step1,Dir1);
    stepper1.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
    stepper1.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
    stepper1.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
    stepper1.registerTargetPositionReachedCallback(targetPositionReachedCallbackStepper1);
    stepper1.registerEmergencyStopTriggeredCallback(emergencyStopCallback1);

    stepper2.connectToPins(Step2,Dir2);
    stepper2.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
    stepper2.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
    stepper2.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
    stepper2.registerTargetPositionReachedCallback(targetPositionReachedCallbackStepper2);
    stepper2.registerEmergencyStopTriggeredCallback(emergencyStopCallback2);

    stepper3.connectToPins(Step3,Dir3);
    stepper3.setSpeedInStepsPerSecond(SPEED_IN_STEPS_PER_SECOND);
    stepper3.setAccelerationInStepsPerSecondPerSecond(ACCELERATION_IN_STEPS_PER_SECOND);
    stepper3.setDecelerationInStepsPerSecondPerSecond(DECELERATION_IN_STEPS_PER_SECOND);
    stepper3.registerTargetPositionReachedCallback(targetPositionReachedCallbackStepper3);
    stepper3.registerEmergencyStopTriggeredCallback(emergencyStopCallback3);

    stepper1.startAsService(1);
    stepper2.startAsService(1);
    stepper3.startAsService(1);
}


// =========================================================
// EEPROM
// =========================================================

void loadCalibration() {
    PREFS_LOCK();
    prefs.begin("peristaltica", true);
    StepsPerMili1 = prefs.getLong(CAL_KEY[0], 1600);
    StepsPerMili2 = prefs.getLong(CAL_KEY[1], 1600);
    StepsPerMili3 = prefs.getLong(CAL_KEY[2], 1600);
    prefs.end();
    PREFS_UNLOCK();
    if (StepsPerMili1 <= 0) StepsPerMili1 = 1600;
    if (StepsPerMili2 <= 0) StepsPerMili2 = 1600;
    if (StepsPerMili3 <= 0) StepsPerMili3 = 1600;
}


// =========================================================
// SETUP
// =========================================================

void setup() {
    Serial.begin(115200);

    // Mutexes must exist before any task can take them
    prefsMutex  = xSemaphoreCreateMutex();
    sensorMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(core0assignments,"Core_0",10000,NULL,1,&C0,0);

    mqttReconnectTimer=xTimerCreate("mqttTimer",pdMS_TO_TICKS(2000),pdFALSE,(void*)0,mqttTimerCb);
    wifiReconnectTimer=xTimerCreate("wifiTimer",pdMS_TO_TICKS(2000),pdFALSE,(void*)0,wifiTimerCb);

    WiFi.onEvent(WiFiEvent);

    // Load all persistent state first so the captive portal / web UI / steppers
    // can all see calibrations, schedules, sensors and MQTT config from the
    // moment they come online.
    loadMqttConfig();
    loadCalibration();
    loadSchedules();
    loadSensorConfigs();

    // Start steppers early; they're independent of network state.
    StepperSetup();

    WiFiManager wm;
    wm.setConfigPortalTimeout(180); wm.setConnectTimeout(30); wm.setHostname("Peristaltica");
    if(!wm.autoConnect("Peristaltica-Setup")){
        Serial.println("WiFiManager timeout — restarting"); ESP.restart();
    }

    // MQTT
    mqttClient.onConnect(onMqttConnect); mqttClient.onDisconnect(onMqttDisconnect); mqttClient.onMessage(onMqttMessage);
    startMqtt();

    // Kick off NTP — non-blocking; checkSchedules() guards itself against pre-sync
    setupNTP();

    // BLE (NimBLE init — must be after WiFi for coexistence)
    NimBLEDevice::init("");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); // max TX power for better range

    // OTA + web server come up immediately so the device is reachable
    // even if NTP is still syncing.
    ArduinoOTA.setHostname("Peristaltica");
    ArduinoOTA.begin();
    setupWebServer();

    Serial.print("Ready — http://"); Serial.println(WiFi.localIP());
}


// =========================================================
// LOOP
// =========================================================

void loop() {
    publishPendingMqttEvents();
    checkProgress();
    checkSchedules();
    checkSensorTimer();
    delay(1);
}
