/*
 * Magneto - PEMF coil controller and monitor, for the ESP32-S3
 *
 *   Board   : esp32:esp32:esp32s3 (arduino-esp32 core 2.0.17), USB CDC On Boot = enabled
 *   Wi-Fi   : open soft AP "Magneto" -> http://192.168.4.1 (captive portal opens the page), or after
 *             Settings > Network a home network -> http://magneto.local
 *   Secure  : wss://magneto.local/ws or wss://192.168.4.1/ws for the installed web app (GitHub Pages),
 *             once the phone trusts the board's own certificate (tlshttps.cpp, /magneto.mobileconfig).
 *   Update  : ArduinoOTA on 3232, HTTP upload at /update, and the one-button online update that joins
 *             Wi-Fi and pulls the newest build from the GitHub "latest" release.
 *
 * Hardware (defaults; every pin is remappable in Settings):
 *   GPIO 7  TC4420 -> IRFB4410 low-side gate. HIGH = coil on; idles LOW with a pulldown.
 *   GPIO 1  49E hall, coil center        GPIO 2  49E hall, coil perimeter        (ADC1)
 *   GPIO 3  10k NTC, coil face           GPIO 4  10k NTC, MOSFET                 (ADC1)
 *   GPIO 5/6/8  status RGB (common anode hardwired to 3V3)   GPIO 9  coil-rim LED
 *
 * The treatment engine runs on the device, so it keeps going with no phone connected. The page is the
 * control panel. Safety: thermistor limits and dT/dt limits trip the coil off, latch across reboots,
 * and need an explicit reset with a fresh in-range reading (both behaviors configurable).
 *
 * Files: engine.h (plan/stage engine, host-tested), config.h + json.h (settings), coil.cpp (1 us pulse
 * ISR), sense.cpp (DMA ADC: 2x hall + 2x NTC, FFT, phase fold), leds.cpp (patterns), tlshttps.cpp
 * (secure socket), ghupdate.h (GitHub release update), webui.h (the page, generated).
 */
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_system.h"

#include "engine.h"
#include "json.h"
#include "config.h"
#include "coil.h"
#include "sense.h"
#include "leds.h"
#include "webui.h"
#include "ghupdate.h"
#include "tlshttps.h"

#define FW_VERSION "0.1.0"
#define STA_JOIN_MS 12000
#define MDNS_POKE_MS 90000UL
#define AP_SSID "Magneto"
#define AP_CHANNEL 6
#define AP_MAX_CLIENTS 4
#define HOST_NAME "magneto"
#define OTA_PORT 3232
#define CFG_MAX_BYTES 12000
#define PRESETS_MAX_BYTES 32000
#define IDLE_FIRST_MS 400
static bool fsOk = false;

// ---- shared state -------------------------------------------------------------------------------------------------
static mg::Engine eng;
static mg::Config cfg;
static SemaphoreHandle_t engMtx;
static volatile bool otaBusy = false;
static volatile uint32_t rebootAtMs = 0;
static volatile uint8_t joinPhase = 0;
static bool radioSta = false;
static bool staFailed = false;
static bool dnsOn = false;
static TaskHandle_t ledTaskHandle = nullptr;
static volatile bool gotIpFlag = false;
static uint32_t mdnsPokeAt = 0;
static uint8_t txLevel = 1;
static uint32_t boCount = 0;
static const char* rstName = "";
static volatile uint8_t updPhase = 0, updPct = 0, updErr = 0;
static volatile uint32_t updStartAt = 0;
static volatile uint32_t lastApiMs = 0;

static Preferences prefs;
static String cfgRaw;
static uint32_t cfgRev = 1;
static bool cfgDirty = false;
static uint32_t cfgDirtyAt = 0;

// coil drive as last applied, so loop() only reprograms the timer on a real change
static bool drvOn = false;
static float drvHz = 0, drvPw = 0;
static uint32_t stuckSince = 0;
static SenseSnap snapCache;              // refreshed each loop pass for safety checks and the LEDs

// Safari opens spare connections and sends nothing; drop a silent one when another is waiting.
class MagServer : public WebServer {
 public:
  explicit MagServer(int port) : WebServer(port) {}
  void serve() {
    if (_currentStatus == HC_WAIT_READ && _currentClient.connected() && !_currentClient.available() &&
        (millis() - _statusChange) > IDLE_FIRST_MS && _server.hasClient()) {
      _currentClient.stop();
      _currentClient = WiFiClient();
      _currentStatus = HC_NONE;
    }
    handleClient();
  }
};
static MagServer server(80);
static DNSServer dns;

#define LOCK() xSemaphoreTake(engMtx, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(engMtx)

static void applyHardware() {
  ledsApply(cfg.pins);
  coilInit(cfg.pins.drive);
  senseBegin(cfg.pins.hallC, cfg.pins.hallP, cfg.pins.thC, cfg.pins.thF);
  SenseCal cal{cfg.hall.sens5c, cfg.hall.sens5p, cfg.hall.vcc, cfg.hall.ratio,
               cfg.therm.beta, cfg.therm.r0k, cfg.therm.rsk, cfg.therm.top};
  senseSetCal(cal);
}

// ---- LED task: 250 Hz --------------------------------------------------------------------------------------------
static void ledTask(void*) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(4) > 0 ? pdMS_TO_TICKS(4) : 1;
  SenseSnap sn = {};
  int skip = 0;
  for (;;) {
    if (++skip >= 10) { skip = 0; senseCopy(sn); }     // data feedback needs ~25 Hz, not 250
    int64_t now = esp_timer_get_time();
    LedInput in;
    LOCK();
    in.state = eng.state;
    in.trip = eng.trip;
    double t = eng.elapsed(now);
    int wi = eng.windowAt(t);
    in.stage = wi >= 0 ? eng.win[wi].stage : -1;
    in.elapsed = t;
    in.total = eng.total;
    mg::Config snapshot = cfg;
    UNLOCK();
    in.joinPhase = joinPhase;
    in.updPhase = updPhase;
    in.updPct = updPct;
    in.updErr = updErr;
    in.otaBusy = otaBusy;
    in.client = (uint32_t)(millis() - lastApiMs) < 10000;
    switch (snapshot.led.dataSrc) {
      case 1: in.dataVal = sn.tF; break;
      case 2: in.dataVal = sn.bAvg0; break;
      case 3: in.dataVal = sn.dbMax; break;
      default: in.dataVal = sn.tC; break;
    }
    ledsRender(snapshot, in, now);
    vTaskDelayUntil(&last, period);
  }
}

// ---- settings storage ---------------------------------------------------------------------------------------------
static void cfgSaveNow() {
  prefs.putBytes("cfg", cfgRaw.c_str(), cfgRaw.length());
  cfgDirty = false;
}

static void cfgLoad() {
  size_t n = prefs.getBytesLength("cfg");
  mg::Config c;
  c.setDefaults();
  if (n > 0 && n <= CFG_MAX_BYTES) {
    char* buf = (char*)malloc(n + 1);
    if (buf) {
      prefs.getBytes("cfg", buf, n);
      buf[n] = 0;
      bool pinsOk = true;
      if (mg::cfgFromJson(buf, n, c, &pinsOk)) cfgRaw = String(buf);
      else c.setDefaults();
      free(buf);
    }
  }
  cfg = c;
  eng.configure(cfg.plan);
}

// ---- web API --------------------------------------------------------------------------------------------------------
static void sendJson(int code, const char* body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

static int stateJson(char* buf, size_t cap) {
  int64_t now = esp_timer_get_time();
  SenseSnap s;
  senseCopy(s);
  LOCK();
  double t = eng.elapsed(now);
  if (eng.total > 0 && t > eng.total) t = eng.total;
  int wi = eng.windowAt(t);
  float hz = 0, pw = 0;
  int stageIdx = wi >= 0 ? eng.win[wi].stage : -1;
  double stageT = 0;
  if (eng.state == mg::ST_RUN) eng.drive(now, &hz, &pw, &stageIdx, &stageT);
  else if (stageIdx >= 0) { hz = eng.plan.st[stageIdx].hz; pw = eng.plan.st[stageIdx].pwUs; }
  int n = snprintf(buf, cap,
    "{\"app\":\"magneto\",\"ver\":\"%s\",\"s\":\"%s\",\"t\":%.2f,\"total\":%.1f,\"stage\":%d,\"nwin\":%d,"
    "\"hz\":%.1f,\"pw\":%.0f,\"duty\":%.5f,\"trip\":%d,\"rev\":%lu,\"ota\":%d,\"up\":%lu,\"net\":\"%s\",\"fail\":%d,"
    "\"rst\":\"%s\",\"bo\":%lu,\"heap\":%lu,\"rssi\":%d,\"ps\":%d,\"fftn\":%d,"
    "\"b0\":%.2f,\"b1\":%.2f,\"db\":%.3f,\"bm0\":%.2f,\"bm1\":%.2f,\"dbm\":%.3f,"
    "\"ba0\":%.2f,\"ba1\":%.2f,\"brms\":%.2f,\"ss\":%.1f,\"f\":%.2f,\"fa\":%.2f,\"snr\":%.1f,"
    "\"z0\":%d,\"z1\":%d,\"clip\":%d,\"fs\":%.0f,"
    "\"tc\":%.1f,\"tf\":%.1f,\"dc\":%.1f,\"dfr\":%.1f,\"okc\":%d,\"okf\":%d}",
    FW_VERSION, mg::stateName(eng.state), t, eng.total, stageIdx, eng.nWin,
    hz, pw, mg::dutyOf(hz, pw) * 100.0, (int)eng.trip, (unsigned long)cfgRev, otaBusy ? 1 : 0,
    (unsigned long)(millis() / 1000), radioSta ? "sta" : "ap", staFailed ? 1 : 0,
    rstName, (unsigned long)boCount, (unsigned long)ESP.getFreeHeap(), radioSta ? (int)WiFi.RSSI() : 0,
    sensePsram() ? 1 : 0, senseFftPoints(),
    s.b0, s.b1, s.db, s.bMax0, s.bMax1, s.dbMax,
    s.bAvg0, s.bAvg1, s.bRms0, s.statSec, s.pkHz, s.pkAmp, s.pkSnr,
    s.zeroed0 ? 1 : 0, s.zeroed1 ? 1 : 0, s.clip ? 1 : 0, s.fs,
    isnan(s.tC) ? 0.0 : s.tC, isnan(s.tF) ? 0.0 : s.tF, s.dtdtC, s.dtdtF, s.okC ? 1 : 0, s.okF ? 1 : 0);
  UNLOCK();
  return (n > 0 && (size_t)n < cap) ? n : -1;
}

static void sendState() {
  char buf[1100];
  lastApiMs = millis();
  if (stateJson(buf, sizeof buf) < 0) { sendJson(500, "{\"ok\":false}"); return; }
  sendJson(200, buf);
}

// start/stop/pause/reset/tune and every measurement command, one endpoint
static void handleCmd() {
  lastApiMs = millis();
  String c = server.arg("c");
  int64_t now = esp_timer_get_time();
  const char* err = nullptr;
  SenseSnap s;
  senseCopy(s);
  LOCK();
  if (c == "start") {
    if (eng.state == mg::ST_TRIP) err = "trip";
    else if (cfg.safe.on && cfg.safe.needProbe && (!s.okC || !s.okF)) err = "probe";
    else if (!eng.start(now)) err = "state";
  } else if (c == "pause") {
    if (!eng.pause(now)) err = "state";
  } else if (c == "stop") {
    if (eng.state == mg::ST_TRIP) err = "trip";
    else eng.stop();
  } else if (c == "resettime") {
    eng.resetTime(now);
  } else if (c == "tune") {
    float hz = server.arg("hz").toFloat();
    float pw = server.arg("pw").toFloat();
    if (cfg.maxHz > 0 && hz > cfg.maxHz) hz = cfg.maxHz;
    if (cfg.maxPw > 0 && pw > cfg.maxPw) pw = cfg.maxPw;
    if (!eng.tune(now, hz, pw)) err = "state";
  } else if (c == "tripreset") {
    if (eng.state != mg::ST_TRIP) err = "state";
    else if (cfg.safe.on && cfg.safe.needReset) {
      bool tempTrip = eng.trip >= mg::TR_COIL_TEMP && eng.trip <= mg::TR_FET_RATE;
      bool cool = s.okC && s.okF && (!s.okC || s.tC <= cfg.safe.coilMaxF) && (!s.okF || s.tF <= cfg.safe.fetMaxF);
      if (tempTrip && !cool) err = "cool";
      else { eng.tripReset(); prefs.putUChar("trip", 0); }
    } else {
      eng.tripReset();
      prefs.putUChar("trip", 0);
    }
  } else if (c == "zero") {
    senseZero(server.arg("ch").toInt());
  } else if (c == "zeroclear") {
    senseZeroClear(server.arg("ch").toInt());
  } else if (c == "clear") {
    String w = server.arg("w");
    senseClearStats(w == "max" ? SENSE_CLR_MAX : (w == "avg" ? SENSE_CLR_AVG : SENSE_CLR_ALL));
  } else {
    err = "cmd";
  }
  UNLOCK();
  if (err) {
    char b[64];
    snprintf(b, sizeof b, "{\"ok\":false,\"err\":\"%s\"}", err);
    sendJson(c == "start" || c == "tripreset" ? 409 : 400, b);
    return;
  }
  sendState();
}

static void handleConfigGet() { sendJson(200, cfgRaw.length() ? cfgRaw.c_str() : "{}"); }

static void handleConfigPost() {
  if (!server.hasArg("plain")) { sendJson(400, "{\"ok\":false,\"err\":\"no body\"}"); return; }
  const String& body = server.arg("plain");
  if (body.length() > CFG_MAX_BYTES) { sendJson(413, "{\"ok\":false,\"err\":\"too big\"}"); return; }
  mg::Config c;
  bool pinsOk = true;
  if (!mg::cfgFromJson(body.c_str(), body.length(), c, &pinsOk)) { sendJson(400, "{\"ok\":false,\"err\":\"bad json\"}"); return; }
  bool pinsChanged;
  LOCK();
  bool running = eng.running();
  pinsChanged = memcmp(&c.pins, &cfg.pins, sizeof c.pins) != 0;
  if (pinsChanged && running) {
    UNLOCK();
    sendJson(409, "{\"ok\":false,\"err\":\"run\"}");      // never rewire a live coil
    return;
  }
  cfg = c;
  eng.configure(cfg.plan);
  UNLOCK();
  if (pinsChanged) applyHardware();
  else {
    SenseCal cal{cfg.hall.sens5c, cfg.hall.sens5p, cfg.hall.vcc, cfg.hall.ratio,
                 cfg.therm.beta, cfg.therm.r0k, cfg.therm.rsk, cfg.therm.top};
    senseSetCal(cal);
  }
  if (body != cfgRaw) {
    cfgRaw = body;
    cfgDirty = true;
    cfgDirtyAt = millis();
  }
  lastApiMs = millis();
  cfgRev++;
  char buf[64];
  snprintf(buf, sizeof buf, "{\"ok\":true,\"rev\":%lu,\"pins\":%s}", (unsigned long)cfgRev, pinsOk ? "true" : "false");
  sendJson(200, buf);
}

static void handleRoot() {
  server.sendHeader("Content-Encoding", "gzip");
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html; charset=utf-8", (const char*)WEBUI_GZ, WEBUI_GZ_LEN);
}

// ---- measurement payloads -------------------------------------------------------------------------------------------
static char* bigBuf(size_t cap) {
  static char* buf = nullptr;
  static size_t have = 0;
  if (cap > have) {
    free(buf);
    buf = (char*)malloc(cap);
    have = buf ? cap : 0;
  }
  return buf;
}

static void handleWave() {
  lastApiMs = millis();
  int ch = server.arg("ch").toInt();
  uint32_t ms = (uint32_t)server.arg("ms").toInt();
  if (ms < 10) ms = 10;
  if (ms > 10000) ms = 10000;
  static float mn[240], mx[240];
  float fs = 0;
  int n = senseWave(ch, ms, 240, mn, mx, &fs);
  size_t cap = 100 + (size_t)n * 26;
  char* out = bigBuf(cap);
  if (!out) { sendJson(500, "{\"ok\":false}"); return; }
  int p = snprintf(out, cap, "{\"fs\":%.0f,\"ms\":%u,\"n\":%d,\"mn\":[", fs, (unsigned)ms, n);
  for (int i = 0; i < n; i++) p += snprintf(out + p, cap - p, "%s%.1f", i ? "," : "", mn[i]);
  p += snprintf(out + p, cap - p, "],\"mx\":[");
  for (int i = 0; i < n; i++) p += snprintf(out + p, cap - p, "%s%.1f", i ? "," : "", mx[i]);
  snprintf(out + p, cap - p, "]}");
  sendJson(200, out);
}

static void handleSpec() {
  lastApiMs = millis();
  static float bins[256];
  float df = 0;
  int n = senseSpec(bins, 256, &df);
  SenseSnap s;
  senseCopy(s);
  size_t cap = 160 + (size_t)n * 13;
  char* out = bigBuf(cap);
  if (!out) { sendJson(500, "{\"ok\":false}"); return; }
  int p = snprintf(out, cap, "{\"df\":%.4f,\"n\":%d,\"pk\":{\"hz\":%.2f,\"amp\":%.2f,\"snr\":%.1f},\"a\":[", df, n, s.pkHz, s.pkAmp, s.pkSnr);
  for (int i = 0; i < n; i++) p += snprintf(out + p, cap - p, "%s%.2f", i ? "," : "", bins[i]);
  snprintf(out + p, cap - p, "]}");
  sendJson(200, out);
}

static void handleFold() {
  lastApiMs = millis();
  static float bins[240];
  uint32_t per = 0, cyc = 0;
  int n = senseFold(bins, 240, &per, &cyc);
  size_t cap = 120 + (size_t)n * 13;
  char* out = bigBuf(cap);
  if (!out) { sendJson(500, "{\"ok\":false}"); return; }
  int p = snprintf(out, cap, "{\"per\":%u,\"cyc\":%u,\"n\":%d,\"b\":[", (unsigned)per, (unsigned)cyc, n);
  for (int i = 0; i < n; i++) p += snprintf(out + p, cap - p, "%s%.1f", i ? "," : "", bins[i]);
  snprintf(out + p, cap - p, "]}");
  sendJson(200, out);
}

// ---- iOS certificate profile ------------------------------------------------------------------------------------------
static String pemBody(const String& pem) {
  String o;
  int i = 0;
  while (i < (int)pem.length()) {
    int j = pem.indexOf('\n', i);
    if (j < 0) j = pem.length();
    String line = pem.substring(i, j);
    line.trim();
    if (line.length() && line.indexOf("BEGIN ") < 0 && line.indexOf("END ") < 0) o += line;
    i = j + 1;
  }
  return o;
}

static void handleCertProfile() {
  if (!fsOk || !LittleFS.exists("/ca.crt")) { server.send(503, "text/plain", "certificate not ready"); return; }
  File f = LittleFS.open("/ca.crt", "r");
  if (!f) { server.send(503, "text/plain", "certificate not ready"); return; }
  String b64 = pemBody(f.readString());
  f.close();
  if (!b64.length()) { server.send(500, "text/plain", "certificate unreadable"); return; }
  String xml;
  xml.reserve(b64.length() + 900);
  xml += "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  xml += "<!DOCTYPE plist PUBLIC \"-//Apple//DTD PLIST 1.0//EN\" \"http://www.apple.com/DTDs/PropertyList-1.0.dtd\">\n";
  xml += "<plist version=\"1.0\"><dict><key>PayloadContent</key><array><dict>";
  xml += "<key>PayloadCertificateFileName</key><string>magneto.cer</string>";
  xml += "<key>PayloadContent</key><data>";
  xml += b64;
  xml += "</data><key>PayloadDescription</key><string>Magneto local certificate</string>";
  xml += "<key>PayloadDisplayName</key><string>Magneto</string>";
  xml += "<key>PayloadIdentifier</key><string>local.magneto.ca</string>";
  xml += "<key>PayloadType</key><string>com.apple.security.root</string>";
  xml += "<key>PayloadUUID</key><string>b4e7c2d1-8f3a-4e60-9c15-000000000001</string>";
  xml += "<key>PayloadVersion</key><integer>1</integer></dict></array>";
  xml += "<key>PayloadDisplayName</key><string>Magneto</string>";
  xml += "<key>PayloadIdentifier</key><string>local.magneto.profile</string>";
  xml += "<key>PayloadRemovalDisallowed</key><false/>";
  xml += "<key>PayloadType</key><string>Configuration</string>";
  xml += "<key>PayloadUUID</key><string>b4e7c2d1-8f3a-4e60-9c15-000000000002</string>";
  xml += "<key>PayloadVersion</key><integer>1</integer></dict></plist>\n";
  server.sendHeader("Content-Disposition", "attachment; filename=\"magneto.mobileconfig\"");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/x-apple-aspen-config", xml);
}

// ---- saved setups -------------------------------------------------------------------------------------------------
static void handlePresetsGet() {
  server.sendHeader("Cache-Control", "no-store");
  if (fsOk && LittleFS.exists("/presets.json")) {
    File f = LittleFS.open("/presets.json", "r");
    if (f) { server.streamFile(f, "application/json"); f.close(); return; }
  }
  server.send(200, "application/json", "[]");
}
static void handlePresetsPost() {
  if (!fsOk) { sendJson(503, "{\"ok\":false,\"err\":\"no storage\"}"); return; }
  if (!server.hasArg("plain")) { sendJson(400, "{\"ok\":false,\"err\":\"no body\"}"); return; }
  const String& body = server.arg("plain");
  size_t n = body.length();
  if (n > PRESETS_MAX_BYTES) { sendJson(413, "{\"ok\":false,\"err\":\"too big\"}"); return; }
  if (!mg::presetsJsonOk(body.c_str(), n)) { sendJson(400, "{\"ok\":false,\"err\":\"bad json\"}"); return; }
  File f = LittleFS.open("/presets.tmp", "w");
  if (!f) { sendJson(500, "{\"ok\":false}"); return; }
  size_t w = f.write((const uint8_t*)body.c_str(), n);
  f.close();
  if (w != n) { LittleFS.remove("/presets.tmp"); sendJson(500, "{\"ok\":false}"); return; }
  LittleFS.remove("/presets.json");
  LittleFS.rename("/presets.tmp", "/presets.json");
  lastApiMs = millis();
  sendJson(200, "{\"ok\":true}");
}

// ---- online update from GitHub --------------------------------------------------------------------------------------
extern const uint8_t rootca_crt_bundle_start[] asm("_binary_x509_crt_bundle_start");

static String jsonEsc(const String& s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c < 0x20) o += ' ';
    else o += c;
  }
  return o;
}

static void handleUpdInfo() {
  String ssid = prefs.getString("wSsid", ""), repo = prefs.getString("wRepo", GH_REPO_DEFAULT);
  bool hasPass = prefs.getString("wPass", "").length() > 0, hasTok = prefs.getString("wTok", "").length() > 0;
  String r = String("{\"ssid\":\"") + jsonEsc(ssid) + "\",\"hasPass\":" + (hasPass ? "true" : "false") + ",\"repo\":\"" + jsonEsc(repo) +
             "\",\"hasToken\":" + (hasTok ? "true" : "false") + ",\"last\":\"" + jsonEsc(prefs.getString("wRes", "")) +
             "\",\"from\":\"" + jsonEsc(prefs.getString("wFrom", "")) + "\",\"ver\":\"" FW_VERSION "\"}";
  sendJson(200, r.c_str());
}

static void handleUpdSave() {
  if (server.hasArg("ssid")) { String v = server.arg("ssid"); if (v.length() > 32) { sendJson(400, "{\"ok\":false,\"err\":\"ssid\"}"); return; } prefs.putString("wSsid", v); }
  if (server.hasArg("pass")) { String v = server.arg("pass"); if (v.length() > 63) { sendJson(400, "{\"ok\":false,\"err\":\"pass\"}"); return; } prefs.putString("wPass", v); }
  if (server.hasArg("repo")) {
    String v = server.arg("repo");
    if (v.length() == 0) v = GH_REPO_DEFAULT;
    if (!mg::ghRepoOk(v.c_str())) { sendJson(400, "{\"ok\":false,\"err\":\"repo\"}"); return; }
    prefs.putString("wRepo", v);
  }
  if (server.hasArg("token")) {
    String v = server.arg("token");
    if (!mg::ghTokenOk(v.c_str())) { sendJson(400, "{\"ok\":false,\"err\":\"token\"}"); return; }
    prefs.putString("wTok", v);
  }
  sendJson(200, "{\"ok\":true}");
}

static void handleUpdStart() {
  if (prefs.getString("wSsid", "").length() == 0) { sendJson(400, "{\"ok\":false,\"err\":\"ssid\"}"); return; }
  LOCK();
  bool running = eng.running();
  UNLOCK();
  if (running || otaBusy || updPhase) { sendJson(409, "{\"ok\":false,\"err\":\"busy\"}"); return; }
  sendJson(200, "{\"ok\":true}");
  updStartAt = millis() + 900;            // let this reply reach the phone before the AP goes away
  if (!updStartAt) updStartAt = 1;
}

static bool updFail(uint8_t code, const char* why) {
  updErr = code;
  Serial.printf("online update failed (%u): %s\n", (unsigned)code, why);
  return false;
}

static bool ghFetch(WiFiClientSecure& cl, HTTPClient& http, String url, const String& tok, const char* accept) {
  for (int hop = 0; hop < 6; hop++) {
    char host[80];
    mg::ghUrlHost(url.c_str(), host, sizeof host);
    if (!host[0]) return false;
    http.end();
    cl.stop();
    if (!http.begin(cl, url)) return false;
    http.setUserAgent("magneto");
    http.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
    http.setTimeout(15000);
    http.addHeader("Accept", accept);
    if (tok.length() && mg::ghIsApiHost(host)) http.addHeader("Authorization", String("Bearer ") + tok);
    const char* keys[] = {"Location"};
    http.collectHeaders(keys, 1);
    int code = http.GET();
    if (code == 200) return true;
    if (code == 301 || code == 302 || code == 303 || code == 307 || code == 308) {
      url = http.header("Location");
      if (url.length() == 0) return false;
      continue;
    }
    Serial.printf("HTTP %d from %s\n", code, host);
    return false;
  }
  return false;
}

static void runWebUpdate() {
  if (cfgDirty) cfgSaveNow();
  coilOff();
  senseHold(true);
  String ssid = prefs.getString("wSsid", ""), pass = prefs.getString("wPass", "");
  String repo = prefs.getString("wRepo", GH_REPO_DEFAULT), tok = prefs.getString("wTok", "");
  prefs.putString("wFrom", FW_VERSION);
  prefs.putString("wRes", "");
  otaBusy = true; updErr = 0; updPct = 0; updPhase = 1;
  Serial.println("online update: joining Wi-Fi");

  tlsStop();
  server.stop();
  if (dnsOn) { dns.stop(); dnsOn = false; }
  ArduinoOTA.end();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid.c_str(), pass.c_str());

  bool ok = true;
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 25000) delay(100);
  if (WiFi.status() != WL_CONNECTED) ok = updFail(mg::UE_WIFI, "could not join the Wi-Fi network");

  if (ok) {
    updPhase = 2;
    configTime(0, 0, "pool.ntp.org", "time.cloudflare.com", "time.google.com");
    t0 = millis();
    while (time(nullptr) < 1700000000 && millis() - t0 < 15000) delay(100);
    if (time(nullptr) < 1700000000) ok = updFail(mg::UE_TIME, "no internet time");
  }

  WiFiClientSecure cl;
  HTTPClient http;
  bool begun = false;
  if (ok) {
    updPhase = 3;
    cl.setCACertBundle(rootca_crt_bundle_start);
    String url = String("https://github.com/") + repo + "/releases/download/" GH_TAG "/" GH_ASSET;
    const char* accept = "application/octet-stream";
    if (tok.length()) {
      url = String("https://api.github.com/repos/") + repo + "/releases/tags/" GH_TAG;
      if (!ghFetch(cl, http, url, tok, "application/vnd.github+json")) ok = updFail(mg::UE_LOOKUP, "release lookup");
      else {
        String js = http.getString();
        http.end();
        char au[200];
        if (!mg::ghFindAssetUrl(js.c_str(), js.length(), GH_ASSET, au, sizeof au)) ok = updFail(mg::UE_LOOKUP, "asset not in release");
        else url = au;
      }
    }
    if (ok && !ghFetch(cl, http, url, tok, accept)) ok = updFail(mg::UE_LOOKUP, "download request");
  }

  if (ok) {
    int len = http.getSize();
    if (len < 200000 || len > 0x140000) ok = updFail(mg::UE_DOWNLOAD, "unexpected file size");
    else if (!Update.begin((size_t)len, U_FLASH)) ok = updFail(mg::UE_FLASH, "no room for the update");
    else {
      begun = true;
      updPhase = 4;
      WiFiClient* stream = http.getStreamPtr();
      uint8_t buf[1024];
      size_t done = 0;
      uint32_t lastData = millis();
      while (done < (size_t)len) {
        size_t avail = stream->available();
        if (avail) {
          size_t want = avail < sizeof buf ? avail : sizeof buf;
          if (want > (size_t)len - done) want = (size_t)len - done;
          int n = stream->readBytes(buf, want);
          if (n <= 0) continue;
          if (Update.write(buf, (size_t)n) != (size_t)n) { ok = updFail(mg::UE_FLASH, "flash write"); break; }
          done += (size_t)n;
          updPct = (uint8_t)((uint64_t)done * 100 / (size_t)len);
          lastData = millis();
        } else {
          if (!stream->connected()) break;
          if (millis() - lastData > 20000) { ok = updFail(mg::UE_DOWNLOAD, "download stalled"); break; }
          delay(2);
        }
      }
      if (ok && done != (size_t)len) ok = updFail(mg::UE_DOWNLOAD, "download ended early");
      if (ok && !Update.end(true)) ok = updFail(mg::UE_FLASH, "image check failed");
    }
  }
  http.end();
  if (!ok && begun) Update.abort();

  if (ok) {
    prefs.putString("wRes", "ok");
    updPhase = 5;
    Serial.println("online update installed, restarting");
    delay(2500);
  } else {
    prefs.putString("wRes", String("e") + String((unsigned)updErr));
    updPhase = 6;
    delay(12000);
  }
  ESP.restart();
}

static void handleReboot() {
  sendJson(200, "{\"ok\":true}");
  rebootAtMs = millis() + 600;
}

// Firmware upload from the page (multipart POST to /update).
static void handleUpdateDone() {
  bool ok = !Update.hasError();
  server.sendHeader("Connection", "close");
  sendJson(ok ? 200 : 500, ok ? "{\"ok\":true}" : "{\"ok\":false}");
  if (ok) rebootAtMs = millis() + 800;
  otaBusy = false;
  if (!ok) senseHold(false);
}

static void handleUpdateData() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    otaBusy = true;
    coilOff();
    LOCK();
    if (eng.state == mg::ST_RUN) eng.pause(esp_timer_get_time());
    UNLOCK();
    senseHold(true);
    Serial.printf("HTTP update: %s\n", up.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_END) {
    if (Update.end(true)) Serial.printf("HTTP update done, %u bytes\n", (unsigned)up.totalSize);
    else Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    otaBusy = false;
    senseHold(false);
  }
}

// ---- network ---------------------------------------------------------------------------------------------------------
static void handleNetInfo() {
  String ssid = prefs.getString("wSsid", "");
  String wanted = prefs.getString("wMode", "ap");
  if (wanted != "sta") wanted = "ap";
  bool hasPass = prefs.getString("wPass", "").length() > 0;
  String ip = radioSta ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  String r = String("{\"wanted\":\"") + wanted + "\",\"radio\":\"" + (radioSta ? "sta" : "ap") +
             "\",\"fail\":" + (staFailed ? "true" : "false") + ",\"ssid\":\"" + jsonEsc(ssid) +
             "\",\"hasPass\":" + (hasPass ? "true" : "false") + ",\"ip\":\"" + ip +
             "\",\"host\":\"" HOST_NAME "\",\"tx\":" + String((int)txLevel) +
             ",\"bo\":" + String((unsigned long)boCount) + ",\"rst\":\"" + rstName + "\"}";
  sendJson(200, r.c_str());
}

static void applyTx() {
  static const wifi_power_t LV[3] = {WIFI_POWER_19_5dBm, WIFI_POWER_13dBm, WIFI_POWER_8_5dBm};
  WiFi.setTxPower(LV[txLevel > 2 ? 1 : txLevel]);
}

static void handlePower() {
  if (server.hasArg("tx")) {
    int v = server.arg("tx").toInt();
    if (v < 0 || v > 2) { sendJson(400, "{\"ok\":false,\"err\":\"tx\"}"); return; }
    txLevel = (uint8_t)v;
    prefs.putUChar("wTx", txLevel);
    applyTx();
  }
  if (server.hasArg("clear")) { boCount = 0; prefs.putUInt("bo", 0); }
  handleNetInfo();
}

static void handleNetPost() {
  if (!server.hasArg("mode")) { sendJson(400, "{\"ok\":false,\"err\":\"mode\"}"); return; }
  String mode = server.arg("mode");
  if (mode != "sta" && mode != "ap") { sendJson(400, "{\"ok\":false,\"err\":\"mode\"}"); return; }
  LOCK();
  bool running = eng.running();
  UNLOCK();
  if (running || otaBusy || updPhase) { sendJson(409, "{\"ok\":false,\"err\":\"busy\"}"); return; }
  if (server.hasArg("ssid")) {
    String v = server.arg("ssid");
    if (v.length() > 32) { sendJson(400, "{\"ok\":false,\"err\":\"ssid\"}"); return; }
    prefs.putString("wSsid", v);
  }
  if (server.hasArg("pass")) {
    String v = server.arg("pass");
    if (v.length() > 63) { sendJson(400, "{\"ok\":false,\"err\":\"pass\"}"); return; }
    prefs.putString("wPass", v);
  }
  if (mode == "sta" && prefs.getString("wSsid", "").length() == 0) { sendJson(400, "{\"ok\":false,\"err\":\"ssid\"}"); return; }
  prefs.putString("wMode", mode);
  sendJson(200, "{\"ok\":true}");
  rebootAtMs = millis() + 600;
  if (!rebootAtMs) rebootAtMs = 1;
}

static void handleNotFound() {
  if (radioSta) { server.send(404, "text/plain", "not found"); return; }
  server.sendHeader("Location", "http://192.168.4.1/");
  server.send(302, "text/plain", "");
}

static bool startStation() {
  String ssid = prefs.getString("wSsid", "");
  String pass = prefs.getString("wPass", "");
  if (!ssid.length()) return false;
  joinPhase = 1;
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOST_NAME);
  WiFi.setAutoReconnect(true);
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  applyTx();
  for (int attempt = 0; attempt < 2 && WiFi.status() != WL_CONNECTED; attempt++) {
    if (attempt) { WiFi.disconnect(false, false); delay(300); }
    WiFi.begin(ssid.c_str(), pass.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (uint32_t)(millis() - t0) < STA_JOIN_MS) delay(100);
  }
  joinPhase = 0;
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("home Wi-Fi join failed for \"%s\"\n", ssid.c_str());
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    return false;
  }
  esp_wifi_set_ps(WIFI_PS_NONE);
  Serial.printf("home Wi-Fi \"%s\"  %d dBm  http://%s  http://%s.local\n", ssid.c_str(), (int)WiFi.RSSI(), WiFi.localIP().toString().c_str(), HOST_NAME);
  return true;
}

static void staWatch() {
  if (!radioSta || otaBusy || updPhase) return;
  static uint32_t lastChk = 0, downSince = 0;
  uint32_t now = millis();
  if ((uint32_t)(now - lastChk) < 2000) return;
  lastChk = now;
  if (WiFi.status() == WL_CONNECTED) { downSince = 0; return; }
  if (!downSince) downSince = now ? now : 1;
  else if ((uint32_t)(now - downSince) > 8000) { WiFi.reconnect(); downSince = now; }
}

static void mdnsPoke() {
  static uint32_t n = 0;
  char v[12];
  snprintf(v, sizeof v, "%lu", (unsigned long)++n);
  const char* val = v;
  MDNS.addServiceTxt("http", "tcp", "n", val);
}

static void startAp() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAPsetHostname(HOST_NAME);
  WiFi.softAP(AP_SSID, nullptr, AP_CHANNEL, 0, AP_MAX_CLIENTS);
  applyTx();
  esp_wifi_set_ps(WIFI_PS_NONE);
  dns.start(53, "*", IPAddress(192, 168, 4, 1));
  dnsOn = true;
}

// ---- safety: runs every loop pass -----------------------------------------------------------------------------------
static void tripTo(mg::TripCode code) {
  coilOff();
  LOCK();
  eng.tripNow(esp_timer_get_time(), code);
  UNLOCK();
  drvOn = false;
  prefs.putUChar("trip", (uint8_t)code);
  Serial.printf("TRIP %d: coil off\n", (int)code);
}

static void safetyAndDrive() {
  static uint32_t safeAt = 0;
  int64_t now = esp_timer_get_time();
  uint32_t ms = millis();

  // apply what the engine wants the coil to do
  float hz = 0, pw = 0;
  int stageIdx = -1;
  double stageT = 0;
  bool on;
  LOCK();
  on = eng.drive(now, &hz, &pw, &stageIdx, &stageT);
  if (eng.state == mg::ST_RUN && eng.total > 0 && eng.elapsed(now) >= eng.total) {
    eng.finish();
    on = false;
  }
  UNLOCK();
  if (cfg.maxHz > 0 && hz > cfg.maxHz) hz = cfg.maxHz;
  if (cfg.maxPw > 0 && pw > cfg.maxPw) pw = cfg.maxPw;
  if (on != drvOn || (on && (hz != drvHz || pw != drvPw))) {
    if (on) coilSet(hz, pw);
    else coilOff();
    drvOn = on;
    drvHz = hz;
    drvPw = pw;
  }

  // stuck-pin watchdog: the gate high for longer than one on-time plus margin means the ISR died
  if (coilLevel() == 1 && drvOn && coilPeriodUs() > 0) {
    if (!stuckSince) stuckSince = ms ? ms : 1;
    else if ((uint32_t)(ms - stuckSince) > coilOnUs() / 1000 + 80) { stuckSince = 0; tripTo(mg::TR_STUCK); return; }
  } else {
    stuckSince = 0;
  }

  if ((uint32_t)(ms - safeAt) < 250) return;
  safeAt = ms;
  senseCopy(snapCache);
  if (!cfg.safe.on) return;
  LOCK();
  bool running = eng.state == mg::ST_RUN;
  UNLOCK();
  if (!running) return;
  const SenseSnap& s = snapCache;
  if (cfg.safe.needProbe && (!s.okC || !s.okF)) { tripTo(mg::TR_PROBE); return; }
  if (s.okC && s.tC > cfg.safe.coilMaxF) { tripTo(mg::TR_COIL_TEMP); return; }
  if (s.okF && s.tF > cfg.safe.fetMaxF) { tripTo(mg::TR_FET_TEMP); return; }
  if (s.okC && s.dtdtC > cfg.safe.coilRateF) { tripTo(mg::TR_COIL_RATE); return; }
  if (s.okF && s.dtdtF > cfg.safe.fetRateF) { tripTo(mg::TR_FET_RATE); return; }
}

// ---- setup / loop -----------------------------------------------------------------------------------------------------
static const char* resetName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_SW: return "restart";
    case ESP_RST_PANIC: return "crash";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT: return "watchdog";
    case ESP_RST_DEEPSLEEP: return "wake";
    case ESP_RST_EXT: return "button";
    default: return "other";
  }
}

static void onGotIp(arduino_event_id_t) { gotIpFlag = true; }

void setup() {
  Serial.begin(115200);
  delay(150);
  engMtx = xSemaphoreCreateMutex();

  prefs.begin("magneto", false);
  txLevel = prefs.getUChar("wTx", 1);
  if (txLevel > 2) txLevel = 1;
  esp_reset_reason_t rr = esp_reset_reason();
  rstName = resetName(rr);
  boCount = prefs.getUInt("bo", 0);
  if (rr == ESP_RST_BROWNOUT) { boCount++; prefs.putUInt("bo", boCount); }
  fsOk = LittleFS.begin(true);
  cfgLoad();
  applyHardware();

  // a latched trip survives the reboot; the page explains and offers the reset
  uint8_t latched = prefs.getUChar("trip", 0);
  if (latched && cfg.safe.on && cfg.safe.needReset) {
    eng.state = mg::ST_TRIP;
    eng.trip = (mg::TripCode)latched;
  } else if (latched) {
    prefs.putUChar("trip", 0);
  }

  xTaskCreatePinnedToCore(ledTask, "led", 6144, nullptr, 5, &ledTaskHandle, 1);

  WiFi.persistent(false);
  WiFi.onEvent(onGotIp, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  String wanted = prefs.getString("wMode", "ap");
  if (wanted == "sta") {
    radioSta = startStation();
    staFailed = !radioSta;
  }
  if (!radioSta) startAp();
  else if (MDNS.begin(HOST_NAME)) MDNS.addService("http", "tcp", 80);
  mdnsPokeAt = millis() + MDNS_POKE_MS;

  server.on("/", HTTP_GET, handleRoot);
  server.on("/app", HTTP_GET, handleRoot);
  server.on("/magneto.mobileconfig", HTTP_GET, handleCertProfile);
  server.on("/api/state", HTTP_GET, sendState);
  server.on("/api/secure", HTTP_GET, []() {
    uint32_t o, r;
    tlsStats(&o, &r);
    char b[120];
    snprintf(b, sizeof b, "{\"running\":%s,\"opened\":%u,\"refused\":%u,\"v\":\"%s\"}", tlsRunning() ? "true" : "false", (unsigned)o, (unsigned)r, FW_VERSION);
    server.send(200, "application/json", b);
  });
  server.on("/api/net", HTTP_GET, handleNetInfo);
  server.on("/api/net", HTTP_POST, handleNetPost);
  server.on("/api/power", HTTP_POST, handlePower);
  server.on("/api/update", HTTP_GET, handleUpdInfo);
  server.on("/api/update/save", HTTP_POST, handleUpdSave);
  server.on("/api/update/start", HTTP_POST, handleUpdStart);
  server.on("/api/presets", HTTP_GET, handlePresetsGet);
  server.on("/api/presets", HTTP_POST, handlePresetsPost);
  server.on("/api/cmd", HTTP_POST, handleCmd);
  server.on("/api/config", HTTP_GET, handleConfigGet);
  server.on("/api/config", HTTP_POST, handleConfigPost);
  server.on("/api/wave", HTTP_GET, handleWave);
  server.on("/api/spec", HTTP_GET, handleSpec);
  server.on("/api/fold", HTTP_GET, handleFold);
  server.on("/api/reboot", HTTP_POST, handleReboot);
  server.on("/update", HTTP_POST, handleUpdateDone, handleUpdateData);
  server.onNotFound(handleNotFound);
  server.begin();
  tlsRoutes(nullptr, 0);
  if (fsOk) tlsPrepare();          // runs on its own task; loop() starts the socket when it lands

  ArduinoOTA.setHostname(HOST_NAME);
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.onStart([]() { otaBusy = true; coilOff(); senseHold(true); Serial.println("OTA start"); });
  ArduinoOTA.onEnd([]() { Serial.println("OTA end"); });
  ArduinoOTA.onError([](ota_error_t e) { otaBusy = false; senseHold(false); Serial.printf("OTA error %u\n", (unsigned)e); });
  ArduinoOTA.begin();

  Serial.printf("Magneto %s  reset: %s  brown-outs: %lu  free heap %lu  psram %lu\n", FW_VERSION, rstName,
                (unsigned long)boCount, (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getFreePsram());
  if (radioSta) Serial.printf("STA http://%s/app  http://%s.local/app  wss://%s.local/ws  OTA port %d\n", WiFi.localIP().toString().c_str(), HOST_NAME, HOST_NAME, OTA_PORT);
  else Serial.printf("AP \"%s\" (open)  http://%s  OTA port %d%s\n", AP_SSID, WiFi.softAPIP().toString().c_str(), OTA_PORT, staFailed ? "  (home Wi-Fi join failed)" : "");
}

void loop() {
  if (dnsOn) dns.processNextRequest();
  server.serve();
  ArduinoOTA.handle();
  staWatch();
  safetyAndDrive();
  if (radioSta && (gotIpFlag || (int32_t)(millis() - mdnsPokeAt) >= 0)) {
    gotIpFlag = false;
    mdnsPokeAt = millis() + MDNS_POKE_MS;
    mdnsPoke();
  }
  if (!tlsRunning() && tlsCertReady() && !otaBusy && !updPhase) {
    if (tlsStart() && radioSta) MDNS.addService("wss", "tcp", 443);
  }
  if (updStartAt && (int32_t)(millis() - updStartAt) >= 0) { updStartAt = 0; runWebUpdate(); }
  if (cfgDirty && millis() - cfgDirtyAt > 2000) cfgSaveNow();
  if (rebootAtMs && (int32_t)(millis() - rebootAtMs) >= 0) {
    if (cfgDirty) cfgSaveNow();
    coilOff();
    delay(100);
    ESP.restart();
  }
  delay(2);
}
