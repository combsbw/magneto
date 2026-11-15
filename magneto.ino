/*
 * Magneto - PEMF coil controller and monitor, for the ESP32-S3
 *
 *   Board   : esp32:esp32:esp32s3 (arduino-esp32 core 2.0.17), USB CDC On Boot = enabled
 *   Wi-Fi   : open soft AP "Magneto" -> http://192.168.4.1 (captive portal opens the page), or after
 *             Settings > Connection a local network -> http://magneto.local. Any 2.4 GHz source works,
 *             including a phone hotspot and an open guest network. If the saved network is not there
 *             the board hosts the access point instead - at boot, and again if the link later dies.
 *   Recovery: holding a magnet on either hall sensor for a few seconds while the board is on a local
 *             network reboots it into the access point, so it is never lost behind a network that
 *             moved or changed its password (Settings > Safety has the threshold and the hold time).
 *   Secure  : wss://magneto.local/ws or wss://192.168.4.1/ws for the installed web app (GitHub Pages),
 *             once the phone trusts the board's own certificate (tlshttps.cpp, /magneto.mobileconfig).
 *             The status poll and every command are answered inside the socket task, so a button
 *             press does not wait for a loopback trip through the port 80 server.
 *   Update  : ArduinoOTA on 3232, HTTP upload at /update, and the one-button online update that joins
 *             the local network (or a separate update network) and pulls the newest build from the
 *             GitHub "latest" release.
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
#include "netprobe.h"
#include "ghupdate.h"
#include "tlshttps.h"

#define FW_VERSION "0.2.0"
#define STA_JOIN_MS 12000
#define STA_LOST_MS 45000UL             // local network gone this long -> bring the access point up instead
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
static const char* apWhy = "";           // why this boot is an access point, for the page to explain
static bool apPortal = false;            // true = answer every probe with the page (captive pop-up)
static uint32_t magHoldMs = 0;           // how long a steady field has been on a hall sensor
static float magPeak = 0;                // |B| seen while that field has been held, microtesla

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

// Defined with the rest of the safety work, below; the status JSON reports it.
static bool magArmed(const SenseSnap& s);

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
    "\"tc\":%.1f,\"tf\":%.1f,\"dc\":%.1f,\"dfr\":%.1f,\"okc\":%d,\"okf\":%d,"
    "\"pwmax\":%.0f,\"mga\":%d,\"mgh\":%.1f,\"hwpw\":%lu,\"hwper\":%lu}",
    FW_VERSION, mg::stateName(eng.state), t, eng.total, stageIdx, eng.nWin,
    hz, pw, mg::dutyOf(hz, pw) * 100.0, (int)eng.trip, (unsigned long)cfgRev, otaBusy ? 1 : 0,
    (unsigned long)(millis() / 1000), radioSta ? "sta" : "ap", staFailed ? 1 : 0,
    rstName, (unsigned long)boCount, (unsigned long)ESP.getFreeHeap(), radioSta ? (int)WiFi.RSSI() : 0,
    sensePsram() ? 1 : 0, senseFftPoints(),
    s.b0, s.b1, s.db, s.bMax0, s.bMax1, s.dbMax,
    s.bAvg0, s.bAvg1, s.bRms0, s.statSec, s.pkHz, s.pkAmp, s.pkSnr,
    s.zeroed0 ? 1 : 0, s.zeroed1 ? 1 : 0, s.clip ? 1 : 0, s.fs,
    isnan(s.tC) ? 0.0 : s.tC, isnan(s.tF) ? 0.0 : s.tF, s.dtdtC, s.dtdtF, s.okC ? 1 : 0, s.okF ? 1 : 0,
    mg::pwFor(hz, mg::PW_MAX_US, cfg.maxPw), magArmed(s) ? 1 : 0, magHoldMs / 1000.f,
    // Ground truth, read back off the pulse timer itself rather than from the plan: the on-time and
    // period the ISR is really producing. The page checks the slider against these, so "the slider
    // says 400 us" is backed by the hardware and not by the page agreeing with itself.
    (unsigned long)(coilRunning() ? coilOnUs() : 0), (unsigned long)(coilRunning() ? coilPeriodUs() : 0));
  UNLOCK();
  return (n > 0 && (size_t)n < cap) ? n : -1;
}

static void sendState() {
  char buf[1320];
  lastApiMs = millis();
  if (stateJson(buf, sizeof buf) < 0) { sendJson(500, "{\"ok\":false}"); return; }
  sendJson(200, buf);
}

// ---- commands -------------------------------------------------------------------------------------------------------
// One implementation behind two front doors: the port 80 handler (plain http, and anything the
// secure socket relays) and the fast path inside the TLS task, which answers a button press without
// a loopback trip through the web server. Plain arguments, no struct: the Arduino build inserts its
// own prototypes at the top of this file, where a type declared further down would not exist yet.
//
// Returns nullptr when the command was accepted, otherwise a short error code for the page.
static const char* runCmd(const char* cmd, float argHz, float argPw, int argCh, const char* argW) {
  lastApiMs = millis();
  int64_t now = esp_timer_get_time();
  const char* err = nullptr;
  SenseSnap s;
  senseCopy(s);
  LOCK();
  if (!strcmp(cmd, "start")) {
    if (eng.state == mg::ST_TRIP) err = "trip";
    else if (cfg.safe.on && cfg.safe.needProbe && (!s.okC || !s.okF)) err = "probe";
    else if (!eng.start(now)) err = "state";
  } else if (!strcmp(cmd, "pause")) {
    if (!eng.pause(now)) err = "state";
  } else if (!strcmp(cmd, "stop")) {
    if (eng.state == mg::ST_TRIP) err = "trip";
    else eng.stop();
  } else if (!strcmp(cmd, "resettime")) {
    eng.resetTime(now);
  } else if (!strcmp(cmd, "tune")) {
    float hz = argHz;
    if (cfg.maxHz > 0 && hz > cfg.maxHz) hz = cfg.maxHz;
    hz = mg::snapHz(hz);
    // One call decides the pulse width everywhere: the ceiling first, then the period, so a tune
    // can never ask for more than the slider is allowed to show.
    float pw = mg::pwFor(hz, argPw, cfg.maxPw);
    if (!eng.tune(now, hz, pw)) err = "state";
  } else if (!strcmp(cmd, "tripreset")) {
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
  } else if (!strcmp(cmd, "zero")) {
    senseZero(argCh);
  } else if (!strcmp(cmd, "zeroclear")) {
    senseZeroClear(argCh);
  } else if (!strcmp(cmd, "clear")) {
    senseClearStats(!strcmp(argW, "max") ? SENSE_CLR_MAX : (!strcmp(argW, "avg") ? SENSE_CLR_AVG : SENSE_CLR_ALL));
  } else {
    err = "cmd";
  }
  UNLOCK();
  return err;
}

static void handleCmd() {
  String c = server.arg("c"), w = server.arg("w");
  const char* err = runCmd(c.c_str(), server.arg("hz").toFloat(), server.arg("pw").toFloat(),
                           server.arg("ch").toInt(), w.c_str());
  if (err) {
    char b[64];
    snprintf(b, sizeof b, "{\"ok\":false,\"err\":\"%s\"}", err);
    sendJson(!strcmp(err, "trip") || !strcmp(err, "probe") || !strcmp(err, "cool") || !strcmp(err, "state") ? 409 : 400, b);
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

// The network the online update should join: the separate update network when one is configured and
// switched on, otherwise the local Wi-Fi the board already knows. One place decides, so the page and
// the update itself can never disagree about which credentials are in play.
static bool updUsesAlt() {
  return prefs.getUChar("wUAlt", 0) != 0 && prefs.getString("wUSsid", "").length() > 0;
}
static String updSsid() { return updUsesAlt() ? prefs.getString("wUSsid", "") : prefs.getString("wSsid", ""); }
static String updPass() { return updUsesAlt() ? prefs.getString("wUPass", "") : prefs.getString("wPass", ""); }

static void handleUpdInfo() {
  String repo = prefs.getString("wRepo", GH_REPO_DEFAULT);
  String loc = prefs.getString("wSsid", ""), alt = prefs.getString("wUSsid", "");
  bool hasTok = prefs.getString("wTok", "").length() > 0;
  String r = String("{\"ssid\":\"") + jsonEsc(updSsid()) + "\",\"hasPass\":" + (updPass().length() ? "true" : "false") +
             ",\"local\":\"" + jsonEsc(loc) + "\",\"localHasPass\":" + (prefs.getString("wPass", "").length() ? "true" : "false") +
             ",\"useAlt\":" + (prefs.getUChar("wUAlt", 0) ? "true" : "false") +
             ",\"altSsid\":\"" + jsonEsc(alt) + "\",\"altHasPass\":" + (prefs.getString("wUPass", "").length() ? "true" : "false") +
             ",\"repo\":\"" + jsonEsc(repo) +
             "\",\"hasToken\":" + (hasTok ? "true" : "false") + ",\"last\":\"" + jsonEsc(prefs.getString("wRes", "")) +
             "\",\"from\":\"" + jsonEsc(prefs.getString("wFrom", "")) + "\",\"ver\":\"" FW_VERSION "\"}";
  sendJson(200, r.c_str());
}

// Fields (any subset): useAlt = 0/1 ("use a separate update network"), altSsid, altPass, repo, token.
// The local Wi-Fi itself is only ever set through /api/net, so the two forms cannot overwrite each
// other the way they used to.
static void handleUpdSave() {
  if (server.hasArg("useAlt")) prefs.putUChar("wUAlt", server.arg("useAlt").toInt() ? 1 : 0);
  if (server.hasArg("altSsid")) {
    String v = server.arg("altSsid");
    v.trim();
    if (v.length() > 32) { sendJson(400, "{\"ok\":false,\"err\":\"ssid\"}"); return; }
    prefs.putString("wUSsid", v);
  }
  if (server.hasArg("altPass")) {
    String v = server.arg("altPass");
    if (v.length() > 63) { sendJson(400, "{\"ok\":false,\"err\":\"pass\"}"); return; }
    prefs.putString("wUPass", v);
  }
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
  if (updSsid().length() == 0) { sendJson(400, "{\"ok\":false,\"err\":\"ssid\"}"); return; }
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
  String ssid = updSsid(), pass = updPass();
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
  WiFi.setMinSecurity(WIFI_AUTH_OPEN);          // an open guest network or a hotspot on WPA is still a network
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
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
             ",\"why\":\"" + apWhy + "\",\"portal\":" + (apPortal ? "true" : "false") +
             ",\"updAlt\":" + (prefs.getUChar("wUAlt", 0) ? "true" : "false") +
             ",\"updSsid\":\"" + jsonEsc(updSsid()) + "\",\"ap\":\"" AP_SSID "\"" +
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

// mode = sta | ap, plus optional ssid / pass. The radio only has to restart when the mode actually
// changes, so saving a corrected password while the device sits on its access point - or mid
// treatment - costs nothing. reboot=1 forces it (the page's "Save and reboot").
static void handleNetPost() {
  if (!server.hasArg("mode")) { sendJson(400, "{\"ok\":false,\"err\":\"mode\"}"); return; }
  String mode = server.arg("mode");
  if (mode != "sta" && mode != "ap") { sendJson(400, "{\"ok\":false,\"err\":\"mode\"}"); return; }
  String was = prefs.getString("wMode", "ap");
  if (was != "sta") was = "ap";
  bool restart = mode != was || server.arg("reboot") == "1";
  LOCK();
  bool running = eng.running();
  UNLOCK();
  if (restart && (running || otaBusy || updPhase)) { sendJson(409, "{\"ok\":false,\"err\":\"busy\"}"); return; }
  if (server.hasArg("ssid")) {
    String v = server.arg("ssid");
    v.trim();                              // a trailing space from a phone keyboard is a silent join failure
    if (v.length() > 32) { sendJson(400, "{\"ok\":false,\"err\":\"ssid\"}"); return; }
    prefs.putString("wSsid", v);
  }
  if (server.hasArg("pass")) {
    String v = server.arg("pass");
    if (v.length() > 63) { sendJson(400, "{\"ok\":false,\"err\":\"pass\"}"); return; }
    prefs.putString("wPass", v);
  }
  if (server.hasArg("portal")) {                 // takes effect at once; no reboot needed
    apPortal = server.arg("portal").toInt() != 0;
    prefs.putUChar("wPortal", apPortal ? 1 : 0);
  }
  if (mode == "sta" && prefs.getString("wSsid", "").length() == 0) { sendJson(400, "{\"ok\":false,\"err\":\"ssid\"}"); return; }
  prefs.putString("wMode", mode);
  sendJson(200, restart ? "{\"ok\":true,\"reboot\":true}" : "{\"ok\":true,\"reboot\":false}");
  if (!restart) return;
  rebootAtMs = millis() + 600;
  if (!rebootAtMs) rebootAtMs = 1;
}

// Phones probe a network for internet the moment they join it, over plain http, at a name our
// wildcard DNS points straight back at us. Answering those probes with a redirect to the page is
// what tells the phone "captive portal, no internet" - and iOS then treats the network as unusable:
// it shows "No Internet Connection" and, on a phone with a cellular connection to fall back to,
// quietly leaves within seconds. An iPad with no cellular has nowhere to go, so it hangs on longer.
// Answering the probe the way a working network would keeps the phone on the access point, which is
// what matters here: the page is reached by address and the installed app by the secure socket,
// neither of which needs the portal pop-up.
//
// apProbe = 1 restores the old behaviour for anyone who wants the pop-up instead.
static void handleNotFound() {
  if (radioSta) { server.send(404, "text/plain", "not found"); return; }
  if (!apPortal) {
    String uri = server.uri(), host = server.hostHeader();
    uri.toLowerCase();
    host.toLowerCase();
    mg::ProbeKind k = mg::probeKind(uri.c_str(), host.c_str());
    if (k != mg::PROBE_NONE) {
      server.sendHeader("Cache-Control", "no-store");
      server.send(mg::probeCode(k), mg::probeType(k), mg::probeBody(k));
      return;
    }
  }
  // anything else typed into a browser still lands on the page
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
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);          // look at every access point with this name ...
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);      // ... and join the strongest one
  // The core refuses anything weaker than WPA2 by default, which rules out open guest networks and
  // the handful of hotspots that still offer WPA only. A local network is a local network.
  WiFi.setMinSecurity(WIFI_AUTH_OPEN);
  applyTx();
  for (int attempt = 0; attempt < 2 && WiFi.status() != WL_CONNECTED; attempt++) {
    if (attempt) { WiFi.disconnect(false, false); delay(300); }
    WiFi.begin(ssid.c_str(), pass.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (uint32_t)(millis() - t0) < STA_JOIN_MS) delay(100);
  }
  joinPhase = 0;
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("local Wi-Fi join failed for \"%s\"\n", ssid.c_str());
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);
    return false;
  }
  esp_wifi_set_ps(WIFI_PS_NONE);
  Serial.printf("local Wi-Fi \"%s\"  %d dBm  http://%s  http://%s.local\n", ssid.c_str(), (int)WiFi.RSSI(), WiFi.localIP().toString().c_str(), HOST_NAME);
  return true;
}

static void startAp();

// Leave the local network and host the access point from here on, without rebooting - a treatment
// may be running and must not be interrupted. The page comes back at http://192.168.4.1.
static void toAp(const char* why) {
  Serial.printf("local Wi-Fi lost (%s): starting the access point\n", why);
  tlsStop();
  MDNS.end();
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF);
  delay(100);
  radioSta = false;
  staFailed = true;
  apWhy = why;
  startAp();
  gotIpFlag = false;
}

// On a local network: ask the driver to reconnect when the link drops, and if the network simply is
// not there any more - moved, renamed, router off, hotspot put away - stop waiting and host the
// access point, so the board is reachable again without anyone having to find a USB cable.
static void staWatch() {
  if (!radioSta || otaBusy || updPhase) return;
  static uint32_t lastChk = 0, downSince = 0, lastTry = 0;
  uint32_t now = millis();
  if ((uint32_t)(now - lastChk) < 2000) return;
  lastChk = now;
  if (WiFi.status() == WL_CONNECTED) { downSince = 0; return; }
  if (!downSince) { downSince = now ? now : 1; lastTry = now; return; }
  if ((uint32_t)(now - downSince) > STA_LOST_MS) { downSince = 0; toAp("gone"); return; }
  if ((uint32_t)(now - lastTry) > 8000) { WiFi.reconnect(); lastTry = now; }
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

// Is the magnet escape hatch available right now? Only on a local network (on its own access point
// there is nothing to recover from), and only on a sensor with a stored zero: an un-zeroed 49E can
// sit a few hundred gauss off on its quiescent output alone, which would look exactly like a magnet.
// The zero lives in NVS, so this is a one-time step and the page says when it is missing.
static bool magArmed(const SenseSnap& s) {
  return radioSta && cfg.safe.magAp && (s.zeroed0 || s.zeroed1);
}

// Magnet escape hatch. A steady field held on a zeroed hall sensor past the configured threshold and
// hold time, while the board is on a local network, saves "access point" as the wanted mode and
// restarts. The field has to stay above the threshold the whole time: a pulse train from the coil
// itself drops back towards zero between pulses, so running a treatment does not trip it, and the
// coil is parked before the restart either way.
static void magWatch(const SenseSnap& s, uint32_t ms) {
  static uint32_t lastMs = 0;
  uint32_t dt = lastMs ? ms - lastMs : 0;
  lastMs = ms;
  if (!magArmed(s) || otaBusy || updPhase || rebootAtMs) { magHoldMs = 0; magPeak = 0; return; }
  float b = 0;                                    // only channels with a stored zero are believed
  if (s.zeroed0 && fabsf(s.b0) > b) b = fabsf(s.b0);
  if (s.zeroed1 && fabsf(s.b1) > b) b = fabsf(s.b1);
  if (b < cfg.safe.magUt) { magHoldMs = 0; magPeak = 0; return; }
  if (b > magPeak) magPeak = b;
  if (dt > 1000) dt = 1000;                       // a long gap (a stalled loop) must not count as held time
  magHoldMs += dt;
  if (magHoldMs < (uint32_t)(cfg.safe.magSec * 1000.f)) return;
  Serial.printf("magnet held %.0f uT for %.1f s: rebooting into the access point\n", magPeak, cfg.safe.magSec);
  coilOff();
  LOCK();
  if (eng.state == mg::ST_RUN) eng.pause(esp_timer_get_time());
  UNLOCK();
  prefs.putString("wMode", "ap");
  prefs.putUChar("magBoot", 1);
  if (cfgDirty) cfgSaveNow();
  delay(150);
  ESP.restart();
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
  // The last word on what the coil is driven with: the frequency ceiling, then the pulse-width
  // ceiling, then the period. Whatever the plan or a stale tune asked for, this is what happens, and
  // it is the same mg::pwFor() the page uses to lay out the slider.
  if (cfg.maxHz > 0 && hz > cfg.maxHz) hz = cfg.maxHz;
  if (on) pw = mg::pwFor(hz, pw, cfg.maxPw);
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
  magWatch(snapCache, ms);
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

// Port 443 carries only wss://<board>/ws. The state poll and the commands - the calls that decide
// how quickly the page feels - are answered inside the TLS task. Everything else is repeated to the
// handlers on port 80, which costs a loopback TCP connection and a few passes of loop().
static int jsonState(const char*, char* buf, size_t cap) { lastApiMs = millis(); return stateJson(buf, cap); }

static int jsonCmd(const char* qs, char* buf, size_t cap) {
  char c[20], hz[16], pw[16], ch[8], w[8];
  mg::qsParam(qs, "c", c, sizeof c);
  mg::qsParam(qs, "hz", hz, sizeof hz);
  mg::qsParam(qs, "pw", pw, sizeof pw);
  mg::qsParam(qs, "ch", ch, sizeof ch);
  mg::qsParam(qs, "w", w, sizeof w);
  const char* err = runCmd(c, (float)atof(hz), (float)atof(pw), atoi(ch), w);
  if (err) {                        /* the page reads the error out of the body, so 200 with ok:false */
    int n = snprintf(buf, cap, "{\"ok\":false,\"err\":\"%s\"}", err);
    return (n > 0 && (size_t)n < cap) ? n : -1;
  }
  return stateJson(buf, cap);
}

static const TlsJson TLS_JSON[] = {{"/api/state", 0, jsonState}, {"/api/cmd", 1, jsonCmd}};

void setup() {
  setCpuFrequencyMhz(240);              // the DMA ADC, the FFT and the 1 us pulse ISR all want the full clock
  Serial.begin(115200);
  delay(150);
  engMtx = xSemaphoreCreateMutex();

  prefs.begin("magneto", false);
  txLevel = prefs.getUChar("wTx", 1);
  if (txLevel > 2) txLevel = 1;
  apPortal = prefs.getUChar("wPortal", 0) != 0;
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
  if (prefs.getUChar("magBoot", 0)) {    // the magnet put it here; the page says so and offers the way back
    prefs.putUChar("magBoot", 0);
    apWhy = "magnet";
  }
  String wanted = prefs.getString("wMode", "ap");
  if (wanted == "sta") {
    radioSta = startStation();
    staFailed = !radioSta;
    if (staFailed && !apWhy[0]) apWhy = "join";
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
  tlsRoutes(TLS_JSON, sizeof TLS_JSON / sizeof TLS_JSON[0]);
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
