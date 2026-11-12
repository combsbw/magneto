// config.h - the Magneto settings: JSON from the page into a checked, clamped Config struct.
//
// The raw JSON is stored as the page sent it (NVS) and handed back on GET /api/config, so the page
// owns the schema; the firmware reads only the fields it acts on. Pure C++ for the host tests.
#pragma once
#include "engine.h"
#include "json.h"

namespace mg {

// Generic ESP32-S3 module (Super Mini / DevKit). GPIO 1-18 and 21 are free and safe; GPIO 1-10
// carry ADC1. Excluded: 0 (boot strap), 19/20 (USB), 26-32 (flash) and higher pins that octal
// PSRAM modules reserve.
inline bool pinAllowed(int p) { return (p >= 1 && p <= 18) || p == 21; }
inline bool pinAdc(int p) { return p >= 1 && p <= 10; }

struct Pins {
  int8_t drive;         // TC4420 input -> MOSFET gate. HIGH = coil on.
  int8_t hallC, hallP;  // 49E outputs: coil center, coil perimeter (ADC1)
  int8_t thC, thF;      // 10k NTC dividers: coil face, MOSFET (ADC1)
  int8_t r, g, b;       // status RGB legs (common anode hardwired to 3V3: LOW = lit)
  int8_t coil;          // coil-rim LED
  bool anode;           // status RGB is common-anode (inverted PWM)
  bool coilLow;         // coil LED is active low
};

struct Safety {
  float coilMaxF;    // coil-face thermistor limit, Fahrenheit internally
  float fetMaxF;
  float coilRateF;   // degF per minute
  float fetRateF;
  bool on;           // master enable; off = thermal shutoff bypassed (the page warns loudly)
  bool needReset;    // a trip must be reset before the next start
  bool needProbe;    // refuse to start when an enabled probe reads open
};

struct LedCfg {
  uint8_t mode;      // 0 fixed color, 1 stage colors (plan drives it), 2 data feedback
  Rgb col;           // fixed color
  uint8_t master;    // 0..100 overall brightness
  uint8_t dataSrc;   // 0 coil temp, 1 fet temp, 2 |B| average, 3 dB/dt
  float dataMin, dataMax;   // feedback range mapped blue -> green -> amber -> red
  uint8_t coilBright;       // coil LED brightness 0..100
};

struct HallCfg {
  float sens5c, sens5p;   // mV per gauss at 5 V supply (49E ~1.4) for center / perimeter
  float vcc;              // sensor supply
  bool ratio;             // ratiometric: sensitivity scales with vcc/5
};

struct ThermCfg {
  float beta;     // NTC beta (3950 typical)
  float r0k;      // resistance at 25 C, kilohm
  float rsk;      // fixed divider resistor, kilohm
  bool top;       // true: NTC between 3V3 and the pin; false: NTC between the pin and GND
};

struct Config {
  Pins pins;
  Plan plan;
  float maxHz, maxPw;      // slider ceilings; 0 = none. Enforced on tune while the coil is on.
  Safety safe;
  LedCfg led;
  HallCfg hall;
  ThermCfg therm;

  void setDefaults() {
    pins = Pins{7, 1, 2, 3, 4, 5, 6, 8, 9, true, false};
    plan.reset();
    plan.n = 1;
    plan.st[0] = Stage{true, 10.f, 100.f, 1200.f, Rgb{80.f, 200.f, 255.f}};
    maxHz = 0; maxPw = 0;
    safe = Safety{110.f, 140.f, 20.f, 20.f, true, true, true};
    led = LedCfg{0, Rgb{80.f, 200.f, 255.f}, 80, 0, 70.f, 110.f, 100};
    hall = HallCfg{1.4f, 1.4f, 3.3f, true};
    therm = ThermCfg{3950.f, 10.f, 10.f, false};
  }
};

// ---- JSON -> Config -------------------------------------------------------------------------------------------------

inline bool hexNib(char c, int* v) {
  if (c >= '0' && c <= '9') { *v = c - '0'; return true; }
  if (c >= 'a' && c <= 'f') { *v = c - 'a' + 10; return true; }
  if (c >= 'A' && c <= 'F') { *v = c - 'A' + 10; return true; }
  return false;
}

// "#rrggbb" -> Rgb. Anything else leaves the color alone.
inline void parseColor(const char* s, Rgb* out) {
  if (!s || s[0] != '#' || strlen(s) != 7) return;
  int v[6];
  for (int i = 0; i < 6; i++) if (!hexNib(s[1 + i], &v[i])) return;
  out->r = (float)(v[0] * 16 + v[1]);
  out->g = (float)(v[2] * 16 + v[3]);
  out->b = (float)(v[4] * 16 + v[5]);
}

struct CfgCtx {
  Config* c;
  // Stages arrive sparse ("stages.2.hz"); collected here then packed.
  Stage raw[STAGE_MAX];
  int rFrom, rTo, rN;
};

inline void cfgLeaf(void* vctx, const char* path, char type, double num, const char* str) {
  CfgCtx* x = (CfgCtx*)vctx;
  Config& c = *x->c;
  float f = (float)num;
  int i = (int)num;

  if (strncmp(path, "pins.", 5) == 0) {
    const char* k = path + 5;
    if (!strcmp(k, "drive")) c.pins.drive = (int8_t)i;
    else if (!strcmp(k, "hallC")) c.pins.hallC = (int8_t)i;
    else if (!strcmp(k, "hallP")) c.pins.hallP = (int8_t)i;
    else if (!strcmp(k, "thC")) c.pins.thC = (int8_t)i;
    else if (!strcmp(k, "thF")) c.pins.thF = (int8_t)i;
    else if (!strcmp(k, "r")) c.pins.r = (int8_t)i;
    else if (!strcmp(k, "g")) c.pins.g = (int8_t)i;
    else if (!strcmp(k, "b")) c.pins.b = (int8_t)i;
    else if (!strcmp(k, "coil")) c.pins.coil = (int8_t)i;
    else if (!strcmp(k, "anode")) c.pins.anode = num != 0;
    else if (!strcmp(k, "coilLow")) c.pins.coilLow = num != 0;
    return;
  }
  if (strncmp(path, "run.stages.", 11) == 0) {
    int si = path[11] - '0';
    const char* k = path + 13;
    if (si < 0 || si >= STAGE_MAX || path[12] != '.') return;
    Stage& s = x->raw[si];
    if (!strcmp(k, "on")) s.on = num != 0;
    else if (!strcmp(k, "hz")) s.hz = snapHz(f);
    else if (!strcmp(k, "pw")) s.pwUs = snapPw(f);
    else if (!strcmp(k, "sec")) s.sec = clampf(f, 0.f, 6.f * 3600.f);
    else if (!strcmp(k, "col") && type == 's') parseColor(str, &s.col);
    return;
  }
  if (strncmp(path, "run.", 4) == 0) {
    const char* k = path + 4;
    if (!strcmp(k, "rFrom")) x->rFrom = i;
    else if (!strcmp(k, "rTo")) x->rTo = i;
    else if (!strcmp(k, "rN")) x->rN = i < 1 ? 1 : (i > 10 ? 10 : i);
    return;
  }
  if (!strcmp(path, "max.hz")) { c.maxHz = f <= 0 ? 0 : snapHz(f); return; }
  if (!strcmp(path, "max.pw")) { c.maxPw = f <= 0 ? 0 : snapPw(f); return; }

  if (strncmp(path, "safe.", 5) == 0) {
    const char* k = path + 5;
    if (!strcmp(k, "coilMax")) c.safe.coilMaxF = clampf(f, 80.f, 180.f);
    else if (!strcmp(k, "fetMax")) c.safe.fetMaxF = clampf(f, 80.f, 220.f);
    else if (!strcmp(k, "coilRate")) c.safe.coilRateF = clampf(f, 1.f, 120.f);
    else if (!strcmp(k, "fetRate")) c.safe.fetRateF = clampf(f, 1.f, 120.f);
    else if (!strcmp(k, "on")) c.safe.on = num != 0;
    else if (!strcmp(k, "needReset")) c.safe.needReset = num != 0;
    else if (!strcmp(k, "needProbe")) c.safe.needProbe = num != 0;
    return;
  }
  if (strncmp(path, "led.", 4) == 0) {
    const char* k = path + 4;
    if (!strcmp(k, "mode")) c.led.mode = (uint8_t)(i < 0 ? 0 : (i > 2 ? 2 : i));
    else if (!strcmp(k, "col") && type == 's') parseColor(str, &c.led.col);
    else if (!strcmp(k, "master")) c.led.master = (uint8_t)clampf(f, 0.f, 100.f);
    else if (!strcmp(k, "dataSrc")) c.led.dataSrc = (uint8_t)(i < 0 ? 0 : (i > 3 ? 3 : i));
    else if (!strcmp(k, "dataMin")) c.led.dataMin = f;
    else if (!strcmp(k, "dataMax")) c.led.dataMax = f;
    else if (!strcmp(k, "coilBright")) c.led.coilBright = (uint8_t)clampf(f, 0.f, 100.f);
    return;
  }
  if (strncmp(path, "hall.", 5) == 0) {
    const char* k = path + 5;
    if (!strcmp(k, "sens5c")) c.hall.sens5c = clampf(f, 0.1f, 20.f);
    else if (!strcmp(k, "sens5p")) c.hall.sens5p = clampf(f, 0.1f, 20.f);
    else if (!strcmp(k, "vcc")) c.hall.vcc = clampf(f, 2.5f, 5.5f);
    else if (!strcmp(k, "ratio")) c.hall.ratio = num != 0;
    return;
  }
  if (strncmp(path, "therm.", 6) == 0) {
    const char* k = path + 6;
    if (!strcmp(k, "beta")) c.therm.beta = clampf(f, 1000.f, 8000.f);
    else if (!strcmp(k, "r0")) c.therm.r0k = clampf(f, 0.5f, 500.f);
    else if (!strcmp(k, "rs")) c.therm.rsk = clampf(f, 0.5f, 500.f);
    else if (!strcmp(k, "top")) c.therm.top = num != 0;
    return;
  }
}

// Every assigned pin, in a fixed order, for the duplicate check.
inline int cfgPinList(const Pins& p, int8_t* out) {
  out[0] = p.drive; out[1] = p.hallC; out[2] = p.hallP; out[3] = p.thC; out[4] = p.thF;
  out[5] = p.r; out[6] = p.g; out[7] = p.b; out[8] = p.coil;
  return 9;
}

inline bool pinsValid(const Pins& p) {
  int8_t v[9];
  int n = cfgPinList(p, v);
  for (int i = 0; i < n; i++) {
    if (!pinAllowed(v[i])) return false;
    for (int j = i + 1; j < n; j++) if (v[i] == v[j]) return false;
  }
  return pinAdc(p.hallC) && pinAdc(p.hallP) && pinAdc(p.thC) && pinAdc(p.thF);
}

// Parse the page's settings JSON. Returns false only for unparseable JSON. Bad pins fall back to the
// defaults (polarity kept) and *pinsOk reports it, so one bad select never bricks the board.
inline bool cfgFromJson(const char* s, size_t n, Config& out, bool* pinsOk) {
  out.setDefaults();
  CfgCtx x;
  x.c = &out;
  x.rFrom = -1; x.rTo = -1; x.rN = 1;
  for (int i = 0; i < STAGE_MAX; i++) x.raw[i] = Stage{false, 10.f, 100.f, 0.f, Rgb{80.f, 200.f, 255.f}};
  if (!jsonWalk(s, n, cfgLeaf, &x)) return false;

  // Pack enabled stages; remap the repeat span from raw indices to packed ones.
  Plan p;
  p.reset();
  int map[STAGE_MAX];
  for (int i = 0; i < STAGE_MAX; i++) map[i] = -1;
  for (int i = 0; i < STAGE_MAX; i++) {
    if (!x.raw[i].on) continue;
    map[i] = p.n;
    p.st[p.n++] = x.raw[i];
  }
  if (p.n == 0) { p.st[0] = Stage{true, 10.f, 100.f, 0.f, out.led.col}; p.n = 1; }
  if (x.rFrom >= 0 && x.rFrom < STAGE_MAX && x.rTo >= x.rFrom && x.rTo < STAGE_MAX && x.rN > 1 &&
      map[x.rFrom] >= 0 && map[x.rTo] >= 0) {
    p.repFrom = map[x.rFrom];
    p.repTo = map[x.rTo];
    p.repN = x.rN;
  }
  out.plan = p;

  // Ceilings apply to every stage too: the plan cannot smuggle a value past them.
  for (int i = 0; i < p.n; i++) {
    if (out.maxHz > 0 && out.plan.st[i].hz > out.maxHz) out.plan.st[i].hz = out.maxHz;
    if (out.maxPw > 0 && out.plan.st[i].pwUs > out.maxPw) out.plan.st[i].pwUs = out.maxPw;
  }

  bool ok = pinsValid(out.pins);
  if (!ok) {
    Config d;
    d.setDefaults();
    bool anode = out.pins.anode, coilLow = out.pins.coilLow;
    out.pins = d.pins;
    out.pins.anode = anode;
    out.pins.coilLow = coilLow;
  }
  if (pinsOk) *pinsOk = ok;
  return true;
}

// Saved setups: the board stores the list the page sends, shape-checked only (valid JSON, top-level array).
inline bool presetsJsonOk(const char* s, size_t n) {
  size_t i = 0;
  while (i < n && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) i++;
  if (i >= n || s[i] != '[') return false;
  struct Noop { static void fn(void*, const char*, char, double, const char*) {} };
  return jsonWalk(s, n, Noop::fn, nullptr);
}

}  // namespace mg
