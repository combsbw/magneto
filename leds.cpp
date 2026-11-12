#include "leds.h"

#include <Arduino.h>
#include "driver/gpio.h"

using mg::Rgb;
using mg::clampf;
using mg::mixRgb;

static const int PWM_FREQ = 5000;
static const int PWM_BITS = 12;
static const int PWM_MAX = (1 << PWM_BITS) - 1;

static int8_t gRgbPin[3] = {-1, -1, -1};
static int8_t gCoilPin = -1;
static bool gAnode = true, gCoilLow = false;
static bool gLive = false;

void ledsApply(const mg::Pins& p) {
  if (gLive) {
    for (int i = 0; i < 3; i++) { ledcDetachPin(gRgbPin[i]); pinMode(gRgbPin[i], INPUT); }
    ledcDetachPin(gCoilPin);
    pinMode(gCoilPin, INPUT);
  }
  gRgbPin[0] = p.r; gRgbPin[1] = p.g; gRgbPin[2] = p.b;
  gCoilPin = p.coil;
  gAnode = p.anode;
  gCoilLow = p.coilLow;
  for (int i = 0; i < 3; i++) {
    ledcSetup(i, PWM_FREQ, PWM_BITS);
    ledcAttachPin(gRgbPin[i], i);
    ledcWrite(i, gAnode ? PWM_MAX : 0);
  }
  ledcSetup(3, PWM_FREQ, PWM_BITS);
  ledcAttachPin(gCoilPin, 3);
  ledcWrite(3, gCoilLow ? PWM_MAX : 0);
  gLive = true;
}

// color 0..255 and brightness 0..1, both perceptual; gamma turns them into PWM duty.
static void rgbWrite(const Rgb& c, float b, float master) {
  if (!gLive) return;
  const float v[3] = {c.r, c.g, c.b};
  for (int i = 0; i < 3; i++) {
    float x = clampf(v[i] / 255.f * b * master, 0.f, 1.f);
    int d = (int)(powf(x, mg::GAMMA_F) * PWM_MAX + 0.5f);
    if (gAnode) d = PWM_MAX - d;
    ledcWrite(i, d);
  }
}

static void coilWrite(float b) {
  if (!gLive) return;
  int d = (int)(powf(clampf(b, 0.f, 1.f), mg::GAMMA_F) * PWM_MAX + 0.5f);
  if (gCoilLow) d = PWM_MAX - d;
  ledcWrite(3, d);
}

void ledsDark() {
  rgbWrite(Rgb{0, 0, 0}, 0, 0);
  coilWrite(0);
}

static float breathe(uint32_t ms, uint32_t period) {
  return 0.3f + 0.25f * sinf((float)(ms % period) / (float)period * mg::TAU_F);
}

// blue -> green -> amber -> red over t = 0..1: the data-feedback gradient.
static Rgb heatColor(float t) {
  t = clampf(t, 0.f, 1.f);
  const Rgb blue{40, 120, 255}, green{40, 220, 120}, amber{255, 170, 0}, red{255, 40, 30};
  if (t < 0.3333f) return mixRgb(blue, green, t * 3.f);
  if (t < 0.6667f) return mixRgb(green, amber, (t - 0.3333f) * 3.f);
  return mixRgb(amber, red, (t - 0.6667f) * 3.f);
}

static Rgb hueColor(float h) {           // h 0..1 around the wheel, for the treatment-window sweep
  float r = fabsf(h * 6.f - 3.f) - 1.f;
  float g = 2.f - fabsf(h * 6.f - 2.f);
  float b = 2.f - fabsf(h * 6.f - 4.f);
  return Rgb{clampf(r, 0, 1) * 255.f, clampf(g, 0, 1) * 255.f, clampf(b, 0, 1) * 255.f};
}

// Online update: the whole story with no phone (see the table in leds.h).
static void updLeds(uint8_t ph, uint8_t pct, uint8_t err, uint32_t ms, Rgb* c, float* b) {
  const Rgb white{255, 255, 255};
  float br = breathe(ms, 1500);
  if (ph >= 1 && ph <= 3) {                    // 1 Wi-Fi, 2 clock, 3 release lookup: amber + N white blinks
    uint32_t t = ms % 2400, slot = t / 350;
    if (slot < ph && (t % 350) < 150) { *c = white; *b = 0.9f; }
    else { *c = Rgb{255, 170, 0}; *b = br; }
  } else if (ph == 4) {                        // downloading: a progress color with a heartbeat dip
    float f = pct / 100.f;
    Rgb a{255, 40, 0}, m{255, 200, 0}, g{30, 230, 90};
    *c = f < 0.5f ? mixRgb(a, m, f * 2.f) : mixRgb(m, g, (f - 0.5f) * 2.f);
    *b = (ms % 1000) < 120 ? 0.25f : 0.9f;
  } else if (ph == 5) {
    *c = Rgb{34, 224, 112};
    *b = 1.f;
  } else {                                     // failed: N red blinks, pause, repeat
    uint32_t n = err ? err : 1, cyc = n * 350 + 1200, t = ms % cyc;
    *c = Rgb{255, 31, 45};
    *b = (t < n * 350 && (t % 350) < 150) ? 1.f : 0.04f;
  }
}

void ledsRender(const mg::Config& cfg, const LedInput& in, int64_t nowUs) {
  uint32_t ms = (uint32_t)(nowUs / 1000);
  float master = cfg.led.master / 100.f;
  float coilMaster = cfg.led.coilBright / 100.f;
  Rgb c{0, 0, 0};
  float b = 0, coilB = 0;

  if (in.updPhase) {
    updLeds(in.updPhase, in.updPct, in.updErr, ms, &c, &b);
  } else if (in.joinPhase) {
    c = Rgb{255, 170, 0};
    b = breathe(ms, 1500);
  } else if (in.otaBusy) {
    c = Rgb{0, 170, 255};
    b = breathe(ms, 1500);
  } else if (in.state == mg::ST_TRIP) {
    uint32_t n = in.trip ? in.trip : 1, cyc = n * 350 + 1400, t = ms % cyc;
    c = Rgb{255, 31, 45};
    b = (t < n * 350 && (t % 350) < 150) ? 1.f : 0.05f;
    coilB = b * 0.5f;
  } else if (in.state == mg::ST_PAUSE) {
    uint32_t t = ms % 1600;
    bool onBlink = t < 120 || (t >= 260 && t < 380);
    c = Rgb{255, 170, 0};
    b = onBlink ? 0.9f : 0.08f;
    coilB = onBlink ? 0.5f : 0.f;
  } else if (in.state == mg::ST_DONE) {
    uint32_t t = ms % 3000;
    c = Rgb{60, 230, 130};
    if (t < 120 || (t >= 240 && t < 360) || (t >= 480 && t < 600)) b = 0.95f;
    else b = 0.25f + 0.12f * sinf((float)(t) / 3000.f * mg::TAU_F);
  } else if (in.state == mg::ST_RUN) {
    // the slow fade pulsate: one gentle swell every 3 s, never frequency-coupled
    float env = 0.38f + 0.42f * (0.5f - 0.5f * cosf((float)(ms % 3000) / 3000.f * mg::TAU_F));
    if (cfg.led.mode == 2) {
      float t = (in.dataVal - cfg.led.dataMin) / (cfg.led.dataMax - cfg.led.dataMin + 1e-6f);
      c = heatColor(t);
    } else if (cfg.led.mode == 1) {
      if (cfg.plan.n > 1 && in.stage >= 0) c = cfg.plan.st[in.stage].col;
      else {                               // single stage: sweep the wheel across the window
        float h = in.total > 1 ? (float)(in.elapsed / in.total) : (float)fmod(in.elapsed / 60.0, 1.0);
        c = hueColor(clampf(h, 0.f, 0.999f));
      }
    } else {
      c = (in.stage >= 0 && cfg.plan.n > 1) ? cfg.plan.st[in.stage].col : cfg.led.col;
    }
    b = env;
    coilB = env;
  } else {
    // idle: a quiet pulse; steadier and tinted once a phone is on the line
    if (in.client) { c = cfg.led.col; b = 0.10f + 0.04f * sinf((float)(ms % 5000) / 5000.f * mg::TAU_F); }
    else { c = Rgb{226, 236, 255}; b = 0.05f + 0.05f * sinf((float)(ms % 5000) / 5000.f * mg::TAU_F); }
  }

  rgbWrite(c, b, master);
  coilWrite(coilB * coilMaster);
}
