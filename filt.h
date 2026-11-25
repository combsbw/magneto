// filt.h - signal-cleaning primitives for the measurement pipeline: a one-pole smoother and a
// mains-hum notch, plus the lookup tables that turn a UI "strength" setting into real cutoffs.
// No Arduino headers, so test/host_test.cpp covers the math on a PC.
#pragma once
#include <math.h>
#include <stdint.h>

namespace mg {

// One-pole low-pass step size for a given cutoff at a given sample rate (the same arithmetic
// sense.cpp already used for its two fixed smoothers, just parameterized).
inline float emaAlpha(float cutoffHz, float sampleHz) {
  if (!(sampleHz > 1.f) || !(cutoffHz > 0.f)) return 1.f;
  float a = 1.f - expf(-6.28318530718f * cutoffHz / sampleHz);
  return a < 0.f ? 0.f : (a > 1.f ? 1.f : a);
}

// UI filter strength: 0 raw (today's defaults, unchanged), 1 light, 2 medium, 3 heavy.
static const int FILT_MODE_MAX = 3;
inline uint8_t filtModeClamp(int m) { return (uint8_t)(m < 0 ? 0 : (m > FILT_MODE_MAX ? FILT_MODE_MAX : m)); }
// Display smoothing cutoffs, Hz. Index 0 is exactly the pre-existing hardcoded 300/500 Hz, so
// turning the feature off reproduces today's behavior bit for bit.
inline float filtFieldHz(uint8_t mode) {
  static const float t[FILT_MODE_MAX + 1] = {300.f, 150.f, 60.f, 20.f};
  return t[filtModeClamp(mode)];
}
inline float filtDbHz(uint8_t mode) {
  static const float t[FILT_MODE_MAX + 1] = {500.f, 250.f, 100.f, 35.f};
  return t[filtModeClamp(mode)];
}

// Spectrum averaging: 0 off (each FFT frame replaces the published bins, today's behavior), 1-4
// stronger frame-to-frame exponential averaging of the magnitude spectrum.
static const int SPEC_AVG_MAX = 4;
inline uint8_t specAvgClamp(int m) { return (uint8_t)(m < 0 ? 0 : (m > SPEC_AVG_MAX ? SPEC_AVG_MAX : m)); }
inline float filtSpecAlpha(uint8_t level) {
  static const float t[SPEC_AVG_MAX + 1] = {1.f, 0.5f, 0.3f, 0.18f, 0.1f};
  return t[specAvgClamp(level)];
}

// ---- mains-hum notch: a standard RBJ-cookbook biquad notch, direct form 1 ----------------------
struct Biquad {
  float b0, b1, b2, a1, a2;
};
struct BiquadState {
  float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
};

// f0 = notch frequency, fs = sample rate, q sets how narrow the notch is (higher = narrower).
// Degenerates to a pass-through (identity) whenever f0 is not a sane fraction of fs, so a caller
// never has to special-case "filter off" separately from "filter at a silly frequency."
inline Biquad notchCoeffs(float f0, float fs, float q) {
  if (!(fs > 1.f) || !(f0 > 0.f) || f0 >= fs * 0.5f) return Biquad{1.f, 0.f, 0.f, 0.f, 0.f};
  if (q < 0.1f) q = 0.1f;
  float w0 = 6.28318530718f * f0 / fs;
  float alpha = sinf(w0) / (2.f * q);
  float cosw0 = cosf(w0);
  float a0 = 1.f + alpha;
  if (fabsf(a0) < 1e-9f) return Biquad{1.f, 0.f, 0.f, 0.f, 0.f};
  float inv = 1.f / a0;
  return Biquad{inv, -2.f * cosw0 * inv, inv, -2.f * cosw0 * inv, (1.f - alpha) * inv};
}

inline float biquadStep(const Biquad& c, BiquadState& s, float x) {
  if (!isfinite(x)) return 0.f;
  float y = c.b0 * x + c.b1 * s.x1 + c.b2 * s.x2 - c.a1 * s.y1 - c.a2 * s.y2;
  if (!isfinite(y)) {
    s = BiquadState{};
    return 0.f;
  }
  s.x2 = s.x1;
  s.x1 = x;
  s.y2 = s.y1;
  s.y1 = y;
  return y;
}

inline float emaStep(float cur, float x, float alpha) {
  if (!isfinite(x)) return isfinite(cur) ? cur : 0.f;
  if (!isfinite(cur)) return x;
  float next = cur + alpha * (x - cur);
  return isfinite(next) ? next : x;
}

// ---- user band-pass: high-pass and low-pass by corner frequency ---------------------------------
// Each side is two cascaded one-pole sections, not a biquad: at the ~10 kHz processed rate a 0.5 Hz
// high-pass biquad needs cos(w0) to a precision a float does not have, and its poles land somewhere
// else entirely. A one-pole section only needs exp(-w), which a float holds to ~1e-7, so the corner
// is where it was asked for at any setting. Each section's corner is moved so the PAIR is -3 dB at
// the entered frequency (sqrt(sqrt(2) - 1) = 0.6436), and the pair rolls off at 12 dB/octave.
// 0 on either side = that side off; both off = exact pass-through.
static const float BP_PAIR_K = 0.643594f;
struct BandPass {
  float hpA = 0.f;      // high-pass pole (0 = off)
  float lpB = 0.f;      // low-pass step  (0 = off)
};
struct BandState {
  float hx[2] = {0, 0}, hy[2] = {0, 0};   // high-pass sections: last input, last output
  float ly[2] = {0, 0};                   // low-pass sections
  bool primed = false;
};

// hpHz / lpHz are the corners asked for; fs the processed sample rate. Corners that make no sense at
// this rate (past ~0.45 fs, or a high-pass at or above the low-pass) switch that side off rather than
// leave a filter that rings or blocks everything.
inline BandPass bandCoeffs(float hpHz, float lpHz, float fs) {
  BandPass b;
  if (!(fs > 1.f)) return b;
  bool lpOn = lpHz > 0.f && lpHz < 0.45f * fs;
  bool hpOn = hpHz > 0.f && hpHz < 0.45f * fs && (!lpOn || hpHz < lpHz);
  if (hpOn) {
    float fc = hpHz * BP_PAIR_K;                    // each section lower, so the pair is -3 dB at hpHz
    b.hpA = expf(-6.28318530718f * fc / fs);
  }
  if (lpOn) {
    float fc = lpHz / BP_PAIR_K;                    // each section higher, same reason
    if (fc > 0.49f * fs) fc = 0.49f * fs;
    b.lpB = 1.f - expf(-6.28318530718f * fc / fs);
  }
  return b;
}
inline bool bandActive(const BandPass& b) { return b.hpA > 0.f || b.lpB > 0.f; }

// First sample after a reset seeds the state as if the input had always been x: no step transient.
inline void bandPrime(const BandPass& c, BandState& s, float x) {
  s.hx[0] = x;                                    // section 1 has always seen x ...
  s.hx[1] = 0.f;                                  // ... so section 2 has always seen its zero output
  s.hy[0] = s.hy[1] = 0.f;
  float lpIn = c.hpA > 0.f ? 0.f : x;             // the low-pass sees the high-pass output when both are on
  s.ly[0] = s.ly[1] = lpIn;
  s.primed = true;
}

inline float bandStep(const BandPass& c, BandState& s, float x) {
  if (!isfinite(x)) return 0.f;
  if (!bandActive(c)) return x;
  if (!s.primed) bandPrime(c, s, x);
  float y = x;
  if (c.hpA > 0.f) {
    for (int k = 0; k < 2; k++) {
      float o = c.hpA * (s.hy[k] + y - s.hx[k]);
      s.hx[k] = y;
      s.hy[k] = o;
      y = o;
    }
  }
  if (c.lpB > 0.f) {
    for (int k = 0; k < 2; k++) {
      s.ly[k] += c.lpB * (y - s.ly[k]);
      y = s.ly[k];
    }
  }
  if (!isfinite(y)) { s = BandState{}; return 0.f; }
  return y;
}

// Notch state seeded for a steady input x (unity DC gain), so a re-zero does not ring it.
inline void biquadPrime(BiquadState& s, float x) {
  s.x1 = s.x2 = x;
  s.y1 = s.y2 = x;
}

// ---- ADC stream health & staleness guard -------------------------------------------------------
// ESP32-S3 continuous ADC DMA (IDF 4.4 / arduino-esp32 2.0.17) can stop delivering frames after a
// long SPI-flash erase (non-IRAM GDMA ISR + descriptor wraparound) or a Wi-Fi RF/PHY calibration
// touch on SAR_I2C. SenseWatch detects a silent ringbuffer timeout within SENSE_STALL_MS, tries a
// zero-allocation stop/start bounce first, escalates to a full re-init if bounces do not recover
// data, and invalidates thermistor readings if no fresh window arrives within SENSE_STALE_MS.
static const uint32_t SENSE_STALL_MS = 200;
static const uint32_t SENSE_STALE_MS = 1500;
static const uint8_t SENSE_MAX_BOUNCES = 3;

enum SenseAct {
  SENSE_ACT_NONE = 0,
  SENSE_ACT_BOUNCE = 1,
  SENSE_ACT_REINIT = 2,
};

struct SenseWatch {
  uint32_t lastDataMs = 0;
  uint32_t lastAttemptMs = 0;
  uint32_t thGoodMs[2] = {0, 0};
  bool thEverOk[2] = {false, false};
  uint8_t bounces = 0;
  uint8_t badReads = 0;

  void reset(uint32_t nowMs) {
    lastDataMs = nowMs;
    lastAttemptMs = nowMs;
    bounces = 0;
    badReads = 0;
  }

  void onData(uint32_t nowMs) {
    lastDataMs = nowMs;
    bounces = 0;
    badReads = 0;
  }

  SenseAct nextRecovery(uint32_t nowMs) {
    lastAttemptMs = nowMs;
    badReads = 0;
    if (bounces < SENSE_MAX_BOUNCES) {
      bounces++;
      return SENSE_ACT_BOUNCE;
    }
    bounces = 0;
    return SENSE_ACT_REINIT;
  }

  SenseAct onReadErr(uint32_t nowMs) {
    if (++badReads >= 8) return nextRecovery(nowMs);
    return SENSE_ACT_NONE;
  }

  SenseAct checkStall(uint32_t nowMs) {
    if ((uint32_t)(nowMs - lastDataMs) < SENSE_STALL_MS) return SENSE_ACT_NONE;
    if ((uint32_t)(nowMs - lastAttemptMs) < SENSE_STALL_MS) return SENSE_ACT_NONE;
    return nextRecovery(nowMs);
  }

  bool stalled(uint32_t nowMs) const {
    return (uint32_t)(nowMs - lastDataMs) >= SENSE_STALE_MS;
  }

  void thermWindow(int ch, uint32_t nowMs, bool hasSamples, bool probeOk, float sampleF,
                   float& tempF, bool& okOut) {
    if (ch < 0 || ch > 1) return;
    if (hasSamples) {
      if (probeOk && isfinite(sampleF)) {
        tempF = isfinite(tempF) ? tempF + 0.3f * (sampleF - tempF) : sampleF;
        okOut = true;
        thGoodMs[ch] = nowMs;
        thEverOk[ch] = true;
      } else {
        okOut = false;
        tempF = NAN;
      }
      return;
    }
    if (!thEverOk[ch] || !okOut || (uint32_t)(nowMs - thGoodMs[ch]) >= SENSE_STALE_MS) {
      okOut = false;
      tempF = NAN;
    }
  }
};

}  // namespace mg
