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
  float y = c.b0 * x + c.b1 * s.x1 + c.b2 * s.x2 - c.a1 * s.y1 - c.a2 * s.y2;
  s.x2 = s.x1;
  s.x1 = x;
  s.y2 = s.y1;
  s.y1 = y;
  return y;
}

}  // namespace mg
