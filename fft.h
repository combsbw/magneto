// fft.h - radix-2 FFT, Hann window and a sub-bin peak estimator for the measured-frequency readout.
// No Arduino headers, so the host tests cover it. Modeled on EspHallKit's EspHallFft.
#pragma once
#include <math.h>
#include <stddef.h>
#include <stdint.h>

namespace mg {

// In-place iterative radix-2. n must be a power of two.
inline void fftRun(float* re, float* im, int n) {
  for (int i = 1, j = 0; i < n; i++) {           // bit-reversal permutation
    int bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j |= bit;
    if (i < j) {
      float t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  for (int len = 2; len <= n; len <<= 1) {
    float ang = -6.28318530718f / (float)len;
    float wr = cosf(ang), wi = sinf(ang);
    for (int i = 0; i < n; i += len) {
      float cr = 1.f, ci = 0.f;
      for (int k = 0; k < len / 2; k++) {
        int a = i + k, b = i + k + len / 2;
        float xr = re[b] * cr - im[b] * ci;
        float xi = re[b] * ci + im[b] * cr;
        re[b] = re[a] - xr; im[b] = im[a] - xi;
        re[a] += xr; im[a] += xi;
        float nr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = nr;
      }
    }
  }
}

inline void fftRemoveDc(float* x, int n) {
  double s = 0;
  for (int i = 0; i < n; i++) s += x[i];
  float m = (float)(s / n);
  for (int i = 0; i < n; i++) x[i] -= m;
}

inline void fftHann(float* x, int n) {
  for (int i = 0; i < n; i++) {
    float w = 0.5f * (1.f - cosf(6.28318530718f * (float)i / (float)n));   // periodic Hann
    x[i] *= w;
  }
}

struct FftPeak {
  float hz;    // interpolated peak frequency
  float amp;   // sine amplitude in the input's units (Hann coherent gain corrected)
  float snr;   // peak magnitude over the band's median magnitude
};

// Dominant tone between fLo and fHi. mag[] is the one-sided magnitude array (n/2 bins), df the bin width.
// Parabolic interpolation on the log-ish magnitudes gives sub-bin frequency.
inline bool fftPeak(const float* mag, int bins, float df, float fLo, float fHi, int n, FftPeak* out) {
  int lo = (int)(fLo / df);
  if (lo < 1) lo = 1;
  int hi = (int)(fHi / df);
  if (hi > bins - 2) hi = bins - 2;
  if (hi <= lo) return false;
  int best = lo;
  float bm = 0;
  for (int i = lo; i <= hi; i++) if (mag[i] > bm) { bm = mag[i]; best = i; }
  if (bm <= 0) return false;
  float y0 = mag[best - 1], y1 = mag[best], y2 = mag[best + 1];
  float denom = y0 - 2.f * y1 + y2;
  float delta = 0;
  if (fabsf(denom) > 1e-12f) {
    delta = 0.5f * (y0 - y2) / denom;
    if (delta > 0.5f) delta = 0.5f;
    if (delta < -0.5f) delta = -0.5f;
  }
  // Median of the band for the SNR, via a coarse histogram (no allocation).
  float maxv = bm > 1e-20f ? bm : 1e-20f;
  int histo[64] = {0};
  int cnt = 0;
  for (int i = lo; i <= hi; i++) {
    int bkt = (int)(mag[i] / maxv * 63.f);
    if (bkt < 0) bkt = 0;
    if (bkt > 63) bkt = 63;
    histo[bkt]++;
    cnt++;
  }
  int half = cnt / 2, acc = 0, medB = 0;
  for (int i = 0; i < 64; i++) { acc += histo[i]; if (acc >= half) { medB = i; break; } }
  float med = ((float)medB + 0.5f) / 64.f * maxv;
  out->hz = ((float)best + delta) * df;
  out->amp = 4.f * bm / (float)n;     // periodic Hann coherent gain is 0.5: a sine of A peaks at A*N/4
  out->snr = med > 0 ? bm / med : 0;
  return true;
}

}  // namespace mg
