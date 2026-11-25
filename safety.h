// safety.h - the thermal trip decision, debounced against single bad readings.
//
// The checks run every 250 ms on a thermistor window that averages thousands of conversions, but
// one window can still be wrong for reasons that have nothing to do with the coil's temperature: the
// first conversions after the ADC DMA is restarted (it is restarted after every flash write and
// Wi-Fi reconnect), or a brief open on a probe lead. Tripping a running treatment on one such window
// is a false stop. Each condition now has to hold continuously for a short time before it trips:
//   over-temperature   0.5 s  (two consecutive windows)
//   heating too fast   2 s    (the rate is a 15-45 s slope; 2 s more is nothing thermally)
//   probe not answering 1.5 s (the same staleness bound the sampler itself uses)
// A real over-temperature still trips within half a second.
//
// Pure C++, host-tested.
#pragma once
#include <stdint.h>
#include "engine.h"

namespace mg {

static const uint32_t SAFE_TEMP_MS = 500;
static const uint32_t SAFE_RATE_MS = 2000;
static const uint32_t SAFE_PROBE_MS = 1500;

struct SafetyIn {
  bool okC, okF;
  float tC, tF;           // degF
  float dtdtC, dtdtF;     // degF per minute
};
struct SafetyLim {
  float coilMaxF, fetMaxF, coilRateF, fetRateF;
  bool needProbe;
};

struct SafetyGate {
  uint32_t probeMs = 0, coilMs = 0, fetMs = 0, coilRMs = 0, fetRMs = 0;

  void reset() { probeMs = coilMs = fetMs = coilRMs = fetRMs = 0; }

  static bool hold(bool cond, uint32_t& acc, uint32_t dt, uint32_t need) {
    if (!cond) { acc = 0; return false; }
    acc += dt;
    return acc >= need;
  }

  // dtMs = time since the previous check. Returns the trip to take, TR_NONE for none.
  TripCode step(const SafetyIn& s, const SafetyLim& l, uint32_t dtMs) {
    if (dtMs > 300) dtMs = 300;              // a stalled loop must not count as time held
    if (dtMs == 0) dtMs = 1;
    bool probe = hold(l.needProbe && (!s.okC || !s.okF), probeMs, dtMs, SAFE_PROBE_MS);
    bool coil = hold(s.okC && s.tC > l.coilMaxF, coilMs, dtMs, SAFE_TEMP_MS);
    bool fet = hold(s.okF && s.tF > l.fetMaxF, fetMs, dtMs, SAFE_TEMP_MS);
    bool coilR = hold(s.okC && s.dtdtC > l.coilRateF, coilRMs, dtMs, SAFE_RATE_MS);
    bool fetR = hold(s.okF && s.dtdtF > l.fetRateF, fetRMs, dtMs, SAFE_RATE_MS);
    if (coil) return TR_COIL_TEMP;
    if (fet) return TR_FET_TEMP;
    if (probe) return TR_PROBE;
    if (coilR) return TR_COIL_RATE;
    if (fetR) return TR_FET_RATE;
    return TR_NONE;
  }
};

}  // namespace mg
