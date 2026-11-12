// engine.h - the Magneto treatment engine: run state, staged plans, timers and live tuning.
//
// Pure C++ with no Arduino headers, so test/host_test.cpp builds and runs it on a PC.
// Time is int64 microseconds from the caller (esp_timer_get_time() on the board).
//
// A plan is one to five stages, each with its own frequency, pulse width, length and LED color.
// Optionally a span of stages repeats N times (stage 1..2 three times, then stage 3, ...).
// A stage length of 0 is open-ended: the plan stops advancing there and runs until stopped.
// Simple (single-stage) treatments are a plan with one stage.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

namespace mg {

static const float TAU_F = 6.28318530718f;
static const float GAMMA_F = 2.2f;

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct Rgb { float r, g, b; };
inline Rgb mixRgb(const Rgb& a, const Rgb& b, float t) {
  t = clampf(t, 0.f, 1.f);
  return Rgb{a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t};
}

// Treatment bounds. Frequency 0 is allowed and means the coil holds off (a plan can park between stages).
static const float HZ_MIN = 0.0f;
static const float HZ_MAX = 250.0f;
static const float PW_MIN_US = 1.0f;
static const float PW_MAX_US = 500000.0f;   // half a second; the period caps it further at run time
static const int STAGE_MAX = 5;

inline float snapHz(float hz) { return roundf(clampf(hz, HZ_MIN, HZ_MAX) * 10.f) / 10.f; }
inline float snapPw(float us) { return roundf(clampf(us, PW_MIN_US, PW_MAX_US)); }

// Period of one pulse cycle in microseconds; 0 when the frequency is 0 (coil parked off).
inline uint32_t periodUs(float hz) {
  if (hz < 0.05f) return 0;
  float p = 1e6f / hz;
  if (p < 4000.f) p = 4000.f;            // 250 Hz
  if (p > 20000000.f) p = 20000000.f;
  return (uint32_t)(p + 0.5f);
}

// Duty as a fraction 0..1, from the same numbers the UI shows.
inline float dutyOf(float hz, float pwUs) {
  uint32_t per = periodUs(hz);
  if (!per) return 0.f;
  float d = pwUs / (float)per;
  return clampf(d, 0.f, 1.f);
}

struct Stage {
  bool on;        // stage participates in the plan
  float hz;
  float pwUs;
  float sec;      // 0 = open-ended (plan stops advancing here)
  Rgb col;        // LED color while this stage runs
};

struct Plan {
  Stage st[STAGE_MAX];
  int n;              // enabled stages, packed into st[0..n-1]
  int repFrom, repTo; // 0-based inclusive span that repeats; -1 = no repeat
  int repN;           // total passes through the span (>= 1)
  Plan() { reset(); }
  void reset() {
    n = 0;
    repFrom = repTo = -1;
    repN = 1;
    for (int i = 0; i < STAGE_MAX; i++) st[i] = Stage{false, 10.f, 100.f, 0.f, Rgb{80.f, 200.f, 255.f}};
  }
};

// One contiguous window of the unrolled plan: which stage, when it starts and how long it lasts.
struct Window {
  int stage;      // index into plan.st
  double start;   // seconds from treatment start
  double len;     // seconds; <= 0 = open-ended (must be the last window)
};
static const int WINDOW_MAX = STAGE_MAX * 12;   // 5 stages, repeat span up to 10 passes

// Unroll the repeat span into a flat window list. Returns the number of windows; total gets the
// planned length in seconds (0 when the plan ends in an open-ended stage).
inline int buildWindows(const Plan& p, Window* w, double* total) {
  int nw = 0;
  double t = 0;
  bool openEnd = false;
  int from = p.repFrom, to = p.repTo, reps = p.repN;
  bool rep = from >= 0 && to >= from && to < p.n && reps > 1;
  for (int i = 0; i < p.n && nw < WINDOW_MAX && !openEnd; i++) {
    int passes = (rep && i >= from && i <= to) ? 0 : 1;   // repeated span handled as a block at `from`
    if (rep && i == from) {
      for (int r = 0; r < reps && !openEnd; r++) {
        for (int j = from; j <= to && nw < WINDOW_MAX && !openEnd; j++) {
          double len = p.st[j].sec;
          w[nw++] = Window{j, t, len};
          if (len <= 0) { openEnd = true; break; }
          t += len;
        }
      }
      i = to;                      // the outer loop continues after the span
      continue;
    }
    if (!passes) continue;
    double len = p.st[i].sec;
    w[nw++] = Window{i, t, len};
    if (len <= 0) { openEnd = true; break; }
    t += len;
  }
  *total = openEnd ? 0.0 : t;
  return nw;
}

enum State : uint8_t { ST_IDLE = 0, ST_RUN, ST_PAUSE, ST_DONE, ST_TRIP };

inline const char* stateName(State s) {
  switch (s) {
    case ST_RUN: return "run";
    case ST_PAUSE: return "pause";
    case ST_DONE: return "done";
    case ST_TRIP: return "trip";
    default: return "idle";
  }
}

// Why the engine tripped (safety aborts). The number is also flashed on the status LED.
enum TripCode : uint8_t { TR_NONE = 0, TR_COIL_TEMP = 1, TR_FET_TEMP = 2, TR_COIL_RATE = 3, TR_FET_RATE = 4, TR_STUCK = 5, TR_PROBE = 6 };

struct Engine {
  State state = ST_IDLE;
  Plan plan;
  Window win[WINDOW_MAX];
  int nWin = 0;
  double total = 0;              // planned seconds; 0 = open-ended
  int64_t t0 = 0;                // run started / resumed (us)
  double acc = 0;                // seconds elapsed before the last pause
  TripCode trip = TR_NONE;
  // Live tuning holds an override for the window it was made in; the next window returns to the plan.
  int tuneWin = -1;
  float tuneHz = 0, tunePw = 0;

  void configure(const Plan& p) {
    plan = p;
    nWin = buildWindows(plan, win, &total);
  }

  double elapsed(int64_t now) const {
    double t = acc;
    if (state == ST_RUN) t += (double)(now - t0) * 1e-6;
    return t < 0 ? 0 : t;
  }

  // Which window covers second t. The last window soaks up any overshoot (caller handles DONE).
  int windowAt(double t) const {
    for (int i = 0; i < nWin; i++) {
      if (win[i].len <= 0) return i;                       // open-ended
      if (t < win[i].start + win[i].len) return i;
    }
    return nWin ? nWin - 1 : -1;
  }

  bool running() const { return state == ST_RUN || state == ST_PAUSE; }

  bool start(int64_t now) {
    if (state == ST_RUN || state == ST_TRIP) return false;
    if (!nWin) return false;
    if (state != ST_PAUSE) { acc = 0; tuneWin = -1; }
    t0 = now;
    state = ST_RUN;
    return true;
  }

  bool pause(int64_t now) {
    if (state != ST_RUN) return false;
    acc = elapsed(now);
    state = ST_PAUSE;
    return true;
  }

  void stop() {
    if (state == ST_TRIP) return;      // a trip only clears through reset()
    state = ST_IDLE;
    acc = 0;
    tuneWin = -1;
  }

  // Zero the clock without stopping (mid-run restart of the timer).
  void resetTime(int64_t now) {
    acc = 0;
    t0 = now;
    tuneWin = -1;
  }

  void finish() {               // planned time ran out
    if (state == ST_RUN) { acc = total; state = ST_DONE; }
  }

  void tripNow(int64_t now, TripCode code) {
    if (state == ST_RUN) acc = elapsed(now);
    state = ST_TRIP;
    trip = code;
  }

  // Clears a trip back to idle. The caller gates this on a fresh in-range temperature.
  void tripReset() {
    if (state != ST_TRIP) return;
    state = ST_IDLE;
    trip = TR_NONE;
    acc = 0;
    tuneWin = -1;
  }

  // The drive the coil should produce right now. False = coil off (idle, paused, done, tripped, done plan).
  bool drive(int64_t now, float* hz, float* pwUs, int* stageIdx, double* stageT) const {
    *hz = 0; *pwUs = 0; *stageIdx = -1; *stageT = 0;
    if (state != ST_RUN || !nWin) return false;
    double t = elapsed(now);
    if (total > 0 && t >= total) return false;             // loop() will call finish()
    int wi = windowAt(t);
    if (wi < 0) return false;
    const Window& w = win[wi];
    const Stage& s = plan.st[w.stage];
    *stageIdx = w.stage;
    *stageT = t - w.start;
    if (tuneWin == wi) { *hz = tuneHz; *pwUs = tunePw; }
    else { *hz = s.hz; *pwUs = s.pwUs; }
    return true;
  }

  // Live adjustment while running: applies to the current window only.
  bool tune(int64_t now, float hz, float pwUs) {
    if (state != ST_RUN) return false;
    int wi = windowAt(elapsed(now));
    if (wi < 0) return false;
    tuneWin = wi;
    tuneHz = snapHz(hz);
    tunePw = snapPw(pwUs);
    return true;
  }
};

}  // namespace mg
