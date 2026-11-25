// host_test.cpp - PC tests of the treatment engine, the settings parser and the update helpers.
//   g++ -std=c++11 -Wall -Wextra -O1 -I. test/host_test.cpp -o test/host_test && test/host_test
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "../engine.h"
#include "../json.h"
#include "../config.h"
#include "../filt.h"
#include "../ghupdate.h"
#include "../netprobe.h"
#include <cmath>
#include <initializer_list>

static int failures = 0;

#define CHECK(cond)                                                     \
  do {                                                                  \
    if (!(cond)) {                                                      \
      printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);            \
      failures++;                                                       \
    }                                                                   \
  } while (0)

static bool near(double a, double b, double eps = 1e-6) { return a > b - eps && a < b + eps; }

static void testPeriod() {
  CHECK(mg::periodUs(0.f) == 0);
  CHECK(mg::periodUs(10.f) == 100000);
  CHECK(mg::periodUs(250.f) == 4000);
  CHECK(mg::periodUs(0.1f) == 10000000);
  CHECK(near(mg::dutyOf(10.f, 100.f), 0.001));
  CHECK(near(mg::dutyOf(0.f, 100.f), 0.0));
  CHECK(near(mg::dutyOf(10.f, 200000.f), 1.0));   // pw past the period clamps to DC
  CHECK(mg::snapHz(123.456f) == 123.5f);
  CHECK(mg::snapHz(999.f) == 250.f);
}

// The two promises the pulse-width slider makes: nothing above the ceiling ever reaches the coil,
// and no setting can ask for a pulse as long as its own period (which would be DC, not a pulse).
static void testPwCap() {
  CHECK(mg::pwCapUs(0.f) == 0.f);                 // parked
  CHECK(mg::pwCapUs(10.f) == 99999.f);            // 100 ms period
  CHECK(mg::pwCapUs(250.f) == 3999.f);            // 4 ms period

  // the ceiling is the maximum, exactly
  CHECK(mg::pwFor(10.f, 5000.f, 1000.f) == 1000.f);
  CHECK(mg::pwFor(10.f, 1000.f, 1000.f) == 1000.f);
  CHECK(mg::pwFor(10.f, 999.f, 1000.f) == 999.f);
  CHECK(mg::pwFor(10.f, 1e9f, 1000.f) == 1000.f);

  // no ceiling: the period still caps it, and never equals the period
  CHECK(mg::pwFor(10.f, 500000.f, 0.f) == 99999.f);
  CHECK(mg::pwFor(200.f, 500000.f, 0.f) == 4999.f);
  CHECK(mg::pwFor(0.f, 100.f, 0.f) == 0.f);

  // the bug this closes: a pulse set at a low frequency, then the frequency raised
  CHECK(mg::pwFor(10.f, 10000.f, 0.f) == 10000.f);
  CHECK(mg::pwFor(200.f, 10000.f, 0.f) == 4999.f);
  for (float hz = 0.1f; hz <= 250.f; hz += 0.7f) {
    float pw = mg::pwFor(hz, 500000.f, 0.f);
    CHECK(pw < (float)mg::periodUs(hz));          // always an off phase
    CHECK(mg::dutyOf(hz, pw) < 1.f);
  }
  // a ceiling never lets anything through above itself, at any frequency
  for (float hz = 0.1f; hz <= 250.f; hz += 1.3f) {
    CHECK(mg::pwFor(hz, 500000.f, 250.f) <= 250.f);
  }
}

static void testWindows() {
  mg::Plan p;
  p.n = 3;
  p.st[0] = mg::Stage{true, 10, 100, 60, {0, 0, 0}};
  p.st[1] = mg::Stage{true, 20, 50, 30, {0, 0, 0}};
  p.st[2] = mg::Stage{true, 5, 200, 10, {0, 0, 0}};
  mg::Window w[mg::WINDOW_MAX];
  double total;
  int n = mg::buildWindows(p, w, &total);
  CHECK(n == 3);
  CHECK(near(total, 100));
  CHECK(w[1].start == 60 && w[1].len == 30);

  // repeat stages 0..1 three times: 0 1 0 1 0 1 2
  p.repFrom = 0; p.repTo = 1; p.repN = 3;
  n = mg::buildWindows(p, w, &total);
  CHECK(n == 7);
  CHECK(near(total, 3 * 90 + 10));
  CHECK(w[2].stage == 0 && near(w[2].start, 90));
  CHECK(w[6].stage == 2 && near(w[6].start, 270));

  // an open-ended stage ends the unroll
  p.st[1].sec = 0;
  n = mg::buildWindows(p, w, &total);
  CHECK(total == 0);
  CHECK(w[n - 1].len <= 0);
  CHECK(w[n - 1].stage == 1);
}

static void testEngine() {
  mg::Engine e;
  mg::Plan p;
  p.n = 2;
  p.st[0] = mg::Stage{true, 10, 100, 60, {0, 0, 0}};
  p.st[1] = mg::Stage{true, 20, 50, 0, {0, 0, 0}};
  e.configure(p);
  CHECK(e.state == mg::ST_IDLE);
  int64_t us = 1000000;
  CHECK(e.start(us));
  float hz, pw;
  int si;
  double st;
  CHECK(e.drive(us + 1000000, &hz, &pw, &si, &st));
  CHECK(hz == 10.f && si == 0);
  CHECK(e.drive(us + 61 * 1000000LL, &hz, &pw, &si, &st));
  CHECK(hz == 20.f && si == 1);

  // live tune applies only inside the current window
  CHECK(e.tune(us + 61 * 1000000LL, 25.f, 40.f));
  CHECK(e.drive(us + 62 * 1000000LL, &hz, &pw, &si, &st));
  CHECK(hz == 25.f && pw == 40.f);

  // pause holds the clock
  CHECK(e.pause(us + 70 * 1000000LL));
  double t1 = e.elapsed(us + 70 * 1000000LL);
  double t2 = e.elapsed(us + 90 * 1000000LL);
  CHECK(near(t1, t2));
  CHECK(!e.drive(us + 90 * 1000000LL, &hz, &pw, &si, &st));
  CHECK(e.start(us + 90 * 1000000LL));            // resume
  CHECK(near(e.elapsed(us + 91 * 1000000LL), 71));

  // trip latches: stop() does not clear it, tripReset() does
  e.tripNow(us + 95 * 1000000LL, mg::TR_COIL_TEMP);
  CHECK(e.state == mg::ST_TRIP);
  e.stop();
  CHECK(e.state == mg::ST_TRIP);
  e.tripReset();
  CHECK(e.state == mg::ST_IDLE && e.trip == mg::TR_NONE);

  // a timed plan finishes
  p.st[1].sec = 10;
  e.configure(p);
  CHECK(e.start(us));
  CHECK(!e.drive(us + 71 * 1000000LL, &hz, &pw, &si, &st));
  e.finish();
  CHECK(e.state == mg::ST_DONE);
}

static void testConfig() {
  const char* js =
    "{\"pins\":{\"drive\":7,\"hallC\":1,\"hallP\":2,\"thC\":3,\"thF\":4,\"r\":5,\"g\":6,\"b\":8,\"coil\":9,\"anode\":true,\"coilLow\":false},"
    "\"run\":{\"stages\":[{\"on\":true,\"hz\":12.3,\"pw\":55,\"sec\":120,\"col\":\"#ff8800\"},{\"on\":true,\"hz\":7,\"pw\":1000,\"sec\":60}],"
    "\"rFrom\":0,\"rTo\":1,\"rN\":2},"
    "\"max\":{\"hz\":100,\"pw\":5000},"
    "\"safe\":{\"coilMax\":110,\"fetMax\":140,\"coilRate\":20,\"fetRate\":20,\"on\":true,\"needReset\":true,\"needProbe\":false},"
    "\"led\":{\"mode\":1,\"col\":\"#22ccff\",\"master\":70,\"dataSrc\":2,\"dataMin\":0,\"dataMax\":500,\"coilBright\":60},"
    "\"hall\":{\"sens5c\":1.4,\"sens5p\":1.8,\"vcc\":3.3,\"ratio\":true},"
    "\"therm\":{\"beta\":3950,\"r0\":10,\"rs\":10,\"top\":false}}";
  mg::Config c;
  bool pinsOk = false;
  CHECK(mg::cfgFromJson(js, strlen(js), c, &pinsOk));
  CHECK(pinsOk);
  CHECK(c.pins.drive == 7 && c.pins.coil == 9);
  CHECK(c.plan.n == 2);
  CHECK(near(c.plan.st[0].hz, 12.3, 1e-4));
  CHECK(near(c.plan.st[0].col.r, 255) && near(c.plan.st[0].col.g, 136));
  CHECK(c.plan.repFrom == 0 && c.plan.repTo == 1 && c.plan.repN == 2);
  CHECK(c.maxHz == 100.f && c.maxPw == 5000.f);
  CHECK(c.safe.coilMaxF == 110.f && !c.safe.needProbe);
  CHECK(c.led.mode == 1 && c.led.master == 70);
  CHECK(near(c.hall.sens5p, 1.8, 1e-4));
  CHECK(c.filt.mode == 0 && !c.filt.notch && c.filt.notchHz == 60 && c.filt.specAvg == 0);  // default: off

  // a duplicate pin falls back to defaults but keeps polarity
  const char* bad = "{\"pins\":{\"drive\":7,\"hallC\":7,\"hallP\":2,\"thC\":3,\"thF\":4,\"r\":5,\"g\":6,\"b\":8,\"coil\":9,\"anode\":false}}";
  CHECK(mg::cfgFromJson(bad, strlen(bad), c, &pinsOk));
  CHECK(!pinsOk);
  CHECK(c.pins.drive == 7 && c.pins.hallC == 1);
  CHECK(c.pins.anode == false);

  // ADC inputs must be GPIO 1-9
  const char* badAdc = "{\"pins\":{\"drive\":5,\"hallC\":21,\"hallP\":2,\"thC\":3,\"thF\":4,\"r\":6,\"g\":7,\"b\":8,\"coil\":9}}";
  CHECK(mg::cfgFromJson(badAdc, strlen(badAdc), c, &pinsOk));
  CHECK(!pinsOk);

  // ceilings clamp the plan itself
  const char* ceil =
    "{\"run\":{\"stages\":[{\"on\":true,\"hz\":200,\"pw\":9000,\"sec\":10}]},\"max\":{\"hz\":50,\"pw\":500}}";
  CHECK(mg::cfgFromJson(ceil, strlen(ceil), c, &pinsOk));
  CHECK(c.plan.st[0].hz == 50.f && c.plan.st[0].pwUs == 500.f);

  // ... and with no ceiling the period still does: 9 ms asked for at 200 Hz (5 ms period)
  const char* wide = "{\"run\":{\"stages\":[{\"on\":true,\"hz\":200,\"pw\":9000,\"sec\":10}]}}";
  CHECK(mg::cfgFromJson(wide, strlen(wide), c, &pinsOk));
  CHECK(c.plan.st[0].pwUs == 4999.f);
  CHECK(mg::dutyOf(c.plan.st[0].hz, c.plan.st[0].pwUs) < 1.f);

  // the magnet escape hatch has defaults and takes settings
  const char* mag = "{\"safe\":{\"magAp\":true,\"magUt\":1500,\"magSec\":8}}";
  CHECK(mg::cfgFromJson(mag, strlen(mag), c, &pinsOk));
  CHECK(c.safe.magAp && c.safe.magUt == 1500.f && c.safe.magSec == 8.f);
  const char* magOff = "{\"safe\":{\"magAp\":false,\"magUt\":5,\"magSec\":900}}";
  CHECK(mg::cfgFromJson(magOff, strlen(magOff), c, &pinsOk));
  CHECK(!c.safe.magAp && c.safe.magUt == 200.f && c.safe.magSec == 60.f);   // out of range clamps

  const char* filt = "{\"filt\":{\"mode\":2,\"notch\":true,\"notchHz\":50,\"specAvg\":3}}";
  CHECK(mg::cfgFromJson(filt, strlen(filt), c, &pinsOk));
  CHECK(c.filt.mode == 2 && c.filt.notch && c.filt.notchHz == 50 && c.filt.specAvg == 3);
  const char* filtClamp = "{\"filt\":{\"mode\":99,\"notchHz\":52,\"specAvg\":99}}";
  CHECK(mg::cfgFromJson(filtClamp, strlen(filtClamp), c, &pinsOk));
  CHECK(c.filt.mode == mg::FILT_MODE_MAX && c.filt.notchHz == 50 && c.filt.specAvg == mg::SPEC_AVG_MAX);

  CHECK(mg::presetsJsonOk("[]", 2));
  CHECK(mg::presetsJsonOk("[{\"name\":\"a\"}]", 14));
  CHECK(!mg::presetsJsonOk("{}", 2));
  CHECK(!mg::presetsJsonOk("[{", 2));
}

// The pipeline's "off" state has to reproduce exactly what it did before filtering existed, and the
// notch has to actually notch (and never blow up) across both mains frequencies.
static void testFilt() {
  CHECK(mg::filtFieldHz(0) == 300.f);     // the fixed cutoffs this always used, unchanged at mode 0
  CHECK(mg::filtDbHz(0) == 500.f);
  CHECK(mg::filtModeClamp(-5) == 0 && mg::filtModeClamp(99) == mg::FILT_MODE_MAX);
  CHECK(mg::specAvgClamp(-5) == 0 && mg::specAvgClamp(99) == mg::SPEC_AVG_MAX);
  CHECK(mg::filtSpecAlpha(0) == 1.f);      // off = each frame replaces the last, no blending
  for (int m = 1; m <= mg::FILT_MODE_MAX; m++) {
    CHECK(mg::filtFieldHz(m) < mg::filtFieldHz(m - 1));   // strictly stronger smoothing each step up
    CHECK(mg::filtDbHz(m) < mg::filtDbHz(m - 1));
  }
  CHECK(near(mg::emaAlpha(0.f, 1000.f), 1.f));     // no cutoff = no smoothing (pass the sample through)
  CHECK(mg::emaAlpha(300.f, 0.f) == 1.f);          // guarded against a bogus sample rate
  CHECK(mg::emaAlpha(300.f, 10000.f) > 0.f && mg::emaAlpha(300.f, 10000.f) < 1.f);

  // identity (pass-through) when the frequency makes no sense for this sample rate
  mg::Biquad id = mg::notchCoeffs(0.f, 8000.f, 12.f);
  CHECK(id.b0 == 1.f && id.b1 == 0.f && id.b2 == 0.f && id.a1 == 0.f && id.a2 == 0.f);
  id = mg::notchCoeffs(60.f, 100.f, 12.f);          // past Nyquist at this rate
  CHECK(id.b0 == 1.f && id.a1 == 0.f);

  for (float f0 : {50.f, 60.f}) {
    const float fs = 8000.f;
    mg::Biquad c = mg::notchCoeffs(f0, fs, 12.f);
    auto rmsAt = [&](float freq) {
      mg::BiquadState st;
      const int n = 4000;
      double inSq = 0, outSq = 0;
      for (int i = 0; i < n; i++) {
        float t = (float)i / fs;
        float x = sinf(mg::TAU_F * freq * t);
        float y = mg::biquadStep(c, st, x);
        CHECK(std::isfinite(y) && fabsf(y) < 5.f);     // bounded and stable, never blows up
        if (i >= n / 2) { inSq += (double)x * x; outSq += (double)y * y; }   // skip the startup transient
      }
      return sqrt(outSq / inSq);
    };
    CHECK(rmsAt(f0) < 0.1);           // the notch frequency itself is strongly attenuated
    CHECK(rmsAt(200.f) > 0.85);       // well clear of the notch: passed through close to untouched
  }

  // NaN / Inf inputs must never poison the biquad or EMA smoothers permanently
  mg::Biquad c60 = mg::notchCoeffs(60.f, 8000.f, 12.f);
  mg::BiquadState nst;
  CHECK(mg::biquadStep(c60, nst, NAN) == 0.f);
  CHECK(std::isfinite(mg::biquadStep(c60, nst, 42.f)));
  nst.y1 = NAN;
  CHECK(mg::biquadStep(c60, nst, 10.f) == 0.f);
  CHECK(std::isfinite(mg::biquadStep(c60, nst, 10.f)));

  CHECK(near(mg::emaStep(NAN, 25.f, 0.2f), 25.f));
  CHECK(near(mg::emaStep(25.f, NAN, 0.2f), 25.f));
  CHECK(near(mg::emaStep(0.f, 100.f, 0.25f), 25.f));
}

// Continuous ADC DMA stall recovery and stale-sensor fail-safe: a silent timeout stream must bounce
// the controller within SENSE_STALL_MS, escalate to full re-init if bounces fail, hold thermistors
// steady across a brief bounce, and fail safe to "no probe" if no fresh data arrives for >= 1.5 s.
static void testSenseWatch() {
  mg::SenseWatch w;
  w.reset(1000);
  CHECK(w.checkStall(1100) == mg::SENSE_ACT_NONE);
  w.onData(1150);
  CHECK(w.checkStall(1300) == mg::SENSE_ACT_NONE);

  // Stall starts after t=1150: three soft bounces, then a full re-init on the fourth timeout window
  CHECK(w.checkStall(1350) == mg::SENSE_ACT_BOUNCE);
  CHECK(w.checkStall(1450) == mg::SENSE_ACT_NONE);     // rate-limited while the bounce settles
  CHECK(w.checkStall(1550) == mg::SENSE_ACT_BOUNCE);
  CHECK(w.checkStall(1750) == mg::SENSE_ACT_BOUNCE);
  CHECK(w.checkStall(1950) == mg::SENSE_ACT_REINIT);
  CHECK(!w.stalled(2600));
  CHECK(w.stalled(2650));                              // >= 1500 ms since last good data

  // Data returning clears the bounce escalation counter immediately
  w.onData(2700);
  CHECK(!w.stalled(2700));
  CHECK(w.bounces == 0);
  CHECK(w.checkStall(2900) == mg::SENSE_ACT_BOUNCE);

  // Thermistor window: boot with dead DMA stays unready (ok == false, NAN)
  mg::SenseWatch tw;
  tw.reset(0);
  float tF = NAN;
  bool ok = false;
  tw.thermWindow(0, 250, false, false, NAN, tF, ok);
  CHECK(!ok && std::isnan(tF));

  // Valid thermistor windows smooth normally
  tw.thermWindow(0, 500, true, true, 78.f, tF, ok);
  CHECK(ok && near(tF, 78.f));
  tw.thermWindow(0, 750, true, true, 88.f, tF, ok);
  CHECK(ok && near(tF, 81.f));

  // A brief 1-2 bin DMA dropout during an NVS erase + bounce keeps the reading alive (no false trip)
  tw.thermWindow(0, 1000, false, false, NAN, tF, ok);
  CHECK(ok && near(tF, 81.f));
  tw.thermWindow(0, 1500, false, false, NAN, tF, ok);
  CHECK(ok && near(tF, 81.f));

  // If DMA still has not delivered thermistor samples after SENSE_STALE_MS (1500 ms), fail safe!
  tw.thermWindow(0, 2250, false, false, NAN, tF, ok);
  CHECK(!ok && std::isnan(tF));

  // Recovery when samples come back, and immediate (250 ms) invalidation on a real open probe
  tw.thermWindow(0, 2500, true, true, 82.f, tF, ok);
  CHECK(ok && near(tF, 82.f));
  tw.thermWindow(0, 2750, true, false, NAN, tF, ok);
  CHECK(!ok && std::isnan(tF));
}

static void testGh() {
  CHECK(mg::ghRepoOk("combsbw/magneto"));
  CHECK(!mg::ghRepoOk("nope"));
  CHECK(!mg::ghRepoOk("a/b/c"));
  CHECK(mg::ghTokenOk("ghp_abc123"));
  CHECK(!mg::ghTokenOk("bad token"));
  char host[80];
  mg::ghUrlHost("https://api.github.com/repos/x", host, sizeof host);
  CHECK(strcmp(host, "api.github.com") == 0);
  CHECK(mg::ghIsApiHost(host));
  const char* rel =
    "{\"assets\":[{\"url\":\"https://api.github.com/repos/combsbw/magneto/releases/assets/1\",\"name\":\"other.bin\"},"
    "{\"url\":\"https://api.github.com/repos/combsbw/magneto/releases/assets/2\",\"name\":\"magneto.ota.bin\"}]}";
  char out[200];
  CHECK(mg::ghFindAssetUrl(rel, strlen(rel), "magneto.ota.bin", out, sizeof out));
  CHECK(strstr(out, "/assets/2") != nullptr);
}

static void testTuneCap() {
  mg::Engine e;
  mg::Plan p;
  p.n = 1;
  p.st[0] = mg::Stage{true, 10, 100, 0, {0, 0, 0}};
  e.configure(p);
  CHECK(e.start(0));
  // a tune asking for more than the period gets the period cap, not DC
  CHECK(e.tune(0, 200.f, 10000.f));
  float hz = 0, pw = 0;
  int si = 0;
  double st = 0;
  CHECK(e.drive(0, &hz, &pw, &si, &st));
  CHECK(hz == 200.f);
  CHECK(pw == 4999.f);
  CHECK(mg::dutyOf(hz, pw) < 1.f);
}

// The secure socket reads command arguments with this, so a bug here means buttons do nothing.
static void testQs() {
  char v[16];
  CHECK(mg::qsParam("c=tune&hz=12.5&pw=100", "c", v, sizeof v) && !strcmp(v, "tune"));
  CHECK(mg::qsParam("c=tune&hz=12.5&pw=100", "hz", v, sizeof v) && !strcmp(v, "12.5"));
  CHECK(mg::qsParam("c=tune&hz=12.5&pw=100", "pw", v, sizeof v) && !strcmp(v, "100"));
  CHECK(!mg::qsParam("c=tune&hz=12.5", "pw", v, sizeof v) && v[0] == 0);
  CHECK(mg::qsParam("c=start", "c", v, sizeof v) && !strcmp(v, "start"));
  CHECK(mg::qsParam("c=clear&w=max", "w", v, sizeof v) && !strcmp(v, "max"));
  CHECK(mg::qsParam("ch=1", "ch", v, sizeof v) && !strcmp(v, "1"));
  CHECK(mg::qsParam("a=1&c=", "c", v, sizeof v) && v[0] == 0);     // present, empty value
  CHECK(mg::qsParam("c=&hz=9", "c", v, sizeof v) && v[0] == 0);
  CHECK(!mg::qsParam("", "c", v, sizeof v));
  CHECK(!mg::qsParam(nullptr, "c", v, sizeof v));
  CHECK(!mg::qsParam("hzx=5", "hz", v, sizeof v));                 // no prefix match
  CHECK(!mg::qsParam("xhz=5", "hz", v, sizeof v));
  CHECK(mg::qsParam("a=1&&hz=7", "hz", v, sizeof v) && !strcmp(v, "7"));
  // a value longer than the buffer is truncated, never overrun
  char small[4];
  CHECK(mg::qsParam("hz=123456789", "hz", small, sizeof small) && !strcmp(small, "123"));
}

// The soft AP answers every name, so a phone's internet check lands here. Replying with the page
// is what makes iOS call the network a captive portal and leave it for cellular within seconds, so
// each probe has to get the exact reply its OS expects - and everything else must still reach the
// page.
static void testProbe() {
  using namespace mg;
  // answered as a working network
  CHECK(probeKind("/hotspot-detect.html", "captive.apple.com") == PROBE_APPLE);
  CHECK(probeKind("/library/test/success.html", "www.apple.com") == PROBE_APPLE);
  CHECK(probeKind("/hotspot-detect.html", "192.168.4.1") == PROBE_APPLE);      // path alone is enough
  CHECK(probeKind("/", "captive.apple.com") == PROBE_APPLE);                   // host alone is enough
  CHECK(probeKind("/generate_204", "connectivitycheck.gstatic.com") == PROBE_204);
  CHECK(probeKind("/generate_204", "clients3.google.com") == PROBE_204);
  CHECK(probeKind("/gen_204", "www.google.com") == PROBE_204);
  CHECK(probeKind("/connecttest.txt", "www.msftconnecttest.com") == PROBE_CONNECTTEST);
  // the legacy Windows check shares a host with the newer one and wants its own body
  CHECK(probeKind("/ncsi.txt", "www.msftncsi.com") == PROBE_NCSI_TXT);
  CHECK(strcmp(probeBody(PROBE_NCSI_TXT), "Microsoft NCSI") == 0);
  CHECK(strcmp(probeBody(PROBE_CONNECTTEST), "Microsoft Connect Test") == 0);
  CHECK(strcmp(probeBody(PROBE_APPLE),
               "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>") == 0);
  CHECK(probeCode(PROBE_204) == 204 && probeCode(PROBE_APPLE) == 200);
  CHECK(strcmp(probeType(PROBE_APPLE), "text/html") == 0);
  CHECK(strcmp(probeType(PROBE_204), "text/plain") == 0);

  // everything else still goes to the page
  CHECK(probeKind("/", "example.com") == PROBE_NONE);
  CHECK(probeKind("/favicon.ico", "192.168.4.1") == PROBE_NONE);
  CHECK(probeKind("/whatever", "magneto.local") == PROBE_NONE);
  CHECK(probeKind("/news", "news.ycombinator.com") == PROBE_NONE);
  CHECK(probeKind("/api/state", "192.168.4.1") == PROBE_NONE);
  CHECK(probeKind(nullptr, nullptr) == PROBE_NONE);
}

int main() {
  testPeriod();
  testProbe();
  testQs();
  testPwCap();
  testTuneCap();
  testWindows();
  testEngine();
  testConfig();
  testFilt();
  testSenseWatch();
  testGh();
  if (failures) {
    printf("%d failure(s)\n", failures);
    return 1;
  }
  printf("all tests passed\n");
  return 0;
}
