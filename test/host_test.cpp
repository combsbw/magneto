// host_test.cpp - PC tests of the treatment engine, the settings parser and the update helpers.
//   g++ -std=c++11 -Wall -Wextra -O1 -I. test/host_test.cpp -o test/host_test && test/host_test
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "../engine.h"
#include "../json.h"
#include "../config.h"
#include "../ghupdate.h"

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

  CHECK(mg::presetsJsonOk("[]", 2));
  CHECK(mg::presetsJsonOk("[{\"name\":\"a\"}]", 14));
  CHECK(!mg::presetsJsonOk("{}", 2));
  CHECK(!mg::presetsJsonOk("[{", 2));
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

int main() {
  testPeriod();
  testWindows();
  testEngine();
  testConfig();
  testGh();
  if (failures) {
    printf("%d failure(s)\n", failures);
    return 1;
  }
  printf("all tests passed\n");
  return 0;
}
