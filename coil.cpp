#include "coil.h"

#include <Arduino.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "engine.h"

static hw_timer_t* gTimer = nullptr;
static volatile int gPin = -1;
static volatile int gPhase = 0;          // 0 = next edge turns the coil on
static volatile uint32_t gOnUs = 0, gOffUs = 0;
static volatile uint32_t gPendOn = 0, gPendOff = 0;
static volatile bool gPend = false;
static volatile bool gRun = false;
static volatile bool gDc = false;
static volatile int gLevel = 0;
static volatile int64_t gT0 = 0;
static volatile uint32_t gCycles = 0;

static void forceLow() {
  int pin = gPin;
  if (pin < 0) return;
  gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLDOWN_ONLY);   // keeps the gate off even if the pin floats
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  gLevel = 0;
}

// Two phases, each edge re-arms the alarm for its own span. The pending values load at the ON edge,
// so a mid-cycle change never shortens or stretches the pulse that is already in flight.
static void IRAM_ATTR coilIsr() {
  if (!gTimer || !gRun) return;
  const gpio_num_t pin = (gpio_num_t)gPin;
  if (gPhase == 0) {
    if (gPend) { gOnUs = gPendOn; gOffUs = gPendOff; gPend = false; }
    gpio_set_level(pin, 1);
    gLevel = 1;
    gT0 = esp_timer_get_time();
    gCycles = gCycles + 1;
    timerAlarmWrite(gTimer, gOnUs < 1 ? 1 : gOnUs, true);
    timerWrite(gTimer, 0);
    gPhase = 1;
  } else {
    gpio_set_level(pin, 0);
    gLevel = 0;
    timerAlarmWrite(gTimer, gOffUs < 1 ? 1 : gOffUs, true);
    timerWrite(gTimer, 0);
    gPhase = 0;
  }
}

static void timerHalt() {
  if (!gTimer) return;
  timerAlarmDisable(gTimer);
}

void coilInit(int pin) {
  coilOff();
  gPin = pin;
  forceLow();
  if (!gTimer) {
    gTimer = timerBegin(1, 80, true);                 // 80 MHz / 80 = 1 us per tick
    timerAttachInterrupt(gTimer, &coilIsr, true);
  }
}

void coilOff() {
  gRun = false;
  gDc = false;
  timerHalt();
  forceLow();
  gPhase = 0;
}

void coilSet(float hz, float pwUs) {
  uint32_t per = mg::periodUs(hz);
  if (gPin < 0 || per == 0 || pwUs < 0.5f) { coilOff(); return; }
  uint32_t on = (uint32_t)(pwUs + 0.5f);
  if (on >= per) {                                    // DC: hold the gate high, no edges to time
    timerHalt();
    gRun = true;
    gDc = true;
    gOnUs = per;
    gOffUs = 0;
    gpio_set_level((gpio_num_t)gPin, 1);
    gLevel = 1;
    gT0 = esp_timer_get_time();
    return;
  }
  uint32_t off = per - on;
  if (gRun && !gDc) {                                 // already pulsing: swap at the next ON edge
    gPendOn = on;
    gPendOff = off;
    gPend = true;
    return;
  }
  gDc = false;
  gOnUs = on;
  gOffUs = off;
  gPend = false;
  gPhase = 0;
  gRun = true;
  timerAlarmWrite(gTimer, 10, true);                  // first edge right away
  timerWrite(gTimer, 0);
  timerAlarmEnable(gTimer);
}

bool coilRunning() { return gRun; }
int coilLevel() { return gLevel; }
uint32_t coilOnUs() { return gPend ? gPendOn : gOnUs; }
uint32_t coilPeriodUs() {
  if (!gRun || gDc) return 0;
  uint32_t on = gPend ? gPendOn : gOnUs, off = gPend ? gPendOff : gOffUs;
  return on + off;
}
int64_t coilPeriodT0() { return gT0; }
uint32_t coilCycles() { return gCycles; }
