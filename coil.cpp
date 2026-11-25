#include "coil.h"

#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/timer.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "esp_timer.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#include "engine.h"

// timerBegin(1, ...) in arduino-esp32 2.0.x is timer group 1, timer 0 (its timer_dev[] table is
// {0,0}, {1,0}, {0,1}, {1,1}). The ISR talks to that timer through the IDF's *_in_isr calls, which
// are IRAM functions, so the two must stay in step.
#define COIL_TIMER_NUM 1
#define COIL_TG TIMER_GROUP_1
#define COIL_TI TIMER_0

// Why this file looks the way it does
// -----------------------------------
// The first version armed the timer with timerAttachInterrupt(), which registers a plain
// (flash-resident) interrupt, and re-armed it from the ISR with timerAlarmWrite()/timerWrite() and
// gpio_set_level() - all flash functions. Every SPI-flash write disables the flash cache, and with it
// every interrupt that is not IRAM-safe, for the length of the write: an NVS settings save (two
// seconds after every slider or settings change, and on every Start with edited settings), a sensor
// zero (four NVS keys), a saved setup (LittleFS). When a write began while the gate was high, the gate
// stayed high until the write ended - a 100-500 us pulse became a 10-50 ms one, close to DC through a
// low-resistance coil on a 24 V rail. That is enough to sag the supply into a brown-out reset (the
// treatment "stops on its own", and the settings save that caused it never completes, so the board
// comes back with the old ones) or, if the loop saw the pin high twice across the stall, a stuck-pin
// trip. It needed a flash write AND a pulse in flight, which is why it was intermittent, why it
// followed setting changes and zeroing, and why a run left alone past the first save never did it.
//
// Now: the interrupt is allocated with ESP_INTR_FLAG_IRAM, the ISR only writes GPIO registers and
// calls IRAM-resident timer functions, and every variable it touches lives in DRAM. Flash writes no
// longer delay an edge at all. The alarm is also absolute (the counter free-runs and each edge adds
// its own span to the last alarm) rather than "reset the counter and count to N", so the period no
// longer drifts by the ISR's own latency every cycle, and the edge times are exact - the scope
// trigger and the pulse-shape fold use them.

static hw_timer_t* gTimer = nullptr;
static portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;
static volatile int gPin = -1;
static volatile uint32_t gMask = 0;          // GPIO bit within its out register
static volatile bool gHiBank = false;        // GPIO 32+ live in the second out register
static volatile int gPhase = 0;              // 0 = next edge turns the coil on
static volatile uint32_t gOnUs = 0, gOffUs = 0;
static volatile uint32_t gPendOn = 0, gPendOff = 0;
static volatile bool gPend = false;
static volatile bool gRun = false;
static volatile int gLevel = 0;
static volatile uint64_t gNext = 0;          // next alarm, timer ticks (1 us)
static volatile uint64_t gRiseTg = 0;        // latest rising edge, timer ticks
static volatile uint64_t gSinceTg = 0;       // when the current on/off pair took effect, timer ticks
static volatile uint32_t gCurOn = 0, gCurPer = 0;   // the pair the latest rising edge started
static volatile uint32_t gCycles = 0;
static volatile int64_t gTgOff = 0;          // esp_timer us = timer ticks + gTgOff

static inline void IRAM_ATTR gateHigh() {
  if (gHiBank) REG_WRITE(GPIO_OUT1_W1TS_REG, gMask);
  else REG_WRITE(GPIO_OUT_W1TS_REG, gMask);
}
static inline void IRAM_ATTR gateLow() {
  if (gHiBank) REG_WRITE(GPIO_OUT1_W1TC_REG, gMask);
  else REG_WRITE(GPIO_OUT_W1TC_REG, gMask);
}

// Two phases, each edge schedules the next one at an absolute counter value. The pending values
// load at the ON edge, so a mid-cycle change never shortens or stretches the pulse already in flight.
static void IRAM_ATTR coilIsr() {
  if (!gRun || gPin < 0) return;
  portENTER_CRITICAL_ISR(&gMux);
  uint64_t edge = gNext;
  if (gPhase == 0) {
    if (gPend) { gOnUs = gPendOn; gOffUs = gPendOff; gPend = false; gSinceTg = edge; }
    gateHigh();
    gLevel = 1;
    gRiseTg = edge;
    gCurOn = gOnUs;
    gCurPer = gOnUs + gOffUs;
    gCycles = gCycles + 1;
    gNext = edge + (gOnUs < 1 ? 1 : gOnUs);
    gPhase = 1;
  } else {
    gateLow();
    gLevel = 0;
    gNext = edge + (gOffUs < 1 ? 1 : gOffUs);
    gPhase = 0;
  }
  // Never schedule into the past: if something held interrupts off past the next edge, re-anchor a
  // couple of ticks out instead of waiting on an alarm the counter has already passed.
  uint64_t now = timer_group_get_counter_value_in_isr(COIL_TG, COIL_TI);
  if (gNext <= now + 2) gNext = now + 2;
  timer_group_set_alarm_value_in_isr(COIL_TG, COIL_TI, gNext);
  portEXIT_CRITICAL_ISR(&gMux);
}

static void forceLow() {
  int pin = gPin;
  if (pin < 0) return;
  gpio_set_pull_mode((gpio_num_t)pin, GPIO_PULLDOWN_ONLY);   // keeps the gate off even if the pin floats
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  gLevel = 0;
}

// The timer counts APB/80 and esp_timer counts the system timer; both come off the same crystal, so
// one offset maps between them for good. Taken a few times, keeping the tightest bracket.
static void syncClocks() {
  if (!gTimer) return;
  int64_t best = INT64_MAX, off = gTgOff;
  for (int i = 0; i < 4; i++) {
    int64_t a = esp_timer_get_time();
    uint64_t c = timerRead(gTimer);
    int64_t b = esp_timer_get_time();
    if (b - a < best) { best = b - a; off = (a + b) / 2 - (int64_t)c; }
  }
  gTgOff = off;
}

void coilInit(int pin) {
  coilOff();
  gPin = pin;
  if (pin >= 32) { gHiBank = true; gMask = 1u << (pin - 32); }
  else { gHiBank = false; gMask = 1u << pin; }
  forceLow();
  if (!gTimer) {
    gTimer = timerBegin(COIL_TIMER_NUM, 80, true);         // 80 MHz / 80 = 1 us per tick, free-running
    timerAttachInterruptFlag(gTimer, &coilIsr, false, ESP_INTR_FLAG_IRAM);
  }
  syncClocks();
}

void coilOff() {
  gRun = false;
  if (gTimer) timerAlarmDisable(gTimer);
  forceLow();
  portENTER_CRITICAL(&gMux);
  gPhase = 0;
  gPend = false;
  gLevel = 0;
  portEXIT_CRITICAL(&gMux);
}

void coilSet(float hz, float pwUs) {
  uint32_t per = mg::periodUs(hz);
  if (gPin < 0 || !gTimer || per == 0 || pwUs < 0.5f) { coilOff(); return; }
  uint32_t on = (uint32_t)(pwUs + 0.5f);
  // Last line of defence: the gate always gets an off phase. A pulse as long as its own period
  // would be DC through the coil, which no slider position is allowed to mean. Callers clamp with
  // mg::pwFor(); this catches anything that still arrives too wide.
  if (on >= per) on = per - 1;
  if (on < 1) on = 1;
  uint32_t off = per - on;
  if (gRun) {                                         // already pulsing: swap at the next ON edge
    portENTER_CRITICAL(&gMux);
    gPendOn = on;
    gPendOff = off;
    gPend = true;
    portEXIT_CRITICAL(&gMux);
    return;
  }
  syncClocks();
  uint64_t now = timerRead(gTimer);
  portENTER_CRITICAL(&gMux);
  gOnUs = on;
  gOffUs = off;
  gPend = false;
  gPhase = 0;
  gLevel = 0;
  gNext = now + 20;                                   // first edge right away
  gSinceTg = gNext;
  gRiseTg = gNext;
  gCurOn = on;
  gCurPer = per;
  portEXIT_CRITICAL(&gMux);
  timerAlarmWrite(gTimer, gNext, false);
  gRun = true;
  timerAlarmEnable(gTimer);
}

bool coilRunning() { return gRun; }
int coilLevel() { return gLevel; }
uint32_t coilOnUs() { return gPend ? gPendOn : gOnUs; }
uint32_t coilPeriodUs() {
  if (!gRun) return 0;
  uint32_t on = gPend ? gPendOn : gOnUs, off = gPend ? gPendOff : gOffUs;
  return on + off;
}
int64_t coilPeriodT0() {
  portENTER_CRITICAL(&gMux);
  int64_t t = (int64_t)gRiseTg + gTgOff;
  portEXIT_CRITICAL(&gMux);
  return t;
}
uint32_t coilCycles() { return gCycles; }

bool coilEdge(CoilEdge* e) {
  if (!gRun) return false;
  portENTER_CRITICAL(&gMux);
  e->riseUs = (int64_t)gRiseTg + gTgOff;
  e->sinceUs = (int64_t)gSinceTg + gTgOff;
  e->onUs = gCurOn;
  e->perUs = gCurPer;
  e->cycles = gCycles;
  bool live = gRun && gCycles > 0 && gCurPer > 0;
  portEXIT_CRITICAL(&gMux);
  return live;
}

bool coilStuck(int64_t nowUs) {
  portENTER_CRITICAL(&gMux);
  bool high = gRun && gLevel == 1;
  int64_t rise = (int64_t)gRiseTg + gTgOff;
  uint32_t on = gCurOn;
  portEXIT_CRITICAL(&gMux);
  return high && nowUs - rise > (int64_t)on + 20000;
}
