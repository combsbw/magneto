// coil.h - the pulse generator: a 1 us hardware-timer ISR toggling the gate-driver pin.
//
// Frequency 0.1-250 Hz in 0.1 Hz steps, pulse width down to 1 us. Fail-off everywhere: the pin
// idles LOW with a pulldown, and every stop path forces it low before anything else happens.
//
// The ISR is IRAM-resident and registered with ESP_INTR_FLAG_IRAM, and it touches nothing but
// registers, IRAM functions and DRAM variables. That matters: an ordinary (flash-resident) timer
// interrupt is held off for the whole of every SPI-flash write - a settings save, a sensor zero, a
// saved setup - and if one of those landed inside a pulse the gate stayed high until the write
// finished, tens of milliseconds of near-DC through the coil. See coil.cpp.
#pragma once
#include <stdint.h>

// Claims the pin (forced low first) and prepares the 1 us timer. Call again on a pin change.
void coilInit(int pin);

// New drive. hz <= 0 or pw <= 0 parks the coil off. A pulse width at or past the period is cut to
// one microsecond short of it, so the gate always falls every cycle and DC through the coil is not
// reachable from any setting. Takes effect at the next period edge when already pulsing,
// immediately otherwise.
void coilSet(float hz, float pwUs);

// Stops pulsing and forces the pin low. Safe from any task; called before OTA, on stop/pause/trip.
void coilOff();

bool coilRunning();          // pulses are being produced
int coilLevel();             // gate level right now
uint32_t coilOnUs();         // active on-time of the current setting
uint32_t coilPeriodUs();     // active period (0 = off)
int64_t coilPeriodT0();      // esp_timer time of the latest rising edge (phase folding)
uint32_t coilCycles();       // rising edges since boot

// One consistent read of the pulse train, in esp_timer microseconds: the latest rising edge, the
// on-time and period that edge started, and when the current period took effect (samples older than
// that belong to a different pulse train). False when the coil is not pulsing.
struct CoilEdge {
  int64_t riseUs;
  uint32_t onUs, perUs;
  int64_t sinceUs;
  uint32_t cycles;
};
bool coilEdge(CoilEdge* e);

// Stuck-gate watchdog, judged from the ISR's own edge timestamps rather than from how often the
// main loop happens to look: true when the gate has been high for longer than its on-time plus a
// 20 ms margin, which only a dead ISR can produce.
bool coilStuck(int64_t nowUs);
