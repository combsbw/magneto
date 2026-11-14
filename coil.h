// coil.h - the pulse generator: a 1 us hardware-timer ISR toggling the gate-driver pin.
//
// Frequency 0.1-250 Hz in 0.1 Hz steps, pulse width down to 1 us. Fail-off everywhere: the pin
// idles LOW with a pulldown, and every stop path forces it low before anything else happens.
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
int coilLevel();             // gate level right now (stuck-pin watchdog)
uint32_t coilOnUs();         // active on-time of the current setting
uint32_t coilPeriodUs();     // active period (0 = off)
int64_t coilPeriodT0();      // esp_timer time of the latest rising edge (phase folding)
uint32_t coilCycles();       // rising edges since boot
