// leds.h - the status RGB (common anode on 3V3) and the coil-rim LED.
//
// Rendered at 250 Hz from their own task so patterns stay smooth while the web server is busy.
// Language of the patterns (documented in the UI too):
//   amber breathe            joining the home network / online-update steps 1-3 (with 1-3 white blinks)
//   cyan breathe             firmware update in progress (local upload or OTA)
//   red->yellow->green       online update download progress, 1 Hz heartbeat dip
//   solid green              update installed (about to restart)
//   red N blinks, pause      online update failed, N = reason 1-5
//   soft breathe, dim        idle (steady when a phone is connected, drifting when alone)
//   slow fade pulsate        treatment running - fixed color, stage colors, or data feedback
//   amber double-blink       paused
//   green triple pulse       treatment finished
//   red blinks, N then gap   safety trip, N = reason (1 coil temp, 2 FET temp, 3 coil rate, 4 FET rate, 5 stuck pin, 6 probe)
// The coil LED mirrors the treatment envelope and the pause/trip blinks; it stays dark otherwise.
#pragma once
#include "config.h"

struct LedInput {
  mg::State state;
  mg::TripCode trip;
  int stage;           // active stage index, -1 when none
  double elapsed, total;
  uint8_t joinPhase;   // 1 while joining the home network at boot
  uint8_t updPhase, updPct, updErr;
  bool otaBusy;
  bool client;         // a page talked to the board in the last 10 s
  float dataVal;       // the selected feedback value (already picked by led.dataSrc)
};

void ledsApply(const mg::Pins& pins);                       // claim / re-claim the four pins, output dark
void ledsRender(const mg::Config& cfg, const LedInput& in, int64_t nowUs);
void ledsDark();
