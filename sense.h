// sense.h - the measurement engine: two 49E hall sensors and two 10k NTC thermistors on one
// continuous DMA ADC scan, with waveform rings, running statistics, a phase-folded pulse profile
// and an FFT of the center-sensor field.
//
// All four inputs are ADC1 (GPIO 1-9 on the XIAO ESP32-S3), sampled round-robin by the DMA
// controller so Wi-Fi (which owns ADC2) never steals the SAR. Each hall channel is decimated 2:1
// in volts, converted to microtesla against its stored zero, and kept in a PSRAM ring for the
// scope. Thermistors are averaged hard (they move at seconds, not microseconds).
#pragma once
#include <stdint.h>

struct SenseCal {
  float sens5c, sens5p;   // hall mV per gauss at 5 V (center, perimeter)
  float vcc;              // hall supply
  bool ratio;             // ratiometric sensitivity
  float beta, r0k, rsk;   // NTC beta, R at 25 C (k), series resistor (k)
  bool top;               // NTC wired to 3V3 (true) or to GND (false)
};

struct SenseSnap {
  // Field right now (center, perimeter), microtesla, zero-subtracted, 300 Hz smoothed.
  float b0, b1;
  float db;               // dB/dt at the center, mT/s (4-tap difference, 500 Hz smoothed)
  // Since the last clear: absolute peaks, cumulative mean and RMS, and the window length.
  float bMax0, bMax1, dbMax;
  float bAvg0, bAvg1, bRms0;
  float statSec;
  float v0, v1;           // raw sensor volts (never zeroed)
  bool zeroed0, zeroed1;
  bool clip;              // either hall input touched an ADC rail in the last 0.2 s
  float fs;               // measured processed rate per hall channel, Hz
  // Thermistors, Fahrenheit. ok = probe answers (not stuck at a rail).
  float tC, tF;
  float dtdtC, dtdtF;     // degF per minute, over a 15-45 s lookback
  bool okC, okF;
  // Measured-frequency readout from the FFT of the center sensor.
  float pkHz, pkAmp, pkSnr;
  uint32_t seq;
};

// Claims the four ADC pins and starts the DMA controller plus the two tasks. Call once from setup;
// call again (same function) after a pin change - it restarts the controller cleanly.
bool senseBegin(int hallC, int hallP, int thC, int thF);
void senseSetCal(const SenseCal& cal);

void senseZero(int which);        // 0 center, 1 perimeter, 2 both: average 0.25 s and store as the zero
void senseZeroClear(int which);
enum { SENSE_CLR_MAX = 1, SENSE_CLR_AVG = 2, SENSE_CLR_ALL = 3 };
void senseClearStats(uint8_t bits);

void senseCopy(SenseSnap& out);

// Scope data: the last spanMs of channel 0 (B center), 1 (B perimeter) or 2 (dB/dt center),
// min/max-decimated into at most maxBins bins so pulse edges survive. Returns the bin count.
int senseWave(int ch, uint32_t spanMs, int maxBins, float* mn, float* mx, float* fsOut);

// One-sided spectrum of the center field, already capped to <= maxBins (max-preserving). µT per bin.
int senseSpec(float* out, int maxBins, float* dfOut);

// Phase-folded average of the center field across the active pulse period: the pulse shape, built
// from many cycles. Returns bins (0 when the coil is off), the period and how many cycles are in it.
int senseFold(float* out, int maxBins, uint32_t* perUs, uint32_t* cycles);

void senseHold(bool hold);        // park the ADC and FFT for a firmware update
