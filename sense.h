// sense.h - the measurement engine: two 49E hall sensors and two 10k NTC thermistors on one
// continuous DMA ADC scan, with waveform rings, running statistics, a phase-folded pulse profile,
// a live min/max stream and an FFT of the center-sensor field.
//
// All four inputs are ADC1 (GPIO 1-10 on the ESP32-S3), sampled round-robin by the DMA
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
  // Field right now (center, perimeter), microtesla, zero-subtracted, filtered, display-smoothed.
  float b0, b1;
  float db, db1;          // dB/dt at the center and at the perimeter, mT/s (4-tap difference, smoothed)
  // Since the last clear: absolute peaks, cumulative mean and RMS, and the window length.
  float bMax0, bMax1, dbMax, dbMax1;
  float bAvg0, bAvg1, bRms0;
  float statSec;
  float v0, v1;           // raw sensor volts (never zeroed)
  float br0, br1;         // field against the zero with no notch / band-pass (the magnet watch uses it)
  bool zeroed0, zeroed1;
  uint8_t zBusy;          // bit 0 center, bit 1 perimeter: a zero is still collecting
  uint8_t zFail;          // bits: the last zero on that channel could not collect a quiet baseline
  uint32_t zSeq;          // bumps every time a zero (or a zero clear) completes
  bool clip;              // either hall input touched an ADC rail in the last 0.2 s
  float fs;               // measured processed rate per hall channel, Hz
  bool clk;               // the sample clock is locked (fold, trigger and zero gating are live)
  // Thermistors, Fahrenheit. ok = probe answers (not stuck at a rail).
  float tC, tF;
  float dtdtC, dtdtF;     // degF per minute, over a 15-45 s lookback
  bool okC, okF;
  // Measured-frequency readout from the FFT of the center sensor.
  float pkHz, pkAmp, pkSnr;
  uint32_t seq;
};

// Signal cleaning: display smoothing strength, an optional mains-hum notch and an optional band-pass
// (both applied to both hall channels ahead of the ring/fold/FFT/live stream/stats, not just the
// display) and spectrum frame averaging. mode 0 / notch off / band off / specAvg 0 is bit-for-bit
// what the firmware did before any of this existed.
struct SenseFilt {
  uint8_t mode;      // 0 raw, 1 light, 2 medium, 3 heavy
  bool notch;
  uint8_t notchHz;   // 50 or 60
  uint8_t specAvg;   // 0..4
  float hpHz;        // band-pass high-pass corner, 0 = off
  float lpHz;        // band-pass low-pass corner, 0 = off
};

// Claims the four ADC pins and starts the DMA controller plus the two tasks. Call once from setup;
// call again (same function) after a pin change - it restarts the controller cleanly.
bool senseBegin(int hallC, int hallP, int thC, int thF);
void senseSetCal(const SenseCal& cal);
void senseSetFilt(const SenseFilt& filt);

// 0 center, 1 perimeter, 2 both. Collects 0.5 s of quiet field (a whole number of 50 and 60 Hz mains
// cycles) and stores it as the zero. With the coil running, samples inside a pulse and its decay are
// skipped, so the baseline is the field between pulses, not the pulse's average. Returns at once;
// SenseSnap.zBusy / zSeq say when it is done.
void senseZero(int which);
void senseZeroClear(int which);
enum { SENSE_CLR_MAX = 1, SENSE_CLR_AVG = 2, SENSE_CLR_ALL = 3 };
// Clears and waits (up to ~150 ms) until the sampler has done it and published fresh numbers, so the
// reply to the command already shows the cleared stats.
void senseClearStats(uint8_t bits);

void senseCopy(SenseSnap& out);

// Scope data, channel 0 B center, 1 B perimeter, 2 dB/dt center, 3 dB/dt perimeter, over spanUs.
// Min/max-decimated into at most maxBins bins so pulse edges survive; spans shorter than maxBins
// samples come back as the raw samples. trig = align the window on a coil rising edge (10 %
// pre-trigger) when the coil is pulsing. *x0Us is the time of the first bin relative to the trigger
// edge (or to "now" untriggered), *dxUs the spacing between bins. Returns the bin count.
int senseWave(int ch, uint32_t spanUs, bool trig, int maxBins, float* mn, float* mx, float* fsOut,
              float* x0Us, float* dxUs, bool* trigOk);

// One-sided spectrum of the center field, already capped to <= maxBins (max-preserving). µT per bin.
int senseSpec(float* out, int maxBins, float* dfOut);

// Phase-folded average of both fields: the pulse shape, built from many cycles with equivalent-time
// sampling against the coil's own edge times. winUs > 0 folds only that much time around the rising
// edge (10 % before it), at 240 bins across the window - 2 us per bin at a 480 us window - instead
// of spreading 240 bins over the whole period. Bins no sample has landed in yet are NaN.
// Returns bins (0 when the coil is off or the sample clock is not locked yet).
int senseFold(uint32_t winUs, float* c, float* p, int maxBins, uint32_t* perUs, uint32_t* onUs,
              uint32_t* cycles, float* winOut, float* preOut, int* filled);

// Live stream: fixed-rate min/max buckets (~200 per second) of B center, B perimeter, dB/dt center,
// dB/dt perimeter - 8 floats per bucket, in that order, min then max. from = the first bucket wanted
// (the previous call's *next); 0 or anything no longer in the ring returns the newest maxN.
// *start is the first bucket actually returned, *dtMs the bucket length.
int senseLive(uint32_t from, int maxN, float* out, uint32_t* start, float* dtMs);

void senseHold(bool hold);        // park the ADC and FFT for a firmware update
void senseBounce();               // soft stop/start the ADC DMA engine after flash or Wi-Fi activity

// How the pipeline was sized at boot (PSRAM present = long scope history and a 4096-point FFT).
bool sensePsram();
int senseFftPoints();
