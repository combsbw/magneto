#include "sense.h"

#include <Arduino.h>
#include <Preferences.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <math.h>
#include <soc/sens_struct.h>
#include <string.h>

#include "coil.h"
#include "fft.h"
#include "filt.h"
#include "sclock.h"

// ---- geometry of the pipeline ----------------------------------------------------------------------------------------
static const uint32_t kAdcTry[] = {80000, 60000, 40000, 20000};  // total conversions/s across the 4-slot pattern
static const int kDecim = 2;          // hall: raw conversions averaged into one processed sample
static const int kDiffK = 4;          // dB/dt difference span, processed samples
static const int FOLD_N = 240;        // phase-profile bins
static const int FFT_DECIM = 5;       // processed -> FFT stream (about 2 kHz at the 80 kHz ladder step)
static const int SPEC_MAX = 256;
static const float SPEC_HZ = 300.f;   // publish the spectrum up to here
static const float HOLD_SEC = 0.25f;  // stats holdoff after any disturbance
// Zero: half a second is a whole number of cycles of both 50 Hz (25) and 60 Hz (30) mains, so hum
// averages out of the baseline instead of leaving a fraction of a cycle in it (0.25 s was 12.5
// cycles of 50 Hz).
static const float ZERO_SEC = 0.5f;
static const uint32_t ZERO_TIMEOUT_MS = 5000;   // a zero that cannot find enough quiet samples gives up
static const float LIVE_HZ = 200.f;   // live-stream buckets per second

// ---- state (written by the sense task, read under the mux) ------------------------------------------------------------
static portMUX_TYPE sMux = portMUX_INITIALIZER_UNLOCKED;
static SenseSnap sSnap = {};
static Preferences sPrefs;

static SenseCal sCal = {1.4f, 1.4f, 3.3f, true, 3950.f, 10.f, 10.f, false};
static volatile bool sCalDirty = false;
static SenseCal sCalPend;

static SenseFilt sFilt = {0, false, 60, 0, 0.f, 0.f};
static volatile bool sFiltDirty = false;
static SenseFilt sFiltPend;
static mg::Biquad sNotchCoef = {1.f, 0.f, 0.f, 0.f, 0.f};
static mg::BiquadState sNotchSt[2];
static mg::BandPass sBandCoef;
static mg::BandState sBandSt[2];

static int sGpio[4] = {1, 2, 3, 4};   // hallC, hallP, thC, thF
static int sChan[4] = {0, 1, 2, 3};
static bool sAdcUp = false;
static volatile bool sHold = false;
static volatile bool sRestart = false;
static volatile bool sBounce = false;
static float sLut[257];
static mg::SenseWatch sWatch;

// sample clock: processed-sample index -> esp_timer microseconds
static mg::SampleClock sClk;
static double sClkOff = 0, sClkTs = 100.0;    // published copy for the HTTP task, under sMux
static bool sClkOk = false;

// hall processing
static float sVz[2] = {1.65f, 1.65f};
static bool sZeroed[2] = {false, false};
static volatile uint8_t sZeroReq = 0;      // API -> task: bit 0 center, bit 1 perimeter
static volatile uint8_t sZeroClrReq = 0;
static uint8_t sZeroBusy = 0;              // task-owned: channels collecting a zero
static uint8_t sZeroFail = 0;
static uint32_t sZeroSeq = 0;
static double sZeroSum[2] = {0, 0};
static uint32_t sZeroN[2] = {0, 0};
static uint32_t sZeroAt[2] = {0, 0};
static volatile uint8_t sPersist = 0;      // zeros to write to NVS (the FFT task does it, off the sampler)
static uint8_t sPrime = 0;                 // reseed this channel's filters / smoothers on its next sample
static volatile uint8_t sClearBits = 0;
static uint32_t sHoldoff = 0;              // processed-sample index stats resume at
static uint32_t sProcIdx = 0;

static float sB[2] = {0, 0};               // latest processed field, µT
static float sBf[2] = {0, 0};              // display-smoothed for the tiles
static float sBr[2] = {0, 0};              // unfiltered (no notch / band) smoothed, for the magnet watch
static float sDb[2] = {0, 0}, sDbf[2] = {0, 0};
static float sDbHist[2][kDiffK] = {{0}};
static int sDbPos = 0;
static float sBMax[2] = {0, 0}, sDbMax[2] = {0, 0};
static double sAvgSum[2] = {0, 0}, sRmsSq = 0;
static uint32_t sStatN = 0;
static uint32_t sStatT0 = 0;
static bool sClip = false;
static uint32_t sClipAt = 0;

// waveform rings (PSRAM when available). Ring slot j holds processed sample j + 1.
static float* sRing[2] = {nullptr, nullptr};
static uint32_t sRingCap = 0;
static volatile uint32_t sRingW = 0;       // one write index: both rings advance together

// live stream: fixed-rate min/max buckets
struct LiveB { float mn[4], mx[4]; };      // B center, B perimeter, dB/dt center, dB/dt perimeter
static LiveB* sLive = nullptr;
static uint32_t sLiveCap = 0;
static volatile uint32_t sLiveW = 0;
static int sLiveK = 50;                    // processed samples per bucket
static int sLiveCnt = 0;
static LiveB sLiveAcc;
static float sLiveDtMs = 5.f;

// phase fold, both channels, over a window around the rising edge
static float sFold[2][FOLD_N];
static uint16_t sFoldHit[FOLD_N];
static uint32_t sFoldPer = 0, sFoldOn = 0;
static float sFoldWin = 0, sFoldPre = 0;   // us folded, and how much of it is before the edge
static uint32_t sFoldWinCur = 0;           // window the fold was armed with (0 = whole period)
static volatile uint32_t sFoldWinReq = 0;  // what the page last asked for
static uint32_t sFoldCyc0 = 0, sFoldCycNow = 0;
static volatile bool sFoldReset = false;
// per-block phase tracking against the coil's edges
static CoilEdge sEdge;
static bool sEdgeOk = false;
static float sPh = 0;                      // phase of the next processed sample after the rising edge, us
static float sTsF = 100.f;
static int32_t sPhSkip = 0;                // samples still older than the current pulse train
static bool sFoldLocked = false;

// FFT stream and published spectrum
static float* sFftRing = nullptr;
static uint32_t sFftCap = 0;
static volatile uint32_t sFftW = 0;
static int sFftN = 2048;        // transform size, set from the memory budget
static bool sPsram = false;
static int sFftDecim = 0;
static float sFftAcc = 0;
static float sSpec[SPEC_MAX];
static int sSpecN = 0;
static int sSpecNPrev = 0;   // bin count last published; averaging resets if this changes
static float sSpecDf = 0;
static volatile bool sSpecReset = false;
static float sPkHz = 0, sPkAmp = 0, sPkSnr = 0;
static float sProcHz = 0;                  // nominal/filter rate per hall channel (kept > 0 while configured)
static float sMeasHz = 0;                  // live measured rate per hall channel (0 when stalled)
static uint32_t sRate[4] = {0, 0, 0, 0};
static uint32_t sRateT0 = 0;

// thermistors: window accumulators and a 1 Hz history ring for dT/dt
static double sThSum[2] = {0, 0};
static uint32_t sThN[2] = {0, 0};
static uint32_t sThWinAt = 0;
static float sTempF[2] = {NAN, NAN};
static bool sThOk[2] = {false, false};
static float sDtdt[2] = {0, 0};
struct ThHist { float tF; uint32_t ms; };
static ThHist sThHist[2][48];
static int sThHistN[2] = {0, 0};
static uint32_t sThHistAt = 0;

static TaskHandle_t sTask = nullptr, sFftTask = nullptr;

// ---- conversions ------------------------------------------------------------------------------------------------------
static inline float lutVolts(int raw) {
  const int bucket = raw >> 4;
  const float frac = (float)(raw & 15) / 16.f;
  return sLut[bucket] + (sLut[bucket + 1] - sLut[bucket]) * frac;
}

static inline float sensVperG(int which) {
  float s = (which ? sCal.sens5p : sCal.sens5c) / 1000.f;
  if (sCal.ratio) s *= sCal.vcc / 5.f;
  return s > 1e-9f ? s : 1e-9f;
}

// volts -> microtesla against the stored zero (1 gauss = 100 µT)
static inline float hallUt(int which, float volts) {
  return (volts - sVz[which]) / sensVperG(which) * 100.f;
}

static float thermF(int which, float volts, bool* ok) {
  // Divider off 3.3 V: NTC to GND (default) puts the NTC voltage on the pin directly.
  const float vcc = 3.3f;
  float v = volts;
  if (v < 0.03f || v > vcc - 0.03f) { *ok = false; return NAN; }   // open or shorted probe
  float rk = sCal.top ? sCal.rsk * (vcc - v) / v : sCal.rsk * v / (vcc - v);
  if (!(rk > 0.01f && rk < 10000.f)) { *ok = false; return NAN; }
  float invT = 1.f / 298.15f + logf(rk / sCal.r0k) / sCal.beta;
  float k = 1.f / invT;
  *ok = true;
  return (k - 273.15f) * 9.f / 5.f + 32.f;
  (void)which;
}

// ---- ADC bring-up -----------------------------------------------------------------------------------------------------
static void recomputeFilters() {
  sNotchCoef = sFilt.notch ? mg::notchCoeffs((float)sFilt.notchHz, sProcHz, 12.f) : mg::Biquad{1.f, 0.f, 0.f, 0.f, 0.f};
  sBandCoef = mg::bandCoeffs(sFilt.hpHz, sFilt.lpHz, sProcHz);
}

static void liveSize() {
  int k = (int)lroundf(sProcHz / LIVE_HZ);
  if (k < 1) k = 1;
  sLiveK = k;
  sLiveCnt = 0;
  sLiveDtMs = sProcHz > 1.f ? 1000.f * (float)k / sProcHz : 5.f;
}

// ESP-IDF 4.4's sar_periph_ctrl_power_enable() (called from Wi-Fi / PM / PHY paths) resets
// force_xpd_sar back to FSM mode even while continuous ADC holds a power reference, and a subsequent
// stop/start with s_sar_power_on_cnt > 1 does not restore SAR_CTRL_LL_POWER_ON. Force it explicitly.
static inline void forceSarPowerOn() {
  SENS.sar_peri_clk_gate_conf.saradc_clk_en = 1;
  SENS.sar_power_xpd_sar.force_xpd_sar = 0x3;
}

static void adcStop() {
  if (!sAdcUp) return;
  adc_digi_stop();
  adc_digi_deinitialize();
  sAdcUp = false;
}

static bool adcStart() {
  adcStop();
  sClk.restart();
  esp_adc_cal_characteristics_t cal = {};
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &cal);
  for (int i = 0; i <= 256; i++) {
    const int raw = i == 256 ? 4095 : i * 16;
    sLut[i] = esp_adc_cal_raw_to_voltage(raw, &cal) / 1000.f;
  }
  uint32_t mask = 0;
  for (int i = 0; i < 4; i++) {
    sChan[i] = sGpio[i] - 1;                 // S3: ADC1_CH0 = GPIO1 ... ADC1_CH9 = GPIO10
    mask |= 1u << sChan[i];
  }
  for (uint32_t hz : kAdcTry) {
    // Size each GDMA descriptor near the 4092-byte single-descriptor limit (~12.8 ms per EOF across
    // all rate steps, always a multiple of the 16-byte 4-channel scan) so the 5-descriptor hardware
    // ring spans ~64 ms instead of 16 ms and rides through normal SPI-flash writes without stalling.
    const uint32_t fullFrame = hz >= 80000 ? 4080 : (hz >= 60000 ? 3072 : (hz >= 40000 ? 2048 : 1024));
    const uint32_t midFrame = fullFrame > 2048 ? 2048 : fullFrame;
    const uint32_t storeTry[3] = {16384, 8192, 4096};
    const uint32_t frameTry[3] = {fullFrame, midFrame, 1024};
    esp_err_t err = ESP_ERR_NO_MEM;
    for (int tier = 0; tier < 3 && err != ESP_OK; tier++) {
      adc_digi_init_config_t init = {};
      init.max_store_buf_size = storeTry[tier];
      init.conv_num_each_intr = frameTry[tier];
      init.adc1_chan_mask = mask;
      init.adc2_chan_mask = 0;
      err = adc_digi_initialize(&init);
    }
    if (err != ESP_OK) continue;

    adc_digi_pattern_config_t pat[4] = {};
    for (int i = 0; i < 4; i++) {
      pat[i].atten = ADC_ATTEN_DB_11;
      pat[i].channel = (uint8_t)sChan[i];
      pat[i].unit = 0;
      pat[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
    }
    adc_digi_configuration_t dig = {};
    dig.conv_limit_en = 0;
    dig.conv_limit_num = 255;
    dig.pattern_num = 4;
    dig.adc_pattern = pat;
    dig.sample_freq_hz = hz;
    dig.conv_mode = ADC_CONV_SINGLE_UNIT_1;
    dig.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
    if (adc_digi_controller_configure(&dig) != ESP_OK) { adc_digi_deinitialize(); continue; }
    if (adc_digi_start() != ESP_OK) { adc_digi_deinitialize(); continue; }
    forceSarPowerOn();
    sAdcUp = true;
    sProcHz = (float)hz / 4.f / (float)kDecim;   // refined by measurement every second
    sMeasHz = sProcHz;
    sClk.reset(1e6 / sProcHz);
    recomputeFilters();
    liveSize();
    Serial.printf("sense: ADC dma %u Hz total, ~%.0f Hz per hall channel\n", hz, sProcHz);
    return true;
  }
  Serial.println("sense: ADC did not start");
  return false;
}

// Re-arm APB_SARADC and its GDMA descriptor ring in place without freeing or re-allocating DMA /
// ringbuffer RAM: clears IDF 4.4's sticky ringbuf_overflow_flag, restores descriptor ownership after
// a flash-erase stall, and re-applies SAR power/clock settings after Wi-Fi PHY calibration.
static bool adcBounce() {
  if (!sAdcUp) return adcStart();
  sClk.restart();                         // samples were dropped: the index no longer maps to time
  adc_digi_stop();
  static uint8_t drain[512];
  uint32_t got = 0;
  while (adc_digi_read_bytes(drain, sizeof drain, &got, 0) != ESP_ERR_TIMEOUT && got > 0) {}
  if (adc_digi_start() != ESP_OK) {
    adc_digi_deinitialize();
    sAdcUp = false;
    return adcStart();
  }
  forceSarPowerOn();
  return true;
}

// ---- folding / zero helpers ---------------------------------------------------------------------------------------------
static void foldArm() {
  memset(sFold, 0, sizeof sFold);
  memset(sFoldHit, 0, sizeof sFoldHit);
  sFoldPer = sEdgeOk ? sEdge.perUs : 0;
  sFoldOn = sEdgeOk ? sEdge.onUs : 0;
  sFoldWinCur = sFoldWinReq;
  float per = (float)sFoldPer;
  if (sFoldWinCur > 0 && per > 0 && (float)sFoldWinCur < per) {
    sFoldWin = (float)sFoldWinCur;
    sFoldPre = sFoldWin * 0.1f;
  } else {
    sFoldWin = per;
    sFoldPre = 0;
  }
  sFoldCyc0 = sEdgeOk ? sEdge.cycles : 0;
  sFoldCycNow = sFoldCyc0;
}

// Once per DMA block: read the coil's edges, and work out the phase of the block's first sample.
static void blockPhase() {
  CoilEdge e;
  // the fold needs the clock locked; zero gating only needs it valid (its guard band is milliseconds)
  bool ok = coilEdge(&e) && sClk.valid && e.perUs > 0;
  sFoldLocked = sClk.locked();
  bool changed = ok != sEdgeOk || (ok && (e.perUs != sEdge.perUs || e.onUs != sEdge.onUs));
  sEdgeOk = ok;
  if (ok) sEdge = e;
  if (changed || sFoldReset || sFoldWinReq != sFoldWinCur) { sFoldReset = false; foldArm(); }
  if (!ok) { sFoldPer = 0; return; }
  sFoldCycNow = e.cycles;
  sTsF = (float)sClk.ts;
  double t = sClk.at((double)(sProcIdx + 1));       // the next processed sample
  double ph = fmod(t - (double)e.riseUs, (double)e.perUs);
  if (ph < 0) ph += e.perUs;
  sPh = (float)ph;
  double before = (double)e.sinceUs - t;            // samples older than this pulse train don't fold
  sPhSkip = before > 0 ? (int32_t)(before / sClk.ts) + 1 : 0;
}

// Is the sample at phase ph clear of the pulse and its decay (for zeroing with the coil running)?
static inline bool quietPhase(float ph) {
  if (!sEdgeOk) return !coilRunning();
  float on = (float)sEdge.onUs, per = (float)sEdge.perUs;
  float offT = per - on;
  float guard = offT * 0.5f < 3000.f ? offT * 0.5f : 3000.f;
  return ph >= on + guard;
}

// After a zero (or a zero clear) moves channel h's reference: nothing downstream may keep the old one.
static void afterZero(uint8_t bits) {
  sPrime |= bits;                                         // filters and smoothers reseed on the next sample
  sHoldoff = sProcIdx + (uint32_t)(sProcHz * HOLD_SEC);
  sClearBits |= SENSE_CLR_ALL;                            // stats measured against the old zero are void
  sFoldReset = true;                                      // so is the folded pulse
  sFftW = 0;                                              // and the FFT window (it restarts in ~2 s)
  sSpecReset = true;
  sZeroSeq++;
}

// ---- per-processed-sample work (runs ~10 kHz, keep it lean) -----------------------------------------------------------
static inline void statsAndRings(float b0, float b1) {
  sProcIdx++;
  // phase of this sample against the pulse train
  bool phOk = sEdgeOk;
  float ph = sPh;
  if (sEdgeOk) {
    if (sPhSkip > 0) { sPhSkip--; phOk = false; }
    float per = (float)sEdge.perUs;
    sPh += sTsF;
    if (sPh >= per) sPh -= per;
  }

  // scope rings
  if (sRing[0]) {
    uint32_t w = sRingW % sRingCap;
    sRing[0][w] = b0;
    sRing[1][w] = b1;
    sRingW++;
  }
  // dB/dt per channel: difference over kDiffK processed samples, scaled to per-second, then smoothed
  // (cutoff set by the filter strength; mode 0 is exactly the fixed 500 Hz this always used)
  const float bb[2] = {b0, b1};
  float d[2] = {0, 0};
  bool dOk = sProcIdx > (uint32_t)kDiffK && sProcHz > 1.f;
  float aDb = mg::emaAlpha(mg::filtDbHz(sFilt.mode), sProcHz);
  for (int h = 0; h < 2; h++) {
    float prev = sDbHist[h][sDbPos];
    sDbHist[h][sDbPos] = bb[h];
    if (dOk) {
      d[h] = (bb[h] - prev) * (sProcHz / (float)kDiffK) * 1e-3f;   // µT/s -> mT/s
      sDbf[h] = mg::emaStep(sDbf[h], d[h], aDb);
      sDb[h] = d[h];
    }
  }
  sDbPos = (sDbPos + 1) % kDiffK;
  // display smoothing (mode 0 is exactly the fixed 300 Hz this always used)
  float af = mg::emaAlpha(mg::filtFieldHz(sFilt.mode), sProcHz);
  sBf[0] = mg::emaStep(sBf[0], b0, af);
  sBf[1] = mg::emaStep(sBf[1], b1, af);

  // stats, held off briefly after a zero / pin / cal change so the step never latches as a peak
  if (sProcIdx >= sHoldoff && isfinite(b0) && isfinite(b1) && isfinite(sDbf[0]) && isfinite(sDbf[1])) {
    float a0 = fabsf(b0), a1 = fabsf(b1), ad0 = fabsf(sDbf[0]), ad1 = fabsf(sDbf[1]);
    if (a0 > sBMax[0]) sBMax[0] = a0;
    if (a1 > sBMax[1]) sBMax[1] = a1;
    if (ad0 > sDbMax[0]) sDbMax[0] = ad0;
    if (ad1 > sDbMax[1]) sDbMax[1] = ad1;
    sAvgSum[0] += b0;
    sAvgSum[1] += b1;
    sRmsSq += (double)b0 * (double)b0;
    sStatN++;
  }

  // live stream: min/max per bucket
  if (sLive) {
    const float v[4] = {b0, b1, d[0], d[1]};
    if (sLiveCnt == 0) {
      for (int k = 0; k < 4; k++) sLiveAcc.mn[k] = sLiveAcc.mx[k] = v[k];
    } else {
      for (int k = 0; k < 4; k++) {
        if (v[k] < sLiveAcc.mn[k]) sLiveAcc.mn[k] = v[k];
        if (v[k] > sLiveAcc.mx[k]) sLiveAcc.mx[k] = v[k];
      }
    }
    if (++sLiveCnt >= sLiveK) {
      sLive[sLiveW % sLiveCap] = sLiveAcc;
      sLiveW++;
      sLiveCnt = 0;
    }
  }

  // phase fold against the coil's own edge times (equivalent-time sampling)
  if (phOk && sFoldLocked && sFoldWin > 0) {
    float fp = ph + sFoldPre;
    float per = (float)sEdge.perUs;
    if (fp >= per) fp -= per;
    if (fp < sFoldWin) {
      int bin = (int)(fp * (float)FOLD_N / sFoldWin);
      if (bin >= 0 && bin < FOLD_N) {
        uint16_t n = sFoldHit[bin];
        if (n < 32) {
          n++;
          sFoldHit[bin] = n;
          sFold[0][bin] += (b0 - sFold[0][bin]) / (float)n;
          sFold[1][bin] += (b1 - sFold[1][bin]) / (float)n;
        } else {
          sFold[0][bin] += (b0 - sFold[0][bin]) * (1.f / 32.f);
          sFold[1][bin] += (b1 - sFold[1][bin]) * (1.f / 32.f);
        }
      }
    }
  }

  // FFT stream: box-decimate the center field
  sFftAcc += b0;
  if (++sFftDecim >= FFT_DECIM) {
    if (sFftRing) {
      sFftRing[sFftW % sFftCap] = sFftAcc / (float)FFT_DECIM;
      sFftW = sFftW + 1;
    }
    sFftAcc = 0;
    sFftDecim = 0;
  }
}

static void publishSnap() {
  uint32_t now = millis();
  SenseSnap s;
  s.b0 = sBf[0]; s.b1 = sBf[1];
  s.db = sDbf[0]; s.db1 = sDbf[1];
  s.bMax0 = sBMax[0]; s.bMax1 = sBMax[1]; s.dbMax = sDbMax[0]; s.dbMax1 = sDbMax[1];
  s.bAvg0 = sStatN ? (float)(sAvgSum[0] / sStatN) : 0;
  s.bAvg1 = sStatN ? (float)(sAvgSum[1] / sStatN) : 0;
  s.bRms0 = sStatN ? (float)sqrt(sRmsSq / sStatN) : 0;
  s.statSec = (now - sStatT0) / 1000.f;
  s.br0 = sBr[0]; s.br1 = sBr[1];
  s.v0 = sVz[0] + sBr[0] / 100.f * sensVperG(0);
  s.v1 = sVz[1] + sBr[1] / 100.f * sensVperG(1);
  s.zeroed0 = sZeroed[0]; s.zeroed1 = sZeroed[1];
  s.zBusy = sZeroBusy | sZeroReq;
  s.zFail = sZeroFail;
  s.zSeq = sZeroSeq;
  s.clip = sClip && (uint32_t)(now - sClipAt) < 200;
  s.fs = sWatch.stalled(now) ? 0.f : sMeasHz;
  s.clk = sClk.locked();
  s.tC = sTempF[0]; s.tF = sTempF[1];
  s.dtdtC = sDtdt[0]; s.dtdtF = sDtdt[1];
  s.okC = sThOk[0]; s.okF = sThOk[1];
  portENTER_CRITICAL(&sMux);
  s.pkHz = sPkHz; s.pkAmp = sPkAmp; s.pkSnr = sPkSnr;
  uint32_t seq = sSnap.seq + 1;
  sSnap = s;
  sSnap.seq = seq;
  portEXIT_CRITICAL(&sMux);
}

static void thermWindow(uint32_t now) {
  for (int t = 0; t < 2; t++) {
    bool hasSamples = sThN[t] > 10;
    bool ok = false;
    float f = NAN;
    if (hasSamples) f = thermF(t, (float)(sThSum[t] / sThN[t]), &ok);
    sThSum[t] = 0;
    sThN[t] = 0;
    sWatch.thermWindow(t, now, hasSamples, ok, f, sTempF[t], sThOk[t]);
    if (!sThOk[t]) {
      sThHistN[t] = 0;
      sDtdt[t] = 0;
    }
  }
  // 1 Hz history for the rate of change
  if ((uint32_t)(now - sThHistAt) >= 1000) {
    sThHistAt = now;
    for (int t = 0; t < 2; t++) {
      if (!sThOk[t]) { sThHistN[t] = 0; sDtdt[t] = 0; continue; }
      ThHist* h = sThHist[t];
      int n = sThHistN[t];
      if (n >= 48) { memmove(h, h + 1, sizeof(ThHist) * 47); n = 47; }
      h[n] = ThHist{sTempF[t], now};
      sThHistN[t] = n + 1;
      // the oldest sample between 15 and 45 s back sets the slope
      sDtdt[t] = 0;
      for (int i = 0; i <= n; i++) {
        uint32_t age = now - h[i].ms;
        if (age >= 15000 && age <= 45000) {
          sDtdt[t] = (h[n].tF - h[i].tF) / ((float)age / 60000.f);
          break;
        }
      }
    }
  }
}

static void applyPendingCommands() {
  if (sCalDirty) {
    bool vccChanged = sCal.vcc != sCalPend.vcc || sCal.ratio != sCalPend.ratio;
    sCal = sCalPend;
    sCalDirty = false;
    if (vccChanged && !sZeroed[0]) sVz[0] = sCal.vcc * 0.5f;
    if (vccChanged && !sZeroed[1]) sVz[1] = sCal.vcc * 0.5f;
    sHoldoff = sProcIdx + (uint32_t)(sProcHz * HOLD_SEC);
  }
  if (sFiltDirty) {
    sFilt = sFiltPend;
    sFiltDirty = false;
    recomputeFilters();
    sPrime |= 3;                       // new filters start from the signal as it is, not from zero
    sHoldoff = sProcIdx + (uint32_t)(sProcHz * HOLD_SEC);
    sFoldReset = true;
  }
  // zero requests: start collecting (the sums are reset here, in the task that owns them)
  portENTER_CRITICAL(&sMux);
  uint8_t zr = sZeroReq;
  sZeroReq = 0;
  uint8_t zc = sZeroClrReq;
  sZeroClrReq = 0;
  portEXIT_CRITICAL(&sMux);
  uint32_t ms = millis();
  for (int i = 0; i < 2; i++) {
    if (!(zr & (1 << i))) continue;
    sZeroSum[i] = 0;
    sZeroN[i] = 0;
    sZeroAt[i] = ms;
    sZeroBusy |= (uint8_t)(1 << i);
    sZeroFail &= (uint8_t)~(1 << i);
  }
  if (zc) {
    for (int i = 0; i < 2; i++) {
      if (!(zc & (1 << i))) continue;
      sVz[i] = sCal.vcc * 0.5f;
      sZeroed[i] = false;
      sZeroBusy &= (uint8_t)~(1 << i);
      sZeroFail &= (uint8_t)~(1 << i);
    }
    sPersist |= zc;
    afterZero(zc);
  }
  // a zero that could not find its quiet samples in time gives up rather than hang
  for (int i = 0; i < 2; i++) {
    if (!(sZeroBusy & (1 << i)) || (uint32_t)(ms - sZeroAt[i]) < ZERO_TIMEOUT_MS) continue;
    sZeroBusy &= (uint8_t)~(1 << i);
    sZeroFail |= (uint8_t)(1 << i);
    sZeroSeq++;
  }
  portENTER_CRITICAL(&sMux);
  uint8_t clr = sClearBits;
  sClearBits = 0;
  portEXIT_CRITICAL(&sMux);
  if (clr) {
    if (clr & SENSE_CLR_MAX) { sBMax[0] = sBMax[1] = 0; sDbMax[0] = sDbMax[1] = 0; }
    if (clr & SENSE_CLR_AVG) { sAvgSum[0] = sAvgSum[1] = 0; sRmsSq = 0; sStatN = 0; sStatT0 = millis(); }
    if (clr == SENSE_CLR_ALL) { sFoldReset = true; sSpecReset = true; }
    publishSnap();                     // the command's reply reads this, not a 50 ms-old snapshot
  }
}

// One hall channel's decimated sample: zero collection on the raw volts, then volts -> µT, then the
// signal-cleaning chain, with the per-channel reseed after a zero or a filter change.
static inline float hallSample(int h, float volts, float ph, bool phOk) {
  if (sZeroBusy & (1 << h)) {
    bool quiet = coilRunning() ? (phOk && quietPhase(ph)) : true;
    if (quiet) {
      sZeroSum[h] += volts;
      sZeroN[h]++;
    }
    uint32_t need = (uint32_t)(sProcHz * ZERO_SEC);
    if (need < 16) need = 16;
    if (sZeroN[h] >= need) {
      sVz[h] = (float)(sZeroSum[h] / sZeroN[h]);
      sZeroed[h] = true;
      sZeroBusy &= (uint8_t)~(1 << h);
      sPersist |= (uint8_t)(1 << h);
      afterZero((uint8_t)(1 << h));
    }
  }
  float raw = hallUt(h, volts);
  float b = raw;
  if (sPrime & (1 << h)) {
    mg::biquadPrime(sNotchSt[h], raw);
    sBandSt[h].primed = false;
  }
  if (sFilt.notch) b = mg::biquadStep(sNotchCoef, sNotchSt[h], b);
  b = mg::bandStep(sBandCoef, sBandSt[h], b);
  float af = mg::emaAlpha(mg::filtFieldHz(0), sProcHz);
  if (sPrime & (1 << h)) {
    sPrime &= (uint8_t)~(1 << h);
    sBr[h] = raw;
    sBf[h] = b;
    for (int k = 0; k < kDiffK; k++) sDbHist[h][k] = b;   // no dB/dt spike from the old reference
    sDbf[h] = 0;
  } else {
    sBr[h] = mg::emaStep(sBr[h], raw, af);
  }
  return b;
}

static void senseTask(void*) {
  static uint8_t buf[4096];
  double sumV[2] = {0, 0};
  int nDec[2] = {0, 0};
  bool clipRun = false;
  uint32_t bootMs = millis();
  sWatch.reset(bootMs);
  sRateT0 = bootMs;
  sStatT0 = bootMs;
  sThWinAt = bootMs;
  for (;;) {
    if (sHold) {
      uint32_t ms = millis();
      sWatch.reset(ms);
      sRateT0 = ms;
      sThWinAt = ms;
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    if (sRestart || !sAdcUp) {
      sRestart = false;
      sBounce = false;
      sumV[0] = sumV[1] = 0;
      nDec[0] = nDec[1] = 0;
      adcStart();
      uint32_t ms = millis();
      sWatch.lastAttemptMs = ms;
      if (!sAdcUp) {
        sMeasHz = 0;
        thermWindow(ms);
        publishSnap();
        vTaskDelay(pdMS_TO_TICKS(500));
        continue;
      }
    }
    if (sBounce) {
      sBounce = false;
      sumV[0] = sumV[1] = 0;
      nDec[0] = nDec[1] = 0;
      if (!adcBounce()) {
        sMeasHz = 0;
        thermWindow(millis());
        publishSnap();
        vTaskDelay(pdMS_TO_TICKS(200));
        continue;
      }
      sWatch.lastAttemptMs = millis();
    }
    applyPendingCommands();

    uint32_t n = 0;
    esp_err_t err = adc_digi_read_bytes(buf, sizeof buf, &n, 15);
    bool haveData = (err == ESP_OK || err == ESP_ERR_INVALID_STATE) && n >= sizeof(adc_digi_output_data_t);
    if (err == ESP_ERR_INVALID_STATE) {
      // IDF 4.4 keeps ringbuf_overflow_flag latched until adc_digi_start() runs again, even while
      // copying valid bytes into buf. Consume what arrived, then soft-bounce on the next pass.
      sBounce = true;
    }

    if (haveData) {
      int64_t tEnd = esp_timer_get_time();
      blockPhase();
      int validSamples = 0;
      float lastPh = sPh;
      bool lastPhOk = sEdgeOk && sPhSkip <= 0;

      for (uint32_t i = 0; i + sizeof(adc_digi_output_data_t) <= n; i += sizeof(adc_digi_output_data_t)) {
        const adc_digi_output_data_t* p = (const adc_digi_output_data_t*)&buf[i];
        int raw = (int)p->type2.data;
        if (raw < 0 || raw > 4095) continue;
        int ch = (int)p->type2.channel;
        int slot = -1;
        for (int k = 0; k < 4; k++) if (sChan[k] == ch) { slot = k; break; }
        if (slot < 0) continue;
        validSamples++;
        sRate[slot]++;
        float volts = lutVolts(raw);

        if (slot >= 2) {                                   // thermistor: just accumulate
          sThSum[slot - 2] += volts;
          sThN[slot - 2]++;
          continue;
        }

        // hall channel
        if (raw <= 1 || raw >= 4094) clipRun = true;
        int h = slot;
        sumV[h] += volts;
        if (++nDec[h] >= kDecim) {
          float v = (float)(sumV[h] / nDec[h]);
          sumV[h] = 0;
          nDec[h] = 0;
          if (h == 0) {
            lastPh = sPh;                                  // the phase this center sample is about to take
            lastPhOk = sEdgeOk && sPhSkip <= 0;
            sB[0] = hallSample(0, v, lastPh, lastPhOk);
            statsAndRings(sB[0], sB[1]);
          } else {
            sB[1] = hallSample(1, v, lastPh, lastPhOk);   // the perimeter sample sits next to it
          }
        }
      }
      if (validSamples > 0) {
        sWatch.onData(millis());
        sClk.onBlock((double)sProcIdx, (double)tEnd);
        portENTER_CRITICAL(&sMux);
        sClkOff = sClk.off;
        sClkTs = sClk.ts;
        sClkOk = sClk.locked();
        portEXIT_CRITICAL(&sMux);
      }
      if (clipRun) { sClip = true; sClipAt = millis(); clipRun = false; }
    } else if (err != ESP_ERR_TIMEOUT) {
      mg::SenseAct act = sWatch.onReadErr(millis());
      if (act == mg::SENSE_ACT_BOUNCE) sBounce = true;
      else if (act == mg::SENSE_ACT_REINIT) sRestart = true;
    }

    uint32_t now = millis();
    mg::SenseAct stallAct = sWatch.checkStall(now);
    if (stallAct == mg::SENSE_ACT_BOUNCE) {
      Serial.println("sense: ADC DMA stalled, bouncing");
      sBounce = true;
    } else if (stallAct == mg::SENSE_ACT_REINIT) {
      Serial.println("sense: ADC DMA stalled repeatedly, reinitializing");
      sRestart = true;
    }
    if ((uint32_t)(now - sThWinAt) >= 250) { sThWinAt = now; thermWindow(now); }
    if ((uint32_t)(now - sRateT0) >= 1000) {
      float dtMs = (float)(uint32_t)(now - sRateT0);
      sMeasHz = dtMs > 0.f ? (float)sRate[0] * 1000.f / dtMs / (float)kDecim : 0.f;
      if (sRate[0] > 0) {
        sProcHz = sMeasHz;
        recomputeFilters();     // coefficients depend on sProcHz, which just moved
        sLiveDtMs = 1000.f * (float)sLiveK / sProcHz;
      }
      sRate[0] = sRate[1] = sRate[2] = sRate[3] = 0;
      sRateT0 = now;
    }
    static uint32_t pubAt = 0;
    if ((uint32_t)(now - pubAt) >= 50) { pubAt = now; publishSnap(); }
  }
}

// ---- FFT task (also writes zeros to NVS: a flash write must never sit inside the sampler) -------------------------
static void persistZeros() {
  portENTER_CRITICAL(&sMux);
  uint8_t bits = sPersist;
  sPersist = 0;
  float vz[2] = {sVz[0], sVz[1]};
  bool z[2] = {sZeroed[0], sZeroed[1]};
  portEXIT_CRITICAL(&sMux);
  for (int i = 0; i < 2; i++) {
    if (!(bits & (1 << i))) continue;
    sPrefs.putFloat(i ? "vz1" : "vz0", vz[i]);
    sPrefs.putBool(i ? "z1" : "z0", z[i]);
  }
  if (bits) senseBounce();      // the flash write may have stalled the DMA ring; re-arm it cleanly
}

static void fftTask(void*) {
  const int N = sFftN;
  float* re = (float*)heap_caps_malloc(N * sizeof(float), MALLOC_CAP_SPIRAM);
  float* im = (float*)heap_caps_malloc(N * sizeof(float), MALLOC_CAP_SPIRAM);
  if (!re) re = (float*)malloc(N * sizeof(float));
  if (!im) im = (float*)malloc(N * sizeof(float));
  bool fftOk = re && im && sFftRing;
  if (!fftOk) Serial.println("sense: no memory for the FFT");
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(fftOk ? 600 : 300));
    if (sPersist && !sHold) persistZeros();
    if (sHold || !fftOk) continue;
    uint32_t w = sFftW;
    if (w < (uint32_t)N) continue;
    for (int i = 0; i < N; i++) re[i] = sFftRing[(w - N + i) % sFftCap];
    if (sFftW < w || sFftW - w > sFftCap - N) continue;      // reset or lapped while copying; try again
    memset(im, 0, N * sizeof(float));
    float fs = sProcHz / (float)FFT_DECIM;
    if (fs < 10.f) continue;
    mg::fftRemoveDc(re, N);
    mg::fftHann(re, N);
    mg::fftRun(re, im, N);
    int bins = N / 2;
    float df = fs / (float)N;
    // magnitudes in place, then the peak and the published (max-decimated) spectrum
    for (int i = 0; i < bins; i++) re[i] = sqrtf(re[i] * re[i] + im[i] * im[i]);
    mg::FftPeak pk;
    bool got = mg::fftPeak(re, bins, df, 0.3f, SPEC_HZ, N, &pk);
    int want = (int)(SPEC_HZ / df);
    if (want > bins) want = bins;
    int outN = want > SPEC_MAX ? SPEC_MAX : want;
    // Spectrum averaging (specAvg) smooths frame to frame to pull a tone out of noise; peak detection
    // above already ran on this frame's own raw magnitude, so it stays exactly as responsive as
    // before regardless of this setting. A bin-count change (sProcHz drifted enough to move `want`)
    // or a reset (zero, clear all) invalidates the previous frame's averaging, so this frame reseeds.
    bool reseed = outN != sSpecNPrev || sSpecReset;
    sSpecReset = false;
    float alpha = mg::filtSpecAlpha(sFilt.specAvg);
    portENTER_CRITICAL(&sMux);
    for (int o = 0; o < outN; o++) {
      int a = (int)((int64_t)o * want / outN), b = (int)((int64_t)(o + 1) * want / outN);
      float m = 0;
      for (int i = a; i < b && i < bins; i++) if (re[i] > m) m = re[i];
      float nv = 4.f * m / (float)N;      // amplitude-corrected like the peak
      sSpec[o] = (reseed || alpha >= 0.999f) ? nv : sSpec[o] + alpha * (nv - sSpec[o]);
    }
    sSpecN = outN;
    sSpecNPrev = outN;
    sSpecDf = df * (float)want / (float)outN;
    if (got) { sPkHz = pk.hz; sPkAmp = pk.amp; sPkSnr = pk.snr; }
    else { sPkHz = 0; sPkAmp = 0; sPkSnr = 0; }
    portEXIT_CRITICAL(&sMux);
  }
}

// ---- public API -------------------------------------------------------------------------------------------------------
bool senseBegin(int hallC, int hallP, int thC, int thF) {
  static bool once = false;
  sGpio[0] = hallC; sGpio[1] = hallP; sGpio[2] = thC; sGpio[3] = thF;
  if (once) { sRestart = true; return true; }
  once = true;

  sPrefs.begin("mag-sense", false);
  sVz[0] = sPrefs.getFloat("vz0", sCal.vcc * 0.5f);
  sVz[1] = sPrefs.getFloat("vz1", sCal.vcc * 0.5f);
  sZeroed[0] = sPrefs.getBool("z0", false);
  sZeroed[1] = sPrefs.getBool("z1", false);
  for (int i = 0; i < 2; i++) {
    if (!isfinite(sVz[i]) || sVz[i] < 0.05f || sVz[i] > 3.5f) {
      sVz[i] = sCal.vcc * 0.5f;
      sZeroed[i] = false;
    }
  }

  // Buffer budget. With PSRAM the scope keeps minutes of history and the FFT runs 4096 points.
  // Without it every byte competes with Wi-Fi in internal RAM, so the rings shrink hard and the
  // transform drops to 2048 points (still ~1 Hz resolution over a 1 s window).
  sRing[0] = sRing[1] = nullptr;
  sRingCap = 65536;
  for (int i = 0; i < 2; i++) sRing[i] = (float*)heap_caps_malloc(sRingCap * sizeof(float), MALLOC_CAP_SPIRAM);
  sPsram = sRing[0] && sRing[1];
  if (!sPsram) {
    for (int i = 0; i < 2; i++) { free(sRing[i]); sRing[i] = nullptr; }
    sRingCap = 3072;
    for (int i = 0; i < 2; i++) sRing[i] = (float*)malloc(sRingCap * sizeof(float));
  }
  sFftN = sPsram ? 4096 : 2048;
  sFftCap = sPsram ? 8192 : 2048;
  sFftRing = (float*)heap_caps_malloc(sFftCap * sizeof(float), MALLOC_CAP_SPIRAM);
  if (!sFftRing) sFftRing = (float*)malloc(sFftCap * sizeof(float));
  // live stream: ~5 s of buckets with PSRAM, ~1.3 s without (the page asks every few hundred ms)
  sLiveCap = sPsram ? 1024 : 256;
  sLive = (LiveB*)heap_caps_malloc(sLiveCap * sizeof(LiveB), MALLOC_CAP_SPIRAM);
  if (!sLive) sLive = (LiveB*)malloc(sLiveCap * sizeof(LiveB));
  if (!sLive) sLiveCap = 0;
  memset(sFold, 0, sizeof sFold);
  Serial.printf("sense: %s - scope %u samples/channel, FFT %d points, live %u buckets\n",
                sPsram ? "PSRAM" : "internal RAM only", (unsigned)sRingCap, sFftN, (unsigned)sLiveCap);

  if (!adcStart()) { /* the task keeps retrying */ }
  // Core 0 is where the Wi-Fi/LWIP driver tasks live, at a priority well above anything application
  // code should run at; a real-time ADC consumer sharing that core can be starved by radio traffic
  // (exactly what the page's own HTTP polling generates) long enough to overflow the digi ADC's DMA
  // ring. Both measurement tasks live on core 1 instead, alongside the HTTP/OTA loop and the LED
  // task, where the only contention is cooperative FreeRTOS tasks at sane priorities. senseTask runs
  // above ledTask (250 Hz, bounded per pass) so the sampler always wins; fftTask stays low priority -
  // its 4096-point transform is bounded and infrequent (every 600 ms) and may wait behind either.
  xTaskCreatePinnedToCore(senseTask, "sense", 8192, nullptr, 6, &sTask, 1);
  xTaskCreatePinnedToCore(fftTask, "fft", 6144, nullptr, 1, &sFftTask, 1);
  return true;
}

void senseSetCal(const SenseCal& cal) {
  sCalPend = cal;
  sCalDirty = true;
}

void senseSetFilt(const SenseFilt& filt) {
  sFiltPend = filt;
  sFiltDirty = true;
}

void senseZero(int which) {
  uint8_t bits = which == 2 ? 3 : (1 << (which & 1));
  portENTER_CRITICAL(&sMux);
  sZeroReq |= bits;
  portEXIT_CRITICAL(&sMux);
}

void senseZeroClear(int which) {
  uint8_t bits = which == 2 ? 3 : (1 << (which & 1));
  portENTER_CRITICAL(&sMux);
  sZeroClrReq |= bits;
  portEXIT_CRITICAL(&sMux);
  if (!sTask || sHold) return;
  uint32_t t0 = millis();
  while (sZeroClrReq && (uint32_t)(millis() - t0) < 150) vTaskDelay(1);
  t0 = millis();
  while ((sClearBits || sPrime) && (uint32_t)(millis() - t0) < 150) vTaskDelay(1);
}

void senseClearStats(uint8_t bits) {
  portENTER_CRITICAL(&sMux);
  sClearBits |= bits;
  portEXIT_CRITICAL(&sMux);
  if (!sTask || sHold) return;
  uint32_t t0 = millis();
  while (sClearBits && (uint32_t)(millis() - t0) < 150) vTaskDelay(1);
}

void senseCopy(SenseSnap& out) {
  portENTER_CRITICAL(&sMux);
  out = sSnap;
  portEXIT_CRITICAL(&sMux);
}

static inline float ringDb(int h, uint32_t j) {
  uint32_t k = j >= (uint32_t)kDiffK ? j - kDiffK : 0;
  return (sRing[h][j % sRingCap] - sRing[h][k % sRingCap]) * (sProcHz / (float)kDiffK) * 1e-3f;
}

int senseWave(int ch, uint32_t spanUs, bool trig, int maxBins, float* mn, float* mx, float* fsOut,
              float* x0Us, float* dxUs, bool* trigOk) {
  if (trigOk) *trigOk = false;
  if (!sRing[0] || sProcHz < 1.f) return 0;
  if (ch < 0 || ch > 3) ch = 0;
  if (fsOut) *fsOut = sProcHz;
  portENTER_CRITICAL(&sMux);
  double off = sClkOff, ts = sClkTs;
  bool clk = sClkOk;
  portEXIT_CRITICAL(&sMux);
  if (!(ts > 0)) ts = 1e6 / sProcHz;
  uint32_t w = sRingW;
  uint32_t nS = (uint32_t)((double)spanUs / ts + 0.5);
  if (nS < 4) nS = 4;
  uint32_t keep = sRingCap > 64 ? sRingCap - 64 : sRingCap;   // the writer may be overwriting the oldest
  if (nS > keep) nS = keep;
  if (nS > w) nS = w;
  if (nS < 4) return 0;
  uint32_t start = w - nS;
  double x0 = -(double)nS * ts;

  // Triggered: line the window up on the latest rising edge whose post-trigger part is all in.
  CoilEdge e;
  if (trig && clk && coilEdge(&e) && e.perUs > 0) {
    double tNew = off + (double)w * ts;              // ring slot w-1 holds processed sample w
    double span = (double)nS * ts, pre = span * 0.1;
    double latest = tNew - (span - pre);
    double edge = (double)e.riseUs;
    if (edge > latest) {
      double k = ceil((edge - latest) / (double)e.perUs);
      edge -= k * (double)e.perUs;
    }
    if (edge >= (double)e.sinceUs) {
      double idx = (edge - pre - off) / ts - 1.0;    // ring slot of the first sample
      double lowest = w > keep ? (double)(w - keep) : 0.0;
      if (idx >= lowest && idx + nS <= (double)w) {
        start = (uint32_t)floor(idx);
        x0 = off + ((double)start + 1.0) * ts - edge;
        if (trigOk) *trigOk = true;
      }
    }
  }

  int bins = (int)nS < maxBins ? (int)nS : maxBins;
  for (int o = 0; o < bins; o++) {
    uint32_t a = start + (uint32_t)((uint64_t)o * nS / bins);
    uint32_t b = start + (uint32_t)((uint64_t)(o + 1) * nS / bins);
    if (b <= a) b = a + 1;
    float lo = 1e30f, hi = -1e30f;
    for (uint32_t i = a; i < b; i++) {
      float v = ch >= 2 ? ringDb(ch - 2, i) : sRing[ch][i % sRingCap];
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    mn[o] = lo;
    mx[o] = hi;
  }
  if (x0Us) *x0Us = (float)x0;
  if (dxUs) *dxUs = (float)((double)nS * ts / bins);
  return bins;
}

int senseSpec(float* out, int maxBins, float* dfOut) {
  portENTER_CRITICAL(&sMux);
  int n = sSpecN < maxBins ? sSpecN : maxBins;
  for (int i = 0; i < n; i++) out[i] = sSpec[i];
  if (dfOut) *dfOut = sSpecDf;
  portEXIT_CRITICAL(&sMux);
  return n;
}

int senseFold(uint32_t winUs, float* c, float* p, int maxBins, uint32_t* perUs, uint32_t* onUs,
              uint32_t* cycles, float* winOut, float* preOut, int* filled) {
  if (winUs > 20000000) winUs = 20000000;
  sFoldWinReq = winUs;                   // the sampler re-arms the fold when this changes
  if (!sFoldPer || sFoldWinCur != winUs || !(sFoldWin > 0)) return 0;
  int n = FOLD_N < maxBins ? FOLD_N : maxBins;
  int got = 0;
  for (int i = 0; i < n; i++) {
    int k = (int)((int64_t)i * FOLD_N / n);
    if (sFoldHit[k]) { c[i] = sFold[0][k]; p[i] = sFold[1][k]; got++; }
    else { c[i] = NAN; p[i] = NAN; }
  }
  if (perUs) *perUs = sFoldPer;
  if (onUs) *onUs = sFoldOn;
  if (cycles) *cycles = sFoldCycNow - sFoldCyc0;
  if (winOut) *winOut = sFoldWin;
  if (preOut) *preOut = sFoldPre;
  if (filled) *filled = got;
  return n;
}

int senseLive(uint32_t from, int maxN, float* out, uint32_t* start, float* dtMs) {
  if (dtMs) *dtMs = sLiveDtMs;
  if (!sLive || maxN <= 0) { if (start) *start = sLiveW; return 0; }
  uint32_t w = sLiveW;
  uint32_t avail = w < sLiveCap - 8 ? w : sLiveCap - 8;    // the oldest few may be mid-overwrite
  if (from == 0 || from > w || w - from > avail) from = w - (avail < (uint32_t)maxN ? avail : (uint32_t)maxN);
  uint32_t n = w - from;
  if (n > (uint32_t)maxN) n = (uint32_t)maxN;
  for (uint32_t i = 0; i < n; i++) {
    const LiveB& b = sLive[(from + i) % sLiveCap];
    float* o = out + i * 8;
    for (int k = 0; k < 4; k++) { o[k * 2] = b.mn[k]; o[k * 2 + 1] = b.mx[k]; }
  }
  if (start) *start = from;
  return (int)n;
}

bool sensePsram() { return sPsram; }
int senseFftPoints() { return sFftN; }

void senseHold(bool hold) {
  sHold = hold;
  if (hold) {
    sBounce = false;
    adcStop();
  } else {
    sRestart = true;
  }
}

void senseBounce() {
  if (sTask && !sHold) sBounce = true;
}
