#include "sense.h"

#include <Arduino.h>
#include <Preferences.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <math.h>
#include <string.h>

#include "coil.h"
#include "fft.h"

// ---- geometry of the pipeline ----------------------------------------------------------------------------------------
static const uint32_t kAdcTry[] = {80000, 60000, 40000, 20000};  // total conversions/s across the 4-slot pattern
static const int kDecim = 2;          // hall: raw conversions averaged into one processed sample
static const int kDiffK = 4;          // dB/dt difference span, processed samples
static const int FOLD_N = 240;        // phase-profile bins
static const int FFT_N = 4096;
static const int FFT_DECIM = 5;       // processed -> FFT stream (about 2 kHz at the 80 kHz ladder step)
static const int SPEC_MAX = 256;
static const float SPEC_HZ = 300.f;   // publish the spectrum up to here
static const float ZERO_SEC = 0.25f;  // zeroing average, and the stats holdoff after any disturbance

// ---- state (written by the sense task, read under the mux) ------------------------------------------------------------
static portMUX_TYPE sMux = portMUX_INITIALIZER_UNLOCKED;
static SenseSnap sSnap = {};
static Preferences sPrefs;

static SenseCal sCal = {1.4f, 1.4f, 3.3f, true, 3950.f, 10.f, 10.f, false};
static volatile bool sCalDirty = false;
static SenseCal sCalPend;

static int sGpio[4] = {1, 2, 3, 4};   // hallC, hallP, thC, thF
static int sChan[4] = {0, 1, 2, 3};
static bool sAdcUp = false;
static volatile bool sHold = false;
static volatile bool sRestart = false;
static float sLut[257];

// hall processing
static float sVz[2] = {1.65f, 1.65f};
static bool sZeroed[2] = {false, false};
static volatile uint8_t sZeroArm = 0;      // bit 0 center, bit 1 perimeter
static volatile uint8_t sZeroClr = 0;
static double sZeroSum[2] = {0, 0};
static uint32_t sZeroN[2] = {0, 0};
static volatile uint8_t sClearBits = 0;
static uint32_t sHoldoff = 0;              // processed-sample index stats resume at
static uint32_t sProcIdx = 0;

static float sB[2] = {0, 0};               // latest processed field, µT
static float sBf[2] = {0, 0};              // 300 Hz smoothed for the tiles
static float sDb = 0, sDbf = 0;
static float sDbHist[kDiffK] = {0};
static int sDbPos = 0;
static float sBMax[2] = {0, 0}, sDbMax = 0;
static double sAvgSum[2] = {0, 0}, sRmsSq = 0;
static uint32_t sStatN = 0;
static uint32_t sStatT0 = 0;
static bool sClip = false;
static uint32_t sClipAt = 0;

// waveform rings (PSRAM when available)
static float* sRing[2] = {nullptr, nullptr};
static uint32_t sRingCap = 0;
static volatile uint32_t sRingW = 0;       // one write index: both rings advance together

// phase fold
static float sFold[FOLD_N];
static uint8_t sFoldHit[FOLD_N];
static uint32_t sFoldPer = 0;
static uint32_t sFoldCycles = 0;
static uint32_t sFoldLastCycle = 0;

// FFT stream and published spectrum
static float* sFftRing = nullptr;
static uint32_t sFftCap = 0, sFftW = 0;
static int sFftDecim = 0;
static float sFftAcc = 0;
static float sSpec[SPEC_MAX];
static int sSpecN = 0;
static float sSpecDf = 0;
static float sPkHz = 0, sPkAmp = 0, sPkSnr = 0;
static float sProcHz = 0;                  // measured processed rate per hall channel
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
static void adcStop() {
  if (!sAdcUp) return;
  adc_digi_stop();
  adc_digi_deinitialize();
  sAdcUp = false;
}

static bool adcStart() {
  adcStop();
  esp_adc_cal_characteristics_t cal = {};
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &cal);
  for (int i = 0; i <= 256; i++) {
    const int raw = i == 256 ? 4095 : i * 16;
    sLut[i] = esp_adc_cal_raw_to_voltage(raw, &cal) / 1000.f;
  }
  uint32_t mask = 0;
  for (int i = 0; i < 4; i++) {
    sChan[i] = sGpio[i] - 1;                 // S3: ADC1_CH0 = GPIO1 ... ADC1_CH8 = GPIO9
    mask |= 1u << sChan[i];
  }
  for (uint32_t hz : kAdcTry) {
    adc_digi_init_config_t init = {};
    init.max_store_buf_size = 32768;
    init.conv_num_each_intr = 1024;
    init.adc1_chan_mask = mask;
    init.adc2_chan_mask = 0;
    esp_err_t err = adc_digi_initialize(&init);
    if (err != ESP_OK) {
      init.max_store_buf_size = 4096;
      init.conv_num_each_intr = 256;
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
    sAdcUp = true;
    sProcHz = (float)hz / 4.f / (float)kDecim;   // refined by measurement every second
    Serial.printf("sense: ADC dma %u Hz total, ~%.0f Hz per hall channel\n", hz, sProcHz);
    return true;
  }
  Serial.println("sense: ADC did not start");
  return false;
}

// ---- per-processed-sample work (runs ~10 kHz, keep it lean) -----------------------------------------------------------
static inline void statsAndRings(float b0, float b1, int64_t tUs) {
  sProcIdx++;
  // scope rings
  if (sRing[0]) {
    uint32_t w = sRingW % sRingCap;
    sRing[0][w] = b0;
    sRing[1][w] = b1;
    sRingW++;
  }
  // dB/dt: difference over kDiffK processed samples, scaled to per-second, then 500 Hz smoothing
  float prev = sDbHist[sDbPos];
  sDbHist[sDbPos] = b0;
  sDbPos = (sDbPos + 1) % kDiffK;
  if (sProcIdx > (uint32_t)kDiffK && sProcHz > 1.f) {
    float d = (b0 - prev) * (sProcHz / (float)kDiffK) * 1e-3f;   // µT/s -> mT/s
    float a = 1.f - expf(-6.2831853f * 500.f / sProcHz);
    sDbf += a * (d - sDbf);
    sDb = d;
  }
  // display smoothing
  float af = sProcHz > 1.f ? 1.f - expf(-6.2831853f * 300.f / sProcHz) : 1.f;
  sBf[0] += af * (b0 - sBf[0]);
  sBf[1] += af * (b1 - sBf[1]);

  // stats, held off briefly after a zero / pin / cal change so the step never latches as a peak
  if (sProcIdx >= sHoldoff) {
    float a0 = fabsf(b0), a1 = fabsf(b1), ad = fabsf(sDbf);
    if (a0 > sBMax[0]) sBMax[0] = a0;
    if (a1 > sBMax[1]) sBMax[1] = a1;
    if (ad > sDbMax) sDbMax = ad;
    sAvgSum[0] += b0;
    sAvgSum[1] += b1;
    sRmsSq += (double)b0 * (double)b0;
    sStatN++;
  }

  // phase fold against the live pulse period
  uint32_t per = coilPeriodUs();
  if (per) {
    if (per != sFoldPer || (per > sFoldPer ? per - sFoldPer : sFoldPer - per) > per / 100) {
      // (re)arm on any period change worth 1 %
      if (per != sFoldPer) {
        memset(sFold, 0, sizeof sFold);
        memset(sFoldHit, 0, sizeof sFoldHit);
        sFoldPer = per;
        sFoldCycles = 0;
        sFoldLastCycle = coilCycles();
      }
    }
    int64_t ph = tUs - coilPeriodT0();
    if (ph >= 0 && ph < (int64_t)per * 4) {
      uint32_t p = (uint32_t)(ph % per);
      int bin = (int)((uint64_t)p * FOLD_N / per);
      if (bin >= 0 && bin < FOLD_N) {
        if (sFoldHit[bin] < 8) { sFoldHit[bin]++; sFold[bin] += (b0 - sFold[bin]) / sFoldHit[bin]; }
        else sFold[bin] += 0.06f * (b0 - sFold[bin]);
      }
    }
    uint32_t cyc = coilCycles();
    if (cyc != sFoldLastCycle) { sFoldCycles += cyc - sFoldLastCycle; sFoldLastCycle = cyc; }
  } else if (sFoldPer) {
    sFoldPer = 0;
  }

  // FFT stream: box-decimate the center field
  sFftAcc += b0;
  if (++sFftDecim >= FFT_DECIM) {
    if (sFftRing) {
      sFftRing[sFftW % sFftCap] = sFftAcc / (float)FFT_DECIM;
      sFftW++;
    }
    sFftAcc = 0;
    sFftDecim = 0;
  }
}

static void publishSnap() {
  uint32_t now = millis();
  SenseSnap s;
  s.b0 = sBf[0]; s.b1 = sBf[1];
  s.db = sDbf;
  s.bMax0 = sBMax[0]; s.bMax1 = sBMax[1]; s.dbMax = sDbMax;
  s.bAvg0 = sStatN ? (float)(sAvgSum[0] / sStatN) : 0;
  s.bAvg1 = sStatN ? (float)(sAvgSum[1] / sStatN) : 0;
  s.bRms0 = sStatN ? (float)sqrt(sRmsSq / sStatN) : 0;
  s.statSec = (now - sStatT0) / 1000.f;
  s.v0 = sVz[0] + sBf[0] / 100.f * sensVperG(0);
  s.v1 = sVz[1] + sBf[1] / 100.f * sensVperG(1);
  s.zeroed0 = sZeroed[0]; s.zeroed1 = sZeroed[1];
  s.clip = sClip && (now - sClipAt) < 200;
  s.fs = sProcHz;
  s.tC = sTempF[0]; s.tF = sTempF[1];
  s.dtdtC = sDtdt[0]; s.dtdtF = sDtdt[1];
  s.okC = sThOk[0]; s.okF = sThOk[1];
  s.pkHz = sPkHz; s.pkAmp = sPkAmp; s.pkSnr = sPkSnr;
  portENTER_CRITICAL(&sMux);
  uint32_t seq = sSnap.seq + 1;
  sSnap = s;
  sSnap.seq = seq;
  portEXIT_CRITICAL(&sMux);
}

static void thermWindow(uint32_t now) {
  for (int t = 0; t < 2; t++) {
    bool ok = false;
    float f = NAN;
    if (sThN[t] > 10) f = thermF(t, (float)(sThSum[t] / sThN[t]), &ok);
    sThSum[t] = 0;
    sThN[t] = 0;
    sThOk[t] = ok;
    if (ok) sTempF[t] = isnan(sTempF[t]) ? f : sTempF[t] + 0.3f * (f - sTempF[t]);
    else sTempF[t] = NAN;
  }
  // 1 Hz history for the rate of change
  if (now - sThHistAt >= 1000) {
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
    sHoldoff = sProcIdx + (uint32_t)(sProcHz * ZERO_SEC);
  }
  uint8_t zc = sZeroClr;
  if (zc) {
    sZeroClr = 0;
    for (int i = 0; i < 2; i++) {
      if (!(zc & (1 << i))) continue;
      sVz[i] = sCal.vcc * 0.5f;
      sZeroed[i] = false;
      sPrefs.putFloat(i ? "vz1" : "vz0", sVz[i]);
      sPrefs.putBool(i ? "z1" : "z0", false);
    }
    sHoldoff = sProcIdx + (uint32_t)(sProcHz * ZERO_SEC);
  }
  uint8_t clr = sClearBits;
  if (clr) {
    sClearBits = 0;
    if (clr & SENSE_CLR_MAX) { sBMax[0] = sBMax[1] = sDbMax = 0; }
    if (clr & SENSE_CLR_AVG) { sAvgSum[0] = sAvgSum[1] = 0; sRmsSq = 0; sStatN = 0; sStatT0 = millis(); }
  }
}

static void senseTask(void*) {
  static uint8_t buf[2048];
  double sumV[2] = {0, 0};
  int nDec[2] = {0, 0};
  bool clipRun = false;
  int badRun = 0;
  sRateT0 = millis();
  sStatT0 = millis();
  for (;;) {
    if (sHold) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    if (sRestart || !sAdcUp) {
      sRestart = false;
      adcStart();
      if (!sAdcUp) { vTaskDelay(pdMS_TO_TICKS(500)); continue; }
    }
    applyPendingCommands();

    uint32_t n = 0;
    esp_err_t err = adc_digi_read_bytes(buf, sizeof buf, &n, 10);
    if (err == ESP_ERR_TIMEOUT) { publishSnap(); continue; }
    if (err != ESP_OK) {
      if (++badRun > 40) { sRestart = true; badRun = 0; }
      continue;
    }
    badRun = 0;

    int64_t tEnd = esp_timer_get_time();
    // samples in this block per hall channel, for per-sample time reconstruction in the fold
    int hallInBlock = 0;
    for (uint32_t i = 0; i + sizeof(adc_digi_output_data_t) <= n; i += sizeof(adc_digi_output_data_t)) {
      const adc_digi_output_data_t* p = (const adc_digi_output_data_t*)&buf[i];
      if (p->type2.channel == (uint32_t)sChan[0]) hallInBlock++;
    }
    int hallSeen = 0;

    for (uint32_t i = 0; i + sizeof(adc_digi_output_data_t) <= n; i += sizeof(adc_digi_output_data_t)) {
      const adc_digi_output_data_t* p = (const adc_digi_output_data_t*)&buf[i];
      int raw = (int)p->type2.data;
      if (raw < 0 || raw > 4095) continue;
      int ch = (int)p->type2.channel;
      int slot = -1;
      for (int k = 0; k < 4; k++) if (sChan[k] == ch) { slot = k; break; }
      if (slot < 0) continue;
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
      if (h == 0) hallSeen++;
      if (sZeroArm & (1 << h)) {
        sZeroSum[h] += volts;
        sZeroN[h]++;
        if (sZeroN[h] >= (uint32_t)(sProcHz * kDecim * ZERO_SEC)) {
          sVz[h] = (float)(sZeroSum[h] / sZeroN[h]);
          sZeroed[h] = true;
          sZeroArm &= ~(1 << h);
          sPrefs.putFloat(h ? "vz1" : "vz0", sVz[h]);
          sPrefs.putBool(h ? "z1" : "z0", true);
          sHoldoff = sProcIdx + (uint32_t)(sProcHz * ZERO_SEC);
        }
      }
      sumV[h] += volts;
      if (++nDec[h] >= kDecim) {
        float b = hallUt(h, (float)(sumV[h] / nDec[h]));
        sumV[h] = 0;
        nDec[h] = 0;
        if (h == 0) {
          sB[0] = b;
          // reconstruct this sample's time from its place in the block
          int64_t tS = tEnd;
          if (sProcHz > 1.f && hallInBlock > 0) {
            tS -= (int64_t)((double)(hallInBlock - hallSeen) / (sProcHz * kDecim) * 1e6);
          }
          statsAndRings(sB[0], sB[1], tS);
        } else {
          sB[1] = b;
        }
      }
    }
    if (clipRun) { sClip = true; sClipAt = millis(); clipRun = false; }

    uint32_t now = millis();
    if (now - sThWinAt >= 250) { sThWinAt = now; thermWindow(now); }
    if (now - sRateT0 >= 1000) {
      if (sRate[0] > 0) sProcHz = (float)sRate[0] * 1000.f / (float)(now - sRateT0) / (float)kDecim;
      sRate[0] = sRate[1] = sRate[2] = sRate[3] = 0;
      sRateT0 = now;
    }
    static uint32_t pubAt = 0;
    if (now - pubAt >= 50) { pubAt = now; publishSnap(); }
  }
}

// ---- FFT task ---------------------------------------------------------------------------------------------------------
static void fftTask(void*) {
  float* re = (float*)heap_caps_malloc(FFT_N * sizeof(float), MALLOC_CAP_SPIRAM);
  float* im = (float*)heap_caps_malloc(FFT_N * sizeof(float), MALLOC_CAP_SPIRAM);
  if (!re) re = (float*)malloc(FFT_N * sizeof(float));
  if (!im) im = (float*)malloc(FFT_N * sizeof(float));
  if (!re || !im || !sFftRing) { Serial.println("sense: no memory for the FFT"); vTaskDelete(nullptr); return; }
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(600));
    if (sHold) continue;
    uint32_t w = sFftW;
    if (w < (uint32_t)FFT_N) continue;
    for (int i = 0; i < FFT_N; i++) re[i] = sFftRing[(w - FFT_N + i) % sFftCap];
    if (sFftW - w > sFftCap - FFT_N) continue;      // lapped while copying; try again next pass
    memset(im, 0, FFT_N * sizeof(float));
    float fs = sProcHz / (float)FFT_DECIM;
    if (fs < 10.f) continue;
    mg::fftRemoveDc(re, FFT_N);
    mg::fftHann(re, FFT_N);
    mg::fftRun(re, im, FFT_N);
    int bins = FFT_N / 2;
    float df = fs / (float)FFT_N;
    // magnitudes in place, then the peak and the published (max-decimated) spectrum
    for (int i = 0; i < bins; i++) re[i] = sqrtf(re[i] * re[i] + im[i] * im[i]);
    mg::FftPeak pk;
    bool got = mg::fftPeak(re, bins, df, 0.3f, SPEC_HZ, FFT_N, &pk);
    int want = (int)(SPEC_HZ / df);
    if (want > bins) want = bins;
    int outN = want > SPEC_MAX ? SPEC_MAX : want;
    portENTER_CRITICAL(&sMux);
    for (int o = 0; o < outN; o++) {
      int a = (int)((int64_t)o * want / outN), b = (int)((int64_t)(o + 1) * want / outN);
      float m = 0;
      for (int i = a; i < b && i < bins; i++) if (re[i] > m) m = re[i];
      sSpec[o] = 4.f * m / (float)FFT_N;      // amplitude-corrected like the peak
    }
    sSpecN = outN;
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

  sRingCap = 65536;
  for (int i = 0; i < 2; i++) {
    sRing[i] = (float*)heap_caps_malloc(sRingCap * sizeof(float), MALLOC_CAP_SPIRAM);
  }
  if (!sRing[0] || !sRing[1]) {
    sRingCap = 8192;
    for (int i = 0; i < 2; i++) {
      if (!sRing[i]) sRing[i] = (float*)malloc(sRingCap * sizeof(float));
    }
  }
  sFftCap = 8192;
  sFftRing = (float*)heap_caps_malloc(sFftCap * sizeof(float), MALLOC_CAP_SPIRAM);
  if (!sFftRing) sFftRing = (float*)malloc(sFftCap * sizeof(float));
  memset(sFold, 0, sizeof sFold);

  if (!adcStart()) { /* the task keeps retrying */ }
  xTaskCreatePinnedToCore(senseTask, "sense", 8192, nullptr, 4, &sTask, 0);
  xTaskCreatePinnedToCore(fftTask, "fft", 6144, nullptr, 1, &sFftTask, 0);
  return true;
}

void senseSetCal(const SenseCal& cal) {
  sCalPend = cal;
  sCalDirty = true;
}

void senseZero(int which) {
  uint8_t bits = which == 2 ? 3 : (1 << (which & 1));
  for (int i = 0; i < 2; i++) {
    if (!(bits & (1 << i))) continue;
    sZeroSum[i] = 0;
    sZeroN[i] = 0;
  }
  sZeroArm |= bits;
}

void senseZeroClear(int which) { sZeroClr |= which == 2 ? 3 : (1 << (which & 1)); }
void senseClearStats(uint8_t bits) { sClearBits |= bits; }

void senseCopy(SenseSnap& out) {
  portENTER_CRITICAL(&sMux);
  out = sSnap;
  portEXIT_CRITICAL(&sMux);
}

int senseWave(int ch, uint32_t spanMs, int maxBins, float* mn, float* mx, float* fsOut) {
  if (!sRing[0] || sProcHz < 1.f) return 0;
  if (fsOut) *fsOut = sProcHz;
  uint32_t w = sRingW;
  uint32_t nS = (uint32_t)((double)spanMs * sProcHz / 1000.0);
  if (nS < 8) nS = 8;
  if (nS > sRingCap) nS = sRingCap;
  if (nS > w) nS = w;
  if (nS < 8) return 0;
  int bins = (int)nS < maxBins ? (int)nS : maxBins;
  const float* src = ch == 1 ? sRing[1] : sRing[0];
  uint32_t start = w - nS;
  for (int o = 0; o < bins; o++) {
    uint32_t a = start + (uint32_t)((uint64_t)o * nS / bins);
    uint32_t b = start + (uint32_t)((uint64_t)(o + 1) * nS / bins);
    if (b <= a) b = a + 1;
    float lo = 1e30f, hi = -1e30f;
    if (ch == 2) {                              // dB/dt rebuilt from the center ring
      for (uint32_t i = a; i < b; i++) {
        uint32_t j = i >= (uint32_t)kDiffK ? i - kDiffK : 0;
        float d = (sRing[0][i % sRingCap] - sRing[0][j % sRingCap]) * (sProcHz / (float)kDiffK) * 1e-3f;
        if (d < lo) lo = d;
        if (d > hi) hi = d;
      }
    } else {
      for (uint32_t i = a; i < b; i++) {
        float v = src[i % sRingCap];
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      }
    }
    mn[o] = lo;
    mx[o] = hi;
  }
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

int senseFold(float* out, int maxBins, uint32_t* perUs, uint32_t* cycles) {
  if (!sFoldPer) return 0;
  int n = FOLD_N < maxBins ? FOLD_N : maxBins;
  for (int i = 0; i < n; i++) out[i] = sFold[(int)((int64_t)i * FOLD_N / n)];
  if (perUs) *perUs = sFoldPer;
  if (cycles) *cycles = sFoldCycles;
  return n;
}

void senseHold(bool hold) {
  sHold = hold;
  if (hold) adcStop();
  else sRestart = true;
}
