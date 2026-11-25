// sclock.h - maps the ADC's running sample index to esp_timer microseconds.
//
// The DMA hands the sampler a block of conversions every ~13 ms, and the only time stamp available
// is when the task got round to reading it - the conversion time plus a scheduling delay that
// varies block to block by tens to hundreds of microseconds. Stamping samples from that directly
// (what the fold used to do) smears anything shorter than the jitter, which is everything that
// matters on a 100-500 us pulse.
//
// The samples themselves are evenly spaced, though, so their times lie on a straight line:
// t = off + idx * ts. The delay can only ever make a block look LATE, never early, so the line is
// the lower envelope of (idx, arrival time): once a second the least-delayed block is kept, a line
// is fitted through the last 16 of those, and its offset is set by the lowest point. A block that
// arrives earlier than the line allows pulls the line down at once. If the stream skips samples
// (a DMA overflow) every block suddenly looks milliseconds late; the clock notices and re-anchors.
//
// Pure C++, host-tested.
#pragma once
#include <math.h>
#include <stdint.h>

namespace mg {

struct SampleClock {
  static const int PTS = 16;
  double tsNom = 100.0;    // nominal us per sample
  double ts = 100.0;       // current estimate
  double off = 0.0;        // t(idx) = off + idx * ts
  bool valid = false;
  bool rateOk = false;     // a fit has confirmed ts; kept across a restart (dropped samples move the
                           // anchor, not the rate)
  uint32_t blocks = 0;     // blocks since the anchor was last (re)set
  double winStart = 0, winMinR = 1e30, winIdx = 0, winT = 0;
  double pi[PTS], pt[PTS];
  int n = 0, head = 0;

  void reset(double tsNominal) {
    tsNom = ts = tsNominal > 0 ? tsNominal : 100.0;
    rateOk = false;
    restart();
  }
  // The sample stream broke (ADC restarted, samples dropped): keep the rate, forget the anchor.
  void restart() {
    valid = false;
    blocks = 0;
    n = head = 0;
    winMinR = 1e30;
  }
  double at(double idx) const { return off + idx * ts; }
  // Good enough for microsecond work (the fold, the scope trigger): the rate has been fitted and the
  // anchor has had a quarter second or so of blocks to find a low-delay one.
  bool locked() const { return valid && rateOk && blocks >= 20; }
  // inverse: the (fractional) sample index converted at time t
  double indexAt(double t) const { return ts > 0 ? (t - off) / ts : 0; }

  // The last sample of a block has index idxLast and had been converted by tEnd (us).
  void onBlock(double idxLast, double tEnd) {
    if (!valid) {
      off = tEnd - idxLast * ts;
      valid = true;
      winStart = tEnd;
      winMinR = 1e30;                       // this block's own delay is unknown: it is not a minimum yet
      return;
    }
    blocks++;
    double r = tEnd - at(idxLast);          // the block's delay, >= 0 when the line is right
    if (r < 0) { off += r; r = 0; }         // earlier than possible: the line itself is late
    if (r < winMinR) { winMinR = r; winIdx = idxLast; winT = tEnd; }
    if (tEnd - winStart < 1e6) return;
    if (winMinR > 3000.0) {                  // a whole second with every block 3 ms+ late: samples were lost
      restart();
      if (!rateOk) ts = tsNom;               // an unconfirmed rate may have been the problem: start it over
      onBlock(idxLast, tEnd);
      return;
    }
    pi[head] = winIdx;
    pt[head] = winT;
    head = (head + 1) % PTS;
    if (n < PTS) n++;
    fit();
    winStart = tEnd;
    winMinR = 1e30;
  }

  void fit() {
    if (n < 4) return;
    double lo = pi[0], hi = pi[0];
    for (int k = 1; k < n; k++) { if (pi[k] < lo) lo = pi[k]; if (pi[k] > hi) hi = pi[k]; }
    if ((hi - lo) * ts < 3e6) return;      // under 3 s of baseline the slope is not worth trusting
    // least squares through the window minima, centred for precision
    double mi = 0, mt = 0;
    for (int k = 0; k < n; k++) { mi += pi[k]; mt += pt[k]; }
    mi /= n; mt /= n;
    double sii = 0, sit = 0;
    for (int k = 0; k < n; k++) { double di = pi[k] - mi; sii += di * di; sit += di * (pt[k] - mt); }
    if (sii <= 0) return;
    double b = sit / sii;
    if (!(b > tsNom * 0.97 && b < tsNom * 1.03)) return;   // nonsense fit: keep what we had
    ts = b;
    rateOk = true;
    // the line sits on the lowest point, not through the middle of them
    double o = 1e300;
    for (int k = 0; k < n; k++) { double c = pt[k] - pi[k] * ts; if (c < o) o = c; }
    off = o;
  }
};

}  // namespace mg
