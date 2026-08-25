/*
 * hourly.cpp  -  Time reconstruction and bucket binning (see hourly.h).
 */

#include "hourly.h"
#include <math.h>

// Floor 'ts' down to the bucket boundary of a grid whose boundaries sit at
// (phase + k * bucketSec). Works for any phase in [0, bucketSec).
static uint32_t floorToGrid(uint32_t ts, uint32_t phase, uint32_t bucketSec) {
  if (bucketSec == 0u) return ts;
  uint32_t rel = ts - phase;              // wraps harmlessly: only rel % bucketSec is used
  return ts - (rel % bucketSec);
}

void hourlyCarryClear(HourlyCarry &c) {
  c.magic         = HOURLY_CARRY_MAGIC;
  c.valid         = 0u;
  c.binStartLocal = 0u;
  c.pulses        = 0u;
  c.gallons       = 0.0f;
  c.lastEndLocal  = 0u;
  c.bucketSec     = 0u;
}

bool hourlyCarryUsable(const HourlyCarry &c, uint32_t spanStartLocal,
                       uint32_t bucketSec, uint32_t toleranceSec) {
  if (c.magic != HOURLY_CARRY_MAGIC || !c.valid) return false;
  if (c.bucketSec != bucketSec)                 return false;   // width changed -> meaningless
  if (c.lastEndLocal == 0u)                     return false;
  // The new span must begin where the old one ended, within tolerance. A larger
  // gap means time was lost (a failed session, a re-alignment, a reboot) and the
  // old partial bucket can no longer be completed honestly.
  uint32_t diff = (spanStartLocal > c.lastEndLocal) ? (spanStartLocal - c.lastEndLocal)
                                                    : (c.lastEndLocal - spanStartLocal);
  return diff <= toleranceSec;
}

uint32_t hourlyResolveSpanStart(const PicReportInfo &info, uint32_t endLocal,
                                uint32_t prevEndLocal, uint32_t toleranceSec,
                                const char **source) {
  uint32_t start;
  const char *src;

  if (info.startTimeValid && info.startTime != 0u && info.startTime < endLocal) {
    start = info.startTime;                       // the PIC knew when this span began
    src   = "pic_header";
  } else {
    // Back-computation (Photon request doc 4.2). sampleInterval is the PIC's own
    // LOCO-corrected intent, so this is the best estimate available when the PIC
    // had no valid previous wake to report.
    //
    // Appendix H.4: span the whole run in MILLISECONDS from sampleIntervalMs, then
    // round once to a whole-second epoch. The old intervalSec = sampleIntervalMs /
    // 1000u truncated 5207 ms to 5 s on every block; here the fraction survives to
    // the single rounding at the end.
    uint32_t stepMs = info.sampleIntervalMs ? info.sampleIntervalMs : 1000u;
    uint32_t back   = (uint32_t)(((uint64_t)info.sampleCount * stepMs + 500u) / 1000u);
    start = (endLocal > back) ? (endLocal - back) : 0u;
    src   = "back_computed";
  }

  // Continuity beats both of the above: if the resolved start is within
  // tolerance of where the previous report ended, use that exact instant so the
  // two reports share one seam and no bucket is double-counted or left short.
  if (prevEndLocal != 0u && prevEndLocal < endLocal) {
    uint32_t diff = (start > prevEndLocal) ? (start - prevEndLocal) : (prevEndLocal - start);
    if (diff <= toleranceSec) {
      start = prevEndLocal;
      src   = (info.startTimeValid ? "pic_header+continuity" : "back_computed+continuity");
    }
  }

  if (source) *source = src;
  return start;
}

uint32_t hourlyOverlapSkip(uint32_t n, uint32_t startLocal, uint32_t endLocal,
                           uint32_t prevEndLocal) {
  if (n == 0u)                    return 0u;
  if (prevEndLocal == 0u)         return 0u;   // no seam recorded yet
  if (endLocal <= startLocal)     return 0u;   // degenerate span: leave it alone
  if (prevEndLocal <= startLocal) return 0u;   // starts at or after the seam
  if (prevEndLocal >= endLocal)   return n;    // the whole batch is behind the seam

  float step = (float)(endLocal - startLocal) / (float)n;
  if (step < 0.001f) return 0u;
  float    f    = (float)(prevEndLocal - startLocal) / step;
  uint32_t skip = (uint32_t)(f + 0.5f);
  return (skip > n) ? n : skip;
}

bool hourlyProcess(const PicSample *s, uint32_t n, const PicReportInfo &info,
                   uint32_t startLocal, uint32_t endLocal, uint32_t bucketSec,
                   float nominalSampleSec, SampleToGallonsFn toGallons,
                   uint8_t missedFillMode, HourlyCarry &carry, HourlyResult &out,
                   uint16_t *bucketSamples, uint32_t bucketSamplesCap,
                   float *bucketGalSamples, float *bucketGalMissed,
                   bool reconstructedSpan) {
  memset(&out, 0, sizeof(out));
  out.bucketSec      = bucketSec;
  out.spanStartLocal = startLocal;
  out.spanEndLocal   = endLocal;
  out.samplesUsed    = n;

  if (s == nullptr || n == 0u || bucketSec == 0u) return false;
  if (endLocal <= startLocal)                     return false;

  // ---- 1. equalize ---------------------------------------------------------
  // The measured step is preferred, but a clock jump (first sync after a bench
  // run, a re-aligned grid, a truncated report) must not stretch the series into
  // nonsense. Outside 0.25x..4x of the PIC's own nominal interval we fall back.
  float span    = (float)(endLocal - startLocal);
  float perStep = span / (float)n;
  out.equalized = true;
  if (reconstructedSpan && nominalSampleSec > 0.001f) {
    // Appendix H.4: the span here was fabricated end-to-end from the interval in
    // the sec 6.7 recovery, then rounded to whole-second epochs. span/n is a
    // quantized echo of the interval (35 x 5.207 s round-tripped to 5.000 s), not
    // an independent measurement. Use the exact nominal (millisecond) step so the
    // reconstructed placement matches the PIC's true cadence; mark it non-measured.
    perStep       = nominalSampleSec;
    out.equalized = false;
  } else if (nominalSampleSec > 0.001f &&
      (perStep < nominalSampleSec * 0.25f || perStep > nominalSampleSec * 4.0f)) {
    perStep       = nominalSampleSec;
    out.equalized = false;
  }
  if (perStep < 0.001f) return false;
  out.perSampleSec = perStep;

  // ---- 2. choose the bucket grid phase ------------------------------------
  uint32_t phase;
#if BUCKET_ALIGN_MODE == BUCKET_ALIGN_FROM_NOW
  // Buckets counted back from the end of the span, so the newest one closes
  // exactly at T_end. When a carry is being continued we keep ITS phase instead,
  // otherwise the grid would shift between reports and the seam would not line up.
  if (hourlyCarryUsable(carry, startLocal, bucketSec, bucketSec))
    phase = carry.binStartLocal % bucketSec;
  else
    phase = endLocal % bucketSec;
#else
  // Absolute clock grid: boundaries at multiples of bucketSec, so 3600 s buckets
  // start on the hour. This is what makes the published labels readable.
  phase = 0u;
#endif

  // ---- 3. bin the samples --------------------------------------------------
  // Sample k ends at endLocal - (n-1-k)*perStep, and is credited to the bucket
  // containing that instant. The "- 1" makes a sample that ends exactly on a
  // boundary belong to the bucket it closes, not to the empty one it opens.
  uint32_t firstBin   = floorToGrid(startLocal, phase, bucketSec);
  bool     haveCarry  = hourlyCarryUsable(carry, startLocal, bucketSec, bucketSec);

  // Appendix G.1.7: the carried open bucket must fall INSIDE the window whichever
  // side of the resolved firstBin it lands on. The previous code only merged when
  // carry.binStartLocal <= firstBin and silently dropped the carry otherwise - a
  // real possibility, because hourlyCarryUsable() tolerances the seam by bucketSec
  // while the span-start snap uses SPAN_CONTINUITY_TOL_SEC, so the two can accept
  // a seam that leaves floorToGrid(startLocal) one bucket ahead of the carry.
  // Extend the window to cover the carry bucket in EITHER direction instead of
  // gating on a single inequality; the seed itself is placed by index below.
  if (haveCarry && carry.binStartLocal < firstBin) firstBin = carry.binStartLocal;

  uint32_t lastSampleBin = firstBin;
  {
    uint32_t tsLast = endLocal;
    lastSampleBin   = floorToGrid(tsLast - 1u, phase, bucketSec);
  }
  if (lastSampleBin < firstBin) lastSampleBin = firstBin;
  // Defensive: if the carry bucket is somehow later than the last sample bin,
  // widen the window forward so it still has a home rather than being dropped.
  if (haveCarry && carry.binStartLocal > lastSampleBin) lastSampleBin = carry.binStartLocal;

  uint32_t nBins = (lastSampleBin - firstBin) / bucketSec + 1u;

  // A very long outage can produce far more buckets than we can hold. Keep the
  // NEWEST HOURLY_MAX_BUCKETS: recent data is what the dashboard needs, and the
  // count that was dropped is reported as (totalMakeable - count).
  uint32_t windowBins = nBins;
  if (windowBins > HOURLY_MAX_BUCKETS) windowBins = HOURLY_MAX_BUCKETS;
  uint32_t windowFirstBin = firstBin + (nBins - windowBins) * bucketSec;

  // File-scope, not on the stack: 250 buckets of float + u32 is 2 kB, which is
  // more than this task's stack should carry. hourlyProcess() is never reentered.
  // binGalSample is the sample-only contribution (no carry seed, no missed-fill),
  // kept so the bucket line can be split into its three sources (Appendix G.3.5).
  static float    binGal[HOURLY_MAX_BUCKETS];
  static uint32_t binPul[HOURLY_MAX_BUCKETS];
  static float    binGalSample[HOURLY_MAX_BUCKETS];
  for (uint32_t i = 0; i < windowBins; i++) { binGal[i] = 0.0f; binPul[i] = 0u; binGalSample[i] = 0.0f; }
  if (bucketSamples) {
    uint32_t z = (windowBins < bucketSamplesCap) ? windowBins : bucketSamplesCap;
    for (uint32_t i = 0; i < z; i++) bucketSamples[i] = 0u;
  }
  if (bucketGalSamples) {
    uint32_t z = (windowBins < bucketSamplesCap) ? windowBins : bucketSamplesCap;
    for (uint32_t i = 0; i < z; i++) bucketGalSamples[i] = 0.0f;
  }
  if (bucketGalMissed) {
    uint32_t z = (windowBins < bucketSamplesCap) ? windowBins : bucketSamplesCap;
    for (uint32_t i = 0; i < z; i++) bucketGalMissed[i] = 0.0f;
  }

  // Seed the carried bucket with whatever the previous report left behind. This
  // is the ONLY place the carry gets placed; carryMerged records that it landed.
  out.carryMerged     = false;
  out.carryInGallons  = 0.0f;
  out.carryInPulses   = 0u;
  if (haveCarry && carry.binStartLocal >= windowFirstBin) {
    uint32_t idx = (carry.binStartLocal - windowFirstBin) / bucketSec;
    if (idx < windowBins) {
      binGal[idx]        += carry.gallons;
      binPul[idx]        += carry.pulses;
      out.carryMerged     = true;
      out.carryInGallons  = carry.gallons;   // what actually entered the window
      out.carryInPulses   = carry.pulses;
    }
  }

  uint32_t sentPulses    = 0u;   // every sample's pulses (for the missed-fill gap)
  uint32_t placedPulses  = 0u;   // only the pulses that landed inside the window
  float    sampleGallons = 0.0f; // toGallons() over placed samples (window only)
  for (uint32_t k = 0; k < n; k++) {
    uint32_t back  = (uint32_t)((float)(n - 1u - k) * perStep + 0.5f);
    uint32_t tsEnd = (endLocal > back) ? (endLocal - back) : startLocal;
    uint32_t bin   = floorToGrid((tsEnd > 0u ? tsEnd - 1u : 0u), phase, bucketSec);
    sentPulses += s[k].pulses;
    if (bin < windowFirstBin) continue;                       // outside the kept window
    uint32_t idx = (bin - windowFirstBin) / bucketSec;
    if (idx >= windowBins) idx = windowBins - 1u;
    float g = toGallons(s[k].pulses, perStep);
    binGal[idx]       += g;
    binGalSample[idx] += g;
    binPul[idx]       += s[k].pulses;
    placedPulses      += s[k].pulses;
    sampleGallons     += g;
    if (bucketSamples && idx < bucketSamplesCap && bucketSamples[idx] < 0xFFFFu)
      bucketSamples[idx]++;                                    // Appendix E.4 (c): samples per bucket
  }
  out.sampleGallons = sampleGallons;
  out.placedPulses  = placedPulses;
  if (bucketGalSamples) {
    uint32_t z = (windowBins < bucketSamplesCap) ? windowBins : bucketSamplesCap;
    for (uint32_t i = 0; i < z; i++) bucketGalSamples[i] = binGalSample[i];
  }

  // ---- 4. reconcile against the PIC's hardware totals ----------------------
  // total_impulses counts every pulse of the report period, including any the
  // ring could not keep AND any a single 16-bit sample cell could not represent
  // (the OVFLOW bench phase clamps samples at 0xFFFF while the total keeps the
  // true ~105,800/capture). The difference is real water that has no usable
  // sample. In AVERAGE mode we restore the missing VOLUME, spread across the
  // buckets this span actually covers - never across a fixed 24, which would
  // smear it into hours the water did not flow in.
  if (missedFillMode == PIC_MISSED_FILL_AVERAGE &&
      info.totalImpulses > sentPulses && windowBins > 0u) {
    uint32_t missedPulses   = info.totalImpulses - sentPulses;
    uint32_t missedCaptures = (info.totalCaptures > n) ? (info.totalCaptures - n) : 0u;
    uint32_t spanCaps       = (missedCaptures > 0u) ? missedCaptures : n;
    if (spanCaps == 0u) spanCaps = 1u;
    float perCapturePulses = (float)missedPulses / (float)spanCaps;
    // toGallons takes a uint16_t (a real sample never exceeds 0xFFFF). At OVFLOW
    // magnitude the per-capture average can exceed 65535, so clamp before the
    // cast: an unclamped cast would WRAP (105800 -> 40264) and corrupt the
    // restored volume. NOTE (Appendix G.1.3): the old comment here claimed the
    // gallon figure is "unchanged in practice" because the calibration is
    // saturated at that rate. That is wrong - freqToGpm() does not saturate, it
    // COLLAPSES to 0 above ~109 Hz (Appendix G.1.1), so a clamped OVFLOW average
    // converts to 0 gal, not to a saturated maximum. The clamp is still correct
    // (it prevents the cast wrap); the trace below records the collapse so the
    // caller can warn that the restored volume was lost rather than zero.
    uint32_t pc = (perCapturePulses > 65535.0f) ? 65535u
                                                : (uint32_t)(perCapturePulses + 0.5f);
    bool  clamped   = (perCapturePulses > 65535.0f);
    float missedGal = toGallons((uint16_t)pc, perStep) * (float)spanCaps;
    out.missedGallons = missedGal;
    float per = missedGal / (float)windowBins;
    for (uint32_t i = 0; i < windowBins; i++) {
      binGal[i] += per;
      if (bucketGalMissed && i < bucketSamplesCap) bucketGalMissed[i] += per;
    }

    // Appendix G.3.4: record the intermediates so a 0.0000 gal restoration can be
    // told apart from a collapse. "collapsed" means real missed pulses converted
    // to no gallons - the restored volume is LOST, not genuinely zero.
    out.missed.applied            = true;
    out.missed.missedPulses       = missedPulses;
    out.missed.missedCaptures     = missedCaptures;
    out.missed.spanCaps           = spanCaps;
    out.missed.spanCapsFromMissed = (missedCaptures > 0u);   // Appendix G.1.6 source
    out.missed.perCapturePulses   = perCapturePulses;
    out.missed.clamped            = clamped;
    out.missed.collapsed          = (missedPulses > 0u && missedGal <= 0.0f);
    out.missed.missedGallons      = missedGal;
    out.missed.perBucketGallons   = per;
    out.missed.windowBins         = windowBins;
  }

  // ---- 5. split into completed buckets and one carried remainder ----------
  // A bucket is complete only once the span has passed its closing boundary.
  uint32_t completed = 0u;
  for (uint32_t i = 0; i < windowBins; i++) {
    uint32_t binEnd = windowFirstBin + (i + 1u) * bucketSec;
    if (binEnd <= endLocal) completed = i + 1u; else break;
  }

  out.baseLocal     = windowFirstBin;
  out.count         = (uint16_t)completed;
  out.totalMakeable = (nBins > 0u) ? (nBins - (windowBins - completed)) : 0u;
  for (uint32_t i = 0; i < completed; i++) { out.gal[i] = binGal[i]; out.pulses[i] = binPul[i]; }

  // Whatever is left over becomes the next report's starting point.
  hourlyCarryClear(carry);
  if (completed < windowBins) {
    uint32_t idx = completed;                    // the single still-open bucket
    carry.valid         = 1u;
    carry.binStartLocal = windowFirstBin + idx * bucketSec;
    carry.pulses        = binPul[idx];
    carry.gallons       = binGal[idx];
    carry.bucketSec     = bucketSec;
    // Anything beyond the open bucket cannot exist: bins are contiguous and only
    // the newest can be unfinished. Fold any residue in defensively anyway.
    for (uint32_t i = idx + 1u; i < windowBins; i++) {
      carry.pulses  += binPul[i];
      carry.gallons += binGal[i];
    }
  }
  carry.lastEndLocal = endLocal;
  carry.bucketSec    = bucketSec;
  if (!carry.valid) {                            // span ended exactly on a boundary
    carry.binStartLocal = endLocal;
    carry.pulses        = 0u;
    carry.gallons       = 0.0f;
    carry.valid         = 1u;
  }

  return true;
}
