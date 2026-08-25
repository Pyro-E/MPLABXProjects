/*
 * hourly.h  -  Time reconstruction and bucket binning for LeakSense (V057).
 *
 * WHAT THIS MODULE IS FOR
 * -----------------------
 * The PIC guarantees the AMOUNT of water (pulses) and nothing else: it has no
 * RTC, so it cannot say exactly when each capture happened. Placing those
 * pulses on a real time axis is the Photon's job, and this module is where that
 * happens. Two rules are absolute:
 *
 *   1. The pulse counts are never altered. Only their placement in time is
 *      computed here. A bucket total is the sum of whole samples, nothing is
 *      invented and nothing is thrown away.
 *   2. The number of buckets a report produces is NOT fixed. A 48 h report with
 *      1 h buckets usually makes 48, but the PIC's clock error and grid
 *      alignment routinely make it 47 or 49, and after a long outage it can be
 *      hundreds. Any code that assumes 24 or 48 is wrong.
 *
 * HOW A REPORT IS PROCESSED
 * -------------------------
 *   span start  T_start : from the RSP_DATA header when valid, else
 *                         T_end - sampleCount * sampleInterval (back-computed),
 *                         else snapped to the previous report's end for continuity.
 *   span end    T_end   : the instant the PIC powered the Photon up, which is
 *                         when the PIC froze the batch. Derived by the caller as
 *                         (cloud time at sync) - (millis at sync), NOT the time
 *                         of publishing.
 *   equalize            : per-sample step = (T_end - T_start) / sampleCount.
 *                         Samples are spread evenly over the REAL elapsed span
 *                         rather than at the nominal interval, because the PIC's
 *                         LFINTOSC is only good to a few percent.
 *   bin                 : each sample is credited to the bucket that contains
 *                         the instant it ENDS. A sample ending exactly on a
 *                         boundary belongs to the bucket that closes there.
 *   carry               : the final bucket is normally incomplete. Its pulses,
 *                         gallons and start time are carried to the next report
 *                         and merged there, so no volume is lost at the seam.
 *
 * BUCKET ALIGNMENT - two schemes, selected by BUCKET_ALIGN_MODE in app_config.h
 *   BUCKET_ALIGN_CLOCK (1): buckets sit on absolute clock multiples of
 *      bucketSec, so with 3600 s they read as "22:00-23:00". Only the newest
 *      bucket can be partial. This is the scheme the dashboard wants.
 *   BUCKET_ALIGN_FROM_NOW (2): buckets are measured backwards from the end of
 *      the span, so the newest bucket always closes exactly at T_end and the
 *      partial bucket is the oldest one. Simpler, but the labels are off-clock
 *      ("22:38-23:38").
 *
 * All epochs handled here are LOCAL epochs (UTC + the configured offset), the
 * same timebase the PIC is given in TIME_SYNC. There is no timezone conversion
 * inside this module.
 */

#pragma once
#include "Particle.h"
#include "app_config.h"
#include "pic_link.h"

// Appendix G.3.4: the intermediate values of the missed-fill (ring-overrun /
// clamp) reconstruction, kept so the log can show WHY a restored figure came out
// the way it did - in particular, whether a 0 gal result is "nothing to restore"
// or a polynomial collapse (Appendix G.1.1/G.1.2). Small (scalars only), so it
// costs nothing to carry in HourlyResult.
struct MissedFillTrace {
  bool     applied;            // AVERAGE mode ran (there was a positive discrepancy)
  uint32_t missedPulses;       // totalImpulses - sum(placed samples)
  uint32_t missedCaptures;     // totalCaptures - n (whole captures the ring dropped)
  uint32_t spanCaps;           // divisor actually used
  bool     spanCapsFromMissed; // true: spanCaps = missedCaptures; false: spanCaps = n (Appendix G.1.6)
  float    perCapturePulses;   // missedPulses / spanCaps, before the 0xFFFF clamp
  bool     clamped;            // per-capture average was clamped at 0xFFFF
  bool     collapsed;          // toGallons() returned 0 for a positive missedPulses (Appendix G.1.1)
  float    missedGallons;      // gallons the reconstruction produced
  float    perBucketGallons;   // missedGallons / windowBins (spread evenly, Appendix G.1.5)
  uint32_t windowBins;         // buckets the spread was divided across
};

// One bucket set produced by hourlyProcess(). gal[i] covers
// [baseLocal + i*bucketSec, baseLocal + (i+1)*bucketSec).
struct HourlyResult {
  uint32_t baseLocal;       // local epoch at the start of gal[0]
  uint32_t bucketSec;       // bucket width used
  uint16_t count;           // completed buckets in gal[] / pulses[]  (this is NN)
  uint32_t totalMakeable;   // buckets this report COULD have produced (this is MM);
                            // > count only when the run exceeded HOURLY_MAX_BUCKETS
  uint32_t spanStartLocal;  // T_start actually used
  uint32_t spanEndLocal;    // T_end actually used
  uint32_t samplesUsed;     // K: samples placed on the axis
  float    perSampleSec;    // equalized step actually used
  bool     equalized;       // true = step measured from the span; false = nominal
  float    missedGallons;   // volume reconstructed from the PIC's totals (ring overrun)
  // Appendix G self-check inputs. sampleGallons is the sum of toGallons() over
  // the samples actually placed in the kept window (NOT the dropped prefix of a
  // truncated long-outage), and placedPulses is the matching pulse sum. Together
  // with carry-in/out they make the conservation identity checkable by the
  // caller (Appendix G.3.6/G.3.7):
  //   sum(completed gal) + carryOut - carryIn == sampleGallons + missedGallons
  float    sampleGallons;   // sum of toGallons() over placed samples (window only)
  uint32_t placedPulses;    // sum of pulses over placed samples (window only)
  float    carryInGallons;  // gallons this report was seeded with (0 if no seam)
  uint32_t carryInPulses;   // pulses this report was seeded with
  bool     carryMerged;     // Appendix G.1.7: the carry seed was actually placed
  MissedFillTrace missed;   // Appendix G.3.4 breakdown
  float    gal[HOURLY_MAX_BUCKETS];
  uint32_t pulses[HOURLY_MAX_BUCKETS];
};

// Seam state between two reports. Persisted, because the PIC cuts our power
// between every pair of reports and this is what keeps the axis continuous.
struct HourlyCarry {
  uint32_t magic;           // HOURLY_CARRY_MAGIC when the record is usable
  uint8_t  valid;           // 1 = binStartLocal / pulses / gallons are meaningful
  uint32_t binStartLocal;   // start of the still-incomplete bucket
  uint32_t pulses;          // pulses accumulated in it so far
  float    gallons;         // gallons accumulated in it so far
  uint32_t lastEndLocal;    // T_end of the report that produced this carry
  uint32_t bucketSec;       // bucket width it was made with (a change invalidates it)
};

#define HOURLY_CARRY_MAGIC 0x48435232UL   /* "HCR2" - bump when the layout changes */

// Converts one sample to gallons. Supplied by the caller because the flow
// calibration polynomial lives with the application, not here.
//   pulses : raw 16-bit count for that capture
//   dtSec  : the equalized duration that capture represents
typedef float (*SampleToGallonsFn)(uint16_t pulses, float dtSec);

// Reset a carry record to "nothing carried".
void hourlyCarryClear(HourlyCarry &c);

// True if 'c' can legitimately be merged into a report that starts at
// spanStartLocal with this bucket width.
bool hourlyCarryUsable(const HourlyCarry &c, uint32_t spanStartLocal,
                       uint32_t bucketSec, uint32_t toleranceSec);

// Decide the span start for a batch, in this order:
//   1. the PIC's start_time when it is flagged valid,
//   2. otherwise T_end - sampleCount * sampleInterval,
//   3. and in either case snap to the previous report's end when the two are
//      within 'toleranceSec', so consecutive reports share one seam.
// 'source' receives a short literal describing which rule fired (for logging).
uint32_t hourlyResolveSpanStart(const PicReportInfo &info, uint32_t endLocal,
                                uint32_t prevEndLocal, uint32_t toleranceSec,
                                const char **source);

// V059: how many leading samples of a batch have ALREADY been placed on the
// axis by an earlier report, and must therefore be dropped before binning.
//
// The PIC only forgets a batch when it sees PKT_DATA_RECEIVED (0x0B). If that
// ack is lost on the wire, or if a flash-buffered block is replayed after a
// failed cloud publish, the same samples arrive a second time. Nothing further
// down the path is idempotent - hourlyProcess() spreads every sample it is
// given over [start, end], and leaksense.cpp's rolling accumulators use "+=" -
// so the overlap must be removed here or the water is billed twice.
//
// Samples are equalized, so the overlap is a prefix of exactly
//   round((prevEndLocal - startLocal) / ((endLocal - startLocal) / n))
// samples. Returns 0 when there is no overlap, and n when the whole batch
// precedes prevEndLocal (an exact retransmission of an already-placed report).
uint32_t hourlyOverlapSkip(uint32_t n, uint32_t startLocal, uint32_t endLocal,
                           uint32_t prevEndLocal);

// Place a batch on the time axis and produce completed buckets.
// The last (incomplete) bucket is written back into 'carry' rather than emitted.
// Returns false only when the batch cannot be placed at all (no samples, or a
// non-positive span).
//
// bucketSamples / bucketSamplesCap (optional, Appendix E.4 verification log):
// when non-null, bucketSamples[i] receives the number of THIS batch's samples
// binned into window bucket i (this report's carry-in seed is NOT counted here;
// it is reported separately as "carry in"). Index 0..out.count-1 are the
// completed buckets and index out.count is the still-open (carried) bucket.
// Pass nullptr to skip; only the first bucketSamplesCap entries are written.
//
// bucketGalSamples / bucketGalMissed (optional, Appendix G.3.5): when non-null,
// each receives the per-bucket gallon split so the [HRLY] bucket line can show
// "samples=X + missed=Y + carryIn=Z". bucketGalSamples[i] is the gallons from
// this batch's own samples in bucket i (no carry seed, no missed-fill);
// bucketGalMissed[i] is the missed-fill volume spread into bucket i. The carry
// seed is not written here - the caller derives carry-in per bucket from the
// carry record. Both arrays are indexed the same as bucketSamples and bounded by
// bucketSamplesCap. Pass nullptr to skip either one.
bool hourlyProcess(const PicSample *s, uint32_t n, const PicReportInfo &info,
                   uint32_t startLocal, uint32_t endLocal, uint32_t bucketSec,
                   float nominalSampleSec, SampleToGallonsFn toGallons,
                   uint8_t missedFillMode, HourlyCarry &carry, HourlyResult &out,
                   uint16_t *bucketSamples = nullptr, uint32_t bucketSamplesCap = 0u,
                   float *bucketGalSamples = nullptr, float *bucketGalMissed = nullptr,
                   // Appendix H.4: true when the span was RECONSTRUCTED from the
                   // sample interval (sec 6.7 no-time recovery) rather than measured
                   // from real endpoints. The fabricated span/n is then a quantized
                   // echo of the interval, so equalize uses the exact nominal
                   // (millisecond) step directly instead of re-deriving span/n.
                   bool reconstructedSpan = false);
