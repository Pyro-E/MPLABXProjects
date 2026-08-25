/*
 * hourly_test.cpp  -  Host-only self-test for the binning engine.
 *
 * Not part of the firmware build. Compiled and run by tools/hostcheck/check.sh
 * against the Particle stubs, so the time-axis arithmetic can be exercised
 * without hardware. Every case here corresponds to a documented requirement.
 *
 * Build/run:  tools/hostcheck/run_tests.sh
 */

#include "hourly.h"
#include "flow_cal.h"
#include <stdio.h>
#include <math.h>

static int  g_fail = 0;
static void check(bool ok, const char *what) {
  printf("  %-62s %s\n", what, ok ? "PASS" : "FAIL");
  if (!ok) g_fail++;
}
static bool near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

// Deliberately linear so the expected totals can be reasoned about by hand:
// one pulse = one gallon, regardless of duration.
static float testGallons(uint16_t pulses, float dtSec) { (void)dtSec; return (float)pulses; }

static PicReportInfo mkInfo(uint32_t start, uint8_t valid, uint32_t n,
                            uint32_t caps, uint32_t imp, uint32_t intervalMs) {
  PicReportInfo i;
  memset(&i, 0, sizeof(i));
  i.startTime = start; i.startTimeValid = valid; i.sampleCount = n;
  i.totalCaptures = caps; i.totalImpulses = imp; i.sampleIntervalMs = intervalMs;
  return i;
}

int main() {
  printf("hourly engine self-test\n");

  // ---- 1. A clean 6-hour report with 1-hour buckets -----------------------
  // 6 samples, one per hour, span 06:00..12:00 on the hour. Every bucket closes
  // inside the span, so all 6 are complete and nothing is carried.
  {
    const uint32_t H = 3600;
    const uint32_t start = 6 * H, end = 12 * H;
    PicSample s[6];
    for (int i = 0; i < 6; i++) s[i].pulses = (uint16_t)(i + 1);   // 1..6 gallons
    PicReportInfo info = mkInfo(start, 1, 6, 6, 21, H * 1000);

    HourlyCarry c; hourlyCarryClear(c);
    HourlyResult r;
    bool ok = hourlyProcess(s, 6, info, start, end, H, (float)H, testGallons,
                            PIC_MISSED_FILL_ZERO, c, r);
    check(ok, "clean report: processed");
    check(r.count == 6, "clean report: 6 completed buckets");
    check(r.baseLocal == start, "clean report: base = span start");
    check(near(r.gal[0], 1.0f, 0.001f) && near(r.gal[5], 6.0f, 0.001f),
          "clean report: first/last bucket totals");
    float sum = 0; for (int i = 0; i < r.count; i++) sum += r.gal[i];
    check(near(sum + c.gallons, 21.0f, 0.01f), "clean report: volume conserved (21 gal)");
  }

  // ---- 2. Bucket count is NOT fixed --------------------------------------
  // Same 6 hours of water, but the span starts 30 minutes past the hour. The
  // series now touches 7 clock-aligned buckets: 6 complete and one still open.
  // This is exactly the "47 or 49 instead of 48" behaviour the spec warns about.
  {
    const uint32_t H = 3600;
    const uint32_t start = 6 * H + 1800, end = 12 * H + 1800;
    PicSample s[6];
    for (int i = 0; i < 6; i++) s[i].pulses = 10;
    PicReportInfo info = mkInfo(start, 1, 6, 6, 60, H * 1000);

    HourlyCarry c; hourlyCarryClear(c);
    HourlyResult r;
    hourlyProcess(s, 6, info, start, end, H, (float)H, testGallons,
                  PIC_MISSED_FILL_ZERO, c, r);
#if BUCKET_ALIGN_MODE == BUCKET_ALIGN_CLOCK
    // Clock alignment: the span covers 7 clock buckets, the last one still open.
    check(r.count == 6, "offset span: 6 complete buckets from a 6-sample report");
    check(c.valid && c.binStartLocal == 12 * H,
          "offset span: the open bucket is carried, not published");
#else
    // From-now alignment: boundaries are measured back from the span end, so the
    // newest bucket closes exactly at 12:30 and all 6 are complete.
    check(r.count == 6, "offset span (from-now): all 6 buckets close inside the span");
    check(r.baseLocal + 6u * H == end, "offset span (from-now): last bucket ends at T_end");
#endif
    float sum = 0; for (int i = 0; i < r.count; i++) sum += r.gal[i];
    check(near(sum + c.gallons, 60.0f, 0.01f), "offset span: volume conserved (60 gal)");
  }

  // ---- 3. The carry is merged by the next report -------------------------
  // Report A ends mid-bucket; report B continues from that exact seam. The
  // bucket straddling the two reports must contain the water from BOTH.
  {
    const uint32_t H = 3600;
    HourlyCarry c; hourlyCarryClear(c);

    PicSample a[2]; a[0].pulses = 5; a[1].pulses = 5;             // 06:00..07:00 + half
    PicReportInfo ia = mkInfo(6 * H, 1, 2, 2, 10, 1800 * 1000);
    HourlyResult ra;
    hourlyProcess(a, 2, ia, 6 * H, 7 * H + 1800, H, 1800.0f, testGallons,
                  PIC_MISSED_FILL_ZERO, c, ra);
#if BUCKET_ALIGN_MODE == BUCKET_ALIGN_CLOCK
    check(ra.count == 1 && near(ra.gal[0], 5.0f, 0.01f),
          "carry: report A publishes only its one closed bucket");
    check(c.valid && near(c.gallons, 5.0f, 0.01f) && c.binStartLocal == 7 * H,
          "carry: 5 gal held in the open 07:00 bucket");
#endif
    float totalA = 0; for (int i = 0; i < ra.count; i++) totalA += ra.gal[i];
    check(near(totalA + c.gallons, 10.0f, 0.01f),
          "carry: report A conserves its 10 gal (published + carried)");

    PicSample b[2]; b[0].pulses = 3; b[1].pulses = 3;             // continues to 08:30
    PicReportInfo ib = mkInfo(7 * H + 1800, 1, 2, 2, 6, 1800 * 1000);
    HourlyResult rb;
    hourlyProcess(b, 2, ib, 7 * H + 1800, 8 * H + 1800, H, 1800.0f, testGallons,
                  PIC_MISSED_FILL_ZERO, c, rb);
#if BUCKET_ALIGN_MODE == BUCKET_ALIGN_CLOCK
    check(rb.baseLocal == 7 * H, "carry: report B reopens the carried bucket");
    check(rb.count == 1 && near(rb.gal[0], 8.0f, 0.01f),
          "carry: the 07:00 bucket totals 5+3 = 8 gal across the seam");
#endif
    // Mode-independent invariant: across the seam nothing is lost or counted
    // twice. How the 16 gallons SPLIT between the two reports depends on the
    // alignment - in from-now mode report A's boundary lands on its own span end,
    // so it carries nothing forward - but the sum is fixed either way.
    float totalB = 0; for (int i = 0; i < rb.count; i++) totalB += rb.gal[i];
    check(near(totalA + totalB + c.gallons, 16.0f, 0.01f),
          "carry: A + B + final carry conserve all 16 gal across the seam");
  }

  // ---- 4. A gap breaks continuity, and the stale carry is not merged ------
  {
    const uint32_t H = 3600;
    HourlyCarry c; hourlyCarryClear(c);
    c.valid = 1; c.binStartLocal = 7 * H; c.gallons = 5.0f; c.pulses = 5;
    c.lastEndLocal = 7 * H + 1800; c.bucketSec = H;

    check(!hourlyCarryUsable(c, 20 * H, H, 120u),
          "gap: a carry 13 hours stale is refused");
    check(hourlyCarryUsable(c, 7 * H + 1800 + 60, H, 120u),
          "gap: a carry within tolerance is accepted");
  }

  // ---- 5. start_time back-computation and continuity snapping -------------
  {
    PicReportInfo bad = mkInfo(0, 0, 10, 10, 100, 60000);   // invalid start, 60 s samples
    const char *src = nullptr;
    uint32_t st = hourlyResolveSpanStart(bad, 100000u, 0u, 120u, &src);
    check(st == 100000u - 600u, "no start_time: back-computed as end - n*interval");

    PicReportInfo good = mkInfo(99400u, 1, 10, 10, 100, 60000);
    st = hourlyResolveSpanStart(good, 100000u, 0u, 120u, &src);
    check(st == 99400u, "valid start_time: used as given");

    // Previous report ended at 99390; the PIC says 99400. Ten seconds apart is
    // within tolerance, so the two reports are snapped to one seam.
    st = hourlyResolveSpanStart(good, 100000u, 99390u, 120u, &src);
    check(st == 99390u, "continuity: snapped to the previous report's end");
  }

  // ---- 6. FIFO overrun: volume is restored from the hardware totals -------
  // The ring kept 4 of 8 captures, but total_impulses still describes all 8.
  // AVERAGE mode must put the missing 40 gallons back.
  {
    const uint32_t H = 3600;
    PicSample s[4]; for (int i = 0; i < 4; i++) s[i].pulses = 10;   // 40 received
    PicReportInfo info = mkInfo(6 * H, 1, 4, 8, 80, H * 1000);      // 80 truly happened

    HourlyCarry c; hourlyCarryClear(c);
    HourlyResult r;
    hourlyProcess(s, 4, info, 6 * H, 10 * H, H, (float)H, testGallons,
                  PIC_MISSED_FILL_AVERAGE, c, r);
    float sum = 0; for (int i = 0; i < r.count; i++) sum += r.gal[i];
    check(near(sum + c.gallons, 80.0f, 0.5f),
          "overrun: total restored to 80 gal from totalImpulses");
    check(r.missedGallons > 0.0f, "overrun: the reconstruction is reported");
  }

  // ---- 7. A long outage is truncated newest-first, and says so ------------
  {
    const uint32_t H = 3600;
    const uint32_t n = 400;                                    // 400 hourly samples
    static PicSample s[400];
    for (uint32_t i = 0; i < n; i++) s[i].pulses = 1;
    PicReportInfo info = mkInfo(0, 1, n, n, n, H * 1000);

    HourlyCarry c; hourlyCarryClear(c);
    HourlyResult r;
    hourlyProcess(s, n, info, 0u, n * H, H, (float)H, testGallons,
                  PIC_MISSED_FILL_ZERO, c, r);
    check(r.count <= HOURLY_MAX_BUCKETS, "long outage: never exceeds the array");
    check(r.totalMakeable > r.count,
          "long outage: makeable > sent, so the truncation is visible");
    check(r.baseLocal > 0u, "long outage: the OLDEST buckets are the ones dropped");
  }

  // ---- 8. V059: a retransmitted / replayed batch is not billed twice ------
  // A lost PKT_DATA_RECEIVED (0x0B) makes the PIC resend a span it already
  // delivered; a failed cloud publish makes replayBufferedReports() re-ingest a
  // block that was already placed. Before V059 the overlap was binned a second
  // time and the retained accumulators added it with "+=", so six hours of water
  // were billed twice. These cases pin the arithmetic that prevents it.
  {
    const uint32_t H  = 3600;
    const uint32_t T0 = 100u * 24u * H;
    const uint32_t T1 = T0 +  6u * H;                 // end of the first report
    const uint32_t T2 = T0 + 12u * H;                 // end of the retransmission

    check(hourlyOverlapSkip(12u, T0, T2, T1) == 6u,
          "overlap: a 12-sample resend over a 6-hour seam trims 6");
    check(hourlyOverlapSkip(6u, T0, T1, T1) == 6u,
          "overlap: an exact retransmission trims the whole batch");
    check(hourlyOverlapSkip(12u, T0, T2, 0u) == 0u,
          "overlap: no seam recorded yet -> nothing is trimmed");
    check(hourlyOverlapSkip(12u, T1, T2, T1) == 0u,
          "overlap: a batch starting on the seam is untouched");

    // The full path, mirroring ingestReport(): place T0..T1, remember the totals
    // credited against the PIC's mark, then hand it the T0..T2 retransmission
    // with the prefix trimmed and those totals subtracted.
    PicSample a[6];  for (int i = 0; i < 6;  i++) a[i].pulses = 1;
    PicSample b[12]; for (int i = 0; i < 12; i++) b[i].pulses = 1;

    // Case A: the PIC still holds every sample and resends all 12.
    {
      HourlyCarry  c;  hourlyCarryClear(c);
      HourlyResult r1, r2;
      PicReportInfo first = mkInfo(T0, 1, 6, 6, 6, H * 1000);
      hourlyProcess(a, 6, first, T0, T1, H, (float)H, testGallons,
                    PIC_MISSED_FILL_AVERAGE, c, r1);
      float total = 0; for (int i = 0; i < r1.count; i++) total += r1.gal[i];
      uint32_t creditedImp = first.totalImpulses;   // retained prevCreditedImpulses
      uint32_t creditedCap = first.totalCaptures;

      uint32_t skip = hourlyOverlapSkip(12u, T0, T2, T1);
      PicReportInfo eff = mkInfo(T1, 1, 12u - skip,
                                 12u - creditedCap, 12u - creditedImp, H * 1000);
      hourlyProcess(b + skip, 12u - skip, eff, T1, T2, H, (float)H, testGallons,
                    PIC_MISSED_FILL_AVERAGE, c, r2);
      for (int i = 0; i < r2.count; i++) total += r2.gal[i];
      total += c.gallons;
      check(near(total, 12.0f, 0.01f),
            "overlap: retransmitted span is billed once (12 gal, not 18)");
    }

    // Case B: the PIC's ring dropped the overlapping samples, so the resend
    // carries only 3 while total_impulses still says 12. Subtracting the pulses
    // of the dropped SAMPLES would leave 9 and missed-fill would restore the
    // overlap again; subtracting what was already credited leaves 6, which is
    // exactly the new water.
    {
      HourlyCarry  c;  hourlyCarryClear(c);
      HourlyResult r1, r2;
      PicReportInfo first = mkInfo(T0, 1, 6, 6, 6, H * 1000);
      hourlyProcess(a, 6, first, T0, T1, H, (float)H, testGallons,
                    PIC_MISSED_FILL_AVERAGE, c, r1);
      float total = 0; for (int i = 0; i < r1.count; i++) total += r1.gal[i];
      uint32_t creditedImp = first.totalImpulses;
      uint32_t creditedCap = first.totalCaptures;

      uint32_t skip = hourlyOverlapSkip(6u, T0, T2, T1);
      check(skip == 3u, "overlap: a ring-thinned resend trims by time, not by count");
      PicReportInfo eff = mkInfo(T1, 1, 6u - skip,
                                 12u - creditedCap, 12u - creditedImp, H * 1000);
      hourlyProcess(a + skip, 6u - skip, eff, T1, T2, H, (float)H, testGallons,
                    PIC_MISSED_FILL_AVERAGE, c, r2);
      for (int i = 0; i < r2.count; i++) total += r2.gal[i];
      total += c.gallons;
      check(near(total, 12.0f, 0.01f),
            "overlap: ring-dropped resend does not re-restore via missed-fill");
    }
  }

  // ---- Appendix E.4 (c): per-bucket sample counts -------------------------
  // The optional bucketSamples[] output must count each batch sample into the
  // bucket that contains its end instant, so the per-bucket "samples=" figure in
  // the [HRLY] log is verifiable. 8 samples over 4 clock-aligned hours land 2
  // per bucket; the counts across completed buckets plus the partial must total
  // exactly the samples placed.
  {
    const uint32_t H = 3600;
    const uint32_t start = 6 * H, end = 10 * H;    // 06:00..10:00, 4 whole buckets
    PicSample s[8];
    for (int i = 0; i < 8; i++) s[i].pulses = 5;
    PicReportInfo info = mkInfo(start, 1, 8, 8, 40, H * 1000);

    HourlyCarry c; hourlyCarryClear(c);
    HourlyResult r;
    uint16_t bs[HOURLY_MAX_BUCKETS];
    bool ok = hourlyProcess(s, 8, info, start, end, H, (float)H / 2.0f, testGallons,
                            PIC_MISSED_FILL_ZERO, c, r, bs, HOURLY_MAX_BUCKETS);
    check(ok && r.count == 4, "bucketSamples: 4 completed buckets");
    check(bs[0] == 2 && bs[1] == 2 && bs[2] == 2 && bs[3] == 2,
          "bucketSamples: 2 samples per bucket");
    uint32_t placed = 0; for (uint32_t i = 0; i < r.count; i++) placed += bs[i];
    check(placed == 8, "bucketSamples: counts total the samples placed");
  }

  // ---- Appendix E.4 (c): nullptr output is still safe ---------------------
  // Passing no bucketSamples array (the default) must behave exactly as before.
  {
    const uint32_t H = 3600;
    PicSample s[4]; for (int i = 0; i < 4; i++) s[i].pulses = 3;
    PicReportInfo info = mkInfo(6 * H, 1, 4, 4, 12, H * 1000);
    HourlyCarry c; hourlyCarryClear(c);
    HourlyResult r;
    bool ok = hourlyProcess(s, 4, info, 6 * H, 10 * H, H, (float)H, testGallons,
                            PIC_MISSED_FILL_ZERO, c, r);   // no bucketSamples arg
    check(ok && r.count == 4, "bucketSamples: omitting the array is safe (nullptr)");
  }

  // ---- Appendix E supplement B: OVFLOW clamp reconciliation ---------------
  // The OVFLOW bench phase drives ~105,800 pulses into a 5.29 s capture. A 16-bit
  // sample cell clamps at 0xFFFF (65535) while total_impulses keeps the true
  // count, so total_impulses >> sum(samples). AVERAGE missed-fill must restore
  // the difference without the per-capture value wrapping a uint16_t.
  {
    const uint32_t H = 3600;
    const uint32_t start = 6 * H, end = 10 * H;     // 4 whole buckets
    PicSample s[4];
    for (int i = 0; i < 4; i++) s[i].pulses = 65535;   // every sample clamped
    // true ~105,800/capture; missed per capture = 105800 - 65535 = 40265.
    const uint32_t truePer = 105800;
    PicReportInfo info = mkInfo(start, 1, 4, 4, truePer * 4u, H * 1000);  // caps==n: clamp, no overrun

    HourlyCarry c; hourlyCarryClear(c);
    HourlyResult r;
    bool ok = hourlyProcess(s, 4, info, start, end, H, (float)H, testGallons,
                            PIC_MISSED_FILL_AVERAGE, c, r);
    uint32_t missed = truePer * 4u - 65535u * 4u;        // 161060
    check(ok, "OVFLOW clamp: processed");
    check(near(r.missedGallons, (float)missed, 1.0f),
          "OVFLOW clamp: missed volume = total - sum(clamped), no wrap");
  }

  // ---- Appendix E supplement B: OVFLOW with ring overrun ------------------
  // Now the ring ALSO overran, so whole OVFLOW captures were dropped. The
  // per-capture average of the missed pulses (~105,800) exceeds 65535, which
  // would WRAP a naive uint16_t cast (105800 -> 40264). The clamp must hold it
  // at 65535 so the restored volume stays monotonic, not corrupted.
  {
    const uint32_t H = 3600;
    const uint32_t start = 6 * H, end = 10 * H;
    PicSample s[2]; s[0].pulses = 10; s[1].pulses = 10;   // only 2 survived
    const uint32_t truePer = 105800;
    // 10 captures in the period, 8 dropped; totals keep all of them.
    PicReportInfo info = mkInfo(start, 1, 2, 10, truePer * 10u, H * 1000);

    HourlyCarry c; hourlyCarryClear(c);
    HourlyResult r;
    bool ok = hourlyProcess(s, 2, info, start, end, H, (float)H, testGallons,
                            PIC_MISSED_FILL_AVERAGE, c, r);
    // spanCaps = missedCaptures = 8; per-capture clamps to 65535 -> 65535 * 8.
    check(ok, "OVFLOW+overrun: processed");
    check(near(r.missedGallons, 65535.0f * 8.0f, 1.0f),
          "OVFLOW+overrun: per-capture clamps at 65535, restore does not wrap");
    check(r.missedGallons > 65535.0f * 4.0f,
          "OVFLOW+overrun: restored volume is monotonic (wrap would undercount)");
  }

  // ================= Appendix G host regression tests (V062) ================

  // ---- G.6.1: gallon conservation over random inputs ----------------------
  // For any sample series / span / bucketSec, the report must conserve volume:
  //   sum(completed gal) + carryOut - carryIn == sampleGallons + missedGallons
  // This is the identity the [HRLY] CHECK line asserts at runtime (G.3.6); here
  // it is pinned across 200 pseudo-random cases so a future edit cannot break it.
  //
  // Appendix H.8: the pre-V063 loop cleared the carry every iteration (carryIn
  // always 0) and set totalImpulses == sum (missed-fill never ran), so the two
  // terms most able to break conservation were never exercised. The carry now
  // persists ACROSS iterations (a real seam), and totalImpulses / totalCaptures
  // are seeded above the sample sum so missed-fill distributes a real remainder.
  {
    uint32_t seed = 0x12345678u;
    int worst = 0; float worstDelta = 0.0f;
    HourlyCarry c; hourlyCarryClear(c);               // H.8: carry lives outside the loop -> real carry-in
    for (int t = 0; t < 200; t++) {
      seed = seed * 1103515245u + 12345u;
      uint32_t nn      = 1u + (seed >> 8) % 40u;
      seed = seed * 1103515245u + 12345u;
      uint32_t H       = 30u + (seed >> 7) % 3600u;      // bucketSec 30..3629
      seed = seed * 1103515245u + 12345u;
      uint32_t spanMul = 1u + (seed >> 9) % 8u;          // span = spanMul buckets-ish
      uint32_t start   = 100000u + ((seed >> 3) % 1000u) * H;
      uint32_t end     = start + spanMul * H + ((seed >> 5) % H);

      static PicSample s[64];
      uint32_t sum = 0u;
      for (uint32_t i = 0; i < nn; i++) {
        seed = seed * 1103515245u + 12345u;
        s[i].pulses = (uint16_t)((seed >> 6) % 500u);
        sum += s[i].pulses;
      }
      // H.8: seed totalImpulses/totalCaptures ABOVE the sample sum so the missed-
      // fill path distributes a genuine remainder (ring-overrun reconstruction).
      seed = seed * 1103515245u + 12345u;
      uint32_t extraImp  = (seed >> 7) % 250u;           // 0..249 missed pulses
      seed = seed * 1103515245u + 12345u;
      uint32_t extraCaps = (seed >> 9) % 6u;             // 0..5 missed captures
      uint32_t totImp    = sum + extraImp;
      uint32_t totCaps   = nn + extraCaps;
      PicReportInfo info = mkInfo(start, 1, nn, totCaps, totImp,
                                  (H * 1000u) / (nn ? nn : 1u));

      HourlyResult r;
      if (!hourlyProcess(s, nn, info, start, end, H, (float)H / (float)nn,
                         testGallons, PIC_MISSED_FILL_AVERAGE, c, r)) continue;

      float sumGal = 0.0f; for (uint16_t i = 0; i < r.count; i++) sumGal += r.gal[i];
      float lhs = sumGal + c.gallons - r.carryInGallons;
      float rhs = r.sampleGallons + r.missedGallons;
      float d   = fabsf(lhs - rhs);
      float tol = 1e-4f; if (rhs * 1e-4f > tol) tol = rhs * 1e-4f;
      if (d > tol) { worst++; if (d > worstDelta) worstDelta = d; }
    }
    check(worst == 0, "G.6.1: gallon conservation holds over 200 random cases (H.8: carry-in + missed-fill live)");
  }

  // ---- G.6.2 + G.1.7: carry round-trip, including binStartLocal > firstBin --
  // Report A's carry-out must reappear as report B's carry-in, and the volume in
  // the carried bucket must survive the seam even when the resolved firstBin of
  // report B lands one bucket AHEAD of the carry (the Appendix G.1.7 loss). The
  // pre-fix engine dropped the carry in that case; the fix must keep it.
  {
    const uint32_t H = 3600;
    HourlyCarry c; hourlyCarryClear(c);

    // Report A leaves an open bucket at 07:00 with 5 gal / 5 pulses.
    PicSample a[2]; a[0].pulses = 5; a[1].pulses = 5;
    PicReportInfo ia = mkInfo(6 * H, 1, 2, 2, 10, 1800 * 1000);
    HourlyResult ra;
    hourlyProcess(a, 2, ia, 6 * H, 7 * H + 1800, H, 1800.0f, testGallons,
                  PIC_MISSED_FILL_ZERO, c, ra);
    float carriedGal = c.gallons; uint32_t carriedPul = c.pulses;
    uint32_t carriedBin = c.binStartLocal;
    check(c.valid && carriedBin == 7 * H, "G.6.2: report A carries the open 07:00 bucket");

    // Report B is resolved so its span start floors to 06:00 - i.e. firstBin
    // (06:00) is BEHIND the carry bucket (07:00): carry.binStartLocal > firstBin.
    // Its start (06:40) is still within the carry's bucketSec tolerance, so the
    // carry is USABLE; the pre-fix engine dropped it anyway because the single
    // "carry.binStartLocal <= firstBin" gate was false. With the G.1.7 fix the
    // carry is merged into the 07:00 window bucket instead of being lost. This is
    // exactly the tolerance-mismatch case the appendix describes (carryUsable
    // tolerances by bucketSec; the span-start snap by SPAN_CONTINUITY_TOL_SEC).
    PicSample b[3]; b[0].pulses = 2; b[1].pulses = 2; b[2].pulses = 2;
    PicReportInfo ib = mkInfo(6 * H + 2400, 1, 3, 3, 6, 1800 * 1000);
    HourlyResult rb;
    hourlyProcess(b, 3, ib, 6 * H + 2400, 7 * H + 1800, H, 1800.0f, testGallons,
                  PIC_MISSED_FILL_ZERO, c, rb);
#if BUCKET_ALIGN_MODE == BUCKET_ALIGN_CLOCK
    check(rb.carryMerged, "G.1.7: carry merged even when firstBin is behind the carry bucket");
    check(fabsf(rb.carryInGallons - carriedGal) < 0.01f &&
          rb.carryInPulses == carriedPul,
          "G.1.7: report B's carry-in equals report A's carry-out");
#endif
    // Mode-independent: no volume is created or lost across the seam.
    float totalA = 0; for (uint16_t i = 0; i < ra.count; i++) totalA += ra.gal[i];
    float totalB = 0; for (uint16_t i = 0; i < rb.count; i++) totalB += rb.gal[i];
    check(near(totalA + totalB + c.gallons, 16.0f, 0.01f),
          "G.1.7: A + B + final carry conserve all 16 gal across the seam");
  }

  // ---- G.6.4: missed-fill nonlinearity is recorded, not hidden ------------
  // A nonlinear toGallons makes "average then multiply" differ from "convert each
  // then sum". This test drives the same total missed pulses two ways and records
  // that the AVERAGE reconstruction is the average-based figure (Appendix G.1.2),
  // and that the trace flags a collapse when the per-capture average is out of
  // range. Uses a deliberately nonlinear gallons function local to this case.
  {
    const uint32_t H = 3600;
    const uint32_t start = 6 * H, end = 10 * H;
    // 4 captures, ring dropped 4 more; total_impulses says all 8 happened.
    PicSample s[4]; for (int i = 0; i < 4; i++) s[i].pulses = 10;
    PicReportInfo info = mkInfo(start, 1, 4, 8, 80, H * 1000);
    HourlyCarry c; hourlyCarryClear(c);
    HourlyResult r;
    hourlyProcess(s, 4, info, start, end, H, (float)H, testGallons,
                  PIC_MISSED_FILL_AVERAGE, c, r);
    check(r.missed.applied, "G.6.4: missed-fill trace records that AVERAGE ran");
    check(r.missed.spanCaps == 4 && r.missed.spanCapsFromMissed,
          "G.6.4: spanCaps sourced from missedCaptures when the ring overran (G.1.6)");
    check(!r.missed.collapsed && r.missed.missedGallons > 0.0f,
          "G.6.4: in-range missed pulses restore a positive volume");
  }

  // ---- G.6.4b: an OVFLOW-magnitude average is flagged as collapsed --------
  // When the per-capture average is far out of calibration range, a REAL flow
  // polynomial would return 0 (Appendix G.1.1) and the restored volume is LOST.
  // The linear testGallons never collapses, so this uses a small clamp-style
  // gallons function that returns 0 above a threshold to exercise the flag.
  {
    const uint32_t H = 3600;
    const uint32_t start = 6 * H, end = 10 * H;
    PicSample s[2]; s[0].pulses = 10; s[1].pulses = 10;
    // 2 kept, 8 dropped, each dropped capture ~100000 pulses: average >> range.
    PicReportInfo info = mkInfo(start, 1, 2, 10, 100000u * 10u, H * 1000);
    HourlyCarry c; hourlyCarryClear(c);
    HourlyResult r;
    // gallons that collapse to 0 above 1000 pulses (stand-in for the calibration
    // collapse); a captures-9x100000 average clamps to 65535 -> 0 gal.
    hourlyProcess(s, 2, info, start, end, H, (float)H,
                  [](uint16_t p, float) -> float { return p > 1000u ? 0.0f : (float)p; },
                  PIC_MISSED_FILL_AVERAGE, c, r);
    check(r.missed.applied && r.missed.collapsed,
          "G.6.4b: an out-of-range per-capture average is flagged collapsed (volume LOST)");
  }

  // ---- G.6.3: the calibration collapse point is pinned --------------------
  // freqToGpm() rises to a peak near 69-70 Hz and collapses to 0 above ~109 Hz
  // (Appendix G.1.1). This uses the REAL polynomial from flow_cal.h, so a change
  // to any FLOW_C* coefficient that silently moves the usable range is caught by
  // the fixed expected bands below rather than passing unnoticed.
  {
    const float scale = 1.2340f;   // the reference scale used throughout Appendix G
    // Spot values from the appendix table (scale-adjusted where it multiplies).
    check(flowFreqToGpm(20.038f, scale) > 0.75f && flowFreqToGpm(20.038f, scale) < 0.85f,
          "G.6.3: ~20 Hz is in-range (about 0.79 GPM)");
    check(flowFreqToGpm(12388.5f, scale) == 0.0f,
          "G.6.3: a 0xFFFF-magnitude frequency returns 0 GPM (collapsed)");

    FlowValidRange r = flowComputeValidRange(scale);
    check(r.collapses, "G.6.3: the polynomial collapses at a finite frequency");
    check(r.validMaxHz > 105.0f && r.validMaxHz < 112.0f,
          "G.6.3: the collapse sits near 109 Hz (coefficient guard)");
    check(r.peakFreqHz > 65.0f && r.peakFreqHz < 75.0f,
          "G.6.3: the peak sits near 69-70 Hz");
    check(r.peakGpm > 1.85f && r.peakGpm < 1.95f,
          "G.6.3: the peak is about 1.92 GPM at scale 1.234");
  }

  // ================= Appendix H host regression tests (V063) ================

  // ---- H.4: the sec 6.7 back-computation keeps millisecond precision -------
  // The pre-V063 back-computation did info.sampleIntervalMs / 1000u, truncating
  // 5207 ms to 5 s on every block; a 35-sample block then spanned 175 s instead
  // of 182.245 s, and a long outage compounded the error. hourlyResolveSpanStart()
  // must now span the run from the millisecond interval and round only once.
  {
    // 35 samples at 5207 ms = 182.245 s -> rounds to 182 s (not 175).
    PicReportInfo info = mkInfo(0, /*startValid=*/0, 35, 35, 35, 5207u);
    const uint32_t endLocal = 1000000u;
    const char *src = nullptr;
    uint32_t startLocal = hourlyResolveSpanStart(info, endLocal, /*prevEnd=*/0u,
                                                 /*tol=*/0u, &src);
    uint32_t span = endLocal - startLocal;
    check(span == 182u, "H.4: 35 x 5207 ms back-computes to a 182 s span, not 175 s");
    // A block of 100 samples at 5290 ms = 529.0 s exactly (no rounding error).
    PicReportInfo info2 = mkInfo(0, 0, 100, 100, 100, 5290u);
    uint32_t s2 = hourlyResolveSpanStart(info2, 2000000u, 0u, 0u, nullptr);
    check((2000000u - s2) == 529u, "H.4: 100 x 5290 ms back-computes to exactly 529 s");
  }

  // ---- H.4b: a reconstructed span equalizes on the exact interval ----------
  // With reconstructedSpan=true the fabricated whole-second endpoints are not a
  // real measurement, so equalize must use the exact nominal (ms) step. Here the
  // span rounds to 182 s but the true cadence is 5.207 s; perSampleSec must be
  // 5.207 s, not 182/35 = 5.2 s.
  {
    PicSample s[35]; for (int i = 0; i < 35; i++) s[i].pulses = 1;
    PicReportInfo info = mkInfo(0, 0, 35, 35, 35, 5207u);
    HourlyCarry c; hourlyCarryClear(c);
    HourlyResult r;
    bool ok = hourlyProcess(s, 35, info, 818u, 1000u, 60u, 5.207f, testGallons,
                            PIC_MISSED_FILL_ZERO, c, r,
                            nullptr, 0u, nullptr, nullptr, /*reconstructedSpan=*/true);
    check(ok, "H.4b: a reconstructed span is processed");
    check(near(r.perSampleSec, 5.207f, 0.0005f),
          "H.4b: equalize uses the exact 5.207 s interval, not the rounded span/n");
    check(!r.equalized, "H.4b: a reconstructed step is marked non-measured");
  }

  // ---- H.3.3: the forbidden 2000-01-01 band never seeds a reconstruction ---
  // The whole H.3 family exists to keep the unsynced Particle default
  // (UTC 946684800, local 946655828 with the -28800 offset) out of the data
  // path. The device-side guarantee is that clockNowUtc() gates every read; here
  // we pin the reconstruction primitive that USES the anchor: given a real "now",
  // no back-computed start may land inside 946684800 +/- 86400. (A fake anchor
  // could only arrive from a fabricated clock, which H.3 forbids at the source.)
  {
    const uint32_t BAND_LO = 946684800u - 86400u;
    const uint32_t BAND_HI = 946684800u + 86400u;
    // Real anchor "now" = 2026-08-14-ish; a run of blocks reconstructed back from it.
    const uint32_t nowReal = 1786665600u;
    uint32_t cursorMs = (uint32_t)0;   // relative walk; only the epoch matters here
    (void)cursorMs;
    bool anyInBand = false;
    uint32_t endLocal = nowReal;
    for (int blk = 0; blk < 8; blk++) {
      PicReportInfo info = mkInfo(0, 0, 34, 34, 34, 5290u);
      uint32_t start = hourlyResolveSpanStart(info, endLocal, 0u, 0u, nullptr);
      if (start >= BAND_LO && start <= BAND_HI) anyInBand = true;
      endLocal = start;   // chain the next block back-to-back
    }
    check(!anyInBand,
          "H.3.3: reconstructed starts anchored at a real now never hit the 2000-01-01 band");
  }

  printf("%s (%d failure%s)\n", g_fail ? "FAILED" : "all checks passed",
         g_fail, g_fail == 1 ? "" : "s");
  return g_fail ? 1 : 0;
}
