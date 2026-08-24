/*
 * roll48_test.cpp  -  Host-only self-test for the contract's 48-slot window.
 *
 * Not part of the firmware build. Compiled and run by tools/hostcheck/check.sh
 * against the Particle stubs.
 *
 * WHAT IS ACTUALLY BEING TESTED
 * -----------------------------
 * Only one thing is hard about a sliding window, and it is the empty slots. If
 * the window shifted once per arriving bucket, three buckets arriving after a
 * 10-hour outage would move it by 3 and place the newest reading ten hours away
 * from where it belongs - and the payload would look completely normal. Every
 * case below exists to pin the shift to ELAPSED TIME instead, and to check that
 * the slots nobody had data for are counted in g_roll48Gaps rather than passed
 * off as measured zeroes.
 *
 * The cases mirror the verification table in the V068 request document, so the
 * numbers there can be compared against a run of this suite directly.
 *
 * Build/run:  tools/hostcheck/run_tests.sh
 */

#include "roll48.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

static int  g_fail = 0;
static void check(bool ok, const char *what) {
  printf("  %-62s %s\n", what, ok ? "PASS" : "FAIL");
  if (!ok) g_fail++;
}
static bool near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

static const uint32_t H = 3600u;          // production bucket width

// Build a finished report of 'n' consecutive buckets starting at 'baseLocal',
// with gal[i] = first + i. Only the fields roll48Apply() reads are set.
static HourlyResult mkResult(uint32_t baseLocal, uint32_t bucketSec,
                             uint16_t n, float first) {
  HourlyResult r;
  memset(&r, 0, sizeof(r));
  r.baseLocal = baseLocal;
  r.bucketSec = bucketSec;
  r.count     = n;
  for (uint16_t i = 0; i < n && i < HOURLY_MAX_BUCKETS; i++) {
    r.gal[i]    = first + (float)i;
    r.pulses[i] = (uint32_t)(first + (float)i);
  }
  return r;
}

int main() {
  printf("48-slot window self-test (V068 contract array)\n");

  // ---- 1. A full 48-bucket report fills the window exactly ----------------
  // The ordinary production case: one 48 h report, 1 h buckets, nothing missing.
  // slot 47 must hold the LAST completed bucket and slot 0 the 48th one back.
  const uint32_t T0 = 1787068800u;          // an arbitrary bucket-aligned local epoch
  {
    roll48Reset(0u);
    g_lifetimeGal = 0.0;
    HourlyResult r = mkResult(T0, H, 48, 1.0f);      // 1.0 .. 48.0
    roll48Apply(r);
    check(near(g_roll48Gal[0], 1.0f, 0.001f),  "48 at once: slot 0 = oldest completed bucket");
    check(near(g_roll48Gal[47], 48.0f, 0.001f), "48 at once: slot 47 = newest completed bucket");
    check(g_roll48Filled == 48,                "48 at once: slotsFilled = 48");
    check(g_roll48Gaps == 0,                   "48 at once: no gaps");
    check(g_roll48NewestLocal == T0 + 47u * H, "48 at once: newest bucket start recorded");
    check(fabs(g_lifetimeGal - (48.0 * 49.0 / 2.0)) < 0.01,
          "48 at once: lifetime total = sum of the completed buckets (1176 gal)");
  }

  // ---- 2. A second full report replaces the window wholesale --------------
  // 48 more buckets, contiguous with the first. Nothing from the old window may
  // survive: every slot is now 48 hours newer.
  {
    HourlyResult r = mkResult(T0 + 48u * H, H, 48, 100.0f);   // 100.0 .. 147.0
    roll48Apply(r);
    check(near(g_roll48Gal[0], 100.0f, 0.001f), "next 48: slot 0 replaced");
    check(near(g_roll48Gal[47], 147.0f, 0.001f), "next 48: slot 47 replaced");
    check(g_roll48Filled == 48, "next 48: still full");
    check(g_roll48Gaps == 0,    "next 48: contiguous, so still no gaps");
  }

  // ---- 3. A short report shifts by exactly its own length ----------------
  // 5 buckets arriving means the window moves 5 slots, no more. What was in slot
  // 5 must now be in slot 0 - this is the case that catches an off-by-one shift.
  {
    HourlyResult r = mkResult(T0 + 96u * H, H, 5, 200.0f);    // 200.0 .. 204.0
    roll48Apply(r);
    check(near(g_roll48Gal[0], 105.0f, 0.001f),  "5 buckets: window shifted by exactly 5");
    check(near(g_roll48Gal[42], 147.0f, 0.001f), "5 buckets: previous newest moved to slot 42");
    check(near(g_roll48Gal[47], 204.0f, 0.001f), "5 buckets: slot 47 = newest of the 5");
    check(g_roll48Gaps == 0, "5 buckets: contiguous with the previous report, no gaps");
  }

  // ---- 4. THE CASE THIS MODULE EXISTS FOR: a hole in the middle -----------
  // 10 hours produce nothing, then 3 buckets arrive. Shifting by arrival count
  // would move the window 3 and silently place these 3 where the missing hours
  // belong. Shifting by elapsed time moves it 13, zero-fills 10, and says so.
  {
    roll48Reset(0u);
    HourlyResult a = mkResult(T0, H, 48, 1.0f);
    roll48Apply(a);
    const uint32_t gapStart = T0 + 47u * H + 11u * H;   // 10 empty hours after the last bucket
    HourlyResult b = mkResult(gapStart, H, 3, 300.0f);  // 300.0 .. 302.0
    roll48Apply(b);
    check(g_roll48Gaps == 10, "10 h hole: exactly 10 slots counted as gap-filled");
    check(near(g_roll48Gal[47], 302.0f, 0.001f), "10 h hole: slot 47 = newest arrival");
    check(near(g_roll48Gal[45], 300.0f, 0.001f), "10 h hole: the 3 arrivals occupy 45..47");
    check(near(g_roll48Gal[44], 0.0f, 0.001f) && near(g_roll48Gal[35], 0.0f, 0.001f),
          "10 h hole: the missing hours are zero, not compressed away");
    check(near(g_roll48Gal[34], 48.0f, 0.001f),
          "10 h hole: pre-gap data sits 13 slots back, on the real time axis");
    check(g_roll48NewestLocal == gapStart + 2u * H, "10 h hole: newest bucket start is the real one");
  }

  // ---- 5. A 96-bucket replay leaves a full, correct window ----------------
  // A cloud outage recovery delivers two reports' worth at once. The window can
  // only show the last 48 - which is exactly why the full series is ALSO
  // published, as hourlyBuckets. What must not happen is a half-filled window.
  {
    roll48Reset(0u);
    HourlyResult r = mkResult(T0, H, 96, 1.0f);        // 1.0 .. 96.0
    roll48Apply(r);
    check(g_roll48Filled == 48, "96 replay: window reports itself full");
    check(near(g_roll48Gal[47], 96.0f, 0.001f), "96 replay: slot 47 = the newest of the 96");
    check(near(g_roll48Gal[0], 49.0f, 0.001f),  "96 replay: slot 0 = 48 buckets back, not bucket 1");
    check(g_roll48Gaps == 0, "96 replay: contiguous data, so no gaps");
  }

  // ---- 6. Out-of-order and duplicate buckets add, never shift -------------
  // A retransmitted batch or a replayed flash block re-delivers a bucket we have
  // already placed. Shifting on it would move the window BACKWARDS in time.
  {
    roll48Reset(0u);
    HourlyResult a = mkResult(T0, H, 48, 1.0f);
    roll48Apply(a);
    const uint32_t newest = g_roll48NewestLocal;
    HourlyResult dup = mkResult(T0 + 47u * H, H, 1, 5.0f);   // the newest bucket, again
    roll48Apply(dup);
    check(g_roll48NewestLocal == newest, "duplicate bucket: the window did not move");
    check(near(g_roll48Gal[47], 53.0f, 0.001f), "duplicate bucket: value added into its own slot");
    HourlyResult old = mkResult(T0 + 40u * H, H, 1, 7.0f);   // an older slot, still in the window
    roll48Apply(old);
    check(near(g_roll48Gal[40], 48.0f, 0.001f),
          "older bucket: added into its own slot (41.0 + 7.0), window unmoved");
  }

  // ---- 7. An outage longer than the window clears it ---------------------
  // More than 48 buckets of silence means nothing held is still inside the
  // window. Everything must go - keeping stale values would publish data from
  // days ago as if it were the last 48 hours.
  {
    roll48Reset(0u);
    HourlyResult a = mkResult(T0, H, 48, 1.0f);
    roll48Apply(a);
    HourlyResult b = mkResult(T0 + 47u * H + 200u * H, H, 1, 500.0f);
    roll48Apply(b);
    check(near(g_roll48Gal[47], 500.0f, 0.001f), "long outage: slot 47 = the new bucket");
    check(near(g_roll48Gal[0], 0.0f, 0.001f) && near(g_roll48Gal[46], 0.0f, 0.001f),
          "long outage: every other slot cleared, no stale values published");
    check(g_roll48Gaps == 199, "long outage: the 199 skipped slots are counted");
  }

  // ---- 8. A bucket-width change resets the window ------------------------
  // 48 slots of 60 s and 48 slots of 3600 s are not the same window. Reusing the
  // contents would relabel bench minutes as production hours.
  {
    roll48Reset(0u);
    HourlyResult a = mkResult(T0, H, 48, 1.0f);
    roll48Apply(a);
    HourlyResult b = mkResult(T0 + 100u * H, 60u, 3, 9.0f);   // bench width
    roll48Apply(b);
    check(g_roll48BucketSec == 60u, "width change: window adopts the new width");
    check(g_roll48Filled == 3, "width change: slotsFilled restarts (old slots meant something else)");
    check(g_roll48Gaps == 0,   "width change: gap count restarts too");
    check(near(g_roll48Gal[47], 11.0f, 0.001f), "width change: newest of the new series in slot 47");
    check(near(g_roll48Gal[44], 0.0f, 0.001f),  "width change: no 3600 s values survived");
  }

  // ---- 9. slotsFilled distinguishes "no data" from "no water" ------------
  // A brand-new device and a device that used no water both publish an array of
  // zeroes. slotsFilled is the only thing that tells them apart, so it must not
  // be inflated by data the window never actually received.
  {
    roll48Reset(0u);
    HourlyResult r = mkResult(T0, H, 3, 0.0f);      // three real buckets, all 0.0 gal
    roll48Apply(r);
    check(g_roll48Filled == 3,
          "fresh install: slotsFilled counts real buckets only, even when they are 0 gal");
    check(g_roll48Gaps == 0, "fresh install: three genuine zeroes are not gaps");
  }

  // ---- 10. An empty report changes nothing -------------------------------
  // A zero-sample RSP_DATA is a healthy exchange (V064 P-2), not an error, and
  // it must not disturb the window.
  {
    roll48Reset(0u);
    HourlyResult a = mkResult(T0, H, 48, 1.0f);
    roll48Apply(a);
    const uint32_t newest = g_roll48NewestLocal;
    const uint16_t filled = g_roll48Filled;
    HourlyResult empty = mkResult(T0 + 48u * H, H, 0, 0.0f);
    roll48Apply(empty);
    check(g_roll48NewestLocal == newest && g_roll48Filled == filled,
          "zero-bucket report: window untouched");
  }

  printf("%s (%d failure%s)\n", g_fail ? "FAILED" : "all checks passed",
         g_fail, g_fail == 1 ? "" : "s");
  return g_fail ? 1 : 0;
}
