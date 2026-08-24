#include "roll48.h"

// ---- Window state (see roll48.h for why each field exists) ------------------
retained float    g_roll48Gal[ROLL48_COUNT] = {0.0f};
retained uint32_t g_roll48NewestLocal       = 0;
retained uint16_t g_roll48Filled            = 0;
retained uint32_t g_roll48Gaps              = 0;
retained uint32_t g_roll48BucketSec         = 0;

// The agreement's wording contradicts itself: "lifetimeGal: total gallons since
// system power down" is a lifetime name with a since-power-up definition. It is
// implemented as the lifetime reading - surviving power cycles - because that
// choice is recoverable and the other is not. A since-power-up figure can always
// be derived from a lifetime one; if we shipped the volatile reading and the
// client turns out to have meant the lifetime one, the history needed to build
// it does not exist anywhere and never will.
//
// It counts from the moment this firmware was installed. Usage before that was
// never recorded on either board, so it cannot be included, and that limit must
// be stated to the client rather than left for them to discover.
retained double   g_lifetimeGal             = 0.0;

void roll48Reset(uint32_t bucketSec) {
  for (uint8_t j = 0; j < ROLL48_COUNT; j++) g_roll48Gal[j] = 0.0f;
  g_roll48Filled      = 0;
  g_roll48Gaps        = 0;
  g_roll48NewestLocal = 0;
  g_roll48BucketSec   = bucketSec;
}

void roll48Apply(const HourlyResult &h) {
  if (h.bucketSec == 0u || h.count == 0u) return;

  // A different bucket width changes what a SLOT MEANS, so the existing contents
  // cannot be reinterpreted - 48 slots of 60 s and 48 slots of 3600 s are not
  // the same window. Start again rather than mix the two. In practice this fires
  // on the bench <-> production switch.
  if (g_roll48BucketSec != h.bucketSec) {
    Log.info("[R48] bucket width %lu -> %lu s: the 48-slot window is reset "
             "(a slot no longer covers the same span)",
             (unsigned long)g_roll48BucketSec, (unsigned long)h.bucketSec);
    roll48Reset(h.bucketSec);
  }

  for (uint16_t i = 0; i < h.count; i++) {
    const uint32_t binStart = h.baseLocal + (uint32_t)i * h.bucketSec;

    // Lifetime total, from completed buckets only, for the same reason the
    // window is fed from them: the two can then never disagree.
    g_lifetimeGal += (double)h.gal[i];

    if (g_roll48NewestLocal == 0u) {              // the first bucket this window has ever seen
      g_roll48Gal[ROLL48_COUNT - 1] = h.gal[i];
      g_roll48NewestLocal = binStart;
      g_roll48Filled      = 1;
      continue;
    }

    if (binStart <= g_roll48NewestLocal) {
      // Same or older bucket: ADD, never shift. A replayed flash block or a
      // retransmitted batch arrives out of order, and shifting on it would move
      // the window backwards in time. Buckets older than the window have nowhere
      // to go and are dropped here - g_hourly/hourlyBuckets is what preserves
      // them, which is the whole reason both events are published.
      const uint32_t back = (g_roll48NewestLocal - binStart) / h.bucketSec;
      if (back < ROLL48_COUNT) g_roll48Gal[ROLL48_COUNT - 1 - back] += h.gal[i];
      continue;
    }

    // ---- Shift by ELAPSED TIME, not by arrival count -----------------------
    // Shifting one slot per arriving bucket compresses the time axis whenever
    // data is missing: after a 10 h outage, three buckets would move the window
    // by 3 and place the newest reading 10 hours away from where it belongs,
    // with nothing in the payload to reveal it. Deriving the shift from the
    // absolute bucket start moves the window 13, zero-fills the 10 that had no
    // data, and counts them - so the hole is stated instead of hidden.
    const uint32_t steps = (binStart - g_roll48NewestLocal) / h.bucketSec;
    if (steps >= ROLL48_COUNT) {                  // everything held is now older than the window
      for (uint8_t j = 0; j < ROLL48_COUNT; j++) g_roll48Gal[j] = 0.0f;
      g_roll48Gaps += (steps - 1u);
    } else {
      for (uint32_t s = 0; s < steps; s++) {      // one slot per bucket of elapsed time
        for (uint8_t j = 0; j < ROLL48_COUNT - 1; j++) g_roll48Gal[j] = g_roll48Gal[j + 1];
        g_roll48Gal[ROLL48_COUNT - 1] = 0.0f;
      }
      if (steps > 1u) g_roll48Gaps += (steps - 1u);   // the skipped slots are known-unknown
    }
    g_roll48Gal[ROLL48_COUNT - 1] = h.gal[i];
    g_roll48NewestLocal = binStart;
    if (g_roll48Filled < ROLL48_COUNT) {
      const uint32_t f = (uint32_t)g_roll48Filled + steps;
      g_roll48Filled = (f > (uint32_t)ROLL48_COUNT) ? (uint16_t)ROLL48_COUNT : (uint16_t)f;
    }
  }
}
