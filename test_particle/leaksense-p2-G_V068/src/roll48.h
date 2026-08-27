/*
 * roll48.h  -  The contract's fixed 48-slot sliding window (V068).
 *
 * WHY THIS IS A SEPARATE MODULE
 * -----------------------------
 * The Project Agreement (2026-07-18, sec 4) names a field "hourlyGallons" and
 * defines it as a fixed 48-slot array where slot 47 is the last completed hour
 * and slot 0 is the 48th completed hour back. This firmware already had two
 * other views of the same water:
 *
 *   hourlyData[24]   leftover date-aligned "today" totals. Not published.
 *                    Not zeroed at midnight (req 5: never wipe hourly history).
 *   g_hourly[]       the variable-length series of completed buckets. It answers
 *                    "exactly what was measured", and its length is NOT 48 -
 *                    47 and 49 are routine, and a flash-ring recovery can carry
 *                    96 or more.
 *
 * Neither can be reshaped into the contract array. Forcing the variable series
 * into 48 slots would discard the older half of every recovery. So this is a
 * third view, fed from the same finished buckets, which is what stops the
 * views from disagreeing about a total.
 *
 * Req 5 (local-time 48 h window):
 *   - slots [0..47] are COMPLETED hours only; [47] is the newest completed hour
 *   - the still-open (incomplete) hour is HourlyrReset, never written here
 *   - a new completed hour SHIFTS the window (old values roll toward slot 0);
 *     values older than 48 hours fall off the front (truncated)
 *   - do not zero the window on a short report or at midnight
 *
 * It lives in its own translation unit for one practical reason: it can then be
 * linked into tools/hostcheck/roll48_test.cpp and its arithmetic actually
 * exercised - the gap accounting below is the part most likely to be wrong, and
 * a guard nobody has watched fail is not evidence.
 *
 * All epochs here are LOCAL epochs, the same axis hourly.cpp uses. There is no
 * timezone conversion in this module; that happens once, in the publish path.
 */

#pragma once
#include "Particle.h"
#include "app_config.h"
#include "hourly.h"

// ---- Window state ----------------------------------------------------------
// Defined in roll48.cpp as `retained`, because the Photon is unpowered about
// 87 % of the time and the window has to continue across that.
extern float    g_roll48Gal[ROLL48_COUNT];
extern uint32_t g_roll48NewestLocal;   // local epoch at the START of the bucket in slot 47
extern uint16_t g_roll48Filled;        // slots ever written, 0..48 ("how much do we know")
extern uint32_t g_roll48Gaps;          // slots zero-filled because no bucket existed for them
extern uint32_t g_roll48BucketSec;     // the width the window was built at; a change resets it

// Lifetime total, summed from COMPLETED buckets only so it can never drift from
// the published series. See roll48.cpp for why this is the power-cycle-surviving
// reading and not the since-power-up one.
extern double   g_lifetimeGal;

// Fold one finished report into the window (and into the lifetime total).
// Only completed buckets are passed here; the open bucket is the carry, which is
// published separately as the contract's HourlyrReset field.
void roll48Apply(const HourlyResult &h);

// Discard the window. Used when the bucket width changes, and by the tests.
void roll48Reset(uint32_t bucketSec);
