/* ============================================================================
 * App_Config_Photon.h  -  settings the PIC hands to the Photon at boot.
 * ----------------------------------------------------------------------------
 * WHY: rebuilding/reflashing the Photon is slow. The PIC builds in ~30 s, so we
 * keep the Photon's TIMING and DEBUG settings HERE. Edit this file, rebuild the
 * PIC, and the Photon picks up the new settings on its next power-up - no Photon
 * rebuild needed.
 *
 * The Photon requests these at boot (PKT_REQ_PHOTON_CFG 0x09, with retries). The
 * PIC answers with PKT_RSP_PHOTON_CFG 0x89:
 *   - PIC_PROVIDES_PHOTON_CFG defined  -> provided=1, the values below.
 *   - PIC_PROVIDES_PHOTON_CFG undefined-> provided=0, "use your own defaults".
 * Either way the PIC ANSWERS, so the Photon never hangs waiting.
 *
 * TIMING is auto-derived from the PIC's own capture config (App_Config.h), so the
 * PIC and Photon can never disagree about the sample interval again (that was the
 * source of the GPM/gallon errors).
 * ==========================================================================*/
#ifndef APP_CONFIG_PHOTON_H
#define APP_CONFIG_PHOTON_H

#include "App_Config.h"      /* APP_CAPTURE_PERIOD_MS, APP_SAMPLES_PER_REPORT */
#include <stdint.h>

/* Master switch: define -> PIC supplies the config; comment out -> PIC replies
 * "not provided" and the Photon uses its own compiled defaults. */
#define PIC_PROVIDES_PHOTON_CFG

/* Wire-format version (bump if the block layout changes). */
#define PCFG_VERSION            1u

/* ---- A. TIMING (auto-derived from the PIC -> always in sync) ---------------
 * captureIntervalMs = the PIC's REAL count-multiple capture window in ms.
 * The Photon divides pulses by this (/1000 s) for the rate, so quantization is
 * removed automatically whatever wake/counts the PIC uses. */
#define PCFG_CAPTURE_INTERVAL_MS   ((uint32_t)APP_CAPTURE_PERIOD_MS)
#define PCFG_SAMPLES_PER_REPORT    ((uint16_t)APP_NOMINAL_CAPTURES_PER_REPORT)

/* ---- B. DEBUG TOGGLES (change freely, rebuild PIC only) --------------------
 * These drive the Photon's debug behaviour at runtime. */
/* V023 / fix 1: DERIVED, never hand-set.
 * This field selects the Photon's cadence/bucket profile ONLY. It has nothing
 * to do with cloud enable/disable - on the Photon (V063) cadenceFast and cloud
 * mode are independent settings, and the old comment here claimed otherwise.
 *
 * It used to be a literal 0 while the PIC itself was built with
 * REPORT_CONFIG_DEBUG, which inverted the bench: a Photon that RECEIVED the CFG
 * ran production cadence (bucket 3600 s), while a Photon whose CFG request
 * FAILED fell back to its own defaults and accidentally ran the correct fast
 * bench profile (bucket 60 s).
 *
 * Deriving it from the same switch that governs the PIC's own report timing is
 * the fix; setting the literal to 1 would only postpone the same inversion to
 * the next production/bench switch. This mirrors what section A above already
 * does for the timing fields. */
#ifdef REPORT_CONFIG_DEBUG
  #define PCFG_FAST_BENCH          1u      /* 1 = fast bench cadence/bucket profile   */
#else
  #define PCFG_FAST_BENCH          0u      /* 0 = production cadence/bucket profile   */
#endif
#define PCFG_DEBUG_DATASERIES      1u      /* 1=Photon prints every sample over USB-CDC */
#define PCFG_MISSED_FILL_MODE      1u      /* 0=ZERO, 1=AVERAGE (missed-pulse reconstruction) */
#define PCFG_SERIAL_DELAY_MS       3000u   /* Photon boot delay before the log burst (ms) */

/* ---- C. PROTOCOL (must match the Photon build) ----------------------------
 * V024 / protocol v2. This is the ONE setting that is NOT free to change on its
 * own: it alters the RSP_DATA header and the DATA_RECEIVED payload, so the PIC
 * and the Photon must be flashed as a pair.
 *
 * WHY v2 EXISTS
 * -------------
 * The PIC's whole contract is the AMOUNT of water. v1 states that amount only
 * as a DELTA - "pulses since the last report" - and a delta cannot be audited.
 * If one is lost, duplicated, or computed against a mark that moved, nothing
 * downstream can tell: the number still looks like a plausible amount of water.
 * Every accounting defect found in the first bench campaign was an instance of
 * that one weakness.
 *
 * v2 states the amount as a POSITION on the PIC's lifetime pulse axis: each
 * report carries the cumulative total at its start AND at its end. The delta
 * becomes derived (end - begin), so it can no longer disagree with anything,
 * and the Photon can check on EVERY frame that this report begins exactly where
 * the last one ended. Loss, duplication and gaps stop being invisible.
 *
 *   0  23-byte header, byte-for-byte identical to V022. Works against Photon
 *      V063, and V064 built with PHOTON_BATCH_SEQ_ENABLE 0. All the V023 freeze
 *      and invariance fixes are active; only the audit is withheld.
 *
 *   1  31-byte header with absolute span endpoints and a boot id, and an 8-byte
 *      DATA_RECEIVED echo. REQUIRES the matching Photon build. Against any other
 *      build every frame fails the length check and the link stops dead - which
 *      is deliberate: on a metering link a silent version mismatch is worse than
 *      an obvious halt.
 *
 * NOTE: v2 REPLACES the V023 batch_seq experiment. Do not build a Photon with
 * PHOTON_BATCH_SEQ_ENABLE 1 against a v2 PIC; the span endpoints identify a
 * batch exactly, so a separate synthetic counter is no longer needed and its
 * 24-byte header is not emitted by this firmware.
 *
 * Bench order: verify with 0 first, then flash both sides and move to 1. */
#ifndef PCFG_PROTO_V2
#define PCFG_PROTO_V2              0
#endif

#endif /* APP_CONFIG_PHOTON_H */
