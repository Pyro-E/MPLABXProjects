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
#define PCFG_VERSION            2u

/* ---- A. TIMING (auto-derived from the PIC -> always in sync) ---------------
 * captureIntervalMs = the PIC's REAL count-multiple capture window in ms.
 * The Photon divides pulses by this (/1000 s) for the rate, so quantization is
 * removed automatically whatever wake/counts the PIC uses. */
#define PCFG_CAPTURE_INTERVAL_MS   ((uint32_t)APP_CAPTURE_PERIOD_MS)
#define PCFG_SAMPLES_PER_REPORT    ((uint16_t)APP_SAMPLES_PER_REPORT)
/* REPORT_INTERVAL_HR (App_Config.h: 24 or 48) -- so the Photon's hourlyGallons[24]
 * array can bin at the right width (1 h/slot for 24 h, 2 h/slot for 48 h) instead
 * of assuming 24 h. See leaksense.cpp: hourlyBinWidthHours(). */
#define PCFG_REPORT_INTERVAL_HR    ((uint8_t)REPORT_INTERVAL_HR)

/* ---- B. DEBUG TOGGLES (change freely, rebuild PIC only) --------------------
 * These drive the Photon's debug behaviour at runtime. */
#define PCFG_FAST_BENCH            0u      /* CURRENT: fast bench = cloud SKIPPED (virtual clock, sim cloud).
                                             *          Set 0 for real-cloud production later. */
#define PCFG_DEBUG_DATASERIES      1u      /* 1=Photon prints every sample over USB-CDC */
#define PCFG_MISSED_FILL_MODE      1u      /* 0=ZERO, 1=AVERAGE (missed-pulse reconstruction) */
#define PCFG_SERIAL_DELAY_MS       3000u   /* Photon boot delay before the log burst (ms) */

#endif /* APP_CONFIG_PHOTON_H */
