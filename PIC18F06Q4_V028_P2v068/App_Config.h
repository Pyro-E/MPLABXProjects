#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ============================================================
 *  App_Config.h  -  central place for all tunable settings
 *
 *  Edit values here; the driver modules include this file and
 *  use these macros. Nothing else needs to be touched to change
 *  capture timing, buffer sizes, baud rate, etc.
 * ============================================================ */

/* ############################################################
 * #                                                          #
 * #   QUICK TEST SETTINGS  -  the knobs you change most       #
 * #   often while bench testing live in THIS block.           #
 * #   Flip these, rebuild, and observe. Detailed/!rarely-     #
 * #   touched settings are further down the file.             #
 * #                                                          #
 * ############################################################ */

/* ---- 1. DEBUG vs PRODUCTION timing -----------------------------------
 * Defined   -> fast values for bench testing (short capture period, small
 *              batch, short leak windows, short temp-lock). See the matching
 *              #ifdef REPORT_CONFIG_DEBUG blocks below for the actual numbers.
 * Undefined -> production values (60 s capture, 12 h batch, 8 min / 3 min
 *              leak windows, 10 min temp-lock).
 * This single switch flips EVERY related value at once. */
#define REPORT_CONFIG_DEBUG   /* DEFAULT = DEBUG (bench test). Comment out for production. */

/* Bench self-timing: when defined, the PIC ACKs the Photon's SET_PARAM but keeps
 * its OWN #define grid (LOCO_ALIGN_ANCHOR_SEC / LOCO_REPORT_INTERVAL_SEC) and
 * drives measure/capture/wake/report from it. The Photon only sorts and reports
 * whatever the PIC sends, so its own (possibly different) grid does not matter.
 * Auto-on in bench test; for production leave it OFF so the cloud can change the
 * grid via the Photon. Comment out to always adopt the relayed grid. */
#ifdef REPORT_CONFIG_DEBUG
  #define PIC_USE_OWN_TIMING
#endif

/* Leak parameters (alert1/alert2 window + threshold): by DEFAULT the PIC adopts
 * whatever the Photon relays (GET/SET_PARAM), so the cloud can tune them. The
 * structure lives on the flow side; the Photon can read and write it but by
 * default does not bother, so the PIC runs on its #define defaults
 * (APP_LEAK*_*_DEF). Define PIC_USE_OWN_LEAK_PARAMS to make the PIC ACK the
 * write but keep its own #define values (Photon won't see an error). Left
 * undefined = adopt Photon writes. */
/* #define PIC_USE_OWN_LEAK_PARAMS */

/* ---- LOCO (LFINTOSC) self-calibration switches -----------------------------
 * The deep-sleep wake count is scaled by the LOCO correction factor k (Q7).
 * These two switches independently allow each evaluation source to MOVE k away
 * from the ideal 128 (=1.0):
 *   LOCO_CAL_HFINTOSC_ENABLE : coarse initial eval (Timer0 LFINTOSC vs the
 *                              HFINTOSC ms base), done during the initial hold.
 *   LOCO_CAL_CLOUD_ENABLE    : precise eval from cloud absolute time each report.
 * If BOTH are undefined, k stays 128 forever and the sleep timing is exactly
 * the spec-ideal value (no calibration). Enable either/both to correct drift. */
#define LOCO_CAL_HFINTOSC_ENABLE   /* comment out to forbid HFINTOSC eval  */
/* Number of 500 ms LFINTOSC measurements averaged for the 1st-stage (HFINTOSC)
 * calibration during the cold-boot hold. Bench-validated readings are very
 * stable, so a small count (3) removes single-sample noise cheaply. */
#ifndef LOCO_HF_CAL_SAMPLES
#define LOCO_HF_CAL_SAMPLES  3u
#endif
#define LOCO_CAL_CLOUD_ENABLE      /* comment out to forbid cloud-time eval */

/* ---- 2. SLEEP (power saving) -----------------------------------------
 * Defined   -> between captures the MCU enters deep Sleep (low power).
 * Undefined -> MCU stays awake all the time (super-loop). USE THIS WHILE
 *              BENCH TESTING so the PIC always responds to packets instantly
 *              and you can watch it on the debugger / logic analyzer. */
#define APP_SLEEP_ENABLE

/* ---- 3. VALVE driving ------------------------------------------------
 * VALVE_PWR_CTRL_ENABLE : master enable. Undefine to leave the valve pins
 *                         permanently LOW (safe default - valve never moves,
 *                         leak logic still runs and reports lock state).
 * VALVE_ON_WHEN_STARTUP : on a non-WDT reset, drive the valve OPEN once at
 *                         startup (PWR=H,CTRL=H for TIME_VALVE_FULL_TOGGLE_MS).
 * VALVE_TEST_TOGGLE     : with ENABLE, ignore the leak logic and just toggle
 *                         the pins (bench check that the valve physically
 *                         opens/closes and that CTRL=H really means OPEN).
 *
 * FIRST-TIME VALVE BRING-UP: enable VALVE_PWR_CTRL_ENABLE + VALVE_TEST_TOGGLE
 * and confirm the direction before trusting the leak logic to drive it. */
#define VALVE_PWR_CTRL_ENABLE
#define VALVE_ON_WHEN_STARTUP
//#define VALVE_TEST_TOGGLE

/* ---- 4. LEAK THRESHOLDS (defaults; Photon can change at runtime) ------
 * Two INDEPENDENT alerts, each a sliding-window sum over its own window:
 *   alert1 (counts within window) -> TEMPORARY lock, auto-opens later.
 *   alert2 (counts within window) -> PERMANENT lock, host/reset clears it.
 * Windows are in SECONDS; the firmware quantizes them to capture periods.
 * The actual numbers live in the REPORT_CONFIG_DEBUG block below (so debug
 * and production each get sensible values). Change them there. */
/*   (debug defaults:  alert1 = 50 / 20 s,  alert2 = 100 / 10 s, temp 30 s)  */
/*   (prod  defaults:  alert1 = 100 / 8 min, alert2 = 400 / 3 min, temp 10 m) */

/* ---- 5. DEBUG: auto-report without a Photon -------------------------
 * Defined -> when a report is due the PIC sends RSP_DATA on its own, as if a
 *            REQ_DATA arrived. Lets you watch the UART report with no Photon
 *            attached. Leave UNDEFINED for real packet testing. */
//#define APP_DEBUG_AUTO_DATA_REPORT_WITHOUT_REQ

/* ---- 5b. DEBUG UART2 on RA4 (TX-only, SEPARATE from the packet UART) --
 * Defined -> UART2 is brought up as a transmit-only debug port on RA4, and
 *            the bench log lines (see Dev_Debug_Uart.h) go out there.
 *
 *   RC0/RC1 = UART1 = Photon packets ONLY   (never mixed with text)
 *   RA4     = UART2 = debug text ONLY       (no RX pin is assigned at all)
 *   RA5     = left free on purpose, reserved for a future function
 *
 * Because the two streams use different pins, debug logging is safe EVEN
 * WHILE a Photon is connected - unlike options 6 and 7 below, which share
 * the packet UART and corrupt it. Prefer THIS one for bench work.
 * Wiring: RA4 -> RX of a USB-serial adapter, GND common. Terminal 38400 8N1.
 * Leave undefined for production: no code, no pin use. */
/* ---- VIRTUAL FLOW METER (bench only) --------------------------------
 * With no meter wired to RC5 the hardware counter never moves, so captures are
 * all zero and nothing downstream can be tested. Defining this replaces the
 * pulse source with a software schedule (see FlowMeter.c). The real counter is
 * left configured but ignored; everything above the meter runs unchanged.
 * MUST be undefined for production and whenever a real meter is attached. */
#define APP_VFLOW_ENABLE

/* Rate schedule: { run until this ms, pulses per second, name }.
 * The last entry runs forever. Times are absolute since boot, so a run is
 * reproducible and the log can be checked against this table directly.
 *
 * The phases deliberately walk across the bench leak thresholds
 * (alert1 = 2.5/s over 20 s, alert2 = 10/s over 10 s):
 *   QUIET   0 /s  - during the initial hold; proves no false alarm and that
 *                   cold-boot captures are discarded
 *   NORMAL  1 /s  - real flow, below both thresholds; proves samples and
 *                   gallons flow through with no alert
 *   LEAK1   4 /s  - above alert1 (2.5/s), below alert2 (10/s)
 *                   -> weak alert -> temporary lock -> auto-clear
 *   CALM    1 /s  - back to normal; proves the temporary lock releases
 *   LEAK2  20 /s  - above alert2 (10/s) -> strong alert -> permanent lock
 * Put LEAK2 last: the permanent lock does not auto-clear, so anything after it
 * would be testing a locked valve. */
#define APP_VFLOW_PHASES  { \
    {  90000UL,     0u, "QUIET"  }, \
    { 240000UL,     1u, "NORMAL" }, \
    { 330000UL,     4u, "LEAK1"  }, \
    { 450000UL,     1u, "CALM"   }, \
    { 560000UL, 20000u, "OVFLOW" }, \
    { 0xFFFFFFFFUL, 20u, "LEAK2" } \
}

/* OVFLOW phase: 20000 pulses/s puts about 105,800 pulses into one 5.29 s
 * capture, far past the 16-bit cell limit (65535). It exercises the paths that
 * only a saturating meter can reach:
 *   - the sample is clamped to 0xFFFF and the report's overflow counter (ovf)
 *     increments, so the header shows how many cells were saturated
 *   - total_impulses keeps the TRUE count, so it ends up much larger than the
 *     sum of the samples - which is exactly the case the Photon's missed-fill
 *     is meant to restore, and the reason billing is total-based
 * It also proves the 32-bit accumulators do not wrap, and that both alerts fire
 * long before the cell saturates. Two captures' worth is enough; a longer run
 * would only inflate the totals. */

#define APP_DEBUG_UART2_ENABLE
#define APP_DEBUG_UART2_BAUD  38400UL     /* debug terminal baud (RA4)     */

/* PPS output code that routes U2TX onto RA4.
 * MEASURED ON THIS BOARD, not guessed: a sweep of every output code 0x00-0x3F
 * produced readable UART2 output at exactly one value, 0x13. (For reference
 * U1TX is 0x10, which Dev_Uart.c uses.) Do not change without re-measuring. */
#define APP_DEBUG_UART2_PPS   0x13u

/* Start-up self test: blast 'U' characters on RA4 before anything else, to
 * separate "PPS/wiring wrong" (silence) from "baud wrong" (garbage).
 * Turn off once real log lines appear. */
//#define APP_DEBUG_UART2_SELFTEST
/* Bench event log ([BOOT]/[PWR]/[LOCO]/[RPT]/[CAP]/[LEAK]) on the debug UART.
 * Needs APP_DEBUG_UART2_ENABLE as well - that one opens the port, this one
 * chooses whether the event lines are emitted. Turn BOTH off for production. */
#define APP_DEBUG_EVENT_LOG

#define APP_DEBUG_UART2_FIFO  256u        /* TX FIFO bytes (RAM cost). A log
                                           * line is ~20-50 bytes; if
                                           * DbgUart_Dropped() reports losses,
                                           * raise this or log less.
                                           *
                                           * V021: this knob is now actually
                                           * wired to DBG_FIFO_SIZE - before it
                                           * was dead and the FIFO stayed 128
                                           * whatever was written here. Raised
                                           * 128 -> 256 (+128 B RAM) because a
                                           * whole line was lost in the
                                           * 20260813 bench capture. If the
                                           * build reports a RAM overflow, put
                                           * it back to 128u; nothing else
                                           * depends on the value.
                                           *
                                           * NOTE: the main loop refuses to
                                           * deep-sleep until DBG_IS_IDLE(), so
                                           * a bigger FIFO also means a longer
                                           * awake window (256 B at 38400 baud
                                           * = ~67 ms). Bench only - both
                                           * APP_DEBUG_UART2_ENABLE and
                                           * APP_DEBUG_EVENT_LOG must be OFF in
                                           * production anyway.             */

/* ---- 6. DEBUG print lines (human-readable) ---------------------------
 * Defined -> extra "Sample-..." text lines are emitted on the UART. NOTE:
 *            this text is interleaved into the byte stream and WILL corrupt
 *            the binary packet protocol, so keep it UNDEFINED whenever a
 *            Photon is connected. Bench/terminal observation only.
 *            (Superseded by APP_DEBUG_UART2_ENABLE above, which is safe.) */
//#define APP_DEBUG_PRINT_ENABLE

/* ---- 7. DEBUG: packet-flow log (human-readable) ---------------------
 * Defined -> the PIC prints a short line for every packet it RECEIVES
 *            (RX: REQ_DATA, RX: GET_PARAM, ...) and a one-line summary of each
 *            RSP_DATA it sends (the extended-header fields: impulse/captures/
 *            span/overflow/count). Lets a bench PC (PIC UART -> USB-serial,
 *            NO Photon) watch the exchange. Like all debug text this INTERLEAVES
 *            with the binary protocol, so keep it UNDEFINED with a Photon
 *            attached. */
#define APP_DEBUG_PKT_LOG

/* ---- 8. DEBUG: data-series dump (human-readable, VOLUMINOUS) --------
 * Defined -> in addition to the RSP_DATA summary, the PIC prints every sample
 *            of the series it sends (index, group, pulses). This is the raw
 *            flow log for eyeballing on the bench. It is separate from
 *            APP_DEBUG_PKT_LOG because it is large; enable only when you need
 *            to inspect the actual samples. Bench-only (no Photon), interleaves
 *            with the binary stream. */
//#define APP_DEBUG_DATASERIES

/* ===== end QUICK TEST SETTINGS - details follow below ================ */


/* ---- System clock (must match the RSTOSC config bits) ---- */
#define APP_FOSC_HZ            64000000UL   /* 64 MHz internal */

/* ---- Flow data capture / logging ----
 * REPORT_CONFIG_DEBUG is set ONCE in the QUICK TEST SETTINGS block at the top
 * of this file; do not redefine it here. The #ifdef below simply consumes it. */

/* ---- Flow data capture / logging  (3-tier timing) ----
 * The timing is split into three independent tiers so the capture period can
 * be much longer than the 8-bit deep-sleep timer alone would allow:
 *
 *   1) WAKE   : APP_WAKE_COUNTS  - how often the PIC wakes (in Timer0 counts).
 *               This is the ONLY value the Timer0 hardware sees, so it must be
 *               within the 8-bit timer's reach (<= ~134 s at 1:16384). Each
 *               wake just advances the clock; most wakes do nothing else.
 *   2) SAMPLE : APP_WAKES_PER_SAMPLE - a capture (one ring-buffer sample) is
 *               taken every Nth wake. Capture period = WAKE x N. Because
 *               FlowLog_Process() is time-gated, this "count wakes, capture on
 *               the Nth" happens automatically from the clock; no extra counter.
 *   3) REPORT : derived at run time = captures until the next {anchor,interval}
 *               grid point (Loco_GridCaptures). No fixed samples-per-report.
 *               Report period = capture period x M = WAKE x N x M.
 *
 * Example (production): 60 s x 4 x 720 = capture 240 s (4 min), report 2 days.
 *
 * REPORT_CONFIG_DEBUG is set ONCE in the QUICK TEST SETTINGS block at the top
 * of this file; the #ifdef below simply consumes it. Default = test mode. */

/* WAKE is defined as an INTEGER NUMBER OF TIMER0 COUNTS, not milliseconds.
 * With the fixed 1:16384 prescaler on LFINTOSC(~31 kHz), ONE COUNT = 16384/31 =
 * 528.5 ms (0.5285 s). Expressing the wake as a whole number of counts makes the
 * period an EXACT multiple of the tick, so there is NO time-quantization error
 * for ANY value chosen (this is what removes the GPM-rate error). The nominal
 * seconds are shown in each comment as (counts x 0.5285 s). Max 255 counts
 * (8-bit Timer0) => max wake ~134.8 s. */
#ifdef REPORT_CONFIG_DEBUG
  #define APP_FLOW_SLOTS         1024     /* ring-buffer slots (write==read = EMPTY) */
  /* TIME-COMPRESSED BENCH TEST (same algorithm, parameters only): wake element
   * 0.5285 s KEPT (no prescaler change); 19 wakes -> ~10 s capture. The report
   * period is NOT a constant here -- the PIC derives "captures until the next
   * grid point" at run time from {anchor, interval} (LOCO_REPORT_INTERVAL_SEC
   * below). Pair with Photon {midnight, 2 min, <=1 min binning}. */
  #define APP_WAKE_COUNTS        1u       /* TEST: 1 x 0.5285 s = 0.53 s wake         */
  #define APP_WAKES_PER_SAMPLE   10u      /* capture = 10 wakes (10 x 0.5285) = 5.29 s */
#else
  #define APP_FLOW_SLOTS         1024     /* ring-buffer slots (write==read = EMPTY) */
  #define APP_WAKE_COUNTS        114u     /* PRODUCTION: 114 x 0.5285 s = 60.25 s wake*/
  #define APP_WAKES_PER_SAMPLE   4u       /* capture = 4 wakes (456 x 0.5285) = 241.0 s (~4 min) */
#endif

/* Derived: capture period (ring-buffer sample interval) and report batch.
 * FlowLog gates captures on APP_CAPTURE_PERIOD_MS; the leak window converts
 * seconds->captures with the same value. Built from APP_WAKE_ACTUAL_MS (the real
 * count-multiple wake) so the capture window has NO quantization error. */
#define APP_CAPTURE_PERIOD_MS  (APP_WAKE_ACTUAL_MS * (uint32_t)APP_WAKES_PER_SAMPLE)
/* APP_FLOW_BATCH is defined further down, AFTER LOCO_REPORT_INTERVAL_SEC, because
 * the nominal batch is now DERIVED from the grid interval (not a hardcoded
 * constant). See APP_NOMINAL_CAPTURES_PER_REPORT. */

/* Safety margin (d) for the ring buffer. The un-sent backlog is capped at
 * (SLOTS - d): once it would exceed that, the oldest samples are dropped
 * (read pointer jumps forward). Because used never reaches SLOTS, the write
 * pointer never laps the read pointer, so write==read ALWAYS means EMPTY
 * (never "full") - the classic circular-buffer ambiguity is removed.
 * With SLOTS=1024, d=24 -> usable backlog up to 1000 samples.
 * NOTE: for the consume-on-ACK design (read held until the Photon ACKs), this
 * cap must be enforced on EVERY capture (in FlowLog), not only at report time,
 * so write can never lap a stalled read. See STEP 4. */
#define APP_FLOW_RING_MARGIN   24u

/* ---- Timer0 deep-sleep wake (defined in COUNTS) ----
 * Timer0 runs 8-bit from LFINTOSC (~31 kHz) with a FIXED 1:16384 prescaler.
 * T0CKPS field value E=14 gives prescaler 2^14; T0CON1 = 0x90 | E. One count =
 * 2^14 / 31 ms = 16384/31 = 528.5 ms (0.5285 s).
 *
 * The wake is APP_WAKE_COUNTS counts, so TMR0H = APP_WAKE_COUNTS directly and the
 * real wake time is an EXACT multiple of the tick => ZERO quantization error for
 * any value. APP_WAKE_ACTUAL_MS (below) is that exact time; the software clock
 * advances by it and the capture period is built from it, so the PIC's elapsed
 * time equals real count-multiple time (only LFINTOSC tolerance remains). */
#define APP_FLOW_T0CKPS  14u             /* FIXED 1:16384 -> 528.5 ms/count */

/* T0CON1: T0CS=100 (LFINTOSC), T0ASYNC=1, T0CKPS = E */
#define APP_FLOW_T0CON1  ((uint8_t)(0x90u | APP_FLOW_T0CKPS))

/* TMR0H is the wake count directly. */
#define APP_FLOW_TMR0H   ((uint8_t)APP_WAKE_COUNTS)

/* Reporting interval (seconds) reported/settable via GET/SET_PARAM. Kept
 * consistent with the build mode: ~30 min in test, 48 h in production. This is
 * the declared interval; the actual grid interval = captures * capture period. */
#ifdef REPORT_CONFIG_DEBUG
  #define LOCO_REPORT_INTERVAL_SEC  180UL       /* ~3 min (bench test) */
#else
  #define LOCO_REPORT_INTERVAL_SEC  (48UL*3600UL)
#endif

/* Nominal captures per report, DERIVED from the grid interval and the capture
 * period. This is NOT the authority for when a report fires: at run time the PIC
 * computes "captures until the next {anchor, interval} grid point" from the
 * cloud time. This nominal is used only to (a) size/bootstrap the ring batch
 * before the first cloud sync and (b) hint the Photon. Rounds to nearest. */
#define APP_NOMINAL_CAPTURES_PER_REPORT \
  ((uint16_t)((LOCO_REPORT_INTERVAL_SEC * 1000UL + APP_CAPTURE_PERIOD_MS / 2UL) \
              / APP_CAPTURE_PERIOD_MS))
#define APP_FLOW_BATCH  APP_NOMINAL_CAPTURES_PER_REPORT

#if (APP_WAKE_COUNTS > 255u)
  #error "APP_WAKE_COUNTS > 255: exceeds 8-bit Timer0 (max ~134.8 s at 1:16384)."
#endif
#if (APP_WAKE_COUNTS < 1u)
  #error "APP_WAKE_COUNTS < 1: wake must be at least one Timer0 count."
#endif

/* Exact wake time (ms) = counts * 16384/31, rounded to nearest ms. */
#define APP_WAKE_ACTUAL_MS \
  ((((uint32_t)APP_WAKE_COUNTS << APP_FLOW_T0CKPS) + 15UL) / 31UL)

/* ---- Compression method selection ----
 * Each method lives in its own Compress_*.c/.h. Only the SELECTED
 * one is compiled (its body is wrapped in #if), so unused methods
 * add zero RAM/ROM to the final binary.
 *
 * Method IDs (add new ones here as they are implemented): */
#define COMPRESS_METHOD_NOCOMPRESS_2B_2B   0   /* 2B time + 2B pulses = 4 B/sample */
#define COMPRESS_METHOD_PACK_10_10_4       1   /* 10b grp + 10b pulses + 4b unused = 3 B */
#define COMPRESS_METHOD_PACK_10_14         2   /* 10b sample# + 14b pulses = 3 B */
#define COMPRESS_METHOD_NOCOMP_16          3   /* 16b pulses only        = 2 B */

/* The active method: */
//#define COMPRESS_METHOD_SELECTED   COMPRESS_METHOD_NOCOMPRESS_2B_2B
//#define COMPRESS_METHOD_SELECTED   COMPRESS_METHOD_PACK_10_10_4
//#define COMPRESS_METHOD_SELECTED   COMPRESS_METHOD_PACK_10_14
#define COMPRESS_METHOD_SELECTED   COMPRESS_METHOD_NOCOMP_16   /* 2 B/sample, 16-bit pulses */

/* ---- Debug output master switch ---- */
/* Define -> Debug_Print_* and the "Sample-..." debug report lines
 * are produced. Undefine -> compiled out. */
/* APP_DEBUG_PRINT_ENABLE is set in the QUICK TEST SETTINGS block at the top. */

/* ---- Valid-data (report) output switch ---- */
/* Define -> the real "<grp>=<pulses> ." report lines are sent.
 * Undefine -> compiled out (e.g. to silence everything, or when
 * the report is delivered another way). */
#define APP_REPORT_PRINT_ENABLE

/* ---- Valve power/control drive switch ---- */
/* Valve CONTROL = RA2, Valve POWER = RC2 (both active-high).
 * Either way the pins are initialised as digital outputs and
 * start LOW in LEDs_Init().
 *   Define   -> they run the power/control waveform (Dev_Valve).
 *   Undefine -> they are left permanently LOW (driven once, no toggle). */
//#define VALVE_PWR_CTRL_ENABLE

/* ---- OP3 valve high-level driver (MValve_OP3) ----------------------------
 * VALVE_PWR_CTRL_ENABLE      master enable for valve driving (above).
 * VALVE_TEST_TOGGLE          with ENABLE: bench test toggle only, bypassing
 *                            the leak/lock logic (legacy waveform).
 * VALVE_ON_WHEN_STARTUP      on a NON-WDT reset (power-up / SW reset / MCLR),
 *                            force the valve OPEN once at startup. A WDT reset
 *                            keeps the current valve position (pins stay LOW).
 * OP3 model: PWR=L -> motor off, holds position. PWR=H,CTRL=H -> drive OPEN.
 *            PWR=H,CTRL=L -> drive CLOSE. After TIME_VALVE_FULL_TOGGLE_MS the
 *            driver forces BOTH pins LOW (saves the small holding current). */
/* VALVE_TEST_TOGGLE is set in the QUICK TEST SETTINGS block at the top. */
//#define VALVE_ON_WHEN_STARTUP

/* full open or full close drive time (motor self-cuts at the end stop) */
#define TIME_VALVE_FULL_TOGGLE_MS  10000UL

/* ---- Leak detection (Flow_Control) defaults --------------------------
 * Two independent alerts. Each fires when, within its window (quantized to
 * a capture count), the running no-zero sum reaches its threshold:
 *   alert1 -> temporary lock, auto-clears after TIME_VALVE_TEMP_LOCK_MS,
 *             then the valve auto-opens (if no permanent lock).
 *   alert2 -> permanent lock, cleared only by a Photon unlock packet or reset.
 * A single 0-count capture resets the running sum. Windows are in SECONDS;
 * Photon can update all four at runtime (GET/SET_PARAM). */
#ifdef REPORT_CONFIG_DEBUG
  /* fast bench values: trip quickly so you don't wait minutes */
  /* Bench thresholds are chosen so BOTH alerts are actually REACHABLE. The old
   * alert2 (10000 counts in 10 s = 1000 pulses/s) could never fire on a bench,
   * so the strong-alert path was never exercised. Keeping the design meaning:
   *   alert1 = LONG window,  LOW rate  ->  weak  -> temporary lock, auto-clears
   *   alert2 = SHORT window, HIGH rate ->  strong-> permanent lock
   * Rates:  alert1 = 50/20 s = 2.5 pulses/s   alert2 = 100/10 s = 10 pulses/s
   * The virtual meter schedule below crosses each of these on purpose. */
  #define APP_LEAK1_COUNTS_DEF     50u      /* alert1 threshold counts (2.5/s)*/
  #define APP_LEAK1_WINDOW_S_DEF   20u      /* alert1 window  (20 s in debug) */
  #define APP_LEAK2_COUNTS_DEF     100u     /* alert2 threshold counts (10/s) */
  #define APP_LEAK2_WINDOW_S_DEF   10u      /* alert2 window  (10 s in debug) */
  #define TIME_VALVE_TEMP_LOCK_MS  30000UL  /* temp lock holds 30 s in debug */
#else
  /* production values */
  #define APP_LEAK1_COUNTS_DEF     100u     /* alert1 threshold counts       */
  #define APP_LEAK1_WINDOW_S_DEF   480u     /* alert1 window  (8 min)        */
  #define APP_LEAK2_COUNTS_DEF     400u     /* alert2 threshold counts       */
  #define APP_LEAK2_WINDOW_S_DEF   180u     /* alert2 window  (3 min)        */
  #define TIME_VALVE_TEMP_LOCK_MS  600000UL /* temp lock holds 10 min        */
#endif

/* ---- WAKE line as "comms-ready" signal -------------------------------
 * WAKE (RC4) goes HIGH on: report-period due, a 0xF0 wake, OR any received
 * UART byte. It goes LOW once max(last RX byte time, TX buffer empty time)
 * is older than CLOSE_WAKE_AFTER_UART_MS. The decision variable (not the
 * pin) is what the sleep guard consults; sleep also needs all other guards
 * (report idle, TX shift-register empty, valve idle, dwell). */
#define CLOSE_WAKE_AFTER_UART_MS  500UL

/* ---- Deep-sleep wake (Timer0 + LFINTOSC) ---- */
/* Between captures the MCU enters full Sleep and is woken by Timer0 every
 * APP_FLOW_PERIOD_MS; Timer1 keeps counting pulses. The ON/OFF switch
 * (APP_SLEEP_ENABLE) lives in the QUICK TEST SETTINGS block at the top of this
 * file - it is the single source of truth. Undefined there -> always-awake
 * super-loop (recommended while bench testing). */

/* ---- UART wake (WUE): let Photon2's 0xF0 wake the PIC from Sleep ----
 * Define -> while asleep the UART watches RX; a 0xF0 edge wakes the PIC,
 * which then raises WAKE and waits for 0xAA (same as a batch-driven WAKE).
 * Requires APP_SLEEP_ENABLE. Undefine -> only Timer0 wakes the PIC. */
#define APP_UART_WAKE_ENABLE

/* ---- Watchdog Timer (optional safety net) ----------------------------
 * Define -> a ~4 s software-controlled WDT guards the AWAKE periods only:
 * it is started right after each wake and stopped right before Sleep (a
 * long, normal Sleep must never trigger it). The super-loop kicks it every
 * pass via WDT_KICK(); if the firmware ever hangs while awake (e.g. a stuck
 * UART wait), the WDT resets the MCU so it recovers on its own. A WDT reset
 * clears SRAM, so any not-yet-sent samples are lost -- that is the accepted
 * cost of an automatic recovery. Undefine -> WDT fully off (WDTE=OFF), and
 * all WDT_* macros below compile to nothing (zero cost). */
#define APP_WATCHDOG_ENABLE

#ifdef APP_WATCHDOG_ENABLE
  #define WDT_KICK()   CLRWDT()                       /* pet the dog        */
  #define WDT_START()  do { WDTCON0bits.SEN = 1; } while (0) /* on (awake)  */
  #define WDT_STOP()   do { WDTCON0bits.SEN = 0; } while (0) /* off (sleep) */
#else
  #define WDT_KICK()   ((void)0)
  #define WDT_START()  ((void)0)
  #define WDT_STOP()   ((void)0)
#endif

/* LED + valve toggle only while awake. Before each Sleep main() drives
 * LED / VALVE_PWR / VALVE_CTRL OFF, so nothing draws current while asleep
 * (sleep and valve drive may both be enabled and simply coexist). */

/* Minimum time to stay awake after a wake before sleeping again, so the
 * "LED on at wake" is visible. Set to 0 to sleep immediately (LED blip may
 * be too short to see). A capture itself needs almost no awake time. */
/* Minimum time the MCU stays awake after a wake before it may sleep again.
 * Keeps the wake-LED visible for a moment. Set to 0 to sleep immediately
 * after a capture-only wake (just write the sample, then sleep). */
//#define WAKEUP_TIME_MIN_MS    100UL
#define WAKEUP_TIME_MIN_MS    0

/* After a report becomes due (WAKE asserted), stay awake this long waiting
 * for Photon2 to answer with a REQ_DATA packet. If Photon2 is slow/absent/
 * broken, the PIC gives up after this and returns to Sleep. */
/* ---- DEBUG ONLY: start a data report without a REQ_DATA packet ----
 * Define -> when a report becomes due, the PIC begins sending RSP_DATA
 * immediately, as if a REQ_DATA had arrived (lets you observe the UART
 * report waveform with no Photon2 connected). Undefine for normal operation
 * (wait for a real REQ_DATA packet). TEMPORARY / bench use only. */
/* APP_DEBUG_AUTO_DATA_REPORT_WITHOUT_REQ is set in QUICK TEST SETTINGS (top). */

#define WAIT_PHOTON_UART_RESPONSE_MS  3000UL

/* ============================================================
 *  Photon POWER MANAGEMENT (PMOS power-gating model)
 * ------------------------------------------------------------
 *  RC4 (Dev_Led.h) drives an external P-MOS that switches the Photon's
 *  supply: RC4 LOW = Photon ON, RC4 HIGH = Photon OFF. The Photon has no
 *  wake source; it only (re)starts when the PIC re-applies power. main.c runs
 *  a 3-state power machine:
 *
 *    SLEEP        Photon OFF. PIC sleeps between captures (Timer0 wakes it).
 *                 When a report period is reached -> power Photon ON.
 *    WAIT_FIRST   Photon booting. Wait up to TIMEOUT_NO_MSG_PHOTON2PIC_MS for
 *                 ANY valid (CRC-good) packet. None -> Photon assumed dead ->
 *                 power OFF, back to SLEEP, retry next report period.
 *    ACTIVE       Handle packets (REQ_DATA->RSP_DATA, param/valve, ...).
 *                 Exit -> power OFF + SLEEP when EITHER:
 *                   (a) a PKT_PHOTON_OFF_REQ (func 0x07) arrives, or
 *                   (b) no further valid packet for TIMEOUT_NO_MORE_MSG_MS
 *                       after the last one was fully processed (safety net).
 *  Capture + leak detection run in ALL states (time-based, see FlowLog).
 */

/* Max wait after powering the Photon for its first valid packet. The Photon's
 * boot + cloud-connect is ~20-40 s, so 90 s leaves comfortable margin. */
#define TIMEOUT_NO_MSG_PHOTON2PIC_MS   90000UL

/* Safety-net idle timeout in ACTIVE: if no further valid packet arrives this
 * long after the last one finished processing (for REQ_DATA: after RSP_DATA
 * fully sent), the PIC powers the Photon off and sleeps. Normal shutdown is
 * the explicit PKT_PHOTON_OFF_REQ; this only covers a Photon that died mid-
 * session. Generous so it never cuts a slow cloud publish short. */
#define TIMEOUT_NO_MORE_MSG_MS         20000UL

/* Initial power-hold window. On a COLD power-up (not a WDT/soft reset) the PIC
 * keeps itself and the Photon fully powered for this long: it does NOT deep-
 * sleep and does NOT cut Photon power, while capture and valve on/off logic
 * keep running normally. This gives a continuous window to watch the flow meter
 * live over USB right after power-up, without waiting for the (possibly very
 * long) production report period.
 *
 * The PIC owns the timing. The Photon learns it is in this window by reading
 * PKT_REQ_POWER_STATE once at boot (reply 0 = INITIAL, 1 = NORMAL); if it reads
 * 0 it simply stays powered and does NOT self-sleep and does NOT send OFF_REQ -
 * the PIC cuts its power when the window ends. When the window elapses the PIC
 * actively powers the Photon off and returns to normal power-gated operation.
 * The window is one-shot (only the first cold-boot session); after it, the PIC
 * reports NORMAL for good. Capture keeps filling the ring buffer throughout, so
 * any report periods "missed" during the hold are simply collected in the next
 * normal session (the 1000-slot ring easily covers a 10-min hold). */
/* Initial cold-boot power-hold length.
 *   PRODUCTION            -> ALWAYS 10 minutes (for provisioning / first cloud).
 *   Fast test (REPORT_CONFIG_DEBUG) -> 30 s by default so bench cycles are quick;
 *     define TEST_INITIAL_HOLD_10MIN to force the full 10 min even in test. */
#ifdef REPORT_CONFIG_DEBUG
  //#define TEST_INITIAL_HOLD_10MIN     /* define -> 10 min in test; undefined -> 30 s */
  #ifdef TEST_INITIAL_HOLD_10MIN
    #define INITIAL_POWER_HOLD_MS      (10UL * 60UL * 1000UL)   /* test: 10 minutes */
  #else
    #define INITIAL_POWER_HOLD_MS      (60UL * 1000UL)           /* test: 1 minute */
  #endif
#else
  #define INITIAL_POWER_HOLD_MS        (10UL * 60UL * 1000UL)   /* PRODUCTION: 10 minutes */
#endif

/* On every cold boot (power-up or software RESET from SYS_RESET) the PIC first
 * holds the Photon power OFF this long, so the Photon rail fully collapses and
 * the Photon truly cold-boots (re-detects "PIC first boot"). ~3 s. */
#ifndef PHOTON_COLDBOOT_OFF_MS
#define PHOTON_COLDBOOT_OFF_MS         (8UL * 1000UL)
#endif

/* NOTE: TIMEOUT_CANNOT_FIND_CLOUD_MS (80 s) lives on the PHOTON side: if the
 * Photon cannot reach the cloud within 80 s it sends PKT_PHOTON_OFF_REQ with
 * reason OFF_REASON_CLOUD_FAIL, so it must fire BEFORE the PIC's 90 s cutoff. */

/* ---- UART ---- */
/* Packet UART (PIC <-> Photon) ONLY. The debug UART has its own constant -
 * APP_DEBUG_UART2_BAUD above - and must stay at 38400: the debug FIFO already
 * overflows at that speed, and slowing it would cost us the diagnostic log.
 *
 * BENCH DIAGNOSTIC, 2026-08: temporarily lowered 38400 -> 9600 to separate two
 * candidate causes of the PIC->Photon frame corruption seen on the bench
 * (rx 16/17 bytes truncated, rx 18 bytes with bad CRC, rx 0 bytes):
 *   fixed at 9600 -> signal integrity / noise. One bit is 4x longer, so the
 *                    margin against a glitch of fixed duration is 4x larger.
 *   unchanged     -> oscillator (baud) error. That is a RELATIVE error, so it
 *                    scales with the baud rate and lowering it cannot help.
 * The Photon build MUST be changed to match; a mismatch gives no link at all.
 * Restore 38400 once the cause is found and fixed.
 *
 * Divider accuracy at BRGS=1: 64 MHz / (4 x 1666) = 9600.4 Bd, error +0.004%
 * (38400 was -0.08%), so the divider itself is not a factor either way. */
#define APP_UART_BAUD         9600UL       /* terminal must match. 38400 = normal */
/* TX ring buffer size. The longest single transmission decides this:
 *   debug ON  -> a "Sample-... Raw=.." debug line can reach ~93 bytes,
 *                so use 128 to hold debug + report together.
 *   debug OFF -> only the short "<grp>=<pulses> ." report line (<= ~13 B)
 *                is ever sent, so 64 is plenty and saves RAM. */
#ifdef APP_DEBUG_PRINT_ENABLE
  #define APP_UART_TX_BUF_SIZE  128u
#else
  #define APP_UART_TX_BUF_SIZE  64u
#endif

/* RX ring buffer: holds incoming packet bytes until the parser consumes them.
 * Longest inbound packet = SET_PARAM (7 overhead + 8 data = 15 B). 32 gives
 * margin and is a power of two for cheap masking. */
#define APP_UART_RX_BUF_SIZE  32u

/* ---- Photon2 WAKE signal (comms-ready model) ----
 * WAKE (RC4, defined in Dev_Led.h) is NOT a fixed pulse anymore. It is a
 * "comms-ready" level: it goes HIGH on a report-period being due, on the 0xF0
 * wake byte, or on ANY received UART byte, and it goes LOW only once the last
 * UART activity (last RX byte / TX buffer empty) is older than
 * CLOSE_WAKE_AFTER_UART_MS (defined above, near the top of this file).
 * The old APP_WAKE_PULSE_MS / APP_WAKE_TO_REPORT_MS pulse parameters were
 * removed because the waveform is no longer a pulse. */

/* ---- System time tick ---- */
/* Define -> 1 ms tick advanced inside the Timer2 ISR.
 * Undefine -> advanced by polling in MCU_Time_Process() from main. */
#define APP_SYS_TIME_USE_ISR

/* ---- LED behavior (led_fsm_sysstate) ---- */
#define APP_LED_HEARTBEAT_MS  500UL        /* (legacy) toggle every 0.5 s        */
#define APP_LED_BLINK_MS      100UL        /* (legacy) half-period of blink       */
#define APP_LED_BLINK_COUNT   3u           /* (legacy) fast toggles on 0xAA       */

/* Contract LED behaviors (TEST LED, RC3):
 *  (1) ALIVE: during the initial power-hold window, one short blink every
 *      APP_LED_ALIVE_EVERY_N-th capture ("wake") event, then stop.
 *  (2) TX   : during RSP_DATA transmission, APP_LED_TX_HALF_MS ON / OFF. */
/* ---- TEST LED (RC3) ------------------------------------------------------
 * V028: both behaviours were re-anchored so they can actually be SEEN. See
 * led_fsm_sysstate.h for the full reasoning; the short version:
 *
 *  (1) The 3 s toggle now runs for as long as the PIC holds the Photon
 *      powered, not just while an RSP_DATA frame streams. Powering the Photon
 *      IS the data exchange. The frame alone is 0.1 s (bench) to 2.1 s (worst
 *      case) - it could never complete one 3 s half-period, so anchored there
 *      the LED only ever flashed once and the contract's 3 s never appeared.
 *      Anchored on the session it is visible for the whole ~23 s exchange and
 *      for the entire 10-minute initial hold.
 *
 *  (2) The alive blink now counts FIFO WRITES (captures) while the Photon is
 *      OFF, lit on the wake that performs the write and cleared as the PIC
 *      goes back to sleep. The old "every 10th capture" was 53 s on the bench
 *      and 40 min in production - in production the 10-minute hold would end
 *      before a single blink appeared. */
#define APP_LED_PHOTON_HALF_MS  3000UL     /* Photon powered: 3 s ON / 3 s OFF   */

/* Capture (FIFO-write) blink while the Photon is OFF.
 *   0 = no capture blink at all. The board is then observable only by its
 *       reports - choose this if the LED must not cost anything.
 *   1 = blink on every FIFO write (default)
 *   2 = every 2nd write, and so on. Only the lit cycles pay the hold below. */
#define APP_LED_CAPTURE_EVERY_N   1u

/* How long to keep the PIC awake so the capture blink is actually visible.
 *   0 = no extra hold (default). The LED is on only for the natural awake
 *       processing of that wake, then the PIC sleeps immediately.
 *  >0 = stay awake at least this many ms. ~5 ms was found to be the practical
 *       threshold at which the flash becomes visible to the eye. Costs awake
 *       time on every lit cycle, so raise it only when the blink must be seen. */
#define APP_LED_CAPTURE_HOLD_MS   0u

/* ---- Valve waveform (Dev_Valve), active only if VALVE_PWR_CTRL_ENABLE ----
 * PWR  : HIGH 10 s, LOW 10 s  (period 20 s)
 * CTRL : HIGH  5 s, LOW  5 s  (period 10 s), synchronized to PWR start.
 * CTRL period must divide PWR period for clean sync. */
#define APP_VALVE_PWR_PERIOD_MS   20000UL  /* PWR full cycle  */
#define APP_VALVE_PWR_HIGH_MS     10000UL  /* PWR HIGH part   */
#define APP_VALVE_CTRL_PERIOD_MS  10000UL  /* CTRL full cycle */
#define APP_VALVE_CTRL_HIGH_MS     5000UL  /* CTRL HIGH part  */

#endif /* APP_CONFIG_H */
