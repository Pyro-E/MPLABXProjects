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
//#define REPORT_CONFIG_DEBUG   /* DEFAULT = PRODUCTION. Define this for fast bench test. */

 #define REPORT_INTERVAL_HR 24      // 
// #define REPORT_INTERVAL_HR 48


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

/* ---- 6. DEBUG print lines (human-readable) ---------------------------
 * Defined -> extra "Sample-..." text lines are emitted on the UART. NOTE:
 *            this text is interleaved into the byte stream and WILL corrupt
 *            the binary packet protocol, so keep it UNDEFINED whenever a
 *            Photon is connected. Bench/terminal observation only. */
//#define APP_DEBUG_PRINT_ENABLE

/* ---- 7. DEBUG: packet-flow log (human-readable) ---------------------
 * Defined -> the PIC prints a short line for every packet it RECEIVES
 *            (RX: REQ_DATA, RX: GET_PARAM, ...) and a one-line summary of each
 *            RSP_DATA it sends (the extended-header fields: impulse/captures/
 *            span/overflow/count). Lets a bench PC (PIC UART -> USB-serial,
 *            NO Photon) watch the exchange. Like all debug text this INTERLEAVES
 *            with the binary protocol, so keep it UNDEFINED with a Photon
 *            attached. */
//#define APP_DEBUG_PKT_LOG

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
 *   3) REPORT : APP_SAMPLES_PER_REPORT - a report is due every M captures.
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
  #define APP_FLOW_SLOTS         1000     /* ring-buffer slots (SRAM limit)          */
  #define APP_WAKE_COUNTS        6u       /* TEST: 6 x 0.5285 s = 3.17 s wake        */
  #define APP_WAKES_PER_SAMPLE   4u       /* capture = 4 wakes (24 x 0.5285) = 12.68 s */
  #define APP_SAMPLES_PER_REPORT 10u      /* report  = 10 captures = 126.8 s         */
#else
  #if REPORT_INTERVAL_HR == 24
    #define APP_FLOW_SLOTS         1000     /* <=1024 (10-14 sample# limit)            */
    #define APP_WAKE_COUNTS        5u //6u //114u     /* PRODUCTION: 114 x 0.5285 s = 60.25 s wake*/
    #define APP_WAKES_PER_SAMPLE   1u //4u       /* capture = 4 wakes (456 x 0.5285) = 241.0 s (~4 min) */
    #define APP_SAMPLES_PER_REPORT 720u     /* report  = 720 captures = ~48 hours       */
  #endif
  #if REPORT_INTERVAL_HR == 48
    #define APP_FLOW_SLOTS         1000     /* <=1024 (10-14 sample# limit)            */
    #define APP_WAKE_COUNTS        114u     /* PRODUCTION: 114 x 0.5285 s = 60.25 s wake*/
    #define APP_WAKES_PER_SAMPLE   4u       /* capture = 4 wakes (456 x 0.5285) = 241.0 s (~4 min) */
    #define APP_SAMPLES_PER_REPORT 720u     /* report  = 720 captures = ~48 hours       */
  #endif
#endif

/* Derived: capture period (ring-buffer sample interval) and report batch.
 * FlowLog gates captures on APP_CAPTURE_PERIOD_MS; the leak window converts
 * seconds->captures with the same value. Built from APP_WAKE_ACTUAL_MS (the real
 * count-multiple wake) so the capture window has NO quantization error. */
#define APP_CAPTURE_PERIOD_MS  (APP_WAKE_ACTUAL_MS * (uint32_t)APP_WAKES_PER_SAMPLE)
#define APP_FLOW_BATCH         APP_SAMPLES_PER_REPORT

/* Safety margin (d) for the ring buffer. If the un-sent backlog exceeds
 * (SLOTS - d) we stop trying to drain the whole ring and send only the most
 * recent "last report attempt -> now" span (see FlowReport). Keeps ring writes
 * and reads from colliding. */
#define APP_FLOW_RING_MARGIN   100u

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
#define COMPRESS_METHOD_PACK_10_14         2
/* #define COMPRESS_METHOD_xxxx            3   (future) */

/* The active method: */
//#define COMPRESS_METHOD_SELECTED   COMPRESS_METHOD_NOCOMPRESS_2B_2B
//#define COMPRESS_METHOD_SELECTED   COMPRESS_METHOD_PACK_10_10_4
#define COMPRESS_METHOD_SELECTED   COMPRESS_METHOD_PACK_10_14

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
  #define APP_LEAK1_COUNTS_DEF     50u      /* alert1 threshold counts       */
  #define APP_LEAK1_WINDOW_S_DEF   20u      /* alert1 window  (20 s in debug)*/
  #define APP_LEAK2_COUNTS_DEF     10000u     /* alert2 threshold counts       */
  #define APP_LEAK2_WINDOW_S_DEF   10u      /* alert2 window  (10 s in debug)*/
  #define TIME_VALVE_TEMP_LOCK_MS  30000UL  /* temp lock holds 30 s in debug */
#else
  /* production values */
  #define APP_LEAK1_COUNTS_DEF     5000u     /* alert1 threshold counts for  leaks   */
  #define APP_LEAK1_WINDOW_S_DEF   3600u     /* alert1 window for counting (sec)        */
  #define APP_LEAK2_COUNTS_DEF     400u     /* alert2 threshold counts for overflows     */
  #define APP_LEAK2_WINDOW_S_DEF   180u     /* alert2 window for counting (sec)        */
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
    #define INITIAL_POWER_HOLD_MS      (100UL * 1000UL)          /* test: 30 seconds */
  #endif
#else
  #define INITIAL_POWER_HOLD_MS        (3UL * 60UL * 1000UL)   /* PRODUCTION:  minutes */
#endif

/* NOTE: TIMEOUT_CANNOT_FIND_CLOUD_MS (80 s) lives on the PHOTON side: if the
 * Photon cannot reach the cloud within 80 s it sends PKT_PHOTON_OFF_REQ with
 * reason OFF_REASON_CLOUD_FAIL, so it must fire BEFORE the PIC's 90 s cutoff. */

/* ---- UART ---- */
#define APP_UART_BAUD         38400UL      /* terminal must match */
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
#define APP_LED_HEARTBEAT_MS  500UL        /* toggle every 0.5 s -> 1 s on/off */
#define APP_LED_BLINK_MS      100UL        /* alert: half-period of blink */
#define APP_LED_BLINK_COUNT   3u           /* fast toggles on a successful 0xAA */

/* ---- Valve waveform (Dev_Valve), active only if VALVE_PWR_CTRL_ENABLE ----
 * PWR  : HIGH 10 s, LOW 10 s  (period 20 s)
 * CTRL : HIGH  5 s, LOW  5 s  (period 10 s), synchronized to PWR start.
 * CTRL period must divide PWR period for clean sync. */
#define APP_VALVE_PWR_PERIOD_MS   20000UL  /* PWR full cycle  */
#define APP_VALVE_PWR_HIGH_MS     10000UL  /* PWR HIGH part   */
#define APP_VALVE_CTRL_PERIOD_MS  10000UL  /* CTRL full cycle */
#define APP_VALVE_CTRL_HIGH_MS     5000UL  /* CTRL HIGH part  */

#endif /* APP_CONFIG_H */
