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
#define REPORT_CONFIG_DEBUG

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

/* ===== end QUICK TEST SETTINGS - details follow below ================ */


/* ---- System clock (must match the RSTOSC config bits) ---- */
#define APP_FOSC_HZ            64000000UL   /* 64 MHz internal */

/* ---- Flow data capture / logging ----
 * REPORT_CONFIG_DEBUG is set ONCE in the QUICK TEST SETTINGS block at the top
 * of this file; do not redefine it here. The #ifdef below simply consumes it. */

#ifdef REPORT_CONFIG_DEBUG
  #define APP_FLOW_SLOTS      1000          /* ring-buffer slots */
  #define APP_FLOW_PERIOD_MS  1000       /* fast test: ~2 s capture */
  #define APP_FLOW_BATCH      (1*720)          /* report every 5 captures */
#else
  #define APP_FLOW_SLOTS      1000         /* <=1024 (10-14 sample# limit) */
  #define APP_FLOW_PERIOD_MS  60000UL      /* production: 60 s capture */
  #define APP_FLOW_BATCH      (60*12)      /* 12 h report @ 1 min */
#endif

/* ---- Timer0 deep-sleep wake period (single source of truth) ----
 * Timer0 runs 8-bit from LFINTOSC (~31 kHz). The prescaler is chosen
 * AUTOMATICALLY from APP_FLOW_PERIOD_MS so the deep-sleep wake interval can
 * match capture periods both long and short:
 *   - period >= 500 ms : keep the current 1:16384 prescaler (528 ms/count,
 *                        lowest power, max ~134 s).
 *   - period <  500 ms : step down to the largest prescaler whose single
 *                        count still fits the period, down to 1:256
 *                        (8.25 ms/count), so sub-500 ms periods are honored.
 *
 * T0CKPS field value E gives prescaler 2^E (E=14 ->16384 ... E=8 ->256),
 * and T0CON1 = 0x90 | E  (T0CS=LFINTOSC, T0ASYNC=1, T0CKPS=E).
 * One count = 2^E / 31 ms. TMR0H = round(period_ms / count).
 *
 * NOTE: deep-sleep timing comes from the ~31 kHz LFINTOSC, so the real
 * interval has a few-percent tolerance and is quantized to one count; the
 * software capture clock still advances by the exact APP_FLOW_PERIOD_MS. */

#if   APP_FLOW_PERIOD_MS >= 500UL
  #define APP_FLOW_T0CKPS  14u    /* 1:16384, 528 ms/count (current; low power) */
#elif APP_FLOW_PERIOD_MS >= 9UL
  #define APP_FLOW_T0CKPS  8u     /* 1:256, 8.25 ms/count (fine: accurate <500 ms) */
#else
  #error "APP_FLOW_PERIOD_MS too small (min ~9 ms with Timer0 1:256 in deep sleep)."
#endif

/* T0CON1: T0CS=100 (LFINTOSC), T0ASYNC=1, T0CKPS = selected E */
#define APP_FLOW_T0CON1  ((uint8_t)(0x90u | APP_FLOW_T0CKPS))

/* TMR0H = round(period_ms / (2^E / 31 ms)) = round(period_ms*31 / 2^E) */
#define APP_FLOW_TMR0H \
  ((uint8_t)(((APP_FLOW_PERIOD_MS * 31UL) + (1UL << (APP_FLOW_T0CKPS - 1u))) \
             / (1UL << APP_FLOW_T0CKPS)))

#if (((APP_FLOW_PERIOD_MS * 31UL) + (1UL << (APP_FLOW_T0CKPS - 1u))) \
     / (1UL << APP_FLOW_T0CKPS)) > 255UL
  #error "APP_FLOW_PERIOD_MS too large for 8-bit Timer0 at the selected prescaler."
#endif
#if (((APP_FLOW_PERIOD_MS * 31UL) + (1UL << (APP_FLOW_T0CKPS - 1u))) \
     / (1UL << APP_FLOW_T0CKPS)) < 1UL
  #error "APP_FLOW_PERIOD_MS too small for the selected Timer0 prescaler."
#endif

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
  #define APP_LEAK1_COUNTS_DEF     100u     /* alert1 threshold counts       */
  #define APP_LEAK1_WINDOW_S_DEF   480u     /* alert1 window  (8 min)        */
  #define APP_LEAK2_COUNTS_DEF     400u     /* alert2 threshold counts       */
  #define APP_LEAK2_WINDOW_S_DEF   180u     /* alert2 window  (3 min)        */
  #define TIME_VALVE_TEMP_LOCK_MS  600000UL /* temp lock holds 10 min        */
#endif

/* ---- WAKE line as "comms-ready" signal (active-LOW) ------------------
 * WAKE (RC4) goes LOW on: report-period due, a 0xF0 wake, OR any received
 * UART byte. It goes HIGH (idle) once max(last RX byte time, TX buffer
 * empty time) is older than CLOSE_WAKE_AFTER_UART_MS. The decision variable
 * (not the pin) is what the sleep guard consults; sleep also needs all other
 * guards (report idle, TX shift-register empty, valve idle, dwell). */
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
#define WAIT_PHOTON_UART_RESPONSE_MS  3000UL  // wait up to 3 s for REQ_DATA after WAKE asserted

/* ---- WAKE-to-TX delay ------------------------------------------------
 * After WAKE is asserted LOW (report due), the PIC holds off accepting a
 * REQ_DATA and starting the data transmission for this long. This existed
 * to give the Photon2 time to cold-boot (via the D10 power-enable pulse)
 * and connect to the Particle cloud before a large upload began.
 * The WAKE_WAIT state timeout is extended by this value automatically.
 *
 * D10/WAKE is no longer read by the Photon as a "data ready" GPIO signal
 * (it now only powers the Photon2 on). The Photon2 runs continuously and
 * polls the PIC for data on its own fixed timer, so it is already awake
 * and cloud-connected by the time it sends REQ_DATA -- this hold-off no
 * longer serves a purpose and instead makes the PIC miss the Photon2's
 * (much shorter) reply timeout. Set to 0 to transmit as soon as REQ_DATA
 * arrives. */
#define WAKE_TO_TX_DELAY_MS  0UL

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
 * "comms-ready" level: it goes LOW (asserted) on a report-period being due,
 * on the 0xF0 wake byte, or on ANY received UART byte, and it goes HIGH
 * (deasserted/idle) only once the last
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
