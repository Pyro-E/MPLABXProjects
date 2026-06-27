#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ============================================================
 *  App_Config.h  -  central place for all tunable settings
 *
 *  Edit values here; the driver modules include this file and
 *  use these macros. Nothing else needs to be touched to change
 *  capture timing, buffer sizes, baud rate, etc.
 * ============================================================ */

/* ---- System clock (must match the RSTOSC config bits) ---- */
#define APP_FOSC_HZ            64000000UL   /* 64 MHz internal */

/* ---- Flow data capture / logging ----
 * REPORT_CONFIG_DEBUG defined  -> small/fast values for quick testing.
 * REPORT_CONFIG_DEBUG undefined -> production values (1 min, 360, 360).
 * Toggle the single line below to switch every related setting at once.
 * Default: debug. */
#define REPORT_CONFIG_DEBUG

#ifdef REPORT_CONFIG_DEBUG
  #define APP_FLOW_SLOTS      1000          /* ring-buffer slots */
  #define APP_FLOW_PERIOD_MS  1000       /* fast test: ~2 s capture */
  #define APP_FLOW_BATCH      5           /* report every 5 captures */
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
//#define APP_DEBUG_PRINT_ENABLE

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

/* ---- Deep-sleep wake (Timer0 + LFINTOSC) ---- */
/* Define -> between captures the MCU enters full Sleep and is woken by
 * Timer0 every APP_FLOW_PERIOD_MS; Timer1 keeps counting pulses.
 * Undefine -> original always-awake super-loop (no power saving). */
#define APP_SLEEP_ENABLE

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
//#define WAKEUP_TIME_MIN_MS    50UL
#define WAKEUP_TIME_MIN_MS    0

/* After a full batch fires a WAKE pulse, stay awake this long waiting for
 * Photon2 to answer on the UART (0xAA). If Photon2 is slow/absent/broken,
 * the PIC blinks for this long, then gives up and returns to Sleep.
 * Must be >= APP_WAKE_PULSE_MS. */
/* ---- DEBUG ONLY: start the report without waiting for Photon2's 0xAA ----
 * Define -> right after the WAKE pulse, the PIC pretends a 0xAA arrived and
 * begins sending immediately (lets you see the UART report with no Photon2).
 * Undefine for normal operation (wait for the real 0xAA). TEMPORARY. */
//#define APP_DEBUG_AUTO_AA

#define WAIT_PHOTON_UART_RESPONSE_MS  3000UL

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

/* ---- Photon2 wake signal + report timing ----
 * Sequence each report cycle (t from WAKE going HIGH):
 *   t=0      : WAKE -> HIGH (PHOTON2_WAKE_ON, defined in Dev_Led.h)
 *   t=PULSE  : WAKE -> LOW  (PHOTON2_WAKE_OFF)
 *   t=DELAY  : start UART report
 * The WAKE pin (RC4) itself is defined in Dev_Led.h. */
#define APP_WAKE_PULSE_MS     500UL         /* WAKE stays HIGH this long */
#define APP_WAKE_TO_REPORT_MS 1000UL        /* delay from WAKE high to report start */

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
