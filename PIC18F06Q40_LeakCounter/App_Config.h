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
  #define APP_FLOW_SLOTS      25u          /* ring-buffer slots */
  #define APP_FLOW_PERIOD_MS  1000UL       /* capture interval: 1 s */
  #define APP_FLOW_BATCH      10u          /* samples per report batch */
#else
  #define APP_FLOW_SLOTS      360u         /* ring-buffer slots (360 x 1 min = 6 h) */
  #define APP_FLOW_PERIOD_MS  60000UL      /* capture interval: 60 s */
  #define APP_FLOW_BATCH      10u          /* samples per report batch */
#endif

/* ---- Compression method selection ----
 * Each method lives in its own Compress_*.c/.h. Only the SELECTED
 * one is compiled (its body is wrapped in #if), so unused methods
 * add zero RAM/ROM to the final binary.
 *
 * Method IDs (add new ones here as they are implemented): */
#define COMPRESS_METHOD_NOCOMPRESS_2B_2B   0   /* 2B time + 2B pulses = 4 B/sample */
#define COMPRESS_METHOD_PACK_10_10_4       1   /* 10b grp + 10b pulses + 4b unused = 3 B */
/* #define COMPRESS_METHOD_xxxx            2   (future) */

/* The active method: */
#define COMPRESS_METHOD_SELECTED   COMPRESS_METHOD_NOCOMPRESS_2B_2B

/* ---- UART ---- */
#define APP_UART_BAUD         38400UL      /* terminal must match */
#define APP_UART_TX_BUF_SIZE  128u         /* TX ring buffer (>= longest line) */

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

/* ---- Debug output master switch ---- */
/* Define -> Debug_Print_* and the "Sample-..." debug report lines
 * are produced. Undefine -> compiled out. */
//#define APP_DEBUG_PRINT_ENABLE

/* ---- Valid-data (report) output switch ---- */
/* Define -> the real "<grp>=<pulses> ." report lines are sent.
 * Undefine -> compiled out (e.g. to silence everything, or when
 * the report is delivered another way). */
#define APP_REPORT_PRINT_ENABLE

/* ---- LED behavior (led_fsm_sysstate) ---- */
#define APP_LED_HEARTBEAT_MS  2000UL       /* normal: toggle every 2 s */
#define APP_LED_BLINK_MS      100UL        /* alert: half-period of blink */
#define APP_LED_BLINK_COUNT   5u           /* alert: number of blinks */

#endif /* APP_CONFIG_H */
