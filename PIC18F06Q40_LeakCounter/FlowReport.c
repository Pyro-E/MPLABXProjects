#define FLOWREPORT_C

#include <xc.h>
#include <stdint.h>
#include <string.h>
#include "App_Config.h"
#include "FlowReport.h"
#include "FlowLog.h"
#include "FlowMeter.h"
#include "Dev_Uart.h"
#include "Dev_Led.h"
#include "led_fsm_sysstate.h"
#include "MCU_Time.h"

/* ============================================================
 *  Report cycle FSM (non-blocking). Timeline (t measured from
 *  the moment WAKE goes HIGH):
 *    t = 0                      : WAKE -> HIGH  (wake Photon2)
 *    t = APP_WAKE_PULSE_MS      : WAKE -> LOW
 *    t = APP_WAKE_TO_REPORT_MS  : start UART report (one line/turn)
 *  WAKE idles LOW between cycles.
 * ============================================================ */
typedef enum {
    ST_IDLE = 0,     /* waiting for BATCH new samples */
    ST_WAKE_PULSE,   /* WAKE high, waiting to lower it */
    ST_WAKE_WAIT,    /* WAKE low, waiting until report time */
    ST_SENDING       /* sending the batch, one line per turn */
} report_state_t;

static report_state_t s_state    = ST_IDLE;
static uint16_t       s_read     = 0;   /* free-running consumer index */
static uint16_t       s_i        = 0;   /* sample within current batch (0..BATCH-1) */
static uint32_t       s_wake_ms  = 0;   /* timestamp when WAKE went high */

void FlowReport_Init(void)
{
    /* WAKE pin is configured by LEDs_Init() and idles LOW. */
    s_state    = ST_IDLE;
    s_read     = 0;
    s_i        = 0;
}

/* ---- unsigned->ASCII helpers ---- */
#if defined(APP_DEBUG_PRINT_ENABLE) || defined(APP_REPORT_PRINT_ENABLE)
static uint8_t u16_to_buf(char *p, uint16_t v)
{
    char t[5];
    uint8_t n = 0;
    do { t[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v > 0);
    for (uint8_t k = 0; k < n; k++) p[k] = t[n - 1 - k];
    return n;
}
#endif

#ifdef APP_DEBUG_PRINT_ENABLE
static uint8_t u32_to_buf(char *p, uint32_t v)
{
    char t[10];
    uint8_t n = 0;
    do { t[n++] = (char)('0' + (uint8_t)(v % 10u)); v /= 10u; } while (v > 0);
    for (uint8_t k = 0; k < n; k++) p[k] = t[n - 1 - k];
    return n;
}
static uint8_t u16_to_buf_pad(char *p, uint16_t v, uint8_t width)
{
    char t[5];
    uint8_t n = 0;
    do { t[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v > 0);
    uint8_t w = 0;
    while ((n + w) < width) { p[w++] = '0'; }
    for (uint8_t k = 0; k < n; k++) p[w + k] = t[n - 1 - k];
    return (uint8_t)(w + n);
}
#endif

/* ---- DEBUG line: "Sample-<buf>-<grp> : Pulse=.., Total=.., SampleNumber=.." */
#ifdef APP_DEBUG_PRINT_ENABLE
static uint8_t build_debug_line(char *line, uint16_t readIdx)
{
    flowlog_entry_t e;
    FlowLog_GetAt(readIdx, &e);

    /* buffer slot shown as 0..SLOTS-1 even though readIdx is free-running */
    uint16_t bufNo = (uint16_t)(readIdx % FLOWLOG_SLOTS);

    uint8_t p = 0;
    memcpy(&line[p], "Sample-", 7);            p += 7;
    p += u16_to_buf_pad(&line[p], bufNo, 4);       /* buffer slot 0..SLOTS-1 */
    line[p++] = '-';
    p += u16_to_buf_pad(&line[p], e.grp, 4);       /* sample no within group */
    memcpy(&line[p], " : Pulse=", 9);          p += 9;
    p += u16_to_buf(&line[p], e.pulses);
    memcpy(&line[p], ", Total=", 8);           p += 8;
    p += u32_to_buf(&line[p], FlowMeter_GetTotal());
    memcpy(&line[p], ", SampleNumber=", 15);   p += 15;
    p += u32_to_buf(&line[p], FlowLog_GetCaptureCount());
    line[p++] = '\r';
    line[p++] = '\n';
    line[p]   = '\0';
    return p;
}
#endif /* APP_DEBUG_PRINT_ENABLE */

/* ---- REAL report line (always sent): "<grp>=<pulses> .\r\n" ----
 * Fields separated by space-dot-CRLF for an unambiguous record end. */
#ifdef APP_REPORT_PRINT_ENABLE
static uint8_t build_report_line(char *line, uint16_t readIdx)
{
    flowlog_entry_t e;
    FlowLog_GetAt(readIdx, &e);

    uint8_t p = 0;
    p += u16_to_buf(&line[p], e.grp);      /* sample no within group */
    line[p++] = '=';
    p += u16_to_buf(&line[p], e.pulses);   /* pulse count */
    line[p++] = ' ';
    line[p++] = '.';
    line[p++] = '\r';
    line[p++] = '\n';
    line[p]   = '\0';
    return p;
}
#endif /* APP_REPORT_PRINT_ENABLE */

void FlowReport_Process(void)
{
    switch (s_state) {

    case ST_IDLE: {
        /* a full group of FLOWLOG_BATCH samples just completed */
        if (FlowLog_BatchReady()) {
            PHOTON2_WAKE_ON;           /* wake Photon2 (RC4 high) */
            LedFsm_NotifyDataCycle();  /* LED: start blink burst */
            s_wake_ms = getNowTime();
            s_state   = ST_WAKE_PULSE;
        }
        break;
    }

    case ST_WAKE_PULSE:
        /* hold WAKE high for APP_WAKE_PULSE_MS, then lower it */
        if (timeSpan(s_wake_ms) >= APP_WAKE_PULSE_MS) {
            PHOTON2_WAKE_OFF;          /* lower WAKE (RC4 low) */
            s_state = ST_WAKE_WAIT;
        }
        break;

    case ST_WAKE_WAIT:
        /* wait until APP_WAKE_TO_REPORT_MS after WAKE went high */
        if (timeSpan(s_wake_ms) >= APP_WAKE_TO_REPORT_MS) {
            s_i     = 0;
            s_state = ST_SENDING;
        }
        break;

    case ST_SENDING: {
        /* Build whichever lines are enabled, make sure they all fit
         * wholly into the UART buffer, then send them together so no
         * line is ever split. */
        uint16_t need = 0;

#ifdef APP_DEBUG_PRINT_ENABLE
        char dline[96];
        uint8_t dlen = build_debug_line(dline, s_read);
        need += dlen;
#endif
#ifdef APP_REPORT_PRINT_ENABLE
        char rline[24];
        uint8_t rlen = build_report_line(rline, s_read);
        need += rlen;
#endif

        if (need > 0u && UART_TX_Free() < need) {
            break;                 /* not enough room; retry next turn */
        }

#ifdef APP_DEBUG_PRINT_ENABLE
        print_string(dline);       /* debug (Sample-...) */
#endif
#ifdef APP_REPORT_PRINT_ENABLE
        print_string(rline);       /* real report (<grp>=<pulses> .) */
#endif

        s_i++;
        s_read++;   /* free-running */

        if (s_i >= FLOWLOG_BATCH) {
            s_state = ST_IDLE;     /* batch done */
        }
        break;
    }

    default:
        s_state = ST_IDLE;
        break;
    }
}
