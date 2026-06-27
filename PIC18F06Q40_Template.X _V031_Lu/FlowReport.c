#define FLOWREPORT_C

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "App_Config.h"
#include "FlowReport.h"
#include "FlowLog.h"
#include "FlowMeter.h"
#include "Compress.h"
#include "Dev_Uart.h"
#include "Dev_Led.h"
#include "led_fsm_sysstate.h"
#include "MCU_Time.h"

/* ============================================================
 *  FlowReport - two INDEPENDENT state machines.
 *
 *  (1) WAKE machine
 *      When the number of not-yet-sent samples reaches FLOWLOG_BATCH,
 *      a WAKE pulse is emitted (RC4 high for APP_WAKE_PULSE_MS, then
 *      low). WAKE only tells the Photon2 "you may collect now"; it no
 *      longer starts the transmission. If reports happen often enough
 *      that the backlog never reaches FLOWLOG_BATCH, WAKE never fires.
 *
 *  (2) SEND machine  (triggered ONLY by receiving 0xAA)
 *      On 0xAA the current write position is snapshotted as the end
 *      mark. The PIC then sends:
 *          - a 4-byte big-endian (MSB-first) COUNT of samples, and
 *          - that many samples as raw packed bytes,
 *      one sample per turn so the small UART TX buffer never overflows.
 *      Samples captured AFTER the 0xAA are kept for the next report.
 *      The per-sample index is RE-NUMBERED from 0 for every report
 *      (so it restarts at 0 each upload, and may exceed BATCH-1 if a
 *      WAKE went unanswered and the backlog grew large).
 *
 *  The two machines share only the consumer index s_read.
 * ============================================================ */

typedef enum {
    WAKE_IDLE = 0,   /* watching the backlog */
    WAKE_HIGH        /* WAKE held high, waiting to lower it */
} wake_state_t;

typedef enum {
    SEND_IDLE = 0,   /* waiting for a 0xAA request */
    SEND_HEADER,     /* sending the 4-byte count */
    SEND_BODY        /* sending samples up to the end mark */
} send_state_t;

static wake_state_t   s_wake_state = WAKE_IDLE;
static send_state_t   s_send_state = SEND_IDLE;

static uint16_t       s_read       = 0;   /* free-running consumer index   */
static uint16_t       s_end        = 0;   /* snapshot end mark for a report*/
static uint16_t       s_i          = 0;   /* per-report sample index (0..) */
static uint32_t       s_wake_ms    = 0;   /* timestamp WAKE went high      */
static bool           s_wake_armed = true;/* fire WAKE once per backlog run*/
static volatile bool  s_aa         = false; /* set by RX ISR on 0xAA       */

void FlowReport_Init(void)
{
    /* WAKE pin is configured by LEDs_Init() and idles LOW. */
    s_wake_state = WAKE_IDLE;
    s_send_state = SEND_IDLE;
    s_read       = 0;
    s_end        = 0;
    s_i          = 0;
    s_wake_armed = true;
    s_aa         = false;
}

/* Called from the UART RX path when 0xAA arrives (ISR context). */
void FlowReport_NotifyAA(void)
{
    s_aa = true;
}

/* ---- unsigned->ASCII helpers (debug text only) ---- */
#ifdef APP_DEBUG_PRINT_ENABLE
static uint8_t u16_to_buf(char *p, uint16_t v)
{
    char t[5];
    uint8_t n = 0;
    do { t[n++] = (char)('0' + (v % 10u)); v /= 10u; } while (v > 0);
    for (uint8_t k = 0; k < n; k++) p[k] = t[n - 1 - k];
    return n;
}
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
static uint8_t u8_to_hex(char *p, uint8_t v)
{
    static const char H[] = "0123456789ABCDEF";
    p[0] = H[(v >> 4) & 0x0F];
    p[1] = H[v & 0x0F];
    return 2;
}

/* "Sample-<buf>-<idx> : Pulse=.., Total=.., SampleNumber=.., Raw=XX XX .." */
static uint8_t build_debug_line(char *line, uint16_t bufNo, uint16_t idx,
                                uint16_t pulses, const uint8_t *raw)
{
    uint8_t p = 0;
    memcpy(&line[p], "Sample-", 7);            p += 7;
    p += u16_to_buf_pad(&line[p], bufNo, 4);       /* buffer slot 0..SLOTS-1 */
    line[p++] = '-';
    p += u16_to_buf_pad(&line[p], idx, 4);         /* per-report index       */
    memcpy(&line[p], " : Pulse=", 9);          p += 9;
    p += u16_to_buf(&line[p], pulses);
    memcpy(&line[p], ", Total=", 8);           p += 8;
    p += u32_to_buf(&line[p], FlowMeter_GetTotal());
    memcpy(&line[p], ", SampleNumber=", 15);   p += 15;
    p += u32_to_buf(&line[p], FlowLog_GetCaptureCount());
    memcpy(&line[p], ", Raw=", 6);             p += 6;
    for (uint8_t k = 0; k < COMPRESS_BYTES_PER_SAMPLE; k++) {
        if (k) line[p++] = ' ';
        p += u8_to_hex(&line[p], raw[k]);
    }
    line[p++] = '\r';
    line[p++] = '\n';
    line[p]   = '\0';
    return p;
}

/* "Upload-Started : Count to Send = <n>\r\n" */
static uint8_t build_header_debug_line(char *line, uint32_t count)
{
    uint8_t p = 0;
    memcpy(&line[p], "Upload-Started : Count to Send = ", 33); p += 33;
    p += u32_to_buf(&line[p], count);
    line[p++] = '\r';
    line[p++] = '\n';
    line[p]   = '\0';
    return p;
}
#endif /* APP_DEBUG_PRINT_ENABLE */

/* ---- raw byte sender (report payload) ----
 * Call only after UART_TX_Free() has been checked, so nothing drops. */
#ifdef APP_REPORT_PRINT_ENABLE
static void send_raw(const uint8_t *b, uint8_t n)
{
    for (uint8_t k = 0; k < n; k++) {
        (void)print_char((char)b[k]);
    }
}
#endif

void FlowReport_Process(void)
{
    /* backlog = samples captured but not yet sent (wrap-safe) */
    uint16_t unsent = (uint16_t)(FlowLog_GetWriteIndex() - s_read);

    /* ---------------- WAKE machine (independent) ---------------- */
    switch (s_wake_state) {

    case WAKE_IDLE:
        /* re-arm once the backlog has been drained below a batch */
        if (unsent < FLOWLOG_BATCH) {
            s_wake_armed = true;
        }
        /* fire a single WAKE pulse when a full batch has piled up */
        if (s_wake_armed && (unsent >= FLOWLOG_BATCH)) {
            PHOTON2_WAKE_ON;               /* RC4 high: wake Photon2     */
            LedFsm_NotifyDataCycle();       /* LED: start blink burst     */
            s_wake_ms    = getNowTime();
            s_wake_armed = false;
            s_wake_state = WAKE_HIGH;
        }
        break;

    case WAKE_HIGH:
        if (timeSpan(s_wake_ms) >= APP_WAKE_PULSE_MS) {
            PHOTON2_WAKE_OFF;              /* RC4 low                    */
            s_wake_state = WAKE_IDLE;
        }
        break;

    default:
        s_wake_state = WAKE_IDLE;
        break;
    }

    /* ---------------- SEND machine (0xAA triggered) ------------- */
    switch (s_send_state) {

    case SEND_IDLE:
        if (s_aa) {
            s_aa  = false;                 /* consume the request        */
            s_end = FlowLog_GetWriteIndex(); /* snapshot the end mark    */
            s_i   = 0;                      /* index restarts at 0        */
            s_send_state = SEND_HEADER;
        }
        break;

    case SEND_HEADER: {
        /* number of samples to send this upload (snapshot - consumer) */
        uint32_t count = (uint32_t)(uint16_t)(s_end - s_read);
        uint16_t need  = 0;

#ifdef APP_DEBUG_PRINT_ENABLE
        char hline[48];
        uint8_t hlen = build_header_debug_line(hline, count);
        need += hlen;
#endif
#ifdef APP_REPORT_PRINT_ENABLE
        uint8_t hdr[4];
        hdr[0] = (uint8_t)((count >> 24) & 0xFFu);   /* MSB first */
        hdr[1] = (uint8_t)((count >> 16) & 0xFFu);
        hdr[2] = (uint8_t)((count >>  8) & 0xFFu);
        hdr[3] = (uint8_t)( count        & 0xFFu);
        need += 4u;
#endif
        if (need > 0u && UART_TX_Free() < need) {
            break;                         /* retry next turn */
        }
#ifdef APP_DEBUG_PRINT_ENABLE
        print_string(hline);
#endif
#ifdef APP_REPORT_PRINT_ENABLE
        send_raw(hdr, 4);
#endif
        s_send_state = (count == 0u) ? SEND_IDLE : SEND_BODY;
        break;
    }

    case SEND_BODY: {
        if (s_read == s_end) {             /* reached the snapshot end   */
            s_send_state = SEND_IDLE;      /* a later 0xAA starts anew    */
            break;
        }

        /* fetch the stored pulse count; the stored index is ignored */
        flowlog_entry_t e;
        FlowLog_GetAt(s_read, &e);

        /* re-pack with the per-report index (restarts at 0 each upload) */
        uint8_t raw[COMPRESS_BYTES_PER_SAMPLE];
        Compress_Pack(s_i, e.pulses, raw);

        uint16_t need = 0;
#ifdef APP_DEBUG_PRINT_ENABLE
        char dline[112];
        uint16_t bufNo = (uint16_t)(s_read % FLOWLOG_SLOTS);
        uint8_t dlen = build_debug_line(dline, bufNo, s_i, e.pulses, raw);
        need += dlen;
#endif
#ifdef APP_REPORT_PRINT_ENABLE
        need += COMPRESS_BYTES_PER_SAMPLE;
#endif
        if (need > 0u && UART_TX_Free() < need) {
            break;                         /* retry next turn */
        }
#ifdef APP_DEBUG_PRINT_ENABLE
        print_string(dline);
#endif
#ifdef APP_REPORT_PRINT_ENABLE
        send_raw(raw, COMPRESS_BYTES_PER_SAMPLE);
#endif
        s_read++;   /* free-running consumer advances */
        s_i++;      /* per-report index advances      */
        break;
    }

    default:
        s_send_state = SEND_IDLE;
        break;
    }
}
