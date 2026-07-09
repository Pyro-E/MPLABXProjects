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
#include "Packet.h"
#include "MValve_OP3.h"
#include "Flow_Control.h"

/* ============================================================
 *  FlowReport - two INDEPENDENT state machines.
 *
 *  (1) WAKE machine
 *      When the number of not-yet-sent samples reaches FLOWLOG_BATCH (or a
 *      report is otherwise requested), it raises a "report due" flag that
 *      main() turns into a WAKE-line assertion. The WAKE pin itself is owned
 *      by main (comms-ready model): high on report-due / 0xF0 / any RX byte,
 *      low CLOSE_WAKE_AFTER_UART_MS after the last UART activity. This machine
 *      only decides WHEN a report is due and tracks "busy" so we do not sleep
 *      while a request/response is expected.
 *
 *  (2) SEND machine  (triggered ONLY by REQ_DATA, via FlowReport_NotifyAA)
 *      On the request the current write position is snapshotted as the end
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
    WAKE_HIGH,       /* WAKE held high, waiting to lower it */
    WAKE_WAIT        /* WAKE lowered; staying awake for Photon2's 0xAA */
} wake_state_t;

typedef enum {
    SEND_IDLE = 0,   /* waiting for a request */
    SEND_PKT_HDR,    /* sending AA 55 func len(2) and seeding the CRC */
    SEND_HEADER,     /* sending the 4-byte COUNT */
    SEND_BODY,       /* sending samples up to the end mark */
    SEND_PKT_CRC     /* sending the trailing crc16(2) */
} send_state_t;

static wake_state_t   s_wake_state = WAKE_IDLE;
static volatile bool  s_wake_due   = false;  /* report due -> main raises WAKE */
static send_state_t   s_send_state = SEND_IDLE;

static uint16_t       s_read       = 0;   /* circular consumer index 0..SLOTS-1 */
static uint16_t       s_end        = 0;   /* snapshot end mark for a report*/
static uint16_t       s_i          = 0;   /* per-report sample index (0..) */
static uint16_t       s_rsp_crc    = 0;   /* streaming CRC for RSP_DATA    */
static uint32_t       s_wake_ms    = 0;   /* timestamp WAKE went high      */
static volatile bool  s_report_req = false;/* explicit report request (0xF0)*/
static volatile bool  s_aa         = false; /* set by RX ISR on 0xAA       */

void FlowReport_Init(void)
{
    /* WAKE pin is configured by LEDs_Init() and idles LOW. */
    s_wake_state = WAKE_IDLE;
    s_send_state = SEND_IDLE;
    s_read       = 0;
    s_end        = 0;
    s_i          = 0;
    s_aa         = false;
}

/* Called from the UART RX path when 0xAA arrives (ISR context). */
void FlowReport_NotifyAA(void)
{
    s_aa = true;
}

/* Photon2 wants to talk (0xF0 while awake, or a UART wake from sleep):
 * ask the WAKE machine to raise WAKE even if the batch is not full. */
void FlowReport_RequestReport(void)
{
    s_report_req = true;
}

/* True while either state machine is active, or a request/0xAA is pending. */
bool FlowReport_IsBusy(void)
{
    return (s_wake_state != WAKE_IDLE) ||
           (s_send_state != SEND_IDLE) ||
           s_aa || s_report_req;
}

/* RX gate: true only while actually streaming a response. WAKE-wait and a
 * pending report do NOT count, so the REQ_DATA we raised WAKE to solicit is
 * accepted rather than dropped. */
bool FlowReport_IsSending(void)
{
    return (s_send_state != SEND_IDLE);
}

/* True once when a report becomes due; main turns this into a WAKE raise.
 * Reading it consumes the flag. */
bool FlowReport_WakeDuePending(void)
{
    if (s_wake_due) {
        s_wake_due = false;        return true;
    }
    return false;
}

/* Drop a latched "report due" without acting on it. Used when the initial
 * power-hold ends: report periods that completed during the hold should not
 * force an immediate re-power the instant we cut power - the accumulated data
 * waits in the ring buffer and is collected at the next normal report period. */
void FlowReport_ClearWakeDue(void)
{
    s_wake_due = false;
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

/* ============================================================
 *  Small response packets (PARAM / VALVE / ACK / NAK).
 *  These are short, so we build the whole frame in a tiny stack
 *  buffer and push it at once. RSP_DATA is NOT built this way:
 *  it is streamed sample-by-sample (see the SEND machine) so the
 *  large payload never sits in SRAM.
 * ============================================================ */
#define PKT_SMALL_MAX_DATA  8u

static void send_small_packet(uint8_t func, const uint8_t *data, uint16_t len)
{
    uint8_t frame[PKT_OVERHEAD_BYTES + PKT_SMALL_MAX_DATA];
    uint16_t i = 0;
    uint16_t crc;

    if (len > PKT_SMALL_MAX_DATA) {
        return;                       /* guard: only for small payloads */
    }

    frame[i++] = PKT_MARKER0;
    frame[i++] = PKT_MARKER1;
    frame[i++] = func;
    frame[i++] = (uint8_t)((len >> 8) & 0xFFu);
    frame[i++] = (uint8_t)(len & 0xFFu);
    for (uint16_t k = 0; k < len; k++) {
        frame[i++] = data[k];
    }
    /* CRC over func + len(2) + data (markers excluded) */
    crc = Packet_CRC16(&frame[2], (uint16_t)(3u + len));
    frame[i++] = (uint8_t)((crc >> 8) & 0xFFu);   /* big-endian */
    frame[i++] = (uint8_t)(crc & 0xFFu);

    /* push whole frame if there is room; else drop (Photon will retry) */
    if (UART_TX_Free() >= i) {
        for (uint16_t k = 0; k < i; k++) {
            (void)print_char((char)frame[k]);
        }
    }
}

void FlowReport_SendParam(void)
{
    leak_param_t pr;
    uint8_t d[8];
    FlowControl_GetParams(&pr);
    d[0] = (uint8_t)(pr.leak1_counts   >> 8);  d[1] = (uint8_t)pr.leak1_counts;
    d[2] = (uint8_t)(pr.leak1_window_s >> 8);  d[3] = (uint8_t)pr.leak1_window_s;
    d[4] = (uint8_t)(pr.leak2_counts   >> 8);  d[5] = (uint8_t)pr.leak2_counts;
    d[6] = (uint8_t)(pr.leak2_window_s >> 8);  d[7] = (uint8_t)pr.leak2_window_s;
    send_small_packet(PKT_RSP_PARAM, d, 8u);
}

void FlowReport_SendValve(void)
{
    uint8_t d[8];
    uint32_t tc = FlowControl_GetTempLockCount();
    d[0] = MValve_OP3_GetPwrPin();
    d[1] = MValve_OP3_GetCtrlPin();
    d[2] = (uint8_t)MValve_OP3_GetMotion();
    d[3] = FlowControl_GetLockFlags();
    d[4] = (uint8_t)((tc >> 24) & 0xFFu);
    d[5] = (uint8_t)((tc >> 16) & 0xFFu);
    d[6] = (uint8_t)((tc >>  8) & 0xFFu);
    d[7] = (uint8_t)( tc        & 0xFFu);
    send_small_packet(PKT_RSP_VALVE, d, 8u);
}

void FlowReport_SendAck(uint8_t echoed_func)
{
    uint8_t d = echoed_func;
    send_small_packet(PKT_RSP_ACK, &d, 1u);
}

/* RSP_POWER_STATE: one byte, 0 = INITIAL power-hold, 1 = NORMAL. */
void FlowReport_SendPowerState(uint8_t state)
{
    uint8_t d = state;
    send_small_packet(PKT_RSP_POWER_STATE, &d, 1u);
}

void FlowReport_SendNak(uint8_t reason)
{
    uint8_t d = reason;
    send_small_packet(PKT_RSP_NAK, &d, 1u);
}
#endif

void FlowReport_Process(void)
{
    /* WAKE fires once each time a fresh group of FLOWLOG_BATCH captures
     * completes (FlowLog counts 0..BATCH-1 and signals on each full group),
     * OR when Photon2 explicitly asks (0xF0 / UART wake). This retries every
     * batch even if a previous WAKE went unanswered. The amount actually
     * uploaded later is governed by the read/write indices, so it may be
     * more than one batch (piled up) or fewer (ring wrapped past read).
     *
     * BatchReady() is read only in WAKE_IDLE: if a batch completes while a
     * previous WAKE cycle is still busy, the signal stays latched in FlowLog
     * until we return to WAKE_IDLE (it is not consumed/lost). */

    /* ---------------- WAKE machine (independent) ----------------
     * The WAKE PIN is now owned by main (comms-ready model): this machine
     * only DECIDES when a report is due and raises s_wake_due, which main
     * polls via FlowReport_WakeDuePending() and turns into a Wake_Raise().
     * It still tracks "busy" so main will not sleep while we expect a
     * request/response. */
    switch (s_wake_state) {

    case WAKE_IDLE:
        if (FlowLog_BatchReady() || s_report_req) {
            s_report_req = false;
            s_wake_due   = true;           /* tell main to raise WAKE     */
            s_wake_ms    = getNowTime();
            s_wake_state = WAKE_WAIT;
#ifdef APP_DEBUG_AUTO_DATA_REPORT_WITHOUT_REQ
            s_aa = true;                   /* DEBUG: self-trigger a report
                                            * with no REQ_DATA packet      */
#endif
        }
        break;

    case WAKE_HIGH:
        /* unused in the comms-ready model; fold straight into WAIT */
        s_wake_state = WAKE_WAIT;
        break;

    case WAKE_WAIT:
        /* Stay "busy" (so main() will not sleep) while waiting for Photon2
         * to send its request. If the SEND machine has started, the wait is
         * over. If Photon2 never answers, give up after the timeout. */
        if (s_aa || (s_send_state != SEND_IDLE)) {
            s_wake_state = WAKE_IDLE;
        } else if (timeSpan(s_wake_ms) >= WAIT_PHOTON_UART_RESPONSE_MS) {
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
            LedFsm_NotifyDataCycle();       /* fast blink: request received */
            s_send_state = SEND_PKT_HDR;
        }
        break;

    case SEND_PKT_HDR: {
        /* RSP_DATA frame: AA 55 | 0x81 | len(2,BE) | COUNT(4)+samples | crc16
         * len = 4 (COUNT) + count*COMPRESS_BYTES_PER_SAMPLE. The payload is
         * streamed; only the small header is built here. */
        uint32_t count = (uint32_t)(uint16_t)((s_end - s_read
                                    + FLOWLOG_SLOTS) % FLOWLOG_SLOTS);
        uint16_t plen  = (uint16_t)(4u + count * COMPRESS_BYTES_PER_SAMPLE);
        uint8_t  hdr[5];
        hdr[0] = PKT_MARKER0;
        hdr[1] = PKT_MARKER1;
        hdr[2] = (uint8_t)PKT_RSP_DATA;
        hdr[3] = (uint8_t)((plen >> 8) & 0xFFu);
        hdr[4] = (uint8_t)( plen       & 0xFFu);

        if (UART_TX_Free() < 5u) {
            break;                          /* retry next turn */
        }
        /* seed CRC over func + len(2); markers are excluded */
        s_rsp_crc = Packet_CRC16_Init();
        s_rsp_crc = Packet_CRC16_Update(s_rsp_crc, hdr[2]);
        s_rsp_crc = Packet_CRC16_Update(s_rsp_crc, hdr[3]);
        s_rsp_crc = Packet_CRC16_Update(s_rsp_crc, hdr[4]);
        send_raw(hdr, 5u);
        s_send_state = SEND_HEADER;
        break;
    }

    case SEND_HEADER: {
        /* COUNT(4, big-endian) into the payload, accumulating CRC. */
        uint32_t count = (uint32_t)(uint16_t)((s_end - s_read
                                    + FLOWLOG_SLOTS) % FLOWLOG_SLOTS);
        uint8_t hdr[4];
        hdr[0] = (uint8_t)((count >> 24) & 0xFFu);   /* MSB first */
        hdr[1] = (uint8_t)((count >> 16) & 0xFFu);
        hdr[2] = (uint8_t)((count >>  8) & 0xFFu);
        hdr[3] = (uint8_t)( count        & 0xFFu);

#ifdef APP_DEBUG_PRINT_ENABLE
        {   char hline[48];
            uint8_t hlen = build_header_debug_line(hline, count);
            if (UART_TX_Free() < (uint16_t)(hlen + 4u)) break;
            print_string(hline);
        }
#else
        if (UART_TX_Free() < 4u) break;
#endif
        for (uint8_t k = 0; k < 4u; k++) {
            s_rsp_crc = Packet_CRC16_Update(s_rsp_crc, hdr[k]);
        }
        send_raw(hdr, 4u);
        s_send_state = (count == 0u) ? SEND_PKT_CRC : SEND_BODY;
        break;
    }

    case SEND_BODY: {
        if (s_read == s_end) {             /* reached the snapshot end   */
            s_send_state = SEND_PKT_CRC;   /* append CRC, then idle      */
            break;
        }

        flowlog_entry_t e;
        FlowLog_GetAt(s_read, &e);

        uint8_t raw[COMPRESS_BYTES_PER_SAMPLE];
        Compress_Pack(s_i, e.pulses, raw);

#ifdef APP_DEBUG_PRINT_ENABLE
        {   char dline[112];
            uint16_t bufNo = (uint16_t)(s_read % FLOWLOG_SLOTS);
            uint8_t dlen = build_debug_line(dline, bufNo, s_i, e.pulses, raw);
            if (UART_TX_Free() < (uint16_t)(dlen + COMPRESS_BYTES_PER_SAMPLE)) break;
            print_string(dline);
        }
#else
        if (UART_TX_Free() < COMPRESS_BYTES_PER_SAMPLE) break;
#endif
        for (uint8_t k = 0; k < COMPRESS_BYTES_PER_SAMPLE; k++) {
            s_rsp_crc = Packet_CRC16_Update(s_rsp_crc, raw[k]);
        }
        send_raw(raw, COMPRESS_BYTES_PER_SAMPLE);
        s_read = (uint16_t)((s_read + 1u) % FLOWLOG_SLOTS);  /* circular */
        s_i++;
        break;
    }

    case SEND_PKT_CRC: {
        uint8_t crc[2];
        crc[0] = (uint8_t)((s_rsp_crc >> 8) & 0xFFu);   /* big-endian */
        crc[1] = (uint8_t)( s_rsp_crc       & 0xFFu);
        if (UART_TX_Free() < 2u) break;
        send_raw(crc, 2u);
        s_send_state = SEND_IDLE;          /* a later request starts anew */
        break;
    }

    default:
        s_send_state = SEND_IDLE;
        break;
    }
}
