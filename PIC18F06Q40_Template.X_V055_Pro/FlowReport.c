#define FLOWREPORT_C

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "App_Config.h"
#include "App_Config_Photon.h"
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

/* ---- extended RSP_DATA header (computed once per request in SEND_IDLE) ----
 * Payload layout (big-endian):
 *   [1] u32 impulse_since_report  - true pulses since last report (mark diff)
 *   [2] u32 captures_since_report - captures since last report (counter diff)
 *   [3] u32 impulse_of_span       - true pulses of the sent sample span
 *   [4] u16 overflow_count        - #samples in the span clamped at 14 bits
 *   [5] u32 COUNT                 - #samples that follow
 * then COUNT samples of COMPRESS_BYTES_PER_SAMPLE bytes each. */
static uint32_t s_f_imp_report = 0;   /* field 1 */
static uint32_t s_f_cap_report = 0;   /* field 2 */
static uint32_t s_f_imp_span   = 0;   /* field 3 */
static uint16_t s_f_ovf_span   = 0;   /* field 4 */
static uint16_t s_count        = 0;   /* field 5: samples in this report */

/* Marks carried between reports (a "report" = a REQ_DATA we answer). */
static uint32_t s_impulse_mark  = 0;  /* total at last report (for field 1) */
static uint32_t s_caps_mark     = 0;  /* captures at last report (for field 2) */
static uint32_t s_total_at_read = 0;  /* running total at the current s_read */

#define XHDR_BYTES  18u   /* 4+4+4+2+4 fixed header ahead of the samples */

void FlowReport_Init(void)
{
    /* WAKE pin is configured by LEDs_Init() and idles LOW. */
    s_wake_state = WAKE_IDLE;
    s_send_state = SEND_IDLE;
    s_read       = 0;
    s_end        = 0;
    s_i          = 0;
    s_aa         = false;

    /* Extended-report marks start from the meter/log baseline. */
    s_impulse_mark  = FlowMeter_GetTotal();
    s_caps_mark     = FlowLog_GetCaptureCount();
    s_total_at_read = s_impulse_mark;
    s_f_imp_report  = 0;
    s_f_cap_report  = 0;
    s_f_imp_span    = 0;
    s_f_ovf_span    = 0;
    s_count         = 0;
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
#define PKT_SMALL_MAX_DATA  16u   /* raised from 8 to fit 13B RSP_PHOTON_CFG */

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
    uint8_t d[5];
    d[0] = MValve_OP3_GetPwrPin();
    d[1] = MValve_OP3_GetCtrlPin();
    d[2] = (uint8_t)MValve_OP3_GetMotion();
    d[3] = FlowControl_GetLockFlags();          /* locks currently ACTIVE */
    d[4] = FlowControl_GetSinceReportFlags();   /* LEAK1/LEAK2 tripped since last report */
    send_small_packet(PKT_RSP_VALVE, d, 5u);
    /* This RSP_VALVE IS the report -- the Photon owns the lifetime LEAK1/LEAK2
     * tally now (leakingEventCount/overflowEventCount), so clear our transient
     * since-report flags the moment they've been sent. */
    FlowControl_ClearSinceReport();
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

/* RSP_PHOTON_CFG : 14-byte config block the Photon reads at boot.
 * If PIC_PROVIDES_PHOTON_CFG is undefined we still answer, with provided=0, so
 * the Photon uses its own defaults (and never hangs waiting). */
void FlowReport_SendPhotonCfg(void)
{
    uint8_t d[14];
    uint16_t i = 0;

#ifdef PIC_PROVIDES_PHOTON_CFG
    d[i++] = 1u;                                   /* provided = 1 */
#else
    d[i++] = 0u;                                   /* provided = 0 -> use your own */
#endif
    d[i++] = (uint8_t)PCFG_VERSION;

    /* A. timing (big-endian) */
    d[i++] = (uint8_t)((PCFG_CAPTURE_INTERVAL_MS >> 24) & 0xFFu);
    d[i++] = (uint8_t)((PCFG_CAPTURE_INTERVAL_MS >> 16) & 0xFFu);
    d[i++] = (uint8_t)((PCFG_CAPTURE_INTERVAL_MS >>  8) & 0xFFu);
    d[i++] = (uint8_t)( PCFG_CAPTURE_INTERVAL_MS        & 0xFFu);
    d[i++] = (uint8_t)((PCFG_SAMPLES_PER_REPORT >> 8) & 0xFFu);
    d[i++] = (uint8_t)( PCFG_SAMPLES_PER_REPORT       & 0xFFu);
    d[i++] = (uint8_t)PCFG_REPORT_INTERVAL_HR;

    /* B. debug toggles */
    d[i++] = (uint8_t)PCFG_FAST_BENCH;
    d[i++] = (uint8_t)PCFG_DEBUG_DATASERIES;
    d[i++] = (uint8_t)PCFG_MISSED_FILL_MODE;
    d[i++] = (uint8_t)((PCFG_SERIAL_DELAY_MS >> 8) & 0xFFu);
    d[i++] = (uint8_t)( PCFG_SERIAL_DELAY_MS       & 0xFFu);

    send_small_packet(PKT_RSP_PHOTON_CFG, d, i);   /* i == 14 */
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

            /* ---- ring safety policy ----
             * Un-sent backlog = ring distance (end - read). If it is within the
             * safe fill (SLOTS - margin) send it all. If it has grown past that
             * (a report was skipped, ring nearly wrapped), do NOT try to drain
             * the whole ring: jump the consumer to the most recent report-due
             * boundary and send only that last period. Older samples are dropped
             * from the SERIES, but their pulses are still carried in field 1
             * (impulse_since_report), so no flow total is lost. */
            uint16_t backlog = (uint16_t)((s_end - s_read + FLOWLOG_SLOTS)
                                          % FLOWLOG_SLOTS);
            uint32_t cur_total = FlowMeter_GetTotal();
            uint32_t cur_caps  = FlowLog_GetCaptureCount();

            if (backlog > (uint16_t)(FLOWLOG_SLOTS - APP_FLOW_RING_MARGIN)
                && FlowLog_DueValid()) {
                s_read          = FlowLog_GetWriteAtDue();  /* last period start */
                s_total_at_read = FlowLog_GetTotalAtDue();  /* total there       */
            }

            /* ---- extended header fields ---- */
            s_f_imp_report = cur_total - s_impulse_mark;   /* 1: since last report */
            s_f_cap_report = cur_caps  - s_caps_mark;      /* 2: captures since    */
            s_f_imp_span   = cur_total - s_total_at_read;  /* 3: impulse of span   */
            s_f_ovf_span   = FlowLog_CountOverflows(s_read, s_end); /* 4          */
            s_count        = (uint16_t)((s_end - s_read + FLOWLOG_SLOTS)
                                        % FLOWLOG_SLOTS);  /* 5: samples to send   */

            /* advance the marks for the next report */
            s_impulse_mark  = cur_total;
            s_caps_mark     = cur_caps;
            s_total_at_read = cur_total;   /* after this report s_read -> s_end   */

#ifdef APP_DEBUG_PKT_LOG
            /* One-line summary of the report we are about to stream. */
            print_string("TX DATA imp=");   print_uint(s_f_imp_report);
            print_string(" cap=");          print_uint(s_f_cap_report);
            print_string(" span=");         print_uint(s_f_imp_span);
            print_string(" ovf=");          print_uint((uint32_t)s_f_ovf_span);
            print_string(" n=");            print_uint((uint32_t)s_count);
            print_char('\n');
#endif

            s_i   = 0;                      /* index restarts at 0        */
            LedFsm_NotifyDataCycle();       /* fast blink: request received */
            s_send_state = SEND_PKT_HDR;
        }
        break;

    case SEND_PKT_HDR: {
        /* RSP_DATA frame: AA 55 | 0x81 | len(2,BE) | XHDR(18)+samples | crc16
         * len = 18 (extended header) + count*COMPRESS_BYTES_PER_SAMPLE. */
        uint32_t count = s_count;
        uint16_t plen  = (uint16_t)(XHDR_BYTES
                                    + count * COMPRESS_BYTES_PER_SAMPLE);
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
        /* Extended 18-byte header, all big-endian, accumulating CRC:
         *   f1 impulse_since_report (4), f2 captures_since_report (4),
         *   f3 impulse_of_span (4), f4 overflow_count (2), f5 COUNT (4). */
        uint8_t hdr[XHDR_BYTES];
        uint8_t p = 0;
        hdr[p++] = (uint8_t)((s_f_imp_report >> 24) & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_imp_report >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_imp_report >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( s_f_imp_report        & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_cap_report >> 24) & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_cap_report >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_cap_report >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( s_f_cap_report        & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_imp_span   >> 24) & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_imp_span   >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_imp_span   >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( s_f_imp_span          & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_ovf_span   >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( s_f_ovf_span          & 0xFFu);
        hdr[p++] = (uint8_t)(((uint32_t)s_count >> 24) & 0xFFu);
        hdr[p++] = (uint8_t)(((uint32_t)s_count >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)(((uint32_t)s_count >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( (uint32_t)s_count        & 0xFFu);

        if (UART_TX_Free() < (uint16_t)XHDR_BYTES) break;
        for (uint8_t k = 0; k < XHDR_BYTES; k++) {
            s_rsp_crc = Packet_CRC16_Update(s_rsp_crc, hdr[k]);
        }
        send_raw(hdr, XHDR_BYTES);
        s_send_state = (s_count == 0u) ? SEND_PKT_CRC : SEND_BODY;
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

#if defined(APP_DEBUG_DATASERIES)
        {   /* per-sample dump: index, group, pulses (bench only, voluminous) */
            uint16_t bufNo = (uint16_t)(s_read % FLOWLOG_SLOTS);
            if (UART_TX_Free() < (uint16_t)(40u + COMPRESS_BYTES_PER_SAMPLE)) break;
            print_string("  ["); print_uint((uint32_t)s_i);
            print_string("] buf="); print_uint((uint32_t)bufNo);
            print_string(" grp="); print_uint((uint32_t)e.grp);
            print_string(" pul="); print_uint((uint32_t)e.pulses);
            print_char('\n');
        }
#elif defined(APP_DEBUG_PRINT_ENABLE)
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
