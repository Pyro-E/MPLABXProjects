#define FLOWREPORT_C

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "App_Config.h"
#include "App_Config_Photon.h"
#include "FlowReport.h"
#include "Loco.h"        /* Loco_GetAnchor/GetReportInterval for RSP_PARAM extension */
#include "FlowLog.h"
#include "FlowMeter.h"
#include "Dev_Debug_Uart.h"  /* logs go to the SEPARATE debug UART (RA4) */
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
static uint16_t       s_tx_read    = 0;   /* TEMP send index; read advances     */
                                          /* only on ACK (consume-on-ACK)       */
static bool           s_pending_commit = false; /* a batch was sent, not yet    */
                                          /* ACKed -> retransmit until DATA_RECEIVED */
static uint16_t       s_end        = 0;   /* snapshot end mark for a report*/
static uint32_t       s_commit_total = 0; /* FlowMeter total at s_end (send  */
static uint32_t       s_commit_caps  = 0; /* start) - used at ACK commit so  */
                                          /* marks match s_end, not ACK time */
static uint16_t       s_i          = 0;   /* per-report sample index (0..) */
static uint16_t       s_rsp_crc    = 0;   /* streaming CRC for RSP_DATA    */
static uint32_t       s_wake_ms    = 0;   /* timestamp WAKE went high      */
static volatile bool  s_report_req = false;/* explicit report request (0xF0)*/
static volatile bool  s_aa         = false; /* set by RX ISR on 0xAA       */
/* Initial-boot mode: while the PIC is in its cold-boot hold, every REQ_DATA is
 * answered with ZERO samples and the captured buffer is wiped (even on Photon
 * retransmits). main.c sets this from in_initial_hold(). */
static bool           s_initial_boot = false;

/* ---- extended RSP_DATA header (computed once per request in SEND_IDLE) ----
 * Payload layout (big-endian):
 *   [1] u32 impulse_since_report  - true pulses since last report (mark diff)
 *   [2] u32 captures_since_report - captures since last report (counter diff)
 *   [3] u32 impulse_of_span       - true pulses of the sent sample span
 *   [4] u16 overflow_count        - #samples in the span clamped at 14 bits
 *   [5] u32 COUNT                 - #samples that follow
 * then COUNT samples of COMPRESS_BYTES_PER_SAMPLE bytes each. */
static uint32_t s_f_cap_report = 0;   /* total_captures  : captures this period    */
static uint32_t s_f_imp_span   = 0;   /* total_impulses  : true pulses of the span */
static uint16_t s_f_ovf_span   = 0;   /* overflow_ffff   : #samples clamped        */
static uint16_t s_count        = 0;   /* sample_count    : samples that follow     */
/* NEW header fields (set by main.c via FlowReport_SetReportMeta before a report) */
static uint32_t s_f_start_time = 0;   /* start_time epoch of this report's data     */
static uint8_t  s_f_start_valid = 0;  /* 1 = start_time is a real cloud-derived time */
static uint32_t s_f_interval_ms = 0;  /* sample_interval the PIC intends (LOCO-adj)  */

/* V022 / Appendix H.7.2: small-packet TX drop accounting.
 * send_small_packet() used to discard a frame silently when the packet-UART TX
 * buffer was still busy with the previous reply. On the Photon that is
 * indistinguishable from a timeout or a CRC error ("CFG: no valid reply"), and
 * it is the suspected cause of the n=36/n=1/n=0 report split of H.7.1. The
 * counter is reported once per session on the [PWR ] Photon OFF line and MUST
 * read 0 in a healthy run. */
static uint16_t s_txdrops = 0;

/* Marks carried between reports (a "report" = a REQ_DATA we answer). */
static uint32_t s_impulse_mark  = 0;  /* total at last report (for field 1) */
static uint32_t s_caps_mark     = 0;  /* captures at last report (for field 2) */
static uint32_t s_total_at_read = 0;  /* running total at the current s_read */

/* ---- V023 / fix 4: FROZEN SESSION CUTOFF ---------------------------------
 * A report's data boundary is decided ONCE, when main.c powers the Photon on,
 * and never moves again for the rest of that session. V022 re-read the live
 * write index, capture count and meter total on EVERY REQ_DATA, so a batch
 * grew while the Photon was retrying: the bench log shows one un-ACKed report
 * answered four times as n=39 -> n=40 -> n=40 -> n=41, and the header total
 * drifting (span=2209531 -> 2209611) even between two answers that carried
 * the same 32 samples. Captures that happen while the Photon is awake belong
 * to the NEXT report, not this one. */
static bool     s_cut_valid  = false; /* a cutoff is frozen for this session  */
static uint32_t s_cut_ms     = 0;     /* PIC-local ms when the cutoff was frozen */
static uint16_t s_cut_end    = 0;     /* ring write index at cutoff           */
static uint32_t s_cut_caps   = 0;     /* completed captures at cutoff         */
static uint32_t s_cut_total  = 0;     /* COMPLETED-capture pulse total there  */

/* ---- V024 / protocol v2: THE AMOUNT IS A POSITION, NOT A DELTA -----------
 * The PIC's contract with the Photon is the AMOUNT of water and nothing else
 * (hourly.h: "The PIC guarantees the AMOUNT of water (pulses) and nothing
 * else"). v1 delivered that amount as a delta, which cannot be checked: a lost,
 * duplicated or mis-based delta still looks like a plausible volume.
 *
 * v2 delivers the two ENDPOINTS on the PIC's lifetime pulse axis instead. The
 * delta becomes derived, so it can never contradict the samples, and the Photon
 * gets an auditable statement on every frame:
 *
 *     this report begins at begin_pulses and ends at end_pulses
 *
 * From that alone the Photon can prove continuity (begin == the previous
 * report's end), recognise a retransmission exactly (identical endpoints), and
 * measure a gap in pulses rather than merely flag one. This replaces the V023
 * batch_seq: a synthetic counter was only ever a stand-in for the position the
 * protocol should have carried in the first place, and it could collide after a
 * PIC reset, which endpoints cannot. */
static uint32_t s_span_begin_pul = 0;  /* cumulative pulses at batch start   */
static uint32_t s_span_end_pul   = 0;  /* cumulative pulses at batch end     */
static uint32_t s_span_begin_cap = 0;  /* cumulative captures at batch start */
static uint32_t s_span_end_cap   = 0;  /* cumulative captures at batch end   */

/* Boot generation. The pulse axis restarts at 0 on every PIC restart (there is
 * no NVM register - see the note in FlowReport_InitBootId), so the Photon must
 * be able to tell "the axis moved backwards because it was reset" from "the
 * axis moved backwards because something is wrong". A value that changes on
 * every restart is enough for that, and __persistent survives a reset while
 * still coming up different after a true power-up. */
#ifndef APP_PERSISTENT
  #ifdef __XC8
    #define APP_PERSISTENT __persistent
  #else
    #define APP_PERSISTENT           /* host syntax checks / models */
  #endif
#endif
static APP_PERSISTENT uint16_t s_boot_id;
static APP_PERSISTENT uint16_t s_boot_id_chk;
#define BOOT_ID_XOR  0xA5C3u

/* Make this run distinguishable from the previous one.
 *
 * On a RESET (watchdog, MCLR, brownout) __persistent RAM keeps its contents, the
 * checksum matches, and the counter simply advances. On a true POWER-UP the RAM
 * comes up arbitrary, the checksum almost certainly fails, and we restart from a
 * known point. Either way the value the Photon sees CHANGES, which is the only
 * property the audit needs: it tells the Photon that the pulse axis it was
 * following has been replaced by a new one, so a backwards jump in the totals is
 * a restart rather than corruption. */
static void FlowReport_InitBootId(void)
{
    if ((uint16_t)(s_boot_id ^ BOOT_ID_XOR) != s_boot_id_chk) {
        s_boot_id = 0u;                        /* cold power-up: unknown RAM */
    }
    s_boot_id++;
    if (s_boot_id == 0u) s_boot_id = 1u;       /* 0 reserved for "unknown"   */
    s_boot_id_chk = (uint16_t)(s_boot_id ^ BOOT_ID_XOR);
}

/* ---- RSP_DATA header size -------------------------------------------------
 * See PCFG_PROTO_V2 in App_Config_Photon.h for the compatibility rules. */
#if PCFG_PROTO_V2
  #define XHDR_BYTES  31u /* begin_pul(4)+end_pul(4)+begin_cap(4)+end_cap(4)
                           * +boot_id(2)+sample_count(2)+overflow(2)
                           * +interval(4)+start_time(4)+start_valid(1) = 31 */
#else
  #define XHDR_BYTES  23u /* v1: start(4)+valid(1)+count(4)+caps(4)+imp(4)
                           * +ovf(2)+interval(4) = 23, big-endian */
#endif

void FlowReport_Init(void)
{
    /* V024: establish this run's boot generation FIRST, so anything that reports
     * before the first RSP_DATA already carries the right value.
     *
     * NOTE FOR THE PROJECT OWNER: this only IDENTIFIES a restart, it does not
     * survive one. The PIC has no NVM register (grep: the only NVM reference in
     * this firmware is Loco's, and even that is unused - "No NVM: k always
     * starts at the ideal"), so FlowMeter's lifetime pulse total returns to 0 on
     * every reset and any captured-but-unreported water in the ring is gone with
     * it. For a device whose contract is the AMOUNT of water, that is the one
     * remaining hole this firmware cannot close on its own. boot_id at least
     * makes the hole VISIBLE to the Photon instead of silent. See the decision
     * item in the V024 guidance. */
    FlowReport_InitBootId();

    /* WAKE pin is configured by LEDs_Init() and idles LOW. */
    s_wake_state = WAKE_IDLE;
    s_send_state = SEND_IDLE;
    s_read       = 0;
    s_tx_read    = 0;
    s_pending_commit = false;
    s_end        = 0;
    s_commit_total = 0;
    s_commit_caps  = 0;
    s_i          = 0;
    s_aa         = false;

    s_cut_valid  = false;      /* V023: no session cutoff frozen yet */
    s_cut_end    = 0;
    s_cut_caps   = 0;
    s_cut_total  = 0;
    s_span_begin_pul = 0;
    s_span_end_pul   = 0;
    s_span_begin_cap = 0;
    s_span_end_cap   = 0;

    /* Extended-report marks start from the meter/log baseline. */
    s_impulse_mark  = FlowMeter_GetTotal();
    s_caps_mark     = FlowLog_GetCaptureCount();
    s_total_at_read = s_impulse_mark;
    s_f_cap_report  = 0;
    s_f_imp_span    = 0;
    s_f_ovf_span    = 0;
    s_count         = 0;
}

/* ---- FIFO read-side accessors (the flow record is a single ring FIFO;
 * FlowLog is the PUSH side (write), FlowReport is the POP side (read).
 * These let the PUSH side enforce overrun on every capture. ---- */

/* Committed (ACKed) read index = oldest sample still owned by the consumer. */
uint16_t FlowReport_GetReadIndex(void)
{
    return s_read;
}

/* Unconsumed backlog = ring distance write-read. */
uint16_t FlowReport_GetUsed(void)
{
    uint16_t w = FlowLog_GetWriteIndex();
    return (uint16_t)((w - s_read + FLOWLOG_SLOTS) % FLOWLOG_SLOTS);
}

/* Drop the single oldest sample (advance read past it). Called by the PUSH
 * side when the ring is about to overrun. The dropped sample's pulses are
 * folded into s_total_at_read so the span total (field 3) stays exact; the
 * authoritative "since last report" total (field 1) is independent of the
 * ring and is never affected. The time-DISTRIBUTION of that oldest sample is
 * the only thing lost - by design. */
void FlowReport_DropOldest(void)
{
    if (s_read == FlowLog_GetWriteIndex()) {
        return;                         /* empty - nothing to drop */
    }
    flowlog_entry_t e;
    FlowLog_GetAt(s_read, &e);          /* pulses of the sample we discard */
    s_total_at_read += e.pulses;        /* keep field-3 span total exact   */
    s_read = (uint16_t)((s_read + 1u) % FLOWLOG_SLOTS);

    /* V023 / fix 7: a pending batch starts at s_read, so dropping its oldest
     * sample means the batch we told the Photon about no longer exists in the
     * ring. Retransmitting it is then impossible and committing it on a late
     * ACK would move s_read BACKWARDS onto samples we just discarded. Abandon
     * the pending batch instead: the next REQ_DATA builds a fresh snapshot
     * from the new s_read, and the dropped pulses are still carried in
     * total_impulses (which is measured from s_impulse_mark, untouched here),
     * so no volume is lost - only the discarded sample's time distribution.
     * This is the ring-pressure policy: PROTECT THE NEWEST DATA. It can only
     * be reached when the consumer has been stalled for ~1000 captures. */
    if (s_pending_commit) {
        s_pending_commit = false;
#ifdef APP_DEBUG_EVENT_LOG
        DBG_STR("[RPT ] pending batch ["); DBG_U32(s_span_begin_pul);
        DBG_STR("..");                      DBG_U32(s_span_end_pul);
        DBG_STR("] abandoned: ring pressure dropped its oldest sample"); DBG_NL();
#endif
    }
}

/* V023 / fix 4: freeze THIS session's data boundary.
 *
 * main.c calls this immediately before it powers the Photon on, so the cutoff
 * is the moment the session began - not the moment some REQ_DATA happened to
 * arrive. Whether the Photon takes 10 s or 40 s to boot and ask, the answer
 * covers exactly the same captures.
 *
 * A batch that is still waiting for its ACK is NOT disturbed: it keeps its own
 * frozen end (s_end) and is retransmitted unchanged. The new cutoff only takes
 * effect for the batch built after that one is committed. */
void FlowReport_FreezeSessionCutoff(void)
{
    flowlog_snapshot_t snap;
    FlowLog_GetSnapshot(&snap);

    s_cut_end   = snap.write;
    s_cut_caps  = snap.captures;
    s_cut_total = snap.completed_total;   /* completed boundary, NOT live meter */
    s_cut_ms    = getNowTime();           /* V024b: when this boundary was taken */
    s_cut_valid = true;

#ifdef APP_DEBUG_EVENT_LOG
    DBG_STR("[RPT ] cutoff frozen end="); DBG_U32((uint32_t)s_cut_end);
    DBG_STR(" caps=");                    DBG_U32(s_cut_caps);
    DBG_STR(" total=");                   DBG_U32(s_cut_total);
    DBG_NL();
#endif
}

/* V024b: the PIC-local millisecond at which this session's cutoff was frozen.
 * Used to correct the reported start_time - see the note in main.c's TIME_SYNC
 * handler. Valid only while a session cutoff is frozen. */
uint32_t FlowReport_GetCutoffMs(void)
{
    return s_cut_ms;
}

/* Session over (Photon powered off). Drop the cutoff so the next session must
 * freeze its own. A pending un-ACKed batch deliberately SURVIVES this: it is
 * retransmitted, unchanged, at the start of the next session (spec: an un-ACKed
 * batch always has priority and is never extended or altered). */
void FlowReport_ClearSessionCutoff(void)
{
    s_cut_valid = false;
}

/* Called from the UART RX path when 0xAA arrives (ISR context). */
void FlowReport_NotifyAA(void)
{
    s_aa = true;
}

/* Called when the Photon returns PKT_DATA_RECEIVED (0x0B): the last streamed
 * batch was received & stored, so it is now safe to CONSUME. This is the ONLY
 * place the committed read pointer and the report marks advance
 * (consume-on-ACK). Without this ACK a batch is treated as NOT sent and is
 * retransmitted on the next REQ_DATA. */
void FlowReport_NotifyAck(void)
{
    /* v1 / V063: the ACK carries no identifier, so it can only mean "the last
     * thing you streamed". Accept it as before. */
    FlowReport_NotifyAckSpan(s_span_end_pul, s_span_end_cap);
}

/* V024: an ACK that NAMES the batch it releases, by its position on the pulse
 * axis rather than by a synthetic counter.
 *
 * The pair (end_pulses, end_captures) identifies a batch exactly and for ever:
 * captures advance monotonically within a boot, so no two distinct batches of
 * one run can share it, and unlike a small counter it cannot wrap round to
 * collide with an older one after a PIC reset. It also needs no state of its
 * own - the endpoints are already the thing being committed.
 *
 * A mismatched ACK is DISCARDED rather than acted on. That is the whole point:
 * a delayed or duplicated 0x0B must never release a batch the Photon did not
 * store, because the PIC would then drop water that was never delivered. */
void FlowReport_NotifyAckSpan(uint32_t ackEndPul, uint32_t ackEndCap)
{
    if (!s_pending_commit) {
        return;                            /* nothing outstanding to commit */
    }
    if (ackEndPul != s_span_end_pul || ackEndCap != s_span_end_cap) {
#ifdef APP_DEBUG_EVENT_LOG
        DBG_STR("[RPT ] ACK for a different span: got pul="); DBG_U32(ackEndPul);
        DBG_STR(" cap=");   DBG_U32(ackEndCap);
        DBG_STR(" want pul="); DBG_U32(s_span_end_pul);
        DBG_STR(" cap=");      DBG_U32(s_span_end_cap);
        DBG_STR(" - ignored"); DBG_NL();
#endif
        return;                            /* not this batch: do NOT commit */
    }
    s_read = s_end;                        /* consume up to the sent end     */

    /* Advance the marks to the frozen END of the batch, not to wherever the
     * meter has reached by ACK time. This is what makes the next report begin
     * exactly where this one ended - the continuity the Photon audits. */
    s_impulse_mark  = s_span_end_pul;
    s_caps_mark     = s_span_end_cap;
    s_total_at_read = s_span_end_pul;      /* total at the new committed read */

    s_pending_commit = false;
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

    /* push whole frame if there is room; else drop (Photon will retry).
     * V022 / H.7.2: the drop is no longer silent - it is counted and logged, so
     * a Photon-side "no valid reply" can be matched against txdrops in one round. */
    if (UART_TX_Free() >= i) {
        for (uint16_t k = 0; k < i; k++) {
            (void)print_char((char)frame[k]);
        }
#ifdef APP_DEBUG_EVENT_LOG
        DBG_STR("TX: 0x");  DBG_HEX8(func);
        DBG_STR(" len=");   DBG_U32((uint32_t)len);   DBG_NL();
#endif
    } else {
        if (s_txdrops < 0xFFFFu) s_txdrops++;   /* saturate, never wrap */
#ifdef APP_DEBUG_EVENT_LOG
        DBG_STR("TX DROP: 0x"); DBG_HEX8(func);
        DBG_STR(" need=");      DBG_U32((uint32_t)i);
        DBG_STR(" free=");      DBG_U32((uint32_t)UART_TX_Free());  DBG_NL();
#endif
    }
}

/* ---- V022 / H.7.2: TX drop accounting accessors ---- */
uint16_t FlowReport_GetTxDrops(void)
{
    return s_txdrops;
}

void FlowReport_ClearTxDrops(void)
{
    s_txdrops = 0;
}

void FlowReport_SendParam(void)
{
    leak_param_t pr;
    uint8_t d[16];
    FlowControl_GetParams(&pr);
    d[0] = (uint8_t)(pr.leak1_counts   >> 8);  d[1] = (uint8_t)pr.leak1_counts;
    d[2] = (uint8_t)(pr.leak1_window_s >> 8);  d[3] = (uint8_t)pr.leak1_window_s;
    d[4] = (uint8_t)(pr.leak2_counts   >> 8);  d[5] = (uint8_t)pr.leak2_counts;
    d[6] = (uint8_t)(pr.leak2_window_s >> 8);  d[7] = (uint8_t)pr.leak2_window_s;
    /* extension: align_anchor (seconds-of-day) + report_interval (seconds), BE */
    uint32_t anchor = Loco_GetAnchor();
    uint32_t intvl  = Loco_GetReportInterval();
    d[8]  = (uint8_t)((anchor >> 24) & 0xFFu);  d[9]  = (uint8_t)((anchor >> 16) & 0xFFu);
    d[10] = (uint8_t)((anchor >>  8) & 0xFFu);  d[11] = (uint8_t)( anchor        & 0xFFu);
    d[12] = (uint8_t)((intvl  >> 24) & 0xFFu);  d[13] = (uint8_t)((intvl  >> 16) & 0xFFu);
    d[14] = (uint8_t)((intvl  >>  8) & 0xFFu);  d[15] = (uint8_t)( intvl         & 0xFFu);
    send_small_packet(PKT_RSP_PARAM, d, 16u);
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

/* Dedicated reply to PKT_TIME_SYNC so the Photon can retransmit the cloud time
 * on loss (mirror of DATA_RECEIVED, PIC -> Photon direction). len 0. */
void FlowReport_SetReportMeta(uint32_t start_time, uint8_t start_valid,
                              uint32_t interval_ms)
{
    s_f_start_time  = start_time;
    s_f_start_valid = start_valid;
    s_f_interval_ms = interval_ms;
}

/* Enter/leave initial-boot mode (REQ_DATA -> zero samples + wipe). */
void FlowReport_SetInitialMode(bool initial)
{
    s_initial_boot = initial;
}

/* RSP_LOCO (0x8A): 10-byte big-endian LOCO calibration status snapshot, so the
 * Photon can print the PIC's internal LOCO state on the bench. */
void FlowReport_SendLocoStatus(void)
{
    LocoStatus st;
    uint8_t d[10];
    Loco_GetStatus(&st);
    d[0] = (uint8_t)(st.hf_precision    >> 8);  d[1] = (uint8_t)st.hf_precision;
    d[2] = st.hf_calibrated;
    d[3] = st.hf_in_range;
    d[4] = st.cloud_calibrated;
    d[5] = (uint8_t)(st.cloud_precision >> 8);  d[6] = (uint8_t)st.cloud_precision;
    d[7] = st.cloud_in_range;
    d[8] = (uint8_t)(st.applied_k       >> 8);  d[9] = (uint8_t)st.applied_k;
    send_small_packet(PKT_RSP_LOCO, d, 10u);
}

void FlowReport_SendTimeReceived(void)
{
    send_small_packet(PKT_TIME_RECEIVED, (const uint8_t *)0, 0u);
}

/* RSP_POWER_STATE: one byte, 0 = INITIAL power-hold, 1 = NORMAL. */
void FlowReport_SendPowerState(uint8_t state)
{
    uint8_t d = state;
    send_small_packet(PKT_RSP_POWER_STATE, &d, 1u);
}

/* RSP_PHOTON_CFG : 13-byte config block the Photon reads at boot.
 * If PIC_PROVIDES_PHOTON_CFG is undefined we still answer, with provided=0, so
 * the Photon uses its own defaults (and never hangs waiting). */
void FlowReport_SendPhotonCfg(void)
{
    uint8_t d[13];
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

    /* B. debug toggles */
    d[i++] = (uint8_t)PCFG_FAST_BENCH;
    d[i++] = (uint8_t)PCFG_DEBUG_DATASERIES;
    d[i++] = (uint8_t)PCFG_MISSED_FILL_MODE;
    d[i++] = (uint8_t)((PCFG_SERIAL_DELAY_MS >> 8) & 0xFFu);
    d[i++] = (uint8_t)( PCFG_SERIAL_DELAY_MS       & 0xFFu);

    send_small_packet(PKT_RSP_PHOTON_CFG, d, i);   /* i == 13 */
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

            /* ---- V023 / fix 5: RETRANSMIT, do not rebuild ----
             * If a batch was streamed and the Photon has not ACKed it, this
             * REQ_DATA is a RETRY of that batch. Every header field, the
             * sample span and therefore the CRC must come out byte-for-byte
             * identical, so nothing is recomputed: we only rewind the TEMP
             * send pointer and stream the same frozen snapshot again.
             *
             * V022 fell through to the code below and built a NEW, larger
             * batch on every retry, which is why the bench log shows a single
             * un-ACKed report answered as n=39, n=40, n=40, n=41. That also
             * broke the ACK contract: the design says an un-ACKed batch is
             * re-sent unchanged.
             *
             * This branch is what carries an un-ACKed batch across a power
             * cycle too - s_pending_commit is not cleared when the session
             * ends, so the batch is re-offered at the start of the next one,
             * before any newer data. */
            if (s_pending_commit && !s_initial_boot) {
                s_tx_read = s_read;        /* rewind to the frozen start  */
                s_i       = 0;             /* per-report index restarts   */
#ifdef APP_DEBUG_PKT_LOG
                DBG_STR("TX DATA retransmit pul=["); DBG_U32(s_span_begin_pul);
                DBG_STR("..");                        DBG_U32(s_span_end_pul);
                DBG_STR("] cap=");                    DBG_U32(s_f_cap_report);
                DBG_STR(" span="); DBG_U32(s_f_imp_span);
                DBG_STR(" n=");    DBG_U32((uint32_t)s_count);
                DBG_NL();
#endif
                LedFsm_NotifyDataCycle();
                s_send_state = SEND_PKT_HDR;
                break;
            }

            /* ---- initial-boot: report ZERO and WIPE ----
             * During the cold-boot hold we have no valid time yet, so any
             * captures collected are discarded. Advance the consumer to the
             * last COMPLETED capture (wipe), zero every header field, and
             * stream a 0-sample report. Repeats safely on every retransmit.
             *
             * V023: the wipe boundary now comes from FlowLog_GetSnapshot() too.
             * V022 set s_read from the write index but the marks from the LIVE
             * meter, so the pulses of the capture still being built were
             * declared already-reported while its sample was still queued -
             * the same mismatch, just at the wipe. */
            if (s_initial_boot) {
                flowlog_snapshot_t snap;
                FlowLog_GetSnapshot(&snap);
                s_end           = snap.write;
                s_read          = s_end;             /* drop all captured so far */
                s_tx_read       = s_read;
                s_total_at_read = snap.completed_total;
                s_impulse_mark  = s_total_at_read;
                s_caps_mark     = snap.captures;
                s_commit_total  = s_total_at_read;
                s_commit_caps   = s_caps_mark;
                s_f_start_time  = 0u;  s_f_start_valid = 0u;
                s_f_cap_report  = 0u;  s_f_imp_span    = 0u;
                s_f_ovf_span    = 0u;  s_count         = 0u;
                s_i = 0;
                LedFsm_NotifyDataCycle();
                s_send_state = SEND_PKT_HDR;
                break;
            }

            /* ---- ring safety policy ----
             * Un-sent backlog = ring distance (end - read). If it is within the
             * safe fill (SLOTS - margin) send it all. If it has grown past that
             * (a report was skipped, ring nearly wrapped), do NOT try to drain
             * the whole ring: jump the consumer to the most recent report-due
             * boundary and send only that last period. Older samples are dropped
             * from the SERIES, but their pulses are still carried in field 1
             * (impulse_since_report), so no flow total is lost. */
            /* ---- V023 / fix 4+5: build from the FROZEN cutoff ----
             * end / captures / total all come from the single boundary frozen
             * when this session started. FlowMeter_GetTotal() is NOT called
             * here: it refreshes to the live instant on every call, so V022
             * mixed a sample span ending at capture #34 (total 98) with a
             * meter reading that had already run into capture #35 (total 103).
             * On the next batch that same 5-pulse sample was then sent against
             * a header total of 2, i.e. sum(samples) > total_impulses - a state
             * that cannot occur in healthy data and which the Photon (rightly)
             * treats as corrupt.
             *
             * Fallback: if no cutoff was frozen (a REQ_DATA outside a normal
             * report-due session - the Photon booted for some other reason)
             * freeze one now, at first contact. Late is fine; moving is not. */
            if (!s_cut_valid) {
                FlowReport_FreezeSessionCutoff();
            }
            s_end = s_cut_end;

            uint16_t backlog = (uint16_t)((s_end - s_read + FLOWLOG_SLOTS)
                                          % FLOWLOG_SLOTS);
            uint32_t cur_total = s_cut_total;   /* completed-capture boundary */
            uint32_t cur_caps  = s_cut_caps;

            if (backlog > (uint16_t)(FLOWLOG_SLOTS - APP_FLOW_RING_MARGIN)
                && FlowLog_DueValid()) {
                s_read          = FlowLog_GetWriteAtDue();  /* last period start */
                s_total_at_read = FlowLog_GetTotalAtDue();  /* total there       */
            }

            /* ---- extended header fields ----
             * BOTH totals are measured "since the last delivered report" so the
             * Photon's missed-fill can reconstruct water that has no samples:
             *   total_captures = cur_caps  - s_caps_mark
             *   total_impulses = cur_total - s_impulse_mark
             * Using s_impulse_mark (advances only on ACK) - NOT s_total_at_read
             * (advances on DropOldest/truncate) - means ring-dropped or truncated
             * pulses stay counted in total_impulses, so no billed water is lost.
             * The Photon sees total_impulses > sum(samples) and restores the
             * missing volume (hourly.cpp missed-fill). */
            /* ---- V024: record the span as a POSITION, derive the delta ----
             * begin/end are absolute points on the PIC's lifetime axis. The two
             * delta fields v1 still needs are computed FROM them here and
             * nowhere else, so the header can no longer state a delta that
             * disagrees with the endpoints it came from.
             *
             * begin is s_impulse_mark - the last ACKed position, NOT
             * s_total_at_read. That is what keeps ring-dropped or truncated
             * pulses inside the span: they were never delivered, so they are
             * still owed, and the Photon's missed-fill restores the volume even
             * though the samples are gone. */
            s_span_begin_pul = s_impulse_mark;
            s_span_end_pul   = cur_total;
            s_span_begin_cap = s_caps_mark;
            s_span_end_cap   = cur_caps;

            s_f_cap_report = s_span_end_cap - s_span_begin_cap;  /* total_captures */
            s_f_imp_span   = s_span_end_pul - s_span_begin_pul;  /* total_impulses */
            s_f_ovf_span   = FlowLog_CountOverflows(s_read, s_end); /* 4          */
            s_count        = (uint16_t)((s_end - s_read + FLOWLOG_SLOTS)
                                        % FLOWLOG_SLOTS);  /* 5: samples to send   */

            /* consume-on-ACK: transmit from a TEMP pointer; the committed
             * read (s_read) and the marks do NOT advance here. They advance
             * only when the Photon returns PKT_DATA_RECEIVED (see
             * FlowReport_NotifyAck). Until then the batch can be retransmitted. */
            s_tx_read      = s_read;
            s_commit_total = cur_total;   /* total at s_end - commit uses this  */
            s_commit_caps  = cur_caps;

            /* V023 / fix 6: a NEW snapshot gets a NEW name. Retransmits take
             * the branch above and never reach this line, so one number
             * identifies one batch for as long as it exists. 0 is reserved to
             * mean "no batch", so skip it on wrap. */


#ifdef APP_DEBUG_PKT_LOG
            /* One-line summary of the report we are about to stream. */
            DBG_STR("TX DATA boot=");   DBG_U32((uint32_t)s_boot_id);
            DBG_STR(" pul=[");          DBG_U32(s_span_begin_pul);
            DBG_STR("..");              DBG_U32(s_span_end_pul);
            DBG_STR("] start=");        DBG_U32(s_f_start_time);
            DBG_STR(" valid=");         DBG_U32((uint32_t)s_f_start_valid);
            DBG_STR(" cap=");           DBG_U32(s_f_cap_report);
            DBG_STR(" span=");          DBG_U32(s_f_imp_span);
            DBG_STR(" ovf=");           DBG_U32((uint32_t)s_f_ovf_span);
            DBG_STR(" n=");            DBG_U32((uint32_t)s_count);
            DBG_NL();
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
        /* All fields big-endian, CRC accumulated as they go.
         *
         * v1 (23 B, PCFG_PROTO_V2 == 0) - unchanged from V022:
         *   start_time(u32) start_valid(u8) sample_count(u32) total_captures(u32)
         *   total_impulses(u32) overflow_ffff(u16) sample_interval_ms(u32)
         * total_captures and total_impulses are PERIOD values here, not running
         * totals. In V024 they are derived from the span endpoints, so they can
         * no longer disagree with them - but a v1 receiver still cannot CHECK
         * them, which is precisely why v2 exists.
         *
         * v2 (31 B, PCFG_PROTO_V2 == 1) - the amount as a position:
         *   span_begin_pulses(u32)  cumulative pulse total where this report starts
         *   span_end_pulses(u32)    cumulative pulse total where it ends
         *   span_begin_captures(u32) cumulative capture count at the start
         *   span_end_captures(u32)   cumulative capture count at the end
         *   boot_id(u16)            changes on every PIC restart
         *   sample_count(u16)       (u16 is ample: the ring holds 1024)
         *   overflow_ffff(u16)
         *   sample_interval_ms(u32)
         *   start_time(u32) start_valid(u8)
         *
         * The receiver derives:
         *   total_impulses = span_end_pulses   - span_begin_pulses
         *   total_captures = span_end_captures - span_begin_captures
         * and audits:
         *   span_begin_pulses == the previous accepted report's span_end_pulses
         * A repeat of an already-stored report has identical endpoints; a hole
         * shows up as a forward jump whose SIZE IN PULSES is known exactly. */
        uint8_t hdr[XHDR_BYTES];
        uint8_t p = 0;
#if PCFG_PROTO_V2
        hdr[p++] = (uint8_t)((s_span_begin_pul >> 24) & 0xFFu); /* span_begin_pulses */
        hdr[p++] = (uint8_t)((s_span_begin_pul >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)((s_span_begin_pul >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( s_span_begin_pul        & 0xFFu);
        hdr[p++] = (uint8_t)((s_span_end_pul   >> 24) & 0xFFu); /* span_end_pulses   */
        hdr[p++] = (uint8_t)((s_span_end_pul   >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)((s_span_end_pul   >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( s_span_end_pul          & 0xFFu);
        hdr[p++] = (uint8_t)((s_span_begin_cap >> 24) & 0xFFu); /* span_begin_caps   */
        hdr[p++] = (uint8_t)((s_span_begin_cap >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)((s_span_begin_cap >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( s_span_begin_cap        & 0xFFu);
        hdr[p++] = (uint8_t)((s_span_end_cap   >> 24) & 0xFFu); /* span_end_caps     */
        hdr[p++] = (uint8_t)((s_span_end_cap   >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)((s_span_end_cap   >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( s_span_end_cap          & 0xFFu);
        hdr[p++] = (uint8_t)((s_boot_id        >>  8) & 0xFFu); /* boot_id           */
        hdr[p++] = (uint8_t)( s_boot_id               & 0xFFu);
        hdr[p++] = (uint8_t)((s_count          >>  8) & 0xFFu); /* sample_count      */
        hdr[p++] = (uint8_t)( s_count                 & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_ovf_span     >>  8) & 0xFFu); /* overflow_ffff     */
        hdr[p++] = (uint8_t)( s_f_ovf_span            & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_interval_ms  >> 24) & 0xFFu); /* sample_interval   */
        hdr[p++] = (uint8_t)((s_f_interval_ms  >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_interval_ms  >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( s_f_interval_ms         & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_start_time   >> 24) & 0xFFu); /* start_time        */
        hdr[p++] = (uint8_t)((s_f_start_time   >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_start_time   >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( s_f_start_time          & 0xFFu);
        hdr[p++] = s_f_start_valid;                            /* start_valid       */
#else
        hdr[p++] = (uint8_t)((s_f_start_time >> 24) & 0xFFu);   /* start_time */
        hdr[p++] = (uint8_t)((s_f_start_time >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_start_time >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( s_f_start_time        & 0xFFu);
        hdr[p++] = s_f_start_valid;                            /* start_valid */
        hdr[p++] = (uint8_t)(((uint32_t)s_count >> 24) & 0xFFu);/* sample_count */
        hdr[p++] = (uint8_t)(((uint32_t)s_count >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)(((uint32_t)s_count >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( (uint32_t)s_count        & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_cap_report >> 24) & 0xFFu);   /* total_captures */
        hdr[p++] = (uint8_t)((s_f_cap_report >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_cap_report >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( s_f_cap_report        & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_imp_span   >> 24) & 0xFFu);   /* total_impulses */
        hdr[p++] = (uint8_t)((s_f_imp_span   >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_imp_span   >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( s_f_imp_span          & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_ovf_span   >>  8) & 0xFFu);   /* overflow_ffff */
        hdr[p++] = (uint8_t)( s_f_ovf_span          & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_interval_ms >> 24) & 0xFFu);  /* sample_interval */
        hdr[p++] = (uint8_t)((s_f_interval_ms >> 16) & 0xFFu);
        hdr[p++] = (uint8_t)((s_f_interval_ms >>  8) & 0xFFu);
        hdr[p++] = (uint8_t)( s_f_interval_ms        & 0xFFu);
#endif

        if (UART_TX_Free() < (uint16_t)XHDR_BYTES) break;
        for (uint8_t k = 0; k < XHDR_BYTES; k++) {
            s_rsp_crc = Packet_CRC16_Update(s_rsp_crc, hdr[k]);
        }
        send_raw(hdr, XHDR_BYTES);
        s_send_state = (s_count == 0u) ? SEND_PKT_CRC : SEND_BODY;
        break;
    }

    case SEND_BODY: {
        if (s_tx_read == s_end) {          /* reached the snapshot end   */
            s_send_state = SEND_PKT_CRC;   /* append CRC, then idle      */
            break;
        }

        flowlog_entry_t e;
        FlowLog_GetAt(s_tx_read, &e);

        uint8_t raw[COMPRESS_BYTES_PER_SAMPLE];
        Compress_Pack(s_i, e.pulses, raw);

#if defined(APP_DEBUG_DATASERIES)
        {   /* per-sample dump: index, group, pulses (bench only, voluminous) */
            uint16_t bufNo = (uint16_t)(s_tx_read % FLOWLOG_SLOTS);
            if (UART_TX_Free() < (uint16_t)(40u + COMPRESS_BYTES_PER_SAMPLE)) break;
            DBG_STR("  ["); DBG_U32((uint32_t)s_i);
            DBG_STR("] buf="); DBG_U32((uint32_t)bufNo);
            DBG_STR(" grp="); DBG_U32((uint32_t)e.grp);
            DBG_STR(" pul="); DBG_U32((uint32_t)e.pulses);
            DBG_NL();
        }
#elif defined(APP_DEBUG_PRINT_ENABLE)
        {   char dline[112];
            uint16_t bufNo = (uint16_t)(s_tx_read % FLOWLOG_SLOTS);
            (void)build_debug_line(dline, bufNo, s_i, e.pulses, raw);
            /* The line goes to the DEBUG UART, so it no longer competes with
             * the packet TX buffer; only the packet bytes need room here. */
            if (UART_TX_Free() < COMPRESS_BYTES_PER_SAMPLE) break;
            DBG_STR(dline);
        }
#else
        if (UART_TX_Free() < COMPRESS_BYTES_PER_SAMPLE) break;
#endif
        for (uint8_t k = 0; k < COMPRESS_BYTES_PER_SAMPLE; k++) {
            s_rsp_crc = Packet_CRC16_Update(s_rsp_crc, raw[k]);
        }
        send_raw(raw, COMPRESS_BYTES_PER_SAMPLE);
        s_tx_read = (uint16_t)((s_tx_read + 1u) % FLOWLOG_SLOTS); /* circular */
        s_i++;
        break;
    }

    case SEND_PKT_CRC: {
        uint8_t crc[2];
        crc[0] = (uint8_t)((s_rsp_crc >> 8) & 0xFFu);   /* big-endian */
        crc[1] = (uint8_t)( s_rsp_crc       & 0xFFu);
        if (UART_TX_Free() < 2u) break;
        send_raw(crc, 2u);
        /* Batch fully streamed but NOT yet committed. The committed read
         * (s_read) stays put until the Photon returns PKT_DATA_RECEIVED. If no
         * ACK arrives, the next REQ_DATA retransmits from s_read.
         *
         * V023b: an EMPTY batch is never pending.
         *
         * The Photon does not ACK a zero-sample report - correctly, since there
         * is nothing it could have stored - and V022 got away with that because
         * its SEND_IDLE rebuilt the batch from scratch every time, so a stale
         * pending flag healed itself on the next request. V023 does NOT rebuild
         * while a batch is pending (that is the whole point of fix 5), so an
         * empty batch left pending would be retransmitted for ever and no new
         * data could leave the PIC again. Consume-on-ACK exists to protect
         * SAMPLES; with no samples there is nothing to wait for.
         *
         * The marks are deliberately NOT advanced here either. If the ring
         * dropped samples the header can carry pulses with no samples attached,
         * and those pulses must stay in the next report's span rather than be
         * declared delivered to a Photon that stored nothing. */
        s_pending_commit = (s_count > 0u);
        s_send_state = SEND_IDLE;          /* a later request starts anew */
        break;
    }

    default:
        s_send_state = SEND_IDLE;
        break;
    }
}