#define FLOWLOG_C

#include <xc.h>
#include <stdint.h>
#include "FlowLog.h"
#include "Compress.h"
#include "FlowMeter.h"
#include "MCU_Time.h"
#include "Flow_Control.h"
#include "Dev_Debug_Uart.h"  /* bench logs -> separate debug UART (RA4) */
#include "FlowReport.h"     /* FIFO read-side: enforce overrun on every push */

/* raw byte ring buffer, sized by the selected compression method */
static uint8_t  s_buf[FLOWLOG_SLOTS * COMPRESS_BYTES_PER_SAMPLE];

/* s_write and the consumer index (s_read in FlowReport) both circulate
 * within 0..SLOTS-1; the physical slot IS the index. The ring holds at most
 * SLOTS samples; if the consumer cannot keep up the oldest are overwritten.
 * A report sends the ring distance (write - read), so "buffer exactly full"
 * and "buffer empty" both read as 0 (classic ring full/empty ambiguity);
 * overflow is avoided by correct board installation, not in firmware. */
static uint16_t s_write      = 0;   /* circular write index (0..SLOTS-1) */
static uint16_t s_group      = 0;   /* report counter / group number (0..BATCH-1) */
static uint32_t s_prev_total = 0;   /* FlowMeter total at last capture */
/* --- per-wake accumulation (user's design) ------------------------------
 * EVERY wake we read the async counter, take the wrap-safe delta, and add it
 * to two 32-bit accumulators:
 *   s_sample_accum : pulses for the FIFO sample now being built; zeroed when
 *                    the sample is written to the ring.
 *   (the report-period total is carried by the span logic below, which is
 *    consume-on-ACK safe; see total_impulses = cur - s_total_at_read.)
 * Reading every wake (not once per 4-min capture) means the 16-bit Timer1
 * cannot wrap twice between reads, so no pulses are lost at high flow. */
static uint32_t s_prev_wake_total = 0;   /* FlowMeter total at last wake        */
static uint32_t s_sample_accum    = 0;   /* pulses accumulated for this sample  */
/* "wake" here means the flow-read MINIMUM PERIOD, not literally waking from
 * sleep: the read fires once per APP_WAKE_ACTUAL_MS whether we got here by
 * waking from deep sleep OR by looping while the CPU stayed active (Photon
 * session, valve drive). s_wake_mark gates it on the software clock. */
static uint32_t s_wake_mark       = 0;   /* last flow-read time reference       */
static uint16_t s_wakes_in_sample = 0;   /* wake-reads since last FIFO push     */
static uint32_t s_captures   = 0;   /* 32-bit total captures (SampleNumber) */
static bool     s_batch_ready = false; /* set when a group of BATCH just completed */
static uint16_t s_batch_size  = FLOWLOG_BATCH; /* current report period (captures);
                                       * may be nudged once for midnight alignment */

/* Max pulses that fit the selected method's pulse field. A per-capture delta
 * larger than this is clamped to it, and the sample is flagged as "overflowed"
 * so the report can tell the host how many samples were distorted.
 *   NOCOMP_16  -> 65535 (16-bit)   PACK_10_14 -> 16383 (14-bit) */
#if (COMPRESS_METHOD_SELECTED == COMPRESS_METHOD_NOCOMP_16)
  #define FLOWLOG_SAMPLE_MAX  65535u
#else
  #define FLOWLOG_SAMPLE_MAX  16383u
#endif

/* One overflow flag per ring slot (125 bytes for 1000 slots): bit set = that
 * sample's true pulse count exceeded 14 bits and was clamped. */
static uint8_t  s_ovf_bits[(FLOWLOG_SLOTS + 7u) / 8u];

/* Snapshots taken at report-due boundaries (every BATCH captures): the running
 * total and write index. We keep the last TWO so a truncating report can send a
 * FULL most-recent period (from the previous boundary), not just the partial
 * samples since the very last boundary. */
static uint32_t s_total_at_due  = 0;   /* most recent boundary */
static uint16_t s_write_at_due  = 0;
static uint32_t s_total_at_due2 = 0;   /* one boundary before that */
static uint16_t s_write_at_due2 = 0;
static uint8_t  s_due_count     = 0;   /* how many boundaries seen (caps at 2) */

static inline void ovf_set(uint16_t slot, bool on)
{
    uint16_t byte = (uint16_t)(slot >> 3);
    uint8_t  mask = (uint8_t)(1u << (slot & 7u));
    if (on) s_ovf_bits[byte] |= mask;
    else    s_ovf_bits[byte] = (uint8_t)(s_ovf_bits[byte] & (uint8_t)~mask);
}

/* Count overflow flags set in the ring span [from, to) (circular). */
uint16_t FlowLog_CountOverflows(uint16_t from, uint16_t to)
{
    uint16_t n = 0;
    uint16_t i = from;
    while (i != to) {
        uint16_t byte = (uint16_t)(i >> 3);
        uint8_t  mask = (uint8_t)(1u << (i & 7u));
        if ((s_ovf_bits[byte] & mask) != 0u) n++;
        i = (uint16_t)((i + 1u) % FLOWLOG_SLOTS);
    }
    return n;
}

/* For safety-truncation we want a FULL most-recent period, so we expose the
 * PREVIOUS due boundary (one period before the latest). If only one boundary
 * has occurred, fall back to the latest. */
uint32_t FlowLog_GetTotalAtDue(void)
{
    return (s_due_count >= 2u) ? s_total_at_due2 : s_total_at_due;
}
uint16_t FlowLog_GetWriteAtDue(void)
{
    return (s_due_count >= 2u) ? s_write_at_due2 : s_write_at_due;
}
bool     FlowLog_DueValid(void)       { return s_due_count >= 1u; }

void FlowLog_Init(void)
{
    s_write      = 0;
    s_group      = 0;
    s_captures   = 0;
    s_batch_ready = false;
    s_prev_total = FlowMeter_GetTotal();   /* baseline from the meter */
    s_prev_wake_total = s_prev_total;      /* per-wake delta baseline */
    s_sample_accum    = 0u;                /* empty sample accumulator */
    s_wake_mark       = getNowTime();      /* flow-read period baseline */
    s_wakes_in_sample = 0u;                /* counter to next FIFO push */

    for (uint16_t i = 0; i < (uint16_t)sizeof(s_ovf_bits); i++)
        s_ovf_bits[i] = 0u;
    s_total_at_due  = s_prev_total;
    s_write_at_due  = 0;
    s_total_at_due2 = s_prev_total;
    s_write_at_due2 = 0;
    s_due_count     = 0;
}

void FlowLog_Process(void)
{
    /* ---- EVERY wake: fold the async counter into the running total and add
     * the wrap-safe delta to the sample accumulator. Doing this every wake
     * (not once per capture period) keeps the 16-bit Timer1 from wrapping
     * twice between reads, so high-flow pulses are never lost. ---- */
    /* ===== LAYER 1: asyntimer-read-periodic-routine =========================
     * The ONLY time-based gate. Fires once per APP_WAKE_ACTUAL_MS ("wake" =
     * flow-read minimum period), whether reached by waking from deep sleep OR
     * by looping while the CPU stayed active. Reads the async counter, folds
     * the wrap-safe delta into the 32-bit total, and drives the leak alarm
     * FIFO. Everything below is COUNTER-based off this cadence. ---- */
    if (timeSpan(s_wake_mark) < (uint32_t)APP_WAKE_ACTUAL_MS) {
        return;                              /* not a flow-read tick yet */
    }
    s_wake_mark = getNowTime();

    uint32_t now_total  = FlowMeter_GetTotal();          /* refresh 32-bit total */
    uint32_t wake_delta = now_total - s_prev_wake_total; /* pulses this period   */
    s_prev_wake_total = now_total;
    s_sample_accum   += wake_delta;                      /* -> sample accum      */
    FlowControl_OnWake(wake_delta);                      /* -> alarm FIFO / leak */

    /* ===== LAYER 2: fifo-push-periodic-routine (counter-based) ===============
     * Count flow-read ticks; push ONE FIFO sample every APP_WAKES_PER_SAMPLE
     * ticks. No separate time gate -- strictly nested under Layer 1. ---- */
    s_wakes_in_sample++;
    if (s_wakes_in_sample < (uint16_t)APP_WAKES_PER_SAMPLE) {
        return;                              /* sample not full yet */
    }
    s_wakes_in_sample = 0u;

    /* per-sample pulses = the accumulator built from the per-wake reads. cur is
     * the running total AS OF the last wake read, so (cur - s_prev_total) equals
     * s_sample_accum exactly (both anchored to wake reads). */
    uint32_t cur = s_prev_wake_total;
    uint32_t d32 = s_sample_accum;          /* true pulses this capture period  */
    s_prev_total = cur;
    s_sample_accum = 0u;                    /* zero the sample accumulator      */

#ifdef APP_DEBUG_EVENT_LOG
    /* tot = grand total since boot. Divided by elapsed seconds it must equal
     * the virtual meter's declared rate, so the log alone verifies the flow
     * path end to end. ovf counts samples clamped at 0xFFFF. */
    DBG_STR("[CAP ] #"); DBG_U32(s_captures + 1u);
    DBG_STR(" pul=");    DBG_U32(d32);
    DBG_STR(" tot=");    DBG_U32(cur);
    DBG_STR(" t=");      DBG_U32(getNowTime()); DBG_NL();
#endif

    /* Clamp to the sample field (16-bit). If the true count exceeds it, store
     * the max and flag this sample as overflowed; the per-report overflow
     * count (CountOverflows over the span) then tells the host how many FIFO
     * writes were clamped. The report's true-impulse field still carries the
     * exact total (from the 32-bit running total over the span). */
    uint16_t slot  = (uint16_t)(s_write % FLOWLOG_SLOTS);
    uint16_t delta;
    if (d32 > (uint32_t)FLOWLOG_SAMPLE_MAX) {
        delta = (uint16_t)FLOWLOG_SAMPLE_MAX;
        ovf_set(slot, true);
    } else {
        delta = (uint16_t)d32;
        ovf_set(slot, false);
    }

    s_captures += 1u;

    /* front field = group number (0..BATCH-1), back field = pulses */
    Compress_Pack(s_group, delta,
                  &s_buf[(uint16_t)(slot * COMPRESS_BYTES_PER_SAMPLE)]);

    s_write = (uint16_t)((s_write + 1u) % FLOWLOG_SLOTS);   /* circular */

    /* ---- PUSH-time overrun enforcement (spec 5.4) ----
     * The flow record is one ring FIFO. On EVERY capture, if the unconsumed
     * backlog has reached the usable cap (SLOTS - MARGIN), force the read side
     * to drop its oldest sample. This guarantees write can never lap read, so
     * write==read always means EMPTY, regardless of whether the consumer is
     * stalled waiting for an ACK. Dropped pulses are preserved in the report
     * totals (only the oldest sample's time-distribution is lost). */
    while (FlowReport_GetUsed() >= (uint16_t)(FLOWLOG_SLOTS - APP_FLOW_RING_MARGIN)) {
        FlowReport_DropOldest();
    }

    s_group++;
    if (s_group >= s_batch_size) {
        s_group = 0;
        s_batch_ready = true;    /* a full group just completed */
        s_batch_size = FLOWLOG_BATCH;  /* revert: phase nudge is one-time */
        /* Report-due boundary: shift snapshots (prev <- last <- now) so a
         * truncating report can send a full most-recent period. */
        s_total_at_due2 = s_total_at_due;
        s_write_at_due2 = s_write_at_due;
        s_total_at_due  = cur;
        s_write_at_due  = s_write;
        if (s_due_count < 2u) s_due_count++;
    }

    /* Leak detection is fed per-WAKE now (FlowControl_OnWake above), using its
     * own alarm FIFO, so nothing to do here at capture time. */
}

uint16_t FlowLog_GetWriteIndex(void)
{
    return s_write;
}

/* V023 / fix 3: ONE boundary, THREE values, read together.
 *
 * s_write, s_captures and s_prev_total are all advanced in the same place -
 * the FIFO push at the end of FlowLog_Process() - so as a set they always
 * describe the SAME completed capture. Reading them one at a time (and worse,
 * mixing in FlowMeter_GetTotal(), which refreshes to the live instant on every
 * call) is what let a report claim 34 samples ending at total 98 while its
 * header said 103: the extra 5 pulses belonged to capture #35, which had not
 * been pushed yet.
 *
 * completed_total is s_prev_total - the running total AS OF the last sample
 * actually written to the ring - NOT the live meter. Pulses of the capture
 * currently being built are deliberately excluded; they belong to the next
 * report.
 *
 * Atomicity: FlowLog_Process() and FlowReport_Process() are both called from
 * the main loop (see main.c), never from an ISR, so they cannot interleave and
 * no critical section is needed here. If a future revision ever moves the
 * capture push into an interrupt, this function must disable interrupts around
 * the three copies. */
void FlowLog_GetSnapshot(flowlog_snapshot_t *s)
{
    if (!s) return;
    s->write           = s_write;
    s->captures        = s_captures;
    s->completed_total = s_prev_total;
}

void FlowLog_GetAt(uint16_t index, flowlog_entry_t *out)
{
    uint16_t slot = (uint16_t)(index % FLOWLOG_SLOTS);
    Compress_Unpack(&s_buf[(uint16_t)(slot * COMPRESS_BYTES_PER_SAMPLE)],
                    &out->grp, &out->pulses);
}

/* Copy the raw stored bytes of one slot (exactly COMPRESS_BYTES_PER_SAMPLE
 * bytes) into dst. Lets callers show how a sample is actually packed. */
void FlowLog_GetRawAt(uint16_t index, uint8_t *dst)
{
    uint16_t slot = (uint16_t)(index % FLOWLOG_SLOTS);
    const uint8_t *src = &s_buf[(uint16_t)(slot * COMPRESS_BYTES_PER_SAMPLE)];
    for (uint8_t k = 0; k < COMPRESS_BYTES_PER_SAMPLE; k++) {
        dst[k] = src[k];
    }
}

uint32_t FlowLog_GetCaptureCount(void)
{
    return s_captures;
}

bool FlowLog_BatchReady(void)
{
    if (s_batch_ready) {
        s_batch_ready = false;   /* clear on read: fires once per group */
        return true;
    }
    return false;
}

/* Override the report period (in captures) for the CURRENT group only, for
 * one-time midnight phase alignment. Reverts to FLOWLOG_BATCH after the group
 * completes. Clamped to at least 1. Independent of the LOCO frequency factor. */
void FlowLog_SetNextBatch(uint16_t captures)
{
    s_batch_size = (captures < 1u) ? 1u : captures;
}
