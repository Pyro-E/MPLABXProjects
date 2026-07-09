#define FLOWLOG_C

#include <xc.h>
#include <stdint.h>
#include "FlowLog.h"
#include "Compress.h"
#include "FlowMeter.h"
#include "MCU_Time.h"
#include "Flow_Control.h"   /* leak detection hook */

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
static uint32_t s_mark_ms    = 0;   /* capture-period time reference */
static uint32_t s_prev_total = 0;   /* FlowMeter total at last capture */
static uint32_t s_captures   = 0;   /* 32-bit total captures (SampleNumber) */
static bool     s_batch_ready = false; /* set when a group of BATCH just completed */

/* Max pulses that fit the 14-bit sample field (10-14 pack). A per-capture delta
 * larger than this is clamped to it, and the sample is flagged as "overflowed"
 * so the report can tell the host how many samples were distorted. */
#define FLOWLOG_SAMPLE_MAX  16383u

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
    s_mark_ms    = getNowTime();

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
    if (timeSpan(s_mark_ms) < FLOWLOG_PERIOD_MS) {
        return;
    }
    s_mark_ms = getNowTime();

    /* per-period delta = difference of the 32-bit running total (true count) */
    uint32_t cur = FlowMeter_GetTotal();
    uint32_t d32 = cur - s_prev_total;      /* true pulses this capture period  */
    s_prev_total = cur;

    /* Clamp to the 14-bit sample field. If the true count exceeds it, store the
     * max and flag this sample as overflowed. The loss is by design (very high
     * flow, beyond the meter's intended range); the report's true-impulse field
     * still carries the exact total, and the overflow count tells the host how
     * many samples were clamped. */
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

    s_group++;
    if (s_group >= FLOWLOG_BATCH) {
        s_group = 0;
        s_batch_ready = true;    /* a full group of BATCH just completed */
        /* Report-due boundary: shift snapshots (prev <- last <- now) so a
         * truncating report can send a full most-recent period. */
        s_total_at_due2 = s_total_at_due;
        s_write_at_due2 = s_write_at_due;
        s_total_at_due  = cur;
        s_write_at_due  = s_write;
        if (s_due_count < 2u) s_due_count++;
    }

    /* Feed the leak detector AFTER the sample is in the ring buffer, so the
     * sliding-window logic can simply read the most recent N captures (this
     * delta included) straight from FlowLog. A 0 here resets the window; a
     * non-zero may trip the temporary / permanent valve lock. */
    FlowControl_OnCapture(delta);
}

uint16_t FlowLog_GetWriteIndex(void)
{
    return s_write;
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
