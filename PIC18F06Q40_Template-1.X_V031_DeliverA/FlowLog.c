#define FLOWLOG_C

#include <xc.h>
#include <stdint.h>
#include "FlowLog.h"
#include "Compress.h"
#include "FlowMeter.h"
#include "MCU_Time.h"

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

void FlowLog_Init(void)
{
    s_write      = 0;
    s_group      = 0;
    s_captures   = 0;
    s_batch_ready = false;
    s_prev_total = FlowMeter_GetTotal();   /* baseline from the meter */
    s_mark_ms    = getNowTime();
}

void FlowLog_Process(void)
{
    if (timeSpan(s_mark_ms) < FLOWLOG_PERIOD_MS) {
        return;
    }
    s_mark_ms = getNowTime();

    /* per-period delta = difference of the 32-bit running total */
    uint32_t cur   = FlowMeter_GetTotal();
    uint16_t delta = (uint16_t)(cur - s_prev_total);
    s_prev_total   = cur;

    s_captures += 1u;

    /* front field = group number (0..BATCH-1), back field = pulses */
    uint16_t slot = (uint16_t)(s_write % FLOWLOG_SLOTS);
    Compress_Pack(s_group, delta,
                  &s_buf[(uint16_t)(slot * COMPRESS_BYTES_PER_SAMPLE)]);

    s_group++;
    if (s_group >= FLOWLOG_BATCH) {
        s_group = 0;
        s_batch_ready = true;    /* a full group of BATCH just completed */
    }

    s_write = (uint16_t)((s_write + 1u) % FLOWLOG_SLOTS);   /* circular */
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
