#ifndef FLOWLOG_H
#define FLOWLOG_H

#include <stdint.h>
#include <stdbool.h>
#include "App_Config.h"
#include "Compress.h"   /* COMPRESS_BYTES_PER_SAMPLE, Compress_Pack/Unpack */

/* ============================================================
 *  FlowLog.h  -  periodic capture + ring buffer
 *
 *  Every FLOWLOG_PERIOD_MS this layer reads the running total
 *  from FlowMeter and stores the per-period DIFFERENCE (delta)
 *  into a ring-buffer slot, packed by the selected compression
 *  method. Pure pulse accounting / grand total lives in
 *  FlowMeter, not here.
 *
 *  Stored per slot (2B_2B method):
 *      grp    : number within the 10-sample group (0..BATCH-1)
 *      pulses : pulse delta during that period
 * ============================================================ */

#define FLOWLOG_SLOTS     APP_FLOW_SLOTS
#define FLOWLOG_PERIOD_MS APP_FLOW_PERIOD_MS
#define FLOWLOG_BATCH     APP_FLOW_BATCH

typedef struct {
    uint16_t grp;     /* number within the 10-sample group (0..BATCH-1) */
    uint16_t pulses;  /* pulse delta for that period */
} flowlog_entry_t;

void     FlowLog_Init(void);
void     FlowLog_Process(void);

uint16_t FlowLog_GetWriteIndex(void);
void     FlowLog_GetAt(uint16_t index, flowlog_entry_t *out);

uint32_t FlowLog_GetCaptureCount(void); /* 32-bit total captures (SampleNumber) */

/* True once after a full group of FLOWLOG_BATCH samples has just
 * been captured (the report counter wrapped 0). Reading it clears
 * the flag, so it fires exactly once per completed group. */
bool     FlowLog_BatchReady(void);

#endif /* FLOWLOG_H */
