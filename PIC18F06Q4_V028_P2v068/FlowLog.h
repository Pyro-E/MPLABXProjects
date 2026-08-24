#ifndef FLOWLOG_H
/* ============================================================
 *  FlowLog.h  -  the flow record as a single ring FIFO.
 *
 *  MODEL (one ring, two owners):
 *    PUSH side  = FlowLog     : FlowLog_Process() captures one sample per
 *                 capture period and PUSHES it (advances write). On every
 *                 push it enforces overrun: if the backlog reaches
 *                 (SLOTS - MARGIN) it makes the POP side drop its oldest
 *                 sample, so write can never lap read -> write==read = EMPTY.
 *    POP side   = FlowReport  : owns read/tx_read, POPs on transmit and
 *                 COMMITS (advances read) only on the Photon ACK
 *                 (consume-on-ACK). See FlowReport.{h,c}.
 *
 *  Each slot = COMPRESS_BYTES_PER_SAMPLE bytes (16-bit pulses, MSB first).
 *  Running totals live in FlowMeter (absolute) so a dropped sample never
 *  loses the flow TOTAL - only that sample's time-distribution.
 * ============================================================ */

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
#define FLOWLOG_PERIOD_MS APP_CAPTURE_PERIOD_MS
#define FLOWLOG_BATCH     APP_FLOW_BATCH

typedef struct {
    uint16_t grp;     /* number within the 10-sample group (0..BATCH-1) */
    uint16_t pulses;  /* pulse delta for that period */
} flowlog_entry_t;

void     FlowLog_Init(void);
void     FlowLog_Process(void);

uint16_t FlowLog_GetWriteIndex(void);
void     FlowLog_GetAt(uint16_t index, flowlog_entry_t *out);
void     FlowLog_GetRawAt(uint16_t index, uint8_t *dst); /* raw packed bytes */

uint32_t FlowLog_GetCaptureCount(void); /* 32-bit total captures (SampleNumber) */

/* V023 / fix 3: the three values that define one capture boundary, read as a
 * set. Use this - never FlowLog_GetWriteIndex() + FlowLog_GetCaptureCount() +
 * FlowMeter_GetTotal() separately - when building a report snapshot.
 *   write           : ring write index after the last completed capture
 *   captures        : total captures completed since boot
 *   completed_total : pulse total AS OF that last completed capture
 *                     (the capture in progress is NOT included) */
typedef struct {
    uint16_t write;
    uint32_t captures;
    uint32_t completed_total;
} flowlog_snapshot_t;

void     FlowLog_GetSnapshot(flowlog_snapshot_t *s);

/* Extended-report support: overflow flags and report-due snapshots. */
uint16_t FlowLog_CountOverflows(uint16_t from, uint16_t to); /* clamps in [from,to) */
uint32_t FlowLog_GetTotalAtDue(void);   /* running total at last report-due boundary */
uint16_t FlowLog_GetWriteAtDue(void);   /* write index at last report-due boundary   */
bool     FlowLog_DueValid(void);        /* has a report-due boundary happened yet?    */

/* True once after a full group of FLOWLOG_BATCH samples has just
 * been captured (the report counter wrapped 0). Reading it clears
 * the flag, so it fires exactly once per completed group. */
bool     FlowLog_BatchReady(void);
void     FlowLog_SetNextBatch(uint16_t captures); /* one-time phase-align period */

#endif /* FLOWLOG_H */
