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

/* Extended-report support: overflow flags and report-due snapshots. */
uint16_t FlowLog_CountOverflows(uint16_t from, uint16_t to); /* clamps in [from,to) */
uint32_t FlowLog_GetTotalAtDue(void);   /* running total at last report-due boundary */
uint16_t FlowLog_GetWriteAtDue(void);   /* write index at last report-due boundary   */
bool     FlowLog_DueValid(void);        /* has a report-due boundary happened yet?    */

/* True once after a full group of FLOWLOG_BATCH samples has just
 * been captured (the report counter wrapped 0). Reading it clears
 * the flag, so it fires exactly once per completed group. */
bool     FlowLog_BatchReady(void);

/* Re-anchor the report-due countdown: the NEXT report fires after exactly
 * 'remainingCaptures' more captures (clamped to [1, FLOWLOG_BATCH]), instead
 * of whenever the in-progress group would otherwise complete. Used by the
 * Photon (REQ_SET_SCHEDULE) to align reports to a user-chosen wall-clock
 * hour, since the PIC itself has no RTC. Only changes the FUTURE boundary;
 * does not retroactively trigger FlowLog_BatchReady(). */
void     FlowLog_SetGroupCountdown(uint16_t remainingCaptures);

#endif /* FLOWLOG_H */
