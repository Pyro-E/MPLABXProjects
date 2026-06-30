#ifndef FLOWREPORT_H
#define FLOWREPORT_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 *  FlowReport.h  -  sequential, non-blocking batch transmitter
 *
 *  The UART TX ring buffer is only 64 bytes, so a 10-line batch
 *  cannot be dumped at once. This layer sends ONE line at a time:
 *  it builds a line, and only commits it when the UART buffer has
 *  room for the whole line; otherwise it waits and retries next
 *  call. The background TX ISR drains the bytes meanwhile.
 *
 *  Owns the READ index (consumer) and the SEND MARK. A new batch
 *  of FLOWLOG_BATCH lines starts once the writer has moved
 *  FLOWLOG_BATCH samples past the last send mark.
 *
 *  Line: "Sample-<readIdx>-<seqInBatch> : Pulse-<n>   seq=<seq>"
 * ============================================================ */

void FlowReport_Init(void);

/* Call when the host (Photon2) sends 0xAA: requests an upload of all
 * samples accumulated up to this moment. Safe to call from an ISR. */
void FlowReport_NotifyAA(void);

/* Request a WAKE pulse + report cycle even when the batch is not full.
 * Called when Photon2 wants to talk (0xF0 received while awake, or a UART
 * edge woke the PIC from sleep). The PIC raises WAKE and waits for 0xAA. */
void FlowReport_RequestReport(void);

/* Call often from main(). Advances the sequential send as UART
 * space allows. Non-blocking. */
void FlowReport_Process(void);

/* True while a WAKE pulse or a report send is in progress (or a 0xAA
 * is pending). Used to decide it is NOT safe to enter deep sleep. */
bool FlowReport_IsBusy(void);

/* True ONLY while an RSP_DATA transfer is actually in progress (the SEND
 * machine is mid-stream). Used to gate incoming requests: a new request is
 * ignored only while we are still answering a previous one. Unlike
 * FlowReport_IsBusy(), this does NOT include the WAKE-wait / report-pending
 * states, so a REQ_DATA that arrives right after WAKE goes HIGH (the whole
 * point of raising WAKE) is accepted instead of being dropped. */
bool FlowReport_IsSending(void);

/* True once when a report period is due (batch ready / requested). main polls
 * this and raises the WAKE line; reading it consumes the flag. */
bool FlowReport_WakeDuePending(void);

/* ---- small response packets (built whole, pushed at once) ---- */
void FlowReport_SendParam(void);            /* RSP_PARAM : 4 x u16 (8B)   */
void FlowReport_SendValve(void);            /* RSP_VALVE : 8B status       */
void FlowReport_SendAck(uint8_t echoed_func);
void FlowReport_SendNak(uint8_t reason);

#endif /* FLOWREPORT_H */
