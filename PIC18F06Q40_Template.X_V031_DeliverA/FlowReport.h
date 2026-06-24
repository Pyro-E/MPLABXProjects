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

#endif /* FLOWREPORT_H */
