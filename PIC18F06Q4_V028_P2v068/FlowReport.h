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
void FlowReport_NotifyAck(void);   /* Photon PKT_DATA_RECEIVED -> commit (consume-on-ACK) */
/* V024: an ACK that names the batch it releases by its position on the PIC's
 * lifetime pulse axis. Commits only if BOTH endpoints match the batch we are
 * actually holding, so a delayed or duplicated 0x0B can never make the PIC drop
 * water the Photon never stored. Used when the Photon echoes 8 bytes. */
void FlowReport_NotifyAckSpan(uint32_t ackEndPul, uint32_t ackEndCap);

/* V023: a report's data boundary is decided ONCE per session.
 * main.c calls Freeze immediately before powering the Photon on, and Clear
 * immediately after powering it off. An un-ACKed batch survives Clear and is
 * retransmitted, unchanged, in the next session. */
void FlowReport_FreezeSessionCutoff(void);
void FlowReport_ClearSessionCutoff(void);
/* V024b: PIC-local ms at which the current session's cutoff was frozen. The
 * TIME_SYNC handler uses it to measure how far the cloud time it just received
 * lags behind the boundary that time is supposed to describe. */
uint32_t FlowReport_GetCutoffMs(void);

/* FIFO read-side (POP) accessors, used by the PUSH side (FlowLog) to enforce
 * overrun on every capture. The flow record is one ring FIFO: FlowLog pushes
 * (write), FlowReport pops (read). */
uint16_t FlowReport_GetReadIndex(void);
uint16_t FlowReport_GetUsed(void);
void     FlowReport_DropOldest(void);

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
void FlowReport_ClearWakeDue(void);

/* ---- small response packets (built whole, pushed at once) ---- */
void FlowReport_SendParam(void);            /* RSP_PARAM : 4 x u16 (8B)   */
void FlowReport_SendValve(void);            /* RSP_VALVE : 8B status       */
void FlowReport_SendAck(uint8_t echoed_func);
void FlowReport_SendTimeReceived(void);     /* PKT_TIME_RECEIVED (0x0D) reply */

/* Set the RSP_DATA header fields that come from the time/LOCO system:
 *   start_time  = epoch at which THIS report's data span began (0 if unknown)
 *   start_valid = 1 when start_time is a real cloud-derived time, else 0
 *   interval_ms = the capture interval the PIC intends (nominal dt adjusted by
 *                 LOCO k). Call once per report before FlowReport streams it. */
void FlowReport_SetReportMeta(uint32_t start_time, uint8_t start_valid,
                              uint32_t interval_ms);

/* Enter/leave initial-boot mode: while true, every REQ_DATA is answered with a
 * zero-sample report and the captured buffer is wiped (design section 3.1). */
void FlowReport_SetInitialMode(bool initial);

/* RSP_LOCO (0x8A): send the 10-byte LOCO calibration status for bench readout. */
void FlowReport_SendLocoStatus(void);
void FlowReport_SendNak(uint8_t reason);
void FlowReport_SendPowerState(uint8_t state);  /* RSP_POWER_STATE : 1B (0/1) */
void FlowReport_SendPhotonCfg(void);            /* RSP_PHOTON_CFG : 13B config */

/* ---- V022 / Appendix H.7.2: small-packet TX drop accounting ----
 * A small response frame is discarded when the packet-UART TX buffer has no
 * room for it. The Photon then sees only "no valid reply", which it cannot tell
 * apart from a timeout or a CRC error. These accessors expose the count so the
 * session can report it; main() prints it on the [PWR ] Photon OFF line, where
 * it MUST be 0 in a healthy run. Clear it when the Photon is powered up so the
 * figure is per-session. */
uint16_t FlowReport_GetTxDrops(void);
void     FlowReport_ClearTxDrops(void);

#endif /* FLOWREPORT_H */