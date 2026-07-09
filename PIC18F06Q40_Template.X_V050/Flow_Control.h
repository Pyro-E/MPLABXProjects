/*
 * Flow_Control.h - leak detection + valve lock policy (high-level layer).
 *
 * Sits above MValve_OP3. Each capture it is fed the per-interval pulse delta;
 * it maintains a "running" accumulator that RESETS whenever a single capture
 * reads 0 (flow stopped), so run_sum is always the sum over the most recent
 * uninterrupted (no-zero) run of captures.
 *
 *   Alert 1 (temporary): run within leak1_window_n captures reaches
 *                        leak1_counts  -> temporary lock, auto-clears after
 *                        TIME_VALVE_TEMP_LOCK_MS.
 *   Alert 2 (permanent): run within leak2_window_n captures reaches
 *                        leak2_counts  -> permanent lock, cleared only by a
 *                        Photon unlock packet (or SYS_RESET / power cycle).
 *
 * Temporary and permanent locks are INDEPENDENT; the valve is driven CLOSED
 * if EITHER is active, and OPEN only when BOTH are clear.
 */
#ifndef FLOW_CONTROL_H
#define FLOW_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "App_Config.h"

/* ---- Updatable leak parameters (wire: 4 x uint16 big-endian = 8 bytes) ---- */
typedef struct {
    uint16_t leak1_counts;     /* alert-1 threshold counts        (e.g. 100)   */
    uint16_t leak1_window_s;   /* alert-1 window in seconds        (e.g. 480)   */
    uint16_t leak2_counts;     /* alert-2 threshold counts        (e.g. 400)   */
    uint16_t leak2_window_s;   /* alert-2 window in seconds        (e.g. 180)   */
} leak_param_t;

/* ---- lock state bit flags (shared by RSP_VALVE report and UNLOCK command) ---- */
#define VALVE_LOCK_TEMP_BIT   0x01u   /* bit0: temporary lock (alert 1) */
#define VALVE_LOCK_PERM_BIT   0x02u   /* bit1: permanent lock (alert 2) */

void     FlowControl_Init(void);

/* Recompute window_n from the current parameters + capture period.
 * Called at init and after a parameter update. */
void     FlowControl_RecalcDerived(void);

/* Feed one capture's pulse delta. Updates the run accumulator, evaluates both
 * alerts, and drives the valve lock state. Call once per capture. */
void     FlowControl_OnCapture(uint16_t delta);

/* Drive timers (temp-lock auto-clear) and keep the valve in sync. Call every
 * super-loop pass. */
void     FlowControl_Process(void);

/* ---- parameter access (for GET/SET_PARAM packets) ---- */
void     FlowControl_GetParams(leak_param_t *out);
void     FlowControl_SetParams(const leak_param_t *in);   /* also recalcs + re-evals */

/* ---- valve lock status (for RSP_VALVE) ---- */
uint8_t  FlowControl_GetLockFlags(void);     /* TEMP_BIT | PERM_BIT */
uint32_t FlowControl_GetTempLockCount(void); /* cumulative # of temp locks */

/* ---- unlock command (VALVE_UNLOCK packet) ----
 * flags: TEMP_BIT clears temporary lock, PERM_BIT clears permanent lock. */
void     FlowControl_Unlock(uint8_t flags);

#endif /* FLOW_CONTROL_H */
