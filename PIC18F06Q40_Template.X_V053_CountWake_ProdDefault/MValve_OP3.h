/*
 * MValve_OP3.h - OP3 3-wire motorized valve driver (GND / PWR / CTRL)
 *
 * Physical model:
 *   PWR = LOW                -> motor off, valve HOLDS its position (latched)
 *   PWR = HIGH, CTRL = HIGH  -> drive OPEN  direction
 *   PWR = HIGH, CTRL = LOW   -> drive CLOSE direction
 *   A full open or close takes TIME_VALVE_FULL_TOGGLE_MS. The motor self-cuts
 *   at the end stop (no overheat), but a small current still flows, so after
 *   the drive time we force BOTH PWR and CTRL LOW to save power and to keep a
 *   single, explicit idle state.
 *
 * This driver is "pure": it only drives the valve and tracks the physical
 * motion state. Leak policy (temporary / permanent lock) lives one layer up
 * in Flow_Control, which calls MValve_OP3_CmdOpen() / _CmdClose().
 *
 * Compile switches (App_Config.h):
 *   VALVE_PWR_CTRL_ENABLE  - master enable for valve driving (else pins held LOW)
 *   VALVE_TEST_TOGGLE      - (with ENABLE) test toggle only, bypassing high-level logic
 *   VALVE_ON_WHEN_STARTUP  - (non-WDT reset) force OPEN at startup
 */
#ifndef MVALVE_OP3_H
#define MVALVE_OP3_H

#include <stdint.h>
#include <stdbool.h>
#include "App_Config.h"

/* Physical motion state of the valve (reported in RSP_VALVE). */
typedef enum {
    VALVE_MOTION_UNKNOWN          = 0u, /* position not yet known (power-up)        */
    VALVE_MOTION_OPEN             = 1u, /* fully open, stopped                      */
    VALVE_MOTION_CLOSED           = 2u, /* fully closed, stopped                    */
    VALVE_MOTION_OPENING          = 3u, /* driving open  (PWR=H, CTRL=H)            */
    VALVE_MOTION_CLOSING          = 4u, /* driving close (PWR=H, CTRL=L)            */
    VALVE_MOTION_OPEN_INTERRUPTED = 5u, /* PWR cut before full toggle -> partly open  */
    VALVE_MOTION_CLOSE_INTERRUPTED= 6u  /* PWR cut before full toggle -> partly closed*/
} valve_motion_t;

/* Initialize pins and motion state.
 *   start_open == true  -> assume/assert OPEN as the initial position
 * (caller decides based on reset cause + VALVE_ON_WHEN_STARTUP). */
void           MValve_OP3_Init(bool start_open);

/* Drive the run-out state machine; call every super-loop pass. Non-blocking:
 * it keeps PWR/CTRL asserted until TIME_VALVE_FULL_TOGGLE_MS elapses, then
 * drops both LOW and latches the final OPEN/CLOSED state. */
void           MValve_OP3_Process(void);

/* Command a full OPEN / CLOSE. Starts a drive cycle (ignored if already in
 * the requested end state and not mid-drive). */
void           MValve_OP3_CmdOpen(void);
void           MValve_OP3_CmdClose(void);

/* True while a drive cycle is running (PWR high). Used to inhibit Sleep. */
bool           MValve_OP3_IsBusy(void);

/* Status accessors (for RSP_VALVE). */
valve_motion_t MValve_OP3_GetMotion(void);
uint8_t        MValve_OP3_GetPwrPin(void);    /* current PWR pin level 0/1  */
uint8_t        MValve_OP3_GetCtrlPin(void);   /* current CTRL pin level 0/1 */

/* Abort an in-progress drive (drops pins, marks OPEN/CLOSE_INTERRUPTED). */
void           MValve_OP3_NotifyInterrupted(void);

#endif /* MVALVE_OP3_H */
