#ifndef LED_FSM_SYSSTATE_H
#define LED_FSM_SYSSTATE_H

#include "App_Config.h"

/* ============================================================
 *  led_fsm_sysstate.h  -  LED behavior state machine
 *
 *  Two modes for the TEST LED (RC3):
 *    HEARTBEAT : toggle every APP_LED_HEARTBEAT_MS (0.5 s) - shows
 *                the MCU is alive and running.
 *    BLINK     : on a data-cycle event, blink APP_LED_BLINK_COUNT
 *                times at APP_LED_BLINK_MS per state, then return
 *                to HEARTBEAT automatically.
 *
 *  Non-blocking: call LedFsm_Process() often from main().
 *  The report FSM calls LedFsm_NotifyDataCycle() when a data
 *  exchange cycle begins.
 * ============================================================ */

void LedFsm_Init(void);

/* Call frequently from main(). Drives the current LED mode. */
void LedFsm_Process(void);

/* Trigger the data-cycle blink burst (called by the report FSM). */
void LedFsm_NotifyDataCycle(void);

/* Called right after waking from deep sleep: LED ON and restart the
 * heartbeat phase, so the LED is on (and visible) for a full half-period
 * before the next toggle. */
void LedFsm_NotifyWake(void);

#endif /* LED_FSM_SYSSTATE_H */
