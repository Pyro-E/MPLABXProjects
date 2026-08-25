#ifndef LED_FSM_SYSSTATE_H
#define LED_FSM_SYSSTATE_H

#include <stdint.h>
#include <stdbool.h>
#include "App_Config.h"

/* ============================================================
 *  led_fsm_sysstate.h  -  TEST LED (RC3) behaviour state machine
 *
 *  V028: the two behaviours were re-anchored on what the LED is actually
 *  meant to TELL AN OBSERVER, because the previous anchors could not be
 *  seen on real hardware.
 *
 *  ---- (1) PHOTON-SESSION blink  (HIGHEST priority) ----------------------
 *
 *  While the PIC holds the Photon powered (RC4 LOW) the LED free-runs at
 *  APP_LED_PHOTON_HALF_MS ON / OFF, starting ON, and is cut off the moment
 *  power is removed. A 10 s session therefore shows 3-3-3-1, a 13 s session
 *  3-3-3-3-1.
 *
 *  Why "Photon powered" and not "RSP_DATA streaming": powering the Photon IS
 *  the data exchange - that is the whole reason the PIC turns it on. The
 *  RSP_DATA frame itself is far too short to ever complete one half-period:
 *
 *      bench   n=34    98 byte @9600 =  102 ms
 *      prod    n=718 1466 byte @9600 = 1527 ms
 *      worst   n=1000 2030 byte @9600 = 2115 ms      (half-period is 3000 ms)
 *
 *  Anchored on the frame the LED could only ever flash once and never toggle,
 *  so the 3 s value would never appear. Anchored on the session it is plainly
 *  visible for the whole ~23 s exchange, and for the full 10-minute initial
 *  hold (the Photon is powered throughout it).
 *
 *  ---- (2) CAPTURE blink  (LOWER priority) -------------------------------
 *
 *  While the Photon is OFF the PIC sleeps and wakes only to put one sample
 *  into the capture FIFO. That write is the natural unit of "the board is
 *  alive", so the LED is lit on the wake that performs it and cleared as the
 *  PIC goes back to sleep.
 *
 *      APP_LED_CAPTURE_EVERY_N   0 = no capture blink at all (the board is
 *                                    then only observable by its reports)
 *                                1 = every FIFO write        (default)
 *                                2 = every 2nd, and so on
 *
 *      APP_LED_CAPTURE_HOLD_MS   0 = no extra hold (default): the LED is on
 *                                    only for the natural awake processing,
 *                                    then the PIC sleeps immediately
 *                                >0 = stay awake at least this long so the
 *                                    blink is actually visible. ~5 ms was
 *                                    found to be the practical threshold.
 *
 *  With EVERY_N=2 only the lit cycles pay the hold; the others sleep at once.
 *
 *  This never runs while the Photon is powered: (1) owns the LED there, and
 *  a capture that falls inside a session is simply not signalled.
 *
 *  ---- Boot toggle -------------------------------------------------------
 *
 *  main() flashes the LED five times at reset before this FSM starts. That is
 *  not in the contract but it is the first proof an operator gets that the
 *  part booted and is executing, so it stays.
 * ============================================================ */

void LedFsm_Init(void);

/* Call frequently from main().
 *   photon_powered : true while the PIC holds the Photon's supply on
 *                    (i.e. s_pwr != PWR_SLEEP). Owns the LED when true.
 *   capture_count  : FlowLog_GetCaptureCount() - monotonic FIFO-write tick.
 */
void LedFsm_Process(bool photon_powered, uint32_t capture_count);

/* Called from the sleep branch immediately before Sys_Time_EnterDeepSleep().
 *
 * Returns true while the capture blink still needs the PIC awake (only
 * possible when APP_LED_CAPTURE_HOLD_MS > 0). main must keep looping until
 * it returns false, at which point the LED has been turned off and sleeping
 * is safe. With HOLD_MS = 0 it turns the LED off and returns false at once,
 * so the blink lasts exactly one awake pass.
 */
bool LedFsm_SleepGate(void);

/* Kept for source compatibility with existing call sites; now no-ops. */
void LedFsm_NotifyDataCycle(void);
void LedFsm_NotifyWake(void);

#endif /* LED_FSM_SYSSTATE_H */
