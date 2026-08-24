/* ============================================================
 *  led_fsm_sysstate.c  -  TEST LED (RC3) behaviour state machine
 *  See led_fsm_sysstate.h for the rationale behind each anchor.
 * ============================================================ */

#define LED_FSM_SYSSTATE_C

#include <xc.h>            /* SFR definitions (LATCbits) used by Dev_Led.h */
#include <stdint.h>
#include <stdbool.h>
#include "led_fsm_sysstate.h"
#include "Dev_Led.h"
#include "MCU_Time.h"

/* ---- (1) Photon-session blink ---- */
static bool     s_ph_was_on   = false;  /* Photon was powered last pass       */
static bool     s_ph_led_on   = false;  /* current level of the 3 s toggle    */
static uint32_t s_ph_mark     = 0;      /* start of the current half-period   */

/* ---- (2) Capture blink ---- */
static uint32_t s_last_capture = 0;     /* last capture_count acted on        */
static bool     s_cap_led_on   = false; /* a capture blink is currently lit   */
static uint32_t s_cap_mark     = 0;     /* when it was lit                    */

void LedFsm_Init(void)
{
    s_ph_was_on    = false;
    s_ph_led_on    = false;
    s_ph_mark      = 0;
    s_last_capture = 0;
    s_cap_led_on   = false;
    s_cap_mark     = 0;
    LED_TEST_OFF;
}

void LedFsm_Process(bool photon_powered, uint32_t capture_count)
{
    /* -------- (1) PHOTON SESSION: highest priority -------- */
    if (photon_powered) {
        if (!s_ph_was_on) {
            /* Session just started: begin ON with a fresh half-period. Any
             * capture blink in flight is abandoned - (1) owns the LED here. */
            s_ph_was_on  = true;
            s_ph_led_on  = true;
            s_ph_mark    = getNowTime();
            s_cap_led_on = false;
            LED_TEST_ON;
        } else if (timeSpan(s_ph_mark) >= APP_LED_PHOTON_HALF_MS) {
            s_ph_mark   = getNowTime();
            s_ph_led_on = !s_ph_led_on;
            if (s_ph_led_on) { LED_TEST_ON; } else { LED_TEST_OFF; }
        }
        /* Keep the capture baseline current so captures that happened during
         * the session do not all fire at once when it ends. */
        s_last_capture = capture_count;
        return;
    }

    if (s_ph_was_on) {
        /* Session ended (power cut): the pattern is truncated here, exactly as
         * the observer sees it - e.g. 3-3-3-1 for a 10 s session. */
        s_ph_was_on    = false;
        s_ph_led_on    = false;
        s_last_capture = capture_count;
        LED_TEST_OFF;
    }

    /* -------- (2) CAPTURE blink: Photon is off -------- */
#if (APP_LED_CAPTURE_EVERY_N > 0u)
    if (capture_count != s_last_capture) {
        s_last_capture = capture_count;
        if ((capture_count != 0u) &&
            ((capture_count % (uint32_t)APP_LED_CAPTURE_EVERY_N) == 0u)) {
            s_cap_led_on = true;
            s_cap_mark   = getNowTime();
            LED_TEST_ON;
        }
    }
#else
    s_last_capture = capture_count;      /* blink disabled: baseline only */
#endif
}

bool LedFsm_SleepGate(void)
{
    if (!s_cap_led_on) {
        LED_TEST_OFF;                    /* nothing lit: safe to sleep */
        return false;
    }
#if (APP_LED_CAPTURE_HOLD_MS > 0u)
    if (timeSpan(s_cap_mark) < (uint32_t)APP_LED_CAPTURE_HOLD_MS) {
        return true;                     /* stay awake so the blink is seen */
    }
#endif
    s_cap_led_on = false;
    LED_TEST_OFF;
    return false;
}

void LedFsm_NotifyDataCycle(void) { /* no-op */ }
void LedFsm_NotifyWake(void)      { /* no-op */ }
