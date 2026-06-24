#define LED_FSM_SYSSTATE_C

#include <xc.h>
#include <stdint.h>
#include "led_fsm_sysstate.h"
#include "Dev_Led.h"
#include "MCU_Time.h"

/* ============================================================
 *  LED modes
 * ============================================================ */
typedef enum {
    LED_HEARTBEAT = 0,   /* slow toggle: MCU alive */
    LED_BLINK            /* fast blink burst on data cycle */
} led_mode_t;

static led_mode_t s_mode    = LED_HEARTBEAT;
static uint32_t   s_mark    = 0;   /* time reference for current step */
static uint8_t    s_toggles = 0;   /* remaining toggles in a blink burst */

void LedFsm_Init(void)
{
    s_mode    = LED_HEARTBEAT;
    s_mark    = getNowTime();
    s_toggles = 0;
    LED_TEST_OFF;
}

/* ---- called by the report FSM when a data cycle starts ---- */
void LedFsm_NotifyDataCycle(void)
{
    /* start the blink burst: LED on now, then toggle the rest.
     * COUNT blinks need (2*COUNT - 1) more toggles after this ON,
     * ending in the OFF state. */
    s_mode    = LED_BLINK;
    LED_TEST_ON;
    s_toggles = (uint8_t)(2u * APP_LED_BLINK_COUNT - 1u);
    s_mark    = getNowTime();
}

void LedFsm_Process(void)
{
    if (s_mode == LED_HEARTBEAT) {
        if (timeSpan(s_mark) >= APP_LED_HEARTBEAT_MS) {
            s_mark = getNowTime();
            LED_TEST_TOGGLE;
        }
        return;
    }

    /* LED_BLINK */
    if (timeSpan(s_mark) >= APP_LED_BLINK_MS) {
        s_mark = getNowTime();
        LED_TEST_TOGGLE;
        if (s_toggles > 0) {
            s_toggles--;
        }
        if (s_toggles == 0) {
            /* burst finished (LED now OFF) -> back to heartbeat */
            LED_TEST_OFF;
            s_mode = LED_HEARTBEAT;
            s_mark = getNowTime();
        }
    }
}
