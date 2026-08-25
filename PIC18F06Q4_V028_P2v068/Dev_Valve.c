#define DEV_VALVE_C

#include <xc.h>
#include <stdint.h>
#include "Dev_Valve.h"
#include "Dev_Led.h"
#include "MCU_Time.h"

/* ============================================================
 *  Both signals are derived from one master phase so they stay
 *  perfectly in sync. See Dev_Valve.h for the waveform diagram.
 *
 *  Whole body is active only when VALVE_PWR_CTRL_ENABLE is set;
 *  otherwise these are empty stubs (pins were left LOW by
 *  LEDs_Init) and cost nothing.
 * ============================================================ */

#ifdef VALVE_PWR_CTRL_ENABLE

static uint32_t s_phase0 = 0;   /* time reference: start of a master period */

void Valve_Init(void)
{
    s_phase0 = getNowTime();
    VALVE_PWR_OFF;
    VALVE_CTRL_OFF;
}

void Valve_Process(void)
{
    /* elapsed time folded into one master (PWR) period */
    uint32_t e = timeSpan(s_phase0);

    /* keep the reference within one period so the 32-bit value
     * never needs to grow without bound (purely cosmetic; the
     * modulo below would handle wrap anyway). */
    if (e >= APP_VALVE_PWR_PERIOD_MS) {
        s_phase0 = getNowTime();
        e = 0u;
    }

    /* ---- POWER : HIGH for the first APP_VALVE_PWR_HIGH_MS ---- */
    if (e < APP_VALVE_PWR_HIGH_MS) {
        VALVE_PWR_ON;
    } else {
        VALVE_PWR_OFF;
    }

    /* ---- CONTROL : same start, but HIGH for the first half of
     *      each APP_VALVE_CTRL_PERIOD_MS sub-period ---- */
    {
        uint32_t c = e % APP_VALVE_CTRL_PERIOD_MS;
        if (c < APP_VALVE_CTRL_HIGH_MS) {
            VALVE_CTRL_ON;
        } else {
            VALVE_CTRL_OFF;
        }
    }
}

#else  /* VALVE_PWR_CTRL_ENABLE not defined -> empty stubs */

void Valve_Init(void)    { /* pins left LOW by LEDs_Init */ }
void Valve_Process(void) { /* nothing */ }

#endif /* VALVE_PWR_CTRL_ENABLE */
