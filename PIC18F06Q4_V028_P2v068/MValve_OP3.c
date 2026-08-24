/*
 * MValve_OP3.c - OP3 3-wire motorized valve driver implementation.
 * See MValve_OP3.h for the physical model and state definitions.
 */
#include <xc.h>
#include "MValve_OP3.h"
#include "Dev_Led.h"        /* VALVE_PWR_* / VALVE_CTRL_* pin macros */
#include "MCU_Time.h"       /* getNowTime(), timeSpan() */

/* ---- module state ---- */
static valve_motion_t s_motion   = VALVE_MOTION_UNKNOWN;
static uint32_t       s_drive_ms = 0;        /* time the current drive started */
static bool           s_driving  = false;    /* PWR currently high (mid-drive)  */
static bool           s_drive_open = false;  /* direction of current drive      */

#ifdef VALVE_TEST_TOGGLE
static uint32_t       s_test_phase0 = 0;     /* test-toggle master phase ref    */
#endif

/* ---- pin helpers ---- */
static void valve_pins_idle(void)
{
    /* explicit single idle state: both LOW, valve holds position, ~0 current */
    VALVE_PWR_OFF;
    VALVE_CTRL_OFF;
}

/* ============================================================= */

void MValve_OP3_Init(bool start_open)
{
    valve_pins_idle();
    s_driving  = false;
    s_drive_ms = 0;

#if defined(VALVE_PWR_CTRL_ENABLE) && defined(VALVE_TEST_TOGGLE)
    /* test mode: free-running toggle, high-level open/close logic disabled */
    s_test_phase0 = getNowTime();
    s_motion      = VALVE_MOTION_UNKNOWN;
    return;
#endif

#ifdef VALVE_ON_WHEN_STARTUP
    /* caller asks for a real startup open; CmdOpen() will run the cycle */
    (void)start_open;
    s_motion = VALVE_MOTION_UNKNOWN;
#else
    /* no startup drive: treat the initial position per caller's hint.
     * Per spec, when startup-open is NOT compiled in, assume OPEN. */
    s_motion = start_open ? VALVE_MOTION_OPEN : VALVE_MOTION_UNKNOWN;
#endif
}

void MValve_OP3_CmdOpen(void)
{
#if defined(VALVE_PWR_CTRL_ENABLE) && !defined(VALVE_TEST_TOGGLE)
    VALVE_PWR_ON;
    VALVE_CTRL_ON;            /* PWR=H, CTRL=H -> open direction */
    s_driving    = true;
    s_drive_open = true;
    s_drive_ms   = getNowTime();
    s_motion     = VALVE_MOTION_OPENING;
#endif
}

void MValve_OP3_CmdClose(void)
{
#if defined(VALVE_PWR_CTRL_ENABLE) && !defined(VALVE_TEST_TOGGLE)
    VALVE_PWR_ON;
    VALVE_CTRL_OFF;          /* PWR=H, CTRL=L -> close direction */
    s_driving    = true;
    s_drive_open = false;
    s_drive_ms   = getNowTime();
    s_motion     = VALVE_MOTION_CLOSING;
#endif
}

void MValve_OP3_Process(void)
{
#if defined(VALVE_PWR_CTRL_ENABLE) && defined(VALVE_TEST_TOGGLE)
    /* ---- TEST MODE: free-running toggle, ignores open/close logic ----
     * PWR: HIGH for the first APP_VALVE_PWR_HIGH_MS of APP_VALVE_PWR_PERIOD_MS.
     * CTRL: HIGH for the first APP_VALVE_CTRL_HIGH_MS of APP_VALVE_CTRL_PERIOD_MS.
     * Lets you confirm the valve physically opens/closes on the bench. */
    uint32_t e = timeSpan(s_test_phase0);
    if (e >= APP_VALVE_PWR_PERIOD_MS) {
        s_test_phase0 = getNowTime();
        e = 0;
    }
    if (e < APP_VALVE_PWR_HIGH_MS) { VALVE_PWR_ON;  } else { VALVE_PWR_OFF;  }
    if ((e % APP_VALVE_CTRL_PERIOD_MS) < APP_VALVE_CTRL_HIGH_MS) { VALVE_CTRL_ON; }
    else { VALVE_CTRL_OFF; }
    return;
#else
    if (!s_driving) {
        return;
    }

    if (timeSpan(s_drive_ms) >= TIME_VALVE_FULL_TOGGLE_MS) {
        /* full travel done: latch end state, drop power */
        valve_pins_idle();
        s_driving = false;
        s_motion  = s_drive_open ? VALVE_MOTION_OPEN : VALVE_MOTION_CLOSED;
    }
    /* else: keep driving (pins already asserted) */
#endif
}

bool MValve_OP3_IsBusy(void)
{
    return s_driving;
}

valve_motion_t MValve_OP3_GetMotion(void)
{
    return s_motion;
}

uint8_t MValve_OP3_GetPwrPin(void)
{
    return (uint8_t)(LATCbits.LATC2 & 1u);
}

uint8_t MValve_OP3_GetCtrlPin(void)
{
    return (uint8_t)(LATAbits.LATA2 & 1u);
}

/* ---- external interrupt of a drive (e.g. forced sleep / abort) ----
 * If something drops PWR before the full toggle time, the position is
 * unknown-ish: mark it as interrupted so the host can see an incomplete
 * open/close. (Called by Flow_Control / main only if needed.) */
void MValve_OP3_NotifyInterrupted(void)
{
    if (s_driving) {
        valve_pins_idle();
        s_driving = false;
        s_motion  = s_drive_open ? VALVE_MOTION_OPEN_INTERRUPTED
                                 : VALVE_MOTION_CLOSE_INTERRUPTED;
    }
}
