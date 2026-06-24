#ifndef DEV_VALVE_H
#define DEV_VALVE_H

#include "App_Config.h"

/* ============================================================
 *  Dev_Valve.h  -  periodic valve power/control driver
 *
 *  Pins (defined in Dev_Led.h, active-high):
 *    Valve POWER   = RC2
 *    Valve CONTROL = RA2
 *
 *  Waveform (both derived from ONE master phase, so they never
 *  drift out of sync). Master period = APP_VALVE_PWR_PERIOD_MS:
 *
 *    PWR  : HIGH for APP_VALVE_PWR_HIGH_MS,  then LOW   (10 s / 10 s)
 *    CTRL : HIGH for APP_VALVE_CTRL_HIGH_MS, then LOW   ( 5 s /  5 s)
 *
 *    t(s):  0    5    10   15   20
 *    PWR :  HHHHHHHHHH LLLLLLLLLL
 *    CTRL:  HHHHH LLLLL HHHHH LLLLL
 *
 *  Phase is computed from absolute elapsed time (modulo the period),
 *  NOT by accumulating toggles, so a missed/late call cannot cause
 *  drift, and the pair stays aligned across sleep/wake too.
 *
 *  Gated by VALVE_PWR_CTRL_ENABLE (App_Config.h):
 *    defined   -> this driver runs the waveform.
 *    undefined -> pins stay LOW (set once in LEDs_Init); this
 *                 module compiles to empty stubs.
 * ============================================================ */

/* Reset the phase reference to "now" (call once at startup). */
void Valve_Init(void);

/* Call frequently from main(). Non-blocking: updates the two pins
 * to match the current phase. No effect unless VALVE_PWR_CTRL_ENABLE. */
void Valve_Process(void);

#endif /* DEV_VALVE_H */
