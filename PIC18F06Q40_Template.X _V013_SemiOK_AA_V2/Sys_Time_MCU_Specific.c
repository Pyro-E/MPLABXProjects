#define SYS_TIME_MCU_SPECIFIC_C

#include <xc.h>
#include <stdint.h>
#include "Sys_Time_MCU_Specific.h"
#include "MCU_Time.h"

/* ============================================================
 *  PIC18-Q40 1 ms time base using Timer2 (PR2 auto-reload).
 *
 *  Clock math @ Fosc = 64 MHz:
 *    Timer2 clock = FOSC/4          = 16 MHz   (62.5 ns)
 *    prescaler 1:64                 = 250 kHz  (4 us)
 *    PR2 = 249  -> (249+1) counts   = 250 * 4us = 1000 us = 1 ms
 *
 *  The hardware sets TMR2IF every 1 ms on its own and auto-reloads,
 *  so the *period* is exact regardless of when software services it.
 *  Servicing a little late only delays when the counter is bumped;
 *  it does not shorten or lengthen the 1 ms periods -> no drift.
 * ============================================================ */

void Sys_Time_Init(void)
{
    /* reset the portable ms counter first */
    MCU_Time_Init(0);

    /* ---- Timer2: 1 ms free-running period ---- */
    T2CONbits.TMR2ON = 0;       /* off while configuring */

    T2CLKCON = 0x01;            /* CS = FOSC/4 */
    T2HLT    = 0x00;            /* Free-Running Period mode, sync, software gate */
    T2PR     = 249;             /* period: (249+1) ticks */
    /* T2CON: ON=1, CKPS=110 (1:64), OUTPS=0000 (1:1) */
    T2CON    = 0xE0;

    PIR3bits.TMR2IF = 0;

#ifdef SYS_TIME_USE_ISR
    PIE3bits.TMR2IE = 1;        /* tick advanced inside the ISR */
#else
    PIE3bits.TMR2IE = 0;        /* tick advanced by polling in MCU_Time_Process() */
#endif

    T2CONbits.TMR2ON = 1;       /* start */
}

/* ============================================================
 *  ISR mode: called from the global __interrupt().
 *  Polling mode: this does nothing useful (TMR2IE is off).
 * ============================================================ */
void Sys_Time_ISR(void)
{
#ifdef SYS_TIME_USE_ISR
    if (PIE3bits.TMR2IE && PIR3bits.TMR2IF) {
        PIR3bits.TMR2IF = 0;          /* clear -> next 1 ms will set it again */
        MCU_Time_Increase_Unit();     /* +1 ms */
    }
#endif
}

/* ============================================================
 *  Polling mode: call this as often as possible from main().
 *  ISR mode: harmless no-op, so the SAME main() works both ways.
 *
 *  Note: with a single hardware flag we can only detect "at least
 *  one 1 ms elapsed", not how many. So in polling mode main must
 *  loop at least once per millisecond (avoid long blocking calls).
 *  In ISR mode there is no such constraint.
 * ============================================================ */
void MCU_Time_Process(void)
{
#ifndef SYS_TIME_USE_ISR
    if (PIR3bits.TMR2IF) {
        PIR3bits.TMR2IF = 0;
        MCU_Time_Increase_Unit();     /* +1 ms */
    }
#endif
}
