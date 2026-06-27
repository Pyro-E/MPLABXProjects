#define SYS_TIME_MCU_SPECIFIC_C

#include <xc.h>
#include <stdint.h>
#include "App_Config.h"
#include "Sys_Time_MCU_Specific.h"
#include "MCU_Time.h"
#include "Dev_Uart.h"

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

    /* configure + start the Timer2 1 ms tick */
    Sys_Time_ResumeTick();
}

/* ============================================================
 *  (Re)configure and start the Timer2 1 ms tick. Does NOT touch
 *  the ms counter, so it is safe to call after a deep-sleep wake
 *  to hand the time base back from Timer0 to Timer2.
 * ============================================================ */
void Sys_Time_ResumeTick(void)
{
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
 *  Deep-sleep wake using Timer0 (8-bit) from LFINTOSC.
 *    clock source : LFINTOSC (~31 kHz)
 *    prescaler    : 1:16384  -> 0.528 s / count
 *    period       : TMR0H = APP_FLOW_TMR0H  (derived from
 *                   APP_FLOW_PERIOD_MS in App_Config.h)
 *  Timer0 keeps running in Sleep and wakes the CPU on match.
 * ============================================================ */
#ifdef APP_SLEEP_ENABLE
wake_cause_t Sys_Time_EnterDeepSleep(void)
{
    wake_cause_t cause = WAKE_BY_TIMER0;

    /* hand the time base off Timer2 (it stops in Sleep anyway) */
    T2CONbits.TMR2ON = 0;
    PIE3bits.TMR2IE  = 0;
    PIR3bits.TMR2IF  = 0;

    /* arm Timer0: 8-bit, LFINTOSC, prescaler auto-selected from the capture
     * period (>=500 ms -> 1:16384; shorter -> smaller prescaler). */
    T0CON0bits.T0EN = 0;        /* off while configuring */
    /* T0CON1: T0CS=100 (LFINTOSC), T0ASYNC=1, T0CKPS = APP_FLOW_T0CKPS */
    T0CON1 = APP_FLOW_T0CON1;
    TMR0H  = APP_FLOW_TMR0H;    /* 8-bit period -> one capture interval */
    TMR0L  = 0x00u;
    PIR3bits.TMR0IF = 0;
    PIE3bits.TMR0IE = 1;        /* enable so the match can wake from Sleep */
    /* T0CON0: T0EN=1, T016BIT=0 (8-bit), T0OUTPS=0000 (1:1) */
    T0CON0 = 0x80u;

#ifdef APP_UART_WAKE_ENABLE
    UART_WakeArm();             /* a Photon2 0xF0 edge on RX also wakes us */
#endif

    /* Sleep with GIE cleared: the enabled sources still WAKE the CPU, but
     * do NOT vector to the ISR, so their flags survive for us to inspect
     * here (and no ISR re-entry loop). */
    INTCON0bits.GIE   = 0;
    CPUDOZEbits.IDLEN = 0;      /* full Sleep (Timer1 keeps counting)       */
    WDT_STOP();                 /* disable WDT so a long Sleep never trips it */
    SLEEP();
    NOP();
    /* --- woke here, flags intact --- */
    WDT_START();                /* re-arm WDT to guard the awake period      */

#ifdef APP_UART_WAKE_ENABLE
    if (UART_WokeByEdge()) {
        cause = WAKE_BY_UART;   /* Photon2 sent 0xF0                        */
    }
    UART_WakeDisarm();
#endif

    if (PIR3bits.TMR0IF) {
        MCU_Time_Advance(APP_FLOW_PERIOD_MS);  /* one capture period elapsed */
        /* a UART wake takes priority for "what to do next", but the time
         * still advanced, so keep cause = WAKE_BY_UART if it was set */
    }

    /* stop Timer0 and return the time base to Timer2 */
    T0CON0bits.T0EN = 0;
    PIE3bits.TMR0IE = 0;
    PIR3bits.TMR0IF = 0;

    INTCON0bits.GIE = 1;        /* normal interrupts back on                */

    Sys_Time_ResumeTick();
    return cause;
}
#else
wake_cause_t Sys_Time_EnterDeepSleep(void) { return WAKE_BY_TIMER0; }
#endif

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
    /* Timer0 deep-sleep wake is handled inline in Sys_Time_EnterDeepSleep()
     * (it sleeps with GIE=0, so the wake never vectors here). */
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
