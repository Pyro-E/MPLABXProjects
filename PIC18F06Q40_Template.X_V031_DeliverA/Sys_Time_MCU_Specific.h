#ifndef SYS_TIME_MCU_SPECIFIC_H
#define SYS_TIME_MCU_SPECIFIC_H

#include "App_Config.h"

/* ============================================================
 *  Sys_Time_MCU_Specific.h
 *    MCU-dependent layer for the system time base (PIC18-Q40).
 *
 *  Generates a 1 ms tick using Timer2 (PR2 auto-reload).
 *  Two ways to advance the time counter -- pick ONE below:
 *
 *    SYS_TIME_USE_ISR defined  -> Timer2 ISR advances the time
 *    SYS_TIME_USE_ISR undefined -> main() must call
 *                                  MCU_Time_Process() frequently,
 *                                  which polls the Timer2 flag.
 *
 *  Either way, the timer hardware sets the 1 ms flag on its own,
 *  so a slightly late poll does NOT accumulate drift: missed
 *  flags are caught on the next pass (as long as main loops
 *  faster than the catch-up logic span).
 * ============================================================ */

/* Method selected centrally in App_Config.h (APP_SYS_TIME_USE_ISR). */
#ifdef APP_SYS_TIME_USE_ISR
  #define SYS_TIME_USE_ISR
#endif

/* Set up Timer2 for a 1 ms period and (if ISR mode) enable its interrupt. */
void Sys_Time_Init(void);

/* Restart the Timer2 1 ms tick WITHOUT resetting the ms counter.
 * Used to hand the time base back to Timer2 after a deep-sleep wake. */
void Sys_Time_ResumeTick(void);

/* What woke the MCU from deep sleep. */
typedef enum {
    WAKE_BY_TIMER0 = 0,   /* capture period elapsed                 */
    WAKE_BY_UART          /* RX edge (Photon2's 0xF0 request)       */
} wake_cause_t;

/* Enter full Sleep until either Timer0 (one capture period) or, if
 * APP_UART_WAKE_ENABLE, a UART RX edge wakes the MCU. Returns the cause.
 * On a Timer0 wake the ms clock is advanced by APP_FLOW_PERIOD_MS.
 * Timer1 pulse counting keeps running through Sleep (asynchronous). */
wake_cause_t Sys_Time_EnterDeepSleep(void);

/* Polling entry point. Call as often as possible from main().
 * In ISR mode this is a harmless no-op so the same main() works either way. */
void MCU_Time_Process(void);

/* Called from the global ISR (only relevant in ISR mode). */
void Sys_Time_ISR(void);

#endif /* SYS_TIME_MCU_SPECIFIC_H */
