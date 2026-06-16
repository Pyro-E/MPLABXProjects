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

/* Polling entry point. Call as often as possible from main().
 * In ISR mode this is a harmless no-op so the same main() works either way. */
void MCU_Time_Process(void);

/* Called from the global ISR (only relevant in ISR mode). */
void Sys_Time_ISR(void);

#endif /* SYS_TIME_MCU_SPECIFIC_H */
