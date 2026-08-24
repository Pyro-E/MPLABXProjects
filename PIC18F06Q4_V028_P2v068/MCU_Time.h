#ifndef MCU_TIME_H
#define MCU_TIME_H

#include <stdint.h>

/* ============================================================
 *  MCU_Time.h  -  portable ms time base (PIC18-Q40 adaptation)
 *
 *  Same API as the original portable module:
 *    getNowTime()        - current time in ms (free-running)
 *    timeSpan(old)       - elapsed ms since 'old', wrap-safe
 *    MCU_Time_Delay_Ms() - blocking delay built on the counter
 *    MCU_Time_Increase_Unit() - +1 ms, called by the tick source
 *
 *  Key adaptation vs the STM32 version: the counter is uint32_t
 *  (on XC8/PIC18 'unsigned int' is only 16-bit). 32-bit ms wraps
 *  after ~49.7 days; timeSpan() handles the wrap correctly.
 * ============================================================ */

void     MCU_Time_Init(unsigned char ucKHz);   /* ucKHz kept for API compat (unused) */
void     MCU_Time_Increase_Unit(void);          /* call once per 1 ms tick */
void     MCU_Time_Advance(uint32_t unMs);       /* bulk advance (e.g. after a sleep period) */

uint32_t getNowTime(void);                       /* current ms */
uint32_t timeSpan(uint32_t unOldTime_ms);        /* elapsed ms, wrap-safe */
void     MCU_Time_Delay_Ms(uint32_t unMs);       /* blocking delay (needs ticks running) */

#endif /* MCU_TIME_H */
