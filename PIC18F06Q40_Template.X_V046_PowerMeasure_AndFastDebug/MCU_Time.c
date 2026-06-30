#define MCU_TIME_C

#include "MCU_Time.h"

/* portable 32-bit millisecond counter */
static volatile uint32_t unNowTime_ms = 0;

/* ---- current time in ms ---- */
uint32_t getNowTime(void)
{
    return unNowTime_ms;
}

/* ---- elapsed ms since unOldTime_ms, wrap-safe (32-bit) ----
 * If the counter wrapped past 0xFFFFFFFF, this still returns the
 * correct positive elapsed time. Same logic as the original module. */
uint32_t timeSpan(uint32_t unOldTime_ms)
{
    uint32_t now = getNowTime();
    if (unOldTime_ms > now) {
        return (0xFFFFFFFFUL - unOldTime_ms) + now + 1UL;
    }
    return now - unOldTime_ms;
}

/* ---- blocking delay built on the counter ----
 * Requires the 1 ms tick to be running (ISR mode), otherwise the
 * counter never advances. In polling mode, prefer non-blocking
 * timeSpan() checks instead of this. */
void MCU_Time_Delay_Ms(uint32_t unMs)
{
    uint32_t start = getNowTime();
    while (timeSpan(start) <= unMs) {
        ;
    }
}

/* ---- +1 ms : called by the tick source (ISR or polling) ---- */
void MCU_Time_Increase_Unit(void)
{
    unNowTime_ms++;
}

/* ---- bulk advance : jump the clock forward by a whole sleep period ----
 * Called once on wake from deep sleep, with the period that elapsed
 * (e.g. APP_FLOW_PERIOD_MS). Keeps getNowTime()/timeSpan() meaningful
 * across sleep, so the capture logic triggers exactly as if the 1 ms
 * tick had been running the whole time. */
void MCU_Time_Advance(uint32_t unMs)
{
    unNowTime_ms += unMs;
}

/* ---- init ---- */
void MCU_Time_Init(unsigned char ucKHz)
{
    (void)ucKHz;          /* unused on PIC (kept for API compatibility) */
    unNowTime_ms = 0;
}
