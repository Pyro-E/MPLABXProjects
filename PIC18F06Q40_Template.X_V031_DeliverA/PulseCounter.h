#ifndef PULSECOUNTER_H
#define PULSECOUNTER_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 *  PulseCounter.h  -  flow-meter pulse counter for PIC18-Q40
 *
 *  Uses Timer1 as a 16-bit hardware event counter on the
 *  external pulse input (T1CKI routed to RC5 via PPS).
 *
 *  Configured ASYNCHRONOUS (T1SYNC = 1) so the counter keeps
 *  counting pulses even while the MCU is in Sleep -- the pulse
 *  edges clock the counter directly, no system clock needed.
 *
 *  This layer only owns the hardware counter and offers simple
 *  control. The "read + reset every 1 ms" accumulation logic
 *  lives in a higher driver layer (not here).
 *
 *  Pulse input: RC5  (T1CKI via PPS)
 * ============================================================ */

/* Configure Timer1 as async external counter on RC5. Starts STOPPED;
 * call PulseCounter_Enable() to begin counting. Counter = 0 after init. */
void     PulseCounter_Init(void);

/* Allow counting (Timer1 ON). */
void     PulseCounter_Enable(void);

/* Temporarily stop counting (Timer1 OFF). Current count is preserved. */
void     PulseCounter_Pause(void);

/* Reset the hardware count back to 0. Safe to call while running. */
void     PulseCounter_Clear(void);

/* Read the current 16-bit hardware count (safe 16-bit read). */
uint16_t PulseCounter_Get(void);

/* Read the count and clear to 0 in one step (returns the value read).
 * Useful for the higher layer's periodic snapshot. */
uint16_t PulseCounter_GetAndClear(void);

/* true if counting is currently enabled (Timer1 ON). */
bool     PulseCounter_IsEnabled(void);

#endif /* PULSECOUNTER_H */
