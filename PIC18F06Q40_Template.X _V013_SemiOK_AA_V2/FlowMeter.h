#ifndef FLOWMETER_H
#define FLOWMETER_H

#include <stdint.h>

/* ============================================================
 *  FlowMeter.h  -  pure pulse accounting over PulseCounter
 *
 *  Wraps the 16-bit hardware PulseCounter and keeps a 32-bit
 *  grand total. The total is updated AUTOMATICALLY every time
 *  the meter is read: each read compares against the previous
 *  hardware count and adds the (wrap-safe) increment.
 *
 *  This layer does NOT do periodic / per-interval differencing.
 *  That belongs to a higher layer (e.g. FlowLog). FlowMeter only
 *  answers "how many pulses in total so far" and can clear it.
 * ============================================================ */

void     FlowMeter_Init(void);

/* Read & fold the latest hardware pulses into the 32-bit total,
 * then return that total. Calling this is what advances the total. */
uint32_t FlowMeter_Update(void);

/* Current 32-bit grand total (also refreshes from hardware first). */
uint32_t FlowMeter_GetTotal(void);

/* Reset the grand total to 0. Counting then restarts from 0;
 * the next update still measures correctly (wrap-safe baseline). */
void     FlowMeter_ClearTotal(void);

#endif /* FLOWMETER_H */
