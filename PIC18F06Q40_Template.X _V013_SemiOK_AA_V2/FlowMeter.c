#define FLOWMETER_C

#include <xc.h>
#include <stdint.h>
#include "FlowMeter.h"
#include "PulseCounter.h"

/* ============================================================
 *  The 32-bit total is advanced whenever the meter is read.
 *  Each read takes the wrap-safe 16-bit increment since the
 *  previous read and folds it into the total.
 * ============================================================ */
static uint16_t s_prev_cnt = 0;   /* hardware count at last read */
static uint32_t s_total    = 0;   /* 32-bit grand total */

void FlowMeter_Init(void)
{
    s_prev_cnt = PulseCounter_Get();   /* baseline; no pulses counted yet */
    s_total    = 0;
}

/* fold the latest hardware increment into the total */
static void flowmeter_refresh(void)
{
    uint16_t now = PulseCounter_Get();
    uint16_t inc = (uint16_t)(now - s_prev_cnt);   /* wrap-safe */
    s_prev_cnt = now;
    s_total   += (uint32_t)inc;
}

uint32_t FlowMeter_Update(void)
{
    flowmeter_refresh();
    return s_total;
}

uint32_t FlowMeter_GetTotal(void)
{
    flowmeter_refresh();
    return s_total;
}

void FlowMeter_ClearTotal(void)
{
    /* refresh first so no in-flight pulses are lost, then zero it.
     * keep s_prev_cnt as-is so the next increment stays correct. */
    flowmeter_refresh();
    s_total = 0;
}
