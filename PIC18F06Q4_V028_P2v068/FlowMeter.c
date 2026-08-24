#define FLOWMETER_C

#include <xc.h>
#include <stdint.h>
#include "FlowMeter.h"
#include "PulseCounter.h"
#include "App_Config.h"
#include "MCU_Time.h"
#include "Dev_Debug_Uart.h"

/* ============================================================
 *  The 32-bit total is advanced whenever the meter is read.
 *  Each read takes the wrap-safe 16-bit increment since the
 *  previous read and folds it into the total.
 * ============================================================ */
static uint16_t s_prev_cnt = 0;   /* hardware count at last read */
static uint32_t s_total    = 0;   /* 32-bit grand total */

#ifdef APP_VFLOW_ENABLE
/* ============================================================
 *  VIRTUAL FLOW METER  (bench only)
 *
 *  With no meter wired to RC5 the pulse counter never moves, so every capture
 *  reads zero and nothing downstream can be exercised - not the sample path,
 *  not the leak windows, not the valve, not the Photon's gallon maths. This
 *  generates pulses in software instead, at THIS ONE POINT: the single place
 *  where flow enters the system. Everything above it (capture, FIFO, alarm
 *  ring, report) runs completely unchanged and cannot tell the difference.
 *
 *  The rate follows a fixed schedule (APP_VFLOW_PHASES) so a run is
 *  reproducible and the log alone proves what happened: each phase announces
 *  itself, and the pulses that follow must match the rate it declared.
 *
 *  The count is computed from ELAPSED TIME, not accumulated per call:
 *      total = elapsed_ms * pps / 1000
 *  so it is exact regardless of how often this function is called, and a
 *  missed call cannot lose pulses - the same guarantee the real asynchronous
 *  counter gives.
 * ============================================================ */

typedef struct { uint32_t until_ms; uint16_t pps; const char *name; } vflow_phase_t;

/* Phases are chosen to cross the bench leak thresholds in a known order, so
 * the log shows: quiet -> normal -> weak alert -> recovery -> strong alert. */
static const vflow_phase_t s_vflow[] = APP_VFLOW_PHASES;
#define VFLOW_N  (sizeof(s_vflow)/sizeof(s_vflow[0]))

static uint32_t s_vf_base_ms   = 0;   /* when the current phase started      */
static uint32_t s_vf_base_cnt  = 0;   /* pulse total at that moment          */
static uint8_t  s_vf_idx       = 0xFFu;

/* pulses = ms * pps / 1000, computed so it cannot overflow 32 bits.
 * The direct form (ms * pps) breaks down at high rates: 65535 pps over 180 s
 * needs 11.8e9, far past 2^32. Splitting the elapsed time into whole seconds
 * and a remainder keeps every intermediate small while staying exact:
 *      ms = s*1000 + r      pulses = s*pps + (r*pps)/1000
 * The remainder term is at most 999*65535 = 65.5e6, and the whole-second term
 * only overflows after the total itself would (see the base-count note). */
static uint32_t vflow_pulses(uint32_t ms, uint16_t pps)
{
    uint32_t whole = ms / 1000UL;
    uint32_t rem   = ms % 1000UL;
    return (whole * (uint32_t)pps) + ((rem * (uint32_t)pps) / 1000UL);
}

static uint32_t vflow_total(void)
{
    uint32_t now = getNowTime();

    /* advance through the schedule; the last entry runs forever */
    uint8_t idx = 0;
    while ((idx < (VFLOW_N - 1u)) && (now >= s_vflow[idx].until_ms)) {
        idx++;
    }

    if (idx != s_vf_idx) {
        /* Freeze the count reached so far, then start measuring the new rate
         * from here - so a rate change never creates or loses pulses. */
        if (s_vf_idx != 0xFFu) {
            s_vf_base_cnt += vflow_pulses(now - s_vf_base_ms,
                                          s_vflow[s_vf_idx].pps);
        }
        s_vf_base_ms = now;
        s_vf_idx     = idx;
        DBG_STR("[VFLW] phase "); DBG_STR(s_vflow[idx].name);
        DBG_STR(" pps=");         DBG_U32(s_vflow[idx].pps);
        DBG_STR(" t=");           DBG_U32(now);
        DBG_STR(" base=");        DBG_U32(s_vf_base_cnt);
        DBG_NL();
    }

    return s_vf_base_cnt + vflow_pulses(now - s_vf_base_ms, s_vflow[idx].pps);
}
#endif  /* APP_VFLOW_ENABLE */

void FlowMeter_Init(void)
{
    s_prev_cnt = PulseCounter_Get();   /* baseline; no pulses counted yet */
    s_total    = 0;
#ifdef APP_VFLOW_ENABLE
    s_vf_base_ms  = 0;
    s_vf_base_cnt = 0;
    s_vf_idx      = 0xFFu;             /* forces a phase announcement */
#endif
}


/* fold the latest increment into the total */
static void flowmeter_refresh(void)
{
#ifdef APP_VFLOW_ENABLE
    /* Simulated meter: the running total IS the schedule's integral. The real
     * counter is left untouched and simply ignored. */
    s_total = vflow_total();
#else
    uint16_t now = PulseCounter_Get();
    uint16_t inc = (uint16_t)(now - s_prev_cnt);   /* wrap-safe */
    s_prev_cnt = now;
    s_total   += (uint32_t)inc;
#endif
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
