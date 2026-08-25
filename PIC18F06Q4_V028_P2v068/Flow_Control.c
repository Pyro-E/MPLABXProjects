/*
 * Flow_Control.c - leak detection + valve lock policy (sliding-window).
 * See Flow_Control.h for the model.
 *
 * SLIDING-WINDOW LEAK DETECTION
 * -----------------------------
 * Each alert looks at the SUM of the most recent N capture periods - a true
 * sliding window (0-7, 1-8, 2-9, ...). N ("window_n") is the alert's window
 * length in seconds, quantized to capture periods. The sum is computed by
 * walking BACKWARD from the newest capture through the existing FlowLog ring
 * buffer, so NO separate buffer is needed.
 *
 * The backward walk STOPS at the first of:
 *    (1) a 0-count capture     -> flow stopped; only count captures AFTER it.
 *    (2) N captures counted    -> the window limit reached.
 *    (3) the start of measurement (fewer than N captures exist yet).
 * Whatever was summed up to that stop point is compared with the threshold.
 * Consequences:
 *    - a leak can trip in FEWER than N periods if it crosses the threshold
 *      early (no need to wait for the full window);
 *    - it keeps tripping while flow continues PAST N periods (true sliding);
 *    - a single 0 wipes the accumulation (only post-0 captures are summed).
 *
 * The two alerts are COMPLETELY INDEPENDENT: each has its own window length,
 * its own threshold, its own separately-computed sum, and its own lock:
 *    alert1 -> TEMPORARY lock, auto-clears after TIME_VALVE_TEMP_LOCK_MS, then
 *              the valve auto-opens (unless the permanent lock still holds it).
 *    alert2 -> PERMANENT lock, cleared only by a Photon unlock packet or reset.
 * The valve is driven CLOSED if EITHER lock is set, OPEN only when BOTH clear.
 *
 * Runtime parameter changes are SAFE: only the window_n values change; the
 * FlowLog ring buffer is never touched, so there is nothing to resize or
 * corrupt when the window count changes. After an update we re-evaluate
 * immediately, so a new (lower) threshold already exceeded fires at once.
 *
 * IMPORTANT ORDERING: FlowControl_OnCapture() is called by FlowLog AFTER the
 * new sample has been written to the ring buffer, so the window logic always
 * reads the newest capture (this one included) directly from FlowLog.
 */
#include <xc.h>
#include "Flow_Control.h"
#include "Dev_Debug_Uart.h"
#include "MValve_OP3.h"
#include "MCU_Time.h"
#include "FlowLog.h"        /* reuse the capture ring buffer for the window */

/* ---- parameters (defaults; overwritten by Photon SET_PARAM) ---- */
static leak_param_t s_param = {
    .leak1_counts   = APP_LEAK1_COUNTS_DEF,
    .leak1_window_s = APP_LEAK1_WINDOW_S_DEF,
    .leak2_counts   = APP_LEAK2_COUNTS_DEF,
    .leak2_window_s = APP_LEAK2_WINDOW_S_DEF
};

/* ---- dedicated ALARM FIFO (separate from the capture ring) --------------
 * Leak windows (minutes) cannot be resolved well by the 4-min capture ring, so
 * the alarm logic keeps its OWN small ring. Each slot holds the pulses over a
 * fixed number of wakes (s_slot_wakes), auto-chosen so the LONGER of the two
 * alert windows fits in at most ALARM_FIFO_MAX slots (keeps RAM tiny). Each
 * alert's window is then a count of alarm slots. Fed every wake by
 * FlowControl_OnWake(); a slot is committed (and alerts evaluated) once
 * s_slot_wakes wakes have accumulated. */
#define ALARM_FIFO_MAX   32u
static uint16_t s_alarm_buf[ALARM_FIFO_MAX];   /* pulses per committed slot   */
static uint16_t s_alarm_write   = 0u;          /* ring write index            */
static uint16_t s_alarm_count   = 0u;          /* slots written (sat at MAX)  */
static uint16_t s_slot_wakes    = 1u;          /* wakes per alarm slot (auto) */
static uint16_t s_wake_ctr      = 0u;          /* wakes since last commit     */
static uint32_t s_slot_accum    = 0u;          /* pulses since last commit    */

/* ---- derived window lengths, in ALARM SLOTS ---- */
static uint16_t s_leak1_window_n = 1u;
static uint16_t s_leak2_window_n = 1u;

/* ---- lock state (independent) ---- */
static bool     s_temp_locked    = false;
static uint32_t s_temp_unlock_ms = 0;     /* when temp lock auto-clears   */
static bool     s_perm_locked    = false;
static uint32_t s_temp_lock_count = 0;    /* cumulative # of temp locks   */

/* track what we last commanded so we only drive the valve on a real change */
static bool     s_valve_should_close = false;
static bool     s_valve_cmd_valid    = false;

/* ============================================================= */

/* Auto-choose the alarm-slot size (in wakes) and each alert's window length
 * (in alarm slots), so the LONGER window spans at most ALARM_FIFO_MAX slots.
 *   slot_wakes = ceil(longest_window / (ALARM_FIFO_MAX * wake_period))
 *   window_n   = ceil(window / (slot_wakes * wake_period))     (<= ALARM_FIFO_MAX)
 * Example: wake 0.5 s, windows 10/4 min -> slot_wakes=38 (19 s), n1=32, n2=13.
 *          wake 72 s,  windows 10/4 min -> slot_wakes=1  (72 s), n1=9,  n2=4.  */
void FlowControl_RecalcDerived(void)
{
    uint32_t wake_ms = (uint32_t)APP_WAKE_ACTUAL_MS;
    if (wake_ms == 0u) wake_ms = 1u;
    uint32_t w1_ms = (uint32_t)s_param.leak1_window_s * 1000UL;
    uint32_t w2_ms = (uint32_t)s_param.leak2_window_s * 1000UL;
    uint32_t wlong = (w1_ms > w2_ms) ? w1_ms : w2_ms;

    uint32_t denom = (uint32_t)ALARM_FIFO_MAX * wake_ms;
    uint32_t sw    = (wlong + denom - 1u) / denom;    /* ceil */
    if (sw < 1u) sw = 1u;
    s_slot_wakes = (uint16_t)sw;

    uint32_t slot_ms = (uint32_t)s_slot_wakes * wake_ms;
    uint32_t n1 = (w1_ms + slot_ms - 1u) / slot_ms;   /* ceil, in alarm slots */
    uint32_t n2 = (w2_ms + slot_ms - 1u) / slot_ms;
    if (n1 > ALARM_FIFO_MAX) n1 = ALARM_FIFO_MAX;
    if (n2 > ALARM_FIFO_MAX) n2 = ALARM_FIFO_MAX;
    if (n1 < 1u) n1 = 1u;
    if (n2 < 1u) n2 = 1u;
    s_leak1_window_n = (uint16_t)n1;
    s_leak2_window_n = (uint16_t)n2;

#ifdef APP_DEBUG_EVENT_LOG
    /* The alarm ring sizes itself: one slot spans several wakes so that the
     * LONGER of the two windows still fits in 32 slots. Printing the result
     * lets the derived numbers be checked against the design formula:
     *   slot_wakes = ceil(longest_window / (32 * wake_ms))
     *   window_n   = ceil(window_ms / (slot_wakes * wake_ms))            */
    DBG_STR("[ALRM] ring slot="); DBG_U32(s_slot_wakes);
    DBG_STR(" wakes (");          DBG_U32((uint32_t)s_slot_wakes * APP_WAKE_ACTUAL_MS);
    DBG_STR("ms) n1=");           DBG_U32(s_leak1_window_n);
    DBG_STR(" n2=");              DBG_U32(s_leak2_window_n);
    DBG_STR(" max=");             DBG_U32(ALARM_FIFO_MAX); DBG_NL();
#endif
}

/* Sliding-window sum: total pulses over the most recent `window_n` captures,
 * read straight from the FlowLog ring buffer (newest first). Stops at a 0
 * count, at the window limit, or at the start of measurement (whichever comes
 * first). Returns 0 if no captures exist yet or the newest capture is 0. */
/* Sum the pulses over up to window_n most-recent captures, walking backward and
 * stopping at the first 0-count capture (only captures after a 0 count) or at
 * the start of measurement. The number of captures actually summed is returned
 * through *out_n, so the caller can require a FULL window (out_n == window_n)
 * before it trips: a burst that crosses the threshold in the first few captures
 * must NOT trip until the whole window (window_n captures = window_s seconds)
 * has elapsed. */
static uint32_t sliding_sum(uint16_t window_n, uint16_t *out_n)
{
    uint16_t counted = 0u;
    uint16_t avail   = s_alarm_count;               /* alarm slots stored so far */
    if (avail == 0u) {
        if (out_n) *out_n = 0u;
        return 0u;
    }
    uint16_t limit = window_n;
    if (limit > avail) limit = avail;

    /* s_alarm_write points at the NEXT slot to write; newest is (write - 1). */
    uint32_t sum = 0u;
    for (uint16_t k = 0; k < limit; k++) {
        uint16_t idx = (uint16_t)((s_alarm_write + ALARM_FIFO_MAX - 1u - k) % ALARM_FIFO_MAX);
        uint16_t p   = s_alarm_buf[idx];
        if (p == 0u) {
            break;                 /* 0 slot -> only slots after it count */
        }
        sum += (uint32_t)p;
        counted++;
    }
    if (out_n) *out_n = counted;
    return sum;
}

void FlowControl_Init(void)
{
    s_alarm_write = 0u;
    s_alarm_count = 0u;
    s_wake_ctr    = 0u;
    s_slot_accum  = 0u;
    for (uint16_t i = 0; i < ALARM_FIFO_MAX; i++) s_alarm_buf[i] = 0u;
    FlowControl_RecalcDerived();
    s_temp_locked = false;
    s_perm_locked = false;
    s_temp_lock_count = 0;
    s_valve_should_close = false;
    s_valve_cmd_valid    = false;
}

/* drive the valve to match the combined lock state (only on a real change) */
static void apply_valve(void)
{
    bool want_close = (s_temp_locked || s_perm_locked);

    if (!s_valve_cmd_valid) {
        /* first evaluation: adopt the current (assumed-open) state without
         * driving. Startup opening, if any, is handled by main's
         * VALVE_ON_WHEN_STARTUP. Only drive now if we must already close. */
        s_valve_should_close = want_close;
        s_valve_cmd_valid    = true;
        if (want_close) {
            MValve_OP3_CmdClose();
        }
        return;
    }

    if (want_close != s_valve_should_close) {
        if (want_close) MValve_OP3_CmdClose();
        else            MValve_OP3_CmdOpen();
        s_valve_should_close = want_close;
    }
}

/* Evaluate BOTH alerts independently, each over its own sliding window. */
static void evaluate_alerts(void)
{
    /* Alert 1 (temporary) - its own window length and threshold.
     * Trip ONLY after a full window has accumulated (n1 == window_n): even if
     * the threshold is already exceeded in the first few captures, we wait
     * until the whole leak1 window (leak1_window_s seconds) has elapsed. */
    if (!s_temp_locked) {
        uint16_t n1   = 0u;
        uint32_t sum1 = sliding_sum(s_leak1_window_n, &n1);
        if ((n1 >= s_leak1_window_n) &&
            (sum1 >= (uint32_t)s_param.leak1_counts)) {
            s_temp_locked    = true;
            s_temp_unlock_ms = getNowTime();
            s_temp_lock_count++;
#ifdef APP_DEBUG_EVENT_LOG
            /* sum/threshold over n slots: everything needed to check the
             * decision by hand against the alarm-ring maths. */
            DBG_STR("[LEAK] ALERT1 (weak) TEMP LOCK sum="); DBG_U32(sum1);
            DBG_STR(" >= thr=");  DBG_U32(s_param.leak1_counts);
            DBG_STR(" slots=");   DBG_U32(n1);
            DBG_STR("/");         DBG_U32(s_leak1_window_n);
            DBG_STR(" win=");     DBG_U32(s_param.leak1_window_s);
            DBG_STR("s t=");      DBG_U32(getNowTime()); DBG_NL();
#endif
        }
    }

    /* Alert 2 (permanent) - separately computed, own window and threshold.
     * Same full-window rule: only trip once the whole leak2 window has elapsed. */
    if (!s_perm_locked) {
        uint16_t n2   = 0u;
        uint32_t sum2 = sliding_sum(s_leak2_window_n, &n2);
        if ((n2 >= s_leak2_window_n) &&
            (sum2 >= (uint32_t)s_param.leak2_counts)) {
            s_perm_locked = true;
#ifdef APP_DEBUG_EVENT_LOG
            DBG_STR("[LEAK] ALERT2 (strong) PERM LOCK sum="); DBG_U32(sum2);
            DBG_STR(" >= thr=");  DBG_U32(s_param.leak2_counts);
            DBG_STR(" slots=");   DBG_U32(n2);
            DBG_STR("/");         DBG_U32(s_leak2_window_n);
            DBG_STR(" win=");     DBG_U32(s_param.leak2_window_s);
            DBG_STR("s t=");      DBG_U32(getNowTime()); DBG_NL();
#endif
        }
    }

    apply_valve();
}

void FlowControl_OnWake(uint32_t pulses_this_wake)
{
    /* Accumulate this wake's pulses; commit one alarm slot every s_slot_wakes
     * wakes, then evaluate both alerts over their (alarm-slot) windows. */
    s_slot_accum += pulses_this_wake;
    s_wake_ctr++;
    if (s_wake_ctr < s_slot_wakes) {
        return;                         /* slot not full yet */
    }

    uint16_t v = (s_slot_accum > 65535u) ? 65535u : (uint16_t)s_slot_accum;
    s_alarm_buf[s_alarm_write] = v;
    s_alarm_write = (uint16_t)((s_alarm_write + 1u) % ALARM_FIFO_MAX);
    if (s_alarm_count < ALARM_FIFO_MAX) s_alarm_count++;
    s_slot_accum = 0u;
    s_wake_ctr   = 0u;

    evaluate_alerts();
}

void FlowControl_Process(void)
{
    /* temporary lock auto-clears after TIME_VALVE_TEMP_LOCK_MS; then the
     * valve reopens unless the permanent lock still holds it closed. */
    if (s_temp_locked &&
        (timeSpan(s_temp_unlock_ms) >= TIME_VALVE_TEMP_LOCK_MS)) {
        s_temp_locked = false;
        apply_valve();
#ifdef APP_DEBUG_EVENT_LOG
        /* The weak alert is supposed to release itself; the strong one is not.
         * Printing both states here shows which rule actually applied. */
        DBG_STR("[LEAK] alert1 temp lock RELEASED after ");
        DBG_U32(TIME_VALVE_TEMP_LOCK_MS);
        DBG_STR("ms  perm=");  DBG_U32(s_perm_locked ? 1u : 0u);
        DBG_STR(" valve=");    DBG_STR(s_perm_locked ? "CLOSED" : "OPEN");
        DBG_STR(" t=");        DBG_U32(getNowTime()); DBG_NL();
#endif
    }
}

/* ---- parameter access ---- */
void FlowControl_GetParams(leak_param_t *out)
{
    if (out) *out = s_param;
}

bool FlowControl_SetParams(const leak_param_t *in)
{
    if (!in) return false;

#ifdef REPORT_CONFIG_DEBUG
    /* DEBUG / fast-test build: ignore parameter writes from the host so the
     * PIC keeps its own short test windows (e.g. 50/20 s, 100/10 s) for bench
     * testing, even if the Photon pushes its production defaults. The host
     * still gets a normal ACK (handled by the caller), so it does not see an
     * error - the values simply are not changed here. Rebuild WITHOUT
     * REPORT_CONFIG_DEBUG (production) to honor host SET_PARAM.
     *
     * V023 / fix 2: report the refusal instead of swallowing it. The caller
     * used to print "-> APPLIED" unconditionally, so the bench log claimed
     * SET_LEAK a1=100/480s had been applied while the very next line showed
     * a1=50/20s still in use. Returning false lets the caller log what really
     * happened. */
    (void)in;
    return false;
#else
    s_param = *in;
    FlowControl_RecalcDerived();
    /* Re-evaluate now so a new, already-exceeded threshold fires immediately
     * (the sliding sums are recomputed from the existing buffer). */
    evaluate_alerts();
    return true;
#endif
}

/* ---- lock status ---- */
uint8_t FlowControl_GetLockFlags(void)
{
    uint8_t f = 0u;
    if (s_temp_locked) f |= VALVE_LOCK_TEMP_BIT;
    if (s_perm_locked) f |= VALVE_LOCK_PERM_BIT;
    return f;
}

uint32_t FlowControl_GetTempLockCount(void)
{
    return s_temp_lock_count;
}

/* ---- unlock command ---- */
void FlowControl_Unlock(uint8_t flags)
{
    if (flags & VALVE_LOCK_TEMP_BIT) s_temp_locked = false;
    if (flags & VALVE_LOCK_PERM_BIT) s_perm_locked = false;
    apply_valve();
}
