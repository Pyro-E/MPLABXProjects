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

/* ---- derived (quantized) window lengths, in capture counts ---- */
static uint16_t s_leak1_window_n = 0;
static uint16_t s_leak2_window_n = 0;

/* ---- lock state (independent) ---- */
static bool     s_temp_locked    = false;
static uint32_t s_temp_unlock_ms = 0;     /* when temp lock auto-clears   */
static bool     s_perm_locked    = false;

/* ---- leak trip flags since the last report (no lifetime count kept here --
 * the Photon does that; see Flow_Control.h) ---- */
static bool     s_leak1_since_report = false;   /* alert 1 (temp) tripped since last RSP_VALVE */
static bool     s_leak2_since_report = false;   /* alert 2 (perm) tripped since last RSP_VALVE */

/* track what we last commanded so we only drive the valve on a real change */
static bool     s_valve_should_close = false;
static bool     s_valve_cmd_valid    = false;

/* ============================================================= */

static uint16_t quantize_window(uint16_t window_s)
{
    /* window_n = window_seconds * 1000 / capture_period_ms (rounded down).
     * Using ms keeps it correct even for sub-second capture periods.
     * Clamp to the ring size so the backward walk can never exceed it. */
    uint32_t n = ((uint32_t)window_s * 1000UL) / (uint32_t)APP_CAPTURE_PERIOD_MS;
    if (n > (uint32_t)FLOWLOG_SLOTS) n = (uint32_t)FLOWLOG_SLOTS;
    if (n == 0u)                     n = 1u;     /* at least one capture */
    return (uint16_t)n;
}

void FlowControl_RecalcDerived(void)
{
    s_leak1_window_n = quantize_window(s_param.leak1_window_s);
    s_leak2_window_n = quantize_window(s_param.leak2_window_s);
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
    uint32_t already = FlowLog_GetCaptureCount();   /* captures stored so far */
    if (already == 0u) {
        if (out_n) *out_n = 0u;
        return 0u;
    }

    /* number of slots we may inspect: min(window_n, captures-so-far) */
    uint16_t limit = window_n;
    if ((uint32_t)limit > already) {
        limit = (uint16_t)already;
    }

    /* FlowLog write index points at the NEXT slot to write; the newest stored
     * capture is at (w - 1). Walk backward from there. */
    uint16_t w   = FlowLog_GetWriteIndex();
    uint32_t sum = 0u;
    for (uint16_t k = 0; k < limit; k++) {
        uint16_t idx = (uint16_t)((w + FLOWLOG_SLOTS - 1u - k) % FLOWLOG_SLOTS);
        flowlog_entry_t e;
        FlowLog_GetAt(idx, &e);
        if (e.pulses == 0u) {
            break;                 /* 0 count -> only captures after it count */
        }
        sum += (uint32_t)e.pulses;
        counted++;
    }
    if (out_n) *out_n = counted;
    return sum;
}

void FlowControl_Init(void)
{
    FlowControl_RecalcDerived();
    s_temp_locked = false;
    s_perm_locked = false;
    s_leak1_since_report = false;
    s_leak2_since_report = false;
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
            s_leak1_since_report = true;  
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
            s_leak2_since_report = true;
        }
    }

    apply_valve();
}

void FlowControl_OnCapture(uint16_t delta)
{
    /* `delta` is already stored in FlowLog (FlowLog calls us AFTER writing).
     * It is unused here directly - the window reads it back from the buffer -
     * but kept in the signature for clarity and future use. */
    (void)delta;
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
    }
}

/* ---- parameter access ---- */
void FlowControl_GetParams(leak_param_t *out)
{
    if (out) *out = s_param;
}

void FlowControl_SetParams(const leak_param_t *in)
{
    if (!in) return;

#ifdef REPORT_CONFIG_DEBUG
    /* DEBUG / fast-test build: ignore parameter writes from the host so the
     * PIC keeps its own short test windows (e.g. 50/20 s, 100/10 s) for bench
     * testing, even if the Photon pushes its production defaults. The host
     * still gets a normal ACK (handled by the caller), so it does not see an
     * error - the values simply are not changed here. Rebuild WITHOUT
     * REPORT_CONFIG_DEBUG (production) to honor host SET_PARAM. */
    (void)in;
    return;
#else
    s_param = *in;
    FlowControl_RecalcDerived();
    /* Re-evaluate now so a new, already-exceeded threshold fires immediately
     * (the sliding sums are recomputed from the existing buffer). */
    evaluate_alerts();
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

uint8_t FlowControl_GetSinceReportFlags(void)
{
    uint8_t f = 0u;
    if (s_leak1_since_report) f |= VALVE_LOCK_TEMP_BIT;
    if (s_leak2_since_report) f |= VALVE_LOCK_PERM_BIT;
    return f;
}

void FlowControl_ClearSinceReport(void)
{
    s_leak1_since_report = false;
    s_leak2_since_report = false;
}

/* ---- unlock command ---- */
void FlowControl_Unlock(uint8_t flags)
{
    if (flags & VALVE_LOCK_TEMP_BIT) s_temp_locked = false;
    if (flags & VALVE_LOCK_PERM_BIT) s_perm_locked = false;
    apply_valve();
}
