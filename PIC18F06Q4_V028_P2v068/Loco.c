/* ============================================================
 *  Loco.c  -  LFINTOSC correction factor (Q7 integer). See Loco.h + spec 7.
 *
 *  All math is integer. k in Q7 (denominator 128). Hard-clamped to
 *  [LOCO_K_MIN, LOCO_K_MAX] on every write so a bad evaluation or corrupt
 *  NVM cannot exceed the LFINTOSC +/-15% spec band.
 * ============================================================ */

#include "Loco.h"
#include "App_Config.h"
#include <xc.h>
#include "MCU_Time.h"     /* getNowTime / timeSpan (HFINTOSC ms base) */

#define LOCO_NOMINAL_LF_HZ   31000u   /* datasheet-nominal LFINTOSC */
#define SECS_PER_DAY         86400UL

/* Anomaly gate: only accept a cloud measurement whose actual/ideal ratio is
 * within the LFINTOSC band and is a SINGLE period (not several missed ones). */
#define LOCO_RATIO_LO_PCT    85u      /* 1/1.15 ~ 87%, use 85 for margin  */
#define LOCO_RATIO_HI_PCT    115u
#define LOCO_MISSED_MULT     3u       /* actual > 1.5x ideal => missed     */

/* Smoothing: k_new = ((8-a)*k_old + a*k_meas)/8, a=2 -> 0.75/0.25 blend.
 * Chosen as shifts to stay divide-free. */
#define LOCO_SMOOTH_NUM      2
#define LOCO_SMOOTH_DEN      8

static int16_t      s_k        = LOCO_K_ONE;   /* == applied_k (Q7, 128=1.0)   */
static loco_state_t s_state    = LOCO_NO_REF;
static uint32_t     s_wake_abs_prev = 0;   /* last valid wake-abs (epoch)   */
static bool         s_have_prev = false;
static int32_t      s_phase_err = 0;

/* ---- LOCO calibration status (Photon reads this, all Q7 128=1.0) ---- */
static uint16_t     s_hf_precision    = LOCO_K_ONE;  /* HFINTOSC result           */
static uint8_t      s_hf_calibrated   = 0u;          /* HFINTOSC eval was run      */
static uint8_t      s_hf_in_range     = 0u;          /* HFINTOSC result within 15% */
static uint8_t      s_cloud_calibrated= 0u;          /* cloud eval accepted once    */
static uint16_t     s_cloud_precision = LOCO_K_ONE;  /* last accepted cloud value   */
static uint8_t      s_cloud_in_range  = 0u;          /* last cloud result within 15% */

/* Grid anchor (seconds-of-day, HH*3600+MM*60+SS). Runtime variable so the Photon
 * can change the reporting base time; initialised from the compile-time default. */
static uint32_t     s_align_anchor = (uint32_t)LOCO_ALIGN_ANCHOR_SEC;

/* Reporting interval in seconds (runtime; Photon may change it). Grid parameter,
 * kept next to the anchor. Default from the compile-time production period. */
#ifndef LOCO_REPORT_INTERVAL_SEC
#define LOCO_REPORT_INTERVAL_SEC  (48UL*3600UL)
#endif
static uint32_t     s_report_interval_sec = (uint32_t)LOCO_REPORT_INTERVAL_SEC;

/* Expected ABSOLUTE wake time for the NEXT report (set before sleeping), used to
 * evaluate LOCO by (actual - expected) rather than a raw period ratio. 0=unknown. */
static uint32_t     s_expected_wake_abs = 0u;
static uint32_t     s_expected_period_s = 0u;  /* the intended period that predicts it */

static int16_t loco_clamp(int32_t k)
{
    if (k < LOCO_K_MIN) return (int16_t)LOCO_K_MIN;
    if (k > LOCO_K_MAX) return (int16_t)LOCO_K_MAX;
    return (int16_t)k;
}

void Loco_Init(void)
{
    /* Always start from the ideal (k = 1.0 = 128). No NVM: the correction is
     * re-learned each power-up by the HFINTOSC (coarse) and cloud-time
     * (precise) evaluations. The only persistent guarantee is the clamp. */
    s_k         = LOCO_K_ONE;
    s_state     = LOCO_NO_REF;
    s_have_prev = false;
    s_wake_abs_prev = 0;
    s_phase_err = 0;
}

int16_t Loco_GetK(void)          { return s_k; }
loco_state_t Loco_GetState(void) { return s_state; }
int32_t Loco_GetPhaseErrorS(void){ return s_have_prev ? s_phase_err : 0; }

void Loco_SetK(int16_t k_q7)     { s_k = loco_clamp((int32_t)k_q7); }

/* Bounded report-period nudge (captures) toward the target time-of-day.
 * Independent of k. Positive = lengthen next period (we are early), negative =
 * shorten (we are late). Capped so it can never destabilise the schedule. */
int16_t Loco_PhaseNudgeCaptures(uint16_t base_captures, uint32_t capture_period_ms)
{
    if (!s_have_prev || capture_period_ms == 0u) {
        return 0;                         /* no reference yet */
    }
    /* We are LATE by s_phase_err seconds (if positive). To pull the wake back
     * toward the target we make the next period SHORTER by that many seconds,
     * i.e. nudge = -phase / capture_period (in captures). */
    /* V025: phase is in seconds, the period is now in ms -> scale once. Round
     * to nearest so a half-capture error does not silently become zero. */
    int32_t per_ms = (int32_t)capture_period_ms;
    int32_t nudge  = -((int32_t)s_phase_err * 1000 + (per_ms / 2)) / per_ms;

    /* clamp magnitude to base_captures >> LOCO_PHASE_MAX_SHIFT */
    int32_t cap = (int32_t)(base_captures >> LOCO_PHASE_MAX_SHIFT);
    if (cap < 1) cap = 1;
    if (nudge >  cap) nudge =  cap;
    if (nudge < -cap) nudge = -cap;
    return (int16_t)nudge;
}

/* ONE-TIME first alignment: captures until the next comfortable grid point. */
uint16_t Loco_GridCaptures(uint16_t max_captures, uint32_t capture_period_ms)
{
    if (!s_have_prev || capture_period_ms == 0u || max_captures == 0u) {
        return max_captures;                  /* no time reference yet -> ceiling */
    }
    uint32_t interval_s = s_report_interval_sec;
    if (interval_s == 0u) {
        interval_s = ((uint32_t)max_captures * capture_period_ms + 500u) / 1000u;
    }

    uint32_t to_next;
    if (interval_s >= SECS_PER_DAY) {
        /* Day-or-longer interval (e.g. 24 h, 48 h): the anchor clock-time recurs
         * every 24 h, so aim at the next occurrence of the anchor time-of-day on
         * a 24 h basis -- adding 24 h (not the full interval) still lands on it. */
        uint32_t tod        = s_wake_abs_prev % SECS_PER_DAY;
        uint32_t anchor_tod = s_align_anchor  % SECS_PER_DAY;
        to_next = (anchor_tod + SECS_PER_DAY - tod) % SECS_PER_DAY;
        if (to_next < LOCO_ALIGN_MIN_FIRST_S) to_next += SECS_PER_DAY;  /* too close -> +1 day */
    } else {
        /* Sub-day interval (e.g. {9pm, 3h}): grid points are anchor + k*interval;
         * aim at the next one. The "comfortable skip" only makes sense when the
         * interval itself is larger than the skip window, so it is suppressed for
         * short (bench-test) intervals -- there we simply take the next point. */
        uint32_t rel  = s_wake_abs_prev - s_align_anchor;
        uint32_t pos  = rel % interval_s;
        to_next = interval_s - pos;
        if (interval_s > LOCO_ALIGN_MIN_FIRST_S && to_next < LOCO_ALIGN_MIN_FIRST_S) {
            to_next += interval_s;             /* too close -> next grid point */
        }
    }

    /* V025: to_next is seconds, the period is ms. Do the division in ms with
     * round-to-nearest. to_next is at most 2 days (172800 s -> 172.8e6 ms), so
     * the multiply stays well inside uint32. */
    uint32_t caps = ((to_next * 1000u) + (capture_period_ms / 2u)) / capture_period_ms;
    if (caps < 1u) caps = 1u;
    /* Safety ceiling only (ring capacity); the count is normally well under it. */
    if (caps > (uint32_t)max_captures) caps = max_captures;
    return (uint16_t)caps;
}

uint32_t Loco_ApplySleep(uint32_t ideal_count)
{
    /* (ideal * k) >> 7. ideal is small (<= a few thousand), k <= 147, so the
     * product fits 32 bits comfortably. */
    return (uint32_t)(((uint32_t)ideal_count * (uint32_t)s_k) >> LOCO_K_SHIFT);
}

/* ---- Stage 1: HFINTOSC coarse eval ---- */
void Loco_OnHfintoscEval(uint32_t measured_lf_hz)
{
#ifndef LOCO_CAL_HFINTOSC_ENABLE
    (void)measured_lf_hz;            /* HFINTOSC eval forbidden -> keep k */
    return;
#else
    if (measured_lf_hz == 0u) return;                 /* guard              */
    /* raw = nominal / measured (Q7). If LF measured slower than nominal, raw>1. */
    int32_t raw = (int32_t)(((uint32_t)LOCO_NOMINAL_LF_HZ << LOCO_K_SHIFT)
                            / measured_lf_hz);
    s_hf_calibrated = 1u;
    /* 15% trust band: LOCO_K_MIN..LOCO_K_MAX are the +/-15% clamp bounds.
     * If the raw result is outside, treat the HFINTOSC eval as an algorithm
     * error and do NOT trust it (fall back to 1.0). */
    if (raw >= LOCO_K_MIN && raw <= LOCO_K_MAX) {
        s_hf_in_range = 1u;
        s_hf_precision = (uint16_t)raw;
        /* HFINTOSC is the starting point for the applied value (before cloud). */
        if (!s_cloud_calibrated) {
            s_k = loco_clamp(raw);
        }
    } else {
        s_hf_in_range = 0u;
        s_hf_precision = LOCO_K_ONE;                  /* untrusted -> 1.0    */
        if (!s_cloud_calibrated) s_k = LOCO_K_ONE;
    }
#endif
}

/* ---- Measure real LFINTOSC frequency against the HFINTOSC ms base ----
 *
 *  Principle: the ms time base (getNowTime) is driven by the accurate 64 MHz
 *  HFINTOSC. We let Timer0 free-run from the (inaccurate) LFINTOSC for a fixed
 *  HFINTOSC-timed window and count how many LFINTOSC cycles fit in it:
 *
 *        LFINTOSC_Hz = cycles_counted / (window_seconds)
 *                    = counted * 1000 / actual_ms
 *
 *  Timer0 is otherwise the deep-sleep wake source, so this must run WHILE AWAKE
 *  (e.g. during the initial power-hold). We save and restore Timer0's config.
 *
 *  Register notes (PIC18-Q40 Timer0), VERIFY against the datasheet on target:
 *    T0CON1: [7:5]=T0CS(100=LFINTOSC) [4]=T0ASYNC(1) [3:0]=T0CKPS(0000=1:1)
 *            -> 0x90 = LFINTOSC, async, no prescaler.
 *    T0CON0: [7]=T0EN [4]=T016BIT(1=16-bit) ; 0x10 configure(off,16b), 0x90 run.
 *    16-bit read: read TMR0L first (latches TMR0H), then TMR0H.
 *  At ~31 kHz a 16-bit counter overflows after ~2.1 s, so window is capped. */
uint32_t Loco_MeasureLfHz(uint16_t window_ms)
{
    if (window_ms == 0u)    window_ms = 500u;
    if (window_ms > 1500u)  window_ms = 1500u;   /* keep 16-bit from wrapping */

    /* save current Timer0 state (deep-sleep wake config) */
    uint8_t sT0CON0 = T0CON0;
    uint8_t sT0CON1 = T0CON1;
    uint8_t sT0IE   = PIE3bits.TMR0IE;

    PIE3bits.TMR0IE = 0;             /* no wake/IRQ during measurement        */

    T0CON0bits.T0EN = 0;             /* off while configuring                 */
    T0CON1 = 0x90u;                  /* LFINTOSC, async, 1:1                   */
    T0CON0 = 0x10u;                  /* 16-bit, still off                      */
    TMR0H  = 0x00u;                  /* clear high buffer first...             */
    TMR0L  = 0x00u;                  /* ...writing TMR0L commits both to 0     */

    uint32_t t0 = getNowTime();
    T0CON0bits.T0EN = 1;             /* start counting LFINTOSC edges          */

    while (timeSpan(t0) < (uint32_t)window_ms) {
        CLRWDT();                    /* stay awake, keep the watchdog happy    */
    }

    T0CON0bits.T0EN = 0;             /* stop                                   */
    uint32_t actual_ms = timeSpan(t0);

    uint8_t lo = TMR0L;              /* read low first -> latches high         */
    uint8_t hi = TMR0H;
    uint16_t counted = (uint16_t)(((uint16_t)hi << 8) | lo);

    /* restore deep-sleep Timer0 config exactly as it was */
    T0CON0bits.T0EN = 0;
    T0CON1 = sT0CON1;
    T0CON0 = sT0CON0;
    PIE3bits.TMR0IE = sT0IE;

    if (actual_ms == 0u) return 0u;  /* guard against divide-by-zero          */
    return (uint32_t)(((uint32_t)counted * 1000u) / actual_ms);
}

/* ---- Stage 3: cloud-time precise eval ---- */
void Loco_OnCloudTime(uint8_t valid, uint32_t t_now,
                      uint32_t tnow_delay_s,
                      uint32_t ideal_period_s, bool overrun)
{
#ifndef LOCO_CAL_CLOUD_ENABLE
    (void)valid; (void)t_now; (void)tnow_delay_s;
    (void)ideal_period_s; (void)overrun;   /* cloud eval forbidden -> keep k */
    return;
#else
    if (!valid) return;                               /* no cloud time      */

    /* Recover this wake's absolute time (subtract the wake->receipt delay). */
    uint32_t wake_abs = (t_now > tnow_delay_s) ? (t_now - tnow_delay_s) : t_now;

    /* Signed grid phase: how far this wake is from the nearest grid point
     * { anchor + n*interval }, where interval = ideal_period_s (the report
     * period). Range [-interval/2, +interval/2]. Positive = AFTER a grid point
     * (late), negative = BEFORE (early). */
    if (ideal_period_s != 0u) {
        uint32_t rel = wake_abs - s_align_anchor;
        int32_t  ph  = (int32_t)(rel % ideal_period_s);
        if (ph >  (int32_t)(ideal_period_s / 2u)) ph -= (int32_t)ideal_period_s;
        s_phase_err = ph;
    }

    if (!s_have_prev) {                               /* first reference    */
        s_wake_abs_prev = wake_abs;
        s_have_prev = true;
        s_state = LOCO_HAVE_REF;
        return;
    }

    if (overrun || ideal_period_s == 0u) {            /* boundaries unsafe  */
        s_wake_abs_prev = wake_abs;                   /* keep ref current   */
        return;
    }

    if (wake_abs <= s_wake_abs_prev) {                /* clock rollback     */
        s_wake_abs_prev = wake_abs;
        return;
    }

    /* ----- Expected-time evaluation (not a raw period ratio) -----
     * Before sleeping we predicted s_expected_wake_abs using the applied k and
     * the intended period s_expected_period_s. Now compare the ratio of the
     * REAL interval we actually slept to the interval we PREDICTED. If we have
     * no prediction yet, fall back to the intended period. */
    uint32_t actual = wake_abs - s_wake_abs_prev;     /* real period (s)    */
    uint32_t predicted = (s_expected_wake_abs > s_wake_abs_prev)
                         ? (s_expected_wake_abs - s_wake_abs_prev)
                         : ideal_period_s;             /* fallback           */
    if (predicted == 0u) predicted = ideal_period_s;

    /* missed-cycle gate: far more than one period elapsed -> do not calibrate */
    if (actual > (uint32_t)(ideal_period_s + ideal_period_s / 2u) * (LOCO_MISSED_MULT - 1u)) {
        s_wake_abs_prev = wake_abs;
        return;
    }

    /* meas (Q7) = actual / predicted * 128. If our prediction was perfect this
     * is 128; deviation is the residual LOCO error we still need to correct. */
    int32_t meas = (int32_t)(((uint64_t)actual << LOCO_K_SHIFT) / predicted);

    /* 15% trust band: a cloud result outside +/-15% is treated as an evaluation
     * error (bad network timing etc.) -> DISCARD this round and fall back. */
    if (meas < LOCO_K_MIN || meas > LOCO_K_MAX) {
        s_cloud_in_range = 0u;
        /* keep a previously-accepted cloud value if we have one; else HFINTOSC;
         * else 1.0. (applied_k s_k already holds the best-so-far, but re-assert
         * the fallback when no cloud value was ever accepted.) */
        if (!s_cloud_calibrated) {
            s_k = (s_hf_in_range) ? loco_clamp((int32_t)s_hf_precision) : LOCO_K_ONE;
        }
        s_wake_abs_prev = wake_abs;
        return;                                       /* retry next session */
    }

    /* Accepted. RELATIVE accumulation: applied_k *= meas (not reset to 1.0).
     *   k_new = (k_old * meas) >> 7 . Then clamp to +/-15%. */
    int32_t k_new = (int32_t)(((int32_t)s_k * meas) >> LOCO_K_SHIFT);
    s_k = loco_clamp(k_new);

    s_cloud_calibrated = 1u;
    s_cloud_in_range   = 1u;
    s_cloud_precision  = (uint16_t)s_k;               /* the value now in use */

    s_wake_abs_prev = wake_abs;
    if (s_state < LOCO_CONVERGING) s_state = LOCO_CONVERGING;
#endif /* LOCO_CAL_CLOUD_ENABLE */
}

/* Store the prediction for the NEXT wake so the next cloud eval can compare
 * (actual vs expected). Called from main when arming the next report. */
void Loco_SetExpectedWake(uint32_t expected_wake_abs, uint32_t intended_period_s)
{
    s_expected_wake_abs = expected_wake_abs;
    s_expected_period_s = intended_period_s;
}

/* Runtime grid anchor (seconds-of-day). Photon writes via GET/SET_PARAM. */
void     Loco_SetAnchor(uint32_t anchor_sec_of_day) { s_align_anchor = anchor_sec_of_day; }
uint32_t Loco_GetAnchor(void)                        { return s_align_anchor; }
void     Loco_SetReportInterval(uint32_t sec)        { if (sec) s_report_interval_sec = sec; }
uint32_t Loco_GetReportInterval(void)                { return s_report_interval_sec; }

/* Fill the caller's status snapshot (for Photon to read out over UART). */
void Loco_GetStatus(LocoStatus *st)
{
    if (!st) return;
    st->hf_precision     = s_hf_precision;
    st->hf_calibrated    = s_hf_calibrated;
    st->hf_in_range      = s_hf_in_range;
    st->cloud_calibrated = s_cloud_calibrated;
    st->cloud_precision  = s_cloud_precision;
    st->cloud_in_range   = s_cloud_in_range;
    st->applied_k        = (uint16_t)s_k;
}

