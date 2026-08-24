#ifndef LOCO_H
#define LOCO_H
/* ============================================================
 *  Loco.h  -  LFINTOSC ("LOCO") correction factor for sleep timing.
 *
 *  The deep-sleep wake period is derived from LFINTOSC (~31 kHz, +/-15%).
 *  We carry ONE correction factor k so the real sleep time can be matched:
 *      real_freq = 31kHz / (k)          ideal_count * k = corrected_count
 *
 *  k is stored as an integer in Q7 (denominator 128) so the PIC18 (no FPU,
 *  slow divide) only needs a multiply + right shift:
 *      corrected = (ideal * k_q7) >> 7
 *
 *  k is HARD-CLAMPED to the LFINTOSC spec band [128/1.15 , 128*1.15] =
 *  [111, 147] on every update / NVM restore, so even a broken evaluation or
 *  corrupt NVM can never push timing worse than the raw +/-15% spec.
 *
 *  Evaluation sources (best-effort, "skip if in doubt"):
 *    - HFINTOSC:  fast, coarse (+/-1..3%), always available   (Loco_OnHfintoscEval)
 *    - cloud time: precise (long interval)                    (Loco_OnCloudTime)
 *  See the design spec, section 7.
 * ============================================================ */

#include <stdint.h>
#include <stdbool.h>

#define LOCO_K_SHIFT     7          /* Q7: denominator = 128            */
#define LOCO_K_ONE       128        /* k = 1.0  (no correction, default)*/
#define LOCO_K_MIN       111        /* 128 / 1.15  (~0.87)              */
#define LOCO_K_MAX       147        /* 128 * 1.15                       */

/* ---- Report-time GRID alignment: reports land on the grid { anchor + n*interval }.
 * The interval is the report period itself (= M captures * capture period), so it
 * is passed in; only the ANCHOR is configured here. Examples:
 *   {midnight, 24h}: anchor 0, interval 24h  -> every midnight
 *   {midnight, 48h}: anchor 0, interval 48h  -> every 2nd midnight
 *   {22:00,    2h }: anchor 79200, interval 2h -> 22:00,00:00,02:00,...
 *   {22:00,    26h}: anchor = a 22:00 epoch, interval 26h -> non-daily grid
 * Only (anchor mod interval) matters. Default anchor 0 = UTC midnight
 * (epoch 0 is 1970-01-01 00:00:00 UTC). Set for tests as needed. */
#ifndef LOCO_ALIGN_ANCHOR_SEC
#define LOCO_ALIGN_ANCHOR_SEC    0u
#endif
/* "Comfortable first wake": if the next grid point is closer than
 * interval >> N, skip it and aim at the following one (avoid an ultra-short
 * first period). N=2 -> threshold = 25% of the interval. */
#ifndef LOCO_ALIGN_MIN_FIRST_SHIFT
#define LOCO_ALIGN_MIN_FIRST_SHIFT  2u
#endif
/* First-wake "comfortable" threshold, ABSOLUTE seconds: if the next grid point
 * is closer than this, skip to the following one. Spec = 1 hour. */
#ifndef LOCO_ALIGN_MIN_FIRST_S
#define LOCO_ALIGN_MIN_FIRST_S      3600u
#endif
/* Max one-cycle report-period nudge as a fraction (1/2^N) of the base period,
 * so ongoing phase alignment is gentle and can never destabilise the schedule. */
#define LOCO_PHASE_MAX_SHIFT     3u     /* max nudge = base_captures >> 3 (~12%) */

/* Convergence state (spec 7.7). */
typedef enum {
    LOCO_NO_REF = 0,   /* no time reference yet                        */
    LOCO_HAVE_REF,     /* first valid wake-abs captured                */
    LOCO_CONVERGING,   /* correcting toward midnight                   */
    LOCO_LOCKED        /* settled near target                          */
} loco_state_t;

void         Loco_Init(void);                 /* load NVM (clamped) or default */
int16_t      Loco_GetK(void);                  /* current k_q7                  */
void         Loco_SetK(int16_t k_q7);          /* set with clamp                */
loco_state_t Loco_GetState(void);

/* Apply the correction to an ideal wake/sleep count: (ideal * k) >> 7. */
uint32_t     Loco_ApplySleep(uint32_t ideal_count);

/* Stage 1: coarse evaluation against HFINTOSC.
 *   measured_lf_hz = LFINTOSC frequency measured with the 64 MHz HFINTOSC. */
void         Loco_OnHfintoscEval(uint32_t measured_lf_hz);

/* Measure the real LFINTOSC frequency by counting LFINTOSC edges over a
 * fixed HFINTOSC-timed window (Timer0 = LFINTOSC 16-bit, gated by the ms
 * time base). Call ONLY while awake (e.g. during the initial power-hold);
 * it briefly reconfigures Timer0 and restores it. Returns Hz (~31000), or 0
 * on failure. Feed the result to Loco_OnHfintoscEval(). Accuracy is bounded
 * by the HFINTOSC (~+/-1..3%), so a short window (a few hundred ms) is enough. */
uint32_t     Loco_MeasureLfHz(uint16_t window_ms);

/* Stage 3: precise evaluation from cloud absolute time.
 *   valid       : 1 if T_now is a real cloud time, else skip.
 *   t_now       : absolute epoch seconds at time-of-receipt.
 *   tnow_delay_s: wake->receipt delay (HFINTOSC-measured) to subtract.
 *   ideal_period_s : intended period this session (N * dt).
 *   overrun     : true if this report had a ring overrun (boundaries unsafe). */
void         Loco_OnCloudTime(uint8_t valid, uint32_t t_now,
                              uint32_t tnow_delay_s,
                              uint32_t ideal_period_s, bool overrun);

/* Midnight phase error (seconds) of the last valid wake, for the scheduler
 * to nudge the next period toward 00:00 local. 0 if no reference. */
int32_t      Loco_GetPhaseErrorS(void);

/* Bounded report-period nudge (in captures) to pull the NEXT report toward the
 * target time-of-day. POSITIVE = make the next period LONGER (we are running
 * early / before the target), NEGATIVE = shorter (running late). The magnitude
 * is capped at base_captures >> LOCO_PHASE_MAX_SHIFT so it can never destabilise
 * the schedule. This is applied to the report period only and is INDEPENDENT of
 * k (the frequency factor), so it does not corrupt the LOCO frequency estimate.
 *   capture_period_ms = MILLIseconds per capture. V025: this used to be whole
 *   seconds, but APP_CAPTURE_PERIOD_MS is not a whole number of seconds on the
 *   bench (5290 ms), and truncating it to 5 s made every derived period 5.5%
 *   short. See the note on Loco_GridCaptures() below.
 * Returns 0 when there is no valid reference yet. */
int16_t      Loco_PhaseNudgeCaptures(uint16_t base_captures, uint32_t capture_period_ms);

/* Captures until the next {anchor, interval} grid point, derived from the last
 * cloud time. Called EVERY report period (there is no fixed samples-per-report);
 * a report that lands early/late self-corrects on the next computation.
 * If the next grid point is too soon it aims at the following one, so the first
 * report after a cold boot is not squeezed into a few seconds.
 * max_captures is a safety ceiling (ring capacity). */
/* V025: capture_period_ms is MILLIseconds, not seconds.
 *
 * The old signature took whole seconds and every caller passed
 * APP_CAPTURE_PERIOD_MS / 1000, i.e. an integer division. On the bench that is
 * 5290/1000 = 5, so the capture period was understated by 0.29 s (5.5%). Over a
 * 34-capture report that is 9.9 s, and the log showed exactly that: the phase
 * error grew +9..+11 s per report (17 -> 26 -> 37 -> 45) and k slid to its
 * LOCO_K_MIN clamp trying to correct an error that was not in the hardware.
 *
 * Production hid it - 241004/1000 = 241 loses only 0.004 s (0.002%) - which is
 * why it only ever showed up in the time-compressed bench configuration. */
uint16_t     Loco_GridCaptures(uint16_t max_captures, uint32_t capture_period_ms);

/* LOCO calibration status snapshot. All *_precision / applied_k are Q7 (128=1.0).
 * Photon reads this and prints raw + (raw/128.0) so the bench can see the state. */
typedef struct {
    uint16_t hf_precision;      /* HFINTOSC coarse result (Q7)              */
    uint8_t  hf_calibrated;     /* 1 = HFINTOSC eval was run                */
    uint8_t  hf_in_range;       /* 1 = HFINTOSC result within +/-15%        */
    uint8_t  cloud_calibrated;  /* 1 = a cloud result has been accepted      */
    uint16_t cloud_precision;   /* last accepted/applied cloud value (Q7)    */
    uint8_t  cloud_in_range;    /* 1 = last cloud result within +/-15%       */
    uint16_t applied_k;         /* value actually in use (Q7)               */
} LocoStatus;

/* Predict the next wake (set before sleeping) so the next cloud eval compares
 * actual vs expected instead of a raw period ratio. */
void         Loco_SetExpectedWake(uint32_t expected_wake_abs, uint32_t intended_period_s);
/* Copy out the calibration status (for Photon to read + print). */
void         Loco_GetStatus(LocoStatus *st);

/* Runtime grid anchor (seconds-of-day = HH*3600+MM*60+SS). Photon read/write. */
void         Loco_SetAnchor(uint32_t anchor_sec_of_day);
uint32_t     Loco_GetAnchor(void);
void         Loco_SetReportInterval(uint32_t sec);
uint32_t     Loco_GetReportInterval(void);

/* No NVM: k always starts at the ideal (128) on power-up and is re-learned by
 * the HFINTOSC and cloud-time evaluations. The clamp [111,147] is the only
 * persistent safety guarantee. */

#endif /* LOCO_H */
