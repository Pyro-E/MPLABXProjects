/* ============================================================
 *  main.c  -  flow meter top level (PIC18F06Q40)
 *
 *  Adds to the V032 baseline:
 *    - framed UART packet protocol (Packet.*) replacing raw 0xAA/0xF0 data
 *    - leak detection + OP3 valve auto-shutoff (Flow_Control + MValve_OP3)
 *    - PMOS POWER-GATING model: RC4 no longer feeds the Photon's D10 wake
 *      input; it drives an external P-MOS that switches the Photon's SUPPLY
 *      (RC4 LOW = Photon powered, RC4 HIGH = Photon off). The Photon has no
 *      wake source and only (re)starts when the PIC re-applies power. main
 *      runs a 3-state power machine (SLEEP / WAIT_FIRST / ACTIVE) - see
 *      App_Config.h "Photon POWER MANAGEMENT". A new PKT_PHOTON_OFF_REQ
 *      (func 0x07) lets the Photon ask the PIC to cut its power when done.
 *    - capture + leak detection run in ALL power states (time-based), so the
 *      cadence holds whether the PIC is asleep or awake for Photon comms.
 *    - reset-cause aware startup (PCON0): non-WDT reset may force valve open
 *
 *  Pins:
 *    RC0 = UART TX     RC1 = UART RX
 *    RC3 = TEST LED    RC4 = Photon2 POWER gate (out, active-low via P-MOS)
 *    RC5 = flow pulse input (Timer1 T1CKI via PPS)
 *    RA2 = valve CTRL  RC2 = valve PWR
 * ============================================================ */

// ===== Configuration Bits =====
#pragma config FEXTOSC = OFF             // External oscillator disabled
#pragma config RSTOSC = HFINTOSC_64MHZ   // Internal oscillator at 64 MHz
#pragma config MVECEN = OFF              // Single-vector interrupt mode
#pragma config MCLRE = EXTMCLR           // MCLR pin used as reset
#pragma config LVP = ON                  // Low-voltage programming enabled

#include <xc.h>
#include <stdint.h>          /* uint8_t/uint16_t/uint32_t used by the debug   */
#include <stdbool.h>         /* bool - both are needed before the debug block */
#include "App_Config.h"

#ifdef APP_WATCHDOG_ENABLE
#pragma config WDTCPS = WDTCPS_12        // WDT period ~4 s
#pragma config WDTE   = SWDTEN           // WDT controlled by SEN bit
#pragma config WDTCWS = WDTCWS_7         // window always open (plain WDT)
#else
#pragma config WDTE   = OFF              // Watchdog disabled
#endif

#include "Dev_Led.h"
#include "led_fsm_sysstate.h"
#include "Dev_Uart.h"
#include "Dev_Debug.h"
#define DBG_OWNER            /* this file holds the one debug FIFO buffer */
#include "Dev_Debug_Uart.h"   /* TX-only debug UART2 on RA4 (header-only) */
#include "MCU_Time.h"
#include "Sys_Time_MCU_Specific.h"
#include "Loco.h"
#include "PulseCounter.h"
#include "FlowMeter.h"
#include "FlowLog.h"
#include "FlowReport.h"
#include "MValve_OP3.h"
#include "Flow_Control.h"
#include "Packet.h"

#define _XTAL_FREQ 64000000


/* ---- Photon power state machine (PMOS power-gating model) ----
 * See App_Config.h "Photon POWER MANAGEMENT" for the full description.
 * RC4 LOW powers the Photon ON; RC4 HIGH powers it OFF. */
typedef enum {
    PWR_SLEEP = 0,     /* Photon OFF; PIC sleeps between captures            */
    PWR_WAIT_FIRST,    /* Photon booting; waiting for first valid packet     */
    PWR_ACTIVE         /* Photon up; handling packets                        */
} pwr_state_t;

static pwr_state_t s_pwr          = PWR_SLEEP;
static uint32_t    s_pwr_on_ms    = 0;      /* time Photon was powered on    */
static uint32_t    s_last_valid_ms = 0;     /* time last valid pkt processed */
static bool        s_pkt_seen     = false;  /* any valid pkt since power-on  */
static bool        s_off_request  = false;  /* PKT_PHOTON_OFF_REQ received    */
static bool        s_time_synced  = false;  /* TIME_SYNC applied this session */
static bool        s_first_aligned = false; /* grid first-alignment done once  */
/* Report interval + anchor: normally relayed by the Photon from the cloud and
 * adopted by the PIC. Define PIC_USE_OWN_TIMING (App_Config.h) to make the PIC
 * ACK the write but keep its own #define grid instead (bench self-timing). */
static uint16_t    s_report_captures = APP_NOMINAL_CAPTURES_PER_REPORT; /* captures                                       * cycle now filling (captures); nudged for
                                       * midnight alignment. Used to feed LOCO the
                                       * true intended period (keeps k uncorrupted).*/
static bool        s_cold_boot    = false;  /* set at boot: true on a cold power-up */
/* The report sent this session covers [previous wake, now]; its data STARTED at
 * the previous session's wake time. Track it so RSP_DATA.start_time is the span
 * START (not now, which is the span END). Invalid after boot or a time gap. */
static uint32_t    s_prev_wake_epoch = 0u;
/* V024b: how far the previous session's TIME_SYNC lagged its own frozen cutoff.
 * Subtracted from the reported start_time so the Photon places the series where
 * the data actually began, not where the cloud time happened to arrive. */
static uint16_t    s_prev_sync_lag_ms = 0u;
static uint8_t     s_prev_wake_valid = 0u;

/* True while we are still inside the initial power-hold window: a cold power-up
 * AND less than INITIAL_POWER_HOLD_MS has elapsed since boot. During this window
 * the PIC neither deep-sleeps nor cuts Photon power, and reports POWER_STATE_
 * INITIAL to the Photon. A WDT/soft reset is NOT a cold boot, so it skips the
 * hold entirely (reports NORMAL). */
static bool in_initial_hold(void)
{
    return s_cold_boot && (getNowTime() < (uint32_t)INITIAL_POWER_HOLD_MS);
}

static void photon_power_on(void)
{
    /* V023 / fix 4: freeze THIS session's data boundary BEFORE the Photon can
     * ask for anything. Every REQ_DATA of this session is then answered from
     * the same capture boundary, however long the Photon takes to boot, and
     * captures that happen while it is awake are left for the next report.
     * Doing it here (rather than at the report-due test in the power state
     * machine) covers every path that powers the Photon on. */
    FlowReport_FreezeSessionCutoff();

    /* V029: drive TX again BEFORE the rail comes up, so the line is already
     * idling HIGH by the time the Photon's UART starts sampling it. Doing it
     * after would present a 0 V line (the 3.3 k pull-down) to a booting
     * receiver, which reads as a framing error / break rather than idle. */
    UART_TX_Resume();

    PHOTON_PWR_ON;                 /* RC4 LOW -> P-MOS on -> Photon powered */
    s_pwr_on_ms    = getNowTime();
    s_pkt_seen     = false;
    s_off_request  = false;
    s_time_synced  = false;        /* per-session: TIME_SYNC not yet applied */
    FlowReport_ClearTxDrops();     /* V022 / H.7.2: count drops per session   */
    s_pwr          = PWR_WAIT_FIRST;
    LedFsm_NotifyWake();
#ifdef APP_DEBUG_EVENT_LOG
    DBG_STR("[PWR ] Photon ON t="); DBG_U32(getNowTime()); DBG_NL();
#endif
}

static void photon_power_off(void)
{
    PHOTON_PWR_OFF;                /* RC4 HIGH -> P-MOS off -> Photon off   */
    /* V029: with the far end unpowered there is nothing to receive, so stop
     * driving the line. This removes the 1.05 mA the level-shifting divider
     * draws continuously (see Dev_Uart.h) and stops the ~3.9 mA that a driven
     * output was pushing into the Photon's unpowered RX pin. The caller has
     * already waited for UART_TX_IsEmpty(), so nothing in flight is cut. */
    UART_TX_Release();
    s_pwr = PWR_SLEEP;
    /* V023 / fix 4: the session's cutoff dies with the session; the next one
     * freezes its own. An un-ACKed batch deliberately does NOT die here - it
     * is re-offered, unchanged, at the start of the next session. */
    FlowReport_ClearSessionCutoff();
#ifdef APP_DEBUG_EVENT_LOG
    /* V022 / H.7.2: txdrops MUST be 0 in a healthy session. A non-zero value
     * means a small response frame never reached the Photon, which the Photon
     * can only report as "no valid reply". */
    DBG_STR("[PWR ] Photon OFF t="); DBG_U32(getNowTime());
    DBG_STR(" txdrops=");            DBG_U32((uint32_t)FlowReport_GetTxDrops());
    /* V024b: say whether the LOG itself was complete for this session. Without
     * this a missing line is ambiguous - "that never happened" and "the debug
     * FIFO threw it away" look identical, and earlier campaigns did overflow
     * ("[BOOT] initial hold done (log FIFO overflowed)"). CLOUD_FAST adds new
     * lines on an untested path, so the reader must be able to trust an absence. */
    DBG_STR(" logdrop=");            DBG_STR(DBG_DROPPED() ? "YES" : "no");
    DBG_NL();
#endif
}

/* ============================================================
 *  Packet RX: bytes queued by the ISR; main feeds the parser.
 * ============================================================ */
static pkt_parser_t s_parser;

/* ISR-context RX callback: enqueue the byte and assert comms-ready. The
 * heavy parse/CRC happens in main context. */
static void on_uart_rx(uint8_t ch)
{
    /* Just queue the byte. Power is owned solely by the power state machine;
     * RX activity must NOT touch RC4 anymore (RC4 is now the power gate, not
     * a comms-ready line). Validity (CRC) is judged in main context. */
    UART_RX_Push(ch);
}

static void dispatch_packet(const pkt_parser_t *p)
{
#ifdef APP_DEBUG_PKT_LOG
    /* Goes out on the DEBUG UART (RA4), never on the packet UART. */
    /* Human-readable log of every received packet (bench PC, no Photon). */
    DBG_STR("RX: ");
    switch (p->func) {
    case PKT_REQ_DATA:         DBG_STR("REQ_DATA");        break;
    case PKT_REQ_GET_PARAM:    DBG_STR("REQ_GET_PARAM");   break;
    case PKT_REQ_GET_LOCO:     DBG_STR("REQ_GET_LOCO");    break;
    case PKT_REQ_SET_LEAK:     DBG_STR("REQ_SET_LEAK");    break;
    case PKT_REQ_SET_GRID:     DBG_STR("REQ_SET_GRID");    break;
    case PKT_REQ_GET_VALVE:    DBG_STR("REQ_GET_VALVE");   break;
    case PKT_REQ_VALVE_UNLOCK: DBG_STR("REQ_VALVE_UNLOCK");break;
    case PKT_SYS_RESET:        DBG_STR("SYS_RESET");       break;
    case PKT_PHOTON_OFF_REQ:   DBG_STR("PHOTON_OFF_REQ r=");
                               DBG_U32((p->len >= 1u) ? p->data[0] : 0u); break;
    case PKT_REQ_POWER_STATE:  DBG_STR("REQ_POWER_STATE"); break;
    case PKT_REQ_PHOTON_CFG:   DBG_STR("REQ_PHOTON_CFG");  break;
    case PKT_KEEPALIVE:        DBG_STR("KEEPALIVE");       break;
    case PKT_DATA_RECEIVED:    DBG_STR("DATA_RECEIVED");   break;
    case PKT_TIME_SYNC:        DBG_STR("TIME_SYNC");       break;
    default:                   DBG_STR("func=0x");
                               DBG_U32((uint32_t)p->func);   break;
    }
    DBG_STR(" len="); DBG_U32((uint32_t)p->len);
    DBG_NL();
#endif

    switch (p->func) {

    case PKT_REQ_DATA:
        FlowReport_NotifyAA();               /* RSP_DATA (streamed) */
        break;

    case PKT_DATA_RECEIVED:
        /* V024: the Photon echoes the span endpoints it is acknowledging -
         * span_end_pulses(u32) then span_end_captures(u32), big-endian. Commit
         * only if they name the batch we are actually holding.
         *
         * A len-0 ACK (v1 Photon) carries no identifier and can only mean "the
         * last thing you streamed", so it is accepted as before. The two builds
         * must be paired anyway; refusing it here would wedge the link silently
         * instead of failing loudly at the header-length check. */
        if (p->len >= 8u) {
            uint32_t ep = ((uint32_t)p->data[0] << 24) | ((uint32_t)p->data[1] << 16)
                        | ((uint32_t)p->data[2] <<  8) |  (uint32_t)p->data[3];
            uint32_t ec = ((uint32_t)p->data[4] << 24) | ((uint32_t)p->data[5] << 16)
                        | ((uint32_t)p->data[6] <<  8) |  (uint32_t)p->data[7];
            FlowReport_NotifyAckSpan(ep, ec);
        } else {
            FlowReport_NotifyAck();          /* consume-on-ACK: commit last batch */
        }
        break;

    case PKT_TIME_SYNC:
        if (p->len == 5u) {
            uint8_t  valid = p->data[0];
            uint32_t t_now = ((uint32_t)p->data[1] << 24) |
                             ((uint32_t)p->data[2] << 16) |
                             ((uint32_t)p->data[3] <<  8) |
                             ((uint32_t)p->data[4]);
            /* Apply LOCO + phase nudge only ONCE per session (a retransmitted
             * TIME_SYNC still gets its 0x0D ack below, but is not re-applied). */
            if (!s_time_synced) {
                s_time_synced = true;
                uint32_t delay_s = (getNowTime() - s_pwr_on_ms) / 1000UL;
                /* ---- V025: keep the capture period in MILLIseconds ----
                 * This was `cap_s = APP_CAPTURE_PERIOD_MS / 1000`, an integer
                 * division. On the bench APP_CAPTURE_PERIOD_MS is 5290, so
                 * cap_s became 5 and every period built from it was 5.5% short:
                 *     ideal = 34 x 5    = 170 s
                 *     real  = 34 x 5.29 = 179.9 s      -> 9.9 s per report
                 * LOCO then saw a phase error that was not in the hardware and
                 * pushed k down to its LOCO_K_MIN clamp trying to remove it. The
                 * bench log shows exactly that: perr 17 -> 26 -> 37 -> 45 while
                 * the measured wake interval was a steady 180 s, and k went
                 * 126 -> 117 -> 111 -> 111 -> 111.
                 *
                 * Production never showed it because 241004/1000 = 241 loses
                 * 0.004 s (0.002%). That is luck, not safety: any capture period
                 * that is not a near-whole number of seconds brings it back. */
                uint32_t cap_ms = (uint32_t)APP_CAPTURE_PERIOD_MS;
                if (cap_ms == 0u) cap_ms = 1u;
                /* true intended period of the cycle that just completed =
                 * its (possibly nudged) capture count x capture period.
                 * Multiply in ms, THEN convert once, rounding to nearest. */
                uint32_t ideal_period_s =
                    (((uint32_t)s_report_captures * cap_ms) + 500UL) / 1000UL;
                /* V024b: capture k BEFORE the cloud evaluation so the log can
                 * show what the evaluation actually decided. Loco.c emits no
                 * debug output of its own (grep: zero DBG_ calls), so until now
                 * the second-stage calibration was a black box: the only view of
                 * it was the RSP_LOCO the Photon asks for once per session, which
                 * shows the RESULT but never the REASONING. Every bench run so
                 * far reported cloud_cal=0, i.e. this path has never run. */
                int16_t k_before = Loco_GetK();

                Loco_OnCloudTime(valid, t_now, delay_s, ideal_period_s, false);

#ifdef APP_DEBUG_EVENT_LOG
                {
                    LocoStatus ls;
                    Loco_GetStatus(&ls);
                    int32_t perr = Loco_GetPhaseErrorS();
                    DBG_STR("[LOCO] cloud v=");  DBG_U32((uint32_t)valid);
                    DBG_STR(" perr=");
                    if (perr < 0) { DBG_STR("-"); DBG_U32((uint32_t)(-perr)); }
                    else          {               DBG_U32((uint32_t)perr);    }
                    DBG_STR("s k=");             DBG_U32((uint32_t)k_before);
                    DBG_STR("->");               DBG_U32((uint32_t)ls.applied_k);
                    DBG_STR(" ccal=");           DBG_U32((uint32_t)ls.cloud_calibrated);
                    DBG_STR(" crng=");           DBG_U32((uint32_t)ls.cloud_in_range);
                    DBG_NL();
                }
#endif

                /* GRID alignment: every report period, the count is DERIVED from
                 * the grid -- captures until the next {anchor, interval} point,
                 * computed from the cloud time we just applied. There is no fixed
                 * "samples per report"; a report that lands early/late is
                 * self-corrected because the next count is recomputed from the
                 * actual grid. The ceiling guards against a bad time. */
                if (valid) {
                    s_first_aligned = true;
                    s_report_captures = Loco_GridCaptures(
                        (uint16_t)(APP_FLOW_SLOTS - APP_FLOW_RING_MARGIN), cap_ms);
                }
                /* if time is invalid this cycle, keep the previously derived count */
                FlowLog_SetNextBatch(s_report_captures);

                /* Header meta for the NEXT RSP_DATA: this report's data began at
                 * the previous wake (the cloud time we just applied is when this
                 * span ENDS; the next span STARTS here).
                 *
                 * V022 / Appendix H.4: interval is derived from the wake count
                 * that will ACTUALLY be loaded into Timer0, not from the nominal
                 * period scaled by k. The old form was
                 *     APP_CAPTURE_PERIOD_MS * k >> 7
                 * which claimed a correction that the hardware never applied: on
                 * the bench APP_WAKE_COUNTS is 1, so Loco_ApplySleep(1) =
                 * (1*126)>>7 = 0 and Sys_Time_SetWakeCounts() clamps it back to
                 * 1 - the wake period stays 5290 ms while the header reported
                 * 5207 ms, contradicting RSP_PHOTON_CFG.captureIntervalMs. The
                 * expression below mirrors Sys_Time_SetWakeCounts() exactly
                 * (same clamps) and then applies the APP_WAKE_ACTUAL_MS formula
                 * to the clamped count, so the two values can no longer diverge.
                 *   bench      : wc=1   -> 529 ms  x10 = 5290 ms  (= CFG value)
                 *   production : wc=112 -> 59194 ms x4 = 236776 ms (k applied) */
                {
                    uint32_t wc = Loco_ApplySleep((uint32_t)APP_WAKE_COUNTS);
                    if (wc == 0u)  wc = 1u;     /* setter's zero clamp          */
                    if (wc > 255u) wc = 255u;   /* 8-bit Timer0 (setter's cast) */
                    uint32_t wake_ms =
                        (uint32_t)((((uint32_t)wc << APP_FLOW_T0CKPS) + 15UL) / 31UL);
                    uint32_t interval_ms =
                        wake_ms * (uint32_t)APP_WAKES_PER_SAMPLE;
                    /* ---- V024b: report the span START, not the sync moment ----
                     * The intent above is right: start_time is where this
                     * report's data BEGINS. That instant is the PREVIOUS
                     * session's cutoff - the boundary frozen when the Photon was
                     * powered on - not the moment that session's TIME_SYNC
                     * happened to arrive.
                     *
                     * The two are not the same. TIME_SYNC lands only after the
                     * Photon has booted, fetched CFG and read the power state:
                     * about 4 s on the current bench. Using the sync epoch put
                     * every reported series ~4 s late - a constant shift that
                     * moves roughly 7% of the volume across each 60 s bucket
                     * boundary.
                     *
                     * So remember how far that session's sync lagged its own
                     * cutoff, and subtract it here. The lag is measured with the
                     * PIC's own clock across a few seconds, where its drift is
                     * negligible - unlike the minutes-long intervals the cloud
                     * time exists to correct.
                     *
                     * NOTE: this whole branch has never executed on hardware -
                     * every bench session to date carried time_valid=0. Verify
                     * the [TIME] line below against the Photon's bucket
                     * placement on the first CLOUD_FAST run. */
                    uint32_t lag_s    = ((uint32_t)s_prev_sync_lag_ms + 500u) / 1000u;
                    uint32_t start_ep = 0u;
                    uint8_t  start_vl = 0u;
                    if (s_prev_wake_valid) {
                        start_ep = (s_prev_wake_epoch > lag_s)
                                 ? (s_prev_wake_epoch - lag_s) : s_prev_wake_epoch;
                        start_vl = 1u;
                    }
                    FlowReport_SetReportMeta(start_ep, start_vl, interval_ms);

                    /* Remember this wake as the START of the NEXT report's span,
                     * together with how far it lagged this session's cutoff. */
                    uint32_t lag_now = getNowTime() - FlowReport_GetCutoffMs();
                    if (lag_now > 60000UL) lag_now = 60000UL;      /* sanity clamp */
                    if (valid) {
                        s_prev_wake_epoch  = t_now;
                        s_prev_wake_valid  = 1u;
                        s_prev_sync_lag_ms = (uint16_t)lag_now;
                    } else {
                        s_prev_wake_valid  = 0u;  /* gap -> next start unknown */
                    }
#ifdef APP_DEBUG_EVENT_LOG
                    DBG_STR("[TIME] sync v="); DBG_U32((uint32_t)valid);
                    DBG_STR(" ep=");           DBG_U32(t_now);
                    DBG_STR(" lag=");          DBG_U32(lag_now);
                    DBG_STR("ms st=");         DBG_U32(start_ep);
                    DBG_STR(" ivl=");          DBG_U32(interval_ms);
                    DBG_STR("ms grid=");       DBG_U32(Loco_GetReportInterval());
                    DBG_STR("s caps=");        DBG_U32((uint32_t)s_report_captures);
                    DBG_NL();
                    /* An invalid sync breaks the time axis: the NEXT report will
                     * carry start_valid=0 and the Photon cannot place buckets for
                     * it. Say so explicitly - CLOUD_FAST injects connect failures,
                     * so this will happen on purpose and must be easy to spot. */
                    if (!valid) {
                        DBG_STR("[TIME] no cloud time - next report start_valid=0"
                                " (buckets cannot be placed for that span)");
                        DBG_NL();
                    }
#endif
                    /* Predict the next wake so the next cloud eval can compare
                     * actual vs expected: expected = now + intended period. */
                    uint32_t next_period_s =
                        (((uint32_t)s_report_captures * cap_ms) + 500UL) / 1000UL;
                    if (valid) {
                        Loco_SetExpectedWake(t_now + next_period_s, next_period_s);
                    }
                }
            }
            FlowReport_SendTimeReceived();   /* dedicated 0x0D ack (always) */
        }
        break;

    case PKT_REQ_GET_PARAM:
        FlowReport_SendParam();              /* RSP_PARAM (8B) */
        break;

    case PKT_REQ_GET_LOCO:
        FlowReport_SendLocoStatus();         /* RSP_LOCO (10B) - bench readout */
        break;

    case PKT_REQ_SET_LEAK:
        /* Leak thresholds only, fixed length 8: 4 x u16 big-endian.
         * The grid moved to its own packet (SET_GRID) because the two are
         * unrelated quantities and carrying both here made the LENGTH mean
         * something, which is what silently broke 16-byte writes. */
        if (p->len == 8u) {
            leak_param_t np;
            np.leak1_counts   = (uint16_t)(((uint16_t)p->data[0] << 8) | p->data[1]);
            np.leak1_window_s = (uint16_t)(((uint16_t)p->data[2] << 8) | p->data[3]);
            np.leak2_counts   = (uint16_t)(((uint16_t)p->data[4] << 8) | p->data[5]);
            np.leak2_window_s = (uint16_t)(((uint16_t)p->data[6] << 8) | p->data[7]);
            /* V023 / fix 2: log what actually happened, not what we asked for.
             * The write can be refused by the build (PIC_USE_OWN_LEAK_PARAMS)
             * OR by FlowControl_SetParams() itself (REPORT_CONFIG_DEBUG keeps
             * its own bench windows). V022 only knew about the first, so the
             * log printed APPLIED while the values stayed unchanged. */
            bool applied;
#ifndef PIC_USE_OWN_LEAK_PARAMS
            applied = FlowControl_SetParams(&np); /* adopt the relayed values  */
#else
            applied = false;                     /* ACK but keep our #defines */
            (void)np;
#endif
            (void)applied;   /* only read by the debug log below */
#ifdef APP_DEBUG_EVENT_LOG
            DBG_STR("[PARM] SET_LEAK a1="); DBG_U32(np.leak1_counts);
            DBG_STR("/");                   DBG_U32(np.leak1_window_s);
            DBG_STR("s a2=");               DBG_U32(np.leak2_counts);
            DBG_STR("/");                   DBG_U32(np.leak2_window_s);
            if (applied) {
                DBG_STR("s -> APPLIED");
            } else {
#ifdef PIC_USE_OWN_LEAK_PARAMS
                DBG_STR("s -> IGNORED (PIC_USE_OWN_LEAK_PARAMS)");
#else
                DBG_STR("s -> IGNORED (REPORT_CONFIG_DEBUG keeps bench params)");
#endif
            }
            DBG_NL();
            {   /* Read back what the leak logic is really using, so the log
                 * proves the write landed rather than just that it was ACKed. */
                leak_param_t cur; FlowControl_GetParams(&cur);
                DBG_STR("[PARM] in use  a1="); DBG_U32(cur.leak1_counts);
                DBG_STR("/");                  DBG_U32(cur.leak1_window_s);
                DBG_STR("s a2=");              DBG_U32(cur.leak2_counts);
                DBG_STR("/");                  DBG_U32(cur.leak2_window_s);
                DBG_STR("s"); DBG_NL();
            }
#endif
            FlowReport_SendAck(PKT_REQ_SET_LEAK);
        } else {
            FlowReport_SendNak(NAK_BAD_LEN);
        }
        break;

    case PKT_REQ_SET_GRID:
        /* Report grid only, fixed length 8: anchor u32 + interval u32, both
         * big-endian. This says WHEN the PIC wakes the Photon; it has nothing
         * to do with leak detection. */
        if (p->len == 8u) {
            uint32_t anchor = ((uint32_t)p->data[0] << 24) | ((uint32_t)p->data[1] << 16)
                            | ((uint32_t)p->data[2] <<  8) |  (uint32_t)p->data[3];
            uint32_t intvl  = ((uint32_t)p->data[4] << 24) | ((uint32_t)p->data[5] << 16)
                            | ((uint32_t)p->data[6] <<  8) |  (uint32_t)p->data[7];
#ifndef PIC_USE_OWN_TIMING
            if (anchor < 86400UL) Loco_SetAnchor(anchor);
            if (intvl  > 0u)      Loco_SetReportInterval(intvl);
#else
            (void)anchor; (void)intvl;           /* ACK but keep our #defines */
#endif
#ifdef APP_DEBUG_EVENT_LOG
            DBG_STR("[PARM] SET_GRID anchor="); DBG_U32(anchor);
            DBG_STR(" interval=");              DBG_U32(intvl);
  #ifndef PIC_USE_OWN_TIMING
            DBG_STR("s -> APPLIED");
  #else
            DBG_STR("s -> IGNORED (PIC_USE_OWN_TIMING)");
  #endif
            DBG_NL();
            DBG_STR("[PARM] in use  anchor="); DBG_U32(Loco_GetAnchor());
            DBG_STR(" interval=");             DBG_U32(Loco_GetReportInterval());
            DBG_STR("s"); DBG_NL();
#endif
            FlowReport_SendAck(PKT_REQ_SET_GRID);
        } else {
            FlowReport_SendNak(NAK_BAD_LEN);
        }
        break;

    case PKT_REQ_GET_VALVE:
        FlowReport_SendValve();              /* RSP_VALVE (8B) */
        break;

    case PKT_REQ_VALVE_UNLOCK:
        if (p->len == 1u) {
            FlowControl_Unlock(p->data[0]);
            FlowReport_SendAck(PKT_REQ_VALVE_UNLOCK);
        } else {
            FlowReport_SendNak(NAK_BAD_LEN);
        }
        break;

    case PKT_SYS_RESET:
        RESET();                             /* no response; clears SRAM */
        break;

    case PKT_PHOTON_OFF_REQ:
        /* Photon says "I'm done, cut my power". No response. main's power
         * machine sees this flag, finishes any in-flight TX, then powers the
         * Photon off and returns to sleep. data[0] = reason (informational).
         * NOTE: during the initial power-hold this flag is ignored by the power
         * machine, so an early OFF_REQ will not cut power until the hold ends. */
        s_off_request = true;
        break;

    case PKT_REQ_POWER_STATE:
        /* Photon asks whether we are still in the initial power-hold window.
         * Reply 0 = INITIAL (stay powered/awake), 1 = NORMAL (run a normal
         * session then let us cut power). */
        FlowReport_SendPowerState(in_initial_hold() ? POWER_STATE_INITIAL
                                                    : POWER_STATE_NORMAL);
        break;

    case PKT_REQ_PHOTON_CFG:
        /* Photon asks for its timing + debug config (App_Config_Photon.h). We
         * always answer (provided=1 or 0) so the Photon never hangs. */
        FlowReport_SendPhotonCfg();
        break;

    case PKT_KEEPALIVE:
        /* Photon "still alive, connecting" heartbeat. The ACTIVE idle timer
         * (s_last_valid_ms) was already reset when this CRC-valid packet was
         * parsed (see on-RX loop), so there is nothing to do and NO response is
         * sent. This explicit case only prevents the default from returning a
         * RSP_NAK(NAK_BAD_FUNC). */
        break;

    default:
        FlowReport_SendNak(NAK_BAD_FUNC);
        break;
    }
}

static void packet_rx_pump(void)
{
    uint8_t b;
    Packet_ParserTimeoutCheck(&s_parser);
    while (UART_RX_Pop(&b)) {
        if (Packet_ParseByte(&s_parser, b)) {
            /* Packet_ParseByte returns true ONLY when CRC-16 matches, so this
             * is a valid packet: the Photon is alive and talking. */
            s_pkt_seen      = true;
            s_last_valid_ms = getNowTime();
            if (!FlowReport_IsSending()) {    /* ignore new req only while mid-stream */
                dispatch_packet(&s_parser);
            }
        }
    }
}



/* ============================================================ */
void __interrupt() isr(void)
{
    UART_ISR();
    Sys_Time_ISR();
}

void main(void)
{
    /* ---- capture reset cause BEFORE anything clears it ----
     * PCON0 flags are active-low (0 = that reset occurred). */
    uint8_t s_pcon0_at_boot = PCON0;   /* keep the raw flags for the report below */
    bool wdt_reset = (PCON0bits.RWDT == 0);
    PCON0 = 0xFFu;                  /* re-arm for the next reset */

    /* A non-WDT reset is treated as a cold power-up: it starts the initial
     * power-hold window (fully powered, no sleep/power-cut for
     * INITIAL_POWER_HOLD_MS). A WDT/soft reset is not a cold boot. */
    s_cold_boot = !wdt_reset;

    /* ---- init ---- */
    LEDs_Init();

    /* Power-up sign of life: 10 toggles at 0.2 s = FIVE visible flashes, in
     * BOTH bench and production builds. This runs before the tick interrupt is
     * enabled, so it uses the compiler delay rather than getNowTime(). It is
     * the first thing an operator sees after applying power - proof that the
     * part booted and is executing - and it costs one second, once, at reset. */
    for (uint8_t i = 0; i < 10u; i++) {
        LED_TEST_TOGGLE;
        CLRWDT();               /* 1 s total is well inside the ~4 s WDT, but
                                 * clear it anyway so the blink can never be
                                 * the thing that trips a reset */
        __delay_ms(200);
    }
    LED_TEST_OFF;               /* leave it off; the FSM owns it from here */
    UART_Init();                 /* UART1 on RC0/RC1: Photon packets        */
    DBG_INIT();            /* UART2 on RA4, configured EXACTLY ONCE */
    DBG_STR("\r\n[BOOT] debug uart up (UART2/RA4)"); DBG_NL();
    UART_RX_SetCallback(on_uart_rx);
    Sys_Time_Init();

    PulseCounter_Init();
    PulseCounter_Enable();
    FlowMeter_Init();
    FlowLog_Init();
    FlowReport_Init();
    LedFsm_Init();
    Loco_Init();                 /* load k from NVM (or default 128) */

    Packet_ParserReset(&s_parser);
    FlowControl_Init();

    /* Valve: non-WDT reset -> optionally force open; WDT reset -> hold. */
    MValve_OP3_Init(/*start_open=*/!wdt_reset);
#if defined(VALVE_PWR_CTRL_ENABLE) && defined(VALVE_ON_WHEN_STARTUP)
  #if BENCH_NO_STARTUP_VALVE
    /* Motor left alone: see BENCH_NO_STARTUP_VALVE above. Leak detection and
     * the valve command path still work; only the one-shot open at boot is
     * skipped, so nothing else about the test changes. */
    DBG_STR("[BOOT] startup valve drive SKIPPED (bench power)"); DBG_NL();
  #else
    if (!wdt_reset) {
        MValve_OP3_CmdOpen();
    }
  #endif
#endif


    INTCON0bits.GIEL = 1;
    INTCON0bits.GIE  = 1;

#ifdef APP_DEBUG_EVENT_LOG
    DBG_STR("[BOOT] ready"); DBG_NL();
#endif

    /* Cold boot (power-up OR software RESET from a SYS_RESET command): make sure
     * the Photon is FULLY powered down first. A SYS_RESET is used e.g. after the
     * cloud changes the local-time offset / {anchor, interval}; the Photon must
     * cold-boot too so it re-detects "PIC first boot" and re-sends the new params
     * + current (offset-corrected) time. Hold power OFF ~3 s so the Photon's rail
     * fully collapses, then power it on for the initial hold. */
    PHOTON_PWR_OFF;                                  /* ensure Photon rail off  */
    UART_TX_Release();               /* V029: nothing to talk to for the next
                                      * PHOTON_COLDBOOT_OFF_MS; do not power the
                                      * divider (or the Photon's RX pin) through
                                      * it. photon_power_on() restores the pin. */
    {
        uint32_t t_off = getNowTime();
#ifdef APP_DEBUG_EVENT_LOG
        DBG_STR("[BOOT] cold: cut Photon "); DBG_U32(PHOTON_COLDBOOT_OFF_MS);
        DBG_STR(" ms"); DBG_NL();

        /* The board was seen restarting over and over inside this wait, so print
         * WHY the last reset happened and HOW FAR the wait gets. PCON0 flags are
         * active-low (0 = that reset occurred); the raw byte is captured at the
         * top of main() before anything clears it.
         *   RWDT=0 -> watchdog     STKOVF/STKUNF=0 -> stack
         *   RMCLR=0 -> MCLR pin    POR/BOR=0       -> power-up / brown-out
         * The progress line then shows whether the millisecond tick is advancing
         * at all: if it stops, the loop can never finish. */
        DBG_STR("[BOOT] reset cause PCON0=0x"); DBG_HEX8(s_pcon0_at_boot);
        DBG_NL();
#endif
        uint32_t next_mark = 1000u;
        while (timeSpan(t_off) < PHOTON_COLDBOOT_OFF_MS) {
            CLRWDT();                                /* keep WDT happy (~3 s)   */
#ifdef APP_DEBUG_EVENT_LOG
            if (timeSpan(t_off) >= next_mark) {
                DBG_STR("[BOOT] cut t="); DBG_U32(timeSpan(t_off)); DBG_NL();
                next_mark += 1000u;
            }
#endif
            DBG_PROCESS();   /* this loop never reaches the main loop for ~3 s,
                              * so keep draining here or the boot lines would
                              * sit in the FIFO until the cut is over */
        }
    }

    /* Cold boot: count 0 is a report period, so power the Photon on right away
     * (it will get 0-2 capture periods of data on this first upload). */
    photon_power_on();

    WDT_START();

    while (1) {
        WDT_KICK();
        DBG_PROCESS();               /* hand one debug byte to UART2 if free */


#ifdef LOCO_CAL_HFINTOSC_ENABLE
        /* Coarse LOCO eval once, early in the initial hold: measure LFINTOSC
         * against the HFINTOSC ms base and set k (+/-1..3%). One-shot. */
        {
            /* ---- LOCO 1st-stage (HFINTOSC) calibration ----
             * The standalone bench test proved a single 500 ms measurement is
             * stable (LF ~ 31.27 kHz, +/-6 Hz). To be robust against a lone
             * glitchy reading we take a few spaced measurements during the hold
             * and average them, then apply once. One-shot. */
            static bool     s_loco_hf_done  = false;
            static uint8_t  s_hf_samples    = 0u;
            static uint32_t s_hf_accum       = 0u;
            static uint32_t s_hf_next_ms      = 2000UL;   /* 1st after ~2 s settle */
            if (!s_loco_hf_done && in_initial_hold() &&
                getNowTime() >= s_hf_next_ms) {
                uint32_t lf = Loco_MeasureLfHz(500u);     /* validated method   */
                if (lf > 0u) { s_hf_accum += lf; s_hf_samples++; }
                s_hf_next_ms = getNowTime() + 1000UL;     /* next ~1 s later     */
                if (s_hf_samples >= LOCO_HF_CAL_SAMPLES) {
                    uint32_t avg = (s_hf_samples > 0u)
                                   ? (s_hf_accum / s_hf_samples) : 0u;
                    Loco_OnHfintoscEval(avg);             /* apply averaged LF   */
#ifdef APP_DEBUG_EVENT_LOG
                    DBG_STR("[LOCO] hf cal LF="); DBG_U32(avg);
                    DBG_STR("Hz k="); DBG_I32(Loco_GetK()); DBG_NL();
#endif
                    s_loco_hf_done = true;
                }
            }
        }
#endif
        MCU_Time_Process();
        /* Contract LED: alive-blink during the initial hold (every Nth capture)
         * and 3 s ON/OFF while an RSP_DATA transmission is streaming. */
        /* V028: the LED now follows the Photon's POWER state, not the RSP_DATA
         * stream. Powering the Photon is the data exchange; the frame itself
         * (0.1 s bench, 1.5 s production) is far too short to ever complete a
         * 3 s half-period. See led_fsm_sysstate.h. */
        LedFsm_Process((s_pwr != PWR_SLEEP), FlowLog_GetCaptureCount());
        MValve_OP3_Process();          /* valve run-out (10 s -> pins LOW)  */
        FlowControl_Process();         /* temp-lock auto-clear -> reopen    */
        FlowLog_Process();             /* capture (runs in ALL power states)*/
        FlowReport_SetInitialMode(in_initial_hold()); /* cold boot -> 0+wipe */
        FlowReport_Process();          /* report-due decision + RSP_DATA TX */
        packet_rx_pump();              /* parse RX, dispatch requests       */

        /* -------- Photon power state machine -------- */
        switch (s_pwr) {

        case PWR_WAIT_FIRST:
            /* If the cold-boot hold has just elapsed while we were still waiting
             * (Photon never established a session), end it: power off and clear
             * the one-shot cold-boot flag so we return to normal operation. */
            if (s_cold_boot && !in_initial_hold()) {
                FlowReport_ClearWakeDue();   /* collect at next normal period */
                photon_power_off();
                s_cold_boot = false;
                break;
            }
            /* Photon is booting. Move to ACTIVE as soon as any valid packet
             * arrives; give up (power off) after the boot timeout - but never
             * cut power during the initial hold window. */
            if (s_pkt_seen) {
                s_last_valid_ms = getNowTime();
                s_pwr           = PWR_ACTIVE;
            } else if (!in_initial_hold() &&
                       timeSpan(s_pwr_on_ms) >= TIMEOUT_NO_MSG_PHOTON2PIC_MS) {
#ifdef APP_DEBUG_EVENT_LOG
                DBG_STR("[PWR ] no reply in "); DBG_U32(TIMEOUT_NO_MSG_PHOTON2PIC_MS);
                DBG_STR(" ms -> assume dead"); DBG_NL();
#endif
                photon_power_off();    /* Photon never answered -> assume dead */
            }
            break;

        case PWR_ACTIVE:
            /* While we are still streaming a response (or bytes remain in the
             * TX buffer), keep the idle timer from expiring. */
            if (!UART_TX_IsEmpty() || FlowReport_IsSending()) {
                s_last_valid_ms = getNowTime();
            }
            if (in_initial_hold()) {
                /* Cold-boot hold: keep the Photon powered no matter what. It
                 * will not send OFF_REQ during the hold; drop any stray one so
                 * nothing is pending when the hold ends. The PIC itself ends
                 * the window (below), by design. */
                s_off_request = false;
            } else if (s_cold_boot) {
                /* The 10-minute cold-boot hold just elapsed. The PIC actively
                 * powers the Photon off (once TX has drained) and clears the
                 * one-shot cold-boot flag, so from now on it reports NORMAL and
                 * runs normal power-gated sessions. */
                if (UART_TX_IsEmpty() && !FlowReport_IsSending()) {
#ifdef APP_DEBUG_EVENT_LOG
                    DBG_STR("[BOOT] initial hold done");
                    if (DBG_DROPPED()) {
                        /* Some log bytes were lost because the FIFO filled.
                         * The measurement itself is unaffected - only the log
                         * is incomplete - but raise DBG_FIFO_SIZE or log less
                         * per pass if this appears. */
                        DBG_STR(" (log FIFO overflowed)");
                    }
                    DBG_NL();
#endif
                    FlowReport_ClearWakeDue();   /* collect at next normal period */
                    photon_power_off();
                    s_cold_boot = false;
                }
            } else if (s_off_request ||
                       (timeSpan(s_last_valid_ms) >= TIMEOUT_NO_MORE_MSG_MS)) {
                /* Normal operation: power off when the Photon asks (OFF_REQ) or
                 * after a long idle (safety net). Drain TX first so a final
                 * response/ack is not truncated. */
                if (UART_TX_IsEmpty() && !FlowReport_IsSending()) {
                    photon_power_off();
                }
            }
            break;

        case PWR_SLEEP:
        default:
            /* Photon is off. If a report period has arrived, power it on.
             * Otherwise sleep between captures (Timer0 wakes us each period);
             * capture still happens on every wake via FlowLog_Process. */
            if (FlowReport_WakeDuePending()) {
#ifdef APP_DEBUG_EVENT_LOG
                /* V024b: show the batch size, not just the moment. In CLOUD_FAIL
                 * the count is the fixed nominal (34 on the bench), but once
                 * cloud time arrives Loco_GridCaptures() derives it from the
                 * {anchor, interval} grid and it VARIES from report to report.
                 * Without it a shifted report time is indistinguishable from a
                 * miscomputed grid. */
                DBG_STR("[RPT ] report due t="); DBG_U32(getNowTime());
                DBG_STR(" caps=");               DBG_U32((uint32_t)s_report_captures);
                DBG_STR(" aligned=");            DBG_U32(s_first_aligned ? 1u : 0u);
                DBG_NL();
#endif
                photon_power_on();
                break;
            }
#ifdef APP_SLEEP_ENABLE
            /* Deep-sleep only when nothing needs the CPU awake. A valve drive
             * or a not-yet-drained TX must finish first. Never deep-sleep
             * during the initial power-hold window (the PIC must stay awake). */
            if (!in_initial_hold() &&
                !MValve_OP3_IsBusy() && UART_TX_IsEmpty() &&
                DBG_IS_IDLE() &&     /* let the last debug byte finish: Fosc
                                      * stops in Sleep and would cut it in half */
                !LedFsm_SleepGate()) {
                /* V028: LedFsm_SleepGate() owns the LED here. It turns the
                 * capture blink off and returns false when sleeping is safe;
                 * it returns true only while APP_LED_CAPTURE_HOLD_MS has not
                 * elapsed yet, which keeps this pass in the main loop so the
                 * flash is long enough to see. */
                /* LOCO-correct the wake period: (ideal * k) >> 7. When both
                 * LOCO switches are off, k stays 128 so this equals the ideal
                 * APP_WAKE_COUNTS (spec behavior).
                 * V022 / H.4: saturate instead of casting. The bare (uint8_t)
                 * cast wrapped silently above 255 (e.g. 260 -> 4, a 100x shorter
                 * wake); it cannot be reached at the present settings, but the
                 * RSP_DATA header now derives its interval from this same
                 * expression, so the two must agree in every case. */
                {
                    uint32_t wc_sleep = Loco_ApplySleep((uint32_t)APP_WAKE_COUNTS);
                    if (wc_sleep > 255u) wc_sleep = 255u;
                    Sys_Time_SetWakeCounts((uint8_t)wc_sleep);
                }
                (void)Sys_Time_EnterDeepSleep();   /* WDT stop/start inside */
                LedFsm_NotifyWake();
            }
#endif
            break;
        }
    }
}