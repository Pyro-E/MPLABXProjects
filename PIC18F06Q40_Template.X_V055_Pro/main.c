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
#include "MCU_Time.h"
#include "Sys_Time_MCU_Specific.h"
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
static bool        s_cold_boot    = false;  /* set at boot: true on a cold power-up */

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
    PHOTON_PWR_ON;                 /* RC4 LOW -> P-MOS on -> Photon powered */
    s_pwr_on_ms    = getNowTime();
    s_pkt_seen     = false;
    s_off_request  = false;
    s_pwr          = PWR_WAIT_FIRST;
    LedFsm_NotifyWake();
}

static void photon_power_off(void)
{
    PHOTON_PWR_OFF;                /* RC4 HIGH -> P-MOS off -> Photon off   */
    s_pwr = PWR_SLEEP;
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
    /* Human-readable log of every received packet (bench PC, no Photon). */
    print_string("RX: ");
    switch (p->func) {
    case PKT_REQ_DATA:         print_string("REQ_DATA");        break;
    case PKT_REQ_GET_PARAM:    print_string("REQ_GET_PARAM");   break;
    case PKT_REQ_SET_PARAM:    print_string("REQ_SET_PARAM");   break;
    case PKT_REQ_GET_VALVE:    print_string("REQ_GET_VALVE");   break;
    case PKT_REQ_VALVE_UNLOCK: print_string("REQ_VALVE_UNLOCK");break;
    case PKT_SYS_RESET:        print_string("SYS_RESET");       break;
    case PKT_PHOTON_OFF_REQ:   print_string("PHOTON_OFF_REQ r=");
                               print_uint((p->len >= 1u) ? p->data[0] : 0u); break;
    case PKT_REQ_POWER_STATE:  print_string("REQ_POWER_STATE"); break;
    case PKT_REQ_PHOTON_CFG:   print_string("REQ_PHOTON_CFG");  break;
    case PKT_KEEPALIVE:        print_string("KEEPALIVE");       break;
    default:                   print_string("func=0x");
                               print_uint((uint32_t)p->func);   break;
    }
    print_string(" len="); print_uint((uint32_t)p->len);
    print_char('\n');
#endif

    switch (p->func) {

    case PKT_REQ_DATA:
        FlowReport_NotifyAA();               /* RSP_DATA (streamed) */
        break;

    case PKT_REQ_GET_PARAM:
        FlowReport_SendParam();              /* RSP_PARAM (8B) */
        break;

    case PKT_REQ_SET_PARAM:
        if (p->len == 8u) {
            leak_param_t np;
            np.leak1_counts   = (uint16_t)(((uint16_t)p->data[0] << 8) | p->data[1]);
            np.leak1_window_s = (uint16_t)(((uint16_t)p->data[2] << 8) | p->data[3]);
            np.leak2_counts   = (uint16_t)(((uint16_t)p->data[4] << 8) | p->data[5]);
            np.leak2_window_s = (uint16_t)(((uint16_t)p->data[6] << 8) | p->data[7]);
            FlowControl_SetParams(&np);
            FlowReport_SendAck(PKT_REQ_SET_PARAM);
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
    bool wdt_reset = (PCON0bits.RWDT == 0);
    PCON0 = 0xFFu;                  /* re-arm for the next reset */

    /* A non-WDT reset is treated as a cold power-up: it starts the initial
     * power-hold window (fully powered, no sleep/power-cut for
     * INITIAL_POWER_HOLD_MS). A WDT/soft reset is not a cold boot. */
    s_cold_boot = !wdt_reset;

    /* ---- init ---- */
    LEDs_Init();
    UART_Init();
    UART_RX_SetCallback(on_uart_rx);
    Sys_Time_Init();

    PulseCounter_Init();
    PulseCounter_Enable();
    FlowMeter_Init();
    FlowLog_Init();
    FlowReport_Init();
    LedFsm_Init();

    Packet_ParserReset(&s_parser);
    FlowControl_Init();

    /* Valve: non-WDT reset -> optionally force open; WDT reset -> hold. */
    MValve_OP3_Init(/*start_open=*/!wdt_reset);
#if defined(VALVE_PWR_CTRL_ENABLE) && defined(VALVE_ON_WHEN_STARTUP)
    if (!wdt_reset) {
        MValve_OP3_CmdOpen();
    }
#endif

    INTCON0bits.GIEL = 1;
    INTCON0bits.GIE  = 1;

    Debug_Print_String("ready\r\n");

    /* Cold boot: count 0 is a report period, so power the Photon on right away
     * (it will get 0-2 capture periods of data on this first upload). */
    photon_power_on();

    WDT_START();
    while (1) {
        WDT_KICK();
        MCU_Time_Process();
        LedFsm_Process();
        MValve_OP3_Process();          /* valve run-out (10 s -> pins LOW)  */
        FlowControl_Process();         /* temp-lock auto-clear -> reopen    */
        FlowLog_Process();             /* capture (runs in ALL power states)*/
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
                photon_power_on();
                break;
            }
#ifdef APP_SLEEP_ENABLE
            /* Deep-sleep only when nothing needs the CPU awake. A valve drive
             * or a not-yet-drained TX must finish first. Never deep-sleep
             * during the initial power-hold window (the PIC must stay awake). */
            if (!in_initial_hold() &&
                !MValve_OP3_IsBusy() && UART_TX_IsEmpty()) {
                LED_TEST_OFF;
                (void)Sys_Time_EnterDeepSleep();   /* WDT stop/start inside */
                LedFsm_NotifyWake();
            }
#endif
            break;
        }
    }
}
