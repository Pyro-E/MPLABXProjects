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
         * Photon off and returns to sleep. data[0] = reason (informational). */
        s_off_request = true;
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
            /* Photon is booting. Move to ACTIVE as soon as any valid packet
             * arrives; give up (power off) after the boot timeout. */
            if (s_pkt_seen) {
                s_last_valid_ms = getNowTime();
                s_pwr           = PWR_ACTIVE;
            } else if (timeSpan(s_pwr_on_ms) >= TIMEOUT_NO_MSG_PHOTON2PIC_MS) {
                photon_power_off();    /* Photon never answered -> assume dead */
            }
            break;

        case PWR_ACTIVE:
            /* While we are still streaming a response (or bytes remain in the
             * TX buffer), keep the idle timer from expiring. */
            if (!UART_TX_IsEmpty() || FlowReport_IsSending()) {
                s_last_valid_ms = getNowTime();
            }
            /* Power off when the Photon asks (OFF_REQ) or after a long idle
             * (safety net). Only actually cut power once TX has drained, so a
             * final response/ack is not truncated. */
            if (s_off_request ||
                (timeSpan(s_last_valid_ms) >= TIMEOUT_NO_MORE_MSG_MS)) {
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
             * or a not-yet-drained TX must finish first. */
            if (!MValve_OP3_IsBusy() && UART_TX_IsEmpty()) {
                LED_TEST_OFF;
                (void)Sys_Time_EnterDeepSleep();   /* WDT stop/start inside */
                LedFsm_NotifyWake();
            }
#endif
            break;
        }
    }
}
