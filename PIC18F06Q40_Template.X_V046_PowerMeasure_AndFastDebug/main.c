/* ============================================================
 *  main.c  -  flow meter top level (PIC18F06Q40)
 *
 *  Adds to the V032 baseline:
 *    - framed UART packet protocol (Packet.*) replacing raw 0xAA/0xF0 data
 *    - leak detection + OP3 valve auto-shutoff (Flow_Control + MValve_OP3)
 *    - WAKE (RC4) as a "comms-ready" signal: LOW on report-due / 0xF0 /
 *      any RX byte; HIGH (idle) CLOSE_WAKE_AFTER_UART_MS after the last UART activity
 *    - reset-cause aware startup (PCON0): non-WDT reset may force valve open
 *
 *  Pins:
 *    RC0 = UART TX     
 *    RC1 = UART RX
 *    RC3 = TEST LED   
 *    RC4 = Photon2 WAKE (out)
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

/* ============================================================
 *  WAKE (comms-ready) management - kept here with the UART/sleep
 *  code rather than in a separate file.
 *
 *  s_wake_active is the decision variable the sleep guard reads
 *  (NOT the pin). True while WAKE should be LOW (asserted). Set by
 *  report-due / 0xF0 / any RX byte. Cleared (and pin driven HIGH)
 *  as soon as TX shift-register is empty AND FlowReport is idle.
 *  The PIC will not sleep while s_wake_active is true.
 * ============================================================ */
static volatile bool s_wake_active = false;

static void Wake_Raise(void)
{
    s_wake_active = true;
    PHOTON2_WAKE_ON;
}

static void Wake_Process(void)
{
    if (s_wake_active && UART_TX_IsEmpty() && !FlowReport_IsBusy()) {
        s_wake_active = false;
        PHOTON2_WAKE_OFF;
    }
}

/* ============================================================
 *  Packet RX: bytes queued by the ISR; main feeds the parser.
 * ============================================================ */
static pkt_parser_t s_parser;

/* ISR-context RX callback: enqueue the byte and assert comms-ready. The
 * heavy parse/CRC happens in main context. */
static void on_uart_rx(uint8_t ch)
{
    UART_RX_Push(ch);
    s_wake_active = true;        /* any byte -> Photon is talking */
    PHOTON2_WAKE_ON;
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

#ifdef APP_SLEEP_ENABLE
    uint32_t wake_ms = getNowTime();
#endif
    WDT_START();
    while (1) {
        WDT_KICK();
        MCU_Time_Process();
        LedFsm_Process();
        MValve_OP3_Process();          /* valve run-out (10 s -> pins LOW) */
        FlowControl_Process();         /* temp-lock auto-clear -> reopen   */
        FlowLog_Process();             /* capture (feeds leak detector)    */
        FlowReport_Process();          /* WAKE + packet TX (RSP_DATA)      */
        if (FlowReport_WakeDuePending()) {
            Wake_Raise();              /* report due -> assert comms-ready */
        }
        packet_rx_pump();              /* parse RX, dispatch requests      */
        Wake_Process();                /* lower WAKE after quiet period    */

#ifdef APP_SLEEP_ENABLE
        /* Sleep only when truly idle. Guard reads s_wake_active (variable,
         * not the pin). A valve drive in progress also blocks sleep. */
        if (!s_wake_active && !FlowReport_IsBusy() && UART_TX_IsEmpty() &&
            !MValve_OP3_IsBusy()
#if (WAKEUP_TIME_MIN_MS > 0)
            && (timeSpan(wake_ms) >= WAKEUP_TIME_MIN_MS)
#endif
           ) {

            LED_TEST_OFF;
            wake_cause_t cause = Sys_Time_EnterDeepSleep();

            LedFsm_NotifyWake();
            wake_ms = getNowTime();

            if (cause == WAKE_BY_UART) {
                Wake_Raise();
                FlowReport_RequestReport();
            }
        }
#endif
    }
}
