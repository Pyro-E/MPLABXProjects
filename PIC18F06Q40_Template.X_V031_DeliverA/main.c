/* ============================================================
 *  main.c  -  flow meter top level (PIC18F06Q40)
 *
 *  Layer stack (each is non-blocking, driven from the loop):
 *    PulseCounter     : Timer1 16-bit hardware pulse total on RC5
 *    FlowLog          : ring buffer, captures pulse delta / period,
 *                       stores via the selected compression method
 *    FlowReport       : on each data cycle, wakes Photon2 (RC4),
 *                       then sends a batch over UART, one line/turn
 *    led_fsm_sysstate : LED heartbeat (2 s) / blink burst on cycle
 *
 *  All tunables live in App_Config.h.
 *
 *  Pins:
 *    RC0 = UART TX     RC1 = UART RX
 *    RC3 = TEST LED    RC4 = Photon2 WAKE (out)
 *    RC5 = flow pulse input (Timer1 T1CKI via PPS)
 * ============================================================ */

// ===== Configuration Bits =====
#pragma config FEXTOSC = OFF             // External oscillator disabled
#pragma config RSTOSC = HFINTOSC_64MHZ   // Internal oscillator at 64 MHz
#pragma config MVECEN = OFF              // Single-vector interrupt mode
#pragma config MCLRE = EXTMCLR           // MCLR pin used as reset
#pragma config LVP = ON                  // Low-voltage programming enabled

#include <xc.h>
#include "App_Config.h"

/* Watchdog config depends on APP_WATCHDOG_ENABLE (from App_Config.h above).
 * SWDTEN: WDT turned on/off at runtime via WDTCON0bits.SEN.
 * WDTCPS_12: period ~4.096 s (1:131072 of 31 kHz LFINTOSC).
 * WDTCWS_7: window always open -> a plain (non-windowed) WDT. */
#ifdef APP_WATCHDOG_ENABLE
#pragma config WDTCPS = WDTCPS_12        // WDT period ~4 s
#pragma config WDTE   = SWDTEN           // WDT controlled by SEN bit
#pragma config WDTCWS = WDTCWS_7         // window always open (plain WDT)
#else
#pragma config WDTE   = OFF              // Watchdog disabled
#endif

#include "Dev_Led.h"
#include "Dev_Valve.h"
#include "led_fsm_sysstate.h"
#include "Dev_Uart.h"
#include "Dev_Debug.h"
#include "MCU_Time.h"
#include "Sys_Time_MCU_Specific.h"
#include "PulseCounter.h"
#include "FlowMeter.h"
#include "FlowLog.h"
#include "FlowReport.h"

#define _XTAL_FREQ 64000000

/* UART receive callback (ISR context). Handles host commands. */
static void on_uart_rx(uint8_t ch)
{
    /* 0xAA = "upload now": snapshot and start the report.
     * 0xF0 = "I want to talk": raise WAKE and wait for the 0xAA.
     *        (When asleep, 0xF0 is consumed by the wake edge and never
     *         reaches here; main() requests the report from the wake
     *         cause instead. This path handles 0xF0 received while awake.)
     * Other bytes are ignored. */
    if (ch == 0xAAu) {
        LED_TEST_TOGGLE;
        FlowReport_NotifyAA();
    } else if (ch == 0xF0u) {
        FlowReport_RequestReport();
    }
}

/* Single-vector interrupt handler (MVECEN = OFF). */
void __interrupt() isr(void)
{
    UART_ISR();
    Sys_Time_ISR();
}

void main(void)
{
    /* ---- init ---- */
    LEDs_Init();                       // LED + WAKE (RC4) pins
    UART_Init();
    UART_RX_SetCallback(on_uart_rx);
    Sys_Time_Init();                   // 1 ms system tick (Timer2)

    PulseCounter_Init();               // Timer1 async counter on RC5
    PulseCounter_Enable();
    FlowMeter_Init();                  // pure pulse accounting + total
    FlowLog_Init();                    // periodic capture ring buffer
    FlowReport_Init();                 // wake + report FSM
    LedFsm_Init();                     // LED behavior FSM
    Valve_Init();                      // valve power/control waveform

    /* ---- interrupts on ---- */
    INTCON0bits.GIEL = 1;
    INTCON0bits.GIE  = 1;

    Debug_Print_String("ready\r\n");

    /* ---- cooperative super-loop ---- */
#ifdef APP_SLEEP_ENABLE
    uint32_t wake_ms = getNowTime();   /* time of last wake (dwell timer) */
#endif
    WDT_START();                       /* guard the awake periods (no-op if WDT off) */
    while (1) {
        WDT_KICK();            // pet the watchdog each pass (no-op if WDT off)
        MCU_Time_Process();    // no-op in ISR tick mode
        LedFsm_Process();      // LED heartbeat / blink burst
        Valve_Process();       // valve power/control waveform
        FlowLog_Process();     // capture one sample per period
        FlowReport_Process();  // wake Photon2 + sequential UART report

#ifdef APP_SLEEP_ENABLE
        /* Sleep only when fully idle: no report/wake activity, UART drained,
         * and we have stayed awake at least WAKEUP_TIME_MIN_MS (so the
         * LED-on at wake is visible). LED + valve outputs are driven OFF
         * before sleeping so nothing draws current while asleep. */
        if (!FlowReport_IsBusy() && UART_TX_IsEmpty() &&
            (timeSpan(wake_ms) >= WAKEUP_TIME_MIN_MS)) {

            LED_TEST_OFF;
            VALVE_CTRL_OFF;
            VALVE_PWR_OFF;

            wake_cause_t cause = Sys_Time_EnterDeepSleep();  // sleeps until Timer0 or UART

            LedFsm_NotifyWake();         // LED on, restart heartbeat phase
            wake_ms = getNowTime();      // reset the awake-dwell timer

            /* A UART wake means Photon2 sent 0xF0 -> it wants to talk.
             * Raise WAKE and wait for 0xAA (Timer0 wakes just capture). */
            if (cause == WAKE_BY_UART) {
                FlowReport_RequestReport();
            }
        }
#endif
    }
}
