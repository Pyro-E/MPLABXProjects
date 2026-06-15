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
#pragma config WDTE = OFF                // Watchdog timer disabled
#pragma config MCLRE = EXTMCLR           // MCLR pin used as reset
#pragma config LVP = ON                  // Low-voltage programming enabled

#include <xc.h>
#include "App_Config.h"
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

#define _XTAL_FREQ 64000000

/* UART receive callback (ISR context). Reserved for future
 * commands from the host / Photon2. */
static void on_uart_rx(uint8_t ch)
{
    (void)ch;
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

    /* ---- interrupts on ---- */
    INTCON0bits.GIEL = 1;
    INTCON0bits.GIE  = 1;

    Debug_Print_String("ready\r\n");

    /* ---- cooperative super-loop ---- */
    while (1) {
        MCU_Time_Process();    // no-op in ISR tick mode
        LedFsm_Process();      // LED heartbeat / blink burst
        FlowLog_Process();     // capture one sample per period
        FlowReport_Process();  // wake Photon2 + sequential UART report
    }
}
