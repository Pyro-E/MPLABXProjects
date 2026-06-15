/*
 * File:   main.c
 * Author: kevinlu
 *
 * Created on May 24, 2026, 12:40 PM
 */


// CONFIG1
// #pragma config FEXTOSC = ECH    // External Oscillator Selection (EC (external clock) above 8 MHz)
// #pragma config RSTOSC = EXTOSC  // Reset Oscillator Selection (EXTOSC operating per FEXTOSC bits (device manufacturing default))

// // CONFIG2
// #pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
// #pragma config PR1WAY = ON      // PRLOCKED One-Way Set Enable bit (PRLOCKED bit can be cleared and set only once)
// #pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
// #pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
// #pragma config FCMENP = ON      // Fail-Safe Clock Monitor - Primary XTAL Enable bit (Fail-Safe Clock Monitor enabled; timer will flag FSCMP bit and OSFIF interrupt on EXTOSC failure.)
// #pragma config FCMENS = ON      // Fail-Safe Clock Monitor - Secondary XTAL Enable bit (Fail-Safe Clock Monitor enabled; timer will flag FSCMP bit and OSFIF interrupt on SOSC failure.)

// // CONFIG3
// #pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
// #pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
// #pragma config MVECEN = ON      // Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
// #pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit (IVTLOCKED bit can be cleared and set only once)
// #pragma config LPBOREN = OFF    // Low Power BOR Enable bit (Low-Power BOR disabled)
// #pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// // CONFIG4
// #pragma config BORV = VBOR_1P9  // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 1.9V)
// #pragma config ZCD = OFF        // ZCD Disable bit (ZCD module is disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
// #pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCKED bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
// #pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
// #pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)
// #pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// // CONFIG5
// #pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
// #pragma config WDTE = ON        // WDT operating mode (WDT enabled regardless of sleep; SWDTEN is ignored)

// // CONFIG6
// #pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
// #pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// // CONFIG7
// #pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
// #pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
// #pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
// #pragma config DEBUG = OFF      // Background Debugger (Background Debugger disabled)

// // CONFIG8
// #pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not Write protected)
// #pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not Write protected)
// #pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not Write protected)
// #pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
// #pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// // CONFIG9
// #pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

// // #pragma config statements should precede project file includes.
// // Use project enums instead of #define for ON and OFF.
// #include <xc.h>

// #define _XTAL_FREQ 4000000
// void main(void) {
//     /* Configure PORTC pin RC1 as digital output and clear latch */
//     ANSELC = 0x00;         // Disable analog on PORTC
//     TRISCbits.TRISC3 = 0;  // RC1 as output for PIC18F16Q40 Curiosity Nano or PIC18F06Q40
//     LATCbits.LATC3 = 0;    // start low

//     while (1) {
//         LATCbits.LATC3 ^= 1; // toggle LED pin using LAT
//         __delay_ms(500);
//     }

//     return;
// }

////////////////////////////////////////////////////////////////////////////////////////////////
// PIC18F06Q40 - LED blink on RC3

// ===== Configuration Bits =====
// #pragma config FEXTOSC = OFF             // External oscillator disabled
// #pragma config RSTOSC = HFINTOSC_64MHZ   // Start with internal oscillator at 64MHz
// #pragma config WDTE = OFF                // Watchdog timer disabled
// #pragma config MCLRE = EXTMCLR           // MCLR pin used as reset
// #pragma config LVP = ON                  // Low-voltage programming enabled

// #include <xc.h>

// #define _XTAL_FREQ 64000000   // For __delay_ms() calculation (64MHz)

// void main(void) {
//     ANSELCbits.ANSELC3 = 0;   // Set RC3 to digital (default is analog - required!)
//     TRISCbits.TRISC3  = 0;    // Set RC3 as output
//     LATCbits.LATC3    = 0;    // Initial state LOW

//     while (1) {
//         LATCbits.LATC3 = !LATCbits.LATC3;  // Toggle RC3
//         __delay_ms(500);                    // Wait 0.5 second
//     }
// }
////////////////////////////////////////////////////////////////////////////////////////////////

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define _XTAL_FREQ 4000000UL

// CONFIG bits: set these in MPLAB X "Configuration Bits" window if preferred.
// These are examples; adjust to your project.
#pragma config FEXTOSC = OFF
#pragma config RSTOSC  = HFINTOSC_1MHZ
#pragma config CLKOUTEN = OFF
#pragma config WDTE = OFF
#pragma config MCLRE = EXTMCLR

// UART baud calculation for PIC18-Q40 UART:
// U1BRG = Fosc / (16 * baud) - 1, depending on BRGS mode.
// For 4 MHz and 9600 baud: 4000000/(16*9600)-1 = ~25
#define UART1_BRG_9600_4MHZ 25

static void OSC_Init_4MHz(void) {
    // Set HFINTOSC to 4 MHz.
    // Register names can vary slightly by header version.
    // If OSCCON1/OSCFRQ compile error, set clock with MCC instead.
    OSCCON1 = 0x60;   // HFINTOSC selected
    OSCFRQ  = 0x02;   // 4 MHz on many Q-series PIC18 headers
}

static void PPS_Unlock(void) {
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x00;
}

static void PPS_Lock(void) {
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x01;
}

static void UART1_Init_TX_RA5_9600(void) {
    // RA5 as digital output
    ANSELAbits.ANSELA5 = 0;
    TRISAbits.TRISA5 = 0;
    LATAbits.LATA5 = 1;    // UART idle high

    // PPS: map UART1 TX to RA5.
    // For many PIC18-Q40 headers, 0x20 is UART1 TX function.
    // If this does not compile or transmit, use MCC Pin Grid:
    // UART1 TX -> RA5 and let MCC generate the exact RA5PPS value.
    PPS_Unlock();
    RA5PPS = 0x20;         // RA5 output source = UART1 TX
    PPS_Lock();

    // Disable UART before configuring
    U1CON1bits.ON = 0;

    // 8-bit async UART, TX enabled, RX disabled
    // Microchip MCC generated code for PIC18-Q40 commonly uses:
    // U1CON0 = 0x30 for async 8-bit, RXEN + TXEN.
    // Since TX-only is needed, TXEN=1 is the key.
    U1CON0 = 0x20;         // TXEN enabled, async 8-bit mode
    U1CON1 = 0x00;
    U1CON2 = 0x00;

    // 9600 baud at 4 MHz
    U1BRGL = (uint8_t)(UART1_BRG_9600_4MHZ & 0xFF);
    U1BRGH = (uint8_t)(UART1_BRG_9600_4MHZ >> 8);

    // FIFO basic setup
    U1FIFO = 0x20;         // TXBE empty status behavior, common MCC setting

    // Clear flags/errors
    U1ERRIR = 0x00;
    U1ERRIE = 0x00;

    // Enable UART
    U1CON1bits.ON = 1;
}

static bool UART1_IsTxReady(void) {
    return (bool)(U1FIFObits.TXBE && U1CON0bits.TXEN);
}

static void UART1_Write(uint8_t b) {
    while (!UART1_IsTxReady()) {
        // wait
    }
    U1TXB = b;
}

static void UART1_WriteString(const char *s) {
    while (*s) {
        UART1_Write((uint8_t)*s++);
    }
}

void main(void) {
    OSC_Init_4MHz();
    UART1_Init_TX_RA5_9600();

    while (1) {
        UART1_WriteString("1\n");
        __delay_ms(1000);
    }
}
