#define DEV_UART_C

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "Dev_Uart.h"
#include "App_Config.h"

/* ============================================================
 *  CONFIGURATION  --  edit to match board / clock
 * ============================================================ */
#define _XTAL_FREQ   APP_FOSC_HZ
#define UART_BAUD    APP_UART_BAUD

/* TX = RC0, RX = RC1 */
#define U1TX_PPS_CODE   0x10u      /* PIC18-Q40 U1TX PPS output source */
#define U1RX_PPS_CODE   0x11u      /* RC1 as PPS input: portC(2)*8 + pin1 = 0x11 */

/* BRG for BRGS=1 (4x): round(Fosc/(4*baud)) - 1 */
#define UART_BRG  ((uint16_t)(((_XTAL_FREQ + (2UL*UART_BAUD)) / (4UL*UART_BAUD)) - 1UL))

/* ============================================================
 *  TX ring buffer (64 bytes)
 * ============================================================ */
static volatile uint8_t  s_tx_buf[UART_TX_BUF_SIZE];
static volatile uint8_t  s_tx_head = 0;   /* write index (producer) */
static volatile uint8_t  s_tx_tail = 0;   /* read index  (consumer = ISR) */
static volatile uint8_t  s_tx_count = 0;  /* bytes currently queued */

static uart_rx_cb_t s_rx_cb = 0;

void UART_RX_SetCallback(uart_rx_cb_t cb)
{
    s_rx_cb = cb;
}

/* ============================================================
 *  Init
 * ============================================================ */
void UART_Init(void)
{
    /* ---- TX pin: RC0, digital output ---- */
    ANSELCbits.ANSELC0 = 0;
    TRISCbits.TRISC0   = 0;
    RC0PPS = U1TX_PPS_CODE;

    /* ---- RX pin: RC1, digital input ---- */
    ANSELCbits.ANSELC1 = 0;
    TRISCbits.TRISC1   = 1;
    U1RXPPS = U1RX_PPS_CODE;

    /* ---- Baud, 4x mode ---- */
    U1CON0bits.BRGS = 1;
    U1BRGL = (uint8_t)(UART_BRG & 0xFF);
    U1BRGH = (uint8_t)(UART_BRG >> 8);

    /* ---- 8-bit async, enable TX + RX ---- */
    U1CON0bits.MODE = 0b0000;
    U1CON0bits.TXEN = 1;
    U1CON0bits.RXEN = 1;
    U1CON1bits.ON   = 1;

    /* ---- TX buffer empty ---- */
    s_tx_head = s_tx_tail = s_tx_count = 0;

    /* ---- RX interrupt on; TX interrupt off until we have data ---- */
    PIE4bits.U1RXIE = 1;
    PIE4bits.U1TXIE = 0;
    /* GIE/GIEL enabled in main */
}

/* ============================================================
 *  ISR worker - call from the global __interrupt()
 *  Handles both RX (byte in) and TX (drain buffer).
 * ============================================================ */
void UART_ISR(void)
{
    /* ---- RX: a byte arrived ---- */
    if (PIE4bits.U1RXIE && PIR4bits.U1RXIF) {
        uint8_t ch = U1RXB;            /* reading clears the flag */
        if (s_rx_cb) {
            s_rx_cb(ch);
        }
    }

    /* ---- TX: hardware ready for another byte ---- */
    if (PIE4bits.U1TXIE && PIR4bits.U1TXIF) {
        if (s_tx_count > 0) {
            U1TXB = s_tx_buf[s_tx_tail];
            s_tx_tail = (uint8_t)((s_tx_tail + 1u) % UART_TX_BUF_SIZE);
            s_tx_count--;
        }
        if (s_tx_count == 0) {
            PIE4bits.U1TXIE = 0;       /* nothing left -> stop TX interrupts */
        }
    }
}

/* ============================================================
 *  TX (non-blocking): enqueue into ring buffer
 * ============================================================ */
bool print_char(char c)
{
    /* buffer full? drop the byte and report failure */
    if (s_tx_count >= UART_TX_BUF_SIZE) {
        return false;
    }

    /* critical section: protect shared indices from the ISR */
    PIE4bits.U1TXIE = 0;
    s_tx_buf[s_tx_head] = (uint8_t)c;
    s_tx_head = (uint8_t)((s_tx_head + 1u) % UART_TX_BUF_SIZE);
    s_tx_count++;
    PIE4bits.U1TXIE = 1;               /* (re)enable TX int -> ISR drains it */

    return true;
}

void print_string(const char *s)
{
    while (*s) {
        print_char(*s++);
    }
}

void print_uint(uint32_t v)
{
    char buf[10];
    int8_t i = 0;

    if (v == 0) {
        print_char('0');
        return;
    }
    while (v > 0) {
        buf[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    while (i > 0) {
        print_char(buf[--i]);
    }
}

void print_int(int32_t v)
{
    if (v < 0) {
        print_char('-');
        print_uint((uint32_t)(-(int64_t)v));
    } else {
        print_uint((uint32_t)v);
    }
}

/* ============================================================
 *  Helpers
 * ============================================================ */
bool UART_TX_IsEmpty(void)
{
    return (s_tx_count == 0);
}

uint8_t UART_TX_Free(void)
{
    return (uint8_t)(UART_TX_BUF_SIZE - s_tx_count);
}
