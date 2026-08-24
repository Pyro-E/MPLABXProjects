#define PULSECOUNTER_C

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "PulseCounter.h"

/* ============================================================
 *  Timer1 as a 16-bit ASYNCHRONOUS external event counter.
 *
 *  Pulse input : RC5  -> T1CKI (via PPS, T1CKIPPS = 0x15)
 *  Clock source: T1CKIPPS pin  (T1CLK CS = 0)
 *  Sync        : NOT_SYNC = 1  -> asynchronous -> counts in Sleep
 *  Prescaler   : 1:1           -> every pulse counted
 *  RD16        : 1             -> atomic 16-bit read of TMR1H:TMR1L
 *
 *  PPS input code for RC5 = portC(2)*8 + 5 = 0x15
 * ============================================================ */

#define T1CKIPPS_RC5    0x15u

void PulseCounter_Init(void)
{
    /* ---- RC5 as digital input for the pulse signal ---- */
    ANSELCbits.ANSELC5 = 0;     /* digital (REQUIRED on Q40) */
    TRISCbits.TRISC5   = 1;     /* input */
    T1CKIPPS = T1CKIPPS_RC5;    /* route RC5 -> Timer1 clock input */

    /* ---- Timer1 off while configuring ---- */
    T1CONbits.ON = 0;

    /* ---- Gate disabled (count continuously, no gating) ---- */
    T1GCON = 0x00;

    /* ---- Clock source = external T1CKI pin ---- */
    T1CLK = 0x00;               /* CS = 0 -> T1CKIPPS */

    /* ---- T1CON: RD16=1, NOT_SYNC=1 (async), CKPS=1:1, ON=0 ----
     * bit1 RD16=1, bit2 NOT_SYNC=1 -> 0b0000_0110 = 0x06 */
    T1CON = 0x06;

    /* ---- start at zero, stopped ---- */
    TMR1H = 0;
    TMR1L = 0;
}

void PulseCounter_Enable(void)
{
    T1CONbits.ON = 1;           /* allow counting */
}

void PulseCounter_Pause(void)
{
    T1CONbits.ON = 0;           /* stop; current count preserved */
}

void PulseCounter_Clear(void)
{
    /* With RD16=1 the high byte is buffered: writing TMR1H loads a
     * buffer, writing TMR1L commits both bytes at once. */
    TMR1H = 0;
    TMR1L = 0;
}

uint16_t PulseCounter_Get(void)
{
    /* With RD16=1, reading TMR1L latches TMR1H, so the pair is
     * coherent even while counting asynchronously. */
    uint8_t lo = TMR1L;
    uint8_t hi = TMR1H;
    return (uint16_t)(((uint16_t)hi << 8) | lo);
}

uint16_t PulseCounter_GetAndClear(void)
{
    /* NOTE: tiny race -- a pulse arriving between the read and the
     * clear would be lost. For exact totals the higher layer may
     * prefer reading the free-running count and tracking deltas
     * instead of clearing. Provided here for convenience. */
    uint16_t v = PulseCounter_Get();
    PulseCounter_Clear();
    return v;
}

bool PulseCounter_IsEnabled(void)
{
    return (bool)T1CONbits.ON;
}
