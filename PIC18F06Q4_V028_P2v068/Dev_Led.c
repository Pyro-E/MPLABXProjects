#define DEV_LED_C

#include <xc.h>
#include "Dev_Led.h"

/* ============================================================
 *  Only one real function is needed: pin setup.
 *  All ON/OFF/TOGGLE are macros in the header (single LAT write).
 *
 *  PIC18-Q40 pins power up as ANALOG, so each managed pin must
 *  be set digital (ANSEL=0) and output (TRIS=0) here.
 * ============================================================ */
void LEDs_Init(void)
{
    /* TEST LED : RC3 */
    ANSELCbits.ANSELC3 = 0;   /* digital (REQUIRED on Q40) */
    TRISCbits.TRISC3   = 0;   /* output */

    /* TP1 : RA4 - NOT configured here.
     * RA4 is the debug UART transmit pin (U2TX via PPS). Driving it as a plain
     * GPIO and latching it LOW here fights the PPS routing that DbgUart_Init()
     * sets up later, which is exactly why the stand-alone demo transmitted and
     * the integrated firmware did not: the demo never touched RA4 as a GPIO.
     * Leave the pin alone and let the debug UART own it. */

    /* TP2 : RA5 - NOT configured here, and never driven.
     * RA5 sits right next to RA4, and on this board the two are connected.
     * Making RA5 an output and latching it LOW therefore also pulls RA4 low -
     * and RA4 carries the debug UART transmit signal, so the port went silent.
     * A module isolation test confirmed it: with every module enabled, leaving
     * this one pin alone was enough to make everything work.
     * RA5 is unused by the application, so the safest thing is to not touch it
     * at all: no ANSEL, no TRIS, no LAT. It stays an input and simply follows
     * whatever RA4's UART does. */

    /* Valve CONTROL : RA2 */
    ANSELAbits.ANSELA2 = 0;
    TRISAbits.TRISA2   = 0;

    /* Valve POWER : RC2 */
    ANSELCbits.ANSELC2 = 0;
    TRISCbits.TRISC2   = 0;

    /* Photon2 POWER GATE : RC4 - set the OFF level BEFORE enabling the output so
     * there is no ON glitch at boot (correct for either RC4 polarity). */
    ANSELCbits.ANSELC4 = 0;
    PHOTON2_WAKE_OFF;              /* drive OFF level first */
    TRISCbits.TRISC4   = 0;        /* then enable the output */

    /* start all OFF */
    LED_TEST_OFF;
    /* LED_TP1_OFF removed: RA4 belongs to the debug UART (see above). */
    /* LED_TP2_OFF removed: RA5 is left untouched (see above). */
    VALVE_CTRL_OFF;         /* valve control idles LOW */
    VALVE_PWR_OFF;          /* valve power   idles LOW */
    PHOTON2_WAKE_OFF;       /* Photon starts UNPOWERED (RC4 HIGH); main
                             * powers it on explicitly when a report is due */
}
