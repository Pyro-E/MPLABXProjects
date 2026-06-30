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

    /* TP1 : RA4 */
    ANSELAbits.ANSELA4 = 0;
    TRISAbits.TRISA4   = 0;

    /* TP2 : RA5 */
    ANSELAbits.ANSELA5 = 0;
    TRISAbits.TRISA5   = 0;

    /* Valve CONTROL : RA2 */
    ANSELAbits.ANSELA2 = 0;
    TRISAbits.TRISA2   = 0;

    /* Valve POWER : RC2 */
    ANSELCbits.ANSELC2 = 0;
    TRISCbits.TRISC2   = 0;

    /* Photon2 WAKE : RC4 */
    ANSELCbits.ANSELC4 = 0;
    TRISCbits.TRISC4   = 0;

    /* start all OFF */
    LED_TEST_OFF;
    LED_TP1_OFF;
    LED_TP2_OFF;
    VALVE_CTRL_OFF;         /* valve control idles LOW */
    VALVE_PWR_OFF;          /* valve power   idles LOW */
    PHOTON2_WAKE_OFF;       /* WAKE idles HIGH (active-low; deasserted) */
}
