#ifndef DEV_LED_H
#define DEV_LED_H

/* ============================================================
 *  Dev_Led.h  -  Simple per-pin LED / test-point control
 *                for PIC18F06Q40
 *
 *  No masks, no tables, no loops. Each pin gets its own
 *  ON / OFF / TOGGLE macro that compiles to a single LAT write.
 *
 *  Managed pins:
 *    TEST LED   -> RC3
 *    TP1        -> RA4   (test point)
 *    TP2        -> RA5   (test point)
 *    Valve CTRL -> RA2
 *    Valve PWR  -> RC2
 *    Photon2 WAKE -> RC4
 *
 *  Polarity assumption: HIGH = ON (active-high).
 *  If a pin is wired active-low, swap its ON/OFF below.
 * ============================================================ */

/* ---- TEST LED : RC3 ---- */
#define LED_TEST_ON       (LATCbits.LATC3 = 1)
#define LED_TEST_OFF      (LATCbits.LATC3 = 0)
#define LED_TEST_TOGGLE   (LATCbits.LATC3 ^= 1)

/* ---- Test point 1 : RA4 ---- */
#define LED_TP1_ON        (LATAbits.LATA4 = 1)
#define LED_TP1_OFF       (LATAbits.LATA4 = 0)
#define LED_TP1_TOGGLE    (LATAbits.LATA4 ^= 1)

/* ---- Test point 2 : RA5 ---- */
#define LED_TP2_ON        (LATAbits.LATA5 = 1)
#define LED_TP2_OFF       (LATAbits.LATA5 = 0)
#define LED_TP2_TOGGLE    (LATAbits.LATA5 ^= 1)

/* ---- Valve CONTROL : RA2 (active-high) ---- */
#define VALVE_CTRL_ON       (LATAbits.LATA2 = 1)
#define VALVE_CTRL_OFF      (LATAbits.LATA2 = 0)
#define VALVE_CTRL_TOGGLE   (LATAbits.LATA2 ^= 1)

/* ---- Valve POWER : RC2 (active-high) ---- */
#define VALVE_PWR_ON        (LATCbits.LATC2 = 1)
#define VALVE_PWR_OFF       (LATCbits.LATC2 = 0)
#define VALVE_PWR_TOGGLE    (LATCbits.LATC2 ^= 1)

/* ---- Photon2 WAKE : RC4 (GPIO output, active-high) ----
 * From the PIC's view this is just an output, so it's defined
 * here like an LED. No toggle: WAKE is driven explicitly ON/OFF. */
#define PHOTON2_WAKE_ON   (LATCbits.LATC4 = 1)
#define PHOTON2_WAKE_OFF  (LATCbits.LATC4 = 0)

/* ---- Init: set all managed pins digital + output, then OFF ---- */
void LEDs_Init(void);

#endif /* DEV_LED_H */
