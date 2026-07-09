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

/* ---- Photon2 POWER GATE : RC4 (GPIO output, ACTIVE-LOW) ----
 * RC4 no longer connects to the Photon's D10 "wake" input. It now drives the
 * gate of an external P-MOS that switches the Photon's SUPPLY:
 *     RC4 = LOW  -> P-MOS ON  -> Photon POWERED (turned on)
 *     RC4 = HIGH -> P-MOS OFF -> Photon UNPOWERED (turned off)
 * The sense is inverted vs. the old comms-ready model. The PIC decides when
 * the Photon runs by powering it; the Photon cannot wake itself and is only
 * (re)started when this power is re-applied.
 *
 * PHOTON2_WAKE_ON / _OFF keep their names (ON = "make comms possible",
 * OFF = "stop comms") so the rest of the code needs no change, but now
 * ON = power the Photon = RC4 LOW. PHOTON_PWR_ON/_OFF are clearer aliases. */
#define PHOTON2_WAKE_ON   (LATCbits.LATC4 = 0)   /* power Photon ON  (P-MOS on)  */
#define PHOTON2_WAKE_OFF  (LATCbits.LATC4 = 1)   /* power Photon OFF (P-MOS off) */
#define PHOTON_PWR_ON     PHOTON2_WAKE_ON
#define PHOTON_PWR_OFF    PHOTON2_WAKE_OFF

/* ---- Init: set all managed pins digital + output, then OFF ---- */
void LEDs_Init(void);

#endif /* DEV_LED_H */
