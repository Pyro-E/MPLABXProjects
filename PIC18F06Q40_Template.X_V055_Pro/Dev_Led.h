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

/* ---- Photon2 POWER GATE : RC4 ----
 * RC4 controls whether the Photon is powered. There are TWO wiring options,
 * selected by the PHOTON_EN_DRIVE switch below:
 *
 *  DEFAULT (real hardware, P-MOS gate) - PHOTON_EN_DRIVE UNDEFINED:
 *     RC4 = LOW  -> P-MOS ON  -> Photon POWERED (on)
 *     RC4 = HIGH -> P-MOS OFF -> Photon UNPOWERED (off)
 *
 *  BENCH TEST (RC4 wired DIRECTLY to the Photon's active-HIGH EN pin) -
 *  PHOTON_EN_DRIVE DEFINED:
 *     RC4 = HIGH -> Photon ENABLED  (on)
 *     RC4 = LOW  -> Photon DISABLED (off)
 *
 * The rest of the code only uses PHOTON2_WAKE_ON/_OFF (aka PHOTON_PWR_ON/_OFF),
 * so flipping this one switch inverts the physical level everywhere. Default is
 * the real P-MOS hardware; define PHOTON_EN_DRIVE only for the EN-direct rig. */

//#define PHOTON_EN_DRIVE   /* define -> RC4 drives Photon EN directly (active-HIGH) */

#ifdef PHOTON_EN_DRIVE
  #define PHOTON2_WAKE_ON   (LATCbits.LATC4 = 1)   /* EN direct: HIGH -> Photon ON  */
  #define PHOTON2_WAKE_OFF  (LATCbits.LATC4 = 0)   /* EN direct: LOW  -> Photon OFF */
#else
  #define PHOTON2_WAKE_ON   (LATCbits.LATC4 = 0)   /* P-MOS: LOW  -> Photon ON  (real HW) */
  #define PHOTON2_WAKE_OFF  (LATCbits.LATC4 = 1)   /* P-MOS: HIGH -> Photon OFF (real HW) */
#endif
#define PHOTON_PWR_ON     PHOTON2_WAKE_ON
#define PHOTON_PWR_OFF    PHOTON2_WAKE_OFF

/* ---- Init: set all managed pins digital + output, then OFF ---- */
void LEDs_Init(void);

#endif /* DEV_LED_H */
