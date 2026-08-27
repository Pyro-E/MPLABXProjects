/*
 * Blinky.X / main.c   -   PIC18 lessons
 *
 * ---------------------------------------------------------------------
 * HOW TO USE
 * ---------------------------------------------------------------------
 * Change the LESSON number below, then Build + "Make and Program Device".
 * Only one lesson is compiled at a time, so there is only ever one main().
 *
 *   #define LESSON 4   ->  blink test        (start here)
 *   #define LESSON 5   ->  pulse counter      (do this after 4 works)
 *   #define LESSON 6   ->  UART echo test     ("PIC alive" / "Got: x")
 *   #define LESSON 7   ->  telemetry sender   ("P=38,F=0,V=0")
 *
 * ---------------------------------------------------------------------
 * ICSP WIRING (PICkit  ->  PIC)  - same for every lesson
 * ---------------------------------------------------------------------
 *   PICkit Pin 1  VPP/MCLR  ->  PIC MCLR/VPP
 *   PICkit Pin 2  VDD       ->  PIC VDD
 *   PICkit Pin 3  GND       ->  PIC GND
 *   PICkit Pin 4  PGD       ->  PIC PGD / ICSPDAT
 *   PICkit Pin 5  PGC       ->  PIC PGC / ICSPCLK
 *   PICkit Pin 6  AUX       ->  usually not connected
 * ---------------------------------------------------------------------
 */

#include <xc.h>
#include <stdint.h>
#include <stdio.h>

// ================= CONFIG BITS (apply to all lessons) =================
#pragma config FEXTOSC = OFF              // no external oscillator
#pragma config RSTOSC  = HFINTOSC_64MHZ   // start on internal 64 MHz clock
#pragma config CLKOUTEN = OFF
#pragma config WDTE = OFF                 // watchdog off (no surprise resets)
#pragma config MCLRE = EXTMCLR            // MCLR pin acts as reset
#pragma config MVECEN = OFF               // single-vector interrupts (Lesson 5)

#define _XTAL_FREQ 64000000UL             // must match the clock for __delay_ms()


// ============================ LESSON SELECT ===========================
#define LESSON 7
// ======================================================================


#if LESSON == 4
/* =====================================================================
 * LESSON 4 - First PIC18 lesson: blink test
 * ---------------------------------------------------------------------
 * Goal: prove the whole toolchain works before anything fancy.
 * This confirms:
 *   - MPLAB sees the programmer
 *   - MPLAB sees the PIC18
 *   - ICSP wiring is correct
 *   - the chip can be erased / programmed
 *   - the oscillator is running (delay timing works)
 *   - a GPIO can toggle (the LED)
 *
 * Do NOT add sleep, UART, cloud, or interrupts yet. Just blink.
 *
 * Pin: LED on RC3.  (If your board's LED is on another pin, change RC3.)
 * ===================================================================== */

void main(void) {
    ANSELCbits.ANSELC3 = 0;   // RC3 digital (Q-series pins power up analog)
    TRISCbits.TRISC3   = 0;   // RC3 = output
    LATCbits.LATC3     = 0;   // start LOW (LED off)

    while (1) {
        LATCbits.LATC3 = 1;   // LED ON
        __delay_ms(500);
        LATCbits.LATC3 = 0;   // LED OFF
        __delay_ms(500);
    }
}


#elif LESSON == 5
/* =====================================================================
 * LESSON 5 - Second PIC18 lesson: read a pulse input (meter counter)
 * ---------------------------------------------------------------------
 * Idea:
 *   flow sensor pulse arrives -> PIC interrupt fires -> pulse_count++
 *   main loop reports the total once a second over the UART.
 * We count with an interrupt (not slow polling) so no pulses are missed.
 *
 * Pins (matches the carrier board / product firmware):
 *   RC5 = pulse input  (meter pulse output -> RC5, meter GND -> PIC GND)
 *   RC0 = UART TX      (9600 8N1; PIC RC0 -> Particle Serial1 RX -> USB)
 *   RC3 = LED          (toggles on each pulse, as backup visual proof)
 *
 * Test:
 *   1. Program the PIC. The LED (RC3) toggles on each pulse - that alone
 *      proves counting works, no serial needed.
 *   2. To see "count=" numbers: flash the Particle bridge firmware, then
 *      `particle serial monitor --follow`.
 *   3. Pulse RC5 (or tap RC5 to GND) and watch the count climb.
 *   4. count should never jump back to 0 on its own (that = a reset).
 * ===================================================================== */

#define BAUD 9600UL
// BRGS=1 (/4 mode): BRG = Fosc/(4*baud) - 1 = 64e6/(4*9600)-1 = ~1665
#define BRG  ((uint16_t)(_XTAL_FREQ/(4UL*BAUD) - 1UL))

volatile uint32_t pulse_count = 0;   // shared with the ISR (volatile!)
uint32_t last_report_count = 0;

// ---- UART TX on RC0 (matches the carrier: PIC RC0 -> Particle Serial1 RX) ----
static void uart_init(void) {
    ANSELCbits.ANSELC0 = 0;          // RC0 digital
    TRISCbits.TRISC0   = 0;          // RC0 output
    RC0PPS = 0x10;                   // RC0 -> UART1 TX (0x10 = U1TX on PIC18-Q40)
    U1CON0bits.BRGS = 1;             // high-speed baud (/4)
    U1BRGL = (uint8_t)(BRG & 0xFF);
    U1BRGH = (uint8_t)(BRG >> 8);
    U1CON0bits.MODE = 0;             // async 8-bit
    U1CON0bits.TXEN = 1;             // enable transmitter
    U1CON1bits.ON   = 1;             // turn UART on
}

static void uart_putc(char c) { while (!U1FIFObits.TXBE) { } U1TXB = c; }

// printf() routes each character here, so printf() prints over the UART.
void putch(char c) { uart_putc(c); }

// ---- LED on RC3: visual proof a pulse was counted (no serial needed) ----
static void led_init(void) {
    ANSELCbits.ANSELC3 = 0;          // RC3 digital
    TRISCbits.TRISC3   = 0;          // RC3 output
    LATCbits.LATC3     = 0;          // start off
}

// ---- pulse input on RC5, interrupt on each rising edge ----
static void pulse_init(void) {
    ANSELCbits.ANSELC5 = 0;          // RC5 digital
    TRISCbits.TRISC5   = 1;          // RC5 input
    WPUCbits.WPUC5     = 1;          // weak pull-up so a floating pin reads HIGH
    IOCCPbits.IOCCP5   = 1;          // interrupt on RISING edge of RC5
    IOCCFbits.IOCCF5   = 0;          // clear the pin's flag
    PIE0bits.IOCIE     = 1;          // enable interrupt-on-change
}

// ---- interrupt handler: runs the instant a pulse arrives ----
void __interrupt() isr(void) {
    if (PIR0bits.IOCIF && IOCCFbits.IOCCF5) {
        IOCCFbits.IOCCF5 = 0;        // clear flag so it can fire again
        pulse_count++;               // count this pulse
        LATCbits.LATC3 ^= 1;         // toggle LED as visual proof of a pulse
    }
}

void main(void) {
    led_init();
    uart_init();
    pulse_init();
    INTCON0bits.GIE = 1;             // turn on global interrupts

    printf("pulse counter ready\r\n");

    while (1) {
        // Read pulse_count atomically (pause interrupts for the copy).
        INTCON0bits.GIE = 0;
        uint32_t now = pulse_count;
        INTCON0bits.GIE = 1;

        uint32_t delta = now - last_report_count;
        printf("count=%lu delta=%lu\r\n", (unsigned long)now, (unsigned long)delta);
        last_report_count = now;

        __delay_ms(1000);            // report once per second
    }
}


#elif LESSON == 6
/* =====================================================================
 * LESSON 6 - UART send + receive test ("PIC alive" / echo)
 * ---------------------------------------------------------------------
 * The PIC prints "PIC alive" once a second, and whenever it receives a
 * byte it echoes "Got: <char>". This proves BOTH directions of the UART.
 *
 * Pins (matches the carrier board / product firmware):
 *   RC0 = UART TX  (PIC -> Particle Serial1 RX -> USB)
 *   RC1 = UART RX  (Particle Serial1 TX -> PIC)
 *
 * This pairs with the Particle "UART bridge" firmware (Lesson 10):
 *   PIC TX (RC0) -> Particle Serial1 RX
 *   PIC RX (RC1) <- Particle Serial1 TX
 *   GND          <-> GND
 * View it with the Particle: `particle serial monitor --follow`.
 * ===================================================================== */

#define BAUD 9600UL
// BRGS=1 (/4 mode): BRG = Fosc/(4*baud) - 1 = 64e6/(4*9600)-1 = ~1665
#define BRG  ((uint16_t)(_XTAL_FREQ/(4UL*BAUD) - 1UL))

static void uart_init(void) {
    // --- TX on RC0 (output) ---
    ANSELCbits.ANSELC0 = 0;
    TRISCbits.TRISC0   = 0;
    // --- RX on RC1 (input) ---
    ANSELCbits.ANSELC1 = 0;
    TRISCbits.TRISC1   = 1;

    RC0PPS  = 0x10;                  // RC0 -> UART1 TX (0x10 = U1TX on PIC18-Q40)
    U1RXPPS = 0x11;                  // UART1 RX <- RC1 (port C=2, pin 1 = 0x11)

    U1CON0bits.BRGS = 1;             // high-speed baud (/4)
    U1BRGL = (uint8_t)(BRG & 0xFF);
    U1BRGH = (uint8_t)(BRG >> 8);
    U1CON0bits.MODE = 0;             // async 8-bit
    U1CON0bits.TXEN = 1;             // enable transmitter
    U1CON0bits.RXEN = 1;             // enable receiver
    U1CON1bits.ON   = 1;             // turn UART on
}

static void uart_putc(char c) { while (!U1FIFObits.TXBE) { } U1TXB = c; }

// printf() routes each character here, so printf() prints over the UART.
void putch(char c) { uart_putc(c); }

// True when a received byte is waiting.
static uint8_t UART1_DataReady(void) { return (uint8_t)(!U1FIFObits.RXBE); }

// Read one received byte.
static char UART1_ReadChar(void) { return (char)U1RXB; }

// If a receive error latched (framing / FIFO overflow), clear it so RX
// can recover instead of returning corrupted bytes forever.
static void UART1_ClearErrors(void) {
    if (U1ERRIRbits.RXFOIF) {          // receive FIFO overflow
        U1ERRIRbits.RXFOIF = 0;
    }
    if (U1ERRIRbits.FERIF) {           // framing error: flush the bad byte
        volatile uint8_t junk = U1RXB;
        (void)junk;
    }
}

void main(void) {
    uart_init();

    uint16_t ms = 0;
    while (1) {
        // Check RX every 1 ms (responsive) so bytes are not missed and
        // the 2-byte FIFO never overflows while we wait.
        UART1_ClearErrors();
        if (UART1_DataReady()) {
            char c = UART1_ReadChar();
            printf("Got: %c\r\n", c);
        }

        __delay_ms(1);
        if (++ms >= 1000) {            // once per second
            ms = 0;
            printf("PIC alive\r\n");
        }
    }
}


#elif LESSON == 7
/* =====================================================================
 * LESSON 7 - First integrated LeakSense telemetry sender
 * ---------------------------------------------------------------------
 * (pairs with the Particle "Lesson 11" telemetry firmware)
 *
 * Once a second the PIC sends ONE line describing its state:
 *
 *      P=<pulse_count>,F=<flags>,V=<valve>
 *   e.g.  P=38,F=0,V=0
 *
 * The Particle parses that line and re-publishes it to the cloud as JSON:
 *      { "pulse_count": 38, "flags": 0, "valve": 0 }
 *
 * Field meaning:
 *   P = live pulse count from the meter on RC5
 *   F = status flags  (0 for now - reserved for leak / alarm bits)
 *   V = valve state   (0 = open/idle, 1 = closed - reserved for now)
 *
 * Pins (matches the carrier board / product firmware):
 *   RC5 = pulse input  (meter pulse output -> RC5, meter GND -> PIC GND)
 *   RC0 = UART TX      (9600 8N1; PIC RC0 -> Particle Serial1 RX -> USB)
 *   RC3 = LED          (toggles on each pulse, as backup visual proof)
 *
 * Test:
 *   1. Program the PIC, flash the Particle "Lesson 11" firmware.
 *   2. `particle serial monitor --follow`.
 *   3. See "publish leaksense_telemetry: {...}" once a second.
 *   4. Pulse RC5 (or tap RC5 to GND) and watch pulse_count climb.
 * ===================================================================== */

#define BAUD 9600UL
// BRGS=1 (/4 mode): BRG = Fosc/(4*baud) - 1 = 64e6/(4*9600)-1 = ~1665
#define BRG  ((uint16_t)(_XTAL_FREQ/(4UL*BAUD) - 1UL))

volatile uint32_t pulse_count = 0;   // shared with the ISR (volatile!)
uint8_t g_flags = 0;                 // status flags (reserved: leak/alarm)
uint8_t g_valve = 0;                 // valve state  (0 open, 1 closed)

// ---- UART TX on RC0 (matches the carrier: PIC RC0 -> Particle Serial1 RX) ----
static void uart_init(void) {
    ANSELCbits.ANSELC0 = 0;          // RC0 digital
    TRISCbits.TRISC0   = 0;          // RC0 output
    RC0PPS = 0x10;                   // RC0 -> UART1 TX (0x10 = U1TX on PIC18-Q40)
    U1CON0bits.BRGS = 1;             // high-speed baud (/4)
    U1BRGL = (uint8_t)(BRG & 0xFF);
    U1BRGH = (uint8_t)(BRG >> 8);
    U1CON0bits.MODE = 0;             // async 8-bit
    U1CON0bits.TXEN = 1;             // enable transmitter
    U1CON1bits.ON   = 1;             // turn UART on
}

static void uart_putc(char c) { while (!U1FIFObits.TXBE) { } U1TXB = c; }

// printf() routes each character here, so printf() prints over the UART.
void putch(char c) { uart_putc(c); }

// ---- LED on RC3: visual proof a pulse was counted (no serial needed) ----
static void led_init(void) {
    ANSELCbits.ANSELC3 = 0;          // RC3 digital
    TRISCbits.TRISC3   = 0;          // RC3 output
    LATCbits.LATC3     = 0;          // start off
}

// ---- pulse input on RC5, interrupt on each rising edge ----
static void pulse_init(void) {
    ANSELCbits.ANSELC5 = 0;          // RC5 digital
    TRISCbits.TRISC5   = 1;          // RC5 input
    WPUCbits.WPUC5     = 1;          // weak pull-up so a floating pin reads HIGH
    IOCCPbits.IOCCP5   = 1;          // interrupt on RISING edge of RC5
    IOCCFbits.IOCCF5   = 0;          // clear the pin's flag
    PIE0bits.IOCIE     = 1;          // enable interrupt-on-change
}

// ---- interrupt handler: runs the instant a pulse arrives ----
void __interrupt() isr(void) {
    if (PIR0bits.IOCIF && IOCCFbits.IOCCF5) {
        IOCCFbits.IOCCF5 = 0;        // clear flag so it can fire again
        pulse_count++;               // count this pulse
        LATCbits.LATC3 ^= 1;         // toggle LED as visual proof of a pulse
    }
}

void main(void) {
    led_init();
    uart_init();
    pulse_init();
    INTCON0bits.GIE = 1;             // turn on global interrupts

    while (1) {
        // Read pulse_count atomically (pause interrupts for the copy).
        INTCON0bits.GIE = 0;
        uint32_t now = pulse_count;
        INTCON0bits.GIE = 1;

        // One telemetry line the Particle knows how to parse.
        printf("P=%lu,F=%u,V=%u\r\n",
               (unsigned long)now, (unsigned)g_flags, (unsigned)g_valve);

        __delay_ms(1000);            // send once per second
    }
}

#else
#error "Set LESSON to 4 (blink), 5 (pulse counter), 6 (UART echo), or 7 (telemetry)."
#endif
