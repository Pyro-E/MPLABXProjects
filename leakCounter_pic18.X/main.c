/*
 * File:   main.c
 * Author: kevinlu
 *
 * Created on June 10, 2026, 11:27 AM
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <xc.h>

#define _XTAL_FREQ 1000000UL

//// CONFIG

#pragma config FEXTOSC = OFF         // External Oscillator Selection (internal oscillator only)
#pragma config RSTOSC = HFINTOSC_1MHZ // Reset Oscillator Selection (HFINTOSC @ 1 MHz)
#pragma config CLKOUTEN = OFF        // Clock Out Enable bit (CLKOUT function disabled)
#pragma config PR1WAY = ON           // PRLOCKED One-Way Set Enable bit
#pragma config CSWEN = ON            // Clock Switch Enable bit (writing to NOSC and NDIV allowed)
#pragma config FCMEN = ON            // Fail-Safe Clock Monitor Enable bit
#pragma config FCMENP = ON           // Fail-Safe Clock Monitor - Primary XTAL Enable bit
#pragma config FCMENS = ON           // Fail-Safe Clock Monitor - Secondary XTAL Enable bit

#pragma config MCLRE = EXTMCLR       // MCLR Enable bit
#pragma config PWRTS = PWRT_OFF      // Power-up timer selection bits (PWRT disabled)
#pragma config MVECEN = OFF          // Multi-vector enable bit (use single-vector ISR)
#pragma config IVT1WAY = ON          // IVTLOCK one-way set enable bit
#pragma config LPBOREN = OFF         // Low Power BOR disabled
#pragma config BOREN = SBORDIS       // Brown-out Reset enabled, SBOREN ignored

#pragma config BORV = VBOR_1P9       // Brown-out Reset Voltage Selection bits
#pragma config ZCD = OFF             // ZCD Disable bit
#pragma config PPS1WAY = ON          // PPSLOCK one-way set enable bit
#pragma config STVREN = ON           // Stack Full/Underflow Reset enabled
#pragma config LVP = ON              // Low Voltage Programming enabled
#pragma config XINST = OFF           // Extended Instruction Set disabled

#pragma config WDTCPS = WDTCPS_31    // WDT Period selection bits
#pragma config WDTE = ON             // WDT enabled regardless of sleep
#pragma config WDTCWS = WDTCWS_7     // WDT Window Select bits
#pragma config WDTCCS = SC           // WDT input clock selector

#pragma config BBSIZE = BBSIZE_512   // Boot Block Size selection bits
#pragma config BBEN = OFF            // Boot Block disabled
#pragma config SAFEN = OFF           // Storage Area Flash disabled
#pragma config DEBUG = OFF           // Background Debugger disabled

#pragma config WRTB = OFF            // Boot Block Write Protection disabled
#pragma config WRTC = OFF            // Configuration Register Write Protection disabled
#pragma config WRTD = OFF            // Data EEPROM Write Protection disabled
#pragma config WRTSAF = OFF          // SAF Write Protection disabled
#pragma config WRTAPP = OFF          // Application Block write protection disabled

#pragma config CP = OFF              // PFM and Data EEPROM Code Protection disabled

// //
// // ===================== PIN MAP ==================== // Pin mapping banner
// // RA5 = PIC_DATA  -> Photon D3 // Serial data output to Photon
// // RA4 = PIC_CLK   <- Photon D2 // Clock input from Photon
// // RC0 = FLOW_PULSE_CLEAN input // Flow pulse input pin
// // RA0 = PIC_WAKE  -> Photon D10/WKP // Wake signal output to Photon
// // RC1 = STATUS_LED output // Curiosity Nano on-board LED
//

#define PIC_DATA_TRIS TRISAbits.TRISA5
#define PIC_DATA_LAT LATAbits.LATA5
#define PIC_DATA_PORT PORTAbits.RA5

#define PIC_CLK_TRIS TRISAbits.TRISA4
#define PIC_CLK_PORT PORTAbits.RA4

#define FLOW_TRIS TRISCbits.TRISC0
#define FLOW_PORT PORTCbits.RC0
#define FLOW_WPU WPUCbits.WPUC0

#define PIC_WAKE_TRIS TRISAbits.TRISA0
#define PIC_WAKE_LAT LATAbits.LATA0

#define STATUS_LED_TRIS TRISCbits.TRISC1
#define STATUS_LED_LAT LATCbits.LATC1

#define TEST_MODE_FIXED_PACKET 0u
#define TEST_PACKET_INTERVAL_SECONDS 10u
#define TEST_PACKET_COUNT 0x1234u
#define TEST_PACKET_FLAGS 0xA5u
#define START_WAIT_TIMEOUT_MS 5000u
#define ACK_WAIT_TIMEOUT_MS 1000u     // ~2.6ms/byte * 10 pairs * 3 bytes/pair = 60 bytes => 156ms at 120us/bit + margin
#define RETRY_COOLDOWN_MS 5u

// High-frequency input support (up to 100 Hz)
#define INPUT_DEBOUNCE_MS 2u    // Debounce period for noisy input
#define PULSE_HOLD_TIME_MS 5u   // Minimum pulse width to count

// 10-minute rolling buffer: one entry per minute, 10 slots, resets every 10 minutes
#define TOGGLE_BUFFER_PAIRS 10
#define TOGGLE_BUFFER_BYTES (TOGGLE_BUFFER_PAIRS * 3)

// 24 bits per pair: 4-bit interval (0-9) + 20-bit count (uint16_t in low 16)
// byte[0] = (interval & 0x0F) << 4   top nibble = interval, lower nibble = 0
// byte[1] = count >> 8
// byte[2] = count & 0xFF
typedef struct {
  uint8_t buf[TOGGLE_BUFFER_BYTES];
  uint8_t pair_count;       // entries stored this window (0 to TOGGLE_BUFFER_PAIRS)
  uint16_t minute_elapsed;  // absolute minutes since session start
} toggle_meter_t;

volatile uint16_t pulse_count = 0;
volatile uint8_t current_pair[3] = {0};  // latest packed (interval, count) frame
volatile uint8_t minute_ready = 0;
volatile uint8_t sec_ticks = 0;
volatile uint8_t interval_count = 0;     // 0-9, position within 10-minute window
volatile uint8_t led_pulse_ticks = 0;
volatile uint16_t tx_retry_cooldown_ms = 0;
volatile uint8_t blink_ms_counter = 0;
volatile toggle_meter_t toggle_meter = {0};
volatile uint8_t print_buf_requested = 0;

// 24 bits per pair: 4-bit interval (0-9) + 20-bit count (uint16_t in low 16)
// byte[0] = (interval & 0x0F) << 4   upper nibble = interval, lower nibble = 0
// byte[1] = count >> 8
// byte[2] = count & 0xFF
static void pack_meter_pair(uint8_t interval, uint16_t count, uint8_t *buf) {
  buf[0] = (uint8_t)((interval & 0x0F) << 4);
  buf[1] = (uint8_t)(count >> 8);
  buf[2] = (uint8_t)(count & 0xFF);
}

static void unpack_meter_pair(const uint8_t *buf, uint8_t *interval, uint16_t *count) {
  *interval = buf[0] >> 4;
  *count    = ((uint16_t)buf[1] << 8) | buf[2];
}

// Write snapshot into fixed slot buf[interval*3]; interval is the direct index (0-9)
static void store_meter_snapshot(uint8_t interval, uint16_t count) {
  pack_meter_pair(interval, count, (uint8_t *)&toggle_meter.buf[interval * 3]);
  toggle_meter.pair_count = interval + 1;
}

static void toggle_meter_init(void) {
  toggle_meter.pair_count = 0;
  toggle_meter.minute_elapsed = 0;
}

static void toggle_meter_reset(void) {
  toggle_meter.pair_count = 0;
}

static void queue_test_packet(void) {
  pack_meter_pair(interval_count, TEST_PACKET_COUNT & 0xFFFF, (uint8_t *)current_pair);
  minute_ready = 1;
  led_pulse_ticks = 120;
  STATUS_LED_LAT = 1;
}

static void clock_init(void) {
  OSCCON1 = 0x60; // HFINTOSC, divider 1
  OSCFRQ = 0x03;  // 1 MHz
}

static void gpio_init(void) {

  // Disable analog
  ANSELA = 0x00;
  ANSELC = 0x00;

  // Disable ADC/comparator/DAC that can steal pins
  ADCON0bits.ADON = 0;
  CM1CON0 = 0x00;
//   DAC1CON0 = 0x00;

  // Ensure push-pull outputs (not open-drain)
  ODCONA = 0x00;

  // Optional: disable slew-rate limiting
  SLRCONA = 0x00;

  // Clear latches first
  LATA = 0x00;
  LATC = 0x00;

  // Directions 
// RA5 = PIC_DATA → Particle D3
// RA4 = PIC_CLK ← Particle D2
// RA0 = PIC_WAKE → Particle D10 / WKP
// RC0 = FLOW_PULSE_CLEAN input (flow pulse sensor input on the PIC)
  TRISAbits.TRISA5 = 0; // DATA out
  TRISAbits.TRISA4 = 1; // CLK in
  TRISCbits.TRISC0 = 1; // FLOW in
  TRISAbits.TRISA0 = 0; // WAKE out
  TRISCbits.TRISC1 = 0; // LED out

  // Pull-ups
  WPUA = 0x00;
  WPUC = 0x00;

  PIC_DATA_TRIS = 0;
  PIC_DATA_LAT = 0;

  PIC_CLK_TRIS = 1;

  FLOW_TRIS = 1;
  FLOW_WPU = 0; // set to 1 only if you need internal pullup

  PIC_WAKE_TRIS = 0;
  PIC_WAKE_LAT = 0;

  STATUS_LED_TRIS = 0;
  STATUS_LED_LAT = 1;
}

static void ioc_init(void) {
  // Count rising edges on RC0
  IOCCPbits.IOCCP0 = 1;
  IOCCNbits.IOCCN0 = 0;
  IOCCFbits.IOCCF0 = 0;
}

static void timer1_init_1s(void) {
  // Fosc = 1 MHz => instruction clock = 250 kHz
  // Timer1 clock = Fosc/4 = 250 kHz
  // Prescaler 1:8 => 31.25 kHz
  // 1 second = 31250 counts
  // preload = 65536 - 31250 = 34286 = 0x85EE

  // Configure Timer1 using device-specific bit names
  TMR1CONbits.TMR1ON = 0;            // ensure Timer1 off while configuring
  TMR1CONbits.T1CKPS0 = 1;          // prescaler bit 0
  TMR1CONbits.T1CKPS1 = 1;          // prescaler bit 1 => 1:8
  TMR1CONbits.nT1SYNC = 1;          // not synchronized (matches previous T1SYNC = 0)
  TMR1H = 0x85;
  TMR1L = 0xEE;
  PIR3bits.TMR1IF = 0;
  PIE3bits.TMR1IE = 1;
  TMR1CONbits.TMR1ON = 1;
}

static void timer1_reload_1s(void) {
  TMR1H = 0x85;
  TMR1L = 0xEE;
}

static void pps_init(void) {
  // not needed here since we are bit-banging
}

// UART1 TX on RF0 at 9600 baud (Curiosity Nano CDC port)
// BRGS=1 (4x mode): BRG = 1MHz/(4*9600) - 1 = 25 => 9615 baud (0.16% error)
static void uart_init(void) {
  ANSELBbits.ANSELB5 = 0;   // RB5 digital (nEDBG CDC TX on Curiosity Nano)
  TRISBbits.TRISB5 = 0;     // RB5 output
  RB5PPS = 0x20;             // route U1TX to RB5

  U1CON0bits.BRGS = 1;      // 4x baud rate divisor
  U1BRGL = 25;
  U1BRGH = 0;
  U1CON0bits.TXEN = 1;      // enable TX
  U1CON0bits.RXEN = 0;
  U1CON0bits.MODE = 0;      // async 8-bit, no parity
  U1CON1bits.ON = 1;        // enable UART
}

void putch(char c) {
  while (!PIR4bits.U1TXIF);  // wait for TX buffer empty
  U1TXB = c;
}

static void print_buffer(void) {
  uint8_t n = toggle_meter.pair_count;
  uint8_t interval;
  uint16_t count;

  printf("min=%u:", (n > 0) ? (uint8_t)(n - 1) : 0);
  for (uint8_t i = 0; i < n; i++) {
    unpack_meter_pair((const uint8_t *)&toggle_meter.buf[i * 3], &interval, &count);
    printf(" %u=%u", interval, count);
  }
  printf("\r\n");
}

static void interrupt_init(void) {
  PIR0bits.IOCIF = 0;
  PIE0bits.IOCIE = 1;
  INTCON0bits.GIEL = 1; // enable peripheral interrupts (PEIE)
  INTCON0bits.GIE = 1;  // global interrupt enable
}

static void send_bit(uint8_t b) {
  while (PIC_CLK_PORT != 0) {
    ;
  } // wait for CLK low
  PIC_DATA_LAT = b ? 1 : 0; // drive DATA before rising edge
  __delay_us(5);            // setup time
  while (PIC_CLK_PORT == 0) {
    ;
  } // wait for CLK high (Particle samples here)
  __delay_us(5); // hold time
  while (PIC_CLK_PORT != 0) {
    ;
  } // wait for CLK low (bit complete)
}

static void send_byte(uint8_t v) {
  for (uint8_t i = 0; i < 8; i++) {
    send_bit((v & 0x80) != 0);
    v <<= 1;
  }
}

static uint8_t wait_for_photon_start(uint16_t timeout_ms) {
  while (timeout_ms--) {
    if (PIC_CLK_PORT != 0) {
      while (PIC_CLK_PORT != 0) {
        ;
      }
      return 1;
    }
    __delay_ms(1);
  }
  return 0;
}

static uint8_t wait_for_ack_8clocks(uint16_t timeout_ms) {
  uint8_t count = 0;

  while (timeout_ms && count < 8) {
    if (PIC_CLK_PORT == 0) {
      __delay_us(20);
      if (PIC_CLK_PORT != 0) {
        while (PIC_CLK_PORT != 0) {
          ;
        }
        count++;
        continue;
      }
    }
    __delay_ms(1);
    timeout_ms--;
  }

  return (count == 8);
}

// Send full buffer to Boron: 1 byte pair_count, then pair_count*3 bytes of packed pairs.
// Boron decodes interval = buf[0]>>4, count = (buf[1]<<8)|buf[2] for each pair.
static uint8_t service_particle_send_packed(void) {
  uint8_t gie_was_enabled = INTCON0bits.GIE;
  INTCON0bits.GIE = 0;

  PIC_WAKE_LAT = 1;
  __delay_ms(2);

  if (!wait_for_photon_start(START_WAIT_TIMEOUT_MS)) {
    PIC_WAKE_LAT = 0;
    INTCON0bits.GIE = gie_was_enabled;
    return 0;
  }

  uint8_t n = toggle_meter.pair_count;
  send_byte(n);
  for (uint8_t i = 0; i < n * 3u; i++) {
    send_byte((uint8_t)toggle_meter.buf[i]);
  }

  PIC_DATA_LAT = 0;
  if (!wait_for_ack_8clocks(ACK_WAIT_TIMEOUT_MS)) {
    PIC_WAKE_LAT = 0;
    INTCON0bits.GIE = gie_was_enabled;
    return 0;
  }

  PIC_WAKE_LAT = 0;
  minute_ready = 0;
  INTCON0bits.GIE = gie_was_enabled;
  return 1;
}

void __interrupt() isr(void) {
  if (PIR0bits.IOCIF) {
    if (IOCCFbits.IOCCF0) {
      // Optimized for high-frequency input (up to 100 Hz)
      // Minimal ISR processing: just count pulses
      pulse_count++;
      // Clear flag
      IOCCFbits.IOCCF0 = 0;
    }
    PIR0bits.IOCIF = 0;
  }

  if (PIR3bits.TMR1IF) {
    PIR3bits.TMR1IF = 0;
    timer1_reload_1s();

    sec_ticks++;

    // At minute boundary: pack, store, send, then advance interval and reset counter
    if (sec_ticks >= 60) {
      sec_ticks = 0;
      pack_meter_pair(interval_count, pulse_count, (uint8_t *)current_pair);
      store_meter_snapshot(interval_count, pulse_count);
      minute_ready = 1;
      print_buf_requested = 1;
      pulse_count = 0;
      interval_count++;
      if (interval_count >= 10) {
        interval_count = 0;
      }
      toggle_meter.minute_elapsed++;
    }

#if TEST_MODE_FIXED_PACKET
    if (!minute_ready) {
      queue_test_packet();
    }
#endif
  }
}

void main(void) {

  clock_init();
  uart_init();
  gpio_init();
  pps_init();
  ioc_init();
  timer1_init_1s();
  interrupt_init();
  toggle_meter_init();

  while (1) {
    // LED behavior: pulse has priority; otherwise blink at 10Hz (10 cycles/sec => toggle every 50ms)
    // if (led_pulse_ticks) {
    //   STATUS_LED_LAT = 1;
    //   led_pulse_ticks--;
    //   if (led_pulse_ticks == 0) {
    //     STATUS_LED_LAT = 0;
    //   }
    // } else {
    //   blink_ms_counter++;
    //   if (blink_ms_counter >= 50) {
    //     blink_ms_counter = 0;
    //     STATUS_LED_LAT = !STATUS_LED_LAT;
    //   }
    // }

    if (print_buf_requested) {
      print_buf_requested = 0;
      print_buffer();
    }

    if (tx_retry_cooldown_ms) {
      tx_retry_cooldown_ms--;
    }

    if (minute_ready && (tx_retry_cooldown_ms == 0)) {
      if (!service_particle_send_packed()) {
        tx_retry_cooldown_ms = RETRY_COOLDOWN_MS;
      }
    }
    __delay_ms(1);
  }
}
