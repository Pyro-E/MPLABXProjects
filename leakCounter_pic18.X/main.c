/*
 * File:   main.c
 * Author: kevinlu
 *
 * Created on June 10, 2026, 11:27 AM
 */
#include <stdbool.h>
#include <stdint.h>
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
#define ACK_WAIT_TIMEOUT_MS 1000u
#define RETRY_COOLDOWN_MS 5u

volatile uint16_t pulse_count = 0;
volatile uint16_t minute_count_latched = 0;
volatile uint8_t minute_ready = 0;
volatile uint8_t sec_ticks = 0;
volatile uint8_t led_pulse_ticks = 0;
volatile uint16_t tx_retry_cooldown_ms = 0;
volatile uint8_t blink_ms_counter = 0;

static void queue_test_packet(void) {
  minute_count_latched = TEST_PACKET_COUNT;
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

static uint8_t service_particle_readout(void) {
  uint16_t c;
  uint8_t flags = TEST_PACKET_FLAGS; // simple frame marker/debug byte
  uint8_t gie_was_enabled = INTCON0bits.GIE;

  // Keep bit-banged transfer timing deterministic.
  INTCON0bits.GIE = 0;

  c = minute_count_latched;

  PIC_WAKE_LAT = 1;
  __delay_ms(2);

  if (!wait_for_photon_start(START_WAIT_TIMEOUT_MS)) {
    PIC_WAKE_LAT = 0;
    INTCON0bits.GIE = gie_was_enabled;
    return 0;
  }

  send_byte((uint8_t)(c >> 8));
  send_byte((uint8_t)(c & 0xFF));
  send_byte(flags);

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
      pulse_count++;
      minute_count_latched = pulse_count;
      minute_ready = 1;
      STATUS_LED_LAT = !STATUS_LED_LAT;
      IOCCFbits.IOCCF0 = 0;
    }
    PIR0bits.IOCIF = 0;
  }

  if (PIR3bits.TMR1IF) {
    PIR3bits.TMR1IF = 0;
    timer1_reload_1s();

    sec_ticks++;
    if (sec_ticks >= TEST_PACKET_INTERVAL_SECONDS) {
      sec_ticks = 0;
#if TEST_MODE_FIXED_PACKET
      if (!minute_ready) {
        queue_test_packet();
      }
#endif
    }
  }
}

void main(void) {

  clock_init();
  gpio_init();
  pps_init();
  ioc_init();
  timer1_init_1s();
  interrupt_init();

  // Temporary: force DATA high for 2 seconds after boot to verify line works
  //   PIC_DATA_LAT = 1;
  //   for (int i = 0; i < 2000; i++) {
  //     __delay_ms(1);
  //   }
  //   PIC_DATA_LAT = 0;

  while (1) {
    // LED behavior: pulse has priority; otherwise blink at 10Hz (10 cycles/sec => toggle every 50ms)
    if (led_pulse_ticks) {
      STATUS_LED_LAT = 1;
      led_pulse_ticks--;
      if (led_pulse_ticks == 0) {
        STATUS_LED_LAT = 0;
      }
    } else {
      blink_ms_counter++;
      if (blink_ms_counter >= 50) {
        blink_ms_counter = 0;
        STATUS_LED_LAT = !STATUS_LED_LAT;
      }
    }

    if (tx_retry_cooldown_ms) {
      tx_retry_cooldown_ms--;
    }

    if (minute_ready && (tx_retry_cooldown_ms == 0)) {
      if (!service_particle_readout()) {
        tx_retry_cooldown_ms = RETRY_COOLDOWN_MS;
      }
    }
    __delay_ms(1);
  }
}
