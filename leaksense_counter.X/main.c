/* // File header start
 * File:   main.c // Source file name
 * Author: kevinlu // Original author
 * // Spacer inside header
 * Created on March 30, 2026, 7:23 PM // Creation timestamp
 */ // File header end

#include <stdbool.h>
#include <stdint.h>
#include <xc.h>

#define _XTAL_FREQ 1000000UL

//// CONFIG
#pragma config FEXTOSC =                                                       \
    OFF // ECH    // FEXTOSC External Oscillator mode Selection bits (EC
        // (external clock) above 8 MHz)
#pragma config RSTOSC = HFINT1 // EXT1X   // Power-up default value for COSC
                               // bits (EXTOSC operating per FEXTOSC bits)
#pragma config CLKOUTEN = OFF  // Clock Out Enable bit (CLKOUT function is
                               // disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN =                                                         \
    ON // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN =                                                         \
    ON // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

#pragma config MCLRE = ON // Master Clear Enable bit (MCLR/VPP pin function is
                          // MCLR; Weak pull-up enabled )
#pragma config WRT =                                                           \
    OFF // User NVM self-write protection bits (Write protection off)
#pragma config LVP =                                                           \
    ON // Low Voltage Programming Enable bit (Low voltage programming enabled.
       // MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)
#pragma config CP = OFF // User NVM Program Memory Code Protection bit (User NVM
                        // code protection disabled)
#pragma config CPD = OFF // Data NVM Memory Code Protection bit (Data NVM code
                         // protection disabled)

// //
// // ===================== PIN MAP ==================== // Pin mapping banner
// // RA0 = PIC_DATA  -> Photon D3 // Serial data output to Photon
// // RA1 = PIC_CLK   <- Photon D2 // Clock input from Photon
// // RA2 = FLOW_PULSE_CLEAN input // Flow pulse input pin
// // RA4 = PIC_WAKE  -> Photon D10/WKP // Wake signal output to Photon
// //

#define PIC_DATA_TRIS TRISAbits.TRISA0
#define PIC_DATA_LAT LATAbits.LATA0
#define PIC_DATA_PORT PORTAbits.RA0

#define PIC_CLK_TRIS TRISAbits.TRISA1
#define PIC_CLK_PORT PORTAbits.RA1

#define FLOW_TRIS TRISAbits.TRISA2
#define FLOW_PORT PORTAbits.RA2
#define FLOW_WPU WPUAbits.WPUA2

#define PIC_WAKE_TRIS TRISAbits.TRISA4
#define PIC_WAKE_LAT LATAbits.LATA4

#define STATUS_LED_TRIS TRISAbits.TRISA5
#define STATUS_LED_LAT LATAbits.LATA5

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

  // Directions
  TRISAbits.TRISA0 = 0; // DATA out
  TRISAbits.TRISA1 = 1; // CLK in
  TRISAbits.TRISA2 = 1; // FLOW in
  TRISAbits.TRISA4 = 0; // WAKE out
  TRISAbits.TRISA5 = 0; // LED out

  WPUA = 0x00;

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
  // Count rising edges on RA2
  IOCAPbits.IOCAP2 = 1;
  IOCANbits.IOCAN2 = 0;
  IOCAFbits.IOCAF2 = 0;
}

static void timer1_init_1s(void) {
  // Fosc = 1 MHz => instruction clock = 250 kHz
  // Timer1 clock = Fosc/4 = 250 kHz
  // Prescaler 1:8 => 31.25 kHz
  // 1 second = 31250 counts
  // preload = 65536 - 31250 = 34286 = 0x85EE

  T1CONbits.TMR1CS = 0b00; // clock source = FOSC/4
  T1CONbits.T1CKPS = 0b11; // prescaler = 1:8
  T1CONbits.T1SYNC = 0;    // not relevant for internal clock, keep 0
  T1CONbits.T1SOSC = 0;    // use T1CKI path, not SOSC
  TMR1H = 0x85;
  TMR1L = 0xEE;
  PIR1bits.TMR1IF = 0;
  PIE1bits.TMR1IE = 1;
  T1CONbits.TMR1ON = 1;
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

  INTCONbits.PEIE = 1;
  INTCONbits.GIE = 1;
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
  uint8_t gie_was_enabled = INTCONbits.GIE;

  // Keep bit-banged transfer timing deterministic.
  INTCONbits.GIE = 0;

  c = minute_count_latched;

  PIC_WAKE_LAT = 1;
  __delay_ms(2);

  if (!wait_for_photon_start(START_WAIT_TIMEOUT_MS)) {
    PIC_WAKE_LAT = 0;
    INTCONbits.GIE = gie_was_enabled;
    return 0;
  }

  send_byte((uint8_t)(c >> 8));
  send_byte((uint8_t)(c & 0xFF));
  send_byte(flags);

  PIC_DATA_LAT = 0;
  if (!wait_for_ack_8clocks(ACK_WAIT_TIMEOUT_MS)) {
    PIC_WAKE_LAT = 0;
    INTCONbits.GIE = gie_was_enabled;
    return 0;
  }

  PIC_WAKE_LAT = 0;
  minute_ready = 0;
  INTCONbits.GIE = gie_was_enabled;
  return 1;
}

void __interrupt() isr(void) {
  if (PIR0bits.IOCIF) {
    if (IOCAFbits.IOCAF2) {
      pulse_count++;
      minute_count_latched = pulse_count;
      minute_ready = 1;
      IOCAFbits.IOCAF2 = 0;
    }
    PIR0bits.IOCIF = 0;
  }

  if (PIR1bits.TMR1IF) {
    PIR1bits.TMR1IF = 0;
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
    STATUS_LED_LAT = 1;

    if (led_pulse_ticks) {
      led_pulse_ticks--;
      if (led_pulse_ticks == 0) {
        STATUS_LED_LAT = 0;
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
