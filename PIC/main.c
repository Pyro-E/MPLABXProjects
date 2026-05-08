#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define _XTAL_FREQ 1000000UL

/* ===== CONFIG ===== */
#pragma config FEXTOSC  = OFF
#pragma config RSTOSC   = HFINT1
#pragma config CLKOUTEN = OFF
#pragma config CSWEN    = ON
#pragma config FCMEN    = ON

#pragma config MCLRE = ON
#pragma config WDTE  = OFF
#pragma config LVP   = ON
#pragma config CP    = OFF

/* ===== Pin map ===== */
#define PIC_DATA_TRIS   TRISAbits.TRISA0
#define PIC_DATA_LAT    LATAbits.LATA0
#define PIC_DATA_PORT   PORTAbits.RA0

#define PIC_CLK_TRIS    TRISAbits.TRISA1
#define PIC_CLK_PORT    PORTAbits.RA1

#define FLOW_TRIS       TRISAbits.TRISA2
#define FLOW_PORT       PORTAbits.RA2
#define FLOW_WPU        WPUAbits.WPUA2

#define PIC_WAKE_TRIS   TRISAbits.TRISA4
#define PIC_WAKE_LAT    LATAbits.LATA4

#define VALVE_TRIS      TRISAbits.TRISA5
#define VALVE_LAT       LATAbits.LATA5

/* ===== Protocol constants ===== */
#define FRAME_START         0xAA
#define FRAME_END           0x55

/* PIC -> Photon message types */
#define MSG_MINUTE_REPORT   0x01    /* full 3-h ring buffer dump */
#define MSG_EVENT           0x02    /* immediate event/alert */
#define MSG_STATUS          0x03    /* response to STATUS_REQ */

/* Photon -> PIC commands */
#define CMD_ACK             0x10
#define CMD_STATUS_REQ      0x11
#define CMD_VALVE_OPEN      0x12
#define CMD_VALVE_CLOSE     0x13
#define CMD_AUTO_ARM        0x14
#define CMD_AUTO_DISARM     0x15
#define CMD_SET_INTERVAL    0x16    /* 1 byte payload: minutes */
#define CMD_CLEAR_FAULTS    0x17

/* Event / fault flag bits */
#define FLAG_LEAK           0x01    /* sustained low-rate flow */
#define FLAG_BIG_FLOW       0x02    /* abnormal high rate */
#define FLAG_VALVE_FAULT    0x04
#define FLAG_OVERFLOW       0x08    /* per-minute counter saturated */
#define FLAG_UART_TIMEOUT   0x10
#define FLAG_BAD_CMD        0x20
#define FLAG_BROWNOUT       0x40
#define FLAG_BUFFER_FULL    0x80

/* Ring buffer */
#define FLOW_BUF_MINUTES    30      /* 30 minutes - fits in PIC16LF18313 256-byte RAM */

/* Sensor calibration: pulses per US gallon (K-factor).
 * Adjust to match the installed flow sensor. */
#define PULSES_PER_GALLON       450u

/* Stored units are gpm * 10 (one decimal place, 0.0 .. 25.5 gpm). */
#define MINUTE_COUNT_MAX        255u    /* uint8_t saturation */

/* Thresholds expressed in the same gpm*10 units. */
#define LEAK_GPM_X10            1u      /* 0.1 gpm sustained = leak */
#define LEAK_SUSTAIN_MINUTES    10u
#define BIG_FLOW_GPM_X10        80u     /* >= 8.0 gpm = abnormal */

/* Timing */
#define STARTUP_QUIET_MS        100
#define BITBANG_TIMEOUT_TICKS   30      /* 1-s ticks; ~30 s patience */

/* Valve fault detection: after a CLOSE has settled, if we keep
 * seeing flow >= LEAK_GPM_X10 for this many consecutive minutes,
 * the valve is presumed stuck/leaking past the seat. */
#define VALVE_CLOSE_SETTLE_MIN  1u
#define VALVE_FAULT_FLOW_MIN    3u

static uint8_t  g_cmd     = 0;
static uint8_t  g_arg     = 0;
static bool     g_has_arg = false;


/* ===== Valve state machine ===== */
typedef enum {
    VALVE_IDLE = 0,
    VALVE_OPEN,
    VALVE_CLOSE_REQUESTED,
    VALVE_CLOSED,
    VALVE_FAULT
} valve_state_t;

/* ===== Ring buffer ===== */
typedef struct {
    uint8_t minute_val[FLOW_BUF_MINUTES];
    uint8_t head;
    uint8_t count;
} flow_ring_buf_t;

/* ===== Globals ===== */
static volatile uint16_t pulse_accum_1s     = 0;
static volatile uint16_t pulse_accum_minute = 0;
static volatile uint8_t  sec_ticks          = 0;
static volatile uint8_t  minute_ready       = 0;
static volatile uint8_t  latched_minute_count = 0;

static flow_ring_buf_t   flow_buf;
static volatile uint8_t  event_flags        = 0;
static uint8_t           leak_run_minutes   = 0;
static valve_state_t     valve_state        = VALVE_IDLE;
static uint8_t           auto_shutoff_armed = 1;
static uint8_t           report_interval_min = FLOW_BUF_MINUTES;
static uint8_t           close_settle_minutes = 0;
static uint8_t           post_close_flow_minutes = 0;

/* CRC-8, polynomial 0x07 (CCITT-style), init 0x00. Small and good
 * enough for the short frames used here. */
static uint8_t crc8_update(uint8_t crc, uint8_t v){
    crc ^= v;
    for (uint8_t i = 0; i < 8; i++){
        crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

static void clock_init(void){
    OSCCON1 = 0x60;
    OSCFRQ  = 0x03;     /* 1 MHz */
}

static void gpio_init(void){
    ANSELA = 0x00;

    /* Defensive: ensure the global weak-pull-up enable is on. On
     * PIC16(L)F18xxx parts WPU is controlled per-pin via WPUAx; on
     * older parts an OPTION_REG bit can disable them globally. */
#ifdef _OPTION_REG
    OPTION_REGbits.nWPUEN = 0;
#endif

    PIC_DATA_TRIS = 1;
    PIC_DATA_LAT  = 0;
    PIC_CLK_TRIS  = 1;

    /* Flow input - enable internal weak pull-up for open-drain front ends. */
    FLOW_TRIS = 1;
    FLOW_WPU  = 1;

    /* Wake output - idle low */
    PIC_WAKE_TRIS = 0;
    PIC_WAKE_LAT  = 0;

    /* Valve - idle in safe state (depends on driver polarity; assume
     * low = no actuation / valve open as default). */
    VALVE_TRIS = 0;
    VALVE_LAT  = 0;
}

static void ioc_init(void){
    /* Flow sensor: rising edge on RA2 wakes us to count a pulse. */
    IOCAPbits.IOCAP2 = 1;
    IOCANbits.IOCAN2 = 0;
    IOCAFbits.IOCAF2 = 0;

    /* Photon-initiated session: rising edge on shared CLK (RA1) wakes
     * us so the Photon can solicit attention without us having raised
     * PIC_WAKE first. */
    IOCAPbits.IOCAP1 = 1;
    IOCANbits.IOCAN1 = 0;
    IOCAFbits.IOCAF1 = 0;
}

static void timer1_reload_1s(void){
    /* LFINTOSC = 31 kHz, prescaler 1:1.
     * 1 s = 31000 counts; preload = 65536 - 31000 = 34536 = 0x86E8.
     * (LFINTOSC keeps running in SLEEP, unlike Fosc/4.) */
    TMR1H = 0x86;
    TMR1L = 0xE8;
}

static void timer1_init_1s(void){
    T1CONbits.TMR1CS    = 0b11;     /* LFINTOSC - runs through SLEEP */
    T1CONbits.T1CKPS    = 0b00;     /* 1:1 prescaler */
    T1CONbits.T1SYNC    = 1;        /* async: required for sleep operation */
    timer1_reload_1s();
    PIR1bits.TMR1IF     = 0;
    PIE1bits.TMR1IE     = 1;
    T1CONbits.TMR1ON    = 1;
}

static void interrupt_init(void){
    PIR0bits.IOCIF = 0;
    PIE0bits.IOCIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE  = 1;
}

static void ring_init(void){
    flow_buf.head  = 0;
    flow_buf.count = 0;
}

static void ring_push(uint8_t v){
    flow_buf.minute_val[flow_buf.head] = v;
    flow_buf.head = (uint8_t)((flow_buf.head + 1) % FLOW_BUF_MINUTES);
    if (flow_buf.count < FLOW_BUF_MINUTES){
        flow_buf.count++;
    }
}

static void ring_clear(void){
    flow_buf.head  = 0;
    flow_buf.count = 0;
}

static void valve_apply(void){
    switch (valve_state){
    case VALVE_OPEN:
    case VALVE_IDLE:
        VALVE_LAT = 0;
        break;
    case VALVE_CLOSE_REQUESTED:
    case VALVE_CLOSED:
        VALVE_LAT = 1;
        break;
    case VALVE_FAULT:
        /* Fail-safe: command close */
        VALVE_LAT = 1;
        break;
    }
}

static void valve_request_close(void){
    if (valve_state != VALVE_CLOSED && valve_state != VALVE_FAULT){
        valve_state = VALVE_CLOSE_REQUESTED;
    }
    valve_apply();
}

static void valve_request_open(void){
    if (valve_state != VALVE_FAULT){
        valve_state = VALVE_OPEN;
    }
    valve_apply();
}

static bool wait_clk_state(uint8_t state, uint16_t timeout_loops){
    while (PIC_CLK_PORT != state){
        if (--timeout_loops == 0) return false;
    }
    return true;
}

static bool send_bit(uint8_t b){
    PIC_DATA_LAT  = b ? 1 : 0;
    PIC_DATA_TRIS = 0;                              /* drive */
    if (!wait_clk_state(0, 60000)) return false;    /* CLK low */
    if (!wait_clk_state(1, 60000)) return false;    /* CLK rising */
    if (!wait_clk_state(0, 60000)) return false;    /* CLK falling */
    return true;
}

static bool send_byte(uint8_t v){
    for (uint8_t i = 0; i < 8; i++){
        if (!send_bit((v & 0x80) != 0)) return false;
        v <<= 1;
    }
    return true;
}

static bool recv_bit(uint8_t *out){
    PIC_DATA_TRIS = 1;                              /* release */
    if (!wait_clk_state(0, 60000)) return false;
    if (!wait_clk_state(1, 60000)) return false;    /* sample on rising */
    *out = PIC_DATA_PORT ? 1 : 0;
    if (!wait_clk_state(0, 60000)) return false;
    return true;
}

static bool recv_byte(uint8_t *out){
    uint8_t v = 0, b = 0;
    for (uint8_t i = 0; i < 8; i++){
        if (!recv_bit(&b)) return false;
        v = (uint8_t)((v << 1) | b);
    }
    *out = v;
    return true;
}

static bool tx_frame(uint8_t type, const uint8_t *payload, uint8_t len){
    uint8_t crc = 0;
    crc = crc8_update(crc, type);
    crc = crc8_update(crc, len);
    if (!send_byte(FRAME_START)) return false;
    if (!send_byte(type))        return false;
    if (!send_byte(len))         return false;
    for (uint8_t i = 0; i < len; i++){
        if (!send_byte(payload[i])) return false;
        crc = crc8_update(crc, payload[i]);
    }
    if (!send_byte(crc))        return false;
    if (!send_byte(FRAME_END))  return false;
    return true;
}


static bool rx_command(void){
    g_has_arg = false;
    g_arg     = 0;
    if (!recv_byte(&g_cmd)) return false;

    switch (g_cmd){
    case CMD_SET_INTERVAL: {
        uint8_t c;
        if (!recv_byte(&g_arg)) return false;
        if (!recv_byte(&c))     return false;
        if (c != (uint8_t)(g_cmd ^ g_arg)){
            event_flags |= FLAG_BAD_CMD;
            return false;
        }
        g_has_arg = true;
        return true;
    }
    case CMD_ACK:
    case CMD_STATUS_REQ:
    case CMD_VALVE_OPEN:
    case CMD_VALVE_CLOSE:
    case CMD_AUTO_ARM:
    case CMD_AUTO_DISARM:
    case CMD_CLEAR_FAULTS: {
        uint8_t c;
        if (!recv_byte(&c)) return false;
        if (c != g_cmd){
            event_flags |= FLAG_BAD_CMD;
            return false;
        }
        return true;
    }
    default:
        event_flags |= FLAG_BAD_CMD;
        return false;
    }
}

static void apply_command(void){
    switch (g_cmd){
    case CMD_ACK:
        ring_clear();
        event_flags &= (uint8_t)~FLAG_BUFFER_FULL;
        break;
    case CMD_STATUS_REQ: {
        uint8_t p[4];
        p[0] = (uint8_t)valve_state;
        p[1] = event_flags;
        p[2] = flow_buf.count;
        p[3] = latched_minute_count;
        (void)tx_frame(MSG_STATUS, p, sizeof p);
        break;
    }
    case CMD_VALVE_OPEN:
        valve_request_open();
        break;
    case CMD_VALVE_CLOSE:
        valve_request_close();
        break;
    case CMD_AUTO_ARM:
        auto_shutoff_armed = 1;
        break;
    case CMD_AUTO_DISARM:
        auto_shutoff_armed = 0;
        break;
    case CMD_SET_INTERVAL:
        if (g_has_arg && g_arg > 0 && g_arg <= FLOW_BUF_MINUTES){
            report_interval_min = g_arg;
        } else {
            event_flags |= FLAG_BAD_CMD;
        }
        break;
    case CMD_CLEAR_FAULTS:
        event_flags = 0;
        if (valve_state == VALVE_FAULT) valve_state = VALVE_IDLE;
        valve_apply();
        break;
    default:
        event_flags |= FLAG_BAD_CMD;
        break;
    }
}

static void valve_settle(void){
    /* Track time in CLOSE_REQUESTED so a subsequent flow watchdog can
     * decide whether the valve actually closed. */
    if (valve_state == VALVE_CLOSE_REQUESTED){
        if (close_settle_minutes < 0xFF) close_settle_minutes++;
        if (close_settle_minutes >= VALVE_CLOSE_SETTLE_MIN){
            valve_state = VALVE_CLOSED;
            post_close_flow_minutes = 0;
        }
    } else {
        close_settle_minutes = 0;
    }
}

static void wake_photon_and_serve(uint8_t msg_type){
    uint8_t timeout = BITBANG_TIMEOUT_TICKS;
    uint8_t prev_ticks = sec_ticks;

    PIC_WAKE_LAT = 1;

    while (PIC_CLK_PORT == 0){
        SLEEP();
        NOP();
        /* Decrement the timeout once per real 1 s tick, not once per
         * wake event (IOC pulses also wake us). */
        if (sec_ticks != prev_ticks){
            prev_ticks = sec_ticks;
            if (--timeout == 0){
                event_flags |= FLAG_UART_TIMEOUT;
                PIC_WAKE_LAT = 0;
                return;
            }
        }
    }

    if (msg_type == MSG_MINUTE_REPORT){
        /* Header: count, head, flags, then count bytes of data */
        uint8_t hdr[3];
        hdr[0] = flow_buf.count;
        hdr[1] = flow_buf.head;
        hdr[2] = event_flags;

        uint8_t crc = 0;
        uint8_t len = (uint8_t)(3 + flow_buf.count);
        crc = crc8_update(crc, MSG_MINUTE_REPORT);
        crc = crc8_update(crc, len);

        if (!send_byte(FRAME_START))            goto fail;
        if (!send_byte(MSG_MINUTE_REPORT))      goto fail;
        if (!send_byte(len))                    goto fail;
        for (uint8_t i = 0; i < 3; i++){
            if (!send_byte(hdr[i])) goto fail;
            crc = crc8_update(crc, hdr[i]);
        }
        //Walk ring buffer in chronological order
        uint8_t idx = (uint8_t)((flow_buf.head + FLOW_BUF_MINUTES
                                 - flow_buf.count) % FLOW_BUF_MINUTES);
        for (uint8_t i = 0; i < flow_buf.count; i++){
            uint8_t b = flow_buf.minute_val[idx];
            if (!send_byte(b)) goto fail;
            crc = crc8_update(crc, b);
            idx = (uint8_t)((idx + 1) % FLOW_BUF_MINUTES);
        }
        if (!send_byte(crc))       goto fail;
        if (!send_byte(FRAME_END)) goto fail;
    } else {
        uint8_t p[3];
        p[0] = event_flags;
        p[1] = (uint8_t)valve_state;
        p[2] = latched_minute_count;
        if (!tx_frame(MSG_EVENT, p, sizeof p)) goto fail;
    }

    //Wait for Photon command/ACK
    {
        uint8_t cmd = 0, arg = 0;
        bool has_arg = false;
        if (rx_command()){
            apply_command();
        } else {
            event_flags |= FLAG_UART_TIMEOUT;
        }
    }

    //Release shared lines 
    PIC_DATA_TRIS = 1;
    PIC_WAKE_LAT  = 0;
    return;

fail:
    event_flags  |= FLAG_UART_TIMEOUT;
    PIC_DATA_TRIS = 1;
    PIC_WAKE_LAT  = 0;
}

void __interrupt() isr(void){
    if (PIR0bits.IOCIF){
        if (IOCAFbits.IOCAF2){
            pulse_accum_1s++;
            IOCAFbits.IOCAF2 = 0;
        }
        if (IOCAFbits.IOCAF1){
            /* Photon poked CLK - just wake; main loop / wait routines
             * will discover the activity. */
            IOCAFbits.IOCAF1 = 0;
        }
        PIR0bits.IOCIF = 0;
    }

    if (PIR1bits.TMR1IF){
        PIR1bits.TMR1IF = 0;
        timer1_reload_1s();

        /* fold 1 s bucket into minute accumulator */
        uint16_t s = pulse_accum_1s;
        pulse_accum_1s = 0;
        pulse_accum_minute += s;

        sec_ticks++;
        if (sec_ticks >= 60){
            sec_ticks = 0;
            uint16_t pulses = pulse_accum_minute;
            pulse_accum_minute = 0;

            /* Convert pulses/minute to gpm * 10.
             * Max pulses before uint16_t overflow: 65535*450/10 >> possible rate,
             * so uint16_t is safe. Saturation guard keeps result in uint8_t. */
            uint16_t scaled = (uint16_t)((pulses * 10u + (PULSES_PER_GALLON / 2u))
                               / PULSES_PER_GALLON);
            if (scaled > MINUTE_COUNT_MAX){
                scaled = MINUTE_COUNT_MAX;
                event_flags |= FLAG_OVERFLOW;
            }
            latched_minute_count = (uint8_t)scaled;
            minute_ready = 1;
        }
    }
}

static void process_minute(void){
    uint8_t m = latched_minute_count;
    minute_ready = 0;

    ring_push(m);

    /* Threshold checks (units: gpm * 10) */
    bool event_now = false;

    if (m >= BIG_FLOW_GPM_X10){
        event_flags |= FLAG_BIG_FLOW;
        event_now = true;
    }

    if (m >= LEAK_GPM_X10 && m < BIG_FLOW_GPM_X10){
        if (leak_run_minutes < 0xFF) leak_run_minutes++;
        if (leak_run_minutes >= LEAK_SUSTAIN_MINUTES){
            event_flags |= FLAG_LEAK;
            event_now = true;
        }
    } else if (m < LEAK_GPM_X10){
        /* Reset on any sub-leak reading (including the dead-band that
         * used to leave the counter latched). */
        leak_run_minutes = 0;
    }

    /* Local fail-safe: if armed and any safety flag is set, close valve. */
    if (auto_shutoff_armed &&
        (event_flags & (FLAG_LEAK | FLAG_BIG_FLOW))){
        valve_request_close();
    }

    /* Valve fault watchdog: if the valve is reported CLOSED but flow
     * persists, declare a fault. Force-close (already done) and flag. */
    if (valve_state == VALVE_CLOSED){
        if (m >= LEAK_GPM_X10){
            if (post_close_flow_minutes < 0xFF) post_close_flow_minutes++;
            if (post_close_flow_minutes >= VALVE_FAULT_FLOW_MIN){
                valve_state = VALVE_FAULT;
                event_flags |= FLAG_VALVE_FAULT;
                event_now = true;
                valve_apply();
            }
        } else {
            post_close_flow_minutes = 0;
        }
    }

    /* Buffer full / interval reached -> request Photon wake */
    if (flow_buf.count >= report_interval_min){
        event_flags |= FLAG_BUFFER_FULL;
    }

    if (event_flags & FLAG_BUFFER_FULL){
        wake_photon_and_serve(MSG_MINUTE_REPORT);
    } else if (event_now){
        wake_photon_and_serve(MSG_EVENT);
    }

    valve_settle();
}

void main(void){
    clock_init();
    gpio_init();
    ring_init();
    ioc_init();
    timer1_init_1s();
    interrupt_init();

    /* Quiet window: do not transmit on shared ICSP/runtime nets right
     * after reset, in case a programmer or sleeping Photon is present. */
    __delay_ms(STARTUP_QUIET_MS);

    while (1){
        if (minute_ready){
            process_minute();
        }
        SLEEP();
        NOP();
    }
}


