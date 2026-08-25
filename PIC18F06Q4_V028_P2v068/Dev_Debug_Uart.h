#ifndef DEV_DEBUG_UART_H
#define DEV_DEBUG_UART_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "App_Config.h"

/* ============================================================
 *  Dev_Debug_Uart.h  -  TX-ONLY debug UART on UART2 / RA4
 *
 *  WHY A SECOND UART
 *  -----------------
 *  UART1 (RC0/RC1) carries the BINARY packet protocol to the Photon. Mixing
 *  human-readable text into that stream corrupts frames, so debug output gets
 *  its own port and its own pin:
 *
 *      RC0 = U1TX  -> Photon packets ONLY
 *      RC1 = U1RX  -> Photon packets ONLY
 *      RA4 = U2TX  -> debug text ONLY  (this module)
 *      RA5         -> NEVER TOUCHED - see the warning below
 *
 *  Because the two streams use different pins, logging is safe even while a
 *  Photon session is running.
 *
 *  !! RA5 MUST NOT BE DRIVEN !!
 *  On this board RA5 is connected to RA4. Configuring RA5 as an output and
 *  latching it low also pulls the transmit line low and silences this port -
 *  which is exactly what happened, and took a long hunt to find. LEDs_Init()
 *  therefore leaves RA5 completely unconfigured, and nothing here touches it.
 *
 *  HEADER-ONLY ON PURPOSE
 *  ----------------------
 *  The implementation lives here as static inline functions rather than in a
 *  .c file. A .c file only runs if the MPLAB project lists it, and a project
 *  that omits it still builds cleanly while the port stays silent - no error,
 *  no warning. Putting the code in the header removes that failure mode: any
 *  translation unit that includes this file gets a working port.
 *
 *  ONE SHARED FIFO
 *  ---------------
 *  The buffer is defined once (in the file that sets DBG_OWNER before including
 *  this header - main.c) and merely declared everywhere else. An earlier
 *  version gave every including file its own static copy, which looked tidy but
 *  was broken: DBG_PROCESS() runs in main.c and drained only main.c's copy, so
 *  a line logged from FlowLog.c or Flow_Control.c sat in a buffer nobody ever
 *  emptied. Sharing one buffer also costs a quarter of the RAM.
 *
 *  NON-BLOCKING FIFO, DRAINED FROM THE MAIN LOOP (no interrupt)
 *  ------------------------------------------------------------
 *  DbgUart_Char() only enqueues and returns at once, so a log call never
 *  stalls the caller and cannot change the timing of the code under test.
 *  DbgUart_Process(), called once per main-loop pass, hands ONE byte to the
 *  transmitter whenever it is free and returns immediately. Hence:
 *    - no ISR, so the 1 ms tick and packet timing are untouched
 *    - the main loop is never held up waiting for the wire
 *  The loop iterates far faster than a byte takes to shift out (about 260 us
 *  at 38400 baud), so the transmitter stays continuously busy.
 *  If the FIFO fills, bytes are DROPPED rather than blocking - logging must
 *  never change the behaviour under test.
 *
 *  SLEEP: the UART clock stops in Sleep, which would truncate a byte in
 *  flight. Use DBG_IS_IDLE() in the sleep condition. Never log from an ISR.
 *
 *  Wiring: RA4 -> RX of a 3.3 V USB-serial adapter, GND common. 38400 8N1.
 * ============================================================ */

#ifdef APP_DEBUG_UART2_ENABLE

/* These values are fixed here rather than read from App_Config.h, so a stale
 * configuration cannot silently change them. */
#ifndef DBG_XTAL_HZ
  #define DBG_XTAL_HZ    64000000UL
#endif
#ifndef DBG_BAUD_HZ
  #define DBG_BAUD_HZ    38400UL
#endif
/* PPS output code for U2TX, MEASURED on this board: a sweep of every output
 * code 0x00-0x3F produced readable output at exactly 0x13. (U1TX is 0x10.) */
#ifndef DBG_U2TX_PPS
  #define DBG_U2TX_PPS   0x13u
#endif
#ifndef DBG_FIFO_SIZE
  /* Per including file. 64 proved too small: during the 3 s Photon power cut
   * several boot lines queue up at once and the tail of one was dropped
   * ("[BOOT] cold: c" instead of the full line). 128 holds a few full lines
   * while earlier bytes are still shifting out.
   *
   * V021: App_Config.h has always carried an APP_DEBUG_UART2_FIFO knob, but it
   * was never wired to anything - changing it did nothing and the size stayed
   * hardcoded at 128 here. It is honoured now. A whole Photon session emits
   * several lines inside ONE main-loop pass while DbgUart_Process() drains only
   * one byte per pass, so 128 overflows and whole lines vanish (a [CAP] line was
   * lost outright in the 20260813 bench capture). */
  #ifdef APP_DEBUG_UART2_FIFO
    #define DBG_FIFO_SIZE  APP_DEBUG_UART2_FIFO
  #else
    #define DBG_FIFO_SIZE  128u
  #endif
#endif

/* BRGS = 1 (4x mode): BRG = Fosc / (4 * baud) - 1, rounded to nearest.
 * 64 MHz / 38400 -> 416, giving 38369 baud (-0.08 %). */
#define DBG_BRG  ((uint16_t)(((DBG_XTAL_HZ + (2UL*DBG_BAUD_HZ)) / (4UL*DBG_BAUD_HZ)) - 1UL))

/* Storage lives in ONE translation unit: the one that does
 *     #define DBG_OWNER
 * before including this header (main.c). Everyone else sees declarations. */
#ifdef DBG_OWNER
  uint8_t  g_dbg_fifo[DBG_FIFO_SIZE];
  uint16_t g_dbg_head  = 0;              /* write index (producer) */
  uint16_t g_dbg_tail  = 0;              /* read index  (consumer) */
  uint16_t g_dbg_count = 0;              /* bytes currently queued */
  bool     g_dbg_drop  = false;          /* set if the FIFO overflowed */
#else
  extern uint8_t  g_dbg_fifo[DBG_FIFO_SIZE];
  extern uint16_t g_dbg_head;
  extern uint16_t g_dbg_tail;
  extern uint16_t g_dbg_count;
  extern bool     g_dbg_drop;
#endif

static inline void DbgUart_Init(void)
{
    /* Configured exactly ONCE. Re-writing U2BRGL/H or U2CON while the module
     * is already ON restarts the baud generator mid-flight, so nothing else
     * ever touches these registers again - only U2TXB. */
    ANSELAbits.ANSELA4 = 0;              /* digital, not analog */
    LATAbits.LATA4     = 1;              /* idle-high, the UART idle level */
    TRISAbits.TRISA4   = 0;              /* output */
    RA4PPS = DBG_U2TX_PPS;

    /* No RX: U2RXPPS is never written and RXEN stays 0, so this port uses
     * exactly one pin. RA5 is not touched (see the warning above). */
    U2CON0bits.BRGS = 1;
    U2BRGL = (uint8_t)(DBG_BRG & 0xFFu);
    U2BRGH = (uint8_t)(DBG_BRG >> 8);
    U2CON0bits.MODE = 0b0000;            /* 8-bit async, no parity */
    U2CON0bits.TXEN = 1;
    U2CON0bits.RXEN = 0;
    U2CON1bits.ON   = 1;
}

static inline void DbgUart_Char(char c)
{
    if (g_dbg_count >= DBG_FIFO_SIZE) {
        g_dbg_drop = true;               /* full: drop rather than block */
        return;
    }
    g_dbg_fifo[g_dbg_head] = (uint8_t)c;
    g_dbg_head = (uint16_t)((g_dbg_head + 1u) % DBG_FIFO_SIZE);
    g_dbg_count++;
}

static inline void DbgUart_Process(void)
{
    /* One byte per call, only when the transmit shift register is free. */
    if ((g_dbg_count != 0u) && (U2ERRIRbits.TXMTIF == 1)) {
        U2TXB = g_dbg_fifo[g_dbg_tail];
        g_dbg_tail = (uint16_t)((g_dbg_tail + 1u) % DBG_FIFO_SIZE);
        g_dbg_count--;
    }
}

/* Idle = this file's queue is empty AND the shifter has finished. */
static inline bool DbgUart_IsIdle(void)
{
    return (g_dbg_count == 0u) && (U2ERRIRbits.TXMTIF == 1);
}

static inline bool DbgUart_Dropped(void) { return g_dbg_drop; }

static inline void DbgUart_Str(const char *s)
{
    if (s == 0) return;
    while (*s != '\0') { DbgUart_Char(*s++); }
}

static inline void DbgUart_Uint(uint32_t v)
{
    char buf[11];                        /* 32-bit max is 10 digits */
    uint8_t i = 0;
    if (v == 0u) { DbgUart_Char('0'); return; }
    while ((v != 0u) && (i < sizeof(buf))) {
        buf[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    }
    while (i != 0u) { DbgUart_Char(buf[--i]); }   /* digits came out reversed */
}

static inline void DbgUart_Int(int32_t v)
{
    if (v < 0) { DbgUart_Char('-'); DbgUart_Uint((uint32_t)(-(int64_t)v)); }
    else       { DbgUart_Uint((uint32_t)v); }
}

static inline void DbgUart_Hex8(uint8_t v)
{
    static const char hx[] = "0123456789ABCDEF";
    DbgUart_Char(hx[(v >> 4) & 0x0Fu]);
    DbgUart_Char(hx[v & 0x0Fu]);
}

static inline void DbgUart_NL(void) { DbgUart_Char('\r'); DbgUart_Char('\n'); }

#define DBG_INIT()      DbgUart_Init()
#define DBG_PROCESS()   DbgUart_Process()
#define DBG_IS_IDLE()   DbgUart_IsIdle()
#define DBG_DROPPED()   DbgUart_Dropped()
#define DBG_CH(c)       DbgUart_Char(c)
#define DBG_STR(s)      DbgUart_Str(s)
#define DBG_U32(v)      DbgUart_Uint((uint32_t)(v))
#define DBG_I32(v)      DbgUart_Int((int32_t)(v))
#define DBG_HEX8(v)     DbgUart_Hex8((uint8_t)(v))
#define DBG_NL()        DbgUart_NL()

#else   /* disabled: every call vanishes, no code and no pin use */

/* do{}while(0) - not ((void)0) - so XC8 does not warn 759 at each call site. */
#define DBG_INIT()      do { } while (0)
#define DBG_PROCESS()   do { } while (0)
#define DBG_IS_IDLE()   (true)
#define DBG_DROPPED()   (false)
#define DBG_CH(c)       do { } while (0)
#define DBG_STR(s)      do { } while (0)
#define DBG_U32(v)      do { } while (0)
#define DBG_I32(v)      do { } while (0)
#define DBG_HEX8(v)     do { } while (0)
#define DBG_NL()        do { } while (0)

#endif  /* APP_DEBUG_UART2_ENABLE */

#endif  /* DEV_DEBUG_UART_H */
