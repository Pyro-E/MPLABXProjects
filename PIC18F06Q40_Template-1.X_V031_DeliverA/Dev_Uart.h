#ifndef DEV_UART_H
#define DEV_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "App_Config.h"

/* ============================================================
 *  Dev_Uart.h  -  UART1 driver for PIC18-Q40
 *                 TX: interrupt-driven, 64-byte ring buffer
 *                 RX: interrupt + callback
 *
 *  Hardware:
 *    TX = RC0  -> PICkit 5 pin 7 (TX target)
 *    RX = RC1  -> PICkit 5 pin 8 (RX target)
 *    GND       -> PICkit 5 pin 3
 *
 *  TX is non-blocking: print_* enqueue into the ring buffer and
 *  return immediately; the TX ISR drains the buffer in the
 *  background. If the buffer is full, the byte is dropped
 *  (see print_char return value).
 * ============================================================ */

#define UART_TX_BUF_SIZE  APP_UART_TX_BUF_SIZE   /* from App_Config.h */

/* receive callback: called from RX ISR with each received byte */
typedef void (*uart_rx_cb_t)(uint8_t ch);

void UART_Init(void);
void UART_RX_SetCallback(uart_rx_cb_t cb);

/* ---- Auto-wake-up (WUE): let an RX edge (Photon2's 0xF0) wake the MCU
 * from Sleep. Arm right before SLEEP(), disarm right after. WokeByEdge()
 * reports whether the wake came from the UART (read before disarming). ---- */
void UART_WakeArm(void);
void UART_WakeDisarm(void);
bool UART_WokeByEdge(void);

/* must be called from the global ISR (handles both TX and RX) */
void UART_ISR(void);

/* TX (non-blocking, background send) */
bool print_char(char c);        /* returns false if buffer was full (dropped) */
void print_string(const char *s);
void print_uint(uint32_t v);
void print_int(int32_t v);

/* optional helpers */
bool     UART_TX_IsEmpty(void); /* true if nothing pending to send */
uint8_t  UART_TX_Free(void);    /* free space in the TX buffer */

#endif /* DEV_UART_H */
