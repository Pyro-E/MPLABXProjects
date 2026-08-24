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
#define UART_RX_BUF_SIZE  APP_UART_RX_BUF_SIZE   /* from App_Config.h */

/* receive callback: called from RX ISR with each received byte */
typedef void (*uart_rx_cb_t)(uint8_t ch);

void UART_Init(void);

/* ---- V029: park / restore the TX pin while the Photon has no power --------
 *
 * The PIC's TX (RC0) idles HIGH whenever the UART is enabled. On this board the
 * line reaches the Photon's RX through a 1 k series resistor and a 3.3 k pull-
 * down to ground - a divider added because the PIC runs at 3.6-4.5 V while the
 * Photon's RX is rated to 3.6 V max, and the bare 1 k was not a level shifter
 * at all (it was relying on the Photon's ESD diode to clamp the excess).
 *
 * The divider does its job, but it also conducts continuously:
 *
 *     4.5 V / (1 k + 3.3 k) = 1.05 mA   measured on the PPK2 as the sleep floor
 *
 * That is roughly a thousand times the part's own sleep current, so nothing
 * about the sleep configuration can be measured while it is present.
 *
 * The Photon is off ~87 % of the time, and while it is off there is nothing on
 * the far end to receive anything. Releasing TX to high-impedance for exactly
 * that period lets the 3.3 k pull the node to 0 V and the divider stops
 * drawing:
 *
 *     driven all the time   1.05 mA continuous
 *     released when off     ~0.13 mA average (23 s session / 180 s period)
 *
 * It also removes the other half of the problem: with the Photon unpowered, a
 * driven TX was pushing ~3.9 mA into an input pin whose own rail was down,
 * which is what "phantom power" through an I/O pin means - the Photon's supply
 * sat at some non-zero voltage and it never saw a clean cold boot.
 *
 * Call Release when the Photon's supply is cut and Resume BEFORE it is applied,
 * so the line is already idling HIGH by the time the Photon samples it. The
 * caller must ensure the TX buffer has drained first; main's power state
 * machine already waits for UART_TX_IsEmpty().
 */
void UART_TX_Release(void);   /* RC0 -> input (Hi-Z); the 3.3 k holds it at 0 V */
void UART_TX_Resume(void);    /* RC0 -> output; UART drives it back to idle HIGH */
void UART_RX_SetCallback(uart_rx_cb_t cb);

/* RX ring buffer (for the packet parser) */
void UART_RX_Push(uint8_t b);          /* ISR context: enqueue          */
bool UART_RX_Pop(uint8_t *b);          /* main context: false if empty  */

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
