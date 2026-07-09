/*
 * Packet.h - framed UART protocol between PIC and Photon2.
 *
 * Frame:   AA 55 | func(1) | len(2,BE) | data[len] | crc16(2,BE)
 *          \__marker__/                  \__ payload __/  \_ CRC-16/MODBUS _/
 *
 *   - markers AA 55 let the RX state machine resync on a byte stream.
 *   - func identifies the message; REQ_* expect a response, others do not.
 *   - len is the payload byte count (big-endian, up to ~3 KB for flow data).
 *   - crc16 is CRC-16/MODBUS over func + len + data (markers excluded),
 *     transmitted big-endian (high byte first) for in-protocol consistency.
 *
 * The wake/handshake (0xF0 wake byte, WAKE line) is separate and unchanged;
 * packets ride on top once WAKE is high.
 */
#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>
#include <stdbool.h>

/* ---- frame constants ---- */
#define PKT_MARKER0         0xAAu
#define PKT_MARKER1         0x55u
#define PKT_OVERHEAD_BYTES  7u      /* marker(2)+func(1)+len(2)+crc(2)      */

/* Largest INBOUND payload = longest REQ packet's data. Currently SET_PARAM
 * (4 x uint16 = 8 bytes). Any received len greater than this is treated as a
 * framing error and the parser immediately resyncs (hunts AA 55), so a bad
 * length byte cannot make us wait for thousands of bytes. Bump this if a
 * longer inbound packet is ever added.
 * NOTE: this caps only what the PIC RECEIVES. The len field is still 2 bytes
 * because PIC->Photon flow data is large (COUNT + up to 1000 samples). */
#define PKT_MAX_RX_DATA     8u

/* If the gap between two bytes of a packet exceeds this, assume the stream
 * desynced and reset the parser to hunt for AA 55 again. */
#define PKT_RX_BYTE_TIMEOUT_MS  1000UL

/* ---- function codes ----
 * REQ_*  : Photon -> PIC, a response IS required.
 * (plain): Photon -> PIC, NO response (e.g. SYS_RESET).
 * RSP_*  : PIC -> Photon responses.                                       */
typedef enum {
    /* Photon -> PIC */
    PKT_REQ_DATA          = 0x01u,  /* send flow data            -> RSP_DATA   */
    PKT_REQ_GET_PARAM     = 0x02u,  /* read 4 leak params        -> RSP_PARAM  */
    PKT_REQ_SET_PARAM     = 0x03u,  /* write 4 leak params       -> RSP_ACK/NAK*/
    PKT_REQ_GET_VALVE     = 0x04u,  /* read valve status         -> RSP_VALVE  */
    PKT_REQ_VALVE_UNLOCK  = 0x05u,  /* clear lock(s) data:flags  -> RSP_ACK    */
    PKT_SYS_RESET         = 0x06u,  /* software reset (no response)            */
    PKT_PHOTON_OFF_REQ    = 0x07u,  /* Photon: "I'm done, cut my power" (no response)
                                     * data[0] = reason (see photon_off_reason_t).
                                     * PIC powers Photon OFF and returns to sleep. */
    PKT_REQ_POWER_STATE   = 0x08u,  /* Photon: "are you in the initial power-hold?"
                                     * (no payload) -> RSP_POWER_STATE. MUST be the
                                     * Photon's first packet each boot; retry @1 s
                                     * until it succeeds before any other request. */

    /* PIC -> Photon */
    PKT_RSP_DATA          = 0x81u,  /* data: COUNT(4)+samples(3*N)             */
    PKT_RSP_PARAM         = 0x82u,  /* data: 4 x u16 (8 bytes)                 */
    PKT_RSP_VALVE         = 0x84u,  /* data: valve status (8 bytes)            */
    PKT_RSP_POWER_STATE   = 0x88u,  /* data[0] = power_state_t (0=initial,1=normal) */
    PKT_RSP_ACK           = 0x8Eu,  /* data: echoed func(1)                    */
    PKT_RSP_NAK           = 0x8Fu   /* data: reason(1)                         */
} pkt_func_t;

/* NAK reason codes (data[0] of RSP_NAK). */
typedef enum {
    NAK_BAD_CRC   = 0x01u,
    NAK_BAD_LEN   = 0x02u,
    NAK_BAD_FUNC  = 0x03u,
    NAK_BUSY      = 0x04u
} nak_reason_t;

/* Reason byte carried in a PKT_PHOTON_OFF_REQ (func 0x07) data[0]. The PIC
 * cuts Photon power and sleeps regardless of the reason; it is informational
 * only (e.g. for logging / diagnostics). */
typedef enum {
    OFF_REASON_DONE       = 0x00u,  /* Photon finished all comms normally      */
    OFF_REASON_CLOUD_FAIL = 0x01u   /* Photon could not reach the cloud        */
} photon_off_reason_t;

/* Value in RSP_POWER_STATE data[0]. Tells the Photon whether the PIC is still
 * in its initial power-hold window (kept fully powered, no sleep, no power-cut)
 * right after a cold power-up, or has returned to normal power-gated operation.
 * The Photon uses this to decide whether to stay up (INITIAL) or run a normal
 * session and let the PIC cut power (NORMAL). */
typedef enum {
    POWER_STATE_INITIAL = 0x00u,    /* cold-boot hold window active (stay powered) */
    POWER_STATE_NORMAL  = 0x01u     /* hold window over -> normal gated operation  */
} power_state_t;

/* ---- CRC-16/MODBUS (poly 0xA001 reflected, init 0xFFFF) ---- */
uint16_t Packet_CRC16(const uint8_t *data, uint16_t len);
/* incremental form for streaming large TX payloads */
uint16_t Packet_CRC16_Init(void);
uint16_t Packet_CRC16_Update(uint16_t crc, uint8_t b);

/* ---- RX parser state machine ---- */
typedef enum {
    PKT_ST_MARKER0 = 0u,
    PKT_ST_MARKER1,
    PKT_ST_FUNC,
    PKT_ST_LEN_HI,
    PKT_ST_LEN_LO,
    PKT_ST_DATA,
    PKT_ST_CRC_HI,
    PKT_ST_CRC_LO
} pkt_state_t;

typedef struct {
    pkt_state_t state;
    uint8_t     func;
    uint16_t    len;                    /* declared payload length          */
    uint16_t    idx;                    /* data bytes collected so far      */
    uint8_t     data[PKT_MAX_RX_DATA];  /* payload (inbound only, small)    */
    uint16_t    crc_calc;               /* running CRC over func..data      */
    uint16_t    crc_rx;                 /* received CRC                     */
    uint32_t    last_byte_ms;           /* time of last accepted byte       */
    bool        bad_len;                /* last reset was due to len > max  */
    bool        bad_crc;                /* last completed frame failed CRC  */
} pkt_parser_t;

/* Reset the parser to hunt for the next AA 55. */
void Packet_ParserReset(pkt_parser_t *p);

/* Call periodically: if mid-packet and the inter-byte gap exceeded
 * PKT_RX_BYTE_TIMEOUT_MS, reset the parser. Safe to call every loop pass. */
void Packet_ParserTimeoutCheck(pkt_parser_t *p);

/* Feed one received byte. Returns true exactly once, when a COMPLETE and
 * CRC-VALID packet has been assembled (fields in *p are then valid until the
 * next call). On a bad length (> PKT_MAX_RX_DATA) it resyncs immediately
 * (p->bad_len set). On CRC failure it resyncs (p->bad_crc set). */
bool Packet_ParseByte(pkt_parser_t *p, uint8_t b);

#endif /* PACKET_H */
