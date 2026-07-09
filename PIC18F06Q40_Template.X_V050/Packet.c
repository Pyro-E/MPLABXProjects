/*
 * Packet.c - CRC-16/MODBUS + RX packet parser state machine.
 * See Packet.h for the frame format.
 */
#include "Packet.h"
#include "MCU_Time.h"     /* getNowTime(), timeSpan() */

/* ============================================================= *
 *  CRC-16/MODBUS : poly 0xA001 (reflected 0x8005), init 0xFFFF  *
 *  Table-free; ~8 shifts/byte. Minimal ROM/RAM.                 *
 * ============================================================= */
uint16_t Packet_CRC16_Init(void)
{
    return 0xFFFFu;
}

uint16_t Packet_CRC16_Update(uint16_t crc, uint8_t b)
{
    crc ^= (uint16_t)b;
    for (uint8_t i = 0u; i < 8u; i++) {
        if (crc & 0x0001u) crc = (uint16_t)((crc >> 1) ^ 0xA001u);
        else               crc = (uint16_t)(crc >> 1);
    }
    return crc;
}

uint16_t Packet_CRC16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = Packet_CRC16_Init();
    while (len--) {
        crc = Packet_CRC16_Update(crc, *data++);
    }
    return crc;
}

/* ============================================================= *
 *  RX parser                                                    *
 * ============================================================= */
void Packet_ParserReset(pkt_parser_t *p)
{
    p->state   = PKT_ST_MARKER0;
    p->func    = 0u;
    p->len     = 0u;
    p->idx     = 0u;
    p->crc_calc= 0u;
    p->crc_rx  = 0u;
    /* bad_len / bad_crc are sticky flags for the caller; cleared on new sync */
}

void Packet_ParserTimeoutCheck(pkt_parser_t *p)
{
    /* only meaningful once we've started a frame (past idle marker hunt) */
    if (p->state != PKT_ST_MARKER0) {
        if (timeSpan(p->last_byte_ms) >= PKT_RX_BYTE_TIMEOUT_MS) {
            Packet_ParserReset(p);
        }
    }
}

bool Packet_ParseByte(pkt_parser_t *p, uint8_t b)
{
    p->last_byte_ms = getNowTime();

    switch (p->state) {

    case PKT_ST_MARKER0:
        if (b == PKT_MARKER0) {
            p->bad_len = false;     /* clear stale flags only when a new   */
            p->bad_crc = false;     /* frame actually begins (AA seen)     */
            p->state   = PKT_ST_MARKER1;
        }
        break;

    case PKT_ST_MARKER1:
        if (b == PKT_MARKER1) {
            p->state    = PKT_ST_FUNC;
        } else if (b == PKT_MARKER0) {
            p->state    = PKT_ST_MARKER1;   /* AA AA ... stay, last AA counts */
        } else {
            p->state    = PKT_ST_MARKER0;   /* not a header, resync          */
        }
        break;

    case PKT_ST_FUNC:
        p->func     = b;
        p->crc_calc = Packet_CRC16_Init();
        p->crc_calc = Packet_CRC16_Update(p->crc_calc, b);
        p->state    = PKT_ST_LEN_HI;
        break;

    case PKT_ST_LEN_HI:
        p->len      = (uint16_t)((uint16_t)b << 8);
        p->crc_calc = Packet_CRC16_Update(p->crc_calc, b);
        p->state    = PKT_ST_LEN_LO;
        break;

    case PKT_ST_LEN_LO:
        p->len     |= (uint16_t)b;
        p->crc_calc = Packet_CRC16_Update(p->crc_calc, b);
        p->idx      = 0u;
        /* reject any inbound length longer than we can accept -> resync now */
        if (p->len > PKT_MAX_RX_DATA) {
            p->bad_len = true;
            Packet_ParserReset(p);          /* immediate hunt for AA 55      */
            break;
        }
        p->state    = (p->len == 0u) ? PKT_ST_CRC_HI : PKT_ST_DATA;
        break;

    case PKT_ST_DATA:
        p->data[p->idx++] = b;
        p->crc_calc = Packet_CRC16_Update(p->crc_calc, b);
        if (p->idx >= p->len) {
            p->state = PKT_ST_CRC_HI;
        }
        break;

    case PKT_ST_CRC_HI:
        p->crc_rx = (uint16_t)((uint16_t)b << 8);
        p->state  = PKT_ST_CRC_LO;
        break;

    case PKT_ST_CRC_LO:
        p->crc_rx |= (uint16_t)b;
        p->state   = PKT_ST_MARKER0;        /* frame done either way         */
        if (p->crc_rx == p->crc_calc) {
            return true;                    /* valid packet ready            */
        }
        p->bad_crc = true;                  /* CRC mismatch -> caller may NAK */
        break;

    default:
        Packet_ParserReset(p);
        break;
    }
    return false;
}
