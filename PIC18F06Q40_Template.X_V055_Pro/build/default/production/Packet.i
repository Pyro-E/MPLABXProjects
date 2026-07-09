# 1 "Packet.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 295 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "/Applications/microchip/xc8/v3.10/pic/include/language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "Packet.c" 2




# 1 "./Packet.h" 1
# 19 "./Packet.h"
# 1 "/Applications/microchip/xc8/v3.10/pic/include/c99/stdint.h" 1 3



# 1 "/Applications/microchip/xc8/v3.10/pic/include/c99/musl_xc8.h" 1 3
# 5 "/Applications/microchip/xc8/v3.10/pic/include/c99/stdint.h" 2 3
# 26 "/Applications/microchip/xc8/v3.10/pic/include/c99/stdint.h" 3
# 1 "/Applications/microchip/xc8/v3.10/pic/include/c99/bits/alltypes.h" 1 3
# 133 "/Applications/microchip/xc8/v3.10/pic/include/c99/bits/alltypes.h" 3
typedef unsigned __int24 uintptr_t;
# 148 "/Applications/microchip/xc8/v3.10/pic/include/c99/bits/alltypes.h" 3
typedef __int24 intptr_t;
# 164 "/Applications/microchip/xc8/v3.10/pic/include/c99/bits/alltypes.h" 3
typedef signed char int8_t;




typedef short int16_t;




typedef __int24 int24_t;




typedef long int32_t;





typedef long long int64_t;
# 194 "/Applications/microchip/xc8/v3.10/pic/include/c99/bits/alltypes.h" 3
typedef long long intmax_t;





typedef unsigned char uint8_t;




typedef unsigned short uint16_t;




typedef __uint24 uint24_t;




typedef unsigned long uint32_t;





typedef unsigned long long uint64_t;
# 235 "/Applications/microchip/xc8/v3.10/pic/include/c99/bits/alltypes.h" 3
typedef unsigned long long uintmax_t;
# 27 "/Applications/microchip/xc8/v3.10/pic/include/c99/stdint.h" 2 3

typedef int8_t int_fast8_t;

typedef int64_t int_fast64_t;


typedef int8_t int_least8_t;
typedef int16_t int_least16_t;

typedef int24_t int_least24_t;
typedef int24_t int_fast24_t;

typedef int32_t int_least32_t;

typedef int64_t int_least64_t;


typedef uint8_t uint_fast8_t;

typedef uint64_t uint_fast64_t;


typedef uint8_t uint_least8_t;
typedef uint16_t uint_least16_t;

typedef uint24_t uint_least24_t;
typedef uint24_t uint_fast24_t;

typedef uint32_t uint_least32_t;

typedef uint64_t uint_least64_t;
# 148 "/Applications/microchip/xc8/v3.10/pic/include/c99/stdint.h" 3
# 1 "/Applications/microchip/xc8/v3.10/pic/include/c99/bits/stdint.h" 1 3
typedef int16_t int_fast16_t;
typedef int32_t int_fast32_t;
typedef uint16_t uint_fast16_t;
typedef uint32_t uint_fast32_t;
# 149 "/Applications/microchip/xc8/v3.10/pic/include/c99/stdint.h" 2 3
# 20 "./Packet.h" 2
# 1 "/Applications/microchip/xc8/v3.10/pic/include/c99/stdbool.h" 1 3
# 21 "./Packet.h" 2
# 44 "./Packet.h"
typedef enum {

    PKT_REQ_DATA = 0x01u,
    PKT_REQ_GET_PARAM = 0x02u,
    PKT_REQ_SET_PARAM = 0x03u,
    PKT_REQ_GET_VALVE = 0x04u,
    PKT_REQ_VALVE_UNLOCK = 0x05u,
    PKT_SYS_RESET = 0x06u,
    PKT_PHOTON_OFF_REQ = 0x07u,


    PKT_REQ_POWER_STATE = 0x08u,



    PKT_REQ_PHOTON_CFG = 0x09u,






    PKT_KEEPALIVE = 0x0Au,
# 75 "./Packet.h"
    PKT_RSP_DATA = 0x81u,






    PKT_RSP_PARAM = 0x82u,
    PKT_RSP_VALVE = 0x84u,
    PKT_RSP_POWER_STATE = 0x88u,
    PKT_RSP_PHOTON_CFG = 0x89u,
# 94 "./Packet.h"
    PKT_RSP_ACK = 0x8Eu,
    PKT_RSP_NAK = 0x8Fu
} pkt_func_t;


typedef enum {
    NAK_BAD_CRC = 0x01u,
    NAK_BAD_LEN = 0x02u,
    NAK_BAD_FUNC = 0x03u,
    NAK_BUSY = 0x04u
} nak_reason_t;




typedef enum {
    OFF_REASON_DONE = 0x00u,
    OFF_REASON_CLOUD_FAIL = 0x01u
} photon_off_reason_t;






typedef enum {
    POWER_STATE_INITIAL = 0x00u,
    POWER_STATE_NORMAL = 0x01u
} power_state_t;


uint16_t Packet_CRC16(const uint8_t *data, uint16_t len);

uint16_t Packet_CRC16_Init(void);
uint16_t Packet_CRC16_Update(uint16_t crc, uint8_t b);


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
    uint8_t func;
    uint16_t len;
    uint16_t idx;
    uint8_t data[8u];
    uint16_t crc_calc;
    uint16_t crc_rx;
    uint32_t last_byte_ms;
    _Bool bad_len;
    _Bool bad_crc;
} pkt_parser_t;


void Packet_ParserReset(pkt_parser_t *p);



void Packet_ParserTimeoutCheck(pkt_parser_t *p);





_Bool Packet_ParseByte(pkt_parser_t *p, uint8_t b);
# 6 "Packet.c" 2
# 1 "./MCU_Time.h" 1
# 20 "./MCU_Time.h"
void MCU_Time_Init(unsigned char ucKHz);
void MCU_Time_Increase_Unit(void);
void MCU_Time_Advance(uint32_t unMs);

uint32_t getNowTime(void);
uint32_t timeSpan(uint32_t unOldTime_ms);
void MCU_Time_Delay_Ms(uint32_t unMs);
# 7 "Packet.c" 2





uint16_t Packet_CRC16_Init(void)
{
    return 0xFFFFu;
}

uint16_t Packet_CRC16_Update(uint16_t crc, uint8_t b)
{
    crc ^= (uint16_t)b;
    for (uint8_t i = 0u; i < 8u; i++) {
        if (crc & 0x0001u) crc = (uint16_t)((crc >> 1) ^ 0xA001u);
        else crc = (uint16_t)(crc >> 1);
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




void Packet_ParserReset(pkt_parser_t *p)
{
    p->state = PKT_ST_MARKER0;
    p->func = 0u;
    p->len = 0u;
    p->idx = 0u;
    p->crc_calc= 0u;
    p->crc_rx = 0u;

}

void Packet_ParserTimeoutCheck(pkt_parser_t *p)
{

    if (p->state != PKT_ST_MARKER0) {
        if (timeSpan(p->last_byte_ms) >= 1000UL) {
            Packet_ParserReset(p);
        }
    }
}

_Bool Packet_ParseByte(pkt_parser_t *p, uint8_t b)
{
    p->last_byte_ms = getNowTime();

    switch (p->state) {

    case PKT_ST_MARKER0:
        if (b == 0xAAu) {
            p->bad_len = 0;
            p->bad_crc = 0;
            p->state = PKT_ST_MARKER1;
        }
        break;

    case PKT_ST_MARKER1:
        if (b == 0x55u) {
            p->state = PKT_ST_FUNC;
        } else if (b == 0xAAu) {
            p->state = PKT_ST_MARKER1;
        } else {
            p->state = PKT_ST_MARKER0;
        }
        break;

    case PKT_ST_FUNC:
        p->func = b;
        p->crc_calc = Packet_CRC16_Init();
        p->crc_calc = Packet_CRC16_Update(p->crc_calc, b);
        p->state = PKT_ST_LEN_HI;
        break;

    case PKT_ST_LEN_HI:
        p->len = (uint16_t)((uint16_t)b << 8);
        p->crc_calc = Packet_CRC16_Update(p->crc_calc, b);
        p->state = PKT_ST_LEN_LO;
        break;

    case PKT_ST_LEN_LO:
        p->len |= (uint16_t)b;
        p->crc_calc = Packet_CRC16_Update(p->crc_calc, b);
        p->idx = 0u;

        if (p->len > 8u) {
            p->bad_len = 1;
            Packet_ParserReset(p);
            break;
        }
        p->state = (p->len == 0u) ? PKT_ST_CRC_HI : PKT_ST_DATA;
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
        p->state = PKT_ST_CRC_LO;
        break;

    case PKT_ST_CRC_LO:
        p->crc_rx |= (uint16_t)b;
        p->state = PKT_ST_MARKER0;
        if (p->crc_rx == p->crc_calc) {
            return 1;
        }
        p->bad_crc = 1;
        break;

    default:
        Packet_ParserReset(p);
        break;
    }
    return 0;
}
