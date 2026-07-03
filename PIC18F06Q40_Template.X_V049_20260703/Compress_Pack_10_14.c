#define COMPRESS_PACK_10_14_C

#include "App_Config.h"

/* ============================================================
 *  Whole body compiles ONLY when this method is selected,
 *  so it costs zero RAM/ROM otherwise.
 *
 *    24 bits = [ sample#:10 ][ pulses:14 ]   (no padding)
 *
 *  Field A (time16 arg) = sample / group number   -> 10 bits
 *  Field B (pulses arg) = pulse delta for period   -> 14 bits
 * ============================================================ */
#if (COMPRESS_METHOD_SELECTED == COMPRESS_METHOD_PACK_10_14)

#include <stdint.h>
#include "Compress.h"

#define PACK_MAX_SAMPLE   1023u    /* 10-bit field max */
#define PACK_MAX_PULSES   16383u   /* 14-bit field max */

void Compress_Pack(uint16_t time16, uint16_t pulses, uint8_t *dst)
{
    /* saturate each field to its width */
    uint16_t g = (time16 > PACK_MAX_SAMPLE) ? PACK_MAX_SAMPLE : time16;
    uint16_t p = (pulses > PACK_MAX_PULSES) ? PACK_MAX_PULSES : pulses;

    /* 24-bit value: sample# in [23:14], pulses in [13:0] */
    uint32_t v = ((uint32_t)g << 14) | (uint32_t)p;

    dst[0] = (uint8_t)(v >> 16);   /* sample#[9:2]                 */
    dst[1] = (uint8_t)(v >> 8);    /* sample#[1:0] | pulses[13:8]  */
    dst[2] = (uint8_t)(v);         /* pulses[7:0]                  */
}

void Compress_Unpack(const uint8_t *src, uint16_t *time16, uint16_t *pulses)
{
    uint32_t v = ((uint32_t)src[0] << 16) |
                 ((uint32_t)src[1] << 8)  |
                 ((uint32_t)src[2]);

    *time16 = (uint16_t)((v >> 14) & 0x03FFu);   /* sample#:10 */
    *pulses = (uint16_t)( v        & 0x3FFFu);   /* pulses:14  */
}

#endif /* COMPRESS_METHOD_SELECTED == ... */
