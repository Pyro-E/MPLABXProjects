#define COMPRESS_PACK_10_10_4_C

#include "App_Config.h"

/* ============================================================
 *  Whole body compiles ONLY when this method is selected,
 *  so it costs zero RAM/ROM otherwise.
 * ============================================================ */
#if (COMPRESS_METHOD_SELECTED == COMPRESS_METHOD_PACK_10_10_4)

#include <stdint.h>
#include "Compress.h"

#define PACK_MAX_10BIT   1023u

void Compress_Pack(uint16_t time16, uint16_t pulses, uint8_t *dst)
{
    /* clamp each field to 10 bits (saturate) */
    uint16_t g = (time16 > PACK_MAX_10BIT) ? PACK_MAX_10BIT : time16;
    uint16_t p = (pulses > PACK_MAX_10BIT) ? PACK_MAX_10BIT : pulses;

    /* 24-bit value: grp in [23:14], pulses in [13:4], low 4 unused */
    uint32_t v = ((uint32_t)g << 14) | ((uint32_t)p << 4);

    dst[0] = (uint8_t)(v >> 16);
    dst[1] = (uint8_t)(v >> 8);
    dst[2] = (uint8_t)(v);
}

void Compress_Unpack(const uint8_t *src, uint16_t *time16, uint16_t *pulses)
{
    uint32_t v = ((uint32_t)src[0] << 16) |
                 ((uint32_t)src[1] << 8)  |
                 ((uint32_t)src[2]);

    *time16 = (uint16_t)((v >> 14) & 0x03FFu);   /* grp:10 */
    *pulses = (uint16_t)((v >> 4)  & 0x03FFu);   /* pulses:10 */
}

#endif /* COMPRESS_METHOD_SELECTED == ... */
