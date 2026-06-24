#define COMPRESS_NOCOMP_2B_2B_C

#include "App_Config.h"

/* ============================================================
 *  Whole body compiles ONLY when this method is selected.
 *  Otherwise this file produces no code/data at all.
 * ============================================================ */
#if (COMPRESS_METHOD_SELECTED == COMPRESS_METHOD_NOCOMPRESS_2B_2B)

#include <stdint.h>
#include "Compress.h"

void Compress_Pack(uint16_t time16, uint16_t pulses, uint8_t *dst)
{
    dst[0] = (uint8_t)(time16 >> 8);
    dst[1] = (uint8_t)(time16 & 0xFF);
    dst[2] = (uint8_t)(pulses >> 8);
    dst[3] = (uint8_t)(pulses & 0xFF);
}

void Compress_Unpack(const uint8_t *src, uint16_t *time16, uint16_t *pulses)
{
    *time16 = (uint16_t)(((uint16_t)src[0] << 8) | src[1]);
    *pulses = (uint16_t)(((uint16_t)src[2] << 8) | src[3]);
}

#endif /* COMPRESS_METHOD_SELECTED == ... */
