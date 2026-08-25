/* ============================================================
 *  Compress_NoComp_16.c  -  Method 3: 16-bit pulses only (2 B/sample).
 *  See Compress_NoComp_16.h for the layout and rationale.
 * ============================================================ */

#include "App_Config.h"
#include "Compress.h"

#if (COMPRESS_METHOD_SELECTED == COMPRESS_METHOD_NOCOMP_16)

/* Pack: store 16-bit pulses, MSB first. The sample-number argument
 * (time16) is intentionally ignored by this method. */
void Compress_Pack(uint16_t time16, uint16_t pulses, uint8_t *dst)
{
    (void)time16;                       /* not stored - see header note */
    dst[0] = (uint8_t)(pulses >> 8);    /* pulses[15:8] */
    dst[1] = (uint8_t)(pulses & 0xFFu); /* pulses[7:0]  */
}

/* Unpack: recover 16-bit pulses; there is no stored sample number, so
 * time16 is returned as 0 (caller uses the in-batch index instead). */
void Compress_Unpack(const uint8_t *src, uint16_t *time16, uint16_t *pulses)
{
    *pulses = (uint16_t)(((uint16_t)src[0] << 8) | (uint16_t)src[1]);
    *time16 = 0u;
}

#endif
