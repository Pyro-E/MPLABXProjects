#ifndef COMPRESS_H
#define COMPRESS_H

#include <stdint.h>
#include "App_Config.h"

/* ============================================================
 *  Compress.h  -  uniform interface to the SELECTED method
 *
 *  Only the selected method's header is pulled in, and only that
 *  method's .c body compiles (each is wrapped in #if). Unused
 *  methods cost zero RAM/ROM.
 *
 *  Every method must provide:
 *    - COMPRESS_BYTES_PER_SAMPLE   (macro: bytes one sample takes)
 *    - Compress_Pack(time, pulses, dst)
 *    - Compress_Unpack(src, &time, &pulses)
 * ============================================================ */

#if   (COMPRESS_METHOD_SELECTED == COMPRESS_METHOD_NOCOMPRESS_2B_2B)
    #include "Compress_NoComp_2B_2B.h"
#elif (COMPRESS_METHOD_SELECTED == COMPRESS_METHOD_PACK_10_10_4)
    #include "Compress_Pack_10_10_4.h"
#else
    #error "App_Config.h: COMPRESS_METHOD_SELECTED is not a known method"
#endif

/* uniform API (implemented by the selected method) */
void Compress_Pack(uint16_t time16, uint16_t pulses, uint8_t *dst);
void Compress_Unpack(const uint8_t *src, uint16_t *time16, uint16_t *pulses);

#endif /* COMPRESS_H */
