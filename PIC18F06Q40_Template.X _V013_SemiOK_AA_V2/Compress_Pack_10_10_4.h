#ifndef COMPRESS_PACK_10_10_4_H
#define COMPRESS_PACK_10_10_4_H

#include "App_Config.h"

/* ============================================================
 *  Method 1: bit-pack into 3 bytes.
 *    24 bits = [ grp:10 ][ pulses:10 ][ unused:4 ]
 *
 *  Layout (MSB first):
 *    byte0 = grp[9:2]
 *    byte1 = grp[1:0] | pulses[9:4]
 *    byte2 = pulses[3:0] | 0000
 *
 *  Ranges: grp 0..1023, pulses 0..1023 (saturated at 1023).
 *  Saves 1 byte/sample vs the 4-byte method.
 *
 *  This header only contributes when this method is selected.
 * ============================================================ */
#if (COMPRESS_METHOD_SELECTED == COMPRESS_METHOD_PACK_10_10_4)

#define COMPRESS_BYTES_PER_SAMPLE   3u

#endif

#endif /* COMPRESS_PACK_10_10_4_H */
