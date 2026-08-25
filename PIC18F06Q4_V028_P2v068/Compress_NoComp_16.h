#ifndef COMPRESS_NOCOMP_16_H
#define COMPRESS_NOCOMP_16_H

#include "App_Config.h"

/* ============================================================
 *  Method 3: 16-bit pulses only, 2 bytes/sample (NO sample #).
 *
 *  Layout (MSB first):
 *    byte0 = pulses[15:8]
 *    byte1 = pulses[7:0]
 *
 *  Range: pulses 0..65535 (saturated).
 *
 *  Rationale: the 10-bit sample number is no longer needed - time
 *  alignment on the Photon uses the report's boundary timestamps +
 *  sample count, not a per-sample index. Dropping the identifier:
 *    - shrinks the ring buffer 3 B -> 2 B/sample (~33% RAM saved), and
 *    - widens the pulse field 14b -> 16b (16383 -> 65535, 4x), which
 *      drastically reduces per-capture overflow.
 *
 *  The uniform Compress_Pack(time16, pulses, dst) still takes a time16
 *  argument for API compatibility, but this method IGNORES it. On
 *  unpack, time16 is returned as 0 (the caller derives position from
 *  the sample's index within the batch instead).
 *
 *  Because there is no 10-bit index, the ring buffer is NOT limited to
 *  1024 samples by this method (it is limited only by SRAM).
 *
 *  This header only contributes when this method is selected.
 * ============================================================ */
#if (COMPRESS_METHOD_SELECTED == COMPRESS_METHOD_NOCOMP_16)

#define COMPRESS_BYTES_PER_SAMPLE   2u
#define COMPRESS_PULSE_MAX          65535u   /* 16-bit field max */

#endif

#endif /* COMPRESS_NOCOMP_16_H */
