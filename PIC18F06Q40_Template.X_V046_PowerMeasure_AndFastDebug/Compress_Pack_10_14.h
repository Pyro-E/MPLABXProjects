#ifndef COMPRESS_PACK_10_14_H
#define COMPRESS_PACK_10_14_H

#include "App_Config.h"

/* ============================================================
 *  Method 2: bit-pack into 3 bytes  (NO wasted bits).
 *    24 bits = [ sample#:10 ][ pulses:14 ]
 *
 *  Layout (MSB first):
 *    byte0 = sample#[9:2]
 *    byte1 = sample#[1:0] | pulses[13:8]
 *    byte2 = pulses[7:0]
 *
 *  Ranges: sample# 0..1023, pulses 0..16383 (saturated).
 *
 *  Same 3 B/sample as 10-10-4, but the 4 previously-unused bits
 *  are reclaimed to widen the pulse field 10b -> 14b
 *  (1024 -> 16384). No extra RAM, no extra ROM per sample.
 *
 *  CONSTRAINT: sample# stays 10-bit, so the ring buffer must hold
 *  at most 1024 samples (a larger buffer would saturate the index).
 *
 *  This header only contributes when this method is selected.
 * ============================================================ */
#if (COMPRESS_METHOD_SELECTED == COMPRESS_METHOD_PACK_10_14)

#define COMPRESS_BYTES_PER_SAMPLE   3u

#endif

#endif /* COMPRESS_PACK_10_14_H */
