#ifndef COMPRESS_NOCOMP_2B_2B_H
#define COMPRESS_NOCOMP_2B_2B_H

#include "App_Config.h"

/* ============================================================
 *  Method 0: NO compression, 2 bytes time + 2 bytes pulses.
 *  Layout per sample (4 bytes, big-endian):
 *    [0]=time hi  [1]=time lo  [2]=pulses hi  [3]=pulses lo
 *
 *  This header only contributes when this method is selected.
 * ============================================================ */
#if (COMPRESS_METHOD_SELECTED == COMPRESS_METHOD_NOCOMPRESS_2B_2B)

#define COMPRESS_BYTES_PER_SAMPLE   4u

#endif

#endif /* COMPRESS_NOCOMP_2B_2B_H */
