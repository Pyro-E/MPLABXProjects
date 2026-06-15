# 1 "Compress_NoComp_2B_2B.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 295 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "/Applications/microchip/xc8/v3.10/pic/include/language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "Compress_NoComp_2B_2B.c" 2


# 1 "./App_Config.h" 1
# 4 "Compress_NoComp_2B_2B.c" 2







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
# 12 "Compress_NoComp_2B_2B.c" 2
# 1 "./Compress.h" 1
# 21 "./Compress.h"
# 1 "./Compress_NoComp_2B_2B.h" 1
# 22 "./Compress.h" 2







void Compress_Pack(uint16_t time16, uint16_t pulses, uint8_t *dst);
void Compress_Unpack(const uint8_t *src, uint16_t *time16, uint16_t *pulses);
# 13 "Compress_NoComp_2B_2B.c" 2

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
