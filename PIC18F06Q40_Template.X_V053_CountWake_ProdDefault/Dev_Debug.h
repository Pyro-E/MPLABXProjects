#ifndef DEV_DEBUG_H
#define DEV_DEBUG_H

#include <stdint.h>
#include "Dev_Uart.h"
#include "App_Config.h"

/* ============================================================
 *  Dev_Debug.h  -  switchable debug print layer over UART
 *
 *  Define DEBUG_PRINT_ENABLE below to turn ALL debug output on.
 *  Comment it out and every Debug_Print_* call compiles to
 *  nothing (no function call, no code, no string in flash).
 *
 *  Usage:
 *    Debug_Print_String("count=");
 *    Debug_Print_Uint(count);
 *    Debug_Print_Int(delta);
 *    Debug_Print_Char('\n');
 * ============================================================ */

/* Master switch lives in App_Config.h (APP_DEBUG_PRINT_ENABLE). */
#ifdef APP_DEBUG_PRINT_ENABLE
  #define DEBUG_PRINT_ENABLE
#endif


#ifdef DEBUG_PRINT_ENABLE
    /* enabled: map straight onto the UART primitives */
    #define Debug_Print_Char(c)     print_char(c)
    #define Debug_Print_String(s)   print_string(s)
    #define Debug_Print_Uint(v)     print_uint(v)
    #define Debug_Print_Int(v)      print_int(v)
#else
    /* disabled: expand to nothing. The (void)0 keeps a trailing
     * semicolon legal, e.g. "Debug_Print_Uint(x);" stays valid. */
    #define Debug_Print_Char(c)     ((void)0)
    #define Debug_Print_String(s)   ((void)0)
    #define Debug_Print_Uint(v)     ((void)0)
    #define Debug_Print_Int(v)      ((void)0)
#endif

#endif /* DEV_DEBUG_H */
