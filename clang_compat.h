#ifndef CLANG_COMPAT_H
#define CLANG_COMPAT_H

/* 
 * Tasking compiler keyword polyfills for Clangd 
 */

#ifndef __TASKING__
#define __TASKING__ 1
#endif

#ifndef __CTC__
#define __CTC__ 1
#endif

#ifndef __TRICORE__
#define __TRICORE__ 1
#endif

/* Tasking Keywords */
#define __far
#define __near
#define __sfr
#define __a0
#define __a1
#define __a8
#define __a9
#define __sfrbit8
#define __sfrbit16
#define __sfrbit32
#define __bit           unsigned char
#define _INLINE_        inline
#define __ischar        char

/* Tasking Attributes and pragmas shims */
#define __interrupt(...)
#define __vector_table(...)
#define __interrupt_fast(...)
#define __trap(...)
#define __bisr_(...)
#define __syscall(...)
#define __at(...)
#define __align(n)      __attribute__((aligned(n)))

/* TriCore specific built-ins */
#define __nop()         ((void)0)
#define __enable()      ((void)0)
#define __disable()     ((void)0)
#define __mtcr(reg,val) ((void)0)
#define __mfcr(reg)     (0U)
#define __extru(v,p,w)  (0U)
#define __extr(v,p,w)   (0)
#define __insert(d,s,p,w) (0U)
#define __imaskldmst(a,v,p,w) ((void)0)
#define __cmpswapw(a,v,c) (0U)
#define __round16(v)    (0)
#define __dsync()       ((void)0)
#define __isync()       ((void)0)

/* Fixed-point types */
#define __fract         float
#define __sfract        float
#define __accum         float
#define __laccum        float
#define __sat

/* 
 * Basic types - Aligned with Platform_Types.h (AURIX) and zf_common_typedef.h
 * Platform_Types.h: uint32 is 'unsigned long'
 * zf_common_typedef.h: int32 is 'signed int'
 */

#ifndef uint32
typedef unsigned long uint32;
#define uint32 uint32
#endif

#ifndef uint16
typedef unsigned short uint16;
#define uint16 uint16
#endif

#ifndef uint8
typedef unsigned char uint8;
#define uint8 uint8
#endif

#ifndef int32
typedef signed int int32;
#define int32 int32
#endif

#ifndef int16
typedef short int16;
#define int16 int16
#endif

#ifndef int8
typedef signed char int8;
#define int8 int8
#endif

#ifndef boolean
typedef unsigned char boolean;
#define boolean boolean
#endif

#ifndef float32
typedef float float32;
#define float32 float32
#endif

#ifndef float64
typedef double float64;
#define float64 float64
#endif

#ifndef NULL
#define NULL ((void *)0)
#endif

/* Suppress some Clang-specific warnings that are noise in Tasking projects */
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wmain-return-type"
#pragma clang diagnostic ignored "-Wimplicitly-unsigned-literal"
#pragma clang diagnostic ignored "-Wtypedef-redefinition"

#endif /* CLANG_COMPAT_H */
