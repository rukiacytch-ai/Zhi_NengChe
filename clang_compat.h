#ifndef CLANG_COMPAT_H
#define CLANG_COMPAT_H

/* Tasking compiler keyword polyfills for Clangd */

#define __size_t unsigned int
#define __wchar_t unsigned short
#define __ptrdiff_t int
#define __intptr_t int
#define __uintptr_t unsigned int
#define __int32_t int
#define __uint32_t unsigned int
#define __int16_t short
#define __uint16_t unsigned short
#define __int8_t signed char
#define __uint8_t unsigned char
#define __int64_t long long
#define __uint64_t unsigned long long

/* Standard C99 types for Clangd compatibility */
typedef unsigned char      uint8_t;
typedef unsigned short     uint16_t;
typedef unsigned int       uint32_t;
typedef unsigned long long uint64_t;
typedef signed char        int8_t;
typedef signed short       int16_t;
typedef signed int         int32_t;
typedef signed long long   int64_t;

#define __far
#define __near
#define __sfr
#define __a0
#define __a1
#define __a8
#define __a9
#define __syscall(...)
#define __at(...)
#define __align(...)
#define __sfrbit8
#define __sfrbit16
#define __sfrbit32
#define __bit unsigned char
#define _INLINE_ inline
#define __indirect

/* Fixed-point types */
#ifndef __fract
#define __fract float
#endif
#ifndef __sfract
#define __sfract float
#endif
#ifndef __accum
#define __accum float
#endif
#ifndef __laccum
#define __laccum float
#endif
#define __sat

/* TriCore specific intrinsics */
#define __nop()
#define __enable()
#define __disable()
#define __isync()
#define __dsync()
#define __debug()
#define __ischar char
#define __mtcr(...)
#define __mfcr(...) 0
#define __extru(...) 0
#define __extr(...) 0
#define __insert(...) 0
#define __imaskldmst(...)
#define __cmpswapw(...) 0
#define __round16(...) 0
#define __abs(...) 0
#define __fabs(x) (x)

/* Tasking interrupts and attributes */
#define __interrupt(...)
#define __vector_table(...)
#define __interrupt_fast(...)
#define __trap(...)
#define __bisr_(...)

/* Frequently used types in AURIX projects if headers are missed */
#ifndef uint32
typedef unsigned long uint32;
#endif
#ifndef uint16
typedef unsigned short uint16;
#endif
#ifndef uint8
typedef unsigned char uint8;
#endif
#ifndef int32
typedef int int32;
#endif
#ifndef int16
typedef short int16;
#endif
#ifndef int8
typedef signed char int8;
#endif
#ifndef boolean
typedef unsigned char boolean;
#endif

#ifndef float32
typedef float float32;
#endif
#ifndef float64
typedef double float64;
#endif

#ifndef NULL
#define NULL ((void *)0)
#endif

/* Standard library polyfills for IntelliSense */
int printf(const char *format, ...);
double fabs(double x);

/* Suppress Tasking specific pragmas that Clang doesn't like */
#pragma clang diagnostic ignored "-Wunknown-pragmas"
#pragma clang diagnostic ignored "-Wmicrosoft-anon-tag"

#endif
