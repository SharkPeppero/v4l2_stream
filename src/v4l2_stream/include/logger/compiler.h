﻿#ifndef _TOOLS_LINUX_COMPILER_H_
#define _TOOLS_LINUX_COMPILER_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <stddef.h>
#include <assert.h>

#if !defined STDCALL
#if _WIN32
#define STDCALL __stdcall
#else
#define STDCALL
#endif
#endif

#if !defined __extern__
#if defined __cplusplus
#define __extern__  extern "C"
#else
#define __extern__  extern
#endif /*__cplusplus */
#endif

#if !defined __ALIGNED_SIZE__
#define __ALIGNED_SIZE__        4/*(sizeof(void *))*/
#endif /* !__ALIGNED_SIZE__ */

#if !_WIN32
#if !defined __POSIX_TYPE_ALIGNED__
#define __POSIX_TYPE_ALIGNED__ /*__attribute__((aligned(__ALIGNED_SIZE__))) */
#endif
#else
#define __POSIX_TYPE_ALIGNED__
#endif

#if !defined __POSIX_POINTER_ALIGNED__
#define __POSIX_POINTER_ALIGNED__(ptr)   ((0 == ((long)ptr) % __ALIGNED_SIZE__))
#endif /* !__POSIX_POINTER_ALIGNED__ */

#if !defined __POSIX_EFFICIENT_ALIGNED_PTR__
#define __POSIX_EFFICIENT_ALIGNED_PTR__(ptr)    ((NULL != ptr) && __POSIX_POINTER_ALIGNED__(ptr))
#endif /* !__POSIX_EFFICIENT_ALIGNED_PTR__ */

#define __POSIX_EFFICIENT_PTR__(ptr) ((NULL != ptr))
#define __POSIX_EFFICIENT_PTR_IR__(ptr) do { if (!__POSIX_EFFICIENT_PTR__(ptr)) return -EINVAL; } while (0)
#define __POSIX_EFFICIENT_PTR_NR__(ptr) do { if (!__POSIX_EFFICIENT_PTR__(ptr)) return; } while (0)

#define __POSIX_EFFICIENT_ALIGNED_PTR_IR__(ptr) do { if (!__POSIX_EFFICIENT_ALIGNED_PTR__(ptr)) return -EINVAL; } while (0)
#define __POSIX_EFFICIENT_ALIGNED_PTR_NR__(ptr) do { if (!__POSIX_EFFICIENT_ALIGNED_PTR__(ptr)) return; } while (0)

#if !defined UINT64_STRFMT
#if _WIN32
#define UINT64_STRFMT "%I64u"
#else
#define UINT64_STRFMT "%llu"
#endif
#endif

#if !defined INT64_STRFMT
#if _WIN32
#define INT64_STRFMT "%I64d"
#else
#define INT64_STRFMT "%lld"
#endif
#endif

#if !defined POSIX__EOL
#if _WIN32
#define POSIX__EOL              "\r\n"
#define POSIX__DIR_SYMBOL       '\\'
#define POSIX__DIR_SYMBOL_STR   "\\"
#else
#define POSIX__EOL              "\n"
#define POSIX__DIR_SYMBOL       '/'
#define POSIX__DIR_SYMBOL_STR   "/"
#endif
#endif


#if !defined INET_ADDRSTRLEN
#define INET_ADDRSTRLEN 16
#endif

#if !defined INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 46
#endif

#if !_WIN32
#if defined __USE_MISC
#if !__USE_MISC
#undef __USE_MISC
#define __USE_MISC 1 /* syscall nice in <unistd.h> */
#endif /* !__USE_MISC */
#else
#define __USE_MISC 1
#endif /* defined __USE_MISC */
#endif /* !_WIN32 */

#if _WIN32
#if !defined NT_SUCCESS
#define NT_SUCCESS(Status) ((NTSTATUS)(Status) >= 0)
#endif
#endif /* _WIN32 */

#if !defined __always_inline
#if _WIN32
#define __always_inline __forceinline
#else
#define __always_inline inline
#endif
#endif

#if defined __static_inline_function
#undef __static_inline_function
#endif

#if _WIN32
#define __static_inline_function(type) __always_inline static type
#else
#define __static_inline_function(type) static __always_inline type
#endif

#if !defined NULL
#define NULL ((void *)0)
#endif

#if !defined MAXPATH
#define MAXPATH (260)
#endif

#ifndef __cplusplus
#if !defined max
#define max(a,b)    (((a) > (b)) ? (a) : (b))
#endif
#if !defined min
#define min(a,b)    (((a) < (b)) ? (a) : (b))
#endif
#endif

/* 判断 @x 是否为 2 的正整次幂 */
#if !defined is_powerof_2
#define is_powerof_2(x) ((x) != 0 && (((x) & ((x) - 1)) == 0))
#endif

/* 浮点为0的对比目标 */
#if !defined EPSINON
#define EPSINON  0.000001
#endif

/* 判断浮点数 @x 是否为 0 */
#if !defined is_float_zero
#define is_float_zero(x) (((x) < EPSINON) && ((x) > -EPSINON))
#endif

/* 判断浮点数 @n @m 是否相等 */
#if !defined is_float_equal
#define is_float_equal(n, m) ((fabs((n)-(m))) <= EPSINON )
#endif

#if !defined containing_record
#define containing_record(__address, __type, __field) ((__type *)( (char *)(__address) -  (char *)(&((__type *)0)->__field)))
#endif

#if !defined cchof
#define cchof(__array)   (int)(sizeof(__array) / sizeof(__array[0]))
#endif

#if !defined offsetof
#define offsetof(__type, __field)      (( unsigned long )(&((__type*)0)->__field))
#endif

#if !defined msizeof
#define msizeof(__type, __field)      (sizeof(((__type*)0)->__field))
#endif

#if !defined container_of
#define container_of(__address, __type, __field) containing_record(__address, __type, __field)
#endif

#if _WIN32

#if !defined smp_mb
#define smp_mb() do {__asm { mfence } } while( 0 )
#endif

#if !defined smp_rmb
#define smp_rmb() do {__asm { lfence } } while( 0 )
#endif

#if !defined smp_wmb
#define smp_wmb() do {__asm { sfence } } while( 0 )
#endif

#else /* _GNU_ */

#if !defined smp_mb
#define smp_mb() /* asm volatile("mfence" ::: "memory") */
#endif

#if !defined smp_rmb
#define smp_rmb() /* asm volatile("lfence" ::: "memory") */
#endif

#if !defined smp_wmb
#define smp_wmb() /* asm volatile("sfence" ::: "memory")  */
#endif

#endif /* !_WIN32 */

/* Optimization barrier */
#ifndef barrier
#define barrier() __memory_barrier()
#endif

#ifndef barrier_data
#define barrier_data(ptr) barrier()
#endif

/* Unreachable code */
#ifndef unreachable
#define unreachable() do { } while (1)
#endif

/**
 * fls - find last bit set
 * @x: the word to search
 *
 * This is defined the same way as ffs:
 * - return 32..1 to indicate bit 31..0 most significant bit set
 * - return 0 to indicate no bits set
 */
#if _WIN32

__static_inline_function(int) fls(int x) {
    int position;
    int i;
    if (0 != x) {
        for (i = (x >> 1), position = 0; i != 0; ++position) {
            i >>= 1;
        }
    } else {
        position = -1;
    }
    return position + 1;
}
#else

__static_inline_function(int) fls(int x) {
    int r;
    __asm__( "bsrl %1,%0\n\t"
            "jnz 1f\n\t"
            "movl $-1,%0\n"
            "1:" : "=r" (r) : "rm" (x));
    return r + 1;
}
#endif

/**
 * fls64 - find last bit set in a 64-bit value
 * @n: the value to search
 *
 * This is defined the same way as ffs:
 * - return 64..1 to indicate bit 63..0 most significant bit set
 * - return 0 to indicate no bits set
 */
__static_inline_function(int) fls64(uint64_t x) {
    uint32_t h = x >> 32;
    if (h)
        return fls(h) + 32;
    return fls((int) x);
}

__static_inline_function(unsigned long) fls_long(unsigned long l) {
    if (sizeof ( void *) == 4)
        return fls(l);
    return fls64(l);
}

__static_inline_function(uint32_t) roundup_pow_of_two(uint32_t x) {
    return 1UL << fls(x - 1);
}

__static_inline_function(uint64_t) roundup_pow_of_two64(uint64_t x) {
    return (uint64_t) 1 << fls64(x - 1);
}

static __always_inline void __read_once_size(const volatile void *p, void *res, int size) {
    switch (size) {
        case 1: *(uint8_t *) res = *(volatile uint8_t *) p;
            break;
        case 2: *(uint16_t *) res = *(volatile uint16_t *) p;
            break;
        case 4: *(uint32_t *) res = *(volatile uint32_t *) p;
            break;
        case 8: *(uint64_t *) res = *(volatile uint64_t *) p;
            break;
        default: break;
    }
}

__static_inline_function(void) __write_once_size(volatile void *p, void *res, int size) {
    switch (size) {
        case 1: *(volatile uint8_t *) p = *(uint8_t *) res;
            break;
        case 2: *(volatile uint16_t *) p = *(uint16_t *) res;
            break;
        case 4: *(volatile uint32_t *) p = *(uint32_t *) res;
            break;
        case 8: *(volatile uint64_t *) p = *(uint64_t *) res;
            break;
        default: break;
    }
}

#if !defined write_once
#if _WIN32
#define write_once(xtype, x, val) \
        { union { xtype __val; char __c[1]; } __u = { .__val = (val) }; __write_once_size(&(x), __u.__c, sizeof(x)); __u.__val; }
#else
#define write_once(xtype, x, val) \
        { union { xtype __val; char __c[1]; } __u = { .__val = (val) }; __write_once_size(&(x), __u.__c, sizeof(x)); __u.__val; }
/*({ union { typeof(x) __val; char __c[1]; } __u = { .__val = (val) }; __write_once_size(&(x), __u.__c, sizeof(x)); __u.__val; })*/
#endif
#endif

#if !defined read_once
#if _WIN32
#define read_once(xtype, x) \
        { union { xtype __val; char __c[1]; } __u; __read_once_size(&(x), __u.__c, sizeof(x)); __u.__val; }
#else
#define read_once(xtype, x) \
        { union { xtype __val; char __c[1]; } __u; __read_once_size(&(x), __u.__c, sizeof(x)); __u.__val; }
/* ({ union { typeof(x) __val; char __c[1]; } __u; __read_once_size(&(x), __u.__c, sizeof(x)); __u.__val; }) */
#endif
#endif

#if !defined access_once
#if _WIN32
#define access_once(__type, __x) (*(volatile __type *)&(x))
#else
#define access_once(__type, __x) (*(volatile typeof(x) *)&(x))
#endif
#endif

#if defined BITS_P_BYTE
#undef BITS_P_BYTE
#endif
#define BITS_P_BYTE     (8)

#if defined PI
#undef PI
#endif
#define PI ((double)3.14159265359)

enum byte_order_t {
    kByteOrder_LittleEndian = 0,
    kByteOrder_BigEndian,
};

#if !defined PAGE_SIZE
#define PAGE_SIZE (4096)
#endif

#if !defined BYTES_PER_SECTOR
#define BYTES_PER_SECTOR		(512)
#endif

#define posix__makeerror(e)   (((int)(e)) <= 0 ? (e) : (int)(~((int)(e)) + 1))

/* the maximum of integer */
#define NSP_MAX_UINT8       (0xFF)
#define NSP_MAX_INT8        (0x7f)

#define NSP_MAX_UINT16      (0xFFFF)
#define NSP_MAX_INT16       (0x7FFF)

#define NSP_MAX_UINT32      (0xFFFFFFFF)
#define NSP_MAX_INT32       (0x7FFFFFFF)

#define NSP_MAX_UINT64      (0xFFFFFFFFFFFFFFFF)
#define NSP_MAX_INT64       (0x7FFFFFFFFFFFFFFF)

#define sal(n, b)               ((n) << (b))
#define sal_set(n, b)           ((n) <<= (b))
#define sar(n, b)               ((n) >> (b))
#define sar_set(n, b)           ((n) >>= (b))
#define bit_set(n, b)           ((n) |= sal(1, b))
#define bit_unset(n, b)         ((n) &= ~sal(1, b))
#define is_bit_set(n, b)        ((n) & sal(1, b))
#define is_bit_unset(n, b)      ((~(n)) & sal(1, b))

#if !defined RE_ERROR
#define RE_ERROR(eval)   ((((int)eval) > 0 ) ? (int)((~(int)(eval)) + 1) : eval)
#endif 
#if !defined RE_ERRNO
#define RE_ERRNO    RE_ERROR(errno)
#endif 

#endif
