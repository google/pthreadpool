// Copyright (c) 2017 Facebook Inc.
// Copyright (c) 2015-2017 Georgia Institute of Technology
// All rights reserved.
//
// Copyright 2019 Google LLC
//
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef __PTHREADPOOL_SRC_THREADPOOL_UTILS_H_
#define __PTHREADPOOL_SRC_THREADPOOL_UTILS_H_

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

/* SSE-specific headers */
#if defined(__SSE__) || defined(__x86_64__) || \
    defined(_M_X64) && !defined(_M_ARM64EC) || \
    (defined(_M_IX86_FP) && _M_IX86_FP >= 1)
#include <xmmintrin.h>
#endif

/* MSVC-specific headers */
#if defined(_MSC_VER)
#include <intrin.h>
#endif

struct fpu_state {
#if defined(__GNUC__) && defined(__arm__) && defined(__ARM_FP) && \
        (__ARM_FP != 0) ||                                        \
    defined(_MSC_VER) && defined(_M_ARM)
  uint32_t fpscr;
#elif defined(__GNUC__) && defined(__aarch64__) || \
    defined(_MSC_VER) && (defined(_M_ARM64) || defined(_M_ARM64EC))
  uint64_t fpcr;
#elif defined(__SSE__) || defined(__x86_64__) || defined(_M_X64) || \
    (defined(_M_IX86_FP) && _M_IX86_FP >= 1)
  uint32_t mxcsr;
#else
  char unused;
#endif
};

static inline struct fpu_state get_fpu_state(void) {
  struct fpu_state state = {0};
#if defined(_MSC_VER) && defined(_M_ARM)
  state.fpscr = (uint32_t)_MoveFromCoprocessor(10, 7, 1, 0, 0);
#elif defined(_MSC_VER) && (defined(_M_ARM64) || defined(_M_ARM64EC))
  state.fpcr = (uint64_t)_ReadStatusReg(0x5A20);
#elif defined(__SSE__) || defined(__x86_64__) || defined(_M_X64) || \
    (defined(_M_IX86_FP) && _M_IX86_FP >= 1)
  state.mxcsr = (uint32_t)_mm_getcsr();
#elif defined(__GNUC__) && defined(__arm__) && defined(__ARM_FP) && \
    (__ARM_FP != 0)
  __asm__ __volatile__("VMRS %[fpscr], fpscr" : [fpscr] "=r"(state.fpscr));
#elif defined(__GNUC__) && defined(__aarch64__)
  __asm__ __volatile__("MRS %[fpcr], fpcr" : [fpcr] "=r"(state.fpcr));
#endif
  return state;
}

static inline void set_fpu_state(const struct fpu_state state) {
#if defined(_MSC_VER) && defined(_M_ARM)
  _MoveToCoprocessor((int)state.fpscr, 10, 7, 1, 0, 0);
#elif defined(_MSC_VER) && (defined(_M_ARM64) || defined(_M_ARM64EC))
  _WriteStatusReg(0x5A20, (__int64)state.fpcr);
#elif defined(__GNUC__) && defined(__arm__) && defined(__ARM_FP) && \
    (__ARM_FP != 0)
  __asm__ __volatile__("VMSR fpscr, %[fpscr]" : : [fpscr] "r"(state.fpscr));
#elif defined(__GNUC__) && defined(__aarch64__)
  __asm__ __volatile__("MSR fpcr, %[fpcr]" : : [fpcr] "r"(state.fpcr));
#elif defined(__SSE__) || defined(__x86_64__) || defined(_M_X64) || \
    (defined(_M_IX86_FP) && _M_IX86_FP >= 1)
  _mm_setcsr((unsigned int)state.mxcsr);
#endif
}

static inline void disable_fpu_denormals(void) {
#if defined(_MSC_VER) && defined(_M_ARM)
  int fpscr = _MoveFromCoprocessor(10, 7, 1, 0, 0);
  fpscr |= 0x1000000;
  _MoveToCoprocessor(fpscr, 10, 7, 1, 0, 0);
#elif defined(_MSC_VER) && (defined(_M_ARM64) || defined(_M_ARM64EC))
  __int64 fpcr = _ReadStatusReg(0x5A20);
  fpcr |= 0x1080000;
  _WriteStatusReg(0x5A20, fpcr);
#elif defined(__GNUC__) && defined(__arm__) && defined(__ARM_FP) && \
    (__ARM_FP != 0)
  uint32_t fpscr;
#if defined(__thumb__) && !defined(__thumb2__)
  __asm__ __volatile__(
      "VMRS %[fpscr], fpscr\n"
      "ORRS %[fpscr], %[bitmask]\n"
      "VMSR fpscr, %[fpscr]\n"
      : [fpscr] "=l"(fpscr)
      : [bitmask] "l"(0x1000000)
      : "cc");
#else
  __asm__ __volatile__(
      "VMRS %[fpscr], fpscr\n"
      "ORR %[fpscr], #0x1000000\n"
      "VMSR fpscr, %[fpscr]\n"
      : [fpscr] "=r"(fpscr));
#endif
#elif defined(__GNUC__) && defined(__aarch64__)
  uint64_t fpcr;
  __asm__ __volatile__(
      "MRS %[fpcr], fpcr\n"
      "ORR %w[fpcr], %w[fpcr], 0x1000000\n"
      "ORR %w[fpcr], %w[fpcr], 0x80000\n"
      "MSR fpcr, %[fpcr]\n"
      : [fpcr] "=r"(fpcr));
#elif defined(__SSE__) || defined(__x86_64__) || defined(_M_X64) || \
    (defined(_M_IX86_FP) && _M_IX86_FP >= 1)
  _mm_setcsr(_mm_getcsr() | 0x8040);
#endif
}

static inline size_t modulo_decrement(size_t i, size_t n) {
  /* Wrap modulo n, if needed */
  if (i == 0) {
    i = n;
  }
  /* Decrement input variable */
  return i - 1;
}

static inline size_t divide_round_up(size_t dividend, size_t divisor) {
  assert(divisor != 0);
  if (dividend % divisor == 0) {
    return dividend / divisor;
  } else {
    return dividend / divisor + 1;
  }
}

/* Windows headers define min and max macros; undefine it here */
#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif

static inline size_t min(size_t a, size_t b) { return a < b ? a : b; }
static inline size_t max(size_t a, size_t b) { return a > b ? a : b; }

struct pthreadpool_div_result {
  size_t quotient;
  size_t remainder;
};

static inline struct pthreadpool_div_result pthreadpool_div_size_t(size_t n,
                                                                   size_t d) {
  struct pthreadpool_div_result result;
  result.quotient = n / d;
  result.remainder = n % d;
  return result;
}

// --- Begin FXdiv-like implementation ---

static inline uint64_t pthreadpool_mulext_uint32_t(uint32_t a, uint32_t b) {
#if defined(_MSC_VER) && defined(_M_IX86)
  return (uint64_t) __emulu((unsigned int) a, (unsigned int) b);
#else
  return (uint64_t) a * (uint64_t) b;
#endif
}

static inline uint32_t pthreadpool_mulhi_uint32_t(uint32_t a, uint32_t b) {
#if defined(__OPENCL_VERSION__)
  return mul_hi(a, b);
#elif defined(__CUDA_ARCH__)
  return (uint32_t) __umulhi((unsigned int) a, (unsigned int) b);
#elif defined(_MSC_VER) && defined(_M_IX86)
  return (uint32_t) (__emulu((unsigned int) a, (unsigned int) b) >> 32);
#elif defined(_MSC_VER) && defined(_M_ARM)
  return (uint32_t) _MulUnsignedHigh((unsigned long) a, (unsigned long) b);
#else
  return (uint32_t) (((uint64_t) a * (uint64_t) b) >> 32);
#endif
}

static inline uint64_t pthreadpool_mulhi_uint64_t(uint64_t a, uint64_t b) {
#if defined(__OPENCL_VERSION__)
  return mul_hi(a, b);
#elif defined(__CUDA_ARCH__)
  return (uint64_t) __umul64hi((unsigned long long) a, (unsigned long long) b);
#elif defined(_MSC_VER) && defined(_M_X64)
  return (uint64_t) __umulh((unsigned __int64) a, (unsigned __int64) b);
#elif defined(__GNUC__) && defined(__SIZEOF_INT128__)
  return (uint64_t) (((((unsigned __int128) a) * ((unsigned __int128) b))) >> 64);
#else
  const uint32_t a_lo = (uint32_t) a;
  const uint32_t a_hi = (uint32_t) (a >> 32);
  const uint32_t b_lo = (uint32_t) b;
  const uint32_t b_hi = (uint32_t) (b >> 32);

  const uint64_t t = pthreadpool_mulext_uint32_t(a_hi, b_lo) +
    (uint64_t) pthreadpool_mulhi_uint32_t(a_lo, b_lo);
  return pthreadpool_mulext_uint32_t(a_hi, b_hi) + (t >> 32) +
    ((pthreadpool_mulext_uint32_t(a_lo, b_hi) + (uint64_t) (uint32_t) t) >> 32);
#endif
}

static inline size_t pthreadpool_mulhi_size_t(size_t a, size_t b) {
#if SIZE_MAX == UINT32_MAX
  return (size_t) pthreadpool_mulhi_uint32_t((uint32_t) a, (uint32_t) b);
#elif SIZE_MAX == UINT64_MAX
  return (size_t) pthreadpool_mulhi_uint64_t((uint64_t) a, (uint64_t) b);
#else
  #error Unsupported platform
#endif
}

struct pthreadpool_divisor_uint32_t {
  uint32_t value;
  uint32_t m;
  uint8_t s1;
  uint8_t s2;
};

struct pthreadpool_divisor_uint64_t {
  uint64_t value;
  uint64_t m;
  uint8_t s1;
  uint8_t s2;
};

struct pthreadpool_divisor_size_t {
  size_t value;
  size_t m;
  uint8_t s1;
  uint8_t s2;
};

static inline struct pthreadpool_divisor_uint32_t pthreadpool_init_uint32_t(uint32_t d) {
  struct pthreadpool_divisor_uint32_t result = { d };
  if (d == 1) {
    result.m = UINT32_C(1);
    result.s1 = 0;
    result.s2 = 0;
  } else {
    #if defined(__GNUC__)
      const uint32_t l_minus_1 = 31 - __builtin_clz(d - 1);
    #elif defined(_MSC_VER) && (defined(_M_IX86) || defined(_M_X64) || defined(_M_ARM) || defined(_M_ARM64))
      unsigned long l_minus_1;
      _BitScanReverse(&l_minus_1, (unsigned long) (d - 1));
    #else
      uint32_t l_minus_1 = 0;
      uint32_t x = d - 1;
      uint32_t y = x >> 16;
      if (y != 0) { l_minus_1 += 16; x = y; }
      y = x >> 8;
      if (y != 0) { l_minus_1 += 8; x = y; }
      y = x >> 4;
      if (y != 0) { l_minus_1 += 4; x = y; }
      y = x >> 2;
      if (y != 0) { l_minus_1 += 2; x = y; }
      if ((x & 2) != 0) { l_minus_1 += 1; }
    #endif
    uint32_t u_hi = (UINT32_C(2) << (uint32_t) l_minus_1) - d;
    const uint32_t q = ((uint64_t) u_hi << 32) / d;

    result.m = q + UINT32_C(1);
    result.s1 = 1;
    result.s2 = (uint8_t) l_minus_1;
  }
  return result;
}

static inline struct pthreadpool_divisor_uint64_t pthreadpool_init_uint64_t(uint64_t d) {
  struct pthreadpool_divisor_uint64_t result = { d };
  if (d == 1) {
    result.m = UINT64_C(1);
    result.s1 = 0;
    result.s2 = 0;
  } else {
    #if defined(__GNUC__)
      const uint32_t l_minus_1 = 63 - __builtin_clzll(d - 1);
      const uint32_t nlz_d = __builtin_clzll(d);
    #elif defined(_MSC_VER) && (defined(_M_X64) || defined(_M_ARM64))
      unsigned long l_minus_1;
      _BitScanReverse64(&l_minus_1, (unsigned __int64) (d - 1));
      unsigned long bsr_d;
      _BitScanReverse64(&bsr_d, (unsigned __int64) d);
      const uint32_t nlz_d = bsr_d ^ 0x3F;
    #else
      const uint64_t d_minus_1 = d - 1;
      const uint32_t d_is_power_of_2 = (d & d_minus_1) == 0;
      uint32_t l_minus_1 = 0;
      uint32_t x = (uint32_t) d_minus_1;
      uint32_t y = d_minus_1 >> 32;
      if (y != 0) { l_minus_1 += 32; x = y; }
      y = x >> 16;
      if (y != 0) { l_minus_1 += 16; x = y; }
      y = x >> 8;
      if (y != 0) { l_minus_1 += 8; x = y; }
      y = x >> 4;
      if (y != 0) { l_minus_1 += 4; x = y; }
      y = x >> 2;
      if (y != 0) { l_minus_1 += 2; x = y; }
      if ((x & 2) != 0) { l_minus_1 += 1; }
      const uint32_t nlz_d = (l_minus_1 ^ UINT32_C(0x3F)) - d_is_power_of_2;
    #endif
    uint64_t u_hi = (UINT64_C(2) << (uint32_t) l_minus_1) - d;

    #if defined(__GNUC__) && defined(__x86_64__)
      uint64_t q;
      __asm__("DIVQ %[d]"
        : "=a" (q), "+d" (u_hi)
        : [d] "r" (d), "a" (UINT64_C(0))
        : "cc");
    #elif (defined(_MSC_VER) && _MSC_VER >= 1920) && !defined(__clang__) && !defined(__INTEL_COMPILER) && defined(_M_X64)
      unsigned __int64 remainder;
      const uint64_t q = (uint64_t) _udiv128((unsigned __int64) u_hi, 0, (unsigned __int64) d, &remainder);
    #else
      /* Portable implementation */
      uint64_t d_shift = d << nlz_d;
      uint64_t u_hi_shift = u_hi << nlz_d;
      const uint64_t d_hi = (uint32_t) (d_shift >> 32);
      const uint32_t d_lo = (uint32_t) d_shift;

      uint64_t q1 = u_hi_shift / d_hi;
      uint64_t r1 = u_hi_shift - q1 * d_hi;
      while ((q1 >> 32) != 0 || pthreadpool_mulext_uint32_t((uint32_t) q1, d_lo) > (r1 << 32)) {
        q1 -= 1;
        r1 += d_hi;
        if ((r1 >> 32) != 0) { break; }
      }
      u_hi_shift = (u_hi_shift << 32) - q1 * d_shift;
      uint64_t q0 = u_hi_shift / d_hi;
      uint64_t r0 = u_hi_shift - q0 * d_hi;
      while ((q0 >> 32) != 0 || pthreadpool_mulext_uint32_t((uint32_t) q0, d_lo) > (r0 << 32)) {
        q0 -= 1;
        r0 += d_hi;
        if ((r0 >> 32) != 0) { break; }
      }
      const uint64_t q = (q1 << 32) | (uint32_t) q0;
    #endif
    result.m = q + UINT64_C(1);
    result.s1 = 1;
    result.s2 = (uint8_t) l_minus_1;
  }
  return result;
}

static inline struct pthreadpool_divisor_size_t pthreadpool_init_divisor(size_t d) {
#if SIZE_MAX == UINT32_MAX
  const struct pthreadpool_divisor_uint32_t uint_result = pthreadpool_init_uint32_t((uint32_t) d);
#elif SIZE_MAX == UINT64_MAX
  const struct pthreadpool_divisor_uint64_t uint_result = pthreadpool_init_uint64_t((uint64_t) d);
#else
  #error Unsupported platform
#endif
  struct pthreadpool_divisor_size_t size_result = {
    (size_t) uint_result.value,
    (size_t) uint_result.m,
    uint_result.s1,
    uint_result.s2
  };
  return size_result;
}

static inline uint32_t pthreadpool_quotient_uint32_t(uint32_t n, const struct pthreadpool_divisor_uint32_t divisor) {
  const uint32_t t = pthreadpool_mulhi_uint32_t(n, divisor.m);
  return (t + ((n - t) >> divisor.s1)) >> divisor.s2;
}

static inline uint64_t pthreadpool_quotient_uint64_t(uint64_t n, const struct pthreadpool_divisor_uint64_t divisor) {
  const uint64_t t = pthreadpool_mulhi_uint64_t(n, divisor.m);
  return (t + ((n - t) >> divisor.s1)) >> divisor.s2;
}

static inline size_t pthreadpool_quotient_size_t(size_t n, const struct pthreadpool_divisor_size_t divisor) {
#if SIZE_MAX == UINT32_MAX
  const struct pthreadpool_divisor_uint32_t uint32_divisor = {
    (uint32_t) divisor.value,
    (uint32_t) divisor.m,
    divisor.s1,
    divisor.s2
  };
  return pthreadpool_quotient_uint32_t((uint32_t) n, uint32_divisor);
#elif SIZE_MAX == UINT64_MAX
  const struct pthreadpool_divisor_uint64_t uint64_divisor = {
    (uint64_t) divisor.value,
    (uint64_t) divisor.m,
    divisor.s1,
    divisor.s2
  };
  return pthreadpool_quotient_uint64_t((uint64_t) n, uint64_divisor);
#else
  #error Unsupported platform
#endif
}

static inline struct pthreadpool_div_result pthreadpool_divide_with_divisor(size_t n, const struct pthreadpool_divisor_size_t divisor) {
  const size_t quotient = pthreadpool_quotient_size_t(n, divisor);
  const size_t remainder = n - quotient * divisor.value;
  struct pthreadpool_div_result result = { quotient, remainder };
  return result;
}

// --- End FXdiv-like implementation ---

#endif  // __PTHREADPOOL_SRC_THREADPOOL_UTILS_H_
