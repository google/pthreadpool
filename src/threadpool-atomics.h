// Copyright (c) 2017 Facebook Inc.
// Copyright (c) 2015-2017 Georgia Institute of Technology
// All rights reserved.
//
// Copyright 2019 Google LLC
//
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef __PTHREADPOOL_SRC_THREADPOOL_ATOMICS_H_
#define __PTHREADPOOL_SRC_THREADPOOL_ATOMICS_H_

/* Standard C headers */
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Windows-specific headers */
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <sched.h>
#endif

/* SSE-specific headers */
#if defined(__i386__) || defined(__i686__) || defined(__x86_64__) || \
    defined(_M_IX86) || defined(_M_X64) && !defined(_M_ARM64EC)
#include <xmmintrin.h>
#endif

/* ARM-specific headers */
#if defined(__ARM_ACLE)
#include <arm_acle.h>
#endif

/* MSVC-specific headers */
#ifdef _MSC_VER
#include <intrin.h>
#endif

/* Configuration header */
#include "threadpool-common.h"

#ifdef __has_feature
#define PTHREADPOOL_HAS_FEATURE(f) __has_feature(f)
#else
#define PTHREADPOOL_HAS_FEATURE(f) 0
#endif  // __has_feature
#if defined(THREAD_SANITIZER) || defined(__SANITIZE_THREAD__) || \
    PTHREADPOOL_HAS_FEATURE(thread_sanitizer)
#define PTHREADPOOL_TSAN 1
#include <sanitizer/tsan_interface.h>
#else
#define PTHREADPOOL_TSAN 0
#endif

/* Align the atomic values on the size of a cache line to avoid false sharing,
 * i.e. two or more atomic variables sharing the same cache line will block
 * each other during atomic operations.
 */
typedef atomic_uint_fast32_t PTHREADPOOL_CACHELINE_ALIGNED
    pthreadpool_atomic_uint32_t;
typedef atomic_size_t PTHREADPOOL_CACHELINE_ALIGNED pthreadpool_atomic_size_t;
typedef atomic_uintptr_t PTHREADPOOL_CACHELINE_ALIGNED
    pthreadpool_atomic_void_p;
typedef atomic_uint_fast32_t PTHREADPOOL_CACHELINE_ALIGNED
    pthreadpool_spin_lock_t;

static inline uint32_t pthreadpool_load_relaxed_uint32_t(
    pthreadpool_atomic_uint32_t* address) {
  return atomic_load_explicit(address, memory_order_relaxed);
}

static inline size_t pthreadpool_load_relaxed_size_t(
    pthreadpool_atomic_size_t* address) {
  return atomic_load_explicit(address, memory_order_relaxed);
}

static inline void* pthreadpool_load_relaxed_void_p(
    pthreadpool_atomic_void_p* address) {
  return (void*)atomic_load_explicit(address, memory_order_relaxed);
}

static inline uint32_t pthreadpool_load_acquire_uint32_t(
    pthreadpool_atomic_uint32_t* address) {
  return atomic_load_explicit(address, memory_order_acquire);
}

static inline size_t pthreadpool_load_acquire_size_t(
    pthreadpool_atomic_size_t* address) {
  return atomic_load_explicit(address, memory_order_acquire);
}

static inline void pthreadpool_store_relaxed_uint32_t(
    pthreadpool_atomic_uint32_t* address, uint32_t value) {
  atomic_store_explicit(address, value, memory_order_relaxed);
}

static inline void pthreadpool_store_relaxed_size_t(
    pthreadpool_atomic_size_t* address, size_t value) {
  atomic_store_explicit(address, value, memory_order_relaxed);
}

static inline void pthreadpool_store_relaxed_void_p(
    pthreadpool_atomic_void_p* address, void* value) {
  atomic_store_explicit(address, (uintptr_t)value, memory_order_relaxed);
}

static inline void pthreadpool_store_release_uint32_t(
    pthreadpool_atomic_uint32_t* address, uint32_t value) {
  atomic_store_explicit(address, value, memory_order_release);
}

static inline void pthreadpool_store_release_size_t(
    pthreadpool_atomic_size_t* address, size_t value) {
  atomic_store_explicit(address, value, memory_order_release);
}

static inline size_t pthreadpool_decrement_fetch_relaxed_size_t(
    pthreadpool_atomic_size_t* address) {
  return atomic_fetch_sub_explicit(address, 1, memory_order_relaxed) - 1;
}

static inline uint32_t pthreadpool_decrement_fetch_relaxed_uint32_t(
    pthreadpool_atomic_uint32_t* address) {
  return atomic_fetch_sub_explicit(address, 1, memory_order_relaxed) - 1;
}

static inline size_t pthreadpool_decrement_n_fetch_relaxed_size_t(
    pthreadpool_atomic_size_t* address, size_t n) {
  return atomic_fetch_sub_explicit(address, n, memory_order_relaxed) - n;
}

static inline size_t pthreadpool_fetch_decrement_n_relaxed_size_t(
    pthreadpool_atomic_size_t* address, size_t n) {
  return atomic_fetch_sub_explicit(address, n, memory_order_relaxed);
}

static inline size_t pthreadpool_decrement_fetch_release_size_t(
    pthreadpool_atomic_size_t* address) {
  return atomic_fetch_sub_explicit(address, 1, memory_order_release) - 1;
}

static inline size_t pthreadpool_decrement_fetch_acquire_release_size_t(
    pthreadpool_atomic_size_t* address) {
  return atomic_fetch_sub_explicit(address, 1, memory_order_acq_rel) - 1;
}

static inline bool pthreadpool_try_decrement_relaxed_size_t(
    pthreadpool_atomic_size_t* value) {
  size_t actual_value = atomic_load_explicit(value, memory_order_acquire);
  while (actual_value != 0) {
    if (atomic_compare_exchange_weak_explicit(
            value, &actual_value, actual_value - 1, memory_order_relaxed,
            memory_order_relaxed)) {
      return true;
    }
  }
  return false;
}

static inline size_t pthreadpool_fetch_add_relaxed_size_t(
    pthreadpool_atomic_size_t* address, size_t value) {
  return atomic_fetch_add_explicit(address, value, memory_order_relaxed);
}

static inline size_t pthreadpool_fetch_add_relaxed_uint32_t(
    pthreadpool_atomic_uint32_t* address, uint32_t value) {
  return atomic_fetch_add_explicit(address, value, memory_order_relaxed);
}

static inline uint32_t pthreadpool_exchange_relaxed_uint32_t(
    pthreadpool_atomic_uint32_t* address, uint32_t value) {
  return atomic_exchange_explicit(address, value, memory_order_relaxed);
}

static inline bool pthreadpool_compare_exchange_relaxed_size_t(
    pthreadpool_atomic_size_t* address, size_t* expected_value,
    size_t new_value) {
  return atomic_compare_exchange_weak_explicit(address, expected_value,
                                               new_value, memory_order_relaxed,
                                               memory_order_relaxed);
}

static inline bool pthreadpool_compare_exchange_relaxed_uint32_t(
  pthreadpool_atomic_uint32_t* address, uint_fast32_t* expected_value,
  uint32_t new_value) {
return atomic_compare_exchange_weak_explicit(address, expected_value,
                                             new_value, memory_order_relaxed,
                                             memory_order_relaxed);
}

static inline void pthreadpool_fence_acquire() {
  atomic_thread_fence(memory_order_acquire);
}

static inline void pthreadpool_fence_release() {
  atomic_thread_fence(memory_order_release);
}

static inline void pthreadpool_yield(uint32_t step) {
  if (step < PTHREADPOOL_SPIN_PAUSE_ITERATIONS) {
#if defined(__ARM_ACLE) || \
    defined(_MSC_VER) &&   \
        (defined(_M_ARM) || defined(_M_ARM64) || defined(_M_ARM64EC))
    __yield();
#elif defined(__GNUC__) &&                                      \
    (defined(__ARM_ARCH) && (__ARM_ARCH >= 7) ||                \
     (defined(__ARM_ARCH_6K__) || defined(__ARM_ARCH_6KZ__)) && \
         !defined(__thumb__))
    __asm__ __volatile__("yield");
#elif defined(__i386__) || defined(__i686__) || defined(__x86_64__) || \
    defined(_M_IX86) || defined(_M_X64)
    _mm_pause();
#else
    pthreadpool_fence_acquire();
#endif
  } else {
#ifdef _WIN32
    Sleep(0);
#else
    sched_yield();
#endif
  }
}

static inline void pthreadpool_spin_lock_init(
    pthreadpool_spin_lock_t* spin_lock) {
  atomic_store_explicit(spin_lock, 0, memory_order_release);
#if PTHREADPOOL_TSAN
  __tsan_mutex_create(spin_lock, __tsan_mutex_not_static);
#endif
}

static inline void pthreadpool_spin_lock_destroy(
    pthreadpool_spin_lock_t* spin_lock) {
#if PTHREADPOOL_TSAN
  __tsan_mutex_destroy(spin_lock, __tsan_mutex_not_static);
#endif
}

static inline void pthreadpool_spin_lock_acquire(
    pthreadpool_spin_lock_t* spin_lock) {
  uint32_t iters = 0;
#if PTHREADPOOL_TSAN
  __tsan_mutex_pre_lock(spin_lock, /*flags=*/0);
#endif
  while (atomic_exchange_explicit(spin_lock, 1, memory_order_seq_cst) != 0) {
    while (atomic_load_explicit(spin_lock, memory_order_relaxed) != 0) {
      pthreadpool_yield(0);
    }
  }
  pthreadpool_fence_acquire();
#if PTHREADPOOL_TSAN
  __tsan_mutex_post_lock(spin_lock, /*flags=*/0, /*recursion=*/0);
#endif
}

static inline void pthreadpool_spin_lock_release(
    pthreadpool_spin_lock_t* spin_lock) {
#if PTHREADPOOL_TSAN
  __tsan_mutex_pre_unlock(spin_lock, /*flags=*/0);
#endif
  atomic_store_explicit(spin_lock, 0, memory_order_release);
  pthreadpool_fence_release();
#if PTHREADPOOL_TSAN
  __tsan_mutex_post_unlock(spin_lock, /*flags=*/0);
#endif
}

#endif  // __PTHREADPOOL_SRC_THREADPOOL_ATOMICS_H_
