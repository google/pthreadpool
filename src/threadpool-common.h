// Copyright (c) 2017 Facebook Inc.
// Copyright (c) 2015-2017 Georgia Institute of Technology
// All rights reserved.
//
// Copyright 2019 Google LLC
//
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#ifndef __PTHREADPOOL_SRC_THREADPOOL_COMMON_H_
#define __PTHREADPOOL_SRC_THREADPOOL_COMMON_H_

#ifndef PTHREADPOOL_USE_CPUINFO
#define PTHREADPOOL_USE_CPUINFO 0
#endif

#ifndef PTHREADPOOL_USE_FUTEX
#if defined(__linux__)
#define PTHREADPOOL_USE_FUTEX 1
#elif defined(__EMSCRIPTEN__)
#define PTHREADPOOL_USE_FUTEX 1
#else
#define PTHREADPOOL_USE_FUTEX 0
#endif
#endif

#ifndef PTHREADPOOL_USE_GCD
#if defined(__APPLE__)
#define PTHREADPOOL_USE_GCD 1
#else
#define PTHREADPOOL_USE_GCD 0
#endif
#endif

#ifndef PTHREADPOOL_USE_EVENT
#if defined(_WIN32) || defined(__CYGWIN__)
#define PTHREADPOOL_USE_EVENT 1
#else
#define PTHREADPOOL_USE_EVENT 0
#endif
#endif

#ifndef PTHREADPOOL_USE_CONDVAR
#if PTHREADPOOL_USE_GCD || PTHREADPOOL_USE_FUTEX || PTHREADPOOL_USE_EVENT
#define PTHREADPOOL_USE_CONDVAR 0
#else
#define PTHREADPOOL_USE_CONDVAR 1
#endif
#endif

/* Number of iterations in spin-wait loop before going into futex/condvar wait
 */
#if defined(__ANDROID__)
/* We really don't want the process to sleep on Android, so spin for much longer
 * than we otherwise would. */
#define PTHREADPOOL_SPIN_YIELD_ITERATIONS 10
#define PTHREADPOOL_SPIN_PAUSE_ITERATIONS 100000
#else

#define PTHREADPOOL_SPIN_YIELD_ITERATIONS 0
#define PTHREADPOOL_SPIN_PAUSE_ITERATIONS 1000
#endif  // defined(__ANDROID__)
#define PTHREADPOOL_SPIN_WAIT_ITERATIONS \
  (PTHREADPOOL_SPIN_PAUSE_ITERATIONS + PTHREADPOOL_SPIN_YIELD_ITERATIONS)

#define PTHREADPOOL_CACHELINE_SIZE 64
#if defined(__GNUC__)
#define PTHREADPOOL_CACHELINE_ALIGNED \
  __attribute__((__aligned__(PTHREADPOOL_CACHELINE_SIZE)))
#elif defined(_MSC_VER)
#define PTHREADPOOL_CACHELINE_ALIGNED \
  __declspec(align(PTHREADPOOL_CACHELINE_SIZE))
#else
#error \
    "Platform-specific implementation of PTHREADPOOL_CACHELINE_ALIGNED required"
#endif

#if defined(__clang__)
#if __has_extension(c_static_assert) || __has_feature(c_static_assert)
#define PTHREADPOOL_STATIC_ASSERT(predicate, message) \
  _Static_assert((predicate), message)
#else
#define PTHREADPOOL_STATIC_ASSERT(predicate, message)
#endif
#elif defined(__GNUC__) && \
    ((__GNUC__ > 4) || (__GNUC__ == 4) && (__GNUC_MINOR__ >= 6))
/* Static assert is supported by gcc >= 4.6 */
#define PTHREADPOOL_STATIC_ASSERT(predicate, message) \
  _Static_assert((predicate), message)
#else
#define PTHREADPOOL_STATIC_ASSERT(predicate, message)
#endif

// We declare these symbols as having weak linkage, so they can be replaced by
// a custom implementation.
#if defined(__GNUC__)
#define PTHREADPOOL_WEAK __attribute__((__weak__))
#else
#define PTHREADPOOL_WEAK
#endif

#if defined(__GNUC__) && defined(__linux__)
#define PTHREADPOOL_PRIVATE_IMPL(name) \
  extern __typeof(name) name##_private_impl __attribute__((alias(#name)));
#else
#define PTHREADPOOL_PRIVATE_IMPL(name)
#endif

#ifndef PTHREADPOOL_INTERNAL
#if defined(__ELF__)
#define PTHREADPOOL_INTERNAL __attribute__((__visibility__("internal")))
#elif defined(__MACH__)
#define PTHREADPOOL_INTERNAL __attribute__((__visibility__("hidden")))
#else
#define PTHREADPOOL_INTERNAL
#endif
#endif

// ported from:
// https://stackoverflow.com/questions/18298280/how-to-declare-a-variable-as-thread-local-portably
/* gcc doesn't know _Thread_local from C11 yet */
#ifdef __GNUC__
#define thread_local __thread
/*
// c11 standard already has thread_local specified
// https://en.cppreference.com/w/c/thread/thread_local
#elif __STDC_VERSION__ >= 201112L
# define thread_local _Thread_local
*/
#elif defined(_MSC_VER)
#define thread_local __declspec(thread)
#else
#error Cannot define thread_local
#endif

#endif  // __PTHREADPOOL_SRC_THREADPOOL_COMMON_H_
