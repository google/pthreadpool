// Copyright (c) 2017 Facebook Inc.
// Copyright (c) 2015-2017 Georgia Institute of Technology
// All rights reserved.
//
// Copyright 2019 Google LLC
//
// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

// Needed for syscall.

#ifndef _GNU_SOURCE
#define _GNU_SOURCE 1
#endif  // _GNU_SOURCE

/* Standard C headers */
#include <assert.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* Configuration header */
#include <fxdiv.h>
#include "threadpool-common.h"

/* POSIX headers */
#include <pthread.h>
#include <unistd.h>

/* Futex-specific headers */
#if PTHREADPOOL_USE_FUTEX
#if defined(__linux__)
#include <linux/futex.h>
#include <sys/syscall.h>

/* Logging-related headers */
#ifndef PTHREADPOOL_DEBUG_LOGGING
#define PTHREADPOOL_DEBUG_LOGGING 0
#endif  // PTHREADPOOL_DEBUG_LOGGING
#if PTHREADPOOL_DEBUG_LOGGING
#include <stdio.h>
#include <time.h>
#define pthreadpool_log_debug(format, ...)                                 \
  fprintf(stderr, "[%zu] %s (%s:%i): " format "\n", clock(), __FUNCTION__, \
          __FILE__, __LINE__, ##__VA_ARGS__);
#else
#define pthreadpool_log_debug(format, ...)
#endif  // PTHREADPOOL_DEBUG_LOGGING

/* Old Android NDKs do not define SYS_futex and FUTEX_PRIVATE_FLAG */
#ifndef SYS_futex
#define SYS_futex __NR_futex
#endif  // SYS_futex

#ifndef FUTEX_PRIVATE_FLAG
#define FUTEX_PRIVATE_FLAG 128
#endif  // FUTEX_PRIVATE_FLAG

#elif defined(__EMSCRIPTEN__)
/* math.h for INFINITY constant */
#include <emscripten/threading.h>
#include <math.h>

#else
#error \
    "Platform-specific implementation of futex_wait and futex_wake_all required"
#endif  // defined(__linux__)
#endif  // PTHREADPOOL_USE_FUTEX

/* Windows-specific headers */
#ifdef _WIN32
#include <sysinfoapi.h>
#endif

/* Dependencies */
#if PTHREADPOOL_USE_CPUINFO
#include <cpuinfo.h>
#endif

/* Public library header */
#include <pthreadpool.h>

/* Internal library headers */
#include "threadpool-atomics.h"
#include "threadpool-object.h"
#include "threadpool-utils.h"

#if PTHREADPOOL_USE_FUTEX
#if defined(__linux__)
static int futex_wait(pthreadpool_atomic_uint32_t* address, uint32_t value) {
  return syscall(SYS_futex, address, FUTEX_WAIT | FUTEX_PRIVATE_FLAG, value,
                 NULL);
}

static int futex_wake_all(pthreadpool_atomic_uint32_t* address) {
  return syscall(SYS_futex, address, FUTEX_WAKE | FUTEX_PRIVATE_FLAG, INT_MAX);
}
#elif defined(__EMSCRIPTEN__)
static int futex_wait(pthreadpool_atomic_uint32_t* address, uint32_t value) {
  return emscripten_futex_wait((volatile void*)address, value, INFINITY);
}

static int futex_wake_all(pthreadpool_atomic_uint32_t* address) {
  return emscripten_futex_wake((volatile void*)address, INT_MAX);
}
#else
#error \
    "Platform-specific implementation of futex_wait and futex_wake_all required"
#endif
#endif

size_t pthreadpool_set_threads_count(struct pthreadpool* threadpool,
                                     size_t num_threads) {
  if (threadpool == NULL) {
    return 1;
  } else if (!threadpool->executor) {
    return threadpool->threads_count.value;
  }

  /* We shouldn't change this while a parallel computation is running. */
  pthread_mutex_lock(&threadpool->execution_mutex);

  // Adjust `num_threads` to the feasible limits.
  num_threads = max(num_threads, 1);
  num_threads = min(num_threads, threadpool->num_thread_info);
  threadpool->max_active_threads = num_threads;

  pthread_mutex_unlock(&threadpool->execution_mutex);

  return num_threads;
}

static void run_thread_function(struct pthreadpool* threadpool,
                                uint32_t thread_id,
                                thread_function_t thread_function,
                                uint32_t flags) {
  pthreadpool_log_debug("thread %u working on job %lu.", thread_id,
                        threadpool->job_id);
  // Save the current FPU state, if requested.
  struct fpu_state saved_fpu_state = {0};
  if (flags & PTHREADPOOL_FLAG_DISABLE_DENORMALS) {
    saved_fpu_state = get_fpu_state();
    disable_fpu_denormals();
  }

  // Call the job function.
  thread_function(threadpool, &threadpool->threads[thread_id]);

  // Restore the original FPU state in case we clobbered it.
  if (flags & PTHREADPOOL_FLAG_DISABLE_DENORMALS) {
    set_fpu_state(saved_fpu_state);
  }

  pthreadpool_log_debug("thread %u done working on job %lu.", thread_id,
                        threadpool->job_id);

  // If we were the last thread on this job, switch the state back to "idle"
  // and signal anybody waiting on that state change.
  pthreadpool_spin_lock_acquire(&threadpool->state_spin_lock);
  if (pthreadpool_decrement_fetch_relaxed_uint32_t(
          &threadpool->num_active_threads) == 0) {
    pthreadpool_log_debug("thread %u switching state from %u to %u.", thread_id,
                          (uint32_t)threadpool->state, threadpool_state_idle);
    pthreadpool_store_release_uint32_t(&threadpool->state,
                                       threadpool_state_idle);
#if PTHREADPOOL_USE_FUTEX
    futex_wake_all((pthreadpool_atomic_uint32_t*)&threadpool->state);
#else
    pthread_mutex_lock(&threadpool->state_mutex);
    pthread_cond_signal(&threadpool->state_condvar);
    pthread_mutex_unlock(&threadpool->state_mutex);
#endif  // PTHREADPOOL_USE_FUTEX
  } else if (threadpool->state == threadpool_state_running) {
    // Otherwise, switch the state from "running" to "wrapping up" since
    // there is no work left to steal.
    pthreadpool_log_debug("thread %u switching state from %u to %u.", thread_id,
                          (uint32_t)threadpool->state,
                          threadpool_state_wrapping_up);
    pthreadpool_store_release_uint32_t(&threadpool->state,
                                       threadpool_state_wrapping_up);
#if PTHREADPOOL_USE_FUTEX
    futex_wake_all((pthreadpool_atomic_uint32_t*)&threadpool->state);
#else
    pthread_mutex_lock(&threadpool->state_mutex);
    pthread_cond_signal(&threadpool->state_condvar);
    pthread_mutex_unlock(&threadpool->state_mutex);
#endif  // PTHREADPOOL_USE_FUTEX
  }
  pthreadpool_spin_lock_release(&threadpool->state_spin_lock);
}

static void* thread_run(void* arg) {
  // Unpack the argument, i.e. extract the pointer to the `pthreadpool` from the
  // provided pointer to this thread's `thread_info`.
  struct thread_info* thread = (struct thread_info*)arg;
  const uint32_t thread_id = thread->thread_number;
  struct pthreadpool* threadpool =
      (struct pthreadpool*)((uintptr_t)thread -
                            thread_id * sizeof(struct thread_info) -
                            offsetof(struct pthreadpool, threads));

  // Get the current threadpool state.
  uint32_t curr_state = pthreadpool_load_acquire_uint32_t(&threadpool->state);

  // Main loop.
  while (true) {
    // Get a hold on the threadpool state lock.
    pthreadpool_spin_lock_acquire(&threadpool->state_spin_lock);

    // Get the current threadpool state.
    curr_state = pthreadpool_load_acquire_uint32_t(&threadpool->state);

    // If the state is `idle` or `wrapping_up`, wait for a state change and
    // start over.
    if (curr_state == threadpool_state_idle ||
        curr_state == threadpool_state_wrapping_up) {
      pthreadpool_spin_lock_release(&threadpool->state_spin_lock);
      pthreadpool_log_debug("thread %u waiting on running (curr_state: %u).",
                            thread_id, curr_state);
#if PTHREADPOOL_USE_FUTEX
      for (uint32_t iter = 0; curr_state == threadpool_state_idle ||
                              curr_state == threadpool_state_wrapping_up;
           iter++) {
        if (iter < PTHREADPOOL_SPIN_WAIT_ITERATIONS) {
          pthreadpool_yield(iter);
        } else {
          futex_wait((pthreadpool_atomic_uint32_t*)&threadpool->state,
                     curr_state);
          iter = 0;
        }
        curr_state = pthreadpool_load_acquire_uint32_t(&threadpool->state);
      }
#else
      for (uint32_t iter = 0; curr_state == threadpool_state_idle ||
                              curr_state == threadpool_state_wrapping_up &&
                                  iter < PTHREADPOOL_SPIN_WAIT_ITERATIONS;
           iter++) {
        pthreadpool_yield(iter);
        curr_state = pthreadpool_load_acquire_uint32_t(&threadpool->state);
      }
      if (curr_state == threadpool_state_idle ||
          curr_state == threadpool_state_wrapping_up) {
        pthread_mutex_lock(&threadpool->state_mutex);
        while (curr_state == threadpool_state_idle ||
               curr_state == threadpool_state_wrapping_up) {
          pthread_cond_wait(&threadpool->state_condvar,
                            &threadpool->state_mutex);
          curr_state = pthreadpool_load_acquire_uint32_t(&threadpool->state);
        }
        pthread_mutex_unlock(&threadpool->state_mutex);
      }
#endif  // PTHREADPOOL_USE_FUTEX

      // Go back to the top of the loop with the new state.
      continue;
    }

    // If we're the last pending thread of a "done" threadpool, signal for any
    // waiting cleanup and bail before anything else can go wrong (e.g. the
    // `threadpool` might get cleaned up).
    if (curr_state == threadpool_state_done) {
      const uint32_t num_recruited_threads =
          pthreadpool_decrement_fetch_relaxed_uint32_t(
              &threadpool->num_recruited_threads);
      pthreadpool_spin_lock_release(&threadpool->state_spin_lock);
      if (num_recruited_threads == 0) {
#if PTHREADPOOL_USE_FUTEX
        futex_wake_all(&threadpool->num_recruited_threads);
#else
        pthread_mutex_lock(&threadpool->completion_mutex);
        pthread_cond_signal(&threadpool->completion_condvar);
        pthread_mutex_unlock(&threadpool->completion_mutex);
#endif  // PTHREADPOOL_USE_FUTEX
      }
      break;
    }

    // Get some useful variables from the threadpool.
    thread_function_t thread_function = NULL;
    uint32_t flags;

    // If the threadpool is currently running a job...
    if (curr_state == threadpool_state_running) {
      // Try to join in on the current job, unless there are too many threads
      // already on it.
      uint_fast32_t curr_num_threads =
          pthreadpool_load_acquire_uint32_t(&threadpool->num_active_threads);
      const uint32_t max_active_threads = threadpool->max_active_threads;
      while (curr_num_threads < max_active_threads &&
             !pthreadpool_compare_exchange_relaxed_uint32_t(
                 &threadpool->num_active_threads, &curr_num_threads,
                 curr_num_threads + 1)) {
      }

      // Only run the job if there are not already too many threads.
      if (curr_num_threads < max_active_threads) {
        // Get some useful variables from the threadpool.
        thread_function = (thread_function_t)pthreadpool_load_relaxed_void_p(
            &threadpool->thread_function);
        flags = pthreadpool_load_relaxed_uint32_t(&threadpool->flags);
      }

      // If we were the last thread to join this job, then we need to change the
      // state from "running" to "wrapping up".
      if (curr_num_threads + 1 == max_active_threads) {
        pthreadpool_log_debug("thread %u switching state from %u to %u.",
                              thread_id, curr_state,
                              threadpool_state_wrapping_up);
        pthreadpool_store_release_uint32_t(&threadpool->state,
                                           threadpool_state_wrapping_up);
      }
    }

    // Release the threadpool state lock.
    pthreadpool_spin_lock_release(&threadpool->state_spin_lock);

    // If we have a job to do, just do it.
    if (thread_function) {
      run_thread_function(threadpool, thread_id, thread_function, flags);
    }
  }

  pthreadpool_log_debug("thread %u leaving main loop.", thread_id);

  return NULL;
}

static size_t get_num_cpus() {
#if PTHREADPOOL_USE_CPUINFO
  return cpuinfo_get_processors_count();
#elif defined(_SC_NPROCESSORS_ONLN)
  size_t num_cpus = (size_t)sysconf(_SC_NPROCESSORS_ONLN);
#if defined(__EMSCRIPTEN_PTHREADS__)
  /* Limit the number of threads to 8 to match link-time PTHREAD_POOL_SIZE
   * option */
  if (num_cpus >= 8) {
    num_cpus = 8;
  }
#endif
  return num_cpus;
#elif defined(_WIN32)
  SYSTEM_INFO system_info;
  ZeroMemory(&system_info, sizeof(system_info));
  GetSystemInfo(&system_info);
  return = (size_t)system_info.dwNumberOfProcessors;
#else
#error \
    "Platform-specific implementation of sysconf(_SC_NPROCESSORS_ONLN) required"
#endif
}

PTHREADPOOL_WEAK struct pthreadpool* pthreadpool_create(size_t threads_count) {
  return pthreadpool_create_v2(NULL, threads_count);
}

PTHREADPOOL_PRIVATE_IMPL(pthreadpool_create)

struct pthreadpool* pthreadpool_create_v2(struct pthreadpool_executor* executor,
                                          size_t max_num_threads) {
#if PTHREADPOOL_USE_CPUINFO
  if (!cpuinfo_initialize()) {
    return NULL;
  }
#endif

  if (max_num_threads == 0) {
    max_num_threads = get_num_cpus();
  }

  const uint32_t num_threads =
      executor ? min(max_num_threads, executor->num_threads(executor) + 1)
               : max_num_threads;
  struct pthreadpool* threadpool = pthreadpool_allocate(num_threads);
  if (threadpool == NULL) {
    return NULL;
  }
  threadpool->executor = executor;
  threadpool->max_num_threads = num_threads;
  threadpool->max_active_threads = num_threads;
  threadpool->threads_count = fxdiv_init_size_t(num_threads);
  for (size_t tid = 0; tid < num_threads; tid++) {
    threadpool->threads[tid].thread_number = tid;
    threadpool->threads[tid].threadpool = threadpool;
  }
  threadpool->num_active_threads = 0;

  if (num_threads > 1) {
    /* Initialize the condition variables and mutexes. */
    pthread_mutex_init(&threadpool->execution_mutex, NULL);
#if !PTHREADPOOL_USE_FUTEX
    pthread_mutex_init(&threadpool->completion_mutex, NULL);
    pthread_cond_init(&threadpool->completion_condvar, NULL);
    pthread_mutex_init(&threadpool->state_mutex, NULL);
    pthread_cond_init(&threadpool->state_condvar, NULL);
#endif

    /* If we weren't given an executor, start our own threads. */
    if (!executor) {
      /* Caller thread serves as worker #0. Thus, we create system threads
       * starting with worker #1. */
      threadpool->num_recruited_threads = num_threads - 1;
      for (size_t tid = 1; tid < num_threads; tid++) {
        pthread_create(&threadpool->threads[tid].thread_object, NULL,
                       &thread_run, &threadpool->threads[tid]);
      }
    }
  }

  return threadpool;
}

static void ensure_num_threads(struct pthreadpool* threadpool,
                               uint32_t num_threads) {
  assert(num_threads < threadpool->max_num_threads);
  struct pthreadpool_executor* executor = threadpool->executor;

  /* If we're not using an executor, do nothing. */
  if (!executor) {
    return;
  }

  /* Create the threads for this threadpool. */
  for (uint32_t tid = threadpool->num_recruited_threads; tid < num_threads;
       tid++) {
    pthreadpool_fetch_add_relaxed_size_t(&threadpool->num_recruited_threads, 1);
    pthreadpool_log_debug("starting thread %u (arg=%p).", tid + 1,
                          &threadpool->threads[tid + 1]);

    /* Fly, my pretties! Fly, fly, fly! */
    executor->schedule(executor, &threadpool->threads[tid + 1],
                       (void (*)(void*))thread_run);
  }
}

PTHREADPOOL_INTERNAL void pthreadpool_parallelize(
    struct pthreadpool* threadpool, thread_function_t thread_function,
    const void* params, size_t params_size, void* task, void* context,
    size_t linear_range, uint32_t flags) {
  assert(threadpool != NULL);
  assert(thread_function != NULL);
  assert(task != NULL);
  assert(linear_range > 1);
  static struct pthreadpool* threadpool_last;
  threadpool_last = threadpool;

  // For now just fall back to the old implementation with the threadpool's own
  // threads.
  // if (!threadpool->executor) {
  //   pthreadpool_parallelize_old(threadpool, thread_function, params,
  //                               params_size, task, context, linear_range,
  //                               flags);
  //   return;
  // }

  /* Protect the global threadpool structures */
  pthread_mutex_lock(&threadpool->execution_mutex);

  // Get a hold on the threadpool state lock.
  pthreadpool_spin_lock_acquire(&threadpool->state_spin_lock);

  /* Make sure the threadpool is idle. */
  assert(threadpool->state == threadpool_state_idle);
  assert(threadpool->num_active_threads == 0);

  /* Increment the number of active threads for this thread. */
  threadpool->num_active_threads = 1;

  /* Setup global arguments */
  pthreadpool_store_relaxed_void_p(&threadpool->thread_function,
                                   (void*)thread_function);
  pthreadpool_store_relaxed_void_p(&threadpool->task, task);
  pthreadpool_store_relaxed_void_p(&threadpool->argument, context);
  pthreadpool_store_relaxed_uint32_t(&threadpool->flags, flags);
  if (params_size != 0) {
    memcpy(&threadpool->params, params, params_size);
  }
  threadpool->job_id += 1;

  // How many threads should we parallelize over?
  const uint32_t num_threads = threadpool->max_active_threads;
  threadpool->threads_count = fxdiv_init_size_t(num_threads);

  /* Make sure we have enough threads (minus the calling thread) running. */
  ensure_num_threads(threadpool, num_threads - 1);

  pthreadpool_log_debug("main thread starting job %u with %u threads.",
                        (uint32_t)threadpool->job_id, num_threads);

  /* Populate a `thread_info` struct for each thread */
  const struct fxdiv_result_size_t range_params =
      fxdiv_divide_size_t(linear_range, threadpool->threads_count);
  size_t range_start = 0;
  for (size_t tid = 0; tid < num_threads; tid++) {
    struct thread_info* thread = &threadpool->threads[tid];
    const size_t range_length =
        range_params.quotient + (size_t)(tid < range_params.remainder);
    const size_t range_end = range_start + range_length;
    pthreadpool_store_relaxed_size_t(&thread->range_start, range_start);
    pthreadpool_store_relaxed_size_t(&thread->range_end, range_end);
    pthreadpool_store_relaxed_size_t(&thread->range_length, range_length);

    /* The next subrange starts where the previous ended */
    range_start = range_end;
  }

  /* Switch the threadpool state to "running". */
  assert(threadpool->state == threadpool_state_idle);
  pthreadpool_store_release_uint32_t(&threadpool->state,
                                     threadpool_state_running);

  // Release the threadpool state lock.
  pthreadpool_spin_lock_release(&threadpool->state_spin_lock);

  /* Wake up any thread waiting on a change of state. */
#if PTHREADPOOL_USE_FUTEX
  futex_wake_all((pthreadpool_atomic_uint32_t*)&threadpool->state);
#else
  pthread_mutex_lock(&threadpool->state_mutex);
  pthread_cond_signal(&threadpool->state_condvar);
  pthread_mutex_unlock(&threadpool->state_mutex);
#endif  // PTHREADPOOL_USE_FUTEX

  /* Do a bit of work ourselves, as thread zero. */
  run_thread_function(threadpool, /*thread_id=*/0, thread_function, flags);

  /* Since we finished our part of the job, we know the state should be either
   * "wrapping up" or "idle", i.e. definitely no longer "running". */
  uint32_t state = pthreadpool_load_acquire_uint32_t(&threadpool->state);
  assert(state == threadpool_state_idle ||
         state == threadpool_state_wrapping_up);
#if PTHREADPOOL_USE_FUTEX
  for (uint32_t iter = 0; state != threadpool_state_idle; iter++) {
    if (iter < PTHREADPOOL_SPIN_WAIT_ITERATIONS) {
      pthreadpool_yield(iter);
    } else {
      futex_wait((pthreadpool_atomic_uint32_t*)&threadpool->state, state);
    }
    state = pthreadpool_load_acquire_uint32_t(&threadpool->state);
  }
#else
  for (uint32_t iter = 0; state != threadpool_state_idle &&
                          iter < PTHREADPOOL_SPIN_WAIT_ITERATIONS;
       iter++) {
    pthreadpool_yield(iter);
    state = pthreadpool_load_acquire_uint32_t(&threadpool->state);
  }
  if (state != threadpool_state_idle) {
    pthread_mutex_lock(&threadpool->state_mutex);
    while (state != threadpool_state_idle) {
      pthread_cond_wait(&threadpool->state_condvar, &threadpool->state_mutex);
      state = pthreadpool_load_acquire_uint32_t(&threadpool->state);
    }
    pthread_mutex_unlock(&threadpool->state_mutex);
  }
#endif  // PTHREADPOOL_USE_FUTEX

  assert(threadpool->num_active_threads == 0);
  assert(threadpool->state == threadpool_state_idle);

  /* Unprotect the global threadpool structures now that we're done. */
  pthread_mutex_unlock(&threadpool->execution_mutex);
}

void pthreadpool_release_all_threads(struct pthreadpool* threadpool) {
  if (threadpool != NULL) {
    // Get a hold on the threadpool state lock.
    pthreadpool_spin_lock_acquire(&threadpool->state_spin_lock);

    // Set the state to "done".
    pthreadpool_log_debug("main thread switching state from %u to %u.",
                          (uint32_t)threadpool->state, threadpool_state_done);
    pthreadpool_store_release_uint32_t(&threadpool->state,
                                       threadpool_state_done);

    /* Wake up any thread waiting on a change of state. */
#if PTHREADPOOL_USE_FUTEX
    futex_wake_all((pthreadpool_atomic_uint32_t*)&threadpool->state);
#else
    pthread_mutex_lock(&threadpool->state_mutex);
    pthread_cond_signal(&threadpool->state_condvar);
    pthread_mutex_unlock(&threadpool->state_mutex);
#endif  // PTHREADPOOL_USE_FUTEX

    // Wait for any pending jobs to complete.
    uint32_t num_recruited_threads =
        pthreadpool_load_relaxed_uint32_t(&threadpool->num_recruited_threads);
    if (num_recruited_threads) {
#if !PTHREADPOOL_USE_FUTEX
      pthread_mutex_lock(&threadpool->completion_mutex);
#endif  // !PTHREADPOOL_USE_FUTEX
      while (num_recruited_threads) {
        pthreadpool_spin_lock_release(&threadpool->state_spin_lock);
        pthreadpool_log_debug("waiting on %u threads...",
                              num_recruited_threads);
#if PTHREADPOOL_USE_FUTEX
        futex_wait(&threadpool->num_recruited_threads, num_recruited_threads);
#else
        pthread_cond_wait(&threadpool->completion_condvar,
                          &threadpool->completion_mutex);
#endif  // PTHREADPOOL_USE_FUTEX
        pthreadpool_spin_lock_acquire(&threadpool->state_spin_lock);
        num_recruited_threads =
            pthreadpool_load_relaxed_size_t(&threadpool->num_recruited_threads);
      }
#if !PTHREADPOOL_USE_FUTEX
      pthread_mutex_unlock(&threadpool->completion_mutex);
#endif  // !PTHREADPOOL_USE_FUTEX
    }

    // Set the state back to "idle".
    pthreadpool_store_release_uint32_t(&threadpool->state,
                                       threadpool_state_idle);

    // Release the threadpool state lock.
    pthreadpool_spin_lock_release(&threadpool->state_spin_lock);
  }
}

PTHREADPOOL_WEAK void pthreadpool_destroy(struct pthreadpool* threadpool) {
  if (threadpool != NULL) {
    /* Tell all threads to stop. */
    pthreadpool_release_all_threads(threadpool);

    if (!threadpool->executor) {
      /* Wait until all threads return */
      for (size_t thread = 1; thread < threadpool->max_num_threads; thread++) {
        pthread_join(threadpool->threads[thread].thread_object, NULL);
      }
    }

    /* Release resources */
    pthread_mutex_destroy(&threadpool->execution_mutex);
#if !PTHREADPOOL_USE_FUTEX
    pthread_mutex_destroy(&threadpool->completion_mutex);
    pthread_cond_destroy(&threadpool->completion_condvar);
    pthread_mutex_destroy(&threadpool->command_mutex);
    pthread_cond_destroy(&threadpool->command_condvar);
    pthread_mutex_destroy(&threadpool->state_mutex);
    pthread_cond_destroy(&threadpool->state_condvar);
#endif

#if PTHREADPOOL_USE_CPUINFO
    cpuinfo_deinitialize();
#endif

    pthreadpool_log_debug("destroying threadpool at %p.", threadpool);
    pthreadpool_deallocate(threadpool);
  }
}

PTHREADPOOL_PRIVATE_IMPL(pthreadpool_destroy)
