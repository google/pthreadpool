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
#include <stdatomic.h>
#include <stdbool.h>
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
#ifdef PTHREADPOOL_DEBUG_LOGGING
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

static void checkin_worker_thread(struct pthreadpool* threadpool) {
#if PTHREADPOOL_USE_FUTEX
  if (pthreadpool_decrement_fetch_acquire_release_size_t(
          &threadpool->active_threads) == 0) {
    pthreadpool_store_release_uint32_t(&threadpool->has_active_threads, 0);
    futex_wake_all(&threadpool->has_active_threads);
  }
#else
  pthread_mutex_lock(&threadpool->completion_mutex);
  if (pthreadpool_decrement_fetch_release_size_t(&threadpool->active_threads) ==
      0) {
    pthread_cond_signal(&threadpool->completion_condvar);
  }
  pthread_mutex_unlock(&threadpool->completion_mutex);
#endif
}

static void wait_worker_threads(struct pthreadpool* threadpool) {
/* Initial check */
#if PTHREADPOOL_USE_FUTEX
  uint32_t has_active_threads =
      pthreadpool_load_acquire_uint32_t(&threadpool->has_active_threads);
  if (has_active_threads == 0) {
    return;
  }
#else
  size_t active_threads =
      pthreadpool_load_acquire_size_t(&threadpool->active_threads);
  if (active_threads == 0) {
    return;
  }
#endif

  /* Spin-wait */
  for (uint32_t i = 0; i < PTHREADPOOL_SPIN_WAIT_ITERATIONS; i++) {
    pthreadpool_yield(i);

#if PTHREADPOOL_USE_FUTEX
    has_active_threads =
        pthreadpool_load_acquire_uint32_t(&threadpool->has_active_threads);
    if (has_active_threads == 0) {
      return;
    }
#else
    active_threads =
        pthreadpool_load_acquire_size_t(&threadpool->active_threads);
    if (active_threads == 0) {
      return;
    }
#endif
  }

/* Fall-back to mutex/futex wait */
#if PTHREADPOOL_USE_FUTEX
  while ((has_active_threads = pthreadpool_load_acquire_uint32_t(
              &threadpool->has_active_threads)) != 0) {
    futex_wait(&threadpool->has_active_threads, 1);
  }
#else
  pthread_mutex_lock(&threadpool->completion_mutex);
  while (pthreadpool_load_acquire_size_t(&threadpool->active_threads) != 0) {
    pthread_cond_wait(&threadpool->completion_condvar,
                      &threadpool->completion_mutex);
  };
  pthread_mutex_unlock(&threadpool->completion_mutex);
#endif
}

static uint32_t wait_for_new_command(struct pthreadpool* threadpool,
                                     uint32_t last_command,
                                     uint32_t last_flags) {
  uint32_t command = pthreadpool_load_acquire_uint32_t(&threadpool->command);
  if (command != last_command) {
    return command;
  }

  if ((last_flags & PTHREADPOOL_FLAG_YIELD_WORKERS) == 0) {
    /* Spin-wait loop */
    for (uint32_t i = 0; i < PTHREADPOOL_SPIN_WAIT_ITERATIONS; i++) {
      pthreadpool_yield(i);

      command = pthreadpool_load_acquire_uint32_t(&threadpool->command);
      if (command != last_command) {
        return command;
      }
    }
  }

/* Spin-wait disabled or timed out, fall back to mutex/futex wait */
#if PTHREADPOOL_USE_FUTEX
  do {
    futex_wait(&threadpool->command, last_command);
    command = pthreadpool_load_acquire_uint32_t(&threadpool->command);
  } while (command == last_command);
#else
  /* Lock the command mutex */
  pthread_mutex_lock(&threadpool->command_mutex);
  /* Read the command */
  while ((command = pthreadpool_load_acquire_uint32_t(&threadpool->command)) ==
         last_command) {
    /* Wait for new command */
    pthread_cond_wait(&threadpool->command_condvar, &threadpool->command_mutex);
  }
  /* Read a new command */
  pthread_mutex_unlock(&threadpool->command_mutex);
#endif
  return command;
}

struct pthreadpool_single_job_data {
  struct pthreadpool* threadpool;
  uint32_t thread_id;
  uint32_t job_id;
};

static void thread_run_single_job(void* arg) {
  // Unpack the argument and free it now so that we don't have to worry about
  // doing that later.
  const struct pthreadpool_single_job_data data =
      *(const struct pthreadpool_single_job_data*)arg;
  free(arg);
  struct pthreadpool* threadpool = data.threadpool;

  // Get a hold on the threadpool state lock.
  pthreadpool_spin_lock_acquire(&threadpool->state_spin_lock);

  // Get the current threadpool state.
  const uint32_t curr_state =
      atomic_load_explicit(&threadpool->state, memory_order_acquire);

  // If we're the last pending thread of a "done" threadpool, signal for any
  // waiting cleanup and bail before anything else can go wrong (e.g.
  // `threadpool` might get cleaned up).
  if (pthreadpool_decrement_fetch_relaxed_uint32_t(
          &threadpool->num_pending_jobs) == 0 &&
      curr_state == threadpool_state_done) {
    pthreadpool_spin_lock_release(&threadpool->state_spin_lock);
#if PTHREADPOOL_USE_FUTEX
    futex_wake_all(&threadpool->num_pending_jobs);
#else
    pthread_mutex_lock(&threadpool->completion_mutex);
    pthread_cond_signal(&threadpool->completion_condvar);
    pthread_mutex_unlock(&threadpool->completion_mutex);
#endif  // PTHREADPOOL_USE_FUTEX
    return;
  }

  // Get some useful variables from the threadpool.
  thread_function_t thread_function = NULL;
  uint32_t flags;

  // If this is the job ID we are looking for, and it is running...
  if (threadpool->job_id == data.job_id &&
      curr_state == threadpool_state_running) {
    // Try to join in on the current job, unless there are too many threads
    // already on it.
    size_t curr_num_threads = atomic_load_explicit(
        &threadpool->num_active_threads, memory_order_acquire);
    const uint32_t max_active_threads = threadpool->max_active_threads;
    while (curr_num_threads < max_active_threads &&
           !atomic_compare_exchange_weak_explicit(
               &threadpool->num_active_threads, &curr_num_threads,
               curr_num_threads + 1, memory_order_relaxed,
               memory_order_relaxed)) {
    }

    // Only run the job if there are not already too many threads.
    if (curr_num_threads < max_active_threads) {
      // Get some useful variables from the threadpool.
      thread_function = (thread_function_t)atomic_load_explicit(
          &threadpool->thread_function, memory_order_acquire);
      flags = pthreadpool_load_relaxed_uint32_t(&threadpool->flags);
    }

    // If we were the last thread to join this job, then we need to change the
    // state from "running" to "wrapping up".
    if (curr_num_threads + 1 == max_active_threads) {
      pthreadpool_log_debug("thread %u switching state from %u to %u.",
                            data.thread_id, curr_state,
                            threadpool_state_wrapping_up);
      atomic_store_explicit(&threadpool->state, threadpool_state_wrapping_up,
                            memory_order_release);
    }
  }

  // Release the threadpool state lock.
  pthreadpool_spin_lock_release(&threadpool->state_spin_lock);

  // If we have a job to do, just do it.
  if (thread_function) {
    pthreadpool_log_debug("thread %u working on job %u (arg=%p).",
                          data.thread_id, data.job_id, arg);
    // Save the current FPU state, if requested.
    struct fpu_state saved_fpu_state = {0};
    if (flags & PTHREADPOOL_FLAG_DISABLE_DENORMALS) {
      saved_fpu_state = get_fpu_state();
      disable_fpu_denormals();
    }

    // Call the job function.
    thread_function(threadpool, &threadpool->threads[data.thread_id]);

    // Restore the original FPU state in case we clobbered it.
    if (flags & PTHREADPOOL_FLAG_DISABLE_DENORMALS) {
      set_fpu_state(saved_fpu_state);
    }

    pthreadpool_log_debug("thread %u done working on job %u (arg=%p).",
                          data.thread_id, data.job_id, arg);

    // If we were the last thread on this job, switch the state back to "idle"
    // and signal anybody waiting on that state change.
    pthreadpool_spin_lock_acquire(&threadpool->state_spin_lock);
    if (atomic_fetch_sub_explicit(&threadpool->num_active_threads, 1,
                                  memory_order_relaxed) == 1) {
      pthreadpool_log_debug("thread %u switching state from %u to %u.",
                            data.thread_id, (uint32_t)threadpool->state,
                            threadpool_state_idle);
      atomic_store_explicit(&threadpool->state, threadpool_state_idle,
                            memory_order_release);
#if PTHREADPOOL_USE_FUTEX
      futex_wake_all((pthreadpool_atomic_uint32_t*)&threadpool->state);
#else
      pthread_mutex_lock(&threadpool->completion_mutex);
      pthread_cond_signal(&threadpool->completion_condvar);
      pthread_mutex_unlock(&threadpool->completion_mutex);
#endif  // PTHREADPOOL_USE_FUTEX
    } else if (threadpool->state == threadpool_state_running) {
      // Otherwise, switch the state from "running" to "wrapping up" since there
      // is no work left to steal.
      pthreadpool_log_debug("thread %u switching state from %u to %u.",
                            data.thread_id, (uint32_t)threadpool->state,
                            threadpool_state_wrapping_up);
      atomic_store_explicit(&threadpool->state, threadpool_state_wrapping_up,
                            memory_order_release);
    }
    pthreadpool_spin_lock_release(&threadpool->state_spin_lock);
  }

  pthreadpool_log_debug("thread %u leaving main loop.", data.thread_id);
}

static void* thread_main(void* arg) {
  struct thread_info* thread = (struct thread_info*)arg;
  struct pthreadpool* threadpool = thread->threadpool;
  uint32_t last_command = threadpool_command_init;
  struct fpu_state saved_fpu_state = {0};
  uint32_t flags = 0;

  /* Check in */
  checkin_worker_thread(threadpool);

  /* Monitor new commands and act accordingly */
  for (;;) {
    uint32_t command = wait_for_new_command(threadpool, last_command, flags);
    pthreadpool_fence_acquire();

    flags = pthreadpool_load_relaxed_uint32_t(&threadpool->flags);

    /* Process command */
    switch (command & THREADPOOL_COMMAND_MASK) {
      case threadpool_command_parallelize: {
        const thread_function_t thread_function =
            (thread_function_t)pthreadpool_load_relaxed_void_p(
                &threadpool->thread_function);
        if (flags & PTHREADPOOL_FLAG_DISABLE_DENORMALS) {
          saved_fpu_state = get_fpu_state();
          disable_fpu_denormals();
        }

        thread_function(threadpool, thread);
        if (flags & PTHREADPOOL_FLAG_DISABLE_DENORMALS) {
          set_fpu_state(saved_fpu_state);
        }
        break;
      }
      case threadpool_command_shutdown:
        /* Exit immediately: the master thread is waiting on pthread_join */
        return NULL;
      case threadpool_command_init:
        /* To inhibit compiler warning */
        break;
    }
    /* Notify the master thread that we finished processing */
    checkin_worker_thread(threadpool);
    /* Update last command */
    last_command = command;
  };
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
#if PTHREADPOOL_USE_CPUINFO
  if (!cpuinfo_initialize()) {
    return NULL;
  }
#endif

  if (threads_count == 0) {
    threads_count = get_num_cpus();
  }

  // If the user wants just one thread, then there's no need to use a scheduler.
  if (threads_count == 1) {
    return NULL;
  }

  struct pthreadpool* threadpool = pthreadpool_allocate(threads_count);
  if (threadpool == NULL) {
    return NULL;
  }
  threadpool->threads_count = fxdiv_init_size_t(threads_count);
  for (size_t tid = 0; tid < threads_count; tid++) {
    threadpool->threads[tid].thread_number = tid;
    threadpool->threads[tid].threadpool = threadpool;
  }

  /* Thread pool with a single thread computes everything on the caller thread.
   */
  if (threads_count > 1) {
    pthread_mutex_init(&threadpool->execution_mutex, NULL);
#if !PTHREADPOOL_USE_FUTEX
    pthread_mutex_init(&threadpool->completion_mutex, NULL);
    pthread_cond_init(&threadpool->completion_condvar, NULL);
    pthread_mutex_init(&threadpool->command_mutex, NULL);
    pthread_cond_init(&threadpool->command_condvar, NULL);
#endif

#if PTHREADPOOL_USE_FUTEX
    pthreadpool_store_relaxed_uint32_t(&threadpool->has_active_threads, 1);
#endif
    pthreadpool_store_relaxed_size_t(&threadpool->active_threads,
                                     threads_count - 1 /* caller thread */);

    /* Caller thread serves as worker #0. Thus, we create system threads
     * starting with worker #1. */
    for (size_t tid = 1; tid < threads_count; tid++) {
      pthread_create(&threadpool->threads[tid].thread_object, NULL,
                     &thread_main, &threadpool->threads[tid]);
    }

    /* Wait until all threads initialize */
    wait_worker_threads(threadpool);
  }
  return threadpool;
}

PTHREADPOOL_PRIVATE_IMPL(pthreadpool_create)

struct pthreadpool* pthreadpool_create_v2(
    struct pthreadpool_scheduler* scheduler, size_t max_num_threads) {
#if PTHREADPOOL_USE_CPUINFO
  if (!cpuinfo_initialize()) {
    return NULL;
  }
#endif

  if (max_num_threads == 0) {
    max_num_threads = get_num_cpus();
  }

  struct pthreadpool* threadpool = pthreadpool_allocate(max_num_threads);
  if (threadpool == NULL) {
    return NULL;
  }
  threadpool->scheduler = scheduler;
  threadpool->max_num_threads = max_num_threads;
  threadpool->threads_count = fxdiv_init_size_t(max_num_threads);
  for (size_t tid = 0; tid < max_num_threads; tid++) {
    threadpool->threads[tid].thread_number = tid;
    threadpool->threads[tid].threadpool = threadpool;
  }
  threadpool->num_active_threads = 0;

  /* Thread pool with a single thread computes everything on the caller thread.
   */
  if (max_num_threads > 1) {
    pthread_mutex_init(&threadpool->execution_mutex, NULL);
#if !PTHREADPOOL_USE_FUTEX
    pthread_mutex_init(&threadpool->completion_mutex, NULL);
    pthread_cond_init(&threadpool->completion_condvar, NULL);
    pthread_mutex_init(&threadpool->command_mutex, NULL);
    pthread_cond_init(&threadpool->command_condvar, NULL);
#endif
  }

  return threadpool;
}

PTHREADPOOL_INTERNAL void pthreadpool_parallelize_old(
    struct pthreadpool* threadpool, thread_function_t thread_function,
    const void* params, size_t params_size, void* task, void* context,
    size_t linear_range, uint32_t flags) {
  assert(threadpool != NULL);
  assert(thread_function != NULL);
  assert(task != NULL);
  assert(linear_range > 1);

  /* Protect the global threadpool structures */
  pthread_mutex_lock(&threadpool->execution_mutex);

#if !PTHREADPOOL_USE_FUTEX
  /* Lock the command variables to ensure that threads don't start processing
   * before they observe complete command with all arguments */
  pthread_mutex_lock(&threadpool->command_mutex);
#endif

  /* Setup global arguments */
  pthreadpool_store_relaxed_void_p(&threadpool->thread_function,
                                   (void*)thread_function);
  pthreadpool_store_relaxed_void_p(&threadpool->task, task);
  pthreadpool_store_relaxed_void_p(&threadpool->argument, context);
  pthreadpool_store_relaxed_uint32_t(&threadpool->flags, flags);

  /* Locking of completion_mutex not needed: readers are sleeping on
   * command_condvar */
  const struct fxdiv_divisor_size_t threads_count = threadpool->threads_count;
  pthreadpool_store_relaxed_size_t(&threadpool->active_threads,
                                   threads_count.value - 1 /* caller thread */);
#if PTHREADPOOL_USE_FUTEX
  pthreadpool_store_relaxed_uint32_t(&threadpool->has_active_threads, 1);
#endif

  if (params_size != 0) {
    memcpy(&threadpool->params, params, params_size);
    pthreadpool_fence_release();
  }

  /* Spread the work between threads */
  const struct fxdiv_result_size_t range_params =
      fxdiv_divide_size_t(linear_range, threads_count);
  size_t range_start = 0;
  for (size_t tid = 0; tid < threads_count.value; tid++) {
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

  /*
   * Update the threadpool command.
   * Imporantly, do it after initializing command parameters (range, task,
   * argument, flags)
   * ~(threadpool->command | THREADPOOL_COMMAND_MASK) flips the bits not in
   * command mask to ensure the unmasked command is different then the last
   * command, because worker threads monitor for change in the unmasked command.
   */
  const uint32_t old_command =
      pthreadpool_load_relaxed_uint32_t(&threadpool->command);
  const uint32_t new_command =
      ~(old_command | THREADPOOL_COMMAND_MASK) | threadpool_command_parallelize;

  /*
   * Store the command with release semantics to guarantee that if a worker
   * thread observes the new command value, it also observes the updated command
   * parameters.
   *
   * Note: release semantics is necessary even with a conditional variable,
   * because the workers might be waiting in a spin-loop rather than the
   * conditional variable.
   */
  pthreadpool_store_release_uint32_t(&threadpool->command, new_command);
#if PTHREADPOOL_USE_FUTEX
  /* Wake up the threads */
  futex_wake_all(&threadpool->command);
#else
  /* Unlock the command variables before waking up the threads for better
   * performance */
  pthread_mutex_unlock(&threadpool->command_mutex);

  /* Wake up the threads */
  pthread_cond_broadcast(&threadpool->command_condvar);
#endif

  /* Save and modify FPU denormals control, if needed */
  struct fpu_state saved_fpu_state = {0};
  if (flags & PTHREADPOOL_FLAG_DISABLE_DENORMALS) {
    saved_fpu_state = get_fpu_state();
    disable_fpu_denormals();
  }

  /* Do computations as worker #0 */
  thread_function(threadpool, &threadpool->threads[0]);

  /* Restore FPU denormals control, if needed */
  if (flags & PTHREADPOOL_FLAG_DISABLE_DENORMALS) {
    set_fpu_state(saved_fpu_state);
  }

  /* Wait until the threads finish computation */
  wait_worker_threads(threadpool);

  /* Unprotect the global threadpool structures */
  pthread_mutex_unlock(&threadpool->execution_mutex);
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
  if (!threadpool->scheduler) {
    pthreadpool_parallelize_old(threadpool, thread_function, params,
                                params_size, task, context, linear_range,
                                flags);
    return;
  }

  /* Protect the global threadpool structures */
  pthread_mutex_lock(&threadpool->execution_mutex);

  /* Make sure the threadpool is idle. */
  assert(threadpool->state == threadpool_state_idle);

  /* Setup global arguments */
  struct pthreadpool_scheduler* scheduler = threadpool->scheduler;
  pthreadpool_store_relaxed_void_p(&threadpool->thread_function,
                                   (void*)thread_function);
  pthreadpool_store_relaxed_void_p(&threadpool->task, task);
  pthreadpool_store_relaxed_void_p(&threadpool->argument, context);
  pthreadpool_store_relaxed_uint32_t(&threadpool->flags, flags);
  assert(threadpool->num_active_threads == 0);
  if (params_size != 0) {
    memcpy(&threadpool->params, params, params_size);
  }
  threadpool->job_id += 1;

  // How many threads should we parallelize over?
  const uint32_t num_threads =
      min(threadpool->max_num_threads, scheduler->num_threads(scheduler) + 1);
  atomic_store_explicit(&threadpool->max_active_threads, num_threads,
                        memory_order_relaxed);
  threadpool->threads_count = fxdiv_init_size_t(num_threads);

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

  /* Change the threadpool state to "running". */
  pthreadpool_spin_lock_acquire(&threadpool->state_spin_lock);
  assert(threadpool->state == threadpool_state_idle);
  atomic_store_explicit(&threadpool->state, threadpool_state_running,
                        memory_order_release);
  pthreadpool_spin_lock_release(&threadpool->state_spin_lock);

  /* Create the threads for this parallel job. */
  pthreadpool_fetch_add_relaxed_size_t(&threadpool->num_pending_jobs,
                                       num_threads);
  for (size_t tid = 1; tid < num_threads; tid++) {
    // Quit early?
    if (atomic_load_explicit(&threadpool->state, memory_order_acquire) !=
        threadpool_state_running) {
      pthreadpool_fetch_decrement_n_relaxed_size_t(
          &threadpool->num_pending_jobs, num_threads - tid);
      break;
    }

    // Create the data for this job.
    struct pthreadpool_single_job_data* data =
        malloc(sizeof(struct pthreadpool_single_job_data));
    *data = (struct pthreadpool_single_job_data){.threadpool = threadpool,
                                                 .thread_id = tid,
                                                 .job_id = threadpool->job_id};

    /* Fly, my pretties! Fly, fly, fly! */
    pthreadpool_log_debug("scheduling thread %u of job %u (arg=%p).",
                          data->thread_id, data->job_id, data);
    scheduler->schedule(scheduler, data, thread_run_single_job);
  }

  /* Do a bit of work ourselves, as thread zero. */
  if (atomic_load_explicit(&threadpool->state, memory_order_acquire) ==
      threadpool_state_running) {
    struct pthreadpool_single_job_data* data =
        malloc(sizeof(struct pthreadpool_single_job_data));
    *data = (struct pthreadpool_single_job_data){
        .threadpool = threadpool, .thread_id = 0, .job_id = threadpool->job_id};
    thread_run_single_job(data);
  } else {
    pthreadpool_decrement_fetch_relaxed_size_t(&threadpool->num_pending_jobs);
  }

  /* Since we finished our part of the job, we know the state should be either
   * "wrapping up" or "idle", i.e. definitely no longer "running". */
  uint32_t state =
      atomic_load_explicit(&threadpool->state, memory_order_acquire);
  assert(state == threadpool_state_idle ||
         state == threadpool_state_wrapping_up);
  if (state != threadpool_state_idle) {
#if !PTHREADPOOL_USE_FUTEX
    pthread_mutex_lock(&threadpool->completion_mutex);
#endif  // !PTHREADPOOL_USE_FUTEX
    for (uint32_t iter = 0; state != threadpool_state_idle; iter++) {
      if (iter < PTHREADPOOL_SPIN_WAIT_ITERATIONS) {
        pthreadpool_yield(iter);
      } else {
#if PTHREADPOOL_USE_FUTEX
        futex_wait((pthreadpool_atomic_uint32_t*)&threadpool->state, state);
#else
        pthread_cond_wait(&threadpool->completion_condvar,
                          &threadpool->completion_mutex);
#endif  // PTHREADPOOL_USE_FUTEX
        state = atomic_load_explicit(&threadpool->state, memory_order_acquire);
      }
    }
#if !PTHREADPOOL_USE_FUTEX
    pthread_mutex_unlock(&threadpool->completion_mutex);
#endif  // !PTHREADPOOL_USE_FUTEX
  }

  assert(threadpool->state == threadpool_state_idle);

  /* Unprotect the global threadpool structures now that we're done. */
  pthread_mutex_unlock(&threadpool->execution_mutex);
}

PTHREADPOOL_WEAK void pthreadpool_destroy(struct pthreadpool* threadpool) {
  if (threadpool != NULL) {
    if (threadpool->scheduler) {
      // Set the state to "done".
      pthreadpool_spin_lock_acquire(&threadpool->state_spin_lock);
      pthreadpool_log_debug("main thread switching state from %u to %u.",
                            (uint32_t)threadpool->state, threadpool_state_done);
      atomic_store_explicit(&threadpool->state, threadpool_state_done,
                            memory_order_release);

      uint32_t num_pending_jobs =
          pthreadpool_load_relaxed_uint32_t(&threadpool->num_pending_jobs);
      if (num_pending_jobs) {
#if !PTHREADPOOL_USE_FUTEX
        pthread_mutex_lock(&threadpool->completion_mutex);
#endif  // !PTHREADPOOL_USE_FUTEX
        while (num_pending_jobs) {
          pthreadpool_spin_lock_release(&threadpool->state_spin_lock);
          pthreadpool_log_debug("waiting on %u jobs...", num_pending_jobs);
#if PTHREADPOOL_USE_FUTEX
          futex_wait(&threadpool->num_pending_jobs, num_pending_jobs);
#else
          pthread_cond_wait(&threadpool->completion_condvar,
                            &threadpool->completion_mutex);
#endif  // PTHREADPOOL_USE_FUTEX
          pthreadpool_spin_lock_acquire(&threadpool->state_spin_lock);
          num_pending_jobs =
              pthreadpool_load_relaxed_size_t(&threadpool->num_pending_jobs);
        }
#if !PTHREADPOOL_USE_FUTEX
        pthread_mutex_unlock(&threadpool->completion_mutex);
#endif  // !PTHREADPOOL_USE_FUTEX
      }
      pthreadpool_log_debug("destroying threadpool at %p.", threadpool);
    } else {
      const size_t threads_count = threadpool->threads_count.value;
      if (threads_count > 1) {
#if PTHREADPOOL_USE_FUTEX
        pthreadpool_store_relaxed_size_t(&threadpool->active_threads,
                                         threads_count - 1 /* caller thread */);
        pthreadpool_store_relaxed_uint32_t(&threadpool->has_active_threads, 1);

        /*
         * Store the command with release semantics to guarantee that if a
         * worker thread observes the new command value, it also observes the
         * updated active_threads/has_active_threads values.
         */
        pthreadpool_store_release_uint32_t(&threadpool->command,
                                           threadpool_command_shutdown);

        /* Wake up worker threads */
        futex_wake_all(&threadpool->command);
#else
        /* Lock the command variable to ensure that threads don't shutdown until
         * both command and active_threads are updated */
        pthread_mutex_lock(&threadpool->command_mutex);

        pthreadpool_store_relaxed_size_t(&threadpool->active_threads,
                                         threads_count - 1 /* caller thread */);

        /*
         * Store the command with release semantics to guarantee that if a
         * worker thread observes the new command value, it also observes the
         * updated active_threads value.
         *
         * Note: the release fence inside pthread_mutex_unlock is insufficient,
         * because the workers might be waiting in a spin-loop rather than the
         * conditional variable.
         */
        pthreadpool_store_release_uint32_t(&threadpool->command,
                                           threadpool_command_shutdown);

        /* Wake up worker threads */
        pthread_cond_broadcast(&threadpool->command_condvar);

        /* Commit the state changes and let workers start processing */
        pthread_mutex_unlock(&threadpool->command_mutex);
#endif

        /* Wait until all threads return */
        for (size_t thread = 1; thread < threads_count; thread++) {
          pthread_join(threadpool->threads[thread].thread_object, NULL);
        }

        /* Release resources */
        pthread_mutex_destroy(&threadpool->execution_mutex);
#if !PTHREADPOOL_USE_FUTEX
        pthread_mutex_destroy(&threadpool->completion_mutex);
        pthread_cond_destroy(&threadpool->completion_condvar);
        pthread_mutex_destroy(&threadpool->command_mutex);
        pthread_cond_destroy(&threadpool->command_condvar);
#endif
      }
    }
#if PTHREADPOOL_USE_CPUINFO
    cpuinfo_deinitialize();
#endif
    pthreadpool_deallocate(threadpool);
  }
}

PTHREADPOOL_PRIVATE_IMPL(pthreadpool_destroy)
