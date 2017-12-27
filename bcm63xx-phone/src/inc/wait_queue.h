/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __WAIT_QUEUE_H__
#define __WAIT_QUEUE_H__

#include "config.h"

#include <extern/linux/atomic.h>
#include <extern/linux/sched.h>
#include <extern/linux/wait.h>

#include <bcm63xx_log.h>
#include <mutex.h>

typedef struct {
   // Kernel wait queue used to block processes
   wait_queue_head_t wq;
   // Counter that tells processes blocked in wait queue that something
   // has changed
   atomic_t counter;
} bcm_wait_queue_t;

static inline void bcm_wait_queue_init(bcm_wait_queue_t *t)
{
   bcm_pr_debug("%s()\n", __func__);
   init_waitqueue_head(&(t->wq));
   atomic_set(&(t->counter), 0);
}

static inline void bcm_wait_queue_deinit(bcm_wait_queue_t *t)
{
   bcm_pr_debug("%s()\n", __func__);
   deinit_waitqueue_head(&(t->wq));
}

static inline void bcm_wait_queue_wake_up(bcm_wait_queue_t *t)
{
   dd_bcm_pr_debug("%s()\n", __func__);
   atomic_inc(&(t->counter));
   wake_up_interruptible_all(&(t->wq));
}

static inline int bcm_wait_queue_get_counter(bcm_wait_queue_t *t)
{
   return (atomic_read(&(t->counter)));
}

extern int bcm_wait_queue_wait_event_counter(bcm_wait_queue_t *t, int counter,
   bcmph_mutex_t *lock);

static inline int bcm_wait_queue_wait_event(bcm_wait_queue_t *t,
   bcmph_mutex_t *lock)
{
   return (bcm_wait_queue_wait_event_counter(t,
      bcm_wait_queue_get_counter(t), lock));
}

extern int bcm_wait_queue_wait_event_counter_timeout(bcm_wait_queue_t *t,
   int counter, long timeout_in_jiffies, bcmph_mutex_t *lock);

static inline int bcm_wait_queue_wait_event_timeout(bcm_wait_queue_t *t,
   long timeout_in_jiffies, bcmph_mutex_t *lock)
{
   return (bcm_wait_queue_wait_event_counter_timeout(t,
      bcm_wait_queue_get_counter(t), timeout_in_jiffies,
      lock));
}

#endif // __WAIT_QUEUE_H__
