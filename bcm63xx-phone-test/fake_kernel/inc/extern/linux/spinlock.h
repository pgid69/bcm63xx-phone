/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_SPINLOCK_H__
#define __FK_LINUX_SPINLOCK_H__

#include <fake_kernel_compile.h>

#include <extern/linux/atomic.h>
#include <extern/linux/barrier.h>
#include <extern/linux/delay.h>

#include <pthread.h>
#include <sched.h>
#include <string.h>

#include <bcm63xx_log.h>

typedef struct {
   struct {
      atomic_t locked;
      pthread_t thread_id;
   } priv;
} spinlock_t;

static inline void spin_lock_init(spinlock_t *t)
{
   memset(t, 0, sizeof(*t));
   atomic_set(&(t->priv.locked), 0);
}

static inline void spin_lock(spinlock_t *t)
{
   bcm_assert((!pthread_equal(pthread_self(), t->priv.thread_id))
      || (0 == atomic_read(&(t->priv.locked))));

   while (!atomic_cas(&(t->priv.locked), 0, 1)) {
      sched_yield();
   }
   t->priv.thread_id = pthread_self();
   barrier();
}

static inline void spin_unlock(spinlock_t *t)
{
   if (pthread_equal(pthread_self(), t->priv.thread_id)) {
      atomic_cas(&(t->priv.locked), 1, 0);
      barrier();
   }
   else {
      bcm_assert(false);
   }
}

static inline int spin_is_locked(spinlock_t *t)
{
   return (atomic_read(&(t->priv.locked)));
}

static inline void spin_lock_bh(spinlock_t *t)
{
   spin_lock(t);
}

static inline void spin_unlock_bh(spinlock_t *t)
{
   spin_unlock(t);
}

static inline void spin_lock_irqsave(spinlock_t *t, unsigned long flags)
{
   spin_lock(t);
}

static inline void spin_unlock_irqrestore(spinlock_t *t, unsigned long flags)
{
   spin_unlock(t);
}

#endif // __FK_LINUX_SPINLOCK_H__

