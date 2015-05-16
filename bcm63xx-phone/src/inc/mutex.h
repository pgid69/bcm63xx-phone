/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifndef __MUTEX_H__
#define __MUTEX_H__

#ifdef __KERNEL__
#include <linux/mutex.h>
#else // !__KERNEL__
#include <fake_kernel.h>
#endif // !__KERNEL__

typedef struct
{
   struct mutex *lock;
   bool is_locked;
} bcmph_mutex_t;

static inline void bcmph_mutex_init(bcmph_mutex_t *t, struct mutex *lock, bool is_locked)
{
   bcm_assert(NULL != lock);
   t->lock = lock;
   t->is_locked = is_locked;
}

static inline int bcmph_mutex_lock_interruptible(bcmph_mutex_t *t)
{
   int ret;

   bcm_assert(!t->is_locked);
   ret = mutex_lock_interruptible(t->lock);
   if (!ret) {
      t->is_locked = true;
   }
   return (ret);
}

static inline bool bcmph_mutex_is_locked(const bcmph_mutex_t *t)
{
   return (t->is_locked);
}

static inline bool bcmph_mutex_trylock(bcmph_mutex_t *t)
{
   bcm_assert(!t->is_locked);

   // BEWARE : mutex_trylock() return 1 on success and 0 if mutex can't be locked
   if (mutex_trylock(t->lock)) {
      t->is_locked = true;
   }
   return (t->is_locked);
}

static inline void bcmph_mutex_unlock(bcmph_mutex_t *t)
{
   bcm_assert(t->is_locked);
   mutex_unlock(t->lock);
   t->is_locked = false;
}

static inline void bcmph_mutex_deinit(bcmph_mutex_t *t)
{
   if (bcmph_mutex_is_locked(t)) {
      bcmph_mutex_unlock(t);
   }
}

#endif // __MUTEX_H__
