/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_MUTEX_H__
#define __FK_LINUX_MUTEX_H__

#include <fake_kernel_compile.h>

#include <pthread.h>
#include <string.h>

#include <bcm63xx_log.h>

struct mutex {
   pthread_mutex_t obj;
   struct {
      pthread_mutexattr_t attr;
   } priv;
};

extern void mutex_init(struct mutex *t);

static inline void mutex_destroy(struct mutex *t)
{
   int ret = pthread_mutex_destroy(&(t->obj));
   if (ret) {
      bcm_pr_warn("**** pthread_mutex_destroy() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
   }
   ret = pthread_mutexattr_destroy(&(t->priv.attr));
   if (ret) {
      bcm_pr_warn("**** pthread_mutexattr_destroy() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
   }
}

static inline int mutex_trylock(struct mutex *t)
{
   if (pthread_mutex_trylock(&(t->obj))) {
      return (0);
   }
   else {
      return (1);
   }
}

static inline int mutex_lock_interruptible(struct mutex *t)
{
   int ret = pthread_mutex_lock(&(t->obj));
   if (ret) {
      bcm_pr_err("**** pthread_mutex_lock() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
   }
   return (ret);
}

static inline void mutex_unlock(struct mutex *t)
{
   int ret = pthread_mutex_unlock(&(t->obj));
   if (ret) {
      bcm_pr_warn("**** pthread_mutex_lock() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
   }
}

#endif // __FK_LINUX_MUTEX_H__

