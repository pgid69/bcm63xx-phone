/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_WAIT_H__
#define __FK_LINUX_WAIT_H__

#include <fake_kernel_compile.h>

#include <extern/linux/jiffies.h>
#include <extern/linux/list.h>
#include <extern/linux/mutex.h>

#include <pthread.h>
#include <time.h>

#include <bcm63xx_log.h>

typedef struct wait_queue {
   struct {
      struct list_head list;
   } priv;
} wait_queue_t;

static inline void init_waitqueue(wait_queue_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   INIT_LIST_HEAD(&(t->priv.list));
}

typedef struct {
   struct {
      struct list_head wait_queues;
      pthread_cond_t cond;
      struct mutex mutex;
   } priv;
} wait_queue_head_t;

extern void init_waitqueue_head(wait_queue_head_t *t);

extern void deinit_waitqueue_head(wait_queue_head_t *t);

static inline void prepare_to_wait(wait_queue_head_t *t, wait_queue_t *wait)
{
   bcm_assert(list_empty(&(wait->priv.list)));
   mutex_lock_interruptible(&(t->priv.mutex));
   list_add_tail(&(wait->priv.list), &(t->priv.wait_queues));
}

static inline void finish_wait(wait_queue_head_t *t, wait_queue_t *wait)
{
   list_del_init(&(wait->priv.list));
   mutex_unlock(&(t->priv.mutex));
}

#define DEFINE_WAIT(name) \
   wait_queue_t name; \
   init_waitqueue(&(name))

#define wait_event_interruptible(wq, condition) \
   ({ \
      int __ret = 0; \
      do { \
         DEFINE_WAIT(__wait); \
         prepare_to_wait(&(wq), &(__wait)); \
         for (;;) { \
            if (condition) { \
               break; \
            } \
            __ret = -(pthread_cond_wait(&(wq.priv.cond), &(wq.priv.mutex.obj))); \
            if (__ret) { \
               bcm_pr_err("**** pthread_mutex_lock() failed with error code %d : '%s' ****\n", \
                  (int)(__ret), strerror(__ret)); \
               break; \
            } \
         } \
         finish_wait(&(wq), &(__wait)); \
      } while (0); \
      __ret; \
   })

#define wait_event_interruptible_timeout(wq, condition, timeout) \
   ({ \
      long __timeout = (timeout); \
      unsigned long __j0 = get_jiffies(); \
      long __ret; \
      do { \
         DEFINE_WAIT(__wait); \
         prepare_to_wait(&(wq), &(__wait)); \
         for (;;) { \
            struct timespec __ts; \
            unsigned long __j1 = get_jiffies(); \
            if (__j1 >= (__j0 + __timeout)) { \
               __ret = 0; \
               break; \
            } \
            if (condition) { \
               __ret = __j0 + __timeout - __j1; \
               break; \
            } \
            jiffies_to_timespec(__j1, &(__ts)); \
            __ret = -(pthread_cond_timedwait(&(wq.priv.cond), &(wq.priv.mutex.obj), &(__ts))); \
            if (__ret) { \
               if (-ETIMEDOUT == __ret) { \
                  __ret = 0; \
               } \
               else { \
                  bcm_pr_err("**** pthread_cond_timedwait() failed with error code %d : '%s' ****\n", \
                     (int)(__ret), strerror(__ret)); \
               } \
               break; \
            } \
         } \
         finish_wait(&(wq), &(__wait)); \
         if (!__ret && (condition)) { \
            __ret = 1; \
         } \
      } while (0); \
      __ret; \
   })

static inline void wake_up_interruptible(wait_queue_head_t *t)
{
   int ret = pthread_cond_signal(&(t->priv.cond));
   if (ret) {
      bcm_pr_warn("**** pthread_cond_signal() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
   }
}

static inline void wake_up_interruptible_all(wait_queue_head_t *t)
{
   int ret = pthread_cond_broadcast(&(t->priv.cond));
   if (ret) {
      bcm_pr_warn("**** pthread_cond_broadcast() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
   }
}

#endif // __FK_LINUX_WAIT_H__

