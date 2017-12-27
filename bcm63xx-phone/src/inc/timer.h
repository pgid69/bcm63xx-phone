/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __TIMER_H__
#define __TIMER_H__

#include "config.h"

#include <extern/linux/atomic.h>
#include <extern/linux/jiffies.h>
#include <extern/linux/list.h>
#include <extern/linux/spinlock.h>
#include <extern/linux/timer.h>
#include <extern/linux/workqueue.h>
#include <extern/linux/types.h>

typedef struct {
   unsigned long period_in_jiffies;
   unsigned long drift_increment; // in usecs
   unsigned long drift; // in usecs
   unsigned long expires_in_jiffies;
} bcm_period_t;

extern void bcm_period_init(bcm_period_t *t);

static inline void bcm_period_deinit(bcm_period_t *t)
{
   bcm_pr_debug("%s()\n", __func__);
}

extern void bcm_period_set_period(bcm_period_t *t,
   unsigned long period_in_usecs);

static inline unsigned long bcm_period_get_period(const bcm_period_t *t)
{
   return (jiffies_to_usecs(t->period_in_jiffies) - t->drift_increment);
}

extern void bcm_period_inc(bcm_period_t *t);

extern unsigned long bcm_period_inc_to_jiffies(bcm_period_t *t,
   unsigned long jiffies);

typedef struct bcm_timer {
   bool is_active;
   struct timer_list kobject;
   bcm_period_t period;
   void (*callback)(struct bcm_timer *t);
#ifdef BCMPH_DEBUG
   struct {
      __u32 too_soon;
      __u32 reset_drift;
   } stats;
#endif // BCMPH_DEBUG
} bcm_timer_t;

extern int bcm_timer_init(bcm_timer_t *t, void (*callback)(bcm_timer_t *timer));

extern void bcm_timer_deinit(bcm_timer_t *t);

extern void bcm_timer_reschedule(bcm_timer_t *t);

extern void bcm_timer_start(bcm_timer_t *t, unsigned long period_in_usecs);

extern void bcm_timer_stop(bcm_timer_t *t);

static inline bool bcm_timer_is_active(const bcm_timer_t *t)
{
   dd_bcm_pr_debug("%s()\n", __func__);

   return (t->is_active);
}

typedef struct {
   struct workqueue_struct *wq;
   struct work_struct work;
   struct list_head list;
   bcm_period_t period;
   atomic_t tick_count;
} bcm_periodic_work_t;

static inline void bcm_periodic_work_init(bcm_periodic_work_t *t,
   struct workqueue_struct *wq, void (*make_work)(struct work_struct *w))
{
   bcm_assert((NULL != wq) && (NULL != make_work));

   bcm_pr_debug("%s()\n", __func__);

   t->wq = wq;
   INIT_WORK(&(t->work), make_work);
   INIT_LIST_HEAD(&(t->list));
   bcm_period_init(&(t->period));
   atomic_set(&(t->tick_count), 0);
}

static inline void bcm_periodic_work_deinit(bcm_periodic_work_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   bcm_period_deinit(&(t->period));
   bcm_assert(list_empty(&(t->list)));
   cancel_work_sync(&(t->work));
   t->wq = NULL;
}

static inline void bcm_periodic_work_queue(bcm_periodic_work_t *t,
   int tick_count)
{
   dd_bcm_pr_debug("%s()\n", __func__);

   atomic_add(tick_count, &(t->tick_count));
   queue_work(t->wq, &(t->work));
}

static inline int bcm_periodic_work_get_tick_count(bcm_periodic_work_t *t)
{
   return (atomic_read(&(t->tick_count)));
}

static inline int bcm_periodic_work_dec_tick_count(bcm_periodic_work_t *t,
   int tick_count)
{
   return (atomic_sub_return(tick_count, &(t->tick_count)));
}

typedef struct {
   spinlock_t lock;
   struct list_head works;
   bcm_timer_t timer;
#ifdef BCMPH_DEBUG
   struct {
      __u32 count;
   } stats;
#endif // BCMPH_DEBUG
} bcm_periodic_timer_t;

extern void bcm_periodic_timer_init(bcm_periodic_timer_t *t);

extern void bcm_periodic_timer_deinit(bcm_periodic_timer_t *t);

static inline bool bcm_periodic_timer_is_active(const bcm_periodic_timer_t *t)
{
   dd_bcm_pr_debug("%s()\n", __func__);

   return (bcm_timer_is_active(&(t->timer)));
}

static inline void bcm_periodic_timer_start(bcm_periodic_timer_t *t,
   unsigned long period_in_usecs)
{
   bcm_pr_debug("%s()\n", __func__);

   bcm_timer_start(&(t->timer), period_in_usecs);
}

static inline void bcm_periodic_timer_stop(bcm_periodic_timer_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   bcm_timer_stop(&(t->timer));

#ifdef BCMPH_DEBUG
   bcm_pr_info("Timer stats: count=%lu, drift=%lu, too_soon=%lu, reset_drift=%lu\n",
      (unsigned long)(t->stats.count),
      (unsigned long)(t->timer.period.drift),
      (unsigned long)(t->timer.stats.too_soon),
      (unsigned long)(t->timer.stats.reset_drift));
#endif // BCMPH_DEBUG
}

static inline void bcm_periodic_timer_add_work(bcm_periodic_timer_t *t,
   bcm_periodic_work_t *w, unsigned long period_in_us)
{
   unsigned long flags;

   bcm_pr_debug("%s(period_in_us=%lu)\n", __func__, period_in_us);

   bcm_assert((NULL != w) && (period_in_us > 0));

   spin_lock_irqsave(&(t->lock), flags);
   bcm_period_set_period(&(w->period), period_in_us);
   bcm_period_inc(&(w->period));
   atomic_set(&(w->tick_count), 0);
   list_del_init(&(w->list));
   list_add(&(w->list), &(t->works));
   spin_unlock_irqrestore(&(t->lock), flags);
}

static inline void bcm_periodic_timer_del_work(bcm_periodic_timer_t *t,
   bcm_periodic_work_t *w)
{
   unsigned long flags;

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert(NULL != w);
   spin_lock_irqsave(&(t->lock), flags);
   list_del_init(&(w->list));
   spin_unlock_irqrestore(&(t->lock), flags);
}

#endif // __TIMER_H__
