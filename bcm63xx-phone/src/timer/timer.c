/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <config.h>

#include <extern/linux/barrier.h>
#include <extern/linux/jiffies.h>
#include <extern/linux/stddef.h>

#include <bcm63xx_log.h>
#include <timer.h>

// Include after system files
#include <compile.h>

/* The number of usecs for one jiffie */
static unsigned long one_jiffy_to_usecs;

#define PERIOD_MAX_DRIFT 1000000 // in usecs

void __init bcm_period_init(bcm_period_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   one_jiffy_to_usecs = jiffies_to_usecs(1);

   t->period_in_jiffies = msecs_to_jiffies(1000);
   t->drift_increment = 0;
   t->drift = 0;
   t->expires_in_jiffies = 0;
}

void bcm_period_set_period(bcm_period_t *t, unsigned long period_in_usecs)
{
   unsigned long real_period;

   bcm_pr_debug("%s(period_in_usecs=%lu)\n", __func__, period_in_usecs);

   bcm_assert(period_in_usecs > 0);

   t->period_in_jiffies = usecs_to_jiffies(period_in_usecs);
   real_period = jiffies_to_usecs(t->period_in_jiffies);
   if (real_period < period_in_usecs) {
      t->period_in_jiffies += 1;
      real_period = jiffies_to_usecs(t->period_in_jiffies);
      bcm_assert(real_period >= period_in_usecs);
   }
   bcm_assert(t->period_in_jiffies > 0);
   t->drift_increment = real_period - period_in_usecs;

   t->expires_in_jiffies = get_jiffies();
   t->drift = 0;

   bcm_pr_debug("one_jiffy_to_usecs=%lu, period_in_jiffies=%lu, real_period=%lu, drift_increment=%lu\n",
      (unsigned long)(one_jiffy_to_usecs), (unsigned long)(t->period_in_jiffies),
      (unsigned long)(real_period), (unsigned long)(t->drift_increment));
}

void bcm_period_inc(bcm_period_t *t)
{
   unsigned long period_in_jiffies = t->period_in_jiffies;

   dd_bcm_pr_debug("%s()\n", __func__);

   t->drift += t->drift_increment;
   if (t->drift >= one_jiffy_to_usecs) {
      period_in_jiffies -= 1;
      t->drift -= one_jiffy_to_usecs;
   }
   t->expires_in_jiffies += period_in_jiffies;
}


unsigned long bcm_period_inc_to_jiffies(bcm_period_t *t,
   unsigned long jiffies)
{
   unsigned long ret = 0;

   while (time_before(t->expires_in_jiffies, jiffies)) {
      ret += 1;
      bcm_period_inc(t);
   }

   return (ret);
}

static inline unsigned long bcm_period_inc_drift_to_jiffies(bcm_period_t *t,
   unsigned long jiffies)
{
   unsigned long ret = bcm_period_inc_to_jiffies(t, jiffies);
   if (ret > 0) {
      t->drift += (ret * bcm_period_get_period(t));
   }
   return (ret);
}

static inline void bcm_period_reset_drift(bcm_period_t *t)
{
   if (t->drift_increment > 0) {
      t->drift %= one_jiffy_to_usecs;
   }
   else {
      t->drift = 0;
   }
}

static void bcm_timer_fn(unsigned long data)
{
   bcm_timer_t *t = (bcm_timer_t *)(data);

   dd_bcm_pr_debug("%s()\n", __func__);

   (*(t->callback))(t);
}

static inline void bcm_timer_fix_if_expired_or_expires_now(bcm_timer_t *t)
{
   unsigned long now = get_jiffies();
   unsigned long tick_count;

   dd_bcm_pr_debug("%s()\n", __func__);

   tick_count = bcm_period_inc_drift_to_jiffies(&(t->period), now + 1);
   if (tick_count > 0) {
#ifdef BCMPH_DEBUG
      t->stats.too_soon += 1;
#endif // BCMPH_DEBUG
      if ((t->period.period_in_jiffies <= 1)
          || (t->period.drift > PERIOD_MAX_DRIFT)) {
         // If (t->period.period_in_jiffies <= 1), we reset drift as it
         // will never decrease.
         // Because to decrease drift we should schedule timer at
         // (jiffies + t->period.period_in_jiffies - 1), but if
         // t->period.period_in_jiffies <= 1,it means scheduling timer
         // now or in the past, and we forbid that.
#ifdef BCMPH_DEBUG
         t->stats.reset_drift += 1;
#endif // BCMPH_DEBUG
         bcm_period_reset_drift(&(t->period));
      }
   }
}

void bcm_timer_reschedule(bcm_timer_t *t)
{
   dd_bcm_pr_debug("%s()\n", __func__);

   if (bcm_timer_is_active(t)) {
      // We compute next expires
      bcm_period_inc(&(t->period));

      // Now we test that timer will not expire now or in the past
      bcm_timer_fix_if_expired_or_expires_now(t);

      t->kobject.expires = t->period.expires_in_jiffies;
      add_timer(&(t->kobject));
   }
}

void bcm_timer_stop(bcm_timer_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   if (bcm_timer_is_active(t)) {
      t->is_active = false;
      /* Make sure is_active is written to memory, so that all processor cores can see the change */
      wmb();
      del_timer_sync(&(t->kobject));
   }
}

void bcm_timer_start(bcm_timer_t *t, unsigned long period_in_usecs)
{
   bcm_pr_debug("%s(period_in_usecs=%lu)\n", __func__, period_in_usecs);

   bcm_timer_stop(t);
   bcm_period_set_period(&(t->period), period_in_usecs);
   t->is_active = true;
   bcm_timer_reschedule(t);
}

int __init bcm_timer_init(bcm_timer_t *t, void (*callback)(bcm_timer_t *timer))
{
   int ret = 0;

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert(NULL != callback);

   t->is_active = false;

   bcm_period_init(&(t->period));

   t->callback = callback;

   // Init kernel timer
   init_timer(&(t->kobject));
   t->kobject.function = bcm_timer_fn;
   t->kobject.data = (unsigned long)(t);

#ifdef BCMPH_DEBUG
   t->stats.too_soon = 0;
   t->stats.reset_drift = 0;
#endif // BCMPH_DEBUG

   return (ret);
}

void bcm_timer_deinit(bcm_timer_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   bcm_timer_stop(t);
   bcm_period_deinit(&(t->period));
}

static void bcm_periodic_timer_callback(struct bcm_timer *timer)
{
   bcm_periodic_timer_t *t = container_of(timer, bcm_periodic_timer_t, timer);
   bcm_periodic_work_t *w;
   unsigned long now = get_jiffies();
   unsigned long flags;

   dd_bcm_pr_debug("%s()\n", __func__);

#ifdef BCMPH_DEBUG
   t->stats.count += 1;
#endif // BCMPH_DEBUG

   spin_lock_irqsave(&(t->lock), flags);
   list_for_each_entry(w, &(t->works), list) {
      unsigned long tick_count = bcm_period_inc_to_jiffies(&(w->period), now);
      if (tick_count) {
         bcm_periodic_work_queue(w, (int)(tick_count));
      }
   }
   spin_unlock_irqrestore(&(t->lock), flags);

   bcm_timer_reschedule(&(t->timer));
}

void __init bcm_periodic_timer_init(bcm_periodic_timer_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   spin_lock_init(&(t->lock));
   INIT_LIST_HEAD(&(t->works));
   bcm_timer_init(&(t->timer), bcm_periodic_timer_callback);
#ifdef BCMPH_DEBUG
   t->stats.count = 0;
#endif // BCMPH_DEBUG
}

void bcm_periodic_timer_deinit(bcm_periodic_timer_t *t)
{
   bcm_periodic_work_t *w;
   bcm_periodic_work_t *w_tmp;
   unsigned long flags;

   bcm_pr_debug("%s()\n", __func__);

   bcm_timer_stop(&(t->timer));

   spin_lock_irqsave(&(t->lock), flags);
   list_for_each_entry_safe(w, w_tmp, &(t->works), list) {
      list_del_init(&(w->list));
   }
   spin_unlock_irqrestore(&(t->lock), flags);
}
