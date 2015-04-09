/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@gilande.com>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#include <config.h>

#ifdef __KERNEL__
#include <linux/jiffies.h>
#endif // __KERNEL__

#include <bcm63xx_log.h>
#include <timer.h>

// Include after system files
#include <compile.h>

#define TIMER_MAX_DRIFT 1000000 // in usecs

// The number of usecs for one jiffie
static unsigned long one_jiffie_to_usecs;

static void bcm_timer_fn(unsigned long data)
{
   bcm_timer_t *t = (bcm_timer_t *)(data);

#ifdef BCMPH_DEBUG
   t->stats.count += 1;
#endif // BCMPH_DEBUG

   (*(t->callback))(t);
}

void bcm_timer_reschedule(bcm_timer_t *t)
{
   unsigned long period_in_jiffies = t->period_in_jiffies;
   unsigned long soon;

   if (bcm_timer_is_active(t)) {
      // We compute next expires
      t->drift += t->drift_increment;
      if (t->drift >= one_jiffie_to_usecs) {
         period_in_jiffies -= 1;
         t->drift -= one_jiffie_to_usecs;
      }

      // Now we test that timer will not expire too soon
      t->kobject.expires += period_in_jiffies;

#ifdef __KERNEL__
      soon = jiffies + 1;
#else // !__KERNEL__
      soon = get_jiffies() + 1;
#endif // !__KERNEL__
      if (time_before(t->kobject.expires, soon)) {
#ifdef BCMPH_DEBUG
         d_bcm_pr_debug("too soon %lu\n", (unsigned long)(t->stats.count));
         t->stats.too_soon += 1;
#endif // BCMPH_DEBUG
         t->drift += (((long)(soon) - (long)(t->kobject.expires)) * one_jiffie_to_usecs);
         if (t->drift > TIMER_MAX_DRIFT) {
#ifdef BCMPH_DEBUG
            d_bcm_pr_debug("reset drift %lu\n", (unsigned long)(t->stats.count));
            t->stats.reset_drift += 1;
#endif // BCMPH_DEBUG
            t->drift = 0;
         }
         t->kobject.expires = soon;
      }

      add_timer(&(t->kobject));
   }
}

static void bcm_timer_compute_params(bcm_timer_t *t, unsigned long period_in_usecs)
{
   unsigned long real_period;

   bcm_pr_debug("bcm_timer_compute_params(period_in_usecs=%lu)\n", (unsigned long)(period_in_usecs));

   t->period_in_jiffies = usecs_to_jiffies(period_in_usecs);
   real_period = jiffies_to_usecs(t->period_in_jiffies);
   if (real_period < period_in_usecs) {
      t->period_in_jiffies += 1;
      real_period = jiffies_to_usecs(t->period_in_jiffies);
      bcm_assert(real_period >= period_in_usecs);
   }
   t->drift_increment = real_period - period_in_usecs;
   bcm_assert((0 == t->drift_increment) || ((t->drift_increment > 0) && (t->period_in_jiffies >= 1)));

   bcm_pr_debug("one_jiffie_to_usecs = %lu, period_in_jiffies = %lu, real_period = %lu, drift_increment = %lu\n",
      (unsigned long)(one_jiffie_to_usecs), (unsigned long)(t->period_in_jiffies),
      (unsigned long)(real_period), (unsigned long)(t->drift_increment));
}

void bcm_timer_start(bcm_timer_t *t, unsigned long period_in_msecs)
{
   bcm_pr_debug("bcm_timer_start()\n");

   bcm_timer_stop(t);
   bcm_timer_compute_params(t, period_in_msecs * 1000);
   t->is_active = true;
#ifdef __KERNEL__
   t->kobject.expires = jiffies + t->period_in_jiffies;
#else // !__KERNEL__
   t->kobject.expires = get_jiffies() + t->period_in_jiffies;
#endif // !__KERNEL__
   t->drift = 0;
   add_timer(&(t->kobject));
}

void bcm_timer_stop(bcm_timer_t *t)
{
   bcm_pr_debug("bcm_timer_stop()\n");

   if (bcm_timer_is_active(t)) {
      t->is_active = false;
      del_timer_sync(&(t->kobject));
#ifdef BCMPH_DEBUG
      bcm_pr_debug("stats.count = %lu, stats.too_soon = %lu, stats.drift = %lu, stats.reset_drift = %lu\n",
         (unsigned long)(t->stats.count),
         (unsigned long)(t->stats.too_soon),
         (unsigned long)(t->drift),
         (unsigned long)(t->stats.reset_drift));
#endif // BCMPH_DEBUG
   }
}

int __init bcm_timer_init(bcm_timer_t *t, void (*callback)(bcm_timer_t *t))
{
   int ret = 0;

   bcm_pr_debug("bcm_timer_init()\n");

   bcm_assert(NULL != callback);

   one_jiffie_to_usecs = jiffies_to_usecs(1);

   t->callback = callback;
   t->period_in_jiffies = msecs_to_jiffies(1000);

   // Init kernel timer
   init_timer(&(t->kobject));
   t->kobject.function = bcm_timer_fn;
   t->kobject.data = (unsigned long)(t);

   return (ret);
}

void bcm_timer_deinit(bcm_timer_t *t)
{
   bcm_pr_debug("bcm_timer_deinit()\n");

   bcm_timer_stop(t);
}
