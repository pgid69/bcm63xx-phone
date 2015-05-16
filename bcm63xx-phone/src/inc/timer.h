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

#ifdef __KERNEL__
#include <linux/workqueue.h>
#else
#include "fake_kernel.h"
#endif // __KERNEL__

typedef struct bcm_timer {
   bool is_active;
   struct timer_list kobject;
   unsigned long period_in_jiffies;
   unsigned long drift_increment; // in usecs
   unsigned long drift; // in usecs
   void (*callback)(struct bcm_timer *t);
#ifdef BCMPH_DEBUG
   struct {
      __u32 count;
      __u32 too_soon;
      __u32 reset_drift;
   } stats;
#endif // BCMPH_DEBUG
} bcm_timer_t;

extern int bcm_timer_init(bcm_timer_t *t, void (*callback)(bcm_timer_t *timer));

extern void bcm_timer_deinit(bcm_timer_t *t);

extern void bcm_timer_start(bcm_timer_t *t, unsigned long period_in_msecs);

extern void bcm_timer_stop(bcm_timer_t *t);

static inline bool bcm_timer_is_active(const bcm_timer_t *t)
{
   return (t->is_active);
}

extern void bcm_timer_reschedule(bcm_timer_t *t);

#endif // __TIMER_H__
