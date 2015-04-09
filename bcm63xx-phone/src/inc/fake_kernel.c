/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@gilande.com>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#include <config.h>

#ifndef __KERNEL__

#include <sys/time.h>

#include <fake_kernel.h>

unsigned long get_jiffies(void)
{
   static bool initialized = false;
   static unsigned long t0_secs;
   static unsigned int t0_msecs;
   struct timeval tv;
   unsigned long ret;

   if (!initialized) {
      gettimeofday(&(tv), NULL);
      t0_secs = tv.tv_sec;
      t0_msecs = (tv.tv_usec + 500) / 1000;
      initialized = true;
      ret = 0;
   }
   else {
      unsigned int msecs;
      gettimeofday(&(tv), NULL);
      ret = (tv.tv_sec - t0_secs) * 1000;
      msecs = (tv.tv_usec + 500) / 1000;
      if (msecs > t0_msecs) {
         ret += (msecs - t0_msecs);
      }
      else {
         ret -= (t0_msecs - msecs);
      }
   }
   return (msecs_to_jiffies(ret));
}

static struct timer_list *timers = NULL;

void add_timer(struct timer_list *t)
{
   struct timer_list *prev = NULL;
   struct timer_list *cur = timers;

   bcm_assert((NULL != t->function) && (NULL == t->next));

   while (NULL != cur) {
      bcm_assert(cur != t);
      if (cur->expires <= t->expires) {
         prev = cur;
         cur = cur->next;
      }
      else {
         break;
      }
   }
   if (NULL != prev) {
      t->next = prev->next;
      prev->next = cur;
   }
   else {
      t->next = timers;
      timers = t;
   }
}

void del_timer_sync(struct timer_list *t)
{
   bcm_assert(t->function != NULL);
   struct timer_list *prev = NULL;
   struct timer_list *cur = timers;

   bcm_assert((NULL != t->function) && (NULL == t->next));

   while (NULL != cur) {
      if (cur == t) {
         break;
      }
      prev = cur;
      cur = cur->next;
   }
   if (NULL != cur) {
      bcm_assert(t == cur);
      if (NULL != prev) {
         bcm_assert(cur != timers);
         prev->next = cur->next;
      }
      else {
         bcm_assert(cur == timers);
         timers = cur->next;
      }
      cur->next = NULL;
   }
}

void schedule(void)
{
   unsigned long jiffies = get_jiffies();
   while (NULL != timers) {
      if (time_after(jiffies, timers->expires)) {
         struct timer_list *cur = timers;
         timers = timers->next;
         cur->next = NULL;
         (*(cur->function))(cur->data);
         jiffies = get_jiffies();
      }
      else {
         break;
      }
   }
}

void msleep(unsigned long msecs)
{
   unsigned long j = get_jiffies() + msecs_to_jiffies(msecs);

   for (;;) {
      schedule();
      if (get_jiffies() > j) {
         break;
      }
      usleep(1000);
   }
}

#endif // !__KERNEL__
