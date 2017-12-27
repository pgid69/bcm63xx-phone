/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <config.h>

#include <extern/linux/jiffies.h>

#include <stdlib.h>
#include <string.h>

#include <bcm63xx_log.h>

static struct {
   struct timespec ts0;
} jiffies_state;

unsigned long get_jiffies(void)
{
   int ret;
   unsigned long ms;
   struct timespec ts;

   ret = clock_gettime(CLOCK_MONOTONIC, &(ts));
   if (ret) {
      bcm_pr_warn("**** clock_gettime() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
   }
   if (ts.tv_nsec < jiffies_state.ts0.tv_nsec) {
      ts.tv_sec -= 1;
      ts.tv_nsec += 1000000000;
   }
   ts.tv_sec -= jiffies_state.ts0.tv_sec;
   ts.tv_nsec -= jiffies_state.ts0.tv_nsec;
   ms = (ts.tv_sec * 1000) + ((ts.tv_nsec + 500000) / 1000000);
   return (msecs_to_jiffies(ms));
}

void jiffies_to_timespec(unsigned long jiffies, struct timespec *ts)
{
   unsigned long ms = jiffies_to_msecs(jiffies);
   ts->tv_sec = ms / 1000;
   ts->tv_nsec = (ms % 1000) * 1000000;
   ts->tv_sec += jiffies_state.ts0.tv_sec;
   ts->tv_nsec += jiffies_state.ts0.tv_nsec;
   if (ts->tv_nsec >= 1000000000) {
      ts->tv_nsec -= 1000000000;
      ts->tv_sec += 1;
   }
}

void jiffies_init(void)
{
   bcm_pr_debug("%s()\n", __func__);

   int ret = clock_gettime(CLOCK_MONOTONIC, &(jiffies_state.ts0));
   if (ret) {
      bcm_pr_err("**** timer_create() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
      exit(ret);
   }
}

void jiffies_deinit(void)
{
   bcm_pr_debug("%s()\n", __func__);
}
