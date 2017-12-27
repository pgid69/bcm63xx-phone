/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_JIFFIES_H__
#define __FK_LINUX_JIFFIES_H__

#include <fake_kernel_compile.h>

#include <time.h>

extern void jiffies_init(void);

extern void jiffies_deinit(void);

#define MAX_JIFFY_OFFSET ((LONG_MAX >> 1) - 1)

#define MS_PER_JIFFY 4

#define msecs_to_jiffies(arg) ((arg + (MS_PER_JIFFY - 1)) / MS_PER_JIFFY)

#define jiffies_to_msecs(arg) (arg * MS_PER_JIFFY)

#define usecs_to_jiffies(arg) (((arg) + ((MS_PER_JIFFY * 1000) - 1)) / (MS_PER_JIFFY * 1000))

#define jiffies_to_usecs(arg) ((arg) * (MS_PER_JIFFY * 1000))

static inline int time_before(unsigned long v, unsigned long ref)
{
   return (v < ref);
}

static inline int time_after(unsigned long v, unsigned long ref)
{
   return (v > ref);
}

extern unsigned long get_jiffies(void);

extern void jiffies_to_timespec(unsigned long jiffies, struct timespec *ts);

#endif // __FK_LINUX_JIFFIES_H__

