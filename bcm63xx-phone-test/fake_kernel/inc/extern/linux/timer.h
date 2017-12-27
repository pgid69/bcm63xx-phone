/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_TIMER_H__
#define __FK_LINUX_TIMER_H__

#include <fake_kernel_compile.h>

#include <extern/linux/workqueue.h>

#include <signal.h>
#include <time.h>

struct timer_queue;

struct timer_list {
   void (*function)(unsigned long);
   unsigned long data;
   unsigned long expires;
   struct {
      struct work_struct work;
   } priv;
};

extern void init_timer(struct timer_list *t);

extern void add_timer(struct timer_list *t);

extern int del_timer_sync(struct timer_list *t);

extern void timer_init(void);

extern void timer_deinit(void);

#endif // __FK_LINUX_TIMER_H__

