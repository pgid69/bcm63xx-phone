/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_WORKQUEUE_H__
#define __FK_LINUX_WORKQUEUE_H__

#include <fake_kernel_compile.h>

#include <extern/linux/list.h>

#include <pthread.h>

extern void workqueue_init(void);

extern void workqueue_deinit(void);

struct work_struct {
   struct {
      void (*cb)(struct work_struct *work);
      struct list_head list;
      int state;
      long owner;
      pthread_t thread_id;
   } priv;
};

extern void INIT_WORK(struct work_struct *t, void (*cb)(struct work_struct *work));

#define WQ_UNBOUND 0x01
#define WQ_FREEZABLE 0x02
#define WQ_HIGHPRI 0

struct workqueue_struct;

extern struct workqueue_struct *alloc_workqueue(const char *fmt, unsigned int flags, int max_active);

extern void flush_workqueue(struct workqueue_struct *t);

extern void destroy_workqueue(struct workqueue_struct *t);

extern bool queue_work(struct workqueue_struct *t, struct work_struct *work);

extern bool cancel_work_sync(struct work_struct *t);

#endif // __FK_LINUX_WORKQUEUE_H__

