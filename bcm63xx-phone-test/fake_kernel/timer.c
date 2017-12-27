/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <config.h>

#include <extern/linux/bitops.h>
#include <extern/linux/delay.h>
#include <extern/linux/jiffies.h>
#include <extern/linux/sched.h>
#include <extern/linux/stddef.h>
#include <extern/linux/timer.h>

#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include <workqueue_internal.h>

enum {
   TQ_PROCESSING_BIT = WQ_LAST_BIT + 1,
};

typedef struct {
   struct {
      abstract_workqueue_t wq;
      timer_t timer_id;
      struct sigevent sev;
   } priv;
} timer_queue_t;

typedef struct {
   struct timer_list timer;
   timer_queue_t *tq;
   volatile bool has_run;
} timer_barrier_t;

static inline struct timer_list *work_struct_to_timer_list(struct work_struct *work)
{
   return (container_of(work, struct timer_list, priv.work));
}

static inline struct work_struct *timer_list_to_work_struct(struct timer_list *timer)
{
   return (&(timer->priv.work));
}

static void timer_list_work_cb(struct work_struct *work)
{
   struct timer_list *timer = work_struct_to_timer_list(work);
   (*(timer->function))(timer->data);
}

void init_timer(struct timer_list *t)
{
   bcm_pr_debug("%s()\n", __func__);

   t->function = NULL;
   t->data = 0;
   t->expires = 0;
   INIT_WORK(&(t->priv.work), timer_list_work_cb);
}

static void timer_queue_set_time(timer_queue_t *t)
{
   bcm_assert(abstract_workqueue_spin_is_locked(&(t->priv.wq)));

   if ((!test_bit(TQ_PROCESSING_BIT, &(t->priv.wq.priv.flags)))
       && (!test_bit(WQ_DEINIT_BIT, &(t->priv.wq.priv.flags)))) {
      struct work_struct *work;

      work = list_first_entry(&(t->priv.wq.priv.works), struct work_struct, priv.list);
      if (NULL != work) {
         struct timer_list *timer = work_struct_to_timer_list(work);
         struct itimerspec timer_spec;
         int ret;

         memset(&(timer_spec.it_interval), 0, sizeof(timer_spec.it_interval));
         jiffies_to_timespec(timer->expires, &(timer_spec.it_value));
         ret = timer_settime(t->priv.timer_id, TIMER_ABSTIME, &(timer_spec), NULL);
         if (ret) {
            bcm_pr_warn("**** timer_settime() failed with error code %d : '%s' ****\n",
               (int)(ret), strerror(ret));
         }
      }
   }
}

static void timer_queue_run(union sigval sigval)
{
   timer_queue_t *t = sigval.sival_ptr;

   abstract_workqueue_spin_lock(&(t->priv.wq));
   set_bit(TQ_PROCESSING_BIT, &(t->priv.wq.priv.flags));
   while (!test_bit(WQ_DEINIT_BIT, &(t->priv.wq.priv.flags))) {
      struct work_struct *cur;
      struct timer_list *timer = NULL;
      unsigned long now = get_jiffies();

      // Look for a timer to run
      list_for_each_entry(cur, &(t->priv.wq.priv.works), priv.list) {
         int state = work_struct_get_state(cur);
         bcm_assert(&(t->priv.wq) == work_struct_get_owner(cur));
         if (WORK_STATE_WAITING == state) {
            timer = work_struct_to_timer_list(cur);
            if (!time_after(timer->expires, now)) {
               break;
            }
            else {
               timer = NULL;
            }
         }
         else {
            bcm_assert(WORK_STATE_RUNNING_WAITING == state);
         }
      }
      if (NULL == timer) {
         // No timer to run, exit the loop
         break;
      }
      else {
         abstract_workqueue_run_work(&(t->priv.wq), timer_list_to_work_struct(timer));
      }
   }
   clear_bit(TQ_PROCESSING_BIT, &(t->priv.wq.priv.flags));
   timer_queue_set_time(t);
   abstract_workqueue_spin_unlock(&(t->priv.wq));
}

static void timer_queue_queue_work(abstract_workqueue_t *t,
   struct work_struct *work)
{
   struct timer_list *timer = work_struct_to_timer_list(work);
   struct work_struct *cur;
   bool first_entry;

   // Remove the timer from que queue
   bcm_assert(abstract_workqueue_spin_is_locked(t));
   list_del_init(&(work->priv.list));
   first_entry = true;
   list_for_each_entry(cur, &(t->priv.works), priv.list) {
      struct timer_list *cur_timer = work_struct_to_timer_list(cur);
      if (time_before(timer->expires, cur_timer->expires)) {
         break;
      }
      first_entry = false;
   }
   if (first_entry) {
      list_add(&(work->priv.list), &(t->priv.works));
   }
   else {
      list_add(&(work->priv.list), &(cur->priv.list));
   }
}

static void timer_queue_dequeue_work(abstract_workqueue_t *t,
   struct work_struct *work)
{
   abstract_workqueue_dequeue_work(t, work);
}

static void timer_queue_new_work_to_run(abstract_workqueue_t *t,
   struct work_struct *work)
{
   if (abstract_workqueue_spin_is_locked(t)) {
      timer_queue_set_time(container_of(t, timer_queue_t, priv.wq));
   }
   else {
      abstract_workqueue_spin_lock(t);
      timer_queue_set_time(container_of(t, timer_queue_t, priv.wq));
      abstract_workqueue_spin_unlock(t);
   }
}

static vtable_abstract_workqueue_t timer_queue_vtbl = {
   .queue_work = timer_queue_queue_work,
   .dequeue_work = timer_queue_dequeue_work,
   .new_work_to_run = timer_queue_new_work_to_run,
};

static inline void timer_queue_init(timer_queue_t *t)
{
   int ret;

   bcm_pr_debug("%s()\n", __func__);

   abstract_workqueue_init(&(t->priv.wq), &(timer_queue_vtbl));

   // Create a system timer
   clear_bit(TQ_PROCESSING_BIT, &(t->priv.wq.priv.flags));
   memset(&(t->priv.sev), 0, sizeof(t->priv.sev));
   t->priv.sev.sigev_notify = SIGEV_THREAD;
   t->priv.sev.sigev_notify_function = timer_queue_run;
   t->priv.sev.sigev_value.sival_ptr = t;
   ret = timer_create(CLOCK_MONOTONIC, &(t->priv.sev), &(t->priv.timer_id));
   if (ret) {
      bcm_pr_err("**** timer_create() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
      exit(ret);
   }
}

static void timer_queue_deinit_cb(unsigned long data)
{
   timer_barrier_t *t = (timer_barrier_t *)(data);

   /*
    We set flag WQ_DEINIT_BIT to prevent :
    - adding a new timer
    - processing the timers still remaining in the list
    - rearming the system timer
   */
   abstract_workqueue_prepare_deinit(&(t->tq->priv.wq));

   t->has_run = true;
}

static inline void timer_queue_deinit(timer_queue_t *t)
{
   int ret;
   timer_barrier_t barrier;

   bcm_pr_debug("%s()\n", __func__);

   // Init a barrier and set the callback to timer_queue_deinit_cb()
   init_timer(&(barrier.timer));
   barrier.timer.function = timer_queue_deinit_cb;
   barrier.timer.data = (unsigned long)(&(barrier));
   barrier.timer.expires = get_jiffies();
   barrier.tq = t;
   barrier.has_run = false;
   add_timer(&(barrier.timer));

   // Wait that the barrier has run
   while (!barrier.has_run) {
      sched_yield();
   }

   /*
    Wait for the timer to be really stopped
    After the barrier ran,
    - adding a new timer is no more possible
    - the system timer callback stop processing the list of timers.
    - the system timer is not rearmed
    So we just have to wait the flag TQ_PROCESSING_BIT to be cleared
    to ensure that we exit the system timer callback
   */
   abstract_workqueue_spin_lock(&(t->priv.wq));
   for (;;) {
      if (test_bit(TQ_PROCESSING_BIT, &(t->priv.wq.priv.flags))) {
         abstract_workqueue_spin_unlock(&(t->priv.wq));
         sched_yield();
         abstract_workqueue_spin_lock(&(t->priv.wq));
      }
      else {
         break;
      }
   }
   abstract_workqueue_spin_unlock(&(t->priv.wq));

   // Destroy the system timer
   ret = timer_delete(t->priv.timer_id);
   if (ret) {
      bcm_pr_warn("**** timer_delete() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
   }

   abstract_workqueue_deinit(&(t->priv.wq));
}

static timer_queue_t timer_queue;

void add_timer(struct timer_list *t)
{
   abstract_workqueue_queue_work(&(timer_queue.priv.wq), &(t->priv.work));
}

int del_timer_sync(struct timer_list *timer)
{
   bcm_pr_debug("%s()\n", __func__);
   return (cancel_work_sync(timer_list_to_work_struct(timer)));
}

void timer_init(void)
{
   bcm_pr_debug("%s()\n", __func__);
   memset(&(timer_queue), 0, sizeof(timer_queue));
   timer_queue_init(&(timer_queue));
}

void timer_deinit(void)
{
   bcm_pr_debug("%s()\n", __func__);
   timer_queue_deinit(&(timer_queue));
}
