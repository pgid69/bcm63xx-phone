/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <config.h>

#include <extern/linux/bitops.h>
#include <extern/linux/mutex.h>
#include <extern/linux/slab.h>
#include <extern/linux/stddef.h>
#include <extern/linux/workqueue.h>

#include <workqueue_internal.h>

static void _work_struct_set_owner(struct work_struct *t,
   abstract_workqueue_t *new_owner)
{
   long old_val = 0;
   long new_val = (long)(new_owner);
   do {
      old_val = __sync_val_compare_and_swap(&(t->priv.owner), old_val, new_val);
   } while (new_val != old_val);
}

static inline void work_struct_clear_owner(struct work_struct *t)
{
   _work_struct_set_owner(t, NULL);
}

static abstract_workqueue_t *work_struct_set_owner_if_none(
   struct work_struct *t, abstract_workqueue_t *new_owner)
{
   long old_val = (long)(NULL);
   long new_val = (long)(new_owner);
   old_val = __sync_val_compare_and_swap(&(t->priv.owner), old_val, new_val);
   return ((abstract_workqueue_t *)(old_val));
}

static inline void work_struct_set_state(struct work_struct *t, int new_state)
{
   t->priv.state = new_state;
}

static inline pthread_t work_struct_get_thread_id(const struct work_struct *t)
{
   return (t->priv.thread_id);
}

static inline void work_struct_set_thread_id(struct work_struct *t)
{
   t->priv.thread_id = pthread_self();
}

void INIT_WORK(struct work_struct *t, void (*cb)(struct work_struct *work))
{
   bcm_pr_debug("%s()\n", __func__);

   memset(t, 0, sizeof(*t));
   bcm_assert(NULL != cb);
   t->priv.cb = cb;
   INIT_LIST_HEAD(&(t->priv.list));
   work_struct_clear_owner(t);
   work_struct_set_state(t, WORK_STATE_IDLE);
}

void abstract_workqueue_run_work(abstract_workqueue_t *t,
   struct work_struct *work)
{
   int state;

   bcm_assert((abstract_workqueue_spin_is_locked(t))
      && (t == work_struct_get_owner(work)));

   // Dequeue the work
   (*(t->priv.vtbl->dequeue_work))(t, work);
   state = work_struct_get_state(work);
   // Run the work if not cancelled
   if (WORK_STATE_WAITING == state) {
      work_struct_set_state(work, WORK_STATE_RUNNING);
      // Set thread ID executing the work to prevent cancelling
      // the work in the same thread
      work_struct_set_thread_id(work);
      abstract_workqueue_spin_unlock(t);
      (*(work->priv.cb))(work);
      // During execution of a work, it's state can change but not
      // its owner
      abstract_workqueue_spin_lock(t);
      bcm_assert(t == work_struct_get_owner(work));
      state = work_struct_get_state(work);
      if (WORK_STATE_RUNNING == state) {
         work_struct_set_state(work, WORK_STATE_IDLE);
         work_struct_clear_owner(work);
      }
      else if (WORK_STATE_RUNNING_WAITING == state) {
         work_struct_set_state(work, WORK_STATE_WAITING);
         // Signal to one of the other threads of the workqueue
         // that there is a job waiting execution
         (*(t->priv.vtbl->new_work_to_run))(t, work);
      }
      else if (WORK_STATE_RUNNING_CANCELLED == state) {
         work_struct_set_state(work, WORK_STATE_CANCELLED);
      }
   }
}

bool abstract_workqueue_queue_work(abstract_workqueue_t *t, struct work_struct *work)
{
   bool ret = false;
   abstract_workqueue_t *old_owner;

   bcm_assert((NULL != work) && (NULL != work->priv.cb));

   if (!test_bit(WQ_DEINIT_BIT, &(t->priv.flags))) {
      // Try to set work's owner if none and lock it
      abstract_workqueue_spin_lock(t);
      old_owner = work_struct_set_owner_if_none(work, t);
      if (NULL == old_owner) {
         // Work owned by t
         old_owner = t;
      }
      if (t != old_owner) {
         // Work owned by another workqueue
         abstract_workqueue_spin_unlock(t);
      }
      else {
         int state = work_struct_get_state(work);
         if (WORK_STATE_IDLE == state) {
            // Add the work in the queue
            work_struct_set_state(work, WORK_STATE_WAITING);
            (*(t->priv.vtbl->queue_work))(t, work);
            ret = true;
         }
         else if (WORK_STATE_RUNNING == state) {
            // Add the work in the queue
            work_struct_set_state(work, WORK_STATE_RUNNING_WAITING);
            (*(t->priv.vtbl->queue_work))(t, work);
            ret = true;
         }
         abstract_workqueue_spin_unlock(t);
      }
      if (ret) {
         (*(t->priv.vtbl->new_work_to_run))(t, work);
      }
   }
   return (ret);
}

static bool abstract_workqueue_wait_work_termination(
   abstract_workqueue_t *t, struct work_struct *work)
{
   bool ret = false;
   do { // Empty loop
      if (WORK_STATE_RUNNING_CANCELLED == work_struct_get_state(work)) {
         // If same thread work can't terminate because it means that
         // function is called from the work
         if (pthread_equal(pthread_self(), work_struct_get_thread_id(work))) {
            abstract_workqueue_spin_lock(t);
            if (WORK_STATE_RUNNING_CANCELLED == work_struct_get_state(work)) {
               // Revert state to WORK_STATE_RUNNING
               work_struct_set_state(work, WORK_STATE_RUNNING);
            }
            abstract_workqueue_spin_unlock(t);
            break;
         }
         do {
            // Wait that works finished execution
            sched_yield();
         } while (WORK_STATE_RUNNING_CANCELLED == work_struct_get_state(work));
      }
      ret = true;
      abstract_workqueue_spin_lock(t);
      if (WORK_STATE_CANCELLED == work_struct_get_state(work)) {
         work_struct_set_state(work, WORK_STATE_IDLE);
         work_struct_clear_owner(work);
         abstract_workqueue_spin_unlock(t);
      }
      else {
         abstract_workqueue_spin_unlock(t);
         bcm_assert(false);
      }
   } while (false);

   return (ret);
}

void abstract_workqueue_init(abstract_workqueue_t *t,
   vtable_abstract_workqueue_t *vtbl)
{
   bcm_pr_debug("%s()\n", __func__);

   t->priv.vtbl = vtbl;

   // Init list of works
   INIT_LIST_HEAD(&(t->priv.works));

   // Spinlock use to access list of works and manipulate state of a work
   spin_lock_init(&(t->priv.spinlock));
}

void abstract_workqueue_prepare_deinit(abstract_workqueue_t *t)
{
   set_bit(WQ_DEINIT_BIT, &(t->priv.flags));
}

void abstract_workqueue_deinit(abstract_workqueue_t *t)
{
   struct work_struct *cur;
   struct work_struct *next;

   bcm_pr_debug("%s()\n", __func__);

   // Prevent queuing of new work
   abstract_workqueue_prepare_deinit(t);

   // Cancel all works in the list
   abstract_workqueue_spin_lock(t);
   list_for_each_entry_safe(cur, next, &(t->priv.works), priv.list) {
      bcm_assert((t == work_struct_get_owner(cur))
         && (WORK_STATE_WAITING == work_struct_get_state(cur)));
      abstract_workqueue_dequeue_work(t, cur);
      work_struct_set_state(cur, WORK_STATE_IDLE);
      work_struct_clear_owner(cur);
   }
   bcm_assert(list_empty(&(t->priv.works)));
   list_del_init(&(t->priv.works));
   abstract_workqueue_spin_unlock(t);
}

bool cancel_work_sync(struct work_struct *work)
{
   bool ret = false;
   abstract_workqueue_t *owner;

   bcm_pr_debug("%s()\n", __func__);

   // Try to lock work's owner
   for (;;) {
      owner = work_struct_get_owner(work);
      if (NULL == owner) {
         // No owner so nothing to do
         break;
      }
      abstract_workqueue_spin_lock(owner);
      if (owner == work_struct_get_owner(work)) {
         break;
      }
      else {
         // Retry
         abstract_workqueue_spin_unlock(owner);
      }
   }
   if (NULL == owner) {
      // No owner so nothing to do
      ret = true;
   }
   else {
      int state = work_struct_get_state(work);
      if (WORK_STATE_WAITING == state) {
         // Remove the work from the list of pending works
         (*(owner->priv.vtbl->dequeue_work))(owner, work);
         work_struct_set_state(work, WORK_STATE_IDLE);
         work_struct_clear_owner(work);
         abstract_workqueue_spin_unlock(owner);
         ret = true;
      }
      else if (WORK_STATE_RUNNING_WAITING == state) {
         // Remove the work from the list of pending works
         (*(owner->priv.vtbl->dequeue_work))(owner, work);
         work_struct_set_state(work, WORK_STATE_RUNNING_CANCELLED);
         abstract_workqueue_spin_unlock(owner);
         // Wait for termination
         ret = abstract_workqueue_wait_work_termination(owner, work);
      }
      else if (WORK_STATE_RUNNING == state) {
         work_struct_set_state(work, WORK_STATE_RUNNING_CANCELLED);
         abstract_workqueue_spin_unlock(owner);
         // Wait for termination
         ret = abstract_workqueue_wait_work_termination(owner, work);
      }
      else {
         abstract_workqueue_spin_unlock(owner);
         bcm_assert(false);
      }
   }
   return (ret);
}

enum {
   WQ_PROCESSING_BIT = WQ_LAST_BIT + 1,
};

struct workqueue_struct {
   struct {
      abstract_workqueue_t wq;
      pthread_t thread_id;
      struct mutex mutex;
      pthread_cond_t cond;
   } priv;
};

typedef struct {
   struct work_struct work;
   volatile bool has_run;
} work_barrier_t;

static void *workqueue_struct_run(void *data)
{
   struct workqueue_struct *t = (struct workqueue_struct *)(data);

   bcm_pr_debug("%s()\n", __func__);

   abstract_workqueue_spin_lock(&(t->priv.wq));
   while (test_bit(WQ_PROCESSING_BIT, &(t->priv.wq.priv.flags))) {
      // Look for a work to run
      struct work_struct *cur;
      struct work_struct *work = NULL;
      list_for_each_entry(cur, &(t->priv.wq.priv.works), priv.list) {
         int state = work_struct_get_state(cur);
         if (WORK_STATE_WAITING == state) {
            work = cur;
            break;
         }
         bcm_assert(WORK_STATE_RUNNING_WAITING == state);
      }
      if (NULL == work) {
         // No work to run, wait for a signal on condition variable
         abstract_workqueue_spin_unlock(&(t->priv.wq));
         mutex_lock_interruptible(&(t->priv.mutex));
         pthread_cond_wait(&(t->priv.cond), &(t->priv.mutex.obj));
         mutex_unlock(&(t->priv.mutex));
         abstract_workqueue_spin_lock(&(t->priv.wq));
      }
      else {
         abstract_workqueue_run_work(&(t->priv.wq), work);
      }
   }
   abstract_workqueue_spin_unlock(&(t->priv.wq));

   bcm_pr_debug("%s() : exiting\n", __func__);

   return (data);
}


static void work_barrier_cb(struct work_struct *work)
{
   work_barrier_t *t = container_of(work, work_barrier_t, work);

   bcm_pr_debug("%s()\n", __func__);

   t->has_run = true;
}

void flush_workqueue(struct workqueue_struct *t)
{
   bcm_pr_debug("%s()\n", __func__);

   if (!pthread_equal(pthread_self(), t->priv.thread_id)) {
      work_barrier_t barrier;

      INIT_WORK(&(barrier.work), work_barrier_cb);
      barrier.has_run = false;

      if (queue_work(t, &(barrier.work))) {
         // Wait that there's no more jobs waiting or running
         while (!barrier.has_run) {
            sched_yield();
         }
      }
   }
}

static void workqueue_struct_queue_work(abstract_workqueue_t *t,
   struct work_struct *work)
{
   // Add the work at the tail of the queue
   bcm_assert(abstract_workqueue_spin_is_locked(t));
   list_del_init(&(work->priv.list));
   list_add_tail(&(work->priv.list), &(t->priv.works));
}

static void workqueue_struct_dequeue_work(abstract_workqueue_t *t,
   struct work_struct *work)
{
   abstract_workqueue_dequeue_work(t, work);
}

static void workqueue_struct_new_work_to_run(abstract_workqueue_t *_t,
   struct work_struct *work)
{
   struct workqueue_struct *t = container_of(_t, struct workqueue_struct, priv.wq);
   pthread_cond_signal(&(t->priv.cond));
}

static vtable_abstract_workqueue_t workqueue_struct_vtbl = {
   .queue_work = workqueue_struct_queue_work,
   .dequeue_work = workqueue_struct_dequeue_work,
   .new_work_to_run = workqueue_struct_new_work_to_run,
};

static inline int workqueue_struct_init(struct workqueue_struct *t, int max_active)
{
   int ret = -1;
   void *dummy;

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert(1 == max_active);

   abstract_workqueue_init(&(t->priv.wq), &(workqueue_struct_vtbl));

   // Create a mutex
   mutex_init(&(t->priv.mutex));

   // Create a condition variable used to suspend / resume the thread
   ret = pthread_cond_init(&(t->priv.cond), NULL);
   if (ret) {
      bcm_pr_err("**** pthread_cond_init() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
      goto fail_create_cond_var;
   }

   // Create the thread that will run the works
   set_bit(WQ_PROCESSING_BIT, &(t->priv.wq.priv.flags));
   ret = pthread_create(&(t->priv.thread_id), NULL, workqueue_struct_run, t);
   if (ret) {
      bcm_pr_err("**** pthread_create() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
      goto fail_create_thread;
   }

   return (0);

   // Destroy thread
   clear_bit(WQ_PROCESSING_BIT, &(t->priv.wq.priv.flags));
   pthread_join(t->priv.thread_id, &(dummy));
   bcm_assert(dummy == t);
fail_create_thread:
   // Destroy condition variable
   pthread_cond_destroy(&(t->priv.cond));
fail_create_cond_var:
   // Destroy mutext
   mutex_destroy(&(t->priv.mutex));
   abstract_workqueue_deinit(&(t->priv.wq));
   return (ret);
}

static inline void workqueue_struct_deinit(struct workqueue_struct *t)
{
   void *dummy;

   bcm_pr_debug("%s()\n", __func__);

   abstract_workqueue_prepare_deinit(&(t->priv.wq));

   // Set flag to end thread loop
   abstract_workqueue_spin_lock(&(t->priv.wq));
   clear_bit(WQ_PROCESSING_BIT, &(t->priv.wq.priv.flags));
   abstract_workqueue_spin_unlock(&(t->priv.wq));
   // Destroy thread
   pthread_cond_broadcast(&(t->priv.cond));
   pthread_join(t->priv.thread_id, &(dummy));
   bcm_assert(dummy == t);

   abstract_workqueue_deinit(&(t->priv.wq));

   // Destroy condition variable
   pthread_cond_destroy(&(t->priv.cond));

   // Destroy mutext
   mutex_destroy(&(t->priv.mutex));
}

struct workqueue_struct *alloc_workqueue(const char *fmt, unsigned int flags, int max_active)
{
   struct workqueue_struct *ret;

   bcm_assert((flags & WQ_UNBOUND) && (flags & WQ_FREEZABLE));

   ret = (struct workqueue_struct *)(kmalloc(sizeof(struct workqueue_struct), GFP_KERNEL | __GFP_ZERO));
   if (workqueue_struct_init(ret, max_active)) {
      kfree(ret);
      ret = NULL;
   }
   return (ret);
}

void destroy_workqueue(struct workqueue_struct *t)
{
   bcm_assert(NULL != t);
   workqueue_struct_deinit(t);
   kfree(t);
}

bool queue_work(struct workqueue_struct *t, struct work_struct *work)
{
   return (abstract_workqueue_queue_work(&(t->priv.wq), work));
}

void workqueue_init(void)
{
   bcm_pr_debug("%s()\n", __func__);
}

void workqueue_deinit(void)
{
   bcm_pr_debug("%s()\n", __func__);
}
