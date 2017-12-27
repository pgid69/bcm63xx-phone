#ifndef __WORKQUEUE_INTERNAL_H__
#define __WORKQUEUE_INTERNAL_H__

#include <fake_kernel_compile.h>

#include <extern/linux/list.h>
#include <extern/linux/spinlock.h>
#include <extern/linux/workqueue.h>

#include <bcm63xx_log.h>

enum {
   // Work states

   /*
    Work if not running nor is waiting execution.
    In this state the work is not in the list of pending work of a
    workqueue
    It can only transition to state WORK_STATE_WAITING when passed
    as parameter to queue_work()
    */
   WORK_STATE_IDLE = 0,
   /*
    Work is waiting execution. It is in the list of pending
    works of a workqueue.
    It can transition to states :
    - WORK_STATE_RUNNING when a workqueue's thread removes it from the
      list of pending works.
    - WORK_STATE_IDLE when passed as parameter to cancel_work_sync()
   */
   WORK_STATE_WAITING = 1,
   /*
    Work is running. A workqueue's thread removed the work from the list
    of pending work and is running it.
    It can transition to states :
    - WORK_STATE_IDLE when work finishes execution
    - WORK_STATE_RUNNING_WAITING when passed as parameter to queue_work()
      for the same workqueue
    - WORK_STATE_RUNNING_CANCELLED when passed as parameter to
      cancel_work_sync()
   */
   WORK_STATE_RUNNING = 2,
   /*
    Work is running and has been queued with a call to queue_work()
    while still running.
    It can transition to states :
    - WORK_STATE_WAITING when work finishes execution
    - WORK_STATE_RUNNING_CANCELLED when passed a parameter
      to cancel_work_sync()
   */
   WORK_STATE_RUNNING_WAITING = 3,
   /*
    Work has been cancelled while it was running with a call to
    cancel_work_sync()
    It can transition to WORK_STATE_CANCELLED when work finishes
    execution
   */
   WORK_STATE_RUNNING_CANCELLED = 4,
   /*
    Work has been cancelled while it was running and now has finished
    execution.
    It can only transition to WORK_STATE_IDLE when call to
    cancel_work_sync() (who cancels the work) exits
   */
   WORK_STATE_CANCELLED = 5
};

enum {
   WQ_DEINIT_BIT = 1,

   // Must be the last entry in the enum
   WQ_LAST_BIT,
};

struct vtable_abstract_workqueue;

typedef struct {
   struct {
      const struct vtable_abstract_workqueue *vtbl;
      unsigned long flags;
      spinlock_t spinlock;
      struct list_head works;
   } priv;
} abstract_workqueue_t;

typedef struct vtable_abstract_workqueue {
   void (*queue_work)(abstract_workqueue_t *t, struct work_struct *work);
   void (*dequeue_work)(abstract_workqueue_t *t, struct work_struct *work);
   void (*new_work_to_run)(abstract_workqueue_t *t, struct work_struct *work);
} vtable_abstract_workqueue_t;

extern void abstract_workqueue_init(abstract_workqueue_t *t,
   vtable_abstract_workqueue_t *vtbl);

extern void abstract_workqueue_prepare_deinit(abstract_workqueue_t *t);

extern void abstract_workqueue_deinit(abstract_workqueue_t *t);

static inline void abstract_workqueue_spin_lock(abstract_workqueue_t *t)
{
   spin_lock(&(t->priv.spinlock));
}

static int abstract_workqueue_spin_is_locked(abstract_workqueue_t *t)
{
   return (spin_is_locked(&(t->priv.spinlock)));
}

static inline void abstract_workqueue_spin_unlock(abstract_workqueue_t *t)
{
   spin_unlock(&(t->priv.spinlock));
}

extern bool abstract_workqueue_queue_work(abstract_workqueue_t *t,
   struct work_struct *w);

static inline void abstract_workqueue_dequeue_work(abstract_workqueue_t *t,
   struct work_struct *work)
{
   // Remove the work from que queue
   bcm_assert(spin_is_locked(&(t->priv.spinlock)));
   list_del_init(&(work->priv.list));
}

extern void abstract_workqueue_run_work(abstract_workqueue_t *t,
   struct work_struct *work);

static inline abstract_workqueue_t *work_struct_get_owner(struct work_struct *t)
{
   return ((abstract_workqueue_t *)(__sync_val_compare_and_swap(&(t->priv.owner), 0, 0)));
}

static inline int work_struct_get_state(const struct work_struct *t)
{
   return (t->priv.state);
}

#endif // __WORKQUEUE_INTERNAL_H__
