/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#include <config.h>

#include <wait_queue.h>

int bcm_wait_queue_wait_event_counter(bcm_wait_queue_t *t, int counter, bcmph_mutex_t *lock)
{
   int ret;

   dd_bcm_pr_debug("bcm_wait_queue_wait_event_counter()\n");

   do { // Empty loop
      if (NULL != lock) {
         bcm_assert(bcmph_mutex_is_locked(lock));
         bcmph_mutex_unlock(lock);
      }
      ret = wait_event_interruptible(t->wq, (atomic_read(&(t->counter)) != counter));
      if (0 != ret) {
         if (-ERESTARTSYS == ret) {
            d_bcm_pr_debug("wait_event_interruptible() has been interupted !\n");
         }
         else {
            d_bcm_pr_err("wait_event_interruptible() failed !\n");
         }
         break;
      }
      if (NULL != lock) {
         if (bcmph_mutex_lock_interruptible(lock)) {
            ret = -ERESTARTSYS;
            break;
         }
      }
   } while (0);
   return (ret);
}

int bcm_wait_queue_wait_event_counter_timeout(bcm_wait_queue_t *t, int counter,
   long timeout_in_jiffies, bcmph_mutex_t *lock)
{
   int ret;

   dd_bcm_pr_debug("bcm_wait_queue_wait_event_counter_timeout()\n");

   do { // Empty loop
      if (NULL != lock) {
         bcm_assert(bcmph_mutex_is_locked(lock));
         bcmph_mutex_unlock(lock);
      }
      ret = wait_event_interruptible_timeout(t->wq, (atomic_read(&(t->counter)) != counter), timeout_in_jiffies);
      if (ret < 0) {
         if (-ERESTARTSYS == ret) {
            dd_bcm_pr_debug("wait_event_interruptible_timeout() has been interupted !\n");
         }
         else {
            d_bcm_pr_err("wait_event_interruptible_timeout() failed !\n");
         }
         break;
      }
      if (NULL != lock) {
         if (bcmph_mutex_lock_interruptible(lock)) {
            ret = -ERESTARTSYS;
            break;
         }
      }
   } while (0);
   return (ret);
}
