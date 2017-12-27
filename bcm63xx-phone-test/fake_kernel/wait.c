/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <config.h>

#include <extern/linux/wait.h>

#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#include <bcm63xx_log.h>

void init_waitqueue_head(wait_queue_head_t *t)
{
   int ret;

   bcm_pr_debug("%s()\n", __func__);

   INIT_LIST_HEAD(&(t->priv.wait_queues));
   mutex_init(&(t->priv.mutex));
   ret = pthread_cond_init(&(t->priv.cond), NULL);
   if (ret) {
      bcm_pr_err("**** pthread_cond_init() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
      exit(ret);
   }
}

void deinit_waitqueue_head(wait_queue_head_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   mutex_lock_interruptible(&(t->priv.mutex));
   bcm_assert(list_empty(&(t->priv.wait_queues)));
   list_del_init(&(t->priv.wait_queues));
   mutex_unlock(&(t->priv.mutex));
   pthread_cond_destroy(&(t->priv.cond));
   mutex_destroy(&(t->priv.mutex));
}
