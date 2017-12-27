/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <config.h>

#include <extern/linux/mutex.h>

#include <stdlib.h>
#include <string.h>

#include <bcm63xx_log.h>

void mutex_init(struct mutex *t)
{
   int ret;

   bcm_pr_debug("%s()\n", __func__);

   ret = pthread_mutexattr_init(&(t->priv.attr));
   if (ret) {
      bcm_pr_err("**** pthread_mutexattr_init() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
      exit(ret);
   }
   ret = pthread_mutexattr_settype(&(t->priv.attr), PTHREAD_MUTEX_ERRORCHECK);
   if (ret) {
      bcm_pr_err("**** pthread_mutexattr_settype() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
      exit(ret);
   }
   ret = pthread_mutex_init(&(t->obj), &(t->priv.attr));
   if (ret) {
      bcm_pr_err("**** pthread_mutexattr_init() failed with error code %d : '%s' ****\n",
         (int)(ret), strerror(ret));
      exit(ret);
   }
}
