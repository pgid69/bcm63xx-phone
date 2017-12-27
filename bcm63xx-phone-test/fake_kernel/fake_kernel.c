/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <config.h>

#include <extern/linux/jiffies.h>
#include <extern/linux/timer.h>

#include <bcm63xx_log.h>

#include <fake_kernel.h>

void fake_kernel_init(void)
{
   bcm_pr_debug("%s()\n", __func__);

   jiffies_init();
   timer_init();
   workqueue_init();
}

void fake_kernel_deinit(void)
{
   bcm_pr_debug("%s()\n", __func__);

   workqueue_deinit();
   timer_deinit();
   jiffies_deinit();
}
