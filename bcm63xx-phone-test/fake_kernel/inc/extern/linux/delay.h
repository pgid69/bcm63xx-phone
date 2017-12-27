/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_DELAY_H__
#define __FK_LINUX_DELAY_H__

#include <fake_kernel_compile.h>

#include <unistd.h>

static inline void msleep(unsigned long msecs)
{
   usleep(msecs * 1000);
}

#endif // __FK_LINUX_DELAY_H__

