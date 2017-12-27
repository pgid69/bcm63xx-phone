/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_LOG2_H__
#define __FK_LINUX_LOG2_H__

#include <fake_kernel_compile.h>

static inline bool is_power_of_2(size_t val)
{
   return ((~((val - 1) ^ (~(val - 1)))) == 0);
}

#endif // __FK_LINUX_LOG2_H__

