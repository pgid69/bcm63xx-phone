/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_KERNEL_H__
#define __FK_LINUX_KERNEL_H__

#include <fake_kernel_compile.h>

#include <limits.h>
#include <stdlib.h>

/*
 * min()/max()/clamp() macros that also do
 * strict type-checking.. See the
 * "unnecessary" pointer comparison.
 */
#define min(x, y) ({ \
   typeof(x) _min1 = (x); \
   typeof(y) _min2 = (y); \
   (void) (&_min1 == &_min2); \
   _min1 < _min2 ? _min1 : _min2; })

#define max(x, y) ({ \
   typeof(x) _max1 = (x); \
   typeof(y) _max2 = (y); \
   (void) (&_max1 == &_max2); \
   _max1 > _max2 ? _max1 : _max2; })

#define simple_strtol strtol

#endif // __FK_LINUX_KERNEL_H__
