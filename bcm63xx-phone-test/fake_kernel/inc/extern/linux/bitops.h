/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_BITOPS_H__
#define __FK_LINUX_BITOPS_H__

#include <fake_kernel_compile.h>

static inline int test_and_clear_bit(int bit, unsigned long *addr)
{
   return (__sync_fetch_and_and(addr, (~(1UL << bit))));
}

static inline int test_and_set_bit(int bit, unsigned long *addr)
{
   return (__sync_fetch_and_or(addr, 1UL << bit));
}

static inline void clear_bit(int bit, unsigned long *addr)
{
   test_and_clear_bit(bit, addr);
}

static inline void set_bit(int bit, unsigned long *addr)
{
   test_and_set_bit(bit, addr);
}

static inline int test_bit(int bit, unsigned long *addr)
{
   return ((*addr & (1UL << bit)));
}

static inline int fls(int x)
{
   return ((x) ? sizeof(x) * 8 - __builtin_clz(x) : 0);
}

#endif // __FK_LINUX_BITOPS_H__

