/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_MM_H__
#define __FK_LINUX_MM_H__

#include <fake_kernel_compile.h>

#include <extern/linux/slab.h>

#define PAGE_SIZE 4096UL

#define __get_free_pages(flags, mm_order) ((unsigned long)(kmalloc(PAGE_SIZE << mm_order, GFP_KERNEL)))

#define free_pages(ptr, mm_order) kfree((void *)(ptr))

static inline int get_order(size_t len)
{
   int ret = 0;
   while ((PAGE_SIZE << ret) < len) {
      ret += 1;
   }
   return (ret);
}

#endif // __FK_LINUX_MM_H__
