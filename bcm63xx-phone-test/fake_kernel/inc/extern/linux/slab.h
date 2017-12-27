/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_SLAB_H__
#define __FK_LINUX_SLAB_H__

#include <fake_kernel_compile.h>

#include <stdlib.h>
#include <string.h>

#include <bcm63xx_log.h>

#define __GFP_COMP 0
#define __GFP_NORETRY 0
#define __GFP_ZERO		0x0001u

#define GFP_DMA 0
#define GFP_KERNEL 0

static inline void *kmalloc(size_t size, unsigned int flags)
{
   void *ret = malloc(size);
   if ((NULL != ret) && ((flags & __GFP_ZERO))) {
      memset(ret, 0, size);
   }
   return (ret);
}

static inline void *kzalloc(size_t size, unsigned int flags)
{
	return kmalloc(size, flags | __GFP_ZERO);
}

static inline void *kcalloc(size_t n, size_t size, unsigned int flags)
{
   return (kmalloc(n * size, flags));
}

static inline void kfree(void *ptr)
{
   bcm_assert(NULL != ptr);
   free(ptr);
}

#endif // __FK_LINUX_SLAB_H__

