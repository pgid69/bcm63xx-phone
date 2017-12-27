/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_UACCESS_H__
#define __FK_LINUX_UACCESS_H__

#include <fake_kernel_compile.h>

#include <string.h>

static inline int copy_from_user(void *dest, void *src, size_t s)
{
   memcpy(dest, src, s);
   return (0);
}

static inline int copy_to_user(void *dest, void *src, size_t s)
{
   memcpy(dest, src, s);
   return (0);
}

#define VERIFY_READ	0
#define VERIFY_WRITE	1

static inline int access_ok(int type, const void *addr, unsigned long size)
{
   return (1);
}

#endif // __FK_LINUX_UACCESS_H__

