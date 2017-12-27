/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_STDDEF_H__
#define __FK_LINUX_STDDEF_H__

#include <fake_kernel_compile.h>

#include <stddef.h>

#define ARRAY_SIZE(tab) (sizeof(tab) / sizeof(tab[0]))

#define container_of(ptr, type, member) ({			\
	const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (char *)__mptr - offsetof(type,member) );})

#endif // __FK_LINUX_STDDEF_H__

