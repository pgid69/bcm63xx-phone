/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_FILE_H__
#define __FK_LINUX_FILE_H__

#include <fake_kernel_compile.h>

struct inode {
   int dummy;
};

struct file {
   void *private_data;
   int f_flags;
};

static inline int nonseekable_open(struct inode *n, struct file *f)
{
   return (0);
}

#endif // __FK_LINUX_FILE_H__

