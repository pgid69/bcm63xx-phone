/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@gilande.com>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifndef __UTILS_H__
#define __UTILS_H__

#include "config.h"

#include <linux/types.h>

#ifndef BCMPH_NOHW

// Simple macro to get pseudo TimeStamp Counter on MIPS processor
#define rdtscl(dest) \
   __asm__ __volatile__("mfc0 %0,$9; nop" : "=r" (dest))

#endif // BCMPH_NOHW

// Boundary must be a power of 2
static inline size_t round_up_to_pow_of_2(size_t addr, size_t boundary)
{
   bcm_assert((~((boundary - 1) ^ (~(boundary - 1)))) == 0);
#ifdef __KERNEL__
   return (round_up(addr, boundary));
#else // !__KERNEL__
   return ((addr + boundary - 1) & (~(boundary - 1)));
#endif // !__KERNEL__
}

static inline size_t round_down_to_pow_of_2(size_t addr, size_t boundary)
{
   bcm_assert((~((boundary - 1) ^ (~(boundary - 1)))) == 0);
#ifdef __KERNEL__
   return (round_down(addr, boundary));
#else // !__KERNEL__
   return (addr & (~(boundary - 1)));
#endif // !__KERNEL__
}

static inline size_t round_up_generic(size_t addr, size_t boundary)
{
   return (((addr + boundary - 1) / boundary) * boundary);
}

static inline size_t round_down_generic(size_t addr, size_t boundary)
{
   return ((addr / boundary) * boundary);
}

static inline size_t round_up_to_next_pow_of_2(size_t addr)
{
   size_t pow = 0;
   if (addr > 0) {
      addr -= 1;
      while (0 != addr) {
         addr >>= 1;
         pow += 1;
      }
   }
   return (1 << pow);
}


#endif // __UTILS_H__
