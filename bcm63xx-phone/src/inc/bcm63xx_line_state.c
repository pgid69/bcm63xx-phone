/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@gilande.com>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifdef __KERNEL__
#include <linux/bug.h>
#include <linux/kernel.h>
#include <linux/string.h>
#else /* !__KERNEL__ */
#include <stddef.h>
#include <string.h>
#ifndef ARRAY_SIZE
#define ARRAY_SIZE(tab) (sizeof(tab) / sizeof(tab[0]))
#endif /* !ARRAY_SIZE */
#endif /* !__KERNEL__ */

#include "bcm63xx_line_state.h"

void bcm_phone_line_state_reset(bcm_phone_line_state_t *t,
   bcm_phone_line_status_t status, bcm_phone_codec_t codec,
   bcm_phone_line_mode_t mode, bcm_phone_line_tone_t tone)
{
   t->codec = codec;
   t->status = status;
   t->mode = mode;
   t->tone = tone;
   bcm_phone_line_state_reset_change_counts(t);
   bcm_phone_line_state_reset_flash_count(t);
   bcm_phone_line_state_reset_digits(t);
}

void bcm_phone_line_state_move(bcm_phone_line_state_t *t, bcm_phone_line_state_t *dest)
{
   size_t len;

   dest->codec = t->codec;
   dest->codec_change_count += t->codec_change_count;
   t->codec_change_count = 0;
   dest->status = t->status;
   dest->status_change_count += t->status_change_count;
   t->status_change_count = 0;
   dest->mode = t->mode;
   dest->mode_change_count += t->mode_change_count;
   t->mode_change_count = 0;
   dest->tone = t->tone;
   dest->tone_change_count += t->tone_change_count;
   t->tone_change_count = 0;
   len = t->digits_count;
   if (len > (ARRAY_SIZE(dest->digits) - dest->digits_count)) {
      len = ARRAY_SIZE(dest->digits) - dest->digits_count;
   }
   if (len > 0) {
      memcpy(&(dest->digits[dest->digits_count]), t->digits, len);
      dest->digits_count += len;
      t->digits_count -= len;
      if (t->digits_count > 0) {
         memmove(t->digits, &(t->digits[len]), t->digits_count * sizeof(char));
      }
   }
   dest->flash_count += t->flash_count;
   t->flash_count = 0;
}
