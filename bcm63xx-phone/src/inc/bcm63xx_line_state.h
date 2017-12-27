/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __BCM63XX_LINE_STATE_H__
#define __BCM63XX_LINE_STATE_H__

#include <linux/types.h>
#ifdef __KERNEL__
# include <linux/uaccess.h>
#else /* !__KERNEL__ */
# include <stdbool.h>
# include <string.h>
#endif /* !__KERNEL__ */

#include "bcm63xx_log.h"

typedef enum {
   BCMPH_STATUS_UNSPECIFIED = 0,
   BCMPH_STATUS_ON_HOOK = 1,
   BCMPH_STATUS_OFF_HOOK = 2,
   /* Must be the last constant of the enum */
   BCMPH_MAX_STATUS,
} bcm_phone_line_status_t;

typedef enum {
   BCMPH_CODEC_UNSPECIFIED = 0,
   BCMPH_CODEC_ALAW = 1,
   BCMPH_CODEC_ULAW = 2,
   BCMPH_CODEC_LINEAR = 3,
   BCMPH_CODEC_LINEAR16 = 4,
   BCMPH_CODEC_ALAW16 = 5,
   BCMPH_CODEC_ULAW16 = 6,
   /* Must be the last constant of the enum */
   BCMPH_MAX_CODECS,
} bcm_phone_codec_t;

typedef enum {
   BCMPH_MODE_UNSPECIFIED = 0,
   /* Phone is disconnected */
   BCMPH_MODE_DISCONNECT = 1,
   /* Phone does nothing */
   BCMPH_MODE_IDLE = 2,
   /* Phone is on hook and ringing */
   BCMPH_MODE_ON_RINGING = 3,
   /* Phone is on hook and is sending and receiving voice */
   BCMPH_MODE_ON_TALKING = 4,
   /* Phone is off hook and is sending and receiving voice */
   BCMPH_MODE_OFF_TALKING = 5,
   /* Must be the last constant of the enum */
   BCMPH_MAX_MODES,
} bcm_phone_line_mode_t;

typedef enum {
   BCMPH_TONE_UNSPECIFIED = 0,
   BCMPH_TONE_NONE = 1,
   /*
    Phone is off hook and emits a tone that signals that we are
    waiting for the user to dial one or more digits
   */
   BCMPH_TONE_WAITING_DIAL = 2,
   /* Phone is off hook and emits a tone that signals a problem */
   BCMPH_TONE_INVALID = 3,
   /*
    Phone is off hook and emits a tone that signals that the called
    phone is ringing
   */
   BCMPH_TONE_RINGBACK = 4,
   /*
    Phone is off hook and emits a tone that signals that the called
    phone is busy
   */
   BCMPH_TONE_BUSY = 5,
   /*
    Phone is off hook and emits a tone that signals that the called
    phone has been hooked on
   */
   BCMPH_TONE_DISCONNECT = 6,
   /* Phone is off hook and emits a DTMF tone */
   BCMPH_TONE_DTMF_0 = 7,
   BCMPH_TONE_DTMF_1 = 8,
   BCMPH_TONE_DTMF_2 = 9,
   BCMPH_TONE_DTMF_3 = 10,
   BCMPH_TONE_DTMF_4 = 11,
   BCMPH_TONE_DTMF_5 = 12,
   BCMPH_TONE_DTMF_6 = 13,
   BCMPH_TONE_DTMF_7 = 14,
   BCMPH_TONE_DTMF_8 = 15,
   BCMPH_TONE_DTMF_9 = 16,
   BCMPH_TONE_DTMF_ASTER = 17,
   BCMPH_TONE_DTMF_POUND = 18,
   BCMPH_TONE_DTMF_A = 19,
   BCMPH_TONE_DTMF_B = 20,
   BCMPH_TONE_DTMF_C = 21,
   BCMPH_TONE_DTMF_D = 22,
   /* Must be the last constant of the enum */
   BCMPH_MAX_TONES,
} bcm_phone_line_tone_t;

#define BCMPH_TONE_INDEX_SHIFT 0
#define BCMPH_TONE_INDEX_MASK (0x3F << BCMPH_TONE_INDEX_SHIFT)
#define BCMPH_TONE_ON_TIME_SHIFT 6
#define BCMPH_TONE_ON_TIME_MASK (0x1FFF << BCMPH_TONE_ON_TIME_SHIFT)
#define BCMPH_TONE_OFF_TIME_SHIFT 19
#define BCMPH_TONE_OFF_TIME_MASK (0x1FFF << BCMPH_TONE_OFF_TIME_SHIFT)

static inline __u32 bcm_phone_line_tone_code_index(bcm_phone_line_tone_t index)
{
   __u32 ret = (((__u32)(index)) << BCMPH_TONE_INDEX_SHIFT);
   dd_bcm_pr_debug("%s(index=%d) -> %lu\n", __func__,
      (int)(index), (unsigned long)(ret));
   bcm_assert(0 == (ret & (~(BCMPH_TONE_INDEX_MASK))));
   return (ret);
}

static inline __u32 bcm_phone_line_tone_code(
   bcm_phone_line_tone_t index, __u16 on_time, __u16 off_time)
{
   __u32 ret = (((__u32)(index)) << BCMPH_TONE_INDEX_SHIFT);
   __u32 tmp;
   bcm_assert(0 == (ret & (~(BCMPH_TONE_INDEX_MASK))));
   tmp = (((__u32)(on_time)) << BCMPH_TONE_ON_TIME_SHIFT);
   bcm_assert(0 == (tmp & (~(BCMPH_TONE_ON_TIME_MASK))));
   ret |= tmp;
   tmp = (((__u32)(off_time)) << BCMPH_TONE_OFF_TIME_SHIFT);
   bcm_assert(0 == (tmp & (~(BCMPH_TONE_OFF_TIME_MASK))));
   ret |= tmp;
   dd_bcm_pr_debug("%s(index=%d, on_time=%u, off_time=%u) -> %lu\n", __func__,
      (int)(index), (unsigned int)(on_time), (unsigned int)(off_time),
      (unsigned long)(ret));
   return (ret);
}

static inline bcm_phone_line_tone_t bcm_phone_line_tone_decode_index(__u32 v)
{
   bcm_phone_line_tone_t ret = (bcm_phone_line_tone_t)((v & BCMPH_TONE_INDEX_MASK) >> BCMPH_TONE_INDEX_SHIFT);
   dd_bcm_pr_debug("%s(v=%lu) -> %d\n", __func__, (unsigned long)(v), (int)(ret));
   return (ret);
}

static inline __u16 bcm_phone_line_tone_decode_on_time(__u32 v)
{
   __u16 ret = (__u16)((v & BCMPH_TONE_ON_TIME_MASK) >> BCMPH_TONE_ON_TIME_SHIFT);
   dd_bcm_pr_debug("%s(v=%lu) -> %u\n", __func__, (unsigned long)(v), (unsigned int)(ret));
   return (ret);
}

static inline __u16 bcm_phone_line_tone_decode_off_time(__u32 v)
{
   __u16 ret = (__u16)((v & BCMPH_TONE_OFF_TIME_MASK) >> BCMPH_TONE_OFF_TIME_SHIFT);
   dd_bcm_pr_debug("%s(v=%lu) -> %u\n", __func__, (unsigned long)(v), (unsigned int)(ret));
   return (ret);
}

typedef struct {
   /* The current status of the line */
   bcm_phone_line_status_t status;
   /*
    The number of change of the status since last call of
    BCMPH_IOCTL_GET_LINE_STATE.
    A change in the status is detected if status_change_count > 0
   */
   __u16 status_change_count;
   /* The current codec */
   bcm_phone_codec_t codec;
   __u16 codec_change_count;
   /* The current state of the flag that asks to reverse polarity */
   bool rev_polarity;
   /*
    The number of change of the flag rev_polarity since last call of
    BCMPH_IOCTL_GET_LINE_STATE.
   */
   __u16 rev_polarity_change_count;
   /* The current mode of the line */
   bcm_phone_line_mode_t mode;
   /*
    The number of change of the mode since last call of
    BCMPH_IOCTL_GET_LINE_STATE.
   */
   __u16 mode_change_count;
   /* The current tone emitted */
   bcm_phone_line_tone_t tone;
   /*
    The number of change of the tone since last call of
    BCMPH_IOCTL_GET_LINE_STATE.
   */
   __u16 tone_change_count;
   /*
    The digits detected since last call of BCMPH_IOCTL_GET_LINE_STATE
   */
   char digits[64];
   __u16 digits_count;
   /*
    The number of flash events detected since last call of
    BCMPH_IOCTL_GET_LINE_STATE
   */
   __u16 flash_count;
} bcm_phone_line_state_t;

extern void bcm_phone_line_state_reset(bcm_phone_line_state_t *t,
   bcm_phone_line_status_t status, bcm_phone_codec_t codec,
   bool rev_polarity, bcm_phone_line_mode_t mode,
   bcm_phone_line_tone_t tone);

static inline void bcm_phone_line_state_reset_status_change_count(bcm_phone_line_state_t *t)
{
   t->status_change_count = 0;
}

static inline void bcm_phone_line_state_reset_codec_change_count(bcm_phone_line_state_t *t)
{
   t->codec_change_count = 0;
}

static inline void bcm_phone_line_state_reset_rev_polarity_change_count(bcm_phone_line_state_t *t)
{
   t->rev_polarity_change_count = 0;
}

static inline void bcm_phone_line_state_reset_mode_change_count(bcm_phone_line_state_t *t)
{
   t->mode_change_count = 0;
}

static inline void bcm_phone_line_state_reset_tone_change_count(bcm_phone_line_state_t *t)
{
   t->tone_change_count = 0;
}

static inline void bcm_phone_line_state_reset_flash_count(bcm_phone_line_state_t *t)
{
   t->flash_count = 0;
}

static inline char bcm_phone_line_state_peek_digit(bcm_phone_line_state_t *t)
{
   char ret = '\0';
   if (t->digits_count > 0) {
      ret = t->digits[0];
   }
   return (ret);
}

static inline char bcm_phone_line_state_pop_digit(bcm_phone_line_state_t *t)
{
   char ret = '\0';
   if (t->digits_count > 0) {
      ret = t->digits[0];
      t->digits_count -= 1;
      if (t->digits_count > 0) {
         memmove(t->digits, &(t->digits[1]), t->digits_count * sizeof(char));
      }
   }
   return (ret);
}

static inline void bcm_phone_line_state_reset_digits(bcm_phone_line_state_t *t)
{
   t->digits_count = 0;
}

static inline void bcm_phone_line_state_reset_change_counts(bcm_phone_line_state_t *t)
{
   bcm_phone_line_state_reset_status_change_count(t);
   bcm_phone_line_state_reset_codec_change_count(t);
   bcm_phone_line_state_reset_rev_polarity_change_count(t);
   bcm_phone_line_state_reset_mode_change_count(t);
   bcm_phone_line_state_reset_tone_change_count(t);
}

static inline void bcm_phone_line_state_init(bcm_phone_line_state_t *t)
{
   bcm_phone_line_state_reset(t, BCMPH_STATUS_UNSPECIFIED,
      BCMPH_CODEC_LINEAR, false, BCMPH_MODE_DISCONNECT, BCMPH_TONE_NONE);
}

static inline void bcm_phone_line_state_deinit(bcm_phone_line_state_t *t)
{
   bcm_phone_line_state_reset(t, BCMPH_STATUS_UNSPECIFIED,
      BCMPH_CODEC_LINEAR, false, BCMPH_MODE_DISCONNECT, BCMPH_TONE_NONE);
}

extern void bcm_phone_line_state_move(bcm_phone_line_state_t *t, bcm_phone_line_state_t *dest);

static inline int bcm_phone_line_state_has_changes(const bcm_phone_line_state_t *t)
{
   if ((t->status_change_count) || (t->codec_change_count)
       || (t->rev_polarity_change_count) || (t->mode_change_count)
       || (t->tone_change_count) || (t->digits_count)
       || (t->flash_count)) {
      return (1);
   }
   else {
      return (0);
   }
}

#endif /* __BCM63XX_LINE_STATE_H__ */
