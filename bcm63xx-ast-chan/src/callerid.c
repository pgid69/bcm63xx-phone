/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <linux/types.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <asterisk.h>
#include <asterisk/logger.h>
#include <asterisk/strings.h>

#include <bcm63xx_phone.h>
#include <macros.h>

#include "logger.h"
#include "tone_generation.h"
#include "callerid.h"

/* For specs see
 ETSI 300 659-1, 300 659-3 and 300 659-3
 http://what-when-how.com/voip/fsk-caller-id-on-pstn-voip/
 http://what-when-how.com/voip/dtmf-based-caller-id-voip/
 */

/*
  According to CID type (onhook=Type1, Offhook=Type2), different
   Mark/Seizure configuration apply according to specifications:
   GR-30-CORE (Bell) : Type 1 (Onhook)  => Seizure = 300 bits, Mark = 180 bits
                Type 2 (Offhook) => Seizure = 0 bits, Mark = 80 bits
   ETSI       : Type 1 (Onhook)  => Seizure = 300 bits, Mark = 180+/-25 bits
                Type 2 (Offhook) => Seizure = 0 bits, Mark =  80+/-25bits
   SIN227     : Type 1 (Onhook)  => Seizure = 96 to 315 bits, Mark >= 55 bits
                Type 2 (Offhook) => Seizure = 0 bits, Mark >= 55 bits

   To meet all these specifications, following is set as default:
      Type 1 (Onhook)  => Seizure = 300 bits, Mark = 180 bits
      Type 2 (Offhook) => Seizure =   0 bits, Mark = 80 bits

   Note: for CID2 there is no channel seizure neccessary. So the the seizure is
   always 0 by definition and hard coded in the LL drivers.
*/

enum {
   CID_MAX_NUMBER_LENGTH = 20,
   CID_MAX_NAME_LENGTH = 20,

   CID_DTMF_BEFORE_DATA_DELAY_RING_PULSE = 600,
   CID_DTMF_BEFORE_DATA_DELAY_POLARITY = 150,
   CID_DTMF_BEFORE_DATA_DELAY_RING = 600,
   CID_DTMF_ON_DELAY = 70,
   CID_DTMF_OFF_DELAY = 70,
   CID_DTMF_AFTER_DATA_DELAY_BEFORE_FIRST_RING = 350,
   CID_DTMF_AFTER_DATA_DELAY_RING = 350,
   CID_DTMF_VOLUME = 7219, /* Values come from Asterisk code. Default to -8db. */

   CID_DTAS_VOLUME = 2048, /* Value comes from Asterisk code */

   CID_FSK_BITRATE_SEC = 1200, /* 1200 bps */
   CID_FSK_BEFORE_DATA_DELAY_POLARITY = 150, /* Should be >= 100ms */
   CID_FSK_DTAS_DELAY = 80, /* CID_FSK_BEFORE_DATA_DELAY_POLARITY + CID_FSK_DTAS_DELAY + CID_FSK_AFTER_DTAS_DELAY should be <= 700ms */
   CID_FSK_AFTER_DTAS_DELAY = 100, /* Should be between 45 and 500ms */
   CID_FSK_BEFORE_DATA_DELAY_RING_PULSE = 600, /* Should be between 500 and 800ms */
   CID_FSK_BEFORE_DATA_DELAY_RING = 600, /* Should be between 500 and 2000ms */
   CID_FSK_CHANNEL_SEIZURE_BYTES = 30, /* 300 bits counting start and stop bits */
   CID_FSK_MARK_SIGNAL_BEFORE_BITS = 180,
   CID_FSK_MARK_SIGNAL_AFTER_BITS = 60, /* Optional, useful if type of message is GDMF */
   CID_FSK_AFTER_DATA_DELAY_BEFORE_FIRST_RING = 300, /* Should be between 200 and 500ms */
   CID_FSK_AFTER_DATA_DELAY_RING = 300, /* Should be between 200 and 500ms */

   CID_FSK_BELL_MARK_FREQ = 1200,
   CID_FSK_BELL_SPACE_FREQ = 2200,

   CID_FSK_V23_MARK_FREQ = 1300,
   CID_FSK_V23_SPACE_FREQ = 2100,
};

static size_t bcmph_callerid_sig_dtmf_genmsg(const char *number,
   int flags, __u8 *buffer, size_t buffer_len)
{
   size_t number_idx;
   size_t ret = 0;

   bcm_assert((NULL != buffer) && (buffer_len >= 4));

   /* Start tone */
   buffer[0] = 'A';
   ret = 1;

   if ((!(flags & (BCMPH_CID_PRIVATE_NUMBER | BCMPH_CID_UNKNOWN_NUMBER)))
       && (NULL != number)) {
      bool invalid_characters = false;
      /* Checks that all characters are digits */
      for (number_idx = 0; ('\0' != number[number_idx]); number_idx += 1) {
         if ((isdigit(number[number_idx])) || ('*' == number[number_idx]) || ('#' == number[number_idx])) {
            if ((ret + 1) >= buffer_len) {
               bcm_pr_warn("CID number '%s' is too long. Only first %u digits are used.\n",
                  number, (unsigned int)(buffer_len - 2));
               break;
            }
            buffer[ret] = number[number_idx];
            ret += 1;
            continue;
         }
         if (isspace(number[number_idx])) {
            continue;
         }
         else {
            invalid_characters = true;
         }
      }
      if (invalid_characters) {
         bcm_pr_warn("CID number '%s' contains invalid character(s). Only digits [0-9] are used.\n", number);
      }
   }

   if (ret <= 1) {
      buffer[0] = 'B';
      if (flags & BCMPH_CID_PRIVATE_NUMBER) {
         /*
          Page http://www.epanorama.net/documents/telecom/cid_dtmf.html
          at section 7.1.2 states that private number is encoded with '10'
          '01' is the code mentionned in ETSI standard
          */
         buffer[1] = '0';
         buffer[2] = '1';
      }
      else {
         buffer[1] = '0';
         buffer[2] = '0';
      }
      ret = 3;
   }

   /* Stop tone */
   buffer[ret] = 'C';
   ret += 1;

   return (ret);
}

/* From Asterisk callerid.c */

static size_t bcmph_callerid_sig_fsk_genmsg(const char *name, const char *number,
   int flags, __u8 *buffer, size_t buffer_len)
{
   size_t ret = 0;

   bcm_assert(NULL != buffer);

   /*
    Checks that buffer can contain number and date before writing date
    (because number is mandatory but date is optional)
   */
   if ((buffer_len - ret) >= (10 + (CID_MAX_NUMBER_LENGTH + 2))) {
      /* Format time and message header */
      int res;
      struct timeval now = ast_tvnow();
      struct ast_tm tm;

      /* Get the time */
#ifdef AST_VERSION
      ast_localtime(&(now), &(tm), NULL);
#else
      memset(&(tm), 0, sizeof(tm));
#endif

      *buffer = '\001';
      buffer += 1;
      *buffer = '\010';
      buffer += 1;
      ret += 2;
      res = snprintf((char *)(buffer), buffer_len - ret, "%02d%02d%02d%02d",
         (int)(tm.tm_mon + 1), (int)(tm.tm_mday),
         (int)(tm.tm_hour), (int)(tm.tm_min));
      bcm_assert(8 == res);
      buffer += res;
      ret += res;
   }

   if ((buffer_len - ret) >= 3) {
      /* Format number */
      size_t count = 0;

      if ((!(flags & (BCMPH_CID_PRIVATE_NUMBER | BCMPH_CID_UNKNOWN_NUMBER)))
          && (NULL != number)) {
         bool invalid_characters = false;
         /* Checks that all characters are valid */
         for (; ('\0' != *number); number += 1) {
            char c = *number;
            if ((isdigit(c)) || ('*' == c) || ('#' == c)
                || (' ' == c) || ('-' == c) || ('(' == c) || (')' == c)) {
               if ((count >= CID_MAX_NUMBER_LENGTH) || ((ret + count + 2) >= buffer_len)) {
                  bcm_pr_warn("CID number '%s' is too long. Only first %u digits are used.\n",
                     number, (unsigned int)(count));
                  break;
               }
               buffer[count + 2] = c;
               count += 1;
            }
            else {
               invalid_characters = true;
            }
         }
         if (invalid_characters) {
            bcm_pr_warn("CID number '%s' contains invalid character(s). Only digits [0-9] + characters '*', '#', ' ', '-', '(' and ')' are used.\n", number);
         }
      }
      if (count > 0) {
         *buffer = '\002';
         buffer += 1;
         *buffer = count;
         buffer += (1 + count);
         ret += (count + 2);
      }
      else {
         *buffer = '\004';
         buffer += 1;
         *buffer = '\001';
         buffer += 1;
         if (flags & BCMPH_CID_PRIVATE_NUMBER) {
            /* Indicate number is private */
            *buffer = 'P';
         }
         else {
            /* Indicate number is unknown */
            *buffer = 'O';
         }
         buffer += 1;
         ret += 3;
      }
   }

   if ((buffer_len - ret) >= 3) {
      /* Format name */
      size_t count = 0;

      if ((!(flags & (BCMPH_CID_PRIVATE_NAME | BCMPH_CID_UNKNOWN_NAME)))
          && (NULL != name)) {
         bool invalid_characters = false;
         /* Checks that all characters are valid */
         for (; ('\0' != *name); name += 1) {
            char c = *name;
            if ((c >= 32) && (c < 127)) {
               if ((count >= CID_MAX_NAME_LENGTH) || ((ret + count + 2) >= buffer_len)) {
                  bcm_pr_warn("CID name '%s' is too long. Only first %u characters are used.\n",
                     name, (unsigned int)(count));
                  break;
               }
               buffer[count + 2] = c;
               count += 1;
            }
            else {
               invalid_characters = true;
            }
         }
         if (invalid_characters) {
            bcm_pr_warn("CID name '%s' contains invalid character(s).\n", number);
         }
      }
      if (count > 0) {
         *buffer = '\007';
         buffer += 1;
         *buffer = count;
         buffer += (1 + count);
         ret += (count + 2);
      }
      else {
         *buffer = '\010';
         buffer += 1;
         *buffer = '\001';
         buffer += 1;
         if (flags & BCMPH_CID_PRIVATE_NUMBER) {
            /* Indicate name is private */
            *buffer = 'P';
         }
         else {
            /* Indicate name is unknown */
            *buffer = 'O';
         }
         buffer += 1;
         ret += 3;
      }
   }

   return (ret);
}

static inline void bcmph_fsk_modulator_init(
   bcmph_fsk_modulator_t *t, int frequency_0, int frequency_1,
   int fsk_bit_rate, int sample_rate, __s16 vol)
{
   t->sample_rate = sample_rate;
   t->current.quot = 0;
   t->current.rem = 0;
   t->inc[0] = div(frequency_0 * BCMPH_TRIGINT_ANGLES_PER_CYCLE, sample_rate);
   t->inc[1] = div(frequency_1 * BCMPH_TRIGINT_ANGLES_PER_CYCLE, sample_rate);
   bcmph_fraction_seq_init(&(t->counter), 0, fsk_bit_rate, sample_rate);
   t->vol = vol;
}

static inline int bcmph_fsk_modulator_get_val(bcmph_fsk_modulator_t *t)
{
   return ((bcmph_trigint_sin16(t->current.quot) * t->vol) >> BCMPH_TRIGINT_LOG2_FACTOR_FLOAT_TO_INT);
}

static inline void bcmph_fsk_modulator_next_val(
   bcmph_fsk_modulator_t *t, size_t index)
{
   t->current.quot += t->inc[index].quot;
   t->current.rem += t->inc[index].rem;
   if (t->current.rem >= t->sample_rate) {
      t->current.quot += 1;
      t->current.rem -= t->sample_rate;
   }
   t->current.quot &= BCMPH_TRIGINT_ANGLE_MAX;
}

static inline size_t bcmph_fsk_modulator_write_bit(bcmph_fsk_modulator_t *t,
   size_t bit, __s16 *buffer)
{
   size_t ret = 0;
   for (;;) {
      bcmph_fsk_modulator_next_val(t, bit);
      *buffer = (__s16)(bcmph_fsk_modulator_get_val(t));
      ret += 1;
      buffer += 1;
      bcmph_fraction_seq_next_val(&(t->counter));
      if (t->counter.current.quot) {
         t->counter.current.quot = 0;
         break;
      }
   }
   return (ret);
}

static inline size_t bcmph_fsk_modulator_write_byte(
   bcmph_fsk_modulator_t *fsk_modulator, __u8 byte, __s16 *buffer)
{
   size_t ret = 0;
   size_t tmp;
   size_t bit_idx;

   tmp = bcmph_fsk_modulator_write_bit(fsk_modulator, 0, buffer);
   ret += tmp;
   buffer += tmp;
   for (bit_idx = 0; (bit_idx < 8); bit_idx += 1) {
      tmp = bcmph_fsk_modulator_write_bit(fsk_modulator, byte & 1, buffer);
      ret += tmp;
      buffer += tmp;
      byte >>= 1;
   }
   tmp = bcmph_fsk_modulator_write_bit(fsk_modulator, 1, buffer);
   ret += tmp;
   buffer += tmp;

   return (ret);
}

static size_t bcmph_callerid_generator_get_min_buffer_len(
   const bcmph_sound_generator_t *_t)
{
   return ((((10 * BCMPH_SAMPLE_RATE) + CID_FSK_BITRATE_SEC - 1) / CID_FSK_BITRATE_SEC));
}

static size_t bcmph_callerid_generator_write(bcmph_sound_generator_t *_t
   , __s16 *buffer, size_t len)
{
   bcmph_callerid_generator_t *t = container_of(_t, bcmph_callerid_generator_t, sound_generator);

   bcm_assert(len >= bcmph_callerid_generator_get_min_buffer_len(_t));

   return ((*(t->write))(t, buffer, len));
}

static void bcmph_callerid_generator_deinit(bcmph_sound_generator_t *_t)
{
   bcmph_callerid_generator_t *t = container_of(_t, bcmph_callerid_generator_t, sound_generator);
   (*(t->dual_tone_generator.sound_generator.vtbl->deinit))(&(t->dual_tone_generator.sound_generator));
}

static bcmph_sound_generator_ops_t ops_callerid_generator = {
   .get_min_buffer_len = bcmph_callerid_generator_get_min_buffer_len,
   .write = bcmph_callerid_generator_write,
   .deinit = bcmph_callerid_generator_deinit,
};

static void bcmph_callerid_init_preamble_fsk(
   bcmph_callerid_generator_t *t, bcmph_cid_start_t cid_start)
{
   t->samples_dtas = 0;
   t->silence_in_samples_before_dtas = 0;
   t->silence_in_samples_before_data = 0;
   switch (cid_start) {
      case BCMPH_CID_START_RING: {
         t->silence_in_samples_before_data = CID_FSK_BEFORE_DATA_DELAY_RING;
         break;
      }
      case BCMPH_CID_START_RING_PULSE: {
         t->silence_in_samples_before_data = CID_FSK_BEFORE_DATA_DELAY_RING_PULSE;
         break;
      }
      case BCMPH_CID_START_POLARITY: {
         t->silence_in_samples_before_data = CID_FSK_BEFORE_DATA_DELAY_POLARITY;
         break;
      }
      case BCMPH_CID_START_POLARITY_DTMF: {
         t->silence_in_samples_before_dtas = CID_FSK_BEFORE_DATA_DELAY_POLARITY;
         // Fall through BCMPH_CID_START_DTMF case
         /* break; */
      }
      case BCMPH_CID_START_DTMF: {
         /* Generate DTMF */
         t->samples_dtas = CID_FSK_DTAS_DELAY;
         t->silence_in_samples_before_data = CID_FSK_AFTER_DTAS_DELAY;
         break;
      }
      default: {
         bcm_assert(0);
         break;
      }
   }
   t->silence_in_samples_before_dtas *= BCMPH_SAMPLES_PER_MS;
   t->samples_dtas *= BCMPH_SAMPLES_PER_MS;
   t->silence_in_samples_before_data *= BCMPH_SAMPLES_PER_MS;
}

static void bcmph_callerid_init_data_fsk(bcmph_callerid_generator_t *t,
   const char *name, const char *number,
   int flags, int frequency_0, int frequency_1)
{
   static const __s16 vol = 8192; /* Value comes from Asterisk code */
   size_t msg_idx;
   __u32 sum;

   /* Build the msg */
   t->msg[0] = 0x80; /* Message type. 0x80 == call set up message */
   bcm_assert(ARRAY_SIZE(t->msg) >= 3);
   t->msg_len = bcmph_callerid_sig_fsk_genmsg(name, number, flags, &(t->msg[2]), ARRAY_SIZE(t->msg) - 3);
   t->msg[1] = t->msg_len;
   t->msg_len += 2;
   /* Calculate checksum */
   sum = 0;
   for (msg_idx = 0; (msg_idx < t->msg_len); msg_idx += 1) {
      sum += t->msg[msg_idx];
   }
   t->msg[t->msg_len] = 256 - (sum & 255);
   t->msg_len += 1;

   t->u.fsk.seizure_bytes = CID_FSK_CHANNEL_SEIZURE_BYTES;
   t->u.fsk.mark_bits_before = CID_FSK_MARK_SIGNAL_BEFORE_BITS;
   t->u.fsk.mark_bits_after = CID_FSK_MARK_SIGNAL_AFTER_BITS;

   bcmph_fsk_modulator_init(&(t->u.fsk.modulator), frequency_0, frequency_1, CID_FSK_BITRATE_SEC, BCMPH_SAMPLE_RATE, vol);
}

static void bcmph_callerid_init_postamble_fsk(
   bcmph_callerid_generator_t *t, bcmph_cid_start_t cid_start)
{
   t->silence_in_samples_after_data = 0;
   switch (cid_start) {
      case BCMPH_CID_START_RING: {
         t->silence_in_samples_after_data = CID_FSK_AFTER_DATA_DELAY_RING;
         break;
      }
      case BCMPH_CID_START_RING_PULSE:
      case BCMPH_CID_START_POLARITY:
      case BCMPH_CID_START_DTMF:
      case BCMPH_CID_START_POLARITY_DTMF: {
         t->silence_in_samples_after_data = CID_FSK_AFTER_DATA_DELAY_BEFORE_FIRST_RING;
         break;
      }
      default: {
         bcm_assert(0);
         break;
      }
   }
   t->silence_in_samples_after_data *= BCMPH_SAMPLES_PER_MS;
}

static void bcmph_callerid_init_preamble_dtmf(
   bcmph_callerid_generator_t *t, bcmph_cid_start_t cid_start)
{
   t->silence_in_samples_before_data = 0;
   switch (cid_start) {
      case BCMPH_CID_START_RING: {
         t->silence_in_samples_before_data = CID_DTMF_BEFORE_DATA_DELAY_RING;
         break;
      }
      case BCMPH_CID_START_RING_PULSE: {
         t->silence_in_samples_before_data = CID_DTMF_BEFORE_DATA_DELAY_RING_PULSE;
         break;
      }
      case BCMPH_CID_START_POLARITY: {
         t->silence_in_samples_before_data = CID_DTMF_BEFORE_DATA_DELAY_POLARITY;
         break;
      }
      default: {
         bcm_assert(0);
         break;
      }
   }
   t->silence_in_samples_before_data *= BCMPH_SAMPLES_PER_MS;
}

static void bcmph_callerid_init_data_dtmf(bcmph_callerid_generator_t *t,
   const char *number, int flags)
{
   /* Build the msg */
   t->msg_len = bcmph_callerid_sig_dtmf_genmsg(number, flags, t->msg, ARRAY_SIZE(t->msg));
}

static void bcmph_callerid_init_postamble_dtmf(
   bcmph_callerid_generator_t *t, bcmph_cid_start_t cid_start)
{
   t->silence_in_samples_after_data = 0;
   switch (cid_start) {
      case BCMPH_CID_START_RING: {
         t->silence_in_samples_after_data = CID_DTMF_AFTER_DATA_DELAY_RING;
         break;
      }
      case BCMPH_CID_START_RING_PULSE:
      case BCMPH_CID_START_POLARITY: {
         t->silence_in_samples_after_data = CID_DTMF_AFTER_DATA_DELAY_BEFORE_FIRST_RING;
         break;
      }
      default: {
         bcm_assert(0);
         break;
      }
   }
   t->silence_in_samples_after_data *= BCMPH_SAMPLES_PER_MS;
}

static void bcmph_callerid_init_data(bcmph_callerid_generator_t *t,
   const char *name, const char *number,
   bcmph_cid_signalling_t cid_signalling, bcmph_cid_start_t cid_start,
   int flags)
{
   switch (cid_signalling) {
      case BCMPH_CID_SIG_BELL: {
         t->generate_dtmf = false;
         bcmph_callerid_init_preamble_fsk(t, cid_start);
         bcmph_callerid_init_data_fsk(t, name, number, flags,
            CID_FSK_BELL_SPACE_FREQ, CID_FSK_BELL_MARK_FREQ);
         bcmph_callerid_init_postamble_fsk(t, cid_start);
         break;
      }
      case BCMPH_CID_SIG_V23: {
         t->generate_dtmf = false;
         bcmph_callerid_init_preamble_fsk(t, cid_start);
         bcmph_callerid_init_data_fsk(t, name, number, flags,
            CID_FSK_V23_SPACE_FREQ, CID_FSK_V23_MARK_FREQ);
         bcmph_callerid_init_postamble_fsk(t, cid_start);
         break;
      }
      case BCMPH_CID_SIG_DTMF: {
         t->generate_dtmf = true;
         bcmph_callerid_init_preamble_dtmf(t, cid_start);
         bcmph_callerid_init_data_dtmf(t, number, flags);
         bcmph_callerid_init_postamble_dtmf(t, cid_start);
         break;
      }
      default: {
         bcm_assert(0);
         break;
      }
   }
}

static size_t bcmph_callerid_write_silence_after_data(
   bcmph_callerid_generator_t *t, __s16 *buffer, size_t len)
{
   size_t ret = 0;

   if (t->pos < t->silence_in_samples_after_data) {
      size_t tmp = t->silence_in_samples_after_data - t->pos;
      if (tmp > len) {
         tmp = len;
      }
      ret = bcmph_write_silence(buffer, tmp);
      t->pos += ret;
   }

   return (ret);
}

static size_t bcmph_callerid_write_marks_bits_after_msg(
   bcmph_callerid_generator_t *t, __s16 *buffer, size_t len)
{
   size_t ret = 0;
   size_t samples_per_bit = (((1 * BCMPH_SAMPLE_RATE) + CID_FSK_BITRATE_SEC - 1) / CID_FSK_BITRATE_SEC);

   while (t->pos < t->u.fsk.mark_bits_after) {
      if (len >= (ret + samples_per_bit)) {
         ret += bcmph_fsk_modulator_write_bit(&(t->u.fsk.modulator), 1, &(buffer[ret]));
         t->pos += 1;
      }
      else {
         break;
      }
   }
   if (t->pos >= t->u.fsk.mark_bits_after) {
      t->write = bcmph_callerid_write_silence_after_data;
      t->pos = 0;
      if (len > ret) {
         ret += (*(t->write))(t, &(buffer[ret]), len - ret);
      }
   }

   return (ret);
}

static size_t bcmph_callerid_write_msg_as_fsk(
   bcmph_callerid_generator_t *t, __s16 *buffer, size_t len)
{
   size_t ret = 0;
   size_t samples_per_byte = (((10 * BCMPH_SAMPLE_RATE) + CID_FSK_BITRATE_SEC - 1) / CID_FSK_BITRATE_SEC);

   while (t->pos < t->msg_len) {
      if (len >= (ret + samples_per_byte)) {
         ret += bcmph_fsk_modulator_write_byte(&(t->u.fsk.modulator), t->msg[t->pos], &(buffer[ret]));
         t->pos += 1;
      }
      else {
         break;
      }
   }
   if (t->pos >= t->msg_len) {
      t->write = bcmph_callerid_write_marks_bits_after_msg;
      t->pos = 0;
      if (len > ret) {
         ret += (*(t->write))(t, &(buffer[ret]), len - ret);
      }
   }

   return (ret);
}


static size_t bcmph_callerid_write_marks_bits_before_msg(
   bcmph_callerid_generator_t *t, __s16 *buffer, size_t len)
{
   size_t ret = 0;
   size_t samples_per_bit = (((1 * BCMPH_SAMPLE_RATE) + CID_FSK_BITRATE_SEC - 1) / CID_FSK_BITRATE_SEC);

   while (t->pos < t->u.fsk.mark_bits_before) {
      if (len >= (ret + samples_per_bit)) {
         ret += bcmph_fsk_modulator_write_bit(&(t->u.fsk.modulator), 1, &(buffer[ret]));
         t->pos += 1;
      }
      else {
         break;
      }
   }
   if (t->pos >= t->u.fsk.mark_bits_before) {
      t->write = bcmph_callerid_write_msg_as_fsk;
      t->pos = 0;
      if (len > ret) {
         ret += (*(t->write))(t, &(buffer[ret]), len - ret);
      }
   }

   return (ret);
}

static size_t bcmph_callerid_write_seizure_bytes(
   bcmph_callerid_generator_t *t, __s16 *buffer, size_t len)
{
   size_t ret = 0;
   size_t samples_per_byte = (((10 * BCMPH_SAMPLE_RATE) + CID_FSK_BITRATE_SEC - 1) / CID_FSK_BITRATE_SEC);

   while (t->pos < t->u.fsk.seizure_bytes) {
      if (len >= (ret + samples_per_byte)) {
         ret += bcmph_fsk_modulator_write_byte(&(t->u.fsk.modulator), 0x55, &(buffer[ret]));
         t->pos += 1;
      }
      else {
         break;
      }
   }
   if (t->pos >= t->u.fsk.seizure_bytes) {
      t->write = bcmph_callerid_write_marks_bits_before_msg;
      t->pos = 0;
      if (len > ret) {
         ret += (*(t->write))(t, &(buffer[ret]), len - ret);
      }
   }

   return (ret);
}

static size_t bcmph_callerid_write_msg_as_dtmfs(
   bcmph_callerid_generator_t *t, __s16 *buffer, size_t len)
{
   size_t ret = 0;

   for (;;) {
      size_t tmp = t->u.dtmf.samples_remaining;
      if (0 == (t->pos % 2)) {
         /* Write tone */
         if (tmp > (len - ret)) {
            tmp = (*(t->dual_tone_generator.sound_generator.vtbl->write))(&(t->dual_tone_generator.sound_generator), &(buffer[ret]), len - ret);
            ret += tmp;
            t->u.dtmf.samples_remaining -= tmp;
            break;
         }
         else {
            tmp = (*(t->dual_tone_generator.sound_generator.vtbl->write))(&(t->dual_tone_generator.sound_generator), &(buffer[ret]), tmp);
            ret += tmp;
            t->u.dtmf.samples_remaining -= tmp;
            bcm_assert(t->u.dtmf.samples_remaining <= 0);
            t->pos += 1;
            t->u.dtmf.samples_remaining = CID_DTMF_OFF_DELAY * BCMPH_SAMPLES_PER_MS;
         }
      }
      else {
         /* Write silence */
         if (tmp > (len - ret)) {
            tmp = bcmph_write_silence(&(buffer[ret]), len - ret);
            ret += tmp;
            t->u.dtmf.samples_remaining -= tmp;
            break;
         }
         else {
            tmp = bcmph_write_silence(&(buffer[ret]), tmp);
            ret += tmp;
            t->u.dtmf.samples_remaining -= tmp;
            bcm_assert(t->u.dtmf.samples_remaining <= 0);
            t->pos += 1;
            if ((t->pos >> 1) < t->msg_len) {
               (*(t->dual_tone_generator.sound_generator.vtbl->deinit))(&(t->dual_tone_generator.sound_generator));
               bcmph_dual_tone_generator_init(&(t->dual_tone_generator), bcmph_get_tone_sequence_for_dtmf(t->msg[t->pos >> 1]), CID_DTMF_VOLUME);
               t->u.dtmf.samples_remaining = CID_DTMF_ON_DELAY * BCMPH_SAMPLES_PER_MS;
            }
            else {
               t->write = bcmph_callerid_write_silence_after_data;
               t->pos = 0;
               if (len > ret) {
                  ret += (*(t->write))(t, &(buffer[ret]), len - ret);
               }
               break;
            }
         }
      }
   }

   return (ret);
}

static size_t bcmph_callerid_write_silence_before_data(
   bcmph_callerid_generator_t *t, __s16 *buffer, size_t len)
{
   size_t ret = 0;

   do { /* Empty loop */
      if (t->pos < t->silence_in_samples_before_data) {
         size_t tmp = t->silence_in_samples_before_data - t->pos;
         if (tmp > len) {
            ret = bcmph_write_silence(buffer, len);
            t->pos += ret;
            break;
         }
         else {
            ret = bcmph_write_silence(buffer, tmp);
            t->pos += ret;
            bcm_assert(t->pos >= t->silence_in_samples_before_data);
         }
      }
      if (t->generate_dtmf) {
         t->write = bcmph_callerid_write_msg_as_dtmfs;
         if (t->msg_len > 0) {
            t->pos = 0;
            (*(t->dual_tone_generator.sound_generator.vtbl->deinit))(&(t->dual_tone_generator.sound_generator));
            bcmph_dual_tone_generator_init(&(t->dual_tone_generator), bcmph_get_tone_sequence_for_dtmf(t->msg[0]), CID_DTMF_VOLUME);
            t->u.dtmf.samples_remaining = CID_DTMF_ON_DELAY * BCMPH_SAMPLES_PER_MS;
         }
         else {
            t->pos = 1;
            t->u.dtmf.samples_remaining = 0;
         }
      }
      else {
         t->write = bcmph_callerid_write_seizure_bytes;
         t->pos = 0;
      }
      if (len > ret) {
         ret += (*(t->write))(t, &(buffer[ret]), len - ret);
      }
   } while (false);

   return (ret);
}

static size_t bcmph_callerid_write_dtas(bcmph_callerid_generator_t *t,
   __s16 *buffer, size_t len)
{
   size_t ret = 0;

   do { /* Empty loop */
      if (t->pos < t->samples_dtas) {
         size_t tmp = t->samples_dtas - t->pos;
         if (tmp > len) {
            ret = (*(t->dual_tone_generator.sound_generator.vtbl->write))(&(t->dual_tone_generator.sound_generator), buffer, len);
            t->pos += ret;
            break;
         }
         else {
            ret = (*(t->dual_tone_generator.sound_generator.vtbl->write))(&(t->dual_tone_generator.sound_generator), buffer, tmp);
            t->pos += ret;
            bcm_assert(t->pos >= t->samples_dtas);
         }
      }
      t->write = bcmph_callerid_write_silence_before_data;
      t->pos = 0;
      if (len > ret) {
         ret += (*(t->write))(t, &(buffer[ret]), len - ret);
      }
   } while (false);

   return (ret);
}

static size_t bcmph_callerid_write_silence_before_dtas(
   bcmph_callerid_generator_t *t, __s16 *buffer, size_t len)
{
   size_t ret = 0;

   do { /* Empty loop */
      if (t->pos < t->silence_in_samples_before_dtas) {
         size_t tmp = t->silence_in_samples_before_dtas - t->pos;
         if (tmp > len) {
            ret = bcmph_write_silence(buffer, len);
            t->pos += ret;
            break;
         }
         else {
            ret = bcmph_write_silence(buffer, tmp);
            t->pos += ret;
            bcm_assert(t->pos >= t->silence_in_samples_before_dtas);
         }
      }
      (*(t->dual_tone_generator.sound_generator.vtbl->deinit))(&(t->dual_tone_generator.sound_generator));
      bcmph_dual_tone_generator_init(&(t->dual_tone_generator), &(bcmph_dtmf_dtas), CID_DTAS_VOLUME);
      t->write = bcmph_callerid_write_dtas;
      t->pos = 0;
      if (len > ret) {
         ret += (*(t->write))(t, &(buffer[ret]), len - ret);
      }
   } while (false);

   return (ret);
}

void bcmph_callerid_generator_init(bcmph_callerid_generator_t *t,
   const char *name, const char *number,
   bcmph_cid_signalling_t cid_signalling, bcmph_cid_start_t cid_start,
   int flags)
{
   bcm_pr_debug("%s(name='%s', number='%s', cid_signalling=%d, cid_start=%d, flags=%d)\n",
      __func__, ((NULL != name) ? name : ""), ((NULL != number) ? number : ""),
      (int)(cid_signalling), (int)(cid_start), (int)(flags));
   memset(t, 0, sizeof(*t));
   t->sound_generator.vtbl = &(ops_callerid_generator);
   bcmph_dual_tone_generator_init(&(t->dual_tone_generator), &(bcmph_dtmf_null), 0);
   bcmph_callerid_init_data(t, name, number, cid_signalling, cid_start, flags);
   t->write = bcmph_callerid_write_silence_before_dtas;
   t->pos = 0;
}

size_t bcmph_callerid_generate(const char *name, const char *number,
   bcmph_cid_signalling_t cid_signalling, bcmph_cid_start_t cid_start,
   int flags, __s16 *buffer, size_t buffer_len)
{
   size_t ret = 0;
   bcmph_callerid_generator_t callerid_generator;

   bcmph_callerid_generator_init(&(callerid_generator),
      name, number, cid_signalling, cid_start, flags);
   ret = (*(callerid_generator.sound_generator.vtbl->write))(&(callerid_generator.sound_generator), buffer, buffer_len);
   (*(callerid_generator.sound_generator.vtbl->deinit))(&(callerid_generator.sound_generator));

   return (ret);
}

#ifdef BCMPH_NOHW

static const char *cid_name = "mazoyer.gilles@omega";
static const char *cid_number = "0635861992";

#if (defined AST_VERSION) && (AST_VERSION >= 110)

#include <asterisk/alaw.h>
#if (AST_VERSION > 110)
#include <asterisk/format_cache.h>
#endif /* (AST_VERSION > 110) */
#include <asterisk/callerid.h>

#if (110 == AST_VERSION)
/*
 If a new format is added, functions init_cache_ast_format must
 be updated
 */
static struct ast_format *ast_format_ulaw;
static struct ast_format *ast_format_alaw;
static struct ast_format *ast_format_slin;
static struct ast_format *ast_format_slin16;
#endif /* (110 == AST_VERSION) */

static void init_cache_ast_format(void)
{
#if (110 == AST_VERSION)
   static struct ast_format format_ulaw;
   static struct ast_format format_alaw;
   static struct ast_format format_slin;
   static struct ast_format format_slin16;
   ast_format_ulaw = ast_getformatbyname("ulaw", &(format_ulaw));
   bcm_assert(NULL != ast_format_ulaw);
   ast_format_alaw = ast_getformatbyname("alaw", &(format_alaw));
   bcm_assert(NULL != ast_format_alaw);
   ast_format_slin = ast_getformatbyname("slin", &(format_slin));
   bcm_assert(NULL != ast_format_slin);
   ast_format_slin16 = ast_getformatbyname("slin16", &(format_slin16));
   bcm_assert(NULL != ast_format_slin16);
#endif /* (110 == AST_VERSION) */
}

static void decode_cid_spill_u8(int cid_signalling, __u8 *spill, size_t spill_len)
{
   struct callerid_state *cid_state = callerid_new(cid_signalling);
   if (NULL == cid_state) {
      bcm_pr_debug("callerid_new() failed !\n");
   }
   else {
      size_t i;
      size_t j;
      int res;

      for (res = 0, i = 0, j = 0; (i < spill_len); i += 1) {
         if (i >= (j + 256)) {
            res = callerid_feed(cid_state, &(spill[j]), i - j, ast_format_alaw);
            j = i + 1;
            if (res) {
               break;
            }
         }
      }
      if ((!res) && (j < i)) {
         res = callerid_feed(cid_state, &(spill[j]), i - j, ast_format_alaw);
      }
      if (!res) {
         bcm_pr_debug("CID spill not complete\n");
      }
      else if (res < 0) {
         bcm_pr_debug("Error processing CID spill\n");
      }
      else {
         int flags = 0;
         char *name = NULL;
         char *number = NULL;
         callerid_get(cid_state, &name, &number, &flags);
         if (NULL == name) {
            name = "";
         }
         else {
            name[32] = '\0';
         }
         if (NULL == number) {
            number = "";
         }
         else {
            number[32] = '\0';
         }
         bcm_pr_debug("CID spill processed : name='%s', number='%s', flags=0x%x\n",
            name, number, (int)(flags));
         bcm_assert(0 == strncmp(cid_name, name, 16));
         bcm_assert(0 == strncmp(cid_number, number, 16));
      }

      callerid_free(cid_state);
   }
}

static void decode_cid_spill_s16(int cid_signalling, const __s16 *spill, size_t spill_len)
{
   struct callerid_state *cid_state = callerid_new(cid_signalling);
   if (NULL == cid_state) {
      bcm_pr_debug("callerid_new() failed !\n");
   }
   else {
      size_t i;
      __u8 tmp[256];
      size_t j;
      int res;

      for (res = 0, i = 0, j = 0; (i < spill_len); i += 1) {
         tmp[j] = AST_LIN2A(spill[i]);
         j += 1;
         if (j >= ARRAY_SIZE(tmp)) {
            res = callerid_feed(cid_state, tmp, j, ast_format_alaw);
            j = 0;
            if (res) {
               break;
            }
         }
      }
      if ((!res) && (j > 0)) {
         res = callerid_feed(cid_state, tmp, j, ast_format_alaw);
      }
      if (!res) {
         bcm_pr_debug("CID spill not complete\n");
      }
      else if (res < 0) {
         bcm_pr_debug("Error processing CID spill\n");
      }
      else {
         int flags = 0;
         char *name = NULL;
         char *number = NULL;
         callerid_get(cid_state, &name, &number, &flags);
         if (NULL == name) {
            name = "";
         }
         if (NULL == number) {
            number = "";
         }
         bcm_pr_debug("CID spill processed : name='%s', number='%s', flags=0x%x\n",
            name, number, (int)(flags));
         bcm_assert(0 == strncmp(cid_name, name, CID_MAX_NAME_LENGTH));
         bcm_assert(0 == strncmp(cid_number, number, CID_MAX_NUMBER_LENGTH));
      }

      callerid_free(cid_state);
   }
}

#else /* !AST_VERSION */

#define MAX_CALLERID_SIZE 32000

static void decode_cid_spill_s16(int cid_signalling, const __s16 *spill, size_t spill_len)
{
}

#ifdef ast_malloc
# undef ast_malloc
#endif
#define ast_malloc malloc
#ifdef ast_free
# undef ast_free
#endif
#define ast_free free

#endif /* !AST_VERSION */

static float my_cid_dr[2], my_cid_di[2];
static float my_clidsb = BCMPH_SAMPLE_RATE / 1200.0;

/*! \brief Initialize stuff for inverse FFT */
static void my_callerid_init(int freq_0, int freq_1)
{
	my_cid_dr[0] = cos(freq_0 * 2.0 * M_PI / BCMPH_SAMPLE_RATE);
	my_cid_di[0] = sin(freq_0 * 2.0 * M_PI / BCMPH_SAMPLE_RATE);
	my_cid_dr[1] = cos(freq_1 * 2.0 * M_PI / BCMPH_SAMPLE_RATE);
	my_cid_di[1] = sin(freq_1 * 2.0 * M_PI / BCMPH_SAMPLE_RATE);
}

static inline float my_callerid_getcarrier(float *cr, float *ci, int bit)
{
	/* Move along.  There's nothing to see here... */
	float t;
	t = *cr * my_cid_dr[bit] - *ci * my_cid_di[bit];
	*ci = *cr * my_cid_di[bit] + *ci * my_cid_dr[bit];
	*cr = t;

	t = 2.0 - (*cr * *cr + *ci * *ci);
	*cr *= t;
	*ci *= t;
	return *ci;
}

#define MY_PUT_WORD(a) do { \
	*(buf++) = (a); \
	words++; \
} while(0)

#define MY_PUT_AUDIO_SAMPLE(y) do { \
	int __sample_idx = (short)(rint(8192.0 * (y))); \
	*(buf++) = __sample_idx; \
	words++; \
} while(0)

#define MY_PUT_CLID_MARKMS do { \
	int __clid_x; \
	for (__clid_x=0;__clid_x<8;__clid_x++) \
		MY_PUT_AUDIO_SAMPLE(my_callerid_getcarrier(&cr, &ci, 1)); \
} while(0)

#define MY_PUT_CLID_BAUD(bit) do { \
	while(scont < my_clidsb) { \
		MY_PUT_AUDIO_SAMPLE(my_callerid_getcarrier(&cr, &ci, bit)); \
		scont += 1.0; \
	} \
	scont -= my_clidsb; \
} while(0)

#define MY_PUT_CLID(byte) do { \
	int z; \
	unsigned char b = (byte); \
	MY_PUT_CLID_BAUD(0); 	/* Start bit */ \
	for (z=0;z<8;z++) { \
		MY_PUT_CLID_BAUD(b & 1); \
		b >>= 1; \
	} \
	MY_PUT_CLID_BAUD(1);	/* Stop bit */ \
} while(0)

static size_t my_callerid_generate(const char *name, const char *number, __s16 *buf)
{
	size_t words = 0;
	int x, sum;
	int len;

	/* Initial carriers (real/imaginary) */
	float cr = 1.0;
	float ci = 0.0;
	float scont = 0.0;
	__u8 msg[256];
	len = bcmph_callerid_sig_fsk_genmsg(name, number, 0, msg, ARRAY_SIZE(msg));
   /* Wait a half a second */
   for (x = 0; x < 4000; x++)
      MY_PUT_WORD(0);
   /* Transmit 30 0x55's (looks like a square wave) for channel seizure */
   for (x = 0; x < 30; x++)
      MY_PUT_CLID(0x55);
	/* Send 150ms of callerid marks */
	for (x = 0; x < 150; x++)
		MY_PUT_CLID_MARKMS;
	/* Send 0x80 indicating MDMF format */
	MY_PUT_CLID(0x80);
	/* Put length of whole message */
	MY_PUT_CLID(len);
	sum = 0x80 + len;
	/* Put each character of message and update checksum */
	for (x = 0; x < len; x++) {
		MY_PUT_CLID(msg[x]);
		sum += msg[x];
	}
	/* Send 2's compliment of sum */
	MY_PUT_CLID(256 - (sum & 255));

	/* Send 50 more ms of marks */
	for (x = 0; x < 50; x++)
		MY_PUT_CLID_MARKMS;

	return words;
}

void do_test_cid(void)
{
   __s16 *cid_buffer = NULL;

#ifdef AST_VERSION
   init_cache_ast_format();
#else /* !AST_VERSION */
   bcmph_init_tones();
#endif /* !AST_VERSION */

   do { /* Empty loop */
      __s16 *z1;
      __s16 *z2;
      size_t len_z1;
      size_t len_z2;
      size_t offset;
      bcmph_callerid_generator_t callerid_generator;

      /* Allocate a buffer */
      cid_buffer = ast_malloc(sizeof(__s16) * MAX_CALLERID_SIZE * 2);
      if (NULL == cid_buffer) {
         bcm_pr_debug("Can't allocate memory to generate CID spill\n");
         break;
      }
      z1 = cid_buffer;
      z2 = cid_buffer + MAX_CALLERID_SIZE;

#if (defined AST_VERSION) && (AST_VERSION >= 110)
      len_z1 = ast_callerid_generate((__u8 *)(z1), cid_name, cid_number, ast_format_alaw);
      bcm_pr_debug("Decoding Asterisk cid data (%lu samples)\n",
         (unsigned long)(len_z1));
      bcm_assert(10467 == len_z1);
      offset = 4000;
      decode_cid_spill_u8(1, ((__u8 *)(z1)) + offset, len_z1 - offset);
#endif /* AST_VERSION */

      my_callerid_init(CID_FSK_BELL_SPACE_FREQ, CID_FSK_BELL_MARK_FREQ);

      len_z1 = my_callerid_generate(cid_name, cid_number, z1);
      bcm_assert(10734 == len_z1);
      bcm_pr_debug("Decoding Bell FSK cid data (Asterisk code) (%lu samples)\n",
         (unsigned long)(len_z1));
      offset = 0;
      decode_cid_spill_s16(1, &(z1[offset]), len_z1 - offset);

      my_callerid_init(CID_FSK_V23_SPACE_FREQ, CID_FSK_V23_MARK_FREQ);

      len_z1 = my_callerid_generate(cid_name, cid_number, z1);
      bcm_assert(10734 == len_z1);

      bcm_pr_debug("Decoding V23 FSK cid data (Asterisk code) (%lu samples)\n",
         (unsigned long)(len_z1));
      offset = 0;
      decode_cid_spill_s16(2, &(z1[offset]), len_z1 - offset);

      /*******/

      len_z1 = bcmph_callerid_generate(
         cid_name, cid_number,
         BCMPH_CID_SIG_BELL, BCMPH_CID_START_RING, 0,
         z1, MAX_CALLERID_SIZE);
      bcm_assert(13934 == len_z1);

      bcm_pr_debug("Decoding Bell FSK cid data (%lu samples generated in one call)\n",
         (unsigned long)(len_z1));
      offset = 0; /* To remove silence */
      decode_cid_spill_s16(1, &(z1[offset]), len_z1 - offset);

      bcmph_callerid_generator_init(&(callerid_generator),
         cid_name, cid_number, BCMPH_CID_SIG_BELL, BCMPH_CID_START_RING, 0);
      for (len_z2 = 0;;) {
         size_t tmp = (*(callerid_generator.sound_generator.vtbl->write))(&(callerid_generator.sound_generator), &(z2[len_z2]), 67);
         if (0 == tmp) {
            break;
         }
         len_z2 += tmp;
      }
      (*(callerid_generator.sound_generator.vtbl->deinit))(&(callerid_generator.sound_generator));
      bcm_assert(len_z1 == len_z2);

      bcm_pr_debug("Decoding Bell FSK cid data (%lu samples generated in several calls)\n",
         (unsigned long)(len_z2));
      offset = 0; /* To remove silence */
      decode_cid_spill_s16(2, &(z2[offset]), len_z2 - offset);

      bcm_assert(0 == memcmp(z1, z2, len_z1));


      /*******/

      len_z1 = bcmph_callerid_generate(
         cid_name, cid_number,
         BCMPH_CID_SIG_V23, BCMPH_CID_START_RING, 0,
         z1, MAX_CALLERID_SIZE);
      bcm_assert(13934 == len_z1);

      bcm_pr_debug("Decoding V23 FSK cid data (%lu samples generated in one call))\n",
         (unsigned long)(len_z1));
      offset = 0; /* To remove silence */
      decode_cid_spill_s16(2, &(z1[offset]), len_z1 - offset);

      /*******/

      len_z1 = bcmph_callerid_generate(
         cid_name, cid_number,
         BCMPH_CID_SIG_DTMF, BCMPH_CID_START_RING, 0,
         z1, MAX_CALLERID_SIZE);
      bcm_assert(21040 == len_z1);

      bcmph_callerid_generator_init(&(callerid_generator),
         cid_name, cid_number, BCMPH_CID_SIG_DTMF, BCMPH_CID_START_RING, 0);
      for (len_z2 = 0;;) {
         size_t tmp = (*(callerid_generator.sound_generator.vtbl->write))(&(callerid_generator.sound_generator), &(z2[len_z2]), 73);
         if (0 == tmp) {
            break;
         }
         len_z2 += tmp;
      }
      (*(callerid_generator.sound_generator.vtbl->deinit))(&(callerid_generator.sound_generator));
      bcm_assert(len_z1 == len_z2);
      bcm_assert(0 == memcmp(z1, z2, len_z1));

   } while (0);

   if (NULL != cid_buffer) {
      ast_free(cid_buffer);
      cid_buffer = NULL;
   }
}

#endif /* BCMPH_NOHW */
