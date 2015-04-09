/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@gilande.com>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#include <config.h>

#ifdef __KERNEL__
#include <linux/delay.h>
#include <linux/jiffies.h>
#endif // __KERNEL__

#include <bcm63xx_phone.h>

#include "main.h"
#include <bcm63xx_log.h>
#include <utils.h>

// Include after system files
#include <compile.h>

static void bcm_drv_set_bytes_per_frame(bcm_drv_t *t, bcm_drv_phone_line_t *dpl,
   bool pcm_use_16bits_timeslot, bcm_phone_codec_t codec)
{
   if (!pcm_use_16bits_timeslot) {
      if ((BCMPH_CODEC_ALAW == codec) || (BCMPH_CODEC_ULAW == codec)) {
         dpl->type_transfer = BCMPH_TRF_M_8_BITS_C_xLAW;
         // We need 1 timeslot/channel
         dpl->bytes_per_frame = BCMPH_PCM_CHANNEL_WIDTH;
      }
      else if (BCMPH_CODEC_LINEAR == codec) {
         dpl->type_transfer = BCMPH_TRF_M_8_BITS_C_LINEAR;
         // We need 2 timeslots/channels
         dpl->bytes_per_frame = 2 * BCMPH_PCM_CHANNEL_WIDTH;
      }
      else {
         bcm_assert(BCMPH_CODEC_LINEAR16 == codec);
         dpl->type_transfer = BCMPH_TRF_M_8_BITS_C_LINEAR16;
         // We need 4 timeslots/channels
         dpl->bytes_per_frame = 4 * BCMPH_PCM_CHANNEL_WIDTH;
      }
   }
   else {
      if ((BCMPH_CODEC_ALAW == codec) || (BCMPH_CODEC_ULAW == codec)) {
         dpl->type_transfer = BCMPH_TRF_M_16_BITS_C_xLAW;
         // We need 1 timeslot/channel but we get/set only half of the bytes
         dpl->bytes_per_frame = (BCMPH_PCM_CHANNEL_WIDTH + 1) / 2;
      }
      else if (BCMPH_CODEC_LINEAR == codec) {
         dpl->type_transfer = BCMPH_TRF_M_16_BITS_C_LINEAR;
         // We need 1 timeslot/channel
         dpl->bytes_per_frame = BCMPH_PCM_CHANNEL_WIDTH;
      }
      else {
         bcm_assert(BCMPH_CODEC_LINEAR16 == codec);
         dpl->type_transfer = BCMPH_TRF_M_16_BITS_C_LINEAR16;
         // We need 2 timeslots/channels
         dpl->bytes_per_frame = 2 * BCMPH_PCM_CHANNEL_WIDTH;
      }
   }
#ifdef BCMPH_TEST_PCM
   dpl->type_transfer = BCMPH_TRF_TEST_PCM;
   dpl->bytes_per_frame = BCMPH_PCM_MAX_CHANNELS * BCMPH_PCM_CHANNEL_WIDTH;
#endif // BCMPH_TEST_PCM
   bcm_pr_debug("bcm_drv_set_bytes_per_frame(pcm_use_16bits_timeslot=%d, codec=%d) => type_transfer=%d, bytes_per_frame=%lu\n",
      (int)(pcm_use_16bits_timeslot), (int)(codec),
      (int)(dpl->type_transfer), (unsigned long)(dpl->bytes_per_frame));
}

static void bcm_drv_update_ring_buf_desc(bcm_drv_t *t)
{
   size_t i;
   size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->phone_mgr));
   bcm_ring_buf_desc_t *tmp;

   dd_bcm_pr_debug("bcm_drv_update_ring_buf_desc()\n");

   for (i = 0; (i < phone_line_count); i += 1) {
      if (bcm_phone_mgr_line_is_enabled(&(t->phone_mgr), i)) {
         const bcm_drv_phone_line_t *dpl = &(t->phone_lines[i]);
         tmp = (bcm_ring_buf_desc_t *)(t->mm_buffer + t->mm_rbs_location.rbs[i].rx_ring_buf_desc_off);
         bcm_ring_buf_get_desc(&(dpl->rx_ring_buf), tmp);

         tmp = (bcm_ring_buf_desc_t *)(t->mm_buffer + t->mm_rbs_location.rbs[i].tx_ring_buf_desc_off);
         bcm_ring_buf_get_desc(&(dpl->tx_ring_buf), tmp);
      }
   }
}

static int bcm_drv_set_line_mode(bcm_drv_t *t, size_t line,
   bcm_phone_line_mode_t mode, __u32 tone,
   int wait, bcmph_mutex_t *lock)
{
   int ret = 0;

   d_bcm_pr_debug("bcm_drv_set_line_mode(line=%lu, mode=%d, tone=%lu, wait=%d)\n",
      (unsigned long)(line), (int)(mode), (unsigned long)(tone), (int)(wait));

   bcm_assert(((NULL == lock) && (0 == wait) && (BCMPH_MODE_OFF_TALKING != mode))
      || ((NULL != lock) && (bcmph_mutex_is_locked(lock))));

   do { // Empty loop

      if ((line >= bcm_phone_mgr_get_line_count(&(t->phone_mgr)))
          || (!bcm_phone_mgr_line_is_enabled(&(t->phone_mgr), line))
          || (BCMPH_MODE_UNSPECIFIED == mode) || (mode >= BCMPH_MAX_MODES)) {
         bcm_pr_err("Invalid line or invalid mode\n");
         ret = -EINVAL;
         break;
      }


      if (BCMPH_MODE_OFF_TALKING == mode) {
         bcm_assert(NULL != lock);
         if (!pcm_dma_is_started(&(t->pcm))) {
            bcm_pr_err("DMA not started, so can't change to mode BCMPH_MODE_OFF_TALKING\n");
            ret = -EFAULT;
            break;
         }
         ret = bcm_phone_mgr_set_line_mode(&(t->phone_mgr), line, mode, tone, -1, lock);
         // If set_line_mode failed it's not dramatic
         if ((NULL == lock) || (bcmph_mutex_is_locked(lock))) {
            // It's just a precaution, but must already been done when starting and when codec changes
            bcm_drv_phone_line_t *dpl = &(t->phone_lines[line]);
            bcm_drv_set_bytes_per_frame(t, dpl, pcm_timeslot_is_16bits(&(t->pcm)),
               bcm_phone_mgr_get_current_codec(&(t->phone_mgr), line));
            bcm_ring_buf_clear(&(dpl->rx_ring_buf));
         }
      }
      else {
         ret = bcm_phone_mgr_set_line_mode(&(t->phone_mgr), line, mode, tone, wait, lock);
      }

      if ((NULL == lock) || (bcmph_mutex_is_locked(lock))) {
         // Wake up processes blocked in inq and outq because the mode change
         // may reduce the sets of lines using the PCM bus
         // That may unblock sending and receiving of data
         bcm_drv_update_buffers(t);
      }
   }
   while (false);

   return (ret);
}

static int bcm_drv_set_line_tone(bcm_drv_t *t, size_t line, __u32 tone,
   int wait, bcmph_mutex_t *lock)
{
   int ret = 0;

   d_bcm_pr_debug("bcm_drv_set_line_tone(line=%lu, tone=%lu, wait=%d)\n",
      (unsigned long)(line), (unsigned long)(tone), (int)(wait));

   bcm_assert(((NULL == lock) && (0 == wait))
      || ((NULL != lock) && (bcmph_mutex_is_locked(lock))));

   do { // Empty loop
      bcm_phone_line_tone_t tone_index = bcm_phone_line_tone_decode_index(tone);

      if ((line >= bcm_phone_mgr_get_line_count(&(t->phone_mgr)))
          || (!bcm_phone_mgr_line_is_enabled(&(t->phone_mgr), line))
          || (BCMPH_TONE_UNSPECIFIED == tone_index)
          || (tone_index >= BCMPH_MAX_TONES)) {
         bcm_pr_err("Invalid line or invalid tone\n");
         ret = -EINVAL;
         break;
      }

      ret = bcm_phone_mgr_set_line_tone(&(t->phone_mgr), line, tone, wait, lock);
   }
   while (false);

   return (ret);
}

static void bcm_drv_start_pcm(bcm_drv_t *t)
{
   if (!pcm_dma_is_started(&(t->pcm))) {
      size_t i;
      size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->phone_mgr));

      for (i = 0; (i < phone_line_count); i += 1) {
         bcm_ring_buf_clear(&(t->phone_lines[i].rx_ring_buf));
      }
      pcm_dma_start(&(t->pcm));
   }
}

static void bcm_drv_stop_pcm(bcm_drv_t *t, bool wait)
{
   if (pcm_is_started(&(t->pcm))) {
      size_t i;
      size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->phone_mgr));
      unsigned int delay = ((pcm_get_time_to_send_tx_buffer(&(t->pcm)) + 999) / 1000) + 2;

      if (wait) {
         size_t dummy;
         size_t frames_to_write;

         bcm_drv_fill_rx_buffers(t);
         bcm_drv_empty_tx_buffers(t, true);

         // First we wait until TX buffers are empty
         bcm_drv_get_data_for_tx(t, &(dummy), &(frames_to_write));
         if (frames_to_write > 0) {
            d_bcm_pr_debug("Waiting TX buffers to become empty\n");
            for (;;) {
               size_t tmp;
               d_bcm_pr_debug("Waiting %u ms\n", (unsigned int)(delay));
               msleep(delay);
               bcm_drv_empty_tx_buffers(t, true);
               bcm_drv_get_data_for_tx(t, &(dummy), &(tmp));
               if (frames_to_write == tmp) {
                  // Transmission is frozen
                  bcm_pr_debug("TX buffers are not empty.\n");
                  break;
               }
               frames_to_write = tmp;
            }
         }
         // Then we wait until there's no buffer owned by DMA
         if (frames_to_write <= 0) {
            if (pcm_get_cnt_tx_buffer_really_owned(&(t->pcm)) > 0) {
               d_bcm_pr_debug("Waiting all DMA buffers to be transmitted\n");
               for (;;) {
                  d_bcm_pr_debug("Waiting %u ms\n", (unsigned int)(delay));
                  msleep(delay);
                  if (pcm_tx_reclaim(&(t->pcm), 1) <= 0) {
                     if (pcm_get_cnt_tx_buffer_really_owned(&(t->pcm)) > 0) {
                        bcm_pr_debug("Not all DMA buffers are transmitted.\n");
                     }
                     break;
                  }
               }
            }
         }
      }
      // If some line use pcm, we change the mode of the line in order to not
      // use the pcm
      for (i = 0; (i < phone_line_count); i += 1) {
         if (bcm_phone_mgr_line_tx_use_pcm(&(t->phone_mgr), i)) {
            // if mode is not compatible with line status it will be changed
            // to BCMPH_NONE and that's what we want
            bcm_drv_set_line_tone(t, i,
               bcm_phone_line_tone_code_index(BCMPH_TONE_DISCONNECT),
               0, NULL);
         }
      }
      pcm_dma_stop(&(t->pcm));
      for (i = 0; (i < phone_line_count); i += 1) {
         bcm_ring_buf_clear(&(t->phone_lines[i].tx_ring_buf));
      }
   }
}

static inline void bcm_drv_read_version(bcm_drv_t *t, bcm_phone_drv_ver_t *ver)
{
   memset(ver, 0, sizeof(*ver));
   ver->major = 0;
   ver->minor = 1;
}

static int bcm_drv_set_line_codec(bcm_drv_t *t, size_t line,
   bcm_phone_codec_t codec, bcm_phone_line_mode_t mode,
   __u32 tone, bcmph_mutex_t *lock)
{
   int ret = 0;

   d_bcm_pr_debug("bcm_drv_set_line_codec(line=%lu, codec=%d, mode=%d, tone=%lu)\n",
      (unsigned long)(line), (int)(codec), (int)(mode), (unsigned long)(tone));

   bcm_assert((NULL != lock) && (bcmph_mutex_is_locked(lock)));

   do { // Empty loop
      bcm_phone_codec_t current_codec;
      bcm_drv_phone_line_t *dpl;

      if ((line > bcm_phone_mgr_get_line_count(&(t->phone_mgr)))
          || (!bcm_phone_mgr_line_is_enabled(&(t->phone_mgr), line))) {
         bcm_pr_err("Invalid line\n");
         ret = -EINVAL;
         break;
      }

      if ((BCMPH_CODEC_ALAW != codec) && (BCMPH_CODEC_ULAW != codec)
              && (BCMPH_CODEC_LINEAR != codec) && (BCMPH_CODEC_LINEAR16 != codec)) {
         bcm_pr_err("Invalid codec %d\n", (int)(codec));
         ret = -EINVAL;
         break;
      }

      if (!bcm_phone_mgr_line_supports_codec(&(t->phone_mgr), line, codec)) {
         bcm_pr_err("Device to which the line %lu belongs does not support codec %d\n", (unsigned long)(line), (int)(codec));
         ret = -EINVAL;
         break;
      }

      current_codec = bcm_phone_mgr_get_current_codec(&(t->phone_mgr), line);

      if (!bcm_phone_mgr_line_can_switch_to_codec(&(t->phone_mgr), line, current_codec, codec)) {
         bcm_pr_err("Line %lu can't be switched to codec %d\n", (unsigned long)(line), (int)(codec));
         ret = -EINVAL;
         break;
      }

      if (BCMPH_MODE_OFF_TALKING == mode) {
         if (!pcm_dma_is_started(&(t->pcm))) {
            bcm_pr_err("DMA not started, so can't change to mode BCMPH_MODE_OFF_TALKING\n");
            ret = -EFAULT;
            break;
         }
      }

      // Here if codec is supported, it means we already reserved the right timeslots and PCM channels for the line
      // so we don't do any verifications
      dpl = &(t->phone_lines[line]);

      // Buffers must be emptied
      bcm_ring_buf_clear(&(dpl->rx_ring_buf));
      bcm_ring_buf_clear(&(dpl->tx_ring_buf));

      ret = bcm_phone_mgr_set_line_codec(&(t->phone_mgr), line, codec,
         mode, tone, -1, lock);
      if (ret) {
         // Here we have a problem the process may have been interrupted
         // or we may have failed to lock the mutex, so we don't know
         // what the codec is really.
         bcm_pr_debug("bcm_phone_mgr_set_line_codec() failed\n");
         if (!bcmph_mutex_is_locked(lock)) {
            // We try to lock the mutex
            if (bcmph_mutex_lock_interruptible(lock)) {
               // We can't lock the mutex so we exit the function
               bcm_pr_err("Possible failure to set codec for line %lu (can't lock mutex)\n", (unsigned long)(line));
               break;
            }
         }
         // We check once again if there is a change pending for the line
         if (bcm_phone_mgr_line_has_change_pending(&(t->phone_mgr), line)) {
            // Alas there is, so we wait for more than board_desc->phone_desc->tick_period ms (the timer period)
            msleep((t->board_desc->phone_desc->tick_period * 5) / 2);
            // And we check a last time if there is a change pending for the line
            if (bcm_phone_mgr_line_has_change_pending(&(t->phone_mgr), line)) {
               // We give up : we don't know if the codec has been set or will be set
               bcm_pr_err("Possible failure to set codec for line %lu (change not acknowledged)\n", (unsigned long)(line));
            }
         }
      }
      current_codec = bcm_phone_mgr_get_current_codec(&(t->phone_mgr), line);
      if (codec != current_codec) {
         // If codec is not what we set this is an error, because codec
         // is not supposed to change anytime but only when we set it
         bcm_pr_err("Unexpected failure to set codec for line %lu\n", (unsigned long)(line));
         codec = current_codec;
         ret = -EFAULT;
      }

      bcm_drv_set_bytes_per_frame(t, dpl, pcm_timeslot_is_16bits(&(t->pcm)), codec);
      // Buffers must be emptied
      bcm_ring_buf_clear(&(dpl->rx_ring_buf));
      bcm_ring_buf_clear(&(dpl->tx_ring_buf));

      bcm_drv_update_buffers(t);
   }
   while (false);

   return (ret);
}

static bool bcm_drv_one_line_changes_state(bcm_phone_line_state_t *line_states, size_t line_count)
{
   bool ret = false;
   size_t i;

   dd_bcm_pr_debug("bcm_drv_one_line_changes_state()\n");

   for (i = 0; (i < line_count); i += 1) {
      if (bcm_phone_line_state_has_changes(&(line_states[i]))) {
         ret = true;
         break;
      }
   }
   return (ret);
}

// Transfer line states in structure passed as parameter
static int bcm_drv_get_line_states(bcm_drv_t *t,
   bcm_phone_get_line_states_t *get_line_states, bcmph_mutex_t *lock)
{
   int ret = 0;
   size_t i;
   int line_enabled = 0;
   int eventq_counter = 0;
   long timeout_in_jiffies;

   dd_bcm_pr_debug("bcm_drv_get_line_states()\n");

   bcm_assert((NULL != lock) && (bcmph_mutex_is_locked(lock)));

   memset(get_line_states->line_state, 0, sizeof(get_line_states->line_state));
   for (i = 0; (i < ARRAY_SIZE(get_line_states->line_state)); i += 1) {
      bcm_phone_line_state_init(&(get_line_states->line_state[i]));
      if (bcm_phone_mgr_line_is_enabled(&(t->phone_mgr), i)) {
         line_enabled += 1;
      }
   }

   if (get_line_states->wait < 0) {
      timeout_in_jiffies = MAX_JIFFY_OFFSET;
   }
   else {
      timeout_in_jiffies = msecs_to_jiffies(get_line_states->wait);
   }

   if (line_enabled > 0) {
      for (;;) {
         eventq_counter = bcm_wait_queue_get_counter(bcm_phone_mgr_get_eventq(&(t->phone_mgr)));

         bcm_phone_mgr_get_line_states(&(t->phone_mgr), get_line_states->line_state, ARRAY_SIZE(get_line_states->line_state));
         if (bcm_drv_one_line_changes_state(get_line_states->line_state, bcm_phone_mgr_get_line_count(&(t->phone_mgr)))) {
            break;
         }

         if ((0 != timeout_in_jiffies)
             && (pcm_is_started(&(t->pcm)))) {
            if (MAX_JIFFY_OFFSET == timeout_in_jiffies) {
               ret = bcm_wait_queue_wait_event_counter(
                  bcm_phone_mgr_get_eventq(&(t->phone_mgr)),
                  eventq_counter, lock);
               if (0 != ret) {
                  break;
               }
            }
            else {
               long remaining = bcm_wait_queue_wait_event_counter_timeout(
                  bcm_phone_mgr_get_eventq(&(t->phone_mgr)),
                  eventq_counter, timeout_in_jiffies, lock);
               if (remaining < 0) {
                  ret = remaining;
                  break;
               }
               timeout_in_jiffies = remaining;
            }
         }
         else {
            break;
         }
      }
   }

   return (ret);
}

#ifdef BCMPH_TEST_PCM
static int bcm_drv_set_line_state(bcm_drv_t *t,
   const bcm_phone_set_line_state_t *set_line_state)
{
   int ret = 0;

   d_bcm_pr_debug("bcm_drv_set_line_state()\n");

   bcm_assert(NULL != set_line_state);

   do { // Empty loop
      if ((set_line_state->line > bcm_phone_mgr_get_line_count(&(t->phone_mgr)))
          || (!bcm_phone_mgr_line_is_enabled(&(t->phone_mgr), set_line_state->line))) {
         bcm_pr_err("Invalid line\n");
         ret = -EINVAL;
         break;
      }

      ret = bcm_phone_mgr_set_line_state(&(t->phone_mgr), set_line_state);
   }
   while (false);

   return (ret);
}
#endif // BCMPH_TEST_PCM

// Stop PCM.
// Flag wait tells to wait for transmission of all pending data
// (in TX ring buffers and in DMA buffers)
void bcm_drv_stop(bcm_drv_t *t, int wait)
{
   bcm_pr_debug("bcm_drv_stop()\n");

   bcm_drv_stop_pcm(t, wait);

   pcm_stop(&(t->pcm));

   bcm_phone_mgr_stop(&(t->phone_mgr));

   // We must wake up all processes sleeping in wait_queue
   bcm_wait_queue_wake_up(&(t->inq));
   bcm_wait_queue_wake_up(&(t->outq));
}

static int bcm_drv_use_ts_and_pcm_channel(
   size_t line,
   __u8 *ts_to_channel, size_t ts_to_channel_len, __u32 *pcm_channels_used,
   __u8 timeslot, __u8 pcm_channel)
{
   int ret = 0;

   bcm_assert((NULL != ts_to_channel) && (NULL != pcm_channels_used));

   do { // Empty loop
      if (timeslot >= ts_to_channel_len) {
         bcm_pr_err("Invalid timeslot %u for line %lu\n",
            (unsigned int)(timeslot), (unsigned long)(line));
         ret = -EINVAL;
         break;
      }
      if (ts_to_channel[timeslot] < BCMPH_PCM_MAX_CHANNELS) {
         bcm_pr_err("Timeslot %u for line %lu is already in use\n",
            (unsigned int)(timeslot), (unsigned long)(line));
         ret = -EINVAL;
         break;
      }
      if (pcm_channel >= BCMPH_PCM_MAX_CHANNELS) {
         bcm_pr_err("Invalid PCM channel %u for line %lu\n",
            (unsigned int)(pcm_channel), (unsigned long)(line));
         ret = -EINVAL;
         break;
      }
      if ((*pcm_channels_used & (1 << pcm_channel))) {
         bcm_pr_err("PCM channel %u for line %lu is already in use\n",
            (unsigned int)(pcm_channel), (unsigned long)(line));
         ret = -EINVAL;
         break;
      }

      ts_to_channel[timeslot] = pcm_channel;
      *pcm_channels_used |= (1 << pcm_channel);
   }
   while (false);

   return (ret);
}

// Configure PCM
static int bcm_drv_start(bcm_drv_t *t, const bcm_phone_cfg_params_t *params)
{
   int ret = 0;
   size_t max_timeslots = pcm_get_max_timeslots(&(t->pcm));

   bcm_pr_debug("bcm_drv_start()\n");

   bcm_assert(0 == (max_timeslots % 2));

   do { // Empty loop
      size_t i;
      phone_line_params_t line_params[BCMPH_MAX_LINES];
      size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->phone_mgr));
      size_t disabled_lines;
      size_t enabled_lines;
      __u8 timeslot_to_channel[128];
      __u32 pcm_channels_used;
      __u8 pcm_channel;

      bcm_assert(ARRAY_SIZE(timeslot_to_channel) >= max_timeslots);
      bcm_assert(ARRAY_SIZE(line_params) >= ARRAY_SIZE(params->line_params));

      bcm_drv_stop(t, 0);

      if (/*(params->country < 0) || */(params->country >= BCMPH_COUNTRY_MAX)) {
         ret = -EINVAL;
         break;
      }

      memset(line_params, 0, sizeof(line_params));
      for (i = 0; (i < phone_line_count); i += 1) {
         bcm_drv_phone_line_t *dpl = &(t->phone_lines[i]);
         // We set a very high value for bytes_per_frame, so that we
         // can't have enough data or enough free space to write or
         // read a DMA frame
         dpl->bytes_per_frame = 0x7FFFFFFF;
         dpl->offset_first_pcm_channel = 0;
         dpl->type_transfer = BCMPH_TRF_NO_OP;
         if (i < ARRAY_SIZE(line_params)) {
            if (i < ARRAY_SIZE(params->line_params)) {
               line_params[i].enable = params->line_params[i].enable;
               line_params[i].codec = params->line_params[i].codec;
               line_params[i].first_timeslot = bcm_phone_mgr_get_first_timeslot_line(&(t->phone_mgr), i);
            }
            else {
               line_params[i].enable = false;
            }
         }
      }
      for (; (i < ARRAY_SIZE(line_params)); i += 1) {
         line_params[i].enable = false;
      }

      ret = 0;
      pcm_channel = 0;
      pcm_channels_used = 0;
      enabled_lines = 0;
      if (ARRAY_SIZE(line_params) < phone_line_count) {
         disabled_lines = phone_line_count - ARRAY_SIZE(line_params);
      }
      else {
         disabled_lines = 0;
      }
#if (BCMPH_PCM_MAX_CHANNELS >= 32)
#error BCMPH_PCM_MAX_CHANNELS is >= 32
#endif
      memset(timeslot_to_channel, 255, sizeof(timeslot_to_channel));
      for (i = 0; (i < ARRAY_SIZE(line_params)); i += 1) {
         if (!line_params[i].enable) {
            disabled_lines += 1;
         }
         else {
            enabled_lines += 1;

            // Check that line exists
            if (i >= phone_line_count) {
               bcm_pr_err("Trying to enable line %lu that don't exist\n", (unsigned long)(i));
               ret = -EINVAL;
               break;
            }

            // Check the codec is valid
            if ((BCMPH_CODEC_ALAW != line_params[i].codec)
                && (BCMPH_CODEC_ULAW != line_params[i].codec)
                && (BCMPH_CODEC_LINEAR != line_params[i].codec)
                && (BCMPH_CODEC_LINEAR16 != line_params[i].codec)) {
               bcm_pr_err("Invalid codec %d for line %lu\n",
                  (int)(line_params[i].codec), (unsigned long)(i));
               ret = -EINVAL;
               break;
            }

            // Check that the device supports the codec
            if (!bcm_phone_mgr_line_supports_codec(&(t->phone_mgr), i, line_params[i].codec)) {
               bcm_pr_err("Invalid codec %d for line %lu\n",
                  (int)(line_params[i].codec), (unsigned long)(i));
               ret = -EINVAL;
               break;
            }

            // Check the timeslot is even
            if (0 != (line_params[i].first_timeslot % 2)) {
               bcm_pr_err("The first timeslot of a line must be even. The line %lu does not meet this condition.\n", (unsigned long)(i));
               ret = -EINVAL;
               break;
            }
            // Check that timeslots used by the line are available
            ret = bcm_drv_use_ts_and_pcm_channel(i,
               timeslot_to_channel, max_timeslots, &(pcm_channels_used),
               line_params[i].first_timeslot, pcm_channel);
            if (ret) {
               break;
            }
            pcm_channel += 1;
            if (!params->pcm_use_16bits_timeslot) {
               if (((bcm_phone_mgr_line_supports_codec(&(t->phone_mgr), i, BCMPH_CODEC_LINEAR))
                    && (bcm_phone_mgr_line_can_switch_to_codec(&(t->phone_mgr), i, line_params[i].codec, BCMPH_CODEC_LINEAR)))
                   || ((bcm_phone_mgr_line_supports_codec(&(t->phone_mgr), i, BCMPH_CODEC_LINEAR16))
                       && (bcm_phone_mgr_line_can_switch_to_codec(&(t->phone_mgr), i, line_params[i].codec, BCMPH_CODEC_LINEAR16)))) {
                  __u8 second_timeslot = line_params[i].first_timeslot + 1;
                  ret = bcm_drv_use_ts_and_pcm_channel(i,
                     timeslot_to_channel, max_timeslots, &(pcm_channels_used),
                     second_timeslot, pcm_channel);
                  if (ret) {
                     break;
                  }
                  pcm_channel += 1;
                  if ((bcm_phone_mgr_line_supports_codec(&(t->phone_mgr), i, BCMPH_CODEC_LINEAR16))
                      && (bcm_phone_mgr_line_can_switch_to_codec(&(t->phone_mgr), i, line_params[i].codec, BCMPH_CODEC_LINEAR16))) {
                     __u8 third_timeslot = line_params[i].first_timeslot + (max_timeslots / 2);
                     __u8 fourth_timeslot = third_timeslot + 1;
                     ret = bcm_drv_use_ts_and_pcm_channel(i,
                        timeslot_to_channel, max_timeslots, &(pcm_channels_used),
                        third_timeslot, pcm_channel);
                     if (ret) {
                        break;
                     }
                     pcm_channel += 1;
                     ret = bcm_drv_use_ts_and_pcm_channel(i,
                        timeslot_to_channel, max_timeslots, &(pcm_channels_used),
                        fourth_timeslot, pcm_channel);
                     if (ret) {
                        break;
                     }
                     pcm_channel += 1;
                  }
               }
            }
            else {
               // first_timeslot + 1, is implicitly read by the same channel as line_params[i].timeslot[0]
               bcm_assert(timeslot_to_channel[line_params[i].first_timeslot + 1] >= BCMPH_PCM_MAX_CHANNELS);
               timeslot_to_channel[line_params[i].first_timeslot + 1] = timeslot_to_channel[line_params[i].first_timeslot];
               if ((bcm_phone_mgr_line_supports_codec(&(t->phone_mgr), i, BCMPH_CODEC_LINEAR16))
                   && (bcm_phone_mgr_line_can_switch_to_codec(&(t->phone_mgr), i, line_params[i].codec, BCMPH_CODEC_LINEAR16))) {
                  __u8 third_timeslot = line_params[i].first_timeslot + (max_timeslots / 2);
                  ret = bcm_drv_use_ts_and_pcm_channel(i,
                     timeslot_to_channel, max_timeslots, &(pcm_channels_used),
                     third_timeslot, pcm_channel);
                  if (ret) {
                     break;
                  }
                  pcm_channel += 1;
                  bcm_assert(timeslot_to_channel[third_timeslot + 1] >= BCMPH_PCM_MAX_CHANNELS);
                  timeslot_to_channel[third_timeslot + 1] = timeslot_to_channel[third_timeslot];
               }
            }
         }
      }
      if (ret) {
         break;
      }

      // Now that we know the number of lines enabled we can share the
      // space reserved for the ring buffers between all these lines
      {
         size_t offset = t->offset_rbs;
         size_t size = t->size_rbs;
         size_t offset_dummy_ring_buffer = offset;
         size_t len_dummy_ring_buffer = BCMPH_RB_ROUNDING_FACTOR;
         size_t len_rx_ring_buffer = BCMPH_RB_ROUNDING_FACTOR;
         size_t len_tx_ring_buffer = BCMPH_RB_ROUNDING_FACTOR;

         bcm_assert(offset == round_up_to_pow_of_2(offset, BCMPH_MMAP_ALIGNMENT));
         if (disabled_lines > 0) {
            // Reserve memory for a dummy ring buffer for disabled lines just in case
            offset += round_up_to_pow_of_2(len_dummy_ring_buffer, BCMPH_MMAP_ALIGNMENT);
            size -= round_up_to_pow_of_2(len_dummy_ring_buffer, BCMPH_MMAP_ALIGNMENT);
         }
         if (enabled_lines > 0) {
            len_rx_ring_buffer = round_down_generic(
               round_down_to_pow_of_2((size * RX_BUFFER_PERCENT) / (100 * enabled_lines), BCMPH_MMAP_ALIGNMENT),
               BCMPH_RB_ROUNDING_FACTOR);
            if (len_rx_ring_buffer <= 0) {
               bcm_assert(false);
               len_rx_ring_buffer = BCMPH_RB_ROUNDING_FACTOR;
            }
            len_tx_ring_buffer = round_down_generic(
               round_down_to_pow_of_2(((size - (round_up_to_pow_of_2(len_rx_ring_buffer, BCMPH_MMAP_ALIGNMENT) * enabled_lines)) / enabled_lines), BCMPH_MMAP_ALIGNMENT),
               BCMPH_RB_ROUNDING_FACTOR);
            if (len_tx_ring_buffer <= 0) {
               bcm_assert(false);
               len_tx_ring_buffer = BCMPH_RB_ROUNDING_FACTOR;
            }
            bcm_assert(((round_up_to_pow_of_2(len_rx_ring_buffer, BCMPH_MMAP_ALIGNMENT) + round_up_to_pow_of_2(len_tx_ring_buffer, BCMPH_MMAP_ALIGNMENT)) * enabled_lines) <= size);
         }

         // We check that default_line is a line enabled.
         if (t->default_line >= ARRAY_SIZE(line_params)) {
            t->default_line = 0;
         }
         for (i = 0; (i < ARRAY_SIZE(line_params)); i += 1) {
            bcm_drv_phone_line_t *dpl = &(t->phone_lines[i]);
            if (line_params[i].enable) {
               // Check that default_line is the index of a line enabled
               if (!line_params[t->default_line].enable) {
                  t->default_line = i;
               }
               bcm_drv_set_bytes_per_frame(t, dpl, params->pcm_use_16bits_timeslot, line_params[i].codec);
               dpl->offset_first_pcm_channel = timeslot_to_channel[line_params[i].first_timeslot] * BCMPH_PCM_CHANNEL_WIDTH;
               size = len_rx_ring_buffer;
               t->mm_rbs_location.rbs[i].rx_buffer_offset = offset;
               t->mm_rbs_location.rbs[i].rx_buffer_size = size;
               bcm_pr_debug("t->mm_rbs_location.rbs[%lu].rx_buffer_offset = %lu, t->mm_rbs_location.rbs[%lu].rx_buffer_size = %lu\n",
                  (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].rx_buffer_offset),
                  (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].rx_buffer_size));
               offset += round_up_to_pow_of_2(size, BCMPH_MMAP_ALIGNMENT);

               size = len_tx_ring_buffer;
               t->mm_rbs_location.rbs[i].tx_buffer_offset = offset;
               t->mm_rbs_location.rbs[i].tx_buffer_size = size;
               bcm_pr_debug("t->mm_rbs_location.rbs[%lu].tx_buffer_offset = %lu, t->mm_rbs_location.rbs[%lu].tx_buffer_size = %lu\n",
                  (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].tx_buffer_offset),
                  (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].tx_buffer_size));
               offset += round_up_to_pow_of_2(size, BCMPH_MMAP_ALIGNMENT);
            }
            else {
               bcm_assert((BCMPH_TRF_NO_OP == dpl->type_transfer) && (dpl->bytes_per_frame > t->size_rbs));
               t->mm_rbs_location.rbs[i].rx_buffer_offset = offset_dummy_ring_buffer;
               t->mm_rbs_location.rbs[i].rx_buffer_size = len_dummy_ring_buffer;
               bcm_pr_debug("t->mm_rbs_location.rbs[%lu].rx_buffer_offset = %lu, t->mm_rbs_location.rbs[%lu].rx_buffer_size = %lu\n",
                  (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].rx_buffer_offset),
                  (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].rx_buffer_size));
               t->mm_rbs_location.rbs[i].tx_buffer_offset = offset_dummy_ring_buffer;
               t->mm_rbs_location.rbs[i].tx_buffer_size = len_dummy_ring_buffer;
               bcm_pr_debug("t->mm_rbs_location.rbs[%lu].tx_buffer_offset = %lu, t->mm_rbs_location.rbs[%lu].tx_buffer_size = %lu\n",
                  (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].tx_buffer_offset),
                  (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].tx_buffer_size));
            }
            bcm_ring_buf_init(&(dpl->rx_ring_buf),
               t->mm_buffer + t->mm_rbs_location.rbs[i].rx_buffer_offset,
               t->mm_rbs_location.rbs[i].rx_buffer_size);
            bcm_ring_buf_get_desc(&(dpl->rx_ring_buf),
               (bcm_ring_buf_desc_t *)(t->mm_buffer + t->mm_rbs_location.rbs[i].rx_ring_buf_desc_off));
            bcm_ring_buf_init(&(dpl->tx_ring_buf),
               t->mm_buffer + t->mm_rbs_location.rbs[i].tx_buffer_offset,
               t->mm_rbs_location.rbs[i].tx_buffer_size);
            bcm_ring_buf_get_desc(&(dpl->tx_ring_buf),
               (bcm_ring_buf_desc_t *)(t->mm_buffer + t->mm_rbs_location.rbs[i].tx_ring_buf_desc_off));
         }
         for (; (i < phone_line_count); i += 1) {
            bcm_drv_phone_line_t *dpl = &(t->phone_lines[i]);

            // Phone line is implicitly disabled
            bcm_assert((BCMPH_TRF_NO_OP == dpl->type_transfer) && (dpl->bytes_per_frame > t->size_rbs));
            t->mm_rbs_location.rbs[i].rx_buffer_offset = offset_dummy_ring_buffer;
            t->mm_rbs_location.rbs[i].rx_buffer_size = len_dummy_ring_buffer;
            bcm_pr_debug("t->mm_rbs_location.rbs[%lu].rx_buffer_offset = %lu, t->mm_rbs_location.rbs[%lu].rx_buffer_size = %lu\n",
               (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].rx_buffer_offset),
               (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].rx_buffer_size));
            t->mm_rbs_location.rbs[i].tx_buffer_offset = offset_dummy_ring_buffer;
            t->mm_rbs_location.rbs[i].tx_buffer_size = len_dummy_ring_buffer;
            bcm_pr_debug("t->mm_rbs_location.rbs[%lu].tx_buffer_offset = %lu, t->mm_rbs_location.rbs[%lu].tx_buffer_size = %lu\n",
               (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].tx_buffer_offset),
               (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].tx_buffer_size));
            bcm_ring_buf_init(&(dpl->rx_ring_buf),
               t->mm_buffer + t->mm_rbs_location.rbs[i].rx_buffer_offset,
               t->mm_rbs_location.rbs[i].rx_buffer_size);
            bcm_ring_buf_get_desc(&(dpl->rx_ring_buf),
               (bcm_ring_buf_desc_t *)(t->mm_buffer + t->mm_rbs_location.rbs[i].rx_ring_buf_desc_off));
            bcm_ring_buf_init(&(dpl->tx_ring_buf),
               t->mm_buffer + t->mm_rbs_location.rbs[i].tx_buffer_offset,
               t->mm_rbs_location.rbs[i].tx_buffer_size);
            bcm_ring_buf_get_desc(&(dpl->tx_ring_buf),
               (bcm_ring_buf_desc_t *)(t->mm_buffer + t->mm_rbs_location.rbs[i].tx_ring_buf_desc_off));
         }
      }

      if (params->pcm_use_16bits_timeslot) {
         for (i = 0; (i < max_timeslots); i += 2) {
            // We must correct timeslot because in 16 bits mode a timeslot
            // is 16 bits wide for PCM module
            if (timeslot_to_channel[i] < BCMPH_PCM_MAX_CHANNELS) {
               bcm_assert(timeslot_to_channel[i + 1] == timeslot_to_channel[i]);
               timeslot_to_channel[i >> 1] = timeslot_to_channel[i];
            }
            else {
               bcm_assert(timeslot_to_channel[i + 1] >= BCMPH_PCM_MAX_CHANNELS);
               timeslot_to_channel[i >> 1] = BCMPH_PCM_MAX_CHANNELS;
            }
         }
         max_timeslots >>= 1;
      }
      {
         __u8 channel_to_timeslot[BCMPH_PCM_MAX_CHANNELS];

         memset(channel_to_timeslot, BCMPH_TIMESLOT_UNSPECIFIED, sizeof(channel_to_timeslot));
         for (i = 0; (i < max_timeslots); i += 1) {
            if (timeslot_to_channel[i] < BCMPH_PCM_MAX_CHANNELS) {
               bcm_assert(BCMPH_TIMESLOT_UNSPECIFIED == channel_to_timeslot[timeslot_to_channel[i]]);
               channel_to_timeslot[timeslot_to_channel[i]] = i;
            }
         }
         for (i = 0; (i < pcm_channel); i += 1) {
            bcm_assert(BCMPH_TIMESLOT_UNSPECIFIED != channel_to_timeslot[i]);
         }
         for (; (i < ARRAY_SIZE(channel_to_timeslot)); i += 1) {
            bcm_assert(BCMPH_TIMESLOT_UNSPECIFIED == channel_to_timeslot[i]);
         }
         pcm_configure_channels(&(t->pcm), params->pcm_use_16bits_timeslot, channel_to_timeslot, pcm_channel);
      }
      // We must start PCM before initializing phone devices
      // else phone devices will not detect PCLK and initialization
      // will fail
      pcm_start(&(t->pcm));

      ret = bcm_phone_mgr_start(&(t->phone_mgr), params->country, line_params, ARRAY_SIZE(line_params));
      if (ret) {
         pcm_stop(&(t->pcm));
      }
   } while (false);

   return (ret);
}

long bcm_drv_unlocked_ioctl(struct file *filp,
             unsigned int cmd, unsigned long arg)
{
   long ret = 0;
   bcm_drv_t *t = filp->private_data;
   bcmph_mutex_t lock;

   dd_bcm_pr_debug("bcm_drv_unlocked_ioctl(cmd %u)\n", (unsigned int)(cmd));

   bcm_assert(NULL != t);

   bcmph_mutex_init(&(lock), &(t->lock), false);

   do { // Boucle vide
      if (bcmph_mutex_lock_interruptible(&(lock))) {
         ret = -ERESTARTSYS;
         break;
      }
      switch (cmd) {
         case BCMPH_IOCTL_READ_MM: {
            size_t line = arg & 0xFF;
            size_t len = arg >> 8;

            dd_bcm_pr_debug("BCMPH_IOCTL_READ_MM\n");

            if ((line >= bcm_phone_mgr_get_line_count(&(t->phone_mgr)))
                || (!bcm_phone_mgr_line_is_enabled(&(t->phone_mgr), line))) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid line %lu\n",
                  (unsigned int)(cmd), (unsigned long)(line));
               ret = -EFAULT;
               break;
            }
            if (len > bcm_ring_buf_get_size(&(t->phone_lines[line].rx_ring_buf))) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid len %lu\n",
                  (unsigned int)(cmd), (unsigned long)(len));
               ret = -EFAULT;
               break;
            }

            bcm_ring_buf_remove_len(&(t->phone_lines[line].rx_ring_buf), len);
            bcm_drv_update_buffers(t);
            bcm_drv_update_ring_buf_desc(t);
            break;
         }
         case BCMPH_IOCTL_WRITE_MM: {
            size_t line = arg & 0xFF;
            size_t len = arg >> 8;

            dd_bcm_pr_debug("BCMPH_IOCTL_WRITE_MM\n");

            if ((line >= bcm_phone_mgr_get_line_count(&(t->phone_mgr)))
                || (!bcm_phone_mgr_line_is_enabled(&(t->phone_mgr), line))) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid line %lu\n",
                  (unsigned int)(cmd), (unsigned long)(line));
               ret = -EFAULT;
               break;
            }
            if (len > bcm_ring_buf_get_free_space(&(t->phone_lines[line].tx_ring_buf))) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid len %lu\n",
                  (unsigned int)(cmd), (unsigned long)(len));
               ret = -EFAULT;
               break;
            }

            bcm_ring_buf_add_len(&(t->phone_lines[line].tx_ring_buf), len);
            bcm_drv_update_buffers(t);
            bcm_drv_update_ring_buf_desc(t);
            break;
         }
         case BCMPH_IOCTL_UPDATE_RBS: {
            if (!pcm_is_started(&(t->pcm))) {
               ret = -EFAULT;
               break;
            }
            bcm_drv_update_buffers(t);
            bcm_drv_update_ring_buf_desc(t);
            break;
         }
         case BCMPH_IOCTL_GET_LINE_STATES: {
            bcm_phone_get_line_states_t get_line_states;

            dd_bcm_pr_debug("BCMPH_IOCTL_GET_LINE_STATES\n");

            if (_IOC_SIZE(cmd) != sizeof(get_line_states)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(get_line_states)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(get_line_states), (void *)(arg), sizeof(get_line_states)) > 0) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : copy_from_user failed\n", (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_get_line_states(t, &(get_line_states), &(lock));
            if (!ret) {
               if (copy_to_user((void *)(arg), &(get_line_states), sizeof(get_line_states)) > 0) {
                  bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : copy_to_user failed\n", (unsigned int)(cmd));
                  ret = -EFAULT;
                  break;
               }
            }
            break;
         }
         case BCMPH_IOCTL_SET_LINE_MODE: {
            bcm_phone_set_line_mode_t set_line_mode;

            d_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_MODE\n");

            if (_IOC_SIZE(cmd) != sizeof(set_line_mode)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(set_line_mode)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(set_line_mode), (void *)(arg), sizeof(set_line_mode)) > 0) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : copy_from_user failed\n", (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_mode(t, set_line_mode.line, set_line_mode.mode, set_line_mode.tone, set_line_mode.wait, &(lock));
            break;
         }
         case BCMPH_IOCTL_SET_LINE_CODEC: {
            bcm_phone_set_line_codec_t set_line_codec;

            d_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_CODEC\n");

            if (_IOC_SIZE(cmd) != sizeof(set_line_codec)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(set_line_codec)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(set_line_codec), (void *)(arg), sizeof(set_line_codec)) > 0) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : copy_from_user failed\n", (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_codec(t, set_line_codec.line,
               set_line_codec.codec, set_line_codec.mode,
               set_line_codec.tone, &(lock));
            break;
         }
         case BCMPH_IOCTL_SET_LINE_TONE: {
            bcm_phone_set_line_tone_t set_line_tone;

            d_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_TONE\n");

            if (_IOC_SIZE(cmd) != sizeof(set_line_tone)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(set_line_tone)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(set_line_tone), (void *)(arg), sizeof(set_line_tone)) > 0) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : copy_from_user failed\n", (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_tone(t, set_line_tone.line, set_line_tone.tone, set_line_tone.wait, &(lock));
            break;
         }
         case BCMPH_IOCTL_SET_LINE_MODE_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->mm_buffer + t->mm_desc.ioctl_param_off);

            d_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_MODE_MM\n");

            if (param->size != sizeof(param->p.set_line_mode)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.set_line_mode)));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_mode(t, param->p.set_line_mode.line,
               param->p.set_line_mode.mode, param->p.set_line_mode.tone,
               param->p.set_line_mode.wait, &(lock));
            break;
         }
         case BCMPH_IOCTL_SET_LINE_CODEC_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->mm_buffer + t->mm_desc.ioctl_param_off);

            d_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_CODEC_MM\n");

            if (param->size != sizeof(param->p.set_line_codec)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.set_line_codec)));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_codec(t, param->p.set_line_codec.line,
               param->p.set_line_codec.codec, param->p.set_line_codec.mode,
               param->p.set_line_codec.tone, &(lock));
            break;
         }
         case BCMPH_IOCTL_SET_LINE_TONE_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->mm_buffer + t->mm_desc.ioctl_param_off);

            d_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_TONE_MM\n");

            if (param->size != sizeof(param->p.set_line_tone)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.set_line_tone)));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_tone(t, param->p.set_line_tone.line, param->p.set_line_tone.tone, param->p.set_line_tone.wait, &(lock));
            break;
         }
         case BCMPH_IOCTL_START: {
            bcm_phone_cfg_params_t cfg_params;

            bcm_pr_debug("BCMPH_IOCTL_START\n");

            if (_IOC_SIZE(cmd) != sizeof(cfg_params)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(cfg_params)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(cfg_params), (void *)(arg), sizeof(cfg_params)) > 0) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : copy_from_user failed\n", (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_start(t, &(cfg_params));
            break;
         }
         case BCMPH_IOCTL_STOP: {
            bcm_pr_debug("BCMPH_IOCTL_STOP\n");
            bcm_drv_stop(t, 1);
            break;
         }
         case BCMPH_IOCTL_START_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->mm_buffer + t->mm_desc.ioctl_param_off);

            bcm_pr_debug("BCMPH_IOCTL_START_MM\n");

            if (param->size != sizeof(param->p.start)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.start)));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_start(t, &(param->p.start));
            break;
         }
         case BCMPH_IOCTL_GET_MMAP_RBS_LOCATION: {
            bcm_pr_debug("BCMPH_IOCTL_GET_MMAP_RBS_LOCATION\n");

            if (_IOC_SIZE(cmd) != sizeof(t->mm_rbs_location)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(t->mm_rbs_location)));
               ret = -EFAULT;
               break;
            }
            if (!pcm_is_started(&(t->pcm))) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : pcm is not started\n",
                  (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            if (copy_to_user((void *)(arg), &(t->mm_rbs_location), sizeof(t->mm_rbs_location)) > 0) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : copy_to_user failed\n", (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            break;
         }
         case BCMPH_IOCTL_GET_MMAP_RBS_LOCATION_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->mm_buffer + t->mm_desc.ioctl_param_off);

            bcm_pr_debug("BCMPH_IOCTL_GET_MMAP_RBS_LOCATION_MM\n");

            if (param->size != sizeof(param->p.get_mmap_rbs_location)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.get_mmap_rbs_location)));
               ret = -EFAULT;
               break;
            }
            if (!pcm_is_started(&(t->pcm))) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : pcm is not started\n",
                  (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            memcpy(&(param->p.get_mmap_rbs_location), &(t->mm_rbs_location), sizeof(param->p.get_mmap_rbs_location));
            break;
         }
         case BCMPH_IOCTL_START_PCM: {
            bcm_pr_debug("BCMPH_IOCTL_START_PCM\n");
            bcm_drv_start_pcm(t);
            break;
         }
         case BCMPH_IOCTL_STOP_PCM: {
            bcm_pr_debug("BCMPH_IOCTL_STOP_PCM\n");
            bcm_drv_stop_pcm(t, 1);
            break;
         }
         case BCMPH_IOCTL_GET_MMAP_DESC: {
            bcm_pr_debug("BCMPH_IOCTL_GET_MMAP_DESC\n");

            if (_IOC_SIZE(cmd) != sizeof(t->mm_desc)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(t->mm_desc)));
               ret = -EFAULT;
               break;
            }
            if (copy_to_user((void *)(arg), &(t->mm_desc), sizeof(t->mm_desc)) > 0) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : copy_to_user failed\n", (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            break;
         }
         case BCMPH_IOCTL_SET_DEFAULT_LINE: {
            size_t line = arg;

            d_bcm_pr_debug("BCMPH_IOCTL_SET_DEFAULT_LINE\n");

            if ((line >= bcm_phone_mgr_get_line_count(&(t->phone_mgr)))
                || ((pcm_is_started(&(t->pcm))) && (!bcm_phone_mgr_line_is_enabled(&(t->phone_mgr), line)))) {
               ret = -EFAULT;
               break;
            }

            t->default_line = line;
            break;
         }
         case BCMPH_IOCTL_READ: {
            bcm_phone_read_t read;

            d_bcm_pr_debug("BCMPH_IOCTL_READ\n");

            if (_IOC_SIZE(cmd) != sizeof(read)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(read)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(read), (void *)(arg), sizeof(read)) > 0) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : copy_from_user failed\n", (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            if (read.line >= bcm_phone_mgr_get_line_count(&(t->phone_mgr))) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid line %lu\n",
                  (unsigned int)(cmd), (unsigned long)(read.line));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_read_line(t, read.do_not_block, &(lock), read.line, read.buf, read.count);
            break;
         }
         case BCMPH_IOCTL_WRITE: {
            bcm_phone_write_t write;

            d_bcm_pr_debug("BCMPH_IOCTL_WRITE\n");

            if (_IOC_SIZE(cmd) != sizeof(write)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(write)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(write), (void *)(arg), sizeof(write)) > 0) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : copy_from_user failed\n", (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            if (write.line >= bcm_phone_mgr_get_line_count(&(t->phone_mgr))) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid line %lu\n",
                  (unsigned int)(cmd), (unsigned long)(write.line));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_write_line(t, write.do_not_block, &(lock), write.line, write.buf, write.count);
            break;
         }
         case BCMPH_IOCTL_GET_LINE_STATES_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->mm_buffer + t->mm_desc.ioctl_param_off);

            dd_bcm_pr_debug("BCMPH_IOCTL_GET_LINE_STATES_MM\n");

            if (param->size != sizeof(param->p.get_line_states)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.get_line_states)));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_get_line_states(t, &(param->p.get_line_states), &(lock));
            break;
         }
         case BCMPH_IOCTL_READ_VERSION: {
            bcm_phone_drv_ver_t ver;

            bcm_pr_debug("BCMPH_IOCTL_READ_VERSION\n");

            if (_IOC_SIZE(cmd) != sizeof(ver)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(ver)));
               ret = -EFAULT;
               break;
            }
            bcm_drv_read_version(t, &(ver));
            if (copy_to_user((void *)(arg), &(ver), sizeof(ver)) > 0) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : copy_to_user failed\n", (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            break;
         }
         case BCMPH_IOCTL_READ_VERSION_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->mm_buffer + t->mm_desc.ioctl_param_off);

            bcm_pr_debug("BCMPH_IOCTL_READ_VERSION_MM\n");

            if (param->size != sizeof(param->p.read_version)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.read_version)));
               ret = -EFAULT;
               break;
            }
            bcm_drv_read_version(t, &(param->p.read_version));
            break;
         }
         case BCMPH_IOCTL_READ_PCM_REGS: {
            // Reading register when PCM is started may return
            // inaccurate values for example slot_alloc_tbl[x]
            bcm_phone_pcm_regs_t regs;

            d_bcm_pr_debug("BCMPH_IOCTL_READ_PCM_REGS\n");

            if (_IOC_SIZE(cmd) != sizeof(regs)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(regs)));
               ret = -EFAULT;
               break;
            }
            pcm_read_regs(&(t->pcm), &(regs));
            if (copy_to_user((void *)(arg), &(regs), sizeof(regs)) > 0) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : copy_to_user failed\n", (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            break;
         }
         case BCMPH_IOCTL_READ_PCM_REGS_MM: {
            // Reading register when PCM is started may return
            // inaccurate values for example slot_alloc_tbl[x]
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->mm_buffer + t->mm_desc.ioctl_param_off);

            d_bcm_pr_debug("BCMPH_IOCTL_READ_PCM_REGS_MM\n");

            if (param->size != sizeof(param->p.read_pcm_regs)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.read_pcm_regs)));
               ret = -EFAULT;
               break;
            }
            pcm_read_regs(&(t->pcm), &(param->p.read_pcm_regs));
            break;
         }
         case BCMPH_IOCTL_READ_PCM_STATS: {
            bcm_phone_pcm_stats_t stats;

            d_bcm_pr_debug("BCMPH_IOCTL_READ_PCM_STATS\n");

            if (_IOC_SIZE(cmd) != sizeof(stats)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(stats)));
               ret = -EFAULT;
               break;
            }
            pcm_read_stats(&(t->pcm), &(stats));
            if (copy_to_user((void *)(arg), &(stats), sizeof(stats)) > 0) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : copy_to_user failed\n", (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            break;
         }
         case BCMPH_IOCTL_READ_PCM_STATS_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->mm_buffer + t->mm_desc.ioctl_param_off);

            d_bcm_pr_debug("BCMPH_IOCTL_READ_PCM_STATS_MM\n");

            if (param->size != sizeof(param->p.read_pcm_stats)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.read_pcm_stats)));
               ret = -EFAULT;
               break;
            }
            pcm_read_stats(&(t->pcm), &(param->p.read_pcm_stats));
            break;
         }
#ifdef BCMPH_TEST_PCM
         case BCMPH_IOCTL_SET_LINE_STATE: {
            bcm_phone_set_line_state_t set_line_state;

            dd_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_STATE\n");

            if (_IOC_SIZE(cmd) != sizeof(set_line_state)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(set_line_state)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(set_line_state), (void *)(arg), sizeof(set_line_state)) > 0) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : copy_from_user failed\n", (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_state(t, &(set_line_state));
            break;
         }
         case BCMPH_IOCTL_SET_LINE_STATE_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->mm_buffer + t->mm_desc.ioctl_param_off);

            d_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_STATE_MM\n");

            if (param->size != sizeof(param->p.set_line_state)) {
               bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : invalid parameter size %lu (%lu expected)\n",
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.set_line_state)));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_state(t, &(param->p.set_line_state));
            break;
         }
#endif // BCMPH_TEST_PCM
         default: {
            bcm_pr_err("bcm_drv_unlocked_ioctl(cmd=%u) : no such command\n",
               (unsigned int)(cmd));
            ret = -ENOTTY;
            break;
         }
      }
   } while (false);

   bcmph_mutex_deinit(&(lock));

   if (ret) {
      d_bcm_pr_debug("bcm_drv_unlocked_ioctl(cmd %u) failed -> %d\n", (unsigned int)(cmd), (int)(ret));
   }

   return (ret);
}

