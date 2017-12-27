/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <config.h>

#include <extern/linux/barrier.h>
#include <extern/linux/delay.h>
#include <extern/linux/errno.h>
#include <extern/linux/kernel.h>
#include <extern/linux/jiffies.h>


#include <bcm63xx_phone.h>

#include "main.h"
#include <bcm63xx_log.h>
#include <utils.h>

// Include after system files
#include <compile.h>

#define BCMPH_NULL_BYTE_ALAW (0xd5)
#define BCMPH_NULL_BYTE_ULAW (0xff)

static void _bcm_drv_forget_rx_data(bcm_drv_t *t, bcm_core_drv_phone_line_t *dpl)
{
   dd_bcm_pr_debug("%s()\n", __func__);

   bcm_ring_buf_clear(&(dpl->rx_ring_buf));
   // We reset intermediate RX buffer
   dpl->rx_buffer.len = 0;
#if ((defined __KERNEL__) && (defined BCMPH_NOHW))
   bcm_ring_buf_clear(&(t->debug.write_ring_buf));
#endif // __KERNEL__ && BCMPH_NOHW
}

static void _bcm_drv_forget_tx_data(bcm_drv_t *t, bcm_core_drv_phone_line_t *dpl)
{
   dd_bcm_pr_debug("%s()\n", __func__);

   bcm_ring_buf_clear(&(dpl->tx_ring_buf));
   dpl->tx_buffer.offset = 0;
   dpl->tx_buffer.len = 0;
#if ((defined __KERNEL__) && (defined BCMPH_NOHW))
   bcm_ring_buf_clear(&(t->debug.read_ring_buf));
#endif // __KERNEL__ && BCMPH_NOHW
}

void bcm_drv_forget_rx_data(bcm_drv_t *t, size_t line)
{
   bcm_assert(/* (line >= 0) && */(line < bcm_phone_mgr_get_line_count(&(t->core.phone_mgr))));

   dd_bcm_pr_debug("%s()\n", __func__);

   _bcm_drv_forget_rx_data(t, &(t->core.phone_lines[line]));
}

void bcm_drv_forget_tx_data(bcm_drv_t *t, size_t line)
{
   bcm_assert(/* (line >= 0) && */(line < bcm_phone_mgr_get_line_count(&(t->core.phone_mgr))));

   dd_bcm_pr_debug("%s()\n", __func__);

   _bcm_drv_forget_tx_data(t, &(t->core.phone_lines[line]));
}


static void bcm_drv_set_bytes_per_frame(bcm_drv_t *t, bcm_core_drv_phone_line_t *dpl,
   bool pcm_use_16bits_timeslot, bcm_phone_codec_t codec)
{
   if (!pcm_use_16bits_timeslot) {
      if ((BCMPH_CODEC_ALAW == codec) || (BCMPH_CODEC_ULAW == codec)) {
         dpl->type_transfer = BCMPH_TRF_M_8_BITS_C_xLAW;
         // We need 1 timeslot/channel
         // Data is in first channel of the first PCM block
         dpl->bytes_per_frame = BCMPH_PCM_CHANNEL_WIDTH;
         if (BCMPH_CODEC_ALAW == codec) {
            dpl->null_byte = BCMPH_NULL_BYTE_ALAW;
         }
         else {
            dpl->null_byte = BCMPH_NULL_BYTE_ULAW;
         }
      }
      else if (BCMPH_CODEC_LINEAR == codec) {
         dpl->type_transfer = BCMPH_TRF_M_8_BITS_C_LINEAR;
         // We need 2 timeslots/channels
         // Data is in first and second channels of the first PCM block
         dpl->bytes_per_frame = BCMPH_PCM_CHANNEL_WIDTH * 2;
         dpl->null_byte = 0;
      }
      else if (BCMPH_CODEC_LINEAR16 == codec) {
         dpl->type_transfer = BCMPH_TRF_M_8_BITS_C_LINEAR16;
         // We need 4 timeslots/channels
         // Data is in first and second channels of the first PCM block
         // and in the first and second channels of the second PCM block
         dpl->bytes_per_frame = BCMPH_PCM_CHANNEL_WIDTH * 4;
         dpl->null_byte = 0;
      }
      else {
         bcm_assert((BCMPH_CODEC_ALAW16 == codec) || (BCMPH_CODEC_ULAW16 == codec));
         dpl->type_transfer = BCMPH_TRF_M_8_BITS_C_xLAW16;
         // We need 2 timeslots/channel
         // Data is in first channel of the first PCM block and in the first
         // channel of the second PCM block
         dpl->bytes_per_frame = BCMPH_PCM_CHANNEL_WIDTH * 2;
         if (BCMPH_CODEC_ALAW16 == codec) {
            dpl->null_byte = BCMPH_NULL_BYTE_ALAW;
         }
         else {
            dpl->null_byte = BCMPH_NULL_BYTE_ULAW;
         }
      }
   }
   else {
      if ((BCMPH_CODEC_ALAW == codec) || (BCMPH_CODEC_ULAW == codec)) {
         dpl->type_transfer = BCMPH_TRF_M_16_BITS_C_xLAW;
         // We need 1 timeslot/channel but we get/set only half of the bytes
         // Data is in first channel of the first PCM block
         dpl->bytes_per_frame = (BCMPH_PCM_CHANNEL_WIDTH + 1) / 2;
         if (BCMPH_CODEC_ALAW == codec) {
            dpl->null_byte = BCMPH_NULL_BYTE_ALAW;
         }
         else {
            dpl->null_byte = BCMPH_NULL_BYTE_ULAW;
         }
      }
      else if (BCMPH_CODEC_LINEAR == codec) {
         dpl->type_transfer = BCMPH_TRF_M_16_BITS_C_LINEAR;
         // We need 1 timeslot/channel
         // Data is in first channel of the first PCM block
         dpl->bytes_per_frame = BCMPH_PCM_CHANNEL_WIDTH;
         dpl->null_byte = 0;
      }
      else if (BCMPH_CODEC_LINEAR16 == codec) {
         dpl->type_transfer = BCMPH_TRF_M_16_BITS_C_LINEAR16;
         // We need 2 timeslots/channels
         // Data is in first channel of the first PCM block and in the first
         // channel of the second PCM block
         dpl->bytes_per_frame = ((BCMPH_PCM_CHANNEL_WIDTH + 1) / 2) * 4;
         dpl->null_byte = 0;
      }
      else {
         bcm_assert((BCMPH_CODEC_ALAW16 == codec) || (BCMPH_CODEC_ULAW16 == codec));
         dpl->type_transfer = BCMPH_TRF_M_16_BITS_C_xLAW16;
         // We need 2 timeslots/channel but we get/set only half of the
         // bytes of each channel
         // Data is in first channel of the first PCM block and in the first
         // channel of the second PCM block
         dpl->bytes_per_frame = ((BCMPH_PCM_CHANNEL_WIDTH + 1) / 2) * 2;
         if (BCMPH_CODEC_ALAW16 == codec) {
            dpl->null_byte = BCMPH_NULL_BYTE_ALAW;
         }
         else {
            dpl->null_byte = BCMPH_NULL_BYTE_ULAW;
         }
      }
   }
#if ((defined BCMPH_TEST_PCM) && (!defined BCMPH_NOHW))
   dpl->offset_first_block_pcm_channel = 0;
   dpl->type_transfer = BCMPH_TRF_TEST_PCM;
   dpl->bytes_per_frame = BCMPH_PCM_MAX_FRAME_SIZE;
#endif // BCMPH_TEST_PCM && !BCMPH_NOHW

   d_bcm_pr_debug("%s(pcm_use_16bits_timeslot=%d, codec=%d) => type_transfer=%d, bytes_per_frame=%lu\n",
      __func__, (int)(pcm_use_16bits_timeslot), (int)(codec),
      (int)(dpl->type_transfer), (unsigned long)(dpl->bytes_per_frame));
}

#ifndef BCMPH_DAHDI_DRIVER
static void bcm_drv_set_echo_cancellation(bcm_drv_t *t,
   size_t line, int echo_cancellation)
{
   d_bcm_pr_debug("%s(line=%lu, echo_cancellation=%d)\n", __func__,
      (unsigned long)(line), (int)(echo_cancellation));

   if (echo_cancellation >= 0) {
      bcm_core_drv_phone_line_t *dpl;

      bcm_assert(/*(line >= 0) && */(line < bcm_phone_mgr_get_line_count(&(t->core.phone_mgr))));

      dpl = &(t->core.phone_lines[line]);

      bcm_assert((NULL != dpl->oslec_state) || (!dpl->cancel_echo));

      if (NULL != dpl->oslec_state) {
         if (echo_cancellation > 0) {
            if (!dpl->cancel_echo) {
               bcm_pr_debug("Enabling echo cancellation for line %lu\n",
                  (unsigned long)(line));
               oslec_flush(dpl->oslec_state);
               dpl->cancel_echo = true;
            }
         }
         else {
            if (dpl->cancel_echo) {
               bcm_pr_debug("Disabling echo cancellation for line %lu\n",
                  (unsigned long)(line));
               dpl->cancel_echo = false;
            }
         }
      }
   }
}
#endif /* !BCMPH_DAHDI_DRIVER */

int bcm_drv_set_line_mode(bcm_drv_t *t, size_t line,
   bcm_phone_line_mode_t mode
#ifndef BCMPH_DAHDI_DRIVER
   , int echo_cancellation
#endif /* !BCMPH_DAHDI_DRIVER */
   , int reverse_polarity, __u32 tone
#ifdef BCMPH_EXPORT_DEV_FILE
   , int wait, bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   )
{
   int ret = 0;

   d_bcm_pr_debug("%s(line=%lu, mode=%d"
#ifndef BCMPH_DAHDI_DRIVER
      ", echo_cancellation=%d"
#endif /* !BCMPH_DAHDI_DRIVER */
      ", reverse_polarity=%d, tone=0x%lx"
#ifdef BCMPH_EXPORT_DEV_FILE
      ", wait=%d"
#endif /* BCMPH_EXPORT_DEV_FILE */
      ")\n", __func__, (unsigned long)(line), (int)(mode)
#ifndef BCMPH_DAHDI_DRIVER
      , (int)(echo_cancellation)
#endif /* !BCMPH_DAHDI_DRIVER */
      , (int)(reverse_polarity), (unsigned long)(tone)
#ifdef BCMPH_EXPORT_DEV_FILE
      , (int)(wait)
#endif /* BCMPH_EXPORT_DEV_FILE */
      );

#ifdef BCMPH_EXPORT_DEV_FILE
   bcm_assert(((NULL == lock) && (0 == wait)
      && (BCMPH_MODE_OFF_TALKING != mode) && (BCMPH_MODE_ON_TALKING != mode))
      || ((NULL != lock) && (bcmph_mutex_is_locked(lock))));
#endif /* BCMPH_EXPORT_DEV_FILE */

   do { // Empty loop

      if ((line >= bcm_phone_mgr_get_line_count(&(t->core.phone_mgr)))
          || (!bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line))
          || (((BCMPH_MODE_UNSPECIFIED == mode) || (mode >= BCMPH_MAX_MODES))
              && (reverse_polarity < 0))) {
         bcm_pr_err("Invalid line or invalid mode\n");
         ret = -EINVAL;
         break;
      }

      if ((BCMPH_MODE_OFF_TALKING == mode) || (BCMPH_MODE_ON_TALKING == mode)) {
         // For these two modes we always wait that the mode really changes
#ifdef BCMPH_EXPORT_DEV_FILE
         bcm_assert((NULL != lock) && (bcmph_mutex_is_locked(lock)));
#endif /* BCMPH_EXPORT_DEV_FILE */
         ret = bcm_phone_mgr_set_line_mode(&(t->core.phone_mgr), line,
            mode, reverse_polarity, tone
#ifdef BCMPH_EXPORT_DEV_FILE
            , -1, lock
#endif /* BCMPH_EXPORT_DEV_FILE */
            );
         // If set_line_mode failed it's not dramatic
         {
            // It's just a precaution, but must already been done when starting and when codec changes
            bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line]);
            bcm_drv_set_bytes_per_frame(t, dpl, pcm_timeslot_is_16bits(&(t->core.pcm)),
               bcm_phone_mgr_get_current_codec(&(t->core.phone_mgr), line));
            _bcm_drv_forget_rx_data(t, dpl);
         }

         // We start PCM if it's not already started
         bcm_drv_start_pcm(t);
      }
      else {
         ret = bcm_phone_mgr_set_line_mode(&(t->core.phone_mgr), line,
            mode, reverse_polarity, tone
#ifdef BCMPH_EXPORT_DEV_FILE
            , wait, lock
#endif /* BCMPH_EXPORT_DEV_FILE */
            );
      }

#ifdef BCMPH_EXPORT_DEV_FILE
      if ((NULL == lock) || (bcmph_mutex_is_locked(lock))) {
         // Wake up processes blocked in inq and outq because the mode change
         // may reduce the sets of lines using the PCM bus
         // That may unblock sending and receiving of data
#endif /* BCMPH_EXPORT_DEV_FILE */
#ifndef BCMPH_DAHDI_DRIVER
         if (!ret) {
            bcm_drv_set_echo_cancellation(t, line, echo_cancellation);
         }
#endif /* !BCMPH_DAHDI_DRIVER */
         bcm_drv_update_buffers(t);
#ifdef BCMPH_EXPORT_DEV_FILE
      }
      else {
#ifndef BCMPH_DAHDI_DRIVER
         bcm_assert(echo_cancellation < 0);
#endif /* !BCMPH_DAHDI_DRIVER */
      }
#endif /* BCMPH_EXPORT_DEV_FILE */
   }
   while (false);

   return (ret);
}

int bcm_drv_set_line_tone(bcm_drv_t *t, size_t line, __u32 tone
#ifdef BCMPH_EXPORT_DEV_FILE
   , int wait, bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   )
{
   int ret = 0;

   d_bcm_pr_debug("%s(line=%lu, tone=0x%lx"
#ifdef BCMPH_EXPORT_DEV_FILE
      ", wait=%d"
#endif /* BCMPH_EXPORT_DEV_FILE */
      ")\n", __func__, (unsigned long)(line), (unsigned long)(tone)
#ifdef BCMPH_EXPORT_DEV_FILE
      , (int)(wait)
#endif /* BCMPH_EXPORT_DEV_FILE */
      );

#ifdef BCMPH_EXPORT_DEV_FILE
   bcm_assert(((NULL == lock) && (0 == wait))
      || ((NULL != lock) && (bcmph_mutex_is_locked(lock))));
#endif /* BCMPH_EXPORT_DEV_FILE */

   do { // Empty loop
      bcm_phone_line_tone_t tone_index = bcm_phone_line_tone_decode_index(tone);

      if ((line >= bcm_phone_mgr_get_line_count(&(t->core.phone_mgr)))
          || (!bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line))
          || (BCMPH_TONE_UNSPECIFIED == tone_index)
          || (tone_index >= BCMPH_MAX_TONES)) {
         bcm_pr_err("Invalid line or invalid tone\n");
         ret = -EINVAL;
         break;
      }

      ret = bcm_phone_mgr_set_line_tone(&(t->core.phone_mgr), line, tone
#ifdef BCMPH_EXPORT_DEV_FILE
         , wait, lock
#endif /* BCMPH_EXPORT_DEV_FILE */
         );
   }
   while (false);

   return (ret);
}

void bcm_drv_start_pcm(bcm_drv_t *t)
{
   d_bcm_pr_debug("%s()\n", __func__);

   if (!pcm_dma_is_started(&(t->core.pcm))) {
#ifdef BCMPH_EXPORT_DEV_FILE
      bool wake_up_outq = false;
#endif /* BCMPH_EXPORT_DEV_FILE */
      size_t line_idx;
      unsigned long time_to_send_buffer_in_us;
      size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->core.phone_mgr));

      for (line_idx = 0; (line_idx < phone_line_count); line_idx += 1) {
         if (bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line_idx)) {
            bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line_idx]);

            bcm_assert(
               ARRAY_SIZE(dpl->rx_buffer.array) >= (pcm_get_max_frame_rx_buffer(&(t->core.pcm)) * dpl->bytes_per_frame));
            bcm_assert(
               ARRAY_SIZE(dpl->tx_buffer.array) >= ((pcm_get_max_frame_rx_buffer(&(t->core.pcm)) + pcm_get_max_frame_tx_buffer(&(t->core.pcm))) * dpl->bytes_per_frame));

            /*
             Clear RX buffers in order to delete obsolete data
            */
            _bcm_drv_forget_rx_data(t, dpl);

            /*
             Do not clear data in TX ring buffer as we may push data
             before starting PCM
            */
            dpl->tx_buffer.offset = 0;
            dpl->tx_buffer.len = 0;
         }
      }


      /*
       We fill some TX DMA buffers that will be transmitted before we
       handle the first RX buffer.
       Must be at least tick_period ms.
       But must also be the shortest possible because of echo
       cancellation
       To be comfortable we set (tick_period ms + one jiffy)
      */
      time_to_send_buffer_in_us = pcm_get_time_to_send_tx_buffer(&(t->core.pcm));
      t->core.dma_frames_to_send =
         ((((t->core.board_desc->phone_desc->tick_period * 1000UL) + jiffies_to_usecs(1) + (time_to_send_buffer_in_us - 1))
           / time_to_send_buffer_in_us)
          * pcm_get_max_frame_tx_buffer(&(t->core.pcm)));
#ifdef BCMPH_NOHW
      // Check that there is at least 2 buffers
      if (t->core.dma_frames_to_send <= pcm_get_max_frame_tx_buffer(&(t->core.pcm))) {
         t->core.dma_frames_to_send = pcm_get_max_frame_tx_buffer(&(t->core.pcm)) * 2;
      }
#endif // BCMPH_NOHW

      pcm_dma_start(&(t->core.pcm));

      bcm_drv_pcm_send(t
#ifdef BCMPH_EXPORT_DEV_FILE
         , &(wake_up_outq)
#endif // BCMPH_EXPORT_DEV_FILE
         );
#ifdef BCMPH_EXPORT_DEV_FILE
      if (wake_up_outq) {
         // Data received so we unblock processes waiting to read data
         bcm_wait_queue_wake_up(&(t->dev.outq));
      }
#endif /* BCMPH_EXPORT_DEV_FILE */

      // Perhaps there's not enough TX DMA buffers available
      t->core.dma_frames_to_send = 0;

      bcm_drv_update_buffers(t);
   }
}

void bcm_drv_stop_pcm(bcm_drv_t *t, bool wait)
{
   d_bcm_pr_debug("%s()\n", __func__);

   if (pcm_dma_is_started(&(t->core.pcm))) {
      size_t line_idx;
      size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->core.phone_mgr));
      bool can_stop_pcm = true;

      // Checks that no line use pcm
      for (line_idx = 0; (line_idx < phone_line_count); line_idx += 1) {
         if ((bcm_phone_mgr_line_tx_use_pcm(&(t->core.phone_mgr), line_idx))
             || (bcm_phone_mgr_line_rx_use_pcm(&(t->core.phone_mgr), line_idx))) {
            can_stop_pcm = false;
            break;
         }
      }

      if (can_stop_pcm) {
         if (wait) {
            /*
             We ignore data in TX ring buffers because we are not in
             a mode that allows to consume it
             We only wait until there's no buffer owned by DMA
            */
            unsigned int delay = (pcm_get_time_to_send_tx_buffer(&(t->core.pcm))
               + jiffies_to_usecs(1) + 999) / 1000;

            bcm_drv_update_buffers(t);

            if (pcm_get_cnt_tx_buffer_really_owned(&(t->core.pcm)) > 0) {
               d_bcm_pr_debug("Waiting all DMA buffers to be transmitted\n");
               for (;;) {
                  d_bcm_pr_debug("Waiting %u ms\n", (unsigned int)(delay));
                  msleep(delay);
                  if (pcm_tx_reclaim(&(t->core.pcm), 1) <= 0) {
                     if (pcm_get_cnt_tx_buffer_really_owned(&(t->core.pcm)) > 0) {
                        bcm_pr_debug("Not all DMA buffers are transmitted.\n");
                     }
                     break;
                  }
               }
            }
         }
         pcm_dma_stop(&(t->core.pcm));
         for (line_idx = 0; (line_idx < phone_line_count); line_idx += 1) {
            bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line_idx]);
            /*
             Clear data in TX ring buffers in order to not transmit
             obsolete data when we restart PCM.
             RX ring buffers are not cleared now, because user can read
             data after PCM is stopped. They will be cleared when we
             restart PCM.
             */
            _bcm_drv_forget_tx_data(t, dpl);
         }
      }
      else {
         d_bcm_pr_debug("PCM not stopped because at least one line uses it.\n");
      }
   }
}

int bcm_drv_set_line_codec(bcm_drv_t *t, size_t line,
   bcm_phone_codec_t codec, bcm_phone_line_mode_t mode
#ifndef BCMPH_DAHDI_DRIVER
   , int echo_cancellation
#endif /* !BCMPH_DAHDI_DRIVER */
   , int reverse_polarity, __u32 tone
#ifdef BCMPH_EXPORT_DEV_FILE
   , bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   )
{
   int ret = 0;

   d_bcm_pr_debug("%s(line=%lu, codec=%d, mode=%d"
#ifndef BCMPH_DAHDI_DRIVER
      ", echo_cancellation=%d"
#endif /* !BCMPH_DAHDI_DRIVER */
      ", reverse_polarity=%d, tone=0x%lx)\n", __func__,
      (unsigned long)(line), (int)(codec), (int)(mode)
#ifndef BCMPH_DAHDI_DRIVER
      , (int)(echo_cancellation)
#endif /* !BCMPH_DAHDI_DRIVER */
      , (int)(reverse_polarity),
      (unsigned long)(tone));

#ifdef BCMPH_EXPORT_DEV_FILE
   bcm_assert((NULL != lock) && (bcmph_mutex_is_locked(lock)));
#endif /* BCMPH_EXPORT_DEV_FILE */

   bcm_assert((BCMPH_MODE_UNSPECIFIED == mode)
      || (BCMPH_MODE_ON_TALKING == mode)
      || (BCMPH_MODE_OFF_TALKING == mode));

   do { // Empty loop
      bcm_phone_codec_t current_codec;
      bcm_phone_line_mode_t current_mode;
      bcm_core_drv_phone_line_t *dpl;

      if ((line > bcm_phone_mgr_get_line_count(&(t->core.phone_mgr)))
          || (!bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line))) {
         bcm_pr_err("Invalid line\n");
         ret = -EINVAL;
         break;
      }

      dpl = &(t->core.phone_lines[line]);

      if ((BCMPH_CODEC_ALAW != codec) && (BCMPH_CODEC_ULAW != codec)
          && (BCMPH_CODEC_LINEAR != codec)
          && (BCMPH_CODEC_LINEAR16 != codec)
          && (BCMPH_CODEC_ALAW16 != codec)
          && (BCMPH_CODEC_ULAW16 != codec)) {
         bcm_pr_err("Invalid codec %d\n", (int)(codec));
         ret = -EINVAL;
         break;
      }

      if (!bcm_phone_mgr_line_supports_codec(&(t->core.phone_mgr), line, codec)) {
         bcm_pr_err("Device to which the line %lu belongs does not support codec %d\n", (unsigned long)(line), (int)(codec));
         ret = -EINVAL;
         break;
      }

#ifndef BCMPH_DAHDI_DRIVER
      if ((NULL != dpl->oslec_state) && (BCMPH_CODEC_LINEAR != codec)) {
         bcm_pr_err("Echo cancellation can be enabled for line %lu so no other codec than linear is allowed\n", (unsigned long)(line));
         ret = -EINVAL;
         break;
      }
#endif /* BCMPH_DAHDI_DRIVER */

      current_codec = bcm_phone_mgr_get_current_codec(&(t->core.phone_mgr), line);

      if (!bcm_phone_mgr_line_can_switch_to_codec(&(t->core.phone_mgr), line, current_codec, codec)) {
         bcm_pr_err("Line %lu can't be switched to codec %d\n", (unsigned long)(line), (int)(codec));
         ret = -EINVAL;
         break;
      }

      // Here if codec is supported, it means we already reserved the right timeslots and PCM channels for the line
      // so we don't do any verifications

      // Buffers must be emptied
      _bcm_drv_forget_rx_data(t, dpl);
      _bcm_drv_forget_tx_data(t, dpl);

      ret = bcm_phone_mgr_set_line_codec(&(t->core.phone_mgr), line, codec,
         mode, reverse_polarity, tone
#ifdef BCMPH_EXPORT_DEV_FILE
         , -1, lock
#endif /* BCMPH_EXPORT_DEV_FILE */
         );
      if (ret) {
         // Here we have a problem the process may have been interrupted
         // or we may have failed to lock the mutex, so we don't know
         // what the codec is really.
         bcm_pr_debug("%s() failed\n", __func__);
#ifdef BCMPH_EXPORT_DEV_FILE
         if (!bcmph_mutex_is_locked(lock)) {
            // We try to lock the mutex
            if (bcmph_mutex_lock_interruptible(lock)) {
               // We can't lock the mutex so we exit the function
               bcm_pr_err("Possible failure to set codec for line %lu (can't lock mutex)\n", (unsigned long)(line));
               break;
            }
         }
#endif /* BCMPH_EXPORT_DEV_FILE */
         // We check once again if there is a change pending for the line
         if (bcm_phone_mgr_line_has_change_pending(&(t->core.phone_mgr), line)) {
            // Alas there is, so we wait for more than
            // board_desc->phone_desc->tick_period ms (the timer period)
            // (for codec change, request is processed ASAP, at next
            // timer tick, not later)
            msleep((t->core.board_desc->phone_desc->tick_period * 5) / 2);
            // And we check a last time if there is a change pending for the line
            if (bcm_phone_mgr_line_has_change_pending(&(t->core.phone_mgr), line)) {
               // We give up : we don't know if the codec has been set or will be set
               bcm_pr_err("Possible failure to set codec for line %lu (change not acknowledged)\n", (unsigned long)(line));
            }
            else {
               ret = 0;
            }
         }
         else {
            ret = 0;
         }
      }
      current_codec = bcm_phone_mgr_get_current_codec(&(t->core.phone_mgr), line);
      if (codec != current_codec) {
         // If codec is not what we set this is an error, because codec
         // is not supposed to change anytime but only when we set it
         bcm_pr_err("Unexpected failure to set codec for line %lu\n", (unsigned long)(line));
         codec = current_codec;
         ret = -EFAULT;
      }

#ifndef BCMPH_DAHDI_DRIVER
      if (!ret) {
         bcm_drv_set_echo_cancellation(t, line, echo_cancellation);
      }
#endif /* !BCMPH_DAHDI_DRIVER */
      bcm_drv_set_bytes_per_frame(t, dpl, pcm_timeslot_is_16bits(&(t->core.pcm)), codec);
      // Buffers must be emptied
      _bcm_drv_forget_rx_data(t, dpl);
      _bcm_drv_forget_tx_data(t, dpl);

      current_mode = bcm_phone_mgr_get_current_mode(&(t->core.phone_mgr), line);
      if ((BCMPH_MODE_OFF_TALKING == current_mode) || (BCMPH_MODE_ON_TALKING == current_mode)) {
         // We start PCM if it's not already started
         bcm_drv_start_pcm(t);
      }

      bcm_drv_update_buffers(t);
   }
   while (false);

   return (ret);
}

int bcm_drv_set_line_rev_polarity(bcm_drv_t *t, size_t line,
   bool rev_polarity
#ifdef BCMPH_EXPORT_DEV_FILE
   , int wait, bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   )
{
   int ret = 0;

   bcm_pr_debug("%s(line=%lu, rev_polarity=%d"
#ifdef BCMPH_EXPORT_DEV_FILE
      ", wait=%d"
#endif /* BCMPH_EXPORT_DEV_FILE */
      ")\n", __func__, (unsigned long)(line), (int)(rev_polarity)
#ifdef BCMPH_EXPORT_DEV_FILE
      , (int)(wait)
#endif /* BCMPH_EXPORT_DEV_FILE */
      );

#ifdef BCMPH_EXPORT_DEV_FILE
   bcm_assert(((NULL == lock) && (0 == wait))
      || ((NULL != lock) && (bcmph_mutex_is_locked(lock))));
#endif /* BCMPH_EXPORT_DEV_FILE */

   do { // Empty loop
      if ((line >= bcm_phone_mgr_get_line_count(&(t->core.phone_mgr)))
          || (!bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line))) {
         bcm_pr_err("Invalid line\n");
         ret = -EINVAL;
         break;
      }

      ret = bcm_phone_mgr_set_line_rev_polarity(&(t->core.phone_mgr), line, rev_polarity
#ifdef BCMPH_EXPORT_DEV_FILE
         , wait, lock
#endif /* BCMPH_EXPORT_DEV_FILE */
         );
   }
   while (false);

   return (ret);
}

#ifdef BCMPH_EXPORT_DEV_FILE
static bool bcm_drv_one_line_changes_state(bcm_phone_line_state_t *line_states, size_t line_count)
{
   bool ret = false;
   size_t line_idx;

   dd_bcm_pr_debug("%s()\n", __func__);

   for (line_idx = 0; (line_idx < line_count); line_idx += 1) {
      if (bcm_phone_line_state_has_changes(&(line_states[line_idx]))) {
         ret = true;
         break;
      }
   }
   return (ret);
}
#endif /* BCMPH_EXPORT_DEV_FILE */

// Transfer line states in structure passed as parameter
int bcm_drv_get_line_states(bcm_drv_t *t,
   bcm_phone_get_line_states_t *get_line_states
#ifdef BCMPH_EXPORT_DEV_FILE
   , bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   )
{
   int ret = 0;
   size_t line_idx;
   int line_enabled = 0;
#ifdef BCMPH_EXPORT_DEV_FILE
   int eventq_counter = 0;
   long timeout_in_jiffies;
#endif /* BCMPH_EXPORT_DEV_FILE */
   size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->core.phone_mgr));

   dd_bcm_pr_debug("%s()\n", __func__);

#ifdef BCMPH_EXPORT_DEV_FILE
   bcm_assert((NULL != lock) && (bcmph_mutex_is_locked(lock)));
#endif /* BCMPH_EXPORT_DEV_FILE */

   memset(get_line_states->line_state, 0, sizeof(get_line_states->line_state));
   for (line_idx = 0; (line_idx < ARRAY_SIZE(get_line_states->line_state)); line_idx += 1) {
      bcm_phone_line_state_init(&(get_line_states->line_state[line_idx]));
      if ((line_idx < phone_line_count) && (bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line_idx))) {
         line_enabled += 1;
      }
   }

#ifdef BCMPH_EXPORT_DEV_FILE
   if (get_line_states->wait < 0) {
      timeout_in_jiffies = MAX_JIFFY_OFFSET;
   }
   else {
      timeout_in_jiffies = msecs_to_jiffies(get_line_states->wait);
   }
#endif /* BCMPH_EXPORT_DEV_FILE */

   if (line_enabled > 0) {
      if (phone_line_count > ARRAY_SIZE(get_line_states->line_state)) {
         phone_line_count = ARRAY_SIZE(get_line_states->line_state);
      }
#ifdef BCMPH_EXPORT_DEV_FILE
      for (;;) {
         barrier();
         eventq_counter = bcm_wait_queue_get_counter(bcm_phone_mgr_get_eventq(&(t->core.phone_mgr)));
#endif /* BCMPH_EXPORT_DEV_FILE */

         bcm_phone_mgr_get_line_states(&(t->core.phone_mgr), get_line_states->line_state, phone_line_count);
#ifdef BCMPH_EXPORT_DEV_FILE
         if (bcm_drv_one_line_changes_state(get_line_states->line_state, phone_line_count)) {
            break;
         }

         if ((0 != timeout_in_jiffies)
             && (pcm_is_started(&(t->core.pcm)))) {
            if (MAX_JIFFY_OFFSET == timeout_in_jiffies) {
               ret = bcm_wait_queue_wait_event_counter(
                  bcm_phone_mgr_get_eventq(&(t->core.phone_mgr)),
                  eventq_counter, lock);
               if (0 != ret) {
                  break;
               }
            }
            else {
               long remaining = bcm_wait_queue_wait_event_counter_timeout(
                  bcm_phone_mgr_get_eventq(&(t->core.phone_mgr)),
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
#endif /* BCMPH_EXPORT_DEV_FILE */
   }

   /*
    Set flag pcm_dma_is_started just before exiting in case its state
    changes when we were waiting that at least one line state changes
   */
   get_line_states->pcm_is_started = pcm_dma_is_started(&(t->core.pcm));

   return (ret);
}

#ifdef BCMPH_TEST_PCM
int bcm_drv_set_line_state(bcm_drv_t *t,
   const bcm_phone_set_line_state_t *set_line_state)
{
   int ret = 0;

   d_bcm_pr_debug("%s()\n", __func__);

   bcm_assert(NULL != set_line_state);

   do { // Empty loop
      if ((set_line_state->line > bcm_phone_mgr_get_line_count(&(t->core.phone_mgr)))
          || (!bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), set_line_state->line))) {
         bcm_pr_err("Invalid line\n");
         ret = -EINVAL;
         break;
      }

      ret = bcm_phone_mgr_set_line_state(&(t->core.phone_mgr), set_line_state);
   }
   while (false);

   return (ret);
}
#endif // BCMPH_TEST_PCM

// Stop PCM.
// Flag wait tells to wait for transmission of all pending data
// (in TX ring buffers and in DMA buffers)
void bcm_drv_stop(bcm_drv_t *t, bool wait)
{
   size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->core.phone_mgr));
   size_t line_idx;

   bcm_pr_debug("%s()\n", __func__);

   // For each line, set mode to IDLE because bcm_drv_stop_pcm() does
   // nothing if one of the lines needs PCM
   for (line_idx = 0; (line_idx < phone_line_count); line_idx += 1) {
      if (bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line_idx)) {
         bcm_drv_set_line_mode(t, line_idx, BCMPH_MODE_DISCONNECT
#ifndef BCMPH_DAHDI_DRIVER
            , 0
#endif /* !BCMPH_DAHDI_DRIVER */
            , -1, BCMPH_TONE_NONE
#ifdef BCMPH_EXPORT_DEV_FILE
            , 0, NULL
#endif /* BCMPH_EXPORT_DEV_FILE */
            );
      }
   }

   bcm_drv_stop_pcm(t, wait);
   if (wait) {
      size_t loop_count;
      for (loop_count = 0; ((pcm_dma_is_started(&(t->core.pcm))) && (loop_count < 100)); loop_count += 1) {
         msleep(t->core.board_desc->phone_desc->tick_period);
         bcm_drv_stop_pcm(t, wait);
      }
      bcm_assert(loop_count < 100);
   }

   pcm_stop(&(t->core.pcm));

   bcm_phone_mgr_stop(&(t->core.phone_mgr));

#ifndef BCMPH_DAHDI_DRIVER
   for (line_idx = 0; (line_idx < phone_line_count); line_idx += 1) {
      bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line_idx]);
      dpl->cancel_echo = false;
      if (NULL != dpl->oslec_state) {
         oslec_free(dpl->oslec_state);
         dpl->oslec_state = NULL;
      }
   }
#endif /* !BCMPH_DAHDI_DRIVER */

#ifdef BCMPH_EXPORT_DEV_FILE
   // We must wake up all processes sleeping in wait_queue
   bcm_wait_queue_wake_up(&(t->dev.inq));
   bcm_wait_queue_wake_up(&(t->dev.outq));
#endif /* BCMPH_EXPORT_DEV_FILE */
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
int bcm_drv_start(bcm_drv_t *t, const bcm_phone_cfg_params_t *params)
{
   int ret = 0;
   size_t max_timeslots = pcm_get_max_timeslots(&(t->core.pcm));

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert(0 == (max_timeslots % 2));

   do { // Empty loop
      size_t line_idx;
      phone_line_params_t line_params[BCMPH_MAX_LINES];
      size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->core.phone_mgr));
      size_t disabled_lines;
      size_t enabled_lines;
      __u8 timeslot_to_channel[128];
      __u32 pcm_channels_used;
      __u8 pcm_channel;

      bcm_assert(ARRAY_SIZE(timeslot_to_channel) >= max_timeslots);
      bcm_assert(ARRAY_SIZE(line_params) >= ARRAY_SIZE(params->line_params));

      bcm_drv_stop(t, false);

      if (/*(params->country < 0) || */(params->country >= BCMPH_COUNTRY_MAX)) {
         ret = -EINVAL;
         break;
      }

      memset(line_params, 0, sizeof(line_params));
      for (line_idx = 0; (line_idx < phone_line_count); line_idx += 1) {
         bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line_idx]);
         // We set a very high value for bytes_per_frame, so that we
         // can't have enough data or enough free space to write or
         // read a DMA frame
         dpl->bytes_per_frame = 0x7FFFFFFF;
         dpl->offset_first_block_pcm_channel = 0;
         dpl->offset_second_block_pcm_channel = 0;
         dpl->type_transfer = BCMPH_TRF_NO_OP;
         dpl->rx_buffer.len = 0;
         dpl->tx_buffer.len = 0;
         dpl->tx_buffer.offset = 0;
#ifndef BCMPH_DAHDI_DRIVER
         dpl->cancel_echo = false;
         if (NULL != dpl->oslec_state) {
            oslec_free(dpl->oslec_state);
            dpl->oslec_state = NULL;
         }
#endif /* !BCMPH_DAHDI_DRIVER */
         if (line_idx < ARRAY_SIZE(line_params)) {
            if (line_idx < ARRAY_SIZE(params->line_params)) {
               line_params[line_idx].enable = params->line_params[line_idx].enable;
            }
            else {
               line_params[line_idx].enable = false;
            }
         }
      }
      for (; (line_idx < ARRAY_SIZE(line_params)); line_idx += 1) {
         line_params[line_idx].enable = false;
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
      for (line_idx = 0; (line_idx < ARRAY_SIZE(line_params)); line_idx += 1) {
         if (!line_params[line_idx].enable) {
            disabled_lines += 1;
         }
         else {
            bcm_core_drv_phone_line_t *dpl;
            enabled_lines += 1;

            // Check that line exists
            if (line_idx >= phone_line_count) {
               bcm_pr_err("Trying to enable line %lu that don't exist\n", (unsigned long)(line_idx));
               ret = -EINVAL;
               break;
            }

            dpl = &(t->core.phone_lines[line_idx]);

            bcm_assert((line_idx < ARRAY_SIZE(params->line_params))
               && (params->line_params[line_idx].enable));

            line_params[line_idx].codec = params->line_params[line_idx].codec;
            line_params[line_idx].first_timeslot = bcm_phone_mgr_get_first_timeslot_line(&(t->core.phone_mgr), line_idx);

            // Check the codec is valid
            if ((BCMPH_CODEC_ALAW != line_params[line_idx].codec)
                && (BCMPH_CODEC_ULAW != line_params[line_idx].codec)
                && (BCMPH_CODEC_LINEAR != line_params[line_idx].codec)
                && (BCMPH_CODEC_LINEAR16 != line_params[line_idx].codec)
                && (BCMPH_CODEC_ALAW16 != line_params[line_idx].codec)
                && (BCMPH_CODEC_ULAW16 != line_params[line_idx].codec)) {
               bcm_pr_err("Invalid codec %d for line %lu\n",
                  (int)(line_params[line_idx].codec), (unsigned long)(line_idx));
               ret = -EINVAL;
               break;
            }

            // Check that the device supports the codec
            if (!bcm_phone_mgr_line_supports_codec(&(t->core.phone_mgr), line_idx, line_params[line_idx].codec)) {
               bcm_pr_err("Invalid codec %d for line %lu\n",
                  (int)(line_params[line_idx].codec), (unsigned long)(line_idx));
               ret = -EINVAL;
               break;
            }

#ifndef BCMPH_DAHDI_DRIVER
            if ((params->line_params[line_idx].echo_cancel_tap_length > 0) && (BCMPH_CODEC_LINEAR != line_params[line_idx].codec)) {
               bcm_pr_err("Echo cancellation can be enabled for line %lu so no other codec than linear is allowed\n",
                  (unsigned long)(line_idx));
               ret = -EINVAL;
               break;
            }
            dpl->cancel_echo = false;
            bcm_assert(NULL == dpl->oslec_state);
            {
               int tap_length = params->line_params[line_idx].echo_cancel_tap_length;
               if (tap_length > 0) {
                  switch (tap_length) {
                     case 32:
                     case 64:
                     case 128:
                     case 256:
                     case 512:
                     case 1024: {
                        break;
                     }
                     default: {
                        /*
                         Default value should be coherent with the
                         number of DMA buffers we fill when we start
                         PCM.
                         128 samples == 16 ms
                        */
                        tap_length = 128;
                        break;
                     }
                  }
                  bcm_pr_debug("Creating the echo canceller for line %lu with tap_length = %d\n",
                     (unsigned long)(line_idx), (int)(tap_length));
                  dpl->oslec_state = oslec_create(tap_length,
                     ECHO_CAN_USE_ADAPTION | ECHO_CAN_USE_NLP | ECHO_CAN_USE_CLIP | ECHO_CAN_USE_TX_HPF | ECHO_CAN_USE_RX_HPF);
                  if (NULL == dpl->oslec_state) {
                     bcm_pr_err("Can't create the echo canceller for line %lu.\n", (unsigned long)(line_idx));
                     ret = -ENOMEM;
                     break;
                  }
               }
            }
#endif /* !BCMPH_DAHDI_DRIVER */

            // Check the timeslot is even
            if (0 != (line_params[line_idx].first_timeslot % 2)) {
               bcm_pr_err("The first timeslot of a line must be even. The line %lu does not meet this condition.\n", (unsigned long)(line_idx));
               ret = -EINVAL;
               break;
            }
            /*
             Check that timeslots used or potentially usable by the line
             are available.
             Even if we know that, when echo cancellation is enabled, we
             can only use codec BCMPH_CODEC_LINEAR, we make the same
             checks, so that allocation of timeslots and PCM channels
             are the same whether echo cancellation is enabled or not.
             Perhaps we are a bit lazy too !
            */
            ret = bcm_drv_use_ts_and_pcm_channel(line_idx,
               timeslot_to_channel, max_timeslots, &(pcm_channels_used),
               line_params[line_idx].first_timeslot, pcm_channel);
            if (ret) {
               break;
            }
            dpl->offset_first_block_pcm_channel = (pcm_channel * BCMPH_PCM_CHANNEL_WIDTH);
            pcm_channel += 1;
            if (!params->pcm_use_16bits_timeslot) {
               if (((bcm_phone_mgr_line_supports_codec(&(t->core.phone_mgr), line_idx, BCMPH_CODEC_LINEAR))
                    && (bcm_phone_mgr_line_can_switch_to_codec(&(t->core.phone_mgr), line_idx, line_params[line_idx].codec, BCMPH_CODEC_LINEAR)))
                   || ((bcm_phone_mgr_line_supports_codec(&(t->core.phone_mgr), line_idx, BCMPH_CODEC_LINEAR16))
                       && (bcm_phone_mgr_line_can_switch_to_codec(&(t->core.phone_mgr), line_idx, line_params[line_idx].codec, BCMPH_CODEC_LINEAR16)))) {
                  __u8 second_timeslot = line_params[line_idx].first_timeslot + 1;
                  ret = bcm_drv_use_ts_and_pcm_channel(line_idx,
                     timeslot_to_channel, max_timeslots, &(pcm_channels_used),
                     second_timeslot, pcm_channel);
                  if (ret) {
                     break;
                  }
                  pcm_channel += 1;
               }
               if (((bcm_phone_mgr_line_supports_codec(&(t->core.phone_mgr), line_idx, BCMPH_CODEC_ALAW16))
                    && (bcm_phone_mgr_line_can_switch_to_codec(&(t->core.phone_mgr), line_idx, line_params[line_idx].codec, BCMPH_CODEC_ALAW16)))
                   || ((bcm_phone_mgr_line_supports_codec(&(t->core.phone_mgr), line_idx, BCMPH_CODEC_ULAW16))
                       && (bcm_phone_mgr_line_can_switch_to_codec(&(t->core.phone_mgr), line_idx, line_params[line_idx].codec, BCMPH_CODEC_ULAW16)))
                   || ((bcm_phone_mgr_line_supports_codec(&(t->core.phone_mgr), line_idx, BCMPH_CODEC_LINEAR16))
                       && (bcm_phone_mgr_line_can_switch_to_codec(&(t->core.phone_mgr), line_idx, line_params[line_idx].codec, BCMPH_CODEC_LINEAR16)))) {
                  __u8 third_timeslot = line_params[line_idx].first_timeslot + (max_timeslots / 2);
                  ret = bcm_drv_use_ts_and_pcm_channel(line_idx,
                     timeslot_to_channel, max_timeslots, &(pcm_channels_used),
                     third_timeslot, pcm_channel);
                  if (ret) {
                     break;
                  }
                  dpl->offset_second_block_pcm_channel = (pcm_channel * BCMPH_PCM_CHANNEL_WIDTH);
                  pcm_channel += 1;
                  if ((bcm_phone_mgr_line_supports_codec(&(t->core.phone_mgr), line_idx, BCMPH_CODEC_LINEAR16))
                      && (bcm_phone_mgr_line_can_switch_to_codec(&(t->core.phone_mgr), line_idx, line_params[line_idx].codec, BCMPH_CODEC_LINEAR16))) {
                     __u8 fourth_timeslot = third_timeslot + 1;
                     ret = bcm_drv_use_ts_and_pcm_channel(line_idx,
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
               bcm_assert(timeslot_to_channel[line_params[line_idx].first_timeslot + 1] >= BCMPH_PCM_MAX_CHANNELS);
               timeslot_to_channel[line_params[line_idx].first_timeslot + 1] = timeslot_to_channel[line_params[line_idx].first_timeslot];
               if (((bcm_phone_mgr_line_supports_codec(&(t->core.phone_mgr), line_idx, BCMPH_CODEC_ALAW16))
                    && (bcm_phone_mgr_line_can_switch_to_codec(&(t->core.phone_mgr), line_idx, line_params[line_idx].codec, BCMPH_CODEC_ALAW16)))
                   || ((bcm_phone_mgr_line_supports_codec(&(t->core.phone_mgr), line_idx, BCMPH_CODEC_ULAW16))
                       && (bcm_phone_mgr_line_can_switch_to_codec(&(t->core.phone_mgr), line_idx, line_params[line_idx].codec, BCMPH_CODEC_ULAW16)))
                   || ((bcm_phone_mgr_line_supports_codec(&(t->core.phone_mgr), line_idx, BCMPH_CODEC_LINEAR16))
                       && (bcm_phone_mgr_line_can_switch_to_codec(&(t->core.phone_mgr), line_idx, line_params[line_idx].codec, BCMPH_CODEC_LINEAR16)))) {
                  __u8 third_timeslot = line_params[line_idx].first_timeslot + (max_timeslots / 2);
                  ret = bcm_drv_use_ts_and_pcm_channel(line_idx,
                     timeslot_to_channel, max_timeslots, &(pcm_channels_used),
                     third_timeslot, pcm_channel);
                  if (ret) {
                     break;
                  }
                  dpl->offset_second_block_pcm_channel = (pcm_channel * BCMPH_PCM_CHANNEL_WIDTH);
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
         size_t offset = t->core.offset_rbs;
         size_t size = t->core.size_rbs;
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

#ifdef BCMPH_EXPORT_DEV_FILE
         // We check that default_line is a line enabled.
         if (t->dev.default_line >= ARRAY_SIZE(line_params)) {
            t->dev.default_line = BCMPH_DEFAULT_LINE;
            bcm_assert(BCMPH_DEFAULT_LINE < ARRAY_SIZE(line_params));
         }
#endif /* BCMPH_EXPORT_DEV_FILE */
         for (line_idx = 0; (line_idx < ARRAY_SIZE(line_params)); line_idx += 1) {
            bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line_idx]);
            if (line_params[line_idx].enable) {
#ifdef BCMPH_EXPORT_DEV_FILE
               // Check that default_line is the index of a line enabled
               if (!line_params[t->dev.default_line].enable) {
                  t->dev.default_line = line_idx;
               }
#endif /* BCMPH_EXPORT_DEV_FILE */
               bcm_assert((timeslot_to_channel[line_params[line_idx].first_timeslot] * BCMPH_PCM_CHANNEL_WIDTH) == dpl->offset_first_block_pcm_channel);
               size = len_rx_ring_buffer;
               t->core.mm_rbs_location.rbs[line_idx].rx_buffer_offset = offset;
               t->core.mm_rbs_location.rbs[line_idx].rx_buffer_size = size;
               bcm_pr_debug("t->mm_rbs_location.rbs[%lu].rx_buffer_offset = %lu, t->mm_rbs_location.rbs[%lu].rx_buffer_size = %lu\n",
                  (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].rx_buffer_offset),
                  (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].rx_buffer_size));
               offset += round_up_to_pow_of_2(size, BCMPH_MMAP_ALIGNMENT);

               size = len_tx_ring_buffer;
               t->core.mm_rbs_location.rbs[line_idx].tx_buffer_offset = offset;
               t->core.mm_rbs_location.rbs[line_idx].tx_buffer_size = size;
               bcm_pr_debug("t->mm_rbs_location.rbs[%lu].tx_buffer_offset = %lu, t->mm_rbs_location.rbs[%lu].tx_buffer_size = %lu\n",
                  (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].tx_buffer_offset),
                  (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].tx_buffer_size));
               offset += round_up_to_pow_of_2(size, BCMPH_MMAP_ALIGNMENT);
            }
            else {
               bcm_assert((BCMPH_TRF_NO_OP == dpl->type_transfer) && (dpl->bytes_per_frame > t->core.size_rbs));
               t->core.mm_rbs_location.rbs[line_idx].rx_buffer_offset = offset_dummy_ring_buffer;
               t->core.mm_rbs_location.rbs[line_idx].rx_buffer_size = len_dummy_ring_buffer;
               bcm_pr_debug("t->mm_rbs_location.rbs[%lu].rx_buffer_offset = %lu, t->mm_rbs_location.rbs[%lu].rx_buffer_size = %lu\n",
                  (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].rx_buffer_offset),
                  (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].rx_buffer_size));
               t->core.mm_rbs_location.rbs[line_idx].tx_buffer_offset = offset_dummy_ring_buffer;
               t->core.mm_rbs_location.rbs[line_idx].tx_buffer_size = len_dummy_ring_buffer;
               bcm_pr_debug("t->mm_rbs_location.rbs[%lu].tx_buffer_offset = %lu, t->mm_rbs_location.rbs[%lu].tx_buffer_size = %lu\n",
                  (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].tx_buffer_offset),
                  (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].tx_buffer_size));
            }
            bcm_ring_buf_init(&(dpl->rx_ring_buf),
               t->core.mm_buffer + t->core.mm_rbs_location.rbs[line_idx].rx_buffer_offset,
               t->core.mm_rbs_location.rbs[line_idx].rx_buffer_size);
#ifdef BCMPH_EXPORT_DEV_FILE
            bcm_ring_buf_get_desc(&(dpl->rx_ring_buf),
               (bcm_ring_buf_desc_t *)(t->core.mm_buffer + t->core.mm_rbs_location.rbs[line_idx].rx_ring_buf_desc_off));
#endif /* BCMPH_EXPORT_DEV_FILE */
            bcm_ring_buf_init(&(dpl->tx_ring_buf),
               t->core.mm_buffer + t->core.mm_rbs_location.rbs[line_idx].tx_buffer_offset,
               t->core.mm_rbs_location.rbs[line_idx].tx_buffer_size);
#ifdef BCMPH_EXPORT_DEV_FILE
            bcm_ring_buf_get_desc(&(dpl->tx_ring_buf),
               (bcm_ring_buf_desc_t *)(t->core.mm_buffer + t->core.mm_rbs_location.rbs[line_idx].tx_ring_buf_desc_off));
#endif /* BCMPH_EXPORT_DEV_FILE */
            if (line_params[line_idx].enable) {
               // Must be called after initializing ring buffers
               bcm_drv_set_bytes_per_frame(t, dpl, params->pcm_use_16bits_timeslot, line_params[line_idx].codec);
            }
         }
         for (; (line_idx < phone_line_count); line_idx += 1) {
            bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line_idx]);

            // Phone line is implicitly disabled
            bcm_assert((BCMPH_TRF_NO_OP == dpl->type_transfer) && (dpl->bytes_per_frame > t->core.size_rbs));
            t->core.mm_rbs_location.rbs[line_idx].rx_buffer_offset = offset_dummy_ring_buffer;
            t->core.mm_rbs_location.rbs[line_idx].rx_buffer_size = len_dummy_ring_buffer;
            bcm_pr_debug("t->mm_rbs_location.rbs[%lu].rx_buffer_offset = %lu, t->mm_rbs_location.rbs[%lu].rx_buffer_size = %lu\n",
               (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].rx_buffer_offset),
               (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].rx_buffer_size));
            t->core.mm_rbs_location.rbs[line_idx].tx_buffer_offset = offset_dummy_ring_buffer;
            t->core.mm_rbs_location.rbs[line_idx].tx_buffer_size = len_dummy_ring_buffer;
            bcm_pr_debug("t->mm_rbs_location.rbs[%lu].tx_buffer_offset = %lu, t->mm_rbs_location.rbs[%lu].tx_buffer_size = %lu\n",
               (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].tx_buffer_offset),
               (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].tx_buffer_size));
            bcm_ring_buf_init(&(dpl->rx_ring_buf),
               t->core.mm_buffer + t->core.mm_rbs_location.rbs[line_idx].rx_buffer_offset,
               t->core.mm_rbs_location.rbs[line_idx].rx_buffer_size);
#ifdef BCMPH_EXPORT_DEV_FILE
            bcm_ring_buf_get_desc(&(dpl->rx_ring_buf),
               (bcm_ring_buf_desc_t *)(t->core.mm_buffer + t->core.mm_rbs_location.rbs[line_idx].rx_ring_buf_desc_off));
#endif /* BCMPH_EXPORT_DEV_FILE */
            bcm_ring_buf_init(&(dpl->tx_ring_buf),
               t->core.mm_buffer + t->core.mm_rbs_location.rbs[line_idx].tx_buffer_offset,
               t->core.mm_rbs_location.rbs[line_idx].tx_buffer_size);
#ifdef BCMPH_EXPORT_DEV_FILE
            bcm_ring_buf_get_desc(&(dpl->tx_ring_buf),
               (bcm_ring_buf_desc_t *)(t->core.mm_buffer + t->core.mm_rbs_location.rbs[line_idx].tx_ring_buf_desc_off));
#endif /* BCMPH_EXPORT_DEV_FILE */
         }
      }

      if (params->pcm_use_16bits_timeslot) {
         size_t timeslot;
         for (timeslot = 0; (timeslot < max_timeslots); timeslot += 2) {
            // We must correct timeslot because in 16 bits mode a timeslot
            // is 16 bits wide for PCM module
            if (timeslot_to_channel[timeslot] < BCMPH_PCM_MAX_CHANNELS) {
               bcm_assert(timeslot_to_channel[timeslot + 1] == timeslot_to_channel[timeslot]);
               timeslot_to_channel[timeslot >> 1] = timeslot_to_channel[timeslot];
            }
            else {
               bcm_assert(timeslot_to_channel[timeslot + 1] >= BCMPH_PCM_MAX_CHANNELS);
               timeslot_to_channel[timeslot >> 1] = BCMPH_PCM_MAX_CHANNELS;
            }
         }
         max_timeslots >>= 1;
      }
      {
         size_t timeslot_idx;
         size_t channel_idx;
         __u8 channel_to_timeslot[BCMPH_PCM_MAX_CHANNELS];

         memset(channel_to_timeslot, BCMPH_TIMESLOT_UNSPECIFIED, sizeof(channel_to_timeslot));
         for (timeslot_idx = 0; (timeslot_idx < max_timeslots); timeslot_idx += 1) {
            if (timeslot_to_channel[timeslot_idx] < BCMPH_PCM_MAX_CHANNELS) {
               bcm_assert(BCMPH_TIMESLOT_UNSPECIFIED == channel_to_timeslot[timeslot_to_channel[timeslot_idx]]);
               channel_to_timeslot[timeslot_to_channel[timeslot_idx]] = timeslot_idx;
            }
         }
         for (channel_idx = 0; (channel_idx < pcm_channel); channel_idx += 1) {
            bcm_assert(BCMPH_TIMESLOT_UNSPECIFIED != channel_to_timeslot[channel_idx]);
         }
         for (; (channel_idx < ARRAY_SIZE(channel_to_timeslot)); channel_idx += 1) {
            bcm_assert(BCMPH_TIMESLOT_UNSPECIFIED == channel_to_timeslot[channel_idx]);
         }
         pcm_configure_channels(&(t->core.pcm), params->pcm_use_16bits_timeslot, channel_to_timeslot, pcm_channel);
      }
      // We must start PCM before initializing phone devices
      // else phone devices will not detect PCLK and initialization
      // will fail
      pcm_start(&(t->core.pcm));

      ret = bcm_phone_mgr_start(&(t->core.phone_mgr), params->country, line_params, ARRAY_SIZE(line_params));
      if (ret) {
         pcm_stop(&(t->core.pcm));
      }
   } while (false);

   return (ret);
}
