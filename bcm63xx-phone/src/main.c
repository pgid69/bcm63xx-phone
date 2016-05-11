/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#include <config.h>

#ifdef __KERNEL__
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#else
#include <fcntl.h>
#endif // __KERNEL__

#ifdef BCMPH_TEST_PCM
#include <bcm63xx.h>
#endif // BCMPH_TEST_PCM
#include <bcm63xx_phone.h>
#include <bcm63xx_log.h>

#include "main.h"
#include "utils.h"

// Include after system files
#include <compile.h>

#define VERSION "0.1"

#ifdef BCMPH_TEST_PCM
// This param is a sequence of bytes used to initialize the Le88221
char *bcm_drv_param_mpi =
   "0x4A,0x01," // Select channel 1
   "0x40,0,0x42,0," // Set RX and TX TS to 0
   "0x4A,0x02," // Select channel 2
   "0x40,2,0x42,2," // Set RX and TX TS to 2
   "0x4A,0x03," // Select both channels
   "0x46,0x02," // Set PCM clock frequency to 2048 kHz (replace 0x02 by 0x03 for 1024 kHz)
   "0x44,0x40," // Transmit changes on positive edge of PCLK
   "0x4A,0x03," // Select both channel
   "0x60,0x80," // Set linear coding
   "0x70,0x04," // Set interface loopback
   "0x56,0x20," // Activate codec and set system state to disconnected
   ;
// This param is used to set some bits of the register PCM_CTRL_REG
uint bcm_drv_param_pcm_ctrl = PCM_CLOCK_INV | PCM_FS_TRIG | PCM_DATA_OFF;
// This param is used to initialize the register PCM_CHAN_CTRL_REG
uint bcm_drv_param_pcm_chan_ctrl = 0x0000FFFF;
// This param is used to remap PCM channels
// Each nibble define a channel number (between 0 and BCMPH_PCM_MAX_CHANNELS)
// Defaut value of 0x01234567 map each channel number to itself
// Value of 0x76543210 remap channel 0 to channel 7,
// channel 1 to channel 6, ..., and channel 7 to channel 0.
uint bcm_drv_param_channel_mapping = 0x01234567;

#ifdef __KERNEL__
module_param_named(mpi, bcm_drv_param_mpi, charp, 0);
module_param_named(pcm_ctrl, bcm_drv_param_pcm_ctrl, uint, 0);
module_param_named(pcm_chan_ctrl, bcm_drv_param_pcm_chan_ctrl, uint, 0);
module_param_named(channel_mapping, bcm_drv_param_channel_mapping, uint, 0);
#endif // __KERNEL__
#endif // !BCMPH_TEST_PCM

// Callback called periodically by timer of phone_mgr
static void bcm_drv_timer_cb(bcm_phone_mgr_t *phone_mgr)
{
   bcm_drv_t *t = container_of(phone_mgr, bcm_drv_t, phone_mgr);

   dd_bcm_pr_debug("bcm_drv_timer_cb()\n");

   // If we can get the global lock we try to :
   // - fill TX DMA buffers
   // - empty RX DMA buffers
   // BEWARE : mutex_trylock() return 1 on success and 0 if mutex can't be locked
   if (mutex_trylock(&(t->lock))) {
      if (pcm_dma_is_started(&(t->pcm))) {
         bcm_drv_update_buffers(t);
      }
      mutex_unlock(&(t->lock));
   }
}

// Enumerate all active lines to compute the minimum and maximum number of
// DMA frames that can be read
static void bcm_drv_get_free_space_for_rx(bcm_drv_t *t, size_t *min_frame, size_t *max_frame)
{
   size_t i;
   size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->phone_mgr));
   bool min_max_not_set = true;

   dd_bcm_pr_debug("bcm_drv_get_free_space_for_rx()\n");

   *min_frame = 0;
   *max_frame = 0;
   for (i = 0; (i < phone_line_count); i += 1) {
#ifdef BCMPH_TEST_PCM
      if (i != t->default_line) {
         continue;
      }
#endif // BCMPH_TEST_PCM
      if (bcm_phone_mgr_line_rx_use_pcm(&(t->phone_mgr), i)) {
         size_t count = bcm_ring_buf_get_free_space(&(t->phone_lines[i].rx_ring_buf)) / t->phone_lines[i].bytes_per_frame;
         if (min_max_not_set) {
            *min_frame = count;
            *max_frame = count;
            min_max_not_set = false;
         }
         else {
            if (*min_frame > count) {
               *min_frame = count;
            }
            else if (*max_frame < count) {
               *max_frame = count;
            }
         }
      }
   }
}

// Callback called by pcm to read a DMA frame
static void bcm_drv_pcm_receive_cb(pcm_t *pcm,
   const __u8 *data, size_t data_len)
{
   bcm_drv_t *t = container_of(pcm, bcm_drv_t, pcm);
   size_t i;
   size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->phone_mgr));

   dd_bcm_pr_debug("bcm_drv_fill_rx_ring_buf_cb()\n");

   for (i = 0; (i < phone_line_count); i += 1) {
      bcm_drv_phone_line_t *dpl = &(t->phone_lines[i]);
      bcm_ring_buf_t *rb = &(dpl->rx_ring_buf);
#ifdef BCMPH_TEST_PCM
      if (i != t->default_line) {
         continue;
      }
#endif // BCMPH_TEST_PCM
      // Check that they are enough room in the ring buffer
      if ((bcm_phone_mgr_line_rx_use_pcm(&(t->phone_mgr), i))
          && (bcm_ring_buf_get_free_space(rb) >= dpl->bytes_per_frame)) {
         const __u8 *src0 = data + dpl->offset_first_block_pcm_channel;
         switch (dpl->type_transfer) {
            case BCMPH_TRF_M_8_BITS_C_xLAW: {
               /*
                A sample is just a byte.
                Data must be read from the 1st chanrnel of the 1st PCM
                block.
                Transfer is straightforward
               */
               bcm_assert((BCMPH_PCM_CHANNEL_WIDTH == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               bcm_ring_buf_add(rb, src0, BCMPH_PCM_CHANNEL_WIDTH);
               break;
            }
            case BCMPH_TRF_M_8_BITS_C_LINEAR: {
               /*
                A sample is a word of 2 bytes.
                The most significant byte is in the 1st timeslot
                and the least significant byte is in the next
                timeslot.
                We suppose the data is in the native endianness of the
                platform, ie big endian, so the most significant byte
                must be read from the 1st PCM channel of the 1st PCM
                block and the least significant byte must be read from
                the next PCM channel in the same block
                (cf channel allocation in bcm_drv_start())
               */
               __u8 temp[BCMPH_PCM_CHANNEL_WIDTH * 2];
               const __u8 *src1 = src0 + BCMPH_PCM_CHANNEL_WIDTH;
               __u8 *dst = temp;
               bcm_assert((ARRAY_SIZE(temp) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + (BCMPH_PCM_CHANNEL_WIDTH * 2)) <= data_len));
               for (i = 0; (i < BCMPH_PCM_CHANNEL_WIDTH); i += 1) {
                  *dst = *src0;
                  dst += 1;
                  src0 += 1;
                  *dst = *src1;
                  dst += 1;
                  src1 += 1;
               }
               bcm_ring_buf_add(rb, temp, ARRAY_SIZE(temp));
               break;
            }
            case BCMPH_TRF_M_8_BITS_C_LINEAR16: {
               /*
                A sample is a pair of word of 2 bytes
                The most significant byte of the 1st word is in the
                1st timeslot and the least significant byte of the 1st
                word is in the next timeslot.
                The most significant byte of the 2nd word is in the
                (1st timeslot + (max_timeslots / 2)) and the least
                significant byte of the 2nd word is in the next
                timeslot.
                We suppose the data is in the native endianness of the
                platform, ie big endian, so the most significant byte of
                the 1st word must be read from the 1st PCM channel of
                the 1st PCM block, the least significant byte of the 1st
                word must be read from the 2nd PCM channel of the 1st
                PCM block, the most significant byte of the 2nd word
                must be read from the 1st PCM channel of the 2nd PCM
                block and the least significant byte of the 2nd word
                must be read from the 2nd PCM channel of the 2nd PCM
                block
                (cf channel allocation in bcm_drv_start())
               */
               __u8 temp[BCMPH_PCM_CHANNEL_WIDTH * 4];
               const __u8 *src1 = src0 + BCMPH_PCM_CHANNEL_WIDTH;
               const __u8 *src2 = data + dpl->offset_second_block_pcm_channel;
               const __u8 *src3 = src2 + BCMPH_PCM_CHANNEL_WIDTH;
               __u8 *dst = temp;
               bcm_assert((ARRAY_SIZE(temp) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + (BCMPH_PCM_CHANNEL_WIDTH * 2)) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + (BCMPH_PCM_CHANNEL_WIDTH * 2)) <= data_len));
               for (i = 0; (i < BCMPH_PCM_CHANNEL_WIDTH); i += 1) {
                  *dst = *src0;
                  dst += 1;
                  src0 += 1;
                  *dst = *src1;
                  dst += 1;
                  src1 += 1;
                  *dst = *src2;
                  dst += 1;
                  src2 += 1;
                  *dst = *src3;
                  dst += 1;
                  src3 += 1;
               }
               bcm_ring_buf_add(rb, temp, ARRAY_SIZE(temp));
               break;
            }
            case BCMPH_TRF_M_8_BITS_C_xLAW16: {
               /*
                A sample is just a byte but we have two samples per
                frame. The 1st sample is in the 1st timeslot and the 2nd
                sample is in (1st timeslot + (max_timeslots / 2)).
                The 1st sample must be read from the 1st PCM channel of
                the 1st PCM block and the 2nd sample must be read from
                the 1st PCM channel of the 2nd PCM block
                (cf channel allocation in bcm_drv_start())
               */
               __u8 temp[BCMPH_PCM_CHANNEL_WIDTH * 2];
               const __u8 *src1 = data + dpl->offset_second_block_pcm_channel;
               __u8 *dst = temp;
               bcm_assert((ARRAY_SIZE(temp) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               for (i = 0; (i < BCMPH_PCM_CHANNEL_WIDTH); i += 1) {
                  *dst = *src0;
                  dst += 1;
                  src0 += 1;
                  *dst = *src1;
                  dst += 1;
                  src1 += 1;
               }
               bcm_ring_buf_add(rb, temp, ARRAY_SIZE(temp));
               break;
            }
            case BCMPH_TRF_M_16_BITS_C_xLAW: {
               /*
                A sample is just a byte.
                It is in the 1st timeslot.
                As the platform is big endian the byte must be read from
                the most significant byte of the words that
                compose the 1st PCM channel of the 1st PCM block
               */
               __u8 temp[(BCMPH_PCM_CHANNEL_WIDTH + 1) / 2];
               __u8 *dst = temp;
               bcm_assert((ARRAY_SIZE(temp) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               for (i = 0; (i < BCMPH_PCM_CHANNEL_WIDTH); i += 2) {
                  *dst = *src0;
                  dst += 1;
                  src0 += 2;
               }
               bcm_ring_buf_add(rb, temp, ARRAY_SIZE(temp));
               break;
            }
            case BCMPH_TRF_M_16_BITS_C_LINEAR: {
               /*
                A sample is a word of 2 bytes.
                The 2 bytes are read from the 1st PCM channel of the
                1st PCM block
                The most significant byte is in the 1st timeslot,
                and the least significant byte is in the 2nd
                timeslot.
                We suppose the data is in the native endianness of the
                platform, ie big endian, so there no need to swap the
                bytes
               */
               bcm_assert((BCMPH_PCM_CHANNEL_WIDTH == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               bcm_ring_buf_add(rb, src0, BCMPH_PCM_CHANNEL_WIDTH);
               break;
            }
            case BCMPH_TRF_M_16_BITS_C_LINEAR16: {
               /*
                A sample is a pair of word of 2 bytes
                The 1st word is read from the 1st PCM channel of the 1st
                PCM block and the 2nd word is read from the 1st PCM
                channel of the 2nd PCM block
                (cf channel allocation in bcm_drv_start())
                For each word the most significant byte is in the
                1st timeslot and the least significant byte is in
                the 2nd timeslot.
                We suppose the data is in the native endianness of the
                platform, ie big endian.
               */
               __u8 temp[((BCMPH_PCM_CHANNEL_WIDTH + 1) / 2) * 4];
               const __u8 *src1 = data + dpl->offset_second_block_pcm_channel;
               __u8 *dst = temp;
               bcm_assert((ARRAY_SIZE(temp) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               /* We transfer bytes and not words, because
                casting __u8 * to __u16 * can fail if __u8 * is not
                correctly aligned */
               for (i = 0; (i < BCMPH_PCM_CHANNEL_WIDTH); i += 2) {
                  *dst = *src0;
                  dst += 1;
                  src0 += 1;
                  *dst = *src0;
                  dst += 1;
                  src0 += 1;
                  *dst = *src1;
                  dst += 1;
                  src1 += 1;
                  *dst = *src1;
                  dst += 1;
                  src1 += 1;
               }
               bcm_ring_buf_add(rb, temp, ARRAY_SIZE(temp));
               break;
            }
            case BCMPH_TRF_M_16_BITS_C_xLAW16: {
               /*
                A sample is just a byte but we have two samples per
                frame. The 1st sample is in the 1st timeslot and the 2nd
                sample is in (1st timeslot + (max_timeslots / 2)).
                As the platform is big endian the 1st byte must be read
                from the most significant byte of the words that compose
                the 1st PCM channel of the 1st PCM block, and the 2nd
                byte must be read from the the most significant byte of
                the words that compose the 1st PCM channel of the 2nd
                PCM block
                (cf channel allocation in bcm_drv_start())
               */
               __u8 temp[((BCMPH_PCM_CHANNEL_WIDTH + 1) / 2) * 2];
               const __u8 *src1 = data + dpl->offset_second_block_pcm_channel;
               __u8 *dst = temp;
               bcm_assert((ARRAY_SIZE(temp) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               for (i = 0; (i < BCMPH_PCM_CHANNEL_WIDTH); i += 2) {
                  *dst = *src0;
                  dst += 1;
                  src0 += 2;
                  *dst = *src1;
                  dst += 1;
                  src1 += 2;
               }
               bcm_ring_buf_add(rb, temp, ARRAY_SIZE(temp));
               break;
            }
#ifdef BCMPH_TEST_PCM
            case BCMPH_TRF_TEST_PCM: {
               bcm_assert((dpl->offset_first_block_pcm_channel + dpl->bytes_per_frame) <= data_len);
               bcm_ring_buf_add(rb, src0, dpl->bytes_per_frame);
               break;
            }
#endif // BCMPH_TEST_PCM
            default: {
               bcm_assert(false);
               break;
            }
         }
      }
   }
}

// Try to read PCM data into RX ring buffers
void bcm_drv_fill_rx_buffers(bcm_drv_t *t)
{
   size_t frames_received = 0;

   dd_bcm_pr_debug("bcm_drv_fill_rx_buffers()\n");

   do { // Empty loop
      size_t max_frames_per_buffer;
      size_t min_frames;
      size_t max_frames;
      size_t frame_count;

#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
      if (!pcm_has_rx_buffer_filled(&(t->pcm))) {
         break;
      }
#endif // BCMPH_ENABLE_PCM_INTERRUPTS

      max_frames_per_buffer = pcm_get_max_frame_rx_buffer(&(t->pcm));

      bcm_drv_get_free_space_for_rx(t, &(min_frames), &(max_frames));
      bcm_assert(min_frames <= max_frames);

      if (min_frames > 0) {
         // There is free space so we try to empty as much DMA buffers
         // as possible to get min_frames frames
         frame_count = pcm_receive(&(t->pcm), bcm_drv_pcm_receive_cb, min_frames);
         bcm_assert(frame_count <= min_frames);
         min_frames -= frame_count;
         max_frames -= frame_count;
         frames_received += frame_count;
         if (min_frames >= max_frames_per_buffer) {
            // Here we have enough free space to empty a full DMA
            // buffer so it means no more DMA buffer to empty
            break;
         }
      }
      if ((max_frames <= 0) || (min_frames == max_frames)) {
         // There's no ring buffer with free space,
         // or all the ring buffer have the same space
         // so we exit even if all the DMA buffers are full
         break;
      }

      // If only one line active (which is by far the most common case)
      // as it means min_frames == max_frames, the following code is not executed

      {
         size_t time_to_receive_buffer;
         size_t time_left;
         size_t min_time;
         size_t buffer_count;

         //
         // We count the number of DMA buffers really owned by DMA
         // to compute the time it represents and to decide if we can
         // wait to empty some DMA buffers or not
         time_to_receive_buffer = pcm_get_time_to_receive_rx_buffer(&(t->pcm));
         time_left = time_to_receive_buffer * pcm_get_cnt_rx_buffer_really_owned(&(t->pcm));
         min_time = ((t->board_desc->phone_desc->tick_period * 1000 * 3) / 2);
         if (time_left >= min_time) {
            // We can wait
            break;
         }

         // We can't wait
         buffer_count = ((min_time - time_left + time_to_receive_buffer - 1) / time_to_receive_buffer);
         frame_count = (buffer_count * max_frames_per_buffer);
         if (frame_count > max_frames) {
            frame_count = max_frames;
         }
         frame_count = pcm_rx_reclaim(&(t->pcm), bcm_drv_pcm_receive_cb, buffer_count, frame_count);
         if (frame_count > max_frames) {
            frame_count = max_frames;
         }
         frames_received += frame_count;
      }
   } while (false);

   if (frames_received > 0) {
      // Data received so we unblock processes waiting to read data
      bcm_wait_queue_wake_up(&(t->inq));
   }
}

// Enumerate all active lines to compute the minimum and maximum number of
// DMA frames that can be written
void bcm_drv_get_data_for_tx(bcm_drv_t *t, size_t *min_frame, size_t *max_frame)
{
   size_t i;
   size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->phone_mgr));
   bool min_max_not_set = true;

   dd_bcm_pr_debug("bcm_drv_get_data_for_tx()\n");

   *min_frame = 0;
   *max_frame = 0;
   for (i = 0; (i < phone_line_count); i += 1) {
#ifdef BCMPH_TEST_PCM
      if (i != t->default_line) {
         continue;
      }
#endif // BCMPH_TEST_PCM
      if (bcm_phone_mgr_line_tx_use_pcm(&(t->phone_mgr), i)) {
         size_t count = bcm_ring_buf_get_size(&(t->phone_lines[i].tx_ring_buf)) / t->phone_lines[i].bytes_per_frame;
         if (min_max_not_set) {
            *min_frame = count;
            *max_frame = count;
            min_max_not_set = false;
         }
         else {
            if (*min_frame > count) {
               *min_frame = count;
            }
            else if (*max_frame < count) {
               *max_frame = count;
            }
         }
      }
   }
}

// Callback called by pcm to write a DMA frame
static bool bcm_drv_pcm_send_cb(pcm_t *pcm,
   __u8 *data, size_t data_len)
{
   bcm_drv_t *t = container_of(pcm, bcm_drv_t, pcm);
   size_t i;
   size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->phone_mgr));
   bool frame_filled = false;

   dd_bcm_pr_debug("bcm_drv_empty_tx_ring_buf_cb()\n");

   for (i = 0; (i < phone_line_count); i += 1) {
      bcm_drv_phone_line_t *dpl = &(t->phone_lines[i]);
      bcm_ring_buf_t *rb = &(dpl->tx_ring_buf);
#ifdef BCMPH_TEST_PCM
      if (i != t->default_line) {
         continue;
      }
#endif // BCMPH_TEST_PCM
      // Check that they are enough data in the ring buffer
      if ((bcm_phone_mgr_line_tx_use_pcm(&(t->phone_mgr), i)) && (bcm_ring_buf_get_size(rb) >= dpl->bytes_per_frame)) {
         __u8 *dst0 = data + dpl->offset_first_block_pcm_channel;
         frame_filled = true;
         switch (dpl->type_transfer) {
            case BCMPH_TRF_M_8_BITS_C_xLAW: {
               /*
                A sample is just a byte.
                Data must be sent in 1st channel of the 1st PCM block
                Transfer is straightforward
               */
               bcm_assert((BCMPH_PCM_CHANNEL_WIDTH == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               bcm_ring_buf_remove(rb, dst0, BCMPH_PCM_CHANNEL_WIDTH);
               break;
            }
            case BCMPH_TRF_M_8_BITS_C_LINEAR: {
               /*
                A sample is a word of 2 bytes.
                The most significant byte is in the 1st timeslot
                and the least significant byte is in the next
                timeslot.
                We suppose the data is in the native endianness of the
                platform, ie big endian, so the most significant byte
                must be sent in the 1st PCM channel of the 1st PCM
                block and the least significant byte must be sent in
                the next PCM channel in the same block
                (cf channel allocation in bcm_drv_start())
               */
               __u8 temp[BCMPH_PCM_CHANNEL_WIDTH * 2];
               __u8 *dst1 = dst0 + BCMPH_PCM_CHANNEL_WIDTH;
               const __u8 *src = temp;
               bcm_assert((ARRAY_SIZE(temp) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + (BCMPH_PCM_CHANNEL_WIDTH * 2)) <= data_len));
               bcm_ring_buf_remove(rb, temp, ARRAY_SIZE(temp));
               for (i = 0; (i < BCMPH_PCM_CHANNEL_WIDTH); i += 1) {
                  *dst0 = *src;
                  dst0 += 1;
                  src += 1;
                  *dst1 = *src;
                  dst1 += 1;
                  src += 1;
               }
               break;
            }
            case BCMPH_TRF_M_8_BITS_C_LINEAR16: {
               /*
                A sample is a pair of word of 2 bytes
                The most significant byte of the 1st word is in the
                1st timeslot and the least significant byte of the 1st
                word is in the next timeslot.
                The most significant byte of the 2nd word is in the
                (1st timeslot + (max_timeslots / 2)) and the least
                significant byte of the 2nd word is in the next
                timeslot.
                We suppose the data is in the native endianness of the
                platform, ie big endian, so the most significant byte of
                the 1st word must be sent in the 1st PCM channel of
                the 1st PCM block, the least significant byte of the 1st
                word must be sent in the 2nd PCM channel of the 1st
                PCM block, the most significant byte of the 2nd word
                must be sent in the 1st PCM channel of the 2nd PCM
                block and the least significant byte of the 2nd word
                must be sent in the 2nd PCM channel of the 2nd PCM
                block
                (cf channel allocation in bcm_drv_start())
               */
               __u8 temp[BCMPH_PCM_CHANNEL_WIDTH * 4];
               __u8 *dst1 = dst0 + BCMPH_PCM_CHANNEL_WIDTH;
               __u8 *dst2 = data + dpl->offset_second_block_pcm_channel;
               __u8 *dst3 = dst2 + BCMPH_PCM_CHANNEL_WIDTH;
               const __u8 *src = temp;
               bcm_assert((ARRAY_SIZE(temp) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + (BCMPH_PCM_CHANNEL_WIDTH * 2)) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + (BCMPH_PCM_CHANNEL_WIDTH * 2)) <= data_len));
               bcm_ring_buf_remove(rb, temp, ARRAY_SIZE(temp));
               for (i = 0; (i < BCMPH_PCM_CHANNEL_WIDTH); i += 1) {
                  *dst0 = *src;
                  dst0 += 1;
                  src += 1;
                  *dst1 = *src;
                  dst1 += 1;
                  src += 1;
                  *dst2 = *src;
                  dst2 += 1;
                  src += 1;
                  *dst3 = *src;
                  dst3 += 1;
                  src += 1;
               }
               break;
            }
            case BCMPH_TRF_M_8_BITS_C_xLAW16: {
               /*
                A sample is just a byte but we have two samples per
                frame. The 1st sample is in the 1st timeslot and the 2nd
                sample is (1st timeslot + (max_timeslots / 2)).
                The 1st sample must be sent in the 1st PCM channel of
                the 1st PCM block and the 2nd sample must be sent in
                the 1st PCM channel in the 2nd PCM block
                (cf channel allocation in bcm_drv_start())
               */
               __u8 temp[BCMPH_PCM_CHANNEL_WIDTH * 2];
               __u8 *dst1 = data + dpl->offset_second_block_pcm_channel;
               const __u8 *src = temp;
               bcm_assert((ARRAY_SIZE(temp) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               bcm_ring_buf_remove(rb, temp, ARRAY_SIZE(temp));
               for (i = 0; (i < BCMPH_PCM_CHANNEL_WIDTH); i += 1) {
                  *dst0 = *src;
                  dst0 += 1;
                  src += 1;
                  *dst1 = *src;
                  dst1 += 1;
                  src += 1;
               }
               break;
            }
            case BCMPH_TRF_M_16_BITS_C_xLAW: {
               /*
                A sample is just a byte.
                It is in the 1st timeslot.
                As the platform is big endian the byte must be sent in
                the most significant byte of the words that
                compose the 1st PCM channel of the 1st PCM block
               */
               __u8 temp[(BCMPH_PCM_CHANNEL_WIDTH + 1) / 2];
               const __u8 *src = temp;
               bcm_assert((ARRAY_SIZE(temp) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               bcm_ring_buf_remove(rb, temp, ARRAY_SIZE(temp));
               for (i = 0; (i < BCMPH_PCM_CHANNEL_WIDTH); i += 2) {
                  *dst0 = *src;
                  dst0 += 2;
                  src += 1;
               }
               break;
            }
            case BCMPH_TRF_M_16_BITS_C_LINEAR: {
               /*
                A sample is a word of 2 bytes.
                The 2 bytes are sent in the 1st PCM channel of the
                1st PCM block
                The most significant byte is in the 1st timeslot,
                and the least significant byte is in the 2nd
                timeslot.
                We suppose the data is in the native endianness of the
                platform, ie big endian, so there no need to swap the
                bytes
               */
               bcm_assert((BCMPH_PCM_CHANNEL_WIDTH == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               bcm_ring_buf_remove(rb, dst0, BCMPH_PCM_CHANNEL_WIDTH);
               break;
            }
            case BCMPH_TRF_M_16_BITS_C_LINEAR16: {
               /*
                A sample is a pair of word of 2 bytes
                The 1st word is sent in the 1st PCM channel of the 1st
                PCM block and the 2nd word is sent in the 1st PCM
                channel of the 2nd PCM block
                (cf channel allocation in bcm_drv_start())
                For each word the most significant byte is in the
                1st timeslot and the least significant byte is in
                the 2nd timeslot.
                We suppose the data is in the native endianness of the
                platform, ie big endian.
               */
               __u8 temp[((BCMPH_PCM_CHANNEL_WIDTH + 1) / 2) * 4];
               __u8 *dst1 = data + dpl->offset_second_block_pcm_channel;
               const __u8 *src = temp;
               bcm_assert((ARRAY_SIZE(temp) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               bcm_ring_buf_remove(rb, temp, ARRAY_SIZE(temp));
               /* We transfer bytes and not words, because
                casting __u8 * to __u16 * can fail if __u8 * is not
                correctly aligned */
               for (i = 0; (i < BCMPH_PCM_CHANNEL_WIDTH); i += 2) {
                  *dst0 = *src;
                  dst0 += 1;
                  src += 1;
                  *dst0 = *src;
                  dst0 += 1;
                  src += 1;
                  *dst1 = *src;
                  dst1 += 1;
                  src += 1;
                  *dst1 = *src;
                  dst1 += 1;
                  src += 1;
               }
               break;
            }
            case BCMPH_TRF_M_16_BITS_C_xLAW16: {
               /*
                A sample is just a byte but we have two samples per
                frame. The 1st sample is in the 1st timeslot and the 2nd
                sample is in (1st timeslot + (max_timeslots / 2)).
                As the platform is big endian the 1st byte must be sent
                in the most significant byte of the words that compose
                the 1st PCM channel of the 1st PCM block, and the 2nd
                byte must be sent in the the most significant byte of
                the words that compose the 1st PCM channel of the 2nd
                PCM block
                (cf channel allocation in bcm_drv_start())
               */
               __u8 temp[((BCMPH_PCM_CHANNEL_WIDTH + 1) / 2) * 2];
               const __u8 *src = temp;
               __u8 *dst1 = data + dpl->offset_second_block_pcm_channel;
               bcm_assert((ARRAY_SIZE(temp) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               bcm_ring_buf_remove(rb, temp, ARRAY_SIZE(temp));
               for (i = 0; (i < BCMPH_PCM_CHANNEL_WIDTH); i += 2) {
                  *dst0 = *src;
                  dst0 += 2;
                  src += 1;
                  *dst1 = *src;
                  dst1 += 2;
                  src += 1;
               }
               break;
            }
#ifdef BCMPH_TEST_PCM
            case BCMPH_TRF_TEST_PCM: {
               bcm_assert((dpl->offset_first_block_pcm_channel + dpl->bytes_per_frame) <= data_len);
               bcm_ring_buf_remove(rb, dst0, dpl->bytes_per_frame);
               break;
            }
#endif // BCMPH_TEST_PCM
            default: {
               bcm_assert(false);
               break;
            }
         }
      }
   }
   return (frame_filled);
}

// Try to write PCM data from TX ring buffers
// Flag fsync tells if we accept to write PCM data that don't fill an
// entire TX DMA buffer
void bcm_drv_empty_tx_buffers(bcm_drv_t *t, bool fsync)
{
   size_t frames_sent = 0;

   dd_bcm_pr_debug("bcm_drv_empty_tx_buffers(fsync=%d)\n", (int)(fsync));

   do { // Empty loop
      size_t min_frames;
      size_t max_frames;
      size_t frame_count;

#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
      if (!pcm_has_tx_buffer_empty(&(t->pcm))) {
         break;
      }
#endif // BCMPH_ENABLE_PCM_INTERRUPTS

      bcm_drv_get_data_for_tx(t, &(min_frames), &(max_frames));
      bcm_assert(min_frames <= max_frames);

      if (fsync) {
         if (max_frames <= 0) {
            // Nothing to do because no more data
            break;
         }
         // Here we try to transmit all the frames available
         frame_count = pcm_send(&(t->pcm), bcm_drv_pcm_send_cb, max_frames);
         bcm_assert(frame_count <= max_frames);
         frames_sent += frame_count;
      }
      else {
         size_t max_frames_per_buffer = pcm_get_max_frame_tx_buffer(&(t->pcm));

         if (min_frames >= max_frames_per_buffer) {
            // Enough frames to fully fill at least one DMA buffer
            frame_count = pcm_send(&(t->pcm), bcm_drv_pcm_send_cb, round_down_generic(min_frames, max_frames_per_buffer));
            bcm_assert(frame_count <= min_frames);
            min_frames -= frame_count;
            max_frames -= frame_count;
            frames_sent += frame_count;
            if (min_frames >= max_frames_per_buffer) {
               // Some frames sent but more could have been sent so it means
               // no more DMA buffer to fill
               break;
            }
         }

         // We don't allow non full TX DMA buffer because it will make
         // difficult to compute the time that the DMA buffers really owned
         // represents
         if (max_frames < max_frames_per_buffer) {
            // There's not enough data in ring buffer so we exit even if all
            // the DMA buffers are empty
            break;
         }

         // Here we can't have min_frames == max_frames because
         // ((min_frames < max_frames_per_buffer) && (max_frames >= max_frames_per_buffer))
         // So if only one line active (which is by far the most common case)
         // as it means min_frames == max_frames, the following code is not executed

         {
            size_t time_to_send_buffer;
            size_t time_left;
            size_t min_time;

            // We count the number of DMA buffers owned by DMA
            // to compute the time before DMA raises a TX underrun condition,
            // and then to decide if we can wait to fill some DMA buffers or not
            time_to_send_buffer = pcm_get_time_to_send_tx_buffer(&(t->pcm));
            time_left = time_to_send_buffer * pcm_get_cnt_tx_buffer_really_owned(&(t->pcm));
            min_time = ((t->board_desc->phone_desc->tick_period * 1000 * 3) / 2);
            if (time_left >= min_time) {
               // We can wait
               break;
            }
            // We can't wait
            frame_count = (((min_time - time_left + time_to_send_buffer - 1) / time_to_send_buffer) * max_frames_per_buffer);
            if (frame_count > max_frames) {
               // Don't forget to round_down to forbid non full DMA buffer
               frame_count = round_down_generic(max_frames, max_frames_per_buffer);
            }
            frame_count = pcm_send(&(t->pcm), bcm_drv_pcm_send_cb, frame_count);
            bcm_assert(frame_count <= max_frames);
            frames_sent += frame_count;
         }
      }
   } while (false);

   if (frames_sent > 0) {
      // Data sent so we unblock processes waiting for to write data
      bcm_wait_queue_wake_up(&(t->outq));
   }
}

static void bcm_drv_resume_read_cb(pcm_t *pcm)
{
   bcm_drv_t *t = container_of(pcm, bcm_drv_t, pcm);
   dd_bcm_pr_debug("bcm_drv_resume_read_cb()\n");
   bcm_wait_queue_wake_up(&(t->inq));
}

ssize_t bcm_drv_read_line(bcm_drv_t *t,
   bool do_not_block, bcmph_mutex_t *lock,
   size_t line, __u8 *buf, size_t count)
{
   ssize_t ret = 0;
   int wq_counter;

   dd_bcm_pr_debug("bcm_drv_read_line(line=%lu, buf=%lx, count=%lu)\n",
      (unsigned long)(line), (unsigned long)(buf), (unsigned long)(count));

   bcm_assert((NULL != t) && (line < bcm_phone_mgr_get_line_count(&(t->phone_mgr)))
      && (NULL != lock) && (bcmph_mutex_is_locked(lock)));

   // We must init the counter before updating the RX buffers
   // to be sure to not loose an event on inq that would occur after
   // updating the buffers
   wq_counter = bcm_wait_queue_get_counter(&(t->inq));

   bcm_drv_update_buffers(t);

   do { // Empty loop
      bcm_drv_phone_line_t *dpl = &(t->phone_lines[line]);
      if (!bcm_phone_mgr_line_is_enabled(&(t->phone_mgr), line)) {
         ret = -EFAULT;
         break;
      }

      if (count <= 0) {
         break;
      }

#ifdef __KERNEL__
      if (!access_ok(VERIFY_WRITE, buf, count)) {
         ret = -EFAULT;
         break;
      }
#endif // __KERNEL__

      for (;;) {
         size_t len;

         len = bcm_ring_buf_get_size(&(dpl->rx_ring_buf));
         if (len > 0) {
            // RX buffer is not empty, so we can read data from it
            dd_bcm_pr_debug("Reading from RX buffer that contains %lu bytes\n", (unsigned long)(len));
            if (len > count) {
               len = count;
            }
            if (bcm_ring_buf_remove_to_user(&(dpl->rx_ring_buf), buf, len)) {
               bcm_pr_debug("Read() stopped because copy_to_user() failed\n");
               break;
            }
            buf += len;
            count -= len;
            ret += len;
            bcm_drv_fill_rx_buffers(t);
            if (count <= 0) {
               break;
            }
         }
         else {
            int tmp;
            // Nothing to read
            if ((ret) || (!bcm_phone_mgr_line_rx_use_pcm(&(t->phone_mgr), line))) {
               // We have read at least one byte, or we are not in a state where
               // we can receive data, so we exit
               break;
            }
            if (do_not_block) {
               // We are not allowed to block so we exit
               ret = -EAGAIN;
               dd_bcm_pr_debug("Nothing read, returned %ld\n", (long)(ret));
               break;
            }
            dd_bcm_pr_debug("Waiting for the availability of free space in RX ring buffer\n");
            tmp = bcm_wait_queue_wait_event_counter_timeout(&(t->inq),
               wq_counter,
               msecs_to_jiffies(t->board_desc->phone_desc->tick_period), lock);
            if (tmp < 0) {
               ret = tmp;
               break;
            }
            // We don't forget to update the counter for the next call
            // to wait_event, before updating the RX buffers
            barrier();
            wq_counter = bcm_wait_queue_get_counter(&(t->inq));
            bcm_drv_fill_rx_buffers(t);
         }
      }
   } while (false);

   return (ret);
}

#ifdef __KERNEL__
static
#endif // __KERNEL__
ssize_t bcm_drv_read(struct file *filp, char *buf, size_t count,
   loff_t *offset)
{
   ssize_t ret = 0;
   bcm_drv_t *t = (bcm_drv_t *)(filp->private_data);
   bcmph_mutex_t lock;

   dd_bcm_pr_debug("bcm_drv_read(buf=%lx, count=%lu)\n", (unsigned long)(buf), (unsigned long)(count));

   bcm_assert(NULL != t);

   bcmph_mutex_init(&(lock), &(t->lock), false);

   if (bcmph_mutex_lock_interruptible(&(lock))) {
      ret = -ERESTARTSYS;
   }
   else {
      ret = bcm_drv_read_line(t,
         ((filp->f_flags & O_NONBLOCK) != 0), &(lock),
         t->default_line, (__u8 *)(buf), count);
      if (ret > 0) {
         bcm_assert((0 == (ret % t->phone_lines[t->default_line].bytes_per_frame)) && (((size_t)(ret)) <= count));
         if (NULL != offset) {
            *offset += ret;
         }
      }
   }

   bcmph_mutex_deinit(&(lock));

   return (ret);
}

static void bcm_drv_resume_write_cb(pcm_t *pcm)
{
   bcm_drv_t *t = container_of(pcm, bcm_drv_t, pcm);
   dd_bcm_pr_debug("bcm_drv_resume_write_cb()\n");
   bcm_wait_queue_wake_up(&(t->outq));
}

ssize_t bcm_drv_write_line(bcm_drv_t *t,
   bool do_not_block, bcmph_mutex_t *lock,
   size_t line, const __u8 *buf, size_t count)
{
   ssize_t ret = 0;
   int wq_counter;

   dd_bcm_pr_debug("bcm_drv_write_line(line=%lu, buf=%lx, count=%lu)\n",
      (unsigned long)(line), (unsigned long)(buf), (unsigned long)(count));

   bcm_assert((NULL != t) && (line < bcm_phone_mgr_get_line_count(&(t->phone_mgr)))
      && (NULL != lock) && (bcmph_mutex_is_locked(lock)));

   // We must init the counter before updating the TX buffers
   // to be sure to not loose an event on outq that would occur after
   // updating the buffers
   wq_counter = bcm_wait_queue_get_counter(&(t->outq));

   bcm_drv_update_buffers(t);


   do { // Empty loop
      bcm_drv_phone_line_t *dpl = &(t->phone_lines[line]);

      if (!bcm_phone_mgr_line_is_enabled(&(t->phone_mgr), line)) {
         ret = -EFAULT;
         break;
      }

      if (count <= 0) {
         break;
      }

#ifdef __KERNEL__
      if (!access_ok(VERIFY_READ, buf, count)) {
         ret = -EFAULT;
         break;
      }
#endif // __KERNEL__

      dd_bcm_pr_debug("Writing to TX buffer that contains %lu bytes\n", (unsigned long)(bcm_ring_buf_get_size(&(dpl->tx_ring_buf))));
      for (;;) {
         size_t len;

         len = bcm_ring_buf_get_free_space(&(dpl->tx_ring_buf));
         if (len > 0) {
            if (len > count) {
               len = count;
            }
            dd_bcm_pr_debug("Copying %lu bytes into TX buffer, from user buffer\n", (unsigned long)(len));
            if (bcm_ring_buf_add_from_user(&(dpl->tx_ring_buf), buf, len)) {
               bcm_pr_debug("Write() stopped because copy_from_user() failed\n");
               break;
            }
            buf += len;
            count -= len;
            ret += len;
            bcm_drv_empty_tx_buffers(t, false);
            if (count <= 0) {
               break;
            }
         }
         else {
            int tmp;
            if ((ret) || (!bcm_phone_mgr_line_tx_use_pcm(&(t->phone_mgr), line))) {
               // We have written at least one byte, or we are not in a state where
               // we can write data, so we exit
               break;
            }
            if (do_not_block) {
               // We are not allowed to block so we exit
               ret = -EAGAIN;
               dd_bcm_pr_debug("Nothing written, returned %ld\n", (long)(ret));
               break;
            }
            dd_bcm_pr_debug("Waiting for the availability of space in TX ring buffer\n");
            tmp = bcm_wait_queue_wait_event_counter_timeout(&(t->outq),
               wq_counter,
               msecs_to_jiffies(t->board_desc->phone_desc->tick_period), lock);
            if (tmp < 0) {
               ret = tmp;
               break;
            }
            // We don't forget to update the counter for the next call
            // to wait_event, before updating the TX buffers
            barrier();
            wq_counter = bcm_wait_queue_get_counter(&(t->outq));
            bcm_drv_empty_tx_buffers(t, false);
         }
      }
   } while (false);

   return (ret);
}

#ifdef __KERNEL__
static
#endif // __KERNEL__
ssize_t bcm_drv_write(struct file *filp, const char *buf,
            size_t count, loff_t *offset)
{
   ssize_t ret = 0;
   bcm_drv_t *t = (bcm_drv_t *)(filp->private_data);
   bcmph_mutex_t lock;

   dd_bcm_pr_debug("bcm_drv_write(buf=%lx, count=%lu)\n", (unsigned long)(buf), (unsigned long)(count));

   bcm_assert(NULL != t);

   bcmph_mutex_init(&(lock), &(t->lock), false);

   if (bcmph_mutex_lock_interruptible(&(lock))) {
      ret = -ERESTARTSYS;
   }
   else {
      ret = bcm_drv_write_line(t, ((filp->f_flags & O_NONBLOCK) != 0), &(lock),
         t->default_line, (const __u8 *)(buf), count);
      if (ret > 0) {
         bcm_assert(((size_t)(ret)) <= count);
         if (NULL != offset) {
            *offset += ret;
         }
      }
   }

   bcmph_mutex_deinit(&(lock));

   return (ret);
}

#ifdef __KERNEL__
static unsigned int bcm_drv_poll(struct file *filp, struct poll_table_struct *poll_table)
{
   unsigned int ret = 0;
   bcm_drv_t *t = filp->private_data;
   bcmph_mutex_t lock;

   d_bcm_pr_debug("bcm_drv_poll()\n");

   bcm_assert(NULL != t);

   bcmph_mutex_init(&(lock), &(t->lock), false);

   // If we can't get the semaphore we return 0
   if (!bcmph_mutex_lock_interruptible(&(lock))) {
      size_t line = t->default_line;
      bcm_drv_phone_line_t *dpl = &(t->phone_lines[line]);

      if (NULL != poll_table) {
         // Allow the kernel to be notified of a change in the status of
         // this file
         // Does not block the call
         poll_wait(filp, &(t->inq.wq), poll_table);
         poll_wait(filp, &(t->outq.wq), poll_table);
      }

      bcm_drv_update_buffers(t);

      // If pcm is stopped, we lie and tell that data are ready,
      // because we can't wait for data we can't receive
      if ((bcm_ring_buf_get_size(&(dpl->rx_ring_buf)) > 0) || (!bcm_phone_mgr_line_rx_use_pcm(&(t->phone_mgr), line))) {
         /* readable */
         d_bcm_pr_debug("Can read\n");
         ret |= (POLLIN | POLLRDNORM);
      }
      // If pcm is stopped, we lie and tell that we can write data
      // because we can't wait for pending data to be transmitted as we can't transfer
      if ((bcm_ring_buf_get_free_space(&(dpl->tx_ring_buf)) > 0) || (!bcm_phone_mgr_line_tx_use_pcm(&(t->phone_mgr), line))) {
         /* writable */
         d_bcm_pr_debug("Can write\n");
         ret |= (POLLOUT | POLLWRNORM);
      }
   }

   bcmph_mutex_deinit(&(lock));

   return (ret);
}

static int bcm_drv_fsync(struct file *filp, loff_t start, loff_t end, int datasync)
{
   int ret = 0;
   bcm_drv_t *t = filp->private_data;
   int wq_counter;
   bcmph_mutex_t lock;

   d_bcm_pr_debug("bcm_drv_fsync()\n");

   bcm_assert(NULL != t);

   bcmph_mutex_init(&(lock), &(t->lock), false);

   do { // Empty loop
      size_t dummy;
      size_t max_frames;

      if (bcmph_mutex_lock_interruptible(&(lock))) {
         ret = -ERESTARTSYS;
         break;
      }

      for (;;) {
         if (!pcm_dma_is_started(&(t->pcm))) {
            break;
         }
         // We init the counter before updating the TX buffers
         // to be sure to not loose an event on outq that would occur after
         // updating the buffers
         barrier();
         wq_counter = bcm_wait_queue_get_counter(&(t->outq));
         bcm_drv_empty_tx_buffers(t, true);
         bcm_drv_get_data_for_tx(t, &(dummy), &(max_frames));
         if (max_frames <= 0) {
            break;
         }
         if (pcm_tx_reclaim(&(t->pcm), 1) <= 0) {
            d_bcm_pr_debug("Waiting for the availability of a DMA TX buffer\n");
            ret = bcm_wait_queue_wait_event_counter_timeout(&(t->outq),
               wq_counter, msecs_to_jiffies(t->board_desc->phone_desc->tick_period), &(lock));
            if (ret < 0) {
               break;
            }
#ifdef BCMPH_NOHW
            bcm_drv_fill_rx_buffers(t);
#endif // BCMPH_NOHW
         }
      }
   } while (false);

   bcmph_mutex_deinit(&(lock));

   return (ret);
}
#endif // __KERNEL__

static bcm_drv_t bcm_drv;

#ifdef __KERNEL__
static
#endif // __KERNEL__
int bcm_drv_open(struct inode *inode, struct file *filp)
{
   int ret = 0;
   bcm_drv_t *t = &(bcm_drv);
   bcmph_mutex_t lock;

   bcm_pr_debug("bcm_drv_open()\n");

   bcmph_mutex_init(&(lock), &(t->lock), false);

   do { // Empty loop
      if (bcmph_mutex_lock_interruptible(&(lock))) {
         ret = -ERESTARTSYS;
         break;
      }

      if (0 != t->open_count) {
         /* -EPERM would confuse the user */
         bcm_pr_debug("open() failed because open_count != 0 (%ld)\n", (long)(t->open_count));
         ret = -EBUSY;
         break;
      }

      t->open_count += 1;

#ifdef __KERNEL__
      ret = nonseekable_open(inode, filp);
      if (ret) {
         break;
      }
#endif // __KERNEL__

      filp->private_data = t;
   } while (false);

   bcmph_mutex_deinit(&(lock));

   return (ret);
}

#ifdef __KERNEL__
static
#endif // __KERNEL__
int bcm_drv_release(struct inode *inode, struct file *filp)
{
   int ret = 0;
   bcm_drv_t *t = (bcm_drv_t *)(filp->private_data);
   bcmph_mutex_t lock;

   bcm_pr_debug("bcm_drv_release()\n");

   bcm_assert(NULL != t);

   bcmph_mutex_init(&(lock), &(t->lock), false);

   if (!bcmph_mutex_lock_interruptible(&(lock))) {
      bcm_drv_stop(t, 0);
      t->open_count -= 1;
   }
   else {
      ret = -ERESTARTSYS;
   }

   bcmph_mutex_deinit(&(lock));

   return (ret);
}


#ifdef __KERNEL__
/* Dirty? Well, I still did not learn better way to account
 * for user mmaps.
 */

static void bcm_drv_mm_open(struct vm_area_struct *vma)
{
   bcm_pr_debug("bcmp_phone_mm_open()");
}

static void bcm_drv_mm_close(struct vm_area_struct *vma)
{
   bcm_pr_debug("bcm_drv_mm_close()");
}

static int bcm_drv_mm_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
   bcm_pr_debug("bcm_drv_mm_fault()");
   return (VM_FAULT_SIGBUS);
}


static const struct vm_operations_struct bcm_drv_mmap_ops = {
   .open = bcm_drv_mm_open,
   .close = bcm_drv_mm_close,
   .fault = bcm_drv_mm_fault,
};

static int bcm_drv_mmap(struct file *filp, struct vm_area_struct *vma)
{
   bcm_drv_t *t = filp->private_data;
   int ret = 0;
   bcmph_mutex_t lock;

   bcm_pr_debug("bcm_drv_mmap()");

   bcm_assert(NULL != t);

   bcmph_mutex_init(&(lock), &(t->lock), false);

   do { // Empty loop
      unsigned long start;
      u8 *kaddr;
      size_t pg_count;
      size_t pg_num;

      if (vma->vm_pgoff) {
         ret = -EINVAL;
         break;
      }

      if (bcmph_mutex_lock_interruptible(&(lock))) {
         ret = -ERESTARTSYS;
         break;
      }

      pg_count = 1 << t->mm_buffer_order;

      if ((vma->vm_end - vma->vm_start) != (PAGE_SIZE * pg_count)) {
         ret = -EINVAL;
         break;
      }

      start = vma->vm_start;
      kaddr = t->mm_buffer;
      for (pg_num = 0; (pg_num < pg_count); pg_num += 1) {
         struct page *page = virt_to_page(kaddr);
         ret = vm_insert_page(vma, start, page);
         if (unlikely(ret)) {
            break;
         }
         start += PAGE_SIZE;
         kaddr += PAGE_SIZE;
      }
      if (pg_num < pg_count) {
         break;
      }

      vma->vm_ops = &(bcm_drv_mmap_ops);
      vma->vm_private_data = t;
      ret = 0;

      bcmph_mutex_unlock(&(lock));

      bcm_drv_mm_open(vma);
   } while (false);

   bcmph_mutex_deinit(&(lock));

   return (ret);
}

static struct file_operations bcm_drv_fops = {
   .owner          = THIS_MODULE,
   .llseek         = no_llseek,
   .read           = bcm_drv_read,
   .write          = bcm_drv_write,
   .poll           = bcm_drv_poll,
   .unlocked_ioctl = bcm_drv_unlocked_ioctl,
   .open           = bcm_drv_open,
   .release        = bcm_drv_release,
   .fsync          = bcm_drv_fsync,
   .mmap           = bcm_drv_mmap,
};
#endif // __KERNEL__

/*
 * entry point
 */
#ifdef __KERNEL__
static
#endif // __KERNEL__
int __init bcm_drv_init(void)
{
   int ret;
   const board_desc_t *board_desc;
   bcm_drv_t *t = &(bcm_drv);

   bcm_pr_info("%s: v%s\n", driver_name, VERSION);

   bcm_pr_debug("bcm_drv_init()\n");

   memset(t, 0, sizeof(*t));

   ret = board_init();
   if (ret) {
      goto fail_board;
   }

   board_desc = board_get_desc();
   bcm_assert((NULL != board_desc) && (NULL != board_desc->phone_desc)
      && (board_desc->phone_desc->device_count > 0));
   t->board_desc = board_desc;

   // mutex_init() should be called before initializing the wait queues
   bcm_pr_debug("Initializing mutex\n");
   mutex_init(&(t->lock));

   bcm_pr_debug("Initializing RX wait queue\n");
   bcm_wait_queue_init(&(t->inq));
   bcm_pr_debug("Initializing TX wait queue\n");
   bcm_wait_queue_init(&(t->outq));

   // As we have to register the device before calling pcm_init,
   // we set open_count to -1 so open() will fail if it is called before
   // init function terminates successfully
   t->open_count = -1;

#ifdef __KERNEL__
   /* register device with kernel */
   memset(&(t->misc_dev), 0, sizeof(t->misc_dev));
   t->misc_dev.minor = MISC_DYNAMIC_MINOR;
   t->misc_dev.name = driver_name;
   t->misc_dev.fops = &(bcm_drv_fops);

   bcm_pr_debug("Registering device %s\n", t->misc_dev.name);
   ret = misc_register(&(t->misc_dev));
   if (ret) {
      bcm_pr_err("Can't register device\n");
      goto fail_register_dev;
   }
#endif // __KERNEL__

   ret = pcm_init(&(t->pcm),
#ifdef __KERNEL__
      t->misc_dev.this_device,
#endif // __KERNEL__
      t->board_desc,
      bcm_drv_resume_read_cb, bcm_drv_resume_write_cb);
   if (ret) {
      goto fail_pcm;
   }

   // Alloc main buffer used for RX and TX and that can be mapped
   {
      size_t min_buf_size;
      size_t rx_buf_len;
      size_t tx_buf_len;
      size_t i;
      size_t offset;
      size_t size;

      // At a frequency of 8 kHz, we have 8 samples/ms
      // A sample can be up to 4 bytes (BCMPH_CODEC_LINEAR16)
      // So we need 32 bytes/ms
#ifndef BCMPH_NOHW
      // We choose a buffer size allowing storage of at least
      // (BCMPH_MIN_SIZE_RING_BUFFER * 2) rounded to a multiple of
      // t->board_desc->phone_desc->tick_period in time
      min_buf_size = ((((((BCMPH_MIN_SIZE_RING_BUFFER + 31) / 32) * 2) + t->board_desc->phone_desc->tick_period  - 1)
                       / t->board_desc->phone_desc->tick_period)
                      * t->board_desc->phone_desc->tick_period * 32);
#else // BCMPH_NOHW
      min_buf_size = (t->board_desc->phone_desc->tick_period * 2 * 32);
#endif
      bcm_assert((min_buf_size >= (2 * BCMPH_PCM_CHANNEL_WIDTH)) && (RX_BUFFER_PERCENT > 0) && (RX_BUFFER_PERCENT < 100));

      rx_buf_len = ((min_buf_size * 2 * RX_BUFFER_PERCENT) / 100);
      rx_buf_len = round_up_to_pow_of_2(round_up_generic(rx_buf_len, BCMPH_RB_ROUNDING_FACTOR), BCMPH_MMAP_ALIGNMENT);
      tx_buf_len = ((min_buf_size * 2 * (100 - RX_BUFFER_PERCENT)) / 100);
      tx_buf_len = round_up_to_pow_of_2(round_up_generic(tx_buf_len, BCMPH_RB_ROUNDING_FACTOR), BCMPH_MMAP_ALIGNMENT);

      t->mm_desc.ioctl_param_off = 0;
      t->mm_desc.ioctl_param_size = sizeof(bcm_phone_ioctl_param_t);
      bcm_pr_debug("t->mm_desc.ioctl_param_off = %lu, t->mm_desc.ioctl_param_size = %lu\n",
         (unsigned long)(t->mm_desc.ioctl_param_off), (unsigned long)(t->mm_desc.ioctl_param_size));

      offset = t->mm_desc.ioctl_param_off + round_up_to_pow_of_2(t->mm_desc.ioctl_param_size, BCMPH_MMAP_ALIGNMENT);
      bcm_assert(ARRAY_SIZE(t->phone_lines) == ARRAY_SIZE(t->mm_rbs_location.rbs));
      for (i = 0; (i < ARRAY_SIZE(t->mm_rbs_location.rbs)); i += 1) {
         size = sizeof(bcm_ring_buf_desc_t);
         t->mm_rbs_location.rbs[i].rx_ring_buf_desc_off = offset;
         t->mm_rbs_location.rbs[i].rx_ring_buf_desc_size = size;
         bcm_pr_debug("t->mm_desc.rbs[%lu].rx_ring_buf_desc_off = %lu, t->mm_desc.rbs[%lu].rx_ring_buf_desc_size = %lu\n",
            (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].rx_ring_buf_desc_off),
            (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].rx_ring_buf_desc_size));
         offset += round_up_to_pow_of_2(size, BCMPH_MMAP_ALIGNMENT);

         size = sizeof(bcm_ring_buf_desc_t);
         t->mm_rbs_location.rbs[i].tx_ring_buf_desc_off = offset;
         t->mm_rbs_location.rbs[i].tx_ring_buf_desc_size = size;
         bcm_pr_debug("t->mm_desc.rbs[%lu].tx_ring_buf_desc_off = %lu, t->mm_desc.rbs[%lu].tx_ring_buf_desc_size = %lu\n",
            (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].tx_ring_buf_desc_off),
            (unsigned long)(i), (unsigned long)(t->mm_rbs_location.rbs[i].tx_ring_buf_desc_size));
         offset += round_up_to_pow_of_2(size, BCMPH_MMAP_ALIGNMENT);
      }
      t->offset_rbs = offset;
      t->mm_buffer_order = get_order(offset
         + (rx_buf_len + tx_buf_len) * ARRAY_SIZE(t->mm_rbs_location.rbs));
      t->mm_desc.mmap_size = (PAGE_SIZE << t->mm_buffer_order);
      t->size_rbs = t->mm_desc.mmap_size - t->offset_rbs;

      // Allocate the buffer
      // Do not omit flag __GFP_COMP, else calls to vm_insert_page() in
      // bcm_drv_mmap() will fail
      // We don't use alloc_pages_exact because when we read kernel sources
      // we see that the function calls split_page() that doesn't expect the
      // flag __GFP_COMP
      t->mm_buffer = (__u8 *)(__get_free_pages(GFP_KERNEL | __GFP_ZERO | __GFP_NORETRY | __GFP_COMP, t->mm_buffer_order));
      if (NULL == t->mm_buffer) {
         bcm_pr_err("Cannot allocate mmap buffer (%lu bytes) used for RX, TX...\n",
            (unsigned long)(t->mm_desc.mmap_size));
         ret = -ENOMEM;
         goto fail_buf;
      }
      bcm_pr_debug("%lu bytes of memory allocated, starting at address 0x%lx\n", (unsigned long)(t->mm_desc.mmap_size), (unsigned long)(t->mm_buffer));
   }

   ret = bcm_phone_mgr_init(&(t->phone_mgr), t->board_desc, bcm_drv_timer_cb);
   if (ret) {
      goto fail_phones;
   }

   bcm_assert(ARRAY_SIZE(t->phone_lines) >= bcm_phone_mgr_get_line_count(&(t->phone_mgr)));

   // Now we can set open_count to 0 to allow open() to success,
   // as we complete successfully the init of the driver
   t->open_count = 0;

   bcm_pr_debug("%s successfully initialized\n", driver_name);

   return (0);

   bcm_phone_mgr_deinit(&(t->phone_mgr));
fail_phones:
   bcm_pr_debug("Freeing mmap buffer used for RX, TX...\n");
   free_pages((unsigned long)(t->mm_buffer), t->mm_buffer_order);
fail_buf:
   pcm_deinit(&(t->pcm));
fail_pcm:
#ifdef __KERNEL__
   bcm_pr_debug("Unregistering device\n");
   misc_deregister(&(t->misc_dev));
fail_register_dev:
#endif // __KERNEL__
   bcm_wait_queue_deinit(&(t->outq));
   bcm_wait_queue_deinit(&(t->inq));
   mutex_destroy(&(t->lock));
   board_deinit();
fail_board:
   return (ret);
}

#ifdef __KERNEL__
static
#endif // __KERNEL__
void __exit bcm_drv_exit(void)
{
   bcm_drv_t *t = &(bcm_drv);

   bcm_phone_mgr_deinit(&(t->phone_mgr));
   bcm_pr_debug("Freeing mmap buffer used for RX, TX...\n");
   free_pages((unsigned long)(t->mm_buffer), t->mm_buffer_order);
   pcm_deinit(&(t->pcm));
   bcm_pr_debug("Unregistering device\n");
#ifdef __KERNEL__
   misc_deregister(&(t->misc_dev));
#endif // __KERNEL__
   bcm_wait_queue_deinit(&(t->outq));
   bcm_wait_queue_deinit(&(t->inq));
   mutex_destroy(&(t->lock));
   board_deinit();
   bcm_pr_debug("%s removed\n", driver_name);
}

#ifdef __KERNEL__
module_init(bcm_drv_init);
module_exit(bcm_drv_exit);

MODULE_DESCRIPTION("bcm63xx-phone driver");
MODULE_AUTHOR("Gilles Mazoyer <mazoyer.gilles@omega.ovh>");
MODULE_LICENSE("GPL");
MODULE_VERSION(VERSION);
#endif // __KERNEL__
