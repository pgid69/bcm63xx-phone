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
#endif // __KERNEL__
#include <extern/linux/errno.h>
#include <extern/linux/kernel.h>
#include <extern/linux/mm.h>
#include <extern/linux/module.h>

#ifdef BCMPH_TEST_PCM
#include <bcm63xx.h>
#endif // BCMPH_TEST_PCM
#include <bcm63xx_phone.h>
#include <bcm63xx_log.h>

#include "main.h"
#include "utils.h"

// Include after system files
#include <compile.h>

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
   bcm_drv_t *t = container_of(phone_mgr, bcm_drv_t, core.phone_mgr);

   dd_bcm_pr_debug("%s()\n", __func__);

#ifdef BCMPH_EXPORT_DEV_FILE
   // If we can get the global lock we try to :
   // - fill TX DMA buffers
   // - empty RX DMA buffers
   // BEWARE : mutex_trylock() return 1 on success and 0 if mutex can't be locked
   if (mutex_trylock(&(t->dev.lock))) {
#endif /* BCMPH_EXPORT_DEV_FILE */
      bcm_drv_update_buffers(t);
#ifdef BCMPH_EXPORT_DEV_FILE
      mutex_unlock(&(t->dev.lock));
   }
#endif /* BCMPH_EXPORT_DEV_FILE */
}

// Enumerate all active lines to compute the minimum and maximum number of
// DMA frames that can be read
static void bcm_drv_get_free_space_for_rx(bcm_drv_t *t, size_t *min_frame, size_t *max_frame)
{
   size_t line_idx;
   size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->core.phone_mgr));
   bool min_max_not_set = true;

   dd_bcm_pr_debug("%s()\n", __func__);

   *min_frame = 0;
   *max_frame = 0;
   for (line_idx = 0; (line_idx < phone_line_count); line_idx += 1) {
      if (bcm_phone_mgr_line_rx_use_pcm(&(t->core.phone_mgr), line_idx)) {
         size_t count = bcm_ring_buf_get_free_space(&(t->core.phone_lines[line_idx].rx_ring_buf)) / t->core.phone_lines[line_idx].bytes_per_frame;
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
   bcm_drv_t *t = container_of(pcm, bcm_drv_t, core.pcm);
   size_t line_idx;
   size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->core.phone_mgr));

   dd_bcm_pr_debug("%s()\n", __func__);

   for (line_idx = 0; (line_idx < phone_line_count); line_idx += 1) {
      // Check that they are enough room in the ring buffer
      if (bcm_phone_mgr_line_rx_use_pcm(&(t->core.phone_mgr), line_idx)) {
         bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line_idx]);
         __u8 *dst = &(dpl->rx_buffer.array[dpl->rx_buffer.len]);
         const __u8 *src0 = data + dpl->offset_first_block_pcm_channel;
         size_t width;

         switch (dpl->type_transfer) {
            case BCMPH_TRF_M_8_BITS_C_xLAW: {
               /*
                A sample is just a byte.
                Data must be read from the 1st channel of the 1st PCM
                block.
                Transfer is straightforward
               */
               bcm_assert((BCMPH_PCM_CHANNEL_WIDTH == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               memcpy(dst, src0, BCMPH_PCM_CHANNEL_WIDTH);
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
               const __u8 *src1 = src0 + BCMPH_PCM_CHANNEL_WIDTH;
               bcm_assert(((BCMPH_PCM_CHANNEL_WIDTH * 2) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + (BCMPH_PCM_CHANNEL_WIDTH * 2)) <= data_len));
               for (width = 0; (width < BCMPH_PCM_CHANNEL_WIDTH); width += 1) {
                  *dst = *src0;
                  dst += 1;
                  src0 += 1;
                  *dst = *src1;
                  dst += 1;
                  src1 += 1;
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
               const __u8 *src1 = src0 + BCMPH_PCM_CHANNEL_WIDTH;
               const __u8 *src2 = data + dpl->offset_second_block_pcm_channel;
               const __u8 *src3 = src2 + BCMPH_PCM_CHANNEL_WIDTH;
               bcm_assert(((BCMPH_PCM_CHANNEL_WIDTH * 4) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + (BCMPH_PCM_CHANNEL_WIDTH * 2)) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + (BCMPH_PCM_CHANNEL_WIDTH * 2)) <= data_len));
               for (width = 0; (width < BCMPH_PCM_CHANNEL_WIDTH); width += 1) {
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
               const __u8 *src1 = data + dpl->offset_second_block_pcm_channel;
               bcm_assert(((BCMPH_PCM_CHANNEL_WIDTH * 2) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               for (width = 0; (width < BCMPH_PCM_CHANNEL_WIDTH); width += 1) {
                  *dst = *src0;
                  dst += 1;
                  src0 += 1;
                  *dst = *src1;
                  dst += 1;
                  src1 += 1;
               }
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
               bcm_assert((((BCMPH_PCM_CHANNEL_WIDTH + 1) / 2) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               for (width = 0; (width < BCMPH_PCM_CHANNEL_WIDTH); width += 2) {
                  *dst = *src0;
                  dst += 1;
                  src0 += 2;
               }
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
               memcpy(dst, src0, BCMPH_PCM_CHANNEL_WIDTH);
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
               const __u8 *src1 = data + dpl->offset_second_block_pcm_channel;
               bcm_assert(((((BCMPH_PCM_CHANNEL_WIDTH + 1) / 2) * 4) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               /* We transfer bytes and not words, because
                casting __u8 * to __u16 * can fail if __u8 * is not
                correctly aligned */
               for (width = 0; (width < BCMPH_PCM_CHANNEL_WIDTH); width += 2) {
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
               const __u8 *src1 = data + dpl->offset_second_block_pcm_channel;
               bcm_assert(((((BCMPH_PCM_CHANNEL_WIDTH + 1) / 2) * 2) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               for (width = 0; (width < BCMPH_PCM_CHANNEL_WIDTH); width += 2) {
                  *dst = *src0;
                  dst += 1;
                  src0 += 2;
                  *dst = *src1;
                  dst += 1;
                  src1 += 2;
               }
               break;
            }
#if ((defined BCMPH_TEST_PCM) && (!defined BCMPH_NOHW))
            case BCMPH_TRF_TEST_PCM: {
               bcm_assert((dpl->offset_first_block_pcm_channel + dpl->bytes_per_frame) <= data_len);
               memcpy(dst, src0, dpl->bytes_per_frame);
               break;
            }
#endif // BCMPH_TEST_PCM && !BCMPH_NOHW
            default: {
               bcm_assert(false);
               break;
            }
         }
         dpl->rx_buffer.len += dpl->bytes_per_frame;
      }
   }
}

// Enumerate all active lines to compute the minimum and maximum number of
// DMA frames that can be written
void bcm_drv_get_data_for_tx(bcm_drv_t *t, size_t *min_frame, size_t *max_frame)
{
   size_t line_idx;
   size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->core.phone_mgr));
   bool min_max_not_set = true;

   dd_bcm_pr_debug("%s()\n", __func__);

   *min_frame = 0;
   *max_frame = 0;
   for (line_idx = 0; (line_idx < phone_line_count); line_idx += 1) {
      if (bcm_phone_mgr_line_tx_use_pcm(&(t->core.phone_mgr), line_idx)) {
         size_t count = bcm_ring_buf_get_size(&(t->core.phone_lines[line_idx].tx_ring_buf)) / t->core.phone_lines[line_idx].bytes_per_frame;
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
static void bcm_drv_pcm_send_cb(pcm_t *pcm,
   __u8 *data, size_t data_len)
{
   bcm_drv_t *t = container_of(pcm, bcm_drv_t, core.pcm);
   size_t line_idx;
   size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->core.phone_mgr));

   dd_bcm_pr_debug("%s()\n", __func__);

   for (line_idx = 0; (line_idx < phone_line_count); line_idx += 1) {
      if (bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line_idx)) {
         bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line_idx]);
         const __u8 *src = &(dpl->tx_buffer.array[dpl->tx_buffer.offset]);
         __u8 *dst0 = data + dpl->offset_first_block_pcm_channel;
         size_t width;

         switch (dpl->type_transfer) {
            case BCMPH_TRF_M_8_BITS_C_xLAW: {
               /*
                A sample is just a byte.
                Data must be sent in 1st channel of the 1st PCM block
                Transfer is straightforward
               */
               bcm_assert((BCMPH_PCM_CHANNEL_WIDTH == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               memcpy(dst0, src, BCMPH_PCM_CHANNEL_WIDTH);
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
               __u8 *dst1 = dst0 + BCMPH_PCM_CHANNEL_WIDTH;
               bcm_assert(((BCMPH_PCM_CHANNEL_WIDTH * 2) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + (BCMPH_PCM_CHANNEL_WIDTH * 2)) <= data_len));
               for (width = 0; (width < BCMPH_PCM_CHANNEL_WIDTH); width += 1) {
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
               __u8 *dst1 = dst0 + BCMPH_PCM_CHANNEL_WIDTH;
               __u8 *dst2 = data + dpl->offset_second_block_pcm_channel;
               __u8 *dst3 = dst2 + BCMPH_PCM_CHANNEL_WIDTH;
               bcm_assert(((BCMPH_PCM_CHANNEL_WIDTH * 4) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + (BCMPH_PCM_CHANNEL_WIDTH * 2)) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + (BCMPH_PCM_CHANNEL_WIDTH * 2)) <= data_len));
               for (width = 0; (width < BCMPH_PCM_CHANNEL_WIDTH); width += 1) {
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
               __u8 *dst1 = data + dpl->offset_second_block_pcm_channel;
               bcm_assert(((BCMPH_PCM_CHANNEL_WIDTH * 2) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               for (width = 0; (width < BCMPH_PCM_CHANNEL_WIDTH); width += 1) {
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
               bcm_assert((((BCMPH_PCM_CHANNEL_WIDTH + 1) / 2) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               for (width = 0; (width < BCMPH_PCM_CHANNEL_WIDTH); width += 2) {
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
               memcpy(dst0, src, BCMPH_PCM_CHANNEL_WIDTH);
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
               __u8 *dst1 = data + dpl->offset_second_block_pcm_channel;
               bcm_assert(((((BCMPH_PCM_CHANNEL_WIDTH + 1) / 2) * 4) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               /* We transfer bytes and not words, because
                casting __u8 * to __u16 * can fail if __u8 * is not
                correctly aligned */
               for (width = 0; (width < BCMPH_PCM_CHANNEL_WIDTH); width += 2) {
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
               __u8 *dst1 = data + dpl->offset_second_block_pcm_channel;
               bcm_assert(((((BCMPH_PCM_CHANNEL_WIDTH + 1) / 2) * 2) == dpl->bytes_per_frame)
                  && ((dpl->offset_first_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= dpl->offset_second_block_pcm_channel)
                  && ((dpl->offset_second_block_pcm_channel + BCMPH_PCM_CHANNEL_WIDTH) <= data_len));
               for (width = 0; (width < BCMPH_PCM_CHANNEL_WIDTH); width += 2) {
                  *dst0 = *src;
                  dst0 += 2;
                  src += 1;
                  *dst1 = *src;
                  dst1 += 2;
                  src += 1;
               }
               break;
            }
#if ((defined BCMPH_TEST_PCM) && (!defined BCMPH_NOHW))
            case BCMPH_TRF_TEST_PCM: {
               bcm_assert((dpl->offset_first_block_pcm_channel + dpl->bytes_per_frame) <= data_len);
               if (
# ifdef BCMPH_EXPORT_DEV_FILE
                   (t->dev.default_line == i)
# else // !BCMPH_EXPORT_DEV_FILE
                   (BCMPH_DEFAULT_LINE == i)
# endif // !BCMPH_EXPORT_DEV_FILE
                  ) {
                  memcpy(dst0, src, dpl->bytes_per_frame);
               }
               break;
            }
#endif // BCMPH_TEST_PCM && !BCMPH_NOHW
            default: {
               bcm_assert(false);
               break;
            }
         }
         dpl->tx_buffer.offset += dpl->bytes_per_frame;
      }
   }
}

static void bcm_drv_fill_tx_buffer(bcm_drv_t *t,
   size_t line, size_t bytes_needed
#ifdef BCMPH_EXPORT_DEV_FILE
   , bool *wake_up_outq
#endif // BCMPH_EXPORT_DEV_FILE
   )
{
   __u8 *dst;
   bcm_core_drv_phone_line_t *dpl;

   bcm_assert(/* (line >= 0) && */ (line < bcm_phone_mgr_get_line_count(&(t->core.phone_mgr)))
      && (bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line))
      && (bytes_needed <= ARRAY_SIZE(dpl->tx_buffer.array)));

   dpl = &(t->core.phone_lines[line]);
   if ((dpl->tx_buffer.len + bytes_needed) > ARRAY_SIZE(dpl->tx_buffer.array)) {
      if (dpl->tx_buffer.offset >= dpl->tx_buffer.len) {
         dpl->tx_buffer.offset = 0;
         dpl->tx_buffer.len = 0;
      }
      else {
         size_t new_len = dpl->tx_buffer.len - dpl->tx_buffer.offset;
         if ((new_len + bytes_needed) > ARRAY_SIZE(dpl->tx_buffer.array)) {
            size_t diff;

            bcm_assert(false);

            diff = new_len + bytes_needed - ARRAY_SIZE(dpl->tx_buffer.array);
            dpl->tx_buffer.offset += diff;
            new_len -= diff;
         }
         memmove(dpl->tx_buffer.array, &(dpl->tx_buffer.array[dpl->tx_buffer.offset]), new_len);
         dpl->tx_buffer.offset = 0;
         dpl->tx_buffer.len = new_len;
      }
   }
   dst = &(dpl->tx_buffer.array[dpl->tx_buffer.len]);
   if (bcm_phone_mgr_line_tx_use_pcm(&(t->core.phone_mgr), line)) {
      bcm_ring_buf_t *rb = &(dpl->tx_ring_buf);
      size_t avail_in_ring_buf = bcm_ring_buf_get_size(rb);
      if (avail_in_ring_buf >= bytes_needed) {
         bcm_ring_buf_remove(rb, dst, bytes_needed);
#ifdef BCMPH_EXPORT_DEV_FILE
         *wake_up_outq = true;
#endif // BCMPH_EXPORT_DEV_FILE
      }
      else {
         if (avail_in_ring_buf > 0) {
            bcm_ring_buf_remove(rb, dst, avail_in_ring_buf);
#ifdef BCMPH_EXPORT_DEV_FILE
            *wake_up_outq = true;
#endif // BCMPH_EXPORT_DEV_FILE
         }
         dst += avail_in_ring_buf;
         memset(dst, dpl->null_byte, bytes_needed - avail_in_ring_buf);
      }
   }
   else {
      memset(dst, dpl->null_byte, bytes_needed);
   }
   dpl->tx_buffer.len += bytes_needed;
}

size_t bcm_drv_pcm_send(bcm_drv_t *t
#ifdef BCMPH_EXPORT_DEV_FILE
   , bool *wake_up_outq
#endif // BCMPH_EXPORT_DEV_FILE
)
{
   size_t ret = 0;
   size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->core.phone_mgr));
   size_t max_frames = pcm_get_max_frame_tx_buffer(&(t->core.pcm));
   // Now we check if the frame count we receive since

   dd_bcm_pr_debug("%s()\n", __func__);

   // We try to send the maximum number of frames that we can
   while (t->core.dma_frames_to_send >= max_frames) {
      size_t line_idx;
      size_t frame_count;

#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
      if (!pcm_has_tx_buffer_empty(&(t->core.pcm))) {
         break;
      }
#endif // BCMPH_ENABLE_PCM_INTERRUPTS

      // We check there is a DMA buffer that we can fill
      if (!pcm_can_send_one_buffer(&(t->core.pcm))) {
         break;
      }

      // Ensure that intermediate TX buffer has enough data to send
      // max_frames
      for (line_idx = 0; (line_idx < phone_line_count); line_idx += 1) {
         if (bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line_idx)) {
            bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line_idx]);
            size_t bytes_needed = max_frames * dpl->bytes_per_frame;
            if ((dpl->tx_buffer.offset + bytes_needed) > dpl->tx_buffer.len) {
               bcm_drv_fill_tx_buffer(t, line_idx,
                  dpl->tx_buffer.offset + bytes_needed - dpl->tx_buffer.len
#ifdef BCMPH_EXPORT_DEV_FILE
               , wake_up_outq
#endif // BCMPH_EXPORT_DEV_FILE
               );
            }
#if ((defined __KERNEL__) && (defined BCMPH_NOHW))
            if ((t->debug.open_count > 0)
# ifdef BCMPH_EXPORT_DEV_FILE
                && (t->dev.default_line == line_idx)
# else // !BCMPH_EXPORT_DEV_FILE
                && (BCMPH_DEFAULT_LINE == line_idx)
# endif // !BCMPH_EXPORT_DEV_FILE
                && (bcm_phone_mgr_line_tx_use_pcm(&(t->core.phone_mgr), line_idx))
                && (BCMPH_CODEC_LINEAR == bcm_phone_mgr_get_current_codec(&(t->core.phone_mgr), line_idx))) {
               // We copy bytes into t->debug.read_buffer
               size_t len = bcm_ring_buf_get_free_space(&(t->debug.read_ring_buf));
               if (len > bytes_needed) {
                  len = bytes_needed;
               }
               bcm_ring_buf_add(&(t->debug.read_ring_buf), &(dpl->tx_buffer.array[dpl->tx_buffer.offset]), len);
            }
#endif // __KERNEL__ && BCMPH_NOHW
         }
      }

      // pcm_send_one_buffer() can't fail as call to
      // pcm_can_send_one_buffer() returns true
      frame_count = pcm_send_one_buffer(&(t->core.pcm), bcm_drv_pcm_send_cb);
      bcm_assert(frame_count == max_frames);
      ret += frame_count;

      // We update t->core.frame_count_to_send
      t->core.dma_frames_to_send -= frame_count;
   }

   return (ret);
}

static size_t bcm_drv_pcm_receive(bcm_drv_t *t
#ifdef BCMPH_EXPORT_DEV_FILE
   , bool *wake_up_inq, bool *wake_up_outq
#endif // BCMPH_EXPORT_DEV_FILE
   )
{
   size_t ret = 0;

   dd_bcm_pr_debug("%s()\n", __func__);

#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
   if (pcm_has_rx_buffer_filled(&(t->core.pcm))) {
#endif // BCMPH_ENABLE_PCM_INTERRUPTS
      // We reclaim one (non empty) DMA buffer
      ret = pcm_receive_one_buffer(&(t->core.pcm), bcm_drv_pcm_receive_cb);
      if (ret > 0) {
         size_t line_idx;
         size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->core.phone_mgr));
         for (line_idx = 0; (line_idx < phone_line_count); line_idx += 1) {
            if (bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line_idx)) {
               bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line_idx]);
               size_t bytes_received = ret * dpl->bytes_per_frame;

               // We fill TX buffer with the number of bytes corresponding
               // to number of frames received.
               // It will be use by echo canceller.
               // If echo canceller is disabled this data will be used when
               // send a new DMA buffer
               bcm_assert(dpl->tx_buffer.offset <= dpl->tx_buffer.len);
               bcm_drv_fill_tx_buffer(t, line_idx, bytes_received
#ifdef BCMPH_EXPORT_DEV_FILE
               , wake_up_outq
#endif // BCMPH_EXPORT_DEV_FILE
               );
               bcm_assert((dpl->tx_buffer.offset <= dpl->tx_buffer.len)
                  && (dpl->tx_buffer.len >= (dpl->tx_buffer.offset + bytes_received)));

#if ((defined __KERNEL__) && (defined BCMPH_NOHW))
               if ((t->debug.open_count > 0)
# ifdef BCMPH_EXPORT_DEV_FILE
                   && (t->dev.default_line == line_idx)
# else // !BCMPH_EXPORT_DEV_FILE
                   && (BCMPH_DEFAULT_LINE == line_idx)
# endif // !BCMPH_EXPORT_DEV_FILE
                  ) {
                  if ((dpl->rx_buffer.len > 0)
                      && (BCMPH_CODEC_LINEAR == bcm_phone_mgr_get_current_codec(&(t->core.phone_mgr), line_idx))) {
                     size_t len;

                     bcm_assert(bcm_phone_mgr_line_rx_use_pcm(&(t->core.phone_mgr), line_idx));
                     /*
                       We ignore bytes in DMA buffer and get bytes
                       from t->debug.write_buffer
                       We add silence if not enought data
                     */
                     len = bcm_ring_buf_get_size(&(t->debug.write_ring_buf));
                     if (len > dpl->rx_buffer.len) {
                        len = dpl->rx_buffer.len;
                     }
                     if (len > 0) {
                        bcm_ring_buf_remove(&(t->debug.write_ring_buf), dpl->rx_buffer.array, len);
                     }
                     if (len < dpl->rx_buffer.len) {
                        memset(&(dpl->rx_buffer.array[len]), dpl->null_byte, dpl->rx_buffer.len - len);
                     }
                  }
                  else {
                     /*
                      We remove data that we don't need or not in
                      correct format
                     */
                     bcm_ring_buf_clear(&(t->debug.write_ring_buf));
                  }
               }
#endif // __KERNEL__ && BCMPH_NOHW
               if (dpl->rx_buffer.len > 0) {
                  bcm_ring_buf_t *rb = &(dpl->rx_ring_buf);
                  size_t len;

                  bcm_assert((bcm_phone_mgr_line_rx_use_pcm(&(t->core.phone_mgr), line_idx))
                     && (dpl->rx_buffer.len == bytes_received));

#ifndef BCMPH_DAHDI_DRIVER
                  // We apply echo cancellation on all bytes received not just
                  // the bytes we can add to the ring buffer because the echo
                  // canceller "learn" from the bytes transmitted.
                  // So we must submit to the echo canceller all the bytes
                  // transmitted
                  if ((dpl->cancel_echo)
                      && (bcm_phone_mgr_line_tx_use_pcm(&(t->core.phone_mgr), line_idx))) {
                     size_t sample_idx;
                     size_t sample_count = bytes_received / sizeof(__s16);
                     const __s16 *tx = (const __s16 *)(&(dpl->tx_buffer.array[dpl->tx_buffer.len - bytes_received]));
                     __s16 *rx = (__s16 *)(dpl->rx_buffer.array);
                     for (sample_idx = 0; (sample_idx < sample_count); sample_idx += 1, tx += 1, rx += 1) {
                        *rx = oslec_update(dpl->oslec_state, *tx, *rx);
                     }
                  }
#endif /* !BCMPH_DAHDI_DRIVER */

                  // We check free space in RX ring buffer
                  len = bcm_ring_buf_get_free_space(rb);

                  // And we only handle bytes that we can add in ring buffer
                  if (len > dpl->rx_buffer.len) {
                     len = dpl->rx_buffer.len;
                  }

                  if (len > 0) {
                     // We add data in ring buffer
                     bcm_ring_buf_add(rb, dpl->rx_buffer.array, len);
#ifdef BCMPH_EXPORT_DEV_FILE
                     *wake_up_inq = true;
#endif // BCMPH_EXPORT_DEV_FILE
                  }

                  // Reset len of intermediate RX buffer for next call to
                  // bcm_drv_pcm_receive()
                  dpl->rx_buffer.len = 0;
               }
               else {
                  bcm_assert(!bcm_phone_mgr_line_rx_use_pcm(&(t->core.phone_mgr), line_idx));
               }
            }
         }
      }
#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
   }
#endif // BCMPH_ENABLE_PCM_INTERRUPTS

   return (ret);
}

/*
 Avant l'ajout de l'echo canceller (oslec), les buffers DMA etaient considerees
 comme une extension des ring buffers.
 Pour l'envoi des donnees (vers le telephone) des qu'il y avait un buffer DMA
 vide, on le remplissait a partir des donnees du ring buffer a condition qu'il
 ne soit pas vide.
 Inversement pour la reception des donnees (depuis le telephone) des qu'il y
 avait un buffer DMA plein, on transferait ses donnees dans le ring buffer a
 condition qu'il y ait de la place.
 Donc si les ring buffers n'etaient pas remplis ou vides de maniere symetrique,
 les buffers DMAs n'etaient pas egalement remplis ou vides de maniere
 symetrique.

 Avec l'echo canceller la logique est modifiee.
 L'echo canceller corrige les donnees recues a partir des donnees envoyees : il
 "soustrait" des donnees recues, l'echo genere par les donnees envoyees.
 Comme on ne peut pas agir en temps reel, l'echo provient des donnees transmises
 dans le passe, avec un certain decalage donc.
 Pour que l'echo canceller soit le plus efficace possible il faut que ce
 decalage soit le plus constant possible, mais aussi le plus faible.
 Pour que le delai soit le plus faible possible, on travaille, pour les donnees
 recues des qu'un buffer DMA est rempli.
 Et pour que le delai soit constant, on ne remplit un buffer DMA, pour les
 donnees envoyees, que lorsque l'on en a recu un buffer DMA.
 La reception des buffers DMA en reception va donc nous servir de cadencement
 pour le remplissage des buffers DMA en emission.
 Cela implique que lorsque l'on traite un buffer DMA en reception s'il n'y a pas
 de place dans le ring buffer, les donnees sont perdues.
 Inversement lorsque l'on remplit un buffer DMA en emission, s'il n'y a pas de
 donnees dans le ring buffer, on genere des donnees supplementaires correspondant
 a du silence.

 Notes d'implementation :
 - le buffer rx_buffer.array sert a minimiser le nombre d'acces au ring buffer
 au prix d'une copie intermediaire.
 La taille du buffer rx_buffer.array doit etre suffisante pour traiter un
 buffer DMA.
 - le buffer tx_buffer.array sert lui aussi a minimiser le nombre d'acces au
 ring buffer, mais egalement a memoriser les silences generes lorsqu'il n'y a
 pas assez de donnees dans le ring buffer, afin que les donnees reellement
 transmises soient identiques aux donnees passees a l'echo canceller (car
 les donnees passees a l'echo canceller, ne sont pas immediatement utilisees
 dans le DMA buffer mais eventuellement avec un petit decalage).
 La taille du buffer tx_buffer.array doit etre suffisante pour traiter un
 buffer DMA en emission et un buffer DMA en reception pour l'echo canceller.
 Au cas ou l'envoi d'un buffer DMA soit en decalage par rapport a la reception,
 la taille du du buffer tx_buffer.array est la somme de la taille d'un buffer
 DMA en reception et en emission plutot que la taille max des 2 buffers.
*/

void bcm_drv_update_buffers(bcm_drv_t *t)
{
   if ((pcm_dma_is_started(&(t->core.pcm)))
#if ((!defined BCMPH_EXPORT_DEV_FILE) && (defined __KERNEL__) && (defined BCMPH_NOHW))
      // If we can get the global lock we try to :
      // - fill TX DMA buffers
      // - empty RX DMA buffers
      // BEWARE : mutex_trylock() return 1 on success and 0 if mutex can't be locked
      && (mutex_trylock(&(t->debug.lock)))
#endif /* !BCMPH_EXPORT_DEV_FILE && __KERNEL__ && BCMPH_NOHW */
       ) {
#ifdef BCMPH_EXPORT_DEV_FILE
      bool wake_up_inq = false;
      bool wake_up_outq = false;
#endif /* BCMPH_EXPORT_DEV_FILE */
      for (;;) {
         size_t frame_count = bcm_drv_pcm_receive(t
#ifdef BCMPH_EXPORT_DEV_FILE
            , &(wake_up_inq), &(wake_up_outq)
#endif /* BCMPH_EXPORT_DEV_FILE */
            );

         if (frame_count <= 0) {
            break;
         }

         t->core.dma_frames_to_send += frame_count;
         bcm_drv_pcm_send(t
#ifdef BCMPH_EXPORT_DEV_FILE
            , &(wake_up_outq)
#endif /* BCMPH_EXPORT_DEV_FILE */
            );
      }
#if ((!defined BCMPH_EXPORT_DEV_FILE) && (defined __KERNEL__) && (defined BCMPH_NOHW))
      mutex_unlock(&(t->debug.lock));
#endif /* !BCMPH_EXPORT_DEV_FILE && __KERNEL__ && BCMPH_NOHW */
#ifdef BCMPH_EXPORT_DEV_FILE
      if (wake_up_inq) {
         // Data received so we unblock processes waiting to read data
         bcm_wait_queue_wake_up(&(t->dev.inq));
      }
      if (wake_up_outq) {
         // Data received so we unblock processes waiting to read data
         bcm_wait_queue_wake_up(&(t->dev.outq));
      }
#endif /* BCMPH_EXPORT_DEV_FILE */
   }
}

static bcm_drv_t bcm_drv;

/*
 * entry point
 */
#ifdef __KERNEL__
static
#endif // __KERNEL__
int __init bcm_eval_macro_2(bcm_concat_2, BCMPH_MODULE_NAME, _init)(void)
{
   int ret = -1;
   const board_desc_t *board_desc;
   bcm_drv_t *t = &(bcm_drv);

   bcm_pr_info("%s: v" bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_VERSION) "\n", driver_name);

   bcm_pr_debug("%s()\n", __func__);

   memset(t, 0, sizeof(*t));

   ret = board_init();
   if (ret) {
      goto fail_board;
   }

   board_desc = board_get_desc();
   bcm_assert((NULL != board_desc) && (NULL != board_desc->phone_desc)
      && (board_desc->phone_desc->device_count > 0));
   t->core.board_desc = board_desc;

   /* Alloc a workqueue with at most only one work active at any time */
   t->core.wq = alloc_workqueue(KBUILD_MODNAME, WQ_UNBOUND | WQ_FREEZABLE | WQ_HIGHPRI, 1);
   if (NULL == t->core.wq) {
      ret = -ENOMEM;
      goto fail_wq;
   }

   /* Init the periodic timer that will schedule works */
   bcm_periodic_timer_init(&(t->core.periodic_timer));

   /* Alloc main buffer used for RX and TX and that can be mapped */
   {
      size_t min_buf_size;
      size_t rx_buf_len;
      size_t tx_buf_len;
      size_t offset;
#ifdef BCMPH_EXPORT_DEV_FILE
      size_t line_idx;
#endif /* BCMPH_EXPORT_DEV_FILE */

      // At a frequency of BCMPH_SAMPLES_PER_MS kHz, we have BCMPH_SAMPLES_PER_MS samples/ms
      // A sample can be up to 4 bytes (BCMPH_CODEC_LINEAR16)
      // So we need (BCMPH_SAMPLES_PER_MS * 4) bytes/ms
#ifndef BCMPH_NOHW
      // We choose a buffer size allowing storage of at least
      // (BCMPH_MIN_SIZE_RING_BUFFER * 2) rounded to a multiple of
      // t->board_desc->phone_desc->tick_period in time
      min_buf_size =
         ((((BCMPH_MIN_SIZE_RING_BUFFER * 2) + t->core.board_desc->phone_desc->tick_period  - 1)
           / t->core.board_desc->phone_desc->tick_period)
          * t->core.board_desc->phone_desc->tick_period);
#else // BCMPH_NOHW
      min_buf_size = (t->core.board_desc->phone_desc->tick_period * 2 * 32);
#endif // BCMPH_NOHW
      bcm_assert((min_buf_size >= (2 * BCMPH_PCM_CHANNEL_WIDTH)) && (RX_BUFFER_PERCENT > 0) && (RX_BUFFER_PERCENT < 100));

      rx_buf_len = ((min_buf_size * 2 * RX_BUFFER_PERCENT) / 100);
      rx_buf_len = round_up_to_pow_of_2(round_up_generic(rx_buf_len, BCMPH_RB_ROUNDING_FACTOR), BCMPH_MMAP_ALIGNMENT);
      tx_buf_len = ((min_buf_size * 2 * (100 - RX_BUFFER_PERCENT)) / 100);
      tx_buf_len = round_up_to_pow_of_2(round_up_generic(tx_buf_len, BCMPH_RB_ROUNDING_FACTOR), BCMPH_MMAP_ALIGNMENT);

      bcm_assert(ARRAY_SIZE(t->core.phone_lines) == ARRAY_SIZE(t->core.mm_rbs_location.rbs));
#ifdef BCMPH_EXPORT_DEV_FILE
      t->dev.mm_desc.ioctl_param_off = 0;
      t->dev.mm_desc.ioctl_param_size = sizeof(bcm_phone_ioctl_param_t);
      bcm_pr_debug("t->mm_desc.ioctl_param_off = %lu, t->mm_desc.ioctl_param_size = %lu\n",
         (unsigned long)(t->dev.mm_desc.ioctl_param_off), (unsigned long)(t->dev.mm_desc.ioctl_param_size));

      offset = t->dev.mm_desc.ioctl_param_off + round_up_to_pow_of_2(t->dev.mm_desc.ioctl_param_size, BCMPH_MMAP_ALIGNMENT);
      for (line_idx = 0; (line_idx < ARRAY_SIZE(t->core.mm_rbs_location.rbs)); line_idx += 1) {
         size_t size = sizeof(bcm_ring_buf_desc_t);
         t->core.mm_rbs_location.rbs[line_idx].rx_ring_buf_desc_off = offset;
         t->core.mm_rbs_location.rbs[line_idx].rx_ring_buf_desc_size = size;
         bcm_pr_debug("t->mm_desc.rbs[%lu].rx_ring_buf_desc_off = %lu, t->mm_desc.rbs[%lu].rx_ring_buf_desc_size = %lu\n",
            (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].rx_ring_buf_desc_off),
            (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].rx_ring_buf_desc_size));
         offset += round_up_to_pow_of_2(size, BCMPH_MMAP_ALIGNMENT);

         size = sizeof(bcm_ring_buf_desc_t);
         t->core.mm_rbs_location.rbs[line_idx].tx_ring_buf_desc_off = offset;
         t->core.mm_rbs_location.rbs[line_idx].tx_ring_buf_desc_size = size;
         bcm_pr_debug("t->mm_desc.rbs[%lu].tx_ring_buf_desc_off = %lu, t->mm_desc.rbs[%lu].tx_ring_buf_desc_size = %lu\n",
            (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].tx_ring_buf_desc_off),
            (unsigned long)(line_idx), (unsigned long)(t->core.mm_rbs_location.rbs[line_idx].tx_ring_buf_desc_size));
         offset += round_up_to_pow_of_2(size, BCMPH_MMAP_ALIGNMENT);
      }
#else /* !BCMPH_EXPORT_DEV_FILE */
      offset = 0;
#endif /* !BCMPH_EXPORT_DEV_FILE */
      t->core.offset_rbs = offset;
      t->core.mm_buffer_order = get_order(offset
         + ((rx_buf_len + tx_buf_len) * ARRAY_SIZE(t->core.mm_rbs_location.rbs)));
#ifdef BCMPH_EXPORT_DEV_FILE
      t->dev.mm_desc.mmap_size = (PAGE_SIZE << t->core.mm_buffer_order);
      t->core.size_rbs = t->dev.mm_desc.mmap_size;
#else /* !BCMPH_EXPORT_DEV_FILE */
      t->core.size_rbs = (PAGE_SIZE << t->core.mm_buffer_order);
#endif /* !BCMPH_EXPORT_DEV_FILE */
      t->core.size_rbs -= t->core.offset_rbs;

      // Allocate the buffer
      // Do not omit flag __GFP_COMP, else calls to vm_insert_page() in
      // bcm_drv_mmap() will fail
      // We don't use alloc_pages_exact because when we read kernel sources
      // we see that the function calls split_page() that doesn't expect the
      // flag __GFP_COMP
      t->core.mm_buffer = (__u8 *)(__get_free_pages(GFP_KERNEL | __GFP_ZERO | __GFP_NORETRY | __GFP_COMP, t->core.mm_buffer_order));
      if (NULL == t->core.mm_buffer) {
         bcm_pr_err("Cannot allocate mmap buffer (%lu bytes) used for RX, TX...\n",
            (unsigned long)(PAGE_SIZE << t->core.mm_buffer_order));
         ret = -ENOMEM;
         goto fail_buf;
      }
      bcm_pr_debug("%lu bytes of memory allocated, starting at address 0x%lx\n", (unsigned long)(PAGE_SIZE << t->core.mm_buffer_order), (unsigned long)(t->core.mm_buffer));
   }

   ret = bcm_phone_mgr_init(&(t->core.phone_mgr), t->core.board_desc,
      &(t->core.periodic_timer), t->core.wq, bcm_drv_timer_cb);
   if (ret) {
      goto fail_phones;
   }

   bcm_assert(ARRAY_SIZE(t->core.phone_lines) >= bcm_phone_mgr_get_line_count(&(t->core.phone_mgr)));

   ret = bcm_drv_init(t);
   if (ret) {
      goto fail_drv;
   }

#if ((defined __KERNEL__) && (defined BCMPH_NOHW))
   ret = bcm_debug_init(t);
   if (ret) {
      goto fail_debug;
   }
#endif // __KERNEL__ && BCMPH_NOHW

   ret = pcm_init(&(t->core.pcm)
#ifdef __KERNEL__
# ifdef BCMPH_EXPORT_DEV_FILE
      , t->dev.misc_dev.this_device
# endif /* BCMPH_EXPORT_DEV_FILE */
# ifdef BCMPH_DAHDI_DRIVER
      , &(t->dahdi.d_dev->dev)
# endif /* BCMPH_DAHDI_DRIVER */
#endif // __KERNEL__
      , t->core.board_desc, bcm_drv_resume_read_cb, bcm_drv_resume_write_cb);
   if (ret) {
      goto fail_pcm;
   }

#ifdef BCMPH_EXPORT_DEV_FILE
   // Now we can set open_count to 0 to allow open() to success,
   // as we complete successfully the init of the driver
   t->dev.open_count = 0;

   bcm_periodic_timer_start(&(t->core.periodic_timer), max((unsigned int)(t->core.board_desc->phone_desc->tick_period) * 1000U, jiffies_to_usecs(1U)));
#endif /* BCMPH_EXPORT_DEV_FILE */
#ifdef BCMPH_DAHDI_DRIVER
   bcm_assert(TIMER_PERIOD_US < (t->core.board_desc->phone_desc->tick_period * 1000U));
   bcm_periodic_timer_start(&(t->core.periodic_timer), max((unsigned int)(TIMER_PERIOD_US), jiffies_to_usecs(1U)));

   ret = bcm_dahdi_start(t);
   if (ret) {
      goto fail_dahdi;
   }
#endif /* BCMPH_DAHDI_DRIVER */

#if ((defined __KERNEL__) && (defined BCMPH_NOHW))
   // Now we can set open_count to 0 to allow open() to success,
   // as we complete successfully the init of the driver
   t->debug.open_count = 0;
#endif /* KERNEL && BCMPH_NOHW */

   bcm_pr_debug("%s successfully initialized\n", driver_name);

   return (0);

#ifdef BCMPH_DAHDI_DRIVER
   bcm_dahdi_stop(t);
fail_dahdi:
#endif /* BCMPH_DAHDI_DRIVER */
   bcm_periodic_timer_stop(&(t->core.periodic_timer));
   pcm_deinit(&(t->core.pcm));
fail_pcm:
#if ((defined __KERNEL__) && (defined BCMPH_NOHW))
   bcm_debug_exit(t);
fail_debug:
#endif // __KERNEL__ && BCMPH_NOHW
   bcm_drv_exit(t);
fail_drv:
   bcm_phone_mgr_deinit(&(t->core.phone_mgr));
fail_phones:
   bcm_pr_debug("Freeing mmap buffer used for RX, TX...\n");
   free_pages((unsigned long)(t->core.mm_buffer), t->core.mm_buffer_order);
   t->core.mm_buffer = NULL;
fail_buf:
   bcm_periodic_timer_deinit(&(t->core.periodic_timer));
   flush_workqueue(t->core.wq);
   destroy_workqueue(t->core.wq);
   t->core.wq = NULL;
fail_wq:
   board_deinit();
fail_board:
   return (ret);
}

#ifdef __KERNEL__
static
#endif // __KERNEL__
void __exit bcm_eval_macro_2(bcm_concat_2, BCMPH_MODULE_NAME, _exit)(void)
{
   bcm_drv_t *t = &(bcm_drv);

   bcm_pr_debug("%s()\n", __func__);

#ifdef BCMPH_DAHDI_DRIVER
   bcm_dahdi_stop(t);
#endif /* BCMPH_DAHDI_DRIVER */
   bcm_drv_stop(t, false);

   bcm_periodic_timer_stop(&(t->core.periodic_timer));
   pcm_deinit(&(t->core.pcm));
#if ((defined __KERNEL__) && (defined BCMPH_NOHW))
   bcm_debug_exit(t);
#endif // __KERNEL__ && BCMPH_NOHW
   bcm_drv_exit(t);
   bcm_phone_mgr_deinit(&(t->core.phone_mgr));
   bcm_pr_debug("Freeing mmap buffer used for RX, TX...\n");
   free_pages((unsigned long)(t->core.mm_buffer), t->core.mm_buffer_order);
   t->core.mm_buffer = NULL;
   bcm_periodic_timer_deinit(&(t->core.periodic_timer));
   flush_workqueue(t->core.wq);
   destroy_workqueue(t->core.wq);
   t->core.wq = NULL;
   board_deinit();
   bcm_pr_debug("%s removed\n", driver_name);
}

#ifdef __KERNEL__
bcm_eval_macro_1(module_init, bcm_eval_macro_2(bcm_concat_2, BCMPH_MODULE_NAME, _init));
bcm_eval_macro_1(module_exit, bcm_eval_macro_2(bcm_concat_2, BCMPH_MODULE_NAME, _exit));

bcm_eval_macro_1(MODULE_DESCRIPTION, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " driver");
MODULE_AUTHOR("Gilles Mazoyer <mazoyer.gilles@omega.ovh>");
MODULE_LICENSE("GPL");
bcm_eval_macro_1(MODULE_VERSION, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_VERSION));
#endif // __KERNEL__
