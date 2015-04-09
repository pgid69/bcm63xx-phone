/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@gilande.com>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifndef __MAIN_H__
#define __MAIN_H__

#include <config.h>

#ifdef __KERNEL__
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#endif // __KERNEL__

#include <bcm63xx_phone.h>
#include <bcm63xx_ring_buf.h>

#include <board.h>
#include <pcm.h>
#include <phone_mgr.h>
#include <wait_queue.h>

// We can have more lines than we have PCM channels
#if (BCMPH_MAX_LINES > BCMPH_PCM_MAX_CHANNELS)
#error BCMPH_MAX_LINES must be <= than BCMPH_PCM_MAX_CHANNELS
#endif

// In a PCM frame a line can use up to 4 channels : we round
// the ring buffer size to (4 * BCMPH_PCM_CHANNEL_WIDTH) */
#ifndef BCMPH_TEST_PCM
#define BCMPH_RB_ROUNDING_FACTOR (4 * BCMPH_PCM_CHANNEL_WIDTH)
#else // BCMPH_TEST_PCM
#define BCMPH_RB_ROUNDING_FACTOR (BCMPH_PCM_MAX_CHANNELS * BCMPH_PCM_CHANNEL_WIDTH)
#endif // BCMPH_TEST_PCM

// The alignment used for structures in the mmap_buffer
#define BCMPH_MMAP_ALIGNMENT 8

typedef struct {
   // Index of device into phone_devices[]
   size_t index_dev;
   // Index of line
   size_t index_line;
   // Receive buffer (contained in mm_buffer)
   bcm_ring_buf_t rx_ring_buf;
   // Transmit buffer (contained in mm_buffer)
   bcm_ring_buf_t tx_ring_buf;
   // The number of bytes needed to fill a DMA frame
   size_t bytes_per_frame;
   // The offset in bytes in the DMA frame of the first PCM channel
   // carrying data for the line
   // The number of adjacent PCM channels used depends upon the codec
   // and the PCM mode (8 bits or 16 bits)
   size_t offset_first_pcm_channel;
   // Enum that tells the type of transfer between ring buffer and DMA frame
   enum {
      BCMPH_TRF_NO_OP,
      BCMPH_TRF_M_8_BITS_C_xLAW,
      BCMPH_TRF_M_8_BITS_C_LINEAR,
      BCMPH_TRF_M_8_BITS_C_LINEAR16,
      BCMPH_TRF_M_16_BITS_C_xLAW,
      BCMPH_TRF_M_16_BITS_C_LINEAR,
      BCMPH_TRF_M_16_BITS_C_LINEAR16,
#ifdef BCMPH_TEST_PCM
      BCMPH_TRF_TEST_PCM
#endif // BCMPH_TEST_PCM
   } type_transfer;
} bcm_drv_phone_line_t;

typedef struct {
#ifdef __KERNEL__
   // The misc device registered to the kernel
   struct miscdevice misc_dev;
#endif // __KERNEL__
   // Counter how many times the device is open.
   int open_count;
   // Mutex that control access, used in file operations and in
   // the timer work
   // Even if we accept driver to be opened only once, mutex
   // is better than an atomic because it makes the caller sleep
   // if not available
   // From linux documentation, mutex is better than semaphore
   // to provide mutual exclusion
   struct mutex lock;
   // Buffer mmaped and the order used to allocate it
   __u8 *mm_buffer;
   size_t mm_buffer_order;
   // Description of the buffer
   bcm_phone_get_mmap_desc_t mm_desc;
   // Offset from start of mm_buffer and size reserved for
   // TX and RX ring buffers of the lines
   size_t offset_rbs;
   size_t size_rbs;
   bcm_phone_get_mmap_rbs_location_t mm_rbs_location;
   // Wait queue used to block processes waiting for reading data
   bcm_wait_queue_t inq;
   // Wait queue used to block processes waiting for writing data
   bcm_wait_queue_t outq;
   // Board description
   const board_desc_t *board_desc;
   // Data passed to all pcm functions
   pcm_t pcm;
   // Data passed to all bcm_phone_mgr functions
   bcm_phone_mgr_t phone_mgr;

   bcm_drv_phone_line_t phone_lines[BCMPH_MAX_LINES];
   // The default line for read, write and poll
   size_t default_line;
} bcm_drv_t;

extern void bcm_drv_fill_rx_buffers(bcm_drv_t *t);

extern void bcm_drv_get_data_for_tx(bcm_drv_t *t, size_t *min_frame, size_t *max_frame);

extern void bcm_drv_empty_tx_buffers(bcm_drv_t *t, bool fsync);

static inline  void bcm_drv_update_buffers(bcm_drv_t *t)
{
   bcm_drv_fill_rx_buffers(t);
   bcm_drv_empty_tx_buffers(t, false);
}

extern ssize_t bcm_drv_read_line(bcm_drv_t *t,
   bool do_not_block, bcmph_mutex_t *lock,
   size_t line, __u8 *buf, size_t count);

extern ssize_t bcm_drv_write_line(bcm_drv_t *t,
   bool do_not_block, bcmph_mutex_t *lock,
   size_t line, const __u8 *buf, size_t count);

extern void bcm_drv_stop(bcm_drv_t *t, int wait);

extern long bcm_drv_unlocked_ioctl(struct file *filp,
             unsigned int cmd, unsigned long arg);

#ifndef __KERNEL__

extern int bcm_drv_init(void);

extern void bcm_drv_exit(void);

extern int bcm_drv_open(struct inode *inode, struct file *filp);

extern int bcm_drv_release(struct inode *inode, struct file *filp);

extern ssize_t bcm_drv_read(struct file *filp, char *buf, size_t count,
           loff_t *offset);

extern ssize_t bcm_drv_write(struct file *filp, const char *buf,
            size_t count, loff_t *offset);
#endif // __KERNEL__

// For the memory allocated to the ring buffers of a line,
// define the percentages dedicated to the RX ring buffer and implicitly
// for the TX ring buffer
#define RX_BUFFER_PERCENT 50

#endif // __MAIN_H__
