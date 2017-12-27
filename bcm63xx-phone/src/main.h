/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __MAIN_H__
#define __MAIN_H__

#include <config.h>

#include <extern/linux/file.h>
#include <extern/linux/mutex.h>
#include <extern/linux/spinlock.h>
#include <extern/linux/wait.h>
#include <extern/linux/workqueue.h>
#ifndef BCMPH_DAHDI_DRIVER
# include <extern/oslec.h>
#endif /* BCMPH_DAHDI_DRIVER */
#ifdef __KERNEL__
#include <linux/cdev.h>
#include <linux/miscdevice.h>
#endif // __KERNEL__

#ifdef BCMPH_DAHDI_DRIVER
# include <dahdi/kernel.h>
#endif /* BCMPH_DAHDI_DRIVER */

#include <bcm63xx_phone.h>
#include <bcm63xx_ring_buf.h>

#include <board.h>
#include <pcm.h>
#include <phone_mgr.h>
#include <wait_queue.h>

// We can have more lines than we have PCM channels
#if (BCMPH_MAX_LINES > BCMPH_PCM_MAX_CHANNELS)
# error BCMPH_MAX_LINES must be <= than BCMPH_PCM_MAX_CHANNELS
#endif

// In a PCM frame a line can use up to 4 channels : we round
// the ring buffer size to (4 * BCMPH_PCM_CHANNEL_WIDTH) */
#ifndef BCMPH_TEST_PCM
# define BCMPH_RB_ROUNDING_FACTOR (4 * BCMPH_PCM_CHANNEL_WIDTH)
#else // BCMPH_TEST_PCM
# define BCMPH_RB_ROUNDING_FACTOR BCMPH_PCM_MAX_FRAME_SIZE
#endif // BCMPH_TEST_PCM

// The alignment used for structures in the mmap_buffer
#define BCMPH_MMAP_ALIGNMENT 8

#define BCMPH_DEFAULT_LINE 0

#ifdef BCMPH_DAHDI_DRIVER
# define DAHDI_TICK_PERIOD_US 1000
/*
 DADHI holds data in internal buffers and each time we call
 dahdi_tranmit/receive, DAHDI transfers data from internal buffers to/from
 chan->writechunk/readchunk
 So we can schedule the DAHDI work not every millisecond, but less frequenty
 as far as we call dahdi_receive/transmit for each millisecond ellapsed since
 the last execution of the work.
 If our span is the master span of course it's a not a very good timing source,
 but it's not our problem. If others spans needs a more precise timing source
 they can provide their own timing source.
*/
# define TIMER_PERIOD_US ((DAHDI_TICK_PERIOD_US * 5) / 2)
#endif /* BCMPH_DAHDI_DRIVER */

typedef enum {
      BCMPH_TRF_NO_OP,
      BCMPH_TRF_M_8_BITS_C_xLAW,
      BCMPH_TRF_M_8_BITS_C_LINEAR,
      BCMPH_TRF_M_8_BITS_C_LINEAR16,
      BCMPH_TRF_M_8_BITS_C_xLAW16,
      BCMPH_TRF_M_16_BITS_C_xLAW,
      BCMPH_TRF_M_16_BITS_C_LINEAR,
      BCMPH_TRF_M_16_BITS_C_LINEAR16,
      BCMPH_TRF_M_16_BITS_C_xLAW16,
#if ((defined BCMPH_TEST_PCM) && (!defined BCMPH_NOHW))
      BCMPH_TRF_TEST_PCM
#endif // BCMPH_TEST_PCM && !BCMPH_NOHW
} bcm_drv_phone_line_type_transfer_t;

typedef struct {
   // Receive buffer (contained in mm_buffer)
   bcm_ring_buf_t rx_ring_buf;
   // Transmit buffer (contained in mm_buffer)
   bcm_ring_buf_t tx_ring_buf;
   // The number of bytes needed to fill a DMA frame
   size_t bytes_per_frame;
   // Value to use to fill DMA frame with logical 0
   // With slin it's 0, with xlaw or ulaw it depends
   __u8 null_byte;
   // PCM channels are reserved by blocks (see bcm_drv_start())
   // The number of adjacent PCM channels in the block depends upon
   // the codec and the PCM mode (8 bits or 16 bits)
   // The first block is use for narrowband and wideband mode
   // The second block is use for wideband mode
   // The offset in bytes in the DMA frame of the first block of PCM
   // channels carrying data for the line
   size_t offset_first_block_pcm_channel;
   // The offset in bytes in the DMA frame of the second block of PCM
   // channels carrying data for the line
   size_t offset_second_block_pcm_channel;
   // Enum that tells the type of transfer between ring buffer and DMA frame
   bcm_drv_phone_line_type_transfer_t type_transfer;
   // Intermediate buffer used to transfer from DMA buffer to ring buffer
   // The idea is to minimize the number of accesses to ring buffer,
   // We ensure that the buffer is __s16 aligned so that it can be
   // passed as parameters to the echo canceller as __s16 array
   struct {
      union {
         __s16 dummy;
         __u8 array[(BCMPH_PCM_CHANNEL_WIDTH * 4) * BCMPH_PCM_MAX_FRAME_RX_BUFFER];
      };
      size_t len;
   } rx_buffer;
   // Intermediate buffer used to transfer to DMA buffer from ring buffer
   // The idea is to minimize the number of accesses to ring buffer,
   // and also to store silence generated if no data available in
   // ring buffer.
   // We ensure that the buffer is __s16 aligned so that it can be
   // passed as parameters to the echo canceller as __s16 array
   struct {
      union {
         __s16 dummy;
         __u8 array[(BCMPH_PCM_CHANNEL_WIDTH * 4) * (BCMPH_PCM_MAX_FRAME_TX_BUFFER + BCMPH_PCM_MAX_FRAME_RX_BUFFER)];
      };
      // This is the offset in transmit buffer of the data used for
      // filling the next DMA buffer
      size_t offset;
      size_t len;
   } tx_buffer;
#ifndef BCMPH_DAHDI_DRIVER
   bool cancel_echo;
   struct oslec_state *oslec_state;
#endif /* !BCMPH_DAHDI_DRIVER */
} bcm_core_drv_phone_line_t;

typedef struct {
   // Buffer mmaped and the order used to allocate it
   __u8 *mm_buffer;
   size_t mm_buffer_order;
   struct workqueue_struct *wq;
   /*
    We use two distinct works for DAHDI and phone mgr
    task, so that DAHDI work can be started and stopped independently of
    phone mgr.
    But we use only a single timer to spare some resource.
    The timer function will only schedule the works when needed.
   */
   bcm_periodic_timer_t periodic_timer;
   // Offset from start of mm_buffer and size reserved for
   // TX and RX ring buffers of the lines
   size_t offset_rbs;
   size_t size_rbs;
   bcm_phone_get_mmap_rbs_location_t mm_rbs_location;
   // Board description
   const board_desc_t *board_desc;
   // Data passed to all pcm functions
   pcm_t pcm;
   // Data passed to all bcm_phone_mgr functions
   bcm_phone_mgr_t phone_mgr;

   bcm_core_drv_phone_line_t phone_lines[BCMPH_MAX_LINES];
   // The number of DMA frames we can send
   // Used to strictly send the same number of DMA frames that we receive
   // It is incremented when we handle a RX buffer
   // And decremented when we fill a TX buffer
   size_t dma_frames_to_send;
} bcm_core_drv_t;

#ifdef BCMPH_EXPORT_DEV_FILE
typedef struct {
#ifdef __KERNEL__
   // The misc device registered to the kernel
   struct miscdevice misc_dev;
#endif // __KERNEL__
   // Description of the buffer
   bcm_phone_get_mmap_desc_t mm_desc;
   // Counter how many times the device is open.
   int open_count;
   // The default line for read, write and poll
   size_t default_line;
   // Mutex that control access, used in file operations and in
   // the timer work
   // Even if we accept driver to be opened only once, mutex
   // is better than an atomic because it makes the caller sleep
   // if not available
   // From linux documentation, mutex is better than semaphore
   // to provide mutual exclusion
   struct mutex lock;
   // Wait queue used to block processes waiting for reading data
   bcm_wait_queue_t inq;
   // Wait queue used to block processes waiting for writing data
   bcm_wait_queue_t outq;
} bcm_dev_drv_t;
#endif /* BCMPH_EXPORT_DEV_FILE */

#ifdef BCMPH_DAHDI_DRIVER

#ifdef BCMPH_DEBUG
#define DYN_STATS_PERIOD_US 1000000 /* 1 second */
#define MAX_COUNT_DYN_STATS ((2 * 60 * 1000000) / DYN_STATS_PERIOD_US) /* 2 minutes */
#endif // BCMPH_DEBUG

typedef struct {
   struct dahdi_chan d_chan;

   spinlock_t lock;

   struct device *dev;

   bool force_update_line;

   // If positive and line is on hook and not ringing, line mode must be
   // BCMPH_MODE_ON_TALKING instead of BCMPH_MODE_ON_IDLE
   int onhook_transfer_tick_count;

   // Last DAHDI signal received from DAHDI
   enum dahdi_txsig txsig;
   // Flag set to signal that we have must handle change of txsig
   // and also set when onhook_transfer_tick_count is initialized
   bool txsig_pending;

   // Last DAHDI signal transmitted to DAHDI
   enum dahdi_rxsig rxsig;
   // Flag set to signal that we must handle change of rxsig (ie we must call
   // dahdi_hooksig)
   bool rxsig_pending;

   struct {
      struct dahdi_vmwi_info vmwi_info;
      bool message_waiting;
      bool reverse_polarity;
      bool computed_rev_polarity;
      bool previous_rev_polarity;
   } fxs_data;

   bool pcm_started;

#ifdef SHORTCUT_TX
   size_t idx_demo_thanks;
#endif /* SHORTCUT_TX */

#ifdef BCMPH_DEBUG
   struct {
      unsigned long bytes_rx_from_dahdi;
      unsigned long bytes_tx_to_dahdi;
      unsigned int err_rx_ring_buf_empty;
      unsigned int err_tx_ring_buf_full;
   } stats;
#endif /* BCMPH_DEBUG */
} bcm_dahdi_drv_phone_line_t;

typedef struct {
   struct dahdi_device *d_dev;
   struct dahdi_span d_span;
   struct dahdi_chan *d_chans[BCMPH_MAX_LINES];
   bcm_dahdi_drv_phone_line_t lines[BCMPH_MAX_LINES];
   size_t line_count;
   bcm_periodic_work_t periodic_work;
#ifdef BCMPH_DEBUG
   struct {
      unsigned long calls_to_make_dahdi_work;
      unsigned long calls_to_dahdi_tick;
   } stats;
#endif /* BCMPH_DEBUG */
} bcm_dahdi_drv_t;
#endif /* BCMPH_DAHDI_DRIVER */

#if ((defined __KERNEL__) && (defined BCMPH_NOHW))
typedef struct {
   // Buffer used for ring buffers
   __u8 *buf;
   // Ring buffer filled by user program
   bcm_ring_buf_t write_ring_buf;
   // Ring buffer read by user program
   bcm_ring_buf_t read_ring_buf;
   // The misc device registered to the kernel
   struct miscdevice misc_dev;
      // Counter how many times the device is open.
   int open_count;
# ifndef BCMPH_EXPORT_DEV_FILE
   // Mutex that control access, used in file operations and in
   // the timer work
   struct mutex lock;
# endif // BCMPH_EXPORT_DEV_FILE
#ifdef BCMPH_DEBUG
   struct {
      unsigned long bytes_rx_from_user;
      unsigned long bytes_tx_to_user;
   } stats;
#endif // BCMPH_DEBUG
} bcm_debug_t;
#endif /* __KERNEL__ && BCMPH_NOHW */

typedef struct {
   bcm_core_drv_t core;
#ifdef BCMPH_EXPORT_DEV_FILE
   bcm_dev_drv_t dev;
#endif
#ifdef BCMPH_DAHDI_DRIVER
   bcm_dahdi_drv_t dahdi;
#endif /* BCMPH_DAHDI_DRIVER */
#if ((defined __KERNEL__) && (defined BCMPH_NOHW))
   bcm_debug_t debug;
#endif /* __KERNEL__ && BCMPH_NOHW */
} bcm_drv_t;

extern void bcm_drv_get_data_for_tx(bcm_drv_t *t, size_t *min_frame, size_t *max_frame);

extern size_t bcm_drv_pcm_send(bcm_drv_t *t
#ifdef BCMPH_EXPORT_DEV_FILE
   , bool *wake_up_outq
#endif // BCMPH_EXPORT_DEV_FILE
);

extern void bcm_drv_forget_rx_data(bcm_drv_t *t, size_t line);

extern void bcm_drv_forget_tx_data(bcm_drv_t *t, size_t line);

extern void bcm_drv_update_buffers(bcm_drv_t *t);

extern int bcm_drv_get_line_states(bcm_drv_t *t,
   bcm_phone_get_line_states_t *get_line_states
#ifdef BCMPH_EXPORT_DEV_FILE
   , bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   );

#ifdef BCMPH_TEST_PCM
extern int bcm_drv_set_line_state(bcm_drv_t *t,
   const bcm_phone_set_line_state_t *set_line_state);
#endif // BCMPH_TEST_PCM

extern int bcm_drv_set_line_mode(bcm_drv_t *t, size_t line,
   bcm_phone_line_mode_t mode
#ifndef BCMPH_DAHDI_DRIVER
   , int echo_cancellation
#endif /* !BCMPH_DAHDI_DRIVER */
   , int reverse_polarity, __u32 tone
#ifdef BCMPH_EXPORT_DEV_FILE
   , int wait, bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   );

extern int bcm_drv_set_line_tone(bcm_drv_t *t, size_t line, __u32 tone
#ifdef BCMPH_EXPORT_DEV_FILE
   , int wait, bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   );

extern int bcm_drv_set_line_codec(bcm_drv_t *t, size_t line,
   bcm_phone_codec_t codec, bcm_phone_line_mode_t mode
#ifndef BCMPH_DAHDI_DRIVER
   , int echo_cancellation
#endif /* !BCMPH_DAHDI_DRIVER */
   , int reverse_polarity, __u32 tone
#ifdef BCMPH_EXPORT_DEV_FILE
   , bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   );

extern int bcm_drv_set_line_rev_polarity(bcm_drv_t *t, size_t line,
   bool rev_polarity
#ifdef BCMPH_EXPORT_DEV_FILE
   , int wait, bcmph_mutex_t *lock
#endif /* BCMPH_EXPORT_DEV_FILE */
   );

extern void bcm_drv_start_pcm(bcm_drv_t *t);

extern void bcm_drv_stop_pcm(bcm_drv_t *t, bool wait);

static inline void bcm_drv_read_version(bcm_drv_t *t, bcm_phone_drv_ver_t *ver)
{
   memset(ver, 0, sizeof(*ver));
   ver->major = 0;
   ver->minor = 1;
}

extern int bcm_drv_start(bcm_drv_t *t, const bcm_phone_cfg_params_t *params);

extern void bcm_drv_stop(bcm_drv_t *t, bool wait);

extern void bcm_drv_resume_read_cb(pcm_t *pcm);

extern void bcm_drv_resume_write_cb(pcm_t *pcm);

extern int bcm_drv_init(bcm_drv_t *t);

extern void bcm_drv_exit(bcm_drv_t *t);

#ifdef BCMPH_DAHDI_DRIVER

extern int bcm_dahdi_start(bcm_drv_t *t);

extern void bcm_dahdi_stop(bcm_drv_t *t);

# ifdef BCMPH_DEBUG
extern void bcm_dahdi_pr_stats(bcm_drv_t *t);
# endif /* BCMPH_DEBUG */

#endif /* BCMPH_DAHDI_DRIVER */

#ifndef __KERNEL__

extern int bcm63xx_phone_init(void);

extern void bcm63xx_phone_exit(void);

#ifdef BCMPH_EXPORT_DEV_FILE

extern int bcm_drv_open(struct inode *inode, struct file *filp);

extern int bcm_drv_release(struct inode *inode, struct file *filp);

extern ssize_t bcm_drv_read(struct file *filp, char *buf, size_t count,
            loff_t *offset);

extern ssize_t bcm_drv_write(struct file *filp, const char *buf,
            size_t count, loff_t *offset);

extern long bcm_drv_unlocked_ioctl(struct file *filp,
            unsigned int cmd, unsigned long arg);

#endif /* BCMPH_EXPORT_DEV_FILE */

#else // __KERNEL__

# ifdef BCMPH_NOHW

extern int bcm_debug_init(bcm_drv_t *t);

extern void bcm_debug_exit(bcm_drv_t *t);

# endif // BCMPH_NOHW

#endif // __KERNEL__

// For the memory allocated to the ring buffers of a line,
// define the percentages dedicated to the RX ring buffer and implicitly
// for the TX ring buffer
#define RX_BUFFER_PERCENT 50

#endif // __MAIN_H__
