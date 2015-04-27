/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@gilande.com>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifndef __PCM_H__
#define __PCM_H__

#include "config.h"

#ifdef __KERNEL__
#include <linux/bitops.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#endif // __KERNEL__

#include "bcm63xx_phone.h"
#include "bcm63xx_ring_buf.h"
#include "board.h"

/* Number of PCM channels.
 From what i understand a PCM channel is used to read or write
 a single timeslot of a frame of the PCM bus.
 However registers of the PCM module does not have an array of 8 entries
 containing a timeslot number, but an array of 128 entries containing a PCM
 channel number. Perhaps it's more efficient from the hardware perspective. */
#define BCMPH_PCM_MAX_CHANNELS   8
/* Width in bytes of a PCM channel : depending of the codec used it contains
 2 (BCMPH_CODEC_LINEAR16 or BCMPH_CODEC_LINEAR) or
 4 samples (BCMPH_CODEC_ALAW, BCMPH_CODEC_ALAW16, BCMPH_CODEC_ULAW or
 BCMPH_CODEC_ULAW16 if PCM set in 8 bits mode) */
#define BCMPH_PCM_CHANNEL_WIDTH  4

/* The number of bytes of a frame transferred to or from the PCM module by DMA
 Here a frame does not refer to the frame of the PCM bus (which is composed of
 128 timeslots of 8 bits if PCM bus is clocked at 8192 MHz and signal FS is
 clocked at 8 kHz) but to the frame transferred to or from the PCM module by DMA
 (which is composed of BCMPH_PCM_MAX_CHANNELS PCM channels of
  BCMPH_PCM_CHANNEL_WIDTH bytes wide) */
#define PCM_MAX_FRAME_SIZE (BCMPH_PCM_MAX_CHANNELS * BCMPH_PCM_CHANNEL_WIDTH)

#define BIT_NR_RX_DESC_RETURNED 0
#define BIT_NR_TX_DESC_EMPTY 1

/*
 * rx/tx dma descriptor
 */
typedef struct {
   __u32 len_stat;
   dma_addr_t address;
#ifdef BCMPH_NOHW
   unsigned long jiffies;
#endif // BCMPH_NOHW
} pcm_dma_desc_t;

typedef struct pcm {
#ifdef __KERNEL__
   /* Linux device */
   struct device *dev;
#endif // __KERNEL__

   const board_desc_t *board_desc;

#ifndef BCMPH_NOHW
   void __iomem *base;
   void __iomem *dma_base;
   void __iomem *dmac_base;
   void __iomem *dmas_base;

#ifdef BCMPH_TEST_PCM
   int irq;
#endif // BCMPH_TEST_PCM

#endif // !BCMPH_NOHW

   /* maximum dma burst size */
   size_t dma_maxburst;

#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
   /* RX dma irq */
   int irq_dma_rx;
#endif // BCMPH_ENABLE_PCM_INTERRUPTS

   /* dma channel id for RX */
   int rx_chan;

   /* number of dma desc in RX ring */
   size_t rx_ring_size;

   /* size of a single RX buffer */
   size_t rx_data_buffer_size;

   /* size of a single RX buffer rounded to max_frame_size */
   size_t rx_data_buffer_rounded_size;

   /* allocated size (in bytes) for RX desc ring */
   size_t rx_desc_alloc_size;

#ifndef BCMPH_NOHW
   /* hw view of RX dma desc ring */
   dma_addr_t rx_dma_desc;
#endif // !BCMPH_NOHW

   /* cpu view of RX dma desc ring */
   pcm_dma_desc_t *rx_cpu_desc;

   /* dma addresses of RX data ring buffers */
   dma_addr_t *rx_dma_addr;

   /* cpu view of RX data ring */
   __u8 *rx_cpu_data;

   /* number of available descriptors for RX owned by DMA
    (ie descriptors waiting to be filled by DMA)
    (buf_desc->rx_ring_size - rx_desc_cnt_owned) == number of descriptors
     that can be given to DMA to be filled) */
   size_t rx_desc_cnt_owned;

   /* index of next RX descriptor to fetch from DMA (if rx_desc_cnt_owned > 0) */
   size_t rx_curr_desc;

   /* index of next RX descriptor to give to DMA (if rx_desc_cnt_owned < rx_ring_size) */
   size_t rx_dirty_desc;

   /* callback called when bit BIT_NR_RX_DESC_RETURNED is set to 1.
    Can be called from interrupt */
   void (*rx_desc_returned_cb)(struct pcm *t);

#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
   /* TX dma irq */
   int irq_dma_tx;
#endif // BCMPH_ENABLE_PCM_INTERRUPTS

   /* dma channel id for TX */
   int tx_chan;

   /* number of dma desc in TX ring */
   __u32 tx_ring_size;

   /* size of a single TX buffer */
   __u32 tx_data_buffer_size;

   /* size of a single TX buffer rounded to max_frame_size */
   size_t tx_data_buffer_rounded_size;

   /* allocated size (in bytes) for TX dma ring */
   size_t tx_desc_alloc_size;

#ifndef BCMPH_NOHW
   /* hw view of TX dma ring */
   dma_addr_t tx_dma_desc;
#endif // !BCMPH_NOHW

   /* cpu view of TX dma ring */
   pcm_dma_desc_t *tx_cpu_desc;

   /* dma addresses of TX data ring buffers */
   dma_addr_t *tx_dma_addr;

   /* cpu view of TX data ring */
   __u8 *tx_cpu_data;

   /* number of available descriptors for TX owned by DMA
    (ie descriptors filled waiting to be transmitted by DMA)
    (buf_desc->tx_ring_size - tx_desc_cnt_owned) == number of descriptors
     that can be filled) */
   size_t tx_desc_cnt_owned;

   /* number of TX descriptors empty */
   size_t tx_desc_cnt_empty;

   /* index of next TX descriptor that can be filled (if tx_desc_cnt_empty > 0) */
   size_t tx_curr_desc;

   /* index of next dirty TX descriptor to reclaim to DMA (if tx_desc_cnt_owned > 0) */
   size_t tx_dirty_desc;

   /* callback called when bit BIT_NR_TX_DESC_EMPTY is set to 1
    Can be called from interrupt */
   void (*tx_desc_empty_cb)(struct pcm *t);

#ifdef BCMPH_NOHW
   unsigned long prev_jiffies;
#endif // BCMPH_NOHW

   /* bit fields */
   unsigned long bits;

   size_t max_frame_size;

   /* Tell how many bytes we can read or write in each frame
    * It depends upon the number of active channels
    > 0 when PCM is started, == 0 when PCM is stopped  */
   size_t frame_size;

   // Tell if a timeslot is 8 or 16 bits wide
   bool timeslot_is_16bits;

   // Tell the timeslot read by each channel
   __u8 channel_to_timeslot[BCMPH_PCM_MAX_CHANNELS];

   // Tell if PCM module is running
   bool is_started;

   // Tell if DMA is started
   bool dma_is_started;

   bcm_phone_pcm_stats_t stats;
} pcm_t;

extern int pcm6358_pll_init(const pcm_t *t);

extern void pcm6358_pll_deinit(const pcm_t *t);

extern int pcm6368_pll_init(const pcm_t *t);

extern void pcm6368_pll_deinit(const pcm_t *t);

extern int pcm_init(
   pcm_t *t,
#ifdef __KERNEL__
   struct device *dev,
#endif // __KERNEL__
   const board_desc_t *board_desc,
   void (*rx_desc_returned_cb)(pcm_t *t),
   void (*tx_desc_empty_cb)(pcm_t *t));

extern void pcm_deinit(pcm_t *t);

#define BCMPH_TIMESLOT_UNSPECIFIED 0xFF

extern void pcm_configure_channels(pcm_t *t,
   bool use_16bits_mode,
   const __u8 *chans_to_ts, size_t chans_to_ts_len);

static inline bool pcm_timeslot_is_16bits(const pcm_t *t)
{
   bool ret = t->timeslot_is_16bits;
   dd_bcm_pr_debug("pcm_timeslot_is_16bits() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

extern __u8 pcm_get_channnel_used_for_timeslot(const pcm_t *t, __u8 timeslot);

extern void pcm_start(pcm_t *t);

extern void pcm_stop(pcm_t *t);

extern void pcm_dma_start(pcm_t *t);

extern void pcm_dma_stop(pcm_t *t);

extern void pcm_read_regs(const pcm_t *t, bcm_phone_pcm_regs_t *regs);

extern void pcm_read_stats(const pcm_t *t, bcm_phone_pcm_stats_t *regs);

// Return the number of timeslot contained in a frame (depends on the PCM clock frequency)
static inline size_t pcm_get_max_timeslots(const pcm_t *t)
{
   size_t ret = t->board_desc->phone_desc->clk_rate / 64;
   dd_bcm_pr_debug("pcm_get_max_frame_size() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

// Return the number of bytes of a frame (it depends on the number
// of channels allocated to a timeslot)
static inline size_t pcm_get_frame_size(const pcm_t *t)
{
   size_t ret = t->frame_size;
   dd_bcm_pr_debug("pcm_get_frame_size() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

static inline bool pcm_is_started(const pcm_t *t)
{
   bool ret = t->is_started;
   dd_bcm_pr_debug("pcm_is_started() -> %d\n", (int)(ret));
   return (ret);
}

static inline bool pcm_dma_is_started(const pcm_t *t)
{
   bool ret = t->dma_is_started;
   dd_bcm_pr_debug("pcm_dma_is_started() -> %d\n", (int)(ret));
   return (ret);
}

static inline int pcm_has_rx_buffer_filled(pcm_t *t)
{
   int ret = test_bit(BIT_NR_RX_DESC_RETURNED, &(t->bits));
   dd_bcm_pr_debug("pcm_has_rx_buffer_filled() -> %d\n", (int)(ret));
   return (ret);
}

// Returns size of RX data buffer in bytes
static inline size_t pcm_get_max_phys_size_rx_buffer(pcm_t *t)
{
   size_t ret = t->rx_data_buffer_rounded_size;
   dd_bcm_pr_debug("pcm_get_max_phys_size_rx_buffer() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

static inline size_t pcm_get_max_frame_rx_buffer(pcm_t *t)
{
   size_t ret = (pcm_get_max_phys_size_rx_buffer(t) / t->max_frame_size);
   dd_bcm_pr_debug("pcm_get_max_frame_rx_buffer() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

// Returns maximum number of bytes that can be copied from a RX buffer
// (that depends on the number of active channels)
static inline size_t pcm_get_max_log_size_rx_buffer(pcm_t *t)
{
   size_t ret = pcm_get_max_frame_rx_buffer(t) * pcm_get_frame_size(t);
   dd_bcm_pr_debug("pcm_get_max_log_size_rx_buffer() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

// Returns time in microseconds used by PCM module to transmit a DMA frame
static inline unsigned long pcm_get_time_to_transmit_frame(pcm_t *t)
{
   /* A sample is transmitted every 125 microseconds. A sample is 1 or 2 bytes long */
   unsigned long ret = BCMPH_PCM_CHANNEL_WIDTH * 125;
   if (pcm_timeslot_is_16bits(t)) {
      ret /= 2;
   }
   dd_bcm_pr_debug("pcm_get_time_to_transmit_frame() -> %lu\n", (unsigned long)(ret));
   return (ret);
}


// Returns time in microseconds used by PCM module to fill one RX buffer
static inline unsigned long pcm_get_time_to_receive_rx_buffer(pcm_t *t)
{
   unsigned long ret = pcm_get_time_to_transmit_frame(t) * pcm_get_max_frame_rx_buffer(t);
   dd_bcm_pr_debug("pcm_get_time_to_receive_rx_buffer() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

// Returns number of RX buffers currently owned by DMA
extern size_t pcm_get_cnt_rx_buffer_really_owned(pcm_t *t);

extern size_t pcm_rx_reclaim(pcm_t *t,
   void (*receive_cb)(pcm_t *t, const __u8 *data, size_t data_len),
   size_t buffer_count, size_t max_frame_count);

extern size_t pcm_receive(pcm_t *t,
   void (*receive_cb)(pcm_t *t, const __u8 *data, size_t data_len),
   size_t frame_count);

static inline int pcm_has_tx_buffer_empty(pcm_t *t)
{
   int ret = test_bit(BIT_NR_TX_DESC_EMPTY, &(t->bits));
   dd_bcm_pr_debug("pcm_has_tx_buffer_empty() -> %d\n", (int)(ret));
   return (ret);
}

// Returns size of TX data buffer in bytes
static inline size_t pcm_get_phys_size_tx_buffer(pcm_t *t)
{
   size_t ret = t->tx_data_buffer_rounded_size;
   dd_bcm_pr_debug("pcm_get_phys_size_tx_buffer() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

static inline size_t pcm_get_max_frame_tx_buffer(pcm_t *t)
{
   size_t ret = (pcm_get_phys_size_tx_buffer(t) / t->max_frame_size);
   dd_bcm_pr_debug("pcm_get_max_frame_tx_buffer() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

// Returns time in microseconds used by PCM module to send one TX buffer
static inline unsigned long pcm_get_time_to_send_tx_buffer(pcm_t *t)
{
   unsigned long ret = pcm_get_time_to_transmit_frame(t) * pcm_get_max_frame_tx_buffer(t);
   dd_bcm_pr_debug("pcm_get_time_to_send_tx_buffer() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

// Returns number of bytes that can be copied in a TX buffer
// (that depends on the number of active channels)
static inline size_t pcm_get_log_size_tx_buffer(pcm_t *t)
{
   size_t ret = pcm_get_max_frame_tx_buffer(t) * pcm_get_frame_size(t);
   dd_bcm_pr_debug("pcm_get_log_size_tx_buffer() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

extern size_t pcm_tx_reclaim(pcm_t *t, size_t buffer_count);

extern size_t pcm_send(pcm_t *t,
   bool (*send_cb)(pcm_t *t, __u8 *data, size_t data_len),
   size_t frame_count);

// Returns number of TX buffers currently owned by DMA
static inline size_t pcm_get_cnt_tx_buffer_really_owned(pcm_t *t)
{
   size_t ret;

   pcm_tx_reclaim(t, t->tx_desc_cnt_owned);
   ret = t->tx_desc_cnt_owned;
   dd_bcm_pr_debug("pcm_get_cnt_tx_buffer_really_owned() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

#endif // __PCM_H__
