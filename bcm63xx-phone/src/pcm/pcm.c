/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <config.h>

#ifdef __KERNEL__
# ifdef BCMPH_ENABLE_PCM_INTERRUPTS
#  include <linux/interrupt.h>
# endif // BCMPH_ENABLE_PCM_INTERRUPTS
# include <linux/ioport.h>
#endif // __KERNEL__
#include <extern/linux/barrier.h>
#include <extern/linux/delay.h>
#include <extern/linux/errno.h>
#ifdef BCMPH_NOHW
# include <extern/linux/jiffies.h>
#endif // BCMPH_NOHW
#include <extern/linux/kernel.h>
#include <extern/linux/sched.h>
#include <extern/linux/slab.h>
#include <extern/linux/stddef.h>
#include <extern/linux/uaccess.h>

#include <bcm63xx.h>
#include <bcm63xx_log.h>
#include <pcm.h>
#include <utils.h>

// Include after system files
#include <compile.h>

// Il y a un probleme avec le fait que les octets sont transferes par blocs de 8
// channels : que se passe-t'il si on ne recoit pas le bon nombre d'octets
// comment se synchroniser : si on avait la notion de paquets ca serait plus
// facile tandis que la !!!

#ifndef BCMPH_NOHW

typedef struct {
   resource_size_t start;
   resource_size_t end;
   void __iomem *base;
} pcm_mem_res_t;

static struct {
   pcm_mem_res_t pcm_mem;
   pcm_mem_res_t pcm_dma_mem;
   pcm_mem_res_t pcm_dmac_mem;
   pcm_mem_res_t pcm_dmas_mem;
} pcm_shared_res = {
   .pcm_mem = {
      .start   = -1, /* filled at runtime */
      .end     = -1, /* filled at runtime */
      .base    = NULL
   },
   .pcm_dma_mem = {
      .start   = -1, /* filled at runtime */
      .end     = -1, /* filled at runtime */
      .base    = NULL
   },
   .pcm_dmac_mem = {
      .start   = -1, /* filled at runtime */
      .end     = -1, /* filled at runtime */
      .base    = NULL
   },
   .pcm_dmas_mem = {
      .start   = -1, /* filled at runtime */
      .end     = -1, /* filled at runtime */
      .base    = NULL
   }
};

// Number of reserved bytes between DMA data buffers
#define RSV_DATA_BUFFER_LEN 0

#else // BCMPH_NOHW

#define RSV_DATA_BUFFER_LEN 16

static __u32 io_mem[256];

#endif // BCMPH_NOHW

/*
 * io helpers to access shared registers
 */
static inline __u32 pcm_readl(const pcm_t *t, __u32 off)
{
   __u32 ret;
#ifndef BCMPH_NOHW
   ret = bcm_readl(t->base + off);
#else
   ret = io_mem[off >> 2];
#endif // BCMPH_NOHW
   dd_bcm_pr_debug("%s(off=0x%lx) -> 0x%lx\n", __func__, (unsigned long)(off), (unsigned long)(ret));
   return (ret);
}

static inline void pcm_writel(const pcm_t *t, __u32 val, __u32 off)
{
   dd_bcm_pr_debug("%s(val=0x%lx, off=0x%lx)\n", __func__, (unsigned long)(val), (unsigned long)(off));
#ifndef BCMPH_NOHW
   bcm_writel(val, t->base + off);
#else
   io_mem[off >> 2] = val;
#endif // BCMPH_NOHW
}

static inline __u32 pcm_dma_readl(const pcm_t *t, __u32 off)
{
   __u32 ret;
#ifndef BCMPH_NOHW
   ret = bcm_readl(t->dma_base + off);
#else
   ret = io_mem[(off + 256) >> 2];
#endif // BCMPH_NOHW
   dd_bcm_pr_debug("%s(off=0x%lx) -> 0x%lx\n", __func__, (unsigned long)(off), (unsigned long)(ret));
   return (ret);
}

static inline void pcm_dma_writel(const pcm_t *t, __u32 val, __u32 off)
{
   dd_bcm_pr_debug("%s(val=0x%lx, off=0x%lx)\n", __func__, (unsigned long)(val), (unsigned long)(off));
#ifndef BCMPH_NOHW
   bcm_writel(val, t->dma_base + off);
#else
   io_mem[(off + 256) >> 2] = val;
#endif // BCMPH_NOHW
}

static inline __u32 pcm_dmac_readl(const pcm_t *t, __u32 off)
{
   __u32 ret;
#ifndef BCMPH_NOHW
   ret = bcm_readl(t->dmac_base + off);
#else
   ret = io_mem[(off + 512) >> 2];
#endif // BCMPH_NOHW
   dd_bcm_pr_debug("%s(off=0x%lx) -> 0x%lx\n", __func__, (unsigned long)(off), (unsigned long)(ret));
   return (ret);
}

static inline void pcm_dmac_writel(const pcm_t *t, __u32 val, __u32 off)
{
   dd_bcm_pr_debug("%s(val=0x%lx, off=0x%lx)\n", __func__, (unsigned long)(val), (unsigned long)(off));
#ifndef BCMPH_NOHW
   bcm_writel(val, t->dmac_base + off);
#else
   io_mem[(off + 512) >> 2] = val;
#endif // BCMPH_NOHW
}

static inline __u32 pcm_dmas_readl(const pcm_t *t, __u32 off)
{
   __u32 ret;
#ifndef BCMPH_NOHW
   ret = bcm_readl(t->dmas_base + off);
#else
   ret = io_mem[(off + 768) >> 2];
#endif // BCMPH_NOHW
   dd_bcm_pr_debug("%s(off=0x%lx) -> 0x%lx\n", __func__, (unsigned long)(off), (unsigned long)(ret));
   return (ret);
}

static inline void pcm_dmas_writel(const pcm_t *t, __u32 val, __u32 off)
{
   dd_bcm_pr_debug("%s(val=0x%lx, 0x%lx)\n", __func__, (unsigned long)(val), (unsigned long)(off));
#ifndef BCMPH_NOHW
   bcm_writel(val, t->dmas_base + off);
#else
   io_mem[(off + 768) >> 2] = val;
#endif // BCMPH_NOHW
}

// Do NOT call when pcm dma is stopped
static void pcm_give_one_rx_buf_to_dma(pcm_t *t)
{
   __u32 len_stat;
   pcm_dma_desc_t *desc;

   dd_bcm_pr_debug("%s() : %lu\n", __func__, (unsigned long)(t->rx_dirty_desc));

   bcm_assert(t->rx_desc_cnt_owned < t->rx_ring_size);

   desc = &(t->rx_cpu_desc[t->rx_dirty_desc]);

   /*
    We must call rmb() (barrier() is not enough because it's not a
    hardware barrier), to make sure we actually read the descriptor
    fields at each loop
   */
   rmb();

   desc->address = t->rx_dma_addr[t->rx_dirty_desc];
   len_stat = ((t->rx_data_buffer_rounded_size << DMADESC_LENGTH_SHIFT) | DMADESC_OWNER_MASK);

   t->rx_dirty_desc += 1;
   if (t->rx_dirty_desc >= t->rx_ring_size) {
      len_stat |= DMADESC_WRAP_MASK;
      t->rx_dirty_desc = 0;
   }
   t->rx_desc_cnt_owned += 1;

   desc->len_stat = len_stat;

   /* Make sure len_stat is written to memory before we tell DMA
    engine there's a new buffer */
   wmb();

#if 0
   /* Tell DMA engine we allocated one buffer */
   pcm_dma_writel(t, 1, PCMDMA_BUFALLOC_REG(t->rx_chan));
#endif

   /* Kick RX DMA */
   pcm_dmac_writel(t, PCMDMAC_CHANCFG_EN_MASK, PCMDMAC_CHANCFG_REG(t->rx_chan));
}

static inline void pcm_rx_curr_buffer_handled(pcm_t *t)
{
   dd_bcm_pr_debug("%s(index=%lu)\n", __func__, (unsigned long)(t->rx_curr_desc));

   bcm_assert((!t->dma_is_started)
      || ((t->rx_curr_desc == t->rx_dirty_desc)
          && (t->rx_desc_cnt_owned == t->rx_ring_size)));

   t->rx_curr_desc += 1;
   if (t->rx_curr_desc >= t->rx_ring_size) {
      t->rx_curr_desc = 0;
   }
   t->rx_desc_cnt_owned -= 1;
   if (t->dma_is_started) {
      pcm_give_one_rx_buf_to_dma(t);
   }
}

#ifdef BCMPH_NOHW
static void pcm_loopback_buffers(pcm_t *t)
{
   size_t desc_idx_rx;
   size_t desc_idx_tx;
   pcm_dma_desc_t *desc_rx;
   pcm_dma_desc_t *desc_tx;
   size_t i;
   size_t buffer_count;
   unsigned long now;

   dd_bcm_pr_debug("%s()\n", __func__);

   do { /* Empty loop */
      if (!t->dma_is_started) {
         break;
      }

      // Look for a TX buffer still owned
      i = t->tx_desc_cnt_owned;
      desc_idx_tx = t->tx_dirty_desc;
      while (i > 0) {
         desc_tx = &(t->tx_cpu_desc[desc_idx_tx]);
         if (desc_tx->len_stat & DMADESC_OWNER_MASK) {
            break;
         }
         desc_idx_tx += 1;
         if (desc_idx_tx >= t->tx_ring_size) {
            bcm_assert((desc_tx->len_stat & DMADESC_WRAP_MASK) == DMADESC_WRAP_MASK);
            desc_idx_tx = 0;
         }
         else {
            bcm_assert((desc_tx->len_stat & DMADESC_WRAP_MASK) == 0);
         }
         i -= 1;
      }
      if (i <= 0) {
         dd_bcm_pr_debug("No TX buffer owned by DMA, so there's nothing to copy from\n");
         break;
      }
      buffer_count = i;

      // Look for a RX buffer still owned
      i = t->rx_desc_cnt_owned;
      desc_idx_rx = t->rx_curr_desc;
      while (i > 0) {
         desc_rx = &(t->rx_cpu_desc[desc_idx_rx]);
         if (desc_rx->len_stat & DMADESC_OWNER_MASK) {
            break;
         }
         desc_idx_rx += 1;
         if (desc_idx_rx >= t->rx_ring_size) {
            bcm_assert((desc_rx->len_stat & DMADESC_WRAP_MASK) == DMADESC_WRAP_MASK);
            desc_idx_rx = 0;
         }
         else {
            bcm_assert((desc_rx->len_stat & DMADESC_WRAP_MASK) == 0);
         }
         i -= 1;
      }
      if (i <= 0) {
         dd_bcm_pr_debug("No RX buffer owned by DMA, so there's nothing to copy into\n");
         break;
      }
      if (buffer_count > i) {
         buffer_count = i;
      }

      now = get_jiffies();
      i = 0;
      while (i < buffer_count) {
         size_t len;
         unsigned long v;

         desc_rx = &(t->rx_cpu_desc[desc_idx_rx]);
         if (!(desc_rx->len_stat & DMADESC_OWNER_MASK)) {
            bcm_pr_err("RX desc %lu not in correct state\n", (unsigned long)(desc_idx_rx));
         }

         desc_tx = &(t->tx_cpu_desc[desc_idx_tx]);
         if (!(desc_tx->len_stat & DMADESC_OWNER_MASK)) {
            bcm_pr_err("TX desc %lu not in correct state\n", (unsigned long)(desc_idx_tx));
         }

         v = desc_tx->jiffies;
         if (time_after(v, now)) {
            // The buffer can't be copied to RX ring
            dd_bcm_pr_debug("The TX buffer can't be copied to RX ring (too young)\n");
            break;
         }

         // Transfer the buffer in RX
         len = ((desc_tx->len_stat & DMADESC_LENGTH_MASK) >> DMADESC_LENGTH_SHIFT);
         if (len > t->rx_data_buffer_rounded_size) {
            len = t->rx_data_buffer_rounded_size;
         }
         memcpy(&(t->rx_cpu_data[desc_idx_rx * (t->rx_data_buffer_size + RSV_DATA_BUFFER_LEN)]), &(t->tx_cpu_data[desc_idx_tx * (t->tx_data_buffer_size + RSV_DATA_BUFFER_LEN)]), len);

         dd_bcm_pr_debug("TX buffer %lu copied to RX buffer %lu\n",
            (unsigned long)(desc_idx_tx), (unsigned long)(desc_idx_rx));

         // Clear the bit DMADESC_OWNER_MASK in TX desc
         desc_tx->len_stat &= (~(DMADESC_OWNER_MASK));
         // Clear all bits but DMADESC_WRAP_MASK in RX desc and set length and jiffies
         desc_rx->len_stat &= (DMADESC_WRAP_MASK);
         desc_rx->len_stat |= (len << DMADESC_LENGTH_SHIFT);
         desc_rx->jiffies = desc_tx->jiffies;

         desc_idx_rx += 1;
         if (desc_idx_rx >= t->rx_ring_size) {
            bcm_assert((desc_rx->len_stat & DMADESC_WRAP_MASK) == DMADESC_WRAP_MASK);
            desc_idx_rx = 0;
         }
         else {
            bcm_assert((desc_rx->len_stat & DMADESC_WRAP_MASK) == 0);
         }

         desc_idx_tx += 1;
         if (desc_idx_tx >= t->tx_ring_size) {
            bcm_assert((desc_tx->len_stat & DMADESC_WRAP_MASK) == DMADESC_WRAP_MASK);
            desc_idx_tx = 0;
         }
         else {
            bcm_assert((desc_tx->len_stat & DMADESC_WRAP_MASK) == 0);
         }

         i += 1;
      }
   } while (0);
}
#endif // BCMPH_NOHW

size_t pcm_get_cnt_rx_buffer_really_owned(pcm_t *t)
{
   size_t ret;
   size_t curr_desc;

   dd_bcm_pr_debug("%s()\n", __func__);

#ifdef BCMPH_NOHW
   pcm_loopback_buffers(t);
#endif // BCMPH_NOHW

   ret = t->rx_desc_cnt_owned;
   curr_desc = t->rx_curr_desc;
   while (ret > 0) {
      __u32 len_stat;
      pcm_dma_desc_t *desc;

      dd_bcm_pr_debug("Testing RX buffer %lu\n", (unsigned long)(curr_desc));

      desc = &(t->rx_cpu_desc[curr_desc]);

      /*
       We must call rmb() (barrier() is not enough because it's not a
       hardware barrier), to make sure we actually read the descriptor
       fields at each loop
      */
      rmb();

      len_stat = desc->len_stat;

      /* break if dma ownership still belongs to hw */
      if (len_stat & DMADESC_OWNER_MASK) {
         break;
      }

      curr_desc += 1;
      if (curr_desc >= t->rx_ring_size) {
         curr_desc = 0;
      }
      ret -= 1;
   }

   dd_bcm_pr_debug("%lu RX buffers really owned\n", (unsigned long)(ret));

   return (ret);
}

static size_t pcm_get_size_next_valid_rx_buffer(pcm_t *t, size_t *buffer_count)
{
   size_t ret = 0;

   dd_bcm_pr_debug("%s()\n", __func__);

#ifdef BCMPH_NOHW
   pcm_loopback_buffers(t);
#endif // BCMPH_NOHW

   while ((t->rx_desc_cnt_owned > 0) && (*buffer_count > 0)) {
      int old;
      __u32 len_stat;
      pcm_dma_desc_t *desc;

      dd_bcm_pr_debug("Testing RX buffer %lu\n", (unsigned long)(t->rx_curr_desc));

      desc = &(t->rx_cpu_desc[t->rx_curr_desc]);

      /*
       We must call rmb() (barrier() is not enough because it's not a
       hardware barrier), to make sure we actually read the descriptor
       fields at each loop
      */
      rmb();

      len_stat = desc->len_stat;

      /* break if dma ownership still belongs to hw */
      if (len_stat & DMADESC_OWNER_MASK) {
         old = test_and_clear_bit(BIT_NR_RX_DESC_RETURNED, &(t->bits));
         if (old) {
            dd_bcm_pr_debug("Flag 'f_rx_desc_returned' set to false\n");
#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
            if (t->dma_is_started) {
               __u32 reg;
               // Ack RX interrupt
               reg = pcm_dmac_readl(t, PCMDMAC_IR_REG(t->rx_chan));
               pcm_dmac_writel(t, reg, PCMDMAC_IR_REG(t->rx_chan));
               // Enable RX interrupt
               pcm_dmac_writel(t, PCMDMAC_IR_BUFDONE_MASK | PCMDMAC_IR_PKTDONE_MASK, PCMDMAC_IRMASK_REG(t->rx_chan));
            }
#endif // BCMPH_ENABLE_PCM_INTERRUPTS
         }
         break;
      }

      old = test_and_set_bit(BIT_NR_RX_DESC_RETURNED, &(t->bits));
      if (!old) {
         dd_bcm_pr_debug("Running RX callback\n");
         (*(t->rx_desc_returned_cb))(t);
      }

      ret = ((len_stat & DMADESC_LENGTH_MASK) >> DMADESC_LENGTH_SHIFT);
      if (unlikely(ret <= 0)) {
         // Empty buffer, so we recycle it
         bcm_pr_debug("RX buffer %lu is empty\n", (unsigned long)(t->rx_curr_desc));
         pcm_rx_curr_buffer_handled(t);
         *buffer_count -= 1;
         t->stats.rx_errors += 1;
         t->stats.rx_empty_errors += 1;
      }
      else {
         if (unlikely(0 != (ret % t->max_frame_size))) {
            // Buffer length is not a multiple of t->max_frame_size, so we recycle it
            bcm_pr_debug("RX buffer %lu has invalid length %lu\n", (unsigned long)(t->rx_curr_desc), (unsigned long)(ret));
            pcm_rx_curr_buffer_handled(t);
            *buffer_count -= 1;
            t->stats.rx_errors += 1;
            t->stats.rx_length_errors += 1;
         }
         else {
            break;
         }
      }
   }

   dd_bcm_pr_debug("Next valid RX buffer contains %lu bytes\n", (unsigned long)(ret));

   return (ret);
}

static inline void pcm_rx_handle_curr_buffer(pcm_t *t, size_t frame_count,
   void (*receive_cb)(pcm_t *t, const __u8 *data, size_t data_len))
{
#ifdef BCMPH_TEST_PCM
   pcm_dma_desc_t *desc;
#endif // !BCMPH_TEST_PCM
#ifndef BCMPH_NOHW
   dma_addr_t address;
#endif // !BCMPH_NOHW
   const __u8 *src;
   size_t frame;
   size_t len = (frame_count * t->max_frame_size);

   dd_bcm_pr_debug("%s(frame_count=%lu)\n", __func__,
      (unsigned long)(frame_count));

#ifdef BCMPH_TEST_PCM
   desc = &(t->rx_cpu_desc[t->rx_curr_desc]);
   /*
    We must call rmb() (barrier() is not enough because it's not a
    hardware barrier), to make sure we actually read the descriptor
    fields at each loop
   */
   rmb();

   if (desc->len_stat & DMADESC_SOP_MASK) {
      t->stats.rx_sop += 1;
   }
   if (desc->len_stat & DMADESC_EOP_MASK) {
      t->stats.rx_eop += 1;
   }
   if (desc->len_stat & (~(DMADESC_LENGTH_MASK | DMADESC_SOP_MASK | DMADESC_EOP_MASK | DMADESC_WRAP_MASK))) {
      bcm_pr_debug("Weird len_stat %lx\n", (unsigned long)(desc->len_stat));
   }
#endif // !BCMPH_TEST_PCM

   /* copy data */
#ifndef BCMPH_NOHW
   address = t->rx_dma_addr[t->rx_curr_desc];
   dma_sync_single_for_cpu(t->dev, address, t->rx_data_buffer_size, DMA_FROM_DEVICE);
#endif // !BCMPH_NOHW
   src = &(t->rx_cpu_data[t->rx_curr_desc * (t->rx_data_buffer_size + RSV_DATA_BUFFER_LEN)]);
   for (frame = 0; (frame < frame_count); frame += 1) {
      (*(receive_cb))(t, src, t->frame_size);
      src += t->max_frame_size;
   }
#ifndef BCMPH_NOHW
   dma_sync_single_for_device(t->dev, address, t->rx_data_buffer_size, DMA_FROM_DEVICE);
#endif // !BCMPH_NOHW

#ifdef BCMPH_TEST_PCM
   if (len < t->stats.min_size_rx_buffer) {
      t->stats.min_size_rx_buffer = len;
   }
   if (len > t->stats.max_size_rx_buffer) {
      t->stats.max_size_rx_buffer = len;
   }
#endif // BCMPH_TEST_PCM

   /* Make sure data is copied before we give buffer to DMA, because
    * DMA is polling */
   rmb();

   pcm_rx_curr_buffer_handled(t);
   t->stats.rx_good += 1;
   t->stats.rx_bytes += len;
}

/*
 * Handle at most buffers_count buffers of data from rx queue.
 * For each frame of a buffer, call receive_cb to store data
 * We stop if we exceed frame_count
 * Returns the number of frames read (that can be greater than frame_count)
 */
size_t pcm_rx_reclaim(pcm_t *t,
   void (*receive_cb)(pcm_t *t, const __u8 *data, size_t data_len),
   size_t buffer_count, size_t frame_count)
{
   size_t frames_not_copied = frame_count;

   dd_bcm_pr_debug("%s(frame_count=%lu)\n", __func__,
     (unsigned long)(frame_count));

   d_bcm_assert(NULL != receive_cb);

   // Testing rx_desc_cnt_owned is useless because done in pcm_get_size_next_valid_rx_buffer()
   while (frame_count > 0) {
      size_t len;
      size_t buf_frame_count;

      len = pcm_get_size_next_valid_rx_buffer(t, &(buffer_count));
      if (0 == len) {
         // No data
         break;
      }
      buf_frame_count = (len / t->max_frame_size);
      pcm_rx_handle_curr_buffer(t, buf_frame_count, receive_cb);
      if (buf_frame_count > frame_count) {
         // We have extracted more frame than requested
         frames_not_copied += (buf_frame_count - frame_count);
         frame_count = 0;
         break;
      }
      frame_count -= buf_frame_count;
   }

   return (frames_not_copied - frame_count);
}

/*
 * Handle one non empty buffer of data from rx queue.
 * For each frame of a buffer, call receive_cb to store data
 * Returns the number of frames read in the buffer
 */
size_t pcm_receive_one_buffer(pcm_t *t,
   void (*receive_cb)(pcm_t *t, const __u8 *data, size_t data_len))
{
   size_t ret;
   size_t buffer_count;
   size_t len;

   dd_bcm_pr_debug("%s()\n", __func__);

   d_bcm_assert(NULL != receive_cb);

   // Testing rx_desc_cnt_owned is useless because done in pcm_get_size_next_valid_rx_buffer()
   buffer_count = t->rx_ring_size;
   len = pcm_get_size_next_valid_rx_buffer(t, &(buffer_count));
   if (0 == len) {
      ret = 0;
   }
   else {
      ret = (len / t->max_frame_size);
      pcm_rx_handle_curr_buffer(t, ret, receive_cb);
   }

   return (ret);
}

/*
 * Handle at most frame_count frames of data from rx queue.
 * For each frame of a buffer, call receive_cb to store data
 * Returns the number of frames read
 */
size_t pcm_receive(pcm_t *t,
   void (*receive_cb)(pcm_t *t, const __u8 *data, size_t data_len),
   size_t frame_count)
{
   size_t buffer_count;
   size_t frames_not_copied = frame_count;

   dd_bcm_pr_debug("%s(frame_count=%lu)\n", __func__,
     (unsigned long)(frame_count));

   d_bcm_assert(NULL != receive_cb);

   buffer_count = t->rx_ring_size;
   // Testing rx_desc_cnt_owned is useless because done in pcm_get_size_next_valid_rx_buffer()
   while (frame_count > 0) {
      size_t len;
      size_t buf_frame_count;

      len = pcm_get_size_next_valid_rx_buffer(t, &(buffer_count));
      if (0 == len) {
         // No data
         break;
      }
      buf_frame_count = (len / t->max_frame_size);
      if (buf_frame_count > frame_count) {
         // Not enough space to empty the buffer
         break;
      }
      pcm_rx_handle_curr_buffer(t, buf_frame_count, receive_cb);
      frame_count -= buf_frame_count;
   }

   return (frames_not_copied - frame_count);
}


#ifdef BCMPH_NOHW
static inline void pcm_set_desc_jiffies(pcm_t *t, pcm_dma_desc_t *desc)
{
   unsigned long now = get_jiffies();

   bcm_period_inc(&(t->timestamp));
   bcm_period_inc_to_jiffies(&(t->timestamp), now);
   desc->jiffies = t->timestamp.expires_in_jiffies;
}
#endif // BCMPH_NOHW


// Do NOT call when pcm dma is stopped
static void pcm_refill_tx(pcm_t *t)
{
   size_t count;

   dd_bcm_pr_debug("%s()\n", __func__);

   // We compute the number of buffer filled that we can give to DMA
   bcm_assert(t->tx_ring_size >= (t->tx_desc_cnt_owned + t->tx_desc_cnt_empty));
   count = t->tx_ring_size - (t->tx_desc_cnt_owned + t->tx_desc_cnt_empty);
   if (count > 0) {
      size_t index;

      // We compute the index of the first buffer not owned by DMA
      index = t->tx_dirty_desc + t->tx_desc_cnt_owned;
      if (index >= t->tx_ring_size) {
         index -= t->tx_ring_size;
      }
      // We compute the number of desc owned at the end of the loop
      count += t->tx_desc_cnt_owned;
      do {
         __u32 len_stat;
         pcm_dma_desc_t *desc;

         bcm_assert(index != t->tx_curr_desc);

         dd_bcm_pr_debug("Give TX buffer %lu to DMA\n", (unsigned long)(index));

         desc = &(t->tx_cpu_desc[index]);

         /*
          We must call rmb() (barrier() is not enough because it's not a
          hardware barrier), to make sure we actually read the descriptor
          fields at each loop
         */
         rmb();

         len_stat = desc->len_stat;
         len_stat |= DMADESC_OWNER_MASK;

#ifdef BCMPH_NOHW
         pcm_set_desc_jiffies(t, desc);
#endif // BCMPH_NOHW
         desc->len_stat = len_stat;

         /* Make sure len_stat is written to memory before we tell DMA
          engine there's a new buffer */
         wmb();

         index += 1;
         if (index >= t->tx_ring_size) {
            index = 0;
         }
         t->tx_desc_cnt_owned += 1;

         /* Kick TX DMA */
         pcm_dmac_writel(t, PCMDMAC_CHANCFG_EN_MASK, PCMDMAC_CHANCFG_REG(t->tx_chan));
      }
      while (t->tx_desc_cnt_owned < count);

      bcm_assert((index == t->tx_curr_desc) && (t->tx_desc_cnt_owned + t->tx_desc_cnt_empty) == t->tx_ring_size);
   }
}

/*
 * try to reclaim transmitted buffers and returns the number of buffer reclaimed
 */
size_t pcm_tx_reclaim(pcm_t *t, size_t buffer_count)
{
   size_t released = 0;

   dd_bcm_pr_debug("%s(buffer_count=%lu)\n", __func__,
     (unsigned long)(buffer_count));

#ifdef BCMPH_NOHW
   pcm_loopback_buffers(t);
#endif // BCMPH_NOHW

   while (released < buffer_count) {
      int old;
      pcm_dma_desc_t *desc;
      __u32 len_stat;

      if (t->tx_desc_cnt_owned <= 0) {
         /* No descriptor to reclaim because none owned by DMA */
         break;
      }
      bcm_assert((t->tx_desc_cnt_owned + t->tx_desc_cnt_empty) <= t->tx_ring_size);

      dd_bcm_pr_debug("Testing TX buffer %lu\n", (unsigned long)(t->tx_dirty_desc));

      desc = &(t->tx_cpu_desc[t->tx_dirty_desc]);

      /*
       We must call rmb() (barrier() is not enough because it's not a
       hardware barrier), to make sure we actually read the descriptor
       fields at each loop
      */
      rmb();

      len_stat = desc->len_stat;

      if (len_stat & DMADESC_OWNER_MASK) {
         // Buffer still owned by DMA
         break;
      }

      dd_bcm_pr_debug("TX buffer %lu released\n", (unsigned long)(t->tx_dirty_desc));

      t->tx_dirty_desc += 1;
      if (t->tx_dirty_desc >= t->tx_ring_size) {
         t->tx_dirty_desc = 0;
      }
      t->tx_desc_cnt_owned -= 1;
      t->tx_desc_cnt_empty += 1;

      old = test_and_set_bit(BIT_NR_TX_DESC_EMPTY, &(t->bits));
      if (!old) {
         dd_bcm_pr_debug("Running TX callback\n");
         (*(t->tx_desc_empty_cb))(t);
      }

      released += 1;
   }

   dd_bcm_pr_debug("%lu TX buffer(s) released\n", (unsigned long)(released));

   return (released);
}

/*
 * Handle at most one buffer of data from tx queue.
 * For each frame, call send_cb to get data
 * Returns the number of frames written
 */
size_t pcm_send_one_buffer(pcm_t *t,
   void (*send_cb)(pcm_t *t, __u8 *data, size_t data_len))
{
   size_t ret = 0;

   dd_bcm_pr_debug("%s()\n", __func__);

   d_bcm_assert(NULL != transmit_cb);

   if (pcm_can_send_one_buffer(t)) {
      size_t len;
      pcm_dma_desc_t *desc;
#ifndef BCMPH_NOHW
      dma_addr_t address;
#endif // !BCMPH_NOHW
      __u8 *dst;
      __u32 len_stat;

      /* point to the next available desc */
      desc = &(t->tx_cpu_desc[t->tx_curr_desc]);

      /*
       We must call rmb() (barrier() is not enough because it's not a
       hardware barrier), to make sure we actually read the descriptor
       fields at each loop
      */
      rmb();

      dd_bcm_pr_debug("Give TX buffer %lu to DMA\n", (unsigned long)(t->tx_curr_desc));

#ifndef BCMPH_NOHW
      address = t->tx_dma_addr[t->tx_curr_desc];
      dma_sync_single_for_cpu(t->dev, address, t->tx_data_buffer_size, DMA_TO_DEVICE);
#endif // !BCMPH_NOHW
      dst = &(t->tx_cpu_data[t->tx_curr_desc * (t->tx_data_buffer_size + RSV_DATA_BUFFER_LEN)]);
      len = 0;
      // Copy t->frame_size bytes
      for (;;) {
         (*(send_cb))(t, dst, t->frame_size);
         dst += t->max_frame_size;
         len += t->max_frame_size;
         ret += 1;
         if ((len + t->max_frame_size) > t->tx_data_buffer_rounded_size) {
            break;
         }
      }
#ifndef BCMPH_NOHW
      dma_sync_single_for_device(t->dev, address, t->tx_data_buffer_size, DMA_TO_DEVICE);
#endif // !BCMPH_NOHW

#ifdef BCMPH_TEST_PCM
      if (len < t->stats.min_size_tx_buffer) {
         t->stats.min_size_tx_buffer = len;
      }
      if (len > t->stats.max_size_tx_buffer) {
         t->stats.max_size_tx_buffer = len;
      }
#endif // !BCMPH_TEST_PCM

      /* Make sure buffer we copy data into is written to memory before we
       update desc, because DMA is polling */
      wmb();

      /* fill descriptor */
      desc->address = t->tx_dma_addr[t->tx_curr_desc];
      len_stat = (len << DMADESC_LENGTH_SHIFT) & DMADESC_LENGTH_MASK;
      len_stat |= (DMADESC_SOP_MASK | DMADESC_EOP_MASK);

      t->tx_curr_desc += 1;
      if (t->tx_curr_desc >= t->tx_ring_size) {
         t->tx_curr_desc = 0;
         len_stat |= DMADESC_WRAP_MASK;
      }
      t->tx_desc_cnt_empty -= 1;

      if (t->dma_is_started) {
         bcm_assert((t->tx_desc_cnt_empty + t->tx_desc_cnt_owned + 1) == t->tx_ring_size);

         len_stat |= (DMADESC_OWNER_MASK);

#ifdef BCMPH_NOHW
         pcm_set_desc_jiffies(t, desc);
#endif // BCMPH_NOHW
         desc->len_stat = len_stat;

         /* Make sure len_stat is written to memory before we enable DMA channel */
         wmb();

         t->tx_desc_cnt_owned += 1;

         /* Kick TX dma */
         pcm_dmac_writel(t, PCMDMAC_CHANCFG_EN_MASK, PCMDMAC_CHANCFG_REG(t->tx_chan));
      }
      else {
         bcm_assert((t->tx_desc_cnt_empty + t->tx_desc_cnt_owned) < t->tx_ring_size);

         desc->len_stat = len_stat;

         /* Make sure len_stat is written to memory */
         wmb();
      }

      t->stats.tx_bytes += len;
      t->stats.tx_good += 1;

      if (t->tx_desc_cnt_empty <= 0) {
         // We try to reclaim some buffers before setting f_tx_desc_empty
         // to 0
         if (pcm_tx_reclaim(t, t->tx_ring_size) <= 0) {
            int old;
            /*
             Set flag f_tx_desc_empty to false and call immediately
             pcm_tx_reclaim() to eventually update the flag
             because since the start of the function some buffers may have
             been transmitted, or worse the transmission of the last buffer
             to transmit is already finished
            */
            old = test_and_clear_bit(BIT_NR_TX_DESC_EMPTY, &(t->bits));
            if (old) {
               dd_bcm_pr_debug("Flag 'f_tx_desc_empty' set to false\n");
#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
               if (t->dma_is_started) {
                  __u32 reg;
                  /* Ack TX interrupts */
                  reg = pcm_dmac_readl(t, PCMDMAC_IR_REG(t->tx_chan));
                  pcm_dmac_writel(t, reg, PCMDMAC_IR_REG(t->tx_chan));
                  // Enable TX interrupts
                  pcm_dmac_writel(t, PCMDMAC_IR_BUFDONE_MASK | PCMDMAC_IR_PKTDONE_MASK, PCMDMAC_IRMASK_REG(t->tx_chan));
               }
#endif // BCMPH_ENABLE_PCM_INTERRUPTS
            }
         }
      }
   }

   return (ret);
}

/*
 * Handle at most frame_count frames of data from tx queue.
 * For each frame, call send_cb to get data
 * Returns the number of frames written
 */
size_t pcm_send(pcm_t *t,
   void (*send_cb)(pcm_t *t, __u8 *data, size_t data_len),
   size_t frame_count)
{
   size_t frames_not_copied = frame_count;
   size_t max_frames = pcm_get_max_frame_tx_buffer(t);

   dd_bcm_pr_debug("%s(frame_count=%lu)\n", __func__,
     (unsigned long)(frame_count));

   d_bcm_assert(NULL != transmit_cb);

   while (frame_count >= max_frames) {
      size_t tmp = pcm_send_one_buffer(t, send_cb);
      if (tmp <= 0) {
         break;
      }
      bcm_assert(tmp == max_frames);
      frame_count -= tmp;
   }

   return (frames_not_copied - frame_count);
}

int pcm6358_pll_init(const pcm_t *t)
{
   __u32 reg;

   bcm_pr_debug("%s()\n", __func__);

   bcm_pr_debug(" pll_ctrl1=%lx, pll_ctrl2=%lx, pll_ctrl3=%lx\n",
      (unsigned long)(pcm_readl(t, PCM_PLL_CTRL1_REG)),
      (unsigned long)(pcm_readl(t, PCM_PLL_CTRL2_REG)),
      (unsigned long)(pcm_readl(t, PCM_PLL_CTRL3_REG)));

   // I have no idea what is the meaning of these values :
   // they come from bcmendpoint.ko of firmware b066 of HG553
   // They have been confirmed by a person
   // who send me definitions of bits of PCM_PLL_CTRL1_REG for
   // the 6358
   pcm_writel(t, 0x8492B, PCM_PLL_CTRL2_REG);

   pcm_writel(t, 0x1E1C, PCM_PLL_CTRL3_REG);

   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg |= (1 << (PCM_6358_PLL_SRCSEL_SHIFT + 1));
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg &= (~(1 << PCM_6358_PLL_SRCSEL_SHIFT));
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   // Power up PLL
   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg &= (~(PCM_6358_PLL_PWRDN));
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg &= (~(PCM_6358_PLL_RESET));
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg &= (~(PCM_6358_PLL_FILRST));
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   msleep(10);

#ifndef BCMPH_NOHW
   reg = pcm_readl(t, PCM_PLL_STAT_6358_REG);
   if (PCM_PLL_LOCK != (reg & PCM_PLL_LOCK)) {
      bcm_pr_err("PLL didn't lock to programmed output frequency 0x%08x\n", (unsigned int)(reg));
      pcm6358_pll_deinit(t);
      return (-1);
   }
#endif // !BCMPH_NOHW

   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg &= (~(PCM_6358_CLK16_RESET));
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   msleep(10);

   // Here pll_ctrl1 should be 0x20001
   bcm_pr_debug(" pll_ctrl1=%lx, pll_ctrl2=%lx, pll_ctrl3=%lx\n",
      (unsigned long)(pcm_readl(t, PCM_PLL_CTRL1_REG)),
      (unsigned long)(pcm_readl(t, PCM_PLL_CTRL2_REG)),
      (unsigned long)(pcm_readl(t, PCM_PLL_CTRL3_REG)));

   return (0);
}

void pcm6358_pll_deinit(const pcm_t *t)
{
   __u32 reg;

   bcm_pr_debug("%s()\n", __func__);

   // Power down PLL
   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg |= PCM_6358_PLL_PWRDN;
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg |= PCM_6358_PLL_RESET;
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg |= PCM_6358_PLL_FILRST;
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   msleep(10);
}

int pcm6368_pll_init(const pcm_t *t)
{
   __u32 reg;

   bcm_pr_debug("%s()\n", __func__);

   bcm_pr_debug(" pll_ctrl1=%lx, pll_ctrl2=%lx, pll_ctrl3=%lx, pll_ctrl4=%lx\n",
      (unsigned long)(pcm_readl(t, PCM_PLL_CTRL1_REG)),
      (unsigned long)(pcm_readl(t, PCM_PLL_CTRL2_REG)),
      (unsigned long)(pcm_readl(t, PCM_PLL_CTRL3_REG)),
      (unsigned long)(pcm_readl(t, PCM_PLL_CTRL4_636x_REG)));

   /* Power up PLL */
   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg &= (~(PCM_636x_PLL_PWRDN | PCM_636x_PLL_PWRDN_CH1 | PCM_636x_PLL_REFCMP_PWRDN));
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   pcm_writel(t, 0x00000015, PCM_PLL_CTRL4_636x_REG); /* [5:0] = PLL_ctrl[37:32] */
#ifdef NTR_SUPPORT
   pcm_writel(t, 0x382c2860, PCM_PLL_CTRL3_REG); /* [31:0] = PLL_ctrl[31:0], VCO > 1.6GHz */
#else
   pcm_writel(t, 0x38000700, PCM_PLL_CTRL3_REG); /* [31:0] = PLL_ctrl[31:0] */
#endif
   pcm_writel(t, 0x10000000, PCM_PLL_CTRL2_REG); /* VCO is 2048MHZ, NDIV_MODE=0, NDIV_FRAC=0 */
   pcm_writel(t, 0x1C80147D, PCM_PLL_CTRL1_REG); /* MDIV=125(0x7D), P1DIV=4(0x4), P2DIV=1(0x1), NDIV_INT=128(0x080) */

   /* Remove Analog Reset */
   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg &= (~(PCM_636x_PLL_ARESET));
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   /* Remove Digital Reset */
   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg &= (~(PCM_636x_PLL_DRESET));
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   /* Clear clk 16 reset */
   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg &= (~(PCM_636x_PLL_CLK16_RESET));
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   msleep(10);

#ifndef BCMPH_NOHW
   reg = pcm_readl(t, PCM_PLL_STAT_636x_REG);
   if (PCM_PLL_LOCK != (reg & PCM_PLL_LOCK)) {
      bcm_pr_err("PLL didn't lock to programmed output frequency 0x%08x\n", (unsigned int)(reg));
      pcm6368_pll_deinit(t);
      return (-1);
   }
#endif // !BCMPH_NOHW

   bcm_pr_debug(" pll_ctrl1=%lx, pll_ctrl2=%lx, pll_ctrl3=%lx, pll_ctrl4=%lx\n",
      (unsigned long)(pcm_readl(t, PCM_PLL_CTRL1_REG)),
      (unsigned long)(pcm_readl(t, PCM_PLL_CTRL2_REG)),
      (unsigned long)(pcm_readl(t, PCM_PLL_CTRL3_REG)),
      (unsigned long)(pcm_readl(t, PCM_PLL_CTRL4_636x_REG)));

   return (0);
}

void pcm6368_pll_deinit(const pcm_t *t)
{
   __u32 reg;

   bcm_pr_debug("%s()\n", __func__);

   /* Apply clock 16 reset */
   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg |= PCM_636x_PLL_CLK16_RESET;
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   /* Analog Reset */
   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg |= PCM_636x_PLL_ARESET;
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   /* Digital Reset */
   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg |= PCM_636x_PLL_DRESET;
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   /* Power down PLL */
   reg = pcm_readl(t, PCM_PLL_CTRL1_REG);
   reg |= (PCM_636x_PLL_PWRDN | PCM_636x_PLL_PWRDN_CH1 | PCM_636x_PLL_REFCMP_PWRDN);
   pcm_writel(t, reg, PCM_PLL_CTRL1_REG);

   msleep(10);
}

/*****************************************************************************
*
*  FUNCTION: pcm_timeslot_alloc
*
*  PURPOSE:
*      Allocate PCM timeslot in the PCM table
*
*  PARAMETERS:
*
*  RETURNS:
*
*  NOTES:
*
*****************************************************************************/
static void pcm_timeslot_alloc(const pcm_t *t, __u8 chnum, __u8 ts)
{
   /*
   ** Each PCM timeslot register contains configuration for 8 timeslots
   ** (4 bits per TS : 1 bit to tell TS is valid and 3 bits for the channel number)
   ** Timeslots 0 to 7 are in the first PCM timeslot register,
   ** timeslots 8 to 15 in the second, etc.
   */
   int bit_offset = 28 - ((ts & 7) << 2);
   int ts_register_offset = PCM_SLOT_ALLOC_TBL_REG + ((ts >> 3) << 2);
   __u32 reg;

   bcm_pr_debug("%s(chnum=%u, timeslot=%u)\n", __func__, (unsigned int)(chnum), (unsigned int)(ts));

   reg = pcm_readl(t, ts_register_offset);
   reg &= (~(0xF << bit_offset));
   reg |= ((PCM_TS_VALID | chnum) << bit_offset);
   pcm_writel(t, reg, ts_register_offset);
}

#ifdef BCMPH_TEST_PCM
extern uint bcm_drv_param_pcm_ctrl;
#endif // BCMPH_TEST_PCM

static void pcm_regs_reset(pcm_t *t)
{
   size_t reg_idx;
   __u32 reg;
#ifndef BCMPH_TEST_PCM
   __u32 mask;
#endif // !BCMPH_TEST_PCM
   __u32 divider;

   bcm_pr_debug("%s()\n", __func__);

   divider = 0;
   for (;;) {
      if (t->board_desc->phone_desc->clk_rate < (8192 >> divider)) {
         divider += 1;
      }
      else {
         bcm_assert((8192 >> divider) == t->board_desc->phone_desc->clk_rate);
         break;
      }
   }
   d_bcm_pr_debug("PCM clock rate wanted is %lu and so clock divider is set to %lu and frame size is set to %lu\n",
      (unsigned long)(t->board_desc->phone_desc->clk_rate),
      (unsigned long)(divider),
      (unsigned long)(8192 >> (divider + 3)));

#ifndef BCMPH_TEST_PCM
   // Bit PCM_FS_FREQ_16_8 must be null : we don't handle 16 kHz frequency
   bcm_assert(8 == BCMPH_SAMPLES_PER_MS);
   mask = (PCM_SLAVE_SEL | PCM_CLOCK_INV | PCM_FS_INVERT | PCM_FS_LONG | PCM_FS_TRIG | PCM_DATA_OFF | PCM_LSB_FIRST | PCM_EXTCLK_SEL);
   reg = (t->board_desc->phone_desc->pcm_ctrl_reg & mask);
#else // BCMPH_TEST_PCM
   reg = bcm_drv_param_pcm_ctrl;
#endif // BCMPH_TEST_PCM
   if (pcm_timeslot_is_16bits(t)) {
      reg |= PCM_DATA_16_8; // If this flag is set, it means timeslot is 16 bits
   }
   reg |= (divider << PCM_CLOCK_SEL_SHIFT);
#ifndef BCMPH_TEST_PCM
   // Set number of bits per frame (not sure it is useful for BCM6358)
   reg |= (8192 >> (divider + 3));
   if (0x6358 == t->board_desc->cpu_desc->cpu_id) {
      reg |= PCM_6358_AP_SEL;
   }
#endif // !BCMPH_TEST_PCM
   pcm_writel(t, reg, PCM_CTRL_REG);
   bcm_pr_debug("PCM_CTRL_REG <- 0x%lx\n", (unsigned long)(reg));

   pcm_writel(t, 0, PCM_CHAN_CTRL_REG);

   // Clear time slot allocation table
   for (reg_idx = 0; (reg_idx < PCM_MAX_TIMESLOT_REGS); reg_idx += 1) {
      pcm_writel(t, 0, PCM_SLOT_ALLOC_TBL_REG + (reg_idx << 2));
   }

   /* Disable PCM interrupts */
   pcm_writel(t, 0, PCM_INT_MASK_REG);

   /* Clear pending interrupts at the PCM level */
   reg = pcm_readl(t, PCM_INT_PENDING_REG);
   pcm_writel(t, reg, PCM_INT_PENDING_REG);
}

static int pcm_regs_init(pcm_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   t->timeslot_is_16bits = false;
   t->frame_size = 0;
   memset(t->channel_to_timeslot, BCMPH_TIMESLOT_UNSPECIFIED, sizeof(t->channel_to_timeslot));
   pcm_regs_reset(t);

   return (0);
}

static void pcm_regs_deinit(pcm_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   pcm_regs_reset(t);
   t->frame_size = 0;
}

#ifdef BCMPH_TEST_PCM
#ifndef BCMPH_NOHW
static irqreturn_t pcm_isr(int irq, void *dev_id)
{
   pcm_t *t = dev_id;
   __u32 reg;

   t->stats.cnt_irq += 1;

   // Ack interrupt
   reg = pcm_dmac_readl(t, PCM_INT_PENDING_REG);
   pcm_dmac_writel(t, reg, PCM_INT_PENDING_REG);

   if ((reg | PCM_TX_UNDERFLOW)) {
      t->stats.cnt_irq_tx_underflow += 1;
   }
   if ((reg | PCM_RX_OVERFLOW)) {
      t->stats.cnt_irq_rx_overflow += 1;
   }

   return (IRQ_HANDLED);
}
#endif // !BCMPH_NOHW
#endif // BCMPH_TEST_PCM

#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
static irqreturn_t pcm_isr_dma_rx(int irq, void *dev_id)
{
   pcm_t *t = dev_id;
   __u32 reg;
   int old;

   d_bcm_pr_debug("%s()\n", __func__);

   /* Mask RX interrupts */
   pcm_dmac_writel(t, 0, PCMDMAC_IRMASK_REG(t->rx_chan));
   // Ack interrupt
   reg = pcm_dmac_readl(t, PCMDMAC_IR_REG(t->rx_chan));
   pcm_dmac_writel(t, reg, PCMDMAC_IR_REG(t->rx_chan));

   t->stats.cnt_irq_rx += 1;
   old = test_and_set_bit(BIT_NR_RX_DESC_RETURNED, &(t->bits));
   if (!old) {
      (*(t->rx_desc_returned_cb))(t);
   }

   return (IRQ_HANDLED);
}

static irqreturn_t pcm_isr_dma_tx(int irq, void *dev_id)
{
   pcm_t *t = dev_id;
   __u32 reg;
   int old;

   d_bcm_pr_debug("%s()\n", __func__);

   /* Mask TX interrupts */
   pcm_dmac_writel(t, 0, PCMDMAC_IRMASK_REG(t->tx_chan));
   // Ack interrupt
   reg = pcm_dmac_readl(t, PCMDMAC_IR_REG(t->tx_chan));
   pcm_dmac_writel(t, reg, PCMDMAC_IR_REG(t->tx_chan));

   t->stats.cnt_irq_tx += 1;
   old = test_and_set_bit(BIT_NR_TX_DESC_EMPTY, &(t->bits));
   if (!old) {
      (*(t->tx_desc_empty_cb))(t);
   }

   return (IRQ_HANDLED);
}
#endif // BCMPH_ENABLE_PCM_INTERRUPTS

static void pcm_dma_rx_descs_reset(pcm_t *t)
{
   size_t desc_idx;

   bcm_pr_debug("%s()\n", __func__);

   // Erase all pending data not read and initialize ring buffer for RX
#if (RSV_DATA_BUFFER_LEN == 0)
   memset(t->rx_cpu_data, 0, sizeof(__u8) * t->rx_data_buffer_size  * t->rx_ring_size);
#endif
   for (desc_idx = 0; (desc_idx < t->rx_ring_size); desc_idx += 1) {
      __u32 len_stat;

#if (RSV_DATA_BUFFER_LEN > 0)
      memset(&(t->rx_cpu_data[desc_idx * (t->rx_data_buffer_size + RSV_DATA_BUFFER_LEN)]), 0, t->rx_data_buffer_size);
#endif
      // Do not set DMADESC_OWNER_MASK now.
      // It will be done just before starting DMA
      len_stat = (t->rx_data_buffer_rounded_size << DMADESC_LENGTH_SHIFT);
      if ((desc_idx + 1) == t->rx_ring_size) {
         len_stat |= DMADESC_WRAP_MASK;
      }
      t->rx_cpu_desc[desc_idx].address = t->rx_dma_addr[desc_idx];
      t->rx_cpu_desc[desc_idx].len_stat = len_stat;
   }
   t->rx_desc_cnt_owned = 0;
   t->rx_dirty_desc = 0;
   t->rx_curr_desc = 0;
   clear_bit(BIT_NR_RX_DESC_RETURNED, &(t->bits));
}

static void pcm_dma_tx_descs_reset(pcm_t *t)
{
   size_t desc_idx;

   bcm_pr_debug("%s()\n", __func__);

   // Erase all pending data not transmitted, so we can start filling TX buffer
   // before starting PCM

#if (RSV_DATA_BUFFER_LEN == 0)
   memset(t->tx_cpu_data, 0, sizeof(__u8) * t->tx_data_buffer_size * t->tx_ring_size);
#endif
   for (desc_idx = 0; (desc_idx < t->tx_ring_size); desc_idx += 1) {
      __u32 len_stat;

#if (RSV_DATA_BUFFER_LEN > 0)
      memset(&(t->tx_cpu_data[desc_idx * (t->tx_data_buffer_size + RSV_DATA_BUFFER_LEN)]), 0, t->tx_data_buffer_size);
#endif
      len_stat = (DMADESC_SOP_MASK | DMADESC_EOP_MASK);
      // Do not set DMADESC_OWNER_MASK as there is nothing to transmit
      if ((desc_idx + 1) == t->tx_ring_size) {
         len_stat |= DMADESC_WRAP_MASK;
      }
      t->tx_cpu_desc[desc_idx].address = t->tx_dma_addr[desc_idx];
      t->tx_cpu_desc[desc_idx].len_stat = len_stat;
   }
   t->tx_desc_cnt_owned = 0;
   t->tx_desc_cnt_empty = t->tx_ring_size;
   t->tx_dirty_desc = 0;
   t->tx_curr_desc = 0;
   set_bit(BIT_NR_TX_DESC_EMPTY, &(t->bits));
}

// Do NOT call when pcm dma is stopped
static inline void pcm_refill_rx(pcm_t *t)
{
   dd_bcm_pr_debug("%s()\n", __func__);

   while (t->rx_desc_cnt_owned < t->rx_ring_size) {
      pcm_give_one_rx_buf_to_dma(t);
   }

   bcm_assert(t->rx_dirty_desc == t->rx_curr_desc);
}

void pcm_dma_start(pcm_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   if (t->is_started) {

      bcm_assert(!t->dma_is_started);

      // We recycle non empty RX buffers not owned by DMA
      while (t->rx_desc_cnt_owned > 0) {
         __u32 len_stat;
         pcm_dma_desc_t *desc;

         desc = &(t->rx_cpu_desc[t->rx_curr_desc]);

         /*
          We must call rmb() (barrier() is not enough because it's not a
          hardware barrier), to make sure we actually read the descriptor
          fields at each loop
         */
         rmb();

         len_stat = desc->len_stat;

         /* break if dma ownership still belongs to hw */
         if (len_stat & DMADESC_OWNER_MASK) {
            break;
         }

         pcm_rx_curr_buffer_handled(t);
      }

      clear_bit(BIT_NR_RX_DESC_RETURNED, &(t->bits));

      t->dma_is_started = true;

#ifdef BCMPH_NOHW
      bcm_period_set_period(&(t->timestamp), pcm_get_time_to_receive_rx_buffer(t));
#endif // BCMPH_NOHW

      // Give RX buffers to DMA
      pcm_refill_rx(t);

      // Give TX buffers filled to DMA
      pcm_refill_tx(t);

      // Just in case, but already done in pcm_refill_rx()
      if (t->rx_desc_cnt_owned > 0) {
         pcm_dmac_writel(t, PCMDMAC_CHANCFG_EN_MASK, PCMDMAC_CHANCFG_REG(t->rx_chan));
      }

      // Just in case, but already done in pcm_refill_tx()
      if (t->tx_desc_cnt_owned > 0) {
         pcm_dmac_writel(t, PCMDMAC_CHANCFG_EN_MASK, PCMDMAC_CHANCFG_REG(t->tx_chan));
      }
   }
}

void pcm_dma_stop(pcm_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   if (t->is_started) {
      /*
       Because we have no documentation, we don't know how to safely
       stop DMA without stopping PCM
       - can master DMA can be stopped (with register PCMDMA_CFG_REG)
         if PCM is still running ?
       - if we stop only RX and TX channels (with register PCMDMAC_CHANCFG_REG)
         how to stop transfer at the end of a buffer and not in the middle
         of a buffer ?
       - can we reset the registers PCMDMAS_RSTART_REG, PCMDMAS_SRAM2_REG,
         PCMDMAS_SRAM3_REG and PCMDMAS_SRAM4_REG if master DMA is not stopped ?

       So we stop it logically, ie RX buffer are not given to DMA when
       emptied, and TX buffer are not given to DMA when filled.
       Of course it means that we continue to receive data until all
       buffers owned by DMA are filled, and to send data
       until all buffer owned by DMA are empty.
      */
      t->dma_is_started = false;
   }
}

static int pcm_dma_init(pcm_t *t)
{
   int ret;
   size_t desc_idx;
   size_t size;
   __u8 *p;
   __u32 reg;

   bcm_pr_debug("%s()\n", __func__);

   // Init DMA registers
   pcm_dma_writel(t, 0, PCMDMA_CFG_REG);

   pcm_dmac_writel(t, 0, PCMDMAC_CHANCFG_REG(t->rx_chan));

   pcm_dmas_writel(t, 0, PCMDMAS_RSTART_REG(t->rx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM2_REG(t->rx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM3_REG(t->rx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM4_REG(t->rx_chan));

   pcm_dmac_writel(t, 0, PCMDMAC_IRMASK_REG(t->rx_chan));
   reg = pcm_dmac_readl(t, PCMDMAC_IR_REG(t->rx_chan));
   pcm_dmac_writel(t, reg, PCMDMAC_IR_REG(t->rx_chan));

   pcm_dmac_writel(t, 0, PCMDMAC_CHANCFG_REG(t->tx_chan));

   pcm_dmas_writel(t, 0, PCMDMAS_RSTART_REG(t->tx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM2_REG(t->tx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM3_REG(t->tx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM4_REG(t->tx_chan));

   pcm_dmac_writel(t, 0, PCMDMAC_IRMASK_REG(t->tx_chan));
   reg = pcm_dmac_readl(t, PCMDMAC_IR_REG(t->tx_chan));
   pcm_dmac_writel(t, reg, PCMDMAC_IR_REG(t->tx_chan));

#ifndef BCMPH_NOHW
   bcm_assert(8 == sizeof(pcm_dma_desc_t));
#endif // !BCMPH_NOHW

   /* Allocate RX desc ring */
   size = sizeof(__u8) * t->rx_ring_size * sizeof(pcm_dma_desc_t);
   bcm_pr_debug("Allocating %lu bytes of DMA memory for RX descs\n", (unsigned long)(size));
#ifndef BCMPH_NOHW
   p = (__u8 *)(dma_alloc_coherent(t->dev, size, &(t->rx_dma_desc), GFP_KERNEL | __GFP_ZERO | __GFP_NORETRY));
#else // BCMPH_NOHW
   p = (__u8 *)(kmalloc(size, GFP_KERNEL | __GFP_ZERO | __GFP_NORETRY));
#endif // BCMPH_NOHW
   if (NULL == p) {
      bcm_pr_err("Cannot allocate RX descs ring %lu\n", (unsigned long)(size));
      ret = -ENOMEM;
      goto fail_rx_descs;
   }
   bcm_pr_debug("%lu bytes of memory allocated, starting at address 0x%lx\n", (unsigned long)(size), (unsigned long)(p));
   t->rx_desc_alloc_size = size;
   t->rx_cpu_desc = (pcm_dma_desc_t *)(p);

   /* Allocate TX desc ring */
   size = sizeof(__u8) * t->tx_ring_size * sizeof(pcm_dma_desc_t);
   bcm_pr_debug("Allocating %lu bytes of DMA memory for TX descs\n", (unsigned long)(size));
#ifndef BCMPH_NOHW
   p = (__u8 *)(dma_alloc_coherent(t->dev, size, &(t->tx_dma_desc), GFP_KERNEL | __GFP_ZERO | __GFP_NORETRY));
#else // BCMPH_NOHW
   p = (__u8 *)(kmalloc(size, GFP_KERNEL | __GFP_ZERO | __GFP_NORETRY));
#endif // BCMPH_NOHW
   if (NULL == p) {
      bcm_pr_err("Cannot allocate TX descs ring %lu\n", (unsigned long)(size));
      ret = -ENOMEM;
      goto fail_tx_descs;
   }
   bcm_pr_debug("%lu bytes of memory allocated, starting at address 0x%lx\n", (unsigned long)(size), (unsigned long)(p));
   t->tx_desc_alloc_size = size;
   t->tx_cpu_desc = (pcm_dma_desc_t *)(p);

   /* Allocate RX and TX data ring */
   size = sizeof(__u8) * (
      round_up_to_pow_of_2(((t->rx_data_buffer_size + RSV_DATA_BUFFER_LEN) * t->rx_ring_size), t->board_desc->cpu_desc->dcache_line_size)
      + round_up_to_pow_of_2(((t->tx_data_buffer_size + RSV_DATA_BUFFER_LEN) * t->tx_ring_size), t->board_desc->cpu_desc->dcache_line_size));
   bcm_pr_debug("Allocating %lu bytes of memory for RX and TX data buffers\n", (unsigned long)(size));
   p = (__u8 *)(kmalloc(size, GFP_KERNEL | __GFP_NORETRY | GFP_DMA));
   if (NULL == p) {
      bcm_pr_err("Cannot allocate data buffers\n");
      ret = -ENOMEM;
      goto fail_data;
   }
   bcm_pr_debug("%lu bytes of memory allocated, starting at address 0x%lx\n", (unsigned long)(size), (unsigned long)(p));
   t->rx_cpu_data = p;
   p += round_up_to_pow_of_2(((t->rx_data_buffer_size + RSV_DATA_BUFFER_LEN) * t->rx_ring_size), t->board_desc->cpu_desc->dcache_line_size);
   t->tx_cpu_data = p;

#if (RSV_DATA_BUFFER_LEN > 0)
   for (desc_idx = 0; (desc_idx < t->rx_ring_size); desc_idx += 1) {
      size_t j;
      p = &(t->rx_cpu_data[(desc_idx * (t->rx_data_buffer_size + RSV_DATA_BUFFER_LEN)) + t->rx_data_buffer_size]);
      for (j = 0; (j < RSV_DATA_BUFFER_LEN); j += 1) {
         p[j] = 0x75 + desc_idx + j;
      }
   }
   for (desc_idx = 0; (desc_idx < t->tx_ring_size); desc_idx += 1) {
      size_t j;
      p = &(t->tx_cpu_data[(desc_idx * (t->tx_data_buffer_size + RSV_DATA_BUFFER_LEN)) + t->tx_data_buffer_size]);
      for (j = 0; (j < RSV_DATA_BUFFER_LEN); j += 1) {
         p[j] = 0xB9 + desc_idx + j;
      }
   }
#endif

   /* Allocate space to store DMA adresses of the RX and TX buffers */
   size = sizeof(__u8) * (
      round_up_to_pow_of_2((sizeof(dma_addr_t) * t->rx_ring_size), t->board_desc->cpu_desc->dcache_line_size)
      + round_up_to_pow_of_2((sizeof(dma_addr_t) * t->tx_ring_size), t->board_desc->cpu_desc->dcache_line_size));
   bcm_pr_debug("Allocating %lu bytes of memory for DMA addresses\n", (unsigned long)(size));
   p = (__u8 *)(kmalloc(size, GFP_KERNEL | __GFP_NORETRY));
   if (NULL == p) {
      bcm_pr_err("Cannot allocate memory for DMA adresses\n");
      ret = -ENOMEM;
      goto fail_dma_addr;
   }
   bcm_pr_debug("%lu bytes of memory allocated, starting at address 0x%lx\n", (unsigned long)(size), (unsigned long)(p));
   t->rx_dma_addr = (dma_addr_t *)(p);
   p += round_up_to_pow_of_2((sizeof(dma_addr_t) * t->rx_ring_size), t->board_desc->cpu_desc->dcache_line_size);
   t->tx_dma_addr = (dma_addr_t *)(p);

#ifndef BCMPH_NOHW
   /* initialize descriptors addresses */
   bcm_pr_debug("Mapping DMA addresses of RX descs\n");
   for (desc_idx = 0; (desc_idx < t->rx_ring_size); desc_idx += 1) {
       t->rx_dma_addr[desc_idx] = dma_map_single(t->dev,
         &(t->rx_cpu_data[desc_idx * t->rx_data_buffer_size]),
         t->rx_data_buffer_size, DMA_FROM_DEVICE);
      if (dma_mapping_error(t->dev, t->rx_dma_addr[desc_idx])) {
         size_t j;
         for (j = 0; (j < desc_idx); j += 1) {
            dma_unmap_single(t->dev, t->rx_dma_addr[j],
               t->rx_data_buffer_size, DMA_FROM_DEVICE);
         }
         bcm_pr_err("Fail to map DMA address for RX data buffer %u", (unsigned int)(desc_idx));
         goto fail_map_rx;
      }
      t->rx_cpu_desc[desc_idx].address = t->rx_dma_addr[desc_idx];
   }

   bcm_pr_debug("Mapping DMA addresses of TX descs\n");
   for (desc_idx = 0; (desc_idx < t->tx_ring_size); desc_idx += 1) {
      t->tx_dma_addr[desc_idx] = dma_map_single(t->dev,
         &(t->tx_cpu_data[desc_idx * t->tx_data_buffer_size]),
         t->tx_data_buffer_size, DMA_TO_DEVICE);
      if (dma_mapping_error(t->dev, t->tx_dma_addr[desc_idx])) {
         size_t j;
         for (j = 0; (j < desc_idx); j += 1) {
            dma_unmap_single(t->dev, t->tx_dma_addr[j],
               t->tx_data_buffer_size, DMA_TO_DEVICE);
         }
         bcm_pr_err("Fail to map DMA address for TX data buffer %u", (unsigned int)(desc_idx));
         goto fail_map_tx;
      }
      t->tx_cpu_desc[desc_idx].address = t->tx_dma_addr[desc_idx];
   }
#else // BCMPH_NOHW
   for (desc_idx = 0; (desc_idx < t->rx_ring_size); desc_idx += 1) {
      t->rx_dma_addr[desc_idx] = (dma_addr_t)(&(t->rx_cpu_data[desc_idx * (t->rx_data_buffer_size + RSV_DATA_BUFFER_LEN)]));
   }
   for (desc_idx = 0; (desc_idx < t->tx_ring_size); desc_idx += 1) {
      t->tx_dma_addr[desc_idx] = (dma_addr_t)(&(t->tx_cpu_data[desc_idx * (t->tx_data_buffer_size + RSV_DATA_BUFFER_LEN)]));
   }
#endif // BCMPH_NOHW

   /* Prepare RX and TX descs as we can read or write before starting PCM */
   pcm_dma_rx_descs_reset(t);
   pcm_dma_tx_descs_reset(t);

   /* Ensure all writes done before initializing ring adresses */
   wmb();

#if 0
   pcm_dma_writel(t, PCMDMA_BUFALLOC_FORCE_MASK | 0,
      PCMDMA_BUFALLOC_REG(t->rx_chan));
#endif

#ifndef BCMPH_NOHW
   pcm_dmas_writel(t, t->rx_dma_desc, PCMDMAS_RSTART_REG(t->rx_chan));
#endif // BCMPH_NOHW
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM2_REG(t->rx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM3_REG(t->rx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM4_REG(t->rx_chan));

#ifndef BCMPH_NOHW
   pcm_dmas_writel(t, t->tx_dma_desc, PCMDMAS_RSTART_REG(t->tx_chan));
#endif // BCMPH_NOHW
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM2_REG(t->tx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM3_REG(t->tx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM4_REG(t->tx_chan));

#if 0
   /* Set flow control low/high threshold to 1/3 / 2/3 */
   reg = (t->rx_ring_size / 3);
   pcm_dma_writel(t, reg, PCMDMA_FLOWCL_REG(t->rx_chan));
   reg = ((t->rx_ring_size * 2) / 3);
   pcm_dma_writel(t, reg, PCMDMA_FLOWCH_REG(t->rx_chan));
#endif

#ifdef BCMPH_TEST_PCM
#ifndef BCMPH_NOHW
   bcm_pr_debug("Requesting PCM irq %d\n", (int)(t->board_desc->pcm_desc->irq));
   ret = request_irq(t->board_desc->pcm_desc->irq, pcm_isr, 0,
           driver_name, t);
   if (ret) {
      bcm_pr_err("Failed to get irq %d", (int)(t->board_desc->pcm_desc->irq));
      goto fail_irq;
   }
   t->irq = t->board_desc->pcm_desc->irq;
#endif // !BCMPH_NOHW
#endif // BCMPH_TEST_PCM

#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
   bcm_pr_debug("Requesting PCM DMA RX irq %d\n", (int)(t->board_desc->pcm_desc->irq_dma_rx));
   ret = request_irq(t->board_desc->pcm_desc->irq_dma_rx, pcm_isr_dma_rx, 0,
           driver_name, t);
   if (ret) {
      bcm_pr_err("Failed to get PCM DMA RX irq %d", (int)(t->board_desc->pcm_desc->irq_dma_rx));
      goto fail_irq_dma_rx;
   }
   t->irq_dma_rx = t->board_desc->pcm_desc->irq_dma_rx;

   bcm_pr_debug("Requesting PCM DMA TX irq %d\n", (int)(t->board_desc->pcm_desc->irq_dma_tx));
   ret = request_irq(t->board_desc->pcm_desc->irq_dma_tx, pcm_isr_dma_tx, 0,
            driver_name, t);
   if (ret) {
      bcm_pr_err("Failed to get PCM DMA TX irq %d", (int)(t->board_desc->pcm_desc->irq_dma_tx));
      goto fail_irq_dma_tx;
   }
   t->irq_dma_tx = t->board_desc->pcm_desc->irq_dma_tx;
#endif // BCMPH_ENABLE_PCM_INTERRUPTS
   return (0);

#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
   bcm_pr_debug("Releasing PCM DMA TX irq\n");
   free_irq(t->irq_dma_tx, t);
   t->irq_dma_tx = -1;
fail_irq_dma_tx:
   bcm_pr_debug("Releasing PCM DMA RX irq\n");
   free_irq(t->irq_dma_rx, t);
   t->irq_dma_rx = -1;
fail_irq_dma_rx:
#endif // BCMPH_ENABLE_PCM_INTERRUPTS
#ifdef BCMPH_TEST_PCM
#ifndef BCMPH_NOHW
   bcm_pr_debug("Releasing PCM irq\n");
   free_irq(t->irq, t);
   t->irq = -1;
fail_irq:
#endif // !BCMPH_NOHW
#endif // BCMPH_TEST_PCM
#ifndef BCMPH_NOHW
   bcm_pr_debug("Unmapping DMA addresses of TX descs\n");
   for (desc_idx = 0; (desc_idx < t->tx_ring_size); desc_idx += 1) {
      dma_unmap_single(t->dev, t->tx_dma_addr[desc_idx],
         t->tx_data_buffer_size, DMA_TO_DEVICE);
   }
fail_map_tx:
   bcm_pr_debug("Unmapping DMA addresses of RX descs\n");
   for (desc_idx = 0; (desc_idx < t->rx_ring_size); desc_idx += 1) {
      dma_unmap_single(t->dev, t->rx_dma_addr[desc_idx],
         t->rx_data_buffer_size, DMA_FROM_DEVICE);
   }
fail_map_rx:
#endif // !BCMPH_NOHW
   bcm_pr_debug("Freeing memory for DMA adresses\n");
   kfree(t->rx_dma_addr);
   t->rx_dma_addr = NULL;
   t->tx_dma_addr = NULL;
fail_dma_addr:
   bcm_pr_debug("Freeing memory of RX and TX data buffers\n");
   kfree(t->rx_cpu_data);
   t->rx_cpu_data = NULL;
   t->tx_cpu_data = NULL;
fail_data:
#ifndef BCMPH_NOHW
   bcm_pr_debug("Freeing DMA memory of TX descs\n");
   dma_free_coherent(t->dev, t->tx_desc_alloc_size,
         t->tx_cpu_desc, t->tx_dma_desc);
#else // BCMPH_NOHW
   bcm_pr_debug("Freeing memory of TX descs\n");
   kfree(t->tx_cpu_desc);
   t->tx_cpu_desc = NULL;
#endif // BCMPH_NOHW
fail_tx_descs:
#ifndef BCMPH_NOHW
   bcm_pr_debug("Freeing DMA memory of RX descs\n");
   dma_free_coherent(t->dev, t->rx_desc_alloc_size,
         t->rx_cpu_desc, t->rx_dma_desc);
#else // BCMPH_NOHW
   bcm_pr_debug("Freeing memory of RX descs\n");
   kfree(t->rx_cpu_desc);
   t->rx_cpu_desc = NULL;
#endif // BCMPH_NOHW
fail_rx_descs:
   return (ret);
}

static void pcm_dma_deinit(pcm_t *t)
{
#if (!defined BCMPH_NOHW) || (RSV_DATA_BUFFER_LEN > 0)
   size_t desc_idx;
#endif // !BCMPH_NOHW  || (RSV_DATA_BUFFER_LEN > 0)
   bcm_pr_debug("%s()\n", __func__);

#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
   /* Frees irqs */
   bcm_pr_debug("Releasing PCM DMA TX irq\n");
   free_irq(t->irq_dma_tx, t);
   t->irq_dma_tx = -1;

   bcm_pr_debug("Releasing PCM DMA RX irq\n");
   free_irq(t->irq_dma_rx, t);
   t->irq_dma_rx = -1;
#endif // BCMPH_ENABLE_PCM_INTERRUPTS

#ifdef BCMPH_TEST_PCM
#ifndef BCMPH_NOHW
   bcm_pr_debug("Releasing PCM irq\n");
   free_irq(t->irq, t);
   t->irq = -1;
#endif // !BCMPH_NOHW
#endif // BCMPH_TEST_PCM

#ifndef BCMPH_NOHW
   bcm_pr_debug("Unmapping DMA addresses of TX descs\n");
   for (desc_idx = 0; (desc_idx < t->tx_ring_size); desc_idx += 1) {
      dma_unmap_single(t->dev, t->tx_dma_addr[desc_idx],
         t->tx_data_buffer_size, DMA_TO_DEVICE);
   }
   bcm_pr_debug("Unmapping DMA addresses of RX descs\n");
   for (desc_idx = 0; (desc_idx < t->rx_ring_size); desc_idx += 1) {
      dma_unmap_single(t->dev, t->rx_dma_addr[desc_idx],
         t->rx_data_buffer_size, DMA_FROM_DEVICE);
   }
#endif // !BCMPH_NOHW

   /* Reset entire IUDMA state RAM */
   pcm_dmas_writel(t, 0, PCMDMAS_RSTART_REG(t->rx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM2_REG(t->rx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM3_REG(t->rx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM4_REG(t->rx_chan));

   /* Reset entire IUDMA state RAM */
   pcm_dmas_writel(t, 0, PCMDMAS_RSTART_REG(t->tx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM2_REG(t->tx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM3_REG(t->tx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM4_REG(t->tx_chan));

#if (RSV_DATA_BUFFER_LEN > 0)
   for (desc_idx = 0; (desc_idx < t->rx_ring_size); desc_idx += 1) {
      size_t j;
      __u8 *p = &(t->rx_cpu_data[(desc_idx * (t->rx_data_buffer_size + RSV_DATA_BUFFER_LEN)) + t->rx_data_buffer_size]);
      for (j = 0; (j < RSV_DATA_BUFFER_LEN); j += 1) {
         if (p[j] != (0x75 + desc_idx + j)) {
            bcm_assert(0);
            break;
         }
      }
   }
   for (desc_idx = 0; (desc_idx < t->tx_ring_size); desc_idx += 1) {
      size_t j;
      __u8 *p = &(t->tx_cpu_data[(desc_idx * (t->tx_data_buffer_size + RSV_DATA_BUFFER_LEN)) + t->tx_data_buffer_size]);
      for (j = 0; (j < RSV_DATA_BUFFER_LEN); j += 1) {
         if (p[j] != (0xB9 + desc_idx + j)) {
            bcm_assert(0);
            break;
         }
      }
   }
#endif

   memset(t->rx_cpu_data, 0, t->rx_data_buffer_size * t->rx_ring_size);
   memset(t->tx_cpu_data, 0, t->tx_data_buffer_size * t->tx_ring_size);
   memset(t->rx_cpu_desc, 0, t->rx_desc_alloc_size);
   memset(t->tx_cpu_desc, 0, t->tx_desc_alloc_size);

   bcm_pr_debug("Freeing memory of RX and TX data buffers\n");
   kfree(t->rx_cpu_data);
   t->rx_cpu_data = NULL;
   t->tx_cpu_data = NULL;
   bcm_pr_debug("Freeing memory for DMA adresses\n");
   kfree(t->rx_dma_addr);
   t->rx_dma_addr = NULL;
   t->tx_dma_addr = NULL;

   bcm_pr_debug("Freeing DMA memory of RX descs\n");
#ifndef BCMPH_NOHW
   dma_free_coherent(t->dev, t->rx_desc_alloc_size,
         t->rx_cpu_desc, t->rx_dma_desc);
#else // BCMPH_NOHW
   kfree(t->rx_cpu_desc);
#endif // BCMPH_NOHW
   t->rx_cpu_desc = NULL;

   bcm_pr_debug("Freeing DMA memory of TX descs\n");
#ifndef BCMPH_NOHW
   dma_free_coherent(t->dev, t->tx_desc_alloc_size,
         t->tx_cpu_desc, t->tx_dma_desc);
#else // BCMPH_NOHW
   kfree(t->tx_cpu_desc);
#endif // BCMPH_NOHW
   t->tx_cpu_desc = NULL;
}

#ifndef BCMPH_NOHW
static int pcm_get_mem_res(pcm_mem_res_t *res, const char *id)
{
   int ret;
   void __iomem *p;

   bcm_pr_debug("%s(id=%s, addr=0x%lx, size=%lu)\n", __func__, id, (unsigned long)(res->start), (unsigned long)(res->end - res->start + 1));

   d_bcm_pr_debug("Requesting memory region\n");
   if (!request_mem_region(res->start, res->end - res->start + 1, id)) {
      bcm_pr_err("Failed to request mem region %s", id);
      ret = -EBUSY;
      goto fail_request;
   }

   d_bcm_pr_debug("Making I/O remap\n");
   p = ioremap_nocache(res->start, res->end - res->start + 1);
   if (NULL == p) {
      bcm_pr_err("Failed to map mem region %s", id);
      ret = -ENOMEM;
      goto fail_ioremap;
   }

   res->base = p;

   return (0);

   iounmap(res->base);
fail_ioremap:
   release_mem_region(res->start, res->end - res->start + 1);
fail_request:
   return (ret);
}

static void pcm_release_mem_res(pcm_mem_res_t *res)
{
   bcm_pr_debug("%s(addr=0x%lx, size=%lu)\n", __func__, (unsigned long)(res->start), (unsigned long)(res->end - res->start + 1));

   d_bcm_pr_debug("Making I/O unmap\n");
   iounmap(res->base);
   res->base = NULL;

   d_bcm_pr_debug("Releasing memory region\n");
   release_mem_region(res->start, res->end - res->start + 1);
}

static int pcm_get_shared_res(const board_desc_t *board_desc)
{
   int ret;

   bcm_pr_debug("%s()\n", __func__);

   pcm_shared_res.pcm_mem.start = board_desc->pcm_desc->base;
   pcm_shared_res.pcm_mem.end = pcm_shared_res.pcm_mem.start;
   pcm_shared_res.pcm_mem.end += (RSET_PCM_SIZE - 1);
   ret = pcm_get_mem_res(&(pcm_shared_res.pcm_mem), "pcm");
   if (ret) {
      goto fail_get_pcm_mem;
   }

   pcm_shared_res.pcm_dma_mem.start = board_desc->pcm_desc->dma_base;
   pcm_shared_res.pcm_dma_mem.end = pcm_shared_res.pcm_dma_mem.start;
   pcm_shared_res.pcm_dma_mem.end += (RSET_PCMDMA_SIZE - 1);
   ret = pcm_get_mem_res(&(pcm_shared_res.pcm_dma_mem), "pcm_dma");
   if (ret) {
      goto fail_get_pcm_dma_mem;
   }

   pcm_shared_res.pcm_dmac_mem.start = board_desc->pcm_desc->dmac_base;
   pcm_shared_res.pcm_dmac_mem.end = pcm_shared_res.pcm_dmac_mem.start;
   // We always use only two dma channels
   pcm_shared_res.pcm_dmac_mem.end += (RSET_PCMDMAC_SIZE(2) - 1);
   ret = pcm_get_mem_res(&(pcm_shared_res.pcm_dmac_mem), "pcm_dmac");
   if (ret) {
      goto fail_get_pcm_dmac_mem;
   }

   pcm_shared_res.pcm_dmas_mem.start = board_desc->pcm_desc->dmas_base;
   pcm_shared_res.pcm_dmas_mem.end = pcm_shared_res.pcm_dmas_mem.start;
   // We always use only two dma channels
   pcm_shared_res.pcm_dmas_mem.end += (RSET_PCMDMAS_SIZE(2) - 1);
   ret = pcm_get_mem_res(&(pcm_shared_res.pcm_dmas_mem), "pcm_dmas");
   if (ret) {
      goto fail_get_pcm_dmas_mem;
   }

   return (0);

   pcm_release_mem_res(&(pcm_shared_res.pcm_dmas_mem));
fail_get_pcm_dmas_mem:
   pcm_release_mem_res(&(pcm_shared_res.pcm_dmac_mem));
fail_get_pcm_dmac_mem:
   pcm_release_mem_res(&(pcm_shared_res.pcm_dma_mem));
fail_get_pcm_dma_mem:
   pcm_release_mem_res(&(pcm_shared_res.pcm_mem));
fail_get_pcm_mem:
   return (ret);
}

static void pcm_release_shared_res(void)
{
   bcm_pr_debug("%s()\n", __func__);

   pcm_release_mem_res(&(pcm_shared_res.pcm_dmas_mem));
   pcm_release_mem_res(&(pcm_shared_res.pcm_dmac_mem));
   pcm_release_mem_res(&(pcm_shared_res.pcm_dma_mem));
   pcm_release_mem_res(&(pcm_shared_res.pcm_mem));
}
#endif // !BCMPH_NOHW

static inline void pcm_pr_regs(const pcm_t *t)
{
   bcm_phone_pcm_regs_t regs;
   size_t reg_idx;

   pcm_read_regs(t, &(regs));

   bcm_pr_debug("======= Regs =======\n");
   bcm_pr_debug("ctlr        = 0x%lx\n", (unsigned long)(regs.ctlr));
   bcm_pr_debug("chan_ctrl   = 0x%lx\n", (unsigned long)(regs.chan_ctrl));
   bcm_pr_debug("int_pending = 0x%lx\n", (unsigned long)(regs.int_pending));
   bcm_pr_debug("int_mask    = 0x%lx\n", (unsigned long)(regs.int_mask));
   bcm_pr_debug("pll_ctrl1   = 0x%lx\n", (unsigned long)(regs.pll_ctrl1));
   bcm_pr_debug("pll_ctrl2   = 0x%lx\n", (unsigned long)(regs.pll_ctrl2));
   bcm_pr_debug("pll_ctrl3   = 0x%lx\n", (unsigned long)(regs.pll_ctrl3));
   bcm_pr_debug("pll_ctrl4   = 0x%lx\n", (unsigned long)(regs.pll_ctrl4));
   bcm_pr_debug("pll_stat    = 0x%lx\n", (unsigned long)(regs.pll_stat));
   for (reg_idx = 0; (reg_idx < ARRAY_SIZE(regs.slot_alloc_tbl)); reg_idx += 1) {
      bcm_pr_debug("slot_alloc_tbl[%lu] = 0x%lx\n", (unsigned long)(reg_idx), (unsigned long)(regs.slot_alloc_tbl[reg_idx]));
   }
   bcm_pr_debug("dma_cfg = 0x%lx\n", (unsigned long)(regs.dma_cfg));
   for (reg_idx = 0; (reg_idx < ARRAY_SIZE(regs.dma_channels)); reg_idx += 1) {
      bcm_pr_debug("dma_channels[%lu].dma_flowcl    = 0x%lx\n",
         (unsigned long)(reg_idx), (unsigned long)(regs.dma_channels[reg_idx].dma_flowcl));
      bcm_pr_debug("dma_channels[%lu].dma_flowch    = 0x%lx\n",
         (unsigned long)(reg_idx), (unsigned long)(regs.dma_channels[reg_idx].dma_flowch));
      bcm_pr_debug("dma_channels[%lu].dma_bufalloc  = 0x%lx\n",
         (unsigned long)(reg_idx), (unsigned long)(regs.dma_channels[reg_idx].dma_bufalloc));

      bcm_pr_debug("dma_channels[%lu].dmac_chancfg  = 0x%lx\n",
         (unsigned long)(reg_idx), (unsigned long)(regs.dma_channels[reg_idx].dmac_chancfg));
      bcm_pr_debug("dma_channels[%lu].dmac_ir       = 0x%lx\n",
         (unsigned long)(reg_idx), (unsigned long)(regs.dma_channels[reg_idx].dmac_ir));
      bcm_pr_debug("dma_channels[%lu].dmac_irmask   = 0x%lx\n",
         (unsigned long)(reg_idx), (unsigned long)(regs.dma_channels[reg_idx].dmac_irmask));
      bcm_pr_debug("dma_channels[%lu].dmac_maxburst = 0x%lx\n",
         (unsigned long)(reg_idx), (unsigned long)(regs.dma_channels[reg_idx].dmac_maxburst));

      bcm_pr_debug("dma_channels[%lu].dmas_rstart   = 0x%lx\n",
         (unsigned long)(reg_idx), (unsigned long)(regs.dma_channels[reg_idx].dmas_rstart));
      bcm_pr_debug("dma_channels[%lu].dmas_sram2    = 0x%lx\n",
         (unsigned long)(reg_idx), (unsigned long)(regs.dma_channels[reg_idx].dmas_sram2));
      bcm_pr_debug("dma_channels[%lu].dmas_sram3    = 0x%lx\n",
         (unsigned long)(reg_idx), (unsigned long)(regs.dma_channels[reg_idx].dmas_sram3));
      bcm_pr_debug("dma_channels[%lu].dmas_sram4    = 0x%lx\n",
         (unsigned long)(reg_idx), (unsigned long)(regs.dma_channels[reg_idx].dmas_sram4));
   }
}

static inline void pcm_pr_stats(const pcm_t *t)
{
   bcm_phone_pcm_stats_t stats;
   pcm_read_stats(t, &(stats));

   bcm_pr_debug("======= Stats =======\n");
   bcm_pr_debug("rx_errors            = %lu\n", (unsigned long)(stats.rx_errors));
   bcm_pr_debug("rx_length_errors     = %lu\n", (unsigned long)(stats.rx_length_errors));
   bcm_pr_debug("rx_empty_errors      = %lu\n", (unsigned long)(stats.rx_empty_errors));
   bcm_pr_debug("rx_good              = %lu\n", (unsigned long)(stats.rx_good));
   bcm_pr_debug("rx_bytes             = %lu\n", (unsigned long)(stats.rx_bytes));
   bcm_pr_debug("tx_errors            = %lu\n", (unsigned long)(stats.tx_errors));
   bcm_pr_debug("tx_good              = %lu\n", (unsigned long)(stats.tx_good));
   bcm_pr_debug("tx_bytes             = %lu\n", (unsigned long)(stats.tx_bytes));
   bcm_pr_debug("cnt_irq_rx           = %lu\n", (unsigned long)(stats.cnt_irq_rx));
   bcm_pr_debug("cnt_irq_tx           = %lu\n", (unsigned long)(stats.cnt_irq_tx));
#ifdef BCMPH_TEST_PCM
   bcm_pr_debug("cnt_irq              = %lu\n", (unsigned long)(stats.cnt_irq));
   bcm_pr_debug("cnt_irq_rx_overflow  = %lu\n", (unsigned long)(stats.cnt_irq_rx_overflow));
   bcm_pr_debug("cnt_irq_tx_underflow = %lu\n", (unsigned long)(stats.cnt_irq_tx_underflow));
   bcm_pr_debug("rx_sop               = %lu\n", (unsigned long)(stats.rx_sop));
   bcm_pr_debug("rx_eop               = %lu\n", (unsigned long)(stats.rx_eop));
   bcm_pr_debug("min_size_rx_buffer   = %lu\n", (unsigned long)(stats.min_size_rx_buffer));
   bcm_pr_debug("max_size_rx_buffer   = %lu\n", (unsigned long)(stats.max_size_rx_buffer));
   bcm_pr_debug("min_size_tx_buffer   = %lu\n", (unsigned long)(stats.min_size_tx_buffer));
   bcm_pr_debug("max_size_tx_buffer   = %lu\n", (unsigned long)(stats.max_size_tx_buffer));
#endif // BCMPH_TEST_PCM
}

#ifdef BCMPH_TEST_PCM
extern uint bcm_drv_param_channel_mapping;
#endif // BCMPH_TEST_PCM

void pcm_configure_channels(pcm_t *t,
   bool use_16bits_mode,
   const __u8 *chans_to_ts, size_t chans_to_ts_len)
{
   size_t chan_idx;

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert((!t->is_started) && (!t->dma_is_started));
   bcm_assert((NULL != chans_to_ts) && /* (chans_to_ts_len >= 0) &&*/ (chans_to_ts_len <= ARRAY_SIZE(t->channel_to_timeslot)));

   t->max_frame_size = BCMPH_PCM_MAX_FRAME_SIZE;

   bcm_assert((t->rx_data_buffer_size >= t->max_frame_size)
      && (t->tx_data_buffer_size >= t->max_frame_size));

   t->rx_data_buffer_rounded_size = (t->rx_data_buffer_size / t->max_frame_size);
   t->rx_data_buffer_rounded_size *= t->max_frame_size;
   bcm_assert(pcm_get_max_frame_rx_buffer(t) <= BCMPH_PCM_MAX_FRAME_RX_BUFFER);

   t->tx_data_buffer_rounded_size = (t->tx_data_buffer_size / t->max_frame_size);
   t->tx_data_buffer_rounded_size *= t->max_frame_size;
   bcm_assert(pcm_get_max_frame_tx_buffer(t) <= BCMPH_PCM_MAX_FRAME_TX_BUFFER);

   t->frame_size = 0;

   if (use_16bits_mode) {
      t->timeslot_is_16bits = true;
   }
   else {
      t->timeslot_is_16bits = false;
   }

   memset(t->channel_to_timeslot, BCMPH_TIMESLOT_UNSPECIFIED, sizeof(t->channel_to_timeslot));
   for (chan_idx = 0; (chan_idx < chans_to_ts_len); chan_idx += 1) {
#ifdef BCMPH_TEST_PCM
      size_t shift;
      __u8 chan;
#endif // BCMPH_TEST_PCM
      bcm_assert(BCMPH_TIMESLOT_UNSPECIFIED != chans_to_ts[chan_idx]);
#ifndef BCMPH_TEST_PCM
      t->channel_to_timeslot[chan_idx] = chans_to_ts[chan_idx];
#else // BCMPH_TEST_PCM
      shift = ((BCMPH_PCM_MAX_CHANNELS - 1 - chan_idx) * 4);
      chan = ((bcm_drv_param_channel_mapping >> shift) & 0xF);
      if (chan >= BCMPH_PCM_MAX_CHANNELS) {
         bcm_assert(false);
         chan = BCMPH_PCM_MAX_CHANNELS - 1;
      }
      t->channel_to_timeslot[chan] = chans_to_ts[chan_idx];
#endif // BCMPH_TEST_PCM
   }
#ifndef BCMPH_TEST_PCM
   t->frame_size = chans_to_ts_len * BCMPH_PCM_CHANNEL_WIDTH;
#else // BCMPH_TEST_PCM
   t->frame_size = t->max_frame_size;
#endif // BCMPH_TEST_PCM
}

__u8 pcm_get_channnel_used_for_timeslot(const pcm_t *t, __u8 timeslot)
{
   __u8 ret = BCMPH_PCM_MAX_CHANNELS;
   size_t chan_idx;

   d_bcm_pr_debug("%s(timeslot=%u)\n", __func__, (unsigned int)(timeslot));

   bcm_assert(BCMPH_TIMESLOT_UNSPECIFIED != timeslot);

   for (chan_idx = 0; (chan_idx < ARRAY_SIZE(t->channel_to_timeslot)); chan_idx += 1) {
      if (t->channel_to_timeslot[chan_idx] == timeslot) {
         ret = chan_idx;
         break;
      }
   }

   return (ret);
}

#ifdef BCMPH_TEST_PCM
extern uint bcm_drv_param_pcm_chan_ctrl;
#endif // BCMPH_TEST_PCM

void pcm_start(pcm_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   bcm_assert(!t->is_started);

   t->stats.cnt_irq_rx = 0;
   t->stats.cnt_irq_tx = 0;
#ifdef BCMPH_TEST_PCM
   t->stats.rx_sop = 0;
   t->stats.rx_eop = 0;
   t->stats.min_size_rx_buffer = t->rx_data_buffer_rounded_size;
   t->stats.max_size_rx_buffer = 0;
   t->stats.min_size_tx_buffer = t->tx_data_buffer_rounded_size;
   t->stats.max_size_tx_buffer = 0;
#endif // !BCMPH_TEST_PCM

   pcm_regs_reset(t);

   if (t->frame_size > 0) {
      __u32 reg;
      size_t chan_idx;
      __u32 mask;
#ifdef BCMPH_TEST_PCM
      mask = bcm_drv_param_pcm_chan_ctrl;
#else // !BCMPH_TEST_PCM
      size_t active_chans_count;

      // For each active channel, declare the timeslot to fill in the frame
      mask = 0;
      active_chans_count = t->frame_size / BCMPH_PCM_CHANNEL_WIDTH;
      for (chan_idx = 0; (chan_idx < active_chans_count); chan_idx += 1) {
         mask |= (PCM_RX0_EN << chan_idx);
         // For 6358 the flags PCM_TXx_EN seem to have no effect
         mask |= (PCM_TX0_EN << chan_idx);
      }
#endif // !BCMPH_TEST_PCM

      // Init timeslots
      for (chan_idx = 0; (chan_idx < ARRAY_SIZE(t->channel_to_timeslot)); chan_idx += 1) {
         if (BCMPH_TIMESLOT_UNSPECIFIED != t->channel_to_timeslot[chan_idx]) {
            pcm_timeslot_alloc(t, chan_idx, t->channel_to_timeslot[chan_idx]);
         }
      }

      /* Reset RX descs and forget previously received data */
      pcm_dma_rx_descs_reset(t);

      /* Ensure all writes done before initializing ring adresses */
      wmb();

#if 0
      /* Initialize flow control buffer allocation */
      pcm_dma_writel(t, PCMDMA_BUFALLOC_FORCE_MASK | 0,
         PCMDMA_BUFALLOC_REG(t->rx_chan));
#endif

      /* Set dma maximum burst len for RX */
      pcm_dmac_writel(t, t->dma_maxburst, PCMDMAC_MAXBURST_REG(t->rx_chan));

      /* Write RX ring addresses and clear remaining state ram for RX channel */
#ifndef BCMPH_NOHW
      pcm_dmas_writel(t, t->rx_dma_desc, PCMDMAS_RSTART_REG(t->rx_chan));
#endif // BCMPH_NOHW
      pcm_dmas_writel(t, 0, PCMDMAS_SRAM2_REG(t->rx_chan));
      pcm_dmas_writel(t, 0, PCMDMAS_SRAM3_REG(t->rx_chan));
      pcm_dmas_writel(t, 0, PCMDMAS_SRAM4_REG(t->rx_chan));

      reg = pcm_dmac_readl(t, PCMDMAC_IR_REG(t->rx_chan));
      pcm_dmac_writel(t, reg, PCMDMAC_IR_REG(t->rx_chan));
#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
      /* Watch "buffer transferred" and "pkt transferred" interrupts in RX */
      pcm_dmac_writel(t, PCMDMAC_IR_BUFDONE_MASK | PCMDMAC_IR_PKTDONE_MASK, PCMDMAC_IRMASK_REG(t->rx_chan));
#endif // BCMPH_ENABLE_PCM_INTERRUPTS

#if 0
      /* Set flow control low/high threshold to 1/3 / 2/3 */
      reg = t->rx_ring_size / 3;
      pcm_dma_writel(t, reg, PCMDMA_FLOWCL_REG(t->rx_chan));
      reg = (t->rx_ring_size * 2) / 3;
      pcm_dma_writel(t, reg, PCMDMA_FLOWCH_REG(t->rx_chan));
#endif

      /* Set dma maximum burst len for TX */
      pcm_dmac_writel(t, t->dma_maxburst, PCMDMAC_MAXBURST_REG(t->tx_chan));

      /* Write tx ring addresses and clear remaining state ram for TX channel */
#ifndef BCMPH_NOHW
      pcm_dmas_writel(t, t->tx_dma_desc, PCMDMAS_RSTART_REG(t->tx_chan));
#endif // BCMPH_NOHW
      pcm_dmas_writel(t, 0, PCMDMAS_SRAM2_REG(t->tx_chan));
      pcm_dmas_writel(t, 0, PCMDMAS_SRAM3_REG(t->tx_chan));
      pcm_dmas_writel(t, 0, PCMDMAS_SRAM4_REG(t->tx_chan));

      reg = pcm_dmac_readl(t, PCMDMAC_IR_REG(t->tx_chan));
      pcm_dmac_writel(t, reg, PCMDMAC_IR_REG(t->tx_chan));
#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
      /* Watch "buffer transferred" and "packet transferred" interrupts in TX */
      pcm_dmac_writel(t, PCMDMAC_IR_BUFDONE_MASK | PCMDMAC_IR_PKTDONE_MASK, PCMDMAC_IRMASK_REG(t->tx_chan));
#endif // BCMPH_ENABLE_PCM_INTERRUPTS

#if ((defined BCMPH_TEST_PCM) && (!defined BCMPH_NOHW))
      pcm_pr_regs(t);
#endif // BCMPH_TEST_PCM

      /* Enable PCM IUDMA controller */
      d_bcm_pr_debug("Enabling PCM DMA\n");
      reg = PCMDMA_CFG_EN_MASK;
#if 0
      reg |= PCMDMA_CFG_FLOWCH_MASK(t->rx_chan);
#endif
      pcm_dma_writel(t, reg, PCMDMA_CFG_REG);

      // Enable PCM
      d_bcm_pr_debug("Enabling PCM\n");
      reg = pcm_readl(t, PCM_CTRL_REG);
      reg |= PCM_ENABLE;
      pcm_writel(t, reg, PCM_CTRL_REG);

      // Enable PCM channels
      // It's important to do it AFTER enabling PCM !!!
#ifndef BCMPH_TEST_PCM
      reg = pcm_readl(t, PCM_CHAN_CTRL_REG);
      reg &= (~(PCM_ALL_CHANS_EN));
#else // BCMPH_TEST_PCM
      reg = 0;
#endif // BCMPH_TEST_PCM
      reg |= mask;
      pcm_writel(t, reg, PCM_CHAN_CTRL_REG);
      bcm_pr_debug("PCM_CHAN_CTRL_REG <- 0x%lx\n", (unsigned long)(reg));

      msleep(100);

#ifdef BCMPH_TEST_PCM
#ifndef BCMPH_NOHW
      /* Enable PCM interrupts */
      // If we enable interrupts, the system freezes as if the CPU spend all
      // of its time in the interrupt routine
      //pcm_writel(t, (PCM_TX_UNDERFLOW | PCM_RX_OVERFLOW), PCM_INT_MASK_REG);
#endif // !BCMPH_NOHW
#endif // BCMPH_TEST_PCM

      t->is_started = true;
   }
   else {
      bcm_pr_info("PCM not started because no channels activated\n");

      // Reset DMA descs so pcm_get_size_next_valid_rx_buffer() will returns 0
      pcm_dma_rx_descs_reset(t);
   }
}

void pcm_stop(pcm_t *t)
{
#ifndef BCMPH_NOHW
   size_t loop_count;
#endif // BCMPH_NOHW
   __u32 reg;


   bcm_pr_debug("%s()\n", __func__);

   pcm_dma_stop(t);

   /* Deinit TX side first in case we loopback TX on RX */

   /* Mask interrupts */
   pcm_dmac_writel(t, 0, PCMDMAC_IRMASK_REG(t->tx_chan));
   /* And acknowledged interrupts pending */
   reg = pcm_dmac_readl(t, PCMDMAC_IR_REG(t->tx_chan));
   pcm_dmac_writel(t, reg, PCMDMAC_IR_REG(t->tx_chan));

   bcm_pr_debug("Halting TX DMA\n");
#ifndef BCMPH_NOHW
   for (loop_count = 0; (loop_count < 500); loop_count += 1) {
      /* Set the burstHalt bit while clearing endma bit */
      pcm_dmac_writel(t, PCMDMAC_CHANCFG_BURSTHALT_MASK, PCMDMAC_CHANCFG_REG(t->tx_chan));
      msleep(1);
      if (0 == (pcm_dmac_readl(t, PCMDMAC_CHANCFG_REG(t->tx_chan)) & PCMDMAC_CHANCFG_EN_MASK)) {
         bcm_pr_debug("TX DMA halted\n");
         break;
      }
      bcm_pr_debug("Waiting...\n");
   }
#endif // !BCMPH_NOHW
   pcm_dmac_writel(t, 0, PCMDMAC_CHANCFG_REG(t->tx_chan));

   /* Mask interrupts */
   pcm_dmac_writel(t, 0, PCMDMAC_IRMASK_REG(t->rx_chan));
   /* And acknowledged interrupts pending */
   reg = pcm_dmac_readl(t, PCMDMAC_IR_REG(t->rx_chan));
   pcm_dmac_writel(t, reg, PCMDMAC_IR_REG(t->rx_chan));

   /* Deinit RX side */
   bcm_pr_debug("Halting RX DMA\n");
#ifndef BCMPH_NOHW
   for (loop_count = 0; (loop_count < 500); loop_count += 1) {
      /* Set the burstHalt bit while clearing endma bit */
      pcm_dmac_writel(t, PCMDMAC_CHANCFG_BURSTHALT_MASK, PCMDMAC_CHANCFG_REG(t->rx_chan));
      msleep(1);
      if (0 == (pcm_dmac_readl(t, PCMDMAC_CHANCFG_REG(t->rx_chan)) & PCMDMAC_CHANCFG_EN_MASK)) {
         bcm_pr_debug("RX DMA halted\n");
         break;
      }
      bcm_pr_debug("Waiting...\n");
   }
#endif // !BCMPH_NOHW
   pcm_dmac_writel(t, 0, PCMDMAC_CHANCFG_REG(t->rx_chan));

   /* Disable PCM IUDMA controller */
   pcm_dma_writel(t, 0, PCMDMA_CFG_REG);

   bcm_pr_debug("Disabling all PCM channels\n");
   pcm_writel(t, 0, PCM_CHAN_CTRL_REG);

   d_bcm_pr_debug("Disabling PCM\n");
   reg = pcm_readl(t, PCM_CTRL_REG);
   reg &= (~(PCM_ENABLE));
   pcm_writel(t, reg, PCM_CTRL_REG);

   pcm_regs_reset(t);

   /* Not sure that it is necessary but just in case */
   pcm_writel(t, 0, PCM_CTRL_REG);

   t->is_started = false;

   /* Write RX ring addresses and clear remaining state ram for RX channel */
#ifndef BCMPH_NOHW
   pcm_dmas_writel(t, t->rx_dma_desc, PCMDMAS_RSTART_REG(t->rx_chan));
#endif // BCMPH_NOHW
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM2_REG(t->rx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM3_REG(t->rx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM4_REG(t->rx_chan));

   /* Reset TX descs, so TX buffers can be filled before restarting DMA
    (but we do not reset RX descs so we can read all the data received
    before stopping DMA) */
   pcm_dma_tx_descs_reset(t);

   /* Write TX ring addresses and clear remaining state ram for TX channel */
#ifndef BCMPH_NOHW
   pcm_dmas_writel(t, t->tx_dma_desc, PCMDMAS_RSTART_REG(t->tx_chan));
#endif // BCMPH_NOHW
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM2_REG(t->tx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM3_REG(t->tx_chan));
   pcm_dmas_writel(t, 0, PCMDMAS_SRAM4_REG(t->tx_chan));

   pcm_pr_stats(t);
}

void pcm_read_regs(const pcm_t *t, bcm_phone_pcm_regs_t *regs)
{
   size_t reg_idx;

   memset(regs, 0, sizeof(*regs));

   regs->ctlr = pcm_readl(t, PCM_CTRL_REG);
   regs->chan_ctrl = pcm_readl(t, PCM_CHAN_CTRL_REG);
   regs->int_pending = pcm_readl(t, PCM_INT_PENDING_REG);
   regs->int_mask = pcm_readl(t, PCM_INT_MASK_REG);
   regs->pll_ctrl1 = pcm_readl(t, PCM_PLL_CTRL1_REG);
   regs->pll_ctrl2 = pcm_readl(t, PCM_PLL_CTRL2_REG);
   regs->pll_ctrl3 = pcm_readl(t, PCM_PLL_CTRL3_REG);
   switch (t->board_desc->cpu_desc->cpu_id) {
      case 0x6358: {
         regs->pll_ctrl4 = 0;
         regs->pll_stat = pcm_readl(t, PCM_PLL_STAT_6358_REG);
         break;
      }
      case 0x6368:
      case 0x6362: {
         regs->pll_ctrl4 = pcm_readl(t, PCM_PLL_CTRL4_636x_REG);
         regs->pll_stat = pcm_readl(t, PCM_PLL_STAT_636x_REG);
         break;
      }
      default: {
         regs->pll_ctrl4 = 0;
         regs->pll_stat = 0;
         break;
      }
   }
   for (reg_idx = 0; (reg_idx < ARRAY_SIZE(regs->slot_alloc_tbl)); reg_idx += 1) {
      regs->slot_alloc_tbl[reg_idx] = pcm_readl(t, PCM_SLOT_ALLOC_TBL_REG + (reg_idx << 2));
   }
   regs->dma_cfg = pcm_dma_readl(t, PCMDMA_CFG_REG);
   for (reg_idx = 0; (reg_idx < ARRAY_SIZE(regs->dma_channels)); reg_idx += 1) {
      regs->dma_channels[reg_idx].dma_flowcl = pcm_dma_readl(t, PCMDMA_FLOWCL_REG(reg_idx));
      regs->dma_channels[reg_idx].dma_flowch = pcm_dma_readl(t, PCMDMA_FLOWCH_REG(reg_idx));
      regs->dma_channels[reg_idx].dma_bufalloc = pcm_dma_readl(t, PCMDMA_BUFALLOC_REG(reg_idx));

      regs->dma_channels[reg_idx].dmac_chancfg = pcm_dmac_readl(t, PCMDMAC_CHANCFG_REG(reg_idx));
      regs->dma_channels[reg_idx].dmac_ir = pcm_dmac_readl(t, PCMDMAC_IR_REG(reg_idx));
      regs->dma_channels[reg_idx].dmac_irmask = pcm_dmac_readl(t, PCMDMAC_IRMASK_REG(reg_idx));
      regs->dma_channels[reg_idx].dmac_maxburst = pcm_dmac_readl(t, PCMDMAC_MAXBURST_REG(reg_idx));

      regs->dma_channels[reg_idx].dmas_rstart = pcm_dmas_readl(t, PCMDMAS_RSTART_REG(reg_idx));
      regs->dma_channels[reg_idx].dmas_sram2 = pcm_dmas_readl(t, PCMDMAS_SRAM2_REG(reg_idx));
      regs->dma_channels[reg_idx].dmas_sram3 = pcm_dmas_readl(t, PCMDMAS_SRAM3_REG(reg_idx));
      regs->dma_channels[reg_idx].dmas_sram4 = pcm_dmas_readl(t, PCMDMAS_SRAM4_REG(reg_idx));
   }
}

void pcm_read_stats(const pcm_t *t, bcm_phone_pcm_stats_t *stats)
{
   memcpy(stats, &(t->stats), sizeof(*stats));
}

int __init pcm_init(pcm_t *t,
#ifdef __KERNEL__
   struct device *dev,
#endif // __KERNEL__
   const board_desc_t *board_desc,
   void (*rx_desc_returned_cb)(pcm_t *t),
   void (*tx_desc_empty_cb)(pcm_t *t))
{
   int ret = -1;
#ifndef BCMPH_NOHW
   __u32 val;
#endif // !BCMPH_NOHW

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert((8192 == board_desc->phone_desc->clk_rate)
      || (4096 == board_desc->phone_desc->clk_rate)
      || (2048 == board_desc->phone_desc->clk_rate)
      || (1024 == board_desc->phone_desc->clk_rate));

#ifndef BCMPH_NOHW
   if (0x6358 == board_desc->cpu_desc->cpu_id) {
      // In endpointdd.ko for a 6358 SoC, they set a value of 4
      // I think it means 4 words of 32 bits
      // We set it to a multiple of a frame size
      t->dma_maxburst = BCMPH_PCM_MAX_FRAME_SIZE / 4;
   }
   else {
      // For a 6368 or 6362 a word is 64 bits wide ?
      t->dma_maxburst = BCMPH_PCM_MAX_FRAME_SIZE / 8;
   }
#else // BCMPH_NOHW
   bcm_period_init(&(t->timestamp));
   t->dma_maxburst = PCMDMAC_MAXBURST_SIZE;
#endif // BCMPH_NOHW

   t->max_frame_size = BCMPH_PCM_MAX_FRAME_SIZE;

   /*
    If the signal FS of PCM bus is clocked at BCMPH_SAMPLES_PER_MS kHz
    we get BCMPH_SAMPLES_PER_MS samples of a timeslot per millisecond.
    A channel can contain BCMPH_PCM_CHANNEL_WIDTH samples in 8 bits mode
    or BCMPH_PCM_CHANNEL_WIDTH / 2 samples in 16 bits mode
    So to hold 1 ms of samples we need at most a buffer of
    BCMPH_PCM_MAX_CHANNELS * (BCMPH_PCM_CHANNEL_WIDTH / 2) * BCMPH_SAMPLES_PER_MS
    As BCMPH_PCM_MAX_CHANNELS * BCMPH_PCM_CHANNEL_WIDTH == BCMPH_PCM_MAX_FRAME_SIZE
    we can rewrite to (BCMPH_PCM_MAX_FRAME_SIZE * BCMPH_SAMPLES_PER_MS) / 2
   */

#ifndef BCMPH_NOHW
   // We choose a buffer size allowing reception of (board_desc->phone_desc->tick_period / 5) ms of data in 16 bits mode
   t->rx_data_buffer_size = ((BCMPH_PCM_MAX_FRAME_SIZE * BCMPH_SAMPLES_PER_MS * board_desc->phone_desc->tick_period) + ((2 * 5) - 1)) / (2 * 5);
#else // BCMPH_NOHW
   // We choose a buffer size allowing reception of (board_desc->phone_desc->tick_period / 10) ms of data
   t->rx_data_buffer_size = ((BCMPH_PCM_MAX_FRAME_SIZE * BCMPH_SAMPLES_PER_MS * board_desc->phone_desc->tick_period) + ((2 * 10) - 1)) / (2 * 10);
#endif // BCMPH_NOHW
   // We round up to a multiple of BCMPH_PCM_MAX_FRAME_SIZE.
   t->rx_data_buffer_size = round_up_generic(t->rx_data_buffer_size, BCMPH_PCM_MAX_FRAME_SIZE);
   // We also want that the multiple is a power of 2
   t->rx_data_buffer_size = round_up_to_next_pow_of_2(t->rx_data_buffer_size / BCMPH_PCM_MAX_FRAME_SIZE) * BCMPH_PCM_MAX_FRAME_SIZE;
   if (t->rx_data_buffer_size > 4095) {
      bcm_pr_err("RX data buffer size (%lu) must be less than 4096\n",
         (unsigned long)(t->rx_data_buffer_size));
      ret = -1;
      goto fail_size;
   }
   if (t->rx_data_buffer_size != round_up_to_pow_of_2(t->rx_data_buffer_size, board_desc->cpu_desc->dcache_line_size)) {
      bcm_pr_err("RX data buffer size (%lu) is not a multiple a cache line size (%lu)\n",
         (unsigned long)(t->rx_data_buffer_size), (unsigned long)(board_desc->cpu_desc->dcache_line_size));
      ret = -1;
      goto fail_size;
   }
   t->rx_data_buffer_rounded_size = t->rx_data_buffer_size;
   if (pcm_get_max_frame_rx_buffer(t) > BCMPH_PCM_MAX_FRAME_RX_BUFFER) {
      bcm_pr_err("RX data buffer size (%lu) is too big. Can contain more than %lu frames\n",
         (unsigned long)(t->rx_data_buffer_size), (unsigned long)(BCMPH_PCM_MAX_FRAME_RX_BUFFER));
      ret = -1;
      goto fail_size;
   }
   // We want to have a number of buffer holding at most 30 ms of data
   t->rx_ring_size = ((BCMPH_PCM_MAX_FRAME_SIZE * BCMPH_SAMPLES_PER_MS * 30) + ((t->rx_data_buffer_size * 2) - 1)) / (t->rx_data_buffer_size * 2);
   // rx_ring_size must be greater than 1 else rx_desc_returned_cb
   // will be called too often
   if (t->rx_ring_size <= 1) {
      t->rx_ring_size = 2;
   }

   bcm_pr_debug("Using %lu buffers of %lu bytes for RX\n",
      (unsigned long)(t->rx_ring_size),
      (unsigned long)(t->rx_data_buffer_size));

   // We choose the same buffer size as for RX */
   t->tx_data_buffer_size = t->rx_data_buffer_size;
   // We round up to a multiple of BCMPH_PCM_MAX_FRAME_SIZE.
   t->tx_data_buffer_size = round_up_generic(t->tx_data_buffer_size, BCMPH_PCM_MAX_FRAME_SIZE);
   // We also want that the multiple is a power of 2
   t->tx_data_buffer_size = round_up_to_next_pow_of_2(t->tx_data_buffer_size / BCMPH_PCM_MAX_FRAME_SIZE) * BCMPH_PCM_MAX_FRAME_SIZE;
   if (t->tx_data_buffer_size > 4095) {
      bcm_pr_err("TX data buffer size (%lu) must be less than 4096\n",
         (unsigned long)(t->tx_data_buffer_size));
      ret = -1;
      goto fail_size;
   }
   if (t->tx_data_buffer_size != round_up_to_pow_of_2(t->tx_data_buffer_size, board_desc->cpu_desc->dcache_line_size)) {
      bcm_pr_err("TX data buffer size (%lu) is not a multiple a cache line size (%lu)\n",
         (unsigned long)(t->tx_data_buffer_size), (unsigned long)(board_desc->cpu_desc->dcache_line_size));
      ret = -1;
      goto fail_size;
   }
   t->tx_data_buffer_rounded_size = t->tx_data_buffer_size;
   if (pcm_get_max_frame_tx_buffer(t) > BCMPH_PCM_MAX_FRAME_TX_BUFFER) {
      bcm_pr_err("TX data buffer size (%lu) is too big. Can contain more than %lu frames\n",
         (unsigned long)(t->tx_data_buffer_size), (unsigned long)(BCMPH_PCM_MAX_FRAME_TX_BUFFER));
      ret = -1;
      goto fail_size;
   }
#ifndef BCMPH_NOHW
   // We choose the same ring size as for RX */
   t->tx_ring_size = t->rx_ring_size;
#else // BCMPH_NOHW
   // We want to have a number of buffer holding at most 20 ms of data
   t->tx_ring_size = ((BCMPH_PCM_MAX_CHANNELS * 2 * BCMPH_SAMPLES_PER_MS * 20) + t->tx_data_buffer_size - 1) / t->tx_data_buffer_size;
#endif // BCMPH_NOHW
   // tx_ring_size must be greater than 1 else tx_desc_empty_cb
   // will be called too often
   if (t->tx_ring_size <= 1) {
      t->tx_ring_size = 2;
   }

   bcm_pr_debug("Using %lu buffers of %lu bytes for TX\n",
      (unsigned long)(t->tx_ring_size),
      (unsigned long)(t->tx_data_buffer_size));

#ifndef BCMPH_NOHW
   ret = pcm_get_shared_res(board_desc);
   if (ret) {
      goto fail_res;
   }
#else // BCMPH_NOHW
   memset(io_mem, 0, sizeof(io_mem));
#endif // BCMPH_NOHW

#ifdef __KERNEL__
   t->dev = dev;
#endif // __KERNEL__
   t->board_desc = board_desc;
   bcm_assert(NULL != rx_desc_returned_cb);
   t->rx_desc_returned_cb = rx_desc_returned_cb;
   bcm_assert(NULL != tx_desc_empty_cb);
   t->tx_desc_empty_cb = tx_desc_empty_cb;
#ifndef BCMPH_NOHW
   t->base = pcm_shared_res.pcm_mem.base;
   t->dma_base = pcm_shared_res.pcm_dma_mem.base;
   t->dmac_base = pcm_shared_res.pcm_dmac_mem.base;
   t->dmas_base = pcm_shared_res.pcm_dmas_mem.base;
#endif // !BCMPH_NOHW

#ifdef BCMPH_ENABLE_PCM_INTERRUPTS
   t->irq_dma_rx = -1;
   t->irq_dma_tx = -1;
#endif // !BCMPH_ENABLE_PCM_INTERRUPTS
   t->rx_chan = 0;
   t->tx_chan = 1;

#ifndef BCMPH_NOHW
   // Enable clock to pcm block
   d_bcm_pr_debug("Enabling PCM clock\n");
   clk_prepare_enable(t->board_desc->pcm_desc->clk);

   msleep(10);

   // Soft reset the PCM block
   d_bcm_pr_debug("Resetting PCM block\n");
   val = bcm_perf_readl(t->board_desc->cpu_desc->off_perf_softreset_reg);
   val &= (~(t->board_desc->pcm_desc->softreset_mask));
   bcm_perf_writel(val, t->board_desc->cpu_desc->off_perf_softreset_reg);

   // Sleep briefly to recover from the reset
   msleep(50);

   // Clear the reset of the PCM block
   val |= t->board_desc->pcm_desc->softreset_mask;
   bcm_perf_writel(val, t->board_desc->cpu_desc->off_perf_softreset_reg);

   msleep(10);
#endif // !BCMPH_NOHW

   // Initialize PCM PLL clock
   ret = (*(t->board_desc->pcm_desc->pll_init))(t);
   if (ret) {
      goto fail_pll;
   }

   // Initialize registers
   ret = pcm_regs_init(t);
   if (ret) {
      goto fail_regs;
   }

   // Initialize DMA
   ret = pcm_dma_init(t);
   if (ret) {
      goto fail_dma;
   }

   return (0);

   pcm_dma_deinit(t);
fail_dma:
   pcm_regs_deinit(t);
fail_regs:
   (*(t->board_desc->pcm_desc->pll_deinit))(t);
fail_pll:
#ifndef BCMPH_NOHW
   d_bcm_pr_debug("Disabling PCM clock\n");
   clk_disable_unprepare(t->board_desc->pcm_desc->clk);
   pcm_release_shared_res();
fail_res:
#endif // !BCMPH_NOHW
fail_size:
   return (ret);
}

void pcm_deinit(pcm_t *t)
{
#ifndef BCMPH_NOHW
   __u32 val;
#endif // !BCMPH_NOHW

   bcm_pr_debug("%s()\n", __func__);

   pcm_stop(t);

   pcm_dma_deinit(t);

   pcm_regs_deinit(t);

   (*(t->board_desc->pcm_desc->pll_deinit))(t);

#ifndef BCMPH_NOHW
   // Soft reset the PCM block
   d_bcm_pr_debug("Resetting PCM block\n");
   val = bcm_perf_readl(t->board_desc->cpu_desc->off_perf_softreset_reg);
   val &= (~(t->board_desc->pcm_desc->softreset_mask));
   bcm_perf_writel(val, t->board_desc->cpu_desc->off_perf_softreset_reg);

   // Sleep briefly to recover from the reset
   msleep(50);

   // Clear the reset of the PCM block
   val |= t->board_desc->pcm_desc->softreset_mask;
   bcm_perf_writel(val, t->board_desc->cpu_desc->off_perf_softreset_reg);

   msleep(10);

   d_bcm_pr_debug("Disabling PCM clock\n");
   clk_disable_unprepare(t->board_desc->pcm_desc->clk);

   pcm_release_shared_res();
#endif // !BCMPH_NOHW
}
