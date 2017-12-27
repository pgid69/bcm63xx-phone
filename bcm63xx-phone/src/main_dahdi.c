/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <config.h>

#ifdef BCMPH_DAHDI_DRIVER

#include <linux/atomic.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/stringify.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <bcm63xx_phone.h>
#include <bcm63xx_log.h>
#include <bcm63xx_ring_buf.h>
#include <timer.h>

#include "main.h"

// Include after system files
#include <compile.h>

#define BCMPH_DAHDI_CODEC_ULAW 1
#define BCMPH_DAHDI_CODEC_ALAW 2
#define BCMPH_DAHDI_CODEC_LINEAR 3

/*
 If we change definition of BCMPH_DAHDI_CODEC, do not forget to update
 bcm_phone_mgr_devs_init()
*/
#ifndef BCMPH_NOHW
#define BCMPH_DAHDI_CODEC BCMPH_DAHDI_CODEC_ALAW
#else /* BCMPH_NOHW */
#define BCMPH_DAHDI_CODEC BCMPH_DAHDI_CODEC_LINEAR
#endif /* BCMPH_NOHW */

#if (BCMPH_DAHDI_CODEC_ULAW == BCMPH_DAHDI_CODEC)
#  define BCMPH_CODEC BCMPH_CODEC_ULAW
typedef __u8 bcm_sample_t;
#  define DAHDI_CODEC DAHDI_LAW_MULAW
#  define DAHDI_SAMPLE_TO_BCMPH_SAMPLE(a) (a)
#  define BCMPH_SAMPLE_TO_DAHDI_SAMPLE(a) (a)
#  define BCMPH_SAMPLE_NULL_VALUE DAHDI_LIN2MU(0)
#elif (BCMPH_DAHDI_CODEC_ALAW == BCMPH_DAHDI_CODEC)
#  define BCMPH_CODEC BCMPH_CODEC_ALAW
typedef __u8 bcm_sample_t;
#  define DAHDI_CODEC DAHDI_LAW_ALAW
#  define DAHDI_SAMPLE_TO_BCMPH_SAMPLE(a) (a)
#  define BCMPH_SAMPLE_TO_DAHDI_SAMPLE(a) (a)
#  define BCMPH_SAMPLE_NULL_VALUE DAHDI_LIN2A(0)
#elif (BCMPH_DAHDI_CODEC_LINEAR == BCMPH_DAHDI_CODEC)
#  define BCMPH_CODEC BCMPH_CODEC_LINEAR
typedef __s16 bcm_sample_t;
#  define DAHDI_CODEC DAHDI_LAW_ALAW
#  if (DAHDI_LAW_ALAW == DAHDI_CODEC)
#    define DAHDI_SAMPLE_TO_BCMPH_SAMPLE DAHDI_ALAW
#    define BCMPH_SAMPLE_TO_DAHDI_SAMPLE DAHDI_LIN2A
#  elif (DAHDI_LAW_MULAW == DAHDI_CODEC)
#    define DAHDI_SAMPLE_TO_BCMPH_SAMPLE DAHDI_MULAW
#    define BCMPH_SAMPLE_TO_DAHDI_SAMPLE DAHDI_LIN2MU
#  else
#    error "DAHDI codec must be DAHDI_LAW_MULAW or DAHDI_LAW_ALAW"
#  endif
#  define BCMPH_SAMPLE_NULL_VALUE 0
#else
#  error "BCMPH_DAHDI_CODEC must be BCMPH_DAHDI_CODEC_ULAW, BCMPH_DAHDI_CODEC_ALAW or BCMPH_DAHDI_CODEC_LINEAR"
#endif

static bcm_sample_t bcm_dahdi_chunk_null_bcm_sample[DAHDI_CHUNKSIZE];
static u_char bcm_dahdi_chunk_null_dahdi_sample[DAHDI_CHUNKSIZE];

static void bcm_dahdi_init_null_samples(void)
{
   size_t chunk_idx;
   bcm_sample_t bcm = BCMPH_SAMPLE_NULL_VALUE;
   u_char dahdi = BCMPH_SAMPLE_TO_DAHDI_SAMPLE(bcm);

   bcm_assert((ARRAY_SIZE(bcm_dahdi_chunk_null_bcm_sample) >= DAHDI_CHUNKSIZE)
      && (ARRAY_SIZE(bcm_dahdi_chunk_null_dahdi_sample) >= DAHDI_CHUNKSIZE));

   for (chunk_idx = 0; (chunk_idx < DAHDI_CHUNKSIZE); chunk_idx += 1) {
      bcm_dahdi_chunk_null_bcm_sample[chunk_idx] = bcm;
      bcm_dahdi_chunk_null_dahdi_sample[chunk_idx] = dahdi;
   }
}

/*
Notes on DAHDI internals

State | txstate    | txsig
------|------------|----------------
A     | ONHOOK     | ONHOOK
B     | OFFHOOK    | OFFHOOK
C     | START      | START
D     | AFTERTART  | OFFHOOK
E     | PREWINK    | ONHOOK
F     | WINK       | OFFHOOK
G     | PREFLASH   | OFFHOOK
H     | FLASH      | ONHOOK
I     | DEBOUNCE   | OFFHOOK
J     | RINGON     | START (pour ligne FXS uniquement)
K     | RINGOFF    | OFFHOOK
L     | KEWL       | KEWL (pour ligne FXS uniquement)
M     | AFTERKEWL  | ONHOOK
N     | PULSEBREAK | ONHOOK
O     | PULSEMAKE  | OFFHOOK
P     | PULSEAFTER | OFFHOOK

Transitions

ioctl(DAHDI_HOOK, DAHDI_OFFHOOK) && ((txstate != KEWL) && (txstate != AFTERKEWL)) - I - B

For FXS with Kewlstart signaling : ioctl(DAHDI_HOOK, DAHDI_ONHOOK) - L - M - A

For FXO : ioctl(DAHDI_HOOK, DAHDI_ONHOOK) - A

ioctl(DAHDI_HOOK, DAHDI_RINGOFF) - A

ioctl(DAHDI_HOOK, DAHDI_FLASH) && (txstate == OFFHOOK) - G - H - B

ioctl(DAHDI_HOOK, DAHDI_WINK) && (txstate == ONHOOK) - E - F - B

For FXS : ioctl(DAHDI_HOOK, DAHDI_RING ou DAHDI_START) - J - K - J - ....

For FXO : ioctl(DAHDI_HOOK, DAHDI_RING ou DAHDI_START) - C - D - B

do_dtmf() && (txhooksig == OFFHOOK) - N - O - N - O - ... - P - B


For FXS : hooksig(RXSIG_OFFHOOK) && (txstate == START) - D - B (but i don't know how an FXS line can be in this state)
For FXS : hooksig(RXSIG_OFFHOOK) ... event RINGOFFHOOK

For FXS : hooksig(RXSIG_ONHOOK)

For FXO : hooksig(RXSIG_RING) ... event RINGBEGIN

For FXO : hooksig(RXSIG_OFFHOOK) ... event RINGOFFOOK

For FXO : hooksig(RXSIG_ONHOOK) ... event ONHOOK

--------------------------------------------------------------------------------

We map rxsig the following way:

- DAHDI_RXSIG_ONHOOK : used to signal to DAHDI that the loop is opened
- DAHDI_RXSIG_OFFHOOK : used to signal to DAHDI that the loop is closed
- DAHDI_RXSIG_START : not used
- DAHDI_RXSIG_RING : for an FXO line, used to signal to DAHDI that a ring is detected
- DAHDI_RXSIG_INITIAL : not used
*/

// TEMP
//#define SHORTCUT_TX

#ifdef SHORTCUT_TX
#  if (BCMPH_DAHDI_CODEC == BCMPH_DAHDI_CODEC_ULAW)
#    include <demo_thanks_mulaw.dat>
static const __u8 *demo_thanks = demo_thanks_mulaw;
static size_t demo_thanks_len = ARRAY_SIZE(demo_thanks_mulaw);
#  elif (BCMPH_DAHDI_CODEC == BCMPH_DAHDI_CODEC_ALAW)
#    include <demo_thanks_alaw.dat>
static const __u8 *demo_thanks = demo_thanks_alaw;
static size_t demo_thanks_len = ARRAY_SIZE(demo_thanks_alaw);
#  elif (BCMPH_DAHDI_CODEC == BCMPH_DAHDI_CODEC_LINEAR)
#    include <demo_thanks_sln.dat>
static const __u8 *demo_thanks = demo_thanks_sln;
static size_t demo_thanks_len = ARRAY_SIZE(demo_thanks_sln);
# else
#    error "Bad codec "
#  endif
#endif /* SHORTCUT_TX */

static bool bcm_reverse_polarity = false;
module_param_named(reverse_polarity, bcm_reverse_polarity, bool, 0600);

static char *bcm_country = NULL;
module_param_named(country, bcm_country, charp, 0600);

void bcm_drv_resume_read_cb(pcm_t *pcm)
{
   // Nothing to do
}

void bcm_drv_resume_write_cb(pcm_t *pcm)
{
   // Nothing to do
}

#ifdef SHORTCUT_TX
static inline void bcm_dahdi_fill_tx_ring_buf(
   bcm_dahdi_drv_phone_line_t *line, bcm_ring_buf_t *rb,
   size_t sample_count)
{
   size_t left = sample_count;
   size_t to_add;

   while (left > 0) {
      to_add = left * sizeof(bcm_sample_t);
      if ((line->idx_demo_thanks + to_add) > demo_thanks_len) {
         to_add = demo_thanks_len - line->idx_demo_thanks;
         if (to_add <= 0) {
            bcm_assert(line->idx_demo_thanks > 0);
            line->idx_demo_thanks = 0;
            continue;
         }
      }
      bcm_ring_buf_add(rb, &(demo_thanks[line->idx_demo_thanks]), to_add);
      left -= (to_add / sizeof(bcm_sample_t));
      line->idx_demo_thanks += to_add;
   }
}
#endif /* SHORTCUT_TX */

static bcm_ring_buf_t *bcm_dahdi_get_line_rx_buf(bcm_drv_t *t, size_t line)
{
   bcm_assert(/*(line >= 0) && */(line < t->dahdi.line_count)
      && (line < bcm_phone_mgr_get_line_count(&(t->core.phone_mgr)))
      && (bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line)));
   return (&(t->core.phone_lines[line].rx_ring_buf));
}

static bcm_ring_buf_t *bcm_dahdi_get_line_tx_buf(bcm_drv_t *t, size_t line)
{
   bcm_assert(/*(line >= 0) && */(line < t->dahdi.line_count)
      && (line < bcm_phone_mgr_get_line_count(&(t->core.phone_mgr)))
      && (bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line)));
   return (&(t->core.phone_lines[line].tx_ring_buf));
}

/*
 * Transfer data from DAHDI into ring buffer
 */
static inline void bcm_pull_dahdi_chunk(
   bcm_dahdi_drv_phone_line_t *line, bcm_ring_buf_t *rb)
{
#ifndef SHORTCUT_TX
   struct dahdi_chan *chan = &(line->d_chan);
   size_t len;
   __u8 *ptr;
#endif /* !SHORTCUT_TX */

   /* Move data from chan->writechunk to line->tx_ring_buf */
#ifdef SHORTCUT_TX
   if (bcm_ring_buf_get_free_space(rb) >= (DAHDI_CHUNKSIZE * sizeof(bcm_sample_t))) {
      bcm_dahdi_fill_tx_ring_buf(line, rb, DAHDI_CHUNKSIZE);
# ifdef BCMPH_DEBUG
      line->stats.bytes_rx_from_dahdi += (DAHDI_CHUNKSIZE * sizeof(bcm_sample_t));
# endif /* BCMPH_DEBUG */
   }
#else /* !SHORTCUT_TX */

   ptr = bcm_ring_buf_get_ptr_store(rb, &(len));
   if (len >= (DAHDI_CHUNKSIZE * sizeof(bcm_sample_t))) {
      if (sizeof(bcm_sample_t) > 1) {
         size_t chunk_idx;
         bcm_sample_t *tmp = (bcm_sample_t *)(ptr);
         for (chunk_idx = 0; (chunk_idx < DAHDI_CHUNKSIZE); chunk_idx += 1, tmp += 1) {
            *tmp = DAHDI_SAMPLE_TO_BCMPH_SAMPLE(chan->writechunk[chunk_idx]);
         }
      }
      else {
         memcpy(ptr, chan->writechunk, DAHDI_CHUNKSIZE * sizeof(bcm_sample_t));
      }
      bcm_ring_buf_add_len(rb, DAHDI_CHUNKSIZE * sizeof(bcm_sample_t));
# ifdef BCMPH_DEBUG
      line->stats.bytes_rx_from_dahdi += (DAHDI_CHUNKSIZE * sizeof(bcm_sample_t));
# endif /* BCMPH_DEBUG */
   }
   else if (bcm_ring_buf_get_free_space(rb) >= (DAHDI_CHUNKSIZE * sizeof(bcm_sample_t))) {
      if (sizeof(bcm_sample_t) > 1) {
         size_t chunk_idx;
         bcm_sample_t tmp[DAHDI_CHUNKSIZE];
         for (chunk_idx = 0; (chunk_idx < DAHDI_CHUNKSIZE); chunk_idx += 1) {
            tmp[chunk_idx] = DAHDI_SAMPLE_TO_BCMPH_SAMPLE(chan->writechunk[chunk_idx]);
         }
         bcm_ring_buf_add(rb, (const __u8 *)(tmp), DAHDI_CHUNKSIZE * sizeof(bcm_sample_t));
      }
      else {
         bcm_ring_buf_add(rb, chan->writechunk, DAHDI_CHUNKSIZE * sizeof(bcm_sample_t));
      }
# ifdef BCMPH_DEBUG
      line->stats.bytes_rx_from_dahdi += (DAHDI_CHUNKSIZE * sizeof(bcm_sample_t));
# endif /* BCMPH_DEBUG */
   }
#endif /* !SHORTCUT_TX */
   else {
#ifdef BCMPH_DEBUG
      line->stats.err_tx_ring_buf_full += 1;
#endif /* BCMPH_DEBUG */
      dd_bcm_dev_warn(line->dev, "%s: no more space in the TX ring buffer\n", __func__);
   }
}

/*
 * Transfer data from ring buffer into DAHDI
 */
static inline void bcm_push_dahdi_chunk(
   bcm_dahdi_drv_phone_line_t *line, bcm_ring_buf_t *rb)
{
   struct dahdi_chan *chan = &(line->d_chan);
   const __u8 *ptr;
   size_t len;

   /* Move data from line->rx_ring_buf to chan->readchunk */
   ptr = bcm_ring_buf_get_ptr_load(rb, 0, &(len));
   if (len >= (DAHDI_CHUNKSIZE * sizeof(bcm_sample_t))) {
      if (sizeof(bcm_sample_t) > 1) {
         const bcm_sample_t *tmp = (const bcm_sample_t *)(ptr);
         size_t chunk_idx;
         for (chunk_idx = 0; (chunk_idx < DAHDI_CHUNKSIZE); chunk_idx += 1, tmp += 1) {
            chan->readchunk[chunk_idx] = BCMPH_SAMPLE_TO_DAHDI_SAMPLE(*tmp);
         }
      }
      else {
         memcpy(chan->readchunk, ptr, DAHDI_CHUNKSIZE * sizeof(bcm_sample_t));
      }
      bcm_ring_buf_remove_len(rb, DAHDI_CHUNKSIZE * sizeof(bcm_sample_t));
#ifdef BCMPH_DEBUG
      line->stats.bytes_tx_to_dahdi += (DAHDI_CHUNKSIZE * sizeof(bcm_sample_t));
#endif /* BCMPH_DEBUG */
   }
   else if (bcm_ring_buf_get_size(rb) >= (DAHDI_CHUNKSIZE * sizeof(bcm_sample_t))) {
      if (sizeof(bcm_sample_t) > 1) {
         size_t chunk_idx;
         bcm_sample_t tmp[DAHDI_CHUNKSIZE];
         bcm_ring_buf_remove(rb, (__u8 *)(tmp), DAHDI_CHUNKSIZE * sizeof(bcm_sample_t));
         for (chunk_idx = 0; (chunk_idx < DAHDI_CHUNKSIZE); chunk_idx += 1) {
            chan->readchunk[chunk_idx] = BCMPH_SAMPLE_TO_DAHDI_SAMPLE(tmp[chunk_idx]);
         }
      }
      else {
         bcm_ring_buf_remove(rb, chan->readchunk, DAHDI_CHUNKSIZE * sizeof(bcm_sample_t));
      }
#ifdef BCMPH_DEBUG
      line->stats.bytes_tx_to_dahdi += (DAHDI_CHUNKSIZE * sizeof(bcm_sample_t));
#endif /* BCMPH_DEBUG */
   }
   else {
#ifdef BCMPH_DEBUG
      line->stats.err_rx_ring_buf_empty += 1;
#endif /* BCMPH_DEBUG */
      dd_bcm_dev_warn(line->dev, "%s: not enough data in the RX buffer\n", __func__);
      // We push null value to dahdi
      memcpy(chan->readchunk, bcm_dahdi_chunk_null_dahdi_sample, DAHDI_CHUNKSIZE);
   }
}

static inline void bcm_dahdi_tick(bcm_drv_t *t)
{
   size_t line_idx;

   /*
    Ask DAHDI to update each chan->writechunk.
    Must be called even if tdm is stopped for all lines as DAHDI
    also makes some periodic tasks.
   */
   dahdi_transmit(&(t->dahdi.d_span));

   /* Pull data from chan->writechunk and push data to chan->readchunk */
   for (line_idx = 0; (line_idx < t->dahdi.line_count); line_idx += 1) {
      bcm_dahdi_drv_phone_line_t *line = &(t->dahdi.lines[line_idx]);
      if (line->pcm_started) {
         bcm_pull_dahdi_chunk(line, bcm_dahdi_get_line_tx_buf(t, line_idx));
         bcm_push_dahdi_chunk(line, bcm_dahdi_get_line_rx_buf(t, line_idx));
      }
      else {
         // We push null value to dahdi
         memcpy(line->d_chan.readchunk, bcm_dahdi_chunk_null_dahdi_sample, DAHDI_CHUNKSIZE);
      }
   }

   /*
    Ask DAHDI to process each chan->readchunk
    Must be called even if tdm is stopped for all lines as DAHDI
    also makes some periodic tasks.
   */
   dahdi_receive(&(t->dahdi.d_span));
   dahdi_ec_span(&(t->dahdi.d_span));
}

static void bcm_dahdi_start_pcm(bcm_drv_t *t, size_t line)
{
   bcm_dahdi_drv_phone_line_t *dahdi_line;

   d_bcm_pr_debug("%s()\n", __func__);

   bcm_assert(/*(line >= 0) && */(line < t->dahdi.line_count));

   dahdi_line = &(t->dahdi.lines[line]);
   if (!dahdi_line->pcm_started) {
      size_t line_idx;
      bool start_pcm;

      /* We must empty the ring buffers */
      bcm_drv_forget_rx_data(t, line);
      bcm_drv_forget_tx_data(t, line);

      start_pcm = true;
      for (line_idx = 0; (line_idx < t->dahdi.line_count); line_idx += 1) {
         if (t->dahdi.lines[line_idx].pcm_started) {
            /* PCM already started */
            start_pcm = false;
            break;
         }
      }

      dahdi_line->pcm_started = true;

      if (start_pcm) {
         bcm_drv_start_pcm(t);
      }
   }
}

static void bcm_dahdi_stop_pcm(bcm_drv_t *t, size_t line)
{
   bcm_dahdi_drv_phone_line_t *dahdi_line;

   d_bcm_pr_debug("%s()\n", __func__);

   bcm_assert(/*(line >= 0) && */(line < t->dahdi.line_count));

   dahdi_line = &(t->dahdi.lines[line]);
   if (dahdi_line->pcm_started) {
      size_t line_idx;
      bool stop_pcm;

      dahdi_line->pcm_started = false;

      stop_pcm = true;
      for (line_idx = 0; (line_idx < t->dahdi.line_count); line_idx += 1) {
         if (t->dahdi.lines[line_idx].pcm_started) {
            /* PCM can't be stopped */
            stop_pcm = false;
            break;
         }
      }

      if (stop_pcm) {
         /*
         Here we try to stop PCM but it may fails if one line
         is not yet in the correct mode
         So we will also try to stop PCM in bcm_make_dahdi_work()
         */
         bcm_drv_stop_pcm(t, false);
      }
   }
}

/* Must be called with spinlock 'line->lock' locked */
static inline void bcm_compute_fxs_reverse_polarity(bcm_dahdi_drv_phone_line_t *line)
{
   line->fxs_data.computed_rev_polarity = ((bcm_reverse_polarity)
      ^ ((line->fxs_data.reverse_polarity) && (line->fxs_data.vmwi_info.vmwi_type & DAHDI_VMWI_LREV))
      ^ (line->fxs_data.message_waiting));
}

static int bcm_dahdi_set_line_mode(bcm_drv_t *t, size_t line,
   bcm_phone_line_mode_t mode, int reverse_polarity)
{
   int ret;
   bool need_pcm;
   bool pcm_started;

   bcm_pr_debug("%s(line=%lu, mode=%d, reverse_polarity=%d)\n", __func__,
      (unsigned long)(line), (int)(mode), (int)(reverse_polarity));

   bcm_assert(/*(line >= 0) && */(line < t->dahdi.line_count));

   pcm_started = t->dahdi.lines[line].pcm_started;
   if ((BCMPH_MODE_OFF_TALKING == mode) || (BCMPH_MODE_ON_TALKING == mode)) {
      need_pcm = true;
      if (!pcm_started) {
         bcm_dahdi_start_pcm(t, line);
      }
   }
   else {
      need_pcm = false;
   }
   ret = bcm_drv_set_line_mode(t, line, mode, reverse_polarity,
      bcm_phone_line_tone_code_index(BCMPH_TONE_NONE));
   if (ret) {
      bcm_dev_err(t->dahdi.lines[line].dev, "bcm_drv_set_line_mode(%d) failed (%d)\n",
         (int)(mode), (int)(ret));
      if ((need_pcm) && (!pcm_started)) {
         bcm_dahdi_stop_pcm(t, line);
      }
   }
   else {
      if ((!need_pcm) && (pcm_started)) {
         bcm_dahdi_stop_pcm(t, line);
      }
   }
   return (ret);
}

/*
  Handles change of rxsig due to VP-API events,
  change of txsig received from DAHDI, and update line state according
  to txsig an rxsig
*/
static inline void bcm_handle_dahdi_sigs(bcm_drv_t *t, int tick_count)
{
   size_t line_idx;

   for (line_idx = 0; (line_idx < t->dahdi.line_count); line_idx += 1) {
      bcm_dahdi_drv_phone_line_t *line = &(t->dahdi.lines[line_idx]);
      unsigned long flags;
      int onhook_transfer_tick_count;
      enum dahdi_txsig txsig;
      bool update_line_mode = line->force_update_line;
      bool update_line_rev_polarity = line->force_update_line;
      bool rev_polarity = false;

      line->force_update_line = false;

      if (line->rxsig_pending) {
         line->rxsig_pending = false;
         dahdi_hooksig(&(line->d_chan), line->rxsig);
         /* Force an update of line state */
         update_line_mode = true;
      }

      spin_lock_irqsave(&(line->lock), flags);
      txsig = line->txsig;
      if (line->txsig_pending) {
         line->txsig_pending = false;
         update_line_mode = true;
      }
      rev_polarity = line->fxs_data.computed_rev_polarity;
      if (rev_polarity != line->fxs_data.previous_rev_polarity) {
         line->fxs_data.previous_rev_polarity = rev_polarity;
         update_line_rev_polarity = true;
      }
      if (line->onhook_transfer_tick_count > 0) {
         // We reset line->onhook_transfer_tick_count if
         // the line is off hook or we received signal DAHDI_TXSIG_KEWL
         if ((line->onhook_transfer_tick_count > tick_count)
             && (DAHDI_RXSIG_ONHOOK == line->rxsig)
             && (DAHDI_TXSIG_KEWL != txsig)) {
            line->onhook_transfer_tick_count -= tick_count;
         }
         else {
            line->onhook_transfer_tick_count = 0;
            update_line_mode = true;
         }
      }
      onhook_transfer_tick_count = line->onhook_transfer_tick_count;
      spin_unlock_irqrestore(&(line->lock), flags);

      if (update_line_rev_polarity) {
         int ret = bcm_drv_set_line_rev_polarity(t, line_idx, rev_polarity);
         if (ret) {
            bcm_dev_err(line->dev, "bcm_drv_set_line_rev_polarity(%d) failed (%d)\n",
               (int)(rev_polarity), (int)(ret));
            line->force_update_line = true;
         }
      }

      if (update_line_mode) {
         int ret;
         int reverse_polarity = (rev_polarity) ? 1 : 0;
         /*
           For an FXS line, setting the line state can't change
           if loop is opened or closed so line->rxsig can't change
         */
         switch (txsig) {
            case DAHDI_TXSIG_START: {
               if (DAHDI_RXSIG_ONHOOK == line->rxsig) {
                  /*
                   Phone is on hook and DAHDI wants/thinks it to ring.
                   We set line mode to BCMPH_MODE_ON_RINGING. That's the
                   only possible choice.
                  */
                  ret = bcm_dahdi_set_line_mode(t, line_idx,
                     BCMPH_MODE_ON_RINGING, reverse_polarity);
               }
               else {
                  bcm_assert(DAHDI_RXSIG_OFFHOOK == line->rxsig);
                  /*
                   Phone is off hook and DAHDI wants/thinks it to ring.
                   This state can happen when the user has just pickup
                   the phone while the telephone was ringing.
                   We transmitted the signal DAHDI_RXSIG_OFFHOOK to
                   DAHDI, but DAHDI does not transmit us the signal
                   DAHDI_TXSIG_OFFHOOK yet.
                   We must stop ringing, so we can't set line mode to
                   BCMPH_MODE_ON_RINGING nor BCMPH_MODE_ON_TALKING
                   because these states are only used for on hook condition.
                   Finally we have the choice between BCMPH_MODE_IDLE
                   and BCMPH_MODE_OFF_TALKING.
                   We choose BCMPH_MODE_IDLE because we prefer to let
                   the PCM Path disabled.
                  */
                  ret = bcm_dahdi_set_line_mode(t, line_idx,
                     BCMPH_MODE_IDLE, reverse_polarity);
               }
               if (ret) {
                  line->force_update_line = true;
               }
               break;
            }
            case DAHDI_TXSIG_OFFHOOK: {
               if (DAHDI_RXSIG_OFFHOOK == line->rxsig) {
                  /*
                   Phone is off hook and DAHDI wants it to be off hook.
                   We set line mode to BCMPH_MODE_OFF_TALKING. That's
                   the only possible choice.
                  */
                  ret = bcm_dahdi_set_line_mode(t, line_idx,
                     BCMPH_MODE_OFF_TALKING, reverse_polarity);
               }
               else {
                  bcm_assert(DAHDI_RXSIG_ONHOOK == line->rxsig);
                  /*
                   Phone is on hook and DAHDI wants/thinks it to be off
                   hook. It's possible when DAHDI applies ring cadencing.
                   We set line mode to BCMPH_MODE_ON_TALKING because
                   DAHDI can send audio data that encodes Caller ID.
                  */
                  ret = bcm_dahdi_set_line_mode(t, line_idx,
                     BCMPH_MODE_ON_TALKING, reverse_polarity);
               }
               if (ret) {
                  line->force_update_line = true;
               }
               break;
            }
            case DAHDI_TXSIG_ONHOOK: {
               if (DAHDI_RXSIG_ONHOOK == line->rxsig) {
                  /*
                   Phone is on hook and DAHDI wants/thinks it to be on hook.
                   If (onhook_transfer_tick_count > 0), we set line
                   mode to BCMPH_MODE_ON_TALKING (bceause DAHDI can send
                   audio data that encodes message waiting indication),
                   else we set line mode to BCMPH_MODE_IDLE
                  */
                  if (onhook_transfer_tick_count > 0) {
                     ret = bcm_dahdi_set_line_mode(t, line_idx,
                        BCMPH_MODE_ON_TALKING, reverse_polarity);
                  }
                  else {
                     ret = bcm_dahdi_set_line_mode(t, line_idx,
                        BCMPH_MODE_IDLE, reverse_polarity);
                  }
               }
               else {
                  bcm_assert(DAHDI_RXSIG_OFFHOOK == line->rxsig);
                  /*
                   Phone is off hook and DAHDI wants/thinks it be on hook.
                   We can't ring (BCMPH_MODE_ON_RINGING), because that's
                   not what DAHDI expects.
                   We can't set line mode to BCMPH_MODE_ON_TALKING neither,
                   because these mode is only used for on hook condition.
                   Finally we have the choice between BCMPH_MODE_IDLE and
                   BCMPH_MODE_OFF_TALKING.
                   We choose BCMPH_MODE_IDLE because we prefer to let
                   the PCM Path disabled.
                  */
                  ret = bcm_dahdi_set_line_mode(t, line_idx,
                     BCMPH_MODE_IDLE, reverse_polarity);
               }
               if (ret) {
                  line->force_update_line = true;
               }
               break;
            }
            case DAHDI_TXSIG_KEWL: {
               /*
                It does not matter if line is on or off hook : we set
                the line mode to BCMPH_MODE_DISCONNECT.
                That's what DAHDI expects.
                Only DAHDI can tell us when to exit from this state, even
                if we transitions from off hook to on hook or the reverse
                (but we continue to send signal associated to these event
                to DAHDI).
               */
               bcm_dev_debug(line->dev, "Chan '%s': transmitting remote disconnect (KEWL) on FXS\n",
                  line->d_chan.name);
               ret = bcm_dahdi_set_line_mode(t, line_idx,
                  BCMPH_MODE_DISCONNECT, -1);
               if (ret) {
                  line->force_update_line = true;
               }
               break;
            }
            default: {
               bcm_dev_debug(line->dev, "Chan '%s': unsupported tx signal for FXS: %d\n",
                  line->d_chan.name, txsig);
               break;
            }
         }
      }
   }
}

static void bcm_make_dahdi_work(struct work_struct *w)
{
   bcm_drv_t *t = container_of(w, bcm_drv_t, dahdi.periodic_work.work);
   const int tick_count = bcm_periodic_work_get_tick_count(&(t->dahdi.periodic_work));
   int dahdi_tick_calls = tick_count;

   bcm_periodic_work_dec_tick_count(&(t->dahdi.periodic_work), tick_count);

#ifdef BCMPH_DEBUG
   t->dahdi.stats.calls_to_make_dahdi_work += 1;
#endif /* BCMPH_DEBUG */

   while (dahdi_tick_calls > 0) {
      dahdi_tick_calls -= 1;
      bcm_dahdi_tick(t);
#ifdef BCMPH_DEBUG
      t->dahdi.stats.calls_to_dahdi_tick += 1;
#endif /* BCMPH_DEBUG */
   }

   {
      bcm_phone_get_line_states_t line_states;
      int ret = bcm_drv_get_line_states(t, &(line_states));
      if (!ret) {
         size_t line_idx;
         size_t lines_using_pcm_bus = 0;
         for (line_idx = 0; (line_idx < t->dahdi.line_count); line_idx += 1) {
            bcm_dahdi_drv_phone_line_t *line = &(t->dahdi.lines[line_idx]);
            const bcm_phone_line_state_t *line_state = &(line_states.line_state[line_idx]);
            if (BCMPH_STATUS_OFF_HOOK == line_state->status) {
               if (DAHDI_RXSIG_OFFHOOK != line->rxsig) {
                  bcm_dev_info(line->dev, "Received 'hook off' event on '%s'\n", line->d_chan.name);
                  /*
                   Signals DAHDI that loop is closed.
                   bcm_handle_dahdi_sigs() will update the line state
                  */
                  line->rxsig = DAHDI_RXSIG_OFFHOOK;
                  line->rxsig_pending = true;
               }
            }
            else {
               if (DAHDI_RXSIG_ONHOOK != line->rxsig) {
                  bcm_dev_info(line->dev, "Received 'hook on' event on '%s'\n", line->d_chan.name);
                  /*
                   Signals DAHDI that loop is opened.
                   bcm_handle_dahdi_sigs() will update the line state.
                  */
                  line->rxsig = DAHDI_RXSIG_ONHOOK;
                  line->rxsig_pending = true;
               }
            }
            /*
             We count the line that uses the PCM bus, so that at the
             end of the loop we can start or stop the PCM
            */
            if ((BCMPH_MODE_OFF_TALKING == line_state->mode)
                || (BCMPH_MODE_ON_TALKING == line_state->mode)) {
               lines_using_pcm_bus += 1;
            }
         }
         if (lines_using_pcm_bus > 0) {
            if (!line_states.pcm_is_started) {
               // This case should never occur because we can't stop PCM
               // if at least one line needs it.

               // Start PCM only if at least one line uses it
               for (line_idx = 0; (line_idx < t->dahdi.line_count); line_idx += 1) {
                  if (t->dahdi.lines[line_idx].pcm_started) {
                     break;
                  }
               }
               if (line_idx < t->dahdi.line_count) {
                  bcm_drv_start_pcm(t);
               }
            }
         }
         else {
            if (line_states.pcm_is_started) {
               // As setting the mode is asynchronous, we must be careful
               // to not stop PCM if at least one line uses it
               for (line_idx = 0; (line_idx < t->dahdi.line_count); line_idx += 1) {
                  if (t->dahdi.lines[line_idx].pcm_started) {
                     break;
                  }
               }
               if (line_idx >= t->dahdi.line_count) {
                  bcm_drv_stop_pcm(t, false);
               }
            }
         }
      }
      else {
         /* Should not occur */
         bcm_assert(false);
      }
   }

   bcm_handle_dahdi_sigs(t, tick_count);
}

static int bcm_dahdi_hooksig(struct dahdi_chan *chan, enum dahdi_txsig txsig)
{
   bcm_dahdi_drv_phone_line_t *line = chan->pvt;
   unsigned long flags;

   bcm_pr_debug("%s(chan='%s', txsig=%d)\n", __func__, chan->name,
     (int)(txsig));

   spin_lock_irqsave(&(line->lock), flags);
   line->txsig = txsig;
   line->txsig_pending = true;
   spin_unlock_irqrestore(&(line->lock), flags);

   return (0);
}

static int bcm_dahdi_ioctl(struct dahdi_chan *chan, unsigned int cmd,
   unsigned long data)
{
   bcm_dahdi_drv_phone_line_t *line = chan->pvt;
   unsigned long flags;

   bcm_pr_debug("%s(chan='%s', io_nr=%u, io_dir=%u, io_size=%u)\n", __func__,
      chan->name, _IOC_NR(cmd), _IOC_DIR(cmd), _IOC_SIZE(cmd));

   switch (cmd) {
      case DAHDI_VMWI_CONFIG: {
         struct dahdi_vmwi_info vmwi_info;
         bcm_dev_debug(line->dev, "DAHDI_VMWI_CONFIG\n");
         if (copy_from_user(&(vmwi_info), (__user void *)(data),
               sizeof(vmwi_info))) {
            return (-EFAULT);
         }
         if (!(vmwi_info.vmwi_type & DAHDI_VMWI_LREV)) {
            bcm_dev_warn(line->dev, "Chan '%s': only lrev VMWI is supported. vmwi_type = %u\n",
               chan->name, (unsigned int)(vmwi_info.vmwi_type));
         }
         else {
            spin_lock_irqsave(&(line->lock), flags);
            line->fxs_data.vmwi_info = vmwi_info;
            bcm_compute_fxs_reverse_polarity(line);
            spin_unlock_irqrestore(&(line->lock), flags);
         }
      }
      break;

      case DAHDI_VMWI: {
         int x;
         if (get_user(x, (__user int *)(data))) {
            return (-EFAULT);
         }
         bcm_dev_debug(line->dev, "DAHDI_VMWI => %d waiting messages\n", (int)(x));
         spin_lock_irqsave(&(line->lock), flags);
         line->fxs_data.message_waiting = ((x > 0) ? true : false);
         bcm_compute_fxs_reverse_polarity(line);
         spin_unlock_irqrestore(&(line->lock), flags);
      }
      break;

      case DAHDI_SETPOLARITY: {
         int x;
         if (get_user(x, (__user int *)(data))) {
            return (-EFAULT);
         }
         bcm_dev_debug(line->dev, "DAHDI_SETPOLARITY, data=%d\n", x);
         spin_lock_irqsave(&(line->lock), flags);
         line->fxs_data.reverse_polarity = ((x) ? true : false);
         bcm_compute_fxs_reverse_polarity(line);
         spin_unlock_irqrestore(&(line->lock), flags);
      }
      break;

      case DAHDI_ONHOOKTRANSFER: {
         // Start pcm for x milliseconds and transfer data
         // containing information about message waiting
         // As soon as we are off hook stop transmitting
         int x;
         if (get_user(x, (__user int *)(data))) {
            return (-EFAULT);
         }
         bcm_dev_debug(line->dev, "DAHDI_ONHOOKTRANSFER for %d ms\n", (int)(x));
         spin_lock_irqsave(&(line->lock), flags);
         line->onhook_transfer_tick_count = (((x * 1000) + DAHDI_TICK_PERIOD_US - 1) / DAHDI_TICK_PERIOD_US) + 1;
         line->txsig_pending = true;
         spin_unlock_irqrestore(&(line->lock), flags);
      }
      break;

      default: {
         bcm_dev_debug(line->dev, "cmd %u not handled\n", (unsigned int)(cmd));
         return (-ENOTTY);
      }
      break;
   }

   return (0);
}

static int __init bcm_dahdi_startup(struct file *file, struct dahdi_span *span)
{
   bcm_drv_t *t = container_of(span, bcm_drv_t, dahdi.d_span);

   bcm_pr_debug("%s()\n", __func__);

   bcm_periodic_timer_add_work(&(t->core.periodic_timer),
      &(t->dahdi.periodic_work), DAHDI_TICK_PERIOD_US);

   return (0);
}

static int bcm_dahdi_shutdown(struct dahdi_span *span)
{
   bcm_drv_t *t = container_of(span, bcm_drv_t, dahdi.d_span);

   bcm_pr_debug("%s()\n", __func__);

   bcm_periodic_timer_del_work(&(t->core.periodic_timer),
      &(t->dahdi.periodic_work));
   cancel_work_sync(&(t->dahdi.periodic_work.work));

   return (0);
}

/*
 We do not handle opening and closing of a channel.
 From what i understand, when a channel is closed it continues to exist in DAHDI
 but events generated for this channel are simply not used.
 So we can continue to call dahdi_hooksig() (which generates events),
 dahdi_qevent_lock() and read or write chan->readchunk and chan->writechunk.
*/
static int bcm_dahdi_open(struct dahdi_chan *chan)
{
   d_bcm_pr_debug("%s()\n", __func__);

   return (0);
}

/* See remarks on function bcm_dahdi_open() */
static int bcm_dahdi_close(struct dahdi_chan *chan)
{
   d_bcm_pr_debug("%s()\n", __func__);

   return (0);
}

/*
 Among the callback that DAHDI can call, it exists sync_tick().
 The callback exists only if DAHDI_SYNC_TICK is defined in file dahdi_config.h
 and it allows low-level drivers to synchronize their internal clocks to the
 DAHDI master clock.
 But the callback is called by function process_masterspan() of DAHDI which
 is called by :
 * dahdi_receive() when that function is called for the master span
 * coretimer_func() if CONFIG_DAHDI_CORE_TIMER is defined AND the master span
 don't call dahdi_receive() since one second.

 If we were sure not to be the master span, we could use sync_tick() to schedule
 the DAHDI work and to count the number of times to call dahdi_receive/transmit,
 but it's possible only :
 * if CONFIG_DAHDI_CORE_TIMER is defined, else if we are the only span,
 sync_tick() will never be called.
 * and if we know how to prevent DAHDI from choosing our span to be the
 master span : there's a flag 'cannot_provide_timing' but it's not tested by
 function set_master_span(), only by function __dahdi_find_master_span().
 So we can't be sure not to be the master span.

 In conclusion, we provide our own timing source and do not use sync_tick().
 That way we don't care if either DAHDI_SYNC_TICK or CONFIG_DAHDI_CORE_TIMER are
 not defined, and don't set flag 'cannot_provide_timing'.
*/
static const struct dahdi_span_ops bcm_dahdi_span_ops = {
   .owner = THIS_MODULE,
   .hooksig = bcm_dahdi_hooksig,
   .ioctl = bcm_dahdi_ioctl,
   .startup = bcm_dahdi_startup,
   .shutdown = bcm_dahdi_shutdown,
   .open = bcm_dahdi_open,
   .close = bcm_dahdi_close,
};

static int __init bcm_dahdi_init(bcm_drv_t *t)
{
   int ret = -1;
   size_t line_idx;

   bcm_pr_debug("%s()\n", __func__);

   t->dahdi.d_dev = dahdi_create_device();
   if (NULL == t->dahdi.d_dev) {
      return (-ENOMEM);
   }

   dev_set_name(&(t->dahdi.d_dev->dev), driver_name);

   for (line_idx = 0; (line_idx < ARRAY_SIZE(t->dahdi.lines)); line_idx += 1) {
      t->dahdi.lines[line_idx].dev = &(t->dahdi.d_dev->dev);
   }

   t->dahdi.d_dev->manufacturer = "ACME";
   t->dahdi.d_dev->devicetype = kasprintf(GFP_KERNEL, driver_name);
   snprintf(t->dahdi.d_span.name, sizeof(t->dahdi.d_span.name), "%s/1",
      driver_name);
   dahdi_copy_string(t->dahdi.d_span.desc,
         "FXS driver for brcm63xx", sizeof(t->dahdi.d_span.desc));
   t->dahdi.d_span.ops = &(bcm_dahdi_span_ops);
   bcm_assert(ARRAY_SIZE(t->dahdi.d_chans) == ARRAY_SIZE(t->dahdi.lines));
   t->dahdi.line_count = bcm_phone_mgr_get_line_count(&(t->core.phone_mgr));
   if (t->dahdi.line_count > ARRAY_SIZE(t->dahdi.d_chans)) {
      t->dahdi.line_count = ARRAY_SIZE(t->dahdi.d_chans);
   }
   t->dahdi.d_span.channels = t->dahdi.line_count;
   t->dahdi.d_span.chans = t->dahdi.d_chans;
   t->dahdi.d_span.deflaw = DAHDI_CODEC;
   t->dahdi.d_span.flags = DAHDI_FLAG_RBS;
   t->dahdi.d_span.spantype = SPANTYPE_ANALOG_FXS;
   for (line_idx = 0; (line_idx < t->dahdi.line_count); line_idx += 1) {
      bcm_dahdi_drv_phone_line_t* line = &(t->dahdi.lines[line_idx]);
      t->dahdi.d_chans[line_idx] = &(line->d_chan);
      line->d_chan.pvt = line;
      line->d_chan.chanpos = line_idx + 1;
      snprintf(line->d_chan.name, sizeof(line->d_chan.name), "%s/%lu",
         t->dahdi.d_span.name, (unsigned long)(line_idx));
      /*
       /!\ FXO uses FXS signalling and vice-versa.
       We don't handle groud key detection,
       (events VP_LINE_EVID_GKEY_DET and VP_LINE_EVID_GKEY_REL in
       function phone_dev_zarlink_tick() in file zarlink_common.c)
       so, we don't set the flag DAHDI_SIG_FXOGS.
      */
      line->d_chan.sigcap = (DAHDI_SIG_FXOKS | DAHDI_SIG_FXOLS);
   }

   list_add_tail(&(t->dahdi.d_span.device_node), &(t->dahdi.d_dev->spans));
   if (dahdi_register_device(t->dahdi.d_dev, NULL)) {
      bcm_dev_warn(&(t->dahdi.d_dev->dev), "Couldn't register span.\n");
      ret = -EINVAL;
      goto err_register_device;
   }

   return (0);

   dahdi_unregister_device(t->dahdi.d_dev);
err_register_device:
   dahdi_free_device(t->dahdi.d_dev);
   t->dahdi.d_dev = NULL;
   return (ret);
}

static void bcm_dahdi_deinit(bcm_drv_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   dahdi_unregister_device(t->dahdi.d_dev);
   dahdi_free_device(t->dahdi.d_dev);
   t->dahdi.d_dev = NULL;
}

static void bcm_dahdi_line_init(bcm_dahdi_drv_phone_line_t *line)
{
   bcm_pr_debug("%s()\n", __func__);

   memset(line, 0, sizeof(*line));

   spin_lock_init(&(line->lock));

   line->dev = NULL;
   line->force_update_line = false;
   line->pcm_started = false;
   line->txsig_pending = false;
   line->rxsig_pending = false;

#ifdef SHORTCUT_TX
   line->idx_demo_thanks = 0;
#endif /* SHORTCUT_TX */

#ifdef BCMPH_DEBUG
   line->stats.bytes_rx_from_dahdi = 0;
   line->stats.bytes_tx_to_dahdi = 0;
   line->stats.err_tx_ring_buf_full = 0;
   line->stats.err_rx_ring_buf_empty = 0;
#endif /* BCMPH_DEBUG */
}

#ifdef BCMPH_DEBUG
static void bcm_dahdi_line_pr_stats(bcm_dahdi_drv_phone_line_t *line)
{
   bcm_pr_info("Line global stats:\n");
   bcm_pr_info("bytes_rx_from_dahdi=%lu, bytes_tx_to_dahdi=%lu\n",
      line->stats.bytes_rx_from_dahdi, line->stats.bytes_tx_to_dahdi);
   bcm_pr_info("err_tx_ring_buf_full=%u, err_rx_ring_buf_empty=%u\n",
      line->stats.err_tx_ring_buf_full, line->stats.err_rx_ring_buf_empty);
}

void bcm_dahdi_pr_stats(bcm_drv_t *t)
{
   size_t line_idx;
   for (line_idx = 0; (line_idx < t->dahdi.line_count); line_idx += 1) {
      bcm_dahdi_line_pr_stats(&(t->dahdi.lines[line_idx]));
   }
   bcm_pr_info("Calls to bcm_make_dahdi_work() : %lu:\n", (unsigned long)(t->dahdi.stats.calls_to_make_dahdi_work));
   bcm_pr_info("Calls to bcm_dahdi_tick() : %lu:\n", (unsigned long)(t->dahdi.stats.calls_to_dahdi_tick));
}
#endif /* BCMPH_DEBUG */

static void bcm_dahdi_line_deinit(bcm_dahdi_drv_phone_line_t *line)
{
   bcm_pr_debug("%s()\n", __func__);

#ifdef BCMPH_DEBUG
   bcm_dahdi_line_pr_stats(line);
#endif /* BCMPH_DEBUG */
}

static void __init bcm_dahdi_init_lines(bcm_drv_t *t)
{
   size_t line_idx;

   bcm_pr_debug("%s()\n", __func__);

   for (line_idx = 0; (line_idx < ARRAY_SIZE(t->dahdi.lines)); line_idx += 1) {
      bcm_dahdi_line_init(&(t->dahdi.lines[line_idx]));
   }
}

static void bcm_dahdi_deinit_lines(bcm_drv_t *t)
{
   size_t line_idx;

   bcm_pr_debug("%s()\n", __func__);

   for (line_idx = 0; (line_idx < ARRAY_SIZE(t->dahdi.lines)); line_idx += 1) {
      bcm_dahdi_line_deinit(&(t->dahdi.lines[line_idx]));
   }
}

static bcmph_country_t bcm_dahdi_decode_country(const char *country)
{
   bcmph_country_t ret = BCMPH_COUNTRY_ETSI;

   if ((NULL == country) || ('\0' == country[0])
       || (!strcasecmp(country, "etsi"))) {
      ret = BCMPH_COUNTRY_ETSI;
   }
   else if (!strcasecmp(country, "gr57")) {
      ret = BCMPH_COUNTRY_GR57;
   }
   else if (!strcasecmp(country, "australia")) {
      ret = BCMPH_COUNTRY_AU;
   }
   else if (!strcasecmp(country, "austria")) {
      ret = BCMPH_COUNTRY_AT;
   }
   else if (!strcasecmp(country, "belgium")) {
      ret = BCMPH_COUNTRY_BE;
   }
   else if (!strcasecmp(country, "brazil")) {
      ret = BCMPH_COUNTRY_BR;
   }
   else if (!strcasecmp(country, "bulgaria")) {
      ret = BCMPH_COUNTRY_BG;
   }
   else if (!strcasecmp(country, "canada")) {
      ret = BCMPH_COUNTRY_CA;
   }
   else if (!strcasecmp(country, "china")) {
      ret = BCMPH_COUNTRY_CN;
   }
   else if (!strcasecmp(country, "denmark")) {
      ret = BCMPH_COUNTRY_DK;
   }
   else if (!strcasecmp(country, "finland")) {
      ret = BCMPH_COUNTRY_FI;
   }
   else if (!strcasecmp(country, "france")) {
      ret = BCMPH_COUNTRY_FR;
   }
   else if (!strcasecmp(country, "germany")) {
      ret = BCMPH_COUNTRY_DE;
   }
   else if (!strcasecmp(country, "great_britain")) {
      ret = BCMPH_COUNTRY_GB;
   }
   else if (!strcasecmp(country, "greece")) {
      ret = BCMPH_COUNTRY_GR;
   }
   else if (!strcasecmp(country, "honk_kong")) {
      ret = BCMPH_COUNTRY_HK;
   }
   else if (!strcasecmp(country, "hungary")) {
      ret = BCMPH_COUNTRY_HU;
   }
   else if (!strcasecmp(country, "iceland")) {
      ret = BCMPH_COUNTRY_IS;
   }
   else if (!strcasecmp(country, "ireland")) {
      ret = BCMPH_COUNTRY_IE;
   }
   else if (!strcasecmp(country, "israel")) {
      ret = BCMPH_COUNTRY_IL;
   }
   else if (!strcasecmp(country, "italy")) {
      ret = BCMPH_COUNTRY_IT;
   }
   else if (!strcasecmp(country, "japan")) {
      ret = BCMPH_COUNTRY_JP;
   }
   else if (!strcasecmp(country, "korea")) {
      ret = BCMPH_COUNTRY_KR;
   }
   else if (!strcasecmp(country, "netherlands")) {
      ret = BCMPH_COUNTRY_NL;
   }
   else if (!strcasecmp(country, "new_zealand")) {
      ret = BCMPH_COUNTRY_NZ;
   }
   else if (!strcasecmp(country, "norway")) {
      ret = BCMPH_COUNTRY_NO;
   }
   else if (!strcasecmp(country, "portugal")) {
      ret = BCMPH_COUNTRY_PT;
   }
   else if (!strcasecmp(country, "russia")) {
      ret = BCMPH_COUNTRY_RU;
   }
   else if (!strcasecmp(country, "singapore")) {
      ret = BCMPH_COUNTRY_SG;
   }
   else if (!strcasecmp(country, "south_africa")) {
      ret = BCMPH_COUNTRY_ZA;
   }
   else if (!strcasecmp(country, "spain")) {
      ret = BCMPH_COUNTRY_ES;
   }
   else if (!strcasecmp(country, "sweden")) {
      ret = BCMPH_COUNTRY_SE;
   }
   else if (!strcasecmp(country, "switzerland")) {
      ret = BCMPH_COUNTRY_CH;
   }
   else if (!strcasecmp(country, "taiwan")) {
      ret = BCMPH_COUNTRY_TW;
   }
   else if (!strcasecmp(country, "turkey")) {
      ret = BCMPH_COUNTRY_TK;
   }
   else if (!strcasecmp(country, "united_states")) {
      ret = BCMPH_COUNTRY_US;
   }
   return (ret);
}

int __init bcm_dahdi_start(bcm_drv_t *t)
{
   int ret = -1;
   size_t line_idx;
   bcm_phone_cfg_params_t params;
   bcm_phone_get_line_states_t line_states;

   bcm_pr_debug("%s()\n", __func__);

   memset(&(params), 0, sizeof(params));
   params.country = bcm_dahdi_decode_country(bcm_country);
#if (BCMPH_DAHDI_CODEC_LINEAR == BCMPH_DAHDI_CODEC)
   params.pcm_use_16bits_timeslot = true;
#else
   params.pcm_use_16bits_timeslot = false;
#endif
   for (line_idx = 0; (line_idx < t->dahdi.line_count); line_idx += 1) {
      params.line_params[line_idx].enable = true;
      params.line_params[line_idx].codec = BCMPH_CODEC;
   }
   ret = bcm_drv_start(t, &(params));
   if (ret) {
      goto fail_start;
   }

   ret = bcm_drv_get_line_states(t, &(line_states));
   if (ret) {
      goto fail_get_line_states;
   }
   for (line_idx = 0; (line_idx < t->dahdi.line_count); line_idx += 1) {
      bcm_dahdi_drv_phone_line_t *line = &(t->dahdi.lines[line_idx]);
      line->force_update_line = false;
      line->onhook_transfer_tick_count = 0;
      line->txsig = DAHDI_TXSIG_ONHOOK;
      line->txsig_pending = false;
      if (BCMPH_STATUS_OFF_HOOK == line_states.line_state[line_idx].status)
      {
         line->rxsig = DAHDI_RXSIG_OFFHOOK;
      }
      else {
         line->rxsig = DAHDI_RXSIG_ONHOOK;
      }
      /*
       Set flag rxsig_pending to send signal rxsig to DAHDI and force updating
       line state
      */
      line->rxsig_pending = true;
      bcm_compute_fxs_reverse_polarity(line);
      line->fxs_data.previous_rev_polarity = line->fxs_data.computed_rev_polarity;

      /* Initialize line if state is not what we expect */
      if (line_states.line_state[line_idx].rev_polarity != line->fxs_data.computed_rev_polarity) {
         ret = bcm_drv_set_line_rev_polarity(t, line_idx, line->fxs_data.computed_rev_polarity);
         if (ret) {
            break;
         }
      }
      if (BCMPH_MODE_IDLE != line_states.line_state[line_idx].mode) {
         ret = bcm_drv_set_line_mode(t, line_idx, BCMPH_MODE_IDLE,
            line->fxs_data.computed_rev_polarity ? 1 : 0,
            bcm_phone_line_tone_code_index(BCMPH_TONE_NONE));
         if (ret) {
            break;
         }
      }
      else if (BCMPH_TONE_NONE != line_states.line_state[line_idx].tone) {
         ret = bcm_drv_set_line_tone(t, line_idx, bcm_phone_line_tone_code_index(BCMPH_TONE_NONE));
         if (ret) {
            break;
         }
      }
   }

   ret = bcm_dahdi_startup(NULL, &(t->dahdi.d_span));
   if (ret) {
      goto fail_startup;
   }

   return (0);

   bcm_dahdi_shutdown(&(t->dahdi.d_span));
fail_startup:
fail_get_line_states:
   bcm_drv_stop(t, true);
fail_start:
   return (ret);
}

void bcm_dahdi_stop(bcm_drv_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   bcm_dahdi_shutdown(&(t->dahdi.d_span));
   bcm_drv_stop(t, true);
}

int __init bcm_drv_init(bcm_drv_t *t)
{
   int ret = -1;

   bcm_pr_debug("%s()\n", __func__);

   bcm_dahdi_init_null_samples();

   bcm_dahdi_init_lines(t);

#ifdef BCMPH_DEBUG
   t->dahdi.stats.calls_to_make_dahdi_work = 0;
   t->dahdi.stats.calls_to_dahdi_tick = 0;
#endif /* BCMPH_DEBUG */
   bcm_periodic_work_init(&(t->dahdi.periodic_work), t->core.wq, bcm_make_dahdi_work);

   ret = bcm_dahdi_init(t);
   if (ret) {
      goto err_dahdi_init;
   }

   return (0);

   bcm_dahdi_deinit(t);
err_dahdi_init:
   bcm_periodic_work_deinit(&(t->dahdi.periodic_work));
   bcm_dahdi_deinit_lines(t);
   return (ret);
}

void bcm_drv_exit(bcm_drv_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   /*
    We can't call bcm_dahdi_stop() because it will call bcm_drv_stop()
    that will call pcm_stop.
    But pcm is already deinit here so that's not possible
   */
   bcm_dahdi_shutdown(&(t->dahdi.d_span));
   bcm_dahdi_deinit(t);
   bcm_periodic_work_deinit(&(t->dahdi.periodic_work));
   bcm_dahdi_deinit_lines(t);
}

#endif /* BCMPH_DAHDI_DRIVER */
