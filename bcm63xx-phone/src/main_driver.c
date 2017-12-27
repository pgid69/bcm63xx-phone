/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <config.h>

#ifdef BCMPH_EXPORT_DEV_FILE

#include <extern/linux/barrier.h>
#include <extern/linux/errno.h>
#include <extern/linux/uaccess.h>
#include <extern/linux/poll.h>

#include "main.h"

// Include after system files
#include <compile.h>

void bcm_drv_resume_read_cb(pcm_t *pcm)
{
   bcm_drv_t *t = container_of(pcm, bcm_drv_t, core.pcm);
   dd_bcm_pr_debug("%s()\n", __func__);
   bcm_wait_queue_wake_up(&(t->dev.inq));
}

static ssize_t bcm_drv_read_line(bcm_drv_t *t,
   bool do_not_block, bcmph_mutex_t *lock,
   size_t line, __u8 *buf, size_t count)
{
   ssize_t ret = 0;
   int wq_counter;

   dd_bcm_pr_debug("%s(line=%lu, buf=%lx, count=%lu)\n", __func__,
      (unsigned long)(line), (unsigned long)(buf), (unsigned long)(count));

   bcm_assert((NULL != t) && (line < bcm_phone_mgr_get_line_count(&(t->core.phone_mgr)))
      && (NULL != lock) && (bcmph_mutex_is_locked(lock)));

   // We must init the counter before updating the RX buffers
   // to be sure to not loose an event on inq that would occur after
   // updating the buffers
   wq_counter = bcm_wait_queue_get_counter(&(t->dev.inq));

   bcm_drv_update_buffers(t);

   do { // Empty loop
      bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line]);
      if (!bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line)) {
         ret = -EFAULT;
         break;
      }

      if (count <= 0) {
         break;
      }

      if (!access_ok(VERIFY_WRITE, buf, count)) {
         ret = -EFAULT;
         break;
      }

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
            bcm_drv_update_buffers(t);
            if (count <= 0) {
               break;
            }
         }
         else {
            int tmp;
            // Nothing to read
            if ((ret) || (!bcm_phone_mgr_line_rx_use_pcm(&(t->core.phone_mgr), line))) {
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
            tmp = bcm_wait_queue_wait_event_counter_timeout(&(t->dev.inq),
               wq_counter,
               msecs_to_jiffies(t->core.board_desc->phone_desc->tick_period), lock);
            if (tmp < 0) {
               ret = tmp;
               break;
            }
            // We don't forget to update the counter for the next call
            // to wait_event, before updating the RX buffers
            barrier();
            wq_counter = bcm_wait_queue_get_counter(&(t->dev.inq));
            bcm_drv_update_buffers(t);
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

   dd_bcm_pr_debug("%s(buf=%lx, count=%lu)\n", __func__, (unsigned long)(buf), (unsigned long)(count));

   bcm_assert(NULL != t);

   bcmph_mutex_init(&(lock), &(t->dev.lock), false);

   if (bcmph_mutex_lock_interruptible(&(lock))) {
      ret = -ERESTARTSYS;
   }
   else {
      ret = bcm_drv_read_line(t,
         ((filp->f_flags & O_NONBLOCK) != 0), &(lock),
         t->dev.default_line, (__u8 *)(buf), count);
      if (ret > 0) {
         bcm_assert((0 == (ret % t->core.phone_lines[t->dev.default_line].bytes_per_frame)) && (((size_t)(ret)) <= count));
         if (NULL != offset) {
            *offset += ret;
         }
      }
   }

   bcmph_mutex_deinit(&(lock));

   return (ret);
}

void bcm_drv_resume_write_cb(pcm_t *pcm)
{
   bcm_drv_t *t = container_of(pcm, bcm_drv_t, core.pcm);
   dd_bcm_pr_debug("%s()\n", __func__);
   bcm_wait_queue_wake_up(&(t->dev.outq));
}

static ssize_t bcm_drv_write_line(bcm_drv_t *t,
   bool do_not_block, bcmph_mutex_t *lock,
   size_t line, const __u8 *buf, size_t count)
{
   ssize_t ret = 0;
   int wq_counter;

   dd_bcm_pr_debug("%s(line=%lu, buf=%lx, count=%lu)\n", __func__,
      (unsigned long)(line), (unsigned long)(buf), (unsigned long)(count));

   bcm_assert((NULL != t) && (line < bcm_phone_mgr_get_line_count(&(t->core.phone_mgr)))
      && (NULL != lock) && (bcmph_mutex_is_locked(lock)));

   // We must init the counter before updating the TX buffers
   // to be sure to not loose an event on outq that would occur after
   // updating the buffers
   wq_counter = bcm_wait_queue_get_counter(&(t->dev.outq));

   bcm_drv_update_buffers(t);

   do { // Empty loop
      bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line]);

      if (!bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line)) {
         ret = -EFAULT;
         break;
      }

      if (count <= 0) {
         break;
      }

      if (!access_ok(VERIFY_READ, buf, count)) {
         ret = -EFAULT;
         break;
      }

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
            bcm_drv_update_buffers(t);
            if (count <= 0) {
               break;
            }
         }
         else {
            int tmp;
            if ((ret) || (!bcm_phone_mgr_line_tx_use_pcm(&(t->core.phone_mgr), line))) {
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
            tmp = bcm_wait_queue_wait_event_counter_timeout(&(t->dev.outq),
               wq_counter,
               msecs_to_jiffies(t->core.board_desc->phone_desc->tick_period), lock);
            if (tmp < 0) {
               ret = tmp;
               break;
            }
            // We don't forget to update the counter for the next call
            // to wait_event, before updating the TX buffers
            barrier();
            wq_counter = bcm_wait_queue_get_counter(&(t->dev.outq));
            bcm_drv_update_buffers(t);
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

   dd_bcm_pr_debug("%s(buf=%lx, count=%lu)\n", __func__, (unsigned long)(buf), (unsigned long)(count));

   bcm_assert(NULL != t);

   bcmph_mutex_init(&(lock), &(t->dev.lock), false);

   if (bcmph_mutex_lock_interruptible(&(lock))) {
      ret = -ERESTARTSYS;
   }
   else {
      ret = bcm_drv_write_line(t, ((filp->f_flags & O_NONBLOCK) != 0), &(lock),
         t->dev.default_line, (const __u8 *)(buf), count);
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

   d_bcm_pr_debug("%s()\n", __func__);

   bcm_assert(NULL != t);

   bcmph_mutex_init(&(lock), &(t->dev.lock), false);

   // If we can't get the semaphore we return 0
   if (!bcmph_mutex_lock_interruptible(&(lock))) {
      size_t line = t->dev.default_line;
      bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line]);

      if (NULL != poll_table) {
         // Allow the kernel to be notified of a change in the status of
         // this file
         // Does not block the call
         poll_wait(filp, &(t->dev.inq.wq), poll_table);
         poll_wait(filp, &(t->dev.outq.wq), poll_table);
      }

      bcm_drv_update_buffers(t);

      // If pcm is stopped, we lie and tell that data are ready,
      // because we can't wait for data we can't receive
      if ((bcm_ring_buf_get_size(&(dpl->rx_ring_buf)) > 0) || (!bcm_phone_mgr_line_rx_use_pcm(&(t->core.phone_mgr), line))) {
         /* readable */
         d_bcm_pr_debug("Can read\n");
         ret |= (POLLIN | POLLRDNORM);
      }
      // If pcm is stopped, we lie and tell that we can write data
      // because we can't wait for pending data to be transmitted as we can't transfer
      if ((bcm_ring_buf_get_free_space(&(dpl->tx_ring_buf)) > 0) || (!bcm_phone_mgr_line_tx_use_pcm(&(t->core.phone_mgr), line))) {
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

   d_bcm_pr_debug("%s()\n", __func__);

   bcm_assert(NULL != t);

   bcmph_mutex_init(&(lock), &(t->dev.lock), false);

   do { // Empty loop
      size_t dummy;
      size_t max_frames;

      if (bcmph_mutex_lock_interruptible(&(lock))) {
         ret = -ERESTARTSYS;
         break;
      }

      for (;;) {
         if (!pcm_dma_is_started(&(t->core.pcm))) {
            break;
         }
         // We init the counter before updating the TX buffers
         // to be sure to not loose an event on outq that would occur after
         // updating the buffers
         barrier();
         wq_counter = bcm_wait_queue_get_counter(&(t->dev.outq));
         bcm_drv_update_buffers(t);
         bcm_drv_get_data_for_tx(t, &(dummy), &(max_frames));
         if (max_frames <= 0) {
            break;
         }
         d_bcm_pr_debug("Waiting for data to be transferred in TX buffers\n");
         ret = bcm_wait_queue_wait_event_counter_timeout(&(t->dev.outq),
            wq_counter, msecs_to_jiffies(t->core.board_desc->phone_desc->tick_period), &(lock));
         if (ret < 0) {
            break;
         }
      }
   } while (false);

   bcmph_mutex_deinit(&(lock));

   return (ret);
}
#endif // __KERNEL__

static void bcm_drv_update_ring_buf_desc(bcm_drv_t *t)
{
   size_t line_idx;
   size_t phone_line_count = bcm_phone_mgr_get_line_count(&(t->core.phone_mgr));

   dd_bcm_pr_debug("%s()\n", __func__);

   for (line_idx = 0; (line_idx < phone_line_count); line_idx += 1) {
      if (bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line_idx)) {
         const bcm_core_drv_phone_line_t *dpl = &(t->core.phone_lines[line_idx]);
         bcm_ring_buf_desc_t *tmp;

         tmp = (bcm_ring_buf_desc_t *)(t->core.mm_buffer + t->core.mm_rbs_location.rbs[line_idx].rx_ring_buf_desc_off);
         bcm_ring_buf_get_desc(&(dpl->rx_ring_buf), tmp);

         tmp = (bcm_ring_buf_desc_t *)(t->core.mm_buffer + t->core.mm_rbs_location.rbs[line_idx].tx_ring_buf_desc_off);
         bcm_ring_buf_get_desc(&(dpl->tx_ring_buf), tmp);
      }
   }
}

#ifdef __KERNEL__
static
#endif // __KERNEL__
long bcm_drv_unlocked_ioctl(struct file *filp,
             unsigned int cmd, unsigned long arg)
{
   long ret = 0;
   bcm_drv_t *t = (bcm_drv_t *)(filp->private_data);
   bcmph_mutex_t lock;

   dd_bcm_pr_debug("%s(cmd %u)\n", __func__, (unsigned int)(cmd));

   bcm_assert(NULL != t);

   bcmph_mutex_init(&(lock), &(t->dev.lock), false);

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

            if ((line >= bcm_phone_mgr_get_line_count(&(t->core.phone_mgr)))
                || (!bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line))) {
               bcm_pr_err("%s(cmd=%u) : invalid line %lu\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(line));
               ret = -EFAULT;
               break;
            }
            if (len > bcm_ring_buf_get_size(&(t->core.phone_lines[line].rx_ring_buf))) {
               bcm_pr_err("%s(cmd=%u) : invalid len %lu\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(len));
               ret = -EFAULT;
               break;
            }

            bcm_ring_buf_remove_len(&(t->core.phone_lines[line].rx_ring_buf), len);
            bcm_drv_update_buffers(t);
            bcm_drv_update_ring_buf_desc(t);
            break;
         }
         case BCMPH_IOCTL_WRITE_MM: {
            size_t line = arg & 0xFF;
            size_t len = arg >> 8;

            dd_bcm_pr_debug("BCMPH_IOCTL_WRITE_MM\n");

            if ((line >= bcm_phone_mgr_get_line_count(&(t->core.phone_mgr)))
                || (!bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line))) {
               bcm_pr_err("%s(cmd=%u) : invalid line %lu\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(line));
               ret = -EFAULT;
               break;
            }
            if (len > bcm_ring_buf_get_free_space(&(t->core.phone_lines[line].tx_ring_buf))) {
               bcm_pr_err("%s(cmd=%u) : invalid len %lu\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(len));
               ret = -EFAULT;
               break;
            }

            bcm_ring_buf_add_len(&(t->core.phone_lines[line].tx_ring_buf), len);
            bcm_drv_update_buffers(t);
            bcm_drv_update_ring_buf_desc(t);
            break;
         }
         case BCMPH_IOCTL_UPDATE_RBS: {
            dd_bcm_pr_debug("BCMPH_IOCTL_UPDATE_RBS\n");

            if (!pcm_is_started(&(t->core.pcm))) {
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
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(get_line_states)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(get_line_states), (void *)(arg), sizeof(get_line_states)) > 0) {
               bcm_pr_err("%s(cmd=%u) : copy_from_user failed\n", __func__, (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_get_line_states(t, &(get_line_states), &(lock));
            if (!ret) {
               if (copy_to_user((void *)(arg), &(get_line_states), sizeof(get_line_states)) > 0) {
                  bcm_pr_err("%s(cmd=%u) : copy_to_user failed\n", __func__, (unsigned int)(cmd));
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
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(set_line_mode)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(set_line_mode), (void *)(arg), sizeof(set_line_mode)) > 0) {
               bcm_pr_err("%s(cmd=%u) : copy_from_user failed\n", __func__, (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_mode(t, set_line_mode.line,
               set_line_mode.mode, set_line_mode.echo_cancellation,
               set_line_mode.reverse_polarity, set_line_mode.tone,
               set_line_mode.wait, &(lock));
            break;
         }
         case BCMPH_IOCTL_SET_LINE_CODEC: {
            bcm_phone_set_line_codec_t set_line_codec;

            d_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_CODEC\n");

            if (_IOC_SIZE(cmd) != sizeof(set_line_codec)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(set_line_codec)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(set_line_codec), (void *)(arg), sizeof(set_line_codec)) > 0) {
               bcm_pr_err("%s(cmd=%u) : copy_from_user failed\n", __func__, (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_codec(t, set_line_codec.line,
               set_line_codec.codec, set_line_codec.mode,
               set_line_codec.echo_cancellation,
               set_line_codec.reverse_polarity, set_line_codec.tone,
               &(lock));
            break;
         }
         case BCMPH_IOCTL_SET_LINE_TONE: {
            bcm_phone_set_line_tone_t set_line_tone;

            d_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_TONE\n");

            if (_IOC_SIZE(cmd) != sizeof(set_line_tone)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(set_line_tone)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(set_line_tone), (void *)(arg), sizeof(set_line_tone)) > 0) {
               bcm_pr_err("%s(cmd=%u) : copy_from_user failed\n", __func__, (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_tone(t, set_line_tone.line, set_line_tone.tone, set_line_tone.wait, &(lock));
            break;
         }
         case BCMPH_IOCTL_SET_LINE_MODE_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->core.mm_buffer + t->dev.mm_desc.ioctl_param_off);

            d_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_MODE_MM\n");

            if (param->size != sizeof(param->p.set_line_mode)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.set_line_mode)));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_mode(t, param->p.set_line_mode.line,
               param->p.set_line_mode.mode,
               param->p.set_line_mode.echo_cancellation,
               param->p.set_line_mode.reverse_polarity,
               param->p.set_line_mode.tone, param->p.set_line_mode.wait,
               &(lock));
            break;
         }
         case BCMPH_IOCTL_SET_LINE_CODEC_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->core.mm_buffer + t->dev.mm_desc.ioctl_param_off);

            d_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_CODEC_MM\n");

            if (param->size != sizeof(param->p.set_line_codec)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.set_line_codec)));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_codec(t, param->p.set_line_codec.line,
               param->p.set_line_codec.codec, param->p.set_line_codec.mode,
               param->p.set_line_codec.echo_cancellation,
               param->p.set_line_codec.reverse_polarity,
               param->p.set_line_codec.tone, &(lock));
            break;
         }
         case BCMPH_IOCTL_SET_LINE_TONE_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->core.mm_buffer + t->dev.mm_desc.ioctl_param_off);

            d_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_TONE_MM\n");

            if (param->size != sizeof(param->p.set_line_tone)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
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
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(cfg_params)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(cfg_params), (void *)(arg), sizeof(cfg_params)) > 0) {
               bcm_pr_err("%s(cmd=%u) : copy_from_user failed\n", __func__, (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_start(t, &(cfg_params));
            break;
         }
         case BCMPH_IOCTL_STOP: {
            bcm_pr_debug("BCMPH_IOCTL_STOP\n");

            bcm_drv_stop(t, true);
            break;
         }
         case BCMPH_IOCTL_START_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->core.mm_buffer + t->dev.mm_desc.ioctl_param_off);

            bcm_pr_debug("BCMPH_IOCTL_START_MM\n");

            if (param->size != sizeof(param->p.start)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.start)));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_start(t, &(param->p.start));
            break;
         }
         case BCMPH_IOCTL_GET_MMAP_RBS_LOCATION: {
            bcm_pr_debug("BCMPH_IOCTL_GET_MMAP_RBS_LOCATION\n");

            if (_IOC_SIZE(cmd) != sizeof(t->core.mm_rbs_location)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(t->core.mm_rbs_location)));
               ret = -EFAULT;
               break;
            }
            if (!pcm_is_started(&(t->core.pcm))) {
               bcm_pr_err("%s(cmd=%u) : pcm is not started\n", __func__,
                  (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            if (copy_to_user((void *)(arg), &(t->core.mm_rbs_location), sizeof(t->core.mm_rbs_location)) > 0) {
               bcm_pr_err("%s(cmd=%u) : copy_to_user failed\n", __func__, (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            break;
         }
         case BCMPH_IOCTL_GET_MMAP_RBS_LOCATION_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->core.mm_buffer + t->dev.mm_desc.ioctl_param_off);

            bcm_pr_debug("BCMPH_IOCTL_GET_MMAP_RBS_LOCATION_MM\n");

            if (param->size != sizeof(param->p.get_mmap_rbs_location)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.get_mmap_rbs_location)));
               ret = -EFAULT;
               break;
            }
            if (!pcm_is_started(&(t->core.pcm))) {
               bcm_pr_err("%s(cmd=%u) : pcm is not started\n", __func__,
                  (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            memcpy(&(param->p.get_mmap_rbs_location), &(t->core.mm_rbs_location), sizeof(param->p.get_mmap_rbs_location));
            break;
         }
         case BCMPH_IOCTL_START_PCM: {
            d_bcm_pr_debug("BCMPH_IOCTL_START_PCM\n");

            bcm_drv_start_pcm(t);
            break;
         }
         case BCMPH_IOCTL_STOP_PCM: {
            d_bcm_pr_debug("BCMPH_IOCTL_STOP_PCM\n");

            bcm_drv_stop_pcm(t, true);
            break;
         }
         case BCMPH_IOCTL_GET_MMAP_DESC: {
            bcm_pr_debug("BCMPH_IOCTL_GET_MMAP_DESC\n");

            if (_IOC_SIZE(cmd) != sizeof(t->dev.mm_desc)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(t->dev.mm_desc)));
               ret = -EFAULT;
               break;
            }
            if (copy_to_user((void *)(arg), &(t->dev.mm_desc), sizeof(t->dev.mm_desc)) > 0) {
               bcm_pr_err("%s(cmd=%u) : copy_to_user failed\n", __func__, (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            break;
         }
         case BCMPH_IOCTL_SET_DEFAULT_LINE: {
            size_t line = arg;

            d_bcm_pr_debug("BCMPH_IOCTL_SET_DEFAULT_LINE\n");

            if ((line >= bcm_phone_mgr_get_line_count(&(t->core.phone_mgr)))
                || ((pcm_is_started(&(t->core.pcm))) && (!bcm_phone_mgr_line_is_enabled(&(t->core.phone_mgr), line)))) {
               ret = -EFAULT;
               break;
            }

            t->dev.default_line = line;
            break;
         }
         case BCMPH_IOCTL_READ: {
            bcm_phone_read_t read;

            d_bcm_pr_debug("BCMPH_IOCTL_READ\n");

            if (_IOC_SIZE(cmd) != sizeof(read)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(read)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(read), (void *)(arg), sizeof(read)) > 0) {
               bcm_pr_err("%s(cmd=%u) : copy_from_user failed\n", __func__, (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            if (read.line >= bcm_phone_mgr_get_line_count(&(t->core.phone_mgr))) {
               bcm_pr_err("%s(cmd=%u) : invalid line %lu\n", __func__,
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
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(write)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(write), (void *)(arg), sizeof(write)) > 0) {
               bcm_pr_err("%s(cmd=%u) : copy_from_user failed\n", __func__, (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            if (write.line >= bcm_phone_mgr_get_line_count(&(t->core.phone_mgr))) {
               bcm_pr_err("%s(cmd=%u) : invalid line %lu\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(write.line));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_write_line(t, write.do_not_block, &(lock), write.line, write.buf, write.count);
            break;
         }
         case BCMPH_IOCTL_GET_LINE_STATES_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->core.mm_buffer + t->dev.mm_desc.ioctl_param_off);

            dd_bcm_pr_debug("BCMPH_IOCTL_GET_LINE_STATES_MM\n");

            if (param->size != sizeof(param->p.get_line_states)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
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
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(ver)));
               ret = -EFAULT;
               break;
            }
            bcm_drv_read_version(t, &(ver));
            if (copy_to_user((void *)(arg), &(ver), sizeof(ver)) > 0) {
               bcm_pr_err("%s(cmd=%u) : copy_to_user failed\n", __func__, (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            break;
         }
         case BCMPH_IOCTL_READ_VERSION_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->core.mm_buffer + t->dev.mm_desc.ioctl_param_off);

            bcm_pr_debug("BCMPH_IOCTL_READ_VERSION_MM\n");

            if (param->size != sizeof(param->p.read_version)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
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
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(regs)));
               ret = -EFAULT;
               break;
            }
            pcm_read_regs(&(t->core.pcm), &(regs));
            if (copy_to_user((void *)(arg), &(regs), sizeof(regs)) > 0) {
               bcm_pr_err("%s(cmd=%u) : copy_to_user failed\n", __func__, (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            break;
         }
         case BCMPH_IOCTL_READ_PCM_REGS_MM: {
            // Reading register when PCM is started may return
            // inaccurate values for example slot_alloc_tbl[x]
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->core.mm_buffer + t->dev.mm_desc.ioctl_param_off);

            d_bcm_pr_debug("BCMPH_IOCTL_READ_PCM_REGS_MM\n");

            if (param->size != sizeof(param->p.read_pcm_regs)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.read_pcm_regs)));
               ret = -EFAULT;
               break;
            }
            pcm_read_regs(&(t->core.pcm), &(param->p.read_pcm_regs));
            break;
         }
         case BCMPH_IOCTL_READ_PCM_STATS: {
            bcm_phone_pcm_stats_t stats;

            d_bcm_pr_debug("BCMPH_IOCTL_READ_PCM_STATS\n");

            if (_IOC_SIZE(cmd) != sizeof(stats)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(stats)));
               ret = -EFAULT;
               break;
            }
            pcm_read_stats(&(t->core.pcm), &(stats));
            if (copy_to_user((void *)(arg), &(stats), sizeof(stats)) > 0) {
               bcm_pr_err("%s(cmd=%u) : copy_to_user failed\n", __func__, (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            break;
         }
         case BCMPH_IOCTL_READ_PCM_STATS_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->core.mm_buffer + t->dev.mm_desc.ioctl_param_off);

            d_bcm_pr_debug("BCMPH_IOCTL_READ_PCM_STATS_MM\n");

            if (param->size != sizeof(param->p.read_pcm_stats)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.read_pcm_stats)));
               ret = -EFAULT;
               break;
            }
            pcm_read_stats(&(t->core.pcm), &(param->p.read_pcm_stats));
            break;
         }
#ifdef BCMPH_TEST_PCM
         case BCMPH_IOCTL_SET_LINE_STATE: {
            bcm_phone_set_line_state_t set_line_state;

            dd_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_STATE\n");

            if (_IOC_SIZE(cmd) != sizeof(set_line_state)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(_IOC_SIZE(cmd)), (unsigned long)(sizeof(set_line_state)));
               ret = -EFAULT;
               break;
            }
            if (copy_from_user(&(set_line_state), (void *)(arg), sizeof(set_line_state)) > 0) {
               bcm_pr_err("%s(cmd=%u) : copy_from_user failed\n", __func__, (unsigned int)(cmd));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_state(t, &(set_line_state));
            break;
         }
         case BCMPH_IOCTL_SET_LINE_STATE_MM: {
            bcm_phone_ioctl_param_t *param = (bcm_phone_ioctl_param_t *)(t->core.mm_buffer + t->dev.mm_desc.ioctl_param_off);

            d_bcm_pr_debug("BCMPH_IOCTL_SET_LINE_STATE_MM\n");

            if (param->size != sizeof(param->p.set_line_state)) {
               bcm_pr_err("%s(cmd=%u) : invalid parameter size %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd), (unsigned long)(param->size), (unsigned long)(sizeof(param->p.set_line_state)));
               ret = -EFAULT;
               break;
            }
            ret = bcm_drv_set_line_state(t, &(param->p.set_line_state));
            break;
         }
#endif // BCMPH_TEST_PCM
         default: {
            bcm_pr_err("%s(cmd=%u) : no such command\n", __func__,
               (unsigned int)(cmd));
            ret = -ENOTTY;
            break;
         }
      }
   } while (false);

   bcmph_mutex_deinit(&(lock));

   if (ret) {
      d_bcm_pr_debug("%s(cmd %u) failed -> %d\n", __func__, (unsigned int)(cmd), (int)(ret));
   }

   return (ret);
}

#ifndef __KERNEL__
static bcm_drv_t *bcm_drv = NULL;
#endif /* !__KERNEL__ */

#ifdef __KERNEL__
static
#endif // __KERNEL__
int bcm_drv_open(struct inode *inode, struct file *filp)
{
   int ret = 0;
#ifdef __KERNEL__
   bcm_drv_t *t = container_of(filp->private_data, bcm_drv_t, dev.misc_dev);
#else /* !__KERNEL__ */
   bcm_drv_t *t = bcm_drv;
#endif /* !__KERNEL__ */
   bcmph_mutex_t lock;

   bcm_pr_debug("%s()\n", __func__);

   bcmph_mutex_init(&(lock), &(t->dev.lock), false);

   do { // Empty loop
      if (bcmph_mutex_lock_interruptible(&(lock))) {
         ret = -ERESTARTSYS;
         break;
      }

      if (0 != t->dev.open_count) {
         /* -EPERM would confuse the user */
         bcm_pr_debug("Open() failed because open_count != 0 (%ld)\n", (long)(t->dev.open_count));
         ret = -EBUSY;
         break;
      }

      t->dev.open_count += 1;

      ret = nonseekable_open(inode, filp);
      if (ret) {
         break;
      }

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

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert(NULL != t);

   bcmph_mutex_init(&(lock), &(t->dev.lock), false);

   if (!bcmph_mutex_lock_interruptible(&(lock))) {
      bcm_drv_stop(t, false);
      t->dev.open_count -= 1;
   }
   else {
      ret = -ERESTARTSYS;
   }

   bcmph_mutex_deinit(&(lock));

   return (ret);
}

#ifdef __KERNEL__

#include <linux/version.h>

/* Dirty? Well, I still did not learn better way to account
 * for user mmaps.
 */

static void bcm_drv_mm_open(struct vm_area_struct *vma)
{
   bcm_pr_debug("%s()\n", __func__);
}

static void bcm_drv_mm_close(struct vm_area_struct *vma)
{
   bcm_pr_debug("%s()\n", __func__);
}

static int bcm_drv_mm_fault(
#if (LINUX_VERSION_CODE <= KERNEL_VERSION(4, 11, 0))
    struct vm_area_struct *vma,
#endif
    struct vm_fault *vmf)
{
   bcm_pr_debug("%s()\n", __func__);
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

   bcm_pr_debug("%s()", __func__);

   bcm_assert(NULL != t);

   bcmph_mutex_init(&(lock), &(t->dev.lock), false);

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

      pg_count = 1 << t->core.mm_buffer_order;

      if ((vma->vm_end - vma->vm_start) != (PAGE_SIZE * pg_count)) {
         ret = -EINVAL;
         break;
      }

      start = vma->vm_start;
      kaddr = t->core.mm_buffer;
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

int __init bcm_drv_init(bcm_drv_t *t)
{
   int ret = -1;

   bcm_pr_debug("%s()\n", __func__);

   // mutex_init() should be called before initializing the wait queues
   mutex_init(&(t->dev.lock));

   bcm_pr_debug("Initializing RX wait queue\n");
   bcm_wait_queue_init(&(t->dev.inq));
   bcm_pr_debug("Initializing TX wait queue\n");
   bcm_wait_queue_init(&(t->dev.outq));

   // As we have to register the device before calling pcm_init,
   // we set open_count to -1 so open() will fail if it is called before
   // init function terminates successfully
   t->dev.open_count = -1;

#ifdef __KERNEL__
   /* register device with kernel */
   memset(&(t->dev.misc_dev), 0, sizeof(t->dev.misc_dev));
   t->dev.misc_dev.minor = MISC_DYNAMIC_MINOR;
   t->dev.misc_dev.name = driver_name;
   t->dev.misc_dev.fops = &(bcm_drv_fops);

   bcm_pr_debug("Registering device %s\n", t->dev.misc_dev.name);
   ret = misc_register(&(t->dev.misc_dev));
   if (ret) {
      bcm_pr_err("Can't register device\n");
      goto fail_register_dev;
   }
#else /* !__KERNEL__ */
   bcm_drv = t;
#endif /* !__KERNEL__ */

   return (0);

#ifdef __KERNEL__
   bcm_pr_debug("Unregistering device\n");
   misc_deregister(&(t->dev.misc_dev));
fail_register_dev:
#endif // __KERNEL__
#ifndef __KERNEL__
   bcm_drv = NULL;
#endif /* !__KERNEL__ */
   bcm_wait_queue_deinit(&(t->dev.outq));
   bcm_wait_queue_deinit(&(t->dev.inq));
   mutex_destroy(&(t->dev.lock));
   return (ret);
}

void bcm_drv_exit(bcm_drv_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   bcm_pr_debug("Unregistering device\n");
#ifdef __KERNEL__
   misc_deregister(&(t->dev.misc_dev));
#endif // __KERNEL__
#ifndef __KERNEL__
   bcm_drv = NULL;
#endif /* !__KERNEL__ */
   t->dev.open_count = -1;
   bcm_wait_queue_deinit(&(t->dev.outq));
   bcm_wait_queue_deinit(&(t->dev.inq));
   mutex_destroy(&(t->dev.lock));
}

#endif /* BCMPH_EXPORT_DEV_FILE */
