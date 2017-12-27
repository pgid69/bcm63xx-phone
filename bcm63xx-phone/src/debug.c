/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include <config.h>

#if ((defined __KERNEL__) && (defined BCMPH_NOHW))

#include <bcm63xx_phone.h>
#include <bcm63xx_log.h>
#include <bcm63xx_ring_buf.h>

#include "main.h"

// Include after system files
#include <compile.h>

static ssize_t bcm_debug_read(struct file *filp, char *buf, size_t count,
   loff_t *offset)
{
   ssize_t ret = 0;
   bcm_drv_t *t = (bcm_drv_t *)(filp->private_data);
   bcmph_mutex_t lock;

   dd_bcm_pr_debug("%s(buf=%lx, count=%lu)\n", __func__, (unsigned long)(buf), (unsigned long)(count));

   bcm_assert(NULL != t);

#ifndef BCMPH_EXPORT_DEV_FILE
   bcmph_mutex_init(&(lock), &(t->debug.lock), false);
#else // BCMPH_EXPORT_DEV_FILE
   bcmph_mutex_init(&(lock), &(t->dev.lock), false);
#endif // BCMPH_EXPORT_DEV_FILE

   if (bcmph_mutex_lock_interruptible(&(lock))) {
      ret = -ERESTARTSYS;
   }
   else {
      if (count > 0) {
         if (!access_ok(VERIFY_WRITE, buf, count)) {
            ret = -EFAULT;
         }
         else {
            size_t len = bcm_ring_buf_get_size(&(t->debug.read_ring_buf));
            if (len > 0) {
               // RX buffer is not empty, so we can read data from it
               dd_bcm_pr_debug("Reading from read ring buffer that contains %lu bytes\n", (unsigned long)(len));
               if (len > count) {
                  len = count;
               }
               if (bcm_ring_buf_remove_to_user(&(t->debug.read_ring_buf), buf, len)) {
                  bcm_pr_debug("Read() stopped because copy_to_user() failed\n");
                  ret = -EFAULT;
               }
               else {
                  ret = len;
#ifdef BCMPH_DEBUG
                  t->debug.stats.bytes_tx_to_user += ret;
#endif // BCMPH_DEBUG
               }
            }
            else {
               // We are not allowed to block so we exit
               ret = -EAGAIN;
               dd_bcm_pr_debug("Nothing read, returned %ld\n", (long)(ret));
            }
         }
      }
   }

   if (ret > 0) {
      bcm_assert((0 == (ret % sizeof(__s16))) && (((size_t)(ret)) <= count));
      if (NULL != offset) {
         *offset += ret;
      }
   }

   bcmph_mutex_deinit(&(lock));

   return (ret);
}

static ssize_t bcm_debug_write(struct file *filp, const char *buf,
            size_t count, loff_t *offset)
{
   ssize_t ret = 0;
   bcm_drv_t *t = (bcm_drv_t *)(filp->private_data);
   bcmph_mutex_t lock;

   dd_bcm_pr_debug("%s(buf=%lx, count=%lu)\n", __func__, (unsigned long)(buf), (unsigned long)(count));

   bcm_assert(NULL != t);

# ifndef BCMPH_EXPORT_DEV_FILE
   bcmph_mutex_init(&(lock), &(t->debug.lock), false);
#else // BCMPH_EXPORT_DEV_FILE
   bcmph_mutex_init(&(lock), &(t->dev.lock), false);
#endif // BCMPH_EXPORT_DEV_FILE

   if (bcmph_mutex_lock_interruptible(&(lock))) {
      ret = -ERESTARTSYS;
   }
   else {
      if (count > 0) {
         if (!access_ok(VERIFY_READ, buf, count)) {
            ret = -EFAULT;
         }
         else {
            size_t len;

            len = bcm_ring_buf_get_free_space(&(t->debug.write_ring_buf));
            if (len > 0) {
               if (len > count) {
                  len = count;
               }
               dd_bcm_pr_debug("Copying %lu bytes into write ring buffer, from user buffer\n", (unsigned long)(len));
               if (bcm_ring_buf_add_from_user(&(t->debug.write_ring_buf), buf, len)) {
                  bcm_pr_debug("Write() stopped because copy_from_user() failed\n");
                  ret = -EFAULT;
               }
               else {
                  ret = len;
#ifdef BCMPH_DEBUG
                  t->debug.stats.bytes_rx_from_user += ret;
#endif // BCMPH_DEBUG
               }
            }
            else {
               // We are not allowed to block so we exit
               ret = -EAGAIN;
               dd_bcm_pr_debug("Nothing written, returned %ld\n", (long)(ret));
            }
         }
      }
   }

   if (ret > 0) {
      bcm_assert((0 == (ret % sizeof(__s16))) && (((size_t)(ret)) <= count));
      if (NULL != offset) {
         *offset += ret;
      }
   }

   bcmph_mutex_deinit(&(lock));

   return (ret);
}

static long bcm_debug_unlocked_ioctl(struct file *filp,
             unsigned int cmd, unsigned long arg)
{
   long ret = 0;
   bcm_drv_t *t = (bcm_drv_t *)(filp->private_data);
   bcmph_mutex_t lock;

   dd_bcm_pr_debug("%s(cmd %u)\n", __func__, (unsigned int)(cmd));

   bcm_assert(NULL != t);

#ifndef BCMPH_EXPORT_DEV_FILE
   bcmph_mutex_init(&(lock), &(t->debug.lock), false);
#else // BCMPH_EXPORT_DEV_FILE
   bcmph_mutex_init(&(lock), &(t->dev.lock), false);
#endif // BCMPH_EXPORT_DEV_FILE

   do { // Boucle vide
      switch (cmd) {
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
#ifdef BCMPH_EXPORT_DEV_FILE
            if (t->dev.default_line != set_line_state.line) {
#else // !BCMPH_EXPORT_DEV_FILE
            if (BCMPH_DEFAULT_LINE != set_line_state.line) {
#endif // !BCMPH_EXPORT_DEV_FILE
               bcm_pr_err("%s(cmd=%u) : invalid line configured %lu (%lu expected)\n", __func__,
                  (unsigned int)(cmd)
#ifdef BCMPH_EXPORT_DEV_FILE
                  , (unsigned long)(set_line_state.line), (unsigned long)(t->dev.default_line)
#else // !BCMPH_EXPORT_DEV_FILE
                  , (unsigned long)(set_line_state.line), (unsigned long)(BCMPH_DEFAULT_LINE)
#endif // !BCMPH_EXPORT_DEV_FILE
               );
               ret = -EFAULT;
            }
            ret = bcm_drv_set_line_state(t, &(set_line_state));
            break;
         }
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

static int bcm_debug_open(struct inode *inode, struct file *filp)
{
   int ret = 0;
   bcm_drv_t *t = container_of(filp->private_data, bcm_drv_t, debug.misc_dev);
   bcmph_mutex_t lock;

   bcm_pr_debug("%s()\n", __func__);

#ifndef BCMPH_EXPORT_DEV_FILE
   bcmph_mutex_init(&(lock), &(t->debug.lock), false);
#else // BCMPH_EXPORT_DEV_FILE
   bcmph_mutex_init(&(lock), &(t->dev.lock), false);
#endif // BCMPH_EXPORT_DEV_FILE

   do { // Empty loop
      if (bcmph_mutex_lock_interruptible(&(lock))) {
         ret = -ERESTARTSYS;
         break;
      }

      if (0 != t->debug.open_count) {
         /* -EPERM would confuse the user */
         bcm_pr_debug("Open() failed because open_count != 0 (%ld)\n", (long)(t->debug.open_count));
         ret = -EBUSY;
         break;
      }

      t->debug.open_count += 1;
#ifdef BCMPH_DEBUG
      t->debug.stats.bytes_rx_from_user = 0;
      t->debug.stats.bytes_tx_to_user = 0;
#endif // BCMPH_DEBUG

      ret = nonseekable_open(inode, filp);
      if (ret) {
         break;
      }

      filp->private_data = t;
   } while (false);

   bcmph_mutex_deinit(&(lock));

   return (ret);
}

static int bcm_debug_release(struct inode *inode, struct file *filp)
{
   int ret = 0;
   bcm_drv_t *t = (bcm_drv_t *)(filp->private_data);
   bcmph_mutex_t lock;

   bcm_pr_debug("%s()\n", __func__);

   bcm_assert(NULL != t);

#ifndef BCMPH_EXPORT_DEV_FILE
   bcmph_mutex_init(&(lock), &(t->debug.lock), false);
#else // BCMPH_EXPORT_DEV_FILE
   bcmph_mutex_init(&(lock), &(t->dev.lock), false);
#endif // BCMPH_EXPORT_DEV_FILE

   if (!bcmph_mutex_lock_interruptible(&(lock))) {
      t->debug.open_count -= 1;
   }
   else {
      ret = -ERESTARTSYS;
   }

#ifdef BCMPH_DEBUG
# ifdef BCMPH_DAHDI_DRIVER
   bcm_dahdi_pr_stats(t);
# endif /* BCMPH_DAHDI_DRIVER */
   bcm_pr_info("bytes_rx_from_user=%lu, bytes_tx_to_user=%lu\n",
      (unsigned long)(t->debug.stats.bytes_rx_from_user),
      (unsigned long)(t->debug.stats.bytes_tx_to_user));
#endif /* BCMPH_DEBUG */

   bcmph_mutex_deinit(&(lock));

   return (ret);
}

static struct file_operations bcm_debug_fops = {
   .owner          = THIS_MODULE,
   .read           = bcm_debug_read,
   .write          = bcm_debug_write,
   .unlocked_ioctl = bcm_debug_unlocked_ioctl,
   .open           = bcm_debug_open,
   .release        = bcm_debug_release,
};

int __init bcm_debug_init(bcm_drv_t *t)
{
   int ret = 0;
   size_t size;

   bcm_pr_debug("%s()\n", __func__);

   /* allocate some memory for ring buffer */
   size = 60 * BCMPH_SAMPLES_PER_MS * sizeof(__s16);
   t->debug.buf = (__u8 *)(kmalloc(size * 2, GFP_KERNEL | __GFP_NORETRY));
   if (NULL == t->debug.buf) {
      bcm_pr_err("Cannot allocate memory for ring buffer\n");
      ret = -ENOMEM;
      goto fail_alloc;
   }
   bcm_pr_debug("%lu bytes of memory allocated, starting at address 0x%lx\n", (unsigned long)(size), (unsigned long)(t->debug.buf));

   bcm_ring_buf_init(&(t->debug.read_ring_buf), t->debug.buf, size);
   bcm_ring_buf_init(&(t->debug.write_ring_buf), t->debug.buf + size, size);

#ifndef BCMPH_EXPORT_DEV_FILE
   mutex_init(&(t->debug.lock));
#endif // !BCMPH_EXPORT_DEV_FILE

   // we set open_count to -1 so open() will fail if it is called before
   // init function terminates successfully
   t->debug.open_count = -1;

   /* register device with kernel */
   memset(&(t->debug.misc_dev), 0, sizeof(t->debug.misc_dev));
   t->debug.misc_dev.minor = MISC_DYNAMIC_MINOR;
   t->debug.misc_dev.name = "bcm63xx_phone_debug";
   t->debug.misc_dev.fops = &(bcm_debug_fops);

   bcm_pr_debug("Registering device %s\n", t->debug.misc_dev.name);
   ret = misc_register(&(t->debug.misc_dev));
   if (ret) {
      bcm_pr_err("Can't register device\n");
      goto fail_register_dev;
   }

   return (0);

   bcm_pr_debug("Unregistering device %s\n", t->debug.misc_dev.name);
   misc_deregister(&(t->debug.misc_dev));
fail_register_dev:
#ifndef BCMPH_EXPORT_DEV_FILE
   mutex_destroy(&(t->debug.lock));
#endif // !BCMPH_EXPORT_DEV_FILE
   kfree(t->debug.buf);
   t->debug.buf = NULL;
fail_alloc:
   return (ret);
}

void bcm_debug_exit(bcm_drv_t *t)
{
   bcm_pr_debug("%s()\n", __func__);

   bcm_pr_debug("Unregistering device %s\n", t->debug.misc_dev.name);
   misc_deregister(&(t->debug.misc_dev));
   t->debug.open_count = -1;
#ifndef BCMPH_EXPORT_DEV_FILE
   mutex_destroy(&(t->debug.lock));
#endif // !BCMPH_EXPORT_DEV_FILE
   kfree(t->debug.buf);
   t->debug.buf = NULL;
}

#endif /* __KERNEL__ && BCMPH_NOHW */
