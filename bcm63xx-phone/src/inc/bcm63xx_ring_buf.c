/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#include "bcm63xx_ring_buf.h"

void bcm_ring_buf_store(bcm_ring_buf_t *rb, const __u8 *src, size_t len)
{
   size_t to;

   dd_bcm_pr_debug("%s(len=%lu)\n", __func__, (unsigned long)(len));

   bcm_assert((NULL != src) && (len <= bcm_ring_buf_get_free_space(rb)) && (rb->desc.off < rb->desc.len));

   to = rb->desc.off + bcm_ring_buf_get_size(rb);
   if (to >= rb->desc.len) {
      to -= rb->desc.len;
   }
   if ((to + len) > rb->desc.len) {
      size_t tmp = rb->desc.len - to;
      memcpy(&(rb->buf[to]), src, tmp);
      src += tmp;
      len -= tmp;
      to = 0;
   }
   memcpy(&(rb->buf[to]), src, len);
}

void bcm_ring_buf_add(bcm_ring_buf_t *rb, const __u8 *src, size_t len)
{
   size_t to;

   dd_bcm_pr_debug("%s(len=%lu)\n", __func__, (unsigned long)(len));

   bcm_assert((NULL != src) && (len <= bcm_ring_buf_get_free_space(rb)) && (rb->desc.off < rb->desc.len));

   to = rb->desc.off + bcm_ring_buf_get_size(rb);
   if (to >= rb->desc.len) {
      to -= rb->desc.len;
   }
   if ((to + len) > rb->desc.len) {
      size_t tmp = rb->desc.len - to;
      memcpy(&(rb->buf[to]), src, tmp);
      src += tmp;
      len -= tmp;
      to = 0;
      rb->desc.size += tmp;
   }
   memcpy(&(rb->buf[to]), src, len);
   rb->desc.size += len;
}

#ifdef __KERNEL__

#include <linux/uaccess.h>
/*
 Use __copy_from_user to copy data from to user buffer to ring
 access_ok(VERIFY_READ) on user buffer should have been called before
*/
int bcm_ring_buf_add_from_user(bcm_ring_buf_t *rb, const __u8 *src, size_t len)
{
   int ret;
   size_t to;
   size_t initial_size = bcm_ring_buf_get_size(rb);

   dd_bcm_pr_debug("%s(len=%lu)\n", __func__, (unsigned long)(len));

   bcm_assert((NULL != src) && (len <= bcm_ring_buf_get_free_space(rb)) && (rb->desc.off < rb->desc.len));

   do { /* Empty loop */
      to = rb->desc.off + bcm_ring_buf_get_size(rb);
      if (to >= rb->desc.len) {
         to -= rb->desc.len;
      }
      if ((to + len) > rb->desc.len) {
         size_t tmp = rb->desc.len - to;
         ret = __copy_from_user(&(rb->buf[to]), src, tmp);
         if (ret > 0) {
            break;
         }
         src += tmp;
         len -= tmp;
         to = 0;
         rb->desc.size += tmp;
      }
      ret = __copy_from_user(&(rb->buf[to]), src, len);
      if (ret > 0) {
         rb->desc.size = initial_size;
         break;
      }
      rb->desc.size += len;
      ret = 0;
   } while (0);

   return (ret);
}
#endif /* __KERNEL__ */

void bcm_ring_buf_load(bcm_ring_buf_t *rb, size_t offset, __u8 *dst, size_t len)
{
   dd_bcm_pr_debug("%s(len=%lu)\n", __func__, (unsigned long)(len));

   bcm_assert((NULL != dst)
      && ((offset + len) <= bcm_ring_buf_get_size(rb))
      && (rb->desc.off < rb->desc.len));

   if (0 != offset) {
      offset += rb->desc.off;
      if (offset >= rb->desc.len) {
         offset -= rb->desc.len;
      }
   }
   else {
      offset = rb->desc.off;
   }
   if ((offset + len) > rb->desc.len) {
      size_t tmp = rb->desc.len - offset;
      memcpy(dst, &(rb->buf[offset]), tmp);
      dst += tmp;
      len -= tmp;
      offset = 0;
   }
   memcpy(dst, &(rb->buf[offset]), len);
}

void bcm_ring_buf_remove(bcm_ring_buf_t *rb, __u8 *dst, size_t len)
{
   dd_bcm_pr_debug("%s(len=%lu)\n", __func__, (unsigned long)(len));

   bcm_assert((NULL != dst) && (len <= bcm_ring_buf_get_size(rb)) && (rb->desc.off < rb->desc.len));

   if ((rb->desc.off + len) > rb->desc.len) {
      size_t tmp = rb->desc.len - rb->desc.off;
      memcpy(dst, &(rb->buf[rb->desc.off]), tmp);
      dst += tmp;
      len -= tmp;
      rb->desc.size -= tmp;
      rb->desc.off = 0;
   }
   memcpy(dst, &(rb->buf[rb->desc.off]), len);
   rb->desc.size -= len;
   rb->desc.off += len;
   if (rb->desc.off >= rb->desc.len) {
      bcm_assert(rb->desc.off == rb->desc.len);
      rb->desc.off = 0;
   }
}

#ifdef __KERNEL__
/*
 Use __copy_to_user to copy data removed from ring, to user memory
 access_ok(VERIFY_WRITE) on user buffer should have been called before
*/
int bcm_ring_buf_remove_to_user(bcm_ring_buf_t *rb, __u8 *dst, size_t len)
{
   int ret;
   size_t initial_size = bcm_ring_buf_get_size(rb);
   size_t initial_offset = rb->desc.off;

   dd_bcm_pr_debug("%s(len=%lu)\n", __func__, (unsigned long)(len));

   bcm_assert((NULL != dst) && (len <= bcm_ring_buf_get_size(rb)) && (rb->desc.off < rb->desc.len));

   do { /* Empty loop */
      if ((rb->desc.off + len) > rb->desc.len) {
         size_t tmp = rb->desc.len - rb->desc.off;
         ret = __copy_to_user(dst, &(rb->buf[rb->desc.off]), tmp);
         if (ret > 0) {
            break;
         }
         dst += tmp;
         len -= tmp;
         rb->desc.size -= tmp;
         rb->desc.off = 0;
      }
      ret = __copy_to_user(dst, &(rb->buf[rb->desc.off]), len);
      if (ret > 0) {
         rb->desc.size = initial_size;
         rb->desc.off = initial_offset;
         break;
      }
      rb->desc.size -= len;
      rb->desc.off += len;
      if (rb->desc.off >= rb->desc.len) {
         bcm_assert(rb->desc.off == rb->desc.len);
         rb->desc.off = 0;
      }
      ret = 0;
   } while (0);

   return (ret);
}
#endif /* __KERNEL__ */
