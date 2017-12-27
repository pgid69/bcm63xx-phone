/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __BCM63XX_RING_BUF_H__
#define __BCM63XX_RING_BUF_H__

#ifdef __KERNEL__
#include <linux/uaccess.h>
#else /* !__KERNEL__ */
#include <linux/types.h>
#include <stddef.h>
#include <string.h>
#endif /* !__KERNEL__ */

#include "bcm63xx_log.h"

typedef struct {
   size_t len;
   size_t off;
   size_t size;
} bcm_ring_buf_desc_t;

static inline void bcm_ring_buf_desc_copy(const bcm_ring_buf_desc_t *src, bcm_ring_buf_desc_t *dest)
{
   dd_bcm_pr_debug("%s()\n", __func__);
   bcm_assert((NULL != src) && (NULL != dest));
   memcpy(dest, src, sizeof(*dest));
}

typedef struct {
   __u8 *buf;
   bcm_ring_buf_desc_t desc;
} bcm_ring_buf_t;

static inline void bcm_ring_buf_init(bcm_ring_buf_t *rb, __u8 *buf, size_t len)
{
   d_bcm_pr_debug("%s(len=%lu)\n",  __func__, (unsigned long)(len));

   bcm_assert((NULL != buf) && (len > 0));

   rb->buf = buf;
   rb->desc.len = len;
   rb->desc.off = 0;
   rb->desc.size = 0;
}

static inline void bcm_ring_buf_get_desc(const bcm_ring_buf_t *rb, bcm_ring_buf_desc_t *desc)
{
   dd_bcm_pr_debug("%s()\n", __func__);
   bcm_assert(NULL != desc);
   bcm_ring_buf_desc_copy(&(rb->desc), desc);
}

static inline size_t bcm_ring_buf_get_len(const bcm_ring_buf_t *rb)
{
   size_t ret = rb->desc.len;
   dd_bcm_pr_debug("%s() -> %lu\n", __func__, (unsigned long)(ret));
   return (ret);
}

static inline void bcm_ring_buf_clear(bcm_ring_buf_t *rb)
{
   dd_bcm_pr_debug("%s()\n", __func__);
   rb->desc.off = 0;
   rb->desc.size = 0;
}

static inline size_t bcm_ring_buf_get_size(const bcm_ring_buf_t *rb)
{
   size_t ret = rb->desc.size;
   dd_bcm_pr_debug("%s() -> %lu\n", __func__, (unsigned long)(ret));
   return (ret);
}

static inline size_t bcm_ring_buf_get_free_space(const bcm_ring_buf_t *rb)
{
   size_t ret = rb->desc.len - rb->desc.size;
   dd_bcm_pr_debug("%s() -> %lu\n", __func__, (unsigned long)(ret));
   return (ret);
}

static inline void bcm_ring_buf_add_len(bcm_ring_buf_t *rb, size_t len)
{
   dd_bcm_pr_debug("%s(len=%lu)\n", __func__, (unsigned long)(len));

   bcm_assert((len <= bcm_ring_buf_get_free_space(rb)) && (rb->desc.off < rb->desc.len));

   rb->desc.size += len;
}

extern void bcm_ring_buf_add(bcm_ring_buf_t *rb, const __u8 *src, size_t len);

static inline void bcm_ring_buf_add_u8(bcm_ring_buf_t *rb, __u8 data)
{
   size_t to;

   dd_bcm_pr_debug("%s()\n", __func__);

   bcm_assert((1 <= bcm_ring_buf_get_free_space(rb)) && (rb->desc.off < rb->desc.len));

   to = rb->desc.off + bcm_ring_buf_get_size(rb);
   if (to >= rb->desc.len) {
      to -= rb->desc.len;
   }
   rb->buf[to] = data;
   rb->desc.size += 1;
}

/* Same a bcm_ring_buf_add but do not update the size.
 Must call bcm_ring_buf_add_len() to really add the data */
extern void bcm_ring_buf_store(bcm_ring_buf_t *rb, const __u8 *src, size_t len);

/* Same a bcm_ring_buf_add_u8 but do not update the size.
 Must call bcm_ring_buf_add_len() to really add the data */
static inline void bcm_ring_buf_store_u8(bcm_ring_buf_t *rb, __u8 data)
{
   size_t to;

   dd_bcm_pr_debug("%s()\n", __func__);

   bcm_assert((1 <= bcm_ring_buf_get_free_space(rb)) && (rb->desc.off < rb->desc.len));

   to = rb->desc.off + bcm_ring_buf_get_size(rb);
   if (to >= rb->desc.len) {
      to -= rb->desc.len;
   }
   rb->buf[to] = data;
}

/*
 Returns a direct pointer on first byte of free space in the ring buffer.
 Set 'len_max' to the size of the consecutive bytes of data that can be written
 (<= bcm_ring_buf_get_free_space())
*/
static inline __u8 *bcm_ring_buf_get_ptr_store(bcm_ring_buf_t *rb, size_t *len_max)
{
   size_t to;
   size_t len;

   dd_bcm_pr_debug("%s()\n", __func__);
   bcm_assert(NULL != len_max);

   to = rb->desc.off + bcm_ring_buf_get_size(rb);
   if (to >= rb->desc.len) {
      to -= rb->desc.len;
   }
   len = bcm_ring_buf_get_free_space(rb);
   if ((to + len) > rb->desc.len) {
      len = rb->desc.len - to;
   }
   *len_max = len;
   return (rb->buf + to);
}

#ifdef __KERNEL__
extern int bcm_ring_buf_add_from_user(bcm_ring_buf_t *rb, const __u8 *src, size_t len);
#else /* !__KERNEL__ */
static inline int bcm_ring_buf_add_from_user(bcm_ring_buf_t *rb, const __u8 *src, size_t len)
{
   bcm_ring_buf_add(rb, src, len);
   return (0);
}
#endif /* !__KERNEL__ */

static inline void bcm_ring_buf_remove_len(bcm_ring_buf_t *rb, size_t len)
{
   dd_bcm_pr_debug("%s(len=%lu)\n", __func__, (unsigned long)(len));

   bcm_assert((len <= bcm_ring_buf_get_size(rb)) && (rb->desc.off < rb->desc.len));

   rb->desc.size -= len;
   rb->desc.off += len;
   if (rb->desc.off >= rb->desc.len) {
      rb->desc.off -= rb->desc.len;
      bcm_assert(rb->desc.off < rb->desc.len);
   }
}

extern void bcm_ring_buf_remove(bcm_ring_buf_t *rb, __u8 *dst, size_t len);

static inline __u8 bcm_ring_buf_remove_u8(bcm_ring_buf_t *rb)
{
   __u8 ret;

   dd_bcm_pr_debug("%s()\n", __func__);

   bcm_assert((1 <= bcm_ring_buf_get_size(rb)) && (rb->desc.off < rb->desc.len));

   ret = rb->buf[rb->desc.off];
   rb->desc.off += 1;
   if (rb->desc.off >= rb->desc.len) {
      bcm_assert(rb->desc.off == rb->desc.len);
      rb->desc.off = 0;
   }

   return (ret);
}

#ifdef __KERNEL__
extern int bcm_ring_buf_remove_to_user(bcm_ring_buf_t *rb, __u8 *dst, size_t len);
#else /* !__KERNEL__ */
static inline int bcm_ring_buf_remove_to_user(bcm_ring_buf_t *rb, __u8 *dst, size_t len)
{
   bcm_ring_buf_remove(rb, dst, len);
   return (0);
}
#endif /* !__KERNEL__ */

/* Same a bcm_ring_buf_remove but leave data in the ring buffer.
 Must call bcm_ring_buf_remove_len() to remove the data */
extern void bcm_ring_buf_load(bcm_ring_buf_t *rb, size_t offset,
   __u8 *dst, size_t len);

/* Same a bcm_ring_buf_remove_u8 but leave data in the ring buffer.
 Must call bcm_ring_buf_remove_len() to remove the data */
static inline __u8 bcm_ring_buf_load_u8(bcm_ring_buf_t *rb, size_t offset)
{
   dd_bcm_pr_debug("%s()\n", __func__);

   bcm_assert((offset < bcm_ring_buf_get_size(rb))
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
   return (rb->buf[offset]);
}

/*
 Returns a direct pointer on byte at offset 'offset' in the ring buffer.
 Set 'len_max' to the size of the consecutive bytes of data that can be read
 (<= bcm_ring_buf_get_size())
*/
static inline const __u8 *bcm_ring_buf_get_ptr_load(
   const bcm_ring_buf_t *rb, size_t offset, size_t *len_max)
{
   size_t len;

   dd_bcm_pr_debug("%s()\n", __func__);
   bcm_assert(NULL != len_max);

   len = bcm_ring_buf_get_size(rb);
   if (0 != offset) {
      bcm_assert(offset <= len);
      len -= offset;
      offset += rb->desc.off;
      if (offset >= rb->desc.len) {
         offset -= rb->desc.len;
      }
   }
   else {
      offset = rb->desc.off;
   }
   if ((offset + len) > rb->desc.len) {
      len = rb->desc.len - offset;
   }
   *len_max = len;

   return (rb->buf + offset);
}

#endif /* __BCM63XX_RING_BUF_H__ */
