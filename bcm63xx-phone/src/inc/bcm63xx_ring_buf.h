/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@gilande.com>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifndef __BCM63XX_RING_BUF_H__
#define __BCM63XX_RING_BUF_H__

#ifdef __KERNEL__
#include <asm/uaccess.h>
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
   dd_bcm_pr_debug("bcm_ring_buf_desc_copy()\n");
   bcm_assert((NULL != src) && (NULL != dest));
   memcpy(dest, src, sizeof(*dest));
}

typedef struct {
   __u8 *buf;
   bcm_ring_buf_desc_t desc;
} bcm_ring_buf_t;

static inline void bcm_ring_buf_init(bcm_ring_buf_t *rb, __u8 *buf, size_t len)
{
   d_bcm_pr_debug("bcm_ring_buf_init(len=%lu)\n", (unsigned long)(len));

   bcm_assert((NULL != buf) && (len > 0));

   rb->buf = buf;
   rb->desc.len = len;
   rb->desc.off = 0;
   rb->desc.size = 0;
}

static inline void bcm_ring_buf_get_desc(const bcm_ring_buf_t *rb, bcm_ring_buf_desc_t *desc)
{
   dd_bcm_pr_debug("bcm_ring_buf_get_desc()\n");
   bcm_assert(NULL != desc);
   bcm_ring_buf_desc_copy(&(rb->desc), desc);
}

static inline size_t bcm_ring_buf_get_len(const bcm_ring_buf_t *rb)
{
   size_t ret = rb->desc.len;
   dd_bcm_pr_debug("bcm_ring_buf_get_len() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

static inline void bcm_ring_buf_clear(bcm_ring_buf_t *rb)
{
   dd_bcm_pr_debug("bcm_ring_buf_clear()\n");
   rb->desc.off = 0;
   rb->desc.size = 0;
}

static inline size_t bcm_ring_buf_get_size(const bcm_ring_buf_t *rb)
{
   size_t ret = rb->desc.size;
   dd_bcm_pr_debug("bcm_ring_buf_get_size() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

static inline size_t bcm_ring_buf_get_free_space(const bcm_ring_buf_t *rb)
{
   size_t ret = rb->desc.len - rb->desc.size;
   dd_bcm_pr_debug("bcm_ring_buf_get_free_space() -> %lu\n", (unsigned long)(ret));
   return (ret);
}

static inline void bcm_ring_buf_add_len(bcm_ring_buf_t *rb, size_t len)
{
   dd_bcm_pr_debug("bcm_ring_buf_add_len(len=%lu)\n", (unsigned long)(len));

   bcm_assert((len <= bcm_ring_buf_get_free_space(rb)) && (rb->desc.off < rb->desc.len));

   rb->desc.size += len;
}

extern void bcm_ring_buf_add(bcm_ring_buf_t *rb, const __u8 *src, size_t len);

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
   dd_bcm_pr_debug("bcm_ring_buf_remove_len(len=%lu)\n", (unsigned long)(len));

   bcm_assert((len <= bcm_ring_buf_get_size(rb)) && (rb->desc.off < rb->desc.len));

   rb->desc.size -= len;
   rb->desc.off += len;
   if (rb->desc.off >= rb->desc.len) {
      rb->desc.off -= rb->desc.len;
      bcm_assert(rb->desc.off < rb->desc.len);
   }
}

extern void bcm_ring_buf_remove(bcm_ring_buf_t *rb, __u8 *dst, size_t len);

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
extern void bcm_ring_buf_peek(bcm_ring_buf_t *rb, __u8 *dst, size_t len);

/*
 Returns a direct pointer on data present in the ring buffer.
 Store in len_max the size of the consecutive bytes of data that can be read
 (<= bcm_ring_buf_get_size())
*/
static inline __u8 *bcm_ring_buf_get_ptr_peek(bcm_ring_buf_t *rb, size_t *len_max)
{
   size_t len;

   dd_bcm_pr_debug("bcm_ring_buf_get_ptr_peek()\n");
   bcm_assert(NULL != len_max);

   len = rb->desc.size;
   if ((rb->desc.off + len) > rb->desc.len) {
      len = rb->desc.len - rb->desc.off;
   }
   *len_max = len;
   return (rb->buf + rb->desc.off);
}

#endif /* __BCM63XX_RING_BUF_H__ */
