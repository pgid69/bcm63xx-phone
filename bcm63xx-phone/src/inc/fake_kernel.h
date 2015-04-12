/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@gilande.com>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifndef __FAKE_KERNEL_H__
#define __FAKE_KERNEL_H__

#ifndef __KERNEL__

/* Define types and functions available only when compiling kernel code */
#include <linux/types.h>
#include <sys/types.h>

#include <errno.h>
#include <limits.h>
#include <malloc.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>

#include <bcm63xx_log.h>

#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wswitch-enum"

#define ARRAY_SIZE(tab) (sizeof(tab) / sizeof(tab[0]))

#define container_of(ptr, type, member) ({			\
	const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
	(type *)( (char *)__mptr - offsetof(type,member) );})

#define __init
#define __exit
#define __iomem

#define ERESTARTSYS ERESTART

typedef signed char schar;
typedef unsigned char uchar;
typedef signed int sint;
typedef unsigned int uint;
typedef signed long slong;
typedef unsigned long ulong;

typedef void * dma_addr_t;

struct inode {
   int dummy;
};

struct file {
   void *private_data;
   int f_flags;
};

static inline bool is_power_of_2(size_t val)
{
   return ((~((val - 1) ^ (~(val - 1)))) == 0);
}

static inline void clear_bit(int bit, unsigned long *addr)
{
   *addr &= (~(1UL << bit));
}

static inline void set_bit(int bit, unsigned long *addr)
{
   *addr |= (1UL << bit);
}

static inline int test_bit(int bit, unsigned long *addr)
{
   return ((*addr & (1UL << bit)));
}

static inline int test_and_clear_bit(int bit, unsigned long *addr)
{
   int ret = test_bit(bit, addr);
   clear_bit(bit, addr);
   return (ret);
}

static inline int test_and_set_bit(int bit, unsigned long *addr)
{
   int ret = test_bit(bit, addr);
   set_bit(bit, addr);
   return (ret);
}

typedef struct {
   __u32 counter;
} atomic_t;

static inline int atomic_read(const atomic_t *v)
{
   return (v->counter);
}

static inline void atomic_set(atomic_t *v, int i)
{
   v->counter = i;
}

static inline void atomic_inc(atomic_t *v)
{
   v->counter += 1;
}

static inline void atomic_dec(atomic_t *v)
{
   v->counter -= 1;
}

typedef struct {
   bool locked;
} spinlock_t;

static inline void spin_lock_init(spinlock_t *t)
{
   t->locked = false;
}

static inline void spin_lock_irqsave(spinlock_t *t, unsigned long flags)
{
   bcm_assert(!t->locked);
   t->locked = true;
}

static inline void spin_unlock_irqrestore(spinlock_t *t, unsigned long flags)
{
   bcm_assert(t->locked);
   t->locked = false;
}

static inline void spin_lock_bh(spinlock_t *t)
{
   bcm_assert(!t->locked);
   t->locked = true;
}

static inline void spin_unlock_bh(spinlock_t *t)
{
   bcm_assert(t->locked);
   t->locked = false;
}

struct mutex {
   bool locked;
};

static inline void mutex_init(struct mutex *t)
{
   t->locked = false;
}

static inline void mutex_destroy(struct mutex *t)
{
   bcm_assert(!t->locked);
}

static inline int mutex_trylock(struct mutex *t)
{
   if (t->locked) {
      return (0);
   }
   else {
      t->locked = true;
      return (1);
   }
}

static inline int mutex_lock_interruptible(struct mutex *t)
{
   bcm_assert(!t->locked);
   t->locked = true;
   return (0);
}

static inline void mutex_unlock(struct mutex *t)
{
   bcm_assert(t->locked);
   t->locked = false;
}

extern unsigned long get_jiffies(void);

#define MAX_JIFFY_OFFSET ((LONG_MAX >> 1)-1)

#define msecs_to_jiffies(arg) (arg)

#define jiffies_to_msecs(arg) (arg)

#define jiffies_to_usecs(arg) ((arg) * 1000)

#define usecs_to_jiffies(arg) ((arg) / 1000)

static inline int time_before(unsigned long v, unsigned long ref)
{
   return (v < ref);
}

static inline int time_after(unsigned long v, unsigned long ref)
{
   return (v > ref);
}

extern void schedule(void);

extern void msleep(unsigned long msecs);

typedef struct wait_queue {
   struct wait_queue *next;
} wait_queue_t;

typedef struct {
   wait_queue_t *first;
} wait_queue_head_t;

static inline void init_waitqueue_head(wait_queue_head_t *t)
{
   t->first = NULL;
}

static inline void prepare_to_wait(wait_queue_head_t *t, wait_queue_t *wait)
{
   bcm_assert(NULL == wait->next);
   wait->next = t->first;
   t->first = wait;
}

static inline void finish_wait(wait_queue_head_t *t, wait_queue_t *wait)
{
   wait_queue_t *prev = NULL;
   wait_queue_t *cur = t->first;
   while ((NULL != cur) && (cur != wait)) {
      prev = cur;
      cur = cur->next;
   }
   if (NULL != cur) {
      bcm_assert(cur == wait);
      if (NULL == prev) {
         bcm_assert(t->first == cur);
         t->first = cur->next;
      }
      else {
         prev->next = cur->next;
      }
      cur->next = NULL;
   }
   else {
      bcm_assert(false);
   }
}

#define DEFINE_WAIT(name) \
	wait_queue_t name = { \
		.next	= NULL, \
	}

#define wait_event_interruptible(wq, condition) \
   ({ \
	   int __ret = 0; \
      do { \
         DEFINE_WAIT(__wait); \
         prepare_to_wait(&wq, &__wait); \
         for (;;) { \
            if (condition) { \
               break; \
            } \
            schedule(); \
         } \
         finish_wait(&wq, &__wait); \
      } while (0); \
      __ret; \
   })

#define wait_event_interruptible_timeout(wq, condition, timeout) \
   ({ \
      long __timeout = (timeout); \
	   long __ret = __timeout; \
      unsigned long __j0 = get_jiffies(); \
      do { \
         DEFINE_WAIT(__wait); \
         prepare_to_wait(&wq, &__wait); \
         for (;;) { \
            unsigned long __j1; \
            if (condition) { \
               break; \
            } \
            schedule(); \
            __j1 = get_jiffies(); \
            if (__j1 > (__j0 + __timeout)) { \
               __ret = 0; \
            } \
            else { \
               __ret = __j0 + __timeout - __j1; \
            } \
            if (!__ret) { \
               break; \
            } \
         } \
         finish_wait(&wq, &__wait); \
         if (!__ret && (condition)) { \
            __ret = 1; \
         } \
      } while (0); \
      __ret; \
   })

static inline void wake_up_interruptible(wait_queue_head_t *t)
{
}

static inline void wake_up_interruptible_all(wait_queue_head_t *t)
{
}

static inline int waitqueue_active(wait_queue_head_t *t)
{
   return (NULL != t->first);
}

struct timer_list {
   struct timer_list *next;
   void (*function)(unsigned long);
   unsigned long data;
   unsigned long expires;
};

static inline void init_timer(struct timer_list *t)
{
   t->next = NULL;
   t->function = NULL;
   t->data = 0;
   t->expires = 0;
}

extern void add_timer(struct timer_list *t);

extern void del_timer_sync(struct timer_list *t);

struct work_struct {
   void (*cb)(struct work_struct *work);
};

static inline void INIT_WORK(struct work_struct *t, void (*cb)(struct work_struct *work))
{
   bcm_assert(NULL != cb);
   t->cb = cb;
}

static inline void cancel_work_sync(struct work_struct *t)
{
}

struct workqueue_struct {
   int dummy;
};

#define alloc_workqueue(driver_name, flags, x) malloc_workqueue()

static inline struct workqueue_struct *malloc_workqueue(void)
{
   struct workqueue_struct *ret = (struct workqueue_struct *)(malloc(sizeof(struct workqueue_struct)));
   return (ret);
}

static inline void flush_workqueue(struct workqueue_struct *t)
{
}

static inline void destroy_workqueue(struct workqueue_struct *t)
{
   bcm_assert(NULL != t);
   free(t);
}

static inline void queue_work(struct workqueue_struct *t, struct work_struct *w)
{
   bcm_assert((NULL != t) && (NULL != w) && (NULL != w->cb));
   (*(w->cb))(w);
}

#define barrier()

#define rmb()

#define wmb()

#define kfree(ptr) free(ptr)

#define kmalloc(size, flags) malloc(size)

#define PAGE_SIZE 4096UL

#define __get_free_pages(flags, mm_order) ((unsigned long)(malloc(PAGE_SIZE << mm_order)))

#define free_pages(ptr, mm_order) free((void *)(ptr))

static inline int get_order(size_t len)
{
   int ret = 0;
   while ((PAGE_SIZE << ret) < len) {
      ret += 1;
   }
   return (ret);
}

static inline int copy_from_user(void *dest, void *src, size_t s)
{
   memcpy(dest, src, s);
   return (0);
}

static inline int copy_to_user(void *dest, void *src, size_t s)
{
   memcpy(dest, src, s);
   return (0);
}

#define simple_strtol strtol

#define likely(cond) cond
#define unlikely(cond) cond

#endif /* !__KERNEL__ */

#endif /* __FAKE_KERNEL_H__ */
