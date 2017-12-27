/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FK_LINUX_ATOMIC_H__
#define __FK_LINUX_ATOMIC_H__

#include <fake_kernel_compile.h>

// Source of this code : http://golubenco.org/atomic-operations.html

/**
 * Atomic type.
 */

typedef struct {
    volatile int counter;
} atomic_t;

#define ATOMIC_INIT(i)  { (i) }

/**
 * Read atomic variable
 * @param v pointer of type atomic_t
 *
 * Atomically reads the value of @v.
 */
static inline int atomic_read(atomic_t *v)
{
   return (__sync_val_compare_and_swap(&(v->counter), 0, 0));
}


/**
 * Set atomic variable
 * @param v pointer of type atomic_t
 * @param i required value
 */
static inline void atomic_set(atomic_t *v, int i)
{
   int old_val = 0;
   do {
      old_val = __sync_val_compare_and_swap(&(v->counter), old_val, i);
   } while (i != old_val);
}


/**
 * Add to the atomic variable
 * @param i integer value to add
 * @param v pointer of type atomic_t
 */
static inline void atomic_add( int i, atomic_t *v )
{
     (void)__sync_add_and_fetch(&v->counter, i);
}

/**
 * Subtract the atomic variable
 * @param i integer value to subtract
 * @param v pointer of type atomic_t
 *
 * Atomically subtracts @i from @v.
 */
static inline void atomic_sub( int i, atomic_t *v )
{
    (void)__sync_sub_and_fetch(&v->counter, i);
}

/**
 * Subtract value from variable and test result
 * @param i integer value to subtract
 * @param v pointer of type atomic_t
 *
 * Atomically subtracts @i from @v and returns
 * true if the result is zero, or false for all
 * other cases.
 */
static inline int atomic_sub_and_test( int i, atomic_t *v )
{
    return !(__sync_sub_and_fetch(&v->counter, i));
}

/**
 * Increment atomic variable
 * @param v pointer of type atomic_t
 *
 * Atomically increments @v by 1.
 */
static inline void atomic_inc( atomic_t *v )
{
   (void)__sync_fetch_and_add(&v->counter, 1);
}

/**
 * @brief decrement atomic variable
 * @param v: pointer of type atomic_t
 *
 * Atomically decrements @v by 1.  Note that the guaranteed
 * useful range of an atomic_t is only 24 bits.
 */
static inline void atomic_dec( atomic_t *v )
{
   (void)__sync_fetch_and_sub(&v->counter, 1);
}

/**
 * @brief Decrement and test
 * @param v pointer of type atomic_t
 *
 * Atomically decrements @v by 1 and
 * returns true if the result is 0, or false for all other
 * cases.
 */
static inline int atomic_dec_and_test( atomic_t *v )
{
   return !(__sync_sub_and_fetch(&v->counter, 1));
}

/**
 * @brief Increment and test
 * @param v pointer of type atomic_t
 *
 * Atomically increments @v by 1
 * and returns true if the result is zero, or false for all
 * other cases.
 */
static inline int atomic_inc_and_test( atomic_t *v )
{
      return !(__sync_add_and_fetch(&v->counter, 1));
}

/**
 * @brief add and test if negative
 * @param v pointer of type atomic_t
 * @param i integer value to add
 *
 * Atomically adds @i to @v and returns true
 * if the result is negative, or false when
 * result is greater than or equal to zero.
 */
static inline int atomic_add_negative( int i, atomic_t *v )
{
   return (__sync_add_and_fetch(&v->counter, i) < 0);
}

static inline int atomic_add_return(int i, atomic_t *v)
{
   return (__sync_add_and_fetch(&(v->counter), i));
}

static inline int atomic_sub_return(int i, atomic_t *v)
{
   return (__sync_sub_and_fetch(&(v->counter), i));
}

/**
 * @brief compare and swap
 * @param v pointer of type atomic_t
 *
 * If the current value of @b v is @b oldval,
 * then write @b newval into @b v. Returns #TRUE if
 * the comparison is successful and @b newval was
 * written.
 */
static inline int atomic_cas( atomic_t *v, int oldval, int newval )
{
   return __sync_bool_compare_and_swap(&v->counter, oldval, newval);

}
#endif // __FK_LINUX_ATOMIC_H__

