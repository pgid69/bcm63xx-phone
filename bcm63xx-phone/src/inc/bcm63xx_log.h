/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifndef __BCM63XX_LOG_H__
#define __BCM63XX_LOG_H__

#ifdef __KERNEL__
#include <linux/kernel.h>
#else
#include <stdio.h>
#endif /* __KERNEL__ */

/* undef macros, just in case */
#undef bcm_pr_debug
#undef bcm_pr_trace
#undef bcm_pr_info
#undef bcm_pr_warn
#undef bcm_pr_err
#ifdef BCMPH_DEBUG
# ifdef __KERNEL__
/* This one if debugging is on, and kernel space */
#define bcm_pr_debug(fmt, args...) printk(KERN_INFO "bcm63xx-phone %s() %lu: " fmt, __FUNCTION__, (unsigned long)(__LINE__), ## args)
#define bcm_pr_trace(fmt, args...) printk(KERN_INFO "bcm63xx-phone %s() %lu: " fmt, __FUNCTION__, (unsigned long)(__LINE__), ## args)
#define bcm_pr_info(fmt, args...) printk(KERN_INFO "bcm63xx-phone: " fmt, ## args)
#define bcm_pr_warn(fmt, args...) printk(KERN_WARNING "bcm63xx-phone: " fmt, ## args)
#define bcm_pr_err(fmt, args...) printk(KERN_ERR "bcm63xx-phone: " fmt, ## args)
# else /* !__KERNEL __ */
/* This one for user space */
#define bcm_pr_debug(fmt, args...) fprintf(stderr, "bcm63xx-phone debug %s() %lu: " fmt, __FUNCTION__, (unsigned long)(__LINE__), ## args)
#define bcm_pr_trace(fmt, args...) fprintf(stderr, "bcm63xx-phone trace %s() %lu: " fmt, __FUNCTION__, (unsigned long)(__LINE__), ## args)
#define bcm_pr_info(fmt, args...) fprintf(stderr, "bcm63xx-phone info: " fmt, ## args)
#define bcm_pr_warn(fmt, args...) fprintf(stderr, "bcm63xx-phone warning: " fmt, ## args)
#define bcm_pr_err(fmt, args...) fprintf(stderr, "bcm63xx-phone error: " fmt, ## args)
# endif /* !__KERNEL __ */
#else /* !BCMPH_DEBUG */
# ifdef __KERNEL__
/* This one if debugging is on, and kernel space */
#define bcm_pr_debug(fmt, args...) /* printk(KERN_DEBUG "bcm63xx-phone %s() %lu : " fmt, __FUNCTION__, (unsigned long)(__LINE__), ## args) */
#define bcm_pr_trace(fmt, args...) /* printk(KERN_DEBUG "bcm63xx-phone %s() %lu : " fmt, __FUNCTION__, (unsigned long)(__LINE__), ## args) */
#define bcm_pr_info(fmt, args...) printk(KERN_INFO "bcm63xx-phone: " fmt, ## args)
#define bcm_pr_warn(fmt, args...) printk(KERN_WARNING "bcm63xx-phone: " fmt, ## args)
#define bcm_pr_err(fmt, args...) printk(KERN_ERR "bcm63xx-phone: " fmt, ## args)
# else /* !__KERNEL __ */
/* This one for user space */
#define bcm_pr_debug(fmt, args...) /* fprintf(stderr, "bcm63xx-phone debug %s() %lu: " fmt, __FUNCTION__, (unsigned long)(__LINE__), ## args) */
#define bcm_pr_trace(fmt, args...) /* fprintf(stderr, "bcm63xx-phone trace %s() %lu: " fmt, __FUNCTION__, (unsigned long)(__LINE__), ## args) */
#define bcm_pr_info(fmt, args...) fprintf(stderr, "bcm63xx-phone info: " fmt, ## args)
#define bcm_pr_warn(fmt, args...) fprintf(stderr, "bcm63xx-phone warning: " fmt, ## args)
#define bcm_pr_err(fmt, args...) fprintf(stderr, "bcm63xx-phone error: " fmt, ## args)
# endif /* !__KERNEL __ */
#endif /* !BCMPH_DEBUG */

/* undef macro, just in case */
#undef bcm_assert
#ifdef BCMPH_DEBUG
#define bcm_assert(cond) if (!(cond)) { bcm_pr_debug("condition '%s' is false\n", #cond); }
#else /* !BCMPH_DEBUG */
#define bcm_assert(cond)
#endif /* !BCMPH_DEBUG */
#undef d_bcm_assert
#define d_bcm_assert(cond)

/* undef macros, just in case */
#undef dd_bcm_pr_debug
#undef dd_bcm_pr_info
#undef dd_bcm_pr_warn
#undef dd_bcm_pr_err
#define dd_bcm_pr_debug(fmt, args...) /* nothing: it's a placeholder */
#define dd_bcm_pr_info(fmt, args...) /* nothing: it's a placeholder */
#define dd_bcm_pr_warn(fmt, args...) /* nothing: it's a placeholder */
#define dd_bcm_pr_err(fmt, args...) /* nothing: it's a placeholder */

/* undef macros, just in case */
#undef d_bcm_pr_debug
#undef d_bcm_pr_info
#undef d_bcm_pr_warn
#undef d_bcm_pr_err
#ifndef BCMPH_NOHW
#define d_bcm_pr_debug dd_bcm_pr_debug
#define d_bcm_pr_info dd_bcm_pr_info
#define d_bcm_pr_warn dd_bcm_pr_warn
#define d_bcm_pr_err dd_bcm_pr_err
#else /* BCMPH_NOHW */
#define d_bcm_pr_debug bcm_pr_debug
#define d_bcm_pr_info bcm_pr_info
#define d_bcm_pr_warn bcm_pr_warn
#define d_bcm_pr_err bcm_pr_err
#endif

#endif /* __BCM63XX_LOG_H__ */
