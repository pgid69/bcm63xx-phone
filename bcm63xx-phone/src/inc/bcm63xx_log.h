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

/* Some useful macros */

/* Convert arg to a string */
#define bcm_stringify(arg) #arg

/* Concat two tokens to make a single token */
#define bcm_concat_2(tok1, tok2) tok1##tok2

/* Eval a macro with one args */
#define bcm_eval_macro_1(macro_name, arg1) macro_name(arg1)

/* Eval a macro with two args */
#define bcm_eval_macro_2(macro_name, arg1, arg2) macro_name(arg1, arg2)

/* undef macros, just in case */
#undef bcm_pr_debug
#undef bcm_dev_debug
#undef bcm_pr_info
#undef bcm_dev_info
#undef bcm_pr_warn
#undef bcm_dev_warn
#undef bcm_pr_err
#undef bcm_dev_err
#ifdef BCMPH_DEBUG
# ifdef __KERNEL__
/* This one if debugging is on, and kernel space */
#define bcm_pr_debug(fmt, args...) printk(KERN_INFO bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " %s() %lu: " fmt, __func__, (unsigned long)(__LINE__), ## args)
#define bcm_dev_debug(dev, fmt, args...) dev_info(dev, "%s() %lu. " fmt, __func__, (unsigned long)(__LINE__), ## args)
#define bcm_pr_info(fmt, args...) printk(KERN_INFO bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) ": " fmt, ## args)
#define bcm_dev_info(dev, fmt, args...) dev_info(dev, fmt, ## args)
#define bcm_pr_warn(fmt, args...) printk(KERN_WARNING bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) ": " fmt, ## args)
#define bcm_dev_warn(dev, fmt, args...) dev_warn(dev, "%s() %lu. " fmt, __func__, (unsigned long)(__LINE__), ## args)
#define bcm_pr_err(fmt, args...) printk(KERN_ERR bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) ": " fmt, ## args)
#define bcm_dev_err(dev, fmt, args...) dev_err(dev, "%s() %lu. " fmt, __func__, (unsigned long)(__LINE__), ## args)
# else /* !__KERNEL __ */
/* This one for user space */
#define bcm_pr_debug(fmt, args...) fprintf(stdout, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " debug: %s() %lu: " fmt, __func__, (unsigned long)(__LINE__), ## args)
#define bcm_dev_debug(dev, fmt, args...) fprintf(stdout, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " debug: %s() %lu. " fmt, __func__, (unsigned long)(__LINE__), ## args)
#define bcm_pr_info(fmt, args...) fprintf(stdout, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " info: " fmt, ## args)
#define bcm_dev_info(dev, fmt, args...) fprintf(stdout, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " info: %s() %lu. " fmt, __func__, (unsigned long)(__LINE__), ## args)
#define bcm_pr_warn(fmt, args...) fprintf(stderr, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " warning: " fmt, ## args)
#define bcm_dev_warn(dev, fmt, args...) fprintf(stderr, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " warning: %s() %lu. " fmt, __func__, (unsigned long)(__LINE__), ## args)
#define bcm_pr_err(fmt, args...) fprintf(stderr, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " error: " fmt, ## args)
#define bcm_dev_err(dev, fmt, args...) fprintf(stderr, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " error: %s() %lu. " fmt, __func__, (unsigned long)(__LINE__), ## args)
# endif /* !__KERNEL __ */
#else /* !BCMPH_DEBUG */
# ifdef __KERNEL__
/* This one if debugging is on, and kernel space */
#define bcm_pr_debug(fmt, args...) /* printk(KERN_DEBUG bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " %s() %lu : " fmt, __func__, (unsigned long)(__LINE__), ## args) */
#define bcm_dev_debug(dev, fmt, args...) /* dev_info(dev, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " %s() %lu. " fmt, __func__, (unsigned long)(__LINE__), ## args) */
#define bcm_pr_info(fmt, args...) printk(KERN_INFO bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) ": " fmt, ## args)
#define bcm_dev_info(dev, fmt, args...) dev_info(dev, fmt, ## args)
#define bcm_pr_warn(fmt, args...) printk(KERN_WARNING bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) ": " fmt, ## args)
#define bcm_dev_warn(dev, fmt, args...) dev_warn(dev, "%s() %lu. " fmt, __func__, (unsigned long)(__LINE__), ## args)
#define bcm_pr_err(fmt, args...) printk(KERN_ERR bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) ": " fmt, ## args)
#define bcm_dev_err(dev, fmt, args...) dev_err(dev, "%s() %lu. " fmt, __func__, (unsigned long)(__LINE__), ## args)
# else /* !__KERNEL __ */
/* This one for user space */
#define bcm_pr_debug(fmt, args...) /* fprintf(stdout, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " debug: %s() %lu: " fmt, __func__, (unsigned long)(__LINE__), ## args) */
#define bcm_dev_debug(dev, fmt, args...) /* fprintf(stdout, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " debug: %s() %lu. " fmt, __func__, (unsigned long)(__LINE__), ## args) */
#define bcm_pr_info(fmt, args...) fprintf(stdout, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " info: " fmt, ## args)
#define bcm_dev_info(dev, fmt, args...) fprintf(stdout, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " info: %s() %lu. " fmt, __func__, (unsigned long)(__LINE__), ## args)
#define bcm_pr_warn(fmt, args...) fprintf(stderr, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " warning: " fmt, ## args)
#define bcm_dev_warn(dev, fmt, args...) fprintf(stderr, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " warning: %s() %lu. " fmt, __func__, (unsigned long)(__LINE__), ## args)
#define bcm_pr_err(fmt, args...) fprintf(stderr, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " error: " fmt, ## args)
#define bcm_dev_err(dev, fmt, args...) fprintf(stderr, bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " error: %s() %lu. " fmt, __func__, (unsigned long)(__LINE__), ## args)
# endif /* !__KERNEL __ */
#endif /* !BCMPH_DEBUG */

/* undef macro, just in case */
#undef bcm_assert
#ifdef BCMPH_DEBUG
#define bcm_assert(cond) if (!(cond)) { bcm_pr_warn(bcm_eval_macro_1(bcm_stringify, BCMPH_MODULE_NAME) " %s() %lu: **** condition '%s' is false ****\n", __func__, (unsigned long)(__LINE__), #cond); }
#else /* !BCMPH_DEBUG */
#define bcm_assert(cond)
#endif /* !BCMPH_DEBUG */
#undef d_bcm_assert
#define d_bcm_assert(cond)

/* undef macros, just in case */
#undef dd_bcm_pr_debug
#undef dd_bcm_dev_debug
#undef dd_bcm_pr_info
#undef dd_bcm_dev_info
#undef dd_bcm_pr_warn
#undef dd_bcm_dev_warn
#undef dd_bcm_pr_err
#undef dd_bcm_dev_err
#define dd_bcm_pr_debug(fmt, args...) /* nothing: it's a placeholder */
#define dd_bcm_dev_debug(fmt, args...) /* nothing: it's a placeholder */
#define dd_bcm_pr_info(fmt, args...) /* nothing: it's a placeholder */
#define dd_bcm_dev_info(fmt, args...) /* nothing: it's a placeholder */
#define dd_bcm_pr_warn(fmt, args...) /* nothing: it's a placeholder */
#define dd_bcm_dev_warn(fmt, args...) /* nothing: it's a placeholder */
#define dd_bcm_pr_err(fmt, args...) /* nothing: it's a placeholder */
#define dd_bcm_dev_err(fmt, args...) /* nothing: it's a placeholder */

/*
#undef dd_bcm_pr_debug
#undef dd_bcm_dev_debug
#undef dd_bcm_pr_info
#undef dd_bcm_dev_info
#undef dd_bcm_pr_warn
#undef dd_bcm_dev_warn
#undef dd_bcm_pr_err
#undef dd_bcm_dev_err
#define dd_bcm_pr_debug bcm_pr_debug
#define dd_bcm_dev_debug bcm_dev_debug
#define dd_bcm_pr_info bcm_pr_info
#define dd_bcm_dev_info bcm_dev_info
#define dd_bcm_pr_warn bcm_pr_warn
#define dd_bcm_dev_warn bcm_dev_warn
#define dd_bcm_pr_err bcm_pr_err
#define dd_bcm_dev_err bcm_dev_err
*/

/* undef macros, just in case */
#undef d_bcm_pr_debug
#undef d_bcm_dev_debug
#undef d_bcm_pr_info
#undef d_bcm_dev_info
#undef d_bcm_pr_warn
#undef d_bcm_dev_warn
#undef d_bcm_pr_err
#undef d_bcm_dev_err
#ifndef BCMPH_NOHW
#define d_bcm_pr_debug dd_bcm_pr_debug
#define d_bcm_dev_debug dd_bcm_dev_debug
#define d_bcm_pr_info dd_bcm_pr_info
#define d_bcm_dev_info dd_bcm_dev_info
#define d_bcm_pr_warn dd_bcm_pr_warn
#define d_bcm_dev_warn dd_bcm_dev_warn
#define d_bcm_pr_err dd_bcm_pr_err
#define d_bcm_dev_err dd_bcm_dev_err
#else /* BCMPH_NOHW */
#define d_bcm_pr_debug bcm_pr_debug
#define d_bcm_dev_debug bcm_dev_debug
#define d_bcm_pr_info bcm_pr_info
#define d_bcm_dev_info bcm_dev_info
#define d_bcm_pr_warn bcm_pr_warn
#define d_bcm_dev_warn bcm_dev_warn
#define d_bcm_pr_err bcm_pr_err
#define d_bcm_dev_err bcm_dev_err
#endif

#endif /* __BCM63XX_LOG_H__ */
