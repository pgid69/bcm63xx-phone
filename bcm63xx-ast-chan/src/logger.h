/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __LOGGER_H__
#define __LOGGER_H__

#include <bcm63xx_log.h>

#ifdef AST_VERSION

#include <asterisk.h>
#include <asterisk/logger.h>

#undef bcm_assert
#undef bcm_pr_debug
#undef bcm_pr_warn
#ifdef BCMPH_DEBUG
#define bcm_assert(cond) if (!(cond)) { ast_log(AST_LOG_DEBUG, "condition '%s' is false\n", #cond); }
#define bcm_pr_debug(fmt, args...) ast_log(AST_LOG_DEBUG, fmt, ## args)
#define bcm_pr_warn(fmt, args...) ast_log(AST_LOG_WARNING, fmt, ## args)
#else /* !BCMPH_DEBUG */
#define bcm_assert(cond)
#define bcm_pr_debug(fmt, args...)
#define bcm_pr_warn(fmt, args...)
#endif /* !BCMPH_DEBUG */

#endif /* AST_VERSION */

#endif /* __LOGGER_H__ */
