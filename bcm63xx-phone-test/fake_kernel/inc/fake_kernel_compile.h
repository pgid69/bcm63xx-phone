/*
 * Copyright (C) 2017
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __FAKE_KERNEL_COMPILE_H__
#define __FAKE_KERNEL_COMPILE_H__

#ifdef __KERNEL__
# error "Can't include this file if __KERNEL__ is defined"
#endif /* !__KERNEL__ */

#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wswitch-enum"

#include <stdbool.h>

#define __init
#define __exit
#define __iomem

#define likely(cond) cond
#define unlikely(cond) cond

/*
struct bcm63xx_spi_trx_opts {
	__u8 fill_byte;
	bool wait_completion_with_irq;
	bool drop_cs_after_each_byte;
	__u8 cs_off_clk_cycles;
};

*/

#endif /* __FAKE_KERNEL_COMPILE_H__ */
