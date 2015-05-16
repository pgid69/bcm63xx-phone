/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifndef __MPI_H__
#define __MPI_H__

#include "config.h"

#ifdef __KERNEL__
#include <linux/spi/spi.h>
#endif // __KERNEL__

#include "board.h"

#ifdef BCMPH_DEBUG_MPI
#define MPI_READ 0
#define MPI_WRITE 1
#endif // BCMPH_DEBUG_MPI


#ifndef BCMPH_USE_SPI_DRIVER
typedef struct {
   __u32 ref_count;
#ifndef BCMPH_NOHW
   struct completion done;
   int irq;

   int num_chipselect;

   void __iomem *regs;
   resource_size_t res_start;
   resource_size_t res_size;

   /* Platform data */
   unsigned fifo_size;
   unsigned int msg_type_shift;
   unsigned int msg_ctl_width;

   /* data iomem */
   __u8 __iomem *tx_io;
   const __u8 __iomem  *rx_io;

   struct clk *clk;
   struct platform_device *pdev;

   __u8 clk_cfg;
   __u8 fill_byte;
#endif // BCMPH_NOHW
} bcm_mpi_dev_data_t;
#endif // !BCMPH_USE_SPI_DRIVER

typedef struct mpi {
   bool toggle_cs; // Toggle CS between each byte
#ifdef BCMPH_USE_SPI_DRIVER
   __u32 mpi_clk; /* SPI clock speed */
   struct spi_device *dev;
#else // !BCMPH_USE_SPI_DRIVER
   bcm_mpi_dev_data_t *dev_data;
   __u16 mpi_cs;
   __u8 clk_cfg;
   __u8 fill_byte;
   bool wait_completion_with_irq;
#endif // !BCMPH_USE_SPI_DRIVER
#ifdef BCMPH_DEBUG_MPI
   __u8 trace[16384];
   size_t trace_len;
#endif // BCMPH_DEBUG_MPI
} bcm_mpi_t;

extern int bcm_mpi_init(bcm_mpi_t *t, const bcm_mpi_params_t *params);

extern void bcm_mpi_deinit(bcm_mpi_t *t);

extern int bcm_mpi_read(bcm_mpi_t *t, __u8 *buf, __u8 buf_len, const __u8 *buf_prepend, __u8 prepend_len);

extern int bcm_mpi_write(bcm_mpi_t *t, const __u8 *buf, __u8 buf_len);

extern int bcm_mpi_read_write(bcm_mpi_t *t, __u8 *buf, __u8 buf_len);

#ifdef BCMPH_DEBUG_MPI
extern void bcm_mpi_dump_and_reset_trace(bcm_mpi_t *t);
#endif // BCMPH_DEBUG_MPI

#endif // __MPI_H__
