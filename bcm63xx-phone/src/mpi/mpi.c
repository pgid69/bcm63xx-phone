/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#include <config.h>

#ifndef BCMPH_NOHW
#ifndef BCMPH_USE_SPI_DRIVER
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/spi/spi.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/pm_runtime.h>
#endif // BCMPH_USE_SPI_DRIVER
#endif // BCMPH_NOHW

#include <bcm63xx.h>
#include <mpi.h>
#include <bcm63xx_log.h>

// Include after system files
#include <compile.h>

#ifndef BCMPH_NOHW
static void bcm_mpi_enable_extra_CSs(void)
{
   /*
    Code adapted from http://pastebin.com/g0bQGPRj
   */

   if (BCMCPU_IS_6348()) {
      /* BCM6348 */
      u32 val;
      /* Enable Extra SPI CS */
      /* GPIO 29 is SS2, GPIO 30 is SS3, GPIO31 is SS4 FIXME */
      val = bcm_gpio_readl(GPIO_MODE_REG);
      val |= GPIO_MODE_6348_G1_SPI_MASTER;
      bcm_gpio_writel(val, GPIO_MODE_REG);
   }

   if (BCMCPU_IS_6358()) {
      /* BCM6358 */
      u32 val;
      /* Enable Overlay for SPI SS Pins */
      val = bcm_gpio_readl(GPIO_MODE_REG);
      val |= GPIO_MODE_6358_EXTRA_SPI_SS;
      bcm_gpio_writel(val, GPIO_MODE_REG);
      /* Enable SPI Slave Select as Output Pins */
      /* GPIO 32 is SS2, GPIO 33 is SS3 */
      val = bcm_gpio_readl(GPIO_CTL_HI_REG);
      val |= 0x0003;
      bcm_gpio_writel(val, GPIO_CTL_HI_REG);
   }

   if (BCMCPU_IS_6368()) {
      /* BCM6368 */
      u32 val;
      /* Enable Extra SPI CS */
      val = bcm_gpio_readl(GPIO_MODE_REG);
      val |= (GPIO_MODE_6368_SPI_SSN2 | GPIO_MODE_6368_SPI_SSN3 | GPIO_MODE_6368_SPI_SSN4 | GPIO_MODE_6368_SPI_SSN5);
      bcm_gpio_writel(val, GPIO_MODE_REG);
      /* Enable SPI Slave Select as Output Pins */
      /* GPIO 28 is SS2, GPIO 29 is SS3, GPIO 30 is SS4, GPIO 31 is SS5*/
      val = bcm_gpio_readl(GPIO_CTL_HI_REG);
      val |= (GPIO_MODE_6368_SPI_SSN2 | GPIO_MODE_6368_SPI_SSN3 | GPIO_MODE_6368_SPI_SSN4 | GPIO_MODE_6368_SPI_SSN5);
      bcm_gpio_writel(val, GPIO_CTL_HI_REG);
   }
}

#ifndef BCMPH_USE_SPI_DRIVER

/*
 * This code is adapted from Broadcom BCM63xx SPI controller support
 * file (spi-bcm63xx.c)
 *
 * Copyright (C) 2009-2012 Florian Fainelli <florian@openwrt.org>
 * Copyright (C) 2010 Tanguy Bouzeloc <tanguy.bouzeloc@efixo.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 */

#define BCM63XX_SPI_MAX_PREPEND 15
#define BCM63XX_SPI_MAX_CS    8

enum bcm63xx_regs_spi {
   SPI_CMD,
   SPI_INT_STATUS,
   SPI_INT_MASK_ST,
   SPI_INT_MASK,
   SPI_ST,
   SPI_CLK_CFG,
   SPI_FILL_BYTE,
   SPI_MSG_TAIL,
   SPI_RX_TAIL,
   SPI_MSG_CTL,
   SPI_MSG_DATA,
   SPI_RX_DATA,
};

static const unsigned long bcm6348_spi_reg_offsets[] = {
   [SPI_CMD]      = SPI_6348_CMD,
   [SPI_INT_STATUS]  = SPI_6348_INT_STATUS,
   [SPI_INT_MASK_ST] = SPI_6348_INT_MASK_ST,
   [SPI_INT_MASK]    = SPI_6348_INT_MASK,
   [SPI_ST]    = SPI_6348_ST,
   [SPI_CLK_CFG]     = SPI_6348_CLK_CFG,
   [SPI_FILL_BYTE]      = SPI_6348_FILL_BYTE,
   [SPI_MSG_TAIL]    = SPI_6348_MSG_TAIL,
   [SPI_RX_TAIL]     = SPI_6348_RX_TAIL,
   [SPI_MSG_CTL]     = SPI_6348_MSG_CTL,
   [SPI_MSG_DATA]    = SPI_6348_MSG_DATA,
   [SPI_RX_DATA]     = SPI_6348_RX_DATA,
};

static const unsigned long bcm6358_spi_reg_offsets[] = {
   [SPI_CMD]      = SPI_6358_CMD,
   [SPI_INT_STATUS]  = SPI_6358_INT_STATUS,
   [SPI_INT_MASK_ST] = SPI_6358_INT_MASK_ST,
   [SPI_INT_MASK]    = SPI_6358_INT_MASK,
   [SPI_ST]    = SPI_6358_ST,
   [SPI_CLK_CFG]     = SPI_6358_CLK_CFG,
   [SPI_FILL_BYTE]      = SPI_6358_FILL_BYTE,
   [SPI_MSG_TAIL]    = SPI_6358_MSG_TAIL,
   [SPI_RX_TAIL]     = SPI_6358_RX_TAIL,
   [SPI_MSG_CTL]     = SPI_6358_MSG_CTL,
   [SPI_MSG_DATA]    = SPI_6358_MSG_DATA,
   [SPI_RX_DATA]     = SPI_6358_RX_DATA,
};

static void bcm_mpi_dev_data_init(bcm_mpi_dev_data_t *bs)
{
   memset(bs, 0, sizeof(*bs));
   bs->num_chipselect = BCM63XX_SPI_MAX_CS;
   if (BCMCPU_IS_6348()) {
      bs->reg_offsets = bcm6348_spi_reg_offsets;
      bs->fifo_size = SPI_6348_MSG_DATA_SIZE;
      bs->msg_type_shift = SPI_6348_MSG_TYPE_SHIFT;
      bs->msg_ctl_width = SPI_6348_MSG_CTL_WIDTH;
   }
   else {
      bs->reg_offsets = bcm6358_spi_reg_offsets;
      bs->fifo_size = SPI_6358_MSG_DATA_SIZE;
      bs->msg_type_shift = SPI_6358_MSG_TYPE_SHIFT;
      bs->msg_ctl_width = SPI_6358_MSG_CTL_WIDTH;
   }
}

static inline u8 bcm_spi_readb(bcm_mpi_dev_data_t *bs,
            unsigned int offset)
{
   u8 ret;
   ret = bcm_readb(bs->regs + bs->reg_offsets[offset]);
   dd_bcm_pr_debug("bcm_spi_readb(off=0x%lx) -> 0x%x\n", (unsigned long)(offset), (unsigned int)(ret));
   return (ret);
}

static inline u16 bcm_spi_readw(bcm_mpi_dev_data_t *bs,
            unsigned int offset)
{
   u16 ret;
   ret = bcm_readw(bs->regs + bs->reg_offsets[offset]);
   dd_bcm_pr_debug("bcm_spi_readw(off=0x%lx) -> 0x%x\n", (unsigned long)(offset), (unsigned int)(ret));
   return (ret);
}

static inline void bcm_spi_writeb(bcm_mpi_dev_data_t *bs,
              u8 value, unsigned int offset)
{
   dd_bcm_pr_debug("bcm_spi_writeb(val=0x%x, off=0x%lx)\n", (unsigned int)(value), (unsigned long)(offset));
   bcm_writeb(value, bs->regs + bs->reg_offsets[offset]);
}

static inline void bcm_spi_writew(bcm_mpi_dev_data_t *bs,
              u16 value, unsigned int offset)
{
   dd_bcm_pr_debug("bcm_spi_writew(val=0x%x, off=0x%lx)\n", (unsigned int)(value), (unsigned long)(offset));
   bcm_writew(value, bs->regs + bs->reg_offsets[offset]);
}

static const unsigned bcm63xx_spi_freq_table[SPI_CLK_MASK][2] = {
   { 20000000, SPI_CLK_20MHZ },
   { 12500000, SPI_CLK_12_50MHZ },
   {  6250000, SPI_CLK_6_250MHZ },
   {  3125000, SPI_CLK_3_125MHZ },
   {  1563000, SPI_CLK_1_563MHZ },
   {   781000, SPI_CLK_0_781MHZ },
   {   391000, SPI_CLK_0_391MHZ }
};

static unsigned bcm_mpi_get_clk_cfg(u32 hz, u8 cs_off_time)
{
   u8 clk_cfg = 0;
   size_t i;

   dd_bcm_pr_debug("bcm_mpi_get_clk_cfg(hz=%lu, cs_off_time=%u)\n", (unsigned long)(hz), (unsigned int)(cs_off_time));

   /* Find the closest clock configuration */
   for (i = 0; (i < ARRAY_SIZE(bcm63xx_spi_freq_table)); i++) {
      if (hz >= bcm63xx_spi_freq_table[i][0]) {
         clk_cfg |= bcm63xx_spi_freq_table[i][1];
         break;
      }
   }

   /* No matching configuration found, default to lowest */
   if (i >= ARRAY_SIZE(bcm63xx_spi_freq_table)) {
      clk_cfg |= SPI_CLK_0_391MHZ;
   }

   bcm_assert(((cs_off_time << SPI_SSOFFTIME_SHIFT) & (~(SPI_SSOFFTIME_MASK))) == 0);

   clk_cfg |= (cs_off_time << SPI_SSOFFTIME_SHIFT);

   return (clk_cfg);
}

static void bcm_mpi_set_clk_cfg(bcm_mpi_dev_data_t *bs, u8 clk_cfg)
{
   u8 reg;

   dd_bcm_pr_debug("bcm_mpi_set_clk_cfg(0x%x)\n", (unsigned int)(clk_cfg));

   /* clear existing clock configuration bits of the register */
   reg = bcm_spi_readb(bs, SPI_CLK_CFG);
   reg &= (~(SPI_CLK_MASK | SPI_SSOFFTIME_MASK));
   reg |= clk_cfg;

   bcm_spi_writeb(bs, reg, SPI_CLK_CFG);

   bs->clk_cfg = clk_cfg;
}

static void bcm_mpi_set_fill_byte(bcm_mpi_dev_data_t *bs, u8 fill_byte)
{
   dd_bcm_pr_debug("bcm_mpi_set_fill_byte(0x%x)\n", (unsigned int)(fill_byte));

   bcm_spi_writeb(bs, fill_byte, SPI_FILL_BYTE);

   bs->fill_byte = fill_byte;
}

static inline void bcm_mpi_setup_transfer(bcm_mpi_t *t)
{
   dd_bcm_pr_debug("bcm_mpi_setup_transfer()\n");

   if (t->dev_data->clk_cfg != t->clk_cfg) {
      bcm_mpi_set_clk_cfg(t->dev_data, t->clk_cfg);
   }
   if (t->dev_data->fill_byte != t->fill_byte) {
      bcm_mpi_set_fill_byte(t->dev_data, t->fill_byte);
   }
}

static int bcm_mpi_rw_buf(bcm_mpi_t *t, u8 *buf, u8 buf_len,
   bool do_tx, bool do_rx,
   const u8 *prepend_buf, u8 prepend_buf_len)
{
   bcm_mpi_dev_data_t *bs = t->dev_data;
   u16 msg_ctl;
   u16 cmd;
   unsigned int timeout;
   u8 rx_tail;

   dd_bcm_pr_debug("bcm_mpi_rw_buf(buf_len=%u, do_tx=%d, do_rx=%d, prepend_buf_len=%u)\n",
      (unsigned int)(buf_len), (int)(do_tx), (int)(do_rx), (unsigned int)(prepend_buf_len));

   /* Disable the CMD_DONE interrupt */
   bcm_spi_writeb(bs, 0, SPI_INT_MASK);

   cmd = SPI_CMD_START_IMMEDIATE;
   cmd |= (t->mpi_cs << SPI_CMD_DEVICE_ID_SHIFT);
   if (t->toggle_cs) {
      cmd |= (1 << SPI_CMD_ONE_BYTE_SHIFT);
   }

   if (do_tx) {
      bcm_assert((0 == prepend_buf_len) && (buf_len > 0));
      memcpy_toio(bs->tx_io, buf, buf_len);
   }
   else {
      bcm_assert(do_rx);
      if (prepend_buf_len > 0) {
         memcpy_toio(bs->tx_io, prepend_buf, prepend_buf_len);
         cmd |= (prepend_buf_len << SPI_CMD_PREPEND_BYTE_CNT_SHIFT);
      }
      else {
         bcm_assert(buf_len > 0);
      }
   }

   if (t->wait_completion_with_irq) {
      init_completion(&(bs->done));
   }

   /* Fill in the Message control register */
   msg_ctl = (buf_len << SPI_BYTE_CNT_SHIFT);

   if (do_tx) {
      if (do_rx) {
         msg_ctl |= (SPI_FD_RW << bs->msg_type_shift);
      }
      else {
         msg_ctl |= (SPI_HD_W << bs->msg_type_shift);
      }
   }
   else {
      msg_ctl |= (SPI_HD_R << bs->msg_type_shift);
   }

   switch (bs->msg_ctl_width) {
      case 8:
         bcm_spi_writeb(bs, msg_ctl, SPI_MSG_CTL);
         break;
      case 16:
         bcm_spi_writew(bs, msg_ctl, SPI_MSG_CTL);
         break;
      default:
         bcm_assert(0);
         break;
   }

   bcm_spi_writeb(bs, SPI_INTR_CLEAR_ALL, SPI_INT_STATUS);
   if (t->wait_completion_with_irq) {
      /* Enable the CMD_DONE interrupt */
      bcm_spi_writeb(bs, SPI_INTR_CMD_DONE, SPI_INT_MASK);
   }

   /* Issue the transfer */
   bcm_spi_writew(bs, cmd, SPI_CMD);

   if (t->wait_completion_with_irq) {
      timeout = wait_for_completion_timeout(&bs->done, HZ);
      if (!timeout) {
         return (-ETIMEDOUT);
      }
   }
   else {
      while (!(bcm_spi_readb(bs, SPI_INT_STATUS) & SPI_INTR_CMD_DONE))
      {
      }
   }

   if (do_rx) {
      /* read out all data */
      rx_tail = bcm_spi_readb(bs, SPI_RX_TAIL);
      if (rx_tail != buf_len) {
         return (-EINVAL);
      }
      if (rx_tail > 0) {
         memcpy_fromio(buf, bs->rx_io, buf_len);
      }
   }

   return (buf_len);
}

/* This driver supports single master mode only. Hence
 * CMD_DONE is the only interrupt we care about
 */
static irqreturn_t bcm63xx_spi_interrupt(int irq, void *dev_id)
{
   bcm_mpi_dev_data_t *bs = (bcm_mpi_dev_data_t *)dev_id;
   u8 intr;

   /* Read interupts and clear them immediately */
   intr = bcm_spi_readb(bs, SPI_INT_STATUS);
   bcm_spi_writeb(bs, SPI_INTR_CLEAR_ALL, SPI_INT_STATUS);
   bcm_spi_writeb(bs, 0, SPI_INT_MASK);

   /* A transfer completed */
   if (intr & SPI_INTR_CMD_DONE)
      complete(&bs->done);

   return IRQ_HANDLED;
}

static bcm_mpi_dev_data_t bcm_mpi_dev_data = {
   .ref_count = 0
};

static int bcm63xx_spi_probe(struct platform_device *pdev)
{
   struct device *dev = &(pdev->dev);
   int ret;
   struct resource *r;
   int irq;
   bcm_mpi_dev_data_t *bs;

   bcm_pr_debug("bcm63xx_spi_probe()\n");

   bs = &(bcm_mpi_dev_data);
   bcm_mpi_dev_data_init(bs);
   bs->ref_count = 1;
   bs->pdev = pdev;

   r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
   if (!r) {
      dev_err(dev, "no iomem\n");
      ret = -ENXIO;
      goto fail_get_res;
   }
   bs->res_start = r->start;
   bs->res_size = resource_size(r);

   irq = platform_get_irq(pdev, 0);
   if (irq < 0) {
      dev_err(dev, "no irq\n");
      ret = -ENXIO;
      goto fail_get_irq;
   }
   bs->irq = irq;

   bs->clk = devm_clk_get(dev, "spi");
   if (IS_ERR(bs->clk)) {
      dev_err(dev, "no clock for device\n");
      ret = PTR_ERR(bs->clk);
      goto fail_get_clk;
   }

   platform_set_drvdata(pdev, bs);

   if (!devm_request_mem_region(&(pdev->dev), bs->res_start, bs->res_size, driver_name)) {
      dev_err(dev, "iomem request failed\n");
      ret = -ENXIO;
      goto fail_req_reg;
   }

   bs->regs = devm_ioremap_nocache(&(pdev->dev), bs->res_start, bs->res_size);
   if (!bs->regs) {
      dev_err(dev, "unable to ioremap regs\n");
      ret = -ENOMEM;
      goto fail_io_remap;
   }
   bs->tx_io = (u8 *)(bs->regs + bs->reg_offsets[SPI_MSG_DATA]);
   bs->rx_io = (const u8 *)(bs->regs + bs->reg_offsets[SPI_RX_DATA]);

   ret = devm_request_irq(&(pdev->dev), irq, bcm63xx_spi_interrupt, 0,
                     pdev->name, bs);
   if (ret) {
      dev_err(dev, "unable to request irq\n");
      goto fail_req_irq;
   }

   /* Initialize hardware */
   clk_enable(bs->clk);
   /* Read interupts and clear them immediately */
   bcm_spi_writeb(bs, SPI_INTR_CLEAR_ALL, SPI_INT_STATUS);
   bcm_spi_writeb(bs, 0, SPI_INT_MASK);

   bcm_mpi_set_clk_cfg(bs, SPI_CLK_0_391MHZ);
   bcm_mpi_set_fill_byte(bs, 0);

   dev_info(dev, "at 0x%08x (irq %d, FIFOs size %d)\n",
       bs->res_start, bs->irq, bs->fifo_size);

   bcm_mpi_enable_extra_CSs();

   return 0;

   clk_disable(bs->clk);
   devm_free_irq(&(pdev->dev), irq, bs);
fail_req_irq:
   devm_iounmap(&(pdev->dev), bs->regs);
fail_io_remap:
   devm_release_mem_region(&(pdev->dev), bs->res_start, bs->res_size);
fail_req_reg:
   platform_set_drvdata(pdev, NULL);
   clk_put(bs->clk);
fail_get_clk:
fail_get_irq:
fail_get_res:
   bs->ref_count = 0;
   return ret;
}

static int bcm63xx_spi_remove(struct platform_device *pdev)
{
   bcm_mpi_dev_data_t *bs = platform_get_drvdata(pdev);

   bcm_pr_debug("bcm63xx_spi_remove()\n");

   bcm_assert(1 == bs->ref_count);

   /* reset spi block */
   bcm_spi_writeb(bs, 0, SPI_INT_MASK);

   /* HW shutdown */
   clk_disable(bs->clk);
   devm_free_irq(&(pdev->dev), bs->irq, bs);
   devm_iounmap(&(pdev->dev), bs->regs);
   devm_release_mem_region(&(pdev->dev), bs->res_start, bs->res_size);
   platform_set_drvdata(pdev, NULL);
   clk_put(bs->clk);
   bs->ref_count = 0;

   return (0);
}

#ifdef CONFIG_PM
static int bcm63xx_spi_suspend(struct device *dev)
{
   bcm_mpi_dev_data_t *bs =
         platform_get_drvdata(to_platform_device(dev));

   bcm_pr_debug("bcm63xx_spi_suspend()\n");

   clk_disable(bs->clk);

   return 0;
}

static int bcm63xx_spi_resume(struct device *dev)
{
   bcm_mpi_dev_data_t *bs =
         platform_get_drvdata(to_platform_device(dev));

   bcm_pr_debug("bcm63xx_spi_resume()\n");

   clk_enable(bs->clk);

   return 0;
}

static const struct dev_pm_ops bcm63xx_spi_pm_ops = {
   .suspend = bcm63xx_spi_suspend,
   .resume     = bcm63xx_spi_resume,
};

#define BCM63XX_SPI_PM_OPS (&bcm63xx_spi_pm_ops)
#else // !CONFIG_PM
#define BCM63XX_SPI_PM_OPS NULL
#endif // !CONFIG_PM

static struct platform_driver bcm63xx_spi_driver = {
   .driver = {
      .name = "bcm63xx-spi",
      .owner   = THIS_MODULE,
      .pm   = BCM63XX_SPI_PM_OPS,
   },
   .probe      = bcm63xx_spi_probe,
   .remove     = bcm63xx_spi_remove,
};
#endif // BCMPH_USE_SPI_DRIVER
#endif // !BCMPH_NOHW

#ifdef BCMPH_DEBUG_MPI
void bcm_mpi_dump_and_reset_trace(bcm_mpi_t *t)
{
   size_t i;
   char message[1024];

   if (t->trace_len > 0) {
      size_t l;

      bcm_pr_info("MPI trace_len %lu\n", (unsigned long)(t->trace_len));
      i = 0;
      l = 0;
      do {
         size_t j;
         size_t len;

         if ((l + 10) >= 256) {
            bcm_pr_info("%s\n", message);
            l = 0;
         }
         if (MPI_READ == t->trace[i]) {
            l += snprintf(&(message[l]), ARRAY_SIZE(message) - l, " R");
         }
         else if (MPI_WRITE == t->trace[i]) {
            l += snprintf(&(message[l]), ARRAY_SIZE(message) - l, " W");
         }
         i += 1;
         len = t->trace[i];
         i += 1;
         l += snprintf(&(message[l]), ARRAY_SIZE(message) - l, "%lu ", (unsigned long)(len));
         for (j = 0; (j < len); j += 1) {
            l += snprintf(&(message[l]), ARRAY_SIZE(message) - l, "%02X", (unsigned int)(t->trace[i]));
            i += 1;
            if ((l + 3) >= 256) {
               bcm_pr_info("%s\n", message);
               l = 0;
            }
         }
      } while (i < t->trace_len);
      if (l > 0) {
         bcm_pr_info("%s\n", message);
      }
   }
   t->trace_len = 0;
}

static void bcm_mpi_add_trace(bcm_mpi_t *t, __u8 dir, const __u8 *buf, int len)
{
   if (len > 0) {
      int i;
      if ((t->trace_len + len + 2) > ARRAY_SIZE(t->trace)) {
         bcm_pr_debug("Trace buffer full. Dumping it\n");
         bcm_mpi_dump_and_reset_trace(t);
      }
      t->trace[t->trace_len] = dir;
      t->trace_len += 1;
      t->trace[t->trace_len] = (__u8)(len);
      t->trace_len += 1;
      for (i = 0; (i < len); i += 1) {
         t->trace[t->trace_len] = buf[i];
         t->trace_len += 1;
      }
   }
}
#endif // BCMPH_DEBUG_MPI

int bcm_mpi_write(bcm_mpi_t *t, const __u8 *buf, __u8 buf_len)
{
   int ret = 0;

#ifdef BCMPH_USE_SPI_DRIVER

#ifndef BCMPH_NOHW
   struct spi_transfer  tr;
   struct spi_message   msg;
#endif // !BCMPH_NOHW

   dd_bcm_pr_debug("bcm_mpi_write(buf_len=%d)\n", (int)(buf_len));

#ifndef BCMPH_NOHW
#ifdef BCMPH_DEBUG_MPI
   bcm_mpi_add_trace(t, MPI_WRITE, buf, buf_len);
#endif // BCMPH_DEBUG_MPI
   if (!t->toggle_cs) {
      spi_message_init(&(msg));
      memset(&(tr), 0, sizeof(tr));
      tr.tx_buf = buf;
      tr.len = buf_len;
      tr.bits_per_word = 8;
      tr.speed_hz = t->mpi_clk;
      tr.cs_change = 0;
      spi_message_add_tail(&(tr), &(msg));
      ret = spi_sync(t->dev, &(msg));
   }
   else {
      ret = 0;
      while (ret < buf_len) {
         int r;
         spi_message_init(&(msg));
         memset(&(tr), 0, sizeof(tr));
         tr.tx_buf = buf;
         tr.len = 1;
         tr.bits_per_word = 8;
         tr.speed_hz = t->mpi_clk;
         tr.cs_change = 1;
         spi_message_add_tail(&(tr), &(msg));
         r = spi_sync(t->dev, &(msg));
         if (r) {
            ret = r;
            break;
         }
         ret += 1;
         buf += 1;
      }
   }
#endif // !BCMPH_NOHW

#else // !BCMPH_USE_SPI_DRIVER

   dd_bcm_pr_debug("bcm_mpi_write(buf_len=%d)\n", (int)(buf_len));

#ifndef BCMPH_NOHW
   bcm_assert(buf_len <= t->dev_data->fifo_size);

   if (buf_len > 0) {
#ifdef BCMPH_DEBUG_MPI
      bcm_mpi_add_trace(t, MPI_WRITE, buf, buf_len);
#endif // BCMPH_DEBUG_MPI
      bcm_mpi_setup_transfer(t);
      ret = bcm_mpi_rw_buf(t, (u8 *)(buf), buf_len, true, false, NULL, 0);
   }
   else {
      ret = 0;
   }
#endif // !BCMPH_NOHW

#endif // !BCMPH_USE_SPI_DRIVER

   return (ret);
}

int bcm_mpi_read(bcm_mpi_t *t, __u8 *buf, __u8 buf_len, const __u8 *prepend_buf, __u8 prepend_buf_len)
{
   int ret = 0;

#ifdef BCMPH_USE_SPI_DRIVER

#ifndef BCMPH_NOHW
   struct spi_transfer tr0;
   struct spi_message msg;
#endif // !BCMPH_NOHW

   dd_bcm_pr_debug("bcm_mpi_read(buf_len=%d, prepend_buf_len=%d)\n", (int)(buf_len), (int)(prepend_buf_len));

#ifndef BCMPH_NOHW
#ifdef BCMPH_DEBUG_MPI
   bcm_mpi_add_trace(t, MPI_WRITE, prepend_buf, prepend_buf_len);
#endif // BCMPH_DEBUG_MPI
   if (!t->toggle_cs) {
      struct spi_transfer tr1;
      spi_message_init(&(msg));
      if (prepend_buf_len > 0) {
         memset(&(tr0), 0, sizeof(tr0));
         tr0.tx_buf = prepend_buf;
         tr0.len = prepend_buf_len;
         tr0.bits_per_word = 8;
         tr0.speed_hz = t->mpi_clk;
         tr0.cs_change = 0;
         spi_message_add_tail(&(tr0), &(msg));
      }
      memset(&(tr1), 0, sizeof(tr1));
      tr1.rx_buf = buf;
      tr1.len = buf_len;
      tr1.bits_per_word = 8;
      tr1.speed_hz = t->mpi_clk;
      tr1.cs_change = 0;
      spi_message_add_tail(&(tr1), &(msg));
      ret = spi_sync(t->dev, &(msg));
   }
   else {
      do { // Empty loop
         if (prepend_buf_len > 0) {
            ret = bcm_mpi_write(t, prepend_buf, prepend_buf_len);
            if (ret != prepend_buf_len) {
               break;
            }
         }
         ret = 0;
         while (ret < buf_len) {
            int r;
            spi_message_init(&(msg));
            memset(&(tr0), 0, sizeof(tr0));
            tr0.rx_buf = buf;
            tr0.len = 1;
            tr0.bits_per_word = 8;
            tr0.speed_hz = t->mpi_clk;
            tr0.cs_change = 1;
            spi_message_add_tail(&(tr0), &(msg));
            r = spi_sync(t->dev, &(msg));
            if (r) {
               ret = r;
               break;
            }
            ret += 1;
            buf += 1;
         }
      } while (0);
   }
#ifdef BCMPH_DEBUG_MPI
   bcm_mpi_add_trace(t, MPI_READ, buf, ret);
#endif // BCMPH_DEBUG_MPI
#endif // !BCMPH_NOHW

#else // !BCMPH_USE_SPI_DRIVER

   dd_bcm_pr_debug("bcm_mpi_read(buf_len=%d, prepend_buf_len=%d)\n", (int)(buf_len), (int)(prepend_buf_len));

#ifndef BCMPH_NOHW
   bcm_assert(buf_len <= t->dev_data->fifo_size);

#ifdef BCMPH_DEBUG_MPI
   bcm_mpi_add_trace(t, MPI_WRITE, prepend_buf, prepend_buf_len);
#endif // BCMPH_DEBUG_MPI
   bcm_mpi_setup_transfer(t);
   do { // Empty loop
      if (buf_len > 0) {
         if (prepend_buf_len > BCM63XX_SPI_MAX_PREPEND) {
            ret = bcm_mpi_rw_buf(t, (u8 *)(prepend_buf), prepend_buf_len, true, false, NULL, 0);
            if (ret != prepend_buf_len) {
               break;
            }
            prepend_buf = NULL;
            prepend_buf_len = 0;
         }
         ret = bcm_mpi_rw_buf(t, buf, buf_len, false, true, prepend_buf, prepend_buf_len);
      }
      else if (prepend_buf_len > 0) {
         ret = bcm_mpi_rw_buf(t, (u8 *)(prepend_buf), prepend_buf_len, true, false, NULL, 0);
      }
      else {
         ret = 0;
      }
   }
   while (0);
#ifdef BCMPH_DEBUG_MPI
   bcm_mpi_add_trace(t, MPI_READ, buf, ret);
#endif // BCMPH_DEBUG_MPI
#endif // !BCMPH_NOHW

#endif // !BCMPH_USE_SPI_DRIVER

   return (ret);
}

int bcm_mpi_read_write(bcm_mpi_t *t, __u8 *buf, __u8 buf_len)
{
   int ret = 0;

#ifdef BCMPH_USE_SPI_DRIVER

#ifndef BCMPH_NOHW
   struct spi_transfer tr;
   struct spi_message  msg;
#endif // !BCMPH_NOHW

   dd_bcm_pr_debug("bcm_mpi_read_write(buf_len=%d)\n", (int)(buf_len));

#ifndef BCMPH_NOHW
#ifdef BCMPH_DEBUG_MPI
   bcm_mpi_add_trace(t, MPI_WRITE, buf, buf_len);
#endif // BCMPH_DEBUG_MPI
   if (!t->toggle_cs) {
      spi_message_init(&(msg));
      memset(&(tr), 0, sizeof(tr));
      tr.tx_buf = buf;
      tr.rx_buf = buf;
      tr.len = buf_len;
      tr.bits_per_word = 8;
      tr.speed_hz = t->mpi_clk;
      tr.cs_change = 0;
      spi_message_add_tail(&(tr), &(msg));
      ret = spi_sync(t->dev, &(msg));
   }
   else {
      ret = 0;
      while (ret < buf_len) {
         int r;
         spi_message_init(&(msg));
         memset(&(tr), 0, sizeof(tr));
         tr.tx_buf = buf;
         tr.rx_buf = buf;
         tr.len = 1;
         tr.bits_per_word = 8;
         tr.speed_hz = t->mpi_clk;
         tr.cs_change = 1;
         spi_message_add_tail(&(tr), &(msg));
         r = spi_sync(t->dev, &(msg));
         if (r) {
            ret = r;
            break;
         }
         ret += 1;
         buf += 1;
      }
   }
#ifdef BCMPH_DEBUG_MPI
   bcm_mpi_add_trace(t, MPI_READ, buf, ret);
#endif // BCMPH_DEBUG_MPI
#endif // !BCMPH_NOHW

#else // !BCMPH_USE_SPI_DRIVER

   dd_bcm_pr_debug("bcm_mpi_read_write(buf_len=%d)\n", (int)(buf_len));

#ifndef BCMPH_NOHW
   bcm_assert(buf_len <= t->dev_data->fifo_size);

   if (buf_len > 0) {
#ifdef BCMPH_DEBUG_MPI
      bcm_mpi_add_trace(t, MPI_WRITE, buf, buf_len);
#endif // BCMPH_DEBUG_MPI
      bcm_mpi_setup_transfer(t);
      ret = bcm_mpi_rw_buf(t, buf, buf_len, true, true, NULL, 0);
#ifdef BCMPH_DEBUG_MPI
      bcm_mpi_add_trace(t, MPI_READ, buf, ret);
#endif // BCMPH_DEBUG_MPI
   }
   else {
      ret = 0;
   }
#endif // !BCMPH_NOHW

#endif // !BCMPH_USE_SPI_DRIVER

   return (ret);
}

int __init bcm_mpi_init(bcm_mpi_t *t, const bcm_mpi_params_t *params)
{
   int ret = 0;

#ifdef BCMPH_USE_SPI_DRIVER

#ifndef BCMPH_NOHW
   struct spi_master *master;
   struct spi_board_info board_info;
#endif // !BCMPH_NOHW

#endif // !BCMPH_USE_SPI_DRIVER

   bcm_pr_debug("bcm_mpi_init()\n");
   bcm_assert(NULL != params);

   t->toggle_cs = params->toggle_cs;

#ifdef BCMPH_USE_SPI_DRIVER

   t->mpi_clk = params->clk;
#ifndef BCMPH_NOHW
   master = spi_busnum_to_master(params->bus_num);
   if (NULL == master) {
      bcm_pr_err("No SPI master found for bus num %d. Module bcm63xx-spi not loaded ?\n",
         (int)(params->bus_num));
      ret = -EINVAL;
      goto fail_master;
   }

   memset(&(board_info), 0, sizeof(board_info));
   strcpy(board_info.modalias, driver_name);
   board_info.max_speed_hz = params->clk;
   board_info.bus_num = params->bus_num;
   board_info.chip_select = params->cs;
   board_info.mode = SPI_MODE_3;
   t->dev = spi_new_device(master, &(board_info));
   if (NULL == t->dev) {
      bcm_pr_err("Failed to add SPI device (busnum = %d, chip select = %d, clock = %lu)\n",
         (int)(params->bus_num), (int)(params->cs), (unsigned long)(params->clk));
      ret = -ENOMEM;
      goto fail_new_dev;
   }
   put_device(&(master->dev));

   bcm_mpi_enable_extra_CSs();
#ifdef BCMPH_DEBUG_MPI
   t->trace_len = 0;
#endif // BCMPH_DEBUG_MPI

   return (ret);

   spi_unregister_device(t->dev);
fail_new_dev:
   put_device(&(master->dev));
fail_master:
#endif // !BCMPH_NOHW

#else // !BCMPH_USE_SPI_DRIVER

#ifndef BCMPH_NOHW
   t->mpi_cs = params->cs;
   t->clk_cfg = bcm_mpi_get_clk_cfg(params->clk, params->cs_off_time);
   t->fill_byte = params->fill_byte;
   t->wait_completion_with_irq = params->wait_completion_with_irq;

   if (bcm_mpi_dev_data.ref_count <= 0) {
      struct device_driver *spi_driver = driver_find(bcm63xx_spi_driver.driver.name, &platform_bus_type);
      if (NULL != spi_driver) {
         bcm_pr_err("Error: Driver '%s' is already registered, aborting...\n",
            bcm63xx_spi_driver.driver.name);
         ret = -EBUSY;
      }
      else {
         ret = platform_driver_register(&(bcm63xx_spi_driver));
      }
      bcm_assert(((ret) && (0 == bcm_mpi_dev_data.ref_count))
         || ((!ret) && (1 == bcm_mpi_dev_data.ref_count)));
   }
   else {
      bcm_mpi_dev_data.ref_count += 1;
      ret = 0;
   }
   if (!ret) {
      bcm_assert(bcm_mpi_dev_data.ref_count > 0);
      if (params->cs > bcm_mpi_dev_data.num_chipselect) {
         dev_err(&(bcm_mpi_dev_data.pdev->dev), "%s, unsupported slave %d\n",
            __FUNCTION__, params->cs);
         if (1 == bcm_mpi_dev_data.ref_count) {
            platform_driver_unregister(&(bcm63xx_spi_driver));
         }
         bcm_mpi_dev_data.ref_count -= 1;
         ret = -EINVAL;
      }
      else {
         t->dev_data = &(bcm_mpi_dev_data);
      }
   }
#endif // !BCMPH_NOHW

#endif // !BCMPH_USE_SPI_DRIVER

   return (ret);
}

void bcm_mpi_deinit(bcm_mpi_t *t)
{
   bcm_pr_debug("bcm_mpi_deinit()\n");

#ifdef BCMPH_DEBUG_MPI
   bcm_mpi_dump_and_reset_trace(t);
#endif


#ifdef BCMPH_USE_SPI_DRIVER

#ifndef BCMPH_NOHW
   spi_unregister_device(t->dev);
#endif // !BCMPH_NOHW
   t->dev = NULL;

#else // !BCMPH_USE_SPI_DRIVER

#ifndef BCMPH_NOHW
   if (bcm_mpi_dev_data.ref_count > 0) {
      if (1 == bcm_mpi_dev_data.ref_count) {
         platform_driver_unregister(&(bcm63xx_spi_driver));
      }
      else {
         bcm_mpi_dev_data.ref_count -= 1;
      }
   }
   else {
      bcm_pr_err("Illegal call of bcm_mpi_deinit()\n");
   }
#endif // !BCMPH_NOHW

#endif // !BCMPH_USE_SPI_DRIVER
}

