/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <linux/types.h>
#include <signal.h>

#define BCMPH_DEBUG
//#define BCMPH_TEST_PCM

#ifdef BCMPH_NOHW
# ifndef BCMPH_TEST_PCM
#  define BCMPH_TEST_PCM
# endif // !BCMPH_TEST_PCM
#endif // BCMPH_NOHW

#include <bcm63xx_phone.h>
#include <bcm63xx_ring_buf.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#include <bcm63xx_ring_buf.c>

#ifndef BCMPH_TEST_PCM
#include "demo-thanks.h"
#endif // BCMPH_TEST_PCM

static int prm_use_16bits = 0;
static size_t prm_default_line = 0;
static struct {
   int enable;
   bcm_phone_codec_t codec;
} prm_lines[BCMPH_MAX_LINES];
static bcm_phone_line_tone_t prm_tone = BCMPH_TONE_WAITING_DIAL;
static unsigned int prm_tone_on_time = 8191;
static unsigned int prm_tone_off_time = 0;
static const char *prm_output = NULL;
static const char *prm_input = NULL;

static void dump_buf(const char *title, const uint8_t *buf, size_t lng_buf)
{
   printf("%s : %d bytes =>", title, (int)(lng_buf));
   if ((NULL != buf) && (lng_buf > 0)) {
      size_t i = 0;
      do {
         if (0 == (i % 16)) {
            fputs("\n", stdout);
            fprintf(stdout, "0x%.8lx: ", (unsigned long)(i));
         }
         else if (0 == (i % 8)) {
            fputs(" ", stdout);
         }
         fprintf(stdout, "%.2X ", buf[i]);
         i += 1;
      }
      while (i < lng_buf);
      fputs("\n", stdout);
   }
}

typedef struct
{
   int fd;
   bcm_phone_get_mmap_desc_t mmap_desc;
   __u8 *start_addr;
   struct {
      bcm_ring_buf_desc_t *rx_ring_buf_desc;
      bcm_ring_buf_t rx_ring_buf;
      bcm_ring_buf_desc_t *tx_ring_buf_desc;
      bcm_ring_buf_t tx_ring_buf;
   } lines[BCMPH_MAX_LINES];
} driver_mem_t;

static int do_ioctl(int fd, unsigned long request, unsigned long arg, int retry)
{
   int ret = ioctl(fd, request, arg);
   if ((retry) && (ret) && ((EAGAIN == errno) || (EINTR == errno))) {
      ret = ioctl(fd, request, arg);
   }
   if (ret < 0) {
      fprintf(stderr, "ioctl(%lu) failed. errno=%ld\n", (unsigned long)(request), (long)(errno));
   }
   return (ret);
}

static int get_driver_mem(int fd, driver_mem_t *driver_mem)
{
   int ret = 0;

   memset(driver_mem, 0, sizeof(*driver_mem));

   driver_mem->fd = fd;
   ret = do_ioctl(fd, BCMPH_IOCTL_GET_MMAP_DESC, (unsigned long)(&(driver_mem->mmap_desc)), 1);
   if (!ret) {
      if (sizeof(bcm_phone_ioctl_param_t) != driver_mem->mmap_desc.ioctl_param_size) {
         ret = -EINVAL;
         fprintf(stderr, "Structures shared with the driver are not of the expected size : driver expects %lu and program expects %lu\n",
            (unsigned long)(driver_mem->mmap_desc.ioctl_param_size),
            (unsigned long)(sizeof(bcm_phone_ioctl_param_t)));
      }
      else {
         driver_mem->start_addr = mmap(NULL, driver_mem->mmap_desc.mmap_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
         if (NULL == driver_mem->start_addr) {
            ret = errno;
            fprintf(stderr, "mmap() failed\n");
         }
      }
   }
   else {
      fprintf(stderr, "Error reading mmap description\n");
   }

   return (ret);
}

static void update_ring_bufs_desc(driver_mem_t *driver_mem)
{
   size_t i;
   for (i = 0; (i < ARRAY_SIZE(driver_mem->lines)); i += 1) {
      bcm_assert((driver_mem->lines[i].rx_ring_buf_desc->len == driver_mem->lines[i].rx_ring_buf.desc.len));
      bcm_ring_buf_desc_copy(driver_mem->lines[i].rx_ring_buf_desc, &(driver_mem->lines[i].rx_ring_buf.desc));

      bcm_assert((driver_mem->lines[i].tx_ring_buf_desc->len == driver_mem->lines[i].tx_ring_buf.desc.len));
      bcm_ring_buf_desc_copy(driver_mem->lines[i].tx_ring_buf_desc, &(driver_mem->lines[i].tx_ring_buf.desc));
   }
}

static void print_regs(const bcm_phone_pcm_regs_t *regs)
{
   size_t i;
   fprintf(stdout, "pcm_regs =>\n");
   fprintf(stdout, "ctlr = 0x%lx\n", (unsigned long)(regs->ctlr));
   fprintf(stdout, "chan_ctrl = 0x%lx\n", (unsigned long)(regs->chan_ctrl));
   fprintf(stdout, "int_pending = 0x%lx\n", (unsigned long)(regs->int_pending));
   fprintf(stdout, "int_mask = 0x%lx\n", (unsigned long)(regs->int_mask));
   fprintf(stdout, "pll_ctrl1 = 0x%lx\n", (unsigned long)(regs->pll_ctrl1));
   fprintf(stdout, "pll_ctrl2 = 0x%lx\n", (unsigned long)(regs->pll_ctrl2));
   fprintf(stdout, "pll_ctrl3 = 0x%lx\n", (unsigned long)(regs->pll_ctrl3));
   fprintf(stdout, "pll_ctrl4 = 0x%lx\n", (unsigned long)(regs->pll_ctrl4));
   fprintf(stdout, "pll_stat = 0x%lx\n", (unsigned long)(regs->pll_stat));
   for (i = 0; (i < ARRAY_SIZE(regs->slot_alloc_tbl)); i += 1) {
      fprintf(stdout, "slot_alloc_tbl[%lu] = 0x%lx\n", (unsigned long)(i), (unsigned long)(regs->slot_alloc_tbl[i]));
   }
   fprintf(stdout, "dma_cfg = 0x%lx\n", (unsigned long)(regs->dma_cfg));
   for (i = 0; (i < ARRAY_SIZE(regs->dma_channels)); i += 1) {
      fprintf(stdout, "dma_channels[%lu].dma_flowcl = 0x%lx\n", (unsigned long)(i), (unsigned long)(regs->dma_channels[i].dma_flowcl));
      fprintf(stdout, "dma_channels[%lu].dma_flowch = 0x%lx\n", (unsigned long)(i), (unsigned long)(regs->dma_channels[i].dma_flowch));
      fprintf(stdout, "dma_channels[%lu].dma_bufalloc = 0x%lx\n", (unsigned long)(i), (unsigned long)(regs->dma_channels[i].dma_bufalloc));

      fprintf(stdout, "dma_channels[%lu].dmac_chancfg = 0x%lx\n", (unsigned long)(i), (unsigned long)(regs->dma_channels[i].dmac_chancfg));
      fprintf(stdout, "dma_channels[%lu].dmac_ir = 0x%lx\n", (unsigned long)(i), (unsigned long)(regs->dma_channels[i].dmac_ir));
      fprintf(stdout, "dma_channels[%lu].dmac_irmask = 0x%lx\n", (unsigned long)(i), (unsigned long)(regs->dma_channels[i].dmac_irmask));
      fprintf(stdout, "dma_channels[%lu].dmac_maxburst = 0x%lx\n", (unsigned long)(i), (unsigned long)(regs->dma_channels[i].dmac_maxburst));

      fprintf(stdout, "dma_channels[%lu].dmas_rstart = 0x%lx\n", (unsigned long)(i), (unsigned long)(regs->dma_channels[i].dmas_rstart));
      fprintf(stdout, "dma_channels[%lu].dmas_sram2 = 0x%lx\n", (unsigned long)(i), (unsigned long)(regs->dma_channels[i].dmas_sram2));
      fprintf(stdout, "dma_channels[%lu].dmas_sram3 = 0x%lx\n", (unsigned long)(i), (unsigned long)(regs->dma_channels[i].dmas_sram3));
      fprintf(stdout, "dma_channels[%lu].dmas_sram4 = 0x%lx\n", (unsigned long)(i), (unsigned long)(regs->dma_channels[i].dmas_sram4));
   }
}

static int read_regs(int fd)
{
   int ret;
   bcm_phone_pcm_regs_t regs;

   memset(&(regs), 0, sizeof(regs));

   ret = do_ioctl(fd, BCMPH_IOCTL_READ_PCM_REGS, (unsigned long)(&(regs)), 1);
   if (!ret) {
      print_regs(&(regs));
   }
   else {
      fprintf(stderr, "Error reading PCM registers\n");
   }

   return (ret);
}

static int read_regs_mm(driver_mem_t *driver_mem)
{
   int ret;
   bcm_phone_ioctl_param_t *ioctl_param = (bcm_phone_ioctl_param_t *)(driver_mem->start_addr + driver_mem->mmap_desc.ioctl_param_off);

   ioctl_param->size = sizeof(bcm_phone_pcm_regs_t);

   ret = do_ioctl(driver_mem->fd, BCMPH_IOCTL_READ_PCM_REGS_MM, 0, 1);
   if (!ret) {
      print_regs(&(ioctl_param->p.read_pcm_regs));
   }
   else {
      fprintf(stderr, "Error reading PCM registers\n");
   }

   return (ret);
}

static void print_stats(const bcm_phone_pcm_stats_t *stats)
{
   fprintf(stdout, "======= Stats =======\n");
   fprintf(stdout, "rx_errors          => %lu\n", (unsigned long)(stats->rx_errors));
   fprintf(stdout, "rx_length_errors   => %lu\n", (unsigned long)(stats->rx_length_errors));
   fprintf(stdout, "rx_empty_errors    => %lu\n", (unsigned long)(stats->rx_empty_errors));
   fprintf(stdout, "rx_good            => %lu\n", (unsigned long)(stats->rx_good));
   fprintf(stdout, "rx_bytes           => %lu\n", (unsigned long)(stats->rx_bytes));
   fprintf(stdout, "tx_errors          => %lu\n", (unsigned long)(stats->tx_errors));
   fprintf(stdout, "tx_good            => %lu\n", (unsigned long)(stats->tx_good));
   fprintf(stdout, "tx_bytes           => %lu\n", (unsigned long)(stats->tx_bytes));
   fprintf(stdout, "cnt_irq_rx         => %lu\n", (unsigned long)(stats->cnt_irq_rx));
   fprintf(stdout, "cnt_irq_tx         => %lu\n", (unsigned long)(stats->cnt_irq_tx));
#ifdef BCMPH_TEST_PCM
   fprintf(stdout, "rx_sop             => %lu\n", (unsigned long)(stats->rx_sop));
   fprintf(stdout, "rx_eop             => %lu\n", (unsigned long)(stats->rx_eop));
   fprintf(stdout, "min_size_rx_buffer => %lu\n", (unsigned long)(stats->min_size_rx_buffer));
   fprintf(stdout, "max_size_rx_buffer => %lu\n", (unsigned long)(stats->max_size_rx_buffer));
   fprintf(stdout, "min_size_tx_buffer => %lu\n", (unsigned long)(stats->min_size_tx_buffer));
   fprintf(stdout, "max_size_tx_buffer => %lu\n", (unsigned long)(stats->max_size_tx_buffer));
#endif // BCMPH_TEST_PCM
}

static int read_stats(int fd)
{
   int ret;
   bcm_phone_pcm_stats_t stats;

   memset(&(stats), 0, sizeof(stats));

   ret = do_ioctl(fd, BCMPH_IOCTL_READ_PCM_STATS, (unsigned long)(&(stats)), 1);
   if (!ret) {
      print_stats(&(stats));
   }
   else {
      fprintf(stderr, "Error reading PCM stats\n");
   }

   return (ret);
}

static int read_stats_mm(driver_mem_t *driver_mem)
{
   int ret;
   bcm_phone_ioctl_param_t *ioctl_param = (bcm_phone_ioctl_param_t *)(driver_mem->start_addr + driver_mem->mmap_desc.ioctl_param_off);

   ioctl_param->size = sizeof(bcm_phone_pcm_stats_t);

   ret = do_ioctl(driver_mem->fd, BCMPH_IOCTL_READ_PCM_STATS_MM, 0, 1);
   if (!ret) {
      print_stats(&(ioctl_param->p.read_pcm_stats));
   }
   else {
      fprintf(stderr, "Error reading PCM stats\n");
   }

   return (ret);
}

static void init_phone_cfg(bcm_phone_cfg_params_t *prms)
{
   size_t i;

   memset(prms, 0, sizeof(*prms));
   prms->country = BCMPH_COUNTRY_FR;
   prms->pcm_use_16bits_timeslot = prm_use_16bits;
   for (i = 0; (i < ARRAY_SIZE(prm_lines)); i += 1) {
      prms->line_params[i].enable = prm_lines[i].enable;
      prms->line_params[i].codec = prm_lines[i].codec;
   }
}

static int start(int fd)
{
   int ret;
   bcm_phone_cfg_params_t prms;

   init_phone_cfg(&(prms));

   ret = do_ioctl(fd, BCMPH_IOCTL_START, (unsigned long)(&(prms)), 1);
   if (ret) {
      fprintf(stderr, "Error starting\n");
   }

   return (ret);
}

static int start_mm(driver_mem_t *driver_mem)
{
   int ret;
   bcm_phone_ioctl_param_t *ioctl_param = (bcm_phone_ioctl_param_t *)(driver_mem->start_addr + driver_mem->mmap_desc.ioctl_param_off);

   ioctl_param->size = sizeof(bcm_phone_cfg_params_t);

   init_phone_cfg(&(ioctl_param->p.start));

   ret = do_ioctl(driver_mem->fd, BCMPH_IOCTL_START_MM, 0, 1);
   if (!ret) {
      bcm_phone_get_mmap_rbs_location_t *mmap_rbs_location = &(ioctl_param->p.get_mmap_rbs_location);
      memset(mmap_rbs_location, 0, sizeof(*mmap_rbs_location));
      ioctl_param->size = sizeof(*mmap_rbs_location);
      ret = do_ioctl(driver_mem->fd, BCMPH_IOCTL_GET_MMAP_RBS_LOCATION_MM, 0, 1);
      if (ret) {
         fprintf(stderr, "Unable to get ring buffer addresses of lines\n");
      }
      else {
         size_t i;
         for (i = 0; (i < ARRAY_SIZE(driver_mem->lines)); i += 1) {
            bcm_assert(i < ARRAY_SIZE(mmap_rbs_location->rbs));
            if ((sizeof(bcm_ring_buf_desc_t) != mmap_rbs_location->rbs[i].rx_ring_buf_desc_size)
                || (sizeof(bcm_ring_buf_desc_t) != mmap_rbs_location->rbs[i].tx_ring_buf_desc_size)) {
               fprintf(stderr, "Structures shared with the driver are not of the expected size\n");
               ret = -EINVAL;
               break;
            }
            driver_mem->lines[i].rx_ring_buf_desc = (bcm_ring_buf_desc_t *)(driver_mem->start_addr + mmap_rbs_location->rbs[i].rx_ring_buf_desc_off);
            driver_mem->lines[i].tx_ring_buf_desc = (bcm_ring_buf_desc_t *)(driver_mem->start_addr + mmap_rbs_location->rbs[i].tx_ring_buf_desc_off);
            bcm_ring_buf_init(&(driver_mem->lines[i].rx_ring_buf), driver_mem->start_addr + mmap_rbs_location->rbs[i].rx_buffer_offset, mmap_rbs_location->rbs[i].rx_buffer_size);
            bcm_ring_buf_init(&(driver_mem->lines[i].tx_ring_buf), driver_mem->start_addr + mmap_rbs_location->rbs[i].tx_buffer_offset, mmap_rbs_location->rbs[i].tx_buffer_size);
         }
      }
   }
   else {
      fprintf(stderr, "Error starting\n");
   }

   return (ret);
}

static int stop(int fd)
{
   int ret;

   ret = do_ioctl(fd, BCMPH_IOCTL_STOP, 0, 1);
   if (ret) {
      fprintf(stderr, "Error stopping\n");
   }

   return (ret);
}

static int start_pcm(int fd)
{
   int ret;

   ret = do_ioctl(fd, BCMPH_IOCTL_START_PCM, 0, 1);
   if (ret) {
      fprintf(stderr, "Error starting PCM\n");
   }

   return (ret);
}

static int stop_pcm(int fd)
{
   int ret;

   ret = do_ioctl(fd, BCMPH_IOCTL_STOP_PCM, 0, 1);
   if (ret) {
      fprintf(stderr, "Error stopping PCM\n");
   }

   return (ret);
}

static int set_line_mode(int fd, size_t line,
   bcm_phone_line_mode_t mode, __u32 tone)
{
   int ret;
   bcm_phone_set_line_mode_t param;

   memset(&(param), 0, sizeof(param));
   param.line = line;
   param.mode = mode;
   param.tone = tone;
   if (BCMPH_MODE_OFF_TALKING == mode) {
      param.wait = -1;
   }
   else {
      param.wait = 0;
   }
   ret = do_ioctl(fd, BCMPH_IOCTL_SET_LINE_MODE, (unsigned long)(&(param)), 1);
   if (ret) {
      fprintf(stderr, "Error setting mode %d for line %lu\n", (int)(mode), (unsigned long)(line));
   }

   return (ret);
}

static int write_mm(driver_mem_t *driver_mem, size_t line, size_t len)
{
   int ret = do_ioctl(driver_mem->fd, BCMPH_IOCTL_WRITE_MM, ((len << 8) | line), 1);
   if (!ret) {
      update_ring_bufs_desc(driver_mem);
   }
   return (ret);
}

static int read_mm(driver_mem_t *driver_mem, size_t line, size_t len)
{
   int ret = do_ioctl(driver_mem->fd, BCMPH_IOCTL_READ_MM, ((len << 8) | line), 1);
   if (!ret) {
      update_ring_bufs_desc(driver_mem);
   }
   return (ret);
}

#ifndef BCMPH_TEST_PCM
static int get_line_states(int fd, int wait, bcm_phone_get_line_states_t *states)
{
   int ret;

   memset(states, 0, sizeof(*states));
   states->wait = wait;
   ret = do_ioctl(fd, BCMPH_IOCTL_GET_LINE_STATES, (unsigned long)(states), 1);
   if (ret) {
      fprintf(stderr, "Error getting line states\n");
   }

   return (ret);
}
#endif // !BCMPH_TEST_PCM

static const char *device = "/dev/bcm63xx-phone";

static __u8 g_buf_tx[1024 * 80];
static __u8 g_buf_rx[1024 * 20];

static int test_loopback(void)
{
   int ret = 0;
   int fd = -1;

   do { // Empty loop
      size_t todo;
      size_t delay;
      size_t total_done_tx;
      size_t done_tx;
      size_t done_rx;
      ssize_t len;
      const __u8 *buf_tx;
      size_t len_buf_tx;
      __u8 *buf_rx = g_buf_rx;
      size_t len_buf_rx = sizeof(g_buf_rx);

#ifndef BCMPH_TEST_PCM
      switch (prm_lines[prm_default_line].codec) {
         case BCMPH_CODEC_ALAW: {
            buf_tx = demo_thanks_alaw;
            len_buf_tx = demo_thanks_alaw_len;
            break;
         }
         case BCMPH_CODEC_ULAW: {
            buf_tx = demo_thanks_ulaw;
            len_buf_tx = demo_thanks_ulaw_len;
            break;
         }
         case BCMPH_CODEC_LINEAR: {
            buf_tx = demo_thanks_sln;
            len_buf_tx = demo_thanks_sln_len;
            break;
         }
         case BCMPH_CODEC_LINEAR16: {
            buf_tx = demo_thanks_sln16;
            len_buf_tx = demo_thanks_sln16_len;
            break;
         }
         case BCMPH_CODEC_UNSPECIFIED:
         case BCMPH_MAX_CODECS:
         default: {
#endif // !BCMPH_TEST_PCM
            for (done_tx = 0; (done_tx < sizeof(g_buf_tx)); done_tx += 1) {
               g_buf_tx[done_tx] = (__u8)(done_tx);
            }
            buf_tx = g_buf_tx;
            len_buf_tx = sizeof(g_buf_tx);
#ifndef BCMPH_TEST_PCM
            break;
         }
      }
#endif // !BCMPH_TEST_PCM

      memset(buf_rx, 0, len_buf_rx);

      // Open device
      fd = open(device, O_RDWR | O_NONBLOCK);
      if (-1 == fd) {
         perror("");
         fprintf(stderr, "Error opening device '%s'\n", device);
         break;
      }

      read_regs(fd);

      // We configure
      ret = start(fd);
      if (ret) {
         break;
      }

      ret = do_ioctl(fd, BCMPH_IOCTL_SET_DEFAULT_LINE, prm_default_line, 1);
      if (ret) {
         fprintf(stderr, "Unable to set default line\n");
         break;
      }

      // We fill TX buffers before starting PCM
      total_done_tx = 0;
      done_tx = 0;
      todo = len_buf_tx - done_tx;
      len = write(fd, &(buf_tx[done_tx]), todo);
      if (len < 0) {
         if ((EINTR != errno) && (EAGAIN != errno) && (EWOULDBLOCK != errno)) {
            perror("");
            fprintf(stderr, "Error writing to the device\n");
            break;
         }
      }
      else {
         total_done_tx += len;
         done_tx += len;
      }

      // Then we start PCM
      ret = start_pcm(fd);
      if (ret) {
         break;
      }

#ifndef BCMPH_TEST_PCM
      fprintf(stdout, "Hook off the phone and press a key\n");
      fgetc(stdin);
#endif // !BCMPH_TEST_PCM

      ret = set_line_mode(fd, prm_default_line, BCMPH_MODE_OFF_TALKING,
         bcm_phone_line_tone_code_index(BCMPH_TONE_NONE));
      if (ret) {
         break;
      }

      done_rx = 0;
      delay = 0;
      while (done_rx < len_buf_rx) {
         int pause = 1;
         todo = len_buf_rx - done_rx;
         len = read(fd, &(buf_rx[done_rx]), todo);
         if (len < 0) {
            if ((EINTR != errno) && (EAGAIN != errno) && (EWOULDBLOCK != errno)) {
               perror("");
               fprintf(stderr, "Error reading from the device\n");
               break;
            }
         }
         else if (len > 0) {
            // We make a pause to know what happens if TX and RX DMA
            // underflow/overflow
            if ((done_rx < 10240) && ((done_rx + len) >= 10240)) {
               fprintf(stdout, "Making a pause\n");
               usleep(3503456);
            }
            done_rx += len;
            pause = 0;
         }

         if (done_tx >= len_buf_tx) {
            done_tx = 0;
         }
         todo = len_buf_tx - done_tx;
         len = write(fd, &(buf_tx[done_tx]), todo);
         if (len < 0) {
            if ((EINTR != errno) && (EAGAIN != errno) && (EWOULDBLOCK != errno)) {
               perror("");
               fprintf(stderr, "Error writing to the device\n");
               break;
            }
         }
         else if (len > 0) {
            total_done_tx += len;
            done_tx += len;
            pause = 0;
         }
         if (pause) {
            delay += 1;
#ifndef BCMPH_NOHW
            usleep(2000);
            if (delay > 50) {
#else
            usleep(10000);
            if (delay > 100) {
#endif
               delay = 0;
               fprintf(stdout, "%lu bytes written and %lu bytes read...\n", (unsigned long)(total_done_tx), (unsigned long)(done_rx));
            }
         }
      }
      fprintf(stdout, "%ld bytes written to the device\n", (long)(total_done_tx));

      fsync(fd);

#ifdef BCMPH_NOHW
      sleep(3);
#endif

      // We stop PCM
      stop_pcm(fd);

      // We read remaining bytes
      todo = len_buf_rx - done_rx;
      len = read(fd, &(buf_rx[done_rx]), todo);
      if (len < 0) {
         if ((EINTR != errno) && (EAGAIN != errno) && (EWOULDBLOCK != errno)) {
            perror("");
            fprintf(stderr, "Error reading from the device\n");
            break;
         }
      }
      done_rx += len;

      // We stop
      stop(fd);

      // Dump the buffer received
      dump_buf("Bytes received", buf_rx, done_rx);

      read_stats(fd);

#ifdef BCMPH_NOHW
      for (todo = 0; ((todo < done_tx) && (todo < done_rx)); todo += 1) {
         if (buf_tx[todo] != buf_rx[todo]) {
            break;
         }
      }
      if ((todo >= done_tx) || (todo >= done_rx)) {
         fprintf(stdout, "Bytes received are the same as those sent\n");
      }
      else {
         fprintf(stdout, "Byte %lu received differ from the one sent : %x vs %x\n", (unsigned long)(todo), (unsigned int)(buf_rx[todo]), (unsigned int)(buf_tx[todo]));
      }
#endif // BCMPH_NOHW
   } while (0);

   if (-1 != fd) {
      close(fd);
      fd = -1;
   }

   return (ret);
}

static int test_loopback_mm(void)
{
   int ret = 0;
   int fd = -1;
   driver_mem_t driver_mem;
   driver_mem.start_addr = NULL;

   do { // Empty loop
      size_t todo;
      size_t total_done_tx;
      size_t done_tx;
      size_t done_rx;
      size_t delay;
      const __u8 *buf_tx;
      size_t len_buf_tx;
      __u8 *buf_rx = g_buf_rx;
      size_t len_buf_rx = sizeof(g_buf_rx);

#ifndef BCMPH_TEST_PCM
      switch (prm_lines[prm_default_line].codec) {
         case BCMPH_CODEC_ALAW: {
            buf_tx = demo_thanks_alaw;
            len_buf_tx = demo_thanks_alaw_len;
            break;
         }
         case BCMPH_CODEC_ULAW: {
            buf_tx = demo_thanks_ulaw;
            len_buf_tx = demo_thanks_ulaw_len;
            break;
         }
         case BCMPH_CODEC_LINEAR: {
            buf_tx = demo_thanks_sln;
            len_buf_tx = demo_thanks_sln_len;
            break;
         }
         case BCMPH_CODEC_LINEAR16: {
            buf_tx = demo_thanks_sln16;
            len_buf_tx = demo_thanks_sln16_len;
            break;
         }
         case BCMPH_CODEC_UNSPECIFIED:
         case BCMPH_MAX_CODECS:
         default: {
#endif // !BCMPH_TEST_PCM
            for (done_tx = 0; (done_tx < sizeof(g_buf_tx)); done_tx += 1) {
               g_buf_tx[done_tx] = (__u8)(done_tx);
            }
            buf_tx = g_buf_tx;
            len_buf_tx = sizeof(g_buf_tx);
#ifndef BCMPH_TEST_PCM
            break;
         }
      }
#endif // !BCMPH_TEST_PCM

      memset(buf_rx, 0, len_buf_rx);

      // Open device
      fd = open(device, O_RDWR);
      if (-1 == fd) {
         perror("");
         fprintf(stderr, "Error opening device '%s'\n", device);
         break;
      }

      ret = get_driver_mem(fd, &(driver_mem));
      if (ret) {
         break;
      }

      read_regs_mm(&(driver_mem));

      // We configure
      ret = start_mm(&(driver_mem));
      if (ret) {
         break;
      }

      // We fill TX buffers before starting PCM
      total_done_tx = 0;
      done_tx = 0;
      todo = bcm_ring_buf_get_free_space(&(driver_mem.lines[prm_default_line].tx_ring_buf));
      if (todo > (len_buf_tx - done_tx)) {
         todo = len_buf_tx - done_tx;
      }
      bcm_ring_buf_add(&(driver_mem.lines[prm_default_line].tx_ring_buf), &(buf_tx[done_tx]), todo);
      ret = write_mm(&(driver_mem), prm_default_line, todo);
      if (ret) {
         break;
      }
      done_tx += todo;
      total_done_tx += todo;

      // Then we start PCM
      ret = start_pcm(fd);
      if (ret) {
         break;
      }

#ifndef BCMPH_TEST_PCM
      fprintf(stdout, "Hook off the phone and press a key\n");
      fgetc(stdin);
#endif // !BCMPH_TEST_PCM

      ret = set_line_mode(driver_mem.fd, prm_default_line, BCMPH_MODE_OFF_TALKING,
         bcm_phone_line_tone_code_index(BCMPH_TONE_NONE));
      if (ret) {
         break;
      }

      do_ioctl(driver_mem.fd, BCMPH_IOCTL_UPDATE_RBS, 0, 1);
      update_ring_bufs_desc(&(driver_mem));

      done_rx = 0;
      delay = 0;
      while (done_rx < len_buf_rx) {
         int pause = 1;
         todo = bcm_ring_buf_get_size(&(driver_mem.lines[prm_default_line].rx_ring_buf));
         if (todo > (len_buf_rx - done_rx)) {
            todo = len_buf_rx - done_rx;
         }
         if (todo > 0) {
            bcm_ring_buf_remove(&(driver_mem.lines[prm_default_line].rx_ring_buf), &(buf_rx[done_rx]), todo);
            ret = read_mm(&(driver_mem), prm_default_line, todo);
            if (ret) {
               break;
            }
            done_rx += todo;
            pause = 0;
         }

         if (done_tx >= len_buf_tx) {
            done_tx = 0;
         }
         todo = bcm_ring_buf_get_free_space(&(driver_mem.lines[prm_default_line].tx_ring_buf));
         if (todo > (len_buf_tx - done_tx)) {
            todo = len_buf_tx - done_tx;
         }
         if (todo > 0) {
            bcm_ring_buf_add(&(driver_mem.lines[prm_default_line].tx_ring_buf), &(buf_tx[done_tx]), todo);
            ret = write_mm(&(driver_mem), prm_default_line, todo);
            if (ret) {
               break;
            }
            done_tx += todo;
            total_done_tx += todo;
            pause = 0;
         }
         if (pause) {
            delay += 1;
#ifndef BCMPH_NOHW
            usleep(2000);
            if (delay > 500) {
#else
            usleep(10000);
            if (delay > 100) {
#endif
               delay = 0;
               fprintf(stdout, "%lu bytes written and %lu bytes read...\n", (unsigned long)(total_done_tx), (unsigned long)(done_rx));
            }
            // Update ring buf descriptor
            ret = write_mm(&(driver_mem), prm_default_line, 0);
            if (ret) {
               break;
            }
         }
      }
      fprintf(stdout, "%lu bytes written to the device\n", (unsigned long)(done_tx));

      fsync(fd);

#ifdef BCMPH_NOHW
      sleep(3);
#endif

      // We stop PCM
      stop_pcm(fd);

      // We read remaining bytes
      ret = read_mm(&(driver_mem), prm_default_line, 0);
      if (ret) {
         break;
      }
      todo = bcm_ring_buf_get_size(&(driver_mem.lines[prm_default_line].rx_ring_buf));
      if (todo > (len_buf_rx - done_rx)) {
         todo = len_buf_rx - done_rx;
      }
      bcm_ring_buf_remove(&(driver_mem.lines[prm_default_line].rx_ring_buf), &(buf_rx[done_rx]), todo);
      ret = read_mm(&(driver_mem), prm_default_line, todo);
      if (ret) {
         break;
      }
      done_rx += todo;

      // We stop
      stop(fd);

      // Dump the buffer received
      dump_buf("Bytes received", buf_rx, done_rx);

      read_stats_mm(&(driver_mem));

#ifdef BCMPH_NOHW
      for (todo = 0; ((todo < done_tx) && (todo < done_rx)); todo += 1) {
         if (buf_tx[todo] != buf_rx[todo]) {
            break;
         }
      }
      if ((todo >= done_tx) || (todo >= done_rx)) {
         fprintf(stdout, "Bytes received are the same as those sent\n");
      }
      else {
         fprintf(stdout, "Byte %lu received differ from the one sent : %x vs %x\n", (unsigned long)(todo), (unsigned int)(buf_rx[todo]), (unsigned int)(buf_tx[todo]));
      }
#endif // BCMPH_NOHW
   } while (0);

   if (-1 != fd) {
      if (NULL != driver_mem.start_addr) {
         munmap(driver_mem.start_addr, driver_mem.mmap_desc.mmap_size);
         driver_mem.start_addr = NULL;
      }
      close(fd);
      fd = -1;
   }

   return (ret);
}

static int kbhit()
{
   struct timeval tv = { 0L, 0L };
   fd_set fds;
   FD_ZERO(&(fds));
   FD_SET(0, &(fds));
   return (select(1, &(fds), NULL, NULL, &tv));
}

static int test_echo(void)
{
   int ret = 0;
   int fd = -1;
   int fd_out = -1;

   do { // Empty loop
      size_t bytes_read = 0;
      const __u8 *buf_tx = g_buf_tx;
      size_t len_buf_tx = sizeof(g_buf_tx);
      __u8 *buf_rx = g_buf_rx;
      size_t len_buf_rx = sizeof(g_buf_rx);
      size_t sample_size;
      int default_wait_time;
      ssize_t len;
      int wait_time;
      size_t default_packet_size;
#ifndef BCMPH_TEST_PCM
      bcm_phone_get_line_states_t states;
#endif // !BCMPH_TEST_PCM

#ifndef BCMPH_TEST_PCM
      default_packet_size = 160;
      switch (prm_lines[prm_default_line].codec) {
         case BCMPH_CODEC_ALAW:
         case BCMPH_CODEC_ULAW: {
            sample_size = 1;
            break;
         }
         case BCMPH_CODEC_LINEAR: {
            sample_size = 2;
            break;
         }
         case BCMPH_CODEC_LINEAR16:
         case BCMPH_CODEC_UNSPECIFIED:
         case BCMPH_MAX_CODECS:
         default: {
            sample_size = 4;
            break;
         }
      }
#else // BCMPH_TEST_PCM
      default_packet_size = 1024;
      sample_size = 32;
#endif // BCMPH_TEST_PCM
      default_wait_time = default_packet_size / (8 * sample_size);

      if ((NULL != prm_output) && ('\0' != prm_output[0])) {
         fd_out = open(prm_output, O_CREAT | O_WRONLY | O_TRUNC, S_IRUSR | S_IWUSR);
         if (-1 == fd_out) {
            perror("");
            fprintf(stderr, "Error opening output file '%s'\n", prm_output);
            break;
         }
      }

      // Open device
      fd = open(device, O_RDWR);
      if (-1 == fd) {
         perror("");
         fprintf(stderr, "Error opening device '%s'\n", device);
         break;
      }

      memset(g_buf_tx, 0, sizeof(g_buf_tx));
      memset(buf_rx, 0, len_buf_rx);

      // We configure
      ret = start(fd);
      if (ret) {
         break;
      }

      ret = do_ioctl(fd, BCMPH_IOCTL_SET_DEFAULT_LINE, prm_default_line, 1);
      if (ret) {
         fprintf(stderr, "Unable to set default line\n");
         break;
      }

#ifndef BCMPH_TEST_PCM
      ret = get_line_states(fd, 0, &(states));
      if (ret) {
         break;
      }

      // We make the phone ringing
      if (BCMPH_STATUS_DISCONNECTED == states.line_state[prm_default_line].status) {
         fprintf(stdout, "Line %lu is disconnected\n", (unsigned long)(prm_default_line));
         break;
      }

      // We wait for the line to become off hook
      if (BCMPH_STATUS_ON_HOOK == states.line_state[prm_default_line].status) {
         ret = set_line_mode(fd, prm_default_line, BCMPH_MODE_ON_RINGING,
            bcm_phone_line_tone_code_index(BCMPH_TONE_NONE));
         if (ret) {
            break;
         }
         fprintf(stdout, "Line %lu should be ringing\n", (unsigned long)(prm_default_line));
         for (;;) {
            fprintf(stdout, "Waiting for the line %lu to be off hook...\n", (unsigned long)(prm_default_line));
            ret = get_line_states(fd, -1, &(states));
            if (ret) {
               break;
            }
            if (BCMPH_STATUS_DISCONNECTED == states.line_state[prm_default_line].status) {
               fprintf(stdout, "Line %lu is disconnected\n", (unsigned long)(prm_default_line));
               break;
            }
            if (BCMPH_STATUS_OFF_HOOK == states.line_state[prm_default_line].status) {
               fprintf(stdout, "Line %lu is off hook\n", (unsigned long)(prm_default_line));
               set_line_mode(fd, prm_default_line, BCMPH_MODE_IDLE,
                  bcm_phone_line_tone_code(prm_tone, prm_tone_on_time, prm_tone_off_time));
               sleep(3);
               break;
            }
            if (BCMPH_MODE_ON_RINGING != states.line_state[prm_default_line].mode) {
               fprintf(stdout, "Line %lu is no more ringing. New mode is %d\n", (unsigned long)(prm_default_line), (int)(states.line_state[prm_default_line].mode));
            }
         }
         if (ret) {
            break;
         }
      }
      if (BCMPH_STATUS_OFF_HOOK != states.line_state[prm_default_line].status) {
         break;
      }
      else {
#endif // !BCMPH_TEST_PCM
         // Now we start pcm
         ret = start_pcm(fd);
         if (ret) {
            break;
         }

         ret = set_line_mode(fd, prm_default_line, BCMPH_MODE_OFF_TALKING,
            bcm_phone_line_tone_code_index(BCMPH_TONE_NONE));
         if (ret) {
            break;
         }
         fprintf(stdout, "Line %lu is in talking mode\n", (unsigned long)(prm_default_line));
#ifndef BCMPH_TEST_PCM
      }
#endif // !BCMPH_TEST_PCM

      // We fill completely TX buffers
      len = write(fd, buf_tx, len_buf_tx);
      if (len < 0) {
         if ((EINTR != errno) && (EAGAIN != errno) && (EWOULDBLOCK != errno)) {
            perror("");
            fprintf(stderr, "Error writing to the device\n");
            break;
         }
      }
      fprintf(stdout, "Wrote %ld bytes\n", (long)(len));

#ifdef BCMPH_TEST_PCM
      // We wait a few ms */
      usleep(1000000);

      // We fill completely TX buffers again
      len = write(fd, buf_tx, len_buf_tx);
      if (len < 0) {
         if ((EINTR != errno) && (EAGAIN != errno) && (EWOULDBLOCK != errno)) {
            perror("");
            fprintf(stderr, "Error writing to the device\n");
            break;
         }
      }
      fprintf(stdout, "Wrote %ld bytes\n", (long)(len));
#endif // BCMPH_TEST_PCM

      // We loop : we read data, then write what we have read and watch if the line
      // is on hook
      for (;;) {
         wait_time = default_wait_time;
         if ((len_buf_rx - bytes_read) > default_packet_size) {
            len = read(fd, &(buf_rx[bytes_read]), default_packet_size);
         }
         else {
            len = read(fd, &(buf_rx[bytes_read]), len_buf_rx - bytes_read);
         }
         if (len < 0) {
            if ((EINTR != errno) && (EAGAIN != errno) && (EWOULDBLOCK != errno)) {
               perror("");
               fprintf(stderr, "Error reading from the device\n");
               break;
            }
            // Perhaps there is data to read, so
            // just get line state and do not wait for a change
            wait_time = 0;
         }
         else {
            if (((size_t)(len)) >= default_packet_size) {
               // There is probably more data to read, so
               // just get line state and do not wait for a change
               wait_time = 0;
            }

            if (0 != (len % sample_size)) {
               fprintf(stdout, "Read %lu bytes\n", (unsigned long)(len));
            }

            if (kbhit()) {
               int c = fgetc(stdin);
               if (' ' == c) {
                  dump_buf("Received : ", &(buf_rx[bytes_read]), len);
               }
            }

            //swap_bytes(&(buf_rx[bytes_read]), len);

            bytes_read += len;

            while (len > 0) {
               ssize_t done;

               done = write(fd, &(buf_rx[bytes_read - len]), len);
               if (done >= 0) {
                  if (done < len) {
                     if (0 == done) {
                        fprintf(stdout, "%ld bytes not written\n", len);
                        len = 0;
                        break;
                     }
                     else {
                        len -= done;
                     }
                  }
                  else {
                     len = 0;
                  }
               }
               else {
                  if ((EINTR != errno) && (EAGAIN != errno) && (EWOULDBLOCK != errno)) {
                     perror("");
                     fprintf(stderr, "Error writing to the device\n");
                     break;
                  }
               }
            }
            if (len > 0) {
               break;
            }
         }

         if ((len_buf_rx - bytes_read) <= 0) {
            if (-1 != fd_out) {
               write(fd_out, buf_rx, bytes_read);
            }
            bytes_read = 0;
         }

#ifndef BCMPH_TEST_PCM
         ret = get_line_states(fd, wait_time, &(states));
         if (ret) {
            break;
         }
         if (states.line_state[prm_default_line].codec_change_count > 0) {
            fprintf(stdout, "Codec changes %u time(s). Codec is now %d\n",
               (unsigned int)(states.line_state[prm_default_line].codec_change_count),
               (int)(states.line_state[prm_default_line].codec));
         }
         if (states.line_state[prm_default_line].mode_change_count > 0) {
            fprintf(stdout, "Mode changes %u time(s). Mode is now %d\n",
               (unsigned int)(states.line_state[prm_default_line].mode_change_count),
               (int)(states.line_state[prm_default_line].mode));
         }
         if (states.line_state[prm_default_line].tone_change_count > 0) {
            fprintf(stdout, "Tone changes %u time(s). Tone is now %d\n",
               (unsigned int)(states.line_state[prm_default_line].tone_change_count),
               (int)(states.line_state[prm_default_line].tone));
         }
         if (states.line_state[prm_default_line].digits_count > 0) {
            size_t i;

            fprintf(stdout, "Digit(s) received => ");
            for (i = 0; (i < states.line_state[prm_default_line].digits_count); i += 1) {
               fprintf(stdout, "%c", (char)(states.line_state[prm_default_line].digits[i]));
            }
            fprintf(stdout, "\n");
         }
         if (states.line_state[prm_default_line].flash_count > 0) {
            fprintf(stdout, "%u flash(es) received\n", (unsigned int)(states.line_state[prm_default_line].flash_count));
         }
         if (BCMPH_STATUS_OFF_HOOK != states.line_state[prm_default_line].status) {
            if (BCMPH_STATUS_ON_HOOK == states.line_state[prm_default_line].status) {
               fprintf(stdout, "Line %lu is on hook\n", (unsigned long)(prm_default_line));
            }
            else if (BCMPH_STATUS_DISCONNECTED == states.line_state[prm_default_line].status) {
               fprintf(stdout, "Line %lu is disconnected\n", (unsigned long)(prm_default_line));
            }
            break;
         }
         if (BCMPH_MODE_OFF_TALKING != states.line_state[prm_default_line].mode) {
            fprintf(stdout, "Line %lu is no more in talking mode. New mode is %d\n", (unsigned long)(prm_default_line), (int)(states.line_state[prm_default_line].mode));
            break;
         }
#else // BCMPH_TEST_PCM
         if (wait_time > 0) {
            usleep(wait_time * 1000);
         }
#endif // BCMPH_TEST_PCM
      }

      // We stop PCM
      stop_pcm(fd);

      // We stop
      stop(fd);
   } while (0);

   if (-1 != fd) {
      close(fd);
      fd = -1;
   }

   if (-1 != fd_out) {
      close(fd_out);
      fd_out = -1;
   }

   return (ret);
}

static int test_tone(void)
{
   int ret = 0;
   int fd = -1;

   do { // Empty loop
#ifndef BCMPH_TEST_PCM
      bcm_phone_get_line_states_t states;
#endif // !BCMPH_TEST_PCM

      // Open device
      fd = open(device, O_RDWR);
      if (-1 == fd) {
         perror("");
         fprintf(stderr, "Error opening device '%s'\n", device);
         break;
      }

      // We configure
      ret = start(fd);
      if (ret) {
         break;
      }

      ret = do_ioctl(fd, BCMPH_IOCTL_SET_DEFAULT_LINE, prm_default_line, 1);
      if (ret) {
         fprintf(stderr, "Unable to set default line\n");
         break;
      }

#ifndef BCMPH_TEST_PCM
      ret = get_line_states(fd, 0, &(states));
      if (ret) {
         break;
      }

      // We make the phone ringing
      if (BCMPH_STATUS_DISCONNECTED == states.line_state[prm_default_line].status) {
         fprintf(stdout, "Line %lu is disconnected\n", (unsigned long)(prm_default_line));
         break;
      }

      // We wait for the line to become off hook
      if (BCMPH_STATUS_ON_HOOK == states.line_state[prm_default_line].status) {
         ret = set_line_mode(fd, prm_default_line, BCMPH_MODE_ON_RINGING,
            bcm_phone_line_tone_code_index(BCMPH_TONE_NONE));
         if (ret) {
            break;
         }
         fprintf(stdout, "Line %lu should be ringing\n", (unsigned long)(prm_default_line));
         for (;;) {
            fprintf(stdout, "Waiting for the line %lu to be off hook...\n", (unsigned long)(prm_default_line));
            ret = get_line_states(fd, -1, &(states));
            if (ret) {
               break;
            }
            if (BCMPH_STATUS_DISCONNECTED == states.line_state[prm_default_line].status) {
               fprintf(stdout, "Line %lu is disconnected\n", (unsigned long)(prm_default_line));
               break;
            }
            if (BCMPH_STATUS_OFF_HOOK == states.line_state[prm_default_line].status) {
               fprintf(stdout, "Line %lu is off hook\n", (unsigned long)(prm_default_line));
               set_line_mode(fd, prm_default_line, BCMPH_MODE_IDLE,
                  bcm_phone_line_tone_code(prm_tone, prm_tone_on_time, prm_tone_off_time));
               break;
            }
            if (BCMPH_MODE_ON_RINGING != states.line_state[prm_default_line].mode) {
               fprintf(stdout, "Line %lu is no more ringing. New mode is %d\n", (unsigned long)(prm_default_line), (int)(states.line_state[prm_default_line].mode));
            }
         }
         if (ret) {
            break;
         }
      }
      if (BCMPH_STATUS_OFF_HOOK != states.line_state[prm_default_line].status) {
         break;
      }
#endif // !BCMPH_TEST_PCM

      // We loop : we watch if the line is on hook
      for (;;) {
#ifndef BCMPH_TEST_PCM
         ret = get_line_states(fd, 1000, &(states));
         if (ret) {
            break;
         }
         if (states.line_state[prm_default_line].codec_change_count > 0) {
            fprintf(stdout, "Codec changes %u time(s). Codec is now %d\n",
               (unsigned int)(states.line_state[prm_default_line].codec_change_count),
               (int)(states.line_state[prm_default_line].codec));
         }
         if (states.line_state[prm_default_line].mode_change_count > 0) {
            fprintf(stdout, "Mode changes %u time(s). Mode is now %d\n",
               (unsigned int)(states.line_state[prm_default_line].mode_change_count),
               (int)(states.line_state[prm_default_line].mode));
         }
         if (states.line_state[prm_default_line].tone_change_count > 0) {
            fprintf(stdout, "Tone changes %u time(s). Tone is now %d\n",
               (unsigned int)(states.line_state[prm_default_line].tone_change_count),
               (int)(states.line_state[prm_default_line].tone));
         }
         if (states.line_state[prm_default_line].digits_count > 0) {
            size_t i;

            fprintf(stdout, "Digit(s) received => ");
            for (i = 0; (i < states.line_state[prm_default_line].digits_count); i += 1) {
               fprintf(stdout, "%c", (char)(states.line_state[prm_default_line].digits[i]));
            }
            fprintf(stdout, "\n");
         }
         if (states.line_state[prm_default_line].flash_count > 0) {
            fprintf(stdout, "%u flash(es) received\n", (unsigned int)(states.line_state[prm_default_line].flash_count));
         }
         if (BCMPH_STATUS_OFF_HOOK != states.line_state[prm_default_line].status) {
            if (BCMPH_STATUS_ON_HOOK == states.line_state[prm_default_line].status) {
               fprintf(stdout, "Line %lu is on hook\n", (unsigned long)(prm_default_line));
            }
            else if (BCMPH_STATUS_DISCONNECTED == states.line_state[prm_default_line].status) {
               fprintf(stdout, "Line %lu is disconnected\n", (unsigned long)(prm_default_line));
            }
            break;
         }
#else // BCMPH_TEST_PCM
         usleep(1000000);
#endif // BCMPH_TEST_PCM
      }

      // We stop PCM
      stop_pcm(fd);

      // We stop
      stop(fd);
   } while (0);

   if (-1 != fd) {
      close(fd);
      fd = -1;
   }

   return (ret);
}

typedef struct {
   int fd;
   size_t total_read;
   char buffer[1024];
   size_t pos;
   size_t pos_max;
} stream_t;

static int get_next_char(stream_t *s)
{
   int ret;
   if (s->pos < s->pos_max) {
      ret = s->buffer[s->pos];
      s->pos += 1;
   }
   else {
      ssize_t bytes_read = read(s->fd, s->buffer, sizeof(s->buffer));
      if (bytes_read <= 0) {
         if (bytes_read < 0) {
            fprintf(stderr, "Failed to read from input file\n");
         }
         ret = bytes_read;
      }
      else {
         ret = s->buffer[0];
         s->pos = 1;
         s->pos_max = (size_t)(bytes_read);
         s->total_read += (size_t)(bytes_read);
      }
   }
   return (ret);
}

static int get_direction(stream_t *s)
{
   int ret;

   for (;;) {
      ret = get_next_char(s);
      if (ret <= 0) {
         break;
      }
      if (!isspace(ret)) {
         ret = toupper(ret);
         if ((ret != 'R') && (ret != 'W') && (ret != 'M')) {
            fprintf(stderr, "Unknown direction\n");
            ret = -1;
         }
         break;
      }
   }

   return (ret);
}

static int get_length(stream_t *s)
{
   int ret;

   for (;;) {
      ret = get_next_char(s);
      if (ret < 0) {
         break;
      }
      if (!isspace(ret)) {
         char length[16];
         size_t i = 0;

         for (i = 0; (i < ARRAY_SIZE(length)); i += 1) {
            if (!isdigit(ret)) {
               break;
            }
            length[i] = ret;
            ret = get_next_char(s);
            if (ret < 0) {
               break;
            }
         }
         if (ret >= 0) {
            if ((i <= 0) || (i >= ARRAY_SIZE(length))) {
               fprintf(stderr, "Invalid length\n");
               ret = -1;
            }
            else {
               length[i] = 0;
               if (isspace(ret)) {
                  ret = atoi(length);
                  if ((ret < 0) || (ret > 256)) {
                     fprintf(stderr, "Invalid length\n");
                     ret = -1;
                  }
               }
               else {
                  fprintf(stderr, "Wrong format\n");
                  ret = -1;
               }
            }
         }
         break;
      }
   }

   return (ret);
}

static int get_bytes(stream_t *s, __u8 *bytes, size_t len)
{
   int ret;
   size_t i;
   int first;

   i = 0;
   first = 1;
   while (i < len) {
      ret = get_next_char(s);
      if (ret < 0) {
         break;
      }
      if (!isspace(ret)) {
         __u8 v;
         if (isdigit(ret)) {
            v = (__u8)(ret - '0');
         }
         else {
            ret = toupper(ret);
            if (('A' == ret) || ('B' == ret) || ('C' == ret)
                || ('D' == ret) || ('E' == ret) || ('F' == ret)) {
               v = (__u8)(ret - 'A' + 10);
            }
            else {
               fprintf(stderr, "Wrong format\n");
               ret = -1;
               break;
            }
         }
         if (first) {
            bytes[i] = v;
            first = 0;
         }
         else {
            bytes[i] = (bytes[i] << 4) + v;
            i += 1;
            first = 1;
         }
      }
      else {
         if (!first) {
            i += 1;
            first = 1;
         }
      }
   }

   return (ret);
}

#ifdef BCMPH_NOHW

#undef ARRAY_SIZE

typedef struct {
   __u8 cmd;
   __u8 data_len;
   const char *name;
} zarlink_cmd_desc_t;

#include <vp_api.h>
#include <vp880_api_int.h>

static zarlink_cmd_desc_t le88221_cmds[] = {
   /* Common */
   { VP880_SW_RESET_WRT, VP880_SW_RESET_LEN, "Software reset VP880_SW_RESET_WRT" }, /* 0x02, 0 */
   { VP880_HW_RESET_WRT, 0, "Hardware Reset VP880_HW_RESET_WRT" }, /* 0x04, 0 */
   { VP880_NO_OP_WRT, 0, "No Operation VP880_NO_OP_WRT" }, /* 0x06, 0 */
   { VP880_XR_CS_RD, VP880_XR_CS_LEN, "Transmit and Receive Clock Slot and Transmit Clock Edge VP880_XR_CS" }, /* 0x45, 1 */
   { VP880_MCLK_CNT_RD, VP880_MCLK_CNT_LEN, "Device Configuration Register VP880_MCLK_CNT" }, /* 0x47, 1 */
   { VP880_OP_MODE_RD, VP880_OP_MODE_LEN, "Channel Enable and Operation Mode VP880_OP_MODE" }, /* 0x4B, 1 */
   { VP880_NO_UL_SIGREG_RD, VP880_NO_UL_SIGREG_LEN, "Read Signaling Register without Clearing VP880_NO_UL_SIGREG" }, /* 0x4D, 2 */
   { VP880_UL_SIGREG_RD, VP880_UL_SIGREG_LEN, "Read Signaling Register and Clear VP880_UL_SIGREG" }, /* 0x4F, 2 */
   { VP880_INT_MASK_RD, VP880_INT_MASK_LEN, "Interrupt Mask VP880_INT_MASK" }, /* 0x6D, 2 */
   { VP880_RCN_PCN_RD, VP880_RCN_PCN_LEN, "Read Revision and Product Code Number VP880_RCN_PCN" }, /* 0x73, 2 */

   /* Line specific */
   { VP880_TX_TS_RD, VP880_TX_TS_LEN, "Transmit Time Slot VP880_TX_TS" }, /* 0x41, 1 */
   { VP880_RX_TS_RD, VP880_RX_TS_LEN, "Receive Time Slot VP880_RX_TS" }, /* 0x43, 1 */
   { VP880_VP_GAIN_RD, VP880_VP_GAIN_LEN, "Voice Path Gains VP880_VP_GAIN" }, /* 0x51, 1 */
   { VP880_IODATA_REG_RD, VP880_IODATA_REG_LEN, "Input/Output Data VP880_IODATA_REG" }, /* 0x53, 1 */
   { VP880_IODIR_REG_RD, VP880_IODIR_REG_LEN, "Input/Output Direction VP880_IODIR_REG" }, /* 0x55, 1 */
   { VP880_SYS_STATE_RD, VP880_SYS_STATE_LEN, "System State VP880_SYS_STATE" }, /* 0x57, 1 */
   { VP880_OP_FUNC_RD, VP880_OP_FUNC_LEN, "Operating Functions VP880_OP_FUNC" }, /* 0x61, 1 */
   { VP880_SS_CONFIG_RD, VP880_SS_CONFIG_LEN, "System State Configuration VP880_SS_CONFIG" }, /* 0x69, 1 */
   { VP880_OP_COND_RD, VP880_OP_COND_LEN, "Operating Conditions VP880_OP_COND" }, /* 0x71, 1 */
   { VP880_GX_GAIN_RD, VP880_GX_GAIN_LEN, "GX Filter Coefficients VP880_GX_GAIN" }, /* 0x81, 2 */
   { VP880_GR_GAIN_RD, VP880_GR_GAIN_LEN, "GR Filter Coefficients VP880_GR_GAIN" }, /* 0x83, 2 */
   { VP880_B1_FILTER_RD, VP880_B1_FILTER_LEN, "B Filter FIR Coefficients VP880_B1_FILTER" }, /* 0x87, 14 */
   { VP880_X_FILTER_RD, VP880_X_FILTER_LEN, "X Filter Coefficients VP880_X_FILTER" }, /* 0x89, 12 */
   { VP880_R_FILTER_RD, VP880_R_FILTER_LEN, "R Filter Coefficients VP880_R_FILTER" }, /* 0x8B, 14 */
   { VP880_B2_FILTER_RD, VP880_B2_FILTER_LEN, "B IIR Filter Coefficients VP880_B2_FILTER" }, /* 0x97, 2 */
   { VP880_Z1_FILTER_RD, VP880_Z1_FILTER_LEN, "Z FIR Filter Coefficients VP880_Z1_FILTER" }, /* 0x99, 10 */
   { VP880_Z2_FILTER_RD, VP880_Z2_FILTER_LEN, "Z IIR Filter Coefficients VP880_Z2_FILTER" }, /* 0x9B, 5 */
   { VP880_CONV_CFG_RD, VP880_CONV_CFG_LEN, "Converter Configuration VP880_CONV_CFG" }, /* 0xA7, 1 */
   { VP880_LOOP_SUP_RD, VP880_LOOP_SUP_LEN, "Loop Supervision Parameters VP880_LOOP_SUP" }, /* 0xC3, 4 */
   { VP880_DC_FEED_RD, VP880_DC_FEED_LEN, "DC Feed Parameters VP880_DC_FEED" }, /* 0xC7, 2 */
   { VP880_DISN_RD, VP880_DISN_LEN, "Digital Impedance Scaling Network VP880_DISN" }, /* 0xCB, 1 */
   { VP880_TX_PCM_DATA_RD, VP880_TX_PCM_DATA_LEN, "Read Transmit PCM/Test Data VP880_TX_PCM_DATA" }, /* 0xCD, 2 */
   { VP880_METERING_PARAM_RD, VP880_METERING_PARAM_LEN, "Metering Parameters VP880_METERING_PARAM" }, /* 0xD1, 4 */
   { VP880_SIGA_PARAMS_RD, VP880_SIGA_PARAMS_LEN, "Signal Generator A, B and Bias Parameters VP880_SIGA_PARAMS" }, /* 0xD3, 11 */
   { VP880_SIGCD_PARAMS_RD, VP880_SIGCD_PARAMS_LEN, "Signal Generator C and D Parameters VP880_SIGCD_PARAMS" }, /* 0xD5, 8 */
   { VP880_GEN_CTRL_RD, VP880_GEN_CTRL_LEN, "Signal Generator Control VP880_GEN_CTRL" }, /* 0xDF, 1 */
   { VP880_CADENCE_TIMER_RD, VP880_CADENCE_TIMER_LEN, "Cadence Timer VP880_CADENCE_TIMER" }, /* 0xE1, 4 */
   { VP880_CID_DATA_RD, VP880_CID_DATA_LEN, "Caller Identification Number Data VP880_CID_DATA" }, /* 0xE3, 1 */
   { VP880_REGULATOR_PARAM_RD, VP880_REGULATOR_PARAM_LEN, "Switching Regulator Parameters VP880_REGULATOR_PARAM" }, /* 0xE5, 3 */
   { VP880_REGULATOR_CTRL_RD, VP880_REGULATOR_CTRL_LEN, "Switching Regulator Control VP880_REGULATOR_CTRL" }, /* 0xE7, 1 */
   { VP880_CID_PARAM_RD, VP880_CID_PARAM_LEN, "Caller Identification Number Parameters VP880_CID_PARAM" }, /* 0xEB, 1 */

   /* Undocumented */
   { VP880_BAT_CALIBRATION_RD, VP880_BAT_CALIBRATION_LEN, "VP880_BAT_CALIBRATION" }, /* 0xE8/0xE9, 2 */
   { VP880_ICR1_RD, VP880_ICR1_LEN, "VP880_ICR1" }, /* 0xEC/0xED, 4 */
   { VP880_ICR2_RD, VP880_ICR2_LEN, "VP880_ICR2" }, /* 0xEE/0xEF, 4 */
   { VP880_ICR3_RD, VP880_ICR3_LEN, "VP880_ICR3" }, /* 0xF2/0xF3, 4 */
   { VP880_ICR4_RD, VP880_ICR4_LEN, "VP880_ICR4" }, /* 0xF4/0xF5, 4 */
   { VP880_ICR5_RD, VP880_ICR5_LEN, "VP880_ICR5" }, /* 0xFA/0xFB, 2 */
   { VP880_ICR6_RD, VP880_ICR6_LEN, "VP880_ICR6" }, /* 0xFC/0xFD, 2 */
};

typedef struct {
   int dir;
   zarlink_cmd_desc_t *cmd;
   __u8 data[256];
   size_t data_len;
   size_t seq_number;
} decoder_t;

static void print_cmd(int dir, zarlink_cmd_desc_t *cmd, __u8 *data)
{
   size_t i;
   __u8 cmd_code = cmd->cmd;
   if ('R' == dir) {
      cmd_code |= 0x01;
   }
   else {
      cmd_code &= (~(0x01));
   }
   fprintf(stdout, "%72s_%c 0x%02X => ", cmd->name, (char)(dir), (unsigned int)(cmd_code));
   for (i = 0; (i < cmd->data_len); i += 1) {
      fprintf(stdout, "0x%02X ", (unsigned int)(data[i]));
   }
   fprintf(stdout, "\n");
}

static int interpret_bytes(int dir, __u8 *bytes, size_t len, decoder_t *d)
{
   int ret = 0;
   size_t i = 0;

   while (i < len) {
      size_t j;
      __u8 cmd;

      if ((NULL != d->cmd) && (d->data_len < d->cmd->data_len)) {
         if ((d->cmd->data_len - d->data_len) > (len - i)) {
            memcpy(&(d->data[d->data_len]), &(bytes[i]), len - i);
            d->data_len += (len - i);
            i = len;
         }
         else {
            memcpy(&(d->data[d->data_len]), &(bytes[i]), d->cmd->data_len - d->data_len);
            i += (d->cmd->data_len - d->data_len);
            d->data_len = d->cmd->data_len;
            if ('R' == dir) {
               if (i < len) {
                  fprintf(stderr, "Unexpected bytes following data of read cmd 0x%02X\n", (unsigned int)(d->cmd->cmd));
                  ret = -1;
                  break;
               }
            }
         }
         if (d->data_len >= d->cmd->data_len) {
            print_cmd(dir, d->cmd, d->data);
         }
         continue;
      }

      cmd = bytes[i];
      i += 1;
      d->data_len = 0;
      if ('W' == dir) {
         if ((cmd & 0x01)) {
            if (i < len) {
               fprintf(stderr, "Unexpected bytes following read cmd 0x%02X\n", (unsigned int)(cmd));
               ret = -1;
               break;
            }
         }
      }
      for (j = 0; (j < ARRAY_SIZE(le88221_cmds)); j += 1) {
         if ((le88221_cmds[j].cmd | 0x01) == (cmd | 0x01)) {
            d->dir = dir;
            d->cmd = &(le88221_cmds[j]);
            break;
         }
      }
      if (j >= ARRAY_SIZE(le88221_cmds)) {
         fprintf(stderr, "Unknown cmd 0x%02X\n", (unsigned int)(cmd));
         ret = -1;
         break;
      }
   }

   return (ret);
}

static int decode_mpi_data(void)
{
   int ret = -1;
   stream_t s;

   memset(&(s), 0, sizeof(s));
   s.fd = -1;

   /* Open the file */
   do { /* Empty loop */
      decoder_t decoder;

      if ((NULL == prm_input) || ('\0' == prm_input[0])) {
         fprintf(stderr, "No input file to decode\n");
         break;
      }

      s.fd = open(prm_input, O_RDONLY);
      if (-1 == s.fd) {
         fprintf(stderr, "Can't open input file '%s'\n", prm_input);
         break;
      }

      memset(&(decoder), 0, sizeof(decoder));
      for (;;) {
         int dir;
         int len;
         __u8 bytes[256];

         dir = get_direction(&(s));
         if (dir <= 0) {
            ret = dir;
            break;
         }

         if ('M' == dir) {
            fprintf(stdout, "\n************************************************************************\n\n");
            continue;
         }

         len = get_length(&(s));
         if (len <= 0) {
            ret = len;
            break;
         }

         if (get_bytes(&(s), bytes, len) <= 0) {
            break;
         }

         if (interpret_bytes(dir, bytes, len, &(decoder)) < 0) {
            break;
         }

         decoder.seq_number += 1;
      }
      ret = 0;
   }
   while (0);

   if (-1 != s.fd) {
      close(s.fd);
      s.fd = -1;
   }

   return (ret);
}

#endif // BCMPH_NOHW

static void parse_option(const char *option, const char **name, size_t *name_len, const char **value)
{
   const char *tmp;

   while (isspace(*option)) {
      option += 1;
   }
   *name = option;

   tmp = strchr(*name, '=');
   if (NULL != tmp) {
      *value = tmp + 1;
   }
   else {
      *value = NULL;
      tmp = *name;
      tmp += strlen(tmp);
   }
   *name_len = tmp - *name;
}

static int decode_codec(const char *value, bcm_phone_codec_t *codec)
{
   int ret = 0;

   do { /* Boucle vide */
      if (0 == strcasecmp(value, "alaw")) {
         *codec = BCMPH_CODEC_ALAW;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "ulaw")) {
         *codec = BCMPH_CODEC_ULAW;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "slin")) {
         *codec = BCMPH_CODEC_LINEAR;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "alaw16")) {
         *codec = BCMPH_CODEC_ALAW16;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "ulaw16")) {
         *codec = BCMPH_CODEC_ULAW16;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "slin16")) {
         *codec = BCMPH_CODEC_LINEAR16;
         ret = 1;
         break;
      }
   } while (0);

   return (ret);
}

static int decode_tone(const char *value, bcm_phone_line_tone_t *tone)
{
   int ret = 0;

   do { /* Boucle vide */
      if (0 == strcasecmp(value, "none")) {
         *tone = BCMPH_TONE_NONE;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "waiting_dial")) {
         *tone = BCMPH_TONE_WAITING_DIAL;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "invalid")) {
         *tone = BCMPH_TONE_INVALID;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "ringback")) {
         *tone = BCMPH_TONE_RINGBACK;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "busy")) {
         *tone = BCMPH_TONE_BUSY;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "disconnect")) {
         *tone = BCMPH_TONE_DISCONNECT;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "0")) {
         *tone = BCMPH_TONE_DTMF_0;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "1")) {
         *tone = BCMPH_TONE_DTMF_1;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "2")) {
         *tone = BCMPH_TONE_DTMF_2;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "3")) {
         *tone = BCMPH_TONE_DTMF_3;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "4")) {
         *tone = BCMPH_TONE_DTMF_4;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "5")) {
         *tone = BCMPH_TONE_DTMF_5;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "6")) {
         *tone = BCMPH_TONE_DTMF_6;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "8")) {
         *tone = BCMPH_TONE_DTMF_8;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "9")) {
         *tone = BCMPH_TONE_DTMF_9;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "a")) {
         *tone = BCMPH_TONE_DTMF_A;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "b")) {
         *tone = BCMPH_TONE_DTMF_B;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "c")) {
         *tone = BCMPH_TONE_DTMF_C;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "d")) {
         *tone = BCMPH_TONE_DTMF_D;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "*")) {
         *tone = BCMPH_TONE_DTMF_ASTER;
         ret = 1;
         break;
      }
      if (0 == strcasecmp(value, "#")) {
         *tone = BCMPH_TONE_DTMF_POUND;
         ret = 1;
         break;
      }
   } while (0);

   return (ret);
}

static inline int value_is_bool(const char *value)
{
   if ((('0' != value[0]) && ('1' != value[0])) || ('\0' != value[1])) {
      return (0);
   }
   else {
      return (1);
   }
}

static const char prm_name_test[] = "test";
static const char prm_name_use_16bits[] = "use_16bits";
static const char prm_name_line[] = "line";
static const char prm_name_codec[] = "codec";
static const char prm_name_default_line[] = "default_line";
static const char prm_name_output[] = "output";
static const char prm_name_input[] = "input";
static const char prm_name_tone[] = "tone";
static const char prm_name_on[] = "on";
static const char prm_name_off[] = "off";

static void print_usage(void)
{
   fprintf(stdout, "Usage : bcm63xx-phone-test arg1 arg2 arg3... where arguments are one of\n");
   fprintf(stdout, " test=loopback|loopback_mm|echo|tone, no default value : the test to make, the only mandatory argument\n");
   fprintf(stdout, " default_line=0|1 : the line to use/test\n");
   fprintf(stdout, " use_16bits=0|1, default value=0 : if PCM use 16 bits timeslot or 8 bits timeslot\n");
   fprintf(stdout, " line0=0|1, default value=1 : if line 0 is enabled or disabled\n");
   fprintf(stdout, " codec0=alaw|ulaw|slin|alaw16|ulaw16|slin16, default value=alaw : codec used for line 0\n");
   fprintf(stdout, " line1=0|1, default value=0 : if line 1 is enabled or disabled\n");
   fprintf(stdout, " codec1=alaw|ulaw|slin|alaw16|ulaw16|slin16, default value=alaw : codec used for line 1\n");
   fprintf(stdout, " output=path_to_file, no default value : in test echo the name of the file where to save data received from the driver\n");
   fprintf(stdout, " tone=none|waiting_dial|invalid|ringback|busy|disconnect|0|1|2|3|4|5|6|7|8|9|a|b|c|d|*|#, default value=waiting_dial : the tone to emit in test 'tone' but also in test 'echo' just after the user hook of the phone\n");
   fprintf(stdout, " on=[0-8191], default value=8191 : the time in msecs the tone must be on. If null and 'off' is not null the tone will be on for 'off' msecs and stops. If null and off is null too used default values of the driver\n");
   fprintf(stdout, " off=[0-8191], default value=0 : the time in msecs the tone must be off. If null and 'on' is not null, the tone will be played continuously\n");
}

int main(int argc, char *argv[])
{
   const char *which_test = NULL;
   int i;
   int pause = 0;

   prm_use_16bits = 0;
   memset(prm_lines, 0, sizeof(prm_lines));
   for (i = 0; (i < ((int)(ARRAY_SIZE(prm_lines)))); i += 1) {
      prm_lines[i].enable = 0;
      prm_lines[i].codec = BCMPH_CODEC_ALAW;
   }
   prm_lines[0].enable = 1;
   prm_default_line = 0;
   prm_output = NULL;

   if (argc > 1) {
      const char *name;
      size_t name_len;
      const char *value;
      for (i = 1; (i < argc); i += 1) {
         parse_option(argv[i], &(name), &(name_len), &(value));
         if ((NULL == value) || ('\0' == value[0])) {
            fprintf(stderr, "Arg '%s' is ignored because it has no value\n", argv[i]);
            pause = 1;
            continue;
         }
         if (((ARRAY_SIZE(prm_name_test) - 1) == name_len) && (0 == strncasecmp(name, prm_name_test, name_len))) {
            which_test = value;
            continue;
         }
         if (((ARRAY_SIZE(prm_name_use_16bits) - 1) == name_len) && (0 == strncasecmp(name, prm_name_use_16bits, name_len))) {
            if (value_is_bool(value)) {
               prm_use_16bits = atoi(value);
            }
            else {
               fprintf(stderr, "Value of arg '%s' must be '0' or '1'\n", argv[i]);
               pause = 1;
            }
            continue;
         }
         if (((ARRAY_SIZE(prm_name_tone) - 1) == name_len) && (0 == strncasecmp(name, prm_name_tone, name_len))) {
            bcm_phone_line_tone_t tone;
            if (decode_tone(value, &(tone))) {
               prm_tone = tone;
            }
            else {
               fprintf(stderr, "Value of arg '%s' is invalid\n", argv[i]);
               pause = 1;
            }
            continue;
         }
         if (((ARRAY_SIZE(prm_name_on) - 1) == name_len) && (0 == strncasecmp(name, prm_name_on, name_len))) {
            int tmp = atoi(value);
            if ((tmp >= 0) && (tmp < 8192)) {
               prm_tone_on_time = tmp;
            }
            else {
               fprintf(stderr, "Value of arg '%s' is invalid\n", argv[i]);
               pause = 1;
            }
            continue;
         }
         if (((ARRAY_SIZE(prm_name_off) - 1) == name_len) && (0 == strncasecmp(name, prm_name_off, name_len))) {
            int tmp = atoi(value);
            if ((tmp >= 0) && (tmp < 8192)) {
               prm_tone_off_time = tmp;
            }
            else {
               fprintf(stderr, "Value of arg '%s' is invalid\n", argv[i]);
               pause = 1;
            }
            continue;
         }
         if (((ARRAY_SIZE(prm_name_line) - 1) < name_len)
             && (0 == strncasecmp(name, prm_name_line, (ARRAY_SIZE(prm_name_line) - 1)))) {
            size_t j;
            for (j = (ARRAY_SIZE(prm_name_line) - 1); (j < name_len); j += 1) {
               if (!isdigit(name[j])) {
                  break;
               }
            }
            if (j >= name_len) {
               int tmp = atoi(&(name[ARRAY_SIZE(prm_name_line) - 1]));
               if ((tmp >= 0) && (tmp < ((int)(ARRAY_SIZE(prm_lines))))) {
                  if (value_is_bool(value)) {
                     prm_lines[tmp].enable = atoi(value);
                  }
                  else {
                     fprintf(stderr, "Value of arg '%s' must be '0' or '1'\n", argv[i]);
                     pause = 1;
                  }
                  continue;
               }
            }
         }
         if (((ARRAY_SIZE(prm_name_codec) - 1) < name_len)
             && (0 == strncasecmp(name, prm_name_codec, (ARRAY_SIZE(prm_name_codec) - 1)))) {
            size_t j;
            for (j = (ARRAY_SIZE(prm_name_codec) - 1); (j < name_len); j += 1) {
               if (!isdigit(name[j])) {
                  break;
               }
            }
            if (j >= name_len) {
               int tmp = atoi(&(name[ARRAY_SIZE(prm_name_codec) - 1]));
               if ((tmp >= 0) && (tmp < ((int)(ARRAY_SIZE(prm_lines))))) {
                  bcm_phone_codec_t codec;
                  if (decode_codec(value, &(codec))) {
                     prm_lines[tmp].codec = codec;
                  }
                  else {
                     fprintf(stderr, "Value of arg '%s' is invalid\n", argv[i]);
                     pause = 1;
                  }
                  continue;
               }
            }
         }
         if (((ARRAY_SIZE(prm_name_default_line) - 1) == name_len) && (0 == strncasecmp(name, prm_name_default_line, name_len))) {
            int tmp = atoi(value);
            if ((tmp >= 0) && (tmp < ((int)(ARRAY_SIZE(prm_lines))))) {
               prm_default_line = tmp;
            }
            else {
               fprintf(stderr, "Value of arg '%s' must be >= 0 and < %lu\n", argv[i], (unsigned long)(ARRAY_SIZE(prm_lines)));
               pause = 1;
            }
            continue;
         }
         if (((ARRAY_SIZE(prm_name_output) - 1) == name_len) && (0 == strncasecmp(name, prm_name_output, name_len))) {
            prm_output = value;
            continue;
         }
#ifdef BCMPH_NOHW
         if (((ARRAY_SIZE(prm_name_input) - 1) == name_len) && (0 == strncasecmp(name, prm_name_input, name_len))) {
            prm_input = value;
            continue;
         }
#endif // BCMPH_NOHW
         fprintf(stderr, "Arg '%s' is ignored because it's unknown\n", argv[i]);
         pause = 1;
      }
   }

   if (!prm_lines[prm_default_line].enable) {
      fprintf(stderr, "Default line %lu is disabled\n", (unsigned long)(prm_default_line));
      prm_default_line = 0;
      pause = 1;
   }
   else {
      for (i = 0; (i < ((int)(ARRAY_SIZE(prm_lines)))); i += 1) {
         if (prm_lines[i].enable) {
            break;
         }
         else if (i == ((int)(prm_default_line))) {
            i += 1;
         }
      }
      if (i >= ((int)(ARRAY_SIZE(prm_lines)))) {
         fprintf(stderr, "No lines enabled\n");
         pause = 1;
      }
   }

   if (pause) {
      fprintf(stdout, "Press a key to continue\n");
      fgetc(stdin);
      if (!prm_lines[prm_default_line].enable) {
         return (-1);
      }
   }

   if (NULL != which_test) {
      if (0 == strcasecmp(which_test, "loopback")) {
         return (test_loopback());
      }
      else if (0 == strcasecmp(which_test, "loopback_mm")) {
         return (test_loopback_mm());
      }
      else if (0 == strcasecmp(which_test, "echo")) {
         return (test_echo());
      }
      else if (0 == strcasecmp(which_test, "tone")) {
         return (test_tone());
      }
#ifdef BCMPH_NOHW
      else if (0 == strcasecmp(which_test, "decode_mpi")) {
         return (decode_mpi_data());
      }
#endif // BCMPH_NOHW
      else {
         fprintf(stderr, "Don't know what to do\n");
         print_usage();
      }
   }
   else {
      print_usage();
   }

   return (-1);
}
