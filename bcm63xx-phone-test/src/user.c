#include <config.h>

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
#include <sys/mman.h>
#include <linux/types.h>
#include <signal.h>

#include <main.h>

#include <bcm63xx_phone.h>
#include <bcm63xx_ring_buf.h>

/*
valgrind --version
valgrind  --leak-check=full --track-origins=yes --xml=yes "/usr/local/src/openwrt/git/12.09/../14.07/custom-feed/bcm63xx-phone-test/bin/Debug/bcm63xx-phone-user"
*/

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

static int read_regs(struct file *filp)
{
   int ret;
   bcm_phone_pcm_regs_t regs;

   memset(&(regs), 0, sizeof(regs));

   ret = bcm_drv_unlocked_ioctl(filp, BCMPH_IOCTL_READ_PCM_REGS, (unsigned long)(&(regs)));
   if (!ret) {
      print_regs(&(regs));
   }
   else {
      fprintf(stderr, "Error reading PCM registers.\n");
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
   fprintf(stdout, "rx_sop             => %lu\n", (unsigned long)(stats->rx_sop));
   fprintf(stdout, "rx_eop             => %lu\n", (unsigned long)(stats->rx_eop));
   fprintf(stdout, "min_size_rx_buffer => %lu\n", (unsigned long)(stats->min_size_rx_buffer));
   fprintf(stdout, "max_size_rx_buffer => %lu\n", (unsigned long)(stats->max_size_rx_buffer));
   fprintf(stdout, "min_size_tx_buffer => %lu\n", (unsigned long)(stats->min_size_tx_buffer));
   fprintf(stdout, "max_size_tx_buffer => %lu\n", (unsigned long)(stats->max_size_tx_buffer));
}

static int read_stats(struct file *filp)
{
   int ret;
   bcm_phone_pcm_stats_t stats;

   memset(&(stats), 0, sizeof(stats));

   ret = bcm_drv_unlocked_ioctl(filp, BCMPH_IOCTL_READ_PCM_STATS, (unsigned long)(&(stats)));
   if (!ret) {
      print_stats(&(stats));
   }
   else {
      fprintf(stderr, "Error reading PCM stats.\n");
   }

   return (ret);
}

static void init_phone_cfg(bcm_phone_cfg_params_t *prms)
{
   memset(prms, 0, sizeof(*prms));
   prms->country = BCMPH_COUNTRY_FR;
   prms->pcm_use_16bits_timeslot = 0;
   prms->line_params[0].enable = 1;
   prms->line_params[0].codec = BCMPH_CODEC_LINEAR16;
   prms->line_params[1].enable = 1;
   prms->line_params[1].codec = BCMPH_CODEC_LINEAR16;
}

static int start(struct file *filp)
{
   int ret;
   bcm_phone_cfg_params_t prms;

   init_phone_cfg(&(prms));

   ret = bcm_drv_unlocked_ioctl(filp, BCMPH_IOCTL_START, (unsigned long)(&(prms)));

   if (ret) {
      fprintf(stderr, "Error starting.\n");
   }

   return (ret);
}

static int stop(struct file *filp)
{
   int ret;

   ret = bcm_drv_unlocked_ioctl(filp, BCMPH_IOCTL_STOP, 0);

   if (!ret) {
      read_regs(filp);
   }
   else {
      fprintf(stderr, "Error stopping.\n");
   }

   return (ret);
}

static int start_pcm(struct file *filp)
{
   int ret;

   ret = bcm_drv_unlocked_ioctl(filp, BCMPH_IOCTL_START_PCM, 0);

   if (ret) {
      fprintf(stderr, "Error starting PCM.\n");
   }

   return (ret);
}

static int stop_pcm(struct file *filp)
{
   int ret;

   ret = bcm_drv_unlocked_ioctl(filp, BCMPH_IOCTL_STOP_PCM, 0);

   if (ret) {
      fprintf(stderr, "Error stopping PCM.\n");
   }

   return (ret);
}

static int set_line_mode(struct file *filp, size_t line, bcm_phone_line_mode_t mode)
{
   int ret;
   bcm_phone_set_line_mode_t slm;

   memset(&(slm), 0, sizeof(slm));
   slm.line = line;
   slm.mode = mode;
   slm.tone = BCMPH_TONE_UNSPECIFIED;
   slm.wait = -1;

   ret = bcm_drv_unlocked_ioctl(filp, BCMPH_IOCTL_SET_LINE_MODE, (unsigned long)(&(slm)));

   if (ret) {
      fprintf(stderr, "Error setting line mode.\n");
   }

   return (ret);
}

static __u8 buf_tx[256];
static __u8 buf_rx[24576 / 4];

static int test_loopback(void)
{
   int ret = 0;

   ret = bcm_drv_init();

   if (!ret) {
      struct file fil;
      fil.private_data = NULL;

      do { // Empty loop
         size_t todo;
         size_t delay;
         size_t total_done_tx;
         size_t done_tx;
         size_t done_rx;
         ssize_t len;

         // Open device
         ret = bcm_drv_open(NULL, &(fil));
         if (ret) {
            fprintf(stderr, "Error opening device.\n");
            break;
         }
         fil.f_flags = O_RDWR | O_NONBLOCK;

         read_regs(&(fil));

         for (done_tx = 0; (done_tx < sizeof(buf_tx)); done_tx += 1) {
            buf_tx[done_tx] = (__u8)(done_tx);
         }
         memset(buf_rx, 0, sizeof(buf_rx));

         // We configure
         ret = start(&(fil));
         if (ret) {
            break;
         }

         // We fill TX buffers before starting PCM
         total_done_tx = 0;
         done_tx = 0;
         todo = sizeof(buf_tx) - done_tx;
         len = bcm_drv_write(&(fil), (const char *)(&(buf_tx[done_tx])), todo, NULL);
         if (len < 0) {
            if (-EAGAIN != len) {
               fprintf(stderr, "Error writing to the device.\n");
               break;
            }
         }
         else {
            total_done_tx += len;
            done_tx += len;
         }

         // Then we start PCM
         ret = start_pcm(&(fil));
         if (ret) {
            break;
         }

         ret = set_line_mode(&(fil), 0, BCMPH_MODE_OFF_TALKING);
         if (ret) {
            break;
         }

         done_rx = 0;
         delay = 0;
         while (done_rx < sizeof(buf_rx)) {
            int pause = 1;
            todo = sizeof(buf_rx) - done_rx;
            len = bcm_drv_read(&(fil), (char *)(&(buf_rx[done_rx])), todo, NULL);
            if (len < 0) {
               if (-EAGAIN != len) {
                  fprintf(stderr, "Error reading from the device.\n");
                  break;
               }
            }
            else if (len > 0) {
               done_rx += len;
               pause = 0;
            }

            if (done_tx >= sizeof(buf_tx)) {
               done_tx = 0;
            }
            todo = sizeof(buf_tx) - done_tx;
            len = bcm_drv_write(&(fil), (const char *)(&(buf_tx[done_tx])), todo, NULL);
            if (len < 0) {
               if (-EAGAIN != len) {
                  fprintf(stderr, "Error writing to the device.\n");
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
               usleep(10000);
               if (delay > 100) {
                  delay = 0;
                  fprintf(stdout, "%lu bytes written and %lu bytes read...\n", (unsigned long)(total_done_tx), (unsigned long)(done_rx));
               }
            }
         }
         fprintf(stdout, "%ld bytes written to the device\n", (long)(total_done_tx));

         sleep(3);

         // We stop PCM
         stop_pcm(&(fil));

         // We read remaining bytes
         todo = sizeof(buf_rx) - done_rx;
         len = bcm_drv_read(&(fil), (char *)(&(buf_rx[done_rx])), todo, NULL);
         if (len < 0) {
            if (-EAGAIN != len) {
               fprintf(stderr, "Error reading from the device.\n");
               break;
            }
         }
         done_rx += len;

         // We stop
         stop(&(fil));

         // Dump the buffer received
         dump_buf("Bytes received", buf_rx, done_rx);

         read_stats(&(fil));

         for (todo = 0; ((todo < done_tx) && (todo < done_rx)); todo += 1) {
            if (buf_tx[todo] != buf_rx[todo]) {
               break;
            }
         }
         if ((todo >= done_tx) || (todo >= done_rx)) {
            fprintf(stdout, "Bytes received are the same as those sent.\n");
         }
         else {
            fprintf(stdout, "Byte %lu received differ from the one sent : %x vs %x.\n", (unsigned long)(todo), (unsigned int)(buf_rx[todo]), (unsigned int)(buf_tx[todo]));
         }
      } while (0);

      if (NULL != fil.private_data) {
         bcm_drv_release(NULL, &(fil));
         fil.private_data = NULL;
      }

      bcm_drv_exit();
   }

   return (ret);
}

int main(int argc, char *argv[])
{
   return (test_loopback());
}

