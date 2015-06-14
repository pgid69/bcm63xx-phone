/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#include <config.h>

#ifdef __KERNEL__
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#ifndef BCMPH_NOHW
#include <bcm63xx_board.h>
#include <bcm63xx_cpu.h>
#include <bcm63xx_io.h>
#include <bcm63xx_irq.h>
#include <bcm63xx_nvram.h>
#endif // !BCMPH_NOHW
#endif // __KERNEL__

#include <bcm63xx.h>
#include <board.h>
#include <bcm63xx_log.h>
#include <pcm.h>

//#include <phone/zarlink/zarlinkCommon.h>
#include <vp_api.h>
#include <vp_api_int.h>
#include <vp880_api_int.h>
#include "../phone/zarlink/zarlink_common.h"
#include "../phone/zarlink/profiles/profiles_common.h"
#include "../phone/zarlink/profiles/VE880_ABS100V_LITE_NB_PCM_Rev2_8.h"
#include "../phone/zarlink/profiles/VE880_ABS100V_LITE_WB_PCM_Rev2_8.h"

// Include after system files
#include <compile.h>

const char *driver_name = "bcm63xx-phone";

void board_set_gpio(int gpio_num, int state)
{
   d_bcm_pr_debug("board_set_gpio(gpio_num=0x%x, state=%d)\n", (int)(gpio_num), (int)(state));
   if (GPIO_IS_INVALID != gpio_num) {
#ifndef BCMPH_NOHW
      u32 val;
#endif // !BCMPH_NOHW
      if (gpio_num & GPIO_IS_ACTIVE_LOW) {
         if (state) {
            state = 0;
         }
         else {
            state = 1;
         }
      }
      gpio_num &= (~(GPIO_IS_ACTIVE_LOW));
      bcm_assert(gpio_num <= 31);
#ifndef BCMPH_NOHW
      val = bcm_gpio_readl(GPIO_CTL_LO_REG);
      val |= (1 << gpio_num);
      bcm_gpio_writel(val, GPIO_CTL_LO_REG);
      msleep(1);
      val = bcm_gpio_readl(GPIO_DATA_LO_REG);
      if (state) {
         val |= (1 << gpio_num);
      }
      else {
         val &= (~(1 << gpio_num));
      }
      bcm_gpio_writel(val, GPIO_DATA_LO_REG);
#endif // !BCMPH_NOHW
   }
}

#ifndef BCMPH_NOHW
// The definition of struct clk comes from file arch/mips/bcm63xx/clk.c
struct clk {
	void		(*set)(struct clk *, int);
	unsigned int	rate;
	unsigned int	usage;
	int		id;
};

/*
 * PCM clock
 */
static void pcm6368_clk_set(struct clk *clk, int enable)
{
   u32 reg = bcm_perf_readl(PERF_CKCTL_REG);

   bcm_pr_debug("pcm6368_clk_set(enable=%d)\n", (int)(enable));

   if (enable) {
      reg |= CKCTL_6368_PCM_EN;
   }
   else {
      reg &= (~(CKCTL_6368_PCM_EN));
   }
   bcm_perf_writel(reg, PERF_CKCTL_REG);
}

// TODO to move in arch/mips/bcm63xx/clk.c
static struct clk pcm6368_clk = {
	.set	= pcm6368_clk_set,
};

static void pcm6362_clk_set(struct clk *clk, int enable)
{
   u32 reg = bcm_perf_readl(PERF_CKCTL_REG);

   bcm_pr_debug("pcm6362_clk_set(enable=%d)\n", (int)(enable));

   if (enable) {
      reg |= CKCTL_6362_PCM_EN;
   }
   else {
      reg &= (~(CKCTL_6362_PCM_EN));
   }
   bcm_perf_writel(reg, PERF_CKCTL_REG);
}

// TODO to move in arch/mips/bcm63xx/clk.c
static struct clk pcm6362_clk = {
	.set	= pcm6362_clk_set,
};

static pcm_desc_t pcm6358_desc = {
   .clk = NULL, // Initialized at runtime
   .softreset_mask = SOFTRESET_6358_PCM_MASK,
   .base = BCM_6358_PCM_BASE,
   .dma_base = BCM_6358_PCMDMA_BASE,
   .dmac_base = BCM_6358_PCMDMAC_BASE,
   .dmas_base = BCM_6358_PCMDMAS_BASE,
   .irq = BCM_6358_PCM_IRQ,
   .irq_dma_rx = BCM_6358_PCM_DMA0_IRQ,
   .irq_dma_tx = BCM_6358_PCM_DMA1_IRQ,
   .dma_chan_count = 16,
   .pll_init = pcm6358_pll_init,
   .pll_deinit = pcm6358_pll_deinit,
};

static pcm_desc_t pcm6368_desc = {
   .clk = &(pcm6368_clk),
   .softreset_mask = SOFTRESET_6368_PCM_MASK,
   .base = BCM_6368_PCM_BASE,
   .dma_base = BCM_6368_PCMDMA_BASE,
   .dmac_base = BCM_6368_PCMDMAC_BASE,
   .dmas_base = BCM_6368_PCMDMAS_BASE,
   .irq = BCM_6368_PCM_IRQ,
   .irq_dma_rx = BCM_6368_PCM_DMA0_IRQ,
   .irq_dma_tx = BCM_6368_PCM_DMA1_IRQ,
   .dma_chan_count = 32,
   .pll_init = pcm6368_pll_init,
   .pll_deinit = pcm6368_pll_deinit,
};

static pcm_desc_t pcm6362_desc = {
   .clk = &(pcm6362_clk),
   .softreset_mask = SOFTRESET_6362_PCM_MASK,
   .base = BCM_6362_PCM_BASE,
   .dma_base = BCM_6362_PCMDMA_BASE,
   .dmac_base = BCM_6362_PCMDMAC_BASE,
   .dmas_base = BCM_6362_PCMDMAS_BASE,
   .irq = BCM_6362_PCM_IRQ,
   .irq_dma_rx = BCM_6362_PCM_DMA0_IRQ,
   .irq_dma_tx = BCM_6362_PCM_DMA1_IRQ,
   .dma_chan_count = 32,
   .pll_init = NULL, // TODO
   .pll_deinit = NULL, // TODO
};

static cpu_desc_t cpu6358_desc = {
   .cpu_id = BCM6358_CPU_ID,
   .dcache_line_size = 16, /* Must be a power of 2.
      Value comes from http://wiki.openwrt.org/doc/hardware/soc/soc.broadcom.bcm63xx */
   .off_perf_softreset_reg = PERF_SOFTRESET_6358_REG,
};

static cpu_desc_t cpu6368_desc = {
   .cpu_id = BCM6368_CPU_ID,
   .dcache_line_size = 16, /* Must be a power of 2.
      Value comes from http://wiki.openwrt.org/doc/hardware/soc/soc.broadcom.bcm63xx */
   .off_perf_softreset_reg = PERF_SOFTRESET_6368_REG,
};

static cpu_desc_t cpu6362_desc = {
   .cpu_id = BCM6362_CPU_ID,
   .dcache_line_size = 16, /* Must be a power of 2.
      Value comes from http://wiki.openwrt.org/doc/hardware/soc/soc.broadcom.bcm63xx */
   .off_perf_softreset_reg = PERF_SOFTRESET_6362_REG,
};

static board_desc_t board6358_desc = {
   .name = NULL, // Initialized at runtime
   .cpu_desc = &(cpu6358_desc),
   .pcm_desc = &(pcm6358_desc),
   .phone_desc = NULL, // Initialized at runtime
};

static board_desc_t board6368_desc = {
   .name = NULL, // Initialized at runtime
   .cpu_desc = &(cpu6368_desc),
   .pcm_desc = &(pcm6368_desc),
   .phone_desc = NULL, // Initialized at runtime
};

static board_desc_t board6362_desc = {
   .name = NULL, // Initialized at runtime
   .cpu_desc = &(cpu6362_desc),
   .pcm_desc = &(pcm6362_desc),
   .phone_desc = NULL, // Initialized at runtime
};
#endif // !BCMPH_NOHW

/*
 We don't use #define for defining missing profile
 but instead define variable so compiler/linker can warn of multiple
 definitions
*/
static const VpProfileDataType AC_FXS_RF14_CA[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType AC_FXS_RF14_HK[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType AC_FXS_RF14_KR[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType AC_FXS_RF14_SG[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType AC_FXS_RF14_TW[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType AC_FXS_RF14_US[] = { 0 , VP_PRFWZ_PROFILE_NONE };

static const VpProfileDataType AC_FXS_RF14_WB_CA[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType AC_FXS_RF14_WB_HK[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType AC_FXS_RF14_WB_KR[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType AC_FXS_RF14_WB_SG[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType AC_FXS_RF14_WB_TW[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType AC_FXS_RF14_WB_US[] = { 0 , VP_PRFWZ_PROFILE_NONE };

static const VpProfileDataType RING_VE880_ABS100V_ETSI[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_GR57[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_AU[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_BE[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_BG[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_BR[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_CH[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_CN[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_DE[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_DK[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_ES[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_GB[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_GR[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_HU[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_IE[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_IL[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_IS[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_IT[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_NL[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_NO[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_NZ[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_PT[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_RU[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_SE[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_TK[] = { 0 , VP_PRFWZ_PROFILE_NONE };
static const VpProfileDataType RING_VE880_ABS100V_ZA[] = { 0 , VP_PRFWZ_PROFILE_NONE };

static zarlink_device_parameters_t hw553_le88221_dev_params = {
   .type = VP_DEV_880_SERIES,
   .profiles = {
      .dev_profile = DEV_PROFILE_VE880_ABS100V_PCM,
      .default_AC_profile_NB = AC_FXS_RF14_600R_DEF,
      .AC_profiles_NB = {
#undef COUNTRY_ARCHIVE_MAKE_NAME
#define COUNTRY_ARCHIVE_MAKE_NAME(country) AC_FXS_RF14_##country,
#include <countryArchive.h>
      },
      .default_AC_profile_WB = NULL,
      .AC_profiles_WB = {
#undef COUNTRY_ARCHIVE_MAKE_NAME
#define COUNTRY_ARCHIVE_MAKE_NAME(country) NULL,
#include <countryArchive.h>
      },
      .default_DC_profile = DC_FXS_VE880_ABS100V_DEF,
      .DC_profiles = {
#undef COUNTRY_ARCHIVE_MAKE_NAME
#define COUNTRY_ARCHIVE_MAKE_NAME(country) NULL,
#include <countryArchive.h>
      },
      .default_ring_profile = RING_25HZ_VE880_ABS100V_DEF,
      .ring_profiles= {
#undef COUNTRY_ARCHIVE_MAKE_NAME
#define COUNTRY_ARCHIVE_MAKE_NAME(country) RING_VE880_ABS100V_##country,
#include <countryArchive.h>
      },
   },
   .modes = {
      .on_hook_idle = VP_LINE_STANDBY,
      .on_hook_ringing = VP_LINE_RINGING,
      .ring_cadence = {
         .on_time = 2000,
         .off_time = 4000,
      },
      .off_hook_idle = VP_LINE_ACTIVE,
      .off_hook_talking = VP_LINE_TALK,
   },
   .tones = {
      .waiting_dial = {
         .profile = TONE_DIAL,
         .on_time = 0x8000,
         .off_time = 0,
      },
      .invalid = {
         .profile = TONE_BUSY,
         .on_time = 250,
         .off_time = 200,
      },
      .ringback = {
         .profile = TONE_RINGBACK,
         .on_time = 2000,
         .off_time = 4000,
      },
      .busy = {
         .profile = TONE_BUSY,
         .on_time = 500,
         .off_time = 500,
      },
      .disconnect = {
         .profile = TONE_ROH,
         .on_time = 100,
         .off_time = 100,
      },
   }
};

/*
 On HW553, FXS port labelled phone 1 is the second line of le88221 (index 1),
 and FXS port labelled phone 2 is the first line of le88221 (index 0)
*/
static zarlink_line_parameters_t hw553_le88221_line0_params = {
   .id = 1,
   .type = VP_TERM_FXS_GENERIC,
};

static zarlink_line_parameters_t hw553_le88221_line1_params = {
   .id = 0,
   .type = VP_TERM_FXS_GENERIC,
};

static phone_desc_t hw553_phone_desc = {
   .pcm_ctrl_reg = (PCM_CLOCK_INV | PCM_FS_TRIG | PCM_DATA_OFF),
   .clk_rate = 2048, // Clock rate must be the same as the one coded in zarlink profiles
   .tick_period = 10, // in msecs, must be coherent with zarlink profiles
   .device_count = 1,
   .devices = {
      {
         .type = BCMPH_VD_ZARLINK_88221,
         .caps = BCMPH_CAPS_REQUIRES_RESET
            | BCMPH_CAPS_ALAW_CODEC | BCMPH_CAPS_ULAW_CODEC
            | BCMPH_CAPS_LINEAR_CODEC,
         .reset_gpio = 24 | GPIO_IS_ACTIVE_LOW,
         .mpi_params = {
#ifdef BCMPH_USE_SPI_DRIVER
            .bus_num = 0,
#endif // BCMPH_USE_SPI_DRIVER
            .cs = 2,
            /* Le88221 supports a MPI CLK period of 122 ns min (8.196721 MHz)
             the closest freq supported by SPI controller is 6.25 MHz.
             But we must have a CS off time of 2500 ns min which means 15.62 CLK period
             As SPI controler can be configured with a value between 0 and 7,
             the frequency can be at most 1,563 MHz (because at this frequency, 2500 ns
             is 3.9075 CLK period)
            */
            .clk = 1563000,
            .toggle_cs = true,
#ifndef BCMPH_USE_SPI_DRIVER
            /* Le88221 requires a CS off time of 2500 ns min between each byte :
             if CLK is 1.563 MHz it means 4 periods */
            .cs_off_time = 4,
            .wait_completion_with_irq = false,
            .fill_byte = 0x06, /* NOP operation for Le88221 */
         },
#endif // !BCMPH_USE_SPI_DRIVER
         .line_count = 2,
         .lines = {
            {
               .type = BCMPH_LIN_FXS,
               .first_timeslot = 0,
               .parameters = {
                  .zarlink = &(hw553_le88221_line0_params),
               },
            },
            {
               .type = BCMPH_LIN_FXS,
               .first_timeslot = 2,
               .parameters = {
                  .zarlink = &(hw553_le88221_line1_params),
               },
            },
         },
         .parameters = {
            .zarlink = &(hw553_le88221_dev_params),
         }
      },
   },
};

static zarlink_device_parameters_t hw556_le88266_dev_params = {
   .type = VP_DEV_880_SERIES,
   .profiles = {
      .dev_profile = DEV_PROFILE_VE880_ABS100V_PCM,
      .default_AC_profile_NB = AC_FXS_RF14_600R_DEF,
      .AC_profiles_NB = {
#undef COUNTRY_ARCHIVE_MAKE_NAME
#define COUNTRY_ARCHIVE_MAKE_NAME(country) AC_FXS_RF14_##country,
#include <countryArchive.h>
      },
      .default_AC_profile_WB = AC_FXS_RF14_WB_600R_DEF,
      .AC_profiles_WB = {
#undef COUNTRY_ARCHIVE_MAKE_NAME
#define COUNTRY_ARCHIVE_MAKE_NAME(country) AC_FXS_RF14_WB_##country,
#include <countryArchive.h>
      },
      .default_DC_profile = DC_FXS_VE880_ABS100V_DEF,
      .DC_profiles = {
#undef COUNTRY_ARCHIVE_MAKE_NAME
#define COUNTRY_ARCHIVE_MAKE_NAME(country) NULL,
#include <countryArchive.h>
      },
      .default_ring_profile = RING_25HZ_VE880_ABS100V_DEF,
      .ring_profiles= {
#undef COUNTRY_ARCHIVE_MAKE_NAME
#define COUNTRY_ARCHIVE_MAKE_NAME(country) RING_VE880_ABS100V_##country,
#include <countryArchive.h>
      },
   },
   .modes = {
      .on_hook_idle = VP_LINE_STANDBY,
      .on_hook_ringing = VP_LINE_RINGING,
      .ring_cadence = {
         .on_time = 2000,
         .off_time = 4000,
      },
      .off_hook_idle = VP_LINE_ACTIVE,
      .off_hook_talking = VP_LINE_TALK,
   },
   .tones = {
      .waiting_dial = {
         .profile = TONE_DIAL,
         .on_time = 0x8000,
         .off_time = 0,
      },
      .invalid = {
         .profile = TONE_BUSY,
         .on_time = 250,
         .off_time = 200,
      },
      .ringback = {
         .profile = TONE_RINGBACK,
         .on_time = 2000,
         .off_time = 4000,
      },
      .busy = {
         .profile = TONE_BUSY,
         .on_time = 500,
         .off_time = 500,
      },
      .disconnect = {
         .profile = TONE_ROH,
         .on_time = 100,
         .off_time = 100,
      },
   }
};

static zarlink_line_parameters_t hw556_le88266_line0_params = {
   .id = 0,
   .type = VP_TERM_FXS_GENERIC,
};

static zarlink_line_parameters_t hw556_le88266_line1_params = {
   .id = 1,
   .type = VP_TERM_FXS_GENERIC,
};

static phone_desc_t hw556_phone_desc = {
   .pcm_ctrl_reg = (PCM_CLOCK_INV | PCM_FS_TRIG | PCM_DATA_OFF),
   .clk_rate = 2048, // Clock rate must be the same as the one coded in zarlink profiles
   .tick_period = 10, // in msecs, must be coherent with zarlink profiles
   .device_count = 1,
   .devices = {
      {
         .type = BCMPH_VD_ZARLINK_88266,
         .caps = BCMPH_CAPS_REQUIRES_RESET
            | BCMPH_CAPS_ALAW_CODEC | BCMPH_CAPS_ULAW_CODEC
            | BCMPH_CAPS_LINEAR_CODEC | BCMPH_CAPS_LINEAR16_CODEC
            | BCMPH_CAPS_ALAW16_CODEC | BCMPH_CAPS_ULAW16_CODEC,
         .reset_gpio = 24 | GPIO_IS_ACTIVE_LOW,
         .mpi_params = {
#ifdef BCMPH_USE_SPI_DRIVER
            .bus_num = 0,
#endif // BCMPH_USE_SPI_DRIVER
            .cs = 2,
            /* Le88266 supports a MPI CLK period of 122 ns min (8.196721 MHz)
             the closest freq supported by SPI controller is 6.25 MHz.
             But we must have a CS off time of 2500 ns min which means 15.62 CLK period
             As SPI controler can be configured with a value between 0 and 7,
             the frequency can be at most 1,563 MHz (because at this frequency, 2500 ns
             is 3.9075 CLK period)
            */
            .clk = 1563000,
            .toggle_cs = true,
#ifndef BCMPH_USE_SPI_DRIVER
            /* Le88266 requires a CS off time of 2500 ns min between each byte :
             if CLK is 1.563 MHz it means 4 periods */
            .cs_off_time = 4,
            .wait_completion_with_irq = false,
            .fill_byte = 0x06, /* NOP operation for Le88266 */
         },
#endif // !BCMPH_USE_SPI_DRIVER
         .line_count = 2,
         .lines = {
            {
               .type = BCMPH_LIN_FXS,
               .first_timeslot = 0,
               .parameters = {
                  .zarlink = &(hw556_le88266_line0_params),
               },
            },
            {
               .type = BCMPH_LIN_FXS,
               .first_timeslot = 2,
               .parameters = {
                  .zarlink = &(hw556_le88266_line1_params),
               },
            },
         },
         .parameters = {
            .zarlink = &(hw556_le88266_dev_params),
         }
      },
   },
};

#ifdef BCMPH_NOHW
static cpu_desc_t cpu_generic_desc = {
   .cpu_id = 0,
   .dcache_line_size = 16, // Must be a power of 2.
};

static pcm_desc_t pcm_generic_desc = {
   .pll_init = pcm6358_pll_init,
   .pll_deinit = pcm6358_pll_deinit,
};

static board_desc_t board_generic_desc = {
   .name = "generic",
   .cpu_desc = &(cpu_generic_desc),
   .pcm_desc = &(pcm_generic_desc),
   .phone_desc = &(hw556_phone_desc),
};
#endif // BCMPH_NOHW

static const board_desc_t *board_desc = NULL; // Filled at runtime

int __init board_init(void)
{
   int ret = 0;

   bcm_pr_debug("board_init()\n");

#ifndef BCMPH_NOHW
   if (BCMCPU_IS_6358()) {
      bcm_pr_info("CPU is Broadcom 6358\n");
      pcm6358_desc.clk = clk_get(NULL, "pcm");
      if (IS_ERR(pcm6358_desc.clk)) {
         bcm_pr_err("Unable to get pcm clock\n");
         ret = PTR_ERR(pcm6358_desc.clk);
         goto fail_clk;
      }
      else {
         board6358_desc.name = bcm63xx_nvram_get_name();
         bcm_pr_info("Board is %s\n", board6358_desc.name);
         if (0 == strcmp(board6358_desc.name, "HW553")) {
            board6358_desc.phone_desc = &(hw553_phone_desc);
            board_desc = &(board6358_desc);
         }
         else if ((0 == strcmp(board6358_desc.name, "HW556"))
                  || (0 == strcmp(board6358_desc.name, "HW556_A"))
                  || (0 == strcmp(board6358_desc.name, "HW556_B"))
                  || (0 == strcmp(board6358_desc.name, "HW556_C"))) {
            board6358_desc.phone_desc = &(hw556_phone_desc);
            board_desc = &(board6358_desc);
         }
         else {
            bcm_pr_err("Unsupported board %s\n", board6358_desc.name);
            ret = -ENODEV;
            goto fail_board;
         }
      }
   } else {
      bcm_pr_err("Unsupported CPU\n");
      ret = -ENODEV;
      goto fail_cpu;
   }

   return (ret);

fail_board:
   // Free the clock
   clk_put(pcm6358_desc.clk);
   pcm6358_desc.clk = NULL;
fail_clk:
fail_cpu:
#else // BCMPH_NOHW
   board_desc = &(board_generic_desc);
   bcm_pr_info("Board is %s\n", board_desc->name);
#endif // BCMPH_NOHW
   return (ret);
}

const board_desc_t *board_get_desc(void)
{
   bcm_pr_debug("board_get_desc()\n");
   return (board_desc);
}

void board_deinit(void)
{
   // Nothing to do right now
   bcm_pr_debug("board_deinit()\n");

#ifndef BCMPH_NOHW
   // Free the clock
   clk_put(pcm6358_desc.clk);
   pcm6358_desc.clk = NULL;
#endif // !BCMPH_NOHW
}
