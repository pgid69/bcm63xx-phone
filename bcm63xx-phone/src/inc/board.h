/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifndef __BOARD_H__
#define __BOARD_H__

#include "config.h"

#ifdef __KERNEL__
#include <linux/clk.h>
#else // !__KERNEL__
#include "fake_kernel.h"
#endif // !__KERNEL__

#include "bcm63xx_phone.h"

extern const char *driver_name;

typedef struct {
   int cpu_id;
   size_t dcache_line_size; // Size in bytes of a line of the cache (must be a power of 2)
#ifndef BCMPH_NOHW
   __u32 off_perf_softreset_reg; // Offset in PERF memory area of register SOFTRESET
#endif // !BCMPH_NOHW
} cpu_desc_t;

struct pcm;

typedef struct {
#ifndef BCMPH_NOHW
   struct clk *clk; // Clock associated with PCM module
   __u32 softreset_mask; // Mask to perform a soft reset of PCM module
   unsigned long base; // Base address of memory area used to access PCM registers
   unsigned long dma_base; // Base address of memory area used to access PCM DMA registers
   unsigned long dmac_base; // Base address of memory area used to access PCM DMA channels registers
   unsigned long dmas_base; // Base address of memory area used to access PCM DMA state ram registers
   int irq;
   int irq_dma_rx;
   int irq_dma_tx;
   int dma_chan_count; // Number of PCM DMA channels
#endif // !BCMPH_NOHW
   int (*pll_init)(const struct pcm *t); // Function to initialize PCM PLL
   void (*pll_deinit)(const struct pcm *t); // Function to de initialize PCM PLL
} pcm_desc_t;

typedef enum {
	BCMPH_LIN_NONE,
	BCMPH_LIN_FXS,
	BCMPH_LIN_FXO
} phone_line_type_t;

typedef struct {
   phone_line_type_t type;
   /*
    Receive/Sending timeslot for the line (considering timeslot is 8 bits wide)
    It must be in the interval [0, 127[ and must be even
    One line can use up to 4 timeslots
    if codec is linear16 and PCM set in 8 bits mode, but
    only the first one can be specified
    NB : even if PCM is in 16 bits mode, timeslot is always the offset
    from the start of the frame expressed in byte (8 bits wide)
   */
   __u8 first_timeslot;
   union {
      const struct zarlink_line_parameters *zarlink;
   } parameters;
} phone_desc_line_t;


/* Maximum numbers of device */
#define BCMPH_MAX_DEVICES 1
/* Maximum numbers of channels per device */
#define BCMPH_MAX_LINES_PER_DEV 2

#if ((BCMPH_MAX_DEVICES * BCMPH_MAX_LINES_PER_DEV) > BCMPH_MAX_LINES)
#error "BCMPH_MAX_LINES must be >= than (BCMPH_MAX_DEVICES * BCMPH_MAX_LINES_PER_DEV)"
#endif

typedef enum {
   BCMPH_VD_NONE = -1,
   BCMPH_VD_ZARLINK_88221,
   BCMPH_VD_ZARLINK_88266,
   BCMPH_VD_MAX,
} phone_dev_type_t;

enum {
   // The device requires reset (through GPIO)
   BCMPH_CAPS_REQUIRES_RESET = 0x0001,
   // The device supports codec BCMPH_CODEC_ALAW
   BCMPH_CAPS_ALAW_CODEC = 0x0002,
   // The device supports codec BCMPH_CODEC_ULAW
   BCMPH_CAPS_ULAW_CODEC = 0x0004,
   // The device supports codec BCMPH_CODEC_LINEAR
   BCMPH_CAPS_LINEAR_CODEC = 0x0008,
   // The device supports codec BCMPH_CODEC_LINEAR16
   BCMPH_CAPS_LINEAR16_CODEC = 0x0010,
   // The device supports codec BCMPH_CODEC_ALAW16
   BCMPH_CAPS_ALAW16_CODEC = 0x0020,
   // The device supports codec BCMPH_CODEC_ULAW16
   BCMPH_CAPS_ULAW16_CODEC = 0x0040,
   // The device can have some lines in narrowband and some line in
   // wideband mode
   // For examples flag is not set for Le88266 because all lines must be
   // in narrowband or in wideband simultaneously
   BCMPH_CAPS_CAN_MIX_NB_AND_WB = 0x0100,
   // The device can switch lines between narrowband and wideband mode
   // without restarting device
   // This flag must not be set is flag BCMPH_CAPS_CAN_MIX_NB_AND_WB is
   // not set.
   // For example, for most of Zarlink devices flag is not set because
   // profiles used in narrowband and wideband are not the same
   BCMPH_CAPS_CAN_SWITCH_BETWEEN_NB_AND_WB = 0x0200,

   GPIO_IS_ACTIVE_LOW = 0x8000,
   GPIO_IS_INVALID = 0xFFFF
};

typedef struct {
#ifdef BCMPH_USE_SPI_DRIVER
   __s16 bus_num;	/* SPI bus num */
#endif // BCMPH_USE_SPI_DRIVER
   __u16 cs;      /* SPI chip select of the device */
   __u32 clk;     /* SPI clock speed (SCLK) */
   bool toggle_cs;  /* If it's necessary to toggle SPI CS between each byte */
#ifndef BCMPH_USE_SPI_DRIVER
   __u8 cs_off_time;  /* Minimum time CS must be inactive between subsequent transfers (in SCLK clock cycles) */
   bool wait_completion_with_irq; /* Whether to use irq to wait for end of transfer or simply loop */
   __u8 fill_byte; /* Byte sent by SPI controler when receiving bytes (in half duplex mode) */
#endif // !BCMPH_USE_SPI_DRIVER
} bcm_mpi_params_t;

typedef struct {
   phone_dev_type_t type;        /* Specific type of device (Le88276, Si32176, etc.) */
   __u32 caps;
   __u16 reset_gpio;  /* Reset GPIO : if no GPIO, set to GPIO_IS_INVALID, if GPIO active high, set bit 15 to 1, else set it to 0 */
   bcm_mpi_params_t mpi_params;
   __u8 line_count;
   phone_desc_line_t lines[BCMPH_MAX_LINES_PER_DEV];   /* Device channels */
   union {
      const struct zarlink_device_parameters *zarlink;
   } parameters;
} phone_desc_device_t;

typedef struct {
   __u32 pcm_ctrl_reg; // Configuration of PCM controller
   __u16 clk_rate; // Operating frequency of the PCLK signal of PCM bus (in kHz)
   __u8 tick_period;  // Period in msecs used to poll phone devices. Must be coherent with zarlink profiles
   __u8 device_count;
   phone_desc_device_t devices[BCMPH_MAX_DEVICES];
} phone_desc_t;

typedef struct {
   const char *name;
   const cpu_desc_t *cpu_desc;
   const pcm_desc_t *pcm_desc;
   const phone_desc_t *phone_desc;
} board_desc_t;

extern int board_init(void);

extern void board_deinit(void);

extern const board_desc_t *board_get_desc(void);

extern void board_set_gpio(int gpio_num, int state);

#endif // __BOARD_H__
