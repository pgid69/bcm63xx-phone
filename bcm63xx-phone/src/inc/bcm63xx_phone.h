/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */
#ifndef __BCM63XX_PHONE_H__
#define __BCM63XX_PHONE_H__

#include <linux/ioctl.h>
#include <linux/types.h>

#include "bcm63xx_line_state.h"
#include "bcm63xx_ring_buf.h"

/*
** The following will generate a list of countries as follows:
**
**        enum
**        {
**           BCMPH_COUNTRY_DE,
**           BCMPH_COUNTRY_JP,
**           BCMPH_COUNTRY_SE,
**           ...
**        } bcmph_country_t;
*/
#define COUNTRY_ARCHIVE_MAKE_NAME( country )    BCMPH_COUNTRY_##country,

typedef enum {
   #include <countryArchive.h>

   BCMPH_COUNTRY_MAX

} bcmph_country_t;

/* Magic number */
#define BCMPH_IOCTL_MAGIC 'E'

typedef struct {
   __u8 major;
   __u8 minor;
} bcm_phone_drv_ver_t;

typedef struct {
   __u32 ctlr;
   __u32 chan_ctrl;
   __u32 int_pending;
   __u32 int_mask;
   __u32 pll_ctrl1;
   __u32 pll_ctrl2;
   __u32 pll_ctrl3;
   __u32 pll_ctrl4;
   __u32 pll_stat;
   __u32 slot_alloc_tbl[16];
   __u32 dma_cfg;
   struct {
      __u32 dma_flowcl;
      __u32 dma_flowch;
      __u32 dma_bufalloc;

      __u32 dmac_chancfg;
      __u32 dmac_ir;
      __u32 dmac_irmask;
      __u32 dmac_maxburst;

      __u32 dmas_rstart;
      __u32 dmas_sram2;
      __u32 dmas_sram3;
      __u32 dmas_sram4;
   } dma_channels[2];
} bcm_phone_pcm_regs_t;

typedef struct {
   __u32 rx_errors;
   __u32 rx_length_errors;
   __u32 rx_empty_errors;
   __u32 rx_good;
   __u32 rx_bytes;
   __u32 tx_errors;
   __u32 tx_good;
   __u32 tx_bytes;
   __u32 cnt_irq_rx;
   __u32 cnt_irq_tx;
#ifdef BCMPH_TEST_PCM
   __u32 cnt_irq;
   __u32 cnt_irq_rx_overflow;
   __u32 cnt_irq_tx_underflow;
   __u32 rx_sop;
   __u32 rx_eop;
   __u32 min_size_rx_buffer;
   __u32 max_size_rx_buffer;
   __u32 min_size_tx_buffer;
   __u32 max_size_tx_buffer;
#endif /* !BCMPH_TEST_PCM */
} bcm_phone_pcm_stats_t;

/* Maximum numbers of lines : 2 seems to be a good value since routers
 with more FXS ports are uncommon */
#define BCMPH_MAX_LINES 2

typedef struct {
   /* Is the line active */
   __u8 enable;
   /* The codec to use for this line */
   bcm_phone_codec_t codec;
} bcm_phone_line_params_t;

typedef struct {
   char month[3];
   char day[3];
   char hour[3];
   char min[3];
   size_t num_len;
   char number[20];
   size_t name_len;
   char name[80];
} bcm_phone_cid_t;

typedef struct {
   bcmph_country_t country;
   __u8 pcm_use_16bits_timeslot;
   bcm_phone_line_params_t line_params[BCMPH_MAX_LINES];
} bcm_phone_cfg_params_t;

/*
 The minimum size of the RX an TX ring buffer.
 We choose a size allowing reception of 30 msecs of BCMPH_CODEC_LINEAR16
 encoded data (30 msecs is the maximum frame size of all the codecs
 listed here : http://www.en.voipforo.com/codec/codecs.php)
 */
#define BCMPH_MIN_SIZE_RING_BUFFER (8 * 4 * 30) /* in bytes */

/* Describe location of ring buffers in mmapped region (location changes after
 call to BCMPH_IOCTL_START and BCMPH_IOCTL_START_MM */
typedef struct {
   struct {
      /*
       Offset and size in the region where are the description of RX ring buffer
       At this offset there is a bcm_ring_buf_desc_t
       that describe the RX ring buffer
      */
      size_t rx_ring_buf_desc_off;
      size_t rx_ring_buf_desc_size;
      /*
       Offset and size in the region where is the memory used by the RX ring buffer
       Beware that rx_buffer_size is the size reserved for the buffer
       but actual len of ring buffer may be less (but not greater)
      */
      size_t rx_buffer_offset;
      size_t rx_buffer_size;
      /*
       Offset and size in the region where is the description of TX ring buffer
       At this offset there is a bcm_ring_buf_desc_t
       that describe the TX ring buffer
      */
      size_t tx_ring_buf_desc_off;
      size_t tx_ring_buf_desc_size;
      /*
       Offset and size in the region where is the memory used by the TX ring buffer
       Beware that tx_buffer_size is the size reserved for the buffer
       but actual len of ring buffer may be less (but not greater)
      */
      size_t tx_buffer_offset;
      size_t tx_buffer_size;
   } rbs[BCMPH_MAX_LINES];
} bcm_phone_get_mmap_rbs_location_t;

typedef struct {
   /* The line index we want to change the codec */
   size_t line;
   /* The new codec */
   bcm_phone_codec_t codec;
   /*
    Eventually change the line mode too if
    (BCMPH_MODE_UNSPECIFIED != mode)
   */
   bcm_phone_line_mode_t mode;
   /*
    Eventually change the line tone too if
    (BCMPH_MODE_UNSPECIFIED != mode)
    && (BCMPH_TONE_UNSPECIFIED != bcm_phone_line_tone_decode_index(tone))
   */
   __u32 tone;
} bcm_phone_set_line_codec_t;

typedef struct {
   /* The line index we want to change the mode */
   size_t line;
   /* The new mode */
   bcm_phone_line_mode_t mode;
   /*
    Eventually change the line tone too if
    (BCMPH_TONE_UNSPECIFIED != bcm_phone_line_tone_decode_index(tone))
   */
   __u32 tone;
   /*
    As mode is not changed immediately,
    asks to wait for the change to occurs before returning from ioctl
    If null, ask to change the mode and exits immediately.
    If positive, wait at most 'wait' ms that the mode changes
    If negative, wait indefinitely that the mode changes
   */
   int wait;
} bcm_phone_set_line_mode_t;

typedef struct {
   /* The line index we want to change the mode */
   size_t line;
   /* The new tone */
   __u32 tone;
   /*
    As tone is not changed immediately,
    asks to wait for the change to occurs before returning from ioctl
    If null, ask to change the mode and exits immediately.
    If positive, wait at most 'wait' ms that the mode changes
    If negative, wait indefinitely that the mode changes
   */
   int wait;
} bcm_phone_set_line_tone_t;

typedef struct {
   /*
    Used on input only.
    If null, read line state and exits immediately.
    If positive, wait at most 'wait' ms that state of at least one line changes
    If negative, wait indefinitely that state of at least one line changes
   */
   int wait;
   /* The state of all the lines */
   bcm_phone_line_state_t line_state[BCMPH_MAX_LINES];
} bcm_phone_get_line_states_t;

typedef struct {
   size_t line;
   /* The status of the line to set */
   bcm_phone_line_status_t status;
   /* The digits to add */
   char digits[64];
   __u16 digits_count;
   /* The number of flash events to add */
   __u16 flash_count;
} bcm_phone_set_line_state_t;

typedef struct {
   /*
     Size is set by the program who calls driver's ioctl
    it tells the number of bytes written or read
   */
   size_t size;
   union {
      bcm_phone_drv_ver_t read_version;
      bcm_phone_pcm_regs_t read_pcm_regs;
      bcm_phone_pcm_stats_t read_pcm_stats;
      bcm_phone_cfg_params_t start;
      bcm_phone_get_mmap_rbs_location_t get_mmap_rbs_location;
      bcm_phone_set_line_codec_t set_line_codec;
      bcm_phone_set_line_mode_t set_line_mode;
      bcm_phone_set_line_tone_t set_line_tone;
      bcm_phone_get_line_states_t get_line_states;
      bcm_phone_set_line_state_t set_line_state;
   } p;
} bcm_phone_ioctl_param_t;

/* Describe mmapped region */
typedef struct {
   /* Size of the region */
   size_t mmap_size;
   /*
    Offset and size (normally sizeof(bcm_phone_ioctl_args_t)) in the
    region where are read or written param of the ioctls that finished by _MM
    At this offset there is a bcm_phone_ioctl_param_t
   */
   size_t ioctl_param_off;
   size_t ioctl_param_size;
} bcm_phone_get_mmap_desc_t;

typedef struct {
   size_t line;
   size_t count;
   __u8 *buf;
   __u8 do_not_block;
} bcm_phone_read_t;

typedef struct {
   size_t line;
   size_t count;
   const __u8 *buf;
   __u8 do_not_block;
} bcm_phone_write_t;

/* Read the version of the driver */
#define BCMPH_IOCTL_READ_VERSION \
    _IOR(BCMPH_IOCTL_MAGIC, 0, bcm_phone_drv_ver_t)

/* Same as above, the result is written in mmap */
#define BCMPH_IOCTL_READ_VERSION_MM \
    _IO(BCMPH_IOCTL_MAGIC, 1)

/* Read the description of the mmap */
#define BCMPH_IOCTL_GET_MMAP_DESC \
   _IOR(BCMPH_IOCTL_MAGIC, 2, bcm_phone_get_mmap_desc_t)

/*
 For system calls read(), write() and poll(), define the default line
 the system call acts upon.
 The parameter is the line index
*/
#define BCMPH_IOCTL_SET_DEFAULT_LINE \
   _IO(BCMPH_IOCTL_MAGIC, 3)

/* Same as system call read but for another line that the default line */
#define BCMPH_IOCTL_READ \
   _IOW(BCMPH_IOCTL_MAGIC, 4, bcm_phone_read_t)

/*
 Tell the driver that some bytes have been read/removed from the RX
 ring buffer.
 The parameter contains the line index in the least significant byte and
 the number of bytes read in the other bytes
 On exit, the driver updates the RX and TX ring buf descriptors of all
 the lines
*/
#define BCMPH_IOCTL_READ_MM \
   _IO(BCMPH_IOCTL_MAGIC, 5)

/* Same as system call write but for another line that the default line */
#define BCMPH_IOCTL_WRITE \
   _IOW(BCMPH_IOCTL_MAGIC, 6, bcm_phone_write_t)

/*
 Tell the driver that some bytes have written/added to the TX ring buf
 The parameter contains the line index in the least significant byte and
 the number of bytes written in the other bytes
 On exit, the driver updates the RX and TX ring buf descriptors of all
 the lines
*/
#define BCMPH_IOCTL_WRITE_MM \
   _IO(BCMPH_IOCTL_MAGIC, 7)

/* Read the PCM registers */
#define BCMPH_IOCTL_READ_PCM_REGS \
    _IOR(BCMPH_IOCTL_MAGIC, 8, bcm_phone_pcm_regs_t)

/* Same as above, the result is written in mmap */
#define BCMPH_IOCTL_READ_PCM_REGS_MM \
    _IO(BCMPH_IOCTL_MAGIC, 9)

/* Read the statistics of the PCM subsytem */
#define BCMPH_IOCTL_READ_PCM_STATS \
    _IOR(BCMPH_IOCTL_MAGIC, 10, bcm_phone_pcm_stats_t)

/* Same as above, the result is written in mmap */
#define BCMPH_IOCTL_READ_PCM_STATS_MM \
    _IO(BCMPH_IOCTL_MAGIC, 11)

/* Configure the driver and start supervision of phone lines */
#define BCMPH_IOCTL_START \
    _IOW(BCMPH_IOCTL_MAGIC, 12, bcm_phone_cfg_params_t)

/* Same as above, the configuration is read from mmap */
#define BCMPH_IOCTL_START_MM \
    _IO(BCMPH_IOCTL_MAGIC, 13)

/* Stop PCM data transfers and the supervision of phone lines */
#define BCMPH_IOCTL_STOP \
    _IO(BCMPH_IOCTL_MAGIC, 14)

/* Get the location of the ring buffer of the active lines */
#define BCMPH_IOCTL_GET_MMAP_RBS_LOCATION \
   _IOR(BCMPH_IOCTL_MAGIC, 15, bcm_phone_get_mmap_rbs_location_t)

/*
 Get the location of the ring buffer of the active lines.
 Must be called after BCMPH_IOCTL_START or BCMPH_IOCTL_START_MM
*/
#define BCMPH_IOCTL_GET_MMAP_RBS_LOCATION_MM \
   _IO(BCMPH_IOCTL_MAGIC, 16)

/* Start PCM data transfers */
#define BCMPH_IOCTL_START_PCM \
    _IO(BCMPH_IOCTL_MAGIC, 17)

/* Stop PCM data transfers */
#define BCMPH_IOCTL_STOP_PCM \
    _IO(BCMPH_IOCTL_MAGIC, 18)

/* Change the codec used by a phone line */
#define BCMPH_IOCTL_SET_LINE_CODEC \
   _IOW(BCMPH_IOCTL_MAGIC, 19, bcm_phone_set_line_codec_t)

/* Same as above, codec is read from mmap */
#define BCMPH_IOCTL_SET_LINE_CODEC_MM \
   _IO(BCMPH_IOCTL_MAGIC, 20)

/* Change the mode of a phone line */
#define BCMPH_IOCTL_SET_LINE_MODE \
   _IOW(BCMPH_IOCTL_MAGIC, 21, bcm_phone_set_line_mode_t)

/* Same as above, mode is read from mmap */
#define BCMPH_IOCTL_SET_LINE_MODE_MM \
   _IO(BCMPH_IOCTL_MAGIC, 22)

/* Change the tone emitted by a phone line */
#define BCMPH_IOCTL_SET_LINE_TONE \
   _IOW(BCMPH_IOCTL_MAGIC, 23, bcm_phone_set_line_tone_t)

/* Same as above, tone is read from mmap */
#define BCMPH_IOCTL_SET_LINE_TONE_MM \
   _IO(BCMPH_IOCTL_MAGIC, 24)

/* Gets the state of all the phone lines */
#define BCMPH_IOCTL_GET_LINE_STATES \
   _IOWR(BCMPH_IOCTL_MAGIC, 25, bcm_phone_get_line_states_t)

/* Same as above, result is written in mmap */
#define BCMPH_IOCTL_GET_LINE_STATES_MM \
   _IO(BCMPH_IOCTL_MAGIC, 26)

/* Gets the state of all the phone lines */
#define BCMPH_IOCTL_SET_LINE_STATE \
   _IOW(BCMPH_IOCTL_MAGIC, 27, bcm_phone_set_line_state_t)

/* Same as above, result is written in mmap */
#define BCMPH_IOCTL_SET_LINE_STATE_MM \
   _IO(BCMPH_IOCTL_MAGIC, 28)

/*
 Ask the driver to update the RX and TX ring buf descriptors of all
 the lines in the mmap. Also done by READ_MM and WRITE_MM
*/
#define BCMPH_IOCTL_UPDATE_RBS \
   _IO(BCMPH_IOCTL_MAGIC, 29)

#endif /* __BCM63XX_PHONE_H__ */
