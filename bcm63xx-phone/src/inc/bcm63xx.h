/*
 * Copyright (C) 2015
 * Gilles Mazoyer <mazoyer.gilles@omega.ovh>
 *
 * This is free software, licensed under the GNU General Public License v2.
 * See /LICENSE for more information.
 */

#ifndef __BCM63XX_H__
#define __BCM63XX_H__

#include "config.h"

#ifndef BCMPH_NOHW
#include <bcm63xx_cpu.h>
#include <bcm63xx_regs.h>
#include <bcm63xx_io.h>
#endif // BCMPH_NOHW

// Extra definitions that should be in <bcm63xx_cpu.h>
#define RSET_PCM_SIZE            256
#define RSET_PCMDMA_SIZE         256
#define RSET_PCMDMAC_SIZE(chans) (16 * (chans))
#define RSET_PCMDMAS_SIZE(chans) (16 * (chans))

#undef BCM_6362_PCM_BASE
#define BCM_6362_PCM_BASE		(0xb000a000)
#undef BCM_6362_PCMDMA_BASE
#define BCM_6362_PCMDMA_BASE		(0xb000a800)
#undef BCM_6362_PCMDMAC_BASE
#define BCM_6362_PCMDMAC_BASE		(0xb000aa00)
#undef BCM_6362_PCMDMAS_BASE
#define BCM_6362_PCMDMAS_BASE		(0xb000ac00)

#define BCM_6328_PCM_IRQ		(IRQ_INTERNAL_BASE + 1)

#define BCM_6358_PCM_IRQ		(IRQ_INTERNAL_BASE + 22)

#define BCM_6368_PCM_IRQ		(IRQ_INTERNAL_BASE + 12)

// Extra definitions that should be in <bcm63xx_regs.h>
#define PERF_SOFTRESET_6358_REG		0x34

#define SOFTRESET_6358_SPI_MASK             (1 << 0)
#define SOFTRESET_6358_ENET_MASK            (1 << 2)
#define SOFTRESET_6358_MPI_MASK             (1 << 3)
#define SOFTRESET_6358_EPHY_MASK            (1 << 6)
#define SOFTRESET_6358_SAR_MASK             (1 << 7)
#define SOFTRESET_6358_USBH_MASK            (1 << 12)
#define SOFTRESET_6358_PCM_MASK             (1 << 13)
#define SOFTRESET_6358_ADSL_MASK            (1 << 14)

/*************************************************************************
 * _REG relative to RSET_PCM
 *************************************************************************/
#define PCM_CTRL_REG 0x00
#define  PCM_ENABLE              0x80000000 // PCM block master enable
#define  PCM_ENABLE_SHIFT        31
#define  PCM_SLAVE_SEL           0x40000000 // PCM TDM slave mode select (1 - TDM slave, 0 - TDM master)
#define  PCM_SLAVE_SEL_SHIFT     30
#define  PCM_CLOCK_INV           0x20000000 // PCM SCLK invert select (1 - invert, 0 - normal)
#define  PCM_CLOCK_INV_SHIFT     29
#define  PCM_FS_INVERT           0x10000000 // PCM FS invert select (1 - invert, 0 - normal)
#define  PCM_FS_INVERT_SHIFT     28
#define  PCM_FS_FREQ_16_8        0x08000000 // PCM FS 16/8 Khz select (1 - 16Khz, 0 - 8Khz)
#define  PCM_FS_FREQ_16_8_SHIFT  27
#define  PCM_FS_LONG             0x04000000 // PCM FS long/short select (1 - long FS, 0 - short FS)
#define  PCM_FS_LONG_SHIFT       26
#define  PCM_FS_TRIG             0x02000000 // PCM FS trigger (1 - falling edge, 0 - rising edge trigger)
#define  PCM_FS_TRIG_SHIFT       25
#define  PCM_DATA_OFF            0x01000000 // PCM data offset from FS (1 - one clock from FS, 0 - no offset)
#define  PCM_DATA_OFF_SHIFT      24
#define  PCM_DATA_16_8           0x00800000 // PCM data word length (1 - 16 bits, 0 - 8 bits)
#define  PCM_DATA_16_8_SHIFT     23
#define  PCM_CLOCK_SEL           0x00700000 // PCM SCLK freq select
#define  PCM_CLOCK_SEL_SHIFT     20
                                              // 000 - 8192 Khz
                                              // 001 - 4096 Khz
                                              // 010 - 2048 Khz
                                              // 011 - 1024 Khz
                                              // 100 - 512 Khz
                                              // 101 - 256 Khz
                                              // 110 - 128 Khz
                                              // 111 - reserved
#define  PCM_LSB_FIRST           0x00040000 // PCM shift direction (1 - LSBit first, 0 - MSBit first)
#define  PCM_LSB_FIRST_SHIFT     18
#define  PCM_LOOPBACK            0x00020000 // PCM diagnostic loobback enable
#define  PCM_LOOPBACK_SHIFT      17
#define  PCM_EXTCLK_SEL          0x00010000 // PCM external timing clock select -- Maybe removed in 6368
#define  PCM_EXTCLK_SEL_SHIFT    16
/*
 The following bit definitions of the PCM_CHAN_CTRL_REG
 are valid for BCM6362 and BCM6368
 For BCM6358 whatever they are, there's no difference : same data,
 same number of bytes sent and received.
*/
#define  PCM_636x_NTR_ENABLE          0x00008000 // PCM NTR counter enable -- Maybe removed in 6368
#define  PCM_636x_NTR_ENABLE_SHIFT    15
#define  PCM_636x_BITS_PER_FRAME_1024 0x00000400 // 1024 - Max
#define  PCM_636x_BITS_PER_FRAME_256  0x00000100 // 256
#define  PCM_636x_BITS_PER_FRAME_8    0x00000008 // 8    - Min
/*
 The following bit definition is valid for BCM6358
*/
#define  PCM_6358_AP_SEL              0x00000001 // set to give pads to PCM, reset for AP to use pads


#define PCM_CHAN_CTRL_REG 0x04
/*
 The following bit definitions of the PCM_CHAN_CTRL_REG
 are valid for BCM6358, BCM6362 and BCM6368
 Their usage is not clear, from trials i made on 6358.
 If one of the PCM_RXx_EN is null, we still receive data for that
 channel but all bytes are null : ok it seems logic
 If one of the PCM_TXx_EN is null, i saw no change, except
 that if all PCM_TXx_EN and PCM_RXx_EN are null, there's no
 DMA transfers from or to memory
 Meaning of PCM_RX_PACKET_SIZE is still to be discovered :
 different values change the data received or sent but i don't find
 any logic
 So must be set to 0 : with this value, data sent or received is divided
 in 32 bytes blocks. Each block include 4 bytes of data for each channel.
 Bytes to or from channel 0 are at offset 0 to 3, bytes to or from
 channel 1 are at offset 4 to 7...
 Beware that enabling PCM in PCM_CTRL_REG, resets the value
 of this register so it must be set after enabling PCM
*/
#define  PCM_TX0_EN              0x00000001 // PCM transmit channel 0 enable
#define  PCM_TX1_EN              0x00000002 // PCM transmit channel 1 enable
#define  PCM_TX2_EN              0x00000004 // PCM transmit channel 2 enable
#define  PCM_TX3_EN              0x00000008 // PCM transmit channel 3 enable
#define  PCM_TX4_EN              0x00000010 // PCM transmit channel 4 enable
#define  PCM_TX5_EN              0x00000020 // PCM transmit channel 5 enable
#define  PCM_TX6_EN              0x00000040 // PCM transmit channel 6 enable
#define  PCM_TX7_EN              0x00000080 // PCM transmit channel 7 enable
#define  PCM_RX0_EN              0x00000100 // PCM receive channel 0 enable
#define  PCM_RX1_EN              0x00000200 // PCM receive channel 1 enable
#define  PCM_RX2_EN              0x00000400 // PCM receive channel 2 enable
#define  PCM_RX3_EN              0x00000800 // PCM receive channel 3 enable
#define  PCM_RX4_EN              0x00001000 // PCM receive channel 4 enable
#define  PCM_RX5_EN              0x00002000 // PCM receive channel 5 enable
#define  PCM_RX6_EN              0x00004000 // PCM receive channel 6 enable
#define  PCM_RX7_EN              0x00008000 // PCM receive channel 7 enable
#define  PCM_ALL_CHANS_EN        0x0000ffff // All PCM channels
#define  PCM_RX_PACKET_SIZE      0x00ff0000 // PCM Rx DMA quasi-packet size
#define  PCM_RX_PACKET_SIZE_SHIFT  16

#define PCM_INT_PENDING_REG 0x08

#define PCM_INT_MASK_REG 0x0c
#define  PCM_TX_UNDERFLOW        0x00000001 // PCM DMA receive overflow
#define  PCM_RX_OVERFLOW         0x00000002 // PCM DMA transmit underflow
#define  PCM_TDM_FRAME           0x00000004 // PCM frame boundary
/*
 The following bit definitions
 are valid for BCM6362 and BCM6368 only
*/
#define  PCM_636x_RX_IRQ              0x00000008 // IUDMA interrupts
#define  PCM_636x_TX_IRQ              0x00000010

#define PCM_PLL_CTRL1_REG 0x10
#define  PCM_6358_PLL_RESET        0x80000000   // Digital reset
#define  PCM_6358_PLL_FILRST       0x40000000   // VCO/loop filter reset
#define  PCM_6358_PLL_PWRDN        0x20000000   // Powerdown enable for PLL
#define  PCM_6358_CLK16_RESET      0x10000000   // 16.382 MHz PCM interface circuitry reset.
                                                // Do not deassert until PCM PLL is out of reset and locked
#define  PCM_6358_PLL_SRCSEL       0x00030000   // Clock input to PLL
#define  PCM_6358_PLL_SRCSEL_SHIFT 16
#define  PCM_6358_PLL_DIVRETIME    0x00000010   // retime for 8-bit dividers (0 - high speed retime enabled, 1 - no retiming)
#define  PCM_6358_PLL_PI           0x00000001   // Phase interpolator (0 - 5-bit PI, 1 - 6-bit PI)

#define  PCM_636x_PLL_PWRDN           0x80000000 // PLL PWRDN
#define  PCM_636x_PLL_PWRDN_CH1       0x40000000 // PLL CH PWRDN
#define  PCM_636x_PLL_REFCMP_PWRDN    0x20000000 // PLL REFCMP PWRDN
#define  PCM_636x_PLL_CLK16_RESET     0x10000000 // 16.382 MHz PCM interface circuitry reset
#define  PCM_636x_PLL_ARESET          0x08000000 // PLL Analog Reset
#define  PCM_636x_PLL_DRESET          0x04000000 // PLL Digital Reset

#define PCM_PLL_CTRL2_REG 0x14

#define PCM_PLL_CTRL3_REG 0x18

#define PCM_PLL_CTRL4_636x_REG 0x1c

#define PCM_PLL_STAT_6358_REG 0x1c
#define PCM_PLL_STAT_636x_REG 0x20
#define  PCM_PLL_LOCK            0x00000001 // Asserted when PLL is locked to programmed frequency

#define PCM_NTR_COUNTER_636x_REG 0x24

#define PCM_SLOT_ALLOC_TBL_REG 0x40
/*
 Number of PCM time slot registers in the table.
 Each register provides the settings for 8 timeslots (4 bits per timeslot)
 For each timeslot we define if it's valid with marker PCM_TS_VALID
 and the PCM channel it is associated with, in the interval [0-7]
 From tests i made, it seems that assigning more than several timeslots
 to the same channel is not possible.
 (BTW why do we have 16 registers ? 8 registers should have been enough)
 Beware that reading the table when PCM module is started
 gives random values
*/
#define  PCM_MAX_TIMESLOT_REGS   16
#define  PCM_TS_VALID            0x8 // valid marker for TS alloc ram entry

/*************************************************************************
 * _REG relative to RSET_PCMDMA
 *************************************************************************/

/*
 DMA controller used by the PCM module has 4 (or 6 ?) channels on BCM6358
 and 8 channels on BCM6362 and BCM6368, like DMA controller
 used by Ethernet controller.
 Channels 0, 2, 4 and 6 are for reception.
 Channels 1, 3, 5, and 7 are for transmission.
 Only channels 0 and 1 are used (by default because perhaps field
 PCMDMA_CFG_6358_NUM_CHP allows to change the DMA channels used ?)
 The PCM module uses channel 0 to transfer data from the PCM bus to memory
 The PCM module uses channel 1 to transfer data from memory to the PCM bus
*/

/* Controller Configuration Register */
#define PCMDMA_CFG_REG        (0x0)
#define PCMDMA_CFG_6358_NUM_CHP_SHIFT    24
#define PCMDMA_CFG_6358 NUM_CHP_MASK     0x0f000000
#define PCMDMA_CFG_6358_FLOWCTL_MASK(x)  (1 << ((x >> 1) + 28))

#define PCMDMA_CFG_EN_SHIFT      0
#define PCMDMA_CFG_EN_MASK    (1 << PCMDMA_CFG_EN_SHIFT)
#define PCMDMA_CFG_FLOWCH_MASK(x)   (1 << ((x >> 1) + 1))

/*
 My understanding of registers FLOWCL and FLOWCH is that they allow
 to suspend DMA transfers if the number of empty buffers is lower
 (or equal ?) than the threshold set in FLOWCL, and to resume transfer
 if the number of empty buffer becomes greater (or equal ?) than the
 threshold set in FLOWCH
 These registers only apply to DMA reception channels
*/

/* Flow Control Descriptor Low Threshold register (used if enabled in PCMDMA_CFG_REG) */
#define PCMDMA_FLOWCL_REG(x)     (0x4 + (x) * 6)

/* Flow Control Descriptor High Threshold register (used if enabled in PCMDMA_CFG_REG) */
#define PCMDMA_FLOWCH_REG(x)     (0x8 + (x) * 6)

/* Flow Control Descriptor Buffer Alloca Threshold register */
#define PCMDMA_BUFALLOC_REG(x)      (0xc + (x) * 6)
#define PCMDMA_BUFALLOC_FORCE_SHIFT 31
#define PCMDMA_BUFALLOC_FORCE_MASK  (1 << PCMDMA_BUFALLOC_FORCE_SHIFT)

/*************************************************************************
 * _REG relative to RSET_PCMDMAC
 *************************************************************************/

/* Channel Configuration register */
#define PCMDMAC_CHANCFG_REG(x)      ((x) * 0x10)
#define PCMDMAC_CHANCFG_EN_SHIFT 0
#define PCMDMAC_CHANCFG_EN_MASK   (1 << PCMDMAC_CHANCFG_EN_SHIFT)
#define PCMDMAC_CHANCFG_PKTHALT_SHIFT  1
#define PCMDMAC_CHANCFG_PKTHALT_MASK   (1 << PCMDMA_CHANCFG_PKTHALT_SHIFT)
#define PCMDMAC_CHANCFG_BURSTHALT_SHIFT 2
#define PCMDMAC_CHANCFG_BURSTHALT_MASK   (1 << PCMDMAC_CHANCFG_BURSTHALT_SHIFT)

/* Interrupt Control/Status register */
#define PCMDMAC_IR_REG(x)     (0x4 + (x) * 0x10)
#define PCMDMAC_IR_BUFDONE_MASK  (1 << 0) // Buffer done
#define PCMDMAC_IR_PKTDONE_MASK  (1 << 1) // Packet done
#define PCMDMAC_IR_NOTOWNER_MASK (1 << 2) // No valid descriptor

/* Interrupt Mask register */
#define PCMDMAC_IRMASK_REG(x)    (0x8 + (x) * 0x10)

/* Maximum Burst Length */
#define PCMDMAC_MAXBURST_REG(x)  (0xc + (x) * 0x10)
#define PCMDMAC_MAXBURST_SIZE 16 /* 32-bit words */

/*************************************************************************
 * _REG relative to RSET_PCMDMAS
 *************************************************************************/

/* Ring Start Address register */
#define PCMDMAS_RSTART_REG(x)    ((x) * 0x10)

/* State info: how manu bytes done and the offset of the current
   descritor in process */
#define PCMDMAS_SRAM2_REG(x)     (0x4 + (x) * 0x10)

#define PCMDMAS_STRAM_DESC_RING_OFFSET 0x3fff

/* Length and status field of the current descriptor */
#define PCMDMAS_SRAM3_REG(x)     (0x8 + (x) * 0x10)

/* Pointer to the current descriptor */
#define PCMDMAS_SRAM4_REG(x)     (0xc + (x) * 0x10)

#define DMADESC_LENGTH_SHIFT 16
#define DMADESC_USEFPM       (1 << 31)
#define DMADESC_MULTICAST    (1 << 30)
#define DMADESC_LENGTH_MASK  (0xfff << DMADESC_LENGTH_SHIFT) /* Buffer length can't exceed 4095 bytes */
#define DMADESC_OWNER_MASK   (1 << 15)
#define DMADESC_EOP_MASK     (1 << 14)
#define DMADESC_SOP_MASK     (1 << 13)
#define DMADESC_ESOP_MASK    (DMADESC_EOP_MASK | DMADESC_SOP_MASK)
#define DMADESC_WRAP_MASK    (1 << 12)

#define DMADESC_APPEND_CRC   (1 << 8)

/*
************************************************************************
*/

/*
 Here are the Broadcom definitions of PCM and DMA for BCM6358.
 They come from sources of DLink DSL 2650U
 (ftp://ftp.dlink.ru/pub/ADSL/GPL_source_code/DSL-2650U_BRU_D/DLink_DSL-2650U_RU_1.00_GPL.rar)

typedef struct PcmControlRegisters
{
    uint32 pcm_ctrl;                            // 00 offset from PCM_BASE
#define  PCM_ENABLE              0x80000000     // PCM block master enable
#define  PCM_ENABLE_SHIFT        31
#define  PCM_SLAVE_SEL           0x40000000     // PCM TDM slave mode select (1 - TDM slave, 0 - TDM master)
#define  PCM_SLAVE_SEL_SHIFT     30
#define  PCM_CLOCK_INV           0x20000000     // PCM SCLK invert select (1 - invert, 0 - normal)
#define  PCM_CLOCK_INV_SHIFT     29
#define  PCM_FS_INVERT           0x10000000     // PCM FS invert select (1 - invert, 0 - normal)
#define  PCM_FS_INVERT_SHIFT     28
#define  PCM_FS_FREQ_16_8        0x08000000     // PCM FS 16/8 Khz select (1 - 16Khz, 0 - 8Khz)
#define  PCM_FS_FREQ_16_8_SHIFT  27
#define  PCM_FS_LONG             0x04000000     // PCM FS long/short select (1 - long FS, 0 - short FS)
#define  PCM_FS_LONG_SHIFT       26
#define  PCM_FS_TRIG             0x02000000     // PCM FS trigger (1 - falling edge, 0 - rising edge trigger)
#define  PCM_FS_TRIG_SHIFT       25
#define  PCM_DATA_OFF            0x01000000     // PCM data offset from FS (1 - one clock from FS, 0 - no offset)
#define  PCM_DATA_OFF_SHIFT      24
#define  PCM_DATA_16_8           0x00800000     // PCM data word length (1 - 16 bits, 0 - 8 bits)
#define  PCM_DATA_16_8_SHIFT     23
#define  PCM_CLOCK_SEL           0x00700000     // PCM SCLK freq select
#define  PCM_CLOCK_SEL_SHIFT     20
                                                  // 000 - 8192 Khz
                                                  // 001 - 4096 Khz
                                                  // 010 - 2048 Khz
                                                  // 011 - 1024 Khz
                                                  // 100 - 512 Khz
                                                  // 101 - 256 Khz
                                                  // 110 - 128 Khz
                                                  // 111 - reserved
#define  PCM_LSB_FIRST           0x00040000     // PCM shift direction (1 - LSBit first, 0 - MSBit first)
#define  PCM_LSB_FIRST_SHIFT     18
#define  PCM_LOOPBACK            0x00020000     // PCM diagnostic loobback enable
#define  PCM_LOOPBACK_SHIFT      17
#define  PCM_EXTCLK_SEL          0x00010000     // PCM external timing clock select
#define  PCM_EXTCLK_SEL_SHIFT    16
#define  PCM_AP_SEL              0x00000001     // set to give pads to PCM, reset for AP to use pads

    uint32 pcm_chan_ctrl;                       // 04
#define  PCM_TX0_EN              0x00000001     // PCM transmit channel 0 enable
#define  PCM_TX1_EN              0x00000002     // PCM transmit channel 1 enable
#define  PCM_TX2_EN              0x00000004     // PCM transmit channel 2 enable
#define  PCM_TX3_EN              0x00000008     // PCM transmit channel 3 enable
#define  PCM_TX4_EN              0x00000010     // PCM transmit channel 4 enable
#define  PCM_TX5_EN              0x00000020     // PCM transmit channel 5 enable
#define  PCM_TX6_EN              0x00000040     // PCM transmit channel 6 enable
#define  PCM_TX7_EN              0x00000080     // PCM transmit channel 7 enable
#define  PCM_RX0_EN              0x00000100     // PCM receive channel 0 enable
#define  PCM_RX1_EN              0x00000200     // PCM receive channel 1 enable
#define  PCM_RX2_EN              0x00000400     // PCM receive channel 2 enable
#define  PCM_RX3_EN              0x00000800     // PCM receive channel 3 enable
#define  PCM_RX4_EN              0x00001000     // PCM receive channel 4 enable
#define  PCM_RX5_EN              0x00002000     // PCM receive channel 5 enable
#define  PCM_RX6_EN              0x00004000     // PCM receive channel 6 enable
#define  PCM_RX7_EN              0x00008000     // PCM receive channel 7 enable
#define  PCM_RX_PACKET_SIZE      0x00ff0000     // PCM Rx DMA quasi-packet size
#define  PCM_RX_PACKET_SIZE_SHIFT  16

    uint32 pcm_int_pending;                     // 08
    uint32 pcm_int_mask;                        // 0c
#define  PCM_TX_UNDERFLOW        0x00000001     // PCM DMA receive overflow
#define  PCM_RX_OVERFLOW         0x00000002     // PCM DMA transmit underflow
#define  PCM_TDM_FRAME           0x00000004     // PCM frame boundary

    uint32 pcm_pll_ctrl1;                       // 10
#define  PCM_PLL_RESET           0x80000000     // Digital reset
#define  PCM_PLL_FILRST          0x40000000     // VCO/loop filter reset
#define  PCM_PLL_PWRDN           0x20000000     // Powerdown enable for PLL
#define  PCM_CLK16_RESET         0x10000000     // 16.382 MHz PCM interface circuitry reset.
                                                // Do not deassert until PCM PLL is out of reset and locked
#define  PCM_PLL_SRCSEL          0x00030000     // Clock input to PLL
#define  PCM_PLL_SRCSEL_SHIFT    16
#define  PCM_PLL_DIVRETIME       0x00000010     // retime for 8-bit dividers (0 - high speed retime enabled, 1 - no retiming)
#define  PCM_PLL_PI              0x00000001     // Phase interpolator (0 - 5-bit PI, 1 - 6-bit PI)

    uint32 pcm_pll_ctrl2;                       // 14
    uint32 pcm_pll_ctrl3;                       // 18

    uint32 pcm_pll_stat;                        // 1c
#define  PCM_PLL_LOCK            0x00000001     // Asserted when PLL is locked to programmed frequency

    uint32 unused[8];

#define  PCM_MAX_TIMESLOT_REGS   16             // Number of PCM time slot registers in the table.
                                                // Each register provides the settings for 8 timeslots (4 bits per timeslot)
    uint32 pcm_slot_alloc_tbl[PCM_MAX_TIMESLOT_REGS];
#define  PCM_TS_VALID            0x8            // valid marker for TS alloc ram entry
} PcmControlRegisters;

#define PCM ((volatile PcmControlRegisters * const) PCM_BASE)

typedef struct PcmIudmaRegisters
{
	uint16 numChp;
#define BCM6358_IUDMA_REGS_NUM_CHP_MASK     0x0f00
#define BCM6358_IUDMA_REGS_NUM_CHP_SHIFT    8
   uint16 ctrlConfig;
#define BCM6358_IUDMA_REGS_CTRLCONFIG_MASTER_EN        0x0001
#define BCM6358_IUDMA_REGS_CTRLCONFIG_FLOWC_CH1_EN     0x0002
#define BCM6358_IUDMA_REGS_CTRLCONFIG_FLOWC_CH3_EN     0x0004

	// Flow control Ch1
   uint16 reserved1;
   uint16 ch1_FC_Low_Thr;

   uint16 reserved2;
   uint16 ch1_FC_High_Thr;

   uint16 reserved3;
   uint16 ch1_Buff_Alloc;

	// Flow control Ch3
	uint16 reserved4;
	uint16 ch3_FC_Low_Thr;

	uint16 reserved5;
	uint16 ch3_FC_High_Thr;

	uint16 reserved6;
	uint16 ch3_Buff_Alloc;

} PcmIudmaRegisters;

typedef struct PcmIudmaChannelCtrl
{
   uint16 reserved1;
	uint16 config;
#define BCM6358_IUDMA_CONFIG_ENDMA       0x0001
#define BCM6358_IUDMA_CONFIG_PKTHALT     0x0002
#define BCM6358_IUDMA_CONFIG_BURSTHALT   0x0004

	uint16 reserved2;
	uint16 intStat;
#define BCM6358_IUDMA_INTSTAT_BDONE   0x0001
#define BCM6358_IUDMA_INTSTAT_PDONE   0x0002
#define BCM6358_IUDMA_INTSTAT_NOTVLD  0x0004
#define BCM6358_IUDMA_INTSTAT_MASK    0x0007
#define BCM6358_IUDMA_INTSTAT_ALL     BCM6358_IUDMA_INTSTAT_MASK

	uint16 reserved3;
	uint16 intMask;
#define BCM6358_IUDMA_INTMASK_BDONE   0x0001
#define BCM6358_IUDMA_INTMASK_PDONE   0x0002
#define BCM6358_IUDMA_INTMASK_NOTVLD  0x0004

	uint16 reserved4;
	uint16 maxBurst;
#define BCM6358_IUDMA_MAXBURST_SIZE 16 // 32-bit words

} PcmIudmaChannelCtrl;

typedef struct PcmIudmaStateRam
{
   uint32 baseDescPointer;                // pointer to first buffer descriptor

   uint32 stateBytesDoneRingOffset;       // state info: how manu bytes done and the offset of the
                                             current descritor in process
#define BCM6358_IUDMA_STRAM_DESC_RING_OFFSET 0x3fff


   uint32 flagsLengthStatus;              // Length and status field of the current descriptor

   uint32 currentBufferPointer;           // pointer to the current descriptor

} PcmIudmaStateRam;

#define BCM6358_MAX_PCM_DMA_CHANNELS 6

typedef struct PcmIudma
{
   PcmIudmaRegisters regs;
   uint16 reserved1[114];
   PcmIudmaChannelCtrl ctrl[BCM6358_MAX_PCM_DMA_CHANNELS];
   uint16 reserved2[80];
   PcmIudmaStateRam stram[BCM6358_MAX_PCM_DMA_CHANNELS];

} PcmIudma;

#define PCM_IUDMA ((volatile PcmIudma * const) PCM_IUDMA_BASE)


#define IUDMA_MAX_CHANNELS      16

// DMA Channel Configuration (1 .. 16)
typedef struct DmaChannelCfg {
  uint32        cfg;                    // (00) assorted configuration
#define         DMA_BURST_HALT  0x00000004  // idle after finish current memory burst
#define         DMA_PKT_HALT    0x00000002  // idle after an EOP flag is detected
#define         DMA_ENABLE  0x00000001      // set to enable channel
  uint32        intStat;                // (04) interrupts control and status
  uint32        intMask;                // (08) interrupts mask
#define         DMA_BUFF_DONE   0x00000001  // buffer done
#define         DMA_DONE        0x00000002  // packet xfer complete
#define         DMA_NO_DESC     0x00000004  // no valid descriptors
  uint32        maxBurst;               // (0C) max burst length permitted
} DmaChannelCfg;

// DMA State RAM (1 .. 16)
typedef struct DmaStateRam {
  uint32        baseDescPtr;            // (00) descriptor ring start address
  uint32        state_data;             // (04) state/bytes done/ring offset
  uint32        desc_len_status;        // (08) buffer descriptor status and len
  uint32        desc_base_bufptr;       // (0C) buffer descrpitor current processing
} DmaStateRam;

// DMA Registers
typedef struct DmaRegs {
#define DMA_MASTER_EN           0x00000001
#define DMA_FLOWC_CH1_EN        0x00000002
#define DMA_FLOWC_CH3_EN        0x00000004
#define DMA_NUM_CHS_MASK        0x0f000000
#define DMA_NUM_CHS_SHIFT       24
#define DMA_FLOWCTL_MASK        0x30000000
#define DMA_FLOWCTL_CH1         0x10000000
#define DMA_FLOWCTL_CH3         0x20000000
#define DMA_FLOWCTL_SHIFT       28
    uint32 controller_cfg;              // (00) controller configuration

    // Flow control Ch1
    uint32 flowctl_ch1_thresh_lo;       // (04) EMAC1 RX DMA channel
    uint32 flowctl_ch1_thresh_hi;       // (08) EMAC1 RX DMA channel
    uint32 flowctl_ch1_alloc;           // (0C) EMAC1 RX DMA channel
#define DMA_BUF_ALLOC_FORCE     0x80000000

    // Flow control Ch3
    uint32 flowctl_ch3_thresh_lo;       // (10) EMAC2 RX DMA channel
    uint32 flowctl_ch3_thresh_hi;       // (14) EMAC2 RX DMA channel
    uint32 flowctl_ch3_alloc;           // (18) EMAC2 RX DMA channel

    // Unused words
    uint32 resv[57];

    // Per channel registers/state ram
    DmaChannelCfg chcfg[IUDMA_MAX_CHANNELS]; // (100) Channel configuration
    union {
        DmaStateRam     s[IUDMA_MAX_CHANNELS];
        uint32          u32[4 * IUDMA_MAX_CHANNELS];
    } stram;                                // (200) state ram
} DmaRegs;

// DMA Buffer
typedef struct DmaDesc {
  uint16        length;                 // in bytes of data in buffer
#define          DMA_DESC_USEFPM    0x8000
#define          DMA_DESC_MULTICAST 0x4000
#define          DMA_DESC_BUFLENGTH 0x0fff
  uint16        status;                 // buffer status
#define          DMA_OWN        0x8000  // cleared by DMA, set by SW
#define          DMA_EOP        0x4000  // last buffer in packet
#define          DMA_SOP        0x2000  // first buffer in packet
#define          DMA_WRAP       0x1000  //
#define          DMA_APPEND_CRC 0x0100

// EMAC Descriptor Status definitions
#define          EMAC_MISS      0x0080  // framed address recognition failed (promiscuous)
#define          EMAC_BRDCAST   0x0040  // DA is Broadcast
#define          EMAC_MULT      0x0020  // DA is multicast
#define          EMAC_LG        0x0010  // frame length > RX_LENGTH register value
#define          EMAC_NO        0x0008  // Non-Octet aligned
#define          EMAC_RXER      0x0004  // RX_ERR on MII while RX_DV assereted
#define          EMAC_CRC_ERROR 0x0002  // CRC error
#define          EMAC_OV        0x0001  // Overflow

// HDLC Descriptor Status definitions
#define          DMA_HDLC_TX_ABORT      0x0100
#define          DMA_HDLC_RX_OVERRUN    0x4000
#define          DMA_HDLC_RX_TOO_LONG   0x2000
#define          DMA_HDLC_RX_CRC_OK     0x1000
#define          DMA_HDLC_RX_ABORT      0x0100

  uint32        address;                // address of data
} DmaDesc;
*/

/*
 Here are the Broadcom definitions of PCM and DMA for BCM6362 and BCM6368.
 They come from sources of Motorola NVG510
 http://sourceforge.net/arris/nvg510/home/Home/

typedef struct PcmControlRegisters
{
    uint32 pcm_ctrl;                            // 00 offset from PCM_BASE
#define  PCM_ENABLE              0x80000000     // PCM block master enable
#define  PCM_ENABLE_SHIFT        31
#define  PCM_SLAVE_SEL           0x40000000     // PCM TDM slave mode select (1 - TDM slave, 0 - TDM master)
#define  PCM_SLAVE_SEL_SHIFT     30
#define  PCM_CLOCK_INV           0x20000000     // PCM SCLK invert select (1 - invert, 0 - normal)
#define  PCM_CLOCK_INV_SHIFT     29
#define  PCM_FS_INVERT           0x10000000     // PCM FS invert select (1 - invert, 0 - normal)
#define  PCM_FS_INVERT_SHIFT     28
#define  PCM_FS_FREQ_16_8        0x08000000     // PCM FS 16/8 Khz select (1 - 16Khz, 0 - 8Khz)
#define  PCM_FS_FREQ_16_8_SHIFT  27
#define  PCM_FS_LONG             0x04000000     // PCM FS long/short select (1 - long FS, 0 - short FS)
#define  PCM_FS_LONG_SHIFT       26
#define  PCM_FS_TRIG             0x02000000     // PCM FS trigger (1 - falling edge, 0 - rising edge trigger)
#define  PCM_FS_TRIG_SHIFT       25
#define  PCM_DATA_OFF            0x01000000     // PCM data offset from FS (1 - one clock from FS, 0 - no offset)
#define  PCM_DATA_OFF_SHIFT      24
#define  PCM_DATA_16_8           0x00800000     // PCM data word length (1 - 16 bits, 0 - 8 bits)
#define  PCM_DATA_16_8_SHIFT     23
#define  PCM_CLOCK_SEL           0x00700000     // PCM SCLK freq select
#define  PCM_CLOCK_SEL_SHIFT     20
                                                  // 000 - 8192 Khz
                                                  // 001 - 4096 Khz
                                                  // 010 - 2048 Khz
                                                  // 011 - 1024 Khz
                                                  // 100 - 512 Khz
                                                  // 101 - 256 Khz
                                                  // 110 - 128 Khz
                                                  // 111 - reserved
#define  PCM_LSB_FIRST           0x00040000     // PCM shift direction (1 - LSBit first, 0 - MSBit first)
#define  PCM_LSB_FIRST_SHIFT     18
#define  PCM_LOOPBACK            0x00020000     // PCM diagnostic loobback enable
#define  PCM_LOOPBACK_SHIFT      17
#define  PCM_EXTCLK_SEL          0x00010000     // PCM external timing clock select -- Maybe removed in 6368
#define  PCM_EXTCLK_SEL_SHIFT    16
#define  PCM_NTR_ENABLE          0x00008000     // PCM NTR counter enable -- Nayve removed in 6368
#define  PCM_NTR_ENABLE_SHIFT    15
#define  PCM_BITS_PER_FRAME_1024 0x00000400     // 1024 - Max
#define  PCM_BITS_PER_FRAME_256  0x00000100     // 256
#define  PCM_BITS_PER_FRAME_8    0x00000008     // 8    - Min

    uint32 pcm_chan_ctrl;                       // 04
#define  PCM_TX0_EN              0x00000001     // PCM transmit channel 0 enable
#define  PCM_TX1_EN              0x00000002     // PCM transmit channel 1 enable
#define  PCM_TX2_EN              0x00000004     // PCM transmit channel 2 enable
#define  PCM_TX3_EN              0x00000008     // PCM transmit channel 3 enable
#define  PCM_TX4_EN              0x00000010     // PCM transmit channel 4 enable
#define  PCM_TX5_EN              0x00000020     // PCM transmit channel 5 enable
#define  PCM_TX6_EN              0x00000040     // PCM transmit channel 6 enable
#define  PCM_TX7_EN              0x00000080     // PCM transmit channel 7 enable
#define  PCM_RX0_EN              0x00000100     // PCM receive channel 0 enable
#define  PCM_RX1_EN              0x00000200     // PCM receive channel 1 enable
#define  PCM_RX2_EN              0x00000400     // PCM receive channel 2 enable
#define  PCM_RX3_EN              0x00000800     // PCM receive channel 3 enable
#define  PCM_RX4_EN              0x00001000     // PCM receive channel 4 enable
#define  PCM_RX5_EN              0x00002000     // PCM receive channel 5 enable
#define  PCM_RX6_EN              0x00004000     // PCM receive channel 6 enable
#define  PCM_RX7_EN              0x00008000     // PCM receive channel 7 enable
#define  PCM_RX_PACKET_SIZE      0x00ff0000     // PCM Rx DMA quasi-packet size
#define  PCM_RX_PACKET_SIZE_SHIFT  16

    uint32 pcm_int_pending;                     // 08
    uint32 pcm_int_mask;                        // 0c
#define  PCM_TX_UNDERFLOW        0x00000001     // PCM DMA receive overflow
#define  PCM_RX_OVERFLOW         0x00000002     // PCM DMA transmit underflow
#define  PCM_TDM_FRAME           0x00000004     // PCM frame boundary
#define  PCM_RX_IRQ              0x00000008     // IUDMA interrupts
#define  PCM_TX_IRQ              0x00000010

    uint32 pcm_pll_ctrl1;                       // 10
#define  PCM_PLL_PWRDN           0x80000000     // PLL PWRDN
#define  PCM_PLL_PWRDN_CH1       0x40000000     // PLL CH PWRDN
#define  PCM_PLL_REFCMP_PWRDN    0x20000000     // PLL REFCMP PWRDN
#define  PCM_CLK16_RESET         0x10000000     // 16.382 MHz PCM interface circuitry reset.
#define  PCM_PLL_ARESET          0x08000000     // PLL Analog Reset
#define  PCM_PLL_DRESET          0x04000000     // PLL Digital Reset

    uint32 pcm_pll_ctrl2;                       // 14
    uint32 pcm_pll_ctrl3;                       // 18
    uint32 pcm_pll_ctrl4;                       // 1c

    uint32 pcm_pll_stat;                        // 20
#define  PCM_PLL_LOCK            0x00000001     // Asserted when PLL is locked to programmed frequency

    uint32 pcm_ntr_counter;                     // 24

    uint32 unused[6];

#define  PCM_MAX_TIMESLOT_REGS   16             // Number of PCM time slot registers in the table.
                                                // Each register provides the settings for 8 timeslots (4 bits per timeslot)
    uint32 pcm_slot_alloc_tbl[PCM_MAX_TIMESLOT_REGS];
#define  PCM_TS_VALID            0x8            // valid marker for TS alloc ram entry
} PcmControlRegisters;

#define PCM ((volatile PcmControlRegisters * const) PCM_BASE)

typedef struct PcmIudmaRegisters
{
	uint16 reserved0;
   uint16 ctrlConfig;
#define BCM6368_IUDMA_REGS_CTRLCONFIG_MASTER_EN        0x0001
#define BCM6368_IUDMA_REGS_CTRLCONFIG_FLOWC_CH1_EN     0x0002
#define BCM6368_IUDMA_REGS_CTRLCONFIG_FLOWC_CH3_EN     0x0004
#define BCM6368_IUDMA_REGS_CTRLCONFIG_FLOWC_CH5_EN     0x0008
#define BCM6368_IUDMA_REGS_CTRLCONFIG_FLOWC_CH7_EN     0x0010

	// Flow control Ch1
   uint16 reserved1;
   uint16 ch1_FC_Low_Thr;

   uint16 reserved2;
   uint16 ch1_FC_High_Thr;

   uint16 reserved3;
   uint16 ch1_Buff_Alloc;

	// Flow control Ch3
	uint16 reserved4;
	uint16 ch3_FC_Low_Thr;

	uint16 reserved5;
	uint16 ch3_FC_High_Thr;

	uint16 reserved6;
	uint16 ch3_Buff_Alloc;

	// Flow control Ch5
	uint16 reserved7;
	uint16 ch5_FC_Low_Thr;

	uint16 reserved8;
	uint16 ch5_FC_High_Thr;

	uint16 reserved9;
	uint16 ch5_Buff_Alloc;

	// Flow control Ch7
	uint16 reserved10;
	uint16 ch7_FC_Low_Thr;

	uint16 reserved11;
	uint16 ch7_FC_High_Thr;

	uint16 reserved12;
	uint16 ch7_Buff_Alloc;

	// Channel resets
	uint16 reserved13;
	uint16 channel_reset;

	uint16 reserved14;
	uint16 channel_debug;

	// Spare register
	uint32 dummy1;

	// Interrupt status registers
	uint16 reserved15;
	uint16 gbl_int_stat;

	// Interrupt mask registers
	uint16 reserved16;
	uint16 gbl_int_mask;
} PcmIudmaRegisters;

typedef struct PcmIudmaChannelCtrl
{
   uint16 reserved1;
	uint16 config;
#define BCM6368_IUDMA_CONFIG_ENDMA       0x0001
#define BCM6368_IUDMA_CONFIG_PKTHALT     0x0002
#define BCM6368_IUDMA_CONFIG_BURSTHALT   0x0004

	uint16 reserved2;
	uint16 intStat;
#define BCM6368_IUDMA_INTSTAT_BDONE   0x0001
#define BCM6368_IUDMA_INTSTAT_PDONE   0x0002
#define BCM6368_IUDMA_INTSTAT_NOTVLD  0x0004
#define BCM6368_IUDMA_INTSTAT_MASK    0x0007
#define BCM6368_IUDMA_INTSTAT_ALL     BCM6368_IUDMA_INTSTAT_MASK

	uint16 reserved3;
	uint16 intMask;
#define BCM6368_IUDMA_INTMASK_BDONE   0x0001
#define BCM6368_IUDMA_INTMASK_PDONE   0x0002
#define BCM6368_IUDMA_INTMASK_NOTVLD  0x0004

	uint32 maxBurst;
#define BCM6368_IUDMA_MAXBURST_SIZE 16 // 32-bit words

} PcmIudmaChannelCtrl;


typedef struct PcmIudmaStateRam
{
   uint32 baseDescPointer;                // pointer to first buffer descriptor

   uint32 stateBytesDoneRingOffset;       // state info: how manu bytes done and the offset of the
                                             current descritor in process
#define BCM6368_IUDMA_STRAM_DESC_RING_OFFSET 0x3fff


   uint32 flagsLengthStatus;              // Length and status field of the current descriptor

   uint32 currentBufferPointer;           // pointer to the current descriptor

} PcmIudmaStateRam;

#define BCM6368_MAX_PCM_DMA_CHANNELS 2

typedef struct PcmIudma
{
   PcmIudmaRegisters regs;                                        //
   uint32 reserved1[110];                                         //
   PcmIudmaChannelCtrl ctrl[BCM6368_MAX_PCM_DMA_CHANNELS];        //
   uint32 reserved2[120];                                         //
   PcmIudmaStateRam stram[BCM6368_MAX_PCM_DMA_CHANNELS];          //

} PcmIudma;

#define PCM_IUDMA ((volatile PcmIudma * const) PCM_DMA_BASE)


#define IUDMA_MAX_CHANNELS          32

// DMA Channel Configuration (1 .. 32)
typedef struct DmaChannelCfg {
  uint32        cfg;                    // (00) assorted configuration
#define         DMA_ENABLE      0x00000001  // set to enable channel
#define         DMA_PKT_HALT    0x00000002  // idle after an EOP flag is detected
#define         DMA_BURST_HALT  0x00000004  // idle after finish current memory burst
  uint32        intStat;                // (04) interrupts control and status
  uint32        intMask;                // (08) interrupts mask
#define         DMA_BUFF_DONE   0x00000001  // buffer done
#define         DMA_DONE        0x00000002  // packet xfer complete
#define         DMA_NO_DESC     0x00000004  // no valid descriptors
  uint32        maxBurst;               // (0C) max burst length permitted
#define         DMA_DESCSIZE_SEL 0x00040000  // DMA Descriptor Size Selection, BCM6362 only
} DmaChannelCfg;

// DMA State RAM (1 .. 16)
typedef struct DmaStateRam {
  uint32        baseDescPtr;            // (00) descriptor ring start address
  uint32        state_data;             // (04) state/bytes done/ring offset
  uint32        desc_len_status;        // (08) buffer descriptor status and len
  uint32        desc_base_bufptr;       // (0C) buffer descrpitor current processing
} DmaStateRam;


// DMA Registers
typedef struct DmaRegs {
    uint32 controller_cfg;              // (00) controller configuration
#define DMA_MASTER_EN           0x00000001
#define DMA_FLOWC_CH1_EN        0x00000002
#define DMA_FLOWC_CH3_EN        0x00000004

    // Flow control Ch1
    uint32 flowctl_ch1_thresh_lo;           // 004
    uint32 flowctl_ch1_thresh_hi;           // 008
    uint32 flowctl_ch1_alloc;               // 00c
#define DMA_BUF_ALLOC_FORCE     0x80000000

    // Flow control Ch3
    uint32 flowctl_ch3_thresh_lo;           // 010
    uint32 flowctl_ch3_thresh_hi;           // 014
    uint32 flowctl_ch3_alloc;               // 018

    // Flow control Ch5
    uint32 flowctl_ch5_thresh_lo;           // 01C
    uint32 flowctl_ch5_thresh_hi;           // 020
    uint32 flowctl_ch5_alloc;               // 024

    // Flow control Ch7
    uint32 flowctl_ch7_thresh_lo;           // 028
    uint32 flowctl_ch7_thresh_hi;           // 02C
    uint32 flowctl_ch7_alloc;               // 030

    uint32 ctrl_channel_reset;              // 034
    uint32 ctrl_channel_debug;              // 038
    uint32 reserved1;                       // 03C
    uint32 ctrl_global_interrupt_status;    // 040
    uint32 ctrl_global_interrupt_mask;      // 044

    // Unused words
    uint8 reserved2[0x200-0x48];

    // Per channel registers/state ram
    DmaChannelCfg chcfg[IUDMA_MAX_CHANNELS];// (200-3FF) Channel configuration
    union {
        DmaStateRam     s[IUDMA_MAX_CHANNELS];
        uint32          u32[4 * IUDMA_MAX_CHANNELS];
    } stram;                                // (400-5FF) state ram
} DmaRegs;

// DMA Buffer
typedef struct DmaDesc {
  uint16        length;                 // in bytes of data in buffer
#define          DMA_DESC_USEFPM    0x8000
#define          DMA_DESC_MULTICAST 0x4000
#define          DMA_DESC_BUFLENGTH 0x0fff
  uint16        status;                 // buffer status
#define          DMA_OWN                0x8000  // cleared by DMA, set by SW
#define          DMA_EOP                0x4000  // last buffer in packet
#define          DMA_SOP                0x2000  // first buffer in packet
#define          DMA_WRAP               0x1000  //
#define          DMA_PRIO               0x0C00  // Prio for Tx, BCM6362 only
#define          DMA_APPEND_BRCM_TAG    0x0200
#define          DMA_APPEND_CRC         0x0100
#define          USB_ZERO_PKT           (1<< 0) // Set to send zero length packet

  uint32        address;                // address of data
} DmaDesc;

// 16 Byte DMA Buffer, BCM6362 only
typedef struct {
  uint16        length;                 // in bytes of data in buffer
#define          DMA_DESC_USEFPM        0x8000
#define          DMA_DESC_MULTICAST     0x4000
#define          DMA_DESC_BUFLENGTH     0x0fff
  uint16        status;                 // buffer status
#define          DMA_OWN                0x8000  // cleared by DMA, set by SW
#define          DMA_EOP                0x4000  // last buffer in packet
#define          DMA_SOP                0x2000  // first buffer in packet
#define          DMA_WRAP               0x1000  //
#define          DMA_PRIO               0x0C00  // Prio for Tx
#define          DMA_APPEND_BRCM_TAG    0x0200
#define          DMA_APPEND_CRC         0x0100
#define          USB_ZERO_PKT           (1<< 0) // Set to send zero length packet

  uint32        address;                 // address of data
  uint32        control;
#define         GEM_ID_MASK             0x001F
  uint32        reserved;
} DmaDesc16;

*/

#endif // __BCM63XX_H__
