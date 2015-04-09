/** \file vp790_api_int.h
 * vp790_api_int.h
 *
 * Header file for the vp790 series API-II c files.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 9115 $
 * $LastChangedDate: 2011-11-15 15:25:17 -0600 (Tue, 15 Nov 2011) $
 */

#ifndef VP790_API_INT_H
#define VP790_API_INT_H

#define VP790_CID_END_OF_MESSAGE_BYTE   0xC0
#define VP790_ENCODE_BUF_SIZE           5

/**< Define the initial hook value to use when determining if the line status
 * has been changed from initialization. This must be an invalid value to force
 * a signaling register read for simple polled mode.
 */
#define VP790_HOOK_INIT_VAL 0xFF

/*
 * Define the mask that will report device busy if there is a currently active
 * event when the user is attempting to perform another "read" transaction.
 */
#define VP790_NO_MASK   0x0000

#define VP790_READ_RESPONSE_MASK (VP_LINE_EVID_LLCMD_RX_CMP \
                                | VP_LINE_EVID_RD_OPTION)

#define VP790_NONSUPPORT_FAULT_EVENTS   (VP_LINE_EVID_DC_FLT \
                                       | VP_LINE_EVID_AC_FLT \
                                       | VP_DEV_EVID_EVQ_OFL_FLT \
                                       | VP_DEV_EVID_WDT_FLT)

#define VP790_NONSUPPORT_SIGNALING_EVENTS   (VP_LINE_EVID_MTONE \
                                           | VP_LINE_EVID_US_TONE_DETECT \
                                           | VP_LINE_EVID_DS_TONE_DETECT \
                                           | VP_DEV_EVID_SEQUENCER)

#define VP790_NONSUPPORT_RESPONSE_EVENTS    (VP_DEV_EVID_BOOT_CMP \
                                           | VP_DEV_EVID_DNSTR_MBOX \
                                           | VP_LINE_EVID_GAIN_CMP)

#define VP790_NONSUPPORT_TEST_EVENTS    VP_EVCAT_TEST_MASK_ALL

#define VP790_NONSUPPORT_PROCESS_EVENTS VP790_NO_MASK

#define VP790_NONSUPPORT_FXO_EVENTS     VP_EVCAT_FXO_MASK_ALL

#define VP790_NONSUPPORT_PACKET_EVENTS  VP_EVCAT_PACKET_MASK_ALL

#define VP790_DEVTYPE_CMD   0x73
#define VP790_DEVTYPE_LEN   0x01

/* The calibrate command */
#define VP790_CALIBRATE_CMD     0x7C

/* Linecard parameters */
#define VP790_LINECARD_PARAM_WRT    0x22
#define VP790_LINECARD_PARAM_RD     0x23
#define VP790_LINECARD_PARAM_LEN    0x03

/* Codec enable disable commands  (Channel specific)*/
#define VP790_DEACTIVATE_CMD    0x00    /**< Deactivate codec */
#define VP790_ACTIVATE_CMD      0x0E    /**< Activate Codec */

/* Device Registers (Effects all device channels)*/
#define VP790_HW_RESET_CMD  0x04    /**< Hardware Reset */

#define VP790_DCR1_WRT      0x46    /**< Device config register 1 write */
#define VP790_DCR1_RD       0x47    /**< Device config register 1 read */
#define VP790_DCR1_LEN      0x01
#define VP790_DCR1_CMODE    0x10
#define VP790_DCR1_TTL      0x80

#define VP790_MCLK_CNT_WRT  0x46    /**< MCLK control reg = DCR1 */
#define VP790_MCLK_CNT_RD   0x47
#define VP790_MCLK_CNT_LEN  0x01

#define VP790_DCR2_WRT      0x36    /**< Device config register 2 write */
#define VP790_DCR2_RD       0x37    /**< Device config register 2 read */
#define VP790_DCR2_LEN      0x01

#define VP790_MCCR_WRT      0x30    /**< MCLK correction register write */
#define VP790_MCCR_RD       0x31    /**< MCLK correction register read */
#define VP790_MCCR_LEN      0x01

#define VP790_XR_CS_WRT     0x44    /**< Tx/Rx clock slot register write */
#define VP790_XR_CS_RD      0x45    /**< Tx/Rx clock slot register read */
#define VP790_XR_CS_LEN     0x01

#define VP790_GIO_DIR_WRT   0x38    /**< Device I/O direction register write */
#define VP790_GIO_DIR_RD    0x39    /**< Device I/O direction register read */
#define VP790_GIO_DIR_LEN   0x01
#define VP790_GIO_DIR_MASK  0x0F

#define VP790_GIO_DIR_OUTPUT        0x01
#define VP790_GIO_DIR_ALL_OUTPUT    0x0F

#define VP790_GIO_DATA_WRT  0x3A    /**< Device I/O data register write */
#define VP790_GIO_DATA_RD   0x3B    /**< Device I/O data register read */
#define VP790_GIO_DATA_LEN  0x01
#define VP790_GIO_DATA_MASK 0x0F
#define VP790_GIO_DATA_HIGH 0x01

#define VP790_GD_STAT_RD    0x3D    /**< Device device status register read */
#define VP790_GD_STAT_LEN   0x01
#define VP790_GD_STAT_PINT  0x80    /**< Pos battery fail bit. */
#define VP790_GD_STAT_LINT  0x40    /**< Low bat fail bit */
#define VP790_GD_STAT_HINT  0x20    /**< High bat fail bit */
#define VP790_GD_STAT_CFAIL 0x10    /**< Clock fail bit */

#define VP790_GD_MASK_WRT   0x34    /**< Device device mask register write */
#define VP790_GD_MASK_RD    0x35    /**< Device device mask register write */
#define VP790_GD_MASK_LEN   0x01
#define VP790_GD_MASK_PINT  0x80
#define VP790_GD_MASK_LINT  0x40
#define VP790_GD_MASK_HINT  0x20

#define VP790_GD_MASK_CLKFAIL   0x10

#define VP790_INTRPT_RD     0x3F    /**< Interrupt register read */
#define VP790_INTRPT_LEN    0x01
#define VP790_INTR_GLOB     0x80    /**< Device interrupt bit */
#define VP790_INTR_INT      0x40    /**< Device interrupt bit */

#define VP790_GLOB_CHAN_MASK    0x30    /**< Masks channel bits from Interrupt
                                         * register.
                                         */

#define VP790_INTR_HOOK     0x08    /**< Hook state bit */
#define VP790_INTR_GNK      0x04    /**< Ground Key State bit */
#define VP790_INTR_SIG      0x01    /**< Line specific signaling interrupt */

#define VP790_GSUP_RD       0x21    /**< Device supervision register read */
#define VP790_GSUP_LEN      0x01

#define VP790_HOOK_STATUS_RD  VP790_GSUP_RD
#define VP790_HOOK_STATUS_LEN VP790_GSUP_LEN

#define VP790_RCODE_RD      0x73    /**< Revision code register read */
#define VP790_RCODE_LEN     0x01
#define VP790_NO_OP         0x06

#define VP790_EC_WRT    0x4A    /**< Enable channel register Write */
#define VP790_EC_RD     0x4B    /**< Enable channel register Read */
#define VP790_EC_LEN    0x01

#ifndef VP790_EC_CH1
#define VP790_EC_CH1    0x01
#define VP790_EC_CH2    0x02
#define VP790_EC_CH3    0x04
#define VP790_EC_CH4    0x08
#endif

#define VP790_EC_STATE_AT_RESET     (VP790_EC_CH1 | VP790_EC_CH2 \
                                   | VP790_EC_CH3 | VP790_EC_CH4)

/* Test and Measurment Commands */
/* Batteries */
#define VP790_LOW_BATT_RD       0xAF    /**< Read low battery */
#define VP790_LOW_BATT_LEN      0x02
#define VP790_LOW_BATT_SCALE    3027    /**< Mutiplication factor for low
                                         * battery
                                         */

#define VP790_HIGH_BATT_RD      0xB1    /**< Read high battery */
#define VP790_HIGH_BATT_LEN     0x02
#define VP790_HIGH_BATT_SCALE   3027    /**< Mutiplication factor for high
                                         * battery
                                         */

#define VP790_POS_BATT_RD       0xB3    /**< Read positive battery */
#define VP790_POS_BATT_LEN      0x02
#define VP790_POS_BATT_SCALE    3027    /**< Mutiplication factor for positive
                                         * battery
                                         */
/* Test Filters */
#define VP790_TEST_FILT1_WRT    0xD6    /**< Write test filter 1 */
#define VP790_TEST_FILT1_RD     0xD7    /**< Read test filter 1 */
#define VP790_TEST_FILT1_LEN    0x0A
#define VP790_TEST_FILT2_WRT    0xD8    /**< Write test filter 1 */
#define VP790_TEST_FILT2_RD     0xD9    /**< Read test filter 1 */
#define VP790_TEST_FILT2_LEN    0x0A
#define VP790_TEST_INT_WRT      0xDA    /**< Write test integrator */
#define VP790_TEST_INT_RD       0xDB    /**< Read test integrator */
#define VP790_TEST_INT_LEN      0x0A
#define VP790_TSTR_RD           0xDD    /**< Read test results register */
#define VP790_TSTR_LEN          0x02

/* Channel specific commands */
#define VP790_SW_RESET_CMD      0x02    /**< Software reset */
#define VP790_RST_TO_NORM_CMD   0x08    /**< Reset to normal conditions */
#define VP790_SET_TO_TEST_CMD   0x0A    /**< Set to test conditions */
#define VP790_RST_FROM_TEST_CMD 0x0C    /**< Reset from test conditions */

#define VP790_TX_TS_WRT         0x40    /**< Transmit time slot write */
#define VP790_TX_TS_RD          0x41    /**< Transmit time slot read */
#define VP790_TX_TS_LEN         0x01
#define VP790_TX_TS_MASK        0x3F
#define VP790_TX_HWY_B          0x80

#define VP790_RX_TS_WRT         0x42    /**< Receive time slot write */
#define VP790_RX_TS_RD          0x43    /**< Receive time slot read */
#define VP790_RX_TS_LEN         0x01
#define VP790_RX_TS_MASK        0x3F
#define VP790_RX_HWY_B          0x80

#define VP790_PCMHWY_SEL        0x80    /**< The PCM Hwy select bit */

#define VP790_ISLIC_FB_TS_WRT   0x58    /**< Direct ISILC feedback timeslot
                                         * write
                                         */
#define VP790_ISLIC_FB_TS_RD    0x59    /**< Direct ISILC feedback timeslot
                                         * read
                                         */
#define VP790_ISLIC_FB_TS_LEN   0x01

#define VP790_ISLIC_CONT_WRT    0x5A    /**< Direct line control write */
#define VP790_ISLIC_CONT_RD     0x5B    /**< Direct line control read */
#define VP790_ISLIC_CONT_LEN    0x01

#define VP790_SLIC_STATE_WRT        0x56    /**< Line state write */
#define VP790_SLIC_STATE_RD         0x57    /**< Line state read */
#define VP790_SLIC_STATE_LEN        0x01
#define VP790_SLIC_HIGH_BAT_SELECT  0x80
#define VP790_SLIC_ST_HIGH_BATT     0x20
#define VP790_SLIC_ST_POS_BATT      0x10
#define VP790_SLIC_ST_POLREV        0x08
#define VP790_AUTO_BAT_SELECTION    VP790_SLIC_ST_POS_BATT
#define VP790_HIGH_BAT_SELECTION    VP790_SLIC_ST_HIGH_BATT
#define VP790_LOW_BAT_SELECTION     0x00

#define VP790_BOOST_BAT_SELECTION   (VP790_SLIC_ST_HIGH_BATT \
                                   | VP790_SLIC_ST_POS_BATT)

#define VP790_SLIC_ST_MASK          0x07
#define VP790_SLIC_ST_STANDBY       (0x00 | VP790_HIGH_BAT_SELECTION)
#define VP790_SLIC_ST_TIPOPEN       (0x01 | VP790_HIGH_BAT_SELECTION)
#define VP790_SLIC_ST_ACTIVE        0x02
#define VP790_SLIC_ST_TELETAX       0x03
#define VP790_SLIC_ST_RESERVED      0x04
#define VP790_SLIC_ST_OHT           (0x05 | VP790_HIGH_BAT_SELECTION)

#define VP790_SLIC_ST_DISCONNECT    (0x06 | VP790_HIGH_BAT_SELECTION)
#define VP790_SLIC_ST_RINGING       (0x07 | VP790_HIGH_BAT_SELECTION)

#define VP790_IO_REG_WRT            0x52    /**< I/O register write */
#define VP790_IO_REG_RD             0x53    /**< I/O register read */
#define VP790_IO_REG_LEN            0x01
#define VP790_IO_REG_RD1_MANUAL     0x40
#define VP790_IO_REG_RD2_AUTO       0x20

#define VP790_NO_UL_SIGREG_RD   0x4D    /**< Read w/o unlock signaling reg */
#define VP790_NO_UL_SIGREG_LEN  0x02

#define VP790_UL_SIGREG_RD      0x4F    /**< Read and unlock signaling reg */
#define VP790_UL_SIGREG_LEN     0x02
#define VP790_SIGREG_HOOK       0x80    /**< Masks for MSByte */
#define VP790_SIGREG_GNK        0x40
#define VP790_SIGREG_AST        0x20
#define VP790_SIGREG_ICON       0x10
#define VP790_SIGREG_TEMPA      0x08
#define VP790_SIGREG_CFPI       0x04
#define VP790_SIGREG_DCFAULT    0x02    /**< Masks for LSByte */
#define VP790_SIGREG_ACFAULT    0x01

#define VP790_CCR1_WRT          0x62    /**< write channel config register 1 */
#define VP790_CCR1_RD           0x63    /**< read channel config register 1 */
#define VP790_CCR1_LEN          0x01
#define VP790_LOOPBACK_WRT      VP790_CCR1_WRT  /**< Map the loop back register
                                                 * to CCR1
                                                 */

#define VP790_LOOPBACK_RD       VP790_CCR1_RD
#define VP790_LOOPBACK_LEN      VP790_CCR1_LEN

#define VP790_FULL_LOOPBACK     0x04    /**< FULL Digital Loopback */
#define VP790_PCM_LOOPBACK      0x20    /**< PCM or Timeslot loopback */
#define VP790_NO_LOOPBACK       0x00    /**< No loopback */
#define VP790_CUT_TXPATH        0x40
#define VP790_CUT_RXPATH        0x01
#define VP790_CCR1_DHP          0x02    /**< Transmit and Receive HPF Control */

#define VP790_CCR2_WRT      0x64    /**< write channel config register 2 */
#define VP790_CCR2_RD       0x65    /**< read channel config register 2 */
#define VP790_CCR2_LEN      0x01

#define VP790_GEN_CTRL_WRT    VP790_CCR2_WRT
#define VP790_GEN_CTRL_RD     VP790_CCR2_RD
#define VP790_GEN_CTRL_LEN    VP790_CCR2_LEN

#define VP790_GENB_EN           0x08
#define VP790_GENA_EN           0x04

#define VP790_START_TEST        0x00
#define VP790_ENABLE_FILT1      0x01
#define VP790_ENABLE_FILT2      0x02
#define VP790_REVERSE_SLOPE     0x40
#define VP790_TEST_VALID        0x80

#define VP790_CCR3_WRT      0x60    /**< write channel config register 3 */
#define VP790_CCR3_RD       0x61    /**< read channel config register 3 */
#define VP790_CCR3_LEN      0x01

#define VP790_CCR4_WRT      0x68    /**< write channel config register 4 */
#define VP790_CCR4_RD       0x69    /**< read channel config register 4 */
#define VP790_CCR4_LEN      0x01
#define VP790_EXT_RINGING   0x10
#define VP790_INT_RINGING   0x00
#define VP790_CCR4_RTM      0x08
#define VP790_CCR4_ZXR_DIS  0x04
#define VP790_CCR4_ARR_DIS  0x01

#define VP790_CODEC_REG_WRT     0x60    /**< Register holding compression
                                         * info
                                         */
#define VP790_CODEC_REG_RD      0x61    /**< Register holding compression info
                                         * read
                                         */
#define VP790_CODEC_REG_LEN     0x01
#define VP790_ALAW_CODEC        0x00    /**< a-Law compression is used */
#define VP790_ULAW_CODEC        0x40    /**< u-law compression is used */
#define VP790_LINEAR_CODEC      0x01    /**< Linear mode is used */

#define VP790_CODEC_COMPRESSION_MASK  (VP790_LINEAR_CODEC | VP790_ULAW_CODEC)

#define VP790_CCR5_WRT          0x6A
#define VP790_CCR5_RD           0x6B
#define VP790_CCR5_LEN          0x01

#define VP790_TELATAX_POLREV    0x10
#define VP790_MTR_ABRUPT        0x20
#define VP790_MTR_FREQ_16KHZ    0x40

/* CCR6 and CCR7 hold the interrupt mask  info for each channel. */
#define VP790_CCR6_WRT          0x6C
#define VP790_CCR6_RD           0x6D
#define VP790_CCR6_LEN          0x01
#define VP790_MASK_HOOK_INTR    0x80
#define VP790_MASK_GNK_INTR     0x40
#define VP790_MASK_AST_INTR     0x20
#define VP790_MASK_TEMPA_INTR   0x08
#define VP790_MASK_CFP_INTR     0x04
#define VP790_MASK_TEST_INTR    0x02

#define VP790_CCR7_WRT          0x6E
#define VP790_CCR7_RD           0x6F
#define VP790_CCR7_LEN          0x01
#define VP790_MASK_MMTONE       0x04
#define VP790_MASK_DCFLT        0x02
#define VP790_MASK_ACFLT        0x01

/* CCR 8 test tools */
#define VP790_CCR8_WRT          0x74
#define VP790_CCR8_RD           0x75
#define VP790_CCR8_LEN          0x01
#define VP790_CONNECT_VIN       0x00    /**< Connect Vin to analog input */
#define VP790_CONNECT_VILG      0x01    /**< Connect Vilg to analog input */
#define VP790_CONNECT_VIMT      0x20    /**< Connect Vimt to analog input */
#define VP790_CONNECT_VSAB      0x30    /**< Connect Vsab to analog input */

/* CCR9 test tools register */
#define VP790_CCR9_WRT          0x76
#define VP790_CCR9_RD           0x77
#define VP790_CCR9_LEN          0x01
#define VP790_MEAS_REQUEST      0x01    /**< Request a line measurement */
#define VP790_MATH_NONE         0x00    /**< No math in the test meas block */
#define VP790_MATH_ABS          0x02    /**< Absolute value */
#define VP790_MATH_SQUARE       0x04    /**< square X*X */
#define VP790_RANGE_1X          0x00    /**< Gain block set to 1X */
#define VP790_RANGE_16X         0x10    /**< Gain block set to 16X */
#define VP790_CONNECT_XMIT      0x00    /**< Connect xmit path to meas block */
#define VP790_CONNECT_DLG       0x20    /**< Connect longitudinal current to
                                         * meas block
                                         */
#define VP790_CONNECT_DMT       0x40    /**< Connect matallic current to meas
                                         * block
                                         */
#define VP790_CONNECT_DAB       0x60    /**< Connect Vab to meas block */
#define VP790_CONNECT_DIA       0x80    /**< Connect 0.5*(Ilg + Imt) to meas
                                         * block
                                         */
#define VP790_CONNECT_DIB       0xA0    /**< Connect 0.5*(Ilg - Imt) to meas
                                         * block
                                         */

/* loop supervision register */
#define VP790_LOOP_SUP_WRT      0xC2
#define VP790_LOOP_SUP_RD       0xC3
#define VP790_LOOP_SUP_LEN      0x08

#define VP790_LOOPSUP_CUR_STEPSIZE  78125ul
#define VP790_LOOPSUP_CUR_DIVISOR   100000ul
#define VP790_LOOPSUP_DSH_STEPSIZE  5ul
#define VP790_LOOPSUP_DSH_DIVISOR   10ul

#define VP790_DEBOUNCE_ACTIVE       0x20    /**< 10mS debounce in active */
#define VP790_DEBOUNCE_STBY         0x64    /**< 50mS debounce in stby */

#define VP790_LOOPSUP_OHM_STEPSIZE  6250L
#define VP790_LOOPSUP_OHM_DIVISOR   100L

/* DC feed coefficient register */
#define VP790_DC_FEED_WRT       0xC6
#define VP790_DC_FEED_RD        0xC7
#define VP790_DC_FEED_LEN       0x0E
#define VP790_DC_PROFILE_LENGTH 24  /**< DC Feed profile length */

/*
 * Signal generator registers. Signal generator A is used to generate the
 * ringing signal on tip/ring. Signal generator B is used to generate the
 * call progress (MF) tones.
 */
#define VP790_RINGER_PARAMS_WRT     0xD2
#define VP790_RINGER_PARAMS_RD      0xD3
#define VP790_RINGER_PARAMS_LEN     0x0A

#define VP790_SIGB_PARAMS_WRT       0xD4
#define VP790_SIGB_PARAMS_RD        0xD5
#define VP790_SIGB_PARAMS_LEN       0x08

#define VP790_SIGGEN_CONTROL_WRT    0x66
#define VP790_SIGGEN_CONTROL_RD     0x67
#define VP790_SIGGEN_CONTROL_LEN    0x01

#define VP790_CID_DATA_WRT    0x66
#define VP790_CID_DATA_RD     0x67
#define VP790_CID_DATA_LEN    0x01

#define VP790_SIGGEN_FSK_OFF        0x00

#define VP790_DISABLE_CID_NOW           0x00
#define VP790_DISABLE_CID_WHEN_CMPL     0xC0

/* The following registers hold the VAB, IMT, ILG, and RLOOP data. */
#define VP790_VAB_RD        0xA7    /**< Read the vab voltage */
#define VP790_VAB_LEN       0x02
#define VP790_VAB_SCALE     3113    /**< The VAB voltage scalling factor */

#define VP790_VIMT_RD       0xA9    /**< Read the vimt voltage for metallic
                                     * current
                                     */
#define VP790_VIMT_LEN      0x02
#define VP790_VIMT_SCALE    3113    /**< The VIMT voltage scalling factor */

#define VP790_VILG_RD       0xAB    /**< Read the vilg voltage for longitudinal
                                     * current
                                     */
#define VP790_VILG_LEN      0x02
#define VP790_VILG_SCALE    3113    /**< The Vilg voltage scalling factor */

#define VP790_VRLOOP_RD     0xAD    /**< Read the Vrloop voltage for loop
                                     * resistance.
                                     */
#define VP790_VRLOOP_LEN    0x02
#define VP790_VRLOOP_SCALE  3113    /**< The Vrloop voltage scalling factor */

#define VP790_METERING_PEAK_RD  0xB7
#define VP790_METERING_PEAK_LEN 0x02

/* filters */
#define VP790_B1_FILTER_WRT     0x86    /**< B Filter Coefficients */
#define VP790_B1_FILTER_RD      0x87
#define VP790_B1_FILTER_LEN     0x0E
#define VP790_B2_FILTER_WRT     0x96
#define VP790_B2_FILTER_RD      0x97

#define VP790_B2_FILTER_LEN_DUAL    0x0E    /**< B2 Length for Dual-ISLAC */
#define VP790_B2_FILTER_LEN_QUAD    0x06    /**< B2 Length for Quad-ISLAC */

#define VP790_X_FILTER_WRT  0x88    /**< X Filter Coefficients */
#define VP790_X_FILTER_RD   0x89
#define VP790_X_FILTER_LEN  0x0C

#define VP790_DISN_WRT      0xCA    /**< DISN AX/AR */
#define VP790_DISN_RD       0xCB
#define VP790_DISN_LEN      0x01

#define VP790_GX_GAIN_WRT   0x80    /**< GX gainblock */
#define VP790_GX_GAIN_RD    0x81
#define VP790_GX_GAIN_LEN   0x02

#define VP790_GR_GAIN_WRT   0x82    /**< GR gainblock */
#define VP790_GR_GAIN_RD    0x83
#define VP790_GR_GAIN_LEN   0x02

#define VP790_MTLL_WRT      0xD0    /**< Meter Target and Limit Levels */
#define VP790_MTLL_RD       0xD1
#define VP790_MTLL_LEN      0x01

typedef enum vp790_deviceProfileParams {
    VP790_DEV_PROFILE_PCLK_MSB = 6,
    VP790_DEV_PROFILE_PCLK_LSB = 7,
    VP790_DEV_PROFILE_DEVCFG1 = 8,
    VP790_DEV_PROFILE_MCLK_CORR = 9,
    VP790_DEV_PROFILE_CLOCK_SLOT = 10,
    VP790_DEV_PROFILE_MAX_EVENTS = 11,
    VP790_DEV_PROFILE_TICKRATE_MSB = 12,
    VP790_DEV_PROFILE_TICKRATE_LSB = 13,
    VP790_DEV_PROFILE_LINECARD_PARAM_CMD = 20,
    VP790_DEV_PROFILE_LINECARD_PARAM_DATA0 = 21,
    VP790_DEV_PROFILE_LINECARD_PARAM_DATA1 = 22,
    VP790_DEV_PROFILE_LINECARD_PARAM_DATA2 = 23,
    VP790_DEV_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
}vp790_deviceProfileParams;

#define DEV_PROFILE_MAX_EVENTS 9

typedef enum vp790_fxo_dialingProfileParams {
    VP790_FXO_DIALING_PROFILE_DTMF_ON_MSB = 6,
    VP790_FXO_DIALING_PROFILE_DTMF_ON_LSB = 7,
    VP790_FXO_DIALING_PROFILE_DTMF_OFF_MSB = 8,
    VP790_FXO_DIALING_PROFILE_DTMF_OFF_LSB = 9,
    VP790_FXO_DIALING_PROFILE_FLASH_HOOK_MSB = 10,
    VP790_FXO_DIALING_PROFILE_FLASH_HOOK_LSB = 11,
    VP790_FXO_DIALING_PROFILE_PULSE_BREAK = 12,
    VP790_FXO_DIALING_PROFILE_PULSE_MAKE = 13,
    VP790_FXO_DIALING_PROFILE_INTERDIGIT_MSB = 14,
    VP790_FXO_DIALING_PROFILE_INTERDIGIT_LSB = 15,
    VP790_FXO_DIALING_PROFILE_RING_PERIOD_MAX = 16,
    VP790_FXO_DIALING_PROFILE_RING_PERIOD_MIN = 17,
    VP790_FXO_DIALING_PROFILE_RING_VOLTAGE_MIN = 18,
    VP790_FXO_DIALING_PROFILE_DISC_VOLTAGE_MIN = 19,
    VP790_FXO_DIALING_PROFILE_LIU_THRESHOLD_MIN = 20,
    VP790_FXO_DIALING_PROFILE_RSVD = 21,
    VP790_FXO_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} vp790_fxo_dialingProfileParams;

typedef enum vp790_toneProfileParams {
    VP790_TONE_MPI_CMD = 6,
    VP790_TONE_PROFILE_DC_BIAS = 7,
    VP790_TONE_PROFILE_START = 8,
    VP790_TONE_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} vp790_toneProfileParams;

/* Api Info */
/* Loop supervision offset into DC FEED profile */
#define VP790_LOOPSUP_OFFSET (VP790_DC_FEED_LEN + 3)

VpStatusType
Vp790MakeLineObject(
    VpTermType termType,
    uint8 channelId,
    VpLineCtxType *pLineCtx,
    void *pLineObj,
    VpDevCtxType *pDevCtx);

/**< Vp790 Initalization Function Prototypes */
VpStatusType
Vp790CalCodec(
    VpLineCtxType *pLineCtx,
    VpDeviceCalType mode);

VpStatusType
Vp790CalCodecInt(
    VpDevCtxType *pDevCtx);

VpStatusType
Vp790InitRing(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pCadProfile,
    VpProfilePtrType pCidProfile);

VpStatusType
Vp790InitCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    uint8p pCidData);

/**< Vp790 Control Function Prototypes */
VpStatusType
Vp790SetDTMFGenerators(
    VpLineCtxType *pLineCtx,
    VpCidGeneratorControlType mode,
    VpDigitType digit);

VpStatusType
Vp790SetLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);

VpStatusType
Vp790SetRelayState(
    VpLineCtxType *pLineCtx,
    VpRelayControlType rState);

VpStatusType
Vp790SendCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    VpProfilePtrType pCidProfile,
    uint8p pCidData);

VpStatusType
Vp790ContinueCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    uint8p pCidData);

VpStatusType
Vp790SetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *value);

VpStatusType
Vp790DeviceIoAccess(
    VpDevCtxType *pDevCtx,
    VpDeviceIoAccessDataType *pDeviceIoData);

#ifndef VP790_SIMPLE_POLLED_MODE
VpStatusType
Vp790VirtualISR(
    VpDevCtxType *pDevCtx);
#endif

VpStatusType
Vp790ApiTick(
    VpDevCtxType *pDevCtx,
    bool *pEventStatus);

#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
VpStatusType
Vp790LowLevelCmd(
    VpLineCtxType *pLineCtx,
    uint8 *pCmdData,
    uint8 len,
    uint16 handle);
#endif

VpStatusType
Vp790EncodeData(
    VpLineCtxType *pLineCtx,
    uint8 *pMessageData);

/**< Vp790 Status and Query Function Prototypes */
bool
Vp790GetEvent(
    VpDevCtxType *pDevCtx,
    VpEventType *pEvent);

VpStatusType
Vp790GetDeviceStatus(
    VpDevCtxType *pDevCtx,
    VpInputType input,
    uint32 *pDeviceStatus);

VpStatusType
Vp790GetLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType *pCurrentState,
    VpLineStateType *pPreviousState);

VpStatusType
Vp790FlushEvents(
    VpDevCtxType *pDevCtx);

VpStatusType
Vp790GetResults(
    VpEventType *pEvent,
    void *pResults);

VpStatusType
Vp790GetLoopCond(
    VpLineCtxType *pLineCtx,
    uint16 handle);

VpStatusType
Vp790GetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    uint16 handle);

bool
Vp790FindSoftwareInterrupts(
    VpDevCtxType *pDevCtx);

bool
Vp790ContinueCalibrate(
    VpDevCtxType *pDevCtx);

bool
Vp790ServiceTimers(
    VpDevCtxType *pDevCtx);

bool
Vp790ServiceType1Int(
    VpDevCtxType *pDevCtx);

bool
Vp790ServiceType2Int(
    VpLineCtxType *pLineCtx,
    uint8 intReg);

bool
Vp790ServiceType3Int(
    VpLineCtxType *pLineCtx);

#endif  /* VP790_API_INT_H */






