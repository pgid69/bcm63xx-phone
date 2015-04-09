/** \file vp880_api_int.h
 * vp880_api_int.h
 *
 * Header file for the vp880 series API-II c files.
 *
 * Copyright (c) 2012, Microsemi Corporation
 *
 * $Revision: 11380 $
 * $LastChangedDate: 2014-04-10 12:55:59 -0500 (Thu, 10 Apr 2014) $
 */

#ifndef VP880_API_INT_H
#define VP880_API_INT_H

#include "vp_api_event.h"
#include "vp_api_option.h"

/**< Define the initial hook value to use when determining if the line status
 * has been changed from initialization. This must be an invalid value to force
 * a signaling register read for simple polled mode.
 */
 #define VP880_HOOK_INIT_VAL 0xFF

/*****************************************************************************
 * Generic Timers used inside the API. Calibration timers are documented with
 * other values used for calibration (e.g., scaling factors)
 *****************************************************************************/
/**< VP880_MAX_RING_DET_PERIOD
 * Used on FXO lines to detect when the specified minimum ringing frequency
 * cannot be supported by the silicon programming alone. The API uses this
 * to jump to "Low Frequency Detection" mode.
 */
#define VP880_MAX_RING_DET_PERIOD   (0x3F)

/**< VP880_RING_TRIP_DEBOUNCE
 * Debounce time in ms when an off-hook/ring trip is detected to prevent
 * additional hook activity from being reported (real, or not real). At
 * the end of this timer the signaling register is read and further hook
 * based processing proceeds as normal.
 */
#define VP880_RING_TRIP_DEBOUNCE    (100)

/**< VP880_SPEEDUP_HOLD_TIME
 * Time used to delay state transitions for external components (e.g., CHL) to
 * stabilize. A longer duration set here is not as critical as being too short.
 * If too short, state transitions while CHL is charging without speedup control
 * being disabled cause significant line transients. Lab test show ~35ms is
 * generally sufficient, so this (50ms) duration could be reduced but to ensure
 * that in presence of 10ms tickrate that the duration is always at least ~40ms.
 */
#define VP880_SPEEDUP_HOLD_TIME (50)

/**< Ground Start Timers
 * The Ground Start Time is the time to hold tip at ground during a Tip-Open to Feed transition.
 * The Ground Start Debounce time is the hook/ground key debounce time after making the final
 * line changes when transitioning fro Tip-Open to Feed condition.
 */
#define VP880_GND_START_TIME        (210)
#define VP880_GND_START_DEBOUNCE    (30)

/**< Internal Test Termination Timers
 * The initial settings of the fake test load state take a while to settle the
 * DC values of the battery.  Wait this time before disabling tip and ring bias
 * to make tip and ring outputs high impedance, and tend to pull to battery.
 * The SHORT time is used for ABS and non-fixed tracking devices.
 * The LONG time is used for fixed trackers.
 */
#define VP880_INTERNAL_TESTTERM_SETTLING_TIME_SHORT (50)    /* ms */
#define VP880_INTERNAL_TESTTERM_SETTLING_TIME_LONG  (200)   /* ms */

/**< VP880_PING_TIME
 * Timer used for silicon revision <= VC when changiing from Disconnect to Feed condition to
 * workaround a polairty reversal issue. The API puts the line into Tip Open for this duration
 * before making the final state change to the target feed condition.
 */
#define VP880_PING_TIME         (30)

/**< Dial Pulse Distortion Correction Timers
 * There is 2ms on-hook delay in the silicon when using a "fast" supply, and an additional 4ms
 * when using a slow supply. This value is applied to the on/off-hook timer values which are in
 * 125us increments
 */
#define VP880_PULSE_DETECT_ADJUSTMENT_SHORT (16)
#define VP880_PULSE_DETECT_ADJUSTMENT_LONG  (48)

/* Convenient conversions from 125us to 1ms for Dial Pulse Correction timers */
#define VP880_OFFHOOK_EVENT_DELAY_SHORT     (VP880_PULSE_DETECT_ADJUSTMENT_SHORT / 8)
#define VP880_OFFHOOK_EVENT_DELAY_LONG      (VP880_PULSE_DETECT_ADJUSTMENT_LONG / 8)

/************************************************************************************************
 * Low Power Mode Timers                                                                        *
 *                                                                                              *
 *  - VP880_TRACKER_DISABLE_TIME is used to delay final line changes when entering LPM Standby  *
 *  - VP880_INVERT_BOOST_DISABLE_TIME used for Disconnect Entry time for Tracking Ringing Mode  *
 *  - VP880_FIXED_TRACK_DISABLE_TIME used for Disconnect Entry time for Fixed Ringing Mode      *
 *  - VP880_PWR_SWITCH_DEBOUNCE_FUT used for Leaky Line Test for Tracking Ringing Mode          *
 *  - VP880_PWR_SWITCH_DEBOUNCE_FXT used for Leaky Line Test for Fixed Ringing Mode             *
 *                                                                                              *
 * Details below                                                                                *
 ************************************************************************************************/
    /**< Power Switch Debounce for Leaky Line Test
     * These timer are used to delay monitoring of hook status when transition in and out of LPM
     * during the Resistive Leaky Line test. For an off-hook condition while in Low Power Mode
     * Standby an intial off-hook event will be delayed by the time specified (+ internal silicon
     * hook debounce).
     *
     *  - VP880_PWR_SWITCH_DEBOUNCE_FUT: Time used when Tracking Mode Ringing is detected
     *  - VP880_PWR_SWITCH_DEBOUNCE_FXT: Time used when Fixed Mode Ringing is detected
     *    Note that by default these times are the same, but in general Tracking Mode could be a
     *    bit shorter than Fixed Ringing mode.
     *
     * If the Application requires operation using a 5REN load as specified by GR57 "Ringing
     * Capability" test, then these values should be set to 130 (for 130ms debounce). The default
     * value of 90 (for 90ms debounce) is to work under real-world conditions where a 5REN load is
     * defined by FCC Part 68 as (6.05uf + 482ohm).
     *
     * For all conditions, these values must be greater than or equal to VP880_TRACKER_DISABLE_TIME.
     * The VP880_TRACKER_DISABLE_TIME value is used to mask hook activity while enter/exiting LPM.
     * Since the hook status itself is used during the leaky line test, the algorithm may not run
     * correctly if the hook masking is longer than the leaky line test itself.
     */
    #define VP880_PWR_SWITCH_DEBOUNCE_FUT   (90)
    #define VP880_PWR_SWITCH_DEBOUNCE_FXT   VP880_PWR_SWITCH_DEBOUNCE_FUT

    /**< VP880_TRACKER_DISABLE_TIME
     * This value is used by Vp880SetLP() when putting the line into Low Power Mode. It is the time
     * required to wait for the supply and line to become stable enough to feed the loop through the
     * 150Kohm resistor with the SLIC in Disconnect state (i.e., weak feed conditions). The worst
     * case transition is from polarity reversal to VP_LINE_STANDBY into a 5REN load. Note that this
     * value must be less than or equal to the times used to manage leaky line test (i.e., both
     * VP880_PWR_SWITCH_DEBOUNCE_FXT and VP880_PWR_SWITCH_DEBOUNCE_FUT). Otherwise, if during the
     * leaky line test the algorithm detects on-hook while in LPM (a good condition), this timer
     * will continue to run and delay the next time the leaky line test can run.
     *
     * Presumably, if Tracking and Fixed Ringing modes have different debounce times - Fixed Ringing
     * would be the longer of the two.
     */
    #define VP880_TRACKER_DISABLE_TIME  VP880_PWR_SWITCH_DEBOUNCE_FXT

    /**< LPM Disconnect Entry Timers
     * The issue when entering LPM Disconnect is the supply (caps) feeding the load have to be fully
     * discharged before the line can be set to Disconnect and Supplies disabled. Otherwise, the
     * circuit will have some voltag remaining.
     *
     * These timers are selected based on Fixed/Tracking Mode ringing and therefore suffer the same
     * issue as the "PWR_SWITCH_DEBOUNCE" timers. Read above for details.
     *
     *     - VP880_INVERT_BOOST_DISABLE_TIME: the time used for Tracked Mode Ringing
     *     - VP880_FIXED_TRACK_DISABLE_TIME: the time used for Fixed Mode Ringing
     */
    #define VP880_INVERT_BOOST_DISABLE_TIME (150)
    #define VP880_FIXED_TRACK_DISABLE_TIME  (400)

/**< Define the number of charge iterations for the ringing transition */
#define IN_RUSH_CYCLE_TOTAL 5   /* 5 = normal, 0 to disable the feature */
#define IN_RUSH_CYCLE_END   1

/**< Event Bit-Masks
 * Convenient/Readable values used to manage VE880 VP-API-II Events. These may only be changed
 * when updating the VE880 to support an existing or new event. Incorrect modifications may
 * result in VE880 API being unable to generate certain events.
 */

/**< VP880_READ_RESPONSE_MASK
 * This value is used to block additional read operations that would result in response data being
 * generated. It (i.e., one of these bits) is set when an event is generated that requires
 * VpGetResults() to be called. It creaetes the VCP Response Mailbox-like behavior where the
 * mailbox is "busy" until read.
 */
#define VP880_READ_RESPONSE_MASK    (VP_LINE_EVID_LLCMD_RX_CMP \
                                   | VP_LINE_EVID_RD_OPTION \
                                   | VP_LINE_EVID_GAIN_CMP)

/**< VP880_NONSUPPORT_YYY_EVENTS
 * These masks are created to overwrite any other attempt to clear event masks that are
 * not supported by the VE880 API. Notice that the list (being inverted) are those events that
 * ARE supported by the VE880 API. This ensures correct behavior when new events are added without
 * changing this file (except for new events that are supported by the VE880 API)
 */
#define VP880_NONSUPPORT_FAULT_EVENTS       (~(VP_DEV_EVID_BAT_FLT \
                                             | VP_DEV_EVID_CLK_FLT \
                                             | VP_LINE_EVID_THERM_FLT \
                                             | VP_LINE_EVID_RES_LEAK_FLT \
                                             | VP_LINE_EVID_DC_FLT))

/**< VP880_NONSUPPORT_SIGNALING_EVENTS
 * Note1: Inclusion of VP_LINE_EVID_DTMF_DIG which is not detected by the VP-API-II. The event is
 * generated when the application calls VpSysDtmfDigitDetected() typically used for Type II CID.
 * However, the application can call this function for any other reason so from an VP-API-II
 * perspective, this event is technically supported.
 *
 * Note1: VP_LINE_EVID_EXTD_FLASH is technically supported but can only be generated if
 * EXTENDED_HOOK_FLASH is set to #define in vp_api_cfg.h
 */
#define VP880_NONSUPPORT_SIGNALING_EVENTS   (~(VP_LINE_EVID_HOOK_OFF \
                                             | VP_LINE_EVID_HOOK_ON \
                                             | VP_LINE_EVID_GKEY_DET \
                                             | VP_LINE_EVID_GKEY_REL \
                                             | VP_LINE_EVID_FLASH \
                                             | VP_LINE_EVID_STARTPULSE \
                                             | VP_LINE_EVID_PULSE_DIG \
                                             | VP_DEV_EVID_TS_ROLLOVER \
                                             | VP_LINE_EVID_EXTD_FLASH \
                                             | VP_LINE_EVID_DTMF_DIG))

    /**< VP880_FXS_SIGNALING_EVENTS
     * This is used to mask off events on the FXO line for events that are only FXS type. It should
     * include just about all signaling events except for the timestamp rollover.
     */
    #define VP880_FXS_SIGNALING_EVENTS  (VP_LINE_EVID_HOOK_OFF \
                                       | VP_LINE_EVID_HOOK_ON \
                                       | VP_LINE_EVID_GKEY_DET \
                                       | VP_LINE_EVID_GKEY_REL \
                                       | VP_LINE_EVID_FLASH \
                                       | VP_LINE_EVID_STARTPULSE \
                                       | VP_LINE_EVID_PULSE_DIG \
                                       | VP_LINE_EVID_EXTD_FLASH \
                                       | VP_LINE_EVID_BREAK_MAX)

#define VP880_NONSUPPORT_RESPONSE_EVENTS    (~(VP_LINE_EVID_LLCMD_TX_CMP \
                                             | VP_LINE_EVID_LLCMD_RX_CMP \
                                             | VP_LINE_EVID_RD_OPTION \
                                             | VP_EVID_CAL_CMP \
                                             | VP_EVID_CAL_BUSY \
                                             | VP_LINE_EVID_GAIN_CMP \
                                             | VP_DEV_EVID_DEV_INIT_CMP \
                                             | VP_LINE_EVID_LINE_INIT_CMP \
                                             | VP_DEV_EVID_IO_ACCESS_CMP \
                                             | VP_LINE_EVID_GAIN_CMP))

#define VP880_NONSUPPORT_TEST_EVENTS        (~(VP_LINE_EVID_TEST_CMP \
                                             | VP_LINE_EVID_ABORT))

#define VP880_NONSUPPORT_PROCESS_EVENTS     (~(VP_LINE_EVID_MTR_CMP \
                                             | VP_LINE_EVID_MTR_ABORT \
                                             | VP_LINE_EVID_CID_DATA \
                                             | VP_LINE_EVID_RING_CAD \
                                             | VP_LINE_EVID_SIGNAL_CMP \
                                             | VP_LINE_EVID_TONE_CAD))

#define VP880_NONSUPPORT_FXO_EVENTS         (~(VP_LINE_EVID_RING_ON \
                                             | VP_LINE_EVID_RING_OFF \
                                             | VP_LINE_EVID_LIU \
                                             | VP_LINE_EVID_LNIU \
                                             | VP_LINE_EVID_FEED_DIS \
                                             | VP_LINE_EVID_FEED_EN \
                                             | VP_LINE_EVID_DISCONNECT \
                                             | VP_LINE_EVID_RECONNECT \
                                             | VP_LINE_EVID_POLREV))

/**< Command Set Values
 * The #define values here MUST be exactly per the Command Set. Unpredictable behavior will
 * occur for any error. DO NOT CHANGE unless you know exactly what you're doing and even
 * then you still shouldn't change these.
 *
 * NOTE: In cases where a value is surrounded by #ifdef value /n #define value (yy) /n #endif
 * it means the specific value is probably used in either the device or line object definition
 * and therefore defined when vp880_api.h is included - prior to this file being included in
 * general.
 */
/**< Revision and Product Code Command info */
#define VP880_DEVTYPE_RD        0x73
#define VP880_DEVTYPE_CMD       VP880_DEVTYPE_RD
#define VP880_DEVTYPE_LEN       0x02    /**< RCN = 1st Byte, PCN = 2nd Byte */

#define VP880_NO_OP_WRT         0x06
#define VP880_NO_OP             VP880_NO_OP_WRT

#define VP880_REV_VA            0x01
#define VP880_REV_VC            0x02
#define VP880_REV_JE            0x04

/**< GLOBAL REGISTERS (Effects all SLAC device channels) */
#define VP880_HW_RESET_WRT      0x04    /**< Hardware Reset */
#define VP880_HW_RESET_CMD      VP880_HW_RESET_WRT
#define VP880_HW_RESET_LEN      0x00

#define VP880_TEST_REG1_WRT     0x30
#define VP880_TEST_REG1_RD      0x31
#define VP880_TEST_REG1_LEN     0x01
#define VP880_TREG1_PLLBYP_MASK 0x80
#define VP880_TREG1_CSEL_MASK   0x40
#define VP880_TREG1_TMODE_MASK  0x0F

#define VP880_TEST_REG2_WRT     0x32
#define VP880_TEST_REG2_RD      0x33
#define VP880_TEST_REG2_LEN     0x01

/**< Transmit/Receive Clock Slot Command info */
#define VP880_XR_CS_WRT         0x44    /**< Tx/Rx clock slot register write */
#define VP880_XR_CS_RD          0x45    /**< Tx/Rx clock slot register read */
#define VP880_XR_CS_LEN         0x01

#define VP880_TX_CSLOT_WRT      0x44    /**< Tx clock slot register write */
#define VP880_TX_CSLOT_RD       0x45    /**< Tx clock slot register read */
#define VP880_TX_CSLOT_LEN      0x01

#define VP880_RX_CSLOT_WRT      0x44    /**< Rx clock slot register write */
#define VP880_RX_CSLOT_RD       0x45    /**< Rx clock slot register read */
#define VP880_RX_CSLOT_LEN      0x01

/**< Bit definitions for Transmit/Receive Clock Slot Register */
#define VP880_TX_SLOT_MASK      0x07
#define VP880_RX_SLOT_MASK      0x38

/**< Transmit Edge Command info */
#define VP880_TX_EDGE_WRT       0x44    /**< Tx clock slot register write */
#define VP880_TX_EDGE_RD        0x45    /**< Tx clock slot register read */
#define VP880_TX_EDGE_LEN       0x01

/**< Bit definitions for Transmit Edge Register */
#define VP880_TX_EDGE_MASK      0x40

/**< Device Configuration Command info */
#define VP880_DCR_WRT           0x46
#define VP880_DCR_RD            0x47
#define VP880_DCR_LEN           0x01

/**< Bit definitions for Device Configuration */
#define VP880_DCR_INTMODE_MASK      0x80
#define VP880_DCR_DPCLK_MASK        0x40
#define VP880_DCR_PCM_SMODE_MASK    0x20

/**< Master Clock Command info */
#define VP880_MCLK_CNT_WRT          0x46
#define VP880_MCLK_CNT_RD           0x47
#define VP880_MCLK_CNT_LEN          0x01

/**< Bit definitions for Master Clock Command */
#define VP880_MCLK_CLKSEL_MASK      0x0F

/**< Operating Mode Command info */
#define VP880_OP_MODE_WRT           0x4A
#define VP880_OP_MODE_RD            0x4B
#define VP880_OP_MODE_LEN           0x01

/**< Bit definitions for Operating Mode Command */
#define VP880_TEST_MODE_MASK        0x80
#define VP880_RBE_MODE_MASK         0x40
#define VP880_WBAND_MODE_MASK       0x20
#define VP880_DSPBYP_MODE_MASK      0x10

/**< Signaling Register Command info */
#define VP880_NO_UL_SIGREG_RD       0x4D    /**< Read w/o unlock signaling reg */
#define VP880_NO_UL_SIGREG_LEN      0x02

#define VP880_UL_SIGREG_RD          0x4F    /**< Read w/unlock signaling reg */

#ifndef VP880_UL_SIGREG_LEN
#define VP880_UL_SIGREG_LEN         0x02
#endif

/**< Interrupt Mask Register info */
#define VP880_INT_MASK_WRT      0x6C
#define VP880_INT_MASK_RD       0x6D
#define VP880_INT_MASK_LEN      0x02

/**< Bit definitions for Signaling Register Commands */
    /**< Byte 1 */
#define VP880_CFAIL_MASK        0x80

#define VP880_POL1_MASK         0x40
#define VP880_OCALMY_MASK       0x40

#define VP880_POH1_MASK         0x20
#define VP880_TEMPA1_MASK       0x20

#define VP880_IO2_1_MASK        0x10
#define VP880_CAD1_INT_MASK     0x08
#define VP880_DISC1_MASK        0x04
#define VP880_CID1_RDY_MASK     0x04
#define VP880_RING1_DET_MASK    0x02
#define VP880_GNK1_MASK         0x02
#define VP880_LIU1_MASK         0x01
#define VP880_HOOK1_MASK        0x01

    /**< Byte 2 (only those different from the first Byte need definition) */
#define VP880_CDAT_MASK         0x80
#define VP880_OCALMZ_MASK       0x40

/**< Revision and Product Code Command info */
/**< PCN Interpretation (sort of):
 *     SLIC2    = xxx1 xxxx
 *     SLIC1    = xxxx 1xxx
 *     HV       = xxxx x1xx
 *     CODEC    = xxxx xx1x
 *     WideBand = xxxx xxx1
 *
 * The silicon will in general comply with the above PCN table. But there are a few instances
 * where the API performs additional register reads to determine the silicon PCN.
 */
#define VP880_RCN_PCN_RD        0x73
#define VP880_RCN_PCN_LEN       0x02    /**< RCN = 1st Byte, PCN = 2nd Byte */
#define VP880_RCN_LOCATION      0x00
#define VP880_PCN_LOCATION      0x01

#define VP880_WIDEBAND_MASK     0x01    /**< Wideband if set in PCN */
#define VP880_CODEC_MASK        0x02    /**< CODEC exists if set in PCN */
#define VP880_HV_MASK           0x04    /**< HV device if set in PCN */
#define VP880_TWO_CHAN_MASK     0x80

#define VP880_SLIC_MASK         0x38    /**< Indicates if, and type of SLIC */
#define VP880_2CH_TRACKER       0x38
#define VP880_2CH_ABS           0x30
#define VP880_TRACK_FXS_FXO     0x28
#define VP880_2CH_FXO           0x20
#define VP880_ABS_FXS_FXO       0x00

#define VP880_IS_IT_WB  (VP880_CODEC_MASK | VP880_WIDEBAND_MASK)

/**< VP-API-II Reported PCNs (VpGetDeviceInfo())
 * Simply put - if it isn't in this list, the API doesn't know about it. If the API
 * doesn't know about a specific PCN, VpInitDevice() will return error.
 */
/**< PCNs for One Channel Devices */
#define VP880_FIRST_OF_ONE_CHANNEL_DEVICES  (0x05)

#define VP880_DEV_PCN_88010  (0x06)     /**< FXO */
#define VP880_DEV_PCN_88111  (0x0A)     /* FXS-Tracker */
#define VP880_DEV_PCN_88116  (0x0B)     /* FXS-Tracker - Wideband */
#define VP880_DEV_PCN_88131  (0x0E)     /* FXS-Tracker */
#define VP880_DEV_PCN_88136  (0x0F)     /* FXS-Tracker - Wideband */

#define VP880_LAST_OF_ONE_CHANNEL_DEVICES  (0x10)

#define VP880_TWO_CHANNNEL_MASK       (0x80)
#define VP880_TRACKER_MASK            (0x08)

/**< PCNs for Two Channel Devices */
#define VP880_DEV_PCN_88211     (0xBA)     /* 2FXS-Tracker */
#define VP880_DEV_PCN_88216     (0xBB)     /* 2FXS-Tracker - Wideband */
#define VP880_DEV_PCN_88221     (0xB2)     /* 2FXS-ABS */
#define VP880_DEV_PCN_88226     (0xB3)     /* 2FXS-ABS - Wideband */

#define VP880_DEV_PCN_88286_QFN (0xB4)      /* 2FXS-ABS, HV, WB, No Test Load */
#define VP880_DEV_PCN_88266_QFN (0xB0)      /* 2FXS-ABS, LV, WB, No Test Load */

#define VP880_DEV_PCN_88231     (0xBE)     /* 2FXS-Tracker */
#define VP880_DEV_PCN_88236     (0xBF)     /* 2FXS-Tracker - Wideband */

#define VP880_DEV_PCN_88241     (0xB6)     /* 2FXS-ABS */
#define VP880_DEV_PCN_88246     (0xB7)     /* 2FXS-ABS - Wideband */

#define VP880_DEV_PCN_88311     (0xAA)     /* FXO/FXS-Tracker */
#define VP880_DEV_PCN_88331     (0xAE)     /* FXO/FXS-Tracker */

#define VP880_DEV_PCN_88506     (0xBC)     /* 2FXS-Tracker - Wideband Split Package*/

#define VP880_DEV_PCN_88536     (0xFF)     /* 2FXS-Tracker - Wideband Split Package, ZSI */
#define VP880_DEV_PCN_88264     (0xF3)     /* 2FXS-ABS - Wideband, ZSI */


/**< Internal Revision Command info */
#define VP880_REV_INFO_RD       0xB9
#define VP880_REV_INFO_LEN      0x01

/**< Switching Regulator Parameters */
#define VP880_REGULATOR_PARAM_WRT   0xE4
#define VP880_REGULATOR_PARAM_RD    0xE5

#ifndef VP880_REGULATOR_PARAM_LEN
#define VP880_REGULATOR_PARAM_LEN   0x03
#endif

#define VP880_REGULATOR_TRACK_INDEX     0x00
#define VP880_REGULATOR_FIXED_RING_SWZ  0x08
#define VP880_REGULATOR_FIXED_RING_SWY  0x02
#define VP880_REGULATOR_FIXED_RING      VP880_REGULATOR_FIXED_RING_SWY
#define VP880_REGULATOR_INVERT_BOOST    0x20

#define VP880_FLOOR_VOLTAGE_BYTE    0x01
#define VP880_FLOOR_VOLTAGE_MASK    0x1F

#define VP880_BAT_CALIBRATION_WRT   0xE8
#define VP880_BAT_CALIBRATION_RD    0xE9
#define VP880_BAT_CALIBRATION_LEN   0x02
#define VP880_BAT_CAL_SWCAL_MASK    0x38
#define VP880_BAT_CAL_SWCAL_SIGN    0x20

#define VP880_CONVERTER_MODE_WRT    0xE4
#define VP880_CONVERTER_MODE_RD     0xE5
#define VP880_CONVERTER_MODE_LEN    0x03

/**< Switching Regulator Parameters bit definitions */
#define VP880_MODE_MASK             0x20
#define VP880_FLYBACK_MODE          0x00
#define VP880_BUCKBOOST_MODE        0x20

#define VP880_BSI_MASK              0xC0
#define VP880_BSI_LY_MX_HZ          0x00
#define VP880_BSI_LY_MZ_HX          0x40
#define VP880_BSI_LX_MY_HZ          0x80
#define VP880_BSI_DEFAULT           VP880_BSI_LY_MX_HZ

#define VP880_ZRING_TRACK_MASK      0x08
#define VP880_ZRING_TRACK_EN        0x00
#define VP880_ZRING_TRACK_DIS       0x08

#define VP880_YRING_TRACK_MASK      0x02
#define VP880_YRING_TRACK_EN        0x00
#define VP880_YRING_TRACK_DIS       0x02

#define VP880_SWY_AUTOPOWER_INDEX   0x01
#define VP880_SWY_AUTOPOWER_MASK    0x80
#define VP880_SWY_AUTOPOWER_EN      0x00
#define VP880_SWY_AUTOPOWER_DIS     0x80

#define VP880_SWZ_AUTOPOWER_INDEX   0x02
#define VP880_SWZ_AUTOPOWER_MASK    0x80
#define VP880_SWZ_AUTOPOWER_EN      0x00
#define VP880_SWZ_AUTOPOWER_DIS     0x80

#define VP880_VOLTAGE_MASK          0x1F
#define VP880_SWY_LOCATION          1
#define VP880_SWZ_LOCATION          2
#define VP880_VOLTAGE_60V           0x0B

/**< Internal Switching Regulator Parameters Command info */
#define VP880_INT_SWREG_PARAM_WRT   0xF6
#define VP880_INT_SWREG_PARAM_RD    0xF7

#ifndef VP880_INT_SWREG_PARAM_LEN
#define VP880_INT_SWREG_PARAM_LEN   0x06
#endif

#define VP880_SWREG_RING_V_BYTE     2
#define VP880_SWREG_FLOOR_V_BYTE    1

/**< Switching Regulator Control Command info */
#define VP880_REGULATOR_CTRL_WRT    0xE6
#define VP880_REGULATOR_CTRL_RD     0xE7

#ifndef VP880_REGULATOR_CTRL_LEN
#define VP880_REGULATOR_CTRL_LEN    0x01
#endif

/**************************************************************************************************
 * Calibration Constants
 **************************************************************************************************/
/**< Calibration Scaling Data
 * These macros are primarily used for debug output used to convert PCM data into readable format.
 * Since they're used in Calibration Related Mathematical computations, force all values to "long"
 * type just to avoid loss of data in case of conversion from 32-bit to 16-bit format. These values
 * are based on silicon performance and Data Sheet accuracies.
 */
#define VP880_V_SCALE           (10000L)    /**< Scale to put in 10mV */
#define VP880_V_1V_SCALE        (13654L)    /**< Based on 7.324V per LSB from PCM Data */
#define VP880_V_1V_RANGE        (100L)      /**< To scale back to PCM range used */
#define VP880_V_1V_PCM          (136L)      /**< Used to detect voltage decay */

/**************************************************************************************************
 * Calibration - Common Timer Macros
 ***********************************
 * These macros are used to set the VP-API-II device/line timers during the calibration process. All
 * values are in mS and generally assumed to be implemented as enforced as a minimum time delay. The
 * specific values have been determined by a combination of silicon filter lengths, line/load time
 * time constants, and target Data Sheet tolerances. In some cases, changing these times can have
 * drastically negative affects while in other cases making significant changes can have almost no
 * affect.
 **************************************************************************************************/
/**< Time to wait before starting calibration after major state change */
#define VP880_MIN_CAL_WAIT_TIME (200)

/**< Time to between changing the converter configuration register and taking a measurmenet -
 * assumes the battery and loop is stable the entire time.
 */
#define VP880_CAL_ABV_DELAY     (20)

/**************************************************************************************************
 * Calibration - ABS Only Timer Macros
 **************************************************************************************************/
/**< VP880_CAL_ABV_ABS_INIT
 * Time to wait at the end of calibration for device/lines to recover before allowing the
 * Application to take (back) over. Previously 2500ms but was unnecessarily long.
 */
#define VP880_CAL_ABV_ABS_INIT  (1000)

/**< VP880_CAL_ABV_SAMPLE, VP880_CAL_ABV_SAMPLE_ERR and VP880_CAL_ABV_SAMPLE_MAX
 * During ABS Battery Calibration, VP880_CAL_ABV_SAMPLE is the time between taking samples waiting
 * for battery to decay. Note that this original value was 250ms, but in most cases was longer than
 * needed.
 *
 * The macro VP880_CAL_ABV_SAMPLE_ERR is the difference in PCM data between two consecutive samples
 * seperated in time "VP880_CAL_ABV_SAMPLE" where the "ABS offset" algorithm converges (i.e., where:
 * "offset" = PCM Data, since Vbat has been disabled and appears to have settled).
 *
 * If after "VP880_CAL_ABV_SAMPLE_MAX" number of tries to converge the Vbat doesn't meet the above
  * criteria, an average of some number of the last set of samples is taken as the "offset".
 */
#define VP880_CAL_ABV_SAMPLE        (100)   /* mS, previously 250ms */
#define VP880_CAL_ABV_SAMPLE_ERR    (3)     /* Raw PCM Data, meaning this is @ 7.324mV */
#define VP880_CAL_ABV_SAMPLE_MAX    (40)    /* Can't converge after 40 tries, give up! */

/**************************************************************************************************
 * Calibration - Tracker Only Macros
 **************************************************************************************************/
/**< VP880_TRACKER_BAT_OFFSET
 * Fixed offset when taking battery offset measurement in Tracker Calibration
 */
#define VP880_TRACKER_BAT_OFFSET    (41)    /**< Represents 300mv at PCM scale */

/*
 * Note that the script uses delta error of 10, but experiments with some
 * silicon starting at known good VAS values shows slightly higher delta on the
 * first 2-3 steps. Since we're starting higher (taking into account minimum
 * allowable VAS from IMT cal and battery correction), it is best to allow a
 * slightly higher delta to minimize the possibility we'll set VAS one or two
 * steps too high.
 */

/*
 * The MAX error has to be more than the IMT offset and less than ILA. The
 * exact value is not important because the current change for each VAS step is
 * approximately 0.75V / 200ohm = 3.75mA, the change coming out of saturation.
 * It is best to keep this far from the IMT offset value unless the IMT offset
 * itself is used to make a precise IMT measurement during VAS calibration.
 * Safe value is 10mA.
 * When the IMT current during VAS calibration exceeds this value, it will
 * indicate failure -- even if the previous value is approximately the same.
 * That will cause either a VAS or battery increase and the algorithm continues.
 */
#define VP880_VAS_OVERHEAD      (3000)  /**< 3V for Temperature Variation */
#define VP880_VAS_MIN_OVERHEAD  (1500)  /**< 1.5V overhead for min VAS */

#define VP880_VAS_MAX           (14250) /**< 14.25V per silicon */
#define VP880_VAS_START         (3000)  /**< 3.00V per silicon */
#define VP880_VAS_STEP          (750)   /**< 0.75V per silicon */

#define VP880_BAT_CAL_MAX       (3750)  /**< 3.75 corresponding to Max Battery Cal setting. This
                                         * values is used in VAS calibration, which is why it's in
                                         * the same scale.
                                         */
#define VP880_BAT_CAL_STEP      (1250)  /**< 1.25 corresponding to Battery Cal step sizes. This
                                         * values is used in VAS calibration, which is why it's
                                         * in the same scale.
                                         */


/* Convenient names for index values into calibration arrays */
#define VP880_NORM_POLARITY (0)
#define VP880_REV_POLARITY  (1)
#define VP880_NUM_POLARITY  (2)

bool
Vp880UpdateCalValue(
    VpLineCtxType *pLineCtx);

void
Vp880AbvMakeAdjustment(
    Vp880DeviceObjectType *pDevObj,
    int16 *targetVoltY,
    int16 *targetVoltZ);

#ifdef VP880_TRACKER_SUPPORT
void
Vp880BatteryCalAdjust(
    Vp880DeviceObjectType *pDevObj,
    uint8 ecVal);
#endif

#define VP880_VOC_SHIFT_MASK        0x40
#define VP880_ILA_MASK              0x1F
#define VP880_ILA_INDEX             0x01

#define VP880_ABV_MASK              0x1F
#define VP880_VAS_MASK_LO           0x03
#define VP880_VAS_MASK_HI           0xC0

/**< Switching Regulator Control bit definitions */
#define VP880_SWY_MODE_MASK         0x03
#define VP880_SWY_OFF               0x00
#define VP880_SWY_LP                0x01
#define VP880_SWY_MP                0x02
#define VP880_SWY_HP                0x03
#define VP880_SWYZ_LP               0x05

#define VP880_SWZ_MODE_MASK         0x0C
#define VP880_SWZ_OFF               0x00
#define VP880_SWZ_LP                0x04
#define VP880_SWZ_MP                0x08
#define VP880_SWZ_HP                0x0C

/**< CHANNEL REGISTERS (must set EC register first) */

/**< Channel Enable Command info */
#define VP880_EC_WRT                0x4A
#define VP880_EC_RD                 0x4B
#define VP880_EC_LEN                0x01

#ifndef VP880_EC_CH1
#define VP880_EC_CH1                0x01
#define VP880_EC_CH2                0x02
#endif

#define VP880_EC_STATE_AT_RESET     0x03

/**< Bit definitions for Channel Enable Command */
#define VP880_EC_BITS_MASK          0x03

#define VP880_DSP_BYPASS            0x10
#define VP880_WIDEBAND_MODE         0x20
#define VP880_TEST_MODE_ENABLE      0x80

/**< Voice Path Gain Command info */
#define VP880_VP_GAIN_WRT           0x50
#define VP880_VP_GAIN_RD            0x51
#ifndef VP880_VP_GAIN_LEN
#define VP880_VP_GAIN_LEN           0x01
#endif
#define VP880_DEFAULT_VP_GAIN       0x00

/**< Bit definitions for Voice Path Gain Commands */
#define VP880_AX_MASK           0x40
#define VP880_AR_MASK           0x30
#define VP880_AR_0DB_LOSS       0x00
#define VP880_AR_6DB_LOSS       0x10
#define VP880_AR_6DB_GAIN       0x20
#define VP880_AR_RSVD1          0x30
#define VP880_DR_LOSS_MASK      0x08    /**< Digital Receive Loss = 0dB (0) or
                                         * 6.02dB (1)
                                         */

/**< IO Data Command info */
#define VP880_IODATA_REG_WRT    0x52    /**< I/O register write */
#define VP880_IODATA_REG_RD     0x53    /**< I/O register read */
#define VP880_IODATA_REG_LEN    0x01
#define VP880_IODATA_IO6        0x20
#define VP880_IODATA_IO5        0x10
#define VP880_IODATA_IO4        0x08
#define VP880_IODATA_IO3        0x04
#define VP880_IODATA_IO2        0x02
#define VP880_IODATA_IO1        0x01

/**< Bit definitions for IO Data Command */
#define VP880_IODATA_BITS_MASK  0x3F    /**< Lowest two bits always available */

/**< IO Direction Command info */
#define VP880_IODIR_REG_WRT     0x54    /**< I/O direction register write */
#define VP880_IODIR_REG_RD      0x55    /**< I/O direction register read */
#define VP880_IODIR_REG_LEN     0x01

/**< Bit definitions for IO Direction Command */
#define VP880_MAX_PINS_PER_LINE (6)

#define VP880_IODIR_IO11_MASK   0x01
#define VP880_IODIR_IO12_MASK   0x02
#define VP880_IODIR_IO1_MASK    0x03
#define VP880_IODIR_IO2_MASK    0x04
#define VP880_IODIR_IO3_MASK    0x08
#define VP880_IODIR_IO4_MASK    0x10
#define VP880_IODIR_IO5_MASK    0x20
#define VP880_IODIR_IO6_MASK    0x40
#define VP880_IODIR_EXPDT_MASK  0x80    /**< External Period detector connected
                                         * to IO4
                                         */

#define VP880_IODIR_IO1_INPUT       0x00
#define VP880_IODIR_IO1_OUTPUT      0x01
#define VP880_IODIR_IO1_OPEN_DRAIN  0x02

#define VP880_IODIR_IO2_OUTPUT      0x04
#define VP880_IODIR_IO3_OUTPUT      0x08
#define VP880_IODIR_IO4_OUTPUT      0x10
#define VP880_IODIR_IO5_OUTPUT      0x20
#define VP880_IODIR_IO6_OUTPUT      0x40

/**< System State Command info */
#define VP880_SYS_STATE_WRT     0x56
#define VP880_SYS_STATE_RD      0x57
#define VP880_SYS_STATE_LEN     0x01

#define VP880_SLIC_STATE_WRT    VP880_SYS_STATE_WRT
#define VP880_SLIC_STATE_RD     VP880_SYS_STATE_RD
#define VP880_SLIC_STATE_LEN    VP880_SYS_STATE_LEN

/**< Command (bit) definitions for FXS System State */
#define VP880_SS_RING_EXIT_MASK     0x80
#define VP880_SS_METERING_MASK      0x40
#define VP880_SS_ACTIVATE_MASK      0x20    /**< (1) = CODEC Active, (0) =
                                             * Deactive
                                             */

#define VP880_SS_POLARITY_MASK      0x10    /**< (0) = Normal, (1) = Reverse */
#define VP880_SS_STATE_MASK         0x3F
#define VP880_SS_LINE_FEED_MASK     0x0F

#define VP880_SS_DISCONNECT         0x00
#define VP880_SS_TIP_OPEN           0x01
#define VP880_SS_RING_OPEN          0x02
#define VP880_SS_ACTIVE             (0x03 | VP880_SS_ACTIVATE_MASK)

#define VP880_SS_ACTIVE_POLREV      (0x03 | VP880_SS_ACTIVATE_MASK \
                                          | VP880_SS_POLARITY_MASK)

#define VP880_SS_IDLE               0x04
#define VP880_SS_IDLE_POLREV        (0x04 | VP880_SS_POLARITY_MASK)

#define VP880_SS_FEED_BALANCED_RINGING      0x07
#define VP880_SS_FEED_UNBALANCED_RINGING    0x0A

#define VP880_SS_LONGITUDINAL_TEST  (0x05 | VP880_SS_ACTIVATE_MASK)
#define VP880_SS_METALLIC_TEST      (0x06 | VP880_SS_ACTIVATE_MASK)
#define VP880_SS_BALANCED_RINGING   (VP880_SS_FEED_BALANCED_RINGING | VP880_SS_ACTIVATE_MASK)
#define VP880_SS_LOW_GAIN           (0x08 | VP880_SS_ACTIVATE_MASK)
#define VP880_SS_RSVD_0             (0x09 | VP880_SS_ACTIVATE_MASK)
#define VP880_SS_UNBALANCED_RINGING (VP880_SS_FEED_UNBALANCED_RINGING | VP880_SS_ACTIVATE_MASK)

#define VP880_SS_BALANCED_RINGING_PR    (VP880_SS_BALANCED_RINGING \
                                       | VP880_SS_POLARITY_MASK)

#define VP880_SS_UNBALANCED_RINGING_PR  (VP880_SS_UNBALANCED_RINGING \
                                       | VP880_SS_POLARITY_MASK)

#define VP880_SS_ACTIVE_MID_BAT     (0x0B | VP880_SS_ACTIVATE_MASK)
#define VP880_SS_ACTIVE_MID_BAT_PR  (VP880_SS_ACTIVE_MID_BAT \
                                    | VP880_SS_POLARITY_MASK)

#define VP880_SS_SHUTDOWN        0x0F

#define VP880_SS_RSVD_1             (0x0C | VP880_SS_ACTIVATE_MASK)
#define VP880_SS_RSVD_2             (0x0D | VP880_SS_ACTIVATE_MASK)
#define VP880_SS_RSVD_3             (0x0E | VP880_SS_ACTIVATE_MASK)
#define VP880_SS_RSVD_4             (0x0F | VP880_SS_ACTIVATE_MASK)

/**< FXO State controlled by General Purpose I/O */
#define VP880_FXO_STATE_WRT    0x52
#define VP880_FXO_STATE_RD     0x53
#define VP880_FXO_STATE_LEN    0x01

#define VP880_FXO_ACTIVATE_CODEC  0x20
#define VP880_FXO_SUPERVISION_EN  0x09

#define VP880_SS_FXO_OHT             0x00  /**< I/O-1 = Low, I/O-2 = Low */
#define VP880_SS_FXO_LOOP_CLOSED     0x01  /**< I/O-1 = High, I/O-2 = Low */
#define VP880_SS_FXO_LOOP_OPEN       0x02  /**< I/O-1 = Low, I/O-2 = High */

/**< Command (bit) definitions for FX0 System State */
#define VP880_SS_SHUTDOWN_MASK      0x00
#define VP880_SS_DEACTIVATED_MASK   0x09
#define VP880_SS_ACTIVATED_MASK     0x29

/**< Operating Functions Command info */
#define VP880_OP_FUNC_WRT       0x60
#define VP880_OP_FUNC_RD        0x61

#ifndef VP880_OP_FUNC_LEN
#define VP880_OP_FUNC_LEN       0x01
#endif

/**< Bit definitions for Operating Functions Command */
#define VP880_ENABLE_GR     0x20
#define VP880_ENABLE_GX     0x10
#define VP880_ENABLE_X      0x08
#define VP880_ENABLE_R      0x04
#define VP880_ENABLE_Z      0x02
#define VP880_ENABLE_B      0x01
#define VP880_ENABLE_LOADED_COEFFICIENTS 0x3F
#define VP880_DEFAULT_OP_FUNC_MODE 0x00

/**< Bit definitions for Codec Compression Command */

/* Note:  If Linear Mode is selected, u-Law/A-Law selection is ignored */
#define VP880_CODEC_COMPRESSION_MASK    0xC0
#define VP880_ALAW_CODEC        0x00    /**< a-Law compression is used */
#define VP880_ULAW_CODEC        0x40    /**< u-law compression is used */
#define VP880_LINEAR_CODEC      0x80    /**< Linear mode is used */

/**< System State Configuration Command info */
#define VP880_SS_CONFIG_WRT     0x68
#define VP880_SS_CONFIG_RD      0x69
#define VP880_SS_CONFIG_LEN     0x01

/*< Automatic Clock Fault Switching */
#define VP880_ACFS_MASK     0x20
#define VP880_ACFS_EN       0x20
#define VP880_ACFS_DIS      0x00

/**< Bit definitions for System State Configuration Command */
/**< Auto Thermal Fault Switching */
#define VP880_ATFS_MASK     0x10
#define VP880_ATFS_EN       0x10
#define VP880_ATFS_DIS      0x00

/**< Zero Cross Ring Entry/Exit */
#define VP880_ZXR_MASK      0x08
#define VP880_ZXR_EN        0x00
#define VP880_ZXR_DIS       0x08

/**< Smooth Polarity Reversal */
#define VP880_SMOOTH_PR_MASK    0x04
#define VP880_SMOOTH_PR_EN      0x04
#define VP880_SMOOTH_PR_DIS     0x00

/**< Automatic System State Control */
#define VP880_AUTO_SSC      0x02
#define VP880_AUTO_SSC_EN   0x00
#define VP880_AUTO_SSC_DIS  0x02

/**< Automatic Battery Switch Control for ABS */
#define VP880_ABS                       0x01
#define VP880_AUTO_BAT_SWITCH_DIS       0x01
#define VP880_AUTO_BAT_SWITCH_EN        0x00

/**< Automatic Battery Shutdown for Tracker */
#define VP880_AUTO_BAT_SHUTDOWN_EN      0x01
#define VP880_AUTO_BAT_SHUTDOWN_DIS     0x00

/**< Operating Conditions Command info */
#define VP880_OP_COND_WRT         0x70
#define VP880_OP_COND_RD          0x71

#ifndef VP880_OP_COND_LEN
#define VP880_OP_COND_LEN         0x01
#endif

/**< Map the loop back register to operating conditions register */
#define VP880_LOOPBACK_WRT        VP880_OP_COND_WRT
#define VP880_LOOPBACK_RD         VP880_OP_COND_RD
#define VP880_LOOPBACK_LEN        VP880_OP_COND_LEN

/**< Bit definitions for Operating Conditions Command */
#define VP880_TX_PATH_MASK      0x80
#define VP880_CUT_TXPATH        0x80
#define VP880_TXPATH_EN         0x00
#define VP880_TXPATH_DIS        0x80

#define VP880_RX_PATH_MASK      0x40
#define VP880_CUT_RXPATH        0x40
#define VP880_RXPATH_EN         0x00
#define VP880_RXPATH_DIS        0x40

#define VP880_HIGH_PASS_MASK    0x20
#define VP880_HIGH_PASS_EN      0x00
#define VP880_HIGH_PASS_DIS     0x20

#define VP880_LOWER_RX_GAIN_MASK    0x10
#define VP880_RX_GAIN_6DB_LOSS      0x10
#define VP880_RX_GAIN_0DB_LOSS      0x00

#define VP880_INTERFACE_LOOPBACK_EN 0x04
#define VP880_1KHZ_TONE_ON          0x01

#define VP880_OPCOND_RSVD_MASK      (0x0A | VP880_1KHZ_TONE_ON)
#define VP880_NORMAL_OP_COND_MODE   0x00

/**< GX Filter Command info */
#define VP880_GX_GAIN_WRT       0x80
#define VP880_GX_GAIN_RD        0x81
#define VP880_GX_GAIN_LEN       0x02

/**< GR Filter Command info */
#define VP880_GR_GAIN_WRT       0x82
#define VP880_GR_GAIN_RD        0x83
#define VP880_GR_GAIN_LEN       0x02

/**< B Filter (FIR) Coefficients */
#define VP880_B1_FILTER_WRT     0x86
#define VP880_B1_FILTER_RD      0x87
#define VP880_B1_FILTER_LEN     0x0E

/**< B Filter (IIR) Coefficients */
#define VP880_B2_FILTER_WRT     0x96
#define VP880_B2_FILTER_RD      0x97
#define VP880_B2_FILTER_LEN     0x02

/**< X Filter Coefficients */
#define VP880_X_FILTER_WRT      0x88
#define VP880_X_FILTER_RD       0x89
#define VP880_X_FILTER_LEN      0x0C

/**< R Filter Coefficients */
#define VP880_R_FILTER_WRT      0x8A
#define VP880_R_FILTER_RD       0x8B
#define VP880_R_FILTER_LEN      0x0E

/**< Z Filter (FIR) Coefficients */
#define VP880_Z1_FILTER_WRT     0x98
#define VP880_Z1_FILTER_RD      0x99
#define VP880_Z1_FILTER_LEN     0x0A

/**< Z Filter (IIR) Coefficients */
#define VP880_Z2_FILTER_WRT     0x9A
#define VP880_Z2_FILTER_RD      0x9B
#define VP880_Z2_FILTER_LEN     0x05

/**< Converter Configuration */
#define VP880_CONV_CFG_WRT      0xA6
#define VP880_CONV_CFG_RD       0xA7

#ifndef VP880_CONV_CFG_LEN
#define VP880_CONV_CFG_LEN      0x01
#endif

/**< Converter Configuration bit definitions */
#define VP880_CONV_CONNECT_BITS 0x0F
#define VP880_METALLIC_AC_V     0x00
#define VP880_SWITCHER_Y        0x01
#define VP880_SWITCHER_Z        0x02
#define VP880_XBR               0x03
#define VP880_TIP_TO_GND_V      0x04
#define VP880_RING_TO_GND_V     0x05
#define VP880_METALLIC_DC_V     0x06
#define VP880_METALLIC_DC_I     0x07
#define VP880_LONGITUDINAL_DC_I 0x08
#define VP880_CALIBRATION_I     0x09
#define VP880_VOICE_DAC         0x0A
#define VP880_NO_CONNECT        0x0B
#define VP880_CC_RSVD1          0x0C
#define VP880_LOW_TIP_TO_GND_V  0x0D
#define VP880_LOW_RING_TO_GND_V 0x0E
#define VP880_CC_RSVD2          0x0F
#define VP880_CC_RATE_MASK      0x70
#define VP880_CC_500HZ_RATE     0x40
#define VP880_CC_1KHZ_RATE      0x30
#define VP880_CC_2KHZ_RATE      0x20
#define VP880_CC_4KHZ_RATE      0x10
#define VP880_CC_8KHZ_RATE      0x00
#define VP880_CC_RATE_BITS      0x70

#define VP880_ILA_SCALE_1MA     (568)
#define VP880_ILA_SCALE_2MA     (2 * VP880_ILA_SCALE_1MA)
#define VP880_ILA_SCALE_18MA    (9 * VP880_ILA_SCALE_2MA)

/**< Loop Supervision Command info */
#define VP880_LOOP_SUP_WRT      0xC2
#define VP880_LOOP_SUP_RD       0xC3

#ifndef VP880_LOOP_SUP_LEN
#define VP880_LOOP_SUP_LEN      0x04
#endif

#define VP880_LOOP_SUP_THRESH_BYTE      0x00
                                                         /* Bits [7:6] = RSVD */
#define VP880_GKEY_THRESH_MASK          0x38             /* Bits [5:3], 0-42mA, step = 6mA */
#define VP880_SWHOOK_THRESH_MASK        0x07             /* Bits [2:0], 8-15mA, step = 1mA */

#define VP880_LOOP_SUP_DEBOUNCE_BYTE    0x01
#define VP880_GKEY_DEBOUNCE_MASK        0xE0             /* Bits [7:5], 0-28ms, step = 4ms */
#define VP880_SWHOOK_DEBOUNCE_MASK      0x1F             /* Bits [4:0], 0-62ms, step = 2ms */

#define VP880_LOOP_SUP_RT_MODE_BYTE     0x02
#define VP880_LOOP_SUP_RT_MODE_TYPE     0x80             /* 1 = 'AC' Ring Trip - requires no Bias */
#define VP880_LOOP_SUP_RT_MODE_THRSHLD  0x7F             /* 0 - 63.5mA, step = 0.5mA */

#define VP880_LOOP_SUP_RING_LIM_BYTE    0x03
                                                         /* Bits [7:5] = RSVD */
#define VP880_LOOP_SUP_RING_LIM_MASK    0x1F             /* Bits [4:0], 50-112mA, step = 2mA */

/* FXO Bit Definitions for Loop Supervision Register */
#define VP880_LIU_DBNC_INDEX    0x01
#define VP880_LIU_DBNC_MASK     0x1F

#define VP880_RING_PERIOD_MIN_INDEX     0x02
#define VP880_RING_PERIOD_1MS           0x04

#define VP880_LOOP_SUP_LIU_THRESH_BYTE  0x00
#define VP880_LOOP_SUP_LIU_THRESH_BITS  0x07
#define VP880_LOOP_SUP_LIU_16V          0x00

/**< Usefull variables and bit definitions for Loop Supervision */
#define VP880_LOOPSUP_CUR_DIVISOR     1ul
#define VP880_LOOPSUP_CUR_STEPSIZE    1ul

#define VP880_LOOPSUP_TSH_DIVISOR     VP880_LOOPSUP_CUR_DIVISOR
#define VP880_LOOPSUP_TSH_STEPSIZE    VP880_LOOPSUP_CUR_STEPSIZE

#define VP880_LOOPSUP_TGK_DIVISOR     1ul
#define VP880_LOOPSUP_TGK_STEPSIZE    3ul

#define VP880_LOOPSUP_DGK_DIVISOR     1ul
#define VP880_LOOPSUP_DGK_STEPSIZE    4ul

#define VP880_LOOPSUP_DSH_DIVISOR     1ul
#define VP880_LOOPSUP_DSH_STEPSIZE    2ul

#define VP880_LOOPSUP_RTTH_DIVISOR    10ul
#define VP880_LOOPSUP_RTTH_STEPSIZE   5ul

#define VP880_LOOPSUP_IRL_DIVISOR     1ul
#define VP880_LOOPSUP_IRL_STEPSIZE    2ul

#define VP880_LOOPSUP_TDIS_DIVISOR    10ul
#define VP880_LOOPSUP_TDIS_STEPSIZE   14ul

#define VP880_LOOPSUP_TLIU_DIVISOR    1ul
#define VP880_LOOPSUP_TLIU_STEPSIZE   11ul

#define VP880_LOOPSUP_DDIS_DIVISOR    1ul
#define VP880_LOOPSUP_DDIS_STEPSIZE   4ul

#define VP880_LOOPSUP_DLIU_DIVISOR    1ul
#define VP880_LOOPSUP_DLIU_STEPSIZE   2ul

#define VP880_LOOPSUP_TMIN_DIVISOR    100ul
#define VP880_LOOPSUP_TMIN_STEPSIZE   25ul

#define VP880_LOOPSUP_TMAX_DIVISOR    100ul
#define VP880_LOOPSUP_TMAX_STEPSIZE   25ul

/**< Command to control how Ring Trip will work */
#define VP880_RINGTRIP_MODE_WRT     0xC2
#define VP880_RINGTRIP_MODE_RD      0xC3
#define VP880_RINGTRIP_MODE_LEN     0x04

/**< Bit definitions for Ring Trip Control (AC or DC) */
#define VP880_RING_TRIP_DC      0x00
#define VP880_RING_TRIP_AC      0x80

#define VP880_RING_DETECT_PERIOD_ONLY   0x80

/**< DC Feed Command info */
#define VP880_DC_FEED_WRT    0xC6
#define VP880_DC_FEED_RD     0xC7

#ifndef VP880_DC_FEED_LEN
#define VP880_DC_FEED_LEN    0x02
#endif
#define VP880_DC_CAL_VOC_DIS        0x10

/* Position in DC Profile specifying VOC and ILA */
#define VP880_VOC_PROFILE_POSITION 12
#define VP880_ILA_PROFILE_POSITION 13
#define VP880_HOOK_HYST_POSITION   14

/**< Usefull variables and bit definitions for DC Feed */
#define VP880_LONG_IMP_MASK     0x20
#define VP880_LONG_IMP_50       0x20
#define VP880_LONG_IMP_100      0x00

/* First Byte */
#define VP880_VOC_INDEX         0x00
#define VP880_VOC_LOW_RANGE     0x40
#define VP880_VOC_MASK          0x1C    /* 0x40 is Range (36-57, 12-33) bit */
#define VP880_VOC_VALUE_MASK    (VP880_VOC_MASK | VP880_VOC_LOW_RANGE)
#define VP880_VOC_57V           0x1C
#define VP880_VOC_54V           0x18
#define VP880_VOC_51V           0x14
#define VP880_VOC_48V           0x10
#define VP880_VOC_45V           0x0C
#define VP880_VOC_42V           0x08
#define VP880_VOC_39V           0x04
#define VP880_VOC_36V           0x00

#define VP880_VAS_MSB_LOC       0x00
#define VP880_VAS_MSB_MASK      0x03

#define VP880_VAS_LSB_LOC       0x01
#define VP880_VAS_LSB_MASK      0xC0

#define VP880_VAS_CONVERSION(msb, lsb)  (750*(((msb << 2) & 0xC) | ((lsb >> 6) & 0x3))+3000)

/**< Digital Impedance Scaling Command info */
#define VP880_DISN_WRT  0xCA
#define VP880_DISN_RD   0xCB

#ifndef VP880_DISN_LEN
#define VP880_DISN_LEN  0x01
#endif

#define VP880_DEFAULT_DISN_GAIN 0x00

/**< Usefull variables and bit definitions for DISN */
#define VP880_DISN_DIVISOR      10000ul
#define VP880_DISN_STEPSIZE     78ul

/**< Read Transmit PCM/Test Data Command info */
#define VP880_TX_PCM_DATA_RD        0xCD
#define VP880_TX_PCM_DATA_LEN       0x02

/**< Read Transmit PCM/Test Buffer Command info */
#define VP880_TX_PCM_BUFF_RD        0xCF
#define VP880_TX_PCM_BUFF_LEN       0x0E

#define VP880_TX_BUF_LEN_MASK           0x70
#define VP880_TX_BUF_LEN_INDEX          0
#define VP880_TX_BUF_HOOK_MSB_MASK      0x0F
#define VP880_TX_BUF_HOOK_MSB_INDEX     0
#define VP880_TX_BUF_HOOK_LSB_MASK      0xFF
#define VP880_TX_BUF_HOOK_LSB_INDEX     1
#define VP880_TX_BUF_HOOK_CHAN1_MASK    0x3F
#define VP880_TX_BUF_HOOK_CHAN2_MASK    0xC0

/**< Device Mode Command info */
#define VP880_DEV_MODE_WRT          0x5E
#define VP880_DEV_MODE_RD           0x5F

#ifndef VP880_DEV_MODE_LEN
#define VP880_DEV_MODE_LEN          0x01
#endif

#define VP880_DEV_MODE_TEST_DATA    0x01
#define VP880_DEV_MODE_CHAN_MASK    0x02
#define VP880_DEV_MODE_CHAN0_SEL    0x00
#define VP880_DEV_MODE_CHAN1_SEL    0x02

/**< Metering Parameters Command info */
#define VP880_METERING_PARAM_WRT    0xD0
#define VP880_METERING_PARAM_RD     0xD1
#define VP880_METERING_PARAM_LEN    0x04

/**< Metering Parameters bit definitions */
#define VP880_METERING_FREQ_MASK    0x80
#define VP880_METERING_FREQ_12K     0x00
#define VP880_METERING_FREQ_16K     0x80

#define VP880_METERING_LIM_MASK     0x7F
#define VP880_METER_LIM_DIVISOR     1000ul
#define VP880_METER_LIM_STEPSIZE    15ul    /**< 15mV @ 12Khz, 18mV @ 16KHz */

#define VP880_METERING_RAMP_MASK    0x80
#define VP880_METERING_RAMP_SMOOTH  0x00
#define VP880_METERING_RAMP_ABRUPT  0x80

#define VP880_METERING_SLOPE_MASK       0x7F
#define VP880_METERING_SLOPE_DIVISOR    1000ul
#define VP880_METERING_SLOPE_STEPSIZE   68ul    /**< 68uA/step */

#define VP880_METERING_TIME_DIVISOR     10000ul
#define VP880_METERING_TIME_STEPSIZE    25ul    /**< 2.5mS/step */

#define VP880_METERING_PEAK_DIVISOR     100000ul
#define VP880_METERING_PEAK_STEPSIZE    78125ul /**< 7.8125mV/step */

/**< Signal Generator A, B, and Bias Command info */
#define VP880_SIGA_PARAMS_WRT       0xD2
#define VP880_SIGA_PARAMS_RD        0xD3

#ifndef VP880_SIGA_PARAMS_LEN
#define VP880_SIGA_PARAMS_LEN       0x0B
#endif

#define VP880_RINGER_PARAMS_WRT     VP880_SIGA_PARAMS_WRT
#define VP880_RINGER_PARAMS_RD      VP880_SIGA_PARAMS_RD
#define VP880_RINGER_PARAMS_LEN     VP880_SIGA_PARAMS_LEN

/* Map of the Signal Generator A/B Byte Locations */
#define VP880_SIGA_BIAS_MSB         0x01
#define VP880_SIGA_BIAS_LSB         0x02

#define VP880_SIGAB_FREQ_START      0x03
#define VP880_SIGA_FREQ_MSB         0x03
#define VP880_SIGA_FREQ_LSB         0x04
#define VP880_SIGA_AMP_MSB          0x05
#define VP880_SIGA_AMP_LSB          0x06
#define VP880_SIGB_FREQ_MSB         0x07
#define VP880_SIGB_FREQ_LSB         0x08
#define VP880_SIGB_AMP_MSB          0x09
#define VP880_SIGB_AMP_LSB          0x0A

/* Step size for Ringing Amplitude is precisely ~0.004730224609 (155/32768)*/
#define VP880_RINGING_AMP_SCALE     473
#define VP880_RINGING_AMP_FACTOR    100000

/* Step size for Ringing Bias is precisely (154.4/32768) */
#define VP880_RINGING_BIAS_SCALE    471
#define VP880_RINGING_BIAS_FACTOR   100000

/**< Signal Generator bit definitions */
#define VP880_RAMP_SLOPE_MASK   0x04
#define VP880_RAMP_POSITIVE     0x00
#define VP880_RAMP_NEGATIVE     0x04

#define VP880_SIGGEN1_OP_MASK   0x02
#define VP880_SIGGEN1_OUT_RAMP  0x02
#define VP880_SIGGEN1_OUT_CONT  0x00

#define VP880_SIGGEN1_SINTRAP_MASK  0x01
#define VP880_SIGGEN1_SIN           0x00
#define VP880_SIGGEN1_TRAP          0x01

/* Step size is more precisely 0.366198824 */
#define VP880_GENA_FREQ_DIVISOR     1000ul
#define VP880_GENA_FREQ_STEPSIZE    366ul   /**< 0.366Hz/step */

#define VP880_GENA_AMP_DIVISOR      100ul
#define VP880_GENA_AMP_STEPSIZE     471ul   /**< 4.71mV/step in Ringing Mode */

#define VP880_GENB_FREQ_DIVISOR     1000ul
#define VP880_GENB_FREQ_STEPSIZE    366ul   /**< 0.366Hz/step */

/**< Signal Generator C and D Command info */
#define VP880_SIGCD_PARAMS_WRT      0xD4
#define VP880_SIGCD_PARAMS_RD       0xD5
#define VP880_SIGCD_PARAMS_LEN      0x08

/**< Signal Generator C/D bit definitions */
#define VP880_GENC_FREQ_DIVISOR     1000ul
#define VP880_GENC_FREQ_STEPSIZE    366ul   /**< 0.366Hz/step */

#define VP880_GEND_FREQ_DIVISOR     1000ul
#define VP880_GEND_FREQ_STEPSIZE    366ul   /**< 0.366Hz/step */

/**< Signal Generator Control Command */
#define VP880_GEN_CTRL_WRT      0xDE
#define VP880_GEN_CTRL_RD       0xDF

#ifndef VP880_GEN_CTRL_LEN
#define VP880_GEN_CTRL_LEN      0x01
#endif

#define VP880_GEND_EN           0x08
#define VP880_GENC_EN           0x04
#define VP880_GENB_EN           0x02
#define VP880_GENA_EN           0x01

#define VP880_GEN_ALLOFF        0x00
#define VP880_GEN_ALLON         0x0F

/**< Signal Generator Control bit definitions */
#define VP880_GEN_CAD_EN        0x80
#define VP880_GEN_CAD_DIS       0x00

#define VP880_GEN_CTRL_CONT     0x00
#define VP880_GEN_CTRL_OS       0x40

#define VP880_GEN_CTRL_DPGEN_EN     0x20
#define VP880_GEN_CTRL_DPGEN_DIS    0x00

#define VP880_GEN_CTRL_EN_BIAS  0x10
#define VP880_GEN_CTRL_DIS_BIAS 0x00

#define VP880_GEN_CTRL_EN_D     0x08
#define VP880_GEN_CTRL_DIS_D    0x00

#define VP880_GEN_CTRL_EN_C     0x04
#define VP880_GEN_CTRL_DIS_C    0x00

#define VP880_GEN_CTRL_EN_B     0x02
#define VP880_GEN_CTRL_DIS_B    0x00

#define VP880_GEN_CTRL_EN_A     0x01
#define VP880_GEN_CTRL_DIS_A    0x00

/**< Cadence Timer Command */
#define VP880_CADENCE_TIMER_WRT     0xE0
#define VP880_CADENCE_TIMER_RD      0xE1
#define VP880_CADENCE_TIMER_LEN     0x04

/**< Cadence Timer bit definitions */
#define VP880_CADENCE_ON_DIVISOR    1ul
#define VP880_CADENCE_ON_STEPSIZE   5ul /**< 5mS/step */

#define VP880_CADENCE_OFF_DIVISOR   1ul
#define VP880_CADENCE_OFF_STEPSIZE  5ul /**< 5mS/step */

/**< Caller ID Data Command info */
#define VP880_CID_DATA_WRT      0xE2
#define VP880_CID_DATA_RD       0xE3
#define VP880_CID_DATA_LEN      0x01

/**< Caller ID Parameters Command info */
#define VP880_CID_PARAM_WRT     0xEA
#define VP880_CID_PARAM_RD      0xEB
#define VP880_CID_PARAM_LEN     0x01

/**< Caller ID Parameters bit definitions */
#define VP880_CID_STATE_MASK    0xE0
#define VP880_CID_STATE_IDLE    0x00
#define VP880_CID_STATE_RDY     0x20
#define VP880_CID_STATE_FULL    0x40
#define VP880_CID_STATE_LBYTE   0x60
#define VP880_CID_STATE_L2BYTE  0x80
#define VP880_CID_STATE_URUN    0xA0

/* New CID States for Rev D Silicon */
#define VP880_CID_STATE_EMPTY_D     0x20
#define VP880_CID_STATE_HALF_FULL_D 0x40
#define VP880_CID_STATE_LBYTE_D     0x60
#define VP880_CID_STATE_L2BYTE_D    0x80
#define VP880_CID_STATE_FULL_D      0xC0
#define VP880_CID_STATE_L3BYTE_D    0xE0

#define VP880_CID_EOM_MASK      0x10
#define VP880_CID_EOM           0x10
#define VP880_CID_NEOM          0x00

#define VP880_CID_FBDIS_MASK    0x08
#define VP880_CID_FBIT_DIS      0x08
#define VP880_CID_FBIT_EN       0x00

#define VP880_CID_FB_START_MASK 0x04
#define VP880_CID_FB_START_0    0x00
#define VP880_CID_FB_START_1    0x04

#define VP880_CID_FB_STOP_MASK  0x02
#define VP880_CID_FB_STOP_0     0x00
#define VP880_CID_FB_STOP_1     0x02

#define VP880_CID_FRAME_BITS    0x06

#define VP880_CID_CTRL_MASK     0x01
#define VP880_CID_EN            0x00
#define VP880_CID_DIS           0x01

/**< Software Reset Command info */
#define VP880_SW_RESET_WRT      0x02    /**< Software reset */
#define VP880_SW_RESET_LEN      0x00

/**< Pulse Period Detector Command info */
#define VP880_PERIOD_DET_RD     0x1F
#define VP880_PERIOD_DET_LEN    0x01    /**< Time between successive pulses */

/**< Pulse Period Detector bit definitions */
#define VP880_PULSE_PERIOD_DIVISOR      100ul
#define VP880_PULSE_PERIOD_STEPSIZE     25ul    /**< 0.25mS/step */

/**< Transmit and Receiver Timeslot Command info */
#define VP880_TX_TS_WRT         0x40    /**< Transmit time slot write */
#define VP880_TX_TS_RD          0x41    /**< Transmit time slot read */
#define VP880_TX_TS_LEN         0x01
#define VP880_TX_TS_MASK        0x7F

#define VP880_RX_TS_WRT         0x42    /**< Receive time slot write */
#define VP880_RX_TS_RD          0x43    /**< Receive time slot read */
#define VP880_RX_TS_LEN         0x01
#define VP880_RX_TS_MASK        0x7F

/**< Internal Configuration Register 1 */
#define VP880_ICR1_WRT      0xEC
#define VP880_ICR1_RD       0xED

#ifndef VP880_ICR1_LEN
#define VP880_ICR1_LEN      0x04
#endif

#define VP880_ICR1_BIAS_OVERRIDE_LOCATION   0x00
#define VP880_ICR1_TIP_BIAS_OVERRIDE        0xF0
#define VP880_ICR1_LINE_BIAS_OVERRIDE       0x0F
#define VP880_ICR1_LINE_BIAS_OVERRIDE_NORM  0x08

#define VP880_ICR1_LINE_BIAS_LOCATION       0x01
#define VP880_ICR1_LINE_BIAS                0x0F
#define VP880_ICR1_TIP_BIAS_LOCATION        0x01
#define VP880_ICR1_TIP_BIAS                 0xF0

#define VP880_ICR1_RING_BIAS_OVERRIDE_LOCATION  0x02
#define VP880_ICR1_RING_BIAS_OVERRIDE           0x0F

#define VP880_ICR1_RING_AND_DAC_LOCATION    0x02
#define VP880_ICR1_RING_AND_DAC_B2_3        0x06
#define VP880_ICR1_RING_BIAS_DAC_MASK       0x0C

#define VP880_ICR1_RING_BIAS_LOCATION       0x03
#define VP880_ICR1_RING_BIAS                0x0F

#define VP880_ICR1_TEST_LOAD_LOCATION       0x03
#define VP880_ICR1_TEST_LOAD_MASK           0xC0
#define VP880_ICR1_TEST_LOAD_NONE           0x00
#define VP880_ICR1_TEST_LOAD_METALLIC       0x40
#define VP880_ICR1_TEST_LOAD_LONGITUDINAL   0x80

/**< Internal Configuration Register 2 */
#define VP880_ICR2_WRT      0xEE
#define VP880_ICR2_RD       0xEF

#ifndef VP880_ICR2_LEN
#define VP880_ICR2_LEN      0x04
#endif

#define VP880_ICR2_SENSE_INDEX      0x00
#define VP880_ICR2_DAC_SENSE        0x80
#define VP880_ICR2_ILA_DAC          0x80
#define VP880_ICR2_FEED_SENSE       0x40
#define VP880_ICR2_VOC_DAC_SENSE    0x20
#define VP880_ICR2_ILA_FDRING_SENSE 0x10

#define VP880_ICR2_TIP_SENSE        0x08
#define VP880_ICR2_RING_SENSE       0x04
#define VP880_ICR2_VOC_DAC_INDEX    0x00

#define VP880_ICR2_SPEEDUP_INDEX    0x02
#define VP880_ICR2_SWY_CTRL_INDEX   0x02
#define VP880_ICR2_MET_SPEED_CTRL   0x80
#define VP880_ICR2_BAT_SPEED_CTRL   0x40
#define VP880_ICR2_SWY_CTRL_EN      0x20
#define VP880_ICR2_SWY_LIM_CTRL1    0x08
#define VP880_ICR2_SWY_LIM_CTRL     0x04

/**< Internal Configuration Register 3 */
#define VP880_ICR3_WRT      0xF2
#define VP880_ICR3_RD       0xF3

#ifndef VP880_ICR3_LEN
#define VP880_ICR3_LEN      0x04
#endif

#define VP880_ICR3_LINE_CTRL_INDEX      0x00
#define VP880_ICR3_LINE_CTRL            0x20
#define VP880_ICR3_SAT_LIM_25_CTRL      0x10
#define VP880_ICR3_VREF_CTRL            0x01


#define VP880_ICR3_LONG_UNCLAMP         0x40
#define VP880_ICR3_LONG_UNCLAMP_INDEX   0x02
#define VP880_ICR3_LONG_FIXED           0x10
#define VP880_ICR3_LONG_FIXED_INDEX     0x00

#define VP880_ICR3_LONG_LOOP_CTRL_LOCATION  0x02
#define VP880_ICR3_LONG_LOOP_CONTROL        0x80
#define VP880_ICR3_SE_RINGING_CONTROL       0x01

/**< Internal Configuration Register 4 */
#define VP880_ICR4_WRT      0xF4
#define VP880_ICR4_RD       0xF5

#ifndef VP880_ICR4_LEN
#define VP880_ICR4_LEN      0x04
#endif

#define VP880_ICR4_VOICE_ADC_INDEX      0x00
#define VP880_ICR4_VOICE_ADC_CTRL       0x01
#define VP880_ICR4_VOICE_DAC_CTRL       0x02

#define VP880_ICR4_SUP_INDEX            0x02
#define VP880_ICR4_SUP_DAC_CTRL         0x08
#define VP880_ICR4_SUP_DET_CTRL         0x04
#define VP880_ICR4_SUP_POL_CTRL         0x02

#define VP880_ICR4_GKEY_DET_LOCATION    0x02
#define VP880_ICR4_GKEY_POL             0x10
#define VP880_ICR4_GKEY_DET             0x01

#define VP880_ICR5_WRT              0xFA
#define VP880_ICR5_RD               0xFB
#define VP880_ICR5_LEN              0x02

#define VP880_ICR5_FEED_HOLD_INDEX  0x00

#define VP880_ICR5_FEED_HOLD_MASK   0xF0
#define VP880_ICR5_BAT_HOLD_MASK    0x0F
#define VP880_ICR5_H2MS_BYTE        1
#define VP880_ICR5_H2MS_MASK        0xF0
#define VP880_ICR5_SUBST_MASK       0x0F

/**< Internal Configuration Register 6 */
#define VP880_ICR6_WRT      0xFC
#define VP880_ICR6_RD       0xFD

#ifndef VP880_ICR6_LEN
#define VP880_ICR6_LEN      0x02
#endif

/* Upper Byte Bits */
#define VP880_DC_CAL_ABS_INDEX          0
#define VP880_DC_CAL_ABS_MASK           0xF0
#define VP880_DC_CAL_BAT_SW_OVRD        0x08
#define VP880_DC_CAL_OFFSET_OVRD        0x04
#define VP880_DC_CAL_RSVD               0x03

/* Lower Byte Bits */
#define VP880_DC_CAL_CUT_INDEX          1
#define VP880_EN_XB_SENSE               0x80
#define VP880_C_RING_SNS_CUT            0x40
#define VP880_C_TIP_SNS_CUT             0x20
#define VP880_VOC_DC_CTRL               0x10
#define VP880_DC_CAL_BAT_SW_MID         0x08
#define VP880_DC_CAL_DIS_INPUT_OFFSET   0x04

#define VP880_DCCAL_BAT_SW_HYST_MASK    0x03
#define VP880_DCCAL_BAT_SW_HYST_1V      0x00
#define VP880_DCCAL_BAT_SW_HYST_3V      0x01
#define VP880_DCCAL_BAT_SW_HYST_5V      0x02
#define VP880_DCCAL_BAT_SW_HYST_7V      0x03

#define VP880_DC_CAL_BLIM_INDEX 1
#define VP880_DC_CAL_BLIM       0x08

VpStatusType
Vp880ConfigLine(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pAcProfile,
    VpProfilePtrType pDcFeedOrFxoCfgProfile,
    VpProfilePtrType pRingProfile);

#ifdef VP_CSLAC_SEQ_EN
VpStatusType
Vp880InitRing(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pCadProfile,
    VpProfilePtrType pCidProfile);

VpStatusType
Vp880InitCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    uint8p pCidData);

/**< Profile index for ramp tone parameters */
typedef enum vp880_rampToneProfileParams {
    VP880_START_FREQ_MSB = 8,
    VP880_START_FREQ_LSB = 9,
    VP880_START_LEVEL_MSB = 10,
    VP880_START_LEVEL_LSB = 11,

    VP880_STEP_FREQ_MSB = 12,
    VP880_STEP_FREQ_LSB = 13,
    VP880_STEP_LEVEL_MSB = 14,
    VP880_STEP_LEVEL_LSB = 15,

    VP880_STOP_FREQ_MSB = 16,
    VP880_STOP_FREQ_LSB = 17,
    VP880_STOP_LEVEL_MSB = 18,
    VP880_STOP_LEVEL_LSB = 19,

    VP880_RAMP_TONE_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} vp880_rampToneProfileParams;

#endif

VpStatusType
Vp880MakeLineObject(
    VpTermType termType,
    uint8 channelId,
    VpLineCtxType *pLineCtx,
    void *pLineObj,
    VpDevCtxType *pDevCtx);

/**< Vp880 Control Function Prototypes */
#ifdef CSLAC_GAIN_RELATIVE
VpStatusType
Vp880SetRelGain(
    VpLineCtxType *pLineCtx,
    uint16 txLevel,
    uint16 rxLevel,
    uint16 handle);
#endif

VpStatusType
Vp880SetLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);

#ifdef VP880_ABS_SUPPORT
uint8
Vp880GetLineStateABS(
    VpLineCtxType *pLineCtx,
    VpLineStateType state,
    bool forced);
#endif

#if defined (VP880_TRACKER_SUPPORT) && defined (VP880_FXS_SUPPORT)
void
Vp880LimitInRushCurrent(
    Vp880DeviceObjectType *pDevObj,
    uint8 ecVal,
    bool callback);
#endif

#ifdef VP880_FXS_SUPPORT
void
Vp880GroundStartProc(
    bool gsMode,
    VpLineCtxType *pLineCtx,
    uint8 currentLineState,
    uint8 userByte);

VpStatusType
Vp880SetRelayState(
    VpLineCtxType *pLineCtx,
    VpRelayControlType rState);

VpStatusType
Vp880SendCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    VpProfilePtrType pCidProfile,
    uint8p pCidData);

VpStatusType
Vp880ContinueCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    uint8p pCidData);
#endif

VpStatusType
Vp880SendSignal(
    VpLineCtxType *pLineCtx,
    VpSendSignalType type,
    void *pStruct);

VpStatusType
Vp880SetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *value);

VpStatusType
Vp880SetCodec(
    VpLineCtxType *pLineCtx,
    VpOptionCodecType codec);

VpStatusType
Vp880DeviceIoAccess(
    VpDevCtxType *pDevCtx,
    VpDeviceIoAccessDataType *pDeviceIoData);

#ifndef VP880_SIMPLE_POLLED_MODE
VpStatusType
Vp880VirtualISR(
    VpDevCtxType *pDevCtx);
#endif

VpStatusType
Vp880ApiTick(
    VpDevCtxType *pDevCtx,
    bool *pEventStatus);

/**< Function called by Api Tick only. Processes the line passed for Api
 * Tick based operations
 */
#ifdef VP880_FXS_SUPPORT
VpStatusType
Vp880ProcessFxsLine(
    Vp880DeviceObjectType *pDevObj,
    VpLineCtxType *pLineCtx);

bool
Vp880OnHookMgmt(
    Vp880DeviceObjectType *pDevObj,
    VpLineCtxType *pLineCtx,
    uint8 ecVal);

bool
Vp880OffHookMgmt(
    Vp880DeviceObjectType *pDevObj,
    VpLineCtxType *pLineCtx,
    uint8 ecVal);

VpStatusType
Vp880FreeRun(
    VpDevCtxType *pDevCtx,
    VpFreeRunModeType freeRunMode);

void
Vp880RestartComplete(
    VpDevCtxType *pDevCtx);
#endif

#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
VpStatusType
Vp880LowLevelCmd(
    VpLineCtxType *pLineCtx,
    uint8 *pCmdData,
    uint8 len,
    uint16 handle);
#endif

VpStatusType
Vp880GetTxRxPcmMode(
    Vp880LineObjectType *pLineObj,
    VpLineStateType state,
    uint8 *mpiByte);

#ifdef VP880_LP_SUPPORT
void
Vp880LowPowerMode(
    VpDevCtxType *pDevCtx);

void
Vp880RunLPDisc(
    VpLineCtxType *pLineCtx,
    bool discMode,
    uint8 nextSlicByte);

void
Vp880SetLPRegisters(
    Vp880LineObjectType *pLineObj,
    bool lpModeTo);

void
Vp880WriteLPExitRegisters(
    VpLineCtxType *pLineOtx,
    VpDeviceIdType deviceId,
    uint8 ecVal,
    uint8 *lineState);

void
Vp880SetLP(
    bool lpMode,
    VpLineCtxType *pLineCtx);

uint16
Vp880SetDiscTimers(
    Vp880DeviceObjectType *pDevObj);
#endif

void
Vp880UpdateBufferChanSel(
    Vp880DeviceObjectType *pDevObj,
    uint8 channelId,
    uint8 sysState,
    bool devWrite);

#ifdef VP880_FXS_SUPPORT
uint8 Vp880ProtectedWriteICR1(
    Vp880LineObjectType *pLineObj,
    uint8 mpiIndex,
    uint8 *mpiBuffer);

#endif

#ifdef VP880_FXO_SUPPORT
VpStatusType
Vp880SetFxoState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);

bool
Vp880ProcessFxoLine(
    Vp880DeviceObjectType *pDevObj,
    VpLineCtxType *pLineCtx);
#endif

/* Processes all interrupts from the device that are line specific (i.e., not
 * Clock or Battery fault).
 */
bool
Vp880ServiceInterrupts(
    VpDevCtxType *pDevCtx);

/**< Vp880 Status and Query Function Prototypes */
bool
Vp880ServiceTimers(
    VpDevCtxType *pDevCtx);

bool
Vp880FindSoftwareInterrupts(
    VpDevCtxType *pDevCtx);

bool
Vp880GetEvent(
    VpDevCtxType *pDevCtx,
    VpEventType *pEvent);

VpStatusType
Vp880GetDeviceStatus(
    VpDevCtxType *pDevCtx,
    VpInputType input,
    uint32 *pDeviceStatus);

VpStatusType
Vp880GetLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType *pCurrentState,
    VpLineStateType *pPreviousState);

VpStatusType
Vp880FlushEvents(
    VpDevCtxType *pDevCtx);

VpStatusType
Vp880GetResults(
    VpEventType *pEvent,
    void *pResults);

VpStatusType
Vp880GetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    uint16 handle);

void
Vp880ServiceDiscExitTimer(
    VpLineCtxType *pLineCtx);
#ifdef VP880_INCLUDE_TESTLINE_CODE
EXTERN VpStatusType
Vp880GetRelayState(
    VpLineCtxType *pLineCtx,
    VpRelayControlType *pRstate);
#endif

#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
VpStatusType
Vp880RegisterDump(
    VpDevCtxType *pDevCtx);

VpStatusType
Vp880ObjectDump(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx);
#endif

#ifdef VP880_INCLUDE_TESTLINE_CODE
/* Testing functions */
VpStatusType
Vp880TestLine(
    VpLineCtxType *pLineCtx,
    VpTestIdType test,
    const void *pArgsUntyped,
    uint16 handle);

VpStatusType
Vp880TestLineInt(
    VpLineCtxType *pLineCtx,
    VpTestIdType test,
    const void *pArgsUntyped,
    uint16 handle,
    bool callback);

EXTERN VpStatusType
Vp880TestLineCallback(
    VpLineCtxType *pLineCtx,
    VpPcmOperationResultsType *pResults);
#endif

/**< Function called by Api only to determine if a particular line is currently
 * running a test. Technically, could be removed if line test is compiled out
 * and then provided all logic calling this function was changed. But it's
 * easier to keep this function but force it to return FALSE if line test is
 * comipled out.
 */
EXTERN bool Vp880IsChnlUndrTst(
    Vp880DeviceObjectType *pDevObj,
    uint8 channelId);

#ifdef VP_CSLAC_RUNTIME_CAL_ENABLED
    /* Calibration functions */
    VpStatusType Vp880CalCodec(VpLineCtxType *pLineCtx, VpDeviceCalType mode);
    VpStatusType Vp880CalLine(VpLineCtxType *pLineCtx);
    void Vp880CalInit(VpDevCtxType *pDevCtx);
    int16 Vp880AdcSettling(Vp880DeviceObjectType *pDevObj, uint8 ecVal, uint8 adcConfig, bool *validData);
    bool Vp880SetCalFlags(Vp880DeviceObjectType *pDevObj);
    VpStatusType Vp880CalCodecInt(VpDevCtxType *pDevCtx);
    VpStatusType Vp880CalAbv(VpDevCtxType *pDevCtx);
    VpStatusType Vp880CalAbvAbsDev(VpDevCtxType *pDevCtx);
    void Vp880AbsCalibration(VpDevCtxType *pDevCtx);
    VpStatusType Vp880CalLineInt(VpLineCtxType *pLineCtx);

    /*
     * Sample time to wait between Battery Calibration (generally long if waiting
     * for discharge).
    */
    #define VP880_CAL_ABV_LONG          (200)
    #define VP880_CAL_ABV_DECAY_STEP    (100)
#endif

#define VP880_V_PCM_LSB         (7324L)  /**< 7.324V per LSB from PCM Data */

VpStatusType
Vp880Cal(
    VpLineCtxType       *pLineCtx,
    VpCalType           calType,
    void                *inputArgs);

bool
Vp880AdjustIla(
    VpLineCtxType *pLineCtx,
    uint8 targetIla);

bool
Vp880AdjustVoc(
    VpLineCtxType *pLineCtx,
    uint8 targetVoc,
    bool previousCal);

/**< VP880_AUTO_BAT_DETECT
 * DO NOT CHANGE to #define unless in a demo environment where the VE880 ABS Supply Configuration
 * is unknown. This enables experimental code that attempts to detect the supply configuration
 * (low or high battery voltage regulated). It was developed in 2007 and has for the most part not
 * been maintained since.
 */
#undef  VP880_AUTO_BAT_DETECT

/* Functions required in a demo environment only. DO NOT USE IN PRODUCTION */
#ifdef VP880_AUTO_BAT_DETECT
VpStatusType
Vp880AutoBatDetect(
    Vp880DeviceObjectType *pDevObj,
    VpProfilePtrType pSwitcherParam);
#endif

#endif  /* Vp880_API_INT_H */

