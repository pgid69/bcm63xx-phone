/** \file vp580_api_int.h
 * vp580_api_int.h
 *
 * Header file for the vp580 series API-II c files.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 9115 $
 * $LastChangedDate: 2011-11-15 15:25:17 -0600 (Tue, 15 Nov 2011) $
 */

#ifndef VP580_API_INT_H
#define VP580_API_INT_H

#include "vp_api_event.h"
#include "vp_api_option.h"

/**< Define the initial hook value to use when determining if the line status
 * has been changed from initialization. This must be an invalid value to force
 * a signaling register read for simple polled mode.
 */
#define VP580_HOOK_INIT_VAL 0xFF

/**< Define the mask that will report device busy if there is a currently active
 * event when the user is attempting to perform another "read" transaction.
 */
#define VP580_NO_MASK   0x0000

#define VP580_READ_RESPONSE_MASK (VP_LINE_EVID_LLCMD_RX_CMP \
                                 | VP_LINE_EVID_RD_OPTION \
                                 | VP_LINE_EVID_GAIN_CMP)

#define VP580_NONSUPPORT_FAULT_EVENTS   (VP_LINE_EVID_DC_FLT \
                                       | VP_LINE_EVID_AC_FLT \
                                       | VP_DEV_EVID_EVQ_OFL_FLT \
                                       | VP_DEV_EVID_WDT_FLT)

#define VP580_NONSUPPORT_SIGNALING_EVENTS   (VP_LINE_EVID_MTONE \
                                           | VP_LINE_EVID_US_TONE_DETECT \
                                           | VP_LINE_EVID_DS_TONE_DETECT \
                                           | VP_DEV_EVID_SEQUENCER)

#define VP580_FXS_SIGNALING_EVENTS  (VP_LINE_EVID_HOOK_OFF \
                                   | VP_LINE_EVID_HOOK_ON \
                                   | VP_LINE_EVID_GKEY_DET \
                                   | VP_LINE_EVID_GKEY_REL \
                                   | VP_LINE_EVID_FLASH \
                                   | VP_LINE_EVID_STARTPULSE \
                                   | VP_LINE_EVID_PULSE_DIG \
                                   | VP_LINE_EVID_BREAK_MAX)

#define VP580_NONSUPPORT_RESPONSE_EVENTS    (VP_DEV_EVID_BOOT_CMP \
                                           | VP_DEV_EVID_DNSTR_MBOX \
                                           | VP_LINE_EVID_RD_LOOP \
                                           | VP_EVID_CAL_CMP \
                                           | VP_EVID_CAL_BUSY)

#define VP580_NONSUPPORT_TEST_EVENTS        (VP_LINE_EVID_TEST_RSVD1 \
                                           | VP_LINE_EVID_DTONE_DET \
                                           | VP_LINE_EVID_DTONE_LOSS \
                                           | VP_DEV_EVID_STEST_CMP \
                                           | VP_DEV_EVID_CHKSUM)

#define VP580_NONSUPPORT_PROCESS_EVENTS VP580_NO_MASK

#define VP580_NONSUPPORT_FXO_EVENTS     VP580_NO_MASK

#if 1
/*
 * This timer is used by the API Tick counter to prevent Active State
 * for 100ms after an on-hook transition.
 */
#define ON_HOOK_TIMER    20
#define ON_HOOK_ABS_CLARE_DELAY 2

/**< Revision and Product Code Command info */
#define VP580_DEVTYPE_CMD       0x73
#define VP580_DEVTYPE_LEN       0x01
#define VP580_NO_OP             0x06

/**< GLOBAL REGISTERS (Effects all SLAC device channels) */
#define VP580_HW_RESET_CMD      0x04    /**< Hardware Reset */

/**< Transmit/Receive Clock Slot Command info */
#define VP580_XR_CS_WRT         0x44    /**< Tx/Rx clock slot register write */
#define VP580_XR_CS_RD          0x45    /**< Tx/Rx clock slot register read */
#define VP580_XR_CS_LEN         0x01

#define VP580_TX_CSLOT_WRT      0x44    /**< Tx clock slot register write */
#define VP580_TX_CSLOT_RD       0x45    /**< Tx clock slot register read */
#define VP580_TX_CSLOT_LEN      0x01

#define VP580_RX_CSLOT_WRT      0x44    /**< Rx clock slot register write */
#define VP580_RX_CSLOT_RD       0x45    /**< Rx clock slot register read */
#define VP580_RX_CSLOT_LEN      0x01

/**< Bit definitions for Transmit/Receive Clock Slot Register */
#define VP580_TX_SLOT_MASK      0x07
#define VP580_RX_SLOT_MASK      0x38

/**< Transmit Edge Command info */
#define VP580_TX_EDGE_WRT       0x44    /**< Tx clock slot register write */
#define VP580_TX_EDGE_RD        0x45    /**< Tx clock slot register read */
#define VP580_TX_EDGE_LEN       0x01

/**< Bit definitions for Transmit Edge Register */
#define VP580_TX_EDGE_MASK      0x40

/**< Device (Chip) Configuration Command info */
#define VP580_DCR_WRT           0x46
#define VP580_DCR_RD            0x47
#define VP580_DCR_LEN           0x01

/**< Bit definitions for Device Configuration */
#define VP580_DCR_INTMODE_MASK      0x80
#define VP580_DCR_CHOPPER_CLK_MASK  0x40
#define VP580_DCR_PCM_SMODE_MASK    0x20

/**< Master Clock Command info */
#define VP580_MCLK_CNT_WRT          0x46
#define VP580_MCLK_CNT_RD           0x47
#define VP580_MCLK_CNT_LEN          0x01

/**< Bit definitions for Master Clock Command */
#define VP580_MCLK_CLOCK_SRC_MASK   0x10
#define VP580_MCLK_CLKSEL_MASK      0x0F

/**< E1Mux, Debounce, PCLK mode, Chopper Clock command info */
#define VP580_DEBOUNCE_TIME_WRT     0xC8
#define VP580_DEBOUNCE_TIME_RD      0xC9
#define VP580_DEBOUNCE_TIME_LEN     0x01

/**< Bit definitions for E1Mux, Debounce, PCLK mode, Chopper Clock command */
#define VP580_DEBOUNCE_TIME_HOOK_SW 2   /* Shift value into the register */

/**< Operating Mode Command info */
#define VP580_OP_MODE_WRT           0x4A
#define VP580_OP_MODE_RD            0x4B
#define VP580_OP_MODE_LEN           0x01

/**< Bit definitions for Operating Mode Command */
#define VP580_RBE_MODE_MASK         0x40
#define VP580_VOUT_MODE_MASK        0x20
#define VP580_LOW_POWER_MODE_MASK   0x10    /* Not needed for QLSLAC */

/**< Signaling Register Command info */
#define VP580_NO_UL_SIGREG_RD       0x4D    /**< Read w/o unlock signaling reg */
#define VP580_NO_UL_SIGREG_LEN      0x01

#define VP580_UL_SIGREG_RD          0x4F    /**< Read w/unlock signaling reg */

#ifndef VP580_UL_SIGREG_LEN
#define VP580_UL_SIGREG_LEN         0x01
#endif

/**< Interrupt Mask Register info */
#define VP580_INT_MASK_WRT      0x6C
#define VP580_INT_MASK_RD       0x6D
#define VP580_INT_MASK_LEN      0x01
#define VP580_INT_MASK_ALL      0xFF
#define VP580_INT_NO_MASK       0x00

/**< Revision and Product Code Command info */
#define VP580_RCN_PCN_RD        0x73
#define VP580_RCN_PCN_LEN       0x01

/**< CHANNEL REGISTERS (must set EC register first) */

/**< Channel Enable Command info */
#define VP580_EC_WRT                0x4A
#define VP580_EC_RD                 0x4B
#define VP580_EC_LEN                0x01
#define VP580_EC_CH1                0x01
#define VP580_EC_CH2                0x02
#define VP580_EC_CH3                0x04
#define VP580_EC_CH4                0x08
#define VP580_EC_STATE_AT_RESET     0x0F

/**< Bit definitions for Channel Enable Command */
#define VP580_EC_BITS_MASK          0x0F

/**< Commands to Activate/Deactivate the Channel */
#define VP580_ACTIVATE_CMD      0x0E
#define VP580_DEACTIVATE_CMD    0x00

#define VP580_EC_RD                 0x4B
#define VP580_EC_LEN                0x01


/**< IO Data Command info */
#define VP580_IODATA_REG_WRT    0x52    /**< I/O register write */
#define VP580_IODATA_REG_RD     0x53    /**< I/O register read */
#define VP580_IODATA_REG_LEN    0x01

#define VP580_IODATA_C7         0x80
#define VP580_IODATA_C6         0x40
#define VP580_IODATA_CD1B       0x20
#define VP580_IODATA_C5         0x10
#define VP580_IODATA_C4         0x08
#define VP580_IODATA_C3         0x04
#define VP580_IODATA_CD2        0x02
#define VP580_IODATA_CD1        0x01

#define VP580_IODATA_IO7        VP580_IODATA_C7
#define VP580_IODATA_IO6        VP580_IODATA_C6
#define VP580_IODATA_IO5        VP580_IODATA_C5
#define VP580_IODATA_IO4        VP580_IODATA_C4
#define VP580_IODATA_IO3        VP580_IODATA_C3
#define VP580_IODATA_IO2        VP580_IODATA_CD2
#define VP580_IODATA_IO1        VP580_IODATA_CD1

typedef enum Vp580ApiIOToBitMap {
    VP580_CH1_IO1,
    VP580_CH2_IO1,
    VP580_CH1_IO2,
    VP580_CH2_IO2
} Vp580ApiIOToBitMap;

/**< Bit definitions for IO Data Command */
#define VP580_IODATA_BITS_MASK  0x3F    /**< Lowest two bits always available */

/**< IO Direction Command info */
#define VP580_IODIR_REG_WRT     0x54    /**< I/O direction register write */
#define VP580_IODIR_REG_RD      0x55    /**< I/O direction register read */
#define VP580_IODIR_REG_LEN     0x01

/**< Bit definitions for IO Direction Command */
/* Device IO Access marking I/O 6 and 7 for input only */
#define VP580_IODIR_INPUT_ONLY  0x60606060

#define VP580_IODIR_CTRL_BITS   0x1F
#define VP580_IODIR_IO1_MASK    0x01
#define VP580_IODIR_IO2_MASK    0x02
#define VP580_IODIR_IO3_MASK    0x04
#define VP580_IODIR_IO4_MASK    0x08
#define VP580_IODIR_IO5_MASK    0x10
#define VP580_CFAIL_MASK        0x20
#define VP580_CH_STATUS_MASK    0x40

#define VP580_IODIR_INPUT       0x00
#define VP580_IODIR_OUTPUT      0x01
#define VP580_IODIR_DEFAULT     0x00


/**< System State Command info */
#define VP580_SYS_STATE_WRT     0x52
#define VP580_SYS_STATE_RD      0x53
#define VP580_SYS_STATE_LEN     0x01

#define VP580_SLIC_STATE_WRT    VP580_SYS_STATE_WRT
#define VP580_SLIC_STATE_RD     VP580_SYS_STATE_RD
#define VP580_SLIC_STATE_LEN    VP580_SYS_STATE_LEN

/* State definition are defined be profile (FXS) or termination type (FXO) */

/**< Operating Functions Command info */
#define VP580_OP_FUNC_WRT       0x60
#define VP580_OP_FUNC_RD        0x61
#define VP580_OP_FUNC_LEN       0x01

/**< Bit definitions for Operating Functions Command */
#define VP580_ENABLE_GR     0x20
#define VP580_ENABLE_GX     0x10
#define VP580_ENABLE_X      0x08
#define VP580_ENABLE_R      0x04
#define VP580_ENABLE_Z      0x02
#define VP580_ENABLE_B      0x01
#define VP580_ENABLE_LOADED_COEFFICIENTS 0x3F

#define VP580_DEFAULT_OP_FUNC_MODE 0x00

/**< Codec Compression Command info (Operating Functions) */
#define VP580_CODEC_REG_WRT     0x60
#define VP580_CODEC_REG_RD      0x61
#define VP580_CODEC_REG_LEN     0x01

/**< Bit definitions for Codec Compression Command */

/* Note:  If Linear Mode is selected, u-Law/A-Law selection is ignored */
#define VP580_CODEC_COMPRESSION_MASK    0xC0
#define VP580_ALAW_CODEC        0x00    /**< a-Law compression is used */
#define VP580_ULAW_CODEC        0x40    /**< u-law compression is used */
#define VP580_LINEAR_CODEC      0x80    /**< Linear mode is used */

/**< Operating Conditions Command info */
#define VP580_OP_COND_WRT         0x70
#define VP580_OP_COND_RD          0x71
#define VP580_OP_COND_LEN         0x01

/**< Map the loop back register to operating conditions register */
#define VP580_LOOPBACK_WRT        VP580_OP_COND_WRT
#define VP580_LOOPBACK_RD         VP580_OP_COND_RD
#define VP580_LOOPBACK_LEN        VP580_OP_COND_LEN

/**< Bit definitions for Operating Conditions Command */
#define VP580_TX_PATH_MASK      0x80
#define VP580_CUT_TXPATH        0x80
#define VP580_TXPATH_EN         0x00
#define VP580_TXPATH_DIS        0x80

#define VP580_RX_PATH_MASK      0x40
#define VP580_CUT_RXPATH        0x40
#define VP580_RXPATH_EN         0x00
#define VP580_RXPATH_DIS        0x40

#define VP580_HIGH_PASS_MASK    0x20
#define VP580_HIGH_PASS_EN      0x00
#define VP580_HIGH_PASS_DIS     0x20

#define VP580_LOWER_RX_GAIN_MASK    0x10
#define VP580_RX_GAIN_6DB_LOSS      0x10
#define VP580_RX_GAIN_0DB_LOSS      0x00

#define VP580_INTERFACE_LOOPBACK_EN 0x04
#define VP580_FULL_DIGL_LOOPBACK_EN 0x02
#define VP580_1KHZ_TONE_ON          0x01

/**< GX Filter Command info */
#define VP580_GX_GAIN_WRT       0x80
#define VP580_GX_GAIN_RD        0x81
#define VP580_GX_GAIN_LEN       0x02

/**< GR Filter Command info */
#define VP580_GR_GAIN_WRT       0x82
#define VP580_GR_GAIN_RD        0x83
#define VP580_GR_GAIN_LEN       0x02

#define VP580_NORMAL_OP_COND_MODE   0x00

/**< Read Transmit PCM/Test Data Command info */
#define VP580_TX_PCM_DATA_RD        0xCD
#define VP580_TX_PCM_DATA_LEN       0x02

/**< Software Reset Command info */
#define VP580_SW_RESET_CMD      0x02    /**< Software reset */
#define VP580_SW_RESET_LEN      0x00

/**< Transmit and Receiver Timeslot Command info */
#define VP580_TX_TS_WRT         0x40    /**< Transmit time slot write */
#define VP580_TX_TS_RD          0x41    /**< Transmit time slot read */
#define VP580_TX_TS_LEN         0x01
#define VP580_TX_TS_MASK        0x7F

#define VP580_RX_TS_WRT         0x42    /**< Receive time slot write */
#define VP580_RX_TS_RD          0x43    /**< Receive time slot read */
#define VP580_RX_TS_LEN         0x01
#define VP580_RX_TS_MASK        0x7F

#endif

typedef enum Vp580RingProfileFieldType {
    VP580_RING_PROFILE_METHOD       = 5,
    VP580_RING_PROFILE_TIMER_PERIOD = 6
} Vp580RingProfileFieldType;

#define VP580_RING_METHOD_NONE       0x00
#define VP580_RING_METHOD_CD2_3STATE 0x01
#define VP580_RING_METHOD_CD2_SQUARE 0x02

typedef enum Vp580RingSigGenControlType {
    VP580_RING_SIG_GEN_INIT,
    VP580_RING_SIG_GEN_START,
    VP580_RING_SIG_GEN_UPDATE,
    VP580_RING_SIG_GEN_STOP,
    VP580_RING_SIG_GEN_SHUTDOWN
} Vp580RingSigGenControlType;

/* Defines for API */

VpStatusType
Vp580MakeLineObject(
    VpTermType termType,
    uint8 channelId,
    VpLineCtxType *pLineCtx,
    void *pLineObj,
    VpDevCtxType *pDevCtx);

VpStatusType
Vp580InitRing(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pCadProfile,
    VpProfilePtrType pCidProfile);

/**< Vp580 Control Function Prototypes */
VpStatusType
Vp580SetLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);

VpStatusType
Vp580SendSignal(
    VpLineCtxType *pLineCtx,
    VpSendSignalType type,
    void *pStruct);

VpStatusType
Vp580SetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *value);

VpStatusType
Vp580DeviceIoAccess(
    VpDevCtxType *pDevCtx,
    VpDeviceIoAccessDataType *pDeviceIoData);

#ifndef VP580_SIMPLE_POLLED_MODE
VpStatusType
Vp580VirtualISR(
    VpDevCtxType *pDevCtx);
#endif

VpStatusType
Vp580ApiTick(
    VpDevCtxType *pDevCtx,
    bool *pEventStatus);

#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
VpStatusType
Vp580LowLevelCmd(
    VpLineCtxType *pLineCtx,
    uint8 *pCmdData,
    uint8 len,
    uint16 handle);
#endif

VpStatusType
Vp580SetRelGain(
    VpLineCtxType *pLineCtx,
    uint16 txLevel,
    uint16 rxLevel,
    uint16 handle);

void
Vp580RingSigGen(
    VpLineCtxType *pLineCtx,
    Vp580RingSigGenControlType control,
    uint8 *pUserByte);

/* Processes all interrupts from the device that are line specific (i.e., not
 * Clock or Battery fault).
 */
bool
Vp580ServiceInterrupts(
    VpDevCtxType *pDevCtx);

/**< Vp580 Status and Query Function Prototypes */
bool
Vp580ServiceTimers(
    VpDevCtxType *pDevCtx);

bool
Vp580FindSoftwareInterrupts(
    VpDevCtxType *pDevCtx);

bool
Vp580GetEvent(
    VpDevCtxType *pDevCtx,
    VpEventType *pEvent);

VpStatusType
Vp580GetDeviceStatus(
    VpDevCtxType *pDevCtx,
    VpInputType input,
    uint32 *pDeviceStatus);

VpStatusType
Vp580GetLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType *pCurrentState,
    VpLineStateType *pPreviousState);

VpStatusType
Vp580FlushEvents(
    VpDevCtxType *pDevCtx);

VpStatusType
Vp580GetResults(
    VpEventType *pEvent,
    void *pResults);

VpStatusType
Vp580GetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    uint16 handle);

#endif  /* Vp580_API_INT_H */




