/** \file vp_api_option.h
 * vp_api_option.h
 *
 * This file contains declaration associated with VP-API Options.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11607 $
 * $LastChangedDate: 2014-10-20 15:31:02 -0500 (Mon, 20 Oct 2014) $
 */

#ifndef VP_API_OPTION
#define VP_API_OPTION

#include "vp_api_types.h"
#include "vp_api_cfg.h"
#include "vp_api_event.h"

/* Option IDs.  (See Options chapter in VP-API-2 Reference Guide.)
 * Line-specific option IDs begin with "VP_OPTION_ID_".  Device-specific
 * option IDs begin with "VP_DEVICE_OPTION_ID_".  When new option IDs are added,
 * the VpOptionValueType struct (below) must be updated accordingly.
 */
typedef enum VpOptionIdType {
    VP_DEVICE_OPTION_ID_PULSE            = 0x00,
    VP_DEVICE_OPTION_ID_CRITICAL_FLT     = 0x01,
    VP_OPTION_ID_ZERO_CROSS              = 0x02,
    VP_DEVICE_OPTION_ID_RAMP2STBY        = 0x03,
    VP_OPTION_ID_PULSE_MODE              = 0x04,
    VP_OPTION_ID_TIMESLOT                = 0x05,
    VP_OPTION_ID_CODEC                   = 0x06,
    VP_OPTION_ID_PCM_HWY                 = 0x07,
    VP_OPTION_ID_LOOPBACK                = 0x08,
    VP_OPTION_ID_LINE_STATE              = 0x09,
    VP_OPTION_ID_EVENT_MASK              = 0x0A,
    VP_OPTION_ID_RESERVED_1              = 0x0B,
    VP_OPTION_ID_RING_CNTRL              = 0x0C,
    VP_OPTION_ID_RESERVED_2              = 0x0D,
    VP_OPTION_ID_DTMF_MODE               = 0x0E,
    VP_DEVICE_OPTION_ID_DEVICE_IO        = 0x0F,
    VP_OPTION_ID_RESERVED_EVENT_MASK_VCP = 0x10,
    VP_OPTION_ID_PCM_TXRX_CNTRL          = 0x11,
    VP_DEVICE_OPTION_ID_PULSE2           = 0x12,
    VP_OPTION_ID_LINE_IO_CFG             = 0x13,
    VP_DEVICE_OPTION_ID_DEV_IO_CFG       = 0x14,
    VP_OPTION_ID_DTMF_SPEC               = 0x15,
    VP_DEVICE_OPTION_ID_PARK_MODE        = 0x16,
    VP_OPTION_ID_DCFEED_SLOPE            = 0x17,
    VP_OPTION_ID_SWITCHER_CTRL           = 0x18,
    VP_OPTION_ID_HOOK_DETECT_MODE        = 0x19,
    VP_OPTION_ID_AUTO_LOOP_COND          = 0x1A,
    VP_OPTION_ID_DCFEED_PARAMS           = 0x1B,
    VP_OPTION_ID_RINGING_PARAMS          = 0x1C,
    VP_OPTION_ID_GND_FLT_PROTECTION      = 0x1D,
    VP_DEVICE_OPTION_ID_ADAPTIVE_RINGING = 0x1E,
    VP_OPTION_ID_RINGTRIP_CONFIRM        = 0x1F,
    VP_DEVICE_OPTION_ID_RING_PHASE_SYNC  = 0x20,
    VP_OPTION_ID_HIGHPASS_FILTER         = 0x21,
    VP_DEVICE_OPTION_ID_FSYNC_RATE       = 0x22,
    VP_OPTION_ID_DTMF_PARAMS             = 0x23,
    VP_OPTION_ID_PULSE                   = 0x24,
    VP_OPTION_ID_DEBUG_SELECT            = 0x25,
    VP_OPTION_ID_ABS_GAIN                = 0x26,
    VP_DEVICE_OPTION_ID_PCM_SIG_CTL      = 0x27,
    VP_OPTION_ID_LINESTATE_CTL_MODE      = 0x28,

    VP_NUM_OPTION_IDS
} VpOptionIdType;

/** Parameters for dial pulse, flash, and on-hook */
typedef struct VpOptionPulseType {
    uint16 breakMin;        /**< Minimum pulse break time (in 125uS) */
    uint16 breakMax;        /**< Maximum pulse break time (in 125uS) */
    uint16 makeMin;         /**< Minimum pulse make time (in 125uS) */
    uint16 makeMax;         /**< Maximum pulse make time (in 125uS) */
    uint16 interDigitMin;   /**< Minimum pulse interdigit time (in 125uS) */
    uint16 flashMin;        /**< Minimum flash break time (in 125uS) */
    uint16 flashMax;        /**< Maximum flash break time (in 125uS) */
#ifdef EXTENDED_FLASH_HOOK
    uint16 onHookMin;       /**< Minimum on-hook time (in 125uS) */
#endif
#ifdef VP_ENABLE_OFFHOOK_MIN
    uint16 offHookMin;      /**< Minimum off-hook time (in 125uS) */
#endif
} VpOptionPulseType;

typedef struct VpOptionLinePulseType {
    uint16 breakMin;        /**< Minimum pulse break time (in 125uS) */
    uint16 breakMax;        /**< Maximum pulse break time (in 125uS) */
    uint16 makeMin;         /**< Minimum pulse make time (in 125uS) */
    uint16 makeMax;         /**< Maximum pulse make time (in 125uS) */
    uint16 interDigitMin;   /**< Minimum pulse interdigit time (in 125uS) */
    uint16 flashMin;        /**< Minimum flash break time (in 125uS) */
    uint16 flashMax;        /**< Maximum flash break time (in 125uS) */
    uint16 onHookMin;       /**< Minimum on-hook time (in 125uS) */
    uint16 offHookMin;      /**< Minimum off-hook time (in 125uS) */
} VpOptionLinePulseType;

/** Method for line control when critical faults are detected */
typedef struct VpOptionCriticalFltType {
    /**< The line is set to disconnect when the specified fault is active and
     * the "En" bit is set TRUE
     */
    bool acFltDiscEn;           /**< AC fault detected */
    bool dcFltDiscEn;           /**< DC fault detected */
    bool thermFltDiscEn;        /**< Thermal fault detected */
} VpOptionCriticalFltType;

/** Method for zero-cross control */
typedef enum VpOptionZeroCrossType {
    VP_OPTION_ZC_M4B  = 0, /**< Zero-Cross On - Make before break */
    VP_OPTION_ZC_B4M  = 1, /**< Zero-Cross On - Break before make */
    VP_OPTION_ZC_NONE = 2, /**< Turn Zero-Cross control off */
    VP_OPTION_ZC_ENUM_RSVD = FORCE_SIGNED_ENUM,
    VP_OPTION_ZC_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req. */
} VpOptionZeroCrossType;

/** Dial Pulse decode enable/disable */
typedef enum VpOptionPulseModeType {
    VP_OPTION_PULSE_DECODE_OFF      = 0, /**< Disable Pulse Decode */
    VP_OPTION_PULSE_DECODE_ON       = 1, /**< Enable Pulse Decode */
    VP_OPTION_PULSE_DECODE_BRIDGING = 2, /**< Enable Pulse Decode (With Bridging) */
    VP_OPTION_PULSE_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpOptionPulseModeType;

/** Transmit/Receive Timeslot setting (timeslot and control) */
typedef struct VpOptionTimeslotType {
    uint8 tx;   /**< 8-bit TX timeslot */
    uint8 rx;   /**< 8-bit RX timeslot */
} VpOptionTimeslotType;

typedef enum VpOptionCodecType {
    VP_OPTION_ALAW              = 0, /**< Select G.711 A-Law PCM encoding */
    VP_OPTION_MLAW              = 1, /**< Select G.711 Mu-Law PCM encoding */
    VP_OPTION_LINEAR            = 2, /**< Select Linear PCM encoding */
    VP_OPTION_WIDEBAND          = 3, /**< Select Wideband Linear PCM encoding */
    VP_OPTION_LINEAR_WIDEBAND   = VP_OPTION_WIDEBAND,
    VP_OPTION_ALAW_WIDEBAND     = 4, /**< Select Wideband A-Law PCM encoding */
    VP_OPTION_MLAW_WIDEBAND     = 5, /**< Select Wideband Mu-Law PCM encoding */
    VP_NUM_OPTION_CODEC_TYPE_IDS,
    VP_OPTION_CODEC_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpOptionCodecType;

/** PCM Highway Selection (B valid on select devices only) */
typedef enum VpOptionPcmHwyType {
    VP_OPTION_HWY_A          = 0, /**< Select the 'A' PCM Highway */
    VP_OPTION_HWY_B          = 1, /**< Select the 'B' PCM Highway */
    VP_OPTION_HWY_TX_A_RX_B  = 2, /**< Transmit on Highway A, receive on B */
    VP_OPTION_HWY_TX_B_RX_A  = 3, /**< Transmit on Highway A, receive on A */
    VP_OPTION_HWY_TX_AB_RX_A = 4, /**< Transmit on Highway A and B, receive on A */
    VP_OPTION_HWY_TX_AB_RX_B = 5, /**< Transmit on Highway A and B, receive on B */
    VP_OPTION_HWY_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpOptionPcmHwyType;

/** Loopback option selection */
typedef enum VpOptionLoopbackType {
    VP_OPTION_LB_OFF      = 0, /**< All loopbacks off */

    /* Following loopback options are supported for CSLAC and VCP only */
    VP_OPTION_LB_TIMESLOT = 1, /**< Perform a timeslot loopback */
    VP_OPTION_LB_DIGITAL  = 2, /**< Perform a full-digital loopback */
    VP_OPTION_LB_CHANNELS = 3, /**< Connects FXO to FXS line on same device */

    VP_NUM_LB_OPTIONS,
    VP_OPTION_LB_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpOptionLoopbackType;

/** Active Line State battery supply selection */
typedef enum VpOptionBatType {
    VP_OPTION_BAT_AUTO  = 0, /**< Automatic Batery selection */
    VP_OPTION_BAT_HIGH  = 1, /**< Use High Batery */
    VP_OPTION_BAT_LOW   = 2, /**< Use Low Batery */
    VP_OPTION_BAT_BOOST = 3, /**< Include Positive Batery */
    VP_NUM_OPTION_BAT_IDS,
    VP_OPTION_BAT_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpOptionBatType;

/** Active Line State battery supply selection */
typedef struct VpOptionLineStateType {
    bool battRev;       /**< Smooth/Abrupt Battery Reversal (TRUE = abrupt) */

    VpOptionBatType bat;    /**< Battery selection for Active line state */
} VpOptionLineStateType;

/** Ring control option */
typedef enum VpLineStateType {

    /* FXS */
    VP_LINE_STANDBY         = 0x00, /**< Low power line feed state */
    VP_LINE_TIP_OPEN        = 0x01, /**< Tip open circuit state */
    VP_LINE_ACTIVE          = 0x02, /**< Line Feed w/out VF */
    VP_LINE_ACTIVE_POLREV   = 0x03, /**< Polarity Reversal Line Feed w/out VF */
    VP_LINE_TALK            = 0x04, /**< Normal off-hook Active State; Voice Enabled */
    VP_LINE_TALK_POLREV     = 0x05, /**< Normal Active with reverse polarity; Voice Enabled */
    VP_LINE_OHT             = 0x06, /**< On-Hook tranmission state */
    VP_LINE_OHT_POLREV      = 0x07, /**< Polarity Reversal On-Hook tranmission state */
    VP_LINE_DISCONNECT      = 0x08, /**< Denial of service */
    VP_LINE_RINGING         = 0x09, /**< Ringing state */
    VP_LINE_RINGING_POLREV  = 0x0A, /**< Ringing w/Polarity Reversal */
    VP_LINE_STANDBY_POLREV  = 0x10, /**< Low power polrev line feed state */
    VP_LINE_PARK            = 0x11, /**< Park mode */
    VP_LINE_RING_OPEN       = 0x12, /**< Ring open */
    VP_LINE_HOWLER          = 0x13, /**< Howler */
    VP_LINE_TESTING         = 0x14, /**< Testing */
    VP_LINE_DISABLED        = 0x15, /**< Disabled */
    VP_LINE_NULLFEED        = 0x16, /**< Null-feed */
    VP_LINE_HOWLER_PCM      = 0x17, /**< Howler with PCM highway enabled */
    VP_LINE_HOWLER_POLREV   = 0x18, /**< Howler with Polarity Reversal */

    /* FXO */
    VP_LINE_FXO_OHT,        /**< FXO Line providing Loop Open w/VF */
    VP_LINE_FXO_LOOP_OPEN,  /**< FXO Line providing Loop Open w/out VF */
    VP_LINE_FXO_LOOP_CLOSE, /**< FXO Line providing Loop Close w/out VF */
    VP_LINE_FXO_TALK,       /**< FXO Line providing Loop Close w/VF */
    VP_LINE_FXO_RING_GND,   /**< FXO Line providing Ring Ground (GS only)*/

    VP_NUM_LINE_STATES,
    VP_LINE_STATE_ENUM_RSVD = FORCE_SIGNED_ENUM,
    VP_LINE_STATE_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpLineStateType;

typedef struct VpOptionRingControlType {
    VpOptionZeroCrossType zeroCross;    /**< LCAS zero cross control */

    uint16 ringExitDbncDur; /**< Ringing Exit Debounce Duration; Used during end
                             * of ON periods of ringing cadences; 125uS
                             * resolution
                             */

    VpLineStateType ringTripExitSt; /**< State to automatically switch to upon
                                     * ring trip
                                     */
}  VpOptionRingControlType;

/** DTMF detection option */
typedef enum VpOptionDtmfModeControlType {
    VP_OPTION_DTMF_DECODE_OFF = 0, /**< Disable DTMF Digit Decode */
    VP_OPTION_DTMF_DECODE_ON  = 1, /**< Enable DTMF Digit  Decode */
    VP_OPTION_DTMF_GET_STATUS = 2, /**< Do not change anything; Just get the DTMF status  */
    VP_NUM_OPTION_DTMF_IDS,
    VP_OPTION_DTMF_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpOptionDtmfModeControlType;

/* Device I/O Option related definitions */
typedef enum VpDeviceIoDirectionType {
    VP_IO_INPUT_PIN  = 0, /* Configure GPIO pin as input pin */
    VP_IO_OUTPUT_PIN = 1, /* Configure GPIO pin as output pin */
    VP_IO_DIR_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpDeviceIoDirectionType;

typedef enum VpDeviceOutputPinType {
    VP_OUTPUT_DRIVEN_PIN = 0, /* Configure as TTL/CMOS output pin */
    VP_OUTPUT_OPEN_PIN   = 1, /* Configure as open collector/drain output pin */
    VP_OUTPUT_TYPE_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpDeviceOutputPinType;

typedef struct VpOptionDeviceIoType {
    uint32 directionPins_31_0;      /* Device specific IO pin direction
                                     * (Pins 0 - 31) */
    uint32 directionPins_63_32;     /* Device specific IO pin direction
                                     * (Pins 32 - 63) */
    uint32 outputTypePins_31_0;     /* Output pin type (Pins 0 - 31) */
    uint32 outputTypePins_63_32;    /* Output pin type (Pins 32 - 63) */
} VpOptionDeviceIoType;

/* Definition for line I/O config option */
typedef struct VpOptionLineIoConfigType {
    uint8 direction;
    uint8 outputType;
} VpOptionLineIoConfigType;

/* Definition for device I/O config option */
typedef struct VpOptionDeviceIoConfigType {
    VpOptionLineIoConfigType lineIoConfig[VP_MAX_LINES_PER_DEVICE];
} VpOptionDeviceIoConfigType;

typedef enum VpOptionPcmTxRxCntrlType {
    VP_OPTION_PCM_BOTH      = 0, /* Enable both PCM transmit and receive paths */
    VP_OPTION_PCM_RX_ONLY   = 1, /* Enable PCM receive path only */
    VP_OPTION_PCM_TX_ONLY   = 2, /* Enable PCM transmit path only */
    VP_OPTION_PCM_ALWAYS_ON = 3, /* Prevents disabling of PCM path */
    VP_OPTION_PCM_OFF       = 4, /* Disable both PCM transmit and receive */
    VP_PCM_TXRX_CNTRL_ENUM_RSVD = FORCE_SIGNED_ENUM,
    VP_PCM_TXRX_CNTRL_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpOptionPcmTxRxCntrlType;

/** Direction Specification */
typedef enum VpDirectionType {
    VP_DIRECTION_DS,
    VP_DIRECTION_US,
    VP_DIRECTION_INVALID,   /**< Used by the API to determine if the direction
                             * field is valid */
    VP_DIRECTION_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpDirectionType;

#define VP_LINE_FLAG_BYTES ((VP_MAX_LINES_PER_DEVICE + 7) / 8)

/** DTMF detection option control */
typedef struct VpOptionDtmfModeType {
    VpOptionDtmfModeControlType dtmfControlMode; /**< DTMF detection
                                                  * Enable/Disable */
    VpDirectionType direction;                  /**< Detection direction */
    uint32 dtmfDetectionSetting;                /**< Indicates the DTMF
                                                 *   detection setting for first
                                                 *    32 lines */
    uint8 dtmfResourcesRemaining;               /**< DTMF decoder resources
                                                 *   remaining */
    uint8 dtmfDetectionEnabled[VP_LINE_FLAG_BYTES];
                                                /**< DTMF detection setting for
                                                 *   lines 7-0, 15-8, etc. */
} VpOptionDtmfModeType;

/** Regional DTMF Specs */
typedef enum VpOptionDtmfSpecType {
    VP_OPTION_DTMF_SPEC_ATT  = 0, /* Q.24 AT&T */
    VP_OPTION_DTMF_SPEC_NTT  = 1, /* Q.24 NTT */
    VP_OPTION_DTMF_SPEC_AUS  = 2, /* Q.24 Australian */
    VP_OPTION_DTMF_SPEC_BRZL = 3, /* Q.24 Brazilian */
    VP_OPTION_DTMF_SPEC_ETSI = 4, /* ETSI ES 201 235-3 v1.3.1 */
    VP_OPTION_DTMF_SPEC_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpOptionDtmfSpecType;

/**< The following types are for Park Mode options */
typedef struct VpOptionParkModeType {
    uint16 discTime;    /**< Specified in 500ms increments, up to 8 seconds */
    uint16 standbyTime; /**< Specified in 100ms increments, up to 8 seconds */
} VpOptionParkModeType;

/** Hook detection modes  */
typedef enum VpOptionHookDetectModeType {
    VP_OPTION_HOOKDET_NORMAL         = 0,  /* normal hook detection behavior */
    VP_OPTION_HOOKDET_DISC_IS_ONHOOK = 1,  /* in the VP_LINE_DISCONNECT or VP_LINE_DISABLED
                                              state, the hook status is always considered
                                              to be on-hook */
    VP_OPTION_HOOKDET_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpOptionHookDetectModeType;

/* Option value struct for VP_OPTION_ID_AUTO_LOOP_COND: */
typedef enum VpLoopCondSelectType {
    VP_LOOP_COND_NONE        = 0x0000, /* feature disabled */
    VP_LOOP_COND_RLOOP       = 0x0001, /* read loop resistance */
    VP_LOOP_COND_ILG         = 0x0002, /* read longitudinal (common mode) current */
    VP_LOOP_COND_IMT         = 0x0004, /* read metallic (differential) current */
    VP_LOOP_COND_VSAB        = 0x0008, /* read metallic (differential) voltage */
    VP_LOOP_COND_SELECTEDBAT = 0x0010, /* report currently-selected battery */
    VP_LOOP_COND_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpLoopCondSelectType;

typedef struct VpOptionAutoLoopCondType {
    uint16 select;  /* bitmask (multiple VpLoopCondSelectType values can be ORed
                       together */
    uint16 delay;   /* amount of time (in ms) after HOOK_OFF before
                       measurement(s) */
} VpOptionAutoLoopCondType;

/* Definition for Absolute Level option */
typedef struct VpOptionAbsGainType {
    int16 gain_AToD;
    int16 gain_DToA;
} VpOptionAbsGainType;

#define VP_OPTION_ABS_GAIN_QUIET        (-32767)
#define VP_OPTION_ABS_GAIN_RESTORE      (-32768)
#define VP_OPTION_ABS_GAIN_NO_CHANGE    (+32767)
#define VP_ABS_GAIN_UNKNOWN             (VP_OPTION_ABS_GAIN_NO_CHANGE - 1)

/* Option value struct for VP_DEVICE_OPTION_ID_PCM_SIG_CTL: */
typedef struct VpOptionPcmSigCtlType {
    bool enable;
    uint8 ctlTimeslot;
    uint8 sigTimeslot;
} VpOptionPcmSigCtlType;

/* Option value struct for VP_OPTION_ID_LINESTATE_CTL_MODE: */
typedef enum VpOptionLinestateCtlModeType {
    VP_OPTION_LINESTATE_CTL_NORMAL     = 0x0000,
    VP_OPTION_LINESTATE_CTL_PCM        = 0x0080
} VpOptionLinestateCtlModeType;

/* Option value struct for VP_OPTION_ID_DCFEED_PARAMS: */
typedef struct VpOptionDcFeedParamsType {
    uint32 validMask;
    int32 voc;
    int32 ila;
    int32 hookThreshold;
    int32 lpHookThreshold;
    int32 gkeyThreshold;
    int32 battFloor;
} VpOptionDcFeedParamsType;

#define VP_OPTION_CFG_VOC                       (0x00000001L)
#define VP_OPTION_CFG_ILA                       (0x00000002L)
#define VP_OPTION_CFG_HOOK_THRESHOLD            (0x00000004L)
#define VP_OPTION_CFG_LP_HOOK_THRESHOLD         (0x00000008L)
#define VP_OPTION_CFG_GKEY_THRESHOLD            (0x00000010L)
#define VP_OPTION_CFG_BATT_FLOOR                (0x00000020L)

/* Option value struct for VP_OPTION_ID_RINGING_PARAMS: */
typedef struct VpOptionRingingParamsType {
    uint32 validMask;
    int32 frequency;
    int32 amplitude;
    int32 dcBias;
    int32 ringTripThreshold;
    int32 ringCurrentLimit;
    int32 trapRiseTime;
} VpOptionRingingParamsType;

#define VP_OPTION_CFG_FREQUENCY                 (0x00000001L)
#define VP_OPTION_CFG_AMPLITUDE                 (0x00000002L)
#define VP_OPTION_CFG_DC_BIAS                   (0x00000004L)
#define VP_OPTION_CFG_RINGTRIP_THRESHOLD        (0x00000008L)
#define VP_OPTION_CFG_RING_CURRENT_LIMIT        (0x00000010L)
#define VP_OPTION_CFG_TRAP_RISE_TIME            (0x00000020L)

/* Mode values for VP_DEVICE_OPTION_ID_ADAPTIVE_RINGING */
typedef enum VpAdaptiveRingingModeType {
    VP_ADAPT_RING_SHARED_TRACKER    = 0x0,  /* For shared tracker supplies */
    VP_ADAPT_RING_SHARED_BB_ABS     = 0x1,  /* For Buck Boost ABS supplies */
    VP_ADAPT_RING_SINGLE_BB_TRACKER = 0x2,  /* For Buck Boost single channel tracker supplies */
    VP_ADAPT_RING_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpAdaptiveRingingModeType;

/* Option value struct for VP_DEVICE_OPTION_ID_ADAPTIVE_RINGING: */
typedef struct VpOptionAdaptiveRingingType {
    uint32 validMask;
    uint8 power;
    uint8 minVoltagePercent;
    VpAdaptiveRingingModeType mode;
} VpOptionAdaptiveRingingType;

#define VP_ADAPTIVE_RINGING_CFG_POWER       (0x0001L)
#define VP_ADAPTIVE_RINGING_CFG_MIN_V_PCT   (0x0002L)
#define VP_ADAPTIVE_RINGING_CFG_MODE        (0x0004L)

/* To disable VP_DEVICE_OPTION_ID_ADAPTIVE_RINGING, 'power' can be set to 0xFF. */
#define VP_ADAPTIVE_RINGING_DISABLED 0xFF

/* Option value struct for VP_OPTION_ID_GND_FLT_PROTECTION: */
typedef struct VpOptionGndFltProtType {
    bool enable;
    uint16 confirmTime; /* 10ms units */
    uint16 pollTime;    /* 10ms units */
    uint16 pollNum;
} VpOptionGndFltProtType;

/* Option values for VP_DEVICE_OPTION_ID_RING_PHASE_SYNC: */
typedef enum VpOptionRingPhaseSyncType {
    VP_RING_PHASE_SYNC_DISABLE = 0,
    VP_RING_PHASE_SYNC_90_OFFSET = 1    /* 90 degree offset between channels */
} VpOptionRingPhaseSyncType;

/* Option values for VP_OPTION_ID_HIGHPASS_FILTER: */
typedef enum VpOptionHighPassFilterType {
    VP_HIGHPASS_FILTER_DISABLE = 0,
    VP_HIGHPASS_FILTER_ENABLE = 1
} VpOptionHighPassFilterType;

/* Option values for VP_DEVICE_OPTION_ID_FSYNC_RATE: */
typedef enum VpOptionFsyncRateType {
    VP_FSYNC_RATE_8KHZ = 0,
    VP_FSYNC_RATE_16KHZ = 1
} VpOptionFsyncRateType;

/* Option values for VP_OPTION_ID_DTMF_PARAMS: */
typedef struct VpOptionDtmfParamsType {
    uint32 validMask;
    int32 minDetect;
    int32 rowToColLimit;
    int32 colToRowLimit;
} VpOptionDtmfParamsType;

#define VP_OPTION_CFG_MIN_DETECT                (0x00000001L)
#define VP_OPTION_CFG_ROW_TO_COL_LIMIT          (0x00000002L)
#define VP_OPTION_CFG_COL_TO_ROW_LIMIT          (0x00000004L)

/* The following struct can be passed to VpGetResults() if the option ID is not
   known at compile time, to ensure that the buffer is large enough regardless
   of the option type. */
typedef union VpOptionValueType {
    VpOptionPulseType pulse;                       /* VP_DEVICE_OPTION_ID_PULSE        */
                                                   /* VP_DEVICE_OPTION_ID_PULSE2       */
    VpOptionCriticalFltType criticalFlt;           /* VP_DEVICE_OPTION_ID_CRITICAL_FLT */
    VpOptionZeroCrossType zeroCross;               /* VP_OPTION_ID_ZERO_CROSS          */
    uint16 ramp2stby;                              /* VP_DEVICE_OPTION_ID_RAMP2STBY    */
    VpOptionPulseModeType pulseMode;               /* VP_OPTION_ID_PULSE_MODE          */
    VpOptionTimeslotType timeslot;                 /* VP_OPTION_ID_TIMESLOT            */
    VpOptionCodecType codec;                       /* VP_OPTION_ID_CODEC               */
    VpOptionPcmHwyType pcmHwy;                     /* VP_OPTION_ID_PCM_HWY             */
    VpOptionLoopbackType loopback;                 /* VP_OPTION_ID_LOOPBACK            */
    VpOptionLineStateType lineState;               /* VP_OPTION_ID_LINE_STATE          */
    VpOptionEventMaskType eventMask;               /* VP_OPTION_ID_EVENT_MASK          */
    VpOptionRingControlType ringControl;           /* VP_OPTION_ID_RING_CNTRL          */
    VpOptionDtmfModeType dtmfMode;                 /* VP_OPTION_ID_DTMF_MODE           */
    VpOptionDeviceIoType deviceIo;                 /* VP_DEVICE_OPTION_ID_DEVICE_IO    */
    VpOptionPcmTxRxCntrlType pcmTxRxCntrl;         /* VP_OPTION_ID_PCM_TXRX_CNTRL      */
    VpOptionDeviceIoConfigType deviceIoConfig;     /* VP_DEVICE_OPTION_ID_DEV_IO_CFG   */
    VpOptionLineIoConfigType lineIoConfig;         /* VP_OPTION_ID_LINE_IO_CFG         */
    VpOptionDtmfSpecType dtmfSpec;                 /* VP_OPTION_ID_DTMF_SPEC           */
    VpOptionParkModeType parkMode;                 /* VP_DEVICE_OPTION_ID_PARK_MODE    */
    VpOptionLinePulseType linePulse;               /* VP_OPTION_ID_PULSE               */
    uint16 dcFeedSlope;                            /* VP_OPTION_ID_DCFEED_SLOPE        */
    bool switcherCtrl;                             /* VP_OPTION_ID_SWITCHER_CTRL       */
    uint32 debugSelect;                            /* VP_OPTION_ID_DEBUG_SELECT        */
    VpOptionAbsGainType absGain;                   /* VP_OPTION_ID_ABS_GAIN            */
    VpOptionHookDetectModeType hookDetectMode;     /* VP_OPTION_ID_HOOK_DETECT_MODE    */
    VpOptionPcmSigCtlType pcmSigCtl;               /* VP_DEVICE_OPTION_ID_PCM_SIG_CTL  */
    VpOptionLinestateCtlModeType linestateCtlMode; /* VP_OPTION_ID_LINESTATE_CTL_MODE  */
    VpOptionAutoLoopCondType autoLoopCond;         /* VP_OPTION_ID_AUTO_LOOP_COND      */
    VpOptionDcFeedParamsType dcFeedParams;         /* VP_OPTION_ID_DCFEED_PARAMS       */
    VpOptionRingingParamsType ringingParams;       /* VP_OPTION_ID_RINGING_PARAMS      */
    VpOptionGndFltProtType gndFltProt;             /* VP_OPTION_ID_GND_FLT_PROTECTION  */
    VpOptionAdaptiveRingingType adaptiveRinging;   /* VP_DEVICE_OPTION_ID_ADAPTIVE_RINGING */
    uint16 ringTripConfirm;                        /* VP_OPTION_ID_RINGTRIP_CONFIRM    */
    VpOptionRingPhaseSyncType ringPhaseSync;       /* VP_DEVICE_OPTION_ID_RING_PHASE_SYNC */
    VpOptionHighPassFilterType highPassFilter;     /* VP_OPTION_ID_HIGHPASS_FILTER     */
    VpOptionFsyncRateType fsyncRate;               /* VP_DEVICE_OPTION_ID_FSYNC_RATE   */
    VpOptionDtmfParamsType dtmfParams;             /* VP_OPTION_ID_DTMF_PARAMS         */
} VpOptionValueType;

#endif /* VP_API_OPTION */
