/** \file vp_CSLAC_types.h
 * vp_CSLAC_types.h
 *
 * Header file for internal variables used in CSLAC type API.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11488 $
 * $LastChangedDate: 2014-07-14 16:56:02 -0500 (Mon, 14 Jul 2014) $
 */

#ifndef VP_CSLAC_TYPES_H
#define VP_CSLAC_TYPES_H

#include "vp_api_fxo_params.h"
#include "vp_api_timer.h"

#define VP_CSLAC_DEV_PROFILE_VERSION_INT_SW_CONFIG_FR   5
#define VP_CSLAC_DEV_PROFILE_VERSION_INT_SW_CONFIG      4
#define VP_CSLAC_DEV_PROFILE_VERSION_LINECARD_PARAM     3
#define VP_CSLAC_DEV_PROFILE_VERSION_VBH_REG            2
#define VP_CSLAC_DEV_PROFILE_VERSION_SW_CONFIG          1

#define VP_CSLAC_FXO_VERSION_DTMF_LEVEL                 1
#define VP_CSLAC_FXO_VERSION_POH                        2

#define VP_CSLAC_DC_PROFILE_VERSION_890                 1

#define VP_CSLAC_RING_PROFILE_VERSION_890               1

#define VP_CSLAC_TICKSTEP_0_5MS     (0x80)      /**< CSLAC Tickrate for 0.5mS */
#define VP_CSLAC_TICKSTEP_1MS       (0x100)     /**< CSLAC Tickrate for 1mS */

#define VP_CID_TIMESCALE    1   /**< Timescale in mS for CID time data */

#define VP_CSLAC_DEV_PROF_TABLE_SIZE            1
#define VP_CSLAC_AC_PROF_TABLE_SIZE             2
#define VP_CSLAC_DC_PROF_TABLE_SIZE             2
#define VP_CSLAC_RINGING_PROF_TABLE_SIZE        2
#define VP_CSLAC_TONE_CADENCE_PROF_TABLE_SIZE   11
#define VP_CSLAC_TONE_PROF_TABLE_SIZE           10
#define VP_CSLAC_RING_CADENCE_PROF_TABLE_SIZE   4
#define VP_CSLAC_METERING_PROF_TABLE_SIZE       2
#define VP_CSLAC_CALLERID_PROF_TABLE_SIZE       2
#define VP_CSLAC_FXO_CONFIG_PROF_TABLE_SIZE     1
#define VP_CSLAC_CUSTOM_TERM_PROF_TABLE_SIZE    4

/* Special Howler Bit-Mask 'xxx1 11xx' */
#define VP_CSLAC_SPECIAL_TONE_MASK      (0x1C)
#define VP_CSLAC_STD_TONE               (0x00)
#define VP_CSLAC_HOWLER_TONE            (0x04)  /* UK BTNR 1080 Specification */
#define VP_CSLAC_AUS_HOWLER_TONE        (0x08)
#define VP_CSLAC_NTT_HOWLER_TONE        (0x0C)
#define VP_CSLAC_UK_HOWLER_TONE_VER15   VP_CSLAC_HOWLER_TONE
#define VP_CSLAC_UK_HOWLER_TONE_DRAFT_G (0x10)

#define VP_CSLAC_TONE_TYPE         5

#define VP_CSLAC_READ_RESPONSE_MASK (VP_LINE_EVID_LLCMD_RX_CMP \
                                   | VP_LINE_EVID_RD_OPTION \
                                   | VP_LINE_EVID_GAIN_CMP)

typedef struct VpCSLACDeviceProfileTableType {
    VpProfilePtrType pDevProfileTable[VP_CSLAC_DEV_PROF_TABLE_SIZE];
    VpProfilePtrType pAcProfileTable[VP_CSLAC_AC_PROF_TABLE_SIZE];
    VpProfilePtrType pDcProfileTable[VP_CSLAC_DC_PROF_TABLE_SIZE];
    VpProfilePtrType pRingingProfileTable[VP_CSLAC_RINGING_PROF_TABLE_SIZE];
    VpProfilePtrType pToneCadProfileTable[VP_CSLAC_TONE_CADENCE_PROF_TABLE_SIZE];
    VpProfilePtrType pToneProfileTable[VP_CSLAC_TONE_PROF_TABLE_SIZE];
    VpProfilePtrType pRingingCadProfileTable[VP_CSLAC_RING_CADENCE_PROF_TABLE_SIZE];
    VpProfilePtrType pMeteringProfileTable[VP_CSLAC_METERING_PROF_TABLE_SIZE];
    VpProfilePtrType pCallerIdProfileTable[VP_CSLAC_CALLERID_PROF_TABLE_SIZE];
    VpProfilePtrType pFxoConfigProfileTable[VP_CSLAC_FXO_CONFIG_PROF_TABLE_SIZE];
    VpProfilePtrType pCustomTermProfileTable[VP_CSLAC_CUSTOM_TERM_PROF_TABLE_SIZE];
} VpCSLACDeviceProfileTableType;

/**
 * The following structure is used by the device as a bitmask indicating
 * whether or not a given profile table entry is valid. In the bit location a
 * value '1' = profile is current, '0' = profile is not current.
 */
typedef struct VpCSLACProfileTableEntryType {
    uint16 devProfEntry;
    uint16 acProfEntry;
    uint16 dcProfEntry;
    uint16 ringingProfEntry;
    uint16 toneCadProfEntry;
    uint16 toneProfEntry;
    uint16 ringCadProfEntry;
    uint16 meterProfEntry;
    uint16 cidCadProfEntry;
    uint16 fxoConfigProfEntry;
    uint16 customTermProfEntry;
} VpCSLACProfileTableEntryType;

#define VP_PROF_ENTRY_INVALID   0
#define VP_PROF_ENTRY_VALID     1

typedef union VpGetResultsOptionsDataType {
    VpOptionCriticalFltType criticalFaultOption;
    VpOptionEventMaskType eventMaskOption;
    VpOptionPulseModeType pulseModeOption;
    VpOptionPulseType pulseTypeOption;
    VpOptionRingControlType ringControlOption;
    VpOptionZeroCrossType zeroCross;
    VpOptionTimeslotType timeSlotOption;
    VpOptionCodecType codecOption;
    VpOptionPcmHwyType pcmHwyOption;
    VpOptionLoopbackType loopBackOption;
    VpOptionLineStateType lineStateOption;
    VpOptionPcmTxRxCntrlType pcmTxRxCtrl;
    VpDeviceIoAccessDataType deviceIoData;
    VpOptionDeviceIoType deviceIo;
    VpLoopCondResultsType loopCond;
    bool autoShutdownEn;
    VpOptionAbsGainType absGain;
    VpOptionDcFeedParamsType dcFeedParams;
    VpOptionRingingParamsType ringingParams;
} VpGetResultsOptionsDataType;

typedef struct VpGetResultsOptionsType {
    VpGetResultsOptionsDataType optionData;
    uint8 chanId;
    VpOptionIdType optionType;
} VpGetResultsOptionsType;

/* Caller ID DTMF On/Off Intervals
 * ================================
 * The macros below are specified in ms with the following conditions to consider:
 *   - Minimum On/Off-Time in ETSI Standards (reference?) is 65ms defined as the time to transition
 *     from the 10% to 90% level and vise-versa. There's a contribution of the silicon to this time
 *     factor that should be taken into consideration.
 *   - VP-API-II Supports Host Processing Time/Jitter of up to 2ms. This is the limitation of the
 *     silicon hook-bit, test data, and caller id buffers.
 *
 * TECHNICAL: The theoretical minimum is 67ms in an ideal world - so 70ms is used primarily to
 *   increase room for error in non-ideal conditions. Since this requirement is a MIN value, the
 *   macro used must be MS_TO_TICKS_ROUND_UP. Applications using tickrates that result in (70/tick)
 *   to result in a non-integer value such as 8.33ms (e.g.) could generate output intervals as low
 *   as 66.7. Taking into account 2ms jitter reduces this to 64.7ms, below the >= 65ms requirement.
 *
 * SUMMARY: DTMF On/Off-Times both need to be generated with 70ms duration. Tick based devices
 * must use the rounding up macro (MS_TO_TICKS_ROUND_UP) in order to avoid unexpected rounding low
 * near the minimum threshold.
 */
#define VP_CID_DTMF_ON_TIME    70
#define VP_CID_DTMF_OFF_TIME   70

typedef enum VpCidGeneratorControlType {
    VP_CID_GENERATOR_DATA,
    VP_CID_GENERATOR_KEYED_CHAR,
    VP_CID_SIGGEN_EOT,
    VP_CID_NO_CHANGE,           /**< Load tones only, do not change generator state */
    VP_CID_GENERATOR_MARKOUT,
    VP_CID_CTRL_TYPE_CNT,

    VP_CID_GENC_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} VpCidGeneratorControlType;

/* Previously VpCidStatusFlagType values */
#define VP_CID_RESET_VALUE      (0x00000000L)

#define VP_CID_IS_DEBOUNCE      (0x00000001L)    /**< Set when debounce has expired */
#define VP_CID_CLIAWAITTONE     (0x00000002L)
#define VP_CID_REPEAT_MSG       (0x00000004L)
#define VP_CID_AWAIT_TONE       (0x00000008L)    /**< Set if waiting for CPE ACK Tone */

#define VP_CID_TERM_FSK         (0x00000010L)    /**< Set if in a Mark or Channel Seizure Interval */
#define VP_CID_END_OF_MSG       (0x00000020L)    /**< Set if API detects an end of
                                                  * message in the buffered data
                                                  */

#define VP_CID_FSK_GEN_VALID    (0x00000040L)    /**< Set if FSK Generator is programmed
                                                  * for Caller ID signals.
                                                  */

#define VP_CID_IN_PROGRESS      (0x00000080L)    /**< Set if Caller ID is in progress */
#define VP_CID_ACTIVE_RING_SEQ  (0x00000100L)    /**< Set if active CID Ring sequence */

#define VP_CID_PRIMARY_FULL     (0x00000200L)    /**< Set when primary buffer contains
                                                  * CID data
                                                  */
#define VP_CID_SECONDARY_FULL   (0x00000400L)    /**< Set when secondary buffer contains
                                                  * CID data
                                                  */
#define VP_CID_PRIMARY_IN_USE   (0x00000800L)    /**< Set when primary buffer in use */
#define VP_CID_SECONDARY_IN_USE (0x00001000L)    /**< Set when secondary buffer in use */
#define VP_CID_WAIT_ON_ACTIVE   (0x00002000L)    /**< Set when a "wait on" mode is
                                                  * active. Should only be cleared in
                                                  * the timer function (since currently
                                                  * support only for Wait on CID or time
                                                  */
#define VP_CID_MID_CHECKSUM     (0x00004000L)    /**< Set when a checksum value is being
                                                  * sent.
                                                  */
#define VP_CID_MUTE_ON          (0x00008000L)    /**< Set when a "Mute On" CID operator
                                                  * is active.
                                                  */
#define VP_CID_FSK_ACTIVE       (0x00010000L)   /**< Set while the FSK Generator is needed */


typedef enum VpCidDtmfStatusType {
    VP_CID_DTMF_RESET_VALUE = 0,
    VP_CID_ACTIVE_ON_TIME = 1,
    VP_CID_ACTIVE_OFF_TIME = 2
} VpCidDtmfStatusType;

/** Caller ID variables used by the line */
typedef struct VpCallerIdType {
    uint32 status;  /**< 'OR'ed combination of VpCidStatusFlagType */

    VpCidDtmfStatusType dtmfStatus; /**< Used only for DTMF CID Message Data */

    uint16 cliTimer;        /**< Time in increments of device ticks */
    uint8 currentData;      /**< Data to repeat on the line */
    uint16 cidCheckSum;     /**< Checksum to append to message data */

    uint8 cliDebounceTime;  /**< Hook debouce specified in device ticks */
    VpProfilePtrType pCliProfile;   /**< Currently running CID Profile */

    VpDigitType cliDetectTone1;
    VpDigitType cliDetectTone2;

    uint8 cliMPIndex;       /**< Current Index into primary message buffer */
    uint8 cliMSIndex;       /**< Current Index into secondary message buffer */

    uint8 cliIndex;         /**< Current Index into profile data */

    uint8 primaryBuffer[VP_SIZEOF_CID_MSG_BUFFER];
    uint8 secondaryBuffer[VP_SIZEOF_CID_MSG_BUFFER];

    uint8 primaryMsgLen;    /**< Length of data in the primary buffer */
    uint8 secondaryMsgLen;  /**< Length of data in the secondary buffer */
    VpDigitType digitDet;   /**< Stores the last DTMF digit detected during
                             * Caller ID (if enabled via tone detection).
                             */
    uint8 messageDataRemain;    /**< Tracks the total number of message bytes remaining */

    /*
     * At the end of FSK Message Data, some systems require a mark-out time. As
     * specified in the Profile Wizard, this is defined in # of bytes with each
     * byte corresponding to 8.33ms (1200 baud with 1 start + 1 stop + 8 data
     * bits = 10/1200 = 8.33ms).
     */
    uint8 markOutByteCount;     /* Retains the value specified in the profile */
    uint8 markOutByteRemain;    /* Current value remaining. Clean up when = 0 */
} VpCallerIdType;

typedef enum VpCadenceStatusType {
    VP_CADENCE_RESET_VALUE = 0x0,
    VP_CADENCE_STATUS_ACTIVE = 0x1,
    VP_CADENCE_STATUS_BRANCHING  = 0x2,
    VP_CADENCE_STATUS_MID_TIMER  = 0x4,
    VP_CADENCE_STATUS_SENDSIG  = 0x8,
    VP_CADENCE_STATUS_METERING = 0x10,
    VP_CADENCE_STATUS_BRANCHING_LVL2  = 0x20,
    VP_CADENCE_STATUS_IGNORE_POLARITY = 0x40,
    VP_CADENCE_STATUS_WAITING_ON_CID = 0x80
} VpCadenceStatusType;

/** Variables to control the cadence on the line using the Sequencer */
#define VP_CSLAC_MAX_BRANCH_DEPTH   2
#define VP_CSLAC_BRANCH_LVL_0       0
#define VP_CSLAC_BRANCH_LVL_1       1

typedef struct VpSeqDataType {
    VpProfilePtrType pActiveCadence;    /**< Currently Active Cadence */
    VpProfilePtrType pCurrentPos;       /**< Current position in profile */

    VpCadenceStatusType status;

    uint8 index;        /**< Index in the the cadence table   */
    uint8 length;       /**< Length of executable cadence instructions */

    /* Branch status info */
    uint8 count[VP_CSLAC_MAX_BRANCH_DEPTH];
    uint8 branchAt;

    uint16 timeRemain;
    uint16 meteringBurst;   /**< Tracks the number of metering pulses sent */
    bool meterPendingAbort; /**< Indicates that metering should abort after the
                             * current pulse ends */
    VpLineStateType meterAbortLineState; /**< The line state that should be set
                                          * when metering aborts */

    /* The following variables are for controlling signal generator ramp */
#define VPCSLAC_MAX_GENERATOR_DATA 11
    uint8 regData[VPCSLAC_MAX_GENERATOR_DATA];  /**< Scratchpad to reduce MPI
                                                 * traffic.
                                                 */
    /*
     * These values should correspond to device specific values for the
     * freqency/level being set -- not a generic value.
     */
    uint16 startFreq;
    bool isFreqIncrease;
    uint16 stopFreq;
    uint16 freqStep;

    uint16 startLevel;
    uint16 stopLevel;
    uint16 levelStep;
    uint8  toneType;
} VpSeqDataType;
/*
 * Used to save values of registers that will be modified for Special Howler Tones. These
 * tones require very high amplitude which can be done on VE880/890 using some back-door
 * controls. Note that VE880/890 have the same command set in the case of these registers
 */
#define VP_CSLAC_ICR1_LEN       (4)
#define VP_CSLAC_ICR2_LEN       (4)
#define VP_CSLAC_ICR3_LEN       (4)
#define VP_CSLAC_ICR4_LEN       (4)
#define VP_CSLAC_DC_FEED_LEN    (2)
#define VP_CSLAC_VP_GAIN_LEN    (1)
#define VP_CSLAC_GR_LEN         (2)
#define VP_CSLAC_R_LEN          (14)
#define VP_CSLAC_DISN_LEN       (1)
#define VP_CSLAC_OP_FUNC_LEN    (1)
#define VP_CSLAC_SW_REG_LEN     (3)

#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
typedef struct VpHowlerModeCachedValues {
    bool isInHowlerMode;    /**< Set to TRUE when in Howler Mode and values below are good */

    uint8 icr1Reg[VP_CSLAC_ICR1_LEN];   /**< ICR1 = 0xEC/ED */
    uint8 icr2Reg[VP_CSLAC_ICR2_LEN];   /**< ICR2 = 0xEE/EF */
    uint8 icr3Reg[VP_CSLAC_ICR3_LEN];   /**< ICR3 = 0xF2/F3 */
    uint8 icr4Reg[VP_CSLAC_ICR4_LEN];   /**< ICR4 = 0xEC/ED */
    uint8 dcFeed[VP_CSLAC_DC_FEED_LEN];

    uint8 digitalRxLoss[VP_CSLAC_VP_GAIN_LEN];  /**< VP Gain Reg - only changing Digital RX Loss */
    uint8 grValue[VP_CSLAC_GR_LEN];
    uint8 rValue[VP_CSLAC_R_LEN];

    uint8 disn[VP_CSLAC_DISN_LEN];

    /*
     * Z and B will be disabled using the Operating Function Register instead of writing
     * "disable" coefficients. Obviously - less MPI traffic by at least 33 bytes
     */
    uint8 opFunc[VP_CSLAC_OP_FUNC_LEN]; /**< Only disabling Z and B Filters */

    uint8 swReg[VP_CSLAC_SW_REG_LEN];   /**< Used to change Tracking floor voltge to -30V */
} VpHowlerModeCachedValues;
#endif

/** Variables to control the CID sequence on the line using the CID Sequencer */
typedef struct VpCidSeqDataType {
    VpProfilePtrType pActiveCadence;    /**< Currently Active Cadence */
    VpProfilePtrType pCurrentPos;       /**< Current position in profile */

    uint8 index;        /**< Index in the the cadence table   */
    uint16 timeRemain;
} VpCidSeqDataType;

typedef VpStatusType
(*VpSeqInstructionFuncPtrType) (
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pSequenceData);

/**< VP_SEQ_OPERATOR_MASK, VP_SEQ_SUBTYPE_MASK and VP_SEQUENCER_TICKRATE
 * First two defines are bit-wise masks to be used with Profile Bytes. The last value
 * VP_SEQUENCER_TICKRATE is a mathematical value of the time in deviceProfile tick scale of the
 * time steps of the sequencer (5ms).
 */
#define VP_SEQ_OPERATOR_MASK    0xE0
#define VP_SEQ_SUBTYPE_MASK     0x1F
#define VP_SEQUENCER_TICKRATE   0x0500  /**< 5mS -> Per-Step time of the Profile Wizard "Time" Cmd
                                         * scaled to the Device Tickrate */


/**< The super commands are the upper 3 bits of the command byte and therefore should never be
 * used with types exceeding 8-bit
 */
#define VP_SEQ_SPRCMD_COMMAND_INSTRUCTION   (0x00)
#define VP_SEQ_SPRCMD_TIME_INSTRUCTION      (0x20)
#define VP_SEQ_SPRCMD_BRANCH_INSTRUCTION    (0x40)

/**< The sub commands are the lower 5 bits of the command byte */
#define VP_SEQ_SUBCMD_SIGGEN            (0x00)
#define VP_SEQ_SUBCMD_LINE_STATE        (0x01)
#define VP_SEQ_SUBCMD_START_CID         (0x02)
#define VP_SEQ_SUBCMD_WAIT_ON           (0x07)  /**< Wait for CID or time to continue */

#define VP_SEQ_SUBCMD_RAMP_GENERATORS   (0x08)  /**< Forces frequency/amplitude change of the tone
                                                 * generator. Tone Cadence associated with this
                                                 * command must be specified as "ramp" type.
                                                 */

    /**< API Internal Use only operators */
#define VP_SEQ_SUBCMD_METERING      (0x10)  /**< Cadence Metering */

    /**< Bit-Fields for Tone Generator Control (see Profile Wizard Spec). */
#define VP_SEQ_SIGGEN_ALL_DISABLED  (0x00)
#define VP_SEQ_SIGGEN_A_EN          (0x01)
#define VP_SEQ_SIGGEN_B_EN          (0x02)
#define VP_SEQ_SIGGEN_C_EN          (0x04)
#define VP_SEQ_SIGGEN_D_EN          (0x08)

/**< Definition for the states in the Profile Wizard output. These must always be used/accessed
 * as uint8 types. Do everything possible to create compiler errors if they're used any differently
 */
typedef uint8 VpProfileCadencerStateTypes;
#define VP_PROFILE_CADENCE_STATE_STANDBY            ((VpProfileCadencerStateTypes)0)
#define VP_PROFILE_CADENCE_STATE_TIP_OPEN           ((VpProfileCadencerStateTypes)1)
#define VP_PROFILE_CADENCE_STATE_TALK               ((VpProfileCadencerStateTypes)2)
#define VP_PROFILE_CADENCE_STATE_ACTIVE             ((VpProfileCadencerStateTypes)3)
#define VP_PROFILE_CADENCE_STATE_RSVD               ((VpProfileCadencerStateTypes)4)
#define VP_PROFILE_CADENCE_STATE_OHT                ((VpProfileCadencerStateTypes)5)
#define VP_PROFILE_CADENCE_STATE_DISCONNECT         ((VpProfileCadencerStateTypes)6)
#define VP_PROFILE_CADENCE_STATE_RINGING            ((VpProfileCadencerStateTypes)7)
#define VP_PROFILE_CADENCE_STATE_POLREV_STANDBY     ((VpProfileCadencerStateTypes)8)
#define VP_PROFILE_CADENCE_STATE_POLREV_TIP_OPEN    ((VpProfileCadencerStateTypes)9)
#define VP_PROFILE_CADENCE_STATE_POLREV_TALK        ((VpProfileCadencerStateTypes)10)
#define VP_PROFILE_CADENCE_STATE_POLREV_ACTIVE      ((VpProfileCadencerStateTypes)11)
#define VP_PROFILE_CADENCE_STATE_POLREV_RSVD        ((VpProfileCadencerStateTypes)12)
#define VP_PROFILE_CADENCE_STATE_POLREV_OHT         ((VpProfileCadencerStateTypes)13)
#define VP_PROFILE_CADENCE_STATE_POLREV_DISCONNECT  ((VpProfileCadencerStateTypes)14)
#define VP_PROFILE_CADENCE_STATE_POLREV_RINGING     ((VpProfileCadencerStateTypes)15)
#define VP_PROFILE_CADENCE_STATE_FXO_OHT            ((VpProfileCadencerStateTypes)16)
#define VP_PROFILE_CADENCE_STATE_FXO_LOOP_OPEN      ((VpProfileCadencerStateTypes)17)
#define VP_PROFILE_CADENCE_STATE_FXO_LOOP_CLOSE     ((VpProfileCadencerStateTypes)18)
#define VP_PROFILE_CADENCE_STATE_FXO_LOOP_TALK      ((VpProfileCadencerStateTypes)19)
#define VP_PROFILE_CADENCE_STATE_MSG_WAIT_NORM      ((VpProfileCadencerStateTypes)20)
#define VP_PROFILE_CADENCE_STATE_MSG_WAIT_POLREV    ((VpProfileCadencerStateTypes)21)
#define VP_PROFILE_CADENCE_STATE_UNKNOWN            ((VpProfileCadencerStateTypes)255)

/* struct VpDialPulseDetectType is for FXS lines when detecting digits */
typedef enum VpDialPulseDetectStatesType {
    VP_DP_DETECT_STATE_LOOP_OPEN = 0,
    VP_DP_DETECT_STATE_LOOP_CLOSE = 1,
    VP_DP_DETECT_STATE_IDLE = 2
} VpDialPulseDetectStatesType;

typedef struct VpDialPulseDetectType {
    int digits;
    uint16 lo_time;
    uint16 lc_time;
    VpDialPulseDetectStatesType state;   /**< Dial Pulse State Machine state */
    bool hookSt;
    uint8 signalingData;
} VpDialPulseDetectType;

/* struct VpDigitGenerationDataType is for FXO lines when generating digits */
typedef struct VpDigitGenerationDataType {
    uint16 dtmfOnTime;
    uint16 dtmfOffTime;
    uint8 breakTime;
    uint8 makeTime;
    uint16 flashTime;
    uint16 dpInterDigitTime;
    uint8 dtmfHighFreqLevel[2];
    uint8 dtmfLowFreqLevel[2];
} VpDigitGenerationDataType;

/** Virtual Device Registers - Used to reduce MPI accesses to the Device */
typedef struct VpVirtualDeviceReg {
    uint8 sigRegMSB;    /**< Signaling Register's Most Significant byte */
    uint8 sigRegLSB;    /**< Signaling Register's Least Significant byte */
    uint8 iMaskMSB;     /**< Interrupt mask for the current line */
    uint8 iMaskLSB;     /**< Interrupt mask for the current line */
    uint8 iMaskMSBSP;   /**< Interrupt mask for the current line */
    uint8 iMaskLSBSP;   /**< Interrupt mask for the current line */
    uint8 ccR1SP;
    uint8 ccR5SP;       /**< Scratchpad for CCR5 used during metering */
    uint8 ccR8SP;       /**< Scratchpad for CCR8 used during calibration */
    uint8 ioReg;        /**< IO Register for QSLAC SLIC states */
} VpVirtualDeviceReg;

/**<
 * IMPORTANT: Make sure to update the type used for "state" member inside
 * objects, structs, and passed to functions where the comment states the value
 * is a bit-mask of "VpDeviceBusyFlagsType". Seach by "VpDeviceBusyFlagsType".
 * There are no instance of this type itself.
 */
typedef enum VpDeviceBusyFlagsType {
    /**< Indicates if function is running from InitDevice. Not the same as in
     * the middle of an initialization process (including calibration). Used to
     * allow InitDevice function calling other functions to be enabled. Otherwise,
     * some functions will return "Device Not Initialized" error.
     */
    VP_DEV_INIT_IN_PROGRESS = 0x0001,

    VP_DEV_INIT_CMP = 0x0002,    /**< Set if device has been initialized */
    VP_DEV_IN_CAL = 0x0004,      /**< Set if device is busy calibrating */
    VP_DEV_PENDING_INT = 0x0008, /**< Set if there is a pending interrupt */

    VP_DEV_ABS_BAT_CAL = 0x0010, /**< Set when running ABS (batt switch) Cal */
    VP_DEV_ABV_CAL = 0x0020,     /**< Set when running ABV Calibration on Tracker */
    VP_DEV_ABV_CAL_ABS = 0x0040, /**< Set when running ABV Calibration on ABS */
    VP_DEV_DISC_PENDING = 0x0080, /**< Set when a Disconnect Timer was suspsended */

    VP_DEV_TEST_BUFFER_READ = 0x0100, /**< Set if test buffer was read this tick */

     /**< Set if device in process of warm reboot. Note that warm reboot
      * detection alone only helps skipping of VpCalCodec() because it is run
      * with VpInitDevice(). But it does not ensure that VpCalLine() was
      * previously run.
      */
    VP_DEV_WARM_REBOOT = 0x0200,

    /* Set while performing VpImplementDefaultSettings().  This can be used to
       avoid entering nested critical sections, or to take other necessary
       precautions due to internally calling the top level VpSetOption(). */
    VP_DEV_IMPL_DEFAULT_OPTIONS = 0x0400,

    VP_TEMP_IGNORE_ALL_BUSY_FLAGS = 0x4000  /* Set to temporarily ignore all other flags */

} VpDeviceBusyFlagsType;

typedef struct VpCSLACDeviceStatusType {
    uint16 state;           /**< This is a bit-mask of VpDeviceBusyFlagsType values */
    uint8 globStatReg;      /**< Holds state of the device stat reg */
    uint8 calibrating;      /**< Calibration timer */
    uint8 numIntServiced;   /**< Max interrupts to be serviced during tick */
} VpCSLACDeviceStatusType;

#define VP_CSLAC_MAX_RCN_PCN_SIZE   2
typedef struct VpDeviceStaticInfoType {
    uint8 rcnPcn[VP_CSLAC_MAX_RCN_PCN_SIZE];    /**< Revision Code Number and
                                                 * Product Code Number as
                                                 * applicable
                                                 */
    uint8 maxChannels;  /**< How many lines supported by this device */
} VpDeviceStaticInfoType;

typedef struct VpDeviceDynamicInfoType {
    uint8 lastChan;     /**< The last line checked */
    bool clkFault;      /** TRUE if a clock fault is active */
    bool bat1Fault;     /** TRUE if a bat1 fault is active and device level */
    bool bat2Fault;     /** TRUE if a bat2 fault is active and device level */
    bool bat3Fault;     /** TRUE if a bat3 fault is active and device level */
} VpDeviceDynamicInfoType;

/** Line state variables used by the line (api internal) */
#ifdef STRICT_SIGNED_ENUM_VALUES
typedef uint16 VpCslacLineCondType;

#define VP_CSLAC_STATUS_INVALID     (0x0000)

#define VP_CSLAC_CLEAR  (0x0000)

/* FXS Type Status */
#define VP_CSLAC_HOOK   (0x0001)
#define VP_CSLAC_GKEY   (0x0002)

/* FXO Type Status */
#define VP_CSLAC_RAW_DISC   (0x0001)
#define VP_CSLAC_RINGING    (0x0004)
#define VP_CSLAC_DISC       (0x0008)

#define VP_CSLAC_AC_FLT     (0x0010)
#define VP_CSLAC_DC_FLT     (0x0020)
#define VP_CSLAC_THERM_FLT  (0x0040)
#define VP_CSLAC_CAL_ENABLE (0x0080)

#define VP_CSLAC_CAL_FAIL           (0x0100)
#define VP_CSLAC_POLREV             (0x0200)
#define VP_CSLAC_POLREV_REPORTED    (0x0400)
#define VP_CSLAC_LIU                (0x0800)

#define VP_CSLAC_LINE_LEAK_TEST (0x1000)
#define VP_CSLAC_RING_AMP_DET   (0x2000)
#define VP_CSLAC_RINGING_EXIT   (0x4000)
#define VP_CSLAC_STATUS_VALID   (0x8000)
#else
typedef enum VpCslacLineCondType {
    VP_CSLAC_STATUS_INVALID = 0x0000,

    VP_CSLAC_CLEAR = 0x0000,

    /* FXS Type Status */
    VP_CSLAC_HOOK = 0x0001,
    VP_CSLAC_GKEY = 0x0002,

    /* FXO Type Status */
    VP_CSLAC_RAW_DISC = 0x0001,     /*
                                     * Set on 890 when device level disconnect
                                     * status is set. Cleared otherwise.
                                     */

    VP_CSLAC_RINGING = 0x0004,      /**< Set when Ringing detected on Line */
    VP_CSLAC_DISC = 0x0008,         /**< Set when Disconnect detected on Line */

    VP_CSLAC_AC_FLT = 0x0010,
    VP_CSLAC_DC_FLT = 0x0020,
    VP_CSLAC_THERM_FLT = 0x0040,
    VP_CSLAC_CAL_ENABLE = 0x0080,   /**< Set when the line is in a state that
                                     * will allow device calibration
                                     */

    VP_CSLAC_CAL_FAIL = 0x0100,
    VP_CSLAC_POLREV = 0x0200,
    VP_CSLAC_POLREV_REPORTED = 0x0400,
    VP_CSLAC_LIU = 0x0800,

    VP_CSLAC_LINE_LEAK_TEST = 0x1000,   /**< Set when line is being tested for
                                         * resistive leak. Clear when test is
                                         * complete.
                                         */
    VP_CSLAC_RING_AMP_DET = 0x2000,
    VP_CSLAC_RINGING_EXIT = 0x4000,     /**< Set when FXS line is exiting Ringing */
    VP_CSLAC_STATUS_VALID = 0x8000
} VpCslacLineCondType;
#endif

typedef enum VpCslacCalType {
    VP_CSLAC_CAL_NONE = 0x0,
    VP_CSLAC_CAL_VOC,
    VP_CSLAC_CAL_ABV
} VpCslacCalType;

typedef struct VpApiIntLineStateType {
    VpLineStateType currentState;   /**< Current state of the line */
    VpLineStateType previous;       /**< Previous sate of the line */
    VpLineStateType usrCurrent;     /**< Current user set state of the line */

    /**< Differences between the usrCurrent state and currentState occur during most API internal
     * line state control operations. E.g., Ringing Cadence and Calibration. When performing line
     * state control functions on behalf of the Application (e.g., VpSendSignal()), these should
     * be the same. For ex: if running a Forrward Disconnect Send Signal operation, for the duration
     * the line is in VP_LINE_DISCONNECT the reported state using VpGetLineState() should be
     * VP_LINE_DISCONNECT - not the state the line was in prior to performing VpSendSignal().
     */

    uint16 condition;   /**< 'OR' Combination of VpCslacLineCondType */
    VpCslacCalType calType;
    
    bool thermFltDebounce;
    uint16 thermFltDebounceTimestamp;
} VpApiIntLineStateType;

    /**< Indexes into the Custome Term Profile Type. #define instead of enums becuase the values
     * must never be negative (many compilers treat enum = int type) and types larger than 8-bit is
     * a waste.
     */
#define VP_CUSTOM_TERM_SLIC_TYPE    (6)
#define VP_CUSTOM_TERM_NUM_STATES   (7)

    /**< Indexes into the FXO/Dialing Profile Type. #define instead of enums becuase the values
     * must never be negative (many compilers treat enum = int type) and types larger than 8-bit is
     * a waste.
     */
#define VP_FXO_DIAL_PRFL_DTMF_ON_MSB    (6)
#define VP_FXO_DIAL_PRFL_DTMF_ON_LSB    (7)
#define VP_FXO_DIAL_PRFL_DTMF_OFF_MSB   (8)
#define VP_FXO_DIAL_PRFL_DTMF_OFF_LSB   (9)
#define VP_FXO_DIAL_PRFL_FLASH_HOOK_MSB (10)
#define VP_FXO_DIAL_PRFL_FLASH_HOOK_LSB (11)
#define VP_FXO_DIAL_PRFL_PULSE_BREAK    (12)
#define VP_FXO_DIAL_PRFL_PULSE_MAKE     (13)
#define VP_FXO_DIAL_PRFL_INTERDIG_MSB   (14)
#define VP_FXO_DIAL_PRFL_INTERDIG_LSB   (15)
#define VP_FXO_DIAL_PRFL_RING_PRD_MAX   (16)
#define VP_FXO_DIAL_PRFL_RING_PRD_MIN   (17)
#define VP_FXO_DIAL_PRFL_RING_VOLT_MIN  (18)
#define VP_FXO_DIAL_PRFL_DISC_VOLT_MIN  (19)
#define VP_FXO_DIAL_PRFL_LIU_THRESH_MIN (20)
#define VP_FXO_DIAL_PRFL_RSVD           (21)
#endif




