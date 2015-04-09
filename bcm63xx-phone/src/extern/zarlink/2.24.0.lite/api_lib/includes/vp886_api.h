/** \file vp886_api.h
 * vp886_api.h
 *
 *  Header file that defines all the commands for the Vp886 series devices.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11610 $
 * $LastChangedDate: 2014-10-20 16:24:12 -0500 (Mon, 20 Oct 2014) $
 */

#ifndef VP886_API_H
#define VP886_API_H

#include "vp_api_cfg.h"
#include "vp_hal.h"
#include "vp_CSLAC_types.h"
#include "vp_api_common.h"
#include "vp_timer_queue.h"
#ifdef VP886_INCLUDE_ADAPTIVE_RINGING
#include "vp_adaptive_ringing.h"
#endif

#define VP886_MAX_NUM_CHANNELS   2

#ifdef VP886_INCLUDE_TESTLINE_CODE
#include "vp_api_test.h"
#include "vp886_testline.h"
#endif

#include "vp886_registers.h"


/* Enforce a minimum configurable event queue size */
#define VP886_EVENT_QUEUE_MIN_SIZE 6
#if VP886_EVENT_QUEUE_SIZE < VP886_EVENT_QUEUE_MIN_SIZE
#error VP886_EVENT_QUEUE_SIZE configured below the minimum
#endif
/* Define a new event structure to use in the queue to reduce size.  VpEventType
 * contains several data members that are not necessary internally. */
typedef struct {
    uint8 channelId;
    VpEventCategoryType eventCategory;
    uint16 eventId;
    uint16 eventData;
    uint16 parmHandle;
    bool hasResults;
} Vp886QueuedEventType;

typedef struct {
    Vp886QueuedEventType events[VP886_EVENT_QUEUE_SIZE];
    uint8 numQueued;
    uint8 pushIndex;
    uint8 popIndex;
    bool overflowed;
} Vp886EventQueueType;


typedef struct {
    VpOptionPulseType pulse;                       /* VP_DEVICE_OPTION_ID_PULSE        */
    VpOptionPulseType pulse2;                      /* VP_DEVICE_OPTION_ID_PULSE2       */
    VpOptionCriticalFltType criticalFlt;           /* VP_DEVICE_OPTION_ID_CRITICAL_FLT */
    VpOptionDeviceIoType deviceIo;                 /* VP_DEVICE_OPTION_ID_DEVICE_IO    */
    VpOptionEventMaskType eventMask;               /* VP_OPTION_ID_EVENT_MASK          */
    VpOptionAdaptiveRingingType adaptiveRinging;   /* VP_DEVICE_OPTION_ID_ADAPTIVE_RINGING */
    VpOptionRingPhaseSyncType ringPhaseSync;       /* VP_DEVICE_OPTION_ID_RING_PHASE_SYNC */
    VpOptionFsyncRateType fsyncRate;               /* VP_DEVICE_OPTION_ID_FSYNC_RATE   */
} Vp886DevOptionsCacheType;

typedef struct {
    VpOptionPulseModeType pulseMode;               /* VP_OPTION_ID_PULSE_MODE          */
    VpOptionTimeslotType timeslot;                 /* VP_OPTION_ID_TIMESLOT            */
    VpOptionCodecType codec;                       /* VP_OPTION_ID_CODEC               */
    VpOptionLoopbackType loopback;                 /* VP_OPTION_ID_LOOPBACK            */
    VpOptionLineStateType lineState;               /* VP_OPTION_ID_LINE_STATE          */
    VpOptionEventMaskType eventMask;               /* VP_OPTION_ID_EVENT_MASK          */
    VpOptionRingControlType ringControl;           /* VP_OPTION_ID_RING_CNTRL          */
    VpOptionPcmTxRxCntrlType pcmTxRxCntrl;         /* VP_OPTION_ID_PCM_TXRX_CNTRL      */
    bool switcherCtrl;                             /* VP_OPTION_ID_SWITCHER_CTRL       */
    VpOptionAbsGainType absGain;                   /* VP_OPTION_ID_ABS_GAIN            */
    VpOptionGndFltProtType gndFltProt;             /* VP_OPTION_ID_GND_FLT_PROTECTION  */
    VpOptionDtmfModeType dtmfMode;                 /* VP_OPTION_ID_DTMF_MODE           */
    uint16 ringTripConfirm;                        /* VP_OPTION_ID_RINGTRIP_CONFIRM    */
    VpOptionHighPassFilterType highPassFilter;     /* VP_OPTION_ID_HIGHPASS_FILTER     */
    VpOptionDtmfParamsType dtmfParams;             /* VP_OPTION_ID_DTMF_PARAMS         */
} Vp886LineOptionsCacheType;


#define VP886_NUM_PROFILE_TYPES 9

/* ABS Supply Configurations: Single, Slave, Master */
typedef enum {
    VP886_ABS_SUPP_CFG_SINGLE = 0,
    VP886_ABS_SUPP_CFG_SLAVE  = 1,
    VP886_ABS_SUPP_CFG_MASTER = 2
} Vp886AbsSuppCfgType;

/* Charge Pump protection modes */
typedef enum {
    VP886_CP_PROT_UV_SHUTDOWN = 0,
    VP886_CP_PROT_CYCLE_SKIP = 1,
    VP886_CP_PROT_DISABLED = 2
} Vp886CPProtectionModeType;

/*
 * Vp886DeviceProfileType -------------------------------- Vp886DeviceProfileType
 */
typedef struct {
    uint8   profVersion;
    uint16  pcmClkRate;      /**< Used to verify valid TX/RX Timeslot setting */
    uint8   devCfg;
    uint8   devMode[2];
    uint8   swyv;
    uint8   swzv;
    uint8   swyLimit;
    uint8   swzLimit;
    uint8   io2Use;
    bool    lowVoltOverride;
    uint8   dialPulseCorrection;
    uint8   swCfg;
    uint8   swOCC[2];
    uint8   blanking;
    Vp886AbsSuppCfgType absSuppCfg;
    uint8   adaptiveRingingMaxPower;
    bool    cpEnable;
    Vp886CPProtectionModeType cpProtection;
    uint8   vsw;
    int8    vbhOffset;
    int8    vbhOverhead;
} Vp886DeviceProfileType;

/* Vp886InitDeviceSM() state variable.  Setting constant values so that we don't
   need to check debug logs against the exact API version being used. */
typedef enum {
    VP886_INIT_DEVICE_SWITCHER_PREP = 2,
    VP886_INIT_DEVICE_CFAIL_CHECK_PREP = 3,
    VP886_INIT_DEVICE_CFAIL_CHECK = 4,
    VP886_INIT_DEVICE_ZSI_DETECT = 19,
    VP886_INIT_DEVICE_ENABLE_VREF = 5,
    VP886_INIT_DEVICE_VREF_CHECK = 6,
    VP886_INIT_DEVICE_CP_ENABLE = 7,
    VP886_INIT_DEVICE_CH1_CH2_DISC = 9,
    VP886_INIT_DEVICE_CP_CHECK = 8,
    VP886_INIT_DEVICE_CH1_CH2_SW_ENABLE = 10,
    VP886_INIT_DEVICE_CAL_CODEC_START = 13,
    VP886_INIT_DEVICE_CAL_CODEC = 14,
    VP886_INIT_DEVICE_SW_ENABLE = 11,
    VP886_INIT_DEVICE_ALARM_CHECK = 12,
    VP886_INIT_DEVICE_INIT_LINES = 15,
    VP886_INIT_DEVICE_LONG_CAL = 16,
    VP886_INIT_DEVICE_GEN_EVENT = 17,
    VP886_INIT_DEVICE_COMPLETE = 18
} Vp886InitDeviceStateType;

/* Vp886CalCodecSM() state variable */
typedef enum {
    VP886_CAL_CODEC_START = 0,
    VP886_CAL_CODEC_PREPARE,
    VP886_CAL_CODEC_SADC,
    VP886_CAL_CODEC_SWY_SENSE,
    VP886_CAL_CODEC_SWZ_SENSE,
    VP886_CAL_CODEC_TIP_SENSE,
    VP886_CAL_CODEC_RING_SENSE,
    VP886_CAL_CODEC_IO2_SENSE,
    VP886_CAL_CODEC_VOC_SENSE,
    VP886_CAL_CODEC_FLOOR,
    VP886_CAL_CODEC_BATTERY,
    VP886_CAL_CODEC_BATTERY_SENSE,
    VP886_CAL_CODEC_BATTERY_SAT,
    VP886_CAL_CODEC_BATTERY_LIMIT,
    VP886_CAL_CODEC_DISCONNECT,
    VP886_CAL_CODEC_LONGITUDINAL,
    VP886_CAL_CODEC_SYNC,
    VP886_CAL_CODEC_CONCLUDE,
    VP886_CAL_CODEC_GEN_EVENT,
    VP886_CAL_CODEC_COMPLETE,
    VP886_CAL_CODEC_FAILED
} Vp886CalCodecStateType;

/* CalCodec state variable */
typedef enum {
    VP886_SADC_INIT = 0,
    VP886_SADC_ICAL_L,
    VP886_SADC_ICAL_H,
    VP886_SENSE_INIT,
    VP886_SENSE_ICAL_L,
    VP886_SENSE_ICAL_H,
    VP886_VOC_S_INIT,
    VP886_VOC_S_START,
    VP886_VOC_S_ICAL_L,
    VP886_VOC_S_ICAL_H,
    VP886_FLOOR_INIT,
    VP886_FLOOR_START,
    VP886_FLOOR_LOW,
    VP886_FLOOR_HIGH,
    VP886_TRACKER_INIT,
    VP886_TRACKER_VAB_START,
    VP886_TRACKER_VAB_LOW,
    VP886_TRACKER_VAB_HIGH,
    VP886_TRACKER_VAS_LOW,
    VP886_TRACKER_VAS_HIGH,
    VP886_TRACKER_COMPUTE,
    VP886_BAT_SENSE_INIT,
    VP886_BAT_SENSE_LOW,
    VP886_BAT_SENSE_HIGH,
    VP886_BAT_SAT_INIT,
    VP886_BAT_SAT_MEAS,
    VP886_SW_LIM_INIT,
    VP886_SW_LIM_LOW,
    VP886_SW_LIM_HIGH,
    VP886_LONG_INIT,
    VP886_LONG_GET_MEAS,
    VP886_LONG_GET_MEAS_RINGING
} Vp886CalCodecSubStateType;

/* CalLine state variable */
typedef enum {
    VP886_SADC_VMODE_INIT = 0,
    VP886_SADC_VMODE_VOFF,
    VP886_SADC_VMODE_VCAL,
    VP886_VADC_INIT,
    VP886_VADC_ICAL_L,
    VP886_VADC_ICAL_H,
    VP886_VAB_INIT,
    VP886_VAB_START,
    VP886_VAB_ICAL_L,
    VP886_VAB_ICAL_H,
    VP886_IMT_INIT,
    VP886_IMT_START,
    VP886_IMT_LOW,
    VP886_IMT_HIGH,
    VP886_VOC_INIT,
    VP886_VOC_START,
    VP886_VOC_LOW,
    VP886_VOC_HIGH,
    VP886_VOC_BUFFER,
    VP886_RINGING_INIT,
    VP886_RINGING_START,
    VP886_RINGING_LOW,
    VP886_RINGING_HIGH,
    VP886_RINGING_BUFFER,
    VP886_HOOK_DET_INIT,
    VP886_HOOK_DET_METALIC,
    VP886_HOOK_DET_COMPARATOR,
    VP886_HOOK_DET_COMPUTE,
    VP886_GND_KEY_INIT,
    VP886_GND_KEY_LONG,
    VP886_GND_KEY_COMPARATOR,
    VP886_GND_KEY_COMPUTE,
    VP886_ABS_INIT,
    VP886_ABS_SWY_START,
    VP886_ABS_CHECK_ICAL,
    VP886_ABS_SWY,
    VP886_ABS_BSW_START,
    VP886_ABS_BSW_LOW,
    VP886_ABS_BSW_HIGH
} Vp886CalLineSubStateType;

/* States for sense recalibration (Vp886QuickCalHandler()) */
typedef enum {
    VP886_QUICKCAL_INACTIVE = 0,
    VP886_QUICKCAL_INIT,
    VP886_QUICKCAL_SETUP_VA_VB_VAB,
    VP886_QUICKCAL_READ_VA_VB_VAB,
    VP886_QUICKCAL_SETUP_VABREV,
    VP886_QUICKCAL_READ_VABREV,
    VP886_QUICKCAL_SETUP_VABRING,
    VP886_QUICKCAL_READ_VABRING,
    VP886_QUICKCAL_FINISHED
} Vp886QuickCalStateType;

typedef struct Vp886QuickCalDataType {
    Vp886QuickCalStateType state;
    uint16 timerId;
    uint32 timerHandle;
    uint16 sadcFlag;
    bool externalCall;
} Vp886QuickCalDataType;

/* Calibration registers scratch-pad */
typedef struct {
    uint8 dcFeed[VP886_R_DCFEED_LEN];
    uint8 sysConfig[VP886_R_SSCFG_LEN];
    uint8 ringGen[VP886_R_RINGGEN_LEN];
    uint8 icr1[VP886_R_ICR1_LEN];
    uint8 icr2[VP886_R_ICR2_LEN];
    uint8 icr3[VP886_R_ICR3_LEN];
    uint8 icr4[VP886_R_ICR4_LEN];
    uint8 icr5[VP886_R_ICR5_LEN];
    uint8 icr6[VP886_R_ICR6_LEN];
    uint8 srp[VP886_R_SWPARAM_LEN];
    uint8 devMode[VP886_R_DEVMODE_LEN];
    uint8 normCal[VP886_R_NORMCAL_LEN];
    uint8 revCal[VP886_R_REVCAL_LEN];
    uint8 indCal[VP886_R_INDCAL_LEN];
    uint8 opCond[VP886_R_OPCOND_LEN];
    uint8 opFunc[VP886_R_OPFUNC_LEN];
    uint8 loopSup[VP886_R_LOOPSUP_LEN];
    uint8 slacState[VP886_R_STATE_LEN];
    uint8 ioDir[VP886_R_IODIR_LEN];
} Vp886RegPadType;

/* Vp886CalLinecSM() state variable */
typedef enum {
    VP886_CAL_LINE_START,
    VP886_CAL_LINE_VADC_ACTIVE_CAL,
    VP886_CAL_LINE_SADC_VMODE_CAL,
    VP886_CAL_LINE_VAB_SENSE_CAL,
    VP886_CAL_LINE_IMT_CAL,
    VP886_CAL_LINE_VOC_CAL,
    VP886_CAL_LINE_RINGING_CAL,
    VP886_CAL_LINE_HOOK_DETECTOR_CAL,
    VP886_CAL_LINE_GROUND_KEY_CAL,
    VP886_CAL_LINE_ABS_CAL,
    VP886_CAL_LINE_RESTORE,
    VP886_CAL_LINE_GEN_EVENT,
    VP886_CAL_LINE_COMPLETE,
    VP886_CAL_LINE_FAILED
} Vp886CalLineStateType;

/* IMPORTANT: Make sure to update the "stateInt" member of the device object if
   the size of this type changes. There is no instance of this type itself. */
typedef enum {
    VP886_SYS_CAL_COMPLETE = 0x0001,        /**< Set when the system calibration structure has been initialied */
    /* This value is set when either Device Calibration (VpInitDevice()/VpCalCodec()
       completes successfully, or when loaded values by Apply System Coeff. After
       VpInitDevice() this should only be NOT SET if device calibration failed or
       if null was passed to Apply System Coeff. */
    VP886_DEVICE_CAL_COMPLETE = 0x0002,

    VP886_DEVICE_ICAL_L_IN_USE  = 0x0004,  /* Low calibration current in use */
    VP886_DEVICE_ICAL_H_IN_USE  = 0x0008,  /* High calibration current in use */

    VP886_FORCE_FREE_RUN = 0x0010,         /**< Set when app calls VpFreeRun() (start), cleared when called with stop.
                                             * This prevents the VP-API-II from automatically exiting free run mode
                                             * upon PCLK recovery.
                                             */
    VP886_ZSI_DETECTED = 0x0020,
    VP886_SHARED_SUPPLY = 0x0040,
    VP886_CONVERT_DEVICE_TYPE = 0x0080
} Vp886DeviceStateIntType;

/* Union of basic results types for Vp886.  Does not include linetest,
 * low level cmd, or result types not supported by Vp886. */
typedef union {
    VpLoopCondResultsType getLoopCond;         /* VP_LINE_EVID_RD_LOOP        */
    VpDeviceIoAccessDataType deviceIoAccess;   /* VP_DEV_EVID_IO_ACCESS_CMP   */
    VpLineIoAccessType lineIoAccess;           /* VP_LINE_EVID_LINE_IO_RD_CMP */
} Vp886BasicResultsType;

#define VP886_MAX_BRANCH_DEPTH 5
typedef struct Vp886SeqDataType {
    VpProfilePtrType pActiveCadence;    /* Currently Active Cadence */
    VpProfilePtrType pCurrentPos;       /* Current position in profile */

    VpCadenceStatusType status;

    uint8 index;    /* Index in the the cadence table   */
    uint8 length;   /* Length of executable cadence instructions */

    uint8 branchCount[VP886_MAX_BRANCH_DEPTH];
    uint8 branchIdx[VP886_MAX_BRANCH_DEPTH];
    uint8 branchDepth;

    uint8 toneType;

    VpLineStateType lineState;  /* Current line state set by the cadence */

    /* Howler data */
    uint8 regData[VP886_R_SIGAB_LEN];
    uint16 startLevel;
    uint16 stopLevel;
    uint16 levelStep;
} Vp886SeqDataType;

/* Contains the information needed to process SendSignal sequences */
typedef struct {
    VpSendSignalType type;
    bool active;
    struct {
        uint8 voltage;
        uint16 onTime;
        uint16 offTime;
        uint8 cycles;
        bool on;
    } msgWait;
} Vp886SendSignalDataType;

/* Contains the information needed to process metering sequences */
typedef struct {
    bool active;
    bool on;
    uint16 onTime;  /* 10ms units */
    uint16 offTime; /* 10ms units */
    uint16 remaining;
    uint16 completed;
    bool abort;
} Vp886MeteringDataType;

typedef enum {
    VP886_CID_ST_NEW_INSTR = 0,
    VP886_CID_ST_ALERT_TONE = 100,
    VP886_CID_ST_SILENT_INTERVAL = 200,
    VP886_CID_ST_SILENT_HOOK_MASK_INTERVAL = 300,
    VP886_CID_ST_DETECT_INTERVAL = 400,
    VP886_CID_ST_MARK_OR_SEIZURE = 500,
    VP886_CID_ST_MARK_OR_SEIZURE_ENDING = 501,
    VP886_CID_ST_MESSAGE_FSK = 600,
    VP886_CID_ST_MESSAGE_FSK_MARKOUT = 601,
    VP886_CID_ST_MESSAGE_FSK_ENDING = 602,
    VP886_CID_ST_MESSAGE_DTMF = 700
} Vp886CidStateType;

#define VP886_CID_BUFFER_SIZE 32
typedef struct {
    bool active;
    VpProfilePtrType pProf;
    uint8 profIdx;
    Vp886CidStateType state;
    uint8 msgBuf[VP886_CID_BUFFER_SIZE];   /* Circular message data buffer */
    uint8 msgLen;       /* Number of bytes currently valid in buffer */
    uint8 msgIdx;       /* Index of oldest valid byte in buffer */
    uint8 checksum;
    uint16 markOrSeizureBytes;
    VpDigitType detectDigit1;
    VpDigitType detectDigit2;
    bool digitDetected;
    bool fskEnabled;
    bool dtmfEnabled;
    bool mute;
    bool needData;
} Vp886CidDataType;

typedef struct {
    int16 gain;
    int16 offset;
} Vp886CalGainOffsetType;

/* Common calibration structure */
typedef struct {
    /* Senses calibration -> accurate ADC readings */
    Vp886CalGainOffsetType sadc;
    Vp886CalGainOffsetType sadcVMode;
    Vp886CalGainOffsetType vadcActive;
    Vp886CalGainOffsetType swySense;
    Vp886CalGainOffsetType swzSense;
    Vp886CalGainOffsetType tipSense;
    Vp886CalGainOffsetType ringSense;
    Vp886CalGainOffsetType io2Sense;
    Vp886CalGainOffsetType vabSenseNormal;
    Vp886CalGainOffsetType vabSenseReverse;
    Vp886CalGainOffsetType vabSenseRinging;

    /* Features calibration -> the device generates accurate signals */
    /* ILA */
    Vp886CalGainOffsetType ilaNormal;
    Vp886CalGainOffsetType ilaReverse;
    /* VOC */
    Vp886CalGainOffsetType vocNormal;
    Vp886CalGainOffsetType vocReverse;
    Vp886CalGainOffsetType vocBuffer;
    /* Ringing */
    Vp886CalGainOffsetType ringingGenerator;
    Vp886CalGainOffsetType ringingBuffer;
    /* VOC, Ringing shared */
    Vp886CalGainOffsetType vocSenseNormal;
    /* Fixed Battery voltage */
    Vp886CalGainOffsetType fixedBat;
    Vp886CalGainOffsetType batterySense;
    /* Longitudinal point */
    Vp886CalGainOffsetType longActive;
    Vp886CalGainOffsetType longRinging;
    Vp886CalGainOffsetType reserved_1;
    Vp886CalGainOffsetType reserved_2;
    /* Hook Detector (1 offset, 4 gains) */
    Vp886CalGainOffsetType hookLPM;
    Vp886CalGainOffsetType hookReverse;
    Vp886CalGainOffsetType hookNormal;
    /* Battery saturation (offset only) */
    Vp886CalGainOffsetType batSat;
    /* Switcher limit */
    Vp886CalGainOffsetType swLimit50V;
    Vp886CalGainOffsetType swLimit100V;
    /* Ground Key detector */
    Vp886CalGainOffsetType gndKeyLong;
    /* Capacitance test tip/ring offsets (pF) */
    int32 tipCapCal;
    int32 ringCapCal;
} Vp886CmnCalDeviceDataType;

/* Tracker calibration structure */
typedef struct {
    /* Tracking battery */
    Vp886CalGainOffsetType trackerVabNormal;
    Vp886CalGainOffsetType trackerVabReverse;
    Vp886CalGainOffsetType trackerVasNormal;
    Vp886CalGainOffsetType trackerVasReverse;
    Vp886CalGainOffsetType reserved_5;
    Vp886CalGainOffsetType reserved_6;
    Vp886CalGainOffsetType trackerBatSense;
} Vp886TrkCalDeviceDataType;

/* ABS calibration structure */
typedef struct {
    Vp886CalGainOffsetType absVabNormal;
    Vp886CalGainOffsetType absVabReverse;
    Vp886CalGainOffsetType reserved_7;
    Vp886CalGainOffsetType reserved_8;
    Vp886CalGainOffsetType reserved_5;
    Vp886CalGainOffsetType reserved_6;
    Vp886CalGainOffsetType absBatSense;
} Vp886AbsCalDeviceDataType;

typedef struct {
    bool valid;
    Vp886CmnCalDeviceDataType cmn;
    union {
        Vp886TrkCalDeviceDataType trk;
        Vp886AbsCalDeviceDataType abs;
    } spe;
} Vp886CalDeviceDataType;

typedef enum {
    VP886_HOWLER_ST_ALIGNMENT = 0,
    VP886_HOWLER_ST_AMP_STEPPING_UK_AUS = 100,
    VP886_HOWLER_ST_AMP_STEPPING_NTT = 200
} Vp886HowlerStateType;

typedef enum {
    VP886_GNDFLTPROT_ST_INACTIVE = 0,
    VP886_GNDFLTPROT_ST_GKEY_DETECTED = 1,
    VP886_GNDFLTPROT_ST_DISCONNECT = 2,
    VP886_GNDFLTPROT_ST_TIP_OPEN = 3,
    VP886_GNDFLTPROT_ST_RING_OPEN = 4
} Vp886GroundFaultProtStateType;

typedef enum {
    VP886_GNDFLTPROT_INP_TIMER = 0,
    VP886_GNDFLTPROT_INP_GKEY_DET = 1,
    VP886_GNDFLTPROT_INP_GKEY_REL = 2,
    VP886_GNDFLTPROT_INP_LINESTATE = 3,
    VP886_GNDFLTPROT_INP_STOP = 4
} Vp886GroundFaultProtInputType;

typedef struct {
    Vp886GroundFaultProtStateType state;
    bool faultDeclared;
    bool settingDisconnect;
    uint16 iterations;
} Vp886GroundFaultProtDataType;

typedef enum {
    VP886_ABS_POWER_REQ_LOW = 0,
    VP886_ABS_POWER_REQ_MED = 1,
    VP886_ABS_POWER_REQ_HIGH = 2,
    VP886_ABS_POWER_REQ_OFF = 3
} Vp886AbsPowerReqType;

typedef struct {
    bool sampling;
    bool overflow;
    VpDigitType currentDigit;
    VpDtmfDetectDataType detectData;
    uint8 sadcCtrl[VP886_R_SADC_LEN];
    bool useVadc;
} Vp886DtmfType;

typedef enum {
    VP886_RINGSYNC_STATE_IDLE = 0,
    VP886_RINGSYNC_STATE_PAUSING = 1,   /* Line which is pausing its ringing */
    VP886_RINGSYNC_STATE_DELAYING = 2,  /* Line which is delaying its ringing */
    VP886_RINGSYNC_STATE_FINISHING_PAUSE = 3,
    VP886_RINGSYNC_STATE_FINISHING_DELAY = 4
} Vp886RingSyncStateType;

typedef enum {
    VP886_IO_CAPABILITY_TWO_PER_CH = 0, /* 2 IOs per channel */
    VP886_IO_CAPABILITY_NONE = 1,       /* No IOs */
    VP886_IO_CAPABILITY_CH0_IO2 = 2     /* IO2 on channel 0 only */
} Vp886IoCapabilityType;

/*
 * Vp886LineObjectType -------------------------------------------------------- Vp886LineObjectType
 */
typedef struct {
    uint8 channelId;
    VpLineIdType lineId;
    VpTermType termType;
    uint8 ecVal;

    bool isFxs;

    /* These flags are used by Vp886ReadyStatus() to decide whether a line can be
       accessed (i.e. is not "busy").  This is a bitmask of
       Vp886LineBusyFlagsType values. */
    uint16 busyFlags;

    VpApiIntLineStateType lineState;

    /* Cache for all line option settings */
    Vp886LineOptionsCacheType options;

    /* Set of VP_CSLAC_____ values defining which line conditions we are
       currently ignoring (such as VP_CSLAC_HOOK | VP_CSLAC_GKEY) */
    uint16 detectMasks;

    /* State variable for the Vp886CalLine state machine. This is used
       inside the Vp886CalLineSM function to keep track of the state
       and specify which state (case) of the switch statement to enter. */
    Vp886CalLineStateType calLineState;
    Vp886CalLineSubStateType calLineSubState;

    /* Info for the quick recalibration routine. */
    Vp886QuickCalDataType quickCal;

    /* Remember if the calibration registers have been programmed. They need to
       be cleared before re-running VpCalLine(). */
    bool calRegsProgrammed;

    /* Vp886GetLoopCond() backups */
    uint16 getLoopCondHandle;
    uint8 vadcConvConf[VP886_R_VADC_LEN];
    uint8 getLoopIcr2[VP886_R_ICR2_LEN];
    uint8 getLoopIcr3[VP886_R_ICR3_LEN];
    uint8 getLoopIcr4[VP886_R_ICR4_LEN];

    /* Pulse decoding state data */
    VpPulseDecodeDataType pulseDecodeData;

    /* Line Topology Info */
    VpLineTopologyType lineTopology;

#ifdef VP_CSLAC_SEQ_EN
    /* Sequencer state information */
    Vp886SeqDataType cadence;
    Vp886HowlerStateType howlerState;
    uint16 howlerLevel;

    /* Ringing cadence set by VpInitRing() */
    VpProfilePtrType pRingingCadence;

    /* Caller ID state information */
    Vp886CidDataType cid;

    /* SendSignal state information */
    Vp886SendSignalDataType sendSignal;

    /* Metering state information */
    Vp886MeteringDataType metering;
#endif /* VP_CSLAC_SEQ_EN */

    /* Keeps track of whether the internal test termination is applied */
    bool intTestTermApplied;

    bool ringExitInProgress;

    /* State to end up in after the ring exit polrev workaround */
    VpLineStateType ringExitCleanupState;

    Vp886RingSyncStateType ringSyncState;

    /* Stores the ringing state that was requested but delayed for the channel
       that wants to start ringing. */
    VpLineStateType ringSyncLineState;

    /* Used for VP_OPTION_ID_RINGTRIP_CONFIRM */
    struct {
        bool confirming;
        bool glitchCheck;
        uint16 lastHookTimestamp;
    } ringTripConfirm;

#if (VP886_USER_TIMERS > 0)
    /* Number of currently running user timers.  Used to implement a limit
       of VP886_USER_TIMERS */
    int16 userTimers;
#endif

#ifdef VP886_INCLUDE_TESTLINE_CODE
    VpRelayControlType relayState;
#endif

    /* Ringing parameters from the ringing profile, in register format. */
    int16 ringBias;
    int16 ringAmplitude;
    int16 ringAmplitudeCal;
    int16 ringFrequency;
    bool ringSine;
    int16 fixedRingBat;

    /* Ringing mode: balanced (FALSE) or unbalanced (TRUE) */
    bool unbalancedRinging;

    /* Ring trip detection algorithm.  Saved from the ringing profile
       so that we can restore it after a msg waiting pulse signal or
       any other potential operation that changes it on the fly */
    uint8 ringTripAlg;

    /* Number of cycles for ring trip (1 or 2), saved so that we can subtract
       it from the ring trip confirm timer */
    uint8 ringTripCycles;

    /* Bit mask of the tones programmed by the most recent tone profile.
       This is used to restrict which tones are enabled in VpSetLineTone() and
       by cadence commands. */
    uint8 toneGens;

    /* Set based on the DC profile, this will determine whether
       groundkey interrupts are interpreted as groundkey events (for
       ground start applications) or DC fault events (loop start). */
    bool reportDcFaults;

    Vp886GroundFaultProtDataType gndFltProt;

    /* Floor voltage in 1V steps */
    uint8 floorVoltage;

    /* Target VOC in 1V steps */
    uint8 targetVoc;

    uint16 gxBase;      /**< Cached GX register, in 2.14 int format */
    uint16 gxUserLevel; /**< User specified relative GX level (2.14) */
    uint16 grBase;      /**< Cached GR register, in 2.14 int format */
    uint16 grUserLevel; /**< User specified relative GR level (2.14) */

    /* Most recent non-QUIET settings for the ABS_GAIN option to support
       VP_OPTION_ABS_GAIN_RESTORE. */
    int16 absGxRestoreOption;
    int16 absGrRestoreOption;
    uint8 absGxRestoreReg[VP886_R_GX_LEN];
    uint8 absGrRestoreReg[VP886_R_GR_LEN];

    /* Event results for VpSetRelGain() */
    VpRelGainResultsType setRelGainResults;

    /* Keep track of IO2 status when it's configured as an interrupt */
    bool io2State;

    /* Set to TRUE if we're using the ila_fdrng bit of ICR2 to enable
       ringing current limits below 50mA.  This changes the way we have to
       read the ringing current limit value for VP_OPTION_ID_RINGING_PARAMS.
       It also changes how the VADC reads current, so ringing power management
       and line test need to be aware of it. */
    bool lowIlr;

    /* Saved base value of RTTH.  The low ILR workaround will double
       the base value, but will be capped at the max register value, so we
       need to remember the original to be able to restore it. */
    uint8 rtth;

#ifdef VP886_INCLUDE_DTMF_DETECT
    /* Flags and data for DTMF detection */
    Vp886DtmfType dtmf;
#endif /* VP886_INCLUDE_DTMF_DETECT */

#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
    bool inHighGainMode;
    struct {
        uint8 icr1[VP886_R_ICR1_LEN];
        uint8 icr2[VP886_R_ICR2_LEN];
        uint8 icr3[VP886_R_ICR3_LEN];
        uint8 icr4[VP886_R_ICR4_LEN];
        uint8 dcFeed[VP886_R_DCFEED_LEN];
        uint8 vpGain[VP886_R_VPGAIN_LEN];
        uint8 grValue[VP886_R_GR_LEN];
        uint8 rValue[VP886_R_R_FILT_LEN];
        uint8 disn[VP886_R_DISN_LEN];
        uint8 opFunc[VP886_R_OPFUNC_LEN];
        uint8 swParam[VP886_R_SWPARAM_LEN];
    } highGainCache;
#endif /* VP_HIGH_GAIN_MODE_SUPPORTED */

    /* Cache for line-specific registers */
    struct {
        uint8 sysState[VP886_R_STATE_LEN];
        uint8 loopSup[VP886_R_LOOPSUP_LEN];
        uint8 opCond[VP886_R_OPCOND_LEN];
        uint8 opFunc[VP886_R_OPFUNC_LEN];
        uint8 swParam[VP886_R_SWPARAM_LEN];
        uint8 calCtrl[VP886_R_CALCTRL_LEN];
        uint8 icr1[VP886_R_ICR1_LEN];
        uint8 icr2[VP886_R_ICR2_LEN];
        uint8 icr3[VP886_R_ICR3_LEN];
        uint8 icr4[VP886_R_ICR4_LEN];
        uint8 dcFeed[VP886_R_DCFEED_LEN];
        uint8 ssCfg[VP886_R_SSCFG_LEN];
        uint8 indCal[VP886_R_INDCAL_LEN];
        uint8 normCal[VP886_R_NORMCAL_LEN];
        uint8 revCal[VP886_R_REVCAL_LEN];
        uint8 ringCal[VP886_R_RINGCAL_LEN];
        uint8 batCal[VP886_R_BATCAL_LEN];
        uint8 ringDelay[VP886_R_RINGDELAY_LEN];
    } registers;


#ifdef VP886_INCLUDE_TESTLINE_CODE
    /* Testing structure */
    Vp886TestInfoType testInfo;

    /* Test results structure */
    VpTestResultType testResults;
#endif

    /* Setting this flag to TRUE will disable all automatic line state changes
       performed by the API for this line. */
    bool inLineTest;

#ifdef VP886_INCLUDE_ADAPTIVE_RINGING

    /* Data structure for ringing thermal management algorithms */
    RingPowerAdaptModuleType ringPowerAdapt;

    /* Running Thermal Adaptive Ringing mode: */
    bool thermalRinging;

    /* Ringing cadence started */
    bool startRingingCadence;

    /* To detect customer cadenced ringing "standby => ringing -> oht -> ringing..." */
    bool wasStandby;
#endif

#ifdef VP_DEBUG
    /* For runtime enabling of debug output: */
    uint32 debugSelectMask;
#endif /* VP_DEBUG */
} Vp886LineObjectType;


typedef enum {
    VP_NORMAL,
    VP_POLREV,
    VP_RINGING
} VpPolarityType;
/*
 * Vp886DeviceObjectType ------------------------------------------------------ Vp886DeviceObjectType
 */
typedef struct {
    /* Device identifier set by the application -- passed to the HAL */
    VpDeviceIdType deviceId;

    /* Silicon RCN/PCN and number of channels supported */
    VpDeviceStaticInfoType staticInfo;

    /* IO capabilities of this device */
    Vp886IoCapabilityType ioCapability;

    /* Weak attempt to put everything that changes during run-time into a
       single structure. Good idea in principle, but in practice the devices
       are too dis-similar to share many common structures.
       This holds the "last channel checked" event information, clock fault,
       and battery fault. */
    VpDeviceDynamicInfoType dynamicInfo;

    /* Variety of common device level status such as init/not init, init in
       progress, test buffer read y/n?, etc.. Internally used by the VP-API-II
       to communicate device state.
       This is a bit-mask of VpDeviceBusyFlagsType values */
    uint16 busyFlags;

    /* Indicates whether an interrupt has been reported by the system service
       layer.  Used for backward compatibility with VpApiTick/VpVirtualISR */
    bool pendingInterrupt;

    /* Similar to common device level status, this is for 886 specific type of
       device level information.
       This is a bit-mask of Vp886DeviceStateIntType values */
    uint16 stateInt;

    /* EC value to be used when a device context is passed to Vp886SlacRegWrite,
       Vp886SlacRegRead, or Vp886SlacBufSend.  During normal operation, this
       should be set to VP886_EC_GLOBAL.  The exception is when a routine
       within VpInitDevice() needs to write or read a channel register.  Since
       there is no requirement to have line contexts/objects defined at the time
       of InitDevice, we can't pass a line context to our register write/read
       functions.  To access channel registers, set this value to VP886_EC_1 or
       VP886_EC_2, do the command using the device context, and then set this
       back to VP886_EC_GLOBAL. */
    uint8 ecVal;

    /* Flag indicates that a function which generates results has been called,
       but VpGetResults() has not been called.  The results are not necessarily
       ready, but no more result-generating functions can run.  This does not
       apply to linetest results, which are handled per-line. */
    bool getResultsRequired;

    /* Union storage for a single basic (non-linetest) result */
    Vp886BasicResultsType basicResults;

    /* Device event queue */
    Vp886EventQueueType eventQueue;

    /* We cache the device timestamp so that we do not end up reading it
       multiple times in quick succession.  We use the flag
       timestampValid so that Vp886GetTimestamp can decide whether to read
       from the device or the cached value.  This flag is set FALSE at the
       beginning/end of each critical section, and TRUE when Vp886GetTimestamp
       reads the register. */
    uint16 timestamp;
    bool timestampValid;

    /* Count of timestamp rollovers that can be used to support long-duration
       timers.  These two variables are used together to keep the timestamp
       and rollover count in sync.  See Vp886GetTimestamp() for details */
    uint16 rolloverCount;
    int32 rolloverBuffer;

    /* When pushing an event into the event, we may need to force an interrupt
       to prompt the application to call VpGetEvent.  If we are already in
       a VpGetEvent call when pushing the event, there is no need to force the
       interrupt.  This flag is set at the beginning of Vp886GetEvent, and
       cleared at the end. */
    bool inGetEvent;

    /* Indicates whether we still have a signaling register read remaining to
       be processed in VpGetEvent().  This is useful for determining if we need
       to force an interrupt to update status information when a detector mask
       ends */
    bool sigregReadPending;

    /* Indicates that a SPI error has been detected, and that no actions should
       be performed until the device is reinitialized. */
    bool spiError;

    /* Keep track of critical section depth.  This is currently only used to
       generate warnings if it goes above 1 (indicating nested critical
       sections) or below 0. */
    uint8 criticalDepth;

#define VP886_TIMER_QUEUE_API_SIZE 8
#if defined(VP886_USER_TIMERS)
    #define VP886_TIMER_QUEUE_SIZE (VP886_TIMER_QUEUE_API_SIZE + VP886_USER_TIMERS)
#else
    #define VP886_TIMER_QUEUE_SIZE (VP886_TIMER_QUEUE_API_SIZE)
#endif
    /* Software timer queue, see vp_timer_queue.c/h */
    VpTimerQueueInfoType timerQueueInfo;
    VpTimerQueueNodeType timerQueueNodes[VP886_TIMER_QUEUE_SIZE];

#if (VP886_USER_TIMERS > 0)
    /* Number of currently running user timers.  Used to implement a limit
       of VP886_USER_TIMERS */
    int16 userTimers;
#endif

    /* Cache for all device option settings */
    Vp886DevOptionsCacheType options;

    /* Soft values from the device profile that are used throughout the VP-API-II
       (not only in Init). Note that other values may be provided by the device
       profile, but are stored in other ways generally because they also may NOT
       be provided in the device profile (i.e., API has default values). */
    Vp886DeviceProfileType devProfileData;

    /* This is the profile pointer table per the VP-API-II User's Guide. */
    VpCSLACDeviceProfileTableType devProfileTable;

    /* This is the structure indicating which profile table entries have been
       initialized. If profile tables are disabled, this and the device profile
       table (devProfileTable) can be removed. */
    uint16 profTableEntryValid[VP886_NUM_PROFILE_TYPES];

    /* Structure to support buffered writes */
    VpSlacBufDataType slacBufData;

    /* Flag to temporarily disable flushing the write buffer before performing
       a read. By default, Vp886SlacRegRead() will send the contents of the
       write buffer before performing the read to prevent issues with assumed
       write->read operations. This flag can be set when it is known that the
       read does not depend on any actions in the write buffer. The flag is set
       back to FALSE on each read. */
    bool dontFlushSlacBufOnRead;

    /* Number of bytes sent/received to/from the device since the last
       VP_QUERY_ID_DEV_TRAFFIC query */
    uint32 trafficBytes;

    /* These switcher paramaters are from the user setting in the
       device profile. They should never be changed except in Init (in case
       the values provided are illegal). */
    uint8 swParams[VP886_R_SWPARAM_LEN];

    /* Switching Regulator Timings for normal and Free Run mode */
    uint8 swTimingParams[VP886_R_SWTIMING_LEN];
    uint8 swTimingParamsFR[VP886_R_SWTIMING_LEN];

    /* The device profile contains switcher timing parameters for low power
       mode (the first two bytes) that should only be applied if all lines are
       low power termination type.  The flag useLowPowerTiming begins TRUE,
       but is set to FALSE if any non-lowpower line is initialized.  As each
       line is initialized, the switcher timing params are set based on this. */
    bool useLowPowerTiming;
    uint8 swTimingParamsLP[2];

    /* State variable for the Vp886InitDevice state machine. This is used
       inside the Vp886InitDeviceSM function to keep track of the state
       and specify which state (case) of the switch statement to enter. */
    Vp886InitDeviceStateType initDeviceState;

    /* Current ABS power mode requirement for each channel.  This is needed for
       some cases where an operation on one line needs to know the current state
       of the other line.  For example, if one line is exiting ringing, we need
       to know the state of the other line to know whether we can lower the
       switcher power mode. */
    Vp886AbsPowerReqType absPowerReq[VP886_MAX_NUM_CHANNELS];

    /* When TRUE for a channel, indicates that the channel requires ringing
       battery levels.  This will be set by Vp886SetABSRingingBattFlag() each
       time the line state changes. */
    bool absRingingBattRequired[VP886_MAX_NUM_CHANNELS];

    /* Peak ringing voltage (amplitude+bias) for each channel that must be
       supported by the battery voltage */
    uint8 absRingingPeak[VP886_MAX_NUM_CHANNELS];

    /* Used by Adaptive Ringing to see if the other line is power limited */
    bool isPowerLimited[2];

    /* Current switcher voltage settings, which can change on the fly to support
       different ringing voltages for ABS */
    uint8 swyv;
    uint8 swzv;

    /* Indicates which channels are using the sampling timer (ch2 timer) so that
       a line-specific function can know whether it is in use by the other line.
       The value is the required timer setting in 0.5ms steps. */
    uint8 samplingTimerReq[VP886_MAX_NUM_CHANNELS];

    /* The current sampling timer period */
    uint8 samplingTimerCurrent;

    /* Current battery fault status, bitmask of VpBatFltEventDataType values */
    uint16 battFltStatus;

    /* Interval in ms that we are polling to check for battery fault changes */
    uint16 battFltPollTime;

    /* Timestamps of the most recent overcurrent interrupts, per channel.
       We need to see two interrupts within a defined period to declare an
       actual overcurrent event. */
    uint32 overCurrentTimestamp[VP886_MAX_NUM_CHANNELS];

    /* State variable for the Vp886CalCodec state machine. This is used
       inside the Vp886CalCodecSM function to keep track of the state
       and specify which state (case) of the switch statement to enter. */
    Vp886CalCodecStateType calCodecState[VP886_MAX_NUM_CHANNELS];

    /* ChannelId of the line under test, 0->line1, 1->line2 */
    bool channelCalBack[VP886_MAX_NUM_CHANNELS];
    bool channelLocked[VP886_MAX_NUM_CHANNELS];

    /* Calibration data */
    int16 icalH;
    int16 icalL;

    VpPolarityType polarity[VP886_MAX_NUM_CHANNELS];

    /* Calibration registers scratch-pad */
    Vp886RegPadType regPad[VP886_MAX_NUM_CHANNELS];

    /* Calibration profile structure */
    Vp886CalDeviceDataType calData[VP886_MAX_NUM_CHANNELS];

    /* CalCodec sub state machine variable */
    Vp886CalCodecSubStateType calCodecSubState[VP886_MAX_NUM_CHANNELS];

    /* Temporary SADC measurements */
    /* Should be channel dependent for concurrent calibration (tmpResultX[VP886_MAX_NUM_CHANNELS]) */
    int16 sadcSetting[VP886_MAX_NUM_CHANNELS];
    int16 sadcSettingGp[VP886_MAX_NUM_CHANNELS][5];
    int16 tmpResultA[VP886_MAX_NUM_CHANNELS];
    int16 tmpResultB[VP886_MAX_NUM_CHANNELS];
    int16 tmpResultC[VP886_MAX_NUM_CHANNELS];
    int16 tmpResultD[VP886_MAX_NUM_CHANNELS];
    int16 tmpResultE[VP886_MAX_NUM_CHANNELS];

    /* Save pointers to the Device, AC, DC, and Ringing profiles.
       These will be used for Vp886InitDevice() only and will only
       be guaranteed valid until the end of the first state of the
       Vp886InitDeviceSM() state machine. They will be used to send down
       the profiles via Vp886ConfigLineInt() before the device bringup
       sequence. */
    VpProfilePtrType pDevProfile;
    VpProfilePtrType pAcProfile;
    VpProfilePtrType pDcProfile;
    VpProfilePtrType pRingProfile;

    /* Switcher mode during InitDevice. Transient value. */
    uint8 initDevSwitcherMode;

    /* Cache for global registers */
    struct {
        uint8 sigreg[VP886_R_SIGREG_LEN];
        uint8 swCtrl[VP886_R_SWCTRL_LEN];
        uint8 intMask[VP886_R_INTMASK_LEN];
        uint8 devCfg[VP886_R_DEVCFG_LEN];
    } registers;

#ifdef VP_DEBUG
    /* For runtime enabling of debug output: */
    uint32 debugSelectMask;
#endif /* VP_DEBUG */
    bool timerOverride;
} Vp886DeviceObjectType;

#endif  /* VP886_API_H */
