/** \file vp880_api.h
 * vp880_api.h
 *
 *  Header file that define all the commands for the Vp880 device.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11037 $
 * $LastChangedDate: 2013-06-25 09:31:07 -0500 (Tue, 25 Jun 2013) $
 */

#ifndef VP880_API_H
#define VP880_API_H

#include "vp_hal.h"
#include "vp_CSLAC_types.h"
#include "vp_api_common.h"

#define VP880_MAX_NUM_CHANNELS   2

#ifdef VP880_INCLUDE_TESTLINE_CODE
#include "vp_api_test.h"
  #ifdef VP880_EZ_MPI_PCM_COLLECT
  #include  "vp_pcm_compute.h"
  #endif
#endif

#define VP880_MAX_MPI_DATA  15   /* Max data from any MPI read command */
#define VP880_INT_SEQ_LEN       22

#ifndef VP880_DC_FEED_LEN
#define VP880_DC_FEED_LEN    0x02
#endif

#ifndef VP880_REGULATOR_PARAM_LEN
#define VP880_REGULATOR_PARAM_LEN   0x03
#endif

#ifndef VP880_LOOP_SUP_LEN
#define VP880_LOOP_SUP_LEN      0x04
#endif

#ifndef VP880_SIGA_PARAMS_LEN
#define VP880_SIGA_PARAMS_LEN   0x0B
#endif

#ifndef VP880_RINGING_PARAMS_LEN
#define VP880_RINGING_PARAMS_LEN   0x0B
#endif

#ifndef VP880_UL_SIGREG_LEN
#define VP880_UL_SIGREG_LEN     0x02
#endif

#ifndef VP880_GX_GAIN_LEN
#define VP880_GX_GAIN_LEN       0x02
#endif

#ifndef VP880_GR_GAIN_LEN
#define VP880_GR_GAIN_LEN       0x02
#endif

#ifndef VP880_B1_FILTER_LEN
#define VP880_B1_FILTER_LEN     0x0E
#endif

#ifndef VP880_B2_FILTER_LEN
#define VP880_B2_FILTER_LEN     0x02
#endif

#ifndef VP880_ICR1_LEN
#define VP880_ICR1_LEN      0x04
#endif

#ifndef VP880_ICR2_LEN
#define VP880_ICR2_LEN      0x04
#endif

#ifndef VP880_ICR3_LEN
#define VP880_ICR3_LEN      0x04
#endif

#ifndef VP880_ICR4_LEN
#define VP880_ICR4_LEN      0x04
#endif

#ifndef VP880_ICR5_LEN
#define VP880_ICR5_LEN      0x02
#endif

#ifndef VP880_ICR6_LEN
#define VP880_ICR6_LEN      0x02
#endif

#ifndef VP880_ICR6_LEN
#define VP880_ICR6_LEN      0x02
#endif

#ifndef VP880_BAT_CALIBRATION_LEN
#define VP880_BAT_CALIBRATION_LEN 0x02
#endif

#ifndef VP880_REGULATOR_CTRL_LEN
#define VP880_REGULATOR_CTRL_LEN    0x01
#endif

#ifndef VP880_TX_PCM_BUFF_LEN
#define VP880_TX_PCM_BUFF_LEN 0x0E
#endif

#ifndef VP880_INT_SWREG_PARAM_LEN
#define VP880_INT_SWREG_PARAM_LEN   0x06
#endif

#ifndef VP880_DISN_LEN
#define VP880_DISN_LEN              0x01
#endif

#ifndef VP880_VP_GAIN_LEN
#define VP880_VP_GAIN_LEN           0x01
#endif

#ifndef VP880_SYS_STATE_LEN
#define VP880_SYS_STATE_LEN     0x01
#endif

#ifndef VP880_OP_FUNC_LEN
#define VP880_OP_FUNC_LEN         0x01
#endif

#ifndef VP880_OP_COND_LEN
#define VP880_OP_COND_LEN         0x01
#endif

#ifndef VP880_CONV_CFG_LEN
#define VP880_CONV_CFG_LEN      0x01
#endif

#ifndef VP880_DEV_MODE_LEN
#define VP880_DEV_MODE_LEN  0x01
#endif

#ifndef VP880_GEN_CTRL_LEN
#define VP880_GEN_CTRL_LEN  0x01
#endif

#ifndef VP880_CID_PARAM_LEN
#define VP880_CID_PARAM_LEN 0x01
#endif


/**< Required Vp880 Device and Line Objects for user instantiation if a Vp880 device is used */

/**< Structure that defines the Vp880 Device Profile. Current as of the first
 * Device Profile version (ver 0).
 */
typedef struct Vp880DeviceProfileType {
    uint8   profVersion;
    uint16  pcmClkRate;      /**< Used to verify valid TX/RX Timeslot setting */
    uint16  tickRate;        /**< Primary API-II tick for this device */
    uint8   devCfg1;
    uint8   clockSlot;
    uint8   systemConfig;   /**< Used to indicate system supply configuration */

#ifdef VP880_FXS_SUPPORT
    bool    peakManagement;
    bool    lowVoltOverride;
#endif
} Vp880DeviceProfileType;

/* Byte/Bit definitions from the device profile */
#define VP880_DEV_PROFILE_OPERATIONAL_CFG   (2u)
#define VP880_DEV_PROFILE_PK_PWR_MGMT       0x01
#define VP880_DEV_PROFILE_LOW_VOLT_OVERRIDE 0x02

#define VP880_DEV_PROFILE_PCLK_MSB          (6u)
#define VP880_DEV_PROFILE_PCLK_LSB          (7u)
#define VP880_DEV_PROFILE_DEVCFG1           (8u)

#define VP880_DEV_PROFILE_SYSTEM_CFG        (9u)
#define VP880_ABS_CFG_SINGLE                0x00
#define VP880_ABS_CFG_SLAVE                 0x01
#define VP880_ABS_CFG_MASTER                0x02
#define VP880_ABS_CFG_MASK                  0x03

#define VP880_DEV_PROFILE_CLOCK_SLOT        (10u)
#define VP880_DEV_PROFILE_MAX_EVENTS        (11u)
#define VP880_DEV_PROFILE_TICKRATE_MSB      (12u)
#define VP880_DEV_PROFILE_TICKRATE_LSB      (13u)
#define VP880_DEV_PROFILE_SWITCHER_CMD      (14u)
#define VP880_DEV_PROFILE_SWITCHER_DATA0    (15u)
#define VP880_DEV_PROFILE_SWITCHER_DATA1    (16u)
#define VP880_DEV_PROFILE_SWITCHER_DATA2    (17u)

#define VP880_ABS_DEV_PROFILE_YVOLT         (18u)
#define VP880_ABS_DEV_PROFILE_ZVOLT         (19u)

#define VP880_DEV_PROFILE_TRACKER_INT_SW_REG    (18u)

#define VP880_DEV_PROFILE_ABS_INT_SW_REG    (24u)


/**< Line Status types to minimize code space in line object (compared to each
 * status being maintined by a uint8 type)
 */
typedef enum Vp880LineStatusType {
    VP880_INIT_STATUS = 0x0000,

    VP880_IS_FXO = 0x0001,  /**< Set if the line is configured for FXO */

    VP880_SLS_CALL_FROM_API = 0x0002,   /**< Set if Set Line State is called
                                         * from an API function (e.g., cadence).
                                         */

    VP880_BAD_LOOP_SUP = 0x0004,    /**< Set when the Loop Supervision has been
                                     * changed in such a way inconsistent with
                                     * the user's specifications. This is done
                                     * in internal to the API to make some
                                     * functions work (e.g., Msg Wait Pulse).
                                     */

    VP880_UNBAL_RINGING = 0x0008,   /**< Set if this line uses unbal ringing */

    VP880_DP_SET1_DONE = 0x0010,    /**< Set when Dial Pulse detection machine
                                     * is "done" on the current dial pulse using
                                     * the "first" set of DP parameters
                                     */

    VP880_DP_SET2_DONE = 0x0020,    /**< Set when Dial Pulse detection machine
                                     * is "done" on the current dial pulse using
                                     * the 2nd set of DP parameters
                                     */

    VP880_LINE_IN_CAL = 0x0040,    /**< Set when line is calibrating */

    VP880_LOW_POWER_EN = 0x0080,    /**< Set when line is operating in low power
                                     * mode
                                     */

    VP880_LINE_LEAK = 0x0100,      /**< Set when leakage is detected on the line
                                    * such that low power mode is prevented.
                                    */

    VP880_LP_STANDBY_IDLE = 0x0200, /**< When set, LPM is using IDLE state for VP_LINE_STANDBY
                                     * instead of Disconnect (+-70V through 150Kohm Ring lead).
                                     */

    VP880_INIT_COMPLETE = 0x0800,  /**< Set when InitLine has been completed
                                    * on this line.
                                    */
    VP880_PREVIOUS_HOOK = 0x1000,  /**< Set if Last Hook Event reported was
                                    * off-hook, cleared if last event was
                                    * on-hook.
                                    */

    VP880_RING_GEN_NORM = 0x2000,  /**< Set when the Generators are known last
                                    * to be programmed to the application
                                    * specified ringing profile. Cleared
                                    * when line tests are run because the same
                                    * generator is used for non-ringing.
                                    */
    VP880_RING_GEN_REV = 0x4000    /**< Set when the Generators are known last
                                    * to be programmed to the application
                                    * specified ringing profile. Cleared
                                    * when line tests are run because the same
                                    * generator is used for non-ringing.
                                    */
} Vp880LineStatusType;

#if defined (VP880_INCLUDE_TESTLINE_CODE)
/* Definitions for Test arguments */
typedef union Vp880TestArgsType {
    VpTestPrepareType           prepare;
    VpTestConcludeType          conclude;
    VpTestOpenVType             openV;
    VpTestDcRLoopType           dcRloop;
    VpTestAcRLoopType           acRloop;
    VpTest3EleResAltResType     resFltAlt;
    VpTestMSocketType           mSocket;
    VpTestXConnectType          xConnect;
    VpTest3EleCapAltResType     capAlt;
    VpTestLoopCondType          loopCond;
    VpTestLoopbackType          loopback;
    VpTestRampType              ramp;
    VpTestRampInitType          rampInit;
} Vp880TestArgsType;

typedef struct Vp880LineTestCalType {
    uint8 slacState;
    uint8 vpGain;
    uint8 opCond;
    uint8 opFunction;
    uint8 icr2[VP880_ICR2_LEN];
    uint8 icr3[VP880_ICR3_LEN];
    uint8 icr4[VP880_ICR4_LEN];
    uint8 icr6[VP880_ICR6_LEN];
} Vp880LineTestCalType;

typedef struct Vp880TestHeapType {
    uint8 adcState;
    int16 nextState; /**< Used when a pcm collection routine is started */

    Vp880TestArgsType testArgs; /**< Saved test input arguments of current test */

    uint8 opCond;                           /**< Saved Operation Condition */
    uint8 opFunction;                       /**< Saved Operation Functions */
    uint8 sigCtrl;                          /**< Signal Generator Control */
    uint8 slacState;                        /**< Saved Slac State */
    uint8 sysConfig;                        /** Saved System Configurations */
    uint8 vpGain;                           /**< Voice Path Gain */
    uint8 switchReg[VP880_REGULATOR_PARAM_LEN]; /**< Switching Reg Parameters */
    uint8 dcFeed[VP880_DC_FEED_LEN];        /**< Saved DC Feed Parameters */
    uint8 disn;                             /**< Digital Imped. Scaling Network */
    uint8 SwRegCtrl;                        /**< Switching Regulator Control */

    uint8 icr1[VP880_ICR1_LEN];
    uint8 icr2[VP880_ICR2_LEN];
    uint8 icr3[VP880_ICR3_LEN];
    uint8 icr4[VP880_ICR4_LEN];
    uint8 icr6[VP880_ICR6_LEN];
    uint8 lpSuper[VP880_LOOP_SUP_LEN];      /**< Saved Loop Sup. Parameters */
    uint8 sigGenAB[VP880_SIGA_PARAMS_LEN];  /**< Saved Signal Generator A & B */
    uint8 b1Filter[VP880_B1_FILTER_LEN];    /**< Saved B1 filter coefficients */
    uint8 b2Filter[VP880_B2_FILTER_LEN];    /**< Saved B2 filter coefficients */

    /* lg res flt uses this to bump up the battery */
    uint8 batCal[VP880_BAT_CALIBRATION_LEN];

    /* used for collecting PCM data */
    bool pcmRequest;        /** < indication that pcm data was requested */
    VpPcmOperationMaskType operationMask;

    VpPcmOperationResultsType pcmResults; /** < stores the pcm operation results */

    /* Used for common setup functions */
    uint16 commonSetupState;

    /* Used for storing line event mask data */
    VpOptionEventMaskType preTestEventMask;

    /* Used for saving and restoring registers during calibration */
    Vp880LineTestCalType calRegs;

    /* Used for resflt lg speed up*/
    uint16 speedupTime;
    int16 previousAvg;
    int16 vabComputed;
    uint8 loopCnt;
    bool compensate;
    bool lowGain;

    /* Used in the capacitance test */
    int16 adcSampleBuffer[52];
    uint8 requestedSamples;
    uint8 saveConvConfig[VP880_CONV_CFG_LEN];
    bool xtraBuffer;

    /* The following members are for EZ mode calculations only*/
#ifdef VP880_EZ_MPI_PCM_COLLECT
    VpPcmComputeTempType ezPcmTemp;

    /* Used to debug under and overflow pcm collect conditions */
#ifdef VP_DEBUG
    int8 underFlowValue;
    uint32 overCnt;
    uint32 underCnt;
#endif
#endif

} Vp880TestHeapType;

typedef struct Vp880CurrentTestType {
    Vp880TestHeapType *pTestHeap;
    uint8 testHeapId;

    uint8 channelId;    /**< Channel # for "this" line on the device.  Indexed
                         * starting at 0, should not exceed the max number of
                         * lines supported by the device - 1 (max = 2, then
                         * channelId = {0, 1}
                         */

    bool prepared;          /**< indicates if the current test is prepared */
    bool preparing;         /**< indicates if the test prepare is complete */
    bool concluding;        /**< indicates that the device is concluding a test */
    bool nonIntrusiveTest;  /**< indicates a "stealth" test */
    VpTestIdType testId;    /** < indicates the test currently running */

    int16 testState;        /**< maintains the currnt state of the current TestId */
    uint16 handle;

} Vp880CurrentTestType;

typedef struct Vp880CalOffCoeffs {
    int16 nullOffset;
    int16 vabOffset;
    int16 vahOffset;
    int16 valOffset;
    int16 vbhOffset;
    int16 vblOffset;
    int16 imtOffset;
    int16 ilgOffset;
} Vp880CalOffCoeffs;

#endif /*VP880_INCLUDE_TESTLINE_CODE*/

typedef struct Vp880AbvCalData {
    uint8 passCnt;
    bool initChange;    /**< As needed, set to TRUE when a state is changed in
                         * the calibration state machine.
                         */

    int16 swyVolt[2]; /**< One per channel used to measure SWY Voltage */
    int16 swzVolt[2]; /**< One per channel used to measure SWZ Voltage */

    uint8 isrpData[VP880_INT_SWREG_PARAM_LEN];
    uint8 icr1[2][VP880_ICR1_LEN];
    uint8 icr2[2][VP880_ICR2_LEN];
    uint8 icr3[2][VP880_ICR3_LEN];
    uint8 icr4[2][VP880_ICR4_LEN];
    uint8 disnVal[2][VP880_DISN_LEN];
    uint8 vpGain[2][VP880_VP_GAIN_LEN];
    uint8 sysState[2][VP880_SYS_STATE_LEN];
    uint8 opFunc[2][VP880_OP_FUNC_LEN];
    uint8 opCond[2][VP880_OP_COND_LEN];
    uint8 converterCfg[2][VP880_CONV_CFG_LEN];

    /*
     * This the switcher adjustments as determined by battery and VAS calibration
     * procedures.
     */
    uint8 switcherAdjust[2][VP880_BAT_CALIBRATION_LEN];

    /* This is to be compatible with VVA P1.3.0 */
    int16 swyOffset[2];
    int16 swzOffset[2];

    /* This is to be compatible with VVA P1.3.0 but not currently measured. */
    int16 swxbOffset[2];
} Vp880AbvCalData;

typedef struct Vp880CalLoopData {
    int16 prevVal;
    uint8 loopNum;
    uint8 adcLoopMax;
    uint8 adcRoute;
} Vp880CalLoopData;

typedef struct Vp880VocCalData {
    int16 vocNorm;
    int16 vocRev;
} Vp880VocCalData;

typedef struct Vp880CalTypeData {
    int16 swyVolt;
    Vp880CalLoopData loopData;
    Vp880VocCalData vocData;
} Vp880CalTypeData;


#define VP880_AC_PROFILE_SIZE   (80)    /* 80 Bytes is fixed size known for 880
                                         * AC Profiles up to PW 2.0.0
                                         */
#define VP880_DC_PROFILE_SIZE   (20)    /* 16 bytes should be enough, but this
                                         * profile size has changed over time.
                                         * Safe value with a few more bytes.
                                         */

#define VP880_RINGING_PROFILE_SIZE  (22)    /* 22 byte Ringing Profile size is
                                             * the only size ever created for
                                             * VE880 API. This should be enough.
                                             */

/*
 * Bit-Masks for calibration update required if line configuration change
 * attempted during calibration. Note that this only needs to flag/update the
 * line configuration that may affect calibration.
 */
#define AC_PROFILE_UPDATE_REQ       0x1
#define DC_PROFILE_UPDATE_REQ       0x2
#define RINGING_PROFILE_UPDATE_REQ  0x4
#define CODEC_UPDATE_REQ            0x8

/*  these are in order of execution ALWAYS do not reorder them !!!! */
typedef enum Vp880CalLineState {
    VP880_CAL_SETUP                 =   0,
    VP880_CAL_SETUP_RELEASE_CLAMPS,
    VP880_RAMP_TO_POLREV_SETUP,
    VP880_RAMP_TO_POLREV_RAMP1,
    VP880_RAMP_TO_POLREV_SWAP_POLARITY,
    VP880_RAMP_TO_POLREV_RAMP2,
    VP880_RAMP_TO_POLREV_GOACTIVE,
    VP880_RAMP_TO_POLREV_COMPLETE,

    VP880_IMT_PR_SETUP              =   20,
    VP880_IMT_PR_SET_ADC,
    VP880_IMT_PR_MEASURE,

#ifdef VP880_TRACKER_SUPPORT
    VP880_VAS_PR_SETUP              =   30,
    VP880_VAS_PR_STEP,
    VP880_VAS_PR_SET_ADC,
    VP880_VAS_PR_MEASURE,
    VP880_VAS_PR_STORE,
#endif

    VP880_VAB_PR_SETUP              =   40,
    VP880_VAB_PR_MEASURE,

    VP880_VAB_PR_ADC_OFFSET_SETUP   =   50,
    VP880_VAB_PR_ADC_OFFSET_MEASURE,

    VP880_VA_PR_ADC_OFFSET_SETUP    =   60,
    VP880_VA_PR_ADC_OFFSET_MEASURE,

    VP880_VB_PR_ADC_OFFSET_SETUP    =   70,
    VP880_VB_PR_ADC_OFFSET_MEASURE,

    VP880_COLLAPSE_FEED             =   80,

    VP880_GENA_NP_OFFSET_SETUP      =   90,
    VP880_GENA_NP_OFFSET_SET_ADC,
    VP880_GENA_NP_OFFSET_MEASURE,
    VP880_GENA_NP_OFFSET_RESTORE,

    VP880_ILG_OFFSET_SETUP          =   100,
    VP880_ILG_OFFSET_MEASURE,

    VP880_ILA_OFFSET_SETUP          =   110,
    VP880_ILA_OFFSET_MEASURE,


    VP880_RESTORE_DAC               =   120,

    VP880_VAB_NP_ADC_OFFSET_SETUP   =   130,
    VP880_VAB_NP_ADC_OFFSET_MEASURE,

    VP880_VA_NP_ADC_OFFSET_SETUP    =   140,
    VP880_VA_NP_ADC_OFFSET_MEASURE,

    VP880_VB_NP_ADC_OFFSET_SETUP    =   150,
    VP880_VB_NP_ADC_OFFSET_MEASURE,

    VP880_ILA_SETUP                 =   160,
    VP880_ILA_ADJUST,
    VP880_ILA_SET_ADC,
    VP880_ILA_MEASURE,
    VP880_ILA_MEASURE_TRACKER,
    VP880_ILA_MEASURE_ABS,

    VP880_RESTORE_FEED              =   170,

    VP880_IMT_NP_SETUP              =   180,
    VP880_IMT_NP_SET_ADC,
    VP880_IMT_NP_MEASURE,

#ifdef VP880_TRACKER_SUPPORT
    VP880_VAS_NP_SETUP              =   190,
    VP880_VAS_NP_STEP,
    VP880_VAS_NP_SET_ADC,
    VP880_VAS_NP_MEASURE,
    VP880_VAS_NP_STORE,
#endif

    VP880_VAB_NP_SETUP              =   200,
    VP880_VAB_NP_MEASURE,

    VP880_CAL_ADJUST                =   210,

    VP880_CAL_RESTORE               =   254,

    /**< Used in API to force valid/unused initial value */
    VP880_CAL_ENUM_SIZE             = (VP880_CAL_RESTORE + 1)

} Vp880CalLineState;

typedef struct Vp880CalLineData {
    bool calDone;           /**< TRUE when calibration has been performed on
                             * this line
                             */
    bool reversePol;        /**< TRUE when line is in reverse polarity */

    Vp880CalTypeData typeData;

    uint8 codecReg;

    uint8 dcFeedRef[VP880_DC_FEED_LEN]; /**< Customer settings per profile */
    uint8 dcFeed[VP880_DC_FEED_LEN];    /**< Normal polarity calibrated values */
    uint8 dcFeedPr[VP880_DC_FEED_LEN];  /**< Reverse polarity calibrated values */

    Vp880CalLineState calLineState;
    uint8 sysState;

    uint8 icr2[VP880_ICR2_LEN];
    uint8 icr3[VP880_ICR3_LEN];

    uint8 disnVal[VP880_DISN_LEN];
    uint8 vpGain[VP880_VP_GAIN_LEN];
    uint8 loopSup[VP880_LOOP_SUP_LEN];

    bool forceCalDataWrite; /**< Used by the Application (hidden) to force a write to the DC Feed
                             * Registers on the next State Change using the cached calibrated data
                             * in dcFeed (normal polarity) or dcFeedPr (polarity reversal).
                             * This is a "one-shot" setting. Once the value is programmed in Set
                             * Line State, this flag is cleared.
                             */

    /* Signal generator calibration temporary backup */
    uint8 sigGenA[VP880_SIGA_PARAMS_LEN];
    uint8 calReg[VP880_ICR6_LEN];
    uint8 asscReg;

    /**< Battery Calibration values used to compute final VAS values */
    uint8 swCal[VP880_BAT_CALIBRATION_LEN];

    uint16 minVas;  /**< Minimum VAS value determined during IMT calibration.
                     * This is stored in the same format used to compute the VAS
                     * value rather than the device format. This increases the
                     * data memory space requirements slightly, but reduces the
                     * size for code space and simplifies the logic when applied.
                     */

    uint16 vasStart;    /**< Starting point for VAS to reduce calibration time.
                         * This value is based on battery adjustment and the
                         * minimum vas allowed ("minVas" above).
                         */

    /*
     * Cached user data applied after calibration in case line configuration is
     * changed during calibration.
     */
    uint8 updateFlags;  /*
                         * Bit-Mask to mark the information that needs to be
                         * updated when calibration is complete. Note that some
                         * information is automatically updated when calibration
                         * is complete simply as part of the restore process.
                         */
    uint8 acProfile[VP880_AC_PROFILE_SIZE];
    uint8 dcProfile[VP880_DC_PROFILE_SIZE];
    uint8 ringingProfile[VP880_RINGING_PROFILE_SIZE];

    VpLineStateType usrState;

} Vp880CalLineData;

typedef enum Vp880CalDeviceState {
    VP880_CAL_INIT                  =   0,
    VP880_CAL_ERROR,
    VP880_CAL_MEASURE,
    VP880_CAL_OFFSET,
    VP880_CAL_ADC,
    VP880_CAL_STATE_CHANGE,
    VP880_CAL_INVERT_POL,
    VP880_CONVERTER_CHECK,
    VP880_CAL_DONE,
    VP880_CAL_CLEANUP,
    VP880_CAL_EXIT
} Vp880CalDeviceState;

typedef struct Vp880CalDeviceData {
    Vp880AbvCalData abvData;
    Vp880CalDeviceState calDeviceState;
    int16 calSet;   /* Target values for calibration */
    uint8 iteration;    /**< Used as fail-safe to prevent infinte retries */
} Vp880CalDeviceData;

typedef struct Vp880RingParams {
    bool channelArray[VP880_MAX_NUM_CHANNELS];
    uint8 stage[VP880_MAX_NUM_CHANNELS];
    uint8 swRegParam[VP880_REGULATOR_PARAM_LEN];
} Vp880RingParams;

typedef struct Vp880LineObjectType {
    uint8 channelId;    /**< Channel # for "this" line on the device.  Indexed
                         * starting at 0, should not exceed the max number of
                         * lines supported by the device - 1 (max = 2, then
                         * channelId = {0, 1}
                         */

    uint8 ecVal;

    VpTermType termType;    /**< Termination type */

    uint16 status;  /**< Bit-mask of Vp880LineStatusType Keeps track of line state/config info*/

#ifdef VP_CSLAC_SEQ_EN
    VpSeqDataType cadence;      /**< Sequencer related information */

    /**< Array to control internally run sequences */
    VpProfileDataType intSequence[VP880_INT_SEQ_LEN];

#ifdef VP880_FXS_SUPPORT
    VpCallerIdType callerId;    /**< Caller ID related information */
    VpCidSeqDataType cidSeq;    /**< CID Sequencer related information */
    uint8 tickBeginState[VP880_CID_PARAM_LEN];
    uint8 cidBytesRemain;
    bool delayConsumed; /**< Set to TRUE when Polling Mode CID uses the loop
                         * read/delay method and has started it.
                         */

    bool suspendCid;

    VpProfilePtrType pRingingCadence;   /**< Currently used ringing cadence on
                                         * this line
                                         */

    VpProfilePtrType pMeterProfile;     /**< Currently used metering profile on
                                         * this line
                                         */

    VpProfilePtrType pCidProfileType1;  /**< Currently used caller ID profile
                                         * on this line for sequenced cid
                                         */
#endif  /* VP880_FXS_SUPPORT */

#ifdef VP880_FXO_SUPPORT
    VpDigitGenerationDataType digitGenStruct;   /**< Used on FXO lines for
                                                 * generating pulse digits
                                                 */
#endif  /* VP880_FXO_SUPPORT */
#endif  /* VP_CSLAC_SEQ_EN */

#ifdef VP880_FXS_SUPPORT
    VpDialPulseDetectType dpStruct; /**< Used on FXS lines for detecting pulse
                                     * digits
                                     */

    VpDialPulseDetectType dpStruct2;/**< Used on FXS lines for detecting pulse
                                     * digits using 2nd set of dp specs.
                                     */

    VpOptionPulseModeType pulseMode;

    /*
     * Array to hold ringing parameters used in the Signal Generator.  This is
     * needed when signal generator A is set to a tone, then set back to ringing
     * without the user re-specifying the ringing profile
     */
    uint8 ringingParams[VP880_RINGING_PARAMS_LEN];
    
    /*
     * Array to hold user defined values EXACTLY as provided in the profile. The values above
     * represent what is in the silicon IF the line state is Ringing. That will include a DC Bias
     * adjustment. It is possible to determine this next value given the ringingParams and the DC
     * Bias error and current state, but it's much easier and safer to adjust this when Config Line
     * is called.
     */
    uint8 ringingParamsRef[VP880_RINGING_PARAMS_LEN];

    uint8 leakyLineCnt; /*
                         * Keeps track of # of times off-hook was detected (LP Mode)
                         * that was not further verified after exiting LP Mode. Reset
                         * when off-hook is verified.
                         */
    /* Loop supervision parameters */
    uint8 hookHysteresis;

    bool internalTestTermApplied;

    VpOptionRingControlType ringCtrl;

    /*
     * Used to track states for Ground Start Exit Timer Management. Use two steps - first step
     * completes final state control, the second step is for final hook and ground key debounce.
     */
    uint8 gsTimerExitState;

    /*
     * Used to track states for Disconnect Exit Timer Management. Use two steps - first step
     * completes final state control, the second step is for final hook and ground key debounce.
     */
    uint8 discTimerExitState;

    VpRelayControlType relayState;   /**< Used to hold current line relay state */

#endif

    VpOptionEventMaskType lineEventsMask;
    VpOptionEventMaskType lineEvents;

    uint16 signaling1;
    uint16 signaling2;
    uint8 signalingData;    /**< Holds data for Signaling events on this line */


#ifdef VP880_FXO_SUPPORT
    uint8 fxoData;          /**< Holds data for FXO events on this line */
    uint8 preRingPolRev;    /**< The polarity detected prior to ring detect */
    VpCslacLineCondType preDisconnect;    /**< The disconnect state prior to timer start */

    uint8 ringDetMax;   /**< Stores the user specified maximum ringing detect
                         * period for FXO lines. This value may be outside the
                         * device range, in which case the SW will implement
                         * upper period detection
                         */
    uint8 ringDetMin;   /**< Stores the user specified maximum ringing detect
                         * period for FXO lines that is within the device range.
                         * This value is used as "minimum" that is detected by
                         * SW. Actual minimum period is supported by the device
                         * period detector itself.
                         */
#endif

    uint16 processData;     /**< Holds data for Process events on this line */

    uint8 responseData;     /**< Holds data for Response events on this line */

    VpCslacTimerStruct lineTimers; /**< Timers for "this" line */
    VpApiIntLineStateType lineState;    /**< Line state info used for state
                                         * transition handling and recall
                                         */
    uint8 nextSlicValue;
    uint8 slicValueCache;   /**< Used to reduce MPI reads. */

    /*
     * Holds user definition for Loop Supervision configuration when
     * "badLoopSup" is TRUE
     */
    uint8 loopSup[VP880_LOOP_SUP_LEN];

    uint16 lineEventHandle; /**< Line specific event handle information */

    VpOptionPcmTxRxCntrlType pcmTxRxCtrl;   /* Defines how the PCM highway is
                                             * set for "talk" linestates
                                             */

    uint16 dtmfDigitSense;          /**< Used to hold the DTMF digit reported
                                     * with VpDtmfDigitDetected() until the
                                     * VP_LINE_EVID_DTMF_DIG is generated.
                                     */

    VpLineIdType lineId;    /**< Application provided value for mapping a line to
                             * a line context
                             */
    /*
     * NOTE: Do not move the location or name of these members. These may be
     * used by applications to determine the gain of the GX/GR blocks in the
     * AC profile. Using return from VpSetRelGain() is not usefull because that
     * is the register setting. The customer would have to know how to convert
     * that to linear values, which is not generally documented.
     */
    struct Vp880RelGainValues {
        uint16 gxInt;       /**< Cached GX register, in 2.14 int format */
        uint16 grInt;       /**< Cached GR register, in 2.14 int format */
        int16 absGxGain;    /**< Cached ABS A-to-D Gain, simplifies code. */
        int16 absGrGain;    /**< Cached ABS D-to-A Gain, simplifies code. */
        /* Most recent non-QUIET settings for the ABS_GAIN option to support
         * VP_OPTION_ABS_GAIN_RESTORE. */
        int16 absGxRestoreOption;
        int16 absGrRestoreOption;
        uint8 absGxRestoreReg[VP880_GX_GAIN_LEN];
        uint8 absGrRestoreReg[VP880_GR_GAIN_LEN];
    } gain;

    Vp880CalLineData calLineData;

    VpOptionCodecType codec;

#ifdef VP880_FXS_SUPPORT
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
    VpHowlerModeCachedValues howlerModeCache;
#endif
    uint8 icr1Values[VP880_ICR1_LEN];   /**< Cached to minimize device access */
    uint8 icr2Values[VP880_ICR2_LEN];   /**< Cached to minimize device access */
    uint8 icr3Values[VP880_ICR3_LEN];   /**< Cached to minimize device access */
    uint8 icr4Values[VP880_ICR4_LEN];   /**< Cached to minimize device access */
    uint8 icr6Values[VP880_ICR6_LEN];   /**< Cached to minimize device access */
#endif

    uint8 opCond[VP880_OP_COND_LEN];    /**< Cached to minimize device access */
    uint8 sigGenCtrl[VP880_GEN_CTRL_LEN];   /**< Cached to minimize device access */

#ifdef VP_DEBUG
    /* For runtime enabling of debug output: */
    uint32 debugSelectMask;
#endif

} Vp880LineObjectType;

/*
 * IMPORTANT: Make sure to update the "stateInt" member of the device object if
 * the size of this type changes. There is no instance of this type itself.
 */
typedef uint32 Vp880DeviceStateIntType;
#define VP880_IS_ABS                (0x00000001L)    /**< Set when the device is ABS type */
#define VP880_RLY_PROT_REQ          (0x00000002L)    /**< Set when the device requires relay protction on I/O 1 */
#define VP880_HAS_TEST_LOAD_SWITCH  (0x00000004L)    /**< Set when test load swith is available */
#define VP880_HAS_CALIBRATE_CIRCUIT (0x00000008L)    /**< Set when test cal circuit is available */

#define VP880_IS_HIGH_VOLTAGE       (0x00000010L)    /**< Set when device is a high voltage device */
#define VP880_IS_SINGLE_CHANNEL     (0x00000020L)    /**< Set when a single channel device is found*/
#define VP880_SYS_CAL_RESET         (0x00000040L)    /**< Set when Apply Cal is passed NULL to reset calibration values */
#define VP880_LINE0_IS_FXO          (0x00000080L)    /**< Set if device detection indicates line0 as FXO */

#define VP880_LINE1_IS_FXO          (0x00000100L)    /**< Set if device detection indicates line1 as FXO */
#define VP880_WIDEBAND              (0x00000200L)    /**< Set if device supports Wideband mode */
#define VP880_LINE0_LP              (0x00000400L)    /**< Set if line 0 allows low power mode */
#define VP880_LINE1_LP              (0x00000800L)    /**< Set if line 1 allows low power mode */

#define VP880_IS_FXO_ONLY           (0x00001000L)    /**< Set when the device contains only FXO lines */
#define VP880_SYS_CAL_COMPLETE      (0x00002000L)    /**< Set when the system calibration structure has been initialied */
#define VP880_CAL_RELOAD_REQ        (0x00004000L)    /**< Set when the line calibration values need to be reloaded. */
#define VP880_FORCE_FREE_RUN        (0x00008000L)    /**< Set when app calls VpFreeRun() (start), cleared when called with stop.
                                                       * This prevents the VP-API-II from automatically exiting free run mode
                                                       * upon PCLK recovery.
                                                       */

#define VP880_SWY_DECAY_CMP         (0x00010000L)
#define VP880_SWZ_DECAY_CMP         (0x00020000L)

#define VP880_IS_TEST_CAPABLE       (0x00040000L)   /**< Set for devices that support Line Test */

/*
 * This value is set when either Device Calibration (VpInitDevice()/VpCalCodec()
 * completes successfully, or when loaded values by Apply System Coeff. After
 * VpInitDevice() this should only be NOT SET if device calibration failed or
 * if null was passed to Apply System Coeff.
 */
#define VP880_DEVICE_CAL_COMPLETE   (0x00080000L)


#define VP880_SYS_CAL_POLARITY_LENGTH  2
#define VP880_SYS_CAL_CHANNEL_LENGTH   2

/* Contains calibration error values -- in +/-10mv LSB */
typedef struct Vp880SysCalResultsType {
    int16 abvError[2];

    int16 vocOffset[VP880_SYS_CAL_CHANNEL_LENGTH][VP880_SYS_CAL_POLARITY_LENGTH];
    int16 vocError[VP880_SYS_CAL_CHANNEL_LENGTH][VP880_SYS_CAL_POLARITY_LENGTH];

    int16 sigGenAError[VP880_SYS_CAL_CHANNEL_LENGTH][VP880_SYS_CAL_POLARITY_LENGTH];

    int16 ila20[VP880_SYS_CAL_CHANNEL_LENGTH];
    int16 ila25[VP880_SYS_CAL_CHANNEL_LENGTH];
    int16 ila32[VP880_SYS_CAL_CHANNEL_LENGTH];
    int16 ila40[VP880_SYS_CAL_CHANNEL_LENGTH];

    int16 ilaOffsetNorm[VP880_SYS_CAL_CHANNEL_LENGTH];
    int16 ilgOffsetNorm[VP880_SYS_CAL_CHANNEL_LENGTH];

    /* Used for Tracker only */
    uint8 vas[VP880_SYS_CAL_CHANNEL_LENGTH][VP880_SYS_CAL_POLARITY_LENGTH];

    int16 vagOffsetNorm[VP880_SYS_CAL_CHANNEL_LENGTH];
    int16 vagOffsetRev[VP880_SYS_CAL_CHANNEL_LENGTH];
    int16 vbgOffsetNorm[VP880_SYS_CAL_CHANNEL_LENGTH];
    int16 vbgOffsetRev[VP880_SYS_CAL_CHANNEL_LENGTH];

    /* Used for ABS only */
    uint8 absNormCal[VP880_SYS_CAL_CHANNEL_LENGTH];
    uint8 absPolRevCal[VP880_SYS_CAL_CHANNEL_LENGTH];

    int16 swyOffset[VP880_SYS_CAL_CHANNEL_LENGTH];     /**< Used to hold SWY Offset */
    int16 swzOffset[VP880_SYS_CAL_CHANNEL_LENGTH];     /**< Used to hold SWZ Offset */
    int16 swxbOffset[VP880_SYS_CAL_CHANNEL_LENGTH];    /**< Used to hold SWXB Offset */

    /* Used for capacitance line test only */
    int32 tipCapCal[VP880_SYS_CAL_CHANNEL_LENGTH];
    int32 ringCapCal[VP880_SYS_CAL_CHANNEL_LENGTH];
} Vp880SysCalResultsType;

#define VP880_CAL_STRUCT_SIZE   (40*sizeof(int16) + 8*sizeof(uint8) + 4*sizeof(int32))

typedef struct Vp880DeviceObjectType {
    /* Device identifier set by the application -- passed to the HAL */
    VpDeviceIdType deviceId;

    /* Silicon RCN/PCN and number of channels supported */
    VpDeviceStaticInfoType staticInfo;

    /*
     * Weak attempt to put everything that changes during run-time into a
     * single structure. Good idea in principle, but in practice the devices
     * are too dis-similar to share many common structures.
     *
     * This holds the "last channel checked" event information, clock fault,
     * and battery fault.
    */
    VpDeviceDynamicInfoType dynamicInfo;

    /*
     * Variety of common device level status such as init/not init, init in
     * progress, test buffer read y/n?, etc.. Internally used by the VP-API-II
     * to communicate device state.
     *
     * This is a bit-mask of VpDeviceBusyFlagsType values
     */
    uint16 state;

    /*
     * Similar to common device level status, this is for 880 specific type of
     * device level information. Such as ABS/Tracker, LPM lines, etc..
     *
     * This is a bit-mask of Vp880DeviceStateIntType values
     */
    Vp880DeviceStateIntType stateInt;

    /*-----------------3/18/2010 3:51PM-----------------
     * Device Level OPTION Values
     * --------------------------------------------------*/
    /* OPTION_ID_EVENT_MASK for device level events */
    VpOptionEventMaskType deviceEventsMask;

    /* OPTION_ID_PULSE */
    VpOptionPulseType pulseSpecs;

    /* OPTION_ID_PULSE2 */
    VpOptionPulseType pulseSpecs2;

    /* OPTION_ID_CRITICAL_FLT */
    VpOptionCriticalFltType criticalFault;

    /*
     * Stored device level events. Note that this is an alternative method to
     * an event queue. So if an event queue is later used, this value could be
     * removed and all it's uses replaced.
     */
    VpOptionEventMaskType deviceEvents;

    /*
     * Device level timers such as clock fault, LPM Enter/Exit, VpCalCodec()
     * steps, etc..
     */
    uint16 devTimer[VP_DEV_TIMER_LAST];

    /*
     * Soft values from the device profile that are used throughout the VP-API-II
     * (not only in Init). Note that other values may be provided by the device
     * profile, but are stored in other ways generally because they also may NOT
     * be provided in the device profile (i.e., API has default values).
     */
    Vp880DeviceProfileType devProfileData;

    /* This is the profile pointer table per the VP-API-II User's Guide. */
    VpCSLACDeviceProfileTableType devProfileTable;

    /*
     * This is the structure indicating which profile table entries have been
     * initialized. If profile tables are disabled, this and the device profile
     * table (devProfileTable) can be removed.
     */
    VpCSLACProfileTableEntryType profEntry;

    /*
     * Raw information from the device signaling register last read. This
     * information is internally converted into useable VP-API-II information.
     */
    uint8 intReg[VP880_UL_SIGREG_LEN];
    uint8 intReg2[VP880_UL_SIGREG_LEN];

    /* Buffer to support VpLowLevelCmd() only. */
#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
    uint8 mpiData[VP880_MAX_MPI_DATA];
#endif

    /*
     * Data length copied into mpiData buffer -- used for VpLowLevelCmd() and
     * Apply/Get System Calibration
     */
    uint8 mpiLen;

    /* Handle to return with events as applicable */
    uint16 eventHandle;

    /*
     * VP-API-II timing in 500us increments (per the VP-API-II User's Guide).
     * Generally incremented at the tick rate.
     */
    uint16 timeStamp;
    int16 timeRemainder;    /**< leftover of tick time for tickrates not divisible by 0.5ms */

    /* Data returned with response events as applicable */
    uint8 responseData;

    /* Data returned with "Get Option" events */
    VpGetResultsOptionsType getResultsOption;

    /* Data returned with VpSetRelGain() function call/event. */
    VpRelGainResultsType relGainResults;

#if defined (VP880_INCLUDE_TESTLINE_CODE)
    /* Testing structure */
    VpTestResultType testResults;

    Vp880CurrentTestType currentTest;

    /**< Used to hold calibration offset coeffs. One per channel */
    Vp880CalOffCoeffs calOffsets[VP880_MAX_NUM_CHANNELS];

#endif /* VP880_INCLUDE_TESTLINE_CODE */


    /*
     * Used to hold battery switch calibration offset. One per channel, per
     * polarity
     */
    uint8 calState;
    Vp880CalDeviceData calData;

    /*
     * Used to get better hook granularity and pcm buffered data.
     */
    uint8 txBuffer[VP880_TX_PCM_BUFF_LEN];
    uint8 txBufferDataRate;

    /*
     * These switcher paramaters are from the user setting in the
     * device profile. They should never be changed except in Init (in case
     * the values provided are illegal).
     */
    uint8 swParams[VP880_REGULATOR_PARAM_LEN];

    /*
     * These switcher paramaters should always be a SW representation of the
     * silicon used to reduce device level access. The driver MUST change this
     * setting if the device itself is changed.
     */
    uint8 swParamsCache[VP880_REGULATOR_PARAM_LEN];

    /*
     * These internal switcher paramaters are from the user setting in the
     * device profile. They should never be changed except in Init (in case
     * the values provided are illegal).
     */
    uint8 intSwParams[VP880_INT_SWREG_PARAM_LEN];

    /*
     * These internal switcher paramaters are for Free Run mode only and should
     * only be set during Init Device (in case a set was not provided or if the
     * provided set is illegal).
     */
    uint8 intSwParamsFR[VP880_INT_SWREG_PARAM_LEN];

    uint8 yVolt;    /* Y-Switcher Target in 1V step */
    uint8 zVolt;    /* Z-Switcher Target in 1V step */

#if defined (VP_CC_880_SERIES) || defined (VP_CC_KWRAP)
    Vp880SysCalResultsType vp880SysCalData;
#endif

    /* used to store the in-rush current function data */
    Vp880RingParams ringParams;

#ifdef VP_DEBUG
    /* For runtime enabling of debug output: */
    uint32 debugSelectMask;
#endif

    /* Used to hold the device level ecVal (to keep track of WB mode). */
    uint8 ecVal;

    /*
     * Holds the last channel information changed to/from Wideband mode since
     * the last tick. If = 0, no change has been made. If = 1, ch0 if = 2, ch1.
     * This is a handshake with the tick which detects it is not 0, makes the
     * appropriate change and sets it back to 0.
     */
    uint8 lastCodecChange;

    /* Device Mode Register (0x5E/5F) cached to minimize MPI traffic */
    uint8 devMode[VP880_DEV_MODE_LEN];
} Vp880DeviceObjectType;

#endif  /**< vp880_api.h */
