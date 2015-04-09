/** \file vp886_api_int.h
 * vp886_api_int.h
 *
 * Header file for the vp886 series API-II c files.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11582 $
 * $LastChangedDate: 2014-10-03 10:57:57 -0500 (Fri, 03 Oct 2014) $
 */

#ifndef VP886_API_INT_H
#define VP886_API_INT_H

#include "vp_api_event.h"
#include "vp_api_option.h"
#include "vp_CSLAC_types.h"
#include "vp_pulse_decode.h"


/* Macro for extracting the device type (VpDeviceType) from the Device Object */
#define VP886_DEVICE_SERIES(pDevObj) \
    (((pDevObj)->staticInfo.rcnPcn[VP886_R_RCNPCN_PCN_IDX] & VP886_R_RCNPCN_PCN_ABS_TRACKER) ? VP_DEV_887_SERIES : VP_DEV_886_SERIES)
#define VP886_IS_TRACKER(pDevObj) /* 887 */ \
    ((pDevObj)->staticInfo.rcnPcn[VP886_R_RCNPCN_PCN_IDX] & VP886_R_RCNPCN_PCN_ABS_TRACKER)
#define VP886_IS_ABS(pDevObj) /* 886 */ \
    !VP886_IS_TRACKER(pDevObj)
#define VP886_IS_HV(pDevObj) \
    (((pDevObj)->staticInfo.rcnPcn[VP886_R_RCNPCN_PCN_IDX] & VP886_R_RCNPCN_PCN_LV_HV) \
        && !((pDevObj)->devProfileData.lowVoltOverride))
#define VP886_IS_LV(pDevObj) \
    !VP886_IS_HV(pDevObj)
#define VP886_IS_1CH(pDevObj) \
    (!((pDevObj)->staticInfo.rcnPcn[VP886_R_RCNPCN_PCN_IDX] & VP886_R_RCNPCN_PCN_2CH))
#define VP886_REVISION(pDevObj) \
    ((pDevObj)->staticInfo.rcnPcn[VP886_R_RCNPCN_RCN_IDX])
#define VP886_IS_SF(pDevObj) \
    (!((pDevObj)->staticInfo.rcnPcn[VP886_R_RCNPCN_PCN_IDX] & VP886_R_RCNPCN_PCN_NOT_SF) && VP886_REVISION(pDevObj) >= 7)
#define VP886_IS_SF0(pDevObj) /* Apollo2 in 48-pin package with SF PCN */ \
    (!((pDevObj)->staticInfo.rcnPcn[VP886_R_RCNPCN_PCN_IDX] & VP886_R_RCNPCN_PCN_NOT_SF) && VP886_REVISION(pDevObj) < 7)
#define VP886_IS_SF1(pDevObj) \
    (VP886_IS_SF(pDevObj) && VP886_REVISION(pDevObj) >= 8 && VP886_IS_1CH(pDevObj))
#define VP886_IS_SF1AB(pDevObj) \
    (VP886_IS_SF(pDevObj) && VP886_REVISION(pDevObj) == 7 && VP886_IS_1CH(pDevObj))
#define VP886_IS_SF2(pDevObj) \
    (VP886_IS_SF(pDevObj) && VP886_REVISION(pDevObj) >= 8 && !VP886_IS_1CH(pDevObj))

/* Timer ID Definitions (uint16) */
#define VP886_TIMERID_INIT_DEVICE               0
#define VP886_TIMERID_CADENCE                   1
#define VP886_TIMERID_CAL_CODEC                 2
#define VP886_TIMERID_CAL_LINE                  3
#define VP886_TIMERID_HOOK_FREEZE               4
#define VP886_TIMERID_PULSE_DECODE              5
#define VP886_TIMERID_SENDSIGNAL                6
#define VP886_TIMERID_METERING                  7
#define VP886_TIMERID_CID                       8
#define VP886_TIMERID_INT_TEST_TERM             9
#define VP886_TIMERID_POLREV_FIX                10
#define VP886_TIMERID_RING_EXIT                 11
#define VP886_TIMERID_RING_EXIT_CLEANUP         12
#define VP886_TIMERID_USER                      13
#define VP886_TIMERID_HOWLER                    14
#define VP886_TIMERID_ABS_POWER                 15
#define VP886_TIMERID_LINE_TEST                 16
#define VP886_TIMERID_THERMAL_RINGING           17
#define VP886_TIMERID_BATFLT_POLL               18
#define VP886_TIMERID_QUICKCAL                  19
#define VP886_TIMERID_GKEY_FREEZE               20
#define VP886_TIMERID_GND_FLT_PROT              21
#define VP886_TIMERID_GROUNDSTART               22
#define VP886_TIMERID_RINGTRIP_CONFIRM          23

/* Use this as 'channelId' for device timers */
#define VP886_DEV_TIMER 255

/* Use this for 'channelId' to Vp886PushEvent() for device events */
#define VP886_DEV_EVENT 255

/* Hardware Reset wait time in ms */
#define VP886_HW_RST_WAIT_TIME 3

/* CFAIL wait time for clock to settle at Hardware Reset */
#define VP886_CFAIL_WAIT_TIME 10

/* Vref turn on wait time in ms */
#define VP886_VREF_WAIT_TIME 10

/* Charge Pump turn on wait time in ms */
#define VP886_CP_WAIT_TIME 10

/* Disconnect come up wait time in ms */
#define VP886_DISC_WAIT_TIME 10

/* Switcher enable wait time in ms */
#define VP886_SW_EN_WAIT_TIME 10

/* Hook freeze durations, in ms */
#define VP886_HOOKFREEZE_DISC_EXIT 100
#define VP886_HOOKFREEZE_POLREV 100
#define VP886_HOOKFREEZE_LOWPOWER 30
#define VP886_HOOKFREEZE_CAL_CMP 25
#define VP886_HOOKFREEZE_RING_TRIP 100

/* Groundkey freeze durations, in ms */
#define VP886_GKEYFREEZE_DISC_EXIT 25
#define VP886_GKEYFREEZE_CAL_CMP 25

/* Time between exiting ringing and restoring switcher settings, in ms. */
#define VP886_RING_EXIT_DELAY 50

/* States for ring exit to polrev workaround.  Passed through timer handle */
#define VP886_RING_EXIT_CLEANUP_CHECK   0
#define VP886_RING_EXIT_CLEANUP_FINISH  1
/* Time between each check of the ring exit state when exiting ringing to
   an opposite polarity state, in ms */
#define VP886_RING_EXIT_CLEANUP_CHECK_DELAY 5
/* Time to allow the battery and internal states to settle after ringing has
   finished, in ms */
#define VP886_RING_EXIT_CLEANUP_FINISH_DELAY 9

/* Time between the initial and steady ICR1 settings for the internal
   test termination, in ms */
#define VP886_INT_TEST_TERM_DELAY 200

/* Time after a polrev to restore ICR2/ICR3 speedup settings,
   in ms (VP886_TIMERID_POLREV_FIX) */
#define VP886_POLREV_FIX_TIME 50

/* Time to delay decreases in ABS power mode, in ms (VP886_TIMERID_ABS_POWER) */
#define VP886_ABS_POWER_DECREASE_DELAY 50

/* Time between polling for battery faults to clear, in ms (VP886_TIMERID_BATFLT_POLL) */
#define VP886_BATFLT_POLL_TIME 150

/* If successive overcurrent alarms occur within this interval (in 0.5ms units),
   we generate the overcurrent battery fault. */
#define VP886_OVERCURRENT_INTERVAL 100 /* 50ms */

/* Time to leave c_long_on enabled in ACTIVE for the groundstart workaround, in ms */
#define VP886_GROUNDSTART_DELAY 210

/* Maximum number of times to read the signaling register. Vp88GetEvent() will
   read the register until either there is no change or it reaches this limit.
   This limit prevents locking up the system if a signal oscillates.
   On polled or level-triggered systems this can be a low number because the
   interrupt will be serviced again later.  But on edge triggered systems an
   unserviced interrupt could be left forever, since no new edge would occur.
   In edge triggered mode, set this higher so that it should only trigger to
   avoid a true lockup. */
#ifdef VP886_INTERRUPT_EDGETRIG_MODE
#define VP886_MAX_SIGREG_READS 20
#else
#define VP886_MAX_SIGREG_READS 3
#endif

/* Byte/Bit definitions from the device profile */
#define VP886_DEV_PROFILE_MPI_LEN               (5u)

#define VP886_DEV_PROFILE_DEV_CFG_DATA0         (7u)

#define VP886_DEV_PROFILE_PD_CORR_MASK          0xF0

#define VP886_DEV_PROFILE_SW_CONF_MASK          0x0F
#define VP886_DEV_PROFILE_SW_CONF_BB            0x00
#define VP886_DEV_PROFILE_SW_CONF_FB100         0x01
#define VP886_DEV_PROFILE_SW_CONF_FB150         0x02
#define VP886_DEV_PROFILE_SW_CONF_IB100         0x03
#define VP886_DEV_PROFILE_SW_CONF_IB150         0x04
#define VP886_DEV_PROFILE_SW_CONF_IB100_5IN     0x05
#define VP886_DEV_PROFILE_SW_CONF_ABS100        0x01
#define VP886_DEV_PROFILE_SW_CONF_ABS120        0x02

#define VP886_DEV_PROFILE_OC_CNT0_OFFSET        (8u)
#define VP886_DEV_PROFILE_OC_CNT1_OFFSET        (9u)

#define VP886_DEV_PROFILE_BLANKING_OFFSET       (10u)

#define VP886_DEV_PROFILE_IO2_USE_BITS          0x03
#define VP886_DEV_PROFILE_IO2_USE_DIG_SIG       0x00
#define VP886_DEV_PROFILE_IO2_USE_DIG_INT       0x01
#define VP886_DEV_PROFILE_IO2_USE_VMM           0x02
#define VP886_DEV_PROFILE_IO2_USE_RSVD          0x03

#define VP886_DEV_PROFILE_ABS_SUPP_CFG          0xF0

#define VP886_DEV_PROFILE_ABS_SUPP_CFG_MODE     0xC0
#define VP886_DEV_PROFILE_ABS_SUPP_CFG_SINGLE   0x00
#define VP886_DEV_PROFILE_ABS_SUPP_CFG_SLAVE    0x40
#define VP886_DEV_PROFILE_ABS_SUPP_CFG_MASTER   0x80

#define VP886_DEV_PROFILE_ABS_SUPP_CFG_Y_SLAVE  0x10
#define VP886_DEV_PROFILE_ABS_SUPP_CFG_Z_SLAVE  0x20

#define VP886_DEV_PROFILE_BLANKING_USE_BITS     0x07

#define VP886_DEV_PROFILE_CP_ENABLE_MASK        0x01
#define VP886_DEV_PROFILE_CP_PROTECTION_MASK    0x0E
#define VP886_DEV_PROFILE_SHARED_SUPPLY         0x80

#define VP886_RING_PROF_LOW_ILR_FLAG            0x80

#define VP886_RING_PROF_LOW_ILR_MASK            0x3E
#define VP886_RING_PROF_BAL_UNBAL               0x01

#define VP886_CAL_PROF_TRACKER_MAX_VERSION  5
#define VP886_CAL_PROF_ABS_MAX_VERSION      3
/* Lengths defined number of bytes following the mpi data
  (length byte - 2 - mpi length) */
#define VP886_CAL_PROF_TRACKER_LENGTH_V0    106
#define VP886_CAL_PROF_TRACKER_LENGTH_V1    110
#define VP886_CAL_PROF_TRACKER_LENGTH_V2    114
#define VP886_CAL_PROF_TRACKER_LENGTH_V3    118
#define VP886_CAL_PROF_TRACKER_LENGTH_V4    140
#define VP886_CAL_PROF_TRACKER_LENGTH_V5    140
#define VP886_CAL_PROF_TRACKER_LENGTH       VP886_CAL_PROF_TRACKER_LENGTH_V5
#define VP886_CAL_PROF_ABS_LENGTH_V0        114
#define VP886_CAL_PROF_ABS_LENGTH_V1        118
#define VP886_CAL_PROF_ABS_LENGTH_V2        140
#define VP886_CAL_PROF_ABS_LENGTH_V3        140
#define VP886_CAL_PROF_ABS_LENGTH           VP886_CAL_PROF_ABS_LENGTH_V3

/* Tone generator enable bits */
#define VP886_TONEGEN_BIAS  0x10
#define VP886_TONEGEN_A     0x01
#define VP886_TONEGEN_B     0x02
#define VP886_TONEGEN_C     0x04
#define VP886_TONEGEN_D     0x08


/* Definitions used for VP_OPTION_ID_DCFEED_PARAMS */

/* VOC is a 3-bit field with step size of 3 V and a range of either
   12-33 or 36-57 V depending on the VOCSFT bit. */
#define VP886_VOC_MIN           12000L
#define VP886_VOC_MAX           57000L
#define VP886_VOC_OFFSET        12000L
#define VP886_VOC_STEP          3000L
#define VP886_VOC_TARGET_STEP   1000L

/* ILA is a 5-bit value with a 18-49 mA scale (1 mA per step + 18 mA
   offset) */
#define VP886_ILA_MIN           18000L
#define VP886_ILA_MAX           49000L
#define VP886_ILA_OFFSET        18000L
#define VP886_ILA_STEP          1000L

/* TSH is a 3-bit value with a 7-14 mA scale (1 mA per step + 7 mA
   offset) */
#define VP886_TSH_MIN           7000L
#define VP886_TSH_MAX           14000L
#define VP886_TSH_OFFSET        7000L
#define VP886_TSH_STEP          1000L

/* LPTSH is a 3-bit value with a 14-28 V scale (2 V per step + 14 V
   offset) */
#define VP886_LPTSH_MIN         14000L
#define VP886_LPTSH_MAX         28000L
#define VP886_LPTSH_OFFSET      14000L
#define VP886_LPTSH_STEP        2000L

/* TGK is a 3-bit value with a 0-42 mA scale (6 mA per step) */
#define VP886_TGK_MIN           0L
#define VP886_TGK_MAX           42000L
#define VP886_TGK_OFFSET        0L
#define VP886_TGK_STEP          6000L

/* Allow battery floor voltage from -5 to -50 V in 1V steps */
#define VP886_BATTFLR_MIN       -50000L
#define VP886_BATTFLR_MAX       -5000L
#define VP886_BATTFLR_OFFSET    0L
#define VP886_BATTFLR_STEP      -1000L


/* Definitions used for VP_OPTION_ID_RINGING_PARAMS */

/* Frequency is a 15-bit value with a 0-12000 Hz range.  12000000 mHz
   per 0x8000 reduces to 46875 mHz per 0x80. */
#define VP886_FREQ_MIN          0L
#define VP886_FREQ_MAX          12000000L
#define VP886_FREQ_STEP_NUM     46875L
#define VP886_FREQ_STEP_DEN     0x80L

/* For trapezoidal ringing, frequency is defined as 8000Hz / FRQB.
   This means the maximum is 8000 Hz (8000000 mHz), and the minimum
   is 8000Hz / 0x7FFF, or 244.1 mHz.  FRQB can be calculated by
   8000000mHz / frequency. */
#define VP886_TRAPFREQ_MIN      244L
#define VP886_TRAPFREQ_MAX      8000000L

/* Amplitude is a 16-bit value with a +/- 154.4V range.  This
   means 154400 mV per 0x8000, which reduces to 4825 mV per 0x400. */
#define VP886_AMP_MIN           -154400L
#define VP886_AMP_MAX           154400L
#define VP886_AMP_STEP_NUM      4825L
#define VP886_AMP_STEP_DEN      0x400L

/* DC Bias is a 16-bit value with a +/- 154.4V range.  This
   means 154400 mV per 0x8000, which reduces to 4825 mV per 0x400. */
#define VP886_BIAS_MIN          -154400L
#define VP886_BIAS_MAX          154400L
#define VP886_BIAS_STEP_NUM     4825L
#define VP886_BIAS_STEP_DEN     0x400L

/* Ring trip threshold is a 7-bit value with a 0-63.5 mA scale (0.5 mA
   per step). */
#define VP886_RTTH_MIN          0L
#define VP886_RTTH_MAX          63500L
#define VP886_RTTH_OFFSET       0L
#define VP886_RTTH_STEP         500L

/* Ring current limit can be specified in either of two overlapping ranges
   for a total range of 22-112 mA */
#define VP886_ILR_MIN           22000L
#define VP886_ILR_MAX           112000L

/* Ring current limit is a 5-bit value with a 50-112 mA scale (2 mA per
   step + 50 mA offset). */
#define VP886_ILR_OFFSET        50000L
#define VP886_ILR_STEP          2000L

/* Low range ring current limit is a 5-bit value with a 18-49 mA scale.
   But in order to work around an issue with how the device targets 7/8
   of the programmed ILR value, we're going to force a +4mA adjustment
   in the calibration register.  This effectively bumps up the offset
   in programming the ILR field (1 mA per step + 22 mA offset). */
#define VP886_ILR_LOW_OFFSET    22000L
#define VP886_ILR_LOW_STEP      1000L
/* Only use the low range for values that will round to 49mA or below */
#define VP886_ILR_LOW_MAX       49499L

/* Trapezoidal rise time is defined as 2.7307sec / FRQA. This means the
   maximum is 2.7307 sec (2730700 usec), and the minimum
   is 2.7307sec / 0x7FFF, or 83.3 usec.  FRQA can be calculated by
   2730700usec / trapRiseTime. */
#define VP886_TRAPRISE_MIN      83L
#define VP886_TRAPRISE_MAX      2730700L

/* States adaptive ringing timer.  Passed through timer handle */
#define VP886_THERMAL_RINGING_DEBOUNCE   0
#define VP886_THERMAL_RINGING_CADENCE    1

/* Definitions related to event masks */
#define VP886_NO_MASK   0x0000

#define VP886_NONSUPPORT_FAULT_EVENTS       (VP_EVCAT_FAULT_UNDEFINED \
                                           | VP_DEV_EVID_WDT_FLT \
                                           | VP_LINE_EVID_SYNC_FLT \
                                           | VP_LINE_EVID_RES_LEAK_FLT \
                                           | VP_LINE_EVID_SEAL_CUR_FLT \
                                           | VP_DEV_EVID_SYSTEM_FLT)

#define VP886_NONSUPPORT_SIGNALING_EVENTS   (VP_EVCAT_SIGNALING_UNDEFINED \
                                           | VP_LINE_EVID_MTONE \
                                           | VP_LINE_EVID_US_TONE_DETECT \
                                           | VP_LINE_EVID_DS_TONE_DETECT \
                                           | VP_DEV_EVID_SEQUENCER)

#define VP886_NONSUPPORT_RESPONSE_EVENTS    (VP_EVCAT_RESPONSE_UNDEFINED \
                                           | VP_DEV_EVID_BOOT_CMP \
                                           | VP_LINE_EVID_LLCMD_TX_CMP \
                                           | VP_LINE_EVID_LLCMD_RX_CMP \
                                           | VP_DEV_EVID_DNSTR_MBOX \
                                           | VP_LINE_EVID_SLAC_INIT_CMP)

#define VP886_NONSUPPORT_TEST_EVENTS        (VP_EVCAT_TEST_UNDEFINED \
                                           | VP_LINE_EVID_TEST_RSVD1 \
                                           | VP_LINE_EVID_DTONE_DET \
                                           | VP_LINE_EVID_DTONE_LOSS \
                                           | VP_DEV_EVID_STEST_CMP \
                                           | VP_DEV_EVID_CHKSUM)

#define VP886_NONSUPPORT_PROCESS_EVENTS     (VP_EVCAT_PROCESS_UNDEFINED \
                                           | VP_LINE_EVID_MTR_CAD \
                                           | VP_LINE_EVID_MTR_ROLLOVER \
                                           | VP_LINE_EVID_AUTO_LOOP_COND)


#define VP886_NONSUPPORT_FXO_EVENTS         (VP_EVCAT_PROCESS_UNDEFINED \
                                           | VP_EVCAT_FXO_MASK_ALL)


typedef enum {
    VP886_LINE_INIT_IN_PROGRESS = 0x0001, /* Currently not used */
    VP886_LINE_INIT_CMP         = 0x0002, /* Currently not used */
    VP886_LINE_IN_CAL           = 0x0004, /* Set during VpInitDevice(),
                                             VpCalCodec(), and VpCalLine() */
    VP886_LINE_GET_LOOP_COND    = 0x0010, /* Get Loop Condition in progress */
    VP886_LINE_TEST             = 0x0020, /* Line Test in progress */
    VP886_LINE_DEFAULT_OPTIONS  = 0x0040, /* Skip SetOption critical sections
                                             during VpImplementDefaultSettings() */
    VP886_LINE_QUICKCAL         = 0x0080,
    VP886_IGNORE_ALL_BUSY_FLAGS = 0x4000  /* For temporarily ignoring busy flags */
} Vp886LineBusyFlagsType;


typedef struct {
    bool voice;                     /* PCM TX/RX allowed */
    bool codec;                     /* Codec activated, tones allowed */
    bool polrev;                    /* Polrev bit set */
    VpLineStateType normalEquiv;    /* Equivalent normal state */
    VpLineStateType polrevEquiv;    /* Equivalent polrev state */
    VpLineStateType oppositeEquiv;  /* Equivalent opposite polarity state */
} Vp886LineStateInfoType;


/* Prototypes for functions that appear in the function pointer table
   or that are used internally between different files. */

/* In vp886_calibration_common.c */

VpStatusType
Vp886CalCodec(
    VpLineCtxType *pLineCtx,
    VpDeviceCalType mode);

void
Vp886StoreIcal(
    VpDevCtxType *pDevCtx);

void
Vp886SetSingleAdcMath(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 sadcRoute,
    uint16 settlingTimeMs,
    uint16 integrationTimeMs);

void
Vp886SetGroupAdcMath(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 sadcRoute[5],
    uint16 settlingTimeMs,
    uint16 integrationTimeMs);

int16
Vp886GetSingleAdcMath(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    bool adcScale);

bool
Vp886GetGroupAdcMath(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    int16* result,
    bool adcScale,
    uint8 numRoutes);

void
Vp886SetSingleVadcMath(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    uint8 vadcRoute,
    uint16 settlingTimeMs,
    uint16 integrationTimeMs);

int16
Vp886GetSingleVadcMath(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    bool adcScale);

bool
Vp886LongitudinalCalibration(
    VpDevCtxType *pDevCtx,
    uint8 channelId);

VpStatusType
Vp886CalLine(
    VpLineCtxType *pLineCtx);

VpStatusType
Vp886Cal(
    VpLineCtxType *pLineCtx,
    VpCalType calType,
    void *inputArgs);

VpStatusType
Vp886ApplyCalGeneral(
    VpLineCtxType *pLineCtx);

VpStatusType
Vp886ApplyCalDC(
    VpLineCtxType *pLineCtx);

VpStatusType
Vp886ApplyCalRing(
    VpLineCtxType *pLineCtx);

void
Vp886ProgramCalRegisters(
    VpLineCtxType *pLineCtx);

VpStatusType
Vp886GetCalProfile(
    VpLineCtxType *pLineCtx,
    uint8 *calProf);

VpStatusType
Vp886SetDefaultCal(
    Vp886DeviceObjectType *pDevObj,
    uint8 channelId);

void
Vp886CalCodecHandler(
    VpDevCtxType *pDevCtx);

void
Vp886CalCodecSM(
    VpDevCtxType *pDevCtx,
    uint8 channelId);

void
Vp886CalLineSM(
    VpLineCtxType *pLineCtx);

VpStatusType
Vp886QuickCalStart(
    VpLineCtxType *pLineCtx,
    uint16 timerId,
    uint32 timerHandle,
    uint16 sadcFlag,
    bool externalCall);

VpStatusType
Vp886QuickCalStop(
    VpLineCtxType *pLineCtx);

VpStatusType
Vp886QuickCalHandler(
    VpLineCtxType *pLineCtx);


/* End vp886_calibration_common.c */


/* In vp_886_common.c */
VpStatusType
Vp886GetProfileArg(
    VpDevCtxType *pDevCtx,
    VpProfileType profType,
    VpProfilePtrType *ppProfileArg);

VpStatusType
Vp886SetProfTable(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType pProfile,
    VpProfilePtrType pProfileIndex,
    VpProfileType profType);

bool
Vp886ReadyStatus(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    VpStatusType *pStatus);

bool
Vp886IsReady(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx);

bool
Vp886IsValidLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);

Vp886LineStateInfoType
Vp886LineStateInfo(
    VpLineStateType state);

void
Vp886ConvertSignedFixed2Csd(
    int32 fixed,
    uint8 *csdBuf);

void
Vp886EnterCritical(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    char* funcName);

void
Vp886ExitCritical(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    char* funcName);

void
Vp886RetrieveRawSadcData(
    VpLineCtxType *pLineCtx,
    int16 *pBuffer,
    uint8 *pLength,
    uint8 *pOverflow,
    bool applyCal);

void
Vp886RetrieveRawVadcData(
    VpLineCtxType *pLineCtx,
    int16 *pBuffer,
    uint8 *pLength,
    uint8 *pOverflow,
    bool applyCal);

#ifdef VP886_INCLUDE_TESTLINE_CODE
void
Vp886TestPrepareCleanup(
    VpLineCtxType *pLineCtx);

void
Vp886TestConcludeCleanup(
    VpLineCtxType *pLineCtx);
#endif
/* End vp_886_common.c */


/* In vp886_control.c */
VpStatusType
Vp886SetLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);

VpStatusType
Vp886SetLineStateFxs(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);

VpStatusType
Vp886SetLineStateFxsInt(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);

#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
void
Vp886HighGainSetRFilter(
    VpLineCtxType *pLineCtx);
#endif

void
Vp886ApplyPcmTxRx(
    VpLineCtxType *pLineCtx);

Vp886AbsPowerReqType
Vp886GetABSPowerReq(
    VpLineCtxType *pLineCtx,
    VpLineStateType lineState);

void
Vp886ManageABSPower(
    VpDevCtxType *pDevCtx);

void
Vp886SetABSRingingBattFlag(
    VpLineCtxType *pLineCtx,
    bool ringExit);

void
Vp886ManageABSRingingBatt(
    VpDevCtxType *pDevCtx,
    bool programCalRegistersCh0,
    bool programCalRegistersCh1);

void
Vp886RingSyncFinish(
    VpLineCtxType *pLineCtx);

void
Vp886RingSyncCancel(
    VpLineCtxType *pLineCtx);

#ifdef CSLAC_GAIN_RELATIVE
VpStatusType
Vp886SetRelGain(
    VpLineCtxType *pLineCtx,
    uint16 txLevel,
    uint16 rxLevel,
    uint16 handle);
#endif

VpStatusType
Vp886SetLineTone(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pToneProfile,
    VpProfilePtrType pCadProfile,
    VpDtmfToneGenType *pDtmfControl);

VpStatusType
Vp886ApplyToneProfile(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pToneProfile);

VpStatusType
Vp886ProgramDTMFDigit(
    VpLineCtxType *pLineCtx,
    VpDigitType digit,
    uint16 amplitude);

void
Vp886SetToneCtrl(
    VpLineCtxType *pLineCtx,
    bool bias,
    bool sigA,
    bool sigB,
    bool sigC,
    bool sigD);

VpStatusType
Vp886SetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *pValue);

VpStatusType
Vp886SetOptionEventMask(
    VpLineCtxType *pLineCtx,
    VpOptionEventMaskType *pEventMask);

VpStatusType
Vp886SetRelayState(
    VpLineCtxType *pLineCtx,
    VpRelayControlType rState);

VpStatusType
Vp886SetRelayStateInt(
    VpLineCtxType *pLineCtx,
    VpRelayControlType rState);

VpStatusType
Vp886DeviceIoAccess(
    VpDevCtxType *pDevCtx,
    VpDeviceIoAccessDataType *pDeviceIoData);

VpStatusType
Vp886LineIoAccess(
    VpLineCtxType *pLineCtx,
    VpLineIoAccessType *pLineIoAccess,
    uint16 handle);

VpStatusType
Vp886DtmfDigitDetected(
    VpLineCtxType *pLineCtx,
    VpDigitType digit,
    VpDigitSenseType sense);

#if (VP886_USER_TIMERS > 0)
VpStatusType
Vp886GenTimerCtrl(
    VpLineCtxType *pLineCtx,
    VpGenTimerCtrlType timerCtrl,
    uint32 duration,
    uint16 handle);
#endif

VpStatusType
Vp886ShutdownDevice(
    VpDevCtxType *pDevCtx);

VpStatusType
Vp886ShutdownDeviceInt(
    VpDevCtxType *pDevCtx);
/* End vp886_control.c */


/* In vp886_events.c */
VpStatusType
Vp886ApiTick(
    VpDevCtxType *pDevCtx,
    bool *pEventStatus);

#ifndef VP886_SIMPLE_POLLED_MODE
VpStatusType
Vp886VirtualISR(
    VpDevCtxType *pDevCtx);
#endif

bool
Vp886GetEvent(
    VpDevCtxType *pDevCtx,
    VpEventType *pEvent);

void
Vp886ProcessHookEvent(
    VpLineCtxType *pLineCtx,
    VpCslacLineCondType newHookSt,
    uint16 timestamp);

VpStatusType
Vp886FlushEvents(
    VpDevCtxType *pDevCtx);

VpStatusType
Vp886GetResults(
    VpEventType *pEvent,
    void *pResults);

VpStatusType
Vp886GetResultsRdOptionLine(
    VpLineCtxType *pLineCtx,
    VpOptionIdType option,
    void *pResults);

VpStatusType
Vp886GetResultsRdOptionDev(
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *pResults);

VpStatusType
Vp886GetResultsQuery(
    VpLineCtxType *pLineCtx,
    VpQueryIdType queryId,
    VpQueryResultsType *pResults);

void
Vp886InitEventQueue(
    VpDevCtxType *pDevCtx);

bool
Vp886PushEvent(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    VpEventCategoryType eventCategory,
    uint16 eventId,
    uint16 eventData,
    uint16 parmHandle,
    bool hasResults);

bool
Vp886PopEvent(
    VpDevCtxType *pDevCtx,
    VpEventType *pEvent);

void
Vp886SetDetectMask(
    VpLineCtxType *pLineCtx,
    uint16 setMask);

void
Vp886ClearDetectMask(
    VpLineCtxType *pLineCtx,
    uint16 clearMask);

void
Vp886GndFltProtHandler(
    VpLineCtxType *pLineCtx,
    Vp886GroundFaultProtInputType input);

void
Vp886GndFltProtStop(
    VpLineCtxType *pLineCtx,
    bool fixLineState);

#ifdef VP886_INCLUDE_DTMF_DETECT
void
Vp886DtmfManage(
    VpLineCtxType *pLineCtx);
#endif
/* End vp886_events.c */


/* In vp886_init.c */
/* Most functions in this file are static because this is the file where
   the function pointer table is set up */
void
Vp886InitDeviceSM(
    VpDevCtxType *pDevCtx);

void
Vp886FreeRunStart(
    VpDevCtxType *pDevCtx);

void
Vp886FreeRunStop(
    VpDevCtxType *pDevCtx);
/* End vp886_init.c */

/* In vp886_ring_power_adapt.c */
VpStatusType
Vp886AdaptiveRingingInit(
    VpLineCtxType *pLineCtx);

VpStatusType
Vp886AdaptiveRingingPrepare(
    VpLineCtxType *pLineCtx);

VpStatusType
Vp886AdaptiveRingingStop(
    VpLineCtxType *pLineCtx);

void
Vp886AdaptiveRingingHandler(
    VpLineCtxType *pLineCtx);

void
Vp886AdaptiveRingingSetTargetSlicPower(
    VpLineCtxType *pLineCtx,
    int16 targetSlicPower);
/* End vp886_ring_power_adapt.c */

/* In vp886_query.c */
VpStatusType
Vp886GetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    uint16 handle);

VpStatusType
Vp886GetOptionImmediate(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *pResults);

VpStatusType
Vp886GetLoopCond(
    VpLineCtxType *pLineCtx,
    uint16 handle);

VpStatusType
Vp886GetLoopCondResults(
    VpLineCtxType *pLineCtx);

VpStatusType
Vp886GetDeviceStatus(
    VpDevCtxType *pDevCtx,
    VpInputType input,
    uint32 *pDeviceStatus);

VpStatusType
Vp886Query(
    VpLineCtxType *pLineCtx,
    VpQueryIdType queryId,
    uint16 handle);

VpStatusType
Vp886QueryImmediate(
    VpLineCtxType *pLineCtx,
    VpQueryIdType queryId,
    void *pResults);

#ifdef VP886_INCLUDE_TESTLINE_CODE
EXTERN VpStatusType
Vp886GetRelayState(
    VpLineCtxType *pLineCtx,
    VpRelayControlType *pRstate);

/* Testing functions */
VpStatusType
Vp886TestLine(
    VpLineCtxType *pLineCtx,
    VpTestIdType testId,
    const void *pArgsUntyped,
    uint16 handle);

VpStatusType
Vp886TestLineInt(
    VpLineCtxType *pLineCtx,
    const void *pArgsUntyped,
    bool callback);

#endif

#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
VpStatusType
Vp886RegisterDump(
    VpDevCtxType *pDevCtx);

VpStatusType
Vp886RegisterDumpInt(
    VpDevCtxType *pDevCtx);

VpStatusType
Vp886ObjectDump(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx);

VpStatusType
Vp886ObjectDumpInt(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx);
#endif
/* End vp886_query.c */


/* In vp886_seq.c */
#ifdef VP_CSLAC_SEQ_EN

VpStatusType
Vp886CadenceStart(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pProfile,
    VpCadenceStatusType flags);

void
Vp886CadenceStop(
    VpLineCtxType *pLineCtx,
    bool aborted,
    bool restoreLineState,
    bool disableTones);

VpStatusType
Vp886CadenceHandler(
    VpLineCtxType *pLineCtx,
    uint32 timerOverrun);

VpStatusType
Vp886InitRing(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pCadProfile,
    VpProfilePtrType pCidProfile);

VpStatusType
Vp886InitCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    uint8p pCidData);

VpStatusType
Vp886SendCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    VpProfilePtrType pCidProfile,
    uint8p pCidData);

VpStatusType
Vp886ContinueCid(
    VpLineCtxType *pLineCtx,
    uint8 length,
    uint8p pCidData);

void
Vp886CidStart(
    VpLineCtxType *pLineCtx);

void
Vp886CidStop(
    VpLineCtxType *pLineCtx);

void
Vp886CidHandler(
    VpLineCtxType *pLineCtx,
    uint32 timerOverrun);

void
Vp886CidAckDetect(
    VpLineCtxType *pLineCtx,
    VpDigitType digit,
    VpDigitSenseType sense);

VpStatusType
Vp886SendSignal(
    VpLineCtxType *pLineCtx,
    VpSendSignalType type,
    void *pStruct);

void
Vp886SendSignalHandler(
    VpLineCtxType *pLineCtx,
    uint32 overrun);

void
Vp886SendSignalStop(
    VpLineCtxType *pLineCtx,
    bool restoreLineState);

VpStatusType
Vp886InitMeter(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pMeterProfile);

VpStatusType
Vp886StartMeter(
    VpLineCtxType *pLineCtx,
    uint16 onTime,
    uint16 offTime,
    uint16 numMeters);

void
Vp886MeterHandler(
    VpLineCtxType *pLineCtx,
    uint32 overrun);

void
Vp886MeterStop(
    VpLineCtxType *pLineCtx,
    bool restoreLineState);

bool
Vp886HowlerToneInit(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pCadProfile,
    VpStatusType *pStatus);

void
Vp886HowlerToneHandler(
    VpLineCtxType *pLineCtx,
    uint32 overrun);
#endif /* VP_CSLAC_SEQ_EN */
/* End vp886_seq.c */


/* In vp886_slac.c */
bool
Vp886SlacBufStart(
    VpDevCtxType *pDevCtx);

bool
Vp886SlacBufSend(
    VpDevCtxType *pDevCtx);

bool
Vp886SlacRegWrite(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint8 cmd,
    uint8 dataLen,
    const uint8 *pDataBuf);

bool
Vp886SlacRegRead(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint8 cmd,
    uint8 dataLen,
    uint8 *pDataBuf);

void
Vp886SlacBufFlush(
    VpDevCtxType *pDevCtx);
/* End vp886_slac.c */


/* In vp886_timers.c */
bool
Vp886InitTimerQueue(
    VpDevCtxType *pDevCtx);

void
Vp886ProcessTimers(
    VpDevCtxType *pDevCtx);

bool
Vp886AddTimerHalfMs(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint16 timerId,
    uint32 duration,
    uint32 overrun,
    uint32 handle);

bool
Vp886AddTimerMs(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint16 timerId,
    uint32 duration,
    uint32 overrun,
    uint32 handle);

bool
Vp886RestartTimerHalfMs(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint16 timerId,
    uint32 duration,
    uint32 handle);

bool
Vp886RestartTimerMs(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint16 timerId,
    uint32 duration,
    uint32 handle);

bool
Vp886ExtendTimerMs(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint16 timerId,
    uint32 duration,
    uint32 handle);

bool
Vp886CancelTimer(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint16 timerId,
    uint32 handle,
    bool matchHandle);

uint16
Vp886GetTimestamp(
    VpDevCtxType *pDevCtx);

uint32
Vp886GetTimestamp32(
    VpDevCtxType *pDevCtx);

void
Vp886SetDeviceTimer(
    VpDevCtxType *pDevCtx,
    bool enable,
    uint32 duration);

void
Vp886ForceTimerInterrupt(
    VpDevCtxType *pDevCtx);

void
Vp886ManageSamplingTimer(
    VpDevCtxType *pDevCtx);

void
Vp886TimerHandler(
    VpDevCtxType *pDevCtx,
    uint16 timerId,
    uint32 overrun,
    uint8 channelId,
    uint32 handle);
/* End vp886_timers.c */

#endif  /* Vp886_API_INT_H */

