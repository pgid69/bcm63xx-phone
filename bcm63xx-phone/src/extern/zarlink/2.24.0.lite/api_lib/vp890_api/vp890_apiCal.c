/** \file vp890_apiCal.c
 * vp890_apiCal.c
 *
 * This file contains the line and device calibration functions for
 * the Vp890 device API.
 *
 * Copyright (c) 2012, Microsemi Corporation
 *
 * $Revision: 10556 $
 * $LastChangedDate: 2012-10-26 17:33:22 -0500 (Fri, 26 Oct 2012) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_890_SERIES)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_api.h"
#include "vp_api_int.h"
#include "vp890_api.h"
#include "vp890_api_int.h"
#include "vp_hal.h"
#include "sys_service.h"


/* Functions that are called only inside this file. */
#if defined (VP890_FXS_SUPPORT) && defined (VP_CSLAC_RUNTIME_CAL_ENABLED)
#define VP890_IMT_SETTLE_MS 100
#define VP890_DCFEED_SETTLE_MS 100
#define VP890_IMT_10MA 5679L

/*  these are in order of execution ALWAYS do not reorder them !!!! */
typedef enum Vp890CalState {
    VP890_CAL_SETUP                 =   0,
    VP890_CAL_SETUP_RELEASE_CLAMPS,
    VP890_RAMP_TO_POLREV_SETUP,
    VP890_RAMP_TO_POLREV_RAMP1,
    VP890_RAMP_TO_POLREV_SWAP_POLARITY,
    VP890_RAMP_TO_POLREV_RAMP2,
    VP890_RAMP_TO_POLREV_GOACTIVE,
    VP890_RAMP_TO_POLREV_COMPLETE,

    VP890_ABV_SET_ADC               =   10,
    VP890_ABV_MEASURE,
    VP890_ABV_OFFSET_SETUP,
    VP890_ABV_OFFSET_SETUP2,
    VP890_ABV_OFFSET_SET_ADC,
    VP890_ABV_OFFSET_MEASURE,
    VP890_ABV_ADJUST,

    VP890_IMT_PR_SETUP              =   20,
    VP890_IMT_PR_SET_ADC,
    VP890_IMT_PR_MEASURE,

    VP890_VAS_PR_SETUP              =   30,
    VP890_VAS_PR_STEP,
    VP890_VAS_PR_SET_ADC,
    VP890_VAS_PR_MEASURE,
    VP890_VAS_PR_STORE,

    VP890_VAB_PR_SETUP              =   40,
    VP890_VAB_PR_MEASURE,

    VP890_VAB_PR_ADC_OFFSET_SETUP   =   50,
    VP890_VAB_PR_ADC_OFFSET_MEASURE,

    VP890_VA_PR_ADC_OFFSET_SETUP    =   60,
    VP890_VA_PR_ADC_OFFSET_MEASURE,

    VP890_VB_PR_ADC_OFFSET_SETUP    =   70,
    VP890_VB_PR_ADC_OFFSET_MEASURE,

    VP890_COLLAPSE_FEED             =   80,

    VP890_GENA_NP_OFFSET_SETUP      =   90,
    VP890_GENA_NP_OFFSET_SET_ADC,
    VP890_GENA_NP_OFFSET_MEASURE,
    VP890_GENA_NP_OFFSET_RESTORE,

    VP890_ILG_OFFSET_SETUP          =   100,
    VP890_ILG_OFFSET_MEASURE,

    VP890_ILA_OFFSET_SETUP          =   110,
    VP890_ILA_OFFSET_MEASURE,


    VP890_RESTORE_DAC               =   120,

    VP890_VAB_NP_ADC_OFFSET_SETUP   =   130,
    VP890_VAB_NP_ADC_OFFSET_MEASURE,

    VP890_VA_NP_ADC_OFFSET_SETUP    =   140,
    VP890_VA_NP_ADC_OFFSET_MEASURE,

    VP890_VB_NP_ADC_OFFSET_SETUP    =   150,
    VP890_VB_NP_ADC_OFFSET_MEASURE,

    VP890_ILA_SETUP                 =   160,
    VP890_ILA_ADJUST,
    VP890_ILA_SET_ADC,
    VP890_ILA_MEASURE,

    VP890_RESTORE_FEED              =   170,

    VP890_IMT_NP_SETUP              =   180,
    VP890_IMT_NP_SET_ADC,
    VP890_IMT_NP_MEASURE,

    VP890_VAS_NP_SETUP              =   190,
    VP890_VAS_NP_STEP,
    VP890_VAS_NP_SET_ADC,
    VP890_VAS_NP_MEASURE,
    VP890_VAS_NP_STORE,
    VP890_VAS_PR_ADJUST,

    VP890_VAB_NP_SETUP              =   200,
    VP890_VAB_NP_MEASURE,

    VP890_CAL_ADJUST                =   210,

    VP890_CAL_RESTORE               =   254,
    VP890_CAL_ENUM_SIZE             =   FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} Vp890CalState;

static bool
CalLineNextState(
    VpDevCtxType            *pDevCtx,
    VpLineCtxType           *pLineCtx,
    Vp890DeviceObjectType   *pDevObj,
    Vp890LineObjectType     *pLineObj,
    uint16                  *pCalTimerMs);

static void
ComputeFinalVas(
    VpLineCtxType *pLineCtx,
    uint16 *vasValue);

static void
SetAdc(
    VpLineCtxType *pLineCtx,
    VpDeviceIdType deviceId,
    uint8 ecVal,
    uint8 adcRoute);

static bool
GetXData(
    VpLineCtxType *pLineCtx,
    VpDeviceIdType deviceId,
    uint8 ecVal,
    int16 *pData);


#endif /* (VP890_FXS_SUPPORT) && defined (VP_CSLAC_RUNTIME_CAL_ENABLED) */

#ifdef VP890_FXO_SUPPORT
static VpStatusType
Vp890CalBFilter(
    VpLineCtxType       *pLineCtx,
    void                *inputArgs);

static VpStatusType
Vp890CalApplyBFilter(
    VpLineCtxType       *pLineCtx,
    void                *inputArgs);

static VpStatusType
Vp890CalMeasureBFilter(
    VpLineCtxType       *pLineCtx);

#ifdef VP890_REDUCE_BFILTER_CAL_SIGNAL_LEVEL
static void
Vp890ReduceNoiseOutput(
    uint8 oldVpGain,
    VpDeviceIdType deviceId,
    uint8 ecVal);
#endif /* VP890_REDUCE_BFILTER_CAL_SIGNAL_LEVEL */


#endif /* VP890_FXO_SUPPORT */

#if defined (VP890_FXS_SUPPORT) && defined (VP_CSLAC_RUNTIME_CAL_ENABLED)
/**
 * Vp890CalLineInt()
 *   This function is called each time the VP_LINE_CAL_LINE_TIMER expires. Its basic purpose is to
 *   continue calling CalLineNextState until it returns false.
 *
 * Postconditions:
 *   This function generates an event upon completing the requested action or sets up the cal timer
 *   to expire at a later time to run the next state.
 */
VpStatusType
Vp890CalLineInt(
    VpLineCtxType *pLineCtx)
{

    Vp890LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 tickRate = pDevObj->devProfileData.tickRate;
    uint16 calTimerMs = 0;
    pDevObj->responseData = VP_CAL_SUCCESS;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890CalLineInt()"));

    /* Continue calling the cal state machine until it indicates FALSE */
    while (CalLineNextState(pDevCtx, pLineCtx, pDevObj, pLineObj, &calTimerMs));

    if (calTimerMs != 0) {
        /* Setup the Cal timer to expire in requested ms */
        pLineObj->lineTimers.timers.timer[VP_LINE_CAL_LINE_TIMER] =
            (MS_TO_TICKRATE(calTimerMs, tickRate) | VP_ACTIVATE_TIMER);

    } else {    /* calTimerMs = 0 */
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Generating VP_EVID_CAL_CMP Event for Channel %d EventMask 0x%04X",
            pLineObj->channelId, pLineObj->lineEventsMask.response));

        /* If no timer was requested, Cal must be done so generate and event */
        pLineObj->lineState.calType = VP_CSLAC_CAL_NONE;
        pLineObj->lineEvents.response |= VP_EVID_CAL_CMP;
        pLineObj->status &= ~VP890_LINE_IN_CAL;

        /*
         * Set calDone = TRUE only if there wasn't a failure detected. If the calDone flag is TRUE,
         * the line can't be recalibrated without the hidden setting with APPLY_SYSTEM_CEOFF and
         * NULL profile.
         */
        if (pDevObj->responseData == VP_CAL_FAILURE) {
            /*
             * This should be redundant because calibration won't run unless calDone was set to
             * FALSE to start with (and VP890_SYS_CAL_COMPLETE was clear). But make sure anyway.
             */
            pLineObj->calLineData.calDone = FALSE;
            pDevObj->stateInt &= ~VP890_SYS_CAL_COMPLETE;
        } else {
            /* Success. Indicate that this line calibration is done. */
            pLineObj->calLineData.calDone = TRUE;
            /*
             * Set at device level here because only the FXS line requires calibration. Therefore,
             * if the FXS line is calibrated, the "system" is calibrated.
             */
            pDevObj->stateInt |= VP890_SYS_CAL_COMPLETE;
        }
        pDevObj->state &= ~VP_DEV_IN_CAL;
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp890CalLineInt()"));

    return VP_STATUS_SUCCESS;
}   /* Vp890CalLineInt() */

/**
 * CalLineNextState()
 * This function implements the steps of the 890 Calibration algorithm.
 *
 * The state machine will move to the next state when the Cal timer
 * expires or if the function returns TRUE.
 *
 * Postconditions:
 *  Returns true indicating that another state should be run immediately.
 *  Returns false indicating that the state machine is done with the current
 *  state.
 */
static bool
CalLineNextState(
    VpDevCtxType            *pDevCtx,
    VpLineCtxType           *pLineCtx,
    Vp890DeviceObjectType   *pDevObj,
    Vp890LineObjectType     *pLineObj,
    uint16                  *pCalTimerMs)
{

    uint8 channelId = pLineObj->channelId;
    uint8 ecVal = pLineObj->ecVal;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp890CalState nextState = VP890_CAL_ENUM_SIZE;
    bool runAnotherState = FALSE;
    uint8 mpiIndex = 0;
    uint8 mpiBuffer[254];

    *pCalTimerMs = 0;

    switch(pLineObj->calLineData.calState) {

        /***********************************************************************
         * This state is used to setup the device for the cal sequence
         * by clearing out values in registers.
         **********************************************************************/
        case VP890_CAL_SETUP: {
            uint8 swYZ[VP890_REGULATOR_PARAM_LEN];
            uint8 swCal[VP890_BAT_CALIBRATION_LEN];
            uint8 disnVal = 0;
            uint8 vpGain = 0;
            uint8 codec = VP890_LINEAR_CODEC;
            uint8 loopSup[VP890_LOOP_SUP_LEN];

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_CAL_SETUP"));
            /*
             * Start off with the assumption calibration will work. Simpler to
             * let the algorithm detect failure and set this accordingly rather
             * than have each possible good exit set it for pass.
             */
            pDevObj->responseData = VP_CAL_SUCCESS;

            /* Store LoopSup, DISN , Voice Path Gain, ICR2, SLIC state, and genA */
            VpMemCpy(pLineObj->calLineData.loopSup, pLineObj->loopSup,
                VP890_LOOP_SUP_LEN);

            VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_RD,
                VP890_SS_CONFIG_LEN, &pLineObj->calLineData.asscReg);

            VpMemCpy(loopSup, pLineObj->calLineData.loopSup, VP890_LOOP_SUP_LEN);
            VpMpiCmdWrapper(deviceId, ecVal, VP890_DISN_RD,
                VP890_DISN_LEN, pLineObj->calLineData.disnVal);
            VpMpiCmdWrapper(deviceId, ecVal, VP890_VP_GAIN_RD,
                VP890_VP_GAIN_LEN,pLineObj->calLineData.vpGain);

            VpMemCpy(pLineObj->calLineData.icr2, pLineObj->icr2Values, VP890_ICR2_LEN);
            VpMemCpy(pLineObj->calLineData.icr3, pLineObj->icr3Values, VP890_ICR3_LEN);

            VpMpiCmdWrapper(deviceId, ecVal, VP890_SIGA_PARAMS_RD,
                VP890_SIGA_PARAMS_LEN, pLineObj->calLineData.sigGenA);
            VpMpiCmdWrapper(deviceId, ecVal, VP890_OP_FUNC_RD,
                VP890_OP_FUNC_LEN, &pLineObj->calLineData.codecReg);

            /*
             * Copy the Ringing Voltage to the Floor Voltage, everything else
             * directly from the device profile
             */
            swYZ[0] = pDevObj->devProfileData.swParams[0];
            swYZ[0] &= ~0x02; /* remove fixed ringing for cal */

            swYZ[VP890_SWY_LOCATION] =
                (pDevObj->devProfileData.swParams[VP890_SWZ_LOCATION]
                & VP890_VOLTAGE_MASK);
            swYZ[VP890_SWY_LOCATION] |=
                (pDevObj->devProfileData.swParams[VP890_SWY_LOCATION]
                & ~VP890_VOLTAGE_MASK);

            swYZ[2] = pDevObj->devProfileData.swParams[2];

            /* Set Hook Debounce to 8 ms */
            loopSup[VP890_LOOP_SUP_DEBOUNCE_BYTE] &= ~VP890_SWHOOK_DEBOUNCE_MASK;
            loopSup[VP890_LOOP_SUP_DEBOUNCE_BYTE] |= 0x04;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_LOOP_SUP_WRT, VP890_LOOP_SUP_LEN, loopSup);

            /* Set Floor Voltage to Target Ringing Voltage */
            VpMemCpy(pDevObj->swParamsCache, swYZ, VP890_REGULATOR_PARAM_LEN);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_REGULATOR_PARAM_WRT, VP890_REGULATOR_PARAM_LEN, swYZ);

            /* Clear existing battery calibration correction factors */
            VpMpiCmdWrapper(deviceId, ecVal, VP890_BAT_CALIBRATION_RD,
                VP890_BAT_CALIBRATION_LEN, swCal);
            swCal[0] &= ~(VP890_BAT_CAL_SWCAL_MASK);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_BAT_CALIBRATION_WRT, VP890_BAT_CALIBRATION_LEN, swCal);

            /* Set DISN = 0 and Voice Path Gain = 0dB TX */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DISN_WRT, VP890_DISN_LEN, &disnVal);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_VP_GAIN_WRT,VP890_VP_GAIN_LEN, &vpGain);

            /* Set the device mode register for single buffer mode */
            pDevObj->devMode[0] &= ~(VP890_DEV_MODE_TEST_DATA);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DEV_MODE_WRT, VP890_DEV_MODE_LEN, pDevObj->devMode);

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("\n\rVP890_CAL_SETUP: Writing 0x%02X to Operating Functions", codec));

            /* Set for Linear Mode and disable AC Coefficients */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_OP_FUNC_WRT, VP890_OP_FUNC_LEN, &codec);

            /* Cut TX/RX PCM and disable HPF */
            pLineObj->opCond[0] = (VP890_CUT_TXPATH | VP890_CUT_RXPATH | VP890_HIGH_PASS_DIS);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_OP_COND_WRT, VP890_OP_COND_LEN, pLineObj->opCond);

            /* Copy the reference dcfeed data into the polrev dcfeed array */
            VpMemCpy(pLineObj->calLineData.dcFeedPr,
                pLineObj->calLineData.dcFeedRef, VP890_DC_FEED_LEN);

            /* set VAS to 3.0V (min) and ila to 40mA */
            pLineObj->calLineData.dcFeedPr[0] &= 0xFC;
            pLineObj->calLineData.dcFeedPr[1] &= 0x3F;
            pLineObj->calLineData.dcFeedPr[1] &= ~VP890_ILA_MASK;
            pLineObj->calLineData.dcFeedPr[1] |= (VP890_ILA_MASK & 0x16);

            /* Set DC-Feed to values from dc profile */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DC_FEED_WRT, VP890_DC_FEED_LEN, pLineObj->calLineData.dcFeedPr);

            /* next state info */
            *pCalTimerMs = 10;
            nextState = VP890_CAL_SETUP_RELEASE_CLAMPS;

            break;
        }

        case VP890_CAL_SETUP_RELEASE_CLAMPS:
            if ((pDevObj->stateInt & VP890_IS_HIGH_VOLTAGE) &&
                !pDevObj->devProfileData.lowVoltOverride) {

                /*
                 * if this is a high voltage device and designed
                 * with a high voltage supply then we need to set
                 * the voltage clamps to 150V to give the slower
                 * battery designs time to ramp up.
                 */
                pLineObj->icr2Values[2] |= 0x0C;
                pLineObj->icr2Values[3] |= 0x0C;
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                    VP890_ICR2_WRT, VP890_ICR2_LEN, pLineObj->icr2Values);
            }

            /* if we are not already in polrev then we need to go there */
            if (!pLineObj->calLineData.reversePol) {
                nextState = VP890_RAMP_TO_POLREV_SETUP;
                *pCalTimerMs = 10;
            } else {
                nextState = VP890_ABV_SET_ADC;
                /*
                 * mimic the time required by ramp pol rev to give
                 * battery from the slower power supplies time to
                 * finish moving.
                 */
                *pCalTimerMs = 100;
            }
            break;

        /***********************************************************************
         * These states are used to force the line into polrev using a ramp
         *
         * Need to ramp to negative polarity so that we dont ding a phone. Due
         * to a device issue, we have to split the ramp in two. The first
         * ramp will start at -45V and ramp to 0V. Once 0V is acheived, the deivce
         * the polarity of the line will be flipped. Then the ramp will
         * continue to +45V. Finally the line will be taken out of ringing
         * and put into active. Performing the polrev at 0V prevents a tip/ring
         * transient at the end when we go from balanced ringing to active polrev.
         **********************************************************************/
        case VP890_RAMP_TO_POLREV_SETUP: {
            uint8 asscReg = (pLineObj->calLineData.asscReg | VP890_ZXR_MASK);

            /* setup siggen to mimic a 45V VOC once ringing is enabled */
            uint8 sigGenA[VP890_SIGA_PARAMS_LEN] =
                {0x00, 0x25, 0x4E, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00 ,0x00, 0x00};

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_RAMP_TO_POLREV_SETUP"));

            /* disable auto ring entery / exit */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_SS_CONFIG_WRT, VP890_SS_CONFIG_LEN, &asscReg);

            /* Prepare the sig gen with ramp info */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_SIGA_PARAMS_WRT, VP890_SIGA_PARAMS_LEN, sigGenA);

            /*
             * go to the balanced ringing state. This should generate a
             * longitudinal shift. The direction and size will depend on
             * the value of VOC in the DC feed register.
             */
            pLineObj->slicValueCache &= ~VP890_SS_STATE_MASK;
            pLineObj->slicValueCache |= VP890_SS_BALANCED_RINGING;

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_SYS_STATE_WRT, VP890_SYS_STATE_LEN, &pLineObj->slicValueCache);

            /* enable the half battery long clamp */
            pLineObj->icr3Values[2] |= 0x40;
            pLineObj->icr3Values[3] = (pLineObj->icr3Values[3] & ~0x40);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_ICR3_WRT, VP890_ICR3_LEN, pLineObj->icr3Values);

            /* next state info */
            *pCalTimerMs = 10;
            nextState = VP890_RAMP_TO_POLREV_RAMP1;
            break;
        }

        case VP890_RAMP_TO_POLREV_RAMP1: {
            /* force the siggen to start a negative ramp of 45V */
            uint8 sigGenA[VP890_SIGA_PARAMS_LEN] =
                {0x07, 0x25, 0x4E, 0x00, 0x25, 0x25, 0x4E, 0x00, 0x00 ,0x00, 0x00};

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_RAMP_TO_POLREV_RAMP1"));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_SIGA_PARAMS_WRT, VP890_SIGA_PARAMS_LEN, sigGenA);
            *pCalTimerMs = 100;
            nextState = VP890_RAMP_TO_POLREV_SWAP_POLARITY;
            break;
        }

        case VP890_RAMP_TO_POLREV_SWAP_POLARITY: {
            /* go to balanced ringing polrev */
            pLineObj->slicValueCache &= ~VP890_SS_STATE_MASK;
            pLineObj->slicValueCache |= VP890_SS_BALANCED_RINGING_PR;

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_SYS_STATE_WRT, VP890_SYS_STATE_LEN, &pLineObj->slicValueCache);

            runAnotherState = TRUE;
            nextState = VP890_RAMP_TO_POLREV_RAMP2;
            break;
        }

        case VP890_RAMP_TO_POLREV_RAMP2: {
            /* force the siggen to complete the ramp */
            uint8 sigGenA[VP890_SIGA_PARAMS_LEN] =
                {0x03, 0x25, 0x4E, 0x00, 0x25, 0x25, 0x4E, 0x00, 0x00 ,0x00, 0x00};

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_RAMP_TO_POLREV_RAMP2"));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_SIGA_PARAMS_WRT, VP890_SIGA_PARAMS_LEN, sigGenA);
            *pCalTimerMs = 50;
            nextState = VP890_RAMP_TO_POLREV_GOACTIVE;
            break;
        }

        case VP890_RAMP_TO_POLREV_GOACTIVE: {
            uint8 sigGenA[VP890_SIGA_PARAMS_LEN] =
                {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00 ,0x00, 0x00};

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_RAMP_TO_POLREV_GOACTIVE"));

            /* go to active polrev */
            pLineObj->slicValueCache &= ~VP890_SS_STATE_MASK;
            pLineObj->slicValueCache |= VP890_SS_ACTIVE_POLREV;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_SYS_STATE_WRT, VP890_SYS_STATE_LEN, &pLineObj->slicValueCache);

            /* remove the siggen ramp */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_SIGA_PARAMS_WRT, VP890_SIGA_PARAMS_LEN, sigGenA);

            *pCalTimerMs = 10;
            nextState = VP890_RAMP_TO_POLREV_COMPLETE;
            break;
        }

        case VP890_RAMP_TO_POLREV_COMPLETE: {
            /* restore the automatic system state control */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_SS_CONFIG_WRT, VP890_SS_CONFIG_LEN, &pLineObj->calLineData.asscReg);

            *pCalTimerMs = 20;
            nextState = VP890_ABV_SET_ADC;
        }

        /***********************************************************************
         * Measure Switching Regulator Y and Y offset -
         *
         * These states setup, measure and adjust ABV.
         **********************************************************************/

        case VP890_ABV_SET_ADC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_ABV_SET_ADC"));

            SetAdc(pLineCtx, deviceId, ecVal, VP890_SWITCHER_Y);
            *pCalTimerMs = 10;
            nextState = VP890_ABV_MEASURE;
            break;
        }

        case VP890_ABV_MEASURE: {
            int16 *pSwyVolt = &pLineObj->calLineData.typeData.swyVolt;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_ABV_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, pSwyVolt)) {
                *pCalTimerMs = 10;
                nextState = VP890_ABV_MEASURE;
            } else {
                runAnotherState = TRUE;
                nextState = VP890_ABV_OFFSET_SETUP;
            }
            break;
        }

        case VP890_ABV_OFFSET_SETUP: {
            uint8 swyTime[VP890_REGULATOR_TIMING_LEN] = {
                0x00, 0x40, 0x00, 0x40, 0x00, 0x40
            };

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_ABV_OFFSET_SETUP"));

            /* Disable switcher by setting duty cycle = 0. */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_REGULATOR_TIMING_WRT, VP890_REGULATOR_TIMING_LEN, swyTime);

            *pCalTimerMs = 10;
            nextState = VP890_ABV_OFFSET_SETUP2;
            break;
        }

        case VP890_ABV_OFFSET_SETUP2: {
            /* Disconnect SWVS pin and ring sense */
            pLineObj->dcCalValues[0] = 0x00;
            pLineObj->dcCalValues[1] = (VP890_C_YBAT_SNS_CUT | VP890_C_TIP_SNS_CUT);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DC_CAL_REG_WRT, VP890_DC_CAL_REG_LEN, pLineObj->dcCalValues);

            *pCalTimerMs = 10;
            nextState = VP890_ABV_OFFSET_SET_ADC;
            break;
        }

        case VP890_ABV_OFFSET_SET_ADC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_ABV_OFFSET_SET_ADC"));

            SetAdc(pLineCtx, deviceId, ecVal, VP890_SWITCHER_Y);
            *pCalTimerMs = 10;
            nextState = VP890_ABV_OFFSET_MEASURE;
            break;
        }

        case VP890_ABV_OFFSET_MEASURE: {
            int16 *pOffset = &pDevObj->vp890SysCalData.swyOffset[channelId];

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_ABV_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, pOffset)) {
                *pCalTimerMs = 10;
                nextState = VP890_ABV_OFFSET_MEASURE;
            } else {
                runAnotherState = TRUE;
                nextState = VP890_ABV_ADJUST;
            }
            break;
        }

        case VP890_ABV_ADJUST: {
            int16 calReq = pDevObj->devProfileData.swParams[VP890_SWREG_RING_V_BYTE];
            int32 abvTarget = 0;
            int32 abvError = 0;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_ABV_ADJUST"));

            /* Always remove the 150V clamp */
            pLineObj->icr2Values[2] |= 0x0C;
            pLineObj->icr2Values[3] = (pLineObj->icr2Values[3] & ~0x0C) | 0x08;

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_ICR2_WRT, VP890_ICR2_LEN, pLineObj->icr2Values);

            /* Reconnect Sense pins */
            pLineObj->dcCalValues[0] = 0x00;
            pLineObj->dcCalValues[1] = 0x00;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DC_CAL_REG_WRT, VP890_DC_CAL_REG_LEN, pLineObj->dcCalValues);

            /* Restore the switching regulator timing parameters */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_REGULATOR_TIMING_WRT, VP890_REGULATOR_TIMING_LEN,
                pDevObj->devProfileData.timingParams);

            /* Now have all data necessary to compute error and adjust channel 0 */
            abvTarget = (calReq * 5L) + 5;   /* Gets it to V scale */
            abvTarget *= 1000L * 1000L;
            abvTarget /= 7324;   /* Now we're scaled to the PCM data */

            abvError = abvTarget -
                (pLineObj->calLineData.typeData.swyVolt
               - pDevObj->vp890SysCalData.swyOffset[0]);

            /* This converts from PCM Scale to 10mV scale */
            pDevObj->vp890SysCalData.abvError[0] =
                (int16)((abvError * VP890_V_PCM_LSB) / VP890_V_SCALE);

            /* force a battery adjust before moving on*/
            Vp890BatteryAdjust(pLineCtx);

            /* Restore Switching Regulator Parameters except the tracking bit */
            VpMemCpy(pDevObj->swParamsCache, pDevObj->devProfileData.swParams,
                VP890_REGULATOR_PARAM_LEN);

            pDevObj->swParamsCache[0] &= ~0x02; /* remove fixed ringing */

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_REGULATOR_PARAM_WRT, VP890_REGULATOR_PARAM_LEN,
                pDevObj->swParamsCache);

            /* next state info */
            nextState = VP890_IMT_PR_SETUP;
            runAnotherState = TRUE;
            break;
        }

        /***********************************************************************
         * Settle IMT in PolRev -
         *
         * These states setup, measure Metallic polrev current until it
         * settles. This is done to prevent high REN loads from distorting
         * the VAS measurements
         **********************************************************************/
        case VP890_IMT_PR_SETUP: {

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_IMT_PR_SETUP"));

            /* Enable battery and DC Speed up */
            pLineObj->icr2Values[2] |= 0xC0;
            pLineObj->icr2Values[3] |= 0xC0;

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_ICR2_WRT, VP890_ICR2_LEN, pLineObj->icr2Values);

            /* prepare storage data */
            pLineObj->calLineData.typeData.loopData.loopNum = 0;
            pLineObj->calLineData.typeData.loopData.prevVal = 0;

            *pCalTimerMs = 10;
            nextState = VP890_IMT_PR_SET_ADC;
            break;
        }

        case VP890_IMT_PR_SET_ADC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_IMT_PR_SET_ADC"));

            SetAdc(pLineCtx, deviceId, ecVal, VP890_METALLIC_DC_I);
            *pCalTimerMs = 10;
            nextState = VP890_IMT_PR_MEASURE;
            break;
        }

        case VP890_IMT_PR_MEASURE: {
            int16 imOld = 0;
            int16 imNew = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_IMT_PR_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &imNew)) {
                *pCalTimerMs = 10;
                nextState = VP890_IMT_PR_MEASURE;
                break;
            }

            /* if this is the first imt measurement then get another */
            if (pLineObj->calLineData.typeData.loopData.loopNum++ == 0) {
                pLineObj->calLineData.typeData.loopData.prevVal = imNew;
                *pCalTimerMs = VP890_IMT_SETTLE_MS;
                nextState = VP890_IMT_PR_MEASURE;
                break;
            }
            imOld = pLineObj->calLineData.typeData.loopData.prevVal;

            /* have we settled enough or taken to long > 200ms*/
            if ( (pLineObj->calLineData.typeData.loopData.loopNum < 20 /*200ms*/) &&
                (((imNew + 15 - imOld) & 0xFFE0) != 0) ) {
                /* nope run it again */
                pLineObj->calLineData.typeData.loopData.prevVal = imNew;
                *pCalTimerMs = VP890_IMT_SETTLE_MS;
                nextState = VP890_IMT_PR_MEASURE;
                break;
            }

            /* move to the next state block */
            nextState = VP890_VAS_PR_SETUP;
            runAnotherState = TRUE;
            break;
        }

        /***********************************************************************
         * Measure VAS in PolRev -
         *
         * These five states step through VAS values and measure IMT.
         * This will continue until:
         *  VAS >= 10.25V ||
         *  new IMT < 10mA && (new IMT - old IMT are with in +9 to -7mA)
         *
         * We only loop to 10.25V because we will always add 3.75V to VAS
         * and the max value of the VAS register is 14.25V.
         **********************************************************************/

        case VP890_VAS_PR_SETUP:

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAS_PR_SETUP"));

            /* prepare storage data */
            pLineObj->calLineData.typeData.loopData.loopNum = 0;
            pLineObj->calLineData.typeData.loopData.prevVal = 0x7FFF;

            pLineObj->calLineData.vasStart = 3000;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP890_VAS_PR_SETUP: Best Actual VAS Start %d",
                pLineObj->calLineData.vasStart));

            runAnotherState = TRUE;
            nextState = VP890_VAS_PR_STEP;
            break;

        case VP890_VAS_PR_STEP: {
            uint8 loopNum = pLineObj->calLineData.typeData.loopData.loopNum;
            uint16 vasVal = pLineObj->calLineData.vasStart + (750 * loopNum);

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAS_PR_STEP"));

            /*
             * If the calculated VAS is max, advance to battery adjustment
             * steps.
             */
            if (vasVal > VP890_VAS_MAX) {
                runAnotherState = TRUE;
                nextState = VP890_VAS_PR_STORE;
                break;
            }

            /* set VAS to the next value */
            VpCSLACSetVas(pLineObj->calLineData.dcFeedPr, vasVal);
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP890_VAS_PR_STEP: Setting VAS (%d) with Values 0x%02X 0x%02X at time %d",
                vasVal,
                pLineObj->calLineData.dcFeedPr[0], pLineObj->calLineData.dcFeedPr[1],
                pDevObj->timeStamp));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DC_FEED_WRT, VP890_DC_FEED_LEN, pLineObj->calLineData.dcFeedPr);

            /* dcfeed takes 100ms to settle after each step be very carefull adjusting this */
            *pCalTimerMs = VP890_DCFEED_SETTLE_MS;
            nextState = VP890_VAS_PR_SET_ADC;
            break;
        }

        case VP890_VAS_PR_SET_ADC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAS_PR_SET_ADC"));

            SetAdc(pLineCtx, deviceId, ecVal, VP890_METALLIC_DC_I);
            *pCalTimerMs = 10;
            nextState = VP890_VAS_PR_MEASURE;
            break;
        }

        case VP890_VAS_PR_MEASURE: {
            int16 imtOld = pLineObj->calLineData.typeData.loopData.prevVal;
            int16 imtNew = 0;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAS_PR_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &imtNew)) {
                *pCalTimerMs = 10;
                nextState = VP890_VAS_PR_MEASURE;
                break;
            }

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP890_VAS_PR_MEASURE: IMT Old %d IMT New %d",
                imtOld, imtNew));

            runAnotherState = TRUE;
            if (pLineObj->calLineData.typeData.loopData.loopNum++ == 0) {

                /* if this is the first imt measurement then get another */
                nextState = VP890_VAS_PR_STEP;

            } else if ((imtNew > VP890_IMT_10MA) ||
                    (((imtNew + 15 - imtOld) & 0xFFE0) != 0) ) {
                /*
                 * if the measured value is greater than 10 mA or the difference
                 * between old and new values are greater +9/-7 then get another
                 */

                 nextState = VP890_VAS_PR_STEP;
            } else {
                /* if there is nothing left then store the data */
                nextState = VP890_VAS_PR_STORE;
            }

            /* replace the old with the new */
            pLineObj->calLineData.typeData.loopData.prevVal = imtNew;
            break;
        }

        case VP890_VAS_PR_STORE: {
            uint16 vasVal;
            uint8 dcFeed[VP890_DC_FEED_LEN];

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAS_PR_STORE"));

            /* Compute final VAS Values */
            VpMpiCmdWrapper(deviceId, ecVal, VP890_DC_FEED_RD, VP890_DC_FEED_LEN, dcFeed);
            vasVal = VP890_VAS_CONVERSION(dcFeed[0], dcFeed[1]);

            /*
             * This function will determine if VAS is at max, and if so then
             * increase Battery correction by the overhead amount.
             */
            ComputeFinalVas(pLineCtx, &vasVal);

            /*
             * Set VAS to the final value. Note that this is NOT redundant from
             * the previous read because VAS can be actually be reduced. It is
             * the case when VAS reaches max and a battery adjustment increases
             * such that the total VAS_MAX + battery_adjust_actual is > the
             * required VAS + Overhead. This occurs due to differences in step
             * sizes betwen battery adjustment and VAS.
             */
            VpCSLACSetVas(pLineObj->calLineData.dcFeedPr, vasVal);

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP890_VAS_PR_STORE: Setting VAS (%d) with Final Values 0x%02X 0x%02X at time %d",
                vasVal,
                pLineObj->calLineData.dcFeedPr[0], pLineObj->calLineData.dcFeedPr[1],
                pDevObj->timeStamp));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DC_FEED_WRT, VP890_DC_FEED_LEN, pLineObj->calLineData.dcFeedPr);

            /* Second byte contains the lower two bits of VAS */
            pDevObj->vp890SysCalData.vas[channelId][VP890_REV_POLARITY] =
                ((pLineObj->calLineData.dcFeedPr[1] >> 6) & 0x3);

            /* First byte contains the upper two bits of VAS */
            pDevObj->vp890SysCalData.vas[channelId][VP890_REV_POLARITY] |=
                ((pLineObj->calLineData.dcFeedPr[0] << 2) & 0xC);

            runAnotherState = TRUE;
            nextState = VP890_VAB_PR_SETUP;
            break;
        }

        /***********************************************************************
         * Measure VAB (VOC)  in polrev -
         *
         * These two states setup and measure the VAB (VOC) value in polrev.
         **********************************************************************/

        case VP890_VAB_PR_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAB_PR_SETUP"));

            SetAdc(pLineCtx, deviceId, ecVal, VP890_METALLIC_DC_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP890_VAB_PR_MEASURE;
            break;
        }

        case VP890_VAB_PR_MEASURE: {
            int16 *pVocRev = &pLineObj->calLineData.typeData.vocData.vocRev;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAB_PR_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, pVocRev)) {
                *pCalTimerMs = 10;
                nextState = VP890_VAB_PR_MEASURE;
            } else {
                runAnotherState = TRUE;
                nextState = VP890_VAB_PR_ADC_OFFSET_SETUP;
            }
            break;
        }

        /***********************************************************************
         * Measure VAB ADC offset in polrev -
         *
         * These two states setup and measure the ADC Tip to Ring voltage in
         * polrev.
         **********************************************************************/

        case VP890_VAB_PR_ADC_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAB_PR_ADC_OFFSET_SETUP"));

            /* Disconnect tip and ring sense - start feed collapse */
            pLineObj->dcCalValues[0] = 0x00;
            pLineObj->dcCalValues[1] = (VP890_C_RING_SNS_CUT | VP890_C_TIP_SNS_CUT);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DC_CAL_REG_WRT, VP890_DC_CAL_REG_LEN, pLineObj->dcCalValues);

            /* set the converter config register to measure Metallic DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP890_METALLIC_DC_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP890_VAB_PR_ADC_OFFSET_MEASURE;
            break;
        }

        case VP890_VAB_PR_ADC_OFFSET_MEASURE: {
            int16 temp = 0;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAB_PR_ADC_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP890_VAB_PR_ADC_OFFSET_MEASURE;
            } else {
                pDevObj->vp890SysCalData.vocOffset[channelId][VP890_REV_POLARITY] = temp;
                runAnotherState = TRUE;
                nextState = VP890_VA_PR_ADC_OFFSET_SETUP;
            }
            break;
        }

        /***********************************************************************
         * Measure VA ADC offset in polrev
         *
         * These two states setup and measure the ADC Tip to Gnd voltage
         * offset in normal polarity.
         **********************************************************************/

        case VP890_VA_PR_ADC_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VA_PR_ADC_OFFSET_SETUP"));

            /* set the converter config register to measure Tip DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP890_TIP_TO_GND_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP890_VA_PR_ADC_OFFSET_MEASURE;
            break;
        }

        case VP890_VA_PR_ADC_OFFSET_MEASURE: {
            int16 temp = 0;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VA_PR_ADC_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP890_VA_PR_ADC_OFFSET_MEASURE;
            } else {
                /* compensate for the static offset of 1.5V on 890*/
                pDevObj->vp890SysCalData.vagOffsetRev[channelId] = temp + 205;
                runAnotherState = TRUE;
                nextState = VP890_VB_PR_ADC_OFFSET_SETUP;
            }
            break;
        }

        /***********************************************************************
         * Measure VB ADC offset in polrev
         *
         * These two states setup and measure the ADC RING to Gnd voltage
         * offset in normal polarity.
         **********************************************************************/

        case VP890_VB_PR_ADC_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VB_PR_ADC_OFFSET_SETUP"));

            /* set the converter config register to measure Ring DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP890_RING_TO_GND_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP890_VB_PR_ADC_OFFSET_MEASURE;
            break;
        }

        case VP890_VB_PR_ADC_OFFSET_MEASURE: {
            int16 temp = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VB_PR_ADC_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP890_VB_PR_ADC_OFFSET_MEASURE;
            } else {
                /* compensate for the static offset of 1.5V on 890*/
                pDevObj->vp890SysCalData.vbgOffsetRev[channelId] = temp + 205;
                runAnotherState = TRUE;
                nextState = VP890_COLLAPSE_FEED;
            }
            break;
        }

        /***********************************************************************
         * Collapse Feed
         *
         * This state collapses tip and ring and switches to normal polarity.
         * This is done with the tip/ring sense cut to prevent a transient.
         *
         * We are also preparing the siggen A for calibration test.
         **********************************************************************/

        case VP890_COLLAPSE_FEED: {
            /* Sig Gen A cal setup */
            uint8 sigGenA[VP890_SIGA_PARAMS_LEN] =
                {0x00, 0x00, 0x00, 0x0A, 0xAB, 0x00, 0x02, 0x00, 0x00 ,0x00, 0x00};

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_COLLAPSE_FEED"));

            /* Disable VOC DAC */
            pLineObj->icr2Values[0] |= 0x20;
            pLineObj->icr2Values[1] &= ~0x20;

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_ICR2_WRT, VP890_ICR2_LEN, pLineObj->icr2Values);

            /* Re-Enable tip and ring sense */
            pLineObj->dcCalValues[0] = 0x00;
            pLineObj->dcCalValues[1] = 0x00;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DC_CAL_REG_WRT, VP890_DC_CAL_REG_LEN, pLineObj->dcCalValues);

            /* switch to normal polarity */
            pLineObj->slicValueCache &= ~VP890_SS_POLARITY_MASK;
            VpMpiCmdWrapper(deviceId, ecVal, VP890_SYS_STATE_WRT,
                VP890_SYS_STATE_LEN, &pLineObj->slicValueCache);

            /*
             * set the sig gen with 0 bias, almost no amplitude and
             * 1000Hz use for ring offset
             */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_SIGA_PARAMS_WRT, VP890_SIGA_PARAMS_LEN, sigGenA);

            *pCalTimerMs = 100;
            nextState = VP890_GENA_NP_OFFSET_SETUP;
            break;
        }

        /***********************************************************************
         * Measure SIGGEN A offset -
         *
         * These states setup and measure the offset in the signal generator
         **********************************************************************/

        case VP890_GENA_NP_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_GENA_NP_OFFSET_SETUP"));

            /* store the current slicState */
            pLineObj->calLineData.sysState = pLineObj->slicValueCache;

            /* go to ringing state */
            pLineObj->slicValueCache &= ~VP890_SS_STATE_MASK;
            pLineObj->slicValueCache |= VP890_SS_BALANCED_RINGING;
            VpMpiCmdWrapper(deviceId, ecVal, VP890_SYS_STATE_WRT,
                VP890_SYS_STATE_LEN, &pLineObj->slicValueCache);
            *pCalTimerMs = 20; /* wait for ringing */
            nextState = VP890_GENA_NP_OFFSET_SET_ADC;

            break;
        }

        case VP890_GENA_NP_OFFSET_SET_ADC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_GENA_NP_OFFSET_SET_ADC"));

            /* set the converter config register to measure Metallic DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP890_METALLIC_DC_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP890_GENA_NP_OFFSET_MEASURE;
            break;
        }

        case VP890_GENA_NP_OFFSET_MEASURE: {
            int16 temp = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_GENA_NP_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP890_GENA_NP_OFFSET_MEASURE;
            } else {
                /* no need to measure offet in rev polrity */
                pDevObj->vp890SysCalData.sigGenAError[channelId][VP890_NORM_POLARITY] = temp;

                /*
                 * we must get the slic out of ringing before restoring
                 * siggen A or risk catching some of the ring cycle
                 * and pinging a phone.
                 */
                pLineObj->slicValueCache = pLineObj->calLineData.sysState;
                VpMpiCmdWrapper(deviceId, ecVal, VP890_SYS_STATE_WRT,
                    VP890_SYS_STATE_LEN, &pLineObj->slicValueCache);
                *pCalTimerMs = 10;
                nextState = VP890_GENA_NP_OFFSET_RESTORE;
            }
            break;
        }

        case VP890_GENA_NP_OFFSET_RESTORE: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_GENA_NP_OFFSET_RESTORE"));

            /* restore the siggen A register */
            VpMpiCmdWrapper(deviceId, ecVal, VP890_SIGA_PARAMS_WRT,
                VP890_SIGA_PARAMS_LEN, pLineObj->calLineData.sigGenA);

            *pCalTimerMs = 30; /* give the device time to get out of ringing */
            nextState = VP890_ILG_OFFSET_SETUP;
            break;
        }

        /***********************************************************************
         * Measure ILG offset -
         *
         * These two states setup and measure ILG offset in normal polarity.
         * This measurement is take with tip/ring collapsed and sense enabled
         **********************************************************************/

        case VP890_ILG_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_ILG_OFFSET_SETUP"));

            /* set the converter config register to measure Logitudinal Current */
            SetAdc(pLineCtx, deviceId, ecVal, VP890_LONGITUDINAL_DC_I);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP890_ILG_OFFSET_MEASURE;
            break;
        }

        case VP890_ILG_OFFSET_MEASURE: {
            int16 temp = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_ILG_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP890_ILG_OFFSET_MEASURE;
            } else {
                pDevObj->vp890SysCalData.ilgOffsetNorm[channelId] = temp;
                runAnotherState = TRUE;
                nextState = VP890_ILA_OFFSET_SETUP;
            }
            break;
        }

        /***********************************************************************
         * Measure ILA offset -
         *
         * These two states setup and measure ILA offset in normal polarity.
         * This measurement is take with tip/ring collapsed and sense enabled
         **********************************************************************/

        case VP890_ILA_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_ILA_OFFSET_SETUP"));

            /* set the converter config register to measure Metallic DC Current */
            SetAdc(pLineCtx, deviceId, ecVal, VP890_METALLIC_DC_I);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP890_ILA_OFFSET_MEASURE;
            break;
        }

        case VP890_ILA_OFFSET_MEASURE: {
            int16 temp = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_ILA_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP890_ILA_OFFSET_MEASURE;
            } else {
                pDevObj->vp890SysCalData.ilaOffsetNorm[channelId] = temp;
                runAnotherState = TRUE;
                nextState = VP890_RESTORE_DAC;
            }
            break;
        }

        /***********************************************************************
         * Restore Feed
         *
         * This state restores the DC Feed to the reference dc
         * profile value and reenables the DAC while in normal polarity.
         *
         * Additionally, this state will disable tip/ring
         * sense for future offset measurements in normal polarity.
         **********************************************************************/

        case VP890_RESTORE_DAC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_RESTORE_DAC"));

            /* Store the ref dcfeed from profile into normal cal dcfeed */
            VpMemCpy(pLineObj->calLineData.dcFeed, pLineObj->calLineData.dcFeedRef,
                VP890_DC_FEED_LEN);

            /* we want to start with a #.0V VAS */
            pLineObj->calLineData.dcFeed[0] &= 0xFC;
            pLineObj->calLineData.dcFeed[1] &= 0x3F;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DC_FEED_WRT, VP890_DC_FEED_LEN, pLineObj->calLineData.dcFeed);

            /* Enable VOC DAC */
            pLineObj->icr2Values[0] &= ~0x20;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_ICR2_WRT, VP890_ICR2_LEN, pLineObj->icr2Values);

            /* disable tip and ring sense */
            pLineObj->dcCalValues[0] = 0x00;
            pLineObj->dcCalValues[1] = (VP890_C_RING_SNS_CUT | VP890_C_TIP_SNS_CUT);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DC_CAL_REG_WRT, VP890_DC_CAL_REG_LEN, pLineObj->dcCalValues);

            runAnotherState = TRUE;
            nextState = VP890_VAB_NP_ADC_OFFSET_SETUP;
            break;
        }

        /***********************************************************************
         * Measure VAB ADC offset in normal polarity -
         *
         * These two states setup and measure ADC Tip to RING voltage offset in
         * normal polarity.
         **********************************************************************/

        case VP890_VAB_NP_ADC_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAB_NP_ADC_OFFSET_SETUP"));

            /* set the converter config register to measure Metallic DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP890_METALLIC_DC_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP890_VAB_NP_ADC_OFFSET_MEASURE;
            break;
        }

        case VP890_VAB_NP_ADC_OFFSET_MEASURE: {
            int16 temp = 0;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAB_NP_ADC_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP890_VAB_NP_ADC_OFFSET_MEASURE;
            } else {
                pDevObj->vp890SysCalData.vocOffset[channelId][VP890_NORM_POLARITY] = temp;
                runAnotherState = TRUE;
                nextState = VP890_VA_NP_ADC_OFFSET_SETUP;
            }
            break;
        }

        /***********************************************************************
         * Measure VA ADC offset in normal polarity
         *
         * These two states setup and measure the ADC Tip to Gnd voltage
         * offset in normal polarity.
         **********************************************************************/

        case VP890_VA_NP_ADC_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VA_NP_ADC_OFFSET_SETUP"));

            /* set the converter config register to measure Tip DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP890_TIP_TO_GND_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP890_VA_NP_ADC_OFFSET_MEASURE;
            break;
        }

        case VP890_VA_NP_ADC_OFFSET_MEASURE: {
            int16 temp = 0;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VA_NP_ADC_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP890_VA_NP_ADC_OFFSET_MEASURE;
            } else {
                /* compensate for the static offset of 1.5V on 890*/
                pDevObj->vp890SysCalData.vagOffsetNorm[channelId] = temp + 205;
                runAnotherState = TRUE;
                nextState = VP890_VB_NP_ADC_OFFSET_SETUP;
            }
            break;
        }

        /***********************************************************************
         * Measure VB ADC offset in normal polarity
         *
         * These two states setup and measure the ADC RING to Gnd voltage
         * offset in normal polarity.
         **********************************************************************/

        case VP890_VB_NP_ADC_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VB_NP_ADC_OFFSET_SETUP"));

            /* set the converter config register to measure Ring DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP890_RING_TO_GND_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP890_VB_NP_ADC_OFFSET_MEASURE;
            break;
        }

        case VP890_VB_NP_ADC_OFFSET_MEASURE: {
            int16 temp = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VB_NP_ADC_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP890_VB_NP_ADC_OFFSET_MEASURE;
            } else {
                /* compensate for the static offset of 1.5V on 890*/
                pDevObj->vp890SysCalData.vbgOffsetNorm[channelId] = temp + 205;
                runAnotherState = TRUE;
                nextState = VP890_ILA_SETUP;
            }
            break;
        }

        /***********************************************************************
         * Measure ILA at 4 different values -
         *
         * These xyz states setup and measure ILA offset in normal polarity
         * at 4 different ILA settings (20, 25, 32 and 40mA).
         **********************************************************************/

        case VP890_ILA_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_ILA_SETUP"));

            /* prepare storage data */
            pLineObj->calLineData.typeData.loopData.loopNum = 0;
            pLineObj->calLineData.typeData.loopData.prevVal = 0;

            pLineObj->calLineData.minVas = 0;

            runAnotherState = TRUE;
            nextState = VP890_ILA_ADJUST;
            break;
        }

        case VP890_ILA_ADJUST: {
            uint8 *pDcFeed = pLineObj->calLineData.dcFeed;
            uint8 loopNum = pLineObj->calLineData.typeData.loopData.loopNum;
            uint16 vasValue = VP890_VAS_CONVERSION(pDcFeed[0], pDcFeed[1]);

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_ILA_ADJUST"));

            /* adjust the ila value and vas */
            pDcFeed[1] &= ~VP890_ILA_MASK;
            if (loopNum == 0) { /* 20mA */
                pDcFeed[1] |= (VP890_ILA_MASK & 0x02);
                vasValue = 3000;
            } else if (loopNum == 1) { /* 25mA */
                pDcFeed[1] |= (VP890_ILA_MASK & 0x07);
                vasValue = 3750;
            } else if (loopNum == 2) { /* 32mA */
                pDcFeed[1] |= (VP890_ILA_MASK & 0x0E);
                vasValue = 4500;
            } else if (loopNum == 3) { /* 40mA */
                pDcFeed[1] |= (VP890_ILA_MASK & 0x16);
                vasValue = 5250;
            } else {
                /*
                 * If we're here, it's because off-hook has not yet been
                 * detected with VAS increments, so we have to continue past
                 * normal ILA calibration.
                 */
                vasValue += 750;
            }

            if (vasValue < VP890_VAS_MAX) {
                VpCSLACSetVas(pDcFeed, vasValue);

                VP_CALIBRATION(VpLineCtxType, pLineCtx,
                    ("VP890_ILA_ADJUST: Writing DC FEED 0x%02X 0x%02X",
                    pDcFeed[0], pDcFeed[1]));

                VpMpiCmdWrapper(deviceId, ecVal, VP890_DC_FEED_WRT,
                    VP890_DC_FEED_LEN, pDcFeed);

                *pCalTimerMs = 10;
                nextState = VP890_ILA_SET_ADC;
            } else {
                /* ERROR!! Cannot increase VAS. Need to move on */
                pLineObj->calLineData.minVas = VP890_VAS_MAX;
                runAnotherState = TRUE;
                nextState = VP890_RESTORE_FEED;
            }
            break;
        }

        case VP890_ILA_SET_ADC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_ILA_SET_ADC"));

            /* set the converter config register to measure Metallic DC Current */
            SetAdc(pLineCtx, deviceId, ecVal, VP890_METALLIC_DC_I);

            *pCalTimerMs = 10;
            nextState = VP890_ILA_MEASURE;
            break;
        }

        case VP890_ILA_MEASURE: {
            int16 temp = 0;
            uint8 loopNum = pLineObj->calLineData.typeData.loopData.loopNum;
            uint8 sigReg[VP890_NO_UL_SIGREG_LEN];
            uint16 vasValue;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_ILA_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP890_ILA_MEASURE;
                break;
            }

            /* store the measured ila data */
            if (loopNum == 0) { /* 20mA */
                pDevObj->vp890SysCalData.ila20[channelId] = temp;
            } else if (loopNum == 1) { /* 25mA */
                pDevObj->vp890SysCalData.ila25[channelId] = temp;
            } else if (loopNum == 2) { /* 32mA */
                pDevObj->vp890SysCalData.ila32[channelId] = temp;
            } else if (loopNum == 3) { /* 40mA */
                pDevObj->vp890SysCalData.ila40[channelId] = temp;
            }

            runAnotherState = TRUE;

            VpMpiCmdWrapper(deviceId, ecVal, VP890_NO_UL_SIGREG_RD,
                VP890_NO_UL_SIGREG_LEN, sigReg);
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP890_ILA_MEASURE: SIG REG 0x%02X 0x%02X", sigReg[0], sigReg[1]));

            if ((sigReg[channelId] & VP890_HOOK_MASK) && (pLineObj->calLineData.minVas == 0)) {
                vasValue = VP890_VAS_CONVERSION(pLineObj->calLineData.dcFeed[0],
                    pLineObj->calLineData.dcFeed[1]);

                if ((vasValue + VP890_VAS_MIN_OVERHEAD) < VP890_VAS_MAX) {
                    pLineObj->calLineData.minVas = vasValue + VP890_VAS_MIN_OVERHEAD;
                } else {
                    pLineObj->calLineData.minVas = VP890_VAS_MAX;
                }

                VP_CALIBRATION(VpLineCtxType, pLineCtx, ("VP890_ILA_MEASURE: Saving Min VAS (%d)",
                    pLineObj->calLineData.minVas));
            }

            if ((pLineObj->calLineData.typeData.loopData.loopNum++ < 3)
             || (!(sigReg[channelId] & VP890_HOOK_MASK))) {
                nextState = VP890_ILA_ADJUST;
            } else {
                nextState = VP890_RESTORE_FEED;
            }

            break;
        }

        /***********************************************************************
         * Restore Feed
         *
         * For the most part feed is already restored. All we do here is
         * reenable the tip and ring sense and allow the line to settle.
         **********************************************************************/

        case VP890_RESTORE_FEED: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_RESTORE_FEED"));

            /* Re-Enable tip and ring sense */
            pLineObj->dcCalValues[0] = 0x00;
            pLineObj->dcCalValues[1] = 0x00;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DC_CAL_REG_WRT, VP890_DC_CAL_REG_LEN, pLineObj->dcCalValues);

            *pCalTimerMs = 10;
            nextState = VP890_IMT_NP_SETUP;
            break;
        }

       /***********************************************************************
         * Settle IMT in PolRev -
         *
         * These states setup, measure Metallic current until it
         * settles. This is done to prevent high REN loads from distorting
         * the VAS measurements
         **********************************************************************/
        case VP890_IMT_NP_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_IMT_NP_SETUP"));

            /* prepare storage data */
            pLineObj->calLineData.typeData.loopData.loopNum = 0;
            pLineObj->calLineData.typeData.loopData.prevVal = 0;

            runAnotherState = TRUE;
            nextState = VP890_IMT_NP_SET_ADC;
            break;
        }

        case VP890_IMT_NP_SET_ADC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_IMT_NP_SET_ADC"));

            SetAdc(pLineCtx, deviceId, ecVal, VP890_METALLIC_DC_I);
            *pCalTimerMs = 10;
            nextState = VP890_IMT_NP_MEASURE;
            break;
        }

        case VP890_IMT_NP_MEASURE: {
            int16 imOld = 0;
            int16 imNew = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_IMT_NP_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &imNew)) {
                *pCalTimerMs = 10;
                nextState = VP890_IMT_NP_MEASURE;
                break;
            }

            /* if this is the first imt measurement then get another */
            if (pLineObj->calLineData.typeData.loopData.loopNum++ == 0) {
                pLineObj->calLineData.typeData.loopData.prevVal = imNew;
                *pCalTimerMs = VP890_IMT_SETTLE_MS;
                nextState = VP890_IMT_NP_MEASURE;
                break;
            }
            imOld = pLineObj->calLineData.typeData.loopData.prevVal;

            /* have we settled enough or taken to long > 200ms*/
            if ( (pLineObj->calLineData.typeData.loopData.loopNum < 20 /*200ms*/) &&
                (((imNew + 15 - imOld) & 0xFFE0) != 0) ) {
                /* nope run it again */
                pLineObj->calLineData.typeData.loopData.prevVal = imNew;
                *pCalTimerMs = VP890_IMT_SETTLE_MS;
                nextState = VP890_IMT_NP_MEASURE;
                break;
            }

            /* move to the next state block */
            nextState = VP890_VAS_NP_SETUP;
            runAnotherState = TRUE;
            break;
        }

        /***********************************************************************
         * Measure VAS in Normal Polarity -
         *
         * These five states step through VAS values and measure IMT.
         * The VAS adjustment portion of this process will continue until:
         *  VAS >= 14.25V ||
         *  new IMT < 10mA && (new IMT - old IMT are with in +9 to -7mA)
         *
         * If VAS reaches 14.25, proceed to Battery Adjustments.
         **********************************************************************/

        case VP890_VAS_NP_SETUP:

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAS_NP_SETUP"));

            /* prepare storage data */
            pLineObj->calLineData.typeData.loopData.loopNum = 0;
            pLineObj->calLineData.typeData.loopData.prevVal = 0;

            pLineObj->calLineData.vasStart = 3000;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP890_VAS_NP_SETUP: Best Actual VAS Start %d",
                pLineObj->calLineData.vasStart));

            runAnotherState = TRUE;
            nextState = VP890_VAS_NP_STEP;
            break;

        case VP890_VAS_NP_STEP: {
            uint8 loopNum = pLineObj->calLineData.typeData.loopData.loopNum;
            uint16 vasVal = pLineObj->calLineData.vasStart + (750 * loopNum);

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAS_NP_STEP"));

            /* If the calculated VAS is max, we need to move to battery steps */
            if (vasVal > VP890_VAS_MAX) {
                runAnotherState = TRUE;
                nextState = VP890_VAS_NP_STORE;
                break;
            }

            /* set VAS to the next value */
            VpCSLACSetVas(pLineObj->calLineData.dcFeed, vasVal);
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP890_VAS_NP_STEP: Setting VAS (%d) with Values 0x%02X 0x%02X at time %d",
                vasVal,
                pLineObj->calLineData.dcFeed[0], pLineObj->calLineData.dcFeed[1],
                pDevObj->timeStamp));
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DC_FEED_WRT, VP890_DC_FEED_LEN, pLineObj->calLineData.dcFeed);

            /* dcfeed takes 100ms to settle after each step be very carefull adjusting this */
            *pCalTimerMs = VP890_DCFEED_SETTLE_MS;
            nextState = VP890_VAS_NP_SET_ADC;
            break;
        }

        case VP890_VAS_NP_SET_ADC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAS_NP_SET_ADC"));

            /* set the converter config register to measure Metallic DC Current */
            SetAdc(pLineCtx, deviceId, ecVal, VP890_METALLIC_DC_I);

            *pCalTimerMs = 10;
            nextState = VP890_VAS_NP_MEASURE;
            break;
        }

        case VP890_VAS_NP_MEASURE: {
            int16 imtOld = pLineObj->calLineData.typeData.loopData.prevVal;
            int16 imtNew = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAS_NP_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &imtNew)) {
                *pCalTimerMs = 10;
                nextState = VP890_VAS_NP_MEASURE;
                break;
            }

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP890_VAS_NP_MEASURE: IMT Old %d IMT New %d",
                imtOld, imtNew));

            runAnotherState = TRUE;
            if (pLineObj->calLineData.typeData.loopData.loopNum++ == 0) {

                /* if this is the first imt measurement then get another */
                nextState = VP890_VAS_NP_STEP;

            } else if ((imtNew > VP890_IMT_10MA) ||
                    (((imtOld + 15 - imtNew) & 0xFFE0) != 0) ) {
                /*
                 * if the measured value is greater than 10 mA or the difference
                 * between old and new values are greater +9/-7 then get another
                 */

                 nextState = VP890_VAS_NP_STEP;
            } else {
                /* if there is nothing left then store the data */
                nextState = VP890_VAS_NP_STORE;
            }

            /* replace the old with the new */
            pLineObj->calLineData.typeData.loopData.prevVal = imtNew;
            break;
        }

        case VP890_VAS_NP_STORE: {
            uint16 vasVal;
            uint8 dcFeed[VP890_DC_FEED_LEN];

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAS_NP_STORE"));

            /* Compute final VAS Values */
            VpMpiCmdWrapper(deviceId, ecVal, VP890_DC_FEED_RD, VP890_DC_FEED_LEN,
                dcFeed);
            vasVal = VP890_VAS_CONVERSION(dcFeed[0], dcFeed[1]);

            /*
             * This function will determine if VAS is at max, and if so then
             * increase Battery correction by the overhead amount.
             */
            ComputeFinalVas(pLineCtx, &vasVal);

            /*
             * Set VAS to the final value. Note that this is NOT redundant from
             * the previous read because VAS can actually be reduced. It is
             * the case when VAS reaches max and a battery adjustment increases
             * such that the total VAS_MAX + battery_adjust_actual is > the
             * required VAS + Overhead. This occurs due to differences in step
             * sizes betwen battery adjustment and VAS.
             *
             * Make sure VAS is not set below minimum determined from Battery
             * Saturation Detection algorithm.
             */
            if (vasVal < pLineObj->calLineData.minVas) {
                vasVal = pLineObj->calLineData.minVas;
            }
            VpCSLACSetVas(pLineObj->calLineData.dcFeed, vasVal);

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP890_VAS_NP_STORE: Setting VAS (%d) with Final Values 0x%02X 0x%02X at time %d",
                vasVal,
                pLineObj->calLineData.dcFeed[0], pLineObj->calLineData.dcFeed[1],
                pDevObj->timeStamp));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DC_FEED_WRT, VP890_DC_FEED_LEN, pLineObj->calLineData.dcFeed);

            /* Second byte contains the lower two bits of VAS */
            pDevObj->vp890SysCalData.vas[channelId][VP890_NORM_POLARITY] =
                ((pLineObj->calLineData.dcFeed[1] >> 6) & 0x3);

            /* First byte contains the upper two bits of VAS */
            pDevObj->vp890SysCalData.vas[channelId][VP890_NORM_POLARITY] |=
                ((pLineObj->calLineData.dcFeed[0] << 2) & 0xC);

            /*
             * Make sure Reverse Polarity VAS is not set below minimum
             * determined from Battery Saturation Detection algorithm.
             */
            vasVal = VP890_VAS_CONVERSION(pLineObj->calLineData.dcFeedPr[0],
                pLineObj->calLineData.dcFeedPr[1]);
            if (vasVal < pLineObj->calLineData.minVas) {
                VpCSLACSetVas(pLineObj->calLineData.dcFeedPr,
                    pLineObj->calLineData.minVas);

                /* Second byte contains the lower two bits of VAS */
                pDevObj->vp890SysCalData.vas[channelId][VP890_REV_POLARITY] =
                    ((pLineObj->calLineData.dcFeedPr[1] >> 6) & 0x3);

                /* First byte contains the upper two bits of VAS */
                pDevObj->vp890SysCalData.vas[channelId][VP890_REV_POLARITY] |=
                    ((pLineObj->calLineData.dcFeedPr[0] << 2) & 0xC);
            }
            runAnotherState = TRUE;
            nextState = VP890_VAB_NP_SETUP;
            break;
        }

        /***********************************************************************
         * Measure VAB (VOC)  in normal polarity -
         *
         * These two states setup and measure the VAB (VOC) value in normal pol.
         **********************************************************************/

        case VP890_VAB_NP_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAB_NP_SETUP"));

            /* set the converter config register to measure Metallic DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP890_METALLIC_DC_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP890_VAB_NP_MEASURE;
            break;
        }

        case VP890_VAB_NP_MEASURE: {
            int16 temp = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_VAB_NP_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP890_VAB_NP_MEASURE;
            } else {
                pLineObj->calLineData.typeData.vocData.vocNorm = temp;
                runAnotherState = TRUE;
                nextState = VP890_CAL_ADJUST;
            }
            break;
        }

        /***********************************************************************
         * This state is used to perform calcualtions
         **********************************************************************/
        case VP890_CAL_ADJUST: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_CAL_ADJUST"));

            /* Set VOC to the device profile value */
            Vp890AdjustVoc(pLineCtx, ((pLineObj->calLineData.dcFeedRef[0] >> 2) & 0x7), FALSE);

            /* Set ILA to the device profile value */
            Vp890AdjustIla(pLineCtx, (pLineObj->calLineData.dcFeedRef[1] & VP890_ILA_MASK));
            runAnotherState = TRUE;
            nextState = VP890_CAL_RESTORE;
            break;
        }


        /***********************************************************************
         * This state is used to restore registers and clean up after cal
         **********************************************************************/
        case VP890_CAL_RESTORE: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP890_CAL_RESTORE"));

            /* Restore Switching Regulator Parameters if necessary */
            if (pDevObj->devProfileData.swParams[0] & 0x02) {
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                    VP890_REGULATOR_PARAM_WRT, VP890_REGULATOR_PARAM_LEN,
                    pDevObj->devProfileData.swParams);
                VpMemCpy(pDevObj->swParamsCache, pDevObj->devProfileData.swParams,
                    VP890_REGULATOR_PARAM_LEN);
            }

            /* Restore Device Mode */
            pDevObj->devMode[0] |= VP890_DEV_MODE_TEST_DATA;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DEV_MODE_WRT, VP890_DEV_MODE_LEN, pDevObj->devMode);

            /* Restore Loop Supervision */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_LOOP_SUP_WRT, VP890_LOOP_SUP_LEN, pLineObj->calLineData.loopSup);

            /* Restore Codec Mode */
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP890_CAL_RESTORE: Writing 0x%02X to Operating Functions",
                pLineObj->calLineData.codecReg));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_OP_FUNC_WRT, VP890_OP_FUNC_LEN, &pLineObj->calLineData.codecReg);

            /* Restore TX/RX PCM and enable HPF */
            pLineObj->opCond[0] = (VP890_CUT_TXPATH | VP890_CUT_RXPATH);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_OP_COND_WRT, VP890_OP_COND_LEN, pLineObj->opCond);

            /* Restore disn */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_DISN_WRT, VP890_DISN_LEN, pLineObj->calLineData.disnVal);

            /* Restore vpGain */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_VP_GAIN_WRT, VP890_VP_GAIN_LEN, pLineObj->calLineData.vpGain);

            /* Restore ICR2 */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_ICR2_WRT, VP890_ICR2_LEN, pLineObj->calLineData.icr2);
            VpMemCpy(pLineObj->icr2Values, pLineObj->calLineData.icr2, VP890_ICR2_LEN);

            /* Restore ICR3 */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP890_ICR3_WRT, VP890_ICR3_LEN, pLineObj->calLineData.icr3);
            VpMemCpy(pLineObj->icr3Values, pLineObj->calLineData.icr3, VP890_ICR2_LEN);

            /*
             * Restore Line State (note: this won't correctly update the
             * calibrated DC feed settings because we're still in calibration.
             * So we have to do that manually)
             */
            Vp890SetFxsLineState(pLineCtx, pLineObj->lineState.usrCurrent);
            if (pLineObj->calLineData.reversePol) {
                /* write polrev dcfeed here */
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                    VP890_DC_FEED_WRT, VP890_DC_FEED_LEN, pLineObj->calLineData.dcFeedPr);
            } else {
                /* write normal dcfeed here */
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                    VP890_DC_FEED_WRT, VP890_DC_FEED_LEN, pLineObj->calLineData.dcFeed);
            }

            Vp890LLSetSysState(deviceId, pLineCtx, 0, FALSE);
            #ifdef VP890_LP_SUPPORT
                /* Force an update on the line */
                Vp890LowPowerMode(pDevCtx);
            #endif
            break;
        }

        default:
            VP_CALIBRATION(VpLineCtxType, pLineCtx,("Cal Error - Bad State jumping to DONE"));
            pLineObj->calLineData.calState = VP890_CAL_RESTORE;
            runAnotherState = TRUE;
            pDevObj->responseData = VP_CAL_FAILURE;
            break;
    }


    /* write down any data required by a state */
    if (mpiIndex > 0) {
        VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0],
            mpiIndex-1, &mpiBuffer[1]);
    }

    pLineObj->calLineData.calState = nextState;
    return runAnotherState;
}

/*
 *  ComputeFinalVas()
 *     This function computes the final VAS value based on converged VAS and
 * overhead. If the result exceeds silicon capabilities, the battery is
 * adjusted and the new value saved to the device object error value. This
 * value is directly used for Battery Settings and provided in the calibration
 * profile (see VpCal(VP_CAL_GET_SYSTEM_COEFFF)).
 */
void
ComputeFinalVas(
    VpLineCtxType *pLineCtx,
    uint16 *vasValue)
{
    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("ComputeFinalVas: Adding Overhead (%d) to base VAS Value (%d)",
        VP890_VAS_OVERHEAD, *vasValue));

    /*
     * Apply known adjustments JUST to the computed VAS Value. This result does
     * not need (yet) to correspond to a programmable value.
     */
    *vasValue += VP890_VAS_OVERHEAD;

    /*
     * Determine if a battery adjustment is necessary just based on the required
     * vas value exceeding programmable limits.
     */
    if (*vasValue > VP890_VAS_MAX) {
        Vp890LineObjectType *pLineObj = pLineCtx->pLineObj;
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;
        uint8 channelId = pLineObj->channelId;  /* Used several times... */

        int32 newTarget;
        int16 targetCalVoltage, batteryCorrection, batteryOverage;

        /*
         * VAS and Battery Adjustments are in two different scales. This will
         * require fixing one (VAS) and round up (Batttery Adjustment). It may
         * give an overall lower VAS+Battery by rounding VAS because VAS is in
         * lower steps, but the Battery needs to be minimized.
         *
         * Remove the maximum VAS setting and round the remaining to the nearest
         * available battery step.
         */
        batteryCorrection = (*vasValue - VP890_VAS_MAX);
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("ComputeFinalVas: Raw Battery Correction: %d",
            batteryCorrection));

        /* Convert the current setting into scale of 1mv */
        targetCalVoltage = pDevObj->vp890SysCalData.abvError[0] * 10;

        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("ComputeFinalVas: Target Cal Voltage (%d) mV from Cal Profile: (%d)",
            targetCalVoltage, pDevObj->vp890SysCalData.abvError[0]));

        /* Compute the unconstrained new battery calibration target. */
        newTarget = (targetCalVoltage + batteryCorrection);
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("ComputeFinalVas: Current Battery Calibration %d New Target Theory %ld",
            targetCalVoltage, newTarget));

        /*
         * At this point, the battery calibration target computed may not be
         * programmable in the silicon. Round up and limit to the max setting.
         * Keep track of the overage being programmed because we'll want to reduce
         * VAS by this amount or as close as possible without going too low.
         */
        batteryOverage = (VP890_BAT_CAL_STEP - (newTarget % VP890_BAT_CAL_STEP));
        newTarget += batteryOverage;
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("ComputeFinalVas: Rounded Battery Correction: %ld", newTarget));

        /*
         * In case we can't adjust the battery enough to match the theoretical
         * requirement, limit to max and disable VAS reduction.
         */
        if (newTarget > VP890_BAT_CAL_MAX) {
            newTarget = VP890_BAT_CAL_MAX;
            batteryOverage = 0;
        }
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("ComputeFinalVas: Final Battery Correction: %ld", newTarget));

        /*
         * The value is now in a programmable range, but does not correspond to
         * the error scale from the calibration profile and stored in the device
         * object. That is in 10mV, this is 10x greater than that.
         */
        newTarget /= 10;

        /*
         * Make sure the value we just computed is NOT lower than the value
         * previously computed. This can occur during VAS calibration due to
         * differences in polarity.
         */
        if (newTarget > pDevObj->vp890SysCalData.abvError[channelId]) {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("ComputeFinalVas: Modifying Current ABV Error from %d to %d on Channel %d",
                pDevObj->vp890SysCalData.abvError[channelId], (int16)newTarget, channelId));

            pDevObj->vp890SysCalData.abvError[channelId] = (int16)newTarget;

            /*
             * Write to the silicon - make sure Wideband Mode Bit IF set is
             * disabled for this write. It shouldn't occur, but be 100% sure
             */
            Vp890BatteryCalAdjust(pDevObj, (pLineObj->ecVal & VP890_EC_BITS_MASK));
        }

        /*
         * Don't forget that the pointer provided needs to correspond to the
         * final VAS value being programmed. Reduce by as much as possible
         * without reducing by more than theoretical.
         */
        *vasValue = VP890_VAS_MAX
                  - (batteryOverage - (batteryOverage % VP890_VAS_STEP));
    }

    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("ComputeFinalVas: Overhead %d => rounded off final VAS %d",
        VP890_VAS_OVERHEAD, *vasValue));

    return;
}

/**
 * Vp890BatteryCalAdjust()
 *  This function computes the battery calibration error and adjust the device
 * register and object content.
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  Battery calibration registers are adjusted. Device object is updated.
 */
void
Vp890BatteryCalAdjust(
    Vp890DeviceObjectType *pDevObj,
    uint8 ecVal)
{
    uint8 channelId = (((ecVal & VP890_EC_BITS_MASK) == VP890_EC_CH1) ? 0 : 1);
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 swCal[VP890_BAT_CALIBRATION_LEN];
    int32 abvError =
        (pDevObj->vp890SysCalData.abvError[channelId] * VP890_V_SCALE / VP890_V_PCM_LSB);
    uint8 wbMode = (pDevObj->ecVal & VP890_WIDEBAND_MODE);
    uint16 swCalError;

    VP_API_FUNC_INT(None, VP_NULL, ("Vp890BatteryCalAdjust+"));

    VpMpiCmdWrapper(deviceId, (wbMode | ecVal), VP890_BAT_CALIBRATION_RD,
        VP890_BAT_CALIBRATION_LEN, swCal);

    swCal[0] &= ~(VP890_BAT_CAL_SWCAL_MASK);

    /* Conversion from 7.324mV to 1.25V */
    swCalError = (ABS(abvError) / 171);
    if (((ABS(abvError) + 85) /  171) > swCalError) {
        swCalError+=1;
    }
    swCalError = (swCalError > 3) ? 3 : swCalError;
    swCal[0] |= (swCalError << 3);

    /*
     * Positive error means voltage is too low (not negative enough).
     * Positive adjustment makes the battery voltage more negative.
     */
    swCal[0] |= (abvError > 0) ? 0 : VP890_BAT_CAL_SWCAL_SIGN;

    VP_CALIBRATION(None, VP_NULL,
        ("Ch %d: Battery Calibration Correction 0x%02X 0x%02X",
        channelId, swCal[0], swCal[1]));

    /*
     * This write MUST take into account Wideband setting because it can occur
     * during normal system operation using "Apply System Coefficients". If
     * called during normal calibration, the value of pDevObj must be cleared
     * of the Wideband setting.
     */
    VpMpiCmdWrapper(deviceId, (wbMode | ecVal), VP890_BAT_CALIBRATION_WRT,
        VP890_BAT_CALIBRATION_LEN, swCal);

    VP_API_FUNC_INT(None, VP_NULL, ("Vp890BatteryCalAdjust-"));

    return;
}


static void
SetAdc(
    VpLineCtxType *pLineCtx,
    VpDeviceIdType deviceId,
    uint8 ecVal,
    uint8 adcRoute)
{
    Vp890LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 adcConfig;

    VpMpiCmdWrapper(deviceId, ecVal, VP890_CONV_CFG_RD,
        VP890_CONV_CFG_LEN, &adcConfig);

    /* set the adc requested measurement do not adjust the sample rate */
    adcConfig = (adcConfig & ~VP890_CONV_CONNECT_BITS) | adcRoute;
    /* adcConfig = adcRoute; */

    VpMpiCmdWrapper(deviceId, ecVal, VP890_CONV_CFG_WRT,
        VP890_CONV_CFG_LEN, &adcConfig);

    /* store the currently requested ADC route */
    pLineObj->calLineData.typeData.loopData.adcRoute = adcRoute;

    /* reset the adcLoopCounter to 0*/
    pLineObj->calLineData.typeData.loopData.adcLoopMax = 0;

    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("INFO: setting ADC to 0x%02x", adcRoute));
    return;
}

static bool
GetXData(
    VpLineCtxType *pLineCtx,
    VpDeviceIdType deviceId,
    uint8 ecVal,
    int16 *pData)
{
    Vp890LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 *adcLoopMax = &pLineObj->calLineData.typeData.loopData.adcLoopMax;
    uint8 reqAdcRoute = pLineObj->calLineData.typeData.loopData.adcRoute;
    uint8 currentAdc;

    VpMpiCmdWrapper(deviceId, ecVal, VP890_CONV_CFG_RD,
        VP890_CONV_CFG_LEN, &currentAdc);

    if ((currentAdc & 0xF) == reqAdcRoute) {
        /*
         * if the ADC is still set to what we want it to
         * requested setting then take the measurement.
         */
        uint8 xdata[2];
        VpMpiCmdWrapper(deviceId, ecVal, VP890_TX_PCM_DATA_RD,
            VP890_TX_PCM_DATA_LEN, xdata);
        *pData = ( (((int16)xdata[0] << 8) & 0xFF00) | ((int16)xdata[1] & 0x00FF) );
        return TRUE;

    } else if ((*adcLoopMax)++ < 10) {
        /*
         * if the ADC is not correct but we have not exceeded
         * our loopcount, then set it again and get out.
        */
        uint8 adcAgain = (currentAdc & ~VP890_CONV_CONNECT_BITS) | reqAdcRoute;

        VpMpiCmdWrapper(deviceId, ecVal, VP890_CONV_CFG_WRT,
            VP890_CONV_CFG_LEN, &adcAgain);

        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("WARNING: ADC route set to 0x%02X, trying for 0x%02x",
            (currentAdc & 0x0F), reqAdcRoute));
        return FALSE;
    }

    /*
     * if the adc is still not right but we have
     * exceeded our loop count then just move on
     * and print a debug error.
     */
    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("ERROR: unable to set ADC to %i after 10 attempts.. moving on", reqAdcRoute));
    return TRUE;
}


/*******************************************************************************
 * External Functions:
 ******************************************************************************/

/**
 * Vp890CalLine()
 *  This function initiates a calibration operation for analog circuits
 * associated with a given line. See VP-API reference guide for more
 * information.

 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action.
 */
VpStatusType
Vp890CalLine(
    VpLineCtxType *pLineCtx)
{
    Vp890LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint16 tickRate = pDevObj->devProfileData.tickRate;
    VpLineStateType currentState = pLineObj->lineState.usrCurrent;
    uint8 ecVal = pLineObj->ecVal;
    uint8 opCond[VP890_OP_COND_LEN];

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890CalLine()"));

    /* Block if device init is not complete */
    if (!(pDevObj->state & VP_DEV_INIT_CMP)) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    if (pLineObj->status & VP890_IS_FXO) {
        return VP_STATUS_INVALID_ARG;
    }

    /*
     * Do not proceed if the device calibration is in progress. This could
     * damage the device.
     */
    if (pDevObj->state & VP_DEV_IN_CAL) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Return NOT_INITIALIZED from Vp890CalLine()"));
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx,
            ("-Vp890CalLine()"));
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("Running Cal Line on Channel %d at time %d target 0x%02X 0x%02X",
        pLineObj->channelId, pDevObj->timeStamp,
        pLineObj->calLineData.dcFeedRef[0],
        pLineObj->calLineData.dcFeedRef[1]));

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Don't run calibration if previously complete on this line */
    if (pLineObj->calLineData.calDone == TRUE) {
        pLineObj->lineEvents.response |= VP_EVID_CAL_CMP;
        pLineObj->lineState.calType = VP_CSLAC_CAL_NONE;
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Calibration Previously Done. Cal Line Complete"));
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_SUCCESS;
    }

    /* Make sure line calibration can be run */
    switch(currentState) {
        case VP_LINE_OHT:
            break;
        case VP_LINE_STANDBY:
            pLineObj->lineState.calType =  VP_CSLAC_CAL_NONE;
            Vp890SetFxsLineState(pLineCtx, VP_LINE_OHT);
            break;
        case VP_LINE_STANDBY_POLREV:
        case VP_LINE_OHT_POLREV:
            pLineObj->lineState.calType =  VP_CSLAC_CAL_NONE;
            Vp890SetFxsLineState(pLineCtx, VP_LINE_OHT_POLREV);
            break;
        default:
            /* Mark the device as NOT in-calibration */
            pDevObj->state &= ~VP_DEV_IN_CAL;
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp890CalLine()"));
            return VP_STATUS_INVALID_ARG;
    }

    /* we need to remember if we started in polrev */
    if (pLineObj->slicValueCache & VP890_SS_POLARITY_MASK) {
        pLineObj->calLineData.reversePol = TRUE;
    } else {
        pLineObj->calLineData.reversePol = FALSE;
    }

#ifdef VP890_LP_SUPPORT
    /*
     * Force a LPM Exit update before proceeding with Calibration. If this
     * isn't done before setting the device mask, the LPM code to exit LPM will
     * not run (because it otherwise does not touch the device while in
     * calibration). If it is not a LPM device, this function doesn't do
     * anything.
     */
    Vp890LowPowerMode(pDevCtx);
#endif

    /* Mark the device as in-calibration */
    pDevObj->state |= VP_DEV_IN_CAL;

/* This delay could be reduced to LPM max, which is ~400ms. */
#define VP890_MIN_CAL_WAIT_TIME (500)

    pLineObj->lineTimers.timers.timer[VP_LINE_CAL_LINE_TIMER] =
        MS_TO_TICKRATE(VP890_MIN_CAL_WAIT_TIME, tickRate);
    pLineObj->lineTimers.timers.timer[VP_LINE_CAL_LINE_TIMER] |=
        VP_ACTIVATE_TIMER;


    /* Reprogram the Operating Conditions Register, affected by Set Line State */
    opCond[0] = (VP890_CUT_TXPATH | VP890_CUT_RXPATH | VP890_HIGH_PASS_DIS);
    /*opCond[0] = (VP890_CUT_RXPATH | VP890_HIGH_PASS_DIS);*/
    VpMpiCmdWrapper(deviceId, ecVal, VP890_OP_COND_WRT, VP890_OP_COND_LEN,
        opCond);
    pLineObj->opCond[0] = opCond[0];

    pLineObj->calLineData.calState =  VP890_CAL_SETUP;

    /*
     * Vp890ServiceFxsInterrupts uses this to decide if ring exit debounce
     * or CID need to be handled. Setting it to something other than
     * VP_CSLAC_CAL_NONE should be good enough.
     */
    pLineObj->lineState.calType = VP_CSLAC_CAL_ABV;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp890CalLine()"));

    return VP_STATUS_SUCCESS;
}
#endif /* (VP890_FXS_SUPPORT) && defined (VP_CSLAC_RUNTIME_CAL_ENABLED) */

#ifdef VP890_FXS_SUPPORT
/**
 * Vp890AdjustIla()
 *  This function adjusts the line object data for the adjusted ILA value. No
 * changes are made however if ILA calibration was not previously done.
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  If previous calibration done, return TRUE and adjust line object data.
 * Otherwise, return FALSE and no line object change made.
 */
bool
Vp890AdjustIla(
    VpLineCtxType *pLineCtx,
    uint8 targetIla)
{
    Vp890LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    int32 imtMeasured = 0;
    uint8 imtTarget;

    int16 ilaError, imtActual;
    uint8 ilaAdjust;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp890AdjustIla+"));

    /*
     * This function can run IF we're currently in calibration OR if the line
     * has been previously calibrated.
     */
    if ((pLineObj->calLineData.calDone == FALSE)    /* Line not previously calibrated */
    &&  (!(pDevObj->state & VP_DEV_IN_CAL))) {      /* Not currently in cal */
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp890AdjustIla-"));
        return FALSE;
    }

    if (targetIla < 4) {            /* 18mA to 22mA */
        imtMeasured = pDevObj->vp890SysCalData.ila20[channelId];
        imtTarget = 20;
    } else if (targetIla < 10) {    /* 23mA to 28mA */
        imtMeasured = pDevObj->vp890SysCalData.ila25[channelId];
        imtTarget = 25;
    } else if (targetIla < 18) {    /* 29mA to 36mA */
        imtMeasured = pDevObj->vp890SysCalData.ila32[channelId];
        imtTarget = 32;
    } else {                        /* 37mA and higher */
        imtMeasured = pDevObj->vp890SysCalData.ila40[channelId];
        imtTarget = 40;
    }

    /* Always force ref ILA into the the normal and rev polarity feeds */
    pLineObj->calLineData.dcFeed[VP890_ILA_INDEX] &= ~VP890_ILA_MASK;
    pLineObj->calLineData.dcFeed[VP890_ILA_INDEX] |=
        (pLineObj->calLineData.dcFeedRef[VP890_ILA_INDEX] & VP890_ILA_MASK);

    pLineObj->calLineData.dcFeedPr[VP890_ILA_INDEX] &= ~VP890_ILA_MASK;
    pLineObj->calLineData.dcFeedPr[VP890_ILA_INDEX] |=
        (pLineObj->calLineData.dcFeedRef[VP890_ILA_INDEX] & VP890_ILA_MASK);

    /* determine the error */
    imtActual = imtMeasured - pDevObj->vp890SysCalData.ilaOffsetNorm[channelId];
    ilaError = imtActual - (imtTarget * VP890_ILA_SCALE_1MA);

    /*
     * only make an adjustment only if the error is greater
     * than or equal to 500uA = 273 at PCM
     */
    if (ABS(ilaError) >= (VP890_ILA_SCALE_1MA / 2)) {

        uint8 tempIlaValue = pLineObj->calLineData.dcFeedRef[VP890_ILA_INDEX] & VP890_ILA_MASK;
        int8 tempLowValue = (int8)tempIlaValue;
        ilaAdjust = ((ABS(ilaError)+(VP890_ILA_SCALE_1MA / 2)) / VP890_ILA_SCALE_1MA);

        if (ilaError < 0) {
            tempIlaValue += ilaAdjust;
            if (tempIlaValue <= VP890_ILA_MASK) {
                pLineObj->calLineData.dcFeed[VP890_ILA_INDEX] += ilaAdjust;
            } else {
                pLineObj->calLineData.dcFeed[VP890_ILA_INDEX] |= VP890_ILA_MASK;
            }
        } else {
            tempLowValue -= ilaAdjust;
            if (tempLowValue >= 0) {
                pLineObj->calLineData.dcFeed[VP890_ILA_INDEX] -= ilaAdjust;
            } else {
                pLineObj->calLineData.dcFeed[VP890_ILA_INDEX] &= ~VP890_ILA_MASK;
            }
        }
    }

    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Chan %d: ILA Actual Norm (10uA) %d",
        channelId,
        (int16)(imtActual * 100 / VP890_ILA_SCALE_1MA)));

    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Chan %d: ILA Target (10uA) %d ILA Error Norm (10uA) %d",
        channelId,
        (int16)(((targetIla + 18) * 100)),
        (int16)(ilaError * 100L / VP890_ILA_SCALE_1MA)));

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp890AdjustIla-"));
    return TRUE;
}

/**
 * Vp890AdjustVoc()
 *  This function adjusts the line object data for the adjusted VOC value. No
 * changes are made however if VOC calibration was not previously done.
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  If previous calibration done, return TRUE and adjust line object data.
 * Otherwise, return FALSE and no line object change made.
 */
bool
Vp890AdjustVoc(
    VpLineCtxType *pLineCtx,
    uint8 targetVoc,
    bool previousCal)
{
    Vp890LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    int16 vocActual, vocActualRev;
    int32 pcmTargetVoc;

    int16 vocErrorList[VP890_NUM_POLARITY];
    uint8 vocErrIndex;

    uint8 *dcFeedByte[VP890_NUM_POLARITY];

    dcFeedByte[VP890_NORM_POLARITY] = &(pLineObj->calLineData.dcFeed[0]);
    dcFeedByte[VP890_REV_POLARITY] = &(pLineObj->calLineData.dcFeedPr[0]);

    pcmTargetVoc = (int32)(targetVoc * 3);
    pcmTargetVoc += 36;
    pcmTargetVoc *= VP890_V_1V_SCALE;
    pcmTargetVoc /= VP890_V_1V_RANGE;

    if (previousCal == FALSE) {
        vocActual =
            pLineObj->calLineData.typeData.vocData.vocNorm
          - pDevObj->vp890SysCalData.vocOffset[channelId][VP890_NORM_POLARITY];

        vocActualRev =
            pLineObj->calLineData.typeData.vocData.vocRev
          - pDevObj->vp890SysCalData.vocOffset[channelId][VP890_REV_POLARITY];

        /*
         * Target is always positive. Normal feed is positive. Negative error means
         * voltage is too low (magnitude), positive means too high (magnitude).
         */
        pDevObj->vp890SysCalData.vocError[channelId][VP890_NORM_POLARITY] = (vocActual - (int16)pcmTargetVoc);

        VP_CALIBRATION(VpLineCtxType, pLineCtx, ("(In 10mV): VOC Norm %d Rev %d OffsetNorm %d OffsetRev %d Channel %d",
            (int16)(pLineObj->calLineData.typeData.vocData.vocNorm  * VP890_V_PCM_LSB/VP890_V_SCALE),
            (int16)(pLineObj->calLineData.typeData.vocData.vocRev  * VP890_V_PCM_LSB/VP890_V_SCALE),
            (int16)(pDevObj->vp890SysCalData.vocOffset[channelId][VP890_NORM_POLARITY]  * VP890_V_PCM_LSB/VP890_V_SCALE),
            (int16)(pDevObj->vp890SysCalData.vocOffset[channelId][VP890_REV_POLARITY]  * VP890_V_PCM_LSB/VP890_V_SCALE),
            channelId));

        VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Chan %d: VOC (10mV) Actual Norm Feed %d Rev Feed %d",
            channelId,
            (int16)(vocActual  * VP890_V_PCM_LSB/VP890_V_SCALE),
            (int16)(vocActualRev  * VP890_V_PCM_LSB/VP890_V_SCALE)));

        /*
         * Target is always positive. Reverse feed is negative. Negative error means
         * voltage is too low (magnitude), positive means too high (magnitude).
         */
        pDevObj->vp890SysCalData.vocError[channelId][VP890_REV_POLARITY] = (-vocActualRev - (int16)pcmTargetVoc);

        VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Chan %d: VOC Target %d VOC Error Norm %d Error Rev %d",
            channelId,
            (int16)((targetVoc * 3 + 36) * 100),
            (int16)(pDevObj->vp890SysCalData.vocError[channelId][VP890_NORM_POLARITY]  * VP890_V_PCM_LSB/VP890_V_SCALE),
            (int16)(pDevObj->vp890SysCalData.vocError[channelId][VP890_REV_POLARITY]  * VP890_V_PCM_LSB/VP890_V_SCALE)));

        VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Chan %d: DC Feed Values Normal Before 0x%02X 0x%02X",
            channelId, pLineObj->calLineData.dcFeed[0],
            pLineObj->calLineData.dcFeed[1]));

        VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Chan %d: DC Feed Values Reverse Before 0x%02X 0x%02X",
            channelId, pLineObj->calLineData.dcFeedPr[0],
            pLineObj->calLineData.dcFeedPr[1]));
    }

    /*
     * Adjust if error is more than 1/2 a step size for each parameter based
     * on PCM scale.
     */
    vocErrorList[VP890_NORM_POLARITY] = pDevObj->vp890SysCalData.vocError[channelId][VP890_NORM_POLARITY];
    vocErrorList[VP890_REV_POLARITY] = pDevObj->vp890SysCalData.vocError[channelId][VP890_REV_POLARITY];

    for (vocErrIndex = 0; vocErrIndex < VP890_NUM_POLARITY; vocErrIndex++) {
        /* VOC Scale: 1.5V = 204.8 at PCM. Adjust to account for bit shift */

        if (ABS(vocErrorList[vocErrIndex]) >= 205) {
            if (vocErrorList[vocErrIndex] < 0) {
                /* Error is low, so need to increase VOC */

                /* Saturate the value, to prevent the rollover */
                if ((*dcFeedByte[vocErrIndex] & VP890_VOC_MASK) !=
                    VP890_VOC_MASK) {

                    /* Not saturated within scale. So can adjust up */
                    *dcFeedByte[vocErrIndex] += 0x04;

                } else if ((*dcFeedByte[vocErrIndex] & VP890_VOC_LOW_RANGE) ==
                    VP890_VOC_LOW_RANGE) {

                    /*
                     * Saturated within scale, but not within device. Change
                     * scale (moves up 3V) and clear incremental values or we'll
                     *  end up at the top of the high range.
                     */
                    *dcFeedByte[vocErrIndex] &= ~VP890_VOC_MASK;
                    *dcFeedByte[vocErrIndex] &= ~VP890_VOC_LOW_RANGE;
                }
            } else {
                /* Error is high, so need to decrease VOC */

                /* Saturate the value, to prevent the rollover */
                if ((*dcFeedByte[vocErrIndex] & VP890_VOC_MASK) != 0x00) {
                    /* Not saturated within scale. So can adjust down */
                    *dcFeedByte[vocErrIndex] -= 0x04;
                } else if ((*dcFeedByte[vocErrIndex] & VP890_VOC_LOW_RANGE) !=
                    VP890_VOC_LOW_RANGE) {

                    /*
                     * Saturated within scale, but not within device. Change
                     * scale (moves down 3V) and max incremental values or we'll
                     *  end up at the bottom of the low range.
                     */
                    *dcFeedByte[vocErrIndex] |= VP890_VOC_MASK;
                    *dcFeedByte[vocErrIndex] |= VP890_VOC_LOW_RANGE;
                }
            }
        }
    }

    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Chan %d: DC Feed Values Normal After 0x%02X 0x%02X",
        channelId, pLineObj->calLineData.dcFeed[0],
        pLineObj->calLineData.dcFeed[1]));

    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Chan %d: DC Feed Values Reverse After 0x%02X 0x%02X",
        channelId, pLineObj->calLineData.dcFeedPr[0],
        pLineObj->calLineData.dcFeedPr[1]));

    return TRUE;
}

/**
 * Vp890BatteryAdjust()
 *  This function programs the device battery calibration based on either
 * calibrated or provided error value.
 */
void
Vp890BatteryAdjust(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 swCal[VP890_BAT_CALIBRATION_LEN];
    uint16 swCalError;
    int32 abvError =
        (pDevObj->vp890SysCalData.abvError[0] * VP890_V_SCALE) / VP890_V_PCM_LSB;

    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Chan 0 Voltage Error: SWY %d",
        (int16)((abvError * 7324L) / VP890_V_SCALE)));

    /* Write the correction value to CH1 register. Steps in 1.25V increment */
    VpMpiCmdWrapper(deviceId, VP890_EC_CH1, VP890_BAT_CALIBRATION_RD,
        VP890_BAT_CALIBRATION_LEN, swCal);
    swCal[0] &= ~(VP890_BAT_CAL_SWCAL_MASK);

    /* Conversion from 7.324mV to 1.25V */
    swCalError = (ABS(abvError) / VP890_ABV_STEP);
    if (((ABS(abvError) + (VP890_ABV_STEP / 2)) /  VP890_ABV_STEP) > swCalError) {
        swCalError+=1;
    }
    swCalError = (swCalError > 3) ? 3 : swCalError;
    swCal[0] |= (swCalError << 3);

    /*
     * Positive error means voltage is too low (not negative enough). Positive
     * adjustment makes the battery voltage more negative.
     */
    swCal[0] |= (abvError > 0) ? 0 : VP890_BAT_CAL_SWCAL_SIGN;

    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Ch 0: Battery Calibration Correction 0x%02X 0x%02X",
        swCal[0], swCal[1]));

    VpMpiCmdWrapper(deviceId, VP890_EC_CH1, VP890_BAT_CALIBRATION_WRT,
        VP890_BAT_CALIBRATION_LEN, swCal);
    return;
}
#endif  /* VP890_FXS_SUPPORT */

/**
 * Vp890Cal()
 *  This function calibrates a selected block of the device/line.
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action.
 */
VpStatusType
Vp890Cal(
    VpLineCtxType       *pLineCtx,
    VpCalType           calType,
    void                *inputArgs)
{
#ifdef VP890_FXS_SUPPORT
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp890LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 profileIndex;
    uint8 *profileData;
#endif

    switch (calType) {
#ifdef VP890_FXS_SUPPORT
        case VP_CAL_GET_SYSTEM_COEFF:
            VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
            if (pDevObj->stateInt & VP890_SYS_CAL_COMPLETE) {
                /* Data length is header (6 bytes) +  890 calibration data */
                pDevObj->mpiLen = (6 + VP890_CAL_STRUCT_SIZE);
                VP_CALIBRATION(VpLineCtxType, pLineCtx,
                    ("Calibration Data Length %d", pDevObj->mpiLen));

                pLineObj->responseData = (uint8)VP_CAL_GET_SYSTEM_COEFF;
                pLineObj->lineEvents.response |= VP_EVID_CAL_CMP;
            } else {
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_LINE_NOT_CONFIG;
            }
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            return VP_STATUS_SUCCESS;

        case VP_CAL_APPLY_SYSTEM_COEFF:
            VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
            profileData = (uint8 *)inputArgs;

            /*
             * Make sure this type of event/condition can only be generated from
             * an FXS line.
             */
            if (pLineObj->status & VP890_IS_FXO) {
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_INVALID_ARG;
            }

            if (profileData == VP_NULL) {
                VpMemSet(&pDevObj->vp890SysCalData, 0, sizeof(Vp890SysCalResultsType));

                pDevObj->stateInt &= ~(VP890_SYS_CAL_COMPLETE | VP890_CAL_RELOAD_REQ);
                pLineObj->lineEvents.response |= VP_EVID_CAL_CMP;
                pLineObj->calLineData.calDone = FALSE;
                pLineObj->responseData = VP_CAL_SUCCESS;
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_SUCCESS;
            }

            if ((profileData[VP_PROFILE_TYPE_LSB] != VP_PRFWZ_PROFILE_CAL) ||
                (profileData[VP_PROFILE_TYPE_MSB] != VP_DEV_890_SERIES)) {
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_INVALID_ARG;
            }

            if (profileData[VP_PROFILE_LENGTH] < (VP890_CAL_STRUCT_SIZE + 2)) {
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_INVALID_ARG;
            }

            profileIndex = VP_PROFILE_DATA_START;

            pDevObj->vp890SysCalData.abvError[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ABV Y Error %d", pDevObj->vp890SysCalData.abvError[0]));

            pDevObj->vp890SysCalData.vocOffset[0][0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Norm Ch 0 Offset %d", pDevObj->vp890SysCalData.vocOffset[0][0]));

            pDevObj->vp890SysCalData.vocError[0][0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Norm Ch 0 Error %d", pDevObj->vp890SysCalData.vocError[0][0]));

            pDevObj->vp890SysCalData.vocOffset[0][1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Rev Ch 0 Offset %d", pDevObj->vp890SysCalData.vocOffset[0][1]));

            pDevObj->vp890SysCalData.vocError[0][1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Rev Ch 0 Error %d", pDevObj->vp890SysCalData.vocError[0][1]));

            pDevObj->vp890SysCalData.sigGenAError[0][0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SigGenA Norm Ch 0 Error %d", pDevObj->vp890SysCalData.sigGenAError[0][0]));

            pDevObj->vp890SysCalData.sigGenAError[0][1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SigGenA Rev Ch 0 Error %d", pDevObj->vp890SysCalData.sigGenAError[0][1]));

            pDevObj->vp890SysCalData.ila20[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA 20mA Ch 0 %d", pDevObj->vp890SysCalData.ila20[0]));

            pDevObj->vp890SysCalData.ila25[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA 25mA Ch 0 %d", pDevObj->vp890SysCalData.ila25[0]));

            pDevObj->vp890SysCalData.ila32[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA 32mA Ch 0 %d", pDevObj->vp890SysCalData.ila32[0]));

            pDevObj->vp890SysCalData.ila40[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA 40mA Ch 0 %d", pDevObj->vp890SysCalData.ila40[0]));

            pDevObj->vp890SysCalData.ilaOffsetNorm[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA Offset Norm Ch 0 %d", pDevObj->vp890SysCalData.ilaOffsetNorm[0]));

            pDevObj->vp890SysCalData.ilgOffsetNorm[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILG Offset Norm Ch 0 %d", pDevObj->vp890SysCalData.ilgOffsetNorm[0]));

            pDevObj->vp890SysCalData.vas[0][0] = profileData[profileIndex++];
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAS Norm Ch 0 %d", pDevObj->vp890SysCalData.vas[0][0]));

            pDevObj->vp890SysCalData.vas[0][1] = profileData[profileIndex++];
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAS Rev Ch 0 %d", pDevObj->vp890SysCalData.vas[0][1]));

            pDevObj->vp890SysCalData.vagOffsetNorm[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAG Offset Norm Ch 0 %d", pDevObj->vp890SysCalData.vagOffsetNorm[0]));

            pDevObj->vp890SysCalData.vagOffsetRev[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAG Offset Rev Ch 0 %d", pDevObj->vp890SysCalData.vagOffsetRev[0]));

            pDevObj->vp890SysCalData.vbgOffsetNorm[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VBG Offset Norm Ch 0 %d", pDevObj->vp890SysCalData.vbgOffsetNorm[0]));

            pDevObj->vp890SysCalData.vbgOffsetRev[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VBG Offset Rev Ch 0 %d", pDevObj->vp890SysCalData.vbgOffsetRev[0]));

            pDevObj->vp890SysCalData.swyOffset[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SWY Offset Ch 0 %d", pDevObj->vp890SysCalData.swyOffset[0]));

            pDevObj->vp890SysCalData.tipCapCal[0] = VpConvertToInt32(&profileData[profileIndex]);
            profileIndex+=4;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Tip Cap Ch 0 %li", pDevObj->vp890SysCalData.tipCapCal[0]));

            pDevObj->vp890SysCalData.ringCapCal[0] = VpConvertToInt32(&profileData[profileIndex]);
            profileIndex+=4;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Ring Cap Ch 0 %li", pDevObj->vp890SysCalData.ringCapCal[0]));

            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Calibration Data Length - %d", profileIndex));

            pDevObj->stateInt |= (VP890_SYS_CAL_COMPLETE | VP890_CAL_RELOAD_REQ);
            pLineObj->lineEvents.response |= VP_EVID_CAL_CMP;
            pLineObj->calLineData.calDone = TRUE;
            pLineObj->responseData = VP_CAL_SUCCESS;
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
           return VP_STATUS_SUCCESS;
#endif

#ifdef VP890_FXO_SUPPORT
        case VP_CAL_BFILTER:
            return Vp890CalBFilter(pLineCtx, inputArgs);

        case VP_CAL_APPLY_BFILTER:
            return Vp890CalApplyBFilter(pLineCtx, inputArgs);

        case VP_CAL_MEASURE_BFILTER:
            return Vp890CalMeasureBFilter(pLineCtx);
#endif

        default:
            return VP_STATUS_INVALID_ARG;
    }
}

#ifdef VP890_FXO_SUPPORT
/**
 * Vp890CalBFilter()
 *  This function starts the B-Filter adaptive balance calibration.
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action.
 */
static VpStatusType
Vp890CalBFilter(
    VpLineCtxType       *pLineCtx,
    void                *inputArgs)
{
    Vp890LineObjectType *pLineObj   = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx           = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj  = pDevCtx->pDevObj;
    VpDeviceIdType deviceId         = pDevObj->deviceId;
    uint8 ecVal                     = pLineObj->ecVal;
    uint8 mpiLen;
    VpProfilePtrType pAcProfile;
    uint16 tempGxCidLevel;
    uint16 tempGxUserLevel;
    uint16 tempGrUserLevel;
    VpStatusType vpStatus;

    uint8 mpiBuffer[3 + VP890_OP_FUNC_LEN + VP890_OP_COND_LEN + VP890_CONV_CFG_LEN];
    uint8 mpiIndex = 0;

    Vp890CalBFilterData *bFilterData = &pLineObj->calLineData.typeData.bFilterData;

    uint8 convCfg[VP890_CONV_CFG_LEN] = {
        VP890_ENH_B_FILTER_AVG_DET
    };

    uint8 opFunctions[VP890_OP_FUNC_LEN] = {
        VP890_ENABLE_LOADED_COEFFICIENTS | VP890_LINEAR_CODEC
    };

    uint8 opCond[VP890_OP_COND_LEN] = {
        VP890_TXPATH_DIS | VP890_NOISE_GEN_EN
    };

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890CalBFilter()"));

    if (inputArgs == VP_NULL) {
        return VP_STATUS_INVALID_ARG;
    }

    if ( ((VpCalBFilterType *)inputArgs)->pAcProfile == VP_NULL ) {
        return VP_STATUS_INVALID_ARG;
    }

    /* Check the legality of the AC profile and look up the profile table
     * entry if necessary */
    if (!VpCSLACIsProfileValid(VP_PROFILE_AC,
            VP_CSLAC_AC_PROF_TABLE_SIZE, pDevObj->profEntry.acProfEntry,
            pDevObj->devProfileTable.pAcProfileTable,
            ((VpCalBFilterType *)inputArgs)->pAcProfile,
            &pAcProfile)) {

        return VP_STATUS_ERR_PROFILE;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    pLineObj->status |= VP890_LINE_IN_CAL;

    /*
     * Copy input parameters to internal line obect. Used to "continue" running
     * calibration after this function returns
     */
    bFilterData->inputData = *(VpCalBFilterType *)inputArgs;
    bFilterData->inputData.pAcProfile = pAcProfile;

    VP_CALIBRATION(VpLineCtxType, pLineCtx,("Profile check: %p, %02X %02X %02X %02X %02X %02X",
        bFilterData->inputData.pAcProfile,
        bFilterData->inputData.pAcProfile[0],
        bFilterData->inputData.pAcProfile[1],
        bFilterData->inputData.pAcProfile[2],
        bFilterData->inputData.pAcProfile[3],
        bFilterData->inputData.pAcProfile[4],
        bFilterData->inputData.pAcProfile[5]));

    /*
     * Initialize vRms starting point to cause initial loading during
     * measurement and comparison phase
     */
    bFilterData->vRms = 0xFFFF;
    bFilterData->currentSet = 0;
    bFilterData->bestSet = 0;

    /* The MPI length field in this profile describes the length of
     * each entry, so (Length)/(length per entry) = number of entries.
     * Length is (profile_length - 2) because profile length includes the
     * version and MPI_LEN fields */
    mpiLen = pAcProfile[VP_PROFILE_MPI_LEN];

    if (VP_PROFILE_MPI_LEN + mpiLen + 1 >= pAcProfile[VP_PROFILE_LENGTH]) {
        /* This profile contains no additional B-filter sets */
        VP_ERROR(VpLineCtxType, pLineCtx, ("AC profile does not contain extra B-filter sets (based on profile length)"));
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_ERR_PROFILE;
    }

    bFilterData->listLength = pAcProfile[VP_PROFILE_MPI_LEN + mpiLen + 1];

    if (bFilterData->listLength == 0) {
        /* This profile contains no additional B-filter sets */
        VP_ERROR(VpLineCtxType, pLineCtx, ("AC profile does not contain extra B-filter sets (based on list length byte)"));
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_ERR_PROFILE;
    }

    VP_CALIBRATION(VpLineCtxType, pLineCtx,("Vp890CalBFilter() - listLength: %d  target: %d   TS: %d",
        bFilterData->listLength,
        bFilterData->inputData.vRms,
        pDevObj->timeStamp));

    /* The first set of coefficients to try are in the main body of the AC
     * profile (index 0).  Use ConfigLine to apply them, and restore relative
     * gain settings afterward. */
    tempGxUserLevel = pLineObj->gxUserLevel;
    tempGxCidLevel = pLineObj->gxCidLevel;
    tempGrUserLevel = pLineObj->grUserLevel;

    vpStatus = Vp890ConfigLine(pLineCtx, pAcProfile, VP_PTABLE_NULL,
        VP_PTABLE_NULL);
    if (vpStatus != VP_STATUS_SUCCESS) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890CalBFilter() - ConfigLine returned %d", vpStatus));
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return vpStatus;
    }

    pLineObj->gxUserLevel = tempGxUserLevel;
    pLineObj->gxCidLevel = tempGxCidLevel;
    pLineObj->grUserLevel = tempGrUserLevel;
    vpStatus = Vp890SetRelGainInt(pLineCtx);
    if (vpStatus != VP_STATUS_SUCCESS) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890CalBFilter() - Vp890SetRelGainInt returned %d", vpStatus));
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return vpStatus;
    }

    /* Save off current register content that will be modified */
    VpMpiCmdWrapper(deviceId, ecVal, VP890_OP_FUNC_RD, VP890_OP_FUNC_LEN,
        bFilterData->opFunct);

    bFilterData->opCond[0] = pLineObj->opCond[0];

    VpMpiCmdWrapper(deviceId, ecVal, VP890_CONV_CFG_RD, VP890_CONV_CFG_LEN,
        bFilterData->convCfg);
    VpMpiCmdWrapper(deviceId, ecVal, VP890_VP_GAIN_RD, VP890_VP_GAIN_LEN,
        bFilterData->vpGain);

#ifdef VP890_REDUCE_BFILTER_CAL_SIGNAL_LEVEL
    Vp890ReduceNoiseOutput(bFilterData->vpGain[0], deviceId, ecVal);
#endif

    /* Program device to measure B-Filter levels and start noise generator */
    VP_CALIBRATION(VpLineCtxType, pLineCtx,("\n\r5. Writing 0x%02X to Operating Functions",
        opFunctions[0]));
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_OP_FUNC_WRT,
        VP890_OP_FUNC_LEN, opFunctions);

    VP_CALIBRATION(VpLineCtxType, pLineCtx,("\n\r7. Writing 0x%02X to Operating Conditions",
        opCond[0]));
    pLineObj->opCond[0] = opCond[0];
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_OP_COND_WRT,
        VP890_OP_COND_LEN, opCond);

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_CONV_CFG_WRT,
        VP890_CONV_CFG_LEN, convCfg);

    VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0],
        mpiIndex-1, &mpiBuffer[1]);

    /* Set timer for measurement */
    pLineObj->lineTimers.timers.fxoTimer.bCalTimer = 0;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
}

/**
 * Vp890CalBFilterInt()
 *  This function calibrates a selected block of the device/line.
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action.
 */
VpStatusType
Vp890CalBFilterInt(
    VpLineCtxType       *pLineCtx)
{
    Vp890LineObjectType *pLineObj   = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx           = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj  = pDevCtx->pDevObj;
    VpDeviceIdType deviceId         = pDevObj->deviceId;
    uint8 ecVal                     = pLineObj->ecVal;

    uint8 mpiIndex                  = 0;
    uint8 mpiBuffer[6 + VP890_OP_FUNC_LEN + VP890_OP_COND_LEN +
                        VP890_CONV_CFG_LEN + VP890_VP_GAIN_LEN +
                        VP890_B1_FILTER_LEN + VP890_B2_FILTER_LEN];

    Vp890CalBFilterData *bFilterData = &pLineObj->calLineData.typeData.bFilterData;
    uint8 bFilterResult[VP890_TX_PCM_DATA_LEN];
    uint16 vRms;

    uint8 convCfg[VP890_CONV_CFG_LEN] = {
        VP890_ENH_B_FILTER_AVG_DET
    };

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890CalBFilterInt()"));

    /* Read the current B-Filter Measured Data */
    VpMpiCmdWrapper(deviceId, ecVal, VP890_TX_PCM_DATA_RD, VP890_TX_PCM_DATA_LEN,
        bFilterResult);
    vRms = (bFilterResult[0] << 8) | bFilterResult[1];

    VP_CALIBRATION(VpLineCtxType, pLineCtx,("Vp890CalBFilterInt() - Set: %d    vRms: %d    TS: %d",
        bFilterData->currentSet, vRms, pDevObj->timeStamp));

    VP_CALIBRATION(VpLineCtxType, pLineCtx,("Profile check: %p, %02X %02X %02X %02X %02X %02X",
        bFilterData->inputData.pAcProfile,
        bFilterData->inputData.pAcProfile[0],
        bFilterData->inputData.pAcProfile[1],
        bFilterData->inputData.pAcProfile[2],
        bFilterData->inputData.pAcProfile[3],
        bFilterData->inputData.pAcProfile[4],
        bFilterData->inputData.pAcProfile[5]));

    /*
     * Compare it with "best known" value. If better, the current known value,
     * save the current B-Filter Coefficients and value.  Always save the first
     * set.
     */

    if (vRms < bFilterData->vRms || bFilterData->currentSet == 0) {
        bFilterData->vRms = vRms;

        /* Copy the current "better" set into the calibration data */
        VpMpiCmdWrapper(deviceId, ecVal, VP890_B1_FILTER_RD, VP890_B1_FILTER_LEN,
            bFilterData->b1FiltData);
        VpMpiCmdWrapper(deviceId, ecVal, VP890_B2_FILTER_RD, VP890_B2_FILTER_LEN,
            bFilterData->b2FiltData);
        VP_CALIBRATION(VpLineCtxType, pLineCtx,("              - BEST"));

        bFilterData->bestSet = bFilterData->currentSet;
    }

    /* NOTE:  Since index 0 refers to the coefficients in the main body of the
     * profile, the additional sets at the end start at index 1 */

    /*
     * Stop if the current measured data meets the customer requirements, OR
     * if all sets have been tested. Otherwise, repeat with the next set.
     */
    if ((bFilterData->vRms < bFilterData->inputData.vRms)
     || (bFilterData->currentSet >= bFilterData->listLength)) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,("\n\r6. Writing 0x%02X to Operating Functions",
            bFilterData->opFunct[0]));

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_OP_FUNC_WRT,
            VP890_OP_FUNC_LEN, bFilterData->opFunct);

        VP_CALIBRATION(VpLineCtxType, pLineCtx,("\n\r8. Writing 0x%02X to Operating Conditions",
            bFilterData->opCond[0]));
        pLineObj->opCond[0] = bFilterData->opCond[0];
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_OP_COND_WRT,
            VP890_OP_COND_LEN, bFilterData->opCond);

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_CONV_CFG_WRT,
            VP890_CONV_CFG_LEN, bFilterData->convCfg);

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_VP_GAIN_WRT,
            VP890_VP_GAIN_LEN, bFilterData->vpGain);

        VP_CALIBRATION(VpLineCtxType, pLineCtx,("Vp890CalBFilterInt() - Done"));

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_B1_FILTER_WRT,
            VP890_B1_FILTER_LEN, bFilterData->b1FiltData);

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_B2_FILTER_WRT,
            VP890_B2_FILTER_LEN, bFilterData->b2FiltData);

        VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0],
            mpiIndex-1, &mpiBuffer[1]);

        pLineObj->lineEvents.response |= VP_EVID_CAL_CMP;
        pLineObj->lineEventHandle = (uint16)(bFilterData->bestSet);
        pLineObj->responseData = bFilterData->vRms;

        VP_CALIBRATION(VpLineCtxType, pLineCtx,("Vp890CalBFilterInt() - Done"));
        VP_CALIBRATION(VpLineCtxType, pLineCtx,("bFilterData->vRms: %d\nbFilterData->inputData.vRms: %d\nbFilterData->currentSet: %d\nbFilterData->listLength: %d",bFilterData->vRms,bFilterData->inputData.vRms,bFilterData->currentSet,bFilterData->listLength));
        VP_CALIBRATION(VpLineCtxType, pLineCtx,("lineEvents.response = %04X", pLineObj->lineEvents.response));
        VP_CALIBRATION(VpLineCtxType, pLineCtx,("lineEventHandle = %04X", pLineObj->lineEventHandle));
        VP_CALIBRATION(VpLineCtxType, pLineCtx,("responseData = %04X", pLineObj->responseData));

        pLineObj->status &= ~VP890_LINE_IN_CAL;
    } else {
        /* Load the next set of coefficients and start the measurement */
        VpProfileDataType   *pMpiData;
        uint8               profileIndex;
        uint8               firstEntry;
        uint8               bFilterLength;
        uint8               filterDataCnt;

        bFilterData->currentSet++;

        firstEntry =
            (VP_PROFILE_MPI_LEN + bFilterData->inputData.pAcProfile[VP_PROFILE_MPI_LEN] + 3);

        bFilterLength = bFilterData->inputData.pAcProfile[firstEntry - 1];

        profileIndex = firstEntry + ((bFilterData->currentSet - 1) * bFilterLength);

        pMpiData = (VpProfileDataType *)(&bFilterData->inputData.pAcProfile[profileIndex]);

        for (filterDataCnt = 0; filterDataCnt < bFilterLength; filterDataCnt++) {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,("0x%02X", pMpiData[filterDataCnt]));
        }

        /* Program this set of coefficients */
        VpMpiCmdWrapper(deviceId, ecVal, NOOP_CMD, bFilterLength, pMpiData);

        /* Make sure the converter configuration is correct */
        VpMpiCmdWrapper(deviceId, ecVal, VP890_CONV_CFG_WRT, VP890_CONV_CFG_LEN,
            convCfg);

        /* Timer is set to 65ms+ so that we can be sure the next measurement
         * will be a full 32ms taken with this set of coefficients. */
        pLineObj->lineTimers.timers.fxoTimer.bCalTimer = 0;
        VP_CALIBRATION(VpLineCtxType, pLineCtx,("Vp890CalBFilterInt() - Setting timer for set %d",
            bFilterData->currentSet));
    }

    return VP_STATUS_SUCCESS;
}

static VpStatusType
Vp890CalApplyBFilter(
    VpLineCtxType       *pLineCtx,
    void                *inputArgs)
{
    Vp890LineObjectType *pLineObj   = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx           = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj  = pDevCtx->pDevObj;
    VpDeviceIdType deviceId         = pDevObj->deviceId;
    uint8 ecVal                     = pLineObj->ecVal;
    VpProfilePtrType pAcProfile;
    uint16 index;
    VpProfileDataType *pMpiData;
    uint8 mpiLen;
    uint8 listLength;
    uint8 bFilterLength;
    uint8 firstEntry;
    uint8 bFilterSetIndex;
    uint8 filterDataCnt;
    uint16 tempGxCidLevel;
    uint16 tempGxUserLevel;
    uint16 tempGrUserLevel;
    VpStatusType vpStatus;

    index = ((VpCalApplyBFilterType *)inputArgs)->index;
    pAcProfile = ((VpCalApplyBFilterType *)inputArgs)->pAcProfile;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890CalMeasureBFilter()"));

    if (pAcProfile == VP_NULL) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Invalid NULL AC profile"));
        return VP_STATUS_ERR_PROFILE;
    }

    if (index > 0xFF) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Invalid index %d", index));
        return VP_STATUS_INVALID_ARG;
    }

    /* Check the legality of the AC profile and look up the profile table
     * entry if necessary */
    if (!VpCSLACIsProfileValid(VP_PROFILE_AC,
            VP_CSLAC_AC_PROF_TABLE_SIZE, pDevObj->profEntry.acProfEntry,
            pDevObj->devProfileTable.pAcProfileTable,
            ((VpCalApplyBFilterType *)inputArgs)->pAcProfile,
            &pAcProfile)) {

        return VP_STATUS_ERR_PROFILE;
    }

    if (index == 0) {
        /* Index 0 refers to the coefficients in the main body of the AC
         * profile.  Use ConfigLine to apply them, and restore relative
         * gain settings afterward. */
        tempGxUserLevel = pLineObj->gxUserLevel;
        tempGxCidLevel = pLineObj->gxCidLevel;
        tempGrUserLevel = pLineObj->grUserLevel;

        vpStatus = Vp890ConfigLine(pLineCtx, pAcProfile, VP_PTABLE_NULL,
            VP_PTABLE_NULL);
        if (vpStatus != VP_STATUS_SUCCESS) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890CalApplyBFilter() - ConfigLine returned %d", vpStatus));
            return vpStatus;
        }

        pLineObj->gxUserLevel = tempGxUserLevel;
        pLineObj->gxCidLevel = tempGxCidLevel;
        pLineObj->grUserLevel = tempGrUserLevel;
        vpStatus = Vp890SetRelGainInt(pLineCtx);
        if (vpStatus != VP_STATUS_SUCCESS) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890CalApplyBFilter() - Vp890SetRelGainInt returned %d", vpStatus));
            return vpStatus;
        }

        pLineObj->lineEvents.response |= VP_EVID_CAL_CMP;

        return VP_STATUS_SUCCESS;
    }

    VP_CALIBRATION(VpLineCtxType, pLineCtx,("Vp890CalApplyBFilter Profile check: %p, %02X %02X %02X %02X %02X %02X",
        pAcProfile,
        pAcProfile[0],
        pAcProfile[1],
        pAcProfile[2],
        pAcProfile[3],
        pAcProfile[4],
        pAcProfile[5]));

    mpiLen = pAcProfile[VP_PROFILE_MPI_LEN];

    if (VP_PROFILE_MPI_LEN + mpiLen + 1 >= pAcProfile[VP_PROFILE_LENGTH]) {
        /* This profile contains no additional B-filter sets */
        VP_ERROR(VpLineCtxType, pLineCtx,
            ("AC profile does not contain extra B-filter sets (based on profile length). %d >= %d",
            VP_PROFILE_MPI_LEN + mpiLen + 1, pAcProfile[VP_PROFILE_LENGTH]));
        return VP_STATUS_ERR_PROFILE;
    }

    listLength = pAcProfile[VP_PROFILE_MPI_LEN + mpiLen + 1];

    /* NOTE:  Since index==0 refers to the coefficients in the main body of the
     * profile, the additional sets at the end start at index 1 */

    if (index > listLength) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("index too high.  index %d, listLength %d", index, listLength));
        return VP_STATUS_INVALID_ARG;
    }

    bFilterLength = pAcProfile[VP_PROFILE_MPI_LEN + mpiLen + 2];

    firstEntry = VP_PROFILE_MPI_LEN + pAcProfile[VP_PROFILE_MPI_LEN] + 3;

    bFilterSetIndex = firstEntry + ((index - 1) * bFilterLength);

    pMpiData = (VpProfileDataType *)(&pAcProfile[bFilterSetIndex]);

    for (filterDataCnt = 0; filterDataCnt < bFilterLength; filterDataCnt++) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,("0x%02X", pMpiData[filterDataCnt]));
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Program this set of coefficients */
    VpMpiCmdWrapper(deviceId, ecVal, NOOP_CMD, bFilterLength, pMpiData);

    pLineObj->lineEvents.response |= VP_EVID_CAL_CMP;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
}

static VpStatusType
Vp890CalMeasureBFilter(
    VpLineCtxType       *pLineCtx)
{
    Vp890LineObjectType *pLineObj   = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx           = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj  = pDevCtx->pDevObj;
    VpDeviceIdType deviceId         = pDevObj->deviceId;
    uint8 ecVal                     = pLineObj->ecVal;

    uint8 mpiIndex                  = 0;
    uint8 mpiBuffer[3 + VP890_OP_FUNC_LEN + VP890_OP_COND_LEN + VP890_CONV_CFG_LEN];

    Vp890CalBFilterData *bFilterData = &pLineObj->calLineData.typeData.bFilterData;

    uint8 convCfg[VP890_CONV_CFG_LEN] = {
        VP890_ENH_B_FILTER_AVG_DET
    };

    uint8 opFunctions[VP890_OP_FUNC_LEN] = {
        VP890_ENABLE_LOADED_COEFFICIENTS | VP890_LINEAR_CODEC
    };

    uint8 opCond[VP890_OP_COND_LEN] = {
        VP890_TXPATH_DIS | VP890_NOISE_GEN_EN
    };

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890CalMeasureBFilter()"));

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    pLineObj->status |= VP890_LINE_IN_CAL;

    /* Save off current register content that will be modified */
    VpMpiCmdWrapper(deviceId, ecVal, VP890_OP_FUNC_RD, VP890_OP_FUNC_LEN,
        bFilterData->opFunct);
    bFilterData->opCond[0] = pLineObj->opCond[0];

    VpMpiCmdWrapper(deviceId, ecVal, VP890_CONV_CFG_RD, VP890_CONV_CFG_LEN,
        bFilterData->convCfg);
    VpMpiCmdWrapper(deviceId, ecVal, VP890_VP_GAIN_RD, VP890_VP_GAIN_LEN,
        bFilterData->vpGain);

#ifdef VP890_REDUCE_BFILTER_CAL_SIGNAL_LEVEL
    Vp890ReduceNoiseOutput(bFilterData->vpGain[0], deviceId, ecVal);
#endif

    /* Program device to measure B-Filter levels and start noise generator */
    VP_CALIBRATION(VpLineCtxType, pLineCtx,("\n\r5. Writing 0x%02X to Operating Functions",
        opFunctions[0]));
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_OP_FUNC_WRT,
        VP890_OP_FUNC_LEN, opFunctions);

    VP_CALIBRATION(VpLineCtxType, pLineCtx,("\n\r7. Writing 0x%02X to Operating Conditions",
        opCond[0]));
    pLineObj->opCond[0] = opCond[0];
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_OP_COND_WRT,
        VP890_OP_COND_LEN, opCond);

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_CONV_CFG_WRT,
        VP890_CONV_CFG_LEN, convCfg);

    VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0],  mpiIndex-1,
        &mpiBuffer[1]);

    /* Set timer for measurement */
    pLineObj->lineTimers.timers.fxoTimer.measureBFilterTimer = 0;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
}

VpStatusType
Vp890CalMeasureBFilterInt(
    VpLineCtxType       *pLineCtx)
{
    Vp890LineObjectType *pLineObj   = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx           = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj  = pDevCtx->pDevObj;
    VpDeviceIdType deviceId         = pDevObj->deviceId;
    uint8 ecVal                     = pLineObj->ecVal;
    uint8 mpiIndex                  = 0;
    uint8 mpiBuffer[4 + VP890_OP_FUNC_LEN + VP890_OP_COND_LEN +
                    VP890_CONV_CFG_LEN + VP890_VP_GAIN_LEN];

    Vp890CalBFilterData *bFilterData = &pLineObj->calLineData.typeData.bFilterData;
    uint8 bFilterResult[VP890_TX_PCM_DATA_LEN];
    uint16 vRms;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890CalMeasureBFilterInt()"));

    /* Read the current B-Filter Measured Data */
    VpMpiCmdWrapper(deviceId, ecVal, VP890_TX_PCM_DATA_RD, VP890_TX_PCM_DATA_LEN,
        bFilterResult);
    vRms = (bFilterResult[0] << 8) | bFilterResult[1];

    VP_CALIBRATION(VpLineCtxType, pLineCtx,("Vp890CalMeasureBFilterInt() - vRms: %d    TS: %d",
        vRms, pDevObj->timeStamp));

    VP_CALIBRATION(VpLineCtxType, pLineCtx,("\n\r6. Writing 0x%02X to Operating Functions",
        bFilterData->opFunct[0]));

    /* Restore saved registers */
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_OP_FUNC_WRT,
        VP890_OP_FUNC_LEN, bFilterData->opFunct);

    VP_CALIBRATION(VpLineCtxType, pLineCtx,("\n\r8. Writing 0x%02X to Operating Conditions",
        bFilterData->opCond[0]));

    pLineObj->opCond[0] = bFilterData->opCond[0];
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_OP_COND_WRT,
        VP890_OP_COND_LEN, bFilterData->opCond);

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_CONV_CFG_WRT,
        VP890_CONV_CFG_LEN, bFilterData->convCfg);

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_VP_GAIN_WRT,
        VP890_VP_GAIN_LEN, bFilterData->vpGain);

    VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0],
        mpiIndex-1, &mpiBuffer[1]);

    pLineObj->lineEvents.response |= VP_EVID_CAL_CMP;
    pLineObj->responseData = vRms;

    VP_CALIBRATION(VpLineCtxType, pLineCtx,("Vp890CalMeasureBFilterInt() - Done"));
    VP_CALIBRATION(VpLineCtxType, pLineCtx,("lineEvents.response = %04X", pLineObj->lineEvents.response));
    VP_CALIBRATION(VpLineCtxType, pLineCtx,("responseData = %04X", pLineObj->responseData));

    pLineObj->status &= ~VP890_LINE_IN_CAL;

    return VP_STATUS_SUCCESS;
}

#ifdef VP890_REDUCE_BFILTER_CAL_SIGNAL_LEVEL
/* Adjust the digital gains as much as possible to lower the level of the
 * noise generator without changing the measurement results.
 * To do so, we increase the digital receive loss (DRL) while increasing
 * the digital transmit gain (DTG) by the same amount. */
static void
Vp890ReduceNoiseOutput(
    uint8 oldVpGain,
    VpDeviceIdType deviceId,
    uint8 ecVal)
{
    /* These lookup tables are indexed by the values of DRL and DTG that we
     * just read from the device.  The results of the lookup will be the
     * DRL and DTG values that give the most possible reduction in the noise
     * output level.
     * The restraints are that DRL is limited to 0db, -6db, -12dB, or -18dB
     * and DTG is limited to 0dB, +6dB, and +12dB.  This means that we can
     * achieve the best reduction if DTG is 0dB and DRL is 0dB or -6dB,
     * allowing us to change both by 12.  If DTG is already +12 or DRL is
     * already -18, we can make no change.
     */
    uint8 vpGainDrlLut[][4] = {
                 /* DRL=0dB         DRL=6dB         DRL=-12dB       DRL=-18dB */
    /* DTG=0dB */{VP890_DRL_12DB, VP890_DRL_18DB, VP890_DRL_18DB, VP890_DRL_18DB},
    /* DTG=6dB */{VP890_DRL_6DB,  VP890_DRL_12DB, VP890_DRL_18DB, VP890_DRL_18DB},
    /* DTG=12dB*/{VP890_DRL_0DB,  VP890_DRL_6DB,  VP890_DRL_12DB, VP890_DRL_18DB},
    /* DTG=rsv */{VP890_DRL_12DB, VP890_DRL_18DB, VP890_DRL_18DB, VP890_DRL_18DB},
    };
    uint8 vpGainDtgLut[][4] = {
                 /* DRL=00          DRL=01          DRL=10          DRL=11 */
    /* DTG=0dB */{VP890_DTG_12DB, VP890_DTG_12DB, VP890_DTG_6DB,  VP890_DTG_0DB},
    /* DTG=6dB */{VP890_DTG_12DB, VP890_DTG_12DB, VP890_DTG_12DB, VP890_DTG_6DB},
    /* DTG=12dB*/{VP890_DTG_12DB, VP890_DTG_12DB, VP890_DTG_12DB, VP890_DTG_12DB},
    /* DTG=rsv */{VP890_DTG_12DB, VP890_DTG_12DB, VP890_DTG_6DB,  VP890_DTG_0DB},
    };
    uint8 drlIndex;
    uint8 dtgIndex;
    uint8 newVpGain;

    drlIndex = (oldVpGain & VP890_DRL_MASK);
    drlIndex <<= VP890_DRL_BITSHIFT;
    dtgIndex = (oldVpGain & VP890_DTG_MASK);

    newVpGain = oldVpGain;
    newVpGain &= ~VP890_DRL_MASK;
    newVpGain &= ~VP890_DTG_MASK;
    newVpGain |= vpGainDrlLut[dtgIndex][drlIndex];
    newVpGain |= vpGainDtgLut[dtgIndex][drlIndex];

    VP_CALIBRATION(VpLineCtxType, pLineCtx,("Vp890ReduceNoiseOutput(): oldVpGain 0x%02X, drlIndex %d, dtgIndex %d, newVpGain 0x%02X",
        oldVpGain, drlIndex, dtgIndex, newVpGain));

    VpMpiCmdWrapper(deviceId, ecVal, VP890_VP_GAIN_WRT, VP890_VP_GAIN_LEN, &newVpGain);
}
#endif /* VP890_REDUCE_BFILTER_CAL_SIGNAL_LEVEL */
#endif /* VP890_FXO_SUPPORT */


#endif /* VP_CC_890_SERIES */
