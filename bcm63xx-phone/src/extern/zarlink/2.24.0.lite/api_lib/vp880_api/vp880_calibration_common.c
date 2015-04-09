/** \file vp880_calibration_common.c
 * vp880_calibration_common.c
 *
 * This file contains the line and device calibration functions that are common
 * within the 880 device family (for ABS and Tracker).
 *
 * Copyright (c) 2012, Microsemi Corporation
 *
 * $Revision: 11491 $
 * $LastChangedDate: 2014-07-15 14:14:06 -0500 (Tue, 15 Jul 2014) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_880_SERIES)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_api.h"
#include "vp_api_int.h"
#include "vp880_api.h"
#include "vp880_api_int.h"
#include "vp_hal.h"
#include "sys_service.h"


#if defined (VP_CSLAC_RUNTIME_CAL_ENABLED) && defined (VP880_FXS_SUPPORT)

#define VP880_IMT_SETTLE_MS 100
#define VP880_DCFEED_SETTLE_MS 100
#define VP880_IMT_10MA 5679L

/* Functions for Cal Line = FXS only */

static bool
CalLineNextState(
    VpDevCtxType            *pDevCtx,
    VpLineCtxType           *pLineCtx,
    Vp880DeviceObjectType   *pDevObj,
    Vp880LineObjectType     *pLineObj,
    uint16                  *pCalTimerMs);

#ifdef VP880_TRACKER_SUPPORT
static void
ComputeFinalVas(
    VpLineCtxType *pLineCtx,
    uint16 *vasValue);
#endif  /* #ifdef VP880_TRACKER_SUPPORT */

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

#endif  /* #if defined (VP_CSLAC_RUNTIME_CAL_ENABLED) && defined (VP880_FXS_SUPPORT) */

#if defined (VP_CSLAC_RUNTIME_CAL_ENABLED) && defined (VP880_FXS_SUPPORT)

/**
 * Vp880CalCodec() -- Tracker and ABS Function
 *  This function initiates a calibration operation for analog circuits
 * associated with all the lines of a device. See VP-API reference guide for
 * more information.

 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action.
 */
VpStatusType
Vp880CalCodec(
    VpLineCtxType *pLineCtx,
    VpDeviceCalType mode)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 maxChannels = pDevObj->staticInfo.maxChannels;
    uint8 chanNum;
    VpLineStateType currentState;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880CalCodec+"));

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    pDevObj->responseData = VP_CAL_SUCCESS;
    if (pDevObj->stateInt & VP880_DEVICE_CAL_COMPLETE) {
        pDevObj->deviceEvents.response |= VP_EVID_CAL_CMP;
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880CalCodec-"));
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_SUCCESS;
    }

    if (mode == VP_DEV_CAL_NBUSY) {
        for (chanNum = 0; chanNum < maxChannels; chanNum++) {
            if (pDevCtx->pLineCtx[chanNum] != VP_NULL) {
                pLineObj = pLineCtx->pLineObj;
                currentState = pLineObj->lineState.currentState;

                if (pLineObj->status & VP880_IS_FXO) {
                    if ((currentState == VP_LINE_FXO_TALK)
                     || (currentState == VP_LINE_FXO_LOOP_CLOSE)) {
                        pDevObj->deviceEvents.response |= VP_EVID_CAL_BUSY;
                        pDevObj->deviceEvents.response &= ~VP_EVID_CAL_CMP;
                        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880CalCodec-"));
                        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                        return VP_STATUS_SUCCESS;
                    }
                } else {
                    if ((currentState != VP_LINE_STANDBY)
                     && (currentState != VP_LINE_DISCONNECT)) {
                        pDevObj->deviceEvents.response |= VP_EVID_CAL_BUSY;
                        pDevObj->deviceEvents.response &= ~VP_EVID_CAL_CMP;
                        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880CalCodec-"));
                        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                        return VP_STATUS_SUCCESS;
                    }
                }
            }
        }
    }

    pDevObj->state |= VP_DEV_IN_CAL;

    if (!(pDevObj->stateInt & VP880_IS_ABS)) {
#ifdef VP880_TRACKER_SUPPORT
        pDevObj->state |= VP_DEV_ABV_CAL;
        Vp880CalCodecInt(pDevCtx);
#endif
    } else {
#ifdef VP880_ABS_SUPPORT
        if (Vp880SetCalFlags(pDevObj) == TRUE) {
            Vp880CalCodecInt(pDevCtx);
        }
#endif
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880CalCodec-"));
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}   /* Vp880CalCodec() */

/**
 * Vp880CalCodecInt() -- Tracker and ABS Function
 *  This function initiates a calibration operation for analog circuits
 * associated with all the lines of a device. See VP-API reference guide for
 * more information.

 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action.
 */
VpStatusType
Vp880CalCodecInt(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880CalCodecInt+"));

#ifdef VP880_TRACKER_SUPPORT
    if (pDevObj->state & VP_DEV_ABV_CAL) {
        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Calling ABV Cal inside Vp880CalCodecInt() for Tracker"));
        Vp880CalAbv(pDevCtx);
        VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880CalCodecInt-"));
        return VP_STATUS_SUCCESS;
    }
#endif

#ifdef VP880_ABS_SUPPORT
    if (pDevObj->state & VP_DEV_ABV_CAL_ABS) {
        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Calling ABV Cal inside Vp880CalCodecInt() for ABS"));
        Vp880CalAbvAbsDev(pDevCtx);
        VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880CalCodecInt-"));
        return VP_STATUS_SUCCESS;
    }

    if (pDevObj->state & VP_DEV_ABS_BAT_CAL) {
        VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Calling ABS Cal (Vp880AbsCalibration()) inside Vp880CalCodecInt()"));
        Vp880AbsCalibration(pDevCtx);
        VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880CalCodecInt-"));
        return VP_STATUS_SUCCESS;
    }
#endif

    VP_ERROR(VpDevCtxType, pDevCtx, ("Vp880CalCodecInt- Exit to unspecified calibration state!!"));
    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp880CalCodecInt-"));
    return VP_STATUS_SUCCESS;
}   /* Vp880CalCodecInt() */

/**
 * Vp880CalLine()
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
Vp880CalLine(
    VpLineCtxType *pLineCtx)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint16 tickRate = pDevObj->devProfileData.tickRate;
    VpLineStateType currentState = pLineObj->lineState.usrCurrent;
    uint8 ecVal = pLineObj->ecVal;
    uint8 opCond[VP880_OP_COND_LEN];

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp880CalLine()"));

    /* Block if device init is not complete */
    if (!(pDevObj->state & VP_DEV_INIT_CMP)) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    if (pLineObj->status & VP880_IS_FXO) {
        return VP_STATUS_INVALID_ARG;
    }

    /*
     * Do not proceed if the device calibration is in progress. This could
     * damage the device.
     */
    if (pDevObj->state & VP_DEV_IN_CAL) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Return NOT_INITIALIZED from Vp880CalLine()"));
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx,
            ("-Vp880CalLine()"));
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("Running Cal Line on Channel %d at time %d target 0x%02X 0x%02X",
        pLineObj->channelId, pDevObj->timeStamp,
        pLineObj->calLineData.dcFeedRef[0],
        pLineObj->calLineData.dcFeedRef[1]));

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * Don't run calibration if previously complete on this line OR if calibration was applied at
     * the System level but hasn't made it to this line yet. Check "complete" and "reload" flags.
     * If both are set, it means VpCal() with "Apply System Coefficients" was provided but this
     * function was called before VpApiTick(). The next call to VpApiTick() will load this line
     * with calibration data and set the calDone = TRUE.
     */
    if ((pLineObj->calLineData.calDone == TRUE)
     || ((pDevObj->stateInt & (VP880_SYS_CAL_COMPLETE | VP880_CAL_RELOAD_REQ))
                           == (VP880_SYS_CAL_COMPLETE | VP880_CAL_RELOAD_REQ) )
     || (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] < VP880_REV_JE)) {
        pLineObj->lineEvents.response |= VP_EVID_CAL_CMP;
        pLineObj->lineState.calType = VP_CSLAC_CAL_NONE;
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Calibration Previously Done. Cal Line Complete"));
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_SUCCESS;
    }

    pLineObj->status |= VP880_LINE_IN_CAL;

    /* Make sure line calibration can be run */
    switch(currentState) {
        case VP_LINE_OHT:
            break;
        case VP_LINE_STANDBY:
            pLineObj->lineState.calType =  VP_CSLAC_CAL_NONE;
            Vp880SetLineState(pLineCtx, VP_LINE_OHT);
            break;
        case VP_LINE_STANDBY_POLREV:
        case VP_LINE_OHT_POLREV:
            pLineObj->lineState.calType =  VP_CSLAC_CAL_NONE;
            Vp880SetLineState(pLineCtx, VP_LINE_OHT_POLREV);
            break;
        default:
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp880CalLine()"));
            return VP_STATUS_INVALID_ARG;
    }

    /* we need to remember if we started in polrev */
    if (pLineObj->slicValueCache & VP880_SS_POLARITY_MASK) {
        pLineObj->calLineData.reversePol = TRUE;
    } else {
        pLineObj->calLineData.reversePol = FALSE;
    }

#ifdef VP880_LP_SUPPORT
    /*
     * Force a LPM Exit update before proceeding with Calibration. If this
     * isn't done before setting the device mask, the LPM code to exit LPM will
     * not run (because it otherwise does not touch the device while in
     * calibration). If it is not a LPM device, this function doesn't do
     * anything.
     */
    Vp880LowPowerMode(pDevCtx);
#endif

    pLineObj->lineTimers.timers.timer[VP_LINE_CAL_LINE_TIMER] =
        MS_TO_TICKRATE(VP880_MIN_CAL_WAIT_TIME, tickRate);
    pLineObj->lineTimers.timers.timer[VP_LINE_CAL_LINE_TIMER] |=
        VP_ACTIVATE_TIMER;


    /* Reprogram the Operating Conditions Register, affected by Set Line State */
    opCond[0] = (VP880_CUT_TXPATH | VP880_CUT_RXPATH | VP880_HIGH_PASS_DIS);
    VpMpiCmdWrapper(deviceId, ecVal, VP880_OP_COND_WRT, VP880_OP_COND_LEN, opCond);
    pLineObj->opCond[0] = opCond[0];

    pLineObj->calLineData.calLineState =  VP880_CAL_SETUP;

    /*
     * Vp880ServiceFxsInterrupts uses this to decide if ring exit debounce
     * or CID need to be handled. Setting it to something other than
     * VP_CSLAC_CAL_NONE should be good enough.
     */
    pLineObj->lineState.calType = VP_CSLAC_CAL_VOC;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp880CalLine()"));

    return VP_STATUS_SUCCESS;
}

/**
 * Vp880CalLineInt()
 * This function is called each time the VP_LINE_CAL_LINE_TIMER expires.
 * Its basic purpose is to continue calling CalLineNextState until it
 * returns false.
 *
 * Postconditions:
 *  This function generates an event upon completing the requested action or
 *  sets up the cal timer to expire at a later time to run the next state.
 */
VpStatusType
Vp880CalLineInt(
    VpLineCtxType *pLineCtx)
{

    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 tickRate = pDevObj->devProfileData.tickRate;
    uint16 calTimerMs = 0;
    pDevObj->responseData = VP_CAL_SUCCESS;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp880CalLineInt()"));

    /* Continue calling the cal state machine until it indicates FALSE */
    while (CalLineNextState(pDevCtx, pLineCtx, pDevObj, pLineObj, &calTimerMs));

    if (calTimerMs != 0) {


        /* Setup the Cal timer to expire in requested ms */
        pLineObj->lineTimers.timers.timer[VP_LINE_CAL_LINE_TIMER] =
            MS_TO_TICKRATE(calTimerMs, tickRate);

        pLineObj->lineTimers.timers.timer[VP_LINE_CAL_LINE_TIMER] |=
            VP_ACTIVATE_TIMER;

    } else {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("Generating VP_EVID_CAL_CMP Event for Channel %d EventMask 0x%04X",
            pLineObj->channelId, pLineObj->lineEventsMask.response));

        /* If no timer was requested, Cal must be done so generate and event */
        pLineObj->lineState.calType = VP_CSLAC_CAL_NONE;
        pLineObj->lineEvents.response |= VP_EVID_CAL_CMP;
        pLineObj->status &= ~VP880_LINE_IN_CAL;

        /*
         * Set calDone = TRUE only if there wasn't a failure detected. If the
         * calDone flag is TRUE, the line can't be recalibrated without the
         * hidden setting with APPLY_SYSTEM_CEOFF and NULL profile.
         */
        if (pDevObj->responseData == VP_CAL_FAILURE) {
            /*
             * This should be redundant because calibration won't run unless
             * this value was previously set to FALSE. But it's safe just to
             * make sure it's set to a know value for a known condition.
             */
            pLineObj->calLineData.calDone = FALSE;
        } else {
            pLineObj->calLineData.calDone = TRUE;
        }

        /* line calibration is no longer in progress at this point */
        pLineObj->status &= ~VP880_LINE_IN_CAL;
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp880CalLineInt()"));

    return VP_STATUS_SUCCESS;
} /* Vp880CalLineInt */

/**
 * CalLineNextState()
 * This function implements the steps of the 880 Calibration algorithm.
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
    Vp880DeviceObjectType   *pDevObj,
    Vp880LineObjectType     *pLineObj,
    uint16                  *pCalTimerMs)
{

    uint8 channelId = pLineObj->channelId;
    uint8 ecVal = pLineObj->ecVal;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    Vp880CalLineState nextState = VP880_CAL_ENUM_SIZE;
    bool runAnotherState = FALSE;
    uint8 mpiIndex = 0;
    uint8 mpiBuffer[254];

    *pCalTimerMs = 0;

    switch(pLineObj->calLineData.calLineState) {

        /***********************************************************************
         * This state is used to setup the device for the cal sequence
         * by clearing out values in registers.
         **********************************************************************/
        case VP880_CAL_SETUP: {
            uint8 disnVal = 0;
            uint8 vpGain = 0;
            uint8 codec = VP880_LINEAR_CODEC;
            uint8 loopSup[VP880_LOOP_SUP_LEN];

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_CAL_SETUP"));
            /*
             * Start off with the assumption calibration will work. Simpler to
             * let the algorithm detect failure and set this accordingly rather
             * than have each possible good exit set it for pass.
             */
            pDevObj->responseData = VP_CAL_SUCCESS;

            /* Store LoopSup, DISN , Voice Path Gain, ICR2, SLIC state, and genA */
            VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_RD,
                VP880_SS_CONFIG_LEN, &pLineObj->calLineData.asscReg);

            VpMemCpy(loopSup, pLineObj->calLineData.loopSup, VP880_LOOP_SUP_LEN);
            VpMpiCmdWrapper(deviceId, ecVal, VP880_DISN_RD,
                VP880_DISN_LEN, pLineObj->calLineData.disnVal);
            VpMpiCmdWrapper(deviceId, ecVal, VP880_VP_GAIN_RD,
                VP880_VP_GAIN_LEN,pLineObj->calLineData.vpGain);

            VpMemCpy(pLineObj->calLineData.icr2, pLineObj->icr2Values, VP880_ICR2_LEN);
            VpMemCpy(pLineObj->calLineData.icr3, pLineObj->icr3Values, VP880_ICR3_LEN);

            VpMpiCmdWrapper(deviceId, ecVal, VP880_SIGA_PARAMS_RD,
                VP880_SIGA_PARAMS_LEN, pLineObj->calLineData.sigGenA);
            VpMpiCmdWrapper(deviceId, ecVal, VP880_OP_FUNC_RD,
                VP880_OP_FUNC_LEN, &pLineObj->calLineData.codecReg);

            /* Set Hook Debounce to 8 ms */
            loopSup[VP880_LOOP_SUP_DEBOUNCE_BYTE] &= ~VP880_SWHOOK_DEBOUNCE_MASK;
            loopSup[VP880_LOOP_SUP_DEBOUNCE_BYTE] |= 0x04;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_LOOP_SUP_WRT, VP880_LOOP_SUP_LEN, loopSup);

            /* Set DISN = 0 and Voice Path Gain = 0dB TX */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_DISN_WRT, VP880_DISN_LEN, &disnVal);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_VP_GAIN_WRT,VP880_VP_GAIN_LEN, &vpGain);

            /* Set the device mode register for single buffer mode */
            pDevObj->devMode[0] &= ~(VP880_DEV_MODE_TEST_DATA);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_DEV_MODE_WRT, VP880_DEV_MODE_LEN, pDevObj->devMode);

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("\n\rVP880_CAL_SETUP: Writing 0x%02X to Operating Functions", codec));

            /* Set for Linear Mode and disable AC Coefficients */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_OP_FUNC_WRT, VP880_OP_FUNC_LEN, &codec);

            /* Cut TX/RX PCM and disable HPF */
            pLineObj->opCond[0] = (VP880_CUT_TXPATH | VP880_CUT_RXPATH | VP880_HIGH_PASS_DIS);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_OP_COND_WRT, VP880_OP_COND_LEN, pLineObj->opCond);

            /* Copy the reference dcfeed data into the polrev dcfeed array */
            VpMemCpy(pLineObj->calLineData.dcFeedPr,
                pLineObj->calLineData.dcFeedRef, VP880_DC_FEED_LEN);
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("\n\rVP880_CAL_SETUP: Copied 0x%02X 0x%02X to dcFeedPr",
                 pLineObj->calLineData.dcFeedPr[0], pLineObj->calLineData.dcFeedPr[1]));

            if (!(pDevObj->stateInt & VP880_IS_ABS)) {
#ifdef VP880_TRACKER_SUPPORT
                /* set VAS to 3.0V (min) */
                pLineObj->calLineData.dcFeedPr[0] &= 0xFC;
                pLineObj->calLineData.dcFeedPr[1] &= 0x3F;

                /* Start for Tracker Device */
                nextState = VP880_CAL_SETUP_RELEASE_CLAMPS;
#endif
            } else {
#ifdef VP880_ABS_SUPPORT
                /* Start for ABS Device */
                nextState = VP880_RAMP_TO_POLREV_SETUP;
#endif
            }

            /* set ila to 40mA */
            pLineObj->calLineData.dcFeedPr[1] &= ~VP880_ILA_MASK;
            pLineObj->calLineData.dcFeedPr[1] |= (VP880_ILA_MASK & 0x16);

            /* Set DC-Feed to values from dc profile */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_DC_FEED_WRT, VP880_DC_FEED_LEN, pLineObj->calLineData.dcFeedPr);
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("\n\rVP880_CAL_SETUP: Modified to 0x%02X 0x%02X to dcFeedPr",
                 pLineObj->calLineData.dcFeedPr[0], pLineObj->calLineData.dcFeedPr[1]));

            /* next state info */
            *pCalTimerMs = 10;

            break;
        }

        case VP880_CAL_SETUP_RELEASE_CLAMPS:
            if ((pDevObj->stateInt & VP880_IS_HIGH_VOLTAGE) &&
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
                    VP880_ICR2_WRT, VP880_ICR2_LEN, pLineObj->icr2Values);
            }

            /* if we are not already in polrev then we need to go there */
            if (!pLineObj->calLineData.reversePol) {
                nextState = VP880_RAMP_TO_POLREV_SETUP;
                *pCalTimerMs = 10;
            } else {
                nextState = VP880_IMT_PR_SETUP;
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
        case VP880_RAMP_TO_POLREV_SETUP: {
            uint8 asscReg = (pLineObj->calLineData.asscReg | VP880_ZXR_MASK);

            /* setup siggen to mimic a 45V VOC once ringing is enabled */
            uint8 sigGenA[VP880_SIGA_PARAMS_LEN] =
                {0x00, 0x25, 0x4E, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00 ,0x00, 0x00};

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_RAMP_TO_POLREV_SETUP"));

            /* disable auto ring entery / exit */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_SS_CONFIG_WRT, VP880_SS_CONFIG_LEN, &asscReg);

            /* Prepare the sig gen with ramp info */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_SIGA_PARAMS_WRT, VP880_SIGA_PARAMS_LEN, sigGenA);

            /*
             * go to the balanced ringing state. This should generate a
             * longitudinal shift. The direction and size will depend on
             * the value of VOC in the DC feed register.
             */
            pLineObj->slicValueCache &= ~VP880_SS_STATE_MASK;
            pLineObj->slicValueCache |= VP880_SS_BALANCED_RINGING;

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_SYS_STATE_WRT, VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);

            /* enable longitudinal clamp even in ringing for duration of the CalLine */
            pLineObj->icr3Values[2] |= 0x40;
            pLineObj->icr3Values[3] = (pLineObj->icr3Values[3] & ~0x40);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_ICR3_WRT, VP880_ICR3_LEN, pLineObj->icr3Values);

            /* next state info */
            *pCalTimerMs = 10;
            nextState = VP880_RAMP_TO_POLREV_RAMP1;
            break;
        }

        case VP880_RAMP_TO_POLREV_RAMP1: {
            /* force the siggen to start a negative ramp of 45V */
            uint8 sigGenA[VP880_SIGA_PARAMS_LEN] =
                {0x07, 0x25, 0x4E, 0x00, 0x25, 0x25, 0x4E, 0x00, 0x00 ,0x00, 0x00};

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_RAMP_TO_POLREV_RAMP1"));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_SIGA_PARAMS_WRT, VP880_SIGA_PARAMS_LEN, sigGenA);
            *pCalTimerMs = 100;
            nextState = VP880_RAMP_TO_POLREV_SWAP_POLARITY;
            break;
        }

        case VP880_RAMP_TO_POLREV_SWAP_POLARITY: {
            /* go to balanced ringing polrev */
            pLineObj->slicValueCache &= ~VP880_SS_STATE_MASK;
            pLineObj->slicValueCache |= VP880_SS_BALANCED_RINGING_PR;

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_SYS_STATE_WRT, VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);

            runAnotherState = TRUE;
            nextState = VP880_RAMP_TO_POLREV_RAMP2;
            break;
        }

        case VP880_RAMP_TO_POLREV_RAMP2: {
            /* force the siggen to complete the ramp */
            uint8 sigGenA[VP880_SIGA_PARAMS_LEN] =
                {0x03, 0x25, 0x4E, 0x00, 0x25, 0x25, 0x4E, 0x00, 0x00 ,0x00, 0x00};

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_RAMP_TO_POLREV_RAMP2"));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_SIGA_PARAMS_WRT, VP880_SIGA_PARAMS_LEN, sigGenA);
            *pCalTimerMs = 50;
            nextState = VP880_RAMP_TO_POLREV_GOACTIVE;
            break;
        }

        case VP880_RAMP_TO_POLREV_GOACTIVE: {
            uint8 sigGenA[VP880_SIGA_PARAMS_LEN] =
                {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00 ,0x00, 0x00};

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_RAMP_TO_POLREV_GOACTIVE"));

            /* go to active polrev */
            pLineObj->slicValueCache &= ~VP880_SS_STATE_MASK;
            pLineObj->slicValueCache |= VP880_SS_ACTIVE_POLREV;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_SYS_STATE_WRT, VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);

            /* remove the siggen ramp */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_SIGA_PARAMS_WRT, VP880_SIGA_PARAMS_LEN, sigGenA);

            *pCalTimerMs = 10;
            nextState = VP880_RAMP_TO_POLREV_COMPLETE;
            break;
        }

        case VP880_RAMP_TO_POLREV_COMPLETE: {
            /* restore the automatic system state control */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_SS_CONFIG_WRT, VP880_SS_CONFIG_LEN, &pLineObj->calLineData.asscReg);

            *pCalTimerMs = 20;
            nextState = VP880_IMT_PR_SETUP;
        }

        /***********************************************************************
         * Settle IMT in PolRev -
         *
         * These states setup, measure Metallic polrev current until it
         * settles. This is done to prevent high REN loads from distorting
         * the VAS measurements
         **********************************************************************/
        case VP880_IMT_PR_SETUP: {

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_IMT_PR_SETUP"));


            if (!(pDevObj->stateInt & VP880_IS_ABS)) {
#ifdef VP880_TRACKER_SUPPORT
                /* Enable battery and DC Speed up */
                pLineObj->icr2Values[2] |= 0xC0;
                pLineObj->icr2Values[3] |= 0xC0;
#endif
            } else {
#ifdef VP880_ABS_SUPPORT
                /* Enable DC Speed up only */
                pLineObj->icr2Values[2] |= 0x80;
                pLineObj->icr2Values[3] |= 0x80;
#endif
            }

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_ICR2_WRT, VP880_ICR2_LEN, pLineObj->icr2Values);

            /* prepare storage data */
            pLineObj->calLineData.typeData.loopData.loopNum = 0;
            pLineObj->calLineData.typeData.loopData.prevVal = 0;

            *pCalTimerMs = 10;
            nextState = VP880_IMT_PR_SET_ADC;
            break;
        }

        case VP880_IMT_PR_SET_ADC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_IMT_PR_SET_ADC"));

            SetAdc(pLineCtx, deviceId, ecVal, VP880_METALLIC_DC_I);
            *pCalTimerMs = 10;
            nextState = VP880_IMT_PR_MEASURE;
            break;
        }

        case VP880_IMT_PR_MEASURE: {
            int16 imOld = 0;
            int16 imNew = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_IMT_PR_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &imNew)) {
                *pCalTimerMs = 10;
                nextState = VP880_IMT_PR_MEASURE;
                break;
            }

            /* if this is the first imt measurement then get another */
            if (pLineObj->calLineData.typeData.loopData.loopNum++ == 0) {
                pLineObj->calLineData.typeData.loopData.prevVal = imNew;
                *pCalTimerMs = VP880_IMT_SETTLE_MS;
                nextState = VP880_IMT_PR_MEASURE;
                break;
            }
            imOld = pLineObj->calLineData.typeData.loopData.prevVal;

            /* have we settled enough or taken to long > 200ms*/
            if ( (pLineObj->calLineData.typeData.loopData.loopNum < 20 /*200ms*/) &&
                (((imNew + 15 - imOld) & 0xFFE0) != 0) ) {
                /* nope run it again */
                pLineObj->calLineData.typeData.loopData.prevVal = imNew;
                *pCalTimerMs = VP880_IMT_SETTLE_MS;
                nextState = VP880_IMT_PR_MEASURE;
                break;
            }

            runAnotherState = TRUE;

            if (!(pDevObj->stateInt & VP880_IS_ABS)) {
#ifdef VP880_TRACKER_SUPPORT
                nextState = VP880_VAS_PR_SETUP;
#endif
            } else {
                nextState = VP880_VAB_PR_SETUP;
            }
            break;
        }

#ifdef VP880_TRACKER_SUPPORT
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

        case VP880_VAS_PR_SETUP:

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAS_PR_SETUP"));

            /* prepare storage data */
            pLineObj->calLineData.typeData.loopData.loopNum = 0;
            pLineObj->calLineData.typeData.loopData.prevVal = 0x7FFF;

            pLineObj->calLineData.vasStart = 3000;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP880_VAS_PR_SETUP: Best Actual VAS Start %d",
                pLineObj->calLineData.vasStart));

            runAnotherState = TRUE;
            nextState = VP880_VAS_PR_STEP;
            break;

        case VP880_VAS_PR_STEP: {
            uint8 loopNum = pLineObj->calLineData.typeData.loopData.loopNum;
            uint16 vasVal = pLineObj->calLineData.vasStart + (750 * loopNum);

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAS_PR_STEP"));

            /*
             * If the calculated VAS is max, advance to battery adjustment
             * steps.
             */
            if (vasVal > VP880_VAS_MAX) {
                runAnotherState = TRUE;
                nextState = VP880_VAS_PR_STORE;
                break;
            }

            /* set VAS to the next value */
            VpCSLACSetVas(pLineObj->calLineData.dcFeedPr, vasVal);
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP880_VAS_PR_STEP: Setting VAS (%d) with Values 0x%02X 0x%02X at time %d",
                vasVal,
                pLineObj->calLineData.dcFeedPr[0], pLineObj->calLineData.dcFeedPr[1],
                pDevObj->timeStamp));
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_DC_FEED_WRT, VP880_DC_FEED_LEN, pLineObj->calLineData.dcFeedPr);

            /* dcfeed takes 100ms to settle after each step be very carefull adjusting this */
            *pCalTimerMs = VP880_DCFEED_SETTLE_MS;
            nextState = VP880_VAS_PR_SET_ADC;
            break;
        }

        case VP880_VAS_PR_SET_ADC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAS_PR_SET_ADC"));

            SetAdc(pLineCtx, deviceId, ecVal, VP880_METALLIC_DC_I);
            *pCalTimerMs = 10;
            nextState = VP880_VAS_PR_MEASURE;
            break;
        }

        case VP880_VAS_PR_MEASURE: {
            int16 imtOld = pLineObj->calLineData.typeData.loopData.prevVal;
            int16 imtNew = 0;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAS_PR_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &imtNew)) {
                *pCalTimerMs = 10;
                nextState = VP880_VAS_PR_MEASURE;
                break;
            }

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP880_VAS_PR_MEASURE: IMT Old %d IMT New %d",
                imtOld, imtNew));

            runAnotherState = TRUE;
            if (pLineObj->calLineData.typeData.loopData.loopNum++ == 0) {

                /* if this is the first imt measurement then get another */
                nextState = VP880_VAS_PR_STEP;

            } else if ((imtNew > VP880_IMT_10MA) ||
                    (((imtNew + 15 - imtOld) & 0xFFE0) != 0) ) {
                /*
                 * if the measured value is greater than 10 mA or the difference
                 * between old and new values are greater +9/-7 then get another
                 */

                 nextState = VP880_VAS_PR_STEP;
            } else {
                /* if there is nothing left then store the data */
                nextState = VP880_VAS_PR_STORE;
            }


            /* replace the old with the new */
            pLineObj->calLineData.typeData.loopData.prevVal = imtNew;
            break;
        }

        case VP880_VAS_PR_STORE: {
            uint16 vasVal;
            uint8 dcFeed[VP880_DC_FEED_LEN];

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAS_PR_STORE"));

            /* Compute final VAS Values */
            VpMpiCmdWrapper(deviceId, ecVal, VP880_DC_FEED_RD, VP880_DC_FEED_LEN, dcFeed);
            vasVal = VP880_VAS_CONVERSION(dcFeed[0], dcFeed[1]);
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
                ("VP880_VAS_PR_STORE: Setting VAS (%d) with Final Values 0x%02X 0x%02X at time %d",
                vasVal,
                pLineObj->calLineData.dcFeedPr[0], pLineObj->calLineData.dcFeedPr[1],
                pDevObj->timeStamp));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_DC_FEED_WRT, VP880_DC_FEED_LEN, pLineObj->calLineData.dcFeedPr);

            /* Second byte contains the lower two bits of VAS */
            pDevObj->vp880SysCalData.vas[channelId][VP880_REV_POLARITY] =
                ((pLineObj->calLineData.dcFeedPr[1] >> 6) & 0x3);

            /* First byte contains the upper two bits of VAS */
            pDevObj->vp880SysCalData.vas[channelId][VP880_REV_POLARITY] |=
                ((pLineObj->calLineData.dcFeedPr[0] << 2) & 0xC);

            runAnotherState = TRUE;
            nextState = VP880_VAB_PR_SETUP;

            break;
        }
#endif  /* #ifdef VP880_TRACKER_SUPPORT */

        /***********************************************************************
         * Measure VAB (VOC)  in polrev -
         *
         * These two states setup and measure the VAB (VOC) value in polrev.
         **********************************************************************/

        case VP880_VAB_PR_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAB_PR_SETUP"));

            SetAdc(pLineCtx, deviceId, ecVal, VP880_METALLIC_DC_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP880_VAB_PR_MEASURE;
            break;
        }

        case VP880_VAB_PR_MEASURE: {
            int16 *pVocRev = &pLineObj->calLineData.typeData.vocData.vocRev;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAB_PR_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, pVocRev)) {
                *pCalTimerMs = 10;
                nextState = VP880_VAB_PR_MEASURE;
            } else {
                runAnotherState = TRUE;
                nextState = VP880_VAB_PR_ADC_OFFSET_SETUP;
            }
            break;
        }

        /***********************************************************************
         * Measure VAB ADC offset in polrev -
         *
         * These two states setup and measure the ADC Tip to Ring voltage in
         * polrev.
         **********************************************************************/

        case VP880_VAB_PR_ADC_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAB_PR_ADC_OFFSET_SETUP"));

            /* Disconnect tip and ring sense - start feed collapse */
            pLineObj->icr6Values[0] = 0x00;
            pLineObj->icr6Values[1] = (VP880_C_RING_SNS_CUT | VP880_C_TIP_SNS_CUT);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_ICR6_WRT, VP880_ICR6_LEN, pLineObj->icr6Values);

            /* set the converter config register to measure Metallic DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP880_METALLIC_DC_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP880_VAB_PR_ADC_OFFSET_MEASURE;
            break;
        }

        case VP880_VAB_PR_ADC_OFFSET_MEASURE: {
            int16 temp = 0;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAB_PR_ADC_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP880_VAB_PR_ADC_OFFSET_MEASURE;
            } else {
                pDevObj->vp880SysCalData.vocOffset[channelId][VP880_REV_POLARITY] = temp;
                runAnotherState = TRUE;
                nextState = VP880_VA_PR_ADC_OFFSET_SETUP;
            }
            break;
        }

        /***********************************************************************
         * Measure VA ADC offset in polrev
         *
         * These two states setup and measure the ADC Tip to Gnd voltage
         * offset in normal polarity.
         **********************************************************************/

        case VP880_VA_PR_ADC_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VA_PR_ADC_OFFSET_SETUP"));

            /* set the converter config register to measure Tip DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP880_TIP_TO_GND_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP880_VA_PR_ADC_OFFSET_MEASURE;
            break;
        }

        case VP880_VA_PR_ADC_OFFSET_MEASURE: {
            int16 temp = 0;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VA_PR_ADC_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP880_VA_PR_ADC_OFFSET_MEASURE;
            } else {
                /* compensate for the static offset of 1.5V on 880*/
                pDevObj->vp880SysCalData.vagOffsetRev[channelId] = temp + 205;
                runAnotherState = TRUE;
                nextState = VP880_VB_PR_ADC_OFFSET_SETUP;
            }
            break;
        }

        /***********************************************************************
         * Measure VB ADC offset in polrev
         *
         * These two states setup and measure the ADC RING to Gnd voltage
         * offset in normal polarity.
         **********************************************************************/

        case VP880_VB_PR_ADC_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VB_PR_ADC_OFFSET_SETUP"));

            /* set the converter config register to measure Ring DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP880_RING_TO_GND_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP880_VB_PR_ADC_OFFSET_MEASURE;
            break;
        }

        case VP880_VB_PR_ADC_OFFSET_MEASURE: {
            int16 temp = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VB_PR_ADC_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP880_VB_PR_ADC_OFFSET_MEASURE;
            } else {
                /* compensate for the static offset of 1.5V on 880*/
                pDevObj->vp880SysCalData.vbgOffsetRev[channelId] = temp + 205;
                runAnotherState = TRUE;
                nextState = VP880_COLLAPSE_FEED;
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

        case VP880_COLLAPSE_FEED: {
            /* Sig Gen A cal setup */
            uint8 sigGenA[VP880_SIGA_PARAMS_LEN] =
                {0x00, 0x00, 0x00, 0x0A, 0xAB, 0x00, 0x02, 0x00, 0x00 ,0x00, 0x00};

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_COLLAPSE_FEED"));

            /* Disable VOC DAC */
            pLineObj->icr2Values[0] |= 0x20;
            pLineObj->icr2Values[1] &= ~0x20;

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_ICR2_WRT, VP880_ICR2_LEN, pLineObj->icr2Values);

            /* Re-Enable tip and ring sense */
            pLineObj->icr6Values[0] = 0x00;
            pLineObj->icr6Values[1] = 0x00;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_ICR6_WRT, VP880_ICR6_LEN, pLineObj->icr6Values);

            /* switch to normal polarity */
            pLineObj->slicValueCache &= ~VP880_SS_POLARITY_MASK;
            VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_WRT,
                VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);

            /*
             * set the sig gen with 0 bias, almost no amplitude and
             * 1000Hz use for ring offset
             */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_SIGA_PARAMS_WRT, VP880_SIGA_PARAMS_LEN, sigGenA);

            *pCalTimerMs = 100;
            nextState = VP880_GENA_NP_OFFSET_SETUP;
            break;
        }

        /***********************************************************************
         * Measure SIGGEN A offset -
         *
         * These states setup and measure the offset in the signal generator
         **********************************************************************/

        case VP880_GENA_NP_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_GENA_NP_OFFSET_SETUP"));

            /* store the current slicState */
            pLineObj->calLineData.sysState = pLineObj->slicValueCache;

            /* go to ringing state */
            pLineObj->slicValueCache &= ~VP880_SS_STATE_MASK;
            pLineObj->slicValueCache |= VP880_SS_BALANCED_RINGING;
            VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_WRT,
                VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);

            *pCalTimerMs = 20; /* wait for ringing */
            nextState = VP880_GENA_NP_OFFSET_SET_ADC;

            break;
        }

        case VP880_GENA_NP_OFFSET_SET_ADC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_GENA_NP_OFFSET_SET_ADC"));

            /* set the converter config register to measure Metallic DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP880_METALLIC_DC_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP880_GENA_NP_OFFSET_MEASURE;
            break;
        }

        case VP880_GENA_NP_OFFSET_MEASURE: {
            int16 temp = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_GENA_NP_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP880_GENA_NP_OFFSET_MEASURE;
            } else {
                /* no need to measure offet in rev polrity */
                pDevObj->vp880SysCalData.sigGenAError[channelId][VP880_NORM_POLARITY] = temp;

                /*
                 * we must get the slic out of ringing before restoring
                 * siggen A or risk catching some of the ring cycle
                 * and pinging a phone.
                 */
                pLineObj->slicValueCache = pLineObj->calLineData.sysState;
                VpMpiCmdWrapper(deviceId, ecVal, VP880_SYS_STATE_WRT,
                    VP880_SYS_STATE_LEN, &pLineObj->slicValueCache);
                *pCalTimerMs = 10;
                nextState = VP880_GENA_NP_OFFSET_RESTORE;
            }
            break;
        }

        case VP880_GENA_NP_OFFSET_RESTORE: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_GENA_NP_OFFSET_RESTORE"));

            /* restore the siggen A register */
            VpMpiCmdWrapper(deviceId, ecVal, VP880_SIGA_PARAMS_WRT,
                VP880_SIGA_PARAMS_LEN, pLineObj->calLineData.sigGenA);

            *pCalTimerMs = 30; /* give the device time to get out of ringing */
            nextState = VP880_ILG_OFFSET_SETUP;
            break;
        }

        /***********************************************************************
         * Measure ILG offset -
         *
         * These two states setup and measure ILG offset in normal polarity.
         * This measurement is take with tip/ring collapsed and sense enabled
         **********************************************************************/

        case VP880_ILG_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_ILG_OFFSET_SETUP"));

            /* set the converter config register to measure Logitudinal Current */
            SetAdc(pLineCtx, deviceId, ecVal, VP880_LONGITUDINAL_DC_I);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP880_ILG_OFFSET_MEASURE;
            break;
        }

        case VP880_ILG_OFFSET_MEASURE: {
            int16 temp = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_ILG_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP880_ILG_OFFSET_MEASURE;
            } else {
                pDevObj->vp880SysCalData.ilgOffsetNorm[channelId] = temp;
                runAnotherState = TRUE;
                nextState = VP880_ILA_OFFSET_SETUP;
            }
            break;
        }

        /***********************************************************************
         * Measure ILA offset -
         *
         * These two states setup and measure ILA offset in normal polarity.
         * This measurement is take with tip/ring collapsed and sense enabled
         **********************************************************************/

        case VP880_ILA_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_ILA_OFFSET_SETUP"));

            /* set the converter config register to measure Metallic DC Current */
            SetAdc(pLineCtx, deviceId, ecVal, VP880_METALLIC_DC_I);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP880_ILA_OFFSET_MEASURE;
            break;
        }

        case VP880_ILA_OFFSET_MEASURE: {
            int16 temp = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_ILA_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP880_ILA_OFFSET_MEASURE;
            } else {
                pDevObj->vp880SysCalData.ilaOffsetNorm[channelId] = temp;
                runAnotherState = TRUE;
                nextState = VP880_RESTORE_DAC;
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

        case VP880_RESTORE_DAC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_RESTORE_DAC"));

            /* Store the ref dcfeed from profile into normal cal dcfeed */
            VpMemCpy(pLineObj->calLineData.dcFeed, pLineObj->calLineData.dcFeedRef,
                VP880_DC_FEED_LEN);

#ifdef VP880_TRACKER_SUPPORT
            if (!(pDevObj->stateInt & VP880_IS_ABS)) {
                /* we want to start with a 3.0V VAS for tracker */
                pLineObj->calLineData.dcFeed[0] &= 0xFC;
                pLineObj->calLineData.dcFeed[1] &= 0x3F;
            }
#endif
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_DC_FEED_WRT, VP880_DC_FEED_LEN, pLineObj->calLineData.dcFeed);

            /* Enable VOC DAC */
            pLineObj->icr2Values[0] &= ~0x20;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_ICR2_WRT, VP880_ICR2_LEN, pLineObj->icr2Values);

            /* disable tip and ring sense */
            pLineObj->icr6Values[0] = 0x00;
            pLineObj->icr6Values[1] = (VP880_C_RING_SNS_CUT | VP880_C_TIP_SNS_CUT);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_ICR6_WRT, VP880_ICR6_LEN, pLineObj->icr6Values);

            runAnotherState = TRUE;
            nextState = VP880_VAB_NP_ADC_OFFSET_SETUP;
            break;
        }

        /***********************************************************************
         * Measure VAB ADC offset in normal polarity -
         *
         * These two states setup and measure ADC Tip to RING voltage offset in
         * normal polarity.
         **********************************************************************/

        case VP880_VAB_NP_ADC_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAB_NP_ADC_OFFSET_SETUP"));

            /* set the converter config register to measure Metallic DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP880_METALLIC_DC_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP880_VAB_NP_ADC_OFFSET_MEASURE;
            break;
        }

        case VP880_VAB_NP_ADC_OFFSET_MEASURE: {
            int16 temp = 0;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAB_NP_ADC_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP880_VAB_NP_ADC_OFFSET_MEASURE;
            } else {
                pDevObj->vp880SysCalData.vocOffset[channelId][VP880_NORM_POLARITY] = temp;
                runAnotherState = TRUE;
                nextState = VP880_VA_NP_ADC_OFFSET_SETUP;
            }
            break;
        }

        /***********************************************************************
         * Measure VA ADC offset in normal polarity
         *
         * These two states setup and measure the ADC Tip to Gnd voltage
         * offset in normal polarity.
         **********************************************************************/

        case VP880_VA_NP_ADC_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VA_NP_ADC_OFFSET_SETUP"));

            /* set the converter config register to measure Tip DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP880_TIP_TO_GND_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP880_VA_NP_ADC_OFFSET_MEASURE;
            break;
        }

        case VP880_VA_NP_ADC_OFFSET_MEASURE: {
            int16 temp = 0;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VA_NP_ADC_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP880_VA_NP_ADC_OFFSET_MEASURE;
            } else {
                /* compensate for the static offset of 1.5V on 880*/
                pDevObj->vp880SysCalData.vagOffsetNorm[channelId] = temp + 205;
                runAnotherState = TRUE;
                nextState = VP880_VB_NP_ADC_OFFSET_SETUP;
            }
            break;
        }

        /***********************************************************************
         * Measure VB ADC offset in normal polarity
         *
         * These two states setup and measure the ADC RING to Gnd voltage
         * offset in normal polarity.
         **********************************************************************/

        case VP880_VB_NP_ADC_OFFSET_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VB_NP_ADC_OFFSET_SETUP"));

            /* set the converter config register to measure Ring DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP880_RING_TO_GND_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP880_VB_NP_ADC_OFFSET_MEASURE;
            break;
        }

        case VP880_VB_NP_ADC_OFFSET_MEASURE: {
            int16 temp = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VB_NP_ADC_OFFSET_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP880_VB_NP_ADC_OFFSET_MEASURE;
            } else {
                /* compensate for the static offset of 1.5V on 880*/
                pDevObj->vp880SysCalData.vbgOffsetNorm[channelId] = temp + 205;
                runAnotherState = TRUE;
                nextState = VP880_ILA_SETUP;
            }
            break;
        }

        /***********************************************************************
         * Measure ILA at 4 different values -
         *
         * These xyz states setup and measure ILA offset in normal polarity
         * at 4 different ILA settings (20, 25, 32 and 40mA).
         **********************************************************************/

        case VP880_ILA_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_ILA_SETUP"));

            /* prepare storage data */
            pLineObj->calLineData.typeData.loopData.loopNum = 0;
            pLineObj->calLineData.typeData.loopData.prevVal = 0;

            pLineObj->calLineData.minVas = 0;

            runAnotherState = TRUE;
            nextState = VP880_ILA_ADJUST;
            break;
        }

        case VP880_ILA_ADJUST: {
            uint8 *pDcFeed = pLineObj->calLineData.dcFeed;
            uint8 loopNum = pLineObj->calLineData.typeData.loopData.loopNum;
            uint16 vasValue = VP880_VAS_CONVERSION(pDcFeed[0], pDcFeed[1]);

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_ILA_ADJUST"));

            /* adjust the ila value and vas */
            pDcFeed[1] &= ~VP880_ILA_MASK;
            if (loopNum == 0) { /* 20mA */
                pDcFeed[1] |= (VP880_ILA_MASK & 0x02);
                vasValue = 3000;
            } else if (loopNum == 1) { /* 25mA */
                pDcFeed[1] |= (VP880_ILA_MASK & 0x07);
                vasValue = 3750;
            } else if (loopNum == 2) { /* 32mA */
                pDcFeed[1] |= (VP880_ILA_MASK & 0x0E);
                vasValue = 4500;
            } else if (loopNum == 3) { /* 40mA */
                pDcFeed[1] |= (VP880_ILA_MASK & 0x16);
                vasValue = 5250;
            } else {
                /*
                 * If we're here, it's because off-hook has not yet been
                 * detected with VAS increments, so we have to continue past
                 * normal ILA calibration.
                 */
                vasValue += 750;
            }

#ifdef VP880_TRACKER_SUPPORT
            /* only adjust VAS if device is a tracking supply */
            if (!(pDevObj->stateInt & VP880_IS_ABS)) {
                if (vasValue < VP880_VAS_MAX) {
                    VpCSLACSetVas(pDcFeed, vasValue);
                } else {
                    /* ERROR!! Cannot increase VAS. Need to move on */
                    pLineObj->calLineData.minVas = VP880_VAS_MAX;
                    runAnotherState = TRUE;
                    nextState = VP880_RESTORE_FEED;

                }
            }
#endif
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP880_ILA_ADJUST: Writing DC FEED 0x%02X 0x%02X", pDcFeed[0], pDcFeed[1]));
            VpMpiCmdWrapper(deviceId, ecVal, VP880_DC_FEED_WRT, VP880_DC_FEED_LEN, pDcFeed);

            *pCalTimerMs = 10;
            nextState = VP880_ILA_SET_ADC;

            break;
        }

        case VP880_ILA_SET_ADC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_ILA_SET_ADC"));

            /* set the converter config register to measure Metallic DC Current */
            SetAdc(pLineCtx, deviceId, ecVal, VP880_METALLIC_DC_I);

            *pCalTimerMs = 10;
            nextState = VP880_ILA_MEASURE;
            break;
        }

        case VP880_ILA_MEASURE: {
            int16 temp = 0;
            uint8 loopNum = pLineObj->calLineData.typeData.loopData.loopNum;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_ILA_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP880_ILA_MEASURE;
                break;
            }

            /* store the measured ila data */
            if (loopNum == 0) {        /* 20mA */
                pDevObj->vp880SysCalData.ila20[channelId] = temp;
            } else if (loopNum == 1) { /* 25mA */
                pDevObj->vp880SysCalData.ila25[channelId] = temp;
            } else if (loopNum == 2) { /* 32mA */
                pDevObj->vp880SysCalData.ila32[channelId] = temp;
            } else if (loopNum == 3) { /* 40mA */
                pDevObj->vp880SysCalData.ila40[channelId] = temp;
            }

            runAnotherState = TRUE;

            if (!(pDevObj->stateInt & VP880_IS_ABS)) {
#ifdef VP880_TRACKER_SUPPORT
                nextState = VP880_ILA_MEASURE_TRACKER;
#endif
            } else {
#ifdef VP880_ABS_SUPPORT
                nextState = VP880_ILA_MEASURE_ABS;
#endif
            }
            break;
        }

        case VP880_ILA_MEASURE_TRACKER: {
            uint8 sigReg[VP880_NO_UL_SIGREG_LEN];
            uint16 vasValue;

            VpMpiCmdWrapper(deviceId, ecVal, VP880_NO_UL_SIGREG_RD,
                VP880_NO_UL_SIGREG_LEN, sigReg);

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP880_ILA_MEASURE: SIG REG 0x%02X 0x%02X", sigReg[0], sigReg[1]));

            if ((sigReg[channelId] & VP880_HOOK1_MASK) && (pLineObj->calLineData.minVas == 0)) {

                vasValue = VP880_VAS_CONVERSION(
                    pLineObj->calLineData.dcFeed[0],
                    pLineObj->calLineData.dcFeed[1]);

                if ((vasValue + VP880_VAS_MIN_OVERHEAD) < VP880_VAS_MAX) {
                    pLineObj->calLineData.minVas = vasValue + VP880_VAS_MIN_OVERHEAD;
                } else {
                    pLineObj->calLineData.minVas = VP880_VAS_MAX;
                }

                VP_CALIBRATION(VpLineCtxType, pLineCtx, ("VP880_ILA_MEASURE: Saving Min VAS (%d)",
                    pLineObj->calLineData.minVas));
            }

            if ((pLineObj->calLineData.typeData.loopData.loopNum++ < 3)
                || (!(sigReg[channelId] & VP880_HOOK1_MASK))) {
                nextState = VP880_ILA_ADJUST;
            } else {
                nextState = VP880_RESTORE_FEED;
            }
            runAnotherState = TRUE;

            break;
        }

        case VP880_ILA_MEASURE_ABS:
            if (pLineObj->calLineData.typeData.loopData.loopNum++ < 3) {
                nextState = VP880_ILA_ADJUST;
            } else {
                nextState = VP880_RESTORE_FEED;
            }
            runAnotherState = TRUE;
            break;

        /***********************************************************************
         * Restore Feed
         *
         * For the most part feed is already restored. All we do here is
         * reenable the tip and ring sense and allow the line to settle.
         **********************************************************************/


        case VP880_RESTORE_FEED: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_RESTORE_FEED"));

            /* Re-Enable tip and ring sense */
            pLineObj->icr6Values[0] = 0x00;
            pLineObj->icr6Values[1] = 0x00;
            if (pDevObj->stateInt & VP880_IS_ABS) {
                /*
                 * Restore the battery switch hysterisis for ABS silicon. This is set to 0V so it
                 * doesn't get in the way during calibration.
                 */
                pLineObj->icr6Values[1] |= VP880_DCCAL_BAT_SW_HYST_5V;
            }
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_ICR6_WRT, VP880_ICR6_LEN, pLineObj->icr6Values);

            *pCalTimerMs = 10;
            nextState = VP880_IMT_NP_SETUP;
            break;
        }

       /***********************************************************************
         * Settle IMT in PolRev -
         *
         * These states setup, measure Metallic current until it
         * settles. This is done to prevent high REN loads from distorting
         * the VAS measurements
         **********************************************************************/
        case VP880_IMT_NP_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_IMT_NP_SETUP"));

            /* prepare storage data */
            pLineObj->calLineData.typeData.loopData.loopNum = 0;
            pLineObj->calLineData.typeData.loopData.prevVal = 0;

            runAnotherState = TRUE;
            nextState = VP880_IMT_NP_SET_ADC;
            break;
        }


        case VP880_IMT_NP_SET_ADC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_IMT_NP_SET_ADC"));

            SetAdc(pLineCtx, deviceId, ecVal, VP880_METALLIC_DC_I);
            *pCalTimerMs = 10;
            nextState = VP880_IMT_NP_MEASURE;
            break;
        }

        case VP880_IMT_NP_MEASURE: {
            int16 imOld = 0;
            int16 imNew = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_IMT_NP_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &imNew)) {
                *pCalTimerMs = 10;
                nextState = VP880_IMT_NP_MEASURE;
                break;
            }

            /* if this is the first imt measurement then get another */
            if (pLineObj->calLineData.typeData.loopData.loopNum++ == 0) {
                pLineObj->calLineData.typeData.loopData.prevVal = imNew;
                *pCalTimerMs = VP880_IMT_SETTLE_MS;
                nextState = VP880_IMT_NP_MEASURE;
                break;
            }
            imOld = pLineObj->calLineData.typeData.loopData.prevVal;

            /* have we settled enough or taken to long > 200ms*/
            if ( (pLineObj->calLineData.typeData.loopData.loopNum < 20 /*200ms*/) &&
                (((imNew + 15 - imOld) & 0xFFE0) != 0) ) {
                /* nope run it again */
                pLineObj->calLineData.typeData.loopData.prevVal = imNew;
                *pCalTimerMs = VP880_IMT_SETTLE_MS;
                nextState = VP880_IMT_NP_MEASURE;
                break;
            }

            if (!(pDevObj->stateInt & VP880_IS_ABS)) {
#ifdef VP880_TRACKER_SUPPORT
                nextState = VP880_VAS_NP_SETUP;
#endif
            } else {
#ifdef VP880_ABS_SUPPORT
                nextState = VP880_VAB_NP_SETUP;
#endif
            }
            runAnotherState = TRUE;
            break;
        }

#ifdef VP880_TRACKER_SUPPORT
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

        case VP880_VAS_NP_SETUP: {

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAS_NP_SETUP"));

            /* prepare storage data */
            pLineObj->calLineData.typeData.loopData.loopNum = 0;
            pLineObj->calLineData.typeData.loopData.prevVal = 0;

            pLineObj->calLineData.vasStart = 3000;

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP880_VAS_NP_SETUP: Best Actual VAS Start %d",
                pLineObj->calLineData.vasStart));

            runAnotherState = TRUE;
            nextState = VP880_VAS_NP_STEP;
            break;
        }

        case VP880_VAS_NP_STEP: {
            uint8 loopNum = pLineObj->calLineData.typeData.loopData.loopNum;
            uint16 vasVal = pLineObj->calLineData.vasStart + (750 * loopNum);

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAS_NP_STEP"));

            /* If the calculated VAS is max, we need to move to battery steps */
            if (vasVal > VP880_VAS_MAX) {
                runAnotherState = TRUE;
                nextState = VP880_VAS_NP_STORE;
                break;
            }

            /* set VAS to the next value */
            VpCSLACSetVas(pLineObj->calLineData.dcFeed, vasVal);
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP880_VAS_NP_STEP: Setting VAS (%d) with Values 0x%02X 0x%02X at time %d",
                vasVal,
                pLineObj->calLineData.dcFeed[0], pLineObj->calLineData.dcFeed[1],
                pDevObj->timeStamp));
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_DC_FEED_WRT, VP880_DC_FEED_LEN, pLineObj->calLineData.dcFeed);

            /* dcfeed takes 100ms to settle after each step be very carefull adjusting this */
            *pCalTimerMs = VP880_DCFEED_SETTLE_MS;
            nextState = VP880_VAS_NP_SET_ADC;
            break;
        }

        case VP880_VAS_NP_SET_ADC: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAS_NP_SET_ADC"));

            /* set the converter config register to measure Metallic DC Current */
            SetAdc(pLineCtx, deviceId, ecVal, VP880_METALLIC_DC_I);

            *pCalTimerMs = 10;
            nextState = VP880_VAS_NP_MEASURE;
            break;
        }

        case VP880_VAS_NP_MEASURE: {
            int16 imtOld = pLineObj->calLineData.typeData.loopData.prevVal;
            int16 imtNew = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAS_NP_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &imtNew)) {
                *pCalTimerMs = 10;
                nextState = VP880_VAS_NP_MEASURE;
                break;
            }

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP880_VAS_NP_MEASURE: IMT Old %d IMT New %d",
                imtOld, imtNew));

            runAnotherState = TRUE;
            if (pLineObj->calLineData.typeData.loopData.loopNum++ == 0) {

                /* if this is the first imt measurement then get another */
                nextState = VP880_VAS_NP_STEP;

            } else if ((imtNew > VP880_IMT_10MA) ||
                    (((imtOld + 15 - imtNew) & 0xFFE0) != 0) ) {
                /*
                 * if the measured value is greater than 10 mA or the difference
                 * between old and new values are greater +9/-7 then get another
                 */

                 nextState = VP880_VAS_NP_STEP;
            } else {
                /* if there is nothing left then store the data */
                nextState = VP880_VAS_NP_STORE;
            }

            /* replace the old with the new */
            pLineObj->calLineData.typeData.loopData.prevVal = imtNew;
            break;
        }

        case VP880_VAS_NP_STORE: {
            uint16 vasVal;
            uint8 dcFeed[VP880_DC_FEED_LEN];

            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAS_NP_STORE"));

            /* Compute final VAS Values */
            VpMpiCmdWrapper(deviceId, ecVal, VP880_DC_FEED_RD, VP880_DC_FEED_LEN, dcFeed);
            vasVal = VP880_VAS_CONVERSION(dcFeed[0], dcFeed[1]);

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
                ("VP880_VAS_NP_STORE: Setting VAS (%d) with Final Values 0x%02X 0x%02X at time %d",
                vasVal,
                pLineObj->calLineData.dcFeed[0], pLineObj->calLineData.dcFeed[1],
                pDevObj->timeStamp));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_DC_FEED_WRT, VP880_DC_FEED_LEN, pLineObj->calLineData.dcFeed);

            /* Second byte contains the lower two bits of VAS */
            pDevObj->vp880SysCalData.vas[channelId][VP880_NORM_POLARITY] =
                ((pLineObj->calLineData.dcFeed[1] >> 6) & 0x3);

            /* First byte contains the upper two bits of VAS */
            pDevObj->vp880SysCalData.vas[channelId][VP880_NORM_POLARITY] |=
                ((pLineObj->calLineData.dcFeed[0] << 2) & 0xC);

            /*
             * Make sure Reverse Polarity VAS is not set below minimum
             * determined from Battery Saturation Detection algorithm.
             */
            vasVal = VP880_VAS_CONVERSION(pLineObj->calLineData.dcFeedPr[0],
                pLineObj->calLineData.dcFeedPr[1]);
            if (vasVal < pLineObj->calLineData.minVas) {
                VpCSLACSetVas(pLineObj->calLineData.dcFeedPr,
                    pLineObj->calLineData.minVas);

                /* Second byte contains the lower two bits of VAS */
                pDevObj->vp880SysCalData.vas[channelId][VP880_REV_POLARITY] =
                    ((pLineObj->calLineData.dcFeedPr[1] >> 6) & 0x3);

                /* First byte contains the upper two bits of VAS */
                pDevObj->vp880SysCalData.vas[channelId][VP880_REV_POLARITY] |=
                    ((pLineObj->calLineData.dcFeedPr[0] << 2) & 0xC);

                VP_CALIBRATION(VpLineCtxType, pLineCtx,
                    ("VP880_VAS_NP_STORE: PolRev VAS below minVas (%d) - correcting VAS (%d) with Final Values 0x%02X 0x%02X at time %d",
                     pLineObj->calLineData.minVas, vasVal,
                     pLineObj->calLineData.dcFeedPr[0], pLineObj->calLineData.dcFeedPr[1],
                     pDevObj->timeStamp));
            }
            runAnotherState = TRUE;
            nextState = VP880_VAB_NP_SETUP;
            break;
        }
#endif  /* #ifdef VP880_TRACKER_SUPPORT */

        /***********************************************************************
         * Measure VAB (VOC)  in normal polarity -
         *
         * These two states setup and measure the VAB (VOC) value in normal pol.
         **********************************************************************/

        case VP880_VAB_NP_SETUP: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAB_NP_SETUP"));

            /* set the converter config register to measure Metallic DC Voltage */
            SetAdc(pLineCtx, deviceId, ecVal, VP880_METALLIC_DC_V);

            *pCalTimerMs = 10; /* wait for data */
            nextState = VP880_VAB_NP_MEASURE;
            break;
        }

        case VP880_VAB_NP_MEASURE: {
            int16 temp = 0;
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_VAB_NP_MEASURE"));

            if ( !GetXData(pLineCtx, deviceId, ecVal, &temp)) {
                *pCalTimerMs = 10;
                nextState = VP880_VAB_NP_MEASURE;
            } else {
                pLineObj->calLineData.typeData.vocData.vocNorm = temp;
                runAnotherState = TRUE;
                nextState = VP880_CAL_ADJUST;
            }
            break;
        }

        /***********************************************************************
         * This state is used to perform calcualtions
         **********************************************************************/
        case VP880_CAL_ADJUST: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_CAL_ADJUST"));

            /* Set VOC to the device profile value */
            Vp880AdjustVoc(pLineCtx, ((pLineObj->calLineData.dcFeedRef[0] >> 2) & 0x7), FALSE);

            /* Set ILA to the device profile value */
            Vp880AdjustIla(pLineCtx, (pLineObj->calLineData.dcFeedRef[1] & VP880_ILA_MASK));
            runAnotherState = TRUE;
            nextState = VP880_CAL_RESTORE;
            break;
        }


        /***********************************************************************
         * This state is used to restore registers and clean up after cal
         **********************************************************************/
        case VP880_CAL_RESTORE: {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("Cal State VP880_CAL_RESTORE"));

            /* Restore Device Mode */
            pDevObj->devMode[0] |= VP880_DEV_MODE_TEST_DATA;
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_DEV_MODE_WRT, VP880_DEV_MODE_LEN, pDevObj->devMode);

            /* Restore Loop Supervision */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_LOOP_SUP_WRT, VP880_LOOP_SUP_LEN, pLineObj->calLineData.loopSup);

            /* Restore Codec Mode */
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("VP880_CAL_RESTORE: Writing 0x%02X to Operating Functions",
                pLineObj->calLineData.codecReg));

            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_OP_FUNC_WRT, VP880_OP_FUNC_LEN, &pLineObj->calLineData.codecReg);

            /* Restore TX/RX PCM and enable HPF */
            pLineObj->opCond[0] = (VP880_CUT_TXPATH | VP880_CUT_RXPATH);
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_OP_COND_WRT, VP880_OP_COND_LEN, pLineObj->opCond);

            /* Restore disn */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_DISN_WRT, VP880_DISN_LEN, pLineObj->calLineData.disnVal);

            /* Restore vpGain */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_VP_GAIN_WRT, VP880_VP_GAIN_LEN, pLineObj->calLineData.vpGain);

            /* Restore ICR2 */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_ICR2_WRT, VP880_ICR2_LEN, pLineObj->calLineData.icr2);
            VpMemCpy(pLineObj->icr2Values, pLineObj->calLineData.icr2, VP880_ICR2_LEN);

            /* Restore ICR3 */
            mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                VP880_ICR3_WRT, VP880_ICR3_LEN, pLineObj->calLineData.icr3);
            VpMemCpy(pLineObj->icr3Values, pLineObj->calLineData.icr3, VP880_ICR2_LEN);

            /*
             * Restore Line State (note: this won't correctly update the
             * calibrated DC feed settings because we're still in calibration.
             * So we have to do that manually)
             */
            Vp880SetLineState(pLineCtx, pLineObj->lineState.usrCurrent);
            if (pLineObj->calLineData.reversePol) {
                /* write polrev dcfeed here */
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                    VP880_DC_FEED_WRT, VP880_DC_FEED_LEN, pLineObj->calLineData.dcFeedPr);
            } else {
                /* write normal dcfeed here */
                mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer,
                    VP880_DC_FEED_WRT, VP880_DC_FEED_LEN, pLineObj->calLineData.dcFeed);
            }

            Vp880LLSetSysState(deviceId, pLineCtx, 0, FALSE);
#ifdef VP880_LP_SUPPORT
            /* Force an update on the line */
            Vp880LowPowerMode(pDevCtx);
#endif
            break;
        }

        default:
            VP_CALIBRATION(VpLineCtxType, pLineCtx,("Cal Error - Bad State jumping to DONE"));
            pLineObj->calLineData.calLineState = VP880_CAL_RESTORE;
            runAnotherState = TRUE;
            pDevObj->responseData = VP_CAL_FAILURE;
            break;
    }


    /* write down any data required by a state */
    if (mpiIndex > 0) {
        VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0],
            mpiIndex-1, &mpiBuffer[1]);
    }

    pLineObj->calLineData.calLineState = nextState;
    return runAnotherState;
}

#ifdef VP880_TRACKER_SUPPORT
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
        VP880_VAS_OVERHEAD, *vasValue));

    /*
     * Apply known adjustments JUST to the computed VAS Value. This result does
     * not need (yet) to correspond to a programmable value.
     */
    *vasValue += VP880_VAS_OVERHEAD;

    /*
     * Determine if a battery adjustment is necessary just based on the required
     * vas value exceeding programmable limits.
     */
    if (*vasValue > VP880_VAS_MAX) {
        Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
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
        batteryCorrection = (*vasValue - VP880_VAS_MAX);
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("ComputeFinalVas: Raw Battery Correction: %d",
            batteryCorrection));

        /* Convert the current setting into scale of 1mv */
        targetCalVoltage = pDevObj->vp880SysCalData.abvError[0] * 10;

        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("ComputeFinalVas: Target Cal Voltage (%d) mV from Cal Profile: (%d)",
            targetCalVoltage, pDevObj->vp880SysCalData.abvError[0]));

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
        batteryOverage = (VP880_BAT_CAL_STEP - (newTarget % VP880_BAT_CAL_STEP));
        newTarget += batteryOverage;
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("ComputeFinalVas: Rounded Battery Correction: %ld", newTarget));

        /*
         * In case we can't adjust the battery enough to match the theoretical
         * requirement, limit to max and disable VAS reduction.
         */
        if (newTarget > VP880_BAT_CAL_MAX) {
            newTarget = VP880_BAT_CAL_MAX;
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
        if (newTarget > pDevObj->vp880SysCalData.abvError[channelId]) {
            VP_CALIBRATION(VpLineCtxType, pLineCtx,
                ("ComputeFinalVas: Modifying Current ABV Error from %d to %d on Channel %d",
                pDevObj->vp880SysCalData.abvError[channelId], (int16)newTarget, channelId));

            pDevObj->vp880SysCalData.abvError[channelId] = (int16)newTarget;

            /*
             * Write to the silicon - make sure Wideband Mode Bit IF set is
             * disabled for this write. It shouldn't occur, but be 100% sure
             */
            Vp880BatteryCalAdjust(pDevObj, (pLineObj->ecVal & VP880_EC_BITS_MASK));
        }

        /*
         * Don't forget that the pointer provided needs to correspond to the
         * final VAS value being programmed. Reduce by as much as possible
         * without reducing by more than theoretical.
         */
        *vasValue = VP880_VAS_MAX - (batteryOverage - (batteryOverage % VP880_VAS_STEP));
    }

    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("ComputeFinalVas: Overhead %d => rounded off final VAS %d",
         VP880_VAS_OVERHEAD, *vasValue));

    return;
}
#endif  /* #ifdef VP880_TRACKER_SUPPORT */

static void
SetAdc(
    VpLineCtxType *pLineCtx,
    VpDeviceIdType deviceId,
    uint8 ecVal,
    uint8 adcRoute)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp880DeviceObjectType *pDevObj = pLineCtx->pDevCtx->pDevObj;
    uint8 adcConfig;

    /* If the device mode was changed (by other channel), need to change it back
       so data can be taken. */
    if ((pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) &&
         (pDevObj->devMode[0] & VP880_DEV_MODE_TEST_DATA)) {
        pDevObj->devMode[0] &= ~(VP880_DEV_MODE_TEST_DATA);
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("SetAdc() Restoring device mode TDIM setting (was 0x%02X)", pDevObj->devMode[0]));
        VpMpiCmdWrapper(deviceId, ecVal, VP880_DEV_MODE_WRT, VP880_DEV_MODE_LEN,
            pDevObj->devMode);
    }

    VpMpiCmdWrapper(deviceId, ecVal, VP880_CONV_CFG_RD,
        VP880_CONV_CFG_LEN, &adcConfig);

    /* set the adc requested measurement do not adjust the sample rate */
    adcConfig = (adcConfig & ~VP880_CONV_CONNECT_BITS) | adcRoute;
    /* adcConfig = adcRoute; */

    VpMpiCmdWrapper(deviceId, ecVal, VP880_CONV_CFG_WRT,
        VP880_CONV_CFG_LEN, &adcConfig);

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
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp880DeviceObjectType *pDevObj = pLineCtx->pDevCtx->pDevObj;
    uint8 *adcLoopMax = &pLineObj->calLineData.typeData.loopData.adcLoopMax;
    uint8 reqAdcRoute = pLineObj->calLineData.typeData.loopData.adcRoute;
    uint8 currentAdc;

    /* If the device mode was changed (by other channel), need to change it back
       so data can be taken. */
    if ((pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) &&
         (pDevObj->devMode[0] & VP880_DEV_MODE_TEST_DATA)) {
        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("GetXData() Restoring device mode TDIM setting (was 0x%02X)", pDevObj->devMode[0]));
        pDevObj->devMode[0] &= ~(VP880_DEV_MODE_TEST_DATA);
        VpMpiCmdWrapper(deviceId, ecVal, VP880_DEV_MODE_WRT, VP880_DEV_MODE_LEN,
            pDevObj->devMode);
    }

    VpMpiCmdWrapper(deviceId, ecVal, VP880_CONV_CFG_RD,
        VP880_CONV_CFG_LEN, &currentAdc);

    if ((currentAdc & 0xF) == reqAdcRoute) {
        /*
         * if the ADC is still set to what we want it to
         * requested setting then take the measurement.
         */
        uint8 xdata[2];
        VpMpiCmdWrapper(deviceId, ecVal, VP880_TX_PCM_DATA_RD,
            VP880_TX_PCM_DATA_LEN, xdata);
        *pData = ( (((int16)xdata[0] << 8) & 0xFF00) | ((int16)xdata[1] & 0x00FF) );
        return TRUE;

    } else if ((*adcLoopMax)++ < 10) {
        /*
         * if the ADC is not correct but we have not exceeded
         * our loopcount, then set it again and get out.
        */
        uint8 adcAgain = (currentAdc & ~VP880_CONV_CONNECT_BITS) | reqAdcRoute;

        VpMpiCmdWrapper(deviceId, ecVal, VP880_CONV_CFG_WRT,
            VP880_CONV_CFG_LEN, &adcAgain);

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

/**
 * Vp880AdcSettling() -- Tracker and ABS Function
 *  This function read ADC/PCM and set the converter register
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling
 * this function.
 *
 * Postconditions:
 *  This function return the value pcm red.
 */
int16
Vp880AdcSettling(
    Vp880DeviceObjectType *pDevObj,
    uint8 ecVal,
    uint8 adcConfig,
    bool *validData)
{
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 xdataTemp[VP880_TX_PCM_DATA_LEN];
    int16 tempNew;
    uint8 cfg[VP880_CONV_CFG_LEN];

    VP_API_FUNC_INT(None, VP_NULL, ("Vp880AdcSettling+"));

    /*
     * If the device mode was changed (by other channel), need to change it back
     * so data can be taken.
     */
    if ((pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) &&
         (pDevObj->devMode[0] & VP880_DEV_MODE_TEST_DATA)) {
        pDevObj->devMode[0] &= ~(VP880_DEV_MODE_TEST_DATA);
        VpMpiCmdWrapper(deviceId, ecVal, VP880_DEV_MODE_WRT, VP880_DEV_MODE_LEN,
            pDevObj->devMode);
    }

    if (validData != VP_NULL) {
        VpMpiCmdWrapper(deviceId, ecVal, VP880_CONV_CFG_RD, VP880_CONV_CFG_LEN, cfg);
        if (cfg[0] != adcConfig) {
            *validData = FALSE;
            VP_CALIBRATION(None, NULL,("Vp880AdcSettling() - CONVERTER FAIL (0x%02X -> should be 0x%02X) on ecVal 0x%02X",
                cfg[0], adcConfig, ecVal));
            VpMpiCmdWrapper(deviceId, ecVal, VP880_CONV_CFG_WRT,
                VP880_CONV_CFG_LEN, &adcConfig);
        } else {
            *validData = TRUE;
        }
    }

    VpMpiCmdWrapper(deviceId, ecVal, VP880_TX_PCM_DATA_RD, VP880_TX_PCM_DATA_LEN,
        xdataTemp);
    tempNew = ((xdataTemp[0] << 8) | xdataTemp[1]);
    VP_CALIBRATION(None, NULL,("Vp880AdcSettling(adcConfig = 0x%02X): AdcPcm %d ecVal 0x%02X",
        adcConfig, tempNew, ecVal));

    VP_API_FUNC_INT(None, VP_NULL, ("Vp880AdcSettling-"));
    return tempNew;
}   /* Vp880AdcSettling() */
#endif  /* #if defined (VP_CSLAC_RUNTIME_CAL_ENABLED) && defined (VP880_FXS_SUPPORT) */

#ifdef VP880_TRACKER_SUPPORT
/**
 * Vp880BatteryCalAdjust()
 *  This function computes the battery calibration error and adjust the device register and object
 * content.
 *
 * Preconditions:
 *  The device and line context must be created and initialized before calling this function.
 *
 * Postconditions:
 *  Battery calibration registers are adjusted. Device object is updated.
 */
void
Vp880BatteryCalAdjust(
    Vp880DeviceObjectType *pDevObj,
    uint8 ecVal)
{
    /* evCal should only be EC_CH1 or EC_CH2, but make sure we're evaluating only the "EC" bits */
    uint8 channelId = (((ecVal & VP880_EC_BITS_MASK) == VP880_EC_CH1) ? 0 : 1);
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 swCal[VP880_BAT_CALIBRATION_LEN];
    uint16 swCalError;
    int32 abvError = pDevObj->vp880SysCalData.abvError[channelId];
    uint8 wbMode = (pDevObj->ecVal & VP880_WBAND_MODE_MASK);

    abvError *= 10000L; /* abvError in PCM * 1,000 scale; 1 = 7324mV */
    abvError /= VP880_V_PCM_LSB;    /* abvError in PCM scale; 1 = 7.324mV */

    VP_API_FUNC_INT(None, VP_NULL, ("Vp880BatteryCalAdjust+"));

    /* Read and clear the bits for Battery Calibration Settings */
    VpMpiCmdWrapper(deviceId, (wbMode | ecVal), VP880_BAT_CALIBRATION_RD,
        VP880_BAT_CALIBRATION_LEN, swCal);
    swCal[0] &= ~(VP880_BAT_CAL_SWCAL_MASK);

    /* Conversion from 7.324mV to 1.25V */
    swCalError = (ABS(abvError) / 171L); /* 171 * 7.324mV (PCM Vbat scale) = 1.25V (1 step) */
    if (((ABS(abvError) + 85L) /  171L) > swCalError) { /* 85 ~ 171/2 - point exceeding 1/2 step */
        swCalError+=1;
    }
    swCalError = (swCalError > 3) ? 3 : swCalError;
    swCal[0] |= (swCalError << 3);

    VP_CALIBRATION(None, VP_NULL,
        ("Ch %d: abvError %li :  swCalError %i", channelId, abvError, swCalError));

    /*
     * Positive error means voltage is too low (not negative enough).
     * Positive adjustment makes the battery voltage more negative.
     */
    swCal[0] |= (abvError > 0) ? 0 : VP880_BAT_CAL_SWCAL_SIGN;

    VP_CALIBRATION(None, VP_NULL,
        ("Ch %d: Battery Calibration Correction 0x%02X 0x%02X",
        channelId, swCal[0], swCal[1]));

    /*
     * This write MUST take into account Wideband setting because it can occur during normal system
     * operation using "Apply System Coefficients". If called during normal calibration, the value
     * of pDevObj must be cleared of the Wideband setting.
     */
    VpMpiCmdWrapper(deviceId, (wbMode | ecVal), VP880_BAT_CALIBRATION_WRT,
        VP880_BAT_CALIBRATION_LEN, swCal);

    VP_API_FUNC_INT(None, VP_NULL, ("Vp880BatteryCalAdjust-"));

    return;
}   /* Vp880BatteryCalAdjust() */
#endif  /* #ifdef VP880_TRACKER_SUPPORT */

/**
 * Vp880AdjustIla() -- Tracker and ABS Function
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
Vp880AdjustIla(
    VpLineCtxType *pLineCtx,
    uint8 targetIla)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    int32 imtMeasured = 0;
    uint8 imtTarget;

    int16 ilaError, imtActual;
    uint8 ilaAdjust;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880AdjustIla+"));

    /*
     * This function can run IF we're currently in calibration OR if the line
     * has been previously calibrated.
     */
    if ((pLineObj->calLineData.calDone == FALSE)        /* Line not previously calibrated */
    &&  (!(pLineObj->status & VP880_LINE_IN_CAL))) {    /* Not currently in cal */
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880AdjustIla-"));
        return FALSE;
    }

    if (targetIla < 4) {            /* 18mA to < 22mA */
        imtMeasured = pDevObj->vp880SysCalData.ila20[channelId];
        imtTarget = 20;
    } else if (targetIla < 10) {    /* 23mA to < 28mA */
        imtMeasured = pDevObj->vp880SysCalData.ila25[channelId];
        imtTarget = 25;
    } else if (targetIla < 18) {    /* 29mA to < 36mA */
        imtMeasured = pDevObj->vp880SysCalData.ila32[channelId];
        imtTarget = 32;
    } else {                        /* 36mA and higher */
        imtMeasured = pDevObj->vp880SysCalData.ila40[channelId];
        imtTarget = 40;
    }

    /* Always force ref ILA into the the normal and rev polarity feeds */
    pLineObj->calLineData.dcFeed[VP880_ILA_INDEX] &= ~VP880_ILA_MASK;
    pLineObj->calLineData.dcFeed[VP880_ILA_INDEX] |=
        (pLineObj->calLineData.dcFeedRef[VP880_ILA_INDEX] & VP880_ILA_MASK);

    pLineObj->calLineData.dcFeedPr[VP880_ILA_INDEX] &= ~VP880_ILA_MASK;
    pLineObj->calLineData.dcFeedPr[VP880_ILA_INDEX] |=
        (pLineObj->calLineData.dcFeedRef[VP880_ILA_INDEX] & VP880_ILA_MASK);

    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("Vp880AdjustIla: dcFeedPr 0x%02X 0x%02X",
         pLineObj->calLineData.dcFeedPr[0], pLineObj->calLineData.dcFeedPr[1]));
    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("Vp880AdjustIla: dcFeed 0x%02X 0x%02X",
         pLineObj->calLineData.dcFeed[0], pLineObj->calLineData.dcFeed[1]));

    /* determine the error */
    imtActual = imtMeasured - pDevObj->vp880SysCalData.ilaOffsetNorm[channelId];
    ilaError = imtActual - (imtTarget * VP880_ILA_SCALE_1MA);

    /*
     * only make an adjustment only if the error is greater
     * than or equal to 500uA = 273 at PCM
     */
    if (ABS(ilaError) >= (VP880_ILA_SCALE_1MA / 2)) {

        uint8 tempIlaValue = pLineObj->calLineData.dcFeedRef[VP880_ILA_INDEX] & VP880_ILA_MASK;
        int8 tempLowValue = (int8)tempIlaValue;
        ilaAdjust = ((ABS(ilaError)+(VP880_ILA_SCALE_1MA / 2)) / VP880_ILA_SCALE_1MA);

        if (ilaError < 0) {
            tempIlaValue += ilaAdjust;
            if (tempIlaValue <= VP880_ILA_MASK) {
                pLineObj->calLineData.dcFeed[VP880_ILA_INDEX] += ilaAdjust;
            } else {
                pLineObj->calLineData.dcFeed[VP880_ILA_INDEX] |= VP880_ILA_MASK;
            }
        } else {
            tempLowValue -= ilaAdjust;
            if (tempLowValue >= 0) {
                pLineObj->calLineData.dcFeed[VP880_ILA_INDEX] -= ilaAdjust;
            } else {
                pLineObj->calLineData.dcFeed[VP880_ILA_INDEX] &= ~VP880_ILA_MASK;
            }
        }
    }

    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Chan %d: ILA Actual Norm (10uA) %d",
        channelId, (imtActual * 100 / VP880_ILA_SCALE_1MA)));

    VP_CALIBRATION(VpLineCtxType, pLineCtx,
        ("Chan %d: ILA Target (10uA) %d ILA Error Norm (10uA) %d",
        channelId, (((targetIla + 18) * 100)), (ilaError * 100 / VP880_ILA_SCALE_1MA)));

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880AdjustIla-"));
    return TRUE;
}   /* Vp880AdjustIla() */

/**
 * Vp880AdjustVoc() -- Tracker and ABS Function
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
Vp880AdjustVoc(
    VpLineCtxType *pLineCtx,
    uint8 targetVoc,
    bool previousCal)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    int16 vocActual, vocActualRev;
    int32 pcmTargetVoc;

    int16 vocErrorList[VP880_NUM_POLARITY];
    uint8 vocErrIndex;

    uint8 *dcFeedByte[VP880_NUM_POLARITY];

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880AdjustVoc+"));

    dcFeedByte[VP880_NORM_POLARITY] = &(pLineObj->calLineData.dcFeed[0]);
    dcFeedByte[VP880_REV_POLARITY] = &(pLineObj->calLineData.dcFeedPr[0]);

    pcmTargetVoc = (int32)(targetVoc * 3L);
    pcmTargetVoc += 36;
    pcmTargetVoc *= VP880_V_1V_SCALE;
    pcmTargetVoc /= VP880_V_1V_RANGE;

    if (previousCal == FALSE) {
        vocActual =
            pLineObj->calLineData.typeData.vocData.vocNorm
          - pDevObj->vp880SysCalData.vocOffset[channelId][VP880_NORM_POLARITY];

        vocActualRev =
            pLineObj->calLineData.typeData.vocData.vocRev
          - pDevObj->vp880SysCalData.vocOffset[channelId][VP880_REV_POLARITY];

        /*
         * Target is always positive. Normal feed is positive. Negative error means
         * voltage is too low (magnitude), positive means too high (magnitude).
         */
        pDevObj->vp880SysCalData.vocError[channelId][VP880_NORM_POLARITY] =
            (vocActual - (int16)pcmTargetVoc);

        VP_CALIBRATION(VpLineCtxType, pLineCtx,
            ("(In 10mV): VOC Norm %ld Rev %ld OffsetNorm %ld OffsetRev %ld Channel %d",
            (pLineObj->calLineData.typeData.vocData.vocNorm  * VP880_V_PCM_LSB/VP880_V_SCALE),
            (pLineObj->calLineData.typeData.vocData.vocRev  * VP880_V_PCM_LSB/VP880_V_SCALE),
            (pDevObj->vp880SysCalData.vocOffset[channelId][VP880_NORM_POLARITY]  * VP880_V_PCM_LSB/VP880_V_SCALE),
            (pDevObj->vp880SysCalData.vocOffset[channelId][VP880_REV_POLARITY]  * VP880_V_PCM_LSB/VP880_V_SCALE),
            channelId));

        VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Chan %d: VOC (10mV) Actual Norm %ld Rev Norm %ld",
            channelId,
            (vocActual  * VP880_V_PCM_LSB/VP880_V_SCALE),
            (vocActualRev  * VP880_V_PCM_LSB/VP880_V_SCALE)));

        /*
         * Target is always positive. Reverse feed is negative. Negative error means
         * voltage is too low (magnitude), positive means too high (magnitude).
         */
        pDevObj->vp880SysCalData.vocError[channelId][VP880_REV_POLARITY] =
            (-vocActualRev - (int16)pcmTargetVoc);

        VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Chan %d: VOC Target %d VOC Error Norm %ld Error Rev %ld",
            channelId, (((uint16)targetVoc * 3 + 36) * 100),
            (pDevObj->vp880SysCalData.vocError[channelId][VP880_NORM_POLARITY]  * VP880_V_PCM_LSB/VP880_V_SCALE),
            (pDevObj->vp880SysCalData.vocError[channelId][VP880_REV_POLARITY]  * VP880_V_PCM_LSB/VP880_V_SCALE)));

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
    vocErrorList[VP880_NORM_POLARITY] = pDevObj->vp880SysCalData.vocError[channelId][VP880_NORM_POLARITY];
    vocErrorList[VP880_REV_POLARITY] = pDevObj->vp880SysCalData.vocError[channelId][VP880_REV_POLARITY];

    for (vocErrIndex = 0; vocErrIndex < VP880_NUM_POLARITY; vocErrIndex++) {
        /* VOC Scale: 1.5V = 204.8 at PCM. Adjust to account for bit shift */

        if (ABS(vocErrorList[vocErrIndex]) >= 205) {
            if (vocErrorList[vocErrIndex] < 0) {
                /* Error is low, so need to increase VOC */

                /* Saturate the value, to prevent the rollover */
                if ((*dcFeedByte[vocErrIndex] & VP880_VOC_MASK) != VP880_VOC_MASK) {

                    /* Not saturated within scale. So can adjust up */
                    *dcFeedByte[vocErrIndex] += 0x04;

                } else if ((*dcFeedByte[vocErrIndex] & VP880_VOC_LOW_RANGE) ==
                           VP880_VOC_LOW_RANGE) {

                    /*
                     * Saturated within scale, but not within device. Change
                     * scale (moves up 3V) and clear incremental values or we'll
                     *  end up at the top of the high range.
                     */
                    *dcFeedByte[vocErrIndex] &= ~VP880_VOC_MASK;
                    *dcFeedByte[vocErrIndex] &= ~VP880_VOC_LOW_RANGE;
                }
            } else {
                /* Error is high, so need to decrease VOC */

                /* Saturate the value, to prevent the rollover */
                if ((*dcFeedByte[vocErrIndex] & VP880_VOC_MASK) != 0x00) {
                    /* Not saturated within scale. So can adjust down */
                    *dcFeedByte[vocErrIndex] -= 0x04;
                } else if ((*dcFeedByte[vocErrIndex] & VP880_VOC_LOW_RANGE) !=
                    VP880_VOC_LOW_RANGE) {

                    /*
                     * Saturated within scale, but not within device. Change
                     * scale (moves down 3V) and max incremental values or we'll
                     *  end up at the bottom of the low range.
                     */
                    *dcFeedByte[vocErrIndex] |= VP880_VOC_MASK;
                    *dcFeedByte[vocErrIndex] |= VP880_VOC_LOW_RANGE;
                }
            }
        }
    }

    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp880AdjustVoc() Chan %d: DC Feed Values Normal After 0x%02X 0x%02X",
        channelId, pLineObj->calLineData.dcFeed[0],
        pLineObj->calLineData.dcFeed[1]));

    VP_CALIBRATION(VpLineCtxType, pLineCtx, ("Vp880AdjustVoc() Chan %d: DC Feed Values Reverse After 0x%02X 0x%02X",
        channelId, pLineObj->calLineData.dcFeedPr[0],
        pLineObj->calLineData.dcFeedPr[1]));

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880AdjustVoc-"));

    return TRUE;
}   /* Vp880AdjustVoc() */

/**
 * Vp880Cal() -- Tracker and ABS Function
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
Vp880Cal(
    VpLineCtxType       *pLineCtx,
    VpCalType           calType,
    void                *inputArgs)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 profileIndex;
    uint8 *profileData;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880Cal+"));

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    switch(calType) {
        case VP_CAL_GET_SYSTEM_COEFF:
            if (pDevObj->stateInt & VP880_SYS_CAL_COMPLETE) {
                /* Data length is header (6 bytes) +  880 calibration data */
                pDevObj->mpiLen = 6 + VP880_CAL_STRUCT_SIZE;

                pLineObj->responseData = (uint8)VP_CAL_GET_SYSTEM_COEFF;
                pLineObj->lineEvents.response |= VP_EVID_CAL_CMP;
            } else {
                status = VP_STATUS_LINE_NOT_CONFIG;
            }
            break;

        case VP_CAL_APPLY_SYSTEM_COEFF:
            profileData = (uint8 *)inputArgs;

            if (profileData == VP_NULL) {
                VpMemSet(&pDevObj->vp880SysCalData, 0, sizeof(Vp880SysCalResultsType));
                pDevObj->stateInt &= ~(VP880_SYS_CAL_COMPLETE | VP880_CAL_RELOAD_REQ | VP880_DEVICE_CAL_COMPLETE);
                pDevObj->stateInt |= VP880_SYS_CAL_RESET;
                pLineObj->lineEvents.response |= VP_EVID_CAL_CMP;
                pLineObj->responseData = VP_CAL_SUCCESS;
                VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880Cal-"));
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_SUCCESS;
            }

            if ((profileData[VP_PROFILE_TYPE_LSB] != VP_PRFWZ_PROFILE_CAL) ||
                (profileData[VP_PROFILE_TYPE_MSB] != VP_DEV_880_SERIES) ||
                (profileData[VP_PROFILE_LENGTH] < (VP880_CAL_STRUCT_SIZE + 2))) {
                VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880Cal-"));
                VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                return VP_STATUS_INVALID_ARG;
            }

            profileIndex = VP_PROFILE_DATA_START;

            pDevObj->vp880SysCalData.abvError[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ABV Y Error %d", pDevObj->vp880SysCalData.abvError[0]));

            pDevObj->vp880SysCalData.abvError[1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ABV Z Error %d", pDevObj->vp880SysCalData.abvError[1]));

            pDevObj->vp880SysCalData.vocOffset[0][0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Offset Norm Ch 0 %d", pDevObj->vp880SysCalData.vocOffset[0][0]));

            pDevObj->vp880SysCalData.vocError[0][0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Error Norm Ch 0 %d", pDevObj->vp880SysCalData.vocError[0][0]));

            pDevObj->vp880SysCalData.vocOffset[0][1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Offset Rev Ch 0 %d", pDevObj->vp880SysCalData.vocOffset[0][1]));

            pDevObj->vp880SysCalData.vocError[0][1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Error Rev Ch 0 %d", pDevObj->vp880SysCalData.vocError[0][1]));

            pDevObj->vp880SysCalData.vocOffset[1][0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Offset Norm Ch 1 %d", pDevObj->vp880SysCalData.vocOffset[1][0]));

            pDevObj->vp880SysCalData.vocError[1][0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Error Norm Ch 1 %d", pDevObj->vp880SysCalData.vocError[1][0]));

            pDevObj->vp880SysCalData.vocOffset[1][1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Offset Rev Ch 1 %d", pDevObj->vp880SysCalData.vocOffset[1][1]));

            pDevObj->vp880SysCalData.vocError[1][1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VOC Error Rev Ch 1 %d", pDevObj->vp880SysCalData.vocError[1][1]));

            pDevObj->vp880SysCalData.sigGenAError[0][0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SigGenA Norm Ch 0 Error %d", pDevObj->vp880SysCalData.sigGenAError[0][0]));

            pDevObj->vp880SysCalData.sigGenAError[0][1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SigGenA Rev Ch 0 Error %d", pDevObj->vp880SysCalData.sigGenAError[0][1]));

            pDevObj->vp880SysCalData.sigGenAError[1][0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SigGenA Norm Ch 1 Error %d", pDevObj->vp880SysCalData.sigGenAError[1][0]));

            pDevObj->vp880SysCalData.sigGenAError[1][1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SigGenA Rev Ch 1 Error %d", pDevObj->vp880SysCalData.sigGenAError[1][1]));

            pDevObj->vp880SysCalData.ila20[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA 20mA Ch 0 %d", pDevObj->vp880SysCalData.ila20[0]));

            pDevObj->vp880SysCalData.ila20[1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA 20mA Ch 1 %d", pDevObj->vp880SysCalData.ila20[1]));

            pDevObj->vp880SysCalData.ila25[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA 25mA Ch 0 %d", pDevObj->vp880SysCalData.ila25[0]));

            pDevObj->vp880SysCalData.ila25[1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA 25mA Ch 1 %d", pDevObj->vp880SysCalData.ila25[1]));

            pDevObj->vp880SysCalData.ila32[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA 32mA Ch 0 %d", pDevObj->vp880SysCalData.ila32[0]));

            pDevObj->vp880SysCalData.ila32[1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA 32mA Ch 1 %d", pDevObj->vp880SysCalData.ila32[1]));

            pDevObj->vp880SysCalData.ila40[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA 40mA Ch 0 %d", pDevObj->vp880SysCalData.ila40[0]));

            pDevObj->vp880SysCalData.ila40[1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA 40mA Ch 1 %d", pDevObj->vp880SysCalData.ila40[1]));

            pDevObj->vp880SysCalData.ilaOffsetNorm[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA Offset Ch 0 %d", pDevObj->vp880SysCalData.ilaOffsetNorm[0]));

            pDevObj->vp880SysCalData.ilaOffsetNorm[1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILA Offset Ch 1 %d", pDevObj->vp880SysCalData.ilaOffsetNorm[1]));

            pDevObj->vp880SysCalData.ilgOffsetNorm[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILG Offset Ch 0 %d", pDevObj->vp880SysCalData.ilgOffsetNorm[0]));

            pDevObj->vp880SysCalData.ilgOffsetNorm[1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("ILG Offset Ch 1 %d", pDevObj->vp880SysCalData.ilgOffsetNorm[1]));

            pDevObj->vp880SysCalData.vas[0][0] = profileData[profileIndex++];
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAS Norm Ch 0 %d", pDevObj->vp880SysCalData.vas[0][0]));

            pDevObj->vp880SysCalData.vas[0][1] = profileData[profileIndex++];
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAS Rev Ch 0 %d", pDevObj->vp880SysCalData.vas[0][1]));

            pDevObj->vp880SysCalData.vas[1][0] = profileData[profileIndex++];
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAS Norm Ch 1 %d", pDevObj->vp880SysCalData.vas[1][0]));

            pDevObj->vp880SysCalData.vas[1][1] = profileData[profileIndex++];
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAS Rev Ch 1 %d", pDevObj->vp880SysCalData.vas[1][1]));

            pDevObj->vp880SysCalData.vagOffsetNorm[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAG Offset Norm Ch 0 %d", pDevObj->vp880SysCalData.vagOffsetNorm[0]));

            pDevObj->vp880SysCalData.vagOffsetNorm[1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAG Offset Norm Ch 1 %d", pDevObj->vp880SysCalData.vagOffsetNorm[1]));

            pDevObj->vp880SysCalData.vagOffsetRev[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAG Offset Rev Ch 0 %d", pDevObj->vp880SysCalData.vagOffsetRev[0]));

            pDevObj->vp880SysCalData.vagOffsetRev[1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VAG Offset Rev Ch 1 %d", pDevObj->vp880SysCalData.vagOffsetRev[1]));

            pDevObj->vp880SysCalData.vbgOffsetNorm[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VBG Offset Norm Ch 0 %d", pDevObj->vp880SysCalData.vbgOffsetNorm[0]));

            pDevObj->vp880SysCalData.vbgOffsetNorm[1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VBG Offset Norm Ch 1 %d", pDevObj->vp880SysCalData.vbgOffsetNorm[1]));

            pDevObj->vp880SysCalData.vbgOffsetRev[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VBG Offset Rev Ch 0 %d", pDevObj->vp880SysCalData.vbgOffsetRev[0]));

            pDevObj->vp880SysCalData.vbgOffsetRev[1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("VBG Offset Rev Ch 1 %d", pDevObj->vp880SysCalData.vbgOffsetRev[1]));

            pDevObj->vp880SysCalData.absNormCal[0] = profileData[profileIndex++];
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Auto-Bat Switch Norm Ch 0 Cal %d", pDevObj->vp880SysCalData.absNormCal[0]));

            pDevObj->vp880SysCalData.absPolRevCal[0] = profileData[profileIndex++];
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Auto-Bat Switch Rev Ch 0 Cal %d", pDevObj->vp880SysCalData.absPolRevCal[0]));

            pDevObj->vp880SysCalData.absNormCal[1] = profileData[profileIndex++];
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Auto-Bat Switch Norm Ch 1 Cal %d", pDevObj->vp880SysCalData.absNormCal[1]));

            pDevObj->vp880SysCalData.absPolRevCal[1] = profileData[profileIndex++];
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Auto-Bat Switch Rev Ch 1 Cal %d", pDevObj->vp880SysCalData.absPolRevCal[1]));

            pDevObj->vp880SysCalData.swyOffset[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SWY Offset Ch 0 %d", pDevObj->vp880SysCalData.swyOffset[0]));

            pDevObj->vp880SysCalData.swyOffset[1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SWY Offset Ch 1 %d", pDevObj->vp880SysCalData.swyOffset[1]));

            pDevObj->vp880SysCalData.swzOffset[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SWZ Offset Ch 0 %d", pDevObj->vp880SysCalData.swzOffset[0]));

            pDevObj->vp880SysCalData.swzOffset[1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("SWZ Offset Ch 1 %d", pDevObj->vp880SysCalData.swzOffset[1]));

            pDevObj->vp880SysCalData.swxbOffset[0] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("XB Offset Ch 0 %d", pDevObj->vp880SysCalData.swxbOffset[0]));

            pDevObj->vp880SysCalData.swxbOffset[1] = VpConvertToInt16(&profileData[profileIndex]);
            profileIndex+=2;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("XB Offset Ch 1 %d", pDevObj->vp880SysCalData.swxbOffset[1]));

            pDevObj->vp880SysCalData.tipCapCal[0] = VpConvertToInt32(&profileData[profileIndex]);
            profileIndex+=4;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Tip Cap Ch 0 %li", pDevObj->vp880SysCalData.tipCapCal[0]));

            pDevObj->vp880SysCalData.tipCapCal[1] = VpConvertToInt32(&profileData[profileIndex]);
            profileIndex+=4;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Tip Cap Ch 1 %li", pDevObj->vp880SysCalData.tipCapCal[1]));

            pDevObj->vp880SysCalData.ringCapCal[0] = VpConvertToInt32(&profileData[profileIndex]);
            profileIndex+=4;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Ring Cap Ch 0 %li", pDevObj->vp880SysCalData.ringCapCal[0]));

            pDevObj->vp880SysCalData.ringCapCal[1] = VpConvertToInt32(&profileData[profileIndex]);
            profileIndex+=4;
            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Ring Cap Ch 1 %li", pDevObj->vp880SysCalData.ringCapCal[1]));

            VP_CALIBRATION(VpDevCtxType, pDevCtx, ("Calibration Data Length - %d", profileIndex));

            pDevObj->stateInt |= (VP880_SYS_CAL_COMPLETE | VP880_CAL_RELOAD_REQ | VP880_DEVICE_CAL_COMPLETE);
            pLineObj->lineEvents.response |= VP_EVID_CAL_CMP;
            pLineObj->responseData = VP_CAL_SUCCESS;
            break;

        default:
            status = VP_STATUS_INVALID_ARG;
            break;
    }

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880Cal-"));
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return status;
}   /* Vp880Cal() */
#endif /* VP_CC_880_SERIES */
