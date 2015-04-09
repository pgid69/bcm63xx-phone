/** \file vp890_control.c
 * vp890_control.c
 *
 *  This file contains the implementation of the VP-API 890 Series
 *  Common Control Functions.
 *
 * Copyright (c) 2012, Microsemi Corporation
 *
 * $Revision: 11432 $
 * $LastChangedDate: 2014-05-29 12:50:35 -0500 (Thu, 29 May 2014) $
 */

/* INCLUDES */
#include    "vp_api.h"

#if defined (VP_CC_890_SERIES)  /* Compile only if required */

#include    "vp_api_int.h"
#include    "vp890_api_int.h"

/* =================================
    Prototypes for Static Functions
   ================================= */
#ifdef VP890_FXS_SUPPORT
static void
Vp890RestartComplete(
    VpDevCtxType *pDevCtx);
#endif

static void
Vp890ServiceInterrupts(
    VpDevCtxType            *pDevCtx);

static void
Vp890ServiceDevTimers(
    VpDevCtxType            *pDevCtx);

static bool
Vp890FindSoftwareInterrupts(
    VpDevCtxType            *pDevCtx);

static VpStatusType
SetDeviceOption(
    VpDevCtxType            *pDevCtx,
    VpDeviceIdType          deviceId,
    VpOptionIdType          option,
    void                    *value);

static VpStatusType
SetLineOption(
    VpLineCtxType           *pLineCtx,
    VpDeviceIdType          deviceId,
    VpOptionIdType          option,
    void                    *value);

static void
SetOptionEventMask(
    Vp890DeviceObjectType   *pDevObj,
    Vp890LineObjectType     *pLineObj,
    VpDeviceIdType          deviceId,
    uint8                   ecVal,
    void                    *value);

#ifdef VP890_FXS_SUPPORT
static VpStatusType
SetOptionDcFeedParams(
    VpLineCtxType           *pLineCtx,
    VpOptionDcFeedParamsType *pDcFeedParams);

static VpStatusType
SetOptionRingingParams(
    VpLineCtxType           *pLineCtx,
    VpOptionRingingParamsType *pRingingParams);
#endif


/* Function called by SetOptionInternal for Event Masking only */
static void
MaskNonSupportedEvents(
    VpOptionEventMaskType   *pLineEventsMask,
    VpOptionEventMaskType   *pDevEventsMask);

/* Function called by SetOptionInternal to set tx and rx timeslot */
static VpStatusType
SetTimeSlot(
    VpLineCtxType           *pLineCtx,
    uint8                   txSlot,
    uint8                   rxSlot);

/**< Profile index for Generator A/B and C/D starting points (std tone) */
typedef enum vp890_toneProfileParams {
    VP890_SIGGEN_AB_START = 8,
    VP890_SIGGEN_CD_START = 16,
    VP890_SIGGEN_ENUM_SIZE = FORCE_STANDARD_C_ENUM_SIZE /* Portability Req.*/
} vp890_toneProfileParams;

/*******************************************************************************
 * Vp890ApiTick()
 *  This function should be called on a periodic basis or attached to an
 * interrupt.
 *
 * Arguments:
 *
 * Preconditions:
 *  The device must first be initialized.
 *
 * Postconditions:
 *  The value passed (by pointer) is set to TRUE if there is an updated event.
 * The user should call the GetEventStatus function to determine the cause of
 * the event (TRUE value set).  This function always returns the success code.
 ******************************************************************************/
VpStatusType
Vp890ApiTick(
    VpDevCtxType        *pDevCtx,
    bool                *pEventStatus)
{
    VpLineCtxType           *pLineCtx;
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    Vp890LineObjectType     *pLineObj;
    VpDeviceIdType          deviceId    = pDevObj->deviceId;

    uint8                   channelId;
    uint8                   ecVal;
    uint8                   numIntServiced = 2;
    bool                    tempClkFault, tempBat1Fault, lineInTest, intServiced;

#ifdef VP_CSLAC_SEQ_EN
    bool isSeqRunning = FALSE;
#endif

    uint16                  timeStampPre, tickAdder;

    *pEventStatus = FALSE;
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * Can't allow tick functions to proceed until Init Device function has
     * been called. Otherwise, "tickrate" is unknown and initally 0.
     */
    if (pDevObj->devProfileData.tickRate == 0) {
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }
    /*
     * The timestamp is in 0.5mS increments, but the device tickrate is
     * something else. So increment by the scaled amount and detect rollover
     * by finding if the previous value is greater than the new value.
     */
    timeStampPre = pDevObj->timeStamp;
    tickAdder = pDevObj->devProfileData.tickRate / VP_CSLAC_TICKSTEP_0_5MS;
    pDevObj->timeStamp+=tickAdder;

    /*
     * Since the tickrate can be in steps that are not multiples of 0.5ms, determine the
     * roundoff remaining over the past several ticks and when it exceeds (0.5ms / 2), increase
     * the timestamp.
     */
    pDevObj->timeRemainder += (pDevObj->devProfileData.tickRate % VP_CSLAC_TICKSTEP_0_5MS);
    if (pDevObj->timeRemainder >= (VP_CSLAC_TICKSTEP_0_5MS / 2)) {
        pDevObj->timeRemainder -= VP_CSLAC_TICKSTEP_0_5MS;
        pDevObj->timeStamp += 1;
    }

    if (timeStampPre > pDevObj->timeStamp) {
        pDevObj->deviceEvents.signaling |= VP_DEV_EVID_TS_ROLLOVER;
    }

#if defined (VP890_INTERRUPT_LEVTRIG_MODE)
    VpSysEnableInt(deviceId);
#endif

    /* Check if since last tick, one of the lines changed to/from WideBand mode */
    if (pDevObj->lastCodecChange != 0) {
        VpOptionCodecType codecMode;
        /*
         * A wideband mode change was made. Figure out which was the last channel
         * changed and enforce that channel's codec setting on both channels.
         */
        pLineCtx = pDevCtx->pLineCtx[pDevObj->lastCodecChange-1];

        /*
         * Only way line context can be null here is if the codec mode was just
         * set and before calling the tick, the line context was set "free".
         * Highly unusual, but technically possible.
         */
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            codecMode = pLineObj->codec;

            if((codecMode == VP_OPTION_WIDEBAND) ||
               (codecMode == VP_OPTION_ALAW_WIDEBAND) ||
               (codecMode == VP_OPTION_MLAW_WIDEBAND)) {
                pDevObj->ecVal |= VP890_WIDEBAND_MODE;
            } else {
                pDevObj->ecVal &= ~VP890_WIDEBAND_MODE;
            }

            /* Force both lines and device to correct wideband mode */
            for(channelId=0; channelId < VP890_MAX_NUM_CHANNELS; channelId++ ) {
                uint8 codecReg[VP890_OP_FUNC_LEN];
                pLineCtx = pDevCtx->pLineCtx[channelId];
                if (pLineCtx == VP_NULL) {
                    continue;
                }
                pLineObj = pLineCtx->pLineObj;
                pLineObj->codec = codecMode;

                pLineObj->ecVal &= ~VP890_WIDEBAND_MODE;
                pLineObj->ecVal |= (pDevObj->ecVal & VP890_WIDEBAND_MODE);

                VpMpiCmdWrapper(deviceId, pLineObj->ecVal, VP890_OP_FUNC_RD, VP890_OP_FUNC_LEN,
                    codecReg);
                if (VpCSLACGetCodecReg(codecMode, codecReg)) {
                    /* Setting to *this* codec mode was ok. Make final settings and continue */
                    VpMpiCmdWrapper(deviceId, pLineObj->ecVal, VP890_OP_FUNC_WRT, VP890_OP_FUNC_LEN,
                        codecReg);
                    pLineObj->codec = codecMode;
                } else {
                    /* Setting to *this* codec mode failed. Don't proceed on current path. */
                    continue;
                }
            }
        }
        pDevObj->lastCodecChange = 0;
    }

    ecVal = pDevObj->ecVal;

    /* Vp890Service Device Timers */
    Vp890ServiceDevTimers(pDevCtx);

    /*
     * Preclear the lineInTest flag that is used to determine if the TX Buffer
     * needs to be read. It is always read if any line is in test.
     */
    lineInTest = FALSE;

    /* Vp890Service Line Timers and check to see if the sequencer needs to run */
    for(channelId=0; channelId < VP890_MAX_NUM_CHANNELS; channelId++ ) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx == VP_NULL) {
            continue;
        }
        pLineObj = pLineCtx->pLineObj;

        /* call the appropriate service timer */
        if ((pLineObj->status & VP890_IS_FXO)) {
#ifdef VP890_FXO_SUPPORT
            Vp890ServiceFxoTimers(pDevCtx, pLineCtx, pLineObj, deviceId, pLineObj->ecVal);
#endif
        } else {
#ifdef VP890_FXS_SUPPORT
            Vp890ServiceFxsTimers(pDevCtx, pLineCtx, pLineObj, deviceId, pLineObj->ecVal);

#ifdef VP890_INCLUDE_TESTLINE_CODE
            if (Vp890IsChnlUndrTst(pDevObj, channelId) == TRUE) {
                lineInTest = TRUE;

                /* Clear flag to indicate the generators are NOT in a Ringing Mode */
                pLineObj->status &= ~(VP890_RING_GEN_NORM | VP890_RING_GEN_REV);
            } else if (pDevObj->currentTest.nonIntrusiveTest == TRUE) {
                lineInTest = TRUE;
            }
#endif  /* VP890_INCLUDE_TESTLINE_CODE */
            if (pDevObj->stateInt & VP890_CAL_RELOAD_REQ) {
                Vp890UpdateCalValue(pLineCtx);
            }
#endif  /* VP890_FXS_SUPPORT */
        }
#ifdef VP_CSLAC_SEQ_EN
        /* Evaluate if Cadencing is required */
        if ((pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) == VP_CADENCE_STATUS_ACTIVE) {
            VP_SEQUENCER(VpLineCtxType, pLineCtx, ("Line %d is Active", channelId));
            isSeqRunning = TRUE;
        }
#endif
    }

#ifdef VP890_LP_SUPPORT
    if (!(pDevObj->stateInt & VP890_IS_FXO_ONLY)) {
        Vp890LowPowerMode(pDevCtx);
    }
#endif

    pDevObj->stateInt &= ~VP890_CAL_RELOAD_REQ;

    /* Reset event pointers pointers */
    pDevObj->dynamicInfo.lastChan = 0;

#ifdef VP_CSLAC_SEQ_EN
    if (isSeqRunning == TRUE) {
        VpServiceSeq(pDevCtx);
    }
#endif

    /*
     * Test the interrupt to see if there is a pending interrupt.  If there is,
     * read the interrupt registers (if running in an interrupt driven mode).
     * If running in polled mode, automatically read the interrupt/status
     * registers.
     */

#if defined (VP890_EFFICIENT_POLLED_MODE)
    /* Poll the device PIO-INT line */
    pDevObj->state |= (VpSysTestInt(deviceId) ? VP_DEV_PENDING_INT : 0x00);
#elif defined (VP890_SIMPLE_POLLED_MODE)
    pDevObj->state |= VP_DEV_PENDING_INT;
#endif

    intServiced = FALSE;

    /* Read this buffer once per tick IF there is a line under test. */
    if (lineInTest == TRUE) {
        /* Collect data from the device test data buffer */
        VpMpiCmdWrapper(deviceId, ecVal, VP890_TEST_DATA_RD, VP890_TEST_DATA_LEN,
            pDevObj->testDataBuffer);
        pDevObj->state |= VP_DEV_TEST_BUFFER_READ;
    } else {
        pDevObj->state &= ~VP_DEV_TEST_BUFFER_READ;
    }

    /* Service all pending interrupts (up to 2) */
    while ((pDevObj->state & VP_DEV_PENDING_INT) && (numIntServiced > 0)) {

        VpMpiCmdWrapper(deviceId, ecVal, VP890_UL_SIGREG_RD,
            VP890_UL_SIGREG_LEN, pDevObj->intReg);

        if (numIntServiced == 2) {
            VpMemCpy(pDevObj->intReg2, pDevObj->intReg, VP890_UL_SIGREG_LEN);
        }

        /*
         * Compare it with what we already know.  If different, generate
         * events and update the line status bits
         */
        intServiced = TRUE;
        Vp890ServiceInterrupts(pDevCtx);

        /*
         * If level triggered, the interrupt may have been disabled (to prevent
         * a flood of interrupts), so reenable it.
         */
    #if defined (VP890_INTERRUPT_LEVTRIG_MODE)
        VpSysEnableInt(deviceId);
    #endif

        /* Clear the current interrupt indication */
        pDevObj->state &= ~(VP_DEV_PENDING_INT);
        numIntServiced--;

        /*
         * If operating in Efficient Polled Mode, check to see if the interrupt
         * line is still indicating an active interrupt. If in simple polled mode,
         * repeat the loop and service interrupts (if anything is changed).
         */
    #if defined (VP890_EFFICIENT_POLLED_MODE)
        /* Poll the PIO-INT line */
        pDevObj->state |=
            (VpSysTestInt(deviceId) ? VP_DEV_PENDING_INT : 0x00);
    #elif defined (VP890_SIMPLE_POLLED_MODE)
        pDevObj->state |= VP_DEV_PENDING_INT;
    #endif
    }/* End while Interrupts*/

    /* Make sure Vp890ServiceInterrupts() is called at least once per tick to
     * keep the API line status up to date */
    if (intServiced == FALSE) {
        Vp890ServiceInterrupts(pDevCtx);
    }

    /* Update the dial pulse handler for lines that are set for pulse decode,
     * and process events for FXO lines */
    for (channelId = 0; channelId < VP890_MAX_NUM_CHANNELS; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;

            if (pLineObj->status & VP890_INIT_COMPLETE) {
                if (!(pLineObj->status & VP890_IS_FXO)) {
#ifdef VP890_FXS_SUPPORT
                    Vp890ProcessFxsLine(pDevObj, pLineCtx);
#endif
                } else {
#ifdef VP890_FXO_SUPPORT
                    Vp890ProcessFxoLine(pDevObj, pLineCtx);
#endif
                }
            }
        }
    }
    /*******************************************************
     *      HANDLE Clock Fail and Battery Fault Events     *
     *******************************************************/
    /* Get the current status of the clock fault bit */
    if (!(pDevObj->devTimer[VP_DEV_TIMER_WB_MODE_CHANGE] & VP_ACTIVATE_TIMER)) {
        tempClkFault = (pDevObj->intReg[0] & VP890_CFAIL_MASK) ? TRUE : FALSE;
        /*
         * Compare it with what we already know.  If different, generate event
         */
        if(tempClkFault ^ pDevObj->dynamicInfo.clkFault) {
#ifdef VP890_FXS_SUPPORT
            if (!(pDevObj->stateInt & VP890_FORCE_FREE_RUN)) {
                if (tempClkFault) {
                    /* Entering clock fault, possibly a system restart. */
                    Vp890FreeRun(pDevCtx, VP_FREE_RUN_START);

                    /*
                     * Clear the flag used to indicate that Vp890FreeRun() was
                     * called by the application -- because it wasn't.
                     */
                    pDevObj->stateInt &= ~VP890_FORCE_FREE_RUN;
                } else {
                    /*
                     * Exiting clock fault (note: this function does not affect
                     * VP890_FORCE_FREE_RUN flag).
                     */
                    Vp890RestartComplete(pDevCtx);
                }

            }
#endif  /* VP890_FXS_SUPPORT */
            pDevObj->dynamicInfo.clkFault = tempClkFault;
            pDevObj->deviceEvents.faults |= VP_DEV_EVID_CLK_FLT;
            pDevObj->eventHandle = pDevObj->timeStamp;
        }
    }

    /* Get the current status of the battery fault bit */
    tempBat1Fault = (pDevObj->intReg[0] & VP890_OCALMY_MASK) ? TRUE : FALSE;
    /*
     * Compare it with what we already know.  If different, generate event
     */
    if(tempBat1Fault ^ pDevObj->dynamicInfo.bat1Fault) {
        pDevObj->dynamicInfo.bat1Fault = tempBat1Fault;
        pDevObj->deviceEvents.faults |= VP_DEV_EVID_BAT_FLT;
        pDevObj->eventHandle = pDevObj->timeStamp;
    }

    /* Collect all event activity and report to the calling function */
    if (Vp890FindSoftwareInterrupts(pDevCtx)) {
        *pEventStatus = TRUE;
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
} /* Vp890ApiTick() */

/*******************************************************************************
 * Vp890IsChnlUndrTst()
 *  This function determines if a particular line of a device is currently
 * running a test.
 *
 * Preconditions:
 *  None.
 *
 * Postconditions:
 *  Device not affected. Return value TRUE if the line is currently running a
 * test, FALSE otherwise.
 ******************************************************************************/
bool
Vp890IsChnlUndrTst(
    Vp890DeviceObjectType *pDevObj,
    uint8 channelId)
{
#ifdef VP890_INCLUDE_TESTLINE_CODE
    if (pDevObj->currentTest.nonIntrusiveTest == FALSE) {
        if ((TRUE == pDevObj->currentTest.prepared) &&
            (channelId == pDevObj->currentTest.channelId)) {
            return TRUE;
        }
    }
#endif
    return FALSE;
}

#ifdef VP890_FXS_SUPPORT
/**
 * Vp890FreeRun()
 *  This function is called by the application when it wants to prepare the
 * system for a restart, or by the VP-API-II internally when a clock fault or
 * other "sign" of a restart is detected.
 *
 * Preconditions:
 *  Conditions defined by purpose of Api Tick.
 *
 * Postconditions:
 *  Device and line are in states 'ready" for a system reboot to occur. Lines
 * are set to VP_LINE_STANDBY if previously ringing.
 */
VpStatusType
Vp890FreeRun(
    VpDevCtxType *pDevCtx,
    VpFreeRunModeType freeRunMode)
{
    VpLineCtxType *pLineCtx;
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp890LineObjectType *pLineObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 ecVal = pDevObj->ecVal;
    VpLineStateType lineState;

    VP_API_FUNC(VpDevCtxType, pDevCtx, ("Vp890FreeRun Mode %d", freeRunMode));

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /*
     * Time value is passed in 500us increment. If timeOut = 0, only PCLK
     * recovery exits restart prepare operations. If less than one tick, force
     * a one tick timeout.
     */
    if (freeRunMode == VP_FREE_RUN_STOP) {
        Vp890RestartComplete(pDevCtx);
        /*
         * Clear the device as being forced into free run mode by application.
         * This allows PCLK fault detection to automatically enter/exit free
         * run mode.
         */
        pDevObj->stateInt &= ~VP890_FORCE_FREE_RUN;
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_SUCCESS;
    }

    /* Take the lines out of Ringing if necessary */
    pLineCtx = pDevCtx->pLineCtx[0];
    if (pLineCtx != VP_NULL) {
        pLineObj = pLineCtx->pLineObj;
        ecVal = pLineObj->ecVal;
        lineState = pLineObj->lineState.usrCurrent;

        if (lineState == VP_LINE_RINGING) {
            Vp890SetLineState(pLineCtx, VP_LINE_STANDBY);
        }
        if (lineState == VP_LINE_RINGING_POLREV) {
            Vp890SetLineState(pLineCtx, VP_LINE_STANDBY_POLREV);
        }
    }

    /* Load the free run timing, if available. Othewise just switch to HP mode */
    if (pDevObj->devProfileData.timingParamsFR[0] != 0x00) {
        VpMpiCmdWrapper(deviceId, ecVal, VP890_REGULATOR_TIMING_WRT,
            VP890_REGULATOR_TIMING_LEN, pDevObj->devProfileData.timingParamsFR);
    } else {
        VpMemCpy(pDevObj->swParamsCache, pDevObj->devProfileData.swParams,
            VP890_REGULATOR_PARAM_LEN);

        pDevObj->swParamsCache[VP890_SWY_AUTOPOWER_INDEX] |= VP890_SWY_AUTOPOWER_DIS;
        VpMpiCmdWrapper(deviceId, ecVal, VP890_REGULATOR_PARAM_WRT,
            VP890_REGULATOR_PARAM_LEN, pDevObj->swParamsCache);

        /* Change the Switchers to High Power Mode */
        pDevObj->switchCtrl[0] &= ~VP890_SWY_MODE_MASK;
        pDevObj->switchCtrl[0] |= VP890_SWY_HP;
        VpMpiCmdWrapper(deviceId, ecVal, VP890_REGULATOR_CTRL_WRT,
            VP890_REGULATOR_CTRL_LEN, pDevObj->switchCtrl);
    }

    /*
     * Mark the device as being forced into free run mode by application. This
     * prevents auto-recovery when PCLK is restored.
     */
    pDevObj->stateInt |= VP890_FORCE_FREE_RUN;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}   /* Vp890FreeRun() */

/**
 * Vp890RestartComplete()
 *  This function is called by the VP-API-II internally when a clock fault is
 * removed.
 *
 * Preconditions:
 *  Conditions defined by purpose of Api Tick.
 *
 * Postconditions:
 *  Device and line are in states recovered from a reboot.
 */
void
Vp890RestartComplete(
    VpDevCtxType *pDevCtx)
{
    VpLineCtxType *pLineCtx;
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp890LineObjectType *pLineObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal = pDevObj->ecVal;

    /* Make sure to use the correct EC value. */
    pLineCtx = pDevCtx->pLineCtx[0];
    if (pLineCtx != VP_NULL) {
        pLineObj = pLineCtx->pLineObj;
        ecVal = pLineObj->ecVal;
    }

    /* Restore normal timing, if available. Othewise just switch to LP mode */
    if (pDevObj->devProfileData.timingParamsFR[0] != 0x00) {
        /* Restore the original timings */
        VpMpiCmdWrapper(deviceId, ecVal, VP890_REGULATOR_TIMING_WRT,
            VP890_REGULATOR_TIMING_LEN, pDevObj->devProfileData.timingParams);
    } else {
        /* Change the Switchers to Low Power Mode */
        pDevObj->switchCtrl[0] &= ~VP890_SWY_MODE_MASK;
        pDevObj->switchCtrl[0] |= VP890_SWY_LP;
        VpMpiCmdWrapper(deviceId, ecVal, VP890_REGULATOR_CTRL_WRT,
            VP890_REGULATOR_CTRL_LEN, pDevObj->switchCtrl);

        /* Relinquish switcher control. */
        pDevObj->swParamsCache[VP890_SWY_AUTOPOWER_INDEX] &= ~VP890_SWY_AUTOPOWER_DIS;
        VpMpiCmdWrapper(deviceId, ecVal, VP890_REGULATOR_PARAM_WRT,
            VP890_REGULATOR_PARAM_LEN, pDevObj->swParamsCache);
    }
}   /* Vp890RestartComplete()   */
#endif  /* VP890_FXS_SUPPORT */

/*******************************************************************************
 * Vp890ServiceInterrupts()
 * This function goes through every line associated with the given device and
 * calls Vp890ServiceFxsInterrupts.
 *
 * Arguments: *pDevCtx - Device context ptr
 *
 * Preconditions: intReg in the device object should contain global signaling
 *                  data as read by Vp890ApiTick
 *
 * Postconditions:
 ******************************************************************************/
static void
Vp890ServiceInterrupts(
    VpDevCtxType    *pDevCtx)
{
    VpLineCtxType           *pLineCtx;
    Vp890LineObjectType     *pLineObj;
    uint8                   channelId;

    for (channelId = 0; channelId < VP890_MAX_NUM_CHANNELS; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            if (!(pLineObj->status & VP890_IS_FXO)) {
#ifdef VP890_FXS_SUPPORT
                Vp890ServiceFxsInterrupts(pLineCtx);
#endif
            }
        }
    }
} /* Vp890ServiceInterrupts() */

/*******************************************************************************
 * Vp890ServiceDevTimers()
 * This function services device-level timers.
 *
 * Arguments:  *pDevCtx - Device context ptr
 *
 * Preconditions:  Sould be called once per tick by Vp890ApiTick.
 *
 * Postconditions: Device timers will be serviced.
 ******************************************************************************/
static void
Vp890ServiceDevTimers(
    VpDevCtxType            *pDevCtx)
{
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    uint8          devTimer;

    for (devTimer = 0; devTimer < VP_DEV_TIMER_LAST; devTimer++) {

        /* if timer is not active go to next timer */
        if (!(pDevObj->devTimer[devTimer] & VP_ACTIVATE_TIMER)) {
            continue;
        }

        /* get the bits associated only with the time of the timer */
        pDevObj->devTimer[devTimer] &= VP_TIMER_TIME_MASK;

        /* decrement the timer */
        if (pDevObj->devTimer[devTimer] > 0) {
            (pDevObj->devTimer[devTimer])--;
        }

        /* if time left on the timer, active it and move on to the next one */
        if (pDevObj->devTimer[devTimer] != 0) {
            pDevObj->devTimer[devTimer] |= VP_ACTIVATE_TIMER;
            continue;
        }

        if (VP_DEV_TIMER_TESTLINE == devTimer) {
#ifdef VP890_INCLUDE_TESTLINE_CODE
            const void *pTestArgs =
                (const void*)&pDevObj->currentTest.pTestHeap->testArgs;
            uint8 testChanId = pDevObj->currentTest.channelId;
            VpTestLineIntFuncPtrType testline =
                pDevCtx->funPtrsToApiFuncs.TestLineInt;
            if ((testline != VP_NULL)
              && (pDevObj->currentTest.testState != -1)) {
                /*
                 * if the TestLineInt function exists and the
                 * current test state has not been set back to
                 * -1 by test conclude before the timer expired
                 * then run the call back
                 */

                testline(
                    pDevCtx->pLineCtx[testChanId],
                    pDevObj->currentTest.testId,
                    pTestArgs,
                    pDevObj->currentTest.handle,
                    TRUE);
            }
#endif  /* VP890_INCLUDE_TESTLINE_CODE */
        } else if (VP_DEV_TIMER_LP_CHANGE == devTimer) {
#ifdef VP890_LP_SUPPORT
            Vp890LineObjectType *pLineObj;
            VpLineCtxType *pLineCtx;
            bool failureMode = FALSE;

            /* Skip this process if we're in calibration */
            if (!(pDevObj->state & VP_DEV_IN_CAL)) {
                VP_HOOK(VpDevCtxType, pDevCtx, ("Signaling 0x%02X 0x%02X",
                    pDevObj->intReg[0], pDevObj->intReg[1]));

                /*
                 * For VE890 equivalent code, this is a loop checking both channels. For VE890,
                 * only need to check FSK conditions on the first channel since 2nd channel if
                 * exists is always FXO
                 */
                pLineCtx = pDevCtx->pLineCtx[0];
                if (pLineCtx != VP_NULL) {
                    bool lpTermType = FALSE;
                    pLineObj = pLineCtx->pLineObj;

                    if (VpIsLowPowerTermType(pLineObj->termType)) {
                        lpTermType = TRUE;
                    }

                    if (lpTermType == TRUE) {
                        VP_HOOK(VpLineCtxType, pLineCtx, ("Last Hook State on line 0 = %d LP Mode %d CurrentState %d Time %d",
                            (pLineObj->lineState.condition & VP_CSLAC_HOOK),
                            (pLineObj->status & VP890_LOW_POWER_EN), pLineObj->lineState.currentState, pDevObj->timeStamp));
                        if (pLineObj->lineState.currentState != VP_LINE_DISCONNECT) {
                            if (pLineObj->status & VP890_LOW_POWER_EN) {
                                /*
                                 * If we're in LP Mode, then the line should be
                                 * detecting on-hook. All other conditions mean
                                 * there could be a leaky line.
                                 */
                                if (((pLineObj->lineState.condition & VP_CSLAC_HOOK)
                                  && ((pDevObj->intReg[0]) & VP890_HOOK_MASK) != VP890_HOOK_MASK)) {
                                    failureMode = TRUE;
                                }
                                VP_HOOK(VpLineCtxType, pLineCtx, ("1. Failure Mode = %d -- Previous %d Current %d",
                                    failureMode,
                                    (pLineObj->lineState.condition & VP_CSLAC_HOOK),
                                    (pDevObj->intReg[0]) & VP890_HOOK_MASK));
                            } else {
                                /*
                                 * If we're not in LP Mode, then the line should be
                                 * detecting off-hook and the signaling bit should
                                 * be high. Otherwise, error.
                                 */
                                if ((pLineObj->lineState.condition & VP_CSLAC_HOOK)
                                  && (((pDevObj->intReg[0]) & VP890_HOOK_MASK) != VP890_HOOK_MASK)) {
                                    failureMode = TRUE;
                                    pLineObj->leakyLineCnt++;
                                }
                                VP_HOOK(VpLineCtxType, pLineCtx, ("2. Failure Mode = %d -- Previous %d Current %d",
                                    failureMode,
                                    (pLineObj->lineState.condition & VP_CSLAC_HOOK),
                                    (pDevObj->intReg[0]) & VP890_HOOK_MASK));
                            }
                        }
                    }

                    /*
                     * If the line was last seen off-hook and is now on-hook as a result
                     * of exiting LP Mode, it could be a leaky line.
                     */
                    if (failureMode == TRUE) {
                        if (pLineObj->leakyLineCnt >= VE8XX_LPM_LEAKY_LINE_CNT) {
                            VP_HOOK(VpLineCtxType, pLineCtx,
                                    ("Flag Channel 0 for Leaky Line at time %d Signaling 0x%02X LineState %d",
                                pDevObj->timeStamp, (pDevObj->intReg[0] & VP890_HOOK_MASK),
                                pLineObj->lineState.usrCurrent));
                            if (!(pLineObj->status & VP890_LINE_LEAK)) {
                                pLineObj->lineEvents.faults |= VP_LINE_EVID_RES_LEAK_FLT;
                            }
                            pLineObj->status |= VP890_LINE_LEAK;
                            pDevObj->stateInt &= ~(VP890_LINE0_LP | VP890_LINE1_LP);

                            /* Leaky Line Test is complete */
                            pLineObj->lineState.condition &= ~VP_CSLAC_LINE_LEAK_TEST;
                        } else {
                            uint16 deviceTimer;
                            VP_HOOK(VpLineCtxType, pLineCtx, ("Potential Leaky Line 0 at time %d  ...retry",
                                pDevObj->timeStamp));

                            /*
                             * Make sure timer is restarted. This may
                             * occur as a result of SetLineState(),
                             * but it may not.
                             */
                            if (pDevObj->swParamsCache[VP890_REGULATOR_TRACK_INDEX] & VP890_REGULATOR_FIXED_RING) {
                                /*
                                 * Longest is using "fixed" ringing mode because the external
                                 * capacitors are generally very large.
                                 */
                                deviceTimer = VP890_PWR_SWITCH_DEBOUNCE_FXT;
                            } else {
                                deviceTimer = VP890_PWR_SWITCH_DEBOUNCE_FUT;
                            }
                            VpCSLACSetTimer(&pDevObj->devTimer[VP_DEV_TIMER_LP_CHANGE],
                                MS_TO_TICKRATE(deviceTimer, pDevObj->devProfileData.tickRate));
                            VP_LINE_STATE(VpDevCtxType, pDevCtx,
                                ("Vp890ServiceDevTimers() Channel %d: Starting VP_DEV_TIMER_LP_CHANGE with %d ms (deviceTimer) at time %d",
                                pLineObj->channelId, deviceTimer, pDevObj->timeStamp));
                        }

                        /* Update the line state */
                        /*
                         * NOTE: Similar to VE890 loop, but don't need to check for LPM because
                         * fail = TRUE can only occur on a LPM line, and looping is not necessary
                         * because only channel 0 on the VE890 device can be FXS.
                         */
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("1. Writing Channel 0 Current Linestate %d Current User Linestate %d at time %d",
                            pLineObj->lineState.currentState, pLineObj->lineState.usrCurrent,
                            pDevObj->timeStamp));

                        /*
                         * If currently indicating a failure (which is why we're here), need to
                         * set the line to the opposite mode that what it's currently in so we
                         * can recheck the hook detector status. If we're currently in LPM, force
                         * out of LPM. If currently NOT in LPM, force it back into LPM.
                         */
                        if (pLineObj->status & VP890_LOW_POWER_EN) {
                            /* Force line to feed state */
                            Vp890SetFxsLineState(pLineCtx, VP_LINE_OHT);
                        } else {
                            Vp890SetFxsLineState(pLineCtx, pLineObj->lineState.usrCurrent);
                        }
                    } else if (lpTermType == TRUE) {
                        /*
                         * No failure. Recover all hook status, line states
                         * and clear Leaky Line Test Flag
                         */

                        /* Leaky Line Test is complete */
                        pLineObj->lineState.condition &= ~VP_CSLAC_LINE_LEAK_TEST;

                        /*
                         * Low Power Mode exit simply sets the line to Active in order to maintain
                         * smooth line transients. This step is done to put the line into the user
                         * (API-II) defined state.
                         */
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("LPM Timer: Current %d, User Current %d Channel 0",
                            pLineObj->lineState.currentState, pLineObj->lineState.usrCurrent));

                        if ((pLineObj->lineState.usrCurrent == VP_LINE_STANDBY)
                          && (!(pLineObj->status & VP890_LOW_POWER_EN))
                          && (pLineObj->calLineData.calDone == TRUE)) {     /* Must not occur during the calibration */
                            uint8 lineState[1] = {VP890_SS_IDLE};
#ifdef VP_CSLAC_SEQ_EN
                            if ((pLineObj->cadence.status & (VP_CADENCE_STATUS_ACTIVE | VP_CADENCE_STATUS_SENDSIG)) !=
                                (VP_CADENCE_STATUS_ACTIVE | VP_CADENCE_STATUS_SENDSIG)) {
#endif
                                pLineObj->slicValueCache = lineState[0];
                                VP_LINE_STATE(VpLineCtxType, pLineCtx,
                                    ("VP_DEV_TIMER_LP_CHANGE: Channel %d Writing System State 0x%02X at Time %d",
                                    pLineObj->channelId, pLineObj->slicValueCache, pDevObj->timeStamp));
                                VpMpiCmdWrapper(pDevObj->deviceId, pLineObj->ecVal, VP890_SYS_STATE_WRT,
                                    VP890_SYS_STATE_LEN, &pLineObj->slicValueCache);
#ifdef VP_CSLAC_SEQ_EN
                            }
#endif
                        }

                        if ((pLineObj->lineState.condition & VP_CSLAC_HOOK)
                            && (pDevObj->intReg[0]) & VP890_HOOK_MASK) {

                            if ((pLineObj->lineState.condition & VP_CSLAC_HOOK) &&
                                (pLineObj->status & VP890_LOW_POWER_EN)) {
                                /* The line is on-hook */
                                pLineObj->lineState.condition &= ~VP_CSLAC_HOOK;
                            } else {
                                /* Valid off-hook */
                                VP_HOOK(VpLineCtxType, pLineCtx, ("Valid Off-Hook on line 0 at time %d",
                                    pDevObj->timeStamp));

                                pLineObj->leakyLineCnt = 0;
                                pLineObj->status &= ~VP890_LINE_LEAK;

                                if (pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_ON) {
                                    pLineObj->dpStruct.hookSt = TRUE;
                                    pLineObj->dpStruct2.hookSt = TRUE;

                                    VpInitDP(&pLineObj->dpStruct);
                                    VpInitDP(&pLineObj->dpStruct2);
                                }

                                pLineObj->lineEvents.signaling |= VP_LINE_EVID_HOOK_OFF;
                                pLineObj->lineState.condition |= VP_CSLAC_HOOK;
                                pLineObj->lineEventHandle = pDevObj->timeStamp;
                            }
                        }
                    }
                }
            }
#endif  /* VP890_LP_SUPPORT */
        } else if (VP_DEV_TIMER_WB_MODE_CHANGE == devTimer) {
        }
    } /* Loop through all device timers */

    return;
}

/*******************************************************************************
 * Vp890FindSoftwareInterrupts()
 *  This function checks for active non-masked device and line events.
 *
 * Preconditions:
 *  None.
 *
 * Postconditions:
 *  Returns true if there is an active, non-masked event on either the device
 * or on a line associated with the device.
 ******************************************************************************/
static bool
Vp890FindSoftwareInterrupts(
    VpDevCtxType            *pDevCtx)
{
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    Vp890LineObjectType     *pLineObj;
    VpLineCtxType           *pLineCtx;
    uint8                   channelId;

    VpOptionEventMaskType   eventsMask  = pDevObj->deviceEventsMask;
    VpOptionEventMaskType   *pEvents    = &(pDevObj->deviceEvents);

    /* First clear all device events that are masked */
    pEvents->faults     &= ~(eventsMask.faults);
    pEvents->signaling  &= ~(eventsMask.signaling);
    pEvents->response   &= ~(eventsMask.response);
    pEvents->process    &= ~(eventsMask.process);
    pEvents->test       &= ~(eventsMask.test);
    pEvents->fxo        &= ~(eventsMask.fxo);

    /* Evaluate if any device events remain */
    if (pEvents->faults     ||
        pEvents->signaling  ||
        pEvents->response   ||
        pEvents->process    ||
        pEvents->test       ||
        pEvents->fxo) {

        return TRUE;
    }

    for (channelId = 0; channelId < VP890_MAX_NUM_CHANNELS; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if(pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            eventsMask = pLineObj->lineEventsMask;
            pEvents = &(pLineObj->lineEvents);

            /* Clear the line events that are masked */
            pEvents->faults     &= ~(eventsMask.faults);
            pEvents->signaling  &= ~(eventsMask.signaling);
            pEvents->response   &= ~(eventsMask.response);
            pEvents->process    &= ~(eventsMask.process);
            pEvents->test       &= ~(eventsMask.test);
            pEvents->fxo        &= ~(eventsMask.fxo);

#ifdef VP890_FXO_SUPPORT
            /* Clear all FXO events during PLL Recovery */
            if (pLineObj->lineTimers.timers.fxoTimer.pllRecovery) {
                pEvents->fxo = 0;
            }
#endif
            /* Evaluate if any line events remain */
            if (pEvents->faults     ||
                pEvents->signaling  ||
                pEvents->response   ||
                pEvents->process    ||
                pEvents->test       ||
                pEvents->fxo) {

                return TRUE;
            }
        }
    }

    return FALSE;
} /* Vp890FindSoftwareInterrupts() */

/*******************************************************************************
 * Vp890VirtualISR()
 *  This function is called everytime the device causes an interrupt
 *
 * Preconditions
 *  A device interrupt has just occured
 *
 * Postcondition
 *  This function should be called from the each device's ISR.
 *  This function could be inlined to improve ISR performance.
 ******************************************************************************/
#ifndef VP890_SIMPLE_POLLED_MODE
VpStatusType
Vp890VirtualISR(
    VpDevCtxType *pDevCtx)
{
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

#if defined (VP890_INTERRUPT_LEVTRIG_MODE)
    VpSysDisableInt(deviceId);
#endif
    /* Device Interrupt Received */
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    pDevObj->state |= VP_DEV_PENDING_INT;
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
} /* Vp890VirtualISR() */
#endif

/*******************************************************************************

* Vp890SetLineState()
 *  This function is the API-II wrapper function for Vp890SetFxsLineState and
 * Vp890SetFxoLineState.
 *
 * Preconditions:
 *  Same as Vp890SetFxsLineState() / Vp890SetFxoLineState()
 *
 * Postconditions:
 *  Same as Vp890SetFxsLineState() / Vp890SetFxoLineState()
 ******************************************************************************/
VpStatusType
Vp890SetLineState(
    VpLineCtxType           *pLineCtx,
    VpLineStateType         state)
{
    Vp890LineObjectType     *pLineObj   = pLineCtx->pLineObj;
    VpStatusType            status      = VP_STATUS_SUCCESS;
    VpDevCtxType            *pDevCtx    = pLineCtx->pDevCtx;
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId    = pDevObj->deviceId;
    bool                    overrideSameState;
    bool                    isFxs;

    VP_API_FUNC(VpLineCtxType, pLineCtx, ("+Vp890SetLineState() State %d", state));

    /* Proceed if device state is either in progress or complete */
    /* Get out if device state is not ready */
    if (!Vp890IsDevReady(pDevObj->state, TRUE)) {
        VP_API_FUNC(VpLineCtxType, pLineCtx, ("-Vp890SetLineState() - Device Not Ready"));
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    overrideSameState = FALSE;
    isFxs = (((pLineObj->status & VP890_IS_FXO) == VP890_IS_FXO) ? FALSE : TRUE);

    if (Vp890IsChnlUndrTst(pDevObj, pLineObj->channelId)) {
        overrideSameState = TRUE;
    }
#ifdef VP_CSLAC_SEQ_EN
    if (pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) {
        overrideSameState = TRUE;
    }
#ifdef VP890_FXS_SUPPORT
    if ((isFxs) && (pLineObj->callerId.status & VP_CID_IN_PROGRESS)) {
        overrideSameState = TRUE;
    }
#endif
#endif

    if (!overrideSameState && (state == pLineObj->lineState.usrCurrent)) {
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Skipping Vp890SetLineState() for ch %d, same state %d, time %d",
            pLineObj->channelId, state, pDevObj->timeStamp));
        VP_API_FUNC(VpLineCtxType, pLineCtx, ("-Vp890SetLineState() - Same State"));
        return VP_STATUS_SUCCESS;
    }

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Setting Channel %d to State %d",
        pLineObj->channelId, state));

    if (isFxs) {
#ifdef VP890_FXS_SUPPORT
        if (!(VpCSLACIsSupportedFxsState(pDevCtx->deviceType, state))) {
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            VP_API_FUNC(VpLineCtxType, pLineCtx, ("-Vp890SetLineState() - Unsupported FXS State"));
            return VP_STATUS_INVALID_ARG;
        }
#endif
    } else {
#ifdef VP890_FXO_SUPPORT
        if (Vp890IsSupportedFxoState(state) == FALSE) {
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            VP_API_FUNC(VpLineCtxType, pLineCtx, ("-Vp890SetLineState() - Unsupported FXO State"));
            return VP_STATUS_INVALID_ARG;
        }
#endif
    }

    /* Clear the "called from API" flag. This affects the cadencer */
    pLineObj->status &= ~(VP890_SLS_CALL_FROM_API);

    if (isFxs) {
#ifdef VP890_FXS_SUPPORT
        /*
         * Special FXS handling to prevent setting the line to ringing if
         * off-hook
         */
        if ((pLineObj->lineState.condition & (VP_CSLAC_HOOK | VP_CSLAC_GKEY))
         && ((state == VP_LINE_RINGING_POLREV) || (state == VP_LINE_RINGING))) {
            state = pLineObj->ringCtrl.ringTripExitSt;
        }
        VP_LINE_STATE(VpLineCtxType, pLineCtx,
            ("From Vp890SetLineState() Channel %d setting to API state %d at time %d",
            pLineObj->channelId, state, pDevObj->timeStamp));
        status = Vp890SetFxsLineState(pLineCtx, state);
#endif
    } else {
#ifdef VP890_FXO_SUPPORT
#ifdef VP890_FXO_DELAYED_RING_TRIP
        /*
         * If line is FXO and is currently detecting ringing, set a flag to change
         * state after ringing is over.  Also make sure the voicepath is cut off
         * so that the ringing signal won't be sent out as voice
         */
        if (pLineObj->lineState.condition & VP_CSLAC_RINGING) {
            pLineObj->fxoRingStateFlag = 1;
            pLineObj->fxoRingState = state;
            status = VP_STATUS_SUCCESS;
        } else {
            status = Vp890SetFxoLineState(pLineCtx, state);
        }
#else
            status = Vp890SetFxoLineState(pLineCtx, state);
#endif
#endif
    }

    if (status == VP_STATUS_SUCCESS) {
#ifdef VP890_LP_SUPPORT
        /*
         * Reset the "Count" for leaky line conditions because there are some
         * normal state change conditions that will increment the count, therefore
         * causing exit of LP for non-leaky line
         */
        pLineObj->leakyLineCnt = 0;
#endif
        pLineObj->lineState.usrCurrent = state;
    }

    /*
     * Set the "called from API" flag. Convenience for API functions so setting
     * this flag does not need to occur in multiple locations
     */
    pLineObj->status |= VP890_SLS_CALL_FROM_API;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VP_API_FUNC(VpLineCtxType, pLineCtx, ("-Vp890SetLineState()"));

    return status;
} /* Vp890SetLineState() */

/*******************************************************************************
 * Vp890LLSetSysState()
 *  This function writes to the System State register and based on the state
 * being set, determines if low power mode can or cannot be used, or if the line
 * is recovering from Disconnect to a Feed state. In the latter case, a timer
 * is set to transition through Tip Open first to prevent "ping" of the phone.
 * In this case, the final state is set when the timer expires.
 *
 * Preconditions:
 *  This function is called internally by the API-II only.
 *
 * Postconditions:
 *  The System State Register is updated with either the value passed, or
 * Tip Open. If low power mode termination type exists, a flag in the device
 * object indicating that low power can or cannot be used is modified. If coming
 * out of Disconnect, Tip Open is written and a timer is set.
 ******************************************************************************/
void
Vp890LLSetSysState(
    VpDeviceIdType deviceId,
    VpLineCtxType *pLineCtx,
    uint8 lineState,
    bool writeToDevice)
{
    Vp890LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 ecVal = pLineObj->ecVal;

#ifdef VP890_FXS_SUPPORT
    uint8 lineStatePre[VP890_SYS_STATE_LEN];
    bool lineIsFxs = FALSE;
#endif

#if ((VP_CC_DEBUG_SELECT & VP_DBG_LINE_STATE) || defined(VP890_FXS_SUPPORT))
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;
#endif

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890LLSetSysState() SlicState 0x%02X Write %d",
        lineState, writeToDevice));

#ifdef VP890_FXS_SUPPORT
    lineStatePre[0] = pLineObj->slicValueCache;

    if (!(pLineObj->status & VP890_IS_FXO)) {
        lineIsFxs = TRUE;
        if ((Vp890IsChnlUndrTst(pDevObj, pLineObj->channelId) == TRUE) ||
            (pLineObj->status & VP890_LINE_IN_CAL)) {
            pDevObj->stateInt &= ~(VP890_LINE0_LP | VP890_LINE1_LP);
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("3. Clearing LP flag for channel %d CurrentState %d UserState %d",
                pLineObj->channelId, pLineObj->lineState.currentState,
                pLineObj->lineState.usrCurrent));
        } else {
            bool lpTermType = FALSE;
            bool hookStatus = FALSE;

            if (VpIsLowPowerTermType(pLineObj->termType)) {
                lpTermType = TRUE;
                VpCSLACGetLineStatus(pLineCtx, VP_INPUT_RAW_HOOK, &hookStatus);
            }
            hookStatus = (pLineObj->lineState.currentState == VP_LINE_STANDBY) ? hookStatus : FALSE;

            if ((lpTermType == TRUE) && (hookStatus == FALSE) && (!(pLineObj->status & VP890_LINE_LEAK))) {
                if ((pLineObj->lineState.currentState == VP_LINE_DISCONNECT) ||
                    (pLineObj->lineState.currentState == VP_LINE_STANDBY)) {

                    if (((lineStatePre[0] & VP890_SS_LINE_FEED_MASK) != VP890_SS_FEED_BALANCED_RINGING)
                     && ((lineStatePre[0] & VP890_SS_LINE_FEED_MASK) != VP890_SS_FEED_UNBALANCED_RINGING)) {
                        pDevObj->stateInt |= (VP890_LINE0_LP | VP890_LINE1_LP);
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,("1. 890-Setting LP flag for channel %d",
                            pLineObj->channelId));
                    } else {
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,("1. Delay Setting LP flag for channel %d",
                            pLineObj->channelId));
                    }
                } else {
                    pDevObj->stateInt &= ~(VP890_LINE0_LP | VP890_LINE1_LP);
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,("1. Clearing LP flag for channel %d Device Status 0x%04X",
                        pLineObj->channelId, pDevObj->stateInt));
                }
            } else {
                pDevObj->stateInt &= ~(VP890_LINE0_LP | VP890_LINE1_LP);
                VP_LINE_STATE(VpLineCtxType, pLineCtx, ("2. Clearing LP flag for channel %d Status 0x%04X",
                    pLineObj->channelId, pLineObj->status));
            }
        }
    }
#endif

    /* Device Write Override: setting flags without a device write */
    if (writeToDevice == FALSE) {
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp890LLSetSysState()"));
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("-Vp890LLSetSysState()"));
        return;
    }

    pLineObj->slicValueCache = lineState;

#ifdef VP890_FXS_SUPPORT
    if (((lineStatePre[0] & VP890_SS_LINE_FEED_MASK) == VP890_SS_DISCONNECT)
      && (lineIsFxs == TRUE)) {
        pLineObj->nextSlicValue = lineState;
    }
#endif

    VP_LINE_STATE(VpLineCtxType, pLineCtx,
        ("Setting Chan %d to System State 0x%02X at Time %d",
        pLineObj->channelId, pLineObj->slicValueCache, pDevObj->timeStamp));

    VpMpiCmdWrapper(deviceId, ecVal, VP890_SYS_STATE_WRT, VP890_SYS_STATE_LEN,
        &pLineObj->slicValueCache);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp890LLSetSysState()"));
}

/*******************************************************************************
 * LineStateMap
 *  Locally used function by SetFxsLineState and SetFxoLineState to map an API
 * line state to a register-level line state
 *
 * Preconditions:
 *  None. State to byte mapping only.
 *
 * Postconditions:
 *  Returns the byte that should be used in the device System State register
 * for the API State passed.
 ******************************************************************************/
uint8
LineStateMap(
    VpLineStateType state)
{
    VP_API_FUNC_INT(None, VP_NULL, ("+LineStateMap()"));

    switch(state) {
        case VP_LINE_STANDBY:
            return VP890_SS_IDLE;
        case VP_LINE_TIP_OPEN:
            return VP890_SS_TIP_OPEN;

        case VP_LINE_ACTIVE:
        case VP_LINE_TALK:
        case VP_LINE_OHT:
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
        case VP_LINE_HOWLER:
#endif
            return VP890_SS_ACTIVE;

        case VP_LINE_ACTIVE_POLREV:
        case VP_LINE_TALK_POLREV:
        case VP_LINE_OHT_POLREV:
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
        case VP_LINE_HOWLER_POLREV:
#endif
            return VP890_SS_ACTIVE_POLREV;

        case VP_LINE_STANDBY_POLREV:
            return VP890_SS_IDLE_POLREV;

        case VP_LINE_DISCONNECT:
            return VP890_SS_DISCONNECT;
        case VP_LINE_RINGING:
            return VP890_SS_BALANCED_RINGING;

        case VP_LINE_RINGING_POLREV:
            return VP890_SS_BALANCED_RINGING_PR;

        case VP_LINE_FXO_OHT:
        case VP_LINE_FXO_LOOP_OPEN:
            return VP890_SS_FXO_ONHOOK;
        case VP_LINE_FXO_LOOP_CLOSE:
        case VP_LINE_FXO_TALK:
            return VP890_SS_FXO_OFFHOOK;

        default:
            return VP890_SS_DISCONNECT;
    }
} /* LineStateMap() */

/*******************************************************************************
 * Vp890SetLineTone()
 *  This function sets the line tone with the cadence specified on the line.
 *
 * Preconditions:
 *  The line must first be initialized.
 *
 * Postconditions:
 *  The tone specified by the tone profile is sent on the line at the cadence
 * specified by the cadence profile.  If the tone is NULL, all line tones are
 * removed.  If the cadence is NULL, the cadence is set to "Always On".  This
 * function returns the success code if the tone cadence is a valid tone cadence
 * and the tone profile is a valid tone profile, or in the case where the user
 * passes in profile indexes, if the tone/cadence indexes are within the range
 * of the device.
 ******************************************************************************/
VpStatusType
Vp890SetLineTone(
    VpLineCtxType       *pLineCtx,
    VpProfilePtrType    pToneProfile,  /**< A pointer to a tone profile, or an
                                        * index into the profile table for the tone
                                        * to put on the line.
                                        */
    VpProfilePtrType    pCadProfile,   /**< A pointer to a tone cadence profile, or
                                        * an index into the profile table for the
                                        * tone cadence to put on the line.
                                        */
    VpDtmfToneGenType   *pDtmfControl) /**< Indicates to send a DTMF tone
                                        * (either upstream or downstream) if
                                        * this parameter is not VP_NULL AND
                                        * the tone specified is VP_PTABLE_NULL
                                        */
{
    Vp890LineObjectType     *pLineObj       = pLineCtx->pLineObj;
    uint8                   ecVal           = pLineObj->ecVal;
    uint8                   opCondTarget    = pLineObj->opCond[0];
    bool                    isFxs           = ((pLineObj->status & VP890_IS_FXO) ? FALSE : TRUE);

    VpDevCtxType            *pDevCtx    = pLineCtx->pDevCtx;
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId    = pDevObj->deviceId;

#ifdef VP_CSLAC_SEQ_EN
    VpProfilePtrType      pCadProf  = VP_PTABLE_NULL;
#endif
    VpProfilePtrType      pToneProf = VP_PTABLE_NULL;
    VpDigitType           digit     = VP_DIG_NONE;
    VpDirectionType       direction = VP_DIRECTION_INVALID;

    uint8 mpiByte, mpiIndex;
#ifdef VP890_FXS_SUPPORT
    uint8 converterCfg[VP890_CONV_CFG_LEN];
#endif
    uint8 mpiBuffer[2 + VP890_SIGAB_PARAMS_LEN + VP890_SIGCD_PARAMS_LEN];

    /* Initialize SigGen A/B values to 0 */
    uint8 sigGenAB[VP890_SIGAB_PARAMS_LEN] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    uint8 sigGenCtrl[] = {0};

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890SetLineTone()"));

    /* Get out if device state is not ready */
    if (!Vp890IsDevReady(pDevObj->state, TRUE)) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Check the legality of the Tone profile */
    if (!VpCSLACIsProfileValid(VP_PROFILE_TONE,
        VP_CSLAC_TONE_PROF_TABLE_SIZE,
        pDevObj->profEntry.toneProfEntry,
        pDevObj->devProfileTable.pToneProfileTable,
        pToneProfile, &pToneProf))
    {
        return VP_STATUS_ERR_PROFILE;
    }

    /* Verify a good profile (index or pointer) for the cadence */
#ifdef VP_CSLAC_SEQ_EN

    /* Check the legality of the Tone Cadence profile */
    if (!VpCSLACIsProfileValid(VP_PROFILE_TONECAD,
        VP_CSLAC_TONE_CADENCE_PROF_TABLE_SIZE,
        pDevObj->profEntry.toneCadProfEntry,
        pDevObj->devProfileTable.pToneCadProfileTable,
        pCadProfile, &pCadProf))
    {
        return VP_STATUS_ERR_PROFILE;
    }
#endif

    if (pDtmfControl != VP_NULL) {
        digit = pDtmfControl->toneId;
        if (VpIsDigit(digit) == FALSE) {
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp890SetLineTone-"));
            return VP_STATUS_INVALID_ARG;
        }

        direction = pDtmfControl->dir;
        if (direction != VP_DIRECTION_DS) {
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp890SetLineTone-"));
            return VP_STATUS_INVALID_ARG;
        }
    }

    /* All input parameters are valid. */
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    if ((isFxs) && (pLineObj->status & VP890_BAD_LOOP_SUP)) {
        pLineObj->status &= ~(VP890_BAD_LOOP_SUP);
        VpMpiCmdWrapper(deviceId, ecVal, VP890_LOOP_SUP_WRT,
            VP890_LOOP_SUP_LEN, pLineObj->loopSup);
    }

    /*
     * Disable signal generator A/B/C/D before making any changes and stop
     * previous cadences
     */
    if (pLineObj->sigGenCtrl[0] != sigGenCtrl[0]) {
        pLineObj->sigGenCtrl[0] = sigGenCtrl[0];
        VpMpiCmdWrapper(deviceId, ecVal, VP890_GEN_CTRL_WRT, VP890_GEN_CTRL_LEN,
            pLineObj->sigGenCtrl);
    }

#if defined (VP_CSLAC_SEQ_EN) && defined (VP890_FXS_SUPPORT)
    if ((isFxs) && (!(pLineObj->callerId.status & VP_CID_IN_PROGRESS))) {
        pLineObj->cadence.pActiveCadence = pCadProf;
        pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;

        /* We're no longer in the middle of a time function */
        pLineObj->cadence.status &= ~VP_CADENCE_STATUS_MID_TIMER;
        pLineObj->cadence.timeRemain = 0;
    }
#endif

#if defined (VP_CSLAC_SEQ_EN) && defined (VP890_FXO_SUPPORT)
    if (!(isFxs)) {
        pLineObj->cadence.pActiveCadence = pCadProf;
        pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;

        /* We're no longer in the middle of a time function */
        pLineObj->cadence.status &= ~VP_CADENCE_STATUS_MID_TIMER;
        pLineObj->cadence.timeRemain = 0;
    }
#endif

    /*
     * If tone profile is NULL, and either the pDtmfControl is NULL or it's
     * "digit" member is "Digit None", then shutoff the tone generators, stop
     * any active cadencing and restore the filter coefficients if they need
     * to be. Also, re-enable the audio path if it was disabled by a previous
     * DTMF generation command
     */
    if (((pToneProf == VP_PTABLE_NULL) && (pDtmfControl == VP_NULL))
     || ((pToneProf == VP_PTABLE_NULL) && (digit == VP_DIG_NONE))) {
#if defined (VP890_FXS_SUPPORT)
        if (isFxs) {
            /* Restore the normal audio path */
            VpMpiCmdWrapper(deviceId, ecVal, VP890_CONV_CFG_RD,
                VP890_CONV_CFG_LEN, converterCfg);

            converterCfg[0] &= ~VP890_CONV_CONNECT_BITS;
            converterCfg[0] |= VP890_METALLIC_AC_V;

            VpMpiCmdWrapper(deviceId, ecVal, VP890_CONV_CFG_WRT,
                VP890_CONV_CFG_LEN, converterCfg);
        }
#endif  /* (VP890_FXS_SUPPORT)  */

#if defined (VP_CSLAC_SEQ_EN) && defined (VP890_FXS_SUPPORT)
        if (!(pLineObj->callerId.status & VP_CID_IN_PROGRESS)) {
#endif
            /*
             * Pre-Or the bits and get the correct values based on the current
             * line state, then update the device
             */
            opCondTarget &= (uint8)(~(VP890_CUT_TXPATH | VP890_CUT_RXPATH));
            if (isFxs) {
#if defined (VP890_FXS_SUPPORT)
                Vp890GetFxsTxRxPcmMode(pLineObj, pLineObj->lineState.currentState, &mpiByte);
#endif
            } else {
#if defined (VP890_FXO_SUPPORT)
                Vp890GetFxoTxRxPcmMode(pLineObj, pLineObj->lineState.currentState, &mpiByte);
#endif
            }

            opCondTarget |= mpiByte;
            if (opCondTarget != pLineObj->opCond[0]) {
                pLineObj->opCond[0] = opCondTarget;
                VP_LINE_STATE(VpLineCtxType, pLineCtx, ("9. Writing 0x%02X to Operating Conditions",
                    pLineObj->opCond[0]));

                VpMpiCmdWrapper(deviceId, ecVal, VP890_OP_COND_WRT,
                    VP890_OP_COND_LEN, pLineObj->opCond);
            }
#if defined (VP_CSLAC_SEQ_EN) && defined (VP890_FXS_SUPPORT)
        }
#endif

        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_SUCCESS;
    }

    /*
     * If we're here, we're sending some tone.  If it's DTMF, we can stop the
     * active cadencer, set the time to "always on" (since the application will
     * tell us when to start/stop).
     *
     * If "direction" is some value other than the initialized value, then
     * the dtmf structure is passed and not NULL
     */
    if (direction != VP_DIRECTION_INVALID) {
#ifdef VP_CSLAC_SEQ_EN
        /* Disable currently active cadence */
        pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;
#endif
        /* Update the DTMF Generators and make the downstream connection */
        Vp890SetDTMFGenerators(pLineCtx, VP_CID_NO_CHANGE, digit);

#if defined (VP890_FXS_SUPPORT)
        if (isFxs) {
            VpMpiCmdWrapper(deviceId, ecVal, VP890_CONV_CFG_RD,
                VP890_CONV_CFG_LEN, converterCfg);
            converterCfg[0] &= ~VP890_CONV_CONNECT_BITS;
            converterCfg[0] |= VP890_METALLIC_AC_V;

            VpMpiCmdWrapper(deviceId, ecVal, VP890_CONV_CFG_WRT,
                VP890_CONV_CFG_LEN, converterCfg);
        }
#endif

        /*
         * Disable only the receive path since disabling the transmit path
         * also may generate noise upstream (e.g., an unterminated, but
         * assigned timeslot
         */
        opCondTarget &= (uint8)(~(VP890_HIGH_PASS_DIS | VP890_OPCOND_RSVD_MASK));
        opCondTarget |= VP890_CUT_RXPATH;

        if (opCondTarget != pLineObj->opCond[0]) {
            pLineObj->opCond[0] = opCondTarget;

            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("15. Writing 0x%02X to Operating Conditions",
                pLineObj->opCond[0]));
            VpMpiCmdWrapper(deviceId, ecVal, VP890_OP_COND_WRT, VP890_OP_COND_LEN,
                pLineObj->opCond);
        }

        /* Enable only generator C/D */
        if (pLineObj->sigGenCtrl[0] != (VP890_GEND_EN | VP890_GENC_EN)) {
            pLineObj->sigGenCtrl[0] = (VP890_GEND_EN | VP890_GENC_EN);
            VpMpiCmdWrapper(deviceId, ecVal, VP890_GEN_CTRL_WRT, VP890_GEN_CTRL_LEN,
                pLineObj->sigGenCtrl);
        }

        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_SUCCESS;
    }

#if defined (VP_CSLAC_SEQ_EN) && defined (VP890_FXS_SUPPORT)
    /* If we're here, we're sending a Tone, not DTMF */
    if ((pCadProf != VP_PTABLE_NULL)
     && (((pCadProf[VP_CSLAC_TONE_TYPE] & VP_CSLAC_SPECIAL_TONE_MASK) == VP_CSLAC_UK_HOWLER_TONE_VER15)
      || ((pCadProf[VP_CSLAC_TONE_TYPE] & VP_CSLAC_SPECIAL_TONE_MASK) == VP_CSLAC_UK_HOWLER_TONE_DRAFT_G)
      || ((pCadProf[VP_CSLAC_TONE_TYPE] & VP_CSLAC_SPECIAL_TONE_MASK) == VP_CSLAC_AUS_HOWLER_TONE)
      || ((pCadProf[VP_CSLAC_TONE_TYPE] & VP_CSLAC_SPECIAL_TONE_MASK) == VP_CSLAC_NTT_HOWLER_TONE))) {

        uint8 sigGenCD[VP890_SIGCD_PARAMS_LEN] = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        };

        /* Return ERROR if the Special Howler Init function doesn't know what this Tone Type is */
        pLineObj->cadence.toneType = (pCadProf[VP_CSLAC_TONE_TYPE] & VP_CSLAC_SPECIAL_TONE_MASK);
        if (!(VpCSLACHowlerInit(&pLineObj->cadence, pDevObj->devProfileData.tickRate))) {
            pLineObj->cadence.toneType = VP_CSLAC_STD_TONE;
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp890SetLineTone-"));
            return VP_STATUS_INVALID_ARG;
        }

        sigGenAB[3] = ((pLineObj->cadence.startFreq >> 8) & 0xFF);
        sigGenAB[4] = (pLineObj->cadence.startFreq & 0xFF);

        sigGenAB[5] = ((pLineObj->cadence.startLevel >> 8) & 0xFF);
        sigGenAB[6] = (pLineObj->cadence.startLevel & 0xFF);

        mpiIndex = 0;

        /* Make sure C/D are cleared */
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_SIGCD_PARAMS_WRT,
            VP890_SIGCD_PARAMS_LEN, sigGenCD);

        /* Program A/B */
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_SIGAB_PARAMS_WRT,
            VP890_SIGAB_PARAMS_LEN, sigGenAB);
        /* Clear flag to indicate the generators are NOT in a Ringing Mode */
        pLineObj->status &= ~(VP890_RING_GEN_NORM | VP890_RING_GEN_REV);

        VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);

        /*
         * Set the parameters in the line object for cadence use. The Cadence operations use
         * the cached values when determining frequency/level adjustments.
         */
        VpMemCpy(pLineObj->cadence.regData, sigGenAB, VP890_SIGAB_PARAMS_LEN);
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
        /* Update the Filter Coefficients if in High Gain Mode */
        if (pLineObj->howlerModeCache.isInHowlerMode) {
            VpCLSACHighGainMode(pLineCtx, TRUE);
        }
#endif
    } else {
#endif
        /*
         * Send the signal generator parameters to the device and enable the
         * Tone Generators -- add in the first 3 bytes (all 0x00)
         */
        VpMemCpy(&sigGenAB[VP890_SIGAB_FREQ_START], &pToneProf[VP890_SIGGEN_AB_START],
            (VP890_SIGAB_PARAMS_LEN - VP890_SIGAB_FREQ_START));

        mpiIndex = 0;
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_SIGAB_PARAMS_WRT,
            VP890_SIGAB_PARAMS_LEN, sigGenAB);
        /* Clear flag to indicate the generators are NOT in a Ringing Mode */
        pLineObj->status &= ~(VP890_RING_GEN_NORM | VP890_RING_GEN_REV);

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_SIGCD_PARAMS_WRT,
            VP890_SIGCD_PARAMS_LEN, (uint8 *)(&pToneProf[VP890_SIGGEN_CD_START]));

        VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1,
            &mpiBuffer[1]);
#if defined (VP_CSLAC_SEQ_EN) && defined (VP890_FXS_SUPPORT)
    }
#endif

#ifdef VP_CSLAC_SEQ_EN
    if (pCadProf == VP_PTABLE_NULL) {
        /*
         * If a tone is being actived due to caller ID, then do not stop the
         * cadencer
         */
#ifdef VP890_FXS_SUPPORT
        if (!(pLineObj->callerId.status & VP_CID_IN_PROGRESS)) {
#endif
            pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;
            pLineObj->cadence.index = VP_PROFILE_TYPE_SEQUENCER_START;
#ifdef VP890_FXS_SUPPORT
        }
#endif
#endif
        if (pLineObj->sigGenCtrl[0] != VP890_GEN_ALLON) {
            pLineObj->sigGenCtrl[0] = VP890_GEN_ALLON;
            VpMpiCmdWrapper(deviceId, ecVal, VP890_GEN_CTRL_WRT, VP890_GEN_CTRL_LEN,
                pLineObj->sigGenCtrl);
        }

#ifdef VP_CSLAC_SEQ_EN
    } else {
        pLineObj->cadence.pCurrentPos =
            &(pCadProf[VP_PROFILE_TYPE_SEQUENCER_START]);
        pLineObj->cadence.status |= VP_CADENCE_STATUS_ACTIVE;
        pLineObj->cadence.length = pCadProf[VP_PROFILE_LENGTH];
        pLineObj->cadence.index = VP_PROFILE_TYPE_SEQUENCER_START;
        pLineObj->cadence.status &= ~VP_CADENCE_STATUS_IGNORE_POLARITY;
        pLineObj->cadence.status |= (pCadProf[VP_PROFILE_MPI_LEN] & 0x01) ?
            VP_CADENCE_STATUS_IGNORE_POLARITY : 0;

        /* Nullify any internal sequence so that the API doesn't think that
         * an internal sequence of some sort is running */
        pLineObj->intSequence[VP_PROFILE_TYPE_LSB] = VP_PRFWZ_PROFILE_NONE;
    }
#endif

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
} /* Vp890SetLineTone() */

/*******************************************************************************
 * Vp890SetRelGain()
 *  This function adjusts the GR and GX values for a given channel of a given
 * device.  It multiplies the profile values by a factor from 1.0 to 4.0 (GX) or
 * from 0.25 to 1.0 (GR).  The adjustment factors are specified in the txLevel
 * and rxLevel parameters, which are 2.14 fixed-point numbers.
 *
 * Arguments:
 *  *pLineCtx  -  Line context to change gains on
 *  txLevel    -  Adjustment to line's relative Tx level
 *  rxLevel    -  Adjustment to line's relative Rx level
 *  handle     -  Handle value returned with the event
 *
 * Preconditions:
 *  The line must first be initialized prior to adjusting the gains.  Any
 * pre-existing results must be cleared by calling VpGetResults() before
 * calling this function.
 *
 * Postconditions:
 *  Returns error if device is not initialized or results are not cleared.
 * Otherwise, generates a VE_LINE_EVID_GAIN_CMP event and saves results in
 * the device object for later retrieval by VpGetResults().
 ******************************************************************************/
VpStatusType
Vp890SetRelGain(
    VpLineCtxType   *pLineCtx,
    uint16          txLevel,
    uint16          rxLevel,
    uint16          handle)
{
    Vp890LineObjectType     *pLineObj   = pLineCtx->pLineObj;
    Vp890DeviceObjectType   *pDevObj    = pLineCtx->pDevCtx->pDevObj;
    VpDeviceIdType          deviceId    = pDevObj->deviceId;
    VpStatusType            status;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890SetRelGain()"));

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Get out if device state is not ready */
    if (!Vp890IsDevReady(pDevObj->state, TRUE)) {
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    if (pDevObj->deviceEvents.response & VP890_READ_RESPONSE_MASK) {
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        VP_GAIN(VpLineCtxType, pLineCtx, ("Vp890SetRelGain() - Waiting to clear previous read"));
        return VP_STATUS_DEVICE_BUSY;
    }

    pLineObj->gxUserLevel = txLevel;
    pLineObj->grUserLevel = rxLevel;

    status = Vp890SetRelGainInt(pLineCtx);

    if (status == VP_STATUS_SUCCESS){
        /* Generate the gain-complete event. */
        pLineObj->lineEvents.response |= VP_LINE_EVID_GAIN_CMP;
        pLineObj->lineEventHandle = handle;

        /* Absolute gain values are now unknown */
        pLineObj->absGxGain = VP_ABS_GAIN_UNKNOWN;
        pLineObj->absGrGain = VP_ABS_GAIN_UNKNOWN;
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return status;
} /* Vp890SetRelGain() */

/*******************************************************************************
 * Vp890SetOption()
 *  This function determines how to process the Option based on pDevCtx,
 *  pLineCtx, and option type.  The actual options are implemented in static
 *  functions SetDeviceOption and SetLineOption.
 *
 * Preconditions:
 *  The line must first be initialized if a line context is passed, or the
 * device must be initialized if a device context is passed.
 *
 * Postconditions:
 *  The option specified is implemented either on the line, or on the device, or
 * on all lines associated with the device (see the API Reference Guide for
 ******************************************************************************/
VpStatusType
Vp890SetOption(
    VpLineCtxType           *pLineCtx,
    VpDevCtxType            *pDevCtx,
    VpOptionIdType          option,
    void                    *value)
{
    Vp890DeviceObjectType   *pDevObj;
    VpDeviceIdType          deviceId;
    VpDevCtxType            *pDevCtxLocal;
    VpStatusType            status          = VP_STATUS_INVALID_ARG;

    VP_API_FUNC_INT(None, VP_NULL, ("+Vp890SetOption()"));

    /* Get Device Object based on input param */
    if (pDevCtx != VP_NULL) {
        pDevObj = pDevCtx->pDevObj;
    } else {
        pDevCtxLocal = pLineCtx->pDevCtx;
        pDevObj = pDevCtxLocal->pDevObj;
    }

    deviceId = pDevObj->deviceId;

    /* Get out if device state is not ready AND not setting the debug mask */
    if (option != VP_OPTION_ID_DEBUG_SELECT) {
        if (!Vp890IsDevReady(pDevObj->state, TRUE)) {
            return VP_STATUS_DEV_NOT_INITIALIZED;
        }
    }
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    if (pDevCtx != VP_NULL) {
        status = SetDeviceOption(pDevCtx, deviceId, option, value);
    } else {
        status = SetLineOption(pLineCtx, deviceId, option, value);
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return status;
} /* Vp890SetOption() */

/*******************************************************************************
 * SetDeviceOption()
 *  This function determines how to process Options based on pDevCtx. Any
 *  options that are line specific, loop through all existing lines
 *  associated with pDevCtx calling SetLineOption() with the LineCtx
 *  associated with that line.
 *
 *  Any options that are device specific, Carry out their task here.
 *
 * Preconditions:
 *  The device associated with this line must be initialized.
 *
 * Postconditions:
 *  This function returns the success code if the option associated with this
 *  function completes without issues.
 ******************************************************************************/
static VpStatusType
SetDeviceOption(
    VpDevCtxType        *pDevCtx,
    VpDeviceIdType      deviceId,
    VpOptionIdType      option,
    void                *value)
{
    Vp890DeviceObjectType   *pDevObj    = pDevCtx->pDevObj;
    VpLineCtxType           *pLineCtx;
    Vp890LineObjectType     *pLineObj;
    VpStatusType            status      = VP_STATUS_SUCCESS;
    VpOptionDeviceIoType    deviceIo;
    uint8                   ioDirection = 0;
    uint8                   ecVal = 0;
    uint8                   chanId;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("+SetDeviceOption()"));

    /*
     * Valid Device Context, we already know Line context is NULL (higher
     * layer SW, process on device if device option, or process on all lines
     * associated with device if line option
     */
    switch (option) {
        /* Line Options */
        case VP_OPTION_ID_EVENT_MASK:  /* Affects Line and Device */

#ifdef VP890_FXS_SUPPORT
        case VP_OPTION_ID_ZERO_CROSS:
        case VP_OPTION_ID_PULSE_MODE:
        case VP_OPTION_ID_LINE_STATE:
        case VP_OPTION_ID_RING_CNTRL:
        case VP_OPTION_ID_SWITCHER_CTRL:
#endif

        case VP_OPTION_ID_TIMESLOT:
        case VP_OPTION_ID_CODEC:
        case VP_OPTION_ID_PCM_HWY:
        case VP_OPTION_ID_LOOPBACK:
        case VP_OPTION_ID_PCM_TXRX_CNTRL:
#ifdef CSLAC_GAIN_ABS
        case VP_OPTION_ID_ABS_GAIN:
#endif
        case VP_OPTION_ID_RINGING_PARAMS:
        case VP_OPTION_ID_DCFEED_PARAMS:

            /*
             * Loop through all of the valid channels associated with this
             * device. Init status variable in case there are currently no
             * line contexts associated with this device
             */
            status = VP_STATUS_SUCCESS;
            for (chanId = 0; chanId < VP890_MAX_NUM_CHANNELS; chanId++) {
                pLineCtx = pDevCtx->pLineCtx[chanId];

                if (pLineCtx == VP_NULL) {
                    continue;
                }
                pLineObj = pLineCtx->pLineObj;

#ifdef VP890_FXS_SUPPORT
                if ((option == VP_OPTION_ID_ZERO_CROSS) ||
                    (option == VP_OPTION_ID_PULSE_MODE) ||
                    (option == VP_OPTION_ID_LINE_STATE) ||
                    (option == VP_OPTION_ID_RING_CNTRL) ||
                    (option == VP_OPTION_ID_SWITCHER_CTRL)){

                    bool onlyFXO = TRUE;
                    uint8 lastChannel = (pDevObj->staticInfo.maxChannels - 1);

                    pLineObj = pLineCtx->pLineObj;

                    /* This device has at least 1 FXS, SetOption will succeed */
                    if (!(pLineObj->status & VP890_IS_FXO)) {
                        onlyFXO = FALSE;
                        status = SetLineOption(pLineCtx, deviceId, option, value);
                    /* Only FXO on this device */
                    } else if ((onlyFXO == TRUE) && (chanId == lastChannel)) {
                        status = VP_STATUS_OPTION_NOT_SUPPORTED;
                    /* Just bailout in case there is at least 1 FXS on this device */
                    } else {
                        break;
                    }
                } else {
                    status = SetLineOption(pLineCtx, deviceId, option, value);
                }
#endif
                if (VP_STATUS_SUCCESS != status) {
                    return status;
                }
            }
            break;

#ifdef VP890_FXS_SUPPORT
        case VP_DEVICE_OPTION_ID_PULSE:
            if (pDevObj->stateInt & VP890_IS_FXO_ONLY) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
            }
            pDevObj->pulseSpecs = *((VpOptionPulseType *)value);
            break;

        case VP_DEVICE_OPTION_ID_PULSE2:
            if (pDevObj->stateInt & VP890_IS_FXO_ONLY) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
            }
            pDevObj->pulseSpecs2 = *((VpOptionPulseType *)value);
            break;

        case VP_DEVICE_OPTION_ID_CRITICAL_FLT: {
            VpOptionCriticalFltType criticalFault =
                *((VpOptionCriticalFltType *)value);

            if (pDevObj->stateInt & VP890_IS_FXO_ONLY) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
            }

            if ((criticalFault.acFltDiscEn == TRUE)
             || (criticalFault.dcFltDiscEn == TRUE)) {
                return VP_STATUS_INVALID_ARG;
            }
            pDevObj->criticalFault = criticalFault;

           /*
            * NOTE: NEVER enable the Auto-Thermal Fault disconnect in the silicon
            * because the silicon is too fast for the VP-API-II. It would be possible
            * to get a thermal fault, have the silicon disable the line, and have
            * the thermal fault go away all before the VP-API-II sees it. In that
            * condition, the line will be disabled without the application being
            * aware of it.
            */
            }
            break;
#endif

        case VP_DEVICE_OPTION_ID_DEVICE_IO:
            VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("+SetDeviceOption() - OPTION_ID_DEVICE_IO"));

            deviceIo = *(VpOptionDeviceIoType *)(value);
            ioDirection = deviceIo.directionPins_31_0 << 1;
            ioDirection &= ~VP890_IODIR_IO1_MASK;

            /* Default 00 is input */
            if ((deviceIo.directionPins_31_0 & 1) == VP_IO_OUTPUT_PIN) {
                /* Driven output only is supported. */
                ioDirection |= VP890_IODIR_IO1_OUTPUT;
            }

            for(chanId=0; chanId < VP890_MAX_NUM_CHANNELS; chanId++ ) {
                pLineCtx = pDevCtx->pLineCtx[chanId];
                if (pLineCtx != VP_NULL) {
                    pLineObj = pLineCtx->pLineObj;
                    ecVal |= pLineObj->ecVal;

#ifdef VP890_FXS_SUPPORT
                    if ((pLineObj->termType == VP_TERM_FXS_SPLITTER_LP)
                     || (pLineObj->termType == VP_TERM_FXS_SPLITTER)
                     || (pLineObj->termType == VP_TERM_FXS_ISOLATE)
                     || (pLineObj->termType == VP_TERM_FXS_ISOLATE_LP)) {
                        /* Splitter/Isolate I/O1 direction must be unchanged */
                        /*
                         * Note that this works only if the line contexts are
                         * associated with the device context passed to this
                         * function -- which is not a requirement.
                         */
                        uint8 localIoDir[VP890_IODIR_REG_LEN];
                        VpMpiCmdWrapper(deviceId, ecVal, VP890_IODIR_REG_RD,
                            VP890_IODIR_REG_LEN, localIoDir);

                        ioDirection &= ~VP890_IODIR_IO1_MASK;
                        ioDirection |= (localIoDir[0] & VP890_IODIR_IO1_MASK);
                    }
#endif
                }
            }
            VpMpiCmdWrapper(deviceId, ecVal, VP890_IODIR_REG_WRT, VP890_IODIR_REG_LEN,
                &ioDirection);
            VP_API_FUNC_INT(VpDevCtxType, pDevCtx, (" -SetDeviceOption() - OPTION_ID_DEVICE_IO"));
            break;

#ifdef VP_DEBUG
        case VP_OPTION_ID_DEBUG_SELECT:
            /* Update the debugSelectMask in the Device Object. */
            pDevObj->debugSelectMask = *(uint32 *)value;
            break;
#endif

        default:
            status = VP_STATUS_OPTION_NOT_SUPPORTED;
            if (!(pDevObj->state & VP_DEV_INIT_IN_PROGRESS)) {
                VP_ERROR(VpDevCtxType, pDevCtx,
                         ("SetDeviceOption() - Device option not supported 0x%02X", option));
            }
            break;
    }

    return status;
} /* SetDeviceOption() */

/*******************************************************************************
 * SetLineOption()
 * This function ...
 *
 * Arguments:
 *
 * Preconditions:
 *
 * Postconditions:
 ******************************************************************************/
static VpStatusType
SetLineOption(
    VpLineCtxType           *pLineCtx,
    VpDeviceIdType          deviceId,
    VpOptionIdType          option,
    void                    *value)
{
    VpDevCtxType            *pDevCtxLocal   = pLineCtx->pDevCtx;
    Vp890LineObjectType     *pLineObj       = pLineCtx->pLineObj;
    Vp890DeviceObjectType   *pDevObj        = pDevCtxLocal->pDevObj;
    VpStatusType            status          = VP_STATUS_SUCCESS;

    uint8                   ecVal           = pLineObj->ecVal;
    uint8                   txSlot, rxSlot;
    uint8                   mpiData, mpiByte;
    uint8                   tempLoopBack[VP890_LOOPBACK_LEN];

#ifdef VP890_FXS_SUPPORT
    uint8                   tempSysConfig[VP890_SS_CONFIG_LEN];
#endif

    VP_API_FUNC_INT(VpDeviceIdType, &deviceId, ("+SetLineOption()"));

    switch (option) {
        /* Device Options - invalid arg */
        case VP_DEVICE_OPTION_ID_DEVICE_IO:

#ifdef VP890_FXS_SUPPORT
        case VP_DEVICE_OPTION_ID_PULSE:
        case VP_DEVICE_OPTION_ID_PULSE2:
        case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
#endif
            return VP_STATUS_INVALID_ARG;

#ifdef CSLAC_GAIN_ABS
        case VP_OPTION_ID_ABS_GAIN:
            status = VpCSLACSetAbsGain(pLineCtx, ((VpOptionAbsGainType *)value));
            break;
#endif

        case VP_OPTION_ID_EVENT_MASK:
             SetOptionEventMask(pDevObj, pLineObj, deviceId, ecVal, value);
             break;

        case VP_OPTION_ID_TIMESLOT:
            txSlot = ((VpOptionTimeslotType *)value)->tx;
            rxSlot = ((VpOptionTimeslotType *)value)->rx;
            status = SetTimeSlot(pLineCtx, txSlot, rxSlot);
            break;

        case VP_OPTION_ID_CODEC:
            status = Vp890SetCodec(pLineCtx, *((VpOptionCodecType *)value));
            break;

        case VP_OPTION_ID_PCM_HWY:
            if (*((VpOptionPcmHwyType *)value) != VP_OPTION_HWY_A) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("SetLineOption() - Invalid PCM Highway option"));
                return VP_STATUS_INVALID_ARG;
            }
            break;

        case VP_OPTION_ID_LOOPBACK:
            /* Timeslot loopback via loopback register */
            VpMpiCmdWrapper(deviceId, ecVal, VP890_LOOPBACK_RD,
                VP890_LOOPBACK_LEN, tempLoopBack);

            switch(*((VpOptionLoopbackType *)value)) {
                case VP_OPTION_LB_TIMESLOT:
                    tempLoopBack[0] |= VP890_INTERFACE_LOOPBACK_EN;
                    pDevObj->devMode[0] &= (uint8)(~(VP890_DEV_MODE_CH21PT | VP890_DEV_MODE_CH12PT));
#ifdef VP890_FXS_SUPPORT
                    pLineObj->icr4Values[0] |= VP890_ICR4_VOICE_DAC_CTRL;
                    pLineObj->icr4Values[1] &= ~VP890_ICR4_VOICE_DAC_CTRL;
#endif
                    break;

                case VP_OPTION_LB_OFF:
                    tempLoopBack[0] &= ~(VP890_INTERFACE_LOOPBACK_EN);
                    pDevObj->devMode[0] &= (uint8)(~(VP890_DEV_MODE_CH21PT | VP890_DEV_MODE_CH12PT));
#ifdef VP890_FXS_SUPPORT
                    pLineObj->icr4Values[0] &= ~VP890_ICR4_VOICE_DAC_CTRL;
#endif
                    break;

                case VP_OPTION_LB_CHANNELS:
                    tempLoopBack[0] &= ~(VP890_INTERFACE_LOOPBACK_EN);
                    pDevObj->devMode[0] |= (VP890_DEV_MODE_CH21PT | VP890_DEV_MODE_CH12PT);
#ifdef VP890_FXS_SUPPORT
                    pLineObj->icr4Values[0] &= ~VP890_ICR4_VOICE_DAC_CTRL;
#endif
                    break;

                case VP_OPTION_LB_DIGITAL:
                default:
                    VP_ERROR(VpLineCtxType, pLineCtx, ("SetLineOption() - Invalid Loopback option"));
                    return VP_STATUS_INVALID_ARG;
            }

            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("1. Loopback 0x%02X", tempLoopBack[0]));
            VpMpiCmdWrapper(deviceId, ecVal, VP890_LOOPBACK_WRT,
                VP890_LOOPBACK_LEN, tempLoopBack);
            VpMpiCmdWrapper(deviceId, ecVal, VP890_DEV_MODE_WRT,
                VP890_DEV_MODE_LEN, pDevObj->devMode);
#ifdef VP890_FXS_SUPPORT
                if (!(pLineObj->status & VP890_IS_FXO)) {
                    VpMpiCmdWrapper(deviceId, ecVal, VP890_ICR4_WRT,
                        VP890_ICR4_LEN, pLineObj->icr4Values);
                }
#endif
            break;

#ifdef VP890_FXS_SUPPORT
        /* Line Options */
        case VP_OPTION_ID_PULSE_MODE:
            /* Option does not apply to FXO */
            if (pLineObj->status & VP890_IS_FXO) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
            }
            pLineObj->pulseMode = *((VpOptionPulseModeType *)value);

            if (pLineObj->lineState.condition & VP_CSLAC_HOOK) {
                pLineObj->dpStruct.hookSt = TRUE;
                pLineObj->dpStruct2.hookSt = TRUE;
            } else {
                pLineObj->dpStruct.hookSt = FALSE;
                pLineObj->dpStruct2.hookSt = FALSE;
            }
            VpInitDP(&pLineObj->dpStruct);
            VpInitDP(&pLineObj->dpStruct2);
            break;

        case VP_OPTION_ID_LINE_STATE:
            /* Option does not apply to FXO */
            if (pLineObj->status & VP890_IS_FXO) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
            }
            /*
             * Only supports one type of battery control, so make sure it
             * is set correctly. If not, return error otherwise continue
             */
            if (((VpOptionLineStateType *)value)->bat != VP_OPTION_BAT_AUTO) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("SetLineOption() - Invalid battery option"));
                return VP_STATUS_INVALID_ARG;
            }

            VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_RD,
                VP890_SS_CONFIG_LEN, tempSysConfig);

            if (((VpOptionLineStateType *)value)->battRev == TRUE) {
                tempSysConfig[0] &= ~(VP890_SMOOTH_PR_EN);
            } else {
                tempSysConfig[0] |= VP890_SMOOTH_PR_EN;
            }
            VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_WRT,
                VP890_SS_CONFIG_LEN, tempSysConfig);
            break;

        case VP_OPTION_ID_ZERO_CROSS:
            /* Option does not apply to FXO */
            if (pLineObj->status & VP890_IS_FXO) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
            }
            VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_RD,
                VP890_SS_CONFIG_LEN, tempSysConfig);

            if (*(VpOptionZeroCrossType *)value == VP_OPTION_ZC_NONE) {
                tempSysConfig[0] |= VP890_ZXR_DIS;
            } else {
                tempSysConfig[0] &= ~(VP890_ZXR_DIS);
            }
            VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_WRT,
                VP890_SS_CONFIG_LEN, tempSysConfig);

            pLineObj->ringCtrl.zeroCross = *((VpOptionZeroCrossType *)value);
            break;

        case VP_OPTION_ID_RING_CNTRL: {
            VpOptionRingControlType ringCtrl = *((VpOptionRingControlType *)value);

            /* Option does not apply to FXO */
            if (pLineObj->status & VP890_IS_FXO) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
            }

            if (!(VpCSLACIsSupportedFxsState(pDevCtxLocal->deviceType, ringCtrl.ringTripExitSt))) {
                return VP_STATUS_INVALID_ARG;
            }

            pLineObj->ringCtrl = ringCtrl;

            VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_RD,
                VP890_SS_CONFIG_LEN, tempSysConfig);

            if (pLineObj->ringCtrl.zeroCross == VP_OPTION_ZC_NONE) {
                tempSysConfig[0] |= VP890_ZXR_DIS;
            } else {
                tempSysConfig[0] &= ~(VP890_ZXR_DIS);
            }

            VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_WRT,
                VP890_SS_CONFIG_LEN, tempSysConfig);
            }
            break;

        case VP_OPTION_ID_SWITCHER_CTRL: {
            bool shutDownEn = *((bool *)value);
            uint8 ssConfig[VP890_SS_CONFIG_LEN];

            /* Option does not apply to FXO */
            if (pLineObj->status & VP890_IS_FXO) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
            }

            VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_RD, VP890_SS_CONFIG_LEN, ssConfig);

            if (shutDownEn == TRUE) {
                ssConfig[0] |= VP890_AUTO_BAT_SHUTDOWN_EN;
            } else {
                ssConfig[0] &= ~VP890_AUTO_BAT_SHUTDOWN_EN;
            }

            VpMpiCmdWrapper(deviceId, ecVal, VP890_SS_CONFIG_WRT, VP890_SS_CONFIG_LEN, ssConfig);
            }
            break;
#endif

        case VP_OPTION_ID_PCM_TXRX_CNTRL:
            pLineObj->pcmTxRxCtrl = *((VpOptionPcmTxRxCntrlType *)value);

            mpiData = pLineObj->opCond[0];

            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("16.a Data Read 0x%02X from Operating Conditions",
                mpiData));

            mpiData &= (uint8)(~(VP890_CUT_TXPATH | VP890_CUT_RXPATH));
            mpiData &= (uint8)(~(VP890_HIGH_PASS_DIS | VP890_OPCOND_RSVD_MASK));
            if ((pLineObj->status & VP890_IS_FXO) == VP890_IS_FXO) {
#ifdef VP890_FXO_SUPPORT
                Vp890GetFxoTxRxPcmMode(pLineObj, pLineObj->lineState.currentState, &mpiByte);
#endif
            } else {
#ifdef VP890_FXS_SUPPORT
                Vp890GetFxsTxRxPcmMode(pLineObj, pLineObj->lineState.currentState, &mpiByte);
#endif
            }

            mpiData |= mpiByte;
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("16.b Writing 0x%02X to Operating Conditions",
                mpiData));
            if (mpiData != pLineObj->opCond[0]) {
                VpMpiCmdWrapper(deviceId, ecVal, VP890_OP_COND_WRT, VP890_OP_COND_LEN,
                    &mpiData);
                pLineObj->opCond[0] = mpiData;
            }
            break;

#ifdef VP_DEBUG
        case VP_OPTION_ID_DEBUG_SELECT:
            /* Update the debugSelectMask in the Line Object. */
            pLineObj->debugSelectMask = *(uint32 *)value;
            break;
#endif

#ifdef VP890_FXS_SUPPORT
        case VP_OPTION_ID_DCFEED_PARAMS:
            /* Option does not apply to FXO */
            if (pLineObj->status & VP890_IS_FXO) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
            }
            status = SetOptionDcFeedParams(pLineCtx, (VpOptionDcFeedParamsType *)value);
            break;

        case VP_OPTION_ID_RINGING_PARAMS:
            /* Option does not apply to FXO */
            if (pLineObj->status & VP890_IS_FXO) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
            }
            status = SetOptionRingingParams(pLineCtx, (VpOptionRingingParamsType *)value);
            break;
#endif /* VP890_FXS_SUPPORT */

        default:
            status = VP_STATUS_OPTION_NOT_SUPPORTED;
            if (pLineObj->status & VP890_INIT_COMPLETE) {
                VP_ERROR(VpLineCtxType, pLineCtx,
                         ("SetLineOption() - Line option not supported 0x%02X", option));
            }
            break;
    }

    return status;
} /* SetLineOption() */

/*******************************************************************************
 * SetOptionEventMask()
 * This function ...
 *
 * Arguments:
 *
 * Preconditions:
 *
 * Postconditions:
 ******************************************************************************/
static void
SetOptionEventMask(
    Vp890DeviceObjectType   *pDevObj,
    Vp890LineObjectType     *pLineObj,
    VpDeviceIdType          deviceId,
    uint8                   ecVal,
    void                    *value)
{
    uint8                   chanId      = pLineObj->channelId;
    VpOptionEventMaskType   *pEventsMask, *pNewEventsMask;
    uint8                   tempData[VP890_INT_MASK_LEN];

    VP_API_FUNC_INT(VpDeviceIdType, &deviceId, ("+SetOptionEventMask()"));

    pNewEventsMask = (VpOptionEventMaskType *)value;

    /* Zero out the line-specific bits before setting the
     * deviceEventsMask in the device object. */
    pEventsMask = &pDevObj->deviceEventsMask;

    pEventsMask->faults     = pNewEventsMask->faults    & VP_EVCAT_FAULT_DEV_EVENTS;
    pEventsMask->signaling  = pNewEventsMask->signaling & VP_EVCAT_SIGNALING_DEV_EVENTS;
    pEventsMask->response   = pNewEventsMask->response  & VP_EVCAT_RESPONSE_DEV_EVENTS;
    pEventsMask->test       = pNewEventsMask->test      & VP_EVCAT_TEST_DEV_EVENTS;
    pEventsMask->process    = pNewEventsMask->process   & VP_EVCAT_PROCESS_DEV_EVENTS;
    pEventsMask->fxo        = pNewEventsMask->fxo       & VP_EVCAT_FXO_DEV_EVENTS;

    /* Zero out the device-specific bits before setting the
     * lineEventsMask in the line object. */
    pEventsMask = &pLineObj->lineEventsMask;

    pEventsMask->faults     = pNewEventsMask->faults    & ~VP_EVCAT_FAULT_DEV_EVENTS;
    pEventsMask->signaling  = pNewEventsMask->signaling & ~VP_EVCAT_SIGNALING_DEV_EVENTS;
    pEventsMask->response   = pNewEventsMask->response  & ~VP_EVCAT_RESPONSE_DEV_EVENTS;
    pEventsMask->test       = pNewEventsMask->test      & ~VP_EVCAT_TEST_DEV_EVENTS;
    pEventsMask->process    = pNewEventsMask->process   & ~VP_EVCAT_PROCESS_DEV_EVENTS;
    pEventsMask->fxo        = pNewEventsMask->fxo       & ~VP_EVCAT_FXO_DEV_EVENTS;

    /* Unmask the unmaskable defined in vp_api_common.c */
    VpImplementNonMaskEvents(&pLineObj->lineEventsMask, &pDevObj->deviceEventsMask);

    /* Mask those events that the VP890 API-II cannot generate */
    MaskNonSupportedEvents(&pLineObj->lineEventsMask, &pDevObj->deviceEventsMask);

    /*
     * The next code section prevents the device from interrupting
     * the processor if all of the events associated with the
     * specific hardware interrupt are masked
     */
    VpMpiCmdWrapper(deviceId, ecVal, VP890_INT_MASK_RD, VP890_INT_MASK_LEN, tempData);

    /* Keep Clock Fault Interrupt Enabled to support auto-free run mode. */
    tempData[0] &= ~VP890_CFAIL_MASK;

    if (!(pLineObj->status & VP890_IS_FXO)) {   /* Line is FXS */
        /* Mask off the FXO events */
        pLineObj->lineEventsMask.fxo |= VP_EVCAT_FXO_MASK_ALL;

        /*
         * Never mask the thermal fault interrupt otherwise the
         * actual thermal fault may not be seen by the VP-API-II.
         */
        tempData[chanId] &= ~VP890_TEMPA_MASK;

        /*
         * Never mask the hook interrupt otherwise interrupt modes
         * of the VP-API-II for LPM types won't work -- hook status
         * is never updated, leaky line never properly detected.
         */
        tempData[chanId] &= ~VP890_HOOK_MASK;

        /*
         * Never mask the gkey interrupt otherwise interrupt modes
         * of the VP-API-II won't support "get line status"
         * correctly.
         */
        tempData[chanId] &= ~VP890_GNK_MASK;

        /* Disable Overcurrent Alaram on device until debounce put in place */
        tempData[chanId] &= ~(VP890_OCALMY_MASK);
    } else {
        /* Line is FXO */
        /* Mask off the FXS events */
        pLineObj->lineEventsMask.signaling |= VP890_FXS_SIGNALING_EVENTS;

        /*
         * Never mask the FXO interrupts otherwise interrupt modes of the VP-API-II
         * won't support FXO "get line status" correctly.
         */
        tempData[chanId] &=
            (uint8)(~(VP890_RING_DET_MASK | VP890_LIU_MASK | VP890_POL_MASK | VP890_LDN_MASK));

        /*
         * NOTE: The overcurrent interrupt is managed by FXO line state control because it is set
         * based on loop open/close condition.
         */

        /* Workaround:  If VISTAT is masked, changing the FXO state can clear
         * existing interrupts.  Force it to be unmasked to avoid this.
         *
         * Specific error case:  Start with FXO offhook and disconnected.
         * Reconnect feed, this will pull the interrupt low.  Go onhook, and
         * the interrupt will go back to high if VISTAT is masked. */
        tempData[chanId] &= ~VP890_VISTAT_MASK;
    }
    VpMpiCmdWrapper(deviceId, ecVal, VP890_INT_MASK_WRT, VP890_INT_MASK_LEN, tempData);

    return;
} /* SetOptionEventMask() */


#ifdef VP890_FXS_SUPPORT
/** SetOptionDcFeedParams()
  Applies the VP_OPTION_ID_DCFEED_PARAMS option for a line.
*/
VpStatusType
SetOptionDcFeedParams(
    VpLineCtxType *pLineCtx,
    VpOptionDcFeedParamsType *pDcFeedParams)
{
    Vp890LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal = pLineObj->ecVal;
    uint8 voc = 0;
    uint8 vocsft = 0;
    uint8 ila = 0;
    uint8 tsh = 0;
    uint8 tgk = 0;
    uint8 battFloor = 0;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+SetOptionDcFeedParams()"));

    /* Perform all error checks and calculations BEFORE changing cached values
       or accessing the device to make this atomic. */

    if (pDcFeedParams->validMask == 0) {
        return VP_STATUS_SUCCESS;
    }

    if (pDcFeedParams->validMask == 0xFFFFFFFF) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("SetOptionDcFeedParams - validMask should not be blindly set to 0xFFFFFFFF"));
        return VP_STATUS_INVALID_ARG;
    }

    if (pDcFeedParams->validMask & VP_OPTION_CFG_VOC) {
        /* VOC is a 3-bit field with step size of 3 V and a range of either
           12-33 or 36-57 V depending on the VOCSFT bit. */
        if (pDcFeedParams->voc < VP890_VOC_MIN || pDcFeedParams->voc > VP890_VOC_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("SetOptionDcFeedParams - voc out of range (%ld)",
                pDcFeedParams->voc));
            return VP_STATUS_INVALID_ARG;
        }
        voc = VpRoundedDivide(pDcFeedParams->voc - VP890_VOC_OFFSET, VP890_VOC_STEP);
        if (voc > 0x7) {
            voc -= 0x8;
            vocsft = 0;
        } else {
            vocsft = 1;
        }
    }

    if (pDcFeedParams->validMask & VP_OPTION_CFG_ILA) {
        /* ILA is a 5-bit value with a 18-49 mA scale (1 mA per step + 18 mA
           offset) */
        if (pDcFeedParams->ila < VP890_ILA_MIN || pDcFeedParams->ila > VP890_ILA_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("SetOptionDcFeedParams - ila out of range (%ld)",
                pDcFeedParams->ila));
            return VP_STATUS_INVALID_ARG;
        }
        ila = VpRoundedDivide(pDcFeedParams->ila - VP890_ILA_OFFSET, VP890_ILA_STEP);
    }

    if (pDcFeedParams->validMask & VP_OPTION_CFG_HOOK_THRESHOLD) {
        /* TSH is a 3-bit value with a 7-14 mA scale (1 mA per step + 7 mA
           offset) */
        if (pDcFeedParams->hookThreshold < VP890_TSH_MIN || pDcFeedParams->hookThreshold > VP890_TSH_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("SetOptionDcFeedParams - hookThreshold out of range (%ld)",
                pDcFeedParams->hookThreshold));
            return VP_STATUS_INVALID_ARG;
        }
        tsh = VpRoundedDivide(pDcFeedParams->hookThreshold - VP890_TSH_OFFSET, VP890_TSH_STEP);
    }

    if (pDcFeedParams->validMask & VP_OPTION_CFG_GKEY_THRESHOLD) {
        /* TGK is a 3-bit value with a 0-42 mA scale (6 mA per step) */
        if (pDcFeedParams->gkeyThreshold < VP890_TGK_MIN || pDcFeedParams->gkeyThreshold > VP890_TGK_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("SetOptionDcFeedParams - gkeyThreshold out of range (%ld)",
                pDcFeedParams->gkeyThreshold));
            return VP_STATUS_INVALID_ARG;
        }
        tgk = VpRoundedDivide(pDcFeedParams->gkeyThreshold - VP890_TGK_OFFSET, VP890_TGK_STEP);
    }

    if (pDcFeedParams->validMask & VP_OPTION_CFG_BATT_FLOOR) {
        /* Allow battery floor voltage from -5 to -100 V in 5V steps */
        if (pDcFeedParams->battFloor < VP890_BATTFLR_MIN || pDcFeedParams->battFloor > VP890_BATTFLR_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("SetOptionDcFeedParams - battFloor out of range (%ld)",
                pDcFeedParams->battFloor));
            return VP_STATUS_INVALID_ARG;
        }
        battFloor = VpRoundedDivide(pDcFeedParams->battFloor - VP890_BATTFLR_OFFSET, VP890_BATTFLR_STEP);
    }

    /* Program the DC feed parameters (VOC, ILA) */
    if ((pDcFeedParams->validMask & VP_OPTION_CFG_VOC) ||
        (pDcFeedParams->validMask & VP_OPTION_CFG_ILA))
    {
        if (pDcFeedParams->validMask & VP_OPTION_CFG_VOC) {
            pLineObj->calLineData.dcFeedRef[VP890_VOC_INDEX] &= ~VP890_VOC_MASK;
            pLineObj->calLineData.dcFeedRef[VP890_VOC_INDEX] |= ((voc << 2) & VP890_VOC_MASK);
            pLineObj->calLineData.dcFeedRef[VP890_VOC_INDEX] &= ~VP890_VOC_SHIFT_MASK;
            pLineObj->calLineData.dcFeedRef[VP890_VOC_INDEX] |= ((vocsft << 6) & VP890_VOC_SHIFT_MASK);
        }
        if (pDcFeedParams->validMask & VP_OPTION_CFG_ILA) {
            uint8 icr5Speedup[VP890_ICR5_LEN];
            pLineObj->calLineData.dcFeedRef[VP890_ILA_INDEX] &= ~VP890_ILA_MASK;
            pLineObj->calLineData.dcFeedRef[VP890_ILA_INDEX] |= (ila & VP890_ILA_MASK);
            
            VpMpiCmdWrapper(deviceId, ecVal, VP890_ICR5_RD, VP890_ICR5_LEN, icr5Speedup);
            icr5Speedup[VP890_ICR5_FEED_HOLD_INDEX] &= ~VP890_ICR5_FEED_HOLD_MASK;
            /* Device value is x + 18mA, so threshold is > 35mA */
            if ((pLineObj->calLineData.dcFeedRef[VP890_ILA_INDEX] & VP890_ILA_MASK) > 17) {
                icr5Speedup[VP890_ICR5_FEED_HOLD_INDEX] |= 0xF0;
            } else {
                icr5Speedup[VP890_ICR5_FEED_HOLD_INDEX] |= 0xA0;
            }
            VpMpiCmdWrapper(deviceId, ecVal, VP890_ICR5_WRT, VP890_ICR5_LEN, icr5Speedup);
            
        }
        /* Adjust for calibration.  This will also program the DC feed register. */
        Vp890UpdateCalValue(pLineCtx);
    }

    /* Program the switching regulator params (battery floor) */
    if (pDcFeedParams->validMask & VP_OPTION_CFG_BATT_FLOOR) {
        pDevObj->devProfileData.swParams[VP890_SWREG_FLOOR_V_BYTE]
            &= ~VP890_SWREG_FLOOR_V_MASK;
        pDevObj->devProfileData.swParams[VP890_SWREG_FLOOR_V_BYTE]
            |= battFloor;

        /* Do not update the floor voltage in low power, the shadow register is still updated */
        if ((pDevObj->stateInt & VP890_LINE0_LP) == FALSE) {
            VpMpiCmdWrapper(deviceId, ecVal, VP890_REGULATOR_PARAM_WRT,
                VP890_REGULATOR_PARAM_LEN, pDevObj->devProfileData.swParams);
            VpMemCpy(pDevObj->swParamsCache, pDevObj->devProfileData.swParams,
                VP890_REGULATOR_PARAM_LEN);
        }
    }

    /* Program the loop supervision parameters (TSH, TGK) */
    if ((pDcFeedParams->validMask & VP_OPTION_CFG_HOOK_THRESHOLD) ||
        (pDcFeedParams->validMask & VP_OPTION_CFG_GKEY_THRESHOLD))
    {
        if (pDcFeedParams->validMask & VP_OPTION_CFG_HOOK_THRESHOLD) {
            pLineObj->loopSup[VP890_LOOP_SUP_THRESH_BYTE] &= ~VP890_SWHOOK_THRESH_MASK;
            pLineObj->loopSup[VP890_LOOP_SUP_THRESH_BYTE] |= (tsh & VP890_SWHOOK_THRESH_MASK);
        }
        if (pDcFeedParams->validMask & VP_OPTION_CFG_GKEY_THRESHOLD) {
            pLineObj->loopSup[VP890_LOOP_SUP_THRESH_BYTE] &= ~VP890_GKEY_THRESH_MASK;
            pLineObj->loopSup[VP890_LOOP_SUP_THRESH_BYTE] |= ((tgk << 3) & VP890_GKEY_THRESH_MASK);
        }
        VpMpiCmdWrapper(deviceId, ecVal, VP890_LOOP_SUP_WRT, VP890_LOOP_SUP_LEN, pLineObj->loopSup);
    }

    return VP_STATUS_SUCCESS;
}


/** SetOptionRingingParams()
  Applies the VP_OPTION_ID_RINGING_PARAMS option for a line.
*/
VpStatusType
SetOptionRingingParams(
    VpLineCtxType *pLineCtx,
    VpOptionRingingParamsType *pRingingParams)
{
    Vp890LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp890DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal = pLineObj->ecVal;
    uint8 channelId = pLineObj->channelId;
    bool trapezoidal = FALSE;
    int16 amplitude = 0;
    int16 frequency = 0;
    int16 bias = 0;
    uint8 rtth = 0;
    uint8 ilr = 0;
    int16 riseTime = 0;
    int16 biasErr;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+SetOptionRingingParams()"));

    /* Perform all error checks and calculations BEFORE changing cached values
       or accessing the device to make this atomic. */

    if (pRingingParams->validMask == 0) {
        return VP_STATUS_SUCCESS;
    }

    if (pRingingParams->validMask == 0xFFFFFFFF) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("SetOptionRingingParams - validMask should not be blindly set to 0xFFFFFFFF"));
        return VP_STATUS_INVALID_ARG;
    }

    if ((pLineObj->ringingParamsRef[0] & VP890_SIGGEN1_SINTRAP_MASK) == VP890_SIGGEN1_TRAP) {
        trapezoidal = TRUE;
    } else {
        trapezoidal = FALSE;
    }

    if (pRingingParams->validMask & VP_OPTION_CFG_FREQUENCY) {
        if (!trapezoidal) {
            /* Frequency is a 15-bit value with a 0-12000 Hz range.  12000000 mHz
               per 0x8000 reduces to 46875 mHz per 0x80. */
            if (pRingingParams->frequency < VP890_FREQ_MIN || pRingingParams->frequency >= VP890_FREQ_MAX) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("SetOptionRingingParams - frequency out of range (%ld)",
                    pRingingParams->frequency));
                return VP_STATUS_INVALID_ARG;
            }
            frequency = (int16)VpRoundedDivide(pRingingParams->frequency * VP890_FREQ_STEP_DEN, VP890_FREQ_STEP_NUM);
        } else {
            /* For trapezoidal ringing, frequency is defined as 8000Hz / FRQB.
               This means the maximum is 8000 Hz (8000000 mHz), and the minimum
               is 8000Hz / 0x7FFF, or 244.1 mHz.  FRQB can be calculated by
               8000000mHz / frequency. */
            if (pRingingParams->frequency <= VP890_TRAPFREQ_MIN || pRingingParams->frequency > VP890_TRAPFREQ_MAX) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("SetOptionRingingParams - frequency out of range (%ld)",
                    pRingingParams->frequency));
                return VP_STATUS_INVALID_ARG;
            }
            frequency = (int16)VpRoundedDivide(VP890_TRAPFREQ_MAX, pRingingParams->frequency);
        }
    }

    if (pRingingParams->validMask & VP_OPTION_CFG_AMPLITUDE) {
        /* Amplitude is a 16-bit value with a +/- 155V range.  This
           means 155000 mV per 0x8000, which reduces to 19375 mV per 0x1000. */
        if (pRingingParams->amplitude < VP890_AMP_MIN || pRingingParams->amplitude >= VP890_AMP_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("SetOptionRingingParams - amplitude out of range (%ld)",
                pRingingParams->amplitude));
            return VP_STATUS_INVALID_ARG;
        }
        amplitude = (int16)VpRoundedDivide(pRingingParams->amplitude * VP890_AMP_STEP_DEN, VP890_AMP_STEP_NUM);
    }

    if (pRingingParams->validMask & VP_OPTION_CFG_DC_BIAS) {
        /* DC Bias is a 16-bit value with a +/- 154.4V range.  This
           means 154400 mV per 0x8000, which reduces to 4825 mV per 0x400. */
        if (pRingingParams->dcBias < VP890_BIAS_MIN || pRingingParams->dcBias > VP890_BIAS_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("SetOptionRingingParams - dcBias out of range (%ld)",
                pRingingParams->dcBias));
            return VP_STATUS_INVALID_ARG;
        }
        bias = (int16)VpRoundedDivide(pRingingParams->dcBias * VP890_BIAS_STEP_DEN, VP890_BIAS_STEP_NUM);
    }

    if (pRingingParams->validMask & VP_OPTION_CFG_RINGTRIP_THRESHOLD) {
        /* Ring trip threshold is a 7-bit value with a 0-63.5 mA scale (0.5 mA
           per step). */
        if (pRingingParams->ringTripThreshold < VP890_RTTH_MIN || pRingingParams->ringTripThreshold > VP890_RTTH_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("SetOptionRingingParams - ringTripThreshold out of range (%ld)",
                pRingingParams->ringTripThreshold));
            return VP_STATUS_INVALID_ARG;
        }
        rtth = VpRoundedDivide(pRingingParams->ringTripThreshold - VP890_RTTH_OFFSET, VP890_RTTH_STEP);
    }

    if (pRingingParams->validMask & VP_OPTION_CFG_RING_CURRENT_LIMIT) {
        /* Ring current limit is a 5-bit value with a 50-112 mA scale (2 mA per
           step + 50 mA offset). */
        if (pRingingParams->ringCurrentLimit < VP890_ILR_MIN || pRingingParams->ringCurrentLimit > VP890_ILR_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("SetOptionRingingParams - ringCurrentLimit out of range (%ld)",
                pRingingParams->ringCurrentLimit));
            return VP_STATUS_INVALID_ARG;
        }
        ilr = VpRoundedDivide(pRingingParams->ringCurrentLimit - VP890_ILR_OFFSET, VP890_ILR_STEP);
    }

    if (trapezoidal && (pRingingParams->validMask & VP_OPTION_CFG_TRAP_RISE_TIME)) {
        /* Trapezoidal rise time is defined as 2.7307sec / FRQA. This means the
           maximum is 2.7307 sec (2730700 usec), and the minimum
           is 2.7307sec / 0x7FFF, or 83.3 usec.  FRQA can be calculated by
           2730700usec / trapRiseTime. */
        if (pRingingParams->trapRiseTime <= VP890_TRAPRISE_MIN || pRingingParams->trapRiseTime > VP890_TRAPRISE_MAX) {
            VP_ERROR(VpLineCtxType, pLineCtx, ("SetOptionRingingParams - trapRiseTime out of range (%ld)",
                pRingingParams->trapRiseTime));
            return VP_STATUS_INVALID_ARG;
        }
        riseTime = (int16)VpRoundedDivide(VP890_TRAPRISE_MAX, pRingingParams->trapRiseTime);
    }


    /* Program the ringing generator parameters (frequency, amplitude, bias) */
    if ((pRingingParams->validMask & VP_OPTION_CFG_FREQUENCY) ||
        (pRingingParams->validMask & VP_OPTION_CFG_AMPLITUDE) ||
        (pRingingParams->validMask & VP_OPTION_CFG_DC_BIAS) ||
        (trapezoidal && (pRingingParams->validMask & VP_OPTION_CFG_TRAP_RISE_TIME)))
    {
        if (!trapezoidal && (pRingingParams->validMask & VP_OPTION_CFG_FREQUENCY)) {
            /* FRQA for sine ringing */
            pLineObj->ringingParamsRef[VP890_SIGA_FREQ_MSB] = (frequency & 0x7F00) >> 8;
            pLineObj->ringingParamsRef[VP890_SIGA_FREQ_LSB] = (frequency & 0x00FF);
        }
        if (trapezoidal && (pRingingParams->validMask & VP_OPTION_CFG_FREQUENCY)) {
            /* FRQB for trapezoidal ringing */
            pLineObj->ringingParamsRef[VP890_SIGB_FREQ_MSB] = (frequency & 0x7F00) >> 8;
            pLineObj->ringingParamsRef[VP890_SIGB_FREQ_LSB] = (frequency & 0x00FF);
        }
        if (pRingingParams->validMask & VP_OPTION_CFG_AMPLITUDE) {
            pLineObj->ringingParamsRef[VP890_SIGA_AMP_MSB] = (amplitude & 0xFF00) >> 8;
            pLineObj->ringingParamsRef[VP890_SIGA_AMP_LSB] = (amplitude & 0x00FF);
        }
        if (pRingingParams->validMask & VP_OPTION_CFG_DC_BIAS) {
            pLineObj->ringingParamsRef[VP890_SIGA_BIAS_MSB] = (bias & 0xFF00) >> 8;
            pLineObj->ringingParamsRef[VP890_SIGA_BIAS_LSB] = (bias & 0x00FF);
        }
        if (trapezoidal && (pRingingParams->validMask & VP_OPTION_CFG_TRAP_RISE_TIME)) {
            pLineObj->ringingParamsRef[VP890_SIGA_FREQ_MSB] = (riseTime & 0x7F00) >> 8;
            pLineObj->ringingParamsRef[VP890_SIGA_FREQ_LSB] = (riseTime & 0x00FF);
        }
        /* If the line is currently ringing, calculate new calibration
           adjustments and update the siggen register.  If it is not ringing,
           the adjustments will be made in Vp890SetStateRinging() the next time
           it starts. */
        if (pLineObj->lineState.currentState == VP_LINE_RINGING) {
            VpMemCpy(pLineObj->ringingParams, pLineObj->ringingParamsRef, VP890_RINGER_PARAMS_LEN);
            biasErr = (int16)((((uint16)(pLineObj->ringingParams[1]) << 8) & 0xFF00) +
                ((uint16)(pLineObj->ringingParams[2]) & 0x00FF));
            biasErr -= ((pDevObj->vp890SysCalData.sigGenAError[channelId][0] -
                pDevObj->vp890SysCalData.vocOffset[channelId][VP890_NORM_POLARITY]) *
                16L / 10L);
            pLineObj->ringingParams[VP890_SIGA_BIAS_MSB] = (uint8)((biasErr >> 8) & 0x00FF);
            pLineObj->ringingParams[VP890_SIGA_BIAS_LSB] = (uint8)(biasErr & 0x00FF);

            VpMpiCmdWrapper(deviceId, ecVal, VP890_RINGER_PARAMS_WRT,
                VP890_RINGER_PARAMS_LEN, pLineObj->ringingParams);
        
            pLineObj->status |= VP890_RING_GEN_NORM;
            pLineObj->status &= ~VP890_RING_GEN_REV;
        } else if (pLineObj->lineState.currentState == VP_LINE_RINGING_POLREV) {
            VpMemCpy(pLineObj->ringingParams, pLineObj->ringingParamsRef, VP890_RINGER_PARAMS_LEN);
            biasErr = (int16)((((uint16)(pLineObj->ringingParams[1]) << 8) & 0xFF00) +
                ((uint16)(pLineObj->ringingParams[2]) & 0x00FF));
            biasErr += ((pDevObj->vp890SysCalData.sigGenAError[channelId][0] -
                pDevObj->vp890SysCalData.vocOffset[channelId][VP890_REV_POLARITY]) *
                16L / 10L);
            pLineObj->ringingParams[VP890_SIGA_BIAS_MSB] = (uint8)((biasErr >> 8) & 0x00FF);
            pLineObj->ringingParams[VP890_SIGA_BIAS_LSB] = (uint8)(biasErr & 0x00FF);

            VpMpiCmdWrapper(deviceId, ecVal, VP890_RINGER_PARAMS_WRT,
                VP890_RINGER_PARAMS_LEN, pLineObj->ringingParams);
        
            pLineObj->status &= ~VP890_RING_GEN_NORM;
            pLineObj->status |= VP890_RING_GEN_REV;
        } else {
            pLineObj->status &= ~VP890_RING_GEN_NORM;
            pLineObj->status &= ~VP890_RING_GEN_REV;
        }
    }

    /* If the amplitude or bias is changing, update the switcher ringing voltage */
    if ((pRingingParams->validMask & VP_OPTION_CFG_AMPLITUDE) ||
        (pRingingParams->validMask & VP_OPTION_CFG_DC_BIAS))
    {
        int32 amplitude_mV;
        int32 bias_mV;
        int32 battery_mV;
        uint8 swrv;

        /* Get the bias and amplitude from the line object, since it's not
           guaranteed that BOTH were adjusted above. */
        amplitude = (int16)pLineObj->ringingParamsRef[VP890_SIGA_AMP_MSB] << 8;
        amplitude |= (int16)pLineObj->ringingParamsRef[VP890_SIGA_AMP_LSB];
        bias = (int16)pLineObj->ringingParamsRef[VP890_SIGA_BIAS_MSB] << 8;
        bias |= (int16)pLineObj->ringingParamsRef[VP890_SIGA_BIAS_LSB];

        /* Calculate the programmed amplitude and bias in mV.
           int16 range is +/- 154.4V (154400 mV per 0x8000, reduces to 4825 mV
           per 0x400). */
        amplitude_mV = VpRoundedDivide(amplitude * VP890_AMP_STEP_NUM, VP890_AMP_STEP_DEN); 
        bias_mV = VpRoundedDivide(bias * VP890_BIAS_STEP_NUM, VP890_BIAS_STEP_DEN); 

        /* The battery should be AT LEAST 5V higher than the sum of the
           absolute values of bias and amplitude. */
        battery_mV = ABS(amplitude_mV) + ABS(bias_mV) + 5000;

        /* SWRV is a 5-bit value with a range of 5-160 V (step size of 5 V +
           5 V offset).  Round up. */
        /* Round to the nearest Volt */
        swrv = VpRoundedDivide(battery_mV, 1000);
        if (swrv > 160) {
            swrv = 160;
        }
        /* Disallow setting the switcher voltage below 60 so that it can not
           run into VOC when entering ringing. */
        if (swrv < 60) {
            swrv = 60;
        }
        /* Round up */
        swrv = (swrv - 5 + 4) / 5;

        pDevObj->devProfileData.swParams[VP890_SWREG_RING_V_BYTE]
            &= ~VP890_SWREG_RING_V_MASK;
        pDevObj->devProfileData.swParams[VP890_SWREG_RING_V_BYTE]
            |= (swrv & VP890_SWREG_RING_V_MASK);
        pDevObj->swParamsCache[VP890_SWREG_RING_V_BYTE] =
            pDevObj->devProfileData.swParams[VP890_SWREG_RING_V_BYTE];

        VpMpiCmdWrapper(deviceId, ecVal, VP890_REGULATOR_PARAM_WRT,
            VP890_REGULATOR_PARAM_LEN, pDevObj->swParamsCache);
    }
    
    /* Program the loop supervision parameters (RTTH, ILR) */
    if ((pRingingParams->validMask & VP_OPTION_CFG_RINGTRIP_THRESHOLD) ||
        (pRingingParams->validMask & VP_OPTION_CFG_RING_CURRENT_LIMIT))
    {
        if (pRingingParams->validMask & VP_OPTION_CFG_RINGTRIP_THRESHOLD) {
            pLineObj->loopSup[VP890_LOOP_SUP_RT_MODE_BYTE] &= ~VP890_RINGTRIP_THRESH_MASK;
            pLineObj->loopSup[VP890_LOOP_SUP_RT_MODE_BYTE] |= (rtth & VP890_RINGTRIP_THRESH_MASK);
        }
        if (pRingingParams->validMask & VP_OPTION_CFG_RING_CURRENT_LIMIT) {
            pLineObj->loopSup[VP890_LOOP_SUP_ILR_BYTE] &= ~VP890_LOOP_SUP_RING_LIM_MASK;
            pLineObj->loopSup[VP890_LOOP_SUP_ILR_BYTE] |= (ilr & VP890_LOOP_SUP_RING_LIM_MASK);
        }
        VpMpiCmdWrapper(deviceId, ecVal, VP890_LOOP_SUP_WRT, VP890_LOOP_SUP_LEN, pLineObj->loopSup);
    }

    return VP_STATUS_SUCCESS;
}
#endif /* VP890_FXS_SUPPORT */


/*******************************************************************************
 * MaskNonSupportedEvents()
 *  This function masks the events that are not supported by the VP890 API-II.
 * It should only be called by SetOptionInternal when event masks are being
 * modified.
 *
 * Arguments:
 *   pLineEventsMask - Line Events Mask to modify for non-masking
 *   pDevEventsMask  - Device Events Mask to modify for non-masking
 *
 * Preconditions:
 *  None. Utility function to modify event structures only.
 *
 * Postconditions:
 *  Event structures passed are modified with masked bits for non-supported
 * VP890 API-II events.
 ******************************************************************************/
void
MaskNonSupportedEvents(
    VpOptionEventMaskType *pLineEventsMask,
    VpOptionEventMaskType *pDevEventsMask)
{
    VP_API_FUNC_INT(None, VP_NULL, ("+MaskNonSupportedEvents()"));

    pLineEventsMask->faults     |= VP890_NONSUPPORT_FAULT_EVENTS;
    pLineEventsMask->signaling  |= VP890_NONSUPPORT_SIGNALING_EVENTS;
    pLineEventsMask->response   |= VP890_NONSUPPORT_RESPONSE_EVENTS;
    pLineEventsMask->test       |= VP890_NONSUPPORT_TEST_EVENTS;
    pLineEventsMask->process    |= VP890_NONSUPPORT_PROCESS_EVENTS;
    pLineEventsMask->fxo        |= VP890_NONSUPPORT_FXO_EVENTS;

    pDevEventsMask->faults      |= VP890_NONSUPPORT_FAULT_EVENTS;
    pDevEventsMask->signaling   |= VP890_NONSUPPORT_SIGNALING_EVENTS;
    pDevEventsMask->response    |= VP890_NONSUPPORT_RESPONSE_EVENTS;
    pDevEventsMask->test        |= VP890_NONSUPPORT_TEST_EVENTS;
    pDevEventsMask->process     |= VP890_NONSUPPORT_PROCESS_EVENTS;
    pDevEventsMask->fxo         |= VP890_NONSUPPORT_FXO_EVENTS;

    return;
} /* MaskNonSupportedEvents() */

/*******************************************************************************
 * Vp890SetCodec()
 *  This function sets the codec mode on the line specified.
 *
 * Arguments:
 *   pLineCtx   - Line Context
 *   codec      - Encoding, as defined by LineCodec typedef
 *
 * Preconditions:
 *  The line must first be initialized.
 *
 * Postconditions:
 *  The codec mode on the line is set.  This function returns the success code
 * if the codec mode specified is supported.
 ******************************************************************************/
VpStatusType
Vp890SetCodec(
    VpLineCtxType           *pLineCtx,
    VpOptionCodecType       codec)
{
    Vp890LineObjectType     *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType            *pDevCtx  = pLineCtx->pDevCtx;
    Vp890DeviceObjectType   *pDevObj  = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId  = pDevObj->deviceId;

    bool                    isNextWideBand      = FALSE;
    bool                    isCurrentWideBand   = FALSE;

    uint8                   codecReg;
    uint8                   ecVal     = pLineObj->ecVal;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890SetCodec()"));

    /* Basic error checking and adjust ecVal as needed */
    switch(codec) {
        case VP_OPTION_ALAW:                /**< Select G.711 A-Law PCM encoding */
        case VP_OPTION_MLAW:                /**< Select G.711 Mu-Law PCM encoding */
        case VP_OPTION_LINEAR:              /**< Select Linear PCM encoding */
            ecVal &= ~VP890_WIDEBAND_MODE;
            break;

        case VP_OPTION_WIDEBAND:            /**< Select Wideband Linear PCM encoding */
        case VP_OPTION_ALAW_WIDEBAND:       /**< Select Wideband A-Law PCM encoding */
        case VP_OPTION_MLAW_WIDEBAND:       /**< Select Wideband Mu-Law PCM encoding */
            isNextWideBand = TRUE;
            ecVal |= VP890_WIDEBAND_MODE;
            break;

        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890SetCodec() - Invalid codec"));
            return VP_STATUS_INVALID_ARG;
    }

    /*
     * Wideband requires 1/2 rate reduction in device programmed rate to maintain the same real
     * sample rate. So we have a rate change when going to/from Wideband mode. Compare the current
     * and new CODEC state relative to the Wideband condition, ignoring compression/linear for this
     * test.
     */
    if((pLineObj->codec == VP_OPTION_WIDEBAND) ||
        (pLineObj->codec == VP_OPTION_ALAW_WIDEBAND) ||
        (pLineObj->codec == VP_OPTION_MLAW_WIDEBAND)) {
        isCurrentWideBand = TRUE;
    }

    if(isCurrentWideBand != isNextWideBand) {
        uint8 converterCfg[VP890_CONV_CFG_LEN];
        uint8 newValue;

        pDevObj->devTimer[VP_DEV_TIMER_WB_MODE_CHANGE] =
            MS_TO_TICKRATE(VP_WB_CHANGE_MASK_TIME,
            pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

        VpMpiCmdWrapper(deviceId, ecVal, VP890_CONV_CFG_RD, VP890_CONV_CFG_LEN, converterCfg);
        converterCfg[0] &= ~VP890_CC_RATE_MASK;

        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("CODEC Change to %sWideband Mode at Time %d",
            ((isNextWideBand) ? "" : "non-"), pDevObj->timeStamp));

        /*
         * Adjust the pcm buffer update rate based on the tickrate and next CODEC setting. If
         * changing TO Wideband mode, the silicon clocks run 2x quicker. So to compensate such
         * that all of the buffers behave the same from the VP-API-II perspective we have to
         * reprogram the device to 1/2 the previous setting. This doesn't make them run at 1/2 the
         * previous speed, just 1/2 the speed it would run had we ONLY changed the CODEC mode to
         * Wideband Mode. It's just that changing to Wideband mode automatically doubles the clock
         * rate of the buffers - otherwise known as an "unintended consequence"
         */
        /*
         * Note that these are not the exact points where the buffer would be full AT the tick
         * service point. As a guide, we chose 10ms/2ms update, and 5ms/1ms update rates as the
         * baseline and other values divided down from there. This allows for some tolerance in the
         * tick servicing which is obviously needed.
         */
        if(pDevObj->devProfileData.tickRate <=160) {        /* 160 * 3.90625uS = 0.625mS */
            newValue = ((isNextWideBand) ? VP890_CC_4KHZ_RATE : VP890_CC_8KHZ_RATE);
        } else if(pDevObj->devProfileData.tickRate <=320){  /* 320 * 3.90625uS = 1.25mS */
            newValue = ((isNextWideBand) ? VP890_CC_2KHZ_RATE : VP890_CC_4KHZ_RATE);
        } else if(pDevObj->devProfileData.tickRate <=640){  /* 640 * 3.90625uS = 2.5mS */
            newValue = ((isNextWideBand) ? VP890_CC_1KHZ_RATE : VP890_CC_2KHZ_RATE);
        } else if(pDevObj->devProfileData.tickRate <=1280){ /* 1280 * 3.90625uS = 5mS */
            newValue = ((isNextWideBand) ? VP890_CC_500HZ_RATE : VP890_CC_1KHZ_RATE);
        } else {
            newValue = VP890_CC_500HZ_RATE;
        }

        pDevObj->txBufferDataRate = newValue;
        converterCfg[0] |= newValue;

        /*
         * If channel is going to Wideband mode, we can immediately update the device object. But
         * if leaving Wideband mode, we have to let the tick manage it because the other line may
         * still be in Wideband mode.
         */
        if (isNextWideBand) {
            pDevObj->ecVal |= VP890_WIDEBAND_MODE;
        }

        VpMpiCmdWrapper(deviceId, ecVal, VP890_CONV_CFG_WRT, VP890_CONV_CFG_LEN, converterCfg);
        /*
         * This value cannot be set to 0 BECAUSE "0" is used to indicate there has been no recent
         * changes. So use channelId's starting index = 1.
         */
        pDevObj->lastCodecChange = pLineObj->channelId + 1;
    }

    /* Read the current state of the codec register */
    VpMpiCmdWrapper(deviceId, ecVal, VP890_OP_FUNC_RD, VP890_OP_FUNC_LEN, &codecReg);

    if (!VpCSLACGetCodecReg(codec, &codecReg)) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890SetCodec() - Failure"));
        return VP_STATUS_FAILURE;
    }

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("1. Writing 0x%02X to Operating Functions EC 0x%02X",
        codecReg, ecVal));
    VpMpiCmdWrapper(deviceId, ecVal, VP890_OP_FUNC_WRT, VP890_OP_FUNC_LEN, &codecReg);

    pLineObj->codec = codec;
    pLineObj->ecVal = ecVal;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp890SetCodec()"));

    return VP_STATUS_SUCCESS;
} /* Vp890SetCodec() */

/*******************************************************************************
 * SetTimeSlot()
 *  This function set the RX and TX timeslot for a device channel. Valid
 * timeslot numbers start at zero. The upper bound is system dependent.
 *
 * Arguments:
 *   pLineCtx   - Line Context
 *   txSlot     - The TX PCM timeslot
 *   rxSlot     - The RX PCM timeslot
 *
 * Preconditions:
 *  The line must first be initialized.
 *
 * Postconditions:
 *  The timeslots on the line are set.  This function returns the success code
 * if the timeslot numbers specified are within the range of the device based on
 * the PCLK rate.
 ******************************************************************************/
static VpStatusType
SetTimeSlot(
    VpLineCtxType           *pLineCtx,
    uint8                   txSlot,
    uint8                   rxSlot)
{
    Vp890LineObjectType     *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType            *pDevCtx  = pLineCtx->pDevCtx;
    Vp890DeviceObjectType   *pDevObj  = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId  = pDevObj->deviceId;

    uint8                   mpiBuffer[2 + VP890_TX_TS_LEN + VP890_RX_TS_LEN];
    uint8                   mpiIndex = 0;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+SetTimeSlot()"));

    /* device only supports 127 time slots */
    if ((VP890_TX_TS_MAX < txSlot) || (VP890_RX_TS_MAX < rxSlot)) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("SetTimeSlot() - Time slot out of range.  rx %d, tx %d",
            rxSlot, txSlot));
        return VP_STATUS_INPUT_PARAM_OOR;
    }

    /* Validate the tx time slot value */
    if(txSlot >= pDevObj->devProfileData.pcmClkRate / 64) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("SetTimeSlot() - Bad Tx time slot value"));
        return VP_STATUS_INPUT_PARAM_OOR;
    }

    /* Validate the rx time slot value */
    if(rxSlot >= pDevObj->devProfileData.pcmClkRate / 64) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("SetTimeSlot() - Bad Rx time slot value"));
        return VP_STATUS_INVALID_ARG;
    }

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_TX_TS_WRT,
        VP890_TX_TS_LEN, &txSlot);

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP890_RX_TS_WRT,
        VP890_RX_TS_LEN, &rxSlot);

    VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1,
        &mpiBuffer[1]);

    return VP_STATUS_SUCCESS;
} /* SetTimeSlot() */

/*******************************************************************************
 * Vp890DeviceIoAccess()
 *  This function is used to access device IO pins of the Vp890. See API-II
 * documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Reads/Writes from device IO pins.
 ******************************************************************************/
VpStatusType
Vp890DeviceIoAccess(
    VpDevCtxType             *pDevCtx,
    VpDeviceIoAccessDataType *pDeviceIoData)
{
    Vp890DeviceObjectType    *pDevObj   = pDevCtx->pDevObj;
    VpLineCtxType            *pLineCtx;
    Vp890LineObjectType      *pLineObj;
    VpDeviceIdType           deviceId   = pDevObj->deviceId;
    uint8                    ecVal      = 0;
    uint8                    ioData;
    uint8                    chanId;
    VpStatusType             status     = VP_STATUS_SUCCESS;

    VpDeviceIoAccessDataType *pAccessData =
        &(pDevObj->getResultsOption.optionData.deviceIoData);

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("+Vp890DeviceIoAccess()"));

    /* Get out if device state is not ready */
    if (!Vp890IsDevReady(pDevObj->state, TRUE)) {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Get wideband mode info from each channel's ecVal. */
    for(chanId=0; chanId < VP890_MAX_NUM_CHANNELS; chanId++ ) {
        pLineCtx = pDevCtx->pLineCtx[chanId];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            ecVal |= pLineObj->ecVal;

            if ((pLineObj->termType == VP_TERM_FXS_SPLITTER_LP)
             || (pLineObj->termType == VP_TERM_FXS_SPLITTER)
             || (pLineObj->termType == VP_TERM_FXS_ISOLATE)) {
                if ((pDeviceIoData->accessMask_31_0 & 0x1)
                 && (pDeviceIoData->accessType == VP_DEVICE_IO_WRITE)) {
                    status = VP_STATUS_DEDICATED_PINS;
                }
                /* Can't let I/O1 Change on Splitter/Isolate Types */
                pDeviceIoData->accessMask_31_0 &= 0xfffffffe;
            }
        }
    }

    *pAccessData = *pDeviceIoData;

    VpMpiCmdWrapper(deviceId, ecVal, VP890_IODATA_REG_RD, VP890_IODATA_REG_LEN,
        &ioData);
    if (pDeviceIoData->accessType == VP_DEVICE_IO_READ) {
        /* Typecast ioData to be at the same size/type as other values in bit manipulation */
        pAccessData->deviceIOData_31_0 = (uint32)ioData & pDeviceIoData->accessMask_31_0;
        pAccessData->deviceIOData_63_32 = 0;
    } else { /* pDeviceIoData->accessType == VP_DEVICE_IO_WRITE */
        ioData &= (uint8)(~(pDeviceIoData->accessMask_31_0 & VP890_IODATA_IOMASK));
        ioData |= (uint8)(pDeviceIoData->deviceIOData_31_0 &
            (pDeviceIoData->accessMask_31_0 & VP890_IODATA_IOMASK));
        VpMpiCmdWrapper(deviceId, ecVal, VP890_IODATA_REG_WRT, VP890_IODATA_REG_LEN,
            &ioData);
    }
    pDevObj->deviceEvents.response |= VP_DEV_EVID_IO_ACCESS_CMP;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, (" -Vp890DeviceIoAccess()"));
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return status;
} /* Vp890DeviceIoAccess() */

/*******************************************************************************
 * Vp890LowLevelCmd()
 *  This function provides direct MPI access to the line/device.
 *
 * Preconditions:
 *  The device associated with the line, and the line must first be initialized.
 *
 * Postconditions:
 *  The command data is passed over the MPI bus and affects only the line passed
 * if the command is line specific, and an event is generated.  If a read
 * command is performed, the user must read the results or flush events.  This

 * function returns the success code if the device is not already in a state
 * where the results must be read.
 ******************************************************************************/
#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
VpStatusType
Vp890LowLevelCmd(
    VpLineCtxType       *pLineCtx,
    uint8               *pCmdData,
    uint8               len,
    uint16              handle)
{
    Vp890LineObjectType     *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType            *pDevCtx  = pLineCtx->pDevCtx;
    Vp890DeviceObjectType   *pDevObj  = pDevCtx->pDevObj;
    VpDeviceIdType          deviceId  = pDevObj->deviceId;
    uint8                   ecVal     = pLineObj->ecVal;

    int i; /* For-loop var */

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp890LowLevelCmd()"));

    if (pLineObj->lineEvents.response & VP890_READ_RESPONSE_MASK) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp890LowLevelCmd() - Waiting to clear previous read"));
        return VP_STATUS_DEVICE_BUSY;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    if(pCmdData[0] & 0x01) { /* Read Command */
        VpMpiCmdWrapper(deviceId, ecVal, pCmdData[0], len, &(pDevObj->mpiData[0]));
        pDevObj->mpiLen = len;
        pLineObj->lineEvents.response |= VP_LINE_EVID_LLCMD_RX_CMP;
    } else {
        VpMpiCmdWrapper(deviceId, ecVal, pCmdData[0], len, &pCmdData[1]);
        for (i = 0; i < len; i++) {
            pDevObj->mpiData[i] = pCmdData[i];
        }
        pLineObj->lineEvents.response |= VP_LINE_EVID_LLCMD_TX_CMP;
    }
    pLineObj->lineEventHandle = handle;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
} /* Vp890LowLevelCmd() */
#endif

#endif /* VP_CC_890_SERIES */

