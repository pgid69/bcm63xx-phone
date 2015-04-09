/** \file vp880_control_common.c
 * vp880_control_common.c
 *
 *  This file contains the control functions for the Vp880 device API.
 *
 * Copyright (c) 2012, Microsemi Corporation
 *
 * $Revision: 11126 $
 * $LastChangedDate: 2013-08-14 20:32:42 -0500 (Wed, 14 Aug 2013) $
 */
#include "vp_api_cfg.h"

#if defined (VP_CC_880_SERIES)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_hal.h"
#include "vp_api_int.h"
#include "vp880_api.h"
#include "vp880_api_int.h"
#include "sys_service.h"

/**< Profile index for Generator A/B and C/D starting points (std tone) */
#define VP880_SIGGEN_AB_START   (8)
#define VP880_SIGGEN_CD_START   (16)

/**< Function called by Set Option only. Implements the options specified by
 * the user. The calling function implements the Device/Line control. If a line
 * option is set and a device option is passed, the calling function will call
 * this function once for each line and pass it the line contexts. Therefore,
 * this function will only be subjected to either a device context and device
 * option, or a line context and a line option.
 */
static VpStatusType
Vp880SetOptionInternal(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *value);

/* Function called by SetOptionInternal for Event Masking only */
static void
Vp880MaskNonSupportedEvents(
    VpOptionEventMaskType *pLineEventsMask,
    VpOptionEventMaskType *pDevEventsMask);

/* Function called by SetOptionInternal to set tx and rx timeslot */
static VpStatusType
Vp880SetTimeSlot(
    VpLineCtxType *pLineCtx,
    uint8 txSlot,
    uint8 rxSlot);

/**
 * Vp880ApiTick()
 *  This function should be called on a periodic basis or attached to an
 * interrupt.
 *
 * Preconditions:
 *  The device must first be initialized.
 *
 * Postconditions:
 *  The value passed (by pointer) is set to TRUE if there is an updated event.
 * The user should call the GetEventStatus function to determine the cause of
 * the event (TRUE value set).  This function always returns the success code.
 */
VpStatusType
Vp880ApiTick(
    VpDevCtxType *pDevCtx,
    bool *pEventStatus)
{
    VpLineCtxType *pLineCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp880LineObjectType *pLineObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal;
    uint8 numIntServiced = 2;
    uint8 channelId;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;
    bool tempClkFault, tempBat1Fault, tempBat2Fault, lineInTest;
    bool intServCalled = FALSE;
    bool linesCal[] = {FALSE, FALSE};
    uint16 timeStampPre, tickAdder;

#ifdef VP_CSLAC_SEQ_EN
    bool isSeqRunning = FALSE;
#endif

    /*
     * Do NOT Add VP_API_FUNC or VP_API_FUNC_INT to this function or any
     * function directly and constantly called by this function (Calibration
     * processes excluded). It will overload the console during debug.
     */

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

    /*
     * Always reset the device object flag indicating that the PCM buffer has
     * already been read this tick, because it hasn't. This flag is used
     * throughout the VP-API-II to determine whether the PCM buffer needs to be
     * read or whether the data exists in the device object already.
     */
    pDevObj->state &= ~VP_DEV_TEST_BUFFER_READ;

#if defined (VP880_INTERRUPT_LEVTRIG_MODE)
    VpSysEnableInt(deviceId);
#endif

    /* Ensure that device is initialized */
    if (!(pDevObj->state & VP_DEV_INIT_CMP)) {
        if (Vp880FindSoftwareInterrupts(pDevCtx)) {
            *pEventStatus = TRUE;
        }

        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_SUCCESS;
    }

    /* Check if since last tick, one of the lines changed to/from WideBand mode */
    if (pDevObj->lastCodecChange != 0) {
        VpOptionCodecType codecMode;
        /*
         * A wideband mode change was made. Figure out which was the last channel
         * changed and enforce that channel's codec setting on both channels.
         */
        pLineCtx = pDevCtx->pLineCtx[pDevObj->lastCodecChange-1];

        VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Last Codec Change Channel %d",
            (pDevObj->lastCodecChange-1)));

        /*
         * Only way line context can be null here is if the codec mode was just
         * set and before calling the tick, the line context was set "free".
         * Highly unusual, but technically possible. It would mean that the
         * particular line object is no longer needed (because all available
         * line contexts/objects MUST be associated with the device context
         * passed into VpApiTick()).
         */
        /*
         * If a line CODEC mode was just changed to/from Wideband mode, transfer the CODEC mode
         * of the last line changed to all remaining lines of the device. Note that this only
         * occurs changing between Narrowband <-> Wideband, not between different Wideband
         * Compression Modes (e.g., Wideband_Linear <-> Wideband_uLaw).
         */
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            codecMode = pLineObj->codec;

            if((codecMode == VP_OPTION_WIDEBAND) ||
               (codecMode == VP_OPTION_ALAW_WIDEBAND) ||
               (codecMode == VP_OPTION_MLAW_WIDEBAND)) {
                pDevObj->ecVal |= VP880_WIDEBAND_MODE;
            } else {
                pDevObj->ecVal &= ~VP880_WIDEBAND_MODE;
            }
            VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Updating Device ecVal to 0x%02X",
                pDevObj->ecVal));

            /* Force both lines and device to correct wideband mode */
            for(channelId=0; channelId < maxChan; channelId++ ) {
                uint8 codecReg[VP880_OP_FUNC_LEN];

                pLineCtx = pDevCtx->pLineCtx[channelId];
                if (pLineCtx == VP_NULL) {
                    continue;
                }
                pLineObj = pLineCtx->pLineObj;
                /*
                 * We have to adjust the compression portion of the CODEC mode. The "ecVal" portion
                 * corrects the silicon WB/NB behavior, and the line object "codec" makes the codec
                 * options report the same. This step makes the compression modes in the silicon
                 * physically the same.
                 */
                /*
                 * Assume the function will work, so use the new ecVal. If it doesn't work, then it
                 * doesn't much matter what ecVal we use here.
                 */
                pLineObj->ecVal &= ~VP880_WIDEBAND_MODE;
                pLineObj->ecVal |= (pDevObj->ecVal & VP880_WIDEBAND_MODE);

                VpMpiCmdWrapper(deviceId, pLineObj->ecVal, VP880_OP_FUNC_RD, VP880_OP_FUNC_LEN,
                    codecReg);
                if (VpCSLACGetCodecReg(codecMode, codecReg)) {
                    /* Setting to *this* codec mode was ok. Make final settings and continue */
                    VpMpiCmdWrapper(deviceId, pLineObj->ecVal, VP880_OP_FUNC_WRT, VP880_OP_FUNC_LEN,
                        codecReg);
                    pLineObj->codec = codecMode;
                } else {
                    /* Setting to *this* codec mode failed. Don't proceed on current path. */
                    continue;
                }
                VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Updating Line %d ecVal to 0x%02X",
                    pLineObj->channelId, pLineObj->ecVal));
            }
        }
        pDevObj->lastCodecChange = 0;
    }

    ecVal = pDevObj->ecVal;

    /* Service API Timers */
    Vp880ServiceTimers(pDevCtx);

#ifdef VP880_LP_SUPPORT
    /* Service LPM Termination Types. */
    Vp880LowPowerMode(pDevCtx);
#endif

    if (pDevObj->state & VP_DEV_IN_CAL) {
        /*
         * While in calibration, read from the signaling register just so the
         * interrupt line clears. Otherwise, the system will see constant
         * interrupts (if active level) AND the VP-API-II will generate multiple
`        * interrupts when calibration completes.
         */
        VpMpiCmdWrapper(deviceId, ecVal, VP880_UL_SIGREG_RD,
            VP880_UL_SIGREG_LEN, pDevObj->intReg);
        VpMpiCmdWrapper(deviceId, ecVal, VP880_UL_SIGREG_RD,
            VP880_UL_SIGREG_LEN, pDevObj->intReg);

        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_SUCCESS;
   }

    /* Reset event pointers pointers */
    pDevObj->dynamicInfo.lastChan = 0;

    for (channelId = 0; channelId < maxChan; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;

#ifdef VP_CSLAC_SEQ_EN
            /* Evaluate if Cadencing is required */
            if (pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) {
                isSeqRunning = TRUE;
            }
#endif
            /* Determine line and system calibration status */
            if ((pLineObj->status & VP880_IS_FXO) ||
                 (pLineObj->calLineData.calDone == TRUE)) {
                pLineObj->calLineData.calDone = TRUE;
                linesCal[pLineObj->channelId] = TRUE;
            }
            if (pDevObj->stateInt & VP880_CAL_RELOAD_REQ) {
                pLineObj->calLineData.calDone = TRUE;
                linesCal[pLineObj->channelId] = TRUE;
                Vp880UpdateCalValue(pLineCtx);
            }
            if (pDevObj->stateInt & VP880_SYS_CAL_RESET) {
                if (!(pLineObj->status & VP880_IS_FXO)) {
                    pLineObj->calLineData.calDone = FALSE;
                    linesCal[pLineObj->channelId] = FALSE;
                }
            }
            if (pDevObj->stateInt & VP880_SYS_CAL_COMPLETE) {
                pLineObj->calLineData.calDone = TRUE;
                linesCal[pLineObj->channelId] = TRUE;
            }

        }
    }

    if ((linesCal[0] == TRUE) && (linesCal[1] == TRUE)) {
        pDevObj->stateInt |= VP880_SYS_CAL_COMPLETE;
    }

    pDevObj->stateInt &= ~(VP880_CAL_RELOAD_REQ | VP880_SYS_CAL_RESET);

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

#if defined (VP880_EFFICIENT_POLLED_MODE)
    /* Poll the device PIO-INT line */
    pDevObj->state |=
        (VpSysTestInt(deviceId) ? VP_DEV_PENDING_INT : 0x00);
#elif defined (VP880_SIMPLE_POLLED_MODE)
    pDevObj->state |= VP_DEV_PENDING_INT;
#endif

    /*
     * Adjust the EC value for Wideband mode as needed and set the line test
     * flag if any line is under test.
     */
    lineInTest = FALSE;

#if defined (VP880_INCLUDE_TESTLINE_CODE)
    for (channelId = 0; channelId < maxChan; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            if (Vp880IsChnlUndrTst(pDevObj, channelId) == TRUE) {
                lineInTest = TRUE;
            }
        }
    }

    /*
     * Also want to consider a line in test if running Read Loop Conditions.
     * But if in Read Loop Conditions, the function "..IsChn" returns FALSE.
     */
    lineInTest = ((pDevObj->currentTest.nonIntrusiveTest == TRUE) ? TRUE : lineInTest);
#endif

    /* Read the PCM buffer once per tick IF there is line under test. */
    if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) {
        if ((lineInTest == TRUE) && (!(pDevObj->state & VP_DEV_TEST_BUFFER_READ))) {
            VpMpiCmdWrapper(deviceId, ecVal, VP880_TX_PCM_BUFF_RD,
                VP880_TX_PCM_BUFF_LEN, pDevObj->txBuffer);
            pDevObj->state |= VP_DEV_TEST_BUFFER_READ;
        }
    }

    /* Service all pending interrupts (up to 2) */
    while ((pDevObj->state & VP_DEV_PENDING_INT) && (numIntServiced > 0)) {
        VpMpiCmdWrapper(deviceId, ecVal, VP880_UL_SIGREG_RD,
            VP880_UL_SIGREG_LEN, pDevObj->intReg);

        if (numIntServiced == 2) {
            VpMemCpy(pDevObj->intReg2, pDevObj->intReg, VP880_UL_SIGREG_LEN);
        }

        /*******************************************************
         *         HANDLE Clock Fail Events                    *
         *******************************************************/
        if (!(pDevObj->devTimer[VP_DEV_TIMER_WB_MODE_CHANGE] & VP_ACTIVATE_TIMER)) {
            /* Get the current status of the fault bit */
            tempClkFault = (pDevObj->intReg[0] & VP880_CFAIL_MASK) ? TRUE : FALSE;
            /*
             * Compare it with what we already know.  If different, generate
             * events and update the line status bits
             */
            if(tempClkFault ^ pDevObj->dynamicInfo.clkFault) {
#ifdef VP880_FXS_SUPPORT
                if (!(pDevObj->stateInt & VP880_FORCE_FREE_RUN)) {
                    if (tempClkFault) {
                        /* Entering clock fault, possibly a system restart. */
                        Vp880FreeRun(pDevCtx, VP_FREE_RUN_START);

                        /*
                         * Clear the flag used to indicate that Vp880FreeRun() was
                         * called by the application -- because it wasn't.
                         */
                        pDevObj->stateInt &= ~VP880_FORCE_FREE_RUN;
                    } else {
                        /*
                         * Exiting clock fault (note: this function does not affect
                         * VP880_FORCE_FREE_RUN flag).
                         */
                        Vp880RestartComplete(pDevCtx);
                    }
                }
#endif
                pDevObj->dynamicInfo.clkFault = tempClkFault;
                pDevObj->deviceEvents.faults |= VP_DEV_EVID_CLK_FLT;
            }
        }

        /* Get the current status of the first battery fault bit */
        tempBat1Fault = (pDevObj->intReg[0] & VP880_OCALMY_MASK) ? TRUE : FALSE;
        tempBat2Fault = (pDevObj->intReg[0] & VP880_OCALMZ_MASK) ? TRUE : FALSE;

        /* If line 1 is FXO, the Y supply is ignored */
        pLineCtx = pDevCtx->pLineCtx[0];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            if (!(pLineObj->status & VP880_IS_FXO)) {
                if(tempBat1Fault ^ pDevObj->dynamicInfo.bat1Fault) {
                    pDevObj->dynamicInfo.bat1Fault = tempBat1Fault;
                    pDevObj->deviceEvents.faults |= VP_DEV_EVID_BAT_FLT;
                }
            }
        }

        /* If line 2 is FXO, the Z supply is ignored */
        pLineCtx = pDevCtx->pLineCtx[1];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            if (!(pLineObj->status & VP880_IS_FXO)) {
                if(tempBat2Fault ^ pDevObj->dynamicInfo.bat2Fault) {
                    pDevObj->dynamicInfo.bat2Fault = tempBat2Fault;
                    pDevObj->deviceEvents.faults |= VP_DEV_EVID_BAT_FLT;
                }
            }
        }

        /*
         * Compare it with what we already know.  If different, generate
         * events and update the line status bits
         */
        intServCalled = TRUE;
        Vp880ServiceInterrupts(pDevCtx);

        /*
         * If level triggered, the interrupt may have been disabled (to prevent
         * a flood of interrupts), so reenable it.
         */
    #if defined (VP880_INTERRUPT_LEVTRIG_MODE)
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
    #if defined (VP880_EFFICIENT_POLLED_MODE)
        /* Poll the PIO-INT line */
        pDevObj->state |= (VpSysTestInt(deviceId) ? VP_DEV_PENDING_INT : 0x00);
    #elif defined (VP880_SIMPLE_POLLED_MODE)
        pDevObj->state |= VP_DEV_PENDING_INT;
    #endif
    }/* End while Interrupts*/

    /* Make sure Vp880ServiceInterrupts() is called at least once per tick to
     * keep the API line status up to date */
    if (intServCalled == FALSE) {
        Vp880ServiceInterrupts(pDevCtx);
    }

    /* Update the dial pulse handler for lines that are set for pulse decode */
    for (channelId = 0; channelId < maxChan; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;

            if (pLineObj->status & VP880_INIT_COMPLETE) {
                if (pLineObj->status & VP880_IS_FXO) {
#ifdef VP880_FXO_SUPPORT
                    Vp880ProcessFxoLine(pDevObj, pLineCtx);
#endif
                } else {
#ifdef VP880_FXS_SUPPORT
                    Vp880ProcessFxsLine(pDevObj, pLineCtx);
#endif
                }
            }
        }
    }

    /* Collect all event activity and report to the calling function */
    if (Vp880FindSoftwareInterrupts(pDevCtx)) {
        *pEventStatus = TRUE;
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}

/**
 * Vp880IsChnlUndrTst()
 *  This function determines if a particular line of a device is currently
 * running a test.
 *
 * Preconditions:
 *  None.
 *
 * Postconditions:
 *  Device not affected. Return value TRUE if the line is currently running a
 * test, FALSE otherwise.
 */
bool
Vp880IsChnlUndrTst(
    Vp880DeviceObjectType *pDevObj,
    uint8 channelId)
{
#ifdef VP880_INCLUDE_TESTLINE_CODE
    if (pDevObj->currentTest.nonIntrusiveTest == FALSE) {
        if ((TRUE == pDevObj->currentTest.prepared) &&
            (channelId == pDevObj->currentTest.channelId)) {
            return TRUE;
        }
    }
#endif
    return FALSE;
}

/**
 * Vp880ServiceInterrupts()
 *  This function should only be called by Vp880ApiTick when an interrupt
 * occurs.
 *
 * Preconditions:
 *  The device must first be initialized.
 *
 * Postconditions:
 *  The Global Signaling Register is read and the data is stored in the device
 * object.  Depending on the dial pulse mode option set, the hook event (on/off)
 * is generated if a hook status changed.  All FXO events are reported by this
 * function (i.e., no other processing necessary). This function will return
 * TRUE if an event has been generated.
 */
bool
Vp880ServiceInterrupts(
    VpDevCtxType *pDevCtx)
{
    bool retFlag = FALSE;

#ifdef VP880_FXS_SUPPORT
    VpLineCtxType *pLineCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp880LineObjectType *pLineObj;

    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal;

    uint8 channelId;
    VpCslacLineCondType tempHookSt, tempGnkSt, tempThermFault;
    VpLineStateType state;

    bool freezeGkey;
    bool ringTrip;

    uint8 maxChannels = pDevObj->staticInfo.maxChannels;

    for (channelId = 0; channelId < maxChannels; channelId++) {
        freezeGkey = FALSE;
        ringTrip = FALSE;

        pLineCtx = pDevCtx->pLineCtx[channelId];

        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;

            if (!(pLineObj->status & VP880_INIT_COMPLETE)) {
                continue;
            }

            ecVal = pLineObj->ecVal;
            state = pLineObj->lineState.currentState;

            if (!(pLineObj->status & VP880_IS_FXO)) {   /* Line Type is FXS */
                VpLineStateType usrState = pLineObj->lineState.usrCurrent;

                /*
                 * If debouncing for Ring Exit or Caller ID, ignore hook.
                 * Otherwise process.
                 */
                if (VpCSLACHookMaskEnabled(pLineObj->lineTimers.timers.timer)
                 || (pDevObj->devTimer[VP_DEV_TIMER_LP_CHANGE] & VP_ACTIVATE_TIMER)
                 || (pLineObj->lineState.calType != VP_CSLAC_CAL_NONE)
                 || (pDevObj->state & VP_DEV_IN_CAL)
#ifdef VP_CSLAC_SEQ_EN
                 || ((pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE)
                  && (pLineObj->intSequence[VP_PROFILE_TYPE_LSB] == VP_PRFWZ_PROFILE_FWD_DISC_INT))
                 || ((pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE)
                  && (pLineObj->intSequence[VP_PROFILE_TYPE_LSB] == VP_PRFWZ_PROFILE_TIP_OPEN_INT))
#endif  /* VP_CSLAC_SEQ_EN */
                 || ((state == VP_LINE_DISCONNECT))
                 || ((state == VP_LINE_TIP_OPEN))) {
                    tempHookSt = (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_HOOK);
                } else {
                    if (pLineObj->status & VP880_LOW_POWER_EN) {
                        if (pDevObj->intReg[channelId] & VP880_HOOK1_MASK) {
                            tempHookSt = VP_CSLAC_STATUS_INVALID;
                        } else {
                            tempHookSt = VP_CSLAC_HOOK;
                        }
                    } else {
                        if (pDevObj->intReg[channelId] & VP880_HOOK1_MASK) {
                            tempHookSt = VP_CSLAC_HOOK;
                        } else {
                            tempHookSt = VP_CSLAC_STATUS_INVALID;
                        }
                    }
                }

                if (pDevObj->intReg[channelId] & VP880_TEMPA1_MASK) {
                    tempThermFault = VP_CSLAC_THERM_FLT;
                } else {
                    tempThermFault = VP_CSLAC_STATUS_INVALID;
                }

                if ((pDevObj->devTimer[VP_DEV_TIMER_LP_CHANGE] & VP_ACTIVATE_TIMER)
                 || (pLineObj->lineState.calType != VP_CSLAC_CAL_NONE)
                 || (state == VP_LINE_DISCONNECT)
                 || (pLineObj->lineTimers.timers.timer[VP_LINE_DISCONNECT_EXIT]  & VP_ACTIVATE_TIMER)
                 || (pLineObj->lineTimers.timers.timer[VP_LINE_RING_EXIT_PROCESS] & VP_ACTIVATE_TIMER)
                 || (pLineObj->lineTimers.timers.timer[VP_LINE_GND_START_TIMER]  & VP_ACTIVATE_TIMER)) {
                    tempGnkSt = (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_GKEY);
                    freezeGkey = TRUE;
                } else {
                    if (pDevObj->intReg[channelId] & VP880_GNK1_MASK) {
                        tempGnkSt = VP_CSLAC_GKEY;
                    } else {
                        tempGnkSt = VP_CSLAC_STATUS_INVALID;
                    }
                }

                /*
                 * We "think" we know what Hook and Gkey are now, but it's
                 * possible the API-II is in the middle of the VoicePort Ground
                 * Start workaround. Check for the conditions where what is
                 * detected MUST be a Ground Key and not a Hook
                 */
                if ((!(pDevObj->devTimer[VP_DEV_TIMER_LP_CHANGE] & VP_ACTIVATE_TIMER))
                 && (freezeGkey == FALSE)) {
                    if ((state == VP_LINE_TIP_OPEN)
                     || (pLineObj->lineTimers.timers.timer[VP_LINE_GND_START_TIMER] & VP_ACTIVATE_TIMER)) {
                        uint8 currentHook = (pDevObj->intReg[channelId] & VP880_HOOK1_MASK);
                        tempGnkSt = (currentHook || tempGnkSt) ? VP_CSLAC_GKEY : VP_CSLAC_STATUS_INVALID;
                        tempHookSt = VP_CSLAC_STATUS_INVALID;
                    }
                }

                /* If the hook conditions changed, continue line processing */
                if((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_HOOK) != tempHookSt) {
                    pLineObj->lineState.condition &= ~VP_CSLAC_HOOK;
                    pLineObj->lineState.condition |= tempHookSt;

                    if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] > VP880_REV_VC) {
                        /*
                         * Read the test buffer IF it was not yet read this tick AND we're
                         * running Dial Pulse Detection AND we're not in LPM (i.e., if
                         * Dial Pulse Detection is not occurring).
                         */
                        if ((pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_ON) &&
                            (!(pDevObj->state & VP_DEV_TEST_BUFFER_READ)) &&
                            (!(pLineObj->status & VP880_LOW_POWER_EN))) {
                            VpMpiCmdWrapper(deviceId, ecVal, VP880_TX_PCM_BUFF_RD,
                                VP880_TX_PCM_BUFF_LEN, pDevObj->txBuffer);
                            pDevObj->state |= VP_DEV_TEST_BUFFER_READ;
                        }
                    }

                    /* Apply the hysteresis on the hook threshold (if available) */
                    if (pLineObj->hookHysteresis != 0) {
                        uint8 loopSupervision[VP880_LOOP_SUP_LEN];

                        VpMemCpy(loopSupervision, pLineObj->loopSup, VP880_LOOP_SUP_LEN);
                        if ((loopSupervision[VP880_LOOP_SUP_LIU_THRESH_BYTE]
                            & VP880_LOOP_SUP_LIU_THRESH_BITS) >= pLineObj->hookHysteresis) {
                            loopSupervision[VP880_LOOP_SUP_LIU_THRESH_BYTE] -=
                                pLineObj->hookHysteresis;
                        } else {
                            loopSupervision[VP880_LOOP_SUP_LIU_THRESH_BYTE] &=
                                ~VP880_LOOP_SUP_LIU_THRESH_BITS;
                        }
                        VpMpiCmdWrapper(deviceId, ecVal, VP880_LOOP_SUP_WRT,
                            VP880_LOOP_SUP_LEN, loopSupervision);
                    }

                    if ((pLineObj->status & VP880_LOW_POWER_EN) && tempHookSt
#ifdef VP880_INCLUDE_TESTLINE_CODE
                     && (pDevObj->currentTest.nonIntrusiveTest == FALSE)
#endif
                    ){
                        VP_HOOK(VpLineCtxType, pLineCtx,
                            ("Off-Hook Detected in Low Power Mode on line %d time %d UserState %d Current State %d Status 0x%04X",
                            channelId, pDevObj->timeStamp, pLineObj->lineState.usrCurrent, pLineObj->lineState.currentState, pLineObj->status));
                        if ((pLineObj->lineState.calType == VP_CSLAC_CAL_NONE)
                         && (Vp880IsChnlUndrTst(pDevObj, channelId) == FALSE)
                            ) {
                            /* Force line to feed state and start leaky line detection */
                            pLineObj->lineState.currentState = VP_LINE_OHT;
                            pDevObj->stateInt &= ~((channelId == 0) ? VP880_LINE0_LP : VP880_LINE1_LP);

                            pLineObj->lineState.condition |= VP_CSLAC_LINE_LEAK_TEST;
                        }
                        break;
                    }

#ifdef VP_CSLAC_SEQ_EN
                    /*
                     * There was a sufficient hook activity to stop the active
                     * CID -- unless the CID sequence knew this would happen and
                     * set the debounce flag. In which case, let CID continue.
                     */
                    if (pLineObj->callerId.status & VP_CID_IN_PROGRESS) {
                        if (pLineObj->callerId.status & VP_CID_IS_DEBOUNCE) {
                            /* Hook event is fully debounced and ready to go */
                            pLineObj->callerId.status &= ~VP_CID_IS_DEBOUNCE;
                        } else {
                            VpCliStopCli(pLineCtx);
                        }
                    }
#endif  /* VP_CSLAC_SEQ_EN */

                    if (tempHookSt == VP_CSLAC_HOOK) {
                        ringTrip = TRUE;
                        /* This function returns TRUE if an event is posted. */
                        if (Vp880OffHookMgmt(pDevObj, pLineCtx, ecVal) == TRUE) {
                            retFlag = TRUE;
                        }
                    } else {
                        if (Vp880OnHookMgmt(pDevObj, pLineCtx, ecVal) == TRUE) {
                            retFlag = TRUE;
                        }
                    }
                }

                /* If the gkey conditions changed, continue line processing */
                if((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_GKEY) != tempGnkSt) {
                    VP_HOOK(VpLineCtxType, pLineCtx, ("GKEY Change to %d on Ch %d Time %d",
                        tempGnkSt, channelId, pDevObj->timeStamp));

                    if (tempGnkSt == VP_CSLAC_GKEY) {
                        ringTrip = TRUE;
                        pLineObj->lineEvents.signaling |= VP_LINE_EVID_GKEY_DET;
                        pLineObj->lineState.condition |= VP_CSLAC_GKEY;
                    } else {
                        pLineObj->lineEvents.signaling |= VP_LINE_EVID_GKEY_REL;
                        pLineObj->lineState.condition &= ~(VP_CSLAC_GKEY);
                    }
                    retFlag = TRUE;
                    pLineObj->lineEventHandle = pDevObj->timeStamp;
                }

                /*
                 * Force to Ring Trip Exit state if off-hook or ground-key is
                 * detected while ringing EXCEPT in case running a line test.
                 */
                if ((ringTrip == TRUE)
                 && (Vp880IsChnlUndrTst(pDevObj, channelId) == FALSE)
                 && ((usrState == VP_LINE_RINGING) || (usrState == VP_LINE_RINGING_POLREV))) {
                    /*
                     * If ringtrip occurs (off-hook detected while ringing) AND we're exiting to a
                     * non-ringing state, debounce the hook bit and start the ringing exit process.
                     */
                    if ((pLineObj->ringCtrl.ringTripExitSt != VP_LINE_RINGING) &&
                        (pLineObj->ringCtrl.ringTripExitSt != VP_LINE_RINGING_POLREV)) {
                        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Forcing Ring Trip"));
                        Vp880SetLineState(pLineCtx, pLineObj->ringCtrl.ringTripExitSt);

                        /*
                         * Stop all currently running tones just in case tones were part of the
                         * ringing cadence. Not typical, but possible. If we don't do this and
                         * tones are part of the ringing cadence, then tones will remain enabled
                         * when a ring trip occurs.
                         */
                        Vp880SetLineTone(pLineCtx, VP_NULL, VP_NULL, VP_NULL);

                        /*
                         * This timer should be set in Vp880SetLineStateInt() called by
                         * Vp880SetLineState(). But just in case, make sure the ring exit time
                         * is set so that ringing will be removed and correctly hook debounce
                         * will be sest
                         */
                        pLineObj->lineTimers.timers.timer[VP_LINE_RING_EXIT_PROCESS] =
                            (1 | VP_ACTIVATE_TIMER);
                        pLineObj->lineTimers.timers.trackingTime = 0;
                    } else {
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("NO RING TRIP configured for Ch %d!!!!", pLineObj->channelId));
                        VP_LINE_STATE(VpLineCtxType, pLineCtx,
                            ("Ringing will continue until fault or stopped by the Application"));
                    }
                }

                if((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_THERM_FLT)
                    != tempThermFault) {
                    pLineObj->lineEventHandle = pDevObj->timeStamp;
                    pLineObj->lineState.condition &= ~(VP_CSLAC_THERM_FLT);
                    pLineObj->lineState.condition |= tempThermFault;

                    pLineObj->lineEvents.faults |= VP_LINE_EVID_THERM_FLT;
                    retFlag = TRUE;

                    if (tempThermFault == VP_CSLAC_THERM_FLT) {
#ifdef VP880_INCLUDE_TESTLINE_CODE
                        if((Vp880IsChnlUndrTst(pDevObj, channelId) == TRUE)
                          || (pDevObj->currentTest.nonIntrusiveTest == TRUE)) {
                            pLineObj->lineEvents.test |= VP_LINE_EVID_ABORT;
                        } else if (pDevObj->criticalFault.thermFltDiscEn == TRUE) {
#endif  /* VP880_INCLUDE_TESTLINE_CODE */
                            Vp880SetLineState(pLineCtx, VP_LINE_DISCONNECT);
#ifdef VP880_INCLUDE_TESTLINE_CODE
                        }
#endif  /* VP880_INCLUDE_TESTLINE_CODE */
                    }
                }
            }
        }
    }
#endif  /* VP880_FXS_SUPPORT */
    return retFlag;
}

/**
 * Vp880SetRelGain
 *  This function adjusts the GR and GX values for a given channel of a given
 * device.  It multiplies the profile values by a factor from 0.0 to 4.0.  The
 * adjustment factors are specified in the txLevel and rxLevel parameters,
 * which are 2.14 fixed-point numbers.
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
 */
#ifdef CSLAC_GAIN_RELATIVE
VpStatusType
Vp880SetRelGain(
    VpLineCtxType *pLineCtx,    /**< Line context to change gains on */
    uint16 txLevel,             /**< Adjustment to line's relative Tx level */
    uint16 rxLevel,             /**< Adjustment to line's relative Rx level */
    uint16 handle)              /**< Handle value returned with the event */
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp880DeviceObjectType *pDevObj = pLineCtx->pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpRelGainResultsType *relGainResults = &pDevObj->relGainResults;
    uint8 ecVal = pLineObj->ecVal;
    uint32 gxInt, grInt;
    uint8 gainCSD[VP880_GX_GAIN_LEN];

    uint8 mpiBuffer[2 + VP880_GX_GAIN_LEN + VP880_GR_GAIN_LEN] ;
    uint8 mpiIndex = 0;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetRelGain+"));

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetRelGain-"));
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /*
     * Do not proceed if the device calibration is in progress. This could
     * damage the device.
     */
    if (pDevObj->state & VP_DEV_IN_CAL) {
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetRelGain-"));
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    if (pDevObj->deviceEvents.response & VP880_READ_RESPONSE_MASK) {
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetRelGain-"));
        return VP_STATUS_DEVICE_BUSY;
    }

    relGainResults->gResult = VP_GAIN_SUCCESS;

    /*
     * Multiply the profile gain values by the requested adjustments.
     * Divide by 16,384 (0x4000) to scale down the input voltage
     * multiplication (where 0x4000 = 1) to +/-1.
     *
     * gxInt and grInt are the representative gain settings of GX/GR
     * in real voltage multiplication scales relative to 16,384.
     */
    gxInt = (uint32)pLineObj->gain.gxInt * txLevel / 16384L;
    grInt = (uint32)pLineObj->gain.grInt * rxLevel / 16384L;

    /*
     * If overflow or underflow occurred, generate out-of-range result.
     * Requirement: 1.0 <= gxInt <= 4.0. Meaning, do not allow an actual
     * voltge multiplier gx setting of <= 1.0 (0dB) or >= 4.0 (+12.04dB)
     */
    if ((gxInt < (uint32)0x4000) || (gxInt > (uint32)0x10000)) {
        VP_GAIN(VpLineCtxType, pLineCtx, ("Vp880SetRelGain(): %u * %cxLevel / 16384 = %u, %s is %u",
            (unsigned)pLineObj->gain.gxInt, 't', (unsigned)gxInt,
            (gxInt < (uint32)0x4000) ? "minimum" : "maximum",
            (gxInt < (uint32)0x4000) ? 0x4000U : 0x10000U));

        relGainResults->gResult |= VP_GAIN_GX_OOR;
        gxInt = pLineObj->gain.gxInt;
    }
    /*
     * If overflow or underflow occurred, generate out-of-range result.
     * Requirement: 0.25 <= grInt <= 1.0. Meaning, do not allow an actual
     * voltge multiplier gr setting of <= 0.25 (-12.04dB) or >= 1.0 (0dB)
     */
    if ((grInt < (uint32)0x1000) || (grInt > (uint32)0x4000)) {
        VP_GAIN(VpLineCtxType, pLineCtx, ("Vp880SetRelGain(): %u * %cxLevel / 16384 = %u, %s is %u",
            (unsigned)pLineObj->gain.grInt, 'r', (unsigned)grInt,
            (grInt < (uint32)0x1000) ? "minimum" : "maximum",
            (grInt < (uint32)0x1000) ? 0x1000U : 0x4000U));

        relGainResults->gResult |= VP_GAIN_GR_OOR;
        grInt = pLineObj->gain.grInt;
    }

    VP_GAIN(VpLineCtxType, pLineCtx, ("Vp880SetRelGain(): %u * %cxLevel / 16384 = %u",
        (unsigned)pLineObj->gain.gxInt, 't', (unsigned)gxInt));

    VP_GAIN(VpLineCtxType, pLineCtx, ("Vp880SetRelGain(): %u * %cxLevel / 16384 = %u",
        (unsigned)pLineObj->gain.grInt, 'r', (unsigned)grInt));

    /*
     * Write adjusted gain values to the device, and remember them for
     * VpGetResults().
     */
    VpConvertFixed2Csd((uint16)(gxInt - 0x4000), gainCSD);
    relGainResults->gxValue = ((uint16)gainCSD[0] << 8) + gainCSD[1];
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_GX_GAIN_WRT,
        VP880_GX_GAIN_LEN, gainCSD);
    VP_GAIN(VpLineCtxType, pLineCtx,
            ("Post Converted gxInt (write to GX_WRT): gainCSD = 0x%02X 0x%02X",
             gainCSD[0], gainCSD[1]));

    VpConvertFixed2Csd((uint16)grInt, gainCSD);
    relGainResults->grValue = ((uint16)gainCSD[0] << 8) + gainCSD[1];
    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_GR_GAIN_WRT,
        VP880_GR_GAIN_LEN, gainCSD);
    VP_GAIN(VpLineCtxType, pLineCtx,
            ("Post Converted grInt (write to GR_WRT): gainCSD = 0x%02X 0x%02X",
             gainCSD[0], gainCSD[1]));

    /* send down the mpi commands */
    VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);

    /* Generate the gain-complete event. */
    pLineObj->lineEvents.response |= VP_LINE_EVID_GAIN_CMP;
    pLineObj->lineEventHandle = handle;

#ifdef CSLAC_GAIN_ABS
    /* Absolute gain values are now unknown */
    pLineObj->gain.absGxGain = VP_ABS_GAIN_UNKNOWN;
    pLineObj->gain.absGrGain = VP_ABS_GAIN_UNKNOWN;
#endif


    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetRelGain-"));

    return VP_STATUS_SUCCESS;
}
#endif

/**
 * Vp880MuteChannel()
 *  This function disables or enables the PCM highway for the selected line and
 * should only be called by API internal functions.
 *
 * Preconditions:
 *  The line context must be valid (i.e., pointing to a valid Vp880 line object
 * type).
 *
 * Postconditions:
 *  If mode is TRUE the TX/RX path is cut. If FALSE, the TX/RX path is enabled
 * according to the current line state and mode used for talk states.
 */
void
Vp880MuteChannel(
    VpLineCtxType *pLineCtx,    /**< Line affected */
    bool mode)                  /**< TRUE = Disable TX/RX, FALSE = enable */
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;

    uint8 ecVal = pLineObj->ecVal;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 postState;
    uint8 mpiByte = 0;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880MuteChannel+"));

    /*
     * Read the status of the Operating Conditions register so we can change
     * only the TX and RX if the line state is a non-communication mode.
     */
    postState = pLineObj->opCond[0];
    postState &= (uint8)(~(VP880_CUT_TXPATH | VP880_CUT_RXPATH));
    postState &= (uint8)(~(VP880_HIGH_PASS_DIS | VP880_OPCOND_RSVD_MASK));

    /*
     * If disabling, simple. Otherwise enable based on the current line state
     * and the state of the "talk" option. The "talk" option is maintained in
     * the line object and abstracted in Vp880GetTxRxMode() function
     */

    Vp880GetTxRxPcmMode(pLineObj, pLineObj->lineState.currentState, &mpiByte);

    if (mode == TRUE) {
        /*
         * If awaiting DTMF detection, enable TX, disable RX. This is higher
         * priority than Mute mode. Otherwise, disable both TX and RX.
         */
        postState |= VP880_CUT_RXPATH;  /* Mute == TRUE always cuts RX path */
#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
        if (!(pLineObj->callerId.status & VP_CID_AWAIT_TONE)) {
#endif
            /* Not awaiting tone, TX Path is disabled as well */
            postState |= VP880_CUT_TXPATH;
#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
        }
#endif
    } else {
        /*
         * It's possible that a Mute off is occuring because of end of DTMF
         * detection, or end of data generation, or end of Mute period. However,
         * we only need to check if Mute On is still enabled since DTMF
         * detection will not occur while data is being generated.
         */
#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
        if (pLineObj->callerId.status & VP_CID_MUTE_ON) {
            /*
             * Some "other" operation completed, but we're still in a Mute On
             * period.
             */
            postState |= (VP880_CUT_RXPATH | VP880_CUT_TXPATH);
        } else  {
#endif
            postState |= mpiByte;
#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
        }
#endif
    }

    if (postState != pLineObj->opCond[0]) {
        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("3. Writing 0x%02X to Operating Conditions",
            postState));
        pLineObj->opCond[0] = postState;
        VpMpiCmdWrapper(deviceId, ecVal, VP880_OP_COND_WRT, VP880_OP_COND_LEN,
            pLineObj->opCond);
    }
    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880MuteChannel-"));

    return;
}

/**
 * Vp880GetTxRxPcmMode()
 *  This function returns the TX/RX PCM bits for the PCM (enable/disable) mode
 * corresponding to the state passed. The results should be or'-ed with the
 * bits set to 0 prior to calling this function.
 *
 * Preconditions:
 *  None. Mapping function only.
 *
 * Postconditions:
 *  None. Mapping function only.
 */
VpStatusType
Vp880GetTxRxPcmMode(
    Vp880LineObjectType *pLineObj,
    VpLineStateType state,  /**< The state associating with PCM mode */
    uint8 *mpiByte) /**< Device Specific byte */
{
    VP_API_FUNC_INT(None, VP_NULL, ("Vp880GetTxRxPcmMode+"));

    switch(pLineObj->pcmTxRxCtrl) {
        case VP_OPTION_PCM_BOTH:
            *mpiByte = 0x00;
            break;

        case VP_OPTION_PCM_RX_ONLY:
            *mpiByte = VP880_CUT_TXPATH;
            break;

        case VP_OPTION_PCM_TX_ONLY:
            *mpiByte = VP880_CUT_RXPATH;
            break;

        case VP_OPTION_PCM_ALWAYS_ON:
            *mpiByte = 0x00;
            return VP_STATUS_SUCCESS;

        case VP_OPTION_PCM_OFF:
            *mpiByte = (VP880_CUT_TXPATH | VP880_CUT_RXPATH);
            return VP_STATUS_SUCCESS;

        default:
            *mpiByte = 0x00;
            break;
    }

    switch(state) {
        /* Non-Talk States */
        case VP_LINE_STANDBY:
        case VP_LINE_STANDBY_POLREV:
        case VP_LINE_TIP_OPEN:
        case VP_LINE_ACTIVE:
        case VP_LINE_ACTIVE_POLREV:
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
        case VP_LINE_HOWLER:
        case VP_LINE_HOWLER_POLREV:
#endif
        case VP_LINE_DISCONNECT:
        case VP_LINE_RINGING:
        case VP_LINE_RINGING_POLREV:
            if (pLineObj->status & VP880_IS_FXO) {
                VP_API_FUNC_INT(None, VP_NULL, ("Vp880GetTxRxPcmMode-"));
                return VP_STATUS_INVALID_ARG;
            }
            *mpiByte |= (VP880_CUT_TXPATH | VP880_CUT_RXPATH);
            break;

        case VP_LINE_FXO_LOOP_OPEN:
        case VP_LINE_FXO_LOOP_CLOSE:
        case VP_LINE_FXO_RING_GND:
            if (!(pLineObj->status & VP880_IS_FXO)) {
                VP_API_FUNC_INT(None, VP_NULL, ("Vp880GetTxRxPcmMode-"));
                return VP_STATUS_INVALID_ARG;
            }
            *mpiByte |= (VP880_CUT_TXPATH | VP880_CUT_RXPATH);
            break;

        /* Talk States */
        case VP_LINE_TALK:
        case VP_LINE_TALK_POLREV:
        case VP_LINE_OHT:
        case VP_LINE_OHT_POLREV:
            if (pLineObj->status & VP880_IS_FXO) {
                VP_API_FUNC_INT(None, VP_NULL, ("Vp880GetTxRxPcmMode-"));
                return VP_STATUS_INVALID_ARG;
            }
            break;

        case VP_LINE_FXO_OHT:
        case VP_LINE_FXO_TALK:
            if (!(pLineObj->status & VP880_IS_FXO)) {
                VP_API_FUNC_INT(None, VP_NULL, ("Vp880GetTxRxPcmMode-"));
                return VP_STATUS_INVALID_ARG;
            }
            break;

        default:
            break;
    }
    VP_API_FUNC_INT(None, VP_NULL, ("Vp880GetTxRxPcmMode-"));

    return VP_STATUS_SUCCESS;
}

/**
 * Vp880SetLineTone()
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
 */
VpStatusType
Vp880SetLineTone(
    VpLineCtxType *pLineCtx,
    VpProfilePtrType pToneProfile,  /**< A pointer to a tone profile, or an
                                     * index into the profile table for the tone
                                     * to put on the line.
                                     */
    VpProfilePtrType pCadProfile,   /**< A pointer to a tone cadence profile, or
                                     * an index into the profile table for the
                                     * tone cadence to put on the line.
                                     */
    VpDtmfToneGenType *pDtmfControl)    /**< Indicates to send a DTMF tone
                                         * (either upstream or downstream) if
                                         * this parameter is not VP_NULL AND
                                         * the tone specified is VP_PTABLE_NULL
                                         */
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpProfilePtrType pToneProf = VP_PTABLE_NULL;

#ifdef VP_CSLAC_SEQ_EN
    VpProfilePtrType pCadProf = VP_PTABLE_NULL;
#endif

    VpDigitType digit = VP_DIG_NONE;
    VpDirectionType direction = VP_DIRECTION_INVALID;

    uint8 ecVal = pLineObj->ecVal;

    uint8 sigGenCtrl, mpiIndex = 0;
    uint8 mpiByte = 0;

    uint8 mpiBuffer[2 + VP880_SIGA_PARAMS_LEN + VP880_SIGCD_PARAMS_LEN];

    /* Initialize SigGen A/B values to 0 */
    uint8 sigGenAB[VP880_SIGA_PARAMS_LEN] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };

    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 opCondTarget = pLineObj->opCond[0];

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineTone+"));

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineTone-"));
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /*
     * Do not proceed if the device calibration is in progress. This could
     * damage the device.
     */
    if (pDevObj->state & VP_DEV_IN_CAL) {
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineTone-"));
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Check the legality of the Tone profile */
    if (!VpCSLACIsProfileValid(VP_PROFILE_TONE, VP_CSLAC_TONE_PROF_TABLE_SIZE,
        pDevObj->profEntry.toneProfEntry,
        pDevObj->devProfileTable.pToneProfileTable, pToneProfile, &pToneProf)) {
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineTone-"));
        return VP_STATUS_ERR_PROFILE;
    }

    /* Verify a good profile (index or pointer) for the cadence */
#ifdef VP_CSLAC_SEQ_EN
    /* Check the legality of the Tone Cadence profile */
    if (!VpCSLACIsProfileValid(VP_PROFILE_TONECAD, VP_CSLAC_TONE_CADENCE_PROF_TABLE_SIZE,
        pDevObj->profEntry.toneCadProfEntry,
        pDevObj->devProfileTable.pToneCadProfileTable, pCadProfile, &pCadProf)) {
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineTone-"));
        return VP_STATUS_ERR_PROFILE;
    }
#endif

    if (pDtmfControl != VP_NULL) {
        digit = pDtmfControl->toneId;
        if (VpIsDigit(digit) == FALSE) {
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineTone-"));
            return VP_STATUS_INVALID_ARG;
        }

        direction = pDtmfControl->dir;
        if (direction != VP_DIRECTION_DS) {
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineTone-"));
            return VP_STATUS_INVALID_ARG;
        }
    }

    /* All input parameters are valid. */
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    if (pLineObj->status & VP880_BAD_LOOP_SUP) {
        pLineObj->status &= ~(VP880_BAD_LOOP_SUP);
        VpMpiCmdWrapper(deviceId, ecVal, VP880_LOOP_SUP_WRT,
            VP880_LOOP_SUP_LEN, pLineObj->loopSup);
    }

    /*
     * Disable signal generator A/B/C/D before making any changes and stop
     * previous cadences
     */
    sigGenCtrl = 0;
    if (sigGenCtrl != pLineObj->sigGenCtrl[0]) {
        pLineObj->sigGenCtrl[0] = sigGenCtrl;
        VpMpiCmdWrapper(deviceId, ecVal, VP880_GEN_CTRL_WRT, VP880_GEN_CTRL_LEN,
            pLineObj->sigGenCtrl);
    }

#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
    /*
     * Force the "tone type" (used to indicate which special howler tone is currently being used)
     * back to no tone so when we set the line to High Gain Mode at some point later it won't use
     * the wrong set of coefficients.
     */
    pLineObj->cadence.toneType = 0;

    if (!(pLineObj->callerId.status & VP_CID_IN_PROGRESS)) {
#endif

#ifdef VP_CSLAC_SEQ_EN
        pLineObj->cadence.pActiveCadence = pCadProf;
        pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;

        /* We're no longer in the middle of a time function */
        pLineObj->cadence.status &= ~VP_CADENCE_STATUS_MID_TIMER;
        pLineObj->cadence.timeRemain = 0;
#endif

#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
    }
#endif

    /*
     * If tone profile is NULL, and either the pDtmfControl is NULL or it's
     * "digit" member is "Digit None", then shutoff the tone generators, stop
     * any active cadencing and restore the filter coefficients if they need
     * to be. Also, re-enable the audio path if it was disabled by a previous
     * DTMF generation command
     */
    if ((pToneProf == VP_PTABLE_NULL)
     && ((pDtmfControl == VP_NULL) || (digit == VP_DIG_NONE))) {
        /*
         * Update the TX/RX Path enable/disable ONLY if not running CID. The CID
         * sequence itself manages TX/RX path control
         */
#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
        if (!(pLineObj->callerId.status & VP_CID_IN_PROGRESS)) {
#endif
            /*
             * Pre-Or the bits and get the correct values based on the current
             * line state, then update the device.
             */
            opCondTarget &= (uint8)(~(VP880_HIGH_PASS_DIS | VP880_OPCOND_RSVD_MASK));
            opCondTarget &= (uint8)(~(VP880_CUT_TXPATH | VP880_CUT_RXPATH));
            Vp880GetTxRxPcmMode(pLineObj, pLineObj->lineState.currentState, &mpiByte);
            opCondTarget |= mpiByte;
            if (opCondTarget != pLineObj->opCond[0]) {
                pLineObj->opCond[0] = opCondTarget;
                VP_LINE_STATE(VpLineCtxType, pLineCtx, ("4. Writing 0x%02X to Operating Conditions",
                    pLineObj->opCond[0]));
                VpMpiCmdWrapper(deviceId, ecVal, VP880_OP_COND_WRT,
                    VP880_OP_COND_LEN, pLineObj->opCond);
            }
#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
        }
#endif

        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineTone-"));
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
        Vp880SetDTMFGenerators(pLineCtx, VP_CID_NO_CHANGE, digit);

        /*
         * Disable only the receive path since disabling the transmit path
         * also may generate noise upstream (e.g., an unterminated, but
         * assigned timeslot
         */
        opCondTarget &= (uint8)(~(VP880_HIGH_PASS_DIS | VP880_OPCOND_RSVD_MASK));
        opCondTarget |= VP880_CUT_RXPATH;
        if (opCondTarget != pLineObj->opCond[0]) {
            pLineObj->opCond[0] = opCondTarget;
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("5. Writing 0x%02X to Operating Conditions",
                pLineObj->opCond[0]));
            VpMpiCmdWrapper(deviceId, ecVal, VP880_OP_COND_WRT, VP880_OP_COND_LEN,
                pLineObj->opCond);
        }

        /* Enable only generator A/B */
        sigGenCtrl = (VP880_GENB_EN | VP880_GENA_EN);
        if (sigGenCtrl != pLineObj->sigGenCtrl[0]) {
            pLineObj->sigGenCtrl[0] = sigGenCtrl;
            VpMpiCmdWrapper(deviceId, ecVal, VP880_GEN_CTRL_WRT, VP880_GEN_CTRL_LEN,
                pLineObj->sigGenCtrl);
        }

        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineTone-"));
        return VP_STATUS_SUCCESS;
    }

#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
    /* If we're here, we're sending a Tone, not DTMF */
    if ((pCadProf != VP_PTABLE_NULL)
     && (((pCadProf[VP_CSLAC_TONE_TYPE] & VP_CSLAC_SPECIAL_TONE_MASK) == VP_CSLAC_UK_HOWLER_TONE_VER15)
      || ((pCadProf[VP_CSLAC_TONE_TYPE] & VP_CSLAC_SPECIAL_TONE_MASK) == VP_CSLAC_UK_HOWLER_TONE_DRAFT_G)
      || ((pCadProf[VP_CSLAC_TONE_TYPE] & VP_CSLAC_SPECIAL_TONE_MASK) == VP_CSLAC_AUS_HOWLER_TONE)
      || ((pCadProf[VP_CSLAC_TONE_TYPE] & VP_CSLAC_SPECIAL_TONE_MASK) == VP_CSLAC_NTT_HOWLER_TONE))) {

        uint8 sigGenCD[VP880_SIGCD_PARAMS_LEN] = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        };

        /* Return ERROR if the Special Howler Init function doesn't know what this Tone Type is */
        pLineObj->cadence.toneType = (pCadProf[VP_CSLAC_TONE_TYPE] & VP_CSLAC_SPECIAL_TONE_MASK);
        if (!(VpCSLACHowlerInit(&pLineObj->cadence, pDevObj->devProfileData.tickRate))) {
            pLineObj->cadence.toneType = VP_CSLAC_STD_TONE;
            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineTone-"));
            return VP_STATUS_INVALID_ARG;
        }

        sigGenAB[3] = ((pLineObj->cadence.startFreq >> 8) & 0xFF);
        sigGenAB[4] = (pLineObj->cadence.startFreq & 0xFF);

        sigGenAB[5] = ((pLineObj->cadence.startLevel >> 8) & 0xFF);
        sigGenAB[6] = (pLineObj->cadence.startLevel & 0xFF);

        /* Make sure C/D are cleared */
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SIGCD_PARAMS_WRT,
            VP880_SIGCD_PARAMS_LEN, sigGenCD);

        /* Program A/B */
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SIGA_PARAMS_WRT,
            VP880_SIGA_PARAMS_LEN, sigGenAB);
        /* Clear flag to indicate the generators are NOT in a Ringing Mode */
        pLineObj->status &= ~(VP880_RING_GEN_NORM | VP880_RING_GEN_REV);

        VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);

        /*
         * Set the parameters in the line object for cadence use. The Cadence operations use
         * the cached values when determining frequency/level adjustments.
         */
        VpMemCpy(pLineObj->cadence.regData, sigGenAB, VP880_SIGA_PARAMS_LEN);

#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
        /* Update the Filter Coefficients if in High Gain Mode */
        if (pLineObj->howlerModeCache.isInHowlerMode) {
            VpCLSACHighGainMode(pLineCtx, TRUE);
        }
#endif

        VP_SEQUENCER(VpLineCtxType, pLineCtx, ("Ramp started at time %d", pDevObj->timeStamp));
    } else {
#endif
        /*
         * Send the signal generator parameters to the device and enable the
         * Tone Generators -- add in the first 3 bytes (all 0x00)
         */
        VpMemCpy(&sigGenAB[VP880_SIGAB_FREQ_START], &pToneProf[VP880_SIGGEN_AB_START],
            (VP880_SIGA_PARAMS_LEN - VP880_SIGAB_FREQ_START));

        mpiIndex = 0;
        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SIGA_PARAMS_WRT,
            VP880_SIGA_PARAMS_LEN, sigGenAB);
        /* Clear flag to indicate the generators are NOT in a Ringing Mode */
        pLineObj->status &= ~(VP880_RING_GEN_NORM | VP880_RING_GEN_REV);

        mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_SIGCD_PARAMS_WRT,
            VP880_SIGCD_PARAMS_LEN, (uint8 *)(&pToneProf[VP880_SIGGEN_CD_START]));
        VpMpiCmdWrapper(deviceId, pLineObj->ecVal, mpiBuffer[0], mpiIndex-1,
            &mpiBuffer[1]);

#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
    }
#endif /* VP_CSLAC_SEQ_EN && VP880_FXS_SUPPORT */

#ifdef VP_CSLAC_SEQ_EN
    if (pCadProf == VP_PTABLE_NULL) {
        /*
         * If a tone is being actived due to caller ID, then do not stop the
         * cadencer
         */
#ifdef VP880_FXS_SUPPORT
        if (!(pLineObj->callerId.status & VP_CID_IN_PROGRESS)) {
#endif /* VP880_FXS_SUPPORT */
            pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;
            pLineObj->cadence.index = VP_PROFILE_TYPE_SEQUENCER_START;
#ifdef VP880_FXS_SUPPORT
        }
#endif /* VP880_FXS_SUPPORT */
#endif /* VP_CSLAC_SEQ_EN */
        sigGenCtrl = VP880_GEN_ALLON;
        if (sigGenCtrl != pLineObj->sigGenCtrl[0]) {
            pLineObj->sigGenCtrl[0] = sigGenCtrl;
            VpMpiCmdWrapper(deviceId, ecVal, VP880_GEN_CTRL_WRT, VP880_GEN_CTRL_LEN,
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

        /* Nullify any internal sequence so that the API doesn't think
         * that an internal sequence of some sort is running */
        pLineObj->intSequence[VP_PROFILE_TYPE_LSB] = VP_PRFWZ_PROFILE_NONE;
    }
#endif /* VP_CSLAC_SEQ_EN */

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetLineTone-"));
    return VP_STATUS_SUCCESS;
}

/**
 * Vp880SetDTMFGenerators()
 *  This function sets signal generator A/B for DTMF tone generation.
 *
 * Preconditions:
 *  The line must first be initialized.
 *
 * Postconditions:
 *  The signal generators A/B are set to the DTMF frequencies and level required
 * by the digit passed.
 */
VpStatusType
Vp880SetDTMFGenerators(
    VpLineCtxType *pLineCtx,
    VpCidGeneratorControlType mode,
    VpDigitType digit)
{
    VpStatusType status;
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    uint8 ecVal = pLineObj->ecVal;
    uint8 sigGenCtrl[VP880_GEN_CTRL_LEN] = {VP880_GEN_ALLOFF};
    VpDeviceIdType deviceId = pDevObj->deviceId;

#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
    uint8 sigByteCount;
    uint8 sigOffset = VP_CID_PROFILE_FSK_PARAM_LEN + 2;
#endif

    uint8 sigGenABParams[] = {
        0x00, 0x00, 0x00,  /* RSVD */
        0x00, 0x00, /* Replace with required column Frequency */
        0x1C, 0x32, /* Level = -10dBm */
        0x00, 0x00, /* Replace with required row Frequency */
        0x1C, 0x32  /* Level = -10dBm */
    };

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetDTMFGenerators+"));

#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
    /*
     * If we're generating caller ID data set the levels based on the data in
     * the CID profile
     */
    if ((pLineObj->callerId.status & VP_CID_IN_PROGRESS) &&
        (pLineObj->callerId.pCliProfile != VP_PTABLE_NULL)) {
        for (sigByteCount = 0; sigByteCount < (VP880_SIGA_PARAMS_LEN - 3);
             sigByteCount++) {
            sigGenABParams[sigByteCount+3] =
                pLineObj->callerId.pCliProfile[sigOffset + sigByteCount];
        }
    } else {
#endif
        /*
         * If it's an FXO line then the DTMF high and low frequency levels are
         * specified in the FXO/Dialing Profile, cached in the line object.
         */
#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXO_SUPPORT)
        if (pLineObj->status & VP880_IS_FXO) {
            sigGenABParams[5] = pLineObj->digitGenStruct.dtmfHighFreqLevel[0];
            sigGenABParams[6] = pLineObj->digitGenStruct.dtmfHighFreqLevel[1];
            sigGenABParams[9] = pLineObj->digitGenStruct.dtmfLowFreqLevel[0];
            sigGenABParams[10] = pLineObj->digitGenStruct.dtmfLowFreqLevel[1];
        }
#endif

#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
    }
#endif
    /*
     * Modify values sigGenABParams[3][4] and [7][8] with values required for the DTMF Frequencies
     * using common VE880/890 computations
     */
    status = VpCSLACSetDTMFGenValues(&sigGenABParams[3], digit);
    if (status != VP_STATUS_SUCCESS) {
        return status;
    }

    VpMpiCmdWrapper(deviceId, ecVal, VP880_SIGA_PARAMS_WRT,
        VP880_SIGA_PARAMS_LEN, sigGenABParams);
    /* Clear flag to indicate the generators are NOT in a Ringing Mode */
    pLineObj->status &= ~(VP880_RING_GEN_NORM | VP880_RING_GEN_REV);

    /*
     * If there is no change to generator control required, it is assumed to be
     * set properly prior to this function call.
     */
    if (mode != VP_CID_NO_CHANGE) {
        /*
         * For DTMF CID, the data passed may be message data, a keyed character
         * (e.g., Mark, Channel Seizure), or End of Transmission. If it's End
         * of Transmission, disable the DTMF generators immediately. Otherwise,
         * enable the DTMF generators
         */
#if defined (VP_CSLAC_SEQ_EN) && defined (VP880_FXS_SUPPORT)
        if ((mode == VP_CID_GENERATOR_DATA)
         || (mode == VP_CID_GENERATOR_KEYED_CHAR)) {
            sigGenCtrl[0] |= (VP880_GENA_EN | VP880_GENB_EN);

            /* Start this line timer to stop after TIME >= DTMF CID On-Time is met. This value
             * should be at least 70ms, giving DTMF detectors sufficient time to detect CID DTMF
             * signal sent. Note that "MS Roundup" is used instead of "MS To Tickrate" in order to
             * make sure the minimum time is provided. This is a case where accuracy is not as
             * critical as providing the minimum duration.
             */
            pLineObj->lineTimers.timers.timer[VP_LINE_TIMER_CID_DTMF] =
                (MS_TO_TICKS_ROUND_UP(VP_CID_DTMF_ON_TIME, pDevObj->devProfileData.tickRate))
                | VP_ACTIVATE_TIMER;

            pLineObj->callerId.dtmfStatus |= VP_CID_ACTIVE_ON_TIME;
        }
#endif

        if (sigGenCtrl[0] != pLineObj->sigGenCtrl[0]) {
            pLineObj->sigGenCtrl[0] = sigGenCtrl[0];
            VpMpiCmdWrapper(deviceId, ecVal, VP880_GEN_CTRL_WRT, VP880_GEN_CTRL_LEN,
                pLineObj->sigGenCtrl);
        }
    }
    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetDTMFGenerators-"));
    return VP_STATUS_SUCCESS;
}

/**
 * Vp880SetOption()
 *  This function determines how to process the Option based on pDevCtx,
 * pLineCtx, and option type.  The actual options are implemented in
 * Vp880SetOptionInternal
 *
 * Preconditions:
 *  The line must first be initialized if a line context is passed, or the
 * device must be initialized if a device context is passed.
 *
 * Postconditions:
 *  The option specified is implemented either on the line, or on the device, or
 * on all lines associated with the device (see the API Reference Guide for
 * details).
 */
VpStatusType
Vp880SetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *value)
{
    uint8 channelId;
    Vp880DeviceObjectType *pDevObj;
    VpStatusType status = VP_STATUS_INVALID_ARG;

    VpDevCtxType *pDevCtxLocal;
    VpLineCtxType *pLineCtxLocal;
    Vp880LineObjectType *pLineObj;
    VpDeviceIdType deviceId;
    bool onlyFXO = TRUE;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetOption+"));

    if (pDevCtx != VP_NULL) {
        pDevObj = pDevCtx->pDevObj;
        deviceId = pDevObj->deviceId;

        if (option != VP_OPTION_ID_DEBUG_SELECT) {
            /* Proceed if device state is either in progress or complete */
            if (pDevObj->state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
            } else {
                VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetOption-"));
                return VP_STATUS_DEV_NOT_INITIALIZED;
            }

            /*
             * Do not proceed if the device calibration is in progress. This could
             * damage the device.
             */
            if (pDevObj->state & VP_DEV_IN_CAL) {
                VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetOption-"));
                return VP_STATUS_DEV_NOT_INITIALIZED;
            }
        }

        VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

        /*
         * Valid Device Context, we already know Line context is NULL (higher
         * layer SW, process on device if device option, or process on all lines
         * associated with device if line option
         */
        switch (option) {
            case VP_OPTION_ID_EVENT_MASK:  /* Line and Device */
                Vp880SetOptionInternal(VP_NULL, pDevCtx, option, value);

            /* Line Options */
            case VP_OPTION_ID_ZERO_CROSS:
            case VP_OPTION_ID_PULSE_MODE:
            case VP_OPTION_ID_TIMESLOT:
            case VP_OPTION_ID_CODEC:
            case VP_OPTION_ID_PCM_HWY:
            case VP_OPTION_ID_LOOPBACK:
            case VP_OPTION_ID_LINE_STATE:
            case VP_OPTION_ID_RING_CNTRL:
            case VP_OPTION_ID_PCM_TXRX_CNTRL:
            case VP_OPTION_ID_RINGING_PARAMS:
            case VP_OPTION_ID_DCFEED_PARAMS:
#ifdef CSLAC_GAIN_ABS
            case VP_OPTION_ID_ABS_GAIN:
#endif
                /*
                 * Loop through all of the valid channels associated with this
                 * device. Init status variable in case there are currently no
                 * line contexts associated with this device
                 */
                status = VP_STATUS_SUCCESS;
                for (channelId = 0; channelId < pDevObj->staticInfo.maxChannels; channelId++) {
                    pLineCtxLocal = pDevCtx->pLineCtx[channelId];

                    if (pLineCtxLocal == VP_NULL) {
                        continue;
                    }

                    if ((option == VP_OPTION_ID_ZERO_CROSS) ||
                        (option == VP_OPTION_ID_PULSE_MODE) ||
                        (option == VP_OPTION_ID_LINE_STATE) ||
                        (option == VP_OPTION_ID_RING_CNTRL)){
                        uint8 lastChannel = (pDevObj->staticInfo.maxChannels - 1);

                        pLineObj = pLineCtxLocal->pLineObj;

                        /* This device has at least 1 FXS, SetOption will succeed */
                        if (!(pLineObj->status & VP880_IS_FXO)) {
                            onlyFXO = FALSE;
                            status = Vp880SetOptionInternal(pLineCtxLocal, VP_NULL, option, value);
                        /* Only FXO on this device */
                        } else if ((onlyFXO == TRUE) && (channelId == lastChannel)) {
                            status = VP_STATUS_OPTION_NOT_SUPPORTED;
                        /* Just bailout in case there is at least 1 FXS on this device */
                        } else {
                            break;
                        }
                    } else {
                        status = Vp880SetOptionInternal(pLineCtxLocal, VP_NULL, option, value);
                    }

                    if (VP_STATUS_SUCCESS != status) {
                        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                        VP_API_FUNC_INT(VpLineCtxType, pLineCtxLocal, ("Vp880SetOption-"));
                        return status;
                    }
                }
                break;
            default:
                /*
                 * Device option, or option unknown option.  Handle in lower
                 * layer
                 */
                status = Vp880SetOptionInternal(VP_NULL, pDevCtx, option, value);
                break;
        }
    } else {
        /*
         * Line context must be valid, device context is NULL, proceed as
         * normal
         */
        pDevCtxLocal = pLineCtx->pDevCtx;
        pDevObj = pDevCtxLocal->pDevObj;
        deviceId = pDevObj->deviceId;
        if (option != VP_OPTION_ID_DEBUG_SELECT) {
            /* Proceed if device state is either in progress or complete */
            if (pDevObj->state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
            } else {
                VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetOption-"));
                return VP_STATUS_DEV_NOT_INITIALIZED;
            }

            /*
             * Do not proceed if the device calibration is in progress. This could
             * damage the device.
             */
            if (pDevObj->state & VP_DEV_IN_CAL) {
                VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetOption-"));
                return VP_STATUS_DEV_NOT_INITIALIZED;
            }
        }

        VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
        status = Vp880SetOptionInternal(pLineCtx, VP_NULL, option, value);
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetOption-"));
    return status;
}

/**
 * Vp880SetOptionInternal()
 *  This function implements on the Vp880 device the options specified from
 * Vp880SetOption().  No other function should call this function.
 *
 * Preconditions:
 *  See Vp880SetOption()
 *
 * Postconditions:
 *  See Vp880SetOption()
 */
VpStatusType
Vp880SetOptionInternal(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *value)
{
    VpDevCtxType *pDevCtxLocal;
    VpLineCtxType *pLineCtxLocal;

    VpStatusType status = VP_STATUS_SUCCESS;

    Vp880LineObjectType *pLineObj;
    Vp880DeviceObjectType *pDevObj;
    uint8 tempData[VP880_INT_MASK_LEN], channelId, txSlot, rxSlot;

    VpDeviceIdType deviceId;

    VpOptionDeviceIoType deviceIo;

    uint8 maxChan;
    uint8 mpiByte = 0;
    uint8 ioDirection[2] = {0x00, 0x00};
#ifdef VP880_FXS_SUPPORT
    uint8 tempSysConfig[VP880_SS_CONFIG_LEN];
#endif
    uint8 ecVal;

    VpOptionEventMaskType *pEventsMask, *pNewEventsMask;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetOptionInternal+"));

    if (pLineCtx != VP_NULL) {
        pDevCtxLocal = pLineCtx->pDevCtx;
        pDevObj = pDevCtxLocal->pDevObj;
        deviceId = pDevObj->deviceId;
        pLineObj = pLineCtx->pLineObj;
        channelId = pLineObj->channelId;
        ecVal = pLineObj->ecVal;

        switch (option) {
            /* Line Options */
#ifdef CSLAC_GAIN_ABS
            case VP_OPTION_ID_ABS_GAIN:
                status = VpCSLACSetAbsGain(pLineCtx, ((VpOptionAbsGainType *)value));
                break;
#endif

#ifdef VP880_FXS_SUPPORT
            case VP_OPTION_ID_PULSE_MODE:
                if (pLineObj->status & VP880_IS_FXO) {
                    status = VP_STATUS_OPTION_NOT_SUPPORTED;
                    break;
                }

                if (pLineObj->pulseMode != *((VpOptionPulseModeType *)value)) {
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
                }
                break;
#endif

            case VP_OPTION_ID_TIMESLOT:
                txSlot = ((VpOptionTimeslotType *)value)->tx;
                rxSlot = ((VpOptionTimeslotType *)value)->rx;
                status = Vp880SetTimeSlot(pLineCtx, txSlot, rxSlot);
                break;

            case VP_OPTION_ID_CODEC:
                status = Vp880SetCodec(pLineCtx, *((VpOptionCodecType *)value));
                break;

            case VP_OPTION_ID_PCM_HWY:
                if (*((VpOptionPcmHwyType *)value) != VP_OPTION_HWY_A) {
                    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetOptionInternal-"));
                    return VP_STATUS_INVALID_ARG;
                }
                break;

            case VP_OPTION_ID_LOOPBACK:
                /* Timeslot loopback via loopback register */
                switch(*((VpOptionLoopbackType *)value)) {
                    case VP_OPTION_LB_TIMESLOT:
                        pLineObj->opCond[0] |= VP880_INTERFACE_LOOPBACK_EN;
#ifdef VP880_FXS_SUPPORT
                        pLineObj->icr4Values[0] |= VP880_ICR4_VOICE_DAC_CTRL;
                        pLineObj->icr4Values[1] &= ~VP880_ICR4_VOICE_DAC_CTRL;
#endif
                        break;

                    case VP_OPTION_LB_OFF:
                        pLineObj->opCond[0] &= ~(VP880_INTERFACE_LOOPBACK_EN);
#ifdef VP880_FXS_SUPPORT
                        pLineObj->icr4Values[0] &= ~VP880_ICR4_VOICE_DAC_CTRL;
#endif
                        break;

                    case VP_OPTION_LB_DIGITAL:
                    default:
                        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetOptionInternal-"));
                        return VP_STATUS_INVALID_ARG;
                }
                VP_LINE_STATE(VpLineCtxType, pLineCtx,("Writing Op Cond (Loopback) 0x%02X",
                    pLineObj->opCond[0]));

                VpMpiCmdWrapper(deviceId, ecVal, VP880_LOOPBACK_WRT,
                    VP880_LOOPBACK_LEN, pLineObj->opCond);
#ifdef VP880_FXS_SUPPORT
                if (!(pLineObj->status & VP880_IS_FXO)) {
                    VpMpiCmdWrapper(deviceId, ecVal, VP880_ICR4_WRT,
                        VP880_ICR4_LEN, pLineObj->icr4Values);
                }
#endif
                break;

#ifdef VP880_FXS_SUPPORT
            case VP_OPTION_ID_LINE_STATE:
                /* Option does not apply to FXO */
                if (pLineObj->status & VP880_IS_FXO) {
                    status = VP_STATUS_OPTION_NOT_SUPPORTED;
                    break;
                }

                /*
                 * Only supports one type of battery control, so make sure it
                 * is set correctly. If not, return error otherwise continue
                 */
                if (((VpOptionLineStateType *)value)->bat
                    != VP_OPTION_BAT_AUTO) {
                    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetOptionInternal-"));
                    return VP_STATUS_INVALID_ARG;
                }

                VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_RD,
                    VP880_SS_CONFIG_LEN, tempSysConfig);
                if (((VpOptionLineStateType *)value)->battRev == TRUE) {
                    tempSysConfig[0] &= ~(VP880_SMOOTH_PR_EN);
                } else {
                    tempSysConfig[0] |= VP880_SMOOTH_PR_EN;
                }
                VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_WRT,
                    VP880_SS_CONFIG_LEN, tempSysConfig);
                break;
#endif

            case VP_OPTION_ID_EVENT_MASK:
                pNewEventsMask = (VpOptionEventMaskType *)value;

                /*
                 * Zero out the line-specific bits before setting the deviceEventsMask in the
                 * device object.
                 */
                pEventsMask = &pDevObj->deviceEventsMask;
                pEventsMask->faults = pNewEventsMask->faults & VP_EVCAT_FAULT_DEV_EVENTS;
                pEventsMask->signaling = pNewEventsMask->signaling & VP_EVCAT_SIGNALING_DEV_EVENTS;
                pEventsMask->response = pNewEventsMask->response & VP_EVCAT_RESPONSE_DEV_EVENTS;
                pEventsMask->test = pNewEventsMask->test & VP_EVCAT_TEST_DEV_EVENTS;
                pEventsMask->process = pNewEventsMask->process & VP_EVCAT_PROCESS_DEV_EVENTS;
                pEventsMask->fxo = pNewEventsMask->fxo & VP_EVCAT_FXO_DEV_EVENTS;

                /*
                 * Zero out the device-specific bits before setting the lineEventsMask in the
                 * line object.
                 */
                pEventsMask = &pLineObj->lineEventsMask;
                pEventsMask->faults = pNewEventsMask->faults & ~VP_EVCAT_FAULT_DEV_EVENTS;
                pEventsMask->signaling = pNewEventsMask->signaling & ~VP_EVCAT_SIGNALING_DEV_EVENTS;
                pEventsMask->response = pNewEventsMask->response & ~VP_EVCAT_RESPONSE_DEV_EVENTS;
                pEventsMask->test = pNewEventsMask->test & ~VP_EVCAT_TEST_DEV_EVENTS;
                pEventsMask->process = pNewEventsMask->process & ~VP_EVCAT_PROCESS_DEV_EVENTS;
                pEventsMask->fxo = pNewEventsMask->fxo & ~VP_EVCAT_FXO_DEV_EVENTS;

                /* Unmask the unmaskable */
                VpImplementNonMaskEvents(&pLineObj->lineEventsMask, &pDevObj->deviceEventsMask);

                /* Mask those events that the VP880 API-II cannot generate */
                Vp880MaskNonSupportedEvents(&pLineObj->lineEventsMask, &pDevObj->deviceEventsMask);

                /*
                 * The next code section prevents the device from interrupting
                 * the processor if all of the events associated with the
                 * specific hardware interrupt are masked
                 */
                VpMpiCmdWrapper(deviceId, ecVal, VP880_INT_MASK_RD, VP880_INT_MASK_LEN, tempData);

                /* Keep Clock Fault Interrupt Enabled for auto-free run mode. */
                tempData[0] &= ~VP880_CFAIL_MASK;

                if (pDevObj->deviceEventsMask.faults & VP_DEV_EVID_CLK_FLT) {
                    tempData[0] &= ~VP880_CFAIL_MASK;
                }

                if (!(pLineObj->status & VP880_IS_FXO)) {  /* Line is FXS */
#ifdef VP880_FXS_SUPPORT
                    /* Mask off the FXO events */
                    pLineObj->lineEventsMask.fxo |= VP_EVCAT_FXO_MASK_ALL;

                    /*
                     * Never mask the thermal fault interrupt otherwise the
                     * actual thermal fault may not be seen by the VP-API-II.
                     */
                    tempData[channelId] &= ~VP880_TEMPA1_MASK;

                    /*
                     * Never mask the hook interrupt otherwise interrupt modes
                     * of the VP-API-II for LPM types won't work -- hook status
                     * is never updated, leaky line never properly detected.
                     */
                    tempData[channelId] &= ~VP880_HOOK1_MASK;

                    /*
                     * Never mask the gkey interrupt otherwise interrupt modes
                     * of the VP-API-II won't support "get line status"
                     * correctly.
                     */
                    tempData[channelId] &= ~VP880_GNK1_MASK;

                    /* Implement Operation Note 8 on errata notice V103 */
                    tempData[channelId] &= ~(VP880_OCALMY_MASK);
#endif
                } else {  /* Line is FXO */
#ifdef VP880_FXO_SUPPORT
                    /* Mask off the FXS events */
                    pLineObj->lineEventsMask.signaling |= VP880_FXS_SIGNALING_EVENTS;

                    /*
                     * Never mask the FXO interrupts otherwise interrupt modes of the VP-API-II
                     * won't support FXO "get line status" correctly.
                     */
                    tempData[channelId] &= (uint8)(~(VP880_LIU1_MASK | VP880_RING1_DET_MASK
                                                   | VP880_POL1_MASK | VP880_DISC1_MASK
                                                   | VP880_IO2_1_MASK));
#endif
                }
                VpMpiCmdWrapper(deviceId, ecVal, VP880_INT_MASK_WRT, VP880_INT_MASK_LEN, tempData);
                break;

#ifdef VP880_FXS_SUPPORT
            case VP_OPTION_ID_ZERO_CROSS:
                /* Option does not apply to FXO */
                if (pLineObj->status & VP880_IS_FXO) {
                    status = VP_STATUS_OPTION_NOT_SUPPORTED;
                    break;
                }

                VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_RD,
                    VP880_SS_CONFIG_LEN, tempSysConfig);
                if (*(VpOptionZeroCrossType *)value == VP_OPTION_ZC_NONE) {
                    tempSysConfig[0] |= VP880_ZXR_DIS;
                } else {
                    tempSysConfig[0] &= ~(VP880_ZXR_DIS);
                }
                VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_WRT,
                    VP880_SS_CONFIG_LEN, tempSysConfig);

                pLineObj->ringCtrl.zeroCross = *((VpOptionZeroCrossType *)value);
                break;

            case VP_OPTION_ID_RING_CNTRL: {
                VpOptionRingControlType TempRingCtrl = *((VpOptionRingControlType *)value);

                /* Option does not apply to FXO */
                if (pLineObj->status & VP880_IS_FXO) {
                    status = VP_STATUS_OPTION_NOT_SUPPORTED;
                    break;
                }

                if (!(VpCSLACIsSupportedFxsState(pDevCtxLocal->deviceType, TempRingCtrl.ringTripExitSt))) {
                    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetOptionInternal-"));
                    return VP_STATUS_INVALID_ARG;
                }

                pLineObj->ringCtrl = *((VpOptionRingControlType *)value);

                VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_RD,
                    VP880_SS_CONFIG_LEN, tempSysConfig);
                if (pLineObj->ringCtrl.zeroCross == VP_OPTION_ZC_NONE) {
                    tempSysConfig[0] |= VP880_ZXR_DIS;
                } else {
                    tempSysConfig[0] &= ~(VP880_ZXR_DIS);
                }

                VpMpiCmdWrapper(deviceId, ecVal, VP880_SS_CONFIG_WRT,
                    VP880_SS_CONFIG_LEN, tempSysConfig);
                break;
            }
#endif

            case VP_OPTION_ID_PCM_TXRX_CNTRL: {
                uint8 opCondTarget = pLineObj->opCond[0];

                pLineObj->pcmTxRxCtrl = *((VpOptionPcmTxRxCntrlType *)value);
                opCondTarget &= (uint8)(~(VP880_CUT_TXPATH | VP880_CUT_RXPATH));
                opCondTarget &= (uint8)(~(VP880_HIGH_PASS_DIS | VP880_OPCOND_RSVD_MASK));

                Vp880GetTxRxPcmMode(pLineObj, pLineObj->lineState.currentState, &mpiByte);
                opCondTarget |= mpiByte;
                if (opCondTarget != pLineObj->opCond[0]) {
                    pLineObj->opCond[0] = opCondTarget;
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("6. Writing 0x%02X to Operating Conditions", pLineObj->opCond[0]));
                    VpMpiCmdWrapper(deviceId, ecVal, VP880_OP_COND_WRT,
                        VP880_OP_COND_LEN, pLineObj->opCond);
                }
                }
                break;

#ifdef VP_DEBUG
            case VP_OPTION_ID_DEBUG_SELECT:
                /* Update the debugSelectMask in the Line Object. */
                pLineObj->debugSelectMask = *(uint32 *)value;
                break;
#endif

#ifdef VP880_FXS_SUPPORT
            case VP_OPTION_ID_DCFEED_PARAMS: {
                VpOptionDcFeedParamsType dcFeedParams = *((VpOptionDcFeedParamsType *)value);
                uint32 error;

                /* Error check the input args first */
                if (dcFeedParams.validMask &=
                   ~(VP_OPTION_CFG_ILA | VP_OPTION_CFG_GKEY_THRESHOLD | VP_OPTION_CFG_HOOK_THRESHOLD |
                     VP_OPTION_CFG_BATT_FLOOR)) {
                     return VP_STATUS_INVALID_ARG;
                }
                /* Error check the ranges */
                if (dcFeedParams.validMask & VP_OPTION_CFG_ILA) {
                     if ((dcFeedParams.ila > 49000) | (dcFeedParams.ila < 18000)) {
                         return VP_STATUS_INVALID_ARG;
                     }
                }
                if (dcFeedParams.validMask & VP_OPTION_CFG_GKEY_THRESHOLD) {
                     if (dcFeedParams.gkeyThreshold > 42000) {
                         return VP_STATUS_INVALID_ARG;
                     }
                }
                if (dcFeedParams.validMask & VP_OPTION_CFG_HOOK_THRESHOLD) {
                     if ((dcFeedParams.hookThreshold > 15000) | (dcFeedParams.hookThreshold < 8000)) {
                         return VP_STATUS_INVALID_ARG;
                     }
                }
                if (dcFeedParams.validMask & VP_OPTION_CFG_BATT_FLOOR) {
                     if ((dcFeedParams.battFloor > 95000) | (dcFeedParams.battFloor < 5000)) {
                         return VP_STATUS_INVALID_ARG;
                     }
                }
                if (dcFeedParams.validMask & VP_OPTION_CFG_VOC) {
                     if ((dcFeedParams.battFloor > 57000) | (dcFeedParams.battFloor < 12000)) {
                         return VP_STATUS_INVALID_ARG;
                     }
                }

                /* Set VOC if the mask is set */
                if (dcFeedParams.validMask & VP_OPTION_CFG_VOC) {
                    /* Convert to V */
                    dcFeedParams.voc /= 1000;

                    /* Subtract 12V if low range, 36V if high range */
                    dcFeedParams.voc -=
                        ((pLineObj->calLineData.dcFeedRef[VP880_VOC_INDEX] & VP880_VOC_LOW_RANGE)
                        ? 12 : 36);

                    /* Set the internal byte for VOC */
                    error = dcFeedParams.voc % 3;
                    if (error > 1) {
                        dcFeedParams.voc += (3 - error);
                    } else {
                        dcFeedParams.voc -= error;
                    }
                    dcFeedParams.voc /= 3;
                    dcFeedParams.voc = ((dcFeedParams.voc << 2) & VP880_VOC_MASK);
                    pLineObj->calLineData.dcFeedRef[VP880_VOC_INDEX] &= ~VP880_VOC_MASK;
                    pLineObj->calLineData.dcFeedRef[VP880_VOC_INDEX] |= (uint8)dcFeedParams.voc;
                }

                /* Set ILA if the mask is set */
                if (dcFeedParams.validMask & VP_OPTION_CFG_ILA) {
                    /* Convert to mA */
                    dcFeedParams.ila /= 1000;

                    /* Remove the offset */
                    dcFeedParams.ila -= 18;

                    /* Set the user specified target ILA */
                    pLineObj->calLineData.dcFeedRef[VP880_ILA_INDEX] &= ~VP880_ILA_MASK;
                    pLineObj->calLineData.dcFeedRef[VP880_ILA_INDEX] |=
                        ((uint8)dcFeedParams.ila & VP880_ILA_MASK);
                }

                /* Set the Hook Switch Threshold if the mask is set */
                if (dcFeedParams.validMask & VP_OPTION_CFG_HOOK_THRESHOLD) {
                    /* Convert to mA */
                    dcFeedParams.hookThreshold /= 1000;

                    /* Remove the offset */
                    dcFeedParams.hookThreshold -= 8;

                    /* Set the Hook Switch Threshold */
                    pLineObj->calLineData.loopSup[VP880_LOOP_SUP_THRESH_BYTE] &=
                        ~VP880_SWHOOK_THRESH_MASK;
                    pLineObj->calLineData.loopSup[VP880_LOOP_SUP_THRESH_BYTE] |=
                        ((uint8)dcFeedParams.hookThreshold & VP880_SWHOOK_THRESH_MASK);
                }

                /* Set the GKey Threshold if the mask is set */
                if (dcFeedParams.validMask & VP_OPTION_CFG_GKEY_THRESHOLD) {
                    /* Convert to mA */
                    dcFeedParams.gkeyThreshold /= 1000;

                    /* Convert to silicon scale */
                    error = dcFeedParams.gkeyThreshold % 6;
                    if (error > 3) {
                        dcFeedParams.gkeyThreshold += (6 - error);
                    } else {
                        dcFeedParams.gkeyThreshold -= error;
                    }
                    dcFeedParams.gkeyThreshold /= 6;

                    /* Shift into silicon location */
                    dcFeedParams.gkeyThreshold = ((uint16)dcFeedParams.gkeyThreshold << 3) &
                        VP880_GKEY_THRESH_MASK;
                    pLineObj->calLineData.loopSup[VP880_LOOP_SUP_THRESH_BYTE] &=
                        ~VP880_GKEY_THRESH_MASK;
                    pLineObj->calLineData.loopSup[VP880_LOOP_SUP_THRESH_BYTE] |=
                        ((uint8)dcFeedParams.gkeyThreshold & VP880_GKEY_THRESH_MASK);
                }

                /* Retreive the Floor Voltage */
                if ((pDevObj->stateInt & VP880_IS_ABS) != VP880_IS_ABS) {
                    /* Convert to V */
                    dcFeedParams.battFloor /= 1000;

                    /* Remove 5V Offset and Convert to Silicon Scale */
                    dcFeedParams.battFloor -= 5;
                    error = dcFeedParams.battFloor % 6;
                    if (error > 2) {
                        dcFeedParams.battFloor += (6 - error);
                    } else {
                        dcFeedParams.battFloor -= error;
                    }
                    dcFeedParams.battFloor /= 5;

                    /* Put into silicon bit location */
                    pDevObj->swParams[VP880_FLOOR_VOLTAGE_BYTE] &= ~VP880_FLOOR_VOLTAGE_MASK;
                    pDevObj->swParams[VP880_FLOOR_VOLTAGE_BYTE] |=
                        ((uint8)dcFeedParams.battFloor & VP880_FLOOR_VOLTAGE_MASK);
                }
            } break;

            case VP_OPTION_ID_RINGING_PARAMS: {
                VpOptionRingingParamsType ringingParams = *((VpOptionRingingParamsType *)value);
                bool updateRingReq = FALSE;
                bool updateLoopSupervisionReq = FALSE;
                bool isTrap = FALSE;

                uint8 ringingValues[VP880_RINGING_PARAMS_LEN];
                VpMemCpy(ringingValues, pLineObj->ringingParamsRef, VP880_RINGER_PARAMS_LEN);

                if ((ringingValues[0] & VP880_SIGGEN1_SINTRAP_MASK) == VP880_SIGGEN1_TRAP) {
                    isTrap = TRUE;
                }

                if ((ringingParams.validMask & VP_OPTION_CFG_FREQUENCY)
                                            == VP_OPTION_CFG_FREQUENCY) {
                    /* Update bool to indicate Ringing is being changed */
                    updateRingReq = TRUE;
                    if (isTrap) {
                        /* Trapezopidal */
                        /* Undo the mHz conversion (last step) */
                        ringingParams.frequency /=  1000;

                        /*
                         * Register Content is FREQB = 8000 / Fring. Convert this value to the
                         * register content.
                         */
                        ringingParams.frequency = (8000 / ringingParams.frequency);

                        /* ringingParams.frequency is NOW in register format */
                        ringingValues[VP880_SIGB_FREQ_MSB] =
                            (uint8)((uint16)(ringingParams.frequency >> 8) & 0x00FF);
                        ringingValues[VP880_SIGB_FREQ_LSB] =
                            (uint8)((uint16)(ringingParams.frequency) & 0x00FF);
                    } else {
                        /* Sine */
                        /* Register Frequency is in 0.3662Hz. Multiply by 10 / 3662 converts from mHz */
                        ringingParams.frequency *= 10;
                        ringingParams.frequency /= 3662; /* VP880_GENA_FREQ_STEPSIZE; */
                        ringingValues[VP880_SIGA_FREQ_MSB] =
                            (uint8)((ringingParams.frequency >> 8) & 0x00FF);
                        ringingValues[VP880_SIGA_FREQ_LSB] =
                            (uint8)((ringingParams.frequency) & 0x00FF);

                        /* Sine - no rise time allowed */
                        ringingValues[VP880_SIGB_FREQ_MSB] = 0;
                        ringingValues[VP880_SIGB_FREQ_LSB] = 0;
                    }
                }

                if ((ringingParams.validMask & VP_OPTION_CFG_AMPLITUDE)
                                            == VP_OPTION_CFG_AMPLITUDE) {
                    /* Update bool to indicate Ringing is being changed */
                    updateRingReq = TRUE;
                    /*
                     * Amplitude is a 16-bit signed value (+/-154.4V) for steps of ~4.71mV. The result
                     * is a 32-bit signed value.
                     */
                    ringingParams.amplitude *= 1000;
                    ringingParams.amplitude /= 4730; /* VP880_RINGING_AMP_SCALE; */
                    ringingValues[VP880_SIGA_AMP_MSB] =
                        (uint8)((ringingParams.amplitude >> 8) & 0x00FF);
                    ringingValues[VP880_SIGA_AMP_LSB] =
                        (uint8)((ringingParams.amplitude) & 0x00FF);
                }

                if ((ringingParams.validMask & VP_OPTION_CFG_DC_BIAS)
                                            == VP_OPTION_CFG_DC_BIAS) {
                    /* Update bool to indicate Ringing is being changed */
                    updateRingReq = TRUE;

                    /*
                     * DC Bias is a 16-bit signed value (+/-154.4V) for steps of ~4.71mV. The result
                     * is a 32-bit signed value because it needs to represent 48000mV
                     */
                    ringingParams.dcBias *= 1000;
                    ringingParams.dcBias /= 4712; /* VP880_RINGING_BIAS_SCALE; */
                    ringingValues[VP880_SIGA_BIAS_MSB] = (uint8)((ringingParams.dcBias >> 8) & 0x00FF);
                    ringingValues[VP880_SIGA_BIAS_LSB] = (uint8)((ringingParams.dcBias) & 0x00FF);
                }

                if ((ringingParams.validMask & VP_OPTION_CFG_TRAP_RISE_TIME)
                                            == VP_OPTION_CFG_TRAP_RISE_TIME) {
                    /* Update bool to indicate Ringing is being changed */
                    updateRingReq = TRUE;
                    if (isTrap) {
                        /* Trapezoidal */
                        /* Protect Divide by 0 error */
                        if (ringingParams.trapRiseTime == 0) {
                            ringingParams.trapRiseTime = 1;
                        }

                        /* Convert per Command Set from uS (FREQA = 2.73 / trise) */
                        ringingParams.trapRiseTime = (2730000 / ringingParams.trapRiseTime);

                        ringingValues[VP880_SIGA_FREQ_MSB] =
                            (uint8)((ringingParams.trapRiseTime >> 8) & 0x00FF);
                        ringingValues[VP880_SIGA_FREQ_LSB] =
                            (uint8)((ringingParams.trapRiseTime) & 0x00FF);
                    } else {
                        /* Sine - no rise time allowed */
                        ringingValues[VP880_SIGB_FREQ_MSB] = 0;
                        ringingValues[VP880_SIGB_FREQ_LSB] = 0;
                    }
                }

                if ((ringingParams.validMask & VP_OPTION_CFG_RINGTRIP_THRESHOLD)
                                            == VP_OPTION_CFG_RINGTRIP_THRESHOLD) {
                    /* Update bool to indicate Ringing is being changed */
                    updateLoopSupervisionReq = TRUE;
                    ringingParams.ringTripThreshold /= 500;
                    pLineObj->calLineData.loopSup[VP880_LOOP_SUP_RT_MODE_BYTE] &=
                        ~VP880_LOOP_SUP_RT_MODE_THRSHLD;
                    pLineObj->calLineData.loopSup[VP880_LOOP_SUP_RT_MODE_BYTE] |=
                        ((uint8)ringingParams.ringTripThreshold & VP880_LOOP_SUP_RT_MODE_THRSHLD);
                }

                if ((ringingParams.validMask & VP_OPTION_CFG_RING_CURRENT_LIMIT)
                                            == VP_OPTION_CFG_RING_CURRENT_LIMIT) {
                    /* Update bool to indicate Ringing is being changed */
                    updateLoopSupervisionReq = TRUE;

                    /* Compute the Ringing Current Limit :: 50-112mA, step = 2mA */
                    ringingParams.ringCurrentLimit /= 2000;
                    ringingParams.ringCurrentLimit -= 25;
                    pLineObj->calLineData.loopSup[VP880_LOOP_SUP_RING_LIM_BYTE] &=
                        ~VP880_LOOP_SUP_RING_LIM_MASK;
                    pLineObj->calLineData.loopSup[VP880_LOOP_SUP_RING_LIM_BYTE] |=
                        ringingParams.ringCurrentLimit;
                }

                if (updateRingReq) {
                    /* Write back to our reference */
                    VpMemCpy(pLineObj->ringingParamsRef, ringingValues, VP880_RINGER_PARAMS_LEN);

                    /* Write back to our register value to be modified when Ringing starts */
                    VpMemCpy(pLineObj->ringingParams, pLineObj->ringingParamsRef,
                        VP880_RINGER_PARAMS_LEN);

                    /*
                     * Clear the flags that indicate Ringing is properly loaded ... because it
                     * isn't since we just changed it. The Signal Generator will change next Ringing
                     * Cycle.
                     */
                    pLineObj->status &= ~(VP880_RING_GEN_NORM | VP880_RING_GEN_REV);
                }
                if (updateLoopSupervisionReq) {
                    /* Write back to the Loop Supervision Cached Data */
                    VpMemCpy(pLineObj->loopSup, pLineObj->calLineData.loopSup, VP880_LOOP_SUP_LEN);

                    /* If off-hook -> apply the hysteresis */
                    if ((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_HOOK)
                        == VP_CSLAC_HOOK) {
                        if ((pLineObj->loopSup[0] & VP880_LOOP_SUP_LIU_THRESH_BITS) >=
                            pLineObj->hookHysteresis) {
                            pLineObj->loopSup[0] -= pLineObj->hookHysteresis;
                        } else {
                            pLineObj->loopSup[0] &= ~VP880_LOOP_SUP_LIU_THRESH_BITS;
                        }
                    }
                    VpMpiCmdWrapper(deviceId, pLineObj->ecVal, VP880_LOOP_SUP_WRT,
                        VP880_LOOP_SUP_LEN, pLineObj->loopSup);

                }
            } break;
#endif /* VP880_FXS_SUPPORT */

            case VP_DEVICE_OPTION_ID_PULSE:
            case VP_DEVICE_OPTION_ID_PULSE2:
            case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
            case VP_DEVICE_OPTION_ID_DEVICE_IO:
                status = VP_STATUS_INVALID_ARG;
                break;

            default:
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
        }
    } else {
        pDevObj = pDevCtx->pDevObj;
        deviceId = pDevObj->deviceId;
        maxChan = pDevObj->staticInfo.maxChannels;
        ecVal = pDevObj->ecVal;

        switch (option) {
#ifdef VP880_FXS_SUPPORT
            case VP_DEVICE_OPTION_ID_PULSE:
                if (pDevObj->stateInt & VP880_IS_FXO_ONLY) {
                    status = VP_STATUS_OPTION_NOT_SUPPORTED;
                    break;
                }
                pDevObj->pulseSpecs = *((VpOptionPulseType *)value);
                break;

            case VP_DEVICE_OPTION_ID_PULSE2:
                if (pDevObj->stateInt & VP880_IS_FXO_ONLY) {
                    status = VP_STATUS_OPTION_NOT_SUPPORTED;
                    break;
                }
                pDevObj->pulseSpecs2 = *((VpOptionPulseType *)value);
                break;

            case VP_DEVICE_OPTION_ID_CRITICAL_FLT: {
                VpOptionCriticalFltType criticalFault =
                    *((VpOptionCriticalFltType *)value);

                if (pDevObj->stateInt & VP880_IS_FXO_ONLY) {
                    status = VP_STATUS_OPTION_NOT_SUPPORTED;
                    break;
                }

                if ((criticalFault.acFltDiscEn == TRUE)
                 || (criticalFault.dcFltDiscEn == TRUE)) {
                    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetOptionInternal-"));
                    return VP_STATUS_INVALID_ARG;
                }

                pDevObj->criticalFault = *((VpOptionCriticalFltType *)value);

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

            case VP_DEVICE_OPTION_ID_DEVICE_IO: {
                uint8 ioTypeReq[2] = {0x00, 0x00};
                uint8 ecMod[] = {VP880_EC_CH1, VP880_EC_CH2};
                uint8 maxPinsPerLine = VP880_MAX_PINS_PER_LINE;
                uint8 pcn = pDevObj->staticInfo.rcnPcn[VP880_PCN_LOCATION];

                /*
                 * This 'AND' mask is only used to check if the I/O pins on
                 * ZSI devices are being configured for open drain. Should be
                 * set to either 0x1 or 0x3
                 */
                uint32 andMask = 0x3;

                deviceIo = *(VpOptionDeviceIoType *)(value);

                if ((pcn == VP880_DEV_PCN_88536) || (pcn == VP880_DEV_PCN_88264)) {
                    /*
                     * Direction = '1' for output, Type = '1' for Open.
                     * So it's ok if the direction is NOT output OR if the
                     * output type is NOT Open for the pins supported.
                     */
                    if (deviceIo.directionPins_31_0
                      & deviceIo.outputTypePins_31_0
                      & andMask) {
                        return VP_STATUS_INVALID_ARG;
                    }

                    /*
                     * VE8830 Chipset (VP880_DEV_PCN_88536) and 88264 both have
                     * only 1 I/O pin
                     */
                    maxPinsPerLine = 1;
                    andMask = 0x1;
                }

                /*
                 * Read the current direction pins and create a local array
                 * that matches what the input is requesting.
                 */
                for (channelId = 0; channelId < maxChan; channelId++) {
                    uint8 pinCnt = 0;

                    VpMpiCmdWrapper(deviceId, (ecVal | ecMod[channelId]),
                        VP880_IODIR_REG_RD, VP880_IODIR_REG_LEN,
                        &ioDirection[channelId]);

                    for (pinCnt = 0; pinCnt < maxPinsPerLine; pinCnt++) {
                        if (deviceIo.directionPins_31_0 & (1 << (channelId + 2 * pinCnt))) {
                            if (pinCnt == 0) {
                                ioTypeReq[channelId] |=
                                    ((deviceIo.outputTypePins_31_0 & (1 << (channelId + 2 * pinCnt)))
                                    ? VP880_IODIR_IO1_OPEN_DRAIN : VP880_IODIR_IO1_OUTPUT);
                            } else {
                                ioTypeReq[channelId] |= (VP880_IODIR_IO2_OUTPUT << (pinCnt - 1));
                            }
                        } else {
                            /*
                             * This is here for show only. Input is 0, so no
                             * OR operation is needed.
                             */
                            /*  ioTypeReq[channelId] |= VP880_IODIR_IO1_INPUT; */
                        }
                    }

                    /* Protect the I/O lines dedictated to termination types */
                    pLineCtxLocal = pDevCtx->pLineCtx[channelId];

                    if (pLineCtxLocal != VP_NULL) {
                        uint8 fxoMask;
                        pLineObj = pLineCtxLocal->pLineObj;
                        switch (pLineObj->termType) {
                            case VP_TERM_FXO_GENERIC:
                            case VP_TERM_FXO_DISC:
#ifdef VP880_CLARE_RINGING_DETECT
                                /* Ringing detector on IO4 */
                                ioTypeReq[channelId] |= VP880_IODIR_EXPDT_MASK;
#endif  /* VP880_CLARE_RINGING_DETECT */
                                fxoMask = (VP880_FXO_CID_LINE == VP880_IODATA_IO2)
                                    ? VP880_IODIR_IO2_MASK : VP880_IODIR_IO3_MASK;

                                ioTypeReq[channelId] &= ~fxoMask;
                                ioTypeReq[channelId] |= (ioDirection[channelId] & fxoMask);

                                /*
                                 * No break required becuase FXO also has I/O1
                                 * dedicated.
                                 */

                            case VP_TERM_FXS_ISOLATE:
                            case VP_TERM_FXS_ISOLATE_LP:
                            case VP_TERM_FXS_SPLITTER:
                                ioTypeReq[channelId] &= ~VP880_IODIR_IO1_MASK;
                                ioTypeReq[channelId] |= (ioDirection[channelId] & VP880_IODIR_IO1_MASK);
                                break;

                            default:
                                break;
                        }
                    }
                }

                /* Set the current device IO control information */
                for (channelId = 0; channelId < maxChan; channelId++) {
                    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("1. Write IODIR 0x%02X on Channel %d",
                        ioTypeReq[channelId], channelId));

                    VpMpiCmdWrapper(deviceId, (ecVal | ecMod[channelId]),
                        VP880_IODIR_REG_WRT, VP880_IODIR_REG_LEN,
                        &ioTypeReq[channelId]);
                }
                }
                break;

#ifdef VP_DEBUG
            case VP_OPTION_ID_DEBUG_SELECT:
                /* Update the debugSelectMask in the Device Object. */
                pDevObj->debugSelectMask = *(uint32 *)value;
                break;
#endif

            default:
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
        }
    }
    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp880SetOptionInternal-"));

    return status;
}

/**
 * Vp880MaskNonSupportedEvents()
 *  This function masks the events that are not supported by the VP880 API-II.
 * It should only be called by SetOptionInternal when event masks are being
 * modified.
 *
 * Preconditions:
 *  None. Utility function to modify event structures only.
 *
 * Postconditions:
 *  Event structures passed are modified with masked bits for non-supported
 * VP880 API-II events.
 */
void
Vp880MaskNonSupportedEvents(
    VpOptionEventMaskType *pLineEventsMask, /**< Line Events Mask to modify for
                                             * non-masking
                                             */
    VpOptionEventMaskType *pDevEventsMask)  /**< Device Events Mask to modify
                                             * for non-masking
                                             */
{
    VP_API_FUNC_INT(None, VP_NULL, ("+Vp880MaskNonSupportedEvents()"));
    pLineEventsMask->faults |= VP880_NONSUPPORT_FAULT_EVENTS;
    pLineEventsMask->signaling |= VP880_NONSUPPORT_SIGNALING_EVENTS;
    pLineEventsMask->response |= VP880_NONSUPPORT_RESPONSE_EVENTS;
    pLineEventsMask->test |= VP880_NONSUPPORT_TEST_EVENTS;
    pLineEventsMask->process |= VP880_NONSUPPORT_PROCESS_EVENTS;
    pLineEventsMask->fxo |= VP880_NONSUPPORT_FXO_EVENTS;

    pDevEventsMask->faults |= VP880_NONSUPPORT_FAULT_EVENTS;
    pDevEventsMask->signaling |= VP880_NONSUPPORT_SIGNALING_EVENTS;
    pDevEventsMask->response |= VP880_NONSUPPORT_RESPONSE_EVENTS;
    pDevEventsMask->test |= VP880_NONSUPPORT_TEST_EVENTS;
    pDevEventsMask->process |= VP880_NONSUPPORT_PROCESS_EVENTS;
    pDevEventsMask->fxo |= VP880_NONSUPPORT_FXO_EVENTS;
    VP_API_FUNC_INT(None, VP_NULL, ("-Vp880MaskNonSupportedEvents()"));
    return;
}

/**
 * Vp880DeviceIoAccess()
 *  This function is used to access device IO pins of the Vp880. See API-II
 * documentation for more information about this function.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Reads/Writes from device IO pins.
 */
VpStatusType
Vp880DeviceIoAccess(
    VpDevCtxType *pDevCtx,
    VpDeviceIoAccessDataType *pDeviceIoData)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    VpLineCtxType *pLineCtx;
    Vp880LineObjectType *pLineObj;

    bool isDedicatedPins = FALSE;

    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal;
    uint8 chanNum, maxChan;
    uint8 ioDataReg[2] = {0x00, 0x00};  /* IO Status from each channel */

    /*
     * tempIoData and tempIoMask are representations of the device content to be
     * written.
     */
    uint8 tempIoData[2] = {0x00, 0x00};
    uint8 tempIoMask[2] = {0x00, 0x00};

    VpDeviceIoAccessDataType *pAccessData =
        &(pDevObj->getResultsOption.optionData.deviceIoData);

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("+Vp880DeviceIoAccess()"));

    /* VE8830 Chip set does not have I/O pins */
    if (pDevObj->staticInfo.rcnPcn[1] == VP880_DEV_PCN_88536) {
        VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("-Vp880DeviceIoAccess()"));
        return VP_STATUS_FUNC_NOT_SUPPORTED;
    }

    maxChan = pDevObj->staticInfo.maxChannels;

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("-Vp880DeviceIoAccess()"));
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /*
     * Do not proceed if the device calibration is in progress. This could
     * damage the device.
     */
    if (pDevObj->state & VP_DEV_IN_CAL) {
        VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("-Vp880DeviceIoAccess()"));
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    for (chanNum = 0; chanNum < maxChan; chanNum++) {
        uint16 dataMask;
        uint8 loopCnt;
        uint16 tempData;
        for (loopCnt = 0; loopCnt < 6; loopCnt++) {
            dataMask = 0x01;
            dataMask = (dataMask << (chanNum + 2 * loopCnt));

            tempData = 0;
            tempData = (uint16)(pDeviceIoData->accessMask_31_0 & dataMask);
            tempIoMask[chanNum] |= (uint8)(tempData >> (chanNum + loopCnt));

            tempData = 0;
            tempData = (uint16)(pDeviceIoData->deviceIOData_31_0 & dataMask);

            tempIoData[chanNum] |= (uint8)(tempData >> (chanNum + loopCnt));
        }
    }

    /* Read the current state of the IO lines */
    for (chanNum = 0; chanNum < maxChan; chanNum++) {
        pLineCtx = pDevCtx->pLineCtx[chanNum];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            ecVal = pLineObj->ecVal;

            /* Protect the CID line for FXO type */
            if ((pLineObj->status & VP880_IS_FXO)
             || (pLineObj->termType == VP_TERM_FXO_DISC)) {
                if (tempIoMask[chanNum] & VP880_FXO_CID_LINE) {
                    VP_INFO(VpLineCtxType, pLineCtx, ("Dedicated Pin Error"));
                    isDedicatedPins = TRUE;
                }
                tempIoMask[chanNum] &= ~VP880_FXO_CID_LINE;
            } else {    /* Force Data [2:4] to 0 for FXS */
                tempIoData[chanNum] &= (VP880_IODATA_IO1 | VP880_IODATA_IO2);
            }

            /* Protect access to I/O1 if FXO or Relay Type terminations */
            if ((pLineObj->status & VP880_IS_FXO)
             || (pLineObj->termType == VP_TERM_FXS_ISOLATE)
             || (pLineObj->termType == VP_TERM_FXS_ISOLATE_LP)
             || (pLineObj->termType == VP_TERM_FXS_SPLITTER)
             || (pLineObj->termType == VP_TERM_FXS_SPLITTER_LP)) {

                if (tempIoMask[chanNum] & VP880_IODATA_IO1) {
                    VP_INFO(VpLineCtxType, pLineCtx, ("Dedicated Pin Error"));
                    isDedicatedPins = TRUE;
                }
                tempIoMask[chanNum] &= ~VP880_IODATA_IO1;
            }
        } else {
            VP_LINE_STATE(None, NULL, ("VpDeviceIoAccess: NULL Line Found on Ch %d",
                chanNum));
            ecVal = pDevObj->ecVal;
            ecVal |= ((chanNum == 0) ? VP880_EC_CH1 : VP880_EC_CH2);
        }

        /* Read the IO Data, whether a line exists or not */
        VpMpiCmdWrapper(deviceId, ecVal, VP880_IODATA_REG_RD,
            VP880_IODATA_REG_LEN, &ioDataReg[chanNum]);
    }

    *pAccessData = *pDeviceIoData;

    if (pDeviceIoData->accessType == VP_DEVICE_IO_WRITE) {
        for (chanNum = 0; chanNum < maxChan; chanNum++) {
            uint8 tempData = ioDataReg[chanNum];
            ecVal = pDevObj->ecVal;
            ecVal |= ((chanNum == 0) ? VP880_EC_CH1 : VP880_EC_CH2);

            tempData &= ~tempIoMask[chanNum];
            tempData |= (tempIoMask[chanNum] & tempIoData[chanNum]);

            VP_LINE_STATE(None, NULL, ("VpDeviceIoAccess: Write IODATA 0x%02X on Ch %d",
                tempData, chanNum));

            VpMpiCmdWrapper(deviceId, ecVal, VP880_IODATA_REG_WRT,
                VP880_IODATA_REG_LEN, &tempData);
        }
    } else {    /* VP_DEVICE_IO_READ */
        isDedicatedPins = FALSE;

        pAccessData->deviceIOData_31_0 = 0;
        pAccessData->deviceIOData_63_32 = 0;

        for (chanNum = 0; chanNum < maxChan; chanNum++) {
            uint8 loopCnt;
            uint32 tempIoRdData;
            uint16 dataMask;

            for (loopCnt = 0; loopCnt < 6; loopCnt++) {
                dataMask = 0x01;
                dataMask = (dataMask << loopCnt);

                /* Extract the bit we're after in this loop */
                tempIoRdData = ioDataReg[chanNum];

                /* This is the location per the device. Move to API location */
                tempIoRdData &= dataMask;
                tempIoRdData = (tempIoRdData << (chanNum + loopCnt));

                /* Mask off ONLY the bit being provided in this loop */
                dataMask = 0x01;
                dataMask = (dataMask << (chanNum + 2 * loopCnt));
                tempIoRdData &= dataMask;

                pAccessData->deviceIOData_31_0 |= tempIoRdData;
            }
        }
        pAccessData->deviceIOData_31_0 &= pDeviceIoData->accessMask_31_0;
    }

    pDevObj->deviceEvents.response |= VP_DEV_EVID_IO_ACCESS_CMP;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("-Vp880DeviceIoAccess()"));
    return ((isDedicatedPins == TRUE) ? VP_STATUS_DEDICATED_PINS : VP_STATUS_SUCCESS);
}

/**
 * Vp880SetCodec()
 *  This function sets the codec mode on the line specified.
 *
 * Preconditions:
 *  The line must first be initialized.
 *
 * Postconditions:
 *  The codec mode on the line is set.  This function returns the success code
 * if the codec mode specified is supported.
 */
VpStatusType
Vp880SetCodec(
    VpLineCtxType *pLineCtx,
    VpOptionCodecType codec)    /* Encoding, as defined by LineCodec typedef */
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 codecReg;
    uint8 ecVal = pLineObj->ecVal;
    bool isNextWideBand = FALSE;
    bool isCurrentWideBand = FALSE;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp880SetCodec()"));

    /* Basic error checking */
    switch(codec) {
        case VP_OPTION_ALAW:                /**< Select G.711 A-Law PCM encoding */
        case VP_OPTION_MLAW:                /**< Select G.711 Mu-Law PCM encoding */
        case VP_OPTION_LINEAR:              /**< Select Linear PCM encoding */
            break;
        case VP_OPTION_WIDEBAND:            /**< Select Wideband Linear PCM encoding */
        case VP_OPTION_ALAW_WIDEBAND:       /**< Select Wideband A-Law PCM encoding */
        case VP_OPTION_MLAW_WIDEBAND:       /**< Select Wideband Mu-Law PCM encoding */
            /*
             * Return Error if this device does not support Wideband Mode while the Application is
             * trying to select it.
             */
            if (!(pDevObj->stateInt & VP880_WIDEBAND)) {
                VP_ERROR(VpLineCtxType, pLineCtx,
                         ("Vp880SetCodec() - Wideband Mode Not Supported on this silicon"));
                return VP_STATUS_INVALID_ARG;
            }
            isNextWideBand = TRUE;
            break;
        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp880SetCodec() - Invalid codec"));
            return VP_STATUS_INVALID_ARG;
    }

    /*
     * Don't allow this change during calibration. Cache the target value that will be applied
     * when calibration is complete.
     */
    if (pLineObj->status & VP880_LINE_IN_CAL) {
        pLineObj->calLineData.updateFlags |= CODEC_UPDATE_REQ;
        pLineObj->codec = codec;
        return VP_STATUS_SUCCESS;
    }

    /* Adjust the EC value for Wideband mode as needed */
    ecVal &= ~VP880_WIDEBAND_MODE;
    ecVal |= ((isNextWideBand) ? VP880_WIDEBAND_MODE : 0);

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
        uint8 converterCfg[VP880_CONV_CFG_LEN];
        uint8 newValue;

        pDevObj->devTimer[VP_DEV_TIMER_WB_MODE_CHANGE] =
            MS_TO_TICKRATE(VP_WB_CHANGE_MASK_TIME,
            pDevObj->devProfileData.tickRate) | VP_ACTIVATE_TIMER;

        VpMpiCmdWrapper(deviceId, ecVal, VP880_CONV_CFG_RD, VP880_CONV_CFG_LEN, converterCfg);
        converterCfg[0] &= ~VP880_CC_RATE_MASK;

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
            newValue = ((isNextWideBand) ? VP880_CC_4KHZ_RATE : VP880_CC_8KHZ_RATE);
        } else if(pDevObj->devProfileData.tickRate <=320){  /* 320 * 3.90625uS = 1.25mS */
            newValue = ((isNextWideBand) ? VP880_CC_2KHZ_RATE : VP880_CC_4KHZ_RATE);
        } else if(pDevObj->devProfileData.tickRate <=640){  /* 640 * 3.90625uS = 2.5mS */
            newValue = ((isNextWideBand) ? VP880_CC_1KHZ_RATE : VP880_CC_2KHZ_RATE);
        } else if(pDevObj->devProfileData.tickRate <=1280){ /* 1280 * 3.90625uS = 5mS */
            newValue = ((isNextWideBand) ? VP880_CC_500HZ_RATE : VP880_CC_1KHZ_RATE);
        } else {
            newValue = VP880_CC_500HZ_RATE;
        }

        pDevObj->txBufferDataRate = newValue;
        converterCfg[0] |= newValue;
        /*
         * If channel is going to Wideband mode, we can immediately update the device object. But
         * if leaving Wideband mode, we have to let the tick manage it because the other line may
         * still be in Wideband mode.
         */
        if (isNextWideBand) {
            pDevObj->ecVal |= VP880_WIDEBAND_MODE;
        }

        VpMpiCmdWrapper(deviceId, ecVal, VP880_CONV_CFG_WRT, VP880_CONV_CFG_LEN, converterCfg);
        /*
         * This value cannot be set to 0 BECAUSE "0" is used to indicate there has been no recent
         * changes. So use channelId's starting index = 1.
         */
        pDevObj->lastCodecChange = pLineObj->channelId+1;
    }

    /* Read the current state of the codec register */
    VpMpiCmdWrapper(deviceId, ecVal, VP880_OP_FUNC_RD, VP880_OP_FUNC_LEN, &codecReg);

    if (!VpCSLACGetCodecReg(codec, &codecReg)) {
        VP_ERROR(VpLineCtxType, pLineCtx, ("Vp880SetCodec() - Failure"));
        return VP_STATUS_FAILURE;
    }

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("1. Writing 0x%02X to Operating Functions EC 0x%02X",
        codecReg, ecVal));
    VpMpiCmdWrapper(deviceId, ecVal, VP880_OP_FUNC_WRT, VP880_OP_FUNC_LEN, &codecReg);

    pLineObj->codec = codec;
    pLineObj->ecVal = ecVal;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp880SetCodec()"));

    return VP_STATUS_SUCCESS;
}   /* Vp880SetCodec()  */

/**
 * Vp880SetTimeSlot()
 *  This function set the RX and TX timeslot for a device channel. Valid
 * timeslot numbers start at zero. The upper bound is system dependent.
 *
 * Preconditions:
 *  The line must first be initialized.
 *
 * Postconditions:
 *  The timeslots on the line are set.  This function returns the success code
 * if the timeslot numbers specified are within the range of the device based on
 * the PCLK rate.
 */
VpStatusType
Vp880SetTimeSlot(
    VpLineCtxType *pLineCtx,
    uint8 txSlot,       /**< The TX PCM timeslot */
    uint8 rxSlot)       /**< The RX PCM timeslot */
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 ecVal = pLineObj->ecVal;
    uint8 mpiBuffer[2 + VP880_TX_TS_LEN + VP880_RX_TS_LEN];
    uint8 mpiIndex = 0;
    uint8 pcn = pDevObj->staticInfo.rcnPcn[VP880_PCN_LOCATION];

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp880SetTimeSlot()"));

    /* Validate the tx and rx time slot value */
    if ((txSlot >= pDevObj->devProfileData.pcmClkRate/64) ||
        (rxSlot >= pDevObj->devProfileData.pcmClkRate/64)) {
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp880SetTimeSlot()"));
        return VP_STATUS_INPUT_PARAM_OOR;
    }

    if ((pcn == VP880_DEV_PCN_88536) || (pcn == VP880_DEV_PCN_88264)) {
        if (txSlot == 0) {
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp880SetTimeSlot()"));
            return VP_STATUS_INVALID_ARG;
        }
        txSlot--;
    }

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_TX_TS_WRT,
        VP880_TX_TS_LEN, &txSlot);

    mpiIndex = VpCSLACBuildMpiBuffer(mpiIndex, mpiBuffer, VP880_RX_TS_WRT,
        VP880_RX_TS_LEN, &rxSlot);

    VpMpiCmdWrapper(deviceId, ecVal, mpiBuffer[0], mpiIndex-1, &mpiBuffer[1]);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp880SetTimeSlot()"));
    return VP_STATUS_SUCCESS;
}   /* Vp880SetTimeSlot()   */

/**
 * Vp880VirtualISR()
 *  This function is called everytime the device causes an interrupt
 *
 * Preconditions
 *  A device interrupt has just occured
 *
 * Postcondition
 *  This function should be called from the each device's ISR.
 *  This function could be inlined to improve ISR performance.
 */
#ifndef VP880_SIMPLE_POLLED_MODE
VpStatusType
Vp880VirtualISR(
    VpDevCtxType *pDevCtx)
{
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("+Vp880VirtualISR()"));

#if defined (VP880_INTERRUPT_LEVTRIG_MODE)
    VpSysDisableInt(deviceId);
#endif
    /* Device Interrupt Received */
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    pDevObj->state |= VP_DEV_PENDING_INT;
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("-Vp880VirtualISR()"));

    return VP_STATUS_SUCCESS;
}   /* Vp880VirtualISR() */
#endif

/**
 * Vp880LowLevelCmd()
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
 */
#if !defined(VP_REDUCED_API_IF) || defined(ZARLINK_CFG_INTERNAL)
VpStatusType
Vp880LowLevelCmd(
    VpLineCtxType *pLineCtx,
    uint8 *pCmdData,
    uint8 len,
    uint16 handle)
{
    Vp880LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp880DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal = pLineObj->ecVal;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("+Vp880LowLevelCmd()"));

    if (pDevObj->deviceEvents.response & VP880_READ_RESPONSE_MASK) {
        VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp880LowLevelCmd()"));
        return VP_STATUS_DEVICE_BUSY;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    if(pCmdData[0] & 0x01) { /* Read Command */
        VpMpiCmdWrapper(deviceId, ecVal, pCmdData[0], len, &(pDevObj->mpiData[0]));
        pDevObj->mpiLen = len;
        pLineObj->lineEvents.response |= VP_LINE_EVID_LLCMD_RX_CMP;
    } else {
        VpMpiCmdWrapper(deviceId, ecVal, pCmdData[0], len, &pCmdData[1]);
        VpMemCpy(pDevObj->mpiData, pCmdData, len);
        pLineObj->lineEvents.response |= VP_LINE_EVID_LLCMD_TX_CMP;
    }
    pLineObj->lineEventHandle = handle;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("-Vp880LowLevelCmd()"));

    return VP_STATUS_SUCCESS;
}   /* Vp880LowLevelCmd()   */
#endif

/**
 * Vp880UpdateBufferChanSel()
 *  This function sets the test buffer channel selection so that the hook bits
 * in the test buffer are valid for any channel in the active state.
 *
 * In order for any of the hook bits to be valid, the codec must be activated
 * for the channel selected by CBS in the device mode register.  This function
 * changes CBS so that if the given channel is being activated, that channel
 * is selected.  If the given channel is being deactivated, the other channel is
 * selected.
 *
 * This function should be called just before activating or deactivating the
 * codec for any channel.
 *
 * Arguments:
 *  channelId  -  The channel that is being changed
 *  sysState   -  Value of the system state register that is going to be
 *                programmed.  Contains the codec activate/deactivate bit.
 */
void
Vp880UpdateBufferChanSel(
    Vp880DeviceObjectType *pDevObj,
    uint8 channelId,
    uint8 sysState,
    bool devWrite)
{
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal = ((channelId == 0) ? VP880_EC_CH1 : VP880_EC_CH2);
    bool activated;
    uint8 preDevMode = pDevObj->devMode[0];

    VP_API_FUNC_INT(None, VP_NULL, ("+Vp880UpdateBufferChanSel()"));

    if (pDevObj->staticInfo.rcnPcn[VP880_RCN_LOCATION] <= VP880_REV_VC) {
        /* No test buffer to worry about in older revs */
        VP_API_FUNC_INT(None, VP_NULL, ("-Vp880UpdateBufferChanSel()"));
        return;
    }

#ifdef VP880_INCLUDE_TESTLINE_CODE
    /* Do nothing if either channel is under test */
    if (Vp880IsChnlUndrTst(pDevObj, 0) || Vp880IsChnlUndrTst(pDevObj, 1)) {
        VP_API_FUNC_INT(None, VP_NULL, ("-Vp880UpdateBufferChanSel()"));
        return;
    }
#endif

    if ((sysState & VP880_SS_ACTIVATE_MASK) == VP880_SS_ACTIVATE_MASK) {
        activated = TRUE;
    } else {
        activated = FALSE;
    }

    preDevMode &= ~VP880_DEV_MODE_CHAN_MASK;

    if (channelId == 0 && activated == TRUE) {
        /* If channel 0 was activated, select channel 0 */
        preDevMode |= VP880_DEV_MODE_CHAN0_SEL;

    } else if (channelId == 0 && activated == FALSE) {
        /* If channel 0 was deactivated, select channel 1 */
        preDevMode |= VP880_DEV_MODE_CHAN1_SEL;

    } else if (channelId == 1 && activated == TRUE) {
        /* If channel 1 was activated, select channel 1 */
        preDevMode |= VP880_DEV_MODE_CHAN1_SEL;

    } else if (channelId == 1 && activated == FALSE) {
        /* If channel 1 was deactivated, select channel 0 */
        preDevMode |= VP880_DEV_MODE_CHAN0_SEL;
    }
    if ((devWrite == TRUE) && (preDevMode != pDevObj->devMode[0])) {
        pDevObj->devMode[0] = preDevMode;
        VpMpiCmdWrapper(deviceId, (ecVal | pDevObj->ecVal), VP880_DEV_MODE_WRT,
            VP880_DEV_MODE_LEN, pDevObj->devMode);
    }
    VP_API_FUNC_INT(None, VP_NULL, ("-Vp880UpdateBufferChanSel()"));
}   /* Vp880UpdateBufferChanSel()   */
#endif
