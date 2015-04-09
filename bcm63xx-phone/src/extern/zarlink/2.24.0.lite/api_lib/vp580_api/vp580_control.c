/** \file vp580_control.c
 * vp580_control.c
 *
 *  This file contains the control functions for the Vp580 device API.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 9215 $
 * $LastChangedDate: 2011-12-06 10:19:05 -0600 (Tue, 06 Dec 2011) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_580_SERIES)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_hal.h"
#include "vp_api_int.h"
#include "vp580_api.h"
#include "vp580_api_int.h"
#include "sys_service.h"

/**< Function called by control functions to abstract TX/RX PCM enable bits from
 * line state and option set for Talk state mode.
 */
static VpStatusType
Vp580GetTxRxPcmMode(
    Vp580LineObjectType *pLineObj,
    VpLineStateType state,
    uint8 *mpiByte);

/**< Function called by Set Line State only to retrieve the byte to write to the
 * I/O register for a specific state.
 */
static uint8
Vp580GetLineStateByte(
    VpLineCtxType *pLineCtx,
    VpLineStateType state);

/**< Function called by Set Option only. Implements the options specified by
 * the user. The calling function implements the Device/Line control. If a line
 * option is set and a device option is passed, the calling function will call
 * this function once for each line and pass it the line contexts. Therefore,
 * this function will only be subjected to either a device context and device
 * option, or a line context and a line option.
 */
static VpStatusType
Vp580SetOptionInternal(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *value);

/* Function called by SetOptionInternal for Event Masking only */
static void
Vp580MaskNonSupportedEvents(
    VpOptionEventMaskType *pLineEventsMask,
    VpOptionEventMaskType *pDevEventsMask);

/* Function called by SetOptionInternal for Event Masking only */
static void
Vp580MaskNonSupportedEvents(
    VpOptionEventMaskType *pLineEventsMask,
    VpOptionEventMaskType *pDevEventsMask);

/* Function called by SetOptionInternal to set CODEC mode */
static VpStatusType
Vp580SetCodec(
    VpLineCtxType *pLineCtx,
    VpOptionCodecType codec);

/* Function called by SetOptionInternal to set tx and rx timeslot */
static VpStatusType
Vp580SetTimeSlot(
    VpLineCtxType *pLineCtx,
    uint8 txSlot,
    uint8 rxSlot);

/**< Function called by Api Tick only. Processes the FXS/FXO line passed for Api
 * Tick based operations
 */
static VpStatusType
Vp580ProcessFxsLine(
    Vp580DeviceObjectType *pDevObj,
    VpLineCtxType *pLineCtx);

static VpStatusType
Vp580ProcessFxoLine(
    Vp580DeviceObjectType *pDevObj,
    VpLineCtxType *pLineCtx);

/**
 * Vp580SetLineState()
 *  This function is the API-II wrapper function for Set Line State - Internal
 * for the Vp580 API.
 *
 * Preconditions:
 *  Same as Vp580SetLineStateInt()
 *
 * Postconditions:
 *  Same as Vp580SetLineStateInt()
 */
VpStatusType
Vp580SetLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state)
{
    Vp580LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpStatusType status;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    /*
     * If this line is a custom termination type and has not been configured
     * with a line state map, return error.
     */
    if ((pLineObj->termType == VP_TERM_FXS_CUSTOM)
     && (pLineObj->lineStateInit == FALSE)) {
        return VP_STATUS_CUSTOM_TERM_NOT_CFG;
    }

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Clear the "called from API" flag. This affects the cadencer */
    pLineObj->status &= ~(VP580_SLS_CALL_FROM_API);

    if ((pLineObj->lineState.condition & (VP_CSLAC_HOOK | VP_CSLAC_GKEY))
     && ((state == VP_LINE_RINGING_POLREV) || (state == VP_LINE_RINGING))) {
        /*
         * Go to Ring Trip Exit state instead, which could be ringing -- but
         * that's up to the applicatoin.
         */
        state = pLineObj->ringCtrl.ringTripExitSt;
    }

    status = Vp580SetLineStateInt(pLineCtx, state);
    if (status == VP_STATUS_SUCCESS) {
            pLineObj->lineState.usrCurrent = state;
    }

    /*
     * Set the "called from API" flag. Convenience for API functions so setting
     * this flag does not need to occur in multiple locations
     */
    pLineObj->status |= VP580_SLS_CALL_FROM_API;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return status;
}

/**
 * Vp580SetLineStateInt()
 *  This function sets the line state for a given channel of a given device. The
 * valid line states are defined in the VpLineState type.
 *
 * Preconditions:
 *  The line must first be initialized prior to setting the line state.  The
 * state must be a valid line state as defined in the VpLineState type.
 *
 * Postconditions:
 *  Returns success code if the channel can be set and the line state is valid
 * for the type of line specified by the line context.  If successfull, the line
 * specified by the line context is set to the line state specified.
 */
VpStatusType
Vp580SetLineStateInt(
    VpLineCtxType *pLineCtx,    /**< Line context to change line state on */
    VpLineStateType state)      /**< The desired line state to set */
{
    uint8 userByte, mpiData, mpiByte;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    VpStatusType status;
    VpProfilePtrType pProfile;

    Vp580LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 ecVal[] = {VP580_EC_CH1, VP580_EC_CH2, VP580_EC_CH3, VP580_EC_CH4};
    uint8 channelId = pLineObj->channelId;

    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpLineStateType currentState = pLineObj->lineState.currentState;

    /*
     * Read the status of the Operating Conditions register so we can change
     * only the TX and RX if the line state is a non-communication mode.
     * This also performs the line type/state verification.
     */
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_OP_COND_RD, VP580_OP_COND_LEN,
        &mpiData);
    mpiData &= (uint8)(~(VP580_CUT_TXPATH | VP580_CUT_RXPATH));

    status = Vp580GetTxRxPcmMode(pLineObj, state, &mpiByte);
    if (status == VP_STATUS_SUCCESS) {
        mpiData |= mpiByte;
    } else {
        return status;
    }

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_OP_COND_WRT, VP580_OP_COND_LEN,
        &mpiData);

    /* We're no longer in the middle of a time function */
    pLineObj->cadence.status &= ~VP_CADENCE_STATUS_MID_TIMER;
    pLineObj->cadence.timeRemain = 0;

    /*
     * If this function is called by the application, stop the cadencer and
     * reset the Loop Supervision if it is incorrect
     */

    if (!(pLineObj->status & VP580_SLS_CALL_FROM_API)) {
        /* If we're in the middle of active cadence, terminate it */
        pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;
        pLineObj->cadence.pActiveCadence = VP_PTABLE_NULL;
    }

    /* FXO TYPE LINE HANDLING */
    if (pLineObj->status & VP580_IS_FXO) {
        /*
         * FXO is straightforward, just set as defined by termination type since
         * we already know it's not an unsupported state (except error maybe)
         */

        /* Activate the channel in case not already activated */
        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_ACTIVATE_CMD,
            NO_DATA, &userByte);

        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODATA_REG_RD,
            VP580_IODATA_REG_LEN, &userByte);

        switch(state) {
            case VP_LINE_FXO_LOOP_OPEN:
            case VP_LINE_FXO_OHT:
                userByte |= VP580_IODATA_C3;
                break;

            case VP_LINE_FXO_TALK:
            case VP_LINE_FXO_LOOP_CLOSE:
                userByte &= ~VP580_IODATA_C3;
                break;

            default:
                /* This should be redundant from TX/RX PCM code section above */
                return VP_STATUS_INVALID_ARG;
        }

        /* Perform the FXO Line State change */
        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODATA_REG_WRT,
            VP580_IODATA_REG_LEN, &userByte);
    } else {  /* FXS Handling */
        userByte = Vp580GetLineStateByte(pLineCtx, state);
        if ((state == VP_LINE_RINGING) || (state == VP_LINE_RINGING_POLREV)) {
            pLineObj->cadence.pActiveCadence = pLineObj->pRingingCadence;
            /*
             * We're entering a ringing state, so determine if we need to
             * cadence. If we're not cadencing, this is "always on", so we can
             * disable the currently active cadence sequence and immediately
             * implement the ringing state change.
             */
            pProfile = pLineObj->cadence.pActiveCadence;
            if (pProfile == VP_PTABLE_NULL) {
                /* Always on - no need to cadence */
                pLineObj->cadence.status = VP_CADENCE_RESET_VALUE;
            } else {
                /*
                 * We have a non-null cadence. If the cadence was not previously
                 * started, we'll start it here and let the sequencer take over.
                 * Otherwise, it was previously started and this state change is
                 * at the request of the sequencer.
                 */
                if (!(pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE)) {
                    /* We have a cadence and are just starting it */
                    pLineObj->cadence.status |= VP_CADENCE_STATUS_ACTIVE;
                    pLineObj->cadence.index = VP_PROFILE_TYPE_SEQUENCER_START;
                    pLineObj->cadence.pCurrentPos =
                        &pProfile[VP_PROFILE_TYPE_SEQUENCER_START];
                    pLineObj->cadence.length = pProfile[VP_PROFILE_LENGTH];
                    return VP_STATUS_SUCCESS;
                }
            }
            /* Cadencing already called or null cadence. */
            /*
             * If we're in an active Ringing Cadence, and ready to go into the
             * Ringing state, generate the Ringing Event and indicate that this
             * is the Ringing On event
             */
            if (pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) {
                if (pProfile[VP_PROFILE_TYPE_LSB] == VP_PRFWZ_PROFILE_RINGCAD) {
                    pLineObj->lineEvents.process |= VP_LINE_EVID_RING_CAD;
                    pLineObj->processData = VP_RING_CAD_MAKE;
                }
            }

            /* Start the external ringing signal generator (if any). */
            Vp580RingSigGen(pLineCtx, VP580_RING_SIG_GEN_START, &userByte);
        }

        /*
         * If we are exiting ringing and still on-hook at the time, set the
         * Ringing exit timer
         */
        switch (pLineObj->lineState.currentState) {
            case VP_LINE_RINGING_POLREV:
            case VP_LINE_RINGING:
                switch(state) {
                    case VP_LINE_RINGING_POLREV:
                    case VP_LINE_RINGING:
                        break;

                    default:
                        /* Stop the external ringing signal generator (if any). */
                        Vp580RingSigGen(pLineCtx, VP580_RING_SIG_GEN_STOP, &userByte);

                        if (!(pLineObj->lineState.condition & VP_CSLAC_HOOK)) {
                            pLineObj->lineTimers.timers.timer[VP_LINE_RING_EXIT_PROCESS] =
                                MS_TO_TICKRATE(pLineObj->ringCtrl.ringExitDbncDur / 8,
                                    pDevObj->devProfileData.tickRate);

                            if (pLineObj->ringCtrl.ringExitDbncDur) {
                                pLineObj->lineTimers.timers.timer[VP_LINE_RING_EXIT_PROCESS]
                                    |= VP_ACTIVATE_TIMER;
                            }
                        }

                        /*
                         * If we're in an active Ringing Cadence, and ready to
                         * go into another state, generate the Ringing Event
                         * and indicate that this is the Ringing Off event
                         */
                        if (pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) {
                            pProfile = pLineObj->cadence.pActiveCadence;
                            if (pProfile[VP_PROFILE_TYPE_LSB] ==
                                 VP_PRFWZ_PROFILE_RINGCAD) {
                                pLineObj->lineEvents.process
                                    |= VP_LINE_EVID_RING_CAD;
                                pLineObj->processData = VP_RING_CAD_BREAK;
                            }
                        }
                        break;
                }
                break;

            default:
                break;
        }

        /* Perform the SLIC state change */
        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_SLIC_STATE_WRT,
           VP580_SLIC_STATE_LEN, &userByte);
    }

    /*
     * Update the line object previous and current line state variables. Note
     * that this is not updated with ringing cadence until the line is actually
     * set to ringing (i.e., not when the application sets the line to ringing
     * with (possibly) a non-ringing state via the ringing cadence.
     */
    pLineObj->lineState.previous = currentState;
    pLineObj->lineState.currentState = state;

    return VP_STATUS_SUCCESS;
}

/**
 * Vp580RingSigGen()
 *  This function supports external ringing signal generator control using a
 * GPIO pin.  Its behavior depends on the method chosen in the Ring Profile.
 *
 * Currently, there is only one method defined, VP580_RING_CD2_3STATE.
 * In this method, the CD2 pin is used to drive an RC circuit that generates
 * a coarse approximation of a sine wave.  To increase smoothness of the wave,
 * the CD2 pin is swich from high, to tri-state, to low, to tri-state, and back
 * to high again.
 *
 * This function is called from four different places.  The 'control' parameter
 * specifies which place the function was called from:
 *   VP580_RING_SIG_GEN_INIT:     called in VpConfigLine() if a non-NULL Ring
 *                                Profile is provided
 *   VP580_RING_SIG_GEN_START:    called at the beginning of each 'on' interval
 *                                in a ring cadence
 *   VP580_RING_SIG_GEN_STOP:     called at the beginning of each 'off' interval
 *                                in a ring cadence
 *   VP580_RING_SIG_GEN_UPDATE:   called periodically, with the frequency
 *                                specified in the Ring Profile
 *   VP580_RING_SIG_GEN_SHUTDOWN: called in VpConfigLine(), before changing to a
 *                                new Ring Profile
 *
 * Preconditions:
 *  The line must first be initialized with a valid Ring Profile.  This
 * function should be called from within an MPI critical section.
 *
 * Postconditions:
 *  No return value, but updates the ioDataReg and ioDirReg fields in the Line
 * Object after updating the corresponding registers in the device.
 */
void
Vp580RingSigGen(
    VpLineCtxType *pLineCtx,
    Vp580RingSigGenControlType control,
    uint8 *pUserByte)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp580LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 channelId = pLineObj->channelId;
    uint8 ecVal[] = {VP580_EC_CH1, VP580_EC_CH2, VP580_EC_CH3, VP580_EC_CH4};
    uint16 timerMs = pLineObj->ringParams.timerMs;
    uint8 ioDirReg = pLineObj->ioDirReg;
    uint8 ioDataReg = pLineObj->ioDataReg;

    if ((pLineObj->ringParams.method != VP580_RING_METHOD_CD2_3STATE) &&
        (pLineObj->ringParams.method != VP580_RING_METHOD_CD2_SQUARE)) {
        /* Unknown signal generation method, or disabled. */
        return;
    }

    switch (control) {
        case VP580_RING_SIG_GEN_INIT: {

            /* Configure CD2 as an output. */
            VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODIR_REG_RD,
                VP580_IODIR_REG_LEN, &ioDirReg);
            ioDirReg |= VP580_IODIR_IO2_MASK;
            VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODIR_REG_WRT,
                VP580_IODIR_REG_LEN, &ioDirReg);

            /* Start the timer for toggling CD2. */
            pLineObj->lineTimers.timers.timer[VP_LINE_GPIO_CLKOUT_TIMER] = VP_ACTIVATE_TIMER |
                MS_TO_TICKRATE(timerMs, pDevObj->devProfileData.tickRate);
            break;
        }

        case VP580_RING_SIG_GEN_START:
        case VP580_RING_SIG_GEN_STOP:
            /* Do not disturb the CD2 value in Vp580SetLineStateInt(). */
            VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODATA_REG_RD,
                VP580_IODATA_REG_LEN, &ioDataReg);
            *pUserByte &= ~VP580_IODATA_CD2;
            *pUserByte |= (ioDataReg & VP580_IODATA_CD2);
            break;

        case VP580_RING_SIG_GEN_UPDATE: {

            VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODIR_REG_RD,
                VP580_IODIR_REG_LEN, &ioDirReg);
            VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODATA_REG_RD,
                VP580_IODATA_REG_LEN, &ioDataReg);


            switch (pLineObj->ringParams.state % 4) {
                case 0: /* low */
                    ioDataReg &= ~VP580_IODATA_CD2;
                    ioDirReg |= VP580_IODIR_IO2_MASK;
                    if (pLineObj->ringParams.method == VP580_RING_METHOD_CD2_3STATE) {
                        timerMs -= 10;
                    }
                    break;
                case 1: /* tri-state */
                    ioDataReg &= ~VP580_IODATA_CD2;
                    ioDirReg &= ~VP580_IODIR_IO2_MASK;
                    timerMs = 10;
                    break;
                case 2: /* high  */
                    ioDataReg |= VP580_IODATA_CD2;
                    ioDirReg |= VP580_IODIR_IO2_MASK;
                    if (pLineObj->ringParams.method == VP580_RING_METHOD_CD2_3STATE) {
                        timerMs -= 10;
                    }
                    break;
                default: /* tri-state */
                    ioDataReg |= VP580_IODATA_CD2;
                    ioDirReg &= ~VP580_IODIR_IO2_MASK;
                    timerMs = 10;
                    break;
            }

            VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODIR_REG_WRT,
                VP580_IODIR_REG_LEN, &ioDirReg);
            VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODATA_REG_WRT,
                VP580_IODATA_REG_LEN, &ioDataReg);

            switch (pLineObj->ringParams.method) {
                case VP580_RING_METHOD_CD2_3STATE:
                    pLineObj->ringParams.state += 1;
                    break;
                case VP580_RING_METHOD_CD2_SQUARE:
                    pLineObj->ringParams.state += 2;
                    break;
                default:
                    break;
            }

            /* Restart the timer for the next update interval. */
            pLineObj->lineTimers.timers.timer[VP_LINE_GPIO_CLKOUT_TIMER] = VP_ACTIVATE_TIMER |
                MS_TO_TICKRATE(timerMs, pDevObj->devProfileData.tickRate);
            break;
        }

        case VP580_RING_SIG_GEN_SHUTDOWN:

            /* Configure CD2 as an input. */
            VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODIR_REG_RD,
                VP580_IODIR_REG_LEN, &ioDirReg);
            ioDirReg &= ~VP580_IODIR_IO2_MASK;
            VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODIR_REG_WRT,
                VP580_IODIR_REG_LEN, &ioDirReg);

            /* Stop the timer for toggling CD2. */
            pLineObj->lineTimers.timers.timer[VP_LINE_GPIO_CLKOUT_TIMER] &= ~VP_ACTIVATE_TIMER;
            break;

        default:
            /* Invalid. */
            break;
    }

    pLineObj->ioDirReg = ioDirReg;
    pLineObj->ioDataReg = ioDataReg;
}

/**
 * Vp580GetLineStateByte
 *  Locally used function by Vp580SetLineState to get the line state byte used
 * for the line (depend on termination type and configuration).
 *
 * Preconditions:
 *  None. State to byte mapping only.
 *
 * Postconditions:
 *  Returns the byte that should be used in the I/O data register for the API
 * State passed.
 */
uint8
Vp580GetLineStateByte(
    VpLineCtxType *pLineCtx,
    VpLineStateType state)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp580LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 ecVal[] = {VP580_EC_CH1, VP580_EC_CH2, VP580_EC_CH3, VP580_EC_CH4};
    uint8 channelId = pLineObj->channelId;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 currentByte[VP580_SYS_STATE_LEN];
    uint8 data;

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_SYS_STATE_RD,
        VP580_SYS_STATE_LEN, currentByte);

    switch(pLineObj->termType) {
        case VP_TERM_FXS_CUSTOM:
            switch(state) {
                case VP_LINE_STANDBY:
                    if (pLineObj->lineStateExist & (0x0001 << VP_PRFWZ_CUSTOM_ST_STANDBY)) {
                        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_DEACTIVATE_CMD,
                            NO_DATA, &data);
                        return pLineObj->lineStateBytes[VP_PRFWZ_CUSTOM_ST_STANDBY];
                    } else {
                        return currentByte[0];
                    }
                    break;

                case VP_LINE_TIP_OPEN:
                    if (pLineObj->lineStateExist & (0x0001 << VP_PRFWZ_CUSTOM_ST_TIP_OPEN)) {
                        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_DEACTIVATE_CMD,
                            NO_DATA, &data);
                        return pLineObj->lineStateBytes[VP_PRFWZ_CUSTOM_ST_TIP_OPEN];
                    } else {
                        return currentByte[0];
                    }
                    break;

                case VP_LINE_ACTIVE:
                case VP_LINE_TALK:
                    if (pLineObj->lineStateExist & (0x0001 << VP_PRFWZ_CUSTOM_ST_ACTIVE)) {
                        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_ACTIVATE_CMD,
                            NO_DATA, &data);
                        return pLineObj->lineStateBytes[VP_PRFWZ_CUSTOM_ST_ACTIVE];
                    } else {
                        return currentByte[0];
                    }
                    break;

                case VP_LINE_OHT:
                    if (pLineObj->lineStateExist & (0x0001 << VP_PRFWZ_CUSTOM_ST_OHT)) {
                        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_ACTIVATE_CMD,
                            NO_DATA, &data);
                        return pLineObj->lineStateBytes[VP_PRFWZ_CUSTOM_ST_OHT];
                    } else {
                        return currentByte[0];
                    }
                    break;

                case VP_LINE_ACTIVE_POLREV:
                case VP_LINE_TALK_POLREV:
                    if (pLineObj->lineStateExist & (0x0001 << VP_PRFWZ_CUSTOM_ST_ACTIVE_PR)) {
                        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_ACTIVATE_CMD,
                            NO_DATA, &data);
                        return pLineObj->lineStateBytes[VP_PRFWZ_CUSTOM_ST_ACTIVE_PR];
                    } else {
                        return currentByte[0];
                    }
                    break;

                case VP_LINE_OHT_POLREV:
                    if (pLineObj->lineStateExist & (0x0001 << VP_PRFWZ_CUSTOM_ST_OHT_PR)) {
                        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_ACTIVATE_CMD,
                            NO_DATA, &data);
                        return pLineObj->lineStateBytes[VP_PRFWZ_CUSTOM_ST_OHT_PR];
                    } else {
                        return currentByte[0];
                    }
                    break;

                case VP_LINE_DISCONNECT:
                    if (pLineObj->lineStateExist & (0x0001 << VP_PRFWZ_CUSTOM_ST_DISCONNECT)) {
                        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_DEACTIVATE_CMD,
                            NO_DATA, &data);
                        return pLineObj->lineStateBytes[VP_PRFWZ_CUSTOM_ST_DISCONNECT];
                    } else {
                        return currentByte[0];
                    }
                    break;

                case VP_LINE_RINGING:
                case VP_LINE_RINGING_POLREV:
                    if (pLineObj->lineStateExist & (0x0001 << VP_PRFWZ_CUSTOM_ST_RINGING)) {
                        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_DEACTIVATE_CMD,
                            NO_DATA, &data);
                        return pLineObj->lineStateBytes[VP_PRFWZ_CUSTOM_ST_RINGING];
                    } else {
                        return currentByte[0];
                    }
                    break;

                default:
                    return currentByte[0];
            }
            break;

        default:
            return currentByte[0];
    }
}

/**
 * Vp580GetTxRxPcmMode()
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
Vp580GetTxRxPcmMode(
    Vp580LineObjectType *pLineObj,
    VpLineStateType state,  /**< The state associating with PCM mode */
    uint8 *mpiByte) /**< Device Specific byte */
{
    /*
     * Based on the structure VpOptionPcmTxRxCntrlType:
     * VP_OPTION_PCM_BOTH = 0,
     * VP_OPTION_PCM_RX_ONLY = 1,
     * VP_OPTION_PCM_TX_ONLY = 2
     */
    uint8 pcmEnBits[] = {0x00, VP580_CUT_TXPATH, VP580_CUT_RXPATH};
    *mpiByte = pcmEnBits[pLineObj->pcmTxRxCtrl];

    switch(state) {
        /* Non-Talk States */
        case VP_LINE_STANDBY:
        case VP_LINE_TIP_OPEN:
        case VP_LINE_ACTIVE:
        case VP_LINE_ACTIVE_POLREV:
        case VP_LINE_DISCONNECT:
        case VP_LINE_RINGING:
        case VP_LINE_RINGING_POLREV:
            if (pLineObj->status & VP580_IS_FXO) {
                return VP_STATUS_INVALID_ARG;
            }
            *mpiByte |= (VP580_CUT_TXPATH | VP580_CUT_RXPATH);
            break;

        case VP_LINE_FXO_LOOP_OPEN:
        case VP_LINE_FXO_LOOP_CLOSE:
        case VP_LINE_FXO_RING_GND:
            if (!(pLineObj->status & VP580_IS_FXO)) {
                return VP_STATUS_INVALID_ARG;
            }
            *mpiByte |= (VP580_CUT_TXPATH | VP580_CUT_RXPATH);
            break;

        /* Talk States */
        case VP_LINE_TALK:
        case VP_LINE_TALK_POLREV:
        case VP_LINE_OHT:
        case VP_LINE_OHT_POLREV:
            if (pLineObj->status & VP580_IS_FXO) {
                return VP_STATUS_INVALID_ARG;
            }
            break;

        case VP_LINE_FXO_OHT:
        case VP_LINE_FXO_TALK:
            if (!(pLineObj->status & VP580_IS_FXO)) {
                return VP_STATUS_INVALID_ARG;
            }
            break;

        default:
            break;
    }
    return VP_STATUS_SUCCESS;
}

/**
 * Vp580SetOption()
 *  This function determines how to process the Option based on pDevCtx,
 * pLineCtx, and option type.  The actual options are implemented in
 * Vp580SetOptionInternal
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
Vp580SetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *value)
{
    uint8 channelId;
    Vp580DeviceObjectType *pDevObj;
    VpStatusType status = VP_STATUS_INVALID_ARG;

    VpDevCtxType *pDevCtxLocal;
    VpLineCtxType *pLineCtxLocal;
    VpDeviceIdType deviceId;

    if (pDevCtx != VP_NULL) {
        pDevObj = pDevCtx->pDevObj;
        deviceId = pDevObj->deviceId;

        if (option != VP_OPTION_ID_DEBUG_SELECT) {
            /* Proceed if device state is either in progress or complete */
            if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
            } else {
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
                Vp580SetOptionInternal(VP_NULL, pDevCtx, option, value);

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
                /*
                 * Loop through all of the valid channels associated with this
                 * device. Init status variable in case there are currently no
                 * line contexts associated with this device
                 */
                status = VP_STATUS_SUCCESS;
                for (channelId = 0;
                     channelId < pDevObj->staticInfo.maxChannels;
                     channelId++) {
                    pLineCtxLocal = pDevCtx->pLineCtx[channelId];
                    if (pLineCtxLocal != VP_NULL) {
                        status = Vp580SetOptionInternal(pLineCtxLocal, VP_NULL,
                            option, value);

                        if (status != VP_STATUS_SUCCESS) {
                            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                            return status;
                        }
                    }
                }
                break;

            default:
                /*
                 * Device option, or option unknown option.  Handle in lower
                 * layer
                 */
                status = Vp580SetOptionInternal(VP_NULL, pDevCtx, option, value);
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
            if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
            } else {
                return VP_STATUS_DEV_NOT_INITIALIZED;
            }
        }

        VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
        status = Vp580SetOptionInternal(pLineCtx, VP_NULL, option, value);
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return status;
}

/**
 * Vp580SetOptionInternal()
 *  This function implements on the Vp580 device the options specified from
 * Vp580SetOption().  No other function should call this function.
 *
 * Preconditions:
 *  See Vp580SetOption()
 *
 * Postconditions:
 *  See Vp580SetOption()
 */
VpStatusType
Vp580SetOptionInternal(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *value)
{
    VpDevCtxType *pDevCtxLocal;

    VpStatusType status = VP_STATUS_SUCCESS;

    Vp580LineObjectType *pLineObj;
    Vp580DeviceObjectType *pDevObj;

    VpDeviceIdType deviceId;

    VpOptionDeviceIoType deviceIo;

    uint8 maxChan, mpiData, mpiByte, channelId, txSlot, rxSlot;
    uint8 tempLoopBack[VP580_LOOPBACK_LEN];
    uint8 ecVal[] = {VP580_EC_CH1, VP580_EC_CH2, VP580_EC_CH3, VP580_EC_CH4};
    uint8 ioDirection[4];   /* One to represent each channel */
    uint8 dirCtrlMask;

    VpOptionEventMaskType *pEventsMask, *pNewEventsMask;

    if (pLineCtx != VP_NULL) {
        pDevCtxLocal = pLineCtx->pDevCtx;
        pDevObj = pDevCtxLocal->pDevObj;
        deviceId = pDevObj->deviceId;
        pLineObj = pLineCtx->pLineObj;
        channelId = pLineObj->channelId;

        switch (option) {
            /* Line Options */
            case VP_OPTION_ID_PULSE_MODE:
                pLineObj->pulseMode = *((VpOptionPulseModeType *)value);
                if (pLineObj->lineState.condition & VP_CSLAC_HOOK) {
                    pLineObj->dpStruct.hookSt = TRUE;
                } else {
                    pLineObj->dpStruct.hookSt = FALSE;
                }
                VpInitDP(&pLineObj->dpStruct);
                break;

            case VP_OPTION_ID_TIMESLOT:
                txSlot = ((VpOptionTimeslotType *)value)->tx;
                rxSlot = ((VpOptionTimeslotType *)value)->rx;
                status = Vp580SetTimeSlot(pLineCtx, txSlot, rxSlot);
                break;

            case VP_OPTION_ID_CODEC:
                status = Vp580SetCodec(pLineCtx, *((VpOptionCodecType *)value));
                break;

            case VP_OPTION_ID_PCM_HWY:
                if (*((VpOptionPcmHwyType *)value) != VP_OPTION_HWY_A) {
                    return VP_STATUS_INVALID_ARG;
                }
                break;

            case VP_OPTION_ID_LOOPBACK:
                /* Timeslot loopback via loopback register */
                VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_LOOPBACK_RD,
                    VP580_LOOPBACK_LEN, tempLoopBack);

                switch(*((VpOptionLoopbackType *)value)) {
                    case VP_OPTION_LB_TIMESLOT:
                        tempLoopBack[0] |= VP580_INTERFACE_LOOPBACK_EN;
                        break;

                    case VP_OPTION_LB_OFF:
                        tempLoopBack[0] &= ~(VP580_INTERFACE_LOOPBACK_EN);
                        tempLoopBack[0] &= ~(VP580_FULL_DIGL_LOOPBACK_EN);
                        break;

                    case VP_OPTION_LB_DIGITAL:
                        tempLoopBack[0] |= VP580_FULL_DIGL_LOOPBACK_EN;
                        break;

                    default:
                        return VP_STATUS_INVALID_ARG;
                }

                VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_LOOPBACK_WRT,
                    VP580_LOOPBACK_LEN, tempLoopBack);
                break;

            case VP_OPTION_ID_EVENT_MASK:
                pNewEventsMask = (VpOptionEventMaskType *)value;

                /* Zero out the line-specific bits before setting the
                 * deviceEventsMask in the device object. */
                pEventsMask = &pDevObj->deviceEventsMask;
                pEventsMask->faults =
                    pNewEventsMask->faults & VP_EVCAT_FAULT_DEV_EVENTS;
                pEventsMask->signaling =
                    pNewEventsMask->signaling & VP_EVCAT_SIGNALING_DEV_EVENTS;
                pEventsMask->response =
                    pNewEventsMask->response & VP_EVCAT_RESPONSE_DEV_EVENTS;
                pEventsMask->test =
                    pNewEventsMask->test & VP_EVCAT_TEST_DEV_EVENTS;
                pEventsMask->process =
                    pNewEventsMask->process & VP_EVCAT_PROCESS_DEV_EVENTS;
                pEventsMask->fxo =
                    pNewEventsMask->fxo & VP_EVCAT_FXO_DEV_EVENTS;

                /* Zero out the device-specific bits before setting the
                 * lineEventsMask in the line object. */
                pEventsMask = &pLineObj->lineEventsMask;
                pEventsMask->faults =
                    pNewEventsMask->faults & ~VP_EVCAT_FAULT_DEV_EVENTS;
                pEventsMask->signaling =
                    pNewEventsMask->signaling & ~VP_EVCAT_SIGNALING_DEV_EVENTS;
                pEventsMask->response =
                    pNewEventsMask->response & ~VP_EVCAT_RESPONSE_DEV_EVENTS;
                pEventsMask->test =
                    pNewEventsMask->test & ~VP_EVCAT_TEST_DEV_EVENTS;
                pEventsMask->process =
                    pNewEventsMask->process & ~VP_EVCAT_PROCESS_DEV_EVENTS;
                pEventsMask->fxo =
                    pNewEventsMask->fxo & ~VP_EVCAT_FXO_DEV_EVENTS;

                /* Unmask the unmaskable */
                VpImplementNonMaskEvents(&pLineObj->lineEventsMask,
                    &pDevObj->deviceEventsMask);

                /* Mask those events that the VP580 API-II cannot generate */
                Vp580MaskNonSupportedEvents(&pLineObj->lineEventsMask,
                    &pDevObj->deviceEventsMask);

                if (!(pLineObj->status & VP580_IS_FXO)) {  /* Line is FXS */
                    /* Mask off the FXO events */
                    pLineObj->lineEventsMask.fxo |= VP_EVCAT_FXO_MASK_ALL;
                } else {  /* Line is FXO */
                    /* Mask off the FXS events */
                    pLineObj->lineEventsMask.signaling
                        |= VP580_FXS_SIGNALING_EVENTS;
                }
                break;

            case VP_OPTION_ID_RING_CNTRL:
                pLineObj->ringCtrl = *((VpOptionRingControlType *)value);
                break;

            case VP_OPTION_ID_PCM_TXRX_CNTRL:
                pLineObj->pcmTxRxCtrl = *((VpOptionPcmTxRxCntrlType *)value);
                VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_OP_COND_RD,
                    VP580_OP_COND_LEN, &mpiData);
                mpiData &= (uint8)(~(VP580_CUT_TXPATH | VP580_CUT_RXPATH));
                Vp580GetTxRxPcmMode(pLineObj, pLineObj->lineState.currentState,
                    &mpiByte);
                mpiData |= mpiByte;
                VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_OP_COND_WRT,
                    VP580_OP_COND_LEN, &mpiData);
                break;

#ifdef VP_DEBUG
            case VP_OPTION_ID_DEBUG_SELECT:
                pLineObj->debugSelectMask = *(uint32 *)value;
                break;
#endif

            default:
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
        }
    } else {
        pDevObj = pDevCtx->pDevObj;
        deviceId = pDevObj->deviceId;
        maxChan = pDevObj->staticInfo.maxChannels;

        switch (option) {
            case VP_DEVICE_OPTION_ID_PULSE:
                pDevObj->pulseSpecs = *((VpOptionPulseType *)value);
                break;

            case VP_DEVICE_OPTION_ID_DEVICE_IO:
                deviceIo = *(VpOptionDeviceIoType *)(value);
                /*
                 * Can only support Driven (0 type) output, so do quick error
                 * check. Also, verify that input only I/O are protected to be
                 * set for input only.
                 */
                if ((deviceIo.outputTypePins_31_0)
                 || (deviceIo.directionPins_31_0 & VP580_IODIR_INPUT_ONLY)) {
                    status = VP_STATUS_INVALID_ARG;
                } else {
                    for (channelId = 0; channelId < maxChan; channelId++) {
                        /* Read the current direction pins */
                        VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODIR_REG_RD,
                            VP580_IODIR_REG_LEN, &ioDirection[channelId]);

                        pLineCtx = pDevCtx->pLineCtx[channelId];
                        if (pLineCtx != VP_NULL) {
                            pLineObj = pLineCtx->pLineObj;

                            if (pLineObj->status & VP580_IS_FXO) {
                                dirCtrlMask = 0xFF & ~(VP580_IODIR_IO3_MASK);
                            } else {
                                if (pLineObj->termType == VP_TERM_FXS_CUSTOM) {
                                    dirCtrlMask = ~pLineObj->bitMask;
                                } else {
                                    dirCtrlMask = 0xFF;
                                }
                            }
                            /*
                             * Direction Control Mask is marking '1' in bit
                             * locations that may be modifed due to termination
                             * type
                             */
                            ioDirection[channelId] &= (~dirCtrlMask);
                            ioDirection[channelId] |=
                                deviceIo.directionPins_31_0 >> (8 * channelId);
                            ioDirection[channelId] &= 0xFF;

                            /* Write the new direction values */
                            VpMpiCmdWrapper(deviceId, ecVal[channelId],
                                VP580_IODIR_REG_WRT, VP580_IODIR_REG_LEN,
                                &ioDirection[channelId]);
                        }
                    }
                }
                break;

#ifdef VP_DEBUG
            case VP_OPTION_ID_DEBUG_SELECT:
                pDevObj->debugSelectMask = *(uint32 *)value;
                break;
#endif

            default:
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
                break;
        }
    }

    return status;
}

/**
 * Vp580MaskNonSupportedEvents()
 *  This function masks the events that are not supported by the VP580 API-II.
 * It should only be called by SetOptionInternal when event masks are being
 * modified.
 *
 * Preconditions:
 *  None. Utility function to modify event structures only.
 *
 * Postconditions:
 *  Event structures passed are modified with masked bits for non-supported
 * VP580 API-II events.
 */
void
Vp580MaskNonSupportedEvents(
    VpOptionEventMaskType *pLineEventsMask, /**< Line Events Mask to modify for
                                             * non-masking
                                             */
    VpOptionEventMaskType *pDevEventsMask)  /**< Device Events Mask to modify
                                             * for non-masking
                                             */
{
    pLineEventsMask->faults |= VP580_NONSUPPORT_FAULT_EVENTS;
    pLineEventsMask->signaling |= VP580_NONSUPPORT_SIGNALING_EVENTS;
    pLineEventsMask->response |= VP580_NONSUPPORT_RESPONSE_EVENTS;
    pLineEventsMask->test |= VP580_NONSUPPORT_TEST_EVENTS;
    pLineEventsMask->process |= VP580_NONSUPPORT_PROCESS_EVENTS;
    pLineEventsMask->fxo |= VP580_NONSUPPORT_FXO_EVENTS;

    pDevEventsMask->faults |= VP580_NONSUPPORT_FAULT_EVENTS;
    pDevEventsMask->signaling |= VP580_NONSUPPORT_SIGNALING_EVENTS;
    pDevEventsMask->response |= VP580_NONSUPPORT_RESPONSE_EVENTS;
    pDevEventsMask->test |= VP580_NONSUPPORT_TEST_EVENTS;
    pDevEventsMask->process |= VP580_NONSUPPORT_PROCESS_EVENTS;
    pDevEventsMask->fxo |= VP580_NONSUPPORT_FXO_EVENTS;
    return;
}

/**
 * Vp580DeviceIoAccess()
 *  This function is used to access device IO pins of the Vp580. See API-II
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
Vp580DeviceIoAccess(
    VpDevCtxType *pDevCtx,
    VpDeviceIoAccessDataType *pDeviceIoData)
{
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    VpLineCtxType *pLineCtx;
    Vp580LineObjectType *pLineObj;

    VpDeviceIdType deviceId = pDevObj->deviceId;
    uint8 ecVal[] = {VP580_EC_CH1, VP580_EC_CH2, VP580_EC_CH3, VP580_EC_CH4};
    uint8 chanNum, maxChan;
    uint8 bitMask = 0xFF;   /* Initially, all bits can be modified */

    uint8 ioDataReg[4];

    uint8 numBits = 8;
    uint8 bitValue, bitData, bitLocate, bitCnt;
    uint8 bitMap[] = {
        VP580_IODATA_IO1, VP580_IODATA_IO2, VP580_IODATA_IO3, VP580_IODATA_IO4,
        VP580_IODATA_IO5, VP580_IODATA_IO6, VP580_IODATA_IO7, 0x00
    };

    VpDeviceIoAccessDataType *pAccessData =
        &(pDevObj->getResultsOption.optionData.deviceIoData);

    maxChan = pDevObj->staticInfo.maxChannels;

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    /* Read the current state of the IO lines */
    for (chanNum = 0; chanNum < maxChan; chanNum++) {
        VpMpiCmdWrapper(deviceId, ecVal[chanNum], VP580_IODATA_REG_RD,
            VP580_IODATA_REG_LEN, &ioDataReg[chanNum]);
    }

    *pAccessData = *pDeviceIoData;

    /*
     * If this is a write operation, detect the bits that should be changed
     * and send to appropriate channel
     */
    if (pDeviceIoData->accessType == VP_DEVICE_IO_WRITE) {
        for (chanNum = 0; chanNum < maxChan; chanNum++) {
            pLineCtx = pDevCtx->pLineCtx[chanNum];
            if (pLineCtx != VP_NULL) {
                pLineObj = pLineCtx->pLineObj;
                if (pLineObj->status & VP580_IS_FXO) {
                    /* FXO Line has fixed I/O line use. */
                    bitMask = 0xFF;    /* All bits can be modified */
                } else {
                    /*
                     * If this line is a custom termination type and has not
                     * been configured with a line state map, return error.
                     */
                    if (pLineObj->termType == VP_TERM_FXS_CUSTOM) {
                        if (pLineObj->lineStateInit == FALSE) {
                            VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
                            return VP_STATUS_CUSTOM_TERM_NOT_CFG;
                        } else {
                            /* Get the bits that are not used by any state */
                            bitMask = ~pLineObj->bitMask;
                        }
                    } else {
                        bitMask = 0xFF;    /* All bits can be modified */
                    }

                    /*
                     * Modify the bit mask with the known protected bits and
                     * the input parameters to mark the bits that will change
                     * with '1'
                     */
                    for (bitCnt = 0; bitCnt < numBits; bitCnt++) {
                        bitMask &= ((pAccessData->accessMask_31_0 >> (8 * chanNum)) & 0xFF);
                    }
                }
            } else {
                /* This line context is null, so set bits as requested */
                bitMask = 0xFF;
            }

            /*
             * Update the Data I/O Register for bits to be modified and
             * input parameters (data)
             */
            bitData = ((pAccessData->deviceIOData_31_0 >> (8 * chanNum)) & 0xFF);
            bitValue = 0x00;

            for (bitCnt = 0; bitCnt < numBits; bitCnt++) {
                bitLocate = (0x01 << bitCnt);
                /* This bit (from deviceIOData_31_0) is being written to with a '1' */
                if (bitData & bitLocate & bitMask) {
                    /*
                     * OR in the VP580 bit corresponding to the bit being set
                     * by the parameters passed. Note that the bits in the silicon do
                     * not map 1:1 with the bit locations in deviceIOData_31_0
                     */
                    bitValue |= (bitMap[bitCnt]);
                }
            }

            /*
             * Set all bits to the values (specified by bitValue) that are not
             * dedicated to term type and being requested to change (to '1').
             */
            ioDataReg[chanNum] = bitValue;
            VpMpiCmdWrapper(deviceId, ecVal[chanNum], VP580_IODATA_REG_WRT,
                VP580_IODATA_REG_LEN, &ioDataReg[chanNum]);
        }
    } else {
        /* We're reading values, replace the data passed with the data from the device */

        /* Pre-Clear all bits returned */
        pAccessData->deviceIOData_31_0 = 0;
        pAccessData->deviceIOData_63_32 = 0;

        /*
         * Set bits in the device object based on device content. Note that the silicon is not a
         * 1:1 mapping of deviceIOData_31_0. Some slight translation is needed.
         */
        for (chanNum = 0; chanNum < maxChan; chanNum++) {
            for (bitCnt = 0; bitCnt < numBits; bitCnt++) {
                /*
                 * Set the bit value we'll return IF the corresponding bit via the bitMap is
                 * set in the silicon. Ignore the access mask for now because that's easy to
                 * correct later.
                 */
                bitValue = (0x01 << bitCnt);
                if (ioDataReg[chanNum] & bitMap[bitCnt]) {
                    /*
                     * Shift the bit value up to the location where it maps 1:1 in the
                     * deviceIOData_31_0 and then update deviceIOData_31_0. While doing this,
                     * make sure the compiler is forced to treat the uint8 bitValue as a uin32
                     * in order to prevent dropping data or generating compiler warnings.
                     */
                    pAccessData->deviceIOData_31_0 |= ((uint32)bitValue << (8 * chanNum));
                }
            }
        }
        /*
         * Per the VP-API-II Specification, set all read bits that are NOT being accessed to
         * '0' in the results.
         */
        pAccessData->deviceIOData_31_0 &= pDeviceIoData->accessMask_31_0;
    }

    pDevObj->deviceEvents.response |= VP_DEV_EVID_IO_ACCESS_CMP;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}

/**
 * Vp580SetCodec()
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
Vp580SetCodec(
    VpLineCtxType *pLineCtx,
    VpOptionCodecType codec)    /* Encoding, as defined by LineCodec typedef */
{
    Vp580LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 codecReg;
    uint8 ecVal[] = {VP580_EC_CH1, VP580_EC_CH2, VP580_EC_CH3, VP580_EC_CH4};
    uint8 channelId = pLineObj->channelId;

    /* Basic error checking */
    if ((codec != VP_OPTION_LINEAR) && (codec != VP_OPTION_ALAW)
     && (codec != VP_OPTION_MLAW)) {
        return VP_STATUS_INVALID_ARG;
    }
    /*
     * If the code mode is already correct, don't change it -- it's a waste of
     * the MPI resource
     */

    if (codec == pLineObj->codec) {
        return VP_STATUS_SUCCESS;
    }

    /* Read the current state of the codec register */
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_CODEC_REG_RD,
        VP580_CODEC_REG_LEN, &codecReg);

    /* Enable the desired CODEC mode */
    switch(codec) {
        case VP_OPTION_LINEAR:              /*16 bit linear PCM */
            codecReg |= VP580_LINEAR_CODEC;
            break;

        case VP_OPTION_ALAW:                /* A-law PCM */
            codecReg &= (uint8)(~(VP580_LINEAR_CODEC | VP580_ULAW_CODEC));
            break;

        case VP_OPTION_MLAW:                /* u-law PCM */
            codecReg |= VP580_ULAW_CODEC;
            codecReg &= ~(VP580_LINEAR_CODEC);
            break;

        default:
            /* Cannot reach here.  Error checking at top */
            break;
    } /* Switch */

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_CODEC_REG_WRT,
        VP580_CODEC_REG_LEN, &codecReg);
    pLineObj->codec = codec;

    return VP_STATUS_SUCCESS;
}

/**
 * Vp580SetTimeSlot()
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
Vp580SetTimeSlot(
    VpLineCtxType *pLineCtx,
    uint8 txSlot,       /**< The TX PCM timeslot */
    uint8 rxSlot)       /**< The RX PCM timeslot */
{
    Vp580LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 ecVal[] = {VP580_EC_CH1, VP580_EC_CH2, VP580_EC_CH3, VP580_EC_CH4};
    uint8 channelId = pLineObj->channelId;

    /* Validate the tx time slot value */
    if(txSlot >= pDevObj->devProfileData.pcmClkRate/64) {
        return VP_STATUS_INPUT_PARAM_OOR;
    }

    /* Validate the rx time slot value */
    if(rxSlot >= pDevObj->devProfileData.pcmClkRate/64) {
        return VP_STATUS_INPUT_PARAM_OOR;
    }

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_TX_TS_WRT,
        VP580_TX_TS_LEN, &txSlot);

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_RX_TS_WRT,
        VP580_RX_TS_LEN, &rxSlot);

    return VP_STATUS_SUCCESS;
}

/**
 * Vp580VirtualISR()
 *  This function is called everytime the device causes an interrupt
 *
 * Preconditions
 *  A device interrupt has just occured
 *
 * Postcondition
 *  This function should be called from the each device's ISR.
 *  This function could be inlined to improve ISR performance.
 */
#ifndef VP580_SIMPLE_POLLED_MODE
VpStatusType
Vp580VirtualISR(
    VpDevCtxType *pDevCtx)
{
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

#if defined (VP580_INTERRUPT_LEVTRIG_MODE)
    VpSysDisableInt(deviceId);
#endif
    /* Device Interrupt Received */
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    pDevObj->status.state |= VP_DEV_PENDING_INT;
    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
}
#endif

/**
 * Vp580ApiTick()
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
Vp580ApiTick(
    VpDevCtxType *pDevCtx,
    bool *pEventStatus)
{
    VpLineCtxType *pLineCtx;
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp580LineObjectType *pLineObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint8 ecVal[] = {VP580_EC_CH1, VP580_EC_CH2, VP580_EC_CH3, VP580_EC_CH4};
    uint8 channelId;
    uint8 maxChan = pDevObj->staticInfo.maxChannels;
    bool tempClkFault;
    uint16 timeStampPre, tickAdder;

#ifdef VP_CSLAC_SEQ_EN
    bool isSeqRunning = FALSE;
#endif

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

#if defined (VP580_INTERRUPT_LEVTRIG_MODE)
    VpSysEnableInt(deviceId);
#endif

    /* Ensure that device is initialized */
    if (!(pDevObj->status.state & VP_DEV_INIT_CMP)) {
        if (Vp580FindSoftwareInterrupts(pDevCtx)) {
            *pEventStatus = TRUE;
        }

        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    /* Reset event pointers pointers */
    pDevObj->dynamicInfo.lastChan = 0;

    /* Service API Timers */
    Vp580ServiceTimers(pDevCtx);

#ifdef VP_CSLAC_SEQ_EN
    /* Evaluate if Cadencing is required */
    for (channelId = 0; channelId < maxChan; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            if (pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) {
                isSeqRunning = TRUE;
            }
        }
    }

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

#if defined (VP580_EFFICIENT_POLLED_MODE)
    /* Poll the PIO-INT line */
    pDevObj->status.state |=
        (VpSysTestInt(deviceId) ? VP_DEV_PENDING_INT : 0x00);
#elif defined (VP580_SIMPLE_POLLED_MODE)
    pDevObj->status.state |= VP_DEV_PENDING_INT;
#endif

    /* Check to see if a re-evaluation of the signaling register exists */
    if (pDevObj->stateInt & VP580_FORCE_SIG_READ) {
        pDevObj->status.state |= VP_DEV_PENDING_INT;
    }

    /* Limit the number of interrupts serviced during one tick */
    pDevObj->status.numIntServiced = pDevObj->devProfileData.maxNumInterrupts;

    /* Service all pending interrupts (up to maxNumInterrupts) */
    while ((pDevObj->status.state & VP_DEV_PENDING_INT)
        && (pDevObj->status.numIntServiced > 0)) {
        /* Clear the current interrupt indication */
        pDevObj->status.state &= ~(VP_DEV_PENDING_INT);
        pDevObj->status.numIntServiced--;

        /* Read the Real-Time Data Register for the device */
        VpMpiCmdWrapper(deviceId, ecVal[0], VP580_UL_SIGREG_RD, VP580_UL_SIGREG_LEN,
            &pDevObj->rtdReg);

        /* Read the I/O Data Register for each channel */
        for (channelId = 0; channelId < maxChan; channelId++) {
            pLineCtx = pDevCtx->pLineCtx[channelId];
            if (pLineCtx != VP_NULL) {
                pLineObj = pLineCtx->pLineObj;
                VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODATA_REG_RD,
                    VP580_IODATA_REG_LEN, &pLineObj->ioDataReg);
            }
        }

        /*******************************************************
         *         HANDLE Clock Fail Events                    *
         *******************************************************/

        /* Get the current status of the fault bit */
        VpMpiCmdWrapper(deviceId, ecVal[0], VP580_IODIR_REG_RD, VP580_IODIR_REG_LEN,
            &pDevObj->clkFailReg);

        tempClkFault = (pDevObj->clkFailReg & VP580_CFAIL_MASK) ? TRUE : FALSE;

        /*
         * Compare it with what we already know.  If different, generate
         * events and update the line status bits
         */
        if(tempClkFault != pDevObj->dynamicInfo.clkFault) {
            pDevObj->dynamicInfo.clkFault = tempClkFault;
            pDevObj->deviceEvents.faults |= VP_DEV_EVID_CLK_FLT;
        }

        /* Process remaining line events */
        Vp580ServiceInterrupts(pDevCtx);

        /*
         * If level triggered, the interrupt may have been disabled (to prevent
         * a flood of interrupts), so reenable it.
         */
    #if defined (VP580_INTERRUPT_LEVTRIG_MODE)
        VpSysEnableInt(deviceId);
    #endif

    /*
     * If operating in Efficient Polled Mode, check to see if the interrupt
     * line is still indicating an active interrupt. If in simple polled mode,
     * repeat the loop and service interrupts (if anything is changed).
     */
    #if defined (VP580_EFFICIENT_POLLED_MODE)
        /* Poll the PIO-INT line */
        pDevObj->status.state |=
            (VpSysTestInt(deviceId) ? VP_DEV_PENDING_INT : 0x00);
    #elif defined (VP580_SIMPLE_POLLED_MODE)
        pDevObj->status.state |= VP_DEV_PENDING_INT;
    #endif
    }/* End while Interrupts*/

    /* Update the dial pulse handler for lines that are set for pulse decode */
    for (channelId = 0; channelId < maxChan; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            if (pLineObj->status & VP580_INIT_COMPLETE) {
                if (!(pLineObj->status & VP580_IS_FXO)) {
                    Vp580ProcessFxsLine(pDevObj, pLineCtx);
                } else {
                    Vp580ProcessFxoLine(pDevObj, pLineCtx);
                }
            }
        }
    }

    /* Collect all event activity and report to the calling function */
    if (Vp580FindSoftwareInterrupts(pDevCtx)) {
        *pEventStatus = TRUE;
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}

/**
 * Vp580ProcessFxsLine()
 *  This function should only be called by Vp580ApiTick on FXS lines. It
 * performs all line processing for operations that are Tick based
 *
 * Preconditions:
 *  Conditions defined by purpose of Api Tick.
 *
 * Postconditions:
 *  The Api variables and events (as appropriate) for the line passed have been
 * updated.
 */
VpStatusType
Vp580ProcessFxsLine(
    Vp580DeviceObjectType *pDevObj,
    VpLineCtxType *pLineCtx)
{
    Vp580LineObjectType *pLineObj = pLineCtx->pLineObj;

    /*
     * If the line is configured for Dial Pulse Detection, run the Dial Pulse
     * detection code. Dial Pulse detection code will generate the appropriate
     * events
     */
    if(pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_ON) {
        if(VpUpdateDP(pDevObj->devProfileData.tickRate, &pDevObj->pulseSpecs,
            &pLineObj->dpStruct, &pLineObj->lineEvents) == TRUE) {
            pLineObj->signalingData = pLineObj->dpStruct.signalingData;
            pLineObj->lineEventHandle = pDevObj->timeStamp;
        }
    }

    return VP_STATUS_SUCCESS;
}

/**
 * Vp580ProcessFxoLine()
 *  This function should only be called by Vp580ApiTick on FXO lines. It
 * performs all line processing for operations that are Tick based
 *
 * Preconditions:
 *  Conditions defined by purpose of Api Tick.
 *
 * Postconditions:
 *  The Api variables and events (as appropriate) for the line passed have been
 * updated.
 */
VpStatusType
Vp580ProcessFxoLine(
    Vp580DeviceObjectType *pDevObj,
    VpLineCtxType *pLineCtx)
{
    Vp580LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    uint16 *pHighToLowTime =
        &pLineObj->lineTimers.timers.fxoTimer.highToLowTime;
    uint16 *pPrevHighToLowTime =
        &pLineObj->lineTimers.timers.fxoTimer.prevHighToLowTime;

    uint16 totalTime;
    uint8 ringMinTime = pLineObj->ringDetMin;
    uint8 ringMaxTime = pLineObj->ringDetMax;

    bool *pPrevState = &pLineObj->lineTimers.timers.fxoTimer.lastState;
    bool ringBit;
    uint8 ecVal[] = {VP580_EC_CH1, VP580_EC_CH2, VP580_EC_CH3, VP580_EC_CH4};
    uint8 channelId = pLineObj->channelId;

    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_IODATA_REG_RD,
        VP580_IODATA_REG_LEN, &pLineObj->ioDataReg);
    ringBit = (pLineObj->ioDataReg & VP580_IODATA_C5) ? TRUE : FALSE;

    if (ringBit != *pPrevState) {
        pLineObj->lineTimers.timers.fxoTimer.noCount = FALSE;
        *pPrevState = ringBit;
        if (ringBit == FALSE) {
            *pPrevHighToLowTime = *pHighToLowTime;
            *pHighToLowTime = pDevObj->timeStamp;
        }
    } else {
        /*
         * It hasn't changed, so detect if it hasn't change for a duration that
         * exceeds the minimum ringing frequency period
         */
        if (pLineObj->lineTimers.timers.fxoTimer.noCount == FALSE) {
            if ((pDevObj->timeStamp - *pHighToLowTime) > 150) {
                pLineObj->lineTimers.timers.fxoTimer.noCount = TRUE;
                *pHighToLowTime = *pPrevHighToLowTime;
            }
        }
    }

    totalTime = (*pHighToLowTime - *pPrevHighToLowTime);

    /*
     * Total time is in 0.5mS increments. Compare with 5mS resolution specified
     * in profile
     */
    if ((ringMinTime < totalTime) && (totalTime < ringMaxTime)) {
        pLineObj->ringingDet = TRUE;
    } else {
        pLineObj->ringingDet = FALSE;
    }

    return VP_STATUS_SUCCESS;
}

/**
 * Vp580ServiceInterrupts()
 *  This function should only be called by Vp580ApiTick when an interrupt
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
Vp580ServiceInterrupts(
    VpDevCtxType *pDevCtx)
{
    VpLineCtxType *pLineCtx;
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp580LineObjectType *pLineObj;
    uint8 channelId;

    VpCslacLineCondType tempHookSt, detState, ringingState;
    VpLineStateType state;

    bool retFlag = FALSE;

    for (channelId = 0;
         channelId < pDevObj->staticInfo.maxChannels;
         channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            if (!(pLineObj->status & VP580_IS_FXO)) {
                /* Determine which register to read the detector status. */
                if (pLineObj->detMap & VP580_IODATA_CD1) {
                    detState = (pDevObj->rtdReg & (0x01 << (2 * channelId)));
                } else {
                    detState = pLineObj->ioDataReg & pLineObj->detMap;
                }

                /* If debouncing for Ring Exit ignore hook. */
                if ((pLineObj->lineTimers.timers.timer[VP_LINE_RING_EXIT_PROCESS]
                    & VP_ACTIVATE_TIMER))  {
                    tempHookSt = (VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_HOOK);
                } else {
                    if (detState) {
                        tempHookSt = VP_CSLAC_STATUS_INVALID;
                    } else {
                        tempHookSt = VP_CSLAC_HOOK;
                    }
                }

                /* If the hook conditions changed, continue line processing */
                if((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_HOOK)
                    != tempHookSt) {

                    if (tempHookSt == VP_CSLAC_HOOK) {
                        pLineObj->lineState.condition |= VP_CSLAC_HOOK;
                        pLineObj->dpStruct.hookSt = TRUE;

                        if(pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_OFF) {
                            pLineObj->lineEvents.signaling |=
                                VP_LINE_EVID_HOOK_OFF;
                            pLineObj->lineEventHandle = pDevObj->timeStamp;
                            retFlag = TRUE;
                        }

                        /*
                         * If an off-hook is detected during the user set state
                         * of Ringing (incl. ringing and silent interval), set
                         * state to the user defined ring trip exit state
                         */
                        state = pLineObj->lineState.usrCurrent;
                        if ((state == VP_LINE_RINGING)
                         || (state == VP_LINE_RINGING_POLREV)) {
                            Vp580SetLineState(pLineCtx,
                                pLineObj->ringCtrl.ringTripExitSt);
                        }
                    } else {
                        pLineObj->lineState.condition &= ~(VP_CSLAC_HOOK);
                        pLineObj->dpStruct.hookSt = FALSE;

                        if(pLineObj->pulseMode == VP_OPTION_PULSE_DECODE_OFF) {
                            /*
                             * If this is the first time after initialization
                             * that we are checking for on-hook and it is
                             * on-hook, don't generate an interrupt
                             */
                            if (!(pLineObj->lineState.condition & VP_CSLAC_STATUS_VALID)) {
                                pLineObj->lineEvents.signaling |=
                                    VP_LINE_EVID_HOOK_ON;
                                pLineObj->lineEventHandle = pDevObj->timeStamp;
                                retFlag = TRUE;
                            }
                        }
                    }
                }
            } else {
                ringingState = pLineObj->ringingDet ? VP_CSLAC_RINGING : VP_CSLAC_STATUS_INVALID;

                /* If the ringing conditions changed, report the event */
                if((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_RINGING)
                    != ringingState) {
                    pLineObj->lineState.condition = ringingState;
                    if (ringingState == VP_CSLAC_RINGING) {
                        pLineObj->lineEvents.fxo |= VP_LINE_EVID_RING_ON;
                    } else {
                        pLineObj->lineEvents.fxo |= VP_LINE_EVID_RING_OFF;
                    }
                    pLineObj->fxoData = pDevObj->timeStamp;
                }
            }
        }
    }

    return retFlag;
}

/**
 * Vp580LowLevelCmd()
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
Vp580LowLevelCmd(
    VpLineCtxType *pLineCtx,
    uint8 *pCmdData,
    uint8 len,
    uint16 handle)
{
    Vp580LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp580DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;

    int i;

    uint8 ecVal[] = {VP580_EC_CH1, VP580_EC_CH2, VP580_EC_CH3, VP580_EC_CH4};
    uint8 channelId = pLineObj->channelId;

    if (pDevObj->deviceEvents.response & VP580_READ_RESPONSE_MASK) {
        return VP_STATUS_DEVICE_BUSY;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    if(pCmdData[0] & 0x01) { /* Read Command */
        VpMpiCmdWrapper(deviceId, ecVal[channelId], pCmdData[0], len,
            &(pDevObj->mpiData[0]));
        pDevObj->mpiLen = len;
        pLineObj->lineEvents.response |= VP_LINE_EVID_LLCMD_RX_CMP;
    } else {
        VpMpiCmdWrapper(deviceId, ecVal[channelId], pCmdData[0], len, &pCmdData[1]);
        for (i = 0; i < len; i++) {
            pDevObj->mpiData[i] = pCmdData[i];
        }
        pLineObj->lineEvents.response |= VP_LINE_EVID_LLCMD_TX_CMP;
    }
    pLineObj->lineEventHandle = handle;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    return VP_STATUS_SUCCESS;
}
#endif

/**
 * Vp580SetRelGain
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
VpStatusType
Vp580SetRelGain(
    VpLineCtxType *pLineCtx,    /**< Line context to change gains on */
    uint16 txLevel,             /**< Adjustment to line's relative Tx level */
    uint16 rxLevel,             /**< Adjustment to line's relative Rx level */
    uint16 handle)              /**< Handle value returned with the event */
{
    Vp580LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp580DeviceObjectType *pDevObj = pLineCtx->pDevCtx->pDevObj;
    VpDeviceIdType deviceId = pDevObj->deviceId;
    VpRelGainResultsType *relGainResults = &pDevObj->relGainResults;
    uint8 ecVal[] = {VP580_EC_CH1, VP580_EC_CH2, VP580_EC_CH3, VP580_EC_CH4};
    uint8 channelId = pLineObj->channelId;
    uint32 gxInt, grInt;
    uint8 gainCSD[VP580_GX_GAIN_LEN];

    /* Proceed if device state is either in progress or complete */
    if (pDevObj->status.state & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS)) {
    } else {
        return VP_STATUS_DEV_NOT_INITIALIZED;
    }

    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);

    if (pDevObj->deviceEvents.response & VP580_READ_RESPONSE_MASK) {
        VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
        return VP_STATUS_DEVICE_BUSY;
    }

    relGainResults->gResult = VP_GAIN_SUCCESS;

    /* Multiply the profile gain values by the requested adjustments. */
    gxInt = (uint32)pLineObj->gain.gxInt * txLevel / 16384;
    grInt = (uint32)pLineObj->gain.grInt * rxLevel / 16384;

    /* If overflow or underflow occurred, generate out-of-range result. */
    /* Requirement: 1.0 <= gxInt <= 4.0 */
    if ((gxInt < (uint32)0x4000) || (gxInt > (uint32)0x10000)) {
        VP_GAIN(VpLineCtxType, pLineCtx,
                   ("Vp580SetRelGain(): %u * %cxLevel / 16384 = %u, %s is %u",
                    (unsigned)pLineObj->gain.gxInt, 't', (unsigned)gxInt,
                    (gxInt < (uint32)0x4000) ? "minimum" : "maximum",
                    (gxInt < (uint32)0x4000) ? 0x4000U : 0x10000U));
        relGainResults->gResult |= VP_GAIN_GX_OOR;
        gxInt = pLineObj->gain.gxInt;
    }
    /* Requirement: 0.25 <= grInt <= 1.0 */
    if ((grInt < (uint32)0x1000) || (grInt > (uint32)0x4000)) {
        VP_GAIN(VpLineCtxType, pLineCtx,
                   ("Vp580SetRelGain(): %u * %cxLevel / 16384 = %u, %s is %u",
                    (unsigned)pLineObj->gain.grInt, 'r', (unsigned)grInt,
                    (grInt < (uint32)0x1000) ? "minimum" : "maximum",
                    (gxInt < (uint32)0x1000) ? 0x1000U : 0x4000U));
        relGainResults->gResult |= VP_GAIN_GR_OOR;
        grInt = pLineObj->gain.grInt;
    }

    /*
     * Write adjusted gain values to the device, and remember them for
     * VpGetResults().
     */
    VpConvertFixed2Csd((uint16)(gxInt - 0x4000), gainCSD);
    relGainResults->gxValue = ((uint16)gainCSD[0] << 8) + gainCSD[1];
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_GX_GAIN_WRT, VP580_GX_GAIN_LEN,
        gainCSD);
    VP_GAIN(VpLineCtxType, pLineCtx,
            ("Post Converted gxInt (write to GX_WRT): gainCSD = 0x%02X 0x%02X",
             gainCSD[0], gainCSD[1]));

    VpConvertFixed2Csd((uint16)grInt, gainCSD);
    relGainResults->grValue = ((uint16)gainCSD[0] << 8) + gainCSD[1];
    VpMpiCmdWrapper(deviceId, ecVal[channelId], VP580_GR_GAIN_WRT, VP580_GR_GAIN_LEN,
        gainCSD);
    VP_GAIN(VpLineCtxType, pLineCtx,
            ("Post Converted grInt (write to GR_WRT): gainCSD = 0x%02X 0x%02X",
             gainCSD[0], gainCSD[1]));

    /* Generate the gain-complete event. */
    pLineObj->lineEvents.response |= VP_LINE_EVID_GAIN_CMP;
    pLineObj->lineEventHandle = handle;

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);
    return VP_STATUS_SUCCESS;
}

#endif




