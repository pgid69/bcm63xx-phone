/** \file vp886_query.c
 * vp886_query.c
 *
 *  This file contains the query functions used in the Vp886 series device API.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11607 $
 * $LastChangedDate: 2014-10-20 15:31:02 -0500 (Mon, 20 Oct 2014) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_886_SERIES)

/* Project Includes */
#include "vp_api_types.h"
#include "vp_pulse_decode.h"
#include "sys_service.h"
#include "vp_hal.h"
#include "vp_api_int.h"
#include "vp886_api.h"
#include "vp886_api_int.h"

#ifdef VP886_INCLUDE_TESTLINE_CODE
#include "vp_api_test.h"
#endif

#ifdef VP886_INCLUDE_DTMF_DETECT
#include "vp_dtmf_detect.h"
#endif

static VpStatusType
Vp886GetResultsRdOption(
    VpEventType *pEvent,
    void *pResults);

#ifdef VP886_INCLUDE_TESTLINE_CODE
static VpStatusType
Vp886GetResultsLinetest(
    VpEventType *pEvent,
    void *pResults);
#endif /* VP886_INCLUDE_TESTLINE_CODE */

static bool
Vp886TypeAInterruptsChanged(
    uint8 sigregOld[],
    uint8 sigregNew[]);

#ifdef VP886_INTERRUPT_EDGETRIG_MODE
bool
Vp886TypeBInterruptsPresent(
    uint8 sigreg[]);
#endif

static void
Vp886ProcessSigReg(
    VpDevCtxType *pDevCtx,
    uint8 sigreg[]);

static void
Vp886ProcessDeviceEvents(
    VpDevCtxType *pDevCtx,
    uint8 sigreg[]);

static void
Vp886ProcessBattFaults(
    VpDevCtxType *pDevCtx,
    uint8 sigreg[]);

static void
Vp886ProcessFxsEvents(
    VpLineCtxType *pLineCtx,
    uint8 sigreg[]);

static bool
Vp886ProcessHookBuffer(
    VpLineCtxType *pLineCtx);

static void
Vp886ProcessPulseDecoding(
    VpLineCtxType *pLineCtx,
    VpCslacLineCondType newHookSt,
    uint16 hookTimestamp);

static uint8
Vp886RingTripConfirmation(
    VpLineCtxType *pLineCtx,
    VpCslacLineCondType newHookSt,
    uint16 timestamp);

static bool
Vp886IsEventMasked(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    VpEventCategoryType eventCategory,
    uint16 eventId);

#ifdef VP886_INCLUDE_DTMF_DETECT
static void
Vp886DtmfSampling(
    VpLineCtxType *pLineCtx);
#endif

/** Vp886ApiTick()
  This function is provided for backward compatibility with existing CSLAC
  applications.
  
  It simply determines whether or not the application needs to call VpGetEvent()
  based on interrupt status and polling mode.  This logic could all be done
  at the application level instead, eliminating the need to call VpApiTick().
  
  If the application does NOT call VpApiTick(), it also has no reason to call
  VpVirtualISR(), and the system service functions VpSysEnableInt(),
  VpSysDisableInt(), and VpSysTestInt()do not need to be implemented.
*/
VpStatusType
Vp886ApiTick(
    VpDevCtxType *pDevCtx,
    bool *pEventStatus)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
#if defined(VP886_INTERRUPT_LEVTRIG_MODE) || defined(VP886_EFFICIENT_POLLED_MODE)
    VpDeviceIdType deviceId = pDevObj->deviceId;
#endif

    Vp886EnterCritical(pDevCtx, VP_NULL, VP_NULL);

#if defined (VP886_INTERRUPT_LEVTRIG_MODE)

    VpSysEnableInt(deviceId);

#elif defined (VP886_EFFICIENT_POLLED_MODE)

    /* Poll the device PIO-INT line */
    pDevObj->pendingInterrupt = VpSysTestInt(deviceId);

#elif defined (VP886_SIMPLE_POLLED_MODE)

    pDevObj->pendingInterrupt = TRUE;

#elif defined (VP886_INTERRUPT_EDGETRIG_MODE)
#else 
#error "Invalid VP886 polling mode"
#endif

    if (pDevObj->pendingInterrupt) {
        *pEventStatus = TRUE;
    } else {
        *pEventStatus = FALSE;
    }

    pDevObj->pendingInterrupt = FALSE;
    Vp886ExitCritical(pDevCtx, VP_NULL, VP_NULL);

    return VP_STATUS_SUCCESS;
}


/** Vp886VirtualISR()
  This function is provided for backward compatibility with existing CSLAC
  applications which call VpApiTick() in an interrupt-driven polling mode.
  
  It sets the pDevObj->pendingInterrupt flag to TRUE so that VpApiTick() will
  return *pEventStatus TRUE to prompt a VpGetEvent() call.
*/
#ifndef VP886_SIMPLE_POLLED_MODE
VpStatusType
Vp886VirtualISR(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
#if defined (VP886_INTERRUPT_LEVTRIG_MODE)
    VpDeviceIdType deviceId = pDevObj->deviceId;
#endif

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886VirtualISR+"));

#if defined (VP886_INTERRUPT_LEVTRIG_MODE)
    VpSysDisableInt(deviceId);
#endif
    /* Device Interrupt Received */
    pDevObj->pendingInterrupt = TRUE;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886VirtualISR-"));

    return VP_STATUS_SUCCESS;
}   /* Vp886VirtualISR() */
#endif


/** Vp886GetEvent()
  Implements VpGetEvent() to report new events that occured on the device.
  This function returns one event for each call to it. It should be called
  repeatedly until no more events are reported for a specific device.
  
  This also performs automatic responses to interrupts and handles timers to
  implement cadences, signals, workarounds, and other internal state machines.
  
  This function is driven by reading the device signaling register.  It will
  read and process the signaling register multiple times until one of the
  following conditions is met:
   - All of the type A persistent signals are the same as the previous signaling
     register read, indicating that our information is now all up to date.
   - The number of reads reaches the maximum defined by VP886_MAX_SIGREG_READS
     to prevent locking up if a signal oscillates.
   - An API event has been put into the queue.  Returning as soon as an event is
     available reduces the probability of the event queue overflowing.  In this
     case, the application should immediately call VpGetEvent() again.

  See the VP-API-II Reference Guide for more details on VpGetEvent().
*/
bool
Vp886GetEvent(
    VpDevCtxType *pDevCtx,
    VpEventType *pEvent)    /**< Pointer to the results event structure */
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 sigreg[VP886_R_SIGREG_LEN];
    uint8 numReads = 0;
    uint8 maxReads;

    Vp886EnterCritical(pDevCtx, NULL, "Vp886GetEvent");

    if (!(pDevObj->busyFlags & (VP_DEV_INIT_CMP | VP_DEV_INIT_IN_PROGRESS))) {
        Vp886ExitCritical(pDevCtx, NULL, "Vp886GetEvent");
        return FALSE;
    }

    pEvent->status = VP_STATUS_SUCCESS;

    /* If there is already an event in the queue, return it immediately */
    if (Vp886PopEvent(pDevCtx, pEvent) == TRUE) {
        VP_INFO(VpDevCtxType, pDevCtx, ("Vp886GetEvent - immediate event"));
        Vp886ExitCritical(pDevCtx, NULL, "Vp886GetEvent");
        return TRUE;
    }

    /* Stop reading the signaling register if there has been a SPI error */
    if (pDevObj->spiError) {
        pEvent->status = VP_STATUS_ERR_SPI;
        pDevObj->spiError = TRUE;
        Vp886ExitCritical(pDevCtx, NULL, "Vp886GetEvent");
        return FALSE;
    }

    pDevObj->inGetEvent = TRUE;
    
    /* Read the signaling register repeatedly until it does not change. This
       should ensure that any interrupts are cleared.  Reading twice back to
       back would only ensure this if the two reads could be done without any
       chance of something happening in between.  Since we could not guarantee
       that, this is the safer approach.  In a simple polled mode, this also
       saves some time by doing only one read if nothing is happening.
       Enforce a maximum number of reads to prevent locking up the system if
       a signal is oscillating. */
    maxReads = VP886_MAX_SIGREG_READS;
    pDevObj->sigregReadPending = TRUE;
    while (pDevObj->sigregReadPending && numReads < maxReads) {
        /* Read the signaling register */
        pDevObj->dontFlushSlacBufOnRead = TRUE;
        VpSlacRegRead(pDevCtx, NULL, VP886_R_SIGREG_UL_RD, VP886_R_SIGREG_LEN, sigreg);
        pDevObj->sigregReadPending = FALSE;

        /* Check for a potential SPI error. The register should never actually
           be 0xFFFFFFFF. */
        if (sigreg[0] == 0xFF && sigreg[1] == 0xFF && sigreg[2] == 0xFF && sigreg[3] == 0xFF) {
            pEvent->status = VP_STATUS_ERR_SPI;
            pDevObj->spiError = TRUE;
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetEvent - SPI error, sigreg read 0xFFFFFFFF"));
            Vp886ExitCritical(pDevCtx, NULL, "Vp886GetEvent");
            return FALSE;
        }

        /* Print out the signaling register.  It may look redundant to check for
           VP_DBG_INTERRUPT enabled before invoking the VP_INTERRUPT macro, but
           this is done to avoid adding timestamp register reads when this debug
           type is disabled. */
#if (VP_CC_DEBUG_SELECT & VP_DBG_INTERRUPT)
        if ((pDevObj->debugSelectMask & VP_DBG_INTERRUPT) &&
            (sigreg[0] != pDevObj->registers.sigreg[0] ||
             sigreg[1] != pDevObj->registers.sigreg[1] ||
             sigreg[2] != pDevObj->registers.sigreg[2] ||
             sigreg[3] != pDevObj->registers.sigreg[3]))
        {
            uint16 timestamp = Vp886GetTimestamp(pDevCtx);
            VP_INTERRUPT(VpDevCtxType, pDevCtx, ("sigreg: %02X %02X %02X %02X  time %u",
                sigreg[0], sigreg[1], sigreg[2], sigreg[3], timestamp));
        }
#endif

        /* Compare it to the previous image of the register. We'll want to
           read again if any Type A interrupt bits changed. */
        if (Vp886TypeAInterruptsChanged(pDevObj->registers.sigreg, sigreg)) {
            pDevObj->sigregReadPending = TRUE;
        }
#ifdef VP886_INTERRUPT_EDGETRIG_MODE
        /* In edge triggered interrupt systems, there is a possibility that an
           interrupt could occur at the same time as our last sigreg read.  In
           this case the interrupt pin would remain low, the application would
           not see this interrupt edge, nor any in the future.  We need to
           continue to read until we see sigreg data indicating that no
           interrupt has occurred, to ensure that the interrupt pin goes high */
        if (Vp886TypeBInterruptsPresent(sigreg)) {
            pDevObj->sigregReadPending = TRUE;
        }
#else
        /* If NOT edge triggered, we can reduce MPI traffic by only reading
           once during InitDevice, since we won't be generating events for the
           type A interrupts yet. */
        if (!(pDevObj->busyFlags & VP_DEV_INIT_CMP)) {
            pDevObj->sigregReadPending = FALSE;
        }
#endif

        /* Cache the signaling register in the device object */
        VpMemCpy(pDevObj->registers.sigreg, sigreg, VP886_R_SIGREG_LEN);
        
        /* Allow a new timestamp for each sigreg processing */
        pDevObj->timestampValid = FALSE;

        /* Process signals */
        Vp886ProcessSigReg(pDevCtx, sigreg);

        numReads++;

        /* If anything is in the event queue now, return the event and TRUE */
        if (Vp886PopEvent(pDevCtx, pEvent) == TRUE) {
            VP_INFO(VpDevCtxType, pDevCtx, ("Vp886GetEvent - returned event (%d)", numReads));
            pDevObj->sigregReadPending = FALSE;
            pDevObj->inGetEvent = FALSE;
            Vp886ExitCritical(pDevCtx, NULL, "Vp886GetEvent");
            return TRUE;
        }

    }

    pDevObj->sigregReadPending = FALSE;
    pDevObj->inGetEvent = FALSE;

    /* If anything is in the event queue now, return the event and TRUE */
    if (Vp886PopEvent(pDevCtx, pEvent) == TRUE) {
        VP_INFO(VpDevCtxType, pDevCtx, ("Vp886GetEvent - returned event"));
        Vp886ExitCritical(pDevCtx, NULL, "Vp886GetEvent");
        return TRUE;
    }

    Vp886ExitCritical(pDevCtx, NULL, "Vp886GetEvent");
    return FALSE;
}


/** Vp886TypeAInterruptsChanged()
  This function compares two signaling register snapshots to check for changes
  in Type A interrupts - those where the signaling register reflects the status
  of a condition rather than just indicating that a particular event occurred.
  When these conditions change, the new value that caused the interrupt is
  latched until the signaling register is read.  This means that if something
  changes twice before the signaling register is read, we will see two
  different values.  We need to read multiple times to be sure we end up
  knowing the current status.
  Type B interrupts such as CID, CAD, and timer interrupts only indicate that
  event occurred.  We know that those bits will change back to 0 after reading
  the signaling register, so there is no need to read it again if only they are
  involved.
*/
bool
Vp886TypeAInterruptsChanged(
    uint8 sigregOld[],
    uint8 sigregNew[])
{
    uint8 typeAMask[VP886_R_SIGREG_LEN];
    uint8 i;
    
    typeAMask[0] = VP886_R_SIGREG_CFAIL | VP886_R_SIGREG_TEMPA |
                   VP886_R_SIGREG_IO2 | VP886_R_SIGREG_GNK | VP886_R_SIGREG_HOOK;
    typeAMask[1] = VP886_R_SIGREG_TEMPA | VP886_R_SIGREG_IO2 |
                   VP886_R_SIGREG_GNK | VP886_R_SIGREG_HOOK;
    typeAMask[2] = VP886_R_SIGREG_CPUVLO | VP886_R_SIGREG_OVALM;
    typeAMask[3] = VP886_R_SIGREG_OVALM;
    
    for (i = 0; i < VP886_R_SIGREG_LEN; i++) {
        if ((sigregOld[i] ^ sigregNew[i]) & typeAMask[i]) {
            return TRUE;
        }
    }
    
    return FALSE;
}

#ifdef VP886_INTERRUPT_EDGETRIG_MODE
/** Vp886TypeBInterruptsPresent()
  This function checks to see whether any type B interrupts are present in
  the current signaling register data.  This is required for edge triggered
  interrupt mode, as explained in the comments in Vp886GetEvent()
*/
bool
Vp886TypeBInterruptsPresent(
    uint8 sigreg[])
{
    if ((sigreg[0] & (VP886_R_SIGREG_CAD | VP886_R_SIGREG_CID |
            VP886_R_SIGREG_OCALM)) ||
        (sigreg[1] & (VP886_R_SIGREG_CAD | VP886_R_SIGREG_CID |
            VP886_R_SIGREG_OCALM)) ||
        (sigreg[2] & (VP886_R_SIGREG_GTIMER | VP886_R_SIGREG_CHTIMER |
            VP886_R_SIGREG_SDAT | VP886_R_SIGREG_VDAT)) ||
        (sigreg[3] & (VP886_R_SIGREG_TIMESTAMP | VP886_R_SIGREG_CHTIMER |
            VP886_R_SIGREG_SDAT | VP886_R_SIGREG_VDAT)))
    {
        return TRUE;
    } else {
        return FALSE;
    }
}
#endif

/** Vp886ProcessSigReg()
  Breaks up the processing of the signaling register into Device, per-Line, and
  Timer interrupt handling.
*/
void
Vp886ProcessSigReg(
    VpDevCtxType *pDevCtx,
    uint8 sigreg[])
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpLineCtxType *pLineCtx;
    uint8 channelId;
    uint8 typeAMask[VP886_R_SIGREG_LEN];

    /* Process device-level events */
    Vp886ProcessDeviceEvents(pDevCtx, sigreg);

    /* Process channel-specific events */
    for (channelId = 0; channelId < pDevObj->staticInfo.maxChannels; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            Vp886ProcessFxsEvents(pLineCtx, sigreg);
        }
    }

    /* Check for global timer interrupt.  Process timers last so that other
       interrupts are treated as higher priority and can cancel timers that
       expire at the same time. */
    if ((sigreg[2] & VP886_R_SIGREG_GTIMER) || pDevObj->timerOverride) {
        Vp886ProcessTimers(pDevCtx);
    }

    /* Now that we've processed everything, clear the type B interrupts from
       the device object cache, since we know that reading the signaling
       register will have cleared these bits. Everything that is not type A is
       type B. */
    typeAMask[0] = VP886_R_SIGREG_CFAIL | VP886_R_SIGREG_OCALM | VP886_R_SIGREG_TEMPA |
                   VP886_R_SIGREG_IO2 | VP886_R_SIGREG_GNK | VP886_R_SIGREG_HOOK;
    typeAMask[1] = VP886_R_SIGREG_OCALM | VP886_R_SIGREG_TEMPA |
                   VP886_R_SIGREG_IO2 | VP886_R_SIGREG_GNK | VP886_R_SIGREG_HOOK;
    typeAMask[2] = VP886_R_SIGREG_CPUVLO | VP886_R_SIGREG_OVALM;
    typeAMask[3] = VP886_R_SIGREG_OVALM;
    pDevObj->registers.sigreg[0] &= typeAMask[0];
    pDevObj->registers.sigreg[1] &= typeAMask[1];
    pDevObj->registers.sigreg[2] &= typeAMask[2];
    pDevObj->registers.sigreg[3] &= typeAMask[3];

    return;
}


/** Vp886ProcessDeviceEvents()
  Processes device-level interrupts and signals in the signaling register.
  
  This processing includes generating API events, running interrupt-driven
  internal state machines, and automatic fault response.
*/
void
Vp886ProcessDeviceEvents(
    VpDevCtxType *pDevCtx,
    uint8 sigreg[])
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpLineCtxType *pLineCtx;
    Vp886LineObjectType *pLineObj;
    bool tempClkFault;

    /* + Timestamp Rollover */
    /* See Vp886GetTimestamp() for rollover tracking details */
    if (sigreg[3] & VP886_R_SIGREG_TIMESTAMP) {
        VP_INFO(VpDevCtxType, pDevCtx, ("TS Rollover"));
        pDevObj->rolloverBuffer++;
        Vp886PushEvent(pDevCtx, VP886_DEV_EVENT, VP_EVCAT_SIGNALING, VP_DEV_EVID_TS_ROLLOVER, 0, Vp886GetTimestamp(pDevCtx), FALSE);
    }
    /* - Timestamp Rollover */

    /* + SADC */
    /* Used during device init and calibration. */
    if ((sigreg[2] & VP886_R_SIGREG_SDAT) || (sigreg[3] & VP886_R_SIGREG_SDAT)) {
        if (pDevObj->busyFlags & VP_DEV_IN_CAL) {
            if (sigreg[2] & VP886_R_SIGREG_SDAT) {
                pDevObj->channelCalBack[0] = TRUE;
            }
            if (sigreg[3] & VP886_R_SIGREG_SDAT) {
                pDevObj->channelCalBack[1] = TRUE;
            }
            if (pDevObj->busyFlags & VP_DEV_INIT_IN_PROGRESS) {
                Vp886InitDeviceSM(pDevCtx);
            } else {
                Vp886CalCodecHandler(pDevCtx);
            }
        }
    }
    /* - SADC */
    
    /* + CH2 Timer */
    /* Used for continuous sampling timing */
    if (sigreg[3] & VP886_R_SIGREG_CHTIMER) {
        uint8 channelId;
        for (channelId = 0; channelId < pDevObj->staticInfo.maxChannels; channelId++) {
            pLineCtx = pDevCtx->pLineCtx[channelId];
            if (pLineCtx == VP_NULL) {
                continue;
            }
            pLineObj = pLineCtx->pLineObj;
            
#ifdef VP886_INCLUDE_DTMF_DETECT
            if (pLineObj->dtmf.sampling) {
                Vp886DtmfSampling(pLineCtx);
            }
#endif /* VP886_INCLUDE_DTMF_DETECT */
        }
    }
    /* - CH2 Timer */

    /* Ignore everything below here during calibration or before init complete */
    if ((pDevObj->busyFlags & VP_DEV_IN_CAL) || !(pDevObj->busyFlags & VP_DEV_INIT_CMP)) {
        return;
    }

    /* + Clock Fault */
    if (sigreg[0] & VP886_R_SIGREG_CFAIL) {
        tempClkFault = TRUE;
    } else {
        tempClkFault = FALSE;
    }
    if (tempClkFault != pDevObj->dynamicInfo.clkFault) {
        if (tempClkFault) {
            if (!(pDevObj->stateInt & VP886_FORCE_FREE_RUN)) {
                Vp886FreeRunStart(pDevCtx);
            }
            Vp886PushEvent(pDevCtx, VP886_DEV_EVENT, VP_EVCAT_FAULT, VP_DEV_EVID_CLK_FLT, 1, Vp886GetTimestamp(pDevCtx), FALSE);
        } else {
            if (!(pDevObj->stateInt & VP886_FORCE_FREE_RUN)) {
                Vp886FreeRunStop(pDevCtx);
            }
            Vp886PushEvent(pDevCtx, VP886_DEV_EVENT, VP_EVCAT_FAULT, VP_DEV_EVID_CLK_FLT, 0, Vp886GetTimestamp(pDevCtx), FALSE);
        }
        pDevObj->dynamicInfo.clkFault = tempClkFault;
    }
    /* - Clock Fault */

    Vp886ProcessBattFaults(pDevCtx, sigreg);

    return;
}


/** Vp886ProcessBattFaults()
  Process battery fault interrupts.
*/
void
Vp886ProcessBattFaults(
    VpDevCtxType *pDevCtx,
    uint8 sigreg[])
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpLineCtxType *pLineCtx;
    Vp886LineObjectType *pLineObj;
    uint16 newBattFltStatus = 0;
    bool newSwyFault = FALSE;
    bool newSwzFault = FALSE;
    bool newCpFault = FALSE;
    uint16 pollTime = 0;

    if (sigreg[2] & VP886_R_SIGREG_OVALM) {
        newBattFltStatus |= VP_BAT_FLT_SWY_OV;
    }
    if (sigreg[3] & VP886_R_SIGREG_OVALM) {
        newBattFltStatus |= VP_BAT_FLT_SWZ_OV;
    }
    if (sigreg[2] & VP886_R_SIGREG_CPUVLO) {
        newBattFltStatus |= VP_BAT_FLT_CP_UV;
    }
    
    /* Overcurrent is a type B interrupt which occurs each time an internal
       event counter reaches a threshold programmed from the device profile.
       Normal operations will occasionally add to the counter, so it can reach
       the threshold and trigger an interrupt at essentially any time.  We only
       want to act if the overcurrent is a sustained condition, so we will look
       for two of the interrupts to occur within a specified time period. */

    if (sigreg[0] & VP886_R_SIGREG_OCALM) {
        uint8 swOc[VP886_R_SW_OC_LEN];
        /* Clear the internal counter */
        pDevObj->ecVal = VP886_EC_1;
        VpSlacRegRead(pDevCtx, VP_NULL, VP886_R_SW_OC_RD, VP886_R_SW_OC_LEN, swOc);
        pDevObj->ecVal = VP886_EC_GLOBAL;
        if (Vp886GetTimestamp32(pDevCtx) - pDevObj->overCurrentTimestamp[0] <= VP886_OVERCURRENT_INTERVAL) {
            newBattFltStatus |= VP_BAT_FLT_SWY_OC;
        }
        pDevObj->overCurrentTimestamp[0] = Vp886GetTimestamp32(pDevCtx);
        VP_INFO(VpDevCtxType, pDevCtx, ("Cleared OC counter for ch0, ts %ld",
            pDevObj->overCurrentTimestamp[0]));
    } else {
        if (pDevObj->battFltStatus & VP_BAT_FLT_SWY_OC) {
            if (Vp886GetTimestamp32(pDevCtx) - pDevObj->overCurrentTimestamp[0] > VP886_OVERCURRENT_INTERVAL) {
                /* Enough time has passed without an OC event, so the fault will
                   be cleared. */
            } else {
                newBattFltStatus |= VP_BAT_FLT_SWY_OC;
            }
        }
    }

    if (sigreg[1] & VP886_R_SIGREG_OCALM) {
        uint8 swOc[VP886_R_SW_OC_LEN];
        /* Clear the internal counter */
        pDevObj->ecVal = VP886_EC_2;
        VpSlacRegRead(pDevCtx, VP_NULL, VP886_R_SW_OC_RD, VP886_R_SW_OC_LEN, swOc);
        pDevObj->ecVal = VP886_EC_GLOBAL;
        if (Vp886GetTimestamp32(pDevCtx) - pDevObj->overCurrentTimestamp[1] <= VP886_OVERCURRENT_INTERVAL) {
            newBattFltStatus |= VP_BAT_FLT_SWZ_OC;
        }
        pDevObj->overCurrentTimestamp[1] = Vp886GetTimestamp32(pDevCtx);
        VP_INFO(VpDevCtxType, pDevCtx, ("Cleared OC counter for ch1, ts %ld",
            pDevObj->overCurrentTimestamp[1]));
    } else {
        if (pDevObj->battFltStatus & VP_BAT_FLT_SWZ_OC) {
            if (Vp886GetTimestamp32(pDevCtx) - pDevObj->overCurrentTimestamp[1] > VP886_OVERCURRENT_INTERVAL) {
                /* Enough time has passed without an OC event, so the fault will
                   be cleared. */
            } else {
                newBattFltStatus |= VP_BAT_FLT_SWZ_OC;
            }
        }
    }

    /* Return if nothing has changed */
    if (newBattFltStatus == pDevObj->battFltStatus) {
        return;
    }

    /* Determine which NEW faults appeared */
    if (((newBattFltStatus & VP_BAT_FLT_SWY_OV) && !(pDevObj->battFltStatus & VP_BAT_FLT_SWY_OV)) ||
        ((newBattFltStatus & VP_BAT_FLT_SWY_OC) && !(pDevObj->battFltStatus & VP_BAT_FLT_SWY_OC)))
    {
        newSwyFault = TRUE;
    }
    if (((newBattFltStatus & VP_BAT_FLT_SWZ_OV) && !(pDevObj->battFltStatus & VP_BAT_FLT_SWZ_OV)) ||
        ((newBattFltStatus & VP_BAT_FLT_SWZ_OC) && !(pDevObj->battFltStatus & VP_BAT_FLT_SWZ_OC)))
    {
        newSwzFault = TRUE;
    }
    if ((newBattFltStatus & VP_BAT_FLT_CP_UV) && !(pDevObj->battFltStatus & VP_BAT_FLT_CP_UV))
    {
        newCpFault = TRUE;
    }

    /* Handle the VP_OPTION_ID_SWITCHER_CTRL option */
    if (VP886_IS_TRACKER(pDevObj)) {
        /* Each switcher corresponds to a line. If the SWITCHER_CTRL option
           for a line is set, and there is a new fault on the associated
           switcher (ch0-bat1-Y, ch1-bat2-Z), set that line to shutdown. */
        if (newSwyFault) {
            pLineCtx = pDevCtx->pLineCtx[0];
            if (pLineCtx != VP_NULL) {
                pLineObj = pLineCtx->pLineObj;
                if (pLineObj->options.switcherCtrl == TRUE) {
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("Setting VP_LINE_DISABLED due to bat1 fault"));
                    Vp886SetLineStateFxs(pLineCtx, VP_LINE_DISABLED);
                }
            }
        }
        if (newSwzFault) {
            pLineCtx = pDevCtx->pLineCtx[1];
            if (pLineCtx != VP_NULL) {
                pLineObj = pLineCtx->pLineObj;
                if (pLineObj->options.switcherCtrl == TRUE) {
                    VP_LINE_STATE(VpLineCtxType, pLineCtx,
                        ("Setting VP_LINE_DISABLED due to bat2 fault"));
                    Vp886SetLineStateFxs(pLineCtx, VP_LINE_DISABLED);
                }
            }
        }
    } else { /* ABS */
        /* Any bat1 or bat2 fault affects both channels. If either channel
           has the SWITCHER_CTRL option set, put both channels in shutdown. */
        if (newSwyFault || newSwzFault) {
            VpLineCtxType *pLineCtx1 = pDevCtx->pLineCtx[0];
            VpLineCtxType *pLineCtx2 = pDevCtx->pLineCtx[1];
            bool shutdown = FALSE;
            
            if (pLineCtx1 != VP_NULL) {
                pLineObj = pLineCtx1->pLineObj;
                if (pLineObj->options.switcherCtrl == TRUE) {
                    shutdown = TRUE;
                }
            }
            pLineCtx = pDevCtx->pLineCtx[1];
            if (pLineCtx2 != VP_NULL) {
                pLineObj = pLineCtx2->pLineObj;
                if (pLineObj->options.switcherCtrl == TRUE) {
                    shutdown = TRUE;
                }
            }
            if (shutdown) {
                Vp886ShutdownDeviceInt(pDevCtx);
            }
        }
    }

    /* Handle charge pump undervoltage shutdown.  If it is enabled by the
       device profile, the switchers should have already shut down due to
       the ssCfg AUTO_CP_UV setting.  Set the lines to DISABLED to match the
       automatic behavior. */
    if (pDevObj->devProfileData.cpProtection == VP886_CP_PROT_UV_SHUTDOWN && newCpFault) {
        VpLineCtxType *pLineCtx1 = pDevCtx->pLineCtx[0];
        VpLineCtxType *pLineCtx2 = pDevCtx->pLineCtx[1];

        if (pLineCtx1 != VP_NULL) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx1,
                ("Setting VP_LINE_DISABLED due to charge pump undervoltage"));
            Vp886SetLineStateFxs(pLineCtx1, VP_LINE_DISABLED);
        }
        if (pLineCtx2 != VP_NULL) {
            VP_LINE_STATE(VpLineCtxType, pLineCtx2,
                ("Setting VP_LINE_DISABLED due to charge pump undervoltage"));
            Vp886SetLineStateFxs(pLineCtx2, VP_LINE_DISABLED);
        }
    }

    /* We'll have to poll to detect when some battery faults go away */
    if (newBattFltStatus != 0) {
        /* We clear an overcurrent fault by watching for when one doesn't occur.
           Since that means there will be no interrupt, we need a timer to know
           when to check. */
        if (newBattFltStatus & (VP_BAT_FLT_SWY_OC | VP_BAT_FLT_SWZ_OC)) {
            pollTime = VP886_OVERCURRENT_INTERVAL;
        } else if ( (newBattFltStatus & VP_BAT_FLT_CP_UV) ||
                    (!VP886_IS_SF(pDevObj) &&
                     ((newBattFltStatus & VP_BAT_FLT_SWY_OV) ||
                      (newBattFltStatus & VP_BAT_FLT_SWZ_OV))) ) {
            /* Other faults do not generate interrupts when they go away.  This
               is true for undervoltage faults on all devices, but was fixed for
               overvoltage faults on SF devices.  This means we need to create
               a periodic timer interrupt so that we can tell when the fault
               goes away.  The specific timing here isn't as important as for
               overcurrent, so allow the overcurrent timer duration to have
               priority. */
            if (pollTime == 0 || VP886_BATFLT_POLL_TIME < pollTime) {
                pollTime = VP886_BATFLT_POLL_TIME;
            }
        }
    } else {
        /* Everything is cleared, no timer needed */
        pollTime = 0;
    }
    if (pollTime > 0) {
        Vp886RestartTimerMs(pDevCtx, VP_NULL, VP886_TIMERID_BATFLT_POLL, pollTime, 0);
    } else {
        /* Stop the polling timer */
        Vp886CancelTimer(pDevCtx, VP_NULL, VP886_TIMERID_BATFLT_POLL, 0, FALSE);
    }
    pDevObj->battFltPollTime = pollTime;

    /* Generate an event and update status */
    if (newBattFltStatus != pDevObj->battFltStatus) {
        pDevObj->battFltStatus = newBattFltStatus;
        Vp886PushEvent(pDevCtx, VP886_DEV_EVENT, VP_EVCAT_FAULT, VP_DEV_EVID_BAT_FLT, newBattFltStatus, Vp886GetTimestamp(pDevCtx), FALSE);
    }

    return;
}

/** Vp886ProcessDeviceEvents()
  Processes channel specific interrupts and signals in the signaling register
  for FXS lines.
  
  This processing includes generating API events, running interrupt-driven
  internal state machines, and automatic fault response.
  
  Some of the signals processed here may be ignored based on detection masks
  set and cleared in Vp886SetDetectMask() and Vp886ClearDetectMask().
*/
void
Vp886ProcessFxsEvents(
    VpLineCtxType *pLineCtx,
    uint8 sigreg[])
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    uint8 chanSignals[2];
    VpCslacLineCondType tempHookSt = VP_CSLAC_CLEAR;
    VpCslacLineCondType tempGkeySt = VP_CSLAC_CLEAR;
    VpCslacLineCondType tempThermFlt = VP_CSLAC_CLEAR;
    VpCslacLineCondType tempDcFlt = VP_CSLAC_CLEAR;
    bool tempIo2State = FALSE;
    uint8 io2Use;

    /* Info for ch 0 is in bytes 0 and 2.  Info for ch 1 is in bytes 1 and 3 */
    chanSignals[0] = sigreg[0 + channelId];
    chanSignals[1] = sigreg[2 + channelId];
    /* chanSignals[0] includes:
     *  VP886_R_SIGREG_OCALM    VP886_R_SIGREG_TEMPA    VP886_R_SIGREG_IO2
     *  VP886_R_SIGREG_CAD      VP886_R_SIGREG_CID      VP886_R_SIGREG_GNK
     *  VP886_R_SIGREG_HOOK
     * chanSignals[1] includes:
     *  VP886_R_SIGREG_OVALM    VP886_R_SIGREG_CHTIMER  VP886_R_SIGREG_SDAT
     *  VP886_R_SIGREG_VDAT
     */

    /* SADC interrupt */
    if (chanSignals[1] & VP886_R_SIGREG_SDAT) {
        /* Handling of SADC interrupts during InitDevice is done in
           Vp886ProcessDeviceEvents */

        /* SADC interrupt used during Get Loop Condition */
        if (pLineObj->busyFlags & VP886_LINE_GET_LOOP_COND) {
            Vp886GetLoopCondResults(pLineCtx);
        }

        /* VpCalLine */
        if (pLineObj->busyFlags & VP886_LINE_IN_CAL) {
            Vp886CalLineSM(pLineCtx);
        }

        /* VpCal():VP_CAL_QUICKCAL */
        if (pLineObj->busyFlags & VP886_LINE_QUICKCAL) {
            Vp886QuickCalHandler(pLineCtx);
        }

#if defined(VP886_INCLUDE_DTMF_DETECT) && defined(VP886_DTMF_DETECT_NONCONTINUOUS)
        if (pLineObj->dtmf.sampling) {
            Vp886DtmfSampling(pLineCtx);
        }
#endif /* VP886_INCLUDE_DTMF_DETECT && VP886_DTMF_DETECT_NONCONTINUOUS */

#ifdef VP886_INCLUDE_TESTLINE_CODE
        #if defined(__KERNEL__) && defined(ZARLINK_CFG_INTERNAL)
            VpSysServiceToggleLed(9);
        #endif
        if (pLineObj->busyFlags & VP886_LINE_TEST) {
            const void *pArgsUntyped;
            if (pLineCtx != VP_NULL) {
                pLineObj->busyFlags &= ~VP886_LINE_TEST;
                pArgsUntyped = (const void*)&pLineObj->testInfo.pTestHeap->testArgs;
                Vp886TestLineInt(pLineCtx, pArgsUntyped, TRUE);
            }
        }
#endif /* VP886_INCLUDE_TESTLINE_CODE */
    }

#ifdef VP886_INCLUDE_TESTLINE_CODE
    if (chanSignals[1] & VP886_R_SIGREG_VDAT) {
        if (pLineObj->busyFlags & VP886_LINE_TEST) {
            const void *pArgsUntyped;
            if (pLineCtx != VP_NULL) {
                pLineObj->busyFlags &= ~VP886_LINE_TEST;
                pArgsUntyped = (const void*)&pLineObj->testInfo.pTestHeap->testArgs;
                Vp886TestLineInt(pLineCtx, pArgsUntyped, TRUE);
            }
        }
    }
#endif

    /* Ignore everything below here during calibration or before init complete*/
    if ((pDevObj->busyFlags & VP_DEV_IN_CAL) || !(pDevObj->busyFlags & VP_DEV_INIT_CMP)) {
        return;
    }
    if ((pLineObj->busyFlags & VP886_LINE_IN_CAL) || !(pLineObj->busyFlags & VP886_LINE_INIT_CMP)) {
        return;
    }
    if (pLineObj->quickCal.state != VP886_QUICKCAL_INACTIVE) {
        return;
    }

    /* Collect temp info from the signaling register */
    if (chanSignals[0] & VP886_R_SIGREG_HOOK) {
        tempHookSt = VP_CSLAC_HOOK;
    }
    if (pLineObj->reportDcFaults == FALSE) {
        if (chanSignals[0] & VP886_R_SIGREG_GNK) {
            tempGkeySt = VP_CSLAC_GKEY;
        }
    } else {
        if (chanSignals[0] & VP886_R_SIGREG_GNK) {
            tempDcFlt = VP_CSLAC_DC_FLT;
        }
    }
    if (chanSignals[0] & VP886_R_SIGREG_TEMPA) {
        tempThermFlt = VP_CSLAC_THERM_FLT;
    }
    if (chanSignals[0] & VP886_R_SIGREG_IO2) {
        tempIo2State = TRUE;
    }

    /* In the tip open and ring open states, we can't detect actual offhooks.
       However, if the non-open lead is grounded, it will appear as an offhook
       in the signaling register.  Translate these off-hook indications in
       lead-open states into groundkey events, since that is what they actually
       are. */
    if ((pLineObj->lineState.currentState == VP_LINE_TIP_OPEN ||
         pLineObj->lineState.currentState == VP_LINE_RING_OPEN) &&
        tempHookSt == VP_CSLAC_HOOK)
    {
        tempGkeySt = VP_CSLAC_GKEY;
        tempHookSt = VP_CSLAC_CLEAR;
    }
    
    /* Translate gkeys into hooks during ringing if VP_OPTION_ID_RINGTRIP
       confirm is enabled, and also in other states while confirming ring trip. */
    if (tempGkeySt == VP_CSLAC_GKEY) {
        if ((Vp886LineStateInfo(pLineObj->lineState.currentState).normalEquiv == VP_LINE_RINGING &&
             pLineObj->options.ringTripConfirm > 0) ||
            pLineObj->ringTripConfirm.confirming)
        {
            tempHookSt = VP_CSLAC_HOOK;
            tempGkeySt = VP_CSLAC_CLEAR;
        }
    }

    /* + Hook */
    if (!(pLineObj->detectMasks & VP_CSLAC_HOOK)) {
        if ((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_HOOK) != tempHookSt) {
            /* Hook state change was detected. */
            if (Vp886ProcessHookBuffer(pLineCtx) == FALSE) {
                /* If the hook buffer didn't contain any useful info, just use
                   the information we have from the signaling register and the
                   latest timestamp. */
                Vp886ProcessHookEvent(pLineCtx, tempHookSt, Vp886GetTimestamp(pDevCtx));
            }
        }
    }
    /* - Hook */

    /* + Ground Key */
    if (!(pLineObj->detectMasks & VP_CSLAC_GKEY)) {
        if ((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_GKEY) != tempGkeySt) {
            if (tempGkeySt == VP_CSLAC_GKEY) {
                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_SIGNALING, VP_LINE_EVID_GKEY_DET, 0, Vp886GetTimestamp(pDevCtx), FALSE);
#ifdef VP_CSLAC_SEQ_EN
                /* Abort caller ID if a groundkey is detected */
                if (tempGkeySt == VP_CSLAC_GKEY && pLineObj->cid.active) {
                    Vp886CidStop(pLineCtx);
                }
#endif
                Vp886GndFltProtHandler(pLineCtx, VP886_GNDFLTPROT_INP_GKEY_DET);
            } else {
                /* If a GND_FLT was declared and not cleared, clear it now */
                if (pLineObj->gndFltProt.faultDeclared) {
                    Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_FAULT, VP_LINE_EVID_GND_FLT, 0, Vp886GetTimestamp(pDevCtx), FALSE);
                    pLineObj->gndFltProt.faultDeclared = FALSE;
                }

                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_SIGNALING, VP_LINE_EVID_GKEY_REL, 0, Vp886GetTimestamp(pDevCtx), FALSE);

                Vp886GndFltProtHandler(pLineCtx, VP886_GNDFLTPROT_INP_GKEY_REL);
            }
            pLineObj->lineState.condition &= ~VP_CSLAC_GKEY;
            pLineObj->lineState.condition |= tempGkeySt;
        }
        /* If a groundkey is detected during balanced ringing, exit the
           current ringing cycle, but without ending a ring cadence. */
        if (tempGkeySt == VP_CSLAC_GKEY && !pLineObj->unbalancedRinging &&
            !pLineObj->inLineTest)
        {
            if (pLineObj->lineState.currentState == VP_LINE_RINGING) {
                VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Groundkey detected, exiting ring phase"));
                Vp886SetLineStateFxsInt(pLineCtx, VP_LINE_ACTIVE);
            }
            if (pLineObj->lineState.currentState == VP_LINE_RINGING_POLREV) {
                VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Groundkey detected, exiting ring phase"));
                Vp886SetLineStateFxsInt(pLineCtx, VP_LINE_ACTIVE_POLREV);
            }
        }
    }
    /* - Ground Key */
    
    /* + DC Fault */
    if (!(pLineObj->detectMasks & VP_CSLAC_DC_FLT) &&
        !(pLineObj->detectMasks & VP_CSLAC_GKEY)) /* Masking GKEY also masks DC_FLT */
    {
        if ((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_DC_FLT) != tempDcFlt) {
            if (tempDcFlt == VP_CSLAC_DC_FLT) {
                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_FAULT, VP_LINE_EVID_DC_FLT, 1, Vp886GetTimestamp(pDevCtx), FALSE);
                if (pDevObj->options.criticalFlt.dcFltDiscEn == TRUE &&
                    !pLineObj->inLineTest)
                {
                    Vp886SetLineStateFxs(pLineCtx, VP_LINE_DISCONNECT);
                }
#ifdef VP_CSLAC_SEQ_EN
                /* Abort caller ID if a DC fault is detected */
                if (pLineObj->cid.active) {
                    Vp886CidStop(pLineCtx);
                }
#endif
                Vp886GndFltProtHandler(pLineCtx, VP886_GNDFLTPROT_INP_GKEY_DET);

            } else {
                /* If a GND_FLT was declared and not cleared, clear it now */
                if (pLineObj->gndFltProt.faultDeclared) {
                    Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_FAULT, VP_LINE_EVID_GND_FLT, 0, Vp886GetTimestamp(pDevCtx), FALSE);
                    pLineObj->gndFltProt.faultDeclared = FALSE;
                }

                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_FAULT, VP_LINE_EVID_DC_FLT, 0, Vp886GetTimestamp(pDevCtx), FALSE);

                Vp886GndFltProtHandler(pLineCtx, VP886_GNDFLTPROT_INP_GKEY_REL);
            }
            pLineObj->lineState.condition &= ~VP_CSLAC_DC_FLT;
            pLineObj->lineState.condition |= tempDcFlt;
        }
    }
    /* - DC Fault */

    /* + Thermal Fault */
    if (!(pLineObj->detectMasks & VP_CSLAC_THERM_FLT)) {
        if ((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_THERM_FLT) != tempThermFlt) {
            if (tempThermFlt == VP_CSLAC_THERM_FLT) {
                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_FAULT, VP_LINE_EVID_THERM_FLT, 1, Vp886GetTimestamp(pDevCtx), FALSE);
                if (pDevObj->options.criticalFlt.thermFltDiscEn == TRUE &&
                    !pLineObj->inLineTest)
                {
                    Vp886SetLineStateFxs(pLineCtx, VP_LINE_DISCONNECT);
                }
#ifdef VP_CSLAC_SEQ_EN
                /* Abort caller ID if a thermal fault is detected */
                if (pLineObj->cid.active) {
                    Vp886CidStop(pLineCtx);
                }
#endif
            } else {
                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_FAULT, VP_LINE_EVID_THERM_FLT, 0, Vp886GetTimestamp(pDevCtx), FALSE);
            }
            pLineObj->lineState.condition &= ~VP_CSLAC_THERM_FLT;
            pLineObj->lineState.condition |= tempThermFlt;
        }
    }
    /* - Thermal Fault */

    /* + IO2 */
    io2Use = pDevObj->devProfileData.io2Use >> (channelId*4);
    io2Use &= VP886_DEV_PROFILE_IO2_USE_BITS;
    if (io2Use == VP886_DEV_PROFILE_IO2_USE_DIG_INT) {
        if (pLineObj->io2State != tempIo2State) {
            /* IO2 state change was detected. */
            if (tempIo2State) {
                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_PROCESS, VP_LINE_EVID_USER, 1, Vp886GetTimestamp(pDevCtx), FALSE);
            } else {
                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_PROCESS, VP_LINE_EVID_USER, 0, Vp886GetTimestamp(pDevCtx), FALSE);
            }
            pLineObj->io2State = tempIo2State;
        }
    }
    /* - IO2 */

#ifdef VP_CSLAC_SEQ_EN
    /* + Caller ID Data */
    if (chanSignals[0] & VP886_R_SIGREG_CID) {
        if (pLineObj->cid.active && pLineObj->cid.fskEnabled) {
            /* VP_CID(VpLineCtxType, pLineCtx, ("CID interrupt (ts %u)", Vp886GetTimestamp(pDevCtx))); */
            Vp886CidHandler(pLineCtx, 0);
        }
    }
    /* - Caller ID Data */
    
    /* + Device Cadencer on-time finished */
    if (chanSignals[0] & VP886_R_SIGREG_CAD) {
        if (pLineObj->cid.active && pLineObj->cid.state == VP886_CID_ST_MESSAGE_DTMF) {
            Vp886CidHandler(pLineCtx, 0);
        }
    }
    /* - Device Cadencer on-time finished */
#endif /* VP_CSLAC_SEQ_EN */

    return;
}


/** Vp886ProcessHookBuffer()
  Uses the contents of the hook buffer register combined with the device
  timestamp register to produce accurately timestamped hook events.
*/
bool
Vp886ProcessHookBuffer(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 hookBuf[VP886_R_HOOKBUF_LEN];
    uint8 numHooks;
    VpCslacLineCondType tempHookSt;
    uint16 timestamp;
    uint16 timestamps[3];
    uint8 tsIdx;
    uint8 tsByte;
    uint8 nextTsByte;

    /* Read the hook buffer to get accurate timestamp and queued hook
     * switch information */
    pDevObj->dontFlushSlacBufOnRead = TRUE;
    VpSlacRegRead(NULL, pLineCtx, VP886_R_HOOKBUF_RD, VP886_R_HOOKBUF_LEN, hookBuf);

    numHooks = (hookBuf[0] & VP886_R_HOOKBUF_COUNT) >> 3;

    /* If the buffer contains no hook information, return FALSE to indicate
       that the signaling register information should be used instead.  This is
       expected to happen under several scenarios:
        - A real hook change happened during a software masked hook period,
          because at the end of a hook freeze, we clear out the hook buffer.
        - If the line is in a non-codec state, the buffer will always be 0 (AAA Bug)
        - If an outside application reads the hook buffer register.
       NOTE:  There is a bug in AAA, and possibly BAA where incoming hooks can
              be blocked from registering in the hook buffer during a hook
              buffer read. This is worked around by alternating reading and
              processing the signaling register.  See svn 9656 for the
              VpGetEvent changes, or ZForge #7362 in the Apollo2 issue tracker
              for the device issue. */
    if (numHooks == 0) {
        VP_HOOK(VpLineCtxType, pLineCtx, ("Hook Buffer %02X %02X %02X %02X",
            hookBuf[0], hookBuf[1], hookBuf[2], hookBuf[3]));
        return FALSE;
    }

    /* The buffer can only store 3 hook bits and timestamps.  If there are more,
       it is an overflow. Report a warning, then continue to attempt to process
       the information we have so that the hook status can still end up in the
       correct state. */
    if (numHooks > 3) {
        VP_WARNING(VpLineCtxType, pLineCtx, ("Hook buffer overflow, %d", numHooks));
        numHooks = 3;
    }

    /* Read a new timestamp from the device.  We must do this instead of using
       an existing value to properly rebuild the full hook timestamps.
       Since the hook buffer timestamps are only 8-bit, we must make the
       assumption that they are less than 128ms apart.  If we used a stored
       timestamp we would need to make additional assumptions about how much
       time could have occurred between the old TS read and a potential new hook
       event in the buffer. */
    pDevObj->timestampValid = FALSE;
    timestamp = Vp886GetTimestamp(pDevCtx);

    VP_HOOK(VpLineCtxType, pLineCtx, ("Hook Buffer %02X %02X %02X %02X, ts %u",
        hookBuf[0], hookBuf[1], hookBuf[2], hookBuf[3], timestamp));

    /* Build the full timestamp for each hook event in the buffer, going
       backwards through time in the buffer.  "nextTsByte" refers to the next
       TS in time, though previous in processing.  If the current byte is
       greater than the "next" byte, it means we need to subtract 256 to keep
       them ordered properly. */
    nextTsByte = (timestamp & 0x00FF);
    for (tsIdx = 0; tsIdx < numHooks; tsIdx++) {
        timestamp &= ~0x00FF;
        tsByte = hookBuf[1 + tsIdx];
        timestamp += tsByte;
        if (tsByte > nextTsByte) {
            timestamp -= 0x0100;
        }
        nextTsByte = tsByte;
        timestamps[tsIdx] = timestamp;
    }

    /* Process each hook event rebuilt from the buffer. */
    while (numHooks > 0) {
        numHooks--;
        if ((hookBuf[0] >> numHooks) & 0x01) {
            tempHookSt = VP_CSLAC_HOOK;
        } else {
            tempHookSt = VP_CSLAC_CLEAR;
        }

        timestamp = timestamps[numHooks];

        Vp886ProcessHookEvent(pLineCtx, tempHookSt, timestamp);
    }

    return TRUE;
}


/** Vp886ProcessHookEvent()
  Performs internal processing for hook events.  The hook change inputs to this
  function come either from the hook buffer or directly from the signaling
  register hook status bit.
  
  These hook events may result in immediate API events or could be processed
  further for pulse dial decoding.
  
  This function also stops caller ID for any hook change and handles ring trip
  and other automatic hook responses.
*/
void
Vp886ProcessHookEvent(
    VpLineCtxType *pLineCtx,
    VpCslacLineCondType newHookSt,
    uint16 timestamp)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;

    if ((VpCslacLineCondType)(pLineObj->lineState.condition & VP_CSLAC_HOOK) == newHookSt) {
        /*  No hook state change was detected.  Nothing to do. */
        return;
    }

    /* Add the dial pulse correction from the device profile (specified in ms)
       to all off-hook timestamps. */
    if (newHookSt == VP_CSLAC_HOOK) {
        timestamp += 2 * pDevObj->devProfileData.dialPulseCorrection;
    }

    VP_HOOK(VpLineCtxType, pLineCtx, ("Vp886ProcessHookEvent - %s, ts %u",
        newHookSt == VP_CSLAC_HOOK ? "OFFHOOK" : "ONHOOK", timestamp));

    pLineObj->lineState.condition &= ~VP_CSLAC_HOOK;
    pLineObj->lineState.condition |= newHookSt;

    /* Handle ring trip confirmation (VP_OPTION_ID_RINGTRIP_CONFIRM)
       If this function returns 1, it means that it wants to hide the event,
       so return. */
    if (Vp886RingTripConfirmation(pLineCtx, newHookSt, timestamp) == 1) {
        return;
    }

    if (pLineObj->options.pulseMode == VP_OPTION_PULSE_DECODE_ON) {
        Vp886ProcessPulseDecoding(pLineCtx, newHookSt, timestamp);
    } else if (newHookSt == VP_CSLAC_HOOK) {
        Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_SIGNALING, VP_LINE_EVID_HOOK_OFF, 0, timestamp, FALSE);
    } else {
        Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_SIGNALING, VP_LINE_EVID_HOOK_ON, 0, timestamp, FALSE);
    }

    /* Don't react to hook events during line tests */
    if (pLineObj->inLineTest) {
        return;
    }

    if (newHookSt == VP_CSLAC_HOOK) {
        if (Vp886LineStateInfo(pLineObj->lineState.usrCurrent).normalEquiv == VP_LINE_RINGING) {
            /* Switch to ring trip exit state.  Hook freeze is performed in VpSetLineStateInt(). */
            if (Vp886LineStateInfo(pLineObj->options.ringControl.ringTripExitSt).normalEquiv != VP_LINE_RINGING) {
                VP_HOOK(VpLineCtxType, pLineCtx, ("Vp886ProcessHookEvent: Ring trip to state %d",
                    pLineObj->options.ringControl.ringTripExitSt));
                Vp886SetLineStateFxs(pLineCtx, pLineObj->options.ringControl.ringTripExitSt);
            }
        }

#ifdef VP_CSLAC_SEQ_EN
        if (pLineObj->sendSignal.active && pLineObj->sendSignal.type == VP_SENDSIG_MSG_WAIT_PULSE) {
            Vp886SendSignalStop(pLineCtx, FALSE);
        }
#endif
    }

#ifdef VP_CSLAC_SEQ_EN
    /* Stop CID if we see a hook event. */
    if (pLineObj->cid.active) {
        Vp886CidStop(pLineCtx);
    }
#endif

    if (newHookSt != VP_CSLAC_HOOK) {
        /* If an onhook is detected on a low power device while the API line
           state is STANDBY, we have to force the device back to low power mode.
           The automatic transitions would have taken it out of low power while
           offhook. */
        if (pLineObj->termType == VP_TERM_FXS_LOW_PWR &&
            pLineObj->lineState.usrCurrent == VP_LINE_STANDBY)
        {
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, pLineObj->registers.sysState);
        }
    }
}


/** Vp886ProcessPulseDecoding()
  Feeds a hook change with timestamp into the pulse decoding module and sets a
  VP886_TIMERID_PULSE_DECODE timer if necessary.
*/
static void
Vp886ProcessPulseDecoding(
    VpLineCtxType *pLineCtx,
    VpCslacLineCondType newHookSt,
    uint16 hookTimestamp)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint16 timer;
    int16 overrun;
    uint16 devTimestamp;
    VpPulseDecodeInputType input;

    if (newHookSt == VP_CSLAC_HOOK) {
        input = VP_PULSE_DECODE_INPUT_OFF_HOOK;
    } else {
        input = VP_PULSE_DECODE_INPUT_ON_HOOK;
    }

    timer = VpPulseDecodeRun(pLineCtx, &pLineObj->pulseDecodeData, input, hookTimestamp);
    if (timer != 0) {
        /* Adjust for time elapsed between the hook event and now */
        devTimestamp = Vp886GetTimestamp(pLineCtx->pDevCtx);
        overrun = devTimestamp - hookTimestamp;
        if (overrun > timer) {
            timer = 0;
        } else {
            timer = timer - overrun;
        }
        Vp886RestartTimerHalfMs(NULL, pLineCtx, VP886_TIMERID_PULSE_DECODE, timer, 0);
    } else {
        Vp886CancelTimer(NULL, pLineCtx, VP886_TIMERID_PULSE_DECODE, 0, FALSE);
    }
}


/** Vp886RingTripConfirmation()
  Return value of 1 means that the hook event requires confirmation and should
  not be reported.  Otherwise returns 0.
*/
uint8
Vp886RingTripConfirmation(
    VpLineCtxType *pLineCtx,
    VpCslacLineCondType newHookSt,
    uint16 timestamp)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    int16 overrun;
    uint16 confirmTimer;
    uint16 ringTripDelay;
    uint16 hookDebounceDelay;
    int32 ringFreq_mHz;

    pLineObj->ringTripConfirm.lastHookTimestamp = timestamp;

    /* First, check if the option is turned on or if we are in the middle of
       confirming a ring trip.  If neither is true, we have nothing to do here.
       Also skip this during line test. */
    if ((pLineObj->options.ringTripConfirm == 0 && !pLineObj->ringTripConfirm.confirming) ||
        pLineObj->inLineTest)
    {
        return 0;
    }
    
    if (pLineObj->ringTripConfirm.confirming) {
        /* Consume all hook events while confirming.  We'll check the final
           status when the timer expires. */
        if (newHookSt == VP_CSLAC_HOOK) {
            /* If we see an offhook during the confirm time, then that means we
               have already filtered out the ring trip hook glitch, no need to
               worry about it later. */
            pLineObj->ringTripConfirm.glitchCheck = FALSE;
        }
        return 1;
    }
    
    /* Offhook, initial possible ring trip */
    if (newHookSt == VP_CSLAC_HOOK && !pLineObj->ringTripConfirm.confirming &&
        Vp886LineStateInfo(pLineObj->lineState.currentState).normalEquiv == VP_LINE_RINGING)
    {
        VP_HOOK(VpLineCtxType, pLineCtx, ("Possible ring trip detected at %d, confirming for %dms", timestamp, pLineObj->options.ringTripConfirm));
        if (pLineObj->lineState.currentState == VP_LINE_RINGING) {
            Vp886SetLineStateFxsInt(pLineCtx, VP_LINE_ACTIVE);
        }
        if (pLineObj->lineState.currentState == VP_LINE_RINGING_POLREV) {
            Vp886SetLineStateFxsInt(pLineCtx, VP_LINE_ACTIVE_POLREV);
        }

        /* Clear any hook freeze.  This function will intercept all hook
           changes, but we need the lineState.condition to keep up to date so
           that we can check it when the timer runs out. */
        Vp886ClearDetectMask(pLineCtx, VP_CSLAC_HOOK);
        Vp886CancelTimer(NULL, pLineCtx, VP886_TIMERID_HOOK_FREEZE, 0, FALSE);

        /* Calculate how long it's been since the actual hook change and now.
           This will be a combination of the difference in timestamp from the
           hook buffer and the time taken to ring trip. */
        overrun = VpRoundedDivide(Vp886GetTimestamp(pDevCtx) - timestamp, 2);
        
        /* Determine the ringing frequency. */
        if (pLineObj->ringSine) {
            /* Sinusoidal ringing frequency is a 15-bit value with a 0-12000 Hz
               range.  12000000 mHz per 0x8000 reduces to 46875 mHz per 0x80. */
            ringFreq_mHz = VpRoundedDivide(pLineObj->ringFrequency * VP886_FREQ_STEP_NUM, VP886_FREQ_STEP_DEN);
        } else {
            /* For trapezoidal ringing, frequency is defined as 8000Hz / FRQB.
               This means the maximum is 8000 Hz (8000000 mHz), and the minimum
               is 8000Hz / 0x7FFF, or 244.1 mHz.  Frequency can be calculated by 
               8000000mHz / FRQB. */
            ringFreq_mHz = VpRoundedDivide(VP886_TRAPFREQ_MAX, pLineObj->ringFrequency);
        }
        /* Calculate the approximate ring trip delay in ms.  The actual delay
           can vary by up to about one cycle. */
        if (ringFreq_mHz == 0) {
            ringTripDelay = 0;
        } else {
            ringTripDelay = VpRoundedDivide(1000000, ringFreq_mHz) * pLineObj->ringTripCycles;
        }
        overrun += ringTripDelay;

        /* The delay caused by the device hook debounce only appears on one side
           of the confirm period, since it is not used for the offhook detection
           in a ring trip.  Subtract this value from the overrun to add it to
           the timer duration.  This allows more time to detect if the line went
           back onhook. */
        hookDebounceDelay = 2 * (pLineObj->registers.loopSup[1] & VP886_R_LOOPSUP_HOOK_DBNC);
        overrun -= hookDebounceDelay;

        if (overrun < pLineObj->options.ringTripConfirm) {
            confirmTimer = pLineObj->options.ringTripConfirm - overrun;
        } else {
            confirmTimer = 1;
        }
        VP_HOOK(VpLineCtxType, pLineCtx, ("Ringtrip confirm adjusted timer %dms", confirmTimer));
        /* Set the confirm timer.  When it expires, we'll check for whether
           the hook changed back. */
        Vp886ExtendTimerMs(NULL, pLineCtx, VP886_TIMERID_RINGTRIP_CONFIRM, confirmTimer, 0);
        
        pLineObj->ringTripConfirm.confirming = TRUE;
        pLineObj->ringTripConfirm.glitchCheck = TRUE;
        
        /* Return 1 so that this hook event will not be visible externally */
        return 1;
    }

    return 0;
}

/** Vp886FlushEvents()
  Implements VpFlushEvents() to clear the contents of the event queue and clear
  the event results flag.
  
  See the VP-API-II Reference Guide for more details on VpFlushEvents().
*/
VpStatusType
Vp886FlushEvents(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;

    Vp886EnterCritical(pDevCtx, VP_NULL, "Vp886FlushEvents");

    Vp886InitEventQueue(pDevCtx);
    pDevObj->getResultsRequired = FALSE;

    Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886FlushEvents");
    return VP_STATUS_SUCCESS;
}


/** Vp886GetResults()
  Implements VpGetResults() to fill the given results structure.
  
  Line test results are handled separately from normal API results to simplify
  the system.

  See the VP-API-II Reference Guide for more details on VpGetResults().
*/
VpStatusType
Vp886GetResults(
    VpEventType *pEvent,
    void *pResults)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpStatusType status = VP_STATUS_SUCCESS;

    Vp886EnterCritical(pDevCtx, VP_NULL, "Vp886GetResults");

#ifdef VP886_INCLUDE_TESTLINE_CODE
    if (pEvent->eventCategory == VP_EVCAT_TEST) {
        status = Vp886GetResultsLinetest(pEvent, pResults);
        Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886GetResults");
        return status;
    }
#endif /* VP886_INCLUDE_TESTLINE_CODE */

    switch (pEvent->eventCategory) {
        case VP_EVCAT_RESPONSE: {
            switch (pEvent->eventId) {
                case VP_LINE_EVID_RD_OPTION: {
                    status = Vp886GetResultsRdOption(pEvent, pResults);
                    break;
                }
                case VP_LINE_EVID_QUERY_CMP: {
                    status = Vp886GetResultsQuery(pEvent->pLineCtx, pEvent->eventData, (VpQueryResultsType *)pResults);
                    break;
                }
                case VP_LINE_EVID_GAIN_CMP: {
                    Vp886LineObjectType *pLineObj;
                    if (pEvent->pLineCtx == VP_NULL) {
                        status = VP_STATUS_INVALID_LINE;
                        VP_ERROR(VpDevCtxType, pDevCtx, ("VpGetResults(VP_LINE_EVID_GAIN_CMP) - NULL line context in event"));
                        break;
                    }
                    pLineObj = pEvent->pLineCtx->pLineObj;
                    *(VpRelGainResultsType *)pResults = pLineObj->setRelGainResults;
                    break;
                }
                case VP_DEV_EVID_IO_ACCESS_CMP: {
                    *(VpDeviceIoAccessDataType *)pResults = pDevObj->basicResults.deviceIoAccess;
                    pDevObj->getResultsRequired = FALSE;
                    break;
                }
                case VP_LINE_EVID_LINE_IO_RD_CMP: {
                    /* Translate the results from VpDeviceIoAccessDataType to
                       VpLineIoAccessType */
                    VpDeviceIoAccessDataType *pDeviceIoResult = &pDevObj->basicResults.deviceIoAccess;
                    VpLineIoAccessType *pLineIoResult = pResults;
                    pLineIoResult->direction = VP_IO_READ;
                    if (pEvent->channelId == 0) {
                        pLineIoResult->ioBits.mask = (pDeviceIoResult->accessMask_31_0 & 0x3);
                        pLineIoResult->ioBits.data = (pDeviceIoResult->deviceIOData_31_0 & 0x3);
                    } else {
                        pLineIoResult->ioBits.mask = ((pDeviceIoResult->accessMask_31_0 >> 2) & 0x3);
                        pLineIoResult->ioBits.data = ((pDeviceIoResult->deviceIOData_31_0 >> 2) & 0x3);
                    }
                    pDevObj->getResultsRequired = FALSE;
                    break;
                }
                case VP_LINE_EVID_RD_LOOP: {
                    *(VpLoopCondResultsType *)pResults = pDevObj->basicResults.getLoopCond;
                    pDevObj->getResultsRequired = FALSE;
                    break;
                }
                case VP_EVID_CAL_CMP: {
                    status = Vp886GetCalProfile(pEvent->pLineCtx, (uint8*)pResults);
                    break;
                }
                default:
                    VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetResults() - Invalid response event ID %d", pEvent->eventId));
                    status = VP_STATUS_INVALID_ARG;
                    break;
            }
            break;
        }
        default:
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetResults() - Invalid event category %d", pEvent->eventCategory));
            status = VP_STATUS_INVALID_ARG;
            break;
    }

    Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886GetResults");
    return status;
}


/** Vp886GetResultsRdOption()
  Fills in the results structure for VP_LINE_EVID_RD_OPTION - the result of
  VpGetOption().
*/
VpStatusType
Vp886GetResultsRdOption(
    VpEventType *pEvent,
    void *pResults)
{
    VpDevCtxType *pDevCtx = pEvent->pDevCtx;
    VpLineCtxType *pLineCtx = pEvent->pLineCtx;
    VpOptionIdType optionId = pEvent->eventData;
    VpStatusType status = VP_STATUS_SUCCESS;

    switch (optionId) {
        case VP_OPTION_ID_PULSE_MODE:
        case VP_OPTION_ID_TIMESLOT:
        case VP_OPTION_ID_CODEC:
        case VP_OPTION_ID_LOOPBACK:
        case VP_OPTION_ID_LINE_STATE:
        case VP_OPTION_ID_EVENT_MASK:
        case VP_OPTION_ID_RING_CNTRL:
        case VP_OPTION_ID_PCM_TXRX_CNTRL:
        case VP_OPTION_ID_SWITCHER_CTRL:
        case VP_OPTION_ID_ABS_GAIN:
        case VP_OPTION_ID_LINE_IO_CFG:
        case VP_OPTION_ID_DCFEED_PARAMS:
        case VP_OPTION_ID_RINGING_PARAMS:
        case VP_OPTION_ID_GND_FLT_PROTECTION:
        case VP_OPTION_ID_DTMF_MODE:
        case VP_OPTION_ID_RINGTRIP_CONFIRM:
        case VP_OPTION_ID_HIGHPASS_FILTER:
        case VP_OPTION_ID_DTMF_PARAMS:
            if (pLineCtx == VP_NULL) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetResultsRdOption() - Missing line ctx for option ID %d", optionId));
                status = VP_STATUS_INVALID_ARG;
                break;
            }
            status = Vp886GetResultsRdOptionLine(pLineCtx, optionId, pResults);
            break;
        case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
        case VP_DEVICE_OPTION_ID_PULSE:
        case VP_DEVICE_OPTION_ID_PULSE2:
        case VP_DEVICE_OPTION_ID_DEVICE_IO:
#ifdef VP886_INCLUDE_ADAPTIVE_RINGING
        case VP_DEVICE_OPTION_ID_ADAPTIVE_RINGING:
#endif
        case VP_DEVICE_OPTION_ID_RING_PHASE_SYNC:
        case VP_DEVICE_OPTION_ID_FSYNC_RATE:
            status = Vp886GetResultsRdOptionDev(pDevCtx, optionId, pResults);
            break;
        default:
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetResultsRdOption() - Invalid option ID %d", optionId));
            status = VP_STATUS_INVALID_ARG;
            break;
    }

    return status;
}


/** Vp886GetResultsRdOptionLine()
  Fills in the option results structure for line options.
  
  This is used for both VpGetResults() and VpGetOptionImmediate().
*/
VpStatusType
Vp886GetResultsRdOptionLine(
    VpLineCtxType *pLineCtx,
    VpOptionIdType option,
    void *pResults)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineOptionsCacheType *pOptions = &pLineObj->options;

    switch (option) {
        case VP_OPTION_ID_PULSE_MODE:
            *(VpOptionPulseModeType *)pResults = pOptions->pulseMode;
            break;
        case VP_OPTION_ID_TIMESLOT:
            *(VpOptionTimeslotType *)pResults = pOptions->timeslot;
            break;
        case VP_OPTION_ID_CODEC:
            *(VpOptionCodecType *)pResults = pOptions->codec;
            break;
        case VP_OPTION_ID_LOOPBACK:
            *(VpOptionLoopbackType *)pResults = pOptions->loopback;
            break;
        case VP_OPTION_ID_LINE_STATE:
            *(VpOptionLineStateType *)pResults = pOptions->lineState;
            break;
        case VP_OPTION_ID_EVENT_MASK: {
            VpOptionEventMaskType *pDevMask = &pDevObj->options.eventMask;
            VpOptionEventMaskType *pLineMask = &pLineObj->options.eventMask;
            VpOptionEventMaskType *pResMask = pResults;

            pResMask->faults = pDevMask->faults | pLineMask->faults;
            pResMask->signaling = pDevMask->signaling | pLineMask->signaling;
            pResMask->response = pDevMask->response | pLineMask->response;
            pResMask->test = pDevMask->test | pLineMask->test;
            pResMask->process = pDevMask->process | pLineMask->process;
            pResMask->fxo = pDevMask->fxo | pLineMask->fxo;
            break;
        }
        case VP_OPTION_ID_RING_CNTRL:
            *(VpOptionRingControlType *)pResults = pOptions->ringControl;
            break;
        case VP_OPTION_ID_PCM_TXRX_CNTRL:
            *(VpOptionPcmTxRxCntrlType *)pResults = pOptions->pcmTxRxCntrl;
            break;
        case VP_OPTION_ID_SWITCHER_CTRL:
            *(bool *)pResults = pOptions->switcherCtrl;
            break;
        case VP_OPTION_ID_ABS_GAIN:
            *(VpOptionAbsGainType *)pResults = pOptions->absGain;
            break;
        case VP_OPTION_ID_LINE_IO_CFG: {
            /* Pull the line IO settings from the device option cache */
            VpOptionDeviceIoType *pDeviceIo = &pDevObj->options.deviceIo;
            VpOptionLineIoConfigType *pLineIoCfg = pResults;
            if (pDevObj->ioCapability == VP886_IO_CAPABILITY_TWO_PER_CH) {
            if (pLineObj->channelId == 0) {
                pLineIoCfg->direction = (pDeviceIo->directionPins_31_0 & 0x3);
                pLineIoCfg->outputType = (pDeviceIo->outputTypePins_31_0 & 0x3);
            } else {
                pLineIoCfg->direction = (pDeviceIo->directionPins_31_0 & 0xC) >> 2;
                pLineIoCfg->outputType = (pDeviceIo->outputTypePins_31_0 & 0xC) >> 2;
                }
            } else if (pDevObj->ioCapability == VP886_IO_CAPABILITY_CH0_IO2 && pLineObj->channelId == 0) {
                pLineIoCfg->direction = (pDeviceIo->directionPins_31_0 & 0x1);
                pLineIoCfg->outputType = (pDeviceIo->outputTypePins_31_0 & 0x1);
            }
            break;
        }
        case VP_OPTION_ID_DCFEED_PARAMS: {
            VpOptionDcFeedParamsType *pDcFeedParams = pResults;
            uint8 ila;
            uint8 tsh;
            uint8 lptsh;
            uint8 tgk;
            
            pDcFeedParams->validMask = 0;
            
            /* VOC target is stored in 1V steps. */
            pDcFeedParams->validMask |= VP_OPTION_CFG_VOC;
            pDcFeedParams->voc = pLineObj->targetVoc * VP886_VOC_TARGET_STEP;
            
            /* ILA is a 5-bit value with a 18-49 mA scale (1 mA per step + 18 mA
               offset) */
            pDcFeedParams->validMask |= VP_OPTION_CFG_ILA;
            ila = (pLineObj->registers.dcFeed[1] & VP886_R_DCFEED_ILA);
            pDcFeedParams->ila = ila * VP886_ILA_STEP + VP886_ILA_OFFSET;
            
            /* TSH is a 3-bit value with a 7-14 mA scale (1 mA per step + 7 mA
               offset) */
            pDcFeedParams->validMask |= VP_OPTION_CFG_HOOK_THRESHOLD;
            tsh = (pLineObj->registers.loopSup[0] & VP886_R_LOOPSUP_HOOK_THRESH);
            pDcFeedParams->hookThreshold = tsh * VP886_TSH_STEP + VP886_TSH_OFFSET;

            /* LPTSH is a 3-bit value with a 14-28 V scale (2 V per step + 14 V
               offset) */
            pDcFeedParams->validMask |= VP_OPTION_CFG_LP_HOOK_THRESHOLD;
            lptsh = (pLineObj->registers.loopSup[3] & VP886_R_LOOPSUP_LPM_HOOK_THRESH) >> 5;
            pDcFeedParams->lpHookThreshold = lptsh * VP886_LPTSH_STEP + VP886_LPTSH_OFFSET;

            /* TGK is a 3-bit value with a 0-42 mA scale (6 mA per step) */
            pDcFeedParams->validMask |= VP_OPTION_CFG_GKEY_THRESHOLD;
            tgk = (pLineObj->registers.loopSup[0] & VP886_R_LOOPSUP_GKEY_THRESH) >> 3;
            pDcFeedParams->gkeyThreshold = tgk * VP886_TGK_STEP + VP886_TGK_OFFSET;

            if (VP886_IS_TRACKER(pDevObj)) {
                pDcFeedParams->validMask |= VP_OPTION_CFG_BATT_FLOOR;
                pDcFeedParams->battFloor = pLineObj->floorVoltage * VP886_BATTFLR_STEP + VP886_BATTFLR_OFFSET;
            }

            break;
        }
        case VP_OPTION_ID_RINGING_PARAMS: {
            VpOptionRingingParamsType *pRingingParams = pResults;
            uint8 ilr;
            int16 riseTime;
            
            pRingingParams->validMask = 0;

            pRingingParams->validMask |= VP_OPTION_CFG_FREQUENCY;
            if (pLineObj->ringSine) {
                /* Sinusoidal ringing frequency is a 15-bit value with a 0-12000 Hz
                   range.  12000000 mHz per 0x8000 reduces to 46875 mHz per 0x80. */
                pRingingParams->frequency = VpRoundedDivide(pLineObj->ringFrequency * VP886_FREQ_STEP_NUM, VP886_FREQ_STEP_DEN);
            } else {
                /* For trapezoidal ringing, frequency is defined as 8000Hz / FRQB.
                   This means the maximum is 8000 Hz (8000000 mHz), and the minimum
                   is 8000Hz / 0x7FFF, or 244.1 mHz.  Frequency can be calculated by 
                   8000000mHz / FRQB. */
                pRingingParams->frequency = VpRoundedDivide(VP886_TRAPFREQ_MAX, pLineObj->ringFrequency);
            }

            /* Amplitude is a 16-bit value with a +/- 154.4V range.  This
               means 154400 mV per 0x8000, which reduces to 4825 mV per 0x400. */
            pRingingParams->validMask |= VP_OPTION_CFG_AMPLITUDE;
            pRingingParams->amplitude = VpRoundedDivide(pLineObj->ringAmplitude * VP886_AMP_STEP_NUM, VP886_AMP_STEP_DEN); 

            /* DC Bias is a 16-bit value with a +/- 154.4V range.  This
               means 154400 mV per 0x8000, which reduces to 4825 mV per 0x400. */
            pRingingParams->validMask |= VP_OPTION_CFG_DC_BIAS;
            pRingingParams->dcBias = VpRoundedDivide(pLineObj->ringBias * VP886_BIAS_STEP_NUM, VP886_BIAS_STEP_DEN); 

            /* Ring trip threshold is a 7-bit value with a 0-63.5 mA scale (0.5 mA
               per step).  Use the value saved in the line object, since the 
               register value may be modified for the low ILR workaround. */
            pRingingParams->validMask |= VP_OPTION_CFG_RINGTRIP_THRESHOLD;
            pRingingParams->ringTripThreshold = pLineObj->rtth * VP886_RTTH_STEP + VP886_RTTH_OFFSET;

            if (!pLineObj->lowIlr) {
                /* Ring current limit is a 5-bit value with a 50-112 mA scale (2 mA per
                   step + 50 mA offset). */
                pRingingParams->validMask |= VP_OPTION_CFG_RING_CURRENT_LIMIT;
                ilr = (pLineObj->registers.loopSup[3] & VP886_R_LOOPSUP_RING_CUR_LIM);
                pRingingParams->ringCurrentLimit = ilr * VP886_ILR_STEP + VP886_ILR_OFFSET;
            } else {
                /* Low range ring current limit is a 5-bit value with a 18-49 mA scale.
                   But in order to work around an issue with how the device targets 7/8
                   of the programmed ILR value, we're going to force a +4mA adjustment
                   in the calibration register.  This effectively bumps up the offset
                   in programming the ILR field (1 mA per step + 22 mA offset). */
                pRingingParams->validMask |= VP_OPTION_CFG_RING_CURRENT_LIMIT;
                ilr = (pLineObj->registers.loopSup[3] & VP886_R_LOOPSUP_RING_CUR_LIM);
                pRingingParams->ringCurrentLimit = ilr * VP886_ILR_LOW_STEP + VP886_ILR_LOW_OFFSET;
            }
            
            
            if (!pLineObj->ringSine) {
                uint8 ringGen[VP886_R_RINGGEN_LEN];
                VpSlacRegRead(NULL, pLineCtx, VP886_R_RINGGEN_RD, VP886_R_RINGGEN_LEN, ringGen);
                /* Trapezoidal rise time is defined as 2.7307sec / FRQA. This means the
                   maximum is 2.7307 sec (2730700 usec), and the minimum
                   is 2.7307sec / 0x7FFF, or 83.3 usec.  Rise time can be calculated by 
                   2730700usec / FRQA. */
                pRingingParams->validMask |= VP_OPTION_CFG_TRAP_RISE_TIME;
                riseTime = (ringGen[3] << 8) | ringGen[4];
                pRingingParams->trapRiseTime = VpRoundedDivide(VP886_TRAPRISE_MAX, riseTime);
            }
            
            break;
        }
        case VP_OPTION_ID_GND_FLT_PROTECTION:
            *(VpOptionGndFltProtType *)pResults = pOptions->gndFltProt;
            break;
        case VP_OPTION_ID_DTMF_MODE:
            *(VpOptionDtmfModeType *)pResults = pOptions->dtmfMode;
            break;
        case VP_OPTION_ID_RINGTRIP_CONFIRM:
            *(uint16 *)pResults = pOptions->ringTripConfirm;
            break;
        case VP_OPTION_ID_HIGHPASS_FILTER:
            *(VpOptionHighPassFilterType *)pResults = pOptions->highPassFilter;
            break;
        case VP_OPTION_ID_DTMF_PARAMS:
            *(VpOptionDtmfParamsType *)pResults = pOptions->dtmfParams;
            ((VpOptionDtmfParamsType *)pResults)->validMask =
                VP_OPTION_CFG_MIN_DETECT | VP_OPTION_CFG_ROW_TO_COL_LIMIT |
                VP_OPTION_CFG_COL_TO_ROW_LIMIT;
            break;
        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886GetResultsRdOptionLine() - Invalid option ID %d", option));
            return VP_STATUS_INVALID_ARG;
    }
    
    return VP_STATUS_SUCCESS;
}


/** Vp886GetResultsRdOptionDev()
  Fills in the option results structure for device options.
  
  This is used for both VpGetResults() and VpGetOptionImmediate().
*/
VpStatusType
Vp886GetResultsRdOptionDev(
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *pResults)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886DevOptionsCacheType *pOptions = &pDevObj->options;

    switch (option) {
        case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
            *(VpOptionCriticalFltType *)pResults = pOptions->criticalFlt;
            break;
        case VP_DEVICE_OPTION_ID_PULSE:
            *(VpOptionPulseType *)pResults = pOptions->pulse;
            break;
        case VP_DEVICE_OPTION_ID_PULSE2:
            *(VpOptionPulseType *)pResults = pOptions->pulse2;
            break;
        case VP_DEVICE_OPTION_ID_DEVICE_IO:
            *(VpOptionDeviceIoType *)pResults = pOptions->deviceIo;
            break;
#ifdef VP886_INCLUDE_ADAPTIVE_RINGING
        case VP_DEVICE_OPTION_ID_ADAPTIVE_RINGING:
            *(VpOptionAdaptiveRingingType *)pResults = pOptions->adaptiveRinging;
            ((VpOptionAdaptiveRingingType *)pResults)->validMask =
                VP_ADAPTIVE_RINGING_CFG_POWER | VP_ADAPTIVE_RINGING_CFG_MIN_V_PCT |
                VP_ADAPTIVE_RINGING_CFG_MODE;
            break;
#endif
        case VP_DEVICE_OPTION_ID_RING_PHASE_SYNC:
            *(VpOptionRingPhaseSyncType *)pResults = pOptions->ringPhaseSync;
            break;
        case VP_DEVICE_OPTION_ID_FSYNC_RATE:
            *(VpOptionFsyncRateType *)pResults = pOptions->fsyncRate;
            break;
        default:
            VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetResultsRdOptionDev() - Invalid option ID %d", option));
            return VP_STATUS_INVALID_ARG;
    }
    
    return VP_STATUS_SUCCESS;
}


/** Vp886GetResultsQuery()
  Fills in the results structure for VP_LINE_EVID_QUERY_CMP - the result of
  VpQuery().  Also used to implement Vp886QueryImmediate().
*/
VpStatusType
Vp886GetResultsQuery(
    VpLineCtxType *pLineCtx,
    VpQueryIdType queryId,
    VpQueryResultsType *pResults)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpStatusType status = VP_STATUS_SUCCESS;

    switch (queryId) {
        case VP_QUERY_ID_DEV_TRAFFIC:
            pResults->trafficBytes = pDevObj->trafficBytes;
            pDevObj->trafficBytes = 0;
            break;
        case VP_QUERY_ID_LINE_CAL_COEFF:
            status = Vp886GetCalProfile(pLineCtx, pResults->calProf);
            break;
        case VP_QUERY_ID_TIMESTAMP:
            pResults->timestamp = Vp886GetTimestamp32(pDevCtx);
            break;
        case VP_QUERY_ID_LINE_TOPOLOGY:
            VpMemCpy(&pResults->lineTopology, &pLineObj->lineTopology, sizeof(VpLineTopologyType));
            break;
        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886GetResultsQuery - Invalid queryId %d", queryId));
            return VP_STATUS_INVALID_ARG;
    }
    
    return status;
}


#ifdef VP886_INCLUDE_TESTLINE_CODE
/** Vp886GetResultsLinetest()
  Fills in the results structure for line test primatives.
*/
VpStatusType
Vp886GetResultsLinetest(
    VpEventType *pEvent,
    void *pResults)
{

    VpStatusType status = VP_STATUS_SUCCESS;
    Vp886LineObjectType *pLineObj;

    if (pEvent->pLineCtx == NULL) {
        VP_ERROR(None, NULL, ("Vp886GetResultsLinetest() - Should never get here."));
        return VP_STATUS_INVALID_ARG;
    }

    pLineObj = pEvent->pLineCtx->pLineObj;

    switch (pEvent->eventId) {
        case VP_LINE_EVID_TEST_CMP:
            *((VpTestResultType *)pResults) = pLineObj->testResults;
            break;
         default:
            VP_ERROR(VpLineCtxType, pEvent->pLineCtx , ("Vp886GetResultsLinetest() - Invalid test event ID %d", pEvent->eventId));
            status = VP_STATUS_INVALID_ARG;
            break;
    }

    return status;
}
#endif /* VP886_INCLUDE_TESTLINE_CODE */


/** Vp886InitEventQueue()
  Initializes the device event queue size, indices, and status flags.  This
  must be called immediately in Vp886InitDevice(), prior to calling
  Vp886PushEvent(), Vp886PopEvent(), or Vp886GetEvent().
*/
void
Vp886InitEventQueue(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886EventQueueType *pEventQueue = &pDevObj->eventQueue;

    pEventQueue->numQueued = 0;
    pEventQueue->pushIndex = 0;
    pEventQueue->popIndex = 0;
    pEventQueue->overflowed = FALSE;

    return;
}


/** Vp886PushEvent()
  Pushes a new event into the device event queue if it is not masked.
  
  For device-level events, provide channelId=VP886_DEV_EVENT to prevent the
  line specific information from being filled out in the final VpEventType
  structure by Vp886PopEvent().
 
  If an overflow occurs, the new event overwrites the oldest event in the
  queue, and the queue's 'overflowed' flag is set to TRUE.  When this flag is
  set, Vp886PopEvent() will return an EVQ_OFL_FLT event before anything in the
  queue.

  Returns FALSE if the event is masked, TRUE otherwise.
*/
bool
Vp886PushEvent(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    VpEventCategoryType eventCategory,
    uint16 eventId,
    uint16 eventData,
    uint16 parmHandle,
    bool hasResults)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886EventQueueType *pEventQueue;
    uint8 pushIndex;
    uint8 popIndex;

    if (Vp886IsEventMasked(pDevCtx, channelId, eventCategory, eventId)) {
        return FALSE;
    }

    pEventQueue = &pDevObj->eventQueue;
    pushIndex = pEventQueue->pushIndex;
    popIndex = pEventQueue->popIndex;

    /* If the event queue is overflowing, consume the oldest event */
    if (pEventQueue->numQueued == VP886_EVENT_QUEUE_SIZE) {
        VP_WARNING(VpDevCtxType, pDevCtx, ("Event queue overflow!"));
        pEventQueue->overflowed = TRUE;
        popIndex = (popIndex + 1) % VP886_EVENT_QUEUE_SIZE;
    }

    pEventQueue->events[pushIndex].channelId = channelId;
    pEventQueue->events[pushIndex].eventCategory = eventCategory;
    pEventQueue->events[pushIndex].eventId = eventId;
    pEventQueue->events[pushIndex].eventData = eventData;
    pEventQueue->events[pushIndex].parmHandle = parmHandle;
    pEventQueue->events[pushIndex].hasResults = hasResults;

    pushIndex = (pushIndex + 1) % VP886_EVENT_QUEUE_SIZE;
    if (!pEventQueue->overflowed) {
        pEventQueue->numQueued++;
    }

    pEventQueue->pushIndex = pushIndex;
    pEventQueue->popIndex = popIndex;

    /* If we're being called outside of VpGetEvent(), then we need to create
       a device interrupt to prompt the application to call VpGetEvent(). */
    if (!pDevObj->inGetEvent) {
        Vp886ForceTimerInterrupt(pDevCtx);
    }

    return TRUE;
}


/** Vp886PopEvent()
  This function pops an event from the front of the queue to fill the
  provided event structure.
 
  If an overflow occured, the event returned will be VP_DEV_EVID_EVQ_OFL_FLT
  instead of anything from the queue.  The queue overflow flag will then be
  cleared and the queue will resume normal operation.

  Returns TRUE if successful, or FALSE if the queue is empty.  If the queue
  is empty, the event structure is not modified.
*/
bool
Vp886PopEvent(
    VpDevCtxType *pDevCtx,
    VpEventType *pEvent)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886EventQueueType *pEventQueue = &pDevObj->eventQueue;
    uint8 popIndex;
    uint8 channelId;

    if (pEventQueue->overflowed) {
        pEventQueue->overflowed = FALSE;

        /* If it is not masked, generate an event queue overflow event before
           popping anything off the queue */
        if (!(pDevObj->options.eventMask.faults & VP_DEV_EVID_EVQ_OFL_FLT)) {
            pEvent->pDevCtx = pDevCtx;
            pEvent->pLineCtx = VP_NULL;
            pEvent->channelId = 0;
            pEvent->deviceId = pDevObj->deviceId;
            pEvent->eventCategory = VP_EVCAT_FAULT;
            pEvent->eventId = VP_DEV_EVID_EVQ_OFL_FLT;
            pEvent->eventData = 0;
            pEvent->parmHandle = Vp886GetTimestamp(pDevCtx);
            pEvent->hasResults = FALSE;
            pEvent->status = VP_STATUS_SUCCESS;

            return TRUE;
        }
    }

    if (pEventQueue->numQueued == 0) {
        return FALSE;
    }

    popIndex = pEventQueue->popIndex;
    channelId = pEventQueue->events[popIndex].channelId;
    if (channelId < VP886_MAX_NUM_CHANNELS) {
        VpLineCtxType *pLineCtx;
        Vp886LineObjectType *pLineObj;
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            pLineObj = pLineCtx->pLineObj;
            pEvent->lineId = pLineObj->lineId;
        }
        pEvent->pLineCtx = pLineCtx;
        pEvent->channelId = channelId;
    } else {
        pEvent->channelId = 0;
        pEvent->pLineCtx = VP_NULL;
    }
    pEvent->pDevCtx = pDevCtx;
    pEvent->deviceId = pDevObj->deviceId;
    pEvent->eventCategory = pEventQueue->events[popIndex].eventCategory;
    pEvent->eventId = pEventQueue->events[popIndex].eventId;
    pEvent->eventData = pEventQueue->events[popIndex].eventData;
    pEvent->parmHandle = pEventQueue->events[popIndex].parmHandle;
    pEvent->hasResults = pEventQueue->events[popIndex].hasResults;
    pEvent->status = VP_STATUS_SUCCESS;

    popIndex = (popIndex + 1) % VP886_EVENT_QUEUE_SIZE;
    pEventQueue->numQueued--;
    pEventQueue->popIndex = popIndex;

    return TRUE;
}


/** Vp886IsEventMasked()
  Checks if an event is masked based on its category and eventId.
  Returns TRUE if the event is masked, FALSE if it is not masked.
*/
bool
Vp886IsEventMasked(
    VpDevCtxType *pDevCtx,
    uint8 channelId,
    VpEventCategoryType eventCategory,
    uint16 eventId)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpLineCtxType *pLineCtx;
    VpOptionEventMaskType fullMask = pDevObj->options.eventMask;
    VpOptionEventMaskType lineMask = {0, 0, 0, 0, 0, 0, 0};
    uint16 catMask;

    /* If channelId is not VP886_DEV_EVENT and the channel exists, retrieve
       the line specific event masks. */
    if (channelId < VP886_MAX_NUM_CHANNELS) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
        if (pLineCtx != VP_NULL) {
            Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
            lineMask = pLineObj->options.eventMask;
        }
    }

    /* Combine the line and device event masks for the category of the event. */
    switch (eventCategory) {
        case VP_EVCAT_FAULT:
            catMask = fullMask.faults | lineMask.faults;
            break;
        case VP_EVCAT_SIGNALING:
            catMask = fullMask.signaling | lineMask.signaling;
            break;
        case VP_EVCAT_RESPONSE:
            catMask = fullMask.response | lineMask.response;
            break;
        case VP_EVCAT_TEST:
            catMask = fullMask.test | lineMask.test;
            break;
        case VP_EVCAT_PROCESS:
            catMask = fullMask.process | lineMask.process;
            break;
        case VP_EVCAT_FXO:
            catMask = fullMask.fxo | lineMask.fxo;
            break;
        default:
            VP_WARNING(VpDevCtxType, pDevCtx, ("Vp886IsEventMasked - unexpected event category %d", eventCategory));
            return TRUE;
    }

    /* Compare the event ID to the combined mask */
    if (catMask & eventId) {
        VP_INFO(VpDevCtxType, pDevCtx, ("Vp886IsEventMasked - event masked (%d:0x%04X ch%d)", eventCategory, eventId, channelId));
        return TRUE;
    } else {
        return FALSE;
    }
}


/** Vp886SetDetectMask()
  Sets flags to completely ignore particular signals while processing the
  signaling register.  This prevents those signals from generating events,
  changing the results of VpGetLineStatus(), or causing any automatic processing
  or response in the API.

  setMask should be a combination of VpCslacLineCondType values, for example
  VP_CSLAC_HOOK to implement a hook freeze.
*/
void
Vp886SetDetectMask(
    VpLineCtxType *pLineCtx,
    uint16 setMask)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    
    pLineObj->detectMasks |= setMask;
    
    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Setting detect mask %04X; new %04X",
        setMask, pLineObj->detectMasks));

    /* TODO: Set device interrupt mask? Not sure if we want to. */
}


/** Vp886ClearDetectMask()
  Clears flags set by Vp886SetDetectMask() to resume processing the specified
  signals.
  
  clearMask should be a combination of VpCslacLineCondType values, for example
  VP_CSLAC_HOOK to end a hook freeze.
*/
void
Vp886ClearDetectMask(
    VpLineCtxType *pLineCtx,
    uint16 clearMask)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 prevMask = pLineObj->detectMasks;
    
    pLineObj->detectMasks &= ~clearMask;
    
    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Clearing detect mask %04X; new %04X",
        clearMask, pLineObj->detectMasks));

    if (prevMask == pLineObj->detectMasks) {
        return;
    }
    
    /* TODO: Set device interrupt mask? Not sure if we want to. */
    
    /* When we've finished masking a detector, we need to make sure that the
       API and application are updated with the current status.  For example, if
       a phone went offhook while masking hook events, and is STILL off hook
       when we end the mask, we need to update the line status and generate an
       event.  Since the device interrupt for the hook has already occurred, we
       need to force a VpGetEvent() call.
       It is possible that we are already in VpGetEvent() with a sigreg read
       left to process, in which case there is no need to force an interrupt.
       Also, if the event queue is non-empty, it means that the application
       should be calling VpGetEvent() again anyway. */
    if (!pDevObj->sigregReadPending &&
        pDevObj->eventQueue.numQueued == 0)
    {
        Vp886ForceTimerInterrupt(pDevCtx);
    }
}


/** Vp886GndFltProtHandler()
  Implements the ground fault protection algorithm, which is enabled by
  VP_OPTION_ID_GND_FLT_PROTECTION.

  When a groundkey is detected, this will set the line to disconnect and
  generate a GND_FLT event if the groundkey persists for a specified amount of
  time.  Then it will use the tip open and ring open states periodically to
  check each lead for whether the fault is still present.  When one of these
  checks finds no faults, the algorithm will end and generate a GND_FLT cleared
  event.

  Every other part of the API that needs to interact with this algorithm does so
  by calling this function with a different "input" argument.
*/
void
Vp886GndFltProtHandler(
    VpLineCtxType *pLineCtx,
    Vp886GroundFaultProtInputType input)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    uint32 confirmTime;
    uint32 pollTime;

    /* Make sure this doesn't run at all during line test or calibration */
    if (pLineObj->inLineTest || (pLineObj->busyFlags & VP886_LINE_IN_CAL)) {
        return;
    }

    /* Exit early if we can to prevent useless debug output when the state
       is inactive or the option is disabled. */
    if (pLineObj->gndFltProt.state == VP886_GNDFLTPROT_ST_INACTIVE) {
        if (!pLineObj->options.gndFltProt.enable) {
            return;
        }
        switch (input) {
            case VP886_GNDFLTPROT_INP_GKEY_REL:
            case VP886_GNDFLTPROT_INP_LINESTATE:
            case VP886_GNDFLTPROT_INP_STOP:
                return;
            default: break;
        }
    }

    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Vp886GndFltProtHandler: State %d, Input %d",
        pLineObj->gndFltProt.state, input));

    /* Get the confirm and poll delay times from the option setting. The option
       settings are in steps of 10ms. */
    confirmTime = 10 * pLineObj->options.gndFltProt.confirmTime;
    pollTime = 10 * pLineObj->options.gndFltProt.pollTime;

    switch (pLineObj->gndFltProt.state) {
        case VP886_GNDFLTPROT_ST_INACTIVE: {
            /* In this state, either ground fault protection is not enabled or
               no potential ground fault has been detected. */
            switch (input) {
                case VP886_GNDFLTPROT_INP_GKEY_DET:
                    /* Groundkey detected.  Set the timer to the duration required to
                       qualify is as a ground fault. */
                    Vp886AddTimerMs(VP_NULL, pLineCtx, VP886_TIMERID_GND_FLT_PROT, confirmTime, 0, 0);
                    pLineObj->gndFltProt.state = VP886_GNDFLTPROT_ST_GKEY_DETECTED;
                    break;
                default: break;
            }
            break;
        }

        case VP886_GNDFLTPROT_ST_GKEY_DETECTED: {
            /* In this state, a groundkey has been detected.  If it remains for
               long enough, it will be declared a ground fault. */
            switch (input) {
                case VP886_GNDFLTPROT_INP_GKEY_REL:
                    Vp886GndFltProtStop(pLineCtx, FALSE);
                    break;
                case VP886_GNDFLTPROT_INP_TIMER:
                    /* The gkey has existed for long enough to be declared a ground fault.
                       Generate the GND_FLT event. */
                    Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_FAULT, VP_LINE_EVID_GND_FLT, 1, Vp886GetTimestamp(pDevCtx), FALSE);
                    
                    /* Set a flag to indicate that a ground fault has been declared.
                       If this sequence is aborted, this flag will be used to generate
                       the fault cleared event when we see the gkey clear. */
                    pLineObj->gndFltProt.faultDeclared = TRUE;
                    
                    /* Set the line to disconnect, at the user level.  But first, set 
                       a flag so that SetLineState knows not to turn right back around
                       and call this function in response. */
                    pLineObj->gndFltProt.settingDisconnect = TRUE;
                    Vp886SetLineStateFxs(pLineCtx, VP_LINE_DISCONNECT);

                    /* If the poll delay time is 0, it means that we don't want to
                       run the checks at all after going to disconnect. */
                    if (pollTime == 0) {
                        Vp886GndFltProtStop(pLineCtx, FALSE);
                    }

                    /* Set the number of times to poll.  0 means infinite. */
                    pLineObj->gndFltProt.iterations = pLineObj->options.gndFltProt.pollNum;

                    /* Schedule the first followup fault check. */
                    Vp886AddTimerMs(VP_NULL, pLineCtx, VP886_TIMERID_GND_FLT_PROT, pollTime, 0, 0);
                    pLineObj->gndFltProt.state = VP886_GNDFLTPROT_ST_DISCONNECT;
                    break;
                case VP886_GNDFLTPROT_INP_STOP:
                    Vp886GndFltProtStop(pLineCtx, FALSE);
                    break;
                default: break;
            }
            break;
        }

        case VP886_GNDFLTPROT_ST_DISCONNECT: {
            /* In this state, a ground fault has been declared and we are in the
               disconnect state, waiting before performing a check to see if the
               fault is still present. */
            switch (input) {
                case VP886_GNDFLTPROT_INP_TIMER: {
                    uint16 debounceTime;
                    /* Set the line to tip open to test for a fault on the ring lead.
                       This includes the ICR3 workaround to disable LONG_ON. */
                    pLineObj->registers.sysState[0] = VP886_R_STATE_SS_TIPOPEN;
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN,
                        pLineObj->registers.sysState);
                    pLineObj->registers.icr3[2] |= VP886_R_ICR3_LONG_ON;
                    pLineObj->registers.icr3[3] &= ~VP886_R_ICR3_LONG_ON;
                    VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN,
                        pLineObj->registers.icr3);
                    
                    /* Set the timer based on the hook debounce */
                    debounceTime = 2 * (pLineObj->registers.loopSup[1] & VP886_R_LOOPSUP_HOOK_DBNC);
                    Vp886AddTimerMs(VP_NULL, pLineCtx, VP886_TIMERID_GND_FLT_PROT, debounceTime, 0, 0);
                    pLineObj->gndFltProt.state = VP886_GNDFLTPROT_ST_TIP_OPEN;
                    break;
                }
                case VP886_GNDFLTPROT_INP_LINESTATE:
                case VP886_GNDFLTPROT_INP_STOP:
                    Vp886GndFltProtStop(pLineCtx, FALSE);
                    break;
                default: break;
            }
            break;
        }

        case VP886_GNDFLTPROT_ST_TIP_OPEN: {
            /* In this state, the line has been set to tip open to test for a
               fault on the ring lead, waiting a short debounce time to check
               the signaling register. */
            switch (input) {
                case VP886_GNDFLTPROT_INP_TIMER: {
                    uint16 debounceTime;
                    /* VpGetEvent() already read the signaling register, so just
                       look at the cached data.*/
                    if (pDevObj->registers.sigreg[channelId] & VP886_R_SIGREG_HOOK) {
                        /* Hook bit indicates that the ground fault is still present
                           on the ring lead.  No need to also check tip, so go back
                           to disconnect to check again later. */
                        pLineObj->registers.sysState[0] = VP886_R_STATE_SS_DISCONNECT;
                        VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN,
                            pLineObj->registers.sysState);
                        pLineObj->registers.icr3[2] &= ~VP886_R_ICR3_LONG_ON;
                        VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN,
                            pLineObj->registers.icr3);

                        /* See whether we should continue or give up */
                        if (pLineObj->gndFltProt.iterations != 0) {
                            pLineObj->gndFltProt.iterations--;
                            if (pLineObj->gndFltProt.iterations == 0) {
                                /* Iterations expired.  Generate the fault event again
                                   with data==2 */
                                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_FAULT, VP_LINE_EVID_GND_FLT, 2, Vp886GetTimestamp(pDevCtx), FALSE);
                                /* Stop the algorithm */
                                Vp886GndFltProtStop(pLineCtx, FALSE);
                                break;
                            }
                        }

                        /* Set the timer for the next fault check */
                        Vp886AddTimerMs(VP_NULL, pLineCtx, VP886_TIMERID_GND_FLT_PROT, pollTime, 0, 0);
                        pLineObj->gndFltProt.state = VP886_GNDFLTPROT_ST_DISCONNECT;
                    } else {
                        /* No hook indication detected in the tip open state.
                           Go to ring open to test the tip lead. */
                        pLineObj->registers.sysState[0] = VP886_R_STATE_SS_RINGOPEN;
                        VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN,
                            pLineObj->registers.sysState);

                        /* Set the timer based on the hook debounce time */
                        debounceTime = 2 * (pLineObj->registers.loopSup[1] & VP886_R_LOOPSUP_HOOK_DBNC);

                        Vp886AddTimerMs(VP_NULL, pLineCtx, VP886_TIMERID_GND_FLT_PROT, debounceTime, 0, 0);
                        pLineObj->gndFltProt.state = VP886_GNDFLTPROT_ST_RING_OPEN;
                    }
                    break;
                }
                case VP886_GNDFLTPROT_INP_LINESTATE:
                case VP886_GNDFLTPROT_INP_STOP:
                    Vp886GndFltProtStop(pLineCtx, TRUE);
                    break;
                default: break;
            }
            break;
        }

        case VP886_GNDFLTPROT_ST_RING_OPEN: {
            /* In this state, the line has been set to ring open to test for a
               fault on the tip lead, waiting a short debounce time to check
               the signaling register. */
            switch (input) {
                case VP886_GNDFLTPROT_INP_TIMER: {
                    if (pDevObj->registers.sigreg[channelId] & VP886_R_SIGREG_HOOK) {
                        /* Hook bit indicates that the ground fault is still present
                           on the tip lead.  Go back to disconnect to check again later. */
                        pLineObj->registers.sysState[0] = VP886_R_STATE_SS_DISCONNECT;
                        VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN,
                            pLineObj->registers.sysState);
                        pLineObj->registers.icr3[2] &= ~VP886_R_ICR3_LONG_ON;
                        VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN,
                            pLineObj->registers.icr3);

                        /* See whether we should continue or give up */
                        if (pLineObj->gndFltProt.iterations != 0) {
                            pLineObj->gndFltProt.iterations--;
                            if (pLineObj->gndFltProt.iterations == 0) {
                                /* Iterations expired.  Generate the fault event again
                                   with data==2 */
                                Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_FAULT, VP_LINE_EVID_GND_FLT, 2, Vp886GetTimestamp(pDevCtx), FALSE);
                                /* Stop the algorithm */
                                Vp886GndFltProtStop(pLineCtx, FALSE);
                                break;
                            }
                        }

                        /* Set the timer for the next fault check */
                        Vp886AddTimerMs(VP_NULL, pLineCtx, VP886_TIMERID_GND_FLT_PROT, pollTime, 0, 0);
                        pLineObj->gndFltProt.state = VP886_GNDFLTPROT_ST_DISCONNECT;
                    } else {
                        /* No hook indication detected on either lead.  Generate events
                           and update status to show that the fault has cleared. */
                        Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_FAULT, VP_LINE_EVID_GND_FLT, 0, Vp886GetTimestamp(pDevCtx), FALSE);
                        pLineObj->gndFltProt.faultDeclared = FALSE;

                        if (pLineObj->reportDcFaults) {
                            Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_FAULT, VP_LINE_EVID_DC_FLT, 0, Vp886GetTimestamp(pDevCtx), FALSE);
                            pLineObj->lineState.condition &= ~VP_CSLAC_DC_FLT;
                        } else {
                            Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_SIGNALING, VP_LINE_EVID_GKEY_REL, 0, Vp886GetTimestamp(pDevCtx), FALSE);
                            pLineObj->lineState.condition &= ~VP_CSLAC_GKEY;
                        }

                        /* Clean up */
                        Vp886GndFltProtStop(pLineCtx, TRUE);
                    }
                    break;
                }
                case VP886_GNDFLTPROT_INP_LINESTATE:
                case VP886_GNDFLTPROT_INP_STOP:
                    Vp886GndFltProtStop(pLineCtx, TRUE);
                    break;
                default: break;
            }
            break;
        }
        default: {
            VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886GndFltProtHandler: invalid state %d", pLineObj->gndFltProt.state));
            Vp886GndFltProtStop(pLineCtx, TRUE);
            break;
        }
    }
    
    return;
}


/** Vp886GndFltProtStop()
  Generic function to end the ground fault protection algorithm, used both when
  it is interrupted and when it ends normally.

  If fixLineState is TRUE, this function will bring the line back to disconnect
  from the tip open or ring open state.
*/
void
Vp886GndFltProtStop(
    VpLineCtxType *pLineCtx,
    bool fixLineState)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;

    pLineObj->gndFltProt.state = VP886_GNDFLTPROT_ST_INACTIVE;

    Vp886CancelTimer(VP_NULL, pLineCtx, VP886_TIMERID_GND_FLT_PROT, 0, FALSE);

    if (fixLineState) {
        pLineObj->registers.sysState[0] = VP886_R_STATE_SS_DISCONNECT;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN,
            pLineObj->registers.sysState);
        pLineObj->registers.icr3[2] &= ~VP886_R_ICR3_LONG_ON;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN,
            pLineObj->registers.icr3);
    }

    return;
}

#ifdef VP886_INCLUDE_DTMF_DETECT
/** Vp886DtmfManage()
  This function looks at the current state of the line and the DTMF detection
  option, and begins or ends sampling for DTMF detection if necessary.
*/
void
Vp886DtmfManage(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    bool enable;
#ifndef VP886_DTMF_DETECT_NONCONTINUOUS
    uint8 timerVal;
#endif

    if (pLineObj->options.dtmfMode.dtmfControlMode == VP_OPTION_DTMF_DECODE_ON &&
        Vp886LineStateInfo(pLineObj->lineState.currentState).codec)
    {
        enable = TRUE;
    } else {
        enable = FALSE;
    }
    
    if (pLineObj->inLineTest) {
        enable = FALSE;
    }
    
    if (pLineObj->busyFlags & VP886_LINE_GET_LOOP_COND) {
        enable = FALSE;
    }

    if (enable && !pLineObj->dtmf.sampling) {
        /* Begin sampling. */
        VP_DTMF(VpLineCtxType, pLineCtx, ("Vp886DtmfManage: Starting sampling"));

        pLineObj->dtmf.sampling = TRUE;
        pLineObj->dtmf.currentDigit = VP_DIG_NONE;
        VpDtmfDetectReset(pLineCtx, &pLineObj->dtmf.detectData, TRUE);

        #ifndef VP886_DTMF_DETECT_NONCONTINUOUS /* Continuous mode */
            /* Set the SADC for continuous mode, and use a recurring timer to
               generate interrupts.
               In order to handle tick-based systems, we need to set the timer to
               less than half of the time that it would take the buffers to fill.
               If it is set for longer than that, it becomes possible for 2 ticks to
               happen between reads which add up to more than the overflow time. */
            pLineObj->dtmf.sadcCtrl[0] = VP886_R_SADC_ENABLE;
            #if defined(VP886_DTMF_DETECT_4KHZ_SAMPLING)
                pLineObj->dtmf.sadcCtrl[0] |= VP886_R_SADC_DRATE_SINGLE_4KHZ;
                timerVal = 14; /* 7ms */
            #elif defined(VP886_DTMF_DETECT_8KHZ_SAMPLING) /* 8kHz */
                pLineObj->dtmf.sadcCtrl[0] |= VP886_R_SADC_DRATE_SINGLE_8KHZ;
                timerVal = 7; /* 3.5ms */
            #endif
            pLineObj->dtmf.sadcCtrl[1] = VP886_R_SADC_SEL_TIP_RING_AC_V;
            pLineObj->dtmf.sadcCtrl[2] = 0x00;
            pLineObj->dtmf.sadcCtrl[3] = 0x00;
            pLineObj->dtmf.sadcCtrl[4] = 0x00;
            pLineObj->dtmf.sadcCtrl[5] = 0x00;
            pLineObj->dtmf.sadcCtrl[6] = 0x00;
            pLineObj->dtmf.sadcCtrl[7] = 0x00;
            pLineObj->dtmf.sadcCtrl[8] = 0x00;
            pLineObj->dtmf.sadcCtrl[9] = 2;

            /* Update the sampling tick timer */
            pDevObj->samplingTimerReq[pLineObj->channelId] = timerVal;
            Vp886ManageSamplingTimer(pDevCtx);

        #else /* Non-continuous mode */
            pLineObj->dtmf.sadcCtrl[0] = VP886_R_SADC_ENABLE;
            pLineObj->dtmf.sadcCtrl[0] |= VP886_R_SADC_DRATE_SINGLE_4KHZ;
            pLineObj->dtmf.sadcCtrl[0] |= VP886_R_SADC_TX_INTERRUPT;
            pLineObj->dtmf.sadcCtrl[1] = VP886_R_SADC_SEL_TIP_RING_AC_V;
            pLineObj->dtmf.sadcCtrl[2] = 0x00;
            pLineObj->dtmf.sadcCtrl[3] = 0x00;
            pLineObj->dtmf.sadcCtrl[4] = 0x00;
            pLineObj->dtmf.sadcCtrl[5] = 0x00;
            pLineObj->dtmf.sadcCtrl[6] = 0x00;
            pLineObj->dtmf.sadcCtrl[7] = VP_DTMF_BLOCK_SIZE_4KHZ_MAX; /* One block */
            pLineObj->dtmf.sadcCtrl[8] = 0x00;
            pLineObj->dtmf.sadcCtrl[9] = 2; /* Skip 2 samples */

        #endif /* VP886_DTMF_DETECT_NONCONTINUOUS */

        if (pLineObj->dtmf.useVadc) {
            uint8 vadcCtrl[VP886_R_VADC_LEN];
            #if defined(VP886_DTMF_DETECT_4KHZ_SAMPLING)
                if (pLineObj->registers.ssCfg[0] & VP886_R_SSCFG_WBAND) {
                    vadcCtrl[0] = VP886_R_VADC_DRATE_2KHZ;
                } else {
                    vadcCtrl[0] = VP886_R_VADC_DRATE_4KHZ;
                }
            #elif defined(VP886_DTMF_DETECT_8KHZ_SAMPLING) /* 8kHz */
                if (pLineObj->registers.ssCfg[0] & VP886_R_SSCFG_WBAND) {
                    vadcCtrl[0] = VP886_R_VADC_DRATE_4KHZ;
                } else {
                    vadcCtrl[0] = VP886_R_VADC_DRATE_8KHZ;
                }
            #endif
            vadcCtrl[0] |= VP886_R_VADC_SM_OVERRIDE;
            vadcCtrl[1] = VP886_R_VADC_SEL_TIP_RING_AC_V;
            vadcCtrl[2] = 0x00;
            vadcCtrl[3] = 0x00;
            vadcCtrl[4] = 0x00;
            vadcCtrl[5] = 2;
            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_VADC_WRT, VP886_R_VADC_LEN, vadcCtrl);
        
            /* Pull samples from VADC */
            pLineObj->dtmf.sadcCtrl[0] |= 0x08;
        } else {
            /* Give up VADC control, in case we're switching modes */
            uint8 vadcCtrl[VP886_R_VADC_LEN];
            VpMemSet(vadcCtrl, 0x00, VP886_R_VADC_LEN);
            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_VADC_WRT, VP886_R_VADC_LEN, vadcCtrl);
        }

        /* Program the SADC */
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_SADC_WRT, VP886_R_SADC_LEN, pLineObj->dtmf.sadcCtrl);
    }
    
    if (!enable && pLineObj->dtmf.sampling) {
        /* Stop sampling */
        VP_DTMF(VpLineCtxType, pLineCtx, ("Vp886DtmfManage: Stopping sampling"));

        pLineObj->dtmf.sampling = FALSE;

        /* Shut off the SADC */
        VpMemSet(pLineObj->dtmf.sadcCtrl, 0x00, VP886_R_SADC_LEN);
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_SADC_WRT, VP886_R_SADC_LEN, pLineObj->dtmf.sadcCtrl);

        if (pLineObj->dtmf.useVadc) {
            /* Shut off the VADC */
            uint8 vadcCtrl[VP886_R_VADC_LEN];
            VpMemSet(vadcCtrl, 0x00, VP886_R_VADC_LEN);
            VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_VADC_WRT, VP886_R_VADC_LEN, vadcCtrl);
        }

        /* Update the sampling tick timer */
        pDevObj->samplingTimerReq[pLineObj->channelId] = 0;
        Vp886ManageSamplingTimer(pDevCtx);

        /* If a digit was active, generate a digit break event */
        if (pLineObj->dtmf.currentDigit != VP_DIG_NONE) {
            Vp886PushEvent(pDevCtx, pLineObj->channelId, VP_EVCAT_SIGNALING, VP_LINE_EVID_DTMF_DIG,
                (pLineObj->dtmf.currentDigit & 0x000F), Vp886GetTimestamp(pDevCtx), FALSE);
#ifdef VP_CSLAC_SEQ_EN
            if (pLineObj->cid.active) {
                /* During CID, use this information to validate ACK detect intervals. */
                Vp886CidAckDetect(pLineCtx, pLineObj->dtmf.currentDigit, VP_DIG_SENSE_BREAK);
            }
#endif /* VP_CSLAC_SEQ_EN */
        }
    }

    return;
}

/** Vp886DtmfSampling()
  Reads the samples from the device for DTMF detection, passes them into the
  detection algorithm, and handles the results.
*/
void
Vp886DtmfSampling(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    VpDtmfDetectDataType *pDtmfData = &pLineObj->dtmf.detectData;
    VpDigitType newDigit;
    int16 eventData;
    int16 buffer[60];
    uint8 length;
    uint8 overflow;
    
    Vp886RetrieveRawSadcData(pLineCtx, buffer, &length, &overflow, FALSE);
    
    if (overflow != 0) {
        if (!pLineObj->dtmf.overflow) {
            pLineObj->dtmf.overflow = TRUE;
            VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886DtmfSampling: Overflow detected (%d)", overflow));
        }
        /* Don't process samples during overflow.  They will be non-contiguous
           and generate bad results. */
        return;
    }
    if (overflow == 0 && pLineObj->dtmf.overflow) {
        pLineObj->dtmf.overflow = FALSE;
        VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886DtmfSampling: Overflow cleared"));
        /* Reset the sampling block when coming out of overflow so that old
           samples don't mix with new ones. */
        VpDtmfDetectReset(pLineCtx, &pLineObj->dtmf.detectData, FALSE);
    }
    
    newDigit = VpDtmfDetectProcess(pLineCtx, pDtmfData, buffer, length);
    
    if (newDigit != pLineObj->dtmf.currentDigit) {
        VP_DTMF(VpLineCtxType, pLineCtx, ("Vp886DtmfSampling: Digit change detected. New %d, Old %d.  ts %d",
            newDigit, pLineObj->dtmf.currentDigit, Vp886GetTimestamp(pDevCtx)));
    
        if (pLineObj->dtmf.currentDigit != VP_DIG_NONE) {
            /* digit break event */
            eventData = (pLineObj->dtmf.currentDigit & 0x000F);
            Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_SIGNALING, VP_LINE_EVID_DTMF_DIG,
                eventData, Vp886GetTimestamp(pDevCtx), FALSE);

            #ifdef VP_CSLAC_SEQ_EN
            if (pLineObj->cid.active) {
                /* During CID, use this information to validate ACK detect intervals. */
                Vp886CidAckDetect(pLineCtx, pLineObj->dtmf.currentDigit, VP_DIG_SENSE_BREAK);
            }
            #endif /* VP_CSLAC_SEQ_EN */
        }
        if (newDigit != VP_DIG_NONE) {
            /* new digit event */
            uint32 avgEnergy;
            uint16 sqrtEnergy;
            int32 dBm; /* *10000 scale */
            uint8 output_dBm;

            /* Compute the average level of the digit */
            avgEnergy = (pLineObj->dtmf.detectData.peakEnergyRow + pLineObj->dtmf.detectData.peakEnergyCol) / 2;
            sqrtEnergy = VpComputeSquareRoot(avgEnergy);

            /* -5dBm has a measured sqrtEnergy of about 1000.  log10 of that is
               3.0 */
            dBm = (20 * (VpCompute_log10(sqrtEnergy) - 30000)) - 50000;

            /* Output is an 8-bit value with a -53.75 to +10 dBm scale
               (0.25 dBm per step - 53.75 dBm offset), but the 0 value (-53.75)
               is not used. */
            if (dBm >= 100000) {
                /* +10dBm or higher */
                output_dBm = 0xFF;
            } else if (dBm <= -535000) {
                /* The lowest value we'll output is a 1, or -53.50.  The 0 value
                   is reserved as "not measured" for compatibility */
                output_dBm = 0x01;
            } else {
                output_dBm = VpRoundedDivide(dBm + 537500, 2500);
            }

            VP_DTMF(VpLineCtxType, pLineCtx, ("Digit level: row %lu, col %lu, avg %lu, sqrt %u, dBm %ld, output %d",
                pLineObj->dtmf.detectData.peakEnergyRow, pLineObj->dtmf.detectData.peakEnergyCol,
                avgEnergy, sqrtEnergy, dBm, output_dBm));

            eventData = (newDigit & 0x000F) | 0x0010 | (output_dBm << 8);
            Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_SIGNALING, VP_LINE_EVID_DTMF_DIG,
                eventData, Vp886GetTimestamp(pDevCtx), FALSE);

            #ifdef VP_CSLAC_SEQ_EN
            if (pLineObj->cid.active) {
                /* During CID, use this information to validate ACK detect intervals. */
                Vp886CidAckDetect(pLineCtx, newDigit, VP_DIG_SENSE_MAKE);
            }
            #endif /* VP_CSLAC_SEQ_EN */
        }
        
        pLineObj->dtmf.currentDigit = newDigit;
    }
    
    #ifdef VP886_DTMF_DETECT_NONCONTINUOUS
        /* Fire off the SADC again to collect a new block. */
        VpDtmfDetectReset(pLineCtx, &pLineObj->dtmf.detectData, FALSE);
        VpSlacRegWrite(VP_NULL, pLineCtx, VP886_R_SADC_WRT, VP886_R_SADC_LEN, pLineObj->dtmf.sadcCtrl);
    #endif /* VP886_DTMF_DETECT_NONCONTINUOUS */
    
    return;
}
#endif /* VP886_INCLUDE_DTMF_DETECT */

#endif /* defined (VP_CC_886_SERIES) */

