/** \file vp886_timers.c
 * vp886_timers.c
 *
 * This file contains the wrapper functions for the timers used in the
 * VP886 API
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11584 $
 * $LastChangedDate: 2014-10-06 11:07:26 -0500 (Mon, 06 Oct 2014) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_886_SERIES)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_api.h"
#include "vp_api_int.h"
#include "vp_pulse_decode.h"
#include "vp886_api.h"
#include "vp886_api_int.h"
#include "vp_hal.h"
#include "sys_service.h"
#include "vp_debug.h"


/** Vp886InitTimerQueue()
  Initializes the timer queue by calling VpInitTimerQueue() and setting up
  the size and array pointer specific to the Vp886 API and device object.
*/
bool
Vp886InitTimerQueue(
    VpDevCtxType *pDevCtx)
{    
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpTimerQueueInfoType *pInfo = &pDevObj->timerQueueInfo;
    VpTimerQueueNodeType *pNodes = pDevObj->timerQueueNodes;

    pInfo->numNodes = VP886_TIMER_QUEUE_SIZE;

    VpInitTimerQueue(pInfo, pNodes);

    VP_TIMER(VpDevCtxType, pDevCtx, ("Timer queue initialized, size %d", VP886_TIMER_QUEUE_SIZE));

    return TRUE;
}


/** Vp886ProcessTimers()
  Processes the timer queue, subtracting elapsed time and handling any timers
  that expire.
*/
void
Vp886ProcessTimers(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpTimerQueueInfoType *pInfo = &pDevObj->timerQueueInfo;
    VpTimerQueueNodeType *pNodes = pDevObj->timerQueueNodes;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886ProcessTimers+"));

    VpProcessTimers(pDevCtx, pInfo, pNodes);

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886ProcessTimers-"));
    return;
}


/** Vp886AddTimerHalfMs()
  Inserts a new timer into the timer queue.
   - duration is specified in timestamp units (0.5ms).  Use Vp886AddTimerMs()
     to start a millisecond-based timer
   - overrun is used to perform long term compensation for the time between
     a timer interrupts occurring and when they are handled in software.  The
     overrun value is passed in to Vp886TimerHandler() and can be blindly
     passed in to this function when starting a subsequent timer.  Specified in
     0.5ms units.
   - handle is used when a timer needs to be restarted, extended, or cancelled.
     Two timers with the same timerId and handle cannot exist at once.
*/
bool
Vp886AddTimerHalfMs(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint16 timerId,
    uint32 duration,
    uint32 overrun,
    uint32 handle)
{
    Vp886DeviceObjectType *pDevObj;
    VpTimerQueueInfoType *pInfo;
    VpTimerQueueNodeType *pNodes;
    uint8 channelId;
    bool success;
    
    if (pDevCtx == VP_NULL) {
        pDevCtx = pLineCtx->pDevCtx;
    }
    
    if (pLineCtx == VP_NULL) {
        channelId = VP886_DEV_TIMER;
    } else {
        Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
        channelId = pLineObj->channelId;
    }
    
    pDevObj = pDevCtx->pDevObj;
    pInfo = &pDevObj->timerQueueInfo;
    pNodes = pDevObj->timerQueueNodes;

    VP_TIMER(VpDevCtxType, pDevCtx, ("Vp886AddTimerHalfMs id %d, dur %lu, ovr %lu, ch %d, hndl %lu", timerId, duration, overrun, channelId, handle));

    /* Subtract the previous timer's overrun to avoid long-term cadence drift */
    if (overrun > duration) {
        duration = 0;
    } else {
        duration = duration - overrun;
    }

    success = VpInsertTimer(pDevCtx, pInfo, pNodes, timerId, duration, channelId, handle);
    if (!success) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886AddTimerHalfMs - Failed to insert timer id %u, ch %d, hndl %lu", timerId, channelId, handle));
    }

    return success;
}


/** Vp886AddTimerMs()
  Wrapper for Vp886AddTimerHalfMs(), with duration specified in 1ms instead of
  0.5ms units.  The overrun parameter is still in 0.5ms units.
*/
bool
Vp886AddTimerMs(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint16 timerId,
    uint32 duration,
    uint32 overrun,
    uint32 handle)
{
    uint32 durationHalfMs;
    if (duration > 0x7FFFFFFF) {
        durationHalfMs = 0xFFFFFFFF;
    } else {
        durationHalfMs = duration * 2;
    }
    return Vp886AddTimerHalfMs(pDevCtx, pLineCtx, timerId, durationHalfMs, overrun, handle);
}


/** Vp886RestartTimerHalfMs()
  Restarts a specified timer, with duration specified in 0.5ms units.
  To restart an existing timer, the timerId and handle must both match.
  If no existing timer matches, a new timer will be added.
*/
bool
Vp886RestartTimerHalfMs(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint16 timerId,
    uint32 duration,
    uint32 handle)
{
    Vp886DeviceObjectType *pDevObj;
    VpTimerQueueInfoType *pInfo;
    VpTimerQueueNodeType *pNodes;
    uint8 channelId;
    bool success;

    if (pDevCtx == VP_NULL) {
        pDevCtx = pLineCtx->pDevCtx;
    }
    
    if (pLineCtx == VP_NULL) {
        channelId = VP886_DEV_TIMER;
    } else {
        Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
        channelId = pLineObj->channelId;
    }
    
    pDevObj = pDevCtx->pDevObj;
    pInfo = &pDevObj->timerQueueInfo;
    pNodes = pDevObj->timerQueueNodes;

    VP_TIMER(VpDevCtxType, pDevCtx, ("Vp886RestartTimerHalfMs id %d, dur %lu, ch %d, hndl %lu", timerId, duration, channelId, handle));

    success = VpRestartTimer(pDevCtx, pInfo, pNodes, timerId, duration, channelId, handle);
    if (!success) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886RestartTimerHalfMs - Failed to restart timer id %u, ch %d, hdnl %lu", timerId, channelId, handle));
    }

    return success;
}


/** Vp886AddTimerMs()
  Wrapper for Vp886RestartTimerHalfMs(), with duration specified in 1ms instead
  of 0.5ms units.
*/
bool
Vp886RestartTimerMs(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint16 timerId,
    uint32 duration,
    uint32 handle)
{
    uint32 durationHalfMs;

    if (duration > 0x7FFFFFFF) {
        durationHalfMs = 0xFFFFFFFF;
    } else {
        durationHalfMs = duration * 2;
    }
    return Vp886RestartTimerHalfMs(pDevCtx, pLineCtx, timerId, durationHalfMs, handle);
}


/** Vp886ExtendTimerMs()
  Extends a specified timer to a new, longer duration specified in 1ms units.
  To extend an existing timer, the timerId and handle must both match.
  If the remaining duration on a matching timer is longer than the requested
  new duration, no change is made.
  If no existing timer matches, a new timer will be added.
*/
bool
Vp886ExtendTimerMs(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint16 timerId,
    uint32 duration,
    uint32 handle)
{
    Vp886DeviceObjectType *pDevObj;
    VpTimerQueueInfoType *pInfo;
    VpTimerQueueNodeType *pNodes;
    uint8 channelId;
    bool success;
    uint32 durationHalfMs;

    if (duration > 0x7FFFFFFF) {
        durationHalfMs = 0xFFFFFFFF;
    } else {
        durationHalfMs = duration * 2;
    }

    if (pDevCtx == VP_NULL) {
        pDevCtx = pLineCtx->pDevCtx;
    }
    
    if (pLineCtx == VP_NULL) {
        channelId = VP886_DEV_TIMER;
    } else {
        Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
        channelId = pLineObj->channelId;
    }
    
    pDevObj = pDevCtx->pDevObj;
    pInfo = &pDevObj->timerQueueInfo;
    pNodes = pDevObj->timerQueueNodes;

    VP_TIMER(VpDevCtxType, pDevCtx, ("Vp886ExtendTimerMs id %d, ms %lu, ch %d, hndl %lu", timerId, duration, channelId, handle));

    success = VpExtendTimer(pDevCtx, pInfo, pNodes, timerId, durationHalfMs, channelId, handle);
    if (!success) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886ExtendTimerMs - Failed to extend timer id %u, ch %d, hdnl %lu", timerId, channelId, handle));
    }

    return success;
}


/** Vp886CancelTimer()
  Cancels an in-progress timer that matches in both timerId and handle.
  If matchHandle is FALSE, the handle is ignored, and all timers matching
  timerId are cancelled.
*/
bool
Vp886CancelTimer(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint16 timerId,
    uint32 handle,
    bool matchHandle)
{
    Vp886DeviceObjectType *pDevObj;
    VpTimerQueueInfoType *pInfo;
    VpTimerQueueNodeType *pNodes;
    uint8 channelId;
    bool success;
    
    if (pDevCtx == VP_NULL) {
        pDevCtx = pLineCtx->pDevCtx;
    }
    
    if (pLineCtx == VP_NULL) {
        channelId = VP886_DEV_TIMER;
    } else {
        Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
        channelId = pLineObj->channelId;
    }
    
    pDevObj = pDevCtx->pDevObj;
    pInfo = &pDevObj->timerQueueInfo;
    pNodes = pDevObj->timerQueueNodes;

    success = VpRemoveTimer(pDevCtx, pInfo, pNodes, timerId, channelId, handle, matchHandle);
    if (!success) {
        VP_TIMER(VpDevCtxType, pDevCtx, ("Vp886CancelTimer - Timer id %u, ch %d, hndl %lu did not exist to be removed", timerId, channelId, handle));
    }
    return success;
}


/** Vp886GetTimestamp()
  This function returns the current device timestamp.  If the timestamp in
  the device object is flagged as being currently valid, this function simply
  returns the cached value to avoid unnecessary register access.
  If the cached timestamp is not flagged as valid, this function will read
  from the device timestamp register.

  The timestamp cache is flagged as valid the first time it is read within a
  critical section, and invalidated when the critical section ends.  This aims
  to reduce the number of timestamp register reads without restricting how often
  this function can be used.  A fresh register read can be forced by setting the
  timestampValid flag to false before calling this function.
*/
uint16
Vp886GetTimestamp(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 timestamp;
    uint8 timestampReg[VP886_R_TIMESTAMP_LEN];

    if (pDevObj->timestampValid) {
        return pDevObj->timestamp;
    }

    /* Read the time stamp from the device */
    pDevObj->dontFlushSlacBufOnRead = TRUE;
    VpSlacRegRead(pDevCtx, NULL, VP886_R_TIMESTAMP_RD, VP886_R_TIMESTAMP_LEN, timestampReg);
    timestamp = (timestampReg[0] << 8) + timestampReg[1];
    
    /* Update the device object.
       rolloverCount and rolloverBuffer are used together to keep rollover
       and timestamp data in sync.  This is needed because it is possible to
       read a rolled-over timestamp before processing a rollover interrupt, and
       it is possible to process a rollover interrupt before reading the
       rolled-over timestamp.
       rolloverBuffer is incremented when a rollover interrupt is processed.
       rolloverCount is updated only when the timestamp is read.  If the new
       timestamp is lower than the old, we know that rolloverCount needs to be
       at least 1 higher than before.  The buffer value is added to the count
       if it has tracked a rollover that is not detected by timestamp
       comparison.
       rolloverBuffer is an int32.  It needs to be signed so that it can go
       negative to pre-emptively count a rollover by timestamp comparison.
       It needs to be larger than int16 so that it doesn't overflow to negative
       when the device is idle for 12.4 days. */
    if (timestamp < pDevObj->timestamp) {
        pDevObj->rolloverCount++;
        pDevObj->rolloverBuffer--;
    }
    if (pDevObj->rolloverBuffer > 0) {
        pDevObj->rolloverCount += pDevObj->rolloverBuffer;
        pDevObj->rolloverBuffer = 0;
    }

    /* There is a corner case where the new timestamp could be greater than the
       previous one, a rollover has occurred, and we have not processed the
       rollover yet.  This should only be possible when the timestamps are both
       small (less than 40, assuming a maximum 20ms interrupt latency, 50 for
       safety).  It is also not an issue during GetEvent, because the timestamp
       rollover should be the first thing processed.  To detect this case, we
       need to do a no-unlock read of the signaling register to see if a
       rollover occurred. */
    if (timestamp >= pDevObj->timestamp &&
        timestamp <= 50 &&
        pDevObj->inGetEvent == FALSE)
    {
        uint8 sigreg[VP886_R_SIGREG_LEN];
        pDevObj->dontFlushSlacBufOnRead = TRUE;
        VpSlacRegRead(pDevCtx, NULL, VP886_R_SIGREG_NO_UL_RD, VP886_R_SIGREG_LEN, sigreg);
       
        if (sigreg[3] & VP886_R_SIGREG_TIMESTAMP) {
            pDevObj->rolloverCount++;
            pDevObj->rolloverBuffer--;
            VP_INFO(VpDevCtxType, pDevCtx, ("Vp886GetTimestamp - Rollover corner case handled"));
        }
    }
    
    pDevObj->timestamp = timestamp;
    
    /* If inside a critical section via Vp886EnterCritical, validate the cached
       timestamp for the duration of the critical section */
    if (pDevObj->criticalDepth > 0) {
        pDevObj->timestampValid = TRUE;
    }
    
    /* Use this function as a way to detect SPI errors.  If the timestamp reads
       back 0x0000 or 0xFFFF, read the RCN/PCN register to double-check. */
    if (timestamp == 0x0000 || timestamp == 0xFFFF) {
        uint8 rcnPcn[VP886_R_RCNPCN_LEN];
        VpSlacRegRead(pDevCtx, NULL, VP886_R_RCNPCN_RD, VP886_R_RCNPCN_LEN, rcnPcn);
        if (((rcnPcn[0] == 0xFF) && (rcnPcn[1] == 0xFF)) ||
            ((rcnPcn[0] == 0x00) && (rcnPcn[1] == 0x00)))
        {
            pDevObj->spiError = TRUE;
            VP_ERROR(VpDevCtxType, pDevCtx, ("SPI error detected.  Timestamp 0x%04X, rcnPcn %02X%02X",
                timestamp, rcnPcn[0], rcnPcn[1]));
        }
    }
    
    VP_INFO(VpDevCtxType, pDevCtx, ("Vp886GetTimestamp - read %u", timestamp));
    return timestamp;
}


/** Vp886GetTimestamp32()
  Same as Vp886GetTimestamp, but uses the rollover count to create a 32-bit
  timestamp.
*/
uint32
Vp886GetTimestamp32(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 rollovers;
    uint16 timestamp16;
    uint32 timestamp32;
    
    timestamp16 = Vp886GetTimestamp(pDevCtx);
    rollovers = pDevObj->rolloverCount;
    timestamp32 = ((uint32)rollovers << 16) + timestamp16;
    
    return timestamp32;
}


/** Vp886SetDeviceTimer()
  This function sets the device timer to run in one-shot mode for the given
  duration.  A duration of 0 will stop the timer.  This function should be
  primarily used to drive the timer queue.
 
  Duration is in units of 0.5ms, the same resolution as is programmed to the
  register, and the same resolution as the device and API timestamp.
*/
void
Vp886SetDeviceTimer(
    VpDevCtxType *pDevCtx,
    bool enable,
    uint32 duration)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 gtimer[VP886_R_GTIMER_LEN];
    
    VP_TIMER(VpDevCtxType, pDevCtx, ("Vp886SetDeviceTimer %lu, %s", duration, enable ? "ON" : "OFF"));
    
    /* Avoid programming a duration too large for the device.  It's OK to
       program a shorter value than requested.  When the device timer expires,
       the software timer processing will just update the remaining duration
       based on the timestamp change and will call this function again. */
    if (duration > VP886_R_GTIMER_TIME_MAXIMUM) {
        duration = VP886_R_GTIMER_TIME_MAXIMUM;
        VP_TIMER(VpDevCtxType, pDevCtx, ("Vp886SetDeviceTimer setting maximum, %lu", duration));
    }
    
    gtimer[0] = 0x00;
    gtimer[1] = 0x00;
    if (enable) {
        /* Programming 0 duration will not generate an interrupt on non-SF
           devices, so program 1 instead (0.5ms) */
        if (duration == 0 && !VP886_IS_SF(pDevObj)) {
            duration = 1;
        }
        gtimer[0] |= VP886_R_GTIMER_ENABLE;
        gtimer[0] |= VP886_R_GTIMER_MODE_1SHOT;
        gtimer[0] |= (duration >> 8);
        gtimer[1] = (duration & 0x00FF);
    }

    VpSlacRegWrite(pDevCtx, NULL, VP886_R_GTIMER_WRT, VP886_R_GTIMER_LEN, gtimer);
    
    return;
}


/** Vp886ForceTimerInterrupt()
  Sets the ch0 device timer to the minimum duration to force an interrupt as
  soon as possible.  This is needed for functions such as VpGetOption() that
  can be called externally and need to generate an event.
  This uses the channel 0 timer instead of the global timer to avoid any
  interference with or from the main timer queue.
*/
void
Vp886ForceTimerInterrupt(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 chTimer[VP886_R_CHTIMER_LEN];
    
    chTimer[0] = VP886_R_CHTIMER_ENABLE | VP886_R_CHTIMER_MODE_1SHOT;
    if (VP886_IS_SF(pDevObj)) {
        chTimer[1] = VP886_R_CHTIMER_INSTANT_INTERRUPT;
    } else {
        chTimer[1] = 0x01; /* 0.5ms */
    }
    pDevObj->ecVal = VP886_EC_1;
    VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_CHTIMER_WRT, VP886_R_CHTIMER_LEN, chTimer);
    pDevObj->ecVal = VP886_EC_GLOBAL;
    
    return;
}


/** Vp886ManageSamplingTimer()
  Sets the ch2 device timer based on the sampling tick requirements of both
  channels as stored in pDevObj->samplingTimer[chId].
*/
void
Vp886ManageSamplingTimer(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 chTimer[VP886_R_CHTIMER_LEN];
    uint8 timerVal;
    
    /* Set timerVal to the lowest non-zero value between the two channels. */
    timerVal = pDevObj->samplingTimerReq[0];
    if (timerVal == 0 ||
        (pDevObj->samplingTimerReq[1] != 0 && pDevObj->samplingTimerReq[1] < timerVal))
    {
        timerVal = pDevObj->samplingTimerReq[1];
    }
    
    /* Don't disrupt the timer if we don't need to. */
    if (timerVal >= pDevObj->samplingTimerCurrent && pDevObj->samplingTimerCurrent != 0) {
        return;
    }
    
    if (timerVal == 0) {
        /* Shut off the sampling tick timer */
        chTimer[0] = 0x00;
        chTimer[1] = 0x00;
        pDevObj->ecVal = VP886_EC_2;
        VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_CHTIMER_WRT, VP886_R_CHTIMER_LEN, chTimer);
        pDevObj->ecVal = VP886_EC_GLOBAL;
    } else {
        /* Start the sampling tick timer. */
        chTimer[0] = VP886_R_CHTIMER_ENABLE | VP886_R_CHTIMER_MODE_CONT;
        chTimer[1] = timerVal;
        pDevObj->ecVal = VP886_EC_2;
        VpSlacRegWrite(pDevCtx, VP_NULL, VP886_R_CHTIMER_WRT, VP886_R_CHTIMER_LEN, chTimer);
        pDevObj->ecVal = VP886_EC_GLOBAL;
    }
        
    pDevObj->samplingTimerCurrent = timerVal;
    
    return;
}


/** Vp886TimerHandler()
  This function is called by the vp_timer_queue.c module when a timer expires.
  It uses the information about the timer to determine the appropriate actions
  to take.
  - timerId, channelId, and handle are the same as the parameters given to
  VpInsertTimer() when the timer was created.
  - overrun is a measure in timestamp units of how much time elapsed beyond
  the timer's original duration.  Passing this value in to the Vp886AddTimer__()
  functions will compensate for the overrun by subtracting it from the next
  timer duration.  This is useful for avoiding drift in cadence timing, but 
  should be ignored for operations such as initialization where minimum delays
  are required between actions.
*/
void
Vp886TimerHandler(
    VpDevCtxType *pDevCtx,
    uint16 timerId,
    uint32 overrun,
    uint8 channelId,
    uint32 handle)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpLineCtxType *pLineCtx = VP_NULL;
    Vp886LineObjectType *pLineObj = VP_NULL;
    uint16 timestamp;

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886TimerHandler+"));

    timestamp = Vp886GetTimestamp(pDevCtx);

    if (channelId < VP886_MAX_NUM_CHANNELS) {
        pLineCtx = pDevCtx->pLineCtx[channelId];
    }
    if (pLineCtx != VP_NULL) {
        pLineObj = pLineCtx->pLineObj;
    }

    VP_TIMER(VpDevCtxType, pDevCtx, ("Vp886TimerHandler id %d, or %lu, ch %d, arg %lu, ts %u", timerId, overrun, channelId, handle, timestamp));

    switch (timerId) {
        case VP886_TIMERID_INIT_DEVICE: {
            /* Drives the InitDevice state machine. */
            Vp886InitDeviceSM(pDevCtx);
            break;
        }
        case VP886_TIMERID_CAL_CODEC: {
            /* Drives the device calibration state machine. */
            if (pDevObj->busyFlags & VP_DEV_INIT_IN_PROGRESS) {
                pDevObj->channelCalBack[(uint8)handle] = TRUE;
                Vp886InitDeviceSM(pDevCtx);
            } else {
                pDevObj->channelCalBack[(uint8)handle] = TRUE;
                Vp886CalCodecHandler(pDevCtx);
            }
            break;
        }
        case VP886_TIMERID_CAL_LINE: {
            /* Drives the line calibration state machine. */
            Vp886CalLineSM(pLineCtx);
            break;
        }
        case VP886_TIMERID_QUICKCAL: {
            /* Drives the VpCal():VP_CAL_QUICKCAL state machine. */
            Vp886QuickCalHandler(pLineCtx);
            break;
        }
        case VP886_TIMERID_HOOK_FREEZE: {
            /* Signals the end of a hook freeze. */
            uint8 hookBuf[VP886_R_HOOKBUF_LEN];
            if (pLineObj == VP_NULL) {
                break;
            }
            VP_HOOK(VpLineCtxType, pLineCtx, ("Ending hook freeze (ts %u)", timestamp));
            
            /* Don't do anything if the line state is disconnect */
            if (pLineObj->lineState.currentState == VP_LINE_DISCONNECT) {
                break;
            }

            /* Read the hook buffer to clear out any hooks that occurred during
               the hook freeze */
            VpSlacRegRead(NULL, pLineCtx, VP886_R_HOOKBUF_RD, VP886_R_HOOKBUF_LEN, hookBuf);

            Vp886ClearDetectMask(pLineCtx, VP_CSLAC_HOOK);

            /* If the user linestate is low-power standby, rewrite the system
               state to standby if the line is onhook.  This is because any
               offhook detection that we ignored during the hook freeze would
               trigger the device's automatic state transition to active. */
            if ((hookBuf[0] & VP886_R_HOOKBUF_HK0) == 0 &&
                pLineObj->termType == VP_TERM_FXS_LOW_PWR &&
                pLineObj->lineState.usrCurrent == VP_LINE_STANDBY &&
                !pLineObj->inLineTest &&
                !(pDevObj->busyFlags & VP_DEV_IN_CAL) &&
                !(pLineObj->busyFlags & VP886_LINE_IN_CAL))
            {
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_STATE_WRT, VP886_R_STATE_LEN, pLineObj->registers.sysState);
            }

            break;
        }
        case VP886_TIMERID_GKEY_FREEZE: {
            /* Signals the end of a GKEY freeze. */
            if (pLineObj == VP_NULL) {
                break;
            }
            /* Don't do anything if the line state is disconnect */
            if (pLineObj->lineState.currentState == VP_LINE_DISCONNECT) {
                break;
            }
            Vp886ClearDetectMask(pLineCtx, VP_CSLAC_GKEY);
            break;
        }
        case VP886_TIMERID_PULSE_DECODE: {
            /* Signals a timeout in pulse decoding, such as interdigit or
               onhook min time */
            uint16 timer;
            if (pLineObj == VP_NULL) {
                break;
            }
            timer = VpPulseDecodeRun(pLineCtx, &pLineObj->pulseDecodeData,
                VP_PULSE_DECODE_INPUT_TIMER, timestamp);
            if (timer != 0) {
                Vp886AddTimerHalfMs(NULL, pLineCtx, VP886_TIMERID_PULSE_DECODE, timer, 0, 0);
            }
            break;
        }
        case VP886_TIMERID_INT_TEST_TERM: {
            /* Signals the delayed second step of enabling the internal test
               termination (VP_RELAY_BRIDGED_TEST). */
            if (pLineObj == VP_NULL) {
                break;
            }
            if (!pLineObj->intTestTermApplied) {
                break;
            }
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Adjusting internal test termination (ts %u)", timestamp));
            /* Adjust tip and ring bias to make tip and ring outputs high
               impedance, and tend to pull to battery */
            pLineObj->registers.icr1[0] = 0xFF;
            pLineObj->registers.icr1[1] = 0x18;
            pLineObj->registers.icr1[2] = 0x3F;
            pLineObj->registers.icr1[3] = 0x04;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR1_WRT, VP886_R_ICR1_LEN, pLineObj->registers.icr1);
            break;
        }
        case VP886_TIMERID_POLREV_FIX: {
            /* Signals the delayed restoration of registers that were changed
               while performing a polrev. */
            if (pLineObj == VP_NULL) {
                break;
            }
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Polrev fix, restoring ICRs (ts %u)", timestamp));
            /* Release control of the speedup bits */
            pLineObj->registers.icr2[2] &= ~VP886_R_ICR2_SPEEDUP_BAT;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                pLineObj->registers.icr2);
            pLineObj->registers.icr3[0] &= ~VP886_R_ICR3_SPEEDUP_LONG;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN,
                pLineObj->registers.icr3);
            break;
        }
        case VP886_TIMERID_RING_EXIT: {
            /* Signals the delayed restoration of registers that were changed
               while performing a ring exit. */
            if (pLineObj == VP_NULL) {
                break;
            }
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Ring exit timer handler (ts %u)", timestamp));
            if (VP886_IS_TRACKER(pDevObj) && VP886_IS_HV(pDevObj)) {
                /* Re-apply the 103V switcher clamp */
                pLineObj->registers.icr2[2] |= VP886_R_ICR2_SW_LIM;
                pLineObj->registers.icr2[3] &= ~VP886_R_ICR2_SW_LIM;
                pLineObj->registers.icr2[3] |= VP886_R_ICR2_SW_LIM_100;
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN,
                    pLineObj->registers.icr2);
            }
            if (VP886_IS_TRACKER(pDevObj) && 
                (pLineObj->registers.swParam[0] & VP886_R_SWPARAM_RING_TRACKING)
                    == VP886_R_SWPARAM_RING_TRACKING_EN)
            {
                /* Restore normal switcher floor voltage */
                pLineObj->registers.swParam[0] &= ~VP886_R_SWPARAM_FLOOR_V;
                pLineObj->registers.swParam[0] |= VpRoundedDivide(pLineObj->floorVoltage - 5, 5);
                VpSlacRegWrite(NULL, pLineCtx, VP886_R_SWPARAM_WRT, VP886_R_SWPARAM_LEN,
                    pLineObj->registers.swParam);
            }
            if (pLineObj->unbalancedRinging) {
                Vp886ClearDetectMask(pLineCtx, VP_CSLAC_GKEY);
            }
            if (VP886_IS_ABS(pDevObj)) {
                Vp886SetABSRingingBattFlag(pLineCtx, TRUE);
                Vp886ManageABSRingingBatt(pDevCtx, TRUE, TRUE);
            }
            pLineObj->ringExitInProgress = FALSE;
            break;
        }
        case VP886_TIMERID_RING_EXIT_CLEANUP: {
            /* Wait for the ring exit state to clear.  Then wait an extra delay
               before setting the final line state to allow the battery and
               internal states to settle. The state is passed through the timer
               handle argument. */
            if (pLineObj == VP_NULL) {
                break;
            }
            switch (handle) {
                case VP886_RING_EXIT_CLEANUP_CHECK: {
                    uint8 stateReg;
                    VpSlacRegRead(NULL, pLineCtx, VP886_R_STATE_RD, VP886_R_STATE_LEN, &stateReg);
                    if (!(stateReg & VP886_R_STATE_REX)) {
                        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Ring exit state cleanup, detected exit (ts %u)", timestamp));
                        Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_RING_EXIT_CLEANUP,
                            VP886_RING_EXIT_CLEANUP_FINISH_DELAY, 0, VP886_RING_EXIT_CLEANUP_FINISH);
                    } else {
                        /* Try again later */
                        Vp886AddTimerMs(NULL, pLineCtx, VP886_TIMERID_RING_EXIT_CLEANUP,
                            VP886_RING_EXIT_CLEANUP_CHECK_DELAY, 0, VP886_RING_EXIT_CLEANUP_CHECK);
                    }
                } break;
                case VP886_RING_EXIT_CLEANUP_FINISH: {
                    if (pLineObj->ringSyncState == VP886_RINGSYNC_STATE_PAUSING) {
                        Vp886RingSyncFinish(pLineCtx);
                    } else {
                        VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Ring exit state cleanup, changing to %d (ts %u)",
                            pLineObj->ringExitCleanupState, timestamp));
                        Vp886SetLineStateFxsInt(pLineCtx, pLineObj->ringExitCleanupState);
                    }
                } break;
                default:
                    VP_WARNING(VpLineCtxType, pLineCtx, ("Ring exit state cleanup, invalid state %lu", handle));
            }
            break;
        }
        case VP886_TIMERID_ABS_POWER: {
            /* Signals a delayed change to ABS power mode.
               Register data is passed through the timer handle */
            pDevObj->registers.swCtrl[0] = (uint8)handle;
            VpSlacRegWrite(pDevCtx, NULL, VP886_R_SWCTRL_WRT, VP886_R_SWCTRL_LEN, pDevObj->registers.swCtrl);
            VP_LINE_STATE(VpDevCtxType, pDevCtx, ("Setting switcher ctrl to 0x%02X (ts %u)",
                pDevObj->registers.swCtrl[0], timestamp));
            break;
        }
        case VP886_TIMERID_BATFLT_POLL: {
            /* Repeat battery fault poll timer until it is cancelled to
               generate interrupts that allow detection of battery fault bits
               clearing. */
            if (pDevObj->battFltPollTime > 0) {
                Vp886AddTimerMs(pDevCtx, VP_NULL, VP886_TIMERID_BATFLT_POLL, pDevObj->battFltPollTime, overrun, 0);
            }
            break;
        }
        case VP886_TIMERID_USER: {
            /* Signals the end of a user timer started by VpGenTimerCtrl() */
            pDevObj->userTimers--;
            pLineObj->userTimers--;
            VP_TIMER(VpLineCtxType, pLineCtx, ("User timer counts: D:%d L:%d",
                pDevObj->userTimers, pLineObj->userTimers));
            Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_PROCESS, VP_LINE_EVID_GEN_TIMER, VP_GEN_TIMER_STATUS_CMP, handle, FALSE);
            break;
        }
        case VP886_TIMERID_GND_FLT_PROT: {
            /* Drives the ground fault protection state machine. */
            if (pLineCtx == VP_NULL) {
                break;
            }
            Vp886GndFltProtHandler(pLineCtx, VP886_GNDFLTPROT_INP_TIMER);
            break;
        }
        case VP886_TIMERID_GROUNDSTART: {
            /* Finishes the ground start workaround by releasing c_long_on. */
            if (pLineCtx == VP_NULL) {
                break;
            }
            if (pLineObj->lineState.currentState == VP_LINE_TIP_OPEN ||
                pLineObj->lineState.currentState == VP_LINE_RING_OPEN)
            {
                /* Don't do anything if we've ended up back in a lead-open
                   state. */
                break;
            }
            /* Release c_long_on to resume normal operation. */
            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Finishing ground start workaround, releasing c_long_on control"));
            pLineObj->registers.icr3[2] &= ~VP886_R_ICR3_LONG_ON;
            VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN,
                pLineObj->registers.icr3);
            break;
        }
        case VP886_TIMERID_RINGTRIP_CONFIRM: {
            if (pLineCtx == VP_NULL) {
                break;
            }
            if (pLineObj->lineState.condition & VP_CSLAC_HOOK) {
                /* Offhook */
                VP_HOOK(VpLineCtxType, pLineCtx, ("Ring trip confirmed"));
                pLineObj->ringTripConfirm.confirming = FALSE;
                /* Pretend that we just now received the offhook indication,
                   generate the event, and do anything else that needs to be
                   done in response to the offhook. */
                pLineObj->lineState.condition &= ~VP_CSLAC_HOOK;
                Vp886ProcessHookEvent(pLineCtx, VP_CSLAC_HOOK, timestamp);
            } else {
                /* Onhook */
                if (pLineObj->ringTripConfirm.glitchCheck) {
                    /* We didn't see the ringtrip hook glitch yet, so we need to
                       set another short timer to see if the status goes back to
                       offhook. */
                    uint16 elapsed_ms;
                    uint16 hookDebounce_ms;
                    uint16 checkTimer_ms;
                    elapsed_ms = (Vp886GetTimestamp(pDevCtx) - pLineObj->ringTripConfirm.lastHookTimestamp) / 2;
                    hookDebounce_ms = 2 * (pLineObj->registers.loopSup[1] & VP886_R_LOOPSUP_HOOK_DBNC);
                    if (elapsed_ms < hookDebounce_ms) {
                        checkTimer_ms = hookDebounce_ms - elapsed_ms + 2;
                    } else {
                        checkTimer_ms = 2;
                    }
                    VP_HOOK(VpLineCtxType, pLineCtx, ("Ringtrip confirm checking hook glitch %dms", checkTimer_ms));
                    Vp886ExtendTimerMs(NULL, pLineCtx, VP886_TIMERID_RINGTRIP_CONFIRM, checkTimer_ms, 0);
                    pLineObj->ringTripConfirm.glitchCheck = FALSE;
                } else {
                    VP_HOOK(VpLineCtxType, pLineCtx, ("Ring trip not confirmed"));
                    pLineObj->ringTripConfirm.confirming = FALSE;
                    /* Don't generate any event, and resume ringing if needed */
                    if (Vp886LineStateInfo(pLineObj->lineState.usrCurrent).normalEquiv == VP_LINE_RINGING) {
                        #ifdef VP_CSLAC_SEQ_EN
                            /* Resume ringing if we are still in a ring-on period, or if not cadencing */
                            if (pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) {
                                if (Vp886LineStateInfo(pLineObj->cadence.lineState).normalEquiv == VP_LINE_RINGING) {
                                    VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Resuming ringing"));
                                    Vp886SetLineStateFxsInt(pLineCtx, pLineObj->cadence.lineState);
                                }
                            } else {
                                VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Resuming ringing"));
                                Vp886SetLineStateFxsInt(pLineCtx, pLineObj->lineState.usrCurrent);
                            }
                        #else
                            /* Resume ringing */
                            VP_LINE_STATE(VpLineCtxType, pLineCtx, ("Resuming ringing"));
                            Vp886SetLineStateFxsInt(pLineCtx, pLineObj->lineState.usrCurrent);
                        #endif /* VP_CSLAC_SEQ_EN */
                    }
                }
            }
            break;
        }
#ifdef VP886_INCLUDE_ADAPTIVE_RINGING
        case VP886_TIMERID_THERMAL_RINGING: {
            /* Drives the adaptive ringing thermal management algorithm. */
            if (pLineCtx == VP_NULL) {
                break;
            }
            switch (handle) {
                case VP886_THERMAL_RINGING_DEBOUNCE: {
                    pLineObj->ringPowerAdapt.debouncing = FALSE;
                } break;
                case VP886_THERMAL_RINGING_CADENCE: {
                    Vp886AdaptiveRingingHandler(pLineCtx);
                } break;
                default:
                    VP_WARNING(VpLineCtxType, pLineCtx, ("Adaptive ringing, invalid state %lu", handle));
            }
            break;
        }
#endif        
#ifdef VP_CSLAC_SEQ_EN
        case VP886_TIMERID_CADENCE: {
            /* Drives the tone/ringing cadence sequencer. */
            if (pLineCtx == VP_NULL) {
                break;
            }
            Vp886CadenceHandler(pLineCtx, overrun);
            break;
        }
        case VP886_TIMERID_CID: {
            /* Drives the Caller ID sequencer. */
            if (pLineCtx == VP_NULL) {
                break;
            }
            VP_CID(VpLineCtxType, pLineCtx, ("CID timer (ts %u)", Vp886GetTimestamp(pDevCtx)));
            Vp886CidHandler(pLineCtx, overrun);
            break;
        }
        case VP886_TIMERID_SENDSIGNAL: {
            /* Drives VpSendSignal() state machines. */
            if (pLineCtx == VP_NULL) {
                break;
            }
            Vp886SendSignalHandler(pLineCtx, overrun);
            break;
        }
        case VP886_TIMERID_METERING: {
            /* Controls metering on/off times. */
            if (pLineCtx == VP_NULL) {
                break;
            }
            Vp886MeterHandler(pLineCtx, overrun);
            break;
        }
        case VP886_TIMERID_HOWLER: {
            /* Controls Howler Tone amplitude stepping. */
            if (pLineCtx == VP_NULL) {
                break;
            }
            Vp886HowlerToneHandler(pLineCtx, overrun);
            break;
        }
#endif /* VP_CSLAC_SEQ_EN */
#ifdef VP886_INCLUDE_TESTLINE_CODE
        case VP886_TIMERID_LINE_TEST: {
            /* Implements delays for line test primitives. */
            const void *pArgsUntyped;
            if (pLineCtx == VP_NULL) {
                break;
            }
            pArgsUntyped = (const void*)&pLineObj->testInfo.pTestHeap->testArgs;
            Vp886TestLineInt(pLineCtx, pArgsUntyped, TRUE);
            break;
        }
#endif /* VP886_INCLUDE_TESTLINE_CODE */
        default:
            VP_ERROR(VpDevCtxType, pDevCtx, ("Unhandled timer id %u, ch %d", timerId, channelId));
            break;
    }

    VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("Vp886TimerHandler-"));
    return;
}


#endif /* VP_CC_886_SERIES */


