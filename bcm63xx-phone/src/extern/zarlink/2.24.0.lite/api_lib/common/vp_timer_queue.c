/** \file vp_timer_queue.c
 * vp_timer_queue.c
 *
 * This file contains the VP Timer Queue Module C code.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11228 $
 * $LastChangedDate: 2013-11-25 10:45:32 -0600 (Mon, 25 Nov 2013) $
 */
#include "vp_timer_queue.h"
#include "vp_debug.h"

#if defined (VP_CC_886_SERIES)
#include "vp886_api_int.h"

/* Prototypes for static internal functions */
static int16
VpFindTimerInt(
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint8 channelId,
    uint32 handle);

static void
VpUpdateTimersInt(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes);

static VpTimerQueueStatusType
VpInsertTimerInt(
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint32 timerDuration,
    uint8 channelId,
    uint32 handle);

static VpTimerQueueStatusType
VpRemoveTimerInt(
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint8 channelId,
    uint32 handle,
    bool matchHandle);

static VpTimerQueueStatusType
VpRemoveChannelTimersInt(
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint8 channelId);

static VpTimerQueueStatusType
VpRestartTimerInt(
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint32 timerDuration,
    uint8 channelId,
    uint32 handle);

static VpTimerQueueStatusType
VpProcessTimerQueueInt(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes);

static void
VpCallTimerHandlerInt(
    VpDevCtxType *pDevCtx,
    uint16 timerId,
    uint32 overrun,
    uint8 channelId,
    uint32 handle);

static uint32
VpGetTimestampInt(
    VpDevCtxType *pDevCtx);

static void
VpSetDevTimerInt(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes);


/**  VpInitTimerQueue()
  Initializes the timer queue to empty.
*/
bool
VpInitTimerQueue(
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes)
{
    int16 i;

    pInfo->index = 0;
    pInfo->timestamp = 0;

    for (i = 0 ; i < pInfo->numNodes ; i++) {
        pNodes[i].active = FALSE;
    }

    pInfo->processing = FALSE;

    return TRUE;
}

/**  VpFindTimerInt()
  Returns the index of timer specified by timerId, channelId, and handle,
  if it is active.  Returns VP_END_OF_TIMER_QUEUE if the timer is not found.
*/
int16
VpFindTimerInt(
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint8 channelId,
    uint32 handle)
{
    int16 idx = pInfo->index;

    /* First, check if the queue is empty */
    if (pNodes[idx].active == FALSE) {
        return VP_END_OF_TIMER_QUEUE;
    }

    /* Look through the queue for a timer matching the criteria */
    while (idx != VP_END_OF_TIMER_QUEUE) {
        if (pNodes[idx].id == timerId && pNodes[idx].channelId == channelId && pNodes[idx].handle == handle) {
            /* Found it */
            return idx;
        }
        idx = pNodes[idx].next;
    }

    /* Didn't find it */
    return VP_END_OF_TIMER_QUEUE;
}

/**  VpUpdateTimersInt()
  Calculates elapsed time based on timestamps, and subtracts that from timers
  in the queue.

  This function updates the queue timestamp and subtracts the elapsed time
  from the timers in the queue.  Starting at the front of the queue, if the
  elapsed time is greater than the remaining duration of the timer, the timer
  is set to 0, its overrun is noted.  The remaining elapsed time is then
  applied to the next timer, and so on until there is no more elapsed time.

  This should be called at the beginning of each external interface function
  other than Init (Insert, Remove, Restart, Extend, Process).
*/
void
VpUpdateTimersInt(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes)
{
    uint32 newTimestamp;
    uint32 elapsed;
    VpTimerQueueNodeType *pTimer;

    newTimestamp = VpGetTimestampInt(pDevCtx);
    elapsed = newTimestamp - pInfo->timestamp;
    VP_INFO(VpDevCtxType, pDevCtx, ("VpUpdateTimersInt: old %lu, new %lu, elapsed %lu", pInfo->timestamp, newTimestamp, elapsed));
    pInfo->timestamp = newTimestamp;

    pTimer = &(pNodes[pInfo->index]);

    /* Subtract out the elapsed time from the queue */
    while (pTimer->active && elapsed > 0) {
        VP_INFO(VpDevCtxType, pDevCtx, ("VpUpdateTimersInt: timer duration: %lu", pTimer->duration));
        if (elapsed >= pTimer->duration) {
            elapsed -= pTimer->duration;
            pTimer->duration = 0;
            pTimer->overrun += elapsed;
        } else {
            pTimer->duration -= elapsed;
            elapsed = 0;
        }
        VP_INFO(VpDevCtxType, pDevCtx, ("VpUpdateTimersInt: new duration %lu, overrun: %lu", pTimer->duration, pTimer->overrun));
        if (pTimer->next != VP_END_OF_TIMER_QUEUE) {
            pTimer = &(pNodes[pTimer->next]);
        } else {
            break;
        }
    }

    return;
}

/**  VpInsertTimer()
  Inserts a new timer into the queue.

  The new timer must be unique by timerId, channelId, and handle.  If the
  new timer becomes the first in the queue, the device timer is adjusted.

  Returns TRUE if successful, FALSE if the timer is already in the queue or
  if the queue is full.
*/
bool
VpInsertTimer(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint32 timerDuration,
    uint8 channelId,
    uint32 handle)
{
    VpTimerQueueStatusType status;

    if (VpFindTimerInt(pInfo, pNodes, timerId, channelId, handle) != VP_END_OF_TIMER_QUEUE) {
        VP_TIMER(VpDevCtxType, pDevCtx, ("VpInsertTimer - Timer already exists (id %d, ch %d, hndl %lu)", timerId, channelId, handle));
        return FALSE;
    }

    VpUpdateTimersInt(pDevCtx, pInfo, pNodes);

    /* Add the new timer to the queue */
    status = VpInsertTimerInt(pInfo, pNodes, timerId, timerDuration, channelId, handle);
    if (status.success == FALSE) {
        status.firstTimerChanged = FALSE;
    }

    /* Check if the newly inserted timer comes in first */
    if (status.firstTimerChanged == TRUE && !pInfo->processing) {
        VpSetDevTimerInt(pDevCtx, pInfo, pNodes);
    }

    return status.success;
}


/**  VpInsertTimerInt()
  Inserts a new timer into the queue.

  The queue is implemented as a linked list within a static array.  We begin
  by finding an unused space in the array.
  Then we go through the existing queue, subtracting the delta of each timer
  from the new duration.
  When the remaining delta of the new timer is less than the delta of the next
  timer in the queue, or we reach the end, the new timer is inserted.

  Returns TRUE if the timer is inserted, FALSE if the queue is already full.
*/
VpTimerQueueStatusType
VpInsertTimerInt(
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint32 timerDuration,
    uint8 channelId,
    uint32 handle)
{
    VpTimerQueueStatusType retStatus;
    int16 *index = &(pInfo->index);
    int16 idx;
    int16 newIdx = 0;
    int16 prevIdx = VP_END_OF_TIMER_QUEUE;

    /* Initialize the return structure (require minimal changes for each cases) */
    retStatus.success = TRUE;
    retStatus.firstTimerChanged = FALSE;

    /* Find an available index in the array */
    while (pNodes[newIdx].active == TRUE) {
        newIdx = (newIdx + 1) % pInfo->numNodes;
        if (newIdx == 0) {
            VP_TIMER(None, VP_NULL,("VpInsertTimer(): Timer queue full"));

            retStatus.success = FALSE;
            return retStatus;
        }
    }

    /* Find where the new timer belongs in the queue */
    if (pNodes[*index].active) {
        idx = *index;
        while (idx != VP_END_OF_TIMER_QUEUE && timerDuration > pNodes[idx].duration) {
            /* Subtract the delta time */
            timerDuration -= pNodes[idx].duration;

            prevIdx = idx;
            idx = pNodes[idx].next;
        }
    } else {
        idx = VP_END_OF_TIMER_QUEUE;
    }

    pNodes[newIdx].id = timerId;
    pNodes[newIdx].duration = timerDuration;
    pNodes[newIdx].overrun = 0;
    pNodes[newIdx].active = TRUE;
    pNodes[newIdx].handle  = handle;
    pNodes[newIdx].channelId  = channelId;
    pNodes[newIdx].next = idx;

    /* Update the previous timer if there is one.  Otherwise, the new timer
       becomes the first in the queue. */
    if (prevIdx != VP_END_OF_TIMER_QUEUE) {
        pNodes[prevIdx].next = newIdx;
    } else {
        *index = newIdx;
        retStatus.firstTimerChanged = TRUE;
    }

    /* Update the delta of the next timer if there is one */
    if (idx != VP_END_OF_TIMER_QUEUE) {
        pNodes[idx].duration -= timerDuration;
    }

    return retStatus;
}

/**  VpRemoveTimer()
  Removes a timer or timers from the queue.

  Removes all timers in the queue that match the provided timerId, channelId,
  and (optionally) handle.  If matchHandle is TRUE, this function will search
  for a timer that matches all three criteria, which means that timer will be
  unique.  If matchHandle is FALSE, this may remove multiple timers that match
  the timerId and channelId.
  If the first timer in the queue is removed, the device timer is adjusted.

  Returns TRUE if any timers were removed, FALSE if the specified timer was
  not found.
*/
bool
VpRemoveTimer(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint8 channelId,
    uint32 handle,
    bool matchHandle)
{
    VpTimerQueueStatusType status;

    VpUpdateTimersInt(pDevCtx, pInfo, pNodes);

    /* Remove the requested timer from the queue */
    status = VpRemoveTimerInt(pInfo, pNodes, timerId, channelId, handle, matchHandle);
    if (status.success == FALSE) {
        VP_INFO(VpDevCtxType, pDevCtx, ("VpRemoveTimer - Timer %d was not removed from the queue", timerId));
    }

    /* Check if the first timer was removed */
    if (status.firstTimerChanged == TRUE && !pInfo->processing) {
        VpSetDevTimerInt(pDevCtx, pInfo, pNodes);
    }

    return status.success;
}

/**  VpRemoveTimerInt()
  Removes a timer or timers from the queue.

  Searches through the active timers in the queue, removing each timer that
  matches the given criteria.  When a timer is removed, we adjust the link
  from the previous timer and the delta duration of the next timer.

  Returns TRUE if any timers were removed, FALSE if the specified timer was
  not found.
*/
VpTimerQueueStatusType
VpRemoveTimerInt(
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint8 channelId,
    uint32 handle,
    bool matchHandle)
{
    VpTimerQueueStatusType retStatus;
    int16 *index = &(pInfo->index);
    int16 idx = *index;
    int16 prevIdx = *index;
    int16 nextIdx;

    /* Initialize the return structure (require minimal changes for each cases) */
    retStatus.success = FALSE;
    retStatus.firstTimerChanged = FALSE;

    /* First, check if the queue is empty */
    if (pNodes[*index].active == FALSE) {
        return retStatus;
    }

    /* Look through the queue for any timers matching the criteria */
    while (idx != VP_END_OF_TIMER_QUEUE) {

        nextIdx = pNodes[idx].next;

        if (pNodes[idx].id == timerId && pNodes[idx].channelId == channelId &&
            (pNodes[idx].handle == handle || !matchHandle))
        {
            /* Found a matching timer */
            retStatus.success = TRUE;

            /* Disable the timer */
            pNodes[idx].active = FALSE;

            if (idx == *index) {
                /* This is the front timer in the queue */
                retStatus.firstTimerChanged = TRUE;
                /* Move the starting index to the next timer if one exists */
                if (nextIdx != VP_END_OF_TIMER_QUEUE) {
                    *index = nextIdx;
                }
            } else {
                /* Not the front timer.  Link the previous timer to the
                   next one */
                pNodes[prevIdx].next = nextIdx;
            }

            /* Update duration in the next node if any */
            if (nextIdx != VP_END_OF_TIMER_QUEUE) {
                pNodes[nextIdx].duration += pNodes[idx].duration;
            }
        } else {
            /* Only change prevIdx if the current timer was NOT removed.
               We don't want prevIdx to point to a removed timer. */
            prevIdx = idx;
        }

        idx = nextIdx;
    }

    return retStatus;
}

/**  VpRemoveChannelTimers()
  Removes all timers associated with the given channelId from the queue.

  Removes all timers in the queue that match the provided channelId.
  If the first timer in the queue is removed, the device timer is adjusted.

  Returns TRUE if any timers were removed, FALSE if no timer was removed.
*/
bool
VpRemoveChannelTimers(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint8 channelId)
{
    VpTimerQueueStatusType status;

    VpUpdateTimersInt(pDevCtx, pInfo, pNodes);

    /* Remove the requested timers from the queue */
    status = VpRemoveChannelTimersInt(pInfo, pNodes, channelId);

    /* Check if the first timer was removed */
    if (status.firstTimerChanged == TRUE && !pInfo->processing) {
        VpSetDevTimerInt(pDevCtx, pInfo, pNodes);
    }

    return status.success;
}

/**  VpRemoveChannelTimersInt()
  Removes all timers associated with the given channelId from the queue.

  Searches through the active timers in the queue, removing each timer that
  matches the given channelId.  When a timer is removed, we adjust the link
  from the previous timer and the delta duration of the next timer.

  Returns TRUE if any timers were removed, FALSE if no timer was removed.
*/
VpTimerQueueStatusType
VpRemoveChannelTimersInt(
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint8 channelId)
{
    VpTimerQueueStatusType retStatus;
    int16 *index = &(pInfo->index);
    int16 idx = *index;
    int16 prevIdx = *index;
    int16 nextIdx;

    /* Initialize the return structure (require minimal changes for each cases) */
    retStatus.success = FALSE;
    retStatus.firstTimerChanged = FALSE;

    /* First, check if the queue is empty */
    if (pNodes[*index].active == FALSE) {
        return retStatus;
    }

    /* Look through the queue for any timers matching the criteria */
    while (idx != VP_END_OF_TIMER_QUEUE) {

        nextIdx = pNodes[idx].next;

        if (pNodes[idx].channelId == channelId) {
            /* Found a matching timer */
            retStatus.success = TRUE;

            /* Disable the timer */
            pNodes[idx].active = FALSE;

            if (idx == *index) {
                /* This is the front timer in the queue */
                retStatus.firstTimerChanged = TRUE;
                /* Move the starting index to the next timer if one exists */
                if (nextIdx != VP_END_OF_TIMER_QUEUE) {
                    *index = nextIdx;
                }
            } else {
                /* Not the front timer.  Link the previous timer to the
                   next one */
                pNodes[prevIdx].next = nextIdx;
            }

            /* Update duration in the next node if any */
            if (nextIdx != VP_END_OF_TIMER_QUEUE) {
                pNodes[nextIdx].duration += pNodes[idx].duration;
            }
        } else {
            /* Only change prevIdx if the current timer was NOT removed.
               We don't want prevIdx to point to a removed timer. */
            prevIdx = idx;
        }

        idx = nextIdx;
    }

    return retStatus;
}

/**  VpRestartTimer()
  Restarts a timer with a new duration.

  The timer to restart is specified by timerId, channelId, and handle.  If a
  matching timer is not already in the queue, a new one will be added.
  If the existing timer is removed from the front of the queue or if the new
  duration timer becomes the front of the queue, the device timer is adjusted.

  Returns TRUE if successful, FALSE if the timer could not be (re)inserted.
*/
bool
VpRestartTimer(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint32 timerDuration,
    uint8 channelId,
    uint32 handle)
{
    VpTimerQueueStatusType status;

    VpUpdateTimersInt(pDevCtx, pInfo, pNodes);

    /* Restart the timer in the queue */
    status = VpRestartTimerInt(pInfo, pNodes, timerId, timerDuration, channelId, handle);
    if (status.success == FALSE) {
        VP_INFO(VpDevCtxType, pDevCtx, ("VpRestartTimer - Error inserting the timer %d", timerId));
        status.firstTimerChanged = FALSE;
    }

    /* Check if the newly re-inserted timer comes in first or the removed timer
       was first */
    if (status.firstTimerChanged == TRUE && !pInfo->processing) {
        VpSetDevTimerInt(pDevCtx, pInfo, pNodes);
    }

    return status.success;
}


/**  VpRestartTimerInt()
  Restarts a timer with a new duration.

  A timer is restarted by first removing it from the queue (if it exists),
  then inserting a new copy with the requested new duration.

  Returns TRUE if successful, FALSE if the timer could not be (re)inserted.
*/
VpTimerQueueStatusType
VpRestartTimerInt(
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint32 timerDuration,
    uint8 channelId,
    uint32 handle)
{
    VpTimerQueueStatusType removeStatus, retStatus;

    /* Remove the timer (works even if the timer doesn't exist) */
    removeStatus = VpRemoveTimerInt(pInfo, pNodes, timerId, channelId, handle, TRUE);

    retStatus = VpInsertTimerInt(pInfo, pNodes, timerId, timerDuration, channelId, handle);

    if (removeStatus.firstTimerChanged) {
        retStatus.firstTimerChanged = TRUE;
    }

    return retStatus;
}

/**  VpExtendTimer()
  Extends a timer's duration.

  Changes a timer's duration to the provided timerDuration if and only if the
  new duration is longer than the timer's current remaining duration.  If the
  requested new duration is shorter, this function does nothing.  If the timer,
  as specified by timerId, channelId, and handle, does not exist in the queue
  a new one is inserted.

  An example use for this function is when two different functions can begin
  debounce periods of different lengths.  Use this function instead of
  VpRestartTimer to avoid overriding the longer debounce period with the
  shorter one.

  If the existing timer is removed from the front of the queue or if the new
  duration timer becomes the front of the queue, the device timer is adjusted.

  Returns TRUE if successful or if the new duration is shorter than the
  current duration.  Returns FALSE if the timer could not be (re)inserted.
*/
bool
VpExtendTimer(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint32 timerDuration,
    uint8 channelId,
    uint32 handle)
{
    VpTimerQueueStatusType status;
    uint32 deltaSum;
    int16 findIdx;
    int16 idx;

    VpUpdateTimersInt(pDevCtx, pInfo, pNodes);

    findIdx = VpFindTimerInt(pInfo, pNodes, timerId, channelId, handle);

    if (findIdx == VP_END_OF_TIMER_QUEUE) {
        /* Timer doesn't exist yet */
        status = VpInsertTimerInt(pInfo, pNodes, timerId, timerDuration, channelId, handle);
    } else {
        /* Timer exists, calculate its total duration */
        idx = pInfo->index;
        deltaSum = 0;

        while (idx != findIdx) {
            deltaSum += pNodes[idx].duration;
            idx = pNodes[idx].next;
        }

        deltaSum += pNodes[findIdx].duration;

        /* If the requested duration is less than the timer's current remaining
           duration, do nothing. */
        if (timerDuration < deltaSum) {
            return TRUE;
        }

        /* Otherwise, replace the timer with the new duration */
        status = VpRestartTimerInt(pInfo, pNodes, timerId, timerDuration, channelId, handle);
    }

    if (status.firstTimerChanged == TRUE && !pInfo->processing) {
        VpSetDevTimerInt(pDevCtx, pInfo, pNodes);
    }

    return status.success;
}

/**  VpProcessTimers()
  Updates the queue for elapsed time and handles finished timers.

  This function should be called whenever the device timer expires.  It may
  also be called at any other time, since the elapsed time is calculated by
  timestamps, not the duration set to the device timer.

  Even if this is called only when the device timer expires, there is no
  guarantee that a timer in the queue will be completed, because timers can
  be specified with durations longer than the maximum duration of some device
  timers.  In these cases, the device timer may need to be set multiple times
  by this function to complete one timer.
*/
void
VpProcessTimers(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes)
{
    VpTimerQueueStatusType status;

    VpUpdateTimersInt(pDevCtx, pInfo, pNodes);

    /* Process any timers that have expired */
    status = VpProcessTimerQueueInt(pDevCtx, pInfo, pNodes);
    if (status.success == FALSE) {
        VP_INFO(VpDevCtxType, pDevCtx, ("Error processing the timer queue"));
        return;
    }

    /* Set the device timer here if there are any active timers, to support
       device timer limits shorter than the software timer limit by making the
       device timer run multiple times */
    if (pNodes[pInfo->index].active == TRUE) {
        VpSetDevTimerInt(pDevCtx, pInfo, pNodes);
    }

    return;
}

/**  VpProcessTimerQueueInt()
  Removes completed timers and runs their handler functions.

  A timer is deemed completed if its duration is 0.  This could mean it was
  initially added with 0 duration, or VpUpdateTimersInt() subtracted off
  enough elapsed time.

  The queue is always ordered from least to most remaining duration, so any
  completed timers will always be found consecutively at the front.  A
  completed timer can be removed simply by clearing the active flag and
  advancing the queue's starting index.

  After removing each timer, its information is passed to the appropriate
  device-specific handler function.  This is done after removing it from the
  queue so that the handler function may re-insert the same timer and to allow
  efficient use of the allocated timer queue memory.
*/
VpTimerQueueStatusType
VpProcessTimerQueueInt(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes)
{
    int16 *index = &(pInfo->index);
    VpTimerQueueStatusType retStatus;
    VpTimerQueueNodeType *pTimer;

    /* Initialize the return structure */
    retStatus.success = TRUE;
    retStatus.firstTimerChanged = FALSE;

    pInfo->processing = TRUE;

    /* Process timers that have been reduced to 0 time remaining */
    pTimer = &(pNodes[*index]);
    while (pTimer->active && pTimer->duration == 0) {
        /* Deactivate the timer */
        pTimer->active = FALSE;

        /* Update the first timer of the queue to the next one if any */
        if (pTimer->next != VP_END_OF_TIMER_QUEUE) {
            *index = pTimer->next;
        }

        /* Queue processed */
        retStatus.firstTimerChanged = TRUE;

        /* Call the timer handler function.
           Note: It is important that this occurs AFTER removing the timer from
           the queue to make the best use of the timer queue size and to better
           allow detection of unwanted multiple timers. */
        VpCallTimerHandlerInt(pDevCtx, pTimer->id, pTimer->overrun, pTimer->channelId, pTimer->handle);

        pTimer = &(pNodes[*index]);
    }

    pInfo->processing = FALSE;

    return retStatus;
}

/**  VpCallTimerHandlerInt()
  Passes the information of a completed timer into the appropriate handler
  function, based on device type.
*/
void
VpCallTimerHandlerInt(
    VpDevCtxType *pDevCtx,
    uint16 timerId,
    uint32 overrun,
    uint8 channelId,
    uint32 handle)
{
    switch (pDevCtx->deviceType) {
#ifdef VP_CC_886_SERIES
        case VP_DEV_886_SERIES:
        case VP_DEV_887_SERIES:
            Vp886TimerHandler(pDevCtx, timerId, overrun, channelId, handle);
            break;
#endif /* VP_CC_886_SERIES */
        default:
            VP_INFO(VpDevCtxType, pDevCtx, ("VpCallTimerHandlerInt: How did we get here?"));
            break;
    }
} /* VpCallTimerHandlerInt */

/**  VpGetTimestampInt()
  Calls the device specific timestamp retrieval function. This may be a
  software or hardware timestamp.
*/
uint32
VpGetTimestampInt(
    VpDevCtxType *pDevCtx)
{
    uint32 timestamp = 0;

    switch (pDevCtx->deviceType) {
#ifdef VP_CC_886_SERIES
        case VP_DEV_886_SERIES:
        case VP_DEV_887_SERIES:
            timestamp = Vp886GetTimestamp32(pDevCtx);
            break;
#endif /* VP_CC_886_SERIES */
        default:
            VP_INFO(VpDevCtxType, pDevCtx, ("VpGetTimestampInt: How did we get here?"));
            break;
    }

    return timestamp;
} /* VpGetTimestampInt */

/**  VpSetDevTimerInt()
  This function calls a device specific function to control a device timer.

  The device timer could be an actual hardware timer or a tick-based software
  timer.  When the timer expires, the device-specific layer should call
  VpProcessTimers().

  If there are timers in the queue, the timer is set to the duration of the
  soonest timer to expire (the front).  If the queue is empty, the device
  timer can be deactivated (enable == FALSE).
*/
void
VpSetDevTimerInt(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes)
{
    bool enable;
    uint32 duration;
    VpTimerQueueNodeType *firstTimer = &(pNodes[pInfo->index]);

    if (firstTimer->active == FALSE) {
        enable = FALSE;
        duration = 0;
    } else {
        enable = TRUE;
        duration = firstTimer->duration;
    }

    VP_INFO(VpDevCtxType, pDevCtx, ("VpSetDevTimerInt: %s %lu ", (enable ? "Enable" : "Disable"), duration));

    switch (pDevCtx->deviceType) {
#ifdef VP_CC_886_SERIES
        case VP_DEV_886_SERIES:
        case VP_DEV_887_SERIES:
            Vp886SetDeviceTimer(pDevCtx, enable, duration);
            break;
#endif /* VP_CC_886_SERIES */
        default:
            VP_ERROR(VpDevCtxType, pDevCtx, ("VpSetDevTimerInt: How did we get here? "));
            break;

    }
} /* VpSetDevTimerInt */


#endif /* VP_CC_886_SERIES */
