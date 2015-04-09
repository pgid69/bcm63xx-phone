/** \file vp_timer_queue.h
 * vp_timer_queue.h
 *
 * Header file for the VP Timer Queue Module c file.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 10976 $
 * $LastChangedDate: 2013-05-21 18:08:34 -0500 (Tue, 21 May 2013) $
 */

#ifndef VP_TIMER_QUEUE_H
#define VP_TIMER_QUEUE_H

#include "vp_api.h"
#include "vp_api_types.h"
#include "sys_service.h"

#define VP_END_OF_TIMER_QUEUE -1

/* Note: The following functions may be called from 
   device specific timer function wrappers if necessary */
EXTERN bool
VpInitTimerQueue(
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes);

EXTERN bool
VpInsertTimer(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint32 timerDuration,
    uint8 channelId,
    uint32 handle);

EXTERN bool
VpRemoveTimer(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint8 channelId,
    uint32 handle,
    bool matchHandle);

EXTERN bool
VpRemoveChannelTimers(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint8 channelId);

EXTERN bool
VpRestartTimer(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint32 timerDuration,
    uint8 channelId,
    uint32 handle);

EXTERN bool
VpExtendTimer(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes,
    uint16 timerId,
    uint32 timerDuration,
    uint8 channelId,
    uint32 handle);

EXTERN void 
VpProcessTimers(
    VpDevCtxType *pDevCtx,
    VpTimerQueueInfoType *pInfo,
    VpTimerQueueNodeType *pNodes);
    
#endif /* VP_TIMER_QUEUE_H */




