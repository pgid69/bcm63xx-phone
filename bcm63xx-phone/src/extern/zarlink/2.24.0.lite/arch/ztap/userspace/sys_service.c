/** \file sys_service.c
 * sys_service.c
 *
 *  This file implements the required system services for the API-II using a
 * Linux OS running on the UVB.  The user should replace the functions provided
 * here with the equivalent based on their OS and hardware.
 *
 *  Note: Applications using these functions must be linked with -lrt.
 *
 * Copyright (c) 2011, Microsemi Corporation
 */
#include "vp_api_types.h"
#include "vp_api.h"
#include "sys_service.h"

#include <semaphore.h>
#include <stdio.h>      /* perror() */
#include <unistd.h>     /* usleep() */

#define MAX_SS_TEST_HEAPS   4

typedef struct {
    VpTestHeapType  testHeap;
    int             testBufId;          /* Test Buffer ID assigned (-1, if none avail.) */
} SysSerTestAcquireType;

/*******************************************************************************
 * Global memory requirements
 *******************************************************************************/
SysSerTestAcquireType gTestHeaps[MAX_SS_TEST_HEAPS];

/* Semaphore for critical sections.  For increased concurrency, separate
   semaphores could be created for each bus, chip-select, etc.  On the ZTAP,
   everything is routed through a single FPGA, so we create a single
   semaphore. */
#ifndef VP_CC_KWRAP   
static sem_t fpga_sem;
#endif

/* Variable for keeping track of the nesting level of critical sections. */
static int criticalNesting = 0;

/*
 * VpSysWait() function implementation is needed only for CSLAC devices
 * (880, 790). For other devices this function could be commented.
 */
void
VpSysWait(
    uint8 time)  /* Time specified in increments of 125uS (e.g. 4 = 500uS) */
{
    /*extern int usleep(unsigned int); */
    usleep(125 * time);
}

/*
 * VpSysInit()
 *
 * This function isn't part of the official System Services Layer, but is
 * required for the ZTAP implementation to initialize semaphores.  Applications
 * should call this function once (and only once) before calling any VP-API-II
 * functions that might use critical sections.
 */
bool
VpSysInit(void)
{
    int heap = 0;
#ifndef VP_CC_KWRAP
    int result = sem_init(&fpga_sem, 0, 1);
#endif

    /* init the test heaps */
    for (; heap < MAX_SS_TEST_HEAPS; heap++) {
        gTestHeaps[heap].testBufId = -1;
    }

#ifndef VP_CC_KWRAP
    if (result == -1) {
        perror("VpSysInit()");
    }

    return (result == 0) ? TRUE : FALSE;
#else
    return TRUE;
#endif    
}

/*
 * VpSysEnterCritical(), VpSysExitCritical():
 *
 *  These functions allow for disabling interrupts while executing nonreentrant
 * portions of VoicePath API code. Note that the following implementations of
 * enter/exit critical section functions are simple implementations. These
 * functions could be expanded (if required) to handle different critical
 * section types differently.
 *
 * Params:
 *  VpDeviceIdType deviceId: Device Id (chip select ID)
 *  VpCriticalSecType: Critical section type
 *
 * Return:
 *  Number of critical sections currently entered for the device.
 */
uint8
VpSysEnterCritical(
    VpDeviceIdType deviceId,
    VpCriticalSecType criticalSecType)
{
    int result;

    if (criticalNesting == 0) {
        /* The process blocks until the FPGA becomes available. */
#ifndef VP_CC_KWRAP
        result = sem_wait(&fpga_sem);
#else
        result = 0;
#endif
        if (result == -1) {
            perror("VpSysEnterCritical()");
            return 0;
        }
    }

    criticalNesting++;

    /* Sanity check.  The API almost never nests critical sections, but when it
       does, the nesting depth is never more than 3.  However, the application
       may wish to call VpSysEnterCritical() and VpSysExitCritical() directly;
       in this case, the nesting depth is determined by application behavior. */
    if (criticalNesting > 3) {
        fprintf(stderr, "Critical section nesting depth is %d!\n", criticalNesting);
    }

    return criticalNesting;
} /* VpSysEnterCritical() */

uint8
VpSysExitCritical(
    VpDeviceIdType deviceId,
    VpCriticalSecType criticalSecType)
{
    int result;

    /* Sanity check for unmatched calls to VpSysEnterCritical() and
       VpSysExitCritical(). */
    if (criticalNesting == 0) {
        fprintf(stderr, "Extra call to VpSysExitCritical()!\n");
        return 0;
    }

    criticalNesting--;

    if (criticalNesting == 0) {

        /* Allow some other process to access the FPGA. */
#ifndef VP_CC_KWRAP
        result = sem_post(&fpga_sem);
#else
        result = 0;
#endif          
        if (result == -1) {
            perror("VpSysExitCritical()");
            criticalNesting = 1;
        }
    }

    return criticalNesting;
} /* VpSysExitCritical() */

/**
 * VpSysDisableInt(), VpSysEnableInt(), and VpSysTestInt()
 *
 *  These functions are used by the CSLAC device family for interrupt driven
 * polling modes. These are called by the API to detect when a non-masked
 * device status has changed.  If using SIMPLE_POLL mode, these functions do not
 * require implementation.
 *
 * Preconditions:
 *  None. The implementation of these functions is architecture dependent.
 *
 * Postconditions:
 *  VpSysDisableInt() - The interrupt associated with the deviceId passed is
 * disabled.
 *
 * VpSysEnableInt() - The interrupt associated with the deviceId passed is
 * enabled.
 *
 * VpSysTestInt() - The return value is TRUE if an interrupt occurred, otherwise
 * return FALSE.
 *
 * These functions are needed only for CSLAC devices
 * (880, 790). For other devices these functions could be commented.
 *
 */
void
VpSysDisableInt(
    VpDeviceIdType deviceId)
{
    return;
}

void
VpSysEnableInt(
    VpDeviceIdType deviceId)
{
    return;
}

bool
VpSysTestInt(
    VpDeviceIdType deviceId)
{
    return FALSE;
}

/**
 * VpSysDtmfDetEnable(), VpSysDtmfDetDisable()
 *
 *  These functions are used by the CSLAC device family for devices that do not
 * internally detect DTMF. It is used for Caller ID type-II and is provided to
 * enable external DTMF detection.
 *
 * Preconditions:
 *  None. The implementation of these functions is application dependent.
 *
 * Postconditions:
 *  VpSysDtmfDetEnable() - The device/channel resource for DTMF detection is
 * enabled.
 *
 *  VpSysDtmfDetDisable() - The device/channel resource for DTMF detection is
 * disabled.
 *
 * These functions are needed only for CSLAC devices
 * (880, 790). For other devices these functions could be commented.
 *
 */
void
VpSysDtmfDetEnable(
    VpDeviceIdType deviceId,
    uint8 channelId)
{
}

void
VpSysDtmfDetDisable(
    VpDeviceIdType deviceId,
    uint8 channelId)
{
}

/*
 * The following functions VpSysTestHeapAcquire(),  VpSysTestHeapRelease()
 * VpSysPcmCollectAndProcess() and are needed only for CSLAC devices
 * (880). For other devices these functions could be commented. Please see
 * the LineTest API documentation for function requirements.
 */
void *
VpSysTestHeapAcquire(
    uint8 *pHeapId)
{
    int i = 0;

    do {
        if (gTestHeaps[i].testBufId == -1) {
            gTestHeaps[i].testBufId = i;
            *pHeapId = i;
            return &gTestHeaps[i].testHeap;
        }
    } while (++i < MAX_SS_TEST_HEAPS);

    return VP_NULL;
} /* VpSysTestHeapAcquire() */

bool
VpSysTestHeapRelease(
    uint8 heapId)
{
    if (heapId >= MAX_SS_TEST_HEAPS) {
        return FALSE;
    }
    gTestHeaps[heapId].testBufId = -1;
    return TRUE;
} /* VpSysTestHeapRelease() */

void
VpSysPcmCollectAndProcess(
    void *pLineCtx,
    VpDeviceIdType deviceId,
    uint8 channelId,
    uint8 startTimeslot,
    uint16 operationTime,
    uint16 settlingTime,
    uint16 operationMask)
{
} /* VpSysPcmCollectAndProcess() */


void 
VpSysServiceToggleLed(
    uint8 ledNum)
{
} /* VpSysServiceToggleLed() */

