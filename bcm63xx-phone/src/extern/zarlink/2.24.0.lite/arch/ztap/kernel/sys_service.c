/** \file sys_service.c
 * sys_service.c
 *
 *  This file implements the required system services for the API-II using a
 * Linux OS running on the UVB.  The user should replace the functions provided
 * here with the equivalent based on their OS and hardware.
 *
 * Copyright (c) 2011, Microsemi Corporation
 */
#include <asm/uaccess.h>
#include <asm/delay.h>
#include <asm/semaphore.h>

#include "vpapi_mod.h"
#include "telecom_mod.h"
#include "telecom_fpga.h"


#define MAX_SS_TEST_HEAPS   4
#define MAX_SS_CONNECTORS   2
#define MAX_SS_CHIPSELECTS  8

typedef struct {
    VpTestHeapType  testHeap;
    int             testBufId;          /* Test Buffer ID assigned (-1, if none avail.) */
} SysSerTestAcquireType;


typedef struct{
    int pid;
    int cnt;
    struct semaphore criticalSem;
    struct semaphore sectionSem;
    spinlock_t       lock;
} VpSysCriticalParamType;

/*******************************************************************************
 * Global memory requirements
 *******************************************************************************/
VpSysCriticalParamType gSysCriticalSec[VP_NUM_CRITICAL_SEC_TYPES];
SysSerTestAcquireType gTestHeaps[MAX_SS_TEST_HEAPS];

/*
 * VpSysWait() function implementation is needed only for CSLAC devices
 * (880, 790). For other devices this function could be commented.
 */
void
VpSysWait(
    uint8 time)  /* Time specified in increments of 125uS (e.g. 4 = 500uS) */
{
    /* Blocking delay function added here based on OS */
    udelay(125*time);
}

void
VpSysServiceInit(void)
{
	int i = 0;
    int heap = 0;

    /* init the test heaps */
    for (; heap < MAX_SS_TEST_HEAPS; heap++) {
        gTestHeaps[heap].testBufId = -1;
    }


    for(;i < VP_NUM_CRITICAL_SEC_TYPES; i++)
    {
        gSysCriticalSec[i].pid = -1;
        gSysCriticalSec[i].cnt = 0;
        spin_lock_init(&gSysCriticalSec[i].lock);
        sema_init( &gSysCriticalSec[i].criticalSem, 1 );
        sema_init( &gSysCriticalSec[i].sectionSem, 1 );
    }

    return;
}

/*
 * VpSysServiceToggleLed():
 *
 *  This function was added as a debug utility for ZTAP. The function will 
 *  toggle an LED from its existing state to the opposite state
 *
 * Params:
 *  uint8 ledNum: LED to toggle
 *
 * Return:
 *  N/A
 */
void 
VpSysServiceToggleLed(
    uint8 ledNum)
{
    unsigned long ledReg;
    unsigned long newLedReg;
    unsigned long ledBit;
    unsigned long ledMask;

    if (ledNum > 15) {
        return;
    }

    ledBit = (1 << ledNum);
    ledMask = (ledBit * 65536);

    /* get the current led reg value */
    ledReg = TelecomFpgaRead(TELECOM_FPGA_LED_CTRL_REG);

    if (ledReg & ledBit) {
        newLedReg =  (ledReg & ~(ledBit)) | ledMask;
    } else {
        newLedReg =  (ledReg | ledBit) | ledMask;
    }
    //printk("ledReg 0x%08lx : ledBit 0x%08lx : ledMask 0x%08lx : newLedReg 0x%08lx \n", ledReg, ledBit, ledMask, newLedReg);
    /* toggle the LED */
    TelecomFpgaWrite(TELECOM_FPGA_LED_CTRL_REG, newLedReg);


    /* mask the led so */
    //TelecomFpgaWrite(TELECOM_FPGA_LED_CTRL_REG, (ledReg & ~ledMask));
    return;
}

/*
 * VpSysEnterCritical() VpSysExitCritical():
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

/* 
 * Do to the fact that the HAL layer may need to sleep (MPI, ZSI and ZMPI)
 * we must use semaphores. We are locally using spin lock to prevent any 
 * issues with interacting with the semaphore. 
 *
 * http://www.linuxjournal.com/article/5833?page=0,0
 *
 */
#if 1
    unsigned long flags;
    /* LOCK: need the lock to modify global variables */
    spin_lock_irqsave(&gSysCriticalSec[criticalSecType].lock, flags);

    if( (gSysCriticalSec[criticalSecType].pid == -1) ||
        (gSysCriticalSec[criticalSecType].pid == current->pid) ) {

        /* check for nested critical sections */
        if(gSysCriticalSec[criticalSecType].pid == -1) {

            /* store the pid requesting the critical section */
            gSysCriticalSec[criticalSecType].pid = current->pid;

            /* UNLOCK: need to un lock so that we can go to sleep if necessary */
            spin_unlock_irqrestore(&gSysCriticalSec[criticalSecType].lock, flags);

            /* SLEEP: If we need to */
            down(&gSysCriticalSec[criticalSecType].criticalSem);

            /* LOCK: need to relock to protect the global data */
            spin_lock_irqsave(&gSysCriticalSec[criticalSecType].lock, flags);
        }

        gSysCriticalSec[criticalSecType].cnt++;

    } else {

        /* UNLOCK: need to un lock so that we can go down */
        spin_unlock_irqrestore(&gSysCriticalSec[criticalSecType].lock, flags);

        /* SLEEP: New task must wait for semaphore */
        down(&gSysCriticalSec[criticalSecType].criticalSem);

        /* LOCK: need to relock to protect the global data */
        spin_lock_irqsave(&gSysCriticalSec[criticalSecType].lock, flags);

        gSysCriticalSec[criticalSecType].pid = current->pid;
        gSysCriticalSec[criticalSecType].cnt++;
    }

    /* UNLOCK: need to unlock before leaving the function */
    spin_unlock_irqrestore(&gSysCriticalSec[criticalSecType].lock, flags);
#endif
    return 1;

} /* VpSysEnterCritical() */


uint8
VpSysExitCritical(
    VpDeviceIdType deviceId,
    VpCriticalSecType criticalSecType)
{
#if 1
    unsigned long flags;
    spin_lock_irqsave(&gSysCriticalSec[criticalSecType].lock, flags);

    /* Check if VpSysExitCritical() was called more times than VpSysEnterCritical */
    if(gSysCriticalSec[criticalSecType].cnt <= 0) {
        /* UNLOCK: need to unlock before leaving the function */
        spin_unlock_irqrestore(&gSysCriticalSec[criticalSecType].lock, flags);
        printk("\n***** VpSysExitCritical() was called overly  criticalSecType = %d*****", criticalSecType);
        return 1;
    }

    gSysCriticalSec[criticalSecType].cnt--;

    if(gSysCriticalSec[criticalSecType].cnt == 0) {

        /* Reset the pid so that the next process can us it */
        gSysCriticalSec[criticalSecType].pid = -1;

        /* WAKE: The next task waiting for semaphore */
        up(&gSysCriticalSec[criticalSecType].criticalSem);
    } else {
        /* nothing to do */
    }

    /* UNLOCK: need to unlock before leaving the function */
    spin_unlock_irqrestore(&gSysCriticalSec[criticalSecType].lock, flags);

#endif
    return 1;

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
#if 1
    uint32 interruptMaskReg = TelecomFpgaRead(TELECOM_FPGA_INT_MASK_REG);

    /* Determine what bit in register belongs to this device Id */
    uint16 shift = (deviceId & 0x0007);

    /* Do we need to shift ? */
    if (deviceId & 0x0010) {
        shift += 8;
    }
    /*printk("VpSysDisableInt()\n");*/
    TelecomFpgaWrite(TELECOM_FPGA_INT_MASK_REG, (interruptMaskReg | (1 << shift)) );
#endif
    return;
}
void
VpSysEnableInt(
    VpDeviceIdType deviceId)
{
#if 1
    uint32 interruptMaskReg = TelecomFpgaRead(TELECOM_FPGA_INT_MASK_REG);

    /* Determine what bit in register belongs to this device Id */
    uint16 shift = (deviceId & 0x0007);

    /* Do we need to shift ? */
    if (deviceId & 0x0010) {
        shift += 8;
    }
    /*printk("VpSysEnableInt()\n");*/
    TelecomFpgaWrite(TELECOM_FPGA_INT_MASK_REG, ~(~interruptMaskReg | (1 << shift)) );
#endif
    return;

}

bool
VpSysTestInt(
    VpDeviceIdType deviceId)
{

    /* 
     * Special ZTAP requirment. Some Microsemi line modules do not
     * bring the device interrupt pins over to the connector. 
     * When ZTAP is compiled for efficient polled mode we must fool the
     * api into believing that it needs to service the device every tick
     * this will essentially force the api back into a simple polled mode
     * of operation. S.H. 02-15-10 
     * 
     */
    if (deviceId & 0x8000) {
        return TRUE;
    } else {
        /*
         * Decode the device Id into an interrupt pin.
         * Check the state of the interrupt pin.
         * If the pin is high and unmasked then an interrupt is
         * present (return TRUE). Otherwise return FALSE.
         *
         */

        /* Read the FPGA's interrupt register */
        uint32 interruptReg = TelecomFpgaRead(TELECOM_FPGA_INT_REG);
        uint32 interruptMaskReg = TelecomFpgaRead(TELECOM_FPGA_INT_MASK_REG);

        /* Determine what bit in register belongs to this device Id */
        uint16 shift = (deviceId & 0x0007);

        /* Do we need to shift ? */
        if (deviceId & 0x0010) {
            shift += 8;
        }

        if (interruptReg & ~interruptMaskReg & (1 << shift)) {
            return TRUE;
        } else {
            return FALSE;
        }
    }
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

