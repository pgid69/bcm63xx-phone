/** \file VE890_initialization.c
 * VE890_initialization.c
 *
 * This file is a Quick-Start application for the VE890 tracker Line module.
 *
 * This Quick-Start is tested on the Microsemi ZTAP host. The quick start can
 * be eaisly modifed to run on a customers platform if need be. The main purpose
 * of this Quick-Start is to provide an example of how to:
 *
 *  - Make device and line objects
 *  - Initialize all devices and lines
 *  - Change the line's feed state
 *  - Perform FXS line calibration
 *  - Detect/Respond to FXS and FXO hook events
 *
 * Copyright (c) 2012, Microsemi Corporation.
 */

/*
 * These headers are for ZTAP/Linux purposes only.  The user will not find them
 * in the API
 */
#include <sys/time.h>
#include <signal.h>
#include <stdlib.h>

/*
 * These files are required by all applications.  "vp_api.h" includes all of the
 * API libraries required by the application.  "profiles.h" includes the
 * profiles used by the application.  At a minimum, there must be a Device
 * Profile for the device to initialize.
 */
#include "vp_api.h"
#include "Le71HR8923_profiles.h"

/* ZTAP specific initialization header */
#include "sdk_qs_board.h"

/* Common utility functions */
#include "sdk_qs_utils.h"

/* ZTAP specific values for device ID depending on the MPI Mode */
#define MPI_3WIRE       0x0000 /* 3 wire SPI w/ 2.5us interbyte chipselect off-time */
#define MPI_4WIRE       0x0100 /* 4 wire SPI w/ 2.5us interbyte chipselect off-time */

#define SM              0x0010
#define DIN             0x0000

/* FXS/FXO lines and VP-API termination types */
#define FXS_LINE    0
#define FXO_LINE    1

#define NB_LINE     2

#define FXS_TERM_TYPE   VP_TERM_FXS_LOW_PWR
#define FXO_TERM_TYPE   VP_TERM_FXO_GENERIC
#define FXS_TERM_TYPE_T "VP_TERM_FXS_LOW_PWR"
#define FXO_TERM_TYPE_T "VP_TERM_FXO_GENERIC"


/* Internal Helper functions */
static void alarmHandle(int val);
static void InitSetMasks(void);

/*
 * Application memory space for the API required Device/Line contexts and
 * objects. This application uses the simplest memory scheme -- global, but the
 * API will work with any memory management scheme.
 */

VpDevCtxType devCtx;
VpLineCtxType lineCtx[NB_LINE];
Vp890DeviceObjectType devObj;
Vp890LineObjectType lineObj[NB_LINE];
VpStatusType status;

/*
 * Create a deviceId. Generally this is just the chipselect number of the device on the SPI bus.
 * However, this is a customer defined type in the vp_api_types.h file and can be whatever
 * the customer chooses.
 *
 * The following example is for the Microsemi ZTAP platform and combines the
 * location of the SLAC on the platform and the SPI interface type used by the SLAC.
 */
VpDeviceIdType deviceId = (SM | MPI_4WIRE);

/**
 * Function:  main()
 *
 * Description: This function initializes the API and Devices/Lines.
 * It also starts the "alarmHandle" function to be called at a periodic rate.
 */
int main(void)
{
    struct itimerval timeStruct, timeOldVal;

    /* ZTAP board initialization (platform specific) */
    BoardInit(); /* See "../common/board.c" */
    VpSysInit();

    /*
     * The following structure is specific to Linux.  It sets up a timed call
     * to a user specified function.  For the purpose of this demo, the timing
     * is 10mS and the function that will be called is "alarmHandle".  The timer
     * and function call does not start until "setitimer()" and "signal()" are
     * called (below).
     */
    timeStruct.it_interval.tv_sec = 0L;
    timeStruct.it_interval.tv_usec = 9990L;
    timeStruct.it_value.tv_sec = 0L;
    timeStruct.it_value.tv_usec = 9990L;

    timeOldVal.it_interval.tv_sec = 0L;
    timeOldVal.it_interval.tv_usec = 0L;
    timeOldVal.it_value.tv_sec = 0L;
    timeOldVal.it_value.tv_usec = 0L;

    /* Make the objects (1 device, 2 lines 1FXS / 1FXO) */
    status = VpMakeDeviceObject(VP_DEV_890_SERIES, deviceId, &devCtx, &devObj);
    if (status != VP_STATUS_SUCCESS) {
        QS_DEBUG("Error making the device object: %s\n", MapStatus(status));
        exit(-1);
    }

    status = VpMakeLineObject(FXS_TERM_TYPE, FXS_LINE, &lineCtx[FXS_LINE], &lineObj[FXS_LINE], &devCtx);
    if (status != VP_STATUS_SUCCESS) {
        QS_DEBUG("Error making the FXS line object %d: %s\n", FXS_LINE, MapStatus(status));
        exit(-1);
    }
    QS_DEBUG("--- Initialized for %s on channel: %d ---\n", FXS_TERM_TYPE_T, FXS_LINE);

    status = VpMakeLineObject(FXO_TERM_TYPE, FXO_LINE, &lineCtx[FXO_LINE], &lineObj[FXO_LINE], &devCtx);
    if (status != VP_STATUS_SUCCESS) {
        QS_DEBUG("Error making the FXS line object %d: %s\n", FXO_LINE, MapStatus(status));
        exit(-1);
    }
    QS_DEBUG("--- Initialized for %s on channel: %d ---\n", FXO_TERM_TYPE_T, FXO_LINE);

    /* Start VpInitDevice and wait for the event VP_DEV_EVID_DEV_INIT_CMP */
    status = VpInitDevice(&devCtx, DEV_PROFILE, AC_FXS_600R, DC_25MA_CC, RING_25HZ_SINE, AC_FXO_LC, FXO_DIALING_DEF);
    if (status != VP_STATUS_SUCCESS) {
        QS_DEBUG("Device not properly initialized: %s\n", MapStatus(status));
        exit(-1);
    } else {
        QS_DEBUG("\nDevice Initialization started\n");
    }

    /*
     * The following two function calls were described above as Linux specific.
     * They are used to call a user defined function at set increments.  The
     * user may have several other methods to perform the same.
     */
    signal(SIGVTALRM, alarmHandle);
    setitimer(ITIMER_VIRTUAL, &timeStruct, &timeOldVal);

    /*
     * Once the "tick" is setup, the main program can enter a "while(1)" loop
     * and the user specified function will take over the application (with the
     * help of the Linux OS)
     */

    while(TRUE);
    return 0;
}

/**
 * Function:  alarmHandle()
 *
 * Description: This function is called at a periodic rate to perform all
 * required API operations.  It implements the functional requirements of the
 * application mentioned in the header.
 */
void alarmHandle(int val)
{
    bool deviceEventStatus = FALSE;
    VpEventType event;
    static bool deviceInitialized = FALSE;

    /*
     * This loop will query the FXS device for events, and when an event is
     * found (deviceEventStatus = TRUE), it will parse the event and perform
     * further operations.
     */
    VpApiTick(&devCtx, &deviceEventStatus);

    if(deviceEventStatus == TRUE) {
        while(VpGetEvent(&devCtx, &event)) {
            /* Print the incoming event */
            UtilPrintEvent(&event); /* See "../common/utils.c" */

            /* Parse the RESPONSE category */
            if (event.eventCategory == VP_EVCAT_RESPONSE) {
                if(event.eventId == VP_DEV_EVID_DEV_INIT_CMP) {
                    InitSetMasks();
                    QS_DEBUG("Device Initialization completed\n");
                    status = VpSetLineState(&lineCtx[FXS_LINE], VP_LINE_STANDBY);
                    if (status != VP_STATUS_SUCCESS) {
                        QS_DEBUG("Error setting the FXS_LINE line state: %s\n", MapStatus(status));
                        exit(-1);
                    }
                    status = VpCalLine(&lineCtx[FXS_LINE]);
                    if (status != VP_STATUS_SUCCESS) {
                        QS_DEBUG("Error starting calibration on FXS_LINE: %s\n", MapStatus(status));
                        exit(-1);
                    }

                    status = VpSetLineState(&lineCtx[FXO_LINE], VP_LINE_FXO_LOOP_OPEN);
                    if (status != VP_STATUS_SUCCESS) {
                        QS_DEBUG("Error setting the FXO_LINE line state: %s\n", MapStatus(status));
                        exit(-1);
                    }

                } else if (event.eventId == VP_EVID_CAL_CMP) {
                    deviceInitialized = TRUE;
                    QS_DEBUG("Calibration completed\n\n\n");
                }

            /* Parse the SIGNALING category */
            } else if (event.eventCategory == VP_EVCAT_SIGNALING) {
                if (event.eventId == VP_LINE_EVID_HOOK_OFF) {
                    status = VpSetLineState(event.pLineCtx, VP_LINE_ACTIVE);

                    if (status != VP_STATUS_SUCCESS) {
                        QS_DEBUG("Error running VpSetLineState on channel %d: %s\n", event.channelId, MapStatus(status));
                        exit(-1);
                    }
                    QS_DEBUG("Set channel %d in ACTIVE state\n\n", event.channelId);

                } else if (event.eventId == VP_LINE_EVID_HOOK_ON) {
                    status = VpSetLineState(event.pLineCtx, VP_LINE_STANDBY);

                    if (status != VP_STATUS_SUCCESS) {
                        QS_DEBUG("Error running VpSetLineState on channel %d: %s\n", event.channelId, MapStatus(status));
                        exit(-1);
                    }
                    QS_DEBUG("Set channel %d in STANDBY state\n\n", event.channelId);
                }

            /* Parse the FXO category */
            } else if (event.eventCategory == VP_EVCAT_FXO) {
                if (event.eventId == VP_LINE_EVID_FEED_EN) {
                    status = VpSetLineState(event.pLineCtx, VP_LINE_FXO_LOOP_CLOSE);

                    if (status != VP_STATUS_SUCCESS) {
                        QS_DEBUG("Error running VpSetLineState on channel %d: %s\n", event.channelId, MapStatus(status));
                        exit(-1);
                    }
                    QS_DEBUG("Set channel %d in VP_LINE_FXO_LOOP_CLOSE state\n\n", event.channelId);
                } else if (event.eventId == VP_LINE_EVID_DISCONNECT) {
                    status = VpSetLineState(event.pLineCtx, VP_LINE_FXO_LOOP_OPEN);

                    if (status != VP_STATUS_SUCCESS) {
                        QS_DEBUG("Error running VpSetLineState on channel %d: %s\n", event.channelId, MapStatus(status));
                        exit(-1);
                    }
                    QS_DEBUG("Set channel %d in VP_LINE_FXO_LOOP_OPEN state\n\n", event.channelId);
                }
            }

        } /* end while(VpGetEvent(&devCtx, &event)) */
    }

    return;
}

/**
 * InitSetMasks(void)
 *
 * Description: This function initialize the masks to unmask all events
 * except the rollover event.
 */

void
InitSetMasks(void)
{
    VpOptionEventMaskType eventMask;

    /* mask everything */
    VpMemSet(&eventMask, 0xFF, sizeof(VpOptionEventMaskType));

    /* unmask only the events the application is interested in */
    eventMask.faults = VP_EVCAT_FAULT_UNMASK_ALL;
    eventMask.signaling = VP_EVCAT_SIGNALING_UNMASK_ALL | VP_DEV_EVID_TS_ROLLOVER;
    eventMask.response = VP_EVCAT_RESPONSE_UNMASK_ALL;
    eventMask.test = VP_EVCAT_TEST_UNMASK_ALL;
    eventMask.process = VP_EVCAT_PROCESS_UNMASK_ALL;
    eventMask.fxo = VP_EVCAT_FXO_UNMASK_ALL;

    /* inform the API of the mask */
    VpSetOption(VP_NULL, &devCtx, VP_OPTION_ID_EVENT_MASK, &eventMask);

    return;
}
