/** \file ZL880_initialisation.c
 * ZL880_initialisation.c
 *
 * This file is a Quick-Start application for a ZL880 ABS or Tracker device.
 *
 * This Quick-Start is tested on the Microsemi ZTAP host. The quick start can
 * be eaisly modifed to run on a customers platform if need be. The main purpose
 * of this Quick-Start is to provide an example of how to:
 *
 *  - Make device and line objects
 *  - Initialize all devices and lines
 *  - Change the line's feed state
 *  - Perform line calibration
 *  - Perform VeriVoice Line Test
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

/* LT-API (line test) library header */
#include "lt_api.h"


/* Pick the supply type TRACKER Fly Back, Tracker Inverting Boost or ABS */
#define FLY_BACK_SUPPLY         0
#define INVERTING_BOOST_SUPPLY  1
#define ABS_SUPPLY              2

#define SUPPLY_CONFIGURATION FLY_BACK_SUPPLY

#if (SUPPLY_CONFIGURATION == FLY_BACK_SUPPLY)
    #include "ZLR88721L_FB_profiles.h"

#elif (SUPPLY_CONFIGURATION == INVERTING_BOOST_SUPPLY)
    #include "ZLR88722L_IB_profiles.h"

#elif (SUPPLY_CONFIGURATION == ABS_SUPPLY)
    #include "ZLR88621L_ABS_profiles.h"

#else
    #error "SUPPLY_CONFIGURATION is incorrectly set"
#endif


/* ZTAP specific initialization header */
#include "sdk_qs_board.h"

/* Common utility functions */
#include "sdk_qs_utils.h"

/* ZTAP specific values for device ID depending on the MPI Mode */
#define MPI_3WIRE       0x0000 /* 3 wire SPI w/ 2.5us interbyte chipselect off-time */
#define MPI_4WIRE       0x0100 /* 4 wire SPI w/ 2.5us interbyte chipselect off-time */
#define SPI8_3WIRE      0x0200 /* 3 wire SPI w/ one clock chipselect off-time */
#define SPI8_4WIRE      0x0400 /* 4 wire SPI w/ one clock chipselect off-time */
#define SPI8_3WIRE_CFM  0x0300 /* 3 wire SPI w/o chipselect off-time per VpMpiCmd() transation */
#define SPI8_4WIRE_CFM  0x0500 /* 4 wire SPI w/o chipselect off-time per VpMpiCmd() transation */

#define SM              0x0010
#define DIN             0x0000

/* Number of lines supported by the quick start */
#define NB_LINE     2

/*
 * Union of LT-API input structs
 */
typedef union {
    LtLineVInputType lineVInp;
    LtRingerInputType ringersInp;
    LtRohInputType rohInp;
    LtResFltInputType resFltInp;
    /* ... Add other input structs here */
} TestInputType;


/*
 * Application memory space for the required VP-API Device/Line contexts and
 * objects. This application uses the simplest memory scheme -- global, but the
 * API will work with any memory management scheme.
 */

VpDevCtxType devCtx;
VpLineCtxType lineCtx[NB_LINE];
Vp886DeviceObjectType devObj;
Vp886LineObjectType lineObj[NB_LINE];

/*
 * Application memory space for line testing. This example only allocates
 * enough memory to perform line testing for a single channel of a single device.
 */
VpLineCtxType *plineCtxUnderTest = NULL;
TestInputType gTestInputs;
LtTestAttributesType gTestAtts;
LtTestTempType gTestTempData;
LtTestResultType gTestResults;
LtTestCtxType gTestCtx;



/* Internal Helper functions */
static void alarmHandle(int val);
static void InitSetMasks(void);
static void StartLineTest(uint8 lineNum, LtTestIdType testId);
static void ProcessLineTestEvent(VpEventType *pEvent);
static void PrintResults(LtTestStatusType ltStatus, LtTestResultType *pTestResults);


/*
 * Create a deviceId. Generally this is just the chipselect number of the device on the SPI bus.
 * However, this is a customer defined type in the vp_api_types.h file and can be whatever
 * the customer chooses.
 *
 * The following example is for the Microsemi ZTAP platform and combines the
 * location of the SLAC on the platform and the SPI interface type used by the SLAC.
 */
VpDeviceIdType deviceId = (DIN | MPI_4WIRE);

/**
 * Function:  main()
 *
 * Description: This function initializes the API and Devices/Lines.
 * It also starts the "alarmHandle" function to be called at a periodic rate.
 */
int main(void)
{
    VpStatusType status;
    struct itimerval timeStruct, timeOldVal;
    uint8 lineNum = 0;

    /* ZTAP board initialization (platform specific) */
    BoardInit(); /* See "../common/board.c" */
    VpSysInit();

    /*
     * The following structure is specific to Linux.  It sets up a timed call
     * to a user specified function.  For the purpose of this demo, the timing
     * is 20mS and the function that will be called is "alarmHandle".  The timer
     * and function call does not start until "setitimer()" and "signal()" are
     * called (below).
     */
    timeStruct.it_interval.tv_sec = 0L;
    timeStruct.it_interval.tv_usec = 19990L;
    timeStruct.it_value.tv_sec = 0L;
    timeStruct.it_value.tv_usec = 19990L;

    timeOldVal.it_interval.tv_sec = 0L;
    timeOldVal.it_interval.tv_usec = 0L;
    timeOldVal.it_value.tv_sec = 0L;
    timeOldVal.it_value.tv_usec = 0L;

    /* Make the objects (1 device, 2 lines) */
#if (SUPPLY_CONFIGURATION == ABS_SUPPLY)
    status = VpMakeDeviceObject(VP_DEV_886_SERIES, deviceId, &devCtx, &devObj);
#else
    status = VpMakeDeviceObject(VP_DEV_887_SERIES, deviceId, &devCtx, &devObj);
#endif
    if (status != VP_STATUS_SUCCESS) {
        QS_DEBUG("Error making the device object: %s\n", MapStatus(status));
        exit(-1);
    }

    for (; lineNum < NB_LINE; lineNum++) {
        status = VpMakeLineObject(VP_TERM_FXS_LOW_PWR, lineNum, &lineCtx[lineNum], &lineObj[lineNum], &devCtx);
        if (status != VP_STATUS_SUCCESS) {
            QS_DEBUG("Error calling VpMakeLineObject on lineNum %i: %s\n", lineNum, MapStatus(status));
            exit(-1);
        }
        QS_DEBUG("--- Created lineNum %i---\n", lineNum);
    }

    /* Start VpInitDevice and wait for the event VP_DEV_EVID_DEV_INIT_CMP */
#if (SUPPLY_CONFIGURATION == FLY_BACK_SUPPLY)
    status = VpInitDevice(&devCtx, FB_DEV_PROFILE, FB_AC_600_FXS, FB_DC_25MA_CC, FB_RING_25HZ_SINE, VP_PTABLE_NULL, VP_PTABLE_NULL);
#elif (SUPPLY_CONFIGURATION == INVERTING_BOOST_SUPPLY)
    status = VpInitDevice(&devCtx, IB_DEV_PROFILE, IB_AC_600_FXS, IB_DC_25MA_CC, IB_RING_25HZ_SINE, VP_PTABLE_NULL, VP_PTABLE_NULL);
#elif (SUPPLY_CONFIGURATION == ABS_SUPPLY)
    status = VpInitDevice(&devCtx, ABS_DEV_PROFILE, ABS_AC_600_FXS, ABS_DC_23MA_CC, ABS_RING_25HZ_SINE, VP_PTABLE_NULL, VP_PTABLE_NULL);
#else
    #error "SUPPLY_CONFIGURATION is incorrectly set"
#endif

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


    while(1);

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
    VpStatusType status;
    VpEventType event;
    static bool deviceInitialized = FALSE;
    static uint8 cmpCounter = 0;

    /*
     * This loop will query the FXS device for events, and when an event is
     * found, it will parse the event and perform further operations.
     */

    while(VpGetEvent(&devCtx, &event)) {
        /* Print the incoming event */
        UtilPrintEvent(&event); /* See "../common/utils.c" */

        /*
         * A channel under test should process all line events without
         * passing the events over to the main event handling loop
         */
        if ((plineCtxUnderTest != NULL) && (plineCtxUnderTest == event.pLineCtx)) {
            ProcessLineTestEvent(&event);
            continue;
        }

        /* Parse the RESPONSE category */
        if (event.eventCategory == VP_EVCAT_RESPONSE) {
            if(event.eventId == VP_DEV_EVID_DEV_INIT_CMP) {
                uint8 lineNum = 0;
                InitSetMasks();
                QS_DEBUG("Device Initialization completed\n");

                for (; lineNum < NB_LINE; lineNum++) {
                    status = VpSetLineState(&lineCtx[lineNum], VP_LINE_STANDBY);
                    if (status != VP_STATUS_SUCCESS) {
                                QS_DEBUG("Error setting lineNum %i to VP_LINE_STANDBY: %s\n", lineNum, MapStatus(status));
                        exit(-1);
                    }
                    status = VpCalLine(&lineCtx[lineNum]);
                    QS_DEBUG("Starting Calibration on lineNum %i\n", lineNum);
                    if (status != VP_STATUS_SUCCESS) {
                        QS_DEBUG("Error calling VpCalLine on lineNum %i:: %s\n", lineNum, MapStatus(status));
                        exit(-1);
                    }
                }

            } else if (event.eventId == VP_EVID_CAL_CMP) {
                if (++cmpCounter == NB_LINE) {
                    deviceInitialized = TRUE;
                    QS_DEBUG("Calibration completed on both lines ... starting Line Test\n\n");
                    StartLineTest(0, LT_TID_LINE_V);
                }
            }
        }

    } /* end while(VpGetEvent(&devCtx, &event)) */

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

    /* inform the API of the mask */
    VpSetOption(VP_NULL, &devCtx, VP_OPTION_ID_EVENT_MASK, &eventMask);

    return;
}

/**
 * StartLineTest()
 *
 * Description: This function starts a line test for the specified line number.
 */

void
StartLineTest(
    uint8 lineNum,
    LtTestIdType testId)
{
    LtTestStatusType ltStatus = LT_STATUS_RUNNING;
    
    /* user handle value (can be any number)*/
    uint16 ltParamHandle = 0xBEEF; 
    
    /* 
     * if the LtTestAttributesType argument to LtStartTest is NULL
     * then the LT-API will use a set of default inputs,
     * criteria, and topology value
     */
    LtTestAttributesType *pTestAtts = &gTestAtts;
    VpMemSet(pTestAtts, 0, sizeof(LtTestAttributesType));

    /* is a line test already in progress */
    if (plineCtxUnderTest != NULL) {
        QS_DEBUG("Line Test is already in progress\n");
        return;
    }

    /* 
     * The following shows examples of the different ways that input
     * parameters can be passed to the LT-API. These are for example
     * purposes and should be modified to fit the customer's needs.
     */
    switch (testId) {
    
        case LT_TID_LINE_V:
            /* using all default input and criteria values built into the LT-API */
            pTestAtts->ltDbgFlag = 0xFFFFFFFF;
            break;

        case LT_TID_ROH:
            /* disable runtime debug */
            pTestAtts->ltDbgFlag = 0;
            break;

        case LT_TID_RINGERS:
            /* configure the input params */
            gTestInputs.ringersInp.ringerTestType = LT_RINGER_ELECTRONIC_PHNE_TEST_3_ELE;
            gTestInputs.ringersInp.vRingerTest = 56000;
            gTestInputs.ringersInp.freq = 20000;
            gTestInputs.ringersInp.renFactor = 1210000;

            /* set the input params pointer to the global variable */
            pTestAtts->inputs.pRingersInp = &gTestInputs.ringersInp;

            /* disable runtime debug */
            pTestAtts->ltDbgFlag = 0;
            break;

        default: 
            QS_DEBUG("Unsupported quick start testid %i\n", testId);
            return;    
    }

    /* Start the line test */
    ltStatus = LtStartTest(&lineCtx[lineNum], testId, ltParamHandle, &gTestAtts, 
        &gTestTempData, &gTestResults, &gTestCtx);

    if (ltStatus != LT_STATUS_RUNNING) {
        QS_DEBUG("Unable to start testId %i: ltStatus = %i\n", testId, ltStatus);
        exit(1);
    }
    
    /* indicate that a test is in progress */
    plineCtxUnderTest = &lineCtx[lineNum];
    
    return;
}

/**
 * ProcessLineTestEvent()
 *
 * Description: This function passes all VP-API events into the the LT-API
 * event handler for a given line.
 */

void
ProcessLineTestEvent(
    VpEventType *pEvent)
{
    VpStatusType status = VP_STATUS_SUCCESS;
    LtTestStatusType ltStatus = LT_STATUS_RUNNING;
    VpResultsType vpResults;
    LtEventType ltEvent;
    
    /* create LT-API event struct */
    ltEvent.pVpEvent = pEvent;
    
    /* get vp-api result if available */
    if (pEvent->hasResults) {
        status = VpGetResults(pEvent, &vpResults);
        if (status != VP_STATUS_SUCCESS) {
            QS_DEBUG("VpGetResult() failed: %i\n", status);
            return;
        }
        ltEvent.pResult = &vpResults;
    } else {
        ltEvent.pResult = NULL;
    }

    /* process the event */
    ltStatus = LtEventHandler(&gTestCtx, &ltEvent);

    switch (ltStatus) {
        case LT_STATUS_DONE:

            /* Test has completed - parse the results */
            PrintResults(ltStatus, &gTestResults);

            /* we are done with the current test*/
            plineCtxUnderTest = NULL;

            /* run another test */
            switch (gTestResults.testId) {
                case LT_TID_LINE_V:
                    StartLineTest(0, LT_TID_ROH);
                    break;

                case LT_TID_ROH:
                    StartLineTest(0, LT_TID_RINGERS);
                    break;

                default:
                    exit(1);
            }
            break;
            
        case LT_STATUS_RUNNING:
        case LT_STATUS_ABORTED:
            /* 
             * If the line test is running or aborting the application must 
             * continue to call the LT-API event handler on subsequent events.
             */
            break;

        default:
            /* All other status indications are errors */
            break;
    }

    return;
}

void 
PrintResults(
    LtTestStatusType ltStatus,
    LtTestResultType *pTestResults)
{
    QS_DEBUG("LT-API test result status: %i\n", ltStatus);
    return;
}
