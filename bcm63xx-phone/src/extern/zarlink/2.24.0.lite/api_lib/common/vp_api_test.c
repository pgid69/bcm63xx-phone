/** \file vp_api_test.c
 * vp_api_test.c
 *
 *  This file contains the implementation of top level VoicePath API-II.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 6827 $
 * $LastChangedDate: 2010-04-01 17:51:38 -0500 (Thu, 01 Apr 2010) $
 */

/* INCLUDES */
#include "vp_api.h"     /* Typedefs and function prototypes for API */

#include "vp_hal.h"
#include "vp_api_int.h" /* Device specific typedefs and function prototypes */
#include "sys_service.h"

#if defined (VP_CC_880_SERIES)
#include "vp880_api_int.h"
#endif


/******************************************************************************
 *                            TEST FUNCTIONS                                  *
 ******************************************************************************/
#ifdef VP_CC_TEST_LINE
/**
 * VpTestLine()
 *  This function performs the requested test. For more information see
 * VP-API-II user guide.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Starts/Stops requested test actions.
 */
VpStatusType
VpTestLine(
    VpLineCtxType *pLineCtx,
    VpTestIdType test,
    const void *pArgs,
    uint16 handle)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "TestLine");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(TestLine, (pLineCtx, test, pArgs, handle));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "TestLine", status);
    return status;
} /* VpTestLine() */
#endif /* VP_CC_TEST_LINE */

#ifdef VP_CC_TEST_LINE_CALLBACK
/**
 * VpTestLineCallback()
 * This function is normally called by an outside application after collecting
 * and processing PCM data requested by the previous PcmCollect routine. If
 * the API is operating in EZ mode this function is actually called from within
 * the API by the VpEzPcmCallback() function.
 *
 * The results structure pointed to by the pResults argument are copied into
 * the TestHeap for later use.
 *
 * Parameters:
 *  pLineCtx    - pointer to the line context
 *  pResults    - pointer to results from the pcmCollect system service layer
 */
VpStatusType
VpTestLineCallback(
    VpLineCtxType *pLineCtx,
    VpPcmOperationResultsType *pResults)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "TestLineCallback");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(TestLineCallback, (pLineCtx, pResults));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "TestLineCallback", status);
    return status;
} /* VpTestLineCallback() */
#endif /* VP_CC_TEST_LINE_CALLBACK */

#ifdef VP_CC_CODE_CHECKSUM
/**
 * VpCodeCheckSum()
 *  This function performs a checksum calculation over the loaded code (for
 * program and data memory). For more information see VP-API-II user guide.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Initiates a checksum verification action.
 */
VpStatusType
VpCodeCheckSum(
    VpDevCtxType *pDevCtx,
    uint16 handle)
{
    VpStatusType status;
    VP_API_ENTER(VpDevCtxType, pDevCtx, "CodeCheckSum");

    /* Basic argument checking */
    if (pDevCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        status = VP_CALL_DEV_FUNC(CodeCheckSum, (pDevCtx, handle));
    }

    VP_API_EXIT(VpDevCtxType, pDevCtx, "CodeCheckSum", status);
    return status;
} /* VpCodeCheckSum() */
#endif /* VP_CC_CODE_CHECKSUM */

#ifdef VP_CC_SELF_TEST
/**
 * VpSelfTest()
 *  This function performs a self test on a given line or for all lines in the
 * device. For more information see VP-API-II user guide.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Initiates self test.
 */
VpStatusType
VpSelfTest(
    VpLineCtxType *pLineCtx)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "SelfTest");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(SelfTest, (pLineCtx));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "SelfTest", status);
    return status;
} /* VpSelfTest() */
#endif /* VP_CC_SELF_TEST */

#ifdef VP_CC_FILL_TEST_BUF
/**
 * VpFillTestBuf()
 *  This function is used to load test vector data in to the device internal
 * memory. For more information see VP-API-II user guide.
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Loads the given test data in to the device.
 */
VpStatusType
VpFillTestBuf(
    VpLineCtxType *pLineCtx,
    uint16 length,
    VpVectorPtrType pData)
{
    VpStatusType status;
    VP_API_ENTER(VpLineCtxType, pLineCtx, "FillTestBuf");

    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(FillTestBuf, (pLineCtx, length, pData));
    }

    VP_API_EXIT(VpLineCtxType, pLineCtx, "FillTestBuf", status);
    return status;

} /* VpFillTestBuf() */
#endif /* VP_CC_FILL_TEST_BUF */

#ifdef VP_CC_READ_TEST_BUF
/**
 * VpReadTestBuf()
 *
 *
 * Preconditions:
 *  Device/Line context should be created and initialized. For applicable
 * devices bootload should be performed before calling the function.
 *
 * Postconditions:
 *  Loads test data from the device into the given buffer.
 */
VpStatusType
VpReadTestBuf(
    VpLineCtxType *pLineCtx,
    uint16 length,
    VpVectorPtrType pData)
{
    VpStatusType status = VP_STATUS_SUCCESS;

    VP_API_ENTER(VpLineCtxType, pLineCtx, "ReadTestBuf");
    /* Basic argument checking */
    if (pLineCtx == VP_NULL) {
        status = VP_STATUS_INVALID_ARG;
    } else {
        VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
        status = VP_CALL_DEV_FUNC(ReadTestBuf, (pLineCtx, length, pData));
    }
    VP_API_EXIT(VpLineCtxType, pLineCtx, "ReadTestBuf", status);
    return status;

} /* VpReadTestBuf() */
#endif /* VP_CC_READ_TEST_BUF */

