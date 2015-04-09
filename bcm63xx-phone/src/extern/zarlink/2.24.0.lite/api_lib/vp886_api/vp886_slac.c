/** \file vp886_hal.c
 * vp886_hal.c
 *
 * This file contains HAL wrapper functions specific to Vp886.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11513 $
 * $LastChangedDate: 2014-08-11 09:52:30 -0500 (Mon, 11 Aug 2014) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_886_SERIES)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_api.h"
#include "vp_api_int.h"
#include "vp886_api.h"
#include "vp886_api_int.h"
#include "vp_hal.h"
#include "sys_service.h"
#include "vp_debug.h"

static void
Vp886SlacBufAddWrite(
    VpDevCtxType *pDevCtx,
    uint8 ecVal,
    uint8 cmd,
    uint8 dataLen,
    const uint8 *pDataBuf);


/** Vp886SlacBufStart()
  Resets the MPI write buffer and sets the flag to enable buffering through
  Vp886SlacRegWrite().
*/
bool
Vp886SlacBufStart(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj;

    pDevObj = pDevCtx->pDevObj;

    /* Check for mismatched buffering start/end commands. */
    if (pDevObj->slacBufData.buffering == TRUE) {
        VP_WARNING(VpDevCtxType, pDevCtx, ("Vp886SlacBufStart(): Buffer already started"));
        return FALSE;
    }

    pDevObj->slacBufData.wrtLen = 0;
    pDevObj->slacBufData.buffering = TRUE;

    return TRUE;
}


/** Vp886SlacBufSend()
  Sends the contents of the MPI write buffer and sets the flag to disable
  further buffering.
*/
bool
Vp886SlacBufSend(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpSlacBufDataType *pSlacBufData;

    pSlacBufData = &pDevObj->slacBufData;

    /* Check for mismatched buffering start/end commands. */
    if (pSlacBufData->buffering == FALSE) {
        VP_WARNING(VpDevCtxType, pDevCtx, ("Vp886SlacBufSend(): Buffer not started"));
        return FALSE;
    }
    
    /* If buffer is not empty, write the contents to the device. */
    if (pSlacBufData->wrtLen != 0) {
        Vp886SlacBufFlush(pDevCtx);
    }

    pSlacBufData->buffering = FALSE;

    return TRUE;
}


/** Vp886SlacRegWrite()
  If buffering has been started by Vp886SlacBufStart(), this will add a write
  command+data to the buffer.

  If buffering is not active, this will immediately send the command+data to
  the device.
*/
bool
Vp886SlacRegWrite(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint8 cmd,
    uint8 dataLen,
    const uint8 *pDataBuf)
{
    Vp886DeviceObjectType *pDevObj;
    VpDeviceIdType deviceId;
    Vp886LineObjectType *pLineObj;
    VpSlacBufDataType *pSlacBufData;
    uint8 ecVal;
    
    /* Read commands should not be allowed */
    if (cmd & 0x01) {
        VP_WARNING(VpDevCtxType, pDevCtx, ("Invalid read command in Vp886SlacRegWrite (0x%02X)", cmd));
        return FALSE;
    }

    if (pLineCtx != NULL) {
        pLineObj = pLineCtx->pLineObj;
        pDevCtx = pLineCtx->pDevCtx;
        pDevObj = pDevCtx->pDevObj;
        ecVal = pLineObj->ecVal;
    } else {
        pDevObj = pDevCtx->pDevObj;
        ecVal = pDevObj->ecVal;
    }
    
    pSlacBufData = &pDevObj->slacBufData;

    if (pSlacBufData->buffering == 0) {
        /* Unbuffered write */
        deviceId = pDevObj->deviceId;
        VpMpiCmdWrapper(deviceId, ecVal, cmd, dataLen, (uint8 *)pDataBuf);

        /* Add to the traffic count. Data length + command + EC command + EC data */
        pDevObj->trafficBytes += dataLen + 3;

    } else {
        /* Buffered write */
        Vp886SlacBufAddWrite(pDevCtx, ecVal, cmd, dataLen, pDataBuf);
    }
    
    return TRUE;
}

/** Vp886SlacRegRead()
  Reads a register from the device.

  If buffering is active and the buffer is non-empty, this function will send
  the current contents of the buffer via Vp886SlacBufFlush() before performing
  the read to avoid write->read problems.  This allows the API code using these
  functions to be written without worrying about whether buffering is active or
  not.

  The buffer flush can be skipped by setting the pDevObj->dontFlushSlacBufOnRead
  flag to TRUE before calling this function, in cases where we are sure that
  no buffered writes would possibly affect the results of the read, such as
  when reading read-only registers.
*/
bool
Vp886SlacRegRead(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    uint8 cmd,
    uint8 dataLen,
    uint8 *pDataBuf)
{
    Vp886DeviceObjectType *pDevObj;
    VpDeviceIdType deviceId;
    Vp886LineObjectType *pLineObj;
    VpSlacBufDataType *pSlacBufData;
    uint8 ecVal;
    
    /* Write commands should not be allowed */
    if (!(cmd & 0x01)) {
        VP_WARNING(VpDevCtxType, pDevCtx, ("Invalid write command in Vp886SlacRegRead (0x%02X)", cmd));
        return FALSE;
    }

    if (pLineCtx != NULL) {
        pLineObj = pLineCtx->pLineObj;
        pDevCtx = pLineCtx->pDevCtx;
        pDevObj = pDevCtx->pDevObj;
        ecVal = pLineObj->ecVal;
    } else {
        pDevObj = pDevCtx->pDevObj;
        ecVal = pDevObj->ecVal;
    }
    deviceId = pDevObj->deviceId;
    
    pSlacBufData = &pDevObj->slacBufData;

    /* Write the contents of the write buffer to avoid errors in write->read
       operations. */
    if (pSlacBufData->buffering && pSlacBufData->wrtLen > 0 &&
        !pDevObj->dontFlushSlacBufOnRead)
    {
        Vp886SlacBufFlush(pDevCtx);
    }
    pDevObj->dontFlushSlacBufOnRead = FALSE;

    VpMpiCmdWrapper(deviceId, ecVal, cmd, dataLen, pDataBuf);

    /* Add to the traffic count. Data length + command + EC command + EC data */
    pDevObj->trafficBytes += dataLen + 3;

    return TRUE;
}


/** Vp886SlacBufFlush()
  Sends the current contents of the MPI write buffer, but does not clear the
  buffering flag, so that buffering will continue.  This is used before
  performing a read operation and when the buffer is too full to add a new
  write.
*/
void
Vp886SlacBufFlush(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpDeviceIdType deviceId;
    VpSlacBufDataType *pSlacBufData;
    uint8 ecVal;
    uint8 cmd;
    uint8 writeLen;
    uint8 *pWriteBuf;

    pSlacBufData = &pDevObj->slacBufData;

    /* Check for mismatched buffering start/end commands. */
    if (pSlacBufData->buffering == FALSE) {
        VP_WARNING(VpDevCtxType, pDevCtx, ("Vp886SlacBufFlush(): Buffer not started"));
        return;
    }
    
    /* If buffer is empty, do nothing. */
    if (pSlacBufData->wrtLen == 0) {
        return;
    }

    ecVal = pSlacBufData->firstEc;
    cmd = pSlacBufData->wrtBuf[0];
    writeLen = pSlacBufData->wrtLen - 1;
    pWriteBuf = &pSlacBufData->wrtBuf[1];
    deviceId = pDevObj->deviceId;

    VpMpiCmdWrapper(deviceId, ecVal, cmd, writeLen, pWriteBuf);

    /* Add to the traffic count. Data length + command + EC command + EC data */
    pDevObj->trafficBytes += writeLen + 3;

    pSlacBufData->wrtLen = 0;

    return;
}


/** Vp886SlacBufAddWrite()
  Adds a new write command+data to the MPI write buffer.

  If the new length would exceed the buffer size, the buffer is flushed first
  via Vp886SlacBufFlush().

  This function will insert EC commands into the queue only when the EC value
  is changed from the previous write.  The first command will NOT add an EC
  command to the queue, because that will be handled by VpMpiCmd() when the
  buffer is eventually sent.
*/
static void
Vp886SlacBufAddWrite(
    VpDevCtxType *pDevCtx,
    uint8 ecVal,
    uint8 cmd,
    uint8 dataLen,
    const uint8 *pDataBuf)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpSlacBufDataType *pSlacBufData = &pDevObj->slacBufData;
    uint8 addLen;
    uint16 newLen;
    uint8 index;
    uint8 *pBuffer;
    bool ecChanged = FALSE;

    addLen = 1 + dataLen;

    if (pSlacBufData->wrtLen == 0) {
        /* First command into the buffer.  Set the first EC value, but
           don't add an EC command.  VpMpiCmd will add the first EC command
           for us. */
        pSlacBufData->firstEc = ecVal;
    } else if (pSlacBufData->currentEc != ecVal) {
        /* EC changed.  Add EC command to the buffer */
        ecChanged = TRUE;
        addLen += 2;
    }

    newLen = (uint16)pSlacBufData->wrtLen + addLen;

    if (newLen > 0xFF || newLen > VP_SLAC_MAX_BUF_WRT) {
        /* Write the existing contents and start a fresh buffer */
        Vp886SlacBufFlush(pDevCtx);

        /* Set the first EC value, and undo the +2 and ecChanged actions if
           performed earlier, since we will NOT be adding an EC command to
           the buffer now. */
        pSlacBufData->firstEc = ecVal;
        if (pSlacBufData->currentEc != ecVal) {
            addLen -= 2;
            ecChanged = FALSE;
        }
        
        newLen = addLen;
    }

    pBuffer = pSlacBufData->wrtBuf;
    index = pSlacBufData->wrtLen;

    if (ecChanged) {
        pBuffer[index++] = VP886_R_EC_WRT;
        pBuffer[index++] = ecVal;
    }

    pBuffer[index++] = cmd;
    VpMemCpy(&pBuffer[index], pDataBuf, (uint16)dataLen);

    pSlacBufData->wrtLen = (uint8)newLen;
    pSlacBufData->currentEc = ecVal;
    
    /* In the case of explicit EC_WRT commands, update the currentEc value */
    if (cmd == VP886_R_EC_WRT) {
        pSlacBufData->currentEc = pDataBuf[0];
    }

    return;
}

#endif /* defined (VP_CC_886_SERIES) */

