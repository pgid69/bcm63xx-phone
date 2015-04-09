/** \file vp886_query.c
 * vp886_query.c
 *
 *  This file contains the query functions used in the Vp886 series device API.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11625 $
 * $LastChangedDate: 2014-10-22 23:01:52 -0500 (Wed, 22 Oct 2014) $
 */

#include "vp_api_cfg.h"

#if defined (VP_CC_886_SERIES)

/* Project Includes */
#include "vp_api_types.h"
#include "sys_service.h"
#include "vp_hal.h"
#include "vp_api_int.h"
#include "vp886_api.h"
#include "vp886_api_int.h"

#ifdef VP886_INCLUDE_TESTLINE_CODE
#include "vp_api_test.h"
#endif


/** Vp886GetOption()
  Implements VpGetOption() to return the value of the requested option with a
  VP_LINE_EVID_RD_OPTION event.

  This function only puts the event into the queue.  The actual option value
  is retrieved when VpGetResults() is called on the event.

  See the VP-API-II Reference Guide for more details on VpGetOption().
*/
VpStatusType
Vp886GetOption(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    uint16 handle)
{
    Vp886DeviceObjectType *pDevObj;
    Vp886LineObjectType *pLineObj = VP_NULL;
    VpStatusType status = VP_STATUS_SUCCESS;
    uint8 channelId;

    Vp886EnterCritical(pDevCtx, pLineCtx, "Vp886GetOption");

    if (pLineCtx != VP_NULL) {
        pDevCtx = pLineCtx->pDevCtx;
        pLineObj = pLineCtx->pLineObj;
        channelId = pLineObj->channelId;
    } else {
        channelId = VP886_DEV_EVENT;
    }
    pDevObj = pDevCtx->pDevObj;

    switch (option) {
        case VP_OPTION_ID_PULSE_MODE:
        case VP_OPTION_ID_LINE_STATE:
        case VP_OPTION_ID_RING_CNTRL:
        case VP_OPTION_ID_SWITCHER_CTRL:
        case VP_OPTION_ID_DCFEED_PARAMS:
        case VP_OPTION_ID_RINGING_PARAMS:
        case VP_OPTION_ID_GND_FLT_PROTECTION:
        case VP_OPTION_ID_RINGTRIP_CONFIRM:
            /* Do not allow FXS specific options on an FXO line */
            if (pLineCtx == VP_NULL) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Line context required for option ID %d", option));
                status = VP_STATUS_INVALID_ARG;
            }
            if (!pLineObj->isFxs) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("FXS option ID %d not supported on FXO line", option));
                status = VP_STATUS_INVALID_ARG;
            }
            break;
        case VP_OPTION_ID_LINE_IO_CFG:
            if (pLineCtx == VP_NULL) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Line context required for option ID %d", option));
                status = VP_STATUS_INVALID_ARG;
                break;
            }
            if (pDevObj->ioCapability == VP886_IO_CAPABILITY_NONE) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886GetOption(option %d) - Device does not support I/O lines", option));
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
            }
            break;
        case VP_OPTION_ID_TIMESLOT:
        case VP_OPTION_ID_CODEC:
        case VP_OPTION_ID_LOOPBACK:
        case VP_OPTION_ID_EVENT_MASK:
        case VP_OPTION_ID_PCM_TXRX_CNTRL:
        case VP_OPTION_ID_ABS_GAIN:
        case VP_OPTION_ID_DTMF_MODE:
        case VP_OPTION_ID_HIGHPASS_FILTER:
        case VP_OPTION_ID_DTMF_PARAMS:
            if (pLineCtx == VP_NULL) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Line context required for option ID %d", option));
                status = VP_STATUS_INVALID_ARG;
                break;
            }
            break;
        case VP_DEVICE_OPTION_ID_DEVICE_IO:
            if (pDevObj->ioCapability == VP886_IO_CAPABILITY_NONE) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetOption(option %d) - Device does not support I/O lines", option));
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
            }
            break;
        case VP_DEVICE_OPTION_ID_FSYNC_RATE:
            if (!VP886_IS_SF1(pDevObj)) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
            }
            break;
        case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
        case VP_DEVICE_OPTION_ID_PULSE:
        case VP_DEVICE_OPTION_ID_PULSE2:
#ifdef VP886_INCLUDE_ADAPTIVE_RINGING
        case VP_DEVICE_OPTION_ID_ADAPTIVE_RINGING:
#endif
        case VP_DEVICE_OPTION_ID_RING_PHASE_SYNC:
            break;
        default:
            if (pLineCtx != VP_NULL) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Unsupported option ID %d", option));
            } else {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Unsupported option ID %d", option));
            }
            status = VP_STATUS_OPTION_NOT_SUPPORTED;
    }
    
    if (status == VP_STATUS_SUCCESS) {
        Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_RESPONSE, VP_LINE_EVID_RD_OPTION, option, handle, TRUE);
    }

    Vp886ExitCritical(pDevCtx, pLineCtx, "Vp886GetOption");
    return status;
}


/** Vp886GetOptionImmediate()
  Implements VpGetOptionImmediate() to return the value of the requested option.

  See the VP-API-II Reference Guide for more details on VpGetOptionImmediate().
*/
VpStatusType
Vp886GetOptionImmediate(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx,
    VpOptionIdType option,
    void *pResults)
{
    Vp886DeviceObjectType *pDevObj;
    Vp886LineObjectType *pLineObj = VP_NULL;
    VpStatusType status = VP_STATUS_SUCCESS;

    Vp886EnterCritical(pDevCtx, pLineCtx, "Vp886GetOptionImmediate");

    if (pLineCtx != VP_NULL) {
        pDevCtx = pLineCtx->pDevCtx;
        pLineObj = pLineCtx->pLineObj;
    }
    pDevObj = pDevCtx->pDevObj;

    switch (option) {
        case VP_OPTION_ID_PULSE_MODE:
        case VP_OPTION_ID_LINE_STATE:
        case VP_OPTION_ID_RING_CNTRL:
        case VP_OPTION_ID_SWITCHER_CTRL:
        case VP_OPTION_ID_DCFEED_PARAMS:
        case VP_OPTION_ID_RINGING_PARAMS:
        case VP_OPTION_ID_GND_FLT_PROTECTION:
        case VP_OPTION_ID_RINGTRIP_CONFIRM:
            /* Do not allow FXS specific options on an FXO line */
            if (pLineCtx == VP_NULL) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Line context required for option ID %d", option));
                status = VP_STATUS_INVALID_ARG;
            }
            if (!pLineObj->isFxs) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("FXS option ID %d not supported on FXO line", option));
                status = VP_STATUS_INVALID_ARG;
            }
            status = Vp886GetResultsRdOptionLine(pLineCtx, option, pResults);
            break;
        case VP_OPTION_ID_LINE_IO_CFG:
            if (pLineCtx == VP_NULL) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Line context required for option ID %d", option));
                status = VP_STATUS_INVALID_ARG;
                break;
            }
            if (pDevObj->ioCapability == VP886_IO_CAPABILITY_NONE) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886GetOption(option %d) - Device does not support I/O lines", option));
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
            }
            status = Vp886GetResultsRdOptionLine(pLineCtx, option, pResults);
            break;
        case VP_OPTION_ID_TIMESLOT:
        case VP_OPTION_ID_CODEC:
        case VP_OPTION_ID_LOOPBACK:
        case VP_OPTION_ID_EVENT_MASK:
        case VP_OPTION_ID_PCM_TXRX_CNTRL:
        case VP_OPTION_ID_ABS_GAIN:
        case VP_OPTION_ID_DTMF_MODE:
        case VP_OPTION_ID_HIGHPASS_FILTER:
        case VP_OPTION_ID_DTMF_PARAMS:
            if (pLineCtx == VP_NULL) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Line context required for option ID %d", option));
                status = VP_STATUS_INVALID_ARG;
                break;
            }
            status = Vp886GetResultsRdOptionLine(pLineCtx, option, pResults);
            break;
        case VP_DEVICE_OPTION_ID_DEVICE_IO:
            if (pDevObj->ioCapability == VP886_IO_CAPABILITY_NONE) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetOption(option %d) - Device does not support I/O lines", option));
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
            }
            break;
        case VP_DEVICE_OPTION_ID_FSYNC_RATE:
            if (!VP886_IS_SF1(pDevObj)) {
                status = VP_STATUS_OPTION_NOT_SUPPORTED;
            }
            break;
        case VP_DEVICE_OPTION_ID_CRITICAL_FLT:
        case VP_DEVICE_OPTION_ID_PULSE:
        case VP_DEVICE_OPTION_ID_PULSE2:
#ifdef VP886_INCLUDE_ADAPTIVE_RINGING
        case VP_DEVICE_OPTION_ID_ADAPTIVE_RINGING:
#endif
        case VP_DEVICE_OPTION_ID_RING_PHASE_SYNC:
            status = Vp886GetResultsRdOptionDev(pDevCtx, option, pResults);
            break;
        default:
            if (pLineCtx != VP_NULL) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Unsupported option ID %d", option));
            } else {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Unsupported option ID %d", option));
            }
            status = VP_STATUS_OPTION_NOT_SUPPORTED;
    }
    
    Vp886ExitCritical(pDevCtx, pLineCtx, "Vp886GetOptionImmediate");
    return status;
}


/** Vp886GetDeviceStatus()
  Implements VpGetDeviceStatus() to return the boolean status of the requested
  input type for all lines on a device.

  The bits of the uint32 pointed to by pDeviceStatus are set (on a per line
  basis) to either '1' if the status if TRUE on the given line, or '0' if the
  status is FALSE on the given line for the status being requested.

  See the VP-API-II Reference Guide for more details on VpGetDeviceStatus().
*/
VpStatusType
Vp886GetDeviceStatus(
    VpDevCtxType *pDevCtx,
    VpInputType input,
    uint32 *pDeviceStatus)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId;
    VpLineCtxType *pLineCtx;
    bool status = FALSE;

    Vp886EnterCritical(pDevCtx, VP_NULL, "Vp886GetDeviceStatus");

    *pDeviceStatus = 0;

    for (channelId = 0; channelId < pDevObj->staticInfo.maxChannels; channelId++) {
        pLineCtx = pDevCtx->pLineCtx[channelId];

        if(pLineCtx != VP_NULL) {
            VpCSLACGetLineStatus(pLineCtx, input, &status);
        } else {
            status = FALSE;
        }
        *pDeviceStatus |= (((status == TRUE) ? 1 : 0) << channelId);
    }

    Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886GetDeviceStatus");
    return VP_STATUS_SUCCESS;
}


/** Vp886GetLoopCond()
  Implements VpGetLoopCond() to measure electric parameters on the line:
        rloop, ilg, imt, vsab, vbat1, vbat2, vbat3

  A SADC interrupt is generated, which results in a call to
  Vp886GetLoopCondResults().

  See the VP-API-II Reference Guide for more details on VpGetLoopCond().
*/
VpStatusType
Vp886GetLoopCond(
    VpLineCtxType *pLineCtx,
    uint16 handle)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 convConf[VP886_R_SADC_LEN];
    uint8 vadcConvConf[VP886_R_VADC_LEN];
    VpStatusType status;

    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886GetLoopCond");

#ifdef VP886_INCLUDE_ADAPTIVE_RINGING
    if ((pDevObj->options.adaptiveRinging.power != VP_ADAPTIVE_RINGING_DISABLED) && 
        ((pLineObj->lineState.usrCurrent == VP_LINE_RINGING) ||
        (pLineObj->lineState.usrCurrent == VP_LINE_RINGING_POLREV))) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886GetLoopCond");
        return VP_STATUS_DEVICE_BUSY;
    }
#endif

    if (!Vp886ReadyStatus(VP_NULL, pLineCtx, &status)) {
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886GetLoopCond");
        return status;
    }

    if (pDevObj->getResultsRequired) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("Vp886GetLoopCond() - Waiting to clear previous read"));
        Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886GetLoopCond");
        return VP_STATUS_DEVICE_BUSY;
    }

    pLineObj->busyFlags |= VP886_LINE_GET_LOOP_COND;

#ifdef VP886_INCLUDE_DTMF_DETECT
    /* Shut down DTMF detection if it is running.  This must come after setting
       the VP886_LINE_GET_LOOP_COND busy-flag. */
    Vp886DtmfManage(pLineCtx);
#endif

    /* Save the handle for the call-back */
    pLineObj->getLoopCondHandle = handle;

    /* Save icr settings if in disconnect */
    if ((pLineObj->registers.sysState[0] & VP886_R_STATE_SS) == VP886_R_STATE_SS_DISCONNECT) {
        VpMemCpy(pLineObj->getLoopIcr2, pLineObj->registers.icr2, VP886_R_ICR2_LEN);
        VpMemCpy(pLineObj->getLoopIcr3, pLineObj->registers.icr3, VP886_R_ICR3_LEN);
        VpMemCpy(pLineObj->getLoopIcr4, pLineObj->registers.icr4, VP886_R_ICR4_LEN);
        
        /* Ensure the tip and ring sense signals are on */
        pLineObj->registers.icr2[0] = 0x0C;
        pLineObj->registers.icr2[1] = 0x0C;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, pLineObj->registers.icr2);

        /* Ensure the line control circuits are on */
        pLineObj->registers.icr3[0] |= 0x21;
        pLineObj->registers.icr3[1] |= 0x21;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN, pLineObj->registers.icr3);

        /* Connect External Ctrl Network to AC Feedback Loop */
        pLineObj->registers.icr4[0] |= 0x01;
        pLineObj->registers.icr4[1] |= 0x01;
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR4_WRT, VP886_R_ICR4_LEN, pLineObj->registers.icr4);
    }

    /* Backup the current VADC settings */
    VpSlacRegRead(NULL, pLineCtx, VP886_R_VADC_RD, VP886_R_VADC_LEN, pLineObj->vadcConvConf);

    /* Signals can't be shared between the VADC and the SADC, disable the VADC if required */
    if ((pLineObj->vadcConvConf[1] == VP886_R_VADC_SEL_LONG_CUR) ||
        (pLineObj->vadcConvConf[1] == VP886_R_VADC_SEL_METALLIC_CUR) ||
        (pLineObj->vadcConvConf[1] == VP886_R_VADC_SEL_TIP_RING_DC_V) ||
        (pLineObj->vadcConvConf[1] == VP886_R_VADC_SEL_SWY) ||
        (pLineObj->vadcConvConf[1] == VP886_R_VADC_SEL_SWZ)) {

        /* Take control of the VADC and disable it */
        vadcConvConf[0] = pLineObj->vadcConvConf[0] | VP886_R_VADC_SM_OVERRIDE;
        vadcConvConf[1] = VP886_R_VADC_SEL_ADC_OFFSET;
        VpMemCpy(&vadcConvConf[2], &pLineObj->vadcConvConf[2], VP886_R_VADC_LEN - 2);
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_VADC_WRT, VP886_R_VADC_LEN, vadcConvConf);
    } else {
        /* Marker value meaning no need to disable the VADC */
        pLineObj->vadcConvConf[1] = 0xFF;
    }

    /* Enable the SADC in math mode with a sampling rate of 2kHz */
    convConf[0] = VP886_R_SADC_ENABLE | VP886_R_SADC_MATH | VP886_R_SADC_DRATE_SINGLE_2KHZ |
        VP886_R_SADC_GROUP_MODE | VP886_R_SADC_TX_INTERRUPT;

    /* Program the SADC routes */
    convConf[1] = VP886_R_SADC_SEL_LONG_CUR;
    convConf[2] = VP886_R_SADC_SEL_METALLIC_CUR;
    convConf[3] = VP886_R_SADC_SEL_TIP_RING_DC_V;
    convConf[4] = VP886_R_SADC_SEL_SWY;
    convConf[5] = VP886_R_SADC_SEL_SWZ;

    /* Skip 1 and collect 1 sample */
    convConf[6] = 0x00;
    convConf[7] = 1;
    convConf[8] = 0x00;
    convConf[9] = 1;

    /* Send down the Supervision Converter Configuration */
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_SADC_WRT, VP886_R_SADC_LEN, convConf);

    /* Prevent any other tests sharing the result structure to run */
    pDevObj->getResultsRequired = TRUE;

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886GetLoopCond");
    return VP_STATUS_SUCCESS;
}


/** Vp886GetLoopCondResults()
  Processes the data from an SADC measurement that was started by
  Vp886GetLoopCond().  This will generate the VP_LINE_EVID_RD_LOOP event.
*/
VpStatusType
Vp886GetLoopCondResults(
    VpLineCtxType *pLineCtx)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp886CmnCalDeviceDataType *pCalData = &pDevObj->calData[pLineObj->channelId].cmn;
    uint8 convConf[VP886_R_SADC_LEN];
    uint8 dataBuff[VP886_R_B6_LEN];
    int16 sample;
    int32 sadcGain = (int32)(pCalData->sadc.gain);
    int32 swyGain = (int32)(pCalData->swySense.gain);
    int32 swzGain = (int32)(pCalData->swzSense.gain);
    int16 sadcOffset = pCalData->sadc.offset;
    int16 swyOffset = pCalData->swySense.offset;
    int16 swzOffset = pCalData->swzSense.offset;
    int16 vabOffset;
    int32 vabGain, vab, imt;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886GetLoopCondResults+"));

    /* Read all 5 data buffers */
    VpSlacRegRead(NULL, pLineCtx, VP886_R_B6_RD, VP886_R_B6_LEN, dataBuff);

    /* Read the longitudinal current */
    sample = (int16)((dataBuff[0] << 8) & 0xFF00);
    sample |= (int16)(dataBuff[1] & 0x00FF);
    /* Apply the SADC offset and gain */
    sample += sadcOffset;
    /* Full scale of 59.5mA */
    pDevObj->basicResults.getLoopCond.ilg = (int16)(((int32)sample * sadcGain) / 1000L);

    /* Read the metallic current */
    sample = (int16)((dataBuff[2] << 8) & 0xFF00);
    sample |= (int16)(dataBuff[3] & 0x00FF);

    /* If low ILR is enabled, imt should be halved */
    if ((pLineObj->vadcConvConf[1] != 0xFF) && pLineObj->lowIlr) {
        sample /= 2;
    }

    /* Apply the SADC offset and gain */
    sample += sadcOffset;
    imt = ((int32)sample * sadcGain) / 1000L;
    pDevObj->basicResults.getLoopCond.imt = (int16)imt;

    /* Read the VAB voltage */
    sample = (int16)((dataBuff[4] << 8) & 0xFF00);
    sample |= (int16)(dataBuff[5] & 0x00FF);
    /* Apply the right calibration according to the polarity */
    if (pLineObj->vadcConvConf[1] != 0xFF) {
        /* Ringing normal polarity */
        vabGain = (int32)(pCalData->vabSenseRinging.gain);
        vabOffset = pCalData->vabSenseRinging.offset;
    } else {
        if (sample > 0) {
            /* Normal polarity */
            vabGain = (int32)(pCalData->vabSenseNormal.gain);
            vabOffset = pCalData->vabSenseNormal.offset;
        } else {
            /* Reverse polarity */
            vabGain = (int32)(pCalData->vabSenseReverse.gain);
            vabOffset = pCalData->vabSenseReverse.offset;
        }
    }
    /* Apply the SADC offset and gain */
    sample += vabOffset;
    vab = ((int32)sample * vabGain) / 1000L;
    pDevObj->basicResults.getLoopCond.vsab = (int16)vab;

    /* Read the SWY voltage */
    sample = (int16)((dataBuff[6] << 8) & 0xFF00);
    sample |= (int16)(dataBuff[7] & 0x00FF);

    /* Apply the SADC offset and gain */
    sample += swyOffset;
    pDevObj->basicResults.getLoopCond.vbat1 = (int16)(((int32)sample * swyGain) / 1000L);

    /* Read the SWZ voltage */
    sample = (int16)((dataBuff[8] << 8) & 0xFF00);
    sample |= (int16)(dataBuff[9] & 0x00FF);
    /* Apply the SADC offset and gain */
    sample += swzOffset;
    pDevObj->basicResults.getLoopCond.vbat2 = (int16)(((int32)sample * swzGain) / 1000L);

    /* Compute the loop resistor */
    if (imt != 0) {
        /* Use a full scale of 11.67kR (240 / 0.0595 / 11670 * 32768 = 11326) */
        pDevObj->basicResults.getLoopCond.rloop = (int16)(vab * 11326L / imt);
    } else {
        pDevObj->basicResults.getLoopCond.rloop = VP_INT16_MAX;
    }

    pDevObj->basicResults.getLoopCond.vbat3 = 0;
    pDevObj->basicResults.getLoopCond.mspl = 0;
    pDevObj->basicResults.getLoopCond.selectedBat = VP_BATTERY_UNDEFINED;
    pDevObj->basicResults.getLoopCond.dcFeedReg = VP_DF_UNDEFINED;

    /* Set the Supervision Converter Configuration to "no connect" and disable interrupts */
    VpSlacRegRead(NULL, pLineCtx, VP886_R_SADC_RD, VP886_R_SADC_LEN, convConf);
    convConf[0] = VP886_R_SADC_ENABLE | VP886_R_SADC_MATH | VP886_R_SADC_DRATE_SINGLE_2KHZ;
    convConf[1] = VP886_R_SADC_SEL_ADC_OFFSET;
    convConf[2] = VP886_R_SADC_SEL_ADC_OFFSET;
    convConf[3] = VP886_R_SADC_SEL_ADC_OFFSET;
    convConf[4] = VP886_R_SADC_SEL_ADC_OFFSET;
    convConf[5] = VP886_R_SADC_SEL_ADC_OFFSET;
    convConf[6] = 0;
    convConf[7] = 1;
    convConf[8] = 0;
    convConf[9] = 1;
    VpSlacRegWrite(NULL, pLineCtx, VP886_R_SADC_WRT, VP886_R_SADC_LEN, convConf);

    /* Restore the VADC if needed */
    if (pLineObj->vadcConvConf[1] != 0xFF) {
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_VADC_WRT, VP886_R_VADC_LEN, pLineObj->vadcConvConf);
    }
    
    /* Restore icr settings if in disconnect */
    if ((pLineObj->registers.sysState[0] & VP886_R_STATE_SS) == VP886_R_STATE_SS_DISCONNECT) {
        VpMemCpy(pLineObj->registers.icr2, pLineObj->getLoopIcr2, VP886_R_ICR2_LEN);
        VpMemCpy(pLineObj->registers.icr3, pLineObj->getLoopIcr3, VP886_R_ICR3_LEN);
        VpMemCpy(pLineObj->registers.icr4, pLineObj->getLoopIcr4, VP886_R_ICR4_LEN);

        VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR2_WRT, VP886_R_ICR2_LEN, pLineObj->registers.icr2);
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR3_WRT, VP886_R_ICR3_LEN, pLineObj->registers.icr3);
        VpSlacRegWrite(NULL, pLineCtx, VP886_R_ICR4_WRT, VP886_R_ICR4_LEN, pLineObj->registers.icr4);
    }

    /* Measurements available, generate the Get Loop complete event VP_LINE_EVID_RD_LOOP */
    Vp886PushEvent(pDevCtx, pLineObj->channelId, VP_EVCAT_RESPONSE, VP_LINE_EVID_RD_LOOP, 0,
        pLineObj->getLoopCondHandle, TRUE);

    pLineObj->busyFlags &= ~VP886_LINE_GET_LOOP_COND;

#ifdef VP886_INCLUDE_DTMF_DETECT
    /* Re-enable DTMF detection if necessary.  Must be done after clearing
       the VP886_LINE_GET_LOOP_COND busy-flag. */
    Vp886DtmfManage(pLineCtx);
#endif

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886GetLoopCondResults-"));

    return VP_STATUS_SUCCESS;
}


/** Vp886Query()
  Implements VpQuery() to return various non-boolean information about a line
  or device based on the queryId type.  The result is returned through a
  VP_LINE_EVID_QUERY_CMP event.

  This function only puts the event into the queue.  The actual result value
  is retrieved when VpGetResults() is called on the event.

  See the VP-API-II Reference Guide for more details on VpQuery().
*/
VpStatusType
Vp886Query(
    VpLineCtxType *pLineCtx,
    VpQueryIdType queryId,
    uint16 handle)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;

    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886Query");

    switch (queryId) {
        case VP_QUERY_ID_DEV_TRAFFIC:
            break;
        case VP_QUERY_ID_LINE_CAL_COEFF:
            if (!pDevObj->calData[channelId].valid) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886Query - Line has not been calibrated"));
                Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886Query");
                return VP_STATUS_LINE_NOT_CONFIG;
            }
            break;
        case VP_QUERY_ID_TIMESTAMP:
            break;
        case VP_QUERY_ID_LINE_TOPOLOGY:
            break;
        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886Query - Invalid queryId %d", queryId));
            Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886Query");
            return VP_STATUS_INVALID_ARG;
    }
    
    Vp886PushEvent(pDevCtx, channelId, VP_EVCAT_RESPONSE, VP_LINE_EVID_QUERY_CMP, queryId, handle, TRUE);

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886Query");
    return VP_STATUS_SUCCESS;
}


/** Vp886Query()
  Implements VpQueryImmediate() to return various non-boolean information about
  a line or device based on the queryId type.

  See the VP-API-II Reference Guide for more details on VpQueryImmediate().
*/
VpStatusType
Vp886QueryImmediate(
    VpLineCtxType *pLineCtx,
    VpQueryIdType queryId,
    void *pResults)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId = pLineObj->channelId;
    VpStatusType status;

    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886QueryImmediate");

    switch (queryId) {
        case VP_QUERY_ID_DEV_TRAFFIC:
            break;
        case VP_QUERY_ID_LINE_CAL_COEFF:
            if (!pDevObj->calData[channelId].valid) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886QueryImmediate - Line has not been calibrated"));
                Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886QueryImmediate");
                return VP_STATUS_LINE_NOT_CONFIG;
            }
            break;
        case VP_QUERY_ID_TIMESTAMP:
            break;
        case VP_QUERY_ID_LINE_TOPOLOGY:
            break;
        default:
            VP_ERROR(VpLineCtxType, pLineCtx, ("Vp886Query - Invalid queryId %d", queryId));
            Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886QueryImmediate");
            return VP_STATUS_INVALID_ARG;
    }

    status = Vp886GetResultsQuery(pLineCtx, queryId, (VpQueryResultsType *)pResults);

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886QueryImmediate");
    return status;
}


#ifdef VP886_INCLUDE_TESTLINE_CODE
/** Vp886GetRelayState()
  Implements VpGetRelayState() to return the current relay state of a line.
  This is only intended to be used by the VeriVoice line test software.
*/
VpStatusType
Vp886GetRelayState(
    VpLineCtxType *pLineCtx,
    VpRelayControlType *pRstate)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp886EnterCritical(VP_NULL, pLineCtx, "Vp886GetRelayState");

    *pRstate = pLineObj->relayState;

    Vp886ExitCritical(VP_NULL, pLineCtx, "Vp886GetRelayState");
    return VP_STATUS_SUCCESS;
}
#endif /* VP886_INCLUDE_TESTLINE_CODE */


#if (VP_CC_DEBUG_SELECT & VP_DBG_INFO)
/** Vp886RegisterDump()
  Implements VpRegisterDump() to print the contents of all device and line
  specific registers (for debug purposes).

  See the VP-API-II Reference Guide for more details on VpRegisterDump().
*/
VpStatusType
Vp886RegisterDump(
    VpDevCtxType *pDevCtx)
{
    VpStatusType status;

    Vp886EnterCritical(pDevCtx, VP_NULL, "Vp886RegisterDump");

    status = Vp886RegisterDumpInt(pDevCtx);

    Vp886ExitCritical(pDevCtx, VP_NULL, "Vp886RegisterDump");
    return status;
}


/** Vp886RegisterDumpInt()
  Internal implementation that can be called to bypass the enter/exit critical
  section.
*/
VpStatusType
Vp886RegisterDumpInt(
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint8 channelId, registerIndex, registerNumber;

    /* Sufficient size to hold a single MPI Read is all that is needed. */
    uint8 registerBuffer[25];

    /* Define buffers containing MPI read commands with lengths */
    #define VP886_DEVICE_REGISTER_COUNT    13
    uint8 deviceRegs[VP886_DEVICE_REGISTER_COUNT][2] = {
        {VP886_R_RCNPCN_RD, VP886_R_RCNPCN_LEN},
        {VP886_R_INTREV_RD, VP886_R_INTREV_LEN},
        {VP886_R_DEVCFG_RD, VP886_R_DEVCFG_LEN},
        {VP886_R_DEVMODE_RD, VP886_R_DEVMODE_LEN},
        {VP886_R_CLKSLOTS_RD, VP886_R_CLKSLOTS_LEN},
        {VP886_R_INTMASK_RD, VP886_R_INTMASK_LEN},
        {VP886_R_SIGREG_NO_UL_RD, VP886_R_SIGREG_LEN},
        {VP886_R_SWCTRL_RD, VP886_R_SWCTRL_LEN},
        {VP886_R_SWTIMING_RD, VP886_R_SWTIMING_LEN},
        {VP886_R_GTIMER_RD, VP886_R_GTIMER_LEN},
        {VP886_R_TIMESTAMP_RD, VP886_R_TIMESTAMP_LEN},
        {VP886_R_CPCYCLES_RD, VP886_R_CPCYCLES_LEN},
        {VP886_R_EC_RD, VP886_R_EC_LEN},
    };

    char *deviceRegsName[VP886_DEVICE_REGISTER_COUNT] = {
        "VP886_R_RCNPCN_RD",        "VP886_R_INTREV_RD",
        "VP886_R_DEVCFG_RD",        "VP886_R_DEVMODE_RD",
        "VP886_R_CLKSLOTS_RD",      "VP886_R_INTMASK_RD",
        "VP886_R_SIGREG_NO_UL_RD",  "VP886_R_SWCTRL_RD",
        "VP886_R_SWTIMING_RD",      "VP886_R_GTIMER_RD",
        "VP886_R_TIMESTAMP_RD",     "VP886_R_CPCYCLES_RD",
        "VP886_R_EC_RD"
    };

    #define VP886_CHANNEL_REGISTER_COUNT    57
    uint8 channelRegs[VP886_CHANNEL_REGISTER_COUNT][2] = {
        {VP886_R_TXSLOT_RD, VP886_R_TXSLOT_LEN},
        {VP886_R_RXSLOT_RD, VP886_R_RXSLOT_LEN},
        {VP886_R_VPGAIN_RD, VP886_R_VPGAIN_LEN},
        {VP886_R_IODATA_RD, VP886_R_IODATA_LEN},
        {VP886_R_IODIR_RD, VP886_R_IODIR_LEN},
        {VP886_R_STATE_RD, VP886_R_STATE_LEN},
        {VP886_R_SW_OC_RD, VP886_R_SW_OC_LEN},
        {VP886_R_HOOKFREEZE_RD, VP886_R_HOOKFREEZE_LEN},
        {VP886_R_OPFUNC_RD, VP886_R_OPFUNC_LEN},
        {VP886_R_SSCFG_RD, VP886_R_SSCFG_LEN},
        {VP886_R_OPCOND_RD, VP886_R_OPCOND_LEN},
        {VP886_R_GX_RD, VP886_R_GX_LEN},
        {VP886_R_GR_RD, VP886_R_GR_LEN},
        {VP886_R_B_FIR_FILT_RD, VP886_R_B_FIR_FILT_LEN},
        {VP886_R_X_FILT_RD, VP886_R_X_FILT_LEN},
        {VP886_R_R_FILT_RD, VP886_R_R_FILT_LEN},
        {VP886_R_B_IIR_FILT_RD, VP886_R_B_IIR_FILT_LEN},
        {VP886_R_Z_FIR_FILT_RD, VP886_R_Z_FIR_FILT_LEN},
        {VP886_R_Z_IIR_FILT_RD, VP886_R_Z_IIR_FILT_LEN},
        {VP886_R_SADC_RD, VP886_R_SADC_LEN},
        {VP886_R_VADC_RD, VP886_R_VADC_LEN},
        {VP886_R_DCANCEL_RD, VP886_R_DCANCEL_LEN},
        {VP886_R_CALCTRL_RD, VP886_R_CALCTRL_LEN},
        {VP886_R_INDCAL_RD, VP886_R_INDCAL_LEN},
        {VP886_R_NORMCAL_RD, VP886_R_NORMCAL_LEN},
        {VP886_R_REVCAL_RD, VP886_R_REVCAL_LEN},
        {VP886_R_RINGCAL_RD, VP886_R_RINGCAL_LEN},
        {VP886_R_RINGGEN_RD, VP886_R_RINGGEN_LEN},
        {VP886_R_LOOPSUP_RD, VP886_R_LOOPSUP_LEN},
        {VP886_R_RINGDELAY_RD, VP886_R_RINGDELAY_LEN},
        {VP886_R_DCFEED_RD, VP886_R_DCFEED_LEN},
        {VP886_R_HOOKBUF_RD, VP886_R_HOOKBUF_LEN},
        {VP886_R_DISN_RD, VP886_R_DISN_LEN},
        {VP886_R_VBUFFER_RD, VP886_R_VBUFFER_LEN},
        {VP886_R_METER_RD, VP886_R_METER_LEN},
        {VP886_R_SIGAB_RD, VP886_R_SIGAB_LEN},
        {VP886_R_SIGCD_RD, VP886_R_SIGCD_LEN},
        {VP886_R_CHTIMER_RD, VP886_R_CHTIMER_LEN},
        {VP886_R_SIGCTRL_RD, VP886_R_SIGCTRL_LEN},
        {VP886_R_CADENCE_RD, VP886_R_CADENCE_LEN},
        {VP886_R_CIDDATA_RD, VP886_R_CIDDATA_LEN},
        {VP886_R_SWPARAM_RD, VP886_R_SWPARAM_LEN},
        {VP886_R_BATCAL_RD, VP886_R_BATCAL_LEN},
        {VP886_R_CIDPARAM_RD, VP886_R_CIDPARAM_LEN},
        {VP886_R_ICR1_RD, VP886_R_ICR1_LEN},
        {VP886_R_ICR2_RD, VP886_R_ICR2_LEN},
        {VP886_R_ICR3_RD, VP886_R_ICR3_LEN},
        {VP886_R_ICR4_RD, VP886_R_ICR4_LEN},
        {VP886_R_ICR5_RD, VP886_R_ICR5_LEN},
        {VP886_R_ICR6_RD, VP886_R_ICR6_LEN},
        {VP886_R_SW_ON_TIME_RD, VP886_R_SW_ON_TIME_LEN},
        {VP886_R_B1_RD, VP886_R_B1_LEN},
        {VP886_R_B2_RD, VP886_R_B2_LEN},
        {VP886_R_B3_RD, VP886_R_B3_LEN},
        {VP886_R_B4_RD, VP886_R_B4_LEN},
        {VP886_R_B5_RD, VP886_R_B5_LEN},
        {VP886_R_B6_RD, VP886_R_B6_LEN}
    };

    char *registerName[VP886_CHANNEL_REGISTER_COUNT] = {
        "VP886_R_TXSLOT_RD",        "VP886_R_RXSLOT_RD",
        "VP886_R_VPGAIN_RD",        "VP886_R_IODATA_RD",
        "VP886_R_IODIR_RD",         "VP886_R_STATE_RD",
        "VP886_R_SW_OC_RD",         "VP886_R_HOOKFREEZE_RD",
        "VP886_R_OPFUNC_RD",        "VP886_R_SSCFG_RD",
        "VP886_R_OPCOND_RD",        "VP886_R_GX_RD",
        "VP886_R_GR_RD",            "VP886_R_B_FIR_FILT_RD",
        "VP886_R_X_FILT_RD",        "VP886_R_R_FILT_RD",
        "VP886_R_B_IIR_FILT_RD",    "VP886_R_Z_FIR_FILT_RD",
        "VP886_R_Z_IIR_FILT_RD",    "VP886_R_SADC_RD",
        "VP886_R_VADC_RD",          "VP886_R_DCANCEL_RD",
        "VP886_R_CALCTRL_RD",       "VP886_R_INDCAL_RD",
        "VP886_R_NORMCAL_RD",       "VP886_R_REVCAL_RD",
        "VP886_R_RINGCAL_RD",       "VP886_R_RINGGEN_RD",
        "VP886_R_LOOPSUP_RD",
        "VP886_R_RINGDELAY_RD",     "VP886_R_DCFEED_RD",
        "VP886_R_HOOKBUF_RD",       "VP886_R_DISN_RD",
        "VP886_R_VBUFFER_RD",       "VP886_R_METER_RD",
        "VP886_R_SIGAB_RD",         "VP886_R_SIGCD_RD",
        "VP886_R_CHTIMER_RD",       "VP886_R_SIGCTRL_RD",
        "VP886_R_CADENCE_RD",       "VP886_R_CIDDATA_RD",
        "VP886_R_SWPARAM_RD",       "VP886_R_BATCAL_RD",
        "VP886_R_CIDPARAM_RD",      "VP886_R_ICR1_RD",
        "VP886_R_ICR2_RD",          "VP886_R_ICR3_RD",
        "VP886_R_ICR4_RD",          "VP886_R_ICR5_RD",
        "VP886_R_ICR6_RD",          "VP886_R_SW_ON_TIME_RD",
        "VP886_R_B1_RD",            "VP886_R_B2_RD",
        "VP886_R_B3_RD",            "VP886_R_B4_RD",
        "VP886_R_B5_RD",            "VP886_R_B6_RD"
    };

    VpSysDebugPrintf("\n\rDevice Registers:\n\r");

    /* Read all of the Global registers and print their contents */
    VpSysDebugPrintf("\n\rDEVICE");
    for (registerNumber = 0; registerNumber < VP886_DEVICE_REGISTER_COUNT; registerNumber++) {
        /* The charge pump cycle counter register does not exist for SF
           devices, so skip it to avoid a potential desync */
        if (deviceRegs[registerNumber][0] == VP886_R_CPCYCLES_RD && VP886_IS_SF(pDevObj)) {
            continue;
        }
    
        VpSlacRegRead(pDevCtx, NULL, deviceRegs[registerNumber][0],
            deviceRegs[registerNumber][1], registerBuffer);

        VpSysDebugPrintf("\n\r%s (0x%02X) ",
            deviceRegsName[registerNumber], deviceRegs[registerNumber][0]);
        for (registerIndex = 0;
             registerIndex < deviceRegs[registerNumber][1];
             registerIndex++) {
            VpSysDebugPrintf("0x%02X ", registerBuffer[registerIndex]);
        }
    }

    /* Read all of the channel specific registers and print their contents */
    for (channelId = 0; channelId < pDevObj->staticInfo.maxChannels; channelId++) {
        VpLineCtxType *pLineCtx = pDevCtx->pLineCtx[channelId];
        VpSysDebugPrintf("\n\n\rCHANNEL %d", channelId);
        for (registerNumber = 0; registerNumber < VP886_CHANNEL_REGISTER_COUNT; registerNumber++) {
            /* Ring delay register was added for SF devices only */
            if (channelRegs[registerNumber][0] == VP886_R_RINGDELAY_RD && !VP886_IS_SF(pDevObj)) {
                continue;
            }
        
            VpSlacRegRead(NULL, pLineCtx, channelRegs[registerNumber][0],
                channelRegs[registerNumber][1], registerBuffer);

            VpSysDebugPrintf("\n\r%s (0x%02X) ",
                registerName[registerNumber], channelRegs[registerNumber][0]);
            for (registerIndex = 0;
                 registerIndex < channelRegs[registerNumber][1];
                 registerIndex++) {
                VpSysDebugPrintf("0x%02X ", registerBuffer[registerIndex]);
            }
        }
    }

    VpSysDebugPrintf("\n\r");

    return VP_STATUS_SUCCESS;
}


/** Vp886ObjectDump()
  Implements VpObjectDump() to print device and line object data.
*/
VpStatusType
Vp886ObjectDump(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx)
{
    VpStatusType status;

    Vp886EnterCritical(pDevCtx, pLineCtx, "Vp886ObjectDump");

    status = Vp886ObjectDumpInt(pLineCtx, pDevCtx);

    Vp886ExitCritical(pDevCtx, pLineCtx, "Vp886ObjectDump");
    return status;
}


/** Vp886ObjectDumpInt()
  Internal implementation that can be called to bypass the enter/exit critical
  section.
*/
VpStatusType
Vp886ObjectDumpInt(
    VpLineCtxType *pLineCtx,
    VpDevCtxType *pDevCtx)
{
    Vp886DeviceObjectType *pDevObj;

    if (pDevCtx != VP_NULL) {
        pDevObj = pDevCtx->pDevObj;

        VP_PRINT_DEVICE_ID(pDevObj->deviceId);
        VpPrintStaticInfoStruct(&pDevObj->staticInfo);
        VpPrintDynamicInfoStruct(&pDevObj->dynamicInfo);
        VpSysDebugPrintf("\n\n\rpDevObj->busyFlags = 0x%04X", pDevObj->busyFlags);
        VpSysDebugPrintf("\n\rpDevObj->stateInt = 0x%04X", pDevObj->stateInt);
        VpSysDebugPrintf("\n\rpDevObj->ecVal = 0x%02X", pDevObj->ecVal);
        VpSysDebugPrintf("\n\rpDevObj->getResultsRequired = %s", pDevObj->getResultsRequired ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->pendingInterrupt = %s", pDevObj->pendingInterrupt ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->inGetEvent = %s", pDevObj->inGetEvent ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->sigregReadPending = %s", pDevObj->sigregReadPending ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->criticalDepth = %d", pDevObj->criticalDepth);
        VpSysDebugPrintf("\n\rpDevObj->trafficBytes = %lu", pDevObj->trafficBytes);
        VpSysDebugPrintf("\n\rpDevObj->timerOverride = %s", pDevObj->timerOverride ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->spiError = %s", pDevObj->spiError ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->ioCapability = %d", pDevObj->ioCapability);
        VpSysDebugPrintf("\n\rpDevObj->isPowerLimited[0] = %s", pDevObj->isPowerLimited[0] ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->isPowerLimited[1] = %s", pDevObj->isPowerLimited[1] ? "TRUE" : "FALSE");

        /* Event queue info */
        VpSysDebugPrintf("\n\n\rpDevObj->eventQueue.numQueued = %d", pDevObj->eventQueue.numQueued);
        VpSysDebugPrintf("\n\rpDevObj->eventQueue.pushIndex = %d", pDevObj->eventQueue.pushIndex);
        VpSysDebugPrintf("\n\rpDevObj->eventQueue.popIndex = %d", pDevObj->eventQueue.popIndex);
        VpSysDebugPrintf("\n\rpDevObj->eventQueue.overflowed = %s", pDevObj->eventQueue.overflowed ? "TRUE" : "FALSE");
        if (pDevObj->eventQueue.numQueued > 0) {
            uint8 i;
            uint8 num;
            for (num = 0; num < pDevObj->eventQueue.numQueued; num++) {
                i = (pDevObj->eventQueue.popIndex + num) % VP886_EVENT_QUEUE_SIZE;
                VpSysDebugPrintf("\n\rpDevObj->eventQueue.events[%d]: ch=%d cat=%d id=%d data=%d hndl=%d", i,
                    pDevObj->eventQueue.events[i].channelId,
                    pDevObj->eventQueue.events[i].eventCategory,
                    pDevObj->eventQueue.events[i].eventId,
                    pDevObj->eventQueue.events[i].eventData,
                    pDevObj->eventQueue.events[i].parmHandle);
            }
        }

        /* Timer related */
        VpPrintTimeStamp(pDevObj->timestamp);
        VpSysDebugPrintf("\n\rpDevObj->timestampValid = %s", pDevObj->timestampValid ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->rolloverCount = %u", pDevObj->rolloverCount);
        VpSysDebugPrintf("\n\rpDevObj->rolloverBuffer = %ld", pDevObj->rolloverBuffer);
        #if (VP886_USER_TIMERS > 0)
        VpSysDebugPrintf("\n\rpDevObj->userTimers = %d", pDevObj->userTimers);
        #endif
        VpPrintTimerQueue(&pDevObj->timerQueueInfo, pDevObj->timerQueueNodes);

        /* For Vp886DevOptionsCacheType options, use VpGetOption */

        VpPrintDeviceProfileStruct(VP_DEV_886_SERIES, &pDevObj->devProfileData);
        VpPrintDeviceProfileTable(&pDevObj->devProfileTable);

        VpPrintSlacBufData(&pDevObj->slacBufData);
        VpSysDebugPrintf("\n\rpDevObj->dontFlushSlacBufOnRead = %s", pDevObj->dontFlushSlacBufOnRead ? "TRUE" : "FALSE");

        VpSysDebugPrintf("\n\n\rpDevObj->swParams = 0x%02X 0x%02X 0x%02X",
            pDevObj->swParams[0], pDevObj->swParams[1], pDevObj->swParams[2]);

        {
            uint8 byteCount;
            VpSysDebugPrintf("\n\rpDevObj->swTimingParams =");
            for (byteCount = 0; byteCount < VP886_R_SWTIMING_LEN; byteCount++) {
                VpSysDebugPrintf(" 0x%02X", pDevObj->swTimingParams[byteCount]);
            }
        }
        {
            uint8 byteCount;
            VpSysDebugPrintf("\n\rpDevObj->swTimingParamsFR =");
            for (byteCount = 0; byteCount < VP886_R_SWTIMING_LEN; byteCount++) {
                VpSysDebugPrintf(" 0x%02X", pDevObj->swTimingParamsFR[byteCount]);
            }
        }

        VpSysDebugPrintf("\n\rpDevObj->useLowPowerTiming = %s", pDevObj->useLowPowerTiming ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->swTimingParamsLP = 0x%02X 0x%02X",
            pDevObj->swTimingParamsLP[0], pDevObj->swTimingParamsLP[1]);

        VpSysDebugPrintf("\n\n\rpDevObj->initDeviceState = %d", pDevObj->initDeviceState);
        VpSysDebugPrintf("\n\rpDevObj->initDevSwitcherMode = 0x%02X", pDevObj->initDevSwitcherMode);
        
        if (VP886_IS_ABS(pDevObj)) {
            VpSysDebugPrintf("\n\n\rpDevObj->absPowerReq[0] = %d", pDevObj->absPowerReq[0]);
            VpSysDebugPrintf("\n\rpDevObj->absPowerReq[1] = %d", pDevObj->absPowerReq[1]);
            VpSysDebugPrintf("\n\rpDevObj->absRingingBattRequired[0] = %s", pDevObj->absRingingBattRequired[0] ? "TRUE" : "FALSE");
            VpSysDebugPrintf("\n\rpDevObj->absRingingBattRequired[1] = %s", pDevObj->absRingingBattRequired[1] ? "TRUE" : "FALSE");
            VpSysDebugPrintf("\n\rpDevObj->absRingingPeak[0] = %d", pDevObj->absRingingPeak[0]);
            VpSysDebugPrintf("\n\rpDevObj->absRingingPeak[1] = %d", pDevObj->absRingingPeak[1]);
            VpSysDebugPrintf("\n\rpDevObj->swyv = %d", pDevObj->swyv);
            VpSysDebugPrintf("\n\rpDevObj->swzv = %d", pDevObj->swzv);
        }

        VpSysDebugPrintf("\n\n\rpDevObj->samplingTimerReq[0] = %d", pDevObj->samplingTimerReq[0]);
        VpSysDebugPrintf("\n\rpDevObj->samplingTimerReq[1] = %d", pDevObj->samplingTimerReq[1]);
        VpSysDebugPrintf("\n\rpDevObj->samplingTimerCurrent = %d", pDevObj->samplingTimerCurrent);

        VpSysDebugPrintf("\n\n\rpDevObj->battFltStatus = %04X", pDevObj->battFltStatus);
        VpSysDebugPrintf("\n\rpDevObj->battFltPollTime = %d", pDevObj->battFltPollTime);
        VpSysDebugPrintf("\n\rpDevObj->overCurrentTimestamp[0] = %lu", pDevObj->overCurrentTimestamp[0]);
        VpSysDebugPrintf("\n\rpDevObj->overCurrentTimestamp[1] = %lu", pDevObj->overCurrentTimestamp[1]);

        /* Calibration info */
        VpSysDebugPrintf("\n\n\rpDevObj->calCodecState[0] = %d", pDevObj->calCodecState[0]);
        VpSysDebugPrintf("\n\rpDevObj->calCodecState[1] = %d", pDevObj->calCodecState[1]);
        VpSysDebugPrintf("\n\rpDevObj->calCodecSubState[0] = %d", pDevObj->calCodecSubState[0]);
        VpSysDebugPrintf("\n\rpDevObj->calCodecSubState[1] = %d", pDevObj->calCodecSubState[1]);
        VpSysDebugPrintf("\n\rpDevObj->channelCalBack[0] = %s", pDevObj->channelCalBack[0] ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->channelCalBack[1] = %s", pDevObj->channelCalBack[1] ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->channelLocked[0] = %s", pDevObj->channelLocked[0] ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->channelLocked[1] = %s", pDevObj->channelLocked[1] ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->icalH = %d", pDevObj->icalH);
        VpSysDebugPrintf("\n\rpDevObj->icalL = %d", pDevObj->icalL);
        VpSysDebugPrintf("\n\rpDevObj->polarity[0] = %d", pDevObj->polarity[0]);
        VpSysDebugPrintf("\n\rpDevObj->polarity[1] = %d", pDevObj->polarity[1]);
        VpSysDebugPrintf("\n\rpDevObj->calData[0].valid = %s", pDevObj->calData[0].valid ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpDevObj->calData[1].valid = %s", pDevObj->calData[1].valid ? "TRUE" : "FALSE");

#if (VP_CC_DEBUG_SELECT & VP_DBG_CALIBRATION)
        /* Print the calibration profiles if calibration debug is enabled for
           the device. */
        if (pDevObj->debugSelectMask & VP_DBG_CALIBRATION) {
            uint8 channelId;
            uint8 calProf[255];
            for (channelId = 0; channelId < pDevObj->staticInfo.maxChannels; channelId++) {
                VpLineCtxType *pLineCtxLocal = pDevCtx->pLineCtx[channelId];
                uint8 idx;

                if (pLineCtxLocal == VP_NULL) {
                    continue;
                }
                
                VpSysDebugPrintf("\n\n\rCalibration profile, ch%d = {", channelId);
                Vp886GetCalProfile(pLineCtxLocal, calProf);
                for (idx = 0; idx < calProf[VP_PROFILE_LENGTH] + 4; idx++) {
                    if (idx % 12 == 0) {
                        VpSysDebugPrintf("\n\r%8s", "");
                    }
                    VpSysDebugPrintf("0x%02X ", calProf[idx]);
                }
                VpSysDebugPrintf("\n\r%4s}", "");
            }
        }
#endif /* (VP_CC_DEBUG_SELECT & VP_DBG_CALIBRATION) */

        /* The Dev, AC, DC, Ring pointers are only used during VpInitDevice().
           They have meaningless values outside of that context. Don't
           print them out */

        VpSysDebugPrintf("\n\rpDevObj->registers.sigreg = 0x%02X 0x%02X 0x%02X 0x%02X",
            pDevObj->registers.sigreg[0], pDevObj->registers.sigreg[1],
            pDevObj->registers.sigreg[2], pDevObj->registers.sigreg[3]);
        VpSysDebugPrintf("\n\rpDevObj->registers.swCtrl = 0x%02X",
            pDevObj->registers.swCtrl[0]);
        VpSysDebugPrintf("\n\rpDevObj->registers.intMask = 0x%02X 0x%02X 0x%02X 0x%02X",
            pDevObj->registers.intMask[0], pDevObj->registers.intMask[1],
            pDevObj->registers.intMask[2], pDevObj->registers.intMask[3]);
        VpSysDebugPrintf("\n\rpDevObj->registers.devCfg = 0x%02X",
            pDevObj->registers.devCfg[0]);

        VpSysDebugPrintf("\n\n\rpDevObj->debugSelectMask = 0x%08lX", pDevObj->debugSelectMask);

    }

    if (pLineCtx != VP_NULL) {
        Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
        pDevObj = pLineCtx->pDevCtx->pDevObj;

        VpSysDebugPrintf("Vp886ObjectDump Line starts here \n");

        VP_PRINT_LINE_ID(pLineObj->lineId);
        VpSysDebugPrintf("\n\n\rpLineObj->channelId = %d\n\rpLineObj->ecVal = 0x%02X",
            pLineObj->channelId, pLineObj->ecVal);
        VpPrintTermType(pLineObj->termType);
        VpSysDebugPrintf("\n\rpLineObj->isFxs = %s", pLineObj->isFxs ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpLineObj->busyFlags = 0x%04X \n", pLineObj->busyFlags);
        VpSysDebugPrintf("\n\rpLineObj->inLineTest = %s", pLineObj->inLineTest ? "TRUE" : "FALSE");
        VpPrintApiIntLineState(&pLineObj->lineState);
        VpSysDebugPrintf("\n\rpLineObj->detectMasks = 0x%04X", pLineObj->detectMasks);
        VpSysDebugPrintf("\n\rpLineObj->io2State = %s", pLineObj->io2State ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\n\rpLineObj->intTestTermApplied = %s", pLineObj->intTestTermApplied ? "TRUE" : "FALSE");
#if (VP886_USER_TIMERS > 0)
        /* Number of currently running user timers.  Used to implement a limit
           of VP886_USER_TIMERS */
        VpSysDebugPrintf("\n\rpLineObj->userTimers = %d", pLineObj->userTimers);
#endif
        VpSysDebugPrintf("\n\rpLineObj->ringExitInProgress = %s", pLineObj->ringExitInProgress ? "TRUE" : "FALSE");
        VpPrintLineStateType(pLineObj->ringExitCleanupState, "pLineObj->ringExitCleanupState");
        VpSysDebugPrintf("\n\rpLineObj->toneGens = 0x%02X", pLineObj->toneGens);
#ifdef VP886_INCLUDE_TESTLINE_CODE
        VpPrintRelayControlType(pLineObj->relayState);
#endif
        if (pLineObj->busyFlags & VP886_LINE_IN_CAL) {
            VpSysDebugPrintf("\n\rpLineObj->calLineState = %d", pLineObj->calLineState);
            VpSysDebugPrintf("\n\rpLineObj->calLineSubState = %d", pLineObj->calLineSubState);
        }
        VpSysDebugPrintf("\n\rpLineObj->calRegsProgrammed = %s", pLineObj->calRegsProgrammed ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpLineObj->ringSyncState = %d", pLineObj->ringSyncState);
        VpPrintLineStateType(pLineObj->ringSyncLineState, "pLineObj->ringSyncLineState");
        VpSysDebugPrintf("\n\rpLineObj->ringTripConfirm.confirming = %d", pLineObj->ringTripConfirm.confirming);
        if (pLineObj->ringTripConfirm.confirming) {
            VpSysDebugPrintf("\n\rpLineObj->ringTripConfirm.glitchCheck = %d", pLineObj->ringTripConfirm.glitchCheck);
            VpSysDebugPrintf("\n\rpLineObj->ringTripConfirm.lastHookTimestamp = %d", pLineObj->ringTripConfirm.lastHookTimestamp);
        }

        /* Don't print out the line options. The user can call VpGetOption for that */

#ifdef VP_CSLAC_SEQ_EN
        VpSysDebugPrintf("\n\n\rpLineObj->cadence.status = 0x%04X", pLineObj->cadence.status);
        if (pLineObj->cadence.status & VP_CADENCE_STATUS_ACTIVE) {
            VpSysDebugPrintf("\n\rpLineObj->cadence.pActiveCadence = %p", pLineObj->cadence.pActiveCadence);
            VpSysDebugPrintf("\n\rpLineObj->cadence.pCurrentPos = %p", pLineObj->cadence.pCurrentPos);
            VpSysDebugPrintf("\n\rpLineObj->cadence.toneType = %d", pLineObj->cadence.toneType);
            if (pLineObj->cadence.toneType == VP_CSLAC_HOWLER_TONE ||
                pLineObj->cadence.toneType == VP_CSLAC_AUS_HOWLER_TONE ||
                pLineObj->cadence.toneType == VP_CSLAC_NTT_HOWLER_TONE)
            {
                VpSysDebugPrintf("\n\rpLineObj->cadence.startLevel = %d", pLineObj->cadence.startLevel);
                VpSysDebugPrintf("\n\rpLineObj->cadence.stopLevel = %d", pLineObj->cadence.stopLevel);
                VpSysDebugPrintf("\n\rpLineObj->cadence.levelStep = %d", pLineObj->cadence.levelStep);
                VpSysDebugPrintf("\n\rpLineObj->howlerState = %d", pLineObj->howlerState);
                VpSysDebugPrintf("\n\rpLineObj->howlerLevel = %d", pLineObj->howlerLevel);
            } else {
                VpSysDebugPrintf("\n\rpLineObj->cadence.index = %d", pLineObj->cadence.index);
                VpSysDebugPrintf("\n\rpLineObj->cadence.length = %d", pLineObj->cadence.length);
                VpSysDebugPrintf("\n\rpLineObj->cadence.branchDepth = %d", pLineObj->cadence.branchDepth);
                if (pLineObj->cadence.branchDepth > 0) {
                    uint8 i;
                    for (i = 0; i < pLineObj->cadence.branchDepth; i++) {
                        VpSysDebugPrintf("\n\rpLineObj->cadence.branchCount[%d] = %d", i, pLineObj->cadence.branchCount[i]);
                        VpSysDebugPrintf("\n\rpLineObj->cadence.branchIdx[%d] = %d", i, pLineObj->cadence.branchIdx[i]);
                    }
                }
            }
        }

        VpSysDebugPrintf("\n\rpLineObj->pRingingCadence = %p", pLineObj->pRingingCadence);

        VpSysDebugPrintf("\n\n\rpLineObj->cid.active = %s", pLineObj->cid.active ? "TRUE" : "FALSE");
        if (pLineObj->cid.active) {
            VpSysDebugPrintf("\n\rpLineObj->cid.pProf = %p", pLineObj->cid.pProf);
            VpSysDebugPrintf("\n\rpLineObj->cid.profIdx = %d", pLineObj->cid.profIdx);
            VpSysDebugPrintf("\n\rpLineObj->cid.state = %d", pLineObj->cid.state);
            VpSysDebugPrintf("\n\rpLineObj->cid.msgLen = %d", pLineObj->cid.msgLen);
            VpSysDebugPrintf("\n\rpLineObj->cid.msgIdx = %d", pLineObj->cid.msgIdx);
            VpSysDebugPrintf("\n\rpLineObj->cid.checksum = %d", pLineObj->cid.checksum);
            VpSysDebugPrintf("\n\rpLineObj->cid.markOrSeizureBytes = %d", pLineObj->cid.markOrSeizureBytes);
            VpSysDebugPrintf("\n\rpLineObj->cid.detectDigit1 = 0x%X", pLineObj->cid.detectDigit1);
            VpSysDebugPrintf("\n\rpLineObj->cid.detectDigit2 = 0x%X", pLineObj->cid.detectDigit2);
            VpSysDebugPrintf("\n\rpLineObj->cid.digitDetected = %s", pLineObj->cid.digitDetected ? "TRUE" : "FALSE");
            VpSysDebugPrintf("\n\rpLineObj->cid.fskEnabled = %s", pLineObj->cid.fskEnabled ? "TRUE" : "FALSE");
            VpSysDebugPrintf("\n\rpLineObj->cid.dtmfEnabled = %s", pLineObj->cid.dtmfEnabled ? "TRUE" : "FALSE");
            VpSysDebugPrintf("\n\rpLineObj->cid.mute = %s", pLineObj->cid.mute ? "TRUE" : "FALSE");
            VpSysDebugPrintf("\n\rpLineObj->cid.needData = %s", pLineObj->cid.needData ? "TRUE" : "FALSE");
        }

        VpSysDebugPrintf("\n\n\rpLineObj->sendSignal.active = %s", pLineObj->sendSignal.active ? "TRUE" : "FALSE");
        if (pLineObj->sendSignal.active) {
            VpSysDebugPrintf("\n\rpLineObj->sendSignal.type = %d", pLineObj->sendSignal.type);
            if (pLineObj->sendSignal.type == VP_SENDSIG_MSG_WAIT_PULSE) {
                VpSysDebugPrintf("\n\rpLineObj->sendSignal.msgWait.voltage = %d", pLineObj->sendSignal.msgWait.voltage);
                VpSysDebugPrintf("\n\rpLineObj->sendSignal.msgWait.onTime = %d", pLineObj->sendSignal.msgWait.onTime);
                VpSysDebugPrintf("\n\rpLineObj->sendSignal.msgWait.offTime = %d", pLineObj->sendSignal.msgWait.offTime);
                VpSysDebugPrintf("\n\rpLineObj->sendSignal.msgWait.cycles = %d", pLineObj->sendSignal.msgWait.cycles);
                VpSysDebugPrintf("\n\rpLineObj->sendSignal.msgWait.on = %s", pLineObj->sendSignal.msgWait.on ? "TRUE" : "FALSE");
            }
        }

        VpSysDebugPrintf("\n\n\rpLineObj->metering.active = %s", pLineObj->metering.active ? "TRUE" : "FALSE");
        if (pLineObj->metering.active) {
            VpSysDebugPrintf("\n\rpLineObj->metering.on = %s", pLineObj->metering.on ? "TRUE" : "FALSE");
            VpSysDebugPrintf("\n\rpLineObj->metering.onTime = %d", pLineObj->metering.onTime);
            VpSysDebugPrintf("\n\rpLineObj->metering.offTime = %d", pLineObj->metering.offTime);
            VpSysDebugPrintf("\n\rpLineObj->metering.remaining = %d", pLineObj->metering.remaining);
            VpSysDebugPrintf("\n\rpLineObj->metering.completed = %d", pLineObj->metering.completed);
            VpSysDebugPrintf("\n\rpLineObj->metering.abort = %s", pLineObj->metering.abort ? "TRUE" : "FALSE");
        }
#endif /* VP_CSLAC_SEQ_EN */

        VpPrintPulseDecodeData(&pLineObj->pulseDecodeData);

#ifdef VP886_INCLUDE_DTMF_DETECT
        VpSysDebugPrintf("\n\n\rpLineObj->dtmf.sampling = %s", pLineObj->dtmf.sampling ? "TRUE" : "FALSE");
        if (pLineObj->dtmf.sampling) {
            VpSysDebugPrintf("\n\n\rpLineObj->dtmf.overflow = %s", pLineObj->dtmf.overflow ? "TRUE" : "FALSE");
            VpPrintDigitType(pLineObj->dtmf.currentDigit, "pLineObj->dtmf.currentDigit");
            VpSysDebugPrintf("\n\rpLineObj->dtmf.useVadc = %s", pLineObj->dtmf.useVadc ? "TRUE" : "FALSE");
            VpSysDebugPrintf("\n\rpLineObj->dtmf.detectData.blockSize = %d", pLineObj->dtmf.detectData.blockSize);
            VpSysDebugPrintf("\n\rpLineObj->dtmf.detectData.currentSample = %d", pLineObj->dtmf.detectData.currentSample);
            VpPrintDigitType(pLineObj->dtmf.detectData.prevHit, "pLineObj->dtmf.detectData.prevHit");
            VpPrintDigitType(pLineObj->dtmf.detectData.digitOutput, "pLineObj->dtmf.detectData.digitOutput");
            VpSysDebugPrintf("\n\rpLineObj->dtmf.detectData.totalEnergy = %lu", pLineObj->dtmf.detectData.totalEnergy);
        }
#endif

        VpSysDebugPrintf("\n\n\rpLineObj->reportDcFaults = %s",
            pLineObj->reportDcFaults ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpLineObj->floorVoltage = %d", pLineObj->floorVoltage);
        VpSysDebugPrintf("\n\rpLineObj->targetVoc = %d", pLineObj->targetVoc);
        VpSysDebugPrintf("\n\rpLineObj->ringBias = %d", pLineObj->ringBias);
        VpSysDebugPrintf("\n\rpLineObj->ringAmplitude = %d", pLineObj->ringAmplitude);
        VpSysDebugPrintf("\n\rpLineObj->ringAmplitudeCal = %d", pLineObj->ringAmplitudeCal);
        VpSysDebugPrintf("\n\rpLineObj->ringFrequency = %d", pLineObj->ringFrequency);
        VpSysDebugPrintf("\n\rpLineObj->ringSine = %s", pLineObj->ringSine ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpLineObj->fixedRingBat = %d", pLineObj->fixedRingBat);
        VpSysDebugPrintf("\n\rpLineObj->unbalancedRinging = %s", pLineObj->unbalancedRinging ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpLineObj->ringTripAlg = 0x%02X", pLineObj->ringTripAlg);
        VpSysDebugPrintf("\n\rpLineObj->ringTripCycles = %d", pLineObj->ringTripCycles);

        /* GR/GX Gain related */
        VpSysDebugPrintf("\n\n\rpLineObj->gxBase = 0x%04X", pLineObj->gxBase);
        VpSysDebugPrintf("\n\rpLineObj->gxUserLevel = 0x%04X", pLineObj->gxUserLevel);
        VpSysDebugPrintf("\n\rpLineObj->grBase = 0x%04X", pLineObj->grBase);
        VpSysDebugPrintf("\n\rpLineObj->grUserLevel = 0x%04X", pLineObj->grUserLevel);
        VpSysDebugPrintf("\n\rpLineObj->absGxRestoreOption = %d", pLineObj->absGxRestoreOption);
        VpSysDebugPrintf("\n\rpLineObj->absGrRestoreOption = %d", pLineObj->absGrRestoreOption);
        VpSysDebugPrintf("\n\rpLineObj->absGxRestoreReg = %02X %02X", pLineObj->absGxRestoreReg[0], pLineObj->absGxRestoreReg[1]);
        VpSysDebugPrintf("\n\rpLineObj->absGrRestoreReg = %02X %02X", pLineObj->absGrRestoreReg[0], pLineObj->absGrRestoreReg[1]);

        VpSysDebugPrintf("\n\n\rpLineObj->lowIlr = %s", pLineObj->lowIlr ? "TRUE" : "FALSE");
        VpSysDebugPrintf("\n\rpLineObj->rtth = 0x%02X", pLineObj->rtth);

        VpSysDebugPrintf("\n\n\rpLineObj->lineTopology.rInsideDcSense = %d", pLineObj->lineTopology.rInsideDcSense);
        VpSysDebugPrintf("\n\rpLineObj->lineTopology.rOutsideDcSense = %d", pLineObj->lineTopology.rOutsideDcSense);

#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
        VpSysDebugPrintf("\n\n\rpLineObj->inHighGainMode = %s", pLineObj->inHighGainMode ? "TRUE" : "FALSE");
        if (pLineObj->inHighGainMode) {
            VpSysDebugPrintf("\n\rpLineObj->highGainCache.icr1 = 0x%02X 0x%02X 0x%02X 0x%02X",
                pLineObj->highGainCache.icr1[0], pLineObj->highGainCache.icr1[1],
                pLineObj->highGainCache.icr1[2], pLineObj->highGainCache.icr1[3]);
            VpSysDebugPrintf("\n\rpLineObj->highGainCache.icr2 = 0x%02X 0x%02X 0x%02X 0x%02X",
                pLineObj->highGainCache.icr2[0], pLineObj->highGainCache.icr2[1],
                pLineObj->highGainCache.icr2[2], pLineObj->highGainCache.icr2[3]);
            VpSysDebugPrintf("\n\rpLineObj->highGainCache.icr3 = 0x%02X 0x%02X 0x%02X 0x%02X",
                pLineObj->highGainCache.icr3[0], pLineObj->highGainCache.icr3[1],
                pLineObj->highGainCache.icr3[2], pLineObj->highGainCache.icr3[3]);
            VpSysDebugPrintf("\n\rpLineObj->highGainCache.icr4 = 0x%02X 0x%02X 0x%02X 0x%02X",
                pLineObj->highGainCache.icr4[0], pLineObj->highGainCache.icr4[1],
                pLineObj->highGainCache.icr4[2], pLineObj->highGainCache.icr4[3]);
            VpSysDebugPrintf("\n\rpLineObj->highGainCache.dcFeed = 0x%02X 0x%02X",
                pLineObj->highGainCache.dcFeed[0], pLineObj->highGainCache.dcFeed[1]);
            VpSysDebugPrintf("\n\rpLineObj->highGainCache.vpGain = 0x%02X", pLineObj->highGainCache.vpGain[0]);
            VpSysDebugPrintf("\n\rpLineObj->highGainCache.grValue = 0x%02X 0x%02X",
                pLineObj->highGainCache.grValue[0], pLineObj->highGainCache.grValue[1]);
            VpSysDebugPrintf("\n\rpLineObj->highGainCache.rValue = 0x%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
                pLineObj->highGainCache.rValue[0], pLineObj->highGainCache.rValue[1],
                pLineObj->highGainCache.rValue[2], pLineObj->highGainCache.rValue[3],
                pLineObj->highGainCache.rValue[4], pLineObj->highGainCache.rValue[5],
                pLineObj->highGainCache.rValue[6], pLineObj->highGainCache.rValue[7],
                pLineObj->highGainCache.rValue[8], pLineObj->highGainCache.rValue[9],
                pLineObj->highGainCache.rValue[10], pLineObj->highGainCache.rValue[11],
                pLineObj->highGainCache.rValue[12], pLineObj->highGainCache.rValue[13]);
            VpSysDebugPrintf("\n\rpLineObj->highGainCache.disn = 0x%02X", pLineObj->highGainCache.disn[0]);
            VpSysDebugPrintf("\n\rpLineObj->highGainCache.opFunc = 0x%02X", pLineObj->highGainCache.opFunc[0]);
            VpSysDebugPrintf("\n\rpLineObj->highGainCache.swParam = 0x%02X 0x%02X 0x%02X ",
                pLineObj->highGainCache.swParam[0], pLineObj->highGainCache.swParam[1],
                pLineObj->highGainCache.swParam[2]);
        }
#endif /* VP_HIGH_GAIN_MODE_SUPPORTED */
        
        VpSysDebugPrintf("\n\n\rpLineObj->gndFltProt.state = %d", pLineObj->gndFltProt.state);
        if (pLineObj->gndFltProt.state != VP886_GNDFLTPROT_ST_INACTIVE) {
            VpSysDebugPrintf("\n\rpLineObj->gndFltProt.faultDeclared = %s", pLineObj->gndFltProt.faultDeclared ? "TRUE" : "FALSE");
            VpSysDebugPrintf("\n\rpLineObj->gndFltProt.settingDisconnect = %s", pLineObj->gndFltProt.settingDisconnect ? "TRUE" : "FALSE");
            VpSysDebugPrintf("\n\rpLineObj->gndFltProt.iterations = %d", pLineObj->gndFltProt.iterations);
        }

#ifdef VP886_INCLUDE_ADAPTIVE_RINGING
        if (pDevObj->options.adaptiveRinging.power != VP_ADAPTIVE_RINGING_DISABLED) {
            VpSysDebugPrintf("\n\n\rpLineObj->thermalRinging = %s", pLineObj->thermalRinging ? "TRUE" : "FALSE");
            VpSysDebugPrintf("\n\rpLineObj->startRingingCadence = %s", pLineObj->startRingingCadence ? "TRUE" : "FALSE");
            VpSysDebugPrintf("\n\rpLineObj->wasStandby = %s", pLineObj->wasStandby ? "TRUE" : "FALSE");
        }
#endif /* VP886_INCLUDE_ADAPTIVE_RINGING */

        /* Cached registers */
        VpSysDebugPrintf("\n\n\rpLineObj->registers.sysState = 0x%02X", pLineObj->registers.sysState[0]);
        VpSysDebugPrintf("\n\rpLineObj->registers.loopSup = 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->registers.loopSup[0], pLineObj->registers.loopSup[1],
            pLineObj->registers.loopSup[2], pLineObj->registers.loopSup[3]);
        VpSysDebugPrintf("\n\rpLineObj->registers.opCond = 0x%02X", pLineObj->registers.opCond[0]);
        VpSysDebugPrintf("\n\rpLineObj->registers.opFunc = 0x%02X", pLineObj->registers.opFunc[0]);
        VpSysDebugPrintf("\n\rpLineObj->registers.swParam = 0x%02X 0x%02X 0x%02X ",
            pLineObj->registers.swParam[0], pLineObj->registers.swParam[1],
            pLineObj->registers.swParam[2]);
        VpSysDebugPrintf("\n\rpLineObj->registers.calCtrl = 0x%02X 0x%02X 0x%02X ",
            pLineObj->registers.calCtrl[0], pLineObj->registers.calCtrl[1],
            pLineObj->registers.calCtrl[2]);
        VpSysDebugPrintf("\n\rpLineObj->registers.icr1 = 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->registers.icr1[0], pLineObj->registers.icr1[1],
            pLineObj->registers.icr1[2], pLineObj->registers.icr1[3]);
        VpSysDebugPrintf("\n\rpLineObj->registers.icr2 = 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->registers.icr2[0], pLineObj->registers.icr2[1],
            pLineObj->registers.icr2[2], pLineObj->registers.icr2[3]);
        VpSysDebugPrintf("\n\rpLineObj->registers.icr3 = 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->registers.icr3[0], pLineObj->registers.icr3[1],
            pLineObj->registers.icr3[2], pLineObj->registers.icr3[3]);
        VpSysDebugPrintf("\n\rpLineObj->registers.icr4 = 0x%02X 0x%02X 0x%02X 0x%02X",
            pLineObj->registers.icr4[0], pLineObj->registers.icr4[1],
            pLineObj->registers.icr4[2], pLineObj->registers.icr4[3]);
        VpSysDebugPrintf("\n\rpLineObj->registers.dcFeed = 0x%02X 0x%02X",
            pLineObj->registers.dcFeed[0], pLineObj->registers.dcFeed[1]);
        VpSysDebugPrintf("\n\rpLineObj->registers.ssCfg = 0x%02X 0x%02X",
            pLineObj->registers.ssCfg[0], pLineObj->registers.ssCfg[1]);
        VpSysDebugPrintf("\n\rpLineObj->registers.indCal = 0x%02X 0x%02X 0x%02X",
            pLineObj->registers.indCal[0], pLineObj->registers.indCal[1],
            pLineObj->registers.indCal[2]);
        VpSysDebugPrintf("\n\rpLineObj->registers.normCal = 0x%02X 0x%02X 0x%02X",
            pLineObj->registers.normCal[0], pLineObj->registers.normCal[1],
            pLineObj->registers.normCal[2]);
        VpSysDebugPrintf("\n\rpLineObj->registers.revCal = 0x%02X 0x%02X 0x%02X",
            pLineObj->registers.revCal[0], pLineObj->registers.revCal[1],
            pLineObj->registers.revCal[2]);
        VpSysDebugPrintf("\n\rpLineObj->registers.ringCal = 0x%02X 0x%02X",
            pLineObj->registers.ringCal[0], pLineObj->registers.ringCal[1]);
        VpSysDebugPrintf("\n\rpLineObj->registers.batCal = 0x%02X 0x%02X",
            pLineObj->registers.batCal[0], pLineObj->registers.batCal[1]);
        VpSysDebugPrintf("\n\rpLineObj->registers.ringDelay = 0x%02X",
            pLineObj->registers.ringDelay[0]);

       VpSysDebugPrintf("\n\n\rpLineObj->debugSelectMask = 0x%08lX ", pLineObj->debugSelectMask);

    }

    VpSysDebugPrintf("\n\r");

    return VP_STATUS_SUCCESS;
}

#endif /* (VP_CC_DEBUG_SELECT & VP_DBG_INFO) */
#endif /* defined (VP_CC_886_SERIES) */

