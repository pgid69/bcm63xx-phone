#include <stdio.h>
#include "sdk_qs_utils.h"

/**
 * PrintEvent(VpEventType *pEvent)
 *
 * Description: This function prints all the events
 * for debugging purpose.
 */

void
UtilPrintEvent(
    VpEventType *pEvent)
{
    VpEventCategoryType eventCategory = pEvent->eventCategory;
    uint16 eventId = pEvent->eventId;

    char *eventCategorySt[] = {
        "VP_EVCAT_FAULT",
        "VP_EVCAT_SIGNALING",
        "VP_EVCAT_RESPONSE",
        "VP_EVCAT_TEST",
        "VP_EVCAT_PROCESS",
        "VP_EVCAT_FXO",
        "VP_EVCAT_PACKET",
    };

    if (eventCategory < VP_NUM_EVCATS) {
        QS_DEBUG("\t\teventCategory = %s\n", eventCategorySt[eventCategory]);
    } else {
        QS_DEBUG("\t\teventCategory = INVALID (0x%x)\n", eventCategory);
    }

    QS_DEBUG("\t\teventId = ");
    if (eventCategory == VP_EVCAT_FAULT) {
        switch (eventId) {
            case VP_DEV_EVID_BAT_FLT:
                QS_DEBUG("VP_DEV_EVID_BAT_FLT\n"); break;
            case VP_DEV_EVID_CLK_FLT:
                QS_DEBUG("VP_DEV_EVID_CLK_FLT\n"); break;
            case VP_LINE_EVID_THERM_FLT:
                QS_DEBUG("VP_LINE_EVID_THERM_FLT\n"); break;
            case VP_LINE_EVID_DC_FLT:
                QS_DEBUG("VP_LINE_EVID_DC_FLT\n"); break;
            case VP_LINE_EVID_AC_FLT:
                QS_DEBUG("VP_LINE_EVID_AC_FLT\n"); break;
            case VP_DEV_EVID_EVQ_OFL_FLT:
                QS_DEBUG("VP_DEV_EVID_EVQ_OFL_FLT\n"); break;
            case VP_DEV_EVID_WDT_FLT:
                QS_DEBUG("VP_DEV_EVID_WDT_FLT\n"); break;
            default:
                QS_DEBUG("INVALID (0x%x)\n", eventId); break;
        }
    } else if (eventCategory == VP_EVCAT_SIGNALING) {
        switch (eventId) {
            case VP_LINE_EVID_HOOK_OFF:
                QS_DEBUG("VP_LINE_EVID_HOOK_OFF\n"); break;
            case VP_LINE_EVID_HOOK_ON:
                QS_DEBUG("VP_LINE_EVID_HOOK_ON\n"); break;
            case VP_LINE_EVID_GKEY_DET:
                QS_DEBUG("VP_LINE_EVID_GKEY_DET\n"); break;
            case VP_LINE_EVID_GKEY_REL:
                QS_DEBUG("VP_LINE_EVID_GKEY_REL\n"); break;
            case VP_LINE_EVID_FLASH:
                QS_DEBUG("VP_LINE_EVID_FLASH\n"); break;
            case VP_LINE_EVID_STARTPULSE:
                QS_DEBUG("VP_LINE_EVID_STARTPULSE\n"); break;
            case VP_LINE_EVID_DTMF_DIG:
                QS_DEBUG("VP_LINE_EVID_DTMF_DIG\n"); break;
            case VP_LINE_EVID_PULSE_DIG:
                QS_DEBUG("VP_LINE_EVID_PULSE_DIG\n"); break;
            case VP_LINE_EVID_MTONE:
                QS_DEBUG("VP_LINE_EVID_MTONE\n"); break;
            case VP_DEV_EVID_TS_ROLLOVER:
                QS_DEBUG("VP_DEV_EVID_TS_ROLLOVER\n"); break;
            default:
                QS_DEBUG("INVALID (0x%x)\n", eventId); break;
        }
    } else if (eventCategory == VP_EVCAT_RESPONSE) {
        switch (eventId) {
            case VP_DEV_EVID_BOOT_CMP:
                QS_DEBUG("VP_DEV_EVID_BOOT_CMP\n"); break;
            case VP_LINE_EVID_LLCMD_TX_CMP:
                QS_DEBUG("VP_LINE_EVID_LLCMD_TX_CMP\n"); break;
            case VP_LINE_EVID_LLCMD_RX_CMP:
                QS_DEBUG("VP_LINE_EVID_LLCMD_RX_CMP\n"); break;
            case VP_DEV_EVID_DNSTR_MBOX:
                QS_DEBUG("VP_DEV_EVID_DNSTR_MBOX\n"); break;
            case VP_LINE_EVID_RD_OPTION:
                QS_DEBUG("VP_LINE_EVID_RD_OPTION\n"); break;
            case VP_LINE_EVID_RD_LOOP:
                QS_DEBUG("VP_LINE_EVID_RD_LOOP\n"); break;
            case VP_EVID_CAL_CMP:
                QS_DEBUG("VP_EVID_CAL_CMP\n"); break;
            case VP_EVID_CAL_BUSY:
                QS_DEBUG("VP_EVID_CAL_BUSY\n"); break;
            case VP_LINE_EVID_GAIN_CMP:
                QS_DEBUG("VP_LINE_EVID_GAIN_CMP\n"); break;
            case VP_DEV_EVID_DEV_INIT_CMP:
                QS_DEBUG("VP_DEV_EVID_DEV_INIT_CMP\n"); break;
            case VP_LINE_EVID_LINE_INIT_CMP:
                QS_DEBUG("VP_LINE_EVID_LINE_INIT_CMP\n"); break;
            case VP_DEV_EVID_IO_ACCESS_CMP:
                QS_DEBUG("VP_DEV_EVID_IO_ACCESS_CMP\n"); break;
            default:
                QS_DEBUG("INVALID (0x%x)\n", eventId); break;
        }
    } else if (eventCategory == VP_EVCAT_TEST) {
        switch (eventId) {
            case VP_LINE_EVID_TEST_CMP:
                QS_DEBUG("VP_LINE_EVID_TEST_CMP\n"); break;
            case VP_LINE_EVID_DTONE_DET:
                QS_DEBUG("VP_LINE_EVID_DTONE_DET\n"); break;
            case VP_LINE_EVID_DTONE_LOSS:
                QS_DEBUG("VP_LINE_EVID_DTONE_LOSS\n"); break;
            case VP_DEV_EVID_STEST_CMP:
                QS_DEBUG("VP_DEV_EVID_STEST_CMP\n"); break;
            case VP_DEV_EVID_CHKSUM:
                QS_DEBUG("VP_DEV_EVID_CHKSUM\n"); break;
            default:
                QS_DEBUG("INVALID (0x%x)\n", eventId); break;
        }
    } else if(eventCategory == VP_EVCAT_PROCESS) {
        switch (eventId) {
            case VP_LINE_EVID_MTR_CMP:
                QS_DEBUG("VP_LINE_EVID_MTR_CMP\n"); break;
            case VP_LINE_EVID_MTR_ABORT:
                QS_DEBUG("VP_LINE_EVID_MTR_ABORT\n"); break;
            case VP_LINE_EVID_CID_DATA:
                QS_DEBUG("VP_LINE_EVID_CID_DATA\n"); break;
            case VP_LINE_EVID_RING_CAD:
                QS_DEBUG("VP_LINE_EVID_RING_CAD\n"); break;
            case VP_LINE_EVID_GEN_TIMER:
                QS_DEBUG("VP_LINE_EVID_GEN_TIMER\n"); break;
            default:
                QS_DEBUG("INVALID (0x%x)\n", eventId); break;
        }
    } else if(eventCategory == VP_EVCAT_FXO) {
        switch (eventId) {
            case VP_LINE_EVID_RING_ON:
                QS_DEBUG("VP_LINE_EVID_RING_ON\n"); break;
            case VP_LINE_EVID_RING_OFF:
                QS_DEBUG("VP_LINE_EVID_RING_OFF\n"); break;
            case VP_LINE_EVID_LIU:
                QS_DEBUG("VP_LINE_EVID_LIU\n"); break;
            case VP_LINE_EVID_LNIU:
                QS_DEBUG("VP_LINE_EVID_LNIU\n"); break;
            case VP_LINE_EVID_FEED_DIS:
                QS_DEBUG("VP_LINE_EVID_FEED_DIS\n"); break;
            case VP_LINE_EVID_FEED_EN:
                QS_DEBUG("VP_LINE_EVID_FEED_EN\n"); break;
            case VP_LINE_EVID_DISCONNECT:
                QS_DEBUG("VP_LINE_EVID_DISCONNECT\n"); break;
            case VP_LINE_EVID_RECONNECT:
                QS_DEBUG("VP_LINE_EVID_RECONNECT\n"); break;
            case VP_LINE_EVID_POLREV:
                QS_DEBUG("VP_LINE_EVID_POLREV\n"); break;
            case VP_LINE_EVID_POH:
                QS_DEBUG("VP_LINE_EVID_POH\n"); break;
            case VP_LINE_EVID_PNOH:
                QS_DEBUG("VP_LINE_EVID_PNOH\n"); break;
            default:
                QS_DEBUG("INVALID (0x%x)\n", eventId); break;
        }
    } else {
        QS_DEBUG("0x%x\n", eventId);
    }
}


char *
MapStatus(
    VpStatusType status)
{
    static const char *strTable[VP_STATUS_NUM_TYPES] = {
        "VP_STATUS_SUCCESS",
        "VP_STATUS_FAILURE",
        "VP_STATUS_FUNC_NOT_SUPPORTED",
        "VP_STATUS_INVALID_ARG",
        "VP_STATUS_MAILBOX_BUSY",
        "VP_STATUS_ERR_VTD_CODE",
        "VP_STATUS_OPTION_NOT_SUPPORTED",
        "VP_STATUS_ERR_VERIFY",
        "VP_STATUS_DEVICE_BUSY",
        "VP_STATUS_MAILBOX_EMPTY",
        "VP_STATUS_ERR_MAILBOX_DATA",
        "VP_STATUS_ERR_HBI",
        "VP_STATUS_ERR_IMAGE",
        "VP_STATUS_IN_CRTCL_SECTN",
        "VP_STATUS_DEV_NOT_INITIALIZED",
        "VP_STATUS_ERR_PROFILE",
        "VP_STATUS_INVALID_VOICE_STREAM",
        "VP_STATUS_CUSTOM_TERM_NOT_CFG",
        "VP_STATUS_DEDICATED_PINS",
        "VP_STATUS_INVALID_LINE"
    };

    if (status >= VP_STATUS_NUM_TYPES) {
        static char buff[5];
        sprintf(buff, "(%d)", (uint16)status);
        return buff;
    } else {
        return (char *)strTable[status];
    }
}
