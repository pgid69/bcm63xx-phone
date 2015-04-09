/** \file vp886_control_common.c
 * vp886_common.c
 *
 *  This file contains common internal functions for the Vp886 device API
 *  that don't fit in any of the other modules.
 *
 * Copyright (c) 2011, Microsemi Corporation
 *
 * $Revision: 11613 $
 * $LastChangedDate: 2014-10-20 17:36:07 -0500 (Mon, 20 Oct 2014) $
 */
#include "vp_api_cfg.h"

#if defined (VP_CC_886_SERIES)

/* INCLUDES */
#include "vp_api_types.h"
#include "vp_hal.h"
#include "vp_api_int.h"
#include "vp886_api.h"
#include "vp886_api_int.h"
#include "sys_service.h"

/* Table specifying the dimensions of the Profile Table
   Number of profiles is the only dimension for now, but we may add others
   such as minimum profile sizes. Maximum profile size was previously included,
   but was removed to allow expanding MPI sections without changing the API. */
const struct {
    int numProfiles;
} ProfileTableDims[VP886_NUM_PROFILE_TYPES] = {
    /* VP_PROFILE_DEVICE  */ { VP_CSLAC_DEV_PROF_TABLE_SIZE          },
    /* VP_PROFILE_AC      */ { VP_CSLAC_AC_PROF_TABLE_SIZE           },
    /* VP_PROFILE_DC      */ { VP_CSLAC_DC_PROF_TABLE_SIZE           },
    /* VP_PROFILE_RING    */ { VP_CSLAC_RINGING_PROF_TABLE_SIZE      },
    /* VP_PROFILE_RINGCAD */ { VP_CSLAC_RING_CADENCE_PROF_TABLE_SIZE },
    /* VP_PROFILE_TONE    */ { VP_CSLAC_TONE_PROF_TABLE_SIZE         },
    /* VP_PROFILE_METER   */ { VP_CSLAC_METERING_PROF_TABLE_SIZE     },
    /* VP_PROFILE_CID     */ { VP_CSLAC_CALLERID_PROF_TABLE_SIZE     },
    /* VP_PROFILE_TONECAD */ { VP_CSLAC_TONE_CADENCE_PROF_TABLE_SIZE }
};

/** Vp886GetProfileTableBaseAddress()
  Returns the profile table base address corresponding to profType.
*/
static VpProfilePtrType *
Vp886GetProfileTableBaseAddress(
    Vp886DeviceObjectType *pDevObj,
    VpProfileType profType)
{
    VpProfilePtrType *pProfTable;
    switch (profType) {
        case VP_PROFILE_DEVICE:
            pProfTable = pDevObj->devProfileTable.pDevProfileTable;
            break;
        case VP_PROFILE_AC:
            pProfTable = pDevObj->devProfileTable.pAcProfileTable;
            break;
        case VP_PROFILE_DC:
            pProfTable = pDevObj->devProfileTable.pDcProfileTable;
            break;
        case VP_PROFILE_RING:
            pProfTable = pDevObj->devProfileTable.pRingingProfileTable;
            break;
        case VP_PROFILE_RINGCAD:
            pProfTable = pDevObj->devProfileTable.pRingingCadProfileTable;
            break;
        case VP_PROFILE_TONE:
            pProfTable = pDevObj->devProfileTable.pToneProfileTable;
            break;
        case VP_PROFILE_METER:
            pProfTable = pDevObj->devProfileTable.pMeteringProfileTable;
            break;
        case VP_PROFILE_CID:
            pProfTable = pDevObj->devProfileTable.pCallerIdProfileTable;
            break;
        case VP_PROFILE_TONECAD:
            pProfTable = pDevObj->devProfileTable.pToneCadProfileTable;
            break;
        default:
            /* Should never happen. */
            pProfTable = (VpProfilePtrType *)0;
    }
    return pProfTable;
}

/** Vp886GetProfileArg()
  This function converts profile table index values into usable profile pointers
  and performs argument checking based on profile type and device type.
  
  If the ppProfileArg pointer refers to a profile table index value, that value
  will be modified to point to the actual profile in the profile table.
*/
VpStatusType
Vp886GetProfileArg(
    VpDevCtxType *pDevCtx,
    VpProfileType profType,
    VpProfilePtrType *ppProfileArg)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    VpProfilePtrType pProfile = *ppProfileArg;
    int profIndex = VpGetProfileIndex(pProfile);
    VpDeviceType profDeviceType;

    /* If profile argument is NULL, nothing to do. */
    if (pProfile == VP_PTABLE_NULL) {
        return VP_STATUS_SUCCESS;

    /* If profile argument is an index, get a pointer to the profile in the table. */
    } else if (profIndex != -1) {
        VpProfilePtrType *pProfTableBase = Vp886GetProfileTableBaseAddress(pDevObj, profType);
        if (profIndex >= ProfileTableDims[profType].numProfiles) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Profile table for %s profiles isn't large enough for VP_PTABLE_INDEX%d", VpGetString_VpProfileType(profType), profIndex + 1));
            return VP_STATUS_ERR_PROFILE;
        } else if (!(pDevObj->profTableEntryValid[profType] & (1 << profIndex))) {
            VP_ERROR(VpDevCtxType, pDevCtx, ("Profile table entry VP_PTABLE_INDEX%d for %s profiles is empty.", profIndex + 1, VpGetString_VpProfileType(profType)));
            return VP_STATUS_ERR_PROFILE;
        }
        pProfile = pProfTableBase[profIndex];
    }

    /* Check that the profile is of the correct type. */
    if (!VpVerifyProfileType(profType, pProfile)) {
        VP_ERROR(VpDevCtxType, pDevCtx, ("%s profile argument is incorrect type.", VpGetString_VpProfileType(profType)));
        return VP_STATUS_ERR_PROFILE;
    }

    /* Check that the profile is targeted for the correct type of device. */
    profDeviceType = (VpDeviceType)pProfile[VP_PROFILE_TYPE_MSB];
    switch (profType) {
        case VP_PROFILE_AC:
        case VP_PROFILE_RINGCAD:
        case VP_PROFILE_METER:
        case VP_PROFILE_CID:
        case VP_PROFILE_TONECAD:
            /* 886/887 are backward compatible with these CSLAC profiles */
            break;
        case VP_PROFILE_DEVICE:
        case VP_PROFILE_DC:
        case VP_PROFILE_RING:
        case VP_PROFILE_CAL:
            /* For these profiles, we need to make sure the device type in the
               profile is the same as the device type in the Device Object. */
            if (VP886_DEVICE_SERIES(pDevObj) != profDeviceType) {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Supplied %s profile was not intended for ZL88%cxx device.",
                    VpGetString_VpProfileType(profType), VP886_IS_ABS(pDevObj) ? '6' : '7'));
                return VP_STATUS_ERR_PROFILE;
            }
            break;
        case VP_PROFILE_TONE:
            if (pProfile[VP_PROFILE_VERSION] >= 2) {
                /* Tone Profile version 2+ includes the device type byte.  886
                   and 887 are compatible. */
                if (profDeviceType != VP_DEV_886_SERIES &&
                    profDeviceType != VP_DEV_887_SERIES)
                {
                    VP_ERROR(VpDevCtxType, pDevCtx, ("Supplied tone profile was not intended for ZL88%cxx device.",
                        VP886_IS_ABS(pDevObj) ? '6' : '7'));
                    return VP_STATUS_ERR_PROFILE;
                }
            } else {
                /* Allow backward compatibility with old version 0-1 profiles
                   that do not include the device type byte. */
            }
            break;
        default:
            /* Can't happen */
            break;
    }

    /* Profile argument is OK. */
    *ppProfileArg = pProfile;
    return VP_STATUS_SUCCESS;
}

/** Vp886SetProfTable()
  This function loads a profile into the profile table.
 
  Arguments:
   pProfile        -   profile to load into table
   pProfileIndex   -   table index
   profType        -   type of profile
*/
VpStatusType
Vp886SetProfTable(
    VpDevCtxType *pDevCtx,
    VpProfilePtrType pProfile,
    VpProfilePtrType pProfileIndex,
    VpProfileType profType)
{
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    uint16 tableSize = ProfileTableDims[profType].numProfiles;
    uint16 *pProfEntry = &pDevObj->profTableEntryValid[profType];
    VpProfilePtrType *pProfTable = Vp886GetProfileTableBaseAddress(pDevObj, profType);

    /* If the profile data is an index, indicated by Get Profile Index return
       value of > -1, return an error (cannot init an indexed entry with an
       index). */
    int profileIndex = VpGetProfileIndex(pProfile);
    if (profileIndex >= 0) {
        VP_ERROR(None, VP_NULL, ("Vp886SetProfTable() - invalid profIndex"));
        return VP_STATUS_INVALID_ARG;
    }

    /* If pProfileIndex is -1, the profile is of pointer type and invalid,
       otherwise it is an index.  If it's an index, make sure the range is
       valid. */
    profileIndex = VpGetProfileIndex(pProfileIndex);
    if ((profileIndex >= tableSize) || (profileIndex < 0)) {
        VP_ERROR(None, VP_NULL, ("Vp886SetProfTable() - profIndex exceeds table size"));
        return VP_STATUS_INVALID_ARG;
    }

    if (VpVerifyProfileType(profType, pProfile) == TRUE) {
        /*  If the profile is null, clear the flag  associated with it */
        if (pProfile == VP_PTABLE_NULL) {
            *pProfEntry &= ~(0x01 << profileIndex);
        } else {
            *pProfEntry |= (0x01 << profileIndex);
        }
        pProfTable[profileIndex] = pProfile;
        return VP_STATUS_SUCCESS;
    }
    VP_ERROR(None, VP_NULL, ("Vp886SetProfTable() - invalid profile type"));
    return VP_STATUS_ERR_PROFILE;
}


/** Vp886ReadyStatus()
  This function checks the state of the device and line against "busy"
  conditions.

  Arguments:
   pDevCtx   - Device to check for "busy", or VP_NULL if pLineCtx is set
   pLineCtx  - Line to check for "busy" (also checks the device), or VP_NULL
               if pDevCtx is set
   pStatus   - Returns a status code indicating why the function returned FALSE.
               Is not modified if this function returns TRUE.

  Returns
   TRUE  - If device and line are ready
   FALSE - If the device or line is busy being initialized or calibrated.
*/
bool
Vp886ReadyStatus(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    VpStatusType *pStatus)
{
    Vp886DeviceObjectType *pDevObj;
    if (pLineCtx != VP_NULL) {
        pDevCtx = pLineCtx->pDevCtx;
    }
    pDevObj = pDevCtx->pDevObj;

    /* Check for "device not ready" conditions. */
    if (pDevCtx != VP_NULL) {
        uint16 busyFlags = pDevObj->busyFlags;

        if (busyFlags & VP_TEMP_IGNORE_ALL_BUSY_FLAGS) {
            /* Skip device-specific busy checks. */
        } else if (busyFlags & VP_DEV_INIT_IN_PROGRESS) {
            *pStatus = VP_STATUS_DEV_NOT_INITIALIZED;
            if (pLineCtx != VP_NULL) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Device initialization in progress"));
            } else {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Device initialization in progress"));
            }
            return FALSE;
        } else if (!(busyFlags & VP_DEV_INIT_CMP)) {
            *pStatus = VP_STATUS_DEV_NOT_INITIALIZED;
            if (pLineCtx != VP_NULL) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Device not initialized"));
            } else {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Device not initialized"));
            }
            return FALSE;
        } else if (busyFlags & VP_DEV_IN_CAL) {
            *pStatus = VP_STATUS_DEVICE_BUSY;
            if (pLineCtx != VP_NULL) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("Device calibration in progress"));
            } else {
                VP_ERROR(VpDevCtxType, pDevCtx, ("Device calibration in progress"));
            }
            return FALSE;
        }
        
        if (pDevObj->spiError) {
            *pStatus = VP_STATUS_ERR_SPI;
            if (pLineCtx != VP_NULL) {
                VP_ERROR(VpLineCtxType, pLineCtx, ("SPI error detected. Device must be reinitialized."));
            } else {
                VP_ERROR(VpDevCtxType, pDevCtx, ("SPI error detected. Device must be reinitialized."));
            }
            return FALSE;
        }
    } else /* (pDevCtx == VP_NULL) */ {
        /* Should never happen.  Indicates API bug. */
        *pStatus = VP_STATUS_FAILURE;
        VP_WARNING(None, VP_NULL, ("Vp886ReadyStatus - pDevCtx NULL error"));
        return FALSE;
    }

    /* Check for "line not ready" conditions. */
    if (pLineCtx != VP_NULL) {
        Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
        uint16 busyFlags = pLineObj->busyFlags;

        if (pLineObj->channelId >= pDevObj->staticInfo.maxChannels) {
            *pStatus = VP_STATUS_INVALID_LINE;
            VP_ERROR(VpLineCtxType, pLineCtx, ("Channel ID %d exceeds device maximum", pLineObj->channelId));
            return FALSE;
        }
        if (busyFlags & VP886_IGNORE_ALL_BUSY_FLAGS) {
            /* Skip line-specific busy checks. */
        } else if (busyFlags & VP886_LINE_INIT_IN_PROGRESS) {
            *pStatus = VP_STATUS_DEVICE_BUSY;
            VP_ERROR(VpLineCtxType, pLineCtx, ("Line initialization in progress"));
            return FALSE;
        } else if (!(busyFlags & VP886_LINE_INIT_CMP)) {
            *pStatus = VP_STATUS_LINE_NOT_CONFIG;
            VP_ERROR(VpLineCtxType, pLineCtx, ("Line not initialized"));
            return FALSE;
        } else if (busyFlags & VP886_LINE_IN_CAL) {
            *pStatus = VP_STATUS_DEVICE_BUSY;
            VP_ERROR(VpLineCtxType, pLineCtx, ("Line calibration in progress"));
            return FALSE;
        }
        if (pLineObj->lineState.usrCurrent == VP_LINE_DISABLED) {
            *pStatus = VP_STATUS_LINE_NOT_CONFIG;
            VP_ERROR(VpLineCtxType, pLineCtx, ("Cannot access line in VP_LINE_DISABLED."));
            return FALSE;
        }
    }

    return TRUE;
} /* Vp886ReadyStatus() */


/** Vp886IsReady()
  Exists for backward compatibility with VeriVoice

  This function checks the state of the device and line against "busy"
  conditions.

  Arguments:
   pDevCtx   - Device to check for "busy", or VP_NULL if pLineCtx is set
   pLineCtx  - Line to check for "busy" (also checks the device), or VP_NULL
               if pDevCtx is set

  Returns
   TRUE  - If device and line are ready
   FALSE - If the device or line is busy being initialized or calibrated.
*/
bool
Vp886IsReady(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx)
{
    VpStatusType status = VP_STATUS_SUCCESS;
    return Vp886ReadyStatus(pDevCtx, pLineCtx, &status);
} /* Vp886IsReady() */


/** Vp886IsValidLineState()
  Returns TRUE or FALSE if a given line state is a valid input or not,
  respectively.
*/
bool
Vp886IsValidLineState(
    VpLineCtxType *pLineCtx,
    VpLineStateType state)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    Vp886DeviceObjectType *pDevObj = pLineCtx->pDevCtx->pDevObj;
    
    if (pLineObj->isFxs) {
        switch (state) {
            case VP_LINE_STANDBY:
            case VP_LINE_STANDBY_POLREV:
            case VP_LINE_TIP_OPEN:
            case VP_LINE_RING_OPEN:
            case VP_LINE_DISCONNECT:
            case VP_LINE_RINGING:
            case VP_LINE_RINGING_POLREV:
            case VP_LINE_ACTIVE:
            case VP_LINE_ACTIVE_POLREV:
            case VP_LINE_TALK:
            case VP_LINE_TALK_POLREV:
            case VP_LINE_OHT:
            case VP_LINE_OHT_POLREV:
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
            case VP_LINE_HOWLER:
            case VP_LINE_HOWLER_POLREV:
#endif /* VP_HIGH_GAIN_MODE_SUPPORTED */
                return TRUE;
            case VP_LINE_DISABLED:
                if (VP886_IS_TRACKER(pDevObj)) {
                    return TRUE;
                } else {
                    return FALSE;
                }
            default:
                return FALSE;
        }
    } else { /* FXO */
        return FALSE;
    }
}


/** Vp886LineStateInfo()
  Returns generic properties of the given line state that can be used to
  simplify decision making.  The goal of this function is to reduce the number
  of line state switch statements in the rest of the code.
*/
Vp886LineStateInfoType
Vp886LineStateInfo(
    VpLineStateType state)
{
    Vp886LineStateInfoType info;
    
    info.voice = FALSE;
    info.codec = FALSE;
    info.polrev = FALSE;
    info.normalEquiv = VP_LINE_DISCONNECT;
    info.polrevEquiv = VP_LINE_DISCONNECT;
    info.oppositeEquiv = VP_LINE_DISCONNECT;
    
    switch (state) {
        case VP_LINE_STANDBY:
            info.voice = FALSE;
            info.codec = FALSE;
            info.polrev = FALSE;
            info.normalEquiv = VP_LINE_STANDBY;
            info.polrevEquiv = VP_LINE_STANDBY_POLREV;
            info.oppositeEquiv = VP_LINE_STANDBY_POLREV;
            break;
        case VP_LINE_STANDBY_POLREV:
            info.voice = FALSE;
            info.codec = FALSE;
            info.polrev = TRUE;
            info.normalEquiv = VP_LINE_STANDBY;
            info.polrevEquiv = VP_LINE_STANDBY_POLREV;
            info.oppositeEquiv = VP_LINE_STANDBY;
            break;
        case VP_LINE_TIP_OPEN:
            info.voice = FALSE;
            info.codec = FALSE;
            info.polrev = FALSE;
            info.normalEquiv = VP_LINE_TIP_OPEN;
            info.polrevEquiv = VP_LINE_TIP_OPEN;
            info.oppositeEquiv = VP_LINE_TIP_OPEN;
            break;
        case VP_LINE_RING_OPEN:
            info.voice = FALSE;
            info.codec = FALSE;
            info.polrev = FALSE;
            info.normalEquiv = VP_LINE_RING_OPEN;
            info.polrevEquiv = VP_LINE_RING_OPEN;
            info.oppositeEquiv = VP_LINE_RING_OPEN;
            break;
        case VP_LINE_DISCONNECT:
            info.voice = FALSE;
            info.codec = FALSE;
            info.polrev = FALSE;
            info.normalEquiv = VP_LINE_DISCONNECT;
            info.polrevEquiv = VP_LINE_DISCONNECT;
            info.oppositeEquiv = VP_LINE_DISCONNECT;
            break;
        case VP_LINE_RINGING:
            info.voice = FALSE;
            info.codec = FALSE;
            info.polrev = FALSE;
            info.normalEquiv = VP_LINE_RINGING;
            info.polrevEquiv = VP_LINE_RINGING_POLREV;
            info.oppositeEquiv = VP_LINE_RINGING_POLREV;
            break;
        case VP_LINE_RINGING_POLREV:
            info.voice = FALSE;
            info.codec = FALSE;
            info.polrev = TRUE;
            info.normalEquiv = VP_LINE_RINGING;
            info.polrevEquiv = VP_LINE_RINGING_POLREV;
            info.oppositeEquiv = VP_LINE_RINGING;
            break;
        case VP_LINE_ACTIVE:
            info.voice = FALSE;
            info.codec = TRUE;
            info.polrev = FALSE;
            info.normalEquiv = VP_LINE_ACTIVE;
            info.polrevEquiv = VP_LINE_ACTIVE_POLREV;
            info.oppositeEquiv = VP_LINE_ACTIVE_POLREV;
            break;
        case VP_LINE_ACTIVE_POLREV:
            info.voice = FALSE;
            info.codec = TRUE;
            info.polrev = TRUE;
            info.normalEquiv = VP_LINE_ACTIVE;
            info.polrevEquiv = VP_LINE_ACTIVE_POLREV;
            info.oppositeEquiv = VP_LINE_ACTIVE;
            break;
        case VP_LINE_TALK:
            info.voice = TRUE;
            info.codec = TRUE;
            info.polrev = FALSE;
            info.normalEquiv = VP_LINE_TALK;
            info.polrevEquiv = VP_LINE_TALK_POLREV;
            info.oppositeEquiv = VP_LINE_STANDBY_POLREV;
            break;
        case VP_LINE_TALK_POLREV:
            info.voice = TRUE;
            info.codec = TRUE;
            info.polrev = TRUE;
            info.normalEquiv = VP_LINE_TALK;
            info.polrevEquiv = VP_LINE_TALK_POLREV;
            info.oppositeEquiv = VP_LINE_TALK;
            break;
        case VP_LINE_OHT:
            info.voice = TRUE;
            info.codec = TRUE;
            info.polrev = FALSE;
            info.normalEquiv = VP_LINE_OHT;
            info.polrevEquiv = VP_LINE_OHT_POLREV;
            info.oppositeEquiv = VP_LINE_OHT_POLREV;
            break;
        case VP_LINE_OHT_POLREV:
            info.voice = TRUE;
            info.codec = TRUE;
            info.polrev = TRUE;
            info.normalEquiv = VP_LINE_OHT;
            info.polrevEquiv = VP_LINE_OHT_POLREV;
            info.oppositeEquiv = VP_LINE_OHT;
            break;
        case VP_LINE_DISABLED:
            info.voice = FALSE;
            info.codec = FALSE;
            info.polrev = FALSE;
            info.normalEquiv = VP_LINE_DISABLED;
            info.polrevEquiv = VP_LINE_DISABLED;
            info.oppositeEquiv = VP_LINE_DISABLED;
            break;
#ifdef VP_HIGH_GAIN_MODE_SUPPORTED
        case VP_LINE_HOWLER:
            info.voice = FALSE;
            info.codec = TRUE;
            info.polrev = FALSE;
            info.normalEquiv = VP_LINE_HOWLER;
            info.polrevEquiv = VP_LINE_HOWLER_POLREV;
            info.oppositeEquiv = VP_LINE_HOWLER_POLREV;
            break;
        case VP_LINE_HOWLER_POLREV:
            info.voice = FALSE;
            info.codec = TRUE;
            info.polrev = TRUE;
            info.normalEquiv = VP_LINE_HOWLER;
            info.polrevEquiv = VP_LINE_HOWLER_POLREV;
            info.oppositeEquiv = VP_LINE_HOWLER;
            break;
#endif /* VP_HIGH_GAIN_MODE_SUPPORTED */
        default:
            VP_WARNING(None, VP_NULL, ("Vp886LineStateInfo - invalid line state %d", state));
            break;
    }
    
    return info;
}

/** ConvertSignedFixed2Csd()
  This function returns a four-nibble CSD (canonical signed digit) number
  whose value matches (as nearly as possible) the supplied signed 2.14
  fixed-point number.

  The CSD number will be placed into a two-byte array (high byte first) at
  the address specified in the csdBuf parameter.
*/
void
Vp886ConvertSignedFixed2Csd(
    int32 fixed,
    uint8 *csdBuf)
{
#define CSD_NIBBLES 4
    uint16 error, power, greaterPower, smallerPower, distGreater, distSmaller;
    uint16 C, m, result;
    int32 sum = 0;
    int8 n, gp, sp;

    /* Data structure for holding the four terms composing the CSD number. */
    typedef struct {
        bool sign;
        int power;
    } term;
    term t[CSD_NIBBLES + 1];

    t[0].power = 0;
    t[0].sign = 0;

    /* Split the 2.14 value into a sum of powers of 2,
         s1 * 2^p1  +  s2 * 2^p2  +  s3 * 2^p3  +  s4 * 2^p4
       where for term x,
         sx = 1 or -1,
         px <= 0.
    */
    for (n = 1; n <= CSD_NIBBLES; n++) {

        if (sum == fixed) break;

        /* If current sum is less than actual value, then the next term
           should be added; otherwise the next term should be
           subtracted. */
        if (sum < fixed) {
            t[n].sign = 0;
            error = fixed - sum;
        } else {
            t[n].sign = 1;
            error = sum - fixed;
        }

        /* If error > 1, then term = +/-1. */
        if (error > 0x4000) {
            t[n].power = 0;
        } else {

            /* Calculate greaterPower = the smallest power of 2 greater
               than error.  Calculate smallerPower = the largest power
               of 2 less than error. */
            greaterPower = 0x4000; gp = 0;
            for (power = 0x2000; power > error; power >>= 1) {
                greaterPower >>= 1; gp--;
            }
            smallerPower = greaterPower >> 1; sp = gp - 1;

            /* Is error closer to greaterPower or smallerPower?
               Whichever is closer, choose that for the value of the
               next term. */
            distGreater = greaterPower - error;
            distSmaller = error - smallerPower;
            if (distGreater < distSmaller) {
                t[n].power = gp;
            } else {
                t[n].power = sp;
            }

            /* The power of this term can differ from the power of the
               previous term by no more than 7. */
            if (t[n - 1].power - t[n].power > 7) {
                t[n].power = t[n - 1].power - 7;
            }
        }

        /* Add or subtract the term to the sum, depending on sign. */
        if (t[n].sign == 0) {
            sum += (uint16)1 << (14 + t[n].power);
        } else {
            sum -= (uint16)1 << (14 + t[n].power);
        }
    }

    /* If we reached the exact value with terms left over, fill these
       extra terms with dummy values which don't affect the CSD value. */
    while (n <= CSD_NIBBLES) {
        if (n == 1) {
            t[1] = t[0];
            t[2].power = 0;
            t[2].sign = 1;
            n += 2;
        } else {
            /* Increase the number of terms by replacing the last term
               with two new terms whose sum is the old term. */
            if (t[n - 1].power == t[n - 2].power) {
                t[n - 1].power--;
                t[n] = t[n - 1];
            } else {
                t[n] = t[n - 1];
                t[n - 1].power++;
                t[n].sign = !(t[n - 1].sign);
            }
            n++;
        }
    }

    /* Compute nibble values from the terms. */
    result = 0;
    for (n = 1; n <= CSD_NIBBLES; n++) {
        int8 bitPos = (n - 1) * 4;
        C = (t[n].sign != t[n - 1].sign);
        m = -(t[n].power - t[n - 1].power);
        result |= (C << (bitPos + 3)) | (m << bitPos);
    }

    /* Split the uint16 result into high and low bytes. */
    csdBuf[0] = (uint8)(result >> 8);
    csdBuf[1] = (uint8)(result & 0xFF);
} /* Vp886ConvertFixed2Csd() */


/**  Vp886EnterCritical()
  Wrapper function for VpSysEnterCritical() that also performs other actions
  that we want to surround each API call.

  This function accepts either device or line context.  If the line context is
  provided, the FUNC_INT debug message will be done for the line.  Otherwise the
  debug message is done at the device level, and other operation is the same.
  If funcName is VP_NULL, the FUNC_INT message is skipped.

  Operations:
   - Handles FUNC_INT debug messages
   - Calls VpSysEnterCritical()
   - Checks for critical section nesting
   - Invalidates the saved timestamp to ensure we update it before using it next
   - Starts the MPI command buffer

  This function should be the only place in the Vp886 API where VpSysEnterCritical
  is called.
*/
void
Vp886EnterCritical(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    char* funcName)
{
    Vp886DeviceObjectType *pDevObj;
    VpDeviceIdType deviceId;

    if (pDevCtx == VP_NULL) {
        pDevCtx = pLineCtx->pDevCtx;
    }
    pDevObj = pDevCtx->pDevObj;
    deviceId = pDevObj->deviceId;
    
    if (funcName != VP_NULL) {
        if (pLineCtx != VP_NULL) {
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("%s+", funcName));
        } else {
            VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("%s+", funcName));
        }
    }
    
    VpSysEnterCritical(deviceId, VP_CODE_CRITICAL_SEC);
    
    pDevObj->criticalDepth++;
    if (pDevObj->criticalDepth > 1) {
        VP_WARNING(VpDevCtxType, pDevCtx, ("Nested critical section, %s, depth %d",
            funcName != VP_NULL ? funcName : "", pDevObj->criticalDepth));
    }
    
    /* Shouldn't need to do this here, but just to be safe.. */
    pDevObj->timestampValid = FALSE;
    
    /* Start the SLAC Write Buffer. */
    if (!VpSlacBufStart(pDevCtx)) {
        VP_WARNING(VpDevCtxType, pDevCtx, ("Failed VpSlacBufStart, %s",
            funcName != VP_NULL ? funcName : ""));
    }
    
    return;
}

/**  Vp886ExitCritical()
  Wrapper function for VpSysExitCritical() that also performs other actions
  that we want to surround each API call.

  This function accepts either device or line context.  If the line context is
  provided, the FUNC_INT debug message will be done for the line.  Otherwise the
  debug message is done at the device level, and other operation is the same.
  If funcName is VP_NULL, the FUNC_INT message is skipped.

  Operations:
   - Invalidates the saved timestamp to ensure we update it before using it next
   - Sends the MPI command buffer
   - Checks for critical section mismatches
   - Calls VpSysExitCritical()
   - Handles FUNC_INT debug messages

  This function should be the only place in the Vp886 API where VpSysExitCritical
  is called.
*/
void
Vp886ExitCritical(
    VpDevCtxType *pDevCtx,
    VpLineCtxType *pLineCtx,
    char* funcName)
{
    Vp886DeviceObjectType *pDevObj;
    VpDeviceIdType deviceId;

    if (pDevCtx == VP_NULL) {
        pDevCtx = pLineCtx->pDevCtx;
    }
    pDevObj = pDevCtx->pDevObj;
    deviceId = pDevObj->deviceId;
    
    /* Cached timestamp is invalid until after the first timestamp register read
       during this critical section */
    pDevObj->timestampValid = FALSE;
    
    /* Send the SLAC Write Buffer. */
    VpSlacBufSend(pDevCtx);

    if (pDevObj->ecVal != VP886_EC_GLOBAL) {
        VP_WARNING(VpDevCtxType, pDevCtx, ("pDevObj->ecVal was not reset during %s",
            funcName != VP_NULL ? funcName : ""));
        pDevObj->ecVal = VP886_EC_GLOBAL;
    }
    
    if (pDevObj->criticalDepth == 0) {
        VP_WARNING(VpDevCtxType, pDevCtx, ("Too many Vp886ExitCritical calls, depth cannot go lower (%s)",
            funcName != VP_NULL ? funcName : ""));
    } else {
        pDevObj->criticalDepth--;
    }

    VpSysExitCritical(deviceId, VP_CODE_CRITICAL_SEC);

    if (funcName != VP_NULL) {
        if (pLineCtx != VP_NULL) {
            VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("%s-", funcName));
        } else {
            VP_API_FUNC_INT(VpDevCtxType, pDevCtx, ("%s-", funcName));
        }
    }
    
    return;
}


void
Vp886RetrieveRawSadcData(
    VpLineCtxType *pLineCtx,
    int16 *pBuffer,
    uint8 *pLength,
    uint8 *pOverflow,
    bool applyCal)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    int16 gain = pDevObj->calData[channelId].cmn.sadc.gain;
    int16 offset = pDevObj->calData[channelId].cmn.sadc.offset;
    uint8 buf[VP886_R_B1_LEN];
    uint8 allData[120];
    uint8 i;

    /* Read the first buffer to find the overflow status, number of valid
       samples, and the first set of samples. */
    VpSlacRegRead(NULL, pLineCtx, VP886_R_B1_RD, VP886_R_B1_LEN, buf);
    
    *pOverflow = (buf[0] & VP886_R_B1_OVFL) >> 6;
    *pLength = buf[0] & VP886_R_B1_PTR;
    
    if (*pLength > 60) {
        VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886RetrieveRawSadcData(): Invalid length %d", *pLength));
        *pLength = 0;
        *pOverflow = 0;
        return;
    }
    
    VpMemCpy(&allData[0], &buf[1], 24);
    
    /* Read more samples from the other buffer registers as necessary */
    if (*pLength > 12) {
        VpSlacRegRead(NULL, pLineCtx, VP886_R_B2_RD, VP886_R_B2_LEN, buf);
        VpMemCpy(&allData[24], &buf[1], 24);
    }
    if (*pLength > 24) {
        VpSlacRegRead(NULL, pLineCtx, VP886_R_B3_RD, VP886_R_B3_LEN, buf);
        VpMemCpy(&allData[48], &buf[1], 24);
    }
    if (*pLength > 36) {
        VpSlacRegRead(NULL, pLineCtx, VP886_R_B4_RD, VP886_R_B4_LEN, buf);
        VpMemCpy(&allData[72], &buf[1], 24);
    }
    if (*pLength > 48) {
        VpSlacRegRead(NULL, pLineCtx, VP886_R_B5_RD, VP886_R_B5_LEN, buf);
        VpMemCpy(&allData[96], &buf[1], 24);
    }
    
    /* Copy the raw data into the int16 buffer */
    for (i = 0; i < *pLength; i++) {
        int32 sample;
        sample = (int16)((allData[i*2] << 8) | allData[i*2+1]);
        if (applyCal) {
            sample = VpRoundedDivide(((int32)sample + (int32)offset) * (int32)gain, 1000L);
            sample = MIN(sample, VP_INT16_MAX);
            sample = MAX(sample, VP_INT16_MIN);
        }
        pBuffer[i] = (int16)sample;
    }
    
    return;
}


void
Vp886RetrieveRawVadcData(
    VpLineCtxType *pLineCtx,
    int16 *pBuffer,
    uint8 *pLength,
    uint8 *pOverflow,
    bool applyCal)
{
    VpDevCtxType *pDevCtx = pLineCtx->pDevCtx;
    Vp886DeviceObjectType *pDevObj = pDevCtx->pDevObj;
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;
    uint8 channelId = pLineObj->channelId;
    int16 gain = pDevObj->calData[channelId].cmn.vadcActive.gain;
    int16 offset = pDevObj->calData[channelId].cmn.vadcActive.offset;
    uint8 buf[VP886_R_VBUFFER_LEN];
    uint8 i;

    /* Read the first buffer to find the overflow status, number of valid
       samples, and the first set of samples. */
    VpSlacRegRead(NULL, pLineCtx, VP886_R_VBUFFER_RD, VP886_R_VBUFFER_LEN, buf);
    
    *pOverflow = (buf[0] & VP886_R_VBUFFER_OVERFLOW) >> 6;
    *pLength = buf[0] & VP886_R_VBUFFER_POINTER;

    if (*pLength > 12) {
        VP_WARNING(VpLineCtxType, pLineCtx, ("Vp886RetrieveRawVadcData(): Invalid length %d", *pLength));
        *pLength = 0;
        *pOverflow = 0;
        return;
    }

    /* Copy the raw data into the int16 buffer */
    for (i = 0; i < *pLength; i++) {
        int32 sample;
        sample = (int16)((buf[i*2+1] << 8) | buf[i*2+2]);
        if (applyCal) {
            sample = VpRoundedDivide(((int32)sample + (int32)offset) * (int32)gain, 1000L);
            sample = MIN(sample, VP_INT16_MAX);
            sample = MAX(sample, VP_INT16_MIN);
        }
        pBuffer[i] = (int16)sample;
    }
    
    return;
}

#ifdef VP886_INCLUDE_TESTLINE_CODE
/**  Vp886TestPrepareCleanup()
  Clean up actions that the API is performing before line test starts, such as
  timers that would cause problems for the test when they expire, or anything
  using resources required for line test.
  This is separate from the VeriVoice code so that we can update it for new API
  features without requiring a VeriVoice update.
  The pLineObj->inLineTest flag should be set before calling this function.
*/
void
Vp886TestPrepareCleanup(
    VpLineCtxType *pLineCtx)
{
    Vp886LineObjectType *pLineObj = pLineCtx->pLineObj;

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886TestPrepareCleanup+"));

    /* For intrusive tests, cancel any ring sync that may be in progress.
       Non-intrusive tests will just let it run. */
    if (!pLineObj->testInfo.nonIntrusiveTest) {
        Vp886RingSyncCancel(pLineCtx);
    }

#ifdef VP886_INCLUDE_DTMF_DETECT
    /* Suspend DTMF detection if it is running. */
    Vp886DtmfManage(pLineCtx);
#endif

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886TestPrepareCleanup-"));
    return;
}

/**  Vp886TestConcludeCleanup()
  Resume actions that the API was performing before line test started which
  were suspended during testing.
  This is separate from the VeriVoice code so that we can update it for new API
  features without requiring a VeriVoice update.
  The pLineObj->inLineTest flag should be cleared before calling this function.
*/
void
Vp886TestConcludeCleanup(
    VpLineCtxType *pLineCtx)
{
    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886TestConcludeCleanup+"));

#ifdef VP886_INCLUDE_DTMF_DETECT
    /* Resume DTMF detection if needed. */
    Vp886DtmfManage(pLineCtx);
#endif

    VP_API_FUNC_INT(VpLineCtxType, pLineCtx, ("Vp886TestConcludeCleanup-"));
    return;
}
#endif /* VP886_INCLUDE_TESTLINE_CODE */

#endif /* defined (VP_CC_886_SERIES) */
