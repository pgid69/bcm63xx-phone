/*
 * VE880_ABS_profiles.h --
 *
 * This header file exports the Profile data types
 *
 * Project Info --
 *   File:   X:\shafer\lightning\ztap2012\ztap\applications\quickstarts\sdk_examples\VE880_initialization\VE880_ABS_profiles.vpw
 *   Type:   VE880 ABS Configuration for -27/-81V Supply with VBL Regulation, Lite Narrowband Coefficients and PCM/MPI Interface at 8.192MHz PCLK
 *   Date:   Tuesday, August 07, 2012 11:43:00
 *   Device: VE880 Le88266
 *
 *   This file was generated with Profile Wizard Version: P2.3.0
 */

#ifndef VE880_ABS_PROFILES_H
#define VE880_ABS_PROFILES_H

#ifdef VP_API_TYPES_H
#include "vp_api_types.h"
#else
typedef unsigned char VpProfileDataType;
#endif


/************** Device Parameters **************/
extern const VpProfileDataType ABS_DEV_PROFILE[];    /* Device Configuration Data - VE880 ABS -27/-81V Power Optimized With Free Run, PCM/MPI Interface */

/************** AC Filter Coefficients **************/
extern const VpProfileDataType ABS_AC_600_FXS[];     /* AC FXS RF14 600R Normal Coefficients (Default)  */

/************** DC Feed Parameters **************/
extern const VpProfileDataType ABS_DC_22MA_CC[];     /* DC FXS Default -- Use for all countries unless country file exists - 22 mA  current feed */

/************** Ring Signal Parameters **************/
extern const VpProfileDataType ABS_RING_25HZ_SINE[]; /* Default Ringing 25Hz 50 Vrms Fixed - Use for all countries unless country file exists */

/************** Call Progress Tones **************/

/************** Cadence Definitions **************/

/************** Caller ID **************/

/************** Metering Profile **************/

#endif /* VE880_ABS_PROFILES_H */

