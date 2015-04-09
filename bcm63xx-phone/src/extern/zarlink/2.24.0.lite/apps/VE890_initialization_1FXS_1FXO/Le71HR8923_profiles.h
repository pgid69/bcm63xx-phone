/*
 * Le71HR8923_profiles.h --
 *
 * This header file exports the Profile data types
 *
 * Project Info --
 *   Type:   Custom Design Using Zarlink VE890 - Le89336
 *   Date:   Thursday, March 29, 2012 08:41:18
 *   Device: VE890 Le89336
 *
 *   This file was generated with Profile Wizard Version: B2.0.0-RC1
 */

#ifndef LE71HR8923_REVA_PROFILES_H
#define LE71HR8923_REVA_PROFILES_H

#ifdef VP_API_TYPES_H
#include "vp_api_types.h"
#else
typedef unsigned char VpProfileDataType;
#endif


/************** Device_Parameters **************/
extern const VpProfileDataType DEV_PROFILE[];        /* VE8911 Inverting Boost (100V) Device Profile */

/************** AC_Coefficients **************/
extern const VpProfileDataType AC_FXS_600R[];        /* AC FXS RF50 600R Normal Coefficients (Default)  */
extern const VpProfileDataType AC_FXO_LC[];          /* AC FXO LC filter 600 Ohm Normal Coefficients (Default) */

/************** DC_Coefficients **************/
extern const VpProfileDataType DC_25MA_CC[];         /* DC FXS Default -- Use for for all countries unless country file exists */

/************** Ring_Parameters **************/
extern const VpProfileDataType RING_25HZ_SINE[];     /* Ringing 25 Hz 60 Vrms Tracking */

/************** Call_Progress_Tones **************/

/************** Cadence_Definitions **************/

/************** Caller_ID **************/

/************** FXO_Dialing_Profile **************/
extern const VpProfileDataType FXO_DIALING_DEF[];    /* Default FXO/Dialing */
#endif /* LE71HR892_PROFILES_H */

