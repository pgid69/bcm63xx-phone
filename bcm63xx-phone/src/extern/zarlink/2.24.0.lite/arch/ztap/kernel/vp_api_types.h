/** \file vp_api_types.h
 * vp_api_types.h
 *
 *  This file is the header for all standard types used in the API code.
 *
 * Copyright (c) 2011, Microsemi Corporation
 */
#ifndef VP_API_TYPES_H
#define VP_API_TYPES_H

#include "vp_api_profile_type.h"
/* For maximum that can be stored in an int - if file exists in library */

/*
 * MODULE is a Linux 2.6 Module Make Flag, I am using it to determine if
 * we are compiling as a module or as an application
 */
#ifdef MODULE
  #include <linux/kernel.h>
  #include <linux/version.h>
#else
  #include <limits.h>
#endif
/* VpDeviceIdType defines the type for the deviceId in the VpDevCtxType type.
 * This information is passed through the API to the HAL to communicate
 * with a specific device.  The values assigned via VpMakeDeviceObject()
 * are user defined and may be simple device indexing (0, 1, .. (n-1)):
 * where n = device number in system
 */
typedef unsigned short VpDeviceIdType;
/*
 * The 'VpLineIdType'  defines a system wide Line identification that the system
 * could use to identify a line. This type can be defined to contain anything
 * that the customer chooses. It could be defined to contain just an index or
 * a pointer. The system wide line identity could be set using the
 * VpMapLineId() function for a given line. The VP-API returns this line id
 * information when line specific events occur (along with the event).
 */
typedef unsigned short VpLineIdType;

/*
 * Macros for displaying VpDeviceIdType and VpLineIdType values.  If you have
 * defined these (in vp_api_types.h) as something other than simple integers,
 * you should modify the printf format strings as needed:
 */
#define VP_PRINT_DEVICE_ID(deviceId)  VpSysDebugPrintf(" (dev 0x%2.2X)", (int)deviceId)
#define VP_PRINT_LINE_ID(lineId)      VpSysDebugPrintf(" (line %d)", (int)lineId)

#ifndef NULL
    #define NULL (0)
#endif
#define VP_NULL NULL
#ifdef EXTERN
  #undef EXTERN
  #error EXTERN was redefined!
#endif /* undef EXTERN */

#ifdef __cplusplus
  #define EXTERN extern "C"
#else
    #define EXTERN extern
#endif /* __cplusplus */
/********************* DECLARATIONS ***************************/
/* Constants */
#ifndef FALSE
  #define FALSE   (0)     /* Boolean constant */
#endif

#ifndef TRUE
  #define TRUE    (1)     /* Boolean constant */
#endif

#ifndef __cplusplus
/* C++ language provides a boolean data type; So no need to define
 * one more data type; Make use of it
 * NOTE: The 'C' potions of the VP-API assume C++ "bool" to be of the
 * same size as that of "char". Please make sure this assumption is correct.
 */

  #ifdef MODULE
    #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,22)
      typedef unsigned char bool;
    #endif
  #else
    typedef unsigned char bool;
  #endif

#endif /* __cplusplus */
/****************** typedefs ***********************************/
/* These are the basic number types used */
/* for uint8, uint16, uint32, int8, int16, int32, bool */
typedef unsigned char   uchar;
typedef unsigned char   uint8;
typedef unsigned short  uint16;
typedef unsigned long   uint32;
typedef signed char    int8;
typedef signed short int int16;
typedef signed long  int32;
typedef   uint8*  uint8p;
typedef   uint16* uint16p;
typedef   uint32* uint32p;
typedef   int8*   int8p;
typedef   int16*  int16p;
typedef   int32*  int32p;
typedef const VpProfileDataType * VpProfilePtrType;
typedef uint8p VpImagePtrType;
typedef uint16p VpVectorPtrType;
typedef uint8 VpPktDataType;
typedef VpPktDataType* VpPktDataPtrType;
/* Some compilers optimize the size of enumeration data types based on
 * the maximum data value assigned to the members of that data type.
 * 'Standard C' requires enumeration data types to be of the same size
 * as that of native 'int' implementation.
 * The VP-API from a portability persepective adds a 'dummy' member to
 * all enumeration data types that force the compilers to allocate the size
 * of enumeration data types to be equal to that of native 'int'
 * implementation */
#define FORCE_STANDARD_C_ENUM_SIZE  (INT_MAX)

/* Eliminate error messages that occur when comparing an enumeration constant
   < 0 */
#define FORCE_SIGNED_ENUM  (INT_MIN)


/* Define any API specific basic data type ranges (that are necessary) */
#ifdef MODULE
  #define VP_INT16_MAX    ((short)(((unsigned short)(~0U))>>1))
  #define VP_INT16_MIN    ((-VP_INT16_MAX)-1)
#else
  #define VP_INT16_MAX    (SHRT_MAX)
  #define VP_INT16_MIN    (SHRT_MIN)
#endif

#define VP_INT32_MAX    (LONG_MAX)
#define VP_INT32_MIN    (LONG_MIN)

#endif /* VP_API_TYPES_H */

