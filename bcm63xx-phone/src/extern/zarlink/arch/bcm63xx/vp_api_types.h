/** \file vp_api_types.h
 * vp_api_types.h
 *
 *  This file is the header for all standard types used in the API code.
 *
 * Copyright (c) 2011, Microsemi Corporation
 */
#ifndef VP_API_TYPES_H
#define VP_API_TYPES_H

#include <linux/types.h>
#include <mpi.h>

typedef struct {
   bcm_mpi_t *mpi;
} zarlink_device_id_t;

#include "vp_api_profile_type.h"
/* VpDeviceIdType defines the type for the deviceId in the VpDevCtxType type.
 * This information is passed through the API to the HAL to communicate
 * with a specific device.  The values assigned via VpMakeDeviceObject()
 * are user defined and may be simple device indexing (0, 1, .. (n-1)):
 * where n = device number in system
 */
typedef const zarlink_device_id_t *VpDeviceIdType;
/*
 * The 'VpLineIdType'  defines a system wide Line identification that the system
 * could use to identify a line. This type can be defined to contain anything
 * that the customer chooses. It could be defined to contain just an index or
 * a pointer. The system wide line identity could be set using the
 * VpMapLineId() function for a given line. The VP-API returns this line id
 * information when line specific events occur (along with the event).
 */
typedef size_t VpLineIdType;

/*
 * Macros for displaying VpDeviceIdType and VpLineIdType values.  If you have
 * defined these (in vp_api_types.h) as something other than simple integers,
 * you should modify the printf format strings as needed:
 */
#define VP_PRINT_DEVICE_ID(deviceId)  VpSysDebugPrintf(" (dev 0x%lX)", (unsigned long)(deviceId))
#define VP_PRINT_LINE_ID(lineId)      VpSysDebugPrintf(" (line %lu)", (unsigned long)(lineId))

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
#define FALSE   (0)     /* Boolean constant */
#define TRUE    (1)     /* Boolean constant */
#ifndef __cplusplus
/* C++ language provides a boolean data type; So no need to define
 * one more data type; Make use of it
 * NOTE: The 'C' portions of the VP-API assume C++ "bool" to be of the
 * same size as that of "char". Please make sure this assumption is correct.
 */
#define bool _Bool
#endif /* __cplusplus */
/****************** typedefs ***********************************/
/* These are the basic number types used */
/* for uint8, uint16, uint32, int8, int16, int32 */
typedef unsigned char   uchar;
typedef __u8   uint8;
typedef __u16  uint16;
typedef __u32  uint32;
typedef __s8   int8;
typedef __s16  int16;
typedef __s32  int32;

typedef uint8*  uint8p;
typedef uint16* uint16p;
typedef uint32* uint32p;
typedef int8*   int8p;
typedef int16*  int16p;
typedef int32*  int32p;
typedef const VpProfileDataType * VpProfilePtrType;
typedef uint8p VpImagePtrType;
typedef uint16p VpVectorPtrType;
typedef uint8 VpPktDataType;
typedef VpPktDataType* VpPktDataPtrType;
/* Some compilers optimize the size of enumeration data types based on
 * the maximum data value assigned to the members of that data type.
 * 'Standard C' requires enumeration data types to be of the same size
 * as that of native 'int' implementation.
 * The VP-API from a portability perspective adds a 'dummy' member to
 * all enumeration data types that force the compilers to allocate the size
 * of enumeration data types to be equal to that of native 'int'
 * implementation */
#define FORCE_STANDARD_C_ENUM_SIZE  (0xFFFF)

/* Eliminate error messages that occur when comparing an enumeration constant
   < 0 */
#define FORCE_SIGNED_ENUM  (0xFFFF)

/* Define any API specific basic data type ranges (that are necessary) */
#define VP_INT16_MAX    (SHRT_MAX)
#define VP_INT16_MIN    (SHRT_MIN)
#define VP_INT32_MAX    (LONG_MAX)
#define VP_INT32_MIN    (LONG_MIN)
#endif /* VP_API_TYPES_H */
