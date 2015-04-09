/** \file vp_hal.c
 * vp_hal.c
 *
 * This file contains the platform dependent code for the Hardware Abstraction
 * Layer (HAL). This is example code only to be used by the customer to help
 * clarify HAL requirements.
 *
 * Copyright (c) 2011, Microsemi Corporation
 */
#include "vp_hal.h"
#include "sys_service.h"
#include "telecom_hal.h"
#include "vpapi_stats.h"

uint8 mpiDevices[256];

/*****************************************************************************
 * HAL functions for VCP and VPP. Not necessary for CSLAC devices.
 ****************************************************************************/
/**
 * VpHalHbiInit(): Configures the HBI bus and glue logic (if any)
 *
 * This function performs any tasks necessary to prepare the system for
 * communicating through the HBI, including writing the HBI configuration
 * register.  The HBI read and write functions should work after HbiHalInit()
 * is successfully executed. HbiHalInit() should be well-behaved even if called
 * more than once between system resets. HbiHalInit() is called from
 * VpBootLoad() since VpBootLoad() is normally the first VoicePath function
 * that the host application will call.
 *
 * This function is called by VpBootLoad() before sending the VCP firmware
 * image through the HBI.
 *
 * Params:
 *  uint8 deviceId: Device Id (chip select ID)
 *
 * Returns:
 *  This function returns FALSE if some error occurred during HBI initialization
 *  or TRUE otherwise.
 */
bool VpHalHbiInit(
    VpDeviceIdType deviceId)
{
    /*
     * Note that Setting up the basic device should be handled by the
     * some board bring up function. That function should setup the
     * CPLD based on the line module that is plugged in. Those functions
     * would configure the CPLD so that basic communication to the part
     * can happen between the HAL and the line module.
     */
    /* Write the HBI configuration register. */
    return VpHalHbiCmd(deviceId, HBI_CMD_CONFIGURE_INT + HBI_PINCONFIG);
} /* VpHalHbiInit() */
/**
 * VpHalHbiCmd(): Sends a command word over the HBI, with no data words.
 *
 *  Accepts a uint16 HBI command which is little-endian or big-endian,
 * depending on the host architecture.  Command words on the HBI bus are always
 * big-endian. This function is responsible for byte-swapping if required.
 *
 * Params:
 * uint8 deviceId: Device Id (chip select ID)
 * uint16 cmd: the command word to send
 *
 * Returns:
 *   TRUE on success, FALSE on failure
 */
bool VpHalHbiCmd(
    VpDeviceIdType deviceId,
    uint16 cmd)
{
    return TelecomVpHalHbiCmd(deviceId, cmd);
} /* VpHalHbiCmdWr() */
/**
 * VpHalHbiWrite(): Sends a command word and up to 256 data words over the HBI.
 *
 *  Accepts a uint16 HBI command which is little-endian or big-endian, depending
 * on the host architecture.  Command words on the HBI bus are always big-
 * endian.  This function is responsible for byte-swapping the command word, if
 * required.
 *
 *  Accepts an array of uint16 data words.  No byte-swapping is necessary on
 * data words in this function.  Instead, the HBI bus can be configured in
 * VpHalHbiInit() to match the endianness of the host platform.
 *
 * Params:
 *   uint8 deviceId: Device Id (chip select ID)
 *   uint16 cmd: the command word to send
 *   uint8 numwords: the number of data words to send, minus 1
 *   uint16p data: the data itself; use data = (uint16p)0 to send
 *      zeroes for all data words
 *
 * Returns:
 *   TRUE on success, FALSE on failure
 */
bool VpHalHbiWrite(
    VpDeviceIdType deviceId,
    uint16 cmd,
    uint8 numwords,
    uint16p data)
{
    return TelecomVpHalHbiWrite(deviceId, cmd, numwords, data);
} /* VpHalHbiWrite() */
/**
 * VpHalHbiRead(): Sends a command, and receives up to 256 data words over the
 * HBI.
 *
 *  Accepts a uint16 HBI command which is little-endian or big-endian, depending
 * on the host architecture.  Command words on the HBI bus are always big-
 * endian.  This function is responsible for byte-swapping the command word, if
 * required.
 *
 * Retrieves an array of uint16 data words.  No byte-swapping is necessary on
 * data words in this function.  Instead, the HBI bus can be configured in
 * VpHalHbiInit() to match the endianness of the host platform.
 *
 * Params:
 *   uint8 deviceId: Device Id (chip select ID)
 *   uint8 numwords: the number of words to receive, minus 1
 *   uint16p data: where to put them
 *
 * Returns:
 *   TRUE on success, FALSE on failure
 */
bool VpHalHbiRead(
    VpDeviceIdType deviceId,
    uint16 cmd,
    uint8 numwords,
    uint16p data)
{
    return TelecomVpHalHbiRead(deviceId, cmd, numwords, data);
} /* VpHalHbiRead() */
/**
 * VpHalHbiWrite8(): Sends a command word and up to 256 data words over the HBI.
 *
 *  Accepts a uint16 HBI command which is little-endian or big-endian, depending
 * on the host architecture.  Command words on the HBI bus are always big-
 * endian.  This function is responsible for byte-swapping the command word, if
 * required. Note that this Linux implementation does not need byte swapping.
 *
 *  Accepts an array of uint8 data words.  No byte-swapping is necessary on
 * data words in this function.  Instead, the HBI bus can be configured in
 * VpHalHbiInit() to match the endianness of the host platform.
 *
 * Params:
 *   uint8 deviceId: Device Id (chip select ID)
 *   uint16 cmd: the command word to send
 *   uint8 numwords: the number of data words to send, minus 1
 *   uint8p data: the data itself; use data = (uint8p)0 to send
 *   zeroes for all data words
 *
 * Returns:
 *   TRUE on success, FALSE on failure
 */
bool VpHalHbiWrite8(
    VpDeviceIdType deviceId,
    uint16 cmd,
    uint8 numwords,
    uint8p data)
{
    return TelecomVpHalHbiWrite8(deviceId, cmd, numwords, data);
} /* VpHalHbiWrite8() */
/**
 * VpHalHbiRead8(): Sends a command, and receives up to 256 data words over the
 * HBI.
 *
 *  Accepts a uint16 HBI command which is little-endian or big-endian, depending
 * on the host architecture.  Command words on the HBI bus are always big-
 * endian.  This function is responsible for byte-swapping the command word, if
 * required.
 *
 *  Retrieves an array of uint8 data bytes.  No byte-swapping is necessary on
 * data words in this function.  Instead, the HBI bus can be configured in
 * VpHalHbiInit() to match the endianness of the host platform.
 *
 * Params:
 *   uint8 deviceId: Device Id (chip select ID)
 *   uint8 numwords: the number of words to receive, minus 1
 *   uint8p data: where to put them
 *
 * Returns:
 *   TRUE on success, FALSE on failure
 */
bool VpHalHbiRead8(
    VpDeviceIdType deviceId,
    uint16 cmd,
    uint8 numwords,
    uint8p data)
{
    return TelecomVpHalHbiRead8(deviceId, cmd, numwords, data);
} /* VpHalHbiRead8() */
/**
 * VpHalHbiBootWr():
 *
 *  This is used by the VpBootLoad() function to send the boot stream to the
 * VCP.  This function is separate from VpHalHbiWrite(), for the following
 * reasons:
 *
 *  1. This function does not accept a command word; only data words.
 *  2. This function accepts uint8 data, instead of uint16 data.  Be careful
 *     not to assume that this data is word-aligned in memory.
 *  3. The HBI must be configured for big-endian data words while the boot
 *     stream is being transmitted, regardless of the endianness of the host
 *     platform.  This is because the boot image is an opaque stream of HBI
 *     command words and data words.  Therefore, commands and data cannot be
 *     distinguished for separate treatment by this function.  Since HBI
 *     command words are always big-endian, data words have to be big-endian,
 *     too.  The boot stream is stored big-endian in memory, even on little-
 *     endian hosts.
 *        If VpHalHbiInit() configures the HBI for little-endian data words,
 *     then this function must temporarily change the configuration by calling
 *     VpHalHbiCmd(HBI_CMD_CONFIGURE(...)), and change it back before
 *     returning.  In such a case, this function will need to swap each pair
 *     of bytes in the boot stream before sending.
 *        Another possibility is a little-endian host architecture, with the HBI
 *     bus configured for big-endian data words.  In this case, byte-swapping
 *     has to be done in VpHalHbiWrite() or in the glue logic between the host
 *     and the VCP. In these setups, VpHalHbiBootWr() does not need to
 *     reconfigure the  HBI.
 *  4. This function takes a VpImagePtrType pointer to char data, which is a
 *     platform-dependent type defined in vp_hal.h.
 *
 * Params
 *   uint8 deviceId: Device Id (chip select ID)
 *  'length' specifies the number of 16-bit words to write to the VCP.
 *  'pBuf' points into the boot stream.
 *
 * Returns
 *  HbiHalBootWr() returns TRUE on success, FALSE on failure.
 *
 * Notes
 *  THIS FUNCTION IS NOT REENTRANT!
 */
bool VpHalHbiBootWr(
    VpDeviceIdType deviceId,
    uint8 numwords,
    VpImagePtrType data)
{
    return TelecomVpHalHbiBootWr(deviceId, numwords, data);
} /* VpHalHbiBootWr() */
/*****************************************************************************
 * HAL functions for CSLAC devices. Not necessary for VCP and VPP
 ****************************************************************************/
/**
 * VpMpiCmd()
 *  This function executes a Device MPI command through the MPI port. It
 * executes both read and write commands. The read or write operation is
 * determined by the "cmd" argument (odd = read, even = write). The caller must
 * ensure that the data array is large enough to hold the data being collected.
 * Because this command used hardware resources, this procedure is not
 * re-entrant.
 *
 * Note: For API-II to support multi-threading, this function has to write to
 * the EC register of the device to set the line being controlled, in addition
 * to the command being passed. The EC register write/read command is the same
 * for every CSLAC device and added to this function. The only exception is
 * if the calling function is accessing the EC register (read), in which case
 * the EC write cannot occur.
 *
 * This example assumes the implementation of two byte level commands:
 *
 *    MpiReadByte(VpDeviceIdType deviceId, uint8 *data);
 *    MpiWriteByte(VpDeviceIdType deviceId, uint8 data);
 *
 * Preconditions:
 *  The device must be initialized.
 *
 * Postconditions:
 *   The data pointed to by dataPtr, using the command "cmd", with length
 * "cmdLen" has been sent to the MPI bus via the chip select associated with
 * deviceId.
 */
void
VpMpiCmd(
    VpDeviceIdType deviceId,    /**< Chip select, connector and 3 or 4 wire
                                 * interface for command
                                 */
    uint8 ecVal,        /**< Value to write to the EC register */
    uint8 cmd,          /**< Command number */
    uint8 cmdLen,       /**< Number of bytes used by command (cmd) */
    uint8 *dataPtr)     /**< Pointer to the data location */
{
#if 0
    /* Limit input from VpDeviceIdType to 255 */
    uint8 deviceIndex = (deviceId & 0xFF);
    uint8 cmdIndex;

    if (mpiDevices[deviceIndex] != ecVal) {
        printk("Device: %d EC Value Changed From 0x%02X to 0x%02X\n\r",
            deviceId, mpiDevices[deviceIndex], ecVal);
        mpiDevices[deviceIndex] = ecVal;
    }

    printk("Device: %d %s Cmd: 0x%02X Data:", deviceId, ((cmd & 0x1) ? "READ" : "WRITE"), cmd);
    for (cmdIndex = 0; cmdIndex < cmdLen; cmdIndex++) {
        printk(" 0x%02X", dataPtr[cmdIndex]);
    }
    printk("\n\r");
#endif
    VPMOD_STAT(VPMOD_STAT_MPI_START,0,0,0);
    TelecomVpMpiCmd(deviceId, ecVal, cmd, cmdLen, dataPtr);
    VPMOD_STAT(VPMOD_STAT_MPI_STOP,(u32)deviceId, cmd, cmdLen);
    return;
} /* End VpMpiCmd */
