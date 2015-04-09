/** \file mpi_hal.h
 * mpi_hal.h
 *
 * Header file for the VP-API-II c files requiring MPI interface.
 *
 * Copyright (c) 2008, Zarlink Semiconductor, Inc.
 */
#ifndef MPI_UVB_HAL_H
#define MPI_UVB_HAL_H
#include "vp_api_types.h"
/*
 * The API header is needed to define the Device Types used by the API to know
 * how to implement VpMpiReset
 */
#include "vp_api_dev_term.h"

EXTERN void
VpMpiCmd(
    VpDeviceIdType deviceId,
    uint8 ecVal,
    uint8 cmd,
    uint8 cmdLen,
    uint8 *dataPtr);
#endif

