#ifndef FTD4222_HELPER_H
#define FTD4222_HELPER_H

/* Constant definitions */
#define FT4222_I2C_KBPS 100

/* FTD4222 */
#include "ftd2xx.h"
#include "libft4222.h"

/* Package specific includes */
#include "../include/bob_status.hpp"

/* Function declarations */
void close4222(FT_HANDLE&);
BobStatus reconnect4222I2C(FT_HANDLE &);
BobStatus write4222GPIO(FT_HANDLE&, int, bool);
BobStatus getftd4222Dev(FT_DEVICE_LIST_INFO_NODE*);
BobStatus init4222SPI(FT_DEVICE_LIST_INFO_NODE, FT_HANDLE&);
BobStatus init4222I2C(FT_DEVICE_LIST_INFO_NODE, FT_HANDLE*);
BobStatus ft4222ConnectI2C(FT_DEVICE_LIST_INFO_NODE * device, FT_HANDLE & ftHandle);
BobStatus ft4222I2CReadExSafe(FT_DEVICE_LIST_INFO_NODE * device, FT_HANDLE & ftHandle, uint16 deviceAddress, uint8 flag,
                            uint8* buffer, uint16 bytesToRead, uint16* sizeTransferred);
BobStatus ft4222I2CWriteSafe(FT_DEVICE_LIST_INFO_NODE * device, FT_HANDLE & ftHandle, uint16 deviceAddress,
                            uint8* buffer, uint16 bytesToWrite, uint16* sizeTransferred);
BobStatus ft4222I2CWriteExSafe(FT_DEVICE_LIST_INFO_NODE * device, FT_HANDLE & ftHandle, uint16 deviceAddress, uint8 flag,
                            uint8* buffer, uint16 bytesToWrite, uint16* sizeTransferred);

#endif