/* General C++ includes */
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/file.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

/* ROS includes */
#include <ros/ros.h>

/* Package specific includes */
#include "../include/utils.hpp"
#include "../include/ftd4222_helper.hpp"

/* Constant definitions */
#define FT4222_MAX_RETRIES 3
#define FTD4222_RETRY_SLEEP_TIME_S 2

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, FTDI                                                    #
#   Inputs:                                                                         #
#           @device               Reference to the node variable for the ft device  #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #             
#                                                                                   #
#   Description: Gets the first ftd4222 device's node handle.                       #
#                                                                                   #
************************************************************************************/

BobStatus getftd4222Dev(FT_DEVICE_LIST_INFO_NODE * device)
{
    std::vector<FT_DEVICE_LIST_INFO_NODE> g_FT4222DevList;

    try
    {
        DWORD numOfDevices = 0;
        FT_STATUS status = FT_CreateDeviceInfoList(&numOfDevices);
        for (DWORD iDev = 0; iDev < numOfDevices; ++iDev)
        {
            FT_DEVICE_LIST_INFO_NODE devInfo;
            memset(&devInfo, 0, sizeof(devInfo));
            status = FT_GetDeviceInfoDetail(iDev,
                                            &devInfo.Flags, &devInfo.Type, &devInfo.ID, &devInfo.LocId,
                                            devInfo.SerialNumber, devInfo.Description, &devInfo.ftHandle);
            if (FT_OK == status)
            {
                const std::string desc = devInfo.Description;
                if (desc == "FT4222" || desc == "FT4222 A")
                    g_FT4222DevList.push_back(devInfo);
            }
        }

        if (g_FT4222DevList.empty())
            return BOB_FTD4222_NOT_FOUND;

        *device = g_FT4222DevList[0];
    }
    catch(...)
    {
        return BOB_FTD4222_ERROR;
    }
    
    return BOB_SUCCESS;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, FTDI                                                    #
#   Inputs:                                                                         #
#           @ftHandle               Handle to the FTDI device                       #           
#                                                                                   #
#   Description: Uninitialize the ftd4222 module and close it, work for both SPI    #
#                and I2C. Thread safe.                                              #
#                                                                                   #
************************************************************************************/

void close4222(FT_HANDLE &ftHandle)
{
    FT4222_UnInitialize(ftHandle);
    FT_Close(ftHandle);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, FTDI                                                    #
#   Inputs:                                                                         #
#           @device               Reference to the node variable for the ft device  #
#           @ftHandle             Handle to the FTDI device                         #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #             
#                                                                                   #
#   Description: Initialies the I2C module on the 4222, getting its handle which    #
#                is modified by reference. Thread safe.                             #
#                                                                                   #
************************************************************************************/

BobStatus init4222I2C(FT_DEVICE_LIST_INFO_NODE device, FT_HANDLE & ftHandle)
{
    BobStatus status;
    FT_STATUS ftStatus;
    FT4222_STATUS ft4222Status;

    try
    {
        for(int retries = 0; retries < FT4222_MAX_RETRIES; retries++)
        {
            // Open the device
            ftStatus = FT_Open(0, &ftHandle);
            if (FT_OK != ftStatus)
            {
                status = BOB_FTD4222_ERROR;
                continue;
            }

            // Initialize the I2C master
            ft4222Status = FT4222_I2CMaster_Init(ftHandle, FT4222_I2C_KBPS);

            // Return if it was a success
            if(ft4222Status == FT4222_STATUS::FT4222_OK)
                return BOB_SUCCESS;

            // Record that the device isn't opened and retry
            if (ft4222Status == FT4222_STATUS::FT4222_DEVICE_NOT_OPENED)
                status = BOB_FTD4222_NOT_FOUND;

            // Record that there was an error and retry
            else 
                status = BOB_FTD4222_ERROR;

            std::this_thread::sleep_for(std::chrono::seconds(FTD4222_RETRY_SLEEP_TIME_S));
        }
    }
    catch(...)
    {
        return BOB_FTD4222_ERROR;
    }
    
    return status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, FTDI                                                    #
#   Inputs:                                                                         #
#           @device               Reference to the node variable for the ft device  #
#           @ftHandle            Handle to the FTDI device                          #
#           @deviceAddress       Address of the slave device                        #
#           @flag                Write flags                                        #
#           @buffer              Buffer of data to write                            #
#           @bytesToWrite        # of bytes to write                                #
#           @sizeTransferred     # of bytes successfully transferred                #
#                                                                                   #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #             
#                                                                                   #
#   Description: Performs a i2c write extension operation with the FTD4222 device,  #
#                with retries.                                                      #
#                                                                                   #   
************************************************************************************/

BobStatus ft4222I2CWriteExSafe(FT_DEVICE_LIST_INFO_NODE * device, FT_HANDLE & ftHandle, uint16 deviceAddress, uint8 flag,
                            uint8* buffer, uint16 bytesToWrite, uint16* sizeTransferred)
{
    BobStatus status = BOB_SUCCESS;
    
    try
    {
        for(int retries = 0; retries < FT4222_MAX_RETRIES; retries++)
        {
            // Attempt to write to the device
            FT_STATUS ft4222Status = FT4222_I2CMaster_WriteEx(ftHandle, deviceAddress, flag, buffer,
                                                            bytesToWrite, sizeTransferred);

            // Return if it was a success
            if(ft4222Status == FT4222_STATUS::FT4222_OK)
                return BOB_SUCCESS;
            else
                status = ft4222ConnectI2C(device, ftHandle);


            std::this_thread::sleep_for(std::chrono::seconds(FTD4222_RETRY_SLEEP_TIME_S));
        }
    }
    catch(...)
    {
        return BOB_FTD4222_ERROR;
    }
    
    return status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, FTDI                                                    #
#   Inputs:                                                                         #
#           @device              Reference to the node variable for the ft device   #
#           @ftHandle            Handle to the FTDI device                          #
#           @deviceAddress       Address of the slave device                        #
#           @buffer              Buffer of data to write                            #
#           @bytesToWrite        # of bytes to write                                #
#           @sizeTransferred     # of bytes successfully transferred                #
#                                                                                   #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #             
#                                                                                   #
#   Description: Performs a i2c write operation with the FTD4222 device,            #
#                with retries.                                                      #
#                                                                                   #   
************************************************************************************/

BobStatus ft4222I2CWriteSafe(FT_DEVICE_LIST_INFO_NODE * device, FT_HANDLE & ftHandle, uint16 deviceAddress,
                            uint8* buffer, uint16 bytesToWrite, uint16* sizeTransferred)
{
    BobStatus status = BOB_SUCCESS;
    
    try
    {
        for(int retries = 0; retries < FT4222_MAX_RETRIES; retries++)
        {
            // Attempt to write to the device
            FT_STATUS ft4222Status = FT4222_I2CMaster_Write(ftHandle, deviceAddress, buffer,
                                                            bytesToWrite, sizeTransferred);
            // Return if it was a success
            if(ft4222Status == FT4222_STATUS::FT4222_OK)
                return BOB_SUCCESS;
            else
                status = ft4222ConnectI2C(device, ftHandle);

            std::this_thread::sleep_for(std::chrono::seconds(FTD4222_RETRY_SLEEP_TIME_S));
        }
    }
    catch(...)
    {
        return BOB_FTD4222_ERROR;
    }
    
    return status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, FTDI                                                    #
#   Inputs:                                                                         #
#           @device              Reference to the node variable for the ft device   #
#           @ftHandle            Handle to the FTDI device                          #
#           @deviceAddress       Address of the slave device                        #
#           @flag                Write flags                                        #
#           @buffer              Buffer of data to write                            #
#           @bytesToRead         # of bytes to read                                 #
#           @sizeTransferred     # of bytes successfully transferred                #
#                                                                                   #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #             
#                                                                                   #
#   Description: Performs a i2c read extension operation with the FTD4222 device,   #
#                with retries.                                                      #
#                                                                                   #   
************************************************************************************/

BobStatus ft4222I2CReadExSafe(FT_DEVICE_LIST_INFO_NODE * device, FT_HANDLE & ftHandle, uint16 deviceAddress, uint8 flag,
                            uint8* buffer, uint16 bytesToRead, uint16* sizeTransferred)
{
    BobStatus status = BOB_SUCCESS;
    
    try
    {
        for(int retries = 0; retries < FT4222_MAX_RETRIES; retries++)
        {
            // Attempt to write to the device
            FT_STATUS ft4222Status = FT4222_I2CMaster_ReadEx(ftHandle, deviceAddress, flag, buffer,
                                                            bytesToRead, sizeTransferred);

            // Return if it was a success
            if(ft4222Status == FT4222_STATUS::FT4222_OK)
                return BOB_SUCCESS;
            else
                status = ft4222ConnectI2C(device, ftHandle);


            std::this_thread::sleep_for(std::chrono::seconds(FTD4222_RETRY_SLEEP_TIME_S));
        }
    }
    catch(...)
    {
        return BOB_FTD4222_ERROR;
    }
    
    return status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, FTDI                                                    #
#   Inputs:                                                                         #
#           @ftHandle            Handle to the FTDI device                          #
#           @pin                  The GPIO pin to set                               #
#           @state                The desired state of the GPIO pin                 #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #             
#                                                                                   #
#   Description: Configures GPIO as output, then sets the desired pin to a          #
#                specific state.                                                    #
#                                                                                   #
************************************************************************************/

BobStatus write4222GPIO(FT_HANDLE &ftHandle, int pin, bool state)
{
    GPIO_Dir gpioDir[4] = {GPIO_OUTPUT, GPIO_OUTPUT, GPIO_OUTPUT, GPIO_OUTPUT};

    // Initialize the PGIO
    FT4222_GPIO_Init(ftHandle, gpioDir);

    if(pin == 2)
    {
        FT4222_SetSuspendOut(ftHandle, false);
        FT4222_GPIO_Write(ftHandle, GPIO_PORT2, state ? 1 : 0);
    }
    else if(pin == 3)
    {
        FT4222_SetWakeUpInterrupt(ftHandle, false);
        FT4222_GPIO_Write(ftHandle, GPIO_PORT3, state ? 1 : 0);
    }
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, FTDI                                                    #
#   Inputs:                                                                         #
#           @device               Reference to the node variable for the ft device  #
#           @ftHandle             Handle to the FTDI device                         #
#   Outputs:                                                                        #
#           @returns              Status indicating the outcome of the function     #             
#                                                                                   #
#   Description: Attempts to discover the FTDI device and initialize the I2C.       #
#                                                                                   #
************************************************************************************/

BobStatus ft4222ConnectI2C(FT_DEVICE_LIST_INFO_NODE * device, FT_HANDLE & ftHandle)
{
     // Attempt to get the device
    BobStatus status = getftd4222Dev(device);

    if(status == BOB_SUCCESS)
    {
        // Attempt to initialize the I2C
        status = init4222I2C(*device, ftHandle);
            
        if(status == BOB_SUCCESS)
            return status;
    }

    return status;

}
