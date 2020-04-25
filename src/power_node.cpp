/* General c++ includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <mutex>
#include <csignal>

/* Package specific includes */
#include "../include/utils.hpp"
#include "../include/gpio.hpp"
#include "../include/bq24770.hpp"
#include "../include/bq4050.hpp"
#include "../include/bob_status.hpp"
#include "../include/ftd4222_helper.hpp"
#include "../include/bob_definitions.hpp"

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <ros_robo_bob/StartCharge.h>
#include <ros_robo_bob/SetCurrent.h>
#include <ros_robo_bob/GetCurrent.h>
#include <ros_robo_bob/GetVoltage.h>
#include <ros_robo_bob/GetCharge.h>
#include <ros_robo_bob/GetAbs.h>
#include <ros_robo_bob/FindChargeCurrent.h>
#include <ros_robo_bob/ChargeMaintenanceState.h>
#include <ros_robo_bob/GetBatteryStatus.h>
#include <ros_robo_bob/GetCells.h>
#include <ros_robo_bob/ResetPower.h>

/* Static variable definitions */
static bool runChargeMaintenance = false;
static bool foundStableCurrent = false;
static bool forceCharge = false;
static FT_HANDLE ftHandle;
static FT_DEVICE_LIST_INFO_NODE device;

/* Static mutex definitions */
static std::mutex runChargeMutex;
static std::mutex stableCurrentMutex;
static std::mutex forceChargeMutex;
static std::timed_mutex ftd4222Mutex;

/* Constant definitions */
#define POWER_NODE_NAME "power_node"
#define MIN_VOLTAGE 12000
#define MAX_VOLTAGE 18000
#define BATTERY_FULL_VOLTAGE 16300
#define MAX_RETIRES 3
#define FT4222_LOCK_TIMEOUT_S 10
#define CURRENT_SERVICE_RETRIES 3

/* Function declariations */
BobStatus resetBattery();

/*******************************************************************************
** I2C Read
** Requires access of the Ft4222 device and the BQ24773 Charging Board
** Will handle simple write protocols if there is no requirements to have
** different registers written to.
** Returns a BobStatus
*******************************************************************************/
BobStatus i2c_read_reg(uint8_t slaveAddr, uint8_t regAddr, uint8_t firstSetValue,
                         uint8_t bytes, int &result)
{
    BobStatus status;
    uint16_t sizeTransferred;
    uint8_t readBuf[3];
    uint8_t slaveData[1];
    slaveData[0] = regAddr;

    if(!ftd4222Mutex.try_lock_for(std::chrono::seconds(FT4222_LOCK_TIMEOUT_S)))
        return BOB_FTD4222_LOCK_TIMEOUT;

    status = ft4222I2CWriteExSafe(&device, ftHandle, slaveAddr, START, slaveData, sizeof(slaveData), &sizeTransferred);

    if (status != BOB_SUCCESS)
    {
        ftd4222Mutex.unlock();
        return status;
    }

    status = ft4222I2CReadExSafe(&device, ftHandle, slaveAddr, Repeated_START | STOP, readBuf, bytes, &sizeTransferred);

    if (status == BOB_SUCCESS)
        result = (int)readBuf[0];

    ftd4222Mutex.unlock();

    return status;
}
/*******************************************************************************
** I2C Write
** Used for special conditions where different registers need to be written to in order
** Allows for the last register to be read in order to confirm that the correct value was written
** Returns a BobStatus
*******************************************************************************/
BobStatus i2c_write_reg(uint8_t slaveAddr, uint8_t firstRegAddr, uint8_t secondRegAddr,
                        uint8_t firstSetValue, uint8_t secondSetValue, uint8_t bytes)
{
    BobStatus status;
    uint16_t sizeTransferred;
    uint8_t writeData[2];
    uint8_t slaveData[2];
    uint8_t readBuf[2];
    uint8_t readAddr[1];
    writeData[0] = firstRegAddr;
    readAddr[0] = secondRegAddr;
    slaveData[0] = secondRegAddr;
    writeData[1] = firstSetValue;
    slaveData[1] = secondSetValue;

    if(!ftd4222Mutex.try_lock_for(std::chrono::seconds(FT4222_LOCK_TIMEOUT_S)))
        return BOB_FTD4222_LOCK_TIMEOUT;

    status = ft4222I2CWriteSafe(&device, ftHandle, slaveAddr, writeData, 2, &sizeTransferred);

    if (status != BOB_SUCCESS)
    {
        ftd4222Mutex.unlock();
        return status;
    }
    status = ft4222I2CWriteSafe(&device, ftHandle, slaveAddr, slaveData, 2, &sizeTransferred);

    if (status != BOB_SUCCESS)
    {
        ftd4222Mutex.unlock();
        return status;
    }
    status = ft4222I2CWriteExSafe(&device, ftHandle, slaveAddr, START, readAddr, sizeof(readAddr), &sizeTransferred);

    if (status != BOB_SUCCESS)
    {
        ftd4222Mutex.unlock();
        return status;
    }

    status = ft4222I2CReadExSafe(&device, ftHandle, slaveAddr, Repeated_START | STOP, readBuf, bytes, &sizeTransferred);

    ftd4222Mutex.unlock();

    return status;
}


/*******************************************************************************
** SMB Gas Gauge Reset
** Used for special conditions where different registers need to be written to in order
** Gains Manufacturer access and calls to reset the chip during erroneous charging errors
** Returns a BobStatus
*******************************************************************************/
BobStatus smb_write_reset()
{
    BobStatus status;
    uint16_t sizeTransferred;
    uint8_t slaveAddr = FUEL_GAUGE_ADDR;
    uint8 writeData[5];
    writeData[0] = MANUFACTURER_ADDRESS;
    writeData[1] = GAS_GAUGE_TOGGLE_BYTE_1;
    writeData[2] = GAS_GAUGE_TOGGLE_BYTE_2;
    writeData[3] = GAS_GAUGE_TOGGLE_BYTE_3;
    writeData[4] = GAS_GAUGE_TOGGLE_BYTE_4;

    if(!ftd4222Mutex.try_lock_for(std::chrono::seconds(FT4222_LOCK_TIMEOUT_S)))
        return BOB_FTD4222_LOCK_TIMEOUT;

    status = ft4222I2CWriteSafe(&device, ftHandle, slaveAddr, writeData, 5, &sizeTransferred);

    if (status != BOB_SUCCESS)
    {
        ftd4222Mutex.unlock();
        return status;
    }

    ftd4222Mutex.unlock();

    return status;
}

/*******************************************************************************
** SMB Read -
** Requires access to the FT4222 device and the slave address of the bq4050 gas gauge
** Writes to the register of the gas gauge and reads the response
*******************************************************************************/
BobStatus smb_read_reg(uint8_t slaveAddr, uint8_t regAddr, uint16_t &result)
{
    FT_STATUS ft4222Status;
    uint16_t sizeTransferred;
    uint8 writeReq[2];
    uint8 recvBuf[3];
    writeReq[0] = regAddr;

    if(!ftd4222Mutex.try_lock_for(std::chrono::seconds(FT4222_LOCK_TIMEOUT_S)))
        return BOB_FTD4222_LOCK_TIMEOUT;

    BobStatus status = ft4222I2CWriteExSafe(&device, ftHandle, slaveAddr, START, &writeReq[0], 1, &sizeTransferred);

    if (status != BOB_SUCCESS)
    {
        ftd4222Mutex.unlock();
        return status;
    }

    status = ft4222I2CReadExSafe(&device, ftHandle, slaveAddr, Repeated_START | STOP, recvBuf, 3, &sizeTransferred);

    if (status != BOB_SUCCESS)
    {
        ftd4222Mutex.unlock();
        return status;
    }
    result = (int)recvBuf[1] * 256 + (int)recvBuf[0];

    ftd4222Mutex.unlock();

    return BOB_SUCCESS;
}

/*******************************************************************************
** Charging Board Read and Writes
** Sets basic parameters of the charging board
** Returns a BobStatus
*******************************************************************************/
BobStatus charger_check_board(bool &chargerConnected)
{
    int result;
    uint8_t buf = 1;
    BobStatus status = i2c_read_reg(CHARGER_ADDR_I2C, CHARGER_DEVICE_ID_ADDR_I2C,
                                                NULL_VAL, buf, result);
    if (result != BOARD_ID)
    {
        chargerConnected = false;
        return status;
    }
    else
    {
        chargerConnected = true;
        return status;
    }
}

BobStatus charger_set_min_sys_voltage(uint16_t millivolts)
{
    bool chargerConnected;
    BobStatus status = charger_check_board(chargerConnected);
    if (chargerConnected)
    {
        uint8_t buf = 1;
        uint8_t setValue = (uint8_t)((millivolts >> 8) & MIN_SYS_VOLTAGE);
        status = i2c_write_reg(CHARGER_ADDR_I2C, CHARGER_MIN_SYS_VOLTAGE_ADDR_I2C,
                                CHARGER_MIN_SYS_VOLTAGE_ADDR_I2C, setValue,
                                setValue, buf);
        return status;
    }
    else
    {
        return status;
    }
}

BobStatus charger_set_max_charge_voltage(uint16_t millivolts)
{
    uint8_t buf = 2;
    uint8_t firstSetValue;
    uint8_t secondSetValue;
    bool chargerConnected;
    BobStatus status = charger_check_board(chargerConnected);
    if (chargerConnected)
    {
        firstSetValue = millivolts % 256;
        secondSetValue = millivolts / 256;
        status = i2c_write_reg(CHARGER_ADDR_I2C, CHARGER_MAX_CHARGE_VOLTAGE_ADDR_I2C,
                                         SECOND_CHARGER_MAX_CHARGE_VOLTAGE_ADDR_I2C, firstSetValue,
                                         secondSetValue, buf);
        return status;
    }
    else
    {
        return status;
    }
}

BobStatus charger_set_charge_current(uint16_t milliamps)
{
    uint8_t buf = 2;
    uint8_t firstSetValue;
    uint8_t secondSetValue;
    bool chargerConnected;
    BobStatus status = charger_check_board(chargerConnected);
    if (chargerConnected)
    {
        firstSetValue = milliamps % 256;
        secondSetValue = (uint8_t)((milliamps / 256) & CHARGE_CURRENT_CONV);
        status = i2c_write_reg(CHARGER_ADDR_I2C, CHARGER_CHARGE_CURRENT_ADDR_I2C,
                                         SECOND_CHARGER_CHARGE_CURRENT_ADDR_I2C, firstSetValue, secondSetValue,
                                         buf);
        return status;
    }
    else
    {
        return status;
    }
}

BobStatus charger_set_input_current(uint16_t milliamps)
{
    uint8_t buf = 1;
    uint8_t firstSetValue;
    bool chargerConnected;
    BobStatus status = charger_check_board(chargerConnected);
    if (chargerConnected)
    {
        firstSetValue = (uint8_t)((milliamps & INPUT_CURRENT_CONV) >> 6);
        status = i2c_write_reg(CHARGER_ADDR_I2C, CHARGER_INPUT_CURRENT_ADDR_I2C,
                                         CHARGER_INPUT_CURRENT_ADDR_I2C, firstSetValue,
                                         firstSetValue, buf);
        return status;
    }
    else
    {
        return status;
    }
}

BobStatus charger_disable_watchdog_timer()
{
    uint8_t buf = 1;
    uint8_t secondSetValue;
    bool chargerConnected;
    BobStatus status = charger_check_board(chargerConnected);
    if (chargerConnected)
    {
        secondSetValue &= DISABLE_WATCHDOG_TIMER;
        status = i2c_write_reg(CHARGER_ADDR_I2C, CHARGER_OPTION_0_ADDR_I2C_2,
                                         CHARGER_OPTION_0_ADDR_I2C_2, NULL_VAL, secondSetValue, buf);
        return status;
    }
    else
    {
        return status;
    }
}

BobStatus charger_enable_charging()
{
    bool chargerConnected;
    BobStatus status = charger_check_board(chargerConnected);
    if (chargerConnected)
    {
        uint8_t buf = 1;
        uint8_t firstSetValue;
        uint8_t secondSetValue;
        firstSetValue &= ENABLE_CHARGING_VAL_1;
        secondSetValue &= ENABLE_CHARGING_VAL_2;
        status = i2c_write_reg(CHARGER_ADDR_I2C, CHARGER_OPTION_0_ADDR_I2C,
                              CHARGER_OPTION_0_ADDR_I2C_2, firstSetValue, secondSetValue, buf);
        return status;
    }
    else
    {
        return status;
    }
}



/*******************************************************************************
** Gas Gauge Read Register -
** Passes register addresses of the gas guage and reads reponse from SMB read
** Returns a BobStatus
*******************************************************************************/
BobStatus fuel_gauge_get_manufacturer(bool &fuelConnected) // in mV
{
    uint16_t result;
    BobStatus status;

    // Try to get the manufacturer ID w/ retries
    for (int i = 0; i < MAX_RETIRES; i++)
    {
        status = smb_read_reg(FUEL_GAUGE_ADDR, MANUFACTURER_ADDR, result);

        if (status == BOB_SUCCESS)
        {
            if (result == MANUFACTURER_ID)
            {
                fuelConnected = true;
                return status;
            }
        }
    }

    // If we've reached here, we're not able to connect
    fuelConnected = false;

    // Check to see if the FTD chip had an error, or the BQ4050
    if (status == BOB_SUCCESS)
        return BOB_BQ4050_I2C_RW_ERR;
    else
        return status;
}

BobStatus fuel_gauge_get_RelativeStateOfCharge(uint16_t &charge)
{
    uint16_t result;
    bool fuelConnected;
    BobStatus status;
    status = fuel_gauge_get_manufacturer(fuelConnected);
    if (fuelConnected)
    {
        status = smb_read_reg(FUEL_GAUGE_ADDR, REALATIVE_STATE_ADDR, result);
        if (status == BOB_SUCCESS)
        {
            charge = result;
            return BOB_SUCCESS;
        }
        else
        {
            charge = ERROR;
            return status;
        }
    }
    else
    {
        charge = ERROR;
        return status;
    }
}

BobStatus fuel_gauge_get_Voltage(uint16_t &voltage) // in mV
{
    uint16_t result;
    bool fuelConnected;
    BobStatus status;
    status = fuel_gauge_get_manufacturer(fuelConnected);

    // Only continue if we've verified that we can connect
    if (fuelConnected)
    {
        // Attempt to read the voltage, with retires
        for (int i = 0; i < MAX_RETIRES; i++)
        {
            status = smb_read_reg(FUEL_GAUGE_ADDR, VOLTAGE_ADDR, result);

            if (status == BOB_SUCCESS)
            {
                voltage = result;

                // Do a sanity check on the reading
                if (result > MIN_VOLTAGE && result < MAX_VOLTAGE)
                    return BOB_SUCCESS;
            }
            else
            {
                voltage = ERROR;
                return status;
            }
        }

        //Check to see if the FTD chip had an error, or the BQ4050
        if (status == BOB_SUCCESS)
            return BOB_BQ4050_BAD_VOLTAGE;
        else
            return status;
    }
    else
    {
        voltage = ERROR;
        return status;
    }
}

BobStatus fuel_gauge_get_Current(int16 &current, int sampleSize) // in mA
{
    uint16_t result;
    bool fuelConnected;
    BobStatus status;
    int averageCurrent = 0;

    status = fuel_gauge_get_manufacturer(fuelConnected);

    if (fuelConnected)
    {
        for (int i = 0; i < sampleSize; i++)
        {
            status = smb_read_reg(FUEL_GAUGE_ADDR, CURRENT_ADDR, result);
            if (status == BOB_SUCCESS)
            {
                averageCurrent += (int)result;
            }
            else
            {
                current = ERROR;
                return status;
            }
        }

        current = averageCurrent / sampleSize;

        return BOB_SUCCESS;
    }
    else
    {
        current = ERROR;
        return status;
    }
}

BobStatus fuel_gauge_get_BatteryStatus(uint16_t &batteryStatus)
{
    uint16_t result;
    bool fuelConnected;
    BobStatus status;
    status = fuel_gauge_get_manufacturer(fuelConnected);
    if (fuelConnected)
    {
        status = smb_read_reg(FUEL_GAUGE_ADDR, BATTERY_STATUS_ADDR, result);
        if (status == BOB_SUCCESS)
        {
            batteryStatus = result;
            return BOB_SUCCESS;
        }
        else
        {
            batteryStatus = ERROR;
            return status;
        }
    }
    else
    {
        batteryStatus = ERROR;
        return status;
    }
}

BobStatus fuel_gauge_get_AbsoluteStateOfCharge(uint16_t &abs) // percent (0 - 100) of remaining battery capacity
{
    uint16_t result;
    bool fuelConnected;
    BobStatus status;
    status = fuel_gauge_get_manufacturer(fuelConnected);
    if (fuelConnected)
    {
        status = smb_read_reg(FUEL_GAUGE_ADDR, ABSOLUTE_STATE_ADDR, result);
        if (status == BOB_SUCCESS)
        {
            abs = result;
            return BOB_SUCCESS;
        }
        else
        {
            abs = ERROR;
            return status;
        }
    }
    else
    {
        abs = ERROR;
        return status;
    }
}

BobStatus fuel_gauge_get_CellVoltage(uint16_t &cell, uint8_t cellAddr) // in mV
{
    uint16_t result;
    bool fuelConnected;
    BobStatus status;
    fuel_gauge_get_manufacturer(fuelConnected);
    if (fuelConnected)
    {
        status = smb_read_reg(FUEL_GAUGE_ADDR, cellAddr, result);
        if (status == BOB_SUCCESS)
        {
            cell = result;
            return BOB_SUCCESS;
        }
        else
        {
            cell = ERROR;
            return status;
        }
    }
    else
    {
        cell = ERROR;
        return status;
    }
}



bool voltageServiceCallback(ros_robo_bob::GetVoltage::Request &req, ros_robo_bob::GetVoltage::Response &res)
{
    uint16_t voltage;
    BobStatus status;

    res.status = fuel_gauge_get_Voltage(voltage);
    res.voltage = voltage;

    return true;
}


bool chargerServiceCallback(ros_robo_bob::GetCharge::Request &req, ros_robo_bob::GetCharge::Response &res)
{
    int retries;
    uint16_t charge;
    BobStatus status;

    res.status = fuel_gauge_get_Voltage(charge);
    res.charge = charge;

    return true;
}



bool currentfServiceCallback(ros_robo_bob::GetCurrent::Request &req, ros_robo_bob::GetCurrent::Response &res)
{
    int16 current = 0;
    res.status = BOB_CHARGER_STATE_LOCKED;

    for(int i = 0; i < CURRENT_SERVICE_RETRIES; i++)
    {
        // Read in the current
        res.status = fuel_gauge_get_Current(current, req.sampleSize);

        // Reset the battery connection if the charger isn't letting a charge happen
        if(res.status == BOB_SUCCESS && current == 0)
        {
            uint16_t voltage;

            // Obtain the battery voltage
            res.status = fuel_gauge_get_Voltage(voltage);

            if(res.status != BOB_SUCCESS)
                return true;

            // Only reset the battery if it isn't full
            if(voltage < BATTERY_FULL_VOLTAGE)
            {
                res.status = resetBattery();

                if(res.status != BOB_SUCCESS)
                    return true;
            }
        }

	// Return if the read was a success
	if(res.status == BOB_SUCCESS)
            break;
    }

    // Populate the current
    res.current = current;

    return true;
}

bool batteryServiceCallback(ros_robo_bob::GetBatteryStatus::Request &req, ros_robo_bob::GetBatteryStatus::Response &res)
{
    uint16_t batteryStatus;
    BobStatus status;

    res.status = fuel_gauge_get_BatteryStatus(batteryStatus);
    res.batteryStatus = batteryStatus;

    return true;
}

bool absServiceCallback(ros_robo_bob::GetAbs::Request &req, ros_robo_bob::GetAbs::Response &res)
{
    uint16_t abs;
    BobStatus status;

    res.status = fuel_gauge_get_AbsoluteStateOfCharge(abs);
    res.abs = abs;

    return true;
}

bool cellServiceCallback(ros_robo_bob::GetCells::Request &req, ros_robo_bob::GetCells::Response &res)
{
    uint16_t cell;
    uint16_t cellVoltage[4];
    BobStatus status;

    status = fuel_gauge_get_CellVoltage(cell, CELL_VOLTAGE_ADDR_1);
    cellVoltage[0] = cell;
    status += fuel_gauge_get_CellVoltage(cell, CELL_VOLTAGE_ADDR_2);
    cellVoltage[1] = cell;
    status += fuel_gauge_get_CellVoltage(cell, CELL_VOLTAGE_ADDR_3);
    cellVoltage[2] = cell;
    status += fuel_gauge_get_CellVoltage(cell, CELL_VOLTAGE_ADDR_4);
    cellVoltage[3] = cell;
    res.status = status;
    res.cellVoltage = {cellVoltage[0], cellVoltage[1], cellVoltage[2], cellVoltage[3]};

    return true;
}
/*******************************************************************************
** Charging Board Services
** Service calls to set specific parameters of the Charging Board
** Returns a BobStatus
*******************************************************************************/
BobStatus charge_start()
{
    BobStatus status = charger_enable_charging();
    if (status == BOB_SUCCESS)
    {
        return BOB_SUCCESS;
    }
    else
    {
        return status;
    }
}

BobStatus charger_current(uint16_t milliamps)
{
    BobStatus status = charger_set_charge_current(milliamps);
    if (status == BOB_SUCCESS)
    {
        return BOB_SUCCESS;
    }
    else
    {
        return status;
    }
}

BobStatus battery_init()
{
    bool chargerConnected;
    BobStatus status = charger_check_board(chargerConnected);
    if (chargerConnected)
    {
        status = charger_enable_charging();
        if (status != BOB_SUCCESS)
        {
            return status;
        }
        status = charger_set_max_charge_voltage(DEFAULT_MAX_VOLTAGE);
        if (status != BOB_SUCCESS)
        {
            return status;
        }
        status = charger_set_input_current(DEFAULT_CURRENT);
        if (status != BOB_SUCCESS)
        {
            return status;
        }
        status = charger_set_min_sys_voltage(DEAFULT_MIN_VOLTAGE);
        if (status != BOB_SUCCESS)
        {
            return status;
        }
        status = charger_set_charge_current(DEFAULT_CURRENT);
        if (status != BOB_SUCCESS)
        {
            return status;
        }
        status = charger_disable_watchdog_timer();
        if (status != BOB_SUCCESS)
        {
            return status;
        }
        return status;
    }
    else
    {
        return status;
    }
}

bool resetPowerCallback(ros_robo_bob::ResetPower::Request &req, ros_robo_bob::ResetPower::Response &res)
{
    // Reset the battery connection
    resetBattery();

    return true;
}

bool chargeServiceCallback(ros_robo_bob::StartCharge::Request &req, ros_robo_bob::StartCharge::Response &res)
{

    res.status = charge_start();
    return true;
}

bool currentServiceCallback(ros_robo_bob::SetCurrent::Request &req, ros_robo_bob::SetCurrent::Response &res)
{
    FT_HANDLE ftHandle;
    uint16_t milliamp;

    milliamp = req.current;
    res.status = charger_current(milliamp);

    return true;
}

bool findChargeCurrentServiceCallback(ros_robo_bob::FindChargeCurrent::Request &req, ros_robo_bob::FindChargeCurrent::Response &res,
                                      ros::NodeHandle & handle, ros::ServiceClient *chargeCurrentClient)
{
    int loopCount = 0;
    int16 currentResult;
    res.current = false;

    res.status = battery_init();

    if(res.status != BOB_SUCCESS)
        return true;

    res.status = charger_set_max_charge_voltage(DEFAULT_MAX_VOLTAGE);

    if(res.status != BOB_SUCCESS)
        return true;


    res.status = charger_set_charge_current(DEFAULT_CURRENT);

    if(res.status != BOB_SUCCESS)
        return true;

    ros_robo_bob::GetCurrent server;
    server.request.sampleSize = CHARGE_CURRENT_SAMPLE_SIZE;
    chargeCurrentClient->call(server);

    if(server.response.status == BOB_SUCCESS)
    {
        currentResult = server.response.current;

        if (currentResult >= FIND_CHARGER_CURRENT_THRESH)
        {
            res.status = server.response.status;
            res.current = true;
        }
        else
        {
            res.status = server.response.status;
            res.current = false;
        }
    }
    else
    {
        res.status = server.response.status;
    }

    return true;
}

bool chargeMaintenanceStateCallback(ros_robo_bob::ChargeMaintenanceState::Request &req, ros_robo_bob::ChargeMaintenanceState::Response &res)

{
    if (req.state)
    {
        // Lock the CTR mutex
        setResource<bool>(stableCurrentMutex, foundStableCurrent, false);
        setResource<bool>(runChargeMutex, runChargeMaintenance, true);
        setResource<bool>(forceChargeMutex, forceCharge, false);
    }
    else
    {
        // Relase the CTR mutex
        setResource<bool>(stableCurrentMutex, foundStableCurrent, false);
        setResource<bool>(runChargeMutex, runChargeMaintenance, false);
        setResource<bool>(forceChargeMutex, forceCharge, false);
    }
    return true;
}

/*******************************************************************************
** Find Max Current
** Ramps up Charge current to find stable charge
** Finds the edge most stable current
** Returns stable current
*******************************************************************************/
BobStatus getMaxChargeCurrent(ros::NodeHandle &handle, uint16_t &stableCurrent, ros::ServiceClient *chargeCurrentClient)
{
    ros::Rate r(MAX_CHARGE_CURRENT_HZ);
    BobStatus status;
    uint16_t defaultCurrent = DEFAULT_CURRENT;
    uint16_t runningCurrent = DEFAULT_CURRENT;
    int16 currentResult;
    int16 currentHolder;
    int16 current;
    int syncLost = 0;
    //
    // initialize the battery
    //
    status = battery_init();
    if (status != BOB_SUCCESS)
        return status;

    //
    // start the charging at default levels
    //
    status = charger_set_max_charge_voltage(DEFAULT_MAX_VOLTAGE);
    if (status != BOB_SUCCESS)
        return status;

    status = charger_set_charge_current(defaultCurrent);
    if (status != BOB_SUCCESS)
        return status;
    //
    // the bq4050 servers to set up current values
    //
    ros_robo_bob::GetCurrent server;
    server.request.sampleSize = CHARGE_CURRENT_SAMPLE_SIZE;
    chargeCurrentClient->call(server);
    currentResult = server.response.current;
    currentHolder = currentResult;
    //
    //enter the ros loop to begin searching for max current
    //
    while (ros::ok())
    {
        //
        //grab the initial current readings from the bq4050
        //
        chargeCurrentClient->call(server);
        currentResult = server.response.current;
	
        // Check and see if the service call was successful
        if(server.response.status != BOB_SUCCESS)
        {
            return server.response.status;
        }
	//quick return for charging
	if(runningCurrent >= MAX_CHARGING_FLAG)
        {
            stableCurrent = MAX_CHARGING_FLAG;
	    ROS_ERROR_STREAM("Quick Return Current %d" << stableCurrent);
            return BOB_SUCCESS;
        }
        //
        //we have hit trickle and need to adjust
        //
        if (currentResult < CURRENT_PADDING)
        {
            //
            //adjust our current
            //
            runningCurrent -= CURRENT_ADJUSTMENT;
            //
            //set charging parameters with our new adjusted running currentl
            //
            status = charger_set_max_charge_voltage(DEFAULT_MAX_VOLTAGE);
            if (status != BOB_SUCCESS)
                return status;

            status = charger_set_charge_current(runningCurrent);
            if (status != BOB_SUCCESS)
                return status;

            //
            //increment the sync lost and check to see if we have hit the edge max charge
            //
            syncLost += 1;
            if (syncLost >= SYNC_LOST_MAX || runningCurrent >= MAX_CHARGING_FLAG)
            {
                if(runningCurrent >= MAX_CHARGING_FLAG)
                {
                  stableCurrent = MAX_CHARGING_FLAG;
                }
                else
                {
                  stableCurrent = runningCurrent;
                }
                ROS_ERROR_STREAM("Find Max Charge Current %d" << stableCurrent);
                return BOB_SUCCESS;
            }
        }
        //
        //check to see if we should increment based on our remainder and that we are above the padding
        //
        else if (currentResult % currentHolder < MAX_MOD_DIF && currentHolder > CURRENT_PADDING)
        {
            //
            //increment the running current value
            //
            runningCurrent += CURRENT_ADJUSTMENT;
            //
            //set up our charging parameters with the new running current
            //
            status = charger_set_max_charge_voltage(DEFAULT_MAX_VOLTAGE);
            if (status != BOB_SUCCESS)
                return status;

            status = charger_set_charge_current(runningCurrent);
            if (status != BOB_SUCCESS)
                return status;

        }
        //
        //call the bq4050 and update our new current readings
        //
        chargeCurrentClient->call(server);
        // Check and see if the service call was successful
        if(server.response.status == BOB_SUCCESS)
        {
            currentResult = server.response.current;
            currentHolder = currentResult;
        }
        else
        {
            return server.response.status;
        }

        r.sleep();
    }
}

BobStatus getChargeCurrent(int16_t & current, ros::ServiceClient * chargeCurrentClient,
                           ros::Publisher * statusPub)
{
    int retries;
    std_msgs::Int8 statusMessage;

    // Instantiate the current server
    ros_robo_bob::GetCurrent currentServer;

    BobStatus status = BOB_CURRENT_SERVICE_FAILED;

    // Set the charge current average sample size
    currentServer.request.sampleSize = CHARGE_CURRENT_SAMPLE_SIZE;

    // Attempt to read the charge current
    for(retries = 0; retries < CHARGE_CURRENT_MAX_RETRIES; retries++)
    {
        // Continue only if the service call succeeded
        if(chargeCurrentClient->call(currentServer))
        {
            if(currentServer.response.status != BOB_SUCCESS)
            {
                status = currentServer.response.status;
            }
            else
            {
                current = currentServer.response.current;
                return BOB_SUCCESS;
            }
        }
        else
        {
            continue;
        }
    }

    // If we've reached here, theres been an error
    current = 0;

    return status;
}

BobStatus getVoltage(uint16_t & voltage, ros::ServiceClient * voltageClient,
                     ros::Publisher * statusPub)
{
    int retries;
    std_msgs::Int8 statusMessage;

    // Instantiate the voltage server
    ros_robo_bob::GetVoltage voltageServer;

    BobStatus status = BOB_VOLTAGE_SERVICE_FAILED;

    // Attempt to read the voltage
    for(retries = 0; retries < VOLTAGE_MAX_RETRIES; retries++)
    {
        // Continue only if the service call succeeded
        if(voltageClient->call(voltageServer))
        {
            if(voltageServer.response.status != BOB_SUCCESS)
            {
                status = voltageServer.response.status;
            }
            else
            {
                voltage = voltageServer.response.voltage;
                return BOB_SUCCESS;
            }
        }
        else
        {
            continue;
        }
    }

    // If we've reached here, theres been an error
    voltage = 0;

    return status;
}


BobStatus getAbs(uint16_t & abs, ros::ServiceClient * absClient,
                     ros::Publisher * statusPub)
{
    int retries;
    std_msgs::Int8 statusMessage;

    // Instantiate the abs server
    ros_robo_bob::GetAbs absServer;

    BobStatus status = BOB_ABS_SERVICE_FAILED;

    // Attempt to read the abs
    for(retries = 0; retries < ABS_MAX_RETRIES; retries++)
    {
        // Continue only if the service call succeeded
        if(absClient->call(absServer))
        {
            if(absServer.response.status != BOB_SUCCESS)
            {
                status = absServer.response.status;
            }
            else
            {
                abs = absServer.response.abs;
                return BOB_SUCCESS;
            }
        }
        else
        {
            continue;
        }
    }

    // If we've reached here, theres been an error
    abs = 0;

    return status;
}


BobStatus checkStateOfCharge(uint16_t voltage, uint16_t abs)
{
    if(abs == MAX_ABS_STATE && voltage < MAX_VOLTAGE_THRESH)
        return resetBattery();

    return BOB_SUCCESS;
}

BobStatus resetBattery()
{
    ROS_ERROR("Resetting the battery connection");

    // Initialize the status
    BobStatus status = BOB_SUCCESS;

    // reset the gas gauge chip
    for(int i = 0; i < GAS_GAUGE_RESET_TRIES; i++)
    {
        status = smb_write_reset();
        if (status == BOB_SUCCESS)
            break;
    }

    return status;

}

/*******************************************************************************
** Charge Maintenance
** Call to find the max stable current we can pull
** This will then monitor the charge current and take actions accordingly
*******************************************************************************/
void chargeMaintenance(ros::Publisher *statusPub, ros::ServiceClient *chargeCurrentClient,
                       ros::ServiceClient *batVoltageClient, ros::ServiceClient *batAbsClient)
{
    ros::NodeHandle handle;
    ros::Publisher chargerStatusPub = handle.advertise<std_msgs::Int8>(addBase2Topic(POWER_NODE_NAME, "charger_status"), 1);
    std_msgs::Int8 msg;
    FT_DEVICE_LIST_INFO_NODE device;
    ros::Rate r(CHARGE_MAINTENANCE_HZ);
    uint16_t stableCurrent;
    int16_t currentResult;
    uint16_t voltageResult;
    uint16_t voltage;
    uint16_t abs;
    int chargeTrimmer;
    int negativeCurrentCount;
    BobStatus status = BOB_SUCCESS;

    //
    //setup the bq4050 servers to call
    //
    ros_robo_bob::GetCurrent currentServer;
    ros_robo_bob::GetVoltage voltageServer;
    ros_robo_bob::GetAbs absServer;

    // Set the charge current average sample size
    currentServer.request.sampleSize = CHARGE_CURRENT_SAMPLE_SIZE;

    while (ros::ok())
    {
        // Check Status
        if (status != BOB_SUCCESS)
        {
            // Advertise the error to the supervisor
            handleStatus(status, statusPub);

            // Reset the status
            status = BOB_SUCCESS;
        }
        //
        //check to see if we are to run charge maintenance
        //
        if (readResource<bool>(runChargeMutex, runChargeMaintenance))
        {

            // Obtain the battery voltage
            status = getVoltage(voltage, batVoltageClient, statusPub);
            if (status != BOB_SUCCESS)
                continue;

            // Obtain the battery abss
            status = getAbs(voltage, batAbsClient, statusPub);
            if (status != BOB_SUCCESS)
                continue;

            // Check to see if the fuel gauge needs a reset
            status = checkStateOfCharge(voltage, abs);
            //
            // check if we need to run force charge meaning we have an unstable receiver
            // that needs more forceful charge measures
            //
            if (readResource<bool>(forceChargeMutex, forceCharge))
            {
                status = getChargeCurrent(currentResult, chargeCurrentClient, statusPub);

                if (status != BOB_SUCCESS)
                    continue;
                // Obtain the battery voltage
                status = getVoltage(voltageResult, batVoltageClient, statusPub);

                if (status != BOB_SUCCESS)
                    continue;
                //
                //check to see if the current reading are withing the padding
                //
                ROS_ERROR_STREAM("force charge manitance current %d" << currentResult);
                if (currentResult < DEFAULT_CURRENT - CURRENT_PADDING)
                {
                    //
                    //set paramters for forceful charging
                    //
                    status = charger_set_max_charge_voltage(DEFAULT_MAX_VOLTAGE);

                    if (status != BOB_SUCCESS)
                        continue;

                    status = charger_set_charge_current(DEFAULT_CURRENT);

                    if (status != BOB_SUCCESS)
                        continue;
                }
                //
                //check to see if we are getting negative current and publish that we need
                //to move off the charger and back on
                //
                if (currentResult < NULL_VAL)
                {
                    negativeCurrentCount += 1;
                    if (negativeCurrentCount > MAX_NEGATIVE_CURRENT_COUNT)
                    {
                        msg.data = BOB_LOST_CHARGER_SYNC;
                        chargerStatusPub.publish(msg);
                        setResource<bool>(stableCurrentMutex, foundStableCurrent, false);
                        setResource<bool>(forceChargeMutex, forceCharge, false);
                    }
                }
            }
            //
            //check to see if we have found the stable current and should maintain
            //
            else if (readResource<bool>(stableCurrentMutex, foundStableCurrent))
            {
                //
                //set forcharge to false every entering
                //
                setResource<bool>(forceChargeMutex, forceCharge, false);

                status = getChargeCurrent(currentResult, chargeCurrentClient, statusPub);
                if (status != BOB_SUCCESS)
                    continue;
                // Obtain the battery voltage
                status = getVoltage(voltageResult, batVoltageClient, statusPub);

                if (status != BOB_SUCCESS)
                    continue;
                //
                //check our charge trimmer and see if we need to trim the charge current
                //
                if (chargeTrimmer >= SIGNAL_TRIM_CHARGE)
                {
                    stableCurrent -= CURRENT_ADJUSTMENT;
                    chargeTrimmer = NULL_VAL;
                }
                //
                //check to see if the current reading is less than the stable current padding
                //
                ROS_ERROR_STREAM("charge manitance current %d" << currentResult);
                if (currentResult < stableCurrent - CURRENT_PADDING)
                {
                    //set the charging paramters using the stable current
                    //
                    status = charger_set_max_charge_voltage(DEFAULT_MAX_VOLTAGE);
                    if (status != BOB_SUCCESS)
                        continue;

                    status = charger_set_charge_current(stableCurrent);
                    if (status != BOB_SUCCESS)
                        continue;

                    //
                    //signal that we have forced stable current and increment the
                    //charge trimmer counter
                    //
                    chargeTrimmer += 1;
                }
                //
                //check to see if have hit negative current and need to publish that we
                //need to move off the charger and back on
                //
                if (currentResult < NULL_VAL)
                {
                    negativeCurrentCount += 1;
                    if (negativeCurrentCount > MAX_NEGATIVE_CURRENT_COUNT)
                    {
                        msg.data = BOB_LOST_CHARGER_SYNC;
                        chargerStatusPub.publish(msg);

                        setResource<bool>(runChargeMutex, runChargeMaintenance, false);
                        setResource<bool>(stableCurrentMutex, foundStableCurrent, false);
                    }
                }
                //
                //check to see if we are below the stable current and begin to forcefully
                //charge bob
                //
                if (stableCurrent < STABLE_CURRENT_DROP)
                {
                    setResource<bool>(forceChargeMutex, forceCharge, true);
                    setResource<bool>(stableCurrentMutex, foundStableCurrent, false);
                    negativeCurrentCount = NULL_VAL;
                }
            }
            //
            //we have not found the stable current so search for the stable charge current
            //
            else
            {
                status = getMaxChargeCurrent(handle, stableCurrent, chargeCurrentClient);
                setResource<bool>(stableCurrentMutex, foundStableCurrent, true);
            }
        }
        //
        //we are not in charge maintenance so make sure we reset our variables
        //
        else
        {
            setResource<bool>(stableCurrentMutex, foundStableCurrent, false);
            chargeTrimmer = NULL_VAL;
            negativeCurrentCount = NULL_VAL;
        }
        //
        //sleep the loop
        //
        r.sleep();
    }
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, POWER_NODE_NAME);

    //node handle
    ros::NodeHandle handle;

    //error publisher
    ros::Publisher statusPub = handle.advertise<std_msgs::UInt8>("status", 1);
    ros::Publisher pub = handle.advertise<std_msgs::Int8>(addBase2Topic(POWER_NODE_NAME, "charge"), 10);

    // Set bat discon to low
    int ret_value = 0;
    ret_value |= GPIOExport(BAT_DISON_PIN);
    ret_value |= GPIODirection(BAT_DISON_PIN, "out");
    ret_value |= GPIOWrite(BAT_DISON_PIN, VALUE_LOW);
    ret_value |= GPIOUnexport(BAT_DISON_PIN);
    if(ret_value != BOB_SUCCESS)
        handleStatus(BOB_UP2_GPIO_ERROR, &statusPub);

    // Initialize the FTDI device
    BobStatus status = ft4222ConnectI2C(&device, ftHandle);
    handleStatus(status, &statusPub);

    //
    //initialize battery for start of node
    //
    battery_init();


    ros::ServiceServer currentfService = handle.advertiseService<ros_robo_bob::GetCurrent::Request,
                                                                ros_robo_bob::GetCurrent::Response>
                                                                (addBase2Topic(POWER_NODE_NAME, "get_current"),
                                                                boost::bind(currentfServiceCallback, _1, _2));

    ros::ServiceServer batteryService = handle.advertiseService<ros_robo_bob::GetBatteryStatus::Request,
                                                                ros_robo_bob::GetBatteryStatus::Response>
                                                                (addBase2Topic(POWER_NODE_NAME, "get_battery_status"),
                                                                boost::bind(batteryServiceCallback, _1, _2));

    ros::ServiceServer absService = handle.advertiseService<ros_robo_bob::GetAbs::Request,
                                                            ros_robo_bob::GetAbs::Response>
                                                            (addBase2Topic(POWER_NODE_NAME, "get_abs"),
                                                            boost::bind(absServiceCallback, _1, _2));

    ros::ServiceServer cellService = handle.advertiseService<ros_robo_bob::GetCells::Request,
                                                             ros_robo_bob::GetCells::Response>
                                                             (addBase2Topic(POWER_NODE_NAME, "get_cell_voltage"),
                                                             boost::bind(cellServiceCallback, _1, _2));

    ros::ServiceServer chargerService = handle.advertiseService<ros_robo_bob::GetCharge::Request,
                                                               ros_robo_bob::GetCharge::Response>
                                                               (addBase2Topic(POWER_NODE_NAME, "get_charge"),
                                                                boost::bind(chargerServiceCallback, _1, _2));
    ros::ServiceServer voltageService = handle.advertiseService<ros_robo_bob::GetVoltage::Request,
                                                                ros_robo_bob::GetVoltage::Response>
                                                                (addBase2Topic(POWER_NODE_NAME, "get_voltage"),
                                                                boost::bind(voltageServiceCallback, _1, _2));

    ros::ServiceServer resetPowerService = handle.advertiseService<ros_robo_bob::ResetPower::Request,
                                                                ros_robo_bob::ResetPower::Response>
                                                                (addBase2Topic(POWER_NODE_NAME, "reset_power"),
                                                                boost::bind(resetPowerCallback, _1, _2));

    ros::ServiceClient chargeCurrentClient =
        handle.serviceClient<ros_robo_bob::GetCurrent>(addBase2Topic(POWER_NODE_NAME, "get_current"));
    ros::ServiceClient batVoltageClient =
        handle.serviceClient<ros_robo_bob::GetVoltage>(addBase2Topic(POWER_NODE_NAME, "get_voltage"));
    ros::ServiceClient batAbsClient =
        handle.serviceClient<ros_robo_bob::GetAbs>(addBase2Topic(POWER_NODE_NAME, "get_abs"));

    //start services
    ros::ServiceServer chargeService = handle.advertiseService<ros_robo_bob::StartCharge::Request,
                                                               ros_robo_bob::StartCharge::Response>
                                                               (addBase2Topic(POWER_NODE_NAME, "start_to_charge"),
                                                                boost::bind(chargeServiceCallback, _1, _2));

    ros::ServiceServer currentService = handle.advertiseService<ros_robo_bob::SetCurrent::Request,
                                                                ros_robo_bob::SetCurrent::Response>
                                                                (addBase2Topic(POWER_NODE_NAME, "set_current"),
                                                                boost::bind(currentServiceCallback, _1, _2));

    ros::ServiceServer findChargeCurrentService = handle.advertiseService<ros_robo_bob::FindChargeCurrent::Request,
                                                                          ros_robo_bob::FindChargeCurrent::Response>
                                                                          (addBase2Topic(POWER_NODE_NAME, "find_charge_current"),
                                                                          boost::bind(findChargeCurrentServiceCallback, _1, _2, handle, &chargeCurrentClient));

    ros::ServiceServer chargeMaintenanceStateService = handle.advertiseService<ros_robo_bob::ChargeMaintenanceState::Request,
                                                                               ros_robo_bob::ChargeMaintenanceState::Response>
                                                                               (addBase2Topic(POWER_NODE_NAME, "charge_maintenance"),
                                                                                boost::bind(chargeMaintenanceStateCallback, _1, _2));

    ros::AsyncSpinner spinner(2);
    spinner.start();

    // Start charge maintenence
    chargeMaintenance(&statusPub, &chargeCurrentClient, &batVoltageClient, &batAbsClient);
}
