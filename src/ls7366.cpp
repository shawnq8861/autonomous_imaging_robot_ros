/* General c++ includes */
#include <chrono>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/Int64.h>

/* MRAA Includes */
#include "mraa/common.hpp"
#include "mraa/spi.hpp"

/* Package specific includes */
#include "../include/spi.hpp"
#include "../include/utils.hpp"
#include "../include/ls7366.hpp"
#include "../include/bob_definitions.hpp"
#include "../include/motion_control_functions.hpp"

/* Service includes */
#include "ros_robo_bob/ResetCounter.h"

/* Mutex definitions */
static std::mutex counterMutex;    // Protects the counter resource
static std::mutex tickOffsetMutex; // Protects the tick offset resource

/* Static shared resources */
static long counter;             // Keeps track of the encoder counts
static bool reset;               // True if the counter just got reset
static long tickOffset;          // Used to keep track of position if encoder resets
static float averageSpeed;       // The average speed in ticks/second
static std::chrono::steady_clock::time_point lastUpdateTime;


/* Constant definitions */
#define LS7366_SPI_PORT 1
#define LS7366_UPDATE_HZ 40
#define LS7366_MAX_SPI_RETRIES 3
#define SPEED_AVERAGE_DECAY_FACTOR 0.95
#define LS7366_REGISTER_SLEEP_US 2.
#define SPI_FREQUENCY_BPS 115200
#define LS7366_MAX_SPEED_BUFFER 1.2

/* Namespace definitions */
using namespace boost::interprocess;

/* Named mutex definitions */
named_mutex spiLock(open_or_create, SPI_LOCK_NAME);

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Synapse                                                 #
#   Inputs:                                                                         #
#           @spi                  Pointer to the MRAA SPI instance                  #
#                                                                                   #
#   Description: Resets the LS7366 counter register.                                #
#                                                                                   #
************************************************************************************/

void ls7366_reset(mraa::Spi *spi)
{
    uint8_t txData[1] = {CLR | CNTR};

    // Obtain exclusive access to SPI
    //scoped_lock<named_mutex> lock(spiLock);

    // Reset the chip
    spi->write(txData, 1);
    usleep(LS7366_REGISTER_SLEEP_US);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @spi                  Pointer to the MRAA SPI instance                  #
#   Outputs:                                                                        #
#           @returns              A count value                                     #
#                                                                                   #
#   Description: Retrieves the latest count value from the OTR register.            #
#                                                                                   #
************************************************************************************/

int64_t ls7366_sync(mraa::Spi *spi)
{
    int64_t count;
    uint8_t rxBuffer[5];
    uint8_t txData[5] = {0, 0, 0, 0, 0};

    // Obtain exclusive access to SPI
    //scoped_lock<named_mutex> lock(spiLock);

    txData[0] = LOAD_COUNTER | OTR;
    spi->write(txData, 1);

    usleep(LS7366_REGISTER_SLEEP_US);

    txData[0] = RD | CNTR;
    spi->transfer(txData, rxBuffer, 5);

    count = ((rxBuffer[1]) << 24) | ((rxBuffer[2]) << 16) | ((rxBuffer[3]) << 8) | (rxBuffer[4]);

    return count;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @spi                  Pointer to the MRAA SPI instance                  #
#                                                                                   #
#   Description: Checks to see if the registers have the values they were written.  #
#                If there is a mismatch, the chip most likely reset.                #
#                                                                                   #
************************************************************************************/

bool ls7366_check_registers(mraa::Spi *spi)
{
    uint8_t txData[2];
    uint8_t rxBuffer[2];

    bool MDR0_OK = false;
    bool MDR1_OK = false;

    // Obtain exclusive access to SPI
    //scoped_lock<named_mutex> lock(spiLock);
    
    for(int i = 0; i < LS7366_MAX_SPI_RETRIES && !(MDR0_OK && MDR1_OK); i++)
    {
        // Read the MDR0 register and make sure it matches the config
        txData[0] = RD | MDR0;
        spi->transfer(txData, rxBuffer, 2);

        if((int)rxBuffer[1] != (int)MDR0_CONF)
            continue;

        MDR0_OK = true;
        
        // Read the MDR1 register and make sure it matches the config
        txData[0] = RD | MDR1;
        spi->transfer(txData, rxBuffer, 2);

        if((int)rxBuffer[1] != (int)MDR1_CONF)
            continue;
        
        MDR1_OK = true;
    }

    // Only return true if both registers contained the correct values
    return MDR0_OK && MDR1_OK;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @spi                  Pointer to the MRAA SPI instance                  #
#                                                                                   #
#   Description: Initializes the LS7366 device for normal operation.                #
#                                                                                   #
************************************************************************************/

void ls7366_init(mraa::Spi *spi)
{
    uint8_t txData[2];

    // Obtain exclusive access to SPI
    //scoped_lock<named_mutex> lock(spiLock);

    txData[0] = WR | MDR0;
    txData[1] = MDR0_CONF;

    // Config MDR0
    spi->write(txData, 2);
    usleep(LS7366_REGISTER_SLEEP_US);

    txData[0] = WR | MDR1;
    txData[1] = MDR1_CONF;

    // Config MDR1
    spi->write(txData, 2);
    usleep(LS7366_REGISTER_SLEEP_US);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @event                ROS timer event, not used                         #
#           @spi                  Pointer to the MRAA SPI instance                  #
#                                                                                   #
#   Description: A timer callback that periodically publishes the encoder value.    #
#                                                                                   #
************************************************************************************/

void encoderPublishCB(const ros::TimerEvent &event, ros::Publisher *counterPub, mraa::Spi *spi)
{
    int retries;
    long newCount;
    float speed, dt;
    std_msgs::Int64 msg;

    // Get the tick offset
    long offset = readResource<long>(tickOffsetMutex, tickOffset);

    // Get the old encoder count
    long oldCount = readResource<long>(counterMutex, counter);
    
    // Attempt to get data from the encoder w/ retires
    for (retries = 0; retries < LS7366_MAX_SPI_RETRIES; retries++)
    {
        // Read the new encoder count
        newCount = -ls7366_sync(spi) + offset;

        // Get the current time
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();

        // Compute the time in between the current and last call
        dt = std::chrono::duration_cast<std::chrono::duration<float> >(currentTime - lastUpdateTime).count();
        
        // Compute the speed at the current timestep
        speed = ((((float)(newCount - oldCount)) / dt) /COUNTS_PER_REV)*ENCODER_WHEEL_DIAMETER*M_PI;

        if (std::abs(speed) < MAX_VELO_IN_SEC*LS7366_MAX_SPEED_BUFFER)
        {
            // Update the moving average of speed
            movingAverage(averageSpeed, speed, SPEED_AVERAGE_DECAY_FACTOR);

            // Update the last time value
            lastUpdateTime = currentTime;
            
            break;
        }
    }
    
    bool OK = ls7366_check_registers(spi);

    // Reset the encoder and use the last known count if it got reset, 
    // or if we got a value that exceeds the maximum expected tick delta rate
    if (!OK || retries == 3)
    {
        ROS_ERROR("LS7366 RESET %ld %ld %f %f %f, %d %d", oldCount, newCount, dt, averageSpeed, speed, retries, (int)OK);

        // Adjust the offset based on the last known value and the last known speed
        setResource(tickOffsetMutex, tickOffset, oldCount+(long)(averageSpeed*dt));
        newCount = oldCount;

        // Re-init the device
        ls7366_init(spi);
        ls7366_reset(spi);
    }

    setResource<long>(counterMutex, counter, newCount);
    msg.data = newCount;

    // Publish the count
    counterPub->publish(msg);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @request              The service request object                        #
#           @response             The service response object                       #
#                                                                                   #
#   Description: Resets the LS7366 counter register.                                #
#                                                                                   #
************************************************************************************/

bool resetCounterCB(ros_robo_bob::ResetCounterRequest &request,
                    ros_robo_bob::ResetCounterResponse &response, mraa::Spi *spi)
{
    response.reset = true;

    // Reset counting resources
    setResource<long>(counterMutex, counter, 0);
    setResource<long>(tickOffsetMutex, tickOffset, 0);

    // Reset the buffer's counter
    ls7366_reset(spi);

    return true;
}

int main(int argc, char **argv)
{
    // Set static variables
    counter = 0;
    tickOffset = 0;
    averageSpeed = 0.;

    // Initialize the encoer buffer node
    ros::init(argc, argv, LS7366_NODE_NAME);

    // The ros handle for this node
    ros::NodeHandle handle;

    // Initialize the SPI instance
    mraa::Spi spi(LS7366_SPI_PORT);

    // Set the SPI frequency
    spi.frequency(SPI_FREQUENCY_BPS);

    // Initialize the encoder device
    ls7366_init(&spi);

    // Sync the counter
    counter = -ls7366_sync(&spi);

    // Initialize the last update time
    lastUpdateTime = std::chrono::steady_clock::now();

    // Create a publisher to publish the encoder counts
    ros::Publisher counterPub = handle.advertise<std_msgs::Int64>(addBase2Topic(LS7366_NODE_NAME, "count"), 1);

    // Create a service to reset the counter
    ros::ServiceServer resetCounterService = handle.advertiseService<ros_robo_bob::ResetCounter::Request,
                                                                     ros_robo_bob::ResetCounter::Response>(addBase2Topic(LS7366_NODE_NAME, "reset"),
                                                                                                           boost::bind(resetCounterCB, _1, _2, &spi));

    // Create a timer to periodically publish the encoder value
    ros::Timer counterTimer = handle.createTimer(ros::Duration(1. / LS7366_UPDATE_HZ),
                                                 boost::bind(encoderPublishCB, _1, &counterPub, &spi));

    ros::spin();
}
