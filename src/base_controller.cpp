
/* General c++ includes */
#include <mutex>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>

/* ROS includes */
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

/* MRAA includes */
#include "mraa/common.hpp"
#include "mraa/pwm.hpp"
#include "mraa/gpio.hpp"

/* Package specific includes */
#include "../include/gpio.hpp"
#include "../include/utils.hpp"
#include "../include/motion_control_functions.hpp"

/* Soft PWM parameters */
#define WR_VALUE _IOW('a','a',int32_t*)
#define DDS_MIN_CAPTURE_SPEED 6.
#define SOFTPWM_DEVICE_NAME "/dev/softpwm_device"

/* Mutex definitions */
static std::mutex dutyCycleMutex;
static std::mutex ddsMutex;

/* Static global variables */
static int softpwmFD;
static float velocity_x;
static float dynamicDriveCoeff;
static uint32_t dutyCycle;
static bool motor_direction = FORWARD;
static bool prev_direction = motor_direction;
static bool first_iteration = true;

/* Dynamic drive defitions */
#define DYNAMIC_DRIVE_SCALING_DATA_POINTS 200
#define DYNAMIC_DRIVE_DEFAULT 0.04
#define DYNAMIC_DRIVE_MIN 0.02
#define DYNAMIC_DRIVE_MAX 0.06
#define DUTY_CYCLE_MAX 999
#define ENABLE_DDS

/************************************************************************************
#                                                                                   #
#   Author(s): Shawn Quinn, Ethan Takla                                             #
#   Inputs:                                                                         #
#           @cmd_vel_msg          Command velocity data from the publisher          #
#                                                                                   #
#   Description: Recives velocity commands from a publisher, converts them to a     #
#                duty cycle, and sends them to the soft PWM kenrel module.          #
#                                                                                   #
************************************************************************************/

void receiveVelCmd(const geometry_msgs::Twist::ConstPtr &cmd_vel_msg)
{
    //
    // parse the message for the x velocity component
    //
    velocity_x = (float)cmd_vel_msg->linear.x;

    //
    // get motor direction from sign of velocity
    //
    motor_direction = (velocity_x >= 0.0) ? FORWARD : REVERSE;
    if (prev_direction != motor_direction || first_iteration) {
   
        int ret_value = GPIOWrite(GPIO_PIN_6_TEST_DIR, motor_direction);
        if (ret_value != 0) {
            ROS_ERROR("failed to change direction...");
        }

        first_iteration = false;
    }
   
    prev_direction = motor_direction;
    uint32_t dutyCycleTmp = (uint32_t)(std::abs(velocity_x)/readResource<float>(ddsMutex, dynamicDriveCoeff));

    // Check to make sure the duty cycle is in range
    if(dutyCycleTmp > DUTY_CYCLE_MAX)
        dutyCycleTmp = DUTY_CYCLE_MAX;

    setResource<uint32_t>(dutyCycleMutex, dutyCycle, dutyCycleTmp);

    // Write the duty cycle to the softpwm module
    ioctl(softpwmFD, WR_VALUE, (int32_t*) &dutyCycleTmp); 
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @odom_msg             The odometry data from the publisher              #
#           @ddsDataPoints        Pointer to the dds data point vector              #
#                                                                                   #
#   Description: Uses linear regression to dynamically compute the relationship     #
#                between the PWM duty cycle and the actual robot speed using        #
#                linear regression. This allows the robot to dynamically adapt      #
#                to various drivetrains and battery voltages.                       #
#                                                                                   #
************************************************************************************/

void dynamicDriveCB(const nav_msgs::Odometry::ConstPtr &odom_msg,
                std::vector<std::tuple<float, float> > * ddsDataPoints)
{
    bool dataQueueFull = false;

    #ifdef ENABLE_DDS

    // Only collect data if we're moving
    if(std::abs(odom_msg->twist.twist.linear.x) > DDS_MIN_CAPTURE_SPEED)
    {
        std::tuple<float, float> dataPoint((float)readResource<uint32_t>(dutyCycleMutex, dutyCycle),
                                            std::abs(odom_msg->twist.twist.linear.x));

        // Pop off the top of the data point vector if it is full
        if(ddsDataPoints->size() == DYNAMIC_DRIVE_SCALING_DATA_POINTS)
        {
            dataQueueFull = true;
            ddsDataPoints->erase(ddsDataPoints->begin());
        }

        // Push the data point to the back of the queue
        ddsDataPoints->push_back(dataPoint);

        // Use linear regression to compute the dynamic drive coefficient if we 
        // have enough data
        if(dataQueueFull)
        {
            float xySum = 0.;
            float xxSum = 0.;

            // Compute the sum of component squares, and sum of duty cycle squares
            for(int i = 0; i < DYNAMIC_DRIVE_SCALING_DATA_POINTS; i++)
            {
                xySum += std::get<0>((*ddsDataPoints)[i]) * std::get<1>((*ddsDataPoints)[i]);
                xxSum += std::get<0>((*ddsDataPoints)[i]) * std::get<0>((*ddsDataPoints)[i]);
            }

            // Compute the dynamic drive coefficient
            // dds = sum(di*vi)/sum(di*di) for i points in the data set
            float dds = xySum / xxSum;
   
            if(dds > DYNAMIC_DRIVE_MIN && dds < DYNAMIC_DRIVE_MAX)
                setResource<float>(ddsMutex, dynamicDriveCoeff, xySum / xxSum);
            
        }
    }

    #endif
}

int main(int argc, char **argv)
{
    // Initialize the DDS coefficient
    dynamicDriveCoeff = DYNAMIC_DRIVE_DEFAULT;

    // Vector of data points for dynamic drive scaling
    std::vector<std::tuple<float, float> > ddsDataPoints;

    //
    // initialize and declare the node
    //
    ros::init(argc, argv, "base_controller");
    //
    // declare the handle to the node
    //
    ros::NodeHandle nh;

    // Open the software PWM device
    softpwmFD = open(SOFTPWM_DEVICE_NAME, O_RDWR);

    // Set up the odometry subscriber
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>
                               ("odom", 10, boost::bind(dynamicDriveCB, _1, &ddsDataPoints));

    // Set up the command velocity subscriber
    ros::Subscriber sub = nh.subscribe("cmd_vel", 1, receiveVelCmd);

    int ret_value = GPIOExport(GPIO_PIN_MOT_EN);
    if (ret_value != 0) {
        ROS_ERROR("failed to export GPIO pin...");
    }
    //
    // set GPIO pin 5 direction as output
    //
    ret_value = GPIODirection(GPIO_PIN_MOT_EN, "out");
    if (ret_value != 0) {
        ROS_ERROR("failed to set GPIO pin direction...");
    }
    //
    // write 0 to GPIO pin 5 value to enable the motor
    //
    ret_value = GPIOWrite(GPIO_PIN_MOT_EN, VALUE_LOW);
    if (ret_value != 0) {
        ROS_ERROR("failed to set GPIO pin value...");
    }
    //
    // export GPIO pin 24, the motor 1 direction pin
    //
    ret_value = GPIOExport(GPIO_PIN_6_TEST_DIR);
    if (ret_value != 0) {
        ROS_ERROR("failed to export GPIO pin...");
    }
    //
    // set GPIO pin 24 direction as output
    //
    ret_value = GPIODirection(GPIO_PIN_6_TEST_DIR, "out");
    if (ret_value != 0) {
        ROS_ERROR("failed to set GPIO pin direction...");
    }
    //
    // process the subscribed topic message through the callback and then
    // return to wait for the next message, until the node is killed
    //
    // start multithreaded spinner to handle main and callback
    //
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();

    return 0;
}
