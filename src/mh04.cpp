/* General C++ includes */
#include <mutex>
#include <string.h>
#include <sys/ioctl.h>

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/Bool.h>

/* Service includes */
#include <ros_robo_bob/FindHome.h>

/* Package specific includes */
#include "../include/gpio.hpp"
#include "../include/utils.hpp"
#include "../include/bob_status.hpp"
#include "../include/bob_definitions.hpp"

using namespace std;

/* Hall effect device parameters */
#define WR_VALUE _IOW('a','a',int32_t*)
#define RD_VALUE _IOR('a','b',int32_t*)

/* Constant definitions */
#define HALL_EFFECT_POLL_HZ 100
#define HALL_EFFECT_DEVICE_NAME "/dev/halleffect_device"

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @hallEffectPub        Pointer to the hall effect state publisher        #
#           @hallEffectFD         File descriptor to the hall effect kernel module  #
#                                 device                                            #
#                                                                                   #
#   Description: Periodically polls the hall effect kernel module and publishes     #
#                when it has been found.                                            #
#                                                                                   #
************************************************************************************/

void pollHallEffect(ros::Publisher * hallEffectPub, int hallEffectFD)
{
    // Temporaily stores the hall effect state
    uint32_t activated;

    // The message to publish upon hall effect activation
    std_msgs::Bool activatedMessage;
    
    // Poll the hall effect kernel module at a specic rate
    ros::Rate rate(HALL_EFFECT_POLL_HZ);

    // Poll the state of the hall effect
    while(ros::ok())
    {
        // Read the hall effect state
        ioctl(hallEffectFD, RD_VALUE, (int32_t*) &activated);

        // Publish that the hall effect was activated when the kernel module
        // indicates it
        if(activated)
        {
            ROS_ERROR("ACTIVATED");

            activatedMessage.data = true;
            hallEffectPub->publish(activatedMessage);

            // Reset the hall effect state in the kernel module
            activated = 0;
            ioctl(hallEffectFD, WR_VALUE, (int32_t*) &activated); 
        }

        rate.sleep();
    }

    // Close the hall effect device
    close(hallEffectFD);
}

int main(int argc, char **argv)
{
    // Initialize the action server node
    ros::init(argc, argv, MH04_NODE_NAME);

    // The ros handle for this node
    ros::NodeHandle handle;

    // Open the hall effect device
    int halleffectFD = open(HALL_EFFECT_DEVICE_NAME, O_RDWR);

    // The publisher for the hall effect state
    ros::Publisher hallEffectPub = handle.advertise<std_msgs::Bool>("home", 1);
    
    // Start the hall effect polling loop
    pollHallEffect(&hallEffectPub, halleffectFD);
}
