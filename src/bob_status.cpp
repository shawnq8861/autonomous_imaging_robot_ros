/* General c++ includes */
#include <string>

/* ROS includes */
#include <std_msgs/UInt8.h>

/* Package specific includes */
#include "../include/bob_status.hpp"

void printStatus(BobStatus status)
{
    if(status != BOB_SUCCESS)
        ROS_ERROR("%s", BOB_STATUS_MESSAGES[status]);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @status               The status to publish                             #
#           @planbStatusPub       The publisher for the status message              #
#                                                                                   #
#   Description: Publishes a status message if the status isn't a BOB_SUCCESS.      #
#                                                                                   #
************************************************************************************/

void handleStatus(BobStatus status, ros::Publisher *statusPub)
{
    std_msgs::UInt8 statusMessage;
    statusMessage.data = status;
    
    if(status != BOB_SUCCESS)
        statusPub->publish(statusMessage);      
}