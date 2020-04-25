#ifndef ROS_ROBO_BOB_UTILS_H
#define ROS_ROBO_BOB_UTILS_H

/* ROS includes */
#include <ros/ros.h>

/* General C++ includes */
#include <mutex>

/* Package specific includes */
#include "../include/track_plan.hpp"
#include "../include/bob_definitions.hpp"

/* Function declarations */
BobStatus createDataDir();
std::string date2String();
BobStatus createDir(std::string);
void writeChecksumFile(std::string);
bool checkFileIntegrity(std::string);
std::string epoch2ISOTime(std::string);
void movingAverage(float &, float , float);
std::string addBase2Topic(const char *, const char *);
void transformWGS84(double lat1, double lon1, double & lat2, double & lon2, double bearing, double distance);

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @lock                Mutex object to share the resource with            #
#           @resource            The resource to modify                             #
#           @state               The desired state of the resource                  #
#                                                                                   #
#   Description: Sets a resource to a desired state in a thread safe way using a    #
#                std mutex object.                                                  #
#                                                                                   #
************************************************************************************/

template <typename T>
void setResource(std::mutex & lock, T & resource, T state)
{
    lock.lock();
    resource = state;
    lock.unlock();
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @lock                Mutex object to share the resource with            #
#           @resource            The resource to read                               #
#           @state               The desired state of the resource                  #
#                                                                                   #
#   Description: Reads a resource's state in a thread safe way using a              #
#                std mutex object.                                                  #
#                                                                                   #
************************************************************************************/

template <typename T>
T readResource(std::mutex & lock, T & resource)
{

    lock.lock();
    T state = resource;
    lock.unlock();

    return state;
}


#endif
