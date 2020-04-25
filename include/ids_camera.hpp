#ifndef ROS_ROBO_BOB_IDS_CAMERA_H
#define ROS_ROBO_BOB_IDS_CAMERA_H

/* ROS includes */
#include <sensor_msgs/Image.h>

/* Package specific includes */
#include "../include/bob_status.hpp"

/* Function declarations */
BobStatus idsCalibrate();
BobStatus idsCaptureImage(sensor_msgs::ImageConstPtr &image, bool autoExp=false, float expComp=1.);

#endif