#ifndef ACTION_SERVER_HPP
#define ACTION_SERVER_HPP

/* General c++ includes */
#include <omp.h>
#include <mutex>
#include <time.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <string>

/* Package specific includes */
#include "../include/utils.hpp"
#include "../include/bob_status.hpp"

/* ROS includes */
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <actionlib/server/simple_action_server.h>

/* Action includes */
#include <ros_robo_bob/SeekAction.h>
#include <ros_robo_bob/MoveAction.h>
#include <ros_robo_bob/FindEndAction.h>
#include <ros_robo_bob/FindHomeAction.h>
#include <ros_robo_bob/MoveCameraAction.h>
#include <ros_robo_bob/FindChargerAction.h>
#include <ros_robo_bob/CaptureImageAction.h>

/* Service includes */
#include "ros_robo_bob/FindHome.h"
#include "ros_robo_bob/FindEnd.h"
#include "ros_robo_bob/FindTheCharger.h"
#include "ros_robo_bob/SeekAGoalPosition.h"
#include "ros_robo_bob/MoveToGoalPosition.h"
#include "ros_robo_bob/SaveImage.h"
#include "ros_robo_bob/MoveCamera.h"

/* Constant definitions */
#define SERVER_NODE_NAME "action_server"

/* Typedefs */
typedef actionlib::SimpleActionServer<ros_robo_bob::SeekAction> SeekServer;
typedef actionlib::SimpleActionServer<ros_robo_bob::MoveAction> MoveServer;
typedef actionlib::SimpleActionServer<ros_robo_bob::FindEndAction> FindEndServer;
typedef actionlib::SimpleActionServer<ros_robo_bob::FindHomeAction> FindHomeServer;
typedef actionlib::SimpleActionServer<ros_robo_bob::MoveCameraAction> MoveCameraServer;
typedef actionlib::SimpleActionServer<ros_robo_bob::FindChargerAction> FindChargerServer;
typedef actionlib::SimpleActionServer<ros_robo_bob::CaptureImageAction> CaptureImageServer;

#endif // ACTION_SERVER_HPP
