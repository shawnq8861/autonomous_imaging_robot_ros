/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/builtin_int64.h>
#include <tf/transform_broadcaster.h>

/* Package specific includes */
#include "../include/utils.hpp"
#include "../include/bob_definitions.hpp"
#include "../include/motion_control_functions.hpp"

/* Service Includes */
#include <ros_robo_bob/GetPosition.h>

/* Static mutexes */
static std::mutex positionMutex;

/* Static resources */
static float x_position = 0.0;
static float prev_x_pos = 0.0;
static float x_vel = 0.0;
static ros::Publisher x_pos_pub;
static ros::Publisher x_vel_pub;
static ros::Time curr_fast_time;
static ros::Time prev_fast_time;
static float inches_per_rev = (float)PI * (float)ENCODER_WHEEL_DIAMETER;

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @req                  The request from the client                       #
#           @res                  The response object for the client                #
#                                                                                   #
#   Description: Services a request from a client to get the position of the        #
#                robot.                                                             #
#                                                                                   #
************************************************************************************/

bool getPositionCB(ros_robo_bob::GetPosition::Request &req, ros_robo_bob::GetPosition::Response &res)
{
    //Populate the response
    res.status = BOB_SUCCESS;
    res.position = readResource<float>(positionMutex, x_position);

    return true;
}

//
// subscriber callback to obtain the current encoder count
//
void getEncoderCount(const std_msgs::Int64 counter)
{
    curr_fast_time = ros::Time::now();
    long enc_count = counter.data;
    //
    // calculate  position
    //
    // x position is calculated as encoder count * inches per count
    // inches per count = (inches per rev) * (rev per count)
    //                  = (inches per rev) / (count per rev)
    // inches per rev = pi * wheel diameter
    // inches per count = (pi * wheel diameter) / count per rev)
    //
    float x_position_tmp = ((float)enc_count) * ((inches_per_rev)/(float)(COUNTS_PER_REV));
    setResource<float>(positionMutex, x_position, x_position_tmp);
    //
    // calculate x velocity
    //
    x_vel = (x_position_tmp - prev_x_pos) / ((curr_fast_time - prev_fast_time).toSec());
    prev_fast_time = curr_fast_time;
    prev_x_pos = x_position_tmp;
}

int main(int argc, char **argv)
{
    //
    // initialize and declare the node
    //
    ros::init(argc, argv, ODOM_NODE_NAME);
    //
    // declare the handle to the node
    //
    ros::NodeHandle handle;
    //
    // create publisher for odometry message
    //
    ros::Publisher odometry_pub = handle.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = POS_UPDATE_RATE;
    //
    // position data variables
    //
    float y = 0.0;
    float th = 0.0;
    //
    // velocity data variables
    //
    float vy = 0.0;
    float vth = 0.0;
    //
    // variables used in elapsed time calculation
    //
    ros::Time curr_time;
    curr_time = ros::Time::now();
    //
    // subscribe to the counter topic and publish x_position data
    //
    ros::Subscriber enc_count_sub = handle.subscribe(addBase2Topic(LS7366_NODE_NAME, "count"), 1000, &getEncoderCount);

    // Create a service to handle tag scan requests.
    ros::ServiceServer positionService = handle.advertiseService<ros_robo_bob::GetPosition::Request,
                                        ros_robo_bob::GetPosition::Response>(addBase2Topic(ODOM_NODE_NAME, "get_position"),
                                        boost::bind(getPositionCB, _1, _2));
    //
    // start multithreaded spinner to handle main and callback
    //
    ros::AsyncSpinner spinner(2);
    spinner.start();
    //
    // update and publish the odometry
    //
    while (ros::ok())
    {
        //
        // grab time stamp data
        //
        curr_time = ros::Time::now();
        //
        // a quaternion created from yaw, needed for odometry even if not used
        //
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        //
        // publish the transform over tf
        //
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = curr_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = readResource<float>(positionMutex, x_position);
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        //
        // send the transform
        //
        odom_broadcaster.sendTransform(odom_trans);
        //
        // publish the odometry message over ROS
        //
        nav_msgs::Odometry odom;
        odom.header.stamp = curr_time;
        odom.header.frame_id = "odom";
        //
        // set the position
        //
        odom.pose.pose.position.x = readResource<float>(positionMutex, x_position);
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        //
        // set the velocity
        //
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = x_vel;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;
        //
        // publish the message
        //
        odometry_pub.publish(odom);
        //
        // pace the main loop
        //
        loop_rate.sleep();
    }
    ros::waitForShutdown();

    return 0;
}
