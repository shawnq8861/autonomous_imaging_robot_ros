//
// This node:
//              1. computes a trajectory
//              2. executes the trajectory from a timer callback
//              3. subscribes to Odometry to get position updates
//              4. publishes velocity commands to control motor
//              5. provides a service to move_to_goal_position
//              6. performs position maintenance after a move
//              7. performs stall detection during a move
//

//
// general header files to include
//
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <string>
#include <chrono>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread/thread.hpp>

/* Action includes */
#include <ros_robo_bob/SeekAction.h>
#include <ros_robo_bob/MoveAction.h>
#include <ros_robo_bob/FindEndAction.h>
#include <ros_robo_bob/FindHomeAction.h>
#include <ros_robo_bob/MoveCameraAction.h>
#include <ros_robo_bob/FindChargerAction.h>
#include <ros_robo_bob/CaptureImageAction.h>

//
// package specific header files to include
//
#include <ros_robo_bob/GetCurrent.h>
#include "ros_robo_bob/MoveToGoalPosition.h"
#include "ros_robo_bob/SeekAGoalPosition.h"
#include "ros_robo_bob/FindTheCharger.h"
#include "ros_robo_bob/FindHome.h"
#include "ros_robo_bob/FindEnd.h"
#include "ros_robo_bob/FindChargeCurrent.h"
#include "ros_robo_bob/ResetCounter.h"
#include "../include/motion_control_functions.hpp"
#include "../include/action_server.hpp"
#include "../include/utils.hpp"
#include "../include/bob_status.hpp"
#include "../include/bob_definitions.hpp"

//
// these buffers hold the incremetal changes required to execute the trajectory
//
static float cmdVelocityBuffer[BUFFER_SIZE];
static float desiredPostionBuffer[BUFFER_SIZE];
//
// IPC mutexes
//
static std::mutex exec_mtx;
static std::mutex main_mtx;
static std::mutex stall_mtx;
static std::mutex hall_mtx;
static std::mutex pos_mtx;
static std::mutex vel_mtx;
static std::mutex preempt_mtx;
static std::mutex moving_mtx;
static std::mutex stepping_mode_mtx;

//
// global variables used in more than one function
//
static bool executing_trajectory = false;
static bool moving = false;
static int stallCount = 0;
static bool stall_detected = false;
static const std::string move_complete_string = "move complete";
static const std::string stall_detected_string = "stall detected";
static const std::string preempt_rec_string = "preempt received";
static bool finding_hall_effect = false;
static bool found_hall = false;
static bool hall_triggered = false;
static bool preempt_received = false;
static bool stepping_mode = false;
static bool stopped_trajectory = false;

//
// position and velocity parameters
//
static float cmd_vel = 0.0;
static float current_velocity = 0.0;
static float previous_velocity = 0.0;
static float current_position = 0.0;
static float previous_position = 0.0;
static std::chrono::steady_clock::time_point lastUpdateTime;
//
// incremental requested change in position
//
static float goal_distance = 0.0;
//
// absolute goal position
//
static float goal_position = 0.0;
//
// starting point for next trajectory
//
static float start_position = 0.0;
//
// stall detection parameters
//
static float moving_stall_window = 0.0;
static const float stall_window_size = 20.0;
//
// constants
//
static const float position_threshold = 0.020;
static const float stall_threshold = .000001;
static const float cmd_vel_threshold = (float)STALL_VEL_IN_SEC;
static const int charger_scan_step_count = 25;
static const float charger_scan_step_size = 0.25;
static const float max_seek_distance = 5000.0;

// The time to wait in between inching moves
#define FIND_CHARGER_STEP_TIME 4

// The number of positive current readings required to
// consider the charger found
#define FIND_CHARGER_NUM_STOP_DETECTIONS 1

// The frequency in which to check state variables during actions
#define ACTION_POLL_HZ 50

// The frequency at which to search for a current
#define CURRENT_SEARCH_HZ 10

// Multiply that defines the sanity range of dt, based on the expected update period
#define DT_SANITY_RANGE 10

//
// used to turn debug on or off
//
#define DEBUG false

void clearBuffers()
{
    memset(cmdVelocityBuffer, 0, BUFFER_SIZE);
    memset(desiredPostionBuffer, 0, BUFFER_SIZE);
}

void stopTrajectory()
{
    // The rate at which to check for move completion
    ros::Rate loop_rate = ACTION_POLL_HZ;

    // Wait for the trajectory to stop
    do
    {
        // Stop any motion
        setResource<bool>(exec_mtx, executing_trajectory, false);
                
        loop_rate.sleep();
    }
    while(!readResource<bool>(exec_mtx, stopped_trajectory));
    
    // Reset the trajectory buffers
    clearBuffers();
}

void resetEncoderCounts(ros::NodeHandle *nh_ptr)
{
    stopTrajectory();
    
    setResource<float>(vel_mtx, cmd_vel, 0.0);
    setResource<float>(pos_mtx, current_position, MAGNET_LOCATION_IN);
    setResource<float>(pos_mtx, previous_position, MAGNET_LOCATION_IN);
    setResource<float>(pos_mtx, current_velocity, 0);
    setResource<float>(pos_mtx, previous_velocity, 0);
    
    //
    // call the reset counter service
    //
    ros::ServiceClient reset_counter_client =
        nh_ptr->serviceClient<ros_robo_bob::ResetCounter>("ls7366/reset");
    ros_robo_bob::ResetCounter counter_srv;
    if (reset_counter_client.call(counter_srv))
    {
        if (counter_srv.response.reset)
        {
            if (DEBUG)
                ROS_ERROR_STREAM("counter reset...");
        }
    }
    else
    {
        if (DEBUG)
            ROS_ERROR_STREAM("could not call reset counter service...");
    }
}

//
// service callback used to set new target position, compute trajectory,
// and move to new target position
//
bool moveToGoalPosition(
    ros_robo_bob::MoveToGoalPositionRequest &req_goal_position,
    ros_robo_bob::MoveToGoalPositionResponse &move_result,
    MoveServer *server)
{
       
    std::string msg;
    
    // Ignore any previous preemption requests
    setResource<bool>(preempt_mtx, preempt_received, false);
    
    // The rate at which to check for move completion
    ros::Rate loop_rate = ACTION_POLL_HZ;

    // Initialize state variables
    setResource<bool>(hall_mtx, finding_hall_effect, false);
    setResource<bool>(hall_mtx, hall_triggered, false);
    setResource<bool>(hall_mtx, found_hall, false);
    setResource<int>(exec_mtx, trajectoryIndex, 0);

    // Obtain goal variables from the message
    setResource<float>(pos_mtx, goal_distance, req_goal_position.goal_position);
    setResource<float>(pos_mtx, goal_position, (current_position + goal_distance));
    
    // Don't do anything if we need to move zero
    if (req_goal_position.goal_position == 0.0)
    {
        move_result.result = BOB_SUCCESS;
        move_result.position = readResource<float>(pos_mtx, current_position);
        return true;
    }
    
    float speedReduction = move_speed_ratio;
    
    // Compute the trajectory
    int trajectorySize = computeTrajectory(cmdVelocityBuffer,
                                           desiredPostionBuffer,
                                           goal_distance,
                                           speedReduction,
                                           readResource<float>(pos_mtx, current_position));
    
    // Reset stall count variables
    setResource<int>(stall_mtx, stallCount, 0);
    setResource<float>(stall_mtx, moving_stall_window, 0.0);
    
    // Execute the trajectory
    setResource<bool>(exec_mtx, executing_trajectory, true);

    // Wait for the move to complete
    while (readResource<bool>(exec_mtx, executing_trajectory) && !readResource<bool>(stall_mtx, stall_detected)
           && !readResource<bool>(preempt_mtx, preempt_received) && ros::ok())
        loop_rate.sleep();
    
    // Check for preemption
    if (readResource<bool>(preempt_mtx, preempt_received))
    {
        stopTrajectory();
        
        move_result.position = readResource<float>(pos_mtx, current_position);
        move_result.result = BOB_ACTION_CANCELLED;
        setResource<bool>(preempt_mtx, preempt_received, false);
            
        return true;
    }
    
    // Check for a stall
    if (readResource<bool>(stall_mtx, stall_detected))
    {
        stopTrajectory();
        
        move_result.result = BOB_UNEXPECTED_STOP;
        setResource<bool>(stall_mtx, stall_detected, false);
        return true;
    }
    else
    {
        move_result.result = BOB_SUCCESS;
    }
    
    // Reset state variables
    setResource<int>(stall_mtx, stallCount, 0);
    setResource<float>(stall_mtx, moving_stall_window, 0.0);
    
    // Populate position in the response
    move_result.position = readResource<float>(pos_mtx, current_position);

    return true;
}

//
// service callback used to set new absolute position, compute trajectory,
// and move to new absolute position
//
bool seekAGoalPosition(
    ros_robo_bob::SeekAGoalPositionRequest &req_goal_position,
    ros_robo_bob::SeekAGoalPositionResponse &seek_result,
    SeekServer *server)
{

    std::string msg;
    
     // Ignore any previous preemption requests
    setResource<bool>(preempt_mtx, preempt_received, false);
    
    // The rate at which to check for move completion
    ros::Rate loop_rate = ACTION_POLL_HZ;
    
    // Initialize state variables
    setResource<bool>(hall_mtx, finding_hall_effect, false);
    setResource<bool>(hall_mtx, hall_triggered, false);
    setResource<bool>(hall_mtx, found_hall, false);
    setResource<int>(exec_mtx, trajectoryIndex, 0);

    // Obtain goal variables from the message
    setResource<float>(pos_mtx, goal_distance, (req_goal_position.goal_position - current_position));
    setResource<float>(pos_mtx, goal_position, req_goal_position.goal_position);
    
    // Only move if the distance is greater than some threshold
    if (std::abs(readResource<float>(pos_mtx, goal_distance)) < position_threshold)
    {
        seek_result.result = BOB_SUCCESS;
        seek_result.position = readResource<float>(pos_mtx, current_position);
        return true;
    }
    
    float speedReduction = move_speed_ratio;
    
    // Compute the trajectory
    int trajectorySize = computeTrajectory(cmdVelocityBuffer,
                                           desiredPostionBuffer,
                                           goal_distance,
                                           speedReduction,
                                           readResource<float>(pos_mtx, current_position));
    
    // Reset stall variables
    setResource<int>(stall_mtx, stallCount, 0);
    setResource<float>(stall_mtx, moving_stall_window, 0.0);

    // Execute the trajectory
    setResource<bool>(exec_mtx, executing_trajectory, true);

    // Wait for the move to complete
    while (readResource<bool>(exec_mtx, executing_trajectory) && !readResource<bool>(stall_mtx, stall_detected)
           && !readResource<bool>(preempt_mtx, preempt_received)&& ros::ok())
        loop_rate.sleep();
    
    // Check for preemption
    if (readResource<bool>(preempt_mtx, preempt_received))
    {
        stopTrajectory();
        
        seek_result.result = BOB_ACTION_CANCELLED;
        seek_result.position = readResource<float>(pos_mtx, current_position);
        setResource<bool>(preempt_mtx, preempt_received, false);

        return true;
    }
    
    // Check for a stall
    if (readResource<bool>(stall_mtx, stall_detected))
    {
        stopTrajectory();
        
        seek_result.result = BOB_UNEXPECTED_STOP;
        setResource<bool>(stall_mtx, stall_detected, false);
        return true;
    }
    else
    {
        seek_result.result = BOB_SUCCESS;
    }
    
    // Reset stall variables
    setResource<int>(stall_mtx, stallCount, 0);
    setResource<float>(stall_mtx, moving_stall_window, 0.0);
    
    // Populate current position in the response object
    seek_result.position = readResource<float>(pos_mtx, current_position);

    return true;
}

//
// service callback used to set new target position, compute trajectory,
// and move to the charger position at reduced velocity
//
bool findTheCharger(
    ros_robo_bob::FindTheChargerRequest &find_charger_request,
    ros_robo_bob::FindTheChargerResponse &find_charger_result,
    FindChargerServer *server, ros::NodeHandle *nh_ptr, ros::Publisher *cmd_vel_pub)
{
    std::string msg;
    BobStatus status;
    
    // Ignore any previous preemption requests
    setResource<bool>(preempt_mtx, preempt_received, false);
    
    // Number of moves that a valid charge current has been detected
    int currentSearchTicks = 0;
    
    // Number of times a current has been detected within a single step
    int currentDetections = 0;
    
    // True when a charge current has been found
    bool foundCurrent = false;
    
    // Wait for moves to complete at a specific rate
    ros::Rate loop_rate = ACTION_POLL_HZ;
    
    // Poll the charge current at a certain rate
    ros::Rate current_search_rate = CURRENT_SEARCH_HZ;
    
    // Initialize the response message with defaults
    find_charger_result.found_charger = false;
    find_charger_result.result = BOB_CANNOT_FIND_CHARGER;

    // Initalize the charge current service client
    ros::ServiceClient find_charge_current_client =
        nh_ptr->serviceClient<ros_robo_bob::FindChargeCurrent>("power_node/find_charge_current");
    
    // Turn off hall effect search if it enabled
    setResource<bool>(hall_mtx, finding_hall_effect, false);
    
    // Used to call the current service
    ros_robo_bob::FindChargeCurrent curr_srv;

    // Step a finite number of times in search of the charger
    for (int step = 0; step < charger_scan_step_count; ++step)
    {
        // Reset the trajectory
        setResource<int>(exec_mtx, trajectoryIndex, 0);
        
        // Turn on stepping mode
        setResource<bool>(stepping_mode_mtx, stepping_mode, true);

        float speedReduction = move_speed_ratio;
        
        // Compute the trajectory
        int trajectorySize = computeTrajectory(cmdVelocityBuffer,
                                               desiredPostionBuffer,
                                               charger_scan_step_size,
                                               speedReduction,
                                               readResource<float>(pos_mtx, current_position));

        // Initialize stall variables
        setResource<int>(stall_mtx, stallCount, 0);
        setResource<float>(stall_mtx, moving_stall_window, 0.0);
        
        // Start executing the trajectory
        setResource<bool>(exec_mtx, stopped_trajectory, false);
        setResource<bool>(exec_mtx, executing_trajectory, true);

        // Wait for the move to complete
        while (readResource<bool>(exec_mtx, executing_trajectory) && !readResource<bool>(stall_mtx, stall_detected)
                && !readResource<bool>(preempt_mtx, preempt_received) && ros::ok())
            loop_rate.sleep();

        // Turn off stepping mode
        setResource<bool>(stepping_mode_mtx, stepping_mode, false);
        
        // Check for a stall
        if (readResource<bool>(stall_mtx, stall_detected))
        {
            stopTrajectory();
            
            find_charger_result.result = BOB_UNEXPECTED_STOP;
            setResource<bool>(stall_mtx, stall_detected, false);
            return true;
        }
        
        // Check for a preemption
        if (readResource<bool>(preempt_mtx, preempt_received))
        {
            stopTrajectory();
            
            find_charger_result.result = BOB_ACTION_CANCELLED;
            setResource<bool>(preempt_mtx, preempt_received, false);

            return true;
        }

        // Initialize the current search ticks for this iteration
        currentSearchTicks = 0;
        
        // Reset the found current state
        foundCurrent = false;

        // Attempt to get a charge current during the step time window
        while (currentSearchTicks < int(FIND_CHARGER_STEP_TIME * CURRENT_SEARCH_HZ) && !foundCurrent)
        {
            // Only continue if the service call was successfull
            if (find_charge_current_client.call(curr_srv))
            {
                status = curr_srv.response.status;

                // Exit if we encountered an error reading the current
                if (status != BOB_SUCCESS)
                {
                    find_charger_result.result = status;
                    setResource<int>(stall_mtx, stallCount, 0);
                    setResource<float>(stall_mtx, moving_stall_window, 0.0);
                    return true;
                }

                // Increase the current detection ticks if we've found a current
                if (curr_srv.response.current)
                {
                    currentDetections++;
                    foundCurrent = true;
                }
            }
            else
            {
                find_charger_result.result = BOB_CURRENT_SERVICE_FAILED;
                return true;
            }

            current_search_rate.sleep();
            currentSearchTicks++;
        }

        // Check to see if we've reached to stop criteria
        if (currentDetections == FIND_CHARGER_NUM_STOP_DETECTIONS)
        {
            find_charger_result.found_charger = true;
            find_charger_result.result = BOB_SUCCESS;
            break;
        }
    }
    
    return true;
}

bool findTheChargerWithBumper(
    ros_robo_bob::FindTheChargerRequest &find_charger_request,
    ros_robo_bob::FindTheChargerResponse &find_charger_result,
    FindChargerServer *server, ros::NodeHandle *nh_ptr, ros::Publisher *cmd_vel_pub)
{
     ros::Rate loop_rate = ACTION_POLL_HZ;

    // Ignore any previous preemption requests
    setResource<bool>(preempt_mtx, preempt_received, false);
    
    // Initialize service variables to default
    find_charger_result.found_charger = false;
    find_charger_result.result = BOB_CANNOT_FIND_CHARGER;
        
    // Reset the trajectory
    setResource<int>(exec_mtx, trajectoryIndex, 0);

    // Compute the speed reduction
    float speedReduction = FIND_CHARGER_VELOCITY / MAX_VELO_IN_SEC;
        
    // Compute the trajectory
    int trajectorySize = computeTrajectory(cmdVelocityBuffer,
                                           desiredPostionBuffer,
                                           -MAX_TRACK_LENGTH,
                                           speedReduction,
                                           readResource<float>(pos_mtx, current_position));

    // Initialize stall variables
    setResource<int>(stall_mtx, stallCount, 0);
    setResource<float>(stall_mtx, moving_stall_window, 0.0);
        
    // Start executing the trajectory
    setResource<bool>(exec_mtx, stopped_trajectory, false);
    setResource<bool>(exec_mtx, executing_trajectory, true);

    // Wait for the move to complete
    while (readResource<bool>(exec_mtx, executing_trajectory) && !readResource<bool>(stall_mtx, stall_detected)
            && !readResource<bool>(preempt_mtx, preempt_received)
            && ros::ok())
        loop_rate.sleep();
    
    // Move on if we just got a stall
    if (readResource<bool>(stall_mtx, stall_detected))
    {       
        stopTrajectory();
            
        // Reset the stall flag
        setResource<bool>(stall_mtx, stall_detected, false);
        
        find_charger_result.result = BOB_SUCCESS;
        find_charger_result.found_charger = true;
    }
    // Check to see if the action was cancelled
    else if (readResource<bool>(preempt_mtx, preempt_received))
    {
        stopTrajectory();
            
        // Indicate that the action has been cancelled
        find_charger_result.result = BOB_ACTION_CANCELLED;

        // Reset the preemption flag
        setResource<bool>(preempt_mtx, preempt_received, false);
    }
    
    stopTrajectory();
    
    return true;
}

//
// service callback used to locate the hall effect sensor and reset
// position to zero.
//
bool findHome(ros_robo_bob::FindHomeRequest &find_home_request,
              ros_robo_bob::FindHomeResponse &find_home_result,
              ros::NodeHandle *nh_ptr)
{
    ros::Rate loop_rate = ACTION_POLL_HZ;

    // Ignore any previous preemption requests
    setResource<bool>(preempt_mtx, preempt_received, false);
    
    // Initialize service variables to default
    find_home_result.home = false;
    find_home_result.status = BOB_CANNOT_FIND_HOME;

    // Initialize the target velocity
    float direction = -1;

    for (int i = 0; i < 2; i++)
    {
        // Initialize the state variables
        setResource<bool>(hall_mtx, finding_hall_effect, true);
        setResource<bool>(hall_mtx, hall_triggered, false);
        
        // Reset the trajectory
        setResource<int>(exec_mtx, trajectoryIndex, 0);

        float speedReduction = FIND_HOME_VELOCITY / MAX_VELO_IN_SEC;
        
        // Compute the trajectory
        int trajectorySize = computeTrajectory(cmdVelocityBuffer,
                                               desiredPostionBuffer,
                                               direction*MAX_TRACK_LENGTH,
                                               speedReduction,
                                               readResource<float>(pos_mtx, current_position));

        // Initialize stall variables
        setResource<int>(stall_mtx, stallCount, 0);
        setResource<float>(stall_mtx, moving_stall_window, 0.0);
        
        // Start executing the trajectory
        setResource<bool>(exec_mtx, stopped_trajectory, false);
        setResource<bool>(exec_mtx, executing_trajectory, true);

        // Wait for the move to complete
        while (readResource<bool>(exec_mtx, executing_trajectory) && !readResource<bool>(stall_mtx, stall_detected)
                && !readResource<bool>(preempt_mtx, preempt_received) && !readResource<bool>(hall_mtx, hall_triggered)
                && ros::ok())
            loop_rate.sleep();

        // Move on if we just got a stall
        if (readResource<bool>(stall_mtx, stall_detected))
        {
            // Temporarily disable hall effect searching flag
            setResource<bool>(hall_mtx, finding_hall_effect, true);
            
            stopTrajectory();
            
            // Reset the stall flag
            setResource<bool>(stall_mtx, stall_detected, false);

            // Invert the target velocity
            direction = 1;
            continue;
        }
        // Check if we found the hall effect
        else if (readResource<bool>(hall_mtx, hall_triggered))
        {
            // Indicate that the action was a success
            find_home_result.status = BOB_SUCCESS;
            find_home_result.home = true;
            
            break;
        }
        // Check to see if the action was cancelled
        else if (readResource<bool>(preempt_mtx, preempt_received))
        {
            stopTrajectory();
            
            // Indicate that the action has been cancelled
            find_home_result.status = BOB_ACTION_CANCELLED;

            // Reset the preemption flag
            setResource<bool>(preempt_mtx, preempt_received, false);
            
            break;
        }
    }
    
    setResource<bool>(hall_mtx, finding_hall_effect, false);
    
    stopTrajectory();
    
    return true;
}

bool findEnd(ros_robo_bob::FindEndRequest &find_end_request,
              ros_robo_bob::FindEndResponse &find_end_result,
              ros::NodeHandle *nh_ptr)
{
    ros::Rate loop_rate = ACTION_POLL_HZ;

    // Ignore any previous preemption requests
    setResource<bool>(preempt_mtx, preempt_received, false);
    
    // Initialize service variables to default
    find_end_result.status = BOB_SUCCESS;

    // Initialize the target direction
    float direction = 1;
        
    // Reset the trajectory
    setResource<int>(exec_mtx, trajectoryIndex, 0);

    float speedReduction = FIND_HOME_VELOCITY / MAX_VELO_IN_SEC;
        
    // Compute the trajectory
    int trajectorySize = computeTrajectory(cmdVelocityBuffer,
                                           desiredPostionBuffer,
                                           direction*MAX_TRACK_LENGTH,
                                           speedReduction,
                                           readResource<float>(pos_mtx, current_position));

    // Initialize stall variables
    setResource<int>(stall_mtx, stallCount, 0);
    setResource<float>(stall_mtx, moving_stall_window, 0.0);
        
    // Start executing the trajectory
    setResource<bool>(exec_mtx, stopped_trajectory, false);
    setResource<bool>(exec_mtx, executing_trajectory, true);

    // Wait for the move to complete
    while (readResource<bool>(exec_mtx, executing_trajectory) && !readResource<bool>(stall_mtx, stall_detected)
            && !readResource<bool>(preempt_mtx, preempt_received) && ros::ok())
        loop_rate.sleep();

    // Move on if we just got a stall
    if (readResource<bool>(stall_mtx, stall_detected))
    {
        // Reset the stall flag
        setResource<bool>(stall_mtx, stall_detected, false);

    }
    // Check to see if the action was cancelled
    else if (readResource<bool>(preempt_mtx, preempt_received))
    {      
        // Indicate that the action has been cancelled
        find_end_result.status = BOB_ACTION_CANCELLED;

        // Reset the preemption flag
        setResource<bool>(preempt_mtx, preempt_received, false);            

    }
    
    stopTrajectory();
    
    return true;
}

//
// subscribe to odometry to receive message for current x position and velocity
// updates stall count if position is not changing
//
void getOdometryCheckStall(const nav_msgs::Odometry::ConstPtr &odom_msg, ros::Publisher *cmd_vel_pub)
{
    geometry_msgs::Twist cmd_vel_msg;

    setResource<float>(pos_mtx, previous_position, current_position);
    setResource<float>(vel_mtx, previous_velocity, current_velocity);
    setResource<float>(vel_mtx, current_velocity, odom_msg->twist.twist.linear.x);
    setResource<float>(pos_mtx, current_position, odom_msg->pose.pose.position.x);
    //
    // update moving stall window, add latest move increment
    //
    float pos_delta = readResource<float>(pos_mtx, previous_position) -
                      readResource<float>(pos_mtx, current_position);
    setResource<float>(pos_mtx, moving_stall_window,
                       moving_stall_window + fabs(pos_delta));
    if (readResource<float>(pos_mtx, moving_stall_window) > stall_window_size)
    {
        setResource<float>(pos_mtx, moving_stall_window, 0.0);
        setResource<int>(stall_mtx, stallCount, 0);
    }
    //
    // check for stall
    //
    if (readResource<bool>(exec_mtx, executing_trajectory))
    {
        if (fabs(pos_delta) < stall_threshold &&
            fabs(readResource<float>(vel_mtx, cmd_vel)) > (cmd_vel_threshold))
        {
            //
            // increment the stall count
            //
            setResource<int>(stall_mtx, stallCount, stallCount + 1);
            
            if (readResource<int>(stall_mtx, stallCount) > (int)MAX_STALL_COUNT)
                setResource<bool>(stall_mtx, stall_detected, true);
            
        }
    }
    if (readResource<bool>(exec_mtx, executing_trajectory))
    {
        float cmd_vel_temp = 0.0;
        //
        // Get the current time
        //
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        //
        // Compute the time delta since the last update
        //
        float dt = std::chrono::duration_cast<std::chrono::duration<float>>(currentTime - lastUpdateTime).count();
        lastUpdateTime = currentTime;
        
        // Sanity check on dt
        if(dt > DT_SANITY_RANGE * POS_UPDATE_PERIOD)
            dt = POS_UPDATE_PERIOD;
        
        bool executing_trajectory_tmp = executeTrajectory(cmdVelocityBuffer,
                                                          desiredPostionBuffer,
                                                          goal_distance, &cmd_vel_temp,
                                                          current_velocity, current_position,
                                                          dt,
                                                          readResource<bool>(stepping_mode_mtx,
                                                          stepping_mode));
        
        // Execute the current part of the trajectory
        setResource<bool>(exec_mtx, executing_trajectory, executing_trajectory_tmp);
        
        // Upate the velocity
        setResource<float>(vel_mtx, cmd_vel, cmd_vel_temp);
        
        // Set control velocity to zero if the trajectory is finished
        if(!readResource<bool>(exec_mtx, executing_trajectory))
        {
           setResource<float>(vel_mtx, cmd_vel, 0.0);
           setResource<bool>(exec_mtx, stopped_trajectory, true);
        }
        
    }
    else
    {
        setResource<float>(vel_mtx, cmd_vel, 0.0);
        setResource<bool>(exec_mtx, stopped_trajectory, true);
    }
    
    // Publish command velocity
    cmd_vel_msg.linear.x = readResource<float>(vel_mtx, cmd_vel);
    (*cmd_vel_pub).publish(cmd_vel_msg);
}

//
// subscriber callback
// parses message for state of hall effect switch
//
void getHallEffect(const std_msgs::Bool::ConstPtr &home_msg, ros::NodeHandle *nh_ptr)
{
    if (home_msg->data)
    {
        if (readResource<bool>(hall_mtx, finding_hall_effect))
        {
            setResource<bool>(hall_mtx, hall_triggered, true);
            resetEncoderCounts(nh_ptr);
        }
    }
}

//
// subscribe the control_preempt_shutdown topic
// terminate any motion in process and return from any
// service calls in progress
//
void getPreemptMessage(const std_msgs::Empty cancel_action)
{
    setResource<bool>(exec_mtx, executing_trajectory, false);
    setResource<bool>(hall_mtx, hall_triggered, false);
    setResource<bool>(hall_mtx, finding_hall_effect, false);
    setResource<int>(exec_mtx, trajectoryIndex, 0);
    setResource<bool>(preempt_mtx, preempt_received, true);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, MOTION_NODE_NAME);
    ros::NodeHandle nh;

    // Clear the trajectory buffers
    clearBuffers();
    
    // Initialize the last update time
    lastUpdateTime = std::chrono::steady_clock::now();
    //
    // create publisher for cmd_vel topic
    //
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",
                                                                    10);
    //
    // create a pointer to an actionlib move server to pass in as place holder
    //
    MoveServer *moveServer;
    //
    // instantiate a service to be called to move to new goal position
    //
    ros::ServiceServer move_to_goal_position =
        nh.advertiseService<ros_robo_bob::MoveToGoalPosition::Request,
                            ros_robo_bob::MoveToGoalPosition::Response>("move_to_goal_position",
                                                                        boost::bind(moveToGoalPosition,
                                                                                    _1, _2, moveServer));
    //
    // create a pointer to an actionlib seek server to pass in as place holder
    //
    SeekServer *seekServer;
    //
    // instantiate a service to be called to seek a new goal position
    //
    ros::ServiceServer seek_a_goal_position =
        nh.advertiseService<ros_robo_bob::SeekAGoalPosition::Request,
                            ros_robo_bob::SeekAGoalPosition::Response>("seek_a_goal_position",
                                                                       boost::bind(seekAGoalPosition,
                                                                                   _1, _2, seekServer));
    //
    // create a pointer to an actionlib find charger server to pass in as
    // place holder
    //
    FindChargerServer *findChargerServer;
    //
    // instantiate a service to be called to find the charger
    //
    ros::ServiceServer find_the_charger =
        nh.advertiseService<ros_robo_bob::FindTheCharger::Request,
                            ros_robo_bob::FindTheCharger::Response>("find_the_charger",
                                                                    boost::bind(findTheCharger,
                                                                                _1, _2, findChargerServer, &nh, &cmd_vel_pub));
    //
    // instantiate a service to be called to find home
    //
    ros::ServiceServer find_home =
        nh.advertiseService<ros_robo_bob::FindHome::Request,
                            ros_robo_bob::FindHome::Response>("find_home",
                                                              boost::bind(findHome, _1, _2, &nh));

    //
    // instantiate a service to be called to find home
    //
    ros::ServiceServer find_end =
        nh.advertiseService<ros_robo_bob::FindEnd::Request,
                            ros_robo_bob::FindEnd::Response>("find_end",
                                                              boost::bind(findEnd, _1, _2, &nh));

    //
    // subscribe to the odom topic
    //
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10,
                                                                boost::bind(getOdometryCheckStall,
                                                                            _1, &cmd_vel_pub));
    //
    // subscribe to the home topic
    //
    ros::Subscriber home_sub = nh.subscribe<std_msgs::Bool>("home", 1,
                                                            boost::bind(getHallEffect, _1, &nh));
    //
    // subscribe to the preempt motion topic
    //
    ros::Subscriber preempt_sub = nh.subscribe(addBase2Topic(CONTROL_NODE_NAME, "cancel_action"), 1,
                                               getPreemptMessage);
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::waitForShutdown();
}
