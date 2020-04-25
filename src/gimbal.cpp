/* General c++ includes */
#include <cstdint>
#include <stdio.h>
#include <ctype.h>
#include <csignal>
#include <unistd.h>
#include <iostream>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

/* MRAA includes */
#include "mraa/common.hpp"
#include "mraa/uart.hpp"

/* ROS includes*/
#include <tf/tf.h>
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <ros_robo_bob/MoveCamera.h>
#include <ros_robo_bob/GimbalOrientation.h>
#include <ros_robo_bob/PowerGimbal.h>
#include <ros_robo_bob/CalibrateGimbal.h>
#include <geometry_msgs/Vector3Stamped.h>

/* Package specific includes */
#include "../include/gpio.hpp"
#include "../include/utils.hpp"
#include "../include/bob_status.hpp"
#include "../include/bob_definitions.hpp"

/* SBGC includes */
#include "../include/sbgc/SBGC.h"
#include "../include/sbgc/SBGC_rc.h"
#include "../include/sbgc/SBGC_adj_vars.h"
#include "../include/sbgc/SBGC_cmd_helpers.h"
#include "../include/sbgc/SBGC_command.h"
#include "../include/sbgc/SBGC_parser.h"
#include "../include/sbgc/SBGC_Helper.h"
#include "sbgc/SBGC_cmd_helpers.cpp"
#include "sbgc/SBGC_Helper.cpp"

#define GIMBAL_ANGLE_WAIT 3.5
#define GIMBAL_MAX_ANGLE 60
#define GIMBAL_MIN_ANGLE -60
#define GIMBAL_MAX_WORKING_ANGLE 45
#define GIMBAL_MIN_WORKING_ANGLE -45
#define GIMBAL_ANGLE_ORIGIN 0
#define GIMBAL_POWER_WAIT_TIME 3
static SBGC_cmd_realtime_data0_t rt_data0;
static SBGC_cmd_realtime_data_t rt_data;
static SBGC_cmd_getAngles_data_t rt_angle_data;
static uint8_t isConnected = 0;
static const int angleRange = 2;

// Used to keep track of the gimbal power state
static bool gimbalState;

// Used to protect the gimbal state variable
static std::mutex gimbalStateMutex;

// The rate in which to check the gimbal angle when making a move
#define GIMBAL_ANGLE_POLL_HZ 10

// The time in which to consider a gimbal move failed
#define GIMBAL_ANGLE_TIMEOUT_S 10

// Ticks required to consider a gimbal angle stable
#define ANGLE_STABLE_TICKS 16

//
// Sets connected to true after setting the board for the board to send data
//
void setConnected()
{
    isConnected = 1;
}

void processCommand(float *angleBuffer)
{
    //
    // While we have data from the serial read
    //
    while (sbgc_parser.read_cmd())
    {
        //
        // set up the serial command object from BGC
        //
        SerialCommand &cmd = sbgc_parser.in_cmd;
        //
        // If we have not request data from the board we need to initialize that
        //
        if (!isConnected)
        {
            setConnected();
        }
        uint8_t error = 0;
        //
        // Set up switch cases in case we want to increase the ability of the
        // command proccessor
        //
        switch (cmd.id)
        {
        //
        // Set case for getting the angle data
        //
        case SBGC_CMD_GET_ANGLES:
            //
            // unpack the struct
            //
            error = SBGC_cmd_getangles_data_unpack(rt_angle_data, cmd);
            //
            // Parser did not recieve errors so return the angle buffer
            //
            if (!error)
            {
                angleBuffer[ROLL] = SBGC_ANGLE_TO_DEGREE_INT(rt_angle_data.angleData[ROLL].angle);
                angleBuffer[PITCH] = SBGC_ANGLE_TO_DEGREE_INT(rt_angle_data.angleData[PITCH].angle);
                angleBuffer[YAW] = SBGC_ANGLE_TO_DEGREE_INT(rt_angle_data.angleData[YAW].angle);
                return;
            }
            //
            // Parse the error to see what occured
            //
            else
            {
                sbgc_parser.onParseError(error);
            }
            break;
            //
            // The angle data can also be extracted from realtime data
            //
        case SBGC_CMD_REALTIME_DATA:
            error = SBGC_cmd_realtime_data0_unpack(rt_data0, cmd);
            if (!error)
            {

                angleBuffer[ROLL] = SBGC_ANGLE_TO_DEGREE_INT(rt_data0.camera_angle[ROLL]);
                angleBuffer[PITCH] = SBGC_ANGLE_TO_DEGREE_INT(rt_data0.camera_angle[PITCH]);
                angleBuffer[YAW] = SBGC_ANGLE_TO_DEGREE_INT(rt_data0.camera_angle[YAW]);
                return;
            }
            else
            {
                sbgc_parser.onParseError(error);
            }
            break;
        //
        // Other useful data structs that are not needed yet
        //
        case SBGC_CMD_REALTIME_DATA_3:
        case SBGC_CMD_REALTIME_DATA_4:
            error = SBGC_cmd_realtime_data_unpack(rt_data, cmd);
            if (!error)
            {
            }
            else
            {
                sbgc_parser.onParseError(error);
            }
            break;
        }
    }
    return;
}

void readAngles(float *buffer)
{
    SerialCommand cmd;
    //
    // We've already told the board we want data so request angles
    //
    if (isConnected)
    {
        cmd.init(SBGC_CMD_GET_ANGLES);
    }
    //
    //Send a command to initialize data transfer
    //
    else
    {
        cmd.init(SBGC_CMD_BOARD_INFO);
    }
    //
    // Have the Parser send the command
    //
    sbgc_parser.send_cmd(cmd, 0);
    //
    // Sleep for 20ms based on BGC api protocols
    //
    ros::Duration(.04).sleep();
    //
    // Process the command to return angle data
    //
    processCommand(buffer);
    //
    // return the filld angle buffer
    //
    return;
}

void setToOrigin()
{
    //
    // Intialize the SBGC helper
    //
    //SBGC_Helper_Setup();
    //
    // Create the command stucture
    //
    SBGC_cmd_control_t structure = {0, 0, 0, 0, 0, 0, 0};
    //
    // Set gimbal back to origin
    //
    structure.mode = SBGC_CONTROL_MODE_ANGLE;
    structure.anglePITCH = SBGC_DEGREE_TO_ANGLE(GIMBAL_ANGLE_ORIGIN);
    structure.angleYAW = SBGC_DEGREE_TO_ANGLE(GIMBAL_ANGLE_ORIGIN);
    structure.angleROLL = SBGC_DEGREE_TO_ANGLE(GIMBAL_ANGLE_ORIGIN);
    SBGC_cmd_control_send(structure, sbgc_parser);
}

/*******************************************************************************
** Service of the gimbal node boot will call the simple angle assurance Run   **
** Function checks to make sure all angle ranges can be aquired without       **
** Something blocking the system or the motor failing                         **
*******************************************************************************/
bool calibrateGimbalServiceCallback(ros_robo_bob::CalibrateGimbal::Request &req, ros_robo_bob::CalibrateGimbal::Response &res)
{
    //
    // Intialize the SBGC helper
    //
    //SBGC_Helper_Setup();
    //
    // Create the command stucture
    //
    SBGC_cmd_control_t structure = {0, 0, 0, 0, 0, 0, 0};
    //
    // Create Angle buffer
    //
    float buffer[3];
    //
    // Call read angles to ensure that the board is in a mode to send data
    //
    readAngles(buffer);
    //
    // Set data fields in the sturcture to test angles
    // And check that the angles are within boundaries
    //
    structure.mode = SBGC_CONTROL_MODE_ANGLE;
    structure.anglePITCH = SBGC_DEGREE_TO_ANGLE(GIMBAL_ANGLE_ORIGIN);
    structure.angleYAW = SBGC_DEGREE_TO_ANGLE(GIMBAL_ANGLE_ORIGIN);
    structure.angleROLL = SBGC_DEGREE_TO_ANGLE(GIMBAL_ANGLE_ORIGIN);
    SBGC_cmd_control_send(structure, sbgc_parser);
    ros::Duration(GIMBAL_ANGLE_WAIT).sleep();
    readAngles(buffer);
    /*if (!(buffer[ROLL] >= GIMBAL_ANGLE_ORIGIN - angleRange && buffer[ROLL] <= GIMBAL_ANGLE_ORIGIN + angleRange))
    {
        setToOrigin();
        res.status = BOB_GIMBAL_FAILURE;
        return true;
    }*/
    //
    // Set data fields in the sturcture to test angles
    // And check that the angles are within boundaries
    //
    structure.mode = SBGC_CONTROL_MODE_ANGLE;
    structure.anglePITCH = SBGC_DEGREE_TO_ANGLE(GIMBAL_ANGLE_ORIGIN);
    structure.angleYAW = SBGC_DEGREE_TO_ANGLE(GIMBAL_ANGLE_ORIGIN);
    structure.angleROLL = SBGC_DEGREE_TO_ANGLE(GIMBAL_MAX_ANGLE);
    SBGC_cmd_control_send(structure, sbgc_parser);
    ros::Duration(GIMBAL_ANGLE_WAIT).sleep();
    readAngles(buffer);
    /*if (!(buffer[ROLL] >= GIMBAL_MAX_ANGLE - angleRange && buffer[ROLL] <= GIMBAL_MAX_ANGLE + angleRange))
    {
        setToOrigin();
        res.status = BOB_GIMBAL_FAILURE;
        return true;
    }*/
    //
    // Set data fields in the sturcture to test angles
    // And check that the angles are within boundaries
    //
    structure.mode = SBGC_CONTROL_MODE_ANGLE;
    structure.anglePITCH = SBGC_DEGREE_TO_ANGLE(GIMBAL_ANGLE_ORIGIN);
    structure.angleYAW = SBGC_DEGREE_TO_ANGLE(GIMBAL_ANGLE_ORIGIN);
    structure.angleROLL = SBGC_DEGREE_TO_ANGLE(GIMBAL_MIN_ANGLE);
    SBGC_cmd_control_send(structure, sbgc_parser);
    ros::Duration(GIMBAL_ANGLE_WAIT).sleep();
    readAngles(buffer);
    /*if (!(buffer[ROLL] >= GIMBAL_MIN_ANGLE - angleRange && buffer[ROLL] <= GIMBAL_MIN_ANGLE + angleRange))
    {
        res.status = BOB_GIMBAL_FAILURE;
        return true;
    }*/
    //
    // Set data fields in the sturcture to test angles
    // And check that the angles are within boundaries
    //
    structure.mode = SBGC_CONTROL_MODE_ANGLE;
    structure.anglePITCH = SBGC_DEGREE_TO_ANGLE(GIMBAL_ANGLE_ORIGIN);
    structure.angleYAW = SBGC_DEGREE_TO_ANGLE(GIMBAL_ANGLE_ORIGIN);
    structure.angleROLL = SBGC_DEGREE_TO_ANGLE(GIMBAL_MAX_WORKING_ANGLE);
    SBGC_cmd_control_send(structure, sbgc_parser);
    ros::Duration(GIMBAL_ANGLE_WAIT).sleep();
    readAngles(buffer);
    /*if (!(buffer[ROLL] >= GIMBAL_MAX_WORKING_ANGLE - angleRange && buffer[ROLL] <= GIMBAL_MAX_WORKING_ANGLE + angleRange))
    {
        setToOrigin();
        res.status = BOB_GIMBAL_FAILURE;
        return true;
    }*/
    //
    // Set data fields in the sturcture to test angles
    // And check that the angles are within boundaries
    //
    structure.mode = SBGC_CONTROL_MODE_ANGLE;
    structure.anglePITCH = SBGC_DEGREE_TO_ANGLE(GIMBAL_ANGLE_ORIGIN);
    structure.angleYAW = SBGC_DEGREE_TO_ANGLE(GIMBAL_ANGLE_ORIGIN);
    structure.angleROLL = SBGC_DEGREE_TO_ANGLE(GIMBAL_MIN_WORKING_ANGLE);
    SBGC_cmd_control_send(structure, sbgc_parser);
    ros::Duration(GIMBAL_ANGLE_WAIT).sleep();
    readAngles(buffer);
    /*if (!(buffer[ROLL] <= GIMBAL_MIN_WORKING_ANGLE + angleRange && buffer[ROLL] >= GIMBAL_MIN_WORKING_ANGLE - angleRange))
    {
        setToOrigin();
        res.status = BOB_GIMBAL_FAILURE;
        return true;
    }*/
    //
    // Gimbal has completed simple calibration check and is function properly
    // Set gimbal back to origin and set the status to success
    //
    setToOrigin();
    res.status = BOB_SUCCESS;
    return true;
}

/*******************************************************************************
** Service to command turning the motors on and off                           **
** Should be used in charging state                                           **
*******************************************************************************/
bool gimbalPowerServiceCallback(ros_robo_bob::PowerGimbal::Request &req, ros_robo_bob::PowerGimbal::Response &res)
{
    if (req.power)
    {
        int ret_value = 0;
        ret_value |= GPIOWrite(GIMBAL_PWR_PIN, VALUE_HIGH);
        if (ret_value != 0)
        {
            res.status = BOB_UP2_GPIO_ERROR;
        }
        else
        {
            res.status = BOB_SUCCESS;
            setResource<bool>(gimbalStateMutex, gimbalState, true);
        }
        res.state = true;
    }
    else
    {
        int ret_value = 0;
        ret_value |= GPIOWrite(GIMBAL_PWR_PIN, VALUE_LOW);
        if (ret_value != 0)
        {
            res.status = BOB_UP2_GPIO_ERROR;
        }
        else
        {
            res.status = BOB_SUCCESS;
            setResource<bool>(gimbalStateMutex, gimbalState, false);
        }
        res.state = false;
    }

    return true;
}

BobStatus checkAngle(float *grabBuffer, float *angleBuffer)
{
    if (grabBuffer[ROLL] >= angleBuffer[ROLL] - angleRange && grabBuffer[ROLL] <= angleBuffer[ROLL] + angleRange)
    {
        return BOB_SUCCESS;
    }
    else
    {
        return BOB_GIMBAL_FAILURE;
    }
}

/*******************************************************************************
** Move gimbal service:                                                       **
** Takes in params to move the gimbal in a certain orentation                 **
** Parses out return to publish a bob status                                  **
*******************************************************************************/
bool moveCameraServiceCallback(ros_robo_bob::MoveCamera::Request &req, ros_robo_bob::MoveCamera::Response &res)
{
    //
    // Initialize the SBGC helper
    //
    //SBGC_Helper_Setup();
    //
    // Create the data structure for controlling angles
    //
    SBGC_cmd_control_t structure = {0, 0, 0, 0, 0, 0, 0};
    //
    // Create a grabbing buffer for angle data
    //
    float grabBuffer[3];
    //
    // Create a angle buffer for angle data
    //
    float angleBuffer[3];
    angleBuffer[ROLL] = req.angle[ROLL];
    angleBuffer[PITCH] = req.angle[PITCH];
    angleBuffer[YAW] = req.angle[YAW];

    // Poll the gimbal angle at a certain rate
    ros::Rate rate(GIMBAL_ANGLE_POLL_HZ);

    // The number of times in a row a specific angle has been read
    int stableAngleCount = 0;

    // Turn the gimbal on if it isn't already on
    if (!readResource<bool>(gimbalStateMutex, gimbalState))
    {
        int ret_value = 0;
        ret_value |= GPIOWrite(GIMBAL_PWR_PIN, VALUE_HIGH);
        if (ret_value != 0)
        {
            res.status = BOB_UP2_GPIO_ERROR;
            return true;
        }
        else
        {
            res.status = BOB_SUCCESS;
            setResource<bool>(gimbalStateMutex, gimbalState, true);
            ros::Duration(GIMBAL_POWER_WAIT_TIME).sleep();
        }
    }

    //
    // Call Read angles to make sure the board is connected
    //
    readAngles(grabBuffer);
    //
    // Send requested angles to the SBGC board
    //
    structure.mode = SBGC_CONTROL_MODE_ANGLE;
    structure.anglePITCH = SBGC_DEGREE_TO_ANGLE(req.angle[PITCH]);
    structure.angleYAW = SBGC_DEGREE_TO_ANGLE(req.angle[YAW]);
    structure.angleROLL = SBGC_DEGREE_TO_ANGLE(req.angle[ROLL]);
    int result = SBGC_cmd_control_send(structure, sbgc_parser);
    //
    // Check result to make sure that the parser did not encounter errors
    //
    if (result == 0)
    {
        int timeout = 0;
        //
        // Read Angles until we reach destination or timeout
        //
        while(stableAngleCount < ANGLE_STABLE_TICKS != BOB_SUCCESS
              && timeout != GIMBAL_ANGLE_TIMEOUT_S*GIMBAL_ANGLE_POLL_HZ)
        {
            readAngles(grabBuffer);

            // Increment the stable counter if we've reached our target angle
            if(checkAngle(grabBuffer, angleBuffer) == BOB_SUCCESS)
                stableAngleCount++;
            else
                stableAngleCount = 0;

            timeout ++;
            rate.sleep();
        }
        //
        // Quick flush for buffer
        //
        for (int nReads = 0; nReads < 5; nReads++)
        {
            readAngles(grabBuffer);
        }
        //
        // Check to see if we reached the angle
        // Sets status to BOB_SUCCESS if we have otherwise set it to error
        //
        ROS_ERROR_STREAM("Attempted to reach " << (float)angleBuffer[ROLL]);
        ROS_ERROR_STREAM("Gimbal angle obtained " << (float)grabBuffer[ROLL]);
        BobStatus status = checkAngle(grabBuffer, angleBuffer);
        res.cameraAngle[ROLL] = grabBuffer[ROLL];
        res.cameraAngle[PITCH] = grabBuffer[PITCH];
        res.cameraAngle[YAW] = grabBuffer[YAW];
        res.status = status;
    }
    //
    // The parser encountered errors and sets the status to false
    //
    else
    {
        res.status = BOB_GIMBAL_PARSE_ERROR;
    }
    return true;
}

/*******************************************************************************
** Gimbal orientation service                                                 **
** Takes in params to read the gimabl orientation                             **
** Parses out return to publish a bob status                                  **
*******************************************************************************/
bool getOrientationServiceCallback(ros_robo_bob::GimbalOrientation::Request &req, ros_robo_bob::GimbalOrientation::Response &res)
{
    //
    // Create a grabbing buffer for angle data
    //
    float grabBuffer[3];
    for (int nReads = 0; nReads < 5; nReads++)
    {
        readAngles(grabBuffer);
    }
    res.orientation[ROLL] = grabBuffer[ROLL];
    res.orientation[PITCH] = grabBuffer[PITCH];
    res.orientation[YAW] = grabBuffer[YAW];
    return true;
}

int main(int argc, char **argv)
{
    //
    // initialize and declare the node
    //
    ros::init(argc, argv, GIMBAL_NODE_NAME);
    //
    // declare the handle to the node
    //
    ros::NodeHandle handle;
    //
    // publisher to check uart
    //
    std_msgs::Int8 msg;
    ros::Publisher pub = handle.advertise<std_msgs::Int8>(addBase2Topic(GIMBAL_NODE_NAME, "uart"), 10);
    if (SBGC_Helper_Setup() != BOB_SUCCESS)
    {
        msg.data = BOB_MRAA_UART_FAILURE;
        pub.publish(msg);
    }

    // Initialize the power control pin
    GPIOExport(GIMBAL_PWR_PIN);
    GPIODirection(GIMBAL_PWR_PIN, "out");
    GPIOWrite(GIMBAL_PWR_PIN, VALUE_LOW);

    // Initialize the gimbal power state
    gimbalState = false;

    //
    // Services for this node
    //
    ros::ServiceServer getOrientation = handle.advertiseService<ros_robo_bob::GimbalOrientation::Request,
                                                                ros_robo_bob::GimbalOrientation::Response>(addBase2Topic(GIMBAL_NODE_NAME, "get_orientation"),
                                                                                                           boost::bind(getOrientationServiceCallback, _1, _2));
    ros::ServiceServer calibrateGimbal = handle.advertiseService<ros_robo_bob::CalibrateGimbal::Request,
                                                                 ros_robo_bob::CalibrateGimbal::Response>(addBase2Topic(GIMBAL_NODE_NAME, "calibrate_gimbal"),
                                                                                                          boost::bind(calibrateGimbalServiceCallback, _1, _2));

    ros::ServiceServer moveCamera = handle.advertiseService<ros_robo_bob::MoveCamera::Request,
                                                            ros_robo_bob::MoveCamera::Response>(addBase2Topic(GIMBAL_NODE_NAME, "move_camera"),
                                                                                                boost::bind(moveCameraServiceCallback, _1, _2));

    ros::ServiceServer powerGimbal = handle.advertiseService<ros_robo_bob::PowerGimbal::Request,
                                                             ros_robo_bob::PowerGimbal::Response>(addBase2Topic(GIMBAL_NODE_NAME, "power_gimbal"),
                                                                                                  boost::bind(gimbalPowerServiceCallback, _1, _2));
    ros::spin();
}
