/* General c++ includes */
#include <omp.h>
#include <mutex>
#include <time.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <algorithm>

/* Service includes */
#include <ros_robo_bob/GetPosition.h>

/* Package specific includes */
#include "../include/utils.hpp"
#include "../include/action_client.hpp"

/* Constant defintions */
#define ODOM_NODE_NAME "odom"

//@TODO: Make more efficient by consolidating shared client code

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #  
#                                                                                   #
#   Description: Constructor for the ActionClient class. Initializes a ROS client   #
#                node and then initializes all necessary services.                  #
#                                                                                   #
************************************************************************************/

ActionClient::ActionClient()
{
    // Initialize each client
    seekClient = new SeekClient(addBase2Topic(SERVER_NODE_NAME, "seek"), true);
    moveClient = new MoveClient(addBase2Topic(SERVER_NODE_NAME, "move"), true);
    findEndClient = new FindEndClient(addBase2Topic(SERVER_NODE_NAME, "find_end"), true);
    findHomeClient = new FindHomeClient(addBase2Topic(SERVER_NODE_NAME, "find_home"), true);
    moveCameraClient = new MoveCameraClient(addBase2Topic(SERVER_NODE_NAME, "move_camera"), true);
    findChargerClient = new FindChargerClient(addBase2Topic(SERVER_NODE_NAME, "find_charger"), true);
    captureImageClient = new CaptureImageClient(addBase2Topic(SERVER_NODE_NAME, "capture_image"), true);

    // Initialize status variables
    seekPending = false;
    movePending = false;
    findEndPending = false;
    findHomePending = false;
    moveCameraPending = false;
    findChargerPending = false;
    captureImagePending = false;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          # 
#                                                                                   #
#   Description: Frees any memory associated with the ActionClient, mainly the      #
#                dynamically allocated services.                                    #
#                                                                                   #
************************************************************************************/

ActionClient::~ActionClient()
{
    delete (seekClient);
    delete (moveClient);
    delete (findEndClient);
    delete (findHomeClient);
    delete (findChargerClient);
    delete (moveCameraClient);
    delete (captureImageClient);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @goal                 The goal distance of the move                     #
#           @result               A ConstPtr to the results                         #
#   Outputs:                                                                        #
#           @returns              Status code indicating the outcome of the move    #
#                                                                                   #
#   Description: Sends a move request to the server, gets the results, and handles  #
#                timeouts.                                                          #
#                                                                                   #
************************************************************************************/

BobStatus ActionClient::moveRB(ros_robo_bob::MoveGoal &goal,
                               ros_robo_bob::MoveResult &result)
{
    // Compute the timeout
    float timeout = (std::abs(goal.distance) / RB_MOVE_MIN_VELOCITY_INCH_S) * RB_MOVESEEK_TIMEOUT_FACTOR;
    timeout = std::max(timeout, (float)RB_MOVESEEK_MIN_TIMEOUT);

    // Wait for the server
    if (!moveClient->waitForServer(ros::Duration(RB_ACTION_SERVER_TIMEOUT)))
        return BOB_AR_SERVER_TIMEOUT;

    setClientStatus(movePendingMutex, movePending, true);

    // Send the goal to the client
    moveClient->sendGoal(goal);

    // Wait for the server to be finished
    if (!moveClient->waitForResult(ros::Duration(timeout)))
        return BOB_MOVE_TIMEOUT;

    // Get the result from the server
    result = *moveClient->getResult();

    setClientStatus(movePendingMutex, movePending, false);

    return result.status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @goal                 The goal distance of seek                         #
#           @result               A ConstPtr to the results                         #
#   Outputs:                                                                        #
#           @returns              Status code indicating the outcome of the seek    #
#                                                                                   #
#   Description: Sends a seek request to the server, gets the results, and handles  #
#                timeouts.                                                          #
#                                                                                   #
************************************************************************************/

BobStatus ActionClient::seekRB(ros_robo_bob::SeekGoal &goal,
                               ros_robo_bob::SeekResult &result)
{
    ros::NodeHandle handle;
    ros_robo_bob::GetPosition server;

    // Create the position service client
    ros::ServiceClient positionClient = handle.serviceClient<ros_robo_bob::GetPosition>
                                        (addBase2Topic(ODOM_NODE_NAME, "get_position"));
    
    // Get the robots position
    positionClient.call(server);
    
    // Compute the timeout
    float timeout = (std::abs((float)goal.distance-(float)server.response.position) / RB_SEEK_MIN_VELOCITY_INCH_S) * RB_MOVESEEK_TIMEOUT_FACTOR;
    timeout = std::max(timeout, (float)RB_MOVESEEK_MIN_TIMEOUT);
    
    // Wait for the server
    if (!seekClient->waitForServer(ros::Duration(RB_ACTION_SERVER_TIMEOUT)))
        return BOB_AR_SERVER_TIMEOUT;

    setClientStatus(seekPendingMutex, seekPending, true);

    // Send the goal to the client
    seekClient->sendGoal(goal);

    // Wait for the server to be finished
    if (!seekClient->waitForResult(ros::Duration(timeout)))
        return BOB_SEEK_TIMEOUT;

    // Get the result from the server
    result = *seekClient->getResult();

    setClientStatus(seekPendingMutex, seekPending, false);

    return result.status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #       
#           @goal                 The goal distance of find_home                    #
#           @result               A ConstPtr to the results                         #
#   Outputs:                                                                        #
#           @returns              Status code indicating the outcome of the         #
#                                 find_home                                         #
#                                                                                   #
#   Description: Sends a find_home request to the server, gets the results, and     #
#                handles timeouts.                                                  #
#                                                                                   #
************************************************************************************/

BobStatus ActionClient::findHomeRB(ros_robo_bob::FindHomeGoal &goal,
                               ros_robo_bob::FindHomeResult &result)
{
    // Wait for the server
    if (!findHomeClient->waitForServer(ros::Duration(RB_ACTION_SERVER_TIMEOUT)))
        return BOB_AR_SERVER_TIMEOUT;

    setClientStatus(findHomePendingMutex, findHomePending, true);

    // Send the goal to the client
    findHomeClient->sendGoal(goal);

    // Wait for the server to be finished
    if (!findHomeClient->waitForResult(ros::Duration(RB_FIND_HOME_TIMEOUT)))
        return BOB_FIND_HOME_TIMEOUT;

    // Get the result from the server
    result = *findHomeClient->getResult();

    setClientStatus(findHomePendingMutex, findHomePending, false);

    return result.status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #       
#           @goal                 The goal distance of find_home                    #
#           @result               A ConstPtr to the results                         #
#   Outputs:                                                                        #
#           @returns              Status code indicating the outcome of the         #
#                                 find_end                                          #
#                                                                                   #
#   Description: Sends a find_end request to the server, gets the results, and      #
#                handles timeouts.                                                  #
#                                                                                   #
************************************************************************************/

BobStatus ActionClient::findEndRB(ros_robo_bob::FindEndGoal &goal,
                               ros_robo_bob::FindEndResult &result)
{
    // Wait for the server
    if (!findEndClient->waitForServer(ros::Duration(RB_ACTION_SERVER_TIMEOUT)))
        return BOB_AR_SERVER_TIMEOUT;

    setClientStatus(findEndPendingMutex, findEndPending, true);

    // Send the goal to the client
    findEndClient->sendGoal(goal);

    // Wait for the server to be finished
    if (!findEndClient->waitForResult(ros::Duration(RB_FIND_END_TIMEOUT)))
        return BOB_FIND_END_TIMEOUT;

    // Get the result from the server
    result = *findEndClient->getResult();

    setClientStatus(findEndPendingMutex, findEndPending, false);

    return result.status;
}


/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @goal                 The goal orientation of the camera                #
#           @result               A ConstPtr to the results                         #
#   Outputs:                                                                        #
#           @returns              Status code indicating the outcome of the move    #
#                                                                                   #
#   Description: Sends a camera move request to the server, gets the results,       #
#                and handles timeouts.                                              #
#                                                                                   #
************************************************************************************/

BobStatus ActionClient::moveCameraRB(ros_robo_bob::MoveCameraGoal &goal,
                                     ros_robo_bob::MoveCameraResult &result)
{
    // Wait for the server
    if (!moveCameraClient->waitForServer(ros::Duration(RB_ACTION_SERVER_TIMEOUT)))
        return BOB_AR_SERVER_TIMEOUT;

    setClientStatus(moveCameraPendingMutex, moveCameraPending, true);

    // Send the goal to the client
    moveCameraClient->sendGoal(goal);

    // Wait for the server to be finished
    if (!moveCameraClient->waitForResult(ros::Duration(RB_CAMERA_MOVE_TIMEOUT)))
        return BOB_MOVE_CAMERA_TIMEOUT;

    // Get the result from the server
    result = *moveCameraClient->getResult();

    setClientStatus(moveCameraPendingMutex, moveCameraPending, false);

    return result.status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @result               A ConstPtr to the results                         #
#   Outputs:                                                                        #
#           @returns              Status code indicating the outcome of the         #
#                                 capture                                           #
#                                                                                   #
#   Description: Sends a camera capture request to the server, gets the results,    #
#                and handles timeouts.                                              #
#                                                                                   #
************************************************************************************/

BobStatus ActionClient::captureImageRB(ros_robo_bob::CaptureImageGoal &goal,
                                       ros_robo_bob::CaptureImageResult &result)
{

    // Wait for the server
    if (!captureImageClient->waitForServer(ros::Duration(RB_ACTION_SERVER_TIMEOUT)))
        return BOB_AR_SERVER_TIMEOUT;

    setClientStatus(captureImagePendingMutex, captureImagePending, true);

    // Send the goal to the client
    captureImageClient->sendGoal(goal);

    // Wait for the server to be finished
    if (!captureImageClient->waitForResult(ros::Duration(RB_CAPTURE_IMAGE_TIMEOUT)))
        return BOB_CAPTURE_IMAGE_TIMEOUT;

    // Get the result from the server
    result = *captureImageClient->getResult();

    setClientStatus(captureImagePendingMutex, captureImagePending, false);

    return result.status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @goal                 The goal distance of the find_charger             #
#           @result               A ConstPtr to the results                         #
#   Outputs:                                                                        #
#           @returns              Status code indicating the outcome of the move    #
#                                                                                   #
#   Description: Sends a find_charger move request to the server, gets the          # 
#                results, and handles timeouts.                                     #
#                                                                                   #
************************************************************************************/

BobStatus ActionClient::findChargerRB(ros_robo_bob::FindChargerGoal &goal,
                                      ros_robo_bob::FindChargerResult &result)
{
    // Wait for the server
    if (!findChargerClient->waitForServer(ros::Duration(RB_ACTION_SERVER_TIMEOUT)))
        return BOB_AR_SERVER_TIMEOUT;

    setClientStatus(findChargerPendingMutex, findChargerPending, false);

    // Send the goal to the client
    findChargerClient->sendGoal(goal);

    // Wait for the server to be finished
    if (!findChargerClient->waitForResult(ros::Duration(RB_FIND_CHARGER_TIMEOUT)))
        return BOB_FIND_CHARGER_TIMEOUT;

    // Get the result from the server
    result = *findChargerClient->getResult();

    setClientStatus(findChargerPendingMutex, findChargerPending, false);

    return result.status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          # 
#   Description: Cancels the current move call.                                     #
#                                                                                   #
************************************************************************************/

void ActionClient::cancelMoveRB()
{
    if(checkClientStatus(movePendingMutex, movePending))
    {
        moveClient->cancelGoal();
        movePending = false;
    }
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          # 
#   Description: Cancels the current seek call.                                     #
#                                                                                   #
************************************************************************************/

void ActionClient::cancelSeekRB()
{
    if(checkClientStatus(seekPendingMutex, seekPending))
    {
        seekClient->cancelGoal();
        seekPending = false;
    }
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          # 
#   Description: Cancels the current move_camera call.                              #
#                                                                                   #
************************************************************************************/

void ActionClient::cancelMoveCameraRB()
{
    if(checkClientStatus(moveCameraPendingMutex, moveCameraPending))
    {
        moveCameraClient->cancelGoal();
        moveCameraPending = false;
    }
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          # 
#   Description: Cancels the current find_home call.                                #
#                                                                                   #
************************************************************************************/

void ActionClient::cancelFindHomeRB()
{
    if(checkClientStatus(findHomePendingMutex, findHomePending))
    {
        findHomeClient->cancelGoal();
        findHomePending = false;
    }
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          # 
#   Description: Cancels the current find_end call.                                 #
#                                                                                   #
************************************************************************************/

void ActionClient::cancelFindEndRB()
{
    if(checkClientStatus(findEndPendingMutex, findEndPending))
    {
        findEndClient->cancelGoal();
        findEndPending = false;
    }
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          # 
#   Description: Cancels the current capture_image camera call.                     #
#                                                                                   #
************************************************************************************/

void ActionClient::cancelCaptureImageRB()
{
    if(checkClientStatus(captureImagePendingMutex, captureImagePending))
    {
        captureImageClient->cancelGoal();
        captureImagePending = false;
    }
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          # 
#   Description: Cancels the current find_charger camera call.                      #
#                                                                                   #
************************************************************************************/

void ActionClient::cancelFindChargerRB()
{
    if(checkClientStatus(findChargerPendingMutex, findChargerPending))
    {
        findChargerClient->cancelGoal();
        findChargerPending = false;
    }
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          # 
#   Description: Cancels all actions currently in progress by the client.           #
#                                                                                   #
************************************************************************************/

void ActionClient::cancelAllActions()
{
    cancelMoveRB();
    cancelSeekRB();
    cancelFindEndRB();
    cancelFindHomeRB();
    cancelMoveCameraRB();
    cancelFindChargerRB();
    cancelCaptureImageRB();
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #  
#   Description: Checks to see if there is a pending seek.                          #              
#                                                                                   #
************************************************************************************/

bool ActionClient::isSeekPending()
{
    return checkClientStatus(seekPendingMutex, seekPending);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #  
#   Description: Checks to see if there is a pending move.                          #              
#                                                                                   #
************************************************************************************/

bool ActionClient::isMovePending()
{
    return checkClientStatus(movePendingMutex, movePending);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #  
#   Description: Checks to see if there is a pending camera move.                   #          
#                                                                                   #
************************************************************************************/

bool ActionClient::isMoveCameraPending()
{
    return checkClientStatus(moveCameraPendingMutex, moveCameraPending);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #  
#   Description: Checks to see if there is a find_charger.                          #                      
#                                                                                   #
************************************************************************************/

bool ActionClient::isFindChargerPending()
{
    return checkClientStatus(findChargerPendingMutex, findChargerPending);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #  
#   Description: Checks to see if there is a capture_image.                         #            
#                                                                                   #
************************************************************************************/

bool ActionClient::isCaptureImagePending()
{
    return checkClientStatus(captureImagePendingMutex, captureImagePending);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #  
#   Description: Check whether or not a client has a pending action.                #             
#                                                                                   #
************************************************************************************/

bool ActionClient::checkClientStatus(std::mutex & mutex, bool & statusVar)
{
    bool status;

    mutex.lock();
    status = statusVar;
    mutex.unlock();

    return status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #  
#   Description: Sets the pending status of a client.                               #           
#                                                                                   #
************************************************************************************/

void ActionClient::setClientStatus(std::mutex & mutex, bool & statusVar, bool value)
{
    mutex.lock();
    statusVar = value;
    mutex.unlock();
}
