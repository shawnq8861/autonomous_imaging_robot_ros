#ifndef ROS_ROBO_BOB_ACTION_CLIENT_H
#define ROS_ROBO_BOB_ACTION_CLIENT_H

/* General C++ includes */
#include <mutex>

/* ROS includes */
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

/* Action includes */
#include <ros_robo_bob/SeekAction.h>
#include <ros_robo_bob/MoveAction.h>
#include <ros_robo_bob/FindEndAction.h>
#include <ros_robo_bob/FindHomeAction.h>
#include <ros_robo_bob/MoveCameraAction.h>
#include <ros_robo_bob/FindChargerAction.h>
#include <ros_robo_bob/CaptureImageAction.h>

/* Package specific includes */
#include "../include/bob_status.hpp"

/* Constant definitions */
#define CLIENT_NODE_NAME "action_client"                // Name of the client node
#define SERVER_NODE_NAME "action_server"                // Name of the server node
#define RB_MOVE_MIN_VELOCITY_INCH_S 2.                  // Min move velocity
#define RB_SEEK_MIN_VELOCITY_INCH_S 2                   // Min seek velocity
#define RB_FCHRG_MIN_VELOCITY_INCH_S 2                  // Max find_charger velocity
#define RB_MAX_TRACK_LENGTH_INCH 3600                   // Max length of any track in inches
#define RB_MOVESEEK_TIMEOUT_FACTOR 2                    // How long a timeout should be compared to
                                                        // the expected time to make a move/seek   
#define RB_MOVESEEK_MIN_TIMEOUT 10                      // The timeout in the case of a net zero move 
#define RB_MAX_TRACK_LENGTH 1000.                       // Maximum length of a track
#define RB_CAMERA_MOVE_TIMEOUT 18.                       // Timeout in seconds for a camera move 
#define RB_ACTION_SERVER_TIMEOUT 5.                     // Maximum time to wait for a server connection 
#define RB_CAPTURE_IMAGE_TIMEOUT 10.                     // Maximum time to wait for a camera capture
#define RB_FIND_HOME_TIMEOUT 600                        // Timeout for a find_home
#define RB_FIND_CHARGER_TIMEOUT 180                     // Timeout for a find_charger
#define RB_FIND_END_TIMEOUT 180                         // Timeout for a find_end move

/* Typedefs */
typedef actionlib::SimpleActionClient<ros_robo_bob::SeekAction> SeekClient;
typedef actionlib::SimpleActionClient<ros_robo_bob::MoveAction> MoveClient;
typedef actionlib::SimpleActionClient<ros_robo_bob::FindEndAction> FindEndClient;
typedef actionlib::SimpleActionClient<ros_robo_bob::FindHomeAction> FindHomeClient;
typedef actionlib::SimpleActionClient<ros_robo_bob::MoveCameraAction> MoveCameraClient;
typedef actionlib::SimpleActionClient<ros_robo_bob::FindChargerAction> FindChargerClient;
typedef actionlib::SimpleActionClient<ros_robo_bob::CaptureImageAction> CaptureImageClient;

/* Class definitions */
class ActionClient
{

  private:

    // Clients for various actions
    SeekClient * seekClient;
    MoveClient * moveClient;
    FindEndClient * findEndClient;
    FindHomeClient * findHomeClient;
    MoveCameraClient * moveCameraClient;
    FindChargerClient * findChargerClient;
    CaptureImageClient * captureImageClient;

    // Status checking mutexes
    std::mutex seekPendingMutex;
    std::mutex movePendingMutex;
    std::mutex findEndPendingMutex;
    std::mutex findHomePendingMutex;
    std::mutex moveCameraPendingMutex;
    std::mutex findChargerPendingMutex;
    std::mutex captureImagePendingMutex;

    // Status checking variables
    bool seekPending;
    bool movePending;
    bool findEndPending;
    bool findHomePending;
    bool moveCameraPending;
    bool findChargerPending;
    bool captureImagePending;
    
  public:
  
    ActionClient();
    ~ActionClient();
    void cancelMoveRB();
    void cancelSeekRB();
    bool isMovePending();
    bool isSeekPending();
    void cancelFindEndRB();
    void cancelFindHomeRB();
    void cancelAllActions();
    void cancelMoveCameraRB();
    void cancelFindChargerRB();
    bool isMoveCameraPending();
    bool isFindChargerPending();
    void cancelCaptureImageRB();
    bool isCaptureImagePending();
    bool checkClientStatus(std::mutex & mutex, bool & statusVar);
    BobStatus moveRB(ros_robo_bob::MoveGoal&, ros_robo_bob::MoveResult&);
    BobStatus seekRB(ros_robo_bob::SeekGoal&, ros_robo_bob::SeekResult&);
    void setClientStatus(std::mutex & mutex, bool & statusVar, bool value);
    BobStatus findEndRB(ros_robo_bob::FindEndGoal&, ros_robo_bob::FindEndResult&);
    BobStatus findHomeRB(ros_robo_bob::FindHomeGoal&, ros_robo_bob::FindHomeResult&);
    BobStatus moveCameraRB(ros_robo_bob::MoveCameraGoal&, ros_robo_bob::MoveCameraResult&);
    BobStatus findChargerRB(ros_robo_bob::FindChargerGoal&, ros_robo_bob::FindChargerResult&);
    BobStatus captureImageRB(ros_robo_bob::CaptureImageGoal&, ros_robo_bob::CaptureImageResult&);
    
};

#endif
