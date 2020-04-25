/* General c++ includes */
#include <csignal>

/* Package specific includes */
#include "../include/action_server.hpp"

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Shawn Quinn                                             #
#   Inputs:                                                                         #
#           @goal                 The goal from the server                          #
#           @server               Pointer to the server                             #
#   Outputs:                                                                        #
#           @publishes            Success message with status code and RFID of tag  #
#                                                                                   #
#   Description: Handles a seek request from a client, which moves the robot at     #
#                low speeds until either the distance goal is met, or an RFID       #
#                tag is found. Set the distance to INT_MAX or -INT_MAX to seek      #
#                without a distance constraint, with the sign of the distance       #
#                indicating the direction of movement. Refer to action/Seek.action  #
#                for action details.                                                #
#                                                                                   #
************************************************************************************/

void seekCB(const ros_robo_bob::SeekGoalConstPtr &goal, SeekServer *server)
{

    // A status code representing the outcome of the action
    ros_robo_bob::SeekResult result;

    // Distance to travel while seeking. Set to INT_MAX when seeking to the end.
    float position = goal->distance;

    ros::NodeHandle nh;
    ros::ServiceClient seek_client =
    nh.serviceClient<ros_robo_bob::SeekAGoalPosition>("seek_a_goal_position");
    ros_robo_bob::SeekAGoalPosition srv;
    srv.request.goal_position = position;
    //
    // blocking call to move_to_goal_position service
    //
    if (seek_client.call(srv)) {
        result.position = srv.response.position;
        result.status = srv.response.result;
    }
    else {
        result.position = 0.0;
        result.status = BOB_SAGP_SERVICE_FAILED;
    }

    server->setSucceeded(result);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Shawn Quinn                                             #
#   Inputs:                                                                         #
#           @goal                 The goal from the server                          #
#           @server               Pointer to the server                             #
#   Outputs:                                                                        #
#           @publishes            Success message with status code and current      #
#                                 position                                          #
#                                                                                   #
#   Description: Handles a move request from the client. Simply moves the robot     #
#                a desired distance, with the sign of the distance controlling the  #
#                direction of the robot. Refer to action/Move.action for action     #
#                details.                                                           #
#                                                                                   #
************************************************************************************/

void moveCB(const ros_robo_bob::MoveGoalConstPtr &goal, MoveServer *server)
{

    // A status code representing the outcome of the action
    ros_robo_bob::MoveResult result;

    // Distance to travel while moving.
    float distance = goal->distance;

    ros::NodeHandle nh;
    ros::ServiceClient move_client =
    nh.serviceClient<ros_robo_bob::MoveToGoalPosition>("move_to_goal_position");
    ros_robo_bob::MoveToGoalPosition srv;
    srv.request.goal_position = distance;
    //
    // blocking call to move_to_goal_position service
    //
    if (move_client.call(srv)) {
        result.position = srv.response.position;
        result.status = srv.response.result;
    }
    else {
        result.position = 0.0;
        result.status = BOB_MTGP_SERVICE_FAILED;
    }

    server->setSucceeded(result);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @goal                 The goal from the server                          #
#           @server               Pointer to the server                             #
#   Outputs:                                                                        #
#           @publishes            Success message with status code and current      #
#                                 position                                          #
#                                                                                   #
#   Description: Handles a find_home request from the client. Searches in the       #
#                negative direction for the magnet first, then searches the other   #
#                way if a stall has been encountered.                               #
#                                                                                   #
************************************************************************************/

void findHomeCB(const ros_robo_bob::FindHomeGoalConstPtr &goal, FindHomeServer *server)
{
    ros::NodeHandle handle;
    ros_robo_bob::FindHome srv;

    // A status code representing the outcome of the action
    ros_robo_bob::FindHomeResult result;

    // Create the service client
    ros::ServiceClient find_home_client = handle.serviceClient<ros_robo_bob::FindHome>("find_home");

    if (find_home_client.call(srv))
        result.status = srv.response.status;

    else
        result.status = BOB_FH_SERVICE_FAILED;

    server->setSucceeded(result);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @goal                 The goal from the server                          #
#           @server               Pointer to the server                             #
#   Outputs:                                                                        #
#           @publishes            Success message with status code and current      #
#                                 position                                          #
#                                                                                   #
#   Description: Handles a find_home request from the client. Searches in the       #
#                positive direction for the end of the track.		            #
#                                                                                   #
************************************************************************************/

void findEndCB(const ros_robo_bob::FindEndGoalConstPtr &goal, FindEndServer *server)
{
    ros::NodeHandle handle;
    ros_robo_bob::FindEnd srv;

    // A status code representing the outcome of the action
    ros_robo_bob::FindEndResult result;

    // Create the service client
    ros::ServiceClient find_end_client = handle.serviceClient<ros_robo_bob::FindEnd>("find_end");

    if (find_end_client.call(srv))
        result.status = srv.response.status;

    else
        result.status = BOB_FE_SERVICE_FAILED;

    server->setSucceeded(result);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @goal                 The goal from the server                          #
#           @server               Pointer to the server                             #
#   Outputs:                                                                        #
#           @publishes            Success message with status code and current      #
#                                 camera orientation                                #
#                                                                                   #
#   Description: Handles a camera move request by setting the orientation of        #
#                the onboard gimbal. Refer to action/MoveCamera.action for action   #
#                details.                                                           #
#                                                                                   #
************************************************************************************/

void moveCameraCB(const ros_robo_bob::MoveCameraGoalConstPtr &goal, MoveCameraServer *server)
{

    // A status code representing the outcome of the action
    ros_robo_bob::MoveCameraResult result;

    // Goal orientation of the camera
    // orientation follows as {x,y,z}
    boost::array<float, 3ul> orientation = goal->orientation;

    ros::NodeHandle handle;
    ros_robo_bob::MoveCamera srv;

    ros::ServiceClient move_camera_client =
    handle.serviceClient<ros_robo_bob::MoveCamera>("gimbal/move_camera");
    srv.request.angle = orientation;
    if (move_camera_client.call(srv)) {
        result.orientation = srv.response.cameraAngle;
        result.status = srv.response.status;
    }
    else {
        result.orientation = {0,0,0};
        result.status = BOB_GIMBAL_SERVICE_FAILED;
    }
    server->setSucceeded(result);
}


/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @server               Pointer to the server                             #
#   Outputs:                                                                        #
#           @publishes            Success message with status code and camera       #
#                                 image                                             #
#                                                                                   #
#   Description: Gets an image from the camera.                                     #
#                                                                                   #
************************************************************************************/

void captureImageCB(const ros_robo_bob::CaptureImageGoalConstPtr &goal, CaptureImageServer *server)
{

    sensor_msgs::ImageConstPtr image;

    // A status code representing the outcome of the action
    ros_robo_bob::CaptureImageResult result;

    //
    // call the save_image service to grab the image
    //
    ros::NodeHandle nh;
    ros::ServiceClient save_image_client =
    nh.serviceClient<ros_robo_bob::SaveImage>("lumenera_camera/save_image");
    ros_robo_bob::SaveImage srv;
    srv.request.autoWB = goal->autoWB;

    //
    // blocking call to move_to_goal_position service
    //
    if (save_image_client.call(srv)) 
    {
        result.image = srv.response.image;
        result.status = srv.response.status;
    }
    else {
        result.status = BOB_CI_SERVICE_FAILED;
    }

    server->setSucceeded(result);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @server               Pointer to the server                             #
#   Outputs:                                                                        #
#           @publishes            Success message with status code                  #
#                                                                                   #
#   Description: Handles a find_charger request from the client. Simply moves the   #
#                robot a desired distance at a low velocity while looking for the   #
#                charger. Refer to action/FindCharger.action for action details.    #
#                                                                                   #
************************************************************************************/

void findChargerCB(const ros_robo_bob::FindChargerGoalConstPtr &goal, FindChargerServer *server)
{

    // A status code representing the outcome of the action
    ros_robo_bob::FindChargerResult result;

    // Distance to travel while seeking. Set to INT_MAX when seeking to the end.
    float distance = goal->distance;

    //
    //  Control code goes here, make sure to set result variables.
    //
    //  To check for preemption, add "SeekServer * s" to the control function arguments, and check
    //  "s->isPreemptRequested()" to detect preemption
    //
    // call the find_the_charger service
    //
    ros::NodeHandle nh;
    ros::ServiceClient move_client =
    nh.serviceClient<ros_robo_bob::FindTheCharger>("find_the_charger");
    ros_robo_bob::FindTheCharger srv;
    if (move_client.call(srv)) {
        result.rfid_code = srv.response.rfid_value;
        result.status = srv.response.result;
    }
    else {
        result.rfid_code = 0;
        result.status = BOB_FC_SERVICE_FAILED;
    }

    server->setSucceeded(result);
}

int main(int argc, char **argv)
{

    // Initialize the action server node
    ros::init(argc, argv, SERVER_NODE_NAME);

    // The ros handle for this node
    ros::NodeHandle handle;
    
    // Create the server for each action
    SeekServer seekServer(handle, addBase2Topic(SERVER_NODE_NAME, "seek"),
                          boost::bind(&seekCB, _1, &seekServer), false);
    MoveServer moveServer(handle, addBase2Topic(SERVER_NODE_NAME, "move"),
                          boost::bind(&moveCB, _1, &moveServer), false);
    FindEndServer findEndServer(handle, addBase2Topic(SERVER_NODE_NAME, "find_end"),
                          boost::bind(&findEndCB, _1, &findEndServer), false);
    FindHomeServer findHomeServer(handle, addBase2Topic(SERVER_NODE_NAME, "find_home"),
                          boost::bind(&findHomeCB, _1, &findHomeServer), false);
    MoveCameraServer moveCameraServer(handle, addBase2Topic(SERVER_NODE_NAME, "move_camera"),
                                      boost::bind(&moveCameraCB, _1, &moveCameraServer), false);
    FindChargerServer findChargerServer(handle, addBase2Topic(SERVER_NODE_NAME, "find_charger"),
                                      boost::bind(&findChargerCB, _1, &findChargerServer), false);
    CaptureImageServer captureImageServer(handle, addBase2Topic(SERVER_NODE_NAME, "capture_image"),
                                      boost::bind(&captureImageCB, _1, &captureImageServer), false);

    // Start the servers
    seekServer.start();
    moveServer.start();
    findEndServer.start();
    findHomeServer.start();
    moveCameraServer.start();
    findChargerServer.start();
    captureImageServer.start();

    ros::spin();

    return 0;
}
