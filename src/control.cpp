/* General c++ includes */
#include <map>
#include <time.h>
#include <string>
#include <mutex>
#include <thread>
#include <fstream>
#include <unistd.h>
#include <limits.h>
#include <sys/stat.h>
#include <boost/filesystem.hpp>

/* ROS includes */
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>

/* Package specific includes */
#include "../include/utils.hpp"
#include "../include/bob_status.hpp"
#include "../include/track_plan.hpp"
#include "../include/image_proc.hpp"
#include "../include/action_client.hpp"
#include "../include/bob_definitions.hpp"

/* Service Includes */
#include <ros_robo_bob/ReadRFID.h>
#include <ros_robo_bob/GetVoltage.h>
#include <ros_robo_bob/ControlMode.h>
#include <ros_robo_bob/GetLastRFID.h>
#include <ros_robo_bob/PowerGimbal.h>
#include <ros_robo_bob/GetPosition.h>
#include <ros_robo_bob/GimbalOrientation.h>
#include <ros_robo_bob/StartStopStreaming.h>
#include <ros_robo_bob/ChargeMaintenanceState.h>

/* Constant definitions */
#define CONTROL_TIMING_HZ 4              // How frequent to check whether it is time to run a plan or not
#define BATTERY_MONITOR_HZ 0.25
#define PLAN_REFRESH_HZ 1./60.
#define CONTROL_PAUSED true
#define CONTROL_RESUMED false
#define VOLTAGE_AVERAGE_DECAY_FACTOR 0.95
#define ACTION_COMPLETED_POLL_HZ 5
#define MAX_STORAGE_THRESHOLD 0.85       // Robot should stop and return at n% storage capacity
#define REBOOT_RECOVERY_TIMEOUT_S 7200   // The maximum period since the last action that the robot will
                                         // attempt recovery from a reboot

/* Mutex definitions */
static std::mutex idleMutex;            // Protects the idle resource
static std::mutex batteryStateMutex;    // Protects the batteryState resource
static std::mutex planFileMutex;        // Protects the plan file resource
static std::mutex chargingMutex;        // Protects the charging status resource
static std::mutex actionsRunningMutex;  // Protects the actionsRunning resource
static std::mutex rfidCodeMutex;        // Protects the RFID code resource
static std::mutex planLoadedMutex;      // Protects the planLoaded resource
static std::mutex homeFoundMutex;       // Protects the homeFound resource
static std::mutex lostChargerSyncMutex; // Protects the lostChargerSync resource
static std::mutex storageFullMutex;     // Protects the storageFull resource

/* Static shared resources */
static bool idle;              // True when the control node is allowed to run
static BobStatus batteryState; // Indicates the battery state
static bool homeFound;         // True when the robot knows its position
static std::string planFile;   // Filename of the plan to run
static bool charging;          // True if the robot has initiated charging
static bool actionsRunning;    // True if a plan is currently running
static float averageVoltage;   // The moving average of the battery voltage
static uint32_t rfidCode;      // The RFID code
static bool planLoaded;        // True if a plan has been loaded
static bool lostChargerSync;   // True if the robot loses sync with the charger
static bool storageFull;       // True if there isn't enough space to store images

/* Static resources */
static bool cancelledActions;   // True when a cancel action request has been sent

/* Function declarations */
void publishState(ros::Publisher *, bool state);
void controlLoop(ActionClient *, ros::Publisher *, TrackPlan *);
BobStatus executeCommand(ActionClient*, ros_robo_bob::ImageMeta*, TrackPlan*, std::vector<std::string>);
BobStatus runCommand(std::vector<std::string>, ActionClient *, ros::Publisher *, TrackPlan *, std::vector<int> *);
void runPlanCallback(const ros::TimerEvent &, ActionClient *, ros::Publisher *, TrackPlan *);
bool controlModeCB(ros_robo_bob::ControlMode::Request &, ros_robo_bob::ControlMode::Response &,
                   ros::Publisher *);

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @statePub             Pointer to the state publisher                    #
#           @state                The state to publish                              #
#                                                                                   #
#   Description: Publishes the state of the control node.                           #
#                                                                                   #
************************************************************************************/

void publishState(ros::Publisher *statePub, bool state)
{
    std_msgs::Bool stateMsg;

    // Publish our state
    stateMsg.data = state;
    statePub->publish(stateMsg);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @state                Desired state of charge maintenance               #
#                                                                                   #
#   Description: Turns charge maintenance on or off.                                #
#                                                                                   #
************************************************************************************/

void setChargeMaintenenceState(bool state)
{
    ros::NodeHandle handle;

    // Create the charge state client
    ros::ServiceClient chargeStateClient = handle.serviceClient<ros_robo_bob::ChargeMaintenanceState>
                                            (addBase2Topic(POWER_NODE_NAME, "charge_maintenance"));

    // Create and populate a request
    ros_robo_bob::ChargeMaintenanceState server;
    server.request.state = state;

    // Make the service call
    chargeStateClient.call(server);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @status               The status of the charger node                    #
#                                                                                   #
#   Description: Starts a critical_charge if the charger loses sync with the        #
#                wireless transmitter.                                              #
#                                                                                   #
************************************************************************************/

void chargeStatusCB(const std_msgs::Int8::ConstPtr &status, ActionClient *actionClient, TrackPlan *plan)
{
    if (status->data == BOB_LOST_CHARGER_SYNC)
    {
        // Record that we've lost sync with the charger
        setResource<bool>(chargingMutex, charging, false);
        setResource<bool>(lostChargerSyncMutex, lostChargerSync, true);
    }
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @req                  The request for the service call                  #
#           @res                  The response for the service call                 #
#           @actionClient         Pointer to the action client                      #
#                                                                                   #
#   Description: Services any client calls to set the control state. The callback   #
#                stops any running actions by default, and runs a plan if the       #
#                state is set to true.                                              #
#                                                                                   #
************************************************************************************/

bool controlModeCB(ros_robo_bob::ControlMode::Request &req, ros_robo_bob::ControlMode::Response &res,
                   ros::Publisher *actionCancelPub, TrackPlan *plan)
{

    std_msgs::Empty cancelMessage;
    BobStatus status = BOB_SUCCESS;

    // Open the control state file
    std::fstream stateFile(CONTROL_STATE_FILENAME, std::ofstream::out | std::ofstream::trunc);

    // Set the control loop to idle
    setResource<bool>(idleMutex, idle, true);

    // Cancel any actions if they are running
    if (readResource<bool>(actionsRunningMutex, actionsRunning))
        actionCancelPub->publish(cancelMessage);

    // Wait until any actions have completed
    while (readResource<bool>(actionsRunningMutex, actionsRunning))
        ros::Duration(1. / ACTION_COMPLETED_POLL_HZ).sleep();

    // Stop charging if we're charging
    setResource<bool>(chargingMutex, charging, false);
    setChargeMaintenenceState(false);

    // User conrol mode if set to true
    if (req.mode == "manual")
    {
        if (req.planName != "idle")
        {
            // Clear the plan
            plan->clear();

            status = plan->loadTrackByFilename(req.planName);

            // Attempt load the plan
            if (status == BOB_SUCCESS)
            {
                setResource<bool>(planLoadedMutex, planLoaded, true);
                setResource<bool>(idleMutex, idle, false);
            }
        }
    }
    // Re-load the the plan from the RFID code if one has be detected
    else if (req.mode == "auto")
    {
        if (readResource<uint32_t>(rfidCodeMutex, rfidCode) != 0)
        {
            plan->clear();

            status = plan->loadTrackFromRFID(readResource<uint32_t>(rfidCodeMutex, rfidCode));

            if (status == BOB_SUCCESS)
            {
                setResource<bool>(planLoadedMutex, planLoaded, true);
            }
        }

        // No longer idling
        setResource<bool>(idleMutex, idle, false);

    }

    // Update the control state file if the state was set succesfully
    if (status == BOB_SUCCESS && stateFile.is_open())
    {
        stateFile << req.mode << "\n";   
        stateFile.close();

        // Compute the checksum
        writeChecksumFile(CONTROL_STATE_FILENAME);
    }
    
    // Record the status in the response
    res.status = status;

    return true;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @actionClient         Pointer to the actionClient being used            #
#           @imageMetaPub	  Pointer to the image metadata publisher           #
#           @plan                 Pointer to the plan object                        #
#           @cmd                  The command to run                                #
#   Outputs:                                                                        #
#           @returns              Status code indicating the outcome of the action  #
#                                                                                   #
#   Description: Runs a single command using ActionClient                           #
#                                                                                   #
************************************************************************************/

BobStatus executeCommand(ActionClient *actionClient, ros::Publisher *imageMetaPub, 
                         TrackPlan *plan, std::vector<std::string> cmd)
{
    BobStatus status = BOB_SUCCESS;

    if (cmd[0] == "move")
    {
        ros_robo_bob::MoveGoal goal;
        ros_robo_bob::MoveResult result;

        // Set the goal distance
        goal.distance = stof(cmd[1]);

        // Execute the command
        status = actionClient->moveRB(goal, result);
    }
    else if (cmd[0] == "seek")
    {
        ros_robo_bob::SeekGoal goal;
        ros_robo_bob::SeekResult result;
    
        // Set the goal distance
        goal.distance = stof(cmd[1]);
        
        // Execute the command
        status = actionClient->seekRB(goal, result);
    }
    else if (cmd[0] == "move_camera")
    {
        ros_robo_bob::MoveCameraGoal goal;
        ros_robo_bob::MoveCameraResult result;

        // Set the goal orientation
        goal.orientation[0] = stof(cmd[1]);
        goal.orientation[1] = stof(cmd[2]);
        goal.orientation[2] = stof(cmd[3]);

        // Execute the command
        status = actionClient->moveCameraRB(goal, result);
    }
    else if (cmd[0] == "find_charger")
    {
        ros_robo_bob::FindChargerGoal goal;
        ros_robo_bob::FindChargerResult result;

        // Execute the command
        status = actionClient->findChargerRB(goal, result);
    }
    else if (cmd[0] == "capture_image")
    {
        ros_robo_bob::CaptureImageGoal goal;
        ros_robo_bob::CaptureImageResult result;

        // Set the auto white balance state
        goal.autoWB = plan->getAutoWB();

        // Execute the command
        status = actionClient->captureImageRB(goal, result);

        if (status == BOB_SUCCESS)
        {
            ros::NodeHandle handle;
            ros_robo_bob::GimbalOrientation gimbalOrientationServer;
            ros_robo_bob::GetPosition positionServer;
            ros_robo_bob::ImageMeta imageMeta;

            // Service client to get the robot position
            ros::ServiceClient positionClient = handle.serviceClient<ros_robo_bob::GetPosition>
                                                (addBase2Topic(ODOM_NODE_NAME, "get_position"));

            // Service client to get the gimbal orientation
            ros::ServiceClient gimbalOrientationClient = handle.serviceClient<ros_robo_bob::GimbalOrientation>
                                                        (addBase2Topic(GIMBAL_NODE_NAME, "get_orientation"));

            // Obtain the position of the robot
            positionClient.call(positionServer);

            // Obtain the orientation of the gimbal
            gimbalOrientationClient.call(gimbalOrientationServer);

            // Save the image and construct metadata
            processImage(result, imageMeta, plan, stoi(cmd[1]), gimbalOrientationServer.response.orientation,
                             positionServer.response.position);

            // Publish the image metadata
            imageMetaPub->publish(imageMeta);
        }
    }
    else if (cmd[0] == "find_home")
    {
        // Only execute if the robot doesn't know where it is, or if we explicitly force it to
        if (!readResource<bool>(homeFoundMutex, homeFound)
            || (cmd.size() > 1 && cmd[1] == "force"))
        {
            ros_robo_bob::FindHomeGoal goal;
            ros_robo_bob::FindHomeResult result;

            // Execute the command
            status = actionClient->findHomeRB(goal, result);

            // Mark location as known
            if (status == BOB_SUCCESS)
                setResource<bool>(homeFoundMutex, homeFound, true);
        }
    }
    else if (cmd[0] == "find_end")
    {
        ros_robo_bob::FindEndGoal goal;
        ros_robo_bob::FindEndResult result;

        // Execute the command
        status = actionClient->findEndRB(goal, result);

    }

    else if (cmd[0] == "start_charging")
    {
        // Record that charging has started
        setResource<bool>(chargingMutex, charging, true);

        setChargeMaintenenceState(true);
    }
    else if (cmd[0] == "stop_charging")
    {
        // Record that charging has ended
        setResource<bool>(chargingMutex, charging, false);

        setChargeMaintenenceState(false);
    }
    else if (cmd[0] == "read_rfid")
    {
        ros::NodeHandle handle;

        ros_robo_bob::ReadRFID rfidServer;
        ros::ServiceClient rfidClient = handle.serviceClient<ros_robo_bob::ReadRFID>
                                        (addBase2Topic(MFRC522_NODE_NAME, "scan_for_tag"));

        // Attempt to read the tag
        rfidClient.call(rfidServer);
        status = rfidServer.response.status;

        // Save the RFID code if it was found
        if (rfidServer.response.found)
            setResource<uint32_t>(rfidCodeMutex, rfidCode, rfidServer.response.tagID);
    }
    else if (cmd[0] == "get_last_tag")
    {
        ros::NodeHandle handle;

        ros_robo_bob::GetLastRFID rfidServer;
        ros::ServiceClient rfidClient = handle.serviceClient<ros_robo_bob::GetLastRFID>
                                        (addBase2Topic(MFRC522_NODE_NAME, "get_last_tag"));

        // Attempt to read the tag
        rfidClient.call(rfidServer);
        status = rfidServer.response.status;

        // Save the RFID code
        if(status == BOB_SUCCESS)
            setResource<uint32_t>(rfidCodeMutex, rfidCode, rfidServer.response.tagID);
    }
    else if (cmd[0] == "gimbal_power")
    {

        ros::NodeHandle handle;

        ros_robo_bob::PowerGimbal gimbalPowerServer;
        ros::ServiceClient gimbalPowerClient = handle.serviceClient<ros_robo_bob::PowerGimbal>
                                        (addBase2Topic(GIMBAL_NODE_NAME, "power_gimbal"));

        // Set the desired gimbal state
        if(cmd[1] == "on")
            gimbalPowerServer.request.power = true;
        else
            gimbalPowerServer.request.power = false;

        gimbalPowerClient.call(gimbalPowerServer);
    }
    else if(cmd[0] == "stream_state")
    {
        ros::NodeHandle handle;
      
        ros_robo_bob::StartStopStreaming streamStateServer;
        ros::ServiceClient streamStateClient = handle.serviceClient<ros_robo_bob::StartStopStreaming>
                                         (addBase2Topic(LUMENARA_NODE_NAME, "start_stop_streaming"));
          
        // Set the desired streaming state
        if(cmd[1] == "on")
            streamStateServer.request.streaming = true;
        else
            streamStateServer.request.streaming = false;
      
        streamStateClient.call(streamStateServer);
    }
    else if (cmd[0] == "auto_wb")
    {
        if(cmd[1] == "on")
            plan->setAutoWB(true);
        else
            plan->setAutoWB(false);
    }
    else if (cmd[0] == "set_stop_number")
    {
        // Set the stop iteration
        plan->setStopIteration(stoi(cmd[1]));

        // Open the stop number file
        std::fstream stopFile(STOP_NUMBER_FILENAME, std::ofstream::out | std::ofstream::trunc);

        // Update the stop number file
        if (stopFile.is_open())
        {
            stopFile << stoi(cmd[1]) << "\n";   
            stopFile.close();

            // Compute the checksum
            writeChecksumFile(STOP_NUMBER_FILENAME);
        }
    }
    else if(cmd[0] == "cpu_scale")
    {
        // Construct the scaling command
        std::string sysCommand = std::string("echo ") + cmd[1] 
                                 + std::string(" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq");
        
        // Issue the command
        std::system(sysCommand.c_str());
    }

    return status;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @vec                  Vector of strings to search through               #
#           @element              String element to search for                      #
#   Outputs:                                                                        #
#           @returns              Index of the element                              #
#                                                                                   #
#   Description: Searches a vector of strings for an element, and returns the       #
#                index if found, and -1 if it is not found.                         #
#                                                                                   #
************************************************************************************/

int findElement(std::vector<std::string> &vec, std::string element)
{
    for (int i = 0; i < vec.size(); i++)
    {
        if (vec[i] == element)
            return i;
    }

    return -1;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Outputs:                                                                        #
#           @programCounter       Program counter value to push                     #
#                                                                                   #
#   Description: Pushes a value to the bottom of the stack file.                    #
#                                                                                   #
************************************************************************************/

void pcStackPush(int programCounter)
{
    // Open the stack file
    std::fstream stackFile(PC_STACK_FILENAME, std::fstream::in | std::fstream::out | std::fstream::app);

    // Make sure the file is open
    if (stackFile.is_open())
    {
        // Write the program counter to
        stackFile << programCounter << "\n";

        // Close the file
        stackFile.close();

        // Compute the checksum
        writeChecksumFile(PC_STACK_FILENAME);
    }
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Outputs:                                                                        #
#           @returns              True if the stack isn't empty                     #
#                                                                                   #
#   Description: Pops a value off the bottom stack (FILO).                          #
#                                                                                   #
************************************************************************************/

bool pcStackPopFILO(int & programCounter)
{
    std::string line;
    std::vector<int> stack;

    if(!checkFileIntegrity(PC_STACK_FILENAME))
        return false;

    // Open the stack file
    std::fstream stackFile(PC_STACK_FILENAME);

    // Make sure the file is open
    if (!stackFile.is_open())
        return false;
    

    // Read in each line
    while (std::getline(stackFile, line))
    {
        std::istringstream stream(line);

        // Get the program counter from the line
        stream >> programCounter;
        stack.push_back(programCounter);

    }

    // Close the file
    stackFile.close();

    // Return false if the stack is empty
    if(stack.size() == 0)
        return false;

    // Re-open the file and overwrite it
    stackFile.open(PC_STACK_FILENAME, std::fstream::trunc | std::fstream::out);

    // Make sure the file is open
    if (!stackFile.is_open())
        return false;
    
    // Add the values back in, ignoring the last line
    for(int i = 0; i < stack.size()-1; i++)
            stackFile << stack[i] << "\n";

    // Remove the value from the
    stack.erase(stack.begin()+stack.size()-1);

    // Close the file
    stackFile.close();

    // Compute the checksum
    writeChecksumFile(PC_STACK_FILENAME);

    // Return true if the stack isn't empty
    return true;

}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Outputs:                                                                        #
#           @returns              True if the stack contains data, false otherwise  #
#                                                                                   #
#   Description: Obtains a copy of the program counter stack.                       #
#                                                                                   #
************************************************************************************/

bool pcStackCopy(std::vector<int> & stackCopy)
{
    std::string line;
    int programCounter;

    if(!checkFileIntegrity(PC_STACK_FILENAME))
        return false;

    // Open the stack file
    std::fstream stackFile(PC_STACK_FILENAME);

    // Make sure the file is open
    if (!stackFile.is_open())
        return false;

    // Read in each line
    while (std::getline(stackFile, line))
    {
        std::istringstream stream(line);

        // Get the program counter from the line
        stream >> programCounter;
        stackCopy.push_back(programCounter);
    }

    // Close the file
    stackFile.close();

    // Return true if the stack isn't empty
    return (stackCopy.size() > 0);

}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @command              The command to run                                #
#           @actionClient         Pointer to the actionClient being used            #
#           @plan                 Pointer to the plan object                        #
#   Outputs:                                                                        #
#           @returns              Status code indicating the outcome of the         #
#                                 action                                            #
#                                                                                   #
#   Description: Executes either a low-level or block command. Block commands are   #
#                recursively evaluated, making nested commands and retries          #
#                possible. If retries are specificed, this will handle the retry    #
#                logic as well.                                                     #
#                                                                                   #
************************************************************************************/

BobStatus runCommand(std::vector<std::string> command, ActionClient *actionClient,
                     ros::Publisher * imageMetaPub, TrackPlan *plan, std::vector<int> * stackCopy=NULL)
{
    int stackPC;
    int retryPos;
    int maxRetries;
    BobStatus status;
    int progCounter = 0;
    int retryCounter = 0;
    bool hardRetry = false;

    // Look for a retry clause
    retryPos = findElement(command, "retry");

    // If a retry isn't specified, look for a hard retry
    if(retryPos == -1)
    {
        retryPos = findElement(command, "hard_retry");

        if(retryPos != -1)
            hardRetry = true;
    }

    // Save the number of retries if it is specified
    if (retryPos != -1)
        maxRetries = std::stoi(command[retryPos + 1]);

    // Log the command we're about to run
    ROS_ERROR("RUNNING COMMAND %s", command[0].c_str());

    for (;;)
    {
        // Check to see if this is a block command
        if ((*plan->getBlocks()).count(command[0]))
        {

            // Loop through each command in the block, starting at the specified program coutner
            while (progCounter < (*plan->getBlocks())[command[0]].size())
            {
                // Push the program counter to the file stack if no stack is specified
                if(stackCopy != NULL && stackCopy->size() == 0)
                    pcStackPush(progCounter);
                

                // Pop the program counter off of the stack copy and use it if the stack has
                // data.
                else if(stackCopy != NULL && stackCopy->size() > 0)
                {
                    progCounter = (*stackCopy)[0];
                    stackCopy->erase(stackCopy->begin());
                }

                // Recursively evaluate the command
                status = runCommand((*plan->getBlocks())[command[0]][progCounter], actionClient, imageMetaPub, plan, stackCopy);

                // Pop the command off of the top of the stack
                pcStackPopFILO(stackPC);

                // Stop the block if there is an issue
                if (status != BOB_SUCCESS)
                    break;

                // Increment the program counter
                progCounter++;
            }
        }
        else
            status = executeCommand(actionClient, imageMetaPub, plan, command);

        if (status != BOB_SUCCESS && status != BOB_ACTION_CANCELLED)
        {
            // See if a retry is specified
            if (retryPos != -1)
            {
                // Check to see if we've exceeded the specified number of retries
                if (retryCounter == maxRetries - 1)
                {
                    return status;
                }
                else
                {
                    // Increment retry counter
                    retryCounter++;

                    // Reset the counter if this is a hard retry
                    if(hardRetry)
                        progCounter = 0;

                    // Check and see if any retry commands are provided (must be a block)
                    if (command.size() > retryPos + 2)
                    {
                        std::vector<std::string> blockCmds;
                        
                        // Load in the retry command
                        for(int i = retryPos+2; i < command.size(); i++)
                             blockCmds.push_back(command[i]);
                      
                        runCommand(blockCmds, actionClient, imageMetaPub, plan);
                    }
                }
            }
            else
                return status;
        }
        else
            return status;
    }
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Outputs:                                                                        #
#           @returns              True for a clean start, false otherwise           #
#                                                                                   #
#   Description: Checks the stack file to see if its last modification time is      #
#                within a specific time window. If it is, a reboot most likely      #
#                occured, otherwise it will delete the stale stack file. On a       #
#                reboot, the function will attempt to read the stack, get the       #
#                last known RFID code, and load the plan. If any of these actions   #
#                aren't succesfull for some reason, the function returns as if      #
#                a reboot didn't occur so that a fresh plan can be started. If the  #
#                robot has been in auto mode, and the stack file is still fresh,    #
#                a clean start will be reported, but it will go out of idle mode    #
#                so that plan runs can resume.                                      #
#                                                                                   #
************************************************************************************/

bool handleReboot(std::vector<int> & stackCopy, ActionClient *actionClient, ros::Publisher *imageMetaPub, TrackPlan *plan)
{

    struct stat attr;

    // Used to keep track of the stop number the plan was at before reboot
    int stopNumber = -1;

    // Used to keep track of what state the control node was in before reboot
    std::string controlState;

    // Obtain the statistics on the file
    stat(PC_STACK_FILENAME, &attr);

    // Make sure the state file isn't corrupted
    if(checkFileIntegrity(CONTROL_STATE_FILENAME))
    {
        // Open the state file
        std::ifstream stateFile(CONTROL_STATE_FILENAME);

        // Make sure the file is open
        if (!stateFile.is_open())
            return true;

        // Read in the control state
        stateFile >> controlState;
    }

    // Make sure the stop number file isn't corrupted
    if(checkFileIntegrity(STOP_NUMBER_FILENAME))
    {
        // Open the state file
        std::ifstream stopFile(STOP_NUMBER_FILENAME);

        // Make sure the file is open
        if (!stopFile.is_open())
            return true;

        // Read in the stop number
        stopFile >> stopNumber;

        ROS_ERROR("STOP NUMBER: %d", stopNumber);
    }

    // Check and see if we're in the recovery time window, or if we were not in "auto" mode
    if(((std::time(NULL)-attr.st_mtime > REBOOT_RECOVERY_TIMEOUT_S ) && controlState != "auto") || controlState == "idle" || controlState == "manual")
    {
        // Remove the stack file
        std::remove(PC_STACK_FILENAME);

        return true;
    }

    // Obtain a copy of the pc stack and check if it has data in it
    bool stackNotEmpty = pcStackCopy(stackCopy);

    // Load the last known plan if the stack had data or the robot was in auto mode within the stack
    // file timeout
    if(stackNotEmpty || controlState == "auto")
    {
        // Read the last RFID tag if one exists
        if(runCommand({"get_last_tag"}, actionClient, imageMetaPub, plan) != BOB_SUCCESS)
        {
            // Remove the stack file
            std::remove(PC_STACK_FILENAME);

            return true;
        }

        // Load the track based on the RFID value
        if(plan->loadTrackFromRFID(readResource<uint32_t>(rfidCodeMutex, rfidCode)) != BOB_SUCCESS)
        {
            // Remove the stack file
            std::remove(PC_STACK_FILENAME);

            return true;
        }

        // Set the last known stop number
        if(stopNumber != -1)
            plan->setStopIteration(stopNumber);

        // Record that a plan has been loaded
        setResource<bool>(planLoadedMutex, planLoaded, true);

    }
    else
    {
        return true;
    }

    // Turn off idle mode to allow the plan to resume
    setResource<bool>(idleMutex, idle, false);

    // If the stack is empty and we're still in auto mode, report a clean start
    if(!stackNotEmpty && controlState == "auto")
        return true;
    

    return false;

}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @actionClient         Pointer to the actionClient being used            #
#           @planStatusPub        Pointer to the plan status publisher              #
#	    @imageMetaPub         Pointer to the image metadata publisher           #
#           @plan                 Pointer to the plan object                        #
#                                                                                   #
#   Description: Activated by a timer callback, this is the main control funciton   #
#                that first queries the gatekeeper for a plan, and then executes    #
#                it. This function can be "pause" by taking the ctrMutex.           #
#                                                                                   #
************************************************************************************/

//@TODO add in startDelay logic

void controlLoop(ActionClient *actionClient, ros::Publisher *planStatusPub, ros::Publisher *imageMetaPub, TrackPlan *plan)
{
    bool alreadyRan = false;

    // Used to time refreshing the plan
    int planRefreshTicks = 0;
  
    // Holds a copy of the stack in the case of a reboot
    std::vector<int> stackCopy;

    // Status of the system
    BobStatus status = BOB_SUCCESS;

    // Rate at which to check if it is time to run the next plan
    ros::Rate rate(CONTROL_TIMING_HZ);

    // Check if the program counter stack is full and act accordingly
    bool cleanStart = handleReboot(stackCopy, actionClient, imageMetaPub, plan);

    while (ros::ok())
    {
        // Initialize the status
        status = BOB_SUCCESS;

        // Get the battery status
        BobStatus batteryStatus = readResource<BobStatus>(batteryStateMutex, batteryState);

        // Don't do anything in idle mode
        if (readResource<bool>(idleMutex, idle))
        {
            alreadyRan = false;
        }
        // Return to charge if the storage is full
        else if (!readResource<bool>(chargingMutex, charging) && readResource<bool>(storageFullMutex, storageFull))
        {
            // Record that charging has started
            setResource<bool>(chargingMutex, charging, true);
          
            // Return and charge
            status = runCommand({"charge", "hard_retry", "3"}, actionClient, imageMetaPub, plan);
          
            // Record that we're not charging if a failure occured
            if (status != BOB_SUCCESS)
                setResource<bool>(chargingMutex, charging, false);

            // Record that we've no longer lost sync with the charger
            else
                setResource<bool>(lostChargerSyncMutex, lostChargerSync, false);
        }
        // Find home, then scan for the RFID tag and load the plan once found
        else if (!readResource<bool>(planLoadedMutex, planLoaded))
        {
            alreadyRan = false;

            // Find home and read RFID
            status = runCommand({"localize"}, actionClient, imageMetaPub, plan);

            if (status == BOB_SUCCESS)
            {
                // Load the track based on the RFID value
                status = plan->loadTrackFromRFID(readResource<uint32_t>(rfidCodeMutex, rfidCode));
            }

            if (status == BOB_SUCCESS)
            {
                // Record that a plan has been loaded
                setResource<bool>(planLoadedMutex, planLoaded, true);
            }
        }
        // Check the battery status
        else if (batteryStatus != BOB_BATTERY_OK && batteryStatus != BOB_BATTERY_UNKNOWN && cleanStart)
        {
            alreadyRan = false;

            // Shut down if neccesary
            if (batteryStatus == BOB_BATTERY_SHUTDOWN)
            {
                //@TODO add log for shutdown
                std::system("shutdown now");
            }

            // Only run block if the robot hasn't started charging, or if a plan isn't running and
            // the battery is low
            if (!readResource<bool>(chargingMutex, charging) || (!readResource<bool>(chargingMutex, charging)
                && batteryStatus == BOB_BATTERY_LOW && !readResource<bool>(actionsRunningMutex, actionsRunning))
                || readResource<bool>(lostChargerSyncMutex, lostChargerSync))
            {
                // Record that charging has started
                setResource<bool>(chargingMutex, charging, true);

                // Return and charge
                status = runCommand({"charge", "hard_retry", "3"}, actionClient, imageMetaPub, plan);

                // Record that we're not charging if a failure occured
                if (status != BOB_SUCCESS)
                    setResource<bool>(chargingMutex, charging, false);

                // Record that we've no longer lost sync with the charger
                else
                    setResource<bool>(lostChargerSyncMutex, lostChargerSync, false);
            }
        }
        // Check if the current time is a scheduled one
        else if ((plan->readyToRun() || !cleanStart) && batteryStatus != BOB_BATTERY_UNKNOWN
                 && !readResource<bool>(storageFullMutex, storageFull))
        {
            // Used to ensure the plan doesn't run twice in the same minute.
            // This imposes the constraint that a plan must have an interval of at least (60 + 1/CONTROL_TIMING_HZ)
            // seconds

            if (!alreadyRan)
            {
                // Record that we are no longer charging
                setResource<bool>(chargingMutex, charging, false);

                // Record that a plan run has begun
                setResource<bool>(actionsRunningMutex, actionsRunning, true);

                // Run the plan commands
                status = runCommand({"plan", "retry", "3"}, actionClient, imageMetaPub, plan, &stackCopy);

                // Record that athe plan has finished
                setResource<bool>(actionsRunningMutex, actionsRunning, false);

                // Set home location to uncertain if we've encountered an error
                if (status != BOB_SUCCESS && status != BOB_ACTION_CANCELLED)
                {
                    alreadyRan = false;

                    // If there was an error, we need to localize ourselves again
                    //setResource<bool>(homeFoundMutex, homeFound, false);
                }

                // Record that the robot has alread made a plan run in this epoch
                alreadyRan = true;

                // Record that the plan has finished cleanly
                cleanStart = true;
            }
        }
        else
        {
            alreadyRan = false;
        }

        // Refresh the plan at a certain rate
        if(planRefreshTicks == ((float)CONTROL_TIMING_HZ)/((float)PLAN_REFRESH_HZ))
        {
            // Only refresh if a plan has been loaded, and a plan isn't already running
            if(readResource<bool>(planLoadedMutex, planLoaded) 
               && !readResource<bool>(actionsRunningMutex, actionsRunning))
                status = plan->loadTrackFromRFID(readResource<uint32_t>(rfidCodeMutex, rfidCode));
          
            planRefreshTicks = 0;
        }
        else
        {
            planRefreshTicks++; 
        }
        
      
        // Advertise the status of the control loop
        handleStatus(status, planStatusPub);

        rate.sleep();
    }
}

void cancelActions(ros::Publisher *actionCancelPub)
{
    std_msgs::Empty cancelMessage;

    // Cancel all actions currently being made if the robot isn't already charging
    // and if a cancel command hasn't already been sent, and only if a plan is running
    if (!cancelledActions && !readResource<bool>(chargingMutex, charging)
        && readResource<bool>(actionsRunningMutex, actionsRunning))
    {
        // Record that actions have already been cancelled to avoid multiple cancels
        cancelledActions = true;
        actionCancelPub->publish(cancelMessage);
    }
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @event                ROS timer event object, not used                  #
#           @batVoltageClient     Service client for the battery voltage            #
#           @actionClient         Pointer to the actionClient being used            #
#           @statusPub            Pointer to the status publisher                   #
#           @plan                 Pointer to the plan object                        #
#                                                                                   #
#   Description: Activated by a timer callback, this monitors the battery voltage   #
#                and adjust the battery state/tackes action as needed.              #
#                                                                                   #
************************************************************************************/

void healthMonitorCB(const ros::TimerEvent &event, ros::ServiceClient *batVoltageClient,
                      ActionClient *actionClient, ros::Publisher *statusPub, ros::Publisher *actionCancelPub,
                      TrackPlan *plan)
{
    ros_robo_bob::GetVoltage server;

    // Only continue if a plan is loaded
    if(!plan->isLoaded())
        return;

    // Compute the % available space
    boost::filesystem::space_info spaceInfo = boost::filesystem::space(boost::filesystem::path("/"));
    float spaceAvailable = (float)spaceInfo.available/(float)spaceInfo.capacity;

    // Cancel any actions if the storage is too full
    if((1.-spaceAvailable) > MAX_STORAGE_THRESHOLD)
    {
        // Record the storage as full
        setResource<bool>(storageFullMutex, storageFull, true);

        // Cancel any running actions
        cancelActions(actionCancelPub);
    }
    else
    {
        setResource<bool>(storageFullMutex, storageFull, false);
    }

    // Make a call to the voltage service
    bool voltageServiceStatus = batVoltageClient->call(server);
  
    // Check the voltage
    if (voltageServiceStatus && server.response.status == BOB_SUCCESS)
    {
        // Update the voltage moving average
        movingAverage(averageVoltage, (float)server.response.voltage, VOLTAGE_AVERAGE_DECAY_FACTOR);

        // Voltage at which the robot should shutdown
        if ((int)averageVoltage <= plan->getPowerInfo()->shutdownVoltage)
        {
            ROS_ERROR("SHUTDOWN: %f %d", averageVoltage, plan->getPowerInfo()->shutdownVoltage);

            // Indicate that a shutdown needs to occure
            setResource<BobStatus>(batteryStateMutex, batteryState, BOB_BATTERY_SHUTDOWN);
        }
        // Voltage at which to stop a plan and return
        else if ((int)averageVoltage <= plan->getPowerInfo()->stopPlanVoltage)
        {
            // Set the battery state to critical
            setResource<BobStatus>(batteryStateMutex, batteryState, BOB_BATTERY_CRITICAL);

            // Cancel any running actions
            cancelActions(actionCancelPub);
        }
        // Minimum voltage to run a plan
        else if ((int)averageVoltage <= plan->getPowerInfo()->minPlanVoltage)
        {
            // Set the battery state to low
            setResource<BobStatus>(batteryStateMutex, batteryState, BOB_BATTERY_LOW);
        }
        else
        {
            // Reset to allow another cancel to occur
            cancelledActions = false;

            // Set the battery state to OK
            setResource<BobStatus>(batteryStateMutex, batteryState, BOB_BATTERY_OK);
        }
    }
    // Publish the error if one occured
    else if (server.response.status != BOB_SUCCESS)
    {
        handleStatus(server.response.status, statusPub);
    }
    else if (!voltageServiceStatus)
    {
        handleStatus(BOB_VOLTAGE_SERVICE_FAILED, statusPub);
    }
}

int main(int argc, char **argv)
{
    TrackPlan plan;

    // Initialize static variables
    idle = true;
    rfidCode = 0;
    charging = false;
    homeFound = false;
    planLoaded = false;
    averageVoltage = 0.;
    storageFull = false;
    actionsRunning = false;
    lostChargerSync = false;
    cancelledActions = false;
    batteryState = BOB_BATTERY_UNKNOWN;

    // Initialize the node
    ros::init(argc, argv, CONTROL_NODE_NAME);

    // The ros handle for this node
    ros::NodeHandle handle;

    // Create a folder for the bags if it hasn't already been created
    createDir(IMAGES_PATH);

    // Create the data dir if it doesn't alread exist yet
    createDataDir();

    // The action client to execute commands with
    ActionClient actionClient;

    // Spinner needs to run asynchronously on its own thread so we can service callbacks
    // and run the control loop simultaneously
    ros::AsyncSpinner spinner(2);

    // Create a publisher to let the gatekeeper know the status of the BOB
    ros::Publisher planStatusPub = handle.advertise<std_msgs::UInt8>("status", 1);

    // Create a publisher to let the motion know when to cancel an action
    ros::Publisher actionCancelPub = handle.advertise<std_msgs::Empty>(addBase2Topic(CONTROL_NODE_NAME, "cancel_action"), 1);
    
    // Create a image publisher to publish image metadata
    ros::Publisher imageMetaPub = handle.advertise<ros_robo_bob::ImageMeta>(addBase2Topic(CONTROL_NODE_NAME, "image_meta"), 10);

    // Create the control node state service
    ros::ServiceServer controlModeService = handle.advertiseService<ros_robo_bob::ControlMode::Request,
                                                                    ros_robo_bob::ControlMode::Response>
                                                                    (addBase2Topic(CONTROL_NODE_NAME, "mode"),
                                                                    boost::bind(controlModeCB, _1, _2, &actionCancelPub, &plan));

    // Create a service client to read battery voltage
    ros::ServiceClient batVoltageClient =
        handle.serviceClient<ros_robo_bob::GetVoltage>(addBase2Topic(POWER_NODE_NAME, "get_voltage"));

    // Create a timer for the battery monitoring
    ros::Timer batteryMonitorTimer = handle.createTimer(ros::Duration(1. / BATTERY_MONITOR_HZ),
                                                        boost::bind(healthMonitorCB, _1, &batVoltageClient, &actionClient,
                                                                    &planStatusPub, &actionCancelPub, &plan));

    // Create a subscriber to know when to retry a charge
    ros::Subscriber chargeRetrySub = handle.subscribe<std_msgs::Int8>(addBase2Topic(POWER_NODE_NAME, "charger_status"), 10,
                                                                      boost::bind(chargeStatusCB, _1, &actionClient, &plan));

    // Start the async spinners
    spinner.start();

    // Start the control loop
    controlLoop(&actionClient, &planStatusPub, &imageMetaPub, &plan);

    return 0;
}
