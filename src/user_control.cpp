/* General c++ includes */
#include <string>
#include <mutex>
#include <sstream>
#include <istream>
#include <boost/program_options.hpp>
#include <syslog.h>

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/Bool.h>

/* Package specific includes */
#include "../include/utils.hpp"
#include "../include/track_plan.hpp"
#include "../include/bob_status.hpp"
#include "../include/image_proc.hpp"
#include "../include/action_client.hpp"
#include "../include/bob_definitions.hpp"

/* Action includes */
#include <ros_robo_bob/SeekAction.h>
#include <ros_robo_bob/MoveAction.h>
#include <ros_robo_bob/MoveCameraAction.h>

/* Service Includes */
#include <ros_robo_bob/GetAbs.h>
#include <ros_robo_bob/ReadRFID.h>
#include <ros_robo_bob/FindHome.h>
#include <ros_robo_bob/GetCurrent.h>
#include <ros_robo_bob/GetVoltage.h>
#include <ros_robo_bob/ControlMode.h>
#include <ros_robo_bob/GetLastRFID.h>
#include <ros_robo_bob/PowerGimbal.h>
#include <ros_robo_bob/GetBatteryStatus.h>
#include <ros_robo_bob/StartStopStreaming.h>
#include <ros_robo_bob/ChargeMaintenanceState.h>

/* Constant definitions */
#define POWER_NODE_NAME "power_node"
#define MFRC522_NODE_NAME "mfrc522"
#define USER_CONTROL_NODE_NAME "user_control"
#define CONTROL_NODE_NAME "control"
#define WELCOME_MESSAGE "\nRoboBOB at your service! Enter a command or \"help\""
#define TERMINAL_PROMPT "\033[1;32mrobobob\033[0m: "
#define CHARGE_CURRENT_SAMPLE_SIZE 20
#define IMAGE_TEST_EDGE_API "https://sensor:fHcpmL5zcgQiUiq8zuUhVr9x1LDcPr0A@dp-api.iunu.com:2443/sensor"
#define IMAGE_TEST_CUSTOMER "lab"
#define IMAGE_TEST_FACILITY "solstice"
#define IMAGE_TEST_SPACE "manual" 

/* Mutex definitions */
std::mutex Mutex;

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @argc                 Pointer to the # of args                          #
#           @argv                 Pointer to the argv 2d array                      #
#   Outputs:                                                                        #
#           @modifies             argc and argv by reference                        #
#                                                                                   #
#   Description: Gets user input from the command line, and converts it to an       #
#                argc/argv pair to be used with boost::program_options.             #
#                                                                                   #
************************************************************************************/

void getCommandLineArgs(int *argc, char ***argv)
{

    // Get the user input
    std::string input;
    std::getline(std::cin, input);

    // Convert string to vector of strings delimited by spaces
    std::stringstream stream("ros " + input);
    std::istream_iterator<std::string> begin(stream);
    std::istream_iterator<std::string> end;
    std::vector<std::string> decomposed(begin, end);

    // Allocate memory for base elements
    *argv = (char **)malloc(decomposed.size() * sizeof(char *));

    // Populate each sub-element
    for (int i = 0; i < decomposed.size(); i++)
    {
        (*argv)[i] = (char *)malloc((decomposed[i].size() + 1) * sizeof(char));
        memcpy((*argv)[i], decomposed[i].c_str(), decomposed[i].size() + 1);
    }

    *argc = (char)decomposed.size();
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @argc                 Pointer to the # of args                          #
#           @argv                 Pointer to the argv 2d array                      #
#   Outputs:                                                                        #
#           @modifies             argv by reference                                 #
#                                                                                   #
#   Description: Erases the memory assosiated with argv.                            #
#                                                                                   #
************************************************************************************/

void freeArgs(int argc, char ***argv)
{
    for (int i = 0; i < argc; i++)
        delete[](*argv)[i];

    delete[] * argv;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @modeClient           The control node mode client                      #
#           @mode                 The desired mode of the control node              #
#           @planName             The name of the plan to run (optional)            #
#                                                                                   #
#   Description: Sets the mode of the control node.                                 #
#                                                                                   #
************************************************************************************/

BobStatus setControlMode(ros::ServiceClient &modeClient, std::string mode, std::string planName = "")
{
    ros_robo_bob::ControlMode server;

    // Make the service call
    server.request.planName = planName;
    server.request.mode = mode;
    modeClient.call(server);

    return server.response.status;
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
#           @testName             Name of the test that was ran                     #
#           @status               Status result of the test                         #
#                                                                                   #
#   Description: Prints out a test result given a name and status.                  #
#                                                                                   #
************************************************************************************/

bool printTestResult(std::string testName, BobStatus status=-1)
{
    if(status == BOB_SUCCESS)
    {
        std::cout << std::string("\033[1;32m") + testName+ std::string(": PASSED\033[0m\n");

        return true;
    }
    else
    {
        std::string message = std::string("\033[1;31m") + testName+ std::string(": FAILED");

        if(status != -1)
            message += " (" + std::string(BOB_STATUS_MESSAGES[status]) + ")\033[0m\n";
        else
            message += "\033[0m\n";

        std::cout << message;

        return false;
    }
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @msg                  An empty message, not used                        #
#                                                                                   #
#   Description: Creates a "robobob" terminal that accepts commands from the user   #
#                and attempts to execute them with ActionClient. The control loop   #
#                is executed when this node receives a "paused" handshake from the  #
#                control node.                                                      #
#                                                                                   #
************************************************************************************/

void userControlLoop(ros::ServiceClient &modeClient, ros::NodeHandle &handle)
{
    // Used to mimic commandline functionality
    int argc;
    char **argv;

    // Used during the system check
    TrackPlan plan = TrackPlan();
    
    // Update the edge API for testing
    plan.getCustomer()->edgeAPI = IMAGE_TEST_EDGE_API;
    plan.getCustomer()->name = IMAGE_TEST_CUSTOMER;
    plan.getLocation()->facility = IMAGE_TEST_FACILITY;
    plan.getLocation()->space = IMAGE_TEST_SPACE;
    
    // The action client to execute commands with
    ActionClient actionClient;

    // Action variables
    ros_robo_bob::SeekGoal seekGoal;
    ros_robo_bob::SeekResult seekResult;
    ros_robo_bob::FindEndGoal findEndGoal;
    ros_robo_bob::FindEndResult findEndResult;
    ros_robo_bob::FindHomeGoal findHomeGoal;
    ros_robo_bob::FindHomeResult findHomeResult;
    ros_robo_bob::MoveCameraGoal moveCameraGoal;
    ros_robo_bob::MoveCameraResult moveCameraResult;
    ros_robo_bob::FindChargerGoal findChargerGoal;
    ros_robo_bob::FindChargerResult findChargerResult;
    ros_robo_bob::CaptureImageGoal captureImageGoal;
    ros_robo_bob::CaptureImageResult captureImageResult;

    // Service variables
    ros_robo_bob::ReadRFID RFIDServer;
    ros_robo_bob::GetVoltage voltageServer;
    ros_robo_bob::PowerGimbal gimbalPowerServer;
    ros_robo_bob::GetCurrent chargeCurrentServer;

    // Service clients to get battery information
    ros::ServiceClient chargeCurrentClient =
        handle.serviceClient<ros_robo_bob::GetCurrent>(addBase2Topic(POWER_NODE_NAME, "get_current"));
    ros::ServiceClient batVoltageClient =
        handle.serviceClient<ros_robo_bob::GetVoltage>(addBase2Topic(POWER_NODE_NAME, "get_voltage"));
    ros::ServiceClient batChargeClient =
        handle.serviceClient<ros_robo_bob::GetAbs>(addBase2Topic(POWER_NODE_NAME, "get_abs"));
    ros::ServiceClient batStatusClient =
        handle.serviceClient<ros_robo_bob::GetBatteryStatus>(addBase2Topic(POWER_NODE_NAME, "get_battery_status"));
    ros::ServiceClient rfidClient =
        handle.serviceClient<ros_robo_bob::ReadRFID>(addBase2Topic(MFRC522_NODE_NAME, "scan_for_tag"));
    ros::ServiceClient gimbalPowerClient = 
        handle.serviceClient<ros_robo_bob::PowerGimbal>(addBase2Topic(GIMBAL_NODE_NAME, "power_gimbal"));

    // Create a image publisher to publish image metadata
    ros::Publisher imageMetaPub = handle.advertise<ros_robo_bob::ImageMeta>(addBase2Topic(CONTROL_NODE_NAME, "image_meta"), 10);

    // Shorten the program_options namespace name
    namespace po = boost::program_options;

    // Print the welcome message
    std::cout << WELCOME_MESSAGE << "\n\n";

    while (ros::ok())
    {
        std::cout << TERMINAL_PROMPT;

        // Convert user input to commandline form
        getCommandLineArgs(&argc, &argv);

        std::vector<std::string> options;
        std::string command;

        // Create the commandline options
        po::options_description desc("User control options");
        desc.add_options()("command,c", po::value<std::string>(), "command to execute on the robot")
                          ("options,o", po::value<std::vector<std::string>>()->multitoken(),
                           "command options (ie. distance value)")("exit", "exit the user_control node");

        po::positional_options_description positionalOptions;
        positionalOptions.add("command", 1);
        positionalOptions.add("options", 3);

        try
        {

            // Parse in the commandline arguments
            po::variables_map variablesMap;
            po::store(po::command_line_parser(argc, argv)
                          .options(desc)
                          .positional(positionalOptions)
                          .style(po::command_line_style::unix_style ^ po::command_line_style::allow_short)
                          .run(),
                      variablesMap);
            po::notify(variablesMap);

            // Handle a help request
            if (variablesMap.count("help"))
            {
                std::cout << desc << "\n";
                continue;
            }

            // Check to make sure the user supplied a command
            if (!variablesMap.count("command"))
            {
                ROS_ERROR("You must supply a command to use this node. Type \"help\" for more info.");
                continue;
            }
            else
                command = variablesMap["command"].as<std::string>();

            if (variablesMap.count("options"))
                options = variablesMap["options"].as<std::vector<std::string>>();
            
            // Stream User input
            syslog(LOG_NOTICE, "input comand from user: %s", command.c_str());
            
            // Handle the seek command
            if (command == "seek")
            {
                if (options.size() != 1)
                {
                    ROS_ERROR("Seek requires one distance value");
                    continue;
                }

                setControlMode(modeClient, "manual", "idle");

                // Get the distance data
                seekGoal.distance = std::stof(options[0]);

                // Issue command to the action server
                BobStatus status = actionClient.seekRB(seekGoal, seekResult);

                // Handle any errors
                printStatus(status);

                // Print out the RFID tag code
                ROS_INFO("Position: %f", seekResult.position);
            }
            // Handle the move command
            else if (command == "move")
            {
                if (options.size() != 1)
                {
                    ROS_ERROR("Move requires one distance value");
                    continue;
                }

                setControlMode(modeClient, "manual", "idle");

                ros_robo_bob::MoveGoal moveGoal;
                ros_robo_bob::MoveResult moveResult;

                // Get the distance data
                moveGoal.distance = std::stof(options[0]);

                // Issue command to the action server
                BobStatus status = actionClient.moveRB(moveGoal, moveResult);

                // Handle any errors
                printStatus(status);

                // Print out the current locaiton of the BOB
                ROS_INFO("Current position: %f", moveResult.position);
            }
            // Handle the camera orientation command
            else if (command == "move_camera")
            {
                if (options.size() != 1)
                {
                    ROS_ERROR("Gimbal orientation requires 1 angles");
                    continue;
                }

                setControlMode(modeClient, "manual", "idle");

                // Get the orientaition data
                std::vector<std::string> rot = options;
                moveCameraGoal.orientation = {std::stof(rot[0]), 0.,  0.};

                // Issue command to the action server
                BobStatus status = actionClient.moveCameraRB(moveCameraGoal, moveCameraResult);

                // Handle any errors
                printStatus(status);

                // Print out the current orientaition of the camera
                ROS_INFO("Current gimabl orientation: (%3.1f, %3.1f, %3.1f)",
                         moveCameraResult.orientation[0],
                         moveCameraResult.orientation[1],
                         moveCameraResult.orientation[2]);
            }
            else if (command == "find_charger")
            {
                // Disable charging
                setControlMode(modeClient, "manual", "idle");

                // Execute the command
                BobStatus status = actionClient.findChargerRB(findChargerGoal, findChargerResult);

                // Handle any errors
                printStatus(status);
            }
            else if (command == "find_home")
            {
                // Disable charging
                setControlMode(modeClient, "manual", "idle");

                // Execute the command
                BobStatus status = actionClient.findHomeRB(findHomeGoal, findHomeResult);

                // Handle any errors
                printStatus(status);
            }
            else if (command == "find_end")
            {
                // Disable charging
                setControlMode(modeClient, "manual", "idle");

                // Execute the command
                BobStatus status = actionClient.findEndRB(findEndGoal, findEndResult);

                // Handle any errors
                printStatus(status);
            }

            else if (command == "start_charging")
            {
                setChargeMaintenenceState(true);
            }
            else if (command == "charge_current")
            {
                chargeCurrentServer.request.sampleSize = CHARGE_CURRENT_SAMPLE_SIZE;

                // Make the request
                chargeCurrentClient.call(chargeCurrentServer);

                // Handle any errors
                printStatus(chargeCurrentServer.response.status);

                // Print out the current
                ROS_INFO("Charge current: %d", chargeCurrentServer.response.current);
            }
            else if (command == "charge_state")
            {
                ros_robo_bob::GetAbs server;

                // Make the request
                batChargeClient.call(server);

                // Handle any errors
                printStatus(server.response.status);

                // Print out the current
                ROS_INFO("Absolute charge: %d %%", server.response.abs);
            }
            else if (command == "voltage")
            {
                // Make the request
                batVoltageClient.call(voltageServer);

                // Handle any errors
                printStatus(voltageServer.response.status);

                // Print out the voltage
                ROS_INFO("Voltage: %d volts", voltageServer.response.voltage);
            }
            else if (command == "battery_status")
            {
                ros_robo_bob::GetBatteryStatus server;

                // Make the request
                batStatusClient.call(server);

                // Handle any errors
                printStatus(server.response.status);

                // Print out the status
                ROS_INFO("Status: %d", server.response.batteryStatus);
            }
            else if (command == "read_rfid")
            {
                // Make the request
                rfidClient.call(RFIDServer);

                // Handle any errors
                printStatus(RFIDServer.response.status);

                if (RFIDServer.response.status == BOB_SUCCESS)

                    // Print out the tag ID
                    ROS_INFO("RFID code: %X", RFIDServer.response.tagID);
            }
            else if (command == "get_last_tag")
            {
                ros::NodeHandle handle;

                ros_robo_bob::GetLastRFID server;
                ros::ServiceClient rfidClient = handle.serviceClient<ros_robo_bob::GetLastRFID>(addBase2Topic(MFRC522_NODE_NAME, "get_last_tag"));

                // Attempt to read the tag
                rfidClient.call(server);
                BobStatus status = server.response.status;

                // Handle any errors
                printStatus(status);

                // Print out the tag ID
                ROS_INFO("RFID code: %X", server.response.tagID);
            }
            else if (command == "gimbal_power")
            {
                ros::NodeHandle handle;

                if (options.size() != 1)
                {
                    ROS_ERROR("Gimbal power only takes one command (on or off)");
                    continue;
                }
                
                // Get the orientaition data
                std::vector<std::string> state = options;

                // Set the desired gimbal state
                if (options[0] == "on")
                    gimbalPowerServer.request.power = true;
                else
                    gimbalPowerServer.request.power = false;

                gimbalPowerClient.call(gimbalPowerServer);
            }
            else if (command == "capture_image")
            {
                ros_robo_bob::ImageMeta imageMeta;
                captureImageGoal.autoWB = false;

                // Attempt to capture and image
                BobStatus status = actionClient.captureImageRB(captureImageGoal, captureImageResult);
                      
                // Handle any errors
                printStatus(status);

                // Save the image
                processImage(captureImageResult, imageMeta, &plan, 0, {30., 0.,  0.}, 0.);

                // Publish the metadata
                imageMetaPub.publish(imageMeta);
            }
            else if (command == "stream_state")
            {
                 
                 ros::NodeHandle handle;
      
                 ros_robo_bob::StartStopStreaming streamStateServer;
                 ros::ServiceClient streamStateClient = handle.serviceClient<ros_robo_bob::StartStopStreaming>
                                         (addBase2Topic(LUMENARA_NODE_NAME, "start_stop_streaming"));
          
                 // Set the desired streaming state
                 if (options[0] == "on")
                      streamStateServer.request.streaming = true;
                 else
                      streamStateServer.request.streaming = false;
            }
            else if (command == "runplan")
            {
                BobStatus status = setControlMode(modeClient, "manual", options[0]);

                // Handle any errors
                printStatus(status);

                // Print out the plan name if its a success
                if (status == BOB_SUCCESS)
                    ROS_INFO("Running plan %s", options[0].c_str());
            }
            else if (command == "stop")
            {
                // Set the control node to manual mode, and don't specifiy a plan,
                // effectively telling the control node to idle
                setControlMode(modeClient, "manual", "idle");
            }
            else if (command == "auto")
            {
                // Set the control node to auto mode
                setControlMode(modeClient, "auto");
            }
            else if (command == "system_check")
            {
                std::string empty;
                BobStatus status;

                // Set the control node idle mode
                setControlMode(modeClient, "manual", "idle");

                // Instruct the user to set up the robot for the system check
                std::cout << "\nPosition the robot about a foot in front of the charger on the track, then press enter to begin!\n";

                std::cin.ignore();
                
                // Begin charge current test
                chargeCurrentServer.request.sampleSize = CHARGE_CURRENT_SAMPLE_SIZE;
                chargeCurrentClient.call(chargeCurrentServer);

                if(!printTestResult("Read charge current", chargeCurrentServer.response.status))
                    continue;

                // Begin voltage test
                batVoltageClient.call(voltageServer);

                if(!printTestResult("Read voltage", voltageServer.response.status))
                    continue;

                // Begin find home test
                if(!printTestResult("Find home", actionClient.findHomeRB(findHomeGoal, findHomeResult)))
                    continue;

                // Begin find charger test
                if(!printTestResult("Find charger", actionClient.findChargerRB(findChargerGoal, findChargerResult)))
                    continue;

                // Begin read RFID test
                rfidClient.call(RFIDServer);

                if(!printTestResult("Read RFID", RFIDServer.response.status))
                    continue;

                // Begin gimbal test
                moveCameraGoal.orientation = {30., 0.,  0.};

                if(!printTestResult("Move camera", actionClient.moveCameraRB(moveCameraGoal, moveCameraResult)))
                    continue;

                gimbalPowerServer.request.power = false;
                gimbalPowerClient.call(gimbalPowerServer);

                // Begin capture image test
                if(!printTestResult("Capture Image", actionClient.captureImageRB(captureImageGoal, captureImageResult)))
                    continue;

                // Make sure the RFID code exists in the system
                if(!printTestResult("RFID in system", plan.loadTrackFromRFID(RFIDServer.response.tagID)))
                    continue;

                // Begin track sweep test 
                seekGoal.distance = plan.getEndWorking();

                if(!printTestResult("Track sweep A", actionClient.seekRB(seekGoal, seekResult)))
                    continue;

                seekGoal.distance = 100;

                if(!printTestResult("Track sweep B", actionClient.seekRB(seekGoal, seekResult)))
                    continue;
                
                // Return to home
                if(!printTestResult("Find home", actionClient.findHomeRB(findHomeGoal, findHomeResult)))
                    continue;
                
                // Return to charger
                if(!printTestResult("Find charger", actionClient.findChargerRB(findChargerGoal, findChargerResult)))
                    continue;
                
                std::cout << "\n";
                
            }
            else if (command == "help")
            {
                // Compose the help message
                std::string helpMessage =
                    "\n\033[1;31mWords highlighted in\033[0m \033[1;36mblue\033[0m \033[1;31mare placeholders for real values\033[0m\n"
                    "\n\033[1;33mvoltage\033[0m                    print out the battery voltage\n"
                    "\033[1;33mrunplan \033[1;36mplan name\033[0m          runs a plan given its file name\n"
                    "\033[1;33mstop\033[0m                       stops any running plans\n"
                    "\033[1;33mauto\033[0m                       starts the track plan associated with the last read RFID code\n"
                    "\033[1;33mcharge_current\033[0m             print out the charge current\n"
                    "\033[1;33mcharge_state\033[0m               print out the absolute charge state\n"
                    "\033[1;33mcharge_status\033[0m              print out the charge status\n"
                    "\033[1;33mfind_charger\033[0m               attempt to detect the charger by micro-stepping and\n"
                    "                           checking charge current\n"
                    "\033[1;33mfind_home\033[0m                  search for the hall effect sensor in both directions\n"
                    "\033[1;33mfind_end\033[0m                   search for the end of the track\n"
                    "\033[1;33mmove \033[1;36mdistance\033[0m              moves the robot a certain distance (in inches)\n"
                    "\033[1;33mseek \033[1;36mposition\033[0m              moves the robot to a specific position (in inches)\n"
                    "\033[1;33mmove_camera \033[1;36mangle\033[0m          sets the gimbal angle\n"
                    "\033[1;33mread_rfid\033[0m                  attempt to read an RFID tag\n"
                    "\033[1;33mget_last_rfid\033[0m              get the last seen RFID tag\n"
                    "\033[1;33mgimbal_power \033[1;36mon/off\033[0m        sets the gimbal power state to (on/off)\n\n";

                // Print the help message
                //ROS_INFO("%s", helpMessage.c_str());
                std::cout << helpMessage;
            }
            else if (command == "exit")
            {
                ros::shutdown();
            }
            else
                ROS_ERROR("Invalid command");
        }
        catch (...)
        {
            ROS_ERROR("Invalid input");
        }

        // Free the argument memory
        freeArgs(argc, &argv);
    }
}

int main(int argc, char **argv)
{

    // Initialize the
    ros::init(argc, argv, USER_CONTROL_NODE_NAME);

    // The ros handle for this node
    ros::NodeHandle handle;

    // Create a client to modify the mode of the control node
    ros::ServiceClient controlModeClient =
        handle.serviceClient<ros_robo_bob::ControlMode>(addBase2Topic(CONTROL_NODE_NAME, "mode"));

    // Start the control loop
    userControlLoop(controlModeClient, handle);

    return 0;
}
