/* General c++ includes */
#include <cmath>
#include <ctime>
#include <sstream>
#include <cstring>
#include <iterator>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>

/* Package specific includes */
#include "../include/track_plan.hpp"
#include "../include/bob_definitions.hpp"

/* Constant definitions */
#define HOMING_RETURN_DIST 100
#define RUNTIME_WINDOW_MIN 10
#define CHARGE_CPU_FREQ 800000
#define NORMAL_CPU_FREQ 1500000

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Description: Default constructor for the track plan. Loads the defualt blocks   #
#                upon instantiation of the class.                                   #
#                                                                                   #
************************************************************************************/

TrackPlan::TrackPlan()
{
    // Load in the commands
    loadDefaultBlocks();

    // Record that no plan has been loaded
    loaded = false;
    
    // Initialize the stop iteration
    stopIteration = 0;

    //Disable auto-wb initially
    autoWB = false;

    // Initialize grid offset
    location.gridOffset[0] = 0;
    location.gridOffset[1] = 0;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @data                 Vector of strings that define the plan            #
#                                                                                   #
#   Description: Initializes a plan object.                                         #
#                                                                                   #
************************************************************************************/

TrackPlan::TrackPlan(std::string trackFile)
{
    // Load in the commands
    load(trackFile);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Description: Loads blocks that aren't dependent on the parameters of plan       #
#                files.                                                             #
#                                                                                   #
************************************************************************************/

void TrackPlan::loadDefaultBlocks()
{
    /* ------------------------------ Start charge block --------------------------------- */
    
    commands.push_back({"startblock", "charge"});
    commands.push_back({"cpu_scale", std::to_string(CHARGE_CPU_FREQ)});
    commands.push_back({"stream_state", "off"});
    commands.push_back({"gimbal_power", "off"});
    commands.push_back({"find_home", "force"});
    commands.push_back({"find_charger"});
    commands.push_back({"read_rfid"});
    commands.push_back({"start_charging"});
    commands.push_back({"endblock"});

    /* ------------------------------- End charge block ---------------------------------- */

    /* ----------------------------- Start localize block -------------------------------- */

    // Block to run to run upon an uncertain location
    commands.push_back({"startblock", "localize"});
    commands.push_back({"find_home", "force"});
    commands.push_back({"find_charger"});
    commands.push_back({"read_rfid"});
    commands.push_back({"endblock"});

    /* ------------------------------ End localize block --------------------------------- */

    // Parse the blocks
    parseBlocks(); 
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @filename               Filename of the yaml plan                       #
#                                                                                   #
#   Description: Parses a .yaml plan file into a plan class. The plan is parsed     #
#                                                                                   #
************************************************************************************/

void TrackPlan::load(std::string trackFile)
{
    int hour, minute;
    int gimbalPos = 0;
    YAML::Node files[2];
    YAML::Node gimbalOrientations;
    std::string startTime, endTime;
    float photoInterval, homingFactor;

    // Find the parent directory of the trackfile
    std::string trackFileDir =  trackFile.substr(0, trackFile.find_last_of("\\/"));

    // Load the track file into a YAML node
    files[0] = YAML::LoadFile(trackFile);

    // Load in Edge API data
    customer.edgeAPI = files[0]["customer"]["edgeAPI"].as<std::string>();

    // Load in grid data
    location.gridOffset[0] = files[0]["location"]["gridOffset"]["x"].as<int>();
    location.gridOffset[1] = files[0]["location"]["gridOffset"]["y"].as<int>();

    // Load in WGS84 Data
    location.latitude = files[0]["location"]["wgs84Data"]["latitude"].as<double>();
    location.longitude = files[0]["location"]["wgs84Data"]["longitude"].as<double>();
    location.heading = files[0]["location"]["wgs84Data"]["heading"].as<double>();

    // Load in absolute position data
    location.absolutePosition[0] = files[0]["location"]["absolutePosition"]["x"].as<int>();
    location.absolutePosition[1] = files[0]["location"]["absolutePosition"]["y"].as<int>();

    // Obtain high-level location data from the folder structure
    boost::filesystem::path spacePath = boost::filesystem::path(trackFile).parent_path();
    boost::filesystem::path buildingPath = boost::filesystem::path(spacePath).parent_path();
    boost::filesystem::path facilityPath = boost::filesystem::path(buildingPath).parent_path().parent_path();
    boost::filesystem::path customerPath = boost::filesystem::path(facilityPath).parent_path();

    // Parse the location data into strings
    location.facility = facilityPath.filename().string();
    location.building = buildingPath.filename().string();
    location.space = spacePath.filename().string();
    customer.name = customerPath.filename().string();

    // Load in attributes
    rfidCode = files[0]["attributes"]["rfidCode"].as<uint32_t>();
 
    // Load the runtime file into a YAML node
    std::string runtimeFile = files[0]["runtimeFile"].as<std::string>();
    files[1] = YAML::LoadFile(trackFileDir+"/../../../runtime/"+runtimeFile);

    // Load in runtime config parameters w/ overrides from the track file
    startDelay = files[files[0]["runtimeConfig"]["startDelay"] ? 0 : 1]["runtimeConfig"]["startDelay"].as<int>();
    runInterval = files[files[0]["runtimeConfig"]["runInterval"] ? 0 : 1]["runtimeConfig"]["runInterval"].as<int>();
    detectEnd = files[files[0]["runtimeConfig"]["detectEnd"] ? 0 : 1]["runtimeConfig"]["detectEnd"].as<std::string>();
    gimbalOrientations = files[files[0]["runtimeConfig"]["gimbalOrientations"] ? 0 : 1]["runtimeConfig"]["gimbalOrientations"];
    startTime = files[files[0]["runtimeConfig"]["operatingWindow"] ? 0 : 1]["runtimeConfig"]["operatingWindow"]["start"].as<std::string>();
    endTime = files[files[0]["runtimeConfig"]["operatingWindow"] ? 0 : 1]["runtimeConfig"]["operatingWindow"]["end"].as<std::string>();
    photoInterval = files[files[0]["runtimeConfig"]["photoInterval"] ? 0 : 1]["runtimeConfig"]["photoInterval"].as<float>();
    homingFactor = files[files[0]["runtimeConfig"]["homingFactor"] ? 0 : 1]["runtimeConfig"]["homingFactor"].as<float>();
    startWorking = files[files[0]["runtimeConfig"]["startWorking"] ? 0 : 1]["runtimeConfig"]["startWorking"].as<float>();
    endWorking = files[files[0]["runtimeConfig"]["endWorking"] ? 0 : 1]["runtimeConfig"]["endWorking"].as<float>();
    powerInfo.minPlanVoltage = files[files[0]["runtimeConfig"]["minPlanVoltage"] ? 0 : 1]["runtimeConfig"]["minPlanVoltage"].as<int>();
    powerInfo.stopPlanVoltage = files[files[0]["runtimeConfig"]["stopPlanVoltage"] ? 0 : 1]["runtimeConfig"]["stopPlanVoltage"].as<int>();
    powerInfo.shutdownVoltage = files[files[0]["runtimeConfig"]["shutdownVoltage"] ? 0 : 1]["runtimeConfig"]["shutdownVoltage"].as<int>();
    wbRegion = files[files[0]["runtimeConfig"]["wbRegion"] ? 0 : 1]["runtimeConfig"]["wbRegion"].as<float>();
    camera.make = files[files[0]["runtimeConfig"]["cameraMake"] ? 0 : 1]["runtimeConfig"]["cameraMake"].as<std::string>();
    camera.model = files[files[0]["runtimeConfig"]["cameraModel"] ? 0 : 1]["runtimeConfig"]["cameraModel"].as<std::string>();
    camera.focalLength = files[files[0]["runtimeConfig"]["cameraFocalLengthMM"] ? 0 : 1]["runtimeConfig"]["cameraFocalLengthMM"].as<float>();

    // Parse the operating start time
    sscanf(startTime.c_str(), "%d:%d", &hour, &minute);
    startOpTime = hour * 60 + minute;

    // Parse the operating end time
    sscanf(endTime.c_str(), "%d:%d", &hour, &minute);
    endOpTime = hour * 60 + minute;

    // Compute the number of stops RoboBOB will need to make
    float workLength = endWorking-startWorking;
    int stops = (int)(workLength/photoInterval)+1;

    /* ---------------------------- Start data_capture block ------------------------------- */

    commands.push_back({"startblock", "data_capture"});

    // Add each desired gimbal orientation into the loop
    for(YAML::const_iterator it=gimbalOrientations.begin();it != gimbalOrientations.end();++it)
    {
        YAML::Node position = *it;

        // Extract the gimbal positions
        float x = position[0]["x"].as<float>();
        float y = position[1]["y"].as<float>();
        float z = position[2]["z"].as<float>();
        
        // Add in the imaging commands
        commands.push_back({"move_camera", std::to_string(x), std::to_string(y), std::to_string(z)});
        commands.push_back({"capture_image", std::to_string(4-gimbalPos++)});
    }

    // End of our work block, loop it a specific number of times
    commands.push_back({"endblock"});

    /* ----------------------------- End data_capture block -------------------------------- */

    /* -------------------------------- Start plan block ----------------------------------- */

    commands.push_back({"startblock", "plan"});

    // Return the CPU frequency to normal operating mode
    commands.push_back({"cpu_scale", std::to_string(NORMAL_CPU_FREQ)});

    // Start the image streaming
    commands.push_back({"stream_state", "on"});
    
    // Find home if unknown
    commands.push_back({"find_home"});

    // Stop charging and move to to start of work area
    commands.push_back({"stop_charging"});
    commands.push_back({"seek", std::to_string(startWorking)});

    // Add stops one-by-one
    for(int i = 0; i < stops; i++)
    {
        // Do a seek move, or an end detection if it is enabled and we're at the end
        if(detectEnd == "on" && i == stops - 1)
            commands.push_back({"find_end", "retry", "3", "move", "-10"});
        else
            commands.push_back({"seek", std::to_string((i*(int)photoInterval)+(int)startWorking), "retry", "3", "move", "-10"});

        commands.push_back({"set_stop_number", std::to_string(stops-i-1)});

        // Turn on auto wb if we're in the wb region
        if(i < (int)(((float)stops)*wbRegion))
	    commands.push_back({"auto_wb", "on"});
        else
            commands.push_back({"auto_wb", "off"});        

        commands.push_back({"data_capture", "retry", "3", "gimbal_power", "off"});
    }

    // Start the image streaming
    commands.push_back({"stream_state", "off"});
    
    // Go back close to home
    commands.push_back({"seek", std::to_string(HOMING_RETURN_DIST)});

    // Run the charge block with retries, executing the retry block
    commands.push_back({"charge", "hard_retry", "3"});

    // End of the plan block
    commands.push_back({"endblock"});

    /* --------------------------------- End plan block ------------------------------------ */

    // Compute the runtimes for the plan
    computeRuntimes();

    // Parse the blocks
    parseBlocks();

    // Record that the plan has been loaded
    loaded = true;

}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @tagCode                The code of the RFID tag to load the plan for   #
#           @trackPath              String containing the path to the track file    #
#   Outputs:                                                                        #
#           @modifies               trackPath variable with the filename of the     #
#                                   track file corresponding to the RFID code       #
#                                                                                   #
#   Description: Searches all possible track files for a matching RFID tag and      #
#                returns the path if found.                                         #
#                                                                                   #
************************************************************************************/

BobStatus TrackPlan::loadTrackFromRFID(uint32_t tagCode)
{
    char currentDir[256];

    // Obtain the current working directory
    getcwd(currentDir, sizeof(currentDir));

    // Find the data folder based on the current working directory
    boost::filesystem::path dataFolder = std::string(PLAN_REPO_PATH);

    // Create an iterator to recursively go through the data
    boost::filesystem::recursive_directory_iterator it{dataFolder};

    // Find all of the track 
    while (it != boost::filesystem::recursive_directory_iterator{})
    {
        std::string path = (*it++).path().string();
        
        // Obtain the RFID code if this is a track file
        if(path.find("track") != std::string::npos)
        {
            // Load the file
            YAML::Node file = YAML::LoadFile(path);

            // Get the RFID code
            if(tagCode == file["attributes"]["rfidCode"].as<uint32_t>())
            {
                load(path);
                return BOB_SUCCESS;
            }
                
        }

    }

    return BOB_TRACK_DOESNT_EXIST;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @tagCode                The code of the RFID tag to load the plan for   #
#                                                                                   #
#   Description: Searches all possible track files for a matching RFID tag and      #
#                returns the path if found.                                         #
#                                                                                   #
************************************************************************************/

BobStatus TrackPlan::loadTrackByFilename(std::string filename)
{
    char currentDir[256];

    std::string trackPath = "";
    
    // Obtain the current working directory
    getcwd(currentDir, sizeof(currentDir));

    // Find the data folder based on the current working directory
    boost::filesystem::path dataFolder = std::string(currentDir) + "/" + PLAN_REPO_PATH;

    // Create an iterator to recursively go through the data
    boost::filesystem::recursive_directory_iterator it{dataFolder};    

    while (it != boost::filesystem::recursive_directory_iterator{})
    {
        std::string path = (*it++).path().string();
        
        if(path.find(filename) != std::string::npos)
            trackPath = path;
    }

    // Load the track file if it exists
    if(trackPath != "")
        load(trackPath);
    else
        return BOB_TRACK_DOESNT_EXIST;

    return BOB_SUCCESS;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#                                                                                   #
#   Description: Computes the run times using the operating hours window and run    #
#                interval.                                                          #
#                                                                                   #
************************************************************************************/

void TrackPlan::computeRuntimes()
{
    int runs;

    // Compute the start/end and interval times in minutes
    int intervalMins = (int)(((float)runInterval / 1000.)/60.);

    // Compute the number of runs we can fit in an operating window
    if(startOpTime < endOpTime)
        runs = std::floor((endOpTime-startOpTime)/intervalMins);
    else
        runs = std::floor(((1440-startOpTime)+endOpTime)/intervalMins);

    // Populate the runtimes list
    for(int i = 0; i < runs; i++)
        runTimes.push_back((intervalMins*i+startOpTime)%1440);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Outputs:                                                                        #
#           @returns              Bool indicating whether the current time matches  #
#                                 a scheduled one                                   #
#                                                                                   #
#   Description: Returns true if the current time exists within the plan schedule.  #
#                                                                                   #
************************************************************************************/

bool TrackPlan::readyToRun()
{
    time_t rawTime;
    struct std::tm * gmt;

    // Get the raw time
    time(&rawTime);

    // Convert to gmt time in minutes
    gmt = gmtime( &rawTime );
    int gmt_min_time = gmt->tm_hour * 60 + gmt->tm_min;
    
    // See if the current time matches any of the scheduled runtimes
    for(int i = 0; i < runTimes.size(); i++)
        if(gmt_min_time >= runTimes[i] && gmt_min_time < RUNTIME_WINDOW_MIN + runTimes[i]) 
            return true;

    return false;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @cmds                 Commands to parse                                 #
#           @blocks               Map of blocks to be populated                     #
#   Outputs:                                                                        #
#           @modifies             Blocks by reference                               #
#                                                                                   #
#   Description: Generates key(block name)-value(commands) pairs in the form of     #
#                an std map given a vector of commands.                             #                      
#                                                                                   #
************************************************************************************/

void TrackPlan::parseBlocks()
{
    bool blockSearch = false;
    std::string blockName = "";
    std::vector<std::vector<std::string> > blockCmds;

    // Start parsing commands
    for (int i = 0; i < commands.size(); i++)
    {
        if (commands[i][0] == "startblock")
        {
            // Only add the block if it doesn't exist
            if(!blocks.count(commands[i][1]))
            {
                // Save the name of the block for later
                blockName = commands[i][1];

                // Start recording commands
                blockSearch = true;
            }
        }
        else if (commands[i][0] == "endblock" && blockSearch)
        {
            // Add the finished block to the map
            blocks.insert(std::make_pair(blockName, blockCmds));

            // End recording commands
            blockSearch = false;
            
            // Clean the commands up for the next block
            blockCmds.clear();
        }
        else if (blockSearch)

            // Add the command to the current block
            blockCmds.push_back(commands[i]);
    }
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Description: Clears all of the objects contained in a plan.                     #                      
#                                                                                   #
************************************************************************************/

void TrackPlan::clear()
{
    // Erase runtimes
    runTimes.clear();

    // Erase all commands
    for(int i = 0; i < commands.size(); i++)
        commands[i].clear();

    commands.clear();

    // Erase all blocks
    blocks.clear();

    // Re-load in the commands
    loadDefaultBlocks();

    // Record that a plan is no longer loaded
    loaded = false;
}
