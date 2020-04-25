#ifndef ROS_ROBO_BOB_TRACK_PLAN_H
#define ROS_ROBO_BOB_TRACK_PLAN_H

/* General c++ includes */
#include <time.h>
#include <vector>
#include <string>
#include <ctime>

/* Package specific includes */
#include "../include/bob_status.hpp"

/* Class definitions */
struct Customer
{
    // Name of the customer
    std::string name;

    // Link to the Edge API
    std::string edgeAPI;
};

struct Location
{
    // Name of the facility
    std::string facility;

    // Name of the building
    std::string building;

    // Name of the space
    std::string space;

    // Used to localize the track within the green house
    int32_t gridOffset[2];

    // The absolute position of the hall effect sensor with respect to the
    // building coordinate system
    float absolutePosition[2];

    // WGS84 data
    double latitude, longitude, heading;
};

struct PowerInfo
{
    // Minimum voltage required for the battery to be at to start a plan
    int minPlanVoltage;

    // Voltage in which the robot must return to the charger
    int stopPlanVoltage;

    // Voltage in which the robot should shut down
    int shutdownVoltage;
};

struct CameraInfo
{
    // The make of the camera
    std::string make;

    // Model of the camera
    std::string model;

    // Focal length of the camera
    float focalLength;
};

class TrackPlan
{

    private:

        // True when a plan has been loaded
        bool loaded;

        // True when the robot should do end detection on the last move
        std::string detectEnd;

        // RFID code belonging to the track
        uint32_t rfidCode;

        // Contains customer information for the plan
        Customer customer;

        // Contains location information for the plans track
        Location location;

        // Contains power constraint information for the plan
        PowerInfo powerInfo;

        // Contains the camera information
        CameraInfo camera;

        // Amount of delay between the technician pressing start and the
        // plan running in ms
        int startDelay;

        // Run interval between each plan in ms
        int runInterval;

        // Vector of times each a plan should start at
        std::vector<int> runTimes;

        // Beginning/end of operating time window in minutes
        int startOpTime, endOpTime;

        // Vector of commands, which are also vectors containing each respective arg
        std::vector<std::vector<std::string> > commands;

        // Map of block commands
        std::map< std::string, std::vector<std::vector<std::string> > > blocks;

        // Indicates the current iteration of the data gathering loop
        int stopIteration;

        // The start and end working distance values of the track relative to the building
        // coordinate system
        float startWorking, endWorking;
    
        // The number of stops in the plan
        int numStops;

        // The % of the track to enable auto-wb for (0 to wbRegion*workingDist)
        float wbRegion;

        // Auto-wb enabled when true
        bool autoWB;

    public:

        TrackPlan();
        void clear();
        bool readyToRun();
        void parseBlocks();
        TrackPlan(std::string);
        void load(std::string);
        void computeRuntimes();
        void loadDefaultBlocks();
        bool isLoaded(){ return loaded; }
        bool getAutoWB(){ return autoWB; }
        BobStatus loadTrackFromRFID(uint32_t);
        int getEndOpTime(){ return endOpTime; }
        int getStartDelay(){ return startDelay; }
        float getEndWorking(){ return endWorking; }
        BobStatus loadTrackByFilename(std::string);
        int getStartOpTime(){ return startOpTime; }
        int getRunInterval(){ return runInterval; }
        Customer * getCustomer(){ return &customer; }
        Location * getLocation(){ return &location; }
        void setAutoWB(bool state){ autoWB = state; }
        CameraInfo * getCameraInfo(){ return &camera; }
        float getStartWorking(){ return startWorking; }
        int getStopIteration(){ return stopIteration; }
        PowerInfo * getPowerInfo(){ return &powerInfo; }
        void setStopIteration(int stopNumber){ stopIteration = stopNumber; }
        std::vector<std::vector<std::string> > getCommands(){ return commands; }
        std::map< std::string, std::vector<std::vector<std::string> > > * getBlocks(){ return &blocks; }

};

#endif
