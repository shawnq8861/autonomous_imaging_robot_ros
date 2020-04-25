/* General c++ includes */
#include <ctime>
#include <mutex>
#include <vector>
#include <string>
#include <math.h>	
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/file.h>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

/* Package specific includes */
#include "../include/utils.hpp"
#include "../include/bob_status.hpp"

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @base                The base character sequence to add to the topic    #
#           @topic               The topic to append the base to                    #
#   Outputs:                                                                        #
#           @returns             Proper ros topic with base appended                #
#                                                                                   #
#   Description: Adds a base string to a ROS topic                                  #
#                                                                                   #
************************************************************************************/

std::string addBase2Topic(const char *base, const char *topic)
{
    return (std::string(base) + "/" + std::string(topic));
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Outputs:                                                                        #
#           @returns             String containing the date and time                #
#                                                                                   #
#   Description: Creates a string representing the current UTC date and time.       #
#                                                                                   #
************************************************************************************/

std::string date2String()
{
    time_t rawTime;
    struct std::tm *gmt;
    char dateBuffer[256];

    // Get the raw time
    time(&rawTime);

    return std::to_string(rawTime);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @epochTime          String representing the epoch time in seconds       #
#   Outputs:                                                                        #
#           @returns             String containing the date and time                #
#                                                                                   #
#   Description: Converts an epoch time string to ISO time.                         #
#                                                                                   #
************************************************************************************/

std::string epoch2ISOTime(std::string epochTimeString)
{

    char buff[64];

    // Convert the epoch time string to an int
    time_t epochTime = (time_t)std::stoi(epochTimeString);

    // Create the ISO date string
    strftime(buff, sizeof buff, "%FT%T.000Z", gmtime(&epochTime));

    return std::string(buff);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#                                                                                   #
#   Description: Creates the working data directory folder if it doesn't exist      #
#                already.                                                           #
#                                                                                   #
************************************************************************************/

BobStatus createDataDir()
{
    // Create the persistent data directory if it doesn't exist already
    const int status = system("mkdir -p data/");
    if (status == -1)
        return BOB_INCORRECT_FILE_PERMISSIONS;

    return BOB_SUCCESS;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#                                                                                   #
#   Description: Creates the working data directory folder if it doesn't exist      #
#                already.                                                           #
#                                                                                   #
************************************************************************************/

BobStatus createDir(std::string name)
{
    // Create the persistent data directory if it doesn't exist already
    const int status = system((std::string("mkdir -p ") + name).c_str());
    if (status == -1)
        return BOB_INCORRECT_FILE_PERMISSIONS;

    return BOB_SUCCESS;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @average             The current moving average value                   #
#           @data                The new data to add to the moving average          #
#           @decayFactor         The influence the new data has on the average      #
#                                                                                   #
#   Outputs:                                                                        #
#           @modifies            The average value by reference                     #
#                                                                                   #
#   Description: Performs a moving average with a decay factor. The lower the       #
#                factor, the more influence the new value will have on the          #
#                moving average.                                                    #
#                                                                                   #
************************************************************************************/

void movingAverage(float &average, float data, float decayFactor)
{
    if (average == 0.)
        average = data;
    else
        average = (average * decayFactor) + ((1. - decayFactor) * data);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @filename            Name of the file to compute a checksum for         #
#                                                                                   #
#   Description: Computes a checksum for a file, and saves it with the files name   #
#                with a ".checksum" attached to the end.                            #
#                                                                                   #
************************************************************************************/

void writeChecksumFile(std::string filename)
{
    int checksum = 0;

    // Open the file
    std::ifstream file(filename);

    // Make sure the file is open
    if (!file.is_open())
        return;

    // Put the bytes into a vector
    std::vector<char> fileBytes((std::istreambuf_iterator<char>(file)),
                                (std::istreambuf_iterator<char>()));

    // Obtain a checksum by XORing
    for (int i = 0; i < fileBytes.size(); i++)
        checksum ^= fileBytes[i];

    file.close();

    // Create the checksum file
    std::fstream checksumFile(filename + ".checksum", std::fstream::trunc | std::fstream::out);

    // Write the checksum
    checksumFile << checksum << "\n";
    checksumFile.close();
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @filename            Name of the file to check                          #
#   Outputs:                                                                        #
#           @returns             True if the checksum matched, false otherwise      #
#                                                                                   #
#   Description: Reads in a checksum file based on the filename and makes sure the  #
#                contents of the file match the checksum in the file.               #
#                                                                                   #
************************************************************************************/

bool checkFileIntegrity(std::string filename)
{
    struct stat buffer;
    int actualChecksum = 0;
    int expectedChecksum = 0;

    // Only continue if the file exists
    if (stat(filename.c_str(), &buffer) == -1)
        return true;

    // Open the checksum file
    std::ifstream checksumFile(filename + ".checksum");

    // Make sure the file is open
    if (!checksumFile.is_open())
        return false;

    // Read in the checksum
    checksumFile >> expectedChecksum;
    checksumFile.close();

    // Open the file
    std::ifstream file(filename);

    // Put the bytes into a vector
    std::vector<char> fileBytes((std::istreambuf_iterator<char>(file)),
                                (std::istreambuf_iterator<char>()));

    // Obtain a checksum by XORing
    for (int i = 0; i < fileBytes.size(); i++)
        actualChecksum ^= fileBytes[i];

    file.close();

    return (actualChecksum == expectedChecksum);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @lat1		 Start latitude, in degrees                         #
#	    @lon1                Start longitude, in degrees                        #
#           @lat2		 Final latitude, in degrees                         #
#           @lon2		 Final longitude, in degrees                        #
#	    @bearing             Bearing to apply the movement in, in degrees       #
#           @distance            Distance to move from the start point, in meters   #
#   Outputs:                                                                        #
#           @modifes             lat2 by reference                                  #
#           @modifies            lon2 by reference                                  #
#                                                                                   #
#   Description: Computes a lat/lon pair from an origin pair with a desired         #
#                bearing and movement distance.                                     #
#                                                                                   #
************************************************************************************/

void transformWGS84(double lat1, double lon1, double & lat2, double & lon2, double bearing, double distance)
{
    // Radius of the earth in meters
    const double R = 6371000;

    // Precompute the distance to earth's radius ratio
    const double dr = distance / R;

    // Convert lat/lon and bearing to radians
    lat1 *= (M_PI/180.);
    lon1 *= (M_PI/180.);
    bearing *= (M_PI/180);

    // Compute the new latitude and longitude
    lat2 = asin(sin(lat1)*cos(dr)+cos(lat1)*sin(dr)*cos(bearing));
    lon2 = lon1 + atan2(sin(bearing)*sin(dr)*cos(lat1), cos(dr)-sin(lat1)*sin(lat2));

    // Convert the new lat/lon back to degrees
    lat2 *= (180./M_PI);
    lon2 *= (180./M_PI);
}
