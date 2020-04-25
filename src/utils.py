import re
import os

# EXIF and XMP tag names
EXIF_DATETIME_ORIGINAL_TAG = "DateTimeOriginal"
EXIF_FOCAL_LENGTH_TAG = "FocalLength"
EXIF_CAMERA_MAKE_TAG = "Make"
EXIF_CAMERA_MODEL_TAG = "Model"
EXIF_GPS_LATITUDE_TAG = "GPSLatitude"
EXIF_GPS_LONGITUDE_TAG = "GPSLongitude"
EXIF_GPS_LATITUDE_REF_TAG = "GPSLatitudeRef"
EXIF_GPS_LONGITUDE_REF_TAG = "GPSLongitudeRef"
XMP_CAMERA_ROLL = "XMP-Camera:Roll"
XMP_CAMERA_PITCH = "XMP-Camera:Pitch"
XMP_CAMERA_YAW = "XMP-Camera:Yaw"
XMP_CAMERA_ROLL_ACCURACY = "XMP-Camera:IMURollAccuracy"
XMP_CAMERA_PITCH_ACCURACY = "XMP-Camera:IMUPitchAccuracy"
XMP_CAMERA_YAW_ACCURACY = "XMP-Camera:IMUYawAccuracy"
EXIF_GPS_LATITUDE_REF_TAG = "GPSLatitudeRef"
EXIF_GPS_LONGITUDE_REF_TAG = "GPSLongitudeRef"
EXIF_CONFIG_FILE_NAME = "/home/catkin_ws/src/ros_robo_bob/config/xmpns.config"

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @path                   Path to the .hpp file                  #
#           @arrayName              Name of the data structure to extract  #
#   Outputs:                                                               #
#           @returns                Contents of the data structure         #
#                                                                          #
#   Description: Extracts a specific 2D list from a .hpp file.             #
#                                                                          #
# *************************************************************************#

def getBobStatusHPPArray(path, arrayName):

    recording = False
    messageFound = False
    arrayContents = []

    # Open the status file
    statusFile = open(path, "r")

    # Load in the status messages line by line
    for line in statusFile:

        # Indicate that we've found the start of the data structure
        if arrayName in line:
            messageFound  = True
            continue

        # Record the message
        if recording:

            element = re.findall(r'"(.*?)"', line)

            if element:
                arrayContents.append(element[0])

        # Toggle recording based on enclosing brackets
        if ("{" in line or "}" in line) and messageFound:
            recording  = not recording 

    statusFile.close()
            
    return arrayContents

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @path                                                          #
#   Outputs:                                                               #
#           @returns           List of sub-directories in the path         #
#                                                                          #
# *************************************************************************#

def getSubDirs(path):

    return [name for name in os.listdir(path) if os.path.isdir(os.path.join(path, name))]

# *************************************************************************#
#                                                                          #
#           @path                                                          #
#   Outputs:                                                               #
#           @returns           List of files in the path                   #
#                                                                          #
# *************************************************************************#

def getFiles(path):

    return [file for file in os.listdir(path) if os.path.isfile(os.path.join(path, file))]

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @tag                    Name of the EXIF or XMP tag to write   #
#           @value                  Value to write                         #
#	    @file                   Path of the file to write to           #
#           @config                 Path to the exiftool config file       #
#                                                                          #
#   Description: Writes an EXIF tag to an image.                           #
#                                                                          #
# *************************************************************************#

def writeEXIFData(data, file, config=None):

    command = "exiftool"
    
    # Add in the config file if provided
    if config:
        command += " -config " + str(config) + "  -overwrite_original"
    
    # Add in all tags and values to the command
    for entry in data:
        command +=  " -" + entry[0] + "=\"" + str(entry[1]) + "\" "

    # Append the file to the command
    command += file
    
    # Run the command
    return os.system(command)

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @file                   Image file to write to                 #
#           @roll                   Camera roll angle                      #
#	    @pitch		    Camera pitch angle                     #
#	    @yaw	            Camera yaw angle		           #
#	    @rollAcc		    Accuracy of the roll angle             #
#	    @pitchAcc               Accuracy of the pitch angle            #
#           @yawAcc                 Accuracy of the yaw angle              #
#	    @focalLength            Camera focal length                    #
#	    @make                   Make of the camera                     #
#           @model                  Model of the camera                    #
#	    @date		    Date the image was taken		   #
#	    @lat	            Latitude of the image		   #
#	    @lon		    Longitude of the image                 #
#                                                                          #
#   Description: Writes an EXIF metadata to an image.	                   #
#                                                                          #
# *************************************************************************#

def writeImageEXIF(file, roll, pitch, yaw, rollAcc, pitchAcc, yawAcc, \
                   focalLength, make, model, date, lat, lon):

    data = []

    # Find the lat/lon refs
    latRef = "N" if lat > 0 else "S"
    lonRef = "E" if lon > 0 else "W"

    # Construct the data payload
    data.append([XMP_CAMERA_ROLL, roll])
    data.append([XMP_CAMERA_PITCH, pitch])
    data.append([XMP_CAMERA_YAW, yaw])
    data.append([XMP_CAMERA_ROLL_ACCURACY , rollAcc])
    data.append([XMP_CAMERA_PITCH_ACCURACY , pitchAcc])
    data.append([XMP_CAMERA_YAW_ACCURACY, yawAcc])
    data.append([EXIF_FOCAL_LENGTH_TAG, focalLength])
    data.append([EXIF_CAMERA_MAKE_TAG, make])
    data.append([EXIF_CAMERA_MODEL_TAG, model])
    data.append([EXIF_DATETIME_ORIGINAL_TAG, date])
    data.append([EXIF_GPS_LATITUDE_TAG, abs(lat)])
    data.append([EXIF_GPS_LONGITUDE_TAG, abs(lon)])
    data.append([EXIF_GPS_LATITUDE_REF_TAG, latRef])
    data.append([EXIF_GPS_LONGITUDE_REF_TAG, lonRef])

    # Write the EXIF data
    writeEXIFData(data, file, EXIF_CONFIG_FILE_NAME)
