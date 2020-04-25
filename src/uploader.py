#!/usr/bin/env python

import os
import cv2
import yaml
import json
import rospy
import rosbag
import socket
import bisect
import pymongo
import rosnode
import collections
import std_msgs.msg
from cloud import *
from utils import *
import ros_robo_bob.msg
from subprocess import call
from std_msgs.msg import UInt8
from ros_robo_bob.srv import *
from cv_bridge import CvBridge
from pymongo import MongoClient
from threading import Thread, Lock
from ros_robo_bob.msg import ImageMeta
from multimaster_msgs_fkie.srv import DiscoverMasters

# Constant definitions
GATEKEEPER_NODE_NAME = "/gatekeeper"
GATEKEEPER_HOST_NAME = "roboserver"
CONTROL_NODE_NAME = "control"
UPLOADER_NODE_NAME = "uploader"
IMAGE_BAGS_PATH = "../../../bob_images"
BOB_SUCCESS = 0
BOB_IMAGE_NOT_FOUND = 38
UPLOADER_POLL_HZ = 1
GATEKEEPER_TIMEOUT_S = 5
ENABLE_MULTIMASTER = False
BOB_NW_CONNECTION_ERROR = 47
MONGO_IMAGE_DB_PORT = 27017
MONGO_IMAGE_DB_NAME = "image_db"
MONGO_IMAGE_DB_COLLECTION = "images"

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @path                                                          #
#   Outputs:                                                               #
#           @returns           List of files in the path                    #
#                                                                          #
# *************************************************************************#

def getFiles(path):

    return [file for file in os.listdir(path) if os.path.isfile(os.path.join(path, file))]

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @image            The image message                            #
#   Outputs:                                                               #
#           @returns          JSON object containing image metadata        #
#                                                                          #
#   Description: Takes the ros_robo_bob Image message, extracts the        #
#                metadata from it, and converts it to a dictionary object  #
#                to be stored as JSON.                                     #
#                                                                          #
# *************************************************************************#

def metadata2Payload(meta):

    # Construct the payload from the image message
    payload = {
       'camdata': {
           'customer': {
               'edgeAPI': meta.edgeAPI,
               'customerName': meta.customerName,
               'facility': meta.facility
           },
           'device': {
               'mount': '',
               'hostname': meta.hostname,
               'lenstype': '',
               'picOptions': '',
               'role': '',
               'rotation': '',
               'camtype': '',
               'make': meta.make,
               'model': meta.model,
               'focalLength': meta.focalLength
           },
           'location': {
               'position': {
                   'x': meta.location[0],
                   'y': meta.location[1]
               },
               'space': meta.space,
               'absolutePosition': {
                   'x': meta.absolutePosition[0],
                   'y': meta.absolutePosition[1]
               },
               'orientation': {
                    'x': meta.orientation[0],
                    'y': meta.orientation[1],
                    'z': meta.orientation[2]
               },
               'coord': {
                   'lat': meta.latitude,
                   'lng': meta.longitude
               }
           }
       },
       'imagedata': {
           'timestamp': meta.timestamp
       }
    }
    
    return payload

#***********************************************************************************#
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @req                  The request from the client                       #
#           @res                  The response object for the client                #
#                                                                                   #
#   Description: Provides a list on images currently on board.                      #
#                                                                                   #
#***********************************************************************************#

def getImageListCB(req):

    imageList = []

    # Loop through each local image and add its name + timestamp to the message
    for entry in getFiles(IMAGE_BAGS_PATH):

        imageList.append(entry)

    return GetImageListResponse(imageList)

#***********************************************************************************#
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @filename             Path to the image bag                             #
#   Outputs:                                                                        #
#           @returns              Parsed image message from a bag                   #
#                                                                                   #
#   Description: Opens an image bag and retrieves the image message from it.        #
#                                                                                   #
#***********************************************************************************#

def imageMessageFromBag(filename):

    try:

        # Create the bag to save the image to
        bag = rosbag.Bag(filename, 'r')

        # Obtain the image message
        for topic, msg, t in bag.read_messages(topics=['image']):
            image = msg

        return image

    except:

        return None

#***********************************************************************************#
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @req                  The request from the client                       #
#           @res                  The response object for the client                #
#                                                                                   #
#   Description: Provides an image bag upon request.                                #
#                                                                                   #
#***********************************************************************************#

def getImageCB(req):

    image = None
    res = GetImageResponse()

    # Construct the filename from the request
    bagFilename = IMAGE_BAGS_PATH+"/"+req.name

    if os.path.exists(bagFilename):

        image = imageMessageFromBag(bagFilename)

        if image:

            # Populate the response
            res.image.data.header       = image.data.header
            res.image.data.height       = image.data.height
            res.image.data.width        = image.data.width
            res.image.data.encoding     = image.data.encoding
            res.image.data.is_bigendian = image.data.is_bigendian
            res.image.data.step         = image.data.step
            res.image.data.data         = image.data.data
            res.image.edgeAPI           = image.edgeAPI
            res.image.customerName      = image.customerName
            res.image.facility          = image.facility
            res.image.hostname          = image.hostname
            res.image.space             = image.space
            res.image.location          = image.location
            res.image.absolutePosition  = image.absolutePosition
            res.image.orientation       = image.orientation
            res.image.latitude          = image.latitude
            res.image.longitude         = image.longitude
            res.image.make              = image.make
            res.image.model             = image.model
            res.image.focalLength       = image.focalLength
            res.image.timestamp         = image.timestamp
            res.status                  = BOB_SUCCESS

        else:

            res.status = BOB_IMAGE_NOT_FOUND
            os.remove(bagFilename)

    else:
        res.status = BOB_IMAGE_NOT_FOUND

    return res

def imageMetaCB(message):

    # Construct the JSON payload
    dbEntry = metadata2Payload(message)

    # Add the local path to the payload
    dbEntry['localPath'] = message.localPath

    # Initialize the Mongo DB client
    imageDBClient = MongoClient("mongodb://localhost:"+str(MONGO_IMAGE_DB_PORT)+"/")
    imageDB = imageDBClient[MONGO_IMAGE_DB_NAME]
    dbCollection = imageDB[MONGO_IMAGE_DB_COLLECTION]

    # Insert the entry
    dbCollection.insert_one(dbEntry)

#***********************************************************************************#
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @msg                  The message from the published topic              #
#                                                                                   #
#   Description: Removes an image from the filesystem upon the gatekeeper letting   #
#                the host know it has succesfully saved it.                         #
#                                                                                   #
#***********************************************************************************#

def imageUploadedCB(message):

    # Obtain the hostname
    hostname = socket.gethostname()

    # Parse the message
    targetHost, imageName = message.data.split(" ")

    bagFilename = IMAGES_PATH+"/"+imageName

    # Erase the image if the message was meant for us
    if hostname == targetHost and os.path.exists(bagFilename):
        os.remove(bagFilename)

    return

#***********************************************************************************#
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#                                                                                   #
#   Description: Uploader worker used to upload images to the cloud. If the         #
#                gatekeeper node isn't present, images and their metadata will be   #
#                uploaded directly from the robot.                                  #
#                                                                                   #
#***********************************************************************************#

def uploader():

    rate = rospy.Rate(UPLOADER_POLL_HZ)

    # Used to convert sensor_msgs/Image to an OpenCV image
    bridge = CvBridge()

    # True when the database has been cleaned
    dbCleaned = False

    # Assume the gatekeeper isn't preset at first if we've enabled it to 
    # avoid uploading images at first
    if ENABLE_MULTIMASTER:
        gatekeeperPresent = True
    else:
        gatekeeperPresent = False

    # Incremented every time the gatekeeper isn't detected
    gatekeeperAbsentTicks = 0

    # Obtain the hostname
    hostname = socket.gethostname()

    # Client for obtaining multimaster node names
    listMastersClient = rospy.ServiceProxy('/'+hostname+'/master_discovery/list_masters', DiscoverMasters)

    # Create a status publisher to let the supervisor know when things go wrong
    statusPub = rospy.Publisher("status", UInt8, queue_size=1)

    # Initialize the Mongo DB client
    imageDBClient = MongoClient("mongodb://localhost:"+str(MONGO_IMAGE_DB_PORT)+"/")
    imageDB = imageDBClient[MONGO_IMAGE_DB_NAME]
    dbCollection = imageDB[MONGO_IMAGE_DB_COLLECTION]

    while not rospy.is_shutdown():

        # Update at a specific rate
        rate.sleep()

        try:
            # Initialize the Mongo DB client
            imageDBClient = MongoClient("mongodb://localhost:"+str(MONGO_IMAGE_DB_PORT)+"/")
            imageDB = imageDBClient[MONGO_IMAGE_DB_NAME]
            dbCollection = imageDB[MONGO_IMAGE_DB_COLLECTION]

            # Obtain an entry from the DB
            entry = dbCollection.find({}).sort('_id', pymongo.ASCENDING)[0]

            # Mark that the database needs cleanup
            dbCleaned = False

        except:

            # Clean the DB if neccesary
            if not dbCleaned:
                
                imageDB.command("repairDatabase")
                dbCleaned = True

            continue

        # Only continue if we've enabled multimaster
        if ENABLE_MULTIMASTER:
            
            try:

                # Get the list of masters on the network
                resp = listMastersClient()

                # See if the gatekeeper is on the network and act accordingly
                if GATEKEEPER_HOST_NAME in [master.name for master in resp.masters]:

                    gatekeeperPresent = True
                    gatekeeperAbsentTicks = 0

                # Increment the counter if the gatekeeper isn't present
                else:
                    gatekeeperAbsentTicks += 1

                # Check and see if the gatekeeper is absent
                if gatekeeperAbsentTicks / UPLOADER_POLL_HZ > GATEKEEPER_TIMEOUT_S:
                    gatekeeperPresent = False

            except:
                pass

        # Upload images if the gatekeeper is absent
        if not gatekeeperPresent and entry:

            # Make sure we have the file
            if not os.path.isfile(entry['localPath']):
                
                try:
                    # Delete the entry from the DB
                    dbCollection.delete_one({'_id':entry['_id']})

                    rospy.logerr("DB entry image path doesn't exist")

                except:
                    pass

                continue

            # Obtain the local path to the image
            localImagePath = entry['localPath']

            # Write the image EXIF metadata
            writeImageEXIF(localImagePath, entry['camdata']['location']['orientation']['x'], 
                           entry['camdata']['location']['orientation']['y'], entry['camdata']['location']['orientation']['z'],
                            1, 1, 1, entry['camdata']['device']['focalLength'], entry['camdata']['device']['make'],
                            entry['camdata']['device']['model'], entry['imagedata']['timestamp'],
                            entry['camdata']['location']['coord']['lat'], entry['camdata']['location']['coord']['lng'])

            # Remove the local path reference from the metadta
            del entry['localPath']

            # Attempt to upload an image and its metadata to the data pipeline
            if upload_image(localImagePath, entry):

                # Delete the image off of the filesystem
                os.remove(localImagePath)

                try:
                    # Delete the entry from the DB
                    dbCollection.delete_one({'_id':entry['_id']})

                except:
                    continue

            else:
                statusPub.publish(BOB_NW_CONNECTION_ERROR)

def main():

    # Initialize the node
    rospy.init_node(UPLOADER_NODE_NAME)

    # Create a service for image list requests from the gatekeeper
    imageListService = rospy.Service(UPLOADER_NODE_NAME+"/get_image_list", GetImageList, getImageListCB)

    # Create a service for image requests from the gatekeeper
    imageService = rospy.Service(UPLOADER_NODE_NAME+"/get_image", GetImage, getImageCB)

    # Create a subscriber to get notified when an image get succesfully uploaded
    planResultSub = rospy.Subscriber(GATEKEEPER_NODE_NAME+"/image_uploaded", std_msgs.msg.String, imageUploadedCB)

    # Create a subscriber to receive image metadata
    imageMetaSub = rospy.Subscriber(CONTROL_NODE_NAME+"/image_meta", ros_robo_bob.msg.ImageMeta, imageMetaCB)

    # Thread that manages uploading
    uploaderThread = Thread(target=uploader)

    # Start the uploader thread
    uploaderThread.start()

    rospy.spin()

if __name__ == '__main__':

    main()
