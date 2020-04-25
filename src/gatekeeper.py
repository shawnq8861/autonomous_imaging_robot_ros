#!/usr/bin/env python

import os
import cv2
import yaml
import rospy
import rosbag
import socket
import bisect
import rosnode
import collections
import std_msgs.msg
from cloud import *
import ros_robo_bob.msg
from subprocess import call
from ros_robo_bob.srv import *
from cv_bridge import CvBridge
from pymongo import MongoClient
from threading import Thread, Lock
from utils import getFiles, getSubDirs
from uploader import imageMessage2Metadata
from multimaster_msgs_fkie.srv import DiscoverMasters

# Constant definitions
GATEKEEPER_NODE_NAME = "gatekeeper"
CONTROL_NODE_NAME = "control"
UPLOADER_NODE_NAME = "uploader"
MFRC522_NODE_NAME = "mfrc522"
MULTIMASTER_START_TIMEOUT_S = 10
HOST_QUERY_RETRIES = 3
HOST_QUERY_RETRY_EPOCH_S = 3.
HOST_DISCOVERY_HZ = .2
GATE_WORKER_NAME = "gateworker"
GATE_WORKER_IMG_RETRY_TIME_S = 10
IMAGE_PRIORITY_METHOD = "roundrobin"
IMAGE_STORAGE_DIR = "../../../data/bob_images"
MONGO_IMAGE_DB_PORT = 27017
MONGO_IMAGE_DB_NAME = "gatekeeper_image_db"
MONGO_IMAGE_DB_COLLECTION = "images"
IMAGE_FILE_TYPE = ".jpg"
UPLOADER_THREADS = 2
UPLOADER_IMAGES_PER_THREAD = 8
UPLOADER_POLL_HZ = 5
PLAN_REPO_DIR = "/home/robo_bob_plans"

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @hostname           Name of the host                           #
#   Outputs:                                                               #
#           @returns            Dictionary object containing all required  #
#                               service clients/subscribers/etc.           #
#                                                                          #
#   Description: This function creates a 'pack' of ROS service clients/    #
#                subsribers/etc. for a namespaced host.                    #
#                                                                          #
# *************************************************************************#

def hostPack(hostname):

    pack = {}

    # Services
    pack['get_image'] = rospy.ServiceProxy(hostname+'/'+UPLOADER_NODE_NAME+'/get_image', GetImage)
    pack['get_image_list'] = rospy.ServiceProxy(hostname+'/'+UPLOADER_NODE_NAME+'/get_image_list', GetImageList)
    pack['get_last_tag'] = rospy.ServiceProxy(hostname+'/'+MFRC522_NODE_NAME+'/get_last_tag', GetLastRFID)

    # Publishers
    pack['image_uploaded'] = rospy.Publisher(GATEKEEPER_NODE_NAME+'/image_uploaded', std_msgs.msg.String, queue_size=1)

    return pack

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @packs             Host packs                                  #
#           @host              Name of the host to query                   #
#   Outputs:                                                               #
#           @modifies          The packs object                            #
#                                                                          #
#   Description: This function deletes a pack from the packs dictionary    #
#                based on the hostname.                                    #
#                                                                          #
# *************************************************************************#

def deleteHostPack(packs, host):

    # Close the service proxys
    packs[host]['get_image'].close()
    packs[host]['get_image_list'].close()
    packs[host]['get_last_tag'].close()

    # Close the publishers
    packs[host]['image_uploaded'].unregister()

    # Delete the pack
    del packs[host]

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @result            The result data from the subscribed topic   #
#                                                                          #
#   Description: This callback is iniated when the control node has        #
#                indicated that a plan run has completed. The result       #
#                will have a status code associated with it, indicating    #
#                whether or not any problems have occured.                 #
#                                                                          #
# *************************************************************************#

def planResultCallback(result):

    #
    #   Result handling code goes here
    #

    rospy.loginfo("Plan finished with status code: " + str(result.data))

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Description: Waits for the master_discovery and master_sync nodes      #
#                to become active, with a timeout period.                  #
#                                                                          #
# *************************************************************************#

def waitForMultimaster():

    tries = 0

    while tries < MULTIMASTER_START_TIMEOUT_S:

        # Obtain the names of currently running nodes
        nodeNames = rosnode.get_node_names()

        # Check to see if the multimaster nodes have started
        if ("/master_sync" in nodeNames) and ("/master_discovery" in nodeNames):
            return

        tries += 1
        rospy.sleep(1)

    # Should never reach here, this indicates the multimaster nodes couldn't start up
    rospy.logerr("Couldn't start multimaster nodes, exiting")
    exit(1)

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @packs             Host packs                                  #
#           @host              Name of the host to query                   #
#           @query             The query to send to the host               #
#           @args              The arguments to send with the query        #
#   Outputs:                                                               #
#           @returns           Data from the query                         #
#                                                                          #
#   Description: Given a hostname, query, and optional arguments, this     #
#                function will make a query over the multimaster network   #
#                and return the response data. The query will be called    #
#                until it succeeds or until the retry limit is met.        #
#                                                                          #
# *************************************************************************#

def queryHost(packs, host, query, args=None):

    data = None
    retries = 0

    # Attempt to query the data from the host
    while retries < HOST_QUERY_RETRIES:

        try:
            if args is None:
                data = packs[host][query]()
            else:
                data = packs[host][query](args)

            return data
        except:
            retries += 1
            rospy.sleep(HOST_QUERY_RETRY_EPOCH_S)

    if retries == HOST_QUERY_RETRIES:
        rospy.logerr("Couldn't complete "+query+" query")

    return data

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @packs             Host packs                                  #
#           @host              Name of the host to query                   #
#           @imageName         Name of the image to download               #
#   Outputs:                                                               #
#           @returns           A ros_robo_bob/Image object                 #
#                                                                          #
#   Description: Attempts to upload and save image from a specific host,   #
#                and additionally lets the host know it has been           #
#                uploaded.                                                 #
#                                                                          #
# *************************************************************************#

def getImage(packs, host, imageName):

    rospy.loginfo("Downloading " + imageName + " from " + host)

    # Query the host for the image
    image = queryHost(packs, host, 'get_image', imageName)

    # Make sure the host still exists, and that the image query was successful
    if host in packs and image:

        # Let the host know we've gotten the image
        packs[host]['image_uploaded'].publish(host+" "+imageName)

    return image

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @packs             Host packs                                  #
#           @map               Map object to be updated                    #
#   Outputs:                                                               #
#           @modifies          Updates host list by reference              #
#                                                                          #
#   Description: Thread that polls the network for available hosts at a    #
#                speific rate and updates the host packs accordingly.      #
#                                                                          #
# *************************************************************************#

def hostDiscovery(packs, map):

    listMastersClient = rospy.ServiceProxy('/master_discovery/list_masters', DiscoverMasters)

    # Look for new hosts at a certain rate
    rate = rospy.Rate(HOST_DISCOVERY_HZ)

    gatekeeperHostname = socket.gethostname()

    while not rospy.is_shutdown():

        hosts = []

        try:
            # Get the list of masters on the network
            resp = listMastersClient()

            # Create a pack for new hosts
            for master in resp.masters:

                    hosts.append(master.name)

                    # Add the host if it doesn't exist already and isn't the gatekeeper
                    if master.name not in packs and master.name != gatekeeperHostname:

                        packs[master.name] = hostPack(master.name)
                        rospy.loginfo("Adding host " + str(master.name))

            # Remove any stale hosts
            for host in packs:
                if host not in hosts:

                    rospy.loginfo("Removing host " + str(host))

                    # Delete all items in the pack
                    deleteHostPack(packs, host)

                    # Remove the host from the map
                    removeHostFromMap(host, map)

            # Update the facility map
            updateFacilityMap(packs, map)

        except:
            continue

        rate.sleep()

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @customer          Name of the customer                        #
#           @facility          Name of the facility                        #
#           @map               Map object to be populated                  #
#   Outputs:                                                               #
#           @modifies          Updates map by reference                    #
#                                                                          #
#   Description: Given a customer and facility, this function traverses    #
#                the data folder structure and creates a map dictionary    #
#                with the hierarch map[building][space][track#], and also  #
#                stores the RFID of the track for later lookup.            #
#                                                                          #
# *************************************************************************#

def genFacilityMap(customer, facility, map):

    buildingDir = PLAN_REPO_DIR+"/"+customer+"/"+facility+"/buildings"

    # Make sure the directory exists
    if not os.path.isdir(buildingDir):
        rospy.logerr("The specified facility or customer doesn't exist")
        exit(1)

    # Loop through each building
    for building in getSubDirs(buildingDir):

        map[building] = {}

        # Loop through each space in the building
        for space in getSubDirs(buildingDir+"/"+building):
            map[building][space] = {}

            spaceDir = buildingDir+"/"+building+"/"+space

            # Iterate through each track file in the space
            for file in getFiles(spaceDir):

                # Load the track file
                stream = open(spaceDir+"/"+file,"r")
                trackFile = yaml.load(stream)

                # Record the RFID code
                map[building][space][trackFile['attributes']['rfidCode']] = None
                stream.close()

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @packs             Host packs                                  #
#           @map               Map object to be populated                  #
#   Outputs:                                                               #
#           @modifies          Updates map by reference                    #
#                                                                          #
#   Description: Polls the hosts for their last known RFID tags and        #
#                updates the map accodingly with hostnames.                #
#                                                                          #
# *************************************************************************#

def updateFacilityMap(packs, map):

    # Get an RFID value from each host
    for host in packs:

        try:
            res = packs[host]['get_last_tag']()
            addHost2Map(host, res.tagID, map)
        except:
            continue

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @host              Name of th host to add to the map           #
#           @rfid              RFID code of the track the host is on       #
#           @map               Map object to be populated                  #
#   Outputs:                                                               #
#           @modifies          Updates map by reference                    #
#                                                                          #
#   Description: Searches the map for a matching RFID, and updates the     #
#                map if it has found a match.                              #
#                                                                          #
# *************************************************************************#

def addHost2Map(host, rfid, map):

    foundInMap = False

    # Search for the hosts RFID in the map, and update the location
    for building in map:
            for space in map[building]:
                for track in map[building][space]:
                    if int(track, 16) == rfid:
                        foundInMap = True
                        map[building][space][track] = host

    return foundInMap

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @host              Name of th host to add to the map           #
#           @map               Map object to be populated                  #
#   Outputs:                                                               #
#           @modifies          Updates map by reference                    #
#                                                                          #
#   Description: Searches the map for a hostname and removes it from the   #
#                map.                                                      #
#                                                                          #
# *************************************************************************#

def removeHostFromMap(host, map):

    # Search for the host through the map and delete it
    for building in map:
        for space in map[building]:
            for track in map[building][space]:
                if map[building][space][track] == host:
                    map[building][space][track] = None

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @hosts             List of hosts to be updated                 #
#           @mapElement        The map element to traverse into for hosts  #
#   Outputs:                                                               #
#           @modifies          Updates hosts by reference                  #
#                                                                          #
#   Description: Recursively searches a map element for all hosts present  #
#                                                                          #
# *************************************************************************#

def findChildHosts(hosts, mapElement):

    if mapElement:
        for element in mapElement:

            if type(mapElement[element]) is str:

                hosts.append(mapElement[element])

            else:
                findChildHosts(hosts, mapElement[element])

    return

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @elementName       Name of the element to search for           #
#           @map               Map object to search through                #
#   Outputs:                                                               #
#           @returns           Element object                              #
#                                                                          #
#   Description: Recursively searches the map for a specific element,      #
#                then returns the element object.                          #
#                                                                          #
# *************************************************************************#

def findElementInMap(elementName, map):

    for element in map:

        # Return none if we've reached the map leaves
        if type(map[element]) is not dict:
            return None

        # Return the element if we've found it
        elif element == elementName:
            return map[element]

        # Recursively search if we're not at the map leaves
        else:
            return findElementInMap(elementName, map[element])

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @groupFilename   The group file to read from                   #
#           @map             Map object to search through                  #
#                                                                          #
#   Outputs:                                                               #
#           @returns         List of hosts in the group                    #
#                                                                          #
#   Description: Given a group file, this function populate a host list    #
#                off all the hosts that belong to the group. Groups can    #
#                consist of buildings, spaces, facilities, or hosts.       #
#                                                                          #
# *************************************************************************#

def findHostsInGroup(groupFilename, map):

    hosts = []
    groupFile = open(groupFilename, "r")

    for line in groupFile:

        element = findElementInMap(line, map)
        findChildHosts(hosts, element)

    groupFile.close()

    return hosts

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @customer          Name of the customer                        #
#           @facility          Name of the facility                        #
#   Outputs:                                                               #
#           @returns           List of gateworker group filenames          #
#                                                                          #
#   Description: Retrieves a list of filenames for all gateworker groups   #
#                defined for the facility.                                 #
#                                                                          #
# *************************************************************************#

def getFacilityWorkerGroups(customer, facility):

    groupFiles = []

    groupsDir = PLAN_REPO_DIR+"/"+customer+"/"+facility+"/groups"

    # Scan the groups folder for 'gateworker' groups
    for file in getFiles(groupsDir):
        if GATE_WORKER_NAME in file:
            groupFiles.append(groupsDir+"/"+file)

    return groupFiles

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @groupHosts       List of hosts that belong to the group       #
#           @packs            Host packs                                   #
#   Outputs:                                                               #
#           @returns           List of images available in the group       #
#                                                                          #
#   Description: Retrieves a list of filenames for all available images    #
#                on the robots in a group.                                 #
#                                                                          #
# *************************************************************************#

def getGroupImageLists(groupHosts, packs):

    imageList = []

    # Obtain a list of images present on each robot
    for host in groupHosts:
        if host in packs:

            query = queryHost(packs, host, 'get_image_list')

            # Only continue if the query has content
            if query:

                # Obtain the host image list from the query
                hostImageList = query.images

                for imageFile in hostImageList:
                    imageList.append(host+" "+imageFile)

    return imageList

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @imageQueue       Prioriry queue of images                     #
#           @imageList        List of available images                     #
#   Outputs:                                                               #
#           @modifies         imageQueue by reference                      #
#                                                                          #
#   Description: This function updates a the priority image queue, with    #
#                the priority function being the age of the image, which   #
#                is available in the file name. The older the image, the   #
#                higher priority it has. This scheme is inherently round   #
#                round-robin.                                              #
#                                                                          #
# *************************************************************************#

def genPriorityImageQueueRR(imageQueue, imageList):

    # Insert each result into the queue if it isn't already in there
    for imageFile in imageList:
        if imageFile not in imageQueue:
            bisect.insort(imageQueue, imageFile)

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @filename         Name of the image to be saved                #
#           @imageBag         The image bag to saved                       #
#                                                                          #
#   Description: Extracts an image and its metadata from the               #
#                ros_robo_bob/Image message and creates an entry in the    #
#                Mongo DB. The image itself is saved into                  #
#                IMAGE_STORAGE_DIR, with the Mong DB entry                 #
#                containing a path to its respective image.                #
#                                                                          #
# *************************************************************************#

def saveImageToDB(filename, image):

    # Used to convert sensor_msgs/Image to an OpenCV image
    bridge = CvBridge()

    # Construct the filename
    filename = IMAGE_STORAGE_DIR+"/"+filename.split(".")[0]+IMAGE_FILE_TYPE

    # Attempt to insert the entry into the DB
    try:

        # Convert the image message to an opencv image
        cvImage = bridge.imgmsg_to_cv2(image.data, "bgr8")

        # Save the image
        cv2.imwrite(filename, cvImage)

        # Create the DB entry
        dbEntry = imageMessage2Metadata(image)
        dbEntry['localpath'] = filename

        # Initialize the Mongo DB client
        imageDBClient = MongoClient("mongodb://localhost:"+str(MONGO_IMAGE_DB_PORT)+"/")
        imageDB = imageDBClient[MONGO_IMAGE_DB_NAME]
        dbCollection = imageDB[MONGO_IMAGE_DB_COLLECTION]

        dbCollection.insert_one(dbEntry)

    except:
        rospy.loginfo("Failed to insert " + filename + " into DB")
        return False

    rospy.loginfo("Succesfully Inserted " + filename + " into DB")

    return True

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @imageQueue       Prioriry queue of images                     #
#           @packs            Host packs                                   #
#                                                                          #
#   Description: Given a queue of images, this function downloads each     #
#                image entry from its respective host and saves them       #
#                as ROS bags                                               #
#                                                                          #
# *************************************************************************#

def downloadImages(imageQueue, packs):

    # Loop through each entry in the priority queue
    for entry in imageQueue:

        # Obtain the name host and image name
        host, imageFilename = entry.split(" ")

        # Make sure the host hasn't been removed
        if host in packs:

            # Download the image bag
            image = getImage(packs, host, imageFilename)

            # Remove the image entry from the queue
            imageQueue.remove(entry)

            # Save the image to the mongo db if there was no error
            if image:

                # Save the image to the local mongo db
                saveImageToDB(imageFilename, image.image)

                rospy.loginfo("Downloaded "+imageFilename)

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @groupHosts       List of hosts that belong to the group       #
#           @packs            Host packs                                   #
#           @map              Map object to search through                 #
#                                                                          #
#   Description: Worker function that runs on its own thread. For each     #
#                gateworker group defined in the groups folder, this will  #
#                manage image uploading for all the hosts in that group.   #
#                If the map changes, the worker will automatically         #
#                adjusts the hosts it addresses.                           #
#                                                                          #
# *************************************************************************#

def gateWorker(groupFilename, packs, map):

    imageQueue = []

    while not rospy.is_shutdown():

        # Find all of the hosts physically present in group
        groupHosts = findHostsInGroup(groupFilename, map)

        if len(groupHosts) > 0:

            # Obtain a list of images from all robots in the group
            imageList = getGroupImageLists(groupHosts, packs)

            # Generate a priority image queue
            if IMAGE_PRIORITY_METHOD == "roundrobin":

                genPriorityImageQueueRR(imageQueue, imageList)

            # Download the images in  the queue
            downloadImages(imageQueue, packs)

        # Wait some time until we ask for more images
        rospy.sleep(GATE_WORKER_IMG_RETRY_TIME_S)

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Description: Worker function that uploads images from the Mongo DB     #
#                as they arrive. If an image is successfully uploaded,     #
#                delete the file off of the filesystem and remove the      #
#                entry from the DB.                                        #
#                                                                          #
# *************************************************************************#

def uploader():

    rate = rospy.Rate(UPLOADER_POLL_HZ)

    # Initialize the Mongo DB client
    imageDBClient = MongoClient("mongodb://localhost:"+str(MONGO_IMAGE_DB_PORT)+"/")
    imageDB = imageDBClient[MONGO_IMAGE_DB_NAME]
    dbCollection = imageDB[MONGO_IMAGE_DB_COLLECTION]

    while not rospy.is_shutdown():

        # Poll the Mongo DB at a certain HZ
        rate.sleep()

        # Obtain an entry from the DB
        entry = dbCollection.find_one()

        if entry:

            # Make sure we have the file
            if not os.path.isfile(os.getcwd()+"/"+entry['localpath']):

                # Delete the entry from the DB
                dbCollection.delete_one({'_id':entry['_id']})

                rospy.logerr("DB entry image path doesn't exist")

                continue

            # Obtain the local path to the image
            localImagePath = os.getcwd()+"/"+entry['localpath']

            # Remove the local path reference from the metadta
            del entry['localpath']

            # Attempt to upload an image and its metadata to the data pipeline
            if upload_image(localImagePath, entry):

                # Delete the image off of the filesystem
                os.remove(localImagePath)

                # Delete the entry from the DB
                dbCollection.delete_one({'_id':entry['_id']})

                rospy.loginfo("Succesfully uploaded " + os.path.basename(localImagePath) + " to the cloud")

            else:
                rospy.loginfo("Failed to upload " + os.path.basename(localImagePath) + " to the cloud")


# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @customer          Name of the customer                        #
#           @facility          Name of the facility                        #
#           @packs             Host packs                                  #
#           @map               Map object to search through                #
#           @workerThreads     Thread list to populate with workers        #
#                                                                          #
#   Description: Finds each gateworker group for the facility,             #
#                creates a gateworker thread for each, and finally starts  #
#                the threads.                                              #
#                                                                          #
# *************************************************************************#

def startGateworkerThreads(customer, facility, packs, map, workerThreads):

    # Get the group file names for this facility
    groupFiles = getFacilityWorkerGroups(customer, facility)

    # Create the worker threads
    for file in groupFiles:
        workerThreads.append(Thread(target=gateWorker, args=(file, packs, map,)))

    # Start the workers
    for worker in workerThreads:
        worker.start()

def main():

    # Nested dictionary object containing the map hierarchy of the facility
    map = {}

    # Dictionary of 'packs' which contain ros service/publisher/subscriber bundles
    packs = {}

    # List of worker threads
    gateWorkerThreads = []

    # Initialize the node
    rospy.init_node(GATEKEEPER_NODE_NAME)

    # Make the data directory if it doesn't already exist yet
    call(["mkdir", "-p", IMAGE_STORAGE_DIR])

    # Obtain the customer and facility from the launch arguments
    customer = rospy.get_param("customer")
    facility = rospy.get_param("facility")

    # Generate the facility map object
    genFacilityMap(customer, facility, map)

    # Wait for the multimaster nodes to come online
    waitForMultimaster()

    # Asynchronousy manages host discovery and pack creation
    hostDiscoveryThread = Thread(target=hostDiscovery, args=(packs, map,))

    # Moves images from the MongoDB to the cloud
    uploaderThread = Thread(target=uploader)

    # Start host discovery and uploader
    uploaderThread.start()
    hostDiscoveryThread.start()

    # State the gateworkers
    startGateworkerThreads(customer, facility, packs, map, gateWorkerThreads)

    rospy.spin()

if __name__ == '__main__':

    main()
