#!/usr/bin/env python

import os
import json
import rospy
import requests
from std_msgs.msg import UInt8
from google.cloud import storage
from google.cloud import pubsub_v1

# Google cloud project info
GOOGLE_CLOUD_DB_BUCKET_NAME = "robobob_office_test_bucket"
GOOGLE_CLOUD_DB_PROJECT_ID = "production-database-209522"
GOOGLE_CLOUD_DB_IMAGE_PUB_TOPIC = "office_test_image_data"

# Defines which data service to use. Options are "aws", and "google"
DATA_SERVICE = "aws"

#
# Taken from Google Storage's reference library
# https://github.com/GoogleCloudPlatform/python-docs-samples/blob/master/storage/cloud-client/snippets.py
#
def upload_blob(bucket_name, source_file_name, destination_blob_name):

    blobNames = []

    try:
        storage_client = storage.Client()
        bucket = storage_client.get_bucket(bucket_name)
        blob = bucket.blob(destination_blob_name)

        # Upload the blob to the bucket
        blob.upload_from_filename(source_file_name)

        # Create a list of blob names currently in the bucket
        blobs = bucket.list_blobs()

        for blob in blobs:
            blobNames.append(blob.name)

            # Check to see if it was uploaded
        if destination_blob_name in blobNames:
            return True
    except:
        rospy.logerr("Exception occured while uploading blob")

    return False

#
# Taken from Google PubSub reference library
# https://github.com/GoogleCloudPlatform/python-docs-samples/blob/master/pubsub/cloud-client/publisher.py
#
def publish_google_data(project_id, topic_name, payload):

    # Create the publisher client
    publisher = pubsub_v1.PublisherClient()

    # Create the topic path
    topic_path = publisher.topic_path(project_id, topic_name)

    # Construct the message
    data = u'{}'.format(payload)

    # Encode the message
    data = data.encode('utf-8')

    # Publish the message
    future = publisher.publish(topic_path, data=data)

    if future.exception():
        return False

    return True

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @path              Path to the image to be uploaded            #
#           @metadata          Dictionary of metadata                      #
#                                                                          #
#   Description: Uploads an image to the google data pipeline. This        #
#                consists of the image itself in blob form, and pub/sub    #
#                for the metadata.                                         #
#                                                                          #
# *************************************************************************#

def google_upload_image(path, metadata):

    # Publish the image metadata using google pub/sub
    if not publish_google_data(GOOGLE_CLOUD_DB_PROJECT_ID, GOOGLE_CLOUD_DB_IMAGE_PUB_TOPIC, metadata):
        return False

    # Attemp to upload the image to the data bucket
    if upload_blob(GOOGLE_CLOUD_DB_BUCKET_NAME, path, os.path.basename(path)):
        return True

    return False

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @path              Path to the image to be uploaded            #
#           @metadata          Dictionary of metadata                      #
#                                                                          #
#   Description: Uploads an image to the aws data pipeline. This           #
#                consists of a POST request.                               #
#                                                                          #
# *************************************************************************#

def aws_upload_image(path, metadata):

    # Create the target url
    targetURL = metadata['camdata']['customer']['edgeAPI'] + "/" \
              + metadata['camdata']['customer']['customerName'] + "/" \
              + metadata['camdata']['customer']['facility'] + "/" \
              + metadata['camdata']['location']['space']  + "/" \
              + metadata['camdata']['device']['hostname']

    # Compile the form data
    formData = {}
    formData['camdata'] = json.dumps(metadata['camdata'])
    formData['imagedata'] = json.dumps(metadata['imagedata'])

    # Obtain the filename from the path
    filename = metadata['imagedata']['timestamp'].split("T")[0]+"/"+os.path.basename(path)

    # Format the image
    image = [(filename, (filename, open(path, 'rb'), 'image/jpeg'))]

    try:

        # Send the POST request
        res = requests.post(targetURL, files=image, data=formData).json()
        
        # Print the post for debugging purposes
        rospy.logerr(res)
        
        # Error checking
        if res['result'] != "OK":
            return False

    except ValueError:

        rospy.logerr("Invalid edge api link provided")
        return False

    except requests.ConnectionError:

        rospy.logerr("Connection error occured while attempting to uplaod an image")
        return False

    except:

        rospy.logerr("An unspecified error occured while attempting to upload an image")
        return False



    return True

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @path              Path to the image to be uploaded            #
#           @metadata          Dictionary of metadata                      #
#                                                                          #
#   Description: Uploads an image to any of the data pipelien services,    #
#                depending on what the setting is.                         #
#                                                                          #
# *************************************************************************#

def upload_image(path, metadata):

    if DATA_SERVICE == "aws":
        return aws_upload_image(path, metadata)
    elif DATA_SERVICE == "google":
        return google_upload_image(path, metadata)
