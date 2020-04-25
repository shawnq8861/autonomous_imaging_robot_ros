#!/usr/bin/env python

import os
import re
import sys
import json
import time
import rospy
import socket
import signal
import syslog
import rosnode
import roslaunch
import subprocess
from threading import Thread
from std_msgs.msg import UInt8
from ros_robo_bob.srv import *
from collections import OrderedDict
from utils import getBobStatusHPPArray

# Name of the supervisor node
SUPERVISOR_NODE_NAME = "supervisor"

# Name of the power node
POWER_NODE_NAME = "power_node"

# Names of data structures to extract from the bob_status.hpp file
BOB_STATUS_LIST_NAME = "BOB_STATUS_MESSAGES"
BOB_STATUS_OOOP_LIST_NAME = "BOB_STATUS_HANDLING_OOOP"

# State file used to keep track of error occurences
ERROR_STATE_FILENAME = "../../../devel/lib/ros_robo_bob/data/error_state.json"

# Time before error stacks are cleared
ERROR_STACK_TIMEOUT_S = 120

# The rate at which to check for commands to execute
COMMAND_POLL_HZ = 1

# True when a reboot condition has been triggered
systemRebooting = False

# True when a restart condition has been triggered
systemRestarting = False

# True when a hard power cycle has been initiated
systemHardPower = False

# A queue for holding commands for the supervisor to execute
commandQueue = []

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @filename               Name of the state file                 #
#           @arrayName              Number of states to keep track of      #
#   Outputs:                                                               #
#           @writes                 A JSON file with state data            #
#                                                                          #
#   Description: Creates a JSON file index by state numbers. Each state    #
#                gets its own stack, and a timestamp. Each level in the    #
#                stack corresponds to the state of the error handling      #
#                with respect to its OOOP entry.                           #
#                                                                          #
# *************************************************************************#

def initStateFile(filename, numStates):

    states = {}

    if not os.path.isfile(filename):
        for state in range(0, numStates):

            # Initialize the state
            states[state] = {'stack':[0], 'timestamp': 0}

        with open(filename, 'w') as stateFile:
            json.dump(states, stateFile)

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @filename               Name of the state file                 #
#           @numStates              The number of states in the error      #
#                                   system                                 #
#   Outputs:                                                               #
#           @returns                The contents of the state file if the  #
#                                   read was successfull                   #
#                                                                          #
#   Description: Attempts to read the state file w/ retries. If the file   #
#                can't be read, it re-initializes the state file.          #
#                                                                          #
# *************************************************************************#

def loadStateFile(filename, numStates):

    retries = 0
    states = None

    # Try to read the state file w/ retries
    while states == None and retries < 3:
        
        # Attempt to read in the states
        try:
            
            # Open the file
            statesFile = open(filename)
        
            states = json.load(statesFile)

        # Remove and re-initialize the file if there was an error
        except:

            try:
                os.remove(filename)
            except:
                pass

            initStateFile(filename, numStates)
            
        retries += 1
    
    statesFile.close()

    return states

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @errorCode              Error code to handle                   #
#           @stateFilename          Path to the state file                 #
#           @ooopList               List of OOOPs for each error code      #
#                                                                          #
#   Description: Uses the OOOP list to take action (if any) on the         #
#                the current error state. During the process, the JSON     #
#                state file is updated with the latest stack for each      #
#                error code along with timestamps of the last time the     #
#                error was seen. If the the time between the same error    #
#                exceedes a certain threshold, the stack for that error    #
#                is reset.                                                 #
#                                                                          #
# *************************************************************************#

def handleError(errorCode, stateFilename, ooopList):

    addLevel = False
    operations = []
    operation2Execute = None
    resetStack = False

    # Don't do anything if a reboot condition has already been triggered
    if systemRebooting:
        return

    # Load the JSON into a dict object
    states = loadStateFile(stateFilename, len(ooopList))

    # Exit if there was an error loading the state file
    if not states:
        return

    # Obtain the state from the dict
    state = states[str(errorCode)]

    if len(ooopList[errorCode]) > 0:
        
        # Parse in the operations from the codes OOOP entry
        ooop = ooopList[errorCode].split(",")

        for op in ooop:
            operations.append(int(op.split(" ")[1]))

        # Update the state stack
        for level in range(0, len(state['stack'])):

            # Exit if we've reset the stack
            if resetStack:
                break

            # Obtain the current time
            currentTime = int(time.time()) 
           
            # Increment the counter at this level of the stack
            state['stack'][level] += 1
 
            # Check and see if a timestamp exists
            if state['timestamp'] > 0:

                # Check to see if a timeout occurred
                if currentTime - state['timestamp'] > ERROR_STACK_TIMEOUT_S:
            
                    # Reset the stack if there was a timeout
                    state['stack'] = [1]

                    # Record that we've reset the stack
                    resetStack = True

            # Check to see if we've reached the criteria for moving to the next level
            if state['stack'][level] > operations[level]:

                # Reset the counter for this level
                state['stack'][level] = 0

                # Check to see if the level exists
                if len(state['stack']) <= level + 1 and len(ooop) > level + 1:
                    addLevel = True

                # Record that an operation needs to be executed
                operation2Execute = ooop[level].split(" ")[0]

                continue
            else:
                break

        # Record the time
        state['timestamp'] = int(time.time())

        # Add a new level if neccesarry
        if addLevel:
            state['stack'].append(1)

        # Update the file with new states
    with open(stateFilename, 'w') as stateFile:
            json.dump(states, stateFile)

    # Perform an action if neccessary
    if operation2Execute:
        commandQueue.append(operation2Execute)

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Inputs:                                                                #
#           @resetPowerClient       The reset power ROS client             #
#                                                                          #
#   Description: Executes a system-level operation.                        #
#                                                                          #
# *************************************************************************#

def commandThread(resetPowerClient):

    # Poll for commands at a certain rate
    rate = rospy.Rate(COMMAND_POLL_HZ)

    while True:

        if len(commandQueue) > 0:

            # Pop the command from the queue FIFO style
            operation = commandQueue.pop(0)
        else:
            continue

    	if operation == "restart":

            systemRestarting = True
            rospy.logerr("Supervisor restarting the system")
            os.system('systemctl restart robobob_daemon.service')

    	elif operation == "reboot":

            systemRebooting = True
            rospy.logerr("Supervisor rebooting the system")
            os.system('shutdown -r 1')

        elif operation == "hardpower":

            systemHardPower = True
            rospy.logerr("Supervisor hard power cycling the system")

            # Sync the system
            os.system("sync")

            # Hard power cycle
            res = resetPowerClient()

    	elif operation == "emergency":

            rospy.logerr("Robot entering emergency state")
            os.system('shutdown')

        rate.sleep()

# *************************************************************************#
#                                                                          #
#   Author(s): Ethan Takla                                                 #
#   Outputs:                                                               #
#           @message                The message object from the callback   #
#           @args                   Used to obtain the status and ooop     #
#                                   lists                                  #
#                                                                          #
#   Description: Runs whenever a status message is publish, which is       #
#                only when an error occurs. The error is automatically     #
#                written to syslogs.                                       #
#                                                                          #
# *************************************************************************#

def statusCB(message, args):

    # Log the status message
    rospy.logerr(args[0][message.data])

    # Appropriatley handle the error
    handleError(message.data, ERROR_STATE_FILENAME, args[1])

def main():

    # Initialize the node
    rospy.init_node(SUPERVISOR_NODE_NAME)

    # Obtain the status messages and OOOP list from the status HPP file
    bobStatusMessages = getBobStatusHPPArray('../include/bob_status.hpp', BOB_STATUS_LIST_NAME)
    bobStatusOOOPList = getBobStatusHPPArray('../include/bob_status.hpp', BOB_STATUS_OOOP_LIST_NAME)

    # Create a service client for hard power cycling
    resetPowerClient = rospy.ServiceProxy(POWER_NODE_NAME+'/reset_power', ResetPower)

    # Check and see if the number of states and size of the state file has changed. If it has, 
    # re-make the state file
    if len(loadStateFile(ERROR_STATE_FILENAME, len(bobStatusMessages))) != len(bobStatusMessages):
       
        rospy.logerr("SIZE MISMATCH DETECTED")

        # Remove the old state file
        os.remove(ERROR_STATE_FILENAME)
        
        # Initialize the state file
        initStateFile(ERROR_STATE_FILENAME, len(bobStatusMessages))
    
    # Create the command handling thread
    commandParsingThread = Thread(target=commandThread, args=[resetPowerClient])

    # Start the command parsing thread
    commandParsingThread.start()

    # Create a subscriber to get status updates from the control system
    rospy.Subscriber("status", UInt8, statusCB, (bobStatusMessages, bobStatusOOOPList), queue_size=1)


    rospy.spin()

if __name__ == '__main__':

    main()
