#!/bin/bash

# Source the proper files to enable ros functionality
source ~/.bashrc
source /opt/ros/kinetic/setup.bash
source /home/catkin_ws/devel/setup.bash

# Create a symlink for the manual control node
ln -s /home/catkin_ws/src/ros_robo_bob/scripts/manual /usr/bin/manual

# Limit the CPU frequency
echo "800000" > /sys/devices/system/cpu/cpu0/cpufreq/scaling_max_freq

# Export the credentials for uploading using google
export GOOGLE_APPLICATION_CREDENTIALS="/home/catkin_ws/src/ros_robo_bob/credentials/google/datastore-cc8d59f69177.json"

# Make sure the software pwm module is loaded
insmod '/home/catkin_ws/src/ros_robo_bob/kernel/softpwm/softpwm.ko'

# Make sure the hall effect module is loaded
insmod '/home/catkin_ws/src/ros_robo_bob/kernel/halleffect/halleffect.ko'

# Run the deploy launch file with the systems hostname
roslaunch ros_robo_bob startControlSystem.launch hostname:="$(hostname)" &
