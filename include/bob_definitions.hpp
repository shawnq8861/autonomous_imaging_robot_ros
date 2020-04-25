#ifndef BOB_DEFINITIONS_H
#define BOB_DEFINITIONS_H

/* Node name definitions */
#define ODOM_NODE_NAME "odom"
#define MH04_NODE_NAME  "mh04"
#define GIMBAL_NODE_NAME "gimbal"
#define POWER_NODE_NAME "power_node"
#define LS7366_NODE_NAME "ls7366"
#define MOTION_NODE_NAME "motion"
#define CONTROL_NODE_NAME "control"
#define ENCODER_NODE_NAME "encoder"
#define MFRC522_NODE_NAME "mfrc522"
#define UPLOADER_NODE_NAME "uploader"
#define USER_CONTROL_NODE "user_control"
#define SUPERVISOR_NODE_NAME "supervisor"
#define GATEKEEPER_NODE_NAME "/gatekeeper"
#define LUMENARA_NODE_NAME "lumenera_camera"
#define BASE_CONTROLLER_NODE_NAME "base_controller"

/* Data definitions */
#define PC_STACK_FILENAME "data/pcstack.dat"
#define CONTROL_STATE_FILENAME "data/controlstate.dat"
#define STOP_NUMBER_FILENAME "data/stopnumber.dat"
#define IMAGES_PATH "/home/catkin_ws/bob_images"
#define PLAN_REPO_PATH "/home/robo_bob_plans"
#define XYZ_GAIN_FILE_PATH "/home/catkin_ws/devel/lib/ros_robo_bob/data/wb_xyz_gain.yaml"

#endif
