cmake_minimum_required(VERSION 2.8.3)
project(ros_robo_bob)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

message($ENV{LOCAL})

IF(NOT $ENV{LOCAL} STREQUAL "")
	set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev/)
ELSE()
	set(OpenCV_DIR /usr/share/OpenCV/)
ENDIF()

#set(Eigen3_DIR /usr/lib/cmake/eigen3/)
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nodelet
  rosbag
  std_msgs
  nav_msgs
  tf
  OpenCV REQUIRED
  sensor_msgs
  cv_bridge
  image_transport
  cmake_modules REQUIRED
  #Eigen3 REQUIRED
  camera_calibration_parsers
  genmsg
  actionlib_msgs
  actionlib
  message_generation
)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

#Add messages
add_message_files(DIRECTORY msg FILES ImageMeta.msg)
add_message_files(DIRECTORY msg FILES NodeHealth.msg)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv FILES MoveToGoalPosition.srv)
add_service_files(DIRECTORY srv FILES SeekAGoalPosition.srv)
add_service_files(DIRECTORY srv FILES FindTheCharger.srv)
add_service_files(DIRECTORY srv FILES ReadRFID.srv)
add_service_files(DIRECTORY srv FILES GetLastRFID.srv)
add_service_files(DIRECTORY srv FILES StartCharge.srv)
add_service_files(DIRECTORY srv FILES SetCurrent.srv)
add_service_files(DIRECTORY srv FILES GetCurrent.srv)
add_service_files(DIRECTORY srv FILES GetVoltage.srv)
add_service_files(DIRECTORY srv FILES GetCharge.srv)
add_service_files(DIRECTORY srv FILES GetBatteryStatus.srv)
add_service_files(DIRECTORY srv FILES GetAbs.srv)
add_service_files(DIRECTORY srv FILES GetCells.srv)
add_service_files(DIRECTORY srv FILES FindHome.srv)
add_service_files(DIRECTORY srv FILES ControlMode.srv)
add_service_files(DIRECTORY srv FILES GetImageList.srv)
add_service_files(DIRECTORY srv FILES GetImage.srv)
add_service_files(DIRECTORY srv FILES FindChargeCurrent.srv)
add_service_files(DIRECTORY srv FILES MoveCamera.srv)
add_service_files(DIRECTORY srv FILES PowerGimbal.srv)
add_service_files(DIRECTORY srv FILES ChargeMaintenanceState.srv)
add_service_files(DIRECTORY srv FILES ResetCounter.srv)
add_service_files(DIRECTORY srv FILES SaveImage.srv)
add_service_files(DIRECTORY srv FILES GetPosition.srv)
add_service_files(DIRECTORY srv FILES CalibrateGimbal.srv)
add_service_files(DIRECTORY srv FILES SetTargetBrightness.srv)
add_service_files(DIRECTORY srv FILES GimbalOrientation.srv)
add_service_files(DIRECTORY srv FILES PerformWhiteBalance.srv)
add_service_files(DIRECTORY srv FILES GetRGBGains.srv)
add_service_files(DIRECTORY srv FILES StartStopStreaming.srv)
add_service_files(DIRECTORY srv FILES ResetPower.srv)
add_service_files(DIRECTORY srv FILES FindEnd.srv)

# The action files for the action client/server
add_action_files(DIRECTORY action FILES Seek.action)
add_action_files(DIRECTORY action FILES Move.action)
add_action_files(DIRECTORY action FILES MoveCamera.action)
add_action_files(DIRECTORY action FILES FindCharger.action)
add_action_files(DIRECTORY action FILES CaptureImage.action)
add_action_files(DIRECTORY action FILES FindHome.action)
add_action_files(DIRECTORY action FILES FindEnd.action)
generate_messages(DEPENDENCIES actionlib_msgs std_msgs sensor_msgs)

# Add our libraries
add_library(utils src/utils.cpp)
add_library(bob_status src/bob_status.cpp)
add_library(track_plan src/track_plan.cpp)
add_library(image_proc src/image_proc.cpp)
add_library(action_client src/action_client.cpp)
add_library(ftd4222_helper src/ftd4222_helper.cpp)
add_library(auto_white_balance src/auto_white_balance.cpp)
add_library(nodelet_camera_acquire src/camera_nodelet.cpp)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib OpenCV OpenMP
  CATKIN_DEPENDS actionlib_msgs
  LIBRARIES action_client
  LIBRARIES nodelet_camera_acquire
)
#find_package(DJIOSDK REQUIRED)

###########
## Build ##
###########

IF(NOT $ENV{LOCAL} STREQUAL "")
	set (mraa_INCLUDE_DIRS "/usr/local/include/mraa/")
	set (mraa_LIBRARIES "/usr/local/lib/libmraa.so")
	set (FTD4222_INCLUDE_DIRS "/usr/local/include/")
	set (FTD4222_LIBRARIES "/usr/local/lib/libft4222.so.1.4.1.231")
ELSE()
	set (mraa_INCLUDE_DIRS "/usr/include/mraa/")
	set (mraa_LIBRARIES "/usr/lib/libmraa.so")
	set (FTD4222_INCLUDE_DIRS "/usr/include/")
	set (FTD4222_LIBRARIES "/usr/lib/libft4222.so")
ENDIF()

set (lumenera_INCLUDE_DIRS "/usr/include/lumenera/")
#set (DJIOSDK_INCLUDE_DIRS "/usr/local/include/djiosdk/")
#set (DJIOSDK_LIBRARIES "/usr/local/lib/libdjiosdk-core.a")
set (lumenera_LIBRARIES "/usr/lib/liblucamapi.so.2.3")
#set (Eigen3_INCLUDE_DIRS "/usr/local/include/eigen3/Eigen/")



## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${mraa_INCLUDE_DIRS}
  #${DJIOSDK_INCLUDE_DIRS}
  ${lumenera_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${FTD4222_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pub_sub_demo.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(utils rosbag)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(base_controller src/base_controller.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(lum_camera src/camera_node.cpp)
add_executable(opt_flow src/opt_flow.cpp)
add_executable(ls7366 src/ls7366.cpp)
#add_executable(struct_from_motion src/struct_from_motion.cpp)
add_executable(action_server src/action_server.cpp)
add_executable(user_control src/user_control.cpp)
add_executable(control src/control.cpp)
add_executable(mfrc522 src/mfrc522.cpp)
add_executable(power_node src/power_node.cpp)
add_executable(mh04 src/mh04.cpp)
add_executable(motion src/motion.cpp)
add_executable(gimbal src/gimbal.cpp)

add_dependencies(action_server ros_robo_bob_generate_messages_cpp)
add_dependencies(action_client ros_robo_bob_generate_messages_cpp)
add_dependencies(odometry ros_robo_bob_generate_messages_cpp)
add_dependencies(motion ros_robo_bob_generate_messages_cpp)
add_dependencies(mh04 ros_robo_bob_generate_messages_cpp)
add_dependencies(power_node ros_robo_bob_generate_messages_cpp)
add_dependencies(mfrc522 ros_robo_bob_generate_messages_cpp)
add_dependencies(control ros_robo_bob_generate_messages_cpp)
add_dependencies(gimbal ros_robo_bob_generate_messages_cpp)
add_dependencies(lum_camera ros_robo_bob_generate_messages_cpp)
add_dependencies(ls7366 ros_robo_bob_generate_messages_cpp)
add_dependencies(nodelet_camera_acquire ros_robo_bob_generate_messages_cpp)
add_dependencies(image_proc ros_robo_bob_generate_messages_cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(utils
   ${catkin_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(base_controller
   ${catkin_LIBRARIES}
   ${mraa_LIBRARIES}
   ${FTD4222_LIBRARIES}
   ftd4222_helper
   bob_status
)
target_link_libraries(odometry
   ${catkin_LIBRARIES}
   utils
)

target_link_libraries(gimbal
   ${catkin_LIBRARIES}
   ${mraa_LIBRARIES}
   utils
)

target_link_libraries(lum_camera
   ${catkin_LIBRARIES}
   ${lumenera_LIBRARIES}
   ${OpenCV_LIBRARIES}
   utils
)

target_link_libraries(auto_white_balance
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${OpenMP_LIBS}
   yaml-cpp
)

target_link_libraries(opt_flow
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)

#target_link_libraries(struct_from_motion
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#)

target_link_libraries(action_server
   ${catkin_LIBRARIES}
   utils
)

target_link_libraries(action_client
   ${catkin_LIBRARIES}
   utils
)

target_link_libraries(bob_status
   ${catkin_LIBRARIES}
)

target_link_libraries(track_plan
   ${catkin_LIBRARIES}
   yaml-cpp
)

target_link_libraries(user_control
   ${catkin_LIBRARIES}
   action_client
   bob_status
   track_plan
   utils
   image_proc
)

target_link_libraries(ftd4222_helper
   ${catkin_LIBRARIES}
   ${FTD4222_LIBRARIES}
)

target_link_libraries(image_proc
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   utils
)

target_link_libraries(control
   ${catkin_LIBRARIES}
   action_client
   bob_status
   track_plan
   utils
   image_proc
)

target_link_libraries(motion
   ${catkin_LIBRARIES}
   utils
)

target_link_libraries(mfrc522
   ${catkin_LIBRARIES}
   ${mraa_LIBRARIES}
   utils
)
target_link_libraries(power_node
   ${catkin_LIBRARIES}
   ftd4222_helper
   bob_status
   utils
)
target_link_libraries(mh04
   ${catkin_LIBRARIES}
   bob_status
   utils
)

target_link_libraries(ls7366
   ${catkin_LIBRARIES}
   ${mraa_LIBRARIES}
    utils
)

target_link_libraries(nodelet_camera_acquire
    ${catkin_LIBRARIES}
    ${lumenera_LIBRARIES}
    ${OpenCV_LIBRARIES}
    image_proc
    auto_white_balance
)
if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(nodelet_camera_acquire ${catkin_EXPORTED_LIBRARIES})
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(
    TARGETS nodelet_camera_acquire
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
    FILES  lumenera_camera_nodelet.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pub_sub_demo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
