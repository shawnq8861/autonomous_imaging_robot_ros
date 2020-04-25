#ifndef ROS_ROBO_BOB_STATUS_H
#define ROS_ROBO_BOB_STATUS_H

/* General c++ includes */
#include <stdint.h>
#include <string>

/* ROS includes */
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

/* Error code definitons */
enum
{
    BOB_SUCCESS,
    BOB_BQ4050_I2C_RW_ERR,
    BOB_BQ24770_I2C_RW_ERR,
    BOB_CURRENT_SERVICE_FAILED,
    BOB_VOLTAGE_SERVICE_FAILED,
    BOB_LOST_CHARGER_SYNC,
    BOB_BQ4050_BAD_VOLTAGE,
    BOB_CANNOT_READ_RFID,
    BOB_CANNOT_FIND_RFID,
    BOB_INVALID_IDS_CAM_CAL,
    BOB_IDS_CAM_ERROR,
    BOB_FTD4222_NOT_FOUND,
    BOB_FTD4222_ERROR,
    BOB_MFRC522_SPI_RW_ERROR,
    BOB_UP2_GPIO_ERROR,
    BOB_NO_RFID_SAVED,
    BOB_UNEXPECTED_STOP,
    BOB_GIMBAL_FAILURE,
    BOB_GIMBAL_PARSE_ERROR,
    BOB_CANNOT_FIND_CHARGER,
    BOB_CANNOT_FIND_HOME,
    BOB_MTGP_SERVICE_FAILED,
    BOB_SAGP_SERVICE_FAILED,
    BOB_FC_SERVICE_FAILED,
    BOB_CI_SERVICE_FAILED,
    BOB_FH_SERVICE_FAILED,
    BOB_GIMBAL_SERVICE_FAILED,
    BOB_ACTION_CANCELLED,
    BOB_MOVE_TIMEOUT,
    BOB_SEEK_TIMEOUT,
    BOB_FIND_CHARGER_TIMEOUT,
    BOB_CAPTURE_IMAGE_TIMEOUT,
    BOB_MOVE_CAMERA_TIMEOUT,
    BOB_FIND_HOME_TIMEOUT,
    BOB_RFID_MISMATCH,
    BOB_AR_SERVER_TIMEOUT,
    BOB_TRACK_DOESNT_EXIST,
    BOB_INCORRECT_FILE_PERMISSIONS,
    BOB_IMAGE_NOT_FOUND,
    BOB_MRAA_UART_FAILURE,
    BOB_ABS_SERVICE_FAILED,
    BOB_IMAGE_CAPTURE_FAILED,
    BOB_FTD4222_LOCK_TIMEOUT,
    BOB_WHITE_BALANCE_FAILED,
    BOB_CHARGER_STATE_LOCKED,
    BOB_DARK_IMAGE,
    BOB_BLURRY_IMAGE,
    BOB_NW_CONNECTION_ERROR,
    BOB_FE_SERVICE_FAILED,
    BOB_FIND_END_TIMEOUT
};

/*  Handling order of operations for each error

    The format is as following: "command <occurences of state one
                                         lower in the hierarchy to trigger
                                         this command>"
    Example (for BOB_BQ4050_I2C_RW_ERR):

        "reboot 3, emergency 1" means that a reboot should occur after
        3 occurences of BOB_BQ4050_I2C_RW_ERR, and an emergency state should
        be entered if a reboot has occured one time already 

    The index of the OOOP string translates to the index of the error code
    in the status code definition enum. So the second string in the 
    BOB_STATUS_HANDLING_OOOP corresponds to a BOB_BQ4050_I2C_RW_ERR error
*/

const char * BOB_STATUS_HANDLING_OOOP[]
{
    "",
    "restart 2,reboot 2,emergency 2",
    "restart 2,reboot 2,emergency 2",
    "restart 0,reboot 2,emergency 2",
    "reboot 0,emergency 3",
    "",
    "restart 2,reboot 2,emergency 2",
    "",
    "",
    "",
    "",
    "restart 2,reboot 2,emergency 2",
    "restart 2,reboot 2,emergency 2",
    "restart 2,reboot 2,emergency 2",
    "restart 2,reboot 2,emergency 2",
    "",
    "restart 2,reboot 2,emergency 2",
    "emergency 2",
    "emergency 2",
    "restart 2,reboot 2,emergency 2",
    "restart 2,reboot 2,emergency 2",
    "restart 0,reboot 2,emergency 2",
    "restart 0,reboot 2,emergency 2",
    "restart 0,reboot 2,emergency 2",
    "restart 0,reboot 2,emergency 2",
    "restart 0,reboot 2,emergency 2",
    "restart 0,reboot 2,emergency 2",
    "",
    "reboot 0,emergency 2",
    "reboot 0,emergency 2",
    "reboot 0,emergency 2",
    "reboot 0,emergency 2",
    "reboot 0,emergency 2",
    "reboot 0,emergency 2",
    "",
    "restart 0,reboot 2,emergency 2",
    "emergency 1",
    "emergency 1",
    "restart 2,reboot 2,emergency 2",
    "restart 2,reboot 2,emergency 2",
    "restart 2,reboot 2,emergency 2",
    "hardpower 0",
    "restart 2,reboot 2,emergency 2",
    "restart 2,reboot 2,emergency 2",
    "emergency 0",
    "",
    "",
    "reboot 30,emergency 2",
    "restart 0,reboot 2,emergency 2",
    "reboot 0,emergency 2"
};

/* Battery state definitions */
enum
{
    BOB_BATTERY_OK,
    BOB_BATTERY_LOW,
    BOB_BATTERY_CRITICAL,
    BOB_BATTERY_SHUTDOWN,
    BOB_BATTERY_UNKNOWN
};

/* Error message definitions */
const char * BOB_STATUS_MESSAGES[] =
{
    "BOB_SUCCESS",
    "BOB_BQ4050_I2C_RW_ERR: An I2C error occured while trying to communicate with the fuel gauge IC",
    "BOB_BQ24770_I2C_RW_ERR: An I2C error occured while trying to communicate with the charger IC",
    "BOB_CURRENT_SERVICE_FAILED: A call to the current service failed",
    "BOB_VOLTAGE_SERVICE_FAILED: A call to the voltage service failed",
    "BOB_LOST_CHARGER_SYNC: The robot lost sync with the wireless transmitter",
    "BOB_BQ4050_BAD_VOLTAGE: The voltage reading from the BQ4050 isn't quite right",
    "BOB_CANNOT_READ_RFID: A tag is in range, but is unable to be read",
    "BOB_CANNOT_FIND_RFID: There is no RFID tag in range of the reader",
    "BOB_INVALID_IDS_CAM_CAL: IDS camera calibration failed",
    "BOB_IDS_CAM_ERROR: An error occured while trying to capture an image",
    "BOB_FTD4222_NOT_FOUND: The FT4222 device isn't registering with the OS",
    "BOB_FTD4222_ERROR: The FT4222 device encountered an error while servicing an API call",
    "BOB_MFRC522_SPI_RW_ERROR: An SPI error occured while trying to communicate with the RFID IC",
    "BOB_UP2_GPIO_ERROR: An error occured during a R/W to a GPIO pin",
    "BOB_NO_RFID_SAVED: No RFID code has been saved to memory yet",
    "BOB_UNEXPECTED_STOP: The robot experienced an un-planned stop while executing a motion command",
    "BOB_GIMBAL_FAILURE: The gimbal could not move to the desired position",
    "BOB_GIMBAL_PARSE_ERROR: An error occured parsing data from the gimbal board",
    "BOB_CANNOT_FIND_CHARGER: The robot failed to detect the charger",
    "BOB_CANNOT_FIND_HOME: The robot failed to find a hall effect sensor",
    "BOB_MTGP_SERVICE_FAILED: A move_to_goal_position service call failed",
    "BOB_SAGP_SERVICE_FAILED: A seek_a_goal_position service call failed",
    "BOB_FC_SERVICE_FAILED: A find_charger service call failed",
    "BOB_CI_SERVICE_FAILED: A capture_image service call failed",
    "BOB_FH_SERVICE_FAILED: A find_home service called failed",
    "BOB_GIMBAL_SERVICE_FAILED: A gimbal service call failed",
    "BOB_ACTION_CANCELLED: The current action has been cancelled by a client",
    "BOB_MOVE_TIMEOUT: A move action exceeded the expected timeout period",
    "BOB_SEEK_TIMEOUT: A seek action exceeded the expected timeout period",
    "BOB_FIND_CHARGER_TIMEOUT: A find_charger action exceeded the timeout period",
    "BOB_CAPTURE_IMAGE_TIMEOUT: A capture_image action exceeded the expected timeout period",
    "BOB_MOVE_CAMERA_TIMEOUT: A move_camera action exceeded the expected timeout period",
    "BOB_FIND_HOME_TIMEOUT: A find_home action exceeded the expected timeout period",
    "BOB_RFID_MISMATCH: The reported RFID tag doesn't match the one present in the plan",
    "BOB_AR_SERVER_TIMEOUT: The action server has taken too long to contact",
    "BOB_TRACK_DOESNT_EXIST: The track file doesn't exist, or the RFID code doesn't have a matching track file",
    "BOB_INCORRECT_FILE_PERMISSIONS: The node has insufficient permissions to create files",
    "BOB_IMAGE_NOT_FOUND: The requested image from the gatekeeper isn't present on the robot",
    "BOB_MRAA_UART_FAILURE: There was a MRAA UART failure",
    "BOB_ABS_SERVICE_FAILED: A absolute charge state service call failed",
    "BOB_IMAGE_CAPTURE_FAILED: The frame grab failed or the camera lost connection",
    "BOB_FTD4222_LOCK_TIMEOUT: The FT4222 device has taken too long to respond",
    "BOB_WHITE_BALANCE_FAILED: Unable to get ro set the white balance gains or perform auto white balance",
    "BOB_CHARGER_STATE_LOCKED: The charger IC is refusing to let the battery charge",
    "BOB_DARK_IMAGE: A Dark image was captured",
    "BOB_BLURRY_IMAGE: A blurry image was captured",
    "BOB_NW_CONNECTION_ERROR: The robot was unable to connect to the network",
    "BOB_FE_SERVICE_FAILED: A find_end service call failed",
    "BOB_FIND_END_TIMEOUT: The action server exceeded the expected timeout period"
};

/* Typedefs */
typedef uint8_t BobStatus;

/* Function declarations */
void printStatus(BobStatus);
void handleStatus(BobStatus, ros::Publisher*);

#endif
