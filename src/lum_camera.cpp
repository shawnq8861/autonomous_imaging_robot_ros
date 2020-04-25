#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <camera_calibration_parsers/parse.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <../include/utils.hpp>
#include <../include/bob_status.hpp>
#include <lumenera/lucamapi.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sys/stat.h>
#include <unistd.h>
#include <libgen.h>
#include <string.h>
#include <ros_robo_bob/SaveImage.h>
#include <pwd.h>
#include <boost/filesystem.hpp>
#include "../include/bob_definitions.hpp"

#define TARGET_BRIGHTNESS       50
#define LOOP_AND_FRAME_RATE     30.0
#define CHAR_BUFFER_SIZE        100

//
// IPC mutex
//
static std::mutex img_mtx;
//
// static global variables
//
static sensor_msgs::Image image;
static ULONG imageWidth;
static ULONG imageHeight;
//
// constants
//
static const std::string cameraName = "lumenera_camera";
static const char *calibrationFolderName =
        "/catkin_ws/src/ros_robo_bob/data/";
static const char *calibrationFileName = "calibration_params.yaml";
static char calibrationFilePath[CHAR_BUFFER_SIZE];

//
// helper function that checks calibration file path, and creates the
// directory if it does not exist
//
bool checkCalibrationFilePath(void)
{
    //
    // get the home directory
    //
    struct passwd *pw = getpwuid(getuid());
    char *homeDir = pw->pw_dir;
    //
    // build up the file path
    //
    // first home home directory
    //
    strcpy(calibrationFilePath, homeDir);
    //
    // next directory path
    //
    strcat(calibrationFilePath, calibrationFolderName);
    ROS_INFO_STREAM("folder dir: " << calibrationFilePath);
    //
    // create the directory if it does not exist
    //
    struct stat statBuff;
    int folderFound = stat(calibrationFilePath, &statBuff);
    if (folderFound == -1) 
        createDir(calibrationFilePath);
    else 
        ROS_INFO_STREAM("folder found...");
    
    //
    // and finally the file name
    //
    strcat(calibrationFilePath, calibrationFileName);
    ROS_INFO_STREAM("calibration file path: " << calibrationFilePath);

    return true;
}

//
// service callback used to set camera calibration parameters and save
// the values to a file
//
bool setAndSaveCameraCalibrationData(sensor_msgs::SetCameraInfo::Request &req,
                                     sensor_msgs::SetCameraInfo::Response &resp,
                                     sensor_msgs::CameraInfo *cameraInfo)
{
    //
    // set the camera info
    //
    cameraInfo->header.stamp = ros::Time::now();
    cameraInfo->height = imageHeight;
    cameraInfo->width = imageWidth;
    cameraInfo->distortion_model = req.camera_info.distortion_model;
    cameraInfo->D = req.camera_info.D;
    cameraInfo->K = req.camera_info.K;
    cameraInfo->R = req.camera_info.R;
    cameraInfo->P = req.camera_info.P;
    cameraInfo->binning_x = req.camera_info.binning_x;
    cameraInfo->binning_y = req.camera_info.binning_y;
    cameraInfo->roi = req.camera_info.roi;
    //
    // write data to yaml file
    //
    camera_calibration_parsers::writeCalibration(calibrationFilePath,
                                                 cameraName,
                                                 *cameraInfo);
    //
    // fill in the response object
    //
    resp.status_message = "calibration data set";

    return true;
}

//
// initialize the camera calibration parameters if the parameter file is
// no found
//
// default values are all set to zero
//
bool initializeCameraCalibrationData(sensor_msgs::CameraInfo *cameraInfo)
{
    //
    // read the data
    //
    if(camera_calibration_parsers::readCalibration(
                calibrationFilePath,
                (std::string&)cameraName,
                *cameraInfo)) {
        ROS_INFO_STREAM("calibration file read...");
        return EXIT_SUCCESS;
    }
    else {
        //
        // set the camera info to default values
        //
        cameraInfo->header.stamp = ros::Time::now();
        setResource<uint32_t>(img_mtx, cameraInfo->height, imageHeight);
        setResource<uint32_t>(img_mtx, cameraInfo->width, imageWidth);
        cameraInfo->distortion_model = "plumb_bob";
        cameraInfo->binning_x = 0;
        cameraInfo->binning_y = 0;
        sensor_msgs::RegionOfInterest roi;
        setResource<uint32_t>(img_mtx, roi.height, imageHeight);
        setResource<uint32_t>(img_mtx, roi.width, imageWidth);
        roi.x_offset = 0;
        roi.y_offset = 0;
        cameraInfo->roi = roi;
        //
        // write data to yaml file
        //
        bool retValue = camera_calibration_parsers::writeCalibration(
                    calibrationFilePath,
                    cameraName,
                    *cameraInfo);
        if(!retValue) {
            return EXIT_FAILURE;
        }
        else {
            return EXIT_SUCCESS;
        }
    }
}

//
// service callback used to acquire the current image buffer for saving
//
bool saveImageCB(ros_robo_bob::SaveImageRequest& request,
                 ros_robo_bob::SaveImageResponse& response)
{
    setResource<sensor_msgs::Image>(img_mtx, response.image, image);
    response.status = BOB_SUCCESS;

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, LUMENARA_NODE_NAME);
    ros::NodeHandle nh;
    //
    // check the calibration file path, return if not valid
    //
    if (!checkCalibrationFilePath()) {
        return EXIT_FAILURE;
    }
    //
    // instantiate a publisher for camera images
    //
    ros::Publisher image_pub =
            nh.advertise<sensor_msgs::Image>(addBase2Topic(LUMENARA_NODE_NAME, "image_raw"), 10);
    //
    // instantiate a publisher for camera calibration info
    //
    ros::Publisher camera_info_pub =
            nh.advertise<sensor_msgs::CameraInfo>(addBase2Topic(LUMENARA_NODE_NAME, "camera_info"), 10);
    //
    // set the loop rate used by spin to control while loop execution
    // this is an integer that equates to loops/second
    //
    ros::Rate loop_rate = LOOP_AND_FRAME_RATE;
    //
    // instantiate a service to be called when saving images
    //
    ros::ServiceServer save_image_service = nh.advertiseService<
            ros_robo_bob::SaveImageRequest,
            ros_robo_bob::SaveImageResponse>(addBase2Topic(LUMENARA_NODE_NAME, "save_image"),
                                             &saveImageCB);
    //
    // camera information held in this data structure
    //
    sensor_msgs::CameraInfo cameraInfo;
    //
    // instantiate a service to be called when after the camera calibrated
    //
    ros::ServiceServer calibration_service =
            nh.advertiseService<sensor_msgs::SetCameraInfoRequest,
            sensor_msgs::SetCameraInfoResponse>
            (addBase2Topic(LUMENARA_NODE_NAME, "set_camera_info"),
             boost::bind(setAndSaveCameraCalibrationData, _1, _2, &cameraInfo));
    //
    // get the number of cameras
    //
    LONG numCameras = LucamNumCameras();
    ROS_INFO_STREAM("number of cameras: " << numCameras);
    //
    // attempt to get a camera handle
    //
    HANDLE hCamera = LucamCameraOpen(1);
    if (NULL == hCamera) {
        ROS_ERROR_STREAM("ERROR %u: Unable to open camera.\n"
                         << (unsigned int)LucamGetLastError());
        return EXIT_FAILURE;
    }
    //
    // get the frame format
    //
    LUCAM_FRAME_FORMAT frameFormat;
    float frameRate = -1.0f;
    LucamGetFormat(hCamera, &frameFormat, &frameRate);
    imageWidth = (frameFormat.width / frameFormat.subSampleX);
    imageHeight = (frameFormat.height / frameFormat.subSampleY);
    LUCAM_CONVERSION conversionParams;
    conversionParams.CorrectionMatrix = LUCAM_CM_FLUORESCENT;
    conversionParams.DemosaicMethod = LUCAM_DM_HIGHER_QUALITY;
    //
    // display current frame rate
    //
    ROS_INFO_STREAM("current frame rate: " << frameRate);
    //
    // set new frame rate
    //
    LucamSetFormat(hCamera, &frameFormat, LOOP_AND_FRAME_RATE);
    //
    // display current frame rate
    //
    LucamGetFormat(hCamera, &frameFormat, &frameRate);
    ROS_INFO_STREAM("current frame rate: " << frameRate);
    loop_rate = frameRate;
    //
    // initialize camera calibration data
    //
    if(EXIT_FAILURE == initializeCameraCalibrationData(&cameraInfo)) {
        ROS_ERROR_STREAM("calibration data initialization failed...");
        return EXIT_FAILURE;
    }
    //
    // create a vector to hold image data
    //
    std::vector<unsigned char> rawImageData(imageHeight * imageWidth);
    //
    // start the video stream, NULL window handle
    //
    if (LucamStreamVideoControl(hCamera, START_STREAMING, NULL) == FALSE){
       ROS_INFO_STREAM("Failed to start streaming");
    }
    //
    // loop while acquiring image frames from the stream
    //
    while(ros::ok()) {
        //
        // set one shot auto exposure target
        //
        UCHAR brightnessTarget = TARGET_BRIGHTNESS;
        ULONG startX = 0;
        ULONG startY = 0;
        //
        // set auto exposure
        //
        LucamOneShotAutoExposure(hCamera,
                                 brightnessTarget,
                                 startX,
                                 startY,
                                 imageWidth,
                                 imageHeight);
        //
        // set auto gain
        //
        LucamOneShotAutoGain(hCamera,
                             brightnessTarget,
                             startX,
                             startY,
                             imageWidth,
                             imageHeight);
        //
        // set auto white balance
        //
        LucamOneShotAutoWhiteBalance(hCamera,
                                     startX,
                                     startY,
                                     imageWidth,
                                     imageHeight);
        //
        // grab a snap shot
        //
        LONG singleFrame = 1;
        if(LucamTakeVideo(hCamera, singleFrame, (BYTE *)rawImageData.data()) == FALSE){
            ROS_ERROR_STREAM("Failed to capture image");
        }
        //
        // allow the auto settings to settle before transferring image data                  
        //
        // update camera info time stamp
        //
        setResource<ros::Time>
                (img_mtx, cameraInfo.header.stamp, ros::Time::now());
        //
        // configure the image message
        //
        setResource<ros::Time>
                (img_mtx, image.header.stamp, cameraInfo.header.stamp);
        setResource<uint32_t>(img_mtx, image.height, imageHeight);
        setResource<uint32_t>(img_mtx, image.width, imageWidth);
        setResource<uint32_t>(img_mtx, image.step, imageWidth);
        setResource<std::string>(img_mtx, image.encoding,
                                 sensor_msgs::image_encodings::BAYER_BGGR8);
        //
        // set the raw pixel data
        //
        setResource<std::vector<unsigned char>>(img_mtx,
                                                image.data,
                                                rawImageData);
        //
        // publish the image to an image_view data type
        //
        image_pub.publish(image);
        //
        // publish the camera info
        //
        camera_info_pub.publish(cameraInfo);
        //
        // process callbacks and check for messages
        //
        ros::spinOnce();
        loop_rate.sleep();
    }

    LucamCameraClose(hCamera);

}