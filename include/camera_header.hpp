

#ifndef CAMERA_HEADER_HPP
#define CAMERA_HEADER_HPP

#include <ros/ros.h>
#include <nodelet/loader.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <camera_calibration_parsers/parse.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include "ros_robo_bob/SaveImage.h"
#include "ros_robo_bob/SetTargetBrightness.h"
#include "ros_robo_bob/PerformWhiteBalance.h"
#include "ros_robo_bob/GetRGBGains.h"
#include "ros_robo_bob/StartStopStreaming.h"
#include <lumenera/lucamapi.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/persistence.hpp>
#include <sys/stat.h>
#include <unistd.h>
#include <libgen.h>
#include <string.h>
#include <pwd.h>
#include <mutex>
#include <boost/filesystem.hpp>
#include "../include/bob_definitions.hpp"
#include "../include/bob_status.hpp"
#include "../include/utils.hpp"
#include "../include/auto_white_balance.hpp"
#include <cv_bridge/cv_bridge.h>
#include "../include/image_proc.hpp"
#include <omp.h>

#define TARGET_BRIGHTNESS       75
#define LOOP_AND_FRAME_RATE     30.0
#define CHAR_BUFFER_SIZE        100
#define FRAME_INTERVAL          30.0
#define MAX_RETRIES             3
#define WB_COUNT                1
#define MAX_EXPOSURE            20.0
#define SNAPSHOT_TIMEOUT        1000.0
#define STROBE_DELAY            10.0
#define EXPOSURE_DELAY          0.0
#define GAIN_RED                1.5
#define GAIN_GREEN              1.0
#define GAIN_BLUE               3.0
#define STABILIZE_COUNT         2
#define DISCARD_IMAGES          5
//#define AUTO_WB
static const std::string cameraName = "lumenera_camera";
static const std::string calibrationFilePath =
        "../../../src/ros_robo_bob/config/calibration_params.yaml";
static const std::string whiteBalanceFilePath =
        "../../../src/ros_robo_bob/config/white_balance_params.yaml";

//
// data structure to hold image acquisition parameters
//
typedef struct {
    HANDLE hCamera;
    LONG singleFrame;
    std::vector<unsigned char> *rawImageData;
    std::vector<unsigned char> *rgbImageData;
    sensor_msgs::CameraInfo *cameraInfo;
    sensor_msgs::Image *image;
    ULONG imageHeight;
    ULONG imageWidth;
    ULONG pixelFormat;
}imageData;

static ULONG getPixelSize(const int pixelFormat)
{
    switch(pixelFormat)
    {
        case LUCAM_PF_8:	return  8/8;
        case LUCAM_PF_16:	return 16/8;
        case LUCAM_PF_24:	return 24/8;
        case LUCAM_PF_32:	return 32/8;
        case LUCAM_PF_48:	return 48/8;
    }
    return 0;
}

//
// Helper function to get the RGB gain values
//
bool getRGBGainValues(HANDLE hCamera, float *red, float *green, float *blue)
{
    LONG flags;
    if (!LucamGetProperty(hCamera, LUCAM_PROP_GAIN_RED, red, &flags)) {
        ROS_ERROR_STREAM("error getting red gain...");
        return false;
    }
    if (!LucamGetProperty(hCamera, LUCAM_PROP_GAIN_GREEN1, green, &flags)) {
        ROS_ERROR_STREAM("error getting green1 gain...");
        return false;
    }
    if (!LucamGetProperty(hCamera, LUCAM_PROP_GAIN_BLUE, blue, &flags)) {
        ROS_ERROR_STREAM("error setting blue gain...");
        return false;
    }

    return true;
}

//
// Helper function to set the RGB gain values
//
bool setRGBGainValues(HANDLE hCamera, float red, float green, float blue)
{
    LONG flags;
    if (!LucamSetProperty(hCamera, LUCAM_PROP_GAIN_RED, red, flags)) {
        ROS_ERROR_STREAM("error setting red gain...");
        return false;
    }
    if (!LucamSetProperty(hCamera, LUCAM_PROP_GAIN_GREEN1, green, flags)) {
        ROS_ERROR_STREAM("error setting green1 gain...");
        return false;
    }
    if (!LucamSetProperty(hCamera, LUCAM_PROP_GAIN_GREEN2, green, flags)) {
        ROS_ERROR_STREAM("error setting green2 gain...");
        return false;
    }
    if (!LucamSetProperty(hCamera, LUCAM_PROP_GAIN_BLUE, blue, flags)) {
        ROS_ERROR_STREAM("error setting blue gain...");
        return false;
    }

    return true;
}

//
// helper function to persist the RGB gain values to a file
//
bool saveRGBGainValues(float red, float green, float blue)
{
    cv::FileStorage fs;
    //
    // open white balance value storage file for writing
    //
    fs.open(whiteBalanceFilePath, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        ROS_ERROR_STREAM("failed to open " << whiteBalanceFilePath);
        fs.release();
        return false;
    }
    else {
        //
        // insert the name value pairs into the file
        //
        fs << "red" << red
           << "green" << green
           << "blue" << blue;
        fs.release();
        return true;
    }
}

//
// helper function to read the RGB gain values from a file
//
bool readRGBGainValues(float *red, float *green, float *blue)
{
    cv::FileStorage fs;
    //
    // open white balance value storage file for writing
    //
    fs.open(whiteBalanceFilePath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR_STREAM("failed to open " << whiteBalanceFilePath);
        fs.release();
        return false;
    }
    else {
        //
        // insert the name value pairs into the file
        //
        *red = fs["red"];
        *green = fs["green"];
        *blue = fs["blue"];
        fs.release();
        return true;
    }
}

//
// grab a snapshot from the image stream
// update the image pixel data and time stamp
//
int grabASnapshot(HANDLE hCamera,
                  LONG frameCount,
                  std::vector<unsigned char> *rawImageData,
                  std::vector<unsigned char> *rgbImageData,
                  sensor_msgs::CameraInfo *cameraInfo,
                  sensor_msgs::Image *image,
                  ULONG imageHeight,
                  ULONG imageWidth,
                  ULONG pixelFormat)
{
    LUCAM_SNAPSHOT snapshotSettings;
    //
    // get the exposure
    //
    float exposure = 0.0;
    LONG flags = 0;
    if(LucamGetProperty(hCamera,
                        LUCAM_PROP_EXPOSURE,
                        &exposure,
                        &flags)) {
    }
    else {
        ROS_ERROR_STREAM("failed to get exposure...");
    }
    snapshotSettings.exposure = exposure;
    //
    // get gain
    //
    float gain = 0.0;
    flags = 0;
    if(LucamGetProperty(hCamera,
                        LUCAM_PROP_GAIN,
                        &gain,
                        &flags)) {
    }
    else {
        ROS_ERROR_STREAM("failed to get gain...");
    }
    snapshotSettings.gain = gain;
    float gainRed = 0.0;
    float gainBlue = 0.0;
    float gainGrn1 = 0.0;
    if (getRGBGainValues(hCamera, &gainRed, &gainGrn1, &gainBlue)) {
        snapshotSettings.gainRed = gainRed;
        snapshotSettings.gainBlue = gainBlue;
        snapshotSettings.gainGrn1 = gainGrn1;
        snapshotSettings.gainGrn2 = gainGrn1;
    }
    else {
        snapshotSettings.gainRed = GAIN_RED;
        snapshotSettings.gainBlue = GAIN_BLUE;
        snapshotSettings.gainGrn1 = GAIN_GREEN;
        snapshotSettings.gainGrn2 = GAIN_GREEN;
    }
    //
    // get the frame format
    //
    LUCAM_FRAME_FORMAT frameFormat;
    float frameRate;
    LucamGetFormat(hCamera, &frameFormat, &frameRate);
    snapshotSettings.format = frameFormat;
    snapshotSettings.useHwTrigger = FALSE;
    snapshotSettings.timeout = SNAPSHOT_TIMEOUT;
    snapshotSettings.strobeFlags = LUCAM_PROP_FLAG_USE;
    snapshotSettings.strobeDelay = STROBE_DELAY;
    snapshotSettings.exposureDelay = EXPOSURE_DELAY;
    snapshotSettings.shutterType = LUCAM_SHUTTER_TYPE_GLOBAL;
    snapshotSettings.bufferlastframe = FALSE;
    //
    // set reserved values to zero
    //
    snapshotSettings.ulReserved2 = 0;
    snapshotSettings.flReserved1 = 0.0;
    snapshotSettings.flReserved2 = 0.0;

    if(FALSE == LucamTakeSnapshot(hCamera,
                                  &snapshotSettings,
                                  (BYTE *)rawImageData->data())) {
        ROS_ERROR_STREAM("take snapshot failed...");
        return BOB_IMAGE_CAPTURE_FAILED;
    }
    //
    // convert bayer encoded raw image to rgb
    //
    LUCAM_CONVERSION convParams;
    convParams.CorrectionMatrix = LUCAM_CM_FLUORESCENT;
    convParams.DemosaicMethod = LUCAM_DM_HIGHER_QUALITY;
    if(!LucamConvertFrameToRgb24(hCamera,
                                 (BYTE *)rgbImageData->data(),
                                 (BYTE *)rawImageData->data(),
                                 imageWidth,
                                 imageHeight,
                                 pixelFormat,
                                 &convParams
                                 )) {
        ROS_ERROR_STREAM("error converting image...");
    }
    //
    // update camera info time stamp
    //
    cameraInfo->header.stamp = ros::Time::now();
    //
    // configure the image message
    //
    image->header.stamp = cameraInfo->header.stamp;
    image->data = *rgbImageData;
    image->height = imageHeight;
    image->width = imageWidth;
    image->step = getPixelSize(LUCAM_PF_24) * imageWidth;
    image->encoding = sensor_msgs::image_encodings::RGB8;

    return BOB_SUCCESS;
}

//
// service callback used to set camera calibration parameters and save
// the values to a file
//
bool setAndSaveCameraCalibrationData(sensor_msgs::SetCameraInfo::Request &req,
                                     sensor_msgs::SetCameraInfo::Response &resp,
                                     sensor_msgs::CameraInfo *cameraInfo,
                                     ULONG *imageHeight,
                                     ULONG *imageWidth)
{
    //
    // set the camera info
    //
    cameraInfo->header.stamp = ros::Time::now();
    cameraInfo->height = *imageHeight;
    cameraInfo->width = *imageWidth;
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
// not found
//
// default values are all set to zero
//
bool initializeCameraCalibrationData(sensor_msgs::CameraInfo cameraInfo,
                                     const ULONG imageHeight,
                                     const ULONG imageWidth)
{
    //
    // read the data
    //
    if(camera_calibration_parsers::readCalibration(
                calibrationFilePath,
                (std::string&)cameraName,
                cameraInfo)) {
        ROS_INFO_STREAM("calibration file read...");
        return EXIT_SUCCESS;
    }
    else {
        //
        // set the camera info to default values
        //
        cameraInfo.header.stamp = ros::Time::now();
        cameraInfo.height = imageHeight;
        cameraInfo.width = imageWidth;
        cameraInfo.distortion_model = "plumb_bob";
        cameraInfo.binning_x = 0;
        cameraInfo.binning_y = 0;
        sensor_msgs::RegionOfInterest roi;
        roi.height = imageHeight;
        roi.width = imageWidth;
        roi.x_offset = 0;
        roi.y_offset = 0;
        cameraInfo.roi = roi;
        //
        // write data to yaml file
        //
        bool retValue = camera_calibration_parsers::writeCalibration(
                    calibrationFilePath,
                    cameraName,
                    cameraInfo);
        if(!retValue) {
            return EXIT_FAILURE;
        }
        else {
            return EXIT_SUCCESS;
        }
    }
}

//
// service to retrieve an image for saving to a file
//
bool saveImageCB(ros_robo_bob::SaveImageRequest& request,
                 ros_robo_bob::SaveImageResponse& response,
                 imageData *data)
{
    cv::Mat gain;

    //
    // check for NULL
    // set status to error in image == NULL
    //
    if (NULL != data->hCamera)
    {
        if (BOB_SUCCESS == grabASnapshot(data->hCamera,
                                         data->singleFrame,
                                         data->rawImageData,
                                         data->rgbImageData,
                                         data->cameraInfo,
                                         data->image,
                                         data->imageHeight,
                                         data->imageWidth,
                                         data->pixelFormat))
        {

            cv_bridge::CvImagePtr cv_ptr;
            response.image = *(data->image);

            // Convert the ROS image message to an OpenCV one
            cv_ptr = cv_bridge::toCvCopy(response.image, sensor_msgs::image_encodings::BGR8);
            
            #ifdef AUTO_WB
                
                // Dynamically perform auto wb, or just used the saved gain file
                if(request.autoWB)
                {
                    if(!processImage(cv_ptr->image, gain, 3))
                        gain = loadXYZGain(XYZ_GAIN_FILE_PATH);
                }
                else
                {
                    gain = loadXYZGain(XYZ_GAIN_FILE_PATH);
                } 

                // Correct the image with the comptured white balance gains
                cv_ptr->image = whiteBalanceImage(cv_ptr->image, gain);

                // Populate the image with the processsed one
                response.image = *(cv_ptr->toImageMsg());
                
            #endif
                
            // Check to see if the image is dark
            if(isImageBlurry(cv_ptr->image))
                response.status = BOB_BLURRY_IMAGE;
            else
                response.status = BOB_SUCCESS;
        }
        else {
            response.image = *(data->image);
            response.status = BOB_IMAGE_CAPTURE_FAILED;
        }
    }
    else{
        response.image = *(data->image);
        response.status = BOB_IMAGE_CAPTURE_FAILED;
    }

    return true;
}

//
// service used to change target brightness
//
bool setTargetBrightnessCB(ros_robo_bob::SetTargetBrightnessRequest& request,
                         ros_robo_bob::SetTargetBrightnessResponse& response,
                         UCHAR *brightness)
{
    *brightness = (ULONG)request.brightness;
    response.status = BOB_SUCCESS;

    return true;
}

//
// service used to retrieve the RGB gain values
//
bool getRGBGainsCB(ros_robo_bob::GetRGBGainsRequest& request,
                         ros_robo_bob::GetRGBGainsResponse& response,
                         HANDLE *pHCamera)
{
    //
    // first get red gain
    //
    float pRed;
    LONG pFlags;
    ROS_ERROR_STREAM("calling LucamGetProperty...");
    if(LucamGetProperty(*pHCamera,
                        LUCAM_PROP_GAIN_RED,
                        &pRed,
                        &pFlags)) {
        response.red = pRed;
        ROS_ERROR_STREAM("red gain = " << response.red);
        response.status = BOB_SUCCESS;
    }
    else {
        response.status = BOB_WHITE_BALANCE_FAILED;
    }
    //
    // next get green1 gain
    //
    float pGreen1;
    if(LucamGetProperty(*pHCamera,
                        LUCAM_PROP_GAIN_GREEN1,
                        &pGreen1,
                        &pFlags)) {
        response.green = pGreen1;
        ROS_ERROR_STREAM("green gain = " << response.green);
        response.status = BOB_SUCCESS;
    }
    else {
        response.status = BOB_WHITE_BALANCE_FAILED;
    }
    //
    // next get green2 gain
    //
    float pGreen2;
    if(LucamGetProperty(*pHCamera,
                        LUCAM_PROP_GAIN_GREEN1,
                        &pGreen2,
                        &pFlags)) {
        response.green = pGreen2;
        ROS_ERROR_STREAM("green gain1 = " << response.green);
        response.status = BOB_SUCCESS;
    }
    else {
        response.status = BOB_WHITE_BALANCE_FAILED;
    }
    //
    // last, get blue gain
    //
    float pBlue;
    if(LucamGetProperty(*pHCamera,
                        LUCAM_PROP_GAIN_BLUE,
                        &pBlue,
                        &pFlags)) {
        response.blue = pBlue;
        ROS_ERROR_STREAM("blue gain = " << response.blue);
        response.status = BOB_SUCCESS;
    }
    else {
        response.status = BOB_WHITE_BALANCE_FAILED;
    }

    return true;
}



//
// service used to initiate white balancing
//
bool performWhiteBalanceCB(ros_robo_bob::PerformWhiteBalanceRequest& request,
                         ros_robo_bob::PerformWhiteBalanceResponse& response,
                         bool *enable)
{
    *enable = request.enable;
    response.status = BOB_SUCCESS;

    return true;
}

//
// start the video stream, NULL window handle
//
int startStreaming(HANDLE hCamera)
{
    if (LucamStreamVideoControl(hCamera, START_STREAMING, NULL) == FALSE) {
       ROS_ERROR_STREAM("Failed to start streaming");
       return false;
    }
    else {
        return true;
    }
}

//
// stop the video stream, NULL window handle
//
int stopStreaming(HANDLE hCamera)
{
    if (LucamStreamVideoControl(hCamera, STOP_STREAMING, NULL) == FALSE) {
       ROS_ERROR_STREAM("Failed to stop streaming");
       return false;
    }
    else {
        return true;
    }
}

//
// service used to start and stop the video stream
//
bool startStopStreamingCB(ros_robo_bob::StartStopStreamingRequest& request,
                          ros_robo_bob::StartStopStreamingResponse& response,
                          bool *streaming,
                          HANDLE *hCamera)

{
    *streaming = request.streaming;
    if (*streaming) {
        ROS_ERROR_STREAM("streaming...");
        startStreaming(*hCamera);
    }
    else {
        ROS_ERROR_STREAM("not streaming...");
        stopStreaming(*hCamera);
    }

    response.status = BOB_SUCCESS;

    return true;
}

//
// initialize the camera parameters, after first obtaining a camera handle
//
int configureCamera(HANDLE& hCamera,
                    ULONG& imageWidth,
                    ULONG& imageHeight,
                    float& frameRate,
                    ULONG& pixelFormat)
{
    //
    // get the number of cameras
    //
    LONG numCameras = LucamNumCameras();
    ROS_INFO_STREAM("number of cameras: " << numCameras);
    //
    // get the frame format
    //
    LUCAM_FRAME_FORMAT frameFormat;
    LucamGetFormat(hCamera, &frameFormat, &frameRate);
    imageWidth = (frameFormat.width / frameFormat.subSampleX);
    imageHeight = (frameFormat.height / frameFormat.subSampleY);
    pixelFormat = frameFormat.pixelFormat;
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
    //
    // initialize camera calibration data
    //
    sensor_msgs::CameraInfo cameraInfo;
    if(EXIT_FAILURE == initializeCameraCalibrationData(cameraInfo,
                                                       imageHeight,
                                                       imageWidth)) {
        ROS_ERROR_STREAM("calibration data initialization failed...");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


#endif // CAMERA_HEADER_HP
