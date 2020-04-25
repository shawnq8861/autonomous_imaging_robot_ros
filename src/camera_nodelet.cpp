#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include "../include/camera_header.hpp"

namespace ros_robo_bob_camera
{

class CameraAcquireNodelet : public nodelet::Nodelet
{
public:
    CameraAcquireNodelet()
    {
        targetBrightness = TARGET_BRIGHTNESS;
        data.hCamera = NULL;
        performWB = false;
        saveImage = false;
        startUpCount = 0;
        numDiscardImages = DISCARD_IMAGES;
    }

private:
    virtual void onInit()
    {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        //
        // create the raw_image topic to publish image data to
        //
        image_pub = private_nh.advertise<sensor_msgs::Image>("image_raw", 10);
        //
        // instantiate a publisher for camera calibration info
        //
        camera_info_pub =
                private_nh.advertise<sensor_msgs::CameraInfo>("camera_info", 10);
        //
        // attempt to get a camera handle the first time
        //
        data.hCamera = LucamCameraOpen(1);
        if (NULL == data.hCamera) {
            ROS_ERROR_STREAM("ERROR %u: Unable to open camera.\n"
                             << (unsigned int)LucamGetLastError());
        }
        else {
            //
            // configure the camera
            //
            configureCamera(data.hCamera,
                            data.imageWidth,
                            data.imageHeight,
                            frameRate,
                            data.pixelFormat);
            //
            // set size of raw data vector
            //
            rawImageData.resize(data.imageHeight * data.imageWidth, 0);
            rgbImageData.resize(getPixelSize(LUCAM_PF_24) * data.imageHeight * data.imageWidth, 0);
            data.rawImageData = &rawImageData;
            data.rgbImageData = &rgbImageData;
            data.image = &image;
            data.cameraInfo = &cameraInfo;
            data.singleFrame = 1;
            float duration = 1.0 / FRAME_INTERVAL;
            //
            // check for white balance file
            // if file found, open, read settings and write to the camera
            //
            float red = 0.0;
            float green = 0.0;
            float blue = 0.0;
            if (readRGBGainValues(&red, &green, &blue)) {
                setRGBGainValues(data.hCamera, red, green, blue);
            }
            else {
                ROS_ERROR_STREAM("wb file not found...");
            }

            //
            // get the max exposure value
            //
            float exposure;
            LONG flags;
            if(LucamGetProperty(data.hCamera,
                                LUCAM_PROP_AUTO_EXP_MAXIMUM,
                                &exposure,
                                &flags)) {
            }
            else {
                ROS_ERROR_STREAM("failed to get current max exposure...");
            }
            //
            // set the max exposure value
            //
            exposure = (float)MAX_EXPOSURE;
            if (!LucamSetProperty(data.hCamera,
                                  LUCAM_PROP_AUTO_EXP_MAXIMUM,
                                  exposure,
                                  flags)) {
                ROS_ERROR_STREAM("error setting max exposure...");
            }
            //
            // verify max exposure value
            //
            if(LucamGetProperty(data.hCamera,
                                LUCAM_PROP_AUTO_EXP_MAXIMUM,
                                &exposure,
                                &flags)) {
            }
            else {
                ROS_ERROR_STREAM("failed to get adjusted max exposure...");
            }
            //
            // create the timer callback in which the images will be published
            //
            timer = private_nh.createTimer(ros::Duration(duration),
                                           boost::bind(&CameraAcquireNodelet::timerCB,
                                                       this, _1) );
            //
            // instantiate a service to be called to save an image to file
            //
            save_image_service = private_nh.advertiseService
                    <ros_robo_bob::SaveImageRequest,
                    ros_robo_bob::SaveImageResponse>(
                        "save_image",
                        boost::bind(saveImageCB, _1, _2, &data));
            //
            // initialize camera calibration data
            //
            if(EXIT_FAILURE == initializeCameraCalibrationData(cameraInfo,
                                                               data.imageHeight,
                                                               data.imageWidth)) {
                NODELET_ERROR_STREAM("calibration data initialization failed...");
            }
            //
            // instantiate a service to be called after the camera is calibrated
            //
            calibration_service = private_nh.advertiseService<sensor_msgs::SetCameraInfoRequest,
                    sensor_msgs::SetCameraInfoResponse>(
                        "set_camera_info",
                        boost::bind(setAndSaveCameraCalibrationData,
                                    _1, _2, &cameraInfo,
                                    &data.imageHeight, &data.imageWidth));
            //
            // instantiate a service to be called to set target brightness
            //
            set_target_brightness = private_nh.advertiseService
                    <ros_robo_bob::SetTargetBrightnessRequest,
                    ros_robo_bob::SetTargetBrightnessResponse>(
                        "set_target_brightness",
                        boost::bind(setTargetBrightnessCB,
                                    _1, _2, &targetBrightness));
            //
            // instantiate a service to allow peforming white balancing
            //
            perform_white_balance = private_nh.advertiseService
                    <ros_robo_bob::PerformWhiteBalanceRequest,
                    ros_robo_bob::PerformWhiteBalanceResponse>(
                        "perform_white_balance",
                        boost::bind(performWhiteBalanceCB,
                                    _1, _2, &performWB));
            //
            // instantiate a service to allow getting RGB gains
            //
            get_rgb_gains = private_nh.advertiseService
                    <ros_robo_bob::GetRGBGainsRequest,
                    ros_robo_bob::GetRGBGainsResponse>(
                        "get_rgb_gains",
                        boost::bind(getRGBGainsCB,
                                    _1, _2, &data.hCamera));
        }
        //
        // initialize start up variables
        //
        startUpCount = 0;
        numDiscardImages = DISCARD_IMAGES;
    }

    //
    // this takes the place of the while loop in the non-nodelet version
    //
    void timerCB(const ros::TimerEvent& timer_event) {
        //
        // check for camera connected, try to re-acquire handle if needed
        //
        static int num_retries = 0;
        int num_cameras = LucamNumCameras();
        if (num_cameras < 1) {
            //
            // no cameras connected
            //
            // attempt to get a camera handle
            //
            if (num_retries < MAX_RETRIES) {
                //
                // attempt to get a camera handle the first time
                //
                data.hCamera = LucamCameraOpen(1);
                if (NULL == data.hCamera) {
                    ROS_ERROR_STREAM("ERROR %u: Unable to open camera.\n"
                                     << (unsigned int)LucamGetLastError());
                    //
                    // increment the retry counter
                    //
                    ++num_retries;
                }
                else {
                    //
                    // configure the camera
                    //
                    configureCamera(data.hCamera,
                                    data.imageWidth,
                                    data.imageHeight,
                                    frameRate,
                                    data.pixelFormat);
                }
            }
            else {
                ROS_ERROR_STREAM(
                            "camera handle retry failed, check camera cable...\n");
            }
        }
        else {
            //
            // found a camera connected
            // check for valid handle, re-acquire if needed
            //
            if (NULL == data.hCamera) {
                data.hCamera = LucamCameraOpen(1);
                if (NULL == data.hCamera) {
                    ROS_ERROR_STREAM("ERROR %u: Unable to open camera.\n"
                                     << (unsigned int)LucamGetLastError());
                }
                else {
                    //
                    // configure the camera
                    //
                    configureCamera(data.hCamera,
                                    data.imageWidth,
                                    data.imageHeight,
                                    frameRate,
                                    data.pixelFormat);
                    //
                    // reset retry counter to 0
                    //
                    num_retries = 0;
                }
            }
            if (NULL != data.hCamera) {
                //
                // publish the raw image data
                //
                // set one shot auto exposure target
                //
                ULONG startX = 0;
                ULONG startY = 0;
                //
                // reject images to reduce intenisty variation
                //
                for (int i = 0; i < STABILIZE_COUNT; ++i) {
                    //
                    // turn the stream on
                    //
                    if (LucamStreamVideoControl(data.hCamera, START_STREAMING, NULL)
                            == FALSE) {
                        ROS_ERROR_STREAM("Failed to start streaming");
                    }
                    //
                    // set auto exposure
                    //
                    if (LucamOneShotAutoExposure(data.hCamera,
                                                 targetBrightness,
                                                 startX,
                                                 startY,
                                                 data.imageWidth,
                                                 data.imageHeight)) {
                    }
                    else {
                        ROS_ERROR_STREAM("one shot auto exposure failed...");
                    }
                    //
                    // perform auto white balance
                    //
                    if (performWB) {
                        LucamOneShotAutoWhiteBalance(data.hCamera,
                                                     startX,
                                                     startY,
                                                     data.imageWidth,
                                                     data.imageHeight);
                        float red = 0.0;
                        float green = 0.0;
                        float blue = 0.0;
                        if (getRGBGainValues(data.hCamera, &red, &green, &blue)) {
                            if (setRGBGainValues(data.hCamera, red, green, blue)) {
                                if (saveRGBGainValues(red, green, blue)) {
                                }
                                else {
                                    ROS_ERROR_STREAM("error saving RGB values...");
                                }
                            }
                            else {
                                ROS_ERROR_STREAM("could not set RGB values...");
                            }
                        }
                        else {
                            ROS_ERROR_STREAM("could not get RGB values...");
                        }
                        performWB = false;
                    }
                    //
                    // turn stream back off
                    //
                    if (LucamStreamVideoControl(data.hCamera, STOP_STREAMING, NULL) == FALSE) {
                        ROS_ERROR_STREAM("Failed to stop streaming");
                    }
                }
            }
        }
    }
    //
    // declare member elements
    //
    ros::Publisher image_pub;
    ros::Publisher camera_info_pub;
    ros::Timer timer;
    ros::ServiceServer save_image_service;
    ros::ServiceServer calibration_service;
    ros::ServiceServer set_target_brightness;
    ros::ServiceServer perform_white_balance;
    ros::ServiceServer get_rgb_gains;
    //
    // camera parameters
    //
    UCHAR targetBrightness;
    float frameRate;
    sensor_msgs::CameraInfo cameraInfo;
    bool performWB;
    bool saveImage;
    unsigned char startUpCount;
    unsigned char numDiscardImages;
    //
    // create vector and cv::Mat to hold image data
    //
    std::vector<unsigned char> rawImageData;
    std::vector<unsigned char> rgbImageData;
    sensor_msgs::Image image;
    imageData data;
};

} // namespace ros_robo_bob_camera

PLUGINLIB_EXPORT_CLASS(ros_robo_bob_camera::CameraAcquireNodelet, nodelet::Nodelet)
