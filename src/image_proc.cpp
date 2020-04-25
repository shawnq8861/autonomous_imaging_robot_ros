/* General c++ includes */
#include <math.h>
#include <string>
#include <stdlib.h>

/* ROS includes */
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/* Package specific includes */
#include "../include/image_proc.hpp"

// The factor to dilate brightness searching by
#define BRIGHTNESS_SEARCH_DILATION_PX 1

// The brightness threshold for an image to be considered black
#define DARK_IMAGE_THRESH 30

// The minimum variance of the laplacian operator in order for an image to be
// considered blurry
#define MIN_LAPLACIAN_VAR 300.

// Size of the focus kernel to apply (n x n)
#define FOCUS_KERNEL_SIZE 300

// The minimum amount of focused kernels required to consider an image in focus
#define MIN_FOCUSED_KERNELS 0

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @image	          The image to analyze			  	    #
#   Outputs:									    #
#	    @returns		  True if the image is "dark"			    #
#                                                                                   #
************************************************************************************/

bool isImageDark(cv::Mat image)
{
    cv::Mat hsvImage;

    // The average value of the value channel for the image
    int valueChannelMean = 0;

    // Used to keep track of the number of pixels we've searched
    int searchedPixels = 0;

    // Convert the BGR image to HSV
    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

    // Sum the value channel of every pixel
    for(int r = 0; r < hsvImage.rows; r += BRIGHTNESS_SEARCH_DILATION_PX)
    {
        for(int c = 0; c < hsvImage.cols; c += BRIGHTNESS_SEARCH_DILATION_PX)
	{
            searchedPixels += 1;
            valueChannelMean += hsvImage.at<cv::Vec3b>(r, c)[2];
	}
    }

    // Compute the average
    valueChannelMean /= searchedPixels;

    return valueChannelMean < DARK_IMAGE_THRESH;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @image                The image to analyze                              #
#   Outputs:                                                                        #
#           @returns              True if the image is "blurry"                     #
#										    #
#   Description: Uses the vairance of the flattened laplacian tranform on	    #
#                non-overlapping kernels of the image. Using a kernelized approach  #
#                allows for parts of the image to be featureless (ie. concrete)     #
#                while still being able to detection focused regions.               #
#                                                                                   #
************************************************************************************/

bool isImageBlurry(cv::Mat image)
{
    // Working images
    cv::Mat grayImage, laplacian, absLaplacian;

    // The number of focused kernels detected
    int focusedKernels = 0;

    // Blur the image to reduce any noise
    cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);

    // Compute the number of kernel steps required in each axis
    int xSteps = image.cols / FOCUS_KERNEL_SIZE;
    int ySteps = image.rows / FOCUS_KERNEL_SIZE;

    for (int x = 0; x < xSteps; x++)
    {
        for (int y = 0; y < ySteps; y++)
        {
            // The mean of the laplacian operator output
            float laplacianMean = 0;

            // Sum of squared errors from the mean
            float sse = 0;

            // Obtain the kernel slice
            cv::Mat kernel = grayImage(cv::Rect(x * FOCUS_KERNEL_SIZE, y * FOCUS_KERNEL_SIZE, FOCUS_KERNEL_SIZE, FOCUS_KERNEL_SIZE));

            // Apply the laplacian operator to the image
            cv::Laplacian(kernel, laplacian, CV_16S, 3, 1, 0, cv::BORDER_DEFAULT);

            // Scale the output back to CV_8U
            cv::convertScaleAbs(laplacian, absLaplacian);

            // Compute the mean of the laplacian output
            for (int r = 0; r < kernel.rows; r++)
                for (int c = 0; c < kernel.cols; c++)
                    laplacianMean += (float)absLaplacian.at<unsigned char>(r, c);

            laplacianMean /= (float)(kernel.rows * kernel.cols);

            // Compute the sum of squared errors from the mean
            for (int r = 0; r < kernel.rows; r++)
                for (int c = 0; c < kernel.cols; c++)
                    sse += pow((float)absLaplacian.at<unsigned char>(r, c) - laplacianMean, 2);

            // Compute the variance of the laplacian operator
            float laplacianVar = sse / (float)((kernel.rows * kernel.cols) - 1);
            
            if(laplacianVar > MIN_LAPLACIAN_VAR)
                focusedKernels++;
            
        }
    }

    return focusedKernels < MIN_FOCUSED_KERNELS;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @captureResult        The result message from the capture_image action  #
#           @plan                 Pointer to the plan object                        #
#                                                                                   #
#   Description: Attaches metadata to a capture result and saves the image.         #
#                                                                                   #
************************************************************************************/

void processImage(ros_robo_bob::CaptureImageResult &captureResult, ros_robo_bob::ImageMeta &imageMeta, TrackPlan *plan,
                      int gimbalPos, boost::array<float, 3ul> gimbalOrientation, float yPosition)
{
    rosbag::Bag bag;
    char hostname[HOST_NAME_MAX];
    std::string date = date2String();
    double lat, lon;
    
    // Obtain the name of the host running this code (ie. arc199)
    gethostname(hostname, HOST_NAME_MAX);

    // Copy over plan data into image metadata
    imageMeta.edgeAPI = plan->getCustomer()->edgeAPI;
    imageMeta.customerName = plan->getCustomer()->name;
    imageMeta.facility = plan->getLocation()->facility;
    imageMeta.space = plan->getLocation()->space;
    imageMeta.location[0] = plan->getLocation()->gridOffset[0] + gimbalPos;
    imageMeta.absolutePosition[0] = plan->getLocation()->absolutePosition[0];
    
    // Robot can start at either ends of the Y axis
    if (plan->getStartWorking() < plan->getEndWorking())
    {
        imageMeta.absolutePosition[1] = yPosition + plan->getLocation()->absolutePosition[1];
        imageMeta.location[1] = plan->getLocation()->gridOffset[1] + plan->getStopIteration();
    }
    else
    {
        imageMeta.absolutePosition[1] = plan->getLocation()->absolutePosition[1] - yPosition;
        imageMeta.location[1] = plan->getLocation()->gridOffset[1] - plan->getStopIteration();
    }

    // Update the WGS84 coordinates
    transformWGS84(plan->getLocation()->latitude, plan->getLocation()->longitude, lat, lon,
                   plan->getLocation()->heading, yPosition*0.0254);

    imageMeta.latitude = lat;
    imageMeta.longitude = lon;

    // Populate the camera info
    imageMeta.make = plan->getCameraInfo()->make;
    imageMeta.model = plan->getCameraInfo()->model;
    imageMeta.focalLength = plan->getCameraInfo()->focalLength;
    
    // Store the orientation of the camera for this image
    imageMeta.orientation = gimbalOrientation;
    imageMeta.orientation[1] = 90;
    imageMeta.orientation[2] = plan->getLocation()->heading;

    // Add the robot's hostname to the image
    imageMeta.hostname = std::string(hostname) + "-" + std::to_string(imageMeta.location[0]) 
                                   + "-" + std::to_string(imageMeta.location[1]);
    
    // Add the ISO timestamp to the image
    imageMeta.timestamp = epoch2ISOTime(date);

    // Create the filename (<hostname>_<x>_<y>_<epoch>)
    std::string filename = std::string(IMAGES_PATH) + "/" + std::string(imageMeta.hostname)
                           + "-" + date + ".jpg";

    // Save the local path to the image in the metadata
    imageMeta.localPath = filename;

    // Convert the image to OpenCV format
    cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(captureResult.image, sensor_msgs::image_encodings::BGR8);
    
    // Save and compress the image
    cv::imwrite(filename.c_str(), cvPtr->image);
}
