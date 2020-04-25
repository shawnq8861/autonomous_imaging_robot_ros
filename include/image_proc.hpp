#ifndef ROS_ROBO_BOB_IMAGE_PROC_HPP
#define ROS_ROBO_BOB_IMAGE_PROC_HPP

/* General C++ includes */
#include <boost/array.hpp>

/* OpenCV includes */
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

/* Package specific includes */
#include "../include/utils.hpp"
#include "../include/bob_status.hpp"
#include "../include/track_plan.hpp"
#include "../include/image_proc.hpp"

/* Action includes */
#include <ros_robo_bob/CaptureImageAction.h>

/* Message incldues */
#include <ros_robo_bob/ImageMeta.h>

/* Function declarations */
bool isImageDark(cv::Mat);
bool isImageBlurry(cv::Mat);
void processImage(ros_robo_bob::CaptureImageResult&, ros_robo_bob::ImageMeta&, TrackPlan*, int,
                      boost::array<float, 3ul>, float);

#endif
