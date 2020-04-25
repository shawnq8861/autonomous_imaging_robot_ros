/* General C++ includes */
#include <omp.h>
#include <math.h>
#include <climits>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <yaml-cpp/yaml.h>

/* BOOST includes */
#include <boost/filesystem.hpp>

/* Package specific includes */
#include "../include/bob_definitions.hpp"
#include "../include/auto_white_balance.hpp"

// Threshold required to consider a pixel gray
#define GRAY_THRESH 0.4

// Maximum allowed iterations of the CAT corrections
#define MAX_CAT_ITERATIONS 100

// Minimum amount of grey pixels required to process the image
#define GRAY_PIXEL_POP_THRESH 100

// Gain window sized used for computing the gain variance
#define GAIN_HISTORY_SIZE 4

// Variance threshold required to consider the algorithm converged
#define GAIN_VAR_THRESHOLD 2.0e-10

// The target pixel width of the image during processing
#define TARGET_PROCCESSING_WIDTH_PX 500

// The default number of threads to use
#define DEFAULT_NUM_THREADS 1

// Used to convert from RGB to XYZ and back
const float SRGB2YXZ[3][3] = {{0.4124, 0.3575, 0.1804},
                              {0.2126, 0.7151, 0.0721},
                              {0.0193, 0.1191, 0.9503}};

// Novel chromatic adaptation transofrmation (CAT) from S. Bianco, R. Schettini et. al,
// found usinging particle swarm opimization
const float MBSCAT[3][3] = {{0.8752, 0.2787, -0.1539},
                            {-0.8904, 1.8709, 0.0195},
                            {-0.0061, 0.0162, 0.9899}};

// The D65 CIE standard illuminant
const float XYZD65[3] = {95.04, 100, 108.8};

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Jason Su                                                #
#   Inputs:                                                                         #
#           @xyz                  XYZ color space matrix to be converted            #
#   Outputs:                                                                        #
#           @returns              XY color space matrix                             #
#                                                                                   #
#   Description: Converts a matrix from the CIELAB 1931 XYZ color space to a        #
#                compressed XY color space.                                         #
#                                                                                   #
************************************************************************************/

cv::Mat XYZ2xy(cv::Mat xyz)
{
    cv::Mat xy = cv::Mat(xyz.rows, xyz.cols, CV_32FC2);

    for (int r = 0; r < xyz.rows; r++)
    {
        // Sum the X, Y, and Z components
        for (int c = 0; c < xyz.cols; c++)
        {
            float channelSum = 0;

            for (int i = 0; i < 3; i++)
                channelSum += xyz.at<cv::Vec3f>(r, c)[i];

            xy.at<cv::Vec2f>(r, c)[0] = xyz.at<cv::Vec3f>(r, c)[0] / channelSum;
            xy.at<cv::Vec2f>(r, c)[1] = xyz.at<cv::Vec3f>(r, c)[1] / channelSum;
        }
    }

    return xy;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Jason Su                                                #
#   Inputs:                                                                         #
#           @xy                   The XY color space matrix to be converted         #
#           @Y                    The desired luminance to normalize the XY         #
#                                 matrix with                                       #
#   Outputs:                                                                        #
#           @returns              XYZ color space matrix                            #
#                                                                                   #
#   Description: Converts a matrix from the CIELAB 1931 XY color space to a         #
#                XYZ color space using a desired luminance value.                   #
#                                                                                   #
************************************************************************************/

cv::Mat xy2XYZ(cv::Mat xy, float Y)
{
    cv::Mat xyz = cv::Mat(xy.rows, xy.cols, CV_32FC3);

    for (int r = 0; r < xy.rows; r++)
    {
        for (int c = 0; c < xy.cols; c++)
        {
            // Obtain the x and y channel values
            float x = xy.at<cv::Vec2f>(r, c)[0];
            float y = xy.at<cv::Vec2f>(r, c)[1];
            float iy = 1. / (y + 1e-6);

            // Construct the XYZ representation given the Y channel
            float X = x * Y * iy;
            float Z = Y * (1. - x - y) * iy;
            xyz.at<cv::Vec3f>(r, c) = cv::Vec3f(X, Y, Z);
        }
    }

    return xyz;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @pixel                A 3-channel pixel vector                          #
#   Outputs:                                                                        #
#           @returns              A matrix representation of the pixel vector       #
#                                                                                   #
************************************************************************************/

cv::Mat cvtPixel2Vector(cv::Vec3f pixel)
{
    cv::Mat vector = cv::Mat(3, 1, CV_32FC1);

    vector.at<float>(0, 0) = pixel[0];
    vector.at<float>(1, 0) = pixel[1];
    vector.at<float>(2, 0) = pixel[2];

    return vector;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Jason Su                                                #
#   Inputs:                                                                         #
#           @yuvfImage            A floating point YUV image                        #
#           @meanU                The mean U value of the gray pixels in the image  #
#           @meanV                The mean V value of the gray pixels in the image  #
#   Outputs:                                                                        #
#           @modifies             meanU by reference                                #
#           @modifies             meanV by reference                                #
#           @returns              True if enough gray pixels were found             #
#                                                                                   #
#   Description: Attempts to find all of the gray pixels in an image, and then      #
#                computes the mean of the U and V channels of the gray pixels.      #
#                                                                                   #
************************************************************************************/

bool calcGrayUVMeans(cv::Mat yuvfImage, float & meanU, float & meanV)
{
    // Used to store the pixels most likely to be
    // considered grey in the image
    std::vector<cv::Vec3f> grayPixels;

    // Segment gray pixels by rows for parallism
    std::vector<std::vector<cv::Vec3f>> grayPixelRows(yuvfImage.rows);

    // Find the gray pixels
    #pragma omp parallel for
    for (int r = 0; r < yuvfImage.rows; r++)
    {
        for (int c = 0; c < yuvfImage.cols; c++)
        {
            float absU = std::abs(yuvfImage.at<cv::Vec3f>(r, c)[1]);
            float absV = std::abs(yuvfImage.at<cv::Vec3f>(r, c)[2]);

            // Add the pixel if (abs(U)+abs(V))/Y < T
            if ((absU + absV) / yuvfImage.at<cv::Vec3f>(r, c)[0] < GRAY_THRESH)
                grayPixelRows[r].push_back(yuvfImage.at<cv::Vec3f>(r, c));
        }
    }

    // Combine the results from each row
    for (int r = 0; r < yuvfImage.rows; r++)
        grayPixels.insert(grayPixels.end(), grayPixelRows[r].begin(), grayPixelRows[r].end());

    // Make sure we have enough gray pixels
    if (grayPixels.size() < GRAY_PIXEL_POP_THRESH)
        return false;

    // Compute the means of the U and V channels of the grey pixels
    #pragma omp parallel for reduction(+: meanU) reduction(+: meanV)
    for (unsigned int i = 0; i < grayPixels.size(); i++)
    {
        meanU += grayPixels[i][1];
        meanV += grayPixels[i][2];
    }

    meanU /= (float)grayPixels.size();
    meanV /= (float)grayPixels.size();

    return true;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Jason Su                                                #
#   Inputs:                                                                         #
#           @meanU                The mean U of the gray pixels                     #
#           @meanV                The mean V of the gray pixels                     #
#   Outputs:                                                                        #
#           @returns              Vector of the estimated illuminant                #
#                                                                                   #
#   Description: Estimates the illuminant of a scene using the mean U and V of      #
#                the gray pixels in an image. Illuminant is in the  CIELAB 1931     #
#                XYZ color space.                                                   #
#                                                                                   #
************************************************************************************/

cv::Mat estimateXYZIlluminant(float meanU, float meanV)
{
    // Working matrices
    cv::Mat grayRGB, grayXYZ;

    // Convert the avarage grey YUV to RGB
    cv::Mat grayYUV = cv::Mat(1, 1, CV_32FC3, cv::Scalar(100, meanU, meanV));

    // Convert the image to XYZ
    cv::cvtColor(grayYUV, grayRGB, CV_YUV2RGB);
    cv::cvtColor(grayRGB, grayXYZ, CV_RGB2XYZ);

    // Compute the XY chromaticity
    cv::Mat xyEst = XYZ2xy(grayXYZ);

    // Normalize Y to 100 for D65 illuminance standard
    cv::Mat xyzEst = xy2XYZ(xyEst, 100);

    return xyzEst;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Jason Su                                                #
#   Inputs:                                                                         #
#           @gain                 The XYZ gain vector to apply                      #
#   Outputs:                                                                        #
#           @returns              RGB CAT matrix                                    #
#                                                                                   #
#   Description: Finds a CAT matrix using a pre-computed gain vector.               #
#                                                                                   #
************************************************************************************/

cv::Mat gain2CAT(cv::Mat gain)
{
    cv::Mat M;

    // Set up the xfm matrix for CAT
    cv::Mat xfm = cv::Mat(3, 3, CV_32FC1);
    std::memcpy(xfm.data, MBSCAT, 3 * 3 * sizeof(float));

    // Used to convert from sRGB to XYZ and back
    cv::Mat sRGBtoXYZ = cv::Mat(3, 3, CV_32FC1);
    std::memcpy(sRGBtoXYZ.data, SRGB2YXZ, 3 * 3 * sizeof(float));

    // Solve the linear system xfm*M=diag(gain*xfm) w/ LU decomposition
    cv::solve(xfm, cv::Mat::diag(gain) * xfm, M);

    return sRGBtoXYZ.inv() * M * sRGBtoXYZ;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Jason Su                                                #
#   Inputs:                                                                         #
#           @xyzEST               Estimated XYZ illuminant                          #
#           @yxzTarger            Desired XYZ illuminant                            #
#           @gain                 Estimated XYZ gain to move from estimated to      #
#                                 target illuminance                                #
#   Outputs:                                                                        #
#           @modifies             gain by reference                                 #
#           @returns              RGB CAT matrix                                    #
#                                                                                   #
#   Description: Computes the XYZ gains required to transform the estimated         #
#                illuminant to the target one, and then subsequent determines the   #
#                CAT matrix in the RGB color space.                                 #    
#                                                                                   #
************************************************************************************/

cv::Mat cbCAT(cv::Mat xyzEst, cv::Mat xyzTarget, cv::Mat & gain)
{
    // Set up the xfm matrix for CAT
    cv::Mat xfm = cv::Mat(3, 3, CV_32FC1);
    std::memcpy(xfm.data, MBSCAT, 3 * 3 * sizeof(float));

    // Compute the gain
    gain = (xfm * xyzTarget) / (xfm * xyzEst);

    return gain2CAT(gain);
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @image                The image to transform                            #
#           @transform            The pixel transformation matrix                   #
#   Outputs:                                                                        #
#           @returns              Transformed RGB image                             #
#                                                                                   #
#   Description: Applys a pixel-wise color transformation to an image.              #
#                                                                                   #
************************************************************************************/

cv::Mat transformImage(cv::Mat image, cv::Mat transform)
{
    // Reshape the image for channel-wise matrix multiplication
    cv::Mat flatImage = image.reshape(1, image.rows*image.cols);

    // Apply the transformation to the image
    cv::Mat transformedFlatImage = flatImage * transform.t();

    // Reshape and return the image
    return transformedFlatImage.reshape(3, image.rows);

}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @gainHistory          Historical record of the most recent gain norm    #
#           @gainNorm             The most recent gain norm                         #
#   Outputs:                                                                        #
#           @modifies             gainHistory by reference                          #
#           @returns              The variance of the most recent gain norms        #
#                                                                                   #
************************************************************************************/

float updateGainVariance(std::vector<float> & gainHistory, float gainNorm)
{
    float gainVar = -1.;
    float gainMean = 0.;

    gainHistory.push_back(gainNorm);

    // Update the gain history vector
    if(gainHistory.size() == GAIN_HISTORY_SIZE+1)
        gainHistory.erase(gainHistory.begin());
      
    // Only compute the variance if we have enough data
    if(gainHistory.size() == GAIN_HISTORY_SIZE)
    { 
        gainVar = 0;

        // Compute the means
        for (int i = 0; i < GAIN_HISTORY_SIZE; i++)
            gainMean += gainHistory[i];
        
        gainMean /= (float)GAIN_HISTORY_SIZE;

        // Compute the variance
        for (int i = 0; i < GAIN_HISTORY_SIZE; i++)
            gainVar += std::pow((gainHistory[i] - gainMean), 2.);

        gainVar /= (float)(GAIN_HISTORY_SIZE - 1);
    }

    // Return the average of the two variances
    return gainVar;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @bgrImage             The BGR color-spaced image                        #
#           @gain                 The XYZ color space gain vector to apply          #
#   Outputs:                                                                        #
#           @returns              The white balanced BGR image                      #
#                                                                                   #
#   Description: White balances an image given an XYZ gain vector                   #
#                                                                                   #
************************************************************************************/

cv::Mat whiteBalanceImage(cv::Mat bgrImage, cv::Mat gain)
{
    cv::Mat rgbfImage, rgbImage, balancedBGRImage;

    // Convert the image to RGB
    cv::cvtColor(bgrImage, rgbImage, CV_BGR2RGB);

    // Convert the image to floating point
    rgbImage.convertTo(rgbfImage, CV_32FC3);

    // Correct the image using the white balance gains
    cv::Mat corrected = transformImage(rgbfImage, gain2CAT(gain));

    // Convert the image back to 8-bit channels
    corrected.convertTo(rgbImage, CV_8UC3);
    cv::cvtColor(rgbImage, balancedBGRImage, CV_RGB2BGR);

    return balancedBGRImage;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla, Jason Su                                                #
#   Inputs:                                                                         #
#           @bgrImage             The BGR color-spaced image                        #
#           @gain                 The total XYZ gain applied to the image           #
#   Outputs:                                                                        #
#           @returns              True if the gain computation was successfull      #
#                                                                                   #
#   Description: Iteratively transforms a BGR image to the D65 illuminant standard  #
#                using chromatic adaptation transformation (CAT).                   #
#                                                                                   #
************************************************************************************/

bool computeGains(cv::Mat bgrImage, cv::Mat & totalGain)
{
    cv::Mat yuvfImage, rgbfImage, rgbImage;

    // XYZ Gain vector used when appling CAT
    cv::Mat gain;
    
    // Used to keep track of the total XYZ gain applied to the image
    totalGain = cv::Mat(3., 1, CV_32FC1, {1., 1., 1.});

    // Time window of the CAT gain
    std::vector<float> gainHistory;

    // Create a working matrix for the D65 illuminant
    cv::Mat xyzD65 = cv::Mat(3, 1, CV_32FC1);
    std::memcpy(xyzD65.data, XYZD65, 3 * sizeof(float));

    // True once the algorithm has converged
    bool converged = false;

    // Convert the image to RGB
    cv::cvtColor(bgrImage, rgbImage, CV_BGR2RGB);

    // Convert the image to floating point
    rgbImage.convertTo(rgbfImage, CV_32FC3);
    
    // Iterate until we've converged, or we've reached the max iterations
    for (int it = 0; it < MAX_CAT_ITERATIONS && !converged; it++)
    {
        // Chanel means for they grey pixels
        float meanU = 0;
        float meanV = 0;

        // Convert the image to YUV
        cv::cvtColor(rgbfImage, yuvfImage, CV_RGB2YUV);

        // Find the grey pixels and compute their means
        if(!calcGrayUVMeans(yuvfImage, meanU, meanV))
            return false;

        // Estimate the XYZ illuminant
        cv::Mat xyzEst = estimateXYZIlluminant(meanU, meanV);
        
        // Compute the correction matrix w/ CAT
        cv::Mat correction = cbCAT(cvtPixel2Vector(xyzEst.at<cv::Vec3f>(0, 0)), xyzD65, gain);
        
        // Compute and update the gain variance
        float gainVar = updateGainVariance(gainHistory, cv::norm(gain));
        
        // Check and see if weyuvfImage, 've converged
        if(gainVar >= 0. && gainVar < GAIN_VAR_THRESHOLD)
        {
            converged = true;
            break;
        }

        // Update the total gain
        totalGain.at<float>(0, 0) *= gain.at<float>(0, 0);
        totalGain.at<float>(1, 0) *= gain.at<float>(1, 0);
        totalGain.at<float>(2, 0) *= gain.at<float>(2, 0);
        
        // Apply correction to the image
        rgbfImage = transformImage(rgbfImage, correction);
    }

    return converged;
}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @gain                 The gain vector to save                           #
#                                                                                   #
#   Description: Saves a gain vector into a yaml file.                              #
#                                                                                   #
************************************************************************************/

void saveXYZGain(cv::Mat gain)
{
    YAML::Emitter emitter;

    // Create the file
    std::ofstream gainFile(XYZ_GAIN_FILE_PATH, std::ios::out | std::ios::trunc);

    // Compose the file
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "x" << YAML::Value << gain.at<float>(0, 0);
    emitter << YAML::Key << "y" << YAML::Value << gain.at<float>(1, 0);
    emitter << YAML::Key << "z" << YAML::Value << gain.at<float>(2, 0);
    emitter << YAML::EndMap;

    // Write to the file
    gainFile << emitter.c_str();

    gainFile.close();

}

/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @path                 The path to the gain file                         #
#   Outputs:                                                                        #
#           @returns              A gain vector                                     #
#                                                                                   #
#   Description: Loads a gain yaml file into a vector.                              #
#                                                                                   #
************************************************************************************/

cv::Mat loadXYZGain(std::string path)
{
    cv::Mat gain = cv::Mat(3, 1, CV_32FC1, {1., 1., 1.});

    // Initialize the gain file if it doesn't already exist
    if(!boost::filesystem::exists(path.c_str()))
    {
        saveXYZGain(gain);
        return gain;
    }

    // Load the gain file
    YAML::Node gainFile = YAML::LoadFile(path);

    // Load in the values
    gain.at<float>(0, 0) = gainFile["x"].as<float>();
    gain.at<float>(1, 0) = gainFile["y"].as<float>();
    gain.at<float>(2, 0) = gainFile["z"].as<float>();

    return gain;
}


/************************************************************************************
#                                                                                   #
#   Author(s): Ethan Takla                                                          #
#   Inputs:                                                                         #
#           @image                The image to process                              #
#           @gain                 The gain vector                                   #
#           @threads              The number of threads to use                      #
#   Outputs:                                                                        #
#           @modifies             gain by reference                                 #
#           @returns              True if the auto-wb was succesfull                #
#                                                                                   #
#   Description: Takes an image, resizes it for speed, computes the white balance   #
#                gains, and returns the gain vector.                                #
#                                                                                   #
************************************************************************************/

bool processImage(cv::Mat image, cv::Mat & gain, int threads)
{
    cv::Mat resizedImage;

    // Set the thread count
    omp_set_num_threads(threads);

    // Scale the image to the desired processing dimension (for speedup)
    float scalingFactor = ((float)TARGET_PROCCESSING_WIDTH_PX)/((float)image.cols);
    cv::resize(image, resizedImage, cv::Size(), scalingFactor, scalingFactor);

    // Compute the channel gains
    if(computeGains(resizedImage, gain))
    {
        // Save the gain
        saveXYZGain(gain);
        
        return true;
    }

    return false;
}
