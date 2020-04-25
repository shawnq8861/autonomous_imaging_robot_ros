/* OpenCV includes */
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/* Function declarations */
cv::Mat XYZ2xy(cv::Mat);
cv::Mat gain2CAT(cv::Mat);
void saveXYZGain(cv::Mat);
cv::Mat xy2XYZ(cv::Mat, float);
cv::Mat loadXYZGain(std::string);
cv::Mat cvtPixel2Vector(cv::Vec3f);
bool computeGains(cv::Mat, cv::Mat&);
cv::Mat transformImage(cv::Mat, cv::Mat);
cv::Mat cbCAT(cv::Mat, cv::Mat, cv::Mat&);
bool processImage(cv::Mat, cv::Mat &, int);
cv::Mat estimateXYZIlluminant(float, float);
cv::Mat whiteBalanceImage(cv::Mat, cv::Mat);
bool calcGrayUVMeans(cv::Mat, float&, float&);
float updateGainVariance(std::vector<float>&, float);
