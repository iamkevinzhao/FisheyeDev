
#include "fisheye_common/fisheye_common.h"
#include "fisheye_common/undistort.h"
#include <opencv2/highgui.hpp>
#include <iostream>

int main(int argc, char* argv[])
{
  cv::Mat image = cv::imread("test.jpg", CV_LOAD_IMAGE_ANYCOLOR);
  fisheye::FisheyeUndistort undistort;
  if (!undistort.LoadCalibResult("result.txt")) {
    std::cout << "unable to read calibration result" << std::endl;
    return -1;
  }
  cv::Mat result = undistort.Undistort(image);
  cv::imshow("result", result);
  cv::waitKey(10000);
	return 0;
}
