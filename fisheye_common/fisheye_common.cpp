
#include "fisheye_common.h"

#include <stdio.h>
#include <iostream>

void fisheye_common(void)
{
	printf("This is fisheye_common version %s\n", FISHEYEDEV_VERSION_STRING);
}

namespace fisheye {
std::pair<cv::Mat, cv::Mat> Split(const cv::Mat& image) {
  std::pair<cv::Mat, cv::Mat> result;
  result.first =
      image(cv::Rect(0, 0, image.size[1] / 2, image.size[0]));
  result.second =
      image(
          cv::Rect(
              image.size[1] / 2, 0,
              image.size[1] / 2, image.size[0]));
  return result;
}
}
