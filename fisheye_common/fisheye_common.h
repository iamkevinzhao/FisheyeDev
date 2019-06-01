#pragma once
#ifndef FISHEYEDEV_FISHEYE_COMMON_H
#define FISHEYEDEV_FISHEYE_COMMON_H

#include <fisheye_common/config.h>
#include <opencv2/core.hpp>

void fisheye_common(void);

namespace fisheye {
  std::pair<cv::Mat, cv::Mat> Split(const cv::Mat& image);
}

#endif
