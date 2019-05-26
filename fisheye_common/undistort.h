#ifndef FISHEYEDEV_UNDISTORT_H
#define FISHEYEDEV_UNDISTORT_H

#include <opencv2/core.hpp>
#include <memory>

#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64

namespace fisheye {

struct ocam_model
{
  double pol[MAX_POL_LENGTH];    // the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
  int length_pol;                // length of polynomial
  double invpol[MAX_POL_LENGTH]; // the coefficients of the inverse polynomial
  int length_invpol;             // length of inverse polynomial
  double xc;         // row coordinate of the center
  double yc;         // column coordinate of the center
  double c;          // affine parameter
  double d;          // affine parameter
  double e;          // affine parameter
  int width;         // image width
  int height;        // image height
};

int get_ocam_model(
    struct ocam_model *myocam_model, const char *filename);
void world2cam(
    double point2D[2], double point3D[3], struct ocam_model *myocam_model);
void cam2world(
    double point3D[3], double point2D[2], struct ocam_model *myocam_model);
void create_perspecive_undistortion_LUT(
    CvMat *mapx, CvMat *mapy, struct ocam_model *ocam_model, float sf);

class FisheyeUndistort {
 public:
  bool LoadCalibResult(const std::string& calib);
  cv::Mat Undistort(const cv::Mat& src);
  float sf = 1.5;
 private:
  std::unique_ptr<ocam_model> model_;
};
}

#endif // FISHEYEDEV_UNDISTORT_H
