#include "undistort.h"
#include <opencv2/imgproc.hpp>
#include <iostream>

namespace fisheye {
void cam2world(
    double point3D[3], double point2D[2], struct ocam_model *myocam_model)
{
  double *pol = myocam_model->pol;
  double xc = (myocam_model->xc);
  double yc = (myocam_model->yc);
  double c = (myocam_model->c);
  double d = (myocam_model->d);
  double e = (myocam_model->e);
  int length_pol = (myocam_model->length_pol);
  // 1/det(A), where A = [c,d;e,1] as in the Matlab file
  double invdet = 1 / (c - d * e);

  double xp = invdet * ((point2D[0] - xc) - d * (point2D[1] - yc));
  double yp = invdet * (-e * (point2D[0] - xc) + c * (point2D[1] - yc));

  //distance [pixels] of  the point from the image center
  double r = sqrt(xp*xp + yp * yp);
  double zp = pol[0];
  double r_i = 1;
  int i;

  for (i = 1; i < length_pol; i++)
  {
    r_i *= r;
    zp += r_i * pol[i];
  }

  //normalize to unit norm
  double invnorm = 1 / sqrt(xp*xp + yp * yp + zp * zp);

  point3D[0] = invnorm * xp;
  point3D[1] = invnorm * yp;
  point3D[2] = invnorm * zp;
}

void world2cam(
    double point2D[2], double point3D[3], struct ocam_model *myocam_model)
{
  double *invpol = myocam_model->invpol;
  double xc = (myocam_model->xc);
  double yc = (myocam_model->yc);
  double c = (myocam_model->c);
  double d = (myocam_model->d);
  double e = (myocam_model->e);
  int width = (myocam_model->width);
  int height = (myocam_model->height);
  int length_invpol = (myocam_model->length_invpol);
  double norm = sqrt(point3D[0] * point3D[0] + point3D[1] * point3D[1]);
  double theta = atan(point3D[2] / norm);
  double t, t_i;
  double rho, x, y;
  double invnorm;
  int i;

  if (norm != 0)
  {
    invnorm = 1 / norm;
    t = theta;
    rho = invpol[0];
    t_i = 1;

    for (i = 1; i < length_invpol; i++)
    {
      t_i *= t;
      rho += t_i * invpol[i];
    }

    x = point3D[0] * invnorm*rho;
    y = point3D[1] * invnorm*rho;

    point2D[0] = x * c + y * d + xc;
    point2D[1] = x * e + y + yc;
  }
  else
  {
    point2D[0] = xc;
    point2D[1] = yc;
  }
}

void create_perspecive_undistortion_LUT(
    cv::Mat *mapx, cv::Mat *mapy, struct ocam_model *ocam_model, float sf)
{
  int i, j;
  int width = mapx->cols; //New width
  int height = mapx->rows;//New height
  float *data_mapx = (float*)mapx->data;
  float *data_mapy = (float*)mapy->data;
  float Nxc = height / 2.0;
  float Nyc = width / 2.0;
  float Nz = -width / sf;
  double M[3];
  double m[2];

  for (i = 0; i<height; i++)
    for (j = 0; j<width; j++)
    {
      M[0] = (i - Nxc);
      M[1] = (j - Nyc);
      M[2] = Nz;
      world2cam(m, M, ocam_model);
      *(data_mapx + i * width + j) = (float)m[1];
      *(data_mapy + i * width + j) = (float)m[0];
    }
}

void create_perspecive_undistortion_LUT(
    CvMat *mapx, CvMat *mapy, struct ocam_model *ocam_model, float sf)
{
  int i, j;
  int width = mapx->cols; //New width
  int height = mapx->rows;//New height
  float *data_mapx = mapx->data.fl;
  float *data_mapy = mapy->data.fl;
  float Nxc = height / 2.0;
  float Nyc = width / 2.0;
  float Nz = -width / sf;
  double M[3];
  double m[2];

  for (i = 0; i<height; i++)
    for (j = 0; j<width; j++)
    {
      M[0] = (i - Nxc);
      M[1] = (j - Nyc);
      M[2] = Nz;
      world2cam(m, M, ocam_model);
      *(data_mapx + i * width + j) = (float)m[1];
      *(data_mapy + i * width + j) = (float)m[0];
    }
}

int get_ocam_model(struct ocam_model *myocam_model, const char *filename)
{
  double *pol = myocam_model->pol;
  double *invpol = myocam_model->invpol;
  double *xc = &(myocam_model->xc);
  double *yc = &(myocam_model->yc);
  double *c = &(myocam_model->c);
  double *d = &(myocam_model->d);
  double *e = &(myocam_model->e);
  int    *width = &(myocam_model->width);
  int    *height = &(myocam_model->height);
  int *length_pol = &(myocam_model->length_pol);
  int *length_invpol = &(myocam_model->length_invpol);
  FILE *f;
  char buf[CMV_MAX_BUF];
  int i;

  //Open file
  if (!(f = fopen(filename, "r")))
  {
    printf("File %s cannot be opened\n", filename);
    return -1;
  }

  //Read polynomial coefficients
  fgets(buf, CMV_MAX_BUF, f);
  fscanf(f, "\n");
  fscanf(f, "%d", length_pol);
  for (i = 0; i < *length_pol; i++)
  {
    fscanf(f, " %lf", &pol[i]);
  }

  //Read inverse polynomial coefficients
  fscanf(f, "\n");
  fgets(buf, CMV_MAX_BUF, f);
  fscanf(f, "\n");
  fscanf(f, "%d", length_invpol);
  for (i = 0; i < *length_invpol; i++)
  {
    fscanf(f, " %lf", &invpol[i]);
  }

  //Read center coordinates
  fscanf(f, "\n");
  fgets(buf, CMV_MAX_BUF, f);
  fscanf(f, "\n");
  fscanf(f, "%lf %lf\n", xc, yc);

  //Read affine coefficients
  fgets(buf, CMV_MAX_BUF, f);
  fscanf(f, "\n");
  fscanf(f, "%lf %lf %lf\n", c, d, e);

  //Read image size
  fgets(buf, CMV_MAX_BUF, f);
  fscanf(f, "\n");
  fscanf(f, "%d %d", height, width);

  fclose(f);
  return 0;
}

bool FisheyeUndistort::LoadCalibResult(const std::string &calib) {
  model_.reset(new ocam_model);
  if (get_ocam_model(model_.get(), calib.c_str()) != 0) {
    model_.reset();
    return false;
  }
  return true;
}

std::pair<cv::Mat, cv::Mat> FisheyeUndistort::Undistort(
    const std::pair<cv::Mat, cv::Mat>& src) {
  std::pair<cv::Mat, cv::Mat> result;
  result.first = Undistort(src.first);
  result.second = Undistort(src.second);
  return result;
}

cv::Mat FisheyeUndistort::Undistort(const cv::Mat &src) {
  if (!model_) {
    return cv::Mat(0, 0, CV_32FC1);
  }
//  IplImage copy = src;
//  IplImage *src_cstyle = &copy;
//  IplImage *dst_persp = cvCreateImage(cvGetSize(src_cstyle), 8, 3);


//  CvMat* mapx_persp = cvCreateMat(src_cstyle->height, src_cstyle->width, CV_32FC1);
//  CvMat* mapy_persp = cvCreateMat(src_cstyle->height, src_cstyle->width, CV_32FC1);

  cv::Mat mapx_persp(src.rows, src.cols, CV_32FC1);
  cv::Mat mapy_persp(src.rows, src.cols, CV_32FC1);
  create_perspecive_undistortion_LUT(&mapx_persp, &mapy_persp, model_.get(), sf);

//  create_perspecive_undistortion_LUT(
//      mapx_persp, mapy_persp, model_.get(), sf);

//  cv::InterpolationFlags;
  cv::Mat dst;
  cv::remap(src, dst, mapx_persp, mapy_persp, cv::INTER_LINEAR);
//  cvRemap(
//      src_cstyle, dst_persp, mapx_persp, mapy_persp,
//      CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));

//  cvReleaseMat(&mapx_persp);
//  cvReleaseMat(&mapy_persp);
//  cv::Mat result = cv::cvarrToMat(dst_persp, true);
//  cvReleaseImage(&dst_persp);
  return dst;
//  return result;
}
}
