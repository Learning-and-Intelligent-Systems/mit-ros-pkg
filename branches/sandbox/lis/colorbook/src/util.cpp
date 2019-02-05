#include "util.h"
#include <cstdio>

void img2matrix(IplImage* img, double matrix[][3]) {
  for (long i = 0; i < img->height; i++) {
    for (long j = 0; j < img->width; j++) {
      uchar* temp_ptr = &((uchar *)(img->imageData + i*img->widthStep))[3*j];
      matrix[i*img->width + j][0] = temp_ptr[0] / 255.0;
      matrix[i*img->width + j][1] = temp_ptr[1] / 255.0;
      matrix[i*img->width + j][2] = temp_ptr[2] / 255.0;
    }
  }
}

void matrix2img(IplImage* img, double matrix[][3]) {
  long n = img->height * img->width;
  for (long i = 0; i < n; i++) {
    long imageI = (long) (i / img->width);
    long imageJ = i % img->width;
    uchar* temp_ptr = &((uchar *)(img->imageData + imageI*(img->widthStep)))[3*imageJ];
    temp_ptr[0] = (uchar) (matrix[i][0]*255);
    temp_ptr[1] = (uchar) (matrix[i][1]*255);
    temp_ptr[2] = (uchar) (matrix[i][2]*255);
  }
}

void fixIntensity(double matrix[][3], long n) {
  for (long i = 0; i < n; i++) {
    double tmp = matrix[i][0] + matrix[i][1] + matrix[i][2];
    if (tmp!=0) {
      matrix[i][0] /= tmp;
      matrix[i][1] /= tmp;
      matrix[i][2] /= tmp;
    }
  }
}

void fixIllumination(double matrix[][3], long n) {
  double sumRed = 0.0, sumGreen = 0.0, sumBlue = 0.0;
  for (long i = 0; i < n; i++) {
    sumBlue += matrix[i][0];
    sumGreen += matrix[i][1];
    sumRed += matrix[i][2];
  }
  
  for (long i = 0; i < n; i++) {
    if (sumBlue != 0)
      matrix[i][0] = (n/3.0) * matrix[i][0] / sumBlue;
    if (sumGreen != 0)
      matrix[i][1] = (n/3.0) * matrix[i][1] / sumGreen;
    if (sumRed != 0)
      matrix[i][2] = (n/3.0) * matrix[i][2] / sumRed;
  }
}

bool change(double prev[][3], double curr[][3], long n, double eps) {
  for (long i = 0; i < n; i++) {
    //printf("Previous: %lf %lf %lf\n", prev[i][0], prev[i][1], prev[i][2]);
    //printf("Current: %lf %lf %lf\n", curr[i][0], curr[i][1], curr[i][2]);
    if (fabs(prev[i][0]-curr[i][0]) > eps || fabs(prev[i][1]-curr[i][1]) > eps || fabs(prev[i][2]-curr[i][2]) > eps) {
      return true;
    }
  }
  return false;  
}

IplImage* normalizeImage(IplImage* original) {
  long num_pixels = original->height*original->width;
  double current[num_pixels][3];
  double previous[num_pixels][3];

  img2matrix(original, current);

  IplImage* normalized = cvCreateImage(cvSize(original->width, original->height), original->depth, 3);
  bool changed;
  
  int cnt = 0;

  // The process is not converging.

  //do {
    cout << cnt << endl;
    for (long i = 0; i < num_pixels; i++) {
      previous[i][0] = current[i][0];
      previous[i][1] = current[i][1];
      previous[i][2] = current[i][2];
    }
    //fixIntensity(current, num_pixels);
    fixIllumination(current, num_pixels);
    changed = change(previous, current, num_pixels, 0.00001);
    cnt++;
    //} while (changed);

  matrix2img(normalized, current);
  return normalized;
}
