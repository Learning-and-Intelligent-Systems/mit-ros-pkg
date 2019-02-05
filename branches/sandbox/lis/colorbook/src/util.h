#ifndef COLORBOOK_UTIL_H_
#define COLORBOOK_UTIL_H_

#include <ros/ros.h>

#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>

#include <cstring>
#include <cmath>

using namespace std;

void img2matrix(IplImage* img, double matrix[][3]);
void matrix2img(IplImage* img, double matrix[][3]);

void fixIntensity(double matrix[][3], long n);
void fixIllumination(double matrix[][3], long n);

bool change(double prev[][3], double curr[][3], long n, double eps);

IplImage* normalizeImage(IplImage* original);

#endif
