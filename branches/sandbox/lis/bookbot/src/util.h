#pragma once

#include <cxcore.h>
#include <cv.h>
#include <highgui.h>
#include <string.h>


using namespace cv;
using namespace std;


const char fig1[] = "Figure 1";
const char fig2[] = "Figure 2";
const char fig3[] = "Figure 3";
const char fig4[] = "Figure 4";

const double PI = M_PI;


class Line {
 public:
  double r;
  double theta;
  Line(double r_, double theta_) { r = r_; theta = theta_; }
};

class LineSegment {
 public:
  Point p1;
  Point p2;
  LineSegment() {}
  LineSegment(Point p1_, Point p2_) { p1 = p1_; p2 = p2_; }
  LineSegment(int x0, int y0, int x1, int y1) { p1 = Point(x0,y0); p2 = Point(x1,y1); }
};


double angleDiff(double theta1, double theta2);
double angleMean(double theta1, double theta2);
double linePointDist(double r, double theta, double x, double y);
void lineClip(Line L, Rect roi, Point& p0, Point& p1);
void edge2polar(double x0, double y0, double x1, double y1, double &r, double &theta);
void bookEdgeDist(double leftR, double leftTheta,
		  double rightR, double rightTheta,
		  double &dr, double &dtheta);
char *sword(char *s, const char *delim, int n);
double gauss2d_pdf(double x, double y, double ux, double uy,
		   double vx, double vy, double vxy);
double gauss2d_dist(double x, double y, double ux, double uy,
		    double vx, double vy, double vxy);
//void im2double(const Mat& src, Mat1d& dst);
//Mat1d im2double(const Mat& src);
void im2float(const Mat& src, Mat1f& dst);
Mat1f im2float(const Mat& src);
Mat1f flipRows(const Mat1f& I);
void imageTrim(const Mat1f& src, Mat1f& dst);
void imageCropQuad(const Mat1f& src, Mat1f& dst, const Point& p0, const Point& p1,
		   const Point& q0, const Point& q1);
double bookEdgeMatchCost(Line& left, Line& right, double ur, double utheta,
			 double vr, double vtheta, double vrtheta);
int saveBookLabels(const char *imdir, const vector<int>& labels);
int saveBookImages(const char *imdir, const vector<Mat1f>& images);
int loadBookLabels(const char *imdir, vector<int>& labels);
int loadBookImages(const char *imdir, int n, vector<Mat1f>& images);
int classifyBookImage(const Mat1f& I, vector<Mat1f>& images,
		      vector<int>& labels, double *thresholds, double& dmin);





