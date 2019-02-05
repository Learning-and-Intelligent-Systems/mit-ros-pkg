#include <cxcore.h>
#include <cv.h>
#include <highgui.h>
#include <string.h>
#include <cstdio>
#include <iostream>

#include "util.h"

using namespace cv;
using namespace std;



double angleDiff(double theta1, double theta2)
{
  double difference = theta2 - theta1;
  while (difference <= -PI)
    difference += 2*PI;
  while (difference > PI)
    difference -= 2*PI;
  return difference;
}

double angleMean(double theta1, double theta2)
{
  double a1 = (theta1 + theta2) / 2;
  double a2 = a1 + PI;
  double a;
  if (fabs(angleDiff(a1, theta1)) < fabs(angleDiff(a2, theta1)))
    a = a1;
  else
    a = a2;
  return a;
}

double linePointDist(double r, double theta, double x, double y)
{
  double nx = cos(theta);	
  double ny = sin(theta);
  double d = nx*x + ny*y - r;
  return d;
}

void edge2polar(double x0, double y0, double x1, double y1, double &r, double &theta)
{
  double eps = 0.00001;
  double vx = x1 - x0;
  double vy = y1 - y0;
  double length = sqrt(vx*vx + vy*vy);
  vx /= length;
  vy /= length;
  double wx = -vy;
  double wy = vx;
  theta = atan2(wy, wx);
  double dist = sqrt(x0*x0 + y0*y0);
  if (dist < eps)
    r = 0;
  else
    r = x0*(wx) + y0*wy;
}

void lineClip(Line L, Rect roi, Point& p0, Point& p1)
{
  double r = L.r;
  double theta = L.theta;
  double x0 = roi.x;
  double y0 = roi.y;
  double x1 = x0 + roi.width;
  double y1 = y0 + roi.height;

  double eps = 0.00001;
  int found = 0;
  vector<double> xtmp;
  vector<double> ytmp;
  
  if (fabs(sin(theta)) > eps) {
    double y = (r - x0*cos(theta))/sin(theta);
    if (y>=y0 && y<=y1) {
      xtmp.push_back(x0);
      ytmp.push_back(y);
      found++;
    }
    y = (r - x1*cos(theta))/sin(theta);
    if (y>=y0 && y<=y1) {
      xtmp.push_back(x1);
      ytmp.push_back(y);
      found++;
    }
  }
  
  if (found == 2) {
    p0.x = xtmp[0];
    p0.y = ytmp[0];
    p1.x = xtmp[1];
    p1.y = ytmp[1];
    return;
  }
  
  if (fabs(cos(theta)) > eps) {
    double x = (r - y0*sin(theta))/cos(theta);	
    if (x>=x0 && x<=x1) {
      xtmp.push_back(x);
      ytmp.push_back(y0);
      found++;
    }
    x = (r - y1*sin(theta))/cos(theta);
    if (x>=x0 && x<=x1) {
      xtmp.push_back(x);
      ytmp.push_back(y1);
      found++;
    }
  }
  
  p0.x = xtmp[0];
  p0.y = ytmp[0];
  p1.x = xtmp[1];
  p1.y = ytmp[1];
}

void bookEdgeDist(double leftR, double leftTheta,
		  double rightR, double rightTheta,
		  double &dr, double &dtheta)
{
  dr = fabs(fabs(rightR) - fabs(leftR));
  dtheta = fabs(rightTheta - leftTheta);
  dtheta = min(min(dtheta, fabs(dtheta-PI)), fabs(dtheta-2*PI));
}


//--------------------------------------------------//


/*
 * Returns a pointer to the nth word (starting from 0) in string s
 */
char *sword(char *s, const char *delim, int n)
{
  if (s == NULL)
    return NULL;

  s += strspn(s, delim);  // skip over initial delimeters

  int i;
  for (i = 0; i < n; i++) {
    s += strcspn(s, delim);  // skip over word
    s += strspn(s, delim);  // skip over delimeters
  }

  return s;
}

double gauss2d_pdf(double x, double y, double ux, double uy,
		   double vx, double vy, double vxy)
{
  double dx = x-ux;
  double dy = y-uy;
  double c = vxy*vxy/vx*vy;
  double sxy = vxy/c;

  return 1/(2*PI*sxy*sqrt(1-c)) * exp(-1/(2*(1-c)) * (dx*dx/vx + dy*dy/vy - 2*c*dx*dy/vxy));
}

double gauss2d_dist(double x, double y, double ux, double uy,
		   double vx, double vy, double vxy)
{
  double dx = x-ux;
  double dy = y-uy;
  double c = vxy*vxy/vx*vy;

  return 1/(2*(1-c)) * (dx*dx/vx + dy*dy/vy - 2*c*dx*dy/vxy);
}


/*
 * Convert an grayscale image to a matrix of doubles.
 */
void im2double(const Mat& src, Mat1d& dst)
{
  src.assignTo(dst, DataType<double>::type);
  dst = dst / 256;
}

Mat1d im2double(const Mat& src)
{
  Mat1d dst;
  src.assignTo(dst, DataType<double>::type);
  dst = dst / 256;

  return dst;
}

/*
 * Convert an grayscale image to a matrix of floats.
 */
void im2float(const Mat& src, Mat1f& dst)
{
  src.assignTo(dst, DataType<float>::type);
  dst = dst / 256;
}

Mat1f im2float(const Mat& src)
{
  Mat1f dst;
  src.assignTo(dst, DataType<float>::type);
  dst = dst / 256;

  return dst;
}


/*
 * Flips the rows of a matrix.
 */
Mat1f flipRows(const Mat1f& I)
{
  int nr = I.rows;
  int nc = I.cols;

  Mat1f J(nr, nc, I.type());

  for (int y=0; y<nr; y++)
    J.row(y) = I.row(nr-y-1);

  return J;
}

/*
 * Autocrop an image to remove zeros on the image border.
 */
void imageTrim(const Mat1f& src, Mat1f& dst)
{
  int nr = src.rows;
  int nc = src.cols;

  for (int i=0; i<20; i++)
    src.row(i) = Mat::zeros(1,nc,src.type());

  int i0=nr, i1=0, j0=nc, j1=0;

  for (int i=0; i<nr; i++) {
    for (int j=0; j<nc; j++) {
      double v = src(i,j);
      if (v > 0) {
	if (i < i0)
	  i0 = i;
	if (i > i1)
	  i1 = i;
	if (j < j0)
	  j0 = j;
	if (j > j1)
	  j1 = j;
      }
    }
  }

  if (i0 > i1 || j0 > j1)
    return;

  dst = src(Range(i0,i1+1), Range(j0,j1+1));
}

/*
 * Crop the subregion from an image between two edges, p and q.
 */
void imageCropQuad(const Mat1f& src, Mat1f& dst, const Point& p0, const Point& p1,
		   const Point& q0, const Point& q1)
{
  double left_r, left_theta, right_r, right_theta;
  double up_r, up_theta, down_r, down_theta;
  edge2polar(p0.x, p0.y, p1.x, p1.y, left_r, left_theta);
  edge2polar(q0.x, q0.y, q1.x, q1.y, right_r, right_theta);
  edge2polar(p0.x, p0.y, q0.x, q0.y, up_r, up_theta);
  edge2polar(p1.x, p1.y, q1.x, q1.y, down_r, down_theta);

  dst = Mat1f::zeros(src.rows, src.cols);

  for (int y=0; y<src.rows; y++) {
    for (int x=0; x<src.cols; x++) {
      if (linePointDist(left_r, left_theta, x, y) < -1 &&
	  linePointDist(right_r, right_theta, x, y) > 1 &&
	  linePointDist(up_r, up_theta, x, y) > 1 &&
	  linePointDist(down_r, down_theta, x, y) < -1)
	{
	  dst(y,x) = src(y,x);
	}
    }
  }

  imageTrim(dst, dst);
}

/*
 * Compute the cost of matching two lines.
 */
double bookEdgeMatchCost(Line& left, Line& right, double ur, double utheta,
			 double vr, double vtheta, double vrtheta)
{
  double dr, dtheta;
  bookEdgeDist(left.r, left.theta, right.r, right.theta, dr, dtheta);
  return gauss2d_dist(dr, dtheta, ur, utheta, vr, vtheta, vrtheta);
}

/*
 * Save book labels.
 */
int saveBookLabels(const char *imdir, const vector<int>& labels)
{
  char fout[256];
  sprintf(fout, "%s/labels.txt", imdir);
  FILE *f = fopen(fout, "w");
  if (f == NULL) {
    printf("Error opening %s for writing\n", fout);
    return -1;
  }

  for (uint i=0; i<labels.size(); i++)
    fprintf(f, "%d\n", labels[i]);

  fclose(f);
  return 0;
}

/*
 * Load book labels.
 */
int loadBookLabels(const char *imdir, vector<int>& labels)
{
  // read labels.txt
  char fin[256];
  sprintf(fin, "%s/labels.txt", imdir);
  FILE *f = fopen(fin, "r");
  if (f == NULL) {
    printf("Error loading %s/labels.txt\n", imdir);
    return -1;
  }
  
  char s[1024];
  int x;
  while (1) {
    if (fgets(s, 1024, f) == NULL)
      break;
    if (sscanf(s, "%d", &x) < 1)
      break;
    labels.push_back(x);
  }

  fclose(f);
  return 0;
}

/*
 * Save book images.
 */
int saveBookImages(const char *imdir, const vector<Mat1f>& images)
{
  char fout[256];

  for (uint i=0; i<images.size(); i++) {
    sprintf(fout, "%s/b%d.png", imdir, (int)(i+1));
    if (imwrite(fout, 128 + 64*images[i]) == false)
      return -1;
  }

  return 0;
}

/*
 * Load normalized book images.
 */
int loadBookImages(const char *imdir, int n, vector<Mat1f>& images)
{
  Mat I;
  char fin[256];

  for (int i=0; i<n; i++) {
    sprintf(fin, "%s/b%d.png", imdir, i+1);
    I = imread(fin, 0);  // grayscale
    if (I.data == NULL) {
      printf("Error loading %s\n", fin);
      return -1;
    }
    images.push_back((im2float(I) - 0.5) * 4.0);
  }

  return 0;
}

/*
 * Classify book image with Nearest Neighbors.
 */
int classifyBookImage(const Mat1f& I, vector<Mat1f>& images,
		      vector<int>& labels, double *thresholds, double& dmin)
{
  int n = images.size();

  dmin = 1000000.0;
  double dmin2 = dmin;
  int L = 0, L2 = 0;

  for (int i=0; i<n; i++) {

    Mat1f& B1 = images[i];
    double d1 = sum(abs(I-B1))[0];
    Mat1f B2 = flipRows(images[i]);
    double d2 = sum(abs(I-B2))[0];

    double d = min(d1,d2);

    if (d < dmin) {
      dmin2 = dmin;
      L2 = L;
      dmin = d;
      L = labels[i];
    }
  }

  if (L != 0) {
    if (dmin > thresholds[L-1]) {
      if (dmin2 > thresholds[L2-1])
	L = 0;
      else {
	L = L2;
	dmin = dmin2;
      }
    }
  }

  return L;
}



//-----------------------------------------------//



void testGui(Mat I)
{
  namedWindow(fig1);

  Mat1f I2;
  im2float(I, I2);
  printf("I2(0,0) = %f\n", I2(0,0));

  imshow(fig1, I2);
  waitKey(0);
}

void testROI(Mat I)
{
  printf("testROI()\n");

  namedWindow(fig1);
  imshow(fig1, I);

  Mat J = I(Range(50,100), Range(50,100));
  namedWindow(fig2);
  imshow(fig2, J);


  waitKey(0);
}

void testTrim(const Mat1f& I)
{
  namedWindow(fig1);
  imshow(fig1, I);

  Mat1f J;
  imageTrim(I, J);

  namedWindow(fig2);
  imshow(fig2, J);

  waitKey(0);
}

void testCropQuad(const Mat1f& I)
{
  Mat1f J;
  Point2d p0(40,85), p1(45,210), q0(65,88), q1(61,212);
  imageCropQuad(I, J, p0, p1, q0, q1);

  namedWindow(fig1);
  imshow(fig1, I);

  namedWindow(fig2);
  imshow(fig2, J);

  waitKey(0);
}

void testLoadBooks(const char *imdir)
{
  vector<int> labels;
  vector<Mat1f> images;
  double thresholds[11] = {1200, 1600, 1000, 1200, 1300, 1000, 1200, 1300, 1300, 1400, 1400};

  if (loadBookLabels(imdir, labels) < 0)
    return;

  if (loadBookImages(imdir, labels.size(), images) < 0)
    return;

  /*
  namedWindow(fig1);
  for (int i=0; i<images.size(); i++) {
    imshow(fig1, images[i]);
    waitKey(0);
  }
  */

  double R = 0.0;

  Mat1f Z = Mat1f::zeros(images[0].size());
  for (uint i=0; i<images.size(); i++) {
    Mat1f I = images[i];
    images[i] = Z;
    double dmin;
    int L = classifyBookImage(I, images, labels, thresholds, dmin);
    images[i] = I;
    printf("true label: %d, estimated label: %d\n", labels[i], L);
    if (labels[i] == L)
      R += 1.0;
  }
  R /= images.size();

  printf(" --> classification rate = %.2f\n", R);

}


/*
void usage(int argc, char *argv[])
{
  printf("%s <image>\n", argv[0]);
  exit(1);
}

int main(int argc, char *argv[])
{
  if (argc < 2)
    usage(argc, argv);

  Mat img = imread(argv[1], 0);
  Mat1f I;
  im2float(img, I);

  //testGui(I);
  //testROI(I);
  //testTrim(I);
  //testCropQuad(I);
  testLoadBooks("books");

  return 0;
}
*/
