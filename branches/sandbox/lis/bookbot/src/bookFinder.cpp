#include <iostream>
#include <cv.h>
#include <highgui.h>

#include <ros/ros.h>
#include <bookbot/BBox.h>
#include <bookbot/BookList.h>
#include <cv_bridge/CvBridge.h>
#include <geometry_msgs/Point.h>
#include <image_geometry/pinhole_camera_model.h>
#include <stereo_msgs/DisparityImage.h>
#include <tf/transform_listener.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "util.h"


// ProjectTfFrameToImage
// image_geometry::StereoCameraModel


using namespace std;
using namespace cv;




class BookCover {
public:
  int id;
  double cost;
  LineSegment leftEdge;
  LineSegment rightEdge;
  BookCover(int i, double c, LineSegment l, LineSegment r) {
    id = i;
    cost = c;
    leftEdge = l;
    rightEdge = r;
  }
};



//---------- global vars ----------//

ros::Publisher pub;
tf::TransformListener *tfListener;
sensor_msgs::CvBridge imBridge;
sensor_msgs::CvBridge dispBridge;

const char imdir[] = "data/books";
vector<int> bookLabels;
vector<Mat1f> bookImages;
//double bookThresholds[11] = {1200, 1600, 1000, 1200, 1300, 1000, 1200, 1300, 1300, 1400, 1400};
double bookThresholds[9] = {1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100, 1100};
Scalar bookColors[9] = {CV_RGB(255,0,0), CV_RGB(0,255,0), CV_RGB(0,0,255), CV_RGB(255,255,0),
			CV_RGB(0,255,255), CV_RGB(255,0,255), CV_RGB(100,0,0), CV_RGB(0,100,0),
			CV_RGB(0,0,100)};
bool doneLabeling = false;

const int KEY_ENTER = 10;
const int KEY_ZERO = 48;
const int KEY_D = 100;
const int KEY_S = 115;



//----------- messages ------------//

int msgFlags[4] = {0,0,0,0};
const int numMsgs = 4;

inline int msgCount()
{
  return msgFlags[0] + msgFlags[1] + msgFlags[2] + msgFlags[3];
}

bookbot::BBox bookStack;
Mat disparityImage;
sensor_msgs::CameraInfo camInfo;
Mat image;


//----------- callbacks ------------//

void bboxCallback(bookbot::BBox b)
{
  printf("b");
  fflush(0);
  if (b.success) {
    bookStack = b;
    msgFlags[0] = 1;
  }
}

void disparityCallback(const stereo_msgs::DisparityImageConstPtr& d)
{
  printf("d");
  fflush(0);

  if (dispBridge.fromImage(d->image)) {
    Mat I = dispBridge.toIpl();
    I.copyTo(disparityImage);
    msgFlags[1] = 1;
  }
  else {
    printf("\n***fuck!***\n");
  }
}

void cameraInfoCallback(sensor_msgs::CameraInfo c)
{
  printf("c");
  fflush(0);

  camInfo = c;
  msgFlags[2] = 1;
}

void imageCallback(const sensor_msgs::ImageConstPtr& I_ptr)
{
  printf("i");
  fflush(0);

  IplImage *I_ipl = imBridge.imgMsgToCv(I_ptr);
  Mat I(I_ipl);
  I.copyTo(image);
  msgFlags[3] = 1;
}


//---------- helper methods ----------//

void bookStackCornersToImage(Point2d& ll, Point2d& ul, Point2d& lr, Point2d& ur)
{
  image_geometry::PinholeCameraModel camModel;
  camModel.fromCameraInfo(camInfo);

  geometry_msgs::Point &pll = bookStack.lowerLeft.point;
  geometry_msgs::Point &pul = bookStack.upperLeft.point;
  geometry_msgs::Point &plr = bookStack.lowerRight.point;
  geometry_msgs::Point &pur = bookStack.upperRight.point;

  camModel.project3dToPixel(Point3d(pll.x, pll.y, pll.z), ll);
  camModel.project3dToPixel(Point3d(pul.x, pul.y, pul.z), ul);
  camModel.project3dToPixel(Point3d(plr.x, plr.y, plr.z), lr);
  camModel.project3dToPixel(Point3d(pur.x, pur.y, pur.z), ur);
}

void rectifyCorners(const Point2d& ll, const Point2d& ul, const Point2d& lr, const Point2d& ur,
		    Mat1f& I, Mat1f& DI, Rect& roi, Mat1f& transform)
{
  double x0 = min(ll.x, ul.x);
  double y0 = min(ul.y, ur.y);
  double x1 = max(lr.x, ur.x);
  double y1 = max(ll.y, lr.y);
  roi = Rect((int)x0, (int)y0, (int)(x1-x0), (int)(y1-y0));
  const Point2f corners[4] = {Point2f(ll.x, ll.y), Point2f(ul.x, ul.y),
			      Point2f(lr.x, lr.y), Point2f(ur.x, ur.y)};
  const Point2f corners2[4] = {Point2f(x0,y1), Point2f(x0,y0),
			       Point2f(x1,y1), Point2f(x1,y0)};
  Mat P = getPerspectiveTransform(corners, corners2);
  Mat warpedImage, warpedDisparityImage;
  warpPerspective(image, warpedImage, P, image.size());
  warpPerspective(disparityImage, warpedDisparityImage, P, disparityImage.size(), INTER_NEAREST);
  im2float(warpedImage, I);
  im2float(warpedDisparityImage, DI);

  P.assignTo(transform, DataType<float>::type);

  // display result
  //namedWindow(fig1);
  //namedWindow(fig2);
  //namedWindow(fig3);
  namedWindow(fig4);

  //circle(image, ll, 3, CV_RGB(255,0,0), -1);
  //circle(image, ul, 3, CV_RGB(255,0,0), -1);
  //circle(image, lr, 3, CV_RGB(255,0,0), -1);
  //circle(image, ur, 3, CV_RGB(255,0,0), -1);
  //imshow(fig1, image);
  //imshow(fig2, warpedImage);
  //imshow(fig3, disparityImage);
  imshow(fig4, warpedDisparityImage / 256);
  //waitKey(0);
}

void getBookTops(Mat1f DI, Rect roi, int *Y)
{
  //cout << "DI.cols = " << DI.cols << endl;
  //cout << "roi.x = " << roi.x << ", roi.x + roi.width = " << roi.x + roi.width << endl;

  // find max disparity in each column of roi in DI
  for (int x = roi.x; x < roi.x + roi.width; x++) {
    int ymax = 0;
    float dmax = -1;
    for (int y = roi.y; y < roi.y + roi.height/2; y++) {
      if (DI(y,x) > dmax) {
	dmax = DI(y,x);
	ymax = y;
      }
    }
    Y[x] = ymax;
  }

  // smooth signal
  for (int x = roi.x; x < roi.x + roi.width; x++) {
    int x0 = MAX(x-1, roi.x);
    int x1 = MIN(x+1, roi.x + roi.width - 1);
    Y[x] = round((Y[x0] + Y[x] + Y[x1]) / 3.0);
  }
}

void getBookBottoms(Mat1f DI, Rect roi, int *Y)
{
  // find min disparity in each column of roi in DI
  //printf("\n");
  for (int x = roi.x; x < roi.x + roi.width; x++) {
    int ymin = 0;
    float dmin = 1000000;
    for (int y = roi.y + roi.height/2; y < roi.y + roi.height; y++) {
      //printf("%f ", DI(y,x));
      if (DI(y,x) > 0 && DI(y,x) < dmin) {
	dmin = DI(y,x);
	ymin = y;
      }
    }
    Y[x] = ymin;
    //printf("\n");
  }

  // smooth signal
  for (int x = roi.x; x < roi.x + roi.width; x++) {
    int x0 = MAX(x-1, roi.x);
    int x1 = MIN(x+1, roi.x + roi.width - 1);
    Y[x] = round((Y[x0] + Y[x] + Y[x1]) / 3.0);
  }
}

vector<Point2i> edgePoints(Mat1b E, Rect roi)
{
  vector<Point2i> P;
  for (int y = roi.y; y < roi.y + roi.height; y++)
    for (int x = roi.x; x < roi.x + roi.width; x++)
      if (E(y,x) > 0.0)
	P.push_back(Point2i(x,y));

  return P;
}

vector<Line> findLines(vector<Point2i> P)
{
  vector<Line> lines;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  cloud.width = P.size();
  cloud.height = 1;
  cloud.points.resize(cloud.width*cloud.height);

  for (uint i=0; i<P.size(); i++) {
    cloud.points[i].x = P[i].x;
    cloud.points[i].y = P[i].y;
    cloud.points[i].z = 0.0;
  }

  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  Eigen::Vector3f axis = Eigen::Vector3f(0.0, 1.0, 0.0);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_ORIENTED_LINE);
  seg.setAxis(axis);
  seg.setEpsAngle(.2);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(2.0);
  seg.setMaxIterations(200);
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  while (cloud.points.size() > .2*P.size()) {

    // fit a line with MLESAC
    seg.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
    seg.segment(inliers, coefficients);

    if (inliers.indices.size() < 10)
      break;

    // convert line to polar coordinates and add it to the list
    double x0 = coefficients.values[0];
    double y0 = coefficients.values[1];
    double x1 = x0 + coefficients.values[3];
    double y1 = y0 + coefficients.values[4];
    double r, theta;
    edge2polar(x0, y0, x1, y1, r, theta);
    lines.push_back(Line(r, theta));

    // remove inliers from point cloud
    extract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(cloud));
    extract.setIndices(boost::make_shared<pcl::PointIndices>(inliers));
    extract.setNegative(true);
    extract.filter(cloud);
  }

  return lines;
}

bool lineSegmentCompareX1(LineSegment s1, LineSegment s2)
{
  return (s1.p1.x < s2.p1.x);
}

Point2d invertPerspective(Point p, Mat1f transform)
{
  double x = p.x;
  double y = p.y;

  Mat1f M;
  invert(transform, M);

  double M11 = M(0,0);
  double M12 = M(0,1);
  double M13 = M(0,2);
  double M21 = M(1,0);
  double M22 = M(1,1);
  double M23 = M(1,2);
  double M31 = M(2,0);
  double M32 = M(2,1);
  double M33 = M(2,2);

  double Z = M31*x + M32*y + M33;
  double x2 = (M11*x + M12*y + M13) / Z;
  double y2 = (M21*x + M22*y + M23) / Z;

  return Point2d(x2,y2);
}

double det(Point3d p1, Point3d p2, Point3d p3)
{
  return p1.x*p2.y*p3.z + p2.x*p3.y*p1.z + p3.x*p1.y*p2.z - p3.x*p2.y*p1.z - p1.x*p3.y*p2.z - p2.x*p1.y*p3.z;
}

Point3d findIntersection(Point3d p1, Point3d p2, Point3d p3, Point3d p)
{
  double t = det(p1, p2, p3);
  double tmp = 0;
  tmp += det(p2, p3, p);
  tmp -= det(p1, p3, p);
  tmp += det(p1, p2, p);
  t /= tmp;
  Point3d intersection;
  intersection.x = t*p.x;
  intersection.y = t*p.y;
  intersection.z = t*p.z;

  return intersection;
}

void publishBookList(vector<BookCover> bookCovers, Mat1f transform)
{
  bookbot::BookList books;
  Point2d p2d;
  Point3d p3d;
  image_geometry::PinholeCameraModel camModel;
  camModel.fromCameraInfo(camInfo);

  for (uint i=0; i<bookCovers.size(); i++) {
    bookbot::Book b;
    b.id = bookCovers[i].id;
    
    Point3d p1, p2, p3;
    p1.x = bookStack.lowerLeft.point.x;
    p1.y = bookStack.lowerLeft.point.y;
    p1.z = bookStack.lowerLeft.point.z;
    
    p2.x = bookStack.lowerRight.point.x;
    p2.y = bookStack.lowerRight.point.y;
    p2.z = bookStack.lowerRight.point.z;

    p3.x = bookStack.upperLeft.point.x;
    p3.y = bookStack.upperLeft.point.y;
    p3.z = bookStack.upperLeft.point.z;
	
    Point3d intersection;    
    
    p2d = invertPerspective(bookCovers[i].leftEdge.p2, transform);
    camModel.projectPixelTo3dRay(p2d, p3d);
    intersection = findIntersection(p1, p2, p3, p3d);
    b.cover.lowerLeft.header = bookStack.lowerLeft.header;
    b.cover.lowerLeft.point.x = intersection.x;
    b.cover.lowerLeft.point.y = intersection.y;
    b.cover.lowerLeft.point.z = intersection.z;

    p2d = invertPerspective(bookCovers[i].leftEdge.p1, transform);
    camModel.projectPixelTo3dRay(p2d, p3d);
    intersection = findIntersection(p1, p2, p3, p3d);
    b.cover.upperLeft.header = bookStack.upperLeft.header;
    b.cover.upperLeft.point.x = intersection.x;
    b.cover.upperLeft.point.y = intersection.y;
    b.cover.upperLeft.point.z = intersection.z;

    p2d = invertPerspective(bookCovers[i].rightEdge.p2, transform);
    camModel.projectPixelTo3dRay(p2d, p3d);
    intersection = findIntersection(p1, p2, p3, p3d);
    b.cover.lowerRight.header = bookStack.lowerRight.header;
    b.cover.lowerRight.point.x = intersection.x;
    b.cover.lowerRight.point.y = intersection.y;
    b.cover.lowerRight.point.z = intersection.z;
		
    p2d = invertPerspective(bookCovers[i].rightEdge.p1, transform);
    camModel.projectPixelTo3dRay(p2d, p3d);
    intersection = findIntersection(p1, p2, p3, p3d);
    b.cover.upperRight.header = bookStack.upperRight.header;
    b.cover.upperRight.point.x = intersection.x;
    b.cover.upperRight.point.y = intersection.y;
    b.cover.upperRight.point.z = intersection.z;
		
    b.cover.lowerLeft.header.stamp = ros::Time::now()-ros::Duration(1.0);
    b.cover.upperLeft.header.stamp = ros::Time::now()-ros::Duration(1.0);
    b.cover.lowerRight.header.stamp = ros::Time::now()-ros::Duration(1.0);
    b.cover.upperRight.header.stamp = ros::Time::now()-ros::Duration(1.0);
    try {
      tfListener->transformPoint("/base_link", b.cover.lowerLeft, b.cover.lowerLeft);
      tfListener->transformPoint("/base_link", b.cover.lowerRight, b.cover.lowerRight);
      tfListener->transformPoint("/base_link", b.cover.upperLeft, b.cover.upperLeft);
      tfListener->transformPoint("/base_link", b.cover.upperRight, b.cover.upperRight);
    } catch (tf::TransformException ex) {
      cout << "Error in bookCallback" << endl;
      ROS_ERROR("%s",ex.what());
    }
    books.oprah.push_back(b);
  }
	ROS_INFO("Before publishing the book");
  pub.publish(books);
}



//---------- find books ----------//

void findBooks()
{
  if (msgCount() < numMsgs)
    return;

  // project bookstack corners into image coordinates
  Point2d ll, ul, lr, ur;
  bookStackCornersToImage(ll, ul, lr, ur);

  // apply projective transform so that bookstack corners are rectangular
  Mat1f I, DI, perspectiveTransform;
  Rect roi;
  rectifyCorners(ll, ul, lr, ur, I, DI, roi, perspectiveTransform);

  // correct for chopped off book bottoms
  roi.height *= 1.3;

  // dbug
  //roi.y -= 10;  
  //roi.height += 10;

  // get edge image points in book stack region
  Mat1b E;
  Mat1b I2;
  I = 256*I;
  I.assignTo(I2, DataType<uchar>::type);
  I = I/256;
  E.create(I2.size());
  Canny(I2,E,100,240); //70,150); 
  vector<Point2i> P = edgePoints(E, roi);

  // show edge image
  namedWindow(fig1);
  imshow(fig1, E);
  waitKey(30);

  // get book tops and bottoms from disparity image
  int *bookTops = new int[DI.cols];
  int *bookBottoms = new int[DI.cols];
  getBookTops(DI, roi, bookTops);
  getBookBottoms(DI, roi, bookBottoms);

  // show book tops and bottoms
  namedWindow(fig2);
  Mat3b tmp(I2.size());
  for (int y=0; y<tmp.rows; y++) {
    for (int x=0; x<tmp.cols; x++) {
      tmp(y,x)[0] = I2(y,x);
      tmp(y,x)[1] = I2(y,x);
      tmp(y,x)[2] = I2(y,x);
    }
  }
  Point *topPoints = new Point[roi.width];
  Point *bottomPoints = new Point[roi.width];
  for (int i=0; i<roi.width; i++) {
    int x = roi.x + i;
    topPoints[i] = Point(x, bookTops[x]);
    bottomPoints[i] = Point(x, bookBottoms[x]);
  }
  polylines(tmp, (const Point**)&topPoints, (const int *)&roi.width, 1, false, CV_RGB(0,255,0));
  polylines(tmp, (const Point**)&bottomPoints, (const int *)&roi.width, 1, false, CV_RGB(0,255,0));
  delete topPoints;
  delete bottomPoints;
  imshow(fig2, tmp);
  waitKey(0);

  // use ransac to find lines, then clip to book stack region
  vector<Line> lines = findLines(P);
  vector<LineSegment> edges;
  //cout << "edges:" << endl;
  for (uint i=0; i<lines.size(); i++) {
    Point p1, p2;
    lineClip(lines[i], roi, p1, p2);
    int x = round((p1.x + p2.x) / 2.0);
    x = MIN(MAX(roi.x, x), roi.x + roi.width - 1);
    int y0 = bookTops[x];
    int y1 = bookBottoms[x];
    //printf("\nx = %d, y0 = %d, y1 = %d\n", x, y0, y1);
    Rect roi2(roi.x, y0, roi.width, y1-y0);
    lineClip(lines[i], roi2, p1, p2);
    edges.push_back(LineSegment(p1,p2));
    //cout << p1.x << " " << p1.y << " " << p2.x << " " << p2.y << endl;
  }
  delete bookTops;
  delete bookBottoms;


  // show line segments
  //namedWindow(fig2);
  //Mat3b tmp(E.size());
  for (int y=0; y<tmp.rows; y++) {
    for (int x=0; x<tmp.cols; x++) {
      tmp(y,x)[0] = E(y,x);
      tmp(y,x)[1] = E(y,x);
      tmp(y,x)[2] = E(y,x);
    }
  }
  for (uint i=0; i<edges.size(); i++)
    line(tmp, edges[i].p1, edges[i].p2, CV_RGB(255,0,0), 2);
  imshow(fig2, tmp);
  waitKey(30);

  // sort edges and recompute lines
  list<LineSegment> edgeList;
  for (uint i=0; i<edges.size(); i++)
    edgeList.push_back(edges[i]);
  edgeList.sort(lineSegmentCompareX1);
  list<LineSegment>::iterator it = edgeList.begin();
  for (uint i=0; i<edges.size(); i++, it++) {
    edges[i] = *it;
    edge2polar(edges[i].p1.x, edges[i].p1.y, edges[i].p2.x, edges[i].p2.y, lines[i].r, lines[i].theta);
  }

  vector<BookCover> books;
  
  for (int iter=0; iter<2; iter++) {

  // search for books between pairs of edges
  for (uint i=0; i<edges.size(); i++) {
    for (uint j=i+1; j<edges.size(); j++) {

      double dr, dtheta;
      bookEdgeDist(lines[i].r, lines[i].theta, lines[j].r, lines[j].theta, dr, dtheta);
      //if (gauss2d_dist(dr, dtheta, 20.0, .025, 46.0, .0004, .1) < 10.0) {
      //if (bookEdgeMatchCost(lines[i], lines[j], 20.0, .025, 46.0, .0004, .1) < 10.0) {
      Point dp1 = edges[i].p1 - edges[j].p1;
      Point dp2 = edges[i].p2 - edges[j].p2;
      double d1 = sqrt(dp1.ddot(dp1));
      double d2 = sqrt(dp2.ddot(dp2));

      if (dr > 10 && dr < 60 && dtheta < .1 && d1 > 10 && d2 > 10) {

	// show edge pair
	for (int y=0; y<tmp.rows; y++) {
	  for (int x=0; x<tmp.cols; x++) {
	    tmp(y,x)[0] = I2(y,x);
	    tmp(y,x)[1] = I2(y,x);
	    tmp(y,x)[2] = I2(y,x);
	  }
	}
	line(tmp, edges[i].p1, edges[i].p2, CV_RGB(255,0,0), 2);
	line(tmp, edges[j].p1, edges[j].p2, CV_RGB(255,0,0), 2);
	imshow(fig1, tmp);
	waitKey(30);

	// get aligned book image between edge pair
	Mat1f BI, BI2;
	imageCropQuad(I, BI, edges[i].p1, edges[i].p2, edges[j].p1, edges[j].p2);
	Point2f center(BI.cols/2, BI.rows/2);
	double theta = angleMean(lines[i].theta, lines[j].theta);
	Mat1f R = getRotationMatrix2D(center, (PI+theta) * 180/PI, 1.0);
	warpAffine(BI, BI2, R, BI.size());
	imageTrim(BI2, BI);
	resize(BI, BI2, Size(20,120));
	BI = BI2;

	// normalize book image
	int cnt = countNonZero(BI);
	double mu = mean(BI)[0] * BI.rows * BI.cols / (double)cnt;
	for (int y=0; y<BI.rows; y++)
	  for (int x=0; x<BI.cols; x++)
	    if (BI(y,x) == 0.0)
	      BI(y,x) = mu;
	Scalar Mu, Sigma;
	meanStdDev(BI, Mu, Sigma);
	BI = (BI - Mu[0])/Sigma[0];

	imshow(fig2, BI+.5);
	waitKey(30);

	// classify book image
	double dmin;
	int bookID = classifyBookImage(BI, bookImages, bookLabels, bookThresholds, dmin);
	cout << "bookID = " << bookID << ", dmin = " << dmin << endl;

	if (bookID != 0)
	  books.push_back(BookCover(bookID, dmin, edges[i], edges[j]));


	if (!doneLabeling) {
	  while (1) {
	    int key = waitKey(0);
	    cout << "key = " << key << endl;
	    if (key == KEY_D) {
	      doneLabeling = true;
	      break;
	    }
	    if (key == KEY_S) {
	      cout << "Saving book images...";
	      saveBookLabels(imdir, bookLabels);
	      saveBookImages(imdir, bookImages);
	      cout << "done!" << endl;

	    }
	    if (key == KEY_ENTER)
	      break;
	    if (key >= KEY_ZERO && key < KEY_ZERO + 10) {
	      cout << "Adding book " << key - KEY_ZERO << " image" << endl;
	      bookImages.push_back(BI);
	      bookLabels.push_back(key - KEY_ZERO);
	      break;
	    }
	  }
	}

      }
    }
  }
  }

  // get a list of the unique books detected
  list<int> bookIDs;
  for (uint i=0; i<books.size(); i++)
    bookIDs.push_back(books[i].id);
  bookIDs.sort();
  bookIDs.unique();
  cout << "Detected books: [ ";
  list<int>::iterator it2;
  for (it2 = bookIDs.begin(); it2 != bookIDs.end(); it2++)
    cout << *it2 << " ";
  cout << "]" << endl;

  // find the best match for each detected book ID, and remove all other matches
  vector<BookCover> uniqueBooks;
  for (it2 = bookIDs.begin(); it2 != bookIDs.end(); it2++) {
    int id = *it2;
    double cmin = 1000000.0;
    double imin = 0;
    for (uint i=0; i<books.size(); i++) {
      if (books[i].id == id && books[i].cost < cmin) {
	cmin = books[i].cost;
	imin = i;
      }
    }
    uniqueBooks.push_back(books[imin]);
  }

  // display unique books
  for (int y=0; y<tmp.rows; y++) {
    for (int x=0; x<tmp.cols; x++) {
      tmp(y,x)[0] = I2(y,x);
      tmp(y,x)[1] = I2(y,x);
      tmp(y,x)[2] = I2(y,x);
    }
  }
  for (uint i=0; i<uniqueBooks.size(); i++) {
    int id = uniqueBooks[i].id;
    Point left1 = uniqueBooks[i].leftEdge.p1;
    Point left2 = uniqueBooks[i].leftEdge.p2;
    Point right1 = uniqueBooks[i].rightEdge.p1;
    Point right2 = uniqueBooks[i].rightEdge.p2;
    right1.x -= 2;  // for non-overlapping display
    right2.x -= 2;  // for non-overlapping display
    line(tmp, left1, left2, bookColors[id-1], 2);
    line(tmp, right1, right2, bookColors[id-1], 2);
  }
  imshow(fig2, tmp);
  waitKey(0);

  // publish list of detected book covers
  if (doneLabeling)
		ROS_INFO("Calling the publisher");
    publishBookList(uniqueBooks, perspectiveTransform);
}


//---------- main loop ----------//

int main(int argc, char** argv)
{
  ros::init(argc, argv, "book_finder");
  ros::NodeHandle n;
  pub = n.advertise<bookbot::BookList>("books", 10);
  ros::Subscriber bboxSub = n.subscribe("bookBox", 10, bboxCallback);
  //ros::Subscriber depthImageSub = n.subscribe("disparity_image", 10, disparityCallback);
  ros::Subscriber depthImageSub = n.subscribe("/narrow_stereo/disparity", 10, disparityCallback);
  //ros::Subscriber cameraInfoSub = n.subscribe("camera_info", 10, cameraInfoCallback);
  ros::Subscriber cameraInfoSub = n.subscribe("/narrow_stereo/left/camera_info", 10, cameraInfoCallback);
  //ros::Subscriber imageSub = n.subscribe("image_rect", 10, imageCallback);
  ros::Subscriber imageSub = n.subscribe("/narrow_stereo/left/image_rect", 10, imageCallback);
  tfListener = new tf::TransformListener;

  if(loadBookLabels(imdir, bookLabels) == 0)
    assert( loadBookImages(imdir, bookLabels.size(), bookImages) == 0 );

  ros::Rate r(10);
  while (ros::ok()) {
    ros::spinOnce();
    findBooks();
    r.sleep();
  }
}
