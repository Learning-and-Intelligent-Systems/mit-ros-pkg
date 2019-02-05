#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>

IplImage* DrawHistogram(CvHistogram *hist, float scaleX=1, float scaleY=1) {
  float histMax = 0;
  cvGetMinMaxHistValue(hist, 0, &histMax, 0, 0);
  IplImage* imgHist = cvCreateImage(cvSize(256*scaleX, 64*scaleY), 8 ,1);
  cvZero(imgHist);
  for(int i=0;i<255;i++) {
    float histValue = cvQueryHistValue_1D(hist, i);
    float nextValue = cvQueryHistValue_1D(hist, i+1);
    
    CvPoint pt1 = cvPoint(i*scaleX, 64*scaleY);
    CvPoint pt2 = cvPoint(i*scaleX+scaleX, 64*scaleY);
    CvPoint pt3 = cvPoint(i*scaleX+scaleX, (64-nextValue*64/histMax)*scaleY);
    CvPoint pt4 = cvPoint(i*scaleX, (64-histValue*64/histMax)*scaleY);
    
    int numPts = 5;
    CvPoint pts[] = {pt1, pt2, pt3, pt4, pt1};
    
    cvFillConvexPoly(imgHist, pts, numPts, cvScalar(255));
  }
  return imgHist;
}

int main(int argc, char **argv) {

  char file_in[100] = "images/";
  strcat(file_in, argv[1]);

  IplImage* img = cvLoadImage(file_in);
  int numBins = 256;
  float range[] = {0, 255};
  float *ranges[] = { range };
 
  CvHistogram *hist = cvCreateHist(1, &numBins, CV_HIST_ARRAY, ranges, 1);
  cvClearHist(hist);

  IplImage* imgRed = cvCreateImage(cvGetSize(img), 8, 1);
  IplImage* imgGreen = cvCreateImage(cvGetSize(img), 8, 1);
  IplImage* imgBlue = cvCreateImage(cvGetSize(img), 8, 1);
   
  cvSplit(img, imgBlue, imgGreen, imgRed, NULL);
  cvCalcHist(&imgRed, hist, 0, 0);
  IplImage* imgHistRed = DrawHistogram(hist);
  cvClearHist(hist);
  
  cvCalcHist(&imgGreen, hist, 0, 0);
  IplImage* imgHistGreen = DrawHistogram(hist);
  cvClearHist(hist);
  
  cvCalcHist(&imgBlue, hist, 0, 0);
  IplImage* imgHistBlue = DrawHistogram(hist);
  cvClearHist(hist);

  cvNamedWindow("Red");
  cvNamedWindow("Green");
  cvNamedWindow("Blue");
  
  cvShowImage("Red", imgHistRed);
  cvShowImage("Green", imgHistGreen);
  cvShowImage("Blue", imgHistBlue);
  
  cvWaitKey(0);
  cvReleaseImage(&img);
  cvReleaseImage(&imgRed);
  cvReleaseImage(&imgGreen);
  cvReleaseImage(&imgBlue);
}
