#include <iostream>
#include <cv.h>
#include <highgui.h>

#include <ros/ros.h>
//#include <bookbot/BBox.h>
//#include <bookbot/BookList.h>
#include <cv_bridge/CvBridge.h>
#include <geometry_msgs/Point.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <stereo_msgs/DisparityImage.h>
//#include <tf/transform_listener.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/ModelCoefficients.h>
//#include <pcl/point_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>

#include <posedetection_msgs/ImageFeature0D.h>


// ProjectTfFrameToImage
// image_geometry::StereoCameraModel


using namespace std;


class SiftViewer{
private:
	ros::NodeHandle _n;
	ros::Subscriber _sub;
	sensor_msgs::CvBridge _imBridge;
	cv::Mat _image;
public:
	SiftViewer(){
		_sub = _n.subscribe("ImageFeature0D", 10, &SiftViewer::siftCallback,this);
	}
	void siftCallback(const posedetection_msgs::ImageFeature0DConstPtr& I_ptr)
	{
		cout<<"got image"<<endl;
	  _imBridge.fromImage(I_ptr->image);

	  IplImage *I_ipl = _imBridge.toIpl();  //imgMsgToCv(I_ptr->image);
	  IplImage *temp;
	  temp = cvCreateImage( cvSize(I_ipl->width,I_ipl->height), 8, 3 );
	  cvCvtColor( I_ipl, temp, CV_GRAY2BGR );

	  cv::Mat I(temp);
	  I.copyTo(_image);


	  cv::namedWindow("Sift Features");
	  cv::Point2d pt;
	  for(int f=0;f<I_ptr->features.scales.size();f++){
		  pt.x=I_ptr->features.positions[2*f+0];
		  pt.y=I_ptr->features.positions[2*f+1];
		  cv::circle(_image, pt, I_ptr->features.scales[f], CV_RGB(255,0,0), 1);
	  }
	  cv::imshow("Sift Features", _image);
	  cv::waitKey(10);
//	  sleep(10);
//	  cvShowImage( "result", _image );
	}

};





//---------- main loop ----------//

int main(int argc, char** argv)
{
  ros::init(argc, argv, "book_finder");
  ros::NodeHandle n;
  SiftViewer sv;
  while (ros::ok()) {
    ros::spin();
  }
}
