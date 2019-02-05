/*
Node to output throttled versions of the openni camera rgb and depth images to lcm (which need to stay approxiamately synchronized)
Maintained: mfallon. Derived from version by hordurj
*/

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h> 

#include <zlib.h>



#include <lcm/lcm.h>
//#include <bot/core/lcmtypes/lcmtypes.h>
//#include <bot_core/bot_core.h>
//#include <bot_frames/bot_frames.h>

#include <boost/concept_check.hpp>

#include <lcmtypes/openni_lcm.h>

using std::string;
using namespace sensor_msgs;
using namespace message_filters;
using namespace std;

//typedef sync_policies::ExactTime<Image, Image> StereoCameraSyncPolicy;

int64_t tic ;
int64_t toc;

typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;

// class for broadcasting synchronized, throttled versions of a stereo camera topic
class OpenniCameraThrottler
{
private:
  ros::NodeHandle node_;    
  ros::Publisher depth_image_pub_;  
  ros::Publisher rgb_image_pub_;  
  ros::Publisher depth_camera_info_pub_;  
  ros::Publisher rgb_camera_info_pub_;  
  message_filters::Subscriber<Image> depth_sub_;
  message_filters::Subscriber<Image> rgb_sub_;
  Synchronizer<MySyncPolicy> sync_;
  ros::Subscriber depth_info_sub_;
  ros::Subscriber rgb_info_sub_;
  CameraInfo depth_cam_info_;
  CameraInfo rgb_cam_info_;
  bool received_depth_cam_info_;
  bool received_rgb_cam_info_;
  double msg_rate_;
  double last_msg_time_;
  string image_topic_;

  lcm_t* lcm_;  
  
  int decimate_;
  
public:
//    sync_(MySyncPolicy(10), depth_sub_, rgb_sub_)

  OpenniCameraThrottler(string image_topic, string depth_topic, double rate,int decimate):
    depth_sub_(node_, depth_topic, 1),
    rgb_sub_(node_, image_topic, 1),
    sync_(MySyncPolicy(10), depth_sub_, rgb_sub_)
  {
    lcm_ = lcm_create(NULL);    


    decimate_ = decimate;
    
    //store the topic and initial rate and initialize the last message time
    image_topic_ = image_topic;
    msg_rate_ = rate;
    last_msg_time_ = 0;
    received_depth_cam_info_ = 0;
    received_rgb_cam_info_ = 0;
    
    //subscribe to the left and right camera info messages 
    depth_info_sub_ = node_.subscribe(string("/camera/depth/camera_info"), 5, &OpenniCameraThrottler::depth_info_cb, this);
    rgb_info_sub_ = node_.subscribe(string("/camera/rgb/camera_info"), 5, &OpenniCameraThrottler::rgb_info_cb, this);
							
    //register the callback for the topic synchronizer
    sync_.registerCallback(boost::bind(&OpenniCameraThrottler::stereo_cb, this, _1, _2));

    //broadcast the throttled versions on out_topic
    //depth_image_pub_ = node_.advertise<Image>(out_topic + string("/left/image_raw"), 10);
    //rgb_image_pub_ = node_.advertise<Image>(out_topic + string("/right/image_raw"), 10);
    //depth_camera_info_pub_ = node_.advertise<CameraInfo>(out_topic + string("/left/camera_info"), 10);
    //rgb_camera_info_pub_ = node_.advertise<CameraInfo>(out_topic + string("/right/camera_info"), 10);

    ROS_INFO("done init for %s", image_topic.c_str());
    ROS_INFO("done init for %s", depth_topic.c_str());
  }
  
  //record the left camera info (once)
  void depth_info_cb(const CameraInfoConstPtr& cam_info)
  {
    if(received_depth_cam_info_) return;
    depth_cam_info_ = *cam_info;
    received_depth_cam_info_ = 1;
    //depth_camera_info_pub_.publish(depth_cam_info_);
    ROS_INFO("got depth_cam_info for %s", image_topic_.c_str());
  }

  //record the right camera info (once)
  void rgb_info_cb(const CameraInfoConstPtr& cam_info)
  {
    if(received_rgb_cam_info_) return;
    rgb_cam_info_ = *cam_info;
    received_rgb_cam_info_ = 1;
    //rgb_camera_info_pub_.publish(rgb_cam_info_);
    ROS_INFO("got rgb_cam_info for %s", image_topic_.c_str());
  }
				   

  //re-broadcast the stereo images and their info, throttled to the desired frequencies
  void stereo_cb(const ImageConstPtr& depth_image, const ImageConstPtr& rgb_image)
  {
    //ROS_INFO("got synchronized image for %s", image_topic_.c_str());
    
    if(ros::Time::now().toSec() - last_msg_time_ > 1/msg_rate_)
    {
      openni_frame_msg_t frame;

      struct timeval tv;
      gettimeofday (&tv, NULL);
      tic = (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
      double elapsed_t = (double (tic - toc))/1000000;
      std::cout << elapsed_t << "sec top\n";
      
      // RGB Image:
      openni_image_msg_t oimage;
      const int* tptr = (const int*)(&(rgb_image->header.stamp));
      int secs = *tptr;
      int nsecs = *(tptr+1);
      oimage.timestamp = secs * 1E6 + nsecs/1E3;
      oimage.width =rgb_image->width/decimate_;
      oimage.height=rgb_image->height/decimate_;
      oimage.pixel_offset[0] = 1;
      oimage.pixel_offset[1] =1;
      oimage.image_data_nbytes= 3*oimage.width*oimage.height;
      oimage.image_data= new unsigned char[oimage.image_data_nbytes];
      // Decimate and copy:
      //for (size_t i=0; i<oimage.image_data_nbytes; i++) oimage.image_data[i] = rgb_image->data[i];      
      for (int j=0,k=0,count=0; j<rgb_image->height; j=j+decimate_){ // height
	for (int i=0; i<rgb_image->width; i=i+decimate_){ // width
	  k = (j*rgb_image->width+ i)*3;
	  oimage.image_data[count] = rgb_image->data[k];      
	  oimage.image_data[count+1] = rgb_image->data[k+1];
	  oimage.image_data[count+2] = rgb_image->data[k+2];      
	  count=count+3;
	}
      }
      oimage.image_data_format =0;

      // Depth:
      openni_depth_msg_t odepth;
      const int* tptr_d = (const int*)(&(depth_image->header.stamp));
      int secsd = *tptr_d;
      int nsecsd = *(tptr_d+1);
      odepth.timestamp = secsd * 1E6 + nsecsd/1E3;
      odepth.timestamp = 1;
      odepth.width =depth_image->width/decimate_;
      odepth.height=depth_image->height/decimate_;
      odepth.pixel_offset[0] = 1;
      odepth.pixel_offset[1] =1;
      
      odepth.depth_data_nbytes= 4*odepth.width*odepth.height;
      int use_zlib=0;
      if(use_zlib) { // this doesnt work...
	int uncompressed_size = odepth.depth_data_nbytes;
	
	// allocate space for unpacking depth data
	uint16_t* depth_unpack_buf;
	int depth_unpack_buf_size;
	
	uint8_t* depth_compress_buf;
	int depth_compress_buf_size;

	depth_unpack_buf_size = 640 * 480 * sizeof(uint16_t);
	depth_unpack_buf = (uint16_t*) malloc(depth_unpack_buf_size);
	
	depth_compress_buf_size = 640 * 480 * sizeof(int16_t) * 4;
	depth_compress_buf = (uint8_t*) malloc(depth_compress_buf_size);
	
	unsigned long compressed_size = depth_compress_buf_size;
	
	compress2(depth_compress_buf, &compressed_size, (const Bytef*)depth_unpack_buf, uncompressed_size, Z_BEST_SPEED);
	odepth.depth_data = depth_compress_buf;
	odepth.depth_data_nbytes = (int)compressed_size;
	odepth.compression = OPENNI_DEPTH_MSG_T_COMPRESSION_ZLIB;
	odepth.uncompressed_size = uncompressed_size;
      } else {
	
	odepth.depth_data= new unsigned char[odepth.depth_data_nbytes];
	//for (size_t i=0; i<odepth.depth_data_nbytes; ++i) odepth.depth_data[i] = depth_image->data[i];      
	for (int j=0,k=0,count=0; j<depth_image->height; j=j+decimate_){ // height
	  for (int i=0; i<depth_image->width; i=i+decimate_){ // width
	    k = (j*depth_image->width+ i)*4;
	    odepth.depth_data[count] = depth_image->data[k];      
	    odepth.depth_data[count+1] = depth_image->data[k+1];
	    odepth.depth_data[count+2] = depth_image->data[k+2];      
	    odepth.depth_data[count+3] = depth_image->data[k+3];      
	    count=count+4;
	  }
	}	
	
	odepth.compression =OPENNI_DEPTH_MSG_T_COMPRESSION_NONE;
	odepth.uncompressed_size =odepth.depth_data_nbytes;	
      }
      odepth.viewpoint =1;
      odepth.focal_length = -999.999;
      
      // Publish:
      frame.depth = odepth;
      frame.image = oimage;
      frame.timestamp = oimage.timestamp;
      openni_frame_msg_t_publish(lcm_, "OPENNI_FRAME", &frame);

      // timing junk:
      cout << frame.timestamp  
      << " | IMAGE: " << (rgb_image->step*rgb_image->height) 
      << " | DEPTH: " << (depth_image->step*depth_image->height) << "\n";
      struct timeval tv2;
      gettimeofday (&tv2, NULL);
      toc= (int64_t) tv2.tv_sec * 1000000 + tv2.tv_usec;      
      double elapsed_b = (double (toc - tic))/1000000;
      std::cout << elapsed_b << "sec bottom\n";

      delete [] oimage.image_data;
      delete [] odepth.depth_data;
      last_msg_time_ = ros::Time::now().toSec();
      ROS_INFO("broadcasting synchronized openni data for %s", image_topic_.c_str());
    }
  }

};


int main(int argc, char** argv)
{
  if(argc < 3)
  {
    puts("usage: openni2lcm OPENNI_FRAME_RATE DECIMATE\n\n");
    return 1;
  }
  // decimate should be an int. usually 16
  
  double openni_rate = atof(argv[1]);
  int decimate = atoi(argv[2]);
  std::cout << "openni2lcm throttle rate: " <<openni_rate << "\n";
  std::cout << "openni2lcm decimate: " <<decimate << "\n";

  ros::init(argc, argv, "throttle_openni_data");

  //create throttler-synchronizers for the wide, narrow, and narrow_textured stereo cameras
  OpenniCameraThrottler *openni_throttler = new OpenniCameraThrottler(string("/camera/rgb/image_color"), 
					     string("/camera/depth/image"), openni_rate, decimate);

  ros::spin();

  delete openni_throttler;

  return 0;
}
