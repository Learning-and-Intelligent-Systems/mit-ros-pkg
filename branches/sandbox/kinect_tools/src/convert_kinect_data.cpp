/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Garratt Gallagher
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name Garratt Gallagher nor the names of other
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/



#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <kinect_tools/CloudWithViewpoint.h>
#include <sensor_msgs/point_cloud_conversion.h>


class LaserStamper
{

private:
  tf::TransformListener tl_;
  ros::NodeHandle n_;
  ros::Publisher pub_,ppub_,lpub_;
  ros::Subscriber sub_;
  std::string fixedframe;

public:

  LaserStamper():tl_(ros::Duration(120.0))
  {

    pub_ = n_.advertise<kinect_tools::CloudWithViewpoint> ("cloud_with_trans", 1);

    ros::Duration(0.5).sleep();

    sub_=n_.subscribe("/kinect/cloud", 1, &LaserStamper::lasercb, this);

  }

  void lasercb(const sensor_msgs::PointCloudConstPtr &scan){
	  sensor_msgs::PointCloud cloudout;
	  tf::StampedTransform trans;
	  kinect_tools::CloudWithViewpoint cwv;
	  ROS_INFO("Got cloud with %d points, at time %f",scan->points.size(),scan->header.stamp.toSec());
	  if(tl_.waitForTransform (scan->header.frame_id, "/odom", scan->header.stamp,ros::Duration(.3),ros::Duration(.001))){
	     tl_.lookupTransform(scan->header.frame_id,"/odom", scan->header.stamp,trans);
//		  tl_.transformPointCloud("/odom",*scan,cloudout);
	     sensor_msgs::convertPointCloudToPointCloud2(*scan,cwv.cloud);

        tf::transformTFToMsg(trans,cwv.transform);
		  pub_.publish(cwv);
	  }
	  else
		  ROS_ERROR("transform failed");
  }

} ;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_stamper");
  ros::NodeHandle n;
  LaserStamper snapshotter;
  ros::spin();
  return 0;
}
