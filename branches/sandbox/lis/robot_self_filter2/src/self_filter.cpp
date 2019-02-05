/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/** \author Ioan Sucan */

#include <ros/ros.h>
#include <sstream>
#include "robot_self_filter/self_see_filter.h"
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

    
class SelfFilter
{
public:

  SelfFilter(void): nh_("~")
  {
    nh_.param<std::string>("sensor_frame", sensor_frame_, std::string());
    self_filter_ = new filters::SelfFilter<sensor_msgs::PointCloud>(nh_);

    sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud>(root_handle_, "cloud_in", 1);	
    mn_ = new tf::MessageFilter<sensor_msgs::PointCloud>(*sub_, tf_, "", 1);

    //mn_ = new tf::MessageNotifier<sensor_msgs::PointCloud>(tf_, boost::bind(&SelfFilter::cloudCallback, this, _1), "cloud_in", "", 1);
    pointCloudPublisher_ = root_handle_.advertise<sensor_msgs::PointCloud>("cloud_out", 1);
    std::vector<std::string> frames;
    self_filter_->getSelfMask()->getLinkNames(frames);
    if(frames.empty())
    {
      ROS_DEBUG("No valid frames have been passed into the self filter. Using a callback that will just forward scans on.");
      no_filter_sub_ = root_handle_.subscribe<sensor_msgs::PointCloud>("cloud_in", 1, boost::bind(&SelfFilter::noFilterCallback, this, _1));
    }
    else
    {
      ROS_DEBUG("Valid frames were passed in. We'll filter them.");
      mn_->setTargetFrames(frames);
      mn_->registerCallback(boost::bind(&SelfFilter::cloudCallback, this, _1));
    }
  }
    
  ~SelfFilter(void)
  {
    delete self_filter_;
    delete mn_;
    delete sub_;
  }
    
private:

  void noFilterCallback(const sensor_msgs::PointCloudConstPtr &cloud){
    pointCloudPublisher_.publish(cloud);
    ROS_DEBUG("Self filter publishing unfiltered frame");
  }
    
  void cloudCallback(const sensor_msgs::PointCloudConstPtr &cloud)
  {
    sensor_msgs::PointCloud out;
      
    ROS_DEBUG("Got pointcloud that is %f seconds old", (ros::Time::now() - cloud->header.stamp).toSec());
    std::vector<int> mask;
    ros::WallTime tm = ros::WallTime::now();
      
    self_filter_->updateWithSensorFrame(*cloud, out, sensor_frame_);
      
    double sec = (ros::WallTime::now() - tm).toSec();

    pointCloudPublisher_.publish(out);
    ROS_DEBUG("Self filter: reduced %d points to %d points in %f seconds", (int)cloud->points.size(), (int)out.points.size(), sec);
  }

  tf::TransformListener                                 tf_;
  //tf::MessageNotifier<sensor_msgs::PointCloud>           *mn_;
  ros::NodeHandle                                       nh_, root_handle_;

  tf::MessageFilter<sensor_msgs::PointCloud>           *mn_;
  message_filters::Subscriber<sensor_msgs::PointCloud> *sub_;

  filters::SelfFilter<sensor_msgs::PointCloud> *self_filter_;
  std::string sensor_frame_;

  ros::Publisher                                        pointCloudPublisher_;
  ros::Subscriber                                       no_filter_sub_;
  
};

    
int main(int argc, char **argv)
{
  ros::init(argc, argv, "self_filter");

  SelfFilter s;
  ros::spin();
    
  return 0;
}
