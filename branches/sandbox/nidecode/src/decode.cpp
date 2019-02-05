/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
 *    Suat Gedikli <gedikli@willowgarage.com>
 *    Patrick Michelich <michelich@willowgarage.com>
 *    Radu Bogdan Rusu <rusu@willowgarage.com>
 *
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 */


#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


void publishXYZRGBPointCloud (const sensor_msgs::Image& depth_msg, const sensor_msgs::Image& rgb_msg) const
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZRGB>() );
  cloud_msg->header.stamp     = depth_msg.header.stamp;
  cloud_msg->header.frame_id  = rgb_msg.header.frame_id;
  cloud_msg->height           = depth_msg.height;
  cloud_msg->width            = depth_msg.width;
  cloud_msg->is_dense         = false;

  // do not publish if rgb image is smaller than color image -> seg fault
  if (rgb_msg.height < depth_msg.height || rgb_msg.width < depth_msg.width)
  {
    // we dont want to flood the terminal with warnings
    static unsigned warned = 0;
    if (warned % 100 == 0)
      NODELET_WARN("rgb image smaller than depth image... skipping point cloud for this frame rgb:%dx%d vs. depth:%3dx%d"
              , rgb_msg.width, rgb_msg.height, depth_msg.width, depth_msg.height );
    ++warned;
    return;
  }
  cloud_msg->points.resize (cloud_msg->height * cloud_msg->width);

  float constant = 0.00190476;
        //1.0f / device_->getImageFocalLength (cloud_msg->width);
  float centerX = (cloud_msg->width >> 1) - 0.5f;
  float centerY = (cloud_msg->height >> 1) - 0.5f;
  const float* depth_buffer = reinterpret_cast<const float*>(&depth_msg.data[0]);
  const uint8_t* rgb_buffer = &rgb_msg.data[0];

  // depth_msg already has the desired dimensions, but rgb_msg may be higher res.
  unsigned color_step = 3 * rgb_msg.width / cloud_msg->width;
  unsigned color_skip = 3 * (rgb_msg.height / cloud_msg->height - 1) * rgb_msg.width;
  int color_idx = 0, depth_idx = 0;
  pcl::PointCloud<pcl::PointXYZRGB>::iterator pt_iter = cloud_msg->begin ();
  for (int v = 0; v < (int)cloud_msg->height; ++v, color_idx += color_skip)
  {
    for (int u = 0; u < (int)cloud_msg->width; ++u, color_idx += color_step, ++depth_idx, ++pt_iter)
    {
      pcl::PointXYZRGB& pt = *pt_iter;
      float Z = depth_buffer[depth_idx];

      // Check for invalid measurements
      if (std::isnan (Z))
      {
        pt.x = pt.y = pt.z = Z;
      }
      else
      {
        // Fill in XYZ
        pt.x = (u - centerX) * Z * constant;
        pt.y = (v - centerY) * Z * constant;
        pt.z = Z;
      }

      // Fill in color
      RGBValue color;
      color.Red   = rgb_buffer[color_idx];
      color.Green = rgb_buffer[color_idx + 1];
      color.Blue  = rgb_buffer[color_idx + 2];
      color.Alpha = 0;
      pt.rgb = color.float_value;
    }
  }

  pub_point_cloud_rgb_.publish (cloud_msg);
}




class Decoder
{

private:
  ros::NodeHandle n_;
  ros::Publisher cloudpub;
  ros::Subscriber imagesub_,depthsub_;


public:

  Decoder()
  {
   cloudpub = n_.advertise<sensor_msgs::PointCloud2> ("cloudout", 1);
   imagesub_=n_.subscribe("/camera/rgb/color_image", 1, &Decoder::imagecb, this);
   depthsub_=n_.subscribe("/camera/depth/image", 1, &Decoder::depthcb, this);
  }





};




int main(int argc, char **argv)
{
  ros::init(argc, argv, "nidecoder");
  ros::NodeHandle n;
  Decoder decoder;
  ros::spin();
  return 0;
}




