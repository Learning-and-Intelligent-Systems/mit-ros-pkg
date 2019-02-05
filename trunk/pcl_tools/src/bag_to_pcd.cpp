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


#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/message_instance.h"
#include <boost/foreach.hpp>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <vector>
#include <fstream>

//this function just grabs the first pointcloud2 it sees.
int readBag(std::string filename, sensor_msgs::PointCloud2 &cloud){
	rosbag::Bag bag(filename);
	//check for PointCloud2s in the bag:
	rosbag::TypeQuery query("sensor_msgs/PointCloud2");
	rosbag::View view(bag,query,ros::TIME_MIN,ros::TIME_MAX);
	if(view.size() > 0){
		ROS_INFO("Found %d PointCloud2 messages. Reading...",view.size());
		boost::shared_ptr <sensor_msgs::PointCloud2> scan;
		BOOST_FOREACH(rosbag::MessageInstance m, view){
			if(scan = m.instantiate<sensor_msgs::PointCloud2>()){
				cloud=*scan;
				ROS_INFO("%d by %d points in cloud",cloud.width,cloud.height);
//				return 0;
			}
		}
		ROS_ERROR("Error reading scans!");
		return -2;
	}
	ROS_ERROR("No PointCloud2 Messages!");
	return -1;
}

int main(int argc, char **argv) {
	if(argc<2)
		return -1;
	sensor_msgs::PointCloud2 cloud2;
	readBag(argv[1],cloud2);
	pcl::io::savePCDFile("sample.pcd",cloud2);
	return 0;
}




