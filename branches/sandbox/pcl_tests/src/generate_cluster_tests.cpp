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


//this program extracts parts of a SurveyScan's point clouds and saves them to pcd files for the testcluster program
#include <furniture_ops/pcl_helpers.hpp>

#include "ScanAnalyzer.h"





int main(int argc, char **argv) {
	if(argc<2){
		std::cout<<"USAGE "<<argv[0]<<" filename.bag "<<std::endl;
		return -1;
	}



	ros::init(argc, argv, "survey_analyzer");
	ScanAnalyzer analyzer(argv[1]);

	boost::shared_ptr <pcl::PointCloud<pcl::PointXYZINormal> > ncloud;

	 pcl::PointCloud<pcl::PointXYZINormal> smallcloud;
	 char filename[500];
	for(uint i=0;i<analyzer.getNumScans();i++){
		ncloud=analyzer.getNormalCloud(i);
		std::cout<<" got cloud "<<i<<", size: "<<ncloud->size()<<std::endl;
		 for(float f=2.0; f>0.0; f-=.1){
			 segmentHeight(*ncloud,smallcloud,f);
			 sprintf(filename,"testlogs/testcloud%02d_%01.1f.pcd",i,f);
			 pcl::io::savePCDFile(filename,smallcloud);
			 std::cout<<"cloud "<<i<<" height: "<<f<<": size: "<<smallcloud.size()<<std::endl;
		 }
	}

	return 0;


}





