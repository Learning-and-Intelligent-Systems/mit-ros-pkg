/*
 * object_analysis.cpp
 *
 *  Created on: Oct 17, 2010
 *      Author: garratt
 */

//this program does some of the heavy analysis so I can test the data on other parts of the algorithm

//#include <furniture_ops/pcl_helpers.hpp>
#include "ScanAnalyzer.h"


int main(int argc, char **argv) {
	if(argc!=3){
		std::cout<<"USAGE "<<argv[0]<<" file_in.bag file_in.bag "<<std::endl;
		return -1;
	}

	ros::init(argc, argv, "initial_processing");
	ScanAnalyzer analyzer(argv[1]);
	ROS_INFO("writing to bag file %s",argv[2]);
	analyzer.writeBag(argv[2]);

	return 0;
}





