/*
 * filterFeatures.cpp
 *
 *  Created on: Oct 24, 2010
 *      Author: garratt
 */

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/message_instance.h"
#include <boost/foreach.hpp>


#include <posedetection_msgs/ImageFeature0D.h>

std::vector<posedetection_msgs::Feature0D> feats;


template <typename type1>
void appendvec(std::vector<type1> &out, std::vector<type1> &in){
	out.insert(out.end(),in.begin(),in.end());
}


void appendFeatures(posedetection_msgs::Feature0D &out, std::vector< posedetection_msgs::Feature0D> feats){
	for(uint i=0; i< feats.size(); i++){
		   appendvec(out.confidences,feats[i].confidences);
		   appendvec(out.orientations,feats[i].orientations);
		   appendvec(out.positions,feats[i].positions);
		   appendvec(out.descriptors,feats[i].descriptors);
		   appendvec(out.scales,feats[i].scales);
	}
	out.descriptor_dim = feats.front().descriptor_dim;
	out.type=feats.front().type;
	out.header=feats[0].header;
}

void writeFeatures(std::string filename){
    rosbag::Bag bag;
    bag.open(filename, rosbag::bagmode::Write);

//    for(uint i=0; i<feats.size();i++)
//    	bag.write("sift_features", feats[i].header.stamp, feats[i]);
    posedetection_msgs::Feature0D onefeat;
    appendFeatures(onefeat,feats);
    bag.write("sift_features", onefeat.header.stamp, onefeat);

    bag.close();
}


int readBag(std::string filename){
	rosbag::Bag bag(filename);
	int featurenum=0;
	rosbag::TypeQuery query("posedetection_msgs/ImageFeature0D");
	rosbag::View view(bag,query,ros::TIME_MIN,ros::TIME_MAX);
	ROS_INFO("Found %d ImageFeature0D messages. Reading...",view.size());
	boost::shared_ptr <posedetection_msgs::ImageFeature0D> imagefeat;
	BOOST_FOREACH(rosbag::MessageInstance m, view){
		if(imagefeat = m.instantiate<posedetection_msgs::ImageFeature0D>()){
			feats.push_back(imagefeat->features);
			featurenum+=imagefeat->features.confidences.size();
			ROS_INFO("%d features",featurenum);

		}
	}
	if(feats.size() == 0){
		ROS_ERROR("Error reading Image features!");
		return -2;
	}
	return 0;

}


int main(int argc, char **argv) {
	if(argc<2)
		std::cout<<"USAGE: file_in file_out"<<std::endl;
	readBag(argv[1]);
	writeFeatures(argv[2]);

}












