//test client for nano17 ROS interface

#include <iostream>
#include "ros/node.h"                //All roscpp nodes will need this
#include "Nano17interface/LocNormalAndForce.h"
#include "Nano17interface/Values.h"

#define NANO17COUNT 3

//call the ROS service get_nano17_values
void call_get_nano17_values(int sensornum, int values[6]){
	int i;
	Nano17interface::Values::Request req;
	Nano17interface::Values::Response res;
	req.sensornum = sensornum;

	if(ros::service::call("get_nano17_values", req, res)){
		for(i=0; i<(int)res.get_values_size(); i++){
			values[i] = res.values[i];
		}
	}
}

//call the ROS service get_nano17_loc_and_force
void call_get_nano17_loc_normal_and_force(int sensornum, double loc[3], double normal[3], double *forcemag){
	int i;
	Nano17interface::LocNormalAndForce::Request req;
	Nano17interface::LocNormalAndForce::Response res;
	req.sensornum = sensornum;
	
	if(ros::service::call("get_nano17_loc_and_force", req, res)){
		for(i=0; i<(int)res.get_loc_size(); i++){
			loc[i] = res.loc[i];
		}
		for(i=0; i<(int)res.get_normal_size(); i++){
			normal[i] = res.normal[i];
		}
		*forcemag = res.forcemag;
	}
}

int main(int argc, char **argv)
{
  // Initialize ros
  ros::init(argc, argv);
	ros::Node n("Nano17Client");

	int sensornum;
	int values[6];
	double loc[3];
	double normal[3];
	double forcemag;
	while(1){
		for(sensornum=0; sensornum<NANO17COUNT; sensornum++){
			call_get_nano17_values(sensornum, values);
			
			call_get_nano17_loc_normal_and_force(sensornum, loc, normal, &forcemag);
			printf("sensornum: %d\n", sensornum);
			printf("values:%d\t%d\t%d\t%d\t%d\t%d\n", values[0], values[1], values[2], values[3], values[4], values[5]);
			printf("normal: %3.2f\t%3.2f\t%3.2f\tloc: %3.2f\t%3.2f\t%3.2f\tforcemag:%3.2f\n", normal[0], normal[1], normal[2], loc[0], loc[1], loc[2], forcemag);
			
			usleep(100000);
		}
	}

	printf("nano17client is done!\n");
	return 0;
}
			
