//ros server to return the current Nano17 sensor readings

#include "ros/node.h"
#include "nano17lib.h"
#include "Nano17interface/LocNormalAndForce.h"
#include "Nano17interface/Values.h"
#include <iostream>

#define NANO17COUNT 3 //how many Nano17s are there?

using namespace std;

Nano17Struct *nano17[NANO17COUNT];


//return the raw sensor values
bool get_nano17_values(Nano17interface::Values::Request &req, Nano17interface::Values::Response &res){
	res.set_values_size(6);
	int values[6];
	nano17_getValues(nano17[req.sensornum], values);
	//printf("values:%d\t%d\t%d\t%d\t%d\t%d\n", values[0], values[1], values[2], values[3], values[4], values[5]);
	int i;
	for(i=0; i<6; i++){
		res.values[i] = values[i];
	}
	return true;
}

//return the location and force of contact
bool get_nano17_loc_normal_and_force(Nano17interface::LocNormalAndForce::Request &req, Nano17interface::LocNormalAndForce::Response &res){
	res.set_loc_size(3);
	res.set_normal_size(3);
	double loc[3];
	double normal[3];
	double forcemag;
	nano17_getLocNormalAndForce(nano17[req.sensornum], loc, normal, &forcemag);
	//printf("normal: %3.2f\t%3.2f\t%3.2f\tloc: %3.2f\t%3.2f\t%3.2f\tforcemag:%3.2f\n", normal[0], normal[1], normal[2], loc[0], loc[1], loc[2], forcemag);
	int i;
	for(i=0; i<3; i++){
		res.loc[i] = loc[i];
		res.normal[i] = normal[i];
	}
	res.forcemag = forcemag;
	return true;
}


int main(int argc, char **argv){

	string portnames[NANO17COUNT];
	portnames[0] = string("/dev/ttyUSB3");
	if(NANO17COUNT > 1) portnames[1] = string("/dev/ttyUSB2");
	if(NANO17COUNT > 2) portnames[2] = string("/dev/ttyUSB1");

 	//Initialize ROS
	ros::init(argc, argv);

	//Create a ROS node
	ros::Node n("Nano17Server");

	//Start up all the Nano17s
	int sensornum;	
	for(sensornum=0; sensornum<NANO17COUNT; sensornum++){
		printf("opening Nano: %d on %s", sensornum, portnames[sensornum].c_str());
		nano17[sensornum] = nano17_open(portnames[sensornum].c_str());
	}

	//Advertise services
	n.advertiseService("get_nano17_values", &get_nano17_values);
	n.advertiseService("get_nano17_loc_normal_and_force", &get_nano17_loc_normal_and_force);


	//Wait for node to finish
	printf("waiting for clients\n");
	n.spin();


	//Unadvertise services
	n.unadvertise("get_nano17_values");
	n.unadvertise("get_nano17_loc_normal_and_force");

	//Close the Nano reader threads
	for(sensornum=0; sensornum<NANO17COUNT; sensornum++){
		nano17_close(nano17[sensornum]);
	}
	printf("nano17server is done!\n");

	return 0;
}
