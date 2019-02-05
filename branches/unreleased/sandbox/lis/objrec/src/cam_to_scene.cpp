/*
 * cam_to_scene.cpp
 *
 *  Created on: Apr 16, 2014
 */

#include "constraints.h"

int main(int argc, char** argv) {
	if(argc<2) {
		std::cout << "usage:" << std::endl;
		std::cout << argv[0] << " table_height 0001.cam 0002.cam ..." << std::endl;
		return -1;
	}
	std::istringstream table_height_stream(argv[1]);
	float table_height;
	table_height_stream >> table_height;

	for(int j=2;j<argc;j++) {
		std::ifstream in(argv[j]);
		if(!in) {
			std::cout << "File " << argv[j] << " not found." << std::endl;
			return -1;
		}
		objrec::camera_info ci;
		objrec::read_camera_info(in,ci);

		objrec::scene_info si;
		si.camera_height_above_floor = ci.pos.z;
		si.camera_tilt_angle = atan2(-ci.rot.m31,sqrt(ci.rot.m32*ci.rot.m32+ci.rot.m33*ci.rot.m33))*180.0/M_PI;
		si.table_height = table_height;

		std::cout << "camera info: " << ci << std::endl;
		std::cout << "scene info: " << si << std::endl;

		std::string cam_file = argv[j];
		size_t dot = cam_file.find_last_of('.');
		std::string scene_file = cam_file.substr(0,dot)+".scene";
		std::ofstream out(scene_file.c_str());
		out << "#" << objrec::scene_info_description << std::endl;
		out << si << std::endl;
	}
}
