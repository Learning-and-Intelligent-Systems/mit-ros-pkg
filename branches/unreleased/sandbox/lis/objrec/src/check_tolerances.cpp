/*
 * check_tolerances.cpp
 *
 *  Created on: May 28, 2014
 */

#include "constraints.h"
#include <fstream>

int main(int argc, char** argv) {
	if(argc<3) {
		std::cerr << "usage:" << std::endl;
		std::cerr << " " << std::string(argv[0]) << " object_file image_file1 image_file2 ..." << std::endl;
		std::cerr << std::endl;
		std::cerr << " Checks the tolerances of the table height and camera angle given labeled object poses." << std::endl;
		return -1;
	}
	std::string object_file = argv[1];

	objrec::object obj;
	{
		std::ifstream in(object_file.c_str());
		if(!in) throw std::runtime_error("Could not open object file '"+object_file+"'.");
		in >> obj;
	}
	float max_ddepth = -std::numeric_limits<float>::infinity(), min_ddepth = std::numeric_limits<float>::infinity();
	float max_drx = -std::numeric_limits<float>::infinity(), min_drx = std::numeric_limits<float>::infinity();
	float max_dry = -std::numeric_limits<float>::infinity(), min_dry = std::numeric_limits<float>::infinity();
	float max_x = -std::numeric_limits<float>::infinity(), min_x = std::numeric_limits<float>::infinity();
	float max_y = -std::numeric_limits<float>::infinity(), min_y = std::numeric_limits<float>::infinity();
	float max_depth = -std::numeric_limits<float>::infinity(), min_depth = std::numeric_limits<float>::infinity();
	float max_rx = -std::numeric_limits<float>::infinity(), min_rx = std::numeric_limits<float>::infinity();
	float max_ry = -std::numeric_limits<float>::infinity(), min_ry = std::numeric_limits<float>::infinity();
	float max_rz = -std::numeric_limits<float>::infinity(), min_rz = std::numeric_limits<float>::infinity();
	std::cout << "image,drx,dry,camera_angle,ddepth" << std::endl;
	for(int image_file_index = 2;image_file_index<argc;image_file_index++) {
		std::string image_file(argv[image_file_index]);
		objrec::scene_info scene;
		{
			std::string scene_file = objrec::corresponding_scene_file(image_file);
			std::ifstream in(scene_file.c_str());
			if(!in) throw std::runtime_error("Could not open scene file '"+scene_file+"'.");
			objrec::read_scene_info(in,scene);
		}
		objrec::table_constraint tc(scene,obj.object_center_height_above_table);
		objrec::point pose;
		{
			std::string pose_file = objrec::corresponding_pose_file(image_file);
			std::ifstream in(pose_file.c_str());
			if(!in) throw std::runtime_error("Could not open pose file '"+pose_file+"'.");
			in >> pose;
		}
		float ddepth = pose.depth - tc.depth(pose.x,pose.y);
		float drx = pose.rx - tc.rx(pose.x,pose.y);
		float dry = pose.ry - tc.ry(pose.x,pose.y);
		std::cout << argv[image_file_index] << "," << drx << "," << dry << "," << scene.camera_tilt_angle << "," << ddepth << std::endl;
		max_ddepth = std::max(ddepth,max_ddepth);
		max_drx = std::max(drx,max_drx);
		max_dry = std::max(dry,max_dry);
		max_x = std::max(pose.x,max_x);
		max_y = std::max(pose.y,max_y);
		max_depth = std::max(pose.depth,max_depth);
		max_rx = std::max(pose.rx,max_rx);
		max_ry = std::max(pose.ry,max_ry);
		max_rz = std::max(pose.rz,max_rz);
		min_ddepth = std::min(ddepth,min_ddepth);
		min_drx = std::min(drx,min_drx);
		min_dry = std::min(dry,min_dry);
		min_x = std::min(pose.x,min_x);
		min_y = std::min(pose.y,min_y);
		min_depth = std::min(pose.depth,min_depth);
		min_rx = std::min(pose.rx,min_rx);
		min_ry = std::min(pose.ry,min_ry);
		min_rz = std::min(pose.rz,min_rz);
	}
//	std::cout << "object depth goes from " << min_ddepth << "m to " << max_ddepth << "m from the labeled table surface." << std::endl;
//	std::cout << "rx goes from " << min_drx << " degrees to " << max_drx << " degrees from the labeled table surface." << std::endl;
//	std::cout << "ry goes from " << min_dry << " degrees to " << max_dry << " degrees from the labeled table surface." << std::endl;
//
//	std::cout << "min_ddepth,max_ddepth,min_drx,max_drx,min_dry,max_dry,min_x,max_x,min_y,max_y,min_depth,max_depth,min_rx,max_rx,min_ry,max_ry,min_rz,max_rz" << std::endl;
//	std::cout << min_ddepth << "," << max_ddepth << ",";
//	std::cout << min_drx << "," << max_drx << ",";
//	std::cout << min_dry << "," << max_dry << ",";
//	std::cout << min_x << "," << max_x << ",";
//	std::cout << min_y << "," << max_y << ",";
//	std::cout << min_depth << "," << max_depth << ",";
//	std::cout << min_rx << "," << max_rx << ",";
//	std::cout << min_ry << "," << max_ry << ",";
//	std::cout << min_rz << "," << max_rz << std::endl;
}
