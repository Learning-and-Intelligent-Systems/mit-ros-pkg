/*
 * pose_to_bbox.cpp
 *
 *  Created on: May 29, 2014
 */

#include "constraints.h"

int main(int argc, char** argv) {
	if(argc<2) {
		std::cerr << "usage:" << std::endl;
		std::cerr << argv[0] << " object_file 0001.pose 0002.pose ..." << std::endl;
		return -1;
	}
	objrec::object obj;
	{
		std::string object_file = argv[1];
		std::ifstream in(object_file.c_str());
		if(!in) {
			std::cerr << "Could not open object file " << object_file << std::endl;
			return -1;
		}
		in >> obj;
	}
	for(int j=2;j<argc;j++) {
		std::string pose_file = argv[j];
		std::ifstream in(pose_file.c_str());
		if(!in) {
			std::cerr << "File " << pose_file << " not found." << std::endl;
			return -1;
		}
		objrec::point pose;
		in >> pose;

		const objrec::view* v = objrec::find_view(obj,pose.rx,pose.ry,pose.rz);
		float x0, x1, y0, y1;
		objrec::get_bounding_box(*v,pose,x0,x1,y0,y1);
//		std::cout << "pose file " << pose_file << ": " << pose << std::endl;
//		std::cout << x0 << " " << x1 << " " << y0 << " " << y1 << std::endl;
		size_t dot = pose_file.find_last_of('.');
		std::string bbox = pose_file.substr(0,dot)+".bbox";
		{
			std::ifstream in(bbox.c_str());
			if(in) {
				objrec::bounding_box bounding_box;
				objrec::read_bounding_box(in,bounding_box);
				std::cerr << "Overwriting existing bounding box " << bbox << ": " << bounding_box << std::endl;
			}
		}
		std::ofstream out(bbox.c_str());
		out << "#<min x> <max x> <min y> <max y>" << std::endl;
		out << floor(x0) << " " << ceil(x1) << " " << floor(y0) << " " << ceil(y1) << std::endl;
	}
	std::cerr << "Wrote " << argc-2 << " bounding box files." << std::endl;
}
