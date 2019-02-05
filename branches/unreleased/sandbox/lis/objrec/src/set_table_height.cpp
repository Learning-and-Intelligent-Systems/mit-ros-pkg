/*
 * set_table_height.cpp
 *
 *  Created on: Jun 19, 2014
 */

#include "io.h"

int main(int argc, char** argv) {
	if(argc<2) {
		std::cout << "usage:" << std::endl;
		std::cout << argv[0] << " table_height 0001.scene 0002.scene ..." << std::endl;
		return -1;
	}
	std::istringstream table_height_stream(argv[1]);
	float table_height;
	table_height_stream >> table_height;

	for(int j=2;j<argc;j++) {
		objrec::scene_info scene;
		{
			std::ifstream in(argv[j]);
			if(!in) {
				std::cout << "File " << argv[j] << " not found." << std::endl;
				return -1;
			}
			objrec::read_scene_info(in,scene);
		}

		scene.table_height = table_height;

		{
			std::ofstream out(argv[j]);
			out << "#" << objrec::scene_info_description << std::endl;
			out << scene << std::endl;
		}
	}
}
