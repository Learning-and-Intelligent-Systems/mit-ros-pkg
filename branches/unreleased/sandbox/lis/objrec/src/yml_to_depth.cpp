/*
 * yml_to_depth.cpp
 *
 *  Created on: Jul 7, 2014
 */

#include "io.h"

int main(int argc, char** argv) {
	if(argc<2) {
		std::cout << "usage:" << std::endl;
		std::cout << argv[0] << " 0001.yml 0002.yml ..." << std::endl;
		return -1;
	}

	for(int j=1;j<argc;j++) {
		std::string yml_file = argv[j];
		cv::Mat_<float> depth = objrec::read_depth(yml_file);
		size_t dot = yml_file.find_last_of('.');
		std::string depth_file = yml_file.substr(0,dot)+".depth";

		std::ofstream out(depth_file.c_str());
		for(int y=0;y<depth.rows;y++) {
			for(int x=0;x<depth.cols;x++) {
				out << depth(y,x) << " ";
			}
			out << std::endl;
		}
	}
}
