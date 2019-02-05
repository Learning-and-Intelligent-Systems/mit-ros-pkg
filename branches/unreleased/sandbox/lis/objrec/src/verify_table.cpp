/*
 * verify_table.cpp
 *
 *  Created on: Apr 22, 2014
 */

#include "constraints.h"

int main(int argc, char** argv) {
	if(argc<2) {
		std::cout << "usage:" << std::endl;
		std::cout << argv[0] << " 0001.png 0002.png ..." << std::endl;
		return -1;
	}
	long long start, end;

	for(int j=1;j<argc;j++) {
		std::cerr << "Loading picture file '" << argv[j] << "'... " << std::flush;
		start = objrec::current_time();
		cv::Mat_<cv::Vec3b> picture;
		picture = cv::imread(argv[j]);
		end = objrec::current_time();
		std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;

		std::string depth_file = objrec::corresponding_depth_file(argv[j]);
		std::cerr << "Loading depth file '" << depth_file << "'... " << std::flush;
		start = objrec::current_time();
		cv::Mat_<float> depth;
		depth = objrec::read_depth(depth_file);
		end = objrec::current_time();
		std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;

		std::string scene_file = objrec::corresponding_scene_file(argv[j]);
		objrec::scene_info info;
		std::cerr << "Loading scene file '" << scene_file << "'... " << std::flush;
		start = objrec::current_time();
		std::ifstream in(scene_file.c_str());
		objrec::read_scene_info(in,info);
		end = objrec::current_time();
		std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;

		cv::imshow("image",picture);
		cv::imshow("depth",objrec::mat_bgr(depth));

		objrec::table_constraint tc(info, 0);
		cv::Mat_<cv::Vec3b> depth_bgr = objrec::mat_bgr(depth);
		for(int y=0;y<depth_bgr.rows;y++) {
			for(int x=0;x<depth_bgr.cols;x++) {
				if(depth(y,x)==depth(y,x)) {
					if(depth(y,x)>tc.depth(x,y)) {
						depth_bgr(y,x) = cv::Vec3b(255,0,0);
					} else {
						depth_bgr(y,x) = cv::Vec3b(0,255,0);
					}
				}
			}
		}
		cv::imshow("blue means farther than expected, green means nearer",depth_bgr);
		cv::waitKey(0);
	}
}
