/*
 * test_edge_directions.cpp
 *
 *  Created on: Jul 8, 2014
 */

#include "edge_detect.h"

int main(int argc, char** argv) {
	if(argc<2) {
		std::cout << "usage:" << std::endl;
		std::cout << argv[0] << " image.png" << std::endl;
		return -1;
	}
	cv::Mat_<cv::Vec3b> im = cv::imread(argv[1]);
	cv::Mat_<float> depth(im.size());
	objrec::edge_detection_parameters edp_canny(8,1,200,1000);
	cv::Mat_<float> low_threshold_angles, high_threshold_angles;
	edp_canny.edge_detect(im,depth,
			//output parameters:
			low_threshold_angles,high_threshold_angles);
	std::string edges_file = objrec::corresponding_edges_file(argv[1]);
	std::vector<cv::Mat_<float> > precomputed_edges;
	objrec::edge_detection_parameters edp_ren(8,1,.1,.5);
	if(edges_file!="") {
		objrec::read_edges(edges_file,
				//output parameter:
				precomputed_edges);
		cv::imshow("all precomputed edges",objrec::edges_bgr(precomputed_edges,edp_ren));
	} else {
		std::cout << "edges file not found." << std::endl;
	}


	cv::imshow("all edges",objrec::edges_bgr(im,depth,edp_canny));
	cv::waitKey(0);
//	for(int j=0;j<edp_canny.get_edge_directions().size();j++) {
//		cv::Mat_<cv::Vec3b> edges(im.size());
//		cv::Mat_<cv::Vec3b> precomputed_edges_rgb(im.size());
//		const objrec::edge_direction& ed = edp_canny.get_edge_directions()[j];
//		for(int y=0;y<im.rows;y++) {
//			for(int x=0;x<im.cols;x++) {
//				if(ed.in_range(high_threshold_angles(y,x))) {
//					edges(y,x) = cv::Vec3b(0,0,0);
//				} else if(ed.in_range(low_threshold_angles(y,x))) {
//					edges(y,x) = cv::Vec3b(127,127,127);
//				} else {
//					edges(y,x) = cv::Vec3b(255,255,255);
//				}
//
//				if(precomputed_edges.size()>0 && precomputed_edges[j](y,x)>threshold) {
//					precomputed_edges_rgb(y,x) = cv::Vec3b(0,0,0);
//				} else {
//					precomputed_edges_rgb(y,x) = cv::Vec3b(255,255,255);
//				}
//			}
//		}
//		std::cout << "edge direction: " << ed << std::endl;
//		cv::imshow("edges",edges);
//		cv::imshow("precomputed edges",precomputed_edges_rgb);
//		cv::waitKey(0);
//	}
}
