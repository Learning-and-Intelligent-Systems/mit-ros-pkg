/*
 * test_edge_detection.cpp
 *
 *  Created on: Jul 31, 2012
 */


//#include "image.h"
#include "edge_detect.h"
#include "gui.h"
#include <iostream>

void angles_bgr(const cv::Mat_<float>& edges, cv::Mat_<cv::Vec3b>& edgesbgr) {
	edgesbgr.create(edges.rows,edges.cols);
	for(int y=0;y<edges.rows;y++) {
		for(int x=0;x<edges.cols;x++) {
			float v = edges(y,x);
			if(v==v) { //if v is not a NaN
				edgesbgr(y,x) = cv::Vec3b(0,0,0);
			}
			else {
				edgesbgr(y,x) = cv::Vec3b(255,255,255);
			}
		}
	}
}


int main(int argc, char** argv) {
	if(argc<2) {
		std::cout << "usage: " << argv[0] << " image_file" << std::endl;
		exit(0);
	}
	std::string image_file(argv[1]);
	std::string depth_file = objrec::corresponding_depth_file(image_file);

	cv::Mat_<cv::Vec3b> image = cv::imread(image_file);

	cv::imshow("image",image);

	cv::Mat_<float> depth = objrec::read_depth(depth_file);

	cv::imshow("depth",objrec::mat_bgr(depth));

	cv::Mat_<unsigned char> gray;
	cv::cvtColor(image,gray,CV_BGR2GRAY);

	int edge_aperture_size = 5;
	bool L2gradient = true;
	double thresh1 = 500;
	double thresh2 = 250;

	cv::Mat_<unsigned char> canny_edges;
	cv::Canny(gray,canny_edges,thresh1,thresh2,edge_aperture_size,L2gradient);
	cv::imshow("Canny",canny_edges);

	objrec::edge_detection_parameters p;


	bool quit = false;
	while(!quit) {
		cv::Mat_<float> Uzun_edges;
		objrec::detect_edges(gray,depth,p,Uzun_edges);
		cv::Mat_<cv::Vec3b> Uzun_edgesbgr;
		angles_bgr(Uzun_edges,Uzun_edgesbgr);
		cv::imshow("Uzun",Uzun_edgesbgr);

		int key = cv::waitKey(0);
		switch(key) {
		case 1048673: //a
			p.TL_missingDepthEdge_SobelMag *= 1.1;
			break;
		case 1048698: //z
			p.TL_missingDepthEdge_SobelMag /= 1.1;
			break;
		case 1048691: //s
			p.TL_discontDepthEdge_SobelMag *= 1.1;
			break;
		case 1048696: //x
			p.TL_discontDepthEdge_SobelMag /= 1.1;
			break;
		case 1048676: //d
			p.TL_discontDepthEdge_dMag *= 1.1;
			break;
		case 1048675: //c
			p.TL_discontDepthEdge_dMag /= 1.1;
			break;
		case 1048678: //f
			p.TL_DepthEdge_SobelMag *= 1.1;
			break;
		case 1048694: //v
			p.TL_DepthEdge_SobelMag /= 1.1;
			break;
		case 1048679: //g
			p.TL_directDerivDepthEdge_Average *= 1.1;
			break;
		case 1048674: //b
			p.TL_directDerivDepthEdge_Average /= 1.1;
			break;
		case 1048680: //h
			p.TH_missingDepthEdge_SobelMag *= 1.1;
			break;
		case 1048686: //n
			p.TH_missingDepthEdge_SobelMag /= 1.1;
			break;
		case 1048682: //j
			p.TH_discontDepthEdge_SobelMag *= 1.1;
			break;
		case 1048685: //m
			p.TH_discontDepthEdge_SobelMag /= 1.1;
			break;
		case 1048683: //k
			p.TH_discontDepthEdge_dMag *= 1.1;
			break;
		case 1048620: //,
			p.TH_discontDepthEdge_dMag /= 1.1;
			break;
		case 1048684: //l
			p.TH_DepthEdge_SobelMag *= 1.1;
			break;
		case 1048622: //.
			p.TH_DepthEdge_SobelMag /= 1.1;
			break;
		case 1048635: //;
			p.TH_directDerivDepthEdge_Average *= 1.1;
			break;
		case 1048623: ///
			p.TH_directDerivDepthEdge_Average /= 1.1;
			break;
		case 1048689: //q
			quit = true;
			break;
		default:
			std::cout << "unrecognized key: " << key << std::endl;
		}
		std::cout << "TL_missingDepthEdge_SobelMag " << p.TL_missingDepthEdge_SobelMag << std::endl;
		std::cout << "TL_discontDepthEdge_SobelMag " << p.TL_discontDepthEdge_SobelMag << std::endl;
		std::cout << "TL_discontDepthEdge_dMag " << p.TL_discontDepthEdge_dMag << std::endl;
		std::cout << "TL_DepthEdge_SobelMag " << p.TL_DepthEdge_SobelMag << std::endl;
		std::cout << "TL_directDerivDepthEdge_Average " << p.TL_directDerivDepthEdge_Average << std::endl;
		std::cout << "TH_missingDepthEdge_SobelMag " << p.TH_missingDepthEdge_SobelMag << std::endl;
		std::cout << "TH_discontDepthEdge_SobelMag " << p.TH_discontDepthEdge_SobelMag << std::endl;
		std::cout << "TH_discontDepthEdge_dMag " << p.TH_discontDepthEdge_dMag << std::endl;
		std::cout << "TH_DepthEdge_SobelMag " << p.TH_DepthEdge_SobelMag << std::endl;
		std::cout << "TH_directDerivDepthEdge_Average " << p.TH_directDerivDepthEdge_Average << std::endl;
		std::cout << std::endl;
	}
////	exit(0);
//	int aperture_size = 5;
//	cv::Mat_<short> dx(image.rows,image.cols);
//	cv::Sobel(gray,dx,CV_16S,1,0,aperture_size);
//	cv::Mat_<short> dy(image.rows,image.cols);
//	cv::Sobel(gray,dy,CV_16S,0,1,aperture_size);
//
////	int depth_aperture_size = 7;
////	cv::Mat_<float> ddx(image.rows,image.cols);
////	cv::Sobel(depth,ddx,CV_32F,1,0,depth_aperture_size);
////	cv::Mat_<float> ddy(image.rows,image.cols);
////	cv::Sobel(depth,ddy,CV_32F,0,1,depth_aperture_size);
//
//	cv::Mat_<float> ddx = sobel_x(depth);
//	cv::Mat_<float> ddy = sobel_y(depth);
//
//	cv::Mat_<unsigned int> mag(image.rows,image.cols);
//	cv::Mat_<float> angle(image.rows,image.cols);
//	for(int y=0;y<image.rows;y++) {
//		for(int x=0;x<image.cols;x++) {
//			int dx_ = dx(y,x);
//			int dy_ = dy(y,x);
//			mag(y,x) = dx_*dx_+dy_*dy_;
//			angle(y,x) = atan((float)dy_/(float)dx_);
//		}
//	}
//
//	cv::Mat_<unsigned int> nms(image.rows,image.cols);
//	for(int y=0;y<image.rows;y++) {
//		for(int x=0;x<image.cols;x++) {
//
//		}
//	}
//
//	cv::Mat_<float> depth_mag(image.rows,image.cols);
//	cv::Mat_<float> depth_angle(image.rows,image.cols);
//	for(int y=0;y<image.rows;y++) {
//		for(int x=0;x<image.cols;x++) {
//			float dx_ = ddx(y,x);
//			float dy_ = ddy(y,x);
//			depth_mag(y,x) = dx_*dx_+dy_*dy_;
//			depth_angle(y,x) = atan((float)dy_/(float)dx_);
//		}
//	}
//
//	float min, max;
//	objrec::min_max_mat(depth_mag, min, max);
//
//	cv::Mat_<float> tdmag(image.rows,image.cols);
//	int total = 0;
//	for(int y=0;y<image.rows;y++) {
//		for(int x=0;x<image.cols;x++) {
//			if(depth_mag(y,x)>70 && depth(y,x)<1.6) {
//				tdmag(y,x) = 255;
//				total++;
//			}
//			else {
//				tdmag(y,x) = 0;
//			}
//		}
//	}
//	std::cout << "total: " << total << std::endl;
//
//
//	std::cout << "min: " << min << " max: " << max << std::endl;
//	cv::imshow("dx",objrec::mat_bgr(dx));
//	cv::imshow("dy",objrec::mat_bgr(dy));
//	cv::imshow("mag",objrec::mat_bgr(mag));
//	cv::imshow("ddx",objrec::mat_bgr(ddx));
//	cv::imshow("ddy",objrec::mat_bgr(ddy));
//	cv::imshow("dmag",objrec::mat_bgr(depth_mag));
//	cv::imshow("tdmag",objrec::mat_bgr(tdmag));
//	cv::waitKey(0);
}
