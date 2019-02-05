/*
 * edge_detect.cpp
 *
 *  Created on: Sep 11, 2012
 */

#include "edge_detect.h"
#include <limits>
namespace objrec {

void sobel(const cv::Mat_<unsigned char>& im, int edge_aperture_size,
		//output parameters:
		cv::Mat_<float>& magnitude, cv::Mat_<float>& angle) {
	cv::Mat_<short> sobelx(cv::Size(im.cols,im.rows));
	cv::Mat_<short> sobely(cv::Size(im.cols,im.rows));
	cv::Sobel(im,sobelx,sobelx.depth(),1,0,edge_aperture_size);
	cv::Sobel(im,sobely,sobely.depth(),0,1,edge_aperture_size);
	magnitude.create(im.rows,im.cols);
	angle.create(im.rows,im.cols);
	for(int y=0;y<im.rows;y++) {
		for(int x=0;x<im.cols;x++) {
			short sx = sobelx(y,x);
			short sy = sobely(y,x);
			magnitude(y,x) = sqrt(sx*sx+sy*sy);
			if(sx==0) {
				angle(y,x) = M_PI/2.0;
			}
			else {
				angle(y,x) = atan((float)sy/(float)sx);
			}
		}
	}
}

void edges_without_hysteresis_thresholding(const cv::Mat_<unsigned char>& im, float low_threshold, float high_threshold,
		//output parameters:
		//in the 'thresholded' matrix, 0 means no edge,
		//                1 means it is between the high and low threshold, and
		//                2 means it is above the high threshold
		cv::Mat_<unsigned char>& thresholded,
		cv::Mat_<float>& angle
		) {
	thresholded.create(im.rows,im.cols);
	cv::Mat_<float> magnitude;
	int edge_aperture_size = 5;
	sobel(im,edge_aperture_size,
			//output parameters:
			magnitude,angle);

	const float rad = M_PI/180.0;
	for(int v=0;v<im.rows;v++) {
		for(int u=0;u<im.cols;u++) {
			float iMag0, iMag1;
			float iAng = angle(v,u);
			if(-67.5*rad <= iAng && iAng < -22.5*rad) {
				iMag0 = magnitude(v-1,u+1);
				iMag1 = magnitude(v+1,u-1);
			}
			else if(-22.5*rad <= iAng && iAng < 22.5*rad) {
				iMag0 = magnitude(v,u+1);
				iMag1 = magnitude(v,u-1);
			}
			else if(22.5*rad <= iAng && iAng < 67.5*rad) {
				iMag0 = magnitude(v-1,u-1);
				iMag1 = magnitude(v+1,u+1);
			}
			else { //if(iAng<-67.5*rad || iAng>=67.5*rad) {
				iMag0 = magnitude(v-1,u);
				iMag1 = magnitude(v+1,u);
			}


			bool high_edge = magnitude(v,u) >= high_threshold;
			bool low_edge = magnitude(v,u) >= low_threshold;
            bool NMS = magnitude(v,u)>=iMag0 && magnitude(v,u)>iMag1;

            if (high_edge && NMS) {
            	thresholded(v,u) = 2;
            } else if (low_edge && NMS) {
            	thresholded(v,u) = 1;
            } else {
            	thresholded(v,u) = 0;
            }
		}
	}
}

void traverse(int u, int v, cv::Mat_<unsigned char>& thresholded);

void traverse_test(int x, int y, cv::Mat_<unsigned char>& thresholded) {
	if(0<y && y<thresholded.cols && 0<x && x<thresholded.rows) {
		if(thresholded(y,x)==1) {
			thresholded(y,x) = 2;
			traverse(x,y,thresholded);
		}
	}
}

void traverse(int u, int v, cv::Mat_<unsigned char>& thresholded) {
	traverse_test(u-1,v-1,thresholded);
	traverse_test(u+0,v-1,thresholded);
	traverse_test(u+1,v-1,thresholded);
	traverse_test(u-1,v+0,thresholded);
	traverse_test(u+1,v+0,thresholded);
	traverse_test(u-1,v+1,thresholded);
	traverse_test(u+0,v+1,thresholded);
	traverse_test(u+1,v+1,thresholded);
}

void hysteresis_threshold(cv::Mat_<unsigned char>& thresholded) {
	for(int v=1;v<thresholded.rows-1;v++) {
		for(int u=1;u<thresholded.cols-1;u++) {
			if(thresholded(v,u)==2) {
				traverse(u,v,thresholded);
			}
		}
	}
}

#ifdef TWO_THRESHOLD_FEATURES
void detect_edges( const cv::Mat_<unsigned char>& im, float low_threshold, float high_threshold,
		//output parameters:
		cv::Mat_<float>& low_threshold_edges, cv::Mat_<float>& high_threshold_edges) {
	cv::Mat_<float> angle;
	cv::Mat_<unsigned char> thresholded;
	edges_without_hysteresis_thresholding(im,low_threshold,high_threshold,
			//output parameters:
			thresholded,angle);
	low_threshold_edges.create(im.rows,im.cols);
	for(int v=0;v<angle.rows;v++) {
		for(int u=0;u<angle.cols;u++) {
			if(thresholded(v,u)==1) {
				low_threshold_edges(v,u) = angle(v,u);
			} else {
				low_threshold_edges(v,u) = std::numeric_limits<float>::quiet_NaN();
			}
		}
	}
	high_threshold_edges.create(im.rows,im.cols);
	for(int v=0;v<angle.rows;v++) {
		for(int u=0;u<angle.cols;u++) {
			if(thresholded(v,u)==2) {
				high_threshold_edges(v,u) = angle(v,u);
			} else {
				high_threshold_edges(v,u) = std::numeric_limits<float>::quiet_NaN();
			}
		}
	}
}
#else
void detect_edges( const cv::Mat_<unsigned char>& im, float low_threshold, float high_threshold,
		//output parameter:
		cv::Mat_<float>& edges) {
	cv::Mat_<float> angle;
	cv::Mat_<unsigned char> thresholded;
	edges_without_hysteresis_thresholding(im,low_threshold,high_threshold,
			//output parameters:
			thresholded,angle);
	hysteresis_threshold(thresholded);
	edges.create(im.rows,im.cols);
	for(int v=0;v<angle.rows;v++) {
		for(int u=0;u<angle.cols;u++) {
			if(thresholded(v,u)!=2) {
				edges(v,u) = std::numeric_limits<float>::quiet_NaN();
			} else {
				edges(v,u) = angle(v,u);
			}
		}
	}
}
#endif


cv::Mat_<cv::Vec3b> edges_bgr(const cv::Mat_<cv::Vec3b>& picture, const cv::Mat_<float>& depth, const edge_detection_parameters& p) {
	cv::Mat_<cv::Vec3b> edgesbgr(picture.rows,picture.cols);
	edgesbgr.create(picture.size());
#ifdef TWO_THRESHOLD_FEATURES
	cv::Mat_<float> low_threshold_edges, high_threshold_edges;
	p.edge_detect(picture, depth,
			//output parameters:
			low_threshold_edges, high_threshold_edges);
	for(int y=0;y<picture.rows;y++) {
		for(int x=0;x<picture.cols;x++) {
			float low = low_threshold_edges(y,x);
			float high = high_threshold_edges(y,x);
			if(high==high) { //if high is not a NaN
				edgesbgr(y,x) = cv::Vec3b(0,0,0);
			} else if(low==low) { //if low is not a NaN
				edgesbgr(y,x) = cv::Vec3b(128,128,128);
			} else {
				edgesbgr(y,x) = cv::Vec3b(255,255,255);
			}
		}
	}
#else
	cv::Mat_<float> edges = p.edge_detect(picture, depth);
	for(int y=0;y<picture.rows;y++) {
		for(int x=0;x<picture.cols;x++) {
			float v = edges(y,x);
			if(v==v) { //if v is not a NaN
				edgesbgr(y,x) = cv::Vec3b(0,0,0);
			}
			else {
				edgesbgr(y,x) = cv::Vec3b(255,255,255);
			}
		}
	}
#endif
	return edgesbgr;
}
cv::Mat_<cv::Vec3b> edges_bgr(const std::vector<cv::Mat_<float> >& precomputed_edges, const edge_detection_parameters& p) {
	float low_threshold = p.get_low_threshold();
	float high_threshold = p.get_high_threshold();
	cv::Mat_<cv::Vec3b> edgesbgr(precomputed_edges[0].rows,precomputed_edges[0].cols);
	set_to(edgesbgr,cv::Vec3b(255,255,255));
	for(int j=0;j<precomputed_edges.size();j++) {
		for(int y=0;y<precomputed_edges[j].rows;y++) {
			for(int x=0;x<precomputed_edges[j].cols;x++) {
				if(precomputed_edges[j](y,x)>=low_threshold && precomputed_edges[j](y,x)<high_threshold) {
					edgesbgr(y,x) = cv::Vec3b(128,128,128);
				} else if(precomputed_edges[j](y,x)>=high_threshold) {
					edgesbgr(y,x) = cv::Vec3b(0,0,0);
				}
			}
		}
	}
	return edgesbgr;
}

} //namespace objrec
