/*
 * draw.h
 *
 *  Created on: Dec 16, 2014
 */

#ifndef DRAW_H_
#define DRAW_H_

#include "simulation.h"
#include "representation.h"

namespace objrec {
cv::Mat_<cv::Vec3b> draw_model(const cv::Mat_<cv::Vec3b>& detection_picture, const objrec::image* detection_image,
		const objrec::point& p, const objrec::view* v) {
	cv::Mat_<cv::Vec3b> im = detection_picture.clone();
	detection_image->draw(*v,p,im);
	return im;
}
cv::Mat_<cv::Vec3b> draw_combined_mesh(const cv::Mat_<cv::Vec3b>& detection_picture, const cv::Mat_<float>& detection_depth,
		const objrec::mesh& m, const objrec::point& p, objrec::OpenGL_X_window& w) {
	cv::Mat_<cv::Vec3b> im = detection_picture.clone();
	cv::Mat_<cv::Vec3b> rendered_image;
	cv::Mat_<float> rendered_depth;
	double xc, yc;
	if(!m.render(w,false,p.rx,p.ry,p.rz,p.depth,0,0,0.0,1.0,1.0,0.1,rendered_image,rendered_depth,xc,yc)) {
		std::cout << "object at (" << p.x << "," << p.y << ") is too close to the camera." << std::endl;
	}
	for(int y=0;y<rendered_depth.rows;y++) {
		for(int x=0;x<rendered_depth.cols;x++) {
			int imx = p.x+x-rendered_depth.cols/2;
			int imy = p.y+y-rendered_depth.rows/2;
			if(rendered_depth(y,x)==rendered_depth(y,x)) { //if the rendered depth value is not a NaN
				if(detection_depth(imy,imx)==detection_depth(imy,imx)) { //if the actual depth value is not a NaN
					if(rendered_depth(y,x)<detection_depth(imy,imx)) {
						im(imy,imx) = rendered_image(y,x);
					}
				} else {
					im(imy,imx) = cv::Vec3b(0,0,255); //red means the depth value in the actual image was missing
				}
			}
		}
	}
	return im;
}
cv::Mat_<cv::Vec3b> draw_superimposed_mesh(const cv::Mat_<cv::Vec3b>& detection_picture,
		const objrec::mesh& m, const objrec::point& p, objrec::OpenGL_X_window& w) {
	cv::Mat_<cv::Vec3b> im = detection_picture.clone();
	cv::Mat_<cv::Vec3b> rendered_image;
	cv::Mat_<float> rendered_depth;
	double xc, yc;
	m.render(w,false,p.rx,p.ry,p.rz,p.depth,0,0,0.0,1.0,1.0,0.1,rendered_image,rendered_depth,xc,yc);
	for(int y=0;y<rendered_depth.rows;y++) {
		for(int x=0;x<rendered_depth.cols;x++) {
			if(rendered_depth(y,x)==rendered_depth(y,x)) { //if the depth value is not a NaN
				im(p.y+y-rendered_depth.rows/2,p.x+x-rendered_depth.cols/2) = rendered_image(y,x);
			}
		}
	}
	return im;
}


} //namespace objrec

#endif /* DRAW_H_ */
