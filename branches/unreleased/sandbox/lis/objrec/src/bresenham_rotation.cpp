/*
 * bresenham_rotation.cpp
 *
 *  Created on: May 15, 2014
 */
#include "image.h"
#include <opencv2/imgproc/imgproc.hpp>

template <class T>
void bresenham_copy_row(const cv::Mat_<T>& in, int row, int x0, int y0, int x1, int y1,
						//this parameter is modified by this function:
						cv::Mat_<T>& out) {
	int dx = abs(x1-x0);
	int dy = abs(y1-y0);
	int sx  = (x0<x1)?1:-1; //sign of x step
	int sy  = (y0<y1)?1:-1; //sign of y step
	int err = dx-dy;
	int col = 0;
	while(true) {
		if(
				0<=y0 && y0<out.rows &&
				0<=x0 && x0<out.cols &&
				0<=row && row<in.rows &&
				0<=col && col<in.cols) {
			out(y0,x0) = in(row,col);
		} else {
//			std::cout << "copying from in(row=" << row << ",col=" << col << "), size(" << in.rows << "," << in.cols << ") " <<
//					"to out(y0=" << y0 << ",x0=" << x0 << "), size(" << out.rows << ","  << out.cols << ")" << std::endl;
		}
		if(x0==x1 && y0==y1) {
			break;
		}
		int e2 = 2*err;
		if(e2>-dy) {
			err -= dy;
			x0 += sx;
		}
		if(e2<dx) {
			err += dx;
			y0 += sy;
		}
		col++;
	}
}

template <class T>
cv::Mat_<T> bresenham_rotate(const cv::Mat_<T>& in, float angle, const T& zero = T()) {
	//
	// (x0,y0) +----+ (x2,y2)
	//         |    |
	//         |    |
	// (x1,y1) +----+
	//
	int x0, x1, x2;
	int y0, y1, y2;

	cv::Mat_<T> out;
	if(0<=angle && angle<=1.0*M_PI/4.0) {
		float t = tan(angle);
		x0 = in.rows*t;
		y0 = 0;
		x1 = 0;
		y1 = in.rows;
		x2 = in.cols+in.rows*t;
		y2 = in.cols*t;
		out.create(in.rows+in.cols*t,x2);
	} else {
		std::cout << "bad angle" << std::endl;
		return out;
	}
	objrec::set_to(out,zero);
	int lx = x2-x0;
	int ly = y2-y0;
	int dx = -(x1-x0);//abs(x1-x0);
	int dy = y1-y0;//abs(y1-y0);
	int sx  = -1;//(x0<x1)?1:-1; //sign of x step
	int sy  = 1;//(y0<y1)?1:-1; //sign of y step
	int err = dx-dy;
	int row = 0;
	while(true) {
		bresenham_copy_row(in,row,x0,y0,x0+lx,y0+ly,
				//output parameter:
				out);
		if(x0==x1 && y0==y1) {
			break;
		}
		int e2 = 2*err;
		if(e2>-dy) {
			err -= dy;
			x0 += sx;
		}
		if(e2<dx) {
			err += dx;
			y0 += sy;
		}
		row++;
	}
	return out;
}

template <class T>
void half_square(const cv::Mat_<T>& in, float angle, unsigned short x, unsigned short y, unsigned short radius,
		//output parameters:
		cv::Mat_<T>& rotated,
		unsigned short x0, unsigned short x1,
		unsigned short y0, unsigned short y1) {
	rotated = bresenham_rotate(in,angle);

}

int main(int argc, char** argv) {
//	cv::Mat_<cv::Vec3b> in(30,30);
//	objrec::set_to(in,cv::Vec3b(0,0,255));
	cv::Mat_<cv::Vec3b> in;
	in = cv::imread("/home/sdavies/data/messy/tiki/0001.png");
	cv::resize(in,in,cv::Size(),.5,.5,CV_INTER_LANCZOS4);
	cv::Mat_<cv::Vec2s> inxy(in.rows,in.cols);
	for(int y=0;y<in.rows;y++) {
		for(int x=0;x<in.cols;x++) {
			inxy(y,x) = cv::Vec2s(x,y);
		}
	}
	for(float j=M_PI/2.0/30.0;j<=M_PI/4.0;j+=M_PI/2.0/30.0) {
		cv::Mat_<cv::Vec3b> out = bresenham_rotate(in,j,cv::Vec3b(0,0,0));
//		cv::Mat_<cv::Vec2s> outxy = bresenham_rotate(inxy,j,cv::Vec2s(-1,-1));
//		for(int y=0;y<outxy.rows;y++) {
//			for(int x=0;x<outxy.cols;x++) {
//				if(outxy(y,x)!=cv::Vec2s(-1,-1)) {
//					std::cout << j << " " << in.cols << " " << in.rows << " " << outxy(y,x)[0] << " " << outxy(y,x)[1] << " " << x << " " << y << std::endl;
//				}
//			}
//		}
		cv::imshow("result",out);
		cv::waitKey(0);
	}
}
