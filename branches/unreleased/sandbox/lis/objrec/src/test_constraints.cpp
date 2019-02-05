/*
 * test_constraints.cpp
 *
 *  Created on: Mar 27, 2014
 */

#include "constraints.h"
#include <fstream>

float range_max(const cv::Mat_<float>& m, int min_x, int max_x, int min_y, int max_y) {
	float max = -std::numeric_limits<float>::infinity();
	for(int y=min_y;y<=max_y;y++) {
		for(int x=min_x;x<=max_x;x++) {
			max = std::max(max,m(y,x));
		}
	}
	return max;
}

float range_min(const cv::Mat_<float>& m, int min_x, int max_x, int min_y, int max_y) {
	float min = std::numeric_limits<float>::infinity();
	for(int y=min_y;y<=max_y;y++) {
		for(int x=min_x;x<=max_x;x++) {
			min = std::min(min,m(y,x));
		}
	}
	return min;
}

void object_angles(int x, int y, double ty, float& rx, float& ry) {
	double projx = ((double)x-objrec::cx-objrec::Tx)/objrec::fx;
	double projy = ((double)y-objrec::cy-objrec::Ty)/objrec::fy;
	ry = asin(-(sin(ty)+cos(ty)*projy)/sqrt(1+projy*projy+projx*projx));
}

int main(int argc, char** argv) {
	double ty = 0.747589;
	std::ifstream in("/home/sdavies/data/downy-untextured.off/0001.cam");
	objrec::camera_info info;
	objrec::read_camera_info(in,info);
	ty = atan2(-info.rot.m31,sqrt(info.rot.m32*info.rot.m32+info.rot.m33*info.rot.m33));
	objrec::table_constraint c(ty,.3);

//	objrec::region r(objrec::point(0,0,0.648269,0,-70,-10),objrec::point(320,240,1.14827,10,-60,0));
//	objrec::region intersected = c.intersect(r);
//	std::cout << "before intersection:" << std::endl << r.min << std::endl << r.max << std::endl;
//	std::cout << "after  intersection:" << std::endl << intersected.min << std::endl << intersected.max << std::endl;

	objrec::object obj;
	{
		std::ifstream obj_in("/var/tmp/downy-untextured.off.txt");
		obj_in >> obj;
	}
	for(int j=0;j<obj.views.size();j++) {
//		std::cout << "view " << j << std::endl;
		const objrec::viewpoint& vp = obj.views[j].get_viewpoint();
		objrec::region r(
				objrec::point(0,0,vp.min_depth,vp.min_rx,vp.min_ry,vp.min_rz),
				objrec::point(640,480,vp.max_depth,vp.max_rx,vp.max_ry,vp.max_rz));
		objrec::region intersected = c.intersect(r);
//		std::cout << " before intersection:" << std::endl << r.min << std::endl << r.max << std::endl;
//		std::cout << " after  intersection:" << std::endl << intersected.min << std::endl << intersected.max << std::endl;
//		std::cout << intersected.is_empty() << " = is_emtpy()" << std::endl;
	}

	//	std::cout << "ty: " << ty << std::endl;
//	{
//		cv::Mat_<float> rx(480,640);
//		cv::Mat_<float> ry(480,640);
//		std::ofstream rx_out("/tmp/rx.txt");
//		std::ofstream ry_out("/tmp/ry.txt");
//		for(int y=0;y<480;y++) {
//			for(int x=0;x<640;x++) {
//				objrec::vector2 px(x,y);
//				objrec::upright_angles(px,info,ry(y,x),rx(y,x));
//				object_angles(x,y,ty,rx(y,x),ry(y,x));
//				rx_out << rx(y,x) << " ";
//				ry_out << ry(y,x) << " ";
//			}
//			rx_out << std::endl;
//			ry_out << std::endl;
//		}
//	}
	for(int j=1;j<100;j++) {
		ty = ((rand()%180)-90)*M_PI/180.0;
		cv::Mat_<float> rx(480,640);
		cv::Mat_<float> ry(480,640);
		cv::Mat_<float> depth(480,640);
		for(int y=0;y<480;y++) {
			for(int x=0;x<640;x++) {
				objrec::vector2 px(x,y);
				objrec::upright_angles(px,info,ry(y,x),rx(y,x));
				rx(y,x) = c.rx(x,y);
				ry(y,x) = c.ry(x,y);
				depth(y,x) = c.depth(x,y);
//				object_angles(x,y,ty,rx(y,x),ry(y,x));
			}
		}
		//	std::cout << info << std::endl;
		for(int min_y=0;min_y<480;min_y+=50) {
			for(int max_y=min_y;max_y<480;max_y+=50) {
				for(int min_x=0;min_x<640;min_x+=50) {
					for(int max_x=min_x;max_x<640;max_x+=50) {
						objrec::region r(objrec::point(min_x,min_y,0,0,0,0),objrec::point(max_x,max_y,5,360,360,360));
						float cmin_y, cmax_y;
						c.ry_range(r,cmin_y,cmax_y);
						float amin_y = range_min(ry,r.min.x,r.max.x,r.min.y,r.max.y);
						float amax_y = range_max(ry,r.min.x,r.max.x,r.min.y,r.max.y);
						if(fabs(cmin_y-amin_y)>.174 || fabs(cmax_y-amax_y)>.174) {
							std::cout << "ry camera angle: " << ty*180.0/M_PI << std::endl;
							std::cout << "region: " << r.min.x << " " << r.max.x << " " << r.min.y << " " << r.max.y << std::endl;
							std::cout << "actual range:   " << amin_y << " " << amax_y << std::endl;
							std::cout << "computed range: " << cmin_y << " " << cmax_y << std::endl;

							cv::Mat_<cv::Vec3b> ry_bgr = objrec::mat_bgr(ry);

							float camera_tilt_angle = ty;
							float sin_cta = sin(camera_tilt_angle);
							float cos_cta = cos(camera_tilt_angle);
							float cot_cta = 1.0/tan(camera_tilt_angle);
							const float fx = 525, fy = 525;
							const float cx = 319.5, cy = 239.5;
							const float Tx = 0, Ty = 0;
							const int im_height = 480, im_width = 640;
							float arg_max_y;
							float dx;
							dx = Tx-r.min.x;
							arg_max_y = (fx*fx*(cy+Ty)+fy*(cx*cx+fx*fx+2*cx*dx+dx*dx)*cot_cta)/(fx*fx); //todo: factor out this division
							if(r.min.y<arg_max_y && arg_max_y<r.max.y) {
								std::cout << "trying x = min_x line" << std::endl;
							}
							dx = Tx-r.max.x;
							arg_max_y = (fx*fx*(cy+Ty)+fy*(cx*cx+fx*fx+2*cx*dx+dx*dx)*cot_cta)/(fx*fx); //todo: factor out this division
							if(r.min.y<arg_max_y && arg_max_y<r.max.y) {
								std::cout << "trying x = max_x line" << std::endl;
							}
							float arg_max_x = cx+Tx;
							if(r.min.x<arg_max_x && arg_max_x<r.max.x) {
								std::cout << "trying y = min_y line" << std::endl;
								std::cout << "trying y = max_y line" << std::endl;

								dx = Tx-arg_max_x;
								arg_max_y = (fx*fx*(cy+Ty)+fy*(cx*cx+fx*fx+2*cx*dx+dx*dx)*cot_cta)/(fx*fx); //todo: factor out this division
								if(r.min.y<arg_max_y && arg_max_y<r.max.y) {
									std::cout << "trying y = point in region (" << c.ry(arg_max_x,arg_max_y) << ")" << std::endl;
									float projx = (arg_max_x - cx - Tx)/fx; //todo: factor out this division
									float projy = (arg_max_y - cy - Ty)/fy; //todo: factor out this division
									std::cout << "numer: " << -(sin_cta+cos_cta*projy) << std::endl;
									std::cout << "denom: " << sqrt(1+projy*projy+projx*projx) << std::endl;
									cv::line(ry_bgr,cv::Point(arg_max_x-5,arg_max_y),cv::Point(arg_max_x+5,arg_max_y),cv::Scalar(0,0,255));
									cv::line(ry_bgr,cv::Point(arg_max_x,arg_max_y-5),cv::Point(arg_max_x,arg_max_y+5),cv::Scalar(0,0,255));
								}
							}

//							cv::Mat_<cv::Vec3b> rx_bgr = objrec::mat_bgr(rx);
//							cv::imshow("rx",rx_bgr);
							cv::line(ry_bgr,cv::Point(r.min.x,r.min.y),cv::Point(r.max.x,r.min.y),cv::Scalar(0,0,255));
							cv::line(ry_bgr,cv::Point(r.max.x,r.min.y),cv::Point(r.max.x,r.max.y),cv::Scalar(0,0,255));
							cv::line(ry_bgr,cv::Point(r.max.x,r.max.y),cv::Point(r.min.x,r.max.y),cv::Scalar(0,0,255));
							cv::line(ry_bgr,cv::Point(r.min.x,r.max.y),cv::Point(r.min.x,r.min.y),cv::Scalar(0,0,255));
							cv::imshow("ry",ry_bgr);
							cv::waitKey(0);
							return 0;
						}
						float cmin_x, cmax_x;
						c.rx_range(r,cmin_x,cmax_x);
						float amin_x = range_min(rx,r.min.x,r.max.x,r.min.y,r.max.y);
						float amax_x = range_max(rx,r.min.x,r.max.x,r.min.y,r.max.y);
						if(fabs(cmin_x-amin_x)>.174 || fabs(cmax_x-amax_x)>.174) {
							std::cout << "ry camera angle: " << ty*180.0/M_PI << std::endl;
							std::cout << "region: " << r.min.x << " " << r.max.x << " " << r.min.y << " " << r.max.y << std::endl;
							std::cout << "actual range:   " << amin_x << " " << amax_x << std::endl;
							std::cout << "computed range: " << cmin_x << " " << cmax_x << std::endl;
							return 0;
						}
						float cmin_depth, cmax_depth;
						c.depth_range(r,cmin_depth,cmax_depth);
						float amin_depth = range_min(depth,r.min.x,r.max.x,r.min.y,r.max.y);
						float amax_depth = range_max(depth,r.min.x,r.max.x,r.min.y,r.max.y);
						if(fabs(cmin_depth-amin_depth)>.01 || fabs(cmax_depth-amax_depth)>.01) {
							std::cout << "ry camera angle: " << ty*180.0/M_PI << std::endl;
							std::cout << "region: " << r.min.x << " " << r.max.x << " " << r.min.y << " " << r.max.y << std::endl;
							std::cout << "actual depth range:   " << amin_depth << " " << amax_depth << std::endl;
							std::cout << "computed depth range: " << cmin_depth << " " << cmax_depth << std::endl;
							cv::Mat_<cv::Vec3b> depth_bgr = objrec::mat_bgr(depth);
							cv::line(depth_bgr,cv::Point(r.min.x,r.min.y),cv::Point(r.max.x,r.min.y),cv::Scalar(0,0,255));
							cv::line(depth_bgr,cv::Point(r.max.x,r.min.y),cv::Point(r.max.x,r.max.y),cv::Scalar(0,0,255));
							cv::line(depth_bgr,cv::Point(r.max.x,r.max.y),cv::Point(r.min.x,r.max.y),cv::Scalar(0,0,255));
							cv::line(depth_bgr,cv::Point(r.min.x,r.max.y),cv::Point(r.min.x,r.min.y),cv::Scalar(0,0,255));
							cv::imshow("depth",depth_bgr);
							cv::waitKey(0);
							return 0;
						}

					}
				}
			}
		}
	}
}
