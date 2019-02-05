/*
 * label_table.cpp
 *
 *  Created on: May 7, 2014
 */

#include "constraints.h"

cv::Mat_<cv::Vec3b> picture;
cv::Mat_<float> depth;
std::vector<cv::Point> points;

float evaluate_table(float camera_tilt_angle, float table_height) {
	objrec::scene_info scene;
	scene.camera_height_above_floor = 0;
	scene.table_height = table_height;
	scene.camera_tilt_angle = camera_tilt_angle;
	objrec::table_constraint tc(scene,0);
	float sum_of_squared_differences = 0;
	for(int j=0;j<points.size();j++) {
		float dd = tc.depth(points[j].x,points[j].y) - depth(points[j]);
		sum_of_squared_differences += dd*dd;
	}
	return sum_of_squared_differences;
}

float table_fit_error(float camera_tilt_angle, float table_height) {
	objrec::scene_info scene;
	scene.camera_height_above_floor = 0;
	scene.table_height = table_height;
	scene.camera_tilt_angle = camera_tilt_angle;
	objrec::table_constraint tc(scene,0);
	float sum_of_differences = 0;
	for(int j=0;j<points.size();j++) {
		sum_of_differences += tc.depth(points[j].x,points[j].y) - depth(points[j]);
	}
	return sum_of_differences;
}


float best_table_height(float camera_tilt_angle) {
//	float table_height_upper_bound = 10;
//	float table_height_lower_bound = -10;
//	float table_height = 0;
//	while(table_height_upper_bound-table_height_lower_bound>.001) {
//		float table_error = table_fit_error(camera_tilt_angle,table_height);
//		if(table_error>0) {
//			table_height_upper_bound = table_height;
//		} else {
//			table_height_lower_bound = table_height;
//		}
//		table_height = (table_height_upper_bound + table_height_lower_bound)*.5;
//	}
	float best_table_height = 0;
	float best_table_evaluation = std::numeric_limits<float>::infinity();
	for(float table_height = -10;table_height<10;table_height += .01) {
		float this_table_evaluation = evaluate_table(camera_tilt_angle,table_height);
		if(this_table_evaluation<best_table_evaluation) {
			best_table_evaluation = this_table_evaluation;
			best_table_height = table_height;
		}
	}
	return best_table_height;
}

void draw_table_fit(float camera_tilt_angle, float table_height) {
	objrec::scene_info scene;
	scene.camera_height_above_floor = 0;
	scene.table_height = table_height;
	scene.camera_tilt_angle = camera_tilt_angle;
	objrec::table_constraint tc(scene, 0);
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
}

float best_camera_tilt_angle;

void best_fit_table() {
	best_camera_tilt_angle = 0;
	float best_table_evaluation = std::numeric_limits<float>::infinity();
	for(float camera_tilt_angle = 0;camera_tilt_angle<90;camera_tilt_angle += .1) {
		float table_height = best_table_height(camera_tilt_angle);
		float this_table_evaluation = evaluate_table(camera_tilt_angle,table_height);
		if(this_table_evaluation<best_table_evaluation) {
			best_table_evaluation = this_table_evaluation;
			best_camera_tilt_angle = camera_tilt_angle;
		}
	}
	float table_height = best_table_height(best_camera_tilt_angle);
	draw_table_fit(best_camera_tilt_angle,table_height);
	std::cout << "camera tilt angle: " << best_camera_tilt_angle << std::endl;
	std::cout << "table height: " << table_height << std::endl;
}

void mouse_handler(int event, int x, int y, int flags, void* userdata) {
	if(event==cv::EVENT_LBUTTONUP) {
		points.push_back(cv::Point(x,y));
		int r = 4;
		cv::line(picture,cv::Point(x+r,y),cv::Point(x-r,y),cv::Scalar(0,255,0));
		cv::line(picture,cv::Point(x,y+r),cv::Point(x,y-r),cv::Scalar(0,255,0));
		cv::imshow("image",picture);
		if(points.size()>=3) {
			best_fit_table();
		}
	}
}

int main(int argc, char** argv) {
	if(argc<2) {
		std::cout << "usage:" << std::endl;
		std::cout << argv[0] << " 0001.png" << std::endl;
		return -1;
	}
	long long start, end;

	std::cerr << "Loading picture file '" << argv[1] << "'... " << std::flush;
	start = objrec::current_time();
	cv::Mat_<cv::Vec3b> original_picture;
	original_picture = cv::imread(argv[1]);
	picture = original_picture.clone();
	end = objrec::current_time();
	std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;

	std::string depth_file = objrec::corresponding_depth_file(argv[1]);
	std::cerr << "Loading depth file '" << depth_file << "'... " << std::flush;
	start = objrec::current_time();
	depth = objrec::read_depth(depth_file);
	end = objrec::current_time();
	std::cerr << " done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;

	cv::imshow("image",picture);
	cv::imshow("depth",objrec::mat_bgr(depth));

	cv::setMouseCallback("image", mouse_handler, NULL);

	std::string scene_file = objrec::corresponding_scene_file(argv[1]);
	bool done = false;
	while(!done) {
		int key = cv::waitKey(0);
		switch(key) {
		case 113: //q
		case 1048689: //q with numlock
		case 27: //esc
		case 1048603: //esc with numlock
			std::cerr << "Did NOT write changes to " << scene_file << "." << std::endl;
			done = true;
			break;
		case 10: //enter
		case 65421: //numpad enter
		case 1048586: //enter with numlock
		case 32: //space
		case 1048608: { //space with numlock
			objrec::scene_info scene;
			scene.camera_height_above_floor = 0;
			scene.table_height = best_table_height(best_camera_tilt_angle);
			scene.camera_tilt_angle = best_camera_tilt_angle;
			std::ofstream out(scene_file.c_str());
			out << "#" << objrec::scene_info_description << std::endl;
			out << scene;
			std::cerr << "Wrote changes to " << scene_file << "." << std::endl;
			done = true;
			break;
		}
		case 65288: //backspace
		case 1113864: //backspace with numlock
			points.clear();
			std::cerr << "Restarting." << std::endl;
			picture = original_picture.clone();
			cv::imshow("image",picture);
			break;
		default:
			std::cerr << "Unrecognized key: " << key << std::endl;
			break;
		}
	}
}
