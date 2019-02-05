/*
 * ros_capture_images.cpp
 *
 *  Created on: Jul 5, 2012
 */

#include "ros_kinect.h"
#include <sys/stat.h>
#include <dirent.h>
#include <iomanip>
#include <opencv2/highgui/highgui.hpp>

bool is_image(const std::string& filename) {
	size_t dot = filename.find_last_of('.');
	if(dot!=std::string::npos) {
		std::string suffix = filename.substr(dot);
		std::transform(suffix.begin(),suffix.end(),suffix.begin(),::tolower);
		return suffix==".bmp" || suffix==".dib" || suffix==".jpeg" || suffix==".jpg" || suffix==".jpe" ||
				suffix==".jp2" || suffix==".png" || suffix==".pbm" || suffix==".pgm" || suffix==".ppm" ||
				suffix==".sr" || suffix==".ras" || suffix==".tiff" || suffix==".tif";
	}
	else return false;
}


int main(int argc, char** argv) {
	if(argc<2) {
		std::cerr << "usage:" << std::endl;
		std::cerr << " " << std::string(argv[0]) << " directory" << std::endl;
		std::cerr << std::endl;
		std::cerr << " Captures image (.png), depth (.yml), and scene (.scene) information from a PR2." << std::endl;
		return -1;
	}

	std::string directory = argv[1];
	struct stat file_stat;
	stat(directory.c_str(), &file_stat);
	int file_number = 1;
	if(S_ISDIR(file_stat.st_mode)) {
		DIR *dir = opendir(directory.c_str());
		if(!dir) throw std::runtime_error("Could not access examples directory '"+directory+"'.");
		dirent *ent;
		while((ent=readdir(dir))!=0) {
			std::string image_file = ent->d_name;
			std::string image_name = directory+"/"+image_file;
			if(is_image(image_name)) {
				size_t dot = image_file.find_last_of('.');
				std::string basename = image_file.substr(0,dot);
				std::istringstream in(basename);
				float this_file_number = std::numeric_limits<float>::quiet_NaN();
				in >> this_file_number;
				if(this_file_number!=this_file_number) continue;
				file_number = std::max((float)file_number,this_file_number+1);
			}
		}
	}
	else {
		throw std::runtime_error("'"+directory+"' is not a directory.");
		return -1;
	}

	ros::init(argc, argv, "ros_capture_images");

	std::string cloud_topic       = "/head_mount_kinect/depth_registered/points";
	std::string camera_info_topic = "/head_mount_kinect/depth/camera_info";
	std::string world_frame       = "/base_link"; //"/torso_lift_link";
	std::string camera_frame      = "/head_mount_kinect_rgb_link";
	objrec::ros_kinect camera(cloud_topic, camera_info_topic, world_frame, camera_frame);
	while(true) {
		cv::Mat_<cv::Vec3b> image;
		cv::Mat_<cv::Vec3f> xyz;
		objrec::scene_info info;
		camera.get(image,xyz,info);
		cv::Mat_<float> depth(cv::Size(xyz.cols,xyz.rows));
		for(int y=0;y<xyz.rows;y++) {
			for(int x=0;x<xyz.cols;x++) {
				const cv::Vec3f& v = xyz(y,x);
				depth(y,x) = sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
			}
		}
		cv::Mat_<cv::Vec3b> depth_rgb = objrec::mat_bgr(depth);
//		cv::Mat_<cv::Vec3b> edges = objrec::edges_bgr(image,depth);

//
		cv::imshow("depth",depth_rgb);
//		cv::imshow("edges",edges);
		cv::imshow("image",image);
		int key = cv::waitKey(100);

		switch(key) {
		case 113: //q
		case 1048689: //q with numlock
		case 27: //esc
		case 1048603: //esc with numlock
			return 0;
		case 32: //space
		case 1048608: //space with numlock
		{
			std::ostringstream basename;
			basename << std::setw(4) << std::setfill('0') << file_number;
			std::string image_name = directory+"/"+basename.str()+".png";
			std::string depth_name = directory+"/"+basename.str()+".yml";
			std::string info_name = directory+"/"+basename.str()+".scene";
			cv::imwrite(image_name,image);
			cv::FileStorage fs(depth_name, cv::FileStorage::WRITE);
			fs << "xyz" << xyz;
			std::ofstream o(info_name.c_str());
			o << "#" << objrec::scene_info_description << std::endl;
			o << info << std::endl;
			std::cerr << "Wrote " << image_name << ", " << depth_name << " and " << info_name << std::endl;
			std::cerr << "scene info:" << std::endl;
			std::cerr << info << std::endl;
			file_number++;
		}
			break;
		}
	}
}
