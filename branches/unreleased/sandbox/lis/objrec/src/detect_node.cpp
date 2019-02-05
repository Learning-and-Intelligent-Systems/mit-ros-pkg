/*
 * detect_node.cpp
 *
 *  Created on: Aug 3, 2012
 */

#include "args.h"
#include "gui.h"
#include "ros_kinect.h"
#include "objrec/Detect.h"
#include "objrec/DetectOnPolygon.h"

void load_object_files(const std::string& object_list_file,
		//output parameter:
		std::vector<objrec::object> objects) {
	objects.clear();

	size_t last_slash = object_list_file.find_last_of("/\\");
	std::string directory = (last_slash==std::string::npos)?"./":object_list_file.substr(0,last_slash+1);

	std::ifstream object_list(object_list_file.c_str());
	if(!object_list) throw std::runtime_error("Could not open objects file " + object_list_file);
	int line_number = 0;
	while(object_list.good()) {
		std::string line = objrec::nextline(object_list, line_number);
		size_t start_of_object_name = line.find_first_not_of(" \t");
		size_t end_of_object_name = line.find_first_of(" \t",start_of_object_name);
		std::string object_name;
		if(line[start_of_object_name]=='\\' || line[start_of_object_name]=='/') {
			object_name = line.substr(start_of_object_name,end_of_object_name-start_of_object_name);
		}
		else {
			object_name = directory+line.substr(start_of_object_name,end_of_object_name-start_of_object_name);
		}

		std::ifstream in(object_name.c_str());
		objrec::object obj;
		in >> obj;
		std::cout << "Loaded '" << obj.name << "' from " << object_name << " with " << obj.views.size() << " views." << std::endl;
		objects.push_back(obj);
	}
}

std::string world_frame = "/base_link";
std::string camera_frame = "/head_mount_kinect_rgb_link";
std::string cloud_topic = "/head_mount_kinect/depth_registered/points";
std::string camera_info_topic = "/head_mount_kinect/depth/camera_info";
objrec::ros_kinect camera(cloud_topic,camera_info_topic,world_frame,camera_frame);
std::vector<objrec::object> objects;
bool detect(objrec::Detect::Request& req, objrec::Detect::Response& res) {
	if(req.object_index>=(int)objects.size()) return false;
	objrec::object& obj = objects[req.object_index];

	cv::Mat_<cv::Vec3b> picture;
	cv::Mat_<float> depth;
	objrec::camera_info info;
	camera.get(picture,depth,info);
	objrec::image im(obj.visual_words,picture,depth);

	bool print_progress = true;
	bool visualize_search = false;
	objrec::level_constraint constr(info);
	std::vector<objrec::localization> locs;
	float lower_bound = -std::numeric_limits<float>::infinity();
	bool only_top_detection = true;
	objrec::localizations<objrec::level_constraint>(obj,im,lower_bound,locs,only_top_detection,
			print_progress,visualize_search,0,obj.views.size(),constr);

	const objrec::viewpoint& vp = locs[0].v->get_viewpoint();

	//turn this into a pose

	std::cout << "Localization: log probability: " << locs[0].log_probability;
	std::cout << " pixel: (" << locs[0].location.x << "," << locs[0].location.y << ")";
	std::cout << " depth: " << locs[0].location.depth;
	std::cout << " Rx: " << (vp.min_rx+vp.max_rx)/2.0f;
	std::cout << " Ry: " << (vp.min_ry+vp.max_ry)/2.0f;
	std::cout << " Rz: " << (vp.min_rz+vp.max_rz)/2.0f;
	std::cout << std::endl;
	objrec::draw(locs[0],im,picture);
	cv::imshow("detection",picture);
	return true;
}
bool detect_on_polygon(objrec::DetectOnPolygon::Request& req, objrec::DetectOnPolygon::Response& res) {
	if(req.object_index>=(int)objects.size()) return false;
	objrec::object& obj = objects[req.object_index];

	cv::Mat_<cv::Vec3b> picture;
	cv::Mat_<float> depth;
	objrec::camera_info info;
	camera.get(picture,depth,info);
	objrec::image im(obj.visual_words,picture,depth);

	bool print_progress = true;
	bool visualize_search = false;
	objrec::level_constraint constr(info);
	std::vector<objrec::localization> locs;
	float lower_bound = -std::numeric_limits<float>::infinity();
	bool only_top_detection = true;
	objrec::localizations<objrec::level_constraint>(obj,im,lower_bound,locs,only_top_detection,
			print_progress,visualize_search,0,obj.views.size(),constr);

	const objrec::viewpoint& vp = locs[0].v->get_viewpoint();

	//turn this into a pose

	std::cout << "Localization: shifted log probability: " << locs[0].log_probability;
	std::cout << " pixel: (" << locs[0].location.x << "," << locs[0].location.y << ")";
	std::cout << " depth: " << locs[0].location.depth;
	std::cout << " Rxy: " << (vp.min_rz+vp.max_rz)/2.0f;
	std::cout << " Rxz: " << (vp.min_ry+vp.max_ry)/2.0f;
	std::cout << " Ryz: " << (vp.min_rx+vp.max_rx)/2.0f;
	std::cout << std::endl;
	objrec::draw(locs[0],im,picture);
	cv::imshow("detection",picture);
	return true;
}

int main(int argc, char** argv) {
	INIT_PARAMS();
	help += "usage:";
	help += "\n";
	help += "  " + std::string(argv[0]) + " -objects <object_list_file>\n";
	help += "\n";
	help += " Searches in an image (with depth) for high-probability detections of an object model.\n";
	help += "\n";
	help += "Required Parameter\n";
	help += "==================\n";
	ADD_STRING_PARAM(object_list_file,"","A file containing a list of object files (one per line) to load.  If paths are relative, they are resolved relative to the path of the object list file.");
	help += "\n";
	help += "Optional Parameters\n";
	help += "===================\n";
//	ADD_STRING_PARAM(world_frame,"base_link", "The world frame in which object poses are reported.  This frame is assumed to have a z-axis pointing upwards, for detecting objects with an upright-constraint or for horizontal polygons.");
//	ADD_STRING_PARAM(camera_frame,"/head_mount_kinect_rgb_link","The frame of the camera.");
//	ADD_STRING_PARAM(cloud_topic,"/head_mount_kinect/depth_registered/points","The topic that publishes PointCloud2 messages from the camera with position (XYZ), and color (RGB) information.");
//	ADD_STRING_PARAM(camera_info_topic,"/head_mount_kinect/depth/camera_info","The topic that publishes CameraInfo messages for the camera.");
	float inf = std::numeric_limits<float>::infinity();
	ADD_PARAM(lower_bound,-inf,"The the lowest log probability value to search before terminating.");
	ADD_INT_PARAM(only_top_detection,0,"If non-zero, the search will terminate immediately after the first (highest probability) detection in the image.");
	CHECK_PARAMS();

	if(!object_list_file_is_passed) {
		std::cout << "The objects parameter was not passed." << std::endl;
		std::cout << std::endl;
		std::cout << help << std::endl;
		return 0;
	}
	ros::init(argc, argv, "detect_node");
	ros::NodeHandle n;

	load_object_files(object_list_file,objects);

	ros::ServiceServer detect_service = n.advertiseService("detect", detect);
	ros::ServiceServer detect_on_polygon_service = n.advertiseService("detect_on_polygon", detect_on_polygon);
	ros::spin();
}
