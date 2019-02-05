/*
 * io.h
 *
 *  Created on: Feb 5, 2013
 */

#ifndef IO_H_
#define IO_H_

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <iomanip>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <sys/time.h>

namespace objrec {

void advance_to_next_line(std::istream& in, int& line_number, std::string filename="") {
	while(in.good()) {
		std::streampos position = in.tellg();
		std::string line;
		try {
			std::getline(in,line); line_number++;
		}
		catch(const std::exception& e) {
			return;
		}

		size_t line_end = line.find_first_of('#');
		if(line_end==std::string::npos) line_end = line.length();
		for(unsigned int j=0;j<line_end;j++) {
			if(!isspace(line[j])) {
				in.seekg(position);
				line_number--;
				return;
			}
		}
	}
}

std::string nextline(std::istream& in, int& line_number, std::string filename="") {
	advance_to_next_line(in,line_number,filename);
	if(!in.good()) {
		std::ostringstream o;
		o << "error reading from stream on line " << line_number;
		if(filename!="") o << " of file '" << filename << "'";
		throw std::runtime_error(o.str());
	}
	std::string line;
	std::getline(in,line); line_number++;
	size_t comment_pos = line.find_first_of('#');
	line = line.substr(0,comment_pos);
	if(line[line.length()-1]=='\r') { //remove '\r' character at the end of the line for files with lines ending in \r\n
		line = line.substr(0,line.length()-1);
	}
	advance_to_next_line(in,line_number,filename);
	return line;
}

std::string dirname(const std::string& path) {
	size_t last_slash = path.find_last_of('/');
	if(last_slash==std::string::npos) {
		return "";
	} else {
		return path.substr(0,last_slash);
	}
}

std::string basename(const std::string& path) {
	return path.substr(path.find_last_of('/')+1);
}

std::string file_extension(const std::string& path) {
	return path.substr(path.find_last_of('.')+1);
}

inline cv::Mat_<cv::Vec3b> read_image(const std::string& file_name) {
	std::ifstream exists(file_name.c_str());
	if(!exists) {
		throw std::runtime_error("Could not open image " + file_name);
	}
	exists.close();
	return cv::imread(file_name);
}

std::string corresponding_depth_file(const std::string& image_file) {
	size_t dot = image_file.find_last_of('.');
	std::string yml = image_file.substr(0,dot)+".yml";
	std::ifstream yml_file(yml.c_str());
	if(yml_file) {
		return yml;
	} else {
		std::string xml = image_file.substr(0,dot)+".xml";
		std::ifstream xml_file(yml.c_str());
		if(xml_file) {
			return xml;
		} else {
			throw std::runtime_error(
					"Could not find the corresponding depth file for image file '"+image_file+"'.");
		}
	}
}

std::string corresponding_edges_file(const std::string& image_file) {
	size_t dot = image_file.find_last_of('.');
	std::string edges = image_file.substr(0,dot)+".edges";
	std::ifstream edges_file(edges.c_str());
	if(edges_file) {
		return edges;
	} else {
		return "";
	}
}

std::istream& read_edges(std::istream& in,
		//output parameter:
		std::vector<cv::Mat_<float> >& edges) {
	in.exceptions(std::ios::failbit);
	edges.resize(8);
	for(int j=0;j<8;j++) {
		edges[j].create(480,640);
		for(int y=0;y<480;y++) {
			for(int x=0;x<640;x++) {
				try {
					in >> edges[j](y,x);
				} catch(...) {
					std::ostringstream err;
					err << "Error reading edges file at (x=" << x << ",y=" << y << ")." << std::endl;
					throw std::runtime_error(err.str());
				}
			}
		}
	}
	return in;
}
void read_edges(const std::string& edges_file,
		//output parameter:
		std::vector<cv::Mat_<float> >& edges) {
	std::ifstream in(edges_file.c_str());
	read_edges(in,edges);
}


std::string corresponding_pose_file(const std::string& image_file) {
	size_t dot = image_file.find_last_of('.');
	std::string pose = image_file.substr(0,dot)+".pose";
	std::ifstream pose_file(pose.c_str());
	if(pose_file) {
		return pose;
	} else {
		throw std::runtime_error(
				"Could not find the corresponding pose file for image file '"+image_file+"'.");
	}
}


cv::Mat_<float> read_depth(const std::string& depth_file) {
	cv::FileStorage depth_storage(corresponding_depth_file(depth_file),cv::FileStorage::READ);
	cv::FileNode depth_node = depth_storage.getFirstTopLevelNode();
	cv::Mat depth_mat;
	depth_node >> depth_mat;
	cv::Mat_<float> depth;
	if(depth_mat.channels()==3 && depth_mat.elemSize1()==4) { //x,y,z float format, compute depth
		cv::Mat_<cv::Vec3f> d = depth_mat;
		depth.create(d.rows,d.cols);
		for(int j=0;j<d.rows;j++) {
			for(int k=0;k<d.cols;k++) {
				float x = d(j,k)[0];
				float y = d(j,k)[1];
				float z = d(j,k)[2];
				depth(j,k) = sqrt(x*x+y*y+z*z);
			}
		}
	}
	else if(depth_mat.channels()==1 && depth_mat.elemSize1()==4) { //depth already computed
		depth = depth_mat;
	}
	else {
		throw std::runtime_error("Unrecognized depth format in '" + depth_file + "'.");
	}
	return depth;
}


struct scene_info {
	float camera_height_above_floor, camera_tilt_angle;
	float table_height;
};
const char* const scene_info_description =
		"<camera height above floor (m)> <camera tilt angle (degrees from horizontal, positive angles mean camera points downwards)> <table height (m)>";
std::ostream& operator<<(std::ostream& out, const scene_info& si) {
	return out
			<< si.camera_height_above_floor << " "
			<< si.camera_tilt_angle << " "
			<< si.table_height;
}
std::istream& operator>>(std::istream& in, scene_info& si) {
	return in
			>> si.camera_height_above_floor
			>> si.camera_tilt_angle
			>> si.table_height;
}

std::istream& read_scene_info(std::istream& in, scene_info& info, int& line_number) {
	in.exceptions(std::ios::failbit);
	std::istringstream scene_info_line(nextline(in,line_number));
	try {
		scene_info_line >> info;
	}
	catch(...) {
		std::ostringstream err;
		err << "Error reading scene info on line " << line_number << "." << std::endl;
		throw std::runtime_error(err.str());
	}
	return in;
}
std::istream& read_scene_info(std::istream& in, scene_info& info) {
	int line_number = 0;
	return read_scene_info(in,info,line_number);
}
std::string corresponding_scene_file(const std::string& image_file) {
	size_t dot = image_file.find_last_of('.');
	std::string scene = image_file.substr(0,dot)+".scene";
	std::ifstream scene_file(scene.c_str());
	if(scene_file) {
		return scene;
	}
	else {
		return "";
	}
}

template<class T>
void min_max_mat(const cv::Mat_<T>& m, T& min_val, T& max_val) {
	max_val = std::numeric_limits<T>::min();
	min_val = std::numeric_limits<T>::max();
	for(int y=0;y<m.rows;y++) {
		for(int x=0;x<m.cols;x++) {
			if(m(y,x)==m(y,x) &&
					m(y,x)!=std::numeric_limits<T>::infinity() &&
					m(y,x)!=-std::numeric_limits<T>::infinity()) {
				max_val = std::max(max_val,m(y,x));
				min_val = std::min(min_val,m(y,x));
			}
		}
	}
}

template <class T>
cv::Mat_<cv::Vec3b> mat_bgr(const cv::Mat_<T>& m, bool log_val = false,
		double red=255, double green=255, double blue=255) {
	T min_val, max_val;
	min_max_mat(m,min_val,max_val);
	cv::Mat_<cv::Vec3b> s(cv::Size(m.cols,m.rows));
	for(int y=0;y<m.rows;y++) {
		for(int x=0;x<m.cols;x++) {
			if(m(y,x)==m(y,x)) {
				double val;
				if(log_val) {
					double max_log = log(max_val);
					double min_log = log(min_val==0?std::numeric_limits<T>::epsilon():min_val);
					val = (log(m(y,x))-(double)min_log)/((double)max_log-(double)min_log);
				}
				else {
					val = ((double)m(y,x)-(double)min_val)/((double)max_val-(double)min_val);
				}
				unsigned char r = round(red  *val);
				unsigned char g = round(green*val);
				unsigned char b = round(blue *val);
				s(y,x) = cv::Vec3b(b,g,r);
			}
			else s(y,x) = cv::Vec3b(0,0,255);
		}
	}
	return s;
}


inline std::string time_string(double t) {
	std::ostringstream o;
	double minutes = floor(t/60.0);
	double seconds = (int)floor(t)%60;
	double milliseconds = round((t-floor(t))*1000.);
	o << std::setw(2) << std::setfill('0') << minutes << ":";
	o << std::setw(2) << std::setfill('0') << seconds << ".";
	o << std::setw(3) << std::setfill('0') << milliseconds;
	return o.str();
}
long long current_time() {
	struct timeval tv; gettimeofday(&tv, 0);
	return ((long long)tv.tv_sec)*1000000 + tv.tv_usec;
}



} //namespace objrec
#endif /* IO_H_ */
