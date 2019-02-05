/*
 * edge_detect.h
 *
 *  Created on: Sep 11, 2012
 */

#ifndef EDGE_DETECT_H_
#define EDGE_DETECT_H_

#include "io.h"
#include <opencv2/imgproc/imgproc.hpp>
#define TWO_THRESHOLD_FEATURES

namespace objrec {

#ifdef TWO_THRESHOLD_FEATURES
void detect_edges( const cv::Mat_<unsigned char>& im, float low_threshold, float high_threshold,
		//output parameters:
		cv::Mat_<float>& low_threshold_edges, cv::Mat_<float>& high_threshold_edges);
#else
void detect_edges( const cv::Mat_<unsigned char>& im, float low_threshold, float high_threshold,
		//output parameter:
		cv::Mat_<float>& edges);
#endif

class edge_direction {
	double min_angle_, max_angle_;
	void initialize(double min_angle, double max_angle) {
		const double rad = M_PI/180.0;
		if(min_angle>=90) {
			min_angle_ = (min_angle - 180.0)*rad;
		} else {
			min_angle_ = min_angle*rad;
		}
		if(max_angle>=90) {
			max_angle_ = (max_angle - 180.0)*rad;
		} else {
			max_angle_ = max_angle*rad;
		}
	}
	inline double average_radians() const {
		double max_angle = max_angle_;
		if(max_angle_<min_angle_) max_angle = max_angle_ + M_PI;
		return (max_angle + min_angle_)/2.0;
	}
public:
	edge_direction() {}
	edge_direction(std::istream& in) {
		double min_angle, max_angle;
		in >> min_angle >> max_angle;
		initialize(min_angle,max_angle);
	}
	edge_direction(double min_angle, double max_angle) { initialize(min_angle,max_angle); }
	edge_direction(const edge_direction& ed) : min_angle_(ed.min_angle_), max_angle_(ed.max_angle_) {}
	inline double average_degrees() const { return average_radians()*180.0/M_PI; }
	void draw(cv::Mat_<cv::Vec3b>& picture, int x, int y, int radius, cv::Scalar color = cv::Scalar(0,0,0)) const {
		double a = average_radians()+M_PI/2.0;
		int fx = round((double)radius*cos(a));
		int fy = round((double)radius*sin(a));
		cv::line(picture,cv::Point(x+fx,y+fy),cv::Point(x-fx,y-fy),color);
	}
	bool operator==(const edge_direction& ed) const {return min_angle_==ed.min_angle_ && max_angle_==ed.max_angle_;}
	bool operator<(const edge_direction& ed) const { //used by std::set in combine_objects.cpp
		if(min_angle_==ed.min_angle_) {
			return max_angle_<ed.max_angle_;
		} else {
			return min_angle_<ed.min_angle_;
		}
	}
	bool in_range(float angle) const {
//		if(angle!=angle) return false; //ignore NaN, but implied by the code below
		if(min_angle_<max_angle_) {
			return min_angle_ <= angle && angle < max_angle_;
		}
		else {
			return min_angle_ <= angle || angle < max_angle_;
		}
	}

	friend std::ostream& operator<<(std::ostream& out, const edge_direction& ed);
};

const char* const edge_direction_description = "<min edge angle (degrees)> <max edge angle (degrees)>";
std::ostream& operator<<(std::ostream& out, const edge_direction& ed) {
	return out << ed.min_angle_*180.0/M_PI << " " << ed.max_angle_*180.0/M_PI;
}

class edge_detection_parameters {
	float low_threshold, high_threshold;
	std::vector<edge_direction> directions;
	friend std::ostream& operator<<(std::ostream& out, const edge_detection_parameters& edp);
public:
	bool compatible_with(const edge_detection_parameters& other) const {
		return low_threshold==other.low_threshold && high_threshold==other.high_threshold;
	}
	bool operator==(const edge_detection_parameters& other) const {
		if(low_threshold!=other.low_threshold || high_threshold!=other.high_threshold || directions.size()!=other.directions.size()) {
			return false;
		}
		for(unsigned int j=0;j<directions.size();j++) {
			if(!(directions[j]==other.directions[j])) return false;
		}
		return true;
	}

	edge_detection_parameters(int n_edge_directions=8, float edge_bin_overlap=1.0, float low_threshold=800, float high_threshold=2400)
	: low_threshold(low_threshold), high_threshold(high_threshold) {
		for(int j=0;j<n_edge_directions;j++) {
			double c = (double)j*-180.0/(double)n_edge_directions;
			double w = edge_bin_overlap*180.0/(double)n_edge_directions;
			double min_angle = c-w;
			if(min_angle<0.0) min_angle+=180.0;
			double max_angle = c+w;
			if(max_angle>=180.0) max_angle-=180.0;
			else if(max_angle<min_angle && max_angle<0) max_angle+=180.0;
			directions.push_back(edge_direction(min_angle,max_angle));
		}
	}
	edge_detection_parameters(float low_threshold, float high_threshold, const std::vector<edge_direction> directions)
	: low_threshold(low_threshold), high_threshold(high_threshold), directions(directions) {}
	edge_detection_parameters(const edge_detection_parameters& other, const std::vector<edge_direction> directions)
	: low_threshold(other.low_threshold), high_threshold(other.high_threshold), directions(directions) {}
	edge_detection_parameters(const edge_detection_parameters& other)
	: low_threshold(other.low_threshold), high_threshold(other.high_threshold), directions(other.directions) {}
	edge_detection_parameters(std::istream& in, int& line_number) {
		in.exceptions(std::ios::failbit | std::ios::badbit);

		std::istringstream n_edge_directions_line(nextline(in,line_number));
		int n_edge_directions;
		try {
			n_edge_directions_line >> n_edge_directions;
		}
		catch(...) {
			std::ostringstream err;
			err << "Error reading number of edge directions on line " << line_number;
			throw std::runtime_error(err.str());
		}

		directions.reserve(n_edge_directions);
		for(int j=0;j<n_edge_directions;j++) {
			std::istringstream edge_direction_line(nextline(in,line_number));
			try {
				edge_direction ed(edge_direction_line);
				directions.push_back(ed);
			} catch(...) {
				std::ostringstream err;
				err << "Error reading edge direction " << j+1 << " of " << n_edge_directions
						<< " on line " << line_number;
				throw std::runtime_error(err.str());
			}
		}

		std::istringstream edge_parameters_line(nextline(in,line_number));
		try {
			edge_parameters_line >> low_threshold >> high_threshold;
		}
		catch(...) {
			std::ostringstream err;
			err << "Error reading edge parameters on line " << line_number;
			throw std::runtime_error(err.str());
		}
	}

	//ignore depth channel for now... later use depth for edge detection
#ifdef TWO_THRESHOLD_FEATURES
	void edge_detect(const cv::Mat_<cv::Vec3b>& picture, const cv::Mat_<float>& depth,
			//output parameters
			cv::Mat_<float>& low_threshold_edges, cv::Mat_<float>& high_threshold_edges) const {
		cv::Mat_<unsigned char> gray;
		cv::cvtColor(picture,gray,CV_BGR2GRAY);
		cv::Mat_<float> edges;
		detect_edges(gray, low_threshold, high_threshold,
				//output parameters:
				low_threshold_edges, high_threshold_edges);
	//	cv::Canny(gray,edges,p.low_threshold,p.high_threshold,5); //requires angles for each edge, so can't use OpenCV implementation
	}
#else
	cv::Mat_<float> edge_detect(const cv::Mat_<cv::Vec3b>& picture, const cv::Mat_<float>& depth) const {
		cv::Mat_<unsigned char> gray;
		cv::cvtColor(picture,gray,CV_BGR2GRAY);
		cv::Mat_<float> edges;
		detect_edges(gray, low_threshold, high_threshold, edges);
	//	cv::Canny(gray,edges,p.low_threshold,p.high_threshold,5); //requires angles for each edge, so can't use OpenCV implementation
		return edges;
	}
#endif
	float get_low_threshold() const { return low_threshold; }
	float get_high_threshold() const { return high_threshold; }
	const std::vector<edge_direction>& get_edge_directions() const {return directions;}
};

std::ostream& operator<<(std::ostream& out, const edge_detection_parameters& edp) {
	out << "#<number of edge directions>" << std::endl;
	out << edp.directions.size() << std::endl;
	out << " #" << edge_direction_description << std::endl;
	for(int j=0;j<(int)edp.directions.size();j++) {
		out << " " <<  edp.directions[j] << std::endl;
	}
	out << "#<low threshold> <high threshold>" << std::endl;
	out << edp.low_threshold << " " << edp.high_threshold;
	return out;
}

template <class T>
void set_to(cv::Mat_<T>& m, T value=T()) {
	for(int y=0;y<m.rows;y++) {
		for(int x=0;x<m.cols;x++) {
			m(y,x) = value;
		}
	}
}

cv::Mat_<cv::Vec3b> edges_bgr(const cv::Mat_<cv::Vec3b>& picture, const cv::Mat_<float>& depth,
		const edge_detection_parameters& p = edge_detection_parameters());

cv::Mat_<cv::Vec3b> edges_bgr(const std::vector<cv::Mat_<float> >& precomputed_edges, const edge_detection_parameters& p = edge_detection_parameters());
} //namespace objrec
#endif /* EDGE_DETECT_H_ */
