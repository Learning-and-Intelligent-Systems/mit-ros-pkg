/*
 * representation.h
 *
 *  Created on: Mar 28, 2012
 */

#ifndef REPRESENTATION_H_
#define REPRESENTATION_H_

#include "io.h"
#include "features.h"
#include <limits>
#include <cmath>

namespace objrec {

///normal distribution with a receptive field radius. immutable.
class normal_distribution {
	float b, c, r, r2;
public:
	normal_distribution() : b(std::numeric_limits<float>::quiet_NaN()) {}
	normal_distribution(float variance, float receptive_field_radius) :
		b(-1.0f/(2.0f*variance)), r(receptive_field_radius), r2(receptive_field_radius*receptive_field_radius) {
		if(variance<0) {
			std::ostringstream err;
			err << "The variance parameter (variance=" << variance << ") to a normal distribution must be non-negative.";
			throw std::invalid_argument(err.str());
		}
		if(receptive_field_radius<0) {
			std::ostringstream err;
			err << "The receptive field radius (receptive_field_radius=" << receptive_field_radius
					<< ") of a normal distribution must be positive.";
			throw std::invalid_argument(err.str());
		}
	}
	inline float shifted_log_probability(float dist2) const {return b*std::min(dist2,r2);}
	float constant_shift() const {return 0.5f*log(-b/M_PI);}
	bool outlier(float dist2) const {return dist2>r2;}
	inline float receptive_field_radius() const {return r;}
	float variance() const {return -1.0f/(2.0f*b);}
};
#define NORMAL_DESCRIPTION "<variance> <receptive field radius>"
std::ostream& operator<<(std::ostream& out, const normal_distribution& n) {
	return out << n.variance() << " " << n.receptive_field_radius();
}
std::istream& operator>>(std::istream& in, normal_distribution& g) {
	float variance, receptive_field_radius;
	in >> variance >> receptive_field_radius;
	g = normal_distribution(variance,receptive_field_radius);
	return in;
}

struct point {
	float x, y, depth, rx, ry, rz;
	inline point() : x(std::numeric_limits<float>::quiet_NaN()),
			y(std::numeric_limits<float>::quiet_NaN()),
			depth(std::numeric_limits<float>::quiet_NaN()),
			rx(std::numeric_limits<float>::quiet_NaN()),
			ry(std::numeric_limits<float>::quiet_NaN()),
			rz(std::numeric_limits<float>::quiet_NaN()) {}
	inline point(float x, float y, float depth, float rx, float ry, float rz) : x(x), y(y), depth(depth), rx(rx), ry(ry), rz(rz) {}
	inline bool operator==(const point& other) const {return x==other.x && y==other.y && depth==other.depth
			&& rx==other.rx && ry==other.ry && rz==other.rz;}
};
std::ostream& operator<<(std::ostream& out, const point& p) {
	return out << p.x << " " << p.y << " " << p.depth << " " << p.rx << " " << p.ry << " " << p.rz;
}
std::istream& operator>>(std::istream& in, point& p) {
	return in >> p.x >> p.y >> p.depth >> p.rx >> p.ry >> p.rz;
}

struct visual_part {
	normal_distribution distribution;
#ifdef TWO_THRESHOLD_FEATURES
	float high_threshold_log_probability_shift;
#endif
	float cx, cy; //constant added to x- and y-coordinates in the image plane
	float xrx, xry, xrz; //change in x-coordinate in the image plane, as a linear function of the object rotation
	float yrx, yry, yrz; //change in y-coordinate in the image plane, as a linear function of the object rotation
	inline float expected_pixel_x(const point& p) const {
		return p.x+(cx+xrx*p.rx+xry*p.ry+xrz*p.rz
				)/p.depth;
	}
	inline float expected_pixel_y(const point& p) const {
		return p.y+(cy+yrx*p.rx+yry*p.ry+yrz*p.rz
				)/p.depth;
	}
	visual_part(std::istream& in) {
		in >> distribution >>
#ifdef TWO_THRESHOLD_FEATURES
			high_threshold_log_probability_shift >>
#endif
			cx >> cy >>
			xrx >> xry >> xrz >>
			yrx >> yry >> yrz;
	}
	visual_part(const normal_distribution& distribution,
#ifdef TWO_THRESHOLD_FEATURES
			float high_threshold_log_probability_shift,
#endif
			float cx, float cy,
			float pxrx, float pxry, float pxrz,
			float pyrx, float pyry, float pyrz) :
		distribution(distribution),
#ifdef TWO_THRESHOLD_FEATURES
		high_threshold_log_probability_shift(high_threshold_log_probability_shift),
#endif
		cx(cx), cy(cy),
		xrx(pxrx), xry(pxry), xrz(pxrz),
		yrx(pyrx), yry(pyry), yrz(pyrz) {}
};
#ifdef TWO_THRESHOLD_FEATURES
const char* const visual_part_description = NORMAL_DESCRIPTION " <high threshold log probability shift> <cx> <cy> <xrx> <xry> <xrz> <yrx> <yry> <yrz>";
#else
const char* const visual_part_description = NORMAL_DESCRIPTION " <cx> <cy> <xrx> <xry> <xrz> <yrx> <yry> <yrz>";
#endif
std::ostream& operator<<(std::ostream& out, const visual_part& vwp) {
	return out << vwp.distribution << " " <<
#ifdef TWO_THRESHOLD_FEATURES
			vwp.high_threshold_log_probability_shift << " " <<
#endif
			vwp.cx << " " << vwp.cy << " " <<
			vwp.xrx << " " << vwp.xry << " " << vwp.xrz << " " <<
			vwp.yrx << " " << vwp.yry << " " << vwp.yrz;
}

struct depth_part {
	normal_distribution distribution;
	float x, y; //x- and y-coordinates in the image plane. todo: possibly change these to shorts?
	float cdepth; //constant added to the depth
	float depthrx, depthry, depthrz; //change in depth, as a linear function of the object rotation
	inline float pixel_x(const point& p) const { return p.x + x/p.depth; }
	inline float pixel_y(const point& p) const { return p.y + y/p.depth; }
	inline float expected_depth(const point& p) const {
		return p.depth+cdepth+depthrx*p.rx+depthry*p.ry+depthrz*p.rz;
	}
	depth_part() {}
	depth_part(const normal_distribution& distribution,
			float x, float y, float cd, float drx, float dry, float drz) :
				distribution(distribution),
				x(x), y(y), cdepth(cd), depthrx(drx), depthry(dry), depthrz(drz) {}
};
const char* const depth_part_description = NORMAL_DESCRIPTION " <x> <y> <cdepth> <depthrx> <depthry> <depthrz>";
std::ostream& operator<<(std::ostream& out, const depth_part& dp) {
	return out << dp.distribution << " " <<
			dp.x << " " << dp.y << " " <<
			dp.cdepth << " " <<
			dp.depthrx << " " << dp.depthry << " " << dp.depthrz;
}
std::istream& operator>>(std::istream& in, depth_part& dp) {
	return in >> dp.distribution >> dp.x >> dp.y >> dp.cdepth >> dp.depthrx >> dp.depthry >> dp.depthrz;
}

class view; //defined below

struct object {
	std::string name;
	std::vector<view> views;
	feature_library library;
	float ddepth_when_missing;
	float object_center_height_above_table;
	object() : name("unnamed"), ddepth_when_missing(.018) {};
	object(const object& other);
};

struct viewpoint {
	viewpoint() {}
	viewpoint(const viewpoint& vp) :
		min_depth(vp.min_depth), max_depth(vp.max_depth),
		min_rx(vp.min_rx), max_rx(vp.max_rx),
		min_ry(vp.min_ry), max_ry(vp.max_ry),
		min_rz(vp.min_rz), max_rz(vp.max_rz) {}
	/// The minimum distance (in meters) from the camera to the center of the object.
	float min_depth;
	/// The maximum distance (in meters) from the camera to the center of the object.
	float max_depth;
	/// The minimum rotation (in degrees) in the image plane of the object represented by this viewpoint.
	float min_rx;
	/// The maximum rotation (in degrees) in the image plane of the object represented by this viewpoint.
	float max_rx;
	/// The minimum rotation (in degrees) tilt around the y-axis of the object represented by this viewpoint.
	float min_ry;
	/// The maximum rotation (in degrees) tilt around the y-axis of the object represented by this viewpoint.
	float max_ry;
	/// The minimum rotation (in degrees) as if on a turn-table of the object represented by this viewpoint.
	float min_rz;
	/// The maximum rotation (in degrees) as if on a turn-table of the object represented by this viewpoint.
	float max_rz;
};
const char* const viewpoint_description =
		"<min depth> <max depth> <min rx> <max rx> <min ry> <max ry> <min rz> <max rz>";
std::ostream& operator<<(std::ostream& out, const viewpoint& vp) {
	return out
			<< vp.min_depth << " " << vp.max_depth << " "
			<< vp.min_rx << " " << vp.max_rx << " "
			<< vp.min_ry << " " << vp.max_ry << " "
			<< vp.min_rz << " " << vp.max_rz;
}
std::istream& operator>>(std::istream& in, viewpoint& vp) {
	return in >> vp.min_depth >> vp.max_depth
			>> vp.min_rx >> vp.max_rx
			>> vp.min_ry >> vp.max_ry
			>> vp.min_rz >> vp.max_rz;
}
bool operator==(const viewpoint& a, const viewpoint& b) {
	return
			a.min_depth==b.min_depth &&
			a.max_depth==b.max_depth &&
			a.min_rx==b.min_rx &&
			a.max_rx==b.max_rx &&
			a.min_ry==b.min_ry &&
			a.max_ry==b.max_ry &&
			a.min_rz==b.min_rz &&
			a.max_rz==b.max_rz;
}

///a single view of an object. immutable.
class view {
private:
	const object *obj;
public:
	inline const object* get_parent_object() const {return obj;}

private:
	viewpoint vp;
public:
	inline const viewpoint& get_viewpoint() const {return vp;}

private:
	float constant_shift;
public:
	float get_constant_shift() const { return constant_shift; }

private:
	/// The visual parts that make up this view. The outer vector has obj->library.size() vectors, one for each feature in the library
	std::vector<std::vector<visual_part> > visual_parts;
public:
	unsigned int n_visual_parts() const {
		unsigned int n = 0;
		for(std::vector<std::vector<visual_part> >::const_iterator e=visual_parts.begin();
				e!=visual_parts.end();e++) n += e->size();
		return n;
	}

	inline const std::vector<std::vector<visual_part> >& get_visual_parts() const { return visual_parts; }

private:
	/// The depth parts that make up this view.
	std::vector<depth_part> depth_parts;
public:
	inline const std::vector<depth_part>& get_depth_parts() const { return depth_parts; }

	view() : obj(0) {}

	view(const object* parent_object, const viewpoint& vp,
			const std::vector<std::vector<visual_part> >& visual_parts,
			const std::vector<depth_part>& depth_parts) : obj(parent_object), vp(vp),
			constant_shift(0),
			visual_parts(visual_parts),
			depth_parts(depth_parts) {
		for(std::vector<std::vector<visual_part> >::const_iterator j=visual_parts.begin();j!=visual_parts.end();j++) {
			for(std::vector<visual_part>::const_iterator k=j->begin();k!=j->end();k++) {
				constant_shift += k->distribution.constant_shift();
			}
		}
		for(std::vector<depth_part>::const_iterator f=depth_parts.begin();f!=depth_parts.end();f++) {
			constant_shift += f->distribution.constant_shift();
		}
	}

	///copy all except parent object
	view(const view& other, const object* parent_object) : obj(parent_object), vp(other.vp),
			constant_shift(other.constant_shift), visual_parts(other.visual_parts),depth_parts(other.depth_parts) {}

	view(const object* parent_object, std::istream& in, int& line_number);

	inline bool operator==(const view& v) const { return v.vp == vp; }

	void draw(const point& p,
			//output argument:
			cv::Mat_<cv::Vec3b>& drawing_image) const {
		int max_feature_radius = 3.f/p.depth;
		cv::Scalar color(0,0,0);
		const std::vector<edge_direction>& edge_directions = obj->library.get_edges().get_edge_directions();
		for(int j=0;j<visual_parts.size();j++) {
			for(std::vector<visual_part>::const_iterator k=visual_parts[j].begin();k!=visual_parts[j].end();k++) {
				const normal_distribution& d = k->distribution;
				int x = k->expected_pixel_x(p);
				int y = k->expected_pixel_y(p);
				if(j<edge_directions.size()) {
					edge_directions[j].draw(drawing_image,x,y,max_feature_radius,color);
				} else {
					cv::circle(drawing_image,cv::Point(x,y),2,color,-1);
				}
			}
		}

		float min_depth = std::numeric_limits<float>::infinity();
		float max_depth = -std::numeric_limits<float>::infinity();
		for(std::vector<depth_part>::const_iterator j=depth_parts.begin();j!=depth_parts.end();j++) {
			min_depth = std::min(min_depth,j->expected_depth(p));
			max_depth = std::max(max_depth,j->expected_depth(p));
		}

		for(std::vector<depth_part>::const_iterator j=depth_parts.begin();j!=depth_parts.end();j++) {
			int c = 255.*(j->expected_depth(p)-min_depth)/(max_depth-min_depth);

			c = std::max(std::min(c,255),0);

			cv::Scalar color(c,c,0);
			int x = j->pixel_x(p);
			int y = j->pixel_y(p);
			cv::circle(drawing_image,cv::Point(x,y),max_feature_radius,color,-1);
		}
	}
};

inline object::object(const object& other) : name(other.name),
		library(other.library), ddepth_when_missing(other.ddepth_when_missing),
		object_center_height_above_table(other.object_center_height_above_table) {
	views.reserve(other.views.size());
	for(int j=0;j<(int)other.views.size();j++) {
		views.push_back(view(other.views[j],this));
	}
}

inline view* find_view(object& obj, float rx, float ry, float rz) {
	view* best_view = &obj.views[0];
	float best_dist = std::numeric_limits<float>::infinity();
	for(int j=0;j<(int)obj.views.size();j++) {
		view* v = &obj.views[j];
		const viewpoint& vpj = v->get_viewpoint();
		float mid_rx = (vpj.min_rx+vpj.max_rx)/2.f;
		float mid_ry = (vpj.min_ry+vpj.max_ry)/2.f;
		float mid_rz = (vpj.min_rz+vpj.max_rz)/2.f;
//		if(fabs(mid_rx+360-rx)<fabs(mid_rx-rx)) mid_rx += 360;
//		if(fabs(mid_ry+360-ry)<fabs(mid_ry-ry)) mid_ry += 360;
//		if(fabs(mid_rz+360-rz)<fabs(mid_rz-rz)) mid_rz += 360;
//		if(fabs(mid_rx-360-rx)<fabs(mid_rx-rx)) mid_rx -= 360;
//		if(fabs(mid_ry-360-ry)<fabs(mid_ry-ry)) mid_ry -= 360;
//		if(fabs(mid_rz-360-rz)<fabs(mid_rz-rz)) mid_rz -= 360;
//		if(fabs(fmod(mid_rx,360)-rx)<fabs(mid_rx-rx)) mid_rx = fmod(mid_rx,360);
//		if(fabs(fmod(mid_ry,360)-ry)<fabs(mid_ry-ry)) mid_ry = fmod(mid_ry,360);
//		if(fabs(fmod(mid_rz,360)-rz)<fabs(mid_rz-rz)) mid_rz = fmod(mid_rz,360);
		mid_rx = fmod(mid_rx,360);
		mid_ry = fmod(mid_ry,360);
		mid_rz = fmod(mid_rz,360);

		float drx = mid_rx-rx;
		float dry = mid_ry-ry;
		float drz = mid_rz-rz;
		float dist = drx*drx + dry*dry + drz*drz;
		if(dist<best_dist) {
			best_dist = dist;
			best_view = v;
		}
	}
	return best_view;
//	for(int j=0;j<(int)obj.views.size();j++) {
//		view& v = obj.views[j];
//		const viewpoint& vpj = v.get_viewpoint();
//
//		if(
//				vpj.min_rx <= rx && rx <= vpj.max_rx &&
//				vpj.min_ry <= ry && ry <= vpj.max_ry &&
//				vpj.min_rz <= rz && rz <= vpj.max_rz
//				) {
//			return &v;
//		}
//	}
//	return &initial_view;
}

std::ostream& operator<<(std::ostream& out, const view& v) {
	out << "# " << viewpoint_description << std::endl;
	out << " " << v.get_viewpoint() << std::endl;
	const std::vector<std::vector<visual_part> > & visual_parts = v.get_visual_parts();
	out << "# <number of visual parts>" << std::endl;
	out << " " << v.n_visual_parts() << std::endl;
	if(v.n_visual_parts()>0) {
		out << "#  <visual feature index> " << visual_part_description << std::endl;
		for(int j=0;j<visual_parts.size();j++) {
			for(std::vector<visual_part>::const_iterator k=visual_parts[j].begin();k!=visual_parts[j].end();k++) {
				out << "  " << j << " " << *k << std::endl;
			}
		}
	}

	out << "# <number of depth parts>" << std::endl;
	const std::vector<depth_part>& depth_parts = v.get_depth_parts();
	out << " " << depth_parts.size() << std::endl;
	if(depth_parts.size()>0) {
		out << "#  " << depth_part_description << std::endl;
		for(std::vector<depth_part>::const_iterator p=depth_parts.begin();p!=depth_parts.end();p++) {
			out << "  " << *p << std::endl;
		}
	}
	return out;
}

view::view(const object* parent_object, std::istream& in, int& line_number) : obj(parent_object), constant_shift(0),
		visual_parts(obj->library.size()) {
	in.exceptions(std::ios::failbit);
	std::istringstream viewpoint_line(nextline(in,line_number));

	try {
		viewpoint_line >> vp;
	} catch(...) {
		std::ostringstream err;
		err << "Error reading viewpoint for view '" << viewpoint_line.str() << "' on line " << line_number << std::endl;
		throw std::runtime_error(err.str());
	}

	std::istringstream n_image_plane_features_line(nextline(in,line_number));
	unsigned int n_visual_parts_;

	try {
		n_image_plane_features_line >> n_visual_parts_;
	} catch(...) {
		std::ostringstream err;
		err << "Error reading number of parts for view '" << viewpoint_line.str() << "' on line " << line_number << std::endl;
		throw std::runtime_error(err.str());
	}

	for(unsigned int j=0;j<n_visual_parts_;j++) {
		try {
			std::istringstream line(nextline(in,line_number));
			int visual_feature_index;
			line >> visual_feature_index;
			visual_parts.resize(visual_feature_index+1);
			visual_part vwp(line);
			visual_parts[visual_feature_index].push_back(vwp);
			constant_shift += vwp.distribution.constant_shift();
		}
		catch(...) {
			std::ostringstream err;
			err << "Error reading visual part " << j+1 << " of " << n_visual_parts_
					<< " for view '" << viewpoint_line.str() << "' on line " << line_number << std::endl;
			throw std::runtime_error(err.str());
		}
	}

	std::istringstream n_depth_features_line(nextline(in,line_number));
	unsigned int n_depth_parts;
	try {
		n_depth_features_line >> n_depth_parts;
	} catch(...) {
		std::ostringstream err;
		err << "Error reading number of depth parts for view '" << viewpoint_line.str() << "' on line " << line_number << std::endl;
		throw std::runtime_error(err.str());
	}

	for(unsigned int j=0;j<n_depth_parts;j++) {
		depth_part dp;
		std::istringstream line(nextline(in,line_number));
		try {
			line >> dp;
		} catch(...) {
			std::ostringstream err;
			err << "Error reading depth part " << j+1 << " of " << n_depth_parts
					<< " for view '" << viewpoint_line.str() << "' on line " << line_number << std::endl;
			throw std::runtime_error(err.str());
		}
		depth_parts.push_back(dp);
		constant_shift += depth_parts.back().distribution.constant_shift();
	}
}

std::ostream& operator<<(std::ostream& out, const object& obj) {

	out << "#<object_name>" << std::endl;
	out << obj.name << std::endl;
	out << "#<height of the origin of the object mesh above the table in meters>" << std::endl;
	out << obj.object_center_height_above_table << std::endl;
	out << "#<depth difference when depth sensor value is missing>" << std::endl;
	out << obj.ddepth_when_missing << std::endl;
	out << obj.library;
	out << "#<number of views>" << std::endl;
	out << obj.views.size() << std::endl;
	out << std::endl;
	for(int j=0;j<(int)obj.views.size();j++) {
		out << "# view number " << j+1 << " of " << obj.views.size() << " in object " << obj.name << std::endl;
		out << obj.views[j] << std::endl;
		out << std::endl;
	}
	return out;
}


std::istream& operator>>(std::istream& in, object& obj) {
	in.exceptions(std::ios::failbit | std::ios::badbit);
	int line_number = 0;
	obj.name = nextline(in,line_number);

	std::istringstream object_center_height_above_table_line(nextline(in,line_number));
	try {
		object_center_height_above_table_line >> obj.object_center_height_above_table;
	} catch(...) {
		std::ostringstream err;
		err << "Error reading the height of the origin of the object mesh above the table on line " << line_number;
		throw std::runtime_error(err.str());
	}
	std::istringstream ddepth_when_missing_line(nextline(in,line_number));
	try {
		ddepth_when_missing_line >> obj.ddepth_when_missing;
	} catch(...) {
		std::ostringstream err;
		err << "Error reading the depth difference for a NaN from the depth sensor on line " << line_number;
		throw std::runtime_error(err.str());
	}
	try {
		obj.library = feature_library(in,line_number);
	} catch(...) {
		std::ostringstream err;
		err << "Error reading the feature library on line " << line_number;
		throw std::runtime_error(err.str());
	}
	std::istringstream n_views_line(nextline(in,line_number));
	int n_views;
	try {
		n_views_line >> n_views;
	} catch(...) {
		std::ostringstream err;
		err << "Error reading number of views on line " << line_number;
		throw std::runtime_error(err.str());
	}
	obj.views.reserve(n_views);
	for(int j=0;j<n_views;j++) {
		obj.views.push_back(view(&obj,in,line_number));
	}

	return in;
}
} //namespace objrec

#endif /* REPRESENTATION_H_ */
