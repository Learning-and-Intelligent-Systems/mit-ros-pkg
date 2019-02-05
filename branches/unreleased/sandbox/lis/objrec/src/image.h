/*
 * image.h
 *
 *  Created on: Dec 29, 2010
 */

#ifndef IMAGE_H_
#define IMAGE_H_

#include "representation.h"
#include <stdexcept>
#include <fstream>

namespace objrec {

class summed_area_table_2d {
private:
	cv::Mat_<unsigned int> summed_area_table;
public:
	template<typename pixel_getter>
	summed_area_table_2d(const pixel_getter& im)
	: summed_area_table(cv::Size(im.width(), im.height())) {
		// horizontal pass
		for(int y=0; y<im.height(); y++) {
			int sum_so_far = 0;
			for(int x=0; x<im.width(); x++) {
				sum_so_far = summed_area_table.at<unsigned int>(y,x) = sum_so_far + (im.get(x,y)?1:0);
			}
		}

		// vertical pass
		for(int x=0; x<im.width(); x++) {
			int sum_so_far = 0;
			for(int y=0; y<im.height(); y++) {
				sum_so_far = summed_area_table.at<unsigned int>(y,x) += sum_so_far;
			}
		}
	}

	inline bool any_in_region(int min_x, int max_x, int min_y, int max_y) const {
		min_x -= 1;
		min_y -= 1;
		if(min_x >= summed_area_table.cols) min_x = summed_area_table.cols-1;
		if(max_x >= summed_area_table.cols) max_x = summed_area_table.cols-1;
		if(min_y >= summed_area_table.rows) min_y = summed_area_table.rows-1;
		if(max_y >= summed_area_table.rows) max_y = summed_area_table.rows-1;

		int top_region, left_region, topleft_region, whole_region;

		//this set of conditionals can be slightly optimized
		if(min_y<0 || max_x<0) top_region = 0;
		else top_region = summed_area_table(min_y,max_x);

		if(max_y<0 || min_x<0) left_region = 0;
		else left_region = summed_area_table(max_y,min_x);

		if(min_y<0 || min_x<0) topleft_region = 0;
		else topleft_region = summed_area_table(min_y,min_x);

		if(max_y<0 || max_x<0) whole_region = 0;
		else whole_region = summed_area_table(max_y,max_x);

		//
		//                ^               ^
		// topleft_region |    top_region |
		//              <-+             <-+
		//
		//                  +-------------+
		//                  |             |
		//                  |
		//                ^ |             ^
		//    left_region | |whole_region |
		//              <-+ +---------- <-+
		//
		//  sum = whole_region - top_region - left_region + topleft_region
		int sum = whole_region - top_region - left_region + topleft_region;

		return sum>0;
	}
};

class summed_area_table_3d {
public: //todo: remove this!
	cv::MatND summed_area_table;
public:
	summed_area_table_3d(const cv::Mat_<float>& depth, float ddepth, float min_depth, float max_depth) {
		int depth_size = ceil((max_depth-min_depth)/ddepth)+1;
		if(depth_size<0) { //if the whole image is NaNs, then max_depth-min_depth will be negative
			depth_size = 0;
		}
		const int sz[3] = {depth_size, depth.rows, depth.cols};//todo: verify that z,y,x is the best order
		summed_area_table.create(3,sz,CV_32S);

		//todo: this can be optimized for depth 3d summed area tables because there is at most one feature along the z axis for each x, y

		// x pass
		for(int z=0; z<sz[0]; z++) {
			float minrange = z*ddepth + min_depth;
			float maxrange = minrange+ddepth;
			for(int y=0; y<sz[1]; y++) {
				int sum_so_far = 0;
				for(int x=0; x<sz[2]; x++) {
					float depth_value = depth(y,x);
					if(minrange<=depth_value && depth_value<maxrange) {
						sum_so_far += 1;
					}
					summed_area_table.at<unsigned int>(z,y,x) = sum_so_far;
				}
			}
		}

		// y pass
		for(int x=0; x<sz[2]; x++) {
			for(int z=0;z<sz[0];z++) {
				int sum_so_far = 0;
				for(int y=0; y<sz[1]; y++) {
					sum_so_far = summed_area_table.at<unsigned int>(z,y,x) += sum_so_far;
				}
			}
		}

		// z pass
		for(int y=0; y<sz[1]; y++) {
			for(int x=0; x<sz[2]; x++) {
				int sum_so_far = 0;
				for(int z=0;z<sz[0];z++) {
					sum_so_far = summed_area_table.at<unsigned int>(z,y,x) += sum_so_far;
				}
			}
		}
	}

	summed_area_table_3d(const summed_area_table_3d& other) {summed_area_table = other.summed_area_table;}
	summed_area_table_3d() {};
	inline bool any_in_region(int min_x, int max_x, int min_y, int max_y, int min_z, int max_z) const {
		min_x -= 1;
		min_y -= 1;
		min_z -= 1;
		if(min_x >= summed_area_table.size[2]) min_x = summed_area_table.size[2]-1;
		if(max_x >= summed_area_table.size[2]) max_x = summed_area_table.size[2]-1;
		if(min_y >= summed_area_table.size[1]) min_y = summed_area_table.size[1]-1;
		if(max_y >= summed_area_table.size[1]) max_y = summed_area_table.size[1]-1;
		if(min_z >= summed_area_table.size[0]) min_z = summed_area_table.size[0]-1;
		if(max_z >= summed_area_table.size[0]) max_z = summed_area_table.size[0]-1;

		int x0y0d0; if(min_x<0 || min_y<0 || min_z<0) x0y0d0 = 0; else x0y0d0 = summed_area_table.at<unsigned int>(min_z,min_y,min_x);
		int x0y0d1; if(min_x<0 || min_y<0 || max_z<0) x0y0d1 = 0; else x0y0d1 = summed_area_table.at<unsigned int>(max_z,min_y,min_x);
		int x0y1d0; if(min_x<0 || max_y<0 || min_z<0) x0y1d0 = 0; else x0y1d0 = summed_area_table.at<unsigned int>(min_z,max_y,min_x);
		int x0y1d1; if(min_x<0 || max_y<0 || max_z<0) x0y1d1 = 0; else x0y1d1 = summed_area_table.at<unsigned int>(max_z,max_y,min_x);
		int x1y0d0; if(max_x<0 || min_y<0 || min_z<0) x1y0d0 = 0; else x1y0d0 = summed_area_table.at<unsigned int>(min_z,min_y,max_x);
		int x1y0d1; if(max_x<0 || min_y<0 || max_z<0) x1y0d1 = 0; else x1y0d1 = summed_area_table.at<unsigned int>(max_z,min_y,max_x);
		int x1y1d0; if(max_x<0 || max_y<0 || min_z<0) x1y1d0 = 0; else x1y1d0 = summed_area_table.at<unsigned int>(min_z,max_y,max_x);
		int x1y1d1; if(max_x<0 || max_y<0 || max_z<0) x1y1d1 = 0; else x1y1d1 = summed_area_table.at<unsigned int>(max_z,max_y,max_x);

		int sum = x1y1d1 - x0y1d1 - x1y0d1 - x1y1d0 + x0y0d1 + x0y1d0 + x1y0d0 - x0y0d0; // Ernesto Tapia, 2009

		return sum>0;
	}
};

struct features_pixel_getter {
	const cv::Mat_<unsigned char> & feature_detections_;
	int min_val;
	features_pixel_getter(const cv::Mat_<unsigned char> & feature_detections_, int min_val) : feature_detections_(feature_detections_), min_val(min_val) {};
	bool get(int x, int y) const { return feature_detections_(y,x)>=min_val; }
	int width() const { return feature_detections_.cols; }
	int height() const { return feature_detections_.rows; }
};

class distance_transform {
private:
	cv::Mat_<float> distance_transform_;
	cv::Mat_<cv::Vec<unsigned short, 2> > indexes;

public:
	template<typename pixel_getter>
	distance_transform(const pixel_getter& im) {
		compute_distance_transform(im,
				//output arguments:
				distance_transform_,indexes);
	}
	inline float get(short x, short y) const {
		if(0<=x && x<distance_transform_.cols && 0<=y && y<distance_transform_.rows) {
			return distance_transform_(y,x);
		} else {
			return std::numeric_limits<float>::infinity();
		}
	}
	float arg(short x, short y,
			//output parameters:
			unsigned short& arg_x, unsigned short& arg_y) const {
		arg_x = indexes(y,x)[0];
		arg_y = indexes(y,x)[1];
		return get(x,y);
	}
};


#ifdef TWO_THRESHOLD_FEATURES
class preprocessed_visual_features {
private:
	summed_area_table_2d summed_area_table_low, summed_area_table_high;
	distance_transform distance_transform_low, distance_transform_high;
public:
	inline float min_dist2_to_nearest_feature_low_lower_bound(float min_x, float max_x, float min_y, float max_y, float receptive_field_radius) const {
		int radius = ceil(receptive_field_radius);
		if(summed_area_table_low.any_in_region(floor(min_x)-radius,ceil(max_x)+radius,floor(min_y)-radius,ceil(max_y)+radius)) {
			return 0;
		} else {
			return std::numeric_limits<float>::infinity();
		}
		//todo: better bound!
	}
	inline float min_dist2_to_nearest_feature_high_lower_bound(float min_x, float max_x, float min_y, float max_y, float receptive_field_radius) const {
		int radius = ceil(receptive_field_radius);
		if(summed_area_table_high.any_in_region(floor(min_x)-radius,ceil(max_x)+radius,floor(min_y)-radius,ceil(max_y)+radius)) {
			return 0;
		} else {
			return std::numeric_limits<float>::infinity();
		}
		//todo: better bound!
	}
	inline float dist2_to_nearest_feature_low(short x, short y) const {
		return distance_transform_low.get(x,y);
	}
	inline float dist2_to_nearest_feature_high(short x, short y) const {
		return distance_transform_high.get(x,y);
	}
	inline float arg_dist2_to_nearest_feature_low(short x, short y,
			//output parameters:
			unsigned short& arg_x, unsigned short& arg_y) const {
		return distance_transform_low.arg(x,y,arg_x,arg_y);
	}
	inline float arg_dist2_to_nearest_feature_high(short x, short y,
			//output parameters:
			unsigned short& arg_x, unsigned short& arg_y) const {
		return distance_transform_high.arg(x,y,arg_x,arg_y);
	}
	preprocessed_visual_features(const cv::Mat_<unsigned char>& feature_map)
	: summed_area_table_low(features_pixel_getter(feature_map,1)),
	  summed_area_table_high(features_pixel_getter(feature_map,2)),
	  distance_transform_low(features_pixel_getter(feature_map,1)),
	  distance_transform_high(features_pixel_getter(feature_map,2)) {}
};

#else
class preprocessed_visual_features {
private:
	summed_area_table_2d summed_area_table;
	distance_transform distance_transform_;
public:
	inline float min_dist2_to_nearest_feature_low_lower_bound(float min_x, float max_x, float min_y, float max_y, float receptive_field_radius) const {
		int radius = ceil(receptive_field_radius);
		if(summed_area_table.any_in_region(floor(min_x)-radius,ceil(max_x)+radius,floor(min_y)-radius,ceil(max_y)+radius)) {
			return 0;
		} else {
			return std::numeric_limits<float>::infinity();
		}
		//todo: better bound!
	}
	inline float dist2_to_nearest_feature_low(short x, short y) const {return distance_transform_.get(x,y);}
	inline float arg_dist2_to_nearest_feature_low(short x, short y,
			//output parameters:
			unsigned short& arg_x, unsigned short& arg_y) const {return distance_transform_.arg(x,y,arg_x,arg_y);}
	preprocessed_visual_features(const cv::Mat_<unsigned char>& feature_map)
	: summed_area_table(features_pixel_getter(feature_map,1)),
	  distance_transform_(features_pixel_getter(feature_map,1)) {}
};
#endif

class preprocessed_depth_features {
private:
public: //todo: remove this
	cv::Mat_<float> depth_;
	float reciprocal_ddepth, min_depth_, max_depth_;
	summed_area_table_3d depth_summed_area_table_;
	summed_area_table_2d NaN_summed_area_table_;
	class NaN_pixel_getter {
		cv::Mat_<float> m;
	public:
		NaN_pixel_getter(const cv::Mat_<float>& m) : m(m) {}
		inline unsigned int height() const {return m.rows;}
		inline unsigned int width() const {return m.cols;}
		inline bool get(unsigned int x, unsigned int y) const { float v = m(y,x); return v!=v; } //"v!=v" checks if v is NaN
	};
public:
//	inline float ddepth() const {return 1.0f/reciprocal_ddepth;}
	inline float min_depth() const {return min_depth_;}
	inline float max_depth() const {return max_depth_;}
	inline const cv::Mat_<float>& get_depth() const {return depth_;}
	preprocessed_depth_features(const cv::Mat_<float> depth, float ddepth)
	: depth_(depth), reciprocal_ddepth(1.0f/ddepth), NaN_summed_area_table_(NaN_pixel_getter(depth)) {
		min_max_mat(depth,min_depth_,max_depth_);
		depth_summed_area_table_ = summed_area_table_3d(depth_,ddepth,min_depth_,max_depth_);
	}
	inline float dist2_to_nearest_feature(float x, float y, float depth, float ddepth_when_missing2) const {
		int x_ = round(x), y_ = round(y);
		if(x_<0 || x_>=depth_.cols || y_<0 || y_>=depth_.rows) {
			return ddepth_when_missing2;
		}
		float d = depth_(y_,x_);
		if(d!=d) {
			return ddepth_when_missing2;
		} else {
			float ddepth = d - depth;
			return ddepth*ddepth;
		}
	}
	inline float min_dist2_to_nearest_feature_lower_bound(
			float min_x, float max_x, float min_y, float max_y, float min_depth, float max_depth, float depth_receptive_field_radius,
			float ddepth_when_missing) const {
		//todo: use a more principled approach: check if max_depth-min_depth is less than the summed area table 3d resolution
		if((max_x-min_x)*(max_y-min_y)>=500) {
			int radius = ceil(depth_receptive_field_radius*reciprocal_ddepth);
			int int_min_x = floor(min_x), int_max_x = ceil(max_x);
			int int_min_y = floor(min_y), int_max_y = ceil(max_y);
			int int_min_depth = floor((min_depth-min_depth_)*reciprocal_ddepth);
			int int_max_depth = ceil( (max_depth-min_depth_)*reciprocal_ddepth);
			if(depth_summed_area_table_.any_in_region(int_min_x,int_max_x,int_min_y,int_max_y,int_min_depth-radius,int_max_depth+radius)) {
				return 0;
			} else {
				if(int_min_x<0 || int_max_x>=depth_.cols || int_min_y<0 || int_max_y>=depth_.rows) { //todo wrap this into any_in_region2
					return ddepth_when_missing;
				}
				if(NaN_summed_area_table_.any_in_region(int_min_x,int_max_x,int_min_y,int_max_y)) {
					return ddepth_when_missing;
				}
				return std::numeric_limits<float>::infinity();
			}
			//todo: better bound!
		} else {
			return min_dist2_to_nearest_feature(min_x,max_x,min_y,max_y,min_depth,max_depth,ddepth_when_missing);
		}
	}
private:
	inline float min_dist2_to_nearest_feature(float min_x, float max_x, float min_y, float max_y, float min_depth, float max_depth,
			float ddepth_when_missing) const {
		int int_min_x = floor(min_x), int_max_x = ceil(max_x);
		int int_min_y = floor(min_y), int_max_y = ceil(max_y);

		float min_dist2 = std::numeric_limits<float>::infinity();
		if(int_min_x<0 || int_max_x>=depth_.cols || int_min_y<0 || int_max_y>=depth_.rows) {
			min_dist2 = ddepth_when_missing;
		}
		int safe_min_x = std::max(int_min_x,0);
		int safe_max_x = std::min(int_max_x,depth_.cols-1);
		int safe_min_y = std::max(int_min_y,0);
		int safe_max_y = std::min(int_max_y,depth_.rows-1);
		for(short y=safe_min_y;y<=safe_max_y;y++) {
			for(short x=safe_min_x;x<=safe_max_x;x++) {
				float d = depth_(y,x);
				if(d!=d) {
					min_dist2 = std::min(min_dist2,ddepth_when_missing);
					continue;
				}

				if(d<max_depth) {
					if(min_depth<=d) {
						return 0;
					} else {
						float dd = d - min_depth;
						min_dist2 = std::min(min_dist2,dd*dd);
					}
				} else {
					float dd = d - max_depth;
					min_dist2 = std::min(min_dist2,dd*dd);
				}
			}
		}
		return min_dist2;
	}
public:
	inline float get(int x, int y) const {
		if(0<=x && x<depth_.cols && 0<=y && y<depth_.rows) return depth_(y,x);
		else return std::numeric_limits<float>::quiet_NaN();
	}
	inline int width() const {return depth_.cols;}
	inline int height() const {return depth_.rows;}
};

struct region {
	point min, max;
	inline region(const point& min, const point& max) : min(min), max(max) {}
	inline region() {}
	inline bool operator==(const region& other) const {return min==other.min && max==other.max;}
	inline point center() const {
		//todo: check assumption that this assumes angles do not wrap
		return point((min.x+max.x)/2.0f,(min.y+max.y)/2.0f,(min.depth+max.depth)/2.0f,
				(min.rx+max.rx)/2.0f,(min.ry+max.ry)/2.0f,(min.rz+max.rz)/2.0f);
	}
	inline bool is_empty() const {
		return min.x>=max.x || min.y>=max.y || min.depth>=max.depth || min.rx>=max.rx || min.ry>=max.ry || min.rz>=max.rz;
	}
	inline bool contains(const point& p) const {
		return
				min.x <= p.x && p.x < max.x &&
				min.y <= p.y && p.y < max.y &&
				min.depth <= p.depth && p.depth < max.depth &&
				min.rx <= p.rx && p.rx < max.rx &&
				min.ry <= p.ry && p.ry < max.ry &&
				min.rz <= p.rz && p.rz < max.rz;
	}
	inline region intersect(const region& r) const {
		return region(
				point(std::max(r.min.x,min.x),std::max(r.min.y,min.y),std::max(r.min.depth,min.depth),
						std::max(r.min.rx,min.rx),std::max(r.min.ry,min.ry),std::max(r.min.rz,min.rz)),
				point(std::min(r.max.x,max.x),std::min(r.max.y,max.y),std::min(r.max.depth,max.depth),
						std::min(r.max.rx,max.rx),std::min(r.max.ry,max.ry),std::min(r.max.rz,max.rz)));

	}
};
std::ostream& operator<<(std::ostream& out, const region& r) {
	return out << r.min << " " << r.max;
}

class image {
private:
	std::vector<preprocessed_visual_features> preprocessed_visual_features_;
public: //todo: remove this
	preprocessed_depth_features preprocessed_depth_features_;
public:
	image(const feature_library& library, const cv::Mat_<cv::Vec3b>& picture, const cv::Mat_<float>& depth,
			const std::vector<cv::Mat_<float> >& precomputed_edges = std::vector<cv::Mat_<float> >(), float ddepth = 0.05f)
				: preprocessed_depth_features_(depth,ddepth) {
		feature_detections feature_maps(&library,picture,depth,precomputed_edges);
		preprocessed_visual_features_.reserve(feature_maps.size());
		for(unsigned int j=0;j<feature_maps.size();j++) {
			preprocessed_visual_features_.push_back(preprocessed_visual_features(feature_maps[j]));

		}
	}

//private: //todo: uncomment this
	inline float max_log_visual_parts_probability(const view& v, const point& p) const {
		const float depth2 = p.depth*p.depth;
		float log_probability = 0;
		const std::vector<std::vector<visual_part> > visual_parts = v.get_visual_parts();
		for(unsigned int j=0;j<visual_parts.size();j++) {
			const std::vector<visual_part>& pj = visual_parts[j];
			const preprocessed_visual_features& pf = preprocessed_visual_features_[j];
			for(std::vector<visual_part>::const_iterator f = pj.begin(); f!=pj.end(); ++f) {
				float dist2_to_nearest_feature_low = pf.dist2_to_nearest_feature_low(
						f->expected_pixel_x(p),
						f->expected_pixel_y(p));
				float dist2_to_nearest_feature_low_weak_perspective = dist2_to_nearest_feature_low*depth2;
				float log_probability_low = f->distribution.shifted_log_probability(dist2_to_nearest_feature_low_weak_perspective);
#ifdef TWO_THRESHOLD_FEATURES
				float dist2_to_nearest_feature_high = pf.dist2_to_nearest_feature_high(
						f->expected_pixel_x(p),
						f->expected_pixel_y(p));
				float dist2_to_nearest_feature_high_weak_perspective = dist2_to_nearest_feature_high*depth2;
				float log_probability_high;
				if(f->distribution.outlier(dist2_to_nearest_feature_high_weak_perspective)) {
					log_probability += log_probability_low;
				} else {
					log_probability_high = f->high_threshold_log_probability_shift+f->distribution.shifted_log_probability(dist2_to_nearest_feature_high_weak_perspective);
					log_probability += std::max(log_probability_low,log_probability_high);
				}

#else
				log_probability += log_probability_low; //if only keeping one threshold features, remove all to the "_low" suffixes from the variable names above
#endif
			}
		}
		return log_probability;
	}
	inline float max_log_depth_parts_probability(const view& v, const point& p) const {
		const float ddepth_when_missing2 = v.get_parent_object()->ddepth_when_missing*v.get_parent_object()->ddepth_when_missing;
		float log_probability = 0;
		std::vector<depth_part> depth_parts = v.get_depth_parts();
		for(std::vector<depth_part>::const_iterator f = depth_parts.begin(); f!=depth_parts.end(); ++f) {
			float dist2_to_nearest_feature = preprocessed_depth_features_.dist2_to_nearest_feature(
					f->pixel_x(p),
					f->pixel_y(p),
					f->expected_depth(p),
					ddepth_when_missing2);
			log_probability += f->distribution.shifted_log_probability(dist2_to_nearest_feature);
		}
		return log_probability;
	}
public:
	inline float max_log_probability(const view& v, const point& p) const {
		return v.get_constant_shift() +
				max_log_visual_parts_probability(v,p)+
				max_log_depth_parts_probability(v,p);
	}

private:
#define BOUND_TERM(dependent_part_var,independent_object_var) {\
	    float min_term = p.dependent_part_var ## independent_object_var * r.min.independent_object_var;\
	    float max_term = p.dependent_part_var ## independent_object_var * r.max.independent_object_var;\
	    if(min_term<max_term) {\
	    	min_ ## dependent_part_var += min_term;\
	    	max_ ## dependent_part_var += max_term;\
	    } else {\
	    	min_ ## dependent_part_var += max_term;\
	    	max_ ## dependent_part_var += min_term;\
	    }\
    }
	inline static void bound_region(const visual_part& p, const region& r,
			//output parameters
			float& min_x, float& max_x, float& min_y, float& max_y) {
		min_x = max_x = p.cx;
		BOUND_TERM(x,rx);
		BOUND_TERM(x,ry);
		BOUND_TERM(x,rz);
		if(min_x>0) {
			min_x /= r.max.depth;//todo: factor out this division
		} else {
			min_x /= r.min.depth;//todo: factor out this division
		}
		min_x += r.min.x;
		if(max_x>0) {
			max_x /= r.min.depth;//todo: factor out this division
		} else {
			max_x /= r.max.depth;//todo: factor out this division
		}
		max_x += r.max.x;

		min_y = max_y = p.cy;
		BOUND_TERM(y,rx);
		BOUND_TERM(y,ry);
		BOUND_TERM(y,rz);
		if(min_y>0) {
			min_y /= r.max.depth;//todo: factor out this division
		} else {
			min_y /= r.min.depth;//todo: factor out this division
		}
		min_y += r.min.y;
		if(max_y>0) {
			max_y /= r.min.depth;//todo: factor out this division
		} else {
			max_y /= r.max.depth;//todo: factor out this division
		}
		max_y += r.max.y;
	}
public: //todo: remove this
	inline float max_log_probability_upper_bound(const visual_part& p, const region& r, int j) const {
		float min_depth2 = r.min.depth*r.min.depth; //todo: factor out this multiplication
		const preprocessed_visual_features& pf = preprocessed_visual_features_[j]; //todo: factor out this reference
		float min_x, max_x, min_y, max_y;
		bound_region(p,r,
				//output parameters:
				min_x,max_x,min_y,max_y);

		float radius = p.distribution.receptive_field_radius()/r.min.depth; //todo: factor out this division
		float nearest_feature_low_dist2_bound =
				pf.min_dist2_to_nearest_feature_low_lower_bound(min_x,max_x,min_y,max_y,radius);
		float dist2_to_nearest_feature_low_bound_weak_perspective = nearest_feature_low_dist2_bound*min_depth2;
		float log_probability_bound_low = p.distribution.shifted_log_probability(dist2_to_nearest_feature_low_bound_weak_perspective);
#ifdef TWO_THRESHOLD_FEATURES
		float nearest_feature_high_dist2_bound =
				pf.min_dist2_to_nearest_feature_high_lower_bound(min_x,max_x,min_y,max_y,radius);
		float dist2_to_nearest_feature_high_bound_weak_perspective = nearest_feature_high_dist2_bound*min_depth2;
		if(p.distribution.outlier(dist2_to_nearest_feature_high_bound_weak_perspective)) {
			return log_probability_bound_low;
		} else {
			float log_probability_bound_high = p.high_threshold_log_probability_shift+p.distribution.shifted_log_probability(dist2_to_nearest_feature_high_bound_weak_perspective);
			return std::max(log_probability_bound_low,log_probability_bound_high);
		}
#else
		return log_probability_bound_low; //if keeping only one threshold, remove the _low suffixes from the above variables
#endif
	}

	inline float max_log_visual_parts_probability_upper_bound(const view& v, const region& r) const {
		float log_probability = 0;
		const std::vector<std::vector<visual_part> > visual_parts = v.get_visual_parts();
		for(unsigned int j=0;j<visual_parts.size();j++) {
			const std::vector<visual_part>& pj = visual_parts[j];
			for(std::vector<visual_part>::const_iterator p = pj.begin(); p!=pj.end(); ++p) {
				log_probability += max_log_probability_upper_bound(*p,r,j);
			}
		}
		return log_probability;
	}
	inline static void bound_region(const depth_part& p, const region& r,
			//output parameters:
			float& min_x, float& max_x, float& min_y, float& max_y, float& min_depth, float& max_depth) {
		if(p.x>=0) {
			min_x = r.min.x+p.x/r.max.depth;//todo: factor out this division
			max_x = r.max.x+p.x/r.min.depth;//todo: factor out this division
		} else {
			min_x = r.min.x+p.x/r.min.depth;//todo: factor out this division
			max_x = r.max.x+p.x/r.max.depth;//todo: factor out this division
		}
		if(p.y>=0) {
			min_y = r.min.y+p.y/r.max.depth;//todo: factor out this division
			max_y = r.max.y+p.y/r.min.depth;//todo: factor out this division
		} else {
			min_y = r.min.y+p.y/r.min.depth;//todo: factor out this division
			max_y = r.max.y+p.y/r.max.depth;//todo: factor out this division
		}

		min_depth = r.min.depth + p.cdepth;
		max_depth = r.max.depth + p.cdepth;
		BOUND_TERM(depth,rx);
		BOUND_TERM(depth,ry);
		BOUND_TERM(depth,rz);

//		if(min_depth>0) {
//			min_depth /= r.max.depth;//todo: factor out this division
//		} else {
//			min_depth /= r.min.depth;//todo: factor out this division
//		}
		// min_depth = r.min.depth +cdepth+depthrx*p.rx+depthry*p.ry+depthrz*p.rz
		//		       p.depth     +cdepth+depthrx*p.rx+depthry*p.ry+depthrz*p.rz

//		if(max_depth>0) {
//			max_depth /= r.min.depth;//todo: factor out this division
//		} else {
//			max_depth /= r.max.depth;//todo: factor out this division
//		}
	}

	inline float max_log_probability_upper_bound(const depth_part& p, const region& r, float ddepth_when_missing) const {
		float ddepth_when_missing2 = ddepth_when_missing*ddepth_when_missing; //todo: factor out this multiplication
		const normal_distribution& n = p.distribution;
		float min_x, max_x, min_y, max_y, min_depth, max_depth;
		bound_region(p,r,
				//output parameters:
				min_x,max_x,min_y,max_y,min_depth,max_depth);
		float nearest_feature_dist2_bound =
				preprocessed_depth_features_.min_dist2_to_nearest_feature_lower_bound(min_x,max_x,min_y,max_y,min_depth,max_depth,
						n.receptive_field_radius(),ddepth_when_missing2);
		return n.shifted_log_probability(nearest_feature_dist2_bound);
	}
	inline float max_log_depth_parts_probability_upper_bound(const view& v, const region& r) const {
		float ddepth_when_missing = v.get_parent_object()->ddepth_when_missing;
		float log_probability = 0;
		const std::vector<depth_part> depth_parts = v.get_depth_parts();
		for(std::vector<depth_part>::const_iterator p = depth_parts.begin(); p!=depth_parts.end(); ++p) {
			log_probability += max_log_probability_upper_bound(*p,r,ddepth_when_missing);
		}
		return log_probability;
	}

	inline std::string debug_depth_parts_probability(const view& v, const point& leaf, const region& parent) const {
		const float ddepth_when_missing2 = v.get_parent_object()->ddepth_when_missing*v.get_parent_object()->ddepth_when_missing;
		float log_probability = 0;
		std::vector<depth_part> depth_parts = v.get_depth_parts();
		float ddepth_when_missing = v.get_parent_object()->ddepth_when_missing;
		int count_violations = 0;
		std::ostringstream out;
		for(std::vector<depth_part>::const_iterator f = depth_parts.begin(); f!=depth_parts.end(); ++f) {
			float dist2_to_nearest_feature = preprocessed_depth_features_.dist2_to_nearest_feature(
					f->pixel_x(leaf),
					f->pixel_y(leaf),
					f->expected_depth(leaf),
					ddepth_when_missing2);
			float leaf_probability = f->distribution.shifted_log_probability(dist2_to_nearest_feature);

			const normal_distribution& n = f->distribution;
			float min_x, max_x, min_y, max_y, min_depth, max_depth;
			bound_region(*f,parent,
					//output parameters:
					min_x,max_x,min_y,max_y,min_depth,max_depth);
			float nearest_feature_dist2_bound =
					preprocessed_depth_features_.min_dist2_to_nearest_feature_lower_bound(min_x,max_x,min_y,max_y,min_depth,max_depth,
							n.receptive_field_radius(),ddepth_when_missing2);

			float parent_bound = max_log_probability_upper_bound(*f,parent,ddepth_when_missing);
			if(leaf_probability>parent_bound) {
				count_violations++;
				out << std::endl;
				out << "leaf probability: " << leaf_probability << ", for point " << leaf << std::endl;
				out << "  > parent bound: " << parent_bound << ", for region " << parent << std::endl;
				out << "leaf_probability-parent_bound: " << leaf_probability-parent_bound << std::endl;
				out << std::endl;
				out << "leaf dist2_to_nearest_feature:     " << dist2_to_nearest_feature << std::endl;
				out << "parent nearest_feature_dist2_bound: " << nearest_feature_dist2_bound << std::endl;
				out << "dist2_to_nearest_feature-nearest_feature_dist2_bound: " << dist2_to_nearest_feature-nearest_feature_dist2_bound << std::endl;
				out << std::endl;
				out << "parent bound_region: " << min_x << " " << max_x << "  " << min_y << " " << max_y << "  " << min_depth << " " << max_depth << std::endl;
				out << "leaf point: " << f->pixel_x(leaf) << " " << f->pixel_y(leaf) << " " << f->expected_depth(leaf) << std::endl;
				out << "f->expected_depth(leaf)-min_depth: " << f->expected_depth(leaf)-min_depth << std::endl;
			}
		}
		out << count_violations << " violations found in " << depth_parts.size() << " depth parts." << std::endl;
		return out.str();
	}
public:
	inline float max_log_probability_upper_bound(const view& v, const region& r) const {
		return v.get_constant_shift() +
				max_log_visual_parts_probability_upper_bound(v,r)+
				max_log_depth_parts_probability_upper_bound(v,r);
	}

	inline int width() const {return preprocessed_depth_features_.width();}
	inline int height() const {return preprocessed_depth_features_.height();}
	void draw(const view& v, const point& p,
			//output argument:
			cv::Mat_<cv::Vec3b>& drawing_image) const {
		int max_feature_radius = 3.f/p.depth;
		float depth2 = p.depth*p.depth;
		cv::Scalar black(0,0,0);
		cv::Scalar white(255,255,255);
		cv::Scalar red(0,0,255);
		const std::vector<edge_direction>& edge_directions = v.get_parent_object()->library.get_edges().get_edge_directions();
		const std::vector<std::vector<visual_part> >& visual_parts = v.get_visual_parts();
		for(int j=0;j<visual_parts.size();j++) {
			for(std::vector<visual_part>::const_iterator k=visual_parts[j].begin();k!=visual_parts[j].end();k++) {
				const normal_distribution& d = k->distribution;
				int x = k->expected_pixel_x(p);
				int y = k->expected_pixel_y(p);
				if(j<edge_directions.size()) {
					edge_directions[j].draw(drawing_image,x,y,max_feature_radius,black);
				} else {
					cv::circle(drawing_image,cv::Point(x,y),2,black,-1);
				}
				unsigned short fx, fy;
				float dist2 = preprocessed_visual_features_[j].arg_dist2_to_nearest_feature_low(x,y,fx,fy);
#ifdef TWO_THRESHOLD_FEATURES
				float dist2_high = preprocessed_visual_features_[j].arg_dist2_to_nearest_feature_high(x,y,fx,fy);
				dist2 = std::min(dist2,dist2_high);
#endif
				bool outlier = d.outlier(dist2*depth2);
				cv::Scalar color;
				if(outlier) {
					color = red; //red means outlier
				} else {
					color = cv::Scalar(0,255,0);
				}
				if(j<edge_directions.size()) {
					edge_directions[j].draw(drawing_image,fx,fy,max_feature_radius,color);
				} else {
					cv::circle(drawing_image,cv::Point(x,y),2,color,-1);
				}
			}
		}

		const std::vector<depth_part>& depth_parts = v.get_depth_parts();
		float min_depth = std::numeric_limits<float>::infinity();
		float max_depth = -std::numeric_limits<float>::infinity();
		for(std::vector<depth_part>::const_iterator j=depth_parts.begin();j!=depth_parts.end();j++) {
			min_depth = std::min(min_depth,j->expected_depth(p));
			max_depth = std::max(max_depth,j->expected_depth(p));
		}

		float ddepth_when_missing = v.get_parent_object()->ddepth_when_missing;
		for(std::vector<depth_part>::const_iterator j=depth_parts.begin();j!=depth_parts.end();j++) {
			int x = j->pixel_x(p);
			int y = j->pixel_y(p);
			float depth = preprocessed_depth_features_.get(x,y);
			float d;
			if(depth!=depth) {
				d = ddepth_when_missing;
			} else {
				d = depth - j->expected_depth(p);
			}
			float dist2 = d*d;

			bool outlier = j->distribution.outlier(dist2);
			cv::Scalar color;
			if(depth>j->expected_depth(p)) {
				color = cv::Scalar(255,0,0); //blue means farther than expected
			} else {
				color = cv::Scalar(0,255,0); //green means nearer than expected
			}
			if(outlier) color = white; //white means outlier
			if(depth!=depth) color = red; //red means missing depth information

			cv::circle(drawing_image,cv::Point(x,y),max_feature_radius,color,-1);
		}
	}

};

} //namespace objrec

#endif /* IMAGE_H_ */
