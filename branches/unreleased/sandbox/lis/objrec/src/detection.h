/*
 * detection.h
 *
 *  Created on: Mar 28, 2012
 */

#ifndef DETECTION_H_
#define DETECTION_H_

#include "image.h"
//#define DEBUG_OBJREC
#include <queue>
#include <iostream>
#include <set>

namespace objrec {

const float default_min_ddepth = .05f; //.002f

//The interface for a search_space_constraint is:
//
//class search_space_constraint {
//public:
//	region intersect(const region& r) const;
//  bool is_satisfied(const point& p) const;
//};
//
//but we do not use class inheritance to avoid the overhead of virtual function calls.
//Instead we use a template parameter.
//
//intersect() intersects a region with the search_space_constraint.
//is_satisfied() determines if a point satisfies the search_space_constraint.
//
//for example:

class no_constraint {
public:
	inline region intersect(const region& r) const {return r;}
	inline bool is_satisfied(const point& p) const {return true;}
};


//todo: fix this so it pushes the leaves onto the queue first
template <class search_space_constraint>
inline float max_log_probability(const view& v, const image& im, const region& r, const search_space_constraint& constraint,
		//output parameter:
		point& best_location,
		const point& dp=point(1,1,.002,1,1,1)) {
	float max_log_probability = -std::numeric_limits<float>::infinity();
//	bool any_satisfy_constraint = false;
	point p;
	for(p.depth=r.min.depth;p.depth<=r.max.depth;p.depth+=dp.depth) {
		for(p.y=r.min.y;p.y<=r.max.y;p.y+=dp.y) {
			for(p.x=r.min.x;p.x<=r.max.x;p.x+=dp.x) {
				for(p.rx=r.min.rx;p.rx<=r.max.rx;p.rx+=dp.rx) {
					for(p.ry=r.min.ry;p.ry<=r.max.ry;p.ry+=dp.ry) {
						for(p.rz=r.min.rz;p.rz<=r.max.rz;p.rz+=dp.rz) {
							float log_probability = im.max_log_probability(v,p);
							if(constraint.is_satisfied(p)) {
//								any_satisfy_constraint = true;
								if(log_probability>max_log_probability) {
									max_log_probability = log_probability;
									best_location = p;
								}
							}
						}
					}
				}
			}
		}
	}
//	if(!any_satisfy_constraint) {
//		best_location = constraint.satisfying_point(r);
//		if(best_location.x!=best_location.x) {
//			std::ostringstream err;
//			err << "could not find a point in the region:" << std::endl;
//			err << r.min << std::endl;
//			err << r.max << std::endl;
//			err << "that satisfies the constraint" << std::endl;
//			float min_rx, max_rx;
//			constraint.rx_range(r,min_rx,max_rx);
//			float min_ry, max_ry;
//			constraint.ry_range(r,min_ry,max_ry);
//			region intersected = constraint.intersect(r);
//			err << "intersected: " << std::endl;
//			err << intersected.min << std::endl;
//			err << intersected.max << std::endl;
//			err << std::endl;
//			err << "original    rx range: " << r.min.rx << "\tto " << r.max.rx << std::endl;
//			err << "constraint  rx range: " << min_rx-2.0 << "\tto " << max_rx+2.0 << std::endl;
//			err << "intersected rx range: " << intersected.min.rx << "\tto " << intersected.max.rx << std::endl;
//			err << std::endl;
//			err << "original    ry range: " << r.min.ry << "\tto " << r.max.ry << std::endl;
//			err << "constraint  ry range: " << min_ry-2.0 << "\tto " << max_ry+2.0 << std::endl;
//			err << "intersected ry range: " << intersected.min.ry << "\tto " << intersected.max.ry << std::endl;
//
//			throw(std::runtime_error(err.str()));
//		}
//		return im.max_log_probability(v,p);
//		point mid = r.center();
//		if(constraint.is_satisfied(mid)) {
//			best_location = mid;
//			return im.max_log_probability(v,mid);
//		} else {
//			best_location.x = std::numeric_limits<float>::quiet_NaN();
//			return std::numeric_limits<float>::infinity();
//		}
//	}
//	if(max_log_probability==-std::numeric_limits<float>::infinity()) {
//		std::cout << "r: " << r << std::endl;
//		std::cout << "p: " << p << std::endl;
//		throw std::runtime_error("-inf found (max)");
//	}
	return max_log_probability;
}


const float min_dx = 2.0f;
const float min_dy = 2.0f;
const float min_ddepth = 0.02f;
const float min_drx = 4.0f;
const float min_dry = 4.0f;
const float min_drz = 4.0f;

class hypothesis {
private:
	const view* v;
	region r;
	float b;
public:
	inline hypothesis() : b(-std::numeric_limits<float>::infinity()) {}
	inline hypothesis(const view* v, const image& im, const region& r) : v(v), r(r), b(im.max_log_probability_upper_bound(*v,r)) {}
	inline hypothesis(const view* v, const image& im, const point& p) : v(v), r(p,p), b(im.max_log_probability(*v,p)) {}
	inline hypothesis(const hypothesis& other) : v(other.v), r(other.r), b(other.b) {}
	struct compare : public std::binary_function<const hypothesis&,const hypothesis &,bool> {
		bool operator()(const hypothesis& h1, const hypothesis& h2) {return h1.b < h2.b;}
	};

	typedef std::priority_queue<hypothesis,std::vector<hypothesis>,compare> priority_queue;
private:

	static inline bool is_leaf(const region& r, const point& min_resolution = point(min_dx,min_dy,min_ddepth,min_drx,min_dry,min_drz)) {
		return
				r.max.x-r.min.x<min_resolution.x &&
				r.max.y-r.min.y<min_resolution.y &&
				r.max.depth-r.min.depth<min_resolution.depth &&
				r.max.rx-r.min.rx<min_resolution.rx &&
				r.max.ry-r.min.ry<min_resolution.ry &&
				r.max.rz-r.min.rz<min_resolution.rz;
	}
public:
	inline bool is_leaf(const point& min_d = point(min_dx,min_dy,min_ddepth,min_drx,min_dry,min_drz)) const { return is_leaf(r,min_d); }
private:
	template <class search_space_constraint>
	inline hypothesis(const hypothesis* parent, const image& im, const region& r,
			const point& min_resolution, const search_space_constraint& constraint) : v(parent->v) {
		if(is_leaf(r, min_resolution)) {
			point p;
			b = max_log_probability(*v,im,r,constraint,
					//output parameter:
					p);
			this->r.min = this->r.max = p;
		} else {
			this->r = r;
			b = im.max_log_probability_upper_bound(*v,r);
		}
	}
	template<class search_space_constraint>
	inline bool child(priority_queue& q, const image& im, float lower_bound, const point& min_resolution, const search_space_constraint& constraint,
			//output parameter:
			const region& br) const {
		region intersected = constraint.intersect(br);
//		if(
//				intersected.min.rx>360 || intersected.min.ry>360 || intersected.min.rz>360 ||
//				intersected.max.rx>360 || intersected.max.ry>360 || intersected.max.rz>360
//				) { //todo: remove this test!!
//			std::ostringstream err;
//			err << "parent       region:" << std::endl << r.min << std::endl << r.max << std::endl;
//			err << "before intersection:" << std::endl << br.min << std::endl << br.max << std::endl;
//			err << "after  intersection:" << std::endl << intersected.min << std::endl << intersected.max << std::endl;
//			throw std::runtime_error(err.str());
//		}
		if(intersected.is_empty()) {
			return false;
		}
		hypothesis h(this, im, intersected, min_resolution, constraint);
#define DEBUG_OBJREC
#ifdef DEBUG_OBJREC
		if(h.get_bound()>get_bound()) {
			if(is_leaf(br,min_resolution)) {
				//				v.compare_max_log_probability(im,this->get_region(),h.get_region().center());
				std::ostringstream err;
				err << "Hypothesis leaf (" << h.get_region().center() << ") has value " << h.b <<
						" which is greater than its parent: (" << this->get_region() << ") bound: " <<
						b << " abs(diff): " << fabs(b-h.b) << " image is " << im.width() << "x" << im.height() << " pixels" << std::endl;
				err << "Child leaf visual parts: " << im.max_log_visual_parts_probability(*v,h.get_region().center()) << std::endl;
				err << "Parent     visual parts: " << im.max_log_visual_parts_probability_upper_bound(*v,this->get_region()) << std::endl;
				err << "Child leaf  depth parts: " << im.max_log_depth_parts_probability(*v,h.get_region().center()) << std::endl;
				err << "Parent      depth parts: " << im.max_log_depth_parts_probability_upper_bound(*v,this->get_region()) << std::endl;
				err << "br: " << br << std::endl;
				err << "intersected: " << intersected << std::endl;
				point p;
				float prob = max_log_probability(*v,im,intersected,constraint,
									//output parameter:
									p);
				err << "prob: " << prob << std::endl;
				err << "p: " << p << std::endl;
				err << im.debug_depth_parts_probability(*v, h.get_region().center(), this->get_region()) << std::endl;
				throw std::runtime_error(err.str());
			} else {
				//				v.compare_max_log_probability_upper_bound(im,this->get_region(),h.get_region());

				std::ostringstream err;
				err << "Hypothesis non-leaf (" << h.get_region() << ") has value " << h.b <<
						" which is greater than its parent: (" << this->get_region() << ") bound: " <<
						b << " abs(diff): " << fabs(b-h.b) << " image is " << im.width() << "x" << im.height() << " pixels" << std::endl;
				err << "Child  visual parts: " << im.max_log_visual_parts_probability_upper_bound(*v,h.get_region()) << std::endl;
				err << "Parent visual parts: " << im.max_log_visual_parts_probability_upper_bound(*v,this->get_region()) << std::endl;
				err << "Child   depth parts: " << im.max_log_depth_parts_probability_upper_bound(*v,h.get_region()) << std::endl;
				err << "Parent  depth parts: " << im.max_log_depth_parts_probability_upper_bound(*v,this->get_region()) << std::endl;

				throw std::runtime_error(err.str());
			}
		}
#endif
		float bound = h.get_bound();
		if(bound>=lower_bound && bound!=-std::numeric_limits<float>::infinity()) {
			q.push(h);
			return true;
		} else {
			return false;
		}
	}
#define BRANCH(current_var, next_level) \
	template<class search_space_constraint> \
	inline bool branch_ ## current_var(priority_queue& q, const image& im, float lower_bound, const point& min_resolution, const search_space_constraint& constraint,\
			region& br) const { \
			float d = r.max.current_var-r.min.current_var; \
			if(d<min_resolution.current_var) { \
				br.min.current_var = r.min.current_var; \
				br.max.current_var = r.max.current_var;	\
				return next_level<search_space_constraint>(q,im,lower_bound,min_resolution,constraint,br); \
			} else { \
				float mid = r.min.current_var+d*0.5f; \
				br.min.current_var = r.min.current_var; \
				br.max.current_var = mid; \
				next_level<search_space_constraint>(q,im,lower_bound,min_resolution,constraint,br); \
				br.min.current_var = mid; \
				br.max.current_var = r.max.current_var; \
				next_level<search_space_constraint>(q,im,lower_bound,min_resolution,constraint,br); \
				return false; \
			} \
	}

	BRANCH(rx,branch_ry)
	BRANCH(ry,branch_rz)
	BRANCH(rz,branch_x)
	BRANCH(x,branch_y)
	BRANCH(y,branch_depth)
	BRANCH(depth,child)
public:
	///returns true if this hypothesis is a leaf
	template<class search_space_constraint>
	inline bool branch(priority_queue& q,const image& im,float lower_bound,
			const search_space_constraint& constraint=no_constraint(),
			const point& min_resolution = point(min_dx,min_dy,min_ddepth,min_drx,min_dry,min_drz)) const {
		region br;
		return branch_rx<search_space_constraint>(q,im,lower_bound,min_resolution,constraint,br);
	}

	inline float get_bound() const {return b;}
	inline const region& get_region() const {return r;}
	inline const view* get_view() const {return v;}

	inline bool operator==(const hypothesis& rhs) const {
		return
				b==rhs.b && //todo do we just need this line in the equality check??
				r==r;
	}
};

template <class search_space_constraint>
class detector {
	hypothesis::priority_queue q;
	const object* obj;
	const image* im;
	const point min_resolution;
	const search_space_constraint constraint;
public:
	detector(const object* obj, const image* im,
			search_space_constraint constraint=no_constraint(),
			const point& min_resolution = point(min_dx,min_dy,min_ddepth,min_drx,min_dry,min_drz)) : obj(obj),
			im(im), min_resolution(min_resolution), constraint(constraint) {}
	inline void add_hypothesis(const hypothesis& h) { q.push(h); }
	inline int n_remaining_hypotheses() const { return q.size(); }
	inline const hypothesis& best_hypothesis() const { return q.top(); }
	inline void remove_best_hypothesis() { q.pop(); }
	///returns true if h is a leaf
	inline bool add_child_hypotheses(const hypothesis& h, float lower_bound=-std::numeric_limits<float>::infinity()) {
		return h.branch<search_space_constraint>(q,*im,lower_bound,constraint,min_resolution);
	}
};

inline region entire_image(const view* v, const image* im) {
	const viewpoint& vp = v->get_viewpoint();
	return region(
			point(0,0,vp.min_depth,vp.min_rx,vp.min_ry,vp.min_rz),
			point(im->width(),im->height(),vp.max_depth,vp.max_rx,vp.max_ry,vp.max_rz));
}

//class hypothesis_iterator : public std::iterator<std::input_iterator_tag, hypothesis> {
//	hypothesis::priority_queue queue;
//	const view* v;
//	const image* im;
//	const point min_d;
//	float lower_bound;
//	bool verbose;
//	hypothesis last_leaf;
//	hypothesis_iterator() : im(0) {}
//public:
//	hypothesis_iterator(const view* v, const image* im,
//			float lower_bound=-std::numeric_limits<float>::infinity(),
//			bool verbose=false, const point& min_d = point(min_dx,min_dy,min_ddepth,min_drx,min_dry,min_drz)) :
//		v(v), im(im), min_d(min_d), lower_bound(lower_bound), verbose(verbose) {
//		const viewpoint& vp = v->get_viewpoint();
//		region r(
//				point(0,0,vp.min_depth,vp.min_rx,vp.min_ry,vp.min_rz),
//				point(im->width(),im->height(),vp.max_depth,vp.max_rx,vp.max_ry,vp.max_rz));
//		queue.push(hypothesis(v, *im, r));
//	}
//	hypothesis_iterator(const view* v, const image* im, const region& search_region,
//			float lower_bound=-std::numeric_limits<float>::infinity(),
//			bool verbose=false, const point& min_d = point(min_dx,min_dy,min_ddepth,min_drx,min_dry,min_drz)) :
//		v(v), im(im), min_d(min_d), lower_bound(lower_bound), verbose(verbose) {
//		const viewpoint& vp = v->get_viewpoint();
//		region r(
//				point(0,0,vp.min_depth,vp.min_rx,vp.min_ry,vp.min_rz),
//				point(im->width(),im->height(),vp.max_depth,vp.max_rx,vp.max_ry,vp.max_rz));
//		queue.push(hypothesis(v, *im, search_region.intersect(r)));
//	}
//	hypothesis_iterator(const hypothesis_iterator& other) : queue(other.queue), v(other.v), im(other.im), verbose(other.verbose) {}
//	inline hypothesis_iterator& operator++() {
//		long pop_count = 0;
//		while(!queue.empty()) {
//			hypothesis h = queue.top();
//			queue.pop();
//			pop_count++;
//			if(h.is_leaf(min_d)) { //todo: each node is checked twice to see if it is a leaf... eliminate this redundancy
//				if(verbose) {
//					std::cerr << "Popped " << pop_count << " hypotheses. Queue size: " << queue.size()
//								<< ". Final hypothesis: ddepth: " << h.get_region().max.depth-h.get_region().min.depth
//								<< " point: " << h.get_region().center() << " region: " << h.get_region() << " bound: " << h.get_bound() << " lower_bound: " << lower_bound << std::endl;
//					//				std::cerr << "evaluated logP again: " << v->max_log_probability(*im,h.get_region().center()) << std::endl;
//				}
//				last_leaf = h;
//				return *this;
//			} else {
//				h.branch<no_constraint>(queue, *im, lower_bound, min_d);
//			}
//			if(verbose && pop_count%100000==0 && pop_count!=0) {
//				std::cerr << "Popped " << pop_count << " hypotheses. Queue size: " << queue.size()
//						<< ". Current hypothesis: ddepth: " << h.get_region().max.depth - h.get_region().min.depth
//						<< " region: " << h.get_region() << " bound: " << h.get_bound() << std::endl;
//			}
//		}
//		last_leaf = hypothesis();
//		return *this;
//	}
//	inline void operator++(int) {operator++();}
//	inline bool operator==(const hypothesis_iterator& rhs) const {
//		return (!queue.empty() && !rhs.queue.empty() && queue.top()==rhs.queue.top()) ||
//				(queue.empty() && rhs.queue.empty());
//	}
//	inline bool operator!=(const hypothesis_iterator& rhs) const {return !operator==(rhs);}
//	inline const hypothesis& operator*() const {return last_leaf;}
//	inline const hypothesis* operator->() const {return &last_leaf;}
//	static hypothesis_iterator end() {return hypothesis_iterator();}
//	bool has_localization() const {region r = last_leaf.get_region(); return r.min.x==r.min.x; } //if r.min.x is not a NaN
//	unsigned int size() const {return queue.size();}
//};

void get_bounding_box(const view& v, const point& p,
		//output parameters:
		float& x0, float& x1, float& y0, float& y1) {
	x0 = y0 = std::numeric_limits<float>::infinity();
	x1 = y1 = -std::numeric_limits<float>::infinity();
	const std::vector<std::vector<visual_part> >& visual_parts = v.get_visual_parts();
	for(std::vector<std::vector<visual_part> >::const_iterator j=visual_parts.begin();j!=visual_parts.end();j++) {
		for(std::vector<visual_part>::const_iterator k=j->begin();k!=j->end();k++) {
			x0 = std::min(k->expected_pixel_x(p),x0); x1 = std::max(k->expected_pixel_x(p),x1);
			y0 = std::min(k->expected_pixel_y(p),y0); y1 = std::max(k->expected_pixel_y(p),y1);
		}
	}
	const std::vector<depth_part>& depth_parts = v.get_depth_parts();
	for(std::vector<depth_part>::const_iterator j=depth_parts.begin();j!=depth_parts.end();j++) {
		x0 = std::min(j->pixel_x(p),x0); x1 = std::max(j->pixel_x(p),x1);
		y0 = std::min(j->pixel_y(p),y0); y1 = std::max(j->pixel_y(p),y1);
	}
}

///Stores a maximum-probability localization of a view in an image.
struct localization {
	point location;
	float log_probability;
	short x0, x1, y0, y1; //bounding box
	const view* v;
	static bool comparefn(const localization& l1, const localization& l2) {return l1.log_probability > l2.log_probability;}
	struct compare : public std::binary_function<const localization&,const localization&,bool> {
		bool operator()(const localization& l1, const localization& l2) {return l1.log_probability > l2.log_probability;}
	};
	typedef std::set<localization,compare> set;
	localization(const view* v, const point& p, float log_probability) : location(p), log_probability(log_probability), v(v) {
		float x0f, x1f, y0f, y1f;
		get_bounding_box(*v,p,
				//output parameters:
				x0f,x1f,y0f,y1f);
		x0 = floor(x0f); x1 = ceil(x1f);
		y0 = floor(y0f); y1 = ceil(y1f);
	}
	localization() : v(0) {}
};
std::ostream& operator<<(std::ostream& out, const localization& loc) {
	return out << loc.log_probability << " " << loc.x0 << " " << loc.x1 << " " << loc.y0 << " " << loc.y1 << " " <<
			loc.location << " " << loc.v->get_viewpoint();
}

inline bool bounding_boxes_overlap(const localization& loc,
		short x0, short x1, short y0, short y1, float minimum_overlap_percent) {
	int dx = (int)std::min(loc.x1,x1) - (int)std::max(loc.x0,x0) + 1;
	int dy = (int)std::min(loc.y1,y1) - (int)std::max(loc.y0,y0) + 1;
	bool overlap = dx>0 && dy>0;
	int intersection;
	if(overlap) {
		intersection = dx*dy;
	} else {
		intersection = 0;
	}

	int loc_area = ((int)loc.x1-(int)loc.x0+1)*((int)loc.y1-(int)loc.y0+1);
	int bbox_area = ((int)x1-(int)x0+1)*((int)y1-(int)y0+1);
	int union_ = loc_area + bbox_area - intersection;
	//the metric used by the PASCAL Visual Object Challenge:

	float overlap_percent = (float)intersection/(float)union_;
	bool enough_overlap =  overlap_percent >= minimum_overlap_percent;
	return enough_overlap;
}

inline bool is_non_maximum(const std::vector<localization>& localizations, const localization& loc, float overlap_percent=0.5) {
	for(std::vector<localization>::const_iterator l=localizations.begin();l!=localizations.end();l++) {
		if(bounding_boxes_overlap(*l,loc.x0,loc.x1,loc.y0,loc.y1,overlap_percent)) {
			return true;
		}
	}
	return false;
}

//The interface for a constraint is:
//
//class constraint {
//  void order_views(const std::vector<view>& views, std::vector<int>& sorted_indexes) const;
//	region bounding_region(const view& v) const;
//  bool is_satisfied(const localization& loc) const;
//	cv::Mat_<cv::Vec3b> draw(const view& v) const;
//};
//
//but we do not use class inheritance to avoid the overhead of virtual function calls.
//Instead we use a template parameter.
//
//order_views() returns indexes of views so that they are in the best order for searching
//bounding_region() gives an overestimate of the region to search.
//is_satisfied() determines if a localization actually satisfies the constraint.
//draw() draws a graphic for the current progress.

//struct no_constraint_old {
//	inline void order_views(const std::vector<view>& views,
//				//output parameter:
//				std::vector<int>& sorted_indexes) const {
//		sorted_indexes.clear();
//		sorted_indexes.reserve(views.size());
//		for(int j=0;j<(int)views.size();j++) sorted_indexes.push_back(j);
//	}
//	inline region bounding_region(const view& v) const {
//		region r;
//		r.min.x = r.min.y = -std::numeric_limits<float>::infinity();
//		r.max.x = r.max.y = std::numeric_limits<float>::infinity();
//		const viewpoint& vp = v.get_viewpoint();
//		r.min.depth = vp.min_depth;
//		r.max.depth = vp.max_depth;
//		r.min.rx = vp.min_rx; r.max.rx = vp.max_rx;
//		r.min.ry = vp.min_ry; r.max.ry = vp.max_ry;
//		r.min.rz = vp.min_rz; r.max.rz = vp.max_rz;
//		return r;
//	}
//	inline bool is_satisfied(const localization& loc) const { return true; }
//	cv::Mat_<cv::Vec3b> draw(const view& v) const {
//		cv::Mat_<cv::Vec3b> empty(cv::Size(640,480));
//		set_to(empty,cv::Vec3b(0,0,0));
//		return empty;
//	}
//};
//
//template <class search_space_constraint>
//localization best_localization(const view& v, const image& im, float lower_bound,
//		//parameters with defaults:
//		search_space_constraint constraint=no_constraint_old(), bool print_progress=false) {
//    localization best_localization;
//    best_localization.log_probability = -std::numeric_limits<float>::infinity();
//	region r = constraint.bounding_region(v);
//	if(r.min.x!=std::numeric_limits<float>::infinity()) {
//		hypothesis_iterator h(&v,&im,r,lower_bound, print_progress);
//		h++;
//		while(h!=h.end() && h.has_localization() /*&& h->getBound()>best_localization.logP*/) {
//			localization loc(&v,h->get_region().min,h->get_bound());
//			if(constraint.is_satisfied(loc)) { return loc; }
//			h++;
//		}
//	}
//	return best_localization;
//}
//
//inline bool bounding_boxes_overlap(const localization& loc,
//		short x0, short x1, short y0, short y1, float minimum_overlap_percent) {
//	int dx = (int)std::min(loc.x1,x1) - (int)std::max(loc.x0,x0) + 1;
//	int dy = (int)std::min(loc.y1,y1) - (int)std::max(loc.y0,y0) + 1;
//	bool overlap = dx>0 && dy>0;
//	int intersection;
//	if(overlap) {
//		intersection = dx*dy;
//	} else {
//		intersection = 0;
//	}
//
//	int loc_area = ((int)loc.x1-(int)loc.x0+1)*((int)loc.y1-(int)loc.y0+1);
//	int bbox_area = ((int)x1-(int)x0+1)*((int)y1-(int)y0+1);
//	int union_ = loc_area + bbox_area - intersection;
//	//the metric used by the PASCAL Visual Object Challenge:
//
//	float overlap_percent = (float)intersection/(float)union_;
//	bool enough_overlap =  overlap_percent >= minimum_overlap_percent;
//	return enough_overlap;
//}
//
//inline bool is_non_maximum(const std::vector<localization>& localizations, const localization& loc, float overlap_percent=0.5) {
//	for(std::vector<localization>::const_iterator l=localizations.begin();l!=localizations.end();l++) {
//		if(bounding_boxes_overlap(*l,loc.x0,loc.x1,loc.y0,loc.y1,overlap_percent)) {
//			return true;
//		}
//	}
//	return false;
//}
//
//template <class search_space_constraint>
//void localizations(const view& v, const image& im, float lower_bound,
//		//output parameters:
//		std::vector<localization>& all_localizations,
//		//parameters with defaults:
//		search_space_constraint constraint=no_constraint_old(), float nms_overlap_percent=0.5,
//		bool print_progress=false) {
//	region r = constraint.bounding_region(v);
//	if(r.min.x!=std::numeric_limits<float>::infinity()) {
//		hypothesis_iterator h(&v,&im,r,lower_bound, print_progress);
//		h++;
//		while(h!=h.end() && h.has_localization() /*&& h->getBound()>best_localization.logP*/) {
//			localization loc(&v,h->get_region().min,h->get_bound());
//			if(constraint.is_satisfied(loc)) {
//				//non maximum suppression:
//				if(!is_non_maximum(all_localizations,loc,nms_overlap_percent)) {
//					all_localizations.push_back(loc);
//				}
//			}
//			h++;
//		}
//	}
//}
//
//void progress(long long start, int j, const object& obj, float lower_bound) {
//        struct timeval tv;
//        gettimeofday(&tv, 0);
//        long long now = ((long long)tv.tv_sec)*1000000 + tv.tv_usec;
//        double elapsed = (double)(now-start)/1000000.0;
//        double remaining = elapsed/(double)(j+1)*(double)(obj.views.size()-1-j);
//
//        std::cerr << "completed view " << j+1 << " of " << obj.views.size() // " for object '" << obj.name << "'"
//                        <<". (" << round(100.0f*(float)(j+1)/(float)obj.views.size()) << "%) ";
//        std::cerr << time_string(elapsed) << " elapsed, " << time_string(remaining) << " left.";
//        std::cerr << " Current lower bound: " << lower_bound << ".             \r" << std::flush;
//}
//
//template <class search_space_constraint>
//inline void localizations(const object& obj, const image& im, float lower_bound,
//		//output parameters:
//		std::vector<localization>& all_localizations,
//		//parameters with defaults:
//		bool only_top_detection=false,
//		bool print_progress = false, bool visualize_search = false,
//		int start_view_index=0, int stop_view_index=-1, search_space_constraint constraint=no_constraint_old(),
//		float nms_overlap_percent=0.5) {
//	if(print_progress) std::cout << std::endl;
//	all_localizations.clear();
//
//    long long start = -1;
//    if(print_progress) start = current_time();
//
//    std::vector<int> view_indexes;
//    constraint.order_views(obj.views,
//    		//output parameter:
//    		view_indexes);
//    start_view_index = std::max(std::min(start_view_index,(int)obj.views.size()-1),0);
//    if(stop_view_index==-1) stop_view_index = obj.views.size();
//    stop_view_index = std::max(std::min(stop_view_index,(int)obj.views.size()),0);
//
//    localization best_loc;
//    best_loc.log_probability = -std::numeric_limits<float>::infinity();
//    for(int j=0;j<(int)view_indexes.size();j++) {
//    	int index = view_indexes[j];
//    	if(index<start_view_index || index>=stop_view_index) continue;
//    	const view& v = obj.views[index];
//    	//todo: uncomment the following once visualize_object is working with the new representation
////		if(visualize_search) show_search_visualization(v,constraint.draw(v));
//
////    	region r = constraint.bounding_region(v);
////    	if(r.min.x!=std::numeric_limits<float>::infinity()) {
////    		if(visualize_search) show_search_visualization(v,constraint.draw(v));
////    		hypothesis_iterator h(&v,&im,r,lower_bound, print_progress);
////    		h++;
////    		while(h!=h.end() && h.has_localization() /*&& h->getBound()>best_loc.logP*/) {
////    			localization loc;
////    			loc.location = h->get_region().min;
////    			loc.log_probability = h->get_bound();
////    			loc.v = &v;
////    			v.get_bounding_box(loc.location,loc.x0,loc.x1,loc.y0,loc.y1);
////    			if(constraint.is_satisfied(loc)) {
////    				if(only_top_detection) {
////    					if(loc.log_probability>best_loc.log_probability) {
////    						best_loc = loc;
////    						lower_bound = loc.log_probability;
////    					}
////    					break;
////    				} else {
////    					all_localizations.push_back(loc);
////    				}
////    			}
////    			h++;
////    		}
////    	}
//		if(only_top_detection) {
//			localization loc = best_localization<search_space_constraint>(v,im,lower_bound,constraint,print_progress);
//			if(loc.log_probability>best_loc.log_probability) {
//				best_loc = loc;
//				lower_bound = loc.log_probability;
//			}
//		} else {
//	    	localizations<search_space_constraint>(v,im,lower_bound,
//	    			//output parameter:
//	    			all_localizations,
//	    			//overriding parameters with default arguments
//	    			constraint,nms_overlap_percent,print_progress);
//		}
//		if(print_progress) progress(start,j,obj,lower_bound);
//	}
//	if(print_progress) std::cout << std::endl;
//	if(only_top_detection && best_loc.log_probability!=-std::numeric_limits<float>::infinity()) {
//		all_localizations.push_back(best_loc);
//	} else {
//		std::sort(all_localizations.begin(),all_localizations.end(),localization::comparefn);
//	}
//}


} //namespace objrec
#endif /* DETECTION_H_ */
