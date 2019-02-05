/*
 * features.h
 *
 *  Created on: Jun 6, 2013
 */

#ifndef FEATURES_H_
#define FEATURES_H_

#include "edge_detect.h"
#include "textons.h"
#include <opencv2/imgproc/imgproc.hpp>

namespace objrec {


class feature_library {
	edge_detection_parameters e;
	textons t;
public:
	feature_library(const edge_detection_parameters& edges=edge_detection_parameters(), const textons& textons_=textons()) : e(edges), t(textons_) {}
	feature_library(std::istream& in, int& line_number) : e(in,line_number), t(in,line_number) {}
	const edge_detection_parameters& get_edges() const {return e;}
	const textons& get_textons() const {return t;}
	unsigned int size() const { return e.get_edge_directions().size() + t.n_clusters(); }
};

std::ostream& operator<<(std::ostream& out, const feature_library& library) {
	out << library.get_edges() << std::endl;
	out << library.get_textons() << std::endl;
	return out;
}


class feature_detections {
	const feature_library* library;
	std::vector<cv::Mat_<float> > edges;
	cv::Mat_<unsigned char> texton_map;
public:
	feature_detections(const feature_library* library, const cv::Mat_<cv::Vec3b>& picture, const cv::Mat_<float>& depth,
			const std::vector<cv::Mat_<float> >& precomputed_edges = std::vector<cv::Mat_<float> >())
	: library(library),
	  edges(precomputed_edges),
	  texton_map(library->get_textons().texton_map(picture)) {
		if(edges.size()==0) {
#ifdef TWO_THRESHOLD_FEATURES
			cv::Mat_<float> low_threshold_edge_angle_map, high_threshold_edge_angle_map;
			library->get_edges().edge_detect(picture,depth,
					//output parameters:
					low_threshold_edge_angle_map, high_threshold_edge_angle_map);
			edges.resize(library->get_edges().get_edge_directions().size());
			for(int j=0;j<edges.size();j++) {
				edges[j].create(picture.size());
				const edge_direction& ed = library->get_edges().get_edge_directions()[j];
				float low_threshold = library->get_edges().get_low_threshold();
				for(int y=0;y<picture.rows;y++) {
					for(int x=0;x<picture.cols;x++) {
						float low_threshold_angle = low_threshold_edge_angle_map(y,x);
						float high_threshold_angle = high_threshold_edge_angle_map(y,x);
						if(high_threshold_angle==high_threshold_angle && ed.in_range(high_threshold_angle)) { //if high_threshold_angle is not a NaN and is in range
							edges[j](y,x) = std::numeric_limits<float>::infinity();
						} else if(low_threshold_angle==low_threshold_angle && ed.in_range(low_threshold_angle)) { //if low_threshold_angle is not a NaN and is in range
							edges[j](y,x) = low_threshold;
						} else {
							edges[j](y,x) = -std::numeric_limits<float>::infinity();
						}
					}
				}
			}
		}
#else
		cv::Mat_<float> angle_map = library->get_edges().edge_detect(picture,depth);
		edges.resize(library->get_edges().get_edge_directions().size());
		for(int j=0;j<edges.size();j++) {
			edges[j].create(picture.size());
			const edge_direction& ed = library->get_edges().get_edge_directions()[j];
			float low_threshold = library->get_edges().get_low_threshold();
			for(int y=0;y<picture.rows;y++) {
				for(int x=0;x<picture.cols;x++) {
					float angle = angle_map(y,x);
					if(angle==angle && ed.in_range(angle)) { //if high_threshold_angle is not a NaN and is in range
						edges[j](y,x) = std::numeric_limits<float>::infinity();
					} else {
						edges[j](y,x) = -std::numeric_limits<float>::infinity();
					}
				}
			}
		}
	}		
#endif
	}

	//the returned matrix has 0 where the feature is detected and 255 otherwise
	cv::Mat_<unsigned char> operator[](unsigned int index) const {
		const std::vector<edge_direction>& edge_directions = library->get_edges().get_edge_directions();
		if(index<edge_directions.size()) {
			float low_threshold = library->get_edges().get_low_threshold();
			float high_threshold = library->get_edges().get_high_threshold();
			cv::Mat_<unsigned char> r(cv::Size(edges[index].cols,edges[index].rows));
			for(int y=0;y<edges[index].rows;y++) {
				for(int x=0;x<edges[index].cols;x++) {
					if(edges[index](y,x)<low_threshold) {
						r(y,x) = 0;
					} else if(edges[index](y,x)<high_threshold) {
						r(y,x) = 1;
					} else {
						r(y,x) = 2;
					}
				}
			}
			return r;
		}
		int texton_index = index - edge_directions.size();
		if(texton_index < library->get_textons().n_clusters()) {
			return texton_detections(texton_index, texton_map);
		}
		std::ostringstream err;
		err << "feature index " << index << " out of range (max: " << size()-1 << ")";
		throw std::runtime_error(err.str());
	}

	unsigned int size() const { return library->size(); }
};

template<typename pixel_getter>
void compute_distance_transform(const pixel_getter& detections,
		//output arguments:
		cv::Mat_<float>& distance_transform_, cv::Mat_<cv::Vec<unsigned short, 2> >& indexes) {
	distance_transform_.create(cv::Size(detections.width(), detections.height()));
	indexes.create(cv::Size(detections.width(), detections.height()));

	unsigned int max_d = detections.width() + detections.height();

	//compute the 1D distance transform in the horizontal direction
	cv::Mat_<unsigned int> dtx(cv::Size(detections.width(), detections.height()));
	cv::Mat_<unsigned short> indx(cv::Size(detections.width(), detections.height()));
	for(int y=0;y<detections.height();y++) {
		unsigned short last_index = std::numeric_limits<unsigned short>::max();
		unsigned int d = max_d;
		for(int x=0;x<detections.width();x++) {
			if(detections.get(x,y)) {
				d = 0;
				last_index = x;
			} else {
				d += 1;
			}
			dtx(y,x) = d*d;
			indx(y,x) = last_index;
		}
		d = max_d;
		for(int x=detections.width()-1;x>=0;x--) {
			if(detections.get(x,y)) {
				d = 0;
				last_index = x;
			} else {
				d += 1;
			}
			unsigned int d2 = d*d;
			if(d2<dtx(y,x)) {
				dtx(y,x) = d2;
				indx(y,x) = last_index;
			}
		}
	}

	//compute the vertical 1D distance transform using dtx,indx (computed above)
	// by the method described in "Distance Transforms of Sampled Functions" (described by Felzenszwalb et al., 2004)
	for(int x=0;x<dtx.cols;x++) {
		int k=0; //index of rightmost parabola in lower envelope
		std::vector<unsigned short> v(dtx.rows); //locations of parabolas in lower envelope
		v[0] = 0;
		std::vector<float> z(dtx.rows+1); //locations of boundaries between parabolas
		z[0] = -std::numeric_limits<float>::infinity();
		z[1] = std::numeric_limits<float>::infinity();

		for(int q=1;q<dtx.rows;q++) {
			float s = (float)((int)(dtx(q,x)+q*q)-(int)(dtx(v[k],x)+v[k]*v[k]))/(float)(2*q-2*(int)v[k]);
			while(s<=z[k]) {
				k = k - 1;
				s = (float)((int)(dtx(q,x)+q*q)-(int)(dtx(v[k],x)+v[k]*v[k]))/(float)(2*q-2*(int)v[k]);
			}
			k = k + 1;
			v[k] = q;
			z[k] = s;
			z[k+1] = std::numeric_limits<float>::infinity();
		}
		k = 0;
		for(int q=0;q<dtx.rows;q++) {
			while(z[k+1]<q) {
				k = k + 1;
			}
			int d = q-v[k];
			distance_transform_(q,x) = d*d+dtx(v[k],x);
			indexes(q,x) = cv::Vec<unsigned short, 2>(indx(v[k],x),v[k]);
		}
	}
}

} // namespace objrec
#endif /* FEATURES_H_ */
