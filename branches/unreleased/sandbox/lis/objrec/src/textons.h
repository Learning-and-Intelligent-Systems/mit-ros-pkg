/*
 * textons.h
 *
 *  Created on: Jun 4, 2013
 */

#ifndef TEXTONS_H_
#define TEXTONS_H_

#include <opencv2/imgproc/imgproc.hpp>

namespace objrec {

class textons {
	float elongation;
	std::vector<float> scales;
	std::vector<std::vector<float> > cluster_centers;
	std::vector<cv::Mat_<float> > filters;

	void initialize(const std::vector<float>& scales_param);
	friend std::ostream& operator<<(std::ostream& out, const textons& t);
	friend std::istream& operator>>(std::istream& in, textons& t);
public:
	textons(float elongation=3.0, const std::vector<float>& scales=std::vector<float>());
	textons(const std::vector<std::vector<float> >& cluster_centers, float elongation=3.0,
			const std::vector<float>& scales=std::vector<float>());
	textons(const textons& other, const std::vector<std::vector<float> >& cluster_centers);
	textons(std::istream& in, int& line_number);

	int n_filters() const { return filters.size(); }
	int n_clusters() const { return cluster_centers.size(); }
	bool operator==(const textons& other) const {
		if(elongation!=other.elongation ||
				scales.size()!=other.scales.size() ||
				n_clusters()!=other.n_clusters()) {
			return false;
		}
		for(unsigned int j=0;j<scales.size();j++) {
			if(scales[j]!=other.scales[j]) return false;
		}
		for(unsigned int j=0;j<cluster_centers.size();j++) {
			for(unsigned int k=0;k<cluster_centers[j].size();k++) {
				if(cluster_centers[j][k]!=other.cluster_centers[j][k]) return false;
			}
		}
		return true;
	}
	void filter_responses(const cv::Mat_<cv::Vec3b>& color_im,
			//output parameter:
			std::vector<cv::Mat_<float> >& responses) const;

	void texton_map(const std::vector<cv::Mat_<float> >& responses,
			//output parameter:
			cv::Mat_<unsigned char>& closest_cluster_indices) const;

	cv::Mat_<unsigned char> texton_map(const cv::Mat_<cv::Vec3b>& color_im) const {
		std::vector<cv::Mat_<float> > responses;
		filter_responses(color_im,
				//output parameter:
				responses);
		cv::Mat_<unsigned char> map;
		texton_map(responses,
				//output parameter:
				map);
		return map;
	}
};

cv::Mat_<unsigned char> texton_detections(unsigned char texton_index, const cv::Mat_<unsigned char> & texton_map) {
	cv::Mat_<unsigned char> d(texton_map.rows,texton_map.cols);
	for(int y=0;y<texton_map.rows;y++) {
		for(int x=0;x<texton_map.cols;x++) {
			if(texton_map(y,x)==texton_index) {
				d(y,x) = 0;
			} else {
				d(y,x) = 255;
			}
		}
	}
	return d;
}

cv::Mat_<cv::Vec3b> textons_bgr(const cv::Mat_<cv::Vec3b>& picture, const textons& t = textons());

} // namespace objrec

#endif /* TEXTONS_H_ */
