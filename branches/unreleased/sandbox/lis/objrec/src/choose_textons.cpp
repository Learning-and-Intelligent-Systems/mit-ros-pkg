/*
 * choose_textons.cpp
 *
 *  Created on: Jun 4, 2013
 */

#include "gui.h"
#include "textons.h"
#include <iostream>

namespace objrec {
void initial_cluster_centers(int k_clusters, const std::vector<std::vector<float> >& filter_responses,
		//output parameter:
		std::vector<std::vector<float> >& cluster_centers) {
	cluster_centers.clear();
	cluster_centers.reserve(k_clusters);
	for(int j=0;j<k_clusters;j++) {
		int random_index = rand()%filter_responses[0].size();
		std::vector<float> cluster_center(filter_responses.size());
		for(int r=0;r<(int)filter_responses.size();r++) {
			cluster_center[r] = filter_responses[r][random_index];
		}
		cluster_centers.push_back(cluster_center);
	}
}

void initial_cluster_centers(int k_clusters, const std::vector<cv::Mat_<float> >& filter_responses,
		//output parameter:
		std::vector<std::vector<float> >& cluster_centers) {
	cluster_centers.clear();
	cluster_centers.reserve(k_clusters);
	for(int j=0;j<k_clusters;j++) {
		int rand_x = rand()%filter_responses[0].cols;
		int rand_y = rand()%filter_responses[0].rows;
		std::vector<float> cluster_center(filter_responses.size());
		for(int r=0;r<(int)filter_responses.size();r++) {
			cluster_center[r] = filter_responses[r](rand_y,rand_x);
		}
		cluster_centers.push_back(cluster_center);
	}
}

void new_cluster_means(int k_clusters, const cv::Mat_<unsigned char>& closest_cluster_indices,
		const std::vector<cv::Mat_<float> >& filter_responses,
		//output parameter:
		std::vector<std::vector<float> >& cluster_means
		) {
	std::vector<int> cluster_counts(k_clusters,0);
	cluster_means.resize(k_clusters);
	int n_filters = filter_responses.size();
	for(int j=0;j<k_clusters;j++) {
		cluster_means[j] = std::vector<float>(n_filters,0.0f);
	}
	for(int y=0;y<closest_cluster_indices.rows;y++) {
		for(int x=0;x<closest_cluster_indices.cols;x++) {
			unsigned char cluster_index = closest_cluster_indices(y,x);
			cluster_counts[cluster_index]++;
			for(int j=0;j<n_filters;j++) {
				cluster_means[cluster_index][j] += filter_responses[j](y,x);
			}
		}
	}
	for(int j=0;j<k_clusters;j++) {
		for(int r=0;r<n_filters;r++) {
			cluster_means[j][r] /= cluster_counts[j];
		}
	}
}

bool compare_clusters(const std::vector<float>& c1, const std::vector<float>& c2) {
	for(unsigned int j=0;j<std::min(c1.size(),c2.size());j++) {
		if(c1[j]<c2[j]) return true;
		if(c1[j]>c2[j]) return false;
		//continue if ==
	}
	return true;
}

} //namespace objrec
int main(int argc, char** argv) {
    const int k_clusters = 16;
    const float elongation = 1.0f;

    if(argc==1) {
		std::cerr << "usage:" << std::endl;
		std::cerr << " " << argv[0] << " image_1 image_2 ..." << std::endl;
		std::cerr << std::endl;
		std::cerr << "Chooses textons by k-means over a set of images (with k=" << k_clusters << ").  Displays progress in the texton map of the last image." << std::endl;
		return -1;
	}

    long long start, end;

	std::cerr << "Creating filters... " << std::flush;
	start = objrec::current_time();
	objrec::textons t(elongation);
	end = objrec::current_time();
    std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;


	int current_arg = 1;
	std::vector<cv::Mat_<float> > filter_responses;

	std::vector<cv::Mat_<float> > all_filter_responses(t.n_filters());
	for(;current_arg<argc;current_arg++) {
		cv::Mat_<cv::Vec3b> image_color = cv::imread(argv[current_arg]);
		std::cerr << "Applying filters to " << argv[current_arg] << "... " << std::flush;
		start = objrec::current_time();

		t.filter_responses(image_color,
				//output parameter:
				filter_responses);

		for(unsigned int j=0;j<filter_responses.size();j++) {
			//reshape the matrix to a column vector and concatenate it with the rest
			cv::Mat_<float> r = filter_responses[j].reshape(1,image_color.rows*image_color.cols);

			cv::Mat_<float> all_r(cv::Size(1,all_filter_responses[j].rows+r.rows));
			for(int k=0;k<all_filter_responses[j].rows;k++) {
				all_r(k,0) = all_filter_responses[j](k,0);
			}
			for(int k=0;k<r.rows;k++) {
				all_r(all_filter_responses[j].rows+k,0) = r(k,0);
			}
			all_filter_responses[j] = all_r;
		}

		end = objrec::current_time();
	    std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
	}


    std::vector<std::vector<float> > cluster_centers;
    objrec::initial_cluster_centers(k_clusters, all_filter_responses,
    		//output parameter:
    		cluster_centers);

    t = objrec::textons(t,cluster_centers);

    //todo: automatically stop when there are no changes
    for(int j=0;j<150;j++) {
    	cv::Mat_<unsigned char> closest_cluster_indices;

    	std::cerr << "Calculating texton map for test image... " << std::flush;
    	start = objrec::current_time();
    	t.texton_map(filter_responses,
    			//output parameter:
    			closest_cluster_indices);
    	end = objrec::current_time();
    	std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;

    	cv::imshow("clusters",objrec::mat_bgr(closest_cluster_indices));
    	cv::waitKey(200);


    	std::cerr << "Finding closest cluster means... " << std::flush;
    	start = objrec::current_time();
    	t.texton_map(all_filter_responses,
    			//output parameter:
    			closest_cluster_indices);
    	end = objrec::current_time();
    	std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;


    	std::cerr << "Finding new cluster means... " << std::flush;
    	start = objrec::current_time();
    	objrec::new_cluster_means(k_clusters,closest_cluster_indices,all_filter_responses,
    			//output parameter:
    			cluster_centers);
    	t = objrec::textons(t,cluster_centers);

    	end = objrec::current_time();
    	std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
    }

    std::sort(cluster_centers.begin(), cluster_centers.end(), objrec::compare_clusters);
    std::cout << t << std::endl;
}
