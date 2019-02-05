/*
 * learning.h
 *
 *  Created on: Aug 25, 2014
 */

#ifndef LEARNING_H_
#define LEARNING_H_

#include "simulation.h"
#include "representation.h"
#include <set>

namespace objrec {

void check_finished();

template <class T>
void resize(const cv::Mat_<T>& src, cv::Mat_<T>& dst, cv::Size dsize, double fx=0, double fy=0, T zero=T()) {
	if(dsize==cv::Size()) {
		dsize = cv::Size(round(src.cols*fx),round(src.rows*fy));
	} else {
		fx = dsize.width/src.cols;
		fy = dsize.height/src.rows;
	}

	dst.create(dsize);
	set_to(dst,zero);
	for(int y=0;y<src.rows;y++) {
		for(int x=0;x<src.cols;x++) {
			dst(round(y*fy),round(x*fx)) = src(y,x);
		}
	}
}
/*
ATA = {
{ATA11,ATA21,ATA31,ATA41,ATA51,ATA61,ATA71},
{ATA21,ATA22,ATA32,ATA42,ATA52,ATA62,ATA72},
{ATA31,ATA32,ATA33,ATA43,ATA53,ATA63,ATA73},
{ATA41,ATA42,ATA43,ATA44,ATA54,ATA64,ATA74},
{ATA51,ATA52,ATA53,ATA54,ATA55,ATA65,ATA75},
{ATA61,ATA62,ATA63,ATA64,ATA65,ATA66,ATA76},
{ATA71,ATA72,ATA73,ATA74,ATA75,ATA76,ATA77}
};

ATB = {
{ATB1x,ATB1y},
{ATB2x,ATB2y},
{ATB3x,ATB3y},
{ATB4x,ATB4y},
{ATB5x,ATB5y},
{ATB6x,ATB6y},
{ATB7x,ATB7y}
};

X = {
{pxrx,pyrx},
{pxry,pyry},
{pxrz,pyrz},
{pxpx,pypx},
{pxpy,pypy},
{pxd,pyd},
{cx,cy}
};

InputForm[Simplify[Tr[Transpose[X].ATA.X]-2*Tr[Transpose[X].ATB]+trBTB]]

InputForm[Simplify[Inverse[ATA]]]

 */
struct visual_feature_statistics {
	double ATA11;
	double ATA21, ATA22;
	double ATA31, ATA32, ATA33;
	double ATA41, ATA42, ATA43, ATA44;

	double ATB1x, ATB1y;
	double ATB2x, ATB2y;
	double ATB3x, ATB3y;
	double ATB4x, ATB4y;

	double trBTB;


	visual_feature_statistics() :
		ATA11(0),
		ATA21(0), ATA22(0),
		ATA31(0), ATA32(0), ATA33(0),
		ATA41(0), ATA42(0), ATA43(0), ATA44(0),
		ATB1x(0), ATB1y(0),
		ATB2x(0), ATB2y(0),
		ATB3x(0), ATB3y(0),
		ATB4x(0), ATB4y(0),
		trBTB(0)
	{}
#ifdef TWO_THRESHOLD_FEATURES
	visual_part part(float receptive_field_radius, float high_threshold_log_probability_shift) const {
#else
	visual_part part(float receptive_field_radius) const {
#endif
		cv::Mat_<double> ATA = (cv::Mat_<double>(4,4) <<
				ATA11,ATA21,ATA31,ATA41,
				ATA21,ATA22,ATA32,ATA42,
				ATA31,ATA32,ATA33,ATA43,
				ATA41,ATA42,ATA43,ATA44);
		cv::Mat_<double> ATB = (cv::Mat_<double>(4,2) <<
				ATB1x,ATB1y,
				ATB2x,ATB2y,
				ATB3x,ATB3y,
				ATB4x,ATB4y);
		cv::Mat_<double> X = ATA.inv()*ATB;
		cv::Mat_<double> XT = X.t();
		double n_images = ATA11;
		double variance = 1.0/(n_images-1.0)*(cv::trace(XT*ATA*X)[0]-2.0*cv::trace(XT*ATB)[0]+trBTB);
		if(variance<0) {
			std::cerr << "negative variance: " << variance << ". setting it to 0" << std::endl;
			variance = 0;
		}
		normal_distribution distribution(variance,receptive_field_radius);
		double cx=X(0,0),cy=X(0,1);
		double pxrx=X(1,0),pyrx=X(1,1);
		double pxry=X(2,0),pyry=X(2,1);
		double pxrz=X(3,0),pyrz=X(3,1);
#ifdef TWO_THRESHOLD_FEATURES
		return visual_part(distribution,high_threshold_log_probability_shift,cx,cy,pxrx,pxry,pxrz,pyrx,pyry,pyrz);
#else
		return visual_part(distribution,cx,cy,pxrx,pxry,pxrz,pyrx,pyry,pyrz);
#endif
	}
	void add(double rx, double ry, double rz, double x, double y) {
		ATA11+=1;
		ATA21+=rx;    ATA22+=rx    *rx;
		ATA31+=ry;    ATA32+=ry    *rx;ATA33+=ry    *ry;
		ATA41+=rz;    ATA42+=rz    *rx;ATA43+=rz    *ry;ATA44+=rz    *rz;

		ATB1x += x;       ATB1y += y;
		ATB2x += x*rx;    ATB2y += y*rx;
		ATB3x += x*ry;    ATB3y += y*ry;
		ATB4x += x*rz;    ATB4y += y*rz;

		trBTB += x*x + y*y;
	}
};

struct visual_part_more_info {
	visual_part_more_info(short visual_feature_index, short x, short y, visual_part part) : visual_feature_index(visual_feature_index),
			x(x), y(y), part(part) {}
	short visual_feature_index, x, y;
	visual_part part;
};

class enumerated_visual_features {
private:
	int current_image;
	double target_object_depth;
	static inline bool compare_features(const visual_part_more_info& f1, const visual_part_more_info& f2) {
		return f1.part.distribution.variance() < f2.part.distribution.variance();
	}
	inline static void align_image(int width, double scale, const cv::Mat_<cv::Vec3b> image,
			//output parameter:
			cv::Mat_<cv::Vec3b>& aligned_image
			) {
		cv::Mat_<cv::Vec3b> resized;
		cv::resize(image,resized,cv::Size(),scale,scale,CV_INTER_LANCZOS4);
		aligned_image.create(width,width);
		set_to(aligned_image,cv::Vec3b(0,0,0)); //fill with black

		int dx = (resized.cols-aligned_image.cols)/2; //must be centered
		int dy = (resized.rows-aligned_image.rows)/2;
		//crop the image
		for(int y=0;y<aligned_image.rows;y++) {
			for(int x=0;x<aligned_image.cols;x++) {
				int xdt = x + dx;
				int ydt = y + dy;
				if(
						0<=xdt && xdt<resized.cols && 0<=ydt && ydt<resized.rows &&
						0<=x   && x<width          && 0<=y   && y<width) {
					aligned_image(y,x) = resized(ydt,xdt);
				}
			}
		}
	}
	struct features_pixel_getter {
		const cv::Mat_<unsigned char>& feature_detections_;
		features_pixel_getter(const cv::Mat_<unsigned char>& feature_detections_) : feature_detections_(feature_detections_) {};
		bool get(int x, int y) const { return feature_detections_(y,x)==1; }
		int width() const { return feature_detections_.cols; }
		int height() const { return feature_detections_.rows; }
	};
protected:
	std::vector<visual_part_more_info> all_enumerated_features;
	std::vector<visual_feature_statistics> feature_statistics;
	unsigned int image_width;
	void compute_variance_maps(
			//output argument:
			std::vector<cv::Mat_<double> >& variance_maps) const {
		variance_maps.resize(n_features());
		for(int j=0;j<n_features();j++) {
			check_finished();
			cv::Mat_<double>& variances = variance_maps[j];
			variances.create(cv::Size(image_width,image_width));
			for(int y=0;y<variances.rows;y++) {
				for(int x=0;x<variances.cols;x++) {
					variances(y,x) = all_enumerated_features[feature_index(j,x,y)].part.distribution.variance();
				}
			}
		}
	}
	unsigned int feature_index(int visual_feature_index, int x, int y) const {
		return x+image_width*(y+image_width*visual_feature_index); //no bounds checking
	}

	enumerated_visual_features() : current_image(0),
			target_object_depth(std::numeric_limits<double>::quiet_NaN()) {}
	virtual ~enumerated_visual_features() {}
	virtual void set_current_image(const cv::Mat_<cv::Vec3b>& picture, const cv::Mat_<float>& depth, double scale)=0;
	virtual void free_current_image()=0;
	virtual void aligned_feature_detections(unsigned int visual_feature_index,
			//output parameter:
			cv::Mat_<unsigned char>& detections) const =0;
	virtual unsigned int n_features() const =0;
public:
	void initialize(int image_width, double target_object_depth) {
		this->image_width = image_width;
		this->target_object_depth = target_object_depth;
		feature_statistics.resize(image_width*image_width*n_features(),visual_feature_statistics());
	}
	void add_image(
			const cv::Mat_<cv::Vec3b>& picture,
			const cv::Mat_<float>& depth,
			double rx, double ry, double rz,
			double depthc,
			const std::string& image_output_directory
			) {
		current_image++;
		double scale = depthc/target_object_depth;

		set_current_image(picture,depth,scale);

		if(image_output_directory!="" && feature_statistics.size()>0) { //todo: write separately for textons (with textures) and edges (no textures)
			cv::Mat_<cv::Vec3b> aligned_image;

			std::ostringstream f;
			f.width(4);
			f.fill('0');
			f << current_image;
			align_image(image_width, scale, picture,
					//output parameter:
					aligned_image);
			cv::imwrite(image_output_directory+"/"+f.str()+".png",aligned_image);
		}

		for(unsigned int j=0;j<n_features();j++) {
			check_finished();
			cv::Mat_<unsigned char> feature_map;
			aligned_feature_detections(j,feature_map);
			cv::Mat_<float> distance_transform;
			cv::Mat_<cv::Vec<unsigned short, 2> > indexes;
			compute_distance_transform(features_pixel_getter(feature_map), distance_transform, indexes);
			//add the distance transforms
			for(int y=0;y<image_width;y++) {
				for(int x=0;x<image_width;x++) {
					double feature_x = ((int)indexes(y,x)[0]-(int)image_width/2)*target_object_depth;
					double feature_y = ((int)indexes(y,x)[1]-(int)image_width/2)*target_object_depth;
					visual_feature_statistics& fxy = feature_statistics[feature_index(j,x,y)];
					fxy.add(rx,ry,rz,feature_x,feature_y);
				}
			}
		}
		free_current_image();
	}
	void calculate_enumerated_features() {
		all_enumerated_features.clear();
		all_enumerated_features.reserve(feature_statistics.size());
		for(int j=0;j<n_features();j++) {
			check_finished();
			for(int y=0;y<image_width;y++) {
				for(int x=0;x<image_width;x++) {
					float receptive_field_radius = 5; //dummy value, will be replaced in select_features()
					const visual_feature_statistics& fxy = feature_statistics[feature_index(j,x,y)];
#ifdef TWO_THRESHOLD_FEATURES
					float high_threshold_log_probability_shift = 0; //dummy value, will be replaced in select_features()
					visual_part_more_info p(j,x,y,fxy.part(receptive_field_radius,high_threshold_log_probability_shift));
#else
					visual_part_more_info p(j,x,y,fxy.part(receptive_field_radius));
#endif
					all_enumerated_features.push_back(p);
				}
			}
		}
	}

	void sort_enumerated_features() {
		std::sort(all_enumerated_features.begin(), all_enumerated_features.end(), compare_features);
	}

	void select_features(int n_features, double receptive_field_radius,
#ifdef TWO_THRESHOLD_FEATURES
			double high_threshold_log_probability_shift,
#endif
			double max_visual_part_position_variance, double min_visual_part_position_variance,
			//output parameter:
			std::vector<visual_part_more_info>& selected_features) const;
};

class enumerated_edge_features : public enumerated_visual_features {
	edge_detection_parameters edp;
	cv::Mat_<float> edge_angles;
protected:
	void set_current_image(const cv::Mat_<cv::Vec3b>& picture, const cv::Mat_<float>& depth, double scale) {
		if(feature_statistics.size()==0) return;

		cv::Mat_<float> resized_edge_angles;
#ifdef TWO_THRESHOLD_FEATURES
		cv::Mat_<float> original_edge_angles_low_threshold, original_edge_angles_high_threshold;
		edp.edge_detect(picture,depth,
				//output parameters:
				original_edge_angles_low_threshold, original_edge_angles_high_threshold);
		resize(original_edge_angles_high_threshold,resized_edge_angles,cv::Size(),scale,scale,std::numeric_limits<float>::quiet_NaN());
#else
		cv::Mat_<float> original_edge_angles;
		original_edge_angles = edp.edge_detect(picture,depth);
		resize(original_edge_angles,resized_edge_angles,cv::Size(),scale,scale,std::numeric_limits<float>::quiet_NaN());
#endif
		int width = image_width;//feature_statistics[0].cols;//resized_edge_angles.cols*1.2;
		int height = image_width;//feature_statistics[0].rows;//resized_edge_angles.rows*1.2;
		edge_angles.create(cv::Size(width,height));

		int dx = (resized_edge_angles.cols-width)/2;
		int dy = (resized_edge_angles.rows-height)/2;

		for(int y=0; y<height; y++) {
			for(int x=0; x<width; x++) {
				int rx = x + dx, ry = y + dy;
				if(0<=rx && rx<resized_edge_angles.cols &&
				   0<=ry && ry<resized_edge_angles.rows) {
					edge_angles(y,x) = resized_edge_angles(ry,rx);
				} else {
					edge_angles(y,x) = std::numeric_limits<float>::quiet_NaN();
				}
			}
		}
	}
	void free_current_image() { edge_angles = cv::Mat_<float>(); }
	void aligned_feature_detections(unsigned int visual_feature_index,
			//output parameter:
			cv::Mat_<unsigned char>& detections) const {
		const edge_direction& ed = edp.get_edge_directions()[visual_feature_index];
		detections.create(cv::Size(edge_angles.cols,edge_angles.rows));
		for(int y=0;y<edge_angles.rows;y++) {
			for(int x=0;x<edge_angles.cols;x++) {
				if(ed.in_range(edge_angles(y,x))) {
					detections(y,x) = 1;
				} else {
					detections(y,x) = 0;
				}
			}
		}
	}
	unsigned int n_features() const { return edp.get_edge_directions().size(); }
public:
	enumerated_edge_features(const edge_detection_parameters& edp) {
//		this->edp = edp;
		this->edp = edge_detection_parameters(200,2600,edp.get_edge_directions()); //todo: fix this hack. included so that simulated edges are not affected by precomputed edges
	}

	const edge_detection_parameters& get_edges() const {return edp;}
	virtual ~enumerated_edge_features() {}
	void show() const {
		std::vector<cv::Mat_<double> > variance_maps;
		compute_variance_maps(variance_maps);
		//todo: add visualization of textons
		const std::vector<edge_direction>& edge_directions = edp.get_edge_directions();
		for(int j=0;j<(int)edge_directions.size();j++) {
			double min_val = std::numeric_limits<double>::infinity();
			double max_val = -std::numeric_limits<double>::infinity();
			const cv::Mat_<double>& variances = variance_maps[j];
			cv::Mat_<cv::Vec3b> show_mat = mat_bgr(variances,true);
			for(int y=0;y<variances.rows;y++) {
				for(int x=0;x<variances.cols;x++) {
					if(variances(y,x)==std::numeric_limits<double>::infinity()) {
						show_mat(y,x) = cv::Vec3b(0,0,255);
					} else {
						min_val = std::min(variances(y,x),min_val);
						max_val = std::max(variances(y,x),max_val);
					}
				}
			}
			std::ostringstream title;
			title << edge_directions[j] << " - edge direction (min: " << min_val << " max: " << max_val << ")";
			cv::imshow(title.str(),show_mat);
			cv::waitKey(80);
		}
	}
};

class enumerated_texton_features : public enumerated_visual_features {
	textons t;
	cv::Mat_<unsigned char> texton_map;
protected:
	void set_current_image(const cv::Mat_<cv::Vec3b>& picture, const cv::Mat_<float>& depth, double scale) {
		if(feature_statistics.size()==0) return;

		cv::Mat_<unsigned char> original_texton_map = t.texton_map(picture);
		for(int y=0;y<depth.rows;y++) {
			for(int x=0;x<depth.cols;x++) {
				float d = depth(y,x);
				if(d!=d) { //d is a NaN
					original_texton_map(y,x) = std::numeric_limits<unsigned char>::max();
				}
			}
		}
		cv::Mat_<unsigned char> resized_texton_map;
		resize(original_texton_map,resized_texton_map,cv::Size(),scale,scale,std::numeric_limits<float>::quiet_NaN());

		int width = image_width;//feature_statistics[0].cols;//resized_texton_map.cols*1.2;
		int height = image_width;//feature_statistics[0].rows;//resized_texton_map.rows*1.2;
		texton_map.create(cv::Size(width,height));

		int dx = (resized_texton_map.cols-width)/2;
		int dy = (resized_texton_map.rows-height)/2;
		for(int y=0; y<height; y++) {
			for(int x=0; x<width; x++) {
				int rx = x + dx, ry = y + dy;
				if(0<=rx && rx<resized_texton_map.cols &&
				   0<=ry && ry<resized_texton_map.rows) {
					texton_map(y,x) = resized_texton_map(ry,rx);
				} else {
					texton_map(y,x) = std::numeric_limits<unsigned char>::max();
				}
			}
		}
	}
	void free_current_image() { texton_map = cv::Mat_<unsigned char>(); }
	void aligned_feature_detections(unsigned int visual_feature_index,
			//output parameter:
			cv::Mat_<unsigned char>& detections) const {
		detections.create(texton_map.rows,texton_map.cols);
		for(int y=0;y<texton_map.rows;y++) {
			for(int x=0;x<texton_map.cols;x++) {
				detections(y,x) = (texton_map(y,x)==visual_feature_index)?1:0;
			}
		}
	}
	unsigned int n_features() const { return t.n_clusters(); }
public:
	enumerated_texton_features(const textons& t) : t(t) {}

	const textons& get_textons() const {return t;}
};

struct depth_feature_statistics {
	double ATA11;
	double ATA21, ATA22;
	double ATA31, ATA32, ATA33;
	double ATA41, ATA42, ATA43, ATA44;

	double ATB1;
	double ATB2;
	double ATB3;
	double ATB4;

	double trBTB;


	depth_feature_statistics() :
		ATA11(0),
		ATA21(0), ATA22(0),
		ATA31(0), ATA32(0), ATA33(0),
		ATA41(0), ATA42(0), ATA43(0), ATA44(0),
		ATB1(0),
		ATB2(0),
		ATB3(0),
		ATB4(0),
		trBTB(0)
	{}
	depth_part part(float x, float y, float receptive_field_radius) const {
		cv::Mat_<double> ATA = (cv::Mat_<double>(4,4) <<
				ATA11,ATA21,ATA31,ATA41,
				ATA21,ATA22,ATA32,ATA42,
				ATA31,ATA32,ATA33,ATA43,
				ATA41,ATA42,ATA43,ATA44);
		cv::Mat_<double> ATB = (cv::Mat_<double>(4,1) <<
				ATB1,
				ATB2,
				ATB3,
				ATB4);
		cv::Mat_<double> X = ATA.inv()*ATB;
		cv::Mat_<double> XT = X.t();
		double n_images = ATA11;
		double variance = 1.0/(n_images-1.0)*(cv::trace(XT*ATA*X)[0]-2.0*cv::trace(XT*ATB)[0]+trBTB);
		if(variance<0) {
			std::cerr << "negative variance: " << variance << ". setting it to 0" << std::endl;
			variance = 0;
		}
		normal_distribution distribution(variance,receptive_field_radius);
		double cd=X(0,0);
		double drx=X(1,0);
		double dry=X(2,0);
		double drz=X(3,0);

		return depth_part(distribution,x,y,cd,drx,dry,drz);
	}
	void add(double rx, double ry, double rz, double depth) {
		ATA11+=1;
		ATA21+=rx;    ATA22+=rx    *rx;
		ATA31+=ry;    ATA32+=ry    *rx;ATA33+=ry    *ry;
		ATA41+=rz;    ATA42+=rz    *rx;ATA43+=rz    *ry;ATA44+=rz    *rz;

		ATB1 += depth;
		ATB2 += depth*rx;
		ATB3 += depth*ry;
		ATB4 += depth*rz;

		trBTB += depth*depth;
	}
};


class enumerated_depth_features {
private:
//	cv::Mat_<double> depth_sum;
//	cv::Mat_<double> depth2_sum;
	cv::Mat_<unsigned int> depth_count;
	static const double min_probability = 1;
	int current_image;
//	double reciprocal_n_images;
	double target_object_depth;
//	double x0_sum, x1_sum, y0_sum, y1_sum; //for average bounding box
//	double x0_max, x1_max, y0_max, y1_max; //for max bounding box
	static inline bool compare_features(const depth_part& f1, const depth_part& f2) {
		return f1.distribution.variance() < f2.distribution.variance();
	}
	std::vector<depth_part> all_enumerated_features;
	std::vector<depth_feature_statistics> feature_statistics;
	unsigned int image_width;
	void compute_variance_map(
			//output argument:
			cv::Mat_<double>& variance_map) const {
		variance_map.create(cv::Size(image_width,image_width));
		for(int y=0;y<variance_map.rows;y++) {
			for(int x=0;x<variance_map.cols;x++) {
				variance_map(y,x) = all_enumerated_features[feature_index(x,y)].distribution.variance();
			}
		}
	}
	unsigned int feature_index(int x, int y) const {
		return x+image_width*y; //no bounds checking
	}
//
//	inline double mean(int x, int y) const {
//		return depth_sum(y,x)/(double)depth_count(y,x);
//	}
//	inline double variance(int x, int y) const {
//		double sum = depth_sum(y,x);
//		double count = depth_count(y,x);
//		double variance = 1./(count-1.)*(depth2_sum(y,x)-sum*sum*1./count);
//		return variance;
//	}
	inline double probability(int x, int y) const {
		return depth_count(y,x)/current_image;
	}
//	void all_features_sorted(
//			//output parameter:
//			std::vector<depth_part>& features
//			) const {
//		features.clear();
//		features.reserve(depth_sum.cols*depth_sum.rows);
//		int cx = depth_sum.cols/2, cy = depth_sum.rows/2;
//		for(int y=0;y<depth_sum.rows;y++) {
//			for(int x=0;x<depth_sum.cols;x++) {
//				float prob = probability(x,y);
//				float m = mean(x,y) - target_object_depth; //mean w.r.t. the object center
//				float var = variance(x,y);
//				float receptive_field_radius = .02; //dummy value, will be replaced in select_depth_features()
//				float nx = (float)(x-cx)*target_object_depth;
//				float ny = (float)(y-cy)*target_object_depth;
//				if(prob>=min_probability) {
//					features.push_back(depth_part(m,normal_distribution(nx,ny,var,receptive_field_radius)));
//				}
//			}
//		}
//		std::sort(features.begin(), features.end(), compare_features);
//	}
public:
	enumerated_depth_features() : current_image(0),
			target_object_depth(std::numeric_limits<double>::quiet_NaN()) {}

	void initialize(int image_width, double target_object_depth) {
		depth_count.create(image_width,image_width);
		set_to(depth_count,(unsigned int)0);
		this->image_width = image_width;
		this->target_object_depth = target_object_depth;
		feature_statistics.resize(image_width*image_width,depth_feature_statistics());
	}
	void add_image(
			const cv::Mat_<float>& depth,
			double rx, double ry, double rz,
			double depthc
			) {

//		cv::Mat_<unsigned char> mask(depth.rows,depth.cols), rotated_mask;
//		for(int y=0;y<depth.rows;y++) {
//			for(int x=0;x<depth.cols;x++) {
//				float d = depth(y,x);
//				if(d==d) { //d==d means d is not a NaN
//					mask(y,x) = 255;
//				}
//				else {
//					mask(y,x) = 0;
//				}
//			}
//		}

//		cv::Mat_<float> ddx(depth.rows,depth.cols); //only use dx because we are working with a stereo camera with its disparity in x
//		int depth_aperture_size = 7;
//		cv::Sobel(depth,ddx,CV_32F,1,0,depth_aperture_size);
//		cv::Mat_<float> see_mask(depth.rows,depth.cols);
//		for(int y=0;y<depth.rows;y++) {
//			for(int x=0;x<depth.cols;x++) {
//				float dx = ddx(y,x);
//				if(dx==dx && fabs(dx)<6) {
//					mask(y,x) = 255;
//				}
//				else {
//					mask(y,x) = 0;
//				}
////				see_mask(y,x) = mask(y,x);
//			}
//		}
//		cv::imshow("ddx",mat_bgr(ddx));
//		cv::imshow("mask",mat_bgr(see_mask));
//		float min_mask, max_mask;
//		min_max_mat(ddx,min_mask,max_mask);
//		std::cerr << "mask from " << min_mask << " to " << max_mask << std::endl;
//		cv::waitKey(0);


		current_image++;
//		reciprocal_n_images = 1.0/(double)current_image;
		double scale = depthc/target_object_depth;
		cv::Mat_<float> resized_depth;
		cv::resize(depth,resized_depth,cv::Size(),scale,scale,cv::INTER_NEAREST);

//		cv::Mat_<unsigned char> resized_mask;
//		cv::resize(mask,resized_mask,cv::Size(),scale,scale,cv::INTER_NEAREST);
////		//todo: use cv::getStructuringElement() to make a nicer erosion
////		cv::Mat erode_kernel = cv::Mat_<unsigned char>(20,20,1);
////		cv::Mat_<unsigned char> eroded_mask;
////		cv::erode(resized_mask,eroded_mask,erode_kernel);

		int dx = (resized_depth.cols-(int)image_width)/2; //must be centered
		int dy = (resized_depth.rows-(int)image_width)/2;
//		float ddepth = target_object_depth - depthc;
//		double x0 = std::numeric_limits<double>::infinity();
//		double x1 = -std::numeric_limits<double>::infinity();
//		double y0 = std::numeric_limits<double>::infinity();
//		double y1 = -std::numeric_limits<double>::infinity();
		//add the depth image
		int cx = image_width/2, cy = image_width/2;
		for(int y=0;y<image_width;y++) {
			for(int x=0;x<image_width;x++) {
				int xdt = x + dx;
				int ydt = y + dy;
				if(0<=xdt && xdt<resized_depth.cols && 0<=ydt && ydt<resized_depth.rows) {// && resized_mask(ydt,xdt)!=0) {
					float d = resized_depth(ydt,xdt) - depthc;

					if(d==d) {
						depth_count(y,x) += 1;
//						double feature_depth = (double)d-depthc; //todo: this cancels the depthc from ddepth
						depth_feature_statistics& fxy = feature_statistics[feature_index(x,y)];
						fxy.add(rx,ry,rz,d);
//						depth_sum(y,x) += d;
//						depth2_sum(y,x) += d*d;
//						depth_count(y,x)++;
//						double xfeature = (x-cx)*target_object_depth;
//						double yfeature = (y-cy)*target_object_depth;
//						x0 = std::min(x0,xfeature);
//						x1 = std::max(x1,xfeature);
//						y0 = std::min(y0,yfeature);
//						y1 = std::max(y1,yfeature);
					}
				}
			}
		}
//		x0_sum += x0;
//		x1_sum += x1;
//		y0_sum += y0;
//		y1_sum += y1;
//		x0_max = std::min(x0_max,x0);
//		x1_max = std::max(x1_max,x1);
//		y0_max = std::min(y0_max,y0);
//		y1_max = std::max(y1_max,y1);
	}
//	void average_bounding_box(double& x0, double& x1, double& y0, double& y1) const {
//		x0 = x0_sum*reciprocal_n_images;
//		x1 = x1_sum*reciprocal_n_images;
//		y0 = y0_sum*reciprocal_n_images;
//		y1 = y1_sum*reciprocal_n_images;
//	}
//	void max_bounding_box(double& x0, double& x1, double& y0, double& y1) const {
//		x0 = x0_max;
//		x1 = x1_max;
//		y0 = y0_max;
//		y1 = y1_max;
//	}
//	int width() const { return depth_sum.cols; }
//	int height() const { return depth_sum.rows; }

	void calculate_enumerated_features() {
		all_enumerated_features.clear();
		all_enumerated_features.reserve(feature_statistics.size());
		const depth_part high_variance_part(normal_distribution(std::numeric_limits<float>::infinity(),5),0,0,0,0,0,0);
		for(int y=0;y<image_width;y++) {
			for(int x=0;x<image_width;x++) {
				if(probability(x,y)>=min_probability) {
					float receptive_field_radius = 5; //dummy value, will be replaced in select_features()
					const depth_feature_statistics& fxy = feature_statistics[feature_index(x,y)];
					float feature_x = (x-(int)image_width/2)*target_object_depth;
					float feature_y = (y-(int)image_width/2)*target_object_depth;
					depth_part p = fxy.part(feature_x,feature_y,receptive_field_radius);
					all_enumerated_features.push_back(p);
				} else {
					all_enumerated_features.push_back(high_variance_part);
				}
			}
		}
	}
	void sort_enumerated_features() {
		std::sort(all_enumerated_features.begin(), all_enumerated_features.end(), compare_features);
	}

	void select_features(int n_features, double receptive_field_radius, double max_depth_variance, double min_depth_variance,
			//output parameter:
			std::vector<depth_part>& selected_features) const;

	void show() const {
		cv::Mat_<double> probability(cv::Size(image_width,image_width),(double)0);
		cv::Mat_<double> variance   (cv::Size(image_width,image_width),(double)0);
		compute_variance_map(variance);

		for(int y=0;y<image_width;y++) {
			for(int x=0;x<image_width;x++) {
				double prob = this->probability(x,y);
				if(prob>=min_probability) {
					probability(y,x) = prob;
				} else {
					probability(y,x) = variance(y,x) = std::numeric_limits<double>::quiet_NaN();
				}
			}
		}
		double min_variance, max_variance, min_probability, max_probability;
		min_max_mat(variance,min_variance,max_variance);
		min_max_mat(probability,min_probability,max_probability);

		std::ostringstream title_variance, title_probability;
		title_variance << "depth variance " << min_variance << " to " << max_variance;
		title_probability << "depth probability " << min_probability << " to " << max_probability;

		cv::imshow(title_variance.str(),mat_bgr(variance));
		cv::waitKey(80);
		cv::imshow(title_probability.str(),mat_bgr(probability));
		cv::waitKey(80);
	}
};

double drand() { return (double) rand() / (double) RAND_MAX; }

//returns depth of center of object
void sample_synthetic_image(OpenGL_X_window& w, const mesh& m, const viewpoint& vp, int rz_symmetry_order, double ambient_light,
		//output parameters:
		cv::Mat_<cv::Vec3b>& cropped_picture, cv::Mat_<cv::Vec3b>& cropped_textured_picture, cv::Mat_<float>& cropped_depth,
		double& rx, double& ry, double& rz,
		double& depthc
		) {

//	double min_screen_updown = m.half_FOV_angle_up_down(), max_screen_updown = -min_screen_updown;
//	double min_screen_leftright = m.half_FOV_angle_up_down(), max_screen_leftright = -min_screen_leftright;

	double min_screen_updown, max_screen_updown;
	double min_screen_leftright, max_screen_leftright;
	double size = 10;
	min_screen_updown = -size;
	max_screen_updown = size;
	min_screen_leftright = -size;
	max_screen_leftright = size;

	rx = vp.min_rx + drand()*(vp.max_rx-vp.min_rx);
	ry = vp.min_ry + drand()*(vp.max_ry-vp.min_ry);
	double symmetry_rz;
	if(rz_symmetry_order==0) {
		symmetry_rz = rz = drand()*360.0;
	} else {
		rz = vp.min_rz + drand()*(vp.max_rz-vp.min_rz);
		//rotates by the angle of symmetry a random number of times to get samples from each multiple of the angle of symmetry
		double angle_of_symmetry = 360.0/(double)rz_symmetry_order;
		symmetry_rz = rz + (rand()%rz_symmetry_order)*angle_of_symmetry;
	}

	double min_lx = -5, max_lx = 5; //used to be +/-15
	double min_ly = -5, max_ly = 5; //used to be +/-15
	double min_lz = 1, max_lz = 1.5;
	double lx_samp = min_lx + drand()*(max_lx-min_lx);
	double ly_samp = min_ly + drand()*(max_ly-min_ly);
	double lz_samp = min_lz + drand()*(max_lz-min_lz);

	double xc, yc;
	double screen_updown_samp, screen_leftright_samp;
	set_to(cropped_picture,cv::Vec3b(0,0,0));
	do { //if a sample is out of the field of view, it will return false.  repeat until the sample is in the field of view
		depthc = vp.min_depth + drand()*(vp.max_depth-vp.min_depth);
		screen_updown_samp = min_screen_updown + (max_screen_updown-min_screen_updown)*drand();
		screen_leftright_samp = min_screen_leftright + (max_screen_leftright-min_screen_leftright)*drand();
	}
	while(!m.render(w,false,rx,ry,symmetry_rz,depthc,screen_updown_samp,screen_leftright_samp,lx_samp,ly_samp,lz_samp,ambient_light,
			//output parameters:
			cropped_picture,cropped_depth,xc,yc));
	m.render(w,true,rx,ry,symmetry_rz,depthc,screen_updown_samp,screen_leftright_samp,lx_samp,ly_samp,lz_samp,ambient_light,
				//output parameters:
				cropped_textured_picture,cropped_depth,xc,yc);
}

class enumerated_features {
private:
	enumerated_edge_features ef;
	enumerated_texton_features tf;
	enumerated_depth_features df;
	const viewpoint& vp;
	std::string mesh_file_name;
public:
	enumerated_features(
			const feature_library& features,
			const mesh& m,
			const std::string& name,
			OpenGL_X_window& w,
			double ambient_light,
			bool use_textured_mesh_for_edges,
			const viewpoint& vp,
			int rz_symmetry_order,
			int n_images,
			const std::string& image_output_directory = ""
			) : ef(features.get_edges()), tf(features.get_textons()), vp(vp), mesh_file_name(name) {

		cv::Mat_<cv::Vec3b> picture, textured_picture;
		cv::Mat_<float> depth;
		//find the largest image size when the object is closest to the camera, and enumerate features at twice that resolution

		double xc, yc;
		bool success = m.render(w,true,0,0,0,vp.min_depth+.001,0,0,0,0,1,0.5,picture,depth,xc,yc);
		if(!success) throw std::runtime_error("could not generate a synthetic image at the closest depth.");
		double scale_amount = 1.1;//2.0;
		int image_width = picture.rows*scale_amount;
		double target_object_depth = vp.min_depth/scale_amount;

		ef.initialize(image_width, target_object_depth);
		tf.initialize(image_width, target_object_depth);
		df.initialize(image_width, target_object_depth);

	    for(int j=0;j<n_images;j++) {
	    	check_finished();
	    	double rx, ry, rz;
			double depthc; //the actual depth of the center of the object in the image
			sample_synthetic_image(w,m,vp,rz_symmetry_order,ambient_light,
					//output arguments:
					picture,textured_picture,depth,rx,ry,rz,
					depthc);
			if(use_textured_mesh_for_edges) {
				ef.add_image(textured_picture,depth,rx,ry,rz,depthc,image_output_directory);
			} else {
				ef.add_image(picture,depth,rx,ry,rz,depthc,image_output_directory);
			}
			tf.add_image(textured_picture,depth,rx,ry,rz,depthc,image_output_directory);
			df.add_image(depth,rx,ry,rz,depthc);
		}
//	    if(image_output_directory!="") {
//	    	double x0, x1, y0, y1;
//	    	df.max_bounding_box(x0,x1,y0,y1);
//	    	//scale to image size:
//	    	int cx = df.width()/2;
//	    	int cy = df.height()/2;
//	    	x0 = x0/target_object_depth+(double)cx;
//	    	x1 = x1/target_object_depth+(double)cx;
//	    	y0 = y0/target_object_depth+(double)cy;
//	    	y1 = y1/target_object_depth+(double)cy;
//	    	std::ofstream out((image_output_directory+"/bounding_box.txt").c_str());
//	    	out << "#<x0> <x1> <y0> <y1>" << std::endl;
//	    	out << x0 << " " << x1 << " " << y0 << " " << y1 << std::endl;
//	    }
	}
	void calculate_enumerated_features() {
		ef.calculate_enumerated_features();
		tf.calculate_enumerated_features();
		df.calculate_enumerated_features();
	}
	void sort_enumerated_features() {
		ef.sort_enumerated_features();
		tf.sort_enumerated_features();
		df.sort_enumerated_features();
	}
	void select_features(
			int n_edge_features,  double edge_receptive_field_radius,  double max_edge_position_variance, double min_edge_position_variance,
#ifdef TWO_THRESHOLD_FEATURES
			double high_threshold_log_probability_shift,
#endif
			int n_texton_features,  double texton_receptive_field_radius,  double max_texton_position_variance, double min_texton_position_variance,
			int n_depth_features, double depth_receptive_field_radius, double max_depth_feature_variance, double min_depth_feature_variance,
			//output parameter:
			object& obj
			) const {
		std::vector<visual_part_more_info> selected_edge_features;
		ef.select_features(n_edge_features,edge_receptive_field_radius,
#ifdef TWO_THRESHOLD_FEATURES
				high_threshold_log_probability_shift,
#endif
				max_edge_position_variance,min_edge_position_variance,
				//output parameter:
				selected_edge_features);

		std::vector<visual_part_more_info> selected_texton_features;
		tf.select_features(n_texton_features,texton_receptive_field_radius,
#ifdef TWO_THRESHOLD_FEATURES
				0, //todo: can we use two thresholds for textons?
#endif
				max_texton_position_variance,min_texton_position_variance,
				//output parameter:
				selected_texton_features);

		//add the texton features to the vector of all image plane features.
		//the texton indices come after all the edge direction features.
		int n_edge_directions = ef.get_edges().get_edge_directions().size();
		int n_textons = tf.get_textons().n_clusters();
		std::vector<std::vector<visual_part> > visual_parts(n_edge_directions+n_textons);
		for(std::vector<visual_part_more_info>::const_iterator f=selected_edge_features.begin();f!=selected_edge_features.end();f++) {
			visual_parts[f->visual_feature_index].push_back(f->part);
		}
		for(std::vector<visual_part_more_info>::const_iterator f=selected_texton_features.begin();f!=selected_texton_features.end();f++) {
			visual_parts[n_edge_directions+f->visual_feature_index].push_back(f->part);
		}

		std::vector<depth_part> depth_parts;
		df.select_features(n_depth_features,depth_receptive_field_radius,max_depth_feature_variance,min_depth_feature_variance,
				//output parameter:
				depth_parts);

		obj.name = mesh_file_name;
		obj.library = feature_library(ef.get_edges(),tf.get_textons());
//		df.average_bounding_box(x0,x1,y0,y1);
		obj.views.push_back(view(&obj,vp,visual_parts,depth_parts));
	}
	void show() const {
		ef.show();
//		tf.show(); //todo: add debug images for textons
		df.show();
	}
};

inline bool visual_part_too_close(const visual_part_more_info& f,
		const std::vector<visual_part_more_info> features, float min_dist2) {
	for(std::vector<visual_part_more_info>::const_iterator fj=features.begin();fj!=features.end();fj++) {
		if(fj->visual_feature_index==f.visual_feature_index) {
			int dx = f.x-fj->x;
			int dy = f.y-fj->y;
			float d2 = dx*dx+dy*dy;
			if(d2<min_dist2) return true;
		}
	}
	return false;
}

inline bool depth_part_too_close(const depth_part& f, const std::vector<depth_part> features, float min_dist2) {
	for(std::vector<depth_part>::const_iterator fj=features.begin();fj!=features.end();fj++) {
		int dx = f.x-fj->x;
		int dy = f.y-fj->y;
		float d2 = dx*dx+dy*dy;
		if(d2<min_dist2) return true;
	}
	return false;
}

///Used for running binary search to set the minimum distance between image plane features just right for the particular object view.
class min_visual_feature_dist2_iterator : public std::vector<unsigned int>::const_iterator {
	const std::vector<visual_part_more_info>* features;
	int n_features;
public:
	min_visual_feature_dist2_iterator(
			const std::vector<visual_part_more_info>& features,
			int n_features,
			const std::vector<unsigned int>::const_iterator& it
			) : std::vector<unsigned int>::const_iterator(it), features(&features), n_features(n_features) {}
	min_visual_feature_dist2_iterator() {}
	const min_visual_feature_dist2_iterator& operator*() const {return *this;}
	unsigned int dist2() const {return ((*this)-1).std::vector<unsigned int>::const_iterator::operator*();}
	bool operator<(unsigned int max_visual_part_position_variance) const {
		unsigned int current_feature = 0;
		std::vector<visual_part_more_info> selected_features;
		unsigned int min_visual_feature_dist2 = std::vector<unsigned int>::const_iterator::operator*(); //call parent operator*
		for(int j=0;j<n_features;j++) {
			while(visual_part_too_close((*features)[current_feature],selected_features,min_visual_feature_dist2)) {
				current_feature++;
				if((*features)[current_feature].part.distribution.variance()>max_visual_part_position_variance) return false;
				if(current_feature>=(*features).size()) return false;
			}
			selected_features.push_back((*features)[current_feature]);
		}
		return true;
	}
};

class min_depth_feature_dist_iterator : public std::vector<unsigned int>::const_iterator {
	const std::vector<depth_part>* features;
	int n_features;
public:
	min_depth_feature_dist_iterator(
			const std::vector<depth_part>& features,
			int n_features,
			const std::vector<unsigned int>::const_iterator& it
			) : std::vector<unsigned int>::const_iterator(it), features(&features), n_features(n_features) {}
	min_depth_feature_dist_iterator() {}
	const min_depth_feature_dist_iterator& operator*() const {return *this;}
	unsigned int dist2() const {return ((*this)-1).std::vector<unsigned int>::const_iterator::operator*();}
	bool operator<(float max_depth_variance) const {
		unsigned int current_feature = 0;
		std::vector<depth_part> selected_features;
		unsigned int min_depth_feature_dist = std::vector<unsigned int>::const_iterator::operator*(); //call parent operator*
		for(int j=0;j<n_features;j++) {
			while(depth_part_too_close((*features)[current_feature],selected_features,min_depth_feature_dist)) {
				current_feature++;
				if((*features)[current_feature].distribution.variance()>max_depth_variance) return false;
				if(current_feature>=(*features).size()) return false;
			}
			selected_features.push_back((*features)[current_feature]);
		}
		return true;
	}
};

std::vector<unsigned int> dists(unsigned int max_dx) {
	std::set<unsigned int> dist_set;
	for(unsigned int x=0;x<max_dx;x++) {
		for(unsigned int y=0;y<max_dx;y++) {
			dist_set.insert(x*x+y*y);
		}
	}
	std::vector<unsigned int> dists(dist_set.size());
	std::copy(dist_set.begin(),dist_set.end(),dists.begin());
//	std::sort(dists.begin(),dists.end()); //unnecessary, since sets give sorted values
	return dists;
}

void enumerated_visual_features::select_features(int n_features, double receptive_field_radius,
#ifdef TWO_THRESHOLD_FEATURES
		double high_threshold_log_probability_shift,
#endif
		double max_visual_part_position_variance, double min_visual_part_position_variance,
		//output parameter:
		std::vector<visual_part_more_info>& selected_features) const {
	if(all_enumerated_features.size()==0) return;

	const std::vector<unsigned int> dist = dists(500);
	min_visual_feature_dist2_iterator begin(all_enumerated_features,n_features,dist.begin());
	min_visual_feature_dist2_iterator end(  all_enumerated_features,n_features,dist.end());
	unsigned int min_visual_feature_dist2 = std::lower_bound(begin,end,max_visual_part_position_variance).dist2();

	if(min_visual_feature_dist2==0) min_visual_feature_dist2 = 1;

	int current_feature = 0;
	for(int j=0;j<n_features;j++) {
		check_finished();
		while(visual_part_too_close(all_enumerated_features[current_feature],selected_features,min_visual_feature_dist2)) {
			current_feature++;
			if(current_feature>=(int)all_enumerated_features.size()) return;
		}
		visual_part_more_info f = all_enumerated_features[current_feature];
#ifdef TWO_THRESHOLD_FEATURES
		f = visual_part_more_info(f.visual_feature_index,f.x,f.y,
				visual_part(f.part.distribution,high_threshold_log_probability_shift,  //rebuild f with a new high_threshold_log_probability
						f.part.cx,f.part.cy,f.part.xrx,f.part.xry,f.part.xrz,f.part.yrx,f.part.yry,f.part.yrz));
#endif
		double variance = std::max(min_visual_part_position_variance,(double)f.part.distribution.variance());
		f.part.distribution = normal_distribution(variance,receptive_field_radius);
		selected_features.push_back(f);
	}
}

void enumerated_depth_features::select_features(int n_features, double receptive_field_radius, double max_depth_variance, double min_depth_variance,
		//output parameter:
		std::vector<depth_part>& selected_features) const {
	if(all_enumerated_features.size()==0) return;

	const std::vector<unsigned int> dist = dists(500);
	min_depth_feature_dist_iterator begin(all_enumerated_features,n_features,dist.begin());
	min_depth_feature_dist_iterator end(  all_enumerated_features,n_features,dist.end()  );
	unsigned int min_depth_feature_dist2 = std::lower_bound(begin,end,max_depth_variance).dist2();

	if(min_depth_feature_dist2==0) min_depth_feature_dist2 = 1;

	int current_feature = 0;
	for(int j=0;j<n_features;j++) {
		check_finished();
		while(depth_part_too_close(all_enumerated_features[current_feature],selected_features,min_depth_feature_dist2)) {
			current_feature++;
			if(current_feature>=(int)all_enumerated_features.size()) return;
		}
		depth_part f = all_enumerated_features[current_feature];
		double variance = std::max(min_depth_variance,(double)f.distribution.variance());
		f.distribution = normal_distribution(variance,receptive_field_radius);
		selected_features.push_back(f);
	}
}

void learn_viewpoint(double rx, double ry, double rz, double rx_bin_width, double ry_bin_width, double rz_bin_width, int rz_symmetry_order,
		double min_depth, double max_depth, feature_library& features, const mesh& m, OpenGL_X_window& win, const std::string& mesh_name,
		double ambient_light, int use_textured_mesh_for_edges, int n_training_images, const std::string& image_output_directory, int verbose,
		int n_edge_features,  double edge_receptive_field_radius,  double max_edge_position_variance, double min_edge_position_variance,
#ifdef TWO_THRESHOLD_FEATURES
		double high_threshold_log_probability_shift,
#endif
		int n_texton_features,  double texton_receptive_field_radius,  double max_texton_position_variance, double min_texton_position_variance,
		int n_depth_features, double depth_receptive_field_radius, double max_depth_feature_variance, double min_depth_position_variance,
		//output parameter:
		view& v,
		//debugging:
		int show_debug_images = 0) {
	objrec::viewpoint vp;
	if(rz_symmetry_order==0) {
		vp.min_rz = rz; vp.max_rz = rz+.1;
	} else {
		vp.min_rz = rz - rz_bin_width/2.0; vp.max_rz = rz + rz_bin_width/2.0;
	}
	vp.min_ry = ry - ry_bin_width/2.0; vp.max_ry = ry + ry_bin_width/2.0;
	vp.min_rx = rx - rx_bin_width/2.0; vp.max_rx = rx + rx_bin_width/2.0;
	vp.min_depth = min_depth+m.radius; vp.max_depth = max_depth+m.radius;

	if(verbose!=0) std::cerr << "Gathering statistics from synthetic training data... " << std::flush;

	long long start, end;
    start = objrec::current_time();

	objrec::enumerated_features f(features,m,mesh_name,win,ambient_light,use_textured_mesh_for_edges!=0,
			vp,rz_symmetry_order,n_training_images,image_output_directory);

	end = objrec::current_time();
    if(verbose!=0) std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;

	if(verbose!=0) std::cerr << "Calculating feature parameters... " << std::flush;
    start = objrec::current_time();
    f.calculate_enumerated_features();
    end = objrec::current_time();
	if(verbose!=0) std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;

    if(show_debug_images!=0) {
    	f.show();
    	cv::waitKey(0);
    }

	if(verbose!=0) std::cerr << "Sorting features by variance... " << std::flush;
    start = objrec::current_time();
    f.sort_enumerated_features();
    end = objrec::current_time();
	if(verbose!=0) std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;


	if(verbose!=0) std::cerr << "Selecting features... " << std::flush;
	start = objrec::current_time();

	objrec::object obj;
	obj.object_center_height_above_table = m.origin_height_above_table;
	f.select_features(
			n_edge_features,  edge_receptive_field_radius,  max_edge_position_variance, min_edge_position_variance,
#ifdef TWO_THRESHOLD_FEATURES
			high_threshold_log_probability_shift,
#endif
			n_texton_features,  texton_receptive_field_radius,  max_texton_position_variance, min_texton_position_variance,
			n_depth_features, depth_receptive_field_radius, max_depth_feature_variance, min_depth_position_variance,
			//output parameter:
			obj);

	v = obj.views[0];
	end = objrec::current_time();
    if(verbose!=0) std::cerr << "done. (" << (double)(end-start)/1000000.0 << "s)" << std::endl;
}

} //namespace objrec



#endif /* LEARNING_H_ */
