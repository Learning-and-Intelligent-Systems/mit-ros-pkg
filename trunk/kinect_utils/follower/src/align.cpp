
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <iostream>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/range_image/range_image.h"
#include "pcl/features/range_image_border_extractor.h"
#include "pcl/keypoints/narf_keypoint.h"
#include "pcl/features/narf_descriptor.h"
#include "pcl/common/point_correspondence.h"
#include "pcl/common/poses_from_matches.h"
#include "pcl/common/common_headers.h"
#include "pcl_visualization/pcl_visualizer.h"


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_tools/pcl_utils.h>
#include <nnn/nnn.hpp>
#include <pcl_tools/segfast.hpp>

#include <Eigen/Geometry>

/**
 * extends Narf36 by adding r,g,b information
 **/
struct Narf39{

	pcl::Narf36 narf36;
	float r;
	float g;
	float b;

};


/**
 * takes two points of type Narf36 and returns
 * the Euclidean distance between their descriptors
 **/
float euclideanDistance(pcl::Narf36 p1, pcl::Narf36 p2){

	float sum = 0.0;

	for (int i = 0; i < 36; i++){

		sum += (p2.descriptor[i]-p1.descriptor[i])*(p2.descriptor[i]-p1.descriptor[i]);
	}

	return sqrt(sum);

}

/**
 * takes two points of type Narf39 and returns
 * the Euclidean distance between their descriptors
 **/
float euclideanDistance(Narf39 p1, Narf39 p2){

	float sum = 0.0;

	for (int i = 0; i < 36; i ++){
		sum+= (p2.narf36.descriptor[i]-p1.narf36.descriptor[i])*(p2.narf36.descriptor[i]-p1.narf36.descriptor[i]);
	}

	sum+=(float)((p2.r-p1.r)*(p2.r-p1.r));
	sum+=(float)((p2.g-p1.g)*(p2.g-p1.g));
	sum+=(float)((p2.b-p1.b)*(p2.b-p1.b));

	return sqrt(sum);

}

/**
 * takes a point cloud of type PointXYZ and returns a point cloud containing
 * and and all Narf36 features found from the corresponding RangeImage
 */
pcl::PointCloud<pcl::Narf36> getNarf36FromPointXYZ(pcl::PointCloud<pcl::PointXYZ>& cloudin){
	float angularResolution =   .2f * (M_PI/180.0f); //  used 1.0 for use with kinect, 0.2f for test data
	float maxAngleWidth     = 360.0f * (M_PI/180.0f); // 360.0 degree in rad
	float maxAngleHeight    = 360.0f * (M_PI/180.0f); // 360.0 degree in rad, originally 180.0
	const Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f); //should be same as kinect coordinate if using kinect
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	float noiseLevel=0.0f;
	float minRange = 0.0f;
	int borderSize = 0;
	//from Narf tutorial
	float support_size = 1.0f; //was .05
	bool setUnseenToMaxRange = true;
	bool rotation_invariant=true;

	std::cout<<"before range image"<<std::endl;
	pcl::RangeImage rangeImage1;
	rangeImage1.createFromPointCloud(cloudin, angularResolution, maxAngleWidth, maxAngleHeight,
			sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
	std::cout<<"got range image"<<std::endl;

	//Narf keypoints for cloud 1
	pcl::RangeImageBorderExtractor range_image_border_extractor1;
	pcl::NarfKeypoint narf_keypoint_detector1;
	narf_keypoint_detector1.setRangeImageBorderExtractor(&range_image_border_extractor1);
	narf_keypoint_detector1.setRangeImage(&rangeImage1);
	narf_keypoint_detector1.getParameters().support_size = support_size;

	pcl::PointCloud<int> keypoint_indices1;
	narf_keypoint_detector1.compute(keypoint_indices1);
	std::cout<<"got keypoints"<<std::endl;


	std::vector<int> keypoint_indices3;
	keypoint_indices3.resize(keypoint_indices1.points.size());
	for (unsigned int i=0; i<keypoint_indices1.size(); ++i){ // This step is necessary to get the right vector type
		keypoint_indices3[i]=keypoint_indices1.points[i];
	}std::cout<<"resize"<<std::endl;


	pcl::NarfDescriptor narf_descriptor1(&rangeImage1, &keypoint_indices3);
	narf_descriptor1.getParameters().support_size = support_size;
	narf_descriptor1.getParameters().rotation_invariant = rotation_invariant;
	pcl::PointCloud<pcl::Narf36> narf_descriptors1;
	narf_descriptor1.compute(narf_descriptors1);
	std::cout<<"got narf descriptors"<<std::endl;
	std::cout<<"size is "<<narf_descriptors1.points.size()<<std::endl;
	return narf_descriptors1;
}

/**
 * Take a point cloud of type PointXYZRGB and one of type PointXYZ. These clouds must have the same points.
 * Returns a vector of Narf39 structs that corresponds to the Narf36 objects found from the XYZ cloud
 * combined with corresponding RGB data from the XYZRGB cloud.
 *
 * If the Narf36 x,y,z values do not correspond exactly to values in the point cloud the returned vector
 * may have fewer elements than there exist Narf36 points.
 */
std::vector<Narf39> getNarf39FromPointXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>& cloudin, pcl::PointCloud<pcl::PointXYZ>& xyzcloudin){

	//keypoints index into the range image
	//rangeimage.compute3Dwhatever(keypoints[i]) -> find in point cloud.
	//at that point, since in RL given only the x,y,z of a point, finding it in the point cloud means checking every point
	//might as well just check from the Narf36 xyz's

	std::vector<Narf39> features39;

	//get Narf36 features from xyz cloud with keypoints
	pcl::PointCloud<pcl::Narf36> features36 = getNarf36FromPointXYZ(xyzcloudin);
	std::cout<<"size of features36 list "<<features36.points.size()<<std::endl;

	for(unsigned int i = 0; i < cloudin.points.size(); i++){
		float cx = cloudin.points[i].x;
		float cy = cloudin.points[i].y;
		float cz = cloudin.points[i].z;

		for(unsigned int j = 0; j < features36.points.size(); j++){

			float fx = features36.points[j].x;
			float fy = features36.points[j].y;
			float fz = features36.points[j].z;

			if(abs((int)(fx*100)-(int)(cx*100)) <= 95 && abs(int(fy*100) - int(cy*100)) <= 95 && abs((int)(fz*100)-(int)(cz*100))<= 95){

	//			std::cout<<"matched cloud point "<<i<<" with features36 point "<<j<<std::endl;
				//found match
				Narf39 narf;
				narf.narf36 = features36.points[j];
				narf.r = ((((uint32_t)(cloudin.points[i].rgb)) >> 16)& 0x000000ff);
				narf.g = (((uint32_t)(cloudin.points[i].rgb) >> 8)& 0x000000ff);
				narf.b = (((uint32_t)(cloudin.points[i].rgb)) &0x000000f);

				features39.push_back(narf);

				features36.points.erase(features36.begin()+j);
			}

			break;
		}
		if (features36.size() == 0){
			break;
		}

	}


	std::cout<<"size of features39 list "<<features39.size()<<std::endl;
	return features39;

}


/**
 * takes two vectors of Narf39 structs and returns a PointCorrespondences6DVector
 * containing any pairs (i,j) where i is from vector1 and j from vector2
 * such that the distance between their descriptors is less than max_descriptor_distance
 */
pcl::PointCorrespondences6DVector getCorrespondencesFromNarf39(std::vector<Narf39>& vec1, std::vector<Narf39>& vec2){

	float max_descriptor_distance = 5.0; //was 5.0
	pcl::PointCorrespondences6DVector feature_matches;

	for(unsigned int i = 0; i < vec1.size(); i++){
			/*get point from pointcloud1*/
			Narf39 p1 = vec1[i];

			for(unsigned int j = 0; j < vec2.size(); j++){
				/*get point from pointcloud2*/

				Narf39 p2 = vec2[j];

				float descriptor_distance = euclideanDistance(p1,p2);

				if (descriptor_distance > max_descriptor_distance)
					continue;

				//3D location of p1
				Eigen::Vector3f p1loc(p1.narf36.x,p1.narf36.y,p1.narf36.z);

				//3D location of p2
				Eigen::Vector3f p2loc(p2.narf36.x,p2.narf36.y,p2.narf36.z);

				pcl::PointCorrespondence6D match;
				match.score  = (max_descriptor_distance-descriptor_distance)/max_descriptor_distance;
				match.index1 = i;
				match.index2 = j;
				match.point1 = p1loc;
				match.point2 = p2loc;

				//get transform
				Eigen::Matrix3f p1pose;
				//get Narf36 clouds from PointXYZcloud
				p1pose = Eigen::AngleAxisf(p1.narf36.roll, Eigen::Vector3f::UnitZ())
				*Eigen::AngleAxisf(p1.narf36.pitch, Eigen::Vector3f::UnitY())
				*Eigen::AngleAxisf(p1.narf36.yaw,Eigen::Vector3f::UnitZ());

				Eigen::Affine3f p1Affine = Eigen::Affine3f::Identity();
				p1Affine.rotate(p1pose);
				p1Affine.translate(Eigen::Vector3f(p1.narf36.x,p1.narf36.y,p1.narf36.z));
				//this hopefully will cast to an Affine transform the way I want?

				Eigen::Matrix3f p2pose;
				p2pose = Eigen::AngleAxisf(p2.narf36.roll, Eigen::Vector3f::UnitZ())
				*Eigen::AngleAxisf(p2.narf36.pitch, Eigen::Vector3f::UnitY())
				*Eigen::AngleAxisf(p2.narf36.yaw,Eigen::Vector3f::UnitZ());

				Eigen::Affine3f p2Affine = Eigen::Affine3f::Identity();
				p2Affine.rotate(p2pose);
				p2Affine.translate(Eigen::Vector3f(p2.narf36.x,p2.narf36.y,p2.narf36.z));

				match.transformation = p1Affine.inverse()*p2Affine; //p1->p2
				match.transformation = match.transformation.inverse(); //p2->p1

				feature_matches.push_back(match);

			}

		}

		return feature_matches;

}

/**
 * takes two point clouds of type Narf36 and returns a PointCorrespondences6DVector
 * containing any pairs (i,j) where i is from vector1 and j from vector2
 * such that the distance between their descriptors is less than max_descriptor_distance
 */
pcl::PointCorrespondences6DVector getCorrespondencesFromClouds(pcl::PointCloud<pcl::Narf36>& pointcloud1, pcl::PointCloud<pcl::Narf36>& pointcloud2){

	float max_descriptor_distance = 0.5; //was .05 for use WITHOUT kinect
	pcl::PointCorrespondences6DVector feature_matches;

	for(unsigned int i = 0; i < pointcloud1.points.size(); i++){
		/*get point from pointcloud1*/
		pcl::Narf36 p1 = pointcloud1.points[i];

		for(unsigned int j = 0; j < pointcloud2.points.size(); j++){
			/*get point from pointcloud2*/
			//get Narf36 clouds from PointXYZcloud
			pcl::Narf36 p2 = pointcloud2.points[j];

			float descriptor_distance = euclideanDistance(p1,p2);

			if (descriptor_distance > max_descriptor_distance)
				continue;

			//3D location of p1
			Eigen::Vector3f p1loc(p1.x,p1.y,p1.z);

			//3D location of p2
			Eigen::Vector3f p2loc(p2.x,p2.y,p2.z);

			pcl::PointCorrespondence6D match;
			match.score  = (max_descriptor_distance-descriptor_distance)/max_descriptor_distance;
			match.index1 = i;
			//get Narf36 clouds from PointXYZcloud
			match.index2 = j;
			match.point1 = p1loc;
			match.point2 = p2loc;

			//get transform
			Eigen::Matrix3f p1pose;
			//get Narf36 clouds from PointXYZcloud
			p1pose = Eigen::AngleAxisf(p1.roll, Eigen::Vector3f::UnitZ())
			*Eigen::AngleAxisf(p1.pitch, Eigen::Vector3f::UnitY())
			*Eigen::AngleAxisf(p1.yaw,Eigen::Vector3f::UnitZ());

			Eigen::Affine3f p1Affine = Eigen::Affine3f::Identity();
			p1Affine.rotate(p1pose);
			p1Affine.translate(Eigen::Vector3f(p1.x,p1.y,p1.z));
			//this hopefully will cast to an Affine transform the way I want?

			Eigen::Matrix3f p2pose;
			p2pose = Eigen::AngleAxisf(p2.roll, Eigen::Vector3f::UnitZ())
			*Eigen::AngleAxisf(p2.pitch, Eigen::Vector3f::UnitY())
			*Eigen::AngleAxisf(p2.yaw,Eigen::Vector3f::UnitZ());

			Eigen::Affine3f p2Affine = Eigen::Affine3f::Identity();
			p2Affine.rotate(p2pose);
			p2Affine.translate(Eigen::Vector3f(p2.x,p2.y,p2.z));

			match.transformation = p1Affine.inverse()*p2Affine; //p1->p2
			match.transformation = match.transformation.inverse(); //p2->p1
			feature_matches.push_back(match);

		}

	}

	return feature_matches;

}





class AlignClouds
{

private:
	ros::NodeHandle n_;
	ros::Publisher cloudpub_;
	ros::Publisher cloudpub_prev;
	ros::Publisher cloudpub_merged;
	ros::Publisher cloudpub_narf;
	ros::Publisher cloudpub_narf2;
	ros::Subscriber sub_;
	std::string fixedframe;
	tf::TransformListener tl_;
	//pcl::PointCloud<pcl::Narf36> prev;
	pcl::PointCloud<pcl::PointXYZ> prev;
	pcl::PointCloud<pcl::Narf36>prevNarf;
	std::vector<Narf39> prevNarf39;
	bool hasprev;

public:
	AlignClouds()
	{
		hasprev = false;
		cloudpub_ = n_.advertise<sensor_msgs::PointCloud2> ("cloudout", 0); //I think the 1 says it's in the global frame?
		cloudpub_prev = n_.advertise<sensor_msgs::PointCloud2>("prevcloud",0);
		cloudpub_merged = n_.advertise<sensor_msgs::PointCloud2>("mergedcloud",0);
		cloudpub_narf = n_.advertise<sensor_msgs::PointCloud2>("narfcloud",0);
		cloudpub_narf2 = n_.advertise<sensor_msgs::PointCloud2>("narfcloud2",0);

		sub_=n_.subscribe("/mypts", 1, &AlignClouds::cloudcb, this);

	}


	void cloudcb(const sensor_msgs::PointCloud2 &scan){
		sensor_msgs::PointCloud2 cloud2, prevcloud, mergedcloud, narfcloud, narfcloud2;
		pcl::PointCloud<pcl::PointXYZ> cloudin,cloudin_filtered,transformOut;
		pcl::PointCloud<pcl::PointXYZRGB> rgbcloudin;


		//stuff for Narf
		//this is mostly from the range Image/border Extraction/NARF tutorials
		float angularResolution =   1.0f * (M_PI/180.0f); //   1.0 degree in rad
		float maxAngleWidth     = 360.0f * (M_PI/180.0f); // 360.0 degree in rad
		float maxAngleHeight    = 180.0f * (M_PI/180.0f); // 180.0 degree in rad
		const Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
		pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		float noiseLevel=0.00;
		float minRange = 0.0f;
		int borderSize = 0;
		//from Narf tutorial
		float support_size = .5f;
		//from features tutorialcloud2
		bool setUnseenToMaxRange = true, rotation_invariant=true;

		pcl::fromROSMsg(scan,cloudin);
		pcl::fromROSMsg(scan,rgbcloudin);

		//remove statistical outliers (maybe I shouldn't do this?)
//		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//		sor.setMeanK (50);
//		sor.setStddevMulThresh (1.0);
//		sor.setInputCloud(cloudin.makeShared());
//		sor.filter(cloudin_filtered);
		cloudin_filtered = cloudin;

		if(hasprev == false){
			//get descriptors from this incoming scan and save it without aligning
			//because there's nothing to align it with

			pcl::PointCloud<pcl::Narf36> narf_descriptors1 = getNarf36FromPointXYZ(cloudin_filtered);
			std::vector<Narf39> narf39cloud1 = getNarf39FromPointXYZRGB(rgbcloudin, cloudin_filtered); //this is going to be really slow

			prev = cloudin_filtered;
			prevNarf = narf_descriptors1;
			prevNarf39 = narf39cloud1;

			hasprev = true;

		}

		else{

			pcl::PointCloud<pcl::Narf36> narf_descriptors2 = getNarf36FromPointXYZ(cloudin_filtered);

			std::vector<Narf39> narf39cloud2 = getNarf39FromPointXYZRGB(rgbcloudin, cloudin_filtered);

 			pcl::PointCloud<pcl::PointXYZ> cloud_from_Narf;
			cloud_from_Narf.points.resize(narf_descriptors2.points.size());
			for(unsigned int k = 0; k < cloud_from_Narf.points.size(); k++){
				cloud_from_Narf.points[k].x = narf_descriptors2.points[k].x;
				cloud_from_Narf.points[k].y = narf_descriptors2.points[k].y;
				cloud_from_Narf.points[k].z = narf_descriptors2.points[k].z;
			}


			pcl::PointCloud<pcl::PointXYZ> cloud_from_Narf2;
			cloud_from_Narf2.points.resize(prevNarf.points.size());
			for(unsigned int j = 0; j < cloud_from_Narf2.points.size(); j++){
				cloud_from_Narf2.points[j].x = prevNarf.points[j].x;
				cloud_from_Narf2.points[j].y = prevNarf.points[j].y;
				cloud_from_Narf2.points[j].z = prevNarf.points[j].z;
			}

			//publish these features so we can look at them
					pcl::toROSMsg(cloud_from_Narf,narfcloud);
					narfcloud.header = scan.header;
					cloudpub_narf.publish(narfcloud);

					pcl::toROSMsg(cloud_from_Narf2,narfcloud2);
					narfcloud2.header =scan.header;
					cloudpub_narf2.publish(narfcloud2);

			//align clouds. transforms new point cloud to match old point cloud

//			pcl::transformPointCloud(prev,transformOut,transform);

			pcl::PointCorrespondences6DVector correspondences = getCorrespondencesFromClouds(prevNarf, narf_descriptors2);

			pcl::PosesFromMatches::PoseEstimatesVector pose_estimates;
			pcl::PosesFromMatches pose_estimation;
			//pose_estimation.estimatePosesUsing1Correspondence(correspondences, 1000, pose_estimates);
			//pose_estimation.estimatePosesUsing2Correspondences(correspondences, 1000000, 10000, pose_estimates);
			pose_estimation.estimatePosesUsing3Correspondences(correspondences, 1000000, 10000, pose_estimates);

			pcl::PointCorrespondences6DVector correspondences39 = getCorrespondencesFromNarf39(prevNarf39, narf39cloud2);

			pcl::PosesFromMatches::PoseEstimatesVector pose_estimates39;
			pcl::PosesFromMatches pose_estimation39;
			//pose_estimation39.estimatePosesUsing1Correspondence(correspondences39, 1000, pose_estimates39);
			//pose_estimation39.estimatePosesUsing2Correspondences(correspondences39, 1000000, 10000, pose_estimates39);
			pose_estimation39.estimatePosesUsing3Correspondences(correspondences39, 1000000, 10000, pose_estimates39);

			if (pose_estimates.empty())
			{
				ROS_DEBUG("pose_estimates is empty");
				cerr << "Was not able to find transformation between the scans.";

			}else{
				if(pose_estimates39.empty()){
					cerr<<"Was not able to find transformation between rgb scans.\n";
					//transform by best regular scan

					std::cout<<"transformation: "<<(int)pose_estimates[0].transformation(0,0)<<" "<<(int)pose_estimates[0].transformation(0,1)<<" "<<(int)pose_estimates[0].transformation(0,2)<<" "<<(int)pose_estimates[0].transformation(0,3)<<std::endl;
					std::cout<<" "<<(int)pose_estimates[0].transformation(1,0)<<" "<<(int)pose_estimates[0].transformation(1,1)<<" "<<(int)pose_estimates[0].transformation(1,2)<<" "<<(int)pose_estimates[0].transformation(1,3)<<std::endl;
					std::cout<<" "<<(int)pose_estimates[0].transformation(2,0)<<" "<<(int)pose_estimates[0].transformation(2,1)<<" "<<(int)pose_estimates[0].transformation(2,2)<<" "<<(int)pose_estimates[0].transformation(2,3)<<std::endl;
					std::cout<<" "<<(int)pose_estimates[0].transformation(3,0)<<" "<<(int)pose_estimates[0].transformation(3,1)<<" "<<(int)pose_estimates[0].transformation(3,2)<<" "<<(int)pose_estimates[0].transformation(3,3)<<std::endl;


					pcl::transformPointCloud(cloudin_filtered , transformOut,pose_estimates[0].transformation);

					ROS_DEBUG("pointcloud2 is transformed");

				}else{

					//compare scores, pick best.

					if(pose_estimates[0].score < pose_estimates39[0].score){

					std::cout<<"rgb transformation: "<<(int)pose_estimates39[0].transformation(0,0)<<" "<<(int)pose_estimates39[0].transformation(0,1)<<" "<<(int)pose_estimates39[0].transformation(0,2)<<" "<<(int)pose_estimates39[0].transformation(0,3)<<std::endl;
					std::cout<<" "<<(int)pose_estimates39[0].transformation(1,0)<<" "<<(int)pose_estimates39[0].transformation(1,1)<<" "<<(int)pose_estimates39[0].transformation(1,2)<<" "<<(int)pose_estimates39[0].transformation(1,3)<<std::endl;
					std::cout<<" "<<(int)pose_estimates39[0].transformation(2,0)<<" "<<(int)pose_estimates39[0].transformation(2,1)<<" "<<(int)pose_estimates39[0].transformation(2,2)<<" "<<(int)pose_estimates39[0].transformation(2,3)<<std::endl;
					std::cout<<" "<<(int)pose_estimates39[0].transformation(3,0)<<" "<<(int)pose_estimates39[0].transformation(3,1)<<" "<<(int)pose_estimates39[0].transformation(3,2)<<" "<<(int)pose_estimates39[0].transformation(3,3)<<std::endl;

					pcl::transformPointCloud(cloudin_filtered,transformOut,pose_estimates39[0].transformation);

					}
					else{
						std::cout<<"standard transform is better"<<std::endl;
						pcl::transformPointCloud(cloudin_filtered , transformOut,pose_estimates[0].transformation);
					}
				}

			}


			pcl::toROSMsg(prev,prevcloud); //previous
			prevcloud.header = scan.header;

			pcl::toROSMsg(cloudin_filtered, cloud2); //current
			cloud2.header = scan.header;

			pcl::toROSMsg(transformOut, mergedcloud);  //current cloud transformed
			mergedcloud.header = scan.header;

//			prev = transformOut;
			prev = cloudin_filtered;
			prevNarf = narf_descriptors2;

			//publish prev & transformOut. They should mostly line up?

			cloudpub_.publish(cloud2);
			cloudpub_prev.publish(prevcloud);
			cloudpub_merged.publish(mergedcloud);

		}

	}
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "align_clouds");
	ros::NodeHandle n;
	/****
	 * COMMENT THESE LINES OUT TO USE WITHOUT KINECT
	 */
	AlignClouds alignClouds;
	ros::spin();
	return 0;
	/**
	 *
	 */

	std::cout<<"start"<<std::endl;
	srand(time(NULL));

	bool makerandom = true;
	bool test = false;
	int cloudsize = 1600;

	//create a point cloud for testing
	pcl::PointCloud<pcl::PointXYZ> pointcloud1,pointcloud2;
	pcl::PointCloud<pcl::PointXYZRGB> rgbpointcloud1, rgbpointcloud2;

	//NOTE: actually using two point clouds is superfluous but I don't feel like re-writing
	//the existing functions that take PointXYZ clouds or templating them.
	//this is equivalent to making a pointXYZRGB cloud and extracting a pointXYZ cloud from it

	if(makerandom){ //make random-ish point clouds
		float x,y,z;
		x = 0;
		y = 0;
		z = 0;

		for (int p = 0; p < sqrt(cloudsize); p++){

			for(int q = 0; q < sqrt(cloudsize); q++){

				pcl::PointXYZ point;
				point.x = x;
				point.y = y;
				point.z = z;

				pointcloud1.push_back(point);

				//update x,y,z
				y += ((float)(rand()%20)/100);
				z += ((float)(rand()%20)/100);
			}
			y = 0;
			z = 0;
			x += ((float)(rand()%20)/100);
		}

	}


	else{ //make a rectangle

		for(float i = -2; i < 2; i+=.1f){
			for(float j = -2; j < 2; j +=.1f){
				pcl::PointXYZ point;
				point.x = i; point.y = j/2; point.z = j;
				pointcloud1.push_back(point);
			}
		}
	}

	//add color to make RGB point clouds

	int r,g,b;
	r = g = b = 128;

	for (unsigned int m = 0; m < pointcloud1.points.size(); m++){
		pcl::PointXYZRGB point;
		point.x = pointcloud1.points[m].x;
		point.y = pointcloud1.points[m].y;
		point.z = pointcloud1.points[m].z;

		uint32_t rgb_val;

		rgb_val = (float)(r%256);
		rgb_val = rgb_val << 8;
		rgb_val += (float)(g%256);
		rgb_val = rgb_val << 8;
		rgb_val += (float)(b%256);

		point.rgb = rgb_val;

		rgbpointcloud1.push_back(point);

		//move colors a little
		r += (rand()%10)-5;
		b += (rand()%10)-5;
		g += (rand()%10)-5;

	}

	ROS_DEBUG("cloud 1 created");

	//this creates the transform that will be used to make cloud2
	Eigen::Matrix3f tf;
	tf = Eigen::AngleAxisf(1.0f, Eigen::Vector3f::UnitZ())
	*Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
	*Eigen::AngleAxisf(0,Eigen::Vector3f::UnitZ());

	Eigen::Affine3f p2Affine = Eigen::Affine3f::Identity();
	p2Affine.rotate(tf);
	p2Affine.translate(Eigen::Vector3f(1.0f,1.0f,1.0f));

	std::cout<<"transformation: "<<(int)p2Affine(0,0)<<" "<<(int)p2Affine(0,1)<<" "<<(int)p2Affine(0,2)<<" "<<(int)p2Affine(0,3)<<std::endl;
			std::cout<<" "<<(int)p2Affine(1,0)<<" "<<(int)p2Affine(1,1)<<" "<<(int)p2Affine(1,2)<<" "<<(int)p2Affine(1,3)<<std::endl;
			std::cout<<" "<<(int)p2Affine(2,0)<<" "<<(int)p2Affine(2,1)<<" "<<(int)p2Affine(2,2)<<" "<<(int)p2Affine(2,3)<<std::endl;
			std::cout<<" "<<(int)p2Affine(3,0)<<" "<<(int)p2Affine(3,1)<<" "<<(int)p2Affine(3,2)<<" "<<(int)p2Affine(3,3)<<std::endl;


	pointcloud2.points.resize(pointcloud1.points.size());
	for(unsigned int k = 0; k < pointcloud1.points.size(); k++){
		pointcloud2.points[k].x = pointcloud1.points[k].x;
		pointcloud2.points[k].y = pointcloud1.points[k].y;
		pointcloud2.points[k].z = pointcloud1.points[k].z;
	}

	//transform pointcloud1 into pointcloud2
	pcl::transformPointCloud(pointcloud1,pointcloud2,p2Affine);

	ROS_DEBUG("cloud 2 created from transformed cloud 1");
	std::cout<<"cloud 2 created"<<std::endl;

	//transform the RGB point cloud the same way
	pcl::transformPointCloud(rgbpointcloud1,rgbpointcloud2,p2Affine);

	sensor_msgs::PointCloud2 cloud_2msg;
	pcl::toROSMsg(pointcloud2, cloud_2msg);
	cloud_2msg.header.frame_id="/";

	sensor_msgs::PointCloud2 rgbcloud_2msg;
	pcl::toROSMsg(rgbpointcloud2, rgbcloud_2msg);
	rgbcloud_2msg.header.frame_id="/";

	//get Narf36s from each cloud
	pcl::PointCloud<pcl::Narf36> narfcloud1 = getNarf36FromPointXYZ(pointcloud1);
	std::cout<<"got narf 1"<<std::endl;

	pcl::PointCloud<pcl::Narf36> narfcloud2 = getNarf36FromPointXYZ(pointcloud2);
	std::cout<<"got narf 2"<<std::endl;

	//get Narf39s from each cloud
	std::vector<Narf39> narf39cloud1 = getNarf39FromPointXYZRGB(rgbpointcloud1, pointcloud1);
	std::vector<Narf39> narf39cloud2 = getNarf39FromPointXYZRGB(rgbpointcloud2, pointcloud2);

	/*****************
	 * ***
	 * TEST DATA
	 * creates testcount NARF points in cloud 1 and replicates them in cloud 2 to ensure correspondences.
	 * ***
	 * ****************
	 */

	if(test){
		std::cout<<"start test data"<<std::endl;

		int testcount = 10;
		narfcloud1.points.resize(testcount);
		narfcloud2.points.resize(testcount);
		narf39cloud1.resize(testcount);
		narf39cloud2.resize(testcount);

		std::cout<<"resize"<<std::endl;
		for (int t = 0; t < testcount; t++){

			pcl::Narf36 narf;
			Narf39 narf39;
			int index = (int)(rand()%pointcloud1.points.size());
			narf.x = pointcloud1.points[index].x;
			narf.y = pointcloud1.points[index].y;
			narf.z = pointcloud1.points[index].z;
			narf.roll = 0.0f;
			narf.pitch = 0.0f;
			narf.yaw = 0.0f;
			for (int d = 0; d < 36; d++){
				narf.descriptor[d] = (float)(rand()%100/10);
			}

			narfcloud1.points[t] = narf;

			narf39.narf36 = narf;
			narf39.r = (((uint32_t)(rgbpointcloud1.points[index].rgb) >> 16)& 0x000000ff);
			narf39.g = (((uint32_t)(rgbpointcloud1.points[index].rgb) >> 8)& 0x000000ff);
			narf39.b = (((uint32_t)(rgbpointcloud1.points[index].rgb))& 0x000000ff);

			narf39cloud1[t] = narf39;

		}


		narfcloud2 = narfcloud1;


		Eigen::Affine3f testrotate = Eigen::Affine3f::Identity();
		testrotate.rotate(tf);
		testrotate.translate(Eigen::Vector3f(1,1,1));

		pcl::transformPointCloud(narfcloud2, narfcloud2, testrotate);

		std::cout<<"test cloud 2 transformed"<<std::endl;

		narf39cloud2 = narf39cloud1;

		for(int n = 0; n < testcount; n++){
			narf39cloud2[n].narf36 = narfcloud2.points[n];

		}

		std::cout<<"have test clouds"<<std::endl;
	}
	/*********************
	 * ***
	 * END TEST DATA.
	 * ***
	 */

	//get XYZ clouds from the Narf points so we can display them

	pcl::PointCloud<pcl::PointXYZ> cloud1_from_Narf;

	pcl::PointCloud<pcl::PointXYZ> cloud2_from_Narf;

	cloud1_from_Narf.points.resize(narfcloud1.points.size());
		for(unsigned int k = 0; k < cloud1_from_Narf.points.size(); k++){
			cloud1_from_Narf.points[k].x = narfcloud1.points[k].x;
			cloud1_from_Narf.points[k].y = narfcloud1.points[k].y;
			cloud1_from_Narf.points[k].z = narfcloud1.points[k].z;
		}

	cloud2_from_Narf.points.resize(narfcloud2.points.size());
	for(unsigned int k = 0; k < cloud2_from_Narf.points.size(); k++){
		cloud2_from_Narf.points[k].x = narfcloud2.points[k].x;
		cloud2_from_Narf.points[k].y = narfcloud2.points[k].y;
		cloud2_from_Narf.points[k].z = narfcloud2.points[k].z;
	}

	ROS_DEBUG("got Narf36 clouds from point clouds");


	//run getPosesFromClouds on Narf36 clouds
	pcl::PointCorrespondences6DVector correspondences = getCorrespondencesFromClouds(narfcloud1, narfcloud2);
	std::cout<<"correspondences vector has "<<correspondences.size()<<" matches"<<std::endl;

	//get poses from Narf39 clouds
	pcl::PointCorrespondences6DVector correspondences39 = getCorrespondencesFromNarf39(narf39cloud1, narf39cloud2);
	std::cout<<"RGB correspondences vector has "<<correspondences39.size()<<" matches"<<std::endl;


	//run posesfromncorrespondences code from perception tutorial example thingy
	pcl::PosesFromMatches::PoseEstimatesVector pose_estimates;
	pcl::PosesFromMatches pose_estimation;
	pose_estimation.estimatePosesUsing1Correspondence(correspondences, 1000, pose_estimates);
	//pose_estimation.estimatePosesUsing2Correspondences(correspondences, 1000000, 10000, pose_estimates);
	//pose_estimation.estimatePosesUsing3Correspondences(correspondences, 1000000, 10000, pose_estimates);


	//run posesfromcorrespondences code on narf39 for rgb matches
	pcl::PosesFromMatches::PoseEstimatesVector pose_estimates39;
	pcl::PosesFromMatches pose_estimation39;
	pose_estimation39.estimatePosesUsing1Correspondence(correspondences39, 1000, pose_estimates39);
	//pose_estimation39.estimatePosesUsing2Correspondences(correspondences39, 1000000, 10000, pose_estimates39);
	//pose_estimation39.estimatePosesUsing3Correspondences(correspondences39, 1000000, 10000, pose_estimates39);


	if (pose_estimates.empty())
	{
		ROS_DEBUG("pose_estimates is empty");
		cerr << "Was not able to find transformation between the scans.\n";

	}else{
		//transform pc2 by best estimate

		std::cout<<"transformation: "<<(int)pose_estimates[0].transformation(0,0)<<" "<<(int)pose_estimates[0].transformation(0,1)<<" "<<(int)pose_estimates[0].transformation(0,2)<<" "<<(int)pose_estimates[0].transformation(0,3)<<std::endl;
		std::cout<<" "<<(int)pose_estimates[0].transformation(1,0)<<" "<<(int)pose_estimates[0].transformation(1,1)<<" "<<(int)pose_estimates[0].transformation(1,2)<<" "<<(int)pose_estimates[0].transformation(1,3)<<std::endl;
		std::cout<<" "<<(int)pose_estimates[0].transformation(2,0)<<" "<<(int)pose_estimates[0].transformation(2,1)<<" "<<(int)pose_estimates[0].transformation(2,2)<<" "<<(int)pose_estimates[0].transformation(2,3)<<std::endl;
		std::cout<<" "<<(int)pose_estimates[0].transformation(3,0)<<" "<<(int)pose_estimates[0].transformation(3,1)<<" "<<(int)pose_estimates[0].transformation(3,2)<<" "<<(int)pose_estimates[0].transformation(3,3)<<std::endl;

		std::cout<<"transform score: "<<pose_estimates[0].score<<std::endl;
		pcl::transformPointCloud(pointcloud2,pointcloud2,pose_estimates[0].transformation);

		ROS_DEBUG("pointcloud2 is transformed");
	}
	if(pose_estimates39.empty()){
		cerr<<"Was not able to find transformation between rgb scans.\n";
	}else{

		std::cout<<"rgb transformation: "<<(int)pose_estimates39[0].transformation(0,0)<<" "<<(int)pose_estimates39[0].transformation(0,1)<<" "<<(int)pose_estimates39[0].transformation(0,2)<<" "<<(int)pose_estimates39[0].transformation(0,3)<<std::endl;
		std::cout<<" "<<(int)pose_estimates39[0].transformation(1,0)<<" "<<(int)pose_estimates39[0].transformation(1,1)<<" "<<(int)pose_estimates39[0].transformation(1,2)<<" "<<(int)pose_estimates39[0].transformation(1,3)<<std::endl;
		std::cout<<" "<<(int)pose_estimates39[0].transformation(2,0)<<" "<<(int)pose_estimates39[0].transformation(2,1)<<" "<<(int)pose_estimates39[0].transformation(2,2)<<" "<<(int)pose_estimates39[0].transformation(2,3)<<std::endl;
		std::cout<<" "<<(int)pose_estimates39[0].transformation(3,0)<<" "<<(int)pose_estimates39[0].transformation(3,1)<<" "<<(int)pose_estimates39[0].transformation(3,2)<<" "<<(int)pose_estimates39[0].transformation(3,3)<<std::endl;
		std::cout<<"rgb transform score: "<<pose_estimates39[0].score<<std::endl;
		pcl::transformPointCloud(rgbpointcloud2,rgbpointcloud2,pose_estimates39[0].transformation);

	}

//publish point clouds (original & transformed*best guess)/display in visualizer
//	pcl_visualization::PCLVisualizer viewer("3D Viewer");
//	viewer.addCoordinateSystem(1.0f);
//	viewer.addPointCloud(pointcloud1, "original point cloud");
//	viewer.addPointCloud(pointcloud2, "transformed point cloud");
//
//	ROS_DEBUG("viewer is initialized");
//	viewer.spin();

//publish point clouds as ROS messages for Rviz


	ros::Publisher cloudpub_origin = n.advertise<sensor_msgs::PointCloud2> ("cloud_origin", 100);
	ros::Publisher cloudpub_cloud2 = n.advertise<sensor_msgs::PointCloud2> ("cloud_2", 100);
	ros::Publisher cloudpub_origin_narf = n.advertise<sensor_msgs::PointCloud2> ("cloud_origin_narf", 100);
	ros::Publisher cloudpub_cloud2_narf = n.advertise<sensor_msgs::PointCloud2> ("cloud2_narf", 100);
	ros::Publisher cloudpub_transformed = n.advertise<sensor_msgs::PointCloud2> ("cloud2_transformed", 100);
	ros::Publisher cloudpub_rgb1 = n.advertise<sensor_msgs::PointCloud2> ("rgbcloud",100);
	ros::Publisher cloudpub_rgb2 = n.advertise<sensor_msgs::PointCloud2>("rgbcloud2", 100);
	ros::Publisher cloudpub_rgb2_tf = n.advertise<sensor_msgs::PointCloud2>("rgbcloud2_transformed",100);

	std::cout<<"publisher"<<std::endl;

	sensor_msgs::PointCloud2 pointcloud1_msg,pointcloud2_msg, cloud1_from_Narf_msg,cloud2_from_Narf_msg,rgbcloud_1msg,rgbcloud_2_tfmsg;


	pcl::toROSMsg(pointcloud1, pointcloud1_msg);
	pointcloud1_msg.header.frame_id="/";
	pcl::toROSMsg(pointcloud2, pointcloud2_msg);
	pointcloud2_msg.header.frame_id="/";
	pcl::toROSMsg(cloud1_from_Narf, cloud1_from_Narf_msg);
	cloud1_from_Narf_msg.header.frame_id="/";
	pcl::toROSMsg(cloud2_from_Narf, cloud2_from_Narf_msg);
	cloud2_from_Narf_msg.header.frame_id="/";
	pcl::toROSMsg(rgbpointcloud1, rgbcloud_1msg);
	rgbcloud_1msg.header.frame_id="/";
	pcl::toROSMsg(rgbpointcloud2,rgbcloud_2_tfmsg);
	rgbcloud_2_tfmsg.header.frame_id="/";

	std::cout<<"messages"<<std::endl;

	ros::Rate loop_rate(3);
	while (ros::ok()){
		std::cout<<"publishing"<<std::endl;
		cloudpub_origin.publish(pointcloud1_msg);
		cloudpub_cloud2.publish(cloud_2msg);
		cloudpub_origin_narf.publish(cloud1_from_Narf_msg);
		cloudpub_cloud2_narf.publish(cloud2_from_Narf_msg);
		cloudpub_transformed.publish(pointcloud2_msg);
		cloudpub_rgb1.publish(rgbcloud_1msg);
		cloudpub_rgb2.publish(rgbcloud_2msg);
		cloudpub_rgb2_tf.publish(rgbcloud_2_tfmsg);
		ros::spinOnce();
		loop_rate.sleep();
	}

	std::cout<<"published messages"<<std::endl;

	return 0;

}
