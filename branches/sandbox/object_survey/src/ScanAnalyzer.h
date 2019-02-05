/*
 * ScanAnalyzer.h
 *
 *  Created on: Oct 22, 2010
 *      Author: garratt
 */

#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "rosbag/message_instance.h"
#include <boost/foreach.hpp>

#include <object_survey/SurveyScan.h>
#include <object_survey/ScanAnalysis.h>
#include <object_survey/CameraScanAnalysis.h>
#include <sensor_msgs/PointCloud2.h>
#include <mapping_msgs/PolygonalMap.h>

#include <iostream>
#include <vector>
#include <fstream>

//#include <eigen_conversions/eigen_msg.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include "pcl_tf/transforms.h"
#include "siftviewer/ProcessImageCluster.h"

#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/fpfh.h"
#include "pcl/registration/registration.h"
#include "pcl/registration/icp.h"
#include "pcl/registration/icp_nl.h"
#include "pcl/registration/ia_ransac.h"
//#include <furniture_ops/pcl_helpers.hpp>

#include "pcl_tools/pcl_utils.h"
template <typename type1,typename type2 >
void cp3dPt(type1 &dest, type2 &src ){
   dest.x=src.x;
   dest.y=src.y;
   dest.z=src.z;
}

typedef pcl::PointCloud<pcl::PointXYZINormal> NCloud;
typedef pcl::PointXYZINormal NPoint;


#include "cloud_to_mesh.h"




//tf::Transform tfFromEigen(Eigen3::Matrix4f trans){
// btMatrix3x3 btm;
// btm.setValue(trans(0,0),trans(0,1),trans(0,2),
//            trans(1,0),trans(1,1),trans(1,2),
//            trans(2,0),trans(2,1),trans(2,2));
// btTransform ret;
// ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
// ret.setBasis(btm);
// return ret;
//}
//
//
//
//
//
////removes the yaw, pitch and z component from the transform
//Eigen3::Matrix4f projectTo2D(Eigen3::Matrix4f etrans){
//   tf::Transform tftrans=tfFromEigen(etrans);
//   tftrans.setRotation(tf::createQuaternionFromYaw(tf::getYaw(tftrans.getRotation())));
//   Eigen3::Matrix4f out;
//   pcl::transformAsMatrix(tftrans,out);
//   out(2,3)=0.0;  //remove z component
//   return out;
//}
//
//double getYaw(Eigen3::Matrix4f etrans){
//   tf::Transform tftrans=tfFromEigen(etrans);
//   return tf::getYaw(tftrans.getRotation());
//}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Decompose a Point Cloud into clusters based on the Euclidean distance between points. Uses Hierarchical Clustering, making it faster
  * \param cloud the point cloud message
  * \param cluster_tol the spatial cluster tolerance as a measure in L2 Euclidean space
  * \param cclusters the resultant clusters containing point indices (as a vector of vector of ints)
  * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
  */
template <typename PointT>
void extractEuclideanClustersFast2(pcl::PointCloud<PointT> &cloud, std::vector<std::vector<int> > &clusters, double cluster_tol=.2, int min_pts_per_cluster=1){
   bool globalverbose=true;
   double smaller_tol = cluster_tol/2.3;
   pcl::KdTreeFLANN<PointT> tree,tree2;
   tree.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud));
   vector<int> minitree(cloud.points.size(),-1);
   vector<int> heads,indices;
   vector<float> dists;

   //First 'downsample' the points into clusters slightly less than half the cluster tolerance
   //minitree keeps track of who each point belongs to (which is an index of the 'head' array)
   for(uint i=0; i<cloud.points.size();i++){
     if(minitree[i]==-1){    //if no one has claimed this point, make it a head
        heads.push_back(i);
        if(!tree.radiusSearch(cloud,i,smaller_tol,indices,dists))  //find all the points close to this point
           ROS_WARN("radius search failed!");
        for(uint j=0;j<indices.size();j++){
           minitree[indices[j]]=heads.size()-1; //assign these points to the current head.
                                       // this overwrites previous claims, but it's ok
        }
     }
   }

   if(globalverbose)
      ROS_INFO("Found %d heads.  Now to cluster them: ",heads.size());
   //now, we have much fewer points to cluster, but since the initial downsampling was less than
   //half the cluster tolerance,we are guaranteed to find all the points within the tolerance
   //(except if  [ clustertol < distance between heads < cluster_tol+ smaller cluster tol] AND the head closest points in the two heads are < cluster_tol
   //The next code block deals with that...
   tree2.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud),boost::make_shared<std::vector<int> >  (heads));
   int searching,currenthead;
   //heads2 is the cluster id.  we now search to make sure we check all the points in our cluster
   //minitree2 corresponds to the heads array, so the points in minitree2 are at heads[i]
   std::vector<int> heads2, minitree2(heads.size(),-1);
   std::list<int> tosearch;
   for(uint i=0; i<minitree2.size();i++){
     if(minitree2[i]==-1){
        heads2.push_back(heads[i]);
        tosearch.push_back(i);
        currenthead=heads2.size()-1; //references an index in heads2
        minitree2[i]=currenthead;
        while(tosearch.size()>0){
           searching=tosearch.front();
           tosearch.pop_front();
           if(!tree2.radiusSearch(cloud.points[heads[searching]],cluster_tol,indices,dists))
              cout<<"radius search failed!"<<endl;
           for(uint j=0;j<indices.size();j++)
              if(minitree2[indices[j]]==-1){//found untouched point (which means this root touched it)
                 minitree2[indices[j]]=currenthead; //claim it
                 tosearch.push_back(indices[j]);   //add it to list of points to search
              }
        }
     }

   }
   clusters.resize(heads2.size());
   //now, for each point in the cloud find its head --> then the head it clustered to. that is its cluster id!
   for(uint j=0;j<minitree.size();j++){
     clusters[minitree2[minitree[j]]].push_back(j);
   }

   if(globalverbose)
      ROS_INFO("Found %d clusters.  now to find if they any are the same: ",heads2.size());

   //now we need to check to see if any of the heads that were NOT clustered together are closer than cluster_tol+smaller_tol.
   //This covers the exception noted in the code block above
   //this is where an adversary could really kill this algorithm, since this check could be polynomial in cloud size. in real circumstances, it is very quick.
   vector<int> indices1;
   vector<float> dists1;
   double distthresh=cluster_tol*cluster_tol; //radius search gives squared distances...
   double automergethresh=cluster_tol*cluster_tol/4.0; //pts are always in same clustr if they are both less than half the tolerance away from the same point
   timeval t1;
   double time1,time2;
   bool verbose=false;
   for(uint i=0; i<minitree2.size();i++){ //for every head
      verbose=false;
//   if(i==1452 || i==1508 || i==1509) verbose=true;
//   else verbose=false;
     if(verbose) cout<<"head "<<i<<"  in cluster "<<minitree2[i]<<std::endl;
       tree2.radiusSearch(cloud.points[heads[i]],cluster_tol+2*smaller_tol,indices,dists);
     if(verbose) {
       cout<<"  radius search found: "<<indices.size()<<"  matches"<<std::endl;
//     for(uint rind=0;rind<indices.size();rind++)
//        cout<<" -> "<<indices[rind]<<" in cluster "<<minitree2[indices[rind]]<<std::endl;

     }
//      tree2.radiusSearch(cloud.points[heads[i]],cluster_tol+smaller_tol,indices,dists);
      for(uint j=0;j<indices.size();j++){ //for every head that is close to this head
        if(verbose)
          cout<<" -> "<<indices[j]<<" in cluster "<<minitree2[indices[j]]<<std::endl;
         if(minitree2[indices[j]] != minitree2[i]){//if the two heads are not in the same cluster (and only check each combo once)
            t1=g_tick();
            if(verbose)
                  ROS_INFO("head %d (in cluster %d) is %f from head %d (in cluster %d). Checking if the two heads share points.",
               i,minitree2[i],sqrt(dists[j]),indices[j],minitree2[indices[j]]);
            //now we have to find if there is a point between head a and head b that is < cluster_tol from both
            //our best chance of finding it is to start searching at a point halfway between them
            PointT heada=cloud.points[heads[i]], headb=cloud.points[heads[indices[j]]];
            PointT inbetween;
            inbetween.x=(heada.x+headb.x)/2.0;
            inbetween.y=(heada.y+headb.y)/2.0;
            inbetween.z=(heada.z+headb.z)/2.0;
            //the farthest 2 points could be from the centroid of the two heads and still be less than
            //cluster_tol apart is: cluster_tol + smaller_tol -dist_between_heads/2)
//            std::cout<<cluster_tol+smaller_tol-sqrt(dists[j])<<std::endl;
//            double searchrad=cluster_tol+smaller_tol-sqrt(dists[j]);
//            indices1.resize(0);

            if(!tree.radiusSearch(inbetween,cluster_tol,indices1,dists1)){ //search in the full tree
               if(globalverbose)
                  ROS_ERROR("Radius Search Failed");
               //nothing close to either cluster. these clusters are separate, so don't consider
               continue; //go to next j
            }
            time1=g_tock(t1);
            t1=g_tick();
            //must match all points that return against each other
            bool shouldmerge=false;
            int mergefrom,mergeinto;
            for(int k=0;k<indices1.size()-1;k++){
                for(uint m=k+1;m<indices1.size();m++){
                  if(minitree2[minitree[indices1[k]]] != minitree2[minitree[indices1[m]]]) //if different clusters
                     if((dists1[k]<automergethresh && dists1[m]<automergethresh) ||
                     (squaredEuclideanDistance(cloud.points[indices1[k]],cloud.points[indices1[m]] ) < distthresh)){
                        mergefrom=minitree2[minitree[indices1[k]]];
                        mergeinto=minitree2[minitree[indices1[m]]];
                        if((mergefrom !=minitree2[i] && mergefrom !=minitree2[indices[j]]) || (mergeinto !=minitree2[i] && mergeinto !=minitree2[indices[j]])){
                          if(globalverbose)
                             ROS_WARN("point %d in cluster %d is %f from point %d in cluster %d, but these points won't be merged. since we are concidering %d and %d.",
                                   indices1[k],minitree2[minitree[indices1[k]]],
                                   euclideanDistance(cloud.points[indices1[k]],cloud.points[indices1[m]]),
                                   indices1[m],minitree2[minitree[indices1[m]]],minitree2[i],minitree2[indices[j]]);
                        }
                        else{
                           if(globalverbose)
                              ROS_WARN("point %d in cluster %d is %f from point %d in cluster %d",
                                    indices1[k],minitree2[minitree[indices1[k]]],
                                    euclideanDistance(cloud.points[indices1[k]],cloud.points[indices1[m]]),
                                    indices1[m],minitree2[minitree[indices1[m]]]);
                           shouldmerge=true;
                           break;  //TODO: shouldn't actually break, but rather see if multiple clusters should merge
                        }
                     }

                }

//             if(verbose){
//             cout<<"    -> dist: ("<<sqrt(dists1[k])<<") "<<indices1[k]<<" in cluster "<<minitree2[minitree[indices1[k]]];
//             cout<<"  "<<euclideanDistance(cloud.points[indices1[k]],heada )<<"  from "<<heads[i];
//             cout<<"  "<<euclideanDistance(cloud.points[indices1[k]],headb )<<"  from "<<heads[indices[j]]<<std::endl;
//             }
               if(shouldmerge){

                  //there is a point that these two clusters share -> they should be merged
                  if(globalverbose)
                     ROS_INFO("head %d (in cluster %d) shares a point with head %d (in cluster %d). Merging cluster %d (%d pts) into cluster %d (%d pts)",
                       i,minitree2[i],indices[j],minitree2[indices[j]],minitree2[indices[j]],clusters[minitree2[indices[j]]].size(), minitree2[i],  clusters[minitree2[i]].size());
                  //rename all heads in cluster b to cluster a
                  int clusterb=minitree2[indices[j]];
                  for(uint m=0;m<minitree2.size();m++){
                     if(minitree2[m]==clusterb) minitree2[m]=minitree2[i];
                  }
                  break;
               } //if the point is shared

//               cout<<"       "<<k<<"  "<<g_tock(t1)<<std::endl;
            } //for every point close to both heads
            time2=g_tock(t1);
//            ROS_INFO("    radius searching took %f for %d pts.  the for loops took %f",time1,indices1.size(),time2);

         } //if a head is close, and from a different cluster
      }//for every nearby head
   }  //for every head

   clusters.clear();
   clusters.resize(heads2.size());
   //now, for each point in the cloud find its head --> then the head it clustered to. that is its cluster id!
   for(uint j=0;j<minitree.size();j++){
     clusters[minitree2[minitree[j]]].push_back(j);
   }
   //erase the deleted clusters, and the ones under the minimum size
   std::vector<int> deletedclusters;
   for(int i=clusters.size()-1;i>=0; i--)
      if(clusters[i].size() < min_pts_per_cluster){
         clusters.erase(clusters.begin()+i);
         deletedclusters.push_back(i);
//         if(globalverbose)
//            cout<<"erasing "<<i<<endl;
      }

//   int clusterofinterest=22;
//   int p1=clusters[clusterofinterest][0];
//   //get original cluster:
//   int offset=0;
//   for(uint j=0;j<deletedclusters.size();j++)
//      if(deletedclusters[j]<=clusterofinterest)
//         offset++;
//   cout<<"Cluster "<<clusterofinterest<<" was originally "<<clusterofinterest+offset<<endl;
//   clusterofinterest+=offset;
//   cout<<"cluster size: "<<clusters[clusterofinterest].size()<<endl;
//   cout<<"heads:  ";
//   for(uint j=0;j<minitree2.size();j++){
//      if(minitree2[j]==clusterofinterest)
//         cout<<j<<"  ";
//   }
//   cout<<endl;
//   cout<<"cluster 22, pt 0: "<<p1<<" head: "<<minitree[p1]<<" which is pt "<<heads[minitree[p1]]<<std::endl;
//   int p2=clusters[43][0];
//   cout<<"cluster 43, pt 0: "<<p2<<" head: "<<minitree[p2]<<" which is pt "<<heads[minitree[p2]]<<std::endl;
//   cout<<"distance between heads: "<<ptdist(cloud.points[heads[minitree[p1]]], cloud.points[heads[minitree[p2]]])<<std::endl;




}


#ifndef SCANANALYZER_H_
#define SCANANALYZER_H_

class ScanAnalyzer {
	std::vector<object_survey::ScanAnalysis> scans;
	std::vector<posedetection_msgs::Feature0D> features;
	std::vector<pcl::PointCloud<pcl::PointXYZINormal> > filtered_clouds;
	std::vector<pcl::PointCloud<pcl::PointXYZINormal> > normal_clouds;
	std::vector<pcl::PointCloud<pcl::PointXYZINormal> > aligned_clouds;
   std::vector<pcl::PointCloud<pcl::PointXYZINormal> > clusters;
   std::vector<Eigen3::Matrix4f > alignments;  //this is the alignment between cloud [i] and cloud [i+1], wrapping around
//	std::vector<std::vector<Eigen3::Matrix4f > > transforms;
	ros::NodeHandle nh_;
	ros::Publisher pub_map,pub_cloud;
	ros::ServiceClient imagesifter;
	NCloud aligned;

	boost::mutex data_mutex;
   boost::mutex print_mutex;

	void writePLY(pcl::PointCloud<pcl::PointXYZINormal> &cloud, std::string filename){
	   std::ofstream outf;
	   outf.open(filename.c_str(),std::ios::out);
	   outf<<"ply\n"<<"format ascii 1.0\n"<<"element vertex "<<cloud.points.size()<<endl;
	   outf<<"property float x\nproperty float y\nproperty float z\n";
	   outf<<"property float nx\nproperty float ny\nproperty float nz\n";
	   outf<<"element face 0\nproperty list uchar int vertex_indices\nend_header\n";
	   for (int i = 0; i < cloud.points.size(); ++i) {
	      outf<<cloud.points[i].x<<" "<<cloud.points[i].y<<" "<<cloud.points[i].z;
	      outf<<" "<<cloud.points[i].normal[0]<<" "<<cloud.points[i].normal[1]<<" "<<cloud.points[i].normal[2]<<endl;
	   }
	   outf.close();

	}


   void writeMesh(pcl::PointCloud<pcl::PointXYZINormal> &cloud, std::string filename_base, std::string object_name){
      mesher::MeshParams params;
      params.object_name=object_name;
      params.texture="Gazebo/Terrain";
      params.Depth=7;
      Eigen3::Vector4f centroid;
      pcl::compute3DCentroid(cloud,centroid);
      params.origin[0]=-centroid(0);
      params.origin[1]=-centroid(1);

      mesher::cloud2Mesh(cloud,filename_base,params);
   }

//do we want this to always transform everything?
   void updateAlignedTransforms(){
     tf::Transform orig, trans, final;
     for(uint i=0;i<scans.size();i++){
       trans = tfFromEigen(alignments[i]);
       tf::transformMsgToTF(scans[i].base_transform.transform,orig);
       final=trans*orig;
       tf::transformTFToMsg(final,scans[i].base_aligned.transform);
       tf::transformMsgToTF(scans[i].laser_transform.transform,orig);
       final=trans*orig;
       tf::transformTFToMsg(final,scans[i].laser_aligned.transform);
       for(int j=0;j<scans[i].camera_scans.size();j++) {
	 tf::transformTFToMsg(final,scans[i].camera_scans[j].base_delta);

	 tf::Transform orig2;
	 tf::transformMsgToTF(scans[i].camera_scans[j].narrow_stereo_transform.transform, orig);  // narrow_stereo_optical -> odom_combined
	 tf::transformMsgToTF(scans[i].base_transform.transform, orig2);                          // odom_combined -> base_link (original)
	 tf::transformMsgToTF(scans[i].camera_scans[j].narrow_stereo_transform.transform, orig);  // narrow_stereo_optical -> odom_combined

       }
       //TODO: update camera transform?
     }


   }


	//void setIdentity(geometry_msgs::TransformStamped &msg) {
	void setIdentity(geometry_msgs::Transform &msg) {
	  tf::Transform I;
	  I.setIdentity();
	  //tf::StampedTransform identity;
	  //identity.setData(I);
	  //tf::transformStampedTFToMsg(identity,msg);
	  tf::transformTFToMsg(I,msg);
	}

	void addSurveyScan(const object_survey::SurveyScan &scan){
		scans.push_back(object_survey::ScanAnalysis());
		scans.back().base_transform=scan.base_transform;
		scans.back().cloud_vp=scan.cloud;
		scans.back().laser_transform=scan.laser_transform;
		scans.back().object_pose=scan.object_pose;
		scans.back().camera_scans = std::vector<object_survey::CameraScanAnalysis>(scan.camera_scans.size());
		for(int j=0;j<scan.camera_scans.size();j++) {
#define SCAN_COPY(x) scans.back().camera_scans[j].x = scan.camera_scans[j].x
		  SCAN_COPY(focus_point);

		  SCAN_COPY(narrow_stereo_right_image);
		  SCAN_COPY(narrow_stereo_right_caminfo);
		  SCAN_COPY(narrow_stereo_left_image);
		  SCAN_COPY(narrow_stereo_left_caminfo);
		  SCAN_COPY(narrow_stereo_cloud);
		  SCAN_COPY(narrow_stereo_transform);

		  SCAN_COPY(wide_stereo_right_image);
		  SCAN_COPY(wide_stereo_right_caminfo);
		  SCAN_COPY(wide_stereo_left_image);
		  SCAN_COPY(wide_stereo_left_caminfo);
		  SCAN_COPY(wide_stereo_cloud);
		  SCAN_COPY(wide_stereo_transform);
		  
		  SCAN_COPY(highdef_image);
		  SCAN_COPY(highdef_caminfo);
		  SCAN_COPY(highdef_transform);
		  getSiftFeatures();
		  setIdentity(scans.back().camera_scans[j].head_delta);
		  setIdentity(scans.back().camera_scans[j].base_delta);

		}
	}

	void processClouds(){
		normal_clouds.resize(scans.size());
		filtered_clouds.resize(scans.size());
		if(scans.front().normal_cloud.width==0){    //need to get do cloud analysis
			std::vector<pcl::PointCloud<pcl::PointXYZINormal> > downsampled;
			std::vector<pcl::PointCloud<pcl::PointWithViewpoint> > vp_clouds;
			vp_clouds.resize(scans.size());
			downsampled.resize(scans.size());
			for(uint s=0;s<scans.size();s++){
				pcl::fromROSMsg (scans[s].cloud_vp,vp_clouds[s]);
				getNormals(vp_clouds[s],normal_clouds[s]);
				downSample(normal_clouds[s],downsampled[s]);
				removeOutliers(downsampled[s],filtered_clouds[s]);
				pcl::toROSMsg(normal_clouds[s],scans[s].normal_cloud);
				pcl::toROSMsg(filtered_clouds[s],scans[s].normal_cloud_filtered);
			}
		}
		else{ //if we have already done the analysis, just unpack them:
			for(uint s=0;s<scans.size();s++){
				pcl::fromROSMsg (scans[s].normal_cloud,normal_clouds[s]);
				pcl::fromROSMsg (scans[s].normal_cloud_filtered,filtered_clouds[s]);
			}
		}
		if(scans.front().normal_cloud_aligned.width>0){

	      aligned_clouds.resize(scans.size());
		   for(uint s=0;s<scans.size();s++)
		        pcl::fromROSMsg (scans[s].normal_cloud_aligned,aligned_clouds[s]);
		}


	}

	int getSiftFeatures(){
	  ROS_INFO("Getting sift features.  This could take a while.");
	  siftviewer::ProcessImageCluster srv;
	  for(uint s=0;s<scans.size();s++){
	    ROS_INFO("Getting sift features for scan %d of %d",s,scans.size());
	    for(uint cs=0;cs<scans[s].camera_scans.size();cs++){
	      srv.request.caminfo = scans[s].camera_scans[cs].highdef_caminfo;
	      srv.request.raw_images.push_back(scans[s].camera_scans[cs].highdef_image);
	      if (!imagesifter.call(srv)){
		ROS_ERROR("Failed to call imagesift service");
		return -1;
	      }
	      scans[s].camera_scans[cs].highdef_sift_features=srv.response.features[0];
	    }
	  }
	  return 0;
	}

	//performs one ICP alignment step with of the target cloud to the rest of the clouds:
   Eigen3::Matrix4f ICPStep(int target, sensor_msgs::PointCloud2& cloud);
   Eigen3::Matrix4f ICPStep2(int target, sensor_msgs::PointCloud2& cloud);


   Eigen3::Matrix4f alignWithICP(NCloud &snormcloud, NCloud &tnormcloud, NCloud &aligned){
     //test registration
     pcl::IterativeClosestPoint<pcl::PointXYZINormal,pcl::PointXYZINormal> icp;
     icp.setInputCloud(snormcloud.makeShared());
     icp.setInputTarget(tnormcloud.makeShared());
     icp.setMaximumIterations (500);
     icp.setTransformationEpsilon (1e-8);
     icp.setMaxCorrespondenceDistance (0.001);
     icp.align(aligned); // is snormcloud, moved
     int step=0;
     while(!icp.hasConverged()){
      step++;
      ROS_INFO("ICP alignment, step %d",step);
      cout<<"not converged.  "<<endl;
      icp.align(aligned);
     }
     return icp.getFinalTransformation();
   }

public:
	ScanAnalyzer(std::string filename){
		pub_map = nh_.advertise<mapping_msgs::PolygonalMap> ("scan_cloud_norms", 100);
		pub_cloud = nh_.advertise<sensor_msgs::PointCloud2> ("scan_cloud", 100);
		imagesifter = nh_.serviceClient<siftviewer::ProcessImageCluster>("/cluster_proc");

		if(readBag(filename)){
			std::cerr<<"could not load file"<<std::endl;
			exit(-1);
		}

//		if(scans.front().camera_scans.front().highdef_sift_features.size()==0)
//		      && scans.front().camera_scans.front().highdef_images.size() > 0)    //need to get sift features
			getSiftFeatures();
		processClouds();  //unpack the rosmsgs, generate normals and filter if needed.


	}


	void writeBag(std::string filename){
		rosbag::Bag bag(filename,rosbag::bagmode::Write);
		for(uint i=0; i<scans.size();i++)
			bag.write("analyzed_scans",ros::Time::now(),scans[i]);

		bag.close();
	}

	//Checks for both ScanAnalysis and SurveyScan messages
	int readBag(std::string filename){
		rosbag::Bag bag(filename);

		//check for SurveyScans in the bag:
		rosbag::TypeQuery query("object_survey/ScanAnalysis");
		rosbag::View view(bag,query,ros::TIME_MIN,ros::TIME_MAX);
		if(view.size() > 0){
			ROS_INFO("Found %d ScanAnalysis messages. Reading...",view.size());
			boost::shared_ptr <object_survey::ScanAnalysis> scan;
			BOOST_FOREACH(rosbag::MessageInstance m, view){
				if(scan = m.instantiate<object_survey::ScanAnalysis>())
					scans.push_back(*scan);
			}
			if(scans.size() == 0){
				ROS_ERROR("Error reading scans!");
				return -2;
			}
			return 0;
		}
		//if we didn't find already analyzed scans:
		//check for SurveyScans in the bag:
		rosbag::TypeQuery query2("object_survey/SurveyScan");
		rosbag::View view2(bag,query2,ros::TIME_MIN,ros::TIME_MAX);
		if(view2.size() > 0){
			ROS_INFO("Found %d unanalyzed surveyScans. Reading...",view2.size());
			boost::shared_ptr <object_survey::SurveyScan> scan;
			BOOST_FOREACH(rosbag::MessageInstance m, view2){
				if(scan = m.instantiate<object_survey::SurveyScan>())
					addSurveyScan(*scan);
			}
		}
		if(scans.size() == 0){
			std::cerr<<"no scans read."<<std::endl;
			return -2;
		}
		return 0;
	}

	boost::shared_ptr <pcl::PointCloud<pcl::PointXYZINormal> > getNormalCloud(int i){
		if(i<normal_clouds.size())
			return normal_clouds[i].makeShared();
	}

	int getNumScans(){
		return scans.size();

	}

	void combineClouds(pcl::PointCloud<pcl::PointXYZINormal>& cloud_pplheight,
			   double lower_cutoff=.15,double upper_cutoff=2.0) {
	  pcl::PointCloud<pcl::PointXYZINormal> full_cloud;
	  if(aligned_clouds.size()==0){
	    ROS_ERROR("No aligned scans to assemble!");
	    return;
	  }
	  full_cloud=aligned_clouds[0];
	  for(uint i=1;i<aligned_clouds.size();i++){
	    full_cloud+=aligned_clouds[i];
	  }

	  segmentHeight(full_cloud,cloud_pplheight,lower_cutoff,upper_cutoff);
	  
	}

	void clusterClouds(double lower_cutoff=.15,double upper_cutoff=2.0, double resolution=.02){
	  pcl::PointCloud<pcl::PointXYZINormal> cloud_pplheight, cloud_downsampled;
	  combineClouds(cloud_pplheight,lower_cutoff,upper_cutoff);
	  downSample(cloud_pplheight,cloud_downsampled,resolution);

	  std::vector<std::vector<int> > iclusters;
	  timeval t0=g_tick();
	  extractEuclideanClustersFast2(cloud_downsampled,iclusters,.2);
	  std::cout<<"clustered "<<iclusters.size()<<" points in: "<<g_tock(t0)<<std::endl;

	  clusters.resize(iclusters.size());
	  for(uint i=0;i<clusters.size();i++)
	    getSubCloud(cloud_downsampled,iclusters[i],clusters[i]);

	}


	void getNearestClusterWithoutDownsampling(pcl::PointCloud<pcl::PointXYZINormal> &cloud,
						  double lower_cutoff=.15,double upper_cutoff=2.0){
	  std::vector<pcl::PointCloud<pcl::PointXYZINormal> > filtered_full_clouds;
	  filtered_full_clouds.resize(scans.size());

	  pcl::PointCloud<pcl::PointXYZINormal> full_cloud;
	  
	  for(uint s=0;s<scans.size();s++){
	    //filter clouds
	    removeOutliers(normal_clouds[s],filtered_full_clouds[s]);
	    ROS_INFO("filtered cloud %d",s);
	    //transform clouds
	    pcl::PointCloud<pcl::PointXYZINormal> transformed;

	    tf::Transform laser, laser_aligned, alignment;
	    tf::transformMsgToTF(scans[s].laser_transform.transform,laser);
	    tf::transformMsgToTF(scans[s].laser_aligned.transform,laser_aligned);
	    //alignment*laser=laser_aligned
	    //alignment      =laser_aligned*inv(laser)
	    alignment = laser_aligned*laser.inverse();
	    pcl::transformPointCloudWithNormals(filtered_full_clouds[s], transformed,alignment);
	    //pcl::transformPointCloudWithNormals(filtered_full_clouds[s], transformed,tfFromEigen(alignments[s]));
	    ROS_INFO("transformed cloud %d",s);

	    //keep points in the normal height range
	    segmentHeight(transformed,filtered_full_clouds[s],lower_cutoff,upper_cutoff);
	    ROS_INFO("segmented cloud %d",s);

	    //combine the clouds together
	    if(s==0) {
	      full_cloud = filtered_full_clouds[s];
	    }
	    else {
	      full_cloud += filtered_full_clouds[s];
	    }
	    ROS_INFO("combined cloud %d",s);
	  }

	  std::vector<std::vector<int> > iclusters;
	  timeval t0=g_tick();
	  extractEuclideanClustersFast2(full_cloud,iclusters,.2);
	  std::cout<<"clustered "<<iclusters.size()<<" points in: "<<g_tock(t0)<<std::endl;


	  //now for each of the clusters, check which one is closest
	  pcl::PointXYZINormal pt,tmp;
	  cp3dPt(pt,scans.back().object_pose.position);

	  int best=-1;
	  double tempd, closestd;
	  for(int j=0;j<iclusters.size();j++) {
	    pcl::PointCloud<pcl::PointXYZINormal> cluster;
	    getSubCloud(full_cloud,iclusters[j],cluster);
	    tempd=getClosestPoint(cluster,pt,tmp);
	    if(!j || tempd<closestd) {
	      best=j;
	      closestd=tempd;
	    }
	  }
	  
	  getSubCloud(full_cloud,iclusters[best],cloud);
	}



	void getNearestStereoClusterWithoutDownsampling(pcl::PointCloud<pcl::PointXYZINormal> &cloud,
							double lower_cutoff=.15,double upper_cutoff=2.0){
	  std::vector<pcl::PointCloud<pcl::PointXYZINormal> > filtered_full_clouds;
	  filtered_full_clouds.resize(scans.size());

	  pcl::PointCloud<pcl::PointXYZINormal> full_cloud;
	  
	  for(uint s=0;s<scans.size();s++){
	    

	    pcl::PointCloud<pcl::PointWithViewpoint> vp_cloud;
	    pcl::fromROSMsg (scans[s].camera_scans[0].narrow_stereo_cloud,vp_cloud);
	    //calculate normals
	    pcl::PointCloud<pcl::PointXYZINormal> normal_cloud;
	    getNormals(vp_cloud,normal_cloud);
	    ROS_INFO("calculated normals for cloud %d",s);
	    //filter clouds
	    removeOutliers(normal_cloud,filtered_full_clouds[s]);
	    ROS_INFO("filtered cloud %d",s);
	    //transform clouds
	    pcl::PointCloud<pcl::PointXYZINormal> transformed;

	    tf::Transform laser, laser_aligned, narrow_stereo, alignment, stereo_alignment;
	    tf::transformMsgToTF(scans[s].laser_transform.transform,laser);
	    tf::transformMsgToTF(scans[s].laser_aligned.transform,laser_aligned);
	    tf::transformMsgToTF(scans[s].camera_scans[0].narrow_stereo_transform.transform,narrow_stereo);
	    //alignment*laser=laser_aligned
	    //alignment      =laser_aligned*inv(laser)
	    alignment = laser_aligned*laser.inverse();
	    stereo_alignment = alignment*narrow_stereo;
	    pcl::transformPointCloudWithNormals(filtered_full_clouds[s], transformed,stereo_alignment);
	    //pcl::transformPointCloudWithNormals(filtered_full_clouds[s], transformed,tfFromEigen(alignments[s]));
	    ROS_INFO("transformed cloud %d",s);

	    //keep points in the normal height range
	    segmentHeight(transformed,filtered_full_clouds[s],lower_cutoff,upper_cutoff);
	    ROS_INFO("segmented cloud %d",s);

	    //combine the clouds together
	    if(s==0) {
	      full_cloud = filtered_full_clouds[s];
	    }
	    else {
	      full_cloud += filtered_full_clouds[s];
	    }
	    ROS_INFO("combined cloud %d",s);
	  }

	  std::vector<std::vector<int> > iclusters;
	  timeval t0=g_tick();
	  extractEuclideanClustersFast2(full_cloud,iclusters,.2);
	  std::cout<<"clustered "<<iclusters.size()<<" points in: "<<g_tock(t0)<<std::endl;


	  //now for each of the clusters, check which one is closest
	  pcl::PointXYZINormal pt,tmp;
	  cp3dPt(pt,scans.back().object_pose.position);

	  int best=-1;
	  double tempd, closestd;
	  for(int j=0;j<iclusters.size();j++) {
	    pcl::PointCloud<pcl::PointXYZINormal> cluster;
	    getSubCloud(full_cloud,iclusters[j],cluster);
	    tempd=getClosestPoint(cluster,pt,tmp);
	    if(!j || tempd<closestd) {
	      best=j;
	      closestd=tempd;
	    }
	  }
	  
	  getSubCloud(full_cloud,iclusters[best],cloud);
	}




	void getNarrowStereoCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, int scan_number){
	  pcl::fromROSMsg (scans[scan_number].camera_scans[0].narrow_stereo_cloud,cloud);
	}

	void getWideStereoCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud, int scan_number){
	  pcl::fromROSMsg (scans[scan_number].camera_scans[0].wide_stereo_cloud,cloud);
	}



	//assumes we have clusters already
	int getClosestCluster(geometry_msgs::Point target){
	   int ret=-1;
      pcl::PointXYZINormal pt,tmp;
      cp3dPt(pt,target);
	   double tempd, closestd;
      for(uint i=0;i<clusters.size();i++){
         tempd=getClosestPoint(clusters[i],pt,tmp);
         if(!i || tempd<closestd){
            ret=i;
            closestd=tempd;
         }
      }
      return ret;
	}


	void AlignClouds(int iterations=2){
	  if(aligned_clouds.size()) {
	      ROS_INFO("Clouds appear to be aligned already. Skipping alignment step");

	  }
	   else
	      AccumulateScans2(iterations);
	}

	void saveObjectCloud(string filename, string object_name){
	   clusterClouds();
	   int nearest= getClosestCluster(scans.back().object_pose.position);

//      pcl::io::savePCDFile("testcluster.pcd",clusters[nearest]);
	   writeMesh(clusters[nearest],filename,object_name);

//
//	   pcl::PointCloud<pcl::PointXYZINormal> full_cloud,cloud_no_floor,cloud_pplheight;
//	   if(aligned_clouds.size()==0){
//	      ROS_ERROR("No aligned scans to assemble!");
//	      return;
//	   }
//	   full_cloud=aligned_clouds[0];
//      for(uint i=1;i<aligned_clouds.size();i++){
//         full_cloud+=aligned_clouds[i];
//      }
//
////      segmentFloor(full_cloud,cloud_no_floor,.15,2.0);
//      segmentHeight(full_cloud,cloud_pplheight,.15,2.0);
//      downSample(cloud_pplheight,cloud_pplheight,.02);
//
//      std::vector<pcl::PointCloud<pcl::PointXYZINormal> > clusters;
//      std::vector<std::vector<int> > iclusters;
//      pcl::PointXYZINormal pt,tmp;
//      cp3dPt(pt,scans.back().object_pose.position);
//      extractEuclideanClustersFast2(cloud_pplheight,iclusters,.2);
////      segfast(cloud_pplheight,clusters);
//      int closesti=0;
//      double tempd, closestd;
//      clusters.resize(iclusters.size());
//      for(uint i=0;i<clusters.size();i++){
//        getSubCloud(cloud_pplheight,iclusters[i],clusters[i]);
//         tempd=getClosestPoint(clusters[i],pt,tmp);
//         if(!i || tempd<closestd){
//            closesti=i;
//            closestd=tempd;
//         }
//      }
//      //closest cluster is: clusters[closesti]
//      writePLY(clusters[closesti],filename);

	}



//	void printFields(){
//		for(uint s=0;s<scans.size();s++){
//			std::cout<<"scan "<<s<<" pts: "<<scans[s].cloud.height*scans[s].cloud.width<<" fields:  ";
//			for(uint f=0;f<scans[s].cloud.fields.size();f++)
//				std::cout<<scans[s].cloud.fields[f].name<<" ";
//			std::cout<<std::endl;
//		}
//	}



//	void PublishCloud(int c){
//		sensor_msgs::PointCloud2 cloud;
//		pcl::toROSMsg(full_clouds[c],cloud);
//		mapping_msgs::PolygonalMap pmap=makePMap(full_clouds[c]);
//		pub_cloud.publish(cloud);
//		pub_map.publish(pmap);
//	}
//	void PublishAligned(){
//		sensor_msgs::PointCloud2 cloud;
//		pcl::toROSMsg(aligned,cloud);
//		mapping_msgs::PolygonalMap pmap=makePMap(aligned);
//		pub_cloud.publish(cloud);
//		pub_map.publish(pmap);
//	}
//
//
	void colorCloud(NCloud cloudin, pcl::PointCloud<pcl::PointXYZRGB> &colored,float color){
		pcl::PointXYZRGB p;
		for(uint i=0;i<cloudin.points.size();i++){
			p.x=cloudin.points[i].x;
			p.y=cloudin.points[i].y;
			p.z=cloudin.points[i].z;
			p.rgb=color;
			colored.push_back(p);
		}
	}

	void PublishClouds(NCloud c1, NCloud c2){
	   NCloud f1=c1;

      for(int p=0;p<f1.points.size();p++)
         f1.points[p].intensity=.1;
	   f1+=c2;
		sensor_msgs::PointCloud2 cloud;
		pcl::toROSMsg(f1,cloud);
		cloud.header.frame_id="/world";
		pub_cloud.publish(cloud);
//		mapping_msgs::PolygonalMap pmap=makePMap(c1);
//		pub_map.publish(pmap);
	}


//	void PublishClouds(int c1, int c2,float a, float b){
//		pcl::PointCloud<pcl::PointXYZRGB> colored2;
//		colorCloud(full_clouds[c1],colored2,a);
//		if(c2>=0)
//			colorCloud(full_clouds[c2],colored2,b);
//		else
//			colorCloud(aligned,colored2,b);
//
//		sensor_msgs::PointCloud2 cloud;
//		pcl::toROSMsg(colored2,cloud);
//		mapping_msgs::PolygonalMap pmap=makePMap(full_clouds[c1]);
//		cloud.header.frame_id="/world";
//		pub_cloud.publish(cloud);
//		pub_map.publish(pmap);
//	}
//
//	void alignClouds(int source, int target, bool useicp=true){
//		Eigen3::Matrix4f trans;
//		if(useicp)
//			trans=alignWithICP(full_clouds[source],full_clouds[target],aligned);
//		else
//			trans=alignInitial(full_clouds[source],full_clouds[target],aligned);
//		std::cout<<"alignment of cloud "<<source<<" with "<<target<<".  useicp = "<<useicp<<std::endl<<trans<<std::endl;
//	}
//
//	Eigen3::Matrix4f getAlign(int source, int target){
//		timeval t=g_tick();
//		NCloud cloud;
//		Eigen3::Matrix4f trans=alignWithICP(full_clouds[source],full_clouds[target],cloud);
//		std::cout<<"aligning "<<source<<" -> "<<target<<" took "<<g_tock(t)<<std::endl;
//		return trans;
//	}
//
//
//
//	Eigen3::Matrix4f getFastAlign(int source, int target){
//		timeval t=g_tick();
//		NCloud scloud,tcloud;
//		NCloud scloud1,tcloud1;
//		segmentHeight(full_clouds[source],scloud1,1.0);
//		segmentHeight(full_clouds[target],tcloud1,1.0);
//
//		cout<<tcloud.points.size()<<" "<<scloud.points.size()<<std::endl;
//
//		matchPoints(scloud1,tcloud1,tcloud,.1);
//		matchPoints(tcloud,scloud1,scloud,.1);
//
//		NCloud cloud,scloudfull;
//		Eigen3::Matrix4f trans=alignWithICP(scloud,tcloud,cloud);
//		trans(2,3)=0.0; //little hack to make sure everything stays on the ground
//		pcl::transformPointCloudWithNormals(full_clouds[source],scloudfull,trans);
//
//		PublishClouds(scloudfull,full_clouds[target],1<<2,0);
//		PublishClouds(scloudfull,full_clouds[target],1<<2,0);
//		PublishClouds(scloudfull,full_clouds[target],1<<2,0);
//		PublishClouds(scloudfull,full_clouds[target],1<<2,0);
//		std::cout<<"fastaligning "<<source<<" -> "<<target<<" took "<<g_tock(t)<<std::endl;
//		return trans;
//	}
//

	//aligns all the scans to the last scan
//	void AccumulateScans();

	//cloud align function, broken out into separable functions:
	void Palign(int s, int t){

      timeval t0=g_tick();
	 {
      boost::mutex::scoped_lock  lock(print_mutex);
      ROS_INFO("aligning clouds %d and %d",s,t);
    }
      NCloud s_segmented,t_segmented;
      NCloud s_matched,t_matched;
      {
         boost::mutex::scoped_lock  lock(data_mutex);
         segmentHeight(filtered_clouds[s],s_segmented,.50, 5.0);
         segmentHeight(filtered_clouds[t],t_segmented,.50,5.0);
//         matchPoints(s_segmented,t_segmented,t_matched,.2);//flann is not thread safe. :(
//         matchPoints(t_matched,s_segmented,s_matched,.2);
         t_matched=t_segmented;
         s_matched=s_segmented;
      }
      {
        boost::mutex::scoped_lock  lock(print_mutex);
        ROS_INFO("segmented and matched %d and %d: %d pts,  %d pts in %f seconds.",s,t,(int)s_matched.points.size(),(int)t_matched.points.size(),g_tock(t0));
      }
      t0=g_tick();
      NCloud transformed_cloud;
      Eigen3::Matrix4f trans=alignWithICP(s_matched,t_matched,transformed_cloud); //get transformed scloud
      {
        boost::mutex::scoped_lock  lock(print_mutex);
        ROS_INFO("aligned clouds %d and %d in %f seconds.",s,t,g_tock(t0));
      }
      {
         boost::mutex::scoped_lock  lock(data_mutex);
         alignments[s]=trans;
      }
	}


//parallel version of alignment algorithm:
	void AccumulateScansParallel(){
      aligned_clouds.resize(filtered_clouds.size());
      alignments.resize(filtered_clouds.size());
//	      Palign(0,19);
//	      return;
	      NCloud scloud,tcloud;
	      NCloud scloud1,tcloud1;
	      NCloud rolling;
	      Eigen3::Matrix4f rollingtrans = Eigen3::Matrix4f::Identity();
	      rolling=filtered_clouds[0];
//	      std::vector<boost::thread> threads(filtered_clouds.size());
	      boost::thread_group threads;
	      for(uint i=0;i<filtered_clouds.size()-1;i++){
	         uint target=(i+1)%filtered_clouds.size(); //wrap around to beginning
	         threads.create_thread(boost::bind(&ScanAnalyzer::Palign,this,i,target));
	      }
	      //wait for everything to finish
	      threads.join_all();
//	      for(uint i=0;i<threads.size();i++){
//	         threads[i].join();
//	      }

//	         Eigen3::Matrix4f trans2d = projectTo2D(trans);
//	//       trans(2,3)=0.0; //little hack to make sure everything stays on the ground
//	         std::cout<<trans2d<<std::endl;
//	//       rollingtrans*=trans;
//	         pcl::transformPointCloudWithNormals(rolling,scloudfull,trans2d);
//	         pcl::transformPointCloudWithNormals(filtered_clouds[i-1],aligned_clouds[i-1],trans2d);
//	         //now propagate the transform for all our aligned scans:
//	         for(int j=0;j<i-1;j++){
//	            pcl::transformPointCloudWithNormals(aligned_clouds[j],aligned_clouds[j],trans2d);
//	         }
//
//	         rolling=scloudfull;
//	         rolling+=filtered_clouds[i];
//	      }
//	      aligned_clouds.back()=filtered_clouds.back();
//
//	      for(uint s=0;s<scans.size();s++)
//	         pcl::toROSMsg (aligned_clouds[s],scans[s].normal_cloud_aligned);

	   }

	//sequentially align all scans with each other
   void AccumulateScans2(int iterations=2);

//	void write_trans(int c1){
//		NCloud cloud;
//		char filename[30];
//		sprintf(filename,"trans%02d.txt",c1);
//		std::ofstream outf;
//		outf.open(filename,ios::out|ios::app);
//		timeval t0;
//		for(uint c2=0;c2<scans.size();c2++){
//			t0=g_tick();
//			Eigen3::Matrix4f m=alignWithICP(full_clouds[c1],full_clouds[c2],cloud);
//			outf<<c1<<" "<<c2<<" ";
//			for (int j=0; j<m.cols(); ++j)   // loop over columns
//			  for (int i=0; i<m.rows(); ++i) // loop over rows
//				  outf<< m(i,j)<<" ";
//			outf<<std::endl;
//			std::cout<<c1<<" -> "<<c2<<" in "<<g_tock(t0)<<std::endl;
//		}
//		outf.close();
//	}
//	void getAligns(){
//		transforms.resize(scans.size());
//		for(uint i=0;i<scans.size();i++)
//			transforms[i].resize(scans.size());
////		#pragma omp parallel for
//		for(uint i=0;i<scans.size();i++)
//			for(uint j=0;j<scans.size();j++)
//				getAlign(i,j);
//
//	}

//	void command(){
////		char cmd;
//		char action;
//		int c1,c2;
//		while(1){
//			cin>>c1>>action>>c2;
//			switch (action) {
//				case '+':
//					PublishClouds(c1,c2,1<<2,0);
//					break;
//				case 'a':
//					alignClouds(c1,c2);
//					PublishClouds(c2,-1,1<<2,0);
//					break;
//				default:
//					std::cout<<"unknown command"<<std::endl;
//					return;
//					break;
//			}
//		}
//	}


	virtual ~ScanAnalyzer();
};

#endif /* SCANANALYZER_H_ */
