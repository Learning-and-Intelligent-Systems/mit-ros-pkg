/*
 * object_analysis.cpp
 *
 *  Created on: Oct 17, 2010
 *      Author: garratt
 */

//this program is designed to analyse the data from object_survey, to get a more accurate registration of poses

//[object_survey/SurveyScan]:
//sensor_msgs/PointCloud2 cloud
//geometry_msgs/TransformStamped laser_transform
//geometry_msgs/TransformStamped camera_transform
//geometry_msgs/TransformStamped base_transform
//sensor_msgs/Image[] images
//sensor_msgs/CameraInfo caminfo
//geometry_msgs/Pose object_pose

//#include <furniture_ops/pcl_helpers.hpp>

//#include "pcl/segmentation/extract_clusters.h"
//
//#include "pcl/segmentation/extract_clusters.hpp"
#include "ScanAnalyzer.h"





//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///** \brief Decompose a Point Cloud into clusters based on the Euclidean distance between points. Uses Hierarchical Clustering, making it faster
//  * \param cloud the point cloud message
//  * \param cluster_tol the spatial cluster tolerance as a measure in L2 Euclidean space
//  * \param cclusters the resultant clusters containing point indices (as a vector of vector of ints)
//  * \param min_pts_per_cluster minimum number of points that a cluster may contain (default = 1)
//  */
//template <typename PointT>
//void extractEuclideanClustersFast2(pcl::PointCloud<PointT> &cloud, std::vector<std::vector<int> > &clusters, double cluster_tol=.2, int min_pts_per_cluster=1){
//   double smaller_tol = cluster_tol/2.3;
//   pcl::KdTreeFLANN<PointT> tree,tree2;
//   tree.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud));
//   vector<int> minitree(cloud.points.size(),-1);
//   vector<int> heads,indices;
//   vector<float> dists;
//
//   //First 'downsample' the points into clusters slightly less than half the cluster tolerance
//   //minitree keeps track of who each point belongs to (which is an index of the 'head' array)
//   for(uint i=0; i<cloud.points.size();i++){
//     if(minitree[i]==-1){    //if no one has claimed this point, make it a head
//        heads.push_back(i);
//        if(!tree.radiusSearch(cloud,i,smaller_tol,indices,dists))  //find all the points close to this point
//           ROS_WARN("radius search failed!");
//        for(uint j=0;j<indices.size();j++){
//           minitree[indices[j]]=heads.size()-1; //assign these points to the current head.
//                                       // this overwrites previous claims, but it's ok
//        }
//     }
//   }
//
//   //now, we have much fewer points to cluster, but since the initial downsampling was less than
//   //half the cluster tolerance,we are guaranteed to find all the points within the tolerance
//   //(except if  [ clustertol < distance between heads < cluster_tol+ smaller cluster tol] AND the head closest points in the two heads are < cluster_tol
//   //The next code block deals with that...
//   tree2.setInputCloud(boost::make_shared<pcl::PointCloud<PointT> > (cloud),boost::make_shared<std::vector<int> >  (heads));
//   int searching,currenthead;
//   //heads2 is the cluster id.  we now search to make sure we check all the points in our cluster
//   //minitree2 corresponds to the heads array, so the points in minitree2 are at heads[i]
//   std::vector<int> heads2, minitree2(heads.size(),-1);
//   std::list<int> tosearch;
//   for(uint i=0; i<minitree2.size();i++){
//     if(minitree2[i]==-1){
//        heads2.push_back(heads[i]);
//        tosearch.push_back(i);
//        currenthead=heads2.size()-1; //references an index in heads2
//        minitree2[i]=currenthead;
//        while(tosearch.size()>0){
//           searching=tosearch.front();
//           tosearch.pop_front();
//           if(!tree2.radiusSearch(cloud.points[heads[searching]],cluster_tol,indices,dists))
//              cout<<"radius search failed!"<<endl;
//           for(uint j=0;j<indices.size();j++)
//              if(minitree2[indices[j]]==-1){//found untouched point (which means this root touched it)
//                 minitree2[indices[j]]=currenthead; //claim it
//                 tosearch.push_back(indices[j]);   //add it to list of points to search
//              }
//        }
//     }
//
//   }
//
//   //now we need to check to see if any of the heads that were NOT clustered together are closer than cluster_tol+smaller_tol.
//   //This covers the exception noted in the code block above
//   //this is where an adversary could really kill this algorithm, since this check could be polynomial in cloud size. in real circumstances, it is very quick.
//   vector<int> indices1;
//   vector<float> dists1;
//   for(uint i=0; i<minitree2.size();i++){ //for every head
//      tree2.radiusSearch(cloud.points[heads[i]],cluster_tol+smaller_tol,indices,dists);
//      for(uint j=0;j<indices.size();j++) //for every head that is close to this head
//         if(minitree2[indices[j]] != minitree2[i] && i<j){//if the two heads are not in the same cluster (and only check each combo once)
//        	ROS_INFO("head %d (in cluster %d) is %f from head %d (in cluster %d). Checking if the two heads share points.",
//        			i,minitree2[i],sqrt(dists[j]),indices[j],minitree2[indices[j]]);
//            //now we have to find if there is a point between head a and head b that is < cluster_tol from both
//            //our best chance of finding it is to start searching at a point halfway between them
//            PointT heada=cloud.points[heads[i]], headb=cloud.points[heads[indices[j]]];
//            PointT inbetween;
//            inbetween.x=(heada.x+headb.x)/2.0;
//            inbetween.y=(heada.y+headb.y)/2.0;
//            inbetween.z=(heada.z+headb.z)/2.0;
//
//            tree.radiusSearch(inbetween,cluster_tol,indices1,dists1); //search in the full tree
//            double distthresh=cluster_tol*cluster_tol; //radius search gives squared distances...
//            for(uint k=0;k<indices1.size();k++){
//               if(squaredEuclideanDistance(cloud.points[indices1[k]],heada ) < distthresh && squaredEuclideanDistance(cloud.points[indices1[k]],headb ) < distthresh ){
//                  //there is a point that these two clusters share -> they should be merged
//            	  ROS_INFO("head %d (in cluster %d) shares a point with head %d (in cluster %d). Merging cluster %d into cluster %d ",
//            			  i,minitree2[i],indices[j],minitree2[indices[j]],minitree2[indices[j]], minitree2[i]);
//                  //rename all heads in cluster b to cluster a
//                  int clusterb=minitree2[indices[j]];
//                  for(uint m=0;m<minitree2.size();m++){
//                     if(minitree2[m]==clusterb) minitree2[m]=minitree2[i];
//                  }
//                  break;
//               } //if the point is shared
//            } //for every point close to both heads
//         } //if a head is close, and from a different cluster
//   }  //for every head
//
//   clusters.resize(heads2.size());
//   //now, for each point in the cloud find its head --> then the head it clustered to. that is its cluster id!
//   for(uint j=0;j<minitree.size();j++){
//     clusters[minitree2[minitree[j]]].push_back(j);
//   }
//
//   //erase the deleted clusters, and the ones under the minimum size
//   for(int i=clusters.size()-1;i>=0; i--)
//      if(clusters[i].size() < min_pts_per_cluster)
//         clusters.erase(clusters.begin()+i);
//}






//
//
//
//
//Eigen3::Matrix4f alignInitial(NCloud &snormcloud, NCloud &tnormcloud, NCloud &aligned){
//	  boost::shared_ptr<const NCloud > cloud_source_ptr, cloud_target_ptr;
//	  cloud_source_ptr = boost::make_shared<const NCloud > (snormcloud);
//	  cloud_target_ptr = boost::make_shared<const NCloud > (tnormcloud);
//
//	  // Initialize estimators for surface normals and FPFH features
//	  pcl::KdTreeANN<NPoint> tree;
//
//	  pcl::FPFHEstimation<NPoint, NPoint, pcl::FPFHSignature33> fpfh_est;
//	  fpfh_est.setSearchMethod (boost::make_shared<pcl::KdTreeANN<NPoint> > (tree));
//	  fpfh_est.setRadiusSearch (0.05);
//	  pcl::PointCloud<pcl::FPFHSignature33> features_source, features_target;
//
//	  fpfh_est.setInputCloud (cloud_source_ptr);
//	  fpfh_est.setInputNormals (cloud_source_ptr);
//	  fpfh_est.compute (features_source);
//
//	  fpfh_est.setInputCloud (cloud_target_ptr);
//	  fpfh_est.setInputNormals (cloud_target_ptr);
//	  fpfh_est.compute (features_target);
//
//	  // Initialize Sample Consensus Initial Alignment (SAC-IA)
//	  pcl::SampleConsensusInitialAlignment<NPoint, NPoint, pcl::FPFHSignature33> reg;
//	  reg.setMinSampleDistance (0.05);
//	  reg.setMaxCorrespondenceDistance (0.2);
//	  reg.setMaximumIterations (1000);
//
//	  reg.setInputCloud (cloud_source_ptr);
//	  reg.setInputTarget (cloud_target_ptr);
//	  reg.setSourceFeatures (boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33> > (features_source));
//	  reg.setTargetFeatures (boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33> > (features_target));
//	  cout<<"aligning"<<endl;
//	  // Register
//	  reg.align (aligned);
//	  return reg.getFinalTransformation();
//}
//

//
////finds point in target cloud that are near points in ref_cloud
////this works by searching for nearest neighbors of a bunch of points in the ref cloud
////it is approximate, because we don't really check every point in ref_cloud
//
//
//mapping_msgs::PolygonalMap makePMap(pcl::PointCloud<pcl::PointXYZINormal> ncloud){
//	mapping_msgs::PolygonalMap pmap;
//	pmap.header.stamp=ros::Time::now();
//	pmap.header.frame_id="/odom_combined";
//	geometry_msgs::Polygon p;
//	p.points.resize(2);
//	for(uint i=0;i<ncloud.points.size();i++){
//		p.points[0].x=ncloud.points[i].x;
//		p.points[0].y=ncloud.points[i].y;
//		p.points[0].z=ncloud.points[i].z;
//		p.points[1].x=ncloud.points[i].x + ncloud.points[i].normal[0]*.1;
//		p.points[1].y=ncloud.points[i].y + ncloud.points[i].normal[1]*.1;
//		p.points[1].z=ncloud.points[i].z + ncloud.points[i].normal[2]*.1;
//		pmap.polygons.push_back(p);
//	}
//	return pmap;
//}
//
//std::vector<double> get2DTrans(Eigen3::Matrix4f trans3d){
//	std::vector<double> ret(3,0.0); //return x,y,theta
//	ret[0]=trans3d(0,3);
//	ret[1]=trans3d(1,3);
//	ret[2]=acos((trans3d(0,0)+trans3d(1,1))/2.0);
//	return ret;
//}
//
//tf::Transform tfFromEigen(Eigen3::Matrix4f trans){
//	btMatrix3x3 btm;
//	btm.setValue(trans(0,0),trans(0,1),trans(0,2),
//			     trans(1,0),trans(1,1),trans(1,2),
//			     trans(2,0),trans(2,1),trans(2,2));
//	btTransform ret;
//	ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
//	ret.setBasis(btm);
//	return ret;
//}


int main(int argc, char **argv) {
//
//   for(int j=0;j<5;j++){
//      double endp=(1.0-(.6-((double)j+1 )*.1));
//      cout<<"endpoint: "<<endp<<endl;
//      for(int i=21-1;i>=0;i--){
//         int targetcloud=(i+1)%21;
//         cout<<i<<" --> "<<targetcloud<<":     \t";
//         //propogate the transform back:
//         if(i>0)
//         cout.precision(3);
//         for(int k=i-1; k>0; k--){
//            double diff=i-k, num=i;
//            double scalef= (1.0-(endp*diff)/num);
//            cout<<scalef<<" ";
////              cout<<"transforming "<<k<<" by "<<scalef<<" times the transform from "<<i<<" to "<<targetcloud<<endl;
//         }
//         cout<<endl;
//      }
//   }
//   return 0;



   if(argc!=3){
      std::cout<<"USAGE "<<argv[0]<<" file_in.bag file_out(no extension) "<<std::endl;
      return -1;
   }

   ros::init(argc, argv, "alignment");
   ScanAnalyzer analyzer(argv[1]);
   analyzer.AlignClouds(2);
//   analyzer.AccumulateScans();
   string bagout=argv[2];
   bagout+=".bag";
   ROS_INFO("writing to bag file %s",bagout.c_str());
   analyzer.writeBag(bagout);
   std::string plyname=argv[2];
//   plyname.append(".ply");
   analyzer.saveObjectCloud(plyname,"chair");

//	boost::shared_ptr <pcl::PointCloud<pcl::PointXYZINormal> > ncloud;
//
//	 pcl::PointCloud<pcl::PointXYZINormal> smallcloud;
//	 char filename[500];
//	for(uint i=0;i<analyzer.getNumScans();i++){
//		ncloud=analyzer.getNormalCloud(i);
//		std::cout<<" got cloud "<<i<<", size: "<<ncloud->size()<<std::endl;
//		 for(float f=2.0; f>0.0; f-=.1){
//			 segmentHeight(*ncloud,smallcloud,f);
//			 sprintf(filename,"testlogs/testcloud%02d_%01.1f.pcd",i,f);
//			 pcl::io::savePCDFile(filename,smallcloud);
//			 std::cout<<"cloud "<<i<<" height: "<<f<<": size: "<<smallcloud.size()<<std::endl;
//		 }
//	}
//	 timeval t0=g_tick();
//	  std::vector<pcl::PointIndices> inds;
//	  // Create the segmentation object
//	  pcl::EuclideanClusterExtraction<pcl::PointXYZINormal> clusterer;
//	  clusterer.setClusterTolerance(.2);
//	  clusterer.setMinClusterSize(1);
//	  clusterer.setMaxClusterSize(1000000);
//	  clusterer.setInputCloud(smallcloud.makeShared());
//	  clusterer.extract(inds);
//
//	  std::cout<<"normal segmentation took:  "<<g_tock(t0)<<"  for "<<inds.size()<<" clusters."<<std::endl;
//
//	for(uint i=0;i<inds.size();i++){
//		std::cout<<"cluster "<<i<<"  --> "<<inds[i].indices.size()<<std::endl;
//	}
//	t0=g_tick();
//	std::vector<std::vector<int> > clusters2;
//	pcl::extractEuclideanClustersFast(smallcloud,clusters2,.2);
//	std::cout<<"fast segmentation took:  "<<g_tock(t0)<<"  for "<<clusters2.size()<<" clusters."<<std::endl;
//	for(uint i=0;i<clusters2.size();i++){
//		std::cout<<"cluster "<<i<<"  --> "<<clusters2[i].size()<<std::endl;
//	}
//
//
//	for(uint i=0; i< clusters2[8].size();i++){
//		for(uint j=0; j< clusters2[11].size();j++){
//			float dist=pcl::euclideanDistance(smallcloud.points[clusters2[8][i]],smallcloud.points[clusters2[11][j]]);
//			if(dist<.2)
//				std::cout<<dist<<"  between "<<i<<"  and "<<j<<std::endl;
//			}
//
//	}



//	if(argc==3){
//		ROS_INFO("writing to bag file %s",argv[2]);
//		analyzer.writeBag(argv[2]);
//	}

//	getSiftFeatures();
//	getClouds();
//	analyzer.AccumulateScans();



//	Eigen3::Matrix4f t1,t2;
////	t1=analyzer.getAlign(1,2);
////	vector<double> t1a=get2DTrans(t1);
////	cout<<   fixed   <<t1a[0]<<" "<<t1a[1]<<" "<<t1a[2]<<endl;
//	t2=analyzer.getFastAlign(4,5);
//	vector<double> t2a=get2DTrans(t2);
//	cout<<   fixed   <<t2a[0]<<" "<<t2a[1]<<" "<<t2a[2]<<endl;
////	std::cout<<"normal: "<<endl<<t1<<endl;
//	std::cout<<"fast: "<<endl<<t2<<std::endl;




//	analyzer.getAligns();
//	analyzer.write_trans(atoi(argv[2]));
//	analyzer.command();

//	analyzer.printFields();
//	for(int i=0;i<32;i++){
//		float a=1<<i;
//		std::cout<<a<<std::endl;
//		analyzer.PublishClouds(1,2,a,0);
//		sleep(1);
//	}
//	analyzer.alignClouds(0,1,true);
//	analyzer.alignClouds(0,1,false);
//	analyzer.alignClouds(1,2,true);
//	analyzer.alignClouds(1,2,false);


	return 0;


}





