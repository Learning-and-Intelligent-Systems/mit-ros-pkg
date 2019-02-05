/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Garratt Gallagher
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name Garratt Gallagher nor the names of other
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/



#include "pcl_tools/pcl_utils.h"

  timeval g_tick(){
     struct timeval tv;
     gettimeofday(&tv, NULL);
     return tv;
  }

  double g_tock(timeval tprev)
  {
     struct timeval tv;
     gettimeofday(&tv, NULL);
     return (double)(tv.tv_sec-tprev.tv_sec) + (tv.tv_usec-tprev.tv_usec)/1000000.0;
  }

  int getUsec(){
     struct timeval tv;
     gettimeofday(&tv, NULL);
     return tv.tv_usec;
  }

  tf::Transform tfFromEigen(Eigen::Matrix4f trans){
   btMatrix3x3 btm;
   btm.setValue(trans(0,0),trans(0,1),trans(0,2),
              trans(1,0),trans(1,1),trans(1,2),
              trans(2,0),trans(2,1),trans(2,2));
   btTransform ret;
   ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
   ret.setBasis(btm);
   return ret;
  }

  double getYaw(Eigen::Matrix4f etrans){
     tf::Transform tftrans=tfFromEigen(etrans);
     return tf::getYaw(tftrans.getRotation());
  }

  //removes the yaw, pitch and z component from the transform
  Eigen::Matrix4f projectTo2D(Eigen::Matrix4f etrans){
     tf::Transform tftrans=tfFromEigen(etrans);
     tftrans.setRotation(tf::createQuaternionFromYaw(tf::getYaw(tftrans.getRotation())));
     Eigen::Matrix4f out;
     pcl_ros::transformAsMatrix(tftrans,out);
     out(2,3)=0.0;  //remove z component
     return out;
  }

  void writePLY(pcl::PointCloud<pcl::PointXYZINormal> &cloud, std::string filename){
     std::ofstream outf;
     outf.open(filename.c_str(),std::ios::out);
     outf<<"ply\n"<<"format ascii 1.0\n"<<"element vertex "<<cloud.points.size()<<std::endl;
     outf<<"property float x\nproperty float y\nproperty float z\n";
     outf<<"property float nx\nproperty float ny\nproperty float nz\n";
     outf<<"element face 0\nproperty list uchar int vertex_indices\nend_header\n";
     for (int i = 0; i < cloud.points.size(); ++i) {
        outf<<cloud.points[i].x<<" "<<cloud.points[i].y<<" "<<cloud.points[i].z;
        outf<<" "<<cloud.points[i].normal[0]<<" "<<cloud.points[i].normal[1]<<" "<<cloud.points[i].normal[2]<<std::endl;
     }
     outf.close();

  }

//include template versions, but we'll compile in non-templated versions

//A helper function for ICP:
//ref should map to target, but not necessarily the other way 'round.  also, target should not be downsampled
template <typename PointT>
int getClosestPoints(pcl::PointCloud<PointT> &ref_cloud, pcl::PointCloud<PointT> &target_cloud, std::vector<int> &ref_pts, std::vector<int> &tgt_pts, double dist_thresh=.2, uint num_pts=500){
   timeval t0=g_tick();
   pcl::KdTreeFLANN<PointT> ttree;
   ttree.setInputCloud(target_cloud.makeShared());
//   ROS_INFO("step 1: initialization  %f secs. ",g_tock(t0));
   t0=g_tick();
   srand(getUsec());
   std::vector<int> indices(1);
   std::vector<float> dists(1);
   ref_pts.clear();
   tgt_pts.clear();
   int ind;

   std::vector<int> notused(ref_cloud.points.size(),1);
   for(uint i=0;i<ref_cloud.points.size();i++)
      notused[i]=i;
   long tcount=0;
   int indsleft=ref_cloud.points.size();
   while(ref_pts.size() < num_pts && indsleft > 10){
      ind=rand() %indsleft;
      if(ttree.nearestKSearch(ref_cloud.points[notused[ind]],1,indices,dists) && dists[0] < dist_thresh)
            if(fabs(ref_cloud.points[notused[ind]].z - target_cloud.points[indices[0]].z) < dist_thresh/10.0 ){
         ref_pts.push_back(notused[ind]);
         tgt_pts.push_back(indices[0]);
      }
      notused[ind]=notused[indsleft-1];
      tcount++;
      indsleft--;
   }
//   ROS_INFO("finding Closest Points took:  %f secs. found %d pts from %d guesses out of %d pts",g_tock(t0),tgt_pts.size(),tcount,target_cloud.points.size());
//   ROS_INFO(" %f secs pre guess",g_tock(t0)/(double)tcount);
   return 0;
}



template <typename PointT>
Eigen::Matrix4f icp2Dt(pcl::PointCloud<PointT> &c1, pcl::PointCloud<PointT> &c2, double max_dist,
      int small_transdiff_countreq=1, int num_pts=500, uint min_pts=10, uint max_iter=50, float transdiff_thresh=.0001 ){
   timeval t0=g_tick(),t1=g_tick();
   std::vector<int> c1pts,c2pts;
   Eigen::Matrix4f transformation_, final_transformation_=Eigen::Matrix4f::Identity(),previous_transformation_,trans2d;
   int small_transdiff_count=0;
   for(uint i=0;i<max_iter;i++){
      t0=g_tick();
      getClosestPoints(c1,c2,c1pts,c2pts,max_dist,num_pts);
      if(c1pts.size() < min_pts){
         ROS_ERROR("not enough correspondences");
         return transformation_;
      }
      previous_transformation_ = final_transformation_;
      pcl::estimateRigidTransformationSVD(c1,c1pts,c2,c2pts,transformation_);
      // Tranform the data

      transformPointCloud (c1, c1, projectTo2D(transformation_));
      // Obtain the final transformation
      final_transformation_ = transformation_ * final_transformation_;
      trans2d=projectTo2D(final_transformation_);
      final_transformation_=trans2d;
      float transdiff=fabs ((final_transformation_ - previous_transformation_).sum ());
      ROS_INFO("icp took:  %f secs. trans diff: %f,  total: %f, %f, %f",g_tock(t0),transdiff,trans2d(0,3),trans2d(1,3),getYaw(trans2d));
      if(transdiff <transdiff_thresh) small_transdiff_count++;
      else small_transdiff_count=0;
      if(small_transdiff_count>small_transdiff_countreq)
         break;
   }
   ROS_INFO("icp took:  %f secs. ",g_tock(t1));
   std::cout<<final_transformation_<<std::endl;
   return final_transformation_;
}


Eigen::Matrix4f icp2D(pcl::PointCloud<pcl::PointXYZINormal> &c1, pcl::PointCloud<pcl::PointXYZINormal> &c2, double max_dist,
      int small_transdiff_countreq, int num_pts, uint min_pts, uint max_iter, float transdiff_thresh){
   timeval t0=g_tick(),t1=g_tick();
   std::vector<int> c1pts,c2pts;
   Eigen::Matrix4f transformation_, final_transformation_=Eigen::Matrix4f::Identity(),previous_transformation_,trans2d;
   int small_transdiff_count=0;
   for(uint i=0;i<max_iter;i++){
      t0=g_tick();
      getClosestPoints(c1,c2,c1pts,c2pts,max_dist,num_pts);
      if(c1pts.size() < min_pts){
         ROS_ERROR("not enough correspondences");
         return transformation_;
      }
      previous_transformation_ = final_transformation_;
      pcl::estimateRigidTransformationSVD(c1,c1pts,c2,c2pts,transformation_);
      // Tranform the data

      transformPointCloudWithNormals (c1, c1, projectTo2D(transformation_));
      // Obtain the final transformation
      final_transformation_ = transformation_ * final_transformation_;
      trans2d=projectTo2D(final_transformation_);
      final_transformation_=trans2d;
      float transdiff=fabs ((final_transformation_ - previous_transformation_).sum ());
      ROS_INFO("icp took:  %f secs. trans diff: %f,  total: %f, %f, %f",g_tock(t0),transdiff,trans2d(0,3),trans2d(1,3),getYaw(trans2d));
      if(transdiff <transdiff_thresh) small_transdiff_count++;
      else small_transdiff_count=0;
      if(small_transdiff_count>small_transdiff_countreq)
         break;
   }
   ROS_INFO("icp took:  %f secs. ",g_tock(t1));
   std::cout<<final_transformation_<<std::endl;
   return final_transformation_;
}

template <typename PointT>
double getClosestPointt(pcl::PointCloud<PointT> &cloud, PointT ref, PointT &point){
   pcl::KdTreeFLANN<PointT> tree;
   tree.setInputCloud(cloud.makeShared());
   std::vector<int> ind(20);
   std::vector<float> dists(20);
   tree.nearestKSearch(ref,1,ind,dists);
   point=cloud.points[ind[0]];
   return dists[0];
}

template <typename PointT>
void getSubCloudt(pcl::PointCloud<PointT> &cloudin,  std::vector<int> &ind, pcl::PointCloud<PointT> &cloudout,bool use_positive=true){
   pcl::ExtractIndices<PointT> extract;
   // Extract the inliers
   extract.setInputCloud (cloudin.makeShared());
   extract.setIndices (boost::make_shared<std::vector<int> > (ind));
   extract.setNegative (!use_positive);
   extract.filter (cloudout);
//    ROS_INFO ("Subcloud representing the planar component: %d data points.", cloudout.width * cloudout.height);

}

template <typename PointT>
void segmentHeightt(pcl::PointCloud<PointT> &cloudin, pcl::PointCloud<PointT> &cloudout,double lowest,double highest, bool use_positive=true){
   timeval t0=g_tick();
   std::vector<int> ind;
   for(uint i=0;i<cloudin.points.size(); i++){
      if(cloudin.points[i].z > lowest && cloudin.points[i].z < highest){ //|| cloudin.points[i].z <.1){
         ind.push_back(i);
      }
   }
   getSubCloud(cloudin,ind,cloudout,use_positive);
   std::cout<<"segmentHeight took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}

template <typename PointT>
void removeOutlierst(pcl::PointCloud<PointT> &cloudin, pcl::PointCloud<PointT> &cloudout){
   timeval t0=g_tick();
   pcl::StatisticalOutlierRemoval<PointT> sor2;
   sor2.setInputCloud (cloudin.makeShared());
   sor2.setMeanK (50);
   sor2.setStddevMulThresh (1.0);
   sor2.filter (cloudout);
   std::cout<<"filtering outliers took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}

template <typename PointT>
void downSamplet(pcl::PointCloud<PointT> &cloudin, pcl::PointCloud<PointT> &cloudout, double leafsize=.01){
   timeval t0=g_tick();
   // Create the filtering object
   pcl::VoxelGrid<PointT> sor;
   sor.setInputCloud (cloudin.makeShared());
   sor.setLeafSize (leafsize, leafsize, leafsize);
   sor.filter (cloudout);
   std::cout<<"downsampling took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}

void myFlipNormals(double vx, double vy, double vz, pcl::PointCloud<pcl::PointXYZINormal> &ncloud){
   for(uint i=0;i<ncloud.points.size();i++){
      pcl::PointXYZINormal p=ncloud.points[i];
      double dotprod=(p.normal[0]*(vx-p.x)+p.normal[1]*(vy-p.y)+p.normal[2]*(vz-p.z));
      if(dotprod<0){
//       cout<<"flipping"<<std::endl;
         ncloud.points[i].normal[0]*=-1.0;
         ncloud.points[i].normal[1]*=-1.0;
         ncloud.points[i].normal[2]*=-1.0;
      }
   }
}


void getNormals(pcl::PointCloud<pcl::PointWithViewpoint> &cloudin, pcl::PointCloud<pcl::PointXYZINormal> &cloud_normals){
   timeval t0=g_tick();
   pcl::PointCloud<pcl::PointWithViewpoint>::ConstPtr cloud_ = cloudin.makeShared();
   pcl::KdTree<pcl::PointWithViewpoint>::Ptr normals_tree_;
    int k_ = 10;                                // 50 k-neighbors by default
    normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointWithViewpoint> > ();
    pcl::NormalEstimation<pcl::PointWithViewpoint, pcl::PointXYZINormal> n3d_;
    n3d_.setKSearch (k_);
    n3d_.setSearchMethod (normals_tree_);
    n3d_.setInputCloud (cloud_);
    n3d_.compute (cloud_normals);
    for(uint i=0;i<cloudin.points.size(); i++){
      cloud_normals.points[i].x=cloudin.points[i].x;
      cloud_normals.points[i].y=cloudin.points[i].y;
      cloud_normals.points[i].z=cloudin.points[i].z;
    }
    myFlipNormals(cloudin.points[0].vp_x,cloudin.points[0].vp_y,cloudin.points[0].vp_z,cloud_normals);
   std::cout<<"finding normals took:  "<<g_tock(t0)<<"  for "<<cloud_normals.points.size()<<" indices."<<std::endl;
}


void getNormals(pcl::PointCloud<pcl::PointXYZ> &cloudin, pcl::PointCloud<pcl::PointXYZINormal> &cloud_normals, pcl::PointXYZ vp){
   timeval t0=g_tick();
   pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_ = cloudin.makeShared();
   pcl::KdTree<pcl::PointXYZ>::Ptr normals_tree_;
    int k_ = 10;                                // 50 k-neighbors by default
    normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointXYZINormal> n3d_;
    n3d_.setKSearch (k_);
    n3d_.setSearchMethod (normals_tree_);
    n3d_.setInputCloud (cloud_);
    n3d_.compute (cloud_normals);
    for(uint i=0;i<cloudin.points.size(); i++){
      cloud_normals.points[i].x=cloudin.points[i].x;
      cloud_normals.points[i].y=cloudin.points[i].y;
      cloud_normals.points[i].z=cloudin.points[i].z;
    }
    myFlipNormals(vp.x,vp.y,vp.z,cloud_normals);
   std::cout<<"finding normals took:  "<<g_tock(t0)<<"  for "<<cloud_normals.points.size()<<" indices."<<std::endl;
}



template <typename PointT>
double ptdist(PointT a,PointT b){
   return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);
}


template <typename PointT>
void segfastt(pcl::PointCloud<PointT> &cloud, std::vector<pcl::PointCloud<PointT> > &cloud_clusters, double cluster_tol=.2){
   double smaller_tol = cluster_tol/2.3;
   timeval t0=g_tick();
   pcl::KdTreeFLANN<PointT> tree,tree2;
   tree.setInputCloud(cloud.makeShared());
   std::vector<int> minitree(cloud.points.size(),-1);
   std::vector<int> heads,indices;
   std::vector<float> dists;

   //First 'downsample' the points into clusters slightly less than half the cluster tolerance
   //minitree keeps track of who each point belongs to (which is an index of the 'head' array)
   for(uint i=0; i<cloud.points.size();i++){
     if(minitree[i]==-1){    //if no one has claimed this point, make it a head
        heads.push_back(i);
        if(!tree.radiusSearch(cloud,i,smaller_tol,indices,dists))  //find all the points close to this point
        	std::cout<<"radius search failed!"<<std::endl;
        for(uint j=0;j<indices.size();j++){
           minitree[indices[j]]=heads.size()-1; //assign these points to the current head.
                                       // this overwrites previous claims, but it's ok
        }
     }
   }
// std::cout<<"radiusSearch took:  "<<g_tock(t0)<<"s  found "<<heads.size()<<" heads"<<std::endl;

   //now, we have much fewer points to cluster, but since the initial downsampling was less than
   //half the cluster tolerance,we are guaranteed to find all the points within the tolerance
   //(except if  [ clustertol < distance between heads < cluster_tol+ smaller cluster tol] AND the head closest points in the two heads are < cluster_tol
   //I need to make something to deal with that...
   tree2.setInputCloud(cloud.makeShared(),boost::make_shared<std::vector<int> >  (heads));
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
        	   std::cout<<"radius search failed!"<<std::endl;
//         cout<<i<<" --> "<<searching<<" --> "<<indices.size()<<std::endl;
           for(uint j=0;j<indices.size();j++)
              if(minitree2[indices[j]]==-1){//found untouched point (which means this root touched it)
                 minitree2[indices[j]]=currenthead; //claim it
                 tosearch.push_back(indices[j]);   //add it to list of points to search
              }
        }
     }

   }

   timeval t1=g_tick();
   std::vector<int> indices1;
   std::vector<float> dists1;
   std::vector<int> deletedclusters;
   //now we need to check to see if any of the heads that were NOT clustered together are closer than cluster_tol+smaller_tol:
   for(uint i=0; i<minitree2.size();i++){
      tree2.radiusSearch(cloud.points[heads[i]],cluster_tol+smaller_tol,indices,dists);
      for(uint j=0;j<indices.size();j++)
         if(minitree2[indices[j]] != minitree2[i] && i<j){//if the two heads are not in the same cluster
        	 std::cout<<"head "<<i<<" ("<<minitree2[i]<<") is "<<sqrt(dists[j])<<" from head "<<indices[j]<<" ("<<minitree2[indices[j]]<<")"<<std::endl;
            //now we have to find if there is a point between head a and head b that is < cluster_tol from both
            PointT heada=cloud.points[heads[i]], headb=cloud.points[heads[indices[j]]];
            PointT inbetween;
            inbetween.x=(heada.x+headb.x)/2.0;
            inbetween.y=(heada.y+headb.y)/2.0;
            inbetween.z=(heada.z+headb.z)/2.0;

            tree.radiusSearch(inbetween,cluster_tol,indices1,dists1); //search in the full tree
            double distthresh=cluster_tol*cluster_tol; //radius search gives squared distances...
            for(uint k=0;k<indices1.size();k++){
               if(ptdist(cloud.points[indices1[k]],heada ) < distthresh && ptdist(cloud.points[indices1[k]],headb ) < distthresh ){
                  //there is a point that these two clusters share -> they should be merged
            	   std::cout<<"head "<<i<<" ("<<minitree2[i]<<") shares "<<indices1[k]<<" with head "<<indices[j]<<" ("<<minitree2[indices[j]]<<")"<<std::endl;
                  //rename all heads in cluster b to cluster a
                  int clusterb=minitree2[indices[j]];
                  for(uint m=0;m<minitree2.size();m++){
                     if(minitree2[m]==clusterb) minitree2[m]=minitree2[i];
                  }
                  deletedclusters.push_back(clusterb);
                  break;
               }

            }

         }
   }

   std::cout<<"checking overlaps took:  "<<g_tock(t1)<<"s  found "<<deletedclusters.size()<<" overlaps"<<std::endl;





   std::cout<<"radiusSearch took:  "<<g_tock(t0)<<"s  found "<<heads2.size()<<" heads"<<std::endl;
   std::vector<std::vector<int> > clusters(heads2.size());
   //now, for each point in the cloud find it's head --> then the head it clustered to. that is it's cluster id!
   for(uint j=0;j<minitree.size();j++){
     clusters[minitree2[minitree[j]]].push_back(j);
   }


   std::cout<<"clustering took:  "<<g_tock(t0)<<"s  found "<<heads2.size()<<" heads"<<std::endl;
   for(uint j=0;j<heads2.size();j++){
	   std::cout<<"cluster "<<j<<"  -->  "<<clusters[j].size()<<std::endl;
   }

   //erase the deleted clusters
   bool loop=true;
   while(loop){
      loop=false;
      for(uint i=0;i<clusters.size(); i++)
         if(clusters[i].size()==0){
            clusters.erase(clusters.begin()+i);
            loop=true;
         }


   }

   cloud_clusters.resize(clusters.size());
   for(uint i=0;i<clusters.size(); i++){
     getSubCloud(cloud,clusters[i],cloud_clusters[i]);
   }

}








//non-templated versions so we can compile this into a nice library:
Eigen::Matrix4f icp2D(pcl::PointCloud<pcl::PointXYZ> &c1, pcl::PointCloud<pcl::PointXYZ> &c2, double max_dist,
     int small_transdiff_countreq, int num_pts, uint min_pts, uint max_iter, float transdiff_thresh){
   return icp2Dt(c1,c2,max_dist,small_transdiff_countreq,num_pts,min_pts,max_iter,transdiff_thresh);
}
double getClosestPoint(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointXYZ ref, pcl::PointXYZ &point){
   return getClosestPointt(cloud,ref,point);
}
void getSubCloud(pcl::PointCloud<pcl::PointXYZ> &cloudin,  std::vector<int> &ind, pcl::PointCloud<pcl::PointXYZ> &cloudout,bool use_positive){
   getSubCloudt(cloudin,ind,cloudout,use_positive);
}
void segmentHeight(pcl::PointCloud<pcl::PointXYZ> &cloudin, pcl::PointCloud<pcl::PointXYZ> &cloudout,double lowest,double highest, bool use_positive){
   segmentHeightt(cloudin,cloudout,lowest,highest,use_positive);
}
void removeOutliers(pcl::PointCloud<pcl::PointXYZ> &cloudin, pcl::PointCloud<pcl::PointXYZ> &cloudout){
   removeOutlierst(cloudin,cloudout);
}
void downSample(pcl::PointCloud<pcl::PointXYZ> &cloudin, pcl::PointCloud<pcl::PointXYZ> &cloudout, double leafsize){
   downSamplet(cloudin,cloudout,leafsize);
}


//Eigen::Matrix4f icp2D(pcl::PointCloud<pcl::PointXYZINormal> &c1, pcl::PointCloud<pcl::PointXYZINormal> &c2, double max_dist,
//     int small_transdiff_countreq, int num_pts, uint min_pts, uint max_iter, float transdiff_thresh ){
//   return icp2Dt(c1,c2,max_dist,small_transdiff_countreq,num_pts,min_pts,max_iter,transdiff_thresh);
//}
double getClosestPoint(pcl::PointCloud<pcl::PointXYZINormal> &cloud, pcl::PointXYZINormal ref, pcl::PointXYZINormal &point){
   return getClosestPointt(cloud,ref,point);
}
void getSubCloud(pcl::PointCloud<pcl::PointXYZINormal> &cloudin,  std::vector<int> &ind, pcl::PointCloud<pcl::PointXYZINormal> &cloudout,bool use_positive){
   getSubCloudt(cloudin,ind,cloudout,use_positive);
}
void segmentHeight(pcl::PointCloud<pcl::PointXYZINormal> &cloudin, pcl::PointCloud<pcl::PointXYZINormal> &cloudout,double lowest,double highest, bool use_positive){
   segmentHeightt(cloudin,cloudout,lowest,highest,use_positive);
}
void removeOutliers(pcl::PointCloud<pcl::PointXYZINormal> &cloudin, pcl::PointCloud<pcl::PointXYZINormal> &cloudout){
   removeOutlierst(cloudin,cloudout);
}
void downSample(pcl::PointCloud<pcl::PointXYZINormal> &cloudin, pcl::PointCloud<pcl::PointXYZINormal> &cloudout, double leafsize){
   downSamplet(cloudin,cloudout,leafsize);
}



Eigen::Matrix4f icp2D(pcl::PointCloud<pcl::PointWithViewpoint> &c1, pcl::PointCloud<pcl::PointWithViewpoint> &c2, double max_dist,
     int small_transdiff_countreq, int num_pts, uint min_pts, uint max_iter, float transdiff_thresh){
   return icp2Dt(c1,c2,max_dist,small_transdiff_countreq,num_pts,min_pts,max_iter,transdiff_thresh);
}
double getClosestPoint(pcl::PointCloud<pcl::PointWithViewpoint> &cloud, pcl::PointWithViewpoint ref, pcl::PointWithViewpoint &point){
   return getClosestPointt(cloud,ref,point);
}
void getSubCloud(pcl::PointCloud<pcl::PointWithViewpoint> &cloudin,  std::vector<int> &ind, pcl::PointCloud<pcl::PointWithViewpoint> &cloudout,bool use_positive){
   getSubCloudt(cloudin,ind,cloudout,use_positive);
}
void segmentHeight(pcl::PointCloud<pcl::PointWithViewpoint> &cloudin, pcl::PointCloud<pcl::PointWithViewpoint> &cloudout,double lowest,double highest, bool use_positive){
   segmentHeightt(cloudin,cloudout,lowest,highest,use_positive);
}
void removeOutliers(pcl::PointCloud<pcl::PointWithViewpoint> &cloudin, pcl::PointCloud<pcl::PointWithViewpoint> &cloudout){
   removeOutlierst(cloudin,cloudout);
}
void downSample(pcl::PointCloud<pcl::PointWithViewpoint> &cloudin, pcl::PointCloud<pcl::PointWithViewpoint> &cloudout, double leafsize){
   downSamplet(cloudin,cloudout,leafsize);
}


void segfast(pcl::PointCloud<pcl::PointXYZ> &cloud, std::vector<pcl::PointCloud<pcl::PointXYZ> > &cloud_clusters, double cluster_tol){
   segfast(cloud,cloud_clusters,cluster_tol);
}
void segfast(pcl::PointCloud<pcl::PointXYZINormal> &cloud, std::vector<pcl::PointCloud<pcl::PointXYZINormal> > &cloud_clusters, double cluster_tol){
   segfast(cloud,cloud_clusters,cluster_tol);
}
void segfast(pcl::PointCloud<pcl::PointWithViewpoint> &cloud, std::vector<pcl::PointCloud<pcl::PointWithViewpoint> > &cloud_clusters, double cluster_tol){
   segfast(cloud,cloud_clusters,cluster_tol);
}


