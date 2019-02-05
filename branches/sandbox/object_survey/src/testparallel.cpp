/*
 * testparallel.cpp
 *
 *  Created on: Nov 5, 2010
 *      Author: garratt
 */


#include <iostream>
#include <vector>
#include <fstream>

//#include <eigen_conversions/eigen_msg.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>


#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/registration/registration.h"
#include "pcl/registration/icp.h"
#include <sys/time.h>
#include "pcl_tf/transforms.h"
//
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"

#include "pcl/filters/passthrough.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_ann.h"
#include "pcl/kdtree/organized_data.h"
#include <list>
#include <fstream>
#include <vector>

#include "pcl_tools/pcl_utils.h"

typedef pcl::PointCloud<pcl::PointXYZINormal> NCloud;
typedef pcl::PointXYZINormal NPoint;

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
//double getYaw(Eigen3::Matrix4f etrans){
//   tf::Transform tftrans=tfFromEigen(etrans);
//   return tf::getYaw(tftrans.getRotation());
//}
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
//template <typename PointT>
//int getClosestPoints(pcl::PointCloud<PointT> &ref_cloud, pcl::PointCloud<PointT> &target_cloud, std::vector<int> &ref_pts, std::vector<int> &tgt_pts, double dist_thresh=.2, uint num_pts=500){
//   timeval t0=g_tick();
//   pcl::KdTreeFLANN<PointT> ttree;
//   ttree.setInputCloud(target_cloud.makeShared());
////   ROS_INFO("step 1: initialization  %f secs. ",g_tock(t0));
//   t0=g_tick();
//   srand(getUsec());
//   std::vector<int> indices(1);
//   std::vector<float> dists(1);
//   ref_pts.clear();
//   tgt_pts.clear();
//   int ind;
//
//   std::vector<int> notused(ref_cloud.points.size(),1);
//   for(uint i=0;i<ref_cloud.points.size();i++)
//      notused[i]=i;
//   long tcount=0;
//   int indsleft=ref_cloud.points.size();
//   while(ref_pts.size() < num_pts && indsleft > 10){
//      ind=rand() %indsleft;
//      if(ttree.nearestKSearch(ref_cloud.points[notused[ind]],1,indices,dists) && dists[0] < dist_thresh)
//            if(fabs(ref_cloud.points[notused[ind]].z - target_cloud.points[indices[0]].z) < dist_thresh/10.0 ){
//         ref_pts.push_back(notused[ind]);
//         tgt_pts.push_back(indices[0]);
//      }
//      notused[ind]=notused[indsleft-1];
//      tcount++;
//      indsleft--;
//   }
////   ROS_INFO("finding Closest Points took:  %f secs. found %d pts from %d guesses out of %d pts",g_tock(t0),tgt_pts.size(),tcount,target_cloud.points.size());
////   ROS_INFO(" %f secs pre guess",g_tock(t0)/(double)tcount);
//   return 0;
//}



//template <typename PointT>
//Eigen3::Matrix4f icp2Dt(pcl::PointCloud<PointT> &c1, pcl::PointCloud<PointT> &c2, double max_dist,
//      int small_transdiff_countreq=1, int num_pts=500, uint min_pts=10, uint max_iter=50, float transdiff_thresh=.0001 ){
//   timeval t0=g_tick(),t1=g_tick();
//   std::vector<int> c1pts,c2pts;
//   Eigen3::Matrix4f transformation_, final_transformation_=Eigen3::Matrix4f::Identity(),previous_transformation_,trans2d;
//   int small_transdiff_count=0;
//   for(uint i=0;i<max_iter;i++){
//      t0=g_tick();
//      getClosestPoints(c1,c2,c1pts,c2pts,max_dist,num_pts);
//      if(c1pts.size() < min_pts){
//         return transformation_;
//      }
//      previous_transformation_ = final_transformation_;
//      pcl::estimateRigidTransformationSVD(c1,c1pts,c2,c2pts,transformation_);
//      // Tranform the data
//
//      transformPointCloud (c1, c1, projectTo2D(transformation_));
//      // Obtain the final transformation
//      final_transformation_ = transformation_ * final_transformation_;
//      trans2d=projectTo2D(final_transformation_);
//      final_transformation_=trans2d;
//      float transdiff=fabs ((final_transformation_ - previous_transformation_).sum ());
//      if(transdiff <transdiff_thresh) small_transdiff_count++;
//      else small_transdiff_count=0;
//      if(small_transdiff_count>small_transdiff_countreq)
//         break;
//      std::cout<<"."<<std::endl;
//   }
//   std::cout<<final_transformation_<<std::endl;
//   return final_transformation_;
//}


//
//timeval g_tick(){
//   struct timeval tv;
//   gettimeofday(&tv, NULL);
//   return tv;
//}
//
//double g_tock(timeval tprev)
//{
//   struct timeval tv;
//   gettimeofday(&tv, NULL);
//   return (double)(tv.tv_sec-tprev.tv_sec) + (tv.tv_usec-tprev.tv_usec)/1000000.0;
//}
//
//int getUsec(){
//   struct timeval tv;
//   gettimeofday(&tv, NULL);
//   return tv.tv_usec;
//}

//just a little helper function to clean up my code.
//computes the min, max and average of the vector, optionally only looking at the first n
void minmaxave(std::vector<float> &in, float &min, float &max, float &ave, int end=-1){
	if(end<0 || end > in.size()) end=in.size();
	ave=0.0;
	//throw out first one, which is self...
    for(uint i=1;i<end;i++){
        ave+=in[i];
        if(in[i]>max || i==1) max=in[i];
        if(in[i] < min || i==1) min=in[i];
    }
    ave/=(float)end-1;
}

struct ptInfo{

   std::vector<int> indices;
   std::vector<float> dists;
   std::vector<float> normdists;
   std::vector<float> axisdists;

   //nn statistics:

   float normnear_dist;
   float normave_dist;
   float normfar_dist;
   float near_dist;
   float ave_dist;
   float far_dist;
   float axisnear_dist;
   float axisave_dist;
   float axisfar_dist;

   float normnear_dist2;
   float normave_dist2;
   float normfar_dist2;
   float near_dist2;
   float ave_dist2;
   float far_dist2;
   float axisnear_dist2;
   float axisave_dist2;
   float axisfar_dist2;

   void resize(int i){
      indices.resize(i);
      dists.resize(i);
      normdists.resize(i);
      axisdists.resize(i);

   }
   void calcStats(){
	  int nearfeats=5;
	  minmaxave( dists,     near_dist,     far_dist,     ave_dist);
	  minmaxave( normdists, normnear_dist, normfar_dist, normave_dist);
	  minmaxave( axisdists, axisnear_dist, axisfar_dist, axisave_dist);

	  minmaxave( dists,     near_dist2,     far_dist2,     ave_dist2,nearfeats);
	  minmaxave( normdists, normnear_dist2, normfar_dist2, normave_dist2,nearfeats);
	  minmaxave( axisdists, axisnear_dist2, axisfar_dist2, axisave_dist2,nearfeats);
   }

   void saveout(std::ostream &outf){
	   outf<<" "<<near_dist<<" "<<far_dist<<" "<<ave_dist<<" ";
	   outf<<" "<<normnear_dist<<" "<<normfar_dist<<" "<<normave_dist<<" ";
	   outf<<" "<<axisnear_dist<<" "<<axisfar_dist<<" "<<axisave_dist<<" ";
	   outf<<" "<<near_dist2<<" "<<far_dist2<<" "<<ave_dist2<<" ";
	   outf<<" "<<normnear_dist2<<" "<<normfar_dist2<<" "<<normave_dist2<<" ";
	   outf<<" "<<axisnear_dist2<<" "<<axisfar_dist2<<" "<<axisave_dist2<<" ";
   }
   //8:  near_dist
   //9:  far_dist
   //10: ave_dist
   //11: normnear_dist
   //12: normfar_dist
   //13: normave_dist
   //14: axisnear_dist
   //15: axisfar_dist
   //16: axisave_dist
   //17:  near_dist2
   //18:  far_dist2
   //19: ave_dist2
   //20: normnear_dist2
   //21: normfar_dist2
   //22: normave_dist2
   //23: axisnear_dist2
   //24: axisfar_dist2
   //25: axisave_dist2





};




class Ptester {

   boost::mutex data_mutex,dist_mutex,ind_mutex;
   boost::mutex print_mutex;
   std::vector< NCloud > clouds;
   std::vector< std::vector<double> > ndists;
   std::vector<Eigen3::Matrix4f> alignments;

   std::vector<ptInfo> pt_info;
   std::vector< std::vector<int> > nnindices;
   std::vector< std::vector<float> > nndists;
//   std::vector< std::vector<float> > nnnormdists;
//
//   //cloud statistics:
//   std::vector<float>  nnnormave_dist;
//   std::vector<float>  nnnormnear_dist;
//   std::vector<float>  nnnormfar_dist;
//
//   std::vector<float>  nnave_dist;
//   std::vector<float>  nnnear_dist;
//   std::vector<float>  nnfar_dist;


   void Pflann(int s, int t){
      uint num_samples=10000;
      //copy cloud
      timeval t0=g_tick();
      NCloud target_cloud, ref_cloud;
      {
        boost::mutex::scoped_lock  lock(print_mutex);
        ROS_INFO("aligning clouds %d and %d",s,t);
      }
      {
        boost::mutex::scoped_lock  lock(data_mutex);
        target_cloud=clouds[t];
        ref_cloud=clouds[s];
      }

      //get array of random ints to sample in:
      int indmax=ref_cloud.size();
      vector<int> testpts(num_samples,-1), closestpts(num_samples,-1);
      vector<double> closestdists(num_samples,-1.0);
      srand(getUsec());
      for(uint i=0;i<num_samples;i++){
         testpts[i]=rand() %indmax;
      }

      //setup tree:
      pcl::KdTreeFLANN<NPoint> ttree;
      ttree.setInputCloud(target_cloud.makeShared());

      std::vector<int> indices(1);
      std::vector<float> dists(1);


      for(uint i=0;i<num_samples;i++){
         if(ttree.nearestKSearch(ref_cloud.points[testpts[i]],1,indices,dists)){
            closestpts[i]=indices[0];
            closestdists[i]=dists[0];
         }
         else{
              boost::mutex::scoped_lock  lock(print_mutex);
              ROS_ERROR("Nearest K search failed! while aligning clouds %d and %d",s,t);
         }
      }
      {
          boost::mutex::scoped_lock  lock(print_mutex);
          ROS_INFO("aligning clouds %d and %d completed in %f",s,t,g_tock(t0));
      }
      {
        boost::mutex::scoped_lock  lock(data_mutex);
        ndists[t]=closestdists;
      }

   }

    void Picp2D(int t){
      timeval t0=g_tick();
     //assemble cloud:
     pcl::PointCloud<pcl::PointXYZINormal> full_cloud, aligned;//,original;
     {
        boost::mutex::scoped_lock  lock(data_mutex);
        if(t==0)
           full_cloud=clouds[1];
        else
           full_cloud=clouds[0];

        for (uint i = 1; i < clouds.size(); ++i) {
           if(i!=t)
              full_cloud+=clouds[i];
        }
        aligned=clouds[t];
     }
     {
         boost::mutex::scoped_lock  lock(print_mutex);
         ROS_INFO("Creating cloud %d took %f",t,g_tock(t0));
     }

     Eigen3::Matrix4f trans2d;
     trans2d=icp2D(clouds[t],full_cloud,.3,2,500,10,10,.0001);
     {
         boost::mutex::scoped_lock  lock(print_mutex);
         ROS_INFO("aligning cloud %d completed in %f",t,g_tock(t0));
     }
     {
        boost::mutex::scoped_lock  lock(data_mutex);
        alignments[t]=trans2d;
     }

   }

   void characterizeSearch(int t) {
      int nn=1000;
      int num_samples=1000;
      timeval t0=g_tick();
      NCloud target_cloud;
      {
        boost::mutex::scoped_lock  lock(data_mutex);
        target_cloud=clouds[t];

      }
      cout<<"size: "<<target_cloud.points.size();
      pcl::KdTreeFLANN<NPoint> ttree;
      ttree.setInputCloud(target_cloud.makeShared());
      std::vector<int> indices(nn);
      std::vector<float> dists(nn);

//      ROS_INFO("setting up %d took %f",t,g_tock(t0));
      cout<<"  \tsetup: "<<g_tock(t0);
      t0=g_tick();
      for(uint i=0;i<num_samples;i++)
       ttree.nearestKSearch(target_cloud.points[i],nn,indices,dists);
       cout<<"  \tsearch: "<<g_tock(t0)<<endl;;
//       ROS_INFO("Search for %d nn took %f",nn,g_tock(t0));


   }

   double getNormalDist(NPoint &p1, NPoint &p2){
      return p1.normal[0]*p2.normal[0]+p1.normal[1]*p2.normal[1]+p1.normal[2]*p2.normal[2];
   }
   double getAxisDist(NPoint &p1, NPoint &p2){
      return p1.normal[0]*fabs(p2.x-p1.x)+p1.normal[1]*fabs(p2.z-p1.z)+p1.normal[2]*fabs(p2.z-p1.z);
   }

   void findSomeNN(int t, int start, int end,int nn){
      timeval t0=g_tick();
      double time0,time1;
      NCloud target_cloud;
      {
        boost::mutex::scoped_lock  lock(data_mutex);
        target_cloud=clouds[t];

      }

      pcl::KdTreeFLANN<NPoint> ttree;
      ttree.setInputCloud(target_cloud.makeShared());
//      std::vector< std::vector<int> > indices(end-start);
//      std::vector< std::vector<float> > dists(end-start);
//      std::vector< std::vector<float> > normdists(end-start);
      std::vector< ptInfo > info(end-start);

      t0=g_tick();
      for(int i=0;i<end-start;i++){
         info[i].resize(nn);
         ttree.nearestKSearch(target_cloud.points[i+start],nn,info[i].indices,info[i].dists);
            for(int j=0;j<nn;j++){
               info[i].normdists[j]=getNormalDist(target_cloud.points[i+start],target_cloud.points[info[i].indices[j]]);
               info[i].axisdists[j]=getAxisDist(target_cloud.points[i+start],target_cloud.points[info[i].indices[j]])/sqrt(info[i].dists[j]);
            }
            info[i].calcStats();

      }
      time0=g_tock(t0);
      t0=g_tick();
       {
         boost::mutex::scoped_lock  lock(dist_mutex);
         for(int i=start;i<end;i++){
            pt_info[i]=info[i-start];
         }
       }
//       {
//         boost::mutex::scoped_lock  lock(ind_mutex);
//         for(int i=start;i<end;i++){
//            nnindices[i]=indices[i-start];
//         }
//       }
//       {
//         boost::mutex::scoped_lock  lock(dist_mutex);
//         for(int i=start;i<end;i++){
//            nnnormdists[i]=normdists[i-start];
//         }
//       }
//         std::copy(indices.begin(),indices.end(),nnindices.begin()+start);
//         std::copy(dists.begin(),dists.end(),nndists.begin()+start);


      time1=g_tock(t0);

       {
           boost::mutex::scoped_lock  lock(print_mutex);
           ROS_INFO("finding nn from %d to %d completed in %f (%f,%f), averaging %f",start,end,time0+time1,time0,time1,(time0+time1)/((float)end-start));
       }
   }
   void findAllNN(int t, int nn){
//      nndists.resize(clouds[t].size()+1);
//      nnindices.resize(clouds[t].size()+1);
//      nnnormdists.resize(clouds[t].size()+1);
      pt_info.resize(clouds[t].size());

      int threadnum=12;
      int incriment=clouds[t].size()/threadnum;
      boost::thread_group threads;
      for(int i=0;i<clouds[t].size()-incriment;i+=incriment){
         if(i+2*incriment < clouds[t].size())
            threads.create_thread(boost::bind(&Ptester::findSomeNN,this,t,i,i+incriment,nn));
         else
            threads.create_thread(boost::bind(&Ptester::findSomeNN,this,t,i,clouds[t].size(),nn));
      }
      //wait for everything to finish
      threads.join_all();

   }


public:

   void PTest(){
      timeval t0=g_tick();
      boost::thread_group threads;
      ndists.resize(clouds.size());
      alignments.resize(clouds.size());
      for(uint i=0;i<clouds.size();i++){
//         uint target=(i+1)%clouds.size(); //wrap around to beginning
         {
             boost::mutex::scoped_lock  lock(print_mutex);
//             ROS_INFO("Creating thread to match %d to %d",i,target);
             ROS_INFO("Creating thread to align %d",i);
         }
//         threads.create_thread(boost::bind(&Ptester::Pflann,this,i,target));
         threads.create_thread(boost::bind(&Ptester::Picp2D,this,i));
      }
      //wait for everything to finish
      threads.join_all();

      ROS_INFO("\n\nAll cloud matching took %f",g_tock(t0));
   }

   void PTest2(){
      timeval t0=g_tick();
//      boost::thread_group threads;
      ndists.resize(clouds.size());
      alignments.resize(clouds.size());
      for(uint i=0;i<clouds.size();i++){
//         findAllNN(i);
//         threads.create_thread(boost::bind(&Ptester::Picp2D,this,i));
      }
      //wait for everything to finish
//      threads.join_all();

   }


   void PTest3(){

      timeval t0=g_tick();
//      boost::thread_group threads;
//      ndists.resize(clouds.size());
////      alignments.resize(clouds.size());
//      pt_info.resize(clouds.size());
      for(uint i=0;i<clouds.size();i++){
         t0=g_tick();
         findAllNN(i,1);
         ROS_INFO("nn for cloud %d took %f",clouds[i].size(),g_tock(t0));
         cout<<"-----------------------------------------------------------"<<endl<<endl;
//         threads.create_thread(boost::bind(&Ptester::Picp2D,this,i));
      }
      //wait for everything to finish
//      threads.join_all();

   }


   void generateClouds(){

      timeval t0=g_tick();
      srand(getUsec());
      for(uint j=0;j<10;j++){
         clouds.push_back(NCloud());
         NPoint pt;
         for(uint i=0;i<1000*pow(2,j);i++){
            pt.x=  ((float)(rand()%10000))/1000.0;
            pt.y=  ((float)(rand()%10000))/1000.0;
            pt.z=  ((float)(rand()%10000))/1000.0;
            clouds[j].push_back(pt);
         }
      }
      ROS_INFO("Generating clouds took %f",g_tock(t0));
   }

   void loadCloud(string filename){
      clouds.push_back(NCloud());
      pcl::io::loadPCDFile(filename,clouds.back());
   }

   void saveStats(){
     std::ofstream outf;
     outf.open("cloudstats.txt",std::ios::out);
     for(int i=0;i<pt_info.size();i++){
        outf<<i<<" "<<clouds[0].points[i].x<<" "<<clouds[0].points[i].y<<" "<<clouds[0].points[i].z;
        outf<<" "<<clouds[0].points[i].normal[0]<<" "<<clouds[0].points[i].normal[1]<<" "<<clouds[0].points[i].normal[2];
        pt_info[i].saveout(outf);
        if(clouds[0].points[i].x > .66 && clouds[0].points[i].x < .76 &&
        	clouds[0].points[i].z > .53 && clouds[0].points[i].z < .7	)
        	outf<<" yes";
        else
        	outf<<" no";
        outf<<endl;
     }
     outf.close();
		system("grep yes cloudstats.txt > statsf");
    // sort -n -k 8 cloudstats.txt | head -17146 | tail -10425 | sort -n -k 10 | tail -1340 > cloudstatf
   }
   //8:  near_dist
   //9:  far_dist
   //10: ave_dist
   //11: normnear_dist
   //12: normfar_dist
   //13: normave_dist
   //14: axisnear_dist
   //15: axisfar_dist
   //16: axisave_dist
   //17:  near_dist2
   //18:  far_dist2
   //19: ave_dist2
   //20: normnear_dist2
   //21: normfar_dist2
   //22: normave_dist2
   //23: axisnear_dist2
   //24: axisfar_dist2
   //25: axisave_dist2

   void makeidentifier(){
	   NCloud temp;
		for(uint i=0;i<clouds[0].size();i++){
//			if(pt_info[i].normave_dist2  < .20 )
//				clouds[0].points[i].intensity+=.2;

			if(pt_info[i].ave_dist2  > .0002)
				clouds[0].points[i].intensity+=.2;
			if(pt_info[i].axisfar_dist2  > .800)
				clouds[0].points[i].intensity+=.1;

			if(clouds[0].points[i].intensity<.01)
				temp.push_back(clouds[0].points[i]);
//			if(pt_info[i].normnear_dist < -.8 ){
//				clouds[0].points[i].intensity=.1;
//				if(pt_info[i].normnear_dist2 < -.5 )
//					clouds[0].points[i].intensity=.2;
//			}
		}
		writePLY(temp,"testcloud.ply");
		pcl::io::savePCDFileASCII("identifiercloud.pcd",clouds[0]);
   }

   void makeplots(){
	   fstream fout;

		system("grep yes cloudstats.txt > statsf");
	   char* descs[] = {"0","i","x","y","z","nx","ny","nz","near_dist", "far_dist", "ave_dist", "normnear_dist", "normfar_dist", "normave_dist",
				"axisnear_dist", "axisfar_dist", "axisave_dist", "near_dist2", "far_dist2", "ave_dist2",
				"normnear_dist2", "normfar_dist2", "normave_dist2", "axisnear_dist2", "axisfar_dist2", "axisave_dist2"};
	   int counter=0;
	   for(uint i=8;i<25;i++){
		   cout<<i<<endl;
		   for(uint j=i+1;j<26;j++){
//	   {{int i=2;int j=4;
				fout.open("plots/plot.gps",fstream::out|fstream::trunc);
				fout<<"set xlabel '"<<descs[i]<<"'"<<endl;
				fout<<"set ylabel '"<<descs[j]<<"'"<<endl;
				fout<<"set terminal png"<<endl;
				fout<<"set output 'plots/plot"<<counter<<".png'"<<endl;
				fout<<"plot 'cloudstats.txt' using "<<i<<":"<<j<<" t 'all pts', ";
				fout<<"'statsf' using "<<i<<":"<<j<<" t 'selected' "<<endl;
				fout.close();
				system("gnuplot plots/plot.gps");

				counter++;
		   }
	   }
   }

};





int main(int argc, char **argv){
   Ptester ptester;
   if(argc>1)
      ptester.loadCloud(argv[1]);
   else
      ptester.generateClouds();
   ptester.PTest3();
   ptester.saveStats();
//   ptester.makeplots();
   ptester.makeidentifier();
   return 0;
}












