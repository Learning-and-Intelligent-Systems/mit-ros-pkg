/*
 * ScanAnalyzer.cpp
 *
 *  Created on: Oct 22, 2010
 *      Author: garratt
 */

#include "ScanAnalyzer.h"
//a forward transform
//  Eigen3::Matrix4f ScanAnalyzer::ICPStep(int target,sensor_msgs::PointCloud2 &cloud){
//      timeval t0=g_tick();
//     //assemble cloud:
//     pcl::PointCloud<pcl::PointXYZINormal> full_cloud, aligned, fullcloud2;//,original;
////     original=aligned_clouds[target];
//     if(target)
//        full_cloud=aligned_clouds[0];
//     else
//        full_cloud=aligned_clouds[1];
//     for (uint i = 1; i < aligned_clouds.size(); ++i) {
//        if(i!=target)
//           full_cloud+=aligned_clouds[i];
//     }
////     fullcloud2=full_cloud;
////     downSample(fullcloud2,full_cloud,.10);
//     fullcloud2=full_cloud;
////     pcl::toROSMsg(fullcloud2,cloud);
////     pub_cloud.publish(cloud);
//     Eigen3::Matrix4f trans2d,trans;
////     ROS_INFO("assemble cloud for %d took %f seconds.",target,g_tock(t0));
//     pcl::IterativeClosestPoint<pcl::PointXYZINormal,pcl::PointXYZINormal> icp;
//     icp.setInputTarget(full_cloud.makeShared());
//     icp.setMaximumIterations (500);
//     icp.setTransformationEpsilon (1e-4);
//     icp.setMaxCorrespondenceDistance (0.08);
////     ROS_INFO("setup for %d took %f seconds.",target,g_tock(t0));
//     for(uint i=0;i<1;i++){
//        icp.setInputCloud(aligned_clouds[target].makeShared());
////        ROS_INFO("   setup2 for %d took %f seconds.",target,g_tock(t0));
//        icp.align(aligned); // is target cloud, moved
//        trans= icp.getFinalTransformation();
//        trans2d=projectTo2D(trans);
//        pcl::transformPointCloudWithNormals(aligned_clouds[target],aligned,trans2d);
//        aligned_clouds[target]=aligned;
//
//        for(int p=0;p<aligned.points.size();p++)
//           aligned.points[p].intensity=.1;
//        fullcloud2+=aligned;
//        pcl::toROSMsg(fullcloud2,cloud);
//        pub_cloud.publish(cloud);
//
//     }
////     ROS_INFO("icp step for %d took %f seconds.",target,g_tock(t0));
////     for(int p=0;p<original.points.size();p++)
////        original.points[p].intensity=.2;
//     for(int p=0;p<aligned.points.size();p++)
//        aligned.points[p].intensity=.1;
//     full_cloud+=aligned;
////     full_cloud+=original;
//     pcl::toROSMsg(full_cloud,cloud);
////    PublishClouds(full_cloud,aligned_clouds[target]);
//     return trans2d;
//   }

//assume matrix is 2d rot,trans.  scale.
Eigen3::Matrix4f scale(Eigen3::Matrix4f in, double s){
      tf::Transform tftrans=tfFromEigen(in);
      tftrans.setRotation(tf::createQuaternionFromYaw(tf::getYaw(tftrans.getRotation())*s));
      Eigen3::Matrix4f out;
      pcl::transformAsMatrix(tftrans,out);
      out(2,3)=0.0;  //remove z component
      out(0,3)*=s;
      out(1,3)*=s;
      return out;

}

//transforms cloud to point
  Eigen3::Matrix4f ScanAnalyzer::ICPStep(int target,sensor_msgs::PointCloud2 &cloud){
      timeval t0=g_tick();
     //assemble cloud:
     pcl::PointCloud<pcl::PointXYZINormal> full_cloud, aligned, fullcloud2, temp;//,original;
     //the first scan does notget transformed
     if(target==0)
        return Eigen3::Matrix4f();

     full_cloud=aligned_clouds[0];
     for (uint i = 1; i < target; ++i) {
           full_cloud+=aligned_clouds[i];
     }
     fullcloud2=full_cloud;
     Eigen3::Matrix4f trans2d,trans;
//     ROS_INFO("assemble cloud for %d took %f seconds.",target,g_tock(t0));
     pcl::IterativeClosestPoint<pcl::PointXYZINormal,pcl::PointXYZINormal> icp;
     icp.setInputCloud(full_cloud.makeShared());
     icp.setMaximumIterations (500);
     icp.setTransformationEpsilon (1e-6);
     icp.setMaxCorrespondenceDistance (0.15);
//     ROS_INFO("setup for %d took %f seconds.",target,g_tock(t0));
     for(uint i=0;i<1;i++){
        icp.setInputTarget(aligned_clouds[target].makeShared());
//        ROS_INFO("   setup2 for %d took %f seconds.",target,g_tock(t0));
        icp.align(aligned); // is target cloud, moved
        trans= icp.getFinalTransformation();
        trans2d=projectTo2D(trans);
        for (uint c = 0; c < target; ++c) {
           pcl::transformPointCloudWithNormals(aligned_clouds[c],temp,trans2d);
           aligned_clouds[c]=temp;
         }
        for(int p=0;p<aligned.points.size();p++)
           aligned.points[p].intensity=.1;
        fullcloud2+=aligned;
        pcl::toROSMsg(fullcloud2,cloud);
        pub_cloud.publish(cloud);

     }
//     ROS_INFO("icp step for %d took %f seconds.",target,g_tock(t0));
//     for(int p=0;p<original.points.size();p++)
//        original.points[p].intensity=.2;
     for(int p=0;p<aligned.points.size();p++)
        aligned.points[p].intensity=.1;
     full_cloud+=aligned;
//     full_cloud+=original;
     pcl::toROSMsg(full_cloud,cloud);
//    PublishClouds(full_cloud,aligned_clouds[target]);
     return trans2d;
   }

  //transforms cloud to point
    Eigen3::Matrix4f ScanAnalyzer::ICPStep2(int target,sensor_msgs::PointCloud2 &cloud){
        timeval t0=g_tick();
       //assemble cloud:
       pcl::PointCloud<pcl::PointXYZINormal> full_cloud, aligned, fullcloud2, temp;//,original;
       if(target==0)
          full_cloud=aligned_clouds[1];
       else
          full_cloud=aligned_clouds[0];

       for (uint i = 1; i < aligned_clouds.size(); ++i) {
          if(i!=target)
             full_cloud+=aligned_clouds[i];
       }
       fullcloud2=full_cloud;
       Eigen3::Matrix4f trans2d;
       aligned=aligned_clouds[target];
       trans2d=icp2D(aligned_clouds[target],full_cloud,.03,2,500,10,10,.0001);
       aligned=aligned_clouds[target];
       for(int p=0;p<aligned.points.size();p++)
          aligned.points[p].intensity=.1;
       fullcloud2+=aligned;
       pcl::toROSMsg(fullcloud2,cloud);
       pub_cloud.publish(cloud);
       return trans2d;
     }


  void ScanAnalyzer::AccumulateScans2(int iterations){
     timeval t0,t1,t2;
     Eigen3::Matrix4f latest_trans;
     aligned_clouds.resize(filtered_clouds.size());
     t0=g_tick();
     t2=g_tick();

     alignments.resize(filtered_clouds.size(),Eigen3::Matrix4f::Identity());

     pcl::PointCloud<pcl::PointXYZINormal> full_cloud,temp;
     for(uint i=0;i<filtered_clouds.size();i++){
        segmentHeight(filtered_clouds[i],aligned_clouds[i],1.50, 5.0);
//        testclouds[i]=aligned_clouds[i];
     }

     ROS_INFO("copied clouds in %f sec. ",g_tock(t0));
     t0=g_tick();
     sensor_msgs::PointCloud2 cloud;
     for(int j=0;j<iterations;j++){
        double endp=(1.0-(.6-((double)j+1 )*.1));
        t0=g_tick();
        for(int i=filtered_clouds.size()-1;i>=0;i--){
           int targetcloud=(i+1)%filtered_clouds.size();
           cout<<"----------------------- Transforming "<<i<<" --> "<<targetcloud<<"----------------------"<<endl;
           t1=g_tick();
           latest_trans=icp2D(aligned_clouds[i],aligned_clouds[targetcloud],.05,1,500,10,50,.0001);
           alignments[i]= latest_trans * alignments[i];
           //Propagate the transform back:
           if(i>0)
              for(int k=i-1; k>=0; k--){
                 double diff=i-k, num=i;
                 double scalef= (1.0-(endp*diff)/num);
              cout<<"transforming "<<k<<" by "<<scalef<<" times the transform from "<<i<<" to "<<targetcloud<<endl;
              pcl::transformPointCloudWithNormals(aligned_clouds[k], aligned_clouds[k],scale(latest_trans,scalef));
              alignments[k] = scale(latest_trans,scalef) * alignments[k];
           }
           for(int k=0; k<filtered_clouds.size(); k++){
              if(k==i){
                 temp=aligned_clouds[i];
                 for(uint p=0;p<temp.points.size();p++){
                    temp.points[p].intensity=.2;
                 }
                 if(!k) full_cloud=temp;
                 else full_cloud+=temp;
              }
              else
                 if(!k) full_cloud=aligned_clouds[k];
                 else full_cloud+=aligned_clouds[k];
           }
           pcl::toROSMsg(full_cloud,cloud);
           pub_cloud.publish(cloud);
           cout<<endl<<endl;
//           latest_trans=ICPStep2(i,cloud);
//           ROS_INFO("  transform for %d:  x= %f, y= %f, theta= %f   %f seconds",i,latest_trans(0,3),latest_trans(1,3),getYaw(latest_trans),g_tock(t1));
//           for(int j=0;j<3;j++)
//              pub_cloud.publish(cloud);
        }
        ROS_INFO("alignment %d in %f sec. ",j,g_tock(t0));
     }

     ROS_INFO("Step 1 of 2 of alignment took: %f sec. ",g_tock(t2));
     t2=g_tick();
     //now align each scan to all of them

     for(int j=0;j<iterations;j++)
        for(int i=0;i<filtered_clouds.size();i++){
           latest_trans=ICPStep2(i,cloud);
           alignments[i]*=latest_trans;
        }

     ROS_INFO("Step 2 of 2 of alignment took: %f sec. ",g_tock(t2));

     ROS_INFO("Saving Alignments.");
     for(uint s=0;s<scans.size();s++){
        pcl::transformPointCloudWithNormals(filtered_clouds[s], aligned_clouds[s],alignments[s]);
        pcl::toROSMsg (aligned_clouds[s],scans[s].normal_cloud_aligned);

	// apply alignment to narrow stereo clouds
	
     }

     //saves our transforms into the *_aligned messages
     updateAlignedTransforms();

     ROS_INFO("Alignment complete.");

  }


ScanAnalyzer::~ScanAnalyzer() {
	// TODO Auto-generated destructor stub
}
