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


#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <mapping_msgs/PolygonalMap.h>
#include <kinect_tools/Hands.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_tools/pcl_utils.h>
#include <nnn/nnn.hpp>
#include <pcl_tools/segfast.hpp>
#include <pcl_ros/transforms.h>

//for sound
#include <SFML/Audio.hpp>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <pcl/ModelCoefficients.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/features/normal_3d.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>


float gdist(pcl::PointXYZ pt, Eigen3::Vector4f v){
   return sqrt((pt.x-v(0))*(pt.x-v(0))+(pt.y-v(1))*(pt.y-v(1))+(pt.z-v(2))*(pt.z-v(2))); //
}

void flipvec(Eigen3::Vector4f palm, Eigen3::Vector4f fcentroid,Eigen3::Vector4f &dir ){
   if((fcentroid-palm).dot(dir) <0)
      dir=dir*-1.0;

 }

geometry_msgs::Point32 eigenToMsgPoint32(Eigen3::Vector4f v){
	geometry_msgs::Point32 p;
	p.x=v(0); p.y=v(1); p.z=v(2);
	return p;
}
geometry_msgs::Point eigenToMsgPoint(Eigen3::Vector4f v){
	geometry_msgs::Point p;
	p.x=v(0); p.y=v(1); p.z=v(2);
	return p;
}

pcl::PointXYZRGB eigenToPclPoint(Eigen3::Vector4f v){
   pcl::PointXYZRGB p;
   p.x=v(0); p.y=v(1); p.z=v(2);
   return p;
}

float randColor(){
   int r = rand()%255;
   int g = rand()%255;
   int b = rand()%255;
   int color = b + (g << 8) + (r << 16);
   float _rgb = *reinterpret_cast<const float*>(&(color));
   return _rgb;
}



void colorCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloudin, float rgb){
   for(uint i=0;i<cloudin.points.size(); i++)
      cloudin.points[i].rgb=rgb;
}


template <typename PointT>
void getSubCloud(pcl::PointCloud<PointT> &cloudin,  std::vector<int> &ind, pcl::PointCloud<PointT> &cloudout,bool use_positive=true){
   pcl::ExtractIndices<PointT> extract;
   // Extract the inliers
   extract.setInputCloud (boost::make_shared<pcl::PointCloud<PointT> > (cloudin));
   extract.setIndices (boost::make_shared<std::vector<int> > (ind));
   extract.setNegative (!use_positive);
   extract.filter (cloudout);
//    ROS_INFO ("Subcloud representing the planar component: %d data points.", cloudout.width * cloudout.height);

}

//find the points that are near a different cloud:
//cloud: the original cloud
//cloudnewcloud a vector of indices into cloud that represents the cluster for which we want to find near points
void findNearbyPts(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, pcl::PointCloud<pcl::PointXYZRGB> &newcloud,pcl::PointCloud<pcl::PointXYZRGB> &cloudout){
   std::vector<int> inds(cloud.size(),0); //a way of marking the points we have looked at
   std::vector<int> newinds(newcloud.size(),0);
   // 1: not in the cluster  0: in the cluster, seen  -1: in the cluster, not seen
   std::vector<int> nearpts; //a way of marking the points we have looked at
   std::vector<int> temp;

   pcl::PointXYZRGB pt;
   SplitCloud2<pcl::PointXYZRGB> sc2(newcloud,.05);
   for(uint i=0;i<inds.size(); ++i){
      if(!inds[i]){
         NNN(cloud,cloud.points[i],temp, .04);
         for(uint j=0;j<temp.size(); ++j)
            inds[temp[j]]=1;   //mark the inds in the old cloud to indicate we've searched near here
         pt.x=cloud.points[i].x;pt.y=cloud.points[i].y;pt.z=cloud.points[i].z;

         sc2.AssignInds_filterz(pt, newinds, .05, 1,.008); //mark all the points in the new cloud near this point in the old cloud
      }
   }
   for(uint i=0;i<newinds.size(); ++i){
      if(newinds[i])
         nearpts.push_back(i);
   }

   getSubCloud(newcloud,nearpts,cloudout,true);
}



void filterEdges(pcl::PointCloud<pcl::PointXYZRGB> &cloudin,pcl::PointCloud<pcl::PointXYZRGB> &cloudout){
   //first get the centroid of the cloud
   Eigen3::Vector4f centroid;
   pcl::compute3DCentroid(cloudin,centroid);
   std::vector<int> temp,nearpts;
   pcl::PointXYZRGB pt=eigenToPclPoint(centroid);

   //establish a baseline of how many points to expect in a NNN search:
   NNN(cloudin,pt,temp, .05);
   uint maxpts=temp.size();
   std::vector<int> inds(cloudin.size(),0);
   for(uint i=0;i<inds.size(); ++i){
      if(inds[i]) continue;
      NNN(cloudin,cloudin.points[i],temp, .05);
      if(temp.size()>.5*maxpts){
         NNN(cloudin,cloudin.points[i],temp, .04);
         for(uint j=0;j<temp.size(); ++j){
            inds[temp[j]]=1;
         }
      }
      else{
         for(uint j=0;j<temp.size(); ++j)
            inds[temp[j]]=-1;
      }
   }

   for(uint i=0;i<inds.size(); ++i){
      if(inds[i]==1)
         nearpts.push_back(i);
   }
   getSubCloud(cloudin,nearpts,cloudout,true);
}


//find the points that are not near a different cloud:
//cloud: the original cloud
//cloudnewcloud
void subtractLines(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<pcl::PointXYZRGB> &newcloud,pcl::PointCloud<pcl::PointXYZRGB> &cloudout){
   std::vector<int> newinds(newcloud.size(),1);
   std::vector<int> nearpts; //a way of marking the points we have looked at

   SplitCloud2<pcl::PointXYZRGB> sc2(newcloud,.005);
   for(uint i=0;i<cloud.points.size(); ++i){
         sc2.AssignInds(cloud.points[i], newinds, .005, 0); //mark all the points in the new cloud near this point in the old cloud
   }
   for(uint i=0;i<newinds.size(); ++i){
      if(newinds[i])
         nearpts.push_back(i);
   }
   getSubCloud(newcloud,nearpts,cloudout,true);
}


class Button{
   std::deque<pcl::PointCloud<pcl::PointXYZRGB> > points;
   Eigen3::Vector4f centeroid;
   std::deque<bool> sightings; //keeps track of how often we have seen this button
   float color;
   int num;
public:
   Button(pcl::PointCloud<pcl::PointXYZRGB> &newpts, Eigen3::Vector4f &newcent,int n){
      points.push_back(newpts);
      centeroid=newcent;
      sightings.push_back(true);
      color=randColor();
      num=n;
   }
   bool isMatched(){ return sightings.back();}
   float check(Eigen3::Vector4f pt){
      return (centeroid-pt).norm();
   }
   bool isGood(){
      int count=0;
      for(uint i=0;i<sightings.size();i++)
         if(sightings[i]) count++;
//      std::cout<<"cloud "<<num<<" has "<<count<<std::endl;
      return count > 3;
   }
   bool isDead(){
      int count=0;
      for(uint i=0;i<sightings.size();i++)
         if(sightings[i]) count++;
      return count > 0;
   }


   void update(pcl::PointCloud<pcl::PointXYZRGB> &newpts, Eigen3::Vector4f newcent){
      sightings.back()=true;
//      int count=0;
//      for(uint i=0;i<sightings.size();i++)
//         if(sightings[i]) count++;
//      std::cout<<"cloud "<<num<<" updated and has "<<count<<" out of "<<sightings.size();
//      for(uint i=0;i<sightings.size();i++)
//         if(sightings[i]) std::cout<<" 1";
//         else std::cout<<" 0";
//      std::cout<<std::endl;
      points.push_back(newpts);
      colorCloud(points.back(),color);
      centeroid=newcent;
   }
   //get ready for new batch of button matches
   void reset(){
      sightings.push_back(false);
      if(sightings.size()>5)
         sightings.pop_front();
      if(points.size()>5)
         points.pop_front();
   }

   pcl::PointCloud<pcl::PointXYZRGB> & getCloud(){
      return points.back();
   }
   pcl::PointXYZRGB getCenter(){
      return eigenToPclPoint(centeroid);
   }
   int getNum(){return num;}


};

class ButtonPanel{
   std::vector< Button > goodbuttons;
//   std::vector< Button > potentialbuttons;
   int buttonnum;

   void makeNewButton(pcl::PointCloud<pcl::PointXYZRGB> &pts, Eigen3::Vector4f &centroid){
      goodbuttons.push_back(Button(pts,centroid,buttonnum++));
   }

   //promote and demote buttons
   void cleanup(){
     uint i=0;
     while(i<goodbuttons.size()){
        if(!goodbuttons[i].isDead())
           goodbuttons.erase(goodbuttons.begin()+i);
        else
           i++;
     }
   }


   Button *getClosest(Eigen3::Vector4f pt){
      Button *bp=0;
      float closest=0.01,tdist;
      for(uint i=0;i<goodbuttons.size();i++){
         tdist=goodbuttons[i].check(pt);
         if(tdist<closest){
            closest=tdist;
            bp=&goodbuttons[i];
         }
      }
//      for(uint i=0;i<potentialbuttons.size();i++){
//         tdist=potentialbuttons[i].check(pt);
//         if(tdist<closest){
//            closest=tdist;
//            bp=&potentialbuttons[i];
//         }
//      }
      return bp;
   }
public:
   ButtonPanel(){ buttonnum=1;}
   void MatchButtons(pcl::PointCloud<pcl::PointXYZRGB> &buttoncloud){
      for(uint i=0;i<goodbuttons.size();i++) goodbuttons[i].reset();
//      for(uint i=0;i<potentialbuttons.size();i++) potentialbuttons[i].reset();
      std::vector<std::vector<int> > clusters;
      std::vector<int> buttonpts;
      Eigen3::Vector4f centroid;
      Button *bp;
      int matches=0;
      pcl::PointCloud<pcl::PointXYZRGB> temp;
      extractEuclideanClustersFast2(buttoncloud, clusters, .004, 30);
      for(uint i=0;i<clusters.size(); i++){
//         std::cout<<"cluster "<<i<<"  has "<<clusters[i].size()<<std::endl;
         if(clusters[i].size() < 1000){ //we got a potential button here!
            buttonpts.insert(buttonpts.end(),clusters[i].begin(),clusters[i].end());
            pcl::compute3DCentroid(buttoncloud,clusters[i],centroid);
            bp=getClosest(centroid);
            getSubCloud(buttoncloud,clusters[i],temp,true);
            if(bp){
               bp->update(temp,centroid);
               matches++;
            }
            else
               makeNewButton(temp,centroid);
         }
      }
      std::cout<<matches<<" matches"<<std::endl;
      cleanup();
   }
   bool getCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloudout){
      bool firstcloud=true;
      for(uint i=0;i<goodbuttons.size();i++){
         if(goodbuttons[i].isGood()){
            if(firstcloud){
               cloudout= goodbuttons[i].getCloud();
               firstcloud=false;
            }
            else
               cloudout+=goodbuttons[i].getCloud();
         }
      }
      return !firstcloud;
   }

   //returns 0 if not pressed
   int isHandAbove(pcl::PointCloud<pcl::PointXYZRGB> &cloudin, float &lowest){
      pcl::PointXYZRGB pt;
      std::vector<int> inds;
      int pressed=0;
      for(uint i=0;i<goodbuttons.size();i++){
         if(goodbuttons[i].isGood()){
            pt=goodbuttons[i].getCenter();
            pt.z+=.06;
            NNN(cloudin,pt,inds,.05);
            if(inds.size()){
               std::cout<<goodbuttons[i].getNum()<<" ";
               if(!pressed){
                  lowest=cloudin.points[inds[0]].z - (pt.z-.06);
                  pressed=goodbuttons[i].getNum();
               }
               for(uint j=0;j<inds.size();j++){
                  if(cloudin.points[inds[j]].z - (pt.z-.06) < lowest){
                     lowest=cloudin.points[inds[j]].z - (pt.z-.06);
                     pressed=goodbuttons[i].getNum();
                  }
               }
            }
         }
      }
      if(pressed)
         std::cout<<"-------------  Hand detected ------------"<<pressed<<"  dist: "<<lowest<<"  "<<pressed%22<<std::endl;
      return pressed;
   }



};


void getButtons(pcl::PointCloud<pcl::PointXYZRGB> &cloudin, pcl::PointCloud<pcl::PointXYZRGB> &cloudout){
   std::vector<std::vector<int> > clusters;
   std::vector<int> buttonpts;
   extractEuclideanClustersFast2(cloudin, clusters, .004, 30);
//   int biggest=0; uint biggestval=0;
   for(uint i=0;i<clusters.size(); i++){
      std::cout<<"cluster "<<i<<"  has "<<clusters[i].size()<<std::endl;
      if(clusters[i].size() < 1000)
         buttonpts.insert(buttonpts.end(),clusters[i].begin(),clusters[i].end());
//
//      if(clusters[i].size()>biggestval){
//         biggest=i;
//         biggestval=clusters[i].size();
//      }
   }

//   std::cout<<"chose cluster "<<biggest<<"  with "<<clusters[biggest].size()<<std::endl;
   getSubCloud(cloudin,buttonpts,cloudout,true);

}




void myFlipNormals(double vx, double vy, double vz, pcl::PointCloud<pcl::PointXYZRGBNormal> &ncloud){
   for(uint i=0;i<ncloud.points.size();i++){
      pcl::PointXYZRGBNormal p=ncloud.points[i];
      double dotprod=(p.normal[0]*(vx-p.x)+p.normal[1]*(vy-p.y)+p.normal[2]*(vz-p.z));
      if(dotprod<0){
//       cout<<"flipping"<<std::endl;
         ncloud.points[i].normal[0]*=-1.0;
         ncloud.points[i].normal[1]*=-1.0;
         ncloud.points[i].normal[2]*=-1.0;
      }
   }
}
void getNormals(pcl::PointCloud<pcl::PointXYZRGB> &cloudin, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud_normals,pcl::PointXYZ viewpoint){
   timeval t0=g_tick();
   pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_ = boost::make_shared<const pcl::PointCloud<pcl::PointXYZRGB> > (cloudin);
   pcl::KdTree<pcl::PointXYZRGB>::Ptr normals_tree_;
    int k_ = 50;                                // 50 k-neighbors by default
    normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ();
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> n3d_;
    n3d_.setKSearch (k_);
    n3d_.setSearchMethod (normals_tree_);
    n3d_.setInputCloud (cloud_);
    n3d_.compute (cloud_normals);
    for(uint i=0;i<cloudin.points.size(); i++){
      cloud_normals.points[i].x=cloudin.points[i].x;
      cloud_normals.points[i].y=cloudin.points[i].y;
      cloud_normals.points[i].z=cloudin.points[i].z;
      cloud_normals.points[i].rgb=cloudin.points[i].rgb;
    }

    myFlipNormals(viewpoint.x,viewpoint.y,viewpoint.z,cloud_normals);
   std::cout<<"finding normals took:  "<<g_tock(t0)<<"  for "<<cloud_normals.points.size()<<" indices."<<std::endl;
}


template <typename PointT>
void segmentNorms(pcl::PointCloud<PointT> &cloudin, pcl::PointCloud<PointT> &cloudout,double lowest,double highest, bool use_positive=true){
   timeval t0=g_tick();
   std::vector<int> ind;
   for(uint i=0;i<cloudin.points.size(); i++){
      if(cloudin.points[i].normal[2] > lowest && cloudin.points[i].normal[2] < highest){ //|| cloudin.points[i].z <.1){
         ind.push_back(i);
      }
   }
   getSubCloud(cloudin,ind,cloudout,use_positive);
   std::cout<<"segmentNorms took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}


//struct colorf{
//   colorf(float _rgb){
//      int color = *reinterpret_cast<const int*>(&(pt.rgb));
//      const int r = (0xff0000 & color) >> 16;
//      const int g = (0x00ff00 & color) >> 8;
//      const int b = 0x0000ff & color;
//
//   }
//
//
//};

//void getcolor(pcl::PointXYZRGB &pt){
//   int color = *reinterpret_cast<const int*>(&(pt.rgb));
//   const int r = (0xff0000 & color) >> 16;
//   const int g = (0x00ff00 & color) >> 8;
//   const int b = 0x0000ff & color;
//}
bool isColor2(pcl::PointXYZRGB &pt){
   int color = *reinterpret_cast<const int*>(&(pt.rgb));
   const int r = (0xff0000 & color) >> 16;
   const int g = (0x00ff00 & color) >> 8;
   const int b = 0x0000ff & color;
   return r+g+b < 300;
}
bool isColor(pcl::PointXYZRGBNormal &pt){
   int color = *reinterpret_cast<const int*>(&(pt.rgb));
   const int r = (0xff0000 & color) >> 16;
   const int g = (0x00ff00 & color) >> 8;
   const int b = 0x0000ff & color;
   return r>240 && g >240 && b >240;
}
//
void segmentColor(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloudin, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloudout){
   std::vector<int> ind;
      for(uint i=0;i<cloudin.points.size(); i++){

         if(isColor(cloudin.points[i])){ //|| cloudin.points[i].z <.1){
            ind.push_back(i);
         }
      }
      getSubCloud(cloudin,ind,cloudout,true);
}
//
void segmentColor2(pcl::PointCloud<pcl::PointXYZRGB> &cloudin, pcl::PointCloud<pcl::PointXYZRGB> &cloudout){
   std::vector<int> ind;
      for(uint i=0;i<cloudin.points.size(); i++){

         if(isColor2(cloudin.points[i])){ //|| cloudin.points[i].z <.1){
            ind.push_back(i);
         }
      }
      getSubCloud(cloudin,ind,cloudout,true);
}
void getBiggestCluster(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloudin, pcl::PointCloud<pcl::PointXYZRGBNormal> &cloudout){
   std::vector<std::vector<int> > clusters;
   extractEuclideanClustersFast2(cloudin, clusters, .02, 30);
   int biggest=0; uint biggestval=0;
   for(uint i=0;i<clusters.size(); i++){
      std::cout<<"cluster "<<i<<"  has "<<clusters[i].size()<<std::endl;
      if(clusters[i].size()>biggestval){
         biggest=i;
         biggestval=clusters[i].size();
      }
   }

   std::cout<<"chose cluster "<<biggest<<"  with "<<clusters[biggest].size()<<std::endl;
   getSubCloud(cloudin,clusters[biggest],cloudout,true);

}
template <typename PointT>
void segmentHeight(pcl::PointCloud<PointT> &cloudin, pcl::PointCloud<PointT> &cloudout,double lowest,double highest, bool use_positive=true){
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

class BoxDetector
{

private:
   tf::TransformListener tl_;
  ros::NodeHandle n_;
  ros::Publisher handspub_,tablepub_;
  ros::Subscriber sub_;
  bool table_found;
  pcl::PointCloud<pcl::PointXYZRGBNormal> tcloud;
  ButtonPanel panel;
//to play sounds:
  std::vector<sf::SoundBuffer> Buffer;
  std::deque<sf::Sound> notes;
  std::vector<ros::Time> lasthit;
 int numsounds;


public:

  BoxDetector():tl_(ros::Duration(120.0))
  {
   handspub_ = n_.advertise<kinect_tools::Hands> ("hands", 1);
   tablepub_ = n_.advertise<sensor_msgs::PointCloud2> ("table_cloud", 1);
   sub_=n_.subscribe("/camera/depth/points2", 1, &BoxDetector::cloudcb, this);
   table_found=false;
   lasthit.resize(5,ros::Time::now());

   //to play sounds:
   numsounds=22;
//   const char *filenames[] = {"bell.wav", "hit2.wav" , "snare3.wav" ,"trumpet.wav", "cymbal.wav" , "metalplate.wav" , "squeaky.wav"};
   const char *filenames[] = {"bell.wav",     "Cym2.wav",    "Fish.wav",      "md_block.wav",
                              "Oboe-E5.wav",  "piano2.wav",  "squeaky.wav",  "trumpet.wav",
                              "bongo01.wav",  "cymbal.wav",  "hit2.wav",      "metalplate.wav",
                              "Percussion(4e).wav",  "snare3.wav",  "Trompeteshort.wav",
                              "bongos.wav",   "Finger.wav",  "marimba1.wav",  "Oboe-C4.wav",
                              "piano1.wav",          "splat.wav",   "trumpet-c4.wav"};
//   const char *filenames[] = {"C.wav", "D.wav", "E.wav", "F.wav", "G.wav"};
   char tempname[500];
   Buffer.resize(numsounds);
   for(int i=0;i<numsounds;i++){

      std::string notes_path = ros::package::getPath("kinect_tools") + "/sounds";
     sprintf(tempname,"%s/%s",notes_path.c_str(),filenames[i]);
     if (!Buffer[i].LoadFromFile(tempname)){
        ROS_ERROR("could not load file %s",filenames[i]);
        exit(-1);
     }
   }
  }
  void hitkey(int k){
     if((ros::Time::now()-lasthit[k]) > ros::Duration(.5)){
        lasthit[k]=ros::Time::now();
        notes.push_back(sf::Sound(Buffer[k]));
        notes.back().Play();
     }
//     fingerdown[k]=true;
  }

//  void unhitkey(int k){
//     fingerdown[k]=false;
//  }

  void cleanupSounds(){
     while(notes.size() && notes.front().GetStatus() == sf::Sound::Stopped)
        notes.pop_front();
  }

  pcl::PointXYZ transformViewpoint(string frame,roslib::Header header){
     geometry_msgs::PointStamped ptin,ptout;
     ptin.header=header;
     ptin.point.x=0; ptin.point.y=0; ptin.point.z=0;
     tl_.transformPoint("/world",ptin,ptout);
     pcl::PointXYZ ret;
     ret.x=ptout.point.x;
     ret.y=ptout.point.y;
     ret.z=ptout.point.z;
     return ret;
  }

  void findTable(const sensor_msgs::PointCloud2ConstPtr &scan){
     sensor_msgs::PointCloud2 cloud2;
     pcl::PointCloud<pcl::PointXYZRGB> cloud;
     pcl::PointCloud<pcl::PointXYZRGBNormal> ncloud,ncloud_seg,ncloud_cseg;
     if(!tl_.waitForTransform("/world",scan->header.frame_id,scan->header.stamp,ros::Duration(.5)))
        return;
     pcl_ros::transformPointCloud ("/world", *scan,cloud2, tl_);
     pcl::PointXYZ viewpoint=transformViewpoint("/world",scan->header);
     pcl::fromROSMsg(cloud2,cloud);
     getNormals(cloud,ncloud,viewpoint);
     segmentNorms(ncloud,ncloud_seg,.95,2.0,true);
     segmentColor(ncloud_seg,ncloud_cseg);
     getBiggestCluster(ncloud_seg,tcloud);
     table_found=true;

  }


  void cloudcb(const sensor_msgs::PointCloud2ConstPtr &scan){
     if(!table_found)
        findTable(scan);
     sensor_msgs::PointCloud2 cloud2,cloud2out;
     pcl::PointCloud<pcl::PointXYZRGB> cloud,tablecloud,darkcloud,tablecloudne, bcloud1, bcloud2;
     pcl::PointCloud<pcl::PointXYZRGBNormal> ncloud,ncloud_seg,ncloud_cseg,ncluster;
     if(!tl_.waitForTransform("/world",scan->header.frame_id,scan->header.stamp,ros::Duration(.5)))
        return;
     pcl_ros::transformPointCloud ("/world", *scan,cloud2, tl_);
     pcl::PointXYZ viewpoint=transformViewpoint("/world",scan->header);
     pcl::fromROSMsg(cloud2,cloud);
     float lowest;
     int pressed=panel.isHandAbove(cloud,lowest);
     if(!pressed){
        findNearbyPts(tcloud,cloud,tablecloud);
        filterEdges(tablecloud,tablecloudne);

   //     getNormals(tablecloud,ncloud,viewpoint);
   //     segmentNorms(ncloud,ncloud_seg,.95,2.0,true);
   //     segmentColor(ncloud_seg,ncloud_cseg);
   //     getBiggestCluster(ncloud_seg,ncluster);
        segmentColor2(tablecloudne,darkcloud);
        subtractLines(darkcloud,tablecloudne,bcloud1);
        panel.MatchButtons(bcloud1);
        if(panel.getCloud(bcloud2))
           pcl::toROSMsg(bcloud2,cloud2out);
        else
           pcl::toROSMsg(bcloud1,cloud2out);
        cloud2out.header=cloud2.header;
        tablepub_.publish(cloud2out);

     }
     else{
        if(lowest < .013 ){
           hitkey(pressed%numsounds);
        }
        if(panel.getCloud(bcloud2)){
           pcl::toROSMsg(bcloud2,cloud2out);
           cloud2out.header=cloud2.header;
           tablepub_.publish(cloud2out);
        }
     }

//     getButtons(bcloud1,bcloud2);
//     colorCloud(bcloud2);
//     pcl::toROSMsg(bcloud2,cloud2out);
//     cloud2out.header=cloud2.header;
//     tablepub_.publish(cloud2out);
//     pcl::ConvexHull2D<pcl::PointXYZ, pcl::PointXYZ> chull;

  }

} ;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "hand_detector");
  ros::NodeHandle n;
  BoxDetector detector;
  ros::spin();
  return 0;
}
