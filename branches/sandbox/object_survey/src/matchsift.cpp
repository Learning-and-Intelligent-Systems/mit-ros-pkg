/*
 * matchsift.cpp
 *
 *  Created on: Oct 22, 2010
 *      Author: garratt
 */
#include <ros/ros.h>
#include <ros/names.h>
#include <ros/master.h>
#include <ros/this_node.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/fill_image.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/foreach.hpp>

#include "image_proc/processor.h"

#include <algorithm>
#include "siftviewer/ProcessImageCluster.h"
#include "siftviewer/ProcessImage.h"
#include <posedetection_msgs/Feature0DDetect.h>
#include <image_geometry/pinhole_camera_model.h>

//#include "pcl/kdtree/kdtree.h"
//#include <boost/thread/mutex.hpp>
#include <flann/flann.h>

#include <flann/flann.hpp>
#include <flann/io/hdf5.h>

#include <sys/time.h>


//This file contains functions for processing sift features


template <typename type1,typename type2 >
void cp3dPt(type1 &dest, type2 &src ){
	dest.x=src.x;
	dest.y=src.y;
	dest.z=src.z;
}

template <typename type1,typename type2 >
void cp2dPt(type1 &dest, type2 &src ){
	dest.x=src.x;
	dest.y=src.y;
}

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

//get rays
//get rays from sift features and camera info
void getRays(sensor_msgs::CameraInfo &caminfo,posedetection_msgs::Feature0D &feats, std::vector<geometry_msgs::Point> &rays){
	image_geometry::PinholeCameraModel model;
	model.fromCameraInfo(caminfo);
	rays.resize(feats.confidences.size());
	for(uint i=0;i<feats.confidences.size();i++){
		cv::Point2d pt2d;
		cv::Point3d pt3d;
		pt2d.x=feats.positions[2*i];
		pt2d.y=feats.positions[2*i+1];
		model.projectPixelTo3dRay(pt2d,pt3d);
		cp3dPt(rays[i],pt3d);
	}
}


//match sift features between two images



















//---------------   ReIndex Functions:  ----------------------------------------
// these functions allow you to get a subset of the sift features in Feature0D.
void ReIndexFeatures(posedetection_msgs::Feature0D &in, posedetection_msgs::Feature0D &out, std::vector<int> &inds){
	timeval t = g_tick();
	   out.confidences.resize(inds.size());
	   out.descriptor_dim=in.descriptor_dim;
	   out.orientations.resize(inds.size());
	   out.positions.resize(inds.size()*2);
	   out.scales.resize(inds.size());
	   out.type=in.type;
	   out.header = in.header;

	   for(uint i=0;i<inds.size();i++){
		   int m=inds[i];
		   out.confidences[i]=in.confidences[m];
		   out.orientations[i]=in.orientations[m];
		   out.positions[2*i]=in.positions[2*m];
		   out.positions[2*i+1]=in.positions[2*m+1];
		   out.scales[i]=in.scales[m];
		   out.descriptors.insert(out.descriptors.end(),
				   in.descriptors.begin()+(in.descriptor_dim*i),
				   in.descriptors.begin()+(in.descriptor_dim*(i+1))   );
	   }
	   cout<<"restruct took "<<g_tock(t)<<std::endl;
}


template <typename type1>
void appendvec(std::vector<type1> &out, std::vector<type1> &in){
	out.insert(out.end(),in.begin(),in.end());
}

void appendFeatures(posedetection_msgs::Feature0D &out, std::vector< posedetection_msgs::Feature0D> feats){
	for(uint i=0; i< feats.size(); i++){
		   appendvec(out.confidences,feats[i].confidences);
		   appendvec(out.orientations,feats[i].orientations);
		   appendvec(out.positions,feats[i].positions);
		   appendvec(out.descriptors,feats[i].descriptors);
		   appendvec(out.scales,feats[i].scales);
	}
	out.descriptor_dim = feats.front().descriptor_dim;
	out.type=feats.front().type;
}

void ReIndex(posedetection_msgs::Feature0D &in, std::vector<int > &vec,int thresh){
	std::vector<int> newindex;
	std::vector<int> newvec;
	for(uint i=0;i<vec.size();i++)
		if(vec[i]>=thresh){
			newindex.push_back(i);
			newvec.push_back(vec[i]);
		}
	posedetection_msgs::Feature0D out;
	ReIndexFeatures(in,out,newindex);
	in=out;
	vec=newvec;
}




//struct feature{
//	  double x,y,scale,angle,confidence;
//	  std::vector<double> desc;
//	  bool operator==(const feature &f){
//		  for(uint i=0;i<desc.size();i++)
//			  if(fabs(desc[i]-f.desc[i])>.00001)
//				  return false;
//		  return true;
//	  }
//	  bool operator==(std::vector<double> &d){
//		  for(uint i=0;i<desc.size();i++)
//			  if(fabs(desc[i]-d[i])>.00001)
//				  return false;
//		  return true;
//	  }
//	  int count;
//	  feature(posedetection_msgs::Feature0D &f, int i){
//		  x=f.positions[2*i];
//		  y=f.positions[2*i+1];
//		  scale=f.scales[i];
//		  angle=f.orientations[i];
//		  confidence=f.confidences[i];
//		  desc.assign(f.descriptors.begin()+(f.descriptor_dim*i),f.descriptors.begin()+(f.descriptor_dim*(i+1))-1);
//		  count=1;
//	  }
//};


// compare
void compareFeats(posedetection_msgs::Feature0D &source, posedetection_msgs::Feature0D &target, std::vector<int> &matches,bool addingmode=false){
	  timeval t = g_tick();
  int nn = 3;

  float *sdata=new float[source.descriptors.size()];
//    float *tdata=new float[target.descriptors.size()];
  std::copy(source.descriptors.begin(),source.descriptors.end(),sdata);
//    std::copy(target.descriptors.begin(),target.descriptors.end(),tdata);
  flann::Matrix<float> dataset(sdata,source.confidences.size(),source.descriptor_dim);
//    flann::Matrix<float> query  (tdata,target.confidences.size(),target.descriptor_dim);

  // construct an randomized kd-tree index using 4 kd-trees
  flann::Index<float> index(dataset, flann::KDTreeIndexParams(4));
  index.buildIndex();
  std::cout<<" init time: "<<g_tock(t);


  float *tdata=new float[target.descriptors.size()];
  std::copy(target.descriptors.begin(),target.descriptors.end(),tdata);
  flann::Matrix<float> query  (tdata,target.confidences.size(),target.descriptor_dim);
  flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
  flann::Matrix<float> dists(new float[query.rows*nn], query.rows, nn);
  if(matches.size()!=query.rows)
	  matches.resize(query.rows,-1);
  // do a knn search, using 128 checks
  index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
  int matchescount=0;
  float* fpt;
  for(uint i=0;i<query.rows;i++){
  	fpt = dists[i];
  	if(fpt[0]/fpt[1] > .8){
  		if(addingmode)
  			matches[i]++;  //= *((int*)(indices[i]));
  		else
			matches[i]= *((int*)(indices[i]));
 		matchescount++;
  	}
  }
  std::cout<<"  found "<<matchescount<<" matches out of "<<query.rows<<" in "<<g_tock(t)<<std::endl;
  dataset.free();
  query.free();
  indices.free();
  dists.free();
}



//compare a set of sift points a vector of sets.
//source: the index in target vector of the feature set that will be compared to the rest.  if negative, grabs the last one
void compareFeats(int source, std::vector<posedetection_msgs::Feature0D> &target, std::vector<std::vector<int> > &matches,bool addingmode=false){

  if(source<0)
	  source=target.size()-1;
  timeval t = g_tick();
  int nn = 3;

  float *sdata=new float[target[source].descriptors.size()];
  std::copy(target[source].descriptors.begin(),target[source].descriptors.end(),sdata);
  flann::Matrix<float> dataset(sdata,target[source].confidences.size(),target[source].descriptor_dim);

  // construct an randomized kd-tree index using 4 kd-trees
  flann::Index<float> index(dataset, flann::KDTreeIndexParams(4));
  index.buildIndex();
  std::cout<<" init time: "<<g_tock(t);

  for(uint f=0;f<target.size();f++){
  	if(f==source) continue;
		float *tdata=new float[target[f].descriptors.size()];
		std::copy(target[f].descriptors.begin(),target[f].descriptors.end(),tdata);
		flann::Matrix<float> query  (tdata,target[f].confidences.size(),target[f].descriptor_dim);
		flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
		flann::Matrix<float> dists(new float[query.rows*nn], query.rows, nn);

		// do a knn search, using 128 checks
		index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
		if(matches[f].size() != query.rows)  //in case we had not allocated space for matches
			matches[f].resize(query.rows,-1);
		int matchescount=0;
		float* fpt;
		for(uint i=0;i<query.rows;i++){
			fpt = dists[i];
			if(fpt[1]/fpt[0] < .8){
				matches[f][i] = *((int*)(indices[i]));
				matches[source][*((int*)(indices[i]))]++; //add a count to the match vector that corresponds with the source
				matchescount++;
			}
		}
		std::cout<<"  found "<<matchescount<<" matches out of "<<query.rows<<" in "<<g_tock(t)<<std::endl;
		query.free();
		indices.free();
		dists.free();
  }

//    std::cout<<"  found "<<matchescount<<" matches out of "<<query.rows<<" in "<<g_tock(t)<<std::endl;
  dataset.free();
}
void filterFeatures(std::vector<posedetection_msgs::Feature0D> &array, posedetection_msgs::Feature0D &combined){
	  std::vector<feature> feats,tfeats;
	  for(uint i=0;i<array[0].confidences.size();i++){
		  feats.push_back(feature(array[0],i));
	  }
	  //then for the rest of the images:
	  for(uint a=1;a<array.size();a++){
		  tfeats.clear();
		  for(uint i=0;i<array[a].confidences.size();i++){
			  tfeats.push_back(feature(array[a],i));
			  for(uint f=0;f<feats.size();f++){
				  if(feats[f]==tfeats.back()){
					  tfeats.pop_back();
					  feats[f].count++;
					  break;
				  }
			  }
		  }
		  ROS_INFO("Image %d had %d overlapping features out of %d",a,array[a].confidences.size()-tfeats.size(),array[a].confidences.size());
		  feats.insert(feats.end(),tfeats.begin(),tfeats.end());
	  }
	  std::vector<int> hist(array.size());
	  for(uint i=0;i<feats.size();i++){
		  if(feats[i].count>array.size()) ROS_WARN("Feature %d found %d times",i,feats[i].count);
		  else
			  hist[feats[i].count]++;
	  }
	  for(uint i=0;i<hist.size();i++)
		  ROS_INFO("Found %d features with %d occurences",hist[i],i);


}

//
//  int min_misses = rect_images.size()/4; //minimum number of non matches we will allow
//  for(uint i=0;i<rect_images.size();i++)
//  	   matches[i].resize(features[i].confidences.size(),0);
//
//  for(uint i=0;i<rect_images.size();i++){
//		   compareFeats(i,features,matches,true);  //compare with other images
////		   if(j>min_misses+1)
////			   ReIndex(features[i],matches[i],j-min_misses); //remove all the features who have too many non-matches
////	   }
//	  for(int m=rect_images.size()-1;  m > rect_images.size()/2; m--)
//		  cout<< std::count(matches[i].begin(),matches[i].end(),m)<<" features have "<<m<<"  matches"<<std::endl;
//  }
//  std::vector<int> goodmatches;  //good matches are matched all the way through
//  for(uint i=0;i<matches[0].size();i++){
//	   if(matches[0][i] > rect_images.size()*3/4)
//		   goodmatches.push_back(i);
//  }
//
////   for(uint f=0;f<matches[0].size();f++){
////	   int currentindex=f;
////	   for(uint i=0;i<rect_images.size()-1;i++){
////		   currentindex=matches[i][currentindex];
////		   if(currentindex == -1) break;
////		   //if evaluating more than all-matchers:
//////		   matches[i][currentindex]=-1;
//////		   matches[0][f]=i; //shows how far we got
////	   }
////	   if(currentindex!=-1)
////		   goodmatches.push_back(f);
////   }
//  std::cout<<goodmatches.size()<<" good matches!"<<std::endl;
//  //now reconstruct features vector:
//  res.features.confidences.resize(goodmatches.size());
//  res.features.descriptor_dim=features[0].descriptor_dim;
//  res.features.orientations.resize(goodmatches.size());
//  res.features.positions.resize(goodmatches.size()*2);
//  res.features.scales.resize(goodmatches.size());
//  res.features.type=features[0].type;
//  res.features.header = req.raw_images[0].header;
//
//  for(uint i=0;i<goodmatches.size();i++){
//	   int m=goodmatches[i];
//	   res.features.confidences[i]=features[0].confidences[m];
//	   res.features.orientations[i]=features[0].orientations[m];
//	   res.features.positions[2*i]=features[0].positions[2*m];
//	   res.features.positions[2*i+1]=features[0].positions[2*m+1];
//	   res.features.scales[i]=features[0].scales[m];
//	   res.features.descriptors.insert(res.features.descriptors.end(),
//			   features[0].descriptors.begin()+(features[0].descriptor_dim*i),
//			   features[0].descriptors.begin()+(features[0].descriptor_dim*(i+1))   );
//  }








