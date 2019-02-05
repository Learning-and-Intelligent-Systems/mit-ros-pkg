/*
 * pcl_helper.cpp
 *
 *  Created on: Oct 6, 2010
 *      Author: garratt
 */

#include "pcl_helpers.h"
using namespace std;


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


  void getSubCloud(pcl::PointCloud<Point> &cloudin, std::vector<int> &ind, pcl::PointCloud<Point> &cloudout,bool setNegative){
  	pcl::ExtractIndices<Point> extract;
  	// Extract the inliers
  	extract.setInputCloud (boost::make_shared<pcl::PointCloud<Point> > (cloudin));
  	extract.setIndices (boost::make_shared<std::vector<int> > (ind));
  	extract.setNegative (setNegative);
  	extract.filter (cloudout);
//  	ROS_INFO ("Subcloud representing the planar component: %d data points.", cloudout.width * cloudout.height);

  }

  void getSubCloud(pcl::PointCloud<Point> &cloudin, pcl::PointIndices &ind, pcl::PointCloud<Point> &cloudout,bool setNegative){
  	pcl::ExtractIndices<Point> extract;
  	// Extract the inliers
  	extract.setInputCloud (boost::make_shared<pcl::PointCloud<Point> > (cloudin));
  	extract.setIndices (boost::make_shared<pcl::PointIndices > (ind));
  	extract.setNegative (setNegative);
  	extract.filter (cloudout);
//  	ROS_INFO ("Subcloud representing the planar component: %d data points.", cloudout.width * cloudout.height);

  }

  void getSubCloud(pcl::PointCloud<pcl::PointXYZINormal> &cloudin,  std::vector<int> &ind, pcl::PointCloud<pcl::PointXYZINormal> &cloudout,bool setNegative){
  	pcl::ExtractIndices<pcl::PointXYZINormal> extract;
  	// Extract the inliers
  	extract.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal> > (cloudin));
  	extract.setIndices (boost::make_shared<std::vector<int> > (ind));
  	extract.setNegative (setNegative);
  	extract.filter (cloudout);
//  	ROS_INFO ("Subcloud representing the planar component: %d data points.", cloudout.width * cloudout.height);

  }
void segment(pcl::PointCloud<Point> &cloud, std::vector<pcl::PointCloud<Point> > &clusters){
	timeval t0=g_tick();
	  std::vector<pcl::PointIndices> inds;
	  // Create the segmentation object
	  pcl::EuclideanClusterExtraction<Point> clusterer;
	  clusterer.setClusterTolerance(.2);
	  clusterer.setMinClusterSize(1000);
	  clusterer.setMaxClusterSize(10000);
	  clusterer.setInputCloud(boost::make_shared<pcl::PointCloud<Point> > (cloud));
	  clusterer.extract(inds);

	  clusters.resize(clusters.size());
	  for(uint i=0;i<clusters.size(); i++){
		  getSubCloud(cloud,inds[i],clusters[i]);
	  }
	  std::cout<<"segmentation took:  "<<g_tock(t0)<<"  for "<<clusters.size()<<" clusters."<<std::endl;
}

void readPCD(std::string filename, sensor_msgs::PointCloud2 &cloudout){
	  // Fill in the cloud data
	  pcl::PCDReader reader;
	  reader.read (filename, cloudout);
}

void readPCD(std::string filename, pcl::PointCloud<Point> &cloudout){
	  // Fill in the cloud data
	  pcl::PCDReader reader;
	  sensor_msgs::PointCloud2 cloud;
	  reader.read (filename, cloud);
	  pcl::fromROSMsg (cloud,cloudout);
}

string detectCloudType(sensor_msgs::PointCloud2 &cloud){
	for(uint i=0;i<cloud.fields.size();i++){
		if(cloud.fields[i].name.compare("vp_x")==0)
			return "PointWithViewpoint";
		if(cloud.fields[i].name.compare("normal_x")==0)
			return "PointXYZINormal";
	}
	return "other";
}


int readPCD(std::string filename, pcl::PointCloud<pcl::PointXYZINormal> &cloudout){
	  // Fill in the cloud data
	  pcl::PCDReader reader;
	  sensor_msgs::PointCloud2 cloud;
	  reader.read (filename, cloud);
	  string type=detectCloudType(cloud);
	  if(type.compare("PointXYZINormal")==0){
		  pcl::fromROSMsg (cloud,cloudout);
		  return 0;
	  }
	  if(type.compare("PointWithViewpoint")==0){
		  pcl::PointCloud<pcl::PointWithViewpoint> tcloud;
		  pcl::fromROSMsg (cloud,tcloud);
		  getNormals(tcloud,cloudout);
		  myFlipNormals(tcloud.points[0].vp_x, tcloud.points[0].vp_y, tcloud.points[0].vp_z, cloudout);
		  return 0;
	  }
	  return -1;
}


void myFlipNormals(double vx, double vy, double vz, pcl::PointCloud<pcl::PointXYZINormal> &ncloud){
	for(uint i=0;i<ncloud.points.size();i++){
		pcl::PointXYZINormal p=ncloud.points[i];
		double dotprod=(p.normal[0]*(vx-p.x)+p.normal[1]*(vy-p.y)+p.normal[2]*(vz-p.z));
		if(dotprod<0){
//			cout<<"flipping"<<endl;
			ncloud.points[i].normal[0]*=-1.0;
			ncloud.points[i].normal[1]*=-1.0;
			ncloud.points[i].normal[2]*=-1.0;
		}
	}
}
//void myFlipNormals(double vx, double vy, double vz, pcl::PointCloud<pcl::Normal> &ncloud){
//	for(uint i=0;i<ncloud.points.size();i++){
//		pcl::PointXYZINormal p=ncloud.points[i];
//		double dotprod=(p.normal[0]*(vx-p.x)+p.normal[1]*(vy-p.y)+p.normal[2]*(vz-p.z));
//		if(dotprod<0){
////			cout<<"flipping"<<endl;
//			ncloud.points[i].normal[0]*=-1.0;
//			ncloud.points[i].normal[1]*=-1.0;
//			ncloud.points[i].normal[2]*=-1.0;
//		}
//	}
//}

void getNormals(pcl::PointCloud<pcl::PointWithViewpoint> &cloudin, pcl::PointCloud<pcl::PointXYZINormal> &cloud_normals){
	timeval t0=g_tick();
	pcl::PointCloud<Point>::ConstPtr cloud_ = boost::make_shared<const pcl::PointCloud<Point> > (cloudin);
	pcl::KdTree<Point>::Ptr normals_tree_;
    int k_ = 10;                                // 50 k-neighbors by default
    normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
    pcl::NormalEstimation<Point, pcl::PointXYZINormal> n3d_;
    n3d_.setKSearch (k_);
    //n3d_.setRadiusSearch (0.015);
    n3d_.setSearchMethod (normals_tree_);
    // ---[ Estimate the point normals
//    pcl::PointCloud<pcl::Normal> cloud_normals;
//      n3d_.setSearchSurface (cloud_);
    n3d_.setInputCloud (cloud_);
    n3d_.compute (cloud_normals);
    for(uint i=0;i<cloudin.points.size(); i++){
    	cloud_normals.points[i].x=cloudin.points[i].x;
    	cloud_normals.points[i].y=cloudin.points[i].y;
    	cloud_normals.points[i].z=cloudin.points[i].z;
    }
    std::cout<<"done."<<endl;
	std::cout<<"finding normals took:  "<<g_tock(t0)<<"  for "<<cloud_normals.points.size()<<" indices."<<std::endl;
}

void getNormals(pcl::PointCloud<Point> &cloudin, pcl::PointCloud<pcl::Normal> &cloud_normals){
	timeval t0=g_tick();
	pcl::PointCloud<Point>::ConstPtr cloud_ = boost::make_shared<const pcl::PointCloud<Point> > (cloudin);
	pcl::KdTree<Point>::Ptr normals_tree_;
    int k_ = 10;                                // 50 k-neighbors by default
    normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
    pcl::NormalEstimation<Point, pcl::Normal> n3d_;
    n3d_.setKSearch (k_);
    //n3d_.setRadiusSearch (0.015);
    n3d_.setSearchMethod (normals_tree_);
    // ---[ Estimate the point normals
//    pcl::PointCloud<pcl::Normal> cloud_normals;
//      n3d_.setSearchSurface (cloud_);
    n3d_.setInputCloud (cloud_);
    n3d_.compute (cloud_normals);
//    myFlipNormals(cloudin.points[0].vp_x,cloudin.points[0].vp_y,cloudin.points[0].vp_z,cloud_normals);
	std::cout<<"finding normals took:  "<<g_tock(t0)<<"  for "<<cloud_normals.points.size()<<" indices."<<std::endl;
}


//this function just extracts the normals from the integrated cloud
void getNormals(pcl::PointCloud<pcl::PointXYZINormal> &cloudin, pcl::PointCloud<pcl::Normal> &cloud_normals){
	pcl::Normal p;
    for(uint i=0;i<cloudin.points.size(); i++){
    	p.normal[0]=cloudin.points[i].normal[0];
    	p.normal[1]=cloudin.points[i].normal[1];
    	p.normal[2]=cloudin.points[i].normal[2];
    	cloud_normals.points.push_back(p);
    }
}


//Why is this so slow?
void getNormalsOMP(pcl::PointCloud<Point> &cloudin, pcl::PointCloud<pcl::Normal> &cloud_normals){
	  std::cout<<"finding normals..."<<endl;
	pcl::PointCloud<Point>::ConstPtr cloud_ = boost::make_shared<const pcl::PointCloud<Point> > (cloudin);
	pcl::KdTree<Point>::Ptr normals_tree_;
    int k_ = 10;                                // 50 k-neighbors by default
    normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
    pcl::NormalEstimationOMP<Point, pcl::Normal> n3d_;
    n3d_.setNumberOfThreads(2);
    n3d_.setKSearch (k_);
    //n3d_.setRadiusSearch (0.015);
    n3d_.setSearchMethod (normals_tree_);
    // ---[ Estimate the point normals
//    pcl::PointCloud<pcl::Normal> cloud_normals;
//      n3d_.setSearchSurface (cloud_);
    n3d_.setViewPoint(cloudin.points[0].vp_x,cloudin.points[0].vp_y,cloudin.points[0].vp_z);
    n3d_.setInputCloud (cloud_);
    n3d_.compute (cloud_normals);
    std::cout<<"done."<<endl;
}


//void getVertical(pcl::PointCloud<Point> &cloudin){
//	pcl::PointIndices ind;
//    pcl::PassThrough<pcl::Normal> pass_;
//    pass_.setFilterFieldName ("normal[2]");
//    pass_.setFilterLimits (min_z_bounds_, max_z_bounds_);
//    pass_.setInputCloud (boost::make_shared<const pcl::PointCloud<Point> > (cloudin));
//    pass_.filter (cloud_filtered);
//
//
//}

//could do this with a passthrough, but come on...
void segmentFloor(pcl::PointCloud<Point> &cloudin, pcl::PointCloud<Point> &cloudout,bool negativeFilter){
	timeval t0=g_tick();
	std::vector<int> ind;
	for(uint i=0;i<cloudin.points.size(); i++){
		if(abs(cloudin.points[i].z) <.05){
			ind.push_back(i);
		}
	}
	getSubCloud(cloudin,ind,cloudout,negativeFilter);
	std::cout<<"segmentFloor took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}



//could do this with a passthrough, but come on...
void segmentPplHeight(pcl::PointCloud<Point> &cloudin, pcl::PointCloud<Point> &cloudout){
	timeval t0=g_tick();
	std::vector<int> ind;
	for(uint i=0;i<cloudin.points.size(); i++){
		if(cloudin.points[i].z > 0.0 && cloudin.points[i].z < 2.0){
			ind.push_back(i);
		}
	}
	getSubCloud(cloudin,ind,cloudout,false);
	std::cout<<"segmentPplHeight took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}


void removeOutliers(pcl::PointCloud<Point> &cloudin, pcl::PointCloud<Point> &cloudout){
	timeval t0=g_tick();
	pcl::StatisticalOutlierRemoval<Point> sor2;
	sor2.setInputCloud (boost::make_shared<pcl::PointCloud<Point> >(cloudin));
	sor2.setMeanK (50);
	sor2.setStddevMulThresh (1.0);
	sor2.filter (cloudout);
	std::cout<<"filtering outliers took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}

void downSample(pcl::PointCloud<Point> &cloudin, pcl::PointCloud<Point> &cloudout){
	timeval t0=g_tick();
	// Create the filtering object
	pcl::VoxelGrid<Point> sor;
	sor.setInputCloud (boost::make_shared<pcl::PointCloud<Point> > (cloudin));
	sor.setLeafSize (0.01, 0.01, 0.01);
	sor.filter (cloudout);
	std::cout<<"downsampling took:  "<<g_tock(t0)<<"  for "<<cloudout.points.size()<<" indices."<<std::endl;
}

double ptdist(Point a,Point b){
	return (a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)+(a.z-b.z)*(a.z-b.z);
}





void segfast(pcl::PointCloud<Point> &cloud, std::vector<pcl::PointCloud<Point> > &cloud_clusters, double cluster_tol){
	double smaller_tol = cluster_tol/2.3;
	timeval t0=g_tick();
	pcl::KdTreeFLANN<Point> tree,tree2;
	tree.setInputCloud(boost::make_shared<pcl::PointCloud<Point> > (cloud));
	vector<int> minitree(cloud.points.size(),-1);
	vector<int> heads,indices;
	vector<float> dists;

	//First 'downsample' the points into clusters slightly less than half the cluster tolerance
	//minitree keeps track of who each point belongs to (which is an index of the 'head' array)
	for(uint i=0; i<cloud.points.size();i++){
	  if(minitree[i]==-1){    //if no one has claimed this point, make it a head
		  heads.push_back(i);
		  if(!tree.radiusSearch(cloud,i,smaller_tol,indices,dists))  //find all the points close to this point
			  cout<<"radius search failed!"<<endl;
		  for(uint j=0;j<indices.size();j++){
			  minitree[indices[j]]=heads.size()-1; //assign these points to the current head.
			  	  	  	  	  	  	  	  	  	   // this overwrites previous claims, but it's ok
		  }
	  }
	}
//	std::cout<<"radiusSearch took:  "<<g_tock(t0)<<"s  found "<<heads.size()<<" heads"<<std::endl;

	//now, we have much fewer points to cluster, but since the initial downsampling was less than
	//half the cluster tolerance,we are guaranteed to find all the points within the tolerance
	//(except if  [ clustertol < distance between heads < cluster_tol+ smaller cluster tol] AND the head closest points in the two heads are < cluster_tol
	//I need to make something to deal with that...
	tree2.setInputCloud(boost::make_shared<pcl::PointCloud<Point> > (cloud),boost::make_shared<std::vector<int> >  (heads));
	int searching,currenthead;
	//heads2 is the cluster id.  we now search to make sure we check all the points in our cluster
	//minitree2 corresponds to the heads array, so the points in minitree2 are at heads[i]
	vector<int> heads2, minitree2(heads.size(),-1);
	list<int> tosearch;
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
//			  cout<<i<<" --> "<<searching<<" --> "<<indices.size()<<endl;
			  for(uint j=0;j<indices.size();j++)
				  if(minitree2[indices[j]]==-1){//found untouched point (which means this root touched it)
					  minitree2[indices[j]]=currenthead; //claim it
					  tosearch.push_back(indices[j]);   //add it to list of points to search
				  }
		  }
	  }

	}

	timeval t1=g_tick();
	vector<int> indices1;
	vector<float> dists1;
	vector<int> deletedclusters;
	//now we need to check to see if any of the heads that were NOT clustered together are closer than cluster_tol+smaller_tol:
	for(uint i=0; i<minitree2.size();i++){
		tree2.radiusSearch(cloud.points[heads[i]],cluster_tol+smaller_tol,indices,dists);
		for(uint j=0;j<indices.size();j++)
			if(minitree2[indices[j]] != minitree2[i] && i<j){//if the two heads are not in the same cluster
				cout<<"head "<<i<<" ("<<minitree2[i]<<") is "<<sqrt(dists[j])<<" from head "<<indices[j]<<" ("<<minitree2[indices[j]]<<")"<<endl;
				//now we have to find if there is a point between head a and head b that is < cluster_tol from both
				Point heada=cloud.points[heads[i]], headb=cloud.points[heads[indices[j]]];
				Point inbetween((heada.x+headb.x)/2.0,(heada.y+headb.y)/2.0,(heada.z+headb.z)/2.0,0,0,0);
				tree.radiusSearch(inbetween,cluster_tol,indices1,dists1); //search in the full tree
				double distthresh=cluster_tol*cluster_tol; //radius search gives squared distances...
				for(uint k=0;k<indices1.size();k++){
					if(ptdist(cloud.points[indices1[k]],heada ) < distthresh && ptdist(cloud.points[indices1[k]],headb ) < distthresh ){
						//there is a point that these two clusters share -> they should be merged
						cout<<"head "<<i<<" ("<<minitree2[i]<<") shares "<<indices1[k]<<" with head "<<indices[j]<<" ("<<minitree2[indices[j]]<<")"<<endl;
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
	vector<vector<int> > clusters(heads2.size());
	//now, for each point in the cloud find it's head --> then the head it clustered to. that is it's cluster id!
	for(uint j=0;j<minitree.size();j++){
	  clusters[minitree2[minitree[j]]].push_back(j);
	}


	std::cout<<"clustering took:  "<<g_tock(t0)<<"s  found "<<heads2.size()<<" heads"<<std::endl;
	for(uint j=0;j<heads2.size();j++){
	  cout<<"cluster "<<j<<"  -->  "<<clusters[j].size()<<endl;
	}

	cloud_clusters.resize(clusters.size());
	for(uint i=0;i<clusters.size(); i++){
	  getSubCloud(cloud,clusters[i],cloud_clusters[i]);
	}

}






