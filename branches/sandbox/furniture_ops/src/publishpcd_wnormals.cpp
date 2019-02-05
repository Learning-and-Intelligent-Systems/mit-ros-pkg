#include <ros/ros.h>

#include <boost/thread.hpp>
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include <string>
#include <iostream>
#include <cmath>

#include <sensor_msgs/PointCloud2.h>
#include <mapping_msgs/PolygonalMap.h>

typedef pcl::PointWithViewpoint Point;
#include "pcl_helpers.h"


using namespace std;

double xcenter,ycenter,zcenter;
double xtol,ytol,ztol;




mapping_msgs::PolygonalMap makePMap(pcl::PointCloud<pcl::PointXYZINormal> ncloud){
	mapping_msgs::PolygonalMap pmap;
	pmap.header.stamp=ros::Time::now();
	pmap.header.frame_id="/odom_combined";
	geometry_msgs::Polygon p;
	p.points.resize(2);
	for(int i=0;i<ncloud.points.size();i++){
//		double hp=ncloud.points[i].normal[2]>=0 ? .10 : -.10;
//		if(abs(ncloud.points[i].x-xcenter)<xtol && abs(ncloud.points[i].y-ycenter)<ytol && abs(ncloud.points[i].z-zcenter)<ztol){
//		if(ncloud.points[i].normal[0]>0.0 && ncloud.points[i].normal[1]<0.0){
		p.points[0].x=ncloud.points[i].x;
		p.points[0].y=ncloud.points[i].y;
		p.points[0].z=ncloud.points[i].z;
		p.points[1].x=ncloud.points[i].x + ncloud.points[i].normal[0]*.1;
		p.points[1].y=ncloud.points[i].y + ncloud.points[i].normal[1]*.1;
		p.points[1].z=ncloud.points[i].z + ncloud.points[i].normal[2]*.1;
//		cout<<ncloud.points[i]<<endl;


		pmap.polygons.push_back(p);
//		}
	}
//	p.points.clear();
//	geometry_msgs::Point32 pt;
//	pt.x=xcenter+xtol;
//	pt.y=ycenter+ytol;
//	pt.z=zcenter+ztol;
//	p.points.push_back(pt);
//	pt.x-=2*xtol; p.points.push_back(pt);
//	pt.y-=2*ytol; p.points.push_back(pt);
//	pt.x+=2*xtol; p.points.push_back(pt);
//	pt.y+=2*ytol; p.points.push_back(pt);
//	pt.z-=2*ztol; p.points.push_back(pt);
//	pt.x-=2*xtol; p.points.push_back(pt);
//	pt.y-=2*ytol; p.points.push_back(pt);
//	pt.x+=2*xtol; p.points.push_back(pt);
//	pt.y+=2*ytol; p.points.push_back(pt);
//	pmap.polygons.push_back(p);



	return pmap;
}




int
  main (int argc, char** argv)
{
	ros::init(argc, argv, "from_pcd");
	ros::NodeHandle nh_;

	string filename;
	if(argc>1){
		filename=argv[1];
	}
	xcenter=-1.0; ycenter=0.0; zcenter=.4;
	xtol=.10; ytol=0.10; ztol=.1;

	if(argc>4){
		xcenter=atof(argv[2]);
		ycenter=atof(argv[3]);
		zcenter=atof(argv[4]);
	}

	sensor_msgs::PointCloud2 cloud2;
	pcl::PointCloud<pcl::PointWithViewpoint> cloud;
	pcl::PointCloud<pcl::PointXYZINormal> ncloud;
//	try{
//		readPCD(filename,cloud);
//		getNormals(cloud,ncloud);
//	}catch(pcl::InvalidConversionException){
		readPCD(filename,ncloud);
//	}
//	pcl::PointWithViewpoint cloud.points[0] = cloud.points[0];
//	myFlipNormals(cloud.points[0].vp_x,cloud.points[0].vp_y,cloud.points[0].vp_z,ncloud);
	mapping_msgs::PolygonalMap pmap=makePMap(ncloud);
//	cloud.points[0].x=cloud.points[0].vp_x;
//	cloud.points[0].y=cloud.points[0].vp_y;
//	cloud.points[0].z=cloud.points[0].vp_z;
//	cout<<cloud.points[0]<<endl;


	 ros::Publisher pub_map = nh_.advertise<mapping_msgs::PolygonalMap> ("pcd_norms", 100);

	  pcl::toROSMsg (ncloud,cloud2);
//		cloud.points.push_back(pcl::PointWithViewpoint(p.vp_x,p.vp_y,p.vp_z));
     ros::Publisher pub_points2_ = nh_.advertise<sensor_msgs::PointCloud2> ("from_pcd", 100);
// 	cout<<cloud.points[1]<<endl;
     for(int i=0;i<3;i++){
    	 cout<<"publishing..."<<endl;
    	 cloud2.header.frame_id="/odom_combined";
		 pub_points2_.publish (cloud2);
		 pub_map.publish(pmap);
		 sleep(1);
     }
  return (0);
}

