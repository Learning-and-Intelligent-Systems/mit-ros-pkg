#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <boost/thread/thread.hpp>
#include "pcl/common/common_headers.h"
#include "pcl/common/common_headers.h"
#include "pcl/features/normal_3d.h"
#include "pcl/io/pcd_io.h"
//#include "visualization_files/pcl_visualizer.h"
#include <pcl/visualization/pcl_visualizer.h>
//#include "parse.h"
#include <cardboard/detection.h>
#include <cardboard/models.h>
#include <cardboard/optimization.h>
#include <cardboard/ply.h>
#include <cardboard/util.h>
#include "fitness.h"
#include <cardboard/common.h>
#include <cardboard/testing.h>
#include <cardboard/SceneHypothesis.h>
#include <cardboard/DetectModels.h>
#include <cardboard/AlignModels.h>
#include <pthread.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <ros/ros.h>
//#include "visualization_files/Joy.h"
#include <sensor_msgs/Joy.h>
#include <string>

Eigen::Matrix4f trans = Matrix4f::Identity();
geometry_msgs::Point point = cardboard::affine_matrix_to_pose(trans).position;
Eigen::Matrix4f rot = Matrix4f::Identity();
Eigen::Matrix4f current_trans;
char* bagName;
char* modelName;
string fname;
int hold = 0;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
pcl::PointCloud<pcl::PointXYZRGB> model;

class joystickControl{
public:
	joystickControl();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	bool isX(const sensor_msgs::Joy::ConstPtr& joy);
	bool isTriangle(const sensor_msgs::Joy::ConstPtr& joy);
	void write();
	
	int forward, right, up, zRotL, zRotR, xDown, yRot, xRot, triangle;
	ros::NodeHandle nh;
	ros::Subscriber joy_sub;
};


joystickControl::joystickControl():
	forward(1),right(0),up(3),zRotL(8),zRotR(9),xDown(14),yRot(16),xRot(17), triangle(12)
{
	nh.param("forward", forward, forward);
	nh.param("right", right, right);
	nh.param("up", up, up);
	nh.param("zRotL", zRotL, zRotL);
	nh.param("zRotR", zRotR, zRotR);
	nh.param("xDown", xDown, xDown);
	nh.param("yRot", yRot, yRot);
	nh.param("xRot", xRot, xRot);
	nh.param("triangle", triangle, triangle);

	joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &joystickControl::joyCallback, this);
}

bool joystickControl::isX(const sensor_msgs::Joy::ConstPtr& joy){
	if(joy->axes[xDown] != 0){
		return true;
	}
	else{
		return false;
	}
}

bool joystickControl::isTriangle(const sensor_msgs::Joy::ConstPtr& joy){
	if(joy->axes[triangle] != 0){
		return true;
	}
	else{
		return false;
	}
}

void joystickControl::write(){
        /*
	ofstream write(("pcds/"+fname).c_str(), ios::app);
	std::cout<<"pcds/"+fname<<std::endl;
	if (!write.is_open()){
		std::cout<<"Creating "<<fname<<std::endl;
		//write.open()
	}
	
	//write << "BagFile Name: "<< bagName <<"\n" << "\n";
	//write << "Model name: "<<modelName <<"\n";
	*/
  geometry_msgs::Pose final = cardboard::affine_matrix_to_pose(trans);
  geometry_msgs::Pose qFinal = cardboard::affine_matrix_to_pose(rot);


  FILE *fp = fopen(fname.c_str(), "w");
  fprintf(fp, "%s %f %f %f %f %f %f %f\n",  modelName, final.position.x, final.position.y, final.position.z,
	  qFinal.orientation.w, qFinal.orientation.x, qFinal.orientation.y, qFinal.orientation.z);
  fclose(fp);

  std::cout << modelName << " " << final.position.x <<" "<<final.position.y<<" "<<final.position.z<<" "
	    <<qFinal.orientation.w<<" "<<qFinal.orientation.x<<" "<<qFinal.orientation.y<<" "<<qFinal.orientation.z<<"\n";

  std::cout <<"saved"<<std::endl;



}

void joystickControl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	Eigen::Vector3f mat(0,0,0);
	Eigen::Matrix4f A_trans;
	if (isTriangle(joy)){
		if (hold==0)
			write();
	hold++;
	if(hold>8)
		hold = 0;
	}

	float scale = 1.0/50.0;//32767.0;
	Eigen::Vector3f mod_trans(-1*scale*joy->axes[right],-1*scale*joy->axes[up],scale*joy->axes[forward]);
	Quaternionf quat(1,0,0,0);
	A_trans = cardboard::pose_to_affine_matrix(mod_trans,quat);

	if (isX(joy)){
	  Quaternionf quatY(cos((joy->axes[yRot]/*+joy->axes[up]*/)/10.0),0,0,sin((joy->axes[yRot]/*+joy->axes[up]*/)/10.0));
	  Eigen::Matrix4f rot_y = cardboard::pose_to_affine_matrix(mat,quatY);
	  Quaternionf quatX(cos((joy->axes[xRot]/*+joy->axes[right]*/)/10.0),sin((joy->axes[xRot]/*+joy->axes[right]*/)/10.0),0,0);
	  Eigen::Matrix4f rot_x = cardboard::pose_to_affine_matrix(mat,quatX);
	  Quaternionf quatZ(cos((-1*joy->axes[zRotL]+joy->axes[zRotR])/50.0),0,sin((-1*joy->axes[zRotL]+joy->axes[zRotR])/50.0),0);
	  Eigen::Matrix4f rot_z = cardboard::pose_to_affine_matrix(mat,quatZ);

	  Eigen::Matrix4f all_rot = rot_x*rot_y*rot_z;
	  rot = rot*all_rot;

	  trans = current_trans*trans;
	  //pcl::PointCloud<pcl::PointXYZ>::Ptr model_new (new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_new (new pcl::PointCloud<pcl::PointXYZRGB>);
	  pcl::transformPointCloud(model, *model_new, rot);
	  pcl::transformPointCloud(*model_new, *model_new, trans);

	  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (model_new,255,0,0);
	  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2 (model_new);

	  //viewer->removePointCloud("Cloud from Model PCD",0);
	  //viewer->addPointCloud(model_new,single_color, "Cloud from Model PCD");
	  //viewer->updatePointCloud(model_new, single_color, "Cloud from Model PCD");
	  viewer->updatePointCloud(model_new, rgb2, "Cloud from Model PCD");

	  printf("updated point cloud\n");
	  //printf

	}
	current_trans = A_trans;

}


boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_simple(pcl::PointCloud<pcl::PointXYZRGB>::Ptr bag_cloud, pcl::PointCloud<pcl::PointXYZRGB> &model_cloud){
//boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_simple(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr bag_cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr model_cloud){
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0,0,0);
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(*bag_cloud);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(bag_cloud);
	//viewer->addPointCloud(*bag_cloud, rgb,"Cloud from bagFile");
	viewer->addPointCloud(bag_cloud, rgb,"Cloud from bagFile");
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb2(model_cloud.makeShared());
	//viewer->addPointCloud(*model_cloud, "Cloud from Model PCD");
	viewer->addPointCloud(model_cloud.makeShared(), rgb2, "Cloud from Model PCD");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "Cloud from bagFile");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud from Model PCD");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->setBackgroundColor(.3,.3,.3,0);
	return viewer;

}


int main(int argc, char** argv){
	fname = argv[3];
	ros::init(argc, argv, "ps3");
	std::cout << "init" << std::endl;
	joystickControl ps3;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	bagName = argv[1];
	pcl::io::loadPCDFile(argv[1], *cloud_ptr);
	std::cout << "loaded" << std::endl;
	pcl::io::loadPCDFile(argv[2], model);
	modelName = argv[2];
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	viewer = vis_simple(cloud_ptr, model);
	std::cout << "function ran" << std::endl;
	ros::Rate r(100);
	//viewer->updateCamera();
	while(!viewer->wasStopped() && ros::ok()){
		viewer->spinOnce(100);
		ros::spinOnce();
		//std::cout<<viewer->camera_.pos[0]<<std::endl;
		//viewer->rens_->InitTraversal();
		//double pos[3];

		//vtkCamera* kam;

		//ROS_INFO("ya");
		//kam = viewer->rens_->GetNextItem()->GetActiveCamera();
		//assert(kam!=NULL);
		//ROS_INFO("one");
		//kam->GetPosition(pos[0],pos[1],pos[2]);




		//std::cout<<pos[0]<<std::endl;

		//trans = current_trans*trans;
		//pcl::PointCloud<pcl::PointXYZ>::Ptr model_new (new pcl::PointCloud<pcl::PointXYZ>);
		//pcl::transformPointCloud(*model_ptr, *model_new, rot);
		//pcl::transformPointCloud(*model_new, *model_new, trans);
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (*model_new,255,0,0);
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color (model_new,255,0,0);
		//viewer->removePointCloud("Cloud from Model PCD",0);
		//viewer->addpointcloud(*model_new,single_color, "cloud from model pcd");
		//viewer->addPointCloud(model_new,single_color, "Cloud from Model PCD");
		//boost::this_thread::sleep (boost::posix_time::microseconds(1000000));
		//r.sleep();

	}
	return 0;
}




