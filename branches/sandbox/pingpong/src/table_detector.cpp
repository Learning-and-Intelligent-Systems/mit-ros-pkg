#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include "util.h"
#include <pingpong/TableCoordinates.h>
#include <geometry_msgs/Point32.h>


ros::Publisher cloud_pub,pose_pub,pointsInPlane_pub;
ros::Subscriber kinect_sub;
float table_score = 0;
Eigen::Vector3f table_pose;
Eigen::Vector4f plane_eq;
std::vector<pcl::PointXYZ> corners;
int counter = 0;

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr)
{   std::cout << "Got new pointcloud!" << std::endl;
	// convert to pcl pointcloud
    sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2());
	pcl::PointCloud<pcl::PointXYZ> plane_finding_cloud,cloud_from_kinect_downsampled;
    //pcl::fromROSMsg(*cloud_ptr, cloud_from_kinect);


    // down-sample the point cloud
    pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  	sor.setInputCloud (cloud_ptr);
  	sor.setLeafSize (0.02f, 0.02f, 0.02f);
  	sor.filter (*cloud_filtered);

    pcl::fromROSMsg(*cloud_filtered, cloud_from_kinect_downsampled);

    plane_finding_cloud = cloud_from_kinect_downsampled;


    std::cout << "Downsampled" << std::endl;
  	// # points in cloud_from_kinect_downsampled
    int nr_points = (int) cloud_from_kinect_downsampled.points.size();

std::cout << "Has " << nr_points<<std::endl;
    
    
    // Create segmentation object for planes
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(.08);

 std::cout << "Downsampled" << std::endl;

    std::vector<pcl::PointCloud<pcl::PointXYZ> > planes;
    std::vector<Eigen::Vector4f> plane_centroids;

    // cloud for filtering purposes
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ> ());
     std::cout << "Downsampled" << std::endl;

    while (cloud_from_kinect_downsampled.points.size() > .2 * nr_points)
    {   std::cout << "In loop" << std::endl;
        // find largest plane

        seg.setInputCloud(cloud_from_kinect_downsampled.makeShared());
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "could not fit a plane in this cloud" << std::endl;
            break;
        }
        // find the plane's indicies 
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_from_kinect_downsampled.makeShared());
        extract.setIndices(inliers);
 std::cout << "halfway through" << std::endl;


        // remove the plane to its own cloud
        extract.setNegative(false);
        pcl::PointCloud<pcl::PointXYZ> plane_cloud;
        extract.filter(plane_cloud);
        // add plane to vector
 std::cout << "three quarters" << std::endl;
std::cout << plane_cloud.points.size() << std::endl;
        planes.push_back(plane_cloud);
 std::cout << "three quarters" << std::endl;

        Eigen::Vector4f plane_centroid;

        pcl::compute3DCentroid(plane_cloud, plane_centroid);
        plane_centroids.push_back(plane_centroid);

        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_from_kinect_downsampled = *cloud_f; 
        
        
    }
 std::cout << "out" << std::endl;



    //std::cout << "Segmented planes" << std::endl;

    double minDistSquared = std::numeric_limits<float>::max();
    pcl::PointCloud<pcl::PointXYZ> nearestPlane;
    Eigen::Vector3f table_plane_centroid;
    // find closest plane to sensor
    for (int i = 0; i < planes.size(); i++)
    {
        Eigen::Vector4f current_centroid = plane_centroids[i];
        double distSquared = current_centroid[0]*current_centroid[0] + current_centroid[1]*current_centroid[1] + current_centroid[2]*current_centroid[2];
        if (distSquared < minDistSquared)
        {
            minDistSquared = distSquared;
            nearestPlane = planes[i];
            table_plane_centroid[0] = current_centroid[0];
            table_plane_centroid[1] = current_centroid[1];
            table_plane_centroid[2] = current_centroid[2];
        }
    }

    std::cout << "Found nearest plane" << std::endl;

    // find equation of this plane
    Eigen::Vector4f plane_parameters;
    float curvature;
    pcl::computePointNormal(nearestPlane, plane_parameters, curvature);
    std::cout << "plane parameters are (" << plane_parameters[0] << "," << plane_parameters[1]<<","<<plane_parameters[2]<<","<<plane_parameters[3]<<")" << std::endl;

    Eigen::Vector3f guess;
    guess(0) = table_plane_centroid(0);
    guess(1) = table_plane_centroid(1);
    guess(2) = M_PI/2 + M_PI;

    Eigen::Vector3f noise;
    noise << .04,.04,.005;

  	TablePositionOptimizer opt = TablePositionOptimizer(.05, noise , 1.0, plane_parameters, plane_finding_cloud);
    std::cout << "Finding Table Position..." << std::endl;
    std::cout << "guess = (" << guess[0] << "," << guess[1] << "," << guess[2] << ")" << std::endl;
    Eigen::Vector3f tablePos = opt.optimize(guess);
    //tablePos = opt.optimize(tablePos);
    std::cout << "Table at (" << tablePos[0] << "," << tablePos[1] << "," << tablePos[2] << ")" << std::endl;
    std::cout << "Score was " << -1 * opt.getFinalCost() << std::endl;
    std::cout << std::endl;

    if (-1 * opt.getFinalCost() > table_score)
    {
        table_score = -1 * opt.getFinalCost();
        table_pose = tablePos;
        plane_eq = plane_parameters;
        std::cout << "Estimate Updated!" << std::endl;
    }

    opt = TablePositionOptimizer(.05, noise, 1.0, plane_eq, plane_finding_cloud);
    corners = opt.corners(table_pose);

    pcl::PointCloud<pcl::PointXYZ> table_corners;
    table_corners = cloud_from_kinect_downsampled;
    table_corners.clear();
    table_corners.points.push_back(corners[0]);
    table_corners.points.push_back(corners[1]);
    table_corners.points.push_back(corners[2]);
    table_corners.points.push_back(corners[3]); // Origin, 0-3 = x axis, 2 - 3 = y axis, 
    table_corners.width = table_corners.points.size();
    table_corners.height = 1;
    table_corners.is_dense = true;

    opt.FillPointsInPlane(table_pose);

    // publish clouds for visualization
    sensor_msgs::PointCloud2 table_corners_msg,plane_points_msg;
    pcl::toROSMsg(table_corners, table_corners_msg);
    pcl::toROSMsg(opt.pointsInPlane, plane_points_msg);
    cloud_pub.publish(table_corners_msg);
    pointsInPlane_pub.publish(plane_points_msg);

    counter++;
    if (counter == 5)
    {   kinect_sub.shutdown();
        std::cout <<"Shutting down" << std::endl;
    }
	return;
}

void tester()
{

    Eigen::Vector3f p1,p2,p3,p4,p5,p6,p7,p8;
    Eigen::Vector4f xy, yz, xz;
    pcl::PointXYZ po1, po2, po3, po4, po5, po6, po7, po8;

    po4.x = .52;
    po4.y = .75;
    po4.z = 2;

    xy << 0,0,1,0;

    p1 << 0,0,0;
    p2 << 1,0,0;
    p3 << 0,1,0;
    p4 << 0,0,1;

    p5 << 1,1,0;
    p6 << .5,.5,0;

    std::cout << in_rectangle(p1,p2,p5,p3,p6) << std::endl;
    std::cout << dist_from_plane(po4, xy) << std::endl;

    p7 = project_to_plane(po4,xy);

    std::cout << p7 << std::endl;



    return;
}

bool makeTransform(pingpong::TableCoordinates::Request &req,
                    pingpong::TableCoordinates::Response &res)
{   std::cout << "Sending Coordinate Transform" << std::endl;
    Eigen::Vector3f origin, x_axis, y_axis, z_axis; 
    origin << corners[3].x,corners[3].y,corners[3].z;
    y_axis << corners[0].x,corners[0].y,corners[0].z;
    x_axis << corners[2].x,corners[2].y,corners[2].z;
    y_axis = y_axis - origin;
    y_axis.normalize();
    x_axis = x_axis - origin;
    x_axis.normalize();
    z_axis = x_axis.cross(y_axis);
    z_axis.normalize();
    res.origin.x = origin(0);
    res.origin.y = origin(1);
    res.origin.z = origin(2);
    res.x_axis.x = x_axis(0);
    res.x_axis.y = x_axis(1);
    res.x_axis.z = x_axis(2);
    res.y_axis.x = y_axis(0);
    res.y_axis.y = y_axis(1);
    res.y_axis.z = y_axis(2);
    res.z_axis.x = z_axis(0);
    res.z_axis.y = z_axis(1);
    res.z_axis.z = z_axis(2);

    std::cout << "Sent Message" << std::endl;
    return true;
}





int main(int argc, char **argv)
{   //tester();
    //return 0;
    ros::init(argc, argv, "table_detector");
    
    ros::NodeHandle n;
    // Initialilze ball position publisher and pointcloud subscriber
    pose_pub = n.advertise<geometry_msgs::Pose>("table_pose", 1000);
    cloud_pub = n.advertise<sensor_msgs::PointCloud2>("table", 1000);
    pointsInPlane_pub = n.advertise<sensor_msgs::PointCloud2>("pointsInPlane", 1000);
    //difference_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("nearest_plane", 1000);
    
    kinect_sub = n.subscribe("camera/depth_registered/points", 1000, pointCloudCallback);
    
    ros::ServiceServer service = n.advertiseService("KinectToTableSpaceTransform", makeTransform);

    /*ros::Rate rate(250);
    while(n.ok()) 
    {   
        std::cout << counter << std::endl;
        if (counter >= 4)
        {   kinect_sub.shutdown();
            std::cout <<"Shutting down" << std::endl;
            break;

        }        
        ros::spinOnce();
        rate.sleep();

    }*/
    ros::spin();
    
    return 0;
}
