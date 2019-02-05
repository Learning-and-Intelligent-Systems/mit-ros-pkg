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
#include "util.h"
#include <pingpong/TableCoordinates.h>
#include <geometry_msgs/Point32.h>
#define THRESH .005

// Global variables
ros::Publisher ball_pub, filtered_cloud_pub, difference_cloud_pub; // Ball location publisher
//pcl::PointCloud<pcl::PointXYZ> last_filtered_cloud; // last filtered cloud
std::vector<Eigen::Vector4f> last_frame_centroids;
// Finds ball in pointcloud from kinect

Eigen::Vector3f origin,x_axis,y_axis,z_axis,p1,p2,p3,p4;
Eigen::Vector4f table_plane_equation;




Eigen::Vector3f TransformToTableCoordinates(const Vector3f& in)
{   
    Eigen::Vector3f originToPoint = in - origin;
    Eigen::Vector3f out;
    out << originToPoint.dot(x_axis),originToPoint.dot(y_axis),originToPoint.dot(z_axis);
    return out;
}

Eigen::Vector3f TransformFromTableCoordinates(const Vector3f& in)
{
    Eigen::Vector3f out = in(0) * x_axis + in(1) * y_axis + in(2) * z_axis + origin;
    return out;
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_ptr)
{   
    //table rectangle




    // convert sensor_msg pointcloud2 to pcl point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud_from_kinect,pointsAboveTable;
    pcl::fromROSMsg(*cloud_ptr, cloud_from_kinect);


    pcl::ExtractIndices<pcl::PointXYZ> ext;
    pcl::PointIndices::Ptr inBoxAbovePlane (new pcl::PointIndices);
    pcl::PointXYZ curPoint;
    Eigen::Vector3f cp, orToCp;
    for (int i = 0; i < cloud_from_kinect.points.size(); i++)
    {   curPoint = cloud_from_kinect.points[i];
        cp << curPoint.x,curPoint.y,curPoint.z;
        orToCp = cp - origin;
        if (orToCp.dot(z_axis) > .04 && in_rectangle(p1,p2,p3,p4,cp))
            inBoxAbovePlane->indices.push_back(i);
    }

    ext.setInputCloud(cloud_from_kinect.makeShared());
    ext.setIndices(inBoxAbovePlane);
    ext.setNegative(false);
    ext.filter(pointsAboveTable);

    sensor_msgs::PointCloud2 pointsAboveTable_msg;
    pcl::toROSMsg(pointsAboveTable, pointsAboveTable_msg);
    filtered_cloud_pub.publish(pointsAboveTable_msg);




/*


    // filter out all points past certain 'z'
    pcl::PointCloud<pcl::PointXYZ> cloud_from_kinect_filtered;
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_from_kinect.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0,4.0);
    pass.filter(cloud_from_kinect_filtered);
    std::cout << "filtered out points beyond 4 meters " << std::endl;  
    cloud_from_kinect = cloud_from_kinect_filtered;

    // # points in cloud_from_kinect
    int nr_points = (int) cloud_from_kinect.points.size();
    
    // cloud for filtering purposes
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ> ());
    
    // Create segmentation object for planes
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(.08);

    std::vector<pcl::PointCloud<pcl::PointXYZ> > planes;
    std::vector<Eigen::Vector4f> plane_centroids;
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    while (cloud_from_kinect.points.size() > .2 * nr_points)
    {   
        // find largest plane
        seg.setInputCloud(cloud_from_kinect.makeShared());
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "could not fit a plane in this cloud" << std::endl;
            break;
        }
        // find the plane's indicies 

        extract.setInputCloud(cloud_from_kinect.makeShared());
        extract.setIndices(inliers);

        // remove the plane to its own cloud
        extract.setNegative(false);
        pcl::PointCloud<pcl::PointXYZ> plane_cloud;
        extract.filter(plane_cloud);
        // add plane to vector
        planes.push_back(plane_cloud);
        Eigen::Vector4f plane_centroid;
        pcl::compute3DCentroid(plane_cloud, plane_centroid);
        plane_centroids.push_back(plane_centroid);
        
        // remove the plane's  indicies
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_from_kinect = *cloud_f;  
        //std::cout << "in plane loop" << std::endl;   
    }

    double minDistSquared = std::numeric_limits<float>::max();
    pcl::PointCloud<pcl::PointXYZ> nearestPlane;
    // find closest plane to sensor
    for (int i = 0; i < planes.size(); i++)
    {
        Eigen::Vector4f current_centroid = plane_centroids[i];
        double distSquared = current_centroid[0]*current_centroid[0] + current_centroid[1]*current_centroid[1] + current_centroid[2]*current_centroid[2];
        if (distSquared < minDistSquared)
        {
            minDistSquared = distSquared;
            nearestPlane = planes[i];
        }
    }



    // find equation of this plane
    //Eigen::Vector4f plane_parameters;
    //float curvature;
    //pcl::computePointNormal(nearestPlane, plane_parameters, curvature);
    //std::cout << "plane parameters are (" << plane_parameters[0] << "," << plane_parameters[1]<<","<<plane_parameters[2]<<","<<plane_parameters[3]<<")" << std::endl;

    

    pcl::search::KdTree<pcl::PointXYZ>::Ptr plane_tree (new pcl::search::KdTree<pcl::PointXYZ>);
    plane_tree->setInputCloud(nearestPlane.makeShared());

    std::vector<pcl::PointIndices> plane_cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> pec;
    pec.setClusterTolerance(.05);
    pec.setMinClusterSize(5000);
    pec.setMaxClusterSize(1000000);
    pec.setSearchMethod(plane_tree);
    pec.setInputCloud(nearestPlane.makeShared());
    pec.extract(plane_cluster_indices);
    std::cout << "Got plane cluster indices, got " << plane_cluster_indices.size() << " clusters " << std::endl;   

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    for (std::vector<pcl::PointIndices>::const_iterator it = plane_cluster_indices.begin (); it != plane_cluster_indices.end (); ++it)
    {
        
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            plane_cloud_cluster->points.push_back (nearestPlane.points[*pit]); //*

    }

    nearestPlane.points = plane_cloud_cluster->points;
    nearestPlane.width = plane_cloud_cluster->points.size ();
    nearestPlane.height = 1;
    nearestPlane.is_dense = true;

    




    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_from_kinect.makeShared());

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(.05);
    ec.setMinClusterSize(80);
    ec.setMaxClusterSize(500);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_from_kinect.makeShared());
    ec.extract(cluster_indices);
    std::cout << "Got cluster indices, got " << cluster_indices.size() << " clusters " << std::endl;   

    // will contain all clustered points
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    // contains this frame's centroids
    std::vector<Eigen::Vector4f> centroids;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        {
            cloud_cluster->points.push_back (cloud_from_kinect.points[*pit]);
            tmp_cloud_cluster->points.push_back (cloud_from_kinect.points[*pit]);
        } 
        tmp_cloud_cluster->width = tmp_cloud_cluster->points.size();
        tmp_cloud_cluster->height = 1;
        tmp_cloud_cluster->is_dense = true;
        Eigen::Vector4f tmp_centroid;
        pcl::compute3DCentroid(*tmp_cloud_cluster,tmp_centroid);
        centroids.push_back(tmp_centroid);

    } 




    std::vector<int> ball_candidate_indices;
    if (last_frame_centroids.size() != 0) // last frame had clusters
    {
        //std::cout << "non empty" << std::endl;
        for (int i = 0; i < centroids.size(); i++)
        {   
            bool candidate = true;
            Eigen::Vector4f curCent = centroids[i];
            Eigen::Vector3f cent;
            cent << curCent[0],curCent[1],curCent[2];
            for (int j = 0; j < last_frame_centroids.size(); j++)
            {
                Eigen::Vector4f curLastCent;
                curLastCent = last_frame_centroids[j];
                double dx,dy,dz,dist;
                dx = curCent[0] - curLastCent[0];
                dy = curCent[1] - curLastCent[1];
                dz = curCent[2] - curLastCent[2];
                dist = dx*dx + dy*dy + dz*dz;
                if (dist < THRESH )//|| !in_rectangle(p1,p2,p3,p4,curCent))
                    candidate = false;

            }
            if (candidate && in_rectangle(p1,p2,p3,p4,cent))
                ball_candidate_indices.push_back(i);
        }


    }
    else
    {
        std::cout << "empty" << std::endl;
    }

    // will contain all candidate cluster points
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < ball_candidate_indices.size(); i++)
    {
        int index = ball_candidate_indices[i];
        pcl::PointIndices candidate_cluster_indices = cluster_indices[index];

        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_candidate_cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = candidate_cluster_indices.indices.begin(); pit != candidate_cluster_indices.indices.end(); pit++)
        {
            candidate_cloud_cluster->points.push_back (cloud_from_kinect.points[*pit]);
        } 

    } 














    std::cout << "found " << ball_candidate_indices.size() << " ball candidates " << std::endl;
    
    pcl::PointCloud<pcl::PointXYZ> cloud_from_kinect_orig = cloud_from_kinect;  
    cloud_from_kinect.width = candidate_cloud_cluster->points.size();
    cloud_from_kinect.height = 1;
    cloud_from_kinect.is_dense = true;
    cloud_from_kinect.points = candidate_cloud_cluster->points;     
    

    /*
    // cloud with outliers removed
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_from_kinect.makeShared());
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(cloud_filtered);
    
    //convert cloud_from_kinect to a sensor_msg
    sensor_msgs::PointCloud2 cloud_from_kinect_sensor_msg,cloud_from_kinect_orig_sensor_msg;
    pcl::toROSMsg(cloud_from_kinect, cloud_from_kinect_sensor_msg);
    pcl::toROSMsg(nearestPlane,cloud_from_kinect_orig_sensor_msg);
    // publish filtered_cloud
    filtered_cloud_pub.publish(cloud_from_kinect_sensor_msg);
    difference_cloud_pub.publish(cloud_from_kinect_orig_sensor_msg);
    
    // update last frame centroids
    last_frame_centroids = centroids;
    //geometry_msgs::Point32 ballPosition;
    //ball_pub.publish(ballPosition);
    std::cout << std::endl;
    */

    return;
}




int main(int argc, char **argv)
{   
    ros::init(argc, argv, "ball_detector");

    ros::NodeHandle n;
    std::cout << "started" << std::endl;
    ros::Duration(5.0).sleep();
    ros::ServiceClient client = n.serviceClient<pingpong::TableCoordinates>("KinectToTableSpaceTransform");
    pingpong::TableCoordinates srv;

    if (client.call(srv))
    {   std::cout << "Successfully recieved coordinate transform!!!" << std::endl;
        origin << srv.response.origin.x,srv.response.origin.y,srv.response.origin.z;
        x_axis << srv.response.x_axis.x,srv.response.x_axis.y,srv.response.x_axis.z;
        y_axis << srv.response.y_axis.x,srv.response.y_axis.y,srv.response.y_axis.z;
        z_axis << srv.response.z_axis.x,srv.response.z_axis.y,srv.response.z_axis.z;

        p1 << 0,0,0;
        p2 << TABLE_LENGTH,0,0;
        p3 << TABLE_LENGTH,TABLE_WIDTH,0;
        p4 << 0,TABLE_WIDTH,0;
        p1 = TransformFromTableCoordinates(p1);
        p2 = TransformFromTableCoordinates(p2);
        p3 = TransformFromTableCoordinates(p3);
        p4 = TransformFromTableCoordinates(p4);

        table_plane_equation << z_axis(0),z_axis(1),z_axis(2),-1*(z_axis(0)*origin(0) + z_axis(1)*origin(1) + z_axis(2)*origin(2));
    }
    else
    {   std::cout <<"Failed to get transform. Exiting..." << std::endl;
        return 0;
    }

     // Initialilze ball position publisher and pointcloud subscriber
    ball_pub = n.advertise<geometry_msgs::Point32>("kinect_ping_pong_ball", 1000);
    filtered_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("ball_candidates", 1000);
    difference_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("nearest_plane", 1000);
    
    ros::Subscriber kinect_sub = n.subscribe("camera/depth_registered/points", 1000, pointCloudCallback);
    

    ros::spin();
    
    return 0;
}


