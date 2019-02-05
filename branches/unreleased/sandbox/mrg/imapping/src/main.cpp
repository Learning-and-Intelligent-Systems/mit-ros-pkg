/*
 * main.cpp
 *
 *  Created on: Dec 2, 2010
 *      Author: hordurj
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

class Mapping
{
public:
    Mapping()
    {

    }

    void update()
    {
        ros::Time t;
        t = ros::Time::now();

        geometry_msgs::TransformStamped map_trans;
        map_trans.header.stamp = t;
        map_trans.header.frame_id = "map";
        map_trans.child_frame_id = "base_link";

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double th = 0.0;

        geometry_msgs::Quaternion map_quat = tf::createQuaternionMsgFromYaw(th);

        map_trans.transform.translation.x = x;
        map_trans.transform.translation.y = y;
        map_trans.transform.translation.z = z;
        map_trans.transform.rotation = map_quat;

        map_broadcaster_.sendTransform(map_trans);
    }

    void on_lidar_data (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
        ROS_INFO("Received laser scan");
    }

private:
    void send_map()
    {

    }

private:
    tf::TransformBroadcaster map_broadcaster_;
};


/**
 * Start up basic map server
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "imapping");

    ros::NodeHandle nh;
    Mapping map;

    ros::Subscriber scan_sub = nh.subscribe("base_scan", 100, &Mapping::on_lidar_data, &map);

    // ros::Publisher pub = nh.advertise<nav_msgs::

    ros::Rate r(1);

    while(ros::ok())
    {
        ROS_INFO("Spin Once");

        ros::spinOnce();

        map.update();

        r.sleep();
    }
}
