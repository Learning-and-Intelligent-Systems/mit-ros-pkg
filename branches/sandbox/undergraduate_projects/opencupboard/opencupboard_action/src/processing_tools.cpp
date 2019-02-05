#include "ros/ros.h"
#include "opencupboard_action/processing_tools.hpp"


ros::Publisher visPub;

int marker_id;


/**
 *\brief Converts the given point from the frame of the given pose.
 *
 *Uses conversion matrix to inverse transform a point when the tf is not 
 *publicly published(e.g. to convert a point from the frame of a table
 *to the frame the table is defined in, give the pose of the detected table)
 *@param pose desired pose to transform from
 *@param point originial point 
 *@param new_point converted point (by reference)
 */
void inverse_transform_point(geometry_msgs::Pose pose,
			     const geometry_msgs::Point32 &point,
			     geometry_msgs::Point32 &new_point)
{
  double x = pose.orientation.x, y = pose.orientation.y, z = pose.orientation.z, w = pose.orientation.w;
  double pose_x = pose.position.x, pose_y = pose.position.y, pose_z = pose.position.z;
  
  // Conversion matrix from table frame
  double m00 = (1.0-2.0*((y*y)+(z*z)));
  double m01 = (2.0*((x*y)-(w*z)));
  double m02 = (2.0*((x*z)+(w*y)));
  double m10 = (2.0*((x*y)+(w*z)));
  double m11 = (1.0-2.0*((x*x)+(z*z)));
  double m12 = (2.0*((y*z)-(w*x)));
  double m20 = (2.0*((x*z)-(w*y)));
  double m21 = (2.0*((y*z)+(w*x)));
  double m22 = (1.0-2.0*((x*x)+(y*y)));
  new_point.x = ((m00*point.x)+(m01*point.y)+(m02*point.z)) + pose_x;
  new_point.y = ((m10*point.x)+(m11*point.y)+(m12*point.z)) + pose_y;
  new_point.z = ((m20*point.x)+(m21*point.y)+(m22*point.z)) + pose_z;
}

/**
 *\brief Converts the given point into the frame of the given pose.
 *
 *Uses conversion matrix to transform a point when the tf is not 
 *publicly published(e.g. to convert a point into the frame of a table
 *from the frame the table is defined in, give the pose of the detected table)
 *@param pose desired pose to transform from
 *@param point originial point 
 *@param new_point converted point (by reference)

 */
void transform_point(geometry_msgs::Pose pose,
		     const geometry_msgs::Point32 &point,
		     geometry_msgs::Point32 &new_point) 
{
  double x = pose.orientation.x, y = pose.orientation.y, z = pose.orientation.z, w = pose.orientation.w;
  double pose_x = pose.position.x, pose_y = pose.position.y, pose_z = pose.position.z;
  
  // Conversion matrix to table frame
  double m00 = (1.0-2.0*((y*y)+(z*z)));
  double m01 = (2.0*((x*y)-(w*z)));
  double m02 = (2.0*((x*z)+(w*y)));
  double m10 = (2.0*((x*y)+(w*z)));
  double m11 = (1.0-2.0*((x*x)+(z*z)));
  double m12 = (2.0*((y*z)-(w*x)));
  double m20 = (2.0*((x*z)-(w*y)));
  double m21 = (2.0*((y*z)+(w*x)));
  double m22 = (1.0-2.0*((x*x)+(y*y)));
  double tx = point.x-pose_x;
  double ty = point.y-pose_y;
  double tz = point.z-pose_z;
  new_point.x = ((m00*tx)+(m10*ty)+(m20*tz));
  new_point.y = ((m01*tx)+(m11*ty)+(m21*tz));
  new_point.z = ((m02*tx)+(m12*ty)+(m22*tz));
}


void marker_init()
{
  ros::NodeHandle n;
  visPub = n.advertise<visualization_msgs::Marker>("/handle_marker",0);
  marker_id = 0;
}


/**
 *\brief publishes a marker in rviz
 *
 *Publishes a purple sphere marker in rviz at specified coordinates
 *@param pose pose of frame to publish the marker in
 *@param type type of marker. must be in all caps (visualization_msgs::Marker::SPHERE,etc)
 *@param x (should be in baselink)
 *@param y (should be in baselink)
 *@param z (should be in baselink)
 */
void make_marker(geometry_msgs::PoseStamped pose,
		 int type,
		 float x, float y, float z, float dim_x, float dim_y, float dim_z,
		 float color_r, float color_g, float color_b, 
		 float orien_x, float orien_y, float orien_z, float orien_w,
		 std::string ns)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = pose.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  // doesn't matter as long as its different for every marker
  marker.id = marker_id;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = orien_x;
  marker.pose.orientation.y = orien_y;
  marker.pose.orientation.z = orien_z;
  marker.pose.orientation.w = orien_w;
  marker.scale.x = dim_x;
  marker.scale.y = dim_y;
  marker.scale.z = dim_z;
  marker.color.r = color_r;
  marker.color.g = color_g;
  marker.color.b = color_b;
  marker.color.a = 0.5;
  //marker.lifetime = ros::Duration(30);
  
  visPub.publish(marker);
  marker_id++;
}
