#ifndef __PROCESSING_TOOLS_HPP__
#define __PROCESSING_TOOLS_HPP__


#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <string>


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
			     geometry_msgs::Point32 &new_point);


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
		     geometry_msgs::Point32 &new_point);

void marker_init();


void make_marker(geometry_msgs::PoseStamped pose,
		 int type,
		 float x, float y, float z, 
		 float dim_x=0.1, float dim_y=0.1, float dim_z=0.1,
		 float color_r=1, float color_g=0, float color_b=1,
		 float orien_x=0.0, float orien_y=0.0, float orien_z=0.0, float orien_w=1.0,
		 std::string ns="mysphere");



#endif
