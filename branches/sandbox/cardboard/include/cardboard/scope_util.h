#ifndef CARDBOARD_SCOPE_UTIL_H_
#define CARDBOARD_SCOPE_UTIL_H_

#include <vector>
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/ChannelFloat32.h>

#include <cardboard/Scope.h>
#include <cardboard/util.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

#include <bingham/util.h>
#include <bingham/olf.h>

using namespace std;

void load_true_pose(string pose_file, simple_pose_t *true_pose);
void find_all_the_features(pcd_t &pcd_bg_full, pcd_t &pcd_objects_full, cardboard::Scope::Request &req);

#endif
