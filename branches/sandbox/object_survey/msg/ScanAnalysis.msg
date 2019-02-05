#this message is for storing the analysis of a scan
#it is mostly used for debugging, to save partially analyzed data
#raw cloud, in PointXYZWithViewpoint form:
sensor_msgs/PointCloud2 cloud_vp
#cloud with normals:
sensor_msgs/PointCloud2 normal_cloud
#filtered cloud with normals:
sensor_msgs/PointCloud2 normal_cloud_filtered
# aligned cloud:
sensor_msgs/PointCloud2 normal_cloud_aligned

#an assembly of stereo clouds:
sensor_msgs/PointCloud2 stereo_cloud_aligned

object_survey/CameraScanAnalysis[] camera_scans

#original transforms:
geometry_msgs/TransformStamped laser_transform
geometry_msgs/TransformStamped base_transform

#aligned transform:
geometry_msgs/TransformStamped laser_aligned
geometry_msgs/TransformStamped base_aligned


#sift features determined to be "good"
posedetection_msgs/Feature0D good_features
#where the features sit in 3d space
geometry_msgs/Point[]        feature_points


geometry_msgs/Pose object_pose