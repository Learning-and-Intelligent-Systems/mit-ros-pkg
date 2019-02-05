geometry_msgs/PointStamped focus_point #where the head was pointing


#narrow stereo
# right
sensor_msgs/Image narrow_stereo_right_image #without projector
sensor_msgs/CameraInfo narrow_stereo_right_caminfo

# left
sensor_msgs/Image narrow_stereo_left_image #without projector
sensor_msgs/CameraInfo narrow_stereo_left_caminfo

# cloud
sensor_msgs/PointCloud2 narrow_stereo_cloud #with projector
geometry_msgs/TransformStamped narrow_stereo_transform



#wide stereo
# right
sensor_msgs/Image wide_stereo_right_image #without projector
sensor_msgs/CameraInfo wide_stereo_right_caminfo

# left
sensor_msgs/Image wide_stereo_left_image #without projector
sensor_msgs/CameraInfo wide_stereo_left_caminfo

# cloud
sensor_msgs/PointCloud2 wide_stereo_cloud #with projector
geometry_msgs/TransformStamped wide_stereo_transform


#prosilica
sensor_msgs/Image highdef_image #without projector
sensor_msgs/CameraInfo highdef_caminfo
geometry_msgs/TransformStamped highdef_transform
posedetection_msgs/Feature0D highdef_sift_features

#This is the difference between where the head was estimated to be, and where it actually was, relative to the base
geometry_msgs/Transform head_delta

#This is the difference between where the base was estimated to be, and where it actually was, relative to the world
#this transform is usually propagated up from the base when it runs through alignment
geometry_msgs/Transform base_delta