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

