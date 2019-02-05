sensor_msgs/PointCloud2 cloud
geometry_msgs/TransformStamped laser_transform
geometry_msgs/TransformStamped base_laser_transform #the transform for the hokuyo base laser scanner
geometry_msgs/TransformStamped base_transform
object_survey/CameraScan[] camera_scans
geometry_msgs/Pose object_pose
sensor_msgs/PointCloud2[] base_laser_clouds #from the hokuyo base laser scanner
sensor_msgs/LaserScan[] base_laser_scans #from the hokuyo base laser scanner