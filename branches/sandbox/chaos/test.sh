roscore &

sleep 1

rosrun point_cloud_converter point_cloud_converter points_in:=/narrow_stereo_textured/points_throttle points2_out:=/pointcloud2 &

sleep 1

rosrun furniture table_detector points:=narrow_stereo_textured/points_throttle &

sleep 1

cd ~/cardboard_scenes/
while ((1)) ; do rosbag play -r .1 cardboard1.bag ; done
cd -

