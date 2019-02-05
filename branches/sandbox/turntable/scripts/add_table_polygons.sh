#!/bin/bash

input="scans.bag"
output="scans-out.bag"

if (( $# > 0 )) ; then
  input=$1
fi
if (( $# > 1 )) ; then
  output=$2
fi

rosrun furniture table_tracker &
pid1=$!
sleep 2
rosrun furniture table_init_publisher.py &
rosbag record tf /scan/narrow_stereo_textured/points /table_polygons -O $output &
pid2=$!
rosbag play $input -r 2 -d 2

sleep 2

#rosnode kill -a
#rosnode kill table_tracker table_init_publisher

sleep 2
echo ""
sleep 2
echo "done"
