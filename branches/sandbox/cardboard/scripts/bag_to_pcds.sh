#!/bin/bash

if (($# < 2)) ; then
  echo "usage: bag_to_pcds.sh <bagfile> <point_cloud_topic> [rate] [target_frame]"
  exit
fi

bagfile=$1
points=$2

rate=.2
if (($# >= 3)) ; then
  rate=$3
fi

if (($# >= 4)) ; then
  target_frame=$4
  rosrun cardboard bag_to_pcds points2:=$points target_frame:=$target_frame &
else
  rosrun cardboard bag_to_pcds points2:=$points &
fi


rosbag play $bagfile -r $rate
sleep 5
rosnode kill bag_to_pcds
