#!/bin/bash

bag="scans.bag"
points="scan_points"

if (( $# > 0 )) ; then
  bag=$1
fi
if (( $# > 1 )) ; then
  points=$2
fi

# extract pcd files
rosrun cardboard bag_to_pcds.sh $bag $points


# extract table center
table=$(rostopic echo -b scans.bag /table_polygons/polygons -p -n 1 | sed -n '2 s/,/\t/p' | cut -f2)

x1=$(echo $table | cut -f1 -d',')
y1=$(echo $table | cut -f2 -d',')
z1=$(echo $table | cut -f3 -d',')
x2=$(echo $table | cut -f4 -d',')
y2=$(echo $table | cut -f5 -d',')
z2=$(echo $table | cut -f6 -d',')
x3=$(echo $table | cut -f7 -d',')
y3=$(echo $table | cut -f8 -d',')
z3=$(echo $table | cut -f9 -d',')
x4=$(echo $table | cut -f10 -d',')
y4=$(echo $table | cut -f11 -d',')
z4=$(echo $table | cut -f12 -d',')

mx=$(echo "scale=4; ($x1 + $x2 + $x3 + $x4)/4" | bc)
my=$(echo "scale=4; ($y1 + $y2 + $y3 + $y4)/4" | bc)
mz=$(echo "scale=4; ($z1 + $z2 + $z3 + $z4)/4" | bc)

echo "origin = [ $mx $my $mz ]" > table_pos.m
