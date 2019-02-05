#!/bin/bash

matlab_fdir=$(rospack find turntable)/matlab
matlab -nodesktop -r "addpath('${matlab_fdir}'); align_turntable_scans(); quit"
rosrun cardboard pcd_to_ply full_cloud.pcd full_cloud.ply

# optional
#meshlab full_cloud.ply
