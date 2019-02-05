/*
 * segment_furniture.cpp
 *
 *  Created on: Sep 30, 2010
 *      Author: garratt
 */

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: filter_extract_indices.cpp 30899 2010-07-16 04:56:51Z rusu $
 *
 */

/**

\author Radu Bogdan Rusu

@b filter_extract_indices exemplifies how to run a Sample Consensus segmentation for planar models and extract the
inlier indices as a separate PointCloud.

**/

#include "pcl/ModelCoefficients.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "pcl/segmentation/extract_clusters.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/extract_indices.h"

#include "pcl/filters/passthrough.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/normal_3d_omp.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_ann.h"
#include "pcl/kdtree/organized_data.h"

  typedef pcl::PointWithViewpoint Point;
#include "pcl_helpers.h"

/* ---[ */
int
  main (int argc, char** argv)
{
  string infile="staticfront_downsampled2.pcd";

  if(argc>1){
	  infile=argv[1];
  }
  string saveprefix=infile;
  size_t found=saveprefix.rfind(".pcd");
  if (found!=string::npos)
	  saveprefix=infile.substr(0,found);



  pcl::PointCloud<Point> cloud_filtered, cloud_downsampled,cloud_nooutliers,cloud_p, cloud_no_floor,cloud_pplheight;

  readPCD(infile, cloud_filtered);

  ROS_INFO ("PointCloud before segmentation: %d data points.", cloud_filtered.width * cloud_filtered.height);
  removeOutliers(cloud_filtered,cloud_nooutliers);
  downSample(cloud_nooutliers,cloud_downsampled);
  segmentFloor(cloud_downsampled,cloud_no_floor);
  segmentPplHeight(cloud_no_floor,cloud_pplheight);

  //save off some clouds
  string nofloorname=saveprefix,pplheightname=saveprefix,downsamplename=saveprefix;
  nofloorname+="_cloud_no_floor.pcd";
  pplheightname+="_cloud_pplheight.pcd";
  downsamplename+="_cloud_ndownsampled.pcd";
  pcl::io::savePCDFileASCII (nofloorname, cloud_no_floor);
  pcl::io::savePCDFileASCII (pplheightname, cloud_pplheight);
  pcl::io::savePCDFileASCII (downsamplename, cloud_downsampled);


  std::vector<pcl::PointCloud<Point> > clusters;
  segfast(cloud_pplheight,clusters);


  char filename[200];
  for(int i=0;i<clusters.size(); i++){
	  if(clusters[i].points.size() > 0){
		  sprintf(filename,"%s_object%02d.pcd",saveprefix.c_str(),i);
		  pcl::io::savePCDFileASCII (filename, clusters[i]);
	  }
  }

  return (0);
}
/* ]--- */


