/*
 * cloud_to_mesh.h
 *
 *  Created on: Nov 3, 2010
 *      Author: garratt
 */

#ifndef CLOUD_TO_MESH_H_
#define CLOUD_TO_MESH_H_

#include <fstream>
#include <stdlib.h>
#include <sstream>
#include <string>


#include "ros/ros.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "ply2stl.h"
#include "MultiGridOcLib.h"

namespace mesher{

struct MeshParams{
   int Depth;  //Running at depth d corresponds to solving on a 2^d x 2^d x 2^d voxel grid.

   int SolverDivide; //The depth at which a block Gauss-Seidel solver is used to solve the Laplacian.
   //Using this parameter helps reduce the memory overhead at the cost of a small increase in reconstruction time.

   int IsoDivide;    //This integer argument specifies the depth at which a block iso-surface extractor
          // should be used to extract the iso-surface. Using this parameter helps reduce the memory overhead
          // at the cost of a small increase in extraction time. (In practice, we have found that for reconstructions
          // of depth 9 or higher a subdivide depth of 7 or 8 can greatly reduce the memory usage.)

   int KernelDepth;
   int Refine;
   float SamplesPerNode; //This parameter specifies the minimum number of points that should fall within an octree node.
   bool Verbose;         //this may not do anything...
   bool NoResetSamples;
   bool noClipTree;
   bool Confidence;   // If this flag is enabled, the size of a sample's normals is used as a confidence value,
                      //  affecting the sample's constribution to the reconstruction process.

   //Params for creating a urdf file:
   //the name given to the object in the urdf model:
   std::string object_name;
   float origin[3];  //x y z of center of the object
   std::string texture;  //This string references a texture, such as: Gazebo/LightWood
   //I believe that any file in:
   //simulator_gazebo/gazebo/gazebo/share/gazebo/Media/materials/textures/
   //can be referenced.


   //default values:
   MeshParams(){
      Depth=8; SolverDivide=8; IsoDivide=8.0;
      KernelDepth=0; Refine=3; SamplesPerNode=1.0;
      Verbose=false; NoResetSamples =false; noClipTree=false; Confidence=false;
      //Params for creating a urdf file:
      origin[0]=0.0;origin[1]=0.0;origin[2]=0.0;
      texture="Gazebo/LightWood";
      object_name="UnNamed";
   }


};


void wrapSTL(std::string stlfile, std::string urdffile,MeshParams params);
void cloud2Mesh(pcl::PointCloud<pcl::PointXYZINormal> &cloud, std::string filename, MeshParams params);

}

#endif /* CLOUD_TO_MESH_H_ */
