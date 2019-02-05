/*
 * cloud_to_mesh.cpp
 *
 *  Created on: Nov 3, 2010
 *      Author: garratt
 */

#include "cloud_to_mesh.h"
namespace mesher{

//generated with:
//cat chair.xml | sed "s/\"/\\\\\"/g" | sed "s/^/outf<<\"/" | sed "s/$/\"<<std::endl;/" | sed "s/chair/\"<<params.object_name<<\"/"
void wrapWRL(std::string wrlfile, std::string xmlfile,MeshParams params){
   std::ofstream outf;
   outf.open(xmlfile.c_str(),std::ios::out);
   outf<<"<?xml version=\"1.0\" encoding=\"utf-8\"?>"<<std::endl;
   outf<<"<KinBody name=\""<<params.object_name<<"\">"<<std::endl;
   outf<<"  <Body name=\"mainbody\" type=\"dynamic\">"<<std::endl;
   outf<<"    <Translation>0 0 0</Translation>"<<std::endl;
   outf<<"    <RotationAxis>1 0 0 90</RotationAxis>"<<std::endl;
   outf<<"    <Geom type=\"trimesh\">"<<std::endl;
   outf<<"      <Data>"<<wrlfile<<" 1.00</Data>"<<std::endl;
   outf<<"      <Render>"<<wrlfile<<"  1.00</Render> "<<std::endl;
   outf<<"    </Geom>"<<std::endl;
   outf<<"  </Body>"<<std::endl;
   outf<<"</KinBody>"<<std::endl;
   outf.close();
}


void wrapSTL(std::string stlfile, std::string urdffile,MeshParams params){
   std::ofstream outf;
   outf.open(urdffile.c_str(),std::ios::out);
   outf<<"<robot name=\""<<params.object_name<<"_model\">"<<std::endl;
   outf<<"  <link name=\""<<params.object_name<<"_body\">"<<std::endl;
   outf<<"    <inertial>"<<std::endl;
   outf<<"      <mass value=\"0.1\" />"<<std::endl;
   outf<<"      <origin xyz=\""<<params.origin[0]<<" "<<params.origin[1]<<" "<<params.origin[2]<<"\" />"<<std::endl;
   outf<<"      <inertia  ixx=\"1.0\" ixy=\"1.0\"  ixz=\"1.0\"  iyy=\"1.0\"  iyz=\"1.0\"  izz=\"1.0\" />"<<std::endl;
   outf<<"    </inertial>"<<std::endl;
   outf<<"    <visual>"<<std::endl;
   outf<<"      <!-- visual origin is defined w.r.t. link local coordinate system -->"<<std::endl;
   outf<<"      <origin xyz=\""<<params.origin[0]<<" "<<params.origin[1]<<" "<<params.origin[2]<<"\" rpy=\"0 0 0\" />"<<std::endl;
   outf<<"      <geometry>"<<std::endl;
   outf<<"        <mesh filename=\""<<stlfile<<"\"/>"<<std::endl;
   outf<<"      </geometry>"<<std::endl;
   outf<<"    </visual>"<<std::endl;
   outf<<"    <collision>"<<std::endl;
   outf<<"      <!-- collision origin is defined w.r.t. link local coordinate system -->"<<std::endl;
   outf<<"      <origin xyz=\""<<params.origin[0]<<" "<<params.origin[1]<<" "<<params.origin[2]<<"\" rpy=\"0 0 0\" />"<<std::endl;
   outf<<"      <geometry>"<<std::endl;
   outf<<"        <mesh filename=\""<<stlfile<<"\"/>"<<std::endl;
   outf<<"      </geometry>"<<std::endl;
   outf<<"    </collision>"<<std::endl;
   outf<<"  </link>"<<std::endl;
   outf<<"  <gazebo reference=\""<<params.object_name<<"_body\">"<<std::endl;
   outf<<"    <material>"<<params.texture<<"</material>"<<std::endl;
   outf<<"    <turnGravityOff>true</turnGravityOff>"<<std::endl;
   outf<<"  </gazebo>"<<std::endl;
   outf<<"</robot>"<<std::endl;
   outf.close();
}



void createOpenraveLauncher(std::string xmlfile,std::string pyfile){
   std::ofstream outf;
   outf.open(pyfile.c_str(),std::ios::out);
   outf<<"#!/usr/bin/env python"<<std::endl;
   outf<<"from openravepy import *"<<std::endl;
   outf<<"env = Environment()"<<std::endl;
   outf<<"env.SetViewer('qtcoin')"<<std::endl;
   outf<<"env.Load('"<<xmlfile<<"')"<<std::endl;
   outf.close();
}




void cloud2Mesh(pcl::PointCloud<pcl::PointXYZINormal> &cloud, std::string filename, MeshParams params){
   std::vector<std::vector<float> > pts(cloud.points.size());
  for(uint i=0;i<pts.size();i++){
     pts[i].resize(6);
     pts[i][0]=cloud.points[i].x;
     pts[i][1]=cloud.points[i].y;
     pts[i][2]=cloud.points[i].z;
     pts[i][3]=cloud.points[i].normal[0];
     pts[i][4]=cloud.points[i].normal[1];
     pts[i][5]=cloud.points[i].normal[2];
  }
  std::string plyfile,stlfile,urdffile,wrlfile,xmlfile,pyfile;
  plyfile=stlfile=urdffile=wrlfile=xmlfile=pyfile=filename;
  plyfile+=".ply";
  stlfile+=".stl";
  urdffile+=".urdf";
  wrlfile+=".wrl";
  xmlfile+=".xml";
  pyfile+="_launcher.py";


  PoissonReconstruction(pts , plyfile,params.Depth,params.SolverDivide, params.IsoDivide,
        params.KernelDepth, params.Refine, params.SamplesPerNode,
        params.Verbose, params.NoResetSamples, params.noClipTree, params.Confidence );

  ply2stl(plyfile.c_str(), stlfile.c_str());
  ply2wrl(plyfile.c_str(), wrlfile.c_str());

  wrapSTL(stlfile,urdffile,params);
  wrapWRL(wrlfile,xmlfile,params);
  createOpenraveLauncher(xmlfile,pyfile);//create a launch file for openrave
}

}
