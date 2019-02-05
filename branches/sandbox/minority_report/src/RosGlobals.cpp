/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Garratt Gallagher
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
*   * Neither the name Garratt Gallagher nor the names of other
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
*********************************************************************/


/*
 * RosGlobals.cpp
 *
 *  Created on: Mar 30, 2009
 *      Author: garratt
 */

#include "RosGlobals.h"


///GG_ROSGLOBALS keeps all the global ros information for the whole system.
namespace GG_ROSGLOBALS{
	RosManager *rosmanager;
	bool rosmanagerstarted=false;
	RosManager * initRosManager(wxWindow* p){
		if(!rosmanagerstarted){
			rosmanager= new RosManager(p);
			rosmanagerstarted=true;

		}
		return rosmanager;
	}


}

/** \brief FILLTHISIN */
RosManager::RosManager(wxWindow* parent):wxPanel(parent,wxID_ANY,wxDefaultPosition,wxDefaultSize,wxBORDER_SIMPLE){
	startRos();
	initGui();
	online=false;
}


/** \brief FILLTHISIN */
void RosManager::initGui(){
	wxStaticText *lbltitle = new wxStaticText(this, wxID_ANY, wxT("ROS Interfaces"));
	wxCheckBox* cbonline = new wxCheckBox(this, -1, wxT("Online Mode"), wxPoint(20, 20));
	Connect(cbonline->GetId(), wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(RosManager::CheckBoxCB));
	wxBoxSizer *hbox_title = new wxBoxSizer(wxHORIZONTAL);
	hbox_title->Add(lbltitle,1,wxEXPAND | wxALIGN_CENTER | wxALL, 5 );
	vbox = new wxBoxSizer(wxVERTICAL);
	vbox->Add(hbox_title,0);
	vbox->Add(cbonline,0);
//	wxLine *line = wxLine(this, -1, wxLI_HORIZONTAL);
//	vbox->Add(line, 0, wxEXPAND|wxALL);

	for (std::list<GRosInterfaceDisplay*>::iterator rid=RosHooks.begin(); rid!= RosHooks.end(); ++rid) {
		vbox->Add(&(*(*rid)),0);
//		std::cout<<(*rid)<<",  ";
	}
//	std::cout<<std::endl;
	SetSizer(vbox);
}


/** \brief Adds an adversiser/subscriber to the system */
void RosManager::AddInterface(GRosInterface *gri){
	GRosInterfaceDisplay* temp = new GRosInterfaceDisplay(this,gri);
	RosHooks.push_back(temp);
	vbox->Add(temp,0);
	SetSizerAndFit(vbox);
}


/** \brief Get 2D pose of a frame id.  Made difficult in this framework because it has to communicate with TF */
gatmo_pose_t RosManager::getpose(ros::Time _t, std::string frame_id){
  static gatmo_pose_t out, odom_to_map;  //static so it will return the last known pose if it fails
  tf::Stamped<btTransform> global_pose_;
  tf::Stamped<tf::Pose> robotPose;
  robotPose.setIdentity();
  robotPose.frame_id_ = frame_id;
  robotPose.stamp_ = _t; // request most recent pose
  //robotPose.time = laserMsg.header.stamp.sec * (uint64_t)1000000000ULL +
  //        laserMsg.header.stamp.nsec; ///HACKE FIXME we should be able to get time somewhere else
  std::cout<<__PRETTY_FUNCTION__<<" getting pose for "<<frame_id<<" at time "<<_t<<std::endl;
  bool waiting=false;
  bool needs_odom_to_map=false;
  try{
//	  do{


	  try
	  {
//#warning using weird mast tf frame
		this->tf_->transformPose("/map", robotPose, global_pose_);
		//save the odom-to-map for later
		if(strcmp("/odom",frame_id.c_str()))
		odom_to_map= getpose(_t,"/odom");
//		waiting=false;
//		std::cout<<"getpose: got pose!  "<<std::endl;
	  }
	  catch(tf::ExtrapolationException& ex){
		  /*if(waiting)*/ std::cout<<"tf::ExtrapolationException  "<<ex.what()<<std::endl;

		  try{
			  this->tf_->transformPose("/odom", robotPose, global_pose_);
			  needs_odom_to_map=true;
		  }
		  catch(tf::ExtrapolationException& ex){
			  std::cout<<"tf::ExtrapolationException - Couldn't even get it without map  "<<ex.what()<<std::endl;


		  }

//		  else std::cout<<"waiting for transform ";
//		  waiting=true;
//		  usleep(1000);
	  }
//	  }while(waiting);
  }
  catch(tf::TransformException& ex)
  {
    puts("L: no global->local Tx yet");
      return out;
  }
  double yaw,pitch,roll; //fixme make cleaner namespace
  btMatrix3x3 mat =  global_pose_.getBasis();
//  btQuaternion bq= global_pose_.getRotation();
//  btVector3 bv =   global_pose_getOrigin();
//  std::cout<<bq.x()<<" "<<bq.y()<<" "<<bq.z()<<" "<<bq.w()<<std::endl;

  mat.getEulerZYX(yaw, pitch, roll);

  out.px=global_pose_.getOrigin().x();
  out.py= global_pose_.getOrigin().y();
  out.pa = yaw;
  if(needs_odom_to_map)
	  out=odom_to_map+out;
  std::cout<<" stamp= "<<_t<<"  ("<<out.px<<", "<<out.py<<", "<<out.pa<<")"<<std::endl;;

  return out;
}

/** \brief Get 3D pose of a frame id.  Made difficult in this framework because it has to communicate with TF */
gatmo_pose6D_t RosManager::getpose6D(ros::Time _t, std::string frame_id){
  static gatmo_pose6D_t out;  //static so it will return the last known pose if it fails
//  tf::Stamped<btTransform> global_pose_;
  tf::StampedTransform global_pose_;
  //static gatmo_pose6D_t odom_to_map;
  tf::Stamped<tf::Pose> robotPose;
  robotPose.setIdentity();
  robotPose.frame_id_ = frame_id;
  robotPose.stamp_ = _t; // request most recent pose
  //robotPose.time = laserMsg.header.stamp.sec * (uint64_t)1000000000ULL +
  //        laserMsg.header.stamp.nsec; ///HACKE FIXME we should be able to get time somewhere else
  std::cout<<__PRETTY_FUNCTION__<<" getting pose for "<<frame_id<<" at time "<<_t<<std::endl;
  bool waiting=false;
  try{
	//  do{

//	    tf::StampedTransform transform;
	    try{
	    	this->tf_->lookupTransform("/base_link", frame_id, _t, global_pose_);
	    }
	    catch (tf::TransformException ex){
	      ROS_ERROR("%s",ex.what());
	    }

//	  try
//	  {
//
//		this->tf_->transformPose("/base_link", robotPose, global_pose_);
////		odom_to_map=
//
//
////		waiting=false;
//	  }
//	  catch(tf::ExtrapolationException& ex){
//		  std::cout<<"tf::ExtrapolationException  "<<ex.what()<<std::endl;
////		  if(waiting) std::cout<<".";
////		  else std::cout<<"waiting for transform ";
////		  waiting=true;
////		  usleep(1000);
//
//		  //try getting just the laser to odom instead:
//
//
//
//
//	  }
//	  }while(waiting);
  }
  catch(tf::TransformException& ex)
  {
    puts("L: no global->local Tx yet");
      return out;
  }
  btQuaternion bq= global_pose_.getRotation();
  btVector3 bv =   global_pose_.getOrigin();
  out.orientation = gatmo_quaternion_t(bq.x(),bq.y(),bq.z(),bq.w());
  out.position = gatmo_point3D_t(bv.x(),bv.y(),bv.z());
  std::cout<<__PRETTY_FUNCTION__;
  out.print();
  return out;
}

/** \brief callback when checkbox is checked -  this starts the node */
void RosManager::CheckBoxCB(wxCommandEvent& event){
	if(event.IsChecked()){
		online=true;
		spinner->start();
		for (std::list<GRosInterfaceDisplay*>::iterator rid=RosHooks.begin(); rid!= RosHooks.end(); ++rid) {
			(*rid)->SetConnection();
		}
	}
	else{
		online=false;
		spinner->stop();
		for (std::list<GRosInterfaceDisplay*>::iterator rid=RosHooks.begin(); rid!= RosHooks.end(); ++rid) {
			(*rid)->ClearConnection();
		}
	}
}

/** \brief Starts an actual ros node */
void RosManager::startRos(){
  static bool started=false;
  if(!started){
//	  spinner.start();
	  started=true;
	  int argc=0;
	  char** argv=NULL;
	  ros::init(argc,argv,"GATMO");
	  // Create a ROS node
	  ros::NodeHandle n;
	  spinner = new ros::AsyncSpinner(3);
	  tf_ = new tf::TransformListener;
  }
}



GRosInterfaceDisplay::GRosInterfaceDisplay(wxWindow* _parent, GRosInterface* _rosinterface)
	:wxCheckBox(_parent,wxID_ANY,wxString::FromAscii(_rosinterface->name.c_str())){
	parent=_parent;
	rosinterface=_rosinterface;
	Connect(GetId(), wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler(GRosInterfaceDisplay::CheckBoxCB));


}

/** \brief update connection to reflect checkbox status */
void GRosInterfaceDisplay::SetConnection(){

	if(GetValue())
		rosinterface->Connect();
	else
		if(rosinterface->connected)
			rosinterface->Disconnect();
}

/** \brief update connection to disconnected */
void GRosInterfaceDisplay::ClearConnection(){
	if(rosinterface->connected)
	rosinterface->Disconnect();
}

/** \brief callback when checkbox is checked */
void GRosInterfaceDisplay::GRosInterfaceDisplay::CheckBoxCB(wxCommandEvent& event){
	if(((RosManager*)parent)->isOnline()) //if already online
		SetConnection();
}














