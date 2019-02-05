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
 * RosGlobals.h
 *
 *  Created on: Mar 28, 2009
 *      Author: garratt
 */

#ifndef ROSGLOBALS_H_
#define ROSGLOBALS_H_
#undef Success
#include "gatmounits.h"
#include <ros/master.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <string.h>
#include "wx/wx.h"
#include <wx/listbox.h>

typedef enum {ADVERTISE, SUBSCRIBE} nodetype;

class GRosInterface;

/** \brief @b GRosInterfaceDisplay represents one advertiser/subscriber.  Through the checkbox interface, the ros connection can be controlled through a gui.
 */
class GRosInterfaceDisplay: public wxCheckBox{
public:
	GRosInterfaceDisplay(wxWindow* _parent, GRosInterface* _rosinterface);
	void SetConnection();
	void ClearConnection();
	wxWindow* parent;
	GRosInterface* rosinterface;
private:

	void CheckBoxCB(wxCommandEvent& event);

};


/** \brief @b RosManager keeps track of all ros subscribers and advertisers in the program.  
 * \author Garratt Gallagher
 */
class RosManager: public wxPanel{
public:
	std::list<GRosInterfaceDisplay*> RosHooks;

	bool online;
	bool isOnline(){return online;}
	wxBoxSizer *vbox;
	ros::AsyncSpinner *spinner;
	tf::TransformListener* tf_;
	gatmo_pose_t getpose(ros::Time _t, std::string frame_id);
	gatmo_pose6D_t getpose6D(ros::Time _t, std::string frame_id);
	RosManager(wxWindow* parent);
	void AddInterface(GRosInterface *gri);

private:
	void CheckBoxCB(wxCommandEvent& event);
	void startRos();
	void initGui();

};












namespace GG_ROSGLOBALS{
	extern RosManager *rosmanager;  //TODO: make sure nothing is mucked with unless this is initialized.
	extern bool rosmanagerstarted;
	RosManager *  initRosManager(wxWindow* p);

}





/** \brief @b AbstractGatmoAFunctor is an abstract class that has a subscribe function, so you can subscribe w/o knowing the topic type  
 * \author Garratt Gallagher
 */
/// 
class AbstractGatmoAFunctor
{
public:
//  virtual void call() = 0;
  virtual bool subscribe(ros::NodeHandle *node,std::string _topic) = 0;
  virtual bool advertise(ros::NodeHandle *node,std::string _topic) = 0;
  //should never have to use this one:
//  virtual void publish(ros::NodeHandle *node,std::string _topic, void* _msg) = 0;
  virtual void publish(std::string _topic, void* _msg) = 0;
};


/** \brief @b GatmoAFFunctor subscribes to a ros topic, then converts the message to GATMO format.  This functor is used when a class function is called as a message callback.
 */
template <class  rosmsgtype,class msgtype,class cbclass>
class GatmoAFFunctor: public AbstractGatmoAFunctor{
protected:
	  void (cbclass::*mp_)(msgtype&);
	  cbclass* obj_;
	  void (*converttogfunc)(msgtype&,const rosmsgtype&);//convert to gatmo function
	  void (*converttorfunc)(const msgtype&,rosmsgtype&);//convert to ros function
	  ros::Publisher publisher;
	  ros::Subscriber sub;
public:
//	rosmsgtype rm;  //get ros messages in here

	GatmoAFFunctor(cbclass *_obj, void (cbclass::*_mp)(msgtype &),void (*_cf)(msgtype&,const rosmsgtype&)): mp_(_mp), obj_(_obj),converttogfunc(_cf){ }
	GatmoAFFunctor(void (*_cf)(const msgtype&,rosmsgtype&)):converttorfunc(_cf){ } //for publishing

	virtual void call(const boost::shared_ptr<rosmsgtype const> &rm){
		msgtype m; 		// gatmo message type.  put in this function because we don't want to randomly construct classes that we send.
		converttogfunc(m,*rm);
	    if (mp_ && obj_)
	      (*obj_.*mp_)(m);
	}
	virtual bool subscribe(ros::NodeHandle *node,std::string _topic){
		sub = node->subscribe(_topic,10,&(GatmoAFFunctor<rosmsgtype,msgtype,cbclass>::call),this);
		return true;
	}
	virtual bool advertise(ros::NodeHandle *node,std::string _topic){
		publisher= node->advertise<rosmsgtype>(_topic, 10);
		return true;
	}
//	virtual void publish(ros::NodeHandle *node,std::string _topic, void* _msg){
//		rosmsgtype rm;
//		converttorfunc(*((msgtype*)_msg),rm);
//		return publisher.publish(_topic,rm);
//	}
	virtual void publish(std::string _topic, void* _msg){
		rosmsgtype rm;
		converttorfunc(*((msgtype*)_msg),rm);
		return publisher.publish(rm);
	}

};


/** \brief @b GatmoAFFunctor publishes to a ros topic, after converting it from a GATMO type.
 */
template <class  rosmsgtype,class msgtype,class cbclass>
class GatmoPublishingFunctor: public AbstractGatmoAFunctor{
protected:
	  void (cbclass::*mp_)(msgtype&);
	  cbclass* obj_;
	  void (*converttogfunc)(msgtype&,const rosmsgtype&);
	  void (*converttorfunc)(const msgtype&,rosmsgtype&);
	  ros::Publisher publisher;
public:
	rosmsgtype rm;  //get ros messages in here

	GatmoPublishingFunctor(cbclass *_obj, void (cbclass::*_mp)(msgtype &),void (*_cf)(msgtype&,const rosmsgtype&)): mp_(_mp), obj_(_obj),converttogfunc(_cf){ }
	GatmoPublishingFunctor(void (*_cf)(const msgtype&,rosmsgtype&)):converttorfunc(_cf){ } //for publishing
	virtual void call(){} //call isn't called, because we never subscribe
	virtual bool subscribe(ros::NodeHandle *node,std::string _topic){
//		return node->subscribe(_topic,rm,&GatmoAFFunctor<rosmsgtype,msgtype,cbclass>::call,this,10);
		return true;
	}
	virtual bool advertise(ros::NodeHandle *node,std::string _topic){
		publisher = node->advertise<rosmsgtype>(_topic, 10);
		return true;
	}
//	virtual void publish(ros::NodeHandle *node,std::string _topic, void* _msg){
//		rosmsgtype rm;
//		converttorfunc(*((msgtype*)_msg),rm);
//		return publisher.publish(_topic,rm);
//	}
	virtual void publish(std::string _topic, void* _msg){
		rosmsgtype rm;
		converttorfunc(*((msgtype*)_msg),rm);
		publisher.publish(rm);
	}

};

/** \brief @b GatmoAFFunctor publishes to a ros topic, after converting it from a GATMO type
 */
template <class msgtype,class cbclass>
class GatmoAFunctor: public AbstractGatmoAFunctor{
protected:
	  void (cbclass::*mp_)(msgtype&);
	  cbclass* obj_;
public:
	GatmoAFunctor(cbclass *_obj, void (cbclass::*_mp)(msgtype &)): mp_(_mp), obj_(_obj){ }
	virtual void call(msgtype m){
//		std::cout<<" GatmoAFunctor:: call "<<std::endl;

	    if (mp_)
	      (*obj_.*mp_)(m);
	}

	virtual bool subscribe(ros::NodeHandle *node,std::string _topic){
		std::cout<<" GatmoAFunctor:: subscribe "<<std::endl;

		return node->subscribe(_topic,10,&GatmoAFunctor<msgtype,cbclass>::call,this);
	}
	virtual bool advertise(ros::NodeHandle *node,std::string _topic){
		std::cout<<" GatmoAFunctor:: advertise "<<std::endl;
		return node->advertise<msgtype>(_topic, 10);
	}
	virtual void publish(ros::NodeHandle *node,std::string _topic, void* _msg){}
	virtual void publish(std::string _topic, void* _msg){}

};


/** \brief @b GRosInterface is the GATMO interface to ROS */
class GRosInterface{
private:
	nodetype interaction;
protected:
    virtual bool Advertise(){
   	 return (gf && gf->advertise(listener_node,topic));
    }
    virtual bool Subscribe(){
   	 return (gf && gf->subscribe(listener_node,topic));
    }
	void init(std::string _name,std::string _topic,nodetype _interaction){
  	  interaction=_interaction;
  	  topic = _topic;\
  	  name=_name;
      connected =false;
      getting_aligned_scans=false;
      msgcounter=0;
      queuelength=10;
//      std::cout<<__PRETTY_FUNCTION__<<": GG_ROSGLOBALS::rosmanager = "<<GG_ROSGLOBALS::rosmanager<<std::endl;
      if(GG_ROSGLOBALS::rosmanagerstarted)
    	  GG_ROSGLOBALS::rosmanager->AddInterface(this);
	  return;
     }

public:
	template <class msgtype,class cbclass>
	GRosInterface(std::string _name,std::string _topic,nodetype _interaction, void(cbclass::*fp)(msgtype &),cbclass *_obj ){
		gf = new GatmoAFunctor<msgtype,cbclass>(_obj,fp);
		init(_name,_topic,_interaction);
	}

	/** \brief This constructor decides what the publish function is based on the conversion function */
	template <class rosmsgtype,class msgtype,class cbclass>
	GRosInterface(std::string _name,std::string _topic,nodetype _interaction, void(cbclass::*fp)(msgtype &),cbclass *_obj,void (*_cf)(msgtype&,const rosmsgtype&) ){
		gf = new GatmoAFFunctor<rosmsgtype,msgtype,cbclass>(_obj,fp,_cf);
		init(_name,_topic,_interaction);
	}

	//Publishing type (really don't need interface flag...)
	///An inferred interface:  decides what the publish function is based on the conversion function
	template <class rosmsgtype,class msgtype>
	GRosInterface(std::string _name,std::string _topic,nodetype _interaction, void (*_cf)(const msgtype&,rosmsgtype&) ){
		gf = new GatmoPublishingFunctor<rosmsgtype,msgtype, GRosInterface>(_cf);
		init(_name,_topic,_interaction);
	}

	template <class rosmsgtype>
	GRosInterface(std::string _name,std::string _topic,nodetype _interaction){
		gf = new GatmoAFFunctor<rosmsgtype,int, GRosInterface>();
		init(_name,_topic,_interaction);
	}


	template <class msgtype>
	void Publish(msgtype _msg){
   	  if(connected)
   			 gf->publish(topic,(void*)&_msg);
//		 std::cout<<"called publish from "<<topic<<std::endl;
	}

	void Publish(const ros::Message& _msg){
   	  if(connected)
   			gf->publish(topic,(void*)&_msg);
//		 std::cout<<"called publish from "<<topic<<std::endl;
	}


	AbstractGatmoAFunctor * gf;
	int queuelength;
	std::string name;
	std::string topic;
     bool connected;
     bool getting_aligned_scans;
     ros::NodeHandle *listener_node;
     int msgcounter;

     void msgCB(){
    	 std::cout<<"got msg"<<std::endl;
    	 msgcounter++;
     }

     /** \brief connect to ros network */
     void Connect(){
    	 if(connected) return;
    	 listener_node = new ros::NodeHandle;
    	if(interaction == SUBSCRIBE){
    	 std::cout<<"subscribing to "<<topic<<std::endl;
    	 if(Subscribe())
    		 connected=true;
    	}
    	if(interaction == ADVERTISE){
      	  std::cout<<"Advertising to "<<topic<<std::endl;
     	 if(Advertise())
 		 connected=true;
    	}
     }
     
     /** \brief disconnect doesn't actually work */
     void Disconnect()
     {
    	if(!connected){
    		std::cerr<<"GRosInterface::Disconnect():  trying to Disconnect, but not connected!"<<std::endl;
    		return;
    	}

    	if(interaction == SUBSCRIBE){
       	  std::cout<<"unsubscribing from "<<topic<<" doesn't actually work! "<<std::endl;
//     	  if(listener_node->unsubscribe(topic))
     		  connected=false;
	}
	if(interaction == ADVERTISE){
	  std::cout<<"unadvertising from "<<topic<<" doesn't actually work! "<<std::endl;
//		  if(listener_node->unadvertise(topic))
	  connected=false;
	}

     }

};















#endif /* ROSGLOBALS_H_ */
