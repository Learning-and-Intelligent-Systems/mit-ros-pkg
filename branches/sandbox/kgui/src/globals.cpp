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


#include "globals.h"


namespace GG_MAIN{
//	gg_mouse_selection_mode selection_mode = OBJECT;
	MyApp *pglobalapp;
//	float datashiftx=0.0;
//	float datashifty=0.0;
//	float datashifta=0.0;
////	list<scantime> timematch;
	GLUquadricObj *quadratic;
	bool runmode=false;
	GatmoUI gatmoui;
	 bool trainingmode=false;
//	boost::mutex dataDisplayMutex;
//	pthread_mutex_t dataDisplayMutex = PTHREAD_MUTEX_INITIALIZER;
	gatmo_mutex_t dataDisplayMutex("dataDisplayMutex");
//	GMouse gmouse;
//    double inittime =0.0;
//   	double lasttime;


}
int gatmo_demo_t::lastid;


//float GatmoUI::getscale(){
//	return 1.0/(view_offset.weight+1.0);
//}

void GatmoUI::updateVO(){
	static double lasttime=-1;
	double timediff=getTime()-lasttime;
	if(lasttime<0) timediff=0;
	lasttime=getTime();
//	std::cout<<__PRETTY_FUNCTION__<<" timediff: "<<timediff<<"  ";
//	(view_vel*timediff).print();
	view_pose.orientation=(view_vel*timediff).orientation*view_pose.orientation;
}


void GatmoObjectDisplay::AddtoDisplayList(){
	glock(GG_MAIN::gatmoui.gdisplayListMutex);
		GG_MAIN::gatmoui.gdisplayList.push_back((GatmoObjectDisplay *)this);
//		std::cout<<__PRETTY_FUNCTION__<<" GG_MAIN::gatmoui.gdisplayList.back().object_display_id = "<<GG_MAIN::gatmoui.gdisplayList.back()->object_display_id;
//		std::cout<<"   for "<<this->name<<std::endl;
	gunlock(GG_MAIN::gatmoui.gdisplayListMutex);
}

void GatmoObjectDisplay::RemoveFromDisplayList(){
//	std::cout<<__PRETTY_FUNCTION__<<": object_display_id = "<<object_display_id<<"  this = "<<this<<std::endl;
	glock(GG_MAIN::gatmoui.gdisplayListMutex);
	for (std::list<GatmoObjectDisplay*>::iterator od=GG_MAIN::gatmoui.gdisplayList.begin(); od!= GG_MAIN::gatmoui.gdisplayList.end(); ++od){
//		int tempint = (*od)->object_display_id;
//		std::cout<<__PRETTY_FUNCTION__<<": (*od)->object_display_id = "<<(*od)->object_display_id<<std::endl;
		if((*od)->object_display_id==object_display_id){
			GG_MAIN::gatmoui.gdisplayList.erase(od);
			gunlock(GG_MAIN::gatmoui.gdisplayListMutex);
			return;
		}
	}
	std::cerr<<"GatmoObjectDisplay::RemoveFromDisplayList(): never removed from list"<<std::endl;
	gunlock(GG_MAIN::gatmoui.gdisplayListMutex);
}



void GatmoDisplayInterface::MenuSelect(wxCommandEvent& evt){
        std::cout<<"menu item checked  "<<evt.GetId()<<std::endl;
        for (std::list<gatmo_demo_t*>::iterator d=GG_MAIN::gatmoui.gdemos.begin(); d!= GG_MAIN::gatmoui.gdemos.end(); ++d) {
			if((*d)->id==evt.GetId()){
				if((*d)->mi->IsChecked())//was just enabled
					(*d)->Enable();
				else
					(*d)->Disable();
			break;
			}
		}
        evt.Skip(); // don't stop event, we still want window to close
	}



void gatmo_demo_t::Setup(){
	id=lastid++;
		mi=GG_MAIN::gatmoui.demomenu->AppendCheckItem(id, wxString::FromAscii((const char *)name), wxT("Run this Demo"));
//	mi=GG_MAIN::gatmoui.demomenu->Append(wxID_ANY, wxString::FromAscii((const char *)name), wxT("Run this Demo"));
	caller->gdi_parent->Connect(mi->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(GatmoDisplayInterface::MenuSelect));
}

void gatmo_demo_t::Enable(){
	call_init();
	runthis=true;
}

void gatmo_demo_t::call(){
	if(!runthis) return;
	static int callcount=0;
	call_spin(callcount);
	callcount++;
}

void gatmo_demo_t::Disable(){
	call_fini();
	runthis=false;
}


void gatmo_demo_t::call_init(){
	if(init)
		vars=init(caller);
}

//call spin takes note of how big the list of display objects was, and notes down any that were added after spin is called
void gatmo_demo_t::call_spin(int iteration){
	if(iteration%freq==0){
		int sizebefore=GG_MAIN::gatmoui.gdisplayList.size();
		std::list<GatmoObjectDisplay*>::iterator before=GG_MAIN::gatmoui.gdisplayList.end();
		before--;
		vars=spin(caller,vars);

		//if any display objects have been added, add those objects to the list we are maintaining.
		if(GG_MAIN::gatmoui.gdisplayList.size()>sizebefore){
			before++;
//			std::cout<<"gatmo_demo_t::call_spin  : "<<GG_MAIN::gatmoui.gdisplayList.size()-sizebefore<<"  additional display objects detected"<<std::endl;
			for (std::list<GatmoObjectDisplay*>::iterator od=before; od!=GG_MAIN::gatmoui.gdisplayList.end(); ++od)
				displayed_objects.push_back(*od);
		}
//		std::cout<<"gatmo_demo_t::call_spin  : "<<displayed_objects.size()<<"  display objects"<<std::endl;
	}

}
void gatmo_demo_t::call_fini(){
	if(fini)
		fini(caller,vars);
	for (std::list<GatmoObjectDisplay*>::iterator od=displayed_objects.begin(); od!= displayed_objects.end(); ++od) {
		std::cout<<"removing "<<(*od)->name<<std::endl;
		(*od)->RemoveFromDisplayList();
//		delete (*od);  //TODO: This is a memory leak.  it needs to be fixed!
	}
}
gatmo_demo_t::gatmo_demo_t(gatmo_demo_init_func i, gatmo_demo_spin_func s, gatmo_demo_fini_func f, int fr, GatmoDisplayInterface * c, char*n){
	init=i;
	spin=s;
	fini=f;
	freq=fr;
	caller=c;
	runthis=false;
	strcpy(name,n);
	Setup();
}
gatmo_demo_t::gatmo_demo_t(gatmo_demo_spin_func s, int fr, GatmoDisplayInterface * c, char*n){
	init=NULL;
	spin=s;
	fini=NULL;
	freq=fr;
	caller=c;
	runthis=false;
	strcpy(name,n);
	Setup();
}














