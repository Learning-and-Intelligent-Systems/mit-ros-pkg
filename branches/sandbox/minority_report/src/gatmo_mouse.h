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


#ifndef GATMO_MOUSE_
#define GATMO_MOUSE_

#include "globals.h"

namespace GG_MAIN{

	extern GMouse gmouse;


}






typedef enum {
	GMB_RIGHT               = 0,
	GMB_CENTER              = 1,
	GMB_LEFT                = 2
} gg_mbutton;

struct gatmo_mouse_event_t{
	double x, y, z;
	int winx, winy;
	double time;
	gatmo_pose6D_t view_offset; //useful when dragging to set position
	gg_mbutton button;
	wxMouseEvent event_; //eventually might not use this, but why?
	gatmo_mouse_event_t(wxMouseEvent& event){
		time=GG_MAIN::gatmoui.getTime();
		view_offset=GG_MAIN::gatmoui.getVO();
		winx=event.m_x; winy=event.m_y;
		if(event.LeftDown() || event.LeftUp() || (event.Dragging() && event.LeftIsDown())) button=GMB_LEFT;
		if(event.MiddleDown() || event.MiddleUp() || (event.Dragging() && event.MiddleIsDown())) button=GMB_CENTER;
		if(event.RightDown() || event.RightUp() || (event.Dragging() && event.RightIsDown())) button=GMB_RIGHT;
		event_ = event;
		tracepoint();
	}
//	gatmo_mouse_event_t(gatmo_mouse_event_t& event){
	///shiftVO calculates the distance that the view should be shifted
	gatmo_point3D_t shiftVO(	gatmo_mouse_event_t &current){
		static gatmo_point3D_t view_offset_=gatmo_point3D_t(0,0,0);
//		if(current.button==GMB_LEFT){ //only change x and y
			view_offset_+=gatmo_point_t(current.x,current.y,current.z)-gatmo_point_t(x,y,z);

//		}
		gatmo_point3D_t out=view_offset_;
//
//		if(current.button==GMB_RIGHT){ //only change zoom
//
//			out.weight+=current.y-y;
//		}
		return out;

//			return gatmo_point_t(current.x,current.y)-gatmo_point_t(x,y);
	}

	///tracepoint converts window coordinates to world coordinates
	void tracepoint(){
		glPushMatrix();
		GG_MAIN::gatmoui.loadGlobalFrame();
		GLint viewport[4];
		GLdouble modelview[16];
		GLdouble projection[16];
		GLfloat twinX, twinY, twinZ;
		GLdouble posX, posY, posZ;

		glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
		glGetDoublev( GL_PROJECTION_MATRIX, projection );
		glGetIntegerv( GL_VIEWPORT, viewport );

		twinX = (float)winx;
		twinY = (float)viewport[3] - (float)winy;
		GLdouble junkX, junkY, junkz;
		gluProject( 0, 0, 0, modelview, projection, viewport, &junkX, &junkY, &junkz);
		twinZ=junkz;
		gluUnProject( twinX, twinY, twinZ, modelview, projection, viewport, &posX, &posY, &posZ);
		x=posX;
		y=posY;
		z=posZ;
//		std::cout<<__PRETTY_FUNCTION__<<"input: "<<twinX<<", "<<twinY<<", "<<twinZ<<"   output: "<<x<<", "<<y<<", "<<z<<std::endl;
		glPopMatrix();
	}

	///tracepoint converts window coordinates to world coordinates
	void inframetracepoint(){
		GLint viewport[4];
		GLdouble modelview[16];
		GLdouble projection[16];
		GLfloat twinX, twinY, twinZ;
		GLdouble posX, posY, posZ;

		glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
		glGetDoublev( GL_PROJECTION_MATRIX, projection );
		glGetIntegerv( GL_VIEWPORT, viewport );

		twinX = (float)winx;
		twinY = (float)viewport[3] - (float)winy;
		GLdouble junkX, junkY, junkz;
		gluProject( 0, 0, 0, modelview, projection, viewport, &junkX, &junkY, &junkz);
		twinZ=junkz;
		gluUnProject( twinX, twinY, twinZ, modelview, projection, viewport, &posX, &posY, &posZ);
		x=posX;
		y=posY;
		z=posZ;
	}
};

struct gatmo_mouse_click_t{
	gatmo_mouse_event_t down, up;
	gatmo_mouse_click_t(gatmo_mouse_event_t &u, gatmo_mouse_event_t &d):down(d),up(u){	}
};

/////use the mouse override class to tell the mouse class what to do when a click occurs
//typedef bool (*mouse_override)(wxMouseEvent& event);
/** \brief @b GM_FunctorAbstract is an abstract class that mouse functions inherit from */
class GM_FunctorAbstract{
public:
	std::string name;
	virtual bool eventcb(gatmo_mouse_event_t last, gatmo_mouse_event_t &prev){ return false;}
	virtual bool eventcb(gatmo_mouse_event_t last){return false;}
	GM_FunctorAbstract(char* _name):name(_name){}
};

/** \brief @b GM_Functor allows mouse callback functions to be called from any class */
template <class cbclass>
class GM_Functor: public GM_FunctorAbstract{
private:
	cbclass *obj;
	bool (cbclass::*eventcbd_)(gatmo_mouse_event_t,gatmo_mouse_event_t&);
	bool (cbclass::*eventcbs_)(gatmo_mouse_event_t);

public:
	virtual bool eventcb(gatmo_mouse_event_t last, gatmo_mouse_event_t &prev){
		if(eventcbd_){
			return (*obj.*eventcbd_)(last,prev);
		}
		return false;
	}
	virtual bool eventcb(gatmo_mouse_event_t last){
		if(eventcbs_){
			return (*obj.*eventcbs_)(last);
		}
		return false;
	}
	GM_Functor(char* _name, cbclass *_obj,
			bool (cbclass::*_eventcbd)(gatmo_mouse_event_t,gatmo_mouse_event_t&),
			bool (cbclass::*_eventcbs)(gatmo_mouse_event_t)):
				GM_FunctorAbstract(_name),obj(_obj),eventcbd_(_eventcbd),eventcbs_(_eventcbs){
	}
};

class GM_GeneralFunctor: public GM_FunctorAbstract{
private:
	bool (*eventcbd_)(gatmo_mouse_event_t,gatmo_mouse_event_t&);
	bool (*eventcbs_)(gatmo_mouse_event_t);

public:
	virtual bool eventcb(gatmo_mouse_event_t last, gatmo_mouse_event_t &prev){
		if(eventcbd_){
			return eventcbd_(last,prev);
		}
		return false;
	}
	virtual bool eventcb(gatmo_mouse_event_t last){
		if(eventcbs_){
			return eventcbs_(last);
		}
		return false;
	}
	GM_GeneralFunctor(char* _name,
			bool (*_eventcbd)(gatmo_mouse_event_t,gatmo_mouse_event_t&),
			bool (*_eventcbs)(gatmo_mouse_event_t)):GM_FunctorAbstract(_name),eventcbd_(_eventcbd),eventcbs_(_eventcbs){}
};






/** \brief GMouse class keeps track of all clicks and drags, so we can move things around */
class GMouse{
	std::list<gatmo_mouse_event_t> current_clicks;
std::list<gatmo_mouse_click_t> past_clicks;
//	bool click_override; //a flag indicating that we are doing something special with the mouse clicks, and we should call the appropriate function.
//	bool drag_override;
//	mouse_override click_override_func;
//	mouse_override drag_override_func;
	std::list<GM_FunctorAbstract*> override_funcs;
	public:
//	GMouse(){click_override=false; drag_override=false;}

	//mouse click event handles all click type events, both up and down
	void mouseClickEvent(wxMouseEvent& event){
		gatmo_mouse_event_t gme(event); //loads up all the relevant data

		if(event.ButtonDown()){
			//first, make sure there are no old clicks in the list, indicating a dragging off screen
			for (std::list<gatmo_mouse_event_t>::iterator gmei=current_clicks.begin(); gmei!= current_clicks.end(); ++gmei) {
				if(gmei->button==gme.button){
					gmei=current_clicks.erase(gmei);
					gmei--;  //may cause issues, if trying to back up past the first...
				}
			}
			//now add event to current_clicks:
			current_clicks.push_back(gme);

			//now run the callback
			if(override_funcs.size())
				for (std::list<GM_FunctorAbstract*>::iterator f=override_funcs.begin(); f!= override_funcs.end(); ++f) {
					if((*f)->eventcb(gme)){ //cycle through event overrides.  If none are true, then execute standard behavior.
						break;
					}
				}

		}
		if(event.ButtonUp()){
//			std::cout<<"button up "<<current_clicks.size()<<std::endl;
			//match the click to a down click
			for (std::list<gatmo_mouse_event_t>::iterator gmei=current_clicks.begin(); gmei!= current_clicks.end(); ++gmei) {
//				std::cout<<"button up trying to delete a "<<gme.button<<" this is a "<<gmei->button<<std::endl;
				if(gmei->button==gme.button){

					past_clicks.push_back(gatmo_mouse_click_t(gme,*gmei));
					current_clicks.erase(gmei);
					//don't let past_clicks get too big - I'm not sure what we need them for, anyway...
					if(past_clicks.size()>10) past_clicks.erase(past_clicks.begin());

					//now run callback
					if(override_funcs.size())
						for (std::list<GM_FunctorAbstract*>::iterator f=override_funcs.begin(); f!= override_funcs.end(); ++f) {
							if((*f)->eventcb(gme,*gmei)){ //cycle through event overrides.  If none are true, then execute standard behavior.
								break;
							}
						}
					break;

				}
			}

			//if up and no down, just ignore...
		}
		if(event.GetWheelRotation()){
			//run the callback
			if(override_funcs.size())
				for (std::list<GM_FunctorAbstract*>::iterator f=override_funcs.begin(); f!= override_funcs.end(); ++f) {
					if((*f)->eventcb(gme)){ //cycle through event overrides.  If none are true, then execute standard behavior.
						break;
					}
				}

		}
	}



	void mouseMotionEvent(wxMouseEvent& event){
//		std::cout<<"motion event "<<event.m_x<<" "<<event.m_y<<"  "<<current_clicks.size()<<std::endl;
		gatmo_mouse_event_t gme(event);
//		std::cout<<"motion event "<<gme.x<<" "<<gme.y<<"  "<<current_clicks.size()<<std::endl;
		GG_MAIN::gatmoui.setStatusPoint(gatmo_point_t(gme.x,gme.y));
		gatmo_mouse_event_t *prev=NULL;
		if(current_clicks.size())
			prev=&(current_clicks.back());
			//we are either using the mouse for navigation, or doing some special function
		bool overrriden=false;
		if(override_funcs.size() && current_clicks.size())
			for (std::list<GM_FunctorAbstract*>::iterator f=override_funcs.begin(); f!= override_funcs.end(); ++f) {
				if((*f)->eventcb(gme,*prev)){ //cycle through event overrides.  If none are true, then execute standard behavior.
					overrriden=true;
					break;
				}
			}
		if(override_funcs.size() && !current_clicks.size())
			for (std::list<GM_FunctorAbstract*>::iterator f=override_funcs.begin(); f!= override_funcs.end(); ++f) {
				if((*f)->eventcb(gme)){ //cycle through event overrides.  If none are true, then execute standard behavior.
					overrriden=true;
					break;
				}
			}
//		if(!overridden){ //may be doing navigation:
//			if(event.Dragging())
////			std::cout<<"dragging "<<prev<<std::endl;
//			//only navigating if dragging (and we know where we first clicked)
//			if(event.Dragging() && prev){
////				std::cout<<"moving "<<event.m_x<<" "<<event.m_y<<std::endl;
//				GG_MAIN::gatmoui.setVO(prev->shiftVO(gme)); //handles zoom and translation
//			}
//		}

	}


	void AddCallback(char* _name, bool (*_eventcbd)(gatmo_mouse_event_t,gatmo_mouse_event_t&), bool (*_eventcbs)(gatmo_mouse_event_t)){
		GM_FunctorAbstract *newfunctor = new GM_GeneralFunctor(_name,_eventcbd,_eventcbs);
		override_funcs.push_front(newfunctor);
	}

	template <class cbclass>
	void AddCallback(char* _name,cbclass *_obj,
			bool (cbclass::*_eventcbd)(gatmo_mouse_event_t,gatmo_mouse_event_t&),
			bool (cbclass::*_eventcbs)(gatmo_mouse_event_t)){
		GM_FunctorAbstract *newfunctor = new GM_Functor<cbclass>(_name,_obj,_eventcbd,_eventcbs);
		override_funcs.push_front(newfunctor);
	}

};


//mouse callbacks:








#endif /*GATMO_MOUSE_*/
