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
//#include <opencv/highgui.h>
//#include <opencv/cv.h>
#ifndef GG_GLOBALS_H
#define GG_GLOBALS_H



#include "gatmounits.h"
#include <wx/wx.h>
#include <wx/glcanvas.h>
#include <wx/textdlg.h>
#include <wx/utils.h>
#include <wx/panel.h>
#include <wx/listbox.h>
#include <wx/tglbtn.h>
#include <GL/glut.h>    // Header File For The GLUT Library
#include <GL/gl.h>	// Header File For The OpenGL32 Library
#include <GL/glu.h>	// Header File For The GLu32 Library
#include <GL/glx.h>     // Header file fot the glx libraries.
#include <vector>
#include <list>
#include <sys/time.h>
#include <boost/thread/mutex.hpp>


class GatmoUI;
class MyFrame;
class BasicGLPane;
class GMouse;
class MyApp;
struct gatmo_demo_t;




namespace GG_MAIN{
	extern MyApp *pglobalapp;
	extern GatmoUI gatmoui;
	extern bool runmode;
	extern bool trainingmode;

	extern GLUquadricObj *quadratic;
//	extern boost::mutex dataDisplayMutex;
//	extern pthread_mutex_t dataDisplayMutex;
	extern gatmo_mutex_t dataDisplayMutex;
	extern int window_width, window_height;
}



//define function pointer type for draw object struct:
typedef void (*draw_func)(gatmo_pose_t pose, float z);

/** \brief  GatmoObjectDisplay is something that will be inherited by everyone, so we can maintain lists of objects to draw */
class GatmoObjectDisplay{
	public:
		GatmoObjectDisplay(){ object_display_id=rand();}
	int object_display_id;
	bool display;
	bool emph;
//	gatmo_bbox_t bbox;
	virtual void Draw()=0;//{bbox.Draw();}
	virtual void DrawAlt()=0;//{bbox.DrawAlt();}
	char name[50];
	virtual ~GatmoObjectDisplay(){}
	void AddtoDisplayList();
	void RemoveFromDisplayList();

};

/** \brief @b GatmoDisplayInterface is a wxpanel than can be displayed, as well as a link to display various objects in the main display  */
class GatmoDisplayInterface : public wxPanel, public GatmoObjectDisplay {
	public:
	virtual void InitGui()=0;
	GatmoDisplayInterface(wxWindow *parent):wxPanel(parent,wxID_ANY,wxDefaultPosition,wxDefaultSize,wxBORDER_SIMPLE){gdi_parent=parent;}
	wxPanel * getPanel(){ return (wxPanel *)this; }
	GatmoObjectDisplay *getObject(){ return (GatmoObjectDisplay *)this;}
	void MenuSelect(wxCommandEvent& evt);
	wxWindow *gdi_parent;  //currently used to connect wxmenuevents

};

/** \brief @b GatmoUI is a global class that is used to control things that are global to the system, like where we are looking, and what time it is.
 */
class GatmoUI{

	//view offset is the camera x,y (weight is used for zoom level)
//	gatmo_point_t view_offset;
	gatmo_pose6D_t view_pose;
	gatmo_pose6D_t view_vel;

	public:
		gatmo_mutex_t gdisplayListMutex;
		float view_xrot;
	void loadGlobalFrame(){
		glLoadIdentity();
		double _tcam[16];
		view_pose.toOpenGLMatrix(_tcam);
		glMultMatrixd(_tcam);
//		glTranslatef(view_offset.x,view_offset.y,-6.0f+view_offset.weight);  //global movement
//		glRotatef(view_xrot,1.0,0.0,0.0);
	}
	double getTime(){
	   struct timeval tv;
	   gettimeofday(&tv, NULL);
	   return (double)(tv.tv_sec) + (tv.tv_usec)/1000000.0;
	}
	float getscale();
	//things that everyone will want to modify:
	wxStatusBar *statusbar;
	wxMenu *demomenu;
	wxMenu *settingsmenu;
	void setStatusPoint(gatmo_point_t pt){
		wxString val;
		val<<wxT("x = ")<<pt.x;
		statusbar->SetStatusText(val,0);
		val.Clear();
		val<<wxT("y = ")<<pt.y;
		statusbar->SetStatusText(val,1);

	}
	GatmoUI():gdisplayListMutex("Display List Mutex"){
		setVOVel(gatmo_pose6D_t(0,0,0));
	}
	//list of objects that we are displaying:
	std::list<GatmoObjectDisplay*> gdisplayList;
	std::list<gatmo_demo_t*> gdemos;

	gatmo_pose6D_t getVO(){return view_pose;}
	void setVO(gatmo_pose6D_t vo){view_pose=vo;}
	void setVOVel(gatmo_pose6D_t vo){view_vel=vo;}
	void updateVO();


};

/** \brief Init function for a gatmo demo */
typedef void *(*gatmo_demo_init_func)(GatmoDisplayInterface*);

/** \brief loop function for a gatmo demo */
typedef void *(*gatmo_demo_spin_func)(GatmoDisplayInterface*, void*);

/** \brief shutdown function for a gatmo demo */
typedef void (*gatmo_demo_fini_func)(GatmoDisplayInterface*, void*);

/** \brief @b gatmo_demo_t is a configurable demo that gets called by an update function
 * it has 3 functions: one to initialize, one to call periodically, and one to shutdown.
 * it also has a void * for storing variables
 * and a frequency for how often it should be called (1=every time, 2 = every other time, etc.)
 *  */
struct gatmo_demo_t{
	static int lastid;
	int id;
	gatmo_demo_init_func init;
	wxMenuItem *mi;
	gatmo_demo_spin_func spin;
	gatmo_demo_fini_func fini;
	GatmoDisplayInterface *caller;
	std::list<GatmoObjectDisplay*> displayed_objects;
	char name[500];
	void * vars;
	int freq;
	bool runthis;

	void Setup();

	void Enable();
	void call();
	void Disable();
	void call_init();
	//call spin takes note of how big the list of display objects was, and notes down any that were added after spin is called
	void call_spin(int iteration);
	void call_fini();
	gatmo_demo_t(gatmo_demo_init_func i, gatmo_demo_spin_func s, gatmo_demo_fini_func f, int fr, GatmoDisplayInterface * c, char*n);
	gatmo_demo_t(gatmo_demo_spin_func s, int fr, GatmoDisplayInterface * c, char*n);
};




struct gatmo_disp_obj_t{
	bool display;
	bool emph;
	float x,y,z,a;
	draw_func df;
	char name[20];

	void Draw(){
		if(!display) return;
		gatmo_pose_t p;
		p.px=x; p.py=y; p.pa=a;
		df(p, z);
	}
	gatmo_disp_obj_t(){
//		df=&drawRobot;
		x=0.0;y=0.0;z=0.0;a=0.0;
		display=false;
		emph=false;
	}
	void setname(char *n){
		strcpy(name,n);
	}

};



struct gdisplay_obj_list_t{
	float select_radius;
	std::list <gatmo_disp_obj_t> objects;

	gdisplay_obj_list_t(){select_radius=.1;}
	std::list<gatmo_disp_obj_t>::iterator add(gatmo_disp_obj_t &obj){
		objects.push_front(obj);
		return objects.begin();
	}
	void drawList(){
		 for (std::list<gatmo_disp_obj_t>::iterator it=objects.begin() ; it != objects.end(); it++ )
			it->Draw();
	}
	std::list<gatmo_disp_obj_t>::iterator get(char* name){
		for (std::list<gatmo_disp_obj_t>::iterator it=objects.begin() ; it != objects.end(); it++ )
			if(strcmp(it->name,name)==0)
				return it;
		return objects.end();
	}
	std::list<gatmo_disp_obj_t>::iterator get(float x,float y){
		for (std::list<gatmo_disp_obj_t>::iterator it=objects.begin() ; it != objects.end(); it++ )
			if((it->x-x)*(it->x-x)+(it->y-y)*(it->y-y)<=select_radius*select_radius  && strcmp(it->name,"planner"))//never select the planner
				return it;
		return objects.end();
	}

};



#endif
