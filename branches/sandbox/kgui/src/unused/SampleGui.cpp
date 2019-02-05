/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Garratt Gallagher
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

#include "SampleGui.h"
//#define VERBOSE_MUTEX 0

gatmo_pose6D_t fakecam,fakeref;
bool showdrag;


namespace GG_MAIN{
	GMouse gmouse;
}



IMPLEMENT_APP(MyApp)

class MyFrame : public wxFrame
{
public:
    MyFrame() : wxFrame((wxFrame *)NULL, 133,  wxT("Robot Tracker"), wxPoint(50,50), wxSize(1280,960))
    {
    }
    void onClose(wxCloseEvent& evt)
    {
        wxGetApp().activateRenderLoop(false);
        evt.Skip(); // don't stop event, we still want window to close
    }
    void onMenu(wxCommandEvent& evt)
    {
        evt.Skip(); // don't stop event, we still want window to close
    }

    void UpdateUIEvent(wxUpdateUIEvent& evt)
    {
        evt.Skip(); // allow the event to run it's course, drawing the frame and such
    }

    DECLARE_EVENT_TABLE()
};

///ViewShifter takes the mouse event and uses it to shift the view.  should be the last callback in the gmouse event list
class ViewShifter{
public:
   gatmo_pose6D_t view_offset;
   gatmo_pose6D_t ref_node;
   double global_yaw,global_pitch, zoom;
   gatmo_point3D_t reference;
   gatmo_point3D_t getZdir(){
      return view_offset.orientation.rotate(gatmo_point3D_t(0,0,1));
   }
   gatmo_point3D_t getXdir(){
      return view_offset.orientation.rotate(gatmo_point3D_t(1,0,0));
   }
   gatmo_point3D_t getYdir(){
      return view_offset.orientation.rotate(gatmo_point3D_t(0,1,0));
   }

   void setCamera(){
//    global_pitch=0;
      double a=zoom*cos(global_pitch);
      //position of the camera from reference point:
      gatmo_point3D_t campos(a*cos(global_yaw),a*sin(global_yaw), zoom*sin(global_pitch));
      //full position = ref+campos
      view_offset.position=reference+campos;

      gatmo_quaternion_t rot1;
      view_offset.orientation.fromAxisAngle(global_yaw,0,0,1.0);
      rot1.fromAxisAngle(-global_pitch,0,1.0,0);
      view_offset.orientation=view_offset.orientation*rot1;


      //switch from x forward to z forward:
      gatmo_quaternion_t rot;
      rot.fromAxisAngle(1.5707,00,1.0,0.0);
      view_offset.orientation=view_offset.orientation*rot;
      rot.fromAxisAngle(1.5707,00,.0,1.0);
      view_offset.orientation=view_offset.orientation*rot;

      //send final orientation to global view frame
      GG_MAIN::gatmoui.setVO(view_offset.inv());
      //reference node is the thing we are looking at.  it looks in our direction.
      ref_node.orientation = view_offset.orientation;
      ref_node.position=reference;
   }

   bool callbackSingle(gatmo_mouse_event_t last){
      showdrag=false;
      if(last.event_.GetWheelRotation()){
         showdrag=true;
         if(last.event_.GetWheelRotation()>0){
            zoom+=.5;
         }else{
            zoom -=.5;
         }
         zoom=std::max(.2,zoom);
         setCamera();
         return true;
      }
      return false;
   }
   bool callbackDouble(gatmo_mouse_event_t last, gatmo_mouse_event_t &prev_){
      showdrag=false;
      if(prev_.event_.ControlDown()) return false;
      gatmo_mouse_event_t prev = prev_;
      if(last.event_.Dragging()){
         showdrag=true;
         if(prev.event_.LeftDown()){
            //just translate
            gatmo_point3D_t shift;
            shift=prev.shiftVO(last);

            double diffx=double(last.winx-prev_.winx)/560.0;
            double diffy=double(prev_.winy-last.winy)/560.0;

            reference=reference-getXdir()*diffx-getYdir()*diffy;
            setCamera();
            prev_.winy=last.winy;
            prev_.winx=last.winx;
            return true;
         }
         if(prev.event_.RightDown()){

            global_pitch +=double(last.winy-prev_.winy)/250.0;
            global_yaw -=double(last.winx-prev_.winx)/260.0;

            global_pitch=std::min(1.57,global_pitch);
            global_pitch=std::max(-1.57,global_pitch);

            setCamera();
            prev_.winy=last.winy;
            prev_.winx=last.winx;
            return true;
         }


      }
      return false;
   }
   ViewShifter(){
      GG_MAIN::gmouse.AddCallback("viewshifter",this,&ViewShifter::callbackDouble,&ViewShifter::callbackSingle);
      global_yaw=0;
      global_pitch=-.2;
   }

};



///Here is where you through all your initialization stuff
bool MyApp::OnInit()
{

   new ViewShifter;
//   std::cout<<"original view"<<std::endl;
//   GG_MAIN::gatmoui.getVO().print();
//   viewshifter->zoom=3.0;
//   viewshifter->global_yaw=.3;
//   viewshifter->global_pitch=.9;
//   viewshifter->setCamera();
//   std::cout<<"new view"<<std::endl;
//   GG_MAIN::gatmoui.getVO().print();
//   gatmo_pose6D_t globalpose = GG_MAIN::gatmoui.getVO();
//   globalpose.position.z=-5.0;
//// globalpose.position.x=1.0;
//   GG_MAIN::gatmoui.setVO(globalpose);
	GG_MAIN::gatmoui.demomenu = new wxMenu;
	GG_MAIN::gatmoui.settingsmenu = new wxMenu;
    render_loop_on = false;

    //make whole  display
    frame = new MyFrame();

    //sizers inside frame - partition how things are displayed
    wxBoxSizer* sizer = new wxBoxSizer(wxHORIZONTAL);
	wxBoxSizer* v_sizer = new wxBoxSizer(wxVERTICAL);

	//add glcanvas
	int args[] = {WX_GL_RGBA, WX_GL_DOUBLEBUFFER, WX_GL_DEPTH_SIZE, 16, 0};
    glPane = new BasicGLPane( (wxFrame*) frame, args);
	// will take up 70% of frame
    sizer->Add(glPane, 7, wxEXPAND);


    //create plugins here.  You can create a bunch of independent plugins, or make one that launches others
    GatmoPluginDisplay *gpd = new GatmoPluginDisplay(frame);
    v_sizer->Add(gpd, 0);

    //make a button that will reset to default view (in case you get lost)
    wxButton *bresetview = new wxButton(frame, wxID_ANY, wxT("Reset View"));
	Connect(bresetview->GetId(), wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(MyApp::ResetView));
	v_sizer->Add(bresetview,0);

	//will take 30% of frame
	sizer->Add(v_sizer, 3);

//Now setup the menu bar:
	menubar = new wxMenuBar;

	//file menu
	wxMenu *fileMenu = new wxMenu;
	/*wxMenuItem *fm =*/fileMenu->Append(1, wxT("Q&uit\t Ctrl-Q"), wxT("Quit this program"));
	menubar->Append(fileMenu,wxT("&File"));

	//demo menu - a global thing...
	menubar->Append(GG_MAIN::gatmoui.demomenu,wxT("&Demos"));
	//settings menu - a global thing...
	menubar->Append(GG_MAIN::gatmoui.settingsmenu,wxT("&Settings"));


	frame->SetMenuBar(menubar);


	GG_MAIN::gatmoui.statusbar =  frame->CreateStatusBar(3);
	int swidths[] = {100,100,-1};
	GG_MAIN::gatmoui.statusbar->SetStatusWidths(3,swidths); //set the widths to
	GG_MAIN::gatmoui.statusbar->SetStatusText(wxT("Ready"),2);
//	GG_MAIN::gatmoui.setStatusPoint(gatmo_point_t(3.1415,9265.35));

//	frame->Connect(fm->GetId(), wxEVT_COMMAND_MENU_SELECTED, wxCommandEventHandler(MyFrame::onMenu));


    frame->SetSizer(sizer);
    frame->Show();


    activateRenderLoop(true);
    return true;
}

void MyApp::activateRenderLoop(bool on)
{
    if(on and !render_loop_on)
    {
        Connect( wxID_ANY, wxEVT_IDLE, wxIdleEventHandler(MyApp::onIdle) );
        render_loop_on = true;
    }
    else if(!on and render_loop_on)
    {
        Disconnect( wxEVT_IDLE, wxIdleEventHandler(MyApp::onIdle) );
        render_loop_on = false;
    }
}
void MyApp::onIdle(wxIdleEvent& evt)
{
    if(render_loop_on)
    {
    	if(!GG_MAIN::runmode)
		usleep(5000);

    	//Run polling update calls here - things that need to be done to update displays

    	for (std::list<gatmo_demo_t*>::iterator d=GG_MAIN::gatmoui.gdemos.begin(); d!= GG_MAIN::gatmoui.gdemos.end(); ++d)
    		(*d)->call();

		glPane->paintNow();
        evt.RequestMore(); // render continuously, not only once on idle
    }
}





void BasicGLPane::mouseClick(wxMouseEvent& event) {
	GG_MAIN::gmouse.mouseClickEvent(event);
}
void BasicGLPane::mouseMotion(wxMouseEvent& event) {
	GG_MAIN::gmouse.mouseMotionEvent(event);
}


void BasicGLPane::paintEvent(wxPaintEvent& evt)
{
    wxPaintDC dc(this);
    render(dc);
}

void BasicGLPane::paintNow()
{
    wxClientDC dc(this);
    render(dc);
}



void BasicGLPane::mouseWheelMoved(wxMouseEvent& event) {
	std::cout<<"mouseWheelMoved: whel rotation: "<<event.GetWheelRotation()<<std::endl;
	if(event.GetWheelRotation()<0)
		GG_MAIN::gatmoui.view_xrot-=.5;
	else
		GG_MAIN::gatmoui.view_xrot+=.5;

}

BasicGLPane::BasicGLPane(wxFrame* parent, int* args) :
wxGLCanvas(parent, wxID_ANY,  wxDefaultPosition, wxDefaultSize, 0, wxT("GLCanvas"),  args)
{
    int argc = 0;
    char** argv = NULL;
    glutInit(&argc, argv);

}

void BasicGLPane::resized(wxSizeEvent& evt)
{
    wxGLCanvas::OnSize(evt);

    Refresh();
}



void BasicGLPane::prepare3DViewport(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y)
{
    /*
     *  Inits the OpenGL viewport for drawing in 3D.
     */

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Black Background
    glClearDepth(1.0f);	// Depth Buffer Setup
    glEnable(GL_DEPTH_TEST); // Enables Depth Testing
    glDepthFunc(GL_LEQUAL); // The Type Of Depth Testing To Do
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

    glEnable(GL_COLOR_MATERIAL);

    glViewport(topleft_x, topleft_y, bottomrigth_x-topleft_x, bottomrigth_y-topleft_y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float ratio_w_h = (float)(bottomrigth_x-topleft_x)/(float)(bottomrigth_y-topleft_y);
    gluPerspective(45 /*view angle*/, ratio_w_h, 0.1 /*clip close*/, 200 /*clip far*/);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

 //     gluPerspective(90.0f,(GLfloat)Width/(GLfloat)Height,0.1f,100.0f);	// Calculate The Aspect Ratio Of The Window

//  glMatrixMode(GL_MODELVIEW);
  GG_MAIN::quadratic=gluNewQuadric();			// Create A Pointer To The Quadric Object ( NEW )
  gluQuadricNormals(GG_MAIN::quadratic, GLU_SMOOTH);	// Create Smooth Normals ( NEW )
//    BuildFont();

}

int BasicGLPane::getWidth()
{
    return GetSize().x;
}

int BasicGLPane::getHeight()
{
    return GetSize().y;
}



///this is what gets called every time the window draws
void BasicGLPane::render(wxDC& dc)
{
    if(!IsShown()) return;

    wxGLCanvas::SetCurrent();
    wxPaintDC(this); // only to be used in paint events. use wxClientDC to paint outside the paint event

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    prepare3DViewport(0,0,getWidth(), getHeight());

    //This call loads the current view:
    GG_MAIN::gatmoui.loadGlobalFrame();

    glPushMatrix();
 	glock(GG_MAIN::gatmoui.gdisplayListMutex);

 	// this calls all the plugin's draw functions if their display variable is true:
 	for (std::list<GatmoObjectDisplay*>::iterator disp=GG_MAIN::gatmoui.gdisplayList.begin(); disp!= GG_MAIN::gatmoui.gdisplayList.end(); ++disp) {
		GG_MAIN::gatmoui.loadGlobalFrame();
		if((*disp)->display){
			(*disp)->Draw();
		}
	}
	gunlock(GG_MAIN::gatmoui.gdisplayListMutex);
 	glPopMatrix();
    glFlush();
    SwapBuffers();
}


BEGIN_EVENT_TABLE(MyFrame, wxFrame)
EVT_CLOSE(MyFrame::onClose)
//EVT_PAINT(MyFrame::paintEvent)
//EVT_MENU(MyFrame::onMenu)
EVT_UPDATE_UI(133,MyFrame::UpdateUIEvent)
END_EVENT_TABLE()


BEGIN_EVENT_TABLE(BasicGLPane, wxGLCanvas)
EVT_MOTION(BasicGLPane::mouseMotion)

EVT_LEFT_DOWN(BasicGLPane::mouseClick)
EVT_LEFT_UP(BasicGLPane::mouseClick)
EVT_RIGHT_DOWN(BasicGLPane::mouseClick)
EVT_RIGHT_UP(BasicGLPane::mouseClick)
EVT_MIDDLE_DOWN(BasicGLPane::mouseClick)
EVT_MIDDLE_UP(BasicGLPane::mouseClick)
//EVT_LEAVE_WINDOW(BasicGLPane::mouseLeftWindow)
//EVT_SIZE(BasicGLPane::resized)
//EVT_KEY_DOWN(BasicGLPane::keyPressed)
//EVT_KEY_UP(BasicGLPane::keyReleased)
EVT_MOUSEWHEEL(BasicGLPane::mouseWheelMoved)
//EVT_PAINT(BasicGLPane::render)
EVT_PAINT(BasicGLPane::paintEvent)
END_EVENT_TABLE()

