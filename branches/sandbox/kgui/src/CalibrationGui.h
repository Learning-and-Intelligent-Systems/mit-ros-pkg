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

#ifndef GATMO_DISP_H
#define GATMO_DISP_H

#include "globals.h"
#include "gatmo_mouse.h"
#include "GatmoCalibrationDisplay.h"

class ViewShifter;
class BasicGLPane : public wxGLCanvas
{

public:
	BasicGLPane(wxFrame* parent, int* args);

	void resized(wxSizeEvent& evt);

	int getWidth();
	int getHeight();

    void render( wxDC& dc );
	void prepare2DViewport(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y);
	void prepare3DViewport(int topleft_x, int topleft_y, int bottomrigth_x, int bottomrigth_y);
    void paintEvent(wxPaintEvent& evt);
    void paintNow();


	// events
	void mouseClick(wxMouseEvent& event);
	void mouseMotion(wxMouseEvent& event);
	void mouseWheelMoved(wxMouseEvent& event);
//	void mouseLeftWindow(wxMouseEvent& event);
//	void keyPressed(wxKeyEvent& event);
//	void keyReleased(wxKeyEvent& event);


	DECLARE_EVENT_TABLE()
};

class MyApp: public wxApp
{
	protected:
    bool render_loop_on;
    bool OnInit();
    void onIdle(wxIdleEvent& evt);

    MyFrame* frame;

    char clog[5000];  //input log name
    bool fileloaded;
    bool maploaded;
	wxPanel* gatmoManagerPanel;
	wxMenuBar* menubar;

	//now things that MyApp keeps around:
	GatmoCalibrationDisplay *gpd;
	ViewShifter *viewshifter;

public:
    BasicGLPane * glPane;
    void activateRenderLoop(bool on);
    void ResetView(wxCommandEvent& event){
    	GG_MAIN::gatmoui.setVO(gatmo_pose6D_t(0,0,-5.0));
    	GG_MAIN::gatmoui.view_xrot=0;
    }

};





#endif
