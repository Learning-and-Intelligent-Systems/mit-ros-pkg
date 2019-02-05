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


#ifndef GATMOPLUGINDISPLAY_H_
#define GATMOPLUGINDISPLAY_H_
#include "globals.h"

class GatmoPluginDisplay : public GatmoDisplayInterface
{
	private:
		//I usually put a pointer to whatever class I'm communicating with,
		//for example a map

	    //features of the gui:
		//only need to declare things here that you will need to reference explicitly other places
		//for example to set the value dynamically
		wxTextCtrl *numres;
		wxCheckBox* drawenablecb;

		//gui callback functions:
		void OnButtonPress(wxCommandEvent& event);
		void DrawEnableCB(wxCommandEvent& event);


	public:
		//have to have these functions:
	    void InitGui();
		void Draw();
		void DrawAlt();
		//have to have a constructor that passes the parent window pointer to the underlying panel
		GatmoPluginDisplay(wxWindow*);

	virtual ~GatmoPluginDisplay();
};




#endif /*GATMOPLUGINDISPLAY_H_*/
