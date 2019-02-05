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

#ifdef USING_ROS
#define GG_DEBUG ROS_DEBUG
#else
#define GG_DEBUG printf
#endif



#include "MinorityReportDisplay.h"

  timeval g_tick(){
     struct timeval tv;
     gettimeofday(&tv, NULL);
     return tv;
  }

  double g_tock(timeval tprev)
  {
     struct timeval tv;
     gettimeofday(&tv, NULL);
     return (double)(tv.tv_sec-tprev.tv_sec) + (tv.tv_usec-tprev.tv_usec)/1000000.0;
  }


  bool isOldFrame(FrameObject &f){
     return f.shouldDelete();
  }





void printOpenGLMatrix(double *mat){
   printf(" %.03f %.03f %.03f %.03f  \n",mat[0],mat[4],mat[8],mat[12]);
   printf(" %.03f %.03f %.03f %.03f  \n",mat[1],mat[5],mat[9],mat[13]);
   printf(" %.03f %.03f %.03f %.03f  \n",mat[2],mat[6],mat[10],mat[14]);
   printf(" %.03f %.03f %.03f %.03f  \n",mat[3],mat[7],mat[11],mat[15]);
}





void drawFrame(gatmo_pose6D_t p, double raylength){

   double _tcam[16];
   glDisable( GL_TEXTURE_2D );
   glPushMatrix();
   p.toOpenGLMatrix(_tcam);
   glMultMatrixd(_tcam);
   glColor3f(1.0f,0.0f,0.0f);
   glBegin(GL_LINES);
      glVertex3f(0,0,0);
      glVertex3f(raylength,0,0);
   glEnd();
   glColor3f(0.0f,1.0f,0.0f);
   glBegin(GL_LINES);
      glVertex3f(0,0,0);
      glVertex3f(0,raylength,0);
   glEnd();
   glColor3f(0.0f,0.0f,1.0f);
   glBegin(GL_LINES);
      glVertex3f(0,0,0);
      glVertex3f(0,0,raylength);
   glEnd();
   glPopMatrix();
   glEnable( GL_TEXTURE_2D );



}

void DrawEllipsoid(gatmo_pose6D_t p,unsigned int uiStacks, unsigned int uiSlices, float fA, float fB, float fC)
{

	   double _tcam[16];
	   glDisable( GL_TEXTURE_2D );
	   glPushMatrix();
	   p.toOpenGLMatrix(_tcam);
	   glMultMatrixd(_tcam);
	float Pi=3.141592;
	float tStep = (Pi) / (float)uiSlices;
	float sStep = (Pi) / (float)uiStacks;
//	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	for(float t = -Pi/2; t <= (Pi/2)+.0001; t += tStep)
	{
		glBegin(GL_TRIANGLE_STRIP);
		for(float s = -Pi; s <= Pi+.0001; s += sStep)
		{
			glVertex3f(fA * cos(t) * cos(s), fB * cos(t) * sin(s), fC * sin(t));
			glVertex3f(fA * cos(t+tStep) * cos(s), fB * cos(t+tStep) * sin(s), fC * sin(t+tStep));
		}
		glEnd();
	}
	   glPopMatrix();
	   glEnable( GL_TEXTURE_2D );
}


void drawCircle(gatmo_point3D_t pt,double innerrad,double outerrad, double colors[]){

   glDisable( GL_TEXTURE_2D );
   gatmo_pose6D_t p(pt);
   double _tcam[16];
   glPushMatrix();
   p.toOpenGLMatrix(_tcam);
   glMultMatrixd(_tcam);
   float Pi=3.141592;
   float tStep = (Pi) / 300;
   for(float t = -Pi; t <= (Pi)+.0001; t += tStep){
      glBegin(GL_QUADS);
      glColor4f(colors[4],colors[5],colors[6],colors[7]);
      glVertex3f(outerrad*cos(t+tStep), outerrad*sin(t+tStep),-.0001f);
      glVertex3f(outerrad*cos(t), outerrad*sin(t),-.0001f);
//      glColor4f(0.0,0.0,0.8,0.7);
      glColor4f(colors[0],colors[1],colors[2],colors[3]);
      glVertex3f(innerrad*cos(t+tStep), innerrad*sin(t+tStep),-.0001f);
      glVertex3f(innerrad*cos(t), innerrad*sin(t),-.0001f);
      glEnd();
   }
   glPopMatrix();
   glEnable( GL_TEXTURE_2D );


}

void drawsphere(gatmo_point3D_t pt, double radius,bool grabbing=false){
   gatmo_pose6D_t p(pt);
   double _tcam[16];
   glDisable( GL_TEXTURE_2D );
   glPushMatrix();
   p.toOpenGLMatrix(_tcam);
   glMultMatrixd(_tcam);
//   glColor3f(1.0f,0.0f,0.0f);
   if(grabbing)
      glColor3f(.00f,0.50f,0.0f);
   else
      glColor3f(.50f,0.50f,0.0f);
   glutWireSphere(radius,32,32);

   glPopMatrix();
   glEnable( GL_TEXTURE_2D );
}




void drawsphere(gatmo_pose6D_t p, double radius,bool grabbing=false){

   double _tcam[16];
   glDisable( GL_TEXTURE_2D );
   glPushMatrix();
   p.toOpenGLMatrix(_tcam);
   glMultMatrixd(_tcam);
//   glColor3f(1.0f,0.0f,0.0f);
   if(grabbing)
      glColor3f(.00f,0.50f,0.0f);
   else
      glColor3f(.50f,0.0f,0.0f);
   glutWireSphere(radius,32,32);

   glPopMatrix();
   glEnable( GL_TEXTURE_2D );
}

void drawHand(HandState h){
   glDisable( GL_TEXTURE_2D );
   glDisable(GL_DEPTH_TEST);
     glEnable (GL_BLEND); glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
   gatmo_quaternion_t temp(1.0,1.0,1.0,0.0);
   temp=temp.inv();
   for(uint i=0;i<h.fingers.size();++i){
	   gatmo_pose6D_t fpose;
	   fpose.position=h.fingers[i];
      gatmo_quaternion_t pt(h.fingers[i]-h.handpose.position);
      pt.normalize();
	   fpose.orientation.fromEuler(0.0,asin(pt.z),atan2(pt.y,pt.x));
      glColor3f(1.0f,0.0f,0.0f);
      if(h.fingers.size()==5){
         double colors[]={0.0,0.0,1.0,0.7,0.0,0.0,0.0,0.0};
         drawCircle(h.fingers[i],.005,.02,colors);
         double colors2[]={0.90,.90,1.0,0.7,0.0,0.0,1.0,0.70};
         drawCircle(h.fingers[i],.00,.005,colors2);
      }
   }
   glColor3f(0.30f,0.30f,0.1f);
   double colors[]={1.0,0.0,1.0,0.7,0.0,0.0,0.0,0.0};
   drawCircle(h.handpose.position,.00,.04,colors);
   glEnable( GL_TEXTURE_2D );
   glEnable (GL_DEPTH_TEST);
}


void transformHand(HandState &h){
   h.handpose.position.y*=-1.0;
   GG_DEBUG("before %.03f %.03f %.03f \n",h.handpose.position.x,h.handpose.position.y,h.handpose.position.z);
   for(uint i=0;i<h.fingers.size();++i)
      h.fingers[i].y*=-1.0;


}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief determine if two hands are framing a frame object
 *  \param h1 First hand (order doesn't matter)
 *  \param h2 Second Hand (order doesn't matter)
 *  \param frames a vector of Frame objects to check against
  * \return -1 if not framing, otherwise, the index of the object being framed
  */
int isFraming(Hand &h1, Hand &h2, std::vector<FrameObject> &frames){
   //criteria: hands are positioned on opposite sides of the frame, in opposite quadrants
   gatmo_point3D_t win1 = h1.getWinPos();
   gatmo_point3D_t win2 = h2.getWinPos();
   win1.z=0;
   win2.z=0;
   gatmo_line3d_t gline(win1,win2);
   for (uint i = 0; i < frames.size(); ++i) {
      if(frames[i].isCloseLine(gline)){
         int q1=frames[i].isInCorner(win1), q2=frames[i].isInCorner(win2);
//         std::cout<<"framecheck "<<q1<<"  "<<q2<<std::endl;
         if(q1 && q2 && fabs(q1-q2) == 2)
            return i;
         }
   }


   return -1;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief given that two hands are framing an object, get the all the recent states where they were doing this
 *  \param h1 First hand (order doesn't matter)
 *  \param h2 Second Hand (order doesn't matter)
 *  \param f the Frame object that the two hands are framing
 *  \param h1pts a vector of 3d points that gets filled with previous hand 1 positions
 *  \param h2pts a vector of 3d points that gets filled with previous hand 2 positions
  * \return -1 if not framing, otherwise, the index of the object being framed
  */
int getFramingsteps(Hand &h1, Hand &h2, FrameObject &f, std::vector<gatmo_point3D_t> &h1pts,
      std::vector<gatmo_point3D_t> &h2pts){
   //criteria: hands are positioned on opposite sides of the frame, in opposite quadrants
   h1pts.clear();
   h2pts.clear();
   if(h1.history.size() < 10 || h2.history.size() < 10)
      return -1;
   //TODO: should check timestamps to make sure we are syncronized
   for (int t = h1.history.size(); t>=1; --t) {
      gatmo_point3D_t win1 = h1.getWinPos(t);win1.z=0;
      gatmo_point3D_t win2 = h2.getWinPos(t); win2.z=0;
      gatmo_line3d_t gline(win1,win2);
      if(f.isCloseLine(gline)){
         h1pts.push_back(win1);
         h2pts.push_back(win2);
      }
      else
         break;
   }
   return h1pts.size();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Analyze the gestures that were detected in the last data frame
  */
void MinorityReportDisplay::getGestures(){
   if(lastgestureupdate >= laststamp){
      lastgestureupdate = laststamp; //having this weird bug where lastgestureupdate is huge...
      GG_DEBUG(" failed seq %.03f %.03f\n",lastgestureupdate,laststamp);
      return;
   }
   GG_DEBUG(" getting gestures seq %.03f \n",lastgestureupdate);
   lastgestureupdate=laststamp;

   //update any current draggings:
   for(uint j=0;j<hands.size();++j)
      if(hands[j].isCurrent(laststamp))
         hands[j].updateDragging();

  //see if we're grabbing an incoming frame
   bool grabbedincoming=false;
   for(uint i=0;i<incomingframes.size();++i){
        for(uint j=0;j<hands.size();++j)
              if(hands[j].isCurrent(laststamp) && incomingframes[i].isOn(hands[j].getLatest().handpose)){
                 //doesn't need to be clicking. just touch it and it's yours
                 frames.push_back(incomingframes[i]);
                 hands[j].held=&frames.back();
                 grabbedincoming=true;
                 incomingframes.erase(incomingframes.begin()+i);
                 break;
              }
        if(grabbedincoming) //only grab one frame at a time.
           break;

   }

   //check for saving gestures
   for(uint j=0;j<hands.size();++j)
      if(hands[j].isCurrent(laststamp) && hands[j].isSaving()){
         for(uint i=0;i<frames.size();++i)
            if(&frames[i]==hands[i].held){
               savedframes.push_back(frames[i]);
               frames.erase(frames.begin()+i);
            }
         organizeSavedFrames();
      }

   //check for grabbing - if the user closes their hand on an frame object
   for(uint i=0;i<frames.size();++i){
      frames[i].resetToPassive();
      for(uint j=0;j<hands.size();++j)
            if(hands[j].isCurrent(laststamp) && frames[i].isOn(hands[j].getLatest().handpose)){
               if(hands[j].clicking)
                  hands[j].held=&frames[i];
               break;
            }
   }


   //check for framing
   std::vector<Hand*> chands;
   getCurrent(chands);

   if(chands.size()==2  && chands[0]->isInWindow()  && chands[1]->isInWindow()
         && !chands[0]->isDragging() && !chands[1]->isDragging()){
      for(uint frameind=0;frameind<frames.size();++frameind)
         if(frames[frameind].isFramed(chands[0]->getLatest().handpose.position,chands[1]->getLatest().handpose.position)){

         //now see if this is a framing gesture:
         std::vector<gatmo_point3D_t> h1pts,h2pts;
         std::vector<double> dists,angles;
         if(getFramingsteps(*chands[0],*chands[1],frames[frameind],h1pts,h2pts) > 3){
            bool zoomin=true,zoomout=true,rotup=true,rotdown=true;
            for (int i = 0; i < 4; ++i) {
               dists.push_back(h1pts[i].dist(h2pts[i]));
               angles.push_back(atan2(h2pts[i].y-h1pts[i].y,h2pts[i].x-h1pts[i].x));
               if(i>0){
                  if(dists[i]>dists[i-1]){  zoomout=false;  }
                  if(dists[i]<dists[i-1]){ zoomin=false;  }
                  if(angles[i]>angles[i-1]){ rotdown=false;   }
                  if(angles[i]<angles[i-1]){ rotup=false; }
               }
            }
            //get dot product of movements to see if they were coherent:
            //compare relative movements - should always be close to -1
            double dot0 = (h1pts[0]-h1pts[4]).dot(h2pts[0]-h2pts[4])/((h1pts[0]-h1pts[4]).mag()*(h2pts[0]-h2pts[4]).mag());
            //compare hand1 to vector btwn hands should be close to +/- 1 for zoom, 0 for rot
            double dot1 = (h1pts[0]-h1pts[4]).dot(h1pts[4]-h2pts[4])/((h1pts[0]-h1pts[4]).mag()*(h1pts[4]-h2pts[4]).mag());
            //compare hand2 to vector btwn hands should be close to +/- 1 for zoom, 0 for rot
            double dot2 = (h2pts[0]-h2pts[4]).dot(h1pts[4]-h2pts[4])/((h2pts[0]-h2pts[4]).mag()*(h1pts[4]-h2pts[4]).mag());

            if(dot0 < -.85)
            if((zoomin || zoomout) && fabs(dists.front()-dists.back()) > 10.0  && fabs(dot1) > .9  && fabs(dot2) > .9 ){
               printf("zooming: %.01f  dots:  %.01f  %.01f  %.01f \n",dists.front()-dists.back(),dot0,dot1,dot2);
               frames[frameind].scale(dists.front()-dists.back());
            }
            if((rotup || rotdown) && fabs(angles.front()-angles.back())>.02 && fabs(dot1) < .3 && fabs(dot2) < .3){
               printf("rotating: %.03f  dots:  %.01f  %.01f  %.01f \n",angles.front()-angles.back(),dot0,dot1,dot2);
               gatmo_quaternion_t trot; trot.fromEuler(0.0,(angles.front()-angles.back()),0.0);
               frames[frameind].rotate(trot);
            }
         }
         break; //only frame one FrameObject
      }//if a frame is framed
   }//if 2 hands are current

   //check for sweeping:
   std::vector<gatmo_point3D_t> winpts;
   for(uint j=0;j<hands.size();++j)
      if(hands[j].isCurrent(laststamp))
         if(hands[j].isSweeping(winpts)){
            double rad=50.0*winpts.size();
            for(uint i=0;i<frames.size();++i){
               frames[i].sweepCheck(winpts.back().y,rad);
            }
         }

   //now look at the effects of this and other sweeps:
   for(uint j=0;j<hands.size();++j)
      if(hands[j].isCurrent(laststamp) && hands[j].held && hands[j].held->isSweeping())
         hands[j].held=NULL;

//  remove any frames that have flown far away
   uint i=0;
   while(i<frames.size()){
      if(frames[i].shouldDelete())
         frames.erase(frames.begin()+i);
      else
         i++;
   }
} //end MinorityReportDisplay::getGestures


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief add the new dataframe of handstates and associate them with the hands we are tracking
 *  \param h HandStates object, with hand detections
  */
void MinorityReportDisplay::addHand(HandStates &h){
   boost::mutex::scoped_lock  lock(hand_mutex);
   for(uint i=0;i<h.hands.size();i++) transformHand(h.hands[i]);
   laststamp=h.stamp;

   std::vector<int> notmatched(hands.size(),1);
   for(uint i=0;i<h.hands.size();i++){
      if(h.hands[i].handpose.position == gatmo_point3D_t()) continue; //filter out null poses
      bool foundmatch=false;
      for(uint currenth=0;currenth<hands.size();currenth++){
         if(notmatched[currenth] && hands[currenth].isMatch(h.hands[i])){
            hands[currenth].add(h.hands[i]);
            notmatched[currenth]=0;
            foundmatch=true;
            break;
         }
      }
      if(!foundmatch){
         hands.push_back(Hand(h.hands[i]));
         notmatched.push_back(0);
      }

   }

   //now cleanup:
   uint i=0;
   while(i<hands.size()){
      if(hands[i].isOld(laststamp))
         hands.erase(hands.begin()+i);
      else
         i++;
   }

   latesthands=h.hands;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Constructor - initializes all the ties to the global displayList
 *  \param parent - wxWindow that this display inherits from
  */
MinorityReportDisplay::MinorityReportDisplay(wxWindow * parent):GatmoDisplayInterface(parent){

	//GatmoDisplayObject part:
	//should always do this
	sprintf(name,"Sample plugin");
	display=false;
	emph=false;

	gui_debug=false;
	//setup gui display
	InitGui();
	incomingrate=0.0;
	//add to the global display list
	AddtoDisplayList();
	lastgestureupdate=0.0;
   display=true;
}

void MinorityReportDisplay::UpdateIncoming(){

   for(uint i=0;i<hands.size();++i)
         if(hands[i].isCurrent(laststamp)){
            double d=hands[i].isCircleGesture();
            if(fabs(d)>.01)
               incomingrate+=d*.0001;
            incomingrate=std::min(incomingrate,.03);
            incomingrate=std::max(incomingrate,-.00005);

         }

   for(uint i=0;i<incomingframes.size();++i)
      incomingframes[i].scoot(incomingrate);
   if(!incomingframes.size() || incomingframes.back().pose.position.x < .4){
      incomingframes.push_back(FrameObject(.04,.04,.5,.25,.8));
      incomingframes.back().gtextureload(imglib.getNext());
   }
   if(incomingframes.front().pose.position.x < -.8)
      incomingframes.pop_front();


}

//this is called whenever we have a new saved frame:
void MinorityReportDisplay::organizeSavedFrames(){
  savedframes.back().markSaved();
 
   if(savedframes.size()==1)
      savedframes.back().saveTo(gatmo_point3D_t(-.3,-.25,.8));
   else
      savedframes.back().saveTo(savedframes[savedframes.size()-2].pose.position + gatmo_point3D_t(0.05,.0,0));

}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Do openGL draws here - do to openGL pickiness with threading, this is the only function you should draw in.
  */
void MinorityReportDisplay::Draw(){

   glPushMatrix();
   glLoadIdentity();
   glTranslatef(0,0,-1.50);
   glColor4f(0.0,0.0,1.0,0.0);
   gatmo_pose6D_t gpose(0,0,0);

   boost::mutex::scoped_lock  lock(hand_mutex);

   //update the positions of frames that are coming across the top of the screen
   UpdateIncoming();

   //draw the various frame states
   for(uint i=0;i<incomingframes.size();++i)
      incomingframes[i].Draw();
   for(uint i=0;i<frames.size();++i)
      frames[i].Draw();
   for(uint i=0;i<savedframes.size();++i)
      savedframes[i].Draw();

   getGestures();

   //draw the hands
   for(uint i=0;i<hands.size();++i){
      if(hands[i].isCurrent(laststamp)){
         drawHand(hands[i].getLatest());

         if(gui_debug)  //draw all the previous positions of the hand
			 for(uint j=0;j<hands[i].history.size();++j)
				drawsphere(hands[i].history[j].handpose,.005,hands[i].history[j].fingers.size()<5);

         if(gui_debug)  //draw the average of the previous hand positions
        	 drawsphere(hands[i].getHistoryCenter(),.01);
      }
      else{
    	  if(gui_debug) //draw hands that we haven't seen in a while
    		  drawsphere(hands[i].project(laststamp),.1);
      }


   }
   glPopMatrix();


}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief initializes the gui panel functions associated with this display
  */
void MinorityReportDisplay::InitGui(){

   wxBoxSizer* v_sizer = new wxBoxSizer(wxVERTICAL);
   //add title
   wxStaticText *lbltitle = new wxStaticText(this, wxID_ANY, wxT("ROS Control"));
   v_sizer->Add(lbltitle,1,wxEXPAND | wxALIGN_CENTER | wxALL, 5 );

   //	//add button
   wxButton* optimizeb = new wxButton(this, wxID_ANY, wxT("Optimize"));
   Connect(optimizeb->GetId(), wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler(MinorityReportDisplay::OnOptimizeButton));
   wxBoxSizer* h_sizer_opt = new wxBoxSizer(wxHORIZONTAL);
   h_sizer_opt->Add(optimizeb,4,wxEXPAND | wxALIGN_CENTER);
   //create ROS control interface
   v_sizer->Add(GG_ROSGLOBALS::initRosManager(this),0);

   //This is the magic function:  you create a new object and pass in:
   //   The human readable name to display next to the button
   //  the ros topic
   //  Subscribe or advertise
   //  the function to call with the data you receive
   //  the 'this' pointer
   //  the function to convert the ROS message to which you are subscribing into the data you expect in the prev. function
   //Obviously, there is some spiffy template magic in the works...
   new GRosInterface("Hand in","/hands_pros",SUBSCRIBE,&MinorityReportDisplay::addHand,this,&convertHandsToGatmo);

   //wxwidgitness
   v_sizer->Add(h_sizer_opt,1,wxEXPAND | wxALIGN_CENTER);
   this->SetSizer(v_sizer);
   this->Show();
   frames.push_back(FrameObject(.04,.04,.1,.1,.8));
   frames.back().gtextureload(std::string(image_path).append("facekinect.jpg"));
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief callback when the Optimize button is clicked
  */
void MinorityReportDisplay::OnOptimizeButton(wxCommandEvent& event){
   frames.push_back(FrameObject(.02,.04,.1,-.1,.8));
   frames.push_back(FrameObject(.02,.04,-.1,-.1,.8));
   frames.push_back(FrameObject(.02,.04,0,-.1,.8));
}

