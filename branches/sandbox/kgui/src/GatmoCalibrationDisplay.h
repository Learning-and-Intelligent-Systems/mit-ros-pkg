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


#ifndef GATMOCALIBRATIONDISPLAY_H_
#define GATMOCALIBRATIONDISPLAY_H_
#include "globals.h"

#include <deque>

#include <fstream>
#include "imagepath.h"
#include "RosGlobals.h"
#include "RosConversion.h"

#include "FrameObject.h"


// a class to maintain a list of images to use
class ImageLibrary{
   std::vector<std::string> filenames;
   int pos;
public:
   ImageLibrary(std::string filelist="imagestoload.txt"){
      pos=0;

      std::ifstream infile((std::string(image_path).append(filelist)).c_str());
      std::string temp;
       while(infile.good() && !infile.eof()){
         infile>>temp;
         filenames.push_back(std::string(image_path).append(temp));
       }
       std::cout<<"loaded "<<filenames.size()<<"from "<<(std::string("image_path").append(filelist)).c_str()<<std::endl;
       infile.close();
   }

   std::string getNext(){
      std::string ret = filenames[pos++];
      pos=pos%(filenames.size());
      return ret;
   }

};








//class Frameobj{
//public:
//      gatmo_pose6D_t pose;
//      double width, height;
//      bool highlighted;
//      bool framed;
//      gatmo_quaternion_t sweeprot;
//      bool sweeping; //indicator that the frame is flying away
//
//      Frameobj(double w, double h, double x, double y, double z):pose(x,y,z){
//         width=w;
//         height=h;
//         highlighted=false;
//         framed=false;
//         sweeping=false;
//      }
//   gatmo_point3D_t getWinPos(){
//         return getVPCoords(pose.position);
//   }
//
//   //there has been a sweep action. If this frame is within rad pixels of the sweep, send it into sweep mode
//   void sweepCheck(double winy, double rad){
//      if(fabs(getVPCoords(pose.position).y-winy)<rad){
//         sweeping=true;
////         sweeprot.generate(); //generate a random quaternion to rotate with
//         sweeprot.fromEuler(0.0,(((double)(rand()%100))/500.0-.1),0.0);
//      }
//
//   }
//
//
//   //indicate that the give pose (in window coords) is in a quadrant diagonal from frame
//   //returns: the quadrant number -> 1: top right,  2: top left, 3: bottom left, 4 bottom right
//   int isInCorner(gatmo_point3D_t p){
//      if(sweeping) return 0;
//      //TODO: add a transform so this works even if the frame is rotated
//
//      gatmo_point3D_t temp(width/2.0,height/2.0,0.0);
//      gatmo_point3D_t upperwin = getVPCoords(pose.position+temp);
//      gatmo_point3D_t lowerwin = getVPCoords(pose.position-temp);
//      if(p.x > upperwin.x){
//         if(p.y < lowerwin.y)
//            return 4;
//         if(p.y > upperwin.y)
//            return 1;
//      }
//      if(p.x < lowerwin.x){
//         if(p.y < lowerwin.y)
//            return 3;
//         if(p.y > upperwin.y)
//            return 2;
//      }
//      return 0;
//   }
//
//   //indicate that the give line (in window coords) goes close to this object
//   bool isCloseLine(gatmo_line3d_t &l){
//      if(sweeping) return false;
//      //TODO: add a transform so this works even if the frame is rotated
//      gatmo_point3D_t temp(width/2.0,height/2.0,0.0);
//      gatmo_point3D_t vppt = getVPCoords(pose.position);
//      vppt.z=0;
//      gatmo_point3D_t upperwin = getVPCoords(pose.position+temp);
//      gatmo_point3D_t lowerwin = getVPCoords(pose.position-temp);
//      if(l.distto(vppt) < lowerwin.dist(upperwin)/2.0)
//         return true;
//      return false;
//   }
//
//
//   bool isOn(gatmo_pose6D_t p){
//      if(sweeping) return false;
//	   gatmo_point3D_t temp(width/2.0,height/2.0,0.0);
//	   gatmo_point3D_t upperwin = getVPCoords(pose.position+temp);
//	   gatmo_point3D_t lowerwin = getVPCoords(pose.position-temp);
//	   gatmo_point3D_t hwin = getVPCoords(p.position);
//	   if(hwin.x > lowerwin.x && hwin.x < upperwin.x && hwin.y > lowerwin.y && hwin.y < upperwin.y){
//
//////	      std::cout<<"hwin: "<<hwin.x<<" "<<hwin.y<<std::endl;
////      gatmo_point3D_t diff=hwin-cwin;
//////      double zdiff=diff.z;
//////      diff.z=0;
////
//////      gatmo_point3D_t diff=p.position-pose.position;
////      std::cout<<"diff: "<<diff.x<<" "<<diff.y<<std::endl;
////      if(fabs(diff.x)<width/2.0 && fabs(diff.y)<height/2.0){
////    	 std::cout<<"isOn:   zdiff: "<<hwin.z-lowerwin.z<<std::endl;
//
//         return true;
//      }
//      return false;
//   }
//
//   void update(gatmo_point3D_t pold, gatmo_point3D_t pnew){
//      if(sweeping) return;
//      pose.position=Updatepose(pose.position,pold,pnew);
//   }
//
//   void Draw(){
//      if(sweeping){
//         pose.orientation=pose.orientation*sweeprot;
//         pose.position.x-=.01;
//
//      }
//      double _tcam[16];
//         glDisable( GL_TEXTURE_2D );
//         glPushMatrix();
//         glLoadIdentity();
//         glTranslatef(0,0,-2.00);
//         pose.toOpenGLMatrix(_tcam);
//         glMultMatrixd(_tcam);
//
//         glColor3f(0.0f,0.0f,1.0f);
//         if(highlighted)
//            glColor3f(1.0f,1.0f,0.0f);
//         if(framed)
//            glColor3f(0.0f,1.0f,0.0f);
//
//      glBegin(GL_QUADS);
//      glVertex3f(-width/2.0,height/2.0,0.0f);
//      glVertex3f(width/2.0,height/2.0,0.0f);
//      glVertex3f(width/2.0,-height/2.0,0.0f);
//      glVertex3f(-width/2.0,-height/2.0,0.0f);
//      glEnd();
//
//      glPopMatrix();
//      glEnable( GL_TEXTURE_2D );
//
//
//   }
//
////   an indicator if this  frame should be removed
//   bool shouldDelete(){
//      return sweeping &&  pose.position.x < -5.0;
//
//   }
//
//};


class Hand{
public:
   std::deque <HandState> history;
   std::deque <gatmo_point3D_t> vhistory;
   gatmo_point3D_t instantvel,avevel;
   bool clicking;
   FrameObject* held;

   bool isOld(double t){
//      std::cout<<" isold: "<<t-history.back().stamp;
      if(t-history.back().stamp > .10){
//         std::cout<<" isold: "<<t-history.back().stamp;
         return true;
      }
      return false;
   }
   bool isCurrent(double t){
      if(t-history.back().stamp < .05)
         return true;
      return false;
   }
   HandState getLatest(){
      return history.back();
   }

   bool isInWindow(){
      gatmo_point3D_t pt = getWinPos();
      if( pt.x < 0 || pt.x > GG_MAIN::window_width ||
            pt.y <0 || pt.y > GG_MAIN::window_height)
         return false;
      return true;

   }

   gatmo_point3D_t getWinPos(int ind=-1){
      if(ind <0 || ind >= (int)history.size())
         return getVPCoords(history.back().handpose.position);
      return getVPCoords(history[ind].handpose.position);
   }

   //ave of prev positions
   gatmo_point3D_t getHistoryCenter(){
      gatmo_point3D_t out;
      for(int i=0; i<history.size();++i)
         out+=history[i].handpose.position;
      return out/history.size();
   }

   //get closest distance to the ave of prev positions
   double getDistFromCenter(){
      gatmo_point3D_t c=getHistoryCenter();
      double dist;
      for(int i=0; i<history.size();++i){
         if(i==0 || c.dist(history[i].handpose.position) <dist){
            dist=c.dist(history[i].handpose.position);
         }
      }
      return dist;
   }

   Hand(HandState h){
      history.push_back(h);
      held=NULL;
   }
   gatmo_point3D_t project(double stamp){
      double tdiff=stamp-history.back().stamp;
      return history.back().handpose.position+instantvel*tdiff;
   }

   bool isMatch(HandState h){
//      if(h.stamp==history.back().stamp){
//         printf(" ismatch checking timestamp %.03f, %d\n",h.stamp-1290820000.0, history.size());
////         std::cout<<" ismatch checking timestamp "<<h.stamp<<std::endl;
//      }
//      double tdiff=h.stamp-history.back().stamp;
      gatmo_point3D_t diff=h.handpose.position-(history.back().handpose.position);//+instantvel*tdiff);
      if(diff.mag() < .1)
         return true;
      return false;
   }

   double maxSpeed(){
      double v;
      for(int i=0; i<vhistory.size();++i){
         if(i==0 || vhistory[i].mag() <v){
            v=vhistory[i].mag();
         }
      }
      return v;
   }
   double circularness(){
      gatmo_point3D_t c=getHistoryCenter();
      double v=0;
      for(int i=0; i<history.size();++i){
         if((c-history[i].handpose.position).mag()<.0001 || vhistory[i].mag() < .0001) return 0.0;
         v+=vhistory[i].cross(c-history[i].handpose.position).z/(vhistory[i].mag()*(c-history[i].handpose.position).mag());
      }
      return v/history.size();
   }

   double isCircleGesture(){
      if(instantvel.mag() > .05 && fabs(getDistFromCenter()*circularness()) > .028)
         return circularness();
      return 0.0;
   }

   void add(HandState h){
      gatmo_point3D_t pdiff=h.handpose.position-history.back().handpose.position;
      gatmo_point3D_t pold=history.back().handpose.position;
      instantvel=pdiff/(h.stamp-history.back().stamp);
//      std::cout<<"  vel:  "<<instantvel.mag()<<std::endl;
      if(history.back().state=="open"  && h.state=="closed")
//      if(history.back().fingers.size()==5 && h.fingers.size()<5 && instantvel.mag() < .1)
         clicking=true;
      else
         clicking=false;

      history.push_back(h);
      vhistory.push_back(instantvel);

      if(history.size()>20){
         history.pop_front();
         vhistory.pop_front();
      }
//      if(instantvel.mag() > .05){
//         printf("  closestdist:  %.03f   circ:  %.03f",getDistFromCenter(),circularness());
//         if(fabs(getDistFromCenter()*circularness()) > .033)
//            std::cout<<"      CIRCLE"<<std::endl;
//         else
//            std::cout<<"     "<<std::endl;
//
//      }
//            std::cout<<"  closestdist:  "<<getDistFromCenter()/ maxSpeed()<<std::endl;

   }

   //if we are dragging an object, update the position of the object
   //this has to be called from within the Draw function, so the matrices are valid
   void updateDragging(){
      //if we are dragging around an object:
      if(held && history.size()>1){
         if(history.back().fingers.size()==5)//if releasing
            held=NULL;
         else{
            held->update(history[history.size()-2].handpose.position,history.back().handpose.position); //pass old, new position of hand
         }
      }

   }

   bool isDragging(){
      return held != NULL;
   }

   //determine if this hand is performing a sweep gesture
   bool isSweeping(std::vector<gatmo_point3D_t> &pts){
      pts.clear();
      double vthresh=-.8;
      if(instantvel.x<vthresh){
         int streak=0;
         for(int i=vhistory.size()-1;i>=0;i--)
            if(vhistory[i].x<vthresh){
               streak++;
               pts.push_back(getVPCoords(history[i].handpose.position));
            }
            else
               break;
         if(streak>1)
            std::cout<<"sweep duration "<<streak<<std::endl;
         if(streak>3)
            return true;
      }
      return false;
   }

   //determine if this hand is performing a sweep gesture
   bool isSaving(){
      if(!held) return false;
      double vthresh=.8;
      if(instantvel.x>vthresh){
         int streak=0;
         for(int i=vhistory.size()-1;i>=0;i--)
            if(vhistory[i].x>vthresh){
               streak++;
//               pts.push_back(getVPCoords(history[i].handpose.position));
            }
            else
               break;
         if(streak>1)
            std::cout<<"save duration "<<streak<<std::endl;
         if(streak>3)
            return true;
      }
      return false;
   }


};


class GatmoCalibrationDisplay : public GatmoDisplayInterface
{
	private:
	bool gui_debug;
   ImageLibrary imglib;
   double laststamp,lastgestureupdate; //time of last message
      boost::mutex hand_mutex;
	    //features of the gui:
		//only need to declare things here that you will need to reference explicitly other places
		//for example to set the value dynamically
      void getGestures();
      void getCurrent(std::vector<Hand*> &chands){
         chands.clear();
         for (uint i = 0; i < hands.size(); ++i) {
            if(hands[i].isCurrent(laststamp))
               chands.push_back(&hands[i]);
         }
      }
		std::vector<HandState> latesthands;
      std::vector<Hand> hands;
      std::vector<FrameObject> frames;
      std::vector<FrameObject> savedframes;
      double incomingrate;
      std::deque<FrameObject> incomingframes;
		//gui callback functions:
		void OnOptimizeButton(wxCommandEvent& event);
		void UpdateIncoming();
		void organizeSavedFrames();
	public:
		//have to have these functions:
	    void InitGui();
	    void UpdateDisplay(){}  //do the wx panel updates here
		void Draw();
		void DrawAlt(){ Draw(); }
		void addHand(HandStates &h);


		//have to have a constructor that passes the parent window pointer to the underlying panel
		GatmoCalibrationDisplay(wxWindow*);
	virtual ~GatmoCalibrationDisplay(){}
};




#endif /*GATMOCALIBRATIONDISPLAY_H_*/
