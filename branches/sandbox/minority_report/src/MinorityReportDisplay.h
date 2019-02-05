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


#ifndef MINORITYREPORTDISPLAY_H_
#define MINORITYREPORTDISPLAY_H_
#include "globals.h"

#include <deque>

#include <fstream>
#include "imagepath.h"
#include "RosGlobals.h"
#include "RosConversion.h"

#include "FrameObject.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b ImageLibrary Maintains a list of images to use.  Loads them in and maintains a position in the list
 * \author Garratt Gallagher
 */
class ImageLibrary{
   std::vector<std::string> filenames;
   int pos;
public:

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief Constructor, loads the images given a path
     * \param filelist path to text file which lists the images to load
     */
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

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief get the filename of the next image in the library.
     * \return The absolute path of the image file
     */
   std::string getNext(){
      std::string ret = filenames[pos++];
      pos=pos%(filenames.size());
      return ret;
   }

};



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b Hand Handles all the hand events, from matching to gesture recognition
 * \author Garratt Gallagher
 */
class Hand{
public:
   std::deque <HandState> history;
   std::deque <gatmo_point3D_t> vhistory;
   gatmo_point3D_t instantvel,avevel;
   bool clicking;
   FrameObject* held;

   Hand(HandState h){
      history.push_back(h);
      held=NULL;
   }
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief determine whether the hand was last seen more than .1 seconds ago
     * \return true if hand is > .1 seconds old
     */
   bool isOld(double t){
      if(t-history.back().stamp > .10){
         return true;
      }
      return false;
   }
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief determine whether the hand was last seen less than than .05 seconds ago
     * \return true if hand was last seen less than than .05 seconds ago
     */
   bool isCurrent(double t){
      if(t-history.back().stamp < .05)
         return true;
      return false;
   }

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief get the most recent hand state
     * \return most recent Hand state object associated with this hand
     */
   HandState getLatest(){
      return history.back();
   }

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief Determine if the hand is within the bounds of the viewing window
     * \return true if in the window
     */
   bool isInWindow(){
      gatmo_point3D_t pt = getWinPos();
      if( pt.x < 0 || pt.x > GG_MAIN::window_width ||
            pt.y <0 || pt.y > GG_MAIN::window_height)
         return false;
      return true;

   }
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief get the position of the hand in the viewing window, in window coordinates
     * \return 3d point representing x,y window coordinates, and z representing the odd OpenGL inverse distance from viewer
     */
   gatmo_point3D_t getWinPos(int ind=-1){
      if(ind <0 || ind >= (int)history.size())
         return getVPCoords(history.back().handpose.position);
      return getVPCoords(history[ind].handpose.position);
   }


   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief get an average of the previous 10 positions of the hand
    *  This is useful for detecting circular motions/gestures
     * \return 3d point representing average hand position in Kinect coordinate frame
     */
   gatmo_point3D_t getHistoryCenter(){
      gatmo_point3D_t out;
      for(uint i=0; i<history.size();++i)
         out+=history[i].handpose.position;
      return out/history.size();
   }


   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief predict where the hand will be next, based on it's previous velocity
     * \return 3d point representing predicted next hand position in Kinect coordinate frame
     */
   gatmo_point3D_t project(double stamp){
      double tdiff=stamp-history.back().stamp;
      return history.back().handpose.position+instantvel*tdiff;
   }


   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief Determine whether incoming handstate should be associated with this hand object
     * \return true if hand is a match
     */
   bool isMatch(HandState h){
      gatmo_point3D_t diff=h.handpose.position-(history.back().handpose.position);//+instantvel*tdiff);
      if(diff.mag() < .1)
         return true;
      return false;
   }



   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief Determine the history of this hand indicates a circle gesture
     * \return 0 if no circle gesture, or the circularity of the gesture if it is a circle gesture
     */
   double isCircleGesture(){
      if(instantvel.mag() > .05 && fabs(getDistFromCenter()*circularness()) > .028)
         return circularness();
      return 0.0;
   }


   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief Update this object with a new hand state
     */
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
   }

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief if we are dragging an object, update the position of the object
    * this has to be called from within the Draw function, so the matrices are valid
     * \return true if this hand is dragging a picture
     */
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

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief determine if this hand is dragging a picture around.
     * \return true if this hand is dragging a picture
     */
   bool isDragging(){
      return held != NULL;
   }

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief determine if this hand is performing a Sweep gesture
     * \return true if this hand is performing a Sweep gesture
     */
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

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief determine if this hand is performing a Save gesture
     * \return true if this hand is performing a Save gesture
     */
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

private:

   //get closest distance to the ave of prev positions
   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief get the distance from the current hand pose to an average of the previous 10 positions of the hand
    *  This is useful for detecting circular motions/gestures
     * \return float representing distance from the current hand pose to an average hand position in Kinect coordinate frame
     */
   double getDistFromCenter(){
      gatmo_point3D_t c=getHistoryCenter();
      double dist;
      for(uint i=0; i<history.size();++i){
         if(i==0 || c.dist(history[i].handpose.position) <dist){
            dist=c.dist(history[i].handpose.position);
         }
      }
      return dist;
   }

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief Reports maximum velocity seen in recent hand history
     * \return float representing the maximum velocity seen in recent history
     */
   double maxSpeed(){
      double v;
      for(uint i=0; i<vhistory.size();++i){
         if(i==0 || vhistory[i].mag() <v){
            v=vhistory[i].mag();
         }
      }
      return v;
   }

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   /** \brief Reports a metric on how circular
     * \return a float metric of how circular the hand motions was.
     */
   double circularness(){
      gatmo_point3D_t c=getHistoryCenter();
      double v=0;
      for(uint i=0; i<history.size();++i){
         if((c-history[i].handpose.position).mag()<.0001 || vhistory[i].mag() < .0001) return 0.0;
         v+=vhistory[i].cross(c-history[i].handpose.position).z/(vhistory[i].mag()*(c-history[i].handpose.position).mag());
      }
      return v/history.size();
   }


};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b Display object class that handles the minority report interface.
 * \author Garratt Gallagher
 */
class MinorityReportDisplay : public GatmoDisplayInterface
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
		MinorityReportDisplay(wxWindow*);
	virtual ~MinorityReportDisplay(){}
};




#endif /*MINORITYREPORTDISPLAY_H_*/
