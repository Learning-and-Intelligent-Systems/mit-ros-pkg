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


#ifndef GatmoTexture_H_
#define GatmoTexture_H_


#include <Magick++.h>
#include "globals.h"


#include "gatmounits.h"

gatmo_point3D_t getVPCoords(gatmo_point3D_t pt);
gatmo_point3D_t getWorldCoords(gatmo_point3D_t pt);
gatmo_point3D_t Updatepose(gatmo_point3D_t obj, gatmo_point3D_t oldref, gatmo_point3D_t newref);


class FrameObject
{
   typedef enum FState { GRABBED, SELECTED, SWEEPING, FRAMED, PASSIVE};
	protected:
      int imgrows,imgcols;
      FState state;
	   GLuint texture;
		double width, height;
      double origwidth, origheight;
	   Magick::Blob blob;
      bool highlighted;
      bool framed;
      float frameintensity;
      float highlightedintensity;

      gatmo_quaternion_t sweeprot;
      bool sweeping; //indicator that the frame is flying away

      //this indicates that the texture is not just queued to load at the first draw
      //it is called this way because we need to load the mipmap in the openGL thread
      bool fullyloaded;

      //makes framing fade in and out
      void updateintensities(){
    	  float ffadein=.03, ffadeout=.2;
    	  float hfadein=.03, hfadeout=.2;

    	  if(framed && frameintensity < 1.0){
    		  frameintensity+=ffadein;
    		  frameintensity=std::min(frameintensity,1.0f);
    	  }
    	  if(!framed && frameintensity > 0.0){
    		  frameintensity-=ffadeout;
    		  frameintensity=std::max(frameintensity,0.0f);
    	  }

    	  if(highlighted && highlightedintensity < 1.0){
    		  highlightedintensity+=hfadein;
    		  highlightedintensity=std::min(highlightedintensity,1.0f);
    	  }
    	  if(!highlighted && highlightedintensity > 0.0){
    		  highlightedintensity-=hfadeout;
    		  highlightedintensity=std::max(highlightedintensity,0.0f);
    	  }

      }


	public:
      gatmo_pose6D_t pose;

      bool textureloaded;
//      FrameObject();
		void gtextureload(std::string filename);
	    void Draw();
       void DrawOutline();
	    void DrawSolid();
       void DrawTexture();



	    FrameObject(double w, double h, double x, double y, double z):pose(x,y,z){
	       fullyloaded=false;
	       textureloaded=false;
	         width=w;
	         height=h;
	         origheight=h;
	         origwidth=w;
	         highlighted=false;
	         framed=false;
	         sweeping=false;
	      }

	    //accessor to get the window coordinates
	    gatmo_point3D_t getWinPos(){     return getVPCoords(pose.position);    }

	    //there has been a sweep action. If this frame is within rad pixels of the sweep, send it into sweep mode
	    void sweepCheck(double winy, double rad);

	    //indicate that the give pose (in window coords) is in a quadrant diagonal from frame
	    //returns: the quadrant number -> 1: top right,  2: top left, 3: bottom left, 4 bottom right
	    int isInCorner(gatmo_point3D_t p);

	    //indicate that the give line (in window coords) goes close to this object
	    bool isCloseLine(gatmo_line3d_t &l);

	    //indicate whether a hand is on the object
	    bool isOn(gatmo_pose6D_t p);


       //detect if the object is framed by two hands
	    //h1 and h2 are points in the world coordinates representing the hand positions
       bool isFramed(gatmo_point3D_t h1, gatmo_point3D_t h2){
          gatmo_point3D_t win1 = getVPCoords(h1);
          gatmo_point3D_t win2 = getVPCoords(h2);
          win1.z=0;
          win2.z=0;
          gatmo_line3d_t gline(win1,win2);
          if(! isCloseLine(gline)) return false;
          framed=true;
          return true;
          int q1=isInCorner(win1), q2=isInCorner(win2);
          if(q1 && q2 && fabs(q1-q2) == 2){
             framed=true;
             return true;
          }

          return false;
       }
        

       void scale(double sfactor){
          //TODO: do something more intelligent here...
          height+=(sfactor)*.001;
          width+=(sfactor)*.001;
       }

       void rotate(gatmo_quaternion_t &rot){
          pose.orientation=pose.orientation*rot;
       }

       bool isSweeping(){return sweeping;}

	    //get moved by a drag
	    void update(gatmo_point3D_t pold, gatmo_point3D_t pnew);

	    // reset the state for a new timestamp
	    //the state may get set later by other functions
	    void resetToPassive(){
          //does not reset sweeping, which is permanent
	       if(sweeping) return;
          highlighted=false;
          framed=false;

	  }

	void markSaved(){
   highlighted=false;
          framed=false;
         frameintensity = 0.0;
         highlightedintensity = 0.0;


	}

	 //   an indicator if this  frame should be removed
	    bool shouldDelete(){  return sweeping &&  pose.position.x < -5.0;}

	    void scoot(double d){
	       pose.position.x-=d;
	    }

	    void saveTo(gatmo_point3D_t p){
	       width=origwidth;
	       height=origheight;
	       pose.orientation=gatmo_quaternion_t(); //reset orientation to straight up
	       pose.position=p;

	    }


	virtual ~FrameObject(){}
};


#endif /*GatmoTexture_H_*/
