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



#include <ros/ros.h>
#include <body_msgs/Hands.h>
#include <algorithm>
#include <SFML/Audio.hpp>
#include <iostream>
#include <deque>

#include "notespath.h"


bool yIsLess (geometry_msgs::Point i,geometry_msgs::Point  j) { return (i.y<j.y); }

float maxDiff(geometry_msgs::Point i,geometry_msgs::Point  j){
	double m1= std::max( fabs(i.x-j.x),fabs(i.y-j.y));
	return std::max( m1,fabs(i.z-j.z));
}

void aveIn(geometry_msgs::Point &i,geometry_msgs::Point  j){
	i.x=(i.x+j.x)/2.0;
	i.y=(i.y+j.y)/2.0;
	i.z=(i.z+j.z)/2.0;
}

void aveIn(std::vector<geometry_msgs::Point> &v1, std::vector<geometry_msgs::Point>  v2){
	for(uint i=0;i<v1.size();++i)
		aveIn(v1[i],v2[i]);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b HandDetector is the main ROS communication class, This is a small program, so it does all the work.
 * \author Garratt Gallagher
 */
class Pianist
{

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  std::vector<geometry_msgs::Point> refs;
  std::vector<bool> fingerdown;
  sf::SoundBuffer Buffer[5];
  std::deque<sf::Sound> notes;
  ros::Time start_time;
  int initializing;

public:

  Pianist()
  {
    sub_=n_.subscribe("/hands_pros", 1, &Pianist::handcb, this);
    initializing=1;
    fingerdown.resize(5,false);
    const char *filenames[] = {"C.wav", "D.wav", "E.wav", "F.wav", "G.wav"};
    char tempname[500];
    for(int i=0;i<5;i++){
    	sprintf(tempname,"%s/%s",notes_path,filenames[i]);
    	if (!Buffer[i].LoadFromFile(tempname)){
    		ROS_ERROR("could not load file %s",filenames[i]);
    		exit(-1);
    	}
    }
	  std::cout<<"Hold Hand .7 meters from the kinect, fingers out, palm facing the sensor."<<std::endl;
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief calibrates the hand position, establishing a baseline
    * \param fingers current positions of the fingers
    */
  bool doneInitializing(std::vector<geometry_msgs::Point> &fingers){
	  if(!refs.size())
		  refs=fingers;
	  if(initializing < 10){
		  std::cout<<"Hold Still."<<std::endl;
		  bool similiar=true;
		  for(uint i=0;i<fingers.size();i++)
			  if(maxDiff(refs[i],fingers[i])>.004){
				  similiar=false;
				  break;
			  }
		  if(similiar){
			  aveIn(refs,fingers);
			  initializing++;
		  }
		  else{
			  initializing=2;
			  refs=fingers;
		  }
		  return false;
	  }
	  //normally, just re-average constantly
	  int similiar=0;
	  for(uint i=0;i<fingers.size();i++)
		  if(maxDiff(refs[i],fingers[i])>.01){
			  similiar++;
		  }
	  if(!similiar){
		  aveIn(refs,fingers);
		  initializing=15;
	  }
	  if(similiar==5)
		  initializing--;


	  return true;

  }

  /** \brief register a key hit */
  void hitkey(int k){
//	  std::cout<<"hit key "<<k<<std::endl;
	  notes.push_back(sf::Sound(Buffer[k]));
	  notes.back().Play();
	  fingerdown[k]=true;
  }

  /** \brief register that we have raised the finger back up */
  void unhitkey(int k){
//	  std::cout<<"unhit key "<<k<<std::endl;
	  fingerdown[k]=false;
  }
  /** \brief Garbage collect the finished notes */
  void cleanupSounds(){
	  while(notes.size() && notes.front().GetStatus() == sf::Sound::Stopped)
		  notes.pop_front();
  }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief The main callback.  Checks for 5 fingers, and if a baseline has been established
    * \param hands the hand message
    */
  void handcb(const body_msgs::HandsConstPtr &hands){

//     printf(" hands size %d \n ",hands->hands.size());
	  if(!hands->hands.size()) return;
//	  printf(" fingers size %d \n ",hands->hands[0].fingers.size());
	  if(hands->hands[0].fingers.size() !=5) return;
	  body_msgs::Hand hand=hands->hands[0];
	  //sort the fingers by y:
	  std::sort(hand.fingers.begin(),hand.fingers.end(),yIsLess);

	  if(!doneInitializing(hand.fingers)) return;

	  std::cout<<"fingers: ";
	  for(uint i=0;i<hand.fingers.size();i++){
//		  printf(" (%.03f, %.03f, %.03f) ",hand.fingers[i].x, hand.fingers[i].y, hand.fingers[i].z);
		  printf(" %.03f ", hand.fingers[i].z-refs[i].z);
		  if(hand.fingers[i].z-refs[i].z < -.02 && !fingerdown[i])
				hitkey(i);
		  if(hand.fingers[i].z-refs[i].z >= -.02 && fingerdown[i])
			  unhitkey(i);
	  }
	  std::cout<<std::endl;
	  cleanupSounds();
  }

} ;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "piano_player");
  ros::NodeHandle n;
  Pianist p;
  ros::spin();
  return 0;
}
