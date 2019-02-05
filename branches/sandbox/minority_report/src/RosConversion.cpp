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


#include "RosConversion.h"


void convertPoint3DToGatmo(gatmo_point3D_t &p, const geometry_msgs::Vector3 &rpoint){
	p.x=rpoint.x;
	p.y=rpoint.y;
	p.z=rpoint.z;
}
void convertPoint3DToGatmo(gatmo_point3D_t &p, const geometry_msgs::Point &rpoint){
	p.x=rpoint.x;
	p.y=rpoint.y;
	p.z=rpoint.z;
}
void convertQuaternionToGatmo(gatmo_quaternion_t &p, const geometry_msgs::Quaternion &q){
	p.x=q.x;
	p.y=q.y;
	p.z=q.z;
	p.w=q.w;
}

void convertPose6DToGatmo(gatmo_pose6D_t &p, const geometry_msgs::Transform &rpose){
	convertQuaternionToGatmo(p.orientation,rpose.rotation);
	convertPoint3DToGatmo(p.position,rpose.translation);
}

void convertHandToGatmo(HandState &p, const body_msgs::Hand &h){
	p.fingers.resize(h.fingers.size());
	for(uint i=0;i<h.fingers.size();i++)
		convertPoint3DToGatmo(p.fingers[i],h.fingers[i]);
	convertPose6DToGatmo(p.handpose,h.palm);
	p.thumb=h.thumb;
	p.stamp=h.stamp.toSec();
	p.state=h.state;
	convertPoint3DToGatmo(p.arm,h.arm);
}

void convertHandsToGatmo(HandStates &p, const body_msgs::Hands &h){
   p.stamp=h.header.stamp.toSec();
   p.hands.resize(h.hands.size());
   for(uint i=0;i<h.hands.size();++i)
      convertHandToGatmo(p.hands[i],h.hands[i]);
}



