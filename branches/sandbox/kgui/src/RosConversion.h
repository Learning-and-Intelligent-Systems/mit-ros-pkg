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


#ifndef ROSCONVERSION_H_
#define ROSCONVERSION_H_


#include <geometry_msgs/Transform.h>
#include <body_msgs/Hand.h>
#include <body_msgs/Hands.h>

#include "gatmounits.h"


struct HandState{
   double stamp;
	std::vector<gatmo_point3D_t> fingers;
	gatmo_pose6D_t handpose;
	int thumb;
	std::string state;
	gatmo_point3D_t arm;
};

struct HandStates{
   double stamp;
   std::vector<HandState> hands;
};

void convertPoint3DToGatmo(gatmo_point3D_t &p, const geometry_msgs::Vector3 &rpoint);
void convertPoint3DToGatmo(gatmo_point3D_t &p, const geometry_msgs::Point &rpoint);
void convertQuaternionToGatmo(gatmo_quaternion_t &p, const geometry_msgs::Quaternion &q);
void convertPose6DToGatmo(gatmo_pose6D_t &p, const geometry_msgs::Transform &rpose);
void convertHandToGatmo(HandState &p, const body_msgs::Hand &h);
void convertHandsToGatmo(HandStates &p, const body_msgs::Hands &h);



#endif /* ROSCONVERSION_H_ */
