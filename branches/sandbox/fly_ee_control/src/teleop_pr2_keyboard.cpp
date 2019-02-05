/*
 * teleop_pr2_keyboard
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Kevin Watts

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "ee_cart_imped_control/StiffPoint.h"

#define KEYCODE_U 0x75
#define KEYCODE_M 0x6D
#define KEYCODE_H 0x68
#define KEYCODE_J 0x6A
#define KEYCODE_K 0x6B
#define KEYCODE_L 0x6C

#define KEYCODE_SPACE 0x20

#define KEYCODE_U_CAP 0x55
#define KEYCODE_M_CAP 0x4D
#define KEYCODE_H_CAP 0x48
#define KEYCODE_J_CAP 0x4A
#define KEYCODE_K_CAP 0x4B
#define KEYCODE_L_CAP 0x4C

class TeleopPR2Keyboard
{
  private:
  double walk_vel, run_vel, yaw_rate, yaw_rate_run;

  ee_cart_imped_control::StiffPoint msg;
  ros::NodeHandle n_;
  ros::Publisher pub_;

  public:
  void init()
  { 
    pub_ = n_.advertise<ee_cart_imped_control::StiffPoint>("ee_cart_imped_control/stiff_point", 1);
  }
  
  ~TeleopPR2Keyboard()   { }
  void keyboardLoop();

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_base_keyboard");

  TeleopPR2Keyboard tpk;
  tpk.init();

  signal(SIGINT,quit);

  tpk.keyboardLoop();

  return(0);
}

void TeleopPR2Keyboard::keyboardLoop()
{

  tf::TransformListener listener;


  char c;
  bool dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'HJKL' to translate in robot ZY plane");
  puts("Modifying direction along Y axis as if one were looking at the robot like a vi terminal");
  puts("Use 'UM' to go away and towards the robot");
  puts("Press 'Shift' to rotate (not yet implemented)");


  double trans_force = 1;
  tf::StampedTransform transform_;
  for(;;)
  {

    try {
        listener.lookupTransform("/base_link", "/r_gripper_tool_frame",  
                ros::Time(0), transform_);
    } catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    printf("transform x: %f y: %f z: %f\n", transform_.getOrigin().x(), transform_.getOrigin().y(), transform_.getOrigin().z());

    msg.pose.position.x = transform_.getOrigin().x();
    msg.pose.position.y = transform_.getOrigin().y();
    msg.pose.position.z = transform_.getOrigin().z();
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 0;
    msg.stiffness.position.x = 10;
    msg.stiffness.position.y = 10;
    msg.stiffness.position.z = 10;
    msg.stiffness.orientation.x = 0;
    msg.stiffness.orientation.y = 0;
    msg.stiffness.orientation.z = 0;
    msg.stiffness.orientation.w = 0;
    msg.wrench.position.x = 0;
    msg.wrench.position.y = 0;
    msg.wrench.position.z = 0;
    msg.wrench.orientation.x = 0.0001;
    msg.wrench.orientation.y = 0.0001;
    msg.wrench.orientation.z = 0.0001;
    msg.wrench.orientation.w = 0.0001;
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    switch(c)
    {
      // translating
    case KEYCODE_U:
      msg.wrench.position.x = trans_force;
      puts("+x");
      dirty = true;
      break;
    case KEYCODE_M:
      msg.wrench.position.x = -1 * trans_force;
      puts("-x");
      dirty = true;
      break;
    case KEYCODE_H:
      msg.wrench.position.y = -1 * trans_force;
      puts("-y");
      dirty = true;
      break;
    case KEYCODE_J:
      msg.wrench.position.z = -1 * trans_force;
      puts("-z");
      dirty = true;
      break;
    case KEYCODE_K:
      msg.wrench.position.z = trans_force;
      puts("+z");
      dirty = true;
      break;
    case KEYCODE_L:
      msg.wrench.position.y = trans_force;
      puts("+y");
      dirty = true;
      break;

    case KEYCODE_SPACE:
      puts("clear");
      dirty = true;
      break;
      // rotating 
    }

    
    if (dirty == true)
    {
      pub_.publish(msg);
    }


  }
}
