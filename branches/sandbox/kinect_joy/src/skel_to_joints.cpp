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


#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

#include <ros/ros.h>
#include <body_msgs/Skeletons.h>
#include <body_msgs/Hands.h>
#include <Eigen3/Geometry>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point32.h>
#include <pr2_controllers_msgs/JointTrajectoryGoal.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <sensor_msgs/JointState.h>
#include <pr2_msgs/SetPeriodicCmd.h>
#include <geometry_msgs/Pose.h>
#include <urdf/model.h>
#include <actionlib/client/simple_action_client.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_common_action_msgs/TuckArmsAction.h>
#include <pr2_msgs/PowerBoardState.h>

static const double GRIPPER_CLOSE_POSITION = 0.000;
static const double GRIPPER_CLOSE_MAX_EFFORT = 10000.0;

static const double GRIPPER_OPEN_POSITION = 0.086;
static const double GRIPPER_OPEN_MAX_EFFORT = 10000.0;

void flipvec(Eigen3::Vector4f palm, Eigen3::Vector4f fcentroid,Eigen3::Vector4f &dir ){
   if((fcentroid-palm).dot(dir) <0)
      dir=dir*-1.0;

 }


geometry_msgs::Point32 eigenToMsgPoint32(Eigen3::Vector4f v){
	geometry_msgs::Point32 p;
	p.x=v(0); p.y=v(1); p.z=v(2);
	return p;
}
geometry_msgs::Point eigenToMsgPoint(Eigen3::Vector4f v){
	geometry_msgs::Point p;
	p.x=v(0); p.y=v(1); p.z=v(2);
	return p;
}

Eigen3::Vector3f jointToEigen(body_msgs::SkeletonJoint &j){
   Eigen3::Vector3f out;
   out(0)=j.position.x;
   out(1)=j.position.y;
   out(2)=j.position.z;
   return out;
}

float sign(float in){ return in>0?1.0:-1.0;}

void getLArmJoints(body_msgs::Skeleton skel,std::vector<float> &joints){
   Eigen3::Vector3f lshoulder=jointToEigen(skel.left_shoulder);
   Eigen3::Vector3f rshoulder=jointToEigen(skel.right_shoulder);
   Eigen3::Vector3f torso=jointToEigen(skel.torso);
   Eigen3::Vector3f lelbow=jointToEigen(skel.left_elbow);
   Eigen3::Vector3f lhand=jointToEigen(skel.left_hand);
   Eigen3::Vector3f v1,v2,v3,va,v3bar,v3s,v4,v34;
   v2=lshoulder-rshoulder;
   v1=lshoulder-torso;
   va=v1.cross(v2);
   v1=v2.cross(va);  //v1 in now perpendicular to v2, in the plane of the original v1 and v2
   v3=lelbow-lshoulder;
   v4=lhand-lelbow;
   v1.normalize();
   va.normalize();
   v2.normalize();
   v3.normalize();
   v4.normalize();
   v3bar=(Eigen3::Matrix3f::Identity()-va*va.transpose())*v3;
   double theta2=-acos(v3bar.dot(v2)/v3bar.norm()) * sign(v3bar.dot(v1));
   double theta1=-1.0*(atan2(va.dot(v3),v3bar.norm())-1.2); //would subtract 1.5, but cannot make skeleton do 1.5 forward
   v34=v3.cross(v4);  v34.normalize();
   double theta3 = -(acos(v34.dot(v1))* sign(v4.dot(v1)) -1.57);
   double theta4 = -(acos(v3.dot(v4)));//* sign(v34.dot(v2))* sign((v2.cross(v3).dot(v1))));
   joints.resize(7);
   joints[0]=theta1; joints[1]=theta2; joints[2]=theta3; joints[3]=theta4; joints[4]=0.0; joints[5]=0.0; joints[6]=0.0;
//   ROS_INFO("th1 = %.03f  th2= %.03f th3 = %.03f  th4= %.03f  ",theta1,theta2,theta3,theta4);
   ROS_INFO("v1 = %.03f   %.03f  %.03f ",v2(0),v2(1),v2(2));
   //if v1(0) < -.2 or > .2 go left or right
   //if v1(2) < -.3 or > .3 go forward or backward
   //if v2(2) < 0.0 or > .6 go right or left

}


void getBaseDir(body_msgs::Skeleton skel,double &vx, double &vy, double &vw){
   Eigen3::Vector3f lshoulder=jointToEigen(skel.left_shoulder);
      Eigen3::Vector3f rshoulder=jointToEigen(skel.right_shoulder);
      Eigen3::Vector3f torso=jointToEigen(skel.torso);
      Eigen3::Vector3f v1,v2,va;
         v2=lshoulder-rshoulder;
         v1=lshoulder-torso;
         va=v1.cross(v2);
         v1=v2.cross(va);  //v1 in now perpendicular to v2, in the plane of the original v1 and v2

         v1.normalize();
         va.normalize();
         v2.normalize();

   //if v1(0) < -.2 or > .2 go left or right
   //if v1(2) < -.3 or > .3 go forward or backward
   //if v2(2) < 0.0 or > .6 go right or left
   vx=0;vy=0;vw=0;
   if(v1(0)<-.2) vy=.2;
   if(v1(0)>.2) vy=-.2;
   if(v1(2)<-.4) vx=.2;
   if(v1(2)>.1) vx=-.2;
   if(v2(2)<-.3) vw=-.2;
   if(v2(2)>.4) vw=.2;

}



void getRArmJoints(body_msgs::Skeleton skel,std::vector<float> &joints){
   Eigen3::Vector3f rshoulder=jointToEigen(skel.right_shoulder);
   Eigen3::Vector3f lshoulder=jointToEigen(skel.left_shoulder);
   Eigen3::Vector3f torso=jointToEigen(skel.torso);
   Eigen3::Vector3f relbow=jointToEigen(skel.right_elbow);
   Eigen3::Vector3f rhand=jointToEigen(skel.right_hand);
   Eigen3::Vector3f v1,v2,v3,va,v3bar,v3s,v4,v34;
   v2=rshoulder-lshoulder;
   v1=rshoulder-torso;
   va=v1.cross(v2);
   v1=v2.cross(va);  //v1 in now perpendicular to v2, in the plane of the original v1 and v2
   v3=relbow-rshoulder;
   v4=rhand-relbow;
   v1.normalize();
   va.normalize();
   v2.normalize();
   v3.normalize();
   v4.normalize();
   v3bar=(Eigen3::Matrix3f::Identity()-va*va.transpose())*v3;
   double theta2=-acos(v3bar.dot(v2)/v3bar.norm()) * sign(v3bar.dot(v1));
   double theta1=-1.0*(atan2(va.dot(v3),v3bar.norm())+1.2); //would subtract 1.5, but cannot make skeleton do 1.5 forward
   v34=v3.cross(v4);  v34.normalize();
   double theta3 = -(acos(v34.dot(v1)) -1.57);
   double theta4 = -(acos(v3.dot(v4)));//* sign(v34.dot(v2))* sign((v2.cross(v3).dot(v1))));
   joints.resize(7);
   joints[0]=theta1; joints[1]=theta2; joints[2]=theta3; joints[3]=theta4; joints[4]=0.0; joints[5]=0.0; joints[6]=0.0;
//   ROS_INFO("th1 = %.03f  th2= %.03f th3 = %.03f  th4= %.03f  ",theta1,theta2,theta3,theta4);

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief @b HandDetector is the main ROS communication class, and its function is just to tye things together.
 * \author Garratt Gallagher
 */
class SkelConverter
{

private:
  ros::NodeHandle n_;
  ros::Subscriber skelsub_,sub_;
  bool lclosed,rclosed;
  ros::Publisher left_arm_traj_pub_;
  ros::Publisher right_arm_traj_pub_, base_pub_;
  std::deque<std::vector<float> > ljoints, rjoints;

  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>* right_gripper_client_;
  actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>* left_gripper_client_;
  float angleset;
public:

  SkelConverter()
  {
     sub_=n_.subscribe("/hands", 1, &SkelConverter::handscb, this);
    base_pub_ = n_.advertise<geometry_msgs::Twist>("base_controller/command", 1);
   skelsub_=n_.subscribe("/skeletons", 1, &SkelConverter::skelcb, this);
   left_arm_traj_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>("/l_arm_controller/command", 1);
   right_arm_traj_pub_ = n_.advertise<trajectory_msgs::JointTrajectory>("/r_arm_controller/command", 1);
   angleset=-1.0;
   lclosed=rclosed=true;
   right_gripper_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>("r_gripper_controller/gripper_action", true);
   left_gripper_client_ = new actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction>("l_gripper_controller/gripper_action", true);

  }
  void sendBaseCMD(double vx, double vy, double vw){
     geometry_msgs::Twist cmd;

     cmd.linear.x = vx;
     cmd.linear.y = vy;
     cmd.angular.z = vw;
     base_pub_.publish(cmd);
  }

  void sendLJointCommand(std::vector<float> joints){
     pr2_controllers_msgs::JointTrajectoryGoal goal;

     std::vector<std::string> joint_names;
     std::string pref = "l";

     // First, the joint names, which apply to all waypoints
     joint_names.push_back(pref+"_"+"shoulder_pan_joint");
     joint_names.push_back(pref+"_"+"shoulder_lift_joint");
     joint_names.push_back(pref+"_"+"upper_arm_roll_joint");
     joint_names.push_back(pref+"_"+"elbow_flex_joint");
     joint_names.push_back(pref+"_"+"forearm_roll_joint");
     joint_names.push_back(pref+"_"+"wrist_flex_joint");
     joint_names.push_back(pref+"_"+"wrist_roll_joint");

     goal.trajectory.joint_names = joint_names;

     // We will have two waypoints in this goal trajectory
     goal.trajectory.points.resize(1);

     for(unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++) {
       goal.trajectory.points[0].positions.push_back(joints[i]);
       goal.trajectory.points[0].velocities.push_back(0.0);
     }
     goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.200);
     goal.trajectory.points[0].time_from_start = ros::Duration(.2);

     left_arm_traj_pub_.publish(goal.trajectory);

  }
  void sendRJointCommand(std::vector<float> joints){
     pr2_controllers_msgs::JointTrajectoryGoal goal;

     std::vector<std::string> joint_names;
     std::string pref = "r";

     // First, the joint names, which apply to all waypoints
     joint_names.push_back(pref+"_"+"shoulder_pan_joint");
     joint_names.push_back(pref+"_"+"shoulder_lift_joint");
     joint_names.push_back(pref+"_"+"upper_arm_roll_joint");
     joint_names.push_back(pref+"_"+"elbow_flex_joint");
     joint_names.push_back(pref+"_"+"forearm_roll_joint");
     joint_names.push_back(pref+"_"+"wrist_flex_joint");
     joint_names.push_back(pref+"_"+"wrist_roll_joint");

     goal.trajectory.joint_names = joint_names;

     // We will have two waypoints in this goal trajectory
     goal.trajectory.points.resize(1);

     for(unsigned int i = 0; i < goal.trajectory.joint_names.size(); i++) {
        std::cout<<joints[i]<<" ";
       goal.trajectory.points[0].positions.push_back(joints[i]);
       goal.trajectory.points[0].velocities.push_back(0.0);
     }
     std::cout<<std::endl;
     goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(0.200);
     goal.trajectory.points[0].time_from_start = ros::Duration(.2);

     right_arm_traj_pub_.publish(goal.trajectory);

  }

  void sendFilteredArms(){
     std::vector<float> joints(7,0.0);
     for(int j=0;j<7;j++){
        for(uint i=0;i<ljoints.size();i++)
              joints[j]+=ljoints[i][j];
        joints[j]/=((float)ljoints.size());
     }
     sendLJointCommand(joints);

     for(int j=0;j<7;j++){
        joints[j]=0.0;
        for(uint i=0;i<rjoints.size();i++)
              joints[j]+=rjoints[i][j];
        joints[j]/=((float)rjoints.size());
     }
     sendRJointCommand(joints);
     if(ljoints.size() > 10)
        ljoints.pop_front();
     if(rjoints.size() > 10)
        rjoints.pop_front();

  }

  void skelcb(const body_msgs::SkeletonsConstPtr &skels){
//     skelmsg=*skels;
//     printf("skel callback tdiff = %.04f \n",(skelmsg.header.stamp-pcloudmsg.header.stamp).toSec());
        if(skels->skeletons.size()==0) return;
        std::vector<float> joints(7,0.0);
//        joints[1]=angleset;
//        angleset +=.01;
        getLArmJoints(skels->skeletons[0],joints);
        ljoints.push_back(joints);
        sendLJointCommand(joints);
        getRArmJoints(skels->skeletons[0],joints);
        rjoints.push_back(joints);

        sendFilteredArms();
//        sendRJointCommand(joints);
//        sendLJointCommand(joints);


        double vx=0,vy=0,vw=0;
        getBaseDir(skels->skeletons[0],vx,vy,vw);
        sendBaseCMD(vx,  vy,  vw);
  }



  void ProcessHand(const body_msgs::Hand &hand){
     ROS_INFO("got hand ");
     pr2_controllers_msgs::Pr2GripperCommandGoal com;
     if(hand.state=="closed"){
        com.command.position = GRIPPER_CLOSE_POSITION;
        com.command.max_effort = GRIPPER_CLOSE_MAX_EFFORT;
        ROS_INFO("sending command close ");
     }
     if(hand.state=="open"){
        com.command.position = GRIPPER_OPEN_POSITION;
        com.command.max_effort = GRIPPER_CLOSE_MAX_EFFORT;
        ROS_INFO("sending command open ");
     }


     if(hand.left) left_gripper_client_->sendGoal(com);
     else right_gripper_client_->sendGoal(com);

  }

  void handscb(const body_msgs::HandsConstPtr &hands){

        for(uint i=0;i<hands->hands.size();i++){
           ProcessHand(hands->hands[i]);
        }
  }

} ;




int main(int argc, char **argv)
{
  ros::init(argc, argv, "skelconverter");
  ros::NodeHandle n;
  SkelConverter detector;
  ros::spin();
  return 0;
}
