#! /usr/bin/python
import roslib
roslib.load_manifest('lis_arm_controller')
import rospy

#load the params for the joint controllers onto the param server
class JointParams:
  def __init__(self, whicharm): #whicharm is 'r' or 'l'
    self.whicharm = whicharm
    self.default_p = [800., 800., 200., 100., 500., 400., 400.]
    self.default_d = [10., 10., 3., 3., 6., 4., 4.]
    joint_names = ['_shoulder_pan_joint', '_shoulder_lift_joint','_upper_arm_roll_joint', '_elbow_flex_joint','_forearm_roll_joint', '_wrist_flex_joint','_wrist_roll_joint']
    self.controller_name = whicharm+"_arm_controller"
    self.joint_names = [whicharm+joint_name for joint_name in joint_names]

  #set gains to some fraction of the default                                     
  def set_gains_fraction(self, fraction):
    for i in range(7):
      rospy.set_param(self.controller_name+'/gains/%s/p'%self.joint_names[i], self.default_p[i]*fraction)
      rospy.set_param(self.controller_name+'/gains/%s/d'%self.joint_names[i], self.default_d[i]*fraction)

