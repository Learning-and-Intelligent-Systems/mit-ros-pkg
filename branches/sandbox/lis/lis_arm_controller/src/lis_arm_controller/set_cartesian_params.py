#! /usr/bin/python
import roslib
roslib.load_manifest('lis_arm_controller')
import rospy

#load the params for the Jtranspose with nullspace Cartesian controllers onto the param server
class JTNullspaceParams:
  def __init__(self, whicharm): #whicharm is 'r' or 'l'
    self.whicharm = whicharm
    self.root_name = "base_link"
    self.tip_name = self.whicharm+"_wrist_roll_link"
    self.k_posture = 25.0
    self.jacobian_inverse_damping = 0.01
    self.pose_command_filter = 0.01
    self.cart_gains_trans_p = 800.
    self.cart_gains_trans_d = 15.
    self.cart_gains_rot_p = 80.
    self.cart_gains_rot_d = 1.2
    self.joint_feedforward = [3.33, 1.16, 0.1, 0.25, 0.133, 0.0727, 0.0727]
    self.joint_max_effort = [11.88, 11.64, 6.143, 6.804, 8.376, 5.568, 5.568]
    self.vel_saturation_trans = 2.0
    self.vel_saturation_rot = 4.0
    self.name = self.whicharm+'_cart'
    #set the default values on the parameter server
    self.set_params()

  #set the current parameter values on the parameter server
  def set_params(self):
    rospy.loginfo("setting Cartesian with nullspace controller parameters")
    rospy.set_param(self.name+'/type', "JTTeleopController")
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')
    rospy.set_param(self.name+'')

#load the params for the Jtranspose Cartesian controllers onto the param server
class JTCartesianParams:
  def __init__(self, whicharm): #whicharm is 'r' or 'l'
    self.whicharm = whicharm
    self.max_vel_trans=.05
    self.max_vel_rot=.1
    self.max_acc_trans=0.2
    self.max_acc_rot=0.3
    self.pose_fb_trans_p=400.
    self.pose_fb_trans_d=6.
    self.pose_fb_rot_p=40.
    self.pose_fb_rot_d=0.
    self.tcname = self.whicharm+'_arm_cartesian_trajectory_controller'
    self.pcname = self.whicharm+'_arm_cartesian_pose_controller'
    self.tip_frame = self.whicharm+"_wrist_roll_link"
    self.base_frame = "base_link"

    #set the default values on the parameter server
    self.set_params()

  #set the parameter values back to their defaults
  def set_params_to_defaults(self, set_tip_frame = 1, set_params_on_server = 1):
    self.max_vel_trans=.15
    self.max_vel_rot=.3
    self.max_acc_trans=0.4
    self.max_acc_rot=0.6
    self.pose_fb_trans_p=2000.
    self.pose_fb_trans_d=30.
    self.pose_fb_rot_p=200.
    self.pose_fb_rot_d=0.
    if set_tip_frame:
      self.tip_frame = self.whicharm+"_wrist_roll_link"
    if set_params_on_server:
      self.set_params()

  #set the gains and max vels and accs lower, so the arm moves more gently/slowly
  def set_params_to_gentle(self, set_tip_frame = 1, set_params_on_server = 1):
    self.max_vel_trans=.05
    self.max_vel_rot=.1
    self.max_acc_trans=0.2
    self.max_acc_rot=0.3
    self.pose_fb_trans_p=400.
    self.pose_fb_trans_d=6.
    self.pose_fb_rot_p=40.
    self.pose_fb_rot_d=0.
    if set_tip_frame:
      self.tip_frame = self.whicharm+'_wrist_roll_link'
      rospy.loginfo("setting tip frame back to wrist_roll_link")
    if set_params_on_server:
      self.set_params()

  #set the current parameter values on the parameter server
  def set_params(self):
    rospy.loginfo("setting Cartesian controller parameters")
    rospy.set_param(self.tcname+'/type', "CartesianTrajectoryController")
    rospy.set_param(self.tcname+'/root_name', self.base_frame)
    rospy.set_param(self.tcname+'/tip_name', self.tip_frame)
    rospy.set_param(self.tcname+'/output', self.pcname)
    rospy.set_param(self.pcname+'/type', "CartesianPoseController")
    rospy.set_param(self.pcname+'/root_name', self.base_frame)
    rospy.set_param(self.pcname+'/tip_name', self.tip_frame)
    rospy.set_param(self.pcname+'/output', 'r_arm_cartesian_twist_controller')
    rospy.set_param(self.tcname+'/max_vel_trans', self.max_vel_trans)
    rospy.set_param(self.tcname+'/max_vel_rot', self.max_vel_rot)
    rospy.set_param(self.tcname+'/max_acc_trans', self.max_acc_trans)
    rospy.set_param(self.tcname+'/max_acc_rot', self.max_acc_rot)
    rospy.set_param(self.pcname+'/fb_trans/p', self.pose_fb_trans_p)
    rospy.set_param(self.pcname+'/fb_trans_d', self.pose_fb_trans_d)
    rospy.set_param(self.pcname+'/fb_rot/p', self.pose_fb_rot_p)
    rospy.set_param(self.pcname+'/fb_rot/d', self.pose_fb_rot_d)
    #tip_frame = rospy.get_param(self.tcname+'/tip_name')
