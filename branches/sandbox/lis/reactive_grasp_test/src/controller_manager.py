#! /usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Kaijen Hsiao


## @package controller_manager
#Interface to various controllers and control services: switches between/uses 
#joint and Cartesian controllers for one arm, controls the corresponding gripper, 
#calls move_arm for collision-free motion planning

from __future__ import division
import roslib; roslib.load_manifest('pr2_gripper_reactive_approach')
import random, time
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped, Vector3Stamped
from pr2_mechanism_msgs.srv import SwitchController, LoadController, UnloadController, ListControllers
import actionlib
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, \
    Pr2GripperCommandGoal, Pr2GripperCommandAction
from pr2_gripper_sensor_msgs.msg import PR2GripperFindContactAction, PR2GripperFindContactGoal, \
    PR2GripperGrabAction, PR2GripperGrabGoal, PR2GripperEventDetectorAction, PR2GripperEventDetectorGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from simple_Jtranspose_controller.srv import CheckMoving, MoveToPose, MoveToPoseRequest
from std_srvs.srv import Empty, EmptyRequest
from move_arm_msgs.msg import MoveArmAction, MoveArmGoal
from motion_planning_msgs.msg import JointConstraint, PositionConstraint, OrientationConstraint, \
    ArmNavigationErrorCodes, AllowedContactSpecification
from geometric_shapes_msgs.msg import Shape
import numpy
import math
import pdb
import copy
from interpolated_ik_motion_planner import ik_utilities
from object_manipulator.convert_functions import *
import joint_states_listener
from actionlib_msgs.msg import GoalStatus

##load the params for the joint controllers onto the param server
class JointParams:

  def __init__(self, whicharm): #whicharm is 'r' or 'l'
    self.whicharm = whicharm
    self.default_p = [800., 800., 200., 100., 500., 400., 400.]
    self.default_d = [10., 10., 3., 3., 6., 4., 4.]
    joint_names = ['_shoulder_pan_joint', '_shoulder_lift_joint',
                     '_upper_arm_roll_joint', '_elbow_flex_joint',
                     '_forearm_roll_joint', '_wrist_flex_joint',
                     '_wrist_roll_joint']
    self.controller_name = whicharm+"_arm_controller"
    self.joint_names = [whicharm+joint_name for joint_name in joint_names]

  ##set gains to some fraction of the default
  def set_gains_fraction(self, fraction):
    for i in range(7):
      rospy.set_param(self.controller_name+'/gains/%s/p'%self.joint_names[i], self.default_p[i]*fraction)
      rospy.set_param(self.controller_name+'/gains/%s/d'%self.joint_names[i], self.default_d[i]*fraction)


##load the params for the Jtranspose Cartesian controllers onto the param server
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


  ##set the parameter values back to their defaults
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


  ##set the gains and max vels and accs lower, so the arm moves more gently/slowly
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


  ##set the current parameter values on the parameter server
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



##class to start/stop/switch between joint and Cartesian controllers
class ControllerManager():

  def __init__(self, whicharm, tf_listener = None, using_slip_controller = 0): #whicharm is 'r' or 'l'
    self.whicharm = whicharm

    self.using_slip_controller = using_slip_controller

    rospy.loginfo("initializing "+whicharm+" controller manager")
    rospy.loginfo("controller manager: using_slip_controller:"+str(using_slip_controller))

    #wait for the load/unload/switch/list controller services to be there and initialize their service proxies
    rospy.loginfo("controller manager waiting for pr2_controller_manager services to be there")
    load_controller_serv_name = 'pr2_controller_manager/load_controller'
    unload_controller_serv_name = 'pr2_controller_manager/unload_controller'
    switch_controller_serv_name = 'pr2_controller_manager/switch_controller'
    list_controllers_serv_name = 'pr2_controller_manager/list_controllers'
    self.wait_for_service(load_controller_serv_name)
    self.wait_for_service(unload_controller_serv_name)
    self.wait_for_service(switch_controller_serv_name)
    self.wait_for_service(list_controllers_serv_name)
    self.load_controller_service = \
        rospy.ServiceProxy(load_controller_serv_name, LoadController)
    self.unload_controller_service = \
        rospy.ServiceProxy(unload_controller_serv_name, UnloadController)
    self.switch_controller_service = \
        rospy.ServiceProxy(switch_controller_serv_name, SwitchController)
    self.list_controllers_service = \
        rospy.ServiceProxy(list_controllers_serv_name, ListControllers)

    #initialize listener for JointStates messages (separate thread)
    self.joint_states_listener = joint_states_listener.LatestJointStates()

    #joint trajectory action client
    joint_trajectory_action_name = whicharm+'_arm_controller/joint_trajectory_action'
    self.joint_action_client = \
        actionlib.SimpleActionClient(joint_trajectory_action_name, JointTrajectoryAction)

    #slip-sensing gripper action clients
    if self.using_slip_controller:
      gripper_action_name = whicharm+'_gripper_sensor_controller/gripper_action'
      gripper_find_contact_action_name = whicharm+'_gripper_sensor_controller/find_contact'
      gripper_grab_action_name = whicharm+'_gripper_sensor_controller/grab'
      gripper_event_detector_action_name = whicharm+'_gripper_sensor_controller/event_detector'
      self.gripper_action_client = actionlib.SimpleActionClient(gripper_action_name, Pr2GripperCommandAction)
      self.gripper_find_contact_action_client = actionlib.SimpleActionClient(gripper_find_contact_action_name, \
                                                                             PR2GripperFindContactAction)
      self.gripper_grab_action_client = actionlib.SimpleActionClient(gripper_grab_action_name, \
                                                                             PR2GripperGrabAction)
      self.gripper_event_detector_action_client = actionlib.SimpleActionClient(gripper_event_detector_action_name, \
                                                                             PR2GripperEventDetectorAction)

    #default gripper action client
    else:
      gripper_action_name = whicharm+'_gripper_controller/gripper_action'
      self.gripper_action_client = \
          actionlib.SimpleActionClient(gripper_action_name, Pr2GripperCommandAction)
      
    #move arm client is filled in the first time it's called
    self.move_arm_client = None
    
    #wait for the joint trajectory and gripper action servers to be there
    self.wait_for_action_server(self.joint_action_client, joint_trajectory_action_name)
    self.wait_for_action_server(self.gripper_action_client, gripper_action_name)
    if self.using_slip_controller:
      self.wait_for_action_server(self.gripper_find_contact_action_client, gripper_find_contact_action_name)
      self.wait_for_action_server(self.gripper_grab_action_client, gripper_grab_action_name)
      self.wait_for_action_server(self.gripper_event_detector_action_client, gripper_event_detector_action_name)

    #initialize a tf listener
    if tf_listener == None:
      self.tf_listener = tf.TransformListener()
    else:
      self.tf_listener = tf_listener

    #names of the controllers
    self.cartesian_controllers = ['_arm_cartesian_pose_controller', 
                                    '_arm_cartesian_trajectory_controller']
    self.cartesian_controllers = [self.whicharm+x for x in self.cartesian_controllers]
    self.joint_controller = self.whicharm+'_arm_controller'
    if self.using_slip_controller:
      self.gripper_controller = self.whicharm+'_gripper_sensor_controller'
    else:
      self.gripper_controller = self.whicharm+'_gripper_controller'

    #parameters for the Cartesian controllers
    self.cart_params = JTCartesianParams(self.whicharm)

    #parameters for the joint controller
    self.joint_params = JointParams(self.whicharm)

    #load the Cartesian controllers if not already loaded
    rospy.loginfo("loading any unloaded Cartesian controllers")
    self.load_cartesian_controllers()
    time.sleep(2)
    rospy.loginfo("done loading controllers")

    #services for the J-transpose Cartesian controller 
    cartesian_check_moving_name = whicharm+'_arm_cartesian_trajectory_controller/check_moving'
    cartesian_move_to_name = whicharm+'_arm_cartesian_trajectory_controller/move_to'
    cartesian_preempt_name = whicharm+'_arm_cartesian_trajectory_controller/preempt'
    self.wait_for_service(cartesian_check_moving_name)      
    self.wait_for_service(cartesian_move_to_name)
    self.wait_for_service(cartesian_preempt_name)      
    self.cartesian_moving_service = rospy.ServiceProxy(cartesian_check_moving_name, CheckMoving)
    self.cartesian_cmd_service = rospy.ServiceProxy(cartesian_move_to_name, MoveToPose)
    self.cartesian_preempt_service = rospy.ServiceProxy(cartesian_preempt_name, Empty)

    #re-load the Cartesian controllers with the gentle params
    self.cart_params.set_params_to_gentle()
    self.reload_cartesian_controllers()

    #create an IKUtilities class object
    rospy.loginfo("creating IKUtilities class objects")
    if whicharm == 'r':
      self.ik_utilities = ik_utilities.IKUtilities('right', self.tf_listener)
    else:
      self.ik_utilities = ik_utilities.IKUtilities('left', self.tf_listener)
    rospy.loginfo("done creating IKUtilities class objects")

    #joint names for the arm
    joint_names = ["_shoulder_pan_joint", 
                   "_shoulder_lift_joint", 
                   "_upper_arm_roll_joint", 
                   "_elbow_flex_joint", 
                   "_forearm_roll_joint", 
                   "_wrist_flex_joint", 
                   "_wrist_roll_joint"]
    self.joint_names = [whicharm + x for x in joint_names]

    rospy.loginfo("done with controller_manager init for the %s arm"%whicharm)


  ##wait for an action server to be ready
  def wait_for_action_server(self, client, name):
    while not rospy.is_shutdown():  
      rospy.loginfo("controller manager: waiting for %s to be there"%name)
      if client.wait_for_server(rospy.Duration(5.0)):
        break
    rospy.loginfo("controller manager: %s found"%name)  


  ##wait for a service to be ready
  def wait_for_service(self, name):
    while not rospy.is_shutdown():  
      rospy.loginfo("controller manager: waiting for %s to be there"%name)
      try:
        rospy.wait_for_service(name, 5.0)
      except rospy.ROSException:
        continue
      break
    rospy.loginfo("controller manager: %s found"%name)  


  ##tell the gripper to close until contact (on one or both finger pads)
  #contacts_desired is "both", "left", "right", or "either"
  def find_gripper_contact(self, contacts_desired, zero_fingertips = 0, blocking = 1, timeout = 12.):

    goal = PR2GripperFindContactGoal()
    contacts_desired_dict = {"both":goal.command.BOTH, "left":goal.command.LEFT, "right":goal.command.RIGHT, "either":goal.command.EITHER}

    goal.command.contact_conditions = contacts_desired_dict[contacts_desired]
    goal.command.zero_fingertip_sensors = zero_fingertips
    
    rospy.loginfo("controller manager: sending find contact goal")
    self.gripper_find_contact_action_client.send_goal(goal)
    
    #if blocking is requested, wait for the state to reach the desired state
    if blocking:
      finished_within_time = self.gripper_find_contact_action_client.wait_for_result(rospy.Duration(timeout))
      if not finished_within_time:
        self.gripper_find_contact_action_client.cancel_goal()
        rospy.logerr("Gripper didn't see the desired number of contacts while closing in time")
        return (0, 0, 0, 0)
      state = self.gripper_find_contact_action_client.get_state()
      if state == GoalStatus.SUCCEEDED:
        result = self.gripper_find_contact_action_client.get_result()
        return (result.data.left_fingertip_pad_contact, result.data.right_fingertip_pad_contact, \
                  result.data.left_fingertip_pad_contact_force, result.data.right_fingertip_pad_contact_force)
      return (0,0,0,0)


  ##get the state from find_gripper_contact 
  def get_find_gripper_contact_state(self):
    return self.gripper_find_contact_action_client.get_state() 


  ##get the final result from find_gripper_contact
  def get_find_gripper_contact_result(self):
    result = self.gripper_find_contact_action_client.get_result()
    if result:
      return (result.data.left_fingertip_pad_contact, result.data.right_fingertip_pad_contact, \
                result.data.left_fingertip_pad_force, result.data.right_fingertip_pad_force)
    else:
      rospy.logerr("gripper_find_contact_action_client.get_result returned None!")
      return (0, 0, 0, 0)


  ##opens and closes the gripper to determine how hard to grasp the object, then starts up slip servo mode
  def start_gripper_grab(self,  hardness_gain = 0.026, timeout = 10., blocking = 1):
    
    goal = PR2GripperGrabGoal()
    goal.command.hardness_gain = hardness_gain

    rospy.loginfo("starting slip controller grab")
    self.gripper_grab_action_client.send_goal(goal)

    if blocking:
      finished_within_time = self.gripper_grab_action_client.wait_for_result(rospy.Duration(timeout))
      if not finished_within_time:
        self.gripper_find_contact_action_client.cancel_goal()
        rospy.logerr("Gripper grab timed out")
      state = self.gripper_find_contact_action_client.get_state()
      if state == GoalStatus.SUCCEEDED:
        return 1
      return 0

    return 1


  ##start up gripper event detector to detect when an object hits the table 
  #or when someone is trying to take an object from the robot
  def start_gripper_event_detector(self, blocking = 0, timeout = 15.):
    
    goal = PR2GripperEventDetectorGoal()
    goal.command.trigger_conditions = goal.command.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC  #use either slip or acceleration as a contact condition
    goal.command.acceleration_trigger_magnitude = 3.25  #contact acceleration used to trigger 
    goal.command.slip_trigger_magnitude = 0.008        #contact slip used to trigger    

    rospy.loginfo("starting gripper event detector")
    self.gripper_event_detector_action_client.send_goal(goal)

    #if blocking is requested, wait until the action returns
    if blocking:
      finished_within_time = self.gripper_event_detector_action_client.wait_for_result(rospy.Duration(timeout))
      if not finished_within_time:
        rospy.logerr("Gripper didn't see the desired event trigger before timing out")
        return 0
      state = self.gripper_event_detector_action_client.get_state()
      if state == GoalStatus.SUCCEEDED:
        result = self.gripper_event_detector_action_client.get_result()
        if result.data.placed:
          return 1
      return 0


  ##get the state from the gripper event detector
  def get_gripper_event_detector_state(self):
    return self.gripper_event_detector_action_client.get_state()


  ##create a basic goal message for move_arm
  def create_move_arm_goal(self):
    goal = MoveArmGoal()
    if self.whicharm == "r":
      goal.motion_plan_request.group_name = "right_arm"
    else:
      goal.motion_plan_request.group_name = "left_arm"
    goal.motion_plan_request.num_planning_attempts = 2;
    goal.motion_plan_request.allowed_planning_time = rospy.Duration(5.0);
    goal.motion_plan_request.planner_id = ""
    goal.planner_service_name = "ompl_planning/plan_kinematic_path"
    return goal


  ##send a goal to move arm and wait for the result, modifying the goal if the start state is in collision
  def send_move_arm_goal(self, goal, blocking = 1):

    #create an action client, if this is the first call to move_arm
    if self.move_arm_client == None:
      if self.whicharm == "r":
        self.move_arm_client = actionlib.SimpleActionClient('move_right_arm', MoveArmAction)
      if self.whicharm == "l":
        self.move_arm_client = actionlib.SimpleActionClient('move_left_arm', MoveArmAction)
      rospy.loginfo("waiting for move arm server")
      self.move_arm_client.wait_for_server()
      rospy.loginfo("move arm server was there")

    #send the goal off
    self.move_arm_client.send_goal(goal)

    #wait for the move to finish and return the result
    if blocking:
      for add_contact_tries in range(10):
        finished_within_time = self.move_arm_client.wait_for_result(rospy.Duration(60))
        if not finished_within_time:
          self.move_arm_client.cancel_goal()
          rospy.logerr("Timed out when asking move_arm to go to a joint goal")
          return 0

        state = self.move_arm_client.get_state()
        result = self.move_arm_client.get_result()
        if state == GoalStatus.SUCCEEDED:
          if result.error_code.val == 1:
            rospy.loginfo("move_arm succeeded")
            break

          #start state was in collision!  Try to add contact regions to ignore the start collisions
          elif result.error_code.val == ArmNavigationErrorCodes.START_STATE_IN_COLLISION:
            rospy.loginfo("move arm start state in collision!  Adding allowed contact regions and trying again, try number %d"%add_contact_tries)
            self.modify_move_arm_goal(goal, result.contacts)
            self.move_arm_client.send_goal(goal)
            continue
        else:
          rospy.logerr("move_arm did not succeed, state %d, error code %d"%(state, result.error_code.val))
          break
      return result.error_code.val
    else:
      return 1


  ##use move_arm to get to a desired joint configuration in a collision-free way
  def move_arm_joint(self, joint_angles, blocking = 1):
    self.check_controllers_ok('joint')

    rospy.loginfo("asking move_arm to go to angles: %s"%self.pplist(joint_angles))

    goal = self.create_move_arm_goal()
    #goal.disable_collision_monitoring = 1

    #add the joint angles as a joint constraint
    for (joint_name, joint_angle) in zip(self.joint_names, joint_angles):
      joint_constraint = JointConstraint()
      joint_constraint.joint_name = joint_name
      joint_constraint.position = joint_angle
      joint_constraint.tolerance_below = .1
      joint_constraint.tolerance_above = .1
      goal.motion_plan_request.goal_constraints.joint_constraints.append(joint_constraint)

    #send the goal off to move arm, blocking if desired and modifying the start state if it's in collision
    result = self.send_move_arm_goal(goal, blocking)
    return result


  ##if move_arm_joint returns an error code of START_STATE_IN_COLLISION, we can try to add 
  #allowed contact regions to get it out of the start collisions (appears to already be done in C++)
  def modify_move_arm_goal(self, goal, contact_info):
    allowed_contacts = []
    allowed_penetration_depth = .5
    for (i, contact) in enumerate(contact_info):
      allowed_contact_tmp = AllowedContactSpecification()
      allowed_contact_tmp.shape.type = allowed_contact_tmp.shape.BOX
      allowed_contact_tmp.shape.dimensions = [allowed_penetration_depth]*3
      allowed_contact_tmp.pose_stamped.pose.position = contact.position
      set_xyzw(allowed_contact_tmp.pose_stamped.pose.orientation, [0,0,0,1])
      allowed_contact_tmp.pose_stamped.header.stamp = rospy.Time.now()
      allowed_contact_tmp.pose_stamped.header.frame_id = contact.header.frame_id
      allowed_contact_tmp.link_names.append(contact.contact_body_1)
      allowed_contact_tmp.penetration_depth = allowed_penetration_depth
      allowed_contacts.append(allowed_contact_tmp);
      rospy.loginfo("Added allowed contact region: %d"%i)
      #rospy.loginfo("Position                    : (%f,%f,%f)"%(contact.position.x, \
      #         contact.position.y,contact.position.z))
      #rospy.loginfo("Frame id                    : %s"%contact.header.frame_id);
      rospy.loginfo("Bodies                      : %s and %s"%(contact.contact_body_1, contact.contact_body_2))
    goal.motion_plan_request.allowed_contacts.extend(allowed_contacts)


  ##move the wrist to the desired location while keeping the current wrist orientation upright
  #start angles are the joint angle start point for IK searches
  #default start angles and location are for moving the arm to the side
  def move_arm_constrained(self, start_angles = None, location = None, blocking = 1):

    current_pose = self.get_current_wrist_pose_stamped('base_link')
    move_arm_goal = self.create_move_arm_goal()

    #default search-starting arm angles are arm-to-the-side
    if start_angles == None:
      if self.whicharm == 'l':
        start_angles = [2.135, 0.803, 1.732, -1.905, 2.369, -1.680, 1.398]
      else:
        start_angles = [-2.135, 0.803, -1.732, -1.905, -2.369, -1.680, 1.398]

    #default location is arm-to-the-side
    if location == None:
      if self.whicharm == 'l':
        location = [0.05, 0.576, 0.794]
      else:
        location = [0.05, -0.576, 0.794]

    #add a position constraint for the goal
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = current_pose.header.frame_id
    position_constraint.link_name = self.ik_utilities.link_name #r_ or l_wrist_roll_link
    set_xyz(position_constraint.position, location)
    position_constraint.constraint_region_shape.type = Shape.BOX
    position_constraint.constraint_region_shape.dimensions = [0.02]*3
    position_constraint.constraint_region_orientation.w = 1.0
    position_constraint.weight = 1.0
    move_arm_goal.motion_plan_request.goal_constraints.position_constraints.append(position_constraint)

    #search for a feasible goal pose that gets our wrist to location while keeping all but the (base-link) yaw fixed
    current_pose_mat = pose_to_mat(current_pose.pose)
    found_ik_solution = 0
    for angle in range(0, 180, 5):
      for dir in [1, -1]:
        rot_mat = tf.transformations.rotation_matrix(dir*angle/180.*math.pi, [0,0,1])
        rotated_pose_mat = rot_mat * current_pose_mat
        desired_pose = stamp_pose(mat_to_pose(rotated_pose_mat), 'base_link')
        desired_pose.pose.position = position_constraint.position
        
        #check if there's a collision-free IK solution at the goal location with that orientation
        (solution, error_code) = self.ik_utilities.run_ik(desired_pose, start_angles, \
                                       self.ik_utilities.link_name, collision_aware = 1)
        if error_code == "SUCCESS":
          found_ik_solution = 1
          break
      if found_ik_solution:
        break
    else:
      rospy.loginfo("no IK solution found for goal, aborting")
      return 0

    #add an orientation constraint for the goal
    orientation_constraint = OrientationConstraint()
    orientation_constraint.header.stamp = rospy.Time.now()
    orientation_constraint.header.frame_id = current_pose.header.frame_id
    orientation_constraint.link_name = self.ik_utilities.link_name
    orientation_constraint.orientation = desired_pose.pose.orientation 
    orientation_constraint.type = OrientationConstraint.HEADER_FRAME
    orientation_constraint.absolute_roll_tolerance = 0.2
    orientation_constraint.absolute_pitch_tolerance = 0.5
    orientation_constraint.absolute_yaw_tolerance = math.pi
    orientation_constraint.weight = 1.0
    move_arm_goal.motion_plan_request.goal_constraints.orientation_constraints.append(orientation_constraint)

    #add an orientation constraint for the entire path to keep the object upright
    path_orientation_constraint = copy.deepcopy(orientation_constraint)
    
    #temporary, while deepcopy of rospy.Time is broken
    path_orientation_constraint.header.stamp = rospy.Time(orientation_constraint.header.stamp.secs)
    path_orientation_constraint.orientation = current_pose.pose.orientation
    move_arm_goal.motion_plan_request.path_constraints.orientation_constraints.append(path_orientation_constraint)    
  
    #send the goal off to move arm, blocking if desired and modifying the start state if it's in collision
    result = self.send_move_arm_goal(move_arm_goal, blocking)
    return result


  ##use move_arm to get to a desired pose in a collision-free way (really, run IK and then call move arm to move to the IK joint angles)
  def move_arm_pose(self, pose_stamped, start_angles = None, blocking = 1):
    if start_angles == None:
      start_angles = self.get_current_arm_angles()
    (solution, error_code) = self.ik_utilities.run_ik(pose_stamped, start_angles, \
                    self.ik_utilities.link_name)
    if not solution:
      rospy.logerr("no IK solution found for goal pose!")
      return 0
    else:
      result = self.move_arm_joint(solution, blocking)
      return result


  ##normalize a trajectory (list of lists of joint angles), so that the desired angles 
  #are the nearest ones for the continuous joints (5 and 7)
  def normalize_trajectory(self, trajectory):
    current_angles = self.get_current_arm_angles()
    trajectory_copy = [list(angles) for angles in trajectory]
    for angles in trajectory_copy:
      angles[4] = self.normalize_angle(angles[4], current_angles[4])
      angles[6] = self.normalize_angle(angles[6], current_angles[6])
    return trajectory_copy


  ##normalize an angle for a continuous joint so that it's the closest version 
  #of the angle to the current angle (not +-2*pi)
  def normalize_angle(self, angle, current_angle):
    while current_angle-angle > math.pi:
      angle += 2*math.pi
    while angle - current_angle > math.pi:
      angle -= 2*math.pi
    return angle


  ##find and execute a joint-angle trajectory to perform a collision-free Cartesian movement
  #if start_pose is None, starts from the current pose (if not None, will go straight to the start pose using the joint trajectory action)
  def command_interpolated_ik(self, end_pose, start_pose = None, collision_aware = 1, step_size = .02, max_joint_vel = .05):
    self.check_controllers_ok('joint')

    #find the current joint angles 
    current_angles = self.get_current_arm_angles()

    if start_pose == None:
      #use the current wrist pose as the start pose/angles
      (current_trans, current_rot) = self.return_cartesian_pose()

      #put the current pose into a poseStamped
      start_pose = create_pose_stamped(current_trans+current_rot)
      #print "start_pose:\n", start_pose
    #print "end_pose:\n", end_pose

    #find a collision-free joint trajectory to move from the current pose 
    #to the desired pose using Cartesian interpolation
    (trajectory, error_codes) = self.ik_utilities.check_cartesian_path(start_pose, \
                     end_pose, current_angles, step_size, .1, math.pi/4, collision_aware)

    #if some point is not possible, quit
    if any(error_codes):
      rospy.loginfo("can't execute an interpolated IK trajectory to that pose")
      return 0

#     print "found trajectory:"
#     for point in trajectory:
#       if type(point) == tuple:
#         print self.pplist(point)
#       else:
#         print point

    #send the trajectory to the joint trajectory action
    #print "moving through trajectory"
    self.command_joint_trajectory(trajectory, max_joint_vel)
    return 1


  ##use the joint_states_listener to get the arm angles
  def get_current_arm_angles(self):
    (found, positions, vels, efforts) = \
        self.joint_states_listener.return_joint_states(self.joint_names)
    return positions


  ##return the current pose of the wrist as a PoseStamped
  def get_current_wrist_pose_stamped(self, frame = 'base_link'):
      (current_trans, current_rot) = self.return_cartesian_pose(frame)
      return create_pose_stamped(current_trans+current_rot, frame)


  ##use the joint_states_listener to get the current gripper opening
  def get_current_gripper_opening(self):
    (found, positions, vels, efforts) = \
        self.joint_states_listener.return_joint_states([self.whicharm+'_gripper_joint'])
    return positions[0]


  ##send a single desired pose to the Cartesian controller 
  #(pose is either a PoseStamped or a 7-list of position and quaternion orientation; if the latter, put the frame
  #in frame_id) 
  def command_cartesian(self, pose, frame_id = 'base_link'):
    self.check_controllers_ok('cartesian')

    if type(pose) == list:
      m = create_pose_stamped(pose, frame_id)
    else:
      m = pose
    req = MoveToPoseRequest()
    req.pose = m
    try:
      self.cartesian_preempt_service(EmptyRequest())
    except:
      rospy.loginfo("preempt unnecessary, robot not moving")
    else:
      rospy.loginfo("Cartesian movement preempted before issuing new command")
      
    self.cartesian_cmd_service(req)
    #self.cartesian_cmd_pub.publish(m)    
      

  ##check if the Cartesian controller thinks it's gotten to its goal (it's pretty loose about goals)
  def check_cartesian_done(self):
    resp = self.cartesian_moving_service()
    return resp.ismoving


  ##check if we're near to a desired Cartesian pose (either PoseStamped or 7-list of position and quaternion
  #in 'base_link' frame) by pos_thres(m) and rot_thres (rad)
  def check_cartesian_near_pose(self, goal_pose, pos_thres, rot_thres):
    if type(goal_pose) == list:
      goal_pos = goal_pose[0:3]
      goal_rot = goal_pose[3:7]
    else:
      (goal_pos, goal_rot) = pose_stamped_to_lists(self.tf_listener, goal_pose, 'base_link')

    (current_pos, current_rot) = self.return_cartesian_pose()
    
    #compute how far the wrist is translated from the goal
    diff = [x-y for (x,y) in zip(goal_pos, current_pos)]
    pos_diff = self.ik_utilities.vect_norm(diff)

    #compute how far the wrist is rotated from the goal
    norm_goal_rot = self.ik_utilities.normalize_vect(goal_rot)
    norm_current_rot = self.ik_utilities.normalize_vect(current_rot)
    rot_diff = self.ik_utilities.quat_angle(norm_current_rot, norm_goal_rot)
    #rospy.loginfo("pos_diff: %5.3f, rot_diff: %5.3f"%(pos_diff, rot_diff))
    if pos_diff < pos_thres and rot_diff < rot_thres:
      return 1
    return 0


  ##check if both the Cartesian controllers think we're done and if we're close to where we want to be
  def check_cartesian_really_done(self, goal_pose, pos_thres, rot_thres):
    if not self.check_cartesian_done():
      return 0
    return self.check_cartesian_near_pose(goal_pose, pos_thres, rot_thres)


  ##wait for either the Cartesian pose to get near enough to a goal pose or timeout (a ROS duration) is reached
  #or settling_time (a ROS duration) after the Cartesian controllers report that they're done
  def wait_cartesian_really_done(self, goal_pose, pos_thres = .02, rot_thres = .1, timeout = 0., settling_time = rospy.Duration(3.0)):
    start_time = rospy.get_rostime()
    done_time = None
    while(1):
      if self.check_cartesian_really_done(goal_pose, pos_thres, rot_thres):
        rospy.loginfo("got to the goal")
        return "success"
      if done_time == None and self.check_cartesian_done():
        #rospy.loginfo("Cartesian controllers think they're done")
        done_time = rospy.get_rostime()
      if done_time and rospy.get_rostime() - done_time > settling_time:
        rospy.loginfo("settling time ran out")
        return "failed"
      if timeout != 0. and rospy.get_rostime() - start_time > timeout:
        rospy.loginfo("timed out")
        return "timed out"


  ##move in Cartesian space using the Cartesian controllers
  #if collision_aware, checks whether the path could be collision-free, but
  #uses the Cartesian controllers to execute it
  #if blocking, waits for completion, otherwise returns after sending the goal
  #step_size only used if collision aware (size of steps to check for collisions)
  #pos_thres and rot_thres are thresholds within which the goal is declared reached
  #times out and quits after timeout, and waits settling_time after the 
  #controllers declare that they're done (which is usually way too early)
  def move_cartesian(self, goal_pose, collision_aware = 0, blocking = 1,
                     step_size = .005, pos_thres = .02, rot_thres = .1,
                     timeout = rospy.Duration(30.), 
                     settling_time = rospy.Duration(1.)):
    #get the current wrist pose as the start pose
    (current_pos, current_rot) = self.return_cartesian_pose('base_link')
    start_pose = create_pose_stamped(current_pos+current_rot)

    #check to see that the path could possibly be collision-free
    if collision_aware:
      current_angles = self.get_current_arm_angles()
      (traj, error_codes) = self.ik_utilities.check_cartesian_path(start_pose, \
                       goal_pose, current_angles, .01, .1, math.pi/4, 1) 
      if any(error_codes):
        return "no solution"

    #go to the goal pose using the Cartesian controllers
    self.command_cartesian(goal_pose)
    if not blocking:
      return "sent goal"

    #blocking: wait for the Cartesian controllers to get there or time out
    result = self.wait_cartesian_really_done(goal_pose, pos_thres, \
                 rot_thres, timeout = timeout, settling_time = settling_time)


  ##move in Cartesian space using IK Cartesian interpolation
  #if collision_aware, quits if the path is not collision-free
  #if blocking, waits for completion, otherwise returns after sending the goal
  #step_size is the step size for interpolation
  #pos_thres and rot_thres are thresholds within which the goal is declared reached
  #times out and quits after timeout, and waits settling_time after the 
  #controllers declare that they're done
  def move_cartesian_ik(self, goal_pose, collision_aware = 0, blocking = 1,
                     step_size = .005, pos_thres = .02, rot_thres = .1,
                     timeout = rospy.Duration(30.), 
                     settling_time = rospy.Duration(0.25), vel = .15):

    #send the interpolated IK goal to the joint trajectory action
    result = self.command_interpolated_ik(goal_pose, collision_aware = collision_aware, \
                                            step_size = step_size, max_joint_vel = vel)
    if not result:
      return "no solution"
    if not blocking:
      return "sent goal"

    #blocking: wait for the joint trajectory action to get there or time out
    joint_action_result = self.wait_joint_trajectory_done(timeout)
    if joint_action_result == "timed out":
      return "timed out"
    
    if self.check_cartesian_really_done(goal_pose, pos_thres, rot_thres):
      return "success"
    return "failed"


  ##make sure the appropriate controllers (joint or Cartesian) are running/stopped
  #mode is 'joint' or 'cartesian'
  def check_controllers_ok(self, mode):
    modewrong = 0
    if mode == 'joint':
#       if not self.joint_controller_state == 'running':
#         modewrong = 1
#       if any([x == 'running' for x in self.cartesian_controllers_state]):
#         modewrong = 1

#       if modewrong:
      self.check_controller_states()
      if not self.joint_controller_state == 'running' and \
            any([x == 'running' for x in self.cartesian_controllers_state]):
        self.switch_to_joint_mode()
      elif not self.joint_controller_state == 'running':
        rospy.logwarn("joint controllers aren't running!  Attempting to start")
        self.start_joint_controllers()
      elif any([x == 'running' for x in self.cartesian_controllers_state]):
        rospy.logwarn("Cartesian controllers are running!  Attempting to stop")
        self.stop_controllers(stop_cartesian = 1)

    elif mode == 'cartesian':
#       if not all([x == 'running' for x in self.cartesian_controllers_state]):
#         modewrong = 1
#       if self.joint_controller_state == 'running':
#         modewrong = 1

#       if modewrong:
      self.check_controller_states()
      if not all([x == 'running' for x in self.cartesian_controllers_state]) and \
            self.joint_controller_state == 'running':
        self.switch_to_cartesian_mode()
      elif not all([x == 'running' for x in self.cartesian_controllers_state]):
        rospy.logwarn("Cartesian controllers aren't running!  Attempting to start")
        self.start_cartesian_controllers()
      elif self.joint_controller_state == 'running':
        rospy.logwarn("joint controllers are running!  Attempting to stop")
        self.stop_controllers(stop_joint = 1)


  ##send a single joint configuration to the joint trajectory action (takes in a 7-list of angles, nonblocking)
  def command_joint(self, jointangles):
    self.check_controllers_ok('joint')

    goal = JointTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.get_rostime()

    goal.trajectory.joint_names = self.joint_names

    point = JointTrajectoryPoint(jointangles, [0]*7, [0]*7, rospy.Duration(5.0))
    goal.trajectory.points = [point,]

    self.joint_action_client.send_goal(goal)


  ##send an entire joint trajectory to the joint trajectory action (nonblocking)
  def command_joint_trajectory(self, trajectory, max_joint_vel = 0.2, current_angles = None, blocking = 0):
    self.check_controllers_ok('joint')

    goal = JointTrajectoryGoal()
    goal.trajectory.joint_names = self.joint_names

    #normalize the trajectory so the continuous joints angles are close to the current angles
    trajectory = self.normalize_trajectory(trajectory)

    #start the trajectory 0.05 s from now
    goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.05)

    #get the current joint angles
    if current_angles == None:
      current_angles = list(self.get_current_arm_angles())

    #set the first trajectory point to the current position
    trajectory = [current_angles] + trajectory
    
    #find appropriate times and velocities for the trajectory
    (times, vels) = self.ik_utilities.trajectory_times_and_vels(trajectory, [max_joint_vel]*7)

#     print "trajectory:"
#     for point in trajectory:
#       print self.pplist(point)
#     print "times:", self.pplist(times)
#     print "vels:"
#     for vel in vels:
#       print self.pplist(vel)

    #fill in the trajectory message
    for ind in range(len(trajectory)):
      point = JointTrajectoryPoint(trajectory[ind], vels[ind], [0]*7, rospy.Duration(times[ind]))
      goal.trajectory.points.append(point)

    #send the goal
    self.joint_action_client.send_goal(goal)
    
    #if blocking, wait until it gets there
    if blocking:
      self.wait_joint_trajectory_done(timeout = rospy.Duration(times[-1]+5))


  ##check the state of the joint trajectory action (returns 1 if done, 0 otherwise) 
  def check_joint_trajectory_done(self):
    state = self.joint_action_client.get_state()
    return state > 1


  ##wait for the joint trajectory action to finish (returns 1 if succeeded, 0 otherwise)
  #timeout is a ROS duration; default (0) is infinite timeout
  #returns 0 if timed out, 1 if succeeded
  def wait_joint_trajectory_done(self, timeout = rospy.Duration(0)):
    finished = self.joint_action_client.wait_for_result(timeout)
    if not finished:
      return "timed out"
    state = self.joint_action_client.get_state()
    if state == 3:
      return "succeeded"
    return "other"


  ##tell the joint trajectory action to stop the arm where it is now
  def freeze_arm(self):
    current_angles = self.get_current_arm_angles()
    self.command_joint(current_angles)


  ##send a command to the gripper action 
  def command_gripper(self, position, max_effort, blocking = 0):
    if not self.gripper_controller_state == 'running':
      self.check_controller_states()
      if not self.gripper_controller_state == 'running':
        rospy.logwarn("gripper controller not running!  Attempting to start")
        self.start_gripper_controller()

    goal = Pr2GripperCommandGoal()
    goal.command.position = position
    goal.command.max_effort = max_effort
    self.gripper_action_client.send_goal(goal)

    #if blocking, wait for the gripper to finish
    if blocking:
      self.gripper_action_client.wait_for_result(rospy.Duration(4.0))
      time.sleep(.5)
    

  ##figure out which controllers are loaded/started (fills in self.joint_controller_state,
  #self.cartesian_controllers_state, and self.gripper_controller_state with 
  #'not loaded', 'stopped', or 'running'
  def check_controller_states(self):
    resp = self.list_controllers_service()

    if self.gripper_controller in resp.controllers:
      self.gripper_controller_state = resp.state[resp.controllers.index(self.gripper_controller)]
    else:
      self.gripper_controller_state = 'not loaded'

    if self.joint_controller in resp.controllers:
      self.joint_controller_state = resp.state[resp.controllers.index(self.joint_controller)]
    else:
      self.joint_controller_state = 'not loaded'

    self.cartesian_controllers_state = []
    for controller in self.cartesian_controllers:
      if controller not in resp.controllers:
        self.cartesian_controllers_state.append('not loaded')
      else:       
        self.cartesian_controllers_state.append(resp.state[resp.controllers.index(controller)])


  ##load the Cartesian controllers with the current set of params on the param server
  def load_cartesian_controllers(self):
    self.check_controller_states()

    for (controller, state) in zip(self.cartesian_controllers, self.cartesian_controllers_state):
      if state == 'not loaded':
        success = self.load_controller_service(controller)
        if not success:
          rospy.logerr("error in loading Cartesian controller!")
          break  
    else:
      rospy.loginfo("all Cartesian controllers loaded")

    self.check_controller_states()


  ##unload the Cartesian controllers (to re-load with new params)
  def unload_cartesian_controllers(self):

    self.check_controller_states()
    if any([x=='running' for x in self.cartesian_controllers_state]):
      self.stop_controllers(stop_cartesian = 1)

    if all([x=='not loaded' for x in self.cartesian_controllers_state]):
      return

    cart_controllers_reversed = self.cartesian_controllers[:]
    cart_controllers_reversed.reverse()
    state_reversed = self.cartesian_controllers_state[:]
    state_reversed.reverse()
    for (controller, state) in zip(cart_controllers_reversed, state_reversed):
      if state == 'stopped':
        success = self.unload_controller_service(controller)
        if not success:
          rospy.logerr("error in unloading Cartesian controller!")
          break
    else:
      rospy.loginfo("Cartesian controllers unloaded")
    self.check_controller_states()


  ##load the joint controller with the current set of params on the param server
  def load_joint_controllers(self):
    self.check_controller_states()
    if self.joint_controller_state == 'not loaded':
      success = self.load_controller_service(self.joint_controller)
      if not success:
        rospy.logerr("error in loading joint controller!")
    else:
      rospy.loginfo("joint controller already loaded")
        
    self.check_controller_states()


  ##unload the joint controller 
  def unload_joint_controllers(self):
    self.check_controller_states()
    if self.joint_controller_state == 'running':
      self.stop_controllers(stop_joint = 1)
    elif self.joint_controller_state == 'not loaded':
      return

    success = self.unload_controller_service(self.joint_controller)
    if not success:
      rospy.logerr("error in unloading joint controller!")

    self.check_controller_states()
    

  ##stops the joint controllers, starts the Cartesian ones (both need to be loaded already)
  def switch_to_cartesian_mode(self):
    self.check_controller_states()
    if all([x=='stopped' for x in self.cartesian_controllers_state]) and \
          self.joint_controller_state == 'running':
      success = self.switch_controller_service(self.cartesian_controllers, [self.joint_controller,], 2)
      if success:
        rospy.loginfo("switched joint to Cartesian successfully")
      else:
        rospy.logerr("switching joint to Cartesian failed")
    else:
      self.start_cartesian_controllers()
      self.stop_controllers(stop_joint = 1)
    self.check_controller_states()


  ##stops the Cartesian controllers, starts the joint ones (both need to be loaded already)
  def switch_to_joint_mode(self):
    self.check_controller_states()
    if self.joint_controller_state == 'stopped' and \
          any([x=='running' for x in self.cartesian_controllers_state]):
      success = self.switch_controller_service([self.joint_controller,], self.cartesian_controllers, 2)
      if success:
        rospy.loginfo("switched Cartesian to joint successfully")
      else:
        rospy.logerr("switching Cartesian to joint failed")
    else:
      self.start_joint_controllers()
      self.stop_controllers(stop_cartesian = 1)
    self.check_controller_states()


  ##just start the joint controllers (need to be loaded already)
  def start_joint_controllers(self):
    self.check_controller_states()
    if self.joint_controller_state == 'stopped':
      success = self.switch_controller_service([self.joint_controller,], [], 2)
      if success:
        rospy.loginfo("started joint controller successfully")
      else:
        rospy.logerr("starting joint controller failed")
    elif self.joint_controller_state == 'not loaded':
      rospy.logerr("joint controller not loaded!")
    self.check_controller_states()


  ##start the Cartesian controllers (need to be loaded already)
  def start_cartesian_controllers(self):
    self.check_controller_states()
    if any([x=='not loaded' for x in self.cartesian_controllers_state]):
      print "not all Cartesian controllers are loaded!"

    elif any([x=='stopped' for x in self.cartesian_controllers_state]):
      success = self.switch_controller_service(self.cartesian_controllers, [], 2)
      if success:
        rospy.loginfo("started Cartesian controllers successfully")
      else:
        rospy.logerr("starting Cartesian controllers failed")
    self.check_controller_states()    


  ##start the gripper controller (needs to be loaded already)
  def start_gripper_controller(self):
    self.check_controller_states()
    if self.gripper_controller_state == 'stopped':
      success = self.switch_controller_service([self.gripper_controller,], [], 2)
      if success:
        rospy.loginfo("started gripper controller successfully")
      else:
        rospy.logerr("starting gripper controller failed")
    elif self.gripper_controller_state == 'not loaded':
      rospy.logerr("gripper controller isn't loaded!")
    self.check_controller_states()


  ##stop all controllers
  def stop_all_controllers(self):
      self.stop_controllers(stop_joint = 1, stop_cartesian = 1, stop_gripper = 1)


  ##stop controllers that are currently running
  def stop_controllers(self, stop_joint = 0, stop_cartesian = 0, stop_gripper = 0):
    self.check_controller_states()

    #stop Cartesian controllers
    if stop_cartesian and any([x=='running' for x in self.cartesian_controllers_state]):
      success = self.switch_controller_service([], self.cartesian_controllers, 2)
      if success:
        rospy.loginfo("stopped Cartesian controllers successfully")
      else:
        rospy.logerr("stopping Cartesian controllers failed")

    #stop joint controllers
    if stop_joint and self.joint_controller_state == 'running':
      success = self.switch_controller_service([], [self.joint_controller,], 2)
      if success:
        rospy.loginfo("stopped joint controller successfully")    
      else:
        rospy.logerr("stopping joint controller failed")  

    #stop gripper controller
    if stop_gripper and self.gripper_controller_state == 'running':
      success = self.switch_controller_service([], [self.gripper_controller,], 2)
      if success:
        rospy.loginfo("stopped gripper controller successfully")
      else:
        rospy.logerr("stopping gripper controller failed")

    self.check_controller_states()


  ##re-load the Cartesian controllers with new parameters (change cart_params before running)
  def reload_cartesian_controllers(self, set_params = 1):
    self.check_controller_states()

    #set the new params on the parameter server
    if set_params:
      self.cart_params.set_params()

    #if we're running the Cartesian controllers, let the joint controllers hold while we switch
    restart = 0
    if any([x=='running' for x in self.cartesian_controllers_state]):
      restart = 1
      self.switch_to_joint_mode()

    #unload the cartesian controllers
    self.unload_cartesian_controllers()

    #load the cartesian controllers
    self.load_cartesian_controllers()

    #switch back from joint to cartesian
    if restart:
      self.switch_to_cartesian_mode()

    self.check_controller_states()


  ##re-load the joint controller with gains a fraction of the defaults
  def reload_joint_controllers(self, fraction = 1.):
    self.check_controller_states()
    
    #set the new params on the parameter server
    self.joint_params.set_gains_fraction(fraction)
    
    #if we're running the joint controllers, let the Cartesian controllers hold while we switch
    restart = 0
    if self.joint_controller_state == 'running':
      restart = 1
      self.switch_to_cartesian_mode()

    #unload the joint controller
    self.unload_joint_controllers()
  
    #re-load the joint controllers
    self.load_joint_controllers()

    #switch back from Cartesian to joint
    if restart:
      self.switch_to_joint_mode()

    self.check_controller_states()
    

  ##return the current Cartesian pose of the gripper
  def return_cartesian_pose(self, frame = 'base_link'):
    (trans, rot) = self.tf_listener.lookupTransform(frame, self.whicharm+'_wrist_roll_link', rospy.Time(0))
    return (list(trans), list(rot))


  ##print the current Cartesian pose of the gripper
  def print_cartesian_pose(self):
    (trans, rot) = self.return_cartesian_pose()
    print "Cartesian pose trans:", self.pplist(trans), "rot:", self.pplist(rot)
    print "rotation matrix:\n", self.ppmat(tf.transformations.quaternion_matrix(rot))


  ##pretty-print list to string
  def pplist(self, list):
    return ' '.join(['%5.3f'%x for x in list])


  ##pretty-print numpy matrix to string
  def ppmat(self, mat):
    str = ''
    for i in range(mat.shape[0]):
      for j in range(mat.shape[1]):
        str += '%2.3f\t'%mat[i,j]
      str += '\n'
    return str



#sample test cases
if __name__ == '__main__':

  def keypause():
    print "press enter to continue"
    raw_input()

  rospy.init_node('controller_manager', anonymous=True)

#points 0-2 and 6-7 have IK solutions; 3-5 don't (but the cartesian-plain controllers will try to get you there anyway)
  points = [
    [0.5, -0.5, 0.8, 0.5, 0.0, 0.0, 0.5],
    [0.6, -0.2, 0.4, 0.0, 0.0, 0.5, 0.5],
    [0.2, -0.8, 0.4, 0.0, 0.5, 0.0, 0.5],
    [0.5, -0.5, 1.2, 0.5, 0.0, 0.0, 0.5],
    [0.6, -0.2, 1.2, 0.0, 0.0, 0.5, 0.5],
    [0.2, -0.8, 1.2, 0.0, 0.5, 0.0, 0.5],
    [.62, -.05, .65, -0.5, 0.5, 0.5, 0.5],
    [.62, -.05, .56, -0.5, 0.5, 0.5, 0.5]
  ]

  zeros = [0]*7
  sideangles = [-0.447, -0.297, -2.229, -0.719, 0.734, -1.489, 1.355]

  cm = ControllerManager('r')

  test_angles = [
  [-0.094, 0.711, -0.000, -1.413, -2.038, -1.172, -0.798],
  [0.126, 0.832, 0.000, -1.740, -2.977, -1.091, 3.043]]

  print "going through test_angles and waiting for the trajectory to finish"
  cm.command_joint_trajectory(test_angles)
  result = cm.wait_joint_trajectory_done(rospy.Duration(30.0))
  print result

  print "starting gripper controller"
  cm.start_gripper_controller()

  print "switching to cartesian controllers"
  cm.switch_to_cartesian_mode()

  print "moving to point 6 and waiting for it to get there"
  cm.command_cartesian(points[6])
  result = cm.wait_cartesian_really_done(points[6], .01, .1, rospy.Duration(30.0), rospy.Duration(10.0))
  print result
  #keypause()
  print "desired pose:", cm.pplist(points[6])
  cm.print_cartesian_pose()

  print "going to all 0 angles with joint controllers, testing contingency-switch"
  cm.command_joint(zeros)
  keypause()

  print "opening gripper"
  cm.command_gripper(0.08, -1.0)
  keypause()

  #cm.start_joint_controllers()

  print "going to near-side-grasp angles"
  cm.command_joint(sideangles)
  keypause()

#   print "going to all 0 angles and then to near-side-grasp angles"
#   cm.command_joint_trajectory([zeros, sideangles])
#   keypause()

  print "commanding an interpolated IK trajectory to go from side approach to side grasp"
  start_angles = sideangles
  tiltangle = math.pi/18.
  sideapproachmat = numpy.array([[0., -1., 0., 0.],  
                                 [math.cos(tiltangle), 0., math.sin(tiltangle), 0.],
                                 [-math.sin(tiltangle), 0., math.cos(tiltangle), 0.],
                                 [0., 0., 0., 1.]])
  sideapproachpos = [.62, -.3, .6]
  approachmat = sideapproachmat
  approachpos = sideapproachpos
  approachquat = list(tf.transformations.quaternion_from_matrix(approachmat))
  sidegrasppos = sideapproachpos[:]
  sidegrasppos[1] += .05
  grasppos = sidegrasppos
  graspquat = approachquat[:]

  start_pose = create_pose_stamped(approachpos+approachquat)
  end_pose = create_pose_stamped(grasppos+graspquat)
  #print "start_pose:\n", start_pose
  #print "end_pose:\n", end_pose
  cm.command_interpolated_ik(end_pose, start_pose)   
  keypause()

  print "closing gripper (blocking)"
  cm.command_gripper(0.0, 50.0, 1)
  print "done"

  print "moving to point 0 in Cartesian mode, testing contingency switch"
  cm.command_cartesian(points[0])
  keypause()
  cm.print_cartesian_pose()

  print "reloading cartesian controllers with zero PID terms (should be unable to move)"
  cm.cart_params.pose_fb_trans_p = 0.
  cm.cart_params.pose_fb_trans_d = 0.
  cm.cart_params.pose_fb_rot_p = 0.
  cm.cart_params.pose_fb_rot_d = 0.
  cm.reload_cartesian_controllers()

  print "trying to move to point 2 (should fail)"
  cm.command_cartesian(points[2])
  keypause()

  print "reloading cartesian controllers with normal PID terms"
  cm.cart_params.pose_fb_trans_p=800.
  cm.cart_params.pose_fb_trans_d=12.
  cm.cart_params.pose_fb_rot_p=80.
  cm.cart_params.pose_fb_rot_d=0.0
  cm.reload_cartesian_controllers()
  
  print "trying again to move to point 2 (should succeed)"
  cm.command_cartesian(points[2])
  keypause()

  print "starting joint controllers and going to all 0 angles"
  cm.command_joint([0,0,0,0,0,0,0])
  keypause()

  print "stopping all controllers"
  cm.stop_all_controllers()
  

