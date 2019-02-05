#!/usr/bin/env python
import roslib
roslib.load_manifest('opendoors_executive')
import sys
import rospy
from tabletop_for_cabinet.srv import *
from geometry_msgs.msg import Pose, PoseStamped
from tabletop_object_detector.msg import Table
from pr2_pick_and_place_demos.pick_and_place_manager import PickAndPlaceManager
import actionlib
import opendoors.msg
from opendoors_executive.control_switcher import PR2CMClient
from ee_cart_imped_control.msg import EECartImpedGoal, EECartImpedAction, StiffPoint
import pr2_controllers_msgs.msg
import time
import trajectory_msgs.msg
import sensor_msgs
import tf
import opendoors_executive.msg
import opendoors_executive.srv

class OpenCupboardAction():
    def __init__(self, name):
        '''
        Begins the action to open the cupboard
        '''
        rospy.loginfo("IN ACTION")
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, opendoors_executive.msg.OpenCupboardAction, execute_cb=self.main1)
        self._as.start()
        self.ppm = PickAndPlaceManager()
        self.s = rospy.Service('table_status', opendoors_executive.srv.table_status, self.table_status)
        print "Ready to check table status"
        self.result = opendoors_executive.msg.OpenCupboardResult()
        '''
        _action_name: name of the action
        _as: action server
        ppm: pick and place manager
        s: service to check whether the door is within the right range
        result: success of the opening
        '''

    def lift_torso(self):
        '''
        Lifts the torso completely
        '''
        torso_client =\
            actionlib.SimpleActionClient\
            ("torso_controller/position_joint_action",\
                 pr2_controllers_msgs.msg.SingleJointPositionAction)
        rospy.loginfo("open_cupboard: waiting for torso action")
        torso_client.wait_for_server()
        rospy.loginfo("open_cupboard: found torso action")
        goal = pr2_controllers_msgs.msg.SingleJointPositionGoal()
        goal.position = 0.3
        rospy.loginfo("open_cupboard: Attempting to raise torso")
        torso_client.send_goal_and_wait(goal)

    def move_arm_to_joint_goal(self, r_joint_positions, l_joint_positions):
        '''
        Moves the arms out of the way for detection

        @type r_joint_positions: trajectory_msgs.msg.JointTrajectoryPoint
        @param r_joint_positions: right arm's goal joint positions
        @type l_joint_positions: trajectory_msgs.msg.JointTrajectoryPoint
        @param l_joint_positions: left arm's goal joint positions
        '''
        # right arm
        traj_client =\
            actionlib.SimpleActionClient\
            ("r_arm_controller/joint_trajectory_action",\
                 pr2_controllers_msgs.msg.JointTrajectoryAction)
        rospy.loginfo("open_cupboard: waiting for joint trajectory_action")
        traj_client.wait_for_server()
        rospy.loginfo("open_cupboard: found joint trajectory action")
        goal = pr2_controllers_msgs.msg.JointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.5)
        goal.trajectory.joint_names.append("r_upper_arm_roll_joint");
        goal.trajectory.joint_names.append("r_shoulder_pan_joint");
        goal.trajectory.joint_names.append("r_shoulder_lift_joint");
        goal.trajectory.joint_names.append("r_forearm_roll_joint");
        goal.trajectory.joint_names.append("r_elbow_flex_joint");
        goal.trajectory.joint_names.append("r_wrist_flex_joint");
        goal.trajectory.joint_names.append("r_wrist_roll_joint");
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = r_joint_positions
        for i in range(7): 
            point.velocities.append(0)
        point.time_from_start = rospy.Duration(4.0)
        goal.trajectory.points.append(point)
        rospy.loginfo("Moving to joint goal")
        traj_client.send_goal_and_wait(goal)

        # left arm
        traj_client =\
            actionlib.SimpleActionClient\
            ("l_arm_controller/joint_trajectory_action",\
                 pr2_controllers_msgs.msg.JointTrajectoryAction)
        rospy.loginfo("open_cupboard: waiting for joint trajectory_action")
        traj_client.wait_for_server()
        rospy.loginfo("open_cupboard: found joint trajectory action")
        goal = pr2_controllers_msgs.msg.JointTrajectoryGoal()
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.5)
        goal.trajectory.joint_names.append("l_upper_arm_roll_joint");
        goal.trajectory.joint_names.append("l_shoulder_pan_joint");
        goal.trajectory.joint_names.append("l_shoulder_lift_joint");
        goal.trajectory.joint_names.append("l_forearm_roll_joint");
        goal.trajectory.joint_names.append("l_elbow_flex_joint");
        goal.trajectory.joint_names.append("l_wrist_flex_joint");
        goal.trajectory.joint_names.append("l_wrist_roll_joint");
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = l_joint_positions
        for i in range(7): 
            point.velocities.append(0)
        point.time_from_start = rospy.Duration(4.0)
        goal.trajectory.points.append(point)
        rospy.loginfo("Moving to joint goal")
        traj_client.send_goal_and_wait(goal)

    def point_head_at(self, point_stamped):
        '''
        Points the head at the door

        @type point_stamped: geometry_msgs.msg.PointStamped
        @param point_stamped: target point for head to look at
        '''
        head_client =\
            actionlib.SimpleActionClient\
            ("head_traj_controller/point_head_action",\
                 pr2_controllers_msgs.msg.PointHeadAction)
        rospy.loginfo("open_cupboard: waiting for point_head_action")
        head_client.wait_for_server()
        rospy.loginfo("open_cupboard: found point_head_action")
        goal = pr2_controllers_msgs.msg.PointHeadGoal()
        goal.target = point_stamped
        goal.pointing_frame = "/narrow_stereo_optical_frame"
        goal.max_velocity = 1.0
        rospy.loginfo("open_cupboard: attempting to point head")
        head_client.send_goal_and_wait(goal)
        rospy.loginfo("open_cupboard: sleeping after head move to allow a new point cloud to be generated")
        time.sleep(1.0)

    def setup(self, head_z):
        '''
        Moves the robot into initial config to open door: arms, head, and torso

        @type head_z: float
        @param head_z: z coordinate of the target point for head to look at (changes when you adjust the head if you're not finding the handle) 
        '''
        initial_r_joint_positions = [-1.0, -2.0, 0.0, -1.0, -1.3, -1.0, -3.3]
        initial_l_joint_positions = [1.0, 2.0, 0.0, 1.0, -1.3, -1.0, -3.3]
        PR2CMClient.load_cartesian(False)
        PR2CMClient.load_cartesian(True)
        
        self.lift_torso()
        self.move_arm_to_joint_goal(initial_r_joint_positions,initial_l_joint_positions)

        # check for params
        required_head_position = geometry_msgs.msg.PointStamped()
        required_head_position.header.frame_id = "base_link"
        adjust_head = rospy.get_param('/adjust_head', [0.8, 0.0, 0.65])
        required_head_position.point.x = 0.0 + adjust_head[0] # 0.8
        required_head_position.point.y = 0.0 + adjust_head[1] # -0.25
        required_head_position.point.z = 0.0 + head_z + adjust_head[2] 
        self.point_head_at(required_head_position)

    def check_table_pos(self):
        '''
        Checks that the table is within the right distance
        
        @rtype: float
        @returns dist to move towards the table, 1000 if horizontal
        '''
        if rospy.has_param('/table_dist'):
            table = self.get_table(self.ppm)
            if table == None:
                return 1000
            actual_table_dist = table.pose.pose.position.x
            table_dist = rospy.get_param('/table_dist', 0.8)
            if table.pose.pose.orientation.w < 0.95:
                if abs(actual_table_dist-table_dist) > 0.15:
                    return actual_table_dist-table_dist
                else:
                    return 0.0
            else:
                return 1000
        else: # can teleop to the correct distance
            return 0.0

    def get_table(self, ppm):
        '''   
        Uses tabletop detection to find a plane which corresponds to the cabinet door 

        @type ppm: PickAndPlaceManager
        @param ppm: an instance of a PickAndPlaceManager
        @rtype: tabletop_object_detector/Table                                         
        @returns the detected cabinet door                                                                       '''
        (things,table) = ppm.call_tabletop_detection(take_static_collision_map=1, update_table=1)
        return table

    def door_processing_client(self):
        '''
        Uses tabletop detection to find the cabinet door. Returns the door and the list of clusters associated with the door.
        
        @rtype: tabletop_object_detector/TabletopDetectionResult
        @returns the cabinet door as a table and the vector of the clusters of possible handles
        '''
        rospy.loginfo('entering door_processing client')
        rospy.wait_for_service('cabinet_table')
        rospy.loginfo('got door service')
        try:
            door_processing = rospy.ServiceProxy('cabinet_table', cabinet_table)
            resp1 = door_processing(self.get_table(self.ppm))
            return resp1
        except rospy.ServiceException, e:
            rospy.logerr("Service call to cabinet_table failed")

    def object_processing_client(self):
        '''                      
        Finds the handle on the cabinet door and returns its pose.   
        
        @rtype: geometry_msgs/PoseStamped                                               
        @returns the pose of the handle on the cabinet door      
        '''
        rospy.loginfo('entering object_processing_client')
        rospy.wait_for_service('cabinet_table_result')
        rospy.loginfo('got object service')
        try:
            object_processing = rospy.ServiceProxy('cabinet_table_result', cabinet_table_result)
            resp1 = object_processing(self.door_processing_client().detection)
            rospy.loginfo("DETECTION HANDLE_RESP=%s", str(resp1))
            return resp1
        except rospy.ServiceException, e:
            rospy.logerr("Service call to cabinet_table_result failed")
                
    def check_detection(self, handle_resp, detection_counter):
        '''
        Checks the detection. if detection returns no handle, it will try again 3 more times,
        then it will jiggle the head and restart the process
        
        @type handle_resp: tabletop_for_cabinet.srv.cabinet_table_result
        @param handle_resp: the PoseStamped and ClusterBoundingBox of the handle
        @type detection_counter: int
        @param detection_counter: keeps track of the number of detections done and readjusts the head every 3 detections
        @rtype: geometry_msgs/PoseStamped
        @returns the pose of the handle
        '''
        # check handle detection worked
        if ((handle_resp.pose.pose.position.x == 0.0) and (handle_resp.pose.pose.position.y == 0.0) and (handle_resp.pose.pose.position.z == 0.0)):
            detection_counter = detection_counter + 1
            if detection_counter == 3:
            #jiggle head
                self.setup(0.1)
                self.setup(0.01)
                detection_counter = 0
            return self.check_detection(self.object_processing_client(),detection_counter)
            
        else:
            rospy.loginfo("HANDLE POSE=%s", str(handle_resp.pose))
            return (handle_resp, True)

    def move_arm(self, handle_resp, grasp_counter):
        '''
        Moves the arm to the handle and grasps it

        @type handle_resp: tabletop_for_cabinet.srv.cabinet_table_result
        @param handle_resp: the PoseStamped and ClusterBoundingBox of the handle
        @type grasp_counter: int 
        @param grasp_counter: keeps track of grasp tries. returns false when over 5
        @rtype: boolean
        @returns: whether the move_arm was successfully completed
        '''
        rospy.loginfo("HANDLE_RESP=%s", str(handle_resp))
        grasp_counter = grasp_counter + 1
        
        gripper_client = actionlib.SimpleActionClient("grasp_handle_action", opendoors.msg.GraspHandleAction)
        rospy.loginfo("Waiting for grasp_handle_action")
        gripper_client.wait_for_server()
        
        goal = opendoors.msg.GraspHandleGoal()
        goal.pose_stamped = handle_resp.pose
        goal.handle_box = handle_resp.box

        which_arm = rospy.get_param('/which_arm', 'left_arm')
        goal.arm = which_arm
        
        rospy.loginfo("GOAL=%s", str(goal))
        rospy.loginfo("Waiting for result of grasp_handle_action")
        result = gripper_client.send_goal_and_wait(goal)
        
        rospy.loginfo("GRASP RESULT = %d", result)
        
        if grasp_counter == 5:
            # start over
            return False

        if result != actionlib.GoalStatus.SUCCEEDED:
            self.move_arm(handle_resp, grasp_counter)

        return True

    def check_grasp(self, handle_resp, grasp_counter):
        '''
        Checks that the gripper is fully closed after the grasp.
        if not, restart

        @type handle_resp: tabletop_for_cabinet.srv.cabinet_table_result
        @param handle_resp: the PoseStamped and ClusterBoundingBox of the handle
        @type grasp_counter: int 
        @param grasp_counter: keeps track of grasp tries. returns false when over 5
        @rtype: boolean
        @returns: whether the grasp was successfully completed (the gripper was not closed completely meaning it was holding something, namely the handle)
        '''
        move_arm_suc = self.move_arm(handle_resp, grasp_counter)
        if not move_arm_suc:
            return False
        
        joint_states= rospy.client.wait_for_message('/joint_states',sensor_msgs.msg.JointState)
        which_arm = rospy.get_param('/which_arm', "left_arm")
        if which_arm == "left_arm":
            joint = 'l_gripper_joint'
        else:
            joint = 'r_gripper_joint'
        for i in range(len(joint_states.name)):
            if joint_states.name[i] == joint:
                gripper_status = joint_states.position[i]
        
        if gripper_status < 0.01:
            # gripper closed
            return False
        return True
                        
    def addTrajectoryPoint(self, goal, x, y, z, ox, oy, oz, ow,
                           fx, fy, fz, tx, ty, tz, isfx, isfy, isfz,
                           istx, isty, istz, time):
        '''
        Creates a new point for force control
        '''
        new_point = len(goal.trajectory)
        #goal.trajectory.resize(new_point+1)
        goal.trajectory.append(StiffPoint())
        goal.trajectory[new_point].pose.position.x = x
        goal.trajectory[new_point].pose.position.y = y
        goal.trajectory[new_point].pose.position.z = z
        goal.trajectory[new_point].pose.orientation.x = ox
        goal.trajectory[new_point].pose.orientation.y = oy
        goal.trajectory[new_point].pose.orientation.z = oz
        goal.trajectory[new_point].pose.orientation.w = ow
        goal.trajectory[new_point].wrench_or_stiffness.force.x = fx
        goal.trajectory[new_point].wrench_or_stiffness.force.y = fy
        goal.trajectory[new_point].wrench_or_stiffness.force.z = fz
        goal.trajectory[new_point].wrench_or_stiffness.torque.x = tx
        goal.trajectory[new_point].wrench_or_stiffness.torque.y = ty
        goal.trajectory[new_point].wrench_or_stiffness.torque.z = tz
        
        goal.trajectory[new_point].isForceX = isfx
        goal.trajectory[new_point].isForceY = isfy
        goal.trajectory[new_point].isForceZ = isfz
        goal.trajectory[new_point].isTorqueX = istx
        goal.trajectory[new_point].isTorqueY = isty
        goal.trajectory[new_point].isTorqueZ = istz
        goal.trajectory[new_point].time_from_start = rospy.Duration(time)

    def make_point(self, handle_pos, point):
        door_width = rospy.get_param('/door_width', 0.18)
        #door_width = door_width+0.1
        pose = geometry_msgs.msg.Pose()
        force = geometry_msgs.msg.Point()
        angle = geometry_msgs.msg.Point()
        if handle_pos == "top":
            pose.position.x = point.x-door_width
            pose.position.y = point.y
            pose.position.z = point.z
            pose.orientation.x = 0.707
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.707
            force.x = -30#1000 #stiffness
            force.y = 100
            force.z = -30#-30
            forces = (True, False, True) #(False,False,True)
            angle.x = 10  
            angle.y = 50 
            angle.z = 40
        elif handle_pos == "left":
            pose.position.x = point.x-door_width
            pose.position.y = point.y
            pose.position.z = point.z
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            force.x = 1000 #stiffness
            force.y = -30
            force.z = 100
            forces = (False,True,False)
            angle.x = 10  
            angle.y = 50 
            angle.z = 40
        elif handle_pos == "bot":
            pose.position.x = point.x-door_width
            pose.position.y = point.y
            pose.position.z = point.z
            pose.orientation.x = 0.707
            pose.orientation.y = 0.0 
            pose.orientation.z = 0.0
            pose.orientation.w = 0.707
            force.x = 1000 #stiffness
            force.y = 100
            force.z = 30
            forces = (False,False,True)
            angle.x = 10  
            angle.y = 50 
            angle.z = 40
        else:
            pose.position.x = point.x-door_width
            pose.position.y = point.y
            pose.position.z = point.z
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            force.x = 1000 #stiffness
            force.y = 30
            force.z = 100
            forces = (False,True,False)
            angle.x = 10  
            angle.y = 50 
            angle.z = 40
        return (pose, force, forces, angle)

    def test_force_control(self, point):
        '''
        Opens the door using force controller
        '''
        door_width = rospy.get_param('/door_width', 0.18)
        handle_pos = rospy.get_param('/handle_pos', "left") # assume handle opp of hinges
        which_arm = rospy.get_param('which_arm', "left_arm")
        if which_arm == "left_arm":
            which_controller = "l_arm_cart_imped_controller/ee_cart_imped_action"
        else:
            which_controller = "r_arm_cart_imped_controller/ee_cart_imped_action"
        orig_point = geometry_msgs.msg.PointStamped()
        orig_point.header.frame_id = '/base_link'
        orig_point.point = point
        open_point = self.ppm.tf_listener.transformPoint('/torso_lift_link',orig_point)
        (goal_pose, goal_force, goal_forces, goal_angle) = self.make_point(handle_pos, open_point.point)
        ee_cart_imped_client =\
            actionlib.SimpleActionClient\
            (which_controller, EECartImpedAction)
        ee_cart_imped_client.wait_for_server(rospy.Duration(5.0))
        print "waiting for server"
        goal = EECartImpedGoal() 
        goal.header.frame_id = "torso_lift_link"
        goal.header.stamp = rospy.Time.now()
        self.addTrajectoryPoint (goal, goal_pose.position.x, goal_pose.position.y, goal_pose.position.z, 
                                 goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w,
                                 goal_force.x, goal_force.y, goal_force.z, goal_angle.x, goal_angle.y, goal_angle.z,
                                 goal_forces[0], goal_forces[1], goal_forces[2], False, False, False, 15)

#        self.addTrajectoryPoint (goal, goal_pose.position.x, goal_pose.position.y, goal_pose.position.z, 
#                                 goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w,
#                                 0.0, 0.0, 0.0, 10, 50, 40,
#                                 False, False, False, False, False, False, 20)        
        print "sending goal"
        ee_cart_imped_client.send_goal(goal)
        print "waiting for result"		
        result = ee_cart_imped_client.wait_for_result()
        print "OPEN RESULT=", result
        #rospy.sleep(20)
        rospy.sleep(5)
        PR2CMClient.load_cartesian(False)
        PR2CMClient.load_cartesian(True)
 
        for i in range(3):
            if self.check_open():
                return True
#            else:
#                rospy.loginfo("NOT OPEN")
#                self.addTrajectoryPoint (goal, point.x-door_width-((i+1.0)*0.03), 0, 0, 0.707, 0, 0, 0.707,
#                                         1000, 0, -20, 10, 50, 40,
#                                         False, False, True, False, False, False, 5)
#                ee_cart_imped_client.send_goal(goal)
#                ee_cart_imped_client.wait_for_result()
        return False
            
    def check_open(self):
        '''
        Checks that door is all the way open
        
        @rtype: bool
        @returns True if open, False if not complete
        '''
        which_arm = rospy.get_param('which_arm', "left_arm")
        if which_arm == "left_arm":
            which_frame = "/l_gripper_tool_frame"
        else:
            which_frame = "/r_gripper_tool_frame"

        if rospy.has_param('/open_door_pos'):
            # sleep until action finished
            rospy.sleep(20)
            (trans,rot) = self.ppm.tf_listener.lookupTransform('/base_link', which_frame, rospy.Time(0))
            open_door_pos = rospy.get_param('/open_door_pos', 0.640)
            handle_pos = rospy.get_param('/handle_pos', "left")
            if handle_pos=="top" or hanlde_pos=="bot":
                rospy.loginfo("FINAL POS===%f",trans[2]) # checking z-coord
                if (trans[2] > (open_door_pos+0.04)):
                    return False
                else:
                    return True
            else:
                rospy.loginfo("FINAL POS===%f",trans[1]) # checking y-coord            
                if (trans[1] > (open_door_pos+0.04)):
                    return False
                else:
                    return True                
        else: # assume successful
            return True
 
    def main(self):
        detection_counter = 0
        grasp_counter = 0 
        
        self.setup(0.0)
        if self.check_table_pos() == 0.0:
            # detection
            (handle_resp, detect_suc) = self.check_detection(self.object_processing_client(),detection_counter);
            # grasphandle
            grasp_suc = self.check_grasp(handle_resp, grasp_counter)
            rospy.loginfo("FINISHED GRASP")
            grasp_suc = True
            if not grasp_suc:
                return False
            
            # open door
            which_arm = rospy.get_param('/which_arm', 'left_arm')
            if which_arm == "left_arm":
                controller_arm = False
            else:
                controller_arm = True
            PR2CMClient.load_ee_cart_imped(controller_arm) # False for left arm
            success = self.test_force_control(handle_resp.pose.pose.position)
            
            if success:
                self.result.success= True
                self._as.set_succeeded(self.result)
            else:
                rospy.loginfo("DOOR COULD NOT BE OPENED")
                self.result.success = False
                self._as.set_aborted(self.result)

            return True
        else:
            rospy.loginfo('TABLE POS=%f', self.check_table_pos())
            rospy.loginfo('TABLE NOT IN RANGE')
            self.result.success= False
            self._as.set_aborted(self.result)

    def main1(self, goal):
        success = self.main()
        if not success:
            self.main1(goal)

    def table_status(self, req):
        self.setup(0.0)
        print "TABLE STATUS=="
        return opendoors_executive.srv.table_statusResponse(self.check_table_pos())

if __name__=="__main__":
    rospy.init_node("open_cupboard_action")
    rospy.loginfo("name=%s", str(rospy.get_name()))
    OpenCupboardAction(rospy.get_name())
    rospy.loginfo('Ready to start open_cupboard action')
    rospy.spin()
