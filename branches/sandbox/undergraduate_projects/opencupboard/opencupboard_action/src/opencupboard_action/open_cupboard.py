#!/usr/bin/env python
import roslib
roslib.load_manifest('opencupboard_action')
import sys
import rospy
from opencupboard_msgs.srv import *
from geometry_msgs.msg import Pose, PoseStamped
from tabletop_object_detector.msg import Table
from pr2_pick_and_place_demos.pick_and_place_manager import PickAndPlaceManager
import actionlib
import opencupboard_msgs.msg
#from opendoors_executive.control_switcher import PR2CMClient
from ee_cart_imped_msgs.msg import EECartImpedGoal, EECartImpedAction, StiffPoint
import pr2_controllers_msgs.msg
import time
import trajectory_msgs.msg
import sensor_msgs
import tf
from pr2_utils import arm_control

class OpenCupboardAction():
    def __init__(self, name):
        '''
        Begins the action to open the cupboard
        '''
        rospy.loginfo("IN ACTION")
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, opencupboard_msgs.msg.OpenCupboardAction, execute_cb=self.main1)
        self._as.start()
        self.ppm = PickAndPlaceManager()
        self.r_arm_control = arm_control.ArmControl("right_arm")
        self.l_arm_control = arm_control.ArmControl("left_arm")
        self.s = rospy.Service('table_status', opencupboard_msgs.srv.table_status, self.table_status)
        print "Ready to check table status"
        self.result = opencupboard_msgs.msg.OpenCupboardResult()
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
        goal = trajectory_msgs.msg.JointTrajectory()
        goal.header.stamp = rospy.Time.now() + rospy.Duration(0.5)
        goal.joint_names.append("r_upper_arm_roll_joint");
        goal.joint_names.append("r_shoulder_pan_joint");
        goal.joint_names.append("r_shoulder_lift_joint");
        goal.joint_names.append("r_forearm_roll_joint");
        goal.joint_names.append("r_elbow_flex_joint");
        goal.joint_names.append("r_wrist_flex_joint");
        goal.joint_names.append("r_wrist_roll_joint");
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = r_joint_positions
        for i in range(7): 
            point.velocities.append(0)
        point.time_from_start = rospy.Duration(4.0)
        goal.points.append(point)
        rospy.loginfo("Moving to joint goal")
        self.r_arm_control.execute_joint_trajectory(goal)

        # left arm
        goal = trajectory_msgs.msg.JointTrajectory()
        goal.header.stamp = rospy.Time.now() + rospy.Duration(0.5)
        goal.joint_names.append("l_upper_arm_roll_joint");
        goal.joint_names.append("l_shoulder_pan_joint");
        goal.joint_names.append("l_shoulder_lift_joint");
        goal.joint_names.append("l_forearm_roll_joint");
        goal.joint_names.append("l_elbow_flex_joint");
        goal.joint_names.append("l_wrist_flex_joint");
        goal.joint_names.append("l_wrist_roll_joint");
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = l_joint_positions
        for i in range(7): 
            point.velocities.append(0)
        point.time_from_start = rospy.Duration(4.0)
        goal.points.append(point)
        rospy.loginfo("Moving to joint goal")
        self.l_arm_control.execute_joint_trajectory(goal)

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
        @param head_z: z coordinate of the target point for head to look at 
                       (changes when you adjust the head if you're not finding the handle) 
        '''
        initial_r_joint_positions = [-1.0, -2.0, 0.0, -1.0, -1.3, -1.0, -3.3]
        initial_l_joint_positions = [1.0, 2.0, 0.0, 1.0, -1.3, -1.0, -3.3]
        
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
        @returns the detected cabinet door                                                               '''
        (things, table) = ppm.call_tabletop_detection(update_table=1)
        return table

    def door_processing_client(self):
        '''
        Uses tabletop detection to find the cabinet door. Returns the door and 
        the list of clusters associated with the door.
        
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
        @param detection_counter: keeps track of the number of detections done 
                                  and readjusts the head every 3 detections
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
        
        gripper_client = actionlib.SimpleActionClient("grasp_handle_action", opencupboard_msgs.msg.GraspHandleAction)
        rospy.loginfo("Waiting for grasp_handle_action")
        gripper_client.wait_for_server()
        
        goal = opencupboard_msgs.msg.GraspHandleGoal()
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
        @returns: whether the grasp was successfully completed (the gripper was not closed 
                  completely meaning it was holding something, namely the handle)
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
  
    def make_point(self, handle_pos, point):
        '''
        Creates the goal point and forces for the force controller.

        Uses the param door_width, and a given handle and handle orientation, 
        creates the proper forces and torques and a goal point. Assumes a 
        handle on the top or bottom is horizontal and one on the left or right
        is vertical.
        @type handle_pos: string
        @param handle_pos: handle orientation on cabinet: "top","bot","left","right"
        @type point: geometry_msgs.msg.Point
        @param point: the detected handle coords
        @rtype: list of geometry_msgs.msg.Point
        @returns: pose: goal handle position and orientation,
                  force: the forces in each axes, 
                  forces: the booleans for force or stiffness in each axes 
                  angle: the torque in each axes
        '''
        door_width = rospy.get_param('/door_width', 0.18)
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

    def make_point_test(self, handle_pos, point):
        '''
        Creates the goal point and forces for the force controller.

        Uses the param door_width, and a given handle and handle orientation, 
        creates the proper forces and torques and a goal point. Assumes a 
        handle on the top or bottom is horizontal and one on the left or right
        is vertical.
        @type handle_pos: string
        @param handle_pos: handle orientation on cabinet: "top","bot","left","right"
        @type point: geometry_msgs.msg.Point
        @param point: the detected handle coords
        @rtype: list of geometry_msgs.msg.Point
        @returns: pose: goal handle position and orientation,
                  force: the forces in each axes, 
                  forces: the booleans for force or stiffness in each axes 
                  angle: the torque in each axes
        '''
        door_width = rospy.get_param('/door_width', 0.18)
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
            pose.position.x = point.x #point.x-door_width
            pose.position.y = point.y-door_width #point.y
            pose.position.z = point.z
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            force.x = 30 #1000 #stiffness
            force.y = 1000 #30
            force.z = 100
            forces = (True,False,False) #(False,True,False)
            angle.x = 50 #10  
            angle.y = 10 #50 
            angle.z = 40
        return (pose, force, forces, angle)

    def test_force_control(self, point):
        '''
        Opens the door using force controller

        Uses parameters of the door_width, handle_pos, and which_arm to open 
        the door assuming the door opens with the handle opposite the hinge 
        and towards the robot.
        @type point: geometry_msgs.msg.Point
        @param point: the detected handle position
        @rtype: boolean
        @returns: whether the opening was successful. namely whether the hand is 
        at the goal position (assumes the hand is still holding the handle)
        '''
        door_width = rospy.get_param('/door_width', 0.18)
        handle_pos = rospy.get_param('/handle_pos', "left") # assume handle opp of hinges
        which_arm = rospy.get_param('which_arm', "left_arm")
        my_arm_control = arm_control.ArmControl(which_arm)
        if which_arm == "left_arm":
            which_controller = "l_arm_cart_imped_controller/ee_cart_imped_action"
        else:
            which_controller = "r_arm_cart_imped_controller/ee_cart_imped_action"
        orig_point = geometry_msgs.msg.PointStamped()
        orig_point.header.frame_id = '/base_link'
        orig_point.point = point
        open_point = self.ppm.tf_listener.transformPoint('/torso_lift_link',orig_point)
        (goal_pose, goal_force, goal_forces, goal_angle) = self.make_point_test(handle_pos, open_point.point)
        print "waiting for server"
        goal = EECartImpedGoal() 
        goal.header.frame_id = "torso_lift_link"
        goal.header.stamp = rospy.Time.now()
        my_arm_control.add_trajectory_point_to_force_control(goal_pose.position.x, goal_pose.position.y, goal_pose.position.z, 
                                 goal_pose.orientation.x, goal_pose.orientation.y, goal_pose.orientation.z, goal_pose.orientation.w,
                                 goal_force.x, goal_force.y, goal_force.z, goal_angle.x, goal_angle.y, goal_angle.z,
                                 goal_forces[0], goal_forces[1], goal_forces[2], False, False, False, 15)

        print "sending goal"
        my_arm_control.executeForceControl()
        print "waiting for result"	
	my_arm_control.stop_in_place()
#        rospy.sleep(20)
 
        for i in range(3):
            if self.check_open():
                return True

        return False
            
    def check_open(self):
        '''
        Checks that door is all the way open
        
        Determines success by whether the hand is very close to the goal point.
        Assumes the hand is still holding the handle.
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
        '''
        Executes the cabinet opening.

        Detects the handle. Moves collision free to directly in front of the 
        handle. Moves forward to grasp handle. Uses the force controller 
        to open the door.
        @rtype: boolean 
        @returns: whether the action was successful. 
        '''
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
        '''
        The callback. Calls main and repeats if it was not successful.

        @type goal: boolean
        @param goal: True to open the cabinet.
        '''
        success = self.main()
        if not success:
            self.main1(goal)

    def table_status(self):
        '''
        Service to determine whether the robot is with in the correct 
        range to open the door. If false, returns the distance needed
        to move to get into the correct range. 
        '''
        self.setup(0.0)
        print "TABLE STATUS=="
        return opencupboard_msgs.srv.table_statusResponse(self.check_table_pos())

if __name__=="__main__":
    rospy.init_node("open_cupboard_action")
    rospy.loginfo("name=%s", str(rospy.get_name()))
    OpenCupboardAction(rospy.get_name())
    rospy.loginfo('Ready to start open_cupboard action')
    rospy.spin()


# I GOT RID OF REQ AS A PARAM TO TABLE_STATUS. MAKE SURE IT DOESNT BREAK
