#! /usr/bin/env python


import roslib; roslib.load_manifest('ee_cart_imped_action')
import rospy
import actionlib
import ee_cart_imped_msgs.msg

MAX_TRANS_STIFFNESS=1000.0
'''
Maximum allowed stiffness for position.
'''

MAX_ROT_STIFFNESS=100.0
'''
Maximum allowed stiffness for rotation.
'''


class EECartImpedClient:
    '''
    A wrapper around the simple simple action client for the
    EECartImpedAction.
    
    See the actionlib documentation for more details on action clients.
    '''
    def __init__(self, arm_name):
        '''
        Initialization of the client.
        @type arm_name: string
        @param arm_name: Which arm.  Must either be 'right_arm' or 'left_arm'.
        '''
        
        self.arm_name = arm_name
        '''
        The name of the arm this client executes on.
        '''
        
        self.client =\
            actionlib.SimpleActionClient\
            ('/'+arm_name[0]+\
                 '_arm_cart_imped_controller/ee_cart_imped_action',\
                 ee_cart_imped_msgs.msg.EECartImpedAction)
        '''
        The simple action client used to communicate with the action server.
        '''
        
        rospy.loginfo("Waiting for ee_cart_imped action server for arm %s",
                      self.arm_name)
        self.client.wait_for_server()
        rospy.loginfo("Found ee_cart_imped action server")

        self.goal = ee_cart_imped_msgs.msg.EECartImpedGoal()
        '''
        The current stored trajectory.
        '''

        self.resetGoal()

    def moveToPoseStamped(self, pose_stamped, time):
        '''
        Moves the arm to a pose stamped using the force/impedance controller.
        @type pose_stamped: geometry_msgs.msg.PoseStamped
        @param pose_stamped: The pose to move the end effector to.
        @type time: double
        @param time: The time after which this goal should be completed.
        '''
        goal = ee_cart_imped_msgs.msg.EECartImpedGoal()
        new_point = len(goal.trajectory);
        goal.trajectory.append(ee_cart_imped_msgs.msg.StiffPoint())
        goal.trajectory[new_point].header.stamp =\
            pose_stamped.header.stamp
        goal.trajectory[new_point].header.frame_id =\
            pose_stamped.header.frame_id
        goal.trajectory[new_point].pose = pose_stamped.pose
        goal.trajectory[new_point].wrench_or_stiffness.force.x =\
            MAX_TRANS_STIFFNESS;
        goal.trajectory[new_point].wrench_or_stiffness.force.y =\
            MAX_TRANS_STIFFNESS;
        goal.trajectory[new_point].wrench_or_stiffness.force.z =\
            MAX_TRANS_STIFFNESS;
        goal.trajectory[new_point].wrench_or_stiffness.torque.x =\
            MAX_ROT_STIFFNESS;
        goal.trajectory[new_point].wrench_or_stiffness.torque.y =\
            MAX_ROT_STIFFNESS;
        goal.trajectory[new_point].wrench_or_stiffness.torque.z =\
            MAX_ROT_STIFFNESS;
        goal.trajectory[new_point].isForceX = False
        goal.trajectory[new_point].isForceY = False
        goal.trajectory[new_point].isForceZ = False
        goal.trajectory[new_point].isTorqueX = False
        goal.trajectory[new_point].isTorqueY = False
        goal.trajectory[new_point].isTorqueZ = False
        goal.trajectory[new_point].time_from_start =\
            rospy.Duration(time);
        goal.header.stamp = rospy.Time.now()
        rospy.logdebug('Sending pose goal %s', str(goal))
        self.client.send_goal_and_wait(goal, 
                                       rospy.Duration(time + 2))


    def addTrajectoryPoint(self, x, y, z, ox, oy, oz, ow,
                           fx, fy, fz, tx, ty, tz, isfx, isfy, isfz,
                           istx, isty, istz, time, frame_id=''):
        '''
        Add a trajectory point to the current goal.
        @type x: double
        @param x: x position of the end effector
        @type y: double
        @param y: y position of the end effector
        @type z: double
        @param z: z position of the end effector
        @type ox: double
        @param ox: x component of the quaternion for the end effector rotation
        @type oy: double
        @param oy: y component of the quaternion for the end effector rotation
        @type oz: double
        @param oz: z component of the quaternion for the end effector rotation
        @type ow: double
        @param ow: w component of the quaternion for the end effector rotation
        @type fx: double
        @param fx: force or stiffness in the x direction
        @type fy: double
        @param fy: force or stiffness in the y direction
        @type fz: double
        @param fz: force or stiffness in the z direction
        @type tx: double
        @param tx: torque or stiffness around the x axis
        @type ty: double
        @param ty: torque or stiffness around the y axis
        @type tz: double
        @param tz: torque or stiffness around the z axis
        @type isfx: boolean
        @param isfx: True if this point should exert the given force in the x direction, false if it should achieve a position in the x direction with the given stiffness.
        @type isfy: boolean
        @param isfy: True if this point should exert the given force in the y direction, false if it should achieve a position in the y direction with the given stiffness.
        @type isfz: boolean
        @param isfz: True if this point should exert the given force in the z direction, false if it should achieve a position in the z direction with the given stiffness.
        @type istx: boolean
        @param istx: True if this point should exert the given torque around the x axis, false if it should achieve an orientation around the x axis with the given stiffness.
        @type isty: boolean
        @param isty: True if this point should exert the given torque around the y axis, false if it should achieve an orientation around the y axis with the given stiffness.
        @type istz: boolean
        @param istz: True if this point should exert the given torque around the z axis, false if it should achieve an orientation around the z axis with the given stiffness.
        @type time: boolean
        @param time: The time from start when this point should be achieved.  Note that this is NOT the time from the last point, but the time from when the entire trajectory is begun.
        @type frame_id: string
        @param frame_id: The frame id of this point.  If left as an empty string will be assumed that the point is in the frame of the root link of the chain for ee_cart_imped_control (likely torso_lift_link).
        '''
        new_point = len(self.goal.trajectory);
        self.goal.trajectory.append(ee_cart_imped_msgs.msg.StiffPoint())
        self.goal.trajectory[new_point].header.stamp = rospy.Time(0)
        self.goal.trajectory[new_point].header.frame_id = frame_id
        self.goal.trajectory[new_point].pose.position.x = x;
        self.goal.trajectory[new_point].pose.position.y = y;
        self.goal.trajectory[new_point].pose.position.z = z;
        self.goal.trajectory[new_point].pose.orientation.x = ox;
        self.goal.trajectory[new_point].pose.orientation.y = oy;
        self.goal.trajectory[new_point].pose.orientation.z = oz;
        self.goal.trajectory[new_point].pose.orientation.w = ow;
        self.goal.trajectory[new_point].wrench_or_stiffness.force.x = fx;
        self.goal.trajectory[new_point].wrench_or_stiffness.force.y = fy; 
        self.goal.trajectory[new_point].wrench_or_stiffness.force.z = fz;
        self.goal.trajectory[new_point].wrench_or_stiffness.torque.x = tx;
        self.goal.trajectory[new_point].wrench_or_stiffness.torque.y = ty;
        self.goal.trajectory[new_point].wrench_or_stiffness.torque.z = tz;
        
        self.goal.trajectory[new_point].isForceX = isfx
        self.goal.trajectory[new_point].isForceY = isfy
        self.goal.trajectory[new_point].isForceZ = isfz
        self.goal.trajectory[new_point].isTorqueX = istx
        self.goal.trajectory[new_point].isTorqueY = isty
        self.goal.trajectory[new_point].isTorqueZ = istz
        self.goal.trajectory[new_point].time_from_start = rospy.Duration(time);

    def trajectoryTime(self):
        '''
        @return: The total time of the stored trajectory.
        '''
        if len(self.goal.trajectory) == 0:
            return rospy.Duration(0)
        return self.goal.trajectory[len(self.goal.trajectory)-1].time_from_start

    def sendGoal(self, wait=True):
        '''
        Sends the stored trajectory to the action server.
        @type wait: boolean
        @param wait: If true, this function will wait for the goal to complete before returning.  If false, this function will return immediately upon sending the goal.
        @return: The state of the simple action client.
        '''
        self.goal.header.stamp = rospy.Time.now()
	rospy.logdebug('Sending goal %s to force controller on %s',
                      str(self.goal), self.arm_name)
        self.client.send_goal(self.goal)
        if wait:
            self.client.wait_for_result()
        return self.client.get_state()

    def resetGoal(self):
        '''
        Resets the stored trajectory to be an empty trajectory.
        '''
        self.goal = ee_cart_imped_msgs.msg.EECartImpedGoal()
        self.goal.header.frame_id = 'torso_lift_link'
        
    def cancelGoal(self):
        '''
        Cancels the current goal.  The arm will hold its current position.
        '''
        self.client.cancel_goal()

    def cancelAllGoals(self):
        '''
        Cancels all goals on the action server.  The arm will hold its current position.
        '''
        self.client.cancel_all_goals()
