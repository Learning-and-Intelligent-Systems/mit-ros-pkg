#!/usr/bin/env python
from numpy import *
import time, threading
from itertools import izip
import roslib; roslib.load_manifest('openrave_pr2')
import rospy

import actionlib
from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

def angleDistance(angle1,angle2):
    """expects both angle1 and angle2 to be within [-pi,pi] 
    """
    d=angle1-angle2
    if d>pi:
        return d-2*pi
    elif d<-pi:
        return d+2*pi
    else:
        return d

if __name__ == '__main__':
    """ init ROS
    """
    rospy.init_node('joint_limit_problem')

    
    print 'moving the right arm out of the way'
    whicharm = 'r'
    arm_joint_trajectory_action_name = ''+whicharm+'_arm_controller/joint_trajectory_action'
    arm_joint_action_client = actionlib.SimpleActionClient(arm_joint_trajectory_action_name, JointTrajectoryAction)
    arm_joint_action_client.wait_for_server()
    
    arm_goal = JointTrajectoryGoal()
    arm_goal.trajectory.joint_names=['r_shoulder_pan_joint','r_shoulder_lift_joint','r_upper_arm_roll_joint',
                      'r_elbow_flex_joint','r_forearm_roll_joint','r_wrist_flex_joint','r_wrist_roll_joint']
    
    arm_pt=JointTrajectoryPoint()
    arm_pt.positions=[-pi/2,0,0,0,0,0,0]
    arm_pt.time_from_start = rospy.Duration(5.0)
    arm_goal.trajectory.points.append(arm_pt)
    
    arm_joint_action_client.send_goal_and_wait(arm_goal)
    print 'arm action state:',actionlib.get_name_of_constant(actionlib.GoalStatus,arm_joint_action_client.get_state())
    
    
    print 'moving the left arm to problematic configuration'
    whicharm = 'l'
    arm_joint_trajectory_action_name = ''+whicharm+'_arm_controller/joint_trajectory_action'
    arm_joint_action_client = actionlib.SimpleActionClient(arm_joint_trajectory_action_name, JointTrajectoryAction)
    print 'waiting for',arm_joint_trajectory_action_name
    print 'have you set ROS_IP?'
    arm_joint_action_client.wait_for_server()
    torso_joint_trajectory_action_name = 'torso_controller/joint_trajectory_action'
    torso_joint_action_client = actionlib.SimpleActionClient(torso_joint_trajectory_action_name, JointTrajectoryAction)
    print 'waiting for',torso_joint_trajectory_action_name
    torso_joint_action_client.wait_for_server()
    print 'done init ROS'


    
    armDofIndices=[12, 15, 16, 17, 18, 19, 20, 21]
    jointNameByIndex={12:'torso_lift_joint',15:'l_shoulder_pan_joint',16:'l_shoulder_lift_joint',17:'l_upper_arm_roll_joint',
                      18:'l_elbow_flex_joint',19:'l_forearm_roll_joint',20:'l_wrist_flex_joint',21:'l_wrist_roll_joint'}
    def executeTraj(trajdata):
        #torsogoal = SingleJointPositionGoal()
        torso_goal = JointTrajectoryGoal()
        arm_goal = JointTrajectoryGoal()
        # parse trajectory data into the ROS structure
        tokens = trajdata.split()
        numpoints = int(tokens[0])
        dof = int(tokens[1])
        trajoptions = int(tokens[2])
        numvalues = dof
        offset = 0
        if trajoptions & 4: # timestamps
            numvalues += 1
            offset += 1
        if trajoptions & 8: # base transforms
            numvalues += 7
        if trajoptions & 16: # velocities
            numvalues += dof
        if trajoptions & 32: # torques
            numvalues += dof

        for i in range(numpoints):
            start = 3+numvalues*i
            torso_pt=JointTrajectoryPoint()
            arm_pt=JointTrajectoryPoint()
            k=0
            for dofInd in armDofIndices:
                pos=float(tokens[start+offset+dofInd])
                if jointNameByIndex[dofInd]=='torso_lift_joint':
                    if i>0:
                        prevpos=torso_goal.trajectory.points[-1].positions[0]
                        newpos=prevpos+angleDistance(pos,prevpos)
                        torso_pt.positions.append(newpos)
                    else:
                        torso_pt.positions.append(pos)
                else:
                    if i>0:
                        prevpos=arm_goal.trajectory.points[-1].positions[k]
                        newpos=prevpos+angleDistance(pos,prevpos)
                        arm_pt.positions.append(newpos)
                    else:
                        arm_pt.positions.append(pos)
                    k=k+1
            if trajoptions & 4:
                arm_pt.time_from_start = rospy.Duration(float(tokens[start]))
                torso_pt.time_from_start = rospy.Duration(float(tokens[start]))

            arm_goal.trajectory.joint_names =  \
                filter(lambda jn: jn!='torso_lift_joint',
                       [jointNameByIndex[i] for i in armDofIndices])
            arm_goal.trajectory.points.append(arm_pt)

            torso_goal.trajectory.joint_names = ['torso_lift_joint']
            torso_goal.trajectory.points.append(torso_pt)

        print 'sending arm JointTrajectoryGoal'
        arm_joint_action_client.send_goal(arm_goal)

        print 'sending torso JointTrajectoryGoal'
        torso_joint_action_client.send_goal(torso_goal)
        
        arm_joint_action_client.wait_for_result()
        torso_joint_action_client.wait_for_result()
        
        arm_state=arm_joint_action_client.get_state()
        torso_state=torso_joint_action_client.get_state()
        
        return arm_state,torso_state

    trajfile=open('shoulder.traj')
    lines=trajfile.readlines()
    trajfile.close()
    lines=lines[5:]
    trajdata=""
    for line in lines:
        trajdata=trajdata+line
    arm_state,torso_state=executeTraj(trajdata)
    print 'arm action state:',actionlib.get_name_of_constant(actionlib.GoalStatus,arm_state)
    print 'torso action state:',actionlib.get_name_of_constant(actionlib.GoalStatus,torso_state)