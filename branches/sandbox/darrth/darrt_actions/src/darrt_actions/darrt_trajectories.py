import roslib; roslib.load_manifest('darrt_actions')
from visualization_msgs.msg import MarkerArray
import pr2_python.visualization_tools as vt
import pr2_python.geometry_tools as gt
import pr2_python.trajectory_tools as tt
from pr2_python.gripper import Gripper
from pr2_python.arm_mover import ArmMover
from pr2_python.arm_planner import ArmPlanner
from pr2_python.base import Base
from pr2_tasks.arm_tasks import ArmTasks
from pr2_python.world_interface import WorldInterface
from pr2_python.exceptions import ActionFailedError, ArmNavError
from pr2_python.planning_scene_interface import get_planning_scene_interface
from pr2_python.hand_description import HandDescription
from arm_navigation_msgs.msg import OrderedCollisionOperations, CollisionOperation, RobotState
from base_trajectory_action.msg import BaseTrajectoryAction, BaseTrajectoryGoal
from geometry_msgs.msg import Pose2D, PoseStamped
from actionlib import SimpleActionClient
from darrt_msgs.msg import DARRTResult
from std_msgs.msg import ColorRGBA
import numpy as np
import rospy
import copy


def type_to_color(n):
    types_to_colors = {'ArmTransit': ColorRGBA(r=1,g=0,b=0,a=0.8),
                       'Pickup': ColorRGBA(r=0,g=1,b=0,a=0.8),
                       'Place': ColorRGBA(r=0,g=1,b=0,a=0.8),
                       'Push': ColorRGBA(r=0,g=1,b=1,a=0.8),
                       'Approach': ColorRGBA(r=0,g=0,b=0,a=0.8),
                       'Retreat': ColorRGBA(r=0,g=0,b=0,a=0.8),
                       'BaseTransit': ColorRGBA(r=1,g=0,b=1,a=0.8),
                       'BaseWarp': ColorRGBA(r=1,g=0,b=1,a=0.8),
                       'UseSpatula': ColorRGBA(r=0,g=0,b=1,a=0.8),
                       'SpatulaTransfer': ColorRGBA(r=1,g=1,b=0,a=0.8),
                       'RigidTransfer': ColorRGBA(r=1,g=0.5,b=0,a=0.8),
                       'start': ColorRGBA(r=0,g=1,b=1)}
    try:
        return types_to_colors[n]
    except KeyError:
        rospy.logwarn('No color for primitive '+n)
        return ColorRGBA(r=0,g=0,b=0,a=0.8)
        

def get_trajectory_markers(trajectories, names, object_trajectories=None, link_names=None, start_id=0):

    state = RobotState()
    state.joint_state = tt.joint_trajectory_point_to_joint_state(trajectories[0].joint_trajectory.points[0],
                                                                          trajectories[0].joint_trajectory.joint_names)
    mdf = trajectories[0].multi_dof_joint_trajectory
    state.multi_dof_joint_state = tt.multi_dof_trajectory_point_to_multi_dof_state(mdf.points[0], mdf.joint_names, mdf.frame_ids,
                                                                                            mdf.child_frame_ids, mdf.stamp)
    n = names[0].split('-')
    c = type_to_color(n[0])
    marray = vt.robot_marker(state, ns='traj_start_state'+str(start_id), r = c.r, g= c.g, b=c.b, a=0.5)
    for (i, m) in enumerate(marray.markers):
        m.id = start_id + i
    traj = [marray]

    mid = start_id
    for (i, t) in enumerate(trajectories):
        splits = names[i].split('-')
        c = type_to_color(splits[0])
        rospy.loginfo('Drawing trajectory ' +str(i)+' of type '+names[i]+' and length '+
                      str(len(t.joint_trajectory.points)))
        #print 'Trajectory is\n', t
        for j in range(len(t.joint_trajectory.points)):
            state.joint_state = tt.joint_trajectory_point_to_joint_state(t.joint_trajectory.points[j],
                                                                         t.joint_trajectory.joint_names)
            state.multi_dof_joint_state = tt.multi_dof_trajectory_point_to_multi_dof_state(
                t.multi_dof_joint_trajectory.points[j], t.multi_dof_joint_trajectory.joint_names, 
                t.multi_dof_joint_trajectory.frame_ids, t.multi_dof_joint_trajectory.child_frame_ids, 
                t.multi_dof_joint_trajectory.stamp)

            marray = vt.robot_marker(state, link_names=link_names, ns='trajectory',  r = c.r, g=c.g, b=c.b, a=0.5)
        #marray = vt.trajectory_markers(t, ns='trajectory', link_names=link_names,resolution=3,
         #                              color=c)
            for (i, m) in enumerate(marray.markers):
                m.id = mid
                mid += 1
            traj.append(copy.deepcopy(marray))
    return traj

    
    #     if object_trajectories:
    #         for o in object_trajectories[i].trajectory:
    #             ps = PoseStamped()
    #             ps.header.frame_id = object_trajectories[i].header.frame_id
    #             ps.pose = o
    #             marray.markers += [vt.marker_at(ps, ns='object_trajectory'+str(start_id), r=c.r, g=c.g, b=c.b, sx=0.1, sy=0.1, sz=0.1)]

    # return traj

class Executor:
    def __init__(self):
        arms = ['right_arm', 'left_arm']
        self.grippers = {}
        self.arm_planners = {}
        for arm in arms:
            self.grippers[arm] = Gripper(arm)
            self.arm_planners[arm] = ArmPlanner(arm)
        self.arm_mover = ArmMover()
        self.arm_tasks = ArmTasks()
        self.base = Base()
        self.wi = WorldInterface()
        self.psi = get_planning_scene_interface()
        self.base_action = SimpleActionClient('base_trajectory_action', BaseTrajectoryAction)
        rospy.loginfo('Waiting for base trajectory action')
        self.base_action.wait_for_server()
        rospy.loginfo('Found base trajectory action')
        rospy.loginfo('DARRT trajectory executor successfully initialized!')

    def execute_trajectories(self, result):
        rospy.loginfo('There are '+str(len(result.primitive_names))+' segments')
        for (i,t) in enumerate(result.primitive_types):
            rospy.loginfo('Executing trajectory ' +str(i)+' of type '+t+' and length '+
                          str(len(result.primitive_trajectories[i].joint_trajectory.points)))
            #print 'Trajectory is\n', result.primitive_trajectories[i]
            splits = t.split('-')
            if splits[0] == 'BaseTransit':
                self.execute_base_trajectory(result.primitive_trajectories[i])
                continue
            if splits[0] == 'BaseWarp':
                self.arm_tasks.move_arm_to_side('right_arm')
                self.arm_tasks.move_arm_to_side('left_arm')
                self.execute_warp_trajectory(result.primitive_trajectories[i])
                continue
            if splits[0] == 'ArmTransit':
                self.execute_arm_trajectory(splits[1], result.primitive_trajectories[i])
                continue
            #if splits[0] == 'Approach':
                #gripper.open()
            if splits[0] == 'Pickup':
                try:
                    self.grippers[splits[1]].close()
                except ActionFailedError:
                    pass
                self.wi.attach_object_to_gripper(splits[1], splits[2])
                #so the trajectory filter doesn't complain
                ops = OrderedCollisionOperations()
                op = CollisionOperation()
                op.operation = op.DISABLE
                
                op.object2 = splits[2]
                for t in self.wi.hands[splits[1]].touch_links:
                    op.object1 = t
                    ops.collision_operations.append(copy.deepcopy(op))
                self.psi.add_ordered_collisions(ops)
            self.execute_straight_line_arm_trajectory(splits[1], result.primitive_trajectories[i])
            if splits[0] == 'Place':
                self.grippers[splits[1]].open()
                self.wi.detach_and_add_back_attached_object(splits[1], splits[2])
                self.psi.reset()

    def execute_warp_trajectory(self, trajectory):
        #figure out the last point on the trajectory
        tp = trajectory.multi_dof_joint_trajectory.points[-1]
        (phi, theta, psi) = gt.quaternion_to_euler(tp.poses[0].orientation)
        self.base.move_to(tp.poses[0].position.x, tp.poses[0].position.y, psi)

    def execute_base_trajectory(self, trajectory):
        goal = BaseTrajectoryGoal()
        #be a little slower and more precise
        goal.linear_velocity = 0.2
        goal.angular_velocity = np.pi/8.0
        goal.linear_error = 0.01
        goal.angular_error = 0.01
        joint = -1
        for (i, n) in enumerate(trajectory.multi_dof_joint_trajectory.joint_names):
            if n == 'world_joint' or n == '/world_joint':
                goal.world_frame = trajectory.multi_dof_joint_trajectory.frame_ids[i]
                goal.robot_frame = trajectory.multi_dof_joint_trajectory.child_frame_ids[i]
                joint = i
                break
        if joint < 0:
            raise ActionFailedError('No world joint found in base trajectory')
        for p in trajectory.multi_dof_joint_trajectory.points:
            (phi, theta, psi) = gt.quaternion_to_euler(p.poses[joint].orientation)
            goal.trajectory.append(Pose2D(x=p.poses[joint].position.x, y=p.poses[joint].position.y,
                                          theta=psi))
        self.base_action.send_goal_and_wait(goal)

    def execute_arm_trajectory(self, arm_name, trajectory):
        arm_trajectory = self.arm_planners[arm_name].joint_trajectory(trajectory.joint_trajectory)
        arm_trajectory.header.stamp = rospy.Time.now()
        #try:
        #    arm_trajectory = self.arm_planners[arm_name].filter_trajectory(arm_trajectory)
        #except ArmNavError, e:
        #rospy.logwarn('Trajectory filter failed with error '+str(e)+'.  Executing anyway')
        arm_trajectory = tt.convert_path_to_trajectory(arm_trajectory, time_per_pt=0.35)
        self.arm_mover.execute_joint_trajectory(arm_name, arm_trajectory)

    def execute_straight_line_arm_trajectory(self, arm_name, trajectory):
        arm_trajectory = self.arm_planners[arm_name].joint_trajectory(trajectory.joint_trajectory)
        arm_trajectory.header.stamp = rospy.Time.now()
        arm_trajectory = tt.convert_path_to_trajectory(arm_trajectory, time_per_pt=0.4)
        self.arm_mover.execute_joint_trajectory(arm_name, arm_trajectory)
        
