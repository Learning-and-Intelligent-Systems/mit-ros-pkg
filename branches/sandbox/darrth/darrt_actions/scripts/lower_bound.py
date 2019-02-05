import roslib; roslib.load_manifest('darrt_actions')
from sim_setup import SimSetup
from pr2_tasks.pickplace_definitions import PickupGoal
from object_manipulation_msgs.msg import GraspableObject
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from arm_navigation_msgs.msg import OrderedCollisionOperations, CollisionOperation

from pr2_python.arm_planner import ArmPlanner
from pr2_python.arm_mover import ArmMover
from nav_msgs.srv import GetPlan
from pr2_python.planning_scene_interface import get_planning_scene_interface
import pr2_python.visualization_tools as vt
from visualization_msgs.msg import MarkerArray
import pr2_python.trajectory_tools as tt

import optparse
import rospy
import numpy as np
import copy
import time

GRIPPER_LENGTH = 0.18
PUSH_DIST = 0.35

def _get_options_parser():
    parser = optparse.OptionParser()
    parser.add_option('-w', '--world', dest='world', default=0, type='int', help='The world to use (0-6).')
    parser.add_option('-n', '--ntrials', dest='ntrials', default=1, type='int', help='The number of trials to run')
    parser.add_option('-p', '--pause', dest='pause', default=False, action='store_true', help='Pause between stages')
    return parser

class BasePoses:
    def __init__(self, push, pick, goal):
        self.push = push
        self.pick = pick
        self.goal = goal

class ArmPoses:
    def __init__(self, approach_push, init_push, final_push, retreat_push, approach_pick, pick, lift, goal):
        self.approach_push = approach_push
        self.init_push = init_push
        self.final_push = final_push
        self.retreat_push = retreat_push
        self.approach_pick = approach_pick
        self.pick = pick
        self.lift = lift
        self.goal = goal

class Planner:
    def __init__(self):
        self.arm_planner = ArmPlanner('right_arm')
        self.arm_mover = ArmMover()
        self.base_plan_service = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
        self.psi = get_planning_scene_interface()
        self.viz_pub = rospy.Publisher('lower_bound', MarkerArray)
        self.pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped)
        #self.base = Base()

    def publish(self, markers):
        for i in range(50):
            self.viz_pub.publish(markers)
            rospy.sleep(0.01)

    def plan_base(self, goal):
        #find the current base pose from the planning scene
        robot_state = self.psi.get_robot_state()
        starting_state = PoseStamped()
        starting_state.header.frame_id = robot_state.multi_dof_joint_state.frame_ids[0]
        starting_state.pose = robot_state.multi_dof_joint_state.poses[0]
        
        goal_robot_state = copy.deepcopy(robot_state)
        goal_robot_state.multi_dof_joint_state.poses[0] = goal.pose

        start_markers = vt.robot_marker(robot_state, ns='base_starting_state')
        goal_markers = vt.robot_marker(goal_robot_state, ns='base_goal_state', g=1, b=0)
        self.publish(start_markers)
        self.publish(goal_markers)

        #plan
        good_plan = False
        while not good_plan:
            try:
                start_time = time.time()
                self.base_plan_service(start=starting_state, goal=goal)
                end_time = time.time()
                good_plan = True
            except:
                rospy.loginfo('Having trouble probably with starting state.  Sleeping and trying again')
                rospy.sleep(0.2)

        
        #it's apparently better just to do the trajectory
        #than fool with the planning scene
        goal_cov = PoseWithCovarianceStamped()
        goal_cov.header = goal.header
        goal_cov.pose.pose = goal.pose
        self.pose_pub.publish(goal_cov)
        self.psi.reset()

        # self.base.move_to(goal.pose.position.x, goal.pose.position.y, 
        #                   2.0*np.arcsin(goal.pose.orientation.z))

        return (end_time - start_time)
 
    def plan_arm(self, goal, ordered_colls=None, interpolated=False):
        time = 0
        robot_state = self.psi.get_robot_state()
        ending_robot_state = copy.deepcopy(robot_state)

        start_markers = vt.robot_marker(robot_state, ns='arm_starting_state')
        self.publish(start_markers)     
        
        marray = MarkerArray()
        marker = vt.marker_at(goal, ns="arm_goal_pose")
        marray.markers.append(marker)
        self.publish(marray)
   
        if not interpolated:
            traj, time = self.arm_planner.plan_collision_free(goal, ordered_collisions=ordered_colls, runtime=True)
        else:
            goal_bl = self.psi.transform_pose_stamped('base_link', goal)
            marray = MarkerArray()
            marker = vt.marker_at(goal_bl, ns="arm_goal_pose_base_link")
            marray.markers.append(marker)
            self.publish(marray)

            traj, time = self.arm_planner.plan_interpolated_ik(goal_bl, ordered_collisions=ordered_colls, 
                                                               consistent_angle=2.0*np.pi, runtime=True)

        joint_state = tt.joint_trajectory_point_to_joint_state(traj.points[-1], traj.joint_names)
        ending_robot_state = tt.set_joint_state_in_robot_state(joint_state, ending_robot_state)

        goal_markers = vt.robot_marker(ending_robot_state, ns='arm_goal_state', g=1, b=0)
        self.publish(goal_markers)

        #do the trajectory
        # traj.points = [traj.points[0], traj.points[1], traj.points[-1]]
        # traj.header.stamp = rospy.Time.now()
        # traj.points[-1].time_from_start = 3
        #rospy.loginfo('Trajectory =\n'+str(traj))
        self.arm_mover.execute_joint_trajectory('right_arm', traj)
        #self.psi.set_robot_state(ending_robot_state)
        
        return time

    def plan_arm_interpolated(self, goal, ordered_colls=None):
        return self.plan_arm(goal, ordered_colls=ordered_colls, interpolated=True)


def get_base_poses(wi, world):
    if world != 6 and world != 10:
        if world == 0:
            push_base_pose = PoseStamped()
            push_base_pose.header.frame_id = wi.world_frame
            push_base_pose.pose.position.x = 2.7
            push_base_pose.pose.position.y = -2.5
            push_base_pose.pose.orientation.w = 1
        else:
            push_base_pose = PoseStamped()
            push_base_pose.header.frame_id = wi.world_frame
            push_base_pose.pose.position.x = 2.75
            push_base_pose.pose.position.y = -2.66
            push_base_pose.pose.orientation.z = -0.329
            push_base_pose.pose.orientation.w = 0.944
            
        pickup_base_pose = PoseStamped()
        pickup_base_pose.header.frame_id = wi.world_frame
        pickup_base_pose.pose.position.x = 2.6
        pickup_base_pose.pose.position.y = -2.4
        pickup_base_pose.pose.orientation.w = 1

        if world == 12:
            pickup_base_pose = PoseStamped()
            pickup_base_pose.header.frame_id = wi.world_frame
            pickup_base_pose.pose.position.x = 2.3
            pickup_base_pose.pose.position.y = -2.4
            pickup_base_pose.pose.orientation.w = 1

    elif world == 6:
        push_base_pose = PoseStamped()
        push_base_pose.header.frame_id = wi.world_frame
        push_base_pose.pose.position.x = 0.6
        push_base_pose.pose.position.y = -0.6
        push_base_pose.pose.orientation.z = np.sqrt(0.5)
        push_base_pose.pose.orientation.w = np.sqrt(0.5)

        pickup_base_pose = PoseStamped()
        pickup_base_pose.header.frame_id = wi.world_frame
        pickup_base_pose.pose.position.x = 0.6
        pickup_base_pose.pose.position.y = -0.8
        push_base_pose.pose.orientation.z = np.sqrt(0.5)
        push_base_pose.pose.orientation.w = np.sqrt(0.5)
    else:
        push_base_pose = PoseStamped()
        push_base_pose.header.frame_id = wi.world_frame
        push_base_pose.pose.position.x = 0.6
        push_base_pose.pose.position.y = -0.6
        push_base_pose.pose.orientation.z = np.sqrt(0.5)
        push_base_pose.pose.orientation.w = np.sqrt(0.5)

        pickup_base_pose = PoseStamped()
        pickup_base_pose.header.frame_id = wi.world_frame
        pickup_base_pose.pose.position.x = 0.5
        pickup_base_pose.pose.position.y = 1
        push_base_pose.pose.orientation.z = -np.sqrt(0.5)
        push_base_pose.pose.orientation.w = np.sqrt(0.5)

        
    if not  world % 2 and world != 12:
        goal_base_pose = PoseStamped()
        goal_base_pose.header.frame_id = wi.world_frame
        goal_base_pose.pose.position.x = 1.5
        goal_base_pose.pose.position.y = -1.5
        goal_base_pose.pose.orientation.z = -1.0/np.sqrt(2.0)
        goal_base_pose.pose.orientation.w = 1.0/np.sqrt(2.0)
        goal_base_poses = [goal_base_pose]
    elif world == 12:
        gbs = PoseStamped()
        gbs.header.frame_id = wi.world_frame
        gbs.pose.position.x = 0.5
        gbs.pose.position.y = 1.5
        gbs.pose.orientation.w = 1

        goal_base_poses = [copy.deepcopy(gbs)]
        
        gbs.pose.position.x = -1.5
        gbs.pose.position.y = -0.5
        goal_base_poses.append(gbs)

        goal_base_pose = PoseStamped()
        goal_base_pose.header.frame_id = wi.world_frame
        goal_base_pose.pose.position.x = 0.5
        goal_base_pose.pose.position.y = -1.3
        goal_base_pose.pose.orientation.z = 1

        goal_base_poses.append(goal_base_pose)

    else:
        goal_base_pose = PoseStamped()
        goal_base_pose.header.frame_id = wi.world_frame
        goal_base_pose.pose.position.x = -1.5
        goal_base_pose.pose.position.y = 1
        goal_base_pose.pose.orientation.w = 1
        goal_base_poses = [goal_base_pose]


    return BasePoses(push_base_pose, pickup_base_pose, goal_base_poses)

def get_arm_poses(wi, world):

    plate = wi.collision_object('plate')

    approach_arm_pose = PoseStamped()
    approach_arm_pose.header = copy.deepcopy(plate.header)
    approach_arm_pose.pose = copy.deepcopy(plate.poses[0])
    approach_arm_pose.pose.position.z += 0.1 + GRIPPER_LENGTH
    approach_arm_pose.pose.position.x += plate.shapes[0].dimensions[0]+0.01
    approach_arm_pose.pose.orientation.y = 1.0/np.sqrt(2.0)
    approach_arm_pose.pose.orientation.w = 1.0/np.sqrt(2.0)
    
    
    push_arm_pose = copy.deepcopy(approach_arm_pose)
    push_arm_pose.pose.position.z -= 0.1

    final_push_arm_pose = copy.deepcopy(push_arm_pose)
    final_push_arm_pose.pose.position.x -= PUSH_DIST

    retreat_push = copy.deepcopy(final_push_arm_pose)
    retreat_push.pose.position.z += 0.1
    
    approach_pick = PoseStamped()
    approach_pick.header = copy.deepcopy(plate.header)
    approach_pick.pose = copy.deepcopy(plate.poses[0])
    approach_pick.pose.position.x -= PUSH_DIST + (plate.shapes[0].dimensions[0]-0.03) + 0.1 + GRIPPER_LENGTH
    approach_pick.pose.orientation.x = 1.0/np.sqrt(2.0)
    approach_pick.pose.orientation.y = 0
    approach_pick.pose.orientation.z = 0
    approach_pick.pose.orientation.w = 1.0/np.sqrt(2.0)

    pick = copy.deepcopy(approach_pick)
    pick.pose.position.x += 0.1

    lift = copy.deepcopy(pick)
    lift.pose.position.z += 0.1

    if not world % 2 and world != 12:
        goal = PoseStamped()
        goal.header = copy.deepcopy(plate.header)
        goal.pose.position.x = 1 - GRIPPER_LENGTH
        goal.pose.position.y = -2
        goal.pose.position.z = 0.85
        goal.pose.orientation.x = 1.0/np.sqrt(2.0)
        goal.pose.orientation.w = 1.0/np.sqrt(2.0)
    elif world == 12:
        goal = PoseStamped()
        goal.header = copy.deepcopy(plate.header)
        goal.pose.position.x = 0
        goal.pose.position.y = -1 + GRIPPER_LENGTH
        goal.pose.position.z = 0.85
        goal.pose.orientation.x = 0.5
        goal.pose.orientation.y = -0.5
        goal.pose.orientation.z = -0.5
        goal.pose.orientation.w = 0.5
    else:
        goal = PoseStamped()
        goal.header = copy.deepcopy(plate.header)
        goal.pose.position.x = -1
        goal.pose.position.y = 1 + GRIPPER_LENGTH
        goal.pose.position.z = 0.85
        goal.pose.orientation.x = 0.5
        goal.pose.orientation.y = 0.5
        goal.pose.orientation.z = 0.5 
        goal.pose.orientation.w = 0.5


    return ArmPoses(approach_arm_pose, push_arm_pose, final_push_arm_pose,
                    retreat_push, approach_pick, pick, lift, goal)

def main():
    print 'Starting!'
    parser = _get_options_parser()
    (options, args) = parser.parse_args()
    print 'pause =', options.pause
    ss = SimSetup(options.world, False)
    world = ss.setup()
    planner = Planner()

    base_poses = get_base_poses(ss.wi, ss.world)
    arm_poses = get_arm_poses(ss.wi, ss.world)
    time = 0


    #note: these should all update the planning scene as we go
    #first plan to initial base position
    stime = planner.plan_base(base_poses.push)
    rospy.loginfo("Initial base move: "+str(stime))
    time += stime
    print 'pause =', options.pause
    if options.pause:
        raw_input()

    #now plan to approach to push
    stime = planner.plan_arm(arm_poses.approach_push)
    rospy.loginfo('Gross move to push: '+str(stime))
    time += stime
    if options.pause:
        raw_input()


    ops = OrderedCollisionOperations()
    op = CollisionOperation()
    op.operation = op.DISABLE
    op.object1 = world.object.id
    op.object2 = world.object_support_surface
    ops.collision_operations.append(op)
    op = copy.deepcopy(op)
    op.object2 = 'r_end_effector'
    ops.collision_operations.append(op)
    op = copy.deepcopy(op)
    op.object1 = world.object_support_surface
    ops.collision_operations.append(op)

    stime = planner.plan_arm_interpolated(arm_poses.init_push, ordered_colls=ops)
    rospy.loginfo('Approach to push: '+str(stime))
    time += stime
    if options.pause:
        raw_input()

    stime = planner.plan_arm_interpolated(arm_poses.final_push, ordered_colls=ops)
    rospy.loginfo('Push: '+str(stime))
    time += stime
    co = copy.deepcopy(world.object)
    co.poses[0].position.x -= PUSH_DIST
    ss.wi.add_object(co)
    planner.psi.reset()    
    if options.pause:
        raw_input()
    
    stime = planner.plan_arm_interpolated(arm_poses.retreat_push, ordered_colls=ops)
    rospy.loginfo('Retreat: '+str(stime))
    time += stime
    if options.pause:
        raw_input()

    stime = planner.plan_base(base_poses.pick)
    rospy.loginfo('Base move to pick: '+str(stime))
    time += stime
    if options.pause:
        raw_input()
 
    stime = planner.plan_arm(arm_poses.approach_pick)
    rospy.loginfo('Gross move to Pick: '+str(stime))
    time += stime
    if options.pause:
        raw_input()

    stime = planner.plan_arm_interpolated(arm_poses.pick, ordered_colls=ops)
    rospy.loginfo('Approach to pick: '+str(stime))
    time += stime
    if options.pause:
        raw_input()

    stime = planner.plan_arm_interpolated(arm_poses.lift, ordered_colls=ops)
    rospy.loginfo('Lift: '+str(stime))
    time  += stime
    if options.pause:
        raw_input()

        
    for g in base_poses.goal:
        stime = planner.plan_base(g)
        rospy.loginfo('Base move to goal: '+str(stime))
        time += stime
        if options.pause:
            raw_input()


    stime = planner.plan_arm(arm_poses.goal)
    rospy.loginfo('Arm move to goal: '+str(stime))
    time += stime
    if options.pause:
        raw_input()

    rospy.loginfo('TIME = '+str(time))
    

rospy.init_node('lower_bound_darrt_node')
main()
