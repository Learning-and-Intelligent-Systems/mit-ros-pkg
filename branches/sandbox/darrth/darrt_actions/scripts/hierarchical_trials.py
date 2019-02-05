#!/usr/bin/env/python
import roslib; roslib.load_manifest('darrt_actions')
from sim_setup import World, SimSetup
from darrt_msgs.msg import DARRTAction, DARRTGoal
from darrt_actions.darrt_trajectories import get_trajectory_markers, Executor

from pr2_python.planning_scene_interface import get_planning_scene_interface
from pr2_tasks.pickplace_definitions import PlaceGoal, PickupGoal

from arm_navigation_msgs.msg import CollisionObject, Shape
from actionlib import SimpleActionClient
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseWithCovarianceStamped

import pr2_python.geometry_tools as gt
import pr2_python.transform_listener as tl

import object_manipulation_msgs.msg as om
import numpy as np
import rospy
import copy
import optparse
import pickle
import numpy as np


NPLATEGRASPS=200
GRASP_DIST=0.3

def grasps(obj):
    if obj.id == 'plate':
        ps = PoseStamped()
        ps.header = obj.header
        ps.pose = obj.poses[0]
        return plate_grasps(ps, obj.header.frame_id)
    return box_grasps(obj, obj.header.frame_id)

def plate_grasps(plate_pose_stamped, grasp_reference_frame):
    plate_pose = tl.transform_pose(grasp_reference_frame, plate_pose_stamped.header.frame_id, plate_pose_stamped.pose)
    #this is the object in the frame of the gripper
    grasp_pose = Pose()
    grasps = []
    rospy.loginfo('plate pose = \n'+str(plate_pose))
    origin = Pose()
    origin.orientation.w = 1.0;
    grasp_pose.position.x = GRASP_DIST;
    for i in range(NPLATEGRASPS):
        alpha = 2.0*np.pi/NPLATEGRASPS*i
        grasp_pose.orientation = gt.multiply_quaternions(Quaternion(x=0.707, w=0.707),
                                                         Quaternion(z = np.sin(alpha/2.0), w = np.cos(alpha/2.0)))
        grasp = om.Grasp()

        grasp.grasp_pose = gt.transform_pose(gt.inverse_transform_pose(origin, grasp_pose), plate_pose)
        grasp.min_approach_distance = 0.0
        grasp.desired_approach_distance = 0.3
        grasps.append(grasp)

    return grasps

def box_grasps(box, grasp_reference_frame):
    box_pose = tl.transform_pose(grasp_reference_frame, box.header.frame_id, box.poses[0])
    #this is the object in the frame of the gripper
    grasp_pose = Pose()
    grasps = []
    #just have an overhead grasp - this isn't great but whatever
    #it's also probably not generally right for rotated boxes...
    grasp = om.Grasp()
    grasp.grasp_pose = copy.deepcopy(box_pose)
    grasp.grasp_pose.position.z += box.shapes[0].dimensions[2]/2.0+0.15
    grasp.grasp_pose.orientation.x = 0
    grasp.grasp_pose.orientation.y = 1.0/np.sqrt(2.0)
    grasp.grasp_pose.orientation.z = 0
    grasp.grasp_pose.orientation.w = 1.0/np.sqrt(2.0)
    grasp.grasp_pose.orientation = gt.multiply_quaternions(grasp.grasp_pose.orientation, box_pose.orientation)
    grasp.min_approach_distance = 0.0
    grasp.desired_approach_distance = 0.3
    grasps.append(grasp)
    return grasps


def _get_options_parser():
    parser = optparse.OptionParser()
    parser.add_option('-w', '--world', dest='world', default=-1, type='int', help='The world to use (0-6).')
    parser.add_option('-l', '--hierarchical', default=False, dest='hierarchical', action='store_true',
                      help='Run using DARRTH')
    parser.add_option('-n', '--ntrials', dest='ntrials', default=1, type='int', help='The number of trials to run')
    parser.add_option('-t', '--time', dest='planning_time', default=30, type='int', help='The number of seconds after which to restart')
    parser.add_option('-d', '--debug_level', dest='debug_level', default=1, type='int', help='Debug level')
    parser.add_option('-p', '--pause', default=False, dest='do_pause', action='store_true', help='Enable pauses during planning')
    parser.add_option('-i', '--interactive', default=False, dest='interactive', action='store_true', help='Enable interactive planning')
    parser.add_option('-b', '--bias', default=0.1, type='float', dest='goal_bias', help='Goal bias')
    parser.add_option('-r', '--darrt-tries', default=1, type='int', dest='darrt_tries', 
                      help='Number of times to repeat DARRT RRTs before trying again at a higher level')
    parser.add_option('-T', '--tries', default=1000, type='int', dest='tries', 
                      help='Number of retries before returning failure.  For timed trials should be huge')
    parser.add_option('-s', '--screenshot', default=False, dest='screenshot', action='store_true', help='Set up for screen shot and exit')
    parser.add_option('-e', '--execute', default=False, dest='execute', action='store_true', help='Execute last path.')
    parser.add_option('-v', '--visualize', default=False, dest='visualize', action='store_true', help='Visualize last path.')
    parser.add_option('-k', '--no-pickle', default=False, dest='no_pickle', action='store_true', help='Disable pickling')
    parser.add_option('-f', '--forward', default=False, dest='forward', action='store_true', help='Use forward rather than bi-directional planning.  Almost always slower.')
    parser.add_option('-o', '--output-file', default='', type='string', dest='output_file', help='Write to an output file')
    parser.add_option('--start-id', default=0, type='int', dest='start_id', help='starting id of markers')
    parser.add_option('-m', '--real-robot', default=False, dest='real', action='store_true', help='Using the real robot.  Do not set the map position as it has presumably already been localized.  Localize the robot if using this option!')
    return parser

class Logger:
    def __init__(self, filename):
        self.filename = filename
        self.file = None
        if len(filename):
            self.file = open(self.filename, 'w')
    
    def log(self, msg):
        rospy.loginfo(msg)
        if self.file:
            self.file.write('\n'+msg)

def main():

    parser = _get_options_parser()
    (options, args) = parser.parse_args()
    logger = Logger(options.output_file)

    ss = SimSetup(options.world, options.screenshot, fake_walls=True, real=options.real)
    goal = ss.setup('right_arm')
    
    if options.screenshot:
        return

    # if options.real:
    #     rospy.loginfo('Ready to plan?')
    #     raw_input()

    #DARRT action client
    client = SimpleActionClient('/darrt_planning/darrt_action', DARRTAction)
    rospy.loginfo('Waiting for DARRT action')
    client.wait_for_server()
    rospy.loginfo('Found DARRT action')

    #set the options
    goal.tries = options.tries
    goal.planning_time = options.planning_time
    goal.debug_level = options.debug_level
    goal.interactive = options.interactive
    goal.do_pause = options.do_pause
    goal.hierarchical = options.hierarchical
    goal.darrt_tries = options.darrt_tries
    goal.goal_bias = options.goal_bias
    goal.forward_planning = options.forward

    avg_planning_time = 0
    avg_object_time = 0
    avg_subgoal_time = []
    avg_total_time = 0
    if options.visualize:
        pub = rospy.Publisher('darrt_trajectory', MarkerArray)
    logger.log('WORLD: '+str(options.world)+ ', RESET TIME: '+ str(options.planning_time) + 
               ', DARRT TRIES: ' + str(options.darrt_tries)+', FORWARD: '+str(options.forward)) 
    for i in range(options.ntrials):
        ss.set_robot_position() #sometimes the simulator goes crazy and bumps the robot
        ss.psi.reset()
        if (options.visualize and (i < 2)):
            goal.visualize = True
        else:
            goal.visualize = False
        client.send_goal_and_wait(goal)
        result = client.get_result()
        if result.error_code != result.SUCCESS:
            logger.log('Unsuccessful planning attempt!')
            break
        logger.log('TRIAL '+str(i+1)+':')
        logger.log('\tPlanning time: '+str(result.planning_time))
        avg_planning_time += result.planning_time
        total_time = 0
        if options.hierarchical:
            logger.log('\tObject planning time: ' + str(result.object_time))
            logger.log('\tSubgoal times:')
            avg_object_time += result.object_time
            for (j, t) in enumerate(result.subgoal_time):
                logger.log('\t\t'+str(j)+': '+str(t))
                if len(avg_subgoal_time) <= j:
                    avg_subgoal_time.append(t)
                else:
                    avg_subgoal_time[j] += t
                total_time += t
            logger.log('\t\tTOTAL: ' + str(total_time))
            total_time += result.object_time
            logger.log('\tTotal time: '+str(total_time))
            avg_total_time += total_time
        # if options.visualize and (i % 10 == 0):
        #     traj = get_trajectory_markers(result.primitive_trajectories, result.primitive_names,
        #                                   object_trajectories=result.object_trajectories,
        #                                   start_id=options.start_id)
        #     rospy.loginfo('Ready to see simulation?')
        #     raw_input()
        #     for (l,m) in enumerate(traj):
        #         rospy.loginfo('Publishing point '+str(l)+' of '+str(len(traj)))
        #         for (j, p) in enumerate(m.markers):
        #             p.id = options.start_id + j
        #         for k in range(10):
        #             pub.publish(m)
        #             rospy.sleep(0.0003)

    logger.log('TOTALS: TRIALS: '+str(i+1)+', AVERAGE TIME: '+str(avg_planning_time/float(i+1))+', AVERAGE TOTAL TIME: '+str(avg_total_time/float(i+1))+
               ' AVERAGE OBJECT TIME: '+str(avg_object_time/float(i+1))+', AVERAGE SUBGOAL TIMES:')
    subgoal_total = 0
    for (j, t) in enumerate(avg_subgoal_time):
        logger.log('\t'+str(j)+': '+ str(t/float(i+1)))
        subgoal_total += t/float(i+1)
    logger.log('\tTOTAL: '+str(subgoal_total))

    #pickle everything
    if not options.no_pickle:
        time = rospy.get_time()
        filename = 'hierarchical_results/plateh_'+str(time)+'.pck'
        logger.log('Saving to file '+str(filename))
        pickle.dump([goal, result, options], open(filename, 'wb'))
    if result.error_code != result.SUCCESS:
        return
    #draw
    # if options.visualize:
    #     traj = get_trajectory_markers(result.primitive_trajectories, result.primitive_names,
    #                                     object_trajectories=result.object_trajectories,
    #                                     start_id=options.start_id)
    #     rospy.loginfo('Ready to see simulation?')
    #     raw_input()
    #     for (i,m) in enumerate(traj):
    #         rospy.loginfo('Publishing point '+str(i)+' of '+str(len(traj)))
    #         for (j, p) in enumerate(m.markers):
    #             p.id = options.start_id + j
    #         for k in range(10):
    #             pub.publish(m)
    #             rospy.sleep(0.001)
            #raw_input()
            
        #publish the whole trajectory
        # mid = 0
        # for t in traj:
        #     for m in t.markers:
        #         m.id = mid
        #         mid += 1
        #     for i in range(10):
        #         pub.publish(t)
        #         rospy.sleep(0.05)
        
    if options.execute:
        executor = Executor()
        rospy.loginfo('Press enter to execute')
        raw_input()
        executor.execute_trajectories(result)
        rospy.loginfo('Successfully executed!')

rospy.init_node('hierarchical_trials_test_node')
main()
