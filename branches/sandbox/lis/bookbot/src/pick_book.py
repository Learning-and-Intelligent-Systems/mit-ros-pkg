#!/usr/bin/python
import roslib
roslib.load_manifest('pr2_pick_and_place_tutorial_python')
import rospy

# import messages and services
from object_manipulation_msgs.msg import GraspableObject
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from bookbot.msg import PullStatus
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal

# import libraries
import actionlib
import tf
import scipy
import math
from math import *
import random
import time
import sys
import threading

# import arm controller
import lis_arm_controller.arm_manager as arm_manager
import lis_reactive_grasper.reactive_grasp as reactive_grasper

# import utitlies
from object_manipulator.convert_functions import *
from object_manipulator.draw_functions import *
import utilities
from utilities import *

class ArmManager():
    def __init__(self, tf_listener):
        self.LEFT = 1
        self.RIGHT = 0

        self.arm_above_and_to_side_angles = [[-0.968, 0.729, -0.554, -1.891, -1.786, -1.127, 0.501],
                                             [0.968, 0.729, 0.554, -1.891, 1.786, -1.127, 0.501]]
        self.arm_to_side_angles = [[-2.135, 0.803, -1.732, -1.905, -2.369, -1.680, 1.398],
                                   [2.135, 0.803, 1.732, -1.905, 2.369, -1.680, 1.398]]
        self.traj_from_above_to_side = lambda whicharm: [self.arm_above_and_to_side_angles[whicharm], self.arm_to_side_angles[whicharm]]
        self.use_slip_detection = 0

        self.tf_listener = tf_listener
        self.ams = [arm_manager.ArmManager('r', self.tf_listener, self.use_slip_detection),
                    arm_manager.ArmManager('l', self.tf_listener, self.use_slip_detection)]
        self.rgs = [reactive_grasper.ReactiveGrasper(self.ams[0]), 
                    reactive_grasper.ReactiveGrasper(self.ams[1])]
    
    def move_home(self,arm):
        whicharm = self.LEFT if arm == 'left' else self.RIGHT
        #self.ams[whicharm].command_joint_trajectory(self.traj_from_above_to_side(whicharm), blocking=1)
        self.guarded_move_joint_traj(arm, self.traj_from_above_to_side(whicharm))

    def move_left_away(self):
        self.ams[self.LEFT].command_joint_trajectory([[0.968, 0.729, 0.554, -1.891, 1.786, -1.127, 0.501]],blocking=1)

    def move_cartesian(self, arm, pose):
        whicharm = self.LEFT if arm == 'left' else self.RIGHT
        self.ams[whicharm].move_cartesian(pose,settling_time = rospy.Duration(1000))

    def move_joints(self,arm,joints,max_vel=.2):
        whicharm = self.LEFT if arm == 'left' else self.RIGHT
        self.ams[whicharm].command_joint_trajectory([joints], max_joint_vel = max_vel,blocking=1)

    def get_ik(self, arm, pose):
        whicharm = self.LEFT if arm == 'left' else self.RIGHT
        start_angles = self.ams[whicharm].get_current_arm_angles()
        link_name = self.ams[whicharm].ik_utilities.link_name
        return self.ams[whicharm].ik_utilities.run_ik(stamp_pose(pose,'base_link'), start_angles, link_name, collision_aware = 0)

    def get_current_pose(self, arm):
        whicharm = self.LEFT if arm == 'left' else self.RIGHT
        return self.ams[whicharm].get_current_wrist_pose_stamped()

    def open_gripper(self, arm):
        whicharm = self.LEFT if arm == 'left' else self.RIGHT
        self.ams[whicharm].command_gripper(.1, -1, 1)

    def close_gripper(self, arm):
        whicharm = self.LEFT if arm == 'left' else self.RIGHT
        self.ams[whicharm].command_gripper(0, 100, 1)

    def guarded_move_joints(self, arm, joints, max_vel = .2):
        whicharm = self.LEFT if arm == 'left' else self.RIGHT
        self.rgs[whicharm].guarded_move_joints(joints, max_vel)

    def guarded_move_joint_traj(self, arm, joint_traj, max_vel = .2):
        whicharm = self.LEFT if arm == 'left' else self.RIGHT
        self.rgs[whicharm].guarded_move_joint_traj(joint_traj, max_vel)

class fixHead(threading.Thread):
    def __init__(self, point_head_action_client):
        threading.Thread.__init__(self)
        self.point_head_action_client = point_head_action_client
        self.shutdown = False

    def run(self):
        while not self.shutdown:
            point_head_goal = PointHeadGoal()
            point_head_goal.target.header.frame_id = '/l_gripper_r_finger_tip_link'
            point_head_action_client.send_goal(point_head_goal)
       
def pullBook(book):
    if len(book.cluster.points) == 0:
        rospy.loginfo('bad pullbook request, too few points')
        return
    rospy.loginfo('pulling book')

    global pullDone
    if pullDone == True:
        return
    global bounding_box_srv, find_bounding_box_name
    bb_req = FindClusterBoundingBoxRequest()
    bb_req.cluster = book.cluster
    try:
        bb_res = bounding_box_srv(bb_req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling %s: %s"%(find_bounding_box_name, e))
        sys.exit()
    #print bb_res, dir(bb_res.pose.pose)

    rot_mat = qy(pi/2.0)*qx(pi/2.0)
    prepare_orientation = tf.transformations.quaternion_from_matrix(rot_mat)

    prepare_pose = Pose(Point(*[.65,0.,1.2]),Quaternion(*prepare_orientation))
    #print 'ppppppppppppppprepare_pose',prepare_pose
    (prepare_joint_angles, prepare_success) = ac.get_ik('left',prepare_pose)
    #print 'ready to move to prepare position?'
    #pause()
    ac.move_joints('left',prepare_joint_angles)

    prepare_pose = Pose(Point(*[bb_res.pose.pose.position.x-.1,
                                bb_res.pose.pose.position.y,
                                bb_res.pose.pose.position.z+.4]),
                        Quaternion(*prepare_orientation))
    #print prepare_pose
    
    push_pose = Pose(Point(*[bb_res.pose.pose.position.x-.1,
                             bb_res.pose.pose.position.y,
                             bb_res.pose.pose.position.z+.2]), 
                     Quaternion(*prepare_orientation))
    #print push_pose
    #draw_grasp(df,push_pose)

    slide_pose = Pose(Point(*[bb_res.pose.pose.position.x-.25,
                              bb_res.pose.pose.position.y,
                              bb_res.pose.pose.position.z+.2]), 
                      Quaternion(*prepare_orientation))
    #print slide_pose
    #draw_grasp(df,slide_pose)

    (prepare_joint_angles, prepare_success) = ac.get_ik('left',prepare_pose)
    (push_joint_angles, push_success) = ac.get_ik('left', push_pose)
    (slide_joint_angles, slide_success) = ac.get_ik('left', slide_pose)

    print 'prepare_success?', prepare_success
    print 'push_success?', push_success
    print 'slide_success?', slide_success

    #print 'ready to move the robot?'
    #pause()

    ac.open_gripper('left')
    ac.move_joints('left',prepare_joint_angles)
    #print 'ready to push down to the book?'
    #pause()
    ac.guarded_move_joints('left',push_joint_angles)
    #print 'ready to slide out?'
    #pause()
    ac.move_joints('left',slide_joint_angles)
    #print 'ready to move away?'
    #pause()

    #ac.move_home('left')
    ac.move_left_away()
    pullDone=True
    
    print 'sending pull done message'
    global pub
    done_msg = PullStatus()
    done_msg.pullValue = True
    pub.publish(done_msg)
    print 'pull done message sent'

def pickBook(book):
    #global pullDone
    #if pullDone == False:
    #    return
    global bounding_box_srv, find_bounding_box_name
    bb_req = FindClusterBoundingBoxRequest()
    bb_req.cluster = book.cluster
    
    #print 'displaying target book cloud'
    #draw_pts(df,book.cluster.points)
    try:
        bb_res = bounding_box_srv(bb_req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling %s: %s"%(find_bounding_box_name, e))
        sys.exit()
    #print bb_res, dir(bb_res.pose.pose)
    #print 'displaying bounding box of the book cloud'
    #draw_bb(df,pose_to_mat(bb_res.pose.pose), bb_res.box_dims)
    #pause()

    rot_mat = qy(0.0)
    prepare_orientation = tf.transformations.quaternion_from_matrix(rot_mat)

    prepare_pose = Pose(Point(*[bb_res.pose.pose.position.x-.3,
                                bb_res.pose.pose.position.y,
                                bb_res.pose.pose.position.z+.1]),
                        Quaternion(*prepare_orientation))
    print prepare_pose
    #draw_grasp(df,prepare_pose)

    pick_pose = Pose(Point(*[bb_res.pose.pose.position.x-.2,
                             bb_res.pose.pose.position.y,
                             bb_res.pose.pose.position.z]), 
                     Quaternion(*prepare_orientation))
    print pick_pose
    #draw_grasp(df,pick_pose)

    lift_pose = Pose(Point(*[bb_res.pose.pose.position.x-.17,
                              bb_res.pose.pose.position.y,
                              bb_res.pose.pose.position.z+.4]), 
                     Quaternion(*prepare_orientation))
    print lift_pose
    #draw_grasp(df,lift_pose)

    look_orientation = tf.transformations.quaternion_from_matrix(qz(pi/2.0)*qy(pi/2.0))
    look_pose = Pose(Point(*[.4,.6,1.2]), 
                     Quaternion(*look_orientation))
    print look_pose

    (prepare_joint_angles, prepare_success) = ac.get_ik('left',prepare_pose)
    (pick_joint_angles, pick_success) = ac.get_ik('left', pick_pose)
    (lift_joint_angles, lift_success) = ac.get_ik('left', lift_pose)
    (look_joint_angles, look_success) = ac.get_ik('left', look_pose)

    print 'prepare_success?', prepare_success
    print 'pick_success?', pick_success
    print 'lift_success?', lift_success
    print 'look success?', look_success

    #print 'ready to move the robot?'
    #pause()

    ac.open_gripper('left')
    ac.move_joints('left',prepare_joint_angles)
    #print 'ready to pick the book?'
    #pause()
    global fixingHead, point_head_action_client
    fixingHead.start()
    ac.move_joints('left',pick_joint_angles)
    ac.close_gripper('left')
    #print 'ready to lift?'
    #pause()
    ac.move_joints('left',lift_joint_angles)
    #print 'ready to look at the book?'
    #pause()
    ac.move_joints('left',look_joint_angles)
    fixingHead.shutdown = True
    point_head_goal = PointHeadGoal()
    point_head_goal.target.header.frame_id = '/base_link'
    point_head_goal.target.point = Point(*[-.65,1.0,.8])
    point_head_action_client.send_goal(point_head_goal)
    print 'ready to hit jared?'
    pause()
    fixingHead = fixHead(point_head_action_client)
    fixingHead.start()
    hit_angles_1 = [1.60,  0.49,  -0.07,  -1.94, -74.28,  -0.01, -17.60]
    #hit_angles_2 = [1.83,  0.19,  -0.71, -0.62, -74.31, 0.00, -17.62]
    hit_angles_2 = [1.97,       0.10 ,     -0.66,      -0.15,     -74.55,      -0.01,     -17.54]
    hit_angles_3 = [1.24,  0.18,  -0.71, -1.56, -74.07, -0.01, -17.43]
    ac.move_joints('left',hit_angles_1,max_vel=1.0)
    ac.move_joints('left',hit_angles_2,max_vel=1.0)
    ac.move_joints('left',hit_angles_3,max_vel=1.0)
    print 'ready to move home?'
    pause()
    fixingHead.shutdown = True
    ac.move_home('left')

rospy.init_node('pick_and_place_tutorial_python', anonymous=True)
rospy.loginfo("initializing...")
pub = rospy.Publisher("pullStatus",PullStatus)
pullDone=False

tf_listener = tf.TransformListener()
find_bounding_box_name = '/find_cluster_bounding_box'
bounding_box_srv = rospy.ServiceProxy(find_bounding_box_name, FindClusterBoundingBox)
point_head_action_client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
fixingHead = fixHead(point_head_action_client)

ac = ArmManager(tf_listener)
df = DrawFunctions('mmmmmarker')

point_head_goal = PointHeadGoal()
point_head_goal.target.header.frame_id = '/base_link'
point_head_goal.target.point = Point(*[.6,0.,.6])
#point_head_goal.target.point = Point(*[-.65,1.0,.8])

point_head_action_client.send_goal(point_head_goal)

ac.open_gripper('left')
ac.move_home('left')
rospy.Subscriber("objectToPull", GraspableObject, pullBook)
rospy.Subscriber("objectToPick", GraspableObject, pickBook)

rospy.spin()
