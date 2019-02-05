#!/usr/bin/env python
import roslib
roslib.load_manifest('kinect_test')
import rospy
import sensor_msgs.msg
from sensor_msgs.msg import PointCloud2, PointField
import struct
import math
import numpy
import cv
import rosbag
import tf
from tf import transformations
#import kinematics_msgs.msg
#import kinematics_msgs.srv
#import pr2_controllers_msgs.msg
import actionlib

#this rooughly matches what is done in the C++ code
def edge_detect2(image):
    gray = cv.CreateMat(image.rows, image.cols, cv.CV_8UC1)
    cv.CvtColor(image,gray,cv.CV_BGR2GRAY)
    edges = cv.CreateMat(image.rows,image.cols,cv.CV_8UC1)
    threshold1 = 6711
    threshold2 = 3089
    aperature_size = 7
    cv.Canny(gray,edges,threshold1,threshold2,aperature_size)
    return edges

def edge_detect(image):
    gray = cv.CreateMat(image.rows, image.cols, cv.CV_8UC1)
    cv.CvtColor(image,gray,cv.CV_BGR2GRAY)
    edges = cv.CreateMat(image.rows,image.cols,cv.CV_8UC1)
    threshold1 = 100
    threshold2 = 50
    #aperature_size = 7
    cv.Canny(gray,edges,threshold1,threshold2)#,aperature_size)
    return edges

def detect_circles(image):
    storage = cv.CreateMat(1, 2, cv.CV_32FC3)
    gray = cv.CreateMat(image.rows, image.cols, cv.CV_8UC1)
    cv.CvtColor(image,gray,cv.CV_BGR2GRAY)
    cv.Smooth(gray,gray,cv.CV_GAUSSIAN, 9, 9)
    circles = []
    try:
        cv.HoughCircles(gray, storage, cv.CV_HOUGH_GRADIENT, 2, 300, 200, 100)
        for r in range(storage.rows):
            for c in range(storage.cols):
                center = (int(storage[r,c][0]),int(storage[r,c][1]))
                radius = int(storage[r,c][2])
                circles = circles + [(center, radius)]
    except:
        pass
    return circles

    

def cloud_to_image(cloud):
    m = cv.CreateMat(cloud.height,cloud.width,cv.CV_8UC3)
    for (u,v,x,y,z,b,g,r) in points(cloud):
        m[v,u] = (b,g,r)
    return m

#points the right gripper at x,y,z in the frame of the kinect
#returns False if it couldn't reach
rot = (0,-.5,0,.5)
def point_at(x,y,z):
    (success, goal) = ik(transform_matrix((x,y,z-.2),rot))
    if success:
        execute(goal)
    else:
        print 'Could not move the hand to that location.'
    return success

def call_tf(a,b):
  r = rospy.Rate(10)
  success = False
  while not success or not tl.frameExists(a) or not tl.frameExists(b):
    r.sleep()
    try:
      (wpos,wrot) = tl.lookupTransform(a,b,rospy.Time(0))
      success = True
    except Exception, e:
      print e
    #except Exception, e:
    #  print e
  return (wpos,wrot)

def transform_matrix(pos,rot):
  #T = transformations.quaternion_matrix(rot)
  #T[0][3] = pos[0]
  #T[1][3] = pos[1]
  #T[2][3] = pos[2]
  T = numpy.dot(transformations.translation_matrix(pos), transformations.quaternion_matrix(rot))
  return T
def transform_from_matrix(T):
  r = transformations.quaternion_from_matrix(T)
  o = (T[0][3],T[1][3],T[2][3])
  return o,r


def ik(T,arm='r'):
  initialize()
  ((px,py,pz),(x,y,z,w)) = transform_from_matrix(T)
  if arm=='r':
    whicharm = 'right'
  else:
    whicharm = 'left'

  rospy.wait_for_service("pr2_"+whicharm+"_arm_kinematics/get_ik_solver_info")
  collision_free = False
  if collision_free:
    rospy.wait_for_service("pr2_"+whicharm+"_arm_kinematics/get_constraint_aware_ik")
  else:
    rospy.wait_for_service("pr2_"+whicharm+"_arm_kinematics/get_ik")
  #try:
  if True:
    query_client = rospy.ServiceProxy("pr2_"+whicharm+"_arm_kinematics/get_ik_solver_info",
                                       kinematics_msgs.srv.GetKinematicSolverInfo)
    response = query_client()
    
    
    if collision_free:
      ik_client = rospy.ServiceProxy("pr2_"+whicharm+"_arm_kinematics/get_constraint_aware_ik",
                                    kinematics_msgs.srv.GetConstraintAwarePositionIK)
      gpik_res = kinematics_msgs.srv.GetConstraintAwarePositionIK
    else:
      ik_client = rospy.ServiceProxy("pr2_"+whicharm+"_arm_kinematics/get_ik",
                                    kinematics_msgs.srv.GetPositionIK)
      gpik_res = kinematics_msgs.srv.GetPositionIK

    ik_request = kinematics_msgs.msg.PositionIKRequest()
    ik_request.ik_link_name = arm+"_wrist_roll_link"
    ik_request.pose_stamped.header.frame_id = '/openni_rgb_optical_frame'#camera_frame
    ik_request.pose_stamped.pose.position.x = px
    ik_request.pose_stamped.pose.position.y = py
    ik_request.pose_stamped.pose.position.z = pz
    ik_request.pose_stamped.pose.orientation.x = x
    ik_request.pose_stamped.pose.orientation.y = y
    ik_request.pose_stamped.pose.orientation.z = z
    ik_request.pose_stamped.pose.orientation.w = w

    ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names
    sz = len(response.kinematic_solver_info.joint_names)
    #ik_request.ik_seed_state.joint_state.position.resize();

    for i in range(sz):
      ik_request.ik_seed_state.joint_state.position.append((response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0);

    timeout = rospy.Duration(5.0)
    #ik_client(ik_request, timeout)
    gpik_res = ik_client(ik_request=ik_request, timeout=timeout)
    if gpik_res.error_code.val != gpik_res.error_code.SUCCESS:
      return (False,None)
    
    goal = pr2_controllers_msgs.msg.JointTrajectoryGoal()
    
    goal.trajectory.joint_names.append(arm+"_shoulder_pan_joint")
    goal.trajectory.joint_names.append(arm+"_shoulder_lift_joint")
    goal.trajectory.joint_names.append(arm+"_upper_arm_roll_joint")
    goal.trajectory.joint_names.append(arm+"_elbow_flex_joint")
    goal.trajectory.joint_names.append(arm+"_forearm_roll_joint")
    goal.trajectory.joint_names.append(arm+"_wrist_flex_joint")
    goal.trajectory.joint_names.append(arm+"_wrist_roll_joint")

    class pvt:
      def __init__(self):
        self.positions = []
        self.velocities = []
        self.accelerations = []
        self.time_from_start = rospy.Duration(2.0)
    goal.trajectory.points.append(pvt())
    for j in range(7):
      goal.trajectory.points[0].positions.append(gpik_res.solution.joint_state.position[j])
      goal.trajectory.points[0].velocities.append(0.0)
      goal.trajectory.points[0].accelerations.append(0.0)

    #goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.0);
    return (True,(arm,goal))
  #except rospy.ServiceException, e:
  #  print "exception: %s"%e
  
def execute(goal):
  initialize()
  (arm,goal) = goal
  traj_client = actionlib.SimpleActionClient(arm+"_arm_controller/joint_trajectory_action",
                                             pr2_controllers_msgs.msg.JointTrajectoryAction)
  traj_client.wait_for_server()
  goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.0);
  traj_client.send_goal(goal)
  traj_client.wait_for_result()
  rospy.sleep(3.0)
    #print traj_client.get_result()
  #except rospy.ServiceException, e:
  #  print "exception: %s"%e


def points(cloud):
    assert isinstance(cloud, roslib.message.Message) and cloud._type == 'sensor_msgs/PointCloud2', 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = '<fffxxxxBBBx'
    width, height, point_step, row_step, data = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data
    unpack_from = struct.Struct(fmt).unpack_from
    for v in xrange(height):
        offset = row_step * v
        for u in xrange(width):
            yield (u,v)+unpack_from(data, offset)
            offset += point_step
PointCloud2.points = points

def cloud_to_depth_image(cloud):
    max_depth = max([p[4] for p in points(cloud)])
    min_depth = min([p[4] for p in points(cloud)])
    print 'max_depth: '+str(max_depth)
    print 'min_depth: '+str(min_depth)
    from math import isnan
    print 'any is not nan?: '+str(any([not isnan(p[4]) for p in points(cloud)]))


#overload [] operator for a PointCloud2
def __getitem__(self, index):
    if type(index)!=tuple or len(index)!=2 or type(index[0])!=int or type(index[1])!=int:
      raise TypeError('PointCloud2 indices must be a pair of integers (u,v)')
    (u,v) = index
    if v<0 or v>=self.height or u<0 or u>=self.width:
        raise IndexError('PointCloud2 index '+str(index)+' is out of bounds ('+str(self.height)+','+str(self.width)+').')
    offset = self.row_step * v + self.point_step * u
    return struct.Struct('<fffxxxxBBBx').unpack_from(self.data,offset)
    
PointCloud2.__getitem__ = __getitem__
PointCloud2.__repr__ = lambda (self): '<PointCloud2>' #prevent hanging for big clouds


initialized = False
def initialize():
    global initialized
    if not initialized:
        print 'initializing a new ROS node...'
        rospy.init_node('kinect_test',anonymous=True)
        initialized = True


class cloud_source:
    topic = '/camera/rgb/points'
    def callback(self, cloud):
        self.cloud = cloud
        self.received_new_cloud = True

    def next(self):
        if self.live_from_kinect:
            r = rospy.Rate(30)
            while not self.received_new_cloud:
                r.sleep()
            self.received_new_cloud = False
            return self.cloud
        else:
            (topic, msg, t) = self.bag.next()
            self.cloud = msg
            return msg
            #for topic, msg, t in self.bag:
            #    return msg
    def __iter__(self):
        return self

    def __init__(self, bag_file_name=None):
        if bag_file_name == None:
            initialize()
            print 'subscribing to '+self.topic+'...'
            self.subscriber = rospy.Subscriber(self.topic,PointCloud2,self.callback)
            print 'waiting for one message...'
            self.live_from_kinect = True
            self.received_new_cloud = False
            self.next()
            print 'done.'
            global tl
            tl = tf.TransformListener()
        else:
            print 'opening bag file '+bag_file_name
            self.bag = rosbag.Bag(bag_file_name).read_messages(topics=[self.topic])
            self.live_from_kinect = False
