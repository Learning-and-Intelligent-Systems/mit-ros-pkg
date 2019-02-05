#!/usr/bin/env python
import roslib; roslib.load_manifest('target_nav')

from target_nav.srv import *
import rospy
import actionlib

#for returning the head back to it's original joint_state
import sensor_msgs.msg

import pr2_controllers_msgs.msg

#for chessboard:
import checkerboard_pose.srv
import geometry_msgs.msg

#for tf:
import tf
from tf import transformations
import numpy
import time
import threading

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
  return (wpos,wrot)

def transform_matrix(pos,rot):
  T = numpy.dot(transformations.translation_matrix(pos), transformations.quaternion_matrix(rot))
  return T

def transform_from_matrix(T):
  r = transformations.quaternion_from_matrix(T)
  p = (T[0][3],T[1][3],T[2][3])
  return p,r


has_detected = False
wTo = False #(w)orld (o)bject transform

lock = threading.Lock()
def get_odom_in_world_frame():
  lock.acquire()
  global wTo, has_detected, cb_client
  cb = cb_client(corners_x,corners_y,spacing_x,spacing_y)
  if cb.board_pose.header.frame_id=='':
    lock.release()
    return (False,transform_matrix((0,0,0),(0,0,0,1)))

  pos = cb.board_pose.pose.position
  p = (pos.x,pos.y,pos.z)
  rot = cb.board_pose.pose.orientation
  r = (rot.x,rot.y,rot.z,rot.w)
  cTo = transform_matrix(p,r) #(c)amera (o)bject transform

  (p,r) = call_tf('odom_combined',cb.board_pose.header.frame_id)
  bTc = transform_matrix(p,r) #(b)ase (c)amera transform (base is actually odom)

  bTo = numpy.dot(bTc,cTo) #(b)ase (o)bject transform

  if not has_detected:
    wTo = bTo
    has_detected = True

  oTb = transformations.inverse_matrix(bTo)

  wTb = numpy.dot(wTo,oTb)
  lock.release()
  return (True,wTb)


def joint_position(joint_states_msg, joint_name):
  for j in range(len(joint_states_msg.name)):
    if joint_states_msg.name[j]==joint_name:
      return joint_states_msg.position[j]


def point_head_goal(pos, frame_id='/odom_combined'):
  client = actionlib.SimpleActionClient(
     '/head_traj_controller/point_head_action', pr2_controllers_msgs.msg.PointHeadAction)
  client.wait_for_server()
  g = pr2_controllers_msgs.msg.PointHeadGoal()
  g.target.header.frame_id = frame_id
  g.target.point.x = pos[0]
  g.target.point.y = pos[1]
  g.target.point.z = pos[2]
  g.min_duration = rospy.Duration(.5)

  client.cancel_all_goals()
  js = rospy.wait_for_message('/joint_states',sensor_msgs.msg.JointState)
  return (client,g)

def point_the_head(pos):
  (client,g) = point_head_goal(pos)
  client.send_goal(g)


def point_the_head_and_wait(pos,frame_id='odom_combined'):
  (client,g) = point_head_goal(pos,frame_id=frame_id)
  client.send_goal(g) ### possibly return this back to client.send_goal_and_wait(g)
  #client.send_goal_and_wait(g)
  time.sleep(1.0)


polock = threading.Lock()
polock.acquire()
position_orig=(0.0,0.0,0.0)
orientation_orig=(0.0,0.0,0.0,1.0) #quaternion

def project_to_ground_plane(position_orig,orientation_orig):
  position = (position_orig[0],position_orig[1],0.0)
  (al,be,ga) = transformations.euler_from_quaternion(orientation_orig,'syxz')
  orientation = transformations.quaternion_from_euler(al,0,0,'syxz')
  return (position,orientation)


#these store the most recently seen CB location,
#without actually updating the TF frame if not in continuous update mode
(position,orientation) = project_to_ground_plane(position_orig,orientation_orig)
polock.release()


seq = 0
def localize(req):
    global position, orientation, position_orig, orientation_orig, seq
    (p,r) = call_tf('torso_lift_link','head_plate_frame')
    tTh = transform_matrix(p,r)
    h = numpy.dot(tTh,[[1],[0],[0],[1]])
    original_head_position = [h[0][0],h[1][0],h[2][0]]


    polock.acquire()
    wTb = transform_matrix(position_orig,orientation_orig)
    polock.release()
    bTo = numpy.dot(transformations.inverse_matrix(wTb),wTo)
    (p,r) = transform_from_matrix(bTo)
    point_the_head_and_wait(p)

    r = rospy.Rate(10) # 10hz
    for j in range(10): #try for up to a second
      (success, wTb) = get_odom_in_world_frame()
      if success: break
      r.sleep()

    if success:
      polock.acquire()
      (position_orig,orientation_orig) = transform_from_matrix(wTb)
      (position,orientation) = project_to_ground_plane(position_orig,orientation_orig)
      polock.release()
    elif req.scan:
      pass

    point_the_head_and_wait(original_head_position,frame_id='torso_lift_link')

    ps = geometry_msgs.msg.PoseStamped()
    #print ps
    ps.header.stamp = rospy.Time.now()
    ps.header.seq = seq
    seq += 1
    ps.header.frame_id = "/odom_combined"
    pt = geometry_msgs.msg.Point()
    polock.acquire()
    pt.x = position[0]
    pt.y = position[1]
    pt.z = position[2]
    ps.pose.position = pt
    ps.pose.orientation.x = orientation[0]
    ps.pose.orientation.y = orientation[1]
    ps.pose.orientation.z = orientation[2]
    ps.pose.orientation.w = orientation[3]
    #print ps
    polock.release()

    return UpdateEstimateResponse(success, ps)

continuous = False

def continuous_update(req):
  global continuous
  previous_value = continuous
  continuous = req.continuous
  return previous_value

corners_x = 6
corners_y = 7
spacing_x = .108
spacing_y = .108
def loc_server():
    global tl, cb_client, corners_x, corners_y, spacing_x, spacing_y
    rospy.init_node('Localization_server')
    corners_x = rospy.get_param('target_nav/corners_x',6)
    corners_y = rospy.get_param('target_nav/corners_y',7)
    spacing_x = rospy.get_param('target_nav/spacing_x',.108)
    spacing_y = rospy.get_param('target_nav/spacing_y',.108)
    print corners_x, corners_y, spacing_x, spacing_y
    tl = tf.TransformListener()
    br = tf.TransformBroadcaster()
    s = rospy.Service('/checkerboard_localize', UpdateEstimate, localize)

    s2 = rospy.Service('continuous_update',ContinuousUpdate, continuous_update)

    service_name = "/get_checkerboard_pose"
    print "waiting for "+service_name
    rospy.wait_for_service(service_name)
    cb_client = rospy.ServiceProxy(service_name,checkerboard_pose.srv.GetCheckerboardPose)
    print "done."


    threading.Thread(target=update_cb).start()
    print "Ready to localize."
    r = rospy.Rate(30) # 30hz
    while not rospy.is_shutdown():
        r.sleep()
        polock.acquire()
        br.sendTransform(position,orientation, rospy.Time.now(), "/odom_combined", "/world")
        polock.release()


def update_cb():
  global position, orientation, position_orig, orientation_orig

  while not rospy.is_shutdown():
    (success, wTb_orig) = get_odom_in_world_frame()
    if success:
      polock.acquire()
      (position_orig,orientation_orig) = transform_from_matrix(wTb_orig)
      polock.release()
      print "Found the checkerboard"
      if continuous:
        polock.acquire()
        (position,orientation) = project_to_ground_plane(position_orig,orientation_orig)
        polock.release()
        wTb = wTb_orig
    if continuous and has_detected:
      bTo = numpy.dot(transformations.inverse_matrix(wTb),wTo)
      (pos,rot) = transform_from_matrix(bTo)
      point_the_head(pos)

if __name__ == "__main__":
    loc_server()
