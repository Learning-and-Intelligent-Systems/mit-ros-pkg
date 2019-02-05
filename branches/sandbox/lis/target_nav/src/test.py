#! /usr/bin/python
#23456789 123456789 123456789 123456789 123456789 123456789 123456789 123456789 
import roslib
roslib.load_manifest('target_nav')
import rospy

##for navigation/head actions
import actionlib
#roslib.load_manifest('actionlib')


import pr2_controllers_msgs.msg

#roslib.load_manifest('move_base_msgs')
#import actionlib
#import move_base
#import move_base_msgs.msg


#for chessboard:
import checkerboard_pose.srv
import geometry_msgs.msg


#for tf:
import tf
from tf import transformations
import numpy

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
last_good_cb = False
def get_cb_in_odom_frame():
  global last_good_cb, has_detected
  cb = cb_client(6,7,.108,.108)
  if cb.board_pose.header.frame_id=='':
    if has_detected:
      return (True,last_good_cb)
    else:
      return (False,transform_matrix((0,0,0),(0,0,0,1)))
  pos = cb.board_pose.pose.position
  p = (pos.x,pos.y,pos.z)
  rot = cb.board_pose.pose.orientation
  r = (rot.x,rot.y,rot.z,rot.w)
  cTo = transform_matrix(p,r)
  (p,r) = call_tf('odom_combined',cb.board_pose.header.frame_id)
  bTc = transform_matrix(p,r)
  bTo = numpy.dot(bTc,cTo)
  last_good_cb = bTo
  has_detected = True
  return (True,bTo)


def point_the_head(pos):
  client = actionlib.SimpleActionClient(
     '/head_traj_controller/point_head_action', pr2_controllers_msgs.msg.PointHeadAction)
  client.wait_for_server()
  g = pr2_controllers_msgs.msg.PointHeadGoal()
  g.target.header.frame_id = '/odom_combined'
  #(pos,rot) = transform_from_matrix(T) 
  g.target.point.x = pos[0]
  g.target.point.y = pos[1]
  g.target.point.z = pos[2]
  g.min_duration = rospy.Duration(.5)

  #client.send_goal_and_wait(g)
  client.cancel_all_goals()
  client.send_goal(g)


if __name__ == '__main__':
    rospy.init_node('target_nav', anonymous=False)
    tl = tf.TransformListener()


    service_name = "get_checkerboard_pose"
    print "waiting for "+service_name
    rospy.wait_for_service(service_name)
    cb_client = rospy.ServiceProxy(service_name,checkerboard_pose.srv.GetCheckerboardPose)
    print "done."


    '''
    ac = actionlib.SimpleActionClient(
        'move_base_local', move_base_msgs.msg.MoveBaseAction)
    print "waiting for move_base server..."
    ac.wait_for_server()
    print "done."

    
    dx = .5
    '''


    
    
    while True:
      (success, T) = get_cb_in_odom_frame()
      
      if success:
        (p0,r0) = transform_from_matrix(T)
        print p0,r0
        point_the_head(p0)
      else:
        print "no checkerboard"
      rospy.sleep(.1)

    '''
    goal = move_base_msgs.msg.MoveBaseGoal()
    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = dx
    goal.target_pose.pose.orientation.w = 1.0
    ac.send_goal_and_wait(goal)

    (p1,r1) = transform_from_matrix(get_cb_in_base_frame())


    print "desired change:",dx


    print "actual  change:",-(p1[0]-p0[0])

    #while True:
    #  print cb_client(6,7,.108,.108)
    #  rospy.sleep(.1)
    '''
