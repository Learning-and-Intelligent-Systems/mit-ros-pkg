import roslib; roslib.load_manifest('tabletop_for_cabinet')
import sys
import rospy
from tabletop_for_cabinet.srv import *
from geometry_msgs.msg import Pose, PoseStamped
from tabletop_object_detector.msg import Table
from pr2_pick_and_place_demos.pick_and_place_manager import PickAndPlaceManager
import actionlib
import opendoors.msg

def get_table():
    '''   
    Uses tabletop detection to find a plane which corresponds to the cabinet door 
    
    @rtype: tabletop_object_detector/Table                                         
    @returns the detected cabinet door                                                                             
    '''
    ppm = PickAndPlaceManager()
    (things,table) = ppm.call_tabletop_detection(take_static_collision_map=1, update_table=1)
    print table
    return table

def move_arm(pose):
    gripper_client = actionlib.SimpleActionClient("grasp_handle_action", opendoors.msg.GraspHandleAction)
    print "wait for server"
    gripper_client.wait_for_server()

    goal = opendoors.msg.GraspHandleGoal()
    goal.pose_stamped = pose
    goal.arm = 'right_arm'
    
    print "waiting for result"
    gripper_client.send_goal_and_wait(goal)

def door_processing_client():
    '''
    Uses tabletop detection to find the cabinet door. Returns the door and the list of clusters associated with the door.

    @rtype: tabletop_object_detector/TabletopDetectionResult
    @returns the cabinet door as a table and the vector of the clusters of possible handles
    '''
    print 'entering door_processing client'
    rospy.wait_for_service('cabinet_table')
    print 'got door service'
    try:
        door_processing = rospy.ServiceProxy('cabinet_table', cabinet_table)
        resp1 = door_processing(get_table())
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

        

def object_processing_client():
    '''                      
    Finds the handle on the cabinet door and returns its pose.   
    
    @rtype: geometry_msgs/PoseStamped                                               
    @returns the pose of the handle on the cabinet door      
    '''
    print 'entering object_processing_client'
    rospy.wait_for_service('cabinet_table_result')
    print 'got object service'
    try:
        object_processing = rospy.ServiceProxy('cabinet_table_result', cabinet_table_result)
        resp1 = object_processing(door_processing_client().detection)
        return resp1.pose
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    rospy.init_node("tabletop_detection_cabinet")
    pose = object_processing_client()
    pose_stamped = geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id = 'torso_lift_link'
    pose_stamped.pose.position.x = 0.620342731476
    pose_stamped.pose.position.y = -0.0527813695371
    pose_stamped.pose.position.z = 0.105797514319
    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0
    #print "cabinet's pose="+str(pose)
#    move_arm(pose_stamped)

    print "handle pose="+str(pose)
#    move_arm(pose)
