import roslib
roslib.load_manifest('bakebot')
import rospy
from bakebot.srv import *
import tf
import actionlib
import math
import sys
import time
from  opendoors_executive.srv import *
from  opendoors_executive.msg import *

class OvenDoorClient:
    
    oven_door_client_instance = None
    MAX_LINEAR_VELOCITY = 0.25

    @staticmethod
    def get_oven_door_client():
        if OvenDoorClient.oven_door_client_instance == None:
            rospy.loginfo('instantiating a new oven door client')
            OvenDoorClient.oven_door_client_instance = OvenDoorClient()
        else:
            rospy.loginfo('returning existing instance of oven door client ')
        return OvenDoorClient.oven_door_client_instance

    def __init__(self):
        pass

    def open_door(self):
        '''
        This method should open the cabinet and return True
        if the cabinet opened successfully (if this can't be determined, return
        True anyway and we'll figure it out later).
        '''

        # TODO: Annie: this is setup with the old way with the open_cupboard_action.  Can you
        # update this to open the toaster with the curent set of code?

        # end result is that it has the hand off of hte handle with the door open
        open_cupboard_client = actionlib.SimpleActionClient('open_cupboard_action', opendoors_executive.msg.OpenCupboardAction)
        print "waiting for server"
        open_cupboard_client.wait_for_server()
        goal = opendoors_executive.msg.OpenCupboardGoal()
        goal.open_cupboard = True
        print "sending goal"
        open_cupboard_client.send_goal(goal)
        print "waiting for result"
        open_cupboard_client.wait_for_result()
        print "RESULT:",open_cupboard_client.get_result()
        return True

    def get_door_pos(self):
        '''
        returns the door position relative ot the robot in the base_link frame
        if this is > 500, means it could not find the door
        otherwise the robot will try to drive this distance in meters
        to get to the ideal position from which to open the door
        '''
        # TODO: Annie: this is setup with the old way with the open_cupboard_action.  Can you
        # update this to work with the current set of code?  Is this feature even implemented anymore?

        rospy.wait_for_service('table_status')
        door_dist_fxn = rospy.ServiceProxy('table_status', table_status)
        resp = door_dist_fxn()
        print 'dist: ', resp.dist 
        return resp.dist

if __name__ == '__main__':
    rospy.init_node('arm_client_ui', anonymous=True)
    odc = OvenDoorClient.get_oven_door_client()
    odc.open_door()
