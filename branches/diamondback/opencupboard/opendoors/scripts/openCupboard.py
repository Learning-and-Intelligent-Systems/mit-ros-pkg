import roslib; roslib.load_manifest('opendoors')
import rospy
import geometry_msgs.msg 
import move_arm_service.srv
import actionlib 
from geometry_msgs.msg import Pose , PoseStamped
import time
import sys
import pdb
import tf
import copy
import opendoors.msg 
import visualization_msgs.msg
import math
from ik_trajectory_tutorial.srv import ExecuteCartesianIKTrajectory


def interpolatedIK(start, end, arm):
	    ik_call = move_arm_service.srv.InterpolatedIKRequest()
	    ik_call.start_pose = start
	    ik_call.end_pose =  end
	    ik_call.arm = arm
	    print 'Calling ik'
	    ik_srv = rospy.ServiceProxy('run_interpolated_ik', move_arm_service.srv.InterpolatedIK)
	    ik_resp = ik_srv(ik_call)
	    print 'ik returned with response', ik_resp.error_code.val

def smallAngleIncrement(currentpose,radius, angle, arm):

	rospy.init_node('my_handle_cupboard_opener_client')
	rospy.wait_for_service('run_interpolated_ik')
	alpha = angle/10
	inc = 2*radius*math.sin(alpha/2)
	pose_stamped = currentpose
	start = copy.deepcopy(pose_stamped) 
#	middle = copy.deepcopy(pose_stamped)
#	middle.pose.position.x = middle.pose.position.x + 0.18*(1-math.cos(alpha))
#	middle.pose.position.y = middle.pose.position.y - 0.18*math.sin(alpha)
#	middle.pose.orientation.x = 0.0
#	middle.pose.orientation.y = 0.0 
#	middle.pose.orientation.z = math.sin(alpha/2)
#	middle.pose.orientation.w = math.cos(alpha/2)
#	end = copy.deepcopy(start)
#	end.pose.position.x = start.pose.position.x - radius*math.sin(alpha)
#	end.pose.position.y = start.pose.position.y  - radius*(1-math.cos(alpha))
#	interpolatedIK(start,middle, arm)
#	interpolatedIK(start,end, arm)
	pub = rospy.Publisher('chatter', visualization_msgs.msg.Marker)
	
	for i in range(0,10):
# list of triangles; t
		
		print "i is ",i
		
		currentAngle = 2*math.asin(start.pose.orientation.z)
		currentAngle = currentAngle + alpha



#		middle.pose.orientation.x = 0.0
#		middle.pose.orientation.y = 0.0 
#		middle.pose.orientation.z = math.sin(currentAngle/2)
#		middle.pose.orientation.w = math.cos(currentAngle/2)
#		middle.pose.position.x = middle.pose.position.x + 0.18*(math.cos(i*alpha)-math.cos((i+1)*alpha))  
#		middle.pose.position.y = middle.pose.position.y - 0.18*(math.sin((i+1)*alpha) - math.sin(i*alpha))
		
		end = copy.deepcopy(start)
		end.pose.position.x = start.pose.position.x - radius*(math.sin(alpha*(i+1)) - math.sin(alpha*i))
		end.pose.position.y = start.pose.position.y + radius*( math.cos(alpha*(i+1)) - math.cos(alpha*i)) 

		interpolatedIK(start,end, arm)
		print "end pose is", end.posemove_arm_path_constraints
		start = copy.deepcopy(end)
		marker = visualization_msgs.msg.Marker()		
		marker.header.frame_id = end.header.frame_id
        	marker.header.stamp = rospy.Time.now()
       	 	marker.ns = "basic_shapes"
        	marker.id = i
    		marker.type = visualization_msgs.msg.Marker.CUBE
    		marker.action = visualization_msgs.msg.Marker.ADD
move_arm_path_constraints
  
  		marker.pose = end.pose
        	
    
        	marker.scale.x =move_arm_path_constraints 0.05move_arm_path_constraints
       	 	marker.scale.y = 0.05
       	 	marker.scale.z = 0.05

     
        	marker.color.r = 0.0
        	marker.color.g = 1.0
        	marker.color.b = 0.0
        	marker.color.a = 1.0
		marker.lifetime = rospy.Duration()

	        pub.publish(marker)
		rospy.sleep(1)
		
	
	
	

def first():.7
	pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = 'torso_lift_link'
        pose_stamped.pose.position.x = 0.7
        pose_stamped.pose.position.y = -0.1
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
	rospy.wait_for_service("move_arm_to_pose")
	move_call = move_arm_service.srv.MoveArmRequest()
        move_call.pose_stamped = pose_stamped
        move_call.arm = 'right_arm'
	print "specified right arm"
        move_srv = rospy.ServiceProxy('move_arm_to_pose',move_arm_service.srv.MoveArm)
        print "initializing the server for right arm"
        move_resp = move_srv(move_call) 
	print "move_resp is ",move_resp.error_code 

if __name__ == '__main__':
	#main(45,60)
	first()
	pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = 'torso_lift_link'
        pose_stamped.pose.position.x = .7
        pose_stamped.pose.position.y = -0.1
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
	smallAngleIncrement(pose_stamped,0.3,math.pi/2, "right_arm")





