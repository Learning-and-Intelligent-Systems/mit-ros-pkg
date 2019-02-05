#!/usr/bin/python

# a simple version of pick and place demo using code from pr2_tabletop_manipulation_apps/pr2_pick_and_place_demos/src/pr2_pick_and_place_manager.py
# comments from http://www.ros.org/wiki/pr2_tabletop_manipulation_apps/Tutorials/Writing a Simple Pick and Place Application

import roslib
roslib.load_manifest('pr2_pick_and_place_tutorial_python')
import rospy

# import messages and services
from object_manipulation_msgs.msg import PickupAction, PickupGoal, PlaceAction, PlaceGoal, ManipulationResult, GraspableObject, GripperTranslation
from tabletop_object_detector.msg import TabletopDetectionResult
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing, TabletopCollisionMapProcessingRequest

# import libraries
import actionlib
import tf
import scipy
import math
import time
import sys

# import utitlies
from object_manipulator.convert_functions import *

if __name__ == '__main__':
    '''
    initializtion
    '''
    rospy.loginfo("initializing...")
    # initialize the ROS node
    rospy.init_node('pick_and_place_tutorial_python', anonymous=True)

    # start tf listener
    tf_listener = tf.TransformListener()
    
    # set service and action names
    grasper_grasp_name = 'object_manipulator/object_manipulator_pickup'
    grasper_place_name = 'object_manipulator/object_manipulator_place'
    grasper_detect_name = 'object_detection'
    collision_map_processing_name = '/tabletop_collision_map_processing/tabletop_collision_map_processing'
    
    # create action clients
    grasper_grasp_action_client = actionlib.SimpleActionClient(grasper_grasp_name, PickupAction)
    rospy.loginfo("waiting for object_manipulator_pickup action")
    grasper_grasp_action_client.wait_for_server()
    grasper_place_action_client = actionlib.SimpleActionClient(grasper_place_name, PlaceAction)
    rospy.loginfo("waiting for object_manipulator_place action")
    grasper_place_action_client.wait_for_server()

    # wait for services
    rospy.loginfo("waiting for object_detection service")
    rospy.wait_for_service(grasper_detect_name)
    rospy.loginfo("waiting for collision_map_processing service")
    rospy.wait_for_service(collision_map_processing_name)

    # create service proxies
    grasper_detect_srv = rospy.ServiceProxy(grasper_detect_name, TabletopDetection)
    collision_map_processing_srv = rospy.ServiceProxy(collision_map_processing_name, TabletopCollisionMapProcessing)

    '''
    object detection
    '''
    rospy.loginfo("detecting objects...")
    # set up detection request
    det_req = TabletopDetectionRequest()
    # we want recognized database objects returned, set this to 0 if using the pipeline without the database
    det_req.return_models = 1
    # we want the individual object point clouds returned as well
    det_req.return_clusters = 1

    # call tabletop detection, get a detection result
    try:
        det_res = grasper_detect_srv(det_req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling %s: %s"%(grasper_detect_name, e))
        sys.exit()
    if det_res.detection.result == det_res.detection.SUCCESS:
        rospy.loginfo("tabletop detection reports success")
    else:
        rospy.logerr("tabletop detection failed with error code %d"%det_res.detection.result)
        sys.exit()

    '''
    collision map processing
    '''
    rospy.loginfo("processing collision map...")
 
    # set up collision map request
    col_req = TabletopCollisionMapProcessingRequest()
    # pass the result of the tabletop detection
    col_req.detection_result = det_res.detection    
    # ask for the existing map and collision models to be reset
    col_req.reset_static_map = 1
    col_req.reset_collision_models = 1
    col_req.reset_attached_models = 1
    # ask for a new static collision map to be taken with the laser after the new models are added to the environment
    col_req.take_static_collision_map = 1
    # ask for the results to be returned in base_link frame
    col_req.desired_frame = '/base_link'

    # call collision map processing to add the detected objects to the collision map and get back a list of GraspableObjects
    try:
        col_res = collision_map_processing_srv(col_req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling %s: %s"%(collision_map_processing_name, e))
        sys.exit()
    
    # print out detected objects
    for (ind, object) in enumerate(col_res.graspable_objects):
        if object.type == GraspableObject.DATABASE_MODEL:
            rospy.loginfo("object %d: recognized object with id %d"%(ind, object.model_pose.model_id))
        else:
            rospy.loginfo("object %d: point cluster with %d points"%(ind, len(object.cluster.points)))

    '''
    object pickup
    '''
    rospy.loginfo("grasping object...")
    # call object pickup
    goal = PickupGoal()
    # pass one of the graspable objects returned by the collision map processor
    goal.target = col_res.graspable_objects[0]
    # pass the name that the object has in the collision environment, this name was also returned by the collision map processor
    goal.collision_object_name = col_res.collision_object_names[0]
    # pass the collision name of the table, also returned by the collision map processor
    goal.collision_support_surface_name = col_res.collision_support_surface_name
    # pick up the object with the left arm
    goal.arm_name = "left_arm"
    # specify the desired distance between pre-grasp and final grasp
    goal.desired_approach_distance = .1
    goal.min_approach_distance = .05
    # we will be lifting the object along the vertical direction which is along the z axis in the base_link frame
    goal.lift = GripperTranslation()
    goal.lift.direction = create_vector3_stamped([0.,0.,1.], 'base_link')
    # request a vertical lift of 10cm after grasping the object
    goal.lift.desired_distance = .1
    goal.lift.min_distance = .05
    # do not use tactile-based grasping or tactile-based lift
    goal.use_reactive_execution = 0
    goal.use_reactive_lift = 0

    # send the goal
    try:
        grasper_grasp_action_client.send_goal(goal)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling %s: %e"(grasper_grasp_name,e))
        sys.exit()
    
    # wait for result
    grasper_grasp_action_client.wait_for_result(rospy.Duration(0))
    result = grasper_grasp_action_client.get_result()
    result_code_dict = {}
    for element in dir(ManipulationResult):
        if element[0].isupper():
            result_code_dict[eval('ManipulationResult.'+element)] = element
    rospy.loginfo("grasp result: %s"%result_code_dict[result.manipulation_result.value])

    '''
    object location
    '''
    rospy.loginfo("calculating put down location...")
    # remember where we picked the object up from
    if goal.target.type == GraspableObject.DATABASE_MODEL:
        # for database recognized objects, the location of the object is encapsulated in GraspableObject message
        pickup_pose = goal.target.model_pose.pose
    else:
        # for unrecognized point clouds, the location of the object is considered to be the orgin of the frame that the cluster is in
        t =get_transform(tf_listener, goal.target.cluster.header.frame_id, 'base_link')
        pickup_pose = stamp_pose(mat_to_pose(get_transform(tf_listener, goal.target.cluster.header.frame_id,'base_link')),'base_link')

    # create a place location, offset by 10cm from the pickup location
    place_pose = pickup_pose
    place_pose.header.stamp = rospy.Time.now()
    place_pose.pose.position.x += .1

    '''
    object place
    '''
    rospy.loginfo("putting down the object...")
    # call place goal
    place_goal = PlaceGoal()
    # place at the prepared location
    place_goal.place_pose = place_pose
    # the collision names of both the objects and the table are the same as in the pickup action
    place_goal.collision_object_name = goal.collision_object_name
    place_goal.collision_support_surface_name = goal.collision_support_surface_name
    # information about which grasp was executed on the object returned by the pickup action
    place_goal.grasp = result.grasp
    # use the right arm to place
    place_goal.arm_name = goal.arm_name
    # padding used when determining if the requested place location would bring the object in collision with the environment
    place_goal.place_padding = .02
    # how much the gripper should retreat after placing the object
    place_goal.desired_retreat_distance = .1
    place_goal.min_retreat_distance = .05
    # we will be putting down the object along the vertical direction which is along the z axis in the base_link frame
    place_goal.approach = GripperTranslation()
    place_goal.approach.direction = create_vector3_stamped([0.,0.,-1.],'base_link')
    # request a vertical put down motion of 10cm before placing the object
    place_goal.approach.desired_distance = .1
    place_goal.approach.min_distance = .05
    # we are not using tactile based placing
    place_goal.use_reactive_place = False
    
    # send the goal
    try:
        grasper_place_action_client.send_goal(place_goal)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling %s: %e"(grasper_place_name, e))
        sys.exit()
        
    # wait for result
    grasper_place_action_client.wait_for_result(rospy.Duration(0))
    
    # return result
    place_res = grasper_place_action_client.get_result()
    rospy.loginfo("place result: %s"%result_code_dict[place_res.manipulation_result.value])

