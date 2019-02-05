import roslib; roslib.load_manifest('table_wiping_demo')
import following_arm
import wiping_arm
import rospy
import threading
import geometry_msgs.msg
import pr2_pick_and_place_demos.pick_and_place_manager
import actionlib_msgs.msg
import random
import pr2_utils.arm_control
import demo_tools

import arm_navigation_msgs.msg

#box details
BOX_X=0.42
BOX_Y=0.36
BOX_Z=0.08
TABLE_WIDTH=0.03

def addBox(manager, arm_control):
    hand_pose = arm_control.get_hand_pose(frame_id='/base_link')
    pose = geometry_msgs.msg.Pose()
    pose.position.x = hand_pose.pose.position.x - BOX_X/2.0
    pose.position.y = hand_pose.pose.position.y
    pose.position.z = hand_pose.pose.position.z - BOX_Z/2.0
    pose.orientation.y = -0.2
    pose.orientation.w = 0.98
    dims = (BOX_X, BOX_Y, BOX_Z)
    manager.collision_map_interface.add_collision_box(pose,
                                                      dims,
                                                      '/base_link',
                                                      'wiper_collection_box')
    manager.collision_map_interface.add_collision_box(pose,
                                                      dims,
                                                      '/base_link',
                                                      'wiper_collection_box')
    rospy.sleep(2.0)
    manager.attach_object(0, 'wiper_collection_box')
    manager.attach_object(0, 'wiper_collection_box')

def addTable(manager, initial_table):
    pose = geometry_msgs.msg.Pose()
    pose.position.x = 0.5*(initial_table.xmin + initial_table.xmax)
    pose.position.y = initial_table.ymax - 1.0
    pose.position.z = initial_table.height - TABLE_WIDTH/2.0
    pose.orientation.w = 1.0
    dims = (initial_table.xmax - initial_table.xmin, 2.0, TABLE_WIDTH)
    manager.collision_map_interface.add_collision_box(pose,
                                                      dims,
                                                      '/base_link',
                                                      'real_table')
    manager.collision_map_interface.add_collision_box(pose,
                                                      dims,
                                                      '/base_link',
                                                      'real_table')

def getHumanApprovedTable(manager, leftarm, rightarm):
    human_approved_table = False
    while not human_approved_table and\
            not rospy.is_shutdown():
        manager.remove_object('real_table')
        manager.remove_object('real_table')
        if not leftarm.findTableEdge():
            demo_tools.pause\
                ('Unable to find table.  Re-position robot and table.')
            continue
        addTable(manager, leftarm.table)
        human_approved_table =\
            demo_tools.askYN('Using table displayed.  Is this the right table?')
    addBox(manager, rightarm.following_arm_control)
    

def main(manager=None):
    if not manager:
        manager = pr2_pick_and_place_demos.pick_and_place_manager.\
            PickAndPlaceManager()
    leftarm = wiping_arm.ArmWiper('left_arm', manager=manager)
    rightarm = following_arm.HandTracker('right_arm', 'left_arm', 
                                         leftarm.strip_topic_name,
                                         offset=-1.0*BOX_X/2.0-0.05,
                                         manager=manager)

    if leftarm.edge == None:
        getHumanApprovedTable(manager, leftarm, rightarm)
    else:
        addTable(manager, leftarm.table)
        addBox(manager, rightarm.following_arm_control)
        if not demo_tools.askYN\
                ('Using table displayed.  Is this the right table?'):
            getHumanApprovedTable(manager, leftarm, rightarm)

    move_succeeded = False

    use_right_arm = True
    while not move_succeeded and use_right_arm and\
            not rospy.is_shutdown():
        ntries = 0
        pose_stamped = geometry_msgs.msg.PoseStamped()
        origx = leftarm.table.xmin - 0.1
        origy = leftarm.edge - 0.1 - BOX_X/2.0 - 0.05
        origz = leftarm.table.height
        pose_stamped.header.frame_id = '/base_link'
        pose_stamped.pose.position.x = origx
        pose_stamped.pose.position.y = origy
        pose_stamped.pose.position.z = origz
        pose_stamped.pose.orientation.x = 0.1
        pose_stamped.pose.orientation.y = 0.7
        pose_stamped.pose.orientation.z = 0.1
        pose_stamped.pose.orientation.w = 0.7

        while not move_succeeded and ntries < 30 and\
                not rospy.is_shutdown():
            move_succeeded = True
            (ma_r, ma_s) =\
                rightarm.following_arm_control.move_arm_collision_free\
                (pose_stamped, bounds=(0.1,0.01,0.2,0.1,0.1,0.1))
            if ma_r.error_code.val != ma_r.error_code.SUCCESS or\
                    ma_s != actionlib_msgs.msg.GoalStatus.SUCCEEDED:
                rospy.logwarn('Unable to move right arm to initial position %s.  Error code was %d and status was %d', str(pose_stamped),
                              ma_r.error_code.val, ma_s)
                pose_stamped.pose.position.x =\
                    origx + random.uniform(-0.05, 0.05)
                pose_stamped.pose.position.z =\
                    origz + random.uniform(-0.2, 0.1)
                if pose_stamped.pose.position.y > origy - 0.15 and\
                        (ntries%5 == 0):
                    pose_stamped.pose.position.y -= 0.05
                move_succeeded = False
            ntries += 1
        if not move_succeeded:
            use_right_arm = demo_tools.askYN\
                ('Unable to follow using right hand.  Try again?') 
            if use_right_arm:
                if demo_tools.askYN\
                        ('Does the table detection need to be redone?'):
                    getHumanApprovedTable(manager, leftarm, rightarm)
    #pose_stamped.pose.position.x = origx
    #pose_stamped.pose.position.z = origz
    rightarm.setInitialPose(pose_stamped)
    demo_tools.pause('Ready to start demo?')

    leftthread = threading.Thread(target=leftarm.wipeSector, 
                                  args=(0, leftarm.edge-0.1, False),
                                  kwargs={'max_wipes':4})
    leftthread.start()
    rospy.loginfo('Left arm wiping table')
    looprate = rospy.Rate(10)
    left_pose = leftarm.arm_control.get_hand_pose()
    while not rospy.is_shutdown() and\
            left_pose.pose.position.y > leftarm.edge - 0.09:
        looprate.sleep()
        left_pose = leftarm.arm_control.get_hand_pose()
    if rospy.is_shutdown():
        leftthread.join()
        return
    #rightarm.track(pose_stamped, 1, offset=-0.25)
    if use_right_arm:
        rightthread = threading.Thread\
            (target=rightarm.track,\
                 kwargs={'jiggle':(0.05, 0, 0.05, 0,0,0,0),\
                             'max_lag_plus': BOX_X/2.0-0.08})
        
        rightthread.start()
    rospy.loginfo('Right arm tracking left arm')
    looprate = rospy.Rate(4)
    while not rospy.is_shutdown() and leftthread.isAlive():
        looprate.sleep()
    leftthread.join()
    if use_right_arm:
        rightarm.stopTracking()
        rightthread.join()
    return

if __name__ == '__main__':
    rospy.init_node('table_wiping_node')
    main()
