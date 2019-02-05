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

import motion_planning_msgs.msg

#box details
BOX_X=0.42
BOX_Y=0.36
BOX_Z=0.08
TABLE_WIDTH=0.03


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
    

def main(manager=None):
    if not manager:
        manager = pr2_pick_and_place_demos.pick_and_place_manager.\
            PickAndPlaceManager()
    leftarm = wiping_arm.ArmWiper('left_arm', manager=manager)
    addTable(manager, leftarm.table)
    while not rospy.is_shutdown():
        leftarm.wipeSector(0, leftarm.edge-0.1, False)

if __name__ == '__main__':
    rospy.init_node('table_wiping_node')
    main()
