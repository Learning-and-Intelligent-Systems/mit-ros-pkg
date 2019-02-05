#!/usr/bin/env python
import roslib; roslib.load_manifest('demo_exec')
import rospy
import controller_demos
import table_wiping_demo.wipe_table
import demo_tools
import pr2_pick_and_place_demos.pick_and_place_manager

def main():
    manager =\
        pr2_pick_and_place_demos.pick_and_place_manager.PickAndPlaceManager()
    demo_tools.pause('Start table wiping demo?')
    table_wiping_demo.wipe_table.main(manager=manager)
    demo_tools.pause('Start controller demos?')
    controller_demos.run_demo(manager=manager)

if __name__ == '__main__':
    rospy.init_node('mit_iros_demo_node')
    main()
