import roslib; roslib.load_manifest('table_wiping_demo')
import pr2_utils.arm_control
import demo_tools
import rospy

PT_X = 0.7
PT_Y = 0
PT_Z = 0
PT_OX = 0
PT_OY = 0
PT_OZ = 0
PT_OW = 1

def run_demo(manager=None):
    arm = pr2_utils.arm_control.ArmControl('right_arm', manager=manager)
    otherarm = pr2_utils.arm_control.ArmControl('left_arm', 
                                                manager=arm.manager)
    otherarm.move_arm_to_side()
    arm.add_trajectory_point_to_force_control\
        (PT_X, PT_Y, PT_Z, PT_OX, PT_OY, PT_OZ, PT_OW,\
             1000, 1000, 1000, 30, 30, 30,\
             False, False, False, False, False, False,\
             4, frame_id='/torso_lift_link')
    demo_tools.pause('Ready to show Cartesian control?')
    arm.executeForceControl()
    arm.add_trajectory_point_to_force_control\
        (PT_X, PT_Y+0.3, PT_Z, PT_OX, PT_OY, PT_OZ, PT_OW,\
             1000, 50, 1000, 30, 30, 30,\
             False, False, False, False, False, False,\
             4, frame_id='/torso_lift_link')
    demo_tools.pause('Ready to show low impedance control?')
    arm.executeForceControl()
    arm.add_trajectory_point_to_force_control\
        (PT_X, PT_Y+0.3, PT_Z, PT_OX, PT_OY, PT_OZ, PT_OW,\
             1000, -5, 1000, 30, 30, 30,\
             False, True, False, False, False, False,\
             2, frame_id='/torso_lift_link')
    arm.add_trajectory_point_to_force_control\
        (PT_X, PT_Y+0.3, PT_Z, PT_OX, PT_OY, PT_OZ, PT_OW,\
             1000, -10, 1000, 30, 30, 30,\
             False, True, False, False, False, False,\
             4, frame_id='/torso_lift_link')
    demo_tools.pause('Ready to show force control?')
    arm.executeForceControl()

    if demo_tools.askYN('Return arms to side?'):
        arm.move_arm_to_side()
        otherarm.move_arm_to_side()

if __name__ == '__main__':
    rospy.init_node('controller_demos_test_node')
    run_demo()
