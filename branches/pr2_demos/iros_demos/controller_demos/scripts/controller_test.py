import roslib; roslib.load_manifest('table_wiping_demo')
import pr2_utils.arm_control
import demo_tools
import rospy

PT_X = 0.6
PT_Y = 0.2
PT_Z = 0.0
PT_OX = -0.5
PT_OY = 0.5
PT_OZ = 0.5
PT_OW = 0.5

def run_test(manager=None):
    arm = pr2_utils.arm_control.ArmControl('left_arm')
    zforce = -1.0
    arm.add_trajectory_point_to_force_control\
        (PT_X, PT_Y, PT_Z+0.3, PT_OX, PT_OY, PT_OZ, PT_OW,\
             1000, 1000, 1000, 30, 30, 30,\
             False, False, False, False, False, False,\
             4, frame_id='/torso_lift_link')
    arm.add_trajectory_point_to_force_control\
        (PT_X, PT_Y+0.1, PT_Z, PT_OX, PT_OY, PT_OZ, PT_OW,\
             1000, 1000, 1000, 30, 30, 30,\
             False, False, False, False, False, False,\
             4, frame_id='/torso_lift_link')
    arm.executeForceControl()

    print 'Entering loop'
    while not rospy.is_shutdown():
        arm.add_trajectory_point_to_force_control\
            (PT_X, PT_Y, PT_Z, PT_OX, PT_OY, PT_OZ, PT_OW,\
                 1000, 1000, 1000, 30, 30, 30,\
                 False, False, False, False, False, False,\
                 4, frame_id='/torso_lift_link')
        arm.executeForceControl(wait=False)
        print 'Sent goal'
        #rospy.sleep(0.25)
        #arm.stop_in_place()
 
if __name__ == '__main__':
    rospy.init_node('ee_controller_test_node')
    run_test()
