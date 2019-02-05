import roslib; roslib.load_manifest('tabletop_for_cabinet')
import rospy
from pr2_pick_and_place_demos.pick_and_place_manager import PickAndPlaceManager

def dude():
    ppm = PickAndPlaceManager()
    (things,table) = ppm.call_tabletop_detection(take_static_collision_map=1, update_table=1)

if __name__ == '__main__':
    print "dude"
    rospy.init_node('my_tabletop_detection_cabinet')
    dude()
