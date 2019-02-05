import roslib; roslib.load_manifest('darrt_actions')
import rospy
import actionlib
import darrt_actions.msg
import arm_navigation_msgs.srv

class Table:
    def __init__(self, pose, x_min, x_max, y_min, y_max):
        self.pose = pose
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max

def get_table():

    scene_client = rospy.ServiceProxy
    ("/environment_server/get_planning_scene", 
     arm_navigation_msgs.srv.GetPlanningScene);

    scene = scene_client.call();
    objects = scene.collision_objects;
    ctable = None
    for o in objects:
        if o.id == "table":
            ctable = o
            break
    if not ctable:
        rospy.logerr("Unable to find table in collision map.")
        return None

    if ctable.shapes.size() != 1 or\
            ctable.shapes[0].type != ctable.shapes[0].BOX:
        rospy.logerr("Table must be a box")
        return None
    tpose = geometry_msgs.msg.PoseStamped()
    tpose.header.frame_id = ctable.header.frame_id
    tpose.pose = ctable.poses[0]
    return Table(tpose, -1.0*ctable.shapes[0].dimensions[0]/2.0,
                 ctable.shapes[0].dimensions[0]/2.0,
                 -1.0*ctable.shapes[0].dimensions[1]/2.0,
                 ctable.shapes[0].dimensions[1]/2.0)

def get_goal_points():
    for i in range(100):
        #and this dies because ti's a point cloud2

def main():
    client = actionlib.SimpleActionClient("darrt_planning/push_plate_action",
                                          darrt_actions.msg.PushPlateAction)
    goal = darrt_actions.msg.PushPlateGoal()
    goal.plate_name = "plate"
    goal.surface_name = "table"
    client.send_goal_and_wait(goal)

if __name__ == '__main__':
    rospy.init_node('blah')
    main()
