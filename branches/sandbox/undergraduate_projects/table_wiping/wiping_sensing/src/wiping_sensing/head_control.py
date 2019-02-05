import roslib; roslib.load_manifest('wiping_sensing')
import rospy
import pr2_pick_and_place_demos.pick_and_place_manager
import geometry_msgs.msg
import wiping_utils.conversions
import scipy
import visualization_msgs.msg
import object_manipulator.convert_functions

TABLE_MIDDLE_X = 0.4 #keep this at least 0.3 so it can't see its hand
TABLE_HEIGHT = 0.7
TABLE_INCR = 0.2
TABLE_START_Y = 0.45
TABLE_TOL = 0.2

vizpub = rospy.Publisher('/table_left_edge', visualization_msgs.msg.Marker)

class Table:
    def __init__(self, pose_stamped, xmin, xmax, ymin, ymax, height):
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax
        self.height = height
        self.pose_stamped = pose_stamped
    
    def couldMatch(self, other):
        if other.height > self.height + TABLE_TOL or\
                other.height < self.height - TABLE_TOL or\
                other.xmin < self.xmin - TABLE_TOL or\
                other.xmin > self.xmin + TABLE_TOL or\
                other.xmax < self.xmax - TABLE_TOL or\
                other.xmax > self.xmax + TABLE_TOL:
            return False
        return True


class LookObservation:
    def __init__(self, things, table):
        self.things = things
        self.table = table

class HeadController:
    def __init__(self, manager=None):
        self.manager = manager
        if not self.manager:
            self.manager = pr2_pick_and_place_demos.pick_and_place_manager.\
                PickAndPlaceManager()

    def lookAt(self, point_stamped):
        self.manager.move_arm_to_side(0)
        self.manager.move_arm_to_side(1)
        r = self.manager.point_head\
            (wiping_utils.conversions.pointToList(point_stamped.point),\
                 point_stamped.header.frame_id)
        if r == 0:
            return -1
        return self.look()

    def look(self):
        (things, table) = self.manager.call_tabletop_detection\
            (take_static_collision_map=1, update_table=1,\
                 clear_attached_objects=1)
        if not table:
            return None
        table = self.tableDimensions(table)
        return LookObservation(things, table)

    def tableDimensions(self, table):
        #copied from the update_table_info
        base_link_pose_stamped = \
            object_manipulator.convert_functions.change_pose_stamped_frame\
            (self.manager.tf_listener, table.pose, 'base_link')
        table_mat = object_manipulator.convert_functions.pose_to_mat\
            (base_link_pose_stamped.pose)
        corners = scipy.matrix([[table.x_min, table.y_min,0,1],
                                [table.x_min, table.y_max,0,1],
                                [table.x_max, table.y_min,0,1],
                                [table.x_max, table.y_max,0,1]]).T
        transformed_corners = table_mat * corners
        #assume the table is still a rectangle...
        #if this is a problem we may need to make an effort
        #to align with the table
        xmin = transformed_corners[0,:].min()
        xmax = transformed_corners[0,:].max()
        ymin = transformed_corners[1,:].min()
        ymax = transformed_corners[1,:].max()
        height = transformed_corners[2,:].max()
        return Table(table.pose, xmin, xmax, ymin, ymax, height)
    
    def findTableLeftEdge(self):
        point = geometry_msgs.msg.PointStamped()
        point.header.stamp = rospy.Time(0)
        point.header.frame_id = 'base_link'
        point.point.x = TABLE_MIDDLE_X
        point.point.y = TABLE_START_Y
        point.point.z = TABLE_HEIGHT
        last_obs = self.lookAt(point)
        if not last_obs:
            obs = last_obs
            while obs == None:
                point.point.y -= TABLE_INCR
                obs = self.lookAt(point)
            if obs != -1:
                return (obs.table.ymax, obs.table)
            #never found a table
            #try going the opposite direction
            point.point.y = TABLE_START_Y
        obs = last_obs
        while obs != None and obs != -1 and\
                last_obs.table.couldMatch(obs.table):
            last_obs = obs
            point.point.y += TABLE_INCR
            obs = self.lookAt(point)
        if last_obs != None and last_obs != -1:
            return (last_obs.table.ymax, last_obs.table)
        #never found a table at all
        return None

if __name__ == '__main__':
    rospy.init_node('head_controller_test_node')
    c = HeadController()
    leftinfo = c.findTableLeftEdge()
    if not leftinfo:
        rospy.logerr('Unable to find table')
    (left, table) = leftinfo
    rospy.loginfo('Left edge of table is at %f.  Table is: %s',
                  left, str(table))
    marker = visualization_msgs.msg.Marker()
    marker.header.stamp = rospy.Time(0)
    marker.header.frame_id = 'base_link'
    marker.ns = 'table_left_edge'
    marker.id = 0
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.position.x = 0.5*(table.xmin + table.xmax)
    marker.pose.position.y = left
    marker.pose.position.z = table.height
    marker.pose.orientation.w = 1
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 0
    marker.color.g = 1
    marker.color.b = 1
    marker.color.a = 1
    vizpub.publish(marker)

