#!/usr/bin/env python
import roslib; roslib.load_manifest('pressure_practice')
import rospy
import tabletop_object_detector.srv
import tf
import time

class table_detector:
    def __init__(self, tf_transformListener):
	self.tf_transformListener = tf_transformListener

    def get_table(self):
        detection_call = \
                tabletop_object_detector.srv.TabletopDetectionRequest()

        rospy.wait_for_service('/object_detection')
        object_detection_srv = rospy.ServiceProxy\
                ('/object_detection',\
                     tabletop_object_detector.srv.TabletopDetection)
        detection_response = object_detection_srv(detection_call)
        if not detection_response:
                table_detection = None
                return table_detection
        if detection_response.detection.result != \
                    detection_response.detection.SUCCESS:
                return None
        table = detection_response.detection.table
        x = (table.x_min + table.x_max)/2.0
        y = (table.y_min + table.y_max)/2.0
        z = table.pose.pose.position.z
        self.tf_transformListener.waitForTransform(table.pose.header.frame_id,"/torso_lift_link",     rospy.rostime.get_rostime(), rospy.Duration(1.0))
        transformed = self.tf_transformListener.transformPose('torso_lift_link', table.pose)
        return ((x, y, transformed.pose.position.z),(table.y_min, table.y_max), (table.x_min, table.x_max))

if __name__ == "__main__":
    	tf_listener = tf.TransformListener()
        table_detector = table_detector(tf_listener)
        print table_detector.get_table()
