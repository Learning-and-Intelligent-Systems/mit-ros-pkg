import roslib
roslib.load_manifest('bakebot')
import rospy
from bakebot.srv import *
import tf
import actionlib
import math
import sys
import time
from geometry_msgs.msg import Twist

class BaseClient:
    
    base_client_instance = None
    #MAX_LINEAR_VELOCITY = 0.25
    MAX_LINEAR_VELOCITY = 0.10

    @staticmethod
    def get_base_client():
        if BaseClient.base_client_instance == None:
            rospy.loginfo('instantiating a new base clinet')
            BaseClient.base_client_instance = BaseClient(tf.TransformListener())
        else:
            rospy.loginfo('returning existing instance of base clint ' + str(BaseClient.base_client_instance))
        return BaseClient.base_client_instance

    def __init__(self, transformListener):
        self.tf_listener = transformListener
        rospy.loginfo('burning off a bad tf reading and waiting')
        self.get_transform()
        self.get_transform()
        self.get_transform()
        self.pub = rospy.Publisher('/base_controller/command', Twist)

    def get_transform(self):
        try:
            self.tf_listener.waitForTransform('odom_combined', 'base_footprint', rospy.Time(0), rospy.Time(5.0))
            (trans, rot) = self.tf_listener.lookupTransform('odom_combined', 'base_footprint', rospy.Time(0))
            record = '\ntrans: ' + str(trans) + '\nrot: ' + str(rot) + '\n'
            #print record
            return (trans, rot)
        except (tf.LookupException, tf.ConnectivityException) as e:
            rospy.loginfo('tf is all messed up in the base client')
            #raise e
            print e

    def go_to_pos(self, x, y, x_first = True):
        rospy.loginfo('going to x,y: ' + str(x) + str(y))
        # has to be oriented correctly
        self.tf_listener.waitForTransform('odom_combined', 'base_footprint', rospy.Time(0), rospy.Time(5.0))
        (trans, rot) = self.get_transform()
        if x_first:
            xact = trans[0]
            yact = trans[1]
            deltax = x - xact
            deltay = y - yact
            print 'delta x: ', deltax, ' delta y: ', deltay
            status = self.translate(True, deltax)
            if status:
                self.tf_listener.waitForTransform('odom_combined', 'base_footprint', rospy.Time(0), rospy.Time(5.0))
                (trans, rot) = self.get_transform()
                yact = trans[1]
                deltay = y - yact
                print 'new delta y: ', deltay
                status= self.translate(False, deltay)
        else:
            xact = trans[0]
            yact = trans[1]
            deltax = x - xact
            deltay = y - yact
            print 'delta x: ', deltax, ' delta y: ', deltay
            status = self.translate(False, deltay)
            if status:
                self.tf_listener.waitForTransform('odom_combined', 'base_footprint', rospy.Time(0), rospy.Time(5.0))
                (trans, rot) = self.get_transform()
                xact = trans[0]
                deltax = x - xact
                print 'new delta x: ', deltax
                status= self.translate(True, deltax)

        return status


    def translate(self, isXAxis, distance):
        axis = 'X' if isXAxis else 'Y'
        rospy.loginfo('moving along the ' + axis + ' axis')
        self.tf_listener.waitForTransform('odom_combined', 'base_footprint', rospy.Time(0), rospy.Time(5.0))
        (trans, rot) = self.get_transform()

        desired = distance + (trans[0] if isXAxis else trans[1])

        #k = 0.5 
        k = 0.4 
        #feed_forward = 0.03
        feed_forward = 0.05
        tolerance = 0.01
        maximum_velocity = 0.25
        rate = rospy.Rate(10.0)
        done = False

        base_cmd = Twist()

        while (not done) and (not rospy.is_shutdown()):
            try:
                (trans, rot) = self.get_transform()
            except:
                rospy.logerror('an exception was raised in the transform')
                break;
            actual = trans[0] if isXAxis else trans[1]

            error = desired - actual
            if abs(error) < tolerance:
                rospy.loginfo('finished with an error of: ' + str(error))
                done = True
                break
            else:
                sign = 1 if error > 0 else -1
                command = k * error + sign * feed_forward

                if abs(command) > maximum_velocity:
                    sign = 1 if error > 0 else -1
                    command = sign * abs(maximum_velocity)
                rospy.loginfo('des: ' + str(desired) + '\t\tact: ' + str(actual) + '\t\terr: ' + str(error) + '\t\tcmd: ' + str(command))

                base_cmd.linear.x = command if isXAxis else 0
                base_cmd.linear.y = 0 if isXAxis else command
                base_cmd.angular.z = 0
                self.pub.publish(base_cmd);
                rate.sleep()

        time.sleep(1)
        rospy.loginfo('sending a zero command')
        base_cmd.linear.x = 0
        base_cmd.linear.y = 0
        base_cmd.angular.z = 0
        self.pub.publish(base_cmd);
        rospy.loginfo('done')
        return done

    def fake_rotate(self, theta):
        print 'fake rotating'
        self.tf_listener.waitForTransform('odom_combined', 'base_footprint', rospy.Time(0), rospy.Time(5.0))
        (trans, rot) = self.get_transform()
        print rot
        sign = 1 if theta >= 0 else -1
        base_cmd = Twist()
        base_cmd.linear.x = 0
        base_cmd.linear.y = 0 
        base_cmd.angular.z = .25 * sign
        r = rospy.Rate(10)
        for i in range(1,9):
            print 'pub'
            self.pub.publish(base_cmd);
            r.sleep()
        rospy.loginfo('sending a zero command')
        base_cmd.linear.x = 0
        base_cmd.linear.y = 0
        base_cmd.angular.z = 0
        self.pub.publish(base_cmd);
        rospy.loginfo('done')
        return True

if __name__ == '__main__':
    rospy.init_node('bakebot_base_client')
    client = BaseClient.get_base_client()
    client.translate(True, -.1)
    time.sleep(1)
    client.translate(False, .2)
    time.sleep(1)
    client.translate(True, .1)
    time.sleep(1)
    client.translate(False, -.2)
    time.sleep(1)
    client.translate(False, .2)
    time.sleep(1)
    client.translate(False, -.2)
    time.sleep(1)
    client.translate(False, .2)
    time.sleep(1)
    client.translate(False, -.2)
