'''ROS client functions to use the Nano17 sensors (sample code at the bottom)'''

import roslib
roslib.load_manifest('Nano17interface')
import rospy
from Nano17interface.srv import *
from std_srvs.srv import *
import sys
import time

#call a ROS service 'servicename' with service message 'type' 
#and request arguments 'args', return the response (None if error)
def call_ROS_service(servicename, type = Empty, args = []):
    rospy.wait_for_service(servicename)
    try:
        s = rospy.ServiceProxy(servicename, type)
        resp = s(*args)
    except rospy.ServiceException, e:
        print "error in %s: %s"%(servicename, e)
        sys.exit(1)
    return resp


#get the raw nano17 values (int32[] data) for sensornum
def call_get_nano17_values(sensornum):
    resp = call_ROS_service('get_nano17_values', Values, [sensornum])
    if resp != None:
        return resp.values
    return None

#get the loc and force of contact for sensornum
def call_get_nano17_loc_normal_and_force(sensornum):
    resp = call_ROS_service('get_nano17_loc_normal_and_force', LocNormalAndForce, [sensornum])
    if resp != None:
        return (resp.loc, resp.normal, resp.forcemag)
    return (None, None, None)

#get locs and forces from all sensors
def get_all_nano17_locs_normals_and_forces(sensorcount):
    locs = []
    normals = []
    forcemags = []
    for sensornum in range(sensorcount):
        (loc, normal, forcemag) = call_get_nano17_loc_normal_and_force(sensornum)
        locs.append(loc)
        normals.append(normal)
        forcemags.append(forcemag)
    return (locs, normals, forcemags)


#sample use of above functions
if __name__ == '__main__':

    nano17count = 3

    while 1:
        (locs, normals, forcemags) = get_all_nano17_locs_normals_and_forces(nano17count)
        for sensornum in range(nano17count):
        #for sensornum in [0]:
            print "sensornum:", sensornum
            #values = call_get_nano17_values(sensornum)
            #print "values:", values
            print "loc: %.3f %.3f %.3f"%locs[sensornum], "normal: %.3f %.3f %.3f"%normals[sensornum], "forcemag: %.3f"%forcemags[sensornum]
            time.sleep(.1)



