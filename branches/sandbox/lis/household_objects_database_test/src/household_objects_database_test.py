#!/usr/bin/env python
import roslib
roslib.load_manifest('household_objects_database_test')
import rospy

# import messages and services
from object_manipulation_msgs.srv import GraspPlanning
from household_objects_database_msgs.srv import GetModelList, GetModelMesh, GetModelDescription

# import utilities
from draw_functions import *

def pause():
    print 'press ENTER to continue...'
    raw_input()

class HODBTest:
    def __init__(self):
        database_grasp_planning_srv_name = '/objects_database_node/database_grasp_planning'
        get_model_description_srv_name = '/objects_database_node/get_model_description'
        get_model_list_srv_name = '/objects_database_node/get_model_list'
        get_model_mesh_srv_name = '/objects_database_node/get_model_mesh'
        self.database_grasp_planning_srv = rospy.ServiceProxy(database_grasp_planning_srv_name, GraspPlanning)
        self.get_model_description_srv = rospy.ServiceProxy(get_model_description_srv_name, GetModelDescription)
        self.get_model_list_srv = rospy.ServiceProxy(get_model_list_srv_name, GetModelList)
        self.database_return_code = {1:'UNKNOWN_ERROR',2:'DATABASE_NOT_CONNECTED',3:'DATABASE_QUERY_ERROR',-1:'SUCCESS'}
        self.get_model_mesh_srv = rospy.ServiceProxy(get_model_mesh_srv_name, GetModelMesh)
        
if __name__ == '__main__':
    rospy.init_node('household_objects_database_test', anonymous=True)
    ht = HODBTest()
    df = DrawFunctions('household_objects_database_test')
    while not rospy.is_shutdown():
        print '--------------------------------------'
        print 'press 1 to get model list'
        print 'press 2 to get a model description'
        print 'press 3 to get all model descriptions'
        print 'press 4 to show a model in rviz'
        print 'press 5 to show all models in rviz'
        print 'press h to haha'
        print 'press q to exit'
        input = raw_input()

        if input == '1':
            result = ht.get_model_list_srv()
            print ht.database_return_code[result.return_code.code]
            print result.model_ids
        elif input =='2':
            print 'enter the model id:'
            input = raw_input()
            result = ht.get_model_description_srv(int(input))
            print ht.database_return_code[result.return_code.code]
            print result
        elif input == '3':
            model_list_result = ht.get_model_list_srv()
            model_ids = model_list_result.model_ids
            for id in model_ids:
                result = ht.get_model_description_srv(int(id))
                print ht.database_return_code[result.return_code.code]
                print result
                pause()
        elif input == '4':
            print 'enter the model id:'
            input = raw_input()
            result = ht.get_model_description_srv(int(input))
            print result
            result = ht.get_model_mesh_srv(int(input))
            print ht.database_return_code[result.return_code.code]
            df.clear_mesh()
            df.draw_mesh(None,result.mesh)
        elif input == '5':
            model_list_result = ht.get_model_list_srv()
            model_ids = model_list_result.model_ids
            for id in model_ids:
                result = ht.get_model_description_srv(int(id))
                print result
                result = ht.get_model_mesh_srv(int(id))
                print ht.database_return_code[result.return_code.code]
                df.clear_mesh()
                df.draw_mesh(None,result.mesh)
                pause()
        elif input == 'h':
            print 'haha'
            df.clear_mesh()
            result = ht.get_model_mesh_srv(18718)
            df.draw_mesh(None,result.mesh)
            '''
            m = Marker()
            m.type = Marker.TRIANGLE_LIST
            m.points = [mesh.vertices[x] for x in mesh.triangles]
            m.header.frame_id = 'base_link'
            m.header.stamp = rospy.Time.now()
            m.ns = 'mesh'
            m.action = Marker.ADD
            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 1.0
            m.color.a = .8
            m.color.r = 0
            m.color.g = 1.0
            m.color.b = 0
            m.lifetime = rospy.Duration(60.0)
            m.id = 123
            m.pose.position.x = 0
            m.pose.position.y = 0
            m.pose.position.z = 0
            m.pose.orientation.x = .5
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 0
            df.marker_pub.publish(m)
            '''
        elif input == 'q':
            break
        else:
            continue

