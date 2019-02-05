#!/usr/bin/env python
import roslib
roslib.load_manifest('household_objects_database_test')
import rospy

# import messages and services
from object_manipulation_msgs.srv import GraspPlanning
from household_objects_database_msgs.srv import GetModelList, GetModelMesh, GetModelDescription

# import libraries
import numpy
from numpy import *

# import utilities
from draw_functions import *

# import openrave stuff
from openravepy import *
from or_model_gen import *

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
        
class ORTest:
    def __init__(self,env):
        print 'a'

if __name__ == '__main__':
    rospy.init_node('household_objects_database_test', anonymous=True)
    ht = HODBTest()
    df = DrawFunctions('household_objects_database_test')

    env = Environment()
    env.SetViewer('qtcoin')
    ot = ORTest(env)
    handles = []
            
    while not rospy.is_shutdown():
        print '--------------------------------------'
        print 'to load or generate model xmls in openrave, please run this script under household_object_database_test/src'
        print 'press 1 to get model list'
        print 'press 2 to get a model description'
        print 'press 3 to get all model descriptions'
        print 'press 4 to show a model in rviz'
        print 'press 5 to show all models in rviz'
        print 'press 6 to generate a model xml for openrave'
        print 'press 7 to generate all model xmls for openrave'
        print 'press 8 to show a model in openrave'
        print 'press 9 to show all models in openrave'
        print 'press a to load a model xml in openrave'
        print 'press b to load all model xmls in openrave'
        print 'press h to haha'
        print 'press q to exit'
        input = raw_input()

        if input == '1':
            print 'getting model list'
            result = ht.get_model_list_srv()
            print ht.database_return_code[result.return_code.code]
            print result.model_ids
        elif input =='2':
            print 'getting one model description'
            print 'enter the model id:'
            input = raw_input()
            result = ht.get_model_description_srv(int(input))
            print ht.database_return_code[result.return_code.code]
            print result
        elif input == '3':
            print 'getting all model descriptions'
            model_list_result = ht.get_model_list_srv()
            model_ids = model_list_result.model_ids
            for id in model_ids:
                result = ht.get_model_description_srv(int(id))
                print ht.database_return_code[result.return_code.code]
                print result
                pause()
        elif input == '4':
            print 'showing one model in rviz'
            print 'enter the model id:'
            input = raw_input()
            result = ht.get_model_description_srv(int(input))
            print result
            result = ht.get_model_mesh_srv(int(input))
            print ht.database_return_code[result.return_code.code]
            df.clear_mesh()
            df.draw_mesh(None,result.mesh)
        elif input == '5':
            print 'showing all models in rviz'
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
        elif input == '6':
            print 'generating one model xml file'
            print 'enter the model id:'
            input = raw_input()
            model_id = int(input)
            result = ht.get_model_description_srv(model_id)
            print result
            result = ht.get_model_mesh_srv(model_id)
            print ht.database_return_code[result.return_code.code]
            mesh = result.mesh
            points = [mesh.vertices[x] for x in mesh.triangles]
            filename = 'models/'+str(model_id)+'.kinbody.xml'
            generate_wg_model_xml(model_id, [[p.x,p.y,p.z] for p in points],filename)
            print filename,'is generated'
        elif input == '7':
            print 'generating all model xml files'
            model_list_result = ht.get_model_list_srv()
            model_ids = model_list_result.model_ids
            for id in model_ids:
                model_id = int(id)
                result = ht.get_model_description_srv(model_id)
                print result
                result = ht.get_model_mesh_srv(model_id)
                print ht.database_return_code[result.return_code.code]
                mesh = result.mesh
                points = [mesh.vertices[x] for x in mesh.triangles]
                filename = 'models/'+str(model_id)+'.kinbody.xml'
                generate_wg_model_xml(model_id, [[p.x,p.y,p.z] for p in points],filename)
                print filename,'is generated'
        elif input == '8':
            print 'showing one model in openrave'
            print 'enter the model id:'
            input = raw_input()
            result = ht.get_model_description_srv(int(input))
            print result
            result = ht.get_model_mesh_srv(int(input))
            print ht.database_return_code[result.return_code.code]
            handles = []
            handles.append(env.drawtrimesh(points=array([[p.x,p.y,p.z] for p in result.mesh.vertices]),
                                           indices=array(result.mesh.triangles)))
        elif input == '9':
            print 'showing all models in openrave'
            model_list_result = ht.get_model_list_srv()
            model_ids = model_list_result.model_ids
            for id in model_ids:
                print 'model',id
                result = ht.get_model_description_srv(int(id))
                print result
                result = ht.get_model_mesh_srv(int(id))
                print ht.database_return_code[result.return_code.code]
                handles = []
                handles.append(env.drawtrimesh(points=array([[p.x,p.y,p.z] for p in result.mesh.vertices]),
                                               indices=array(result.mesh.triangles)))
                pause()
        elif input == 'a':
            print 'loading one model in openrave'
            print 'enter the model id:'
            input = raw_input()
            model_id = int(input)
            body = env.CreateKinBody()
            success = body.InitFromFile('models/'+input+'.kinbody.xml')
            if success:
                env.LockPhysics(True)
                env.AddKinBody(body)
                env.LockPhysics(False)
            else: print 'failed to init kinbody from '+input+'.kinbody.xml'
        elif input == 'b':
            print 'loading all models in openrave'
            model_list_result = ht.get_model_list_srv()
            model_ids = model_list_result.model_ids
            for id in model_ids:
                print 'model:',id
                model_id = int(id)
                result = ht.get_model_description_srv(model_id)
                print result
                result = ht.get_model_mesh_srv(model_id)
                print ht.database_return_code[result.return_code.code]
                body = env.CreateKinBody()
                success = body.InitFromFile('models/'+str(id)+'.kinbody.xml')
                if success:
                    env.LockPhysics(True)
                    env.AddKinBody(body)
                    env.LockPhysics(False)
                else: print 'failed to init kinbody from '+str(id)+'.kinbody.xml'
                pause()
                env.RemoveKinBody(body)
        elif input == 'h':
            print 'haha'
            df.clear_mesh()
            result = ht.get_model_mesh_srv(18781)
            #df.draw_mesh(None,result.mesh)
            
            '''
            # openrave kinbody loading test
            body = env.CreateKinBody()
            body.InitFromFile('18781.kinbody.xml')
            env.LockPhysics(True)
            env.AddKinBody(body)
            env.LockPhysics(False)
            '''

            '''
            # openrave drawing test
            mesh = result.mesh
            vertices = mesh.vertices
            triangles = mesh.triangles
            point_to_list = lambda p: [p.x,p.y,p.z]
            vertices_list = [point_to_list(v) for v in vertices]
            handles = []
            handles.append(env.drawtrimesh(points=array(vertices_list),
                                           indices=array(triangles)))
            '''

            '''
            # rviz drawing test
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

