#!/usr/bin/env python
import roslib
roslib.load_manifest('household_objects_database_test')
import rospy

# import messages and services
from object_manipulation_msgs.msg import GraspableObject
from object_manipulation_msgs.srv import GraspPlanning
from household_objects_database_msgs.srv import GetModelList, GetModelMesh, GetModelDescription
from household_objects_database_msgs.msg import DatabaseModelPose

# import libraries
import numpy
from numpy import *

# import utilities
from draw_functions import *

# import openrave stuff
from openravepy import *
from or_model_gen import *
import os

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
        
    def get_model_list(self):
        result = ht.get_model_list_srv()
        print ht.database_return_code[result.return_code.code]
        print result.model_ids
        return result
    
    def get_model_description(self,id):
        result = ht.get_model_description_srv(id)
        print id
        print ht.database_return_code[result.return_code.code]
        print result
        num_grasps = len((self.get_grasps('left_arm', id)).grasps)
        #num_grasps = len((self.get_grasps('WILLOW_GRIPPER_2010', id)).grasps)

        print '%d grasps'%num_grasps

        return result,num_grasps
    
    def load_model(self,env,model_id):
        filename = 'models/'+str(model_id)+'.kinbody.xml'
        if os.path.isfile(filename):
            body = env.ReadKinBodyXMLFile(filename)
            
            if body:
                env.LockPhysics(True)
                env.AddKinBody(body)
                env.LockPhysics(False)
            else: print 'failed to init kinbody from '+str(model_id)+'.kinbody.xml'
            return body
        else:
            print 'model xml file %s does not exist'%filename
            return None
    
    def write_model_xml(self,env,model_id):
        result = ht.get_model_mesh_srv(model_id)
        mesh = result.mesh
        points = [mesh.vertices[x] for x in mesh.triangles]
        filename = 'models/'+str(model_id)+'.kinbody.xml'
        generate_wg_model_xml(model_id, [[p.x,p.y,p.z] for p in points],filename)
        print filename,'is generated'
    
    def show_model(self,env,model_id):
        print model_id
        result = ht.get_model_mesh_srv(model_id)
        print ht.database_return_code[result.return_code.code]
        
        return env.drawtrimesh(points=array([[p.x,p.y,p.z] for p in result.mesh.vertices]),
                                    indices=array(result.mesh.triangles))    
    
    def get_grasps(self,arm_name,model_id):
        target = GraspableObject()
        modelpose = DatabaseModelPose()
        modelpose.model_id = model_id
        target.model_pose = modelpose
        return ht.database_grasp_planning_srv(arm_name=arm_name,target=target)
    
class ORTest:
    def __init__(self,env):
        print 'a'

def showGrasp(robot,Tgrasp,angle=.548,wait=True,moveback=True,showbody=False,checkcollision=False):
    """visualizes the robot configuration when robot.GetActiveManipulator().GetEndEffectorTransform()==Tgrasp
    
    :param Tgrasp: a row-major 4x4 matrix in numpy.array format
    :param angle: gripper angle
    """

    def show():
        collision = True
        v = robot.GetActiveDOFValues()
        v[robot.GetJoint('l_gripper_l_finger_joint').GetDOFIndex()] = angle
        robot.SetActiveDOFValues(v)
        O_T_R = robot.GetTransform() # robot transform R in global frame O 
        #O_T_G = robot.GetActiveManipulator().GetEndEffectorTransform() # grasping frame G in global frame O
        O_T_G = robot.GetLink('l_wrist_roll_link').GetTransform()
        G_T_O = linalg.inv(O_T_G) # global frame O in grasping frame G
        G_T_R = dot(G_T_O, O_T_R) # robot frame R in grasping frame G
        O_T_G_goal = Tgrasp # final grasping frame G_goal in global frame O 
        O_T_R_goal = dot(O_T_G_goal,G_T_R) # final robot transform R_goal in global frame O                
        robot.SetTransform(O_T_R_goal)
        if checkcollision:
            report = CollisionReport()
            robot.GetEnv().CheckCollision(robot, report)
            if len(report.contacts)==0:
                collision = False
                #print 'not in collision!'
        if wait:
            pause()
        if moveback:
            robot.SetTransform(O_T_R)
        if checkcollision:
            return collision
    
    if not showbody:
        with GripperVisibility(robot.GetActiveManipulator()):
            return show()
    else:
        return show()

class GripperVisibility:
    """When 'entered' will hide all the non-gripper links in order to facilitate visiblity of the gripper"""
    def __init__(self,manip):
        self.manip = manip
        self.robot = self.manip.GetRobot()
        self.hiddengeoms = []
    def __enter__(self):
        self.hiddengeoms = []
        with self.robot.GetEnv():
            # stop rendering the non-gripper links
            childlinkids = [link.GetIndex() for link in self.manip.GetChildLinks()]
            for link in self.robot.GetLinks():
                if link.GetIndex() not in childlinkids:
                    for geom in link.GetGeometries():
                        self.hiddengeoms.append((geom,geom.IsDraw()))
                        geom.SetDraw(False)
                    link.Enable(False) # disable collision checking
                else:
                    for geom in link.GetGeometries():
                        geom.SetTransparency(.9)
    def __exit__(self,type,value,traceback):
        with self.robot.GetEnv():
            for geom,isdraw in self.hiddengeoms:
                geom.SetDraw(isdraw)
            childlinkids = [link.GetIndex() for link in self.manip.GetChildLinks()]
            for link in self.robot.GetLinks():
                if link.GetIndex() not in childlinkids:
                    link.Enable(True)

if __name__ == '__main__':
    rospy.init_node('household_objects_database_test', anonymous=True)
    ht = HODBTest()
    df = DrawFunctions('household_objects_database_test')

    try:
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
                ht.get_model_list()
            elif input =='2':
                print 'getting one model description'
                print 'enter the model id:'
                input = raw_input()
                model_id = int(input)
                ht.get_model_description(model_id)
            elif input == '3':
                print 'getting all model descriptions'
                model_list_result = ht.get_model_list()
                model_ids = model_list_result.model_ids
                for id in model_ids:
                    ht.get_model_description(int(id))
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
                ht.write_model_xml(env, model_id)
                
            elif input == '7':
                print 'generating all model xml files'
                model_list_result = ht.get_model_list_srv()
                model_ids = model_list_result.model_ids
                for id in model_ids:
                    ht.write_model_xml(env, int(id))
                    
            elif input == '8':
                print 'showing one model in openrave'
                print 'enter the model id:'
                input = raw_input()
                model_id = int(input)
                handle = ht.show_model(env, model_id)
                pause()
                handle = None
            elif input == '9':
                print 'showing all models in openrave'
                model_list_result = ht.get_model_list_srv()
                model_ids = model_list_result.model_ids
                for id in model_ids:
                    print 'model',id
                    ht.get_model_description(int(id))
                    handle = ht.show_model(env, int(id))
                    pause()
                    handle = None
            elif input == 'a':
                print 'loading one model in openrave'
                print 'enter the model id:'
                input = raw_input()
                model_id = int(input)
                body = ht.load_model(env, model_id)
                pause()
                env.Remove(body)
            elif input == 'b':
                print 'loading all models in openrave'
                model_list_result = ht.get_model_list_srv()
                model_ids = model_list_result.model_ids
                for id in model_ids:
                    print 'model:',id
                    model_id = int(id)
                    body = ht.load_model(env, model_id)
                    pause()
                    env.Remove(body)
            elif input == 'h':
                print 'haha'
                
                
                model_list_result = ht.get_model_list()
                model_ids = model_list_result.model_ids
                models_with_grasps = []
                for id in model_ids:
                    result,num_grasps=ht.get_model_description(int(id))
                    if num_grasps!=0:
                        models_with_grasps.append(id)
                    
                print models_with_grasps
                pause()
                #models_with_grasps=[18798, 18675, 18683, 18668, 18669, 18670, 18666, 18694, 18671, 18667, 18672, 18673, 18674, 18686, 18676, 18677, 18687, 18678, 18695, 18680, 18688, 18681, 18682, 18700, 18665, 18685, 18689, 18696, 18690, 18692, 18697, 18698, 18684, 18802, 18746, 18766, 18783, 18765, 18791, 18799, 18693, 18744, 18807, 18808, 18699, 18797, 18691]
#
#                for id in models_with_grasps:
#                    handle =ht.show_model(env, id)
#                    pause()
#                    handle=None
                
                # init robot
                robot = env.ReadRobotXMLFile('robots/pr2mit.robot.xml')
                env.AddRobot(robot)
                # init robot pose: l_shoulder_pan, r_shoulder_pan, torso, l_gripper
                v = robot.GetActiveDOFValues()
                v[robot.GetJoint('l_shoulder_pan_joint').GetDOFIndex()]= 3.14/2
                v[robot.GetJoint('r_shoulder_pan_joint').GetDOFIndex()] = -3.14/2
                v[robot.GetJoint('torso_lift_joint').GetDOFIndex()] = 0
                v[robot.GetJoint('l_gripper_l_finger_joint').GetDOFIndex()] = .548
                robot.SetActiveDOFValues(v)
                robot.SetActiveManipulator('leftarm')
                
                for model_id in models_with_grasps:
                    print 'model_id %d'%(model_id)
                    body = ht.load_model(env, model_id)
                    
                    result = ht.get_grasps(arm_name='left_arm', model_id=model_id)
                    #result = ht.get_grasps(arm_name='WILLOW_GRIPPER_2010', model_id=model_id)
                    grasps = result.grasps
                    
                    print 'loaded %d grasps'%(len(grasps))
                    i=0
                    for grasp in grasps:
                        i+=1
                        print 'showing grasp %d/%d'%(i,len(grasps))
                        Tgrasp = matrixFromQuat(array([grasp.grasp_pose.orientation.w,
                                                       grasp.grasp_pose.orientation.x,
                                                       grasp.grasp_pose.orientation.y,
                                                       grasp.grasp_pose.orientation.z]))
                        Tgrasp[:3,3] = [grasp.grasp_pose.position.x,grasp.grasp_pose.position.y,grasp.grasp_pose.position.z]
                        showGrasp(robot,Tgrasp,grasp.grasp_posture.position[0],wait=True)
                        break
                        
                    env.Remove(body) 
                
                '''
                #df.clear_mesh()
                #result = ht.get_model_mesh_srv(18781)
                #df.draw_mesh(None,result.mesh)

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
                env.Destroy()
                RaveDestroy()
                break
            else:
                continue
    except openrave_exception, e:
        print e
    finally:
        # destroy planning environment and clean up
        pass
        #env.Destroy()
        #RaveDestroy()

