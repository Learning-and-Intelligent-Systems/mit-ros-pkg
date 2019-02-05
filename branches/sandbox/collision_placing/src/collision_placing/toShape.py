import roslib; 
roslib.load_manifest('collision_placing')
import rospy
import sys
from arm_navigation_msgs.msg import AttachedCollisionObject, CollisionObject, Shape
from geometry_msgs.msg import Point, Pose

def get_obj():

    attached_obj = AttachedCollisionObject()
    obj = CollisionObject()
    shape = Shape()
    pose = Pose()
    vert = Point()
    verts = []
    
    in_verts = True
    in_position = True
    
    f = open('test.txt', 'r')
    for line in f:
        fields = line.split(':')
        fields = [fields[i].strip() for i in range(len(fields))]
#        print fields
        if fields[0] == "frame_id":
            obj.header.frame_id = fields[1]
        elif fields[0] == "id":
            obj.id = fields[1]
        elif fields[0] == "operation" and fields[1] != "":
            obj.operation.operation = int(fields[1])
        elif fields[0] == "type":
            shape.type = int(fields[1])
        elif fields[0] == "triangles":
            array = fields[1][1:-2]
            ind = array.split(',')
            inds = [int(i) for i in ind]
            shape.triangles = inds
        elif fields[0] == "x" and in_verts:
            vert = Point()
            vert.x = float(fields[1])
        elif fields[0] == "y" and in_verts:
            vert.y = float(fields[1])
        elif fields[0] == "z" and in_verts:
            vert.z = float(fields[1])
            verts.append(vert)
        elif fields[0] == "poses":
            in_verts = False
        elif fields[0] == "x" and in_position:
            pose.position.x = float(fields[1])
        elif fields[0] == "y" and in_position:
            pose.position.y = float(fields[1])
        elif fields[0] == "z" and in_position:
            pose.position.z = float(fields[1])
            in_position = False
        elif fields[0] == "x" and not in_position:
            pose.orientation.x = float(fields[1])
        elif fields[0] == "y" and not in_position:
            pose.orientation.y = float(fields[1])
        elif fields[0] == "z" and not in_position:
            pose.orientation.z = float(fields[1])
        elif fields[0] == "w":
            pose.orientation.w = float(fields[1])
        
        obj.id = "graspable_object_1001"
        shape.vertices = verts
        obj.shapes = [shape]
        obj.poses = [pose]

    attached_obj.object = obj
    return attached_obj
        
#print get_obj()
