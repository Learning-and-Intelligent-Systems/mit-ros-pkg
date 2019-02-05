import objModels as om
import DrawingWindowStandalone as dw
import windowManager as wm
import util
import random
import math
import itertools
import physics_sim as ps
from geometry_msgs.msg import Point

class Trajectory:

    # init:  geometry_msgs.msg.PoseStamped
    # angle: physics.Orientation
    # force: vector
    def __init__(self, init, force):
        self.init = init
        # self.angle = angle
        self.force = force

    # returns: acceleration in each direction 
    def release(self, mass):
        return [self.force[0]/mass, self.force[1]/mass, self.force[2]/mass]

    # def final(self, mass):
    #     x = self.init.position.x + 
    #     y = self.init.position.y + 
    #     z = self.init.position.z + 
    

        

    # def get_point(self, dist):
    #     x = self.init.x + (dist.x * math.cos(self.angle.phi))
    #     y = self.init.y + (dist.y * math.cos(self.angle.theta))
    #     z = self.init.z + (dist.z * math.cos(self.angle.psi))
    #     return Point(x,y,z)

    # def magnitude(self, dist):
        

    # def __init__(self, init, goal):
    #     self.init = init
    #     self.goal = goal


    # def get_vector(self):
    #     return [(self.goal.x - self.init.x), \
    #                 (self.goal.y - self.init.y), \
    #                 (self.goal.z - self.init.z)]

    # def get_length(self):
    #     x = self.goal.x - self.init.x
    #     y = self.goal.y - self.init.y
    #     z = self.goal.z - self.init.z
    #     return math.sqrt(x**2 + y**2 + z**2)

    # def get_angle(self):
    #     # angle from vertical
    #     vert = [0,0,1]
    #     dot_prod = numpy.linalg.dot(vert, self.get_vector())
    #     return math.acos(dot_prod / self.get_length())
    

def drange(start, stop, step):
    r = start
    while r < stop:
        yield r
        r += step

# class trajectory:
#     def __init__(self, start, goal, size, theta):
#         self.start = start
#         self.goal = goal
#         self.size = size
#         self.theta = theta
#         self.good = self.good()        

#     def discretize(self, step):
#         path = []
#         x = (self.goal[0] - self.start[0])/step
#         y = (self.goal[1] - self.start[1])/step
#         z = (self.goal[2] - self.start[2])/step
#         for i in range(step):
#             point = (self.start[0]+(x*i), self.start[1]+(y*i), self.start[2]+(z*i))
#             path.append(point)
#         path.append(self.goal)
#         return path

#     def collides(self, obstacles):
#         pts = self.discretize(4)
#         for pt in pts:
#             traj_pt = om.Box(self.size[0], self.size[1], self.size[2], name='traj_pt', \
#                                  pose=util.Pose(pt[0], pt[1], pt[2], self.theta))
#             for obs in obstacles:
#                 if (traj_pt.collides(obs)):
#                     return True
#         return False

#     def get_verts(self):
#         return [(0.0,0.0,0.0), (self.size[0],0.0,0.0), \
#                     (0.0,self.size[1],0.0), (self.size[0],self.size[1],0.0), \
#                     (0.0,0.0,self.size[2]), (self.size[0],0.0,self.size[2]), \
#                     (0.0,self.size[1],self.size[2]), \
#                     (self.size[0],self.size[1],self.size[2])]

#     def good(self):
#         m = abs(self.slope())
#         (x_theta, y_theta) = self.angle()
#         max_x_theta = ps.max_angle(self.get_verts(),"+x")
#         max_y_theta = ps.max_angle(self.get_verts(),"+y")
#         print "x="+str(x_theta)+" y="+str(y_theta)
#         print "max_x="+str(max_x_theta)+" max_y="+str(max_y_theta)
#         max_x_theta = (math.pi/2) - max_x_theta
#         max_y_theta = (math.pi/2) - max_y_theta
#         #print "max_x="+str(max_x_theta*(180/math.pi))+" x="+str(abs(x_theta*(180/math.pi)))
#         #print "max_y="+str(max_y_theta*(180/math.pi))+" y="+str(abs(y_theta*(180/math.pi)))
#         if (abs(x_theta) > max_x_theta) or (abs(y_theta) > max_y_theta):
#             return False
#         return True

#         # threshold = 0.9
#         # if (abs(self.slope()) < threshold):
#         #     return False
#         # else:
#         #     return True

#     def angle(self):
#         vert = [0.0,0.0,1.0]
#         vec = [self.goal[0]-self.start[0], \
#                    self.goal[1]-self.start[1], \
#                    self.goal[2]-self.start[2]]
#         x_angle = math.atan(vec[0]/vec[2])
#         y_angle = math.atan(vec[1]/vec[2])
#         return (x_angle, y_angle)

#     def slope(self):
#         vert = [0.0,0.0,1.0]
#         vec = [self.goal[0]-self.start[0], \
#                    self.goal[1]-self.start[1], \
#                    self.goal[2]-self.start[2]]
#         mag = math.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)
#         unit_vec = [vec[0]/mag, vec[1]/mag, vec[2]/mag]
#         dot_prod = sum(x*y for x,y in zip(vert, unit_vec))
#         #print "dot_prod="+str(dot_prod)
#         return dot_prod
    
#     def draw_dots(self,window):
#         window.drawOval(((self.goal[0]-0.05),(self.goal[1]-0.05)), ((self.goal[0]+0.05),(self.goal[1]+0.05)), color='blue')
        

#     def draw(self, window):
#         #window.drawPoint(self.goal[0],self.goal[1], color="blue")
#         # window.drawRect(((self.goal[0]-self.size[0]/2),(self.goal[1]-self.size[1]/2)),((self.goal[0]+self.size[0]/2), (self.goal[1]+self.size[1]/2)), color='yellow')
#         #window.drawOval(((self.goal[0]-0.05),(self.goal[1]-0.05)), ((self.goal[0]+0.05),(self.goal[1]+0.05)), color='blue')


#         box = om.Box(self.size[0], self.size[1], self.size[2], name='traj', \
#                          pose=util.Pose(self.goal[0], self.goal[1], 0, \
#                                             self.theta))
#         box.draw(window)
