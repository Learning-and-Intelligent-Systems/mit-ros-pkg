from openravepy import *
from math import sqrt
from numpy import *
from mathutils import *

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

class FingertipVisibility:
    def __init__(self,manip,fingertip='right',reverse=False):
        self.manip = manip
        self.robot = self.manip.GetRobot()
        if fingertip=='both':
            self.both = True
        else:
            self.both = False
            if fingertip == 'right':
                self.fingertip = self.manip.GetRobot().GetLink('l_gripper_r_finger_tip_link')
            elif fingertip == 'left':
                self.fingertip = self.manip.GetRobot().GetLink('l_gripper_l_finger_tip_link')
            else:
                print 'error: fingertip must equal left or right'
        self.reverse = reverse
        self.hiddengeoms = []
    def __enter__(self):
        self.hiddengeoms = []
        lefttip = self.manip.GetRobot().GetLink('l_gripper_l_finger_tip_link')
        righttip = self.manip.GetRobot().GetLink('l_gripper_r_finger_tip_link')
        with self.robot.GetEnv():
            for link in self.manip.GetChildLinks():
                if self.both:
                    if link!=lefttip and link!=righttip:
                        for geom in link.GetGeometries():
                            self.hiddengeoms.append((geom,geom.IsDraw()))
                            geom.SetDraw(False)
                            link.Enable(self.reverse)
                    else:
                        link.Enable(not self.reverse)
                        for geom in link.GetGeometries():
                            geom.SetTransparency(.9)
                    continue
                if link!= self.fingertip:
                    for geom in link.GetGeometries():
                        self.hiddengeoms.append((geom,geom.IsDraw()))
                        geom.SetDraw(False)
                        link.Enable(self.reverse)
                else:
                    #print link
                    link.Enable(not self.reverse)
                    for geom in link.GetGeometries():
                        geom.SetTransparency(.9)

    def __exit__(self,type,value,traceback):
        with self.robot.GetEnv():
            for geom,isdraw in self.hiddengeoms:
                geom.SetDraw(isdraw)
            for link in self.manip.GetChildLinks():
                #if link.GetIndex() != self.fingertip.GetIndex():
                link.Enable(True)

class memoized(object):
    """Decorator that caches a function's return value each time it is called.
    If called later with the same arguments, the cached value is returned, and
    not re-evaluated.
    """
    def __init__(self, func):
        self.func = func
        self.cache = {}
    def __call__(self, *args):
        try:
            return self.cache[args]
        except KeyError:
            value = self.func(*args)
            self.cache[args] = value
            return value
        except TypeError:
            # uncachable -- for instance, passing a list as an argument.
            # Better to not cache than to blow up entirely.
            return self.func(*args)
    def __repr__(self):
        """Return the function's docstring."""
        return self.func.__doc__
    def __get__(self, obj, objtype):
        """Support instance methods."""
        return functools.partial(self.__call__, obj)
    
def pause():
    raw_input('press ENTER to continue...')

def setObjectTransparency(target,transparency=.9):
    for link in target.GetLinks():
        for geom in link.GetGeometries():
            geom.SetTransparency(transparency)

def drawTransform(env,T,length=.01,width=.001):
    """draws a set of arrows around a coordinate system
    """
    return [env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,0],linewidth=width,color=[1.0,0.0,0.0]),
            env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,1],linewidth=width,color=[0.0,1.0,0.0]),
            env.drawarrow(p1=T[0:3,3],p2=T[0:3,3]+length*T[0:3,2],linewidth=width,color=[0.0,0.0,1.0])]
            
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
        O_T_G = robot.GetActiveManipulator().GetEndEffectorTransform() # grasping frame G in global frame O
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

@memoized
def enorm0_3d(x0,y0,z0,x1,y1,z1):
    """
    ENORM0_3D computes the Euclidean norm of (P1-P0) in 3D.
    from http://orion.math.iastate.edu/burkardt/c_src/geometryc/geometryc.html
    """
    value = sqrt((x1-x0)*(x1-x0)+\
                 (y1-y0)*(y1-y0)+\
                 (z1-z0)*(z1-z0))
    return value
        
@memoized
def triangle_area_3d(x1,y1,z1,x2,y2,z2,x3,y3,z3):
    """
    TRIANGLE_AREA_3D computes the area of a triangle in 3D.
    from http://orion.math.iastate.edu/burkardt/c_src/geometryc/geometryc.html
    """
    # find the projection of P3-P1 onto P2-P1
    dot = (x2-x1)*(x3-x1) +\
          (y2-y1)*(y3-y1) +\
          (z2-z1)*(z3-z1) 
      
    base = enorm0_3d(x1,y1,z1,x2,y2,z2)
    
    # the height of the triangle is the length of P3-P1 after its projection onto P2-P1 has been subtracted
    if base == 0.0:
        height = 0.0
    else:
        alpha = dot / (base*base)
        a = x3-x1-alpha*(x2-x1)
        b = y3-y1-alpha*(y2-y1)
        c = z3-z1-alpha*(z2-z1)
        height = sqrt(a*a+b*b+c*c)
        
    area = .5 * base * height
    return area
@memoized
def cross_3d(a,b,c):
    return cross(b-a,c-a)

@memoized
def vec_norm(v):
    return v/vec_mag(v)

@memoized
def vec_mag(v):
    return dot(v,v)**.5
@memoized
def centroid_3d(verts):
    """
    POLYGON_CENTROID_3D computes the centroid of a polygon in 3D.
    The centroid is the area-weighted sum of the centroids of
    disjoint triangles that make up the polygon.
    from http://orion.math.iastate.edu/burkardt/c_src/geometryc/geometryc.html
    """
    
    area = 0.0
    areat = 0.0
    centerx = 0.0
    centery = 0.0
    centerz = 0.0
    n = len(verts)
    
    norms = {}
    
    for i in xrange(n-2):
        ax,ay,az = verts[i]
        bx,by,bz = verts[i+1]
        cx,cy,cz = verts[n-1]
         
        areat = triangle_area_3d(ax,ay,az,
                                 bx,by,bz,
                                 cx,cy,cz)
        
        area += areat
        centerx += areat * (ax+bx+cx) / 3.0
        centery += areat * (ay+by+cy) / 3.0
        centerz += areat * (az+bz+cz) / 3.0 

    if area==0:
        centerx,centery,centerz = mean(verts,axis=0)
    else:
        centerx = centerx / area
        centery = centery / area
        centerz = centerz / area
    #print '(%.3f %.3f %.3f) %.6f'%(centerx,centery,centerz,area)
    return centerx,centery,centerz
