from openravepy import *
from numpy import *
import time
from utilities import *
@memoized
def estimate_com(body):
    area = 0
    centerx,centery,centerz = 0,0,0
    for tri in body.GetLinks()[0].GetCollisionData().indices:
        ax,ay,az = body.GetLinks()[0].GetCollisionData().vertices[tri[0]]
        bx,by,bz = body.GetLinks()[0].GetCollisionData().vertices[tri[1]]
        cx,cy,cz = body.GetLinks()[0].GetCollisionData().vertices[tri[2]]
        areat = triangle_area_3d(ax,ay,az,bx,by,bz,cx,cy,cz)
        area += areat
        centerx += areat * (ax+bx+cx) / 3.0
        centery += areat * (ay+by+cy) / 3.0
        centerz += areat * (az+bz+cz) / 3.0 

    centerx = centerx / area
    centery = centery / area
    centerz = centerz / area
    return centerx,centery,centerz    

if __name__=='__main__':
    env = Environment()
    body = env.ReadKinBodyXMLFile('data/mug2.kinbody.xml')
    setObjectTransparency(body)
    env.AddKinBody(body)
    time.sleep(.2)
    env.SetViewer('qtcoin')
    centerx,centery,centerz = estimate_com(body)
    O_T_C = dot(body.GetTransform(),[centerx,centery,centerz,1])
    O_P_C = O_T_C[:3]
    O_P_CEnd = [O_P_C[0],O_P_C[1],O_P_C[2]-.05]
    handles = []
    handles.append(env.drawarrow(p1=O_P_C, p2=O_P_CEnd, linewidth=.001,color=[0,0,1]))
    pause()
    