from openravepy import *
import operator
from numpy import *
from utilities import *

def is_bizzaro(body):
    for link in body.GetLinks():
        for geom in link.GetGeometries():
            if geom.GetType().name != 'Box':
                return False
    return True 

@memoized    
def estimate_com(body):
    """
    weighted average over box volume, assuming uniform density
    """
    if not is_bizzaro(body):
        return None
    
    c = array([0.0,0.0,0.0])
    v = 0.0
    for link in body.GetLinks():
        for geom in link.GetGeometries():
            x,y,z = geom.GetTransform()[:3,3]
            v_tmp = reduce(operator.mul,geom.GetBoxExtents())
            v += v_tmp
            c += v_tmp* array([x,y,z])
    return c/v

if __name__=='__main__':
    env = Environment()
    body = env.ReadKinBodyXMLFile('../bizzaro_mug_without_handle.kinbody.xml')
    env.AddKinBody(body)
    print estimate_com(body)
    body1 = env.ReadKinBodyXMLFile('../bizzaro_mug_small.kinbody.xml')
    body1.SetName('asf')
    env.AddKinBody(body1)
    print estimate_com(body1)