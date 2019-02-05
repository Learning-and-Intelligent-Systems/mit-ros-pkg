from numpy import *
from mathutils import *
import operator

"""
using svd to fit 3D plane 
http://comments.gmane.org/gmane.comp.python.numeric.general/37854
http://mathforum.org/library/drmath/view/63765.html

3D point projection
http://mathforum.org/library/drmath/view/51727.html
http://www.9math.com/book/projection-point-plane

"""

def fit_plane_3d(points):
    """
    given points consist of 3d point locations,
    1, find centroid of all points
    2, shift all points by centroid
    3, find a,b,c that has least orthogonal distance 
    4, return a,b,c in ax+by+cz=0 and centroid
    """
    centroid = mean(points,axis=0)
    M = points-centroid
    u,s,vh = linalg.linalg.svd(M)
    
    v = vh.conj().transpose()
    a,b,c = v[:,-1]
    return (a,b,c),centroid

def point_projection_on_plane_3d(points):
    """
    0, fit a least orthogonal distance plane
    1, shift all points by centroid
    2, project all points to the plane
    3, shift the projections back
    4, return the new points
    """
    (a,b,c),centroid = fit_plane_3d(points)
    points = points - centroid
    result = []
    for point in points:
        u,v,w = point
        t = (a*u+b*v+c*w)/(a*a+b*b+c*c)
        x = u-a*t
        y = v-b*t
        z = w-c*t
        result.append((x,y,z))
    result = array(result)+centroid
    return result
    
    def test():
        # test algorithm
        A = random.randn(100,2)*mat([[2,3,-1],[0,0,2]])
        A = A + random.randn(100,3)/3.0
        u,s,vh = linalg.linalg.svd(A)
        v = vh.conj().transpose()
        print v[:,-1] # ~ [[-.84], [.53], [.01]]
        
if __name__=="__main__":
    data = array([(1.034, -0.026771870015653045, 1.0841803908545353), (1.034, -0.026617511120084286, 1.0775075659388982), (1.034, -0.026327895522146694, 1.0765811162186782), (1.034, -0.025898037587842204, 1.0756863291415169), (1.034, -0.026137838922249433, 1.0761247838958625), (1.034, -0.025243605734428926, 1.0748475542925005), (1.034, -0.024669165145521081, 1.0741769826904926), (1.034, -0.023702534559277292, 1.0735484677123051), (1.034, -0.022587975657783976, 1.0728823270377577), (1.034, -0.021737260614921483, 1.0726244812849806), (1.034, -0.020784483700306819, 1.0724550453387012), (1.034, -0.016996156220960786, 1.0723843105120912), (1.034, -0.026788548697779777, 1.0953774787004145), (1.034, -0.0071431802788677378, 1.0899175270187917), (1.034, -0.0070943520745306559, 1.0792568593347347), (1.034, -0.026765357896944439, 1.0904893922220045), (1.034, -0.013471222875040051, 1.0723735101865617), (1.034, -0.013201371277356128, 1.0723740230125509), (1.034, -0.012985713141924777, 1.0723790647374685), (1.034, -0.012929326537822824, 1.0723923477344695), (1.034, -0.010218129996154393, 1.0735110564513732), (1.034, -0.0089174619543553671, 1.0746368307732614), (1.034, -0.010475504519395188, 1.073328605026183), (1.034, -0.01177223268241207, 1.0728158639247645), (1.034, -0.0072240100689609857, 1.0780364779534723), (1.034, -0.0071038623091101444, 1.0786092934127125), (1.034, -0.007537581492610879, 1.0770006656656126), (1.034, -0.0078194264728389598, 1.0762570607643744), (1.034, -0.0086278621106048725, 1.0749229901010238), (1.034, -0.0082126821238892866, 1.075531575689687), (1.034, -0.0096155759252126126, 1.0740201380568748), (1.0343345036259888, -0.0185359265497415, 1.1000000000000001), (1.0342750818163129, -0.026045686668489235, 1.1000000000000001), (1.0340812323369999, -0.026625510703360195, 1.1000000000000001), (1.0340905903448498, -0.0073673142721457224, 1.1000000000000001), (1.0342844384034207, -0.0079870309080803774, 1.1000000000000003), (1.0343340320770349, -0.010095040459831481, 1.1000000000000001), (1.0342052636411267, -0.0076172421664917013, 1.1000000000000001), (1.0343308233537791, -0.02573339780327593, 1.1000000000000001), (1.0343395395246555, -0.0090232806678484552, 1.1000000000000001), (1.0343292030708442, -0.011043809431387337, 1.1000000000000001), (1.0341951058479002, -0.026375412315073631, 1.1000000000000001), (1.0343285721527447, -0.010564022921677714, 1.1000000000000001)])

    # test if all points are on the same plane
    points = data
    n = len(points)
    norms = {}
    for i in xrange(n-2):
        a = points[i]
        b = points[i+1]
        c = points[n-1]
        ax,ay,az = points[i]
        bx,by,bz = points[i+1]
        cx,cy,cz = points[n-1]
        norm = vecNormalized(vecNormal(a,b,c))
        norm_key = '%.6f %.6f %.6f'%(norm[0],norm[1],norm[2])
        norm_key_neg = '%.6f %.6f %.6f'%(-norm[0],-norm[1],-norm[2])
        if norms.has_key(norm_key):
            norms[norm_key] += 1
        elif norms.has_key(norm_key_neg):
            norms[norm_key_neg] +=1
        else:
            norms[norm_key] = 0
    print 'not all 40 points are on the same plane'
    print 'most common normal',max(norms.iteritems(),key=operator.itemgetter(1))[0],max(norms.values())
    
    points = point_projection_on_plane_3d(data)
    
    # test if all points are on the same plane
    n = len(points)
    norms = {}
    for i in xrange(n-2):
        a = points[i]
        b = points[i+1]
        c = points[n-1]
        ax,ay,az = points[i]
        bx,by,bz = points[i+1]
        cx,cy,cz = points[n-1]
        norm = vecNormalized(vecNormal(a,b,c))
        norm_key = '%.6f %.6f %.6f'%(norm[0],norm[1],norm[2])
        norm_key_neg = '%.6f %.6f %.6f'%(-norm[0],-norm[1],-norm[2])
        if norms.has_key(norm_key):
            norms[norm_key] += 1
        elif norms.has_key(norm_key_neg):
            norms[norm_key_neg] +=1
        else:
            norms[norm_key] = 0
    print 'all 40 input should be on the same plane'
    print 'most common normal',max(norms.iteritems(),key=operator.itemgetter(1))[0],max(norms.values())