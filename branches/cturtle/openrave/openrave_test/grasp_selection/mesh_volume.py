"""

GNU Triangulated Surface Library
http://gts.sourceforge.net/
http://stackoverflow.com/questions/1406029/how-to-calculate-the-volume-of-a-3d-mesh-object-the-surface-of-which-is-made-up-t

1, install gts
2, install pygts
3, rock on!

MATLAB tetramesh
http://www.mathworks.com/help/techdoc/ref/tetramesh.html
http://stackoverflow.com/questions/1838537/how-do-i-break-a-polyhedron-into-tetrahedra-in-matlab

"""
import gts
from enthought.mayavi import mlab
from numpy import *

def test_tetrahedron():
    
    """
    A tetrahedron is assembled from these primitives as follows.  
    First, create Vertices for each of the tetrahedron's points:
    """
    v1 = gts.Vertex(1,1,1)
    v2 = gts.Vertex(-1,-1,1)
    v3 = gts.Vertex(-1,1,-1)
    v4 = gts.Vertex(1,-1,-1)

    """
    Next, connect the four vertices to create six unique Edges:
    """
    e1 = gts.Edge(v1,v2)
    e2 = gts.Edge(v2,v3)
    e3 = gts.Edge(v3,v1)
    e4 = gts.Edge(v1,v4)
    e5 = gts.Edge(v4,v2)
    e6 = gts.Edge(v4,v3)

    """
    The four triangular faces are composed using three edges each:
    """    
    f1 = gts.Face(e1,e2,e3)
    f2 = gts.Face(e1,e4,e5)
    f3 = gts.Face(e2,e5,e6)
    f4 = gts.Face(e3,e4,e6)
    
    """
    Finally, the surface is assembled from the faces.
    Some care must be taken in the orientation of the faces.  
    In the above example, the surface normals are pointing inward, 
    and so the surface technically defines a void, rather than a solid.  
    To create a tetrahedron with surface normals pointing outward, 
    use the following instead:
    """
    s = gts.Surface()
    for face in [f1,f2,f3,f4]:
        if not face.is_compatible(s):
            face.revert() 
        s.add(face)    

    print s.volume()
    print s.center_of_mass()
    #plot_surface(s)
    #mlab.show()
    
def test_bizzaro_mug():
    pass
def plot_surface(s):
    x,y,z,t = gts.get_coords_and_face_indices(s,True)
    mlab.triangular_mesh(x,y,z,t,color=(0.8,0.8,0.8))
    mlab.triangular_mesh(x,y,z,t,color=(0,0,1),representation='fancymesh', tube_radius=.001,scale_factor=0.001)
    
if __name__ == '__main__':
    test_bizzaro_mug()
    # mug with handle
    #vertices = array([[ 0.04 ,  0.04 ,  0.1  ], [ 0.04 ,  0.04 ,  0.   ], [ 0.04 , -0.034,  0.1  ], [ 0.04 , -0.034,  0.   ],       [ 0.034,  0.04 ,  0.1  ],       [ 0.034,  0.04 ,  0.   ],       [ 0.034, -0.034,  0.1  ],       [ 0.034, -0.034,  0.   ],       [ 0.034,  0.04 ,  0.1  ],       [ 0.034,  0.04 ,  0.   ],       [ 0.034,  0.034,  0.1  ],       [ 0.034,  0.034,  0.   ],       [-0.04 ,  0.04 ,  0.1  ],       [-0.04 ,  0.04 ,  0.   ],       [-0.04 ,  0.034,  0.1  ],       [-0.04 ,  0.034,  0.   ],       [-0.034,  0.034,  0.1  ],       [-0.034,  0.034,  0.   ],       [-0.034, -0.04 ,  0.1  ],       [-0.034, -0.04 ,  0.   ],       [-0.04 ,  0.034,  0.1  ],       [-0.04 ,  0.034,  0.   ],       [-0.04 , -0.04 ,  0.1  ],       [-0.04 , -0.04 ,  0.   ],       [ 0.04 , -0.034,  0.1  ],       [ 0.04 , -0.034,  0.   ],       [ 0.04 , -0.04 ,  0.1  ],       [ 0.04 , -0.04 ,  0.   ],       [-0.034, -0.034,  0.1  ],       [-0.034, -0.034,  0.   ],       [-0.034, -0.04 ,  0.1  ],       [-0.034, -0.04 ,  0.   ],       [ 0.005,  0.07 ,  0.026],       [ 0.005,  0.07 ,  0.02 ],       [ 0.005,  0.04 ,  0.026],       [ 0.005,  0.04 ,  0.02 ],       [-0.005,  0.07 ,  0.026],       [-0.005,  0.07 ,  0.02 ],       [-0.005,  0.04 ,  0.026],       [-0.005,  0.04 ,  0.02 ],       [ 0.005,  0.07 ,  0.08 ],       [ 0.005,  0.07 ,  0.074],       [ 0.005,  0.04 ,  0.08 ],       [ 0.005,  0.04 ,  0.074],       [-0.005,  0.07 ,  0.08 ],       [-0.005,  0.07 ,  0.074],       [-0.005,  0.04 ,  0.08 ],       [-0.005,  0.04 ,  0.074],       [ 0.005,  0.076,  0.08 ],       [ 0.005,  0.076,  0.02 ],       [ 0.005,  0.07 ,  0.08 ],       [ 0.005,  0.07 ,  0.02 ],       [-0.005,  0.076,  0.08 ],       [-0.005,  0.076,  0.02 ],       [-0.005,  0.07 ,  0.08 ],       [-0.005,  0.07 ,  0.02 ],       [ 0.034,  0.034,  0.006],       [ 0.034,  0.034,  0.   ],       [ 0.034, -0.034,  0.006],       [ 0.034, -0.034,  0.   ],       [-0.034,  0.034,  0.006],       [-0.034,  0.034,  0.   ],       [-0.034, -0.034,  0.006],       [-0.034, -0.034,  0.   ]])
    #indices = array([[ 1,  0,  2],       [ 2,  1,  3],       [ 5,  4,  6],       [ 6,  5,  7],       [ 1,  0,  4],       [ 4,  1,  5],       [ 3,  2,  6],       [ 6,  3,  7],       [ 0,  2,  4],       [ 2,  4,  6],       [ 1,  3,  5],       [ 3,  5,  7],       [ 9,  8, 10],       [10,  9, 11],       [13, 12, 14],       [14, 13, 15],       [ 9,  8, 12],       [12,  9, 13],       [11, 10, 14],       [14, 11, 15],       [ 8, 10, 12],       [10, 12, 14],       [ 9, 11, 13],       [11, 13, 15],       [17, 16, 18],       [18, 17, 19],       [21, 20, 22],       [22, 21, 23],       [17, 16, 20],       [20, 17, 21],       [19, 18, 22],       [22, 19, 23],       [16, 18, 20],       [18, 20, 22],       [17, 19, 21],       [19, 21, 23],       [25, 24, 26],       [26, 25, 27],       [29, 28, 30],       [30, 29, 31],       [25, 24, 28],       [28, 25, 29],       [27, 26, 30],       [30, 27, 31],       [24, 26, 28],       [26, 28, 30],       [25, 27, 29],       [27, 29, 31],       [32, 33, 34],       [34, 33, 35],       [36, 37, 38],       [38, 37, 39],       [32, 33, 36],       [33, 36, 37],       [34, 35, 38],       [35, 38, 39],       [34, 32, 36],       [36, 34, 38],       [35, 33, 37],       [37, 35, 39],       [40, 41, 42],       [42, 41, 43],       [44, 45, 46],       [46, 45, 47],       [40, 41, 44],       [41, 44, 45],       [42, 43, 46],       [43, 46, 47],       [42, 40, 44],       [44, 42, 46],       [43, 41, 45],       [45, 43, 47],       [49, 48, 50],       [50, 49, 51],       [53, 52, 54],       [54, 53, 55],       [49, 48, 52],       [52, 49, 53],       [51, 50, 54],       [54, 51, 55],       [48, 50, 52],       [50, 52, 54],       [49, 51, 53],       [51, 53, 55],       [56, 57, 58],       [58, 57, 59],       [60, 61, 62],       [62, 61, 63],       [56, 57, 60],       [60, 57, 61],       [58, 59, 62],       [62, 59, 63],       [56, 58, 60],       [60, 58, 62],       [57, 59, 61],       [61, 59, 63]], dtype=int32)
    # mug without handle
    vertices = array([[ 0.04 ,  0.04 ,  0.1  ],       [ 0.04 ,  0.04 ,  0.   ],       [ 0.04 , -0.034,  0.1  ],       [ 0.04 , -0.034,  0.   ],       [ 0.034,  0.04 ,  0.1  ],       [ 0.034,  0.04 ,  0.   ],       [ 0.034, -0.034,  0.1  ],       [ 0.034, -0.034,  0.   ],       [ 0.034,  0.04 ,  0.1  ],       [ 0.034,  0.04 ,  0.   ],       [ 0.034,  0.034,  0.1  ],       [ 0.034,  0.034,  0.   ],       [-0.04 ,  0.04 ,  0.1  ],       [-0.04 ,  0.04 ,  0.   ],       [-0.04 ,  0.034,  0.1  ],       [-0.04 ,  0.034,  0.   ],       [-0.034,  0.034,  0.1  ],       [-0.034,  0.034,  0.   ],       [-0.034, -0.04 ,  0.1  ],       [-0.034, -0.04 ,  0.   ],       [-0.04 ,  0.034,  0.1  ],       [-0.04 ,  0.034,  0.   ],       [-0.04 , -0.04 ,  0.1  ],       [-0.04 , -0.04 ,  0.   ],       [ 0.04 , -0.034,  0.1  ],       [ 0.04 , -0.034,  0.   ],       [ 0.04 , -0.04 ,  0.1  ],       [ 0.04 , -0.04 ,  0.   ],       [-0.034, -0.034,  0.1  ],       [-0.034, -0.034,  0.   ],       [-0.034, -0.04 ,  0.1  ],       [-0.034, -0.04 ,  0.   ],       [ 0.034,  0.034,  0.006],       [ 0.034,  0.034,  0.   ],       [ 0.034, -0.034,  0.006],       [ 0.034, -0.034,  0.   ],       [-0.034,  0.034,  0.006],       [-0.034,  0.034,  0.   ],      [-0.034, -0.034,  0.006],       [-0.034, -0.034,  0.   ]])
    indices = array([[ 1,  0,  2],       [ 2,  1,  3],       [ 5,  4,  6],       [ 6,  5,  7],       [ 1,  0,  4],       [ 4,  1,  5],       [ 3,  2,  6],       [ 6,  3,  7],       [ 0,  2,  4],       [ 2,  4,  6],       [ 1,  3,  5],       [ 3,  5,  7],       [ 9,  8, 10],       [10,  9, 11],       [13, 12, 14],       [14, 13, 15],       [ 9,  8, 12],       [12,  9, 13],       [11, 10, 14],       [14, 11, 15],       [ 8, 10, 12],       [10, 12, 14],       [ 9, 11, 13],       [11, 13, 15],       [17, 16, 18],       [18, 17, 19],       [21, 20, 22],       [22, 21, 23],       [17, 16, 20],       [20, 17, 21],       [19, 18, 22],       [22, 19, 23],       [16, 18, 20],       [18, 20, 22],       [17, 19, 21],       [19, 21, 23],       [25, 24, 26],       [26, 25, 27],       [29, 28, 30],       [30, 29, 31],       [25, 24, 28],       [28, 25, 29],       [27, 26, 30],       [30, 27, 31],       [24, 26, 28],       [26, 28, 30],       [25, 27, 29],       [27, 29, 31],       [32, 33, 34],       [34, 33, 35],       [36, 37, 38],       [38, 37, 39],       [32, 33, 36],       [36, 33, 37],       [34, 35, 38],       [38, 35, 39],       [32, 34, 36],       [36, 34, 38],       [33, 35, 37],       [37, 35, 39]], dtype=int32)    
    
#    x = tuple(v[0] for v in vertices)
#    y = tuple(v[1] for v in vertices)
#    z = tuple(v[2] for v in vertices)
#
#    t = tuple((i[0],i[1],i[2]) for i in indices)
#
#    mlab.triangular_mesh(x,y,z,t,color=(0.8,0.8,0.8))
#    mlab.triangular_mesh(x,y,z,t,color=(0,0,1),representation='fancymesh',
#                         scale_factor=.00000001)
#    mlab.show()
    vertices_gts = [gts.Vertex(v[0],v[1],v[2]) for v in vertices]
    edges_gts = []
    faces_gts = []
    for tri in indices:
        #print tri,
        edges_gts.append(gts.Edge(vertices_gts[tri[0]],vertices_gts[tri[1]]))
        edges_gts.append(gts.Edge(vertices_gts[tri[1]],vertices_gts[tri[2]]))
        edges_gts.append(gts.Edge(vertices_gts[tri[2]],vertices_gts[tri[0]]))
        faces_gts.append(gts.Face(edges_gts[-3],edges_gts[-2],edges_gts[-1]))
#    for i in xrange(0,len(edges_gts),3):
#        faces_gts.append(gts.Face(edges_gts[i],edges_gts[i+1],edges_gts[i+2]))
    
    s = gts.Surface()
    for face in faces_gts:
        #if not face.is_compatible(s):
        #    face.revert()
        #if face.is_compatible(s):
        #    s.add(face)
        s.add(face)
        
    
    #print s.volume()
    #print s.center_of_mass()
    plot_surface(s)
    mlab.show()