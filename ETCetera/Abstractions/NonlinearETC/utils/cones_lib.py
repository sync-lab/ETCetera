import cdd
import numpy as np

class Cone_on_Plane(object):
    """Attributes:
        Quadratic_Form (list of lists, 2x2 matrix): The quadratic form of the cone.
        Linear_Form (list, 2x1 vector): The linear form of the matrix.
    """
    def __init__(self, a, b, plane_flag):
        self.Quadratic_Form = [[-2*np.sin(a)*np.sin(b), np.sin(a+b)], [np.sin(a+b), -2*np.cos(a)*np.cos(b)]]
        if (plane_flag):
            self.Linear_Form = [np.cos(a),np.sin(a)]
        else:
            self.Linear_Form = [-np.cos(a),-np.sin(a)]

class Polyhedral_Cone(object):
    """Attributes:
        vertices (list of lists): vertices defining the cone.
        A (list of lists, matrix): the A-matrix of the halfspace representation.
        b (list): the b-matrix of the halfspace representation.
    """
    def __init__(self, vertices, A, b):
        self.vertices = vertices
        self.A = A
        self.b = b            

def polyhedral_cone_vertices2hrep(vertices):
    """INPUTS: vertices (list of lists)
    
    Given the vertices, returns the b and A matrix of the halfspace rep.
    of a polyhedral cone. Uses the cdd library.
    """
    cdd_vertices = []
# =============================================================================
#     for i in range(0,np.shape(vertices)[0]): #put a 1 in front of all vertices
#         #this constructs the vertices of the vertex rep for cdd
#          temp=vertices[i][:]
#          temp.append(1)
#          temp.insert(0,1)
#          cdd_vertices.append(temp)
# =============================================================================
    for i in range(0,np.shape(vertices)[0]): #put a 0 in front of all vertices, to get a polyhedral cone
        #this constructs the rays of the vertex rep for cdd
        temp=vertices[i][:]
        #temp.append(1)
        temp.insert(0,0)
        cdd_vertices.append(temp)
    mat = cdd.Matrix(cdd_vertices,number_type='float')
    mat.rep_type = cdd.RepType.GENERATOR
    poly = cdd.Polyhedron(mat)
    ext = poly.get_inequalities()
    b = np.zeros(np.shape(ext)[0])
    A = np.zeros([np.shape(ext)[0], np.shape(ext)[1]-1])
    for i in range(0,np.shape(ext)[0]):
        b[i] = ext[i][0]
        for j in range(0,np.shape(ext)[1]-1):
            A[i,j] = -ext[i][j+1]       
    return [b,A]    
    
def polytope_vrep2hrep(vertices):
    """INPUTS: vertices (list of lists)
    
    Given the vertices, returns the b and A matrix of the halfspace rep.
    of a polytope. Uses the cdd library.
    """
    cdd_vertices = []
    for i in range(0,np.shape(vertices)[0]):#put a 1 in front of all vertices, to get a polytope
        #this constructs the vertices of the vertex rep for cdd
        temp=vertices[i][:]
        temp.insert(0,1)
        cdd_vertices.append(temp[:])
    mat = cdd.Matrix(cdd_vertices,number_type='float')
    mat.rep_type = cdd.RepType.GENERATOR
    poly = cdd.Polyhedron(mat)
    ext = poly.get_inequalities()
    b = np.zeros(np.shape(ext)[0])
    A = np.zeros([np.shape(ext)[0], np.shape(ext)[1]-1])
    for i in range(0,np.shape(ext)[0]):
        b[i] = ext[i][0]
        for j in range(0,np.shape(ext)[1]-1):
            A[i,j] = -ext[i][j+1]       
    return [b,A]
    
