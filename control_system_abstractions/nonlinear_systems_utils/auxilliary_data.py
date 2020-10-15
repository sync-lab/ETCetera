import numpy as np
import sympy
from scipy.optimize import linprog
#import cdd
from control_system_abstractions.exceptions.nonlinear_systems_exceptions.LP_exceptions import LPGeneralException, \
    LPOptimizationFailedException


class GridObject(object):
    """Attributes:
        Centroids (list of lists of float): coordiantes of centers of polytopes of the grid.
        State_space_limits (list of lists of float): intervals for each dimension of the state space (operation region).
        Grid_points_per_dim (list of int): the number of points of discretization of each dimension.
    """

    def __init__(self, centroids, state_space_limits, grid_points_per_dim):
        self.centroids = centroids
        self.state_space_limits = state_space_limits
        self.grid_points_per_dim = grid_points_per_dim


class ConeOnPlane(object):
    """Attributes:
        Quadratic_Form (list of lists, 2x2 matrix): The quadratic form of the cone.
        Linear_Form (list, 2x1 vector): The linear form of the matrix.
    """

    def __init__(self, a, b, plane_flag):
        self.quadratic_form = [[-2 * np.sin(a) * np.sin(b), np.sin(a + b)], [np.sin(a + b), -2 * np.cos(a) * np.cos(b)]]
        if plane_flag:
            self.linear_form = [np.cos(a), np.sin(a)]
        else:
            self.linear_form = [-np.cos(a), -np.sin(a)]


class PolyhedralCone(object):
    """Attributes:
        vertices (list of lists): vertices defining the cone.
        A (list of lists, matrix): the A-matrix of the halfspace representation.
        b (list): the b-matrix of the halfspace representation.
    """

    def __init__(self, vertices, A, b):
        self.vertices = vertices
        self.A = A
        self.b = b


class SphericalDomain(object):
    """Attributes:
        manifold_time (float): The time of the manifold that is approximated by inner_radius and outer_radius.
        cone (list of Cone_On_Plane objects): The conic domain.
        inner_radius, outer_radius (float): The radius that inner/outer-approximates the manifold in the cone.
    """

    def __init__(self, manifold_time, cone, inner_radius, outer_radius):
        self.manifold_time = manifold_time
        self.cone = cone
        self.inner_radius = inner_radius
        self.outer_radius = outer_radius


class RegionHomogeneous(object):
    """Attributes:
        transitions (list of lists): The transitions of the region.
        index(list of 2 elements): The index of the region.
        inner_manifold_time, outer-manifold_time (float): Time of the inner/outer manifold.
        cone (list of Cone_On_Plane objects): Conic domain.
        inner_radius, outer_radius (float): The radii that inner/outer-approximate the region.
        contains_origin (boolean)
        symbolic_region_for_reachability_analysis (sympy expression): The region set written in sympy expression.
        timing_lower_bound, timing_upper_bound (float)
    """

    def __init__(self, state_vector, region_index, inner_manifold_time, outer_manifold_time, cone, inner_radius,
                 outer_radius, contains_origin_flag):
        self.transitions = []
        self.index = region_index
        self.inner_manifold_time = inner_manifold_time
        self.outer_manifold_time = outer_manifold_time
        self.timing_lower_bound = outer_manifold_time
        self.timing_upper_bound = 0
        self.cone = cone
        self.inner_radius = inner_radius
        self.outer_radius = outer_radius
        self.contains_origin = contains_origin_flag
        temp = 0
        temp = sum(state_vector[i] ** 2 for i in range(0, len(state_vector)))   # temp = x^2+y^2+z^2+...
        # Commented from originalfor i in range(0, len(state_vector)):
        # Commented from original    temp = temp + state_vector[i] ** 2  # temp = x^2+y^2+z^2+...
        expression = (temp - (inner_radius) ** 2 >= 0) & (temp - (outer_radius) ** 2 <= 0)
        for i in range(0, len(state_vector) - 1):  # create iteratively the symbolic expression for the conic domain
            # by iteratively taking conic_domain = conic_domain & [x_i x_{i+1}]*Q_i*[x_i x_{i+1}]^T & L_i*[x_i x_{i+1}]
            # where Q_i and L_i are the quadratic and linear forms that define the cone
            vec = sympy.Matrix([state_vector[i], state_vector[i + 1]])
            mat = sympy.Matrix(cone[i].Quadratic_Form)
            expression = expression & ((vec.transpose() * mat * vec)[0] >= 0)
            mat = sympy.Matrix(cone[i].Linear_Form)
            expression = expression & ((mat.transpose() * vec)[0] >= 0)
        self.symbolic_region_for_reachability_analysis = expression

    # Commented from original
    def set_timing_upper_bound(self, upper_bound):
        "upper_bound: float, >0."
        self.timing_upper_bound = upper_bound

    # Commented from original
    def set_transition(self, trans):
        "trans: list of 2 elemets."
        self.transitions.append(trans)


class RegionNonHomogeneous(object):
    """Attributes:
        transitions (list of lists): The transitions of the region.
        index(list of 2 elements): The index of the region.
        center (list of float): The center of the (polytopic) region.
        region_box (list of intervals): The intervals per dimension defining the region.
        hrep_A (list of lists, matrix): the A-matrix of the halfspace representation.
        hrep_b (list): the b-matrix of the halfspace representation.
        contains_origin (boolean)
        symbolic_region_for_reachability_analysis (sympy expression): The region set written in sympy expression.
        timing_lower_bound, timing_upper_bound (float)
    """

    def __init__(self, state_vector, region_index, centroid, polytope_vertices, polytope_halfspaces_A,
                 polytope_halfspaces_b, lower_bound, contains_origin_flag):
        self.transitions = []
        self.index = region_index
        self.timing_lower_bound = lower_bound
        self.timing_upper_bound = 0
        self.center = centroid
        self.hrep_A = polytope_halfspaces_A
        self.hrep_b = polytope_halfspaces_b
        self.contains_origin = contains_origin_flag

        box_domain = np.zeros((len(state_vector), 2))
        box_domain = []
        for i in range(0, len(state_vector)):  # for each i dimension
            vertex = polytope_vertices[0]
            l = [vertex[i], vertex[i]]
            for vertex in polytope_vertices[1:]:  # search all verices
                # and find lowest and highest valus in the i-th coordinate
                # to determine the interval
                if (l[0] >= vertex[i]):
                    l[0] = vertex[i]
                if (l[1] <= vertex[i]):
                    l[1] = vertex[i]
            box_domain.append(l)
        self.region_box = list(box_domain)

        A = sympy.Matrix(polytope_halfspaces_A)
        b = sympy.Matrix(polytope_halfspaces_b)
        A_times_state = A * sympy.Matrix(state_vector)
        self.symbolic_region_for_reachability_analysis = all((A_times_state[i] <= b[i]) for i in range(0, len(b)))
        # Commented from original expression = True
        # Commented from original for i in range(0, len(b)):
        # Commented from original    expression = expression & (A_times_state[i] <= b[i])
        # Commented from original self.symbolic_region_for_reachability_analysis = expression

    def insert_timing_upper_bound(self, upper_bound):
        self.timing_upper_bound = upper_bound
        if (self.timing_lower_bound > self.timing_upper_bound):
            self.timing_lower_bound = 0.8 * self.timing_upper_bound

    # Commented from original def insert_transition(self, trans):
    def set_transition(self, trans):
        self.transitions.append(trans)


class LPData(object):
    """Stores and manipulates the Linear programming data

    Constructor arguments:
        s (object of class Spec): Specification of the Spec class
    optional arguments:
        C (numpy 1-d matrix): LP cost function vector c, s.t. the cost function is c^T.x.
            Dimension: p + 1 (approximation order plus 1).
        D (tuple): bounds on x. Dimension: p + 1 (approximation order plus 1)

    Attributes:
        Spec: specification
        A: A constraint matrix of linear programming
        B: B constraint matrix of linear programming
        C: objective vector
        D: bounds on the optimiztion variables
        solutions: Found solutions of the LP iterations
    """

    def __init__(self, s, C=[], D=[]):
        """Constructor"""
        self.A = []
        self.B = []
        self.C = C
        self.D = D
        self.solutions = []

        # Create A and B matrix
        for point in s.samples:  # given the sample points from Spec
            temp_A, temp_B = self.calculate_constraints(s, point, 0)
            self.A.extend(temp_A)
            self.B.extend(temp_B)

        if not C:  # if cost function is not given use default
            self.C = np.zeros(s.p + 1)
            self.C[s.p - 1::1] = [0, 1]
            # Commented from original self.C[s.p - 1] = 0 self.C[s.p] = 1
        elif C and len(C) != s.p + 1:   # If cost function is given but incorrect length
            raise LPGeneralException('User defined c vector is of the wrong dimension. Default cost function used instead')
        else:  # if cost function is given correct way, then nothing to do
            pass

        if not D:  # if bounds are not given, use default bounds
            # Commented from original self.D = []
            self.D = [(None, None) for d in range(1, s.p)]
            # Commented from original self.D.append((0, None))  # Delta_n self.D.append((0, None))  # Gamma >= 0
            self.D.extend((0, None), (0, None))     # Delta_n and Gamma >= 0
            self.D = tuple(self.D)
        elif D and len(D) != s.p + 1:   # If bounds are given but incorrect length
            raise LPGeneralException('User defined bounds D are of the wrong dimension. Default bounds used instead')
        else:   # if bounds are given correct way, then nothing to do
            pass


    def LP_solve(self, lp_method, Verbose=False):
        """Solves the LP problem"""
        print("Initialising Optimization")
        try:
            if not lp_method:
                lp_method = 'revised simplex'

            if lp_method == 'interior-point':
                res = linprog(self.C, A_ub=self.A, b_ub=self.B, bounds=self.D,
                          options=dict(disp=Verbose, tol=1e-12, presolve=True, lstsq=True), method=lp_method)
            else:
                res = linprog(self.C, A_ub=self.A, b_ub=self.B, bounds=self.D, options=dict(disp=Verbose, tol=1e-12),
                          method=lp_method)

            # Commented from original self.solutions.append(list(res.x))
            return list(res.x)
        except TypeError:
            raise LPOptimizationFailedException()
        except Exception:
            raise LPGeneralException('Exception occurred while LP solving')

    #def append_constraint(self, s, point, slack):
    def calculate_constraints(self, s, point, slack):
        """INPUTS:
            s (object of class Spec)
            point (list): the counterexample point that is to be added as a constraint to the LP
            slack (float): the slack variable
        """
        lies = list(s.lie)
        lie_computed = []
        all_vars = list(s.lie)
        lie_computed = [-float(lies[i].subs(zip(all_vars, point))) for i in range(0, s.p)]  # evaluate all lie derivatives at the given point
        #Commented from original lie_computed.append(-float(lies[i].subs(zip(all_vars, point))))
        An0 = lie_computed[0:s.p - 1]
        An1 = An0 + [-1.0, 0.0]
        An2 = list(-np.array(An0)) + [1.0, -1.0]
        Bn1 = lie_computed[s.p - 1]
        Bn2 = -Bn1
        # positivity constraint d_0*fi(x=x_0) + d_p > 0
        dic = {s.state[i]: s.init_cond_symbols[i] for i in range(0, int(s.n / 2))}
        fi_init_cond = lies[0].subs(dic)
        fi_init_cond_computed = float(fi_init_cond.subs(zip(all_vars, point)))
        An3 = list(np.zeros(s.p + 1, ))
        An3[0] = -fi_init_cond_computed
        An3[-2] = -1
        Bn3 = 0

        # Commented from original self.A.append(An1) self.A.append(An2) self.B.append(Bn1 - 2 * slack)
        # self.B.append(Bn1-1.1*slack)
        # Commented from original self.B.append(Bn2) self.A.append(An3)  # for positivity self.B.append(Bn3)  # for positivity
        return [An1, An2, An3], [Bn1 - 2 * slack, Bn2, Bn3]


def upper_diagonal_ones(i, j):
    """Helps to create a matrix in controllable canonical form.
    Will serve to create the Ad matrix of the linear model"""
    if j - i == 1:
        return 1
    else:
        return 0


def polyhedral_cone_vertices2hrep(vertices):
    """INPUTS: vertices (list of lists)

    Given the vertices, returnes the b and A matrix of the halfspace rep.
    of a polyhedral cone. Uses the cdd library.
    """
    cdd_vertices = []
    for i in range(0, np.shape(vertices)[0]):  # put a 0 in front of all vertices
        # this constructs the vertices of th evertex rep for cdd
        temp = vertices[i][:]
        temp.append(1)
        temp.insert(0, 1)
        cdd_vertices.append(temp)
    for i in range(0, np.shape(vertices)[0]):  # put a 1 in front of all vertices
        # this constructs the rays of the vertex rep for cdd
        temp = vertices[i][:]
        temp.append(1)
        temp.insert(0, 0)
        cdd_vertices.append(temp)
    mat = cdd.Matrix(cdd_vertices, number_type='float')
    mat.rep_type = cdd.RepType.GENERATOR
    poly = cdd.Polyhedron(mat)
    ext = poly.get_inequalities()
    b = np.zeros(np.shape(ext)[0])
    A = np.zeros([np.shape(ext)[0], np.shape(ext)[1] - 1])
    for i in range(0, np.shape(ext)[0]):
        b[i] = ext[i][0]
        for j in range(0, np.shape(ext)[1] - 1):
            A[i, j] = -ext[i][j + 1]
    return [b, A]


def polytope_vrep2hrep(vertices):
    """INPUTS: vertices (list of lists)

    Given the vertices, returnes the b and A matrix of the halfspace rep.
    of a polytope. Uses the cdd library.
    """
    cdd_vertices = []
    for i in range(0, np.shape(vertices)[0]):  # put a 0 in front of all vertices
        # this constructs the vertices of the vertex rep for cdd
        temp = vertices[i][:]
        temp.insert(0, 1)
        cdd_vertices.append(temp[:])
    mat = cdd.Matrix(cdd_vertices, number_type='float')
    mat.rep_type = cdd.RepType.GENERATOR
    poly = cdd.Polyhedron(mat)
    ext = poly.get_inequalities()
    b = np.zeros(np.shape(ext)[0])
    A = np.zeros([np.shape(ext)[0], np.shape(ext)[1] - 1])
    for i in range(0, np.shape(ext)[0]):
        b[i] = ext[i][0]
        for j in range(0, np.shape(ext)[1] - 1):
            A[i, j] = -ext[i][j + 1]
    return [b, A]
