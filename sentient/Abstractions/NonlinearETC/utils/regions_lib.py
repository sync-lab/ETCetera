import numpy as np
import sympy

class Grid_obj(object):
    """Attributes:
        Centroids (list of lists of float): coordiantes of centers of polytopes of the grid.
        State_space_limits (list of lists of float): intervals for each dimension of the state space (operation region).
        Grid_points_per_dim (list of int): the number of points of discretization of each dimension.
    """

    def __init__(self, centroids, state_space_limits, grid_points_per_dim):
        self.Centroids = centroids
        self.State_space_limits = state_space_limits
        self.Grid_points_per_dim = grid_points_per_dim


class Region_Grid(object):
    """Attributes:
        transitions (list of lists): The transitions of the region.
        index(list of 2 elements): The index of the region.
        center (list of float): The center of the (polytopic) region.
        region_box (list of intervals): The intervals per dimension defining the region.
        hrep_A (list of lists, matrix): the A-matrix of the halfspace representation.
        hrep_b (list): the b-matrix of the halfspace representation.
        contains_origin (boolean)
        symbolic_domain_reach (sympy expression): The region set written in sympy expression.
        timing_lower_bound, timing_upper_bound (float)
    """

    def __init__(self, state_vector, region_index, centroid, polytope_vertices, polytope_halfspaces_A,
                 polytope_halfspaces_b, lower_bound, contains_origin_flag):
        self.transitions = []
        self.index = region_index
        self.timing_lower_bound = lower_bound
        self.center = centroid
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
        self.hrep_A = polytope_halfspaces_A
        self.hrep_b = polytope_halfspaces_b
        self.contains_origin = contains_origin_flag
        A = sympy.Matrix(polytope_halfspaces_A)
        b = sympy.Matrix(polytope_halfspaces_b)
        A_times_state = A * sympy.Matrix(state_vector)
        expression = True
        for i in range(0, len(b)):
            expression = expression & (A_times_state[i] <= b[i])
        self.symbolic_domain_reach = expression

    def insert_timing_upper_bound(self, upper_bound):
        self.timing_upper_bound = upper_bound
        if (self.timing_lower_bound > self.timing_upper_bound):
            self.timing_lower_bound = 0.8 * self.timing_upper_bound

    def insert_transition(self, trans):
        self.transitions.append(trans)


class Region_Manifold(object):
    """Attributes:
        transitions (list of lists): The transitions of the region.
        index(list of 2 elements): The index of the region.
        inner_manifold_time, outer-manifold_time (float): Time of the inner/outer manifold.
        cone (list of Cone_On_Plane objects): Conic domain.
        inner_radius, outer_radius (float): The radii that inner/outer-approximate the region.
        contains_origin (boolean)
        symbolic_domain_reach (sympy expression): Overapproximation of the region used for reach analysis.
        symbolic_domain (sympy expression): The actual region.
        timing_lower_bound, timing_upper_bound (float)
    """

    def __init__(self, state_vector, region_index, inner_manifold_time, outer_manifold_time, \
                 conic_domain, inner_radius, outer_radius, contains_origin_flag, homogenized):
        self.transitions = []
        self.index = region_index
        self.inner_manifold_time = inner_manifold_time
        self.outer_manifold_time = outer_manifold_time
        self.timing_lower_bound = outer_manifold_time
        self.inner_radius = inner_radius
        self.outer_radius = outer_radius
        self.contains_origin = contains_origin_flag
        temp = 0

        # construct symbolic expression of region
        for i in range(0, len(state_vector)):
            temp = temp + state_vector[i] ** 2  # temp = x^2+y^2+z^2+...
        expression = conic_domain & (temp - (inner_radius) ** 2 >= 0) & \
                     (temp - (outer_radius) ** 2 <= 0)
        # for i in range(0,len(state_space_limits())):
        #   expression = expression & (state_vector[i]>=state_space_limits[i,0]) & \
        #                          (state_vector[i]<=state_space_limits[i,1])
        if homogenized:
            expression = expression.subs(state_vector[-1], 1)
        self.symbolic_domain_reach = expression

        #if u wanna compute the symbolic expressions of the actual regions
        #u have to parse mu (which is computed via TrafficModelNonlinearETC.compute_mu
        # tau = sympy.symbols("tau")
        # expression2 = (mu.subs({tau: inner_manifold_time}) >= 0) & (mu.subs({tau: outer_manifold_time}) <= 0) \
        #               & conic_domain
        # if homogenized:
        #     expression2 = expression2.subs(state_vector[-1], 1)
        # self.symbolic_domain = expression2

    def insert_timing_upper_bound(self, upper_bound):
        "upper_bound: float, >0."
        self.timing_upper_bound = upper_bound

    def insert_transition(self, trans):
        "trans: list of 2 elemets."
        self.transitions.append(trans)