# -*- coding: utf-8 -*-
import numpy as np
import math
import sympy as sympy
import itertools
from scipy.optimize import linprog
import control_system_abstractions.nonlinear_systems_utils.dReal_communication_2 as dReal
import control_system_abstractions.nonlinear_systems_utils.dReach_communication as dReach
import control_system_abstractions.nonlinear_systems_utils.flowstar_communication as flowstar
from control_system_abstractions.exceptions.nonlinear_systems_exceptions.data_object_exceptions import \
    DataObjectGeneralException
from control_system_abstractions.nonlinear_systems_utils.auxilliary_data import *
from scipy.linalg import expm
from numba import prange
import itertools
from collections import namedtuple
#import cdd

"""Know issues: each variable name cannot be a part of another variable name: variables x and x0 will cause
problems, as x0 contains x. x1 and x0 is a proper naming"""

"""
    Abstractions Tool V2:
    -Creates Traffic Abstractions of Nonlinear Systems with model uncertainties and disturbances.
    -In the case of Homogeneous systems, it partitions the state-space into regions delimited by isochronous manifolds
        and cones (the number of which is specified by the user).
    -In the case of Nonhomogeneous systems, it partitions the state space into hypercubes of equal dimensions.
"""



class InputDataStructureNonLinear(object):
    """
    Attributes:
        path (string): Location where auxilliary files are stored
        dreal_path (string): Locatin of dReal
        dreach_path (string): Location of dReach
        flowstar_path (string): Location of flowstar
        Homogeneity_degree (float): Homogeneity degree of the system
        Homogenization_flag (boolean): True if system has been homogenized, false
                                        otherwise
        Dynamics (symbolic matrix): User defined dynamics
        Lyapunov_function (symbolic expression): Lyapunov function of the system
                                                (if given)
        Lyapunov_lvl_set_c (float): Constant that defines the Lyapunov level set
                                    (if given)
        State (tuple): User defined state vector
        Original_State (tuple): State vector without the homogenization variable
                                andcorresponding measurement error
        Original_Dynamics (symbolic matrix): Dynamics without the homogenization
                                    variable andcorresponding measurement error
        Init_cond_symbols (tuple): Symbols denoting initial conditions for state
                                variables (e.g. x0, y0, w0)
        n (int): state dimension, with measurement errors
        n_init_cond (int): initial condition vector dimension
        Parameters (tuple): unknown parameters in the dynamics (disturbances,
                            uncertainties, etc.) (e.g. (d1,d2))
        Parameters_domain (list of lists): list of intervals where each parameter
                                            belongs (e.g. [[-1,1],[-2,2]])
        Function (symbolic expression): the (triggering) function
        Init_cond_domain (list of lists): box where the initial condition belongs,
                                    for the feasibility problem of the deltas
        Symbolic_variables (tuple): All symbolic variables escept for measurement
                                    errors
        Symbolic_Box_of_Initial_Conditions (symbolic expression): Init_cond_domain
                                            written in a symbolic expression
        Symbolic_Domain_of_Parameters (symbolic expression): Parameters_domain
                                                written in a symbolic expression
        p (int): order of approximation of the triggering function.
        Symbolic_Domain: Box of initial conditions in sympy logic format
        p (int): order of the approximation. p= 1 corresponds to bounding the 0th
                                            lie derivative. Must be larger than 0
        Lie (symbolic matrix): Lie derivatives of self.function up to order self.p
        Gridstep (int): number of points per state dimension, used for creating
              an initial sample set in the box Init_cond_domain. Recommended >= 2
        Samples (List of lists): List of all the samples from the box domain used
                                                           in the LP optimization
        Deltas (list): The deltas found after the construction of a UBF.
                Synthesized by create_upper_bound. 'None' if no UBF is constructed
        Gamma (float): The bound on the infinity norm of the error
                                                    (over the sampled states)
        UBF (symbolic expression): The symbolic UBF. Synthesized by create_upper_bound. 'None' if no UBF is constructed.
        Lie_n (symbolic expression): The p-1th Lie derivative of the function f,
                                                which is bounded by the UBF
        LP_data (LPdata): LP object created within create_upper_bound
        Spherical_Domains (list of objects): This is created only if the system
                        has not been homogenized. list of Spherical_Domain objects
        cones_big_angle (list of objects): List of Cone_on_Plane objects, for the
                                               big angle of the coordinate system
        cones_small_angles (list of lists of objects): Contains a list of
                   Cone_on_Plane objects, for each secondary angle of the
                                                           coordinate system.
        Regions (list of objects): List of Region_Homogeneous or Region_NonHomogeneous
                                                                        objects.
        Grid (object): Created only if the system has been homogenized.
        origin_neighbourhood_degeneracy_flag (boolean): If true, means that due to
            the triggering function provided, the manifolds are degenerate
            in a neighbourhood around the origin (i.e. when phi(0)=0). If so, later on
            the timing lower bound of this region is enforced apriori as the heartbeat.
    Methods

    """

    # constructor
    def __init__(self, path, dreal_path, dreach_path, flowstar_path, dynamics, homogeneity, lyapunov, lvl_set_c,
                 function, state, init_cond_symbols, parameters, parameters_domain, hyperbox_states, order_approx,
                 gridstep, dreal_precision, heart_beat, manifolds_times, remainder_reachability,time_out_reachability,
                 grid_pts_per_dim, time_out_upperbounds, remainder_upper_bounds, timeout, homogenization_flag,
                 t_max=None, origin_neighbourhood_degeneracy_flag=True):
        """Constructor"""
        self.path = path
        self.dreal_path = dreal_path
        self.dreach_path = dreach_path
        self.flowstar_path = flowstar_path
        self.homogeneity_degree = homogeneity
        self.homogenization_flag = homogenization_flag
        self.dynamics = dynamics
        self.lyapunov_function = lyapunov
        self.lyapunov_lvl_set_c = lvl_set_c
        self.state = state
        if self.homogenization_flag:  # If the system has been homogenized
            original_state_temp = ()  # the original state is the self.state, without the element len(self.state)/2-1 and
            original_dynamics_temp = []  # the last element of self.state (w and ew respectively).
            i = 0  # the original dynamics are the self.dynamics without the same elements of self.dynamics (w_dot, ew_dot)
            for var in self.state[:-1]:  # iterate over all variables of self.state (except the last one)
                # put all variables of self.state (except for the last one and the middle one self.state[int(len(self.state)/2)-1]
                # in the original_state vector. Do the same for self.dynamics.
                if (i != int(len(self.state) / 2) - 1):
                    original_state_temp = original_state_temp + (var,)
                    original_dynamics_temp.append(self.dynamics[i].subs(self.state[int(len(self.state) / 2) - 1], 1))
                i = i + 1
        else:  # if the system has not been homogenized the original state and dynamics coincide with self.state and self.dynamics
            original_state_temp = self.state
            original_dynamics_temp = self.dynamics
        self.original_state = original_state_temp
        self.original_dynamics = original_dynamics_temp
        self.init_cond_symbols = init_cond_symbols
        self.n = len(state)
        self.n_init_cond = len(init_cond_symbols)
        self.parameters = parameters
        self.parameters_domain = parameters_domain
        self.function = function
        self.init_cond_domain = None
        self.symbolic_variables = self.state[:int(len(self.state) / 2)] + self.init_cond_symbols + self.parameters
        self.symbolic_box_of_initial_conditions = None
        self.symbolic_domain_of_parameters = self.create_symbolic_domain_of_parameters()
        self.p = order_approx
        if self.p <= 0:
            self.p = 1
            print(
                "WARNING: p must be larger than 0, hence p = 1 is selected instead.\n p = 1 equals bounding the 0th lie derivative")
        self.lie = None
        self.gridstep = gridstep
        if self.gridstep < 2:
            print("WARNING: it is stronly advised to pick a gridstep equal or larger than 2")
        self.samples = None
        self.deltas = None
        self.gamma = None
        self.UBF = None
        self.lie_n = None
        self.LP_data = None
        self.spherical_domains = None
        self.cones_big_angle = None
        self.cones_small_angles = None
        self.regions = None
        self.grid = None
        self.origin_neighbourhood_degeneracy_flag = origin_neighbourhood_degeneracy_flag
        self.manifolds_times = manifolds_times
        self.dreal_precision = dreal_precision
        # self.max_Lie_difference = None
        self.heart_beat = heart_beat
        #self.manifold_times = manifold_times
        self.remainder_reachability = remainder_reachability
        self.time_out_reachability = time_out_reachability
        self.grid_pts_per_dim = grid_pts_per_dim
        self.hyperbox_states = hyperbox_states
        self.time_out_upperbounds = time_out_upperbounds
        self.remainder_upper_bounds = remainder_upper_bounds
        self.timeout = timeout


    def create_symbolic_domain_of_parameters(self):
        """Given the box domain of parameters self.parameters_domain, write it in a symbolic expression"""
        try:
            domain = True
            for i in range(0, len(self.parameters)):  # iterate along the self.parameters tuple
                # to write the box constraint for each parameter in a symbolic way
                domain = domain & (self.parameters[i] >= self.parameters_domain[i][0])
                domain = domain & (self.parameters[i] <= self.parameters_domain[i][1])
            return domain
        except Exception:
            raise Exception('Exception Occurred')

    def calculate_lie_derivatives(self):
        """Computes the Lie derivatives at time t=0 of the specified system in self.dynamics up to order self.p"""
        list_lie = [self.function]  # the 0th derivative is equal to self.function
        for i in range(1, self.p):
            # calculates the i-th derivative
            list_lie.append(((list_lie[i - 1].diff(sympy.Matrix(self.state)).transpose()) * self.dynamics)[0])
        dic = {}
        # creates a dictionary where each variable in self.state[int(len(self.state)/2):] (i.e. all error variables)
        # are substituted by the difference -self.state[i-int(len(self.state)/2)]+self.init_cond_symbols[i-int(len(self.state)/2)] (e.g. ex = x0-x1)
        #for i in range(int(len(self.state) / 2), int(len(self.state))):
        #    dic[self.state[i]] = -self.state[i - int(len(self.state) / 2)] + self.init_cond_symbols[
        #        i - int(len(self.state) / 2)]
        dict_temp = {self.state[i]: -self.state[i - int(len(self.state) / 2)] +
                                    self.init_cond_symbols[i - int(len(self.state) / 2)]
                     for i in range(int(len(self.state) / 2), int(len(self.state)))}

        for i in range(0, self.p):
            list_lie[i] = list_lie[i].subs(dict_temp)  # substitutes the dictionary in the computed lie derivatives
        return list_lie

    def discretize_init_cond_and_param_domain(self):
        """ Creates a set of sample points by discretizing self.init_cond_domain and self.parameters_domain,
        based on the gridstep."""
        dimensions = int(self.n / 2) + self.n_init_cond + len(self.parameters)

        # List that includes self.init_cond_domain and self.parameters_domain
        state_vars_and_pars_domain = self.init_cond_domain[:]
        # Append the elements of self.parameters_domain to state_vars_and_pars_domain
        state_vars_and_pars_domain_additional = [self.parameters_domain[i] for i in range(0, len(self.parameters))]
        state_vars_and_pars_domain.extend(state_vars_and_pars_domain_additional)

        # Create a list of lists containing discretization pts for each interval domain in state_vars_and_pars_domain
        discr = [np.linspace((state_vars_and_pars_domain[i - 1])[0], (state_vars_and_pars_domain[i - 1])[1],
                             self.gridstep)
                 for i in range(1, dimensions + 1)]

        return list(itertools.product(*discr))  # get all combos of elements of constructed lists as discretization pts.

    def create_symbolic_domain_init_cond(self):
        """ Given the domain of initial conditions in the form of intervals, construct a symbolic domain."""
        sym_dom = sympy.And()  # this is the symbolic domain which will be returned by the function
        sym_list = self.init_cond_symbols[:]
        full_dom = self.init_cond_domain[:]

        if self.homogenization_flag:  # if the system is homogenized
            # incorporate the auxilliary variable self.state[int(self.n/2)-1] (i.e. w) in the symbol list
            sym_list = sym_list + (self.state[int(self.n / 2) - 1],)
            full_dom.append(self.init_cond_domain[-1])

        for var in range(0, len(sym_list)):  # iterate along all symbols of initial conditions
            # for each symbol, write the corresponding interval in a symbolic expression and append it to sym_dom
            sym_dom = sympy.And(sym_dom,
                                sympy.And(sym_list[var] >= full_dom[var][0], sym_list[var] <= full_dom[var][1]))

        for var in self.state[:int(self.n / 2)]:
            # for each symbol, write a default big interval in a symbolic expression, for robustness purposes
            sym_dom = sympy.And(sym_dom, sympy.And(var >= -1e2, var <= 1e2))

        return sym_dom  # return the symbolic domain

    def create_upper_bound(self, dreal_precision=0.01, time_out=None, lp_method='revised simplex', C=[], D=[],
                           verbose=False):
        """Solves the feasibility problem, by solving iteratively the LP version of it and checking with dReal
        if the found solutions verify the constraints.

        optional argument:
            dreal_precision (rational): Precision used in dReal. Default = 0.01
            time_out: maximum time in seconds, after which the verification with dReal is canceled. Default = None
            lp-method: The LP method 'revised simplex' or 'interior-point'. Default =' revisedsimplex'
            C (numpy array): Cost function vector c of the LP problem
            D (tuple of tuples): Bounds on the delta's
            verbose (Boolean): Print LP solver information. Default = False
        """
        self.LP_data = LPData(self, C, D)  # Initialize LP data from the user specified data
        res = {'sat': False}  # when the solution is found we set res['sat'] = true
        res_flag = 0
        res_flag_final = 0
        iteration = 1
        while res['sat'] == False:  # iterate until the solution is found
            print("\n Starting iteration {}".format(iteration))
            res_flag = self.LP_data.LP_solve(lp_method, Verbose=verbose)  # first solve the LP
            if res_flag == -1:
                break
            self.deltas = self.LP_data.solutions[-1][:-1]  # these are the solutions found by the LP
            self.gamma = self.LP_data.solutions[-1][-1]
            print("Delta's: {}".format(self.deltas))
            print("Infinity norm: {}".format(self.gamma))
            # Construct the UBF given the set of obtained delta's
            self.construct_UBF()
            # Verify the condition using dReal
            res = self.verify_upper_bound_constraint(dreal_precision,
                                                     time_out=time_out)  # check if the found solutions verify the constraints
            if res['time-out'] == True:
                print(
                    "WARNING: Verification timed-out or other unexpected output. Please modify the time-out variable or adapt the specification")
                res_flag = -2
                break
            if res[
                'sat'] == False:  # if they dont verify the constraints append a new constraint employing the counterexample res['violation']
                # Set the value to lp-data
                self.LP_data.calculate_constraints(self, res['violation'], dreal_precision)
            if (self.LP_data.A[-1] == self.LP_data.A[-3]) and (self.LP_data.B[-1] == self.LP_data.B[-3]):
                # if the same counterexample as before is returned, then terminate
                print('ERROR: Same counterexample found by dReal. Terminating script.')
                res_flag = -3
                break
            if res['sat'] == True:  # if the solutions verify the constraint change the flag res_flag_final
                res_flag_final = 1
            iteration += 1

        if res_flag_final > 0:
            print('Valid bound found!')
            print('The deltas are:{}'.format(self.LP_data.solutions[-1][:-1]))
            return 1
        else:
            print("No solution has been found. Try different LP or dReal settings")
            return -1


    def construct_UBF(self):
        """ Construct symbolic upper bounding function (UBF),
        which we want to verify if it bounds l[-1], given delta's.
        If no delta's are specified, the delta's stored in the SPEC class are used.
        The UBF is a sum of delta'smultiplied by the lie derivatives self.lie[:-1].
        UBF = self.deltas[0]*self.lie[0] + self.deltas[1]*self.lie[1] + ... """
        try:
            deltas = self.deltas

            if self.p == 1:
                return deltas[-1]
            else:
                return (sympy.Matrix(deltas[0:self.p - 1]).transpose() * sympy.Matrix(self.lie[0:self.p - 1]))[0] + \
                           deltas[-1]
        except Exception:
            raise Exception('Exception occurred')

    def verify_upper_bound_constraint(self, dreal_precision, time_out=None):
        """ Verify if:
            1) the constructed UBF bounds the p-th Lie derivative (self.lie[-1]) using dReal
            2) the condition delta_0*self.lie[0].subs(dic)+delta_p > 0 is satisfied
            inside the domain self.Lyapunov_function <= self.lyapunov_lvl_set_c (if the system is not homogenized) or
            inside the domain self.function.subs(dic) <= 0, if the system is homogenized."""
        # iterate along the first half the list self.state to create a dictionary associating each symbol to its
        # initial condition symbol (e.g. dic[x1]=x0)
        dic = {self.state[i]: self.init_cond_symbols[i] for i in range(0, int(self.n / 2))}
        fi_initial_cond = self.lie[0].subs(dic)
        delta_0 = self.deltas[0]
        delta_p = self.deltas[-2]

        # the condition delta_0*self.lie[0].subs(dic)+delta_p > 0 written symbolically
        positivity_expr = (delta_0 * fi_initial_cond + delta_p > 0)

        if not self.homogenization_flag:    # if system is not homogenized
            # the expression to be verified is:
            expression = (self.lyapunov_function > self.lyapunov_lvl_set_c) | ((self.UBF - self.lie_n >= 0) & positivity_expr)
        else:                               # if the system is homogenized
            dic = {self.state[i]: self.init_cond_symbols[i - int(self.n / 2)] - self.state[i - int(self.n / 2)]
                   for i in range(int(self.n / 2), self.n)}
            # the expression to be verified is:
            expression = (self.function.subs(dic) > 0) | ((self.UBF - self.lie_n >= 0) & positivity_expr)
        # can put in the logic
        return dReal.dReal_verify(expression, self.symbolic_box_of_initial_conditions,
                                  self.symbolic_domain_of_parameters, self.symbolic_variables, dreal_precision,
                                  self.dreal_path, self.path, time_out=time_out, verbose=True)

    def cone_matrices(self, nr_cones_small_angles, nr_cones_big_angle):
        """Given the number of cones corresponding to the small angles
        of the coordinate system (nr_cones_small_angle list of int)
        and the number of cones for the big angle (nr_cones_big_angle int),
        it discretizes each angular coordinate into a conic partition, creating a list of Cone_on_Plane
        objects (i.e. planar cones) for the big angle (i.e. self.cones_big_angle)
        and a list of lists containing Cone_on_Plane objects for each small angle (i.e. self.cones_small_angles)."""
        dimension = self.n
        # Isotropic covering
        big_angle_discr = np.linspace(0, math.pi,
                                      int(nr_cones_big_angle / 2 + 1))  # discretize the big angle from 0 to pi.
        small_angles_discr = []
        for i in range(1, int(
                dimension / 2 - 1)):  # there are int(dimension/2-1) small angles for a coordinate system of dim = dimension/2.
            # discretize each small angle from 0 to pi/2
            small_angles_discr.append(np.linspace(0, math.pi / 2, int(nr_cones_small_angles[i - 1] / 2 + 1)))

        self.cones_big_angle = []
        for i in range(0, len(
                big_angle_discr) - 1):  # iterate along all discrete angles in which the big angle was discretized
            a = big_angle_discr[i]
            b = big_angle_discr[i + 1]
            self.cones_big_angle.append(ConeOnPlane(a, b, True))  # create the planar cone from angle a to angle b
        for i in range(0, len(
                big_angle_discr) - 1):  # iterate along all discrete angles in which the big angle was discretized
            a = big_angle_discr[i]
            b = big_angle_discr[i + 1]
            self.cones_big_angle.append(
                ConeOnPlane(a, b, False))  # create the planar cone from angle pi+a to angle pi+b

        if (dimension / 2 > 2):  # in this case there are small angles as well (in case dimension = 2, there are not)
            self.cones_small_angles = []
            for i in range(0, int(dimension / 2) - 2):  # for each small angle
                temp = []
                for j in range(0, len(small_angles_discr[
                                          i]) - 1):  # iterate along all discrete angles in which the i-th small angle was discretized
                    a = small_angles_discr[i][j]
                    b = small_angles_discr[i][j + 1]
                    temp.append(ConeOnPlane(a, b, True))  # create the planar cone from angle a to angle b
                for j in range(0, len(small_angles_discr[
                                          i]) - 1):  # iterate along all discrete angles in which the i-th small angle was discretized
                    a = small_angles_discr[i][j]
                    b = small_angles_discr[i][j + 1]
                    temp.append(ConeOnPlane(a, b, False))  # create the planar cone from angle pi+a to angle pi+b
                self.cones_small_angles.append(temp)

    def check_cone_degeneracy(self, cone):
        """INPUT:
           cone: list of Cone_on_Plane objects.

        Given a combination of planar cones (i.e. Cone_on Plane objects), check if the n-dimansional cone
        defined by all of them is degenerate (return True) or not (return False)."""
        A = np.zeros((int(self.n / 2) - 1, int(self.n / 2)))
        for i in range(1, len(cone)):  # iterate along all planar cones
            A[i][i] = round((cone[i].linear_Form[0]), 5)
            A[i][i + 1] = round((cone[i].linear_Form[1]), 5)
            # print(np.array(A[i])+np.array(A[i-1]))
            if (
            np.all(np.array(A[i]) + np.array(A[i - 1]) == 0)):  # check if a linear constraint between the i-th and the
                # i+1th planar cone is the same. If it is, then the n-dimension cone is degenerate.
                return True  # so return True
        return False  # else return False

    def create_grid(self, state_space_limits=None, grid_points_per_dim=None):
        """INPUT:
           state_space_limits: list of symmetric intervals (e.g. [[-1,1],[-2,2],[-1,1]])
           grid_points_per_dim: list of odd int (e.g. [3,5,7])

        Creates a Grid object, given the limits of the state-space (state_space_limits)
        and the number of grid points per dimension ."""

        if (grid_points_per_dim is None) | (state_space_limits is None):
            raise DataObjectGeneralException("Error. You should either manualy insert a grid, or specify number of hypercubes and limits of the state-space.")

        dimension = int(self.n / 2) - 1
        linspace_list = []
        for i in range(0, dimension):  # for each dimension (i.e. each interval state_space_limits[i])
            lim_min = state_space_limits[i][0]  # create a linspace grid from
            lim_max = state_space_limits[i][1]  # state_space_limits[i][0] to state_space_limits[i][1]
            # with total number of grid points = grid_points_per_dim
            linspace_list.append(list(np.linspace(lim_min + (lim_max - lim_min) / grid_points_per_dim[i] / 2,
                                                  lim_max - (lim_max - lim_min) / grid_points_per_dim[i] / 2,
                                                  grid_points_per_dim[i])))

        # the combinationd of grid points for each dimension are the centers of the created n-dimensional grid
        centroids = list(itertools.product(*linspace_list))
        grid = GridObject(centroids[:], state_space_limits[:], grid_points_per_dim[:])
        # Commented from original self.grid = grid
        return grid

    def check_radius(self, indicator, manifold_time, expression2, radius_check, dreal_precision, time_out,
                     verbose=True):
        """INPUT:
           indicator: string, either 'i' or 'o'
           manifold_time: float, >0
           expression2: symbolic expression
           radius_check: float >0
           dreal_precision: float >0, preferrably <=1e-1
           time_out: int, >0
           verbose: boolean

        Checks if the given radius (radius_check) inner- or outer-approximates (depending on the indicator=='i' or 'o')
        the manifold segment created by the intersection of a manifold of time manifold_time
        and the symbolic expression expression2(e.g. a conic segment)."""
        ##First compute the expression defining a manifold g(x), onto the given ball, ie. g(x) for |x|=radius_check
        r = self.init_cond_domain[1][1] - 0.00000000000000000001 * self.init_cond_domain[1][1]
        C = np.zeros(self.p, )
        C[0] = 1
        C = C.transpose()
        C = sympy.Matrix(C)
        C = C.transpose()
        A = np.zeros((self.p, self.p))
        for i in range(1, self.p):  # create the A matrix of the linear bound, which depends on the deltas
            A[i - 1][i] = 1
            A[-2][i - 1] = self.LP_data.solutions[-1][i - 1]

        linear_bound_init_cond = []
        linear_bound_init_cond.append(self.lie[0])
        for i in range(1, len(self.lie) - 1):
            linear_bound_init_cond.append(self.lie[i] / 2 + abs(self.lie[i]) / 2)
        dic = {}
        for i in range(0, int(self.n / 2)):  # create a dictionary associating each symbol in Init_cond_symbols
            dic[self.init_cond_symbols[i]] = self.state[i]  # to the corresponding of self.state (i.e. dic['x0']=x1)
        for i in range(0, len(
                linear_bound_init_cond)):  # substitute the dictionary into all entries of linear_bound_init_cond
            linear_bound_init_cond[i] = sympy.simplify(linear_bound_init_cond[i].subs(dic))
        dic = {}
        for i in range(0,
                       int(self.n / 2)):  # create a dictionary associating each symbol in the first half of self.state
            dic[self.state[i]] = float(r / radius_check) * self.state[
                i]  # to its projection on the circle of radius_check
        for i in range(0, len(
                linear_bound_init_cond)):  # substitute the new dictionary to all entries of linear_bound_init_cond
            linear_bound_init_cond[i] = sympy.simplify(linear_bound_init_cond[i].subs(dic))
        linear_bound_init_cond.append(self.LP_data.solutions[-1][-2])
        linear_bound_init_cond = sympy.Matrix(linear_bound_init_cond)

        exponential = expm(
            A * manifold_time * (radius_check / r) ** (self.homogeneity_degree))  # compute the symbolic matrix
        exponential = sympy.Matrix(exponential)  # exponential of the linear bound

        manifold = sympy.simplify(C * exponential * linear_bound_init_cond)  # compute the expression of the manifold
        manifold = manifold[0]

        if (indicator == 'i'):  # expression is the symbolic expression to be verified
            expression = (manifold <= -1e-8)  # this denotes an inner-approximation
        else:
            expression = (manifold >= 1e-8)  # this denotes an outer approximation
        # Verify if the given radius inner/outer approximates the manifold
        res = dReal.dReal_verify(expression, expression2, None,
                                 self.symbolic_variables[:int(len(self.symbolic_variables) / 2)], dreal_precision,
                                 self.dreal_path, self.path, 'aekara.smt2', time_out=time_out, verbose=verbose)
        if res['time-out'] == True:
            print(
                "WARNING: Verification timed-out or other unexpected output. Please modify the time-out variable or adapt the specification")
            return -2
        if res['sat'] == False:  # this means that the given radius DOES NOT inner/outer approximate the manifold
            return -1
        if res['sat'] == True:  # this means that the given radius inner/outer approximates the manifold
            return 1

    def radius_conic_section(self, indicator, conic_domain, manifold_time, starting_radius=None, verbose=False):
        """INPUT:
               indicator: string, either 'i' or 'o'
               conic_domain: symbolic expression
               manifold_time: float, >0
               starting_radius: float, >0
               verbose: boolean

           RETURNS:
               radius_check: float >0, the radius that inner/outer approximates the manifold

        Finds radius (radius_check) that inner-/outer-approximates (depending on indicator) manifold of time manifold_time,
        in the conic section conic_domain."""
        # First, if there is no given starting radius, we find a starting_radius
        # that inner/outer approximates the manifold (then we will refine it)
        state_vector = self.state[:int(self.n / 2)]
        temp = 0
        for i in range(0, int(self.n / 2)):
            temp = temp + state_vector[i] ** 2
        if (starting_radius == None):
            if (indicator == 'i'):  # if we are looking for inner-approximations
                starting_radius = 20  # start from a big radius and then decrease it iteratively until it inner-approximates the manifold
            elif (indicator == 'o'):  # if we are looking for outer-approximations
                starting_radius = 0.1  # start from a small radius and then increase it iteratively until it outer-approximates the manifold
            while (True):  # iterate until we find a radius that inner/outer approximates the manifold
                # check if the radius inner/outer approximates the manifold
                expression = (temp - (0.99 * starting_radius) ** 2 >= 0) & (
                            temp - (1.01 * starting_radius) ** 2 <= 0) & conic_domain
                res = self.check_radius(indicator, manifold_time, expression, starting_radius, 0.0001, 100,
                                        verbose=verbose)
                if (res == 1):
                    break
                elif (res == -1):
                    # increase or decrease the radius
                    if (indicator == 'i'):
                        starting_radius = starting_radius / 1.5
                    elif (indicator == 'o'):
                        starting_radius = starting_radius * 1.5
                else:
                    print("Error in trying to find radius...")
                    return -1

        radius_check = starting_radius  # we have found a radius that inner/outer approximates the manifold
        # refine it to make it tighter
        while True:
            if (indicator == 'i'):  # if we are looking for inner-approximations
                radius_check = radius_check * 1.01  # slowly increase the starting radius until it does not inner-approximate the manifold
            elif (indicator == 'o'):  # if we are looking for outer-approximations
                radius_check = radius_check / 1.01  # slowly decrease the starting radius until it does not outer-approximate the manifold
            expression2 = (temp - (0.99 * radius_check) ** 2 >= 0) & (
                        temp - (1.01 * radius_check) ** 2 <= 0) & conic_domain
            res = self.check_radius(indicator, manifold_time, expression2, radius_check, 0.001, 100, verbose=verbose)
            if (res == -1):  # if we find a radius that does not inner/outer-approximate the manifold
                # pick the radius of the previous iteration as the solution (which was the last one to inner/outer-approximate the manifold) and terminate
                if (indicator == 'i'):
                    radius_check = radius_check / 1.01
                elif (indicator == 'o'):
                    radius_check = radius_check * 1.01
                if (verbose):
                    print('We have found radius = {} for the cone: {}'.format(radius_check, conic_domain))
                break
        return radius_check

    def spherical_sectors(self, manifold_time, nr_cones_small_angles=[], nr_cones_big_angle=5, verbose=True):
        """INPUT:
            manifold_time: float >0
            nr_cones_small_angles: list of positive int, of length self.n-2
            nr_cones_big_angle: int, >0

        First, creates a conic partition of the state space into isotropic cones, by calling
        the function self.cone_matrices.

        Then, approximates the partitions defined by the intersections of the isochronous manifold
        of time = manifold_time with the previously defined cones,
        by spherical sectors. Firstly two balls are computed: small_ball which inner-approximates a manifold,
        and big_ball which outer-approximates a manifold. Then, for each conic partition the radii of the
        balls are optimized, such that the spherical sectors defined by the radii and the cones
        better inner- and outer-approximate the conic partitions.

        Finally creates the corresponding Spherical_Domain objects.
        It is called by the function self.abstraction().
        """
        #self.spherical_Domains = []
        state_vector = self.state[:int(self.n / 2)]
        dimension = self.n
        self.cone_matrices(nr_cones_small_angles, nr_cones_big_angle)  # partition each angular coordinate into
        # discrete angles that define planar cones

        # create the n-dimensional conic partition by taking combinations of planar cones from each angular coordinate
        if (dimension / 2 > 2):
            temp = self.cones_small_angles[
                   :]  # create a list containing the lists of planar cones for each angular coordinate
            temp.insert(0, self.cones_big_angle)
            # combine the planar cones from each coordinate to derive n-dimensional cones
            all_combinations_of_cone_matrices = list(itertools.product(*temp))
        else:  # if dimension/2 = 2, then there is only one angular coordinate, the big angle.
            self.cones_small_angles = [[()]]  # the list of cones for each small angle is then empty
            temp = self.cones_small_angles[:]
            temp.insert(0, self.cones_big_angle)
            all_combinations_of_cone_matrices = list(itertools.product(*temp))

        # Calculate a radius that inner/outer approxiamtes the whole manifold (in all conic sections)
        # THis is done only if the system is not homogenized.
        # This will be the starting radius for the line search for each conic section.
        if not self.homogenization_flag:    #(self.homogenization_flag == False):
            temp = 0
            for i in range(0, int(dimension / 2)):
                temp = temp + state_vector[i] ** 2  # temp = x^2+y^2+z^2+...
            small_ball = 20  # start from a big radius and then decrease it iteratively until it inner-approximates
            # the manifold
            while True:  # iterate until small_ball inner-approximates the manifold
                expression2 = (temp - (0.99 * small_ball) ** 2 >= 0) & \
                              (temp - (1.01 * small_ball) ** 2 <= 0)  # this is the expression that defines the ball
                # check if small_ball inner-approximates the manifold
                res = self.check_radius('i', manifold_time, expression2, small_ball, 0.001, 100, verbose=verbose)
                if (res == 1):  # if it does, then proceed
                    print("Inner ball radius = %f" % (small_ball))
                    break
                elif (res == -1):  # if it doesn't, decrease and iterate again
                    small_ball = small_ball / 1.4
                else:
                    print("Error in trying to find small_ball...")
                    return -1

            big_ball = 0.1  # start from a small radius and then increase it iteratively until it outer-approximates
            # the manifold
            while True:  # iterate until big_ball outer-approximates the manifold
                expression2 = (temp - (0.99 * big_ball) ** 2 >= 0) & \
                              (temp - (1.01 * big_ball) ** 2 <= 0)  # this is the expression that defines the ball
                # check if big_ball outer-approximates the manifold
                res = self.check_radius('o', manifold_time, expression2, big_ball, 0.001, 100, verbose=verbose)
                if (res == 1):  # if it does, then proceed
                    print("Outer ball radius = %f" % (big_ball))
                    break
                elif (res == -1):  # if it doesn't, increase and iterate again
                    big_ball = big_ball * 1.4
                else:
                    print("Error in trying to find small_ball...")
                    return -1

        # Refine the obtained small_ball and big_ball for each conic section of the manifold
        for cone in all_combinations_of_cone_matrices:  # iterate along all n-dimensional cones
            # in order to consider each conic section of the manifold
            flag = True
            if (dimension / 2 > 2):  # if dimension>=3, check if the considered cone is degenerate
                flag = self.check_cone_degeneracy(cone)
            if (flag):  # if the cone is empty, delete it and move on to the next cone
                print('deleted')
            else:
                conic_domain = True  # this is going to be the symbolic expression defining the conic domain
                for i in range(0, int(
                        dimension / 2) - 1):  # create iteratively the symbolic expression for the conic domain
                    # by iteratively taking conic_domain = conic_domain & [x_i x_{i+1}]*Q_i*[x_i x_{i+1}]^T & L_i*[x_i x_{i+1}]
                    # where Q_i and L_i are the quadratic and linear forms that define the cone, respectively
                    vec = sympy.Matrix([state_vector[i], state_vector[i + 1]])
                    quadratic_form = sympy.Matrix(cone[i].quadratic_Form)
                    conic_domain = conic_domain & ((vec.transpose() * quadratic_form * vec)[0] >= 0)
                    linear_form = sympy.Matrix(cone[i].linear_Form)
                    conic_domain = conic_domain & ((linear_form.transpose() * vec)[0] >= 0)
                # refine the inner-approximation radius
                inner_radius = self.radius_conic_section('i', conic_domain, manifold_time, small_ball, verbose)
                # refine the outer-approximation radius
                outer_radius = self.radius_conic_section('o', conic_domain, manifold_time, big_ball, verbose)
                # create the Spherical_Domain object and append it to self.spherical_domains
                self.spherical_domains.append(SphericalDomain(manifold_time, cone, inner_radius, outer_radius))

    def create_regions_objects(self, manifolds_times):
        """INPUTS:
            manifolds_times: list of int, >0

        Creates the object Regions_Homogeneous (for homogeneous systems), using the manfifolds of times=manifolds_times and the
        self.spherical_domains object. For each region, it calculates the timing lower bound,
        and the inner and outer approximating radii.
        It is called by the function self.abstraction()."""
        self.regions = []

        region_index = [0, 0]  # index of each region
        for j in range(0, len(manifolds_times)):  # We iterate along all manifolds
            # Here we construct the Regions objects:
            # inner and outer manifolds of the regions, cones defining them, timing lower bounds
            # ball overapproximations (inner and outer radius)
            region_index[0] = region_index[0] + 1
            region_index[1] = 0
            time_low = manifolds_times[j]  # timing lower bound for the region
            lamda_outer_man = (manifolds_times[-1] / time_low) ** (1 / self.homogeneity_degree)  # the scaling between
            # the manifold of the spherical_domains (of time manifolds_times[-1]) and the outer manifold
            # of the region
            if (j == len(manifolds_times) - 1):  # if the region is the innermost one
                contains_origin_flag = True  # it contains the origin
                time_up = 10  # the time of its inner manifold is infinity (practically very large = 10)
                lamda_inner_man = 0
            else:  # else
                contains_origin_flag = False
                time_up = manifolds_times[j + 1]  # this is the time of its inner manifold
                lamda_inner_man = (manifolds_times[-1] / time_up) ** (
                            1 / self.homogeneity_degree)  # the scaling between
                # the manifold of the spherical_domains and the inner manifold of the region
            for element in self.spherical_domains:  # iterate along all cones
                # each iteration is a new region
                region_index[1] = region_index[1] + 1
                outer_manifold_time = time_low
                inner_manifold_time = time_up
                cone = element.cone
                inner_radius = lamda_inner_man * element.inner_radius
                outer_radius = lamda_outer_man * element.outer_radius
                # create the region object
                new_region = RegionHomogeneous(self.state[:int(self.n / 2)], region_index[:], inner_manifold_time,
                                                outer_manifold_time, cone, inner_radius, outer_radius,
                                                contains_origin_flag)
                temp = new_region
                self.regions.append(temp)

    def create_regions_objects_nonhomogeneous(self, manifolds_times, heartbeat):
        """INPUTS:
            manifolds_times: list of int, >0 (practically I only use manifolds_times[-1])

        Creates the objects Region_NonHomogeneous (for homogeneous systems), using
        the manfifold of time=manifolds_times[-1] and the created grid (by self.create_grid).
        """
        manifold_time = manifolds_times[-1]
        dimension = int(self.n / 2) - 1
        polytope_sides_lengths = []
        for i in range(0, dimension):  # for each dimension
            lim_min = self.grid.state_space_limits[i][0]  # calculate what is the displacement
            lim_max = self.grid.state_space_limits[i][1]  # from the center of a grid polytope
            side_length = (lim_max - lim_min) / self.grid.grid_points_per_dim[i]  # to its vertices
            polytope_sides_lengths.append([-side_length / 2, side_length / 2])
        # create a list with all combinations of displacement. each combination, when added to the center of
        # a polytope, it gives one of its vertices.
        differences_between_all_vertices_and_centroid = list(itertools.product(*polytope_sides_lengths))
        region_index = 0
        self.regions = []
        for centroid in self.grid.centroids:  # iterate along all polytopes, each of which representing a region
            region_index = region_index + 1
            polytope_vertices_in_rn = []
            # add each element of differences_between_all_vertices_and_centroid to the center of the region
            # to get its vertices
            for i in range(0, len(differences_between_all_vertices_and_centroid)):
                aux = np.array(centroid) + np.array(differences_between_all_vertices_and_centroid[i])
                aux = aux.tolist()
                polytope_vertices_in_rn.append(aux)
            [halfspaces_b, halfspaces_A] = polytope_vrep2hrep(polytope_vertices_in_rn)  # get the hrep
            #            aux2 = np.zeros(dimension)
            #            polytope_vertices.append(aux2.tolist())
            if all([v == 0 for v in centroid]) & self.origin_neighbourhood_degeneracy_flag:
                # if the polytope contains the origin and the manifolds there are degenerate
                lower_bound = self.timing_lower_bound_origin_polytope(heartbeat)
                temp = RegionNonHomogeneous(self.state[:dimension], region_index, centroid, polytope_vertices_in_rn,
                                             halfspaces_A, halfspaces_b, lower_bound, True)
            else:
                lower_bound = self.timing_lower_bounds_grid(polytope_vertices_in_rn, manifold_time)
                temp = RegionNonHomogeneous(self.state[:dimension], region_index, centroid, polytope_vertices_in_rn,
                                             halfspaces_A, halfspaces_b, lower_bound, False)
            print('Region {} timing lower bound = {}'.format(region_index, lower_bound))
            self.regions.append(temp)

    def timing_lower_bound_origin_polytope(self, lower_bound):
        """This is unfinished. Determines timing lower bound for the region containing the origin."""
        # I have to find a way to determine this, probably with MATI
        a = lower_bound
        return a

    def timing_lower_bounds_grid(self, polytope_vertices, manifold_time):
        """INPUTS:
            polytope_vertices: list of lists of float
            manifold_time: float, >0.

        Given the vertices of a polytope and a manifold of time manifold_time,
        it computes the timing lower bound for the region of the polytope.
        Returns the timing lower bound."""
        # First find a sphere that inner approximates the manifold
        # in the n-dimensional conic section defined by the n-1-dimensional polytope
        state_vector = sympy.Matrix(self.state[:int(self.n / 2)])
        [cone_b, cone_A] = polyhedral_cone_vertices2hrep(polytope_vertices)
        cone_A = sympy.Matrix(cone_A)
        cone_b = sympy.Matrix(cone_b)
        A_times_state = cone_A * state_vector
        symbolic_conic_domain = True
        for i in range(0, len(cone_b)):  # create the symbolic conic domain hrep
            symbolic_conic_domain = symbolic_conic_domain & (A_times_state[i] <= cone_b[i])
        # find radius that inner-approximates manifold in the conic domain
        inner_radius = self.radius_conic_section('i', symbolic_conic_domain, manifold_time, \
                                                 starting_radius=None, verbose=False)
        # Find the polytope vertex with the largest norm
        polytope_vertices[0].append(1)
        max_norm = np.linalg.norm(polytope_vertices[0])
        for i in range(1, len(polytope_vertices)):  # iterates along all vertices
            polytope_vertices[i].append(1)
            if (np.linalg.norm(polytope_vertices[i]) >= max_norm):
                max_norm = np.linalg.norm(polytope_vertices[i])
        # Compute timing lower bound based on distance between polytope and inner sphere,
        # using max_norm and scaling law
        lamda = max_norm / inner_radius
        lower_bound = manifold_time * lamda ** (-self.homogeneity_degree)
        return lower_bound

    def timing_upper_bounds(self, nr_manifolds, time_out=20, \
                            remainder_upper_bounds=1e-1, heartbeat=10, verbose=False):
        """INPUTS:
            nr_manifolds: int, >0. If the system has been homogenized it doesn't play a role.
            time_out: int, >0. Time out for the reachability analysis.
            heartbeat: float, >0. Ad-hoc timing upper bound.
            verbose: boolean.

            It computes timing upper bounds for all previously created Regions.
            In the case of Homogeneous systems, the number of manifolds nr_manifolds specifies
            how many manifolds have they been used for the partitioning of the state space.
            """
        if (self.homogenization_flag == True):  # For homogenized systems
            for region in self.regions:
                #                if all([v == 0 for v in region.center]): #for the region of the origin
                #                    region.insert_timing_upper_bound(heartbeat)
                #                else: #for all other regions use reashability
                upper_bound = self.reach_analysis_upper_bounds(region, verbose, time_out, remainder_upper_bounds,
                                                               heartbeat)
                region.insert_timing_upper_bound(upper_bound)
        else:
            ###For homogeneous systems (regions=conic sections of manifolds)
            if verbose:
                print('Computing timing upper bounds for all outer strips of regions')
            for region in self.regions:
                # First find upper bounds for the second innermost strip of regions
                if (region.index[0] == nr_manifolds - 1):
                    self.reach_analysis_upper_bounds(region, verbose)
            for region in self.regions:
                if (region.index[0] < nr_manifolds - 1):  # for each outer strip
                    for region2 in self.regions:
                        # find the one in the second strip that is in the same cone
                        if ((region.index[1] == region2.index[1]) & (region2.index[0] == nr_manifolds - 1)):
                            # use the scaling law to scale its timing upper bound
                            lamda = (region2.outer_manifold_time / region.outer_manifold_time) ** (
                                        1 / self.homogeneity_degree)
                            region.insert_timing_upper_bound(
                                lamda ** (-self.homogeneity_degree) * region2.timing_upper_bound)

            if verbose:
                print('Declaring timing upper bounds for the inner regions that contain the origin')
            for region in self.regions:
                if (region.index[0] == nr_manifolds):
                    for region2 in self.regions:
                        # For the first strip of regions
                        if ((region.index[1] == region2.index[1]) & (region2.index[0] == nr_manifolds - 1)):
                            # Self-declare upper bounds
                            region.insert_timing_upper_bound(1.5 * region2.timing_upper_bound)

    def reach_analysis_upper_bounds(self, region, verbose=False, time_out=None, \
                                    remainder_upper_bounds=1e-1, heartbeat=10):
        """INPUTS:
            regions: object of class Region_Homogeneous or Region_NonHomogeneous.
            verbose: boolean.
            time_out: int, >0.
            heartbeat: float, >0.

            Given a region, it computes its timing upper bound with reachability analysis.
        """
        tau = sympy.symbols('tau')
        upper_bound = 1.1 * region.timing_lower_bound  # first estimation of the upper bound
        dreal_precision = 0.0001
        while (True):  # iteratively check if the estimated upper_bound
            # is truly a timing upper bound. If it isn't, increase and iterate again.
            if (not self.homogenization_flag):
                goal_set = (tau >= upper_bound) & (tau <= 1.0001 * upper_bound) & (self.function <= 0)
            else:  # if system is homogenized, take out the 'w' variable
                goal_set = (tau >= upper_bound) & (tau <= 1.0001 * upper_bound) & \
                           (self.function.subs(self.state[int(len(self.state) / 2) - 1], 1) <= 0)
            if (self.parameters == ()):  # if no parameters (uncertainties, disturbances, etc.)
                # we use dReach
                # for dReach the initial set is given in the common symbolic format
                initial_set = region.symbolic_region_for_reachability_analysis & (tau >= 0) & (tau <= 0)
                for e in self.original_state[int(len(self.original_state) / 2):]:  # for all measurement errors
                    # initial condition is 0
                    initial_set = initial_set & (e >= 0) & (e <= 0)
                res = dReach.dReach_verify(initial_set, goal_set, 1.2 * upper_bound, self.original_state,
                                           self.original_dynamics, dreal_precision, self.dreach_path, self.path,
                                           "upper_bounds.drh", verbose, time_out)
            else:  # if there are parameters, use flowstar.
                # for flowstar the initial set in the form of intervals (list of lists)
                box_domain = region.region_box[:]
                for e in self.original_state[int(len(self.original_state) / 2):]:  # for all measurement errors
                    # initial condition is 0
                    box_domain.append([0, 0])
                res = flowstar.flowstar_verify(box_domain, goal_set, 1.0001 * upper_bound, self.original_state,
                                               self.original_dynamics, self.parameters, self.parameters_domain,
                                               self.flowstar_path, self.path, "upper_bounds.model", verbose, time_out,
                                               remainder_upper_bounds)
            if (not res['sat']):  # found upper bound
                print('Region {}: Upper Bound found: {}!'.format(region.index, upper_bound))
                break  # terminate
            if (res['time-out']):
                if (verbose):
                    print('dReach or flowstar time out or other unexpected result. Exiting...')
                    print(
                        'flowstar cannot verify this time, due to computational complexity or inexistence of timing upper bound.')
                print('Region {}: Enforcing heartbeat as upper bound: {}'.format(region.index, heartbeat))
                upper_bound = heartbeat
                break
            if (verbose):
                print('Trying upper bound:', 1.05 * upper_bound)
            upper_bound = 1.05 * upper_bound  # increase upper bound
            if (upper_bound >= heartbeat):  # if estimate is bigger than ad-hoc heartbeat
                upper_bound = heartbeat  # enforce heartbeat as upper bound and terminate
                print('Region {}: Enforcing heartbeat as upper bound: {}'.format(region.index, upper_bound))
                break
        region.insert_timing_upper_bound(upper_bound)
        return upper_bound

    def reach_analysis_lower_bounds(self, region, verbose=False, time_out=None, heartbeat=10):
        """INPUTS:
            regions: object of class Region_Homogeneous or Region_NonHomogeneous.
            verbose: boolean.
            time_out: int, >0.
            heartbeat: float, >0.

            Given a region, it computes its timing upper bound with reachability analysis.
        """
        tau = sympy.symbols('tau')
        lower_bound = heartbeat  # first estimation of the lower bound
        dreal_precision = 0.001
        while (True):  # iteratively check if the estimated upper_bound
            print(lower_bound)
            # is truly a timing upper bound. If it isn't, increase and iterate again.
            if not self.homogenization_flag:
                goal_set = (tau >= 0) & (tau <= lower_bound) & (self.function >= 0)
            else:  # if system is homogenized, take out the 'w' variable
                goal_set = (tau >= 0) & (tau <= lower_bound) & \
                           (self.function.subs(self.state[int(len(self.state) / 2) - 1], 1) >= 0)
            if self.parameters == ():  # if no parameters (uncertainties, disturbances, etc.)
                # we use dReach
                # for dReach the initial set is given in the common symbolic format
                initial_set = region.symbolic_region_for_reachability_analysis & (tau >= 0) & (tau <= 0)
                for e in self.original_state[int(len(self.original_state) / 2):]:  # for all measurement errors
                    # initial condition is 0
                    initial_set = initial_set & (e >= 0) & (e <= 0)
                res = dReach.dReach_verify(initial_set, goal_set, 1.01 * lower_bound, self.original_state,
                                           self.original_dynamics, dreal_precision, self.dreach_path, self.path,
                                           "upper_bounds.drh", verbose, time_out)
            else:  # if there are parameters, use flowstar.
                # for flowstar the initial set in the form of intervals (list of lists)
                box_domain = region.region_box[:]
                for e in self.original_state[int(len(self.original_state) / 2):]:  # for all measurement errors
                    # initial condition is 0
                    box_domain.append([0, 0])
                res = flowstar.flowstar_verify(box_domain, goal_set, 1.01 * lower_bound, self.original_state,
                                               self.original_dynamics, self.parameters, self.parameters_domain,
                                               self.flowstar_path, self.path, "upper_bounds.model", verbose, time_out)
            if (not res['sat']):  # found upper bound
                print('Region {}: Lower Bound found: {}!'.format(region.index, lower_bound))
                break  # terminate
            # =============================================================================
            #             if (res['time-out']):
            #                 if (verbose):
            #                     print('dReach or flowstar time out or other unexpected result. Exiting...')
            #                     print('flowstar cannot verify this time, due to computational complexity or inexistence of timing lower bound.')
            #                 print('Region {}: Enforcing heartbeat as lower bound: {}'.format(region.index,heartbeat))
            #                 lower_bound = heartbeat
            #                 break
            # =============================================================================
            if (verbose):
                print('Trying upper bound:', 0.9 * lower_bound)
            lower_bound = 0.9 * lower_bound  # increase upper bound
        return lower_bound

    def transitions(self, verbose, time_out=None, remainder_reach=1e-1):
        """INPUTS:
            verbose: boolean.
            time_out, float, >0.

            Computes transitions for all Regions.
        """
        dreal_precision = 0.0001
        tau = sympy.symbols('tau')
        for region in self.regions:  # for each region
            for region2 in self.regions:  # check if there is transition with each of all regions
                goal_set = region2.symbolic_region_for_reachability_analysis
                goal_set = goal_set & (tau >= region.timing_lower_bound) & (tau <= region.timing_upper_bound)
                if (self.parameters == ()):  # use dreach if no parameters
                    # for dReach the initial set is given in the common symbolic format
                    initial_set = region.symbolic_region_for_reachability_analysis
                    for e in self.original_state[int(len(self.original_state) / 2):]:
                        initial_set = initial_set & (e >= 0) & (e <= 0)
                    res = dReach.dReach_verify(initial_set, goal_set, region.timing_upper_bound, self.original_state,
                                               self.original_dynamics, dreal_precision, self.dreach_path, self.path,
                                               "reach_analysis.drh", verbose, time_out)
                else:  # use flowstar if there are parameters
                    # for flowstar the initial set in the form of iintervals (list of lists)
                    box_domain = region.region_box[:]
                    for e in self.original_state[int(len(self.original_state) / 2):]:
                        box_domain.append([0, 0])
                    res = flowstar.flowstar_verify(box_domain, goal_set, region.timing_upper_bound, self.original_state,
                                                   self.original_dynamics, self.parameters, self.parameters_domain,
                                                   self.flowstar_path, self.path, "reach_analysis.model", verbose,
                                                   time_out, remainder_reach, )
                if (res['time-out']):  # if time out, enforce transition
                    print('dReach or flowstar time out. Enforcing Transition from Region {} to Region {}'.format(
                        region.index, region2.index))
                if (res['sat']):  # there is transition
                    print('Transition found from Region {} to Region {}'.format(region.index, region2.index))
                    region.insert_transition(region2.index)
