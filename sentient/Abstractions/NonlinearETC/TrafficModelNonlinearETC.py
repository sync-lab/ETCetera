
import numpy as np
import scipy
import math

import shortuuid
import sympy as sympy
# import symengine
from scipy.linalg import expm
import itertools
import copy
import tqdm
import logging
from functools import cached_property
from joblib import Parallel, delayed

from ..Abstraction import Abstraction
import sentient.Abstractions.NonlinearETC.utils.dReal_communication_2 as dReal
import sentient.Abstractions.NonlinearETC.utils.dReach_communication as dReach
import sentient.Abstractions.NonlinearETC.utils.flowstar_communication as flowstar
import sentient.Abstractions.NonlinearETC.utils.cones_lib as cones_lib
import sentient.Abstractions.NonlinearETC.utils.lp_lib as lp_lib
from sentient.Abstractions.NonlinearETC.utils.regions_lib import *
from sentient.exceptions import *
import sentient.util as util

from config import smt_path, dreach_path, dreal_path, flowstar_path

class TrafficModelNonlinearETC(Abstraction):
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
        Lie (symbolic matrix): Lie derivatives of self.Function up to order self.p
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
            the timing lower bound of this region is enforced apriori as small enough.

    """

    # constructor
    # def __init__(self, path, dreal_path, dreach_path, flowstar_path, dynamics, homogeneity, function,
    #              state, init_cond_symbols, parameters, parameters_domain,
    #              homogenization_flag=False, # Following are solver options
    #              precision_deltas=1e-4, timeout_deltas=1000, partition_method='grid',
    #              manifolds_times=None, nr_cones_small_angles=None, nr_cones_big_angle=None,
    #              state_space_limits=None, grid_points_per_dim=None, heartbeat=0.1,
    #              precision_timing_bounds=1e-3, precision_transitions=1e-3,
    #              timeout_timing_bounds=200, timeout_transitions=200, order_approx=2):
    def __init__(self, dynamics, trigger,
                 state, homogeneity=2, init_cond_symbols=None, dist_param=None, dist_param_domain=None,
                 homogenization_flag=False,  # Following are solver options
                 precision_deltas=1e-7, timeout_deltas=1000, partition_method='grid',
                 manifolds_times=None, angles_discretization=None,
                 state_space_limits=None, grid_points_per_dim=None, heartbeat=0.1,
                 precision_timing_bounds=1e-3, precision_transitions=1e-3,
                 timeout_timing_bounds=200, timeout_transitions=200, order_approx=4, parallel=False):
        '''
                    precision_deltas: float>0, dreal precision for manifold approximation algorithm
                    timeout_deltas: float>0, timeout for manifold approximation algorithm
                    partition_method: str
                    manifolds_times: list of float>0, the times of the manifolds to be used for partitioning
                                                    in the case of grid partition, insert one time
                                                    to be used as a reference manifold for timing lower bounds
                    nr_cones_big_angle: int > 0 for homogeneous systems, big angle of spherical coordinates is partitioned into this many pieces
                                                    (to create the cones for homogeneous systems)
                    nr_cones_small_angles: list of int > 0 for homogeneous systems, small angles of spherical coordinates are partitioned into this many pieces
                                                    (to create the cones for homogeneous systems)
                    state_space_limits: list of intervals, e.g. [[-1,1],[-2,4], [-1.5,3]], limits of the state space
                    grid_points_per_dim: list of int>0, - in case of grid partition, dictates griding for each dimension
                                                        - in case of manifold partition + nonhom system:
                                                            dictates how partition of the state space on the
                                                            w=1-plane into cubes, which then define the cones
                    heartbeat: float >0
                    precision_timing_bounds: float >0, dreal precision or flowstar remainder for reach analysis for timing bounds
                    precision_transitions: float >0, dreal precision or flowstar remainder for reach analysis for transitions
                    timeout_timing_bounds: float >0, timeout for reach analysis for timing bounds
                    timeout_transitions: float >0, timeout for reach analysis for transitions
        '''
        super().__init__()
        """Constructor"""
        dynamics = sympy.Matrix(dynamics)
        self.Homogenization_Flag = (sympy.symbols('w1') in dynamics.free_symbols) or homogenization_flag
        if len(dynamics) != len(state):
            dynamics = list(dynamics)
            dynamics += [-1 * expr for expr in dynamics]

        dynamics = sympy.Matrix(dynamics)
        hom_deg = util.test_homogeneity(dynamics, state)

        if hom_deg is None:
            print(f'Dynamics {dynamics} are not yet homogeneous.')
            print(f'Make Homogeneous with degree {homogeneity} (Default: 2)')

            # Make homogeneous (default: 2)
            dynamics, state, trigger = util.make_homogeneous_etc(dynamics, state, homogeneity or 2, trigger=trigger)
            self.Homogenization_Flag = True
            self.Homogeneity_degree = homogeneity
        elif hom_deg != homogeneity and homogeneity is not None:
            print('Specified degree of homogeneity does not correspond with the calculated one. Use specified.')

        self.Homogeneity_degree = homogeneity or hom_deg
        # self.Homogenization_Flag = homogenization_flag
        self.Dynamics = dynamics
        self.State = state
        if (self.Homogenization_Flag):  # If the system has been homogenized
            original_state = ()  # the original state is the self.State, without the element len(self.State)/2-1 and
            original_dynamics = []  # the last element of self.State (w and ew respectively).
            i = 0  # the original dynamics are the self.Dynamics without the same elements of self.Dynamics (w_dot, ew_dot)
            for var in self.State[:-1]:  # iterate over all variables of self.State (except the last one)
                # put all variables of self.State (except for the last one and the middle one self.State[int(len(self.State)/2)-1]
                # in the original_state vector. Do the same for self.Dynamics.
                # print(f"{i}: {var}")
                if (i != int(len(self.State) / 2) - 1):
                    original_state = original_state + (var,)
                    # print(f'idx: {int(len(self.State) / 2) - 1}')
                    original_dynamics.append(self.Dynamics[i].subs(self.State[int(len(self.State) / 2) - 1], 1))
                i = i + 1
        else:  # if the system has not been homogenized the original state and dynamics coincide with self.State and self.Dynamics
            original_state = self.State
            original_dynamics = self.Dynamics
        self.Original_State = original_state
        self.Original_Dynamics = original_dynamics

        if init_cond_symbols is None:
            init_cond_symbols = [sympy.Symbol(str(i).replace('x', 'a')) for i in original_state if 'x' in str(i)]
            if self.Homogenization_Flag:
                init_cond_symbols.append(sympy.Symbol('aw'))

            init_cond_symbols = tuple(init_cond_symbols)
        self.Init_cond_symbols = init_cond_symbols
        self.n = len(state)
        self.n_init_cond = len(init_cond_symbols)
        self.Parameters = dist_param or ()
        self.Parameters_domain = dist_param_domain
        self.Function = trigger
        self.Init_cond_domain = self.create_box_domain_for_init_cond()
        self.Symbolic_variables = self.State[:int(len(self.State) / 2)] + self.Init_cond_symbols + self.Parameters
        self.Symbolic_Box_of_Initial_Conditions = self.create_symbolic_domain()
        self.Symbolic_Domain_of_Parameters = self.create_symbolic_domain_of_parameters()

        self.Samples = self.discretization()
        self.Deltas = None
        self.Gamma = None
        self.UBF = None

        self.LP_data = None
        self.cones_big_angle = None
        self.cones_small_angles = None
        self.mu = None
        self.Regions = None
        self.Grid = None


        # Setting solver options
        if manifolds_times is not None:
            manifolds_times.sort()
            self.Manifolds_Times = manifolds_times.copy()
        else:
            self.Manifolds_Times = [0.001]

        logging.info(f'Manifold Times: {self.Manifolds_Times}')

        if (len(self.Parameters) > 0) | (order_approx <= 1):
            logging.warning(
                "WARNING: The order of manifold approximation is always >=2. For perturbed systems it is always =2.")
            self.p = 2
        else:
            self.p = order_approx

        self.Lie = self.lie_derivatives()
        self.Lie_n = self.Lie[-1]
        self.precision_deltas = precision_deltas;
        self.timeout_deltas = timeout_deltas
        self.partition_method = partition_method;
        if angles_discretization is not None:
            if type(angles_discretization) != list:
                angles_discretization = [angles_discretization]
            if len(angles_discretization) != len(original_state)//2 - 1:
                print(f'Not enough angle discretizations given. Expected: {len(original_state)//2 - 1}, Found: {len(angles_discretization)}')
                raise IncorrectNumberOfItemsInListException('angles_discretization', len(original_state)//2 - 1, len(angles_discretization))

            self.nr_cones_small_angles = angles_discretization[1:]
            self.nr_cones_big_angle = angles_discretization[0]
        else: # Set default
            self.nr_cones_small_angles = [4] * (len(original_dynamics) // 2 - 2); self.nr_cones_big_angle = 6


        # self.nr_cones_small_angles = nr_cones_small_angles or [4] * (len(original_dynamics) // 2 - 2)
        # self.nr_cones_big_angle = nr_cones_big_angle;
        self.state_space_limits = state_space_limits or [[-1, 1]] * (len(original_dynamics) // 2)
        self.grid_points_per_dim = grid_points_per_dim or [5] * (len(original_dynamics) // 2);
        self.heartbeat = heartbeat
        self.precision_timing_bounds = precision_timing_bounds;
        self.precision_transitions = precision_transitions
        self.timeout_timing_bounds = timeout_timing_bounds;
        self.timeout_transitions = timeout_transitions
        self.parallel = parallel
        if parallel:
            print("Warning: parallelization is still being tested, so use at your own risk..")

        logging.info(f'Original State Vector: {original_state}')
        logging.info(f'Original Dynamics: {original_dynamics}')
        logging.info(f'Dynamics: {self.Dynamics}')
        logging.info(f'Trigger: {self.Function}')
        logging.info(f'Deg. Hom.: {self.Homogeneity_degree}')
        logging.info(f'Hom. Flag.: {self.Homogenization_Flag}')
        logging.info(f'Init. State Vector: {self.Init_cond_symbols}')
        logging.info(f'Symbolic Variables: {self.Symbolic_variables}')
        logging.info(f'Order Approx.: {self.p}')
        logging.info(f'State space limits: {self.state_space_limits}')
        logging.info(f'Precision deltas: {self.precision_deltas}')
        logging.info(f'Grid points per dim: {self.grid_points_per_dim}')
        logging.info(f'big_angles: {self.nr_cones_big_angle}')
        logging.info(f'small angles: {self.nr_cones_small_angles}')


    def __repr__(self):
        temp = dict()

        temp['regions'] = [str(x.__dict__) for x in self.Regions]
        temp['transitions'] = {str(k): str(v) for (k,v) in self.transitions.items()}
        temp['state_space_limits'] = str(self.state_space_limits)
        temp['dynamics'] = str(self.Dynamics)
        temp['trigger'] = str(self.Function)
        temp['manifolds_times'] = self.Manifolds_Times
        temp['deltas'] = self.Deltas


        temp['region_descriptors'] = {str(i): str(j) for (i, j) in self.return_region_descriptors().items()}
        return temp

    """ User Methods """
    def create_abstraction(self):
        '''
        Creates the abstraction, by populating the self.Regions list. (Using the solver inputs set with self.__init__())
        '''
        r = self.regions
        t = self.transitions
        return r,t

        # self._build_regions()
        # self._build_transitions(False, self.timeout_transitions, self.precision_transitions)

    @cached_property
    def regions(self):
        if self.Regions is None:
            self._build_regions()

        # return self.Regions
        return {str(reg.index): reg.timing_upper_bound for reg in self.Regions}

    def return_region_descriptors(self):
        return {str(reg.index): reg.symbolic_domain_reach for reg in self.Regions}


    @cached_property
    def transitions(self):
        if any([r.transitions == [] for r in self.Regions]):
            self._build_transitions(self.timeout_transitions, self.precision_transitions)

        # return {str(r.index): r.transitions for r in self.Regions}
        return {(str(r.index), (r.timing_lower_bound, r.timing_upper_bound)): r.transitions for r in self.Regions}

    def region_of_state(self, x:np.array):
        xdict = {i:j for (i,j) in zip(self.Original_State, x)}
        for (reg, descr) in self.return_region_descriptors().items():
            if descr.subs(xdict):
                return reg


    """ Implemented Abstract Methods  """
    def _create_automaton(self):
        return None

    def _create_timed_automaton(self):
        locations = {r.index for r in self.regions}
        clocks = {'c'}
        invariants = {r.index: f'c < {r.timing_upper_bound}' for r in self.regions}
        transitions = [(s.index, f'{s.timing_lower_bound} <= c <= {s.timing_upper_bound}', '*', frozenset('c'), t) for s in self.regions for t in self.transitions[s]]
        from sentient.Systems.Automata import TimedAutomaton
        return TimedAutomaton(locations, invariants, {'*'}, clocks, transitions)


    """ Main Algorithm Methods """

    def _build_regions(self):
        ###Find the deltas
        Cu = np.zeros(self.p + 1)  # Cost parameters for the LP
        Cu[self.p - 1] = 2  # penalize the last delta
        Cu[self.p] = 1  # penalize the difference between the wighted sum with the deltas and the actual lie derivs.

        # Bounds for the deltas for the LP
        Du = []
        Du.append((0, 10 ** 2))
        for d in range(1, self.p - 1):
            Du.append((0, 10 ** 2))
        Du.append((0, 10 ** 3))
        Du.append((0, None))  # no bound on the last delta
        Du = tuple(Du)

        logging.info("Approximating Isochronous Manifolds...")
        self._find_deltas(self.precision_deltas, self.timeout_deltas, 'revised simplex', Cu, Du)
        logging.info("Creating Regions...")
        if (len(self.Parameters) > 0):
            logging.warning("WARNING: For perturbed systems, the only supported partitioning method is gridding.")
            self.partition_method = 'grid'
        if (self.partition_method == 'grid') | (self.partition_method == 'Grid') | (self.partition_method == 'GRID'):
            # naive partition
            self.create_regions_grid(self.state_space_limits, self.grid_points_per_dim, self.Manifolds_Times,
                                     self.heartbeat)
        else:
            # manifold partition
            ret = self.create_regions_manifold(self.Manifolds_Times, self.nr_cones_small_angles,
                                               self.nr_cones_big_angle,
                                               self.state_space_limits, self.grid_points_per_dim,
                                               precision_for_sphere_approx=1e-2)
            if ret < 0:
                return -1

        logging.info("Refining Timing Lower Bounds...")
        self.lower_bounds_refinement(self.precision_timing_bounds, self.timeout_timing_bounds)

        logging.info("Calculating Timing Upper Bounds...")
        self.timing_upper_bounds(self.timeout_timing_bounds, self.heartbeat, self.precision_timing_bounds)

    def _build_transitions(self, time_out=None, remainder_reach=1e-1):
        """INPUTS:
            time_out, float, >0.

            Computes transitions for all Regions.
        """
        print("Computing transitions...")
        dreal_precision = remainder_reach
        tau = sympy.symbols('tau')
        pbar1 = tqdm.tqdm(total=len(self.Regions), position=-1)

        for region in self.Regions:  # for each region
            pbar1.update()
            pbar2 = tqdm.tqdm(total=len(self.Regions), position=-1, leave=False)
            for region2 in self.Regions:  # check if there is transition with each of all regions
                pbar2.update()
                goal_set = region2.symbolic_domain_reach
                goal_set = goal_set & (tau >= region.timing_lower_bound) & (tau <= region.timing_upper_bound)
                if (self.Parameters == ()):  # use dreach if no parameters
                    # for dReach the initial set is given in the common symbolic format
                    initial_set = region.symbolic_domain_reach
                    for e in self.Original_State[int(len(self.Original_State) / 2):]:
                        initial_set = initial_set & (e >= 0) & (e <= 0)
                    res = dReach.dReach_verify(initial_set, goal_set, region.timing_upper_bound, self.Original_State,
                                               self.Original_Dynamics, dreal_precision, dreach_path, smt_path,
                                               f"reach_analysis{shortuuid.uuid()[:6]}.drh", time_out)
                else:  # use flowstar if there are parameters
                    # for flowstar the initial set in the form of iintervals (list of lists)
                    box_domain = region.region_box[:]
                    for e in self.Original_State[int(len(self.Original_State) / 2):]:
                        box_domain.append([0, 0])
                    res = flowstar.flowstar_verify(box_domain, goal_set, region.timing_upper_bound, self.Original_State,
                                                   self.Original_Dynamics, self.Parameters, self.Parameters_domain,
                                                   flowstar_path, smt_path, f"fl_reach_analysis{shortuuid.uuid()[:6]}.model",
                                                   time_out, remainder_reach, )
                if (res['time-out']):  # if time out, enforce transition
                    logging.info('dReach or flowstar time out. Enforcing Transition from Region {} to Region {}'.format(
                        region.index, region2.index))
                if (res['sat']):  # there is transition
                    logging.info('Transition found from Region {} to Region {}'.format(region.index, region2.index))
                    region.insert_transition(region2.index)

    def _build_transition_single(self, region, region2, tau, time_out,dreal_precision, remainder_reach):
        goal_set = region2.symbolic_domain_reach
        goal_set = goal_set & (tau >= region.timing_lower_bound) & (tau <= region.timing_upper_bound)
        if (self.Parameters == ()):  # use dreach if no parameters
            # for dReach the initial set is given in the common symbolic format
            initial_set = region.symbolic_domain_reach
            for e in self.Original_State[int(len(self.Original_State) / 2):]:
                initial_set = initial_set & (e >= 0) & (e <= 0)
            res = dReach.dReach_verify(initial_set, goal_set, region.timing_upper_bound, self.Original_State,
                                       self.Original_Dynamics, dreal_precision, dreach_path, smt_path,
                                       f"reach_analysis{shortuuid.uuid()[:6]}.drh", time_out)
        else:  # use flowstar if there are parameters
            # for flowstar the initial set in the form of iintervals (list of lists)
            box_domain = region.region_box[:]
            for e in self.Original_State[int(len(self.Original_State) / 2):]:
                box_domain.append([0, 0])
            res = flowstar.flowstar_verify(box_domain, goal_set, region.timing_upper_bound, self.Original_State,
                                           self.Original_Dynamics, self.Parameters, self.Parameters_domain,
                                           flowstar_path, smt_path, f"fl_reach_analysis{shortuuid.uuid()[:6]}.model",
                                           time_out, remainder_reach)
        if (res['time-out']):  # if time out, enforce transition
            logging.info('dReach or flowstar time out. Enforcing Transition from Region {} to Region {}'.format(
                region.index, region2.index))
        if (res['sat']):  # there is transition
            logging.info('Transition found from Region {} to Region {}'.format(region.index, region2.index))
            region.insert_transition(region2.index)

    def _find_deltas(self, dreal_precision=0.01, time_out=None, lp_method='revised simplex', C=[], D=[]):
        """ Solves the feasibility problem, by solving iteratively the LP version of it and checking with dReal
        if the found solutions verify the constraints.

        optional argument:
            dreal_precision (rational): Precision used in dReal. Default = 0.01
            time_out: maximum time in seconds, after which the verification with dReal is canceled. Default = None
            lp-method: The LP method 'revised simplex' or 'interior-point'. Default =' revisedsimplex'
            C (numpy array): Cost function vector c of the LP problem
            D (tuple of tuples): Bounds on the delta's

        """
        self.LP_data = lp_lib.LPdata(self, C, D)  # Initialize LP data from the user specified data
        res = {'sat': False}  # when the solution is found we set res['sat'] = true
        res_flag = 0
        res_flag_final = 0
        iteration = 1
        while res['sat'] == False:  # iterate until the solution is found
            logging.info("\nIteration {}".format(iteration))
            res_flag = self.LP_data.LP_solve(lp_method)  # first solve the LP
            if res_flag == -1:
                break
            self.Deltas = self.LP_data.solutions[-1][:-1]  # these are the solutions found by the LP
            self.Gamma = self.LP_data.solutions[-1][-1]
            logging.info("Delta's: {}".format(self.Deltas))
            logging.info("Infinity norm: {}".format(self.Gamma))
            # Construct the UBF given the set of obtained delta's
            self.construct_UBF()
            # Verify the condition using dReal
            res = self.verify_upper_bound_constraint(dreal_precision,
                                                     time_out=time_out)  # check if the found solutions verify the constraints
            if res['time-out'] == True:
                logging.warning("WARNING: Verification timed-out or other unexpected output. "
                                "Please modify the time-out variable or adapt the specification")
                res_flag = -2
                break
            if res['sat'] == False:  # if they dont verify the constraints append a new constraint employing the counterexample res['violation']
                self.LP_data.append_constraint(self, res['violation'], dreal_precision)
            if (self.LP_data.A[-1] == self.LP_data.A[-3]) and (self.LP_data.B[-1] == self.LP_data.B[-3]):
                # if the same counterexample as before is returned, then terminate
                logging.error('ERROR: Same counterexample found by dReal. Terminating script.')
                res_flag = -3
                break
            if res['sat'] == True:  # if the solutions verify the constraint change the flag res_flag_final
                res_flag_final = 1
            iteration += 1

        if res_flag_final > 0:
            logging.info('Valid bound found!')
            logging.info('The deltas are:{}'.format(self.LP_data.solutions[-1][:-1]))
            return 1
        else:
            logging.info("No solution has been found. Try different LP or dReal settings")
            return -1


    def create_box_domain_for_init_cond(self):
        """ If the system has not been homogenized, it computes an inner box of the lyapunov level set provided.
        Otherwise, it dictates a box arbitrarily.
        This will serve as the domain of the initial conditions for the feasibility problem."""
        domain_init_cond = []  # the list containing interval domains for each variable, i.e. the box.
        for i in range(0, int(self.n)):  # iterate along all dimensions (each dimension corresponds to one varable)
            # for each dimension (i.e. for each variable) append a corresponding interval
            if (i == int(self.n / 2 - 1)) & self.Homogenization_Flag:
                domain_init_cond.append(
                    [0, 0.1])  # this is the interval domain for the w variable (i.e. self.State(int(self.n/2-1)))
            elif (i == self.n - 1) & self.Homogenization_Flag:
                domain_init_cond.append(
                    [0, 0])  # this is the interval domain for the ew variable (i.e. self.State(int(self.n-1)))
            else:
                domain_init_cond.append([-0.1, 0.1])  # this is the interval domain for all other variabes
        return domain_init_cond  # I am still experimenting with these values

    def create_symbolic_domain_of_parameters(self):
        """Given the box domain of parameters self.Parameters_domain, write it in a symbolic expression"""
        if len(self.Parameters) == 0:
            return None
        else:
            domain = True
            for i in range(0, len(self.Parameters)):  # iterate along the self.Parameters tuple
                # to write the box constraint for each parameter in a symbolic way
                domain = domain & (self.Parameters[i] >= self.Parameters_domain[i][0])
                domain = domain & (self.Parameters[i] <= self.Parameters_domain[i][1])
            return domain

    def lie_derivatives(self):
        """Computes the Lie derivatives at time t=0 of the specified system in self.Dynamics up to order self.p"""
        lie_list = [self.Function] #the 0th derivative is equal to self.Function
        for i in range(1, self.p):
            #calculates the i-th derivative
            lie_list.append(((lie_list[i-1].diff(sympy.Matrix(self.State)).transpose())*self.Dynamics)[0])
        dic={}
        #creates a dictionary where each variable in self.State[int(len(self.State)/2):] (i.e. all error variables)
        # are substituted by the difference -self.State[i-int(len(self.State)/2)]+self.Init_cond_symbols[i-int(len(self.State)/2)] (e.g. ex = x0-x1)
        for i in range(int(len(self.State)/2),int(len(self.State))):
            dic[self.State[i]]=-self.State[i-int(len(self.State)/2)]+self.Init_cond_symbols[i-int(len(self.State)/2)]
        for i in range(0, self.p):
            lie_list[i]=lie_list[i].subs(dic) #substitutes the dictionary in the computed lie derivatives
        return lie_list

    def discretization(self):
        """ Creates a set of sample points by discretizing self.Init_cond_domain and self.Parameters_domain, based on the gridstep."""
        discr = []
        dimensions = int(self.n / 2) + self.n_init_cond + len(self.Parameters)
        state_vars_and_pars_domain = self.Init_cond_domain[
                                     :]  # list that includes self.Init_cond_domain and self.Parameters_domain
        for i in range(0,
                       len(self.Parameters)):  # append the elements of self.Parameters_domain to state_vars_and_pars_domain
            state_vars_and_pars_domain.append(self.Parameters_domain[i])
        for i in range(1,
                       dimensions + 1):  # create a list of lists containing discretization points for each interval domain contained in state_vars_and_pars_domain
            discr.append(np.linspace((state_vars_and_pars_domain[i - 1])[0], (state_vars_and_pars_domain[i - 1])[1],
                                     2))  # discretize into 2 points
        discretization_points = list(
            itertools.product(*discr))  # get all combinations of elements of the constructed lists.
        return discretization_points  # these combinations are the discretization points

    def create_symbolic_domain(self):
        """ Given the domain of initial conditions in the form of intervals, construct a symbolic domain."""
        sym_dom = sympy.And()  # this is the symbolic domain which will be returned by the function
        sym_list = self.Init_cond_symbols[:]
        full_dom = self.Init_cond_domain[:]
        if (self.Homogenization_Flag == True):  # if the system is homogenized
            sym_list = sym_list + (self.State[
                                       int(self.n / 2) - 1],)  # incorporate the auxilliary variable self.State[int(self.n/2)-1] (i.e. w) in the symbol list
            full_dom.append(self.Init_cond_domain[-1])
        for var in range(0, len(sym_list)):  # iterate along all symbols of initial conditions
            # for each symbol, write the corresponding interval in a symbolic expression and append it to sym_dom
            sym_dom = sympy.And(sym_dom,
                                sympy.And(sym_list[var] >= full_dom[var][0], sym_list[var] <= full_dom[var][1]))
        for var in self.State[:int(self.n / 2)]:
            # for each symbol, write a default big interval in a symbolic expression, for robustness purposes
            sym_dom = sympy.And(sym_dom, sympy.And(var >= -1e2, var <= 1e2))
        return sym_dom  # return the symbolic domain

    def construct_UBF(self, deltas=None):
        """ Construct symbolic upper bounding function (UBF),
        which we want to verify if it bounds self.Lie[-1], given delta's.
        If no delta's are specified, the delta's stored in the SPEC class are used.
        The UBF is a sum of delta'smultiplied by the lie derivatives self.Lie[:-1].
        UBF = self.Deltas[0]*self.Lie[0] + self.Deltas[1]*self.Lie[1] + ... """
        try:
            if deltas == None:
                deltas = self.Deltas
                # construct the symbolic condition to be verified
            if self.p == 1:
                self.UBF = deltas[-1]
            else:
                self.UBF = (sympy.Matrix(deltas[0:self.p - 1]).transpose() * sympy.Matrix(self.Lie[0:self.p - 1]))[0] + \
                           deltas[-1]
            return 1
        except Exception:
            return -1

    def verify_upper_bound_constraint(self, dreal_precision, time_out=None):
        """ Verify if:
            1) the constructed UBF bounds the p-th Lie derivative (self.Lie[-1]) using dReal
            2) the condition delta_0*self.Lie[0].subs(dic)+delta_p > 0 is satisfied
            inside the domain self.Lyapunov_function <= self.Lyapunov_lvl_set_c (if the system is not homogenized) or
            inside the domain self.Function.subs(dic) <= 0, if the system is homogenized."""
        dic = {}
        for i in range(0, int(self.n / 2)):  # iterate along the first half the list self.State
            # to create a dictionary associating each symbol to its initial condition symbol (e.g. dic[x1]=x0)
            dic[self.State[i]] = self.Init_cond_symbols[i]
        fi_initial_cond = self.Lie[0].subs(dic)
        delta_0 = self.LP_data.solutions[-1][0]
        delta_p = self.LP_data.solutions[-1][-2]
        positivity_expr = (
                    delta_0 * fi_initial_cond + delta_p > 0)  # the condition delta_0*self.Lie[0].subs(dic)+delta_p > 0 written symbolically
        dic = {}
        for i in range(int(self.n / 2), self.n):  # iterate along the second half of self.State
            # substitute error variables with difference of x-variable minus initial condition (i.e. ex = x0-x1)
            dic[self.State[i]] = self.Init_cond_symbols[i - int(self.n / 2)] - self.State[i - int(self.n / 2)]
        # the expression to be verified is:
        expression = (self.Function.subs(dic) > 0) | ((self.UBF - self.Lie_n >= 0) & (positivity_expr))

        return dReal.dReal_verify(expression, self.Symbolic_Box_of_Initial_Conditions,
                                  self.Symbolic_Domain_of_Parameters, self.Symbolic_variables, dreal_precision,
                                  dreal_path, smt_path, time_out=time_out)


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
        for i in range(1,
                       int(dimension / 2 - 1)):  # there are int(dimension/2-1) small angles for a coordinate system of dim = dimension/2.
            # discretize each small angle from 0 to pi/2
            small_angles_discr.append(np.linspace(0, math.pi / 2, int(nr_cones_small_angles[i - 1] / 2 + 1)))

        self.cones_big_angle = []
        for i in range(0,
                       len(big_angle_discr) - 1):  # iterate along all discrete angles in which the big angle was discretized
            a = big_angle_discr[i]
            b = big_angle_discr[i + 1]
            self.cones_big_angle.append(
                cones_lib.Cone_on_Plane(a, b, True))  # create the planar cone from angle a to angle b
        for i in range(0,
                       len(big_angle_discr) - 1):  # iterate along all discrete angles in which the big angle was discretized
            a = big_angle_discr[i]
            b = big_angle_discr[i + 1]
            self.cones_big_angle.append(
                cones_lib.Cone_on_Plane(a, b, False))  # create the planar cone from angle pi+a to angle pi+b

        if (dimension / 2 > 2):  # in this case there are small angles as well (in case dimension = 2, there are not)
            self.cones_small_angles = []
            for i in range(0, int(dimension / 2) - 2):  # for each small angle
                temp = []
                for j in range(0, len(small_angles_discr[
                                          i]) - 1):  # iterate along all discrete angles in which the i-th small angle was discretized
                    a = small_angles_discr[i][j]
                    b = small_angles_discr[i][j + 1]
                    temp.append(cones_lib.Cone_on_Plane(a, b, True))  # create the planar cone from angle a to angle b
                for j in range(0, len(small_angles_discr[
                                          i]) - 1):  # iterate along all discrete angles in which the i-th small angle was discretized
                    a = small_angles_discr[i][j]
                    b = small_angles_discr[i][j + 1]
                    temp.append(
                        cones_lib.Cone_on_Plane(a, b, False))  # create the planar cone from angle pi+a to angle pi+b
                self.cones_small_angles.append(temp)

    def check_cone_degeneracy(self, cone):
        """INPUT:
           cone: list of Cone_on_Plane objects.

        Given a combination of planar cones (i.e. Cone_on Plane objects), check if the n-dimansional cone
        defined by all of them is degenerate (return True) or not (return False)."""
        A = np.zeros((int(self.n / 2) - 1, int(self.n / 2)))
        for i in range(1, len(cone)):  # iterate along all planar cones
            A[i][i] = round((cone[i].Linear_Form[0]), 5)
            A[i][i + 1] = round((cone[i].Linear_Form[1]), 5)
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

        if ((grid_points_per_dim == None) | (state_space_limits == None)):
            logging.error("Error. You should specify limits of the state-space and parameters (p1,p2,...) so that it is partitioned into p1 x p2 x ... hyper-rectangles.")
            return -1
        else:
            if self.Homogenization_Flag:
                dimension = int(self.n / 2) - 1
            else:
                dimension = int(self.n / 2)
            linspace_list = []
            for i in range(0, dimension):  # for each dimension (i.e. each interval state_space_limits[i])
                lim_min = state_space_limits[i][0]  # create a linspace grid from
                lim_max = state_space_limits[i][1]  # state_space_limits[i][0] to state_space_limits[i][1]
                # with total number of grid points = grid_points_per_dim
                linspace_list.append(list(np.linspace(lim_min + (lim_max - lim_min) / grid_points_per_dim[i] / 2, \
                                                      lim_max - (lim_max - lim_min) / grid_points_per_dim[i] / 2,
                                                      grid_points_per_dim[i])))
            # the combination of grid points for each dimension are the centers of the created n-dimensional grid
            centroids = list(itertools.product(*linspace_list))
            grid = Grid_obj(centroids[:], state_space_limits[:], grid_points_per_dim[:])
            self.Grid = grid

    def check_radius(self, indicator, manifold_time, expression2, radius_check, dreal_precision, time_out):
        """INPUT:
           indicator: string, either 'i' or 'o'
           manifold_time: float, >0
           expression2: symbolic expression
           radius_check: float >0
           dreal_precision: float >0, preferrably <=1e-1
           time_out: int, >0

        Checks if the given radius (radius_check) inner- or outer-approximates (depending on the indicator=='i' or 'o')
        the manifold segment created by the intersection of a manifold of time manifold_time
        and the symbolic expression expression2(e.g. a conic segment)."""
        ##First compute the expression defining a manifold g(x), onto the given ball, ie. g(x) for |x|=radius_check
        r = self.Init_cond_domain[1][1]  # -0.00000000000000000001*self.Init_cond_domain[1][1]
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
        linear_bound_init_cond.append(self.Lie[0])
        for i in range(1, len(self.Lie) - 1):
            linear_bound_init_cond.append(self.Lie[i] / 2 + abs(self.Lie[i]) / 2)
            # linear_bound_init_cond.append(self.Lie[i])
        dic = {}
        for i in range(0, int(self.n / 2)):  # create a dictionary associating each symbol in Init_cond_symbols
            dic[self.Init_cond_symbols[i]] = self.State[i]  # to the corresponding of self.State (i.e. dic['x0']=x1)
        for i in range(0,
                       len(linear_bound_init_cond)):  # substitute the dictionary into all entries of linear_bound_init_cond
            linear_bound_init_cond[i] = sympy.simplify(linear_bound_init_cond[i].subs(dic))
        dic = {}
        for i in range(0,
                       int(self.n / 2)):  # create a dictionary associating each symbol in the first half of self.State
            dic[self.State[i]] = float(r / radius_check) * self.State[
                i]  # to its projection on the circle of radius_check
        for i in range(0,
                       len(linear_bound_init_cond)):  # substitute the new dictionary to all entries of linear_bound_init_cond
            linear_bound_init_cond[i] = sympy.simplify(linear_bound_init_cond[i].subs(dic))
        linear_bound_init_cond.append(self.LP_data.solutions[-1][-2])
        linear_bound_init_cond = sympy.Matrix(linear_bound_init_cond)

        exponential = expm(
            A * manifold_time * (radius_check / r) ** (self.Homogeneity_degree))  # compute the symbolic matrix
        exponential = sympy.Matrix(exponential)  # exponential of the linear bound

        manifold = sympy.simplify(C * exponential * linear_bound_init_cond)  # compute the expression of the manifold
        manifold = manifold[0]

        if (indicator == 'i'):  # expression is the symbolic expression to be verified
            expression = (manifold <= 0)  # this denotes an inner-approximation
        else:
            expression = (manifold >= 0)  # this denotes an outer approximation
        # Verify if the given radius inner/outer approximates the manifold
        res = dReal.dReal_verify(expression, expression2, None,
                                 self.Symbolic_variables[:int(len(self.Symbolic_variables) / 2)], \
                                 dreal_precision, dreal_path, smt_path, f'radius{shortuuid.uuid()[:6]}.smt2', time_out)
        if res['time-out'] == True:
            logging.warning("WARNING: Verification timed-out or other unexpected output. Please modify the time-out variable or adapt the specification")
            return -2
        if res['sat'] == False:  # this means that the given radius DOES NOT inner/outer approximate the manifold
            return -1
        if res['sat'] == True:  # this means that the given radius inner/outer approximates the manifold
            return 1

    def radius_conic_section(self, indicator, conic_domain, manifold_time, starting_radius=None, precision=1e-3):
        """INPUT:
               indicator: string, either 'i' or 'o'
               conic_domain: symbolic expression
               manifold_time: float, >0
               starting_radius: float, >0

           RETURNS:
               radius_check: float >0, the radius that inner/outer approximates the manifold

        Finds radius (radius_check) that inner-/outer-approximates (depending on indicator) manifold of time manifold_time,
        in the conic section conic_domain."""
        # First, if there is no given starting radius, we find a starting_radius
        # that inner/outer approximates the manifold (then we will refine it)
        state_vector = self.State[:int(self.n / 2)]
        temp = 0
        for i in range(0, int(self.n / 2)):
            temp = temp + state_vector[i] ** 2
        if (starting_radius == None):
            if (indicator == 'i'):  # if we are looking for inner-approximations
                starting_radius = 5  # start from a big radius and then decrease it iteratively until it inner-approximates the manifold
            elif (indicator == 'o'):  # if we are looking for outer-approximations
                starting_radius = 0.1  # start from a small radius and then increase it iteratively until it outer-approximates the manifold

        radius_check = starting_radius  # we have found a radius that inner/outer approximates the manifold
        # refine it to make it tighter
        while (True):
            # print(radius_check)
            # if (indicator == 'i'): #if we are looking for inner-approximations
            #     radius_check = radius_check / 1.01 #slowly decrease the starting radius until it inner-approximates the manifold
            # elif (indicator == 'o'): #if we are looking for outer-approximations
            #     radius_check = radius_check * 1.01 #slowly increase the starting radius until it outer-approximates the manifold
            expression2 = (temp - (0.99 * radius_check) ** 2 >= 0) & (
                        temp - (1.01 * radius_check) ** 2 <= 0) & conic_domain
            res = self.check_radius(indicator, manifold_time, expression2, radius_check, precision, 100)
            if (res == -1):  # if present radius does not inner/outer-approximate the manifold
                # decrease/increase radius
                if (indicator == 'i'):
                    radius_check = radius_check / 1.03
                    if (radius_check <= 1e-2):
                        radius_check = 0
                        break
                elif (indicator == 'o'):
                    radius_check = radius_check * 1.03
            else:
                break

        if (indicator == 'i'):
            logging.info('Radius {} inner approximates manifold of time {} inside cone {}'.format(radius_check,
                                                                                           manifold_time,
                                                                                           conic_domain))
        else:
            logging.info('Radius {} outer approximates manifold of time {} inside cone {}'.format(radius_check,
                                                                                           manifold_time,
                                                                                           conic_domain))
        return radius_check

    def create_regions_grid(self, state_space_limits, grid_points_per_dim, manifolds_times, heartbeat):
        """INPUTS:
            manifolds_times: list of int, >0 (practically I only use manifolds_times[-1])

        Creates the objects Region_NonHomogeneous, using
        the manfifold of time=manifolds_times[-1] and the created grid (by self.create_grid).
        """
        print("Constructing Regions by griding the state space, and estimating their timing lower bounds based on an isochronous manifold (the bounds will later be refined by reachability analysis)...")
        self.create_grid(state_space_limits, grid_points_per_dim)
        manifold_time = manifolds_times[-1]
        if (self.Homogenization_Flag):
            dimension = int(self.n / 2) - 1
        else:
            dimension = int(self.n / 2)
        polytope_sides_lengths = []
        for i in range(0, dimension):  # for each dimension
            lim_min = self.Grid.State_space_limits[i][0]  # calculate what is the displacement
            lim_max = self.Grid.State_space_limits[i][1]  # from the center of a grid polytope
            side_length = (lim_max - lim_min) / self.Grid.Grid_points_per_dim[i]  # to its vertices
            polytope_sides_lengths.append([-side_length / 2, side_length / 2])
        # create a list with all combinations of displacement. each combination, when added to the center of
        # a polytope, it gives one of its vertices.
        differences_between_all_vertices_and_centroid = list(itertools.product(*polytope_sides_lengths))
        region_index = 0
        self.Regions = []

        for centroid in tqdm.tqdm(self.Grid.Centroids):  # iterate along all polytopes, each of which representing a region
            region_index = region_index + 1
            polytope_vertices_in_rn = []
            # add each element of differences_between_all_vertices_and_centroid to the center of the region
            # to get its vertices
            for i in range(0, len(differences_between_all_vertices_and_centroid)):
                aux = np.array(centroid) + np.array(differences_between_all_vertices_and_centroid[i])
                aux = aux.tolist()
                polytope_vertices_in_rn.append(aux)
            [halfspaces_b, halfspaces_A] = cones_lib.polytope_vrep2hrep(polytope_vertices_in_rn)  # get the hrep
            #            aux2 = np.zeros(dimension)
            #            polytope_vertices.append(aux2.tolist())
            if all([v == 0 for v in centroid]):
                # if the polytope contains the origin and the manifolds there are degenerate
                lower_bound = 5e-4  # small enough number as a timing lower bound for the region
                temp = Region_Grid(self.State[:dimension], region_index, centroid, polytope_vertices_in_rn,
                                   halfspaces_A, halfspaces_b, lower_bound, True)
            else:
                lower_bound = self.timing_lower_bounds_grid_manifold(polytope_vertices_in_rn, manifold_time)
                temp = Region_Grid(self.State[:dimension], region_index, centroid, polytope_vertices_in_rn,
                                   halfspaces_A, halfspaces_b, lower_bound, False)
            logging.info('Region {} timing lower bound = {}'.format(region_index, lower_bound))
            #print(('Region {} timing lower bound = {}'.format(region_index, lower_bound)))
            self.Regions.append(temp)

    def timing_lower_bounds_grid_manifold(self, polytope_vertices, manifold_time):
        """INPUTS:
            polytope_vertices: list of lists of float
            manifold_time: float, >0.

        Given the vertices of a polytope and a manifold of time manifold_time,
        it computes the timing lower bound for the region of the polytope.
        Returns the timing lower bound."""
        # First find a sphere that inner approximates the manifold
        # in the cone defined by the polytope
        if self.Homogenization_Flag:  # add a 1 in the end to map the polytopes from R^n to
            # the w=1-plane of R^{n+1}
            for ver in polytope_vertices:
                ver.append(1)
        state_vector = sympy.Matrix(self.State[:int(self.n / 2)])
        [cone_b, cone_A] = cones_lib.polyhedral_cone_vertices2hrep(polytope_vertices)
        cone_A = sympy.Matrix(cone_A)
        cone_b = sympy.Matrix(cone_b)
        A_times_state = cone_A * state_vector
        symbolic_conic_domain = True
        for i in range(0, len(cone_b)):  # create the symbolic conic domain hrep
            symbolic_conic_domain = symbolic_conic_domain & (A_times_state[i] <= cone_b[i])
        # find radius that inner-approximates manifold in the cone
        inner_radius = self.radius_conic_section('i', symbolic_conic_domain, manifold_time, \
                                                 starting_radius=None)
        # Find the polytope vertex with the largest norm
        max_norm = np.linalg.norm(polytope_vertices[0])
        for i in range(1, len(polytope_vertices)):  # iterates along all vertices
            if (np.linalg.norm(polytope_vertices[i]) >= max_norm):
                max_norm = np.linalg.norm(polytope_vertices[i])
        # Compute timing lower bound based on distance between polytope and inner sphere,
        # using max_norm and scaling law
        lamda = max_norm / inner_radius
        lower_bound = manifold_time * lamda ** (-self.Homogeneity_degree)
        return lower_bound

    def create_regions_manifold(self, manifolds_times, nr_cones_small_angles, nr_cones_big_angle, \
                                state_space_limits, grid_points_per_dim, \
                                precision_for_sphere_approx=1e-3):
        """INPUTS:
            manifolds_times: list of int, >0

        Creates the object Regions, by partitioning the state-space in cones and manifolds
        of times=manifolds_times. For each region, it calculates the timing lower bound
        (via the outer manifold) and the inner and outer approximating radii.
        It is called by the function self.abstraction()."""
        self.Regions = []
        all_cones = []
        state_vector = sympy.Matrix(self.State[:int(self.n / 2)])
        dimension = self.n

        self.mu = self.compute_mu()

        if (self.Homogenization_Flag):  # if non-hom. system
            # create a grid on w=1
            # and take the cones defined by the grid polytopes
            self.create_grid(state_space_limits, grid_points_per_dim)
            region_index = 0
            polytope_sides_lengths = []
            for i in range(0, int(dimension / 2) - 1):  # for each dimension
                lim_min = self.Grid.State_space_limits[i][0]  # calculate what is the displacement
                lim_max = self.Grid.State_space_limits[i][1]  # from the center of a grid polytope
                side_length = (lim_max - lim_min) / self.Grid.Grid_points_per_dim[i]  # to its vertices
                polytope_sides_lengths.append([-side_length / 2, side_length / 2])
            # create a list with all combinations of displacement. each combination, when added to the center of
            # a polytope, it gives one of its vertices.
            differences_between_all_vertices_and_centroid = list(itertools.product(*polytope_sides_lengths))
            region_index = [0, 0]
            self.Regions = []
            # for centroid in self.Grid.Centroids: #iterate along all polytopes, each of which representing a region
            for idx1 in range(0, len(self.Grid.Centroids)):
                centroid = self.Grid.Centroids[idx1]
                region_index[0] = 0
                region_index[1] = region_index[1] + 1
                polytope_vertices_in_rn = []
                # add each element of differences_between_all_vertices_and_centroid to the center of the region
                # to get its vertices
                for i in range(0, len(differences_between_all_vertices_and_centroid)):
                    aux = np.array(centroid) + np.array(differences_between_all_vertices_and_centroid[i])
                    aux = aux.tolist()
                    aux.append(1)  # w=1
                    polytope_vertices_in_rn.append(aux)
                [cone_b, cone_A] = cones_lib.polyhedral_cone_vertices2hrep(polytope_vertices_in_rn)
                cone_A = sympy.Matrix(cone_A)
                cone_b = sympy.Matrix(cone_b)
                A_times_state = cone_A * state_vector
                conic_domain = True
                for i in range(0, len(cone_b)):  # create the symbolic conic domain hrep
                    conic_domain = conic_domain & (A_times_state[i] <= cone_b[i])
                all_cones.append(conic_domain)
        else:  # if homogeneous
            # partition the spherical coordinates to create the cones
            self.cone_matrices(nr_cones_small_angles, nr_cones_big_angle)  # partition each angular coordinate into
            # discrete angles that define planar cones

            # create the n-dimensional conic partition by taking
            # combinations of planar cones from each angular coordinate
            if (dimension / 2 > 2):
                temp = self.cones_small_angles[:]  # create a list containing the lists of planar cones
                # for each angular coordinate
                temp.insert(0, self.cones_big_angle)
                # combine the planar cones from each coordinate to derive n-dimensional cones
                all_combinations_of_cone_matrices = list(itertools.product(*temp))
            else:  # if dimension/2 = 2, then there is only one angular coordinate, the big angle.
                self.cones_small_angles = [[()]]  # the list of cones for each small angle is then empty
                temp = self.cones_small_angles[:]
                temp.insert(0, self.cones_big_angle)
                all_combinations_of_cone_matrices = list(itertools.product(*temp))
            for cone in all_combinations_of_cone_matrices:
                flag = False
                if (dimension / 2 > 2):  # if dimension>=3, check if the considered cone is degenerate
                    flag = self.check_cone_degeneracy(cone)
                if not flag:  # if the cone is non-degenrate
                    conic_domain = True  # this is going to be the symbolic expression defining the conic domain
                    for i in range(0,
                                   int(dimension / 2) - 1):  # create iteratively the symbolic expression for the conic domain
                        # taking conic_domain = conic_domain & [x_i x_{i+1}]*Q_i*[x_i x_{i+1}]^T & L_i*[x_i x_{i+1}]
                        # where Q_i and L_i are the quadratic and linear forms that define the cone
                        vec = sympy.Matrix([state_vector[i], state_vector[i + 1]])
                        quadratic_form = sympy.Matrix(cone[i].Quadratic_Form)
                        conic_domain = conic_domain & ((vec.transpose() * quadratic_form * vec)[0] >= 0)
                        linear_form = sympy.Matrix(cone[i].Linear_Form)
                        conic_domain = conic_domain & ((linear_form.transpose() * vec)[0] >= 0)
                    all_cones.append(conic_domain)
        ###Create the regions###
        region_index = [0, 0]
        # for conic_domain in all_cones:
        logging.info("Constructing Regions")
        print("\nConstructing regions as intersections of isochronous manifolds and cones, and computing overapproximations of these regions by ball segments.")
        logging.info(manifolds_times)
        if self.parallel:
            Parallel(n_jobs=-2)(delayed(self._build_region_manifold_single_cone)(idx, all_cones[idx], manifolds_times,
                                precision_for_sphere_approx, state_vector) for idx in
                                tqdm.tqdm(range(0, len(all_cones)), desc='Loop over all cones'))
        else:
            for idx in tqdm.tqdm(range(0, len(all_cones)), desc='Loop over all cones'):
                conic_domain = all_cones[idx]
                region_index[1] += 1
                region_index[0] = 0
                for j in tqdm.tqdm(range(0, len(manifolds_times)), leave=False, desc='Loop over all manifolds'):
                    region_index[0] += 1
                    ###Derive inner and outer radius for the region###

                    # First derive inner and outer radii for the outermost manifold
                    # These will serve as starting points for the line searches
                    # for the radii of the other manifolds, through scaling
                    if (j == 0):
                        if (j == len(manifolds_times) - 1):
                            inner_radius = 0
                        else:
                            outermost_manifold_inner_radius = \
                                self.radius_conic_section('i', conic_domain, manifolds_times[j], \
                                                          None, precision_for_sphere_approx)

                        outermost_manifold_outer_radius = self.radius_conic_section('o', conic_domain, manifolds_times[j], \
                                                                                    None, precision_for_sphere_approx)

                    # Here we find the inner and outer radius for the current region
                    # the starting radius for the line search for the inner (outer) approximation
                    # is the scaling of the outer (inner) radius that has been found for the outermost manifold
                    if (j == 0):
                        outer_radius = outermost_manifold_outer_radius

                        lamda = (manifolds_times[0] / manifolds_times[j + 1]) ** (1 / self.Homogeneity_degree)
                        starting_radius = outermost_manifold_outer_radius * lamda
                        inner_radius = self.radius_conic_section('i', conic_domain, manifolds_times[j + 1], \
                                                                 starting_radius, precision_for_sphere_approx)
                    else:
                        lamda = (manifolds_times[0] / manifolds_times[j]) ** (1 / self.Homogeneity_degree)
                        starting_radius = outermost_manifold_inner_radius * lamda
                        outer_radius = self.radius_conic_section('o', conic_domain, manifolds_times[j], \
                                                                 None, precision_for_sphere_approx)
                        if (j == len(manifolds_times) - 1):
                            inner_radius = 0  # these are the innermost regions
                        else:
                            lamda = (manifolds_times[0] / manifolds_times[j + 1]) ** (1 / self.Homogeneity_degree)
                            starting_radius = outermost_manifold_outer_radius * lamda
                            inner_radius = self.radius_conic_section('i', conic_domain, manifolds_times[j + 1], \
                                                                     starting_radius, precision_for_sphere_approx)

                    # if system is homogenized
                    # check if the region enclosed by the manifolds and the cone intersects w=1
                    # if it doesnt, move on to next region
                    flag = True
                    if (self.Homogenization_Flag):
                        expression1 = ~(
                                    (self.State[int(self.n / 2) - 1] >= 0.99) & (self.State[int(self.n / 2) - 1] <= 1.01))
                        temp = 0
                        for i in range(0, int(self.n / 2)):
                            temp = temp + state_vector[i] ** 2
                        expression2 = (temp - inner_radius ** 2 >= 0) & (temp - outer_radius ** 2 <= 0) & conic_domain
                        res = dReal.dReal_verify(expression1, expression2, None,
                                                 self.Symbolic_variables[:int(len(self.Symbolic_variables) / 2)], \
                                                 1e-3, dreal_path, smt_path, f'aekara{shortuuid.uuid()[:6]}.smt2', 100)
                        if res['sat'] == True:  # this means that the region does not intersect w=1
                            flag = False

                    if flag:  # create the region object and append it to self.Regions
                        outer_manifold_time = manifolds_times[j]
                        if (j == len(manifolds_times) - 1):
                            inner_manifold_time = 10  # something very big
                            contains_origin_flag = True
                        else:
                            inner_manifold_time = manifolds_times[j + 1]
                            contains_origin_flag = False
                        temp = Region_Manifold(state_vector, region_index, inner_manifold_time, outer_manifold_time, \
                                               conic_domain, inner_radius, outer_radius, copy.deepcopy(self.mu), \
                                               contains_origin_flag, self.Homogenization_Flag)
                        self.Regions.append(copy.deepcopy(temp))
                        print('Region {}, lower bound {}, inner_radius {}, outer_radius {}\n'.format(temp.index, \
                                                                                                     temp.timing_lower_bound,
                                                                                                     temp.inner_radius,
                                                                                                     temp.outer_radius))
                        # pbar1.write('Region {}, lower bound {}, inner_radius {}, outer_radius {}\n'.format(temp.index, \
                        #                                                                              temp.timing_lower_bound,
                        #                                                                              temp.inner_radius,
                        #                                                                              temp.outer_radius))
                        # print('Cone: {}\n'.format(conic_domain))
                        # print('Domain of Reach. Analysis {} \n'.format(temp.symbolic_domain_reach))

                    else:
                        logging.info(f'{j} Failed.')
        return 0

    def _build_region_manifold_single_cone(self, idx, conic_domain, manifolds_times, precision_for_sphere_approx, state_vector):
        # conic_domain = all_cones[idx]
        region_index = [0, idx+1]
        for j in tqdm.tqdm(range(0, len(manifolds_times)), leave=False, desc=f'Cone{idx}: Loop over all manifolds', position=idx+1):
            region_index[0] += 1
            ###Derive inner and outer radius for the region###

            # First derive inner and outer radii for the outermost manifold
            # These will serve as starting points for the line searches
            # for the radii of the other manifolds, through scaling
            if (j == 0):
                if (j == len(manifolds_times) - 1):
                    inner_radius = 0
                else:
                    outermost_manifold_inner_radius = \
                        self.radius_conic_section('i', conic_domain, manifolds_times[j], \
                                                  None, precision_for_sphere_approx)

                outermost_manifold_outer_radius = self.radius_conic_section('o', conic_domain, manifolds_times[j], \
                                                                            None, precision_for_sphere_approx)

            # Here we find the inner and outer radius for the current region
            # the starting radius for the line search for the inner (outer) approximation
            # is the scaling of the outer (inner) radius that has been found for the outermost manifold
            if (j == 0):
                outer_radius = outermost_manifold_outer_radius

                lamda = (manifolds_times[0] / manifolds_times[j + 1]) ** (1 / self.Homogeneity_degree)
                starting_radius = outermost_manifold_outer_radius * lamda
                inner_radius = self.radius_conic_section('i', conic_domain, manifolds_times[j + 1], \
                                                         starting_radius, precision_for_sphere_approx)
            else:
                lamda = (manifolds_times[0] / manifolds_times[j]) ** (1 / self.Homogeneity_degree)
                starting_radius = outermost_manifold_inner_radius * lamda
                outer_radius = self.radius_conic_section('o', conic_domain, manifolds_times[j], \
                                                         None, precision_for_sphere_approx)
                if (j == len(manifolds_times) - 1):
                    inner_radius = 0  # these are the innermost regions
                else:
                    lamda = (manifolds_times[0] / manifolds_times[j + 1]) ** (1 / self.Homogeneity_degree)
                    starting_radius = outermost_manifold_outer_radius * lamda
                    inner_radius = self.radius_conic_section('i', conic_domain, manifolds_times[j + 1], \
                                                             starting_radius, precision_for_sphere_approx)

            # if system is homogenized
            # check if the region enclosed by the manifolds and the cone intersects w=1
            # if it doesnt, move on to next region
            flag = True
            if (self.Homogenization_Flag):
                expression1 = ~(
                        (self.State[int(self.n / 2) - 1] >= 0.99) & (self.State[int(self.n / 2) - 1] <= 1.01))
                temp = 0
                for i in range(0, int(self.n / 2)):
                    temp = temp + state_vector[i] ** 2
                expression2 = (temp - inner_radius ** 2 >= 0) & (temp - outer_radius ** 2 <= 0) & conic_domain
                res = dReal.dReal_verify(expression1, expression2, None,
                                         self.Symbolic_variables[:int(len(self.Symbolic_variables) / 2)], \
                                         1e-3, dreal_path, smt_path, f'dreal_aekara{shortuuid.uuid()[:6]}.smt2', 100)
                if res['sat'] == True:  # this means that the region does not intersect w=1
                    flag = False

            if flag:  # create the region object and append it to self.Regions
                outer_manifold_time = manifolds_times[j]
                if (j == len(manifolds_times) - 1):
                    inner_manifold_time = 10  # something very big
                    contains_origin_flag = True
                else:
                    inner_manifold_time = manifolds_times[j + 1]
                    contains_origin_flag = False
                temp = Region_Manifold(state_vector, region_index, inner_manifold_time, outer_manifold_time, \
                                       conic_domain, inner_radius, outer_radius, copy.deepcopy(self.mu), \
                                       contains_origin_flag, self.Homogenization_Flag)
                self.Regions.append(copy.deepcopy(temp))
                print('Region {}, lower bound {}, inner_radius {}, outer_radius {}\n'.format(temp.index, \
                                                                                             temp.timing_lower_bound,
                                                                                             temp.inner_radius,
                                                                                             temp.outer_radius))
                # pbar1.write('Region {}, lower bound {}, inner_radius {}, outer_radius {}\n'.format(temp.index, \
                #                                                                              temp.timing_lower_bound,
                #                                                                              temp.inner_radius,
                #                                                                              temp.outer_radius))
                # print('Cone: {}\n'.format(conic_domain))
                # print('Domain of Reach. Analysis {} \n'.format(temp.symbolic_domain_reach))

            else:
                logging.info(f'{j} Failed.')

    # TODO: find some way to speed it up (e.g. using symengine)
    def compute_mu(self):
        print("Computing symbolic expression for the approximation of isochronous manifolds (this might take some time)...")
        r = self.Init_cond_domain[1][1]  # -0.00000000000000000001*self.Init_cond_domain[1][1]
        C = np.zeros(self.p, )
        C[0] = 1
        C = C.transpose()
        C = sympy.Matrix(C)
        C = C.transpose()
        A = np.zeros((self.p, self.p))
        for i in range(1, self.p):  # create the A matrix of the linear bound, which depends on the deltas
            A[i - 1][i] = 1
            A[-2][i - 1] = self.LP_data.solutions[-1][i - 1]

        mu_0 = []
        mu_0.append(self.Lie[0])
        for i in range(1, len(self.Lie) - 1):
            mu_0.append(self.Lie[i] / 2 + abs(self.Lie[i]) / 2)
            # mu_0.append(self.Lie[i])
        dic = {}
        for i in range(0, int(self.n / 2)):  # create a dictionary associating each symbol in Init_cond_symbols
            dic[self.Init_cond_symbols[i]] = self.State[i]  # to the corresponding of self.State (i.e. dic['x0']=x1)
        for i in range(0, len(mu_0)):  # substitute the dictionary into all entries of mu_0
            mu_0[i] = sympy.simplify(mu_0[i].subs(dic))
        state_norm = 0
        for i in range(0, int(self.n / 2)):
            state_norm = state_norm + self.State[i] ** 2

        state_norm = sympy.sqrt(state_norm)
        dic = {}
        for i in range(0, int(self.n / 2)):  # create a dictionary associating each symbol in the first half of self.State
            dic[self.State[i]] = r / state_norm * self.State[i]  # to its projection on the circle of radius_check

        for i in range(0, len(mu_0)):  # substitute the new dictionary to all entries of mu_0
            mu_0[i] = sympy.simplify(mu_0[i].subs(dic))
        mu_0.append(self.LP_data.solutions[-1][-2])
        mu_0 = sympy.Matrix(mu_0)

        tau = sympy.symbols('tau')
        exponential = sympy.exp(
            sympy.Matrix(A * tau * (state_norm / r) ** (self.Homogeneity_degree)))  # compute the symbolic matrix

        mu = C * exponential * mu_0  # compute the expression of the manifold
        return mu[0]

    def lower_bounds_refinement(self, precision=1e-3, time_out=30):
        tau = sympy.symbols('tau')

        if self.parallel:
            lbounds = Parallel(n_jobs=-2)(delayed(self._timing_lower_bounds_single)(region, tau, time_out, precision) for region in self.Regions)
            for idx, lb in lbounds:
                for reg in self.Regions:
                    if idx == reg.index:
                        reg.timing_lower_bounds = lb
        else:
            pbar = tqdm.tqdm(total=len(self.Regions), position=-1)
            for region in self.Regions:
                pbar.update()
                lower_bound = 1.1 * region.timing_lower_bound  # first estimation of the lower bound
                while True:  # iteratively check if the estimated upper_bound
                    # is truly a timing upper bound. If it isn't, increase and iterate again.
                    if not self.Homogenization_Flag:
                        goal_set = (tau <= lower_bound) & (self.Function >= 0)
                    else:  # if system is homogenized, take out the 'w' variable
                        goal_set = (tau <= lower_bound) & (
                                    self.Function.subs(self.State[int(len(self.State) / 2) - 1], 1) >= 0)
                    if (self.Parameters == ()):  # if no parameters (uncertainties, disturbances, etc.)
                        # we use dReach
                        # for dReach the initial set is given in the common symbolic format
                        initial_set = region.symbolic_domain_reach & (tau >= 0) & (tau <= 0)
                        for e in self.Original_State[int(len(self.Original_State) / 2):]:  # for all measurement errors
                            # initial condition is 0
                            initial_set = initial_set & (e >= 0) & (e <= 0)
                        res = dReach.dReach_verify(initial_set, goal_set, 1.001 * lower_bound, self.Original_State,
                                                   self.Original_Dynamics, precision, dreach_path, smt_path,
                                                   f"lower_bounds{shortuuid.uuid()[:6]}.drh", time_out)
                    else:  # if there are parameters, use flowstar.
                        box_domain = region.region_box[:]
                        for e in self.Original_State[int(len(self.Original_State) / 2):]:  # for all measurement errors
                            # initial condition is 0
                            box_domain.append([0, 0])
                        res = flowstar.flowstar_verify(box_domain, goal_set, 1.01 * lower_bound, self.Original_State,
                                                       self.Original_Dynamics, self.Parameters, self.Parameters_domain,
                                                       flowstar_path, smt_path, f"fl_lower_bounds{shortuuid.uuid()[:6]}.model",
                                                       time_out, precision)
                    if (res['time-out']):
                        break
                    if (not res['sat']):  # verified lower bound
                        lower_bound = 1.1 * lower_bound
                    else:
                        break
                lower_bound = lower_bound / 1.1
                if (lower_bound > region.timing_lower_bound):
                    logging.info('Reachability analysis refined lower bound for Region {}: {}'.format(region.index, lower_bound))
                    print('Reachability analysis refined lower bound for Region {}: {}'.format(region.index, lower_bound))
                    region.timing_lower_bound = lower_bound

            pbar.close()

    def _timing_lower_bounds_single(self, region, tau, time_out, precision):
        lower_bound = 1.1 * region.timing_lower_bound  # first estimation of the lower bound
        while True:  # iteratively check if the estimated upper_bound
            # is truly a timing upper bound. If it isn't, increase and iterate again.
            if not self.Homogenization_Flag:
                goal_set = (tau <= lower_bound) & (self.Function >= 0)
            else:  # if system is homogenized, take out the 'w' variable
                goal_set = (tau <= lower_bound) & (
                        self.Function.subs(self.State[int(len(self.State) / 2) - 1], 1) >= 0)
            if (self.Parameters == ()):  # if no parameters (uncertainties, disturbances, etc.)
                # we use dReach
                # for dReach the initial set is given in the common symbolic format
                initial_set = region.symbolic_domain_reach & (tau >= 0) & (tau <= 0)
                for e in self.Original_State[int(len(self.Original_State) / 2):]:  # for all measurement errors
                    # initial condition is 0
                    initial_set = initial_set & (e >= 0) & (e <= 0)
                res = dReach.dReach_verify(initial_set, goal_set, 1.001 * lower_bound, self.Original_State,
                                           self.Original_Dynamics, precision, dreach_path, smt_path,
                                           f"lower_bounds{shortuuid.uuid()[:6]}.drh", time_out)
            else:  # if there are parameters, use flowstar.
                box_domain = region.region_box[:]
                for e in self.Original_State[int(len(self.Original_State) / 2):]:  # for all measurement errors
                    # initial condition is 0
                    box_domain.append([0, 0])
                res = flowstar.flowstar_verify(box_domain, goal_set, 1.01 * lower_bound, self.Original_State,
                                               self.Original_Dynamics, self.Parameters, self.Parameters_domain,
                                               flowstar_path, smt_path, f"fl_lower_bounds{shortuuid.uuid()[:6]}.model",
                                               time_out, precision)
            if (res['time-out']):
                break
            if (not res['sat']):  # verified lower bound
                lower_bound = 1.1 * lower_bound
            else:
                break
        lower_bound = lower_bound / 1.1
        if (lower_bound > region.timing_lower_bound):
            logging.info(
                'Reachability analysis refined lower bound for Region {}: {}'.format(region.index, lower_bound))
            print('Reachability analysis refined lower bound for Region {}: {}'.format(region.index, lower_bound))
            region.timing_lower_bound = lower_bound

        return (region.index, lower_bound)


    def timing_upper_bounds(self, time_out=200, heartbeat=0.5, precision=1e-3):
        """INPUTS:
            time_out: int, >0.
            heartbeat: float, >0.
            dreal_precision: float, >0.

            Computes timing upper bounds of all regions
        """
        tau = sympy.symbols('tau')
        # found_bound_for_a_region = False
        for region in self.Regions:
            region.insert_timing_upper_bound(heartbeat)

        if self.parallel:
            Parallel(n_jobs=-2)(delayed(self._upper_bound_ref_single)(region, tau, heartbeat, time_out, precision) for region in self.Regions)
        else:
            pbar = tqdm.tqdm(total=len(self.Regions), position=-1)
            for region in self.Regions:
                pbar.update()
                upper_bound = 1.05 * region.timing_lower_bound  # first estimation of the upper bound
                while (True):  # iteratively check if the estimated upper_bound
                    # is truly a timing upper bound. If it isn't, increase and iterate again.
                    if (not self.Homogenization_Flag):
                        goal_set = (tau >= upper_bound) & (tau <= 1.0001 * upper_bound) & (self.Function <= 0)
                    else:  # if system is homogenized, take out the 'w' variable
                        goal_set = (tau >= upper_bound) & (tau <= 1.0001 * upper_bound) & \
                                   (self.Function.subs(self.State[int(len(self.State) / 2) - 1], 1) <= 0)
                    if (self.Parameters == ()):  # if no parameters (uncertainties, disturbances, etc.)
                        # we use dReach
                        # for dReach the initial set is given in the common symbolic format
                        initial_set = region.symbolic_domain_reach & (tau >= 0) & (tau <= 0)
                        for e in self.Original_State[int(len(self.Original_State) / 2):]:  # for all measurement errors
                            # initial condition is 0
                            initial_set = initial_set & (e >= 0) & (e <= 0)
                        res = dReach.dReach_verify(initial_set, goal_set, 1.2 * upper_bound, self.Original_State,
                                                   self.Original_Dynamics, precision, dreach_path,
                                                   smt_path, f"upper_bounds{shortuuid.uuid()[:6]}.drh", time_out)
                    else:  # if there are parameters, use flowstar.
                        # for flowstar the initial set in the form of intervals (list of lists)
                        box_domain = region.region_box[:]
                        for e in self.Original_State[int(len(self.Original_State) / 2):]:  # for all measurement errors
                            # initial condition is 0
                            box_domain.append([0, 0])
                        res = flowstar.flowstar_verify(box_domain, goal_set, 1.0001 * upper_bound, self.Original_State,
                                                       self.Original_Dynamics, self.Parameters, self.Parameters_domain,
                                                       flowstar_path, smt_path, f"fl_upper_bounds{shortuuid.uuid()[:6]}.model",
                                                       time_out, precision)
                    if (not res['sat']):  # found upper bound
                        print('Region {}: timing upper bound = {}.'.format(region.index, upper_bound))
                        logging.info('Region {}: timing upper bound = {}.'.format(region.index, upper_bound))
                        # found_bound_for_a_region = True
                        break  # terminate
                    if (res['time-out']):
                        logging.info('dReach or flowstar time out or other unexpected result. Exiting...')
                        logging.info('flowstar cannot verify this time, due to computational complexity or inexistence of timing upper bound.')
                        logging.info('Region {}: Enforcing heartbeat as upper bound: {}'.format(region.index, heartbeat))
                        print('Region {}: Enforcing heartbeat as upper bound: {}'.format(region.index, heartbeat))
                        upper_bound = heartbeat
                        break

                    logging.info(f'Trying upper bound: {1.05 * upper_bound}')
                    upper_bound = 1.05 * upper_bound  # increase upper bound
                    if (upper_bound >= heartbeat):  # if estimate is bigger than ad-hoc heartbeat
                        upper_bound = heartbeat  # enforce heartbeat as upper bound and terminate
                        logging.info('Region {}: Enforcing heartbeat as upper bound: {}'.format(region.index, upper_bound))
                        print('Region {}: Enforcing heartbeat as upper bound: {}'.format(region.index, heartbeat))
                        break
                region.insert_timing_upper_bound(upper_bound)
            pbar.close()

    def _upper_bound_ref_single(self, region, tau, heartbeat, time_out, precision):
        upper_bound = 1.05 * region.timing_lower_bound  # first estimation of the upper bound
        while (True):  # iteratively check if the estimated upper_bound
            # is truly a timing upper bound. If it isn't, increase and iterate again.
            if (not self.Homogenization_Flag):
                goal_set = (tau >= upper_bound) & (tau <= 1.0001 * upper_bound) & (self.Function <= 0)
            else:  # if system is homogenized, take out the 'w' variable
                goal_set = (tau >= upper_bound) & (tau <= 1.0001 * upper_bound) & \
                           (self.Function.subs(self.State[int(len(self.State) / 2) - 1], 1) <= 0)
            if (self.Parameters == ()):  # if no parameters (uncertainties, disturbances, etc.)
                # we use dReach
                # for dReach the initial set is given in the common symbolic format
                initial_set = region.symbolic_domain_reach & (tau >= 0) & (tau <= 0)
                for e in self.Original_State[int(len(self.Original_State) / 2):]:  # for all measurement errors
                    # initial condition is 0
                    initial_set = initial_set & (e >= 0) & (e <= 0)
                res = dReach.dReach_verify(initial_set, goal_set, 1.2 * upper_bound, self.Original_State,
                                           self.Original_Dynamics, precision, dreach_path,
                                           smt_path, f"upper_bounds{shortuuid.uuid()[:6]}.drh", time_out)
            else:  # if there are parameters, use flowstar.
                # for flowstar the initial set in the form of intervals (list of lists)
                box_domain = region.region_box[:]
                for e in self.Original_State[int(len(self.Original_State) / 2):]:  # for all measurement errors
                    # initial condition is 0
                    box_domain.append([0, 0])
                res = flowstar.flowstar_verify(box_domain, goal_set, 1.0001 * upper_bound, self.Original_State,
                                               self.Original_Dynamics, self.Parameters, self.Parameters_domain,
                                               flowstar_path, smt_path, f"fl_upper_bounds{shortuuid.uuid()[:6]}.model",
                                               time_out, precision)
            if (not res['sat']):  # found upper bound
                print('Region {}: timing upper bound = {}.'.format(region.index, upper_bound))
                logging.info('Region {}: timing upper bound = {}.'.format(region.index, upper_bound))
                # found_bound_for_a_region = True
                break  # terminate
            if (res['time-out']):
                logging.info('dReach or flowstar time out or other unexpected result. Exiting...')
                logging.info(
                    'flowstar cannot verify this time, due to computational complexity or inexistence of timing upper bound.')
                logging.info('Region {}: Enforcing heartbeat as upper bound: {}'.format(region.index, heartbeat))
                print('Region {}: Enforcing heartbeat as upper bound: {}'.format(region.index, heartbeat))
                upper_bound = heartbeat
                break

            logging.info(f'Trying upper bound: {1.05 * upper_bound}')
            upper_bound = 1.05 * upper_bound  # increase upper bound
            if (upper_bound >= heartbeat):  # if estimate is bigger than ad-hoc heartbeat
                upper_bound = heartbeat  # enforce heartbeat as upper bound and terminate
                logging.info('Region {}: Enforcing heartbeat as upper bound: {}'.format(region.index, upper_bound))
                print('Region {}: Enforcing heartbeat as upper bound: {}'.format(region.index, heartbeat))
                break
        region.insert_timing_upper_bound(upper_bound)