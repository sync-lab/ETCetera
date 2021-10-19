import numpy as np
from numpy import random
import scipy.linalg as la
from collections import defaultdict
import logging
from collections import namedtuple
from tqdm import tqdm
import itertools
import warnings
from joblib import Parallel, delayed
import sympy
import multiprocessing
from functools import cached_property, wraps

from sentient.Abstractions.Abstraction import Abstraction

from sentient.Systems import linearetc as etc
from sentient.Abstractions.LinearPETC.utils.optim import QuadraticForm, sdr_problem, QuadraticProblem
from sentient.util.etcgraph_python_specific import TrafficAutomaton
# from sentient.Systems.Automata.timed_automaton import TimedAutomaton
# from sentient.Systems.Automata.priced_timed_automaton import PricedTimedAutomaton
from sentient.Systems.Automata import Automaton
from sentient.Systems.Automata import TimedAutomaton#, PricedTimedAutomaton

SMALL_NUMBER = 1e-7

__TEST__ = False

_RELATIVE_COST_TOLERANCE = 0.001
_QCQP_TOLERANCE = 1e-4
_SSC_MAX_ITERS = 30000  # Reaching maximum number of iterations is bad.
_SSC_MAX_ATTEMPTS = 3  # Number of times to try the SDP problem in inaccurate.
# Should avoid it at all cost. Increase this number if inaccurate results
# are obtained.
LEVEL_SMALL_NUMBER = 1e-6
IFAC_WC_2020 = False
CDC_2020 = False
NUM_CORES = max(1, multiprocessing.cpu_count() - 1)

ABSTRACTION_NO_COSTS = False
__DEBUG__ = False
OUTPUT_PTA = False


class ETCAbstractionError(etc.ETCError):
    pass


def symbolic_decorator(func):
    @wraps(func)
    def wrapper(obj):
        if obj.symbolic:
            obj._convert_MQP_to_symbolic()

        logging.info(f'Starting function: {func}, in decorator: symbolic_decorator.')
        func(obj)
        logging.info(f'Ending function: {func}, in decorator: symbolic_decorator.')
        if obj.symbolic:
            obj._convert_MQP_to_numeric()

    return wrapper


class TrafficModelLinearPETC(Abstraction):
    r"""
    Traffic model of a PETC system.

    Considering a linear quadratic periodic event-triggering system, this
    class computes a finite-state transition system and a (bi)simulation
    relation that captures the traffic patterns of the closed-loop system,
    as well as the changes in such patterns if early triggers are applied.
    The traffic model is essentially a system that, given a sampled state
    x(t), can determine its next discrete inter-event time k and the next
    sample x(t+kh), where h is the PETC sampling time.

    This class implements the algorithms of

    * [1]_ for fast and scalable similar models;
    * [2]_ for bisimilar models of either PETC or a modified PETC that
      samples periodically when states get close to the origin;
    * [3]_ for omega-bisimilar models of PETC.

    The resulting transition system is the tuple (X, U, E, Y, H, P) where

    * X is the set of states. A state is labeled by a tuple (k1, k2, ..., km)
      which indicates the expected sequence of discrete inter-event times
      from a related concrete state.
    * U is the set of actions. These are integers representing the next
      discrete inter-sample time.
    * E is the set of transitions, a subset of X x U x X. A transition
      (x,u,x') means that, from state x, using input u can lead the state
      to x'.
    * Y is the set of outputs, which are integers.
    * H : X -> Y is the output map. H((k1, k2, ...)) = k1.
    * P: E -> Q is the transition price map, Q being the set of rational
      numbers. Different prices can be used from the structure generated
      by this class.

    Note: only state-feedback controllers are currently allowed.

    Parameters
    ----------
    trigger : etc.LinearQuadraticPETC
        The ETC control system to be abstracted.
    kmaxextra : int or None, optional
        Set a number if transitions are to be computed for inter-event times
        longer than trigger.kmax. This can be useful for triggering
        conditions that can be satisfied at a second firing. The default
        is None, which makes kmaxextra = trigger.kmax + max_delay_steps
        (see below).
    early_trigger_only : boolean, optional
        If True, transitions from a region (i, ...) are computed for inter-
        event times up to i + max_delay_steps. The default is False.
    max_delay_steps : int, optional
        The maximum extra discrete time for which transitions are computed.
        That is, for a region (i, ...), transitions are computed from kmin
        until min(kmaxetra, i + max_delay_steps). The default is 0.

        Note: if max_delay_steps > 0, early_trigger_only is automatically
        enabled.
    solver : {'sdr', 'z3'}, optional
        The solver to be used to compute the transition relations::

            'sdr'    (Conservative) semi-definite relaxation, which uses CVX.
            'z3'     Exact solutions through computational algebra solver Z3.

        The default is 'sdr'.
    depth : int, optional
        Maximum depth of the bisimulation algorithm. The default is 1,
        which uses the simulation algorithm from [1]_, with no refinement.
        For greater depths, it is recommended to set solver='z3'.
    stop_if_omega_bisimilar : boolean, optional
        Set to True if the bisimulation algorithm should stop if an omega-
        bisimulation is found (thus, using [3]_). The default is False.
    cost_computation : boolean, optional
        Set to True if the transition costs should be computed. A transition
        cost from R(i) to R(j) after sampling instant k is the pair
        (1 + min cost, 1 + max cost) determined by solving::

            min/max (cost(x(k)) - cost(x(0)))
                s.t. x(0)'Px(0) = 1,
                     x(0) in R(i),
                     x(k) = M(k)x(0) in R(j)
            where cost(x) := x'Px, and P is the Lyapunov matrix.

        The default is False.
    stop_around_origin : boolean, optional
        Set to True if the bisimulation algorithm should stop one all states
        are guaranteed to have entered a ball around the origin. The default
        is False.
    end_level : float, optional
        If stop_around_origin is true, specifies the sublevel set
        {x: x'Px <= end_level} from which the bisimulation algorithm stops.
        This assumes that the initial states are normalized to satisfy
        x(0)'Px(0) = 1, following [2]_. The default is 0.01.
    etc_only : bool, optional
        Set to True if early trigger transitions should not be computed.
        This is useful for verification, but not for scheduling, since the
        resulting abstraction becomes autonomous. The default is False.
    mu_threshold : float >= 0.0, optional
        If bigger than 0.0, attempts to reduce the number of regions by
        discarding regions that are geometrically similar. The bigger the
        number, the more regions are likely to be discarded.
        The default is 0.0.
    min_eig_threshold : float, optional
        The mimimum value the smallest eigenvalue of the matrix describing
        a state-space region must have for this region to be considered.
        If bigger than 0.0, it typically results in a maximum uniform
        inter-event time smaller than what would naturally occur, but for a
        very small subset of the state-space. The default is 0.0.
    reduced_actions : boolean, optional
        If True, action set is equal to the set of possible inter-event
        times. The default is False.
    number_samples : int, optional
        (Deprecated) Number of samples to compute transition probabilities.
        The default is 10000.
    consider_noise : boolean, optional
        (Future) Whether pure measurement noise is to be considered.
        The default is False.


    Attributes
    ----------

    n : int
        The data-space dimension (typically the state-space dimension,
        but it can be different for dynamic / output-feedback controllers).
    regions : set of tuples
        The abstraction state space X.
    inputs : set of ints
        The abstraction input space U.
    transition : dict (regions x inputs -> set of regions)
        The abstraction transition map, using the XxU -> powerset(X)
        construction.
    complete_cost : {}
    probability_region = {}
    probability_transition = {}
    K : set of ints
        The set of possible discrete inter-event times.
    M : dict (K -> numpy.array)
        State-to-state transition matrices for each discrete inter-event time,
        such that x(t+kh) = M(k)x(t).
    N : dict (K -> numpy.array)
        (Future) Transition matrices from state to input and output, such that
        [y(t+kh), u(t+kh)].T = N(k) @ x(t).
    Q : dict (K -> numpy.array)
        Matrices Q[k] define quadratic forms such that, if
        x(t) @ Q[k] @ x(t) > epsilon, then x(t) will certainly trigger
        before t + kh. These are basic matrices to partition the state-
        space and build the (bi)simulation relations. See [1]_, [2]_.
    kmin : int
        Minimum discrete inter-efent time, which is by default trigger.kmin.
        It can be overwritten to a higher value if the algorithm detects that
        no state can trigger at kmin.
    kmax : int
        Maximum discrete inter-event time, which is by default trigger.kmax.
        It can be overwritten to a smaller value if the algorithm detects that
        all state would have triggered before kmax.
    kmaxextra : int
        See Parameters above.
    predecessors = {}  # precedence (subset) relation for quadratic forms
    cost = {}  # Lyapunov reduction after a sequence of sampling times


    Raises
    ------
    ETCAbstractionError
        It happens all the time.


    Methods
    -------



    References
    ----------

    .. [1] Gleizer, Gabriel de A., and Manuel Mazo Jr. "Scalable Traffic
       Models for Scheduling of Linear Periodic Event-Triggered Controllers."
       arXiv preprint arXiv:2003.07642 (2020). Presented at IFAC World
       Congress 2020.

    .. [2] G. A. Gleizer and M. Mazo Jr., "Towards Traffic Bisimulation of
       Linear Periodic Event-Triggered Controllers," in IEEE Control Systems
       Letters, vol. 5, no. 1, pp. 25-30, Jan. 2021,
       doi: 10.1109/LCSYS.2020.2999177.

    .. [3] G. A. Gleizer and M. Mazo Jr. WIP.


    """
    is_discrete_time = True
    M = {}  # Transition matrices to state
    N = {}  # Transition matrices to input and output
    Q = {}  # Quadratic forms for the cones
    kmin = 1
    kmax = None  # Maximum triggering time
    kmaxextra = 1000  # Maximum time scheduler forces trigger
    n = 0  # dimension of the data-space (where the cones live in)
    predecessors = {}  # precedence (subset) relation for quadratic forms
    dP = []  # Matrices for Lyapunov function decay after k samples
    K = set()
    _regions = set()
    cost = {}  # cost of transition Lyapunov-wise
    _transition = {}
    complete_cost = {}
    probability_region = {}
    probability_transition = {}

    def __init__(self, trigger: etc.LinearQuadraticPETC, kmaxextra=None,
                 cost_computation=False, consider_noise=False,
                 mu_threshold=0.0, min_eig_threshold=0.0, symbolic=False,
                 reduced_actions=False, early_trigger_only=False,
                 max_delay_steps=0, depth=1,
                 etc_only=False, end_level=0.01, solver='sdr',
                 stop_around_origin=False, stop_if_omega_bisimilar=False,
                 stop_if_mace_equivalent=False,
                 smart_mace=False,
                 strategy=None,
                 strategy_transition=None):

        super().__init__()

        # Parameters to attributes.
        self.trigger = trigger
        self.h = trigger.h
        self.plant = trigger.plant
        self.controller = trigger.controller
        self.consider_noise = consider_noise
        self.mu_threshold = mu_threshold
        self.min_eig_threshold = min_eig_threshold
        self.cost_computation = cost_computation
        self.reduced_actions = reduced_actions
        self.early_trigger_only = early_trigger_only
        self.max_delay_steps = max_delay_steps
        self.depth = depth
        self.etc_only = etc_only
        self.end_level = end_level
        self.solver = solver
        self.stop_around_origin = stop_around_origin
        self.stop_if_omega_bisimilar = stop_if_omega_bisimilar
        self.kmaxextra = kmaxextra
        self.P = trigger.P
        self.stop_if_omega_bisimilar = stop_if_omega_bisimilar
        self.stop_if_mace_equivalent = stop_if_mace_equivalent
        self.smart_mace = smart_mace
        self.strat = None

        self.l_complete = True  # Standard refinement

        # Depending on the solver, symbolic matrix manipulation is used.
        self.symbolic = symbolic
        # TODO: Currently not working so throw error
        if symbolic:
            raise NotImplementedError

        # When a sampling strategy is given
        self.strat = strategy
        if strategy:
            minlstrat = min(len(x) for x in self.strat)
            assert minlstrat == max(len(x) for x in self.strat)
            self.strat_l = minlstrat
            self.strat_transition = {((x,),k):{(z,) for z in y}
                                     for (x,k),y in strategy_transition.items()}

        # Sanity check: algorithm only works for LTI state feedback
        if depth > 1 and (consider_noise or self.controller.is_dynamic or \
                          not self.plant.states_as_output):
            raise ETCAbstractionError(
                'Bisimulation refinement (depth>1) only for LTI state feedback'
            )

        # kmin and kmax can be determined by the abstraction preprocessing
        # logic, if they are note provided in the triggering condition
        # TODO: Do we want to allow None in trigger.kmin/kmax?
        if trigger.kmax is not None:
            self.kmax = trigger.kmax
        if trigger.kmin is not None:
            self.kmin = trigger.kmin

        ''' Pre-processing: compute matrices and base partition. '''
        self._build_matrices()

        # Depending on user input parameters, we may reduce the number of
        # Q matrices (and, thus, the abstraction state-space) or the number
        # of possible actions (the action space)
        if mu_threshold > 0.0 or self.reduced_actions \
                or self.early_trigger_only:
            self._reduce_sets()

        # The next function builds the set self._predecessors, which
        # determines a partial order between quadratic forms. This is
        # useful to reduce the complexity of the transition computations
        self._build_precedence_relation()

        # The next routine builds a final set of matrices that are useful
        # for transition and cost calculations
        self._prepare_matrices_for_transitions()

        # Building final basic sets
        self.K = set(self.Q.keys())
        self.inputs = set(self.M.keys())

        ''' Initiate main algorithm.

        The algorithm is the standard bisimulation algorithm [4]_, but sets
        are represented in a similar fashion as the l-complete abstractions
        from [5]_.'''

        # Store regions that may be not in the final abstraction, but are
        # needed in the algorithm:
        # self._extended relaxes x in X0, that is, includes regions that may
        # not be in the initial state set in case X0 is not compact. This
        # is only used if self.stop_around_origin.
        self._extended = set()
        # self._initial stores s s.t. ks not in regions. These are sets that
        # can only be initial states, i.e., cannot be reached by another
        # initial state. Only used if self.stop_around_origin
        self._initial = set()
        # self._all_regions stores all regions ever visited during the
        # algorithm execution. Essentially used for memoization
        self._all_regions = set((k,) for k in self.K)
        # self._marginal stores regions that also satisfy the constraint
        # V(x) == 1. These regions are associated with sampling sequences
        # that ensure a reduction of the Lyapunov function by a factor of
        # self.end_level
        self._marginal = set()

        # Regions initialize as the empty set. In fact, this would represent
        # the whole state space, without any partition, if X = X0 = R^n,
        # or the set related to epsilon if self._stop_around_the_origin
        # ({x in R^n: x'Px <= end_level})
        self._regions = set()

        # If MACE equivalence is checked, memoize the behavioral cycles
        if stop_if_omega_bisimilar or stop_if_mace_equivalent:
            self.cycles = set()
            self.non_cycles = set()
            if stop_if_mace_equivalent:
                if smart_mace:
                    self.l_complete = False
                self.limavg = min(self.K)  # Worst case, kmin^omega

        self._mac_regions = None  # This will store regions associated with
        # the current MAC, in case MACE equivalence is checked
        self._mac_candidates = None  # This is a subset of MAC regions to be
        # refined

        # self.generate_regions()
        #
        # # Build final transition map
        # self._build_transition()

    # end of __init__
    def __repr__(self):

    # def _self_to_dict(self):
        temp = dict()

        temp_reg = []
        for x in self.regions:
            temp_reg.append(str(x))

        temp['regions'] = temp_reg

        temp_tr = dict()
        for ((x, u), yy) in self.transitions.items():
            temp_tr[str(x)] = dict()
            for y in yy:
                temp_tr[str(x)][str(u)] = str(y)

        temp['transitions'] = temp_tr
        temp['depth'] = self.depth
        temp['early_trigger_only'] = self.early_trigger_only
        temp['max_delay_steps'] = self.max_delay_steps
        temp['region_descriptors'] = {str(i): str(j) for (i,j) in self.return_region_descriptors().items()}
        return temp

    def _create_timed_automaton(self) -> TimedAutomaton:

        # Locations (set of any hashable type)
        location_set = self._regions
        initial_location_set = location_set.copy()

        # Clocks
        clock = sympy.var(f'c', real=True)
        clock_set = set((clock,))

        # Invariants
        delay = self.max_delay_steps
        invariant_dict = {l: (clock <= l[0] + delay) for l in self._regions}

        # Actions
        action_set = {'trigger', 'early', 'late'}

        # Transition set
        if not self._transition:
            # If transition was not populated, fill transition set with rule from
            # L-CSS paper; all transitions are natural triggers.
            transition_list = [(s,  # Source location
                                f'{clock}=={s[0]}',
                                # sympy.Eq(clock, s[0]),  # Guard
                                'trigger',  # Action
                                clock_set,  # Set of reset clocks
                                d)  # Destination location
                               for s, d in itertools.product(self._regions,
                                                             repeat=2)
                               if s[1:] == d[:len(s) - 1]]

            for s, d in itertools.product(self._regions, repeat=2):
                if s[1:] == d[:len(s) - 1]:
                    print(s, d)
                    print(s[1:])
        else:
            transition_list = [(i,  # Source location
                                f'{clock}=={k}',
                                # sympy.Eq(clock, k),  # Guard
                                'early' if k < i[0] else
                                'trigger' if k == i[0] else
                                'late',
                                clock_set,  # Set of reset clocks
                                j)  # Destination location
                               for (i, k), j_set in self.transitions.items()
                               for j in j_set]





        if self.cost_computation:
            # Costs
            transmission_cost = sympy.var('tr', real=True)
            # TODO: Add the performance cost
            cost_set = set((transmission_cost,))

            # Cost functions
            cost_rate_dict = set()  # If empty, all locations have cost rate 0
            cost_transition_dict = {i: {transmission_cost: 1}
                                    for i, (s, g, a, c, d)
                                    in enumerate(transition_list)
                                    if a in ('trigger', 'early', 'late')}
            return PricedTimedAutomaton(location_set, invariant_dict, action_set,
                                        clock_set, transition_list, initial_location_set,
                                        transition_prices=cost_transition_dict, location_prices=cost_rate_dict)

        return TimedAutomaton(location_set, invariant_dict, action_set,
                              clock_set, transition_list, initial_location_set)

    def _create_automaton(self) -> Automaton:
        transitions = [(i, k, j) for (i, k), j_set in self.transitions for j in j_set]
        actions = set(range(1, self.kmax))
        outputs = set(range(1, self.kmax))
        output_map = {i: i[0] for i in self.regions}
        return Automaton(self.regions, actions, transitions, initial_locations=self.regions,
                         outputs=outputs, output_map=output_map)

    # @cached_property
    def create_abstraction(self):
        r = self.regions
        t = self.transitions
        return r, t

    @cached_property
    def regions(self):
        if self._regions == set():
            self.generate_regions()

        # return self._regions
        return {reg: reg[0] for reg in self._regions}

    @cached_property
    def transitions(self):
        # print(self_transition)
        if self._transition == {}:
            self._build_transition()

        return self._transition

    # TODO: When symbolic works also make sure this works
    # @symbolic_decorator
    def refine(self, i=1) -> None:
        self.depth += i
        # If regions are not yet generated -> do more refinements
        if self._regions == set():
            start = 0
        else:
            start = self.depth - i

        for d in range(start, self.depth):
            logging.info(f'Depth: {d + 1}/{self.depth}')
            s = self._refine_regions()
            if not s:
                break

        self._clear_cache()

    def _clear_cache(self):
        # Reset caches: Need to be recomputed.
        if 'timed_automaton' in self.__dict__:
            del self.__dict__['timed_automaton']
        if 'regions' in self.__dict__:
            del self.__dict__['regions']
        if 'transitions' in self.__dict__:
            del self.__dict__['transitions']
        self._transition = {}

    # TODO: When symbolic works also make sure this works
    # @symbolic_decorator
    def generate_regions(self):
        # if self.symbolic:
        #     self._convert_MQP_to_symbolic()

        for d in range(self.depth):
            logging.info(f'Depth: {d + 1}/{self.depth}')
            s = self._refine_regions()
            if not s:
                break
        # if self.symbolic:
        #     self._convert_MQP_to_numeric()

    def _refine_regions(self):
        # Code for Bisimulation from CDC paper ([2]_)
        # Start with {X_epsilon} U quotient set

        # Split the state-space by computing potential Pre's of the
        # current regions. We say potential because we have to verify if
        # these regions can actually occur.
        if not self.smart_mace:
            self._mac_candidates = None

        # Main refinement algorithm
        if self.strat and self.depth == 1:
            new_regions = self.regions
        else:
            new_regions = self._refinement_step(self._mac_candidates,
                                                self.l_complete)

        logging.debug(f'REGIONS: {self.regions}')
        print('regions====', self._regions)

        # First bisimulation stopping criterion: no new candidates are
        # generated (note: it probably never happens)
        if len(new_regions) == 0:
            logging.info('Bisimulation: all sequences computed!')
            return False

        if not self.stop_around_origin:
            # If current system is deterministic, stop: system admits
            # a bisimulation, no deepening is needed!
            G = self.automaton.G
            if all(G.get_out_degrees(G.get_vertices())==1):
                print('SYSTEM ADMITS A BISIMULATION!!!!')
                return False

            # Alternative stopping criteria for cycle equivalence
            if self.stop_if_omega_bisimilar and \
                    self._is_omega_bisimilar():
                print('System is non-deterministic but omega-'
                      'bisimilar. Stopping.')
                return False

            if self.stop_if_mace_equivalent and \
                    self._is_mace_equivalent():
                print('System is MACE-equivalent. Stopping.')
                return False

            if self.stop_if_robust_mace_equivalent and \
                    self._is_robust_mace_equivalent():
                print('System is RobMACE-equivalent. Stopping.')
                return False

        return True

    def _refinement_step(self, candidates, l_complete):
        if not self.stop_around_origin:
            new_regions, verified_regions = \
                self._generate_domino_candidates(
                    subset=candidates,
                    backward=l_complete)
            logging.debug(f'NEW: {new_regions}')
            logging.debug(f'VERIFIED: {verified_regions}')
            logging.debug(f'{len(new_regions)} ({len(verified_regions)})')
            self._verified_regions = verified_regions
        else:
            new_regions = self._generate_backward_candidates(
                subset=candidates,
                backward=l_complete)

        logging.info(f'Number of regions to be verified: '
                      f'{len(new_regions)}. Was: {len(self._regions)}.')

        # Now verify if the new region candidates can actually exist
        # by performing reachability analysis. The following function
        # filters the regions that pass the reachability test.
        # This function also updates
        new_regions, extended_set, marginal_set, initial_set = \
            self._verify_candidates(new_regions)

        # Update self sets with results from the verification
        self._all_regions.update(new_regions)
        self._extended.update(extended_set)
        self._marginal.update(marginal_set)
        self._initial.update(initial_set)

        # Update regions for next iteration
        if self.stop_around_origin:
            # If stop_around_origin, self._initial must always be included
            # in the working region set.
            self._regions = new_regions.union(self._initial)
        else:
            # Replace regions by the new_regions generated in this
            # iteration
            self._regions = new_regions
            if self.strat:
                self.automaton = TrafficAutomaton(self._regions, self.strat)
            else:
                self.automaton = TrafficAutomaton(self._regions)

        return new_regions

    def return_region_descriptors(self):
        """
        Returns a dict containing symbolic expressions describing the regions
        @return:
        """
        ''' Set descriptors '''
        set_descriptor_dict = dict()
        x = np.array(sympy.symbols(f'x1:{self.n + 1}', real=True))
        for l in self._regions:
            constraints = self._add_constraints_for_region_i(l, set())
            sympy_expressions = [c.sympy_expression(x) for c in constraints]
            set_descriptor_dict[l] = sympy.And(*sympy_expressions)

        return set_descriptor_dict

    def _build_matrices(self):
        """ Build matrices Q defining the basic partition of the state-space,
        as well as transition matrices M and N.

        If some Q matrices are negative-definite, they are discarded, and
        attributes kmin and kmax are overwritten accordingly.

        Refer to [1]_ for the matrices involved.

        References
        ----------

        .. [1] Gleizer, Gabriel de A., and Manuel Mazo Jr. "Self-triggered
               output-feedback control of LTI systems subject to disturbances
               and noise." Automatica 120 (2020): 109129, doi:
               10.1016/j.automatica.2020.109129.
        """

        logging.info('Building matrices for traffic model')

        # Extract basic data
        p = self.plant
        c = self.controller
        t = self.trigger

        # TODO: think about how to treat fundamental PETC and
        # abstraction times differently. For now, h of the abstraction
        # is equal to the h of the controller.
        h_abs = t.h

        # nxp: state-space dimension of the plant;
        # nxc: state-space dimension of the controller;
        # ny: output-space dimension
        # nu: input-space dimension
        # nz: combined dimension of the data sent across the network: y and u.
        nxp = p.nx
        nxc = c.nx
        ny = p.ny
        nu = p.nu
        nz = ny + nu

        # First the more obvious CE: [y;u] = CE[xp;xc;y]
        CE = np.block([np.zeros((nz, nxp)),
                       np.block([[np.zeros((ny, nxc)), np.eye(ny)],
                                 [c.C, c.D]])])

        # Fundamental transition matrices
        Abar = np.block([[p.A, p.B], [np.zeros((nu, nxp + nu))]])
        Phibar = la.expm(Abar * h_abs)
        Phip = Phibar[0:nxp, 0:nxp]
        Gammap = Phibar[0:nxp, nxp:]

        # Loop to compute Mks
        Phipk = Phip
        Gammapk = Gammap
        Ack = c.A
        Bck = c.B

        # Initialize lists of matrices and min/max eigenvalues of Q.
        Mlist = []
        Nlist = []
        Qlist = []
        maxeig = []
        mineig = []

        # If the triggering condition can wait for a second zero crossing,
        # kmax can be arbitrarily high: choose the user input.
        if t.must_trigger_at_first_crossing:
            kmax = None
        else:
            kmax = self.kmax

        # Qbar_yuyu is the triggering matrix w.r.t. y and u.
        if not t.triggering_is_time_varying:
            if t.triggering_function_uses_output:
                Qbar_yuyu = t.Qbar
            else:
                Qbar_yuyu = np.zeros((nz * 2, nz * 2))
                Qbar_yuyu[0:ny, 0:ny] = t.Qbar1
                Qbar_yuyu[0:ny, nz:nz + ny] = t.Qbar2
                Qbar_yuyu[nz:nz + ny, 0:ny] = t.Qbar2.T
                Qbar_yuyu[nz:nz + ny, nz:nz + ny] = t.Qbar3

        ''' Transition matrices:
            Compute transition matrix M(\dk) such that
            zeta(k+\dk) = M(\dk)[xp;xc;y],
            where zeta is the composed state vector:
                - zeta = x for static controllers,
                - zeta = (x,xc) for dynamic controllers.
            Also, compute auxiliary transition matrix N(\dk) such that
            [y(t+\dk*h); u(t+\dk*h)].T = N(\dk)[xp; xc; y].'''

        for i in tqdm(range(0, t.kmax)):
            # M and N according to [1]_ (basic linear systems math)
            M1 = np.block([Phipk, Gammapk @ c.C, Gammapk @ c.D])
            M2 = np.block([np.zeros((nxc, nxp)), Ack, Bck])
            N1 = p.C @ np.block([Phipk, Gammapk @ c.C, Gammapk @ c.D])
            N2 = np.block([np.zeros((nu, nxp)), c.C @ Ack, c.C @ Bck + c.D])
            M = np.block([[M1], [M2]])
            N = np.block([[N1], [N2]])

            # Update transition matrices
            Phipk = Phip @ Phipk
            Ack = c.A @ Ack
            Gammapk = Gammap + Phip @ Gammapk
            Bck = c.B + c.A @ Bck

            # Remember: k := i+1
            # Q(k): defines the cone: [xp;xc;y]'Q(k)[xp;xc;y] > 0:
            # trigger at t+kh (or before)
            if t.triggering_is_time_varying:
                Q = np.block([N.T, CE.T]) \
                    @ self.trigger._Q_time_var(i + 1, h_abs) \
                    @ np.block([[N], [CE]])
            else:
                Q = np.block([N.T, CE.T]) @ Qbar_yuyu @ np.block([[N], [CE]])
            # No noise --> no need to separate xp from y, as y = Cxp.
            if not self.consider_noise:
                # y = Cxp - colapse to only xp-dependency
                IIC = np.zeros((nxp + nxc + ny, nxp + nxc))
                IIC[:-ny, ] = np.eye(nxp + nxc)
                IIC[nxp + nxc:, :nxp] = p.C
                # [xp; xc] = IIC [xp; xc; y]
                Q = IIC.T @ Q @ IIC
                M = M @ IIC
                N = N @ IIC

            # Normalize Q
            if self.trigger.threshold == 0.0:
                Q = Q / la.norm(Q)

            # Append calculated matrices to the lists
            Qlist.append(Q)
            Mlist.append(M)
            Nlist.append(N)

            # Eigenvalues. Using np.real to avoid round-off errors
            # (Q matrices are Hermitian).
            lbd = la.eigvalsh(Q)
            maxeig.append(max(np.real(lbd)))
            mineig.append(min(np.real(lbd)))

            # If mineig > 0, at this point all states would have triggered
            # kmax is None means that we can overwrite it.
            if mineig[-1] > 0 and kmax is None:
                kmax = i + 1
                if self.early_trigger_only:
                    self.kmaxextra = kmax
                    break
            # TODO: What the hell is this???
            # if kmax is not None and mineig[-1] <= 0:
            #     kmax = None
        # end for

        # print([x > -self.min_eig_threshold for x in mineig])
        # print(mineig)
        if kmax is None:  # maximum triggering time prevented finding Q(k) > 0
            kmax = t.kmax

        # Update kmax as the last such that to mineig(Q) <= -min_eig_threshold
        if mineig[-1] > -self.min_eig_threshold:
            # Retroactive search of the last k: mineig[k] <= thresh
            for i in range(kmax - 1, -1, -1):
                if mineig[i] <= -self.min_eig_threshold:
                    break
            kmax = i + 2

        # Erase Qs up to kmax
        Qlist = Qlist[:kmax]

        # Find first k such that a trigger could occur
        try:
            kbeg = next(i for i, l in enumerate(maxeig) if l > 0) + 1
        except StopIteration:
            raise ETCAbstractionError(
                f'No triggering would occur up to {self.kmax}-th iteration.'
                f'List of minimum eigenvalues: {mineig}.')

        # TODO: Need to figure out if this is needed at all.
        # try:
        #     kend = next(i for i, l in enumerate(mineig) if l > 0) + 1
        # except StopIteration:
        #     kend = kmax
        kend = kmax

        # Add data to class
        self.kmin = max(self.kmin, kbeg)
        self.kmax = kend
        self.n = Qlist[0].shape[0]
        if self.kmaxextra is None:
            self.kmaxextra = kend + self.max_delay_steps

        self.M = {i + 1: m for i, m in enumerate(Mlist) if i + 1 <= self.kmaxextra}
        self.N = {i + 1: n for i, n in enumerate(Nlist) if i + 1 <= self.kmaxextra}
        self.Q = {i + 1: q for i, q in enumerate(Qlist)
                  if i + 1 >= self.kmin and maxeig[i] > 0}
        self.Q[kend] = np.eye(Qlist[0].shape[0])  # For the last, all states
        # should trigger

        # Normalizing Q requires adjusting non-zero threshold
        # For now, raise an error, as this is not implemented yet.
        if t.threshold is not None and t.threshold != 0:
            raise ETCAbstractionError('Method is not prepared for non-zero'
                                      'threshold in triggering function')

        # Inform resulting kmin and kmax
        logging.info('kmin=%d, kmax=%d', self.kmin, self.kmax)

    def _convert_MQP_to_symbolic(self):
        self.M = {i: sympy.nsimplify(sympy.Matrix(m), tolerance=0.001,
                                     rational=True)
                  for i, m in self.M.items()}
        self.Q = {i: sympy.nsimplify(sympy.Matrix(q), tolerance=0.001,
                                     rational=True)
                  for i, q in self.Q.items()}
        self.P = sympy.nsimplify(sympy.Matrix(self.P), tolerance=0.001,
                                 rational=True)

    def _convert_MQP_to_numeric(self):
        self._M_sym = self.M.copy()
        self.M = {i: np.array(m).astype(np.float64)
                  for i, m in self.M.items()}
        self._Q_sym = self.Q.copy()
        self.Q = {i: np.array(q).astype(np.float64)
                  for i, q in self.Q.items()}
        self._P_sym = self.P.copy()
        self.P = np.array(self.P).astype(np.float64)

    def _reduce_sets(self):
        """ Reduce number of M,N,Q sets based on user choices."""

        logging.info('Reducing number of Q, M, and N matrices.')

        # First reduction method: based on mu_threshold.
        # Two quadratic forms are considered close to each other if
        # there exist lambda > 0:
        # -mu*I <= Q1 - lambda*Q2 <= mu*I
        # in this case, we can discard one of these matrices.
        # (Note: this is experimental and unpublished.)
        if self.mu_threshold > 0.0:
            new_Q = {self.kmin: self.Q[self.kmin]}
            Q1 = QuadraticForm(self.Q[self.kmin])
            for k in tqdm(sorted(self.Q)):
                if k > self.kmin:
                    Q2 = QuadraticForm(self.Q[k])
                    if Q1.difference_magnitude(Q2) >= self.mu_threshold:
                        # Add to reduced dictionary
                        new_Q[k] = self.Q[k]
                        # Update current Q
                        Q1 = Q2
            self.Q = new_Q

        # reduced_actions imposes action set to be equal to the set of
        # natural inter-event times.
        if self.reduced_actions:
            self.M = {k: m for k, m in self.M.items() if k in self.Q}
            self.N = {k: n for k, n in self.N.items() if k in self.Q}

        # early_trigger_only imposes maximum inter-event time to be the same
        # as the maximum natural inter-event time (plus delay, if present).
        if self.early_trigger_only:
            self.M = {k: m for k, m in self.M.items() if k <= max(self.Q)
                      + self.max_delay_steps}
            self.N = {k: n for k, n in self.N.items() if k <= max(self.Q)
                      + self.max_delay_steps}

        # Log resulting reduction in Q space.
        logging.info('Reduced from %d to %d regions',
                     self.kmax - self.kmin + 1, len(self.Q))

    def _build_precedence_relation(self):
        r""" Compute in advance which cones for 1:k-1 are contained
        in cone for k. This will reduce the size of the constraint
        satisfaction problem.

        This problem takes the form

        .. math:: \forall x: x^\top Q_1x \leq 0 \implies x^\top Q_2x \leq 0.

        If this is true, the second inequality above is redundant with the
        first, and therefore it can be omitted wherever the first is.

        This function stores in self._predecessors this precedence relation.
        That is, if the above example holds, then 1 belongs to
        self._predecessors[2].

        self._predecessors is a dictionary of sets.
        """

        logging.info('Building precedence relation for quadratic forms')

        Q = self.Q

        # Build dictionary of quadratic forms
        q = {k: QuadraticForm(x) for k, x in Q.items()}

        # Note: by construction of Q, q is already sorted by k.

        # Initialize precedessor dictionary.
        predecessors = {}

        # Main loop over quadratic forms
        for i, Qi in q.items():
            if i == 1:  # First triggering time cannot have predecessors
                predecessors[i] = {}
            if i == 2:  # Basic verification, cannot use existing results
                predecessors[i] = {j for j, Qj in q.items()
                                   if i > j and Qi > Qj}
                # Qi > Qj checks the mathematical formula on
                # the docstring of this function.
            else:
                # here we can use the existing precessors for j < i.
                predecessors[i] = set()
                # All j < i is a predecessor candidate
                candidates = sorted([j for j in q if j < i])
                # Check backwards from i-1 to 1.
                while len(candidates) > 0:
                    j = candidates.pop()
                    if Qi > q[j]:  # if j is a predecessor of i
                        predecessors[i].add(j)  # Add it to the set...
                        # and include predecessors[j] in predecessors of i
                        predecessors[i].update(predecessors[j])
                        for k in predecessors[j]:  # Then remove these from
                            # the candidate list.
                            if k in candidates:
                                candidates.remove(k)

        self._predecessors = predecessors

        # Logging of simplification statistics
        count_non_ideal = sum(1 for x, p in self._predecessors.items()
                              for y in self.Q if y < x
                              and y not in p)
        logging.info('Average number of preceding regions not subset'
                     ' of the next region: %d',
                     count_non_ideal / len(self.Q))

    def _prepare_matrices_for_transitions(self):
        """ Some other useful matrices to be stored."""
        P = self.trigger.P
        M = self.M
        Q = self.Q

        # Stores dP[k] such that x' dP[k] x is the actual decrease in the
        # Lyapunov (cost) function after [k] instants
        dP = {k: m.T @ P @ m - P for k, m in M.items()}
        self._dP = dP

        # Quadratics MQM(i,j) define if state enters cone j after i+1 steps
        MQM = {(i, j): mi.T @ qj @ mi
               for i, mi in M.items() for j, qj in Q.items()}
        # When threshold is zero, we can also normalize these matrices
        if self.trigger.threshold == 0.0:
            MQM = {x: mqm / la.norm(mqm) for x, mqm in MQM.items()}
        self._MQM = MQM

    def _generate_domino_candidates(self, subset=None, backward=True):
        # Update max and min string lengths existing in self.regions.
        try:
            self._minL = min(len(x) for x in self._regions)
            self._maxL = max(len(x) for x in self._regions)
        except ValueError:
            self._minL, self._maxL = 0, 0
            return set((k,) for k in self.K), set()

        out_set = set()
        verified_set = set()

        G = self.automaton
        if subset is None:
            for e in G.G.edges():
                s = G.regions[int(e.source())]
                d = G.regions[int(e.target())]
                r = s + d[len(s)-1:]
                out_set.add(r)
                if e.source().out_degree() == 1:
                    verified_set.add(r)
        else:
            logging.debug(f'Subset received in domino: {subset}')
            logging.debug(f'Backward refinement? {backward}')
            logging.debug(f'Current graph: {G.G}')
            for i, v in enumerate(G.G.vertices()):
                region = G.regions[i]
                #logging.debug(f'Region: {region}')
                if region in subset:
                    if backward:
                        d = region
                        for vs in v.in_neighbors():
                            s = G.regions[int(vs)]
                            r = s + d[len(s)-1:]
                            out_set.add(r)
                        if v.in_degree() == 1:
                            verified_set.add(r)
                    else:
                        s = region
                        out_s = set()
                        for vs in v.out_neighbors():
                            d = G.regions[int(vs)]
                            r = s + d[len(s)-1:]
                            r = s + (d[len(s)-1],)  # Could it work?
                            logging.debug(f'{s}-->{d}: {r}')
                            out_s.add(r)
                            out_set.add(r)
                        if len(out_s) == 1:
                            verified_set.update(out_s)
                else:
                    out_set.add(region)
                    verified_set.add(region)

        return out_set, verified_set

    def _generate_backward_candidates(self, subset=None, backward=True):
        """Generate candidates for new regions.

        For every current region s that is not initial, generate
        candidates ks for all k in self.K. This is the splitting part of
        the bisimulation algorithm, where a region P is divided among
        Pre(P) and P\Pre(P). Here, we essentially compute the candidates
        Pre_k(P) for k in K, where Pre_k represents the Pre where a specific
        inter-event time k is chosen. In fact, Pre(P) = union_(k in K)Pre_k(P).

        These are just candidates, in the sense that Pre_k(P) may be empty
        for some k. Checking such emptiness is the most expensive part of the
        algorithm, done later in _verify_candidates. Some candidates are
        excluded in this function using some cheaper check.

        Parameters
        ----------
        subset : set
            Subset of self.regions to generate refined candidates. Default:
            None, which means that all regions are used.

        Returns
        -------
        set of tuples
            The set of candidate regions.

        """

        # Update max and min string lengths existing in self.regions.
        try:
            self._minL = min(len(x) for x in self._regions)
            self._maxL = max(len(x) for x in self._regions)
        except ValueError:
            self._minL, self._maxL = 0, 0
            return set((k,) for k in self.K)

        # Generate all candidates
        # TODO: this is not very smart; can use the Pres instead.
        # (For now, leave it as is, it's not a bottleneck of the code runtime)
        if subset is None:
            out_set = set()
            regions = self._regions - self._initial
        else:
            out_set = self._regions - subset
            regions = subset.intersection(self._regions - self._initial)

        possible_initial = set()  # Deprecated for now
        for (i,k) in itertools.product(regions, self.K):
            if backward:
                ki = (k,) + i
            else:
                ki = i + (k,)

            # _substring_exists verifies if all substrings of ki exist in
            # self.regions (respecting minimum and maximum string lengths)
            # If ki has a substring that does not exist in the current
            # region set, that means that the related subsequence of
            # inter-event times cannot be generated by the system, and
            # therefore ki cannot either. This way ki can be excluded from
            # the candidate set.

            # Likewise, if ki is an initial set, it will be added to the
            # set of regions at a later moment, and we do not need to
            # verify its existance.

            if self._substring_exists(ki) and ki not in self._initial:
                # Check if ki's prefix is in _initial
                for s in self._initial:
                    if ki[:len(s)] == s:
                        possible_initial.add(ki)
                        break
                # If not using l-complete refinement, add smaller substrings
                # that have not been checked yet
                if not self.l_complete:
                    # We know that ki[:-1] exists. We need the tail substrings
                    for m in range(1,len(ki)):
                        subki = ki[m:]
                        if subki not in self._all_regions:
                            out_set.add(subki)
                out_set.add(ki)
        return out_set

    def _substring_exists(self, ij):
        """Check if all substrings of ij exist in self.regions.

        Check all substrings of ij of length compatible with lengths in
        self.regions. If one such substring does NOT exist, return False.
        """
        L = len(ij)

        # TODO: can I just check if substrings of s are in self._all_regions?
        # TODO: If all strings of length l exist, how can one of size l-1
        # not exist?

        # The iteration goes backwards from longest to smallest substrings,
        # because it is easier to not find long substrings than short ones.
        # the first substring that is not found breaks the loop returning
        # False.

        # Iterate over string lengths l
        for l in range(min(L, self._maxL), self._minL - 1, -1):
            # Iterate over the starting index of the substring
            for m in range(0, L - l):
                s = ij[m:m + l]  # This is the substring to be checked.
                # Iterate over the existing regions
                for r in self._regions:
                    lr = len(r)
                    # Case 1: lr < l: is r equal to a substring of s?
                    if lr < l and any(r == s[i:lr + i] for i in range(l - lr + 1)):
                        break
                    # Case 2: lr >= l: is s equal to a substring of r?
                    if any(s == r[i:l + i] for i in range(lr - l + 1)):
                        break
                else:  # Here, no substring of s was found in self.regions
                    return False
        return True

    def _verify_candidates(self, regions):
        """Verify the existence of a given list of inter-event sequences.


        Parameters
        ----------
        regions : set
            Current set of candidate regions

        Returns
        -------
        out_set : set of tuples
            New set of regions, after verification.
        extended_set : set of tuples
            Like out_set, but includes regions that are out of the initial
            set.
        marginal_set : set of tuples
            Subset of out_set satisfying V(x) = 1
        initial_set : set of tuples
            Subset of self.regions that cannot be reached from any region
            in out_set

        See also
        --------

        TrafficModelPETC._verify_sequence : Verify a single region.


        """
        out_set = set()

        try:  # Paralelization does not work in iPython
            __IPYTHON__
        except NameError:
            results = Parallel(n_jobs=NUM_CORES)(
                delayed(self._verify_sequence)(r) for r in regions)
        else:
            results = [self._verify_sequence(r) for r in regions]

        extended_set = set()
        marginal_set = set()
        initial_set = set()

        for r, result in zip(regions, results):
            exists, inside, marginal = result
            if exists:
                extended_set.add(r)
                if inside:
                    out_set.add(r)
                    if marginal:
                        marginal_set.add(r)

        for r in self._regions:
            if all((k,) + r not in out_set for k in self.K):
                initial_set.add(r)

        out = namedtuple("candidate_sets", ["out", "extended", "marginal",
                                            "initial"])
        return out(out_set, extended_set, marginal_set, initial_set)

    def _verify_sequence(self, s):
        r"""Verify the existence of a given inter-event sequence.

        The existence is verified by solving the problem: given a
        sequence :math:`s = k_1k_2...k_m, \exists x \in
        \mathbb{R}^\mathtt{self.n}` such that


        .. math::

           x \in \mathcal{R}_{k_1} \\
           M(k_1)x \in \mathcal{R}_{k_2} \\
           \vdots \\
           M(k_{m-1})\cdots M(k_2)M(k_1)x \in \mathcal{R}_{k_m}.

        In addition, it is verified, if such a state exists, whether some
        state also satisfies :math:`V(x) \leq 1` and
        :math:`V(M(k_{m})\cdots M(k_2)M(k_1)x) \geq \mathtt{self.end\_level}`.
        This is returned in the boolean inside, and only computed if
        self.stop_around_the_origin; otherwise, it is True if the first
        result is True. Finally, it is checked, in case the first two
        are True, if there exists another state :math:`x` satisfying all
        previous conditions, in addition to :math:`V(x) = 1`. The result is
        returned in the third and last boolean; like in the previous case,
        this is only computed if self.stop_around_the_origin.


        Parameters
        ----------
        s : tuple of ints
            Candidate sequence of inter-event times.

        Returns
        -------
        exists : boolean
            Whether the sequence can be exhibited by the PETC system
        inside : boolean
            If exists, whether it keeps a related state satisfying V(x) <= 1
            inside the disc with V(x) >= self.end_level.
        marginal : boolean
            If exists and inside, whether there is a related state satisfying
            V(x) == 1.

        """

        # This verification is a short-circuit to prevent the more expensive
        # constraint satisfaction problem described in the docstring. It
        # verifies (1) if there is only one possible successor to the current
        # sequence s; if so, then this sequence must exist; and (2) if the
        # successor of this sequence is in self._initial. If so, we already
        # now this sequence cannot exist (by definition, a sequence is
        # initial if there is no predecessor to it).
        if not self.stop_around_origin:
            # Check whether current string maps into only one of any other
            # string
            n = s[1:]
            if n in self._initial:
                return False, False, False
            if len([sp for sp in self._regions if sp[:-1] == n[:-1]]) == 1:
                return True, True, True

        # Now solve the constraint satisfaction problem. This is where the
        # solvers will be called, and most of the computation is performed.
        # Now, initialize returned booleans.
        exists, inside, marginal = False, False, False

        # This creates a list of QuadraticForm constraints associated with
        # the sequence s.
        con = self._add_constraints_for_region_i(s, set())

        # Create the (basic) quadratic problem.

        prob = QuadraticProblem(con, solver=self.solver)

        # If it has a solution, try the more specific problems
        if prob.solve():
            exists = True
            # Now check if initial state can belong inside {x: V(x) <= 1}
            if self.stop_around_origin:
                # V(x) <= 1
                new_cons = set((QuadraticForm(self.P, c=-1),))
                # Build matrices: Mns = M(k_(m-1))@...@M(k_2)@M(k_1)
                Mns = self._M_prod(s, len(s) - 1)
                # Ms = M(k_m)@M(k_(m-1))@...@M(k_2)@M(k_1)
                Ms = self.M[s[-1]] @ Mns
                # Last point of the trajectory should be in target set
                new_cons.add(QuadraticForm(Ms.T @ self.P @ Ms, c=-self.end_level))
                # The one before the last should not!
                new_cons.add(QuadraticForm(-Mns.T @ self.P @ Mns, c=self.end_level,
                                           strict=True))
                prob.add_constraints(new_cons)
                if prob.solve():  # Solved again? Try the marginal condition
                    inside = True
                    # Now check if there exists x: V(x) = 1
                    new_cons = set((QuadraticForm(-self.P, c=1),))  # V(x) >= 1
                    prob.add_constraints(new_cons)
                    if prob.solve():
                        marginal = True
            else:  # If not self.stop_around_the_origin, set the others True
                inside, marginal = True, True

        out = namedtuple("result", ["exists", "inside", "marginal"])
        return out(exists, inside, marginal)

    # TODO: When symbolic works also make sure this works
    # @symbolic_decorator
    def _build_transition(self):
        """Build the transition relation.

        For every pair of regions i and j within self.regions, determine for
        each action k in self.action_set whether there is a state x in region
        i that can reach region j after k time units applying the same
        control input.


        Raises
        ------
        e
            An exception raised if the solver returns an unexpected flag.

        Returns
        -------
        None.

        Creates
        -------
        self.transition : dictionary (X,U) : 2^X
            The transition set-valued function: for each pair (region, time),
            a set of reachable regions.

        self.complete_cost : #TODO
            Costs of the transitions... possibly broken currently.

        """
        """Build the transition relation"""

        logging.info('Building transition map')

        M = self.M
        dP = self._dP
        R = self._regions

        # Store min and max lengths for later use
        self._minL = min(len(x) for x in self._regions)
        self._maxL = max(len(x) for x in self._regions)

        # Reachability problem: is there x in region i that reaches region j
        # after k samples?
        # Cost comes almost for free here. Use cost computation instead of pure
        # feasibility
        transition = {}
        # (complete_cost[((i,k),j)])
        complete_cost = {}  # for costs depending on reached set j.

        nIK = len(R) * len(M)
        for i, k in tqdm(itertools.product(R, M), total=nIK):
            k_tuple = (k,)
            if type(i) is not tuple:
                i = (i,)
            if self.early_trigger_only:
                if k > i[0] + self.max_delay_steps:
                    continue
            if self.etc_only:
                if k != i[0]:
                    continue

            # New: if there is a terminal cost, don't perform reachability if
            # i is associated with a terminal string.
            # TODO: does this still make sense? I think not.
            if i in self._initial and k == i[0]:
                continue

            if self.cost_computation:
                dPk = dP[k]

            transition[(i, k)] = set()

            for j in R:
                if type(j) is not tuple:
                    j = (j,)
                if not self.cost_computation:
                    if self._reaches(i, j, k_tuple):
                        transition[(i, k)].add(j)
                else:  # TODO: review this whole cost thing
                    try:
                        logging.debug('Checing transition for %d --%d-> %d',
                                      i, k, j)
                        (cost_low, cost_up) \
                            = self._transition_cost(i, k, dPk, j)
                        if cost_low < -1 or cost_up < -1:
                            logging.debug(
                                'Cost of %d --%d-> %d broken: low=%g, up=%g',
                                i, k, j, cost_low, cost_up)
                            continue
                        if cost_low > cost_up:
                            cost_error = abs(1. - abs(cost_low / cost_up))
                            if cost_error > _RELATIVE_COST_TOLERANCE:
                                logging.debug(
                                    'Cost of %d --%d-> %d broken:'
                                    ' low=%g, up=%g',
                                    i, k, j, cost_low, cost_up)
                                continue
                            else:
                                logging.debug('Cost of %d --%d-> %d slightly'
                                              ' broken: low=%g, up=%g',
                                              i, k, j, cost_low, cost_up)
                            cost_low = (cost_low + cost_up) / 2.
                            cost_up = cost_low
                        complete_cost[((i, k), j)] = (cost_low, cost_up)
                    except Exception as e:
                        if 'Relaxation problem status: infeasible' in str(e):
                            continue
                        else:
                            raise e
                    transition[(i, k)].add(j)

        self._transition = transition
        self.complete_cost = complete_cost

    # For building constraints for the reachability problems
    def _add_constraints_for_region_i(self, i_tuple, con):
        # con += the set of constraints  for x \in R_i
        # Constraints related to the current region:
        #   x'Q(i)x > 0
        #   x'Q(s)x <= 0 for all s < i
        #   if p < s and p is in the predecessors of s,
        #      we do not need to include x'Q(p)x <= 0 in the list.
        # NEW: Using reachability for multiple step regions
        # Constraints related to region i1i2i3...iL
        #   x'Q(i1)x > 0
        #   x'Q(s)x <= 0 for all s < i1
        #   x'M(i1)'Q(i2)M(i1)x > 0
        #   x'M(i1)'Q(s)M(i1)x <= 0 for all s < i2
        #   ...
        #   x'(prod_n(M))'Q(in)prod_n(M)x > 0
        #   x'(prod_n(M))'Q(s)prod_n(M)x <= 0 for all s < in
        #      where prod(M) is M(i(L-1))M(i(L-2))...M(i1)

        i_list = sorted(self.Q)
        for l, i in enumerate(i_tuple):
            i_index = i_list.index(i)
            Mprod = self._M_prod(i_tuple, l)
            if i < i_list[-1]:
                MQM = Mprod.T @ self.Q[i] @ Mprod
                if not self.symbolic:
                    MQM = MQM / max(1e-6, min(abs(la.eigvalsh(MQM))))
                con.add(QuadraticForm(- MQM.copy(), strict=True))
            if i >= i_list[1]:
                i_prev = i_list[i_index - 1]
                MQM = Mprod.T @ self.Q[i_prev] @ Mprod
                if not self.symbolic:
                    MQM = MQM / max(1e-6, min(abs(la.eigvalsh(MQM))))
                con.add(QuadraticForm(MQM.copy()))
                for p in i_list[:i_index - 1]:
                    if p not in self._predecessors[i_prev]:
                        MQM = Mprod.T @ self.Q[p] @ Mprod
                        if not self.symbolic:
                            MQM = MQM / max(1e-6, min(abs(la.eigvalsh(MQM))))
                        con.add(QuadraticForm(MQM.copy()))
        return con

    def _add_constraints_for_reaching_j_after_k(self, j_tuple, k_tuple, con):
        # con += the set of constraints  for M(k)x \in R_j

        j_list = sorted(self.Q)
        MK = self._M_prod(k_tuple, len(k_tuple))
        for l, j in enumerate(j_tuple):
            j_index = j_list.index(j)
            Mprod = self._M_prod(j_tuple, l)
            if j < j_list[-1]:
                MQM = MK.T @ Mprod.T @ self.Q[j] @ Mprod @ MK
                if not self.symbolic:
                    MQM = MQM / min(abs(la.eigvalsh(MQM)))
                con.add(QuadraticForm(-MQM.copy(), strict=True))
            if j >= j_list[1]:
                j_prev = j_list[j_index - 1]
                MQM = MK.T @ Mprod.T @ self.Q[j_prev] @ Mprod @ MK
                if not self.symbolic:
                    MQM = MQM / min(abs(la.eigvalsh(MQM)))
                con.add(QuadraticForm(MQM.copy()))
                # Trivially, if p subset s, it also holds for the MQM related
                # cones
                for p in j_list[:j_index - 1]:
                    if p not in self._predecessors[j_prev]:
                        MQM = MK.T @ Mprod.T @ self.Q[p] @ Mprod @ MK
                        if not self.symbolic:
                            MQM = MQM / min(abs(la.eigvalsh(MQM)))
                        con.add(QuadraticForm(MQM.copy()))
        return con

    def _M_prod(self, i_tuple, l):
        """
        Computes the product of M(k) for k in i_tuple
        @param i_tuple: list of indices to use
        @param l: number of indices used
        @return: Resulting product
        """
        M = self.M
        if self.symbolic:
            prod = sympy.Identity(self.n)
        else:
            prod = np.eye(self.n)
        for i in range(l):
            prod = M[i_tuple[i]] @ prod
        return prod

    def _reaches(self, i, j, k):
        '''Reachability problem:
            is there x in region i that reaches region j after k+1 sample?
        '''

        logging.debug('Checking transition for ' + str((i, j, k)))

        # Shortcut: if k is a prefix of i, use the domino rule
        if i[:len(k)] == k:
            return self._is_compatible_transition(i, j, k)

        # Now, build the constraints for the reachability problem.
        con = self._add_constraints_for_region_i(i, set())
        con = self._add_constraints_for_reaching_j_after_k(j, k, con)
        # print(len(con))

        # Solve the problem
        prob = QuadraticProblem(con, solver=self.solver, unit_ball=True)
        return prob.solve()

    def _is_compatible_transition(self, i, j, k):
        """ Determine if transition (i)--(k)->(j) is compatible with
        the tuples, and with the existing regions, following the domino
        rule. """
        i_suffix = i[len(k):]
        j_prefix = j[:len(i_suffix)]
        return j_prefix == i_suffix

    def _is_omega_bisimilar(self):
        # Build automaton
        self.automaton = TrafficAutomaton(self._regions)

        if self.n > 2:
            maybe = False
            return maybe

        # First step: check if cycles exist
        for c in self.automaton.all_behavioral_cycles():
            print(c)
            if not self._verify_cycle(c):
                print(f'{c} is not a valid cycle')
                return False

        maybe = True
        return maybe

        # Get states that have multiple targets
        succ = {s: [d for d in self._regions if d[:-1] == s[1:]]
                for s in self._regions}
        pred = {d: [s for s in self._regions if d[:-1] == s[1:]]
                for d in self._regions}
        nondet = set(s for s, ds in succ.items() if len(ds) > 1)

        # Loop over the successors d and check if
        # forall x ~ d, Pre(x) ~ Pre(d)
        # prednondet = {d:[s for s in pred[d]] for d in nondet}
        # print(prednondet)

        # First condition: only one predecessor of every non-deterministic
        # node
        for n in nondet:
            if len(pred[n]) > 1:
                return False

        # Second check is the most simple... see if Pre(d) is unique
        # for every destination d of a nondeterministic node
        for n in nondet:
            for d in succ[n]:
                if len(pred[d]) > 1:
                    return False

        # Now check if forall x ~ d, Pre(x) ~ Pre(d)
        print('Checking behavioral equivalence')
        for s in nondet:
            for d in succ[s]:
                if not self._verify_completeness(s, d):
                    return False
        return True

    def _verify_cycle(self, cycle):
        # First check if cycle was already there
        for c in self.cycles:
            if self.two_cycles_are_equal(c, cycle):
                return True

        # Now perform the check using eigenvectors
        l = len(cycle)
        m = self._M_prod(cycle, l)
        for (lbd, mult, vecs) in m.eigenvects():
            if lbd.is_real:
                for vec in vecs:  # To check: when is there more than 1?
                    c = cycle[:]
                    for i in range(l):
                        # print(i)
                        if not self.is_related(vec, c):
                            break
                        # re-cycle
                        vec = self.M[c[0]] @ vec
                        c = c[1:] + c[:1]
                    else:
                        self.cycles.add(tuple(cycle))
                        return True
            else:
                # TODO: what to do for R^n, n >= 3?
                return False
        return False

    @staticmethod
    def two_cycles_are_equal(c1, c2):
        """
        Checks whether two cycles are equal.
        A cycle is a path through unique vertices/nodes/locations that have start and end in the same vertex.
        Since equal loops can be shifted, c1 is doubled such that every possible sequence in the loop is
        present as a subsequence.
        """
        c11 = c1 + c1
        l = len(c2)
        for i in range(l):
            if c2 == c11[i:i + l]:
                return True
        return False

    def _transition_cost(self, i, k, dPk, j=None):
        """
        Potentially deprecated.

        Cost of transition from i after k sampling instants if set j is reached
        Problem: min/max (cost(x(k)) - cost(x(0)))
                 s.t. x'Px = 1 (normalized cost)
                      x(0) in R(i) where R is the region where x(0) belongs
                                  when it is supposed to trigger at instant i.
                      x(k) = M(k)x(0) in R(j),  (omitted if j=None)
        where cost(x) := x'Px

        Parameters
        ----------
        i : int, in self.Q.keys()
            index of the origin region
        k : int, in self.M.keys()
            sampling instant
        dPk : np.array, square matrix
            difference of Lyapunov matrices P(k) - P(0)
        j : int, in self.Q.keys(), optional
            index of the target region

        Returns
        -------
        The interval (mincost, maxcost)

        Raises
        ------
        ETCAbstractionError
            If either the problem is infeasible, indicating reachability is
            not satisfied, or if an unexpected error occured.
        """

        n = dPk.shape[0]
        # Mk = self.M[k]
        P = self.trigger.P

        # Start building QCQP
        # dPk_norm = la.norm(dPk)
        dPk_norm = 1.  # Override normalization (to see if it improves)
        # (apparently it changes nothing, since the solver already normalizes
        # things...)
        dPk = dPk / dPk_norm

        # Objective function
        obj = QuadraticForm(dPk)

        # First constraint: x.T @ P @ x == 1  (normalization)
        con = {QuadraticForm(P, np.zeros(n), -1),  # f(x) <= 1: f(x) - 1 <= 0
               QuadraticForm(-P, np.zeros(n), 1)}  # f(x) >= 1: -f(x) + 1 <= 0

        # Adding a valid cut:
        # x'*dP(k)*x = x'*P(k)*x - x*P*x = x'*P(k)*x - 1 >= -1
        # con += [x.T @ dPk @ x >= -1]  # Not effective
        con = self._add_constraints_for_region_i(i, con)
        if j is not None:
            con = self._add_constraints_for_reaching_j_after_k(j, k, con)

        # Build and solve QCQP problem (SDR)
        probMax = sdr_problem(obj, con, minimize=False)

        # Eigenvalues are global bounds
        Pinv_dPk_eigs = la.eig(la.solve(P, dPk))[0]  # Should be real anyway
        max_global_decay = max(np.real(Pinv_dPk_eigs))
        min_global_decay = min(np.real(Pinv_dPk_eigs))

        n_tries = 0
        while n_tries < _SSC_MAX_ATTEMPTS:
            probMax.solve(eps=_QCQP_TOLERANCE, max_iters=_SSC_MAX_ITERS,
                          verbose=__TEST__)
            max_value = probMax.value
            if 'inaccurate' not in probMax.status:
                break
            n_tries += 1

        if 'inaccurate' in probMax.status:
            logging.info(f'MAX {i}--{k}-->{j} is {probMax.status}')
        if 'infeasible' == probMax.status:
            raise ETCAbstractionError(f'MAX {i}--{k}-->{j} is infeasible')
        elif probMax.status in ('unbounded_inaccurate',
                                'infeasible_inaccurate'):
            logging.debug('%s. Max eig(P,dPk): %g',
                          probMax.status, max_global_decay)
            max_value = max_global_decay
        elif 'optimal' not in probMax.status:
            raise ETCAbstractionError(
                f'MAX {i}--{k}-->{j}: ' +
                'Unknown error. Status of the CVX problem is %s'
                % probMax.status)
        # x'dPk x <= probMax.value  (valid cut, not effective)
        # con.add(QuadraticForm(dPk, np.zeros(n), -probMax.value))
        probMin = sdr_problem(obj, con)

        n_tries = 0
        while n_tries < _SSC_MAX_ATTEMPTS:
            probMin.solve(eps=_QCQP_TOLERANCE, max_iters=_SSC_MAX_ITERS,
                          verbose=__TEST__)
            min_value = probMin.value
            if 'inaccurate' not in probMin.status:
                break
            n_tries += 1

        if 'inaccurate' in probMin.status:
            logging.info(f'MIN {i}--{k}-->{j} is {probMin.status}')
        if 'infeasible' == probMin.status:
            raise ETCAbstractionError(f'MIN {i}--{k}-->{j} is infeasible')
        elif probMin.status in ('unbounded_inaccurate',
                                'infeasible_inaccurate'):
            logging.debug('%s. Min eig(P,dPk): %g',
                          probMin.status, min_global_decay)
            min_value = min_global_decay
        elif 'optimal' not in probMin.status:
            raise ETCAbstractionError(
                f'MIN {i}--{k}-->{j}: ' +
                'Unknown error. Status of the CVX problem is %s'
                % probMin.status)

        logging.debug(
            'MAX %d --%d--> %d: SDR bound: %g; maximum eigenvalue: %g',
            i, k, j, probMax.value, max_global_decay)
        logging.debug(
            'MIN %d --%d--> %d: SDR bound: %g; minimum eigenvalue: %g',
            i, k, j, probMin.value, min_global_decay)

        maxdecay = min(max_value, max_global_decay)
        mindecay = max(min_value, min_global_decay)

        return (mindecay * dPk_norm, maxdecay * dPk_norm)

    '''
        User methods
    '''

    def add_level_sets(self, minV, maxV, nV):
        '''
        Performs additional partitioning on the state space, on top of the
        original cones.  These partitions are of the type
                        {x: V_i <= x'Px <= V_{i+1}},
        where
            V_1 = minV: this can be regarded as the terminal value;
            V_{nV} = maxV: this can be regarded as a safety value; and
            V_i = alpha*V_{i+1}, where alpha is computed from the other
                                 given parameters.
        Note: the terminal set is {x: x'Px <= V_1} and the last set, which is
        unsafe, is {x: x'Px >= V_nV}.
        '''

        self.has_level_sets = True

        alpha = (maxV / minV) ** (1. / (nV - 2))
        V_list = [minV * (alpha ** z) for z in range(0, nV - 1)]
        V_list[-1] = maxV

        # Storage
        self.alpha = alpha
        self.minV = minV
        self.maxV = maxV
        self.V_list = V_list

        ''' Redo reachability '''
        # Step 1: for each i, j, we know the minimum and maximum decay of the
        # Lyapunov function.  This can translate to how many discrete level set
        # jumps can be attained.

        transition_levels = {}
        log_alpha = np.log(alpha)

        for (((i, k), j), (cost_low, cost_high)) in self.complete_cost.items():
            steps_down = int(np.floor(np.log(1 + cost_low) / log_alpha))
            # steps_up = int(np.ceil(np.log(alpha+cost_high)/log_alpha)) - 1
            # It looks like the method above is unnecessarily conservative
            steps_up = int(np.ceil(np.log(1 + cost_high) / log_alpha))
            transition_levels[((i, k), j)] = (steps_down, steps_up)

        # Step 2: build the complete reachability map
        complete_transition = {}
        partitions = {}  # For reference, build a list of partitions
        for ((i, k), cone_list) in self._transition.items():
            sampling_time = i  # Using the actual discrete sampling time
            for v in range(0, nV):
                # Set the partition (index 1: discrete time,
                #                    index 2: Lyapunov level interval)
                if v == 0:
                    level_interval = (0, V_list[0])
                elif v == nV - 1:
                    level_interval = (V_list[-1], np.inf)
                else:
                    level_interval = (V_list[v - 1], V_list[v])

                partitions[(i, v)] = Partition(sampling_time, level_interval)

                # Fill in transition dictionary
                complete_transition[((i, v), k)] = []
                for j in cone_list:
                    steps_down, steps_up = transition_levels[((i, k), j)]
                    low_v = max(0, v + steps_down)
                    high_v = min(nV - 1, v + steps_up)
                    complete_transition[((i, v), k)] \
                        += [(j, n) for n in range(low_v, high_v + 1)]
                # Delete empty transitions - these are unsafe edges
                if not complete_transition[((i, v), k)]:
                    del complete_transition[((i, v), k)]

        self.transition_levels = transition_levels
        self.partitions = partitions
        self.complete_transition = complete_transition

    def is_related(self, x: np.array, s: tuple):
        return all(x in Q
                   for Q in self._add_constraints_for_region_i(s, set()))

    def region_of_state(self, x: np.array):
        """ Determines which region state x belongs

        Parameters
        ----------
        x: numpy.array
            Input state

        Returns
        -------
        int
            Region index (key of self.Q)

        """

        # TODO: use a tree search instead.
        for k in sorted(self._regions):
            if self.is_related(x, k):
                return k
        raise ETCAbstractionError('State %s belongs to no region', str(x))

    def level_of_state(self, x: np.array):
        """ Determines the Lyapunov level where state x belongs.


        Parameters
        ----------
        x : np.array
            Input state

        Returns
        -------
        int
            The Lyapunov level index

        """
        Vmin = self.V_list[0]
        V = x @ (self.trigger.P @ x)
        logging.debug('V = %g', V)
        real_z = np.log(V / Vmin) / np.log(self.alpha)
        z = int(real_z) + 1

        if z >= len(self.V_list):
            # If very close to the upper edge, choose the lower level.
            if real_z % 1 <= LEVEL_SMALL_NUMBER:
                z -= 1
            else:
                raise ETCAbstractionError('State is out of bounds.'
                                          ' Level would be %d',
                                          round(z))

        return max(0, min(len(self.V_list), z))

    def reached_region_of_state(self, x, i):
        """ Determines the region state will be from x after i sampling
        instants

        Parameters
        ----------
        x: numpy.array
            Input state

        Returns
        -------
        int
            Region index (key of self.Q)

        """

        y = self.M[i] @ x
        for j in sorted(self.Q.keys()):
            if all(y in Q
                   for Q in self._add_constraints_for_region_i(j, set())):
                return j
        raise ETCAbstractionError('State %s reached no region', str(x))

    '''
        The following methods are deprecated
    '''

    def _reduce_regions(self):
        """Reduce each the string length of each region if it is terminal.


        Returns
        -------
        None.
        """

        # The cost dictionary will accumulate costs of every visited string
        # for computational reasons
        for r in tqdm(self._initial):
            self._regions.remove(r)
            while len(r) > 1:
                r_new = r[:-1]
                try:
                    cost = self.cost[r_new]
                except KeyError:
                    try:
                        dPr = self._dP_of_region(r_new)
                        cost = 1 + self._transition_cost(r_new, r_new, dPr)[1]
                        self.cost[r_new] = cost
                    except ETCAbstractionError as e:
                        if 'infeasible' == str(e)[-10:]:
                            # This region was infeasible all along
                            try:
                                self.cost.pop(r)
                            except KeyError:
                                pass
                            # self.cost.pop(r_new)
                            continue
                if cost > self.end_level:  # previous region was minimal
                    self._regions.add(r)
                    self._minimal.add(r)
                    break
                r = r_new

        self._initial = set(i for i, c in self.cost.items()
                            if c <= self.end_level)

    def estimate_transition(self, N):
        """ Estimate transition using gridded sampling.

        Builds self.transition_estimated, a dictionary

        Parameters
        ----------
        N: int
            number of points per dimension

        Returns
        -------
        None
        """

        # Initialize estimated transition
        transition_estimated = defaultdict(set)

        # Create uniform grid of angles
        angles = []
        for i in range(self.n - 2):
            angles.append(np.arange(0, np.pi, np.pi / N))
        angles.append(np.arange(0, 2 * np.pi, np.pi / N))

        # Loop for creating points on the unit ball
        # TODO: use tangents instead, for speed
        for phi in itertools.product(*angles):
            x = np.ones(self.n)
            for i in range(self.n):
                for j in range(i):
                    x[i] *= np.sin(phi[j])
                if i != self.n - 1:
                    x[i] *= np.cos(phi[i])
            # The point x is made, check the region it is in...
            i = self.region_of_state(x)
            for k in self.M:  # ... and, for each sampling instant,...
                # ... check the region it reaches
                j = self.reached_region_of_state(x, k)
                transition_estimated[(i, k)].add(j)

        self.transition_estimated = transition_estimated

    def _estimate_probabilities(self, N):
        """ Estimate and initial condition and transition probabilities.

        Generate the probability of transitions and initial condition.
        For initial condition, the estimate comes from checking the
        region of N normally i.i.d. distributed vectors. For
        transitions, out of each region and given each possible action,
        estimate the probability of reaching each of the possible
        target regions.

        This function creates self.probability_transition and
        self.probability_region

        Parameters
        ----------
        N: int
            number of points per dimension

        Returns
        -------
        None
        """
        self.probability_region = {k: 0 for k in self.Q}
        # ((region, sample)): {region: prob}
        self.probability_transition \
            = {key: {j: 0 for j in v} for key, v in self._transition.items()}

        # Generate random uniformally distributed numbers
        xs = random.normal(size=(N, self.n))

        # Loop to count
        for p in range(N):
            x = xs[p, :]
            i = self.region_of_state(x)
            self.probability_region[i] += 1
            for k in self.M:
                if self.early_trigger_only:
                    if k > i:
                        continue
                j = self.reached_region_of_state(x, k)
                if j not in self.probability_transition[(i, k)]:
                    warnings.warn(
                        f'Region {j} is not expected to be reachable from '
                        f'region {i} if sample time is {k}: state is {x}.')
                    self.probability_transition[(i, k)][j] = 0
                    self._transition[(i, k)].add(j)
                    dPk = self._dP[k]
                    mincost, maxcost = self._transition_cost(i, k, dPk)
                    self.complete_cost[((i, k), j)] = (mincost, maxcost)
                self.probability_transition[(i, k)][j] += 1

        # Turn counts into probabilities
        return

        # Normalization is not really needed. But then it is not a probability,
        # but rather a "probability weight" as used in UPPAAL.
        self.probability_region = {i: c / N
                                   for i, c in self.probability_region.items()}
        for key, v in self.probability_transition.items():
            total = sum(v.values())
            self.probability_transition[key] = {j: c / total
                                                for j, c in v.items()}

    def _verify_completeness(self, s, d):
        # Check if forall x ~ d: Pre(x) ~ Pre(d) = s
        # Note that, if we are here, either Pre(x) ~ s or Pre(x) does
        # not exist. Thus, we can check if exists x: Pre(x) does not
        # exist. If this is true, completeness is proven to be false.
        # Again, does not exist means Pre(x) !~ s.
        ''' Assumption: s = Pre(d). Equal, not in. '''
        # Problem: exists x ~ d: Pre(x) !~ s.
        #
        # Pre(x) !~ s: M_s[0]^(-1) x not in R_s
        # z not in R_s <==> (OR_i z not in R_s_i) for the i inequalities
        # that compose R_s.
        #
        # Denote by y = M_s[0]^(-1) x
        # Exists y: M_s[0]y ~ d and y !~ s
        # = Exists y: M_s[0]y ~ d and y !~ s_i for any i
        print(f'Checking if Pre({d}) in {s}')
        con_d = self._add_constraints_for_reaching_j_after_k(d, s[:1], set())
        con_s = self._add_constraints_for_region_i(s, set())
        con_s = set(c.complement() for c in con_s)  # Negate

        # Solve the problems
        for c in con_s:
            con = con_d.union((c,))
            prob = QuadraticProblem(con, solver=self.solver, unit_ball=True)
            if prob.solve():
                print(prob.value)
                return False
        return True

    def _compute_costs(self):
        logging.info('Computing terminal Lyapunov level sets')
        self._compute_dP()
        old_cost = self.cost.copy()
        to_be_deleted = set()

        for i in tqdm(self._regions):
            try:
                if i not in old_cost:
                    self.cost[i] = 1 \
                                   + self._transition_cost(i, i, self._dP[i])[1]
            except ETCAbstractionError:
                to_be_deleted.add(i)
        self._regions.difference_update(to_be_deleted)
        self._initial = set(i for i, c in self.cost.items()
                            if c <= self.end_level and i in self._regions)
        self._regions.update()

    def _dP_of_region(self, r):
        l = len(r)
        m = self._M_prod(r, l)
        return m.T @ self.trigger.P @ m - self.trigger.P

    def _compute_dP(self):
        # Compute dP matrices of regions
        self._dP = {r: self._dP_of_region(r) for r in self._regions}

    def _split_regions(self):
        """Split regions one step with reachability.

        If a region represents a string of sampling times i1i2...in, after
        spliting a region represents a string of sampling times i1i2...i(n+1).

        Returns
        -------
        set
            Its elements are tuples containing the possible sequences of
            sampling times generated by the PETC of length n+1, where n is
            the length of the string of the current abstraction.
        """

        # Form concatenated substrings
        out_set = set()
        for (i, k), j_list in self._transition.items():
            if k == i[-1]:
                for j in j_list:
                    # n = len(i)
                    ij = i + j
                    if self._substring_exists(ij):
                        out_set.add(ij)
                    # for m in range(n):
                    #     if ij[m:n+1+m] not in out_list:
                    #         # print(ij[m:n+1+m])
                    #         out_list.append(tuple(ij[m:n+1+m]))
        return out_set
