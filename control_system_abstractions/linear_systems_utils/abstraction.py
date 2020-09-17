#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 24 10:12:45 2018

@author: ggleizer
"""

import numpy as np
from numpy import random
import scipy.linalg as la
from collections import defaultdict
import logging
import time
from collections import namedtuple
from tqdm import tqdm
import itertools
import warnings
from joblib import Parallel, delayed
import multiprocessing
import sympy

from . import linearetc as etc
from . import linearsys
from .etcutil import QuadraticForm, sdr_problem, QuadraticProblem
from .etcgraph import TrafficAutomaton


SMALL_NUMBER = 1e-7
__DEBUG__ = False
__TEST__ = False
ABSTRACTION_NO_COSTS = False
_RELATIVE_COST_TOLERANCE = 0.001
_QCQP_TOLERANCE = 1e-4
_SSC_MAX_ITERS = 30000  # Reaching maximum number of iterations is bad.
_SSC_MAX_ATTEMPTS = 3  # Number of times to try the SDP problem in inaccurate.
# Should avoid it at all cost. Increase this number if inaccurate results
# are obtained.
LEVEL_SMALL_NUMBER = 1e-6
IFAC_WC_2020 = False
CDC_2020 = False
NUM_CORES = max(1, multiprocessing.cpu_count()-1)
OUTPUT_PTA = False


''' Class definitions '''


Partition = namedtuple('Partition', ['k', 'v'])


PTA = namedtuple('PTA', ('Locations', 'InitialLocations', 'Actions',
                         'Clocks', 'Edges', 'Invariants', 'Prices',
                         'LocationPrices', 'EdgePrices'))


class ETCAbstractionError(etc.ETCError):
    pass


class TrafficModel:
    """Base class for a control system's traffic model."""

    plant: linearsys.Plant
    controller: linearsys.Controller
    transition = {}  # Discrete-state transition matrices
    is_discrete_time: bool
    has_level_sets = False

    def region_of_state(self, x: np.array, **kwargs):
        """Determine which region state x belongs.

        Parameters
        ----------
        x : numpy.array
            Input state

        Returns
        -------
        int
            Region index (key of self.Q)
        """
        return 0

    def level_of_state(self, x: np.array):
        """ Determines the Lyapunov level where state x belongs.


        Parameters
        ----------
        x : np.array
            Input state

        Returns
        -------
        int
            The level index

        """
        return 0


class TrafficModelETC(TrafficModel):
    """Base class for an ETC system's traffic model."""
    trigger: etc.ETC


class TrafficModelPETC(TrafficModelETC):
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
    regions = set()
    cost = {}  # cost of transition Lyapunov-wise
    transition = {}
    complete_cost = {}
    probability_region = {}
    probability_transition = {}

    def __init__(self, trigger: etc.LinearQuadraticPETC, kmaxextra=None,
                 cost_computation=False, consider_noise=False,
                 mu_threshold=0.0, min_eig_threshold=0.0,
                 reduced_actions=False, early_trigger_only=False,
                 max_delay_steps=0, number_samples=10000, depth=1,
                 etc_only=False, end_level=0.01, solver='sdr',
                 stop_around_origin=False, stop_if_omega_bisimilar=False):

        # Parameters to attributes.
        self.trigger = trigger
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

        # Depending on the solver, symbolic matrix manipulation is used.
        self._symbolic = solver in {'z3'}

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

        # Deprecated
        # self._minimal = set()

        # Convert M and Q matrices to sympy rational matrices
        if self._symbolic:
            self.M = {i:sympy.nsimplify(sympy.Matrix(m), tolerance=0.001,
                                        rational=True)
                      for i,m in self.M.items()}
            self.Q = {i:sympy.nsimplify(sympy.Matrix(q), tolerance=0.001,
                                        rational=True)
                      for i,q in self.Q.items()}

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

        self._marginal = set()

        # Regions initialize as the empty set. In fact, this would represent
        # the whole state space, without any partition, if X = X0 = R^n,
        # or the set related to epsilon if self._stop_around_the_origin
        # ({x in R^n: x'Px <= end_level})
        self.regions = set()

        # If omega-bisimulation is checked, memoize the behavioral cycles
        if stop_if_omega_bisimilar:
            self.cycles = set()

        # Code for Bisimulation from CDC paper ([2]_)
        # Start with {X_epsilon} U quotient set
        for d in range(self.depth):
            # Split the state-space by computing potential Pre's of the
            # current regions. We say potential because we have to verify if
            # these regions can actually occur.
            new_regions = self._generate_backward_candidates()

            # First bisimulation stopping criterion: no new candidates are
            # generated (note: it probably never happens)
            if len(new_regions) == 0:
                print('Do I ever enter here?')
                logging.info('Bisimulation: all sequences computed!')
                break


            logging.info(f'Number of regions to be verified: '
                         f'{len(new_regions)}. Was: {len(self.regions)}.')

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
                self.regions = new_regions.union(self._initial)
            else:
                # Replace regions by the new_regions generated in this
                # iteration
                self.regions = new_regions

            print('regions====', self.regions)

            # Second bisimulation stopping criterion: if all new regions were
            # already in self._initial, it means that no new states were found,
            # and therefore the bisimulation algorithm has converged.
            # Note: it also seems that this condition is never met.
            if len(new_regions - self._initial) == 0:
                logging.info('Bisimulation: all sequences computed!')
                break

            if not self.stop_around_origin:
                # If current system is deterministic, stop: system admits
                # a bisimulation, no deepening is needed!
                for s in self.regions:
                    # Compute Post(s) following the domino rule.
                    post = [sp for sp in self.regions if sp[:-1] == s[1:]]
                    if len(post) > 1:
                        break  # At least one state has > 1 successor.
                else:  # Enter here if for all s, |Post(s)| = 1 (determinism)
                    # Caps because it is very exciting. I.e., it does not
                    # happen very often.
                    print('SYSTEM ADMITS A BISIMULATION!!!!')
                    break

                # A final stopping criterion, allows stopping if an omega-
                # bisimulation is found. This is work in progress.
                if self.stop_if_omega_bisimilar and \
                        self._is_omega_bisimilar():
                    print('System is non-deterministic but omega-'
                          'bisimilar. Stopping.')
                    break

        # Build final transition map
        self._build_transition()

        # Convert M and Q matrices back to numpy matrices.
        # Store the sympy matrices in different hidden attributes.
        if self._symbolic:
            self._M_sym = self.M.copy()
            self.M = {i:np.array(m).astype(np.float64)
                      for i,m in self.M.items()}
            self._Q_sym = self.Q.copy()
            self.Q = {i:np.array(q).astype(np.float64)
                      for i,q in self.Q.items()}

        # # if d < self.depth - 1:
        # new_regions = self._split_regions()
        # new_regions.update(x for x in self._minimal)

        # Deprecated: probabilistic transition calculation
        if False:
            logging.info('Estimating probabilities')
            self._estimate_probabilities(number_samples)
    # end of __init__

    def _build_matrices(self):
        """ Build matrices Q defining the basic partition of the state-space,
        as well as transition matrices M and N.

        If some Q matrices are negative-definite, they are discarded, and
        attributes kmin and kmax are overwritten accordingly."""

        logging.info('Building matrices for traffic model')

        ''' Transition matrices '''
        # Compute transition matrix M(\dk) such that
        # zeta(k+\dk) = M(\dk)[xp;xc;y]

        p = self.plant
        c = self.controller
        t = self.trigger

        # TODO: think about how to treat fundamental PETC and
        # abstraction times differently
        h_abs = t.h

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
        Abar = np.block([[p.A, p.B], [np.zeros((nu, nxp+nu))]])
        Phibar = la.expm(Abar*h_abs)  # Think about this h_abs
        Phip = Phibar[0:nxp, 0:nxp]
        Gammap = Phibar[0:nxp, nxp:]

        # Loop to compute Mks
        Phipk = Phip
        Gammapk = Gammap
        Ack = c.A
        Bck = c.B

        Mlist = []
        Nlist = []
        Qlist = []
        maxeig = []
        mineig = []

        if t.must_trigger_at_first_crossing:
            kmax = None
        else:
            kmax = self.kmax

        if not t.triggering_is_time_varying:
            if t.triggering_function_uses_output:
                Qbar_yuyu = t.Qbar
            else:
                Qbar_yuyu = np.zeros((nz*2, nz*2))
                Qbar_yuyu[0:ny, 0:ny] = t.Qbar1
                Qbar_yuyu[0:ny, nz:nz+ny] = t.Qbar2
                Qbar_yuyu[nz:nz+ny, 0:ny] = t.Qbar2.T
                Qbar_yuyu[nz:nz+ny, nz:nz+ny] = t.Qbar3

        for i in tqdm(range(0, t.kmax)):  # TODO: Think about this kmax_abs
            # Transition matrices from [xp, xc, y] after k=i+1 steps
            # [xip(t+kh), xic(t+kh)].T = M(k)[xp, xc, y].T
            # [psi(t+kh); ups(t+kh)].T = N(k)[xp, xc, y].T
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
                Q = np.block([N.T, CE.T]) @ self._Q_time_var(i+1, h_abs) \
                    @ np.block([[N], [CE]])
            else:
                Q = np.block([N.T, CE.T]) @ Qbar_yuyu @ np.block([[N], [CE]])
            # No noise --> no need to separate xp from y, there is redundancy
            if not self.consider_noise:
                # y = Cxp - colapse to only xp-dependency
                IIC = np.zeros((nxp+nxc+ny, nxp+nxc))
                IIC[:-ny, ] = np.eye(nxp+nxc)
                IIC[nxp+nxc:, :nxp] = p.C
                # [xp; xc] = IIC [xp; xc; y]
                Q = IIC.T @ Q @ IIC
                M = M @ IIC
                N = N @ IIC
                # Normalize Q
                Q = Q/la.norm(Q)

            # print(Q)
            Qlist.append(Q)
            Mlist.append(M)
            Nlist.append(N)
            lbd, _ = la.eig(Q)
            maxeig.append(max(np.real(lbd)))
            mineig.append(min(np.real(lbd)))
            # At this point, all states have triggered
            if mineig[-1] > 0 and kmax is None:
                kmax = i+1
                if self.early_trigger_only:
                    self.kmaxextra = kmax
                    break
            if kmax is not None and mineig[-1] <= 0:
                kmax = None
        # end for
        # print([x > -self.min_eig_threshold for x in mineig])
        # print(mineig)
        if kmax is None:  # maximum triggering time prevented finding Q(k) > 0
            kmax = t.kmax
        # Erase Qs up to kmaxextra

        Qlist = Qlist[:kmax]
        if mineig[-1] > -self.min_eig_threshold:
            # Retroactive search of the last k: mineig[k] <= thresh
            for i in range(kmax-1, -1, -1):
                if mineig[i] <= -self.min_eig_threshold:
                    break
            kmax = i+2
            Qlist = Qlist[:kmax]

        try:
            kbeg = next(i for i, l in enumerate(maxeig) if l > 0) + 1
        except StopIteration:
            print(mineig)
            raise ETCAbstractionError(
                f'No triggering would occur up to {self.kmax}-th iteration')

        ''' NEED TO FIGURE THIS OUT ONCE AND FOR ALL '''
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

        self.M = {i+1: m for i, m in enumerate(Mlist) if i+1 <= self.kmaxextra}
        self.N = {i+1: n for i, n in enumerate(Nlist) if i+1 <= self.kmaxextra}
        self.Q = {i+1: q for i, q in enumerate(Qlist)
                  if i+1 >= self.kmin and maxeig[i] > 0}
        self.Q[kend] = np.eye(Qlist[0].shape[0])  # For the last, all states
        # should trigger
        # Normalizing Q requires adjusting non-zero threshold
        if t.threshold is not None and t.threshold != 0:
            raise ETCAbstractionError('Method is not prepared for non-zero'
                                      'threshold in triggering function')

        logging.info('kmin=%d, kmax=%d', self.kmin, self.kmax)

    # time-varying Q for Relaxed PETC, checks just the condition if the
    # Lyapunov function exceeds the bound at the next time instant
    def _Q_time_var(self, k, h):
        nx = self.plant.nx
        ny = self.plant.ny
        nu = self.plant.nu
        nz = ny + nu

        M = np.block([[self.trigger.Ad, self.trigger.Bd @ self.controller.K]])
        Z = np.zeros((nx, nx))
        Pe = self.trigger.P*np.exp(-self.trigger.lbd*k*h)
        Qbar = M.T @ self.trigger.P @ M - np.block([[Z, Z], [Z, Pe]])
        Qbar1 = Qbar[:nx, :nx]
        Qbar2 = Qbar[:nx, nx:]
        Qbar3 = Qbar[nx:, nx:]
        # Qbar1 = self.trigger.P
        # Qbar2 = np.zeros((nx, nx))
        # Qbar3 = -self.trigger.P*np.exp(-self.trigger.lbd*k*h)
        self.Qbar1 = Qbar1
        self.Qbar2 = Qbar2
        self.Qbar3 = Qbar3
        # self.Q = np.block([[Qbar1, Qbar2], [Qbar2.T, Qbar3]])
        Qbar_yuyu = np.zeros((nz*2, nz*2))
        Qbar_yuyu[0:ny, 0:ny] = Qbar1
        Qbar_yuyu[0:ny, nz:nz+ny] = Qbar2
        Qbar_yuyu[nz:nz+ny, 0:ny] = Qbar2.T
        Qbar_yuyu[nz:nz+ny, nz:nz+ny] = Qbar3

        return Qbar_yuyu

    def _reduce_sets(self):
        logging.info('Reducing number of Q, M, and N matrices.')

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
        if self.reduced_actions:
            self.M = {k: m for k, m in self.M.items() if k in self.Q}
            self.N = {k: n for k, n in self.N.items() if k in self.Q}
        if self.early_trigger_only:
            self.M = {k: m for k, m in self.M.items() if k <= max(self.Q)
                      + self.max_delay_steps}
            self.N = {k: n for k, n in self.N.items() if k <= max(self.Q)
                      + self.max_delay_steps}

        # Log resulting reduction in Q space.
        logging.info('Reduced from %d to %d regions',
             self.kmax - self.kmin + 1, len(self.Q))

    def _build_precedence_relation(self):
        r""" Computes in advance which cones for 1:k-1 are contained
        in cone for k. This will reduce the size of the constraint
        satisfaction problem.

        This problem takes the form

        .. math:: \forall x: x^\top Q_1x \leq 0 \implies x^\top Q_2x \leq 0.

        If this is true, the second inequality above is redundant with the
        first, and therefore it can be omitted wherever the first is.

        """

        logging.info('Building precedence relation for quadratic forms')

        Q = self.Q
        # Check quadratic forms ordering
        q = {k: QuadraticForm(x) for k, x in Q.items()}
        predecessors = {}
        for i, Qi in q.items():
            if i == 1:
                predecessors[i] = {}
            if i == 2:
                predecessors[i] = {j for j, Qj in q.items()
                                   if i > j and Qi > Qj}
            else:
                predecessors[i] = set()
                candidates = sorted([j for j in q if j < i])
                while len(candidates) > 0:
                    j = candidates.pop()
                    if Qi > q[j]:
                        predecessors[i].add(j)
                        # include predecessors[j] in predecessors of i
                        predecessors[i].update(predecessors[j])
                        for k in predecessors[j]:
                            if k in candidates:
                                candidates.remove(k)
        # nonpredecessors = {i: {j for j in range(0,i)
        #                        if j not in predecessors[i]}
        #                    for i in predecessors}
        self._predecessors = predecessors
        # self.nonpredecessors = nonpredecessors

        # Logging of simplification statistics
        count_non_ideal = sum(1 for x, p in self._predecessors.items()
                              for y in self.Q if y < x
                              and y not in p)
        logging.info('Average number of preceding regions not subset'
                     ' of the next region: %d',
                     count_non_ideal/len(self.Q))

    def _prepare_matrices_for_transitions(self):
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
        MQM = {x: mqm/la.norm(mqm) for x, mqm in MQM.items()}
        self._MQM = MQM

    def _generate_backward_candidates(self):
        # Update max and min string lengths
        try:
            self._minL = min(len(x) for x in self.regions)
            self._maxL = max(len(x) for x in self.regions)
        except ValueError:
            self._minL, self._maxL = 0, 0
            return set((k,) for k in self.K)

        # Generate all candidates
        # TODO: this is not very smart; can use the Pres instead.
        out_set = set()
        possible_initial = set()
        for (i,k) in itertools.product(self.regions - self._initial, self.K):
            ki = (k,) + i
            if self._substring_exists(ki) and ki not in self._initial:
                # Check if ki's prefix is in _initial
                for s in self._initial:
                    if ki[:len(s)] == s:
                        possible_initial.add(ki)
                        break
                out_set.add(ki)
        return out_set

    def _substring_exists(self, ij):
        """Check if all substrings of ij exist in self.regions.

        Check all substrings of ij of length compatible with lengths in
        self.regions. If one such substring does NOT exist, return False.
        """
        L = len(ij)
        for l in range(min(L, self._maxL), self._minL - 1, -1):
            for m in range(0, L-l):
                s = ij[m:m+l]  # This is the substring to be checked.
                for r in self.regions:
                    lr = len(r)
                    if lr < l and any(r == s[i:lr+i] for i in range(l-lr+1)):
                        break
                    if any(s == r[i:l+i] for i in range(lr - l + 1)):
                        break
                else:
                    return False
        return True

    def _verify_candidates(self, regions):
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

        for r in self.regions:
            if all((k,) + r not in out_set for k in self.K):
                initial_set.add(r)

        return out_set, extended_set, marginal_set, initial_set

    def _verify_sequence(self, s):
        if not self.stop_around_origin:
            # Check whether current string maps into only one of any other
            # string
            n = s[1:]
            if n in self._initial:
                return False, False, False
            if len([sp for sp in self.regions if sp[:-1] == n[:-1]]) == 1:
                return True, True, True

        exists, inside, marginal = False, False, False
        con = self._add_constraints_for_region_i(s, set())

        if self.stop_around_origin:
            Mns = self._M_prod(s, len(s)-1)
            Ms = self.M[s[-1]] @ Mns
            # Last should be in target set
            con.add(QuadraticForm(Ms.T @ self.P @ Ms, c=-self.end_level))
            # The one before the last should not!
            con.add(QuadraticForm(-Mns.T @ self.P @ Mns, c=self.end_level,
                                  strict=True))
        prob = QuadraticProblem(con, solver=self.solver, unit_ball=True)
        if prob.solve():
            exists = True
            # Now check if initial state can belong inside {x: V(x) <= 1}
            if self.stop_around_origin:
                new_cons = set((QuadraticForm(self.P, c=-1),))
                prob.add_constraints(new_cons)
                if prob.solve():
                    inside = True
                    # Now check if there exists x: V(x) = 1
                    new_cons = set((QuadraticForm(-self.P, c=1),)) # V(x) >= 1
                    prob.add_constraints(new_cons)
                    if prob.solve():
                        marginal = True
            else:
                inside, marginal = True, True

        return exists, inside, marginal

    def _build_transition(self):
        # Builds a list of transitions where l[(i,j)] is the list of cones
        # reachable from region i (related to instant i+1) after j+1 steps

        logging.info('Building transition map')

        M = self.M
        dP = self._dP
        R = self.regions

        # Store min and max lengths for later use
        self._minL = min(len(x) for x in self.regions)
        self._maxL = max(len(x) for x in self.regions)

        # Reachability problem: is there x in region i that reaches region j
        # after k+1 sample?
        # Cost comes almost for free here. Use cost computation instead of pure
        # feasibility
        transition = {}
        # (complete_cost[((i,k),j)])
        complete_cost = {}  # for costs depending on reached set j.

        nIK = len(R)*len(M)
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
                            cost_error = abs(1. - abs(cost_low/cost_up))
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
                            cost_low = (cost_low + cost_up)/2.
                            cost_up = cost_low
                        complete_cost[((i, k), j)] = (cost_low, cost_up)
                    except Exception as e:
                        if 'Relaxation problem status: infeasible' in str(e):
                            continue
                        else:
                            raise e
                    transition[(i, k)].add(j)

        self.transition = transition
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
        for l,i in enumerate(i_tuple):
            i_index = i_list.index(i)
            Mprod = self._M_prod(i_tuple, l)
            if i < i_list[-1]:
                MQM = Mprod.T @ self.Q[i] @ Mprod
                if not self._symbolic:
                    MQM = MQM / max(1e-6, min(abs(la.eigvalsh(MQM))))
                con.add(QuadraticForm(- MQM.copy(), strict=True))
            if i >= i_list[1]:
                i_prev = i_list[i_index-1]
                MQM = Mprod.T @ self.Q[i_prev] @ Mprod
                if not self._symbolic:
                    MQM = MQM / max(1e-6, min(abs(la.eigvalsh(MQM))))
                con.add(QuadraticForm(MQM.copy()))
                for p in i_list[:i_index-1]:
                    if p not in self._predecessors[i_prev]:
                        MQM = Mprod.T @ self.Q[p] @ Mprod
                        if not self._symbolic:
                            MQM = MQM / max(1e-6, min(abs(la.eigvalsh(MQM))))
                        con.add(QuadraticForm(MQM.copy()))
        return con

    def _add_constraints_for_reaching_j_after_k(self, j_tuple, k_tuple, con):
        # con += the set of constraints  for M(k)x \in R_j

        j_list = sorted(self.Q)
        MK = self._M_prod(k_tuple, len(k_tuple))
        for l,j in enumerate(j_tuple):
            j_index = j_list.index(j)
            Mprod = self._M_prod(j_tuple, l)
            if j < j_list[-1]:
                MQM = MK.T @ Mprod.T @ self.Q[j] @ Mprod @ MK
                if not self._symbolic:
                    MQM = MQM / min(abs(la.eigvalsh(MQM)))
                con.add(QuadraticForm(-MQM.copy(), strict=True))
            if j >= j_list[1]:
                j_prev = j_list[j_index-1]
                MQM = MK.T @ Mprod.T @ self.Q[j_prev] @ Mprod @ MK
                if not self._symbolic:
                    MQM = MQM / min(abs(la.eigvalsh(MQM)))
                con.add(QuadraticForm(MQM.copy()))
                # Trivially, if p subset s, it also holds for the MQM related
                # cones
                for p in j_list[:j_index-1]:
                    if p not in self._predecessors[j_prev]:
                        MQM = MK.T @ Mprod.T @ self.Q[p] @ Mprod @ MK
                        if not self._symbolic:
                            MQM = MQM / min(abs(la.eigvalsh(MQM)))
                        con.add(QuadraticForm(MQM.copy()))
        return con

    def _M_prod(self, i_tuple, l):
        M = self.M
        if self._symbolic:
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
        self.automaton = TrafficAutomaton(self.regions)

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
        succ = {s:[d for d in self.regions if d[:-1] == s[1:]]
                for s in self.regions}
        pred = {d:[s for s in self.regions if d[:-1] == s[1:]]
                for d in self.regions}
        nondet = set(s for s,ds in succ.items() if len(ds) > 1)

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
        c11 = c1 + c1
        l = len(c2)
        for i in range(l):
            if c2 == c11[i:i+l]:
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
        dPk = dPk/dPk_norm

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
            i, k, j, probMin.value,  min_global_decay)

        maxdecay = min(max_value, max_global_decay)
        mindecay = max(min_value, min_global_decay)

        return (mindecay*dPk_norm, maxdecay*dPk_norm)

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

        alpha = (maxV/minV)**(1./(nV-2))
        V_list = [minV*(alpha**z) for z in range(0, nV-1)]
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
            steps_down = int(np.floor(np.log(1+cost_low)/log_alpha))
            # steps_up = int(np.ceil(np.log(alpha+cost_high)/log_alpha)) - 1
            # It looks like the method above is unnecessarily conservative
            steps_up = int(np.ceil(np.log(1+cost_high)/log_alpha))
            transition_levels[((i, k), j)] = (steps_down, steps_up)

        # Step 2: build the complete reachability map
        complete_transition = {}
        partitions = {}  # For reference, build a list of partitions
        for ((i, k), cone_list) in self.transition.items():
            sampling_time = i  # Using the actual discrete sampling time
            for v in range(0, nV):
                # Set the partition (index 1: discrete time,
                #                    index 2: Lyapunov level interval)
                if v == 0:
                    level_interval = (0, V_list[0])
                elif v == nV-1:
                    level_interval = (V_list[-1], np.inf)
                else:
                    level_interval = (V_list[v-1], V_list[v])

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
        for k in sorted(self.regions):
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
        real_z = np.log(V/Vmin)/np.log(self.alpha)
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
            self.regions.remove(r)
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
                    self.regions.add(r)
                    self._minimal.add(r)
                    break
                r = r_new

        self._initial = set(i for i,c in self.cost.items()
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
        for i in range(self.n-2):
            angles.append(np.arange(0, np.pi, np.pi/N))
        angles.append(np.arange(0, 2*np.pi, np.pi/N))

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
        self.probability_region = {k:0 for k in self.Q}
        # ((region, sample)): {region: prob}
        self.probability_transition \
            = {key: {j: 0 for j in v} for key, v in self.transition.items()}

        # Generate random uniformally distributed numbers
        xs = random.normal(size=(N,self.n))

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
                    self.transition[(i, k)].add(j)
                    dPk = self._dP[k]
                    mincost, maxcost = self._transition_cost(i, k, dPk)
                    self.complete_cost[((i, k), j)] = (mincost, maxcost)
                self.probability_transition[(i, k)][j] += 1

        # Turn counts into probabilities
        return

        # Normalization is not really needed. But then it is not a probability,
        # but rather a "probability weight" as used in UPPAAL.
        self.probability_region = {i:c/N
                                   for i, c in self.probability_region.items()}
        for key, v in self.probability_transition.items():
            total = sum(v.values())
            self.probability_transition[key] = {j:c/total
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

        for i in tqdm(self.regions):
            try:
                if i not in old_cost:
                    self.cost[i] = 1 \
                                   + self._transition_cost(i, i, self._dP[i])[1]
            except ETCAbstractionError:
                to_be_deleted.add(i)
        self.regions.difference_update(to_be_deleted)
        self._initial = set(i for i,c in self.cost.items()
                            if c <= self.end_level and i in self.regions)
        self.regions.update()

    def _dP_of_region(self, r):
        l = len(r)
        m = self._M_prod(r, l)
        return m.T @ self.trigger.P @ m - self.trigger.P

    def _compute_dP(self):
        # Compute dP matrices of regions
        self._dP = {r: self._dP_of_region(r) for r in self.regions}

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
        for (i,k),j_list in self.transition.items():
            if k == i[-1]:
                for j in j_list:
                    # n = len(i)
                    ij = i+j
                    if self._substring_exists(ij):
                        out_set.add(ij)
                    # for m in range(n):
                    #     if ij[m:n+1+m] not in out_list:
                    #         # print(ij[m:n+1+m])
                    #         out_list.append(tuple(ij[m:n+1+m]))
        return out_set

"""
    VALIDATION CODE
"""


def validate_simulation(sim_out: dict,
                        traffic_model: TrafficModel):
    """Validate if traffic_model fits a simulation.

    Parameters
    ----------
    sim_out : dict
        As returned by a ETC/STC simulation
    traffic_model : TrafficModel
        The traffic model to be validated

    Returns
    -------
    validity : bool
        True if valid
    trace : list of tuples
        The timed automaton trace. Each element is
        (sample, region, Lyapunov level set [optional], time)

    """
    xs = sim_out['xphat'][:, sim_out['sample']]
    ttrig = sim_out['t'][sim_out['sample']]
    dt = np.diff(ttrig)
    trace = []
    elapsed_time = 0
    for i in range(len(dt)):
        s = [i]
        xnow = xs[:, i]
        xnext = xs[:, i+1]
        source = traffic_model.region_of_state(xnow)
        s.append(source)
        target = traffic_model.region_of_state(xnext)
        if traffic_model.has_level_sets:
            lv_initial = traffic_model.level_of_state(xnow)
            s.append(lv_initial)
            lv_target = traffic_model.level_of_state(xnext)
        s.append(elapsed_time)
        if traffic_model.is_discrete_time:
            time = int(round(dt[i] / traffic_model.controller.h))
            elapsed_time += time
            if target not in traffic_model.transition[(source, time)]:
                print(f'Failed from {xnow} to {xnext} after {time} time units.'
                      )
                return False
            # else
            if traffic_model.has_level_sets:
                delta_lv = lv_target - lv_initial
                lv_range = traffic_model.transition_levels[((source,
                                                             time), target)]
                if not((lv_range[1] >= delta_lv >= lv_range[0])
                        or (lv_target == 0 and delta_lv >= lv_range[0])):
                    print(f'Failed from {xnow} to {xnext} after {time} time'
                          ' units because of level sets.')
                    return False

        else:
            time = dt
            # TODO: fill in code here
            raise Exception('Not implemented')

        trace.append(tuple(s))
    # Append last state
    s = [i+1]
    s.append(target)
    if traffic_model.has_level_sets:
        s.append(lv_target)
    s.append(elapsed_time)
    trace.append(tuple(s))

    return True, trace


def traffic2ta(traffic: TrafficModelPETC):
    # Locations (set of any hashable type)
    location_set = traffic.regions
    initial_location_set = location_set.copy()

    # Clocks
    clock = sympy.var('c', real=True)
    clock_set = set((clock,))

    # Invariants
    delay = traffic.max_delay_steps
    invariant_dict = {l : (clock <= l[0] + delay) for l in traffic.regions}

    # Actions
    action_set = {'trigger', 'early', 'late'}

    # Transition set
    if not traffic.transition:
        # If transition was not populated, fill transition set with rule from
        # L-CSS paper; all transitions are natural triggers.
        transition_list = [(s,  # Source location
                            sympy.Eq(clock, s[0]),  # Guard
                            'trigger',  # Action
                            clock_set,  # Set of reset clocks
                            d)  # Destination location
                           for s, d in itertools.product(traffic.regions,
                                                        repeat=2)
                           if s[1:] == d[:len(s)-1]]
    else:
        print(traffic.transition)
        transition_list = [(i,  # Source location
                            sympy.Eq(clock, k),  # Guard
                            'trigger' if k == i[0] else 'early',
                            clock_set,  # Set of reset clocks
                            j)  # Destination location
                           for (i,k),j_set in traffic.transition.items()
                           for j in j_set]


    # Costs
    transmission_cost = sympy.var('tr', real=True)
    # TODO: Add the performance cost
    cost_set = set((transmission_cost,))

    # Cost functions
    cost_rate_dict = set()  # If empty, all locations have cost rate 0
    cost_transition_dict = {i:{transmission_cost: 1}
                            for i, (s, g, a, c, d)
                            in enumerate(transition_list)
                            if a in ('trigger', 'early', 'late')}

    ''' Set descriptors '''
    set_descriptor_dict = dict()
    x = np.array(sympy.symbols(f'x1:{traffic.n+1}', real=True))
    for l in traffic.regions:
        constraints = traffic._add_constraints_for_region_i(l, set())
        sympy_expressions = [c.sympy_expression(x) for c in constraints]
        set_descriptor_dict[l] = sympy.And(*sympy_expressions)

    return PTA(location_set, initial_location_set, action_set, clock_set,
               transition_list, invariant_dict, cost_set, cost_rate_dict,
               cost_transition_dict), set_descriptor_dict


if __name__ == '__main__':
    # from tests.scenarios.simple import *
    # kmax_abs = 15
    # #kmax_abs = 15
    # #kmax_abs = 4
    # h_abs = 0.05
    # # h_abs = 0.2
    # with open('./tests/scenarios/oscillatory.py') as f:
    #     code = compile(f.read(), "oscillatory.py", 'exec')
    #     exec(code)  #, global_vars, local_vars)

    # from tests.scenarios.five_pools import *
    # kmax_abs = 20
    # h_abs = h

    print(NUM_CORES)

    from tests.scenarios.simple import p, K, Pl, Ql, rho, x0
    import etcsim

    kmax_abs = 10
    h_abs = 0.01
    #p.A[1,0] = -3
    #K[0,0] = 2

    Ap = p.A
    Bp = p.B

    # New: real eigenvalues... would it stop?
    #K = np.array([[0,-6]])
    #K = np.array([[0.5, -5]])
    controller = etc.LinearController(K, 0.0001)

    '''Find the largest period such that the same Lyapunov inequality holds'''
    if False:
        dt = 0.0001
        rho = 0.01
        for t in np.arange(0, 1, dt):
            pc = etc.PeriodicLinearController(p, controller, t)
            Phi = pc.Phi
            M = Ap @ Phi + Bp @ K
            Pt = M.T @ Pl @ Phi + Phi.T @ Pl @ M
            Pt = (Pt + Pt.T)/2
            Qt = Phi.T @ Ql @ Phi
            Qt = (Qt + Qt.T)/2
            delta = Pt + rho*Qt
            if any(np.real(la.eig(delta)[0]) > 0):
                break
        h = t - dt
        print(h)

    h = 0.25
    controller.h = h
    kmax = 2
    trig = etc.DirectLyapunovPETC(p, controller, P=Pl, Q=Ql, rho=rho,
                                  h=h, kmax=kmax, predictive=True)


    if CDC_2020:
            # Check exponential decay rate
        b, Ps = trig.check_stability_pwa()
        print(f'b = {b}')

        '''Find the largest period such that the Lyaopunov function decreases
        over samples'''
        # (MPETC of the CDC 2020 paper)
        dt = 0.001
        for t in np.arange(0, 1, dt):
            pc = etc.PeriodicLinearController(p, controller, t)
            Phi = pc.Phi
            dP = Phi.T @ Pl @ Phi - Pl
            if any(np.real(la.eigvals(dP)) > 0):
                break

        hstar = t - dt
        print(hstar)
        hstar = np.floor(hstar*100)/100

    # For the CDC paper
    r = 0.1  # Paper: 0.1
    controller.h = h
    # x0 = np.array([-1, 0.5])
    t0, t1 = 0, 20
    step = 0.01
    t = np.arange(t0, t1, step)

    w = None  # lambda t: np.array([0*np.sin(t)])
    out = etcsim.simulate_sample_and_hold_control(trig, t, x0, [],
                                                  disturbance=w)
    ks = np.array([x for x in range(len(t))])
    ks = ks[out['sample']]
    dk = np.diff(ks)
    print(np.round(dk*step/trig.h))
    xs = out['xp']
    xs = xs[:,out['sample']]

    if CDC_2020:
        # Check Lyap
        V = [x.T @ Pl @ x for x in xs.T]
        for i,v in enumerate(V):
            if v < r:
                break
        kf = ks[i]
        x0p = xs[:,i]
        t0p = t[kf]
        tp = np.arange(t0p, 5, 0.01) - t0p
        peri = etc.PeriodicLinearController(p, controller, hstar)
        outp = etcsim.simulate_sample_and_hold_control(peri, tp, x0p, [],
                                                    disturbance=w)
        xall = np.concatenate((out['xp'][:,:kf], outp['xp']), axis=1)
        ksp = np.array([x for x in range(len(tp))]) + kf
        ksp = ksp[outp['sample']]
        kstotal = np.concatenate((ks[:i], ksp))
        xsp = outp['xp'][:,outp['sample']]


        import matplotlib.pyplot as plt
        # plt.plot(np.arange(t0, 5, 0.01), xall.T)
        Vall = np.array([x.T @ Pl @ x for x in xall.T])
        plt.plot(np.arange(t0, 5, 0.01), Vall)
        plt.plot(t[ks[:i+1]], [x.T @ Pl @ x for x in xs[:,:i+1].T], 'rx')
        plt.plot(t[ksp][1:], [x.T @ Pl @ x for x in xsp[:,1:].T], 'ro',
                 fillstyle='none')
        # plt.gca().set_yscale('log')

        plt.xlabel('$t$')
        plt.ylabel('\$V(\\xiv(t))\$')
        plt.legend(('V', 'PETC', 'Periodic'))

        import tikzplotlib as tikz
        tikz.save('mpetc.tex')

    '''Now construct the traffic model'''
    controller.h = trig.h
    t = time.time()
    traffic = TrafficModelPETC(trig,
                               mu_threshold=0.00,
                               min_eig_threshold=0,
                               reduced_actions=False,
                               depth=10,
                               etc_only=False,
                               early_trigger_only=True,
                               end_level=r,
                               solver='z3',
                               stop_around_origin=False,
                               stop_if_omega_bisimilar=False)

    # NOTE: Reduced actions seems to degrade resulting strategy severely
    # traffic.add_level_sets(0.01, 10, 100)
    print('Elapsed: %.2f seconds' % (time.time() - t))

    x0 = np.array([-1, 0.5])
    t0, t1 = 0, 20
    t = np.arange(t0, t1, trig.h)

    w = None  # lambda t: np.array([0*np.sin(t)])
    out = etcsim.simulate_sample_and_hold_control(trig, t, x0, [], disturbance=w)
    ks = np.array([x for x in range(len(t))])
    ks = ks[out['sample']]
    dk = np.diff(ks)
    print(dk)

    ''' For the tool '''
    if OUTPUT_PTA:
        ta = traffic2ta(traffic)

        # To pickle
        ((location_set, initial_location_set, action_set, clock_set,
        transition_list, invariant_dict, cost_set, cost_rate_dict,
        cost_transition_dict), set_descriptor_dict) = ta

        import pickle

        with open('ta.pickle', 'wb') as f:
            pickle.dump((location_set, initial_location_set, action_set, clock_set,
                         transition_list, invariant_dict, cost_set, cost_rate_dict,
                         cost_transition_dict, set_descriptor_dict),
                        f, pickle.HIGHEST_PROTOCOL)

        with open('ta.pickle', 'rb') as f:
            (location_set, initial_location_set, action_set, clock_set,
             transition_list, invariant_dict, cost_set, cost_rate_dict,
             cost_transition_dict, set_descriptor_dict) = pickle.load(f)

    if IFAC_WC_2020:
        from tests.scenarios.batch_reactor import *
        kmax_abs = 20
        h_abs = 0.01
        # h_abs = 0.1  # To reduce the number of regions (testing purposes)

        if __DEBUG__:
            lv = logging.DEBUG
        else:
            lv = logging.INFO

        logger = logging.getLogger()
        logger.setLevel(lv)
        ch = logging.StreamHandler()
        ch.setLevel(lv)
        ch.setFormatter(
            logging.Formatter('[%(levelname)s - %(filename)s:%(lineno)s - '
                              '%(funcName)s()] %(message)s'))
        if len(logger.handlers) <= 1:
            logger.addHandler(ch)

        h = h_abs
        kfinal = kmax_abs

        # For fast testing
    #    sigma = sigma
    #    kfinal = 4

        # Compute lyapunov matrices
        Q = Q_lqr + K.T @ R_lqr @ K
        Q = (Q + Q.T)/2

        Acl = Ap + Bp @ K
        P = ct.lyap(Acl.T.copy(), Q.copy())

        plant = etc.LinearPlant(Ap, Bp, Cp)
        controller = etc.LinearController(K, h)
        # sigma = etc.TabuadaPETC(plant, controller).max_sigma*0.9
        # trig = etc.TabuadaPETC(plant, controller, None, None,
        #                        sigma, 0, None, kfinal)
        # trig = etc.RelaxedPETC(plant, controller, Pl, lbd, kmin=8, kmax=kmax_abs)
        '''Find the largest period such that the same Lyapunov inequality holds'''

        dt = 0.0001
        rho = 0.8
        for t in np.arange(0, 1, dt):
            pc = etc.PeriodicLinearController(plant, controller, t)
            Phi = pc.Phi
            M = Ap @ Phi + Bp @ K
            Pt = M.T @ P @ Phi + Phi.T @ P @ M
            Pt = (Pt + Pt.T)/2
            Qt = Phi.T @ Q @ Phi
            Qt = (Qt + Qt.T)/2
            delta = Pt + rho*Qt
            if any(np.real(la.eig(delta)[0]) > 0):
                break
        h = t - dt
        print(h)

        controller.h = 0.01
        trig = etc.DirectLyapunovPETC(plant, controller, P=P, Q=Q, rho=rho,
                                      h=0.01)

        t = time.time()
        traffic = TrafficModelPETC(trig,
                                   mu_threshold=0.00,
                                   min_eig_threshold=1e-3,
                                   reduced_actions=False,
                                   early_trigger_only=True,
                                   stop_around_origin=False,
                                   stop_if_omega_bisimilar=False)
        # NOTE: Reduced actions seems to degrade resulting strategy severely
        # traffic.add_level_sets(0.01, 10, 100)
        print('Elapsed: %.2f seconds' % (time.time() - t))

        if traffic.cost_computation:
            n_partitions = len(traffic.partitions)
            n_actions = traffic.kmaxextra
            n_transitions_max = n_partitions ** 2 * n_actions
            n_transitions_min = n_partitions * n_actions
            n_transitions = sum(len(x) for _, x in traffic.complete_transition.items())
            transition_density = n_transitions/n_transitions_max
            print('Transition density is of %.2f%%' % (100.0 * transition_density))
            average_non_determinism = n_transitions/n_transitions_min
            print('Average non-determinism is of %.2f' % average_non_determinism)

        import pickle

        with open('decay.pickle', 'wb') as f:
            pickle.dump(traffic, f, pickle.HIGHEST_PROTOCOL)
        # with open('test.pickle', 'wb') as f:
        #     pickle.dump(traffic, f, pickle.HIGHEST_PROTOCOL)
        # with open('simple.pickle', 'wb') as f:
        #     pickle.dump(traffic, f, pickle.HIGHEST_PROTOCOL)
        # costrate = {(i,k): (traffic.cost[(i,k)][0]/(k+1),
        #                     traffic.cost[(i,k)][1]/(k+1))
        #             for i,k in traffic.cost}

        # A copy of the plant with a different, more aggressive Controller
        R_lqr_2 = 0.05*np.eye(2)
        K2,_,Eigs = ct.lqr(Ap,Bp,Q_lqr,R_lqr_2)
        K2 = -K2

        controller2 = etc.LinearController(K2, 0.0001)

        # Compute lyapunov matrices
        Q2 = Q_lqr + K2.T @ R_lqr_2 @ K2
        Q2 = (Q2 + Q2.T)/2

        Acl2 = Ap + Bp @ K2
        P2 = ct.lyap(Acl2.T.copy(), Q2.copy())

        # sigma = etc.TabuadaPETC(plant, controller).max_sigma*0.9
        # trig = etc.TabuadaPETC(plant, controller, None, None,
        #                        sigma, 0, None, kfinal)
        # trig = etc.RelaxedPETC(plant, controller, Pl, lbd, kmin=8, kmax=kmax_abs)
        '''Find the largest period such that the same Lyapunov inequality holds'''

        dt = 0.0001
        rho = 0.8
        for t in np.arange(0, 1, dt):
            pc = etc.PeriodicLinearController(plant, controller2, t)
            Phi = pc.Phi
            M = Ap @ Phi + Bp @ K2
            Pt = M.T @ P2 @ Phi + Phi.T @ P2 @ M
            Pt = (Pt + Pt.T)/2
            Qt = Phi.T @ Q2 @ Phi
            Qt = (Qt + Qt.T)/2
            delta = Pt + rho*Qt
            if any(np.real(la.eig(delta)[0]) > 0):
                break
        h2 = t - dt
        print(h2)

        controller2.h = 0.01
        trig2 = etc.DirectLyapunovPETC(plant, controller2, P=P2, Q=Q2, rho=rho,
                                       h=0.01)
        t = time.time()
        traffic2 = TrafficModelPETC(trig2,
                                    mu_threshold=0.00,
                                    min_eig_threshold=1e-3,
                                    reduced_actions=False,
                                    early_trigger_only=True,
                                    stop_around_origin=False,
                                    stop_if_omega_bisimilar=False)
        # NOTE: Reduced actions seems to degrade resulting strategy severely
        # traffic.add_level_sets(0.01, 10, 100)
        print('Elapsed: %.2f seconds' % (time.time() - t))

        with open('decay2.pickle', 'wb') as f:
            pickle.dump(traffic2, f, pickle.HIGHEST_PROTOCOL)

        from tests.scenarios.simple import *
        kmax_abs = 15
        #kmax_abs = 15
        #kmax_abs = 4
        h_abs = 0.01
        # h_abs = 0.2

        plant2 = p
        Ap = plant2.A
        Bp = plant2.B
        controller3 = etc.LinearController(K, 0.0001)

        # sigma = etc.TabuadaPETC(plant, controller).max_sigma*0.9
        # trig = etc.TabuadaPETC(plant, controller, None, None,
        #                        sigma, 0, None, kfinal)
        # trig = etc.RelaxedPETC(plant, controller, Pl, lbd, kmin=8, kmax=kmax_abs)
        '''Find the largest period such that the same Lyapunov inequality holds'''

        dt = 0.0001
        rho = 0.8
        for t in np.arange(0, 1, dt):
            pc = etc.PeriodicLinearController(plant2, controller3, t)
            Phi = pc.Phi
            M = Ap @ Phi + Bp @ K
            Pt = M.T @ Pl @ Phi + Phi.T @ Pl @ M
            Pt = (Pt + Pt.T)/2
            Qt = Phi.T @ Ql @ Phi
            Qt = (Qt + Qt.T)/2
            delta = Pt + rho*Qt
            if any(np.real(la.eig(delta)[0]) > 0):
                break
        h2 = t - dt
        print(h2)

        controller3.h = 0.01
        trig3 = etc.DirectLyapunovPETC(plant2, controller3, P=Pl, Q=Ql, rho=rho,
                                       h=0.01, kmax=20)
        t = time.time()
        traffic3 = TrafficModelPETC(trig3,
                                    mu_threshold=0.00,
                                    min_eig_threshold=1e-3,
                                    reduced_actions=False,
                                    early_trigger_only=True,
                                    stop_around_origin=False,
                                    stop_if_omega_bisimilar=False)
        # NOTE: Reduced actions seems to degrade resulting strategy severely
        # traffic.add_level_sets(0.01, 10, 100)
        print('Elapsed: %.2f seconds' % (time.time() - t))

        with open('decay3.pickle', 'wb') as f:
            pickle.dump(traffic3, f, pickle.HIGHEST_PROTOCOL)









