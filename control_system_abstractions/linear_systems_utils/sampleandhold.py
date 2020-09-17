#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 19 15:02:31 2018

@author: ggleizer
"""
import numpy as np
from numpy import random
import control as ct
import warnings
import scipy.linalg as la
import cvxpy as cvx

from .linearsys import Plant, LinearPlant, Controller, LinearController
from .etcutil import normalize_state_to_lyapunov, is_positive_definite

''' Important to prevent legacy Matrix object returned from control lib'''
ct.use_numpy_matrix(flag=False)

''' Settings for solvers '''
_MAX_TRIES_TIGHT_LYAPUNOV = 10
_ASSERT_SMALL_NUMBER = 1e-6
_LMIS_SMALL_IDENTITY_FACTOR = 1e-6


''' sampleandhold.py '''
''' Base classes and functions for sample-and-hold control, including
    periodic control.'''



'''
    ETC ERRORS
'''


class ETCWarning(Warning):
    pass


class ETCError(Exception):
    pass


'''
    BASE CLASS (SAMPLING STRATEGIES)
'''

class SampleAndHoldController:
    plant: Plant
    controller: Controller
    is_discrete_time = False
    _sampling_type = 'Generic'

    def __init__(self, plant: Plant, controller: Controller):
        assert plant.ny == controller.ny and plant.nu == controller.nu, \
            'Controller and plant have inconsistent dimensions'
        self.plant = plant
        self.controller = controller
        self.is_state_feedback = plant.states_as_output and \
            not controller.is_dynamic


class LinearSampleAndHoldController(SampleAndHoldController):
    P: np.array  # Lyapunov matrix
    Qlyap:  np.array  # Continuous-time Lyapunov decay matrix

    def __init__(self, plant, controller, P=None, Q=None):
        super().__init__(plant, controller)

        nx = plant.nx
        Acl = plant.A + plant.B @ controller.K  # Closed loop matrix

        # Get/make Lyapunov info
        if P is None:
            if Q is None:
                Q = np.eye(nx)
            else:
                assert (Q == Q.T).all() and is_positive_definite(Q), \
                    'Q must be symmetric positive definite'
            P = ct.lyap(Acl.T, Q)
            if not is_positive_definite(P):
                msg = 'Closed loop system is not stable in continuous-time'
                warnings.warn(ETCWarning(msg))
        else:  # P is given
            print(P)
            print(P.T)
            print('checking',(P == P.T).all())
            assert (P == P.T).all() and is_positive_definite(P), \
                'Provided P matrix is not symmetric positive definite'
            if Q is None:
                Q = - Acl.T @ P - P @ Acl
                if not is_positive_definite(Q):
                    msg = 'Provided P matrix is not a continuous-time' \
                        ' Lyapunov matrix'
            else:
                assert la.norm(Acl.T@P + P@Acl + Q) <= _ASSERT_SMALL_NUMBER, \
                    'Provided P and Q do not satisfy the continuous-time' \
                    ' Lyapunov equation'

        self.P = P
        self.Qlyap = Q

    def fp(self, t, x, uhat, disturbance=None):
        if disturbance is not None:
            return self.plant.A @ x + self.plant.B @ uhat + \
                self.plant.E @ disturbance(t)
        else:
            return self.plant.A @ x + self.plant.B @ uhat

    def fc(self, t, x, yhat):
        return self.controller.A @ x + self.controller.B @ yhat

    def evaluate_run(self, sim_out, target_level=0.01):
        """Generate metrics of a simulation.

        Compute metrics of a simulation with respect to the current
        controller, such as time to reach the target Lyapunov level set
        and the number of samples to do so.

        Parameters
        ----------
        sim_out : dict
            As returned by a ETC/STC simulation

        Returns
        -------
        time_to_reach : float
            Time it took to reach level zero
        sample_count : int
            Number of samples before reaching level zero
        """
        n = len(sim_out['t'])
        for i in range(n):
            x = sim_out['xphat'][:, i]
            if x @ self.P @ x <= target_level:
                break
        else:
            raise ETCError('State never reached target Lyapunov value:',
                           f' Target is {target_level}, last value was'
                           f' {x @ self.P @ x}.')

        return sim_out['t'][i], sum(sim_out['sample'][:i+1])

    def evaluate(self, N=1000, T=2, seed=None, initial_level=10,
                 target_level=0.01, **kwargs):
        """Evaluate the controller's performance.

        Compute metrics for N randomly generated initial conditions.
        The initial conditions satisfy x(0)'Px(0) = V_max.

        Parameters
        ----------
        N : int, optional
            Number of simulations. The default is 1000.
        T : float, optional
            Max time of each simulation. The default is 2.
        seed : int, optional
            Seed for random number generation. The default is None,
            thus rng is not called.
        initial_level : float, optional.
            The desired initial value of the Lyapunov function. The
            default is 10.
        target_level : float, optional.
            The target value of the Lyapunov function. The default is
            0.01
        **kwargs : optional.
            Additional parameters for the simulation

        Returns
        -------
        time_to_reach_array : np.array(float)
            Time it took to reach level zero for each simulation.
        sample_count_array : np.array(int)
            Number of samples before reaching level zero for each
            simulation.
        """
        import etcsim

        nx = self.plant.nx
        if seed is not None:
            random.seed(seed)

        x0_array = random.random_sample((N, nx))

        # Preallocate the output variables
        time_to_reach_array = np.zeros(N)
        sample_count_array = np.zeros(N)

        # Time array
        t = np.arange(0, T, self.h)

        # Main loop
        for i in range(N):
            x0 = x0_array[i, :]
            x0 = normalize_state_to_lyapunov(x0, self.P, initial_level)
            xc0 = np.zeros((self.controller.nx,))
            # print(x0 @ self.P @ x0)
            sim_out = etcsim.simulate_sample_and_hold_control(self, t, x0, xc0,
                                                              **kwargs)
            try:
                time_to_reach, sample_count = \
                    self.evaluate_run(sim_out, target_level=target_level)
            except ETCError as e:
                print(x0)
                raise e
            time_to_reach_array[i] = time_to_reach
            sample_count_array[i] = sample_count

        return time_to_reach_array, sample_count_array



'''
    PERIODIC CONTROL CLASSES
'''


class PeriodicController(SampleAndHoldController):
    """Periodic Controller."""
    is_discrete_time = True
    _sampling_type = 'Periodic'

    def __init__(self, plant: Plant, controller: Controller, h=None):
        if not controller.is_discrete_time:
            if controller.is_dynamic:
                raise ETCError('Dynamic controller needs to be discrete-time')
            if h is None:
                raise ETCError('h must be provided if controller is not'
                               ' discrete-time.')
        super().__init__(plant, controller)

        if h is None:
            self.h = controller.h
        else:
            self.h = h
            self.controller.h = h

    def trigger(*args, **kwargs):
        return True


class PeriodicLinearController(PeriodicController,
                               LinearSampleAndHoldController):
    """Periodic Linear Controller of Linear Plant."""

    def __init__(self, plant: Plant, controller: Controller, h=None,
                 P=None, Q=None):
        PeriodicController.__init__(self, plant, controller, h)
        LinearSampleAndHoldController.__init__(self, plant, controller, P, Q)

        # Get discrete-time matrices
        nx = plant.nx
        nu = plant.nu
        s = la.expm(np.block([[plant.A, plant.B],
                              [np.zeros((nu, nx + nu))]])*h)
        self.Ad = s[0:nx, 0:nx]
        self.Bd = s[0:nx, nx:nx+nu]
        self.Phi = self.Ad + self.Bd @ controller.K


'''
    UTILITY PERIODIC FUNCTIONS
'''


def most_economic_periodic(plant: LinearPlant,
                           controller: LinearController,
                           h_step: float, h_max: float):
    h_best = 0
    decay = 1
    for h in np.arange(h_step, h_max, h_step):
        p = PeriodicLinearController(plant, controller, h)
        a = np.max(np.abs(la.eig(p.Phi)[0]))
        if a < decay:
            decay = a
            h_best = h
    if decay >= 1:
        raise ETCError('No stable periodic implementation for given'
                       ' parameters. Try reducing h_step and h_max.')
    return h_best, np.log(decay)/h_best


def tight_lyapunov(pc: PeriodicLinearController):
    """Compute a tight Lyapunov matrix for a periodic controller.

    Parameters
    ----------
    pc : PeriodicLinearController
        The periodic linear controller

    Raises
    ------
    ETCError
        If it fails to find a tight one.

    Returns
    -------
    Pl : np.array
        The output Lyapunov matrix
    lbd_max : TYPE
        The maximum decay associated with it.
    works_for_ct : TYPE
        True if it this Lyapunov matrix also satisfies the Lyapunov
        equation for the continuous-time implementation.

    """
    _MULTIPLIER = 1.0001

    nx = pc.plant.nx
    factor = 1

    Adcl = pc.Phi
    Acl = pc.plant.A + pc.plant.B @ pc.controller.K
    rho_max = np.abs(np.max(la.eig(Adcl)[0]))**2
    Pvar = cvx.Variable(shape=(nx, nx), PSD=True)
    objective = 0  # cvx.norm(Pvar)
    constraints = [Pvar >> np.eye(nx),
                   Adcl.T @ Pvar @ Adcl << factor * rho_max * Pvar,
                   Acl.T @ Pvar + Pvar @ Acl << 0]  # Works in c.t.
    solving = True
    works_for_ct = True
    attempts = 0
    while solving and attempts <= _MAX_TRIES_TIGHT_LYAPUNOV:
        prob = cvx.Problem(cvx.Minimize(objective), constraints)
        prob.solve(eps=1e-6, max_iters=10000)
        if 'inaccurate' in prob.status:
            attempts += 1
        if prob.status == 'infeasible':
            if len(constraints) <= 2:
                attempts += 1
                factor *= _MULTIPLIER
                constraints[1] = Adcl.T @ Pvar @ Adcl \
                    << factor * rho_max * Pvar
            constraints = constraints[:2]
            works_for_ct = False
        else:
            solving = False
    if Pvar.value is None:
        raise ETCError('Tight Lyapunov failed,'
                       ' problem may be ill-conditioned.')

    Pl = Pvar.value
    Pl = (Pl + Pl.T)/2
    Ql = -(Acl.T @ Pl + Pl @ Acl)

    lbd_max = min(np.real(la.eig(Ql, Pl)[0]))
    lbd_max_dt = -np.log(rho_max)/pc.h
    lbd_max = max(lbd_max, lbd_max_dt)

    return Pl, lbd_max, works_for_ct


'''
     TESTS
'''

if __name__ == '__main__':
    from tests.scenarios.batch_reactor import Ap, Bp, Cp, K

    h = 0.01
    plant = LinearPlant(Ap, Bp, Cp)
    controller = LinearController(K, h)

    ''' Periodic '''
    periodic = PeriodicLinearController(plant, controller, 0.08)
    results = periodic.evaluate(N=10)
