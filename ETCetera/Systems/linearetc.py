#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 19 15:02:31 2018

@author: ggleizer
"""
import numpy as np
import control as ct
import warnings
import scipy.linalg as la
import cvxpy as cvx

from .linearsys import LinearPlant, LinearController
from .sampleandhold import SampleAndHoldController, \
    LinearSampleAndHoldController, PeriodicLinearController
from .etcutil import is_positive_definite


''' Important to prevent legacy Matrix object returned from control lib'''
ct.use_numpy_matrix(flag=False)

''' Settings for solvers '''
_MAX_TRIES_TIGHT_LYAPUNOV = 10
_ASSERT_SMALL_NUMBER = 1e-6
_LMIS_SMALL_IDENTITY_FACTOR = 1e-6

''' linearetc.py '''
''' Main building blocks for ETC implementations'''


'''
    ETC ERRORS
'''


class ETCWarning(Warning):
    pass


class ETCError(Exception):
    pass


'''
    ETC CLASSES
'''


class ETC(SampleAndHoldController):
    threshold = 0.0
    is_dynamic = False
    _sampling_type = 'ETC'
    must_trigger_at_first_crossing = True  # By default, most of them do

    # Every ETC must have a triggering condition, that returns true if
    # sampling is necessary. This is just a template
    def trigger(self, dt=None, x=None, xhat=None, y=None, yhat=None, u=None,
                uhat=None, t=None, *args):
        return True


class LinearETC(ETC, LinearSampleAndHoldController):
    pass


class LinearQuadraticETC(LinearETC):
    Qbar = np.array([])
    Qbar1 = np.array([])
    Qbar2 = np.array([])
    Qbar3 = np.array([])


class LinearPETC(LinearETC):
    kmin: int
    kmax: int
    h: float
    is_discrete_time = True
    triggering_function_uses_output = False
    triggering_is_time_varying = False

    def __init__(self, plant: LinearPlant,
                 controller: LinearController,
                 h=None, P=None, Q=None, kmin=1, kmax=50):
        super().__init__(plant, controller, P, Q)

        assert controller.is_discrete_time, \
            'Controller needs to be discrete-time'

        self.kmin = kmin
        if h is None:
            h = controller.h
        self.h = h
        if kmax is not None:
            self.kmax = kmax
        else:
            self.kmax = 50

        '''If provided P matrix is not a valid continuous-time Lyapunov
        matrix, for PETC it also suffices that it is a valid discrete-time
        Lyapunov matrix. The same goes even if the continuous-time controller
        is unstable. (TBP/TBC)
        '''
        nx = plant.nx
        p = PeriodicLinearController(plant, controller, h=h*kmin, P=P)
        controller.h = h  # Revert back because the statement above changed it
        Phi = p.Phi

        # Get/make Lyapunov info based on discrete-time Lyap condition
        P = self.P
        if not is_positive_definite(P):
            if Q is None:
                Q = np.eye(nx)
            else:
                assert (Q == Q.T).all() and is_positive_definite(Q), \
                    'Q must be symmetric positive definite'
            P = ct.dlyap(Phi.T, Q)
            assert (P == P.T).all() and is_positive_definite(P), \
                'Closed-loop system is not stable'
        else:  # P is good in continuous time
            assert (P == P.T).all() and is_positive_definite(P), \
                'Provided P matrix is not symmetric positive definite'
            if Q is None:
                Q = -(Phi.T @ P @ Phi - P)
                assert is_positive_definite(Q), \
                    'Provided P matrix is not a Lyapunov matrix'
            else:
                assert (Phi.T @ P @ Phi - P == -Q).all(), \
                    'Provided P and Q do not satisfy the discrete-time' \
                    ' Lyapunov equation'
        self.P = P
        self.Qd = Q  # Qd for the discrete-time
        self.Phi = Phi
        self.Ad = p.Ad
        self.Bd = p.Bd


class LinearQuadraticPETC(LinearQuadraticETC, LinearPETC):
    r"""
    A linear quadratic periodic event-triggered controller.

    Considers a quadratic event-triggering mechanism of the form

    .. math:: z(t_i)^\top\bar{Q}z(t_i) > \epsilon

    where :math:`t_i = ih` is the time according to a fundamental sampling
    time h, :math:`\bar{Q}` is the triggering matrix, :math:`\epsilon` is
    a non-negative threshold parameter (0 for asymptotic stability of the
    origin), and :math:`z` is a vector composed of (a) :math:`(x, \hat{x})` if
    the controller is state-feedback, or (b) :math:`(y, u, \hat{y}, \hat{u})`
    if the controller is output-feedback. Above, x, y, and u represent
    plant state, plant output and control input, respectively, and the hat
    variables indicate the held versions of the variables. See [1]_
    for more information on (quadratic) PETC.

    Note: only state-feedback is currently implemented.

    Parameters
    ----------
    plant : LinearPlant
        The plant to be controller
    controller : LinearController
        The controller (must be discrete-time)
    h : float, optional
        The fundamental sampling time for the triggering condition. If none
        is provided, controller.h is used.
    P, Q : array_like, optional
        Lyapunov matrices satisfying V = x'Px and V(k+1) - V(x) <= -x'Qx.
        If either P or Q is not provided, or if they fail to satisfy the
        condition above, then the constructor tries to compute it/them.
    kmin : int, optional
        Minimum inter-event time, in discrete-time. Default: 1
    kmax : int, optional
        Maximum inter-event time, in discrete-time. Default: 50
    Qbar : array_like, optional
        Triggering matrix. Defaults to [I -I; -I I], which always triggers.
    threshold : float, optional
        Triggering threshold :math:`\epsilon`. Default: 0.0.

    Attributes
    ----------
    plant : LinearPlant
        The plant to be controller
    controller : LinearController
        The controller (must be discrete-time)
    h : float
        The fundamental sampling time for the triggering condition.
    P, Qd : numpy.array
        Lyapunov matrices satisfying V = x'Px and V(k+1) - V(x) = -x' Qd x.
    Qlyap : numpy.array
        Lyapunov matrix satisfying V = x'Px and dV/dt = -x' Qlyap x.
    kmin : int
        Minimum inter-event time, in discrete-time.
    kmax : int
        Maximum inter-event time, in discrete-time.
    Qbar : numpy.array
        Triggering matrix.
    Qbar1, Qbar2, Qbar3 : numpy.array
        Submatrices satisfying Qbar = [Qbar1, Qbar2; Qbar2' Qbar3]
    threshold : float
        Triggering threshold :math:`\epsilon`.
    Phi : numpy.array
        Closed-loop state transition matrix if sampling: x(k+1) = Phi @ x(k)
    Ad, Bd : numpy.array
        Zero-order-hold state evolution matrices for the plant: x(k+1) =
        Ad @ x(k) + Bd @ u(k)
    must_trigger_at_first_crossing : boolean
        Whether if the triggering condition must trigger at the first time
        it fires for stability, or if it could wait for a second firing if
        the condition ceases to be satisfied.

    Methods
    -------
    check_stability_pwa(eps=1e-3)
        Returns true if PETC is stable following the PWA approach of [1]_.
    check_stability_kmax(P)
        Check the maximum discrete-time decay rate if kmax<inf [2]_.


    References
    ----------

    .. [1] Heemels, W.P.M.H., Donkers, M.C.F., and Teel, A.R. (2013).
       Periodic event-triggered control for linear systems. IEEE Transactions
       on Automatic Control, 58(4), 847-861.

    .. [2] G. A. Gleizer and M. Mazo, "Towards Traffic Bisimulation of
       Linear Periodic Event-Triggered Controllers," in IEEE Control Systems
       Letters, vol. 5, no. 1, pp. 25-30, Jan. 2021,
       doi: 10.1109/LCSYS.2020.2999177

    """

    def __init__(self, plant: LinearPlant,
                 controller: LinearController,
                 h=None, P=None, Q=None, kmin=1, kmax=50, Qbar=None,
                 threshold=0.0):
        # print('lqpetc')
        LinearPETC.__init__(self, plant, controller, h, P, Q, kmin, kmax)

        nx = self.plant.nx
        if Qbar is not None:
            Qbar = np.array(Qbar)
            if self.is_state_feedback:
                assert Qbar.shape == (2*nx, 2*nx), 'Qbar must be 2nx x 2nx'
                ' for state-feedback controllers.'
                self.Qbar = Qbar
                # get the matrix blocks
                self.Qbar1 = self.Qbar[:nx, :nx]
                self.Qbar2 = self.Qbar[:nx, nx:]
                self.Qbar3 = self.Qbar[nx:, nx:]

            else:
                raise ETCError('Output-feedback PETC is yet to be implemented')
        else:
            ''' Build dummy triggering matrix that always triggers '''
            Qbar1 = np.eye(nx)
            Qbar2 = -np.eye(nx)
            Qbar3 = np.eye(nx)
            self.Qbar1 = Qbar1
            self.Qbar2 = Qbar2
            self.Qbar3 = Qbar3
            self.Qbar = np.block([[Qbar1, Qbar2], [Qbar2.T, Qbar3]])

    def check_stability_impulsive(self, P = None, rho = None):
        r"""Check GES using the impulsive method of [1]


        Check global exponential stability of the closed-loop PETC
        by using Corollary III.3 in [1], which is based on an impulsive-
        systems modeling method. Using their remarks, it is the pair
        of LMIS, for some mu_i >= 0,

        .. math:: e^{-2\rho h} P + (-1)^i \mu_i Q + A_i' Q A_i >= 0,
            i \in {1,2}   \quad (1)

        where

        A_1 = [A + BK,  0; I,  0],
        A_2 = [A,  BK; 0,  I].

        Note that, since lambda = exp(-2*rho*h) > 0, we can divide (1) by
            lambda, rename mu_i/lambda as nu_i and 1/lambda > 0 as alpha:

        P + (-1)^i nu_i Q + alpha A_i' Q A_i >= 0, i in {1,2}.   (2)

        This is linear on P, mu, and alpha.

        If P is given but rho is not, it minimizes rho to satisfy (1).
        If rho is given but P is not, it tries to find P that verifies (1).
        If both are given, it simply checks feasibility of (1), and, if it
            is not, the pair (None, None) is returned.
        If neither is given, it tries to find both rho and P simultaneously
            by solving (2).

        Parameters
        ----------
        P : np.array, optional
            The Lyapunov matrix for V(x) = x'Px. The default is None.
        rho : float, optional
            The desired GES decay rate. The default is None.

        Returns
        -------
        float
            The GES decay rate. None if none is found.
        np.array
            The Lyapunov matrix. None if none is found.

        [1] Heemels, W.P.M.H., Donkers, M.C.F., and Teel, A.R.
            (2013). Periodic event-triggered control for linear systems.
            IEEE Transactions on Automatic Control, 58(4),
            847--861.
        """

        if not self.is_state_feedback:
            raise ETCError('Output feedback not yet implemented.')

        raise ETCError('Future implementation.')

        n = self.plant.nx
        A1 = np.block([
            [self.Ad + self.Bd @ self.controller.K, self.zeros((n,n))],
            [np.eye(n),                             np.zeros((n,n))]
            ])
        A2 = np.block([
            [self.Ad,         self.Bd @ self.controller.K],
            [np.zeros((n,n)), np.eye(n)]
            ])
        Q = self.Qbar

        if P is None and rho is None:
            P = cvx.Variable((n, n), PSD=True)
            nu1 = cvx.Variable(pos=True)
            nu2 = cvx.Variable(pos=True)
            alpha = cvx.Variable(pos=True)
            constraints = [P - nu1*Q >> 0]
            prob = cvx.Problem(cvx.Minimize(0), constraints)
            prob.solve(eps=1e-6, max_iters=10000)
        elif P is None:
            return 0.1
        elif rho is None:
            return np.eye(self.nx)
        else:
            return

    def check_stability_pwa(self, eps=1e-3):
        """Check GES using the piecewise affine method of [1]

        Check global exponential stability of the closed-loop PETC
        by using Theorem III.4 in [1], which is based on a piecewise-
        affine model. It is the group of LMIs, for some P > 0,
        alpha_ij >= 0, beta_ij >= 0, kappa_i >= 0, i,j in {1,2},

        $$ e^{(-2*\rho*h)}P_i - A_i^\top P_jA_i + (-1)^i alpha_ij Q  $$
            + (-1)^j beta_ij A_i'QA_i >= 0,     i,j in {1,2}    $$     (1)
        P_i + (-1)^ikappa_iQ > 0,    i in {1,2}                      (2)

        where

        A_1 = [A + BK,  0; I,  0],
        A_2 = [A,  BK; 0,  I].

        This method performs a bisection algorithm on lambda
        := exp(-2*rho*h) to find the smallest value that satisfies
        (1) and (2).

        Parameters
        ----------
        eps : float, optional
            The precision for lambda in the bisection algorithm. The
            default is 1e-3.

        Returns
        -------
        float
            The GES decay rate, or 1 if it is not GES.
        (np.array, np.array)
            The Lyapunov matrices. None if system is not GES

        [1] Heemels, W.P.M.H., Donkers, M.C.F., and Teel, A.R.
            (2013). Periodic event-triggered control for linear systems.
            IEEE Transactions on Automatic Control, 58(4),
            847--861.
        """

        if not self.is_state_feedback:
            raise ETCError('Output feedback not yet implemented.')

        n = self.plant.nx
        K = self.controller.K
        A = {}
        A[1] = np.block([
            [self.Ad + self.Bd @ K, np.zeros((n,n))],
            [np.eye(n),         np.zeros((n,n))]
            ])
        A[2] = np.block([
            [self.Ad,         self.Bd @ K],
            [np.zeros((n,n)), np.eye(n)]
            ])
        Q = self.Qbar

        # CVX variables
        alpha = {(i,j): cvx.Variable(pos=True) for i in range(1,3)
                 for j in range(1,3)}
        beta = {(i,j): cvx.Variable(pos=True) for i in range(1,3)
                for j in range(1,3)}
        kappa = {i: cvx.Variable(pos=True) for i in range(1,3)}
        P = {i: cvx.Variable((2*n, 2*n), PSD=True) for i in range(1,3)}

        # CVX constraints : make a function of the externally defined lbd
        def make_constraints(lbd):
            con = []
            for i in range(1,3):
                for j in range(1,3):
                    con.append(lbd*P[i] - A[i].T @ P[j] @ A[i]
                               + ((-1)**i)*alpha[(i,j)]*Q
                               + ((-1)**j)*beta[(i,j)]*(A[i].T @ Q @ A[i])
                               >> 0)  # Eq. (1))
                con.append(P[i] + (-1)**i * kappa[i]* Q   # Eq. (2)
                           >> _LMIS_SMALL_IDENTITY_FACTOR*np.eye(2*n))
            return con

        # Start bisection algorithm: get extreme points
        a = 0
        b = 1

        # For b = 1, if GES then it must be feasible
        con = make_constraints(b)
        prob = cvx.Problem(cvx.Minimize(0), con)
        prob.solve()
        if 'infeasible' in prob.status:
            return 1, None
        Pout = (p.value for p in P)

        # For a = 0, if it is feasible then this is a deadbeat controller.
        # Can't be better then this
        con = make_constraints(a)
        prob = cvx.Problem(cvx.Minimize(0), con)
        prob.solve()
        if 'optimal' in prob.status:
            return 0, (p.value for p in P)

        # Now we should have b = 1 feasible and a = 0 infeasible. Start
        # bisection algorithm
        while b-a > eps:
            c = (a+b)/2
            con = make_constraints(c)
            prob = cvx.Problem(cvx.Minimize(0), con)
            prob.solve()
            if 'optimal' in prob.status:
                b = c
                Pout = (p.value for p in P)  # Store output P matrices
            elif 'infeasible' in prob.status:
                a = c
            else:
                warnings.warn(f'{prob.status}: TOL is {b-a}')
                break

        return -np.log(b)/2/self.h, Pout

    def check_stability_kmax(self, P):
        """Check stability based on simple S-procedure rule"""

        if not self.is_state_feedback:
            raise ETCError('Output feedback not yet implemented.')

        if self.kmax is None:
            raise ETCError('This method needs kmax')

        kmax = self.kmax
        Ad = self.Ad
        Bd = self.Bd
        K = self.controller.K
        n = self.plant.nx
        Q = self.Qbar

        mu = cvx.Variable((kmax,kmax),pos=True)  # S-procedure variable
        a = cvx.Variable(pos=True)  # Sample-wise contraction rate of V

        # Transition matrix at k=1
        M = Ad + Bd @ K

        Nlist = []  # Store N matrices from 1 to k
        con = []  # constraint list
        # with the case oh k=1

        for k in range(kmax):
            MI = np.block([[M],[np.eye(n)]])
            N = MI.T @ Q @ MI
            con.append(-mu[k,k] * N.copy()
                       + sum(mu[i,k] * n.copy() for i,n in enumerate(Nlist))
                       - M.T.copy() @ P @ M.copy()
                       + a * P >> 0)
            # Iterate for next M
            Nlist.append(N.copy())
            M = Ad @ M + Bd @ K

        prob = cvx.Problem(cvx.Minimize(a), con)
        prob.solve()
        return a.value


class TabuadaPETC(LinearQuadraticPETC):
    '''myETC = TabuadaPETC(plant, controller, P=None, Q=None, sigma=None,
    threshold=0)'''
    _sigma = None  # Should be an attribute
    _max_sigma = None
    kmax = None

    @property
    def max_sigma(self):
        return self._max_sigma

    def __init__(self, plant, controller, P=None, Q=None, sigma=None,
                 threshold=0, h=None,  kmax=None):
        super().__init__(plant, controller, h=h, P=P, kmax=kmax)

        assert not controller.is_dynamic and plant.states_as_output, \
            'Controller needs to be static state-feedback'
        self.threshold = threshold
        if h is None:
            h = controller.h
        self.h = h
        if kmax is not None:
            self.kmax = kmax

        # Compute maximum sigma based on Tabuada's rule
        B = plant.B
        K = controller.K
        self._max_sigma = np.real(np.sqrt(
                min(la.eig(self.Qlyap)[0])
                    / (la.norm(self.P @ B @ K + K.T @ B.T @ self.P, 2))))

        self.sigma = sigma

    def _check_sigma(self, sigma):
        if sigma is not None:
            assert 0. <= sigma <= 1., 'sigma must be between 0 and 1'
            if sigma > self._max_sigma:
                warnings.warn(
                    'Provided sigma is greater than Tabuada\'s maximum of %g'
                    % self._max_sigma,
                    ETCWarning)

    def _generate_Qbar(self):
        nx = self.plant.nx
        if self.sigma is not None:
            Qbar1 = (1-self.sigma**2)*np.eye(nx)
            Qbar2 = -np.eye(nx)
            Qbar3 = np.eye(nx)
            self.Qbar1 = Qbar1
            self.Qbar2 = Qbar2
            self.Qbar3 = Qbar3
            self.Qbar = np.block([[Qbar1, Qbar2], [Qbar2.T, Qbar3]])

    @property
    def sigma(self):
        return self._sigma

    @sigma.setter
    def sigma(self, sigma):
        self._sigma = sigma
        self._check_sigma(sigma)
        self._generate_Qbar()

    # def check_condition(self, x, xhat, y=None, yhat=None,
    # u=None, uhat=None, t=None, *args):
    def trigger(self, dt, x, xhat, y=None, yhat=None,
                u=None, uhat=None, t=None, *args):
        return la.norm(x-xhat) > self.sigma * la.norm(x) \
            or np.round(dt/self.h) >= self.kmax


class HeemelsCentralizedPETC(LinearQuadraticPETC):
    '''myETC = HeemelsCentralizedPETC(plant, controller, sigma=None,
    threshold=0)'''
    sigma = None

    def __init__(self, plant, controller, sigma):
        raise ETCError('Heemels PETC still not available')


class DirectLyapunovPETC(LinearQuadraticPETC):
    """ Check Lyapunov direcly.

    The triggering condition is based on the continuous-time Lyapunov
    condition
             d(x'Px)/dt <= -rho*(x'Qx),     (1)
    where P and Q are the Lyapunov
    matrices of the continuous-time closed-loop system and 0 <= rho < 1
    is the margin parameter. The triggering can be either predictive or
    not. When not, it simply implements the condition (1). When
    predictive, it computes x(k+1) = Ad @ x(k) + Bd @ K @ xhat(k) as the
    estimate of the next checking time state value. Them it uses x(k+1)
    in (1) to determine the triggering condition."""

    def __init__(self, plant, controller, P=None, Q=None, rho=0.5,
                 threshold=0, h=None, kmin=1, kmax=None,
                 predictive=False):
        super().__init__(plant, controller, P=P, kmin=kmin, kmax=kmax)
        self.A = plant.A
        self.B = plant.B
        self.K = controller.K
        self.P = P
        self.Q = Q
        self.rho = rho
        self.predictive = predictive
        if predictive:
            nx = plant.nx
            Abar = la.expm(np.block([[self.A, self.B @ self.K],
                              [np.zeros((nx, 2*nx))]]) * self.h)
            self.Abar = Abar
            self.Ad = Abar[:nx, :nx]
            s = la.expm(np.block([[self.A, self.B],
                        [np.zeros((plant.nu, nx + plant.nu))]]) * self.h)
            self.Bd = s[:nx, nx:]
        self._generate_Qbar()

    def _generate_Qbar(self):
        nx = self.plant.nx
        Qbar1 = self.A.T @ self.P + self.P @ self.A + self.rho * self.Q
        Qbar2 = self.P @ self.B @ self.K
        Qbar3 = np.zeros((nx, nx))
        self.Qbar1 = Qbar1
        self.Qbar2 = Qbar2
        self.Qbar3 = Qbar3
        self.Qbar = np.block([[Qbar1, Qbar2], [Qbar2.T, Qbar3]])
        if self.predictive:
            self.Qbar = self.Abar.T @ self.Qbar @ self.Abar
            self.Qbar1 = self.Qbar[:nx,:nx]
            self.Qbar2 = self.Qbar[:nx,nx:]
            self.Qbar3 = self.Qbar[nx:,nx:]

    def trigger(self, dt, x, xhat, y=None, yhat=None,
                u=None, uhat=None, t=None, *args):
        if self.predictive:
            x = self.Ad @ x + self.Bd @ self.K @ xhat
        z = self.A @ x + self.B @ self.K @ xhat
        return 2 * (z @ self.P @ x) + self.rho * (x @ self.Q @ x) > 0 \
            or dt >= (self.kmax - 1/2) * self.h


class RelaxedPETC(LinearQuadraticPETC):
    '''Relaxed PETC as in Szymanek et al (2019).'''
    must_trigger_at_first_crossing = False

    @property
    def max_sigma(self):
        return self._max_sigma

    def __init__(self, plant, controller, P=None, lbd=None,
                 threshold=0, h=None, kmax=None,
                 kmin=1):
        super().__init__(plant, controller, P=P, h=h, kmin=kmin, kmax=kmax)

        assert not controller.is_dynamic and plant.states_as_output, \
            'Controller needs to be static state-feedback'

        self.threshold = threshold
        h = self.h
        self.tmin = self.kmin*h

        nx = plant.nx
        nu = plant.nu

        if lbd is None:
            raise ETCError(
                    'A decay rate lbd must be provided for this condition')

        lbd0ct = min(np.real(la.eig(self.Qlyap, self.P)[0]))  # continuous-time
        beta2 = max(np.real(la.eig(self.Phi.T @ self.P @ self.Phi, self.P)[0]))
        lbd0dt = -np.log(beta2)/self.tmin

        if lbd0ct < 0 and lbd0dt < 0:
            raise ETCError('Closed-loop system does not seem to be stable')
            # Can we even reach this point after all the previous checks?

        self.lbd0 = max(lbd0ct, lbd0dt)
        self.lbd = lbd
        if lbd > self.lbd0:
            raise ETCError(f'lbd {lbd} must be smaller or equal to the'
                           ' Lyapunov function\'s decay rate, which is'
                           f' {lbd0ct} for continuous time and f{lbd0dt}'
                           ' for discrete time.')

        s = la.expm(np.block([[plant.A, plant.B],
                              [np.zeros((nu, nx + nu))]])*h)
        self.Ad = s[0:nx, 0:nx]
        self.Bd = s[0:nx, nx:nx+nu]
        # Phi = la.expm(Acl*h)
        # dQ = Phi.T @ P @ Phi - P
        self.triggering_is_time_varying = True

        # A'PA + P = Q
        # X'A'PAX + X'PX = X'QX
        # X'A'Xi'X'PXXiAX + X'PX = X'QX
        # P,Q -> X'PX,X'QX
        # Compute maximum sigma based on Tabuada's rule

    def _generate_Qbar(self):  # this one is not used
        pass

    def _Q_time_var(self, k, h):
        # Time-varying Q for Relaxed PETC, checks whether the
        # Lyapunov function exceeds the bound at the next time instant
        nx = self.plant.nx
        ny = self.plant.ny
        nu = self.plant.nu
        nz = ny + nu

        M = np.block([[self.Ad, self.Bd @ self.controller.K]])
        Z = np.zeros((nx, nx))
        Pe = self.P*np.exp(-self.lbd*k*h)
        Qbar = M.T @ self.P @ M - np.block([[Z, Z], [Z, Pe]])
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

    # def check_condition(self, dt, x, xhat, y=None, yhat=None,
    # u=None, uhat=None, t=None, *args):
    def trigger(self, dt, x, xhat, y=None, yhat=None,
                u=None, uhat=None, t=None, *args):
        next_sample = self.Ad@x + self.Bd @ self.controller.K@xhat
        dk = np.round(dt/self.h)
        return dk >= self.kmin \
            and (x.T@self.P@x > xhat.T@self.P@xhat*np.exp(-dt*self.lbd)
                 or (next_sample.T@self.P@next_sample
                     > xhat.T@self.P@xhat*np.exp(-(dt+self.h)*self.lbd))
                 or dk >= self.kmax)

    def max_decrease(self, k: int):
        """
        Worst-case Lyapunov function decrease when sampling at the k-th
        instant.

        Parameters
        ----------
        k : int
            Discrete-time sampling instant.

        Returns
        -------
        float
            min alpha : V(k) <= alpha*V(0)

        """
        return np.exp(-k * self.h * self.lbd)

# Utility
def best_relaxed_petc(plant: LinearPlant,
                      controller: LinearController,
                      **kwargs):
    """ Build 'the best' Relaxed PETC for a plant-controller pair.

    Parameters
    ----------
    plant : LinearPlant
        The plant
    controller : LinearController
        The controller (must be state-feedback)
    **kwargs : TYPE
        DESCRIPTION.

    Returns
    -------
    etc : RelaxedPETC

    """
    from .sampleandhold import most_economic_periodic, tight_lyapunov

    H_STEP = 0.01
    H_MAX = 100
    K_PERIODIC = 10
    KMAX = 30
    LBD_FUDGE = 0.999

    # Step 1. Get the best periodic controller
    h_best, decay = most_economic_periodic(plant, controller, H_STEP, H_MAX)

    # Step 2. Get a tight Lyapunov matrix for it
    pc = PeriodicLinearController(plant, controller, h_best)
    P, lbd, works_for_ct = tight_lyapunov(pc)

    # Step 3. Determine kmin and build the ETC
    h = h_best/K_PERIODIC
    controller.h = h
    lbd = lbd*LBD_FUDGE
    etc = RelaxedPETC(plant, controller, P, lbd, kmin=K_PERIODIC, kmax=KMAX)
    return etc


'''
    SELF-TRIGGERED CONTROL
'''


class STC(SampleAndHoldController):
    _sampling_type = 'STC'
    def sampling_time(self, xp, xc=None, **kwargs):
        """
        Placeholder for sampling_time computation of an STC.

        Parameters
        ----------
        xp : numpy.array
            Plant state
        xc : numpy.array, optional
            Controller state. The default is None.
        **kwargs :
            Any other parameter or data the particular STC strategy may need.

        Returns
        -------
        float > 0
            The sampling time.

        """
        return 0.5


'''
     TESTS
'''

if __name__ == '__main__':
    # with open('./tests/scenarios/simple.py') as f:
    #     code = compile(f.read(), "simple.py", 'exec')
    #     exec(code)  # , global_vars, local_vars)

    # h = 0.05
    # plant = LinearPlant(Ap, Bp, Cp, None, E)
    # controller = LinearController(K, h)
    # trig = TabuadaPETC(plant, controller, None, None, sigma)
    # trig = RelaxedPETC(plant, controller, Pl, lbd, kmin=8)
    # # trig.sigma = 0.2

    # x0 = np.array([0.7, -0.21])
    # x0 = np.array([1, 1])
    # x0 = np.array([1, -1])
    # trig.kmax = 20
    # t0, t1 = 0, 10
    # t = np.arange(t0, t1, 0.01)

    # with open('./tests/scenarios/batch_reactor.py') as f:
    #     code = compile(f.read(), "batch_reactor.py", 'exec')
    #     exec(code)  # , global_vars, local_vars)

    from tests.scenarios.simple import Ap, Bp, Cp, K, Pl, sigma, lbd

    h = 0.01
    plant = LinearPlant(Ap, Bp, Cp)
    controller = LinearController(K, h)
    trig = TabuadaPETC(plant, controller, None, None, sigma)
    trig.check_stability_pwa()

    # Testing base linear quadratic PETC
    qtrig = LinearQuadraticPETC(plant, controller, h=h, P=Pl, Qbar=trig.Qbar)


    trigr = RelaxedPETC(plant, controller, Pl, lbd, kmin=8)

