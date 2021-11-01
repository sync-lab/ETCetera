#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 19 15:02:31 2018

@author: ggleizer
"""


import numpy as np

''' Settings for solvers '''
_MAX_TRIES_TIGHT_LYAPUNOV = 10
_ASSERT_SMALL_NUMBER = 1e-6
_LMIS_SMALL_IDENTITY_FACTOR = 1e-6

''' linearsys.py '''
''' Main linear system definitions'''


'''
    SYSTEM DEFINITION
'''

class Plant:
    """ Base plant class, only has dimensions. """

    nx = 0
    ny = 0
    nu = 0
    nw = 0


class LinearPlant(Plant):
    """
    A linear system of the form

    .. math:: \dot{x} = Ax + Bu + Ew
    .. math:: y = Cx + Du + v,

    where x(t) is the state vector, u(t) is the input vector, y(t) is the
    output vector, w(t) is the vector of external disturbances, and
    v(t) is the vector of measurement noise.
    ...

    Parameters
    ----------
    A : array_like
        State-to-state matrix
    B : array_like
        Input-to-state matrix
    C : array_like, optional
        State-to-output matrix. Default is the identity matrix (states as
        outputs)
    D : array_like, optional
        Input-to-output matrix. Default is the zero matrix.
    E : array_like, optional
        Disturbance-to-state matrix. Default is an empty matrix, i.e.,
        it is considered that no disturbances act in the system.


    Attributes
    ----------
    A : numpy.array
        State-to-state matrix
    B : numpy.array
        Input-to-state matrix
    C : numpy.array
        State-to-output matrix
    D : numpy.array
        Input-to-output matrix
    E : numpy.array
        Disturbance-to-state matrix
    nx : int
        The state-space dimension
    nu : int
        The input-space dimension
    ny : int
        The output-space dimension
    nw : int
        The disturbance-space dimension

    Methods
    -------
    measurement(x, noise=0)
        Returns output y for a given state x and noise (v).

    """

    A = np.array([])
    B = np.array([])
    C = np.array([])
    D = np.array([])
    E = np.array([])

    def __init__(self, A, B, C=None, D=None, E=None):
        # Retrieve A
        A = np.array(A)
        if len(A.shape) == 0:
            A = np.array([[A]])
        nx1, nx2 = A.shape
        assert nx1 == nx2, 'Matrix A needs to be square'
        self.nx = nx1
        self.A = A

        # Retrieve B
        try:
            nx, nu = B.shape
            self.B = B
        except ValueError:  # B is a vector
            nx = B.shape[0]
            nu = 1
            self.B = B.reshape(nx, nu)
        assert self.nx == nx, 'B must have the same number of rows as A'
        self.nu = nu

        # Retrieve C
        if C is None:
            self.C = np.eye(self.nx)
            self.ny = self.nx
        else:
            try:
                ny, nx = C.shape
                self.C = C
            except ValueError:
                nx = C.shape[0]
                ny = 1
                self.C = C.reshape(ny, nx)
            assert self.nx == nx, 'C must have the same number of columns as A'
            self.ny = ny

        # Retreive D
        if D is None:
            self.D = np.zeros((self.ny, self.nu))
        else:
            D = np.array(D)
            assert len(D.shape) in [0, 2], \
                'D cannot be a one-dimensional array'
            try:
                ny, nu = D.shape
                self.D = D
            except ValueError:
                ny = 1
                nu = 1
                self.D = np.array([[D]])
            assert self.ny == ny, 'D must have the same number of rows as C'
            assert self.nu == nu, 'D must have the same number of columns as B'

        # Retreive E
        if E is None:
            self.E = np.zeros((nx, 0))
            self.nw = 0
        else:
            try:
                nx, nw = E.shape
                self.E = E
            except ValueError:  # B is a vector
                nx = E.shape[0]
                nw = 1
                self.E = E.reshape(nx, nw)
            assert self.nx == nx, 'E should have the same number of rows as A'
            self.nw = nw

    @property
    def states_as_output(self):
        return (self.ny == self.nx) and (self.C == np.eye(self.nx)).all()

    def measurement(self, x, noise=0):
        return self.C @ x + noise


class Controller:
    """ Base controller class, has dimensions and basic properties. """

    nx = 0
    ny = 0
    nu = 0
    is_dynamic = False
    h = None

    @property
    def is_dynamic(self):
        return self.nx > 0
    @property
    def is_discrete_time(self):
        return not(self.h is None)

    def output(self, y, xc=None):
        return np.zeros(self.nu)


class LinearController(Controller):
    """A linear (output-feedback) controller of the form

    .. math:: x^+ = Ax + By
    .. math:: u = Cx + Dy,

    where x(t) is the controller state vector, u(t) is the control input
    vector, and y(t) is the measured plant output vector. The symbol +
    represents either the continuous- or discrete-time difference operator.

    Possible constructor calls:
        LinearController(K): static continuous-time controller.

        LinearController(K, h): static discrete-time controller, with
        sampling time h.

        LinearController(A,B,C,D): dynamic continuous-time controller

        LinearController(A,B,C,D,h): dynamic cdiscrete-time controller,
        with sampling time h.


    Parameters
    ----------
    *args : tuple
        Variable-size array, length must be either 1, 2, 4, or 5.
    K : array_like
        If len(args) <= 2, the controller is assumed to be static, i.e.,
        u = Ky.
    A : array_like
        If len(args) >= 4, state-to-state matrix
    B : array_like
        If len(args) >= 4, output-to-state matrix
    C : array_like
        If len(args) >= 4, state-to-input matrix.
    D : array_like
        If len(args) >= 4, output-to-input matrix.
    h : float
        Sampling time (or controller update time)


    Attributes
    ----------
    A : numpy.array
        State-to-state matrix
    B : numpy.array
        Output-to-state matrix
    C : numpy.array
        State-to-input matrix
    D : numpy.array
        Output-to-input matrix
    nx : int
        The state-space dimension
    nu : int
        The input-space dimension
    ny : int
        The output-space dimension
    h : float
        Sampling time (or controller update time)

    Methods
    -------
    output(y, x=None)
        Returns controller's output from plant output y and (optionally)
        controller state x

    """
    A = np.array([])
    B = np.array([])
    C = np.array([])
    D = np.array([])

    def __init__(self, *args):
        if len(args) <= 2:  # Static controller
            self.nx = 0
            K = args[0]
            try:
                h = args[1]
            except IndexError:
                h = None

            K = np.array(K)
            if len(K.shape) == 0:
                K = np.array([[K]])
            assert len(K.shape) == 2, \
                'K needs to be a scalar or a 2-dimensional array'
            nu, ny = K.shape


            self.nu = nu
            self.ny = ny
            self.A = np.zeros((0, 0))
            self.B = np.zeros((0, ny))
            self.C = np.zeros((nu, 0))
            self.D = K
            self.h = h

        elif len(args) in (4, 5):  # Dynamic controller
            try:
                A, B, C, D, h = args
            except ValueError:
                A, B, C, D = args
                h = None

            dummy_plant = LinearPlant(A, B, C, D)
            self.A = dummy_plant.A
            self.B = dummy_plant.B
            self.C = dummy_plant.C
            self.D = dummy_plant.D
            self.nx = dummy_plant.nx
            self.nu = dummy_plant.nu
            self.ny = dummy_plant.ny
            self.h = h
        else:
            raise Exception('Wrong number of inputs')

    @property
    def K(self):
        if not self.is_dynamic:
            return self.D
        else:
            raise AttributeError('Controller is dynamic, no gain available')

    def output(self, y, x=None):
        if self.is_dynamic:
            return self.C @ x + self.D @ y
        else:
            return self.D @ y


'''
     TESTS
'''

if __name__ == '__main__':
    from tests.scenarios.batch_reactor import Ap, Bp, Cp, K

    h = 0.01
    plant = LinearPlant(Ap, Bp, Cp)
    controller = LinearController(K, h)


