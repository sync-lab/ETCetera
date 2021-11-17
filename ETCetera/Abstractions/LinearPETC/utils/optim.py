#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 26 14:30:49 2019

@author: ggleizer
"""

import warnings
import logging
import cvxpy as cvx
import scipy.linalg as la
import numpy as np
import z3
import sympy

__TEST__ = False
_QCQP_TOLERANCE = 1e-4
_SSC_MAX_ITERS = 500000#30000  # Reaching maximum number of iterations is bad.
_SSC_MAX_ATTEMPTS = 20#10  # Number of times to try the SDP problem if inaccurate.


class ETCOptimError(Exception):
    pass

'''Z3 <--> sympy'''

# https://stackoverflow.com/questions/22488553/how-to-use-z3py-and-sympy-together
def sympy_to_z3(sympy_var_list, sympy_exp):
    '''convert a sympy expression to a z3 expression.
    This returns (z3_vars, z3_expression)'''

    z3_vars = []
    z3_var_map = {}

    for var in sympy_var_list:
        name = var.name
        z3_var = z3.Real(name)
        z3_var_map[name] = z3_var
        z3_vars.append(z3_var)

    result_exp = _sympy_to_z3_rec(z3_var_map, sympy_exp)

    return z3_vars, result_exp


def _sympy_to_z3_rec(var_map, e):
    'recursive call for sympy_to_z3()'

    rv = None

    if not isinstance(e, sympy.Expr):
        raise RuntimeError("Expected sympy Expr: " + repr(e))

    if isinstance(e, sympy.Symbol):
        rv = var_map.get(e.name)

        if rv == None:
            raise RuntimeError("No var was corresponds to symbol '"
                               + str(e) + "'")

    elif isinstance(e, sympy.Rational):
        rv = z3.Fraction(e.p, e.q)
    elif isinstance(e, sympy.Float):
        rv = float(e)
    elif isinstance(e, sympy.Mul):
        rv = _sympy_to_z3_rec(var_map, e.args[0])
        for child in e.args[1:]:
            rv *= _sympy_to_z3_rec(var_map, child)
    elif isinstance(e, sympy.Add):
        rv = _sympy_to_z3_rec(var_map, e.args[0])
        for child in e.args[1:]:
            rv += _sympy_to_z3_rec(var_map, child)
    # elif isinstance(e, sympy.Pow):
    #     term = _sympy_to_z3_rec(var_map, e.args[0])
    #     exponent = _sympy_to_z3_rec(var_map, e.args[1])

    #     if exponent == 0.5:
    #         # sqrt
    #         rv = Sqrt(term)
    #     else:
    #         rv = term**exponent

    if rv == None:
        raise RuntimeError("Type '" + str(type(e)) + "' is not yet implemented for convertion to a z3 expresion. " + \
                            "Subexpression was '" + str(e) + "'.")

    return rv

''' Set description functionality '''


class QuadraticForm:
    """ Represents a set of the type {x: x'Qx + 2b'x + c <= 0} """

    def __init__(self, Q, b=None, c=None, strict=False):
        n1, n2 = Q.shape
        assert n1 == n2, 'Q matrix must be square'

        self.n = n1
        self.Q = (Q + Q.T)/2
        self._is_symbolic = isinstance(self.Q, sympy.Basic)

        if b is not None:
            if c is None:
                raise ValueError('If b is provided, c must be provided')
            n = b.shape[0]
            assert n == self.n, \
                'b must be a vector of n elements if Q is n by n'
            if self._is_symbolic and not isinstance(b, sympy.Basic):
                b = sympy.ImmutableMatrix(b).T
        else:
            if c is not None:
                if self._is_symbolic:
                    b = sympy.zeros(1, self.n)
                else:
                    b = np.zeros(self.n, dtype=int)

        if self._is_symbolic and c is not None \
                and not isinstance(c, sympy.Basic):
            c = sympy.ImmutableMatrix([c])

        self.b = b
        self.c = c
        self.is_full = b is not None and c is not None
        self.strict = strict

    def __gt__(self, other):
        # returns true if self(x) <= 0 ==> other(x) <= 0
        assert type(other) is QuadraticForm, \
            'Quadratic forms can only be compared among themselves'
        assert self.n == other.n, \
            'Quadratic forms must have the same dimension'

        n = self.n
        Q1 = self.Q
        b1 = self.b
        c1 = self.c
        Q2 = other.Q
        b2 = other.b
        c2 = other.c
        if self.is_full:
            Q1 = np.block([[Q1, b1.reshape((n, 1))],
                           [b1.reshape((1, n)), c1]])
            if other.is_full:
                Q2 = np.block([[Q2, b2.reshape((n, 1))],
                               [b2.reshape((1, n)), c2]])
            else:
                Q2 = np.block([[Q2, np.zeros((n, 1))],
                               np.zeros((1, n+1))])
        else:
            if other.is_full:
                Q2 = np.block([[Q2, b2.reshape((n, 1))],
                               [b2.reshape((1, n)), c2]])
                Q1 = np.block([[Q1, np.zeros((n, 1))],
                               np.zeros((1, n+1))])

        # Q matrices ready... build SDP: exists mu>=0: mu*Q1 - Q2 >= 0
        mu = cvx.Variable()
        constraints = [mu >= 0,
                       mu*Q1 - Q2 >> 0]
        prob = cvx.Problem(cvx.Minimize(0), constraints)
        prob.solve(eps=1e-6, max_iters=10000)
        # prob.solve(eps=1e-8)
        # prob.solve()
        if 'inaccurate' in prob.status:
            logging.debug('LMI status is %s', prob.status)
        return prob.status == 'optimal'

    def __lt__(self, other):
        return other > self

    def complement(self):
        Q = -self.Q
        if self.b:
            b = -self.b
        else:
            b = None
        if self.c:
            c = -self.c
        else:
            c = None
        strict = not self.strict
        return QuadraticForm(Q, b, c, strict)

    def difference_magnitude(self, other):
        # TODO:
        assert type(other) is QuadraticForm, \
            'Quadratic forms can only be compared among themselves'
        assert self.n == other.n, \
            'Quadratic forms must have the same dimension'

        if (self.b is None
                and self.c is None
                and other.b is None
                and other.c is None):
            lbd = cvx.Variable()
            mid_matrix = self.Q - lbd*other.Q
            con = [lbd >= 0]
        else:
            mid_matrix = self.Q - other.Q
            con = []

        mu_pos = cvx.Variable()
        mu_neg = cvx.Variable()
        con.append(mid_matrix << mu_pos*np.eye(self.n))
        con.append(mid_matrix >> -mu_neg*np.eye(self.n))
        con += [mu_pos >= 0, mu_neg >= 0]

        prob = cvx.Problem(cvx.Minimize(cvx.maximum(mu_pos, mu_neg)), con)
        prob.solve()

        if prob.status != 'optimal':
            if 'innacurate' in prob.status:
                logging.warning('Problem status is %s', prob.status)
            else:
                raise ETCOptimError(
                    'This shouldn''t be happening. Problem status is %s',
                    prob.status)

        return prob.value

    def sdr_expression(self, X, x=None):
        """
        Generates the CVX semidefinite relaxation (SDR) expression associated
        with the set.

        For a quadratic expression x'Qx + 2b'x + c, the SDR relaxation is
        trace(QX) + 2b'x + c.

        The problem must always include [X  x] >= 0 as a constraint.
                                        [x' 1]

        Parameters
        ----------
        X : cvxpy.Variable
            CVX matrix variable satisfying X >= 0, with the same size as Q.
            Relaxes xx'
        x : cvxpy.Variable, optional
            CVX vector variable with the same length as any of the dimensions
            of Q. Must be given if either self.b or self.c is not None.

        Returns
        -------
        cvxpy.Expression
            The expression trace(QX) + 2b'x + c

        """

        # TODO: can we improve conditioning here?

        assert (X.shape == self.Q.shape
                and (x is None or x.shape[0] == self.Q.shape[0])), \
            'Incompatible dimensions'

        if self.b is None:
            return cvx.trace(self.Q @ X)
        else:
            return cvx.trace(self.Q @ X) + 2 * self.b.T @ x + self.c

    def _symbolic_rhs(self, x):
        if self.is_full:
            expr = x.T @ self.Q @ x + 2 * self.b @ x + self.c
        else:
            expr = x.T @ self.Q @ x
        return expr

    def z3_expression(self, x):
        expr = self._symbolic_rhs(x)
        try:
            expr = expr[0,0]
            converted_rhs = sympy_to_z3([v for v in x], expr)[1]
        except TypeError:  # TypeError means it's not sympy, thus it is z3.
            converted_rhs = expr

        if self.strict:
            return converted_rhs < 0
        else:
            return converted_rhs <= 0

    def sympy_expression(self, x):
        expr = self._symbolic_rhs(x)
        if self.strict:
            return expr < 0
        else:
            return expr <= 0

    def __contains__(self, x):
        """ Determines if x belongs to this set

        Parameters
        ----------
        x: numpy.array

        Returns
        -------
        bool
            True if x in set

        """

        if self.b is None:
            expr = x.T @ self.Q @ x
        else:
            expr = x.T @ self.Q @ x + self.b.T @ x + self.c

        try:
            return expr <= 0
        except TypeError:  # Using sympy matrices instead of numpy arrays
            if self.strict:
                return expr[0,0] < 0
            else:
                return expr[0,0] <= 0


''' Constraint satisfaction / optimization problems. '''


def sdr_problem(objective, constraints, minimize=True, unit_ball=False):
    """Creates a semidefinite relaxation (SDR) problem for the
    quadratically constrained quadratic program (QCQP) given in the
    objective and constraints.

    Parameters
    ----------
    objective : QuadraticForm
        A quadratic expression for the objective function.
    constraints: set or list of QuadraticForm
        The constraints of the type x'Qx + 2x'b + c <= 0.
    minimize: bool, optional
        Whether the problem is of minimization (true, default) or of
        maximization.
    unit_ball : bool, optional
        Add the SDR of the constraint x'T @ x == 1, restraining the
        problem to the unit ball. The default is false.

    Returns
    -------
    cvxpy.Problem
        The SDR problem

    See Also
    --------
    QuadraticForm.sdr_expression : SDR relaxation of a quadratic expression.
    """

    if objective:
        n = objective.n
    else:
        for con in constraints:
            break
        n = con.n
        objective = QuadraticForm(np.zeros((n, n)))

    # Create variables and build SDR problem
    X = cvx.Variable((n, n), PSD=True)
    if objective.b is None and all(Q.b is None for Q in constraints):
        obj = objective.sdr_expression(X)
        con = [Q.sdr_expression(X) <= 0 for Q in constraints]
    else:
        x = cvx.Variable((n, 1))
        obj = objective.sdr_expression(X, x)
        con = [Q.sdr_expression(X, x) <= 0 for Q in constraints]
        # Below: [X x; x' 1] >= 0
        con.append(cvx.bmat([[X, x], [x.T, np.array([[1]])]]) >> 0)

    if unit_ball:
        con.append(cvx.trace(X) == 1)

    if minimize:
        return cvx.Problem(cvx.Minimize(obj), con)
    else:
        return cvx.Problem(cvx.Maximize(obj), con)


def z3_problem(constraints, solver: z3.Solver):
    for con in constraints:
        break
    n = con.n
    if con._is_symbolic:
        x = sympy.Matrix([z3.Real('x_%s' % (i+1)) for i in range(n)])
    else:
        x = np.array([z3.Real('x_%s' % (i+1)) for i in range(n)])

    for con in constraints:
        solver.add(con.z3_expression(x))

    return x, solver


class QuadraticProblem():
    def __init__(self, constraints, objective=None, solver='sdr',
                 minimize=True, unit_ball=False):
        self.constraints = constraints
        self.objective = objective
        self.solver = solver
        self.minimize = minimize
        self.unit_ball = unit_ball

        if solver=='sdr':
            self.problem = sdr_problem(objective, constraints, minimize,
                                       unit_ball)
        elif solver=='z3':
            self.problem = z3.Solver()
            self.x, self.problem = z3_problem(constraints, self.problem)

    def add_constraints(self, constraints):
        self.constraints = self.constraints.union(constraints)
        if self.solver=='sdr':
            self.problem = sdr_problem(self.objective, self.constraints,
                                       self.minimize, self.unit_ball)
        elif self.solver=='z3':
            for con in constraints:
                self.problem.add(con.z3_expression(self.x))

    def solve(self):
        if self.solver=='sdr':
            warnings.filterwarnings('ignore', 'Solution may be inaccurate. Try another solver,*')

            n_tries = 0
            while n_tries < _SSC_MAX_ATTEMPTS:
                self.problem.solve(eps=_QCQP_TOLERANCE, max_iters=_SSC_MAX_ITERS,
                              verbose=__TEST__)
                if 'inaccurate' not in self.problem.status:
                    break
                n_tries += 1

            # print(prob.status)
            if 'optimal' in self.problem.status:
                if 'inaccurate' in self.problem.status:
                    logging.info(f'SDR is {self.problem.status}')
            elif 'unbounded' in self.problem.status:
                warnings.warn(f'SDR gave {self.problem.status}!')
            elif 'infeasible' not in self.problem.status:
                raise ETCOptimError(
                    'Some unknown exception happened! The semi-definite'
                    ' relaxation problem should be either feasible or'
                    ' infeasible.  If you are here that means it was neither.'
                    '  Possibily, the problem is numerically ill conditioned'
                    ' and failed.')

            self.problem.solve()
            self.feasible = 'optimal' in self.problem.status \
                or ('inaccurate' in self.problem.status and 'unbounded'
                    not in self.problem.status)
            self.value = self.problem.value
        elif self.solver=='z3':
            result_value = 0
            counter = 0
            while result_value == 0 and counter <= 10:
                #self.problem.set('timeout', 60000*(counter + 1))
                result = self.problem.check()
                # print(result)
                result_value = result.r
                counter += 1

            if result_value == 0:
                error = ETCOptimError('Z3 failed to provide an answer!')
                error.problem = self.problem
                raise error

            self.feasible = result_value == 1
            if self.feasible:
                self.value = self.problem.model()

        return self.feasible

class TwoCone:
    Q = None
    rays = ()

    def __init__(self, Q):
        assert Q.shape == (2, 2), 'Matrix Q must be 2x2'
        self.Q = Q
        self._make_cone_rays()

    def _make_cone_rays(self):
        """
        Returns v1 and v2 as the cone rays defined by x'Qx == 0.
        They are selected so that any convex combination of v1 and v2
        satisfies x'Qx > 0.

        Adds to Class
        -------------
        rays
            A pair of vectors (np.array) supporting the rays

        """

        # Generalized eigenvalue, so that we get which vectors are rotated 90
        # degrees
        _, V = la.eig(self.Q, np.array([[0, -1], [1, 0]]))

        v1 = V[:, 0]
        v2 = V[:, 1]
        if (v1+v2).T @ self.Q @ (v1+v2) < 0:
            v2 = -v2
        self.rays = (v1, v2)

    def get_clipped_cone(self):
        """Generate a polygon representing the cone in a [-1,1] box.


        Returns
        -------
        x : np.array
            x coordinate of the polygon
        y : np.array
            y coordinate of the polygon

        """
        corners = np.array([[1, -1, -1, 1], [1, 1, -1, -1]])
        v1, v2 = self.rays

        v1 = v1/max(abs(v1))
        v2 = v2/max(abs(v2))

        V = []
        VV =np.concatenate(([v1],[v2])).T
        for c in range(4):
            v = corners[:,c]
            if v.T @ self.Q @ v >= 0:
                l = la.solve(VV, v)
                if all(l>0):
                    V.append(v)

        if len(V) > 1:
            # Check order
            if la.norm(V[1]-v1) < la.norm(V[0]-v1):
                V = [x for x in reversed(V)]

        x = [0, v1[0]]
        y = [0, v1[1]]
        for v in V:
            x.append(v[0])
            y.append(v[1])
        x.append(v2[0])
        y.append(v2[1])

        return np.array(x), np.array(y)

class Ellipse:
    def __init__(self, Q):
        if all(la.eigvalsh(Q) >= 0) and Q.shape == (2,2) and (Q == Q.T).all():
            self.Q = Q
            U = la.cholesky(Q)
            self.A = la.inv(U)
        else:
            raise Exception('Q must be a positive definite 2x2 matrix')

    def generate_plot_data(self, N=100):
        th = np.linspace(0, 2*np.pi, N)
        x = np.array([np.cos(th), np.sin(th)])
        print(x)
        return self.A @ x






# Worst-case number of sequences when a bound V(tk+1) <= a^dtV(tk)
class NumSequences:
    def __init__(self, n):
        self.n = n
        self.c = {}

    def __call__(self, N):
        if N in self.c:
            return self.c[N]
        elif N <= 0:
            return 1
        else:
            self.c[N] = sum(self(N-i-1) for i in range(self.n))
            return self.c[N]


# Bisimulation utility
def get_clusters(regions: set):
    regions = regions.copy()
    out = []
    while len(regions) > 0:
        for r in regions:
            break
        cluster = set((r,))
        regions.remove(r)
        rn = set(x for x in regions if r[1:] == x[:-1])
        while len(rn) > 0:
            cluster.update(rn)
            regions = regions - cluster
            rn = set(x for x in regions for y in cluster if y[1:] == x[:-1])
        # Now look at pre(r)
        rp = set(x for x in regions if x[1:] == r[:-1])
        while len(rp) > 0:
            cluster.update(rp)
            regions = regions - cluster
            rp = set(x for x in regions for y in cluster if x[1:] == y[:-1])
        out.append(cluster)
    return out

# Test
if __name__ == '__main__':
    Q1 = QuadraticForm(np.array([[1, 0], [0, 1]]))
    Q2 = QuadraticForm(np.array([[1, 0], [0, 0]]), np.zeros((2,)), -1)
    Q3 = QuadraticForm(np.array([[0, 0], [0, 1]]), np.zeros((2,)), -1)
    prob = QuadraticProblem({Q2, Q3}, objective=Q1)
    prob.solve()
    probz3 = QuadraticProblem({Q2, Q3}, solver='z3', unit_ball=True)
    probz3.solve()
