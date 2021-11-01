import sympy
from typing import List, Union
import logging

def test_homogeneity(exprs: Union[List[sympy.And], sympy.Matrix], vars: List[sympy.Symbol], trigger=None):
    """
    Tests whether dynamics are homogeneous. If so, return also the homogeneity degree.
    @param exprs: List of sympy expressions representing the ETC dynamics. Same length as vars.
    @param vars: List of the used sympy symbols. Same length as exprs.
    @return: Degree of Homogeneity if dynamics are homogeneous. None otherwise.
    """
    # Test whether a given expression is homogeneous. If this is the case return the degree, otherwise -1
    alpha = sympy.symbols('a', real=True, positive=True)
    l = sympy.symbols('l', real=True, positive=True)

    dic = {}
    for v in vars:
        # print(v)
        dic[v] = l * v

    if type(exprs) == list:
        exprs = sympy.Matrix(exprs)

    if type(exprs) == sympy.Add:
        a2 = l ** (alpha + 1) * exprs
    else:
        a2 = sympy.Matrix([l ** (alpha + 1) * x for x in exprs])

    a1 = exprs.subs(dic)
    # a2 = sympy.Matrix([l**(alpha+1)*x for x in exprs])

    res = sympy.solve(a1 - a2, alpha)

    if res != []:
        res = sympy.simplify(res[0][0])
        logging.info(f'Alpha solution: {res}')
        logging.info(f'Meaning the system is homogeneous with degree {res}')
        return res
    else:
        logging.info('System is not Homogeneous')
        return None

def make_homogeneous(exprs: Union[List[sympy.And], sympy.Matrix], vars: List[sympy.Symbol],
                     des_hom_degree: int):
    """
    Make an general system homogeneous with desired degree
    @param exprs: List of sympy expressions representing the dynamics. Same length as vars.
    @param vars: List of the used sympy symbols. Same length as exprs.
    @param des_hom_degree: The desired homogeneity degree
    @return: New list/matrix of expressions and variables

    """
    w1 = sympy.symbols('w1')
    w1dot = 0
    dic = {}
    for v in vars:
        dic[v] = v * w1 ** -1

    n = int(len(vars) / 2)

    res = []
    for i in range(0, int(len(exprs) / 2)):
        res.append(sympy.simplify(w1 ** (1 + des_hom_degree) * exprs[i].subs(dic)))
    res.append(w1dot)
    newvars = tuple((*vars[0:n], w1))

    if type(exprs) == sympy.Matrix:
        res = sympy.Matrix(res)

    return res, newvars


def make_homogeneous_etc(exprs: Union[List[sympy.And], sympy.Matrix], vars: List[sympy.Symbol],
                     des_hom_degree: int, trigger: sympy.And = None):
    """
    Make an ETC system homogeneous with desired degree
    @param exprs: List of sympy expressions representing the ETC dynamics. Same length as vars.
    @param vars: List of the used sympy symbols. Same length as exprs.
    @param des_hom_degree: The desired homogeneity degree
    @return: New list/matrix of expressions and variables
    """

    w1, ew = sympy.symbols('w1 ew')
    w1dot = 0
    ew_dot = -w1dot
    dic = {}
    for v in vars:
        dic[v] = v * w1**-1

    n = int(len(vars) / 2)

    res = []
    for i in range(0, int(len(exprs) / 2)):
        res.append(sympy.simplify(w1 ** (1 + des_hom_degree) * exprs[i].subs(dic)))
    res.append(w1dot)
    for i in range(int(len(exprs) / 2), len(exprs)):
        res.append(sympy.simplify(w1 ** (1 + des_hom_degree) * exprs[i].subs(dic)))
    res.append(ew_dot)
    newvars = tuple((*vars[0:n], w1, *vars[n:], ew))

    if type(exprs) == sympy.Matrix:
        res = sympy.Matrix(res)

    if trigger is not None:
        trigger = sympy.simplify(w1 ** (2) * trigger.subs(dic))
        return res, newvars, trigger

    return res, newvars

if __name__ == '__main__':
    # Variable Declaration
    state_vector = x1, y1, ex, ey = sympy.symbols('x1 y1 ex ey')  # state variables + errors
    init_cond_vector = x0, y0 = sympy.symbols('x0 y0')  # symbols for initial conditions of state variables
    parameters = ()  # disturbances
    parameters_domain = []  # disturbances' domain

    # Declare symbolic dynamics
    x1dot = (-x1 ** 3 + x1 * y1 ** 2)
    u1 = -(ey + y1) ** 3 - (ex + x1) * (ey + y1) ** 2
    y1dot = (x1 * y1 ** 2 - y1 * x1 ** 2 + u1)
    ex_dot = -x1dot
    ey_dot = -y1dot

    # x1dot = -x1
    # u1 = -(y1 + ey) - (x1 + ex) ** 2 * (y1 + ey) - (y1 + ey) ** 3
    # y1dot = x1 ** 2 * y1 + y1 ** 3 + u1
    # print(y1dot)
    # w1dot = 0
    # ex_dot = -x1dot
    # ey_dot = -y1dot

    print("Test Homogeneity:")
    a = test_homogeneity([x1dot, y1dot, ex_dot, ey_dot], [x1, y1, ex, ey])
    if a is None:
        print('')
        print("Make Homogeneous")
        a, b = make_homogeneous([x1dot, y1dot, ex_dot, ey_dot], [x1, y1, ex, ey], 2)
        print(a)
        print(b)

    # w1, ew = sympy.symbols('w1 ew')
    # # Declaring symbolic dynamics
    # x1dot = -x1 * w1 ** 2
    # u1 = -(y1 + ey) * w1 ** 2 - (x1 + ex) ** 2 * (y1 + ey) - (y1 + ey) ** 3
    # y1dot = x1 ** 2 * y1 + y1 ** 3 + u1
    # print(y1dot)