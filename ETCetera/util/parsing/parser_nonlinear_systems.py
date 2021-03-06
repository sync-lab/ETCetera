"""
Created on Sat May 16 14:53:58 2020

@author: gmaddodi
"""

import re
import sys

import ETCetera.util.parsing.syntax_checker as sc
import sympy as sp

from ETCetera.exceptions.parser_exceptions.general_parser_exception import EmptyValueException, \
    MultipleValuesFoundException, NotPositiveRealNumberException, IncorrectSyntaxException, GenericParsingException
from ETCetera.exceptions.parser_exceptions.symbolic_expression_exceptions import \
    IncorrectNumOfSymbolicExpressionException


def parse_nonlinear(line):
    """
    The function takes a string representing the key-value pair of non-linear control system input, checks which input
    datastructure it is and extracts the value. Returns the value in appropriate form if checks are passed, else raises
    an exception.

    Parameters:
    ----------
        line : string
            A string of key and value corresponding to an attribute of InputDataStructureLinear, e.g. Dynamics = [1 2 3 4].

    Returns:
    -------
        An appropriate data structure (list, string, dictionary).

    Exceptions:
    ----------
        IncorrectSyntaxException, EmptyValueException, IncorrectNumOfSymbolicExpressionException,
        NotPositiveRealNumberException.
    """

    # If the key is 'dynamics'
    if line.split(':')[0].strip() == 'Dynamics':
        dynamics = []
        allowed_chars = list(['x', 'u', 'e', 'd', 'w'])  # Defined allowed characters in symbolic expression
        try:
            value = line.split(':')[1].strip()  # Check value exists
        except IndexError:
            raise EmptyValueException
        sc.check_symbols_in_exprs(allowed_chars, value)  # Get the symbols from expression
        for expr in line.split(':')[1].strip().split(', '):  # Split the symbolic expressions delimited by ','
            dynamics.append(sc.check_symbolic_expr(expr))  # Verify the symbolic expression and append
        return dynamics

    # If the line is 'controller'
    elif line.split(':')[0].strip() == 'Controller':
        controller = []
        allowed_chars = list(['x', 'w', 'e'])  # Defined allowed characters in symbolic expression
        try:
            value = line.split(':')[1].strip()
        except IndexError:
            raise EmptyValueException
        sc.check_symbols_in_exprs(allowed_chars, value)  # Get the symbols from expression
        for expr in line.split(':')[1].strip().split(', '):  # Split the symbolic expressions delimited by ','
            controller.append(sc.check_symbolic_expr(expr))   # Verify the symbolic expression and append

        return controller

    # If the line is 'triggering_condition'
    elif line.split(':')[0].strip() == 'Triggering Condition':
        triggering_condition = 0
        allowed_chars = list(['x', 'e', 'w'])  # Defined allowed characters in symbolic expression
        try:
            num_exprs = len(line.split(':')[1].strip().split(', '))
        except IndexError:
            raise EmptyValueException
        if num_exprs == 1:     # There should be only one expression
            sc.check_symbols_in_exprs(allowed_chars, line.split(':')[1].strip())  # Get the symbols from expression
            triggering_condition = sc.check_symbolic_expr(line.split(':')[1].strip())    # Verify the symbolic expression and append
            return triggering_condition
        else:
            raise IncorrectNumOfSymbolicExpressionException(num_exprs, 1)

    # If the key is 'hyperbox_states'
    elif line.split(':')[0].strip() == 'Hyperbox States':
        hyperbox_states = []
        try:
            hyperbox_states_vectors = line.split(':')[1].strip().split(', ')
        except IndexError:
            raise IncorrectSyntaxException
        for item in hyperbox_states_vectors:  # Split the vectors delimited by ','
            list_of_values = sc.check_keyvalue_syntax(' ', '\[(.*)\]', item)  # Check the vector syntax
            sc.check_if_numerical_values(list_of_values)  # Check that values are all real numbers
            hyperbox_states.append([float(i) for i in list_of_values])  # Convert list into vector and append
        return hyperbox_states

    elif line.split(':')[0].strip() == 'Grid Points Per Dimension':
        item = line.split(':')[1].strip()
        list_of_values = sc.check_keyvalue_syntax(' ', '\[(.*)\]', item)  # Check the vector syntax
        sc.check_if_numerical_values(list_of_values)  # Check that values are all real numbers
        grid_points = [int(i) for i in list_of_values]  # Convert list into vector and append
        return grid_points

    # If the line is 'hyperbox_disturbances'
    elif line.split(':')[0].strip() == 'Hyperbox Disturbances':
        hyperbox_disturbances = []
        try:
            hyperbox_disturbances_vectors = line.split(':')[1].strip().split(', ')
        except IndexError:
            raise IncorrectSyntaxException

        len_vectors = len(hyperbox_disturbances_vectors)

        for item in hyperbox_disturbances_vectors:   # Split the vectors delimited by ','
            list_of_values = sc.check_keyvalue_syntax(' ', '\[(.*)\]', item)    # Check the vector syntax
            if len(list_of_values) == 0 and len_vectors > 1:
                raise GenericParsingException('No other vectors can be specified when an empty vector defined. Syntax '
                                              'Error on line ')
            elif len(list_of_values) == 0 and len_vectors == 1:
                pass
            else:
                sc.check_if_numerical_values(list_of_values)    # Check the values are real numbers
                hyperbox_disturbances.append([float(i) for i in list_of_values])  # Convert list into vector and append
        return hyperbox_disturbances

    # If the line is 'deg_of_homogeneity'
    elif line.split(':')[0].strip() == 'Deg. of Homogeneity':
        try:
            value = line.split(':')[1]
        except IndexError:
            raise EmptyValueException
        if len(value.strip().split(',')) != 1:  # There should be only one value
            raise MultipleValuesFoundException
        sc.check_if_numerical_values(value.strip().split(' '))     # Check if float value
        if float(value.strip()) < 0:    # value shpuld be positive integer
            raise NotPositiveRealNumberException
        return float(value.strip())


    # If the line is 'solver_options'
    elif line.split(':')[0].strip() == 'Solver Options':
        solver_options = dict()
        try:
            # Check if no values specified as this data structure can be empty,
            if len(list(filter(None, line.split(':')[1].strip().split(', ')))) == 0:
                return solver_options

            for item in line.split(':')[1].strip().split(', '):
                # re.search('[a-z_]+=[a-z0-9.]+', item).group(0)
                re.search('[a-zA-Z_]+=(\[([a-zA-Z0-9. ])*]|[a-zA-Z0-9.]+)', item).group(0)

                key = item.split('=')[0]
                value = item.split('=')[1]


                if key == 'precision_deltas':
                    solver_options.update({key: float(value)})
                elif key == 'timeout_deltas':
                    solver_options.update({key: int(value)})
                elif key == 'partition_method':
                    solver_options.update({key: str(value)})
                elif key == 'manifolds_times':
                    values = re.search('\[([0-9. ]*)]', value).group(1)
                    values = values.strip().split(' ')
                    values = list(filter(None, values))
                    if len(values) > 0:
                        solver_options.update({key: [float(v) for v in values]})
                elif key == 'angles_discretization':
                    values = re.search('\[([0-9 ]*)]', value).group(1)
                    values = values.strip().split(' ')
                    values = list(filter(None, values))
                    if len(values) > 0:
                        solver_options.update({key: [int(v) for v in values]})
                # elif key == 'nr_cones_small_angles':
                #     values = re.search('\[(.*)]', value).group(1)
                #     values = values.strip().split(' ')
                #     values = list(filter(None, values))
                #     if len(values) > 0:
                #         sc.check_if_numerical_values(values)
                #         solver_options.update({key: [int(i) for i in values]})
                # elif key == 'nr_cones_big_angle':
                #     solver_options.update({key: int(value)})
                # elif key == 'state_space_limits':
                #     hyperbox_states = []
                #     try:
                #         hyperbox_states_vectors = line.split(':')[1].strip().split(', ')
                #     except IndexError:
                #         raise IncorrectSyntaxException
                #     for item in hyperbox_states_vectors:  # Split the vectors delimited by ','
                #         list_of_values = sc.check_keyvalue_syntax(' ', '\[(.*)\]', item)  # Check the vector syntax
                #         sc.check_if_numerical_values(list_of_values)  # Check that values are all real numbers
                #         hyperbox_states.append(
                #             [float(i) for i in list_of_values])  # Convert list into vector and append
                #     solver_options.update({key: hyperbox_states})
                # elif key == 'grid_points_'
                elif key == 'heartbeat':
                    solver_options.update({key: float(value)})
                elif key == 'precision_timing_bounds':
                    solver_options.update({key: float(value)})
                elif key == 'precision_transitions':
                    solver_options.update({key: float(value)})
                elif key == 'timeout_timing_bounds':
                    solver_options.update({key: int(value)})
                elif key == 'timeout_transitions':
                    solver_options.update({key: int(value)})
                elif key == 'order_approx':
                    assert int(value) > 0
                    solver_options.update({key: int(value)})
                else:
                    continue

                # elif key == 'gridstep':
                #     assert int(value) > 1, "Gridstep should be greater than 1."
                #     solver_options.update({key: int(value)})
                #
                # elif key == 'opt_method':
                #     assert value in ['revised simplex', 'simplex', 'interior-point']
                #     solver_options.update({key: value})
                # elif key == 'heart_beat':
                #     solver_options.update({key: float(value)})
                # elif key == 'grid_pts_per_dim':
                #     values = re.search('\{(.*)\}', value).group(1)
                #     values = values.strip().split(' ')
                #     values = list(filter(None, values))
                #     if len(values) > 0:
                #         sc.check_if_numerical_values(values)
                #         solver_options.update({key: [int(i) for i in values]})
                #
                #
                # elif key == 'remainder_reachability':
                #     solver_options.update({key: float(value)})
                # elif key == 'timeout_reachability':
                #     solver_options.update({key: int(value)})
                # else:
                #     pass
            return solver_options
        except Exception:
            raise IncorrectSyntaxException


    # # If the line is 'linesearch_options'
    # elif line.split(':')[0].strip() == 'Linesearch Options':
    #     linesearch_options = dict()
    #     try:
    #         # Check if no values specified, this data structure can be empty
    #         if len(list(filter(None, line.split(':')[1].strip().split(', ')))) == 0:
    #             return linesearch_options
    #         for item in line.split(':')[1].strip().split(', '):
    #             re.search('[a-z_]+=[a-z0-9.{} ]+', item).group(0)
    #             key = item.split('=')[0]
    #             value = item.split('=')[1]
    #             if key == 'timeout_upper_bounds':
    #                 linesearch_options.update({key: int(value)})
    #             elif key == 'remainder_upper_bounds':
    #                 linesearch_options.update({key: float(value)})
    #             else:
    #                 pass
    #         return linesearch_options
    #     except Exception as e:
    #         raise IncorrectSyntaxException

    else:
        pass


def get_etc_controller(controller):
    """
    The function takes a non-linear data structure and checks following:

    1. number of controller expressions is equal to number of dynamics input symbols
    2. initializes etc_controller by replacing xn with (xn + en), where n=1,2,3...
    3. checks all attributes are initialized expect for optional ones (dynamics_disturbances',
    'solver_options', 'linesearch_options).

    Returns the new data structure object.

    Parameters:
    ----------
        data : InputDataStructureNonLinear class object from input_datastructure module.

    Returns:
    -------
        Modified InputDataStructureNonLinear class object from input_datastructure module.
    """
    dynamics_errors = set()
    etc_controller = list()
    for index, item in enumerate(controller):
        item = str(item)  # Get string representation of the symbol
        for symbol in set(re.findall('x\d+', item)):  # get all state symbols, i.e. x0, x1 ...
            error_symbol = 'e' + re.search('\d+', symbol).group(0)  # Construct error symbol, e.g. x0 -> e0
            dynamics_errors.add(sp.Symbol(error_symbol))  # Create error symbol (en) if doesn't exist
            item = item.replace(symbol, '(' + symbol + ' + ' + error_symbol + ')')  # Replace xn with (xn + en)
        etc_controller.append(sp.sympify(item))
    return dynamics_errors, etc_controller
