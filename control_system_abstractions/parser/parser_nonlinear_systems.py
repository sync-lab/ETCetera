import re
import numpy as np
import control_system_abstractions.parser.syntax_checker as sc
import sympy as sp

from control_system_abstractions.exceptions.parser_exceptions.general_parser_exception import EmptyValueException, \
    MultipleValuesFoundException, NotPositiveRealNumberException, IncorrectSyntaxException
from control_system_abstractions.exceptions.parser_exceptions.symbolic_expression_exceptions import \
    ArbitraryVariableNumberingException, IncorrectNumOfSymbolicExpressionException


def parse_nonlinear(line):
    """
	The function takes a string represting the value of non-linear control system input, checks which input
	datastructure it is and extracts the value. The extracted value is filled into to the provided
	InputDataStructureNonLinear from input_datastructure module.

    Parameters:
    ----------
        line : string
            A string of key and value corresponding to an attribute of InputDataStructureNonLinear.

    Returns:
    -------
        An instance of InputDataStructureNonLinear class.
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
        allowed_chars = list(['x', 'w'])  # Defined allowed characters in symbolic expression
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
            raise IncorrectNumOfSymbolicExpressionException

    # If the line is 'lyapunov_func'
    elif line.split(':')[0].strip() == 'Lyapunov Function':
        lyapunov_func = 0
        allowed_chars = list(['x', 'w'])  # Defined allowed characters in symbolic expression
        try:
            num_exprs = len(line.split(':')[1].strip().split(', '))
        except IndexError:
            raise EmptyValueException
        if num_exprs == 1:  # There should be only one expression
            sc.check_symbols_in_exprs(allowed_chars, ''.join(line.split(':')[1].strip()))  # Get the symbols from expression
            lyapunov_func = sc.check_symbolic_expr(line.split(':')[1].strip()) # Verify the symbolic expression and append
            return lyapunov_func
        else:
            raise IncorrectNumOfSymbolicExpressionException

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

    # If the line is 'hyperbox_disturbances'
    elif line.split(':')[0].strip() == 'Hyperbox Disturbances':
        hyperbox_disturbances = []
        try:
            hyperbox_disturbances_vectors = line.split(':')[1].strip().split(', ')
        except IndexError:
            raise IncorrectSyntaxException
        for item in hyperbox_disturbances_vectors:   # Split the vectors delimited by ','
            list_of_values = sc.check_keyvalue_syntax(' ', '\[(.*)\]', item)    # Check the vector syntax
            sc.check_if_numerical_values(list_of_values)    # Check the values are real numbers
            hyperbox_disturbances.append([float(i) for i in list_of_values])  # Convert list into vector and append
        return hyperbox_disturbances


    # If the line is 'triggering_times'
    elif line.split(':')[0].strip() == 'Triggering Times':
        triggering_times = []
        try:
            value = line.split(':')[1].strip()
        except IndexError:
            raise EmptyValueException
        sc.check_if_numerical_values(re.split(', ', value))    # Check that comma separated values are numerical
        for time in line.split(':')[1].strip().split(', '):
            triggering_times.append(float(time))
        return triggering_times


    # If the line is 'deg_of_homogeneity'
    elif line.split(':')[0].strip() == 'Deg. of Homogeneity':
        try:
            value = line.split(':')[1]
        except IndexError:
            raise EmptyValueException
        if len(value.strip().split(',')) != 1:  # There should be only one value
            raise MultipleValuesFoundException
        sc.check_if_numerical_values(value.strip().split(' '))     # Check if float value
        if float(value.strip()) < 0:
            raise NotPositiveRealNumberException
        return float(value.strip())


    # If the line is 'solver_options'
    elif line.split(':')[0].strip() == 'Solver Options':
        solver_options = dict()
        solver_options.update({'opt_method': 'revised simplex'})
        try:
            if len(list(filter(None, line.split(':')[1].strip().split(', ')))) == 0:   # Check if no values specified, this data structure can be empty
                return solver_options

            for item in line.split(':')[1].strip().split(', '):
                re.search('[a-z]+=[a-z0-9.]+', item).group(0)
                key = item.split('=')[0]
                value = item.split('=')[1]

                if key == 'p':
                    assert int(value) > 0
                    solver_options.update({key: int(value)})
                elif key == 'gridstep':
                    assert int(value) > 2
                    solver_options.update({key: int(value)})
                elif key == 'dreal_precision':
                    solver_options.update({key: float(value)})
                elif key == 'timeout':
                    solver_options.update({key: int(value)})
                elif key == 'opt_method':
                    assert value in ['revised simplex', 'simplex', 'interior-point']
                    solver_options.update({key: value})
                else:
                    pass
            return solver_options
        except Exception:
            raise IncorrectSyntaxException


    # If the line is 'linesearch_options'
    elif line.split(':')[0].strip() == 'Linesearch Options':
        linesearch_options = dict()
        try:
            if len(list(filter(None, line.split(':')[1].strip().split(', ')))) == 0:   # Check if no values specified, this data structure can be empty
                return linesearch_options
            for value in line.split(':')[1].strip().split(', '):
                re.search('[a-z]+=[a-z0-9.]+', value).group(0)
                linesearch_options.update({value.split('=')[0]: float(value.split('=')[1])})
            return linesearch_options
        except Exception as e:
            raise IncorrectSyntaxException

    else:
        pass


def get_etc_controller(controller):
    """
    The function takes a non-linear data structure and checks following: 1. number of controller expressions is
    equal to number of dynamics input symbols, 2. initializes etc_controller by replacing xn with (xn + en)
    where n=1,2,3..., and 3. checks all attributes are initialized expect for optional ones (dynamics_disturbances',
    'solver_options', 'linesearch_options). Returns the new data structure object.

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
