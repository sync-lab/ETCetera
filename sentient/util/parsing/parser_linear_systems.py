"""
Created on Sat May 16 14:53:58 2020

@author: gmaddodi
"""

import re
import numpy as np
import sentient.util.parsing.syntax_checker as sc
from sentient.exceptions.parser_exceptions.general_parser_exception import IncorrectSyntaxException, \
    MultipleScalarsSpecifiedException
from sentient.exceptions.parser_exceptions.vector_matrix_syntax_exceptions import \
    MatricesUnequalRowsException, MultipleMatricesSpecifiedException, MatrixNotQuadraticException
import traceback

def parse_linear(line):
    """
    The function takes a string representing the key-value pair of linear control system input, checks which input
    datastructure it is and extracts the value. Returns the value in appropriate form if checks are passed, else raises
    an exception.

    Parameters:
    ----------
        line : string
            A string of key and value corresponding to an attribute of InputDataStructureLinear, e.g. Dynamics = [1 2; 3 4].

    Returns:
    -------
        An appropriate data structure (list, string, dictionary).

    Exceptions:
    ----------
        IncorrectSyntaxException, MatricesUnequalRowsException, MultipleMatricesSpecifiedException,
        MultipleScalarsSpecifiedException, MatrixNotQuadraticException
    """

    if line.split(':')[0].strip() == 'Dynamics':
        dynamics = []
        shape = ()  # Variable to hold the shape of the matrix
        try:
            value = line.split(':')[1].strip().split(', ')
        except Exception:
            raise IncorrectSyntaxException
        for matrix in value:
            list_of_values = sc.check_keyvalue_syntax(' |;', '\[(.*)\]', matrix)     # Check [..] syntax and get value
            sc.check_if_numerical_values(list_of_values)    # Check if all the items are real numbers
            matrix_shaped = np.array(list_of_values, dtype='f')     # Convert values into a matrix
            shape_current = sc.check_matrix_syntax(matrix)  # Check if matrix has same number of cols for each row
            if len(shape) == 0 or shape[0] == shape_current[0]: # Check all matrices have same shape
                shape = shape_current
                dynamics.append(matrix_shaped.reshape(shape))
            else:                                               # If matrices not same shape, raise exception
                raise MatricesUnequalRowsException(matrix, shape[0], shape_current[0])
        return dynamics


    elif line.split(':')[0].strip() == 'Controller':
        try:
            value = line.split(':')[1]
        except Exception:
            raise IncorrectSyntaxException
        # Check that only one matrix definition present
        if not len(value.strip().split(', ')) == 1:
            raise MultipleMatricesSpecifiedException
        list_of_values = sc.check_keyvalue_syntax(' |;', '\[(.*)\]', value.strip())   # Check matrix syntax, get value
        sc.check_if_numerical_values(list_of_values)    # Check if all the items inside matrix are real numbers
        shape_matrix = sc.check_matrix_syntax(value.strip())  # Check matrix has same number of cols for each row
        matrix_shaped = np.array(list_of_values, dtype='f')     # Convert values into a matrix
        return matrix_shaped.reshape(shape_matrix)


    elif line.split(':')[0].strip() == 'Triggering Condition':
        try:
            value = line.split(':')[1]
        except Exception:
            raise IncorrectSyntaxException
        # Check that value contains only one matrix definition
        if not len(value.strip().split(', ')) == 1:
            raise MultipleMatricesSpecifiedException
        list_of_values = sc.check_keyvalue_syntax(' |;', '\[(.*)\]', value.strip())   # Check matrix syntax, get value
        sc.check_if_numerical_values(list_of_values)    # Check if all the items inside matrix are real numbers
        shape_matrix = sc.check_matrix_syntax(value.strip())  # Check matrix has same number of cols for each row
        matrix_shaped = np.array(list_of_values, dtype='f')     # Convert values into a matrix
        return matrix_shaped.reshape(shape_matrix)


    elif line.split(':')[0].strip() == 'Triggering Heartbeat':
        try:
            value = line.split(':')[1]
        except Exception:
            raise IncorrectSyntaxException
        # Check that value contains only one scalar definition
        if len(value.strip().split(', ')) > 1:
            raise MultipleScalarsSpecifiedException
        sc.check_if_numerical_values(re.split(', ', value.strip()))    # Check if all the items are real numbers
        return float(value)


    elif line.split(':')[0].strip() == 'Triggering Sampling Time':
        try:
            value = line.split(':')[1]
        except Exception:
            raise IncorrectSyntaxException
        if len(value.strip().split(', ')) == 0:     # It is possible to have no value specified
            return None
        elif len(value.strip().split(', ')) > 1:    # Check if value contains more than one scalar definition
            raise MultipleScalarsSpecifiedException
        else:
            sc.check_if_numerical_values(re.split(', ', value.strip()))    # Check if all the items are real numbers
            return float(value)


    elif line.split(':')[0].strip() == 'Lyapunov Function':
        try:
            value = line.split(':')[1]
        except Exception:
            raise IncorrectSyntaxException
        if not len(value.split(', ')) == 1:   # Check that value contains only one matrix definition
            raise MultipleMatricesSpecifiedException
        list_of_values = sc.check_keyvalue_syntax(' |;', '\[(.*)\]', line.split(':')[1])   # Check matrix syntax and value
        sc.check_if_numerical_values(list_of_values)    # Check if all the items are real numbers
        shape_matrix = sc.check_matrix_syntax(line.split(':')[1])  # Check matrix has same number of cols for each row
        if not shape_matrix[0] == shape_matrix[1]:  # Check if the matrix is quadratic
            raise MatrixNotQuadraticException
        matrix_shaped = np.array(list_of_values, dtype='f')     # Convert values into a matrix
        return matrix_shaped.reshape(shape_matrix)


    # If the line is 'solver_options'
    elif line.split(':')[0].strip() == 'Solver Options':
        solver_options = dict()
        try:
            if len(list(filter(None, line.split(':')[1].strip().split(', ')))) == 0:   # Check if no values specified, this data structure can be empty
                return solver_options
            for item in line.split(':')[1].strip().split(', '):
                re.search('[a-z]+=[a-z0-9.]+', item).group(0)
                key = item.split('=')[0]
                value = item.split('=')[1]
                if key == 'solver':
                    if value not in ['sdr', 'z3']:
                        raise Exception("TEMP NOT CORRECT SOLVER EXCEPTION")
                    else:
                        solver_options.update({key: value})
                if key == 'p' and float(value) and float(value) > 0:
                    solver_options.update({key: float(value)})
                elif key == 'gridstep' and float(value) and float(value) > 2:
                    solver_options.update({key: float(value)})
                elif key == 'dreal_precision' and float(value):
                    solver_options.update({key: float(value)})
                elif key == 'timeout' and float(value):
                    solver_options.update({key: float(value)})
                elif key == 'opt_method' and value.split('=')[1] in ['revised simplex', 'simplex', 'interior-point']:
                    solver_options.update({key: value})
                else:
                    pass
            return solver_options
        except Exception as e:
            raise IncorrectSyntaxException



    # If the line is 'Abstraction_options'
    elif line.split(':')[0].strip() == 'Abstraction Options':
        linesearch_options = dict()
        try:
            if len(list(filter(None, line.split(':')[1].strip().split(', ')))) == 0:   # Check if no values specified, this data structure can be empty
                return linesearch_options
            for value in line.split(':')[1].strip().split(', '):
                print(value)
                re.search('[a-z]+=[a-zA-Z0-9.]+', value).group(0)
                linesearch_options.update({value.split('=')[0]: eval(value.split('=')[1])})
            return linesearch_options
        except Exception as e:
            traceback.print_exc()
            raise IncorrectSyntaxException

    else:
        pass
