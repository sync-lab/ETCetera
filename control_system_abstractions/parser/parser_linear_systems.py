import re
import sys
import numpy as np
import control_system_abstractions.data.nonlinear_systems_datastructure as dat
import control_system_abstractions.parser.syntax_checker as sc


def parse_linear(line):
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

    if line.split(':')[0].strip() == 'Dynamics':
        try:
            dynamics = []
            shape = ()  # Variable to hold the shape of the matrix
            for matrix in line.split(':')[1].strip().split(', '):
                list_of_values = sc.check_keyvalue_syntax(' |;', '\[(.*)\]', matrix)     # Check [..] syntax and get value
                sc.check_if_numerical_values(list_of_values)    # Check if all the items are real numbers
                matrix_shaped = np.array(list_of_values, dtype='f')     # Convert values into a matrix
                shape_current = sc.check_matrix_syntax(matrix)  # Check if matrix has same number of cols for each row
                if len(shape) == 0 or shape[0] == shape_current[0]:
                    shape = shape_current
                    dynamics.append(matrix_shaped.reshape(shape))
                else:
                    raise Exception('Matrices should have same number of rows')
            return dynamics
        except IndexError:
            raise Exception('Syntax error')
        except Exception as e:
            raise Exception(str(e))

    elif line.split(':')[0].strip() == 'Controller':
        try:
            if not len(line.split(':')[1].strip().split(', ')) == 1:   # Check that only one matrix definition present
                raise Exception('More than one matrix specified')
            list_of_values = sc.check_keyvalue_syntax(' |;', '\[(.*)\]', line.split(':')[1].strip())   # Check matrix syntax, get value
            sc.check_if_numerical_values(list_of_values)    # Check if all the items inside matrix are real numbers
            shape_matrix = sc.check_matrix_syntax(line.split(':')[1].strip())  # Check matrix has same number of cols for each row
            matrix_shaped = np.array(list_of_values, dtype='f')     # Convert values into a matrix
            return matrix_shaped.reshape(shape_matrix)
        except IndexError:
            raise Exception('Syntax error')
        except Exception as e:
            raise Exception(str(e))

    elif line.split(':')[0].strip() == 'Triggering Condition':
        try:
            if not len(line.split(':')[1].strip().split(', ')) == 1:   # Check that value contains only one matrix definition
                raise Exception('More than one matrix specified')
            list_of_values = sc.check_keyvalue_syntax(' |;', '\[(.*)\]', line.split(':')[1])   # Check matrix syntax, get value
            sc.check_if_numerical_values(list_of_values)    # Check if all the items inside matrix are real numbers
            shape_matrix = sc.check_matrix_syntax(line.split(':')[1])  # Check matrix has same number of cols for each row
            matrix_shaped = np.array(list_of_values, dtype='f')     # Convert values into a matrix
            return matrix_shaped.reshape(shape_matrix)
        except IndexError:
            raise Exception('Syntax error')
        except Exception as e:
            raise Exception(str(e))

    elif line.split(':')[0].strip() == 'Triggering Heartbeat':
        try:
            if len(line.split(':')[1].strip().split(', ')) > 1:   # Check that value contains only one scalar definition
                raise Exception('More than one scalar value specified')
            sc.check_if_numerical_values(re.split(', ', line.split(':')[1].strip()))    # Check if all the items are real numbers
            return float(line.split(':')[1])
        except IndexError:
            raise Exception('Syntax error')
        except Exception as e:
            raise Exception(str(e))

    elif line.split(':')[0].strip() == 'Triggering Sampling Time':
        try:
            if len(list(filter(None, line.split(':')[1].strip()))) == 0:
                return None
            if len(line.split(':')[1].split(', ')) > 1:    # Check that value contains only one scalar definition
                raise Exception('More than one scalar value specified')
            sc.check_if_numerical_values(re.split(', ', line.split(':')[1].strip()))    # Check if all the items are real numbers
            return float(line.split(':')[1])
        except IndexError:
            raise Exception('Syntax error')
        except Exception as e:
            raise Exception(str(e))

    elif line.split(':')[0].strip() == 'Lyapunov Function':
        try:
            if not len(line.split(':')[1].split(', ')) == 1:   # Check that value contains only one matrix definition
                raise Exception('More than one matrix specified')
            list_of_values = sc.check_keyvalue_syntax(' |;', '\[(.*)\]', line.split(':')[1])   # Check matrix syntax and value
            sc.check_if_numerical_values(list_of_values)    # Check if all the items are real numbers
            shape_matrix = sc.check_matrix_syntax(line.split(':')[1])  # Check matrix has same number of cols for each row
            if not shape_matrix[0] == shape_matrix[1]:  # Check if the matrix is quadratic
                raise Exception('Matrix is not quadratic')
            matrix_shaped = np.array(list_of_values, dtype='f')     # Convert values into a matrix
            return matrix_shaped.reshape(shape_matrix)
        except IndexError:
            raise Exception('Syntax error')
        except Exception as e:
            raise Exception(str(e))

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
                if key == 'p' and float(value) and float(value) > 0:
                    solver_options.update({key: float(value)})
                elif key == 'gridstep' and float(value) and float(value) > 2:
                    solver_options.update({key: float(value)})
                elif key == 'dreal_precision' and float(value):
                    solver_options.update({key: float(value)})
                elif key == 'timeout' and float(value):
                    solver_options.update({key: float(value)})
                elif key == 'opt_method' and value.split('=')[1] in ['simplex', 'interior-point']:
                    solver_options.update({key: value})
                else:
                    raise Exception('Error in value: ')
            return solver_options
        except IndexError:
            raise Exception('Syntax error')
        except Exception as e:
            raise Exception(str(e))

    # If the line is 'Abstraction_options'
    elif line.split(':')[0].strip() == 'Abstraction Options':
        linesearch_options = dict()
        try:
            if len(list(filter(None, line.split(':')[1].strip().split(', ')))) == 0:   # Check if no values specified, this data structure can be empty
                return linesearch_options
            for value in line.split(':')[1].strip().split(', '):
                re.search('[a-z]+=[a-z0-9.]+', value).group(0)
                linesearch_options.update({value.split('=')[0]: float(value.split('=')[1])})
            return linesearch_options
        except IndexError:
            raise Exception('Syntax error')
        except Exception as e:
            raise Exception(str(e))

    else:
        pass
