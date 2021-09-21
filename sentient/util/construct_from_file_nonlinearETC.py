import os, sys
import re
import sympy
import logging

from sentient.exceptions import *
import sentient.util.parsing as parsing
import sentient.Abstractions as abstr
from sentient.util.homogeneous import *
from config import *

def construct_nonlinearETC_traffic_from_file(file_name, CreateAbstraction=True):
    if not os.path.exists(file_name):
        print(f'File {file_name} could not be found.')
        return None

    # Dictionary to hold non-linear data, and default values where applicable
    dict_key_to_attrs = {'Dynamics': None, 'Controller': None, 'Deg. of Homogeneity': None,
                         'Triggering Condition': None,
                         'Hyperbox States': None, 'Grid Points Per Dimension': None, 'Hyperbox Disturbances': None,
                         'Solver Options': {'precision_deltas': 1e-4, 'timeout_deltas': 1000,
                                            'partition_method': 'grid',
                                            'manifolds_times': None, 'nr_cones_small_angles': [],
                                            'nr_cones_big_angle': 5,
                                            'state_space_limits': None, 'grid_points_per_dim': None, 'heartbeat': 0.1,
                                            'precision_timing_bounds': 1e-3, 'precision_transitions': 1e-3,
                                            'timeout_timing_bounds': 200, 'timeout_transitions': 200,
                                            'order_approx': 2}}



    # Dictionary to hold symbols in expressions
    dict_symbol_to_attr = {'e': set(), 'u': set(), 'd': set(), 'x': set(), 'w': set()}
    try:
        with open(file_name, 'r') as reader:
            for line_num, line in enumerate(reader, 1):  # Read line along with line number
                line_key = line.split(':')[0].strip()  # Get the key in the line
                if line_key in dict_key_to_attrs.keys():
                    parsed_data = parsing.nlp.parse_nonlinear(line)  # Call the non-linear parsing function with line
                elif line_key == '':
                    continue
                else:
                    raise IllegalKeyException(param=[line_key])

                if line_key in ('Solver Options'):  # If data is 'Solver Options', merge with default already defined
                    dict_key_to_attrs[line_key].update(parsed_data)
                else:  # For all other data, simple assignment is fine
                    dict_key_to_attrs[line_key] = parsed_data

    except GenericParsingException as e:
        print(f'Parsing Error Occured on line {line_num}:     {line}')
        print(str(e))
        sys.exit()

    except SentientBaseException as e:
        print(str(e) + str(line_num))
        print(type(e))
        sys.exit()

    dict_key_to_attrs['Solver Options']['state_space_limits'] = dict_key_to_attrs['Hyperbox States']
    dict_key_to_attrs['Solver Options']['grid_points_per_dim'] = dict_key_to_attrs['Grid Points Per Dimension']

    # # Sha it
    # import base64
    # import hashlib
    # rep = base64.b64encode(str(dict_key_to_attrs).encode('utf-8'))
    # sha = hashlib.sha224()
    # sha.update(rep)
    # fname = f'nl_{sha.digest().hex()}.pickle'
    # if os.path.exists(os.path.join(save_path, fname)):
    #     print("Traffic model found, load from save!")

    # Get all the symbols from expressions and assign symbols to correct attr
    all_exprs = str(dict_key_to_attrs['Dynamics']) + str(dict_key_to_attrs['Controller']) + \
                str(dict_key_to_attrs['Triggering Condition'])
    set_symbols = set(re.findall('[a-z]\\d+', all_exprs))  # Get symbols of the form x0, e0, ...

    # Add the symbol to correct attr in 'dict_symbol_to_attr'
    for symbol in set_symbols:
        dict_symbol_to_attr[symbol[0]].add(symbol)


    # First check whether all compulsory inputs are given
    # Required always
    _required_args = ['Dynamics', 'Triggering Condition', 'Hyperbox States', 'Controller']

    _missing_args = []
    for i in _required_args:
        if dict_key_to_attrs[i] is None:
            _missing_args.append(i)

    if len(_missing_args) > 0:
        raise IncompleteInputFileException(_missing_args)

    # Check that each symbol type numbering begins with '1' and is sequential, e.g. x1, x2, ...
    for key in dict_symbol_to_attr.keys():
        sorted_list = sorted(list(dict_symbol_to_attr[key]))
        i = 1
        for item in sorted_list:
            if not int(re.split('(\d.*)', item)[1]) == i:
                # print('Incorrect variable numbering')
                raise ArbitraryVariableNumberingException()
                # sys.exit()
            i += 1

    if not (len(dict_symbol_to_attr['x']) == len(dict_key_to_attrs['Dynamics']) or \
            len(dict_symbol_to_attr['x']) + len(dict_symbol_to_attr['w']) == len(dict_key_to_attrs['Dynamics'])):
        raise IncorrectNumOfSymbolicExpressionException('Dynamics', len(dict_key_to_attrs['Dynamics']),
                                                        len(dict_symbol_to_attr['x']))


    # Number of controller exprs and dynamics inputs have be equal
    if not len(dict_symbol_to_attr['u']) == len(dict_key_to_attrs['Controller']):
        # print('Incorrect number of controller expressions!')
        raise IncorrectNumOfSymbolicExpressionException('Controller', len(dict_key_to_attrs['Controller']),
                                                            len(dict_symbol_to_attr['u']))

    # TODO: Allow more general triggering functions
    # Number of error variables should be equal to the number of state variables?
    # if not len(dict_symbol_to_attr['e']) == len(dict_symbol_to_attr['x']) + len(dict_symbol_to_attr['w']):
    #     raise IncorrectNumberOfVariablesSpecifiedException('e', len(dict_symbol_to_attr['e']),
    #                                                        len(dict_symbol_to_attr['x']) + len(dict_symbol_to_attr['w']))
    if not len(dict_symbol_to_attr['e']) == len(dict_symbol_to_attr['x']):
        raise IncorrectNumberOfVariablesSpecifiedException('e', len(dict_symbol_to_attr['e']),
                                                           len(dict_symbol_to_attr['x']))


    # If 'd' variables specified, 'Hyperbox Disturbances' should not be empty
    if re.search('[d]\d+', ''.join(set_symbols)) and not dict_key_to_attrs['Hyperbox Disturbances']:
        # print('If \'d\' variables specified, \'Hyperbox Disturbances\' should not be empty')
        raise IncompleteInputFileException(['Hyperbox Disturbances'])

    if dict_key_to_attrs['Solver Options']['partition_method'] == 'grid' and \
            not dict_key_to_attrs['Solver Options']['manifolds_times']:
        dict_key_to_attrs['Solver Options']['manifolds_times'] = [1e-4]

    if dict_key_to_attrs['Solver Options']['partition_method'] == 'manifold':
        # Number of values for 'nr_cones_small_angles' should be equal to num of 'x' variables
        if not dict_key_to_attrs['Solver Options']['nr_cones_small_angles']:
            # If not user specified, create list of len of 'x'
            dict_key_to_attrs['Solver Options']['nr_cones_small_angles'] = [4] * (len(dict_symbol_to_attr['x']) - 2)
        elif (len(dict_symbol_to_attr['x']) - 2) != len(dict_key_to_attrs['Solver Options']['nr_cones_small_angles']):
            raise IncorrectNumberOfItemsInListException('x', len(dict_symbol_to_attr['x']) - 2,
                                    len(dict_key_to_attrs['Solver Options']['nr_cones_small_angles']))

        if not dict_key_to_attrs['Solver Options']['manifolds_times']:
            raise IncompleteInputFileException(['manifolds_times'])
        elif len(dict_key_to_attrs['Solver Options']['manifolds_times']) == 1:
            raise IncorrectNumberOfItemsInListException('manifold_times', '> 1', 1)

    # TODO: Check whether dynamics and controller already in ETC form (i.e. contains error signals)
    #       Should then also check whether the format is then correct (i.e. d/dt[z e] = [dz/dt 0])
    # Generate etc_controller data from controller data
    dynamics_errors, etc_controller = parsing.nlp.get_etc_controller(dict_key_to_attrs['Controller'])
    dict_symbol_to_attr['e'] = dynamics_errors

    # Replace the control variables with their corresponding expressions
    replacement_dict = {i: etc_controller[int(i[1:]) - 1] for i in dict_symbol_to_attr['u']}
    dynamics_new = [expr.subs(replacement_dict) for expr in dict_key_to_attrs['Dynamics']]



    # if is_homogenized:
    #     dynamics_new += [0]
    dynamics_new += [-1 * expr for expr in dynamics_new]
    # dict_key_to_attrs['Dynamics'] = dynamics_new#sympy.Matrix(dynamics_new)


    # TODO: Check it by the definition of homogeneous systems and whether it is correct
    # If 'w1' variable is specified, then system is homogenized
    if 'w1' in set_symbols:
        is_homogenized = True

        # Check whether specified deg. of hom. is correct
        xwvars = list(dict_symbol_to_attr['x'])
        xwvars += list(dict_symbol_to_attr['w'])
        xwvars += list(dict_symbol_to_attr['e'])
        xwvars = [sympy.Symbol(i) if type(i) is str else i for i in xwvars]
        logging.info(f'XW vars: {xwvars}')
        # alpha = test_homogeneity(dict_key_to_attrs['Dynamics'], xwvars)
        alpha = test_homogeneity(dynamics_new, xwvars)
        if (x := dict_key_to_attrs['Deg. of Homogeneity']) is not None and int(x) != alpha:
            logging.warning(f'Warning: Specified Deg. of Homogeneity {int(x)} does not match the calculated one: {alpha}' \
            'Using the specified one.')
        if dict_key_to_attrs['Deg. of Homogeneity'] is None:
            dict_key_to_attrs['Deg. of Homogeneity'] = alpha

        # Append dynamics for 'w' variable if needed
        if len(dynamics_new) == len(dict_symbol_to_attr['x']) + len(dict_symbol_to_attr['e']):
            dynamics_new.insert(len(dict_symbol_to_attr['x']), 0)
            dynamics_new += [0]

    else:
        # First check whether the system is then homogeneous already. If no homogenize itself with desired hom. degree
        # (Deg. of Homogeneity)
        # xwvars = dict_symbol_to_attr['x'].copy()
        # xwvars.update(dict_symbol_to_attr['w'])
        # xwvars.update(dict_symbol_to_attr['e'])
        # xwvars = {sympy.Symbol(i) if type(i) is str else i for i in xwvars}

        xwvars = list(dict_symbol_to_attr['x'])
        xwvars += list(dict_symbol_to_attr['w'])
        xwvars += list(dict_symbol_to_attr['e'])
        xwvars = [sympy.Symbol(i) if type(i) is str else i for i in xwvars]
        alpha = test_homogeneity(dynamics_new, xwvars)
        if alpha:
            is_homogenized = False

        else:
            is_homogenized = True
            des_deg = dict_key_to_attrs['Deg. of Homogeneity'] or 2
            dynamics_new, b = make_homogeneous_etc(dynamics_new, list(xwvars), des_deg)
            dict_symbol_to_attr['w'].add('w1')
            dict_symbol_to_attr['e'].add('ew')
            dict_key_to_attrs['Deg. of Homogeneity'] = des_deg

    dict_key_to_attrs['Dynamics'] = sympy.Matrix(dynamics_new)

    if dict_key_to_attrs['Solver Options']['partition_method'] == 'grid' or is_homogenized:
        # Number of values for 'grid_pts_per_dim' should be equal to num of 'x' variables
        if not dict_key_to_attrs['Solver Options']['grid_points_per_dim']:
            # If not user specified, create list of len of 'x'
            dict_key_to_attrs['Solver Options']['grid_points_per_dim'] = [5] * len(dict_symbol_to_attr['x'])
        elif len(dict_symbol_to_attr['x']) != len(dict_key_to_attrs['Solver Options']['grid_points_per_dim']):
            raise IncorrectNumberOfItemsInListException('grid_points_per_dim',len(dict_symbol_to_attr['x']), len(dict_symbol_to_attr['x']))



    # To get parameters, sort the d symbols
    d_str_sorted = sorted([i for i in dict_symbol_to_attr['d']])
    disturbance_symbols = tuple(sympy.Symbol(i) for i in d_str_sorted)

    # State is a union of sorted x and e symbols
    x_str_sorted = sorted([i for i in dict_symbol_to_attr['x']])

    # a_str_sorted = sorted([i.replace('x', 'a') for i in x_str_sorted])
    # a_str_sorted.append('aw') if 'w1' in dict_symbol_to_attr['w'] else print('')
    e_str_sorted = sorted([i.replace('x', 'e') for i in dict_symbol_to_attr['x']])
    e_str_sorted.append('ew') if 'w1' in dict_symbol_to_attr['w'] else print('')  # only append if w1 exists
    # init_cond_symbols = tuple(sympy.Symbol(i) for i in a_str_sorted)
    init_cond_symbols = None

    x_str_sorted.append(list(dict_symbol_to_attr['w'])[0]) if 'w1' in dict_symbol_to_attr['w'] else print('')
    state_str = x_str_sorted + e_str_sorted
    state_vector = tuple(sympy.Symbol(i) for i in state_str)

    logging.info(f"Dynamics: {dict_key_to_attrs['Dynamics']}")
    logging.info(f"State Vector: {state_vector}")

    traffic = abstr.TrafficModelNonlinearETC(dict_key_to_attrs['Dynamics'], dict_key_to_attrs['Deg. of Homogeneity'],
                                             dict_key_to_attrs['Triggering Condition'], state_vector, init_cond_symbols,
                                             disturbance_symbols, dict_key_to_attrs['Hyperbox Disturbances'],
                                             homogenization_flag=is_homogenized, **dict_key_to_attrs['Solver Options'])

    # traffic.export(fname, 'pickle')

    if CreateAbstraction:
        traffic.create_abstraction()

    return traffic
