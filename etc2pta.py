#!/usr/bin/python
"""
Created on Sat May 16 14:53:58 2020

@author: gmaddodi
"""

import re
import sys, getopt
import os 
#import resources 
import random
import string
import control_system_abstractions.parser.parser_linear_systems as lp
import control_system_abstractions.parser.parser_nonlinear_systems as nlp
import control_system_abstractions.data.nonlinear_systems_datastructure as nld
import control_system_abstractions.data.linear_systems_datastructure as ld
import control_system_abstractions.logic.nonlinear_systems as nonlinear_logic
import control_system_abstractions.logic.linear_systems as linear_logic
import sympy as sp


def main(argv):
    inputfile = ''      # Variable to hold input file path
    systemtype = ''     # Variable to hold system type data, either 'linear' or 'non-linear'

    # Read the data from console
    try:
        opts, args = getopt.getopt(argv, "hi:s:", ["ifile=", "systype="])
    except getopt.GetoptError:
        print('etc2pta.py -i <inputfile> -s <systemtype>')
        sys.exit(2)

    for opt, arg in opts:
        if opt == '-h':                                                 # If user asks for info on input arguments
            print('etc2pta.py -i <inputfile> -s <systemtype>')
            sys.exit()
        elif opt in ("-i", "--ifile") and os.path.exists(arg):          # If argument is file string and file exists
            inputfile = arg
        elif opt in ("-i", "--ifile") and not (os.path.exists(arg)):    # If argument is file string and file exists
            sys.exit('No input file specified: ', arg)
        elif opt in ("-s", "--systype") and arg in ("linear", "non-linear"):    # if argument is systemtype and its linear or non-linear
            systemtype = arg
        else:                                                           # In all other cases incorrect input specified
            sys.exit("Incorrect system type specified!")

    #init_models_path()

    if systemtype == 'linear':
        # Dictionary to hold linear data
        dict_key_to_attrs = {'Dynamics':  None, 'Controller': None, 'Triggering Heartbeat': None,
                             'Triggering Condition': None, 'Triggering Sampling Time': None, 'Lyapunov Function': None,
                             'Solver Options': {}, 'Abstraction Options': {}}
    else:
        # Dictionary to hold non-linear data
        dict_key_to_attrs = {'Dynamics': None, 'Controller': None, 'Hyperbox States': None,
                             'Triggering Condition': None, 'Triggering Times': None, 'Hyperbox Disturbances': None,
                             'Solver Options': {'p': 1, 'opt_method': None, 'gridstep': 2, 'dreal_precision': 0.01},
                             'Linesearch Options': {}, 'Lyapunov Function': None, 'Deg. of Homogeneity': None}
        # Dictionary to hold symbols in expressions
        dict_symbol_to_attr = {'e': set(), 'u': set(), 'd': set(), 'x': set()}

    try:
        with open(inputfile, 'r') as reader:
            for line_num, line in enumerate(reader, 1):         # Read line along with line number
                line_key = line.split(':')[0].strip()           # Get the key in the line
                print(line_key)
                if line_key in dict_key_to_attrs.keys() and systemtype == 'linear':     # Check if correct key
                    parsed_data = lp.parse_linear(line)         # Call the linear parser function with line
                elif line_key in dict_key_to_attrs.keys() and systemtype == 'non-linear':
                    parsed_data = nlp.parse_nonlinear(line)     # Call the non-linear parser function with line
                else:
                    raise Exception('Incorrect key string!')

                if line_key in ('Solver Options', 'Linesearch Options'):  # If data is 'Solver Options', merge with default already defined
                    dict_key_to_attrs[line_key].update(parsed_data)
                else:                               # For all other data, simple assignment is fine
                    dict_key_to_attrs[line_key] = parsed_data

        if systemtype == 'linear':
            # triggering sampling time should be lower than triggering heartbeat
            if dict_key_to_attrs['Triggering Sampling Time'] and \
                    dict_key_to_attrs['Triggering Sampling Time'] < dict_key_to_attrs['Triggering Sampling Time']:
                raise Exception('Triggering sampling time should be lower than triggering heartbeat')

            is_PETC = True if (dict_key_to_attrs['Triggering Sampling Time']) else False

            print(dict_key_to_attrs)
            print(is_PETC)

            # Build plant and controller
            data_obj = ld.InputDataStructureLinear(
                dynamics=dict_key_to_attrs['Dynamics'],
                controller=dict_key_to_attrs['Controller'],
                triggering_heartbeat=dict_key_to_attrs['Triggering Heartbeat'],
                triggering_condition=dict_key_to_attrs['Triggering Condition'],
                triggering_sampling_time=dict_key_to_attrs['Triggering Sampling Time'],
                lyapunov_func=dict_key_to_attrs['Lyapunov Function'],
                solver_options=dict_key_to_attrs['Solver Options'],
                abstraction_options=dict_key_to_attrs['Abstraction Options'],
                is_PETC=is_PETC
            )

            print(data_obj.__dict__)

            # Add the field traffic_model to data_obj, which contains the
            # abstraction.
            linear_logic.create_abstractions(data_obj)
            print(data_obj.traffic_model.__dict__)  # Print fields

            # This builds the priced timed automaton from the traffic model
            pta = linear_logic.traffic2ta(data_obj.traffic_model)
            print(pta)

        else:
            # Get all the symbols from expressions and assign symbols to correct attr
            all_exprs = str(dict_key_to_attrs['Dynamics']) + str(dict_key_to_attrs['Controller']) + \
                      str(dict_key_to_attrs['Triggering Condition']) + str(dict_key_to_attrs['Lyapunov Function'])
            set_symbols = set(re.findall('[a-z]\\d+', all_exprs))       # Get symbols of the form x0, e0, ...
            for symbol in set_symbols:
                dict_symbol_to_attr[symbol[0]].add(sp.Symbol(symbol))   # Add the symbol to correct attr

            # Number of controller exprs and dynamics inputs have be equal
            #if not len(dict_symbol_to_attr['u']) == len(dict_key_to_attrs['Controller']):
            #    raise Exception('Incorrect number of controller expressions!')
            assert len(dict_symbol_to_attr['u']) == len(dict_key_to_attrs['Controller']), \
                'Incorrect number of controller expressions!'

            # Generate etc_controller data from controller data
            dynamics_errors, etc_controller = nlp.get_etc_controller(dict_key_to_attrs['Controller'])
            dict_symbol_to_attr['e'] = dynamics_errors.union(dynamics_errors)      # Union with existing error symbols
            print('error from',etc_controller)

            dynamics_new = []
            for expr in dict_key_to_attrs['Dynamics']:
                expr = str(expr)
                for sym in re.findall('u\d+', expr):
                    repl = etc_controller[int(re.search('\d+', sym).group(0))]
                    expr = str.replace(expr, sym, str(repl))
                dynamics_new.append(sp.sympify(expr))
            dynamics_new.extend([-1 * expr for expr in dynamics_new])

            # Check if only one time mentioned, then the system is homogeneous
            is_homogenized = True if (len(dict_key_to_attrs['Triggering Times']) == 1) else False

            # To get parameters, sort the d symbols
            d_str_sorted = sorted([str(i) for i in dict_symbol_to_attr['d']])
            parameters = tuple(sp.Symbol(i) for i in d_str_sorted)

            # State is a union of sorted x and e symbols
            x_str_sorted = sorted([str(i) for i in dict_symbol_to_attr['x']])
            e_str_sorted = sorted([i.replace('x', 'e') for i in x_str_sorted])
            state_str = x_str_sorted + e_str_sorted
            state = tuple(sp.Symbol(i) for i in state_str)

            # Init conditions is tuple to x replaced with a
            a_str_sorted = sorted([i.replace('x', 'a') for i in x_str_sorted])
            init_cond_symbols = tuple(sp.Symbol(i) for i in a_str_sorted)

            path, dreal_path, dreach_path, flowstar_path = None, None, None, None
            data_obj = nld.InputDataStructureNonLinear(path, dreal_path, dreach_path, flowstar_path,
                                                       dynamics_new,
                                                       dict_key_to_attrs['Deg. of Homogeneity'],
                                                       dict_key_to_attrs['Lyapunov Function'],
                                                       random.uniform(0.01, 0.1),
                                                       dict_key_to_attrs['Triggering Condition'],
                                                       state,
                                                       init_cond_symbols,
                                                       parameters,
                                                       dict_key_to_attrs['Hyperbox Disturbances'],
                                                       dict_key_to_attrs['Solver Options']['p'],
                                                       dict_key_to_attrs['Solver Options']['gridstep'],
                                                       dict_key_to_attrs['Solver Options']['dreal_precision'],
                                                       is_homogenized)
            print(data_obj.__dict__)
            #nonlinear_logic.create_abstractions(data_obj)
    except Exception as e:
        if str(e) == 'Incorrect number of controller expressions!!':
            print(str(e))
        elif str(e) == 'Incomplete input data!':
            print(str(e))
        elif str(e) == 'Triggering sampling time should be lower than triggering heartbeat!':
            print(str(e))
        elif str(e) == 'Incorrect key string!':
            print(str(e))
        else:
            print(str(e) + str(line_num))
        sys.exit()

   
def init_models_path():    
    letters = string.ascii_lowercase
    dir_files = ''.join(random.choice(letters) for i in range(8))
    app_path = os.path.dirname(os.path.abspath(__file__))
    res_path = os.path.join(app_path, "resources")
    files_path = os.path.join(res_path, dir_files)
    if not os.path.exists(files_path):
        os.mkdir(files_path)
    #print (os.path.join(os.path.realpath(__file__), 'requirements.txt'))
    return files_path

if __name__ == "__main__":
    main(sys.argv[1:])
