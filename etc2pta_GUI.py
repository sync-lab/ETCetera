
"""
Created on Sat May 16 14:53:58 2020

@author: gmaddodi
"""

import random
from tkinter import *
import control_system_abstractions.parser.parser_linear_systems as lp
import control_system_abstractions.parser.parser_nonlinear_systems as nlp
import control_system_abstractions.data.linear_systems_datastructure as ld
import sympy as sp
#import control_system_abstractions.nonlinear_systems_utils as nonlinear
import control_system_abstractions.data.nonlinear_systems_datastructure as nld
import control_system_abstractions.logic.nonlinear_systems as nonlinear_logic
import control_system_abstractions.logic.linear_systems as linear_logic
from config import dreach_path, flowstar_path, path, dreal_path

# Linear system layout definitions
from control_system_abstractions.exceptions.nonlinear_systems_exceptions.LP_exceptions import \
    LPOptimizationFailedException, LPGeneralException
from control_system_abstractions.exceptions.nonlinear_systems_exceptions.data_object_exceptions import \
    DataObjectGeneralException
from control_system_abstractions.exceptions.parser_exceptions.general_parser_exception import \
    NonnumbericValuesFoundException, MultipleValuesFoundException, NotPositiveRealNumberException, \
    IncorrectSyntaxException, MultipleScalarsSpecifiedException, GenericParsingException
from control_system_abstractions.exceptions.parser_exceptions.symbolic_expression_exceptions import \
    ArbitraryVariableNumberingException, ArbitraryVariableNamingException, IncorrectSymbolicExpressionException, \
    IncorrectNumOfSymbolicExpressionException
from control_system_abstractions.exceptions.parser_exceptions.vector_matrix_syntax_exceptions import \
    IncorrectMatrixBoundaryException, IncorrectVectorBoundaryException, NonnumericValueInMatrixException, \
    NonnumericValueInVectorException, IncorrectMatrixDefinitionException, MatricesUnequalRowsException, \
    MultipleMatricesSpecifiedException, MatrixNotQuadraticException


class LinearSystemLayout:
    """
        Class to hold labels and entries of linear system layout on the tkinter based GUI.

        Attributes:
        ----------
            dynamics_label: Text label for 'Dynamics'
            dynamics_entry: Text entry for 'Dynamics'
            controller_label: Text label for 'Controller'
            controller_entry: Text entry for 'Controller'
            triggering_condition_label: Text label for 'Triggering Condition'
            triggering_condition_entry: Text entry for 'Triggering Condition'
            triggering_heartbeat_label: Text label for 'Triggering Heartbeat'
            triggering_heartbeat_entry: Text entry for 'Triggering Heartbeat'
            triggering_samplingtime_label: Text label for 'Triggering Sampling Time'
            triggering_samplingtime_entry: Text entry for 'Triggering Sampling Time'
            lyapunov_func_label: Text label for 'Lyapunov Function'
            lyapunov_func_entry: Text entry for 'Lyapunov Function'
            solver_options_label: Text label for 'Solver Options'
            solver_options_entry: Text entry for 'Solver Options'
            abstraction_options_label: Text label for 'Abstaction Options'
            abstraction_options_entry: Text entry for 'Abstaction Options'

    """
    def __init__(self, frame):
        self.dynamics_label = Label(frame, text='Dynamics')
        self.dynamics_entry = Entry(frame)
        self.controller_label = Label(frame, text='Controller')
        self.controller_entry = Entry(frame)
        self.triggering_condition_label = Label(frame, text='Triggering Condition')
        self.triggering_condition_entry = Entry(frame)
        self.triggering_heartbeat_label = Label(frame, text='Triggering Heartbeat')
        self.triggering_heartbeat_entry = Entry(frame)
        self.triggering_samplingtime_label = Label(frame, text='Triggering Sampling Time')
        self.triggering_samplingtime_entry = Entry(frame)
        self.lyapunov_func_label = Label(frame, text='Lyapunov Function')
        self.lyapunov_func_entry = Entry(frame)
        self.solver_options_label = Label(frame, text='Solver Options')
        self.solver_options_entry = Entry(frame)
        self.abstraction_options_label = Label(frame, text='Abstraction Options')
        self.abstraction_options_entry = Entry(frame)


# Non-linear system definitions
class NonlinearSystemLayout:
    """
        Class to hold labels and entries of non-linear system layout on the tkinter based GUI.

        Attributes:
        ----------
            hyperbox_states_label: Text label for 'Hyperbox States'
            hyperbox_states_entry: Text entry for 'Hyperbox States'
            hyperbox_disturbances_label: Text label for 'Hyperbox Disturbances'
            hyperbox_disturbances_entry: Text entry for 'Hyperbox Disturbances'
            dynamics_label: Text label for 'Dynamics'
            dynamics_entry: Text entry for 'Dynamics'
            controller_label: Text label for 'Controller'
            controller_entry: Text entry for 'Controller'
            triggering_condition_label: Text label for 'Triggering Condition'
            triggering_condition_entry: Text entry for 'Triggering Condition'
            triggering_times_label: Text label for 'Triggering Times'
            triggering_times_entry: Text entry for 'Triggering Times'
            deg_of_homogeneity_label: Text label for 'Deg of Homogeneity'
            deg_of_homogeneity_entry: Text entry for 'Deg of Homogeneity'
            lyapunov_func_label: Text label for 'Lyapunov Function'
            lyapunov_func_entry: Text entry for 'Lyapunov Function'
            solver_options_label: Text label for 'Solver Options'
            solver_options_entry: Text entry for 'Solver Options'
            linesearch_options_label: Text label for 'Linesearch Options'
            linesearch_options_entry: Text entry for 'Linesearch Options'
    """
    def __init__(self, frame):
        self.hyperbox_states_label = Label(frame, text='Hyperbox States')
        self.hyperbox_states_entry = Entry(frame)
        self.hyperbox_disturbances_label = Label(frame, text='Hyperbox Disturbances')
        self.hyperbox_disturbances_entry = Entry(frame)
        self.dynamics_label = Label(frame, text='Dynamics')
        self.dynamics_entry = Entry(frame)
        self.controller_label = Label(frame, text='Controller')
        self.controller_entry = Entry(frame)
        self.triggering_condition_label = Label(frame, text='Triggering Condition')
        self.triggering_condition_entry = Entry(frame)
        self.triggering_times_label = Label(frame, text='Triggering Times')
        self.triggering_times_entry = Entry(frame)
        self.lyapunov_func_label = Label(frame, text='Lyapunov Function')
        self.lyapunov_func_entry = Entry(frame)
        self.deg_of_homogeneity_label = Label(frame, text='Deg. of Homogeneity')
        self.deg_of_homogeneity_entry = Entry(frame)
        self.solver_options_label = Label(frame, text='Solver Options')
        self.solver_options_entry = Entry(frame)
        self.linesearch_options_label = Label(frame, text='Linesearch Options')
        self.linesearch_options_entry = Entry(frame)


def toggle_linear_or_nonlinear():
    """
        Function toggles layout between linear and non-linear based on user's selection. It clear the current selection,
        and creates layout for the other.
    """
    if var_system_type.get():  # Linear system layout
        for k, v in layout_nonlinear.__dict__.items():
            getattr(layout_nonlinear, k).grid_forget()
        grids_label_and_entries(layout_linear)
    else:               # Non-linear system layout
        for k, v in layout_linear.__dict__.items():
            getattr(layout_linear, k).grid_forget()
        grids_label_and_entries(layout_nonlinear)


def submit_data():
    """
        Function reads the data from the GUI, checks validity, creates data object for linear or non-linear system, and
        calls the functions to create abstractions.
    """
    parsing_success = True
    if var_system_type.get():  # Linear system data submitted
        layout_current = layout_linear
        # Dictionary to hold linear data
        dict_key_to_attrs = {'Dynamics': None, 'Controller': None, 'Triggering Heartbeat': None,
                             'Triggering Condition': None, 'Triggering Sampling Time': None,
                             'Lyapunov Function': None, 'Solver Options': {}, 'Abstraction Options': {}}
    else:               # Non-linear system data submitted
        layout_current = layout_nonlinear
        # Dictionary to hold non-linear data
        dict_key_to_attrs = {'Dynamics': None, 'Controller': None, 'Hyperbox States': None,
                             'Triggering Condition': None, 'Triggering Times': None, 'Hyperbox Disturbances': None,
                             'Solver Options': {'p': 1, 'gridstep': 2, 'heart_beat': float(0.4), 'manifolds_times': None,
                                                'dreal_precision_deltas': 0.001, 'timeout_deltas': 1000,
                                                'opt_method': 'revised simplex', 'remainder_reachability': 0.1,
                                                'timeout_reachability': None, 'grid_pts_per_dim': None,
                                                'nr_cones_small_angles': None, 'nr_cones_big_angle': 4},
                             'Linesearch Options': {'timeout_upper_bounds': 15, 'remainder_upper_bounds': 0.1},
                             'Lyapunov Function': None, 'Deg. of Homogeneity': None}
        # Dictionary to hold symbols in expressions
        dict_symbol_to_attr = {'e': set(), 'u': set(), 'd': set(), 'x': set(), 'w': set()}

    set_entries_border_black(var_system_type.get())    # Set the border to default white

    parsed_data = None
    try:
        for k, v in layout_current.__dict__.items():    # Iterate over labels and entries
            if 'label' in v._name:      # If label, get the text
                line = v.cget('text')    # Append the text to line
            elif 'entry' in v._name and var_system_type.get():    # If entry and linear system
                parsed_data = lp.parse_linear(line + ' : ' + v.get())
            else:           # If entry and non-linear system
                parsed_data = nlp.parse_nonlinear(line + ' : ' + v.get())

            # If data is 'Solver Options', merge with default already defined
            if parsed_data is not None and line in ('Solver Options', 'Linesearch Options'):
                dict_key_to_attrs[line.split(':')[0].strip()].update(parsed_data)
            elif parsed_data is not None:  # For all other data, simple assignment is fine
                dict_key_to_attrs[line.split(':')[0].strip()] = parsed_data
            parsed_data = None
    except ArbitraryVariableNumberingException:
        error_label.config(text='Incorrect numbering of variables.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except ArbitraryVariableNamingException:
        error_label.config(text='Incorrect naming of variables.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except NonnumbericValuesFoundException:
        error_label.config(text='Numerical values expected.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except MultipleValuesFoundException:
        error_label.config(text='Single value expected.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except NotPositiveRealNumberException:
        error_label.config(text='IPositive real number expected.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except IncorrectSyntaxException:
        error_label.config(text='Incorrect syntax.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except IncorrectMatrixBoundaryException:
        error_label.config(text='Incorrect matrix syntax.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except IncorrectVectorBoundaryException:
        error_label.config(text='Incorrect vector syntax.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except NonnumericValueInMatrixException:
        error_label.config(text='Non-numeric values found in matrix.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except NonnumericValueInVectorException:
        error_label.config(text='Non-numeric values found in vector.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except IncorrectMatrixDefinitionException:
        error_label.config(text='Incorrect matrix shape.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except IncorrectSymbolicExpressionException:
        error_label.config(text='Incorrect symbolic expression.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except IncorrectNumOfSymbolicExpressionException:
        error_label.config(text='Incorrect number of symbolic expressions found.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except MatricesUnequalRowsException:
        error_label.config(text='Matrices should have same number of rows.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except MultipleMatricesSpecifiedException:
        error_label.config(text='Only one matrix expected.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except MultipleScalarsSpecifiedException:
        error_label.config(text='Only one scalar expected.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except MatrixNotQuadraticException:
        error_label.config(text='Quadratic matrix expected.')
        v.config(highlightbackground="RED")
        parsing_success = False
    except Exception as e:
        error_label.config(text=str(e))
        v.config(highlightbackground="RED")
        parsing_success = False

    if parsing_success and var_system_type.get():   # If system is non-linear
        try:
            # triggering sampling time should be lower than triggering heartbeat
            if dict_key_to_attrs['Triggering Sampling Time'] and \
                    dict_key_to_attrs['Triggering Sampling Time'] < dict_key_to_attrs['Triggering Sampling Time']:
                getattr(layout_linear, 'triggering_heartbeat_entry').config(highlightbackground="RED")
                getattr(layout_linear, 'triggering_samplingtime_entry').config(highlightbackground="RED")
                # error_label.config(text='Triggering sampling time should be lower than triggering heartbeat')
                raise GenericParsingException('Triggering sampling time should be lower than triggering heartbeat')

            is_PETC = True if (dict_key_to_attrs['Triggering Sampling Time']) else False

            # Destroy the GUI as input data is correct
            root.destroy()
            root.quit()

            # Build plant and controller
            data_obj = ld.InputDataStructureLinear(dynamics=dict_key_to_attrs['Dynamics'],
                                                   controller=dict_key_to_attrs['Controller'],
                                                   triggering_heartbeat=dict_key_to_attrs['Triggering Heartbeat'],
                                                   triggering_condition=dict_key_to_attrs['Triggering Condition'],
                                                   triggering_sampling_time=dict_key_to_attrs['Triggering Sampling Time'],
                                                   lyapunov_func=dict_key_to_attrs['Lyapunov Function'],
                                                   solver_options=dict_key_to_attrs['Solver Options'],
                                                   abstraction_options=dict_key_to_attrs['Abstraction Options'],
                                                   is_PETC=is_PETC)

            # Add the field traffic_model to data_obj, which contains the abstraction.
            linear_logic.create_abstractions(data_obj)
            print(data_obj.traffic_model.__dict__)  # Print fields

            # This builds the priced timed automaton from the traffic model
            pta = linear_logic.traffic2ta(data_obj.traffic_model)
            print(pta)
        except GenericParsingException as e:
            error_label.config(text=str(e))
        except Exception as e:
            print(str(e))
            sys.exit()
    elif parsing_success and (not var_system_type.get()):
        print('here')
        try:
            # Get all the symbols from expressions and assign symbols to correct attr
            all_exprs = str(dict_key_to_attrs['Dynamics']) + str(dict_key_to_attrs['Controller']) + \
                        str(dict_key_to_attrs['Triggering Condition']) + str(dict_key_to_attrs['Lyapunov Function'])
            set_symbols = set(re.findall('[a-z]\\d+', all_exprs))  # Get symbols of the form x0, e0, ...

            # If 'd' variables specified, 'Hyperbox Disturbances' should not be empty
            if re.search('[d]\d+', ''.join(set_symbols)) and not dict_key_to_attrs['Hyperbox Disturbances']:
                # error_label.config(text='If \'d\' variables specified, \'Hyperbox Disturbances\' should not be empty')
                getattr(layout_nonlinear, 'hyperbox_disturbances_entry').config(highlightbackground="RED")
                raise GenericParsingException('If \'d\' variables specified, \'Hyperbox Disturbances\' should not be '
                                              'empty')
            # If 'd' variables specified not specified, 'Hyperbox Disturbances' should be empty
            elif (not re.search('[d]\d+', ''.join(set_symbols)) and dict_key_to_attrs['Hyperbox Disturbances']):
                # error_label.config(text='If \'d\' variables specified not specified, \'Hyperbox Disturbances\' should be empty')
                getattr(layout_nonlinear, 'hyperbox_disturbances_entry').config(highlightbackground="RED")
                raise GenericParsingException('If \'d\' variables specified not specified, \'Hyperbox Disturbances\' '
                                             'should be empty')
            else:
                pass

            # Add the symbol to correct attr in 'dict_symbol_to_attr'
            for symbol in set_symbols:
                dict_symbol_to_attr[symbol[0]].add(symbol)

            # Check that each symbol type numbering begins with '1' and is sequential, e.g. x1, x2, ...
            for key in dict_symbol_to_attr.keys():
                sorted_list = sorted(list(dict_symbol_to_attr[key]))
                i = 1
                for item in sorted_list:
                    if not int(re.split('(\d.*)', item)[1]) == i:
                        print('Incorrect variable numbering')
                        sys.exit()
                    i += 1

            # Number of controller exprs and dynamics inputs have be equal
            if not len(dict_symbol_to_attr['u']) == len(dict_key_to_attrs['Controller']):
                # error_label.config(text='Incorrect number of controller expressions.')
                getattr(layout_nonlinear, 'controller_entry').config(highlightbackground="RED")
                raise GenericParsingException('Incorrect number of controller expressions.')

            # Number of values for 'grid_pts_per_dim' should be equal to num of 'x' variables
            if not dict_key_to_attrs['Solver Options']['grid_pts_per_dim']:  # If not specified, create list of len 'x'
               dict_key_to_attrs['Solver Options']['grid_pts_per_dim'] = [5] * len(dict_symbol_to_attr['x'])
            elif not len(dict_symbol_to_attr['x']) == len(dict_key_to_attrs['Solver Options']['grid_pts_per_dim']):
                # error_label.config(text='grid points per dimension should be equal to number of \'x\' variables!')
                getattr(layout_nonlinear, 'solver_options_entry').config(highlightbackground="RED")
                raise GenericParsingException('grid points per dimension should be equal to number of \'x\' variables!')
            else:
                pass

            # Number of values for 'nr_cones_small_angles' should be equal to num of 'x' variables
            if not dict_key_to_attrs['Solver Options']['nr_cones_small_angles']:  # If not specified, create list of len 'x'
                dict_key_to_attrs['Solver Options']['nr_cones_small_angles'] = [4] * (len(dict_symbol_to_attr['x']) - 2)
            elif not (len(dict_symbol_to_attr['x']) - 2) == \
                     len(dict_key_to_attrs['Solver Options']['nr_cones_small_angles']):
                # error_label.config(text='nr_cones_small_angles should be equal to number of \'x\' variables!')
                getattr(layout_nonlinear, 'solver_options_entry').config(highlightbackground="RED")
                raise GenericParsingException('nr_cones_small_angles should be equal to number of \'x\' variables!')
            else:
                pass

            # Generate etc_controller data from controller data
            dynamics_errors, etc_controller = nlp.get_etc_controller(dict_key_to_attrs['Controller'])
            dict_symbol_to_attr['e'] = dynamics_errors.union(dynamics_errors)  # Union with existing error symbols

            # Generate new dynamics variable by replacing expr from 'etc_controller'
            dynamics_new = []
            for expr in dict_key_to_attrs['Dynamics']:
                expr = str(expr)
                for sym in re.findall('u\d+', expr):
                    replacement_expr = etc_controller[int(re.search('\d+', sym).group(0)) - 1]
                    expr = str.replace(expr, sym, str(replacement_expr))
                dynamics_new.append(sp.sympify(expr))
            dynamics_new.extend([-1 * expr for expr in dynamics_new])
            dynamics_new = sp.Matrix(dynamics_new)

            # If 'w1' variable is specified, then system is homogenized
            if 'w1' in set_symbols:
                is_homogenized = True
            else:
                is_homogenized = False

            # Initialize default value if homogenized but no triggering times specified
            if is_homogenized and (not dict_key_to_attrs['Triggering Times']):  # (len(dict_key_to_attrs['Triggering Times']) == 0):
                dict_key_to_attrs['Triggering Times'] = [0.0001]
            # Assign only lowest value if homogenized and multiple triggering times specified
            elif is_homogenized and (len(dict_key_to_attrs['Triggering Times']) >= 1):
                dict_key_to_attrs['Triggering Times'] = [min(dict_key_to_attrs['Triggering Times'])]
            # Initialize default values if not homogenized but no triggering times or contains less than two values
            elif not is_homogenized and (not dict_key_to_attrs['Triggering Times'] or
                                        (len(dict_key_to_attrs['Triggering Times']) < 2)):
                dict_key_to_attrs['Triggering Times'] = [0.0001, 0.001]
            # Default case not homogenized and triggering times specified and length >2, just sort the values
            else:
                dict_key_to_attrs['Triggering Times'] = sorted(dict_key_to_attrs['Triggering Times'], key=float)

            # Check if only one time mentioned, then the system is homogeneous
            # is_homogenized = True if (len(dict_key_to_attrs['Triggering Times']) == 1) else False

            # To get parameters, sort the d symbols
            d_str_sorted = sorted([i for i in dict_symbol_to_attr['d']])
            parameters = tuple(sp.Symbol(i) for i in d_str_sorted)
            print(dict_symbol_to_attr)
            # State is a union of sorted x and e symbols
            x_str_sorted = sorted([i for i in dict_symbol_to_attr['x']])

            # Init conditions is tuple to x replaced with a
            a_str_sorted = sorted([i.replace('x', 'a') for i in x_str_sorted])
            a_str_sorted.append('aw') if 'w1' in dict_symbol_to_attr['w'] else print('')
            e_str_sorted = sorted([i.replace('x', 'e') for i in dict_symbol_to_attr['x']])
            e_str_sorted.append('ew') if 'w1' in dict_symbol_to_attr['w'] else print('')  # only append if w1 exists
            init_cond_symbols = tuple(sp.Symbol(i) for i in a_str_sorted)

            x_str_sorted.append(list(dict_symbol_to_attr['w'])[0]) if 'w1' in dict_symbol_to_attr['w'] else print('')
            state_str = x_str_sorted + e_str_sorted
            state = tuple(sp.Symbol(i) for i in state_str)

            # Destroy the GUI as input data is correct
            root.destroy()
            root.quit()

            if not path:
                print('Path to SMT files is not set.')
                sys.exit()
            if not dreal_path:
                print('Path to dreal executable is not set.')
                sys.exit()
            if not dreach_path:
                print('Path to dreach executable is not set.')
                sys.exit()
            if not flowstar_path:
                print('Path to flow* executable is not set.')
                sys.exit()

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
                                                       dict_key_to_attrs['Hyperbox States'],
                                                       dict_key_to_attrs['Solver Options']['p'],
                                                       dict_key_to_attrs['Solver Options']['gridstep'],
                                                       dict_key_to_attrs['Solver Options']['dreal_precision_deltas'],
                                                       dict_key_to_attrs['Solver Options']['heart_beat'],
                                                       dict_key_to_attrs['Triggering Times'],
                                                       dict_key_to_attrs['Solver Options']['remainder_reachability'],
                                                       dict_key_to_attrs['Solver Options']['timeout_reachability'],
                                                       dict_key_to_attrs['Solver Options']['grid_pts_per_dim'],
                                                       dict_key_to_attrs['Linesearch Options']['timeout_upper_bounds'],
                                                       dict_key_to_attrs['Linesearch Options']['remainder_upper_bounds'],
                                                       dict_key_to_attrs['Solver Options']['timeout_deltas'],
                                                       dict_key_to_attrs['Solver Options']['nr_cones_small_angles'],
                                                       dict_key_to_attrs['Solver Options']['nr_cones_big_angle'],
                                                       is_homogenized)
            print(data_obj.__dict__)
            nonlinear_logic.create_abstractions(data_obj)
        except GenericParsingException as e:
            error_label.config(text=str(e))
        except LPOptimizationFailedException:
            print("LP optimization failed, terminating script")
            sys.exit()
        except LPGeneralException as e:
            print(e.msg)
            sys.exit()
        except DataObjectGeneralException as e:
            print(e.msg)
            sys.exit()
        except Exception as e:
            print(str(e))
            sys.exit()
    else:
        print('else')
        pass



def grids_label_and_entries(layout_type):
    """
        Function prints the layout for the specifies system type specified.

        Parameters:
        ----------
            layout_type: the type of the system of which the layout to be drawn.
    """
    row = 0
    for k, v in layout_type.__dict__.items():
        if 'label' in v._name:
            getattr(layout_type, k).grid(row=row, column=0, sticky=E)
        else:
            getattr(layout_type, k).grid(row=row, column=1, sticky=E)
            row = row + 1


def set_entries_border_black(linear_or_nonlinear):
    """
        Function makes all the entry boxes in the layout black. This helps to erase any red marked boxes in case of
        incorrect data entered by the user.

        Parameters:
        ----------
            linear_or_nonlinear: Specifies if system is linear or non_linear
    """
    if linear_or_nonlinear:  # Linear system layout
        for k, v in layout_linear.__dict__.items():
            v.config(highlightbackground="WHITE")
    else:
        for k, v in layout_nonlinear.__dict__.items():
            v.config(highlightbackground="WHITE")


# Initialize window parameters
root = Tk()
root.title('etc2pta')
root.geometry("500x300")
root.grid_rowconfigure(0, weight=1)    
root.grid_columnconfigure(0, weight=1) 

# Initialize a frame to hold radio buttons to select linear or non-linear system
frame_linear_or_nonlinear = Frame(root, width=200, height=50)
frame_linear_or_nonlinear.grid(row=0, column=0)
var_system_type = BooleanVar(root)     # Variable to hold choice of linear or non-linear
rb_linear = Radiobutton(frame_linear_or_nonlinear, text='Linear',
                     command=toggle_linear_or_nonlinear, variable=var_system_type, value=True)
rb_nonlinear = Radiobutton(frame_linear_or_nonlinear, text='Non-linear',
                     command=toggle_linear_or_nonlinear, variable=var_system_type, value=False)
var_system_type.set(1)      # Set default as linear system
rb_linear.grid(row=0, column=0, sticky=W)
rb_nonlinear.grid(row=0, column=1, sticky=E)

# Initialize a frame to hold input entries to collect data
frame_input = Frame(root, width=200, height=150)
frame_input.grid(row=1, column=0)
layout_linear = LinearSystemLayout(frame_input)     # Initialize linear system layout of labels and entries
layout_nonlinear = NonlinearSystemLayout(frame_input)   # # Initialize non-linear system layout of labels and entries
grids_label_and_entries(layout_linear)      # Plot default layout, i.e. linear system

# Initialize a frame to text widget to show error message
frame_error_text = Frame(root, width=200, height=100)
frame_error_text.grid(row=2, column=0)
error_label = Label(frame_error_text, text='')
error_label.config(fg="red")
error_label.grid()

# Initialize a frame to hold button to submit data
frame_complete_button = Frame(root, width=200, height=100)
frame_complete_button.grid(row=3, column=0)
button_submit = Button(frame_complete_button, text='Submit', command=submit_data)
button_submit.pack(side=BOTTOM)

# Start the loop
root.mainloop()
