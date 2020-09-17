import random
from tkinter import *
import control_system_abstractions.parser.parser_linear_systems as lp
import control_system_abstractions.parser.parser_nonlinear_systems as nlp
import control_system_abstractions.data.nonlinear_systems_datastructure as dat
import sympy as sp
#import control_system_abstractions.nonlinear_systems_utils as nonlinear
import control_system_abstractions.data.nonlinear_systems_datastructure as nld


# Linear system layout definitions
class LinearSystemLayout:

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
    if var_system_type.get():  # Linear system layout
        for k, v in layout_nonlinear.__dict__.items():
            getattr(layout_nonlinear, k).grid_forget()
        grids_label_and_entries(layout_linear)
    else:               # Non-linear system layout
        for k, v in layout_linear.__dict__.items():
            getattr(layout_linear, k).grid_forget()
        grids_label_and_entries(layout_nonlinear)


def submit_data():
    try:
        if var_system_type.get():  # Linear system data submitted
            layout_current = layout_linear
            # Dictionary to hold linear data
            dict_key_to_attrs = {'Dynamics': None, 'Controller': None, 'Triggering Heartbeat': None,
                                 'Triggering Condition': None, 'Triggering Sampling Time': None,
                                 'Lyapunov Function': None,
                                 'Solver Options': {}, 'Abstraction Options': {}}
        else:               # Non-linear system data submitted
            layout_current = layout_nonlinear
            # Dictionary to hold non-linear data
            dict_key_to_attrs = {'Dynamics': None, 'Controller': None, 'Hyperbox States': None,
                                 'Triggering Condition': None, 'Triggering Times': None, 'Hyperbox Disturbances': None,
                                 'Solver Options': {'p': 1, 'opt_method': None, 'gridstep': 2},
                                 'Linesearch Options': {}, 'Lyapunov Function': None, 'Deg. of Homogeneity': None}
            # Dictionary to hold symbols in expressions
            dict_symbol_to_attr = {'e': set(), 'u': set(), 'd': set(), 'x': set()}

        set_entries_border_black(var_system_type.get())    # Set the border to default white

        parsed_data = None
        for k, v in layout_current.__dict__.items():    # Iterate over labels and entries
            if 'label' in v._name:      # If label, get the text
                line = v.cget('text')    # Append the text to line
            elif 'entry' in v._name and var_system_type.get():    # If entry and linear system
                parsed_data = lp.parse_linear(line + ' : ' + v.get())
            else:           # If entry and non-linear system
                parsed_data = nlp.parse_nonlinear(line + ' : ' + v.get())

            # If data is 'Solver Options', merge with default already defined
            if parsed_data and line in ('Solver Options', 'Linesearch Options'):
                dict_key_to_attrs[line.split(':')[0].strip()].update(parsed_data)
            elif parsed_data:  # For all other data, simple assignment is fine
                dict_key_to_attrs[line.split(':')[0].strip()] = parsed_data
            parsed_data = None

        if var_system_type.get():   # If system is non-linear, do fin
            # triggering sampling time should be lower than triggering heartbeat
            if dict_key_to_attrs['Triggering Sampling Time'] and \
                    dict_key_to_attrs['Triggering Sampling Time'] < dict_key_to_attrs['Triggering Sampling Time']:
                raise Exception('Triggering sampling time should be lower than triggering heartbeat')

            is_PETC = True if (dict_key_to_attrs['Triggering Sampling Time']) else False
        else:
            # Get all the symbols from expressions and assign symbols to correct attr
            all_exprs = str(dict_key_to_attrs['Dynamics']) + str(dict_key_to_attrs['Controller']) + \
                      str(dict_key_to_attrs['Triggering Condition']) + str(dict_key_to_attrs['Lyapunov Function'])
            set_symbols = set(re.findall('[a-z]\\d+', all_exprs))       # Get symbols of the form x0, e0, ...
            for symbol in set_symbols:
                dict_symbol_to_attr[symbol[0]].add(sp.Symbol(symbol))   # Add the symbol to correct attr

            # Number of controller exprs and dynamics inputs have be equal
            if not len(dict_symbol_to_attr['u']) == len(dict_key_to_attrs['Controller']):
                raise Exception('Incorrect number of controller expressions!')

            # Generate etc_controller data from controller data
            dynamics_errors, etc_controller = nlp.get_etc_controller(dict_key_to_attrs['Controller'])
            dict_symbol_to_attr['e'] = dynamics_errors.union(dynamics_errors)      # Union with existing error symbols

            dynamics_new = []
            for expr in dict_key_to_attrs['Dynamics']:
                expr = str(expr)
                for sym in re.findall('u\d+', expr):
                    repl = etc_controller[int(re.search('\d+', sym).group(0))]
                    expr = str.replace(expr, sym, str(repl))
                dynamics_new.append(sp.sympify(expr))
            dynamics_new.extend([-1 * expr for expr in dynamics_new])

            # Check if only one time mentioned, then the system is homogeneous
            is_homogenized = False if (len(dict_key_to_attrs['Triggering Times']) == 1) else True

            init_cond_symbols = tuple(map(lambda st: sp.Symbol(str.replace(str(st), "x", "a")), dict_symbol_to_attr['x']))
            d_str_sorted = sorted([str(i) for i in dict_symbol_to_attr['d']])
            parameters = tuple(sp.Symbol(i) for i in d_str_sorted)

            path, dreal_path, dreach_path, flowstar_path = None, None, None, None
            data_obj = nld.InputDataStructureNonLinear(path, dreal_path, dreach_path, flowstar_path,
                                                       dynamics_new,
                                                       dict_key_to_attrs['Deg. of Homogeneity'],
                                                       dict_key_to_attrs['Lyapunov Function'],
                                                       random.uniform(0.01, 0.1),
                                                       dict_key_to_attrs['Triggering Condition'],
                                                       (dict_symbol_to_attr['x'], dict_symbol_to_attr['e']),
                                                       init_cond_symbols,
                                                       parameters,
                                                       dict_key_to_attrs['Hyperbox Disturbances'],
                                                       dict_key_to_attrs['Solver Options']['p'],
                                                       dict_key_to_attrs['Solver Options']['gridstep'],
                                                       is_homogenized)
            root.destroy()
            root.quit()
            print(data_obj.__dict__)
            #nonlinear_logic.create_abstractions(data_obj)
    except Exception as e:
        error_label.config(text=str(e)+'!!')
        if str(e) == 'Incorrect number of controller expressions!':
            getattr(layout_nonlinear, 'controller_entry').config(highlightbackground="RED")
        elif str(e) == 'Triggering sampling time should be lower than triggering heartbeat!':
            getattr(layout_linear, 'triggering_heartbeat_entry').config(highlightbackground="RED")
            getattr(layout_linear, 'triggering_samplingtime_entry').config(highlightbackground="RED")
        else:
            v.config(highlightbackground="RED")


def grids_label_and_entries(layout_type):
    row = 0
    for k, v in layout_type.__dict__.items():
        if 'label' in v._name:
            getattr(layout_type, k).grid(row=row, column=0, sticky=E)
        else:
            getattr(layout_type, k).grid(row=row, column=1, sticky=E)
            row = row + 1


def set_entries_border_black(linear_or_nonlinear):
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
