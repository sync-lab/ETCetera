import control_system_abstractions.data.nonlinear_systems_datastructure as data
import sympy
import control_system_abstractions.nonlinear_systems_utils.dReal_communication_2 as dReal
import control_system_abstractions.nonlinear_systems_utils.dReach_communication as dReach
import control_system_abstractions.nonlinear_systems_utils.flowstar_communication as flowstar
from control_system_abstractions.nonlinear_systems_utils.auxilliary_data import LPData


def create_abstractions(data_obj):
    """ If the system has not been homogenized, it computes an inner box of the lyapunov level set provided.
    Otherwise, it dictates a box arbitrarily.
    This will serve as the domain of the initial conditions for the feasibility problem."""
    # create_box_domain_for_init_cond
    print(data_obj.state)

    if data_obj.homogenization_flag:  # if the system has been homogenized
        domain_init_cond = []  # the list containing interval domains for each variable, i.e. the box.
        for i in range(0, int(data_obj.n)):  # iterate along all dimensions (each dimension corresponds to one variable)
            # for each dimension (i.e. for each variable) append a corresponding interval
            if i == int(data_obj.n / 2 - 1):
                domain_init_cond.append([0, 0.1])  # this is the interval domain for the w variable (i.e. self.State(int(self.n/2-1)))
            elif i == data_obj.n - 1:
                domain_init_cond.append([0, 0])  # this is the interval domain for the ew variable (i.e. self.State(int(self.n-1)))
            else:
                domain_init_cond.append([-0.1, 0.1])  # this is the interval domain for all other variabes
        data_obj.init_cond_domain = domain_init_cond  # I am still experimenting with these values

    else:  # if the system has not been homogenized, compute an inner box of the lyapunov level set
        print('Computing box contained in the lyapunov level set')
        time_out = 1000
        guess = 100 * data_obj.lyapunov_lvl_set_c  # initial guess for the box, i.e. box = [-guess,guess]
        expression = data_obj.lyapunov_function < data_obj.lyapunov_lvl_set_c  # the set contained in the lyapunov level set
        sym_dom = sympy.And()
        sym_list = list(data_obj.state)[:int(len(data_obj.state) / 2)]
        print(sym_list)
        for var in range(0, len(sym_list)):  # write the box domain in a symbolic expression
            print(sym_list[var])
            sym_dom = sympy.And(sym_dom, sympy.And(sym_list[var] >= -guess, sym_list[var] <= guess))

        flag = True
        # check if the box [-gues,guess] is contained in set expression, by doing a line search on the var guess
        while flag:
            res = dReal.dReal_verify(expression, sym_dom, None, data_obj.state[:int(len(data_obj.state) / 2)],
                                     data_obj.dreal_precision, data_obj.dreal_path, data_obj.path, time_out=time_out,
                                     verbose=False)  # check if the box is contained in the set expression
            if res['time-out']:  # if it is contained solution is found
                break
            if not res['sat']:  # else decrease guess and iterate again
                guess = guess / 1.1
                sym_dom = sympy.And()
                for var in range(0, len(sym_list)):
                    sym_dom = sympy.And(sym_dom, sympy.And(sym_list[var] >= -guess, sym_list[var] <= guess))
            else:
                flag = False

        if res['time-out']:  # if dreal timed out terminate
            print('Could not create box domain for initial conditions')
            # -------to be checked with Giannis--------------
            return -1
        else:  # else return the list [[-guess,guess], [-guess,guess],...] as the box domain for initial conditions
            domain_init_cond = []
            for var in range(0, data_obj.n):
                domain_init_cond.append([-guess, guess])
            print('Box of initial conditions:{}'.format(domain_init_cond))
            data_obj.init_cond_domain = domain_init_cond

    data_obj.symbolic_box_of_initial_conditions = data_obj.create_symbolic_domain_init_cond()

    data_obj.symbolic_domain_of_parameters = data_obj.create_symbolic_domain_of_parameters()

    data_obj.lie = data_obj.calculate_lie_derivatives()

    data_obj.samples = data_obj.discretize_init_cond_and_param_domain()

    data_obj.lie_n = data_obj.lie[-1]


    """ Solves the feasibility problem, by solving iteratively the LP version of it and checking with dReal
    if the found solutions verify the constraints.

    optional argument:
        dreal_precision (rational): Precision used in dReal. Default = 0.01
        time_out: maximum time in seconds, after which the verification with dReal is canceled. Default = None
        lp-method: The LP method 'revised simplex' or 'interior-point'. Default =' revisedsimplex'
        C (numpy array): Cost function vector c of the LP problem
        D (tuple of tuples): Bounds on the delta's
        verbose (Boolean): Print LP solver information. Default = False

    """
    # Initialize parameters to calculate upper bound
    dreal_precision_upper_bound = 0.01
    time_out_upper_bound = None
    lp_method = 'revised simplex'
    C_mat_lp = []
    D_mat_lp = []
    verbose = False
    LP_data = LPData(data_obj, C_mat_lp, D_mat_lp)  # Initialize LP data from the user specified data
    res = {'sat': False}  # when the solution is found we set res['sat'] = true
    res_flag = 0
    res_flag_final = 0
    iteration = 1
    while not res['sat']:  # iterate until the solution is found
        print("\n Starting iteration {}".format(iteration))
        res_flag = LP_data.LP_solve(lp_method, Verbose=verbose)  # first solve the LP
        #-------to be checked with Giannis--------------
        #LP_data.solutions.append(res_flag)
        if res_flag == -1:
            break
        data_obj.deltas = LP_data.solutions[-1][:-1]  # these are the solutions found by the LP
        data_obj.gamma = LP_data.solutions[-1][-1]
        print("Delta's: {}".format(data_obj.deltas))
        print("Infinity norm: {}".format(data_obj.gamma))
        # Construct the UBF given the set of obtained delta's
        data_obj.construct_UBF()
        # Verify the condition using dReal
        res = data_obj.verify_upper_bound_constraint(dreal_precision_upper_bound,
                                                 time_out=time_out_upper_bound)  # check if the found solutions verify the constraints
        if res['time-out']:
            print(
                "WARNING: Verification timed-out or other unexpected output. Please modify the time-out variable or adapt the specification")
            res_flag = -2
            break

        if not res['sat']:  # if they dont verify the constraints append a new constraint employing the counterexample res['violation']
            # Set the value to lp-data
            LP_data.calculate_constraints(data_obj, res['violation'], dreal_precision_upper_bound)

        if (LP_data.A[-1] == LP_data.A[-3]) and (LP_data.B[-1] == LP_data.B[-3]):
            # if the same counterexample as before is returned, then terminate
            print('ERROR: Same counterexample found by dReal. Terminating script.')
            res_flag = -3
            break

        if res['sat']:  # if the solutions verify the constraint change the flag res_flag_final
            res_flag_final = 1

        iteration += 1

    if res_flag_final > 0:
        print('Valid bound found!')
        print('The deltas are:{}'.format(LP_data.solutions[-1][:-1]))
        return 1
    else:
        print("No solution has been found. Try different LP or dReal settings")
        return -1


    """INPUTS:
        manifolds_times: list of float, >0. (for homogeneous systems, see self.spherical_sectors.
                                             for nonhomogeneous, see self.create_regions_objects_nonhomogeneous.)
        nr_cones_small_angles: list of int, >0. (see self.spherical_sectors)
        nr_cones_big_angle: int, >0. (see self.spherical_sectors)
        state_space_limits: list of symmetric intervals (e.g. [[-1,1],[-2,2],[-1,1]]) (see self.create_grid)
        grid_points_per_dim: list of odd int (e.g. [3,5,7]) (see self.create_grid)
        heartbeat: float, >0 (the ad-hoc upper bound in timing upper bounds,
                              i.e. if can't find an upper bound for a region, use heartbeat)
        time_out: int, >0 (time after which, flowstar/dreach/dreal process is forcefully killed.)
        verbose: boolean. (if True print, if False don't print.)

        Creates the Abstraction.
        For homogeneous systems, it uses the arguments manifolds_times, nr_cones_small_angles, nr_cones_big_angle,
            heartbeat, time_out, verbose.
        For non-homogeneous systems, it uses state_space_limits, grid_points_per_dim, heartbeat, time_out, verbose.
    """
    manifolds_times = [0.0001, 0.0001]
    nr_cones_small_angles = []
    nr_cones_big_angle = 0
    state_space_limits = [[-1, 1], [-1, 1]]
    grid_points_per_dim = [7, 7]
    heartbeat=0.04
    time_out_upper_bounds = 8
    time_out_reach = 5
    remainder_upper_bounds = 1e-3
    remainder_reach = 1e-2
    verbose = False

    data_obj.manifolds_times = manifolds_times
    #if (data_obj.Homogenization_Flag == False):
    #    print('Computing radii for one manifold')
    #    data_obj.spherical_sectors(manifolds_times[len(manifolds_times) - 1], nr_cones_small_angles, nr_cones_big_angle,
    #                           verbose)
    #    print('Finished Computing the radii')
    #    print('Creating the Regions and their overapproximations')
    #    data_obj.create_regions_objects(manifolds_times)
    #    print('Moving on to timing upper bounds')
    #    data_obj.timing_upper_bounds(len(manifolds_times), time_out_upper_bounds, remainder_upper_bounds, heartbeat,
    #                             verbose=False)
    #    data_obj.transitions(verbose, time_out_reach, remainder_reach)
    #else:
    #    data_obj.create_grid(state_space_limits, grid_points_per_dim)
    #    data_obj.create_regions_objects_nonhomogeneous(manifolds_times, heartbeat)
    #    data_obj.timing_upper_bounds(None, time_out_upper_bounds, remainder_upper_bounds, heartbeat, verbose)
    #    data_obj.transitions(False, time_out_reach, remainder_reach)
