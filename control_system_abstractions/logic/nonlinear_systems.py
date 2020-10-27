import control_system_abstractions.data.nonlinear_systems_datastructure as data
import sympy
import control_system_abstractions.nonlinear_systems_utils.dReal_communication_2 as dReal
import control_system_abstractions.nonlinear_systems_utils.dReach_communication as dReach
import control_system_abstractions.nonlinear_systems_utils.flowstar_communication as flowstar
from control_system_abstractions.exceptions.nonlinear_systems_exceptions.data_object_exceptions import \
    DataObjectGeneralException
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
            raise DataObjectGeneralException('Could not create box domain for initial conditions')
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
        res_LP = LP_data.LP_solve(lp_method, Verbose=verbose)  # first solve the LP
        LP_data.solutions.append(res_LP)
        data_obj.deltas = LP_data.solutions[-1][:-1]  # these are the solutions found by the LP
        data_obj.gamma = LP_data.solutions[-1][-1]
        print("Delta's: {}".format(data_obj.deltas))
        print("Infinity norm: {}".format(data_obj.gamma))
        # Construct the UBF given the set of obtained delta's
        ubf_temp = data_obj.construct_UBF()
        data_obj.UBF = ubf_temp
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
            LP_data.A, LP_data.B = LP_data.calculate_constraints(data_obj, res['violation'], dreal_precision_upper_bound)

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

    #self.manifolds_times = manifolds_times
    if (data_obj.Homogenization_Flag == False):
        print('Computing radii for one manifold')
    #    data_obj.spherical_sectors(manifolds_times[len(manifolds_times) - 1], nr_cones_small_angles, nr_cones_big_angle,
    #                           verbose)
    #    print('Finished Computing the radii')
    #    print('Creating the Regions and their overapproximations')
    #    data_obj.create_regions_objects(manifolds_times)
    #    print('Moving on to timing upper bounds')
    #    data_obj.timing_upper_bounds(len(manifolds_times), time_out_upper_bounds, remainder_upper_bounds, heartbeat,
    #                             verbose=False)
    #    data_obj.transitions(verbose, time_out_reach, remainder_reach)
    else:
        # Create a grid object
        data_obj.grid = data_obj.create_grid(data_obj.hyperbox_states, data_obj.grid_points_per_dim)

        """INPUTS:
            manifolds_times: list of int, >0 (practically I only use manifolds_times[-1])

        Creates the objects Region_NonHomogeneous (for homogeneous systems), using
        the manfifold of time=manifolds_times[-1] and the created grid (by self.create_grid).
        """
        manifold_time = data_obj.manifolds_times[-1]
        dimension = int(data_obj.n / 2) - 1
        polytope_sides_lengths = []
        for i in range(0, dimension):  # for each dimension
            lim_min = data_obj.grid.State_space_limits[i][0]  # calculate what is the displacement
            lim_max = data_obj.grid.State_space_limits[i][1]  # from the center of a grid polytope
            side_length = (lim_max - lim_min) / data_obj.grid.Grid_points_per_dim[i]  # to its vertices
            polytope_sides_lengths.append([-side_length / 2, side_length / 2])
        # create a list with all combinations of displacement. each combination, when added to the center of
        # a polytope, it gives one of its vertices.
        differences_between_all_vertices_and_centroid = list(itertools.product(*polytope_sides_lengths))
        region_index = 0
        data_obj.regions = []
        for centroid in data_obj.grid.Centroids:  # iterate along all polytopes, each of which representing a region
            region_index = region_index + 1
            polytope_vertices_in_rn = []
            # add each element of differences_between_all_vertices_and_centroid to the center of the region
            # to get its vertices
            for i in range(0, len(differences_between_all_vertices_and_centroid)):
                aux = np.array(centroid) + np.array(differences_between_all_vertices_and_centroid[i])
                aux = aux.tolist()
                polytope_vertices_in_rn.append(aux)
            [halfspaces_b, halfspaces_A] = polytope_vrep2hrep(polytope_vertices_in_rn)  # get the hrep
            #            aux2 = np.zeros(dimension)
            #            polytope_vertices.append(aux2.tolist())
            if all([v == 0 for v in centroid]) & data_obj.origin_neighbourhood_degeneracy_flag:
                # if the polytope contains the origin and the manifolds there are degenerate
                lower_bound = data_obj.timing_lower_bound_origin_polytope(data_obj.heartbeat)
                temp = RegionNonHomogeneous(data_obj.state[:dimension], region_index, centroid, polytope_vertices_in_rn,
                                             halfspaces_A, halfspaces_b, lower_bound, True)
            else:
                lower_bound = data_obj.timing_lower_bounds_grid(polytope_vertices_in_rn, manifold_time)
                temp = RegionNonHomogeneous(data_obj.state[:dimension], region_index, centroid, polytope_vertices_in_rn,
                                             halfspaces_A, halfspaces_b, lower_bound, False)
            print('Region {} timing lower bound = {}'.format(region_index, lower_bound))
            data_obj.regions.append(temp)
        #data_obj.create_regions_objects_nonhomogeneous(manifolds_times, heartbeat)

        for region in data_obj.regions:
            #                if all([v == 0 for v in region.center]): #for the region of the origin
            #                    region.insert_timing_upper_bound(heartbeat)
            #                else: #for all other regions use reashability
            upper_bound = data_obj.reach_analysis_upper_bounds(region, verbose, data_obj.time_out_upper_bounds,
                                                               data_obj.remainder_upper_bounds, data_obj.heartbeat)
            region.insert_timing_upper_bound(upper_bound)
        #data_obj.timing_upper_bounds(None, data_obj.time_out_upper_bounds, data_obj.remainder_upper_bounds,  data_obj.heartbeat, verbose)

        data_obj.transitions(False, data_obj.time_out_reachability, data_obj.remainder_reachability)
