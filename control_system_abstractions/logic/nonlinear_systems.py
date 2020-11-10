import math

import control_system_abstractions.data.nonlinear_systems_datastructure as data
import sympy
import numpy as np
import itertools
import control_system_abstractions.nonlinear_systems_utils.dReal_communication_2 as dReal
import control_system_abstractions.nonlinear_systems_utils.dReach_communication as dReach
import control_system_abstractions.nonlinear_systems_utils.flowstar_communication as flowstar
from control_system_abstractions.exceptions.nonlinear_systems_exceptions.data_object_exceptions import \
    DataObjectGeneralException
from control_system_abstractions.nonlinear_systems_utils.auxilliary_data import LPData, polytope_vrep2hrep, \
    RegionNonHomogeneous, SphericalDomain, ConeOnPlane


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
                                     data_obj.dreal_precision_deltas, data_obj.dreal_path, data_obj.path, time_out=time_out,
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
    #dreal_precision_upper_bound = 0.01
    #time_out_upper_bound = None
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

        res = data_obj.verify_upper_bound_constraint(data_obj.dreal_precision_deltas,
                                                 time_out=data_obj.timeout_deltas)  # check if the found solutions verify the constraints
        if res['time-out']:
            print(
                "WARNING: Verification timed-out or other unexpected output. Please modify the time-out variable or adapt the specification")
            res_flag = -2
            res_flag_final = 1  # Program continues with warnings
            break

        if not res['sat']:  # if they dont verify the constraints append a new constraint employing the counterexample res['violation']
            # Set the value to lp-data
            An, Bn = LP_data.calculate_constraints(data_obj, res['violation'], data_obj.dreal_precision_deltas)
            tempA = An[:]
            tempB = Bn[:]
            LP_data.A.append(tempA[0][:])
            LP_data.A.append(tempA[1][:])
            LP_data.A.append(tempA[2][:])
            LP_data.B.append(tempB[0])
            LP_data.B.append(tempB[1])
            LP_data.B.append(tempB[2])
        if (not (np.array(LP_data.A[-1]) - np.array(LP_data.A[-3])).all()) and (not (np.array(LP_data.B[-1]) - np.array(LP_data.B[-3])).all()):
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
        #return 1
    else:
        raise DataObjectGeneralException("No solution has been found. Try different LP or dReal settings")
        #return -1

    data_obj.LP_data = LP_data
    """INPUTS:
        manifolds_times: list of float, >0. (for homogeneous systems, see self.spherical_sectors.
                                             for nonhomogeneous, see self.create_regions_objects_nonhomogeneous.)
        nr_cones_small_angles: list of int, >0. (see self.spherical_sectors)
        nr_cones_big_angle: int, >0. (see self.spherical_sectors)
        state_space_limits: list of symmetric intervals (e.g. [[-1,1],[-2,2],[-1,1]]) (see self.create_grid)
        grid_pts_per_dim: list of odd int (e.g. [3,5,7]) (see self.create_grid)
        heartbeat: float, >0 (the ad-hoc upper bound in timing upper bounds,
                              i.e. if can't find an upper bound for a region, use heartbeat)
        time_out: int, >0 (time after which, flowstar/dreach/dreal process is forcefully killed.)
        verbose: boolean. (if True print, if False don't print.)

        Creates the Abstraction.
        For homogeneous systems, it uses the arguments manifolds_times, nr_cones_small_angles, nr_cones_big_angle,
            heartbeat, time_out, verbose.
        For non-homogeneous systems, it uses state_space_limits, grid_pts_per_dim, heartbeat, time_out, verbose.
    """
    manifolds_times = [0.0001, 0.0001]
    #nr_cones_small_angles = []
    #nr_cones_big_angle = 0
    state_space_limits = [[-1, 1], [-1, 1]]
    grid_pts_per_dim = [7, 7]
    heartbeat=0.04
    time_out_upper_bounds = 8
    time_out_reach = 5
    remainder_upper_bounds = 1e-3
    remainder_reach = 1e-2
    verbose = False

    #self.manifolds_times = manifolds_times
    if not data_obj.homogenization_flag:
        print('Computing radii for one manifold')
        """INPUT:
            manifold_time: float >0
            nr_cones_small_angles: list of positive int, of length self.n-2
            nr_cones_big_angle: int, >0

        First, creates a conic partition of the state space into isotropic cones, by calling
        the function self.cone_matrices.

        Then, approximates the partitions defined by the intersections of the isochronous manifold
        of time = manifold_time with the previously defined cones,
        by spherical sectors. Firstly two balls are computed: small_ball which inner-approximates a manifold,
        and big_ball which outer-approximates a manifold. Then, for each conic partition the radii of the
        balls are optimized, such that the spherical sectors defined by the radii and the cones
        better inner- and outer-approximate the conic partitions.

        Finally creates the corresponding Spherical_Domain objects.
        It is called by the function self.abstraction().
        """
        manifold_time = data_obj.manifolds_times[len(data_obj.manifolds_times) - 1]
        state_vector = data_obj.state[:int(data_obj.n / 2)]
        dimension = data_obj.n

        #data_obj.cone_matrices(data_obj.nr_cones_small_angles, data_obj.nr_cones_big_angle)  # partition each angular coordinate into
        """Given the number of cones corresponding to the small angles
        of the coordinate system (nr_cones_small_angle list of int)
        and the number of cones for the big angle (nr_cones_big_angle int),
        it discretizes each angular coordinate into a conic partition, creating a list of Cone_on_Plane
        objects (i.e. planar cones) for the big angle (i.e. self.cones_big_angle)
        and a list of lists containing Cone_on_Plane objects for each small angle (i.e. self.cones_small_angles)."""
        # Isotropic covering
        big_angle_discr = np.linspace(0, math.pi, int(data_obj.nr_cones_big_angle / 2 + 1))  # Discretize the big angle
                                                                                             # from 0 to pi.
        small_angles_discr = []
        for i in range(1, int(dimension / 2 - 1)):  # there are int(dimension/2-1) small angles for a coordinate system
                                                    # of dim = dimension/2.
            # discretize each small angle from 0 to pi/2
            small_angles_discr.append(np.linspace(0, math.pi / 2, int(data_obj.nr_cones_small_angles[i - 1] / 2 + 1)))

        data_obj.cones_big_angle = []
        for i in range(0, len(big_angle_discr) - 1):  # iterate along all discrete angles big angle was discretized in
            a = big_angle_discr[i]
            b = big_angle_discr[i + 1]
            data_obj.cones_big_angle.append(ConeOnPlane(a, b, True))  # create the planar cone from angle a to angle b
        for i in range(0, len(big_angle_discr) - 1):  # iterate along all discrete angles big angle was discretized in
            a = big_angle_discr[i]
            b = big_angle_discr[i + 1]
            data_obj.cones_big_angle.append(ConeOnPlane(a, b, False))  # create planar cone from angle pi+a to pi+b

        if (dimension / 2 > 2):  # in this case there are small angles as well (in case dimension = 2, there are not)
            data_obj.cones_small_angles = []
            for i in range(0, int(dimension / 2) - 2):  # for each small angle
                temp = []
                for j in range(0, len(small_angles_discr[
                                          i]) - 1):  # iterate along all discrete angles in which the i-th small angle was discretized
                    a = small_angles_discr[i][j]
                    b = small_angles_discr[i][j + 1]
                    temp.append(ConeOnPlane(a, b, True))  # create the planar cone from angle a to angle b
                for j in range(0, len(small_angles_discr[
                                          i]) - 1):  # iterate along all discrete angles in which the i-th small angle was discretized
                    a = small_angles_discr[i][j]
                    b = small_angles_discr[i][j + 1]
                    temp.append(ConeOnPlane(a, b, False))  # create the planar cone from angle pi+a to angle pi+b
                data_obj.cones_small_angles.append(temp)
        #-----------------------------------

        # discrete angles that define planar cones
        # create the n-dimensional conic partition by taking combinations of planar cones from each angular coordinate
        if (dimension / 2 > 2):
            temp = data_obj.cones_small_angles[
                   :]  # create a list containing the lists of planar cones for each angular coordinate
            temp.insert(0, data_obj.cones_big_angle)
            # combine the planar cones from each coordinate to derive n-dimensional cones
            all_combinations_of_cone_matrices = list(itertools.product(*temp))
        else:  # if dimension/2 = 2, then there is only one angular coordinate, the big angle.
            data_obj.cones_small_angles = [[()]]  # the list of cones for each small angle is then empty
            temp = data_obj.cones_small_angles[:]
            temp.insert(0, data_obj.cones_big_angle)
            all_combinations_of_cone_matrices = list(itertools.product(*temp))

        # Calculate a radius that inner/outer approxiamtes the whole manifold (in all conic sections)
        # THis is done only if the system is not homogenized.
        # This will be the starting radius for the line search for each conic section.
        if not data_obj.homogenization_flag:    #(self.homogenization_flag == False):
            temp = 0
            for i in range(0, int(dimension / 2)):
                temp = temp + state_vector[i] ** 2  # temp = x^2+y^2+z^2+...
            small_ball = 20  # start from a big radius and then decrease it iteratively until it inner-approximates
            # the manifold
            while True:  # iterate until small_ball inner-approximates the manifold
                expression2 = (temp - (0.99 * small_ball) ** 2 >= 0) & \
                              (temp - (1.01 * small_ball) ** 2 <= 0)  # this is the expression that defines the ball
                # check if small_ball inner-approximates the manifold
                res = data_obj.check_radius('i', manifold_time, expression2, small_ball, 0.001, 100, verbose=verbose)
                if res == 1:  # if it does, then proceed
                    print("Inner ball radius = %f" % (small_ball))
                    break
                elif res == -1:  # if it doesn't, decrease and iterate again
                    small_ball = small_ball / 1.4
                else:
                    raise DataObjectGeneralException("Error in trying to find small_ball...")
                    #return -1

            big_ball = 0.1  # start from a small radius and then increase it iteratively until it outer-approximates
            # the manifold
            while True:  # iterate until big_ball outer-approximates the manifold
                expression2 = (temp - (0.99 * big_ball) ** 2 >= 0) & \
                              (temp - (1.01 * big_ball) ** 2 <= 0)  # this is the expression that defines the ball
                # check if big_ball outer-approximates the manifold
                res = data_obj.check_radius('o', manifold_time, expression2, big_ball, 0.001, 100, verbose=verbose)
                if res == 1:  # if it does, then proceed
                    print("Outer ball radius = %f" % (big_ball))
                    break
                elif res == -1:  # if it doesn't, increase and iterate again
                    big_ball = big_ball * 1.4
                else:
                    raise DataObjectGeneralException("Error in trying to find small_ball...")
                    #return -1

        # Refine the obtained small_ball and big_ball for each conic section of the manifold
        for cone in all_combinations_of_cone_matrices:  # iterate along all n-dimensional cones
            # in order to consider each conic section of the manifold
            flag = True
            if (dimension / 2 > 2):  # if dimension>=3, check if the considered cone is degenerate
                flag = data_obj.check_cone_degeneracy(cone)
            if (flag):  # if the cone is empty, delete it and move on to the next cone
                print('deleted')
            else:
                conic_domain = True  # this is going to be the symbolic expression defining the conic domain
                for i in range(0, int(
                        dimension / 2) - 1):  # create iteratively the symbolic expression for the conic domain
                    # by iteratively taking conic_domain = conic_domain & [x_i x_{i+1}]*Q_i*[x_i x_{i+1}]^T & L_i*[x_i x_{i+1}]
                    # where Q_i and L_i are the quadratic and linear forms that define the cone, respectively
                    vec = sympy.Matrix([state_vector[i], state_vector[i + 1]])
                    quadratic_form = sympy.Matrix(cone[i].quadratic_Form)
                    conic_domain = conic_domain & ((vec.transpose() * quadratic_form * vec)[0] >= 0)
                    linear_form = sympy.Matrix(cone[i].linear_Form)
                    conic_domain = conic_domain & ((linear_form.transpose() * vec)[0] >= 0)
                # refine the inner-approximation radius
                inner_radius = data_obj.radius_conic_section('i', conic_domain, manifold_time, small_ball, verbose)
                # refine the outer-approximation radius
                outer_radius = data_obj.radius_conic_section('o', conic_domain, manifold_time, big_ball, verbose)
                # create the Spherical_Domain object and append it to self.spherical_domains
                data_obj.spherical_domains.append(SphericalDomain(manifold_time, cone, inner_radius, outer_radius))
        #data_obj.spherical_sectors(manifolds_times[len(manifolds_times) - 1], nr_cones_small_angles, nr_cones_big_angle,
        #                           verbose)

        print('Finished Computing the radii')
        print('Creating the Regions and their over-approximations')
        data_obj.regions = data_obj.create_regions_objects(data_obj.manifolds_times)
        print('Moving on to timing upper bounds')
        data_obj.timing_upper_bounds(len(data_obj.manifolds_times), data_obj.timeout_upper_bounds,
                                     data_obj.remainder_upper_bounds, data_obj.heartbeat, verbose=False)
        data_obj.transitions(verbose, data_obj.timeout_reachability, data_obj.remainder_reachability)
    else:
        # Create a grid object
        data_obj.grid = data_obj.create_grid(data_obj.hyperbox_states, data_obj.grid_pts_per_dim)

        """INPUTS:
            manifolds_times: list of int, >0 (practically I only use manifolds_times[-1])

        Creates the objects Region_NonHomogeneous (for homogeneous systems), using
        the manfifold of time=manifolds_times[-1] and the created grid (by self.create_grid).
        """
        manifold_time = data_obj.manifolds_times[-1]
        dimension = int(data_obj.n / 2) - 1
        polytope_sides_lengths = []
        for i in range(0, dimension):  # for each dimension
            lim_min = data_obj.grid.state_space_limits[i][0]  # calculate what is the displacement
            lim_max = data_obj.grid.state_space_limits[i][1]  # from the center of a grid polytope
            side_length = (lim_max - lim_min) / data_obj.grid.grid_points_per_dim[i]  # to its vertices
            polytope_sides_lengths.append([-side_length / 2, side_length / 2])
        # create a list with all combinations of displacement. each combination, when added to the center of
        # a polytope, it gives one of its vertices.
        differences_between_all_vertices_and_centroid = list(itertools.product(*polytope_sides_lengths))
        region_index = 0
        data_obj.regions = []
        for centroid in data_obj.grid.centroids:  # iterate along all polytopes, each of which representing a region
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
            upper_bound = data_obj.reach_analysis_upper_bounds(region, verbose, data_obj.timeout_upper_bounds,
                                                               data_obj.remainder_upper_bounds, data_obj.heartbeat)
            region.insert_timing_upper_bound(upper_bound)
        #data_obj.timing_upper_bounds(None, data_obj.time_out_upper_bounds, data_obj.remainder_upper_bounds,  data_obj.heartbeat, verbose)

        data_obj.transitions(False, data_obj.timeout_reachability, data_obj.remainder_reachability)
