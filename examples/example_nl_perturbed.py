import sympy
import ETCetera.util as utils

# Define
state_vector = x1, x2, e1, e2 = sympy.symbols('x1 x2 e1 e2')
d1 = sympy.symbols('d1')

# Define controller (in etc form)
u1 = -(x2+e2) - (x1+e1)**2*(x2+e2) - (x2+e2)**3

# Define dynamics
x1dot = -x1
x2dot = x1**2*x2 + x2**3 + u1 + d1
dynamics = [x1dot, x2dot, -x1dot, -x2dot]

# Triggering condition & other etc.
trigger = e1 ** 2 + e2 ** 2 - 0.01**2

# State space limits
state_space_limits = [[-2, 2], [-2, 2]]
disturbace_limits = [[-0.1, 0.1]]

import ETCetera.Abstractions as abstr

traffic = abstr.TrafficModelNonlinearETC(dynamics, trigger, state_vector,
                                         state_space_limits=state_space_limits, dist_param=(d1,),
                                         dist_param_domain=disturbace_limits, order_approx=2, heartbeat=0.022,
                                         grid_points_per_dim=[7,8], partition_method='grid', precision_deltas=1e-6)
regions, transitions = traffic.create_abstraction()
# Result: {}

# Access regions/transitions in alternative way
regions = traffic.regions
transitions = traffic.transitions

print(regions)
print(transitions)

# # Get symbolic expressions for the regions
region_descriptors = traffic.return_region_descriptors()
print(region_descriptors)

# Visualize results
traffic.visualize()

# To pickle the object:
traffic.export('traffic_etc_dist', 'pickle')

# To save to a .json file:
traffic.export('traffic_etc_dist', 'json')
