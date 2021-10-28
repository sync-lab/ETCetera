import sympy

# Define
state_vector = x1, x2, e1, e2 = sympy.symbols('x1 x2 e1 e2')

# Define controller (in etc form)
u1 = -(x2+e2) - (x1+e1)**2*(x2+e2) - (x2+e2)**3

# Define dynamics
x1dot = -x1
x2dot = x1**2*x2 + x2**3 + u1
dynamics = [x1dot, x2dot, -x1dot, -x2dot]

# Triggering condition & other etc.
trigger = e1 ** 2 + e2 ** 2 - 0.01**2

# State space limits
state_space_limits = [[-2, 2], [-2, 2]]
grid_points_per_dim = [3,3]

import sentient.Abstractions as abstr

# Partition method manifold
traffic = abstr.TrafficModelNonlinearETC(dynamics, trigger, state_vector, state_space_limits=state_space_limits, grid_points_per_dim=grid_points_per_dim, manifolds_times=[0.002, 0.0028, 0.0038, 0.005, 0.0065, 0.0075], partition_method='manifold', heartbeat=0.021, order_approx=4)
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
traffic.export('traffic_etc', 'pickle')

# To save to a .json file:
traffic.export('traffic_etc', 'json')

# To save to a text file:
traffic.export('traffic_etc', 'txt')
