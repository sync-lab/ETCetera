import sympy
import sentient.util as utils

# Define
state_vector = x1, x2, e1, e2 = sympy.symbols('x1 x2 e1 e2')

# Define controller (in etc form)
u1 = -(x2+e2) - (x1+e1)**2*(x2+e2) - (x2+e2)**3

# Define dynamics
x1dot = -x1
x2dot = x1**2*x2 + x2**3 + u1
dynamics = [x1dot, x2dot, -x1dot, -x2dot]

# Make the system homogeneous (with degree 2)
hom_degree = 2
dynamics, state_vector = utils.make_homogeneous_etc(dynamics, state_vector, hom_degree)
dynamics = sympy.Matrix(dynamics)

# Triggering condition & other etc.
trigger = e1 ** 2 + e2 ** 2 - (x1 ** 2 + x2 ** 2) * (0.0127 * 0.3) ** 2

# State space limits
state_space_limits = [[-2.5, 2.5], [-2.5, 2.5]]

import sentient.Abstractions as abstr

traffic = abstr.TrafficModelNonlinearETC(dynamics, hom_degree, trigger, state_vector, homogenization_flag=True, state_space_limits=state_space_limits)
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

# To pickle the object:
traffic.export('traffic_etc', 'pickle')

# To save to a .json file:
traffic.export('traffic_etc', 'json')
