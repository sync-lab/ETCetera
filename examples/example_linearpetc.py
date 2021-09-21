import numpy as np

# Define LTI system matrices
A = np.array([[1.38, -0.208, 6.715, -5.676], [-0.581, -4.29, 0, 0.675], [1.067, 4.273, -6.654, 5.893], [0.048, 4.273, 1.343, -2.104]])
B = np.array([[0, 0],[5.679, 0], [1.136, 3.146],[1.136, 0]])
K = np.array([[0.518, -1.973, -0.448, -2.1356], [-3.812, -0.0231, -2.7961, 1.671]])

# PETC parameters
h = 0.01
kmax = 20
sigma = 0.01

# Triggering condition
Qtrigger = np.block([[(1-sigma)*np.eye(4), -np.eye(4)], [-np.eye(4), np.eye(4)]])


# Construct object representing the PETC system
import sentient.Systems as systems

plant = systems.LinearPlant(A, B)
controller = systems.LinearController(K, h)
trigger = systems.LinearQuadraticPETC(plant, controller, kmax=kmax, Qbar=Qtrigger)

# Create Traffic Model
import sentient.Abstractions as abstr

traffic = abstr.TrafficModelLinearPETC(trigger)
regions, transitions = traffic.create_abstraction()

# Access regions/transitions in alternative way
regions = traffic.regions
transitions = traffic.transitions

print(regions)
print(transitions)

# Get symbolic expressions for the regions
region_descriptors = traffic.return_region_descriptors()

print(region_descriptors)

# To pickle the object:
traffic.export('traffic_petc', 'pickle')

# To save to a .json file:
traffic.export('traffic_petc', 'json')