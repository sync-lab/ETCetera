import numpy as np
import ETCetera.Systems as systems
import ETCetera.Abstractions as abstr


# Define LTI system matrices
A = np.array([[ 0,  1],
              [-2,  3]])
B = np.array([[0],
              [1]])

K = np.array([[ 1, -4]])  # Tabuada

# PETC parameters
h = 0.05  # Channel occupancy time
sigma = 0.7
kmax = 50

for depth in range(1,4):
    print('=============\n\n'
          f'Generating abstraction for l={depth}\n\n=============')

    Qtrigger = np.block([[(1-sigma**2)*np.eye(2), -np.eye(2)],
                         [-np.eye(2), np.eye(2)]])

    # Construct object representing the PETC system
    plant = systems.LinearPlant(A, B)
    controller = systems.LinearController(K, h)
    trigger = systems.LinearQuadraticPETC(plant,
                                          controller,
                                          kmax=kmax,
                                          Qbar=Qtrigger)

    # Create Traffic Model
    traffic = abstr.TrafficModelLinearPETC(trigger,
                                           solver='z3',
                                           depth=depth,
                                           early_trigger_only=True,
                                           symbolic=False)

    regions, transitions = traffic.create_abstraction()

    traffic.export(f'altsim_model_{depth}', 'pickle')