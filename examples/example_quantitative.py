import numpy as np
# import logging

# logging.getLogger().setLevel(logging.INFO)

# Define LTI system matrices
A = np.array([[0, 1], [-2, 3]])
B = np.array([[0],[1]])
K = np.array([[1, -4]])

# PETC parameters
h = 0.1
kmax = 20

# Triggering condition
Qtrigger = np.array([[ -0.14770797,  -2.52808495,  -0.20224506,   0.80898024],
                     [ -2.52808495,  13.75369717,   2.56242462, -10.24969847],
                     [ -0.20224506,   2.56242462,   0.33917205,  -1.3566882 ],
                     [  0.80898024, -10.24969847,  -1.3566882 ,   5.42675281]])


# Construct object representing the PETC system
import sentient.Systems as systems

plant = systems.LinearPlant(A, B)
controller = systems.LinearController(K, h)
trigger = systems.LinearQuadraticPETC(plant, controller, kmax=kmax, Qbar=Qtrigger)

# Create Traffic Model
import sentient.Abstractions as abstr


# Best selection for verification:
#    solver='z3', etc_only, stop_if_mace_equivalent, smart_mace
traffic = abstr.TrafficModelLinearPETC(trigger,
                                       #symbolic=True,
                                       depth=100,
                                       etc_only=True,
                                       solver='z3',
                                       stop_if_mace_equivalent=True,
                                       smart_mace=True)

regions, transitions = traffic.create_abstraction()

# Print obtained SAIST and its minimizing cycle
print(f'SAIST is {traffic.limavg}')
print(f'Smallest average cycles: {traffic.cycles}')


''' Now create an abstraction to design an improved sampling strategy '''
traffic_synth = abstr.TrafficModelLinearPETC(trigger,
                                             depth=1,
                                             etc_only=False,  # <--
                                             solver='z3',
                                             early_trigger_only=True)  # <--

strat, strat_tran = traffic_synth.optimize_sampling_strategy()
# Inspect strategy
print(strat)

# Compute its SAIST value
traffic2 = abstr.TrafficModelLinearPETC(trigger,
                                        #symbolic=True,
                                        depth=100,
                                        etc_only=True,
                                        solver='z3',
                                        stop_if_mace_equivalent=True,
                                        smart_mace=True,
                                        strategy=strat,  # <--
                                        strategy_transition=strat_tran)  # <--

regions2, transitions2 = traffic2.create_abstraction()

print(f'Optimized SAIST is {traffic2.limavg}')

