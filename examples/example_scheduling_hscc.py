from ETCetera.util import *

# First construct the traffic models
traffic1 = construct_linearPETC_traffic_from_file('examples/linear_hscc1.txt')
traffic2 = construct_linearPETC_traffic_from_file('examples/linear_hscc2.txt')

import ETCetera.Scheduling.fpiter as sched

# Convert traffic models into different form
cl1 = sched.controlloop(traffic1, use_bdd=False)
cl2 = sched.controlloop(traffic2, use_bdd=False)

# Construct system and generate scheduler
S = sched.system([cl1, cl2])
Ux, _ = S.generate_safety_scheduler()

# Simulate
S.simulate(x0=[[1, 1], [1, -1]], Tmax=5, use_scheduler=True)

# Simulate without scheduler for reference
S.simulate(x0=[[1, 1], [1, -1]], Tmax=5, use_scheduler=False)

