# Loading the traffic models
import sys, os

# Check if examples/example_linearpetc.py has been run, as its result is used here
from config import root_path
if not os.path.exists(os.path.join(root_path, 'saves/traffic_petc.pickle')):
    print('Run examples/example_linearpetc.py first, as the resulting traffic model is used here!')
    sys.exit()

import ETCetera.Abstractions as abstr
traffic = abstr.TrafficModelLinearPETC.from_bytestream_file('traffic_petc.pickle')

# Scheduling by solving a safety game
import ETCetera.Scheduling.fpiter as sched

# Without BDDs
cl1 = sched.controlloop(traffic, use_bdd=False)
cl2 = sched.controlloop(traffic, use_bdd=False)
S = sched.system([cl1, cl2])
Uxe, Qe = S.generate_safety_scheduler()  # Scheduler
print(str(Uxe)[0:100])

# With BDDs
cl1 = sched.controlloop(traffic, use_bdd=True)
cl2 = sched.controlloop(traffic, use_bdd=True)
S = sched.system([cl1, cl2])
Ux, Q = S.generate_safety_scheduler()  # Scheduler

# With Late triggers
cl1 = sched.controlloop(traffic, use_bdd=True)
cl2 = sched.controlloop(traffic, use_bdd=True, maxLate=2, ratio=2)
S = sched.system([cl1, cl2])
Uxl, Ql = S.generate_safety_scheduler()  # Scheduler
