# Loading the traffic models
import sys, os, logging, time
from datetime import timedelta

logging.getLogger().setLevel(logging.INFO)

# Check if examples/example_linearpetc.py has been run, as its result is used here
from config import root_path
if not os.path.exists(os.path.join(root_path, 'saves/ifac1.pickle')):
    print('Run "python3 etc2traffic.py -o ifac1.json linear examples/linear_ifac2020.txt"'
          ' first, as the resulting traffic model is used here!')
    sys.exit()
if not os.path.exists(os.path.join(root_path, 'saves/ifac2.pickle')):
    print('Run "python3 etc2traffic.py -o ifac2.json linear examples/linear_ifac2020_2.txt"'
          ' first, as the resulting traffic model is used here!')
    sys.exit()

import ETCetera.Abstractions as abstr
traffic1 = abstr.TrafficModelLinearPETC.from_bytestream_file('ifac1.pickle')
traffic2 = abstr.TrafficModelLinearPETC.from_bytestream_file('ifac2.pickle')

# Scheduling by solving a safety game
import ETCetera.Scheduling.fpiter as sched

d = 10

# With BDDs
cl1 = sched.controlloop(traffic1, use_bdd=True, init_steps=d)
cl2 = sched.controlloop(traffic1, use_bdd=True, init_steps=d)
cl3 = sched.controlloop(traffic1, use_bdd=True, init_steps=d)
cl4 = sched.controlloop(traffic1, use_bdd=True, init_steps=d)
cl5 = sched.controlloop(traffic2, use_bdd=True, init_steps=d)
S = sched.system([cl1, cl2, cl3, cl4, cl5], trap_state=False)

now = time.process_time()
Ux, _ = S.generate_safety_scheduler()  # Scheduler
elapsed = timedelta(seconds=time.process_time() - now)
print('Elapsed time on synthesis:', elapsed)


# With BDDs
cl6 = sched.controlloop(traffic2, use_bdd=True, init_steps=d)
S2 = sched.system([cl1, cl2, cl3, cl4, cl5, cl6], trap_state=False)
S2.bdd.configure(reordering=False)

now = time.process_time()
Ux2, _ = S2.gen_safety_scheduler_basic()  # Scheduler
elapsed = timedelta(seconds=time.process_time() - now)
print('Elapsed time on synthesis:', elapsed)