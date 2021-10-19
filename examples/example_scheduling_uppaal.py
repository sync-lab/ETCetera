# Loading the traffic models
import sys, os

# Check if examples/example_linearpetc.py has been run, as its result is used here
from config import root_path
if not os.path.exists(os.path.join(root_path, 'saves/traffic_petc.pickle')):
    print('Run examples/example_linearpetc.py first, as the resulting traffic model is used here!')
    sys.exit()

import sentient.Abstractions as abstr
traffic = abstr.TrafficModelLinearPETC.from_bytestream_file('traffic_petc.pickle')

# Scheduling using UPPAAL Stratego
import sentient.Scheduling.NTGA as sched
cl1 = sched.controlloop(traffic)
cl2 = sched.controlloop(traffic)
net = sched.network()
nta = sched.nta(net, [cl1, cl2])
nta.generate_strategy(parse_strategy=True)


# Loading the nonlinear traffic model
import sentient.Abstractions as abstr
if not os.path.exists(os.path.join(root_path, 'saves/traffic_etc.pickle')):
    print('Run examples/example_nonlinearetc.py first, as the resulting traffic model is used here!')
    sys.exit()

traffic = abstr.TrafficModelLinearPETC.from_bytestream_file('traffic_etc.pickle')

# Scheduling using UPPAAL Stratego
import sentient.Scheduling.NTGA as sched
cl1 = sched.controlloop(traffic)
cl2 = sched.controlloop(traffic)
net = sched.network()
nta = sched.nta(net, [cl1, cl2])
nta.generate_strategy(parse_strategy=True)
