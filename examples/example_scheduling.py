# Loading the traffic models
import sentient.Abstractions as abstr
traffic1 = abstr.TrafficModelLinearPETC.from_bytestream_file('traffic1.pickle')
traffic2 = abstr.TrafficModelLinearPETC.from_bytestream_file('traffic1.pickle')


# Scheduling using UPPAAL Stratego
import sentient.Scheduling.NTGA as sched
cl1 = sched.controlloop(traffic1)
cl2 = sched.controlloop(traffic1)
net = sched.network()
nta = sched.nta(net, [cl1, cl2])
nta.generate_strategy(parse_strategy=True)

# Scheduling by solving a safety game
import sentient.Scheduling.fpiter as sched

# Without BDDs
cl1 = sched.controlloop(traffic1, use_bdd=False)
cl2 = sched.controlloop(traffic1, use_bdd=False)
S = sched.system([cl1, cl2])
Ux, Q = S.generate_safety_scheduler()  # Scheduler
print(str(Ux)[0:100])
print(str(Q)[0:100])

# With BDDs
cl1 = sched.controlloop(traffic1, use_bdd=True)
cl2 = sched.controlloop(traffic1, use_bdd=True)
S = sched.system([cl1, cl2])
Ux = S.generate_safety_scheduler()  # Scheduler

# With Late triggers
cl1 = sched.controlloop(traffic1, use_bdd=True)
cl2 = sched.controlloop(traffic1, use_bdd=True, maxLate=2, ratio=2)
S = sched.system([cl1, cl2])
Ux = S.generate_safety_scheduler()  # Scheduler


# Loading the nonlinear traffic model
import sentient.Abstractions as abstr
traffic1 = abstr.TrafficModelLinearPETC.from_bytestream_file('traffic_etc.pickle')
print(traffic1.regions)

# Scheduling using UPPAAL Stratego
import sentient.Scheduling.NTGA as sched
cl1 = sched.controlloop(traffic1)
cl2 = sched.controlloop(traffic1)
net = sched.network()
nta = sched.nta(net, [cl1, cl2])
nta.generate_strategy(parse_strategy=True)