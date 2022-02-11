#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 26 21:53:13 2021

@author: ggleizer
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 21 15:22:20 2021

@author: ggleizer
"""

import ETCetera.Abstractions as abstr
import ETCetera.Scheduling.fpiter as sched
model = abstr.TrafficModelLinearPETC

import logging
import time
from datetime import timedelta
import os

this_file_path = os.path.dirname(os.path.abspath(__file__))

traffic = []
stats_before = []
stats_after = []
cl = []
c = []

machine_stats = {}  # Save for displaying later

p = 2

print('======== SOLVING SCHEDULING PROBLEM USING BDDS =========\n')

l = 1
print(f'*********** l={l}')

t = model.from_bytestream_file(f'altsim_model_{l}.pickle')
#  Create reduced system
clenum = sched.controlloop(t, use_bdd=False, init_steps=10)
clenum.reduce_altsim()

for _ in range(2):
    c.append(sched.controlloop(t, use_bdd=True, init_steps=10))
    c[-1]._from_enum(clenum)

while True:
    name = f'bdd_basic_{t.depth}_{len(c)}'
    print(f'\t p={len(c)}... building and solving scheduling problem...')

    S = sched.system(c, name=name)
    S.bdd.configure(reordering=False)  # Faster this way
    now = time.process_time()
    Ux, Q = S.generate_safety_scheduler(basic=True)
    elapsed = timedelta(seconds=time.process_time() - now)
    print('Elapsed time:', elapsed)

    if Ux is None:  # Problem is unschedulable
        machine_stats[(l,p)] = (None, elapsed)
        break
    else:
        strat_size = os.path.getsize(this_file_path
                                     + f'/../saves/scheduler_{name}.dddmp')
        machine_stats[(l,p)] = (strat_size, elapsed)
        c.append(sched.controlloop(t, use_bdd=True, init_steps=10))
        c[-1]._from_enum(clenum)
        p += 1

''' Now l = 2 '''
l = 2
print(f'*********** l={l}')

t = model.from_bytestream_file(f'altsim_model_{l}.pickle')
#  Create reduced system
clenum = sched.controlloop(t, use_bdd=False, init_steps=10)
clenum.reduce_altsim()

c = []
for _ in range(p):
    c.append(sched.controlloop(t, use_bdd=True, init_steps=10))
    c[-1]._from_enum(clenum)

name = f'bdd_basic_{t.depth}_{len(c)}'

S = sched.system(c, name=name)
S.bdd.configure(reordering=False)  # Faster this way
now = time.process_time()
Ux, Q = S.generate_safety_scheduler(basic=True)
elapsed = timedelta(seconds=time.process_time() - now)
print('Elapsed time:', elapsed)
if Ux is None:
    machine_stats[(l,p)] = (None, elapsed)

''' Finally l = 3 '''
l = 3
print(f'*********** l={l}')

t = model.from_bytestream_file(f'altsim_model_{l}.pickle')
#  Create reduced system
clenum = sched.controlloop(t, use_bdd=False, init_steps=10)
clenum.reduce_altsim()

c = []
for _ in range(p):
    c.append(sched.controlloop(t, use_bdd=True, init_steps=10))
    c[-1]._from_enum(clenum)

name = f'bdd_basic_{t.depth}_{len(c)}'

S = sched.system(c, name=name)
S.bdd.configure(reordering=False)  # Faster this way
now = time.process_time()
Ux, Q = S.generate_safety_scheduler(basic=True)
elapsed = timedelta(seconds=time.process_time() - now)
print('Elapsed time:', elapsed)
if Ux is None:
    machine_stats[(l,p)] = (None, elapsed)


''' Print stats '''
print('****** Statistics for minimized system (Table 2) ******')
for (ll,pp), (size, cput) in machine_stats.items():
    print(f'l={ll}, p={pp}: scheduler size = {size} bytes, CPU time = {cput}')