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

p = 2

print('======== SOLVING SCHEDULING PROBLEM USING PYTHON-BASED FIXED-POINT ITERATION =========\n')

l = 1
print(f'*********** l={l}')

t = model.from_bytestream_file(f'altsim_model_{l}.pickle')
#  Create reduced system
clenum = sched.controlloop(t, use_bdd=False, init_steps=10)
clenum.reduce_altsim()

for _ in range(2):
    c.append(clenum)

while True:
    name = f'fpiter_basic_{t.depth}_{len(c)}'
    print(f'\t******p={len(c)}... building and solving scheduling problem...')

    S = sched.system(c, name=name)
    now = time.process_time()
    Ux, Q = S.gen_safety_scheduler_basic()
    elapsed = timedelta(seconds=time.process_time() - now)
    print('\t******Elapsed time:', elapsed)

    if Ux is None:  # Problem is unschedulable
        break
    else:
        c.append(clenum)
        p += 1
