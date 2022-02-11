#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 21 15:22:20 2021

@author: ggleizer
"""

import ETCetera.Abstractions as abstr
import ETCetera.Scheduling.fpiter as sched
import time
from datetime import timedelta
model = abstr.TrafficModelLinearPETC



print('========== STATISTICS ON SYSTEM REDUCTION (TABLE 1) ==========')

for l in range(1,4):
    print(f'l={l}:')

    t = model.from_bytestream_file(f'altsim_model_{l}.pickle')


    c = sched.controlloop(t, use_bdd=False, init_steps=10)
    x,x0,d = c.stats()
    print(f'\tOriginal system: |X| = {x}, |X_0| = {x0}, |delta| = {d}')

    now = time.process_time()
    c.reduce_altsim()
    elapsed = timedelta(seconds=time.process_time() - now)
    x,x0,d = c.stats()
    print(f'\tMinimized system: |X| = {x}, |X_0| = {x0}, |delta| = {d}')

    print('\tElapsed time:', elapsed, '\n')



