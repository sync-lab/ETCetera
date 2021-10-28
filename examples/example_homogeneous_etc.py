#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 28 10:18:23 2021

@author: gdelimpaltadak
"""
import sympy

# Define state vector + corresponding measurement errors
state_vector = x1,x2,e1,e2 = sympy.symbols('x1 x2 e1 e2')

# Define controller (in etc form)
u1 = -(e2+x2)**3 - (e1+x1)*(e2+x2)**2

# Define dynamics
x1dot = -x1**3 + x1*x2**2
x2dot = x1*x2**2- x2*x1**2 + u1
dynamics = [x1dot, x2dot, -x1dot, -x2dot]

# Triggering condition & other etc.
trigger = e1**2 + e2**2 - (x1**2+x2**2)*(0.0127*0.3)**2
# Order of manifold approximations
p = 4
    
import sentient.Abstractions as abstr

#Declare parameters
state_space_limits = [[-2.5,2.5],[-2.5,2.5]]
partition_method = 'manifold'
manifolds_times = [0.0004, 0.0006, 0.0008,0.001, 0.0013, 0.0016, 0.0019, 0.0023]
angles_discretization = [6]
heartbeat=0.0042; order_approx=4; precision_deltas=1e-6;

# Abstract the system
traffic = abstr.TrafficModelNonlinearETC(dynamics, trigger, state_vector, \
                                state_space_limits=state_space_limits, \
                                manifolds_times=manifolds_times, \
                                partition_method='manifold', \
                                angles_discretization=angles_discretization, \
                                heartbeat=heartbeat, order_approx=order_approx)
regions, transitions = traffic.create_abstraction()

# Access regions/transitions in alternative way
regions = traffic.regions
transitions = traffic.transitions

print(regions)
print(transitions)

# # Get symbolic expressions for the regions
region_descriptors = traffic.return_region_descriptors()

print(region_descriptors)

# To pickle the object:
traffic.export('traffic_homogeneous_etc', 'pickle')

# To save to a .json file:
traffic.export('traffic_homogeneous_etc', 'json')

# To save to a text file:
traffic.export('traffic_homogeneous_etc', 'txt')    

    
    
    
    
    
    
    
    
    
    
    
    
    
    