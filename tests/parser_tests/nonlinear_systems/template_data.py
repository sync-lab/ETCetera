# -*- coding: utf-8 -*-

import ETCetera.data.nonlinear_systems_datastructure as data
import sympy as sp

def create_template():
    template_data = data.InputDataStructureNonLinear()
    [template_data.hyperbox_states.append(list([i+0.2,i+3.4])) for i in range(1,4)]
    [template_data.hyperbox_disturbances.append(list([i+0.2,i+3.4])) for i in range(4,7)]
    
    [template_data.dynamics_inputs.append(sp.Symbol('u'+str(i))) for i in  range(0,2)]
    [template_data.dynamics_states.append(sp.Symbol('x'+str(i))) for i in  range(0,3)]
    [template_data.dynamics_disturbances.append(sp.Symbol('d'+str(i))) for i in  range(0,3)]
    [template_data.dynamics_errors.append(sp.Symbol('e'+str(i))) for i in  range(0,3)]
    
    sym_dynamics = list(['x0**2+u0+d0', 'x1+x0*x2**2+d1', 'x2*sin(x0)+u1+d2'])
    [template_data.dynamics.append(sp.simplify(i)) for i in sym_dynamics]
    
    sym_controller = list(['-x0**2 - x0**3', '-x2*sin(x0)-x2'])
    [template_data.controller.append(sp.simplify(i)) for i in sym_controller]
    
    sym_triggering_condition = '-x0**2 - e0*sin(x1)'
    template_data.triggering_condition = sp.sympify(sym_triggering_condition)
    
    [template_data.triggeting_times.append(i+0.2) for i in range(10,16)]
    
    sym_lyapunov_function = '-x0**2 - sin(x1)'
    template_data.lyapunov_func = sp.sympify(sym_lyapunov_function)
    
    [template_data.solver_options.append(i+0.2) for i in range(10,16)]
    
    [template_data.lineSearch_options.append(i+0.2) for i in range(10,16)]
    
    template_data.is_homogenous = False
    
    sym_etc_controller = list(['-(e0 + x0)**3 - (e0 + x0)**2', '-e2 - x2 + (-e2 - x2)*sin(e0 + x0)'])
    [template_data.etc_controller.append(sp.simplify(i)) for i in sym_etc_controller]
    
    
    
    
    
    
    
    
    
    
    
    
    