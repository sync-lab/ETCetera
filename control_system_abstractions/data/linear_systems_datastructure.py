# -*- coding: utf-8 -*-
import control_system_abstractions.linear_systems_utils.abstraction as abstraction
import control_system_abstractions.linear_systems_utils.linearetc as etc
import control_system_abstractions.linear_systems_utils.linearsys as linearsys

class InputDataStructureLinear():
    traffic_model : abstraction.TrafficModelPETC

    # Construtor
    def __init__(self, dynamics=[], controller=0, triggering_condition=0, triggering_heartbeat=0,
                 triggering_sampling_time=0, is_PETC=True, lyapunov_func=0, solver_options=[], abstraction_options=[]):
        self.dynamics = dynamics
        self.controller = controller
        self.triggering_condition = triggering_condition
        self.triggering_heartbeat = triggering_heartbeat
        self.triggering_sampling_time = triggering_sampling_time
        self.is_PETC = is_PETC
        self.lyapunov_func = lyapunov_func
        self.solver_options = solver_options
        self.abstraction_options = abstraction_options

        if not is_PETC:
            raise Exception('Only PETC is currently supported.')

        self.plant = linearsys.LinearPlant(dynamics[0], dynamics[1])
        self.controller = linearsys.LinearController(controller, triggering_sampling_time)
        kmax = int(triggering_heartbeat/triggering_sampling_time)
        print('here')
        print(self.plant.__dict__)
        print(self.controller.__dict__)
        self.triggering_mechanism = etc.LinearQuadraticPETC(self.plant,
                                                            self.controller,
                                                            kmax=kmax,
                                                            Qbar=triggering_condition,
                                                            P=lyapunov_func)
