import control_system_abstractions.data.linear_systems_datastructure as data
from control_system_abstractions.linear_systems_utils.abstraction import TrafficModelPETC, traffic2ta


def create_abstractions(data_obj: data.InputDataStructureLinear):
    traffic_model = TrafficModelPETC(data_obj.triggering_mechanism,
                                     **data_obj.solver_options,
                                     **data_obj.abstraction_options)
    data_obj.traffic_model = traffic_model