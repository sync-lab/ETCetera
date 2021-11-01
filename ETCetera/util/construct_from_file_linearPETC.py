import os, sys

from ETCetera.exceptions import *
import ETCetera.util.parsing as parsing

import ETCetera.Abstractions as abstr
import ETCetera.Systems.linearsys as linearsys
import ETCetera.Systems.linearetc as etc

def construct_linearPETC_traffic_from_file(file_name):

    """

    @param file_name:
    @return:

    Originally written

    """


    if not os.path.exists(file_name):
        print(f'File {file_name} could not be found.')
        return None

    dict_key_to_attrs = {'Dynamics': None, 'Controller': None, 'Triggering Heartbeat': None,
                         'Triggering Condition': None, 'Triggering Sampling Time': None, 'Lyapunov Function': None,
                         'Solver Options': {}, 'Abstraction Options': {}}

    try:
        with open(file_name, 'r') as reader:
            for line_num, line in enumerate(reader, 1):         # Read line along with line number
                line_key = line.split(':')[0].strip()           # Get the key in the line
                if line_key in dict_key_to_attrs.keys():
                    parsed_data = parsing.lp.parse_linear(line)         # Call the linear parsing function with line
                else:
                    raise IllegalKeyException(param=[line_key])

                if line_key in ('Solver Options', 'Linesearch Options'):  # If data is 'Solver Options', merge with default already defined
                    dict_key_to_attrs[line_key].update(parsed_data)
                else:                               # For all other data, simple assignment is fine
                    dict_key_to_attrs[line_key] = parsed_data
    except GenericParsingException as e:
        print(f'Parsing Error Occured on line {line_num}:     {line}')
        print(str(e))
        sys.exit()

    except SentientBaseException as e:
        print(str(e) + str(line_num))
        print(type(e))
        sys.exit()

    finally:
        pass




    _required_args = ['Dynamics', 'Controller', 'Triggering Heartbeat',
                         'Triggering Condition', 'Triggering Sampling Time']
    _missing_args = []
    for i in _required_args:
        if dict_key_to_attrs[i] is None:
            _missing_args.append(i)

    if len(_missing_args) > 0:
        raise IncompleteInputFileException(_missing_args)

    if dict_key_to_attrs['Triggering Sampling Time'] and \
            dict_key_to_attrs['Triggering Sampling Time'] < dict_key_to_attrs['Triggering Sampling Time']:
        raise SentientBaseException('Triggering sampling time should be lower than triggering heartbeat')

    dynamics = dict_key_to_attrs['Dynamics']
    controller = dict_key_to_attrs['Controller']
    triggering_sampling_time = dict_key_to_attrs['Triggering Sampling Time']
    triggering_heartbeat = dict_key_to_attrs['Triggering Heartbeat']
    triggering_condition = dict_key_to_attrs['Triggering Condition']
    lyapunov_func = dict_key_to_attrs['Lyapunov Function']



    plant = linearsys.LinearPlant(dynamics[0], dynamics[1])
    controller = linearsys.LinearController(controller, triggering_sampling_time)
    kmax = int(triggering_heartbeat / triggering_sampling_time)
    triggering_mechanism = etc.LinearQuadraticPETC(plant,
                                                   controller,
                                                   kmax=kmax,
                                                   Qbar=triggering_condition,
                                                   P=lyapunov_func)


    r = abstr.TrafficModelLinearPETC(triggering_mechanism, **dict_key_to_attrs['Solver Options'], **dict_key_to_attrs['Abstraction Options'])

    r.create_abstraction()

    return r