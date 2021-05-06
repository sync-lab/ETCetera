from typing import Set
import os
from config import save_path
from .timed_automaton import TimedAutomaton, Action


class TimedGameAutomaton(TimedAutomaton):
    controlled_actions: Set[Action]
    uncontrolled_actions: Set[Action]

    def __init__(self, controlled_actions, uncontrolled_actions, *args, **kwargs):
        self.controlled_actions = controlled_actions
        self.uncontrolled_actions = uncontrolled_actions
        super().__init__(*args, **kwargs)

    def _export_plain(self, file_name: str):
        if not file_name.endswith('.txt'):
            file_name += '.txt'
        super()._export_plain(file_name)

        output = f"""\
controlled actions: {self.controlled_actions}
uncontrolled actions: {self.uncontrolled_actions}"""

        with open(os.path.join(save_path, file_name), 'a') as f:
            f.write(str(output))


class NetworkModel(TimedGameAutomaton):
    """
    Modified from:
    https://github.com/pschalkwijk/Python2Uppaal
    https://github.com/asamant/masters-thesis-sched-strat

    This represents a simple model describing a communication network as in:
    https://arxiv.org/abs/1610.03729
    In this model, a subset of the edges is uncontrolled. It is possible to set these edges
    controlled as well using the parameter: all_controlled
    """

    delta: float  # Maximum channel occupancy time

    # TODO: Add the multiple channels stuff and timeout stuff from aniket
    def __init__(self, delta: float, sync: str = 'up', all_controlled: bool = False):
        self.delta = delta

        clocks = {'c'}
        locations = {'Idle', 'InUse', 'Bad'}
        initial_location = 'Idle'
        invariants = {'InUse': f'c<={delta}', 'Idle': '', 'Bad': ''}
        if all_controlled:
            controlled_actions = {'*', f'{sync}?'}
            uncontrolled_actions = {}
        else:
            controlled_actions = {'*'}
            uncontrolled_actions = {f'{sync}?'}

        actions = controlled_actions.union(uncontrolled_actions)
        guards = {True, False, f'c == {delta}'}
        transitions = {('Idle', 'true', 'up?', frozenset({'c'}), 'InUse'),
                       ('InUse', f'c=={delta}', '*', frozenset(), 'Idle'),
                       ('InUse', 'true', 'up?', frozenset(), 'Bad'), ('Bad', 'true', 'up?', frozenset(), 'Bad')}

        super().__init__(controlled_actions, uncontrolled_actions,
                         locations, invariants, guards, actions, clocks, transitions, initial_location=initial_location)


if __name__ == '__main__':
    net = NetworkModel(1)
    # print(net.export())
    net.export(export_type='xml')
    # location of your uppaal installation
    VERIFYTA = "/home/ivo/uppaal-4.0.15/bin-Linux/verifyta"

    strategy_name = 'strat1'
    print(os.path.exists(VERIFYTA))
    # Write a query to queries/{strategy_name}.q
    print(os.path.exists('../queries'))
    with open(f'../queries/{strategy_name}.q', 'w') as file:
        # file.write(f"strategy {strategy_name} = control: A[] not (NetworkModel.Bad)")
        file.write('')

    # Execute verifyta with the files just created. Output target directory 'strat'
    # arg_list = [VERIFYTA, '-u', '-s', '--generate-strategy', '2', '--print-strategies', 'strat',
    #             f'../saves/NetworkModel.xml', f'../queries/{strategy_name}.q']
    arg_list = [VERIFYTA, '../saves/NetworkModel.xml', f'../queries/{strategy_name}.q']
    print(' '.join(arg_list))
    import subprocess
    verify_ta = subprocess.run(arg_list, stdout=subprocess.PIPE)
    result = verify_ta.stdout.decode('utf-8')
    print(result)
