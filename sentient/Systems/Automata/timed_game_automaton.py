from typing import Set
import os
from config import save_path
from sentient import pyuppaal
from .timed_automaton import TimedAutomaton


class TimedGameAutomaton(TimedAutomaton):

    """
    Class Representing a timed game automaton. Not
    """

    def __init__(self, locations, invariants, controlled_actions, uncontrolled_actions,
                 clocks, transitions, name: str = None, initial_location=None, urgent=None):
        # transitions in the form: (i, g, a_c, a_u, c, t)
        if not all([len(tr) == 6 for tr in transitions]):
            print('Supply transitions in the format (i, g, a_c, a_u, c, t)')
            raise ValueError
        self.controlled_actions = controlled_actions
        self.uncontrolled_actions = uncontrolled_actions
        actions = controlled_actions.union(uncontrolled_actions)
        super().__init__(locations, invariants, actions, clocks, transitions,
                         name, initial_location=initial_location, urgent=urgent)

    def _check_guards(self):
        # For (i,g,a_c, a_u, r, t)
        for (_, g, _, _, _, _) in self.transitions:
            if type(g) == bool:
                continue
            if type(g) == list:
                if not all([type(i) == tuple and len(i) == 2 or type(i) == bool for i in g]):
                    return False
            elif not type(g) == tuple or not len(g) == 2:
                return False

        return True

    def _export_plain(self, file_name: str):
        if not file_name.endswith('.txt'):
            file_name += '.txt'
        super()._export_plain(file_name)

        output = f"""\
controlled actions: {self.controlled_actions}
uncontrolled actions: {self.uncontrolled_actions}"""

        with open(os.path.join(save_path, file_name), 'a') as f:
            f.write(str(output))



    def _generate_transitions(self):
        return [pyuppaal.Transition(src, trg, guard=self._interval2str(guard), assignment='c=0',
                                    controllable=(act_c is not frozenset()))
                for (src, guard, act_c, act_u, _, trg) in self.transitions]


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
    def __init__(self, name: str = 'Network', delta: float = 1.0, sync: str = 'up', all_controlled: bool = False):
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
        transitions = {('Idle', 'true', f'{sync}?', frozenset({'c'}), 'InUse'),
                       ('InUse', f'c=={delta}', '*', frozenset(), 'Idle'),
                       ('InUse', 'true', f'{sync}?', frozenset(), 'Bad'),
                       ('Bad', 'true', f'{sync}?', frozenset(), 'Bad')}

        super().__init__(locations, invariants, guards, uncontrolled_actions, controlled_actions,
                         clocks, transitions, name, initial_location=initial_location)


# class NetworkModelTimeout(NetworkModel):


if __name__ == '__main__':
    net = NetworkModel(1)
    # print(net.export())
    net.export(export_type='xml')
    # location of your uppaal installation
    VERIFYTA = "/home/ivo/uppaal-4.0.15/bin-Linux/verifyta"

    strategy_name = 'strat1'
    print(os.path.exists(VERIFYTA))
    # Write a query to queries/{strategy_name}.q
    print(os.path.exists('../../../queries'))
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


