from typing import Dict, Set, Tuple, NewType
import os, pickle, shortuuid
from functools import cached_property

from ETCetera import pyuppaal

from config import save_path


class TimedAutomaton:
    """
    Class representing a timed automaton. Generally not used for different scheduling algorithms.
    """

    def __init__(self, locations, invariants, actions, clocks, transitions, name: str = None,
                 initial_location=None, urgent=None):

        idx = shortuuid.uuid()[:6]
        self.name = name or f'cl_{idx}'

        self.locations = locations
        self.invariants = invariants
        self.initial_location = initial_location
        self.actions = actions
        self.clocks = clocks
        self.transitions = transitions
        self.urgent = urgent or {}
        self.index = idx

        # Set to scale all numerical values by this number when exporting to xml. This has to be done because
        # UPPAAL can only handle integer numbers.
        self.scale = 1
        #
        # if not self._check_guards():
        #     # TODO: Different Exception
        #
        #     raise Exception
        #
        # if not self._check_invariants():
        #     raise Exception

    def _check_invariants(self):
        for (x, inv) in self.invariants.items():
            if str(inv).lower() == 'true':
                continue
            if type(inv) == list:
                if not all([type(i) == tuple and len(i) == 2 or type(i) == bool or type(i) == str for i in inv]):
                    print(f'Supplied invariant list, but not all elements in list are correct')
                    return False
            elif not type(inv) == tuple or not len(inv) == 2:
                print(f"Invariant {x}:{inv} is not tuple or not length 2")
                return False
        return True

    def _check_guards(self):
        for (_, g, _, _, _) in self.transitions:
            if type(g) == bool:
                continue
            if type(g) == list:
                if not all([type(i) == tuple and len(i) == 2 or type(i) == bool or type(i) == str for i in g]):
                    return False
            elif not type(g) == tuple or not len(g) == 2:
                return False

        return True

    def export(self, file_name: str = None, export_type: str = 'txt'):
        """
        Exports the timed automaton to a specified file_type. Default is plain text
        """
        export_type = export_type.lower()
        if file_name is None:
            file_name = self.__class__.__name__
        if export_type in ['txt', 'text', 'plain']:
            self._export_plain(file_name)
        elif export_type in ['uppaal', 'xml']:
            self._to_xml(file_name)
        elif export_type in ['pickle', 'bytes', 'byte_stream']:
            self._export_pickle(file_name)

    def _export_plain(self, file_name: str):
        output = \
            f"""\
locations: {self.locations}
initial locations: {self.initial_location}
clocks: {self.clocks}
invariants: {self.invariants}
guards: {self.guards}
actions: {self.actions}
transitions: {self.transitions}        
"""
        # TODO: Possibly do this differently?
        if not file_name.endswith('.txt'):
            file_name += '.txt'

        with open(os.path.join(save_path, file_name), 'w') as f:
            f.write(str(output))

    def _export_pickle(self, file_name: str):
        if not file_name.endswith('.pickle'):
            file_name += '.pickle'

        with open(os.path.join(save_path, file_name), 'wb') as f:
            pickle.dump(self, f, pickle.HIGHEST_PROTOCOL)

    # TODO: Add option to either export for UPPAAL or UPPAAL Stratego
    def _to_xml(self, file_name: str):
        if not file_name.endswith('.xml'):
            file_name += '.xml'

        # declarations = self._generate_declarations()
        template = self._generate_template()
        import html
        system = f'{self.name}{self.index} = {self.name}();\n system {self.name}{self.index};'
        outputs = """<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE nta PUBLIC "-//Uppaal Team//DTD Flat System 1.1//EN" "http://www.it.uu.se/research/group/darts/uppaal/flat-1_2.dtd">
<nta>
  <declaration>%s</declaration>
  %s
  <system>%s</system>
</nta>""" % (html.escape(template.declaration), template.to_xml(), html.escape(system))

        with open(os.path.join(save_path, file_name), 'w') as f:
            f.write(outputs)


    """ Exporting to UPPAAL methods. The _generate_* methods should be overwritten in child classes. """

    @cached_property
    def template(self):
        return self._generate_template()

    def _generate_template(self):
        # locations = [pyuppaal.Location(invariant=self.invariants[loc], name=loc) for loc in self.locations]

        # transitions = [pyuppaal.Transition(src, trg, guard=guard, assignment='c=0') for (src, guard, _, _, trg) in self.transitions]
        locations = self._generate_locations()
        transitions = self._generate_transitions()
        declarations = self._generate_declarations()
        template = pyuppaal.Template(name=self.name, declaration=declarations,
                                     locations=locations,
                                     transitions=transitions)
        template.initlocation = template.get_location_by_name(self.initial_location)
        template.layout(auto_nails=True)

        return template

    def _generate_declarations(self):
        return f"clock {' ,'.join(self.clocks)};"

    def _generate_transitions(self):
        return [pyuppaal.Transition(src, trg, guard=self._interval2str(guard), assignment=f'c=0')
                for (src, guard, act, _, trg) in self.transitions]
        # return [pyuppaal.Transition(src, trg, guard=str(guard), assignment=f'c=0')
        #         for (src, guard, act, _, trg) in self.transitions]

    def _generate_locations(self):
        return [
            pyuppaal.Location(invariant=self._interval2str(self.invariants[loc]), urgent=(loc in self.urgent), name=loc)
            for loc in
            self.locations]
        # return [
        #     pyuppaal.Location(invariant=self.invariants[loc], urgent=(loc in self.urgent), name=loc)
        #     for loc in
        #     self.locations]

    def _interval2str(self, g):
        if g is None:
            return ''

        if type(g) == str:
            return g

        if type(g) == bool:
            return str(g).lower()

        if type(g) == tuple:
            g = [g]

        # if type(g) == frozenset:
        #     g = list(g)

        # For now assume all guards use same clock
        # TODO: Change this
        c = [list(self.clocks)[0]] * len(g)

        # if type(g) != list:
        #     return None

        # if not all([len(i) == 2 for i in g]):
        #     return None

        expr = []
        for i, j in zip(g, c):
            if type(i) == str:
                expr.append(i)
            elif i[0] == i[1]:
                # return f'{c} == {g[0]}'
                expr.append(f'{j} == {int(self.scale * i[0])}')  # <- Make integer so that UPPAAL can handle it
            elif i[0] is None:
                expr.append(f'{j} <= {int(self.scale * i[1])}')
            elif i[1] is None:
                expr.append(f'{int(self.scale * i[0])} <= {j}')
            else:
                # return f'{g[0]} <= {c} && {c} <= {g[1]}'
                expr.append(f'{int(self.scale * i[0])} <= {j}')  # <- Make integer so that UPPAAL can handle it
                expr.append(f'{j} <= {int(self.scale * i[1])}')  # <- Make integer so that UPPAAL can handle it

        return ' && '.join(expr)

    @classmethod
    def from_bytestream_file(cls, file_name):
        if not os.path.isfile(file_name):
            file_retry = os.path.join(save_path, file_name)
            if not os.path.isfile(file_retry):
                print("Please specify a valid file.")
                return None
            else:
                file_name = file_retry

        with open(file_name, 'rb') as f:
            obj = pickle.load(f)

        return obj

    def convert_to_tga(self, uncontrolled_actions):
        """
        Converts the timed automaton to a timed game automaton where some actions (and thus also
        some transitions) are uncontrollable.
        Requirements:
        The parameters do not `overlap' and the union of both equals the total action set
        """
        if type(uncontrolled_actions) is not set:
            uncontrolled_actions = set(uncontrolled_actions)

        if not self.actions.issuperset(uncontrolled_actions):
            return None

        c_actions = self.actions.difference(uncontrolled_actions)
        new_tr = []
        # convert the transitions to format (i, g, a_c, a_u, c, t)
        for (i, g, a, c, t) in self.transitions:
            if a in uncontrolled_actions:
                new_tr.append((i, g, frozenset(), a, c, t))
            else:
                new_tr.append((i, g, a, frozenset(), c, t))

        from . import TimedGameAutomaton
        return TimedGameAutomaton(self.name, self.locations, self.invariants, self.actions,
                                  c_actions, uncontrolled_actions, self.clocks, new_tr, self.initial_location)

    # def convert_to_pta(self, transition_prices, location_prices):
    #     """
    #     Converts the TA into a PTA, with prices specified in the two arguments.
    #     If a transitions/location is not given in the list, it will be given price 1.
    #     @param transition_prices:
    #     @param location_prices:
    #     """

    #     from .priced_timed_automaton import PricedTimedAutomaton
    #     raise NotImplementedError
    #     templp = location_prices.copy()
    #     temptp = transition_prices.copy()
    #     for x in self.locations:
    #         if x not in location_prices:
    #             print(f'Location {x} not specified in location_prices. Give it price 0.')
    #             templp[x] = 0

    #     for i, x in enumerate(self.transitions):
    #         if i not in transition_prices:
    #             print(f'Transition {x} not specified in transition_prices. Give it price 0.')
    #             temptp[x] = 0


if __name__ == '__main__':
    a = TimedAutomaton([], [], [], ['c'], [], [])
    g = (2, 3)
    print(a._interval2str(g))
    g = [(1, 2), (3, 4)]
    print(a._interval2str(g, ['a', 'b']))
