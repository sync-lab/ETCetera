from typing import Dict, Set, Tuple, NewType
import os, pickle
from . import pyuppaal

from config import save_path

State = NewType('State', int)
Action = NewType('Action', str)
Clock = NewType('Action', str)
ClockConstraint = NewType('Constraint', str)

class TimedAutomaton:
    """
    Class representing a timed automaton.
    """
    locations: Set[State]
    initial_location: State
    guards: Set[ClockConstraint]
    actions: Set[Action]
    invariants: Dict[State, Set[ClockConstraint]]
    clocks: Set[Clock]
    transitions: Set[Tuple[State, ClockConstraint, Action, Clock, State]]

    # Reference to Abstraction from which this object is created
    reference: object

    def __init__(self, locations, invariants, guards, actions, clocks, transitions, initial_location=None,
                 abstraction=None):
        self.locations = locations
        self.invariants = invariants
        self.initial_location = initial_location
        self.guards = guards
        self.actions = actions
        self.clocks = clocks
        self.transitions = transitions
        self.abstraction = abstraction

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
            self.__to_xml(file_name)
        elif export_type in ['pickle', 'bytes', 'byte_stream']:
            self.__export_pickle(file_name)

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

    def __export_pickle(self, file_name: str):
        if not file_name.endswith('.pickle'):
            file_name += '.pickle'

        with open(os.path.join(save_path, file_name), 'wb') as f:
            pickle.dump(self, f, pickle.HIGHEST_PROTOCOL)

    # TODO: Add option to either export for UPPAAL or UPPAAL Stratego
    #       difference is the version number
    def __to_xml(self, file_name: str):
        if not file_name.endswith('.xml'):
            file_name += '.xml'

        locations = [pyuppaal.Location(invariant=self.invariants[loc], name=loc) for loc in self.locations]

        transitions = [pyuppaal.Transition(src, trg, guard=guard, assignment='c=0') for (src, guard, _, _, trg) in self.transitions]
        declarations = f"clock {' ,'.join(self.clocks)};"
        template = pyuppaal.Template(name=self.__class__.__name__, declaration=declarations,
                                     locations=locations,
                                     transitions=transitions)
        template.initlocation = template.get_location_by_name(self.initial_location)
        template.layout(auto_nails=True)
        import html
        system = "CL = NetworkModel();\n system CL;"
        outputs = """<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE nta PUBLIC "-//Uppaal Team//DTD Flat System 1.1//EN" "http://www.it.uu.se/research/group/darts/uppaal/flat-1_1.dtd">
<nta>
  <declaration>%s</declaration>
  %s
  <system>%s</system>
</nta>""" % (html.escape(declarations), template.to_xml(), html.escape(system))

        with open(os.path.join(save_path, file_name), 'w') as f:
            f.write(outputs)

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

    def convert_to_tga(self, controlled_actions: Set[Action], uncontrolled_actions: Set[Action]):
        """
        Converts the timed automaton to a timed game automaton where some actions (and thus also
        some transitions) are uncontrollable.
        Requirements:
        The parameters do not `overlap' and the union of both equals the total action set
        """
        if type(controlled_actions) is not set:
            controlled_actions = set(controlled_actions)

        if type(uncontrolled_actions) is not set:
            uncontrolled_actions = set(uncontrolled_actions)


        if not controlled_actions.isdisjoint(uncontrolled_actions) or \
                not controlled_actions.union(uncontrolled_actions) == self.actions:
            return None
        raise NotImplementedError
