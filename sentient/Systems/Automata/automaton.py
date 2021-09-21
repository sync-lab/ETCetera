from typing import Dict, Set, Tuple, NewType, List, Union

State = NewType('State', int)
Action = NewType('Action', str)
Output = NewType('Output', str)


class Automaton:
    """
    Class Representing a Automaton/Transitions system S = (X, X_0, U, ->, Y, H)

    Transition are either represented as a list ([(i,u,t), .. ]) or dict ({i: {u1: {t1, .. }, u2: {t1, ...}, ...}, ...})
    """

    locations: Union[Set[State], Dict[State, int]]
    initial_locations: Set[State]
    actions: Set[Action]
    outputs: Set[Output]
    transitions: Union[Dict[State, Dict[Action, Set[State]]], List[Tuple[State, Action, State]]]
    output_map: Dict[State, Output]

    def __init__(self, locations, action_set, transitions, initial_locations=None, outputs=None, output_map=None):
        self.locations = locations
        self.actions = action_set
        self.transitions = transitions
        self.initial_locations = initial_locations or locations
        self.outputs = outputs or {}
        self.output_map = output_map or {}
