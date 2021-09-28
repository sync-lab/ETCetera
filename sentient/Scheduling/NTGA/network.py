import shortuuid

from sentient.Systems.Automata import TimedGameAutomaton
from sentient import pyuppaal

class network(TimedGameAutomaton):
    """
    Class for creating a TGA representing the common communication network's behaviour.
    The difference from M.Mazo, D.Adzkiya 2016 is that the clock has been removed 
    from the network TGA altogether in this model.

    Original author: P. Schalkwijk 
    """

    def __init__(self, name='Network', index=None, sync='up',
                 ack='ack', nack='nack', timeout='timeout', down='down'):

        self.name = name
        self.index = index or shortuuid.uuid()[:6]
        self.sync = sync
        self.ack = ack
        self.nack = nack
        self.timeout = timeout
        self.down = down

        # all locations in a network are urgent
        clocks = {}  # no clock involved in the network



        # non-urgent locations
        locations = {'Idle', 'InUse', 'Bad'}

        # Add urgent locations
        urgent = {'InUse_ack', 'InUse_nack'}
        locations.update(urgent)

        # no invariants since all transitions are ad-hoc, decided by control loops
        invariants = {loc: None for loc in locations}

        # All actions
        actions_u = {f'{sync}?'}  # TODO : A better way to do this?
        actions_c = {f'{timeout}?', f'{ack}!', f'{nack}!'}

        # Define edges
        tranitions = self._create_all_edges()

        l0 = 'Idle'

        super().__init__(locations, invariants, actions_c, actions_u,
                         clocks, tranitions, name=name, initial_location=l0,
                         urgent=urgent)

    def _create_all_edges(self):

        # Any edge = (source, guard, actions_c, actions_u, resets, target)

        edges = set()

        action_sync = {f'{self.sync}?'}
        action_ack = {f'{self.ack}!'}
        action_nack = {f'{self.nack}!'}
        action_timeout = {f'{self.timeout}?'}
        action_down = {f'{self.down}?'}

        edge_idle_to_inuse_ack = {('Idle', True, frozenset(),
                                   frozenset(action_sync), frozenset(), 'InUse_ack')}
        edges.update(edge_idle_to_inuse_ack)

        edge_from_inuse_ack = {('InUse_ack', True, frozenset(action_ack),
                                frozenset(), frozenset(), 'InUse')}
        edges.update(edge_from_inuse_ack)

        edge_from_inuse_nack = {('InUse_nack', True, frozenset(action_nack),
                                 frozenset(), frozenset(), 'InUse')}
        edges.update(edge_from_inuse_nack)

        edge_inuse_to_inuse_nack = {('InUse', True, frozenset(),
                                     frozenset(action_sync), frozenset(), 'InUse_nack')}
        edges.update(edge_inuse_to_inuse_nack)

        edge_to_bad = {('InUse', True, frozenset(action_timeout),
                        frozenset(), frozenset(), 'Bad')}
        edges.update(edge_to_bad)

        edge_to_idle = {('InUse', True, frozenset(action_down),
                         frozenset(), frozenset(), 'Idle')}
        edges.update(edge_to_idle)

        self_loop = {('Bad', True, frozenset(action_sync),
                      frozenset(), frozenset(), 'Bad')}
        edges.update(self_loop)

        return edges

    def _generate_transitions(self):
        transitions = []
        for (source, guard, actions_c, actions_u, resets, target) in self.transitions:
            props = {}
            if guard:
                props.update({'guard': str(guard).lower()
                              if type(guard) is bool else guard})
            if actions_u:
                props.update({'synchronisation': ','.join(actions_u)})
                props.update({'controllable': False})
            if resets:
                props.update({'assignment': ','.join(
                    [f'{clock}=0' for clock in resets])})
            if actions_c:
                props.update({'synchronisation': ','.join(actions_c)})
                props.update({'controllable': True})

            # props.update({'controllable': False})
            transitions.append(pyuppaal.Transition(source,
                                                   target,
                                                   **props))
        return transitions

    # overridden method
    def _generate_declarations(self):
        # no clock, only shared channels

        # The "chans" below is only for debugging!
        # chans = ', '.join([f'{self.sync}', f'{self.ack}',
        # f'{self.nack}', f'{self.timeout}', f'{self.down}'])
        #
        # return f'chan {chans};\n'

        return '\n'

