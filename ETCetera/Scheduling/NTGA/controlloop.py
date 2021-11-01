import logging
import sys

import shortuuid
import math

from ETCetera.Systems.Automata import TimedAutomaton, TimedGameAutomaton
from ETCetera.Abstractions import Abstraction, TrafficModelLinearPETC, TrafficModelNonlinearETC
from ETCetera import pyuppaal


class controlloop(TimedGameAutomaton):
    """
    A Control Loop class to generate a TGA for the control loop
    model developed for the thesis of A. Samant:
    https://repository.tudelft.nl/islandora/object/uuid%3A2dccaa3b-dbff-428e-a5d3-d46ada57504d


    Original authors:
    P. Schalkwijk: https://github.com/pschalkwijk/Python2Uppaal
    A. Samant: https://github.com/asamant/masters-thesis-sched-strat

    The class has been modified such that it is compatible with the TimedGameAutomaton class, as well as
    being able to directly construct from a trafficmodel object.
    """

    # constants
    to_region_decl = 'to_region'
    from_region_decl = 'from_region'
    count_decl = 'count'
    earnum_decl = 'EarNum'

    def __init__(self, abstraction: Abstraction, name=None, index=None,initial_location=None, max_delay_steps=0,
                 max_early=None, max_early_triggers=0, delta=None,
                 sync='up', nack='nack', ack='ack', timeout='timeout', down='down', clock='c'):

        self.name = name or 'controlloop' + shortuuid.uuid()[:6]

        # communicating actions
        self.sync = sync
        self.nack = nack
        self.ack = ack
        self.down = down
        self.timeout = timeout
        self.max_early_triggers = max_early_triggers
        self.abstraction = abstraction

        if delta is None:
            if type(abstraction) == TrafficModelLinearPETC:
                self.delta = abstraction.trigger.h
            else:
                self.delta = 0.01
        else:
            self.delta = delta
        self._clock = clock

        self.index = index or shortuuid.uuid()[:6]
        self.max_delay_steps = max_delay_steps

        maxdefault = 0
        if type(abstraction) == TrafficModelLinearPETC:
            maxdefault = 2
        else:
            maxdefault = 0.1
        self.max_early = max_early if max_early is not None else maxdefault

        if type(abstraction) is TrafficModelLinearPETC:
            self._constr_from_linearpetc(abstraction, initial_location)
            # Set scale
            self.scale = math.ceil(1/abstraction.trigger.h)
        elif type(abstraction) is TrafficModelNonlinearETC:
            self._constr_from_nonlinearetc(abstraction, initial_location)
            # Find the scale ...
            l = set()
            # l.update(1/i for i in self.invariants.values())
            l.update(1/r.timing_lower_bound for r in abstraction.Regions)
            l.update(1 / r.timing_upper_bound for r in abstraction.Regions)
            # print(f'l: {l}')
            self.scale = int(max(l))
            logging.info(f'Set {self.name} scale to: {self.scale}')
        else:
            print("Currently can only use Linear PETC and Nonlinear ETC abstractions.")
            raise NotImplementedError



    def _constr_from_linearpetc(self, abstraction: Abstraction, initial_location=None):

        # Check for refinement -> If refine return also error
        # TODO: Make this possible
        # if any([len(loc) > 1 for loc in abstraction.regions]):
        #     print("Currently cannot use refined locations")
        #     raise NotImplementedError

        h = abstraction.trigger.h
        self.h = h
        self.kmax = abstraction.trigger.kmax
        self.tau_max = h*self.kmax

        locations = {'R' + '_'.join([str(i) for i in loc]) for loc in abstraction.regions}
        # locations = {f'R{loc[0]}' for loc in abstraction.regions}
        self.loc_dict = {x[1:]: i for (i, x) in enumerate(locations)}

        # Add early trigger locations
        # early_loc = {f'Ear{loc[0]}' for loc in abstraction.regions}
        early_loc = {'Ear' + '_'.join([str(i) for i in loc]) for loc in abstraction.regions}
        locations.update(early_loc)
        urgent = early_loc.copy()

        # Add other locations
        common_locations = {'Trans_loc', 'Clk_wait', 'Bad'}
        locations.update(common_locations)
        urgent.update({'Trans_loc'})

        actions_u = set(f'{self.sync}?')
        actions_c = {f'{self.ack}?', f'{self.nack}?', f'{self.down}!', f'{self.timeout}!', '*'}
        self.clocks = frozenset({f'{self._clock}'})

        # Invariants
        # invariants = self._transitions_to_invariants(abstraction.transitions, h)
        # invariants = {f'R{loc[0]}': inv for (loc, inv) in invariants.items()}
        invariants = {f'R' + '_'.join([str(i) for i in loc]): (None, loc[0]*h) for loc in abstraction.regions}
        invariants.update({i: None for i in early_loc})
        invariants.update({i: None for i in common_locations})

        # Initiate with empty transition set
        super().__init__(locations, invariants, actions_c, actions_u, self.clocks, {},
                         name=self.name, initial_location=initial_location, urgent=urgent)

        # Create the transitions. Note: By convention, variable assignments are put in
        # the controlled_actions field

        # print(locations)
        # print(early_loc)
        # print(invariants)
        # print(loc_dict)


        edge_map = {}
        for (start, step), targets in abstraction.transitions.items():
            for target in targets:
                if (start, target) in edge_map:
                    edge_map[(start, target)].append(step*h)
                    edge_map[(start, target)].sort()
                else:
                    edge_map[(start, target)] = [step*h]

        edges = set()
        for (start, end), value in edge_map.items():
            intervals = [(g, g + self.max_delay_steps) for g in value]
            # edges.update({(start[0], guard, frozenset(), self.clocks, end[0])
            #               for i in intervals
            #               for guard in self._interval_to_guard(i)})
            edges.update({(start, i, frozenset(), self.clocks, end)
                          for i in intervals})

        # Create transitions from 'R_i' and 'Ear_i' to 'Trans_loc'
        transitions = set()
        reset = frozenset()
        for (start, guard, assignment, clocks, end) in edges:
            # If late => continue
            if guard[0] > h*start[0] or guard[0] < h*start[0]-self.max_early:
                continue

            loc_t = '_'.join([str(i) for i in end])

            # Add transition from Ear_i if early trigger
            if guard[0] < h*start[0]:
                # now ear_R to trans_state
                ea = set(assignment)
                ea.update({f"{self.to_region_decl} = {self.loc_dict[loc_t]}"})
                ea.update({f'{self.earnum_decl} = {self.earnum_decl} + 1'})
                early_assignment = frozenset(ea)
                edge_earl = (f'Ear' + '_'.join([str(i) for i in start]), True,
                         early_assignment, frozenset({f'{self.sync}!'}),
                         reset, 'Trans_loc')
                transitions.update({edge_earl})
            # Else add transition from R_i
            else:
                # # first R to trans_state
                ia = set(assignment)
                ia.update({f'{self.earnum_decl} = 0'})

                # NOTE: the end should not have "R" before the region number
                ia.update({f"{self.to_region_decl} = {self.loc_dict[loc_t]}"})
                natural_assignment = frozenset(ia)
                edge_nat = (f'R' + '_'.join([str(i) for i in start]), guard, natural_assignment,
                                        frozenset({f'{self.sync}!'}), reset, 'Trans_loc')
                transitions.update({edge_nat})

        for location in abstraction.regions:
            # assumption: location = number of steps
            loc = '_'.join([str(i) for i in location])
            transitions.update([(f'R{loc}', frozenset({(location[0]*h-self.max_early*h, location[0]*h),
                                                               f'{self.earnum_decl} <= {self.max_early_triggers}'}), #f'{self._clock} < {location[0]} &&'
                                                    #f'{location[0] - self.max_early} <= {self._clock} && {self.earnum_decl} <= EarMax',
                                 False, False, frozenset(), f'Ear{loc}')])

            # NACK-edges
            loc_s = f'R{loc}'
            guard = f'{self.from_region_decl} == {self.loc_dict[loc]} &&' \
                    f'{self.count_decl} < {self.max_delay_steps}'

            # update the number of retries
            ca = set()
            ca.update({f"{self.count_decl} = {self.count_decl} + 1"})

            ca.update({f"{self.nack}?"})

            controllable_action = frozenset(ca)
            transitions.update({('Trans_loc', guard,
                                 controllable_action, frozenset(), frozenset(), loc_s)})

            # Communication-delay edges
            # guard = f'{self.to_region_decl} == {location[0]} && {self.delta} <= {self._clock}'
            guard = frozenset({(self.delta, None), f'{self.to_region_decl} == {self.loc_dict[loc]}'})
            action_c = frozenset({f'{self.down}!'})
            resets = frozenset(self.clocks)
            edge = ('Clk_wait', guard, action_c,
                    frozenset(), resets, f'R{loc}')
            transitions.update({edge})

        # ACK-edge
        action_c = set({f'{self.ack}?'})
        action_c.update({f'{self.from_region_decl} = {self.to_region_decl}'})
        action_c.update({f'{self.count_decl} = 0'})

        resets = frozenset(self.clocks)

        transitions.update({('Trans_loc', True, frozenset(action_c),
                             frozenset(), resets, 'Clk_wait')})

        # Initial Location
        if initial_location is None:

            self.locations.add(f'R0')
            transitions.update({(f'R0', True,
                                 frozenset({f'{self.from_region_decl} = {self.loc_dict[(loc := "_".join([str(i) for i in location]))]}'}),
                                 frozenset(), frozenset(),
                                 f'R{loc}') for location in abstraction.regions})
        else:
            if type(initial_location) is not set:
                initial_location = set({initial_location})
            self.locations.add(f'R0')
            transitions.update({(f'R0', True,
                                 frozenset({f'{self.from_region_decl} = {self.loc_dict[(loc := "_".join([str(i) for i in location]))]}'}),
                                 frozenset(), frozenset(),
                                 f'R{loc}') for location in initial_location})

        self.urgent.update({"R0"})
        self.invariants.update({'R0': None})
        self.initial_location = 'R0'
        self.transitions = transitions
        self.loc_dict_inv = {v:k for (k,v) in self.loc_dict.items()}
        self.h = h

    def _constr_from_nonlinearetc(self, abstraction: Abstraction, initial_location=None):

        locations = {f'R{i}' if type(i := eval(r)) is not list else f'R{i[0]}_{i[1]}'
                     for r in abstraction.regions}
        # print(locations)
        urgent = set()

        self.tau_max = abstraction.heartbeat

        # If regions.index is [a,b]: represent regions by single number (only for assignments)
        # self.loc_dict = {r[1:]: i for (r,i) in zip(locations, range(0, len(locations)))}
        self.loc_dict = {r[1:]: i for (i, r) in enumerate(locations)}

        # Add other locations
        common_locations = {'Trans_loc', 'Clk_wait', 'Bad'}
        locations.update(common_locations)
        urgent.update({'Trans_loc'})

        actions_u = set(f'{self.sync}?')
        actions_c = {f'{self.ack}?', f'{self.nack}?', f'{self.down}!', f'{self.timeout}!', '*'}
        self.clocks = frozenset({f'{self._clock}'})

        invariants = {f'R{i}' if type(i := eval(r)) is not list else f'R{i[0]}_{i[1]}':
                          # f'{self._clock} <= {r.timing_upper_bound}'
                          (None, t)
                      for (r, t) in abstraction.regions.items()}

        invariants.update({i: None for i in common_locations})
        # print(invariants)

        if self.max_early > 0:
            early_loc = {f'Ear{i}' if type(i := eval(r)) is not list else f'Ear{i[0]}_{i[1]}'
                         for r in abstraction.regions}
            locations.update(early_loc)
            urgent.update(early_loc.copy())
            invariants.update({i: None for i in early_loc})

        # Initiate with empty transition set
        super().__init__(locations, invariants, actions_c, actions_u, self.clocks, {},
                         name=self.name, initial_location=initial_location, urgent=urgent)

        transitions = set()
        reset = frozenset()
        # TODO: Transitions from Ear_i should ideally come from seperate reach. analysis
        for ((r, tau), post) in abstraction.transitions.items():
            loc_r = f'{i}' if type(i := eval(r)) is not list else f'{i[0]}_{i[1]}'
            # if i == [2,1]:
            #     print(loc_r)
            #     print(r.transitions)
            for t in post:
                loc_t = f'{t}' if type(t) is not list else f'{t[0]}_{t[1]}'


                ia = set()
                ia.update({f'{self.earnum_decl} = 0'})

                # NOTE: the end should not have "R" before the region number
                ia.update({f"{self.to_region_decl} = {self.loc_dict[loc_t]}"})
                natural_assignment = frozenset(ia)



                # add edges
                guard_nat = tau#(r.timing_lower_bound, r.timing_upper_bound)#f'{r.timing_lower_bound} <= c && c <= {r.timing_upper_bound}'
                edge_nat = (f'R{loc_r}', guard_nat, natural_assignment,
                            frozenset({f'{self.sync}!'}), reset, 'Trans_loc')
                # if loc_r == '2_1':
                #     print(edge_nat)
                transitions.update({edge_nat})

                if self.max_early > 0:
                    ea = set()
                    # NOTE: Ideally the "end" value should come from a separate
                    # reachability analysis for early edges
                    ea.update({f"{self.to_region_decl} = {self.loc_dict[loc_t]}"})
                    ea.update({f'{self.earnum_decl} = {self.earnum_decl} + 1'})
                    early_assignment = frozenset(ea)
                    # Early edges should not have any guard
                    edge_earl = (f'Ear{loc_r}', True,
                                 early_assignment, frozenset({f'{self.sync}!'}),
                                 reset, 'Trans_loc')


                    transitions.update({edge_earl})

            if self.max_early > 0:
                transitions.update([(f'R{loc_r}', frozenset({(tau[0] - self.max_early, tau[0]),
                                                   f'{self.earnum_decl} <= {self.max_early_triggers}'}),
                                    #  f'{self._clock} < {r.timing_lower_bound} &&'
                                    # f'{r.timing_lower_bound - self.max_early} <= {self._clock} &&'
                                    #  f'{self.earnum_decl} <= EarMax',
                                     False, False, frozenset(), f'Ear{loc_r}')])

            # NACK-edges
            guard = f'{self.from_region_decl} == {self.loc_dict[loc_r]} &&' \
                    f'{self.count_decl} < {self.max_delay_steps}'

            # update the number of retries
            ca = set()
            ca.update({f"{self.count_decl} = {self.count_decl} + 1"})

            ca.update({f"{self.nack}?"})

            controllable_action = frozenset(ca)
            transitions.update({('Trans_loc', guard,
                                 controllable_action, frozenset(), frozenset(), f'R{loc_r}')})

            # Communication-delay edges
            guard = frozenset({f'{self.to_region_decl} == {self.loc_dict[loc_r]}', (None,self.delta)})

            action_c = frozenset({f'{self.down}!'})
            resets = frozenset(self.clocks)
            edge = ('Clk_wait', guard, action_c,
                    frozenset(), resets, f'R{loc_r}')
            transitions.update({edge})

        # ACK-edge
        action_c = set({f'{self.ack}?'})
        action_c.update({f'{self.from_region_decl} = {self.to_region_decl}'})
        action_c.update({f'{self.count_decl} = 0'})

        resets = frozenset(self.clocks)

        transitions.update({('Trans_loc', True, frozenset(action_c),
                             frozenset(), resets, 'Clk_wait')})

        # Initial Location
        if initial_location is None:

            self.locations.add(f'R0')
            transitions.update({(f'R0', True,
                                 frozenset({f'{self.from_region_decl} = {self.loc_dict[str(i)]}'}) if type(i := eval(location)) is not list
                                 else frozenset({f'{self.from_region_decl} = {self.loc_dict[f"{i[0]}_{i[1]}"]}'}),
                                 frozenset(), frozenset(),
                                 f'R{i}' if type(i := eval(location)) is not list else f'R{i[0]}_{i[1]}') for location in abstraction.regions})
        else:
            self.locations.add(f'R0')
            transitions.update({(f'R0', True,
                                 frozenset({f'{self.from_region_decl} = {self.loc_dict[str(i)]}'}) if type(i := eval(location)) is not list
                                 else frozenset({f'{self.from_region_decl} = {self.loc_dict[f"{i[0]}_{i[1]}"]}'}),
                                 frozenset(), frozenset(),
                                 f'R{i}' if type(i := eval(location)) is not list else f'R{i[0]}_{i[1]}') for location in initial_location})

        self.urgent.update({"R0"})
        self.invariants.update({'R0': None})
        self.initial_location = 'R0'
        self.transitions = transitions
        self.loc_dict_inv = {v: k for (k, v) in self.loc_dict.items()}
        # print(self.loc_dict_inv)
        # print(transitions)
        # sys.exit()
        # pass

    def _interval_to_guard(self, interval):
        """
                Convert an interval to a guard
                :param interval: tuple
                :return: string
                """
        assert type(interval) is tuple
        assert len(interval) == 2
        lower, upper = interval
        guard = set()  # FIXME: should this be a set, a single evaluation or a string?
        if lower == 0:
            if lower == upper:
                lower = upper = 1
            else:
                lower = 1
        if lower == upper:
            for clock in self.clocks:
                guard.add(f'{clock}=={lower}')
        else:
            for clock in self.clocks:
                guard.add(f'{clock}>={lower} && {clock}<={upper}')
        return guard

    def _transitions_to_invariants(self, transitions, h):
        """
                Create the mapping of invariants to locations.
                Each location is upper bounded by the final
                time step found in the transition table
                :param transitions: dict
                :param h: sampling time
                :return: dict
                """
        upper_bound = {}
        for (start, step) in transitions.keys():
            if step > start[0]:
                # Ignore late transitions
                continue
            upper_bound[start] = max(upper_bound.get(start, 0), step)
        # FIXME: variable set of clocks? immutable, hashable table instead of dict?
        assert (len(self.clocks) == 1)
        # for clock in self.clocks:  # Which will be only one
        invariants = {location: (None, final_step*h) for#f"{clock}<={final_step}" for
                      location, final_step in upper_bound.items()}
        return invariants

    def _generate_declarations(self):
        clock_decl = f"clock {', '.join(self.clocks)};"

        # variables needed for the new automata model
        ints_decl = f"int {', '.join([self.to_region_decl, self.from_region_decl, self.count_decl])};"

        # include the max num of retries
        n_max = self.max_delay_steps

        # temp_decl = f'int {self.earnum_decl};' + '\n' + f'const int EarMax = 4;'
        #
        # # Synchronization channels
        # sync = ["up", "down", "timeout", "ack", "nack"]
        # temp_decl += f'chan {", ".join(sync)};\n'

        return clock_decl + '\n' + ints_decl + '\n' + f"const int n_max = {n_max};" + '\n'  #+ temp_decl + '\n'

    def _generate_transitions(self):
        transitions = []
        for (source, guard, actions_c, actions_u, resets, target) in self.transitions:
            props = {}
            if guard:
                # props.update({'guard': str(guard).lower()
                # if type(guard) is bool else guard})
                props.update({'guard': self._interval2str(guard)})
            if actions_u:
                props.update({'synchronisation': '\n'.join(actions_u)})
                props.update({'controllable': False})
            if resets is not frozenset():
                # props.update({'assignment': ', '.join(
                #     [f'{clock}=0' for clock in resets])})
                temp = ', '.join([f'{clock} = 0' for clock in resets])
                props.update({'assignment': temp})
            if actions_c:
                clock_assignments = props.get('assignment', False)
                action_assignments = ''
                for action in actions_c:
                    # sync actions
                    if (("?" in action) or ("!" in action)):
                        # expected to have only one sync action per edge
                        props.update({'synchronisation': action})
                        continue
                    # end
                    # action_assignments = ', '.join(
                    #     [action_assignments, action])
                action_assignments = ', '.join([act for act in actions_c if act not in self.controlled_actions])
                # endfor
                if action_assignments != '':
                    if clock_assignments:
                        action_assignments = ', '.join(
                            [clock_assignments, action_assignments])

                    props.update({'assignment': action_assignments})
            if target not in self.locations:
                print(f'{target} is not found in set of locations')
            else:
                transitions.append(
                    pyuppaal.Transition(source, target, **props))
        return transitions
