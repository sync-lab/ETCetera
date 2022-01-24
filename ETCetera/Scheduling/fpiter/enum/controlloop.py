import logging
import itertools
from functools import cached_property

from ETCetera.Abstractions import TrafficModelLinearPETC
from ETCetera.Systems.Automata.Reduction.altsim \
    import minimize_alternating_simulation_equivalence, stats


class controlloop:

    # def __new__(cls, abstraction: TrafficModelLinearPETC, use_bdd:bool=True, maxLate: int=None, maxLateStates: int = None, ratio: int = 1):
    #     # Some logic to create the correct object type
    #     if

    def __init__(self, abstraction: TrafficModelLinearPETC, maxLate: int = None,
                 maxLateStates: int = None, ratio: int = 1, label_split_T=True,
                 init_steps: int = None):

        if 'TrafficModelLinearPETC' not in str(type(abstraction)):
            print("Can currently only construct from 'TrafficModelLinearPETC' objects.")
            raise NotImplementedError

        # if any([len(loc) > 1 for loc in abstraction.regions]):
        #     print("Currently cannot use refined locations")
        #     raise NotImplementedError

        self.h = abstraction.trigger.h
        self.kmax = abstraction.kmax
        self._label_split = True
        self.abstraction = abstraction # Store (reference) for later use
        # Convert the abstraction traffic model into different form
        states = dict()

        output_map = dict()
        if not label_split_T:
            outputs = {}
            ny = 0
        else:
            if (1,) in abstraction.regions:
                outputs = {'T1': 0, 'T': 1}
                ny = 2
            else:
                outputs = {'T': 0}
                ny = 1

        self.actions = {'w': 0, 't': 1}
        transitions = dict()
        n = 0
        # ny = 2

        def add_state(x, y):
            nonlocal n, ny, states, transitions, outputs, output_map
            if x not in states:
                states.update({x: n})
                transitions.update({x: {u: set() for u in self.actions}})
                n += 1
            if y not in outputs:
                outputs.update({y: ny})
                ny += 1
            output_map.update({x: y})

        real_max_late = 0

        # Detect for late trigger
        if maxLate is not None:
            self.maxLate = maxLate
            self.maxLateStates = maxLateStates or max([i for (i,) in abstraction.regions])
            self.ratio = ratio
            self.actions = {'w': 0, 't': 1, 'lw': 2}
            real_max_late = min(self.maxLate, self.maxLateStates)
            self.aux_ts = self._create_aux_system(self.maxLate, self.ratio)

            self._is_late = True

        # for (i, g, a, c, t) in tsa[4]:
        for (i, u), targets in abstraction.transitions.items():
            loc_i = '_'.join([str(l) for l in i])
            # (i,) = i
            i = i[0]
            if u > i + real_max_late:  # Do not use `late' triggers
                continue
            # x = 'T' + str(i)
            x = f'T{loc_i}'
            if not label_split_T:
                add_state(x, str(i))
            else:
                if i == 1:
                    if label_split_T:
                        add_state(x, 'T1')
                else:
                    add_state(x, 'T')

            for t in targets:
                loc_t = '_'.join([str(l) for l in t])
                # (t,) = t
                t = t[0]
                #y = 'T' + str(t)
                y = f'T{loc_t}'
                if not label_split_T:
                    add_state(x, str(i))
                else:
                    if t == 1:
                        add_state(y, 'T1')
                    else:
                        add_state(y, 'T')

                if u == 1:
                    # transitions.add(('T{q}'.format(q=i), 't', 'T{q}'.format(q=t)))
                    transitions[x]['t'].add(y)
                else:

                    # W = 'W{p},{q}'.format(p=i, q=1)
                    # out = 'W{i}'.format(i=i - 1)
                    W = f'W{loc_i},{1}'
                    out = f'{i-1}'

                    add_state(W, out)

                    # transitions.add(('T{q}'.format(q=i), 'w', 'W{p},{q}'.format(p=i, q=1)))
                    if i == 1:
                        transitions[x]['lw'].add(W)
                    else:
                        transitions[x]['w'].add(W)

                    for j in range(2, u):
                        # W = 'W{p},{q}'.format(p=i, q=j)
                        # out = 'W{p}'.format(p=i - j)
                        W = f'W{loc_i},{j}'#.format(p=i, q=j)
                        out = f'{i-j}'#.format(p=i - j)
                        add_state(W, out)
                        try:
                            if i <= j:
                                # transitions['W{p},{q}'.format(p=i, q=j - 1)]['lw'].add(W)
                                transitions[f'W{loc_i},{j-1}']['lw'].add(W)
                            else:
                                # transitions['W{p},{q}'.format(p=i, q=j - 1)]['w'].add(W)
                                transitions[f'W{loc_i},{j-1}']['w'].add(W)
                        except:
                            print(f'State W{loc_i},{j-1} not in transitions?????')

                    # transitions['W{p},{q}'.format(p=i, q=u - 1)]['t'].add(y)
                    try:
                        transitions[f'W{loc_i},{u-1}']['t'].add(y)
                    except:
                        print(f'State W{loc_i},{u-1} not in transitions?????')
                        # print(states)

        # Work out initialization
        if init_steps is None:
            init_steps = min(abstraction.K)
        self._init_steps = init_steps

        # Create init_steps states 'WI0, WI1, ...' to represent initialization.
        # From each initial step state you can have the first sample (which can
        # go to any T region) or wait (going to the next initial step state).
        for i in range(init_steps):
            x = f'WI{i}'
            add_state(x, 'W')
            if i > 0:
                transitions[f'WI{i-1}']['w'].add(x)
            for xp in states:
                if xp[0] == 'T':
                    transitions[f'WI{i}']['t'].add(xp)

        # Now create initial state set
        if init_steps == 0:
            initial = set()  # No specified initial state
        else:
            initial = {'WI0'}

        # super().__init__(states, actions, transitions, outputs=outputs, output_map=output_map)
        self._states = states
        # self.actions = actions
        self._transitions = transitions
        self._outputs = outputs
        self._output_map = output_map
        self._initial = initial

        # Attribute to store partitioned version
        self._states_part = states
        self._transitions_part = transitions
        self._output_map_part = output_map

        # Boolean used to store whether the system is partitioned
        self._is_part = False

    """ Getter Properties """
    @cached_property
    def states(self):
        # use_states = {}
        if not self._is_part:
            use_states = self._states
        else:
            use_states = self._states_part

        if hasattr(self, '_is_late'):
            return self.__c_dict(use_states, self._states_aux)

        return use_states

    @cached_property
    def transitions(self):
        if not self._is_part:
            use_tr = self._transitions
        else:
            use_tr = self._transitions_part

        if hasattr(self, '_is_late'):
            transitions = {x: {u: set() for u in self.actions} for x in self.states}

            for xx in self.states:
                for u in self.actions:
                    s = [use_tr[xx[0]][u], self._transitions_aux[xx[1]][u]]
                    ls = set(itertools.product(*s))
                    transitions[xx][u].update(ls)

            return transitions

        return use_tr

    @cached_property
    def output_map(self):
        if not self._is_part:
            use_H = self._output_map
        else:
            use_H = self._output_map_part

        if hasattr(self, '_is_late'):
            return self.__c_dict(use_H, self._output_map_aux)

        return use_H

    @cached_property
    def outputs(self):
        if hasattr(self, '_is_late'):
            return self.__c_dict(self._outputs, self._outputs_aux)

        return self._outputs

    def _clear_cache(self):
        rem_attr = {'states', 'transitions', 'output_map', 'outputs'}
        for i in rem_attr:
            if i in self.__dict__:
                del self.__dict__[i]

    def __c_dict(self, x: dict, y: dict):
        a = list(itertools.product(x.keys(), y.keys()))
        b = list(itertools.product(x.values(), y.values()))
        res = {i: j for (i, j) in zip(a, b)}
        return res

    def restore(self):
        """ Restores partitioned system to original system. """
        self._is_part = False
        self._clear_cache()

    ''' Functions without bdd usage '''

    def pre(self, x: dict, u: dict = None, original: bool = False):
        """
        Calculates the set of previous states for given action(s) or all action if actions is not given
        :param x: state(s)
        :param u: action
        :return: Dict of previous states
        """
        r = dict()
        if u is None:
            u = self.actions

        if original:
            tr = self._transitions
        else:
            tr = self.transitions

        if original:
            states = self._states
        else:
            states = self.states

        for (z, Ux) in tr.items():
            for (uk, Post) in Ux.items():
                if uk in u:
                    for y in Post:
                        if y in x:
                            r.update({z: states[z]})

        return r

    def post(self, x: dict, u: dict = None, original: bool = False):
        """
        Calculates the set of next states for given action(s) or all actions if actions is not given.
        Important: Use only for multiple inputs and/or states,
        otherwise can use self.transitions[x][u] for Post_u(x)
        :param x: set of state(s)
        :param u: set of actions
        :return: set of next states
        """
        r = set()
        if type(x) not in {dict, set, list}:
            x = {x}

        if u is None:
            u = self.actions
        elif type(u) not in [dict, set, list]:
            u = {u}

        for i in x:
            for j in u:
                r.update(self.transitions[i][j])

        return r

    """ Main Partition/Refinement methods """

    # TODO: Detect whether the system can even be partitioned in the first place, if not: return self
    def create_initial_partition(self):
        """
        Creates a partition of the NFA, where the states
        with equal outputs are grouped together.
        :return: New NFA object corresponding to the partition
        """
        if self._is_part:
            print("System already partitioned")
            return False

        # Construct new set of states, output_map and initialized transitions
        states = {y: dict() for y in self._outputs}
        output_map = {y: y for y in self._outputs}
        transitions = {y: {u: set() for u in self.actions} for y in self._outputs}

        # Add states to the blocks/new states
        for (x, y) in self._output_map.items():
            states[y].update({x: self._states[x]})

        # Throw away empty blocks
        states = {s: k for (s, k) in states.items() if k != {}}

        for (x, Ux) in self._transitions.items():
            b = self._output_map[x]
            for (u, Post) in Ux.items():
                for y in Post:
                    c = self._output_map[y]
                    transitions[b][u].add(c)

        self._is_part = True
        self._states_part = states
        self._transitions_part = transitions
        self._output_map_part = output_map
        self._clear_cache()
        return True

    def refine(self, block=None):
        """
        Refines itself by means of the refinement operator.
        Also calculates new transitions is refinement successful
        :param block: Try refinement (only) with block, if None-> use all blocks for refinement
        :return: (bool)
        If refinement successful:
        nfa = refined nfa, bool = True
        Else: nfa = self, bool = False
        """
        if not self._is_part:
            print("Partition system before refining.")
            return False

        self._clear_cache()

        if block is not None:
            l = [(block, self._states_part[block])]
        else:
            l = list(self._states_part.items())

        success = False
        states = self._states_part.copy()

        for (B, x) in l:
            states = self.__refine_operator(states, {B: x})

        if states != self._states_part:
            success = True

            # Throw away empty blocks
            states = {s: k for (s, k) in states.items() if k != {}}

        if success:
            # Create output map
            output_map = dict()
            for (k, v) in states.items():
                try:
                    x = next(iter(v))
                    output_map.update({k: self._output_map[x]})
                except:
                    # raise Exception('Failed to update outputmap at {i}'.format(i=(k,v)))
                    # Meaning v was empty -> don't need to add to output map
                    pass

            # Create Transitions
            transitions = {y: {u: set() for u in self.actions} for y in states}

            # Invert block mapping
            invB = {x: k for (k, v) in states.items() for x in v}
            for (x, Ux) in self._transitions.items():
                b = invB[x]
                for (u, Post) in Ux.items():
                    for y in Post:
                        c = invB[y]
                        transitions[b][u].add(c)

            # return controlloop(states, self.actions, transitions, self.outputs, output_map), True
            self._states_part = states
            self._transitions_part = transitions
            self._output_map_part = output_map
            return True
        else:
            # print('Refinement no longer possible')
            return False

    def __refine_operator(self, states, C: dict):
        """
        Refine operator. Computes all the new blocks after refinement.
        :param C: Set of blocks (most of the time just single block)
        :return: New set of states
        """
        res = dict()
        for (B, x) in states.items():
            preC = dict()
            for block in C.values():
                preC.update(self.pre(block, original=True))

            B1 = dict()
            B2 = dict()
            for (k, v) in x.items():
                if k in preC and preC[k] == v:
                    B1.update({k: v})
                else:
                    B2.update({k: v})

            if len(B1) == 0 or len(B2) == 0:
                res.update({B: x})
            else:
                res.update({str(B) + "_1": B1, str(B) + "_2": B2})

        return res

    """ Auxiliary Methods """

    def check_similar(self, other: 'controlloop'):
        """
        Check whether self is simulated by other, i.e. S_self <= S_other
        @param other: Other controlloop
        @return: Solution, Boolean
        """
        Z = {(xa, xb) for xa in self.states for xb in other.states}
        Zold = set()
        while Z != Zold:
            Zold = Z.copy()
            Z = self._sim_operator(other, Z)

        return Z, (Z != set())

    def _sim_operator(self, other, Z):
        # Check all current elements of Z_i
        Z_new = set()
        for (xa, xb) in Z:
            try:
                assert (self.output_map[xa] == other.output_map[xb])
                assert (all([
                    any([(xa2, xb2) in Z for ub in other.actions for xb2 in other.transitions[xb][ub]])
                    for ua in self.actions for xa2 in self.transitions[xa][ua] if xa2 != set()
                ]))
            except AssertionError:
                pass
            else:
                Z_new.add((xa, xb))

        return Z_new




    def check_bisimilar(self, other: 'controlloop'):
        #
        Z = {(xa, xb) for xa in self.states for xb in other.states}
        Zold = set()
        while Z != Zold:
            Zold = Z.copy()
            Z = self._bisim_operator(other, Z)

        return Z, (Z != set())

    def _bisim_operator(self, other, Z):
        res = set()
        nres = set()
        for (xa, xb) in Z:
            try:
                assert (self.output_map[xa] == other.output_map[xb])
                assert (all([
                    any([(xa2, xb2) in Z for ub in other.actions for xb2 in other.transitions[xb][ub]])
                    for ua in self.actions for xa2 in self.transitions[xa][ua] if xa2 != set()
                ]))
                assert (all([
                    any([(xa2, xb2) in Z for ua in self.actions for xa2 in self.transitions[xa][ua]])
                    for ub in other.actions for xb2 in other.transitions[xb][ub] if xb2 != set()
                ]))
            except:
                nres.add((xa, xb))
            else:
                res.add((xa, xb))

        return res

    def refine_completely(self):
        """ Refines self as much as possible using the more efficient refinement stuff. """
        if not self._is_part:
            print("Partition system before refining.")
            return

        part = self.states.copy()
        part_old = {}

        num_its = 1
        while part != part_old:
            print(f'{num_its}')
            part_old = part.copy()

            for (Ck, Cv) in part_old.items():
                part = self.__refine_operator(part, {Ck: Cv})

            num_its += 1


        # Completely refined blocks
        self._states_part = part

        self._clear_cache()

        # Create output map
        output_map = dict()
        for (k, v) in self._states_part.items():
            x = next(iter(v))
            output_map.update({k: self._output_map[x]})

        # Create Transitions
        transitions = {y: {u: set() for u in self.actions} for y in self._states_part}

        # Invert block mapping
        invB = {x: k for (k, v) in self._states_part.items() for x in v}
        for (x, Ux) in self._transitions.items():
            b = invB[x]
            for (u, Post) in Ux.items():
                for y in Post:
                    c = invB[y]
                    transitions[b][u].add(c)
        self._transitions_part = transitions
        self._output_map_part = output_map
        # return controlloop(states, self.actions, transitions, outputs=self.outputs, output_map=output_map), True

    def _create_aux_system(self, maxLate, ratio):
        self._states_aux = {n: n for n in range(0, ratio * maxLate + 2)}
        self._outputs_aux = {'O': 0, 'X': 1}
        self._output_map_aux = {n: ('O' if n < ratio * maxLate + 1 else 'X') for n in range(0, ratio * maxLate + 2)}
        # self._actions_aux = {'w': 0, 't': 1, 'lw': 2, 'lt': 3}

        self._transitions_aux = {x: {u: set() for u in self.actions} for x in self._states_aux}

        for s in self._states_aux:
            if s < ratio * maxLate:
                self._transitions_aux[s]['lw'].add(s + ratio)
            else:
                self._transitions_aux[s]['lw'].add(ratio * maxLate + 1)
            if s > 0:
                self._transitions_aux[s]['w'].add(s - 1)
                self._transitions_aux[s]['t'].add(s - 1)
            else:
                self._transitions_aux[s]['w'].add(s)
                self._transitions_aux[s]['t'].add(s)

    def reduce_altsim(self):
        S = self.transitions
        H = self.output_map
        H = {x:x[0] for x,y in H.items()}
        if len(self._initial) == 0:
            X0 = {x for x in H if x[0] == 'T'}
        else:
            X0 = self._initial

        # Remove actions with no post
        dels = set()
        for x, tran in S.items():
            for u, post in tran.items():
                if len(post) == 0:
                    dels.add((x,u))
        for x,u in dels:
            del S[x][u]

        # Minimize system
        # Hrel = {('T', 'T'), ('W','W'), ('T','W')}
        Sr, HQ, X0r = minimize_alternating_simulation_equivalence(S, H, X0)

        # Rename states
        names = {q:next(x for x in q) for q in Sr}
        Sr = {names[q]:{u: {names[qp] for qp in post}
                        for u, post in tran.items()}
              for q,tran in Sr.items()}
        X0r = {next(x for x in q) for q in X0r}
        HQ = {names[q]:y for q, y in HQ.items()}

        # Push data back to control loop
        self._states = {x:i for i,x in enumerate(Sr)}
        self._transitions = Sr
        if len(self._initial) > 0:
            self._initial = X0r
        self._output_map = HQ
        self._outputs = {y for y in HQ.values()}
        self._outputs = {y:i for i,y in enumerate(self.outputs)}
        self._clear_cache()

    def stats(self):
        return stats(self.transitions, self._initial)
