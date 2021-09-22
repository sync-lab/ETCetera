import logging
from typing import List, Optional
import itertools

from ..abstract_system import abstract_system
from .controlloop import controlloop


class system(abstract_system):

    def __init__(self, cl: List[controlloop]):
        if not all([type(i) == controlloop for i in cl]):
            print('All specified controlloops should be of the enumerative type')
            raise ValueError()

        super().__init__(cl)

        self.states = {}
        self.actions = {}
        self.transitions = {}
        self.outputs = {}
        self.output_map = {}

    def post(self, x: dict, u: dict = None):
        """
        Calculates the set of next states for given action(s) or all actions if actions is not given
        :param x: set of state(s)
        :param u: set of actions
        :return: set of next states
        """
        r = set()
        if u is None:
            u = self.actions

        for i in x:
            for j in u:
                r.update(self.transitions[i][j])

        return r

    def compose(self):
        """
        Creates the sets and dictionaries describing all the NFA's in parallel.
        If R is True, use the partition systems, otherwise
        use the original systems.
        :return: None
        """
        self.states = self._c_dict([o.states for o in self.control_loops])
        self.outputs = self._c_dict([o._outputs for o in self.control_loops])
        self.actions = self._c_dict([o.actions for o in self.control_loops])
        self.output_map = self._c_dict([o.output_map for o in self.control_loops])

        self.transitions = {x: {u: set() for u in self.actions} for x in self.states}
        for xxx in self.states:
            for uuu in self.actions:
                s = [o.transitions[x][u] for (o, x, u) in zip(self.control_loops, xxx, uuu)]
                ls = set(itertools.product(*s))
                self.transitions[xxx][uuu].update(ls)

        # print(self.states)
        # print(self.transitions)

    def safe_set(self) -> Optional[dict]:
        """
        Creates a dict describing the safe set, defined as (x1,...,xn) in W if at most one of the outputs
        of xi is 'T'.
        :return: BDD function describing the safe set W
        """
        if len(self.states) == 0:
            print("Compose the system before generating the safe set.")
            return dict()

        def isSafe(out: tuple):
            numT = 0
            numX = 0
            for i in out:
                if type(i) != tuple:
                    numT += (i == 'T' or i == 'T1')
                else:
                    numT += (i[0] == 'T' or i[0] == 'T1')
                    numX += (i[1] == 'X')
            return (numX == 0 and numT <= 1)

        # W = {k: v for (k, v) in self.states.items()
        #      if self.output_map[k].count('T') + self.output_map[k].count('T1') <= 1}
        W = {k: v for (k, v) in self.states.items() if isSafe(self.output_map[k])}
        return W

    def safety_game(self, W=None):
        """
        Solve Safety Game for the NFA with safe set W using fixed-point iterations
        :param W: The safe set. If it is not specified, it is first created.
        :return: Solution to the Safety Game
        """
        if W is None:
            W = self.safe_set()

        F_old = dict()
        F_new = self.states
        it = 1
        while F_old != F_new:
            logging.info(f'Safety Game Iteration: {it}')
            F_old = F_new
            F_new = self.__safety_operator(W, F_old)
            it += 1

        if F_old == {}:
            return None

        return F_old

    # TODO: Add possibility to return full scheduler transition system
    def create_controller(self, Z: dict, StatesOnlyZ=True, convert_blocks=True):
        """
        Creates a controller
        :param Z:
        :param StatesOnlyZ: Specifies whether to only use the states in Z for the controller
        :return: Ux, Optional[Block->State Mapping]
        """

        if StatesOnlyZ:
            c_states = Z.copy()
        else:
            c_states = self.states.copy()

        U_c = {x: set() for x in c_states}
        for x in c_states:
            for u in self.actions:
                p = self.transitions[x][u]
                if len(p) > 0 and set(Z.keys()).issuperset(p):
                    U_c[x].add(u)

        # c_actions = self.actions
        # c_transitions = {x: dict() for x in c_states}
        # for (xc, xv) in c_states.items():
        #     for (uk, uv) in c_actions.items():
        #         p = self.transitions[xc][uk]
        #         if len(p) > 0 and set(Z.keys()).issuperset(p):
        #             c_transitions[xc].update({uk: p})
        if convert_blocks and any([s._is_part for s in self.control_loops]):
            # curr_State = ('T', 'T')
            # st_temp = self._c_dict([x.values() for x in self.states[curr_State]])
            U_c_n = {x: uuu for (b, uuu) in U_c.items() for x in itertools.product(*[xx.keys() for xx in self.states[b]])}
            return U_c_n, None
        else:
            # Additionally supply look-up for the blocks
            invB = {x:b for (b,xx) in self.states for x in xx}
            return U_c, invB
        # return nfa(c_states, c_actions, c_transitions, self.outputs, self.output_map)

    """ Private Helper Methods """

    def __safety_operator(self, W: dict, Z: dict):
        """
        :param W:
        :param Z:
        :return:
        """
        F = dict()
        for (x, v) in Z.items():
            if x not in W:
                continue
            else:
                for (uk, uv) in self.actions.items():
                    p = self.transitions[x][uk]
                    if len(p) == 0:
                        continue
                    elif not set(Z.keys()).issuperset(p):
                        continue
                    else:
                        F.update({x: v})

        return F

    @staticmethod
    def _c_dict(l: list):
        """
        Combination of list of dicts. I.e. l = [{a:1, b:2}, {c:3, d:4}]
        -> res = {(a,c):(1,3), (a,d):(1,4)...}
        :param l: List of dict's
        :return:
        """
        a = [[key for key in d] for d in l]
        b = [[val for val in d.values()] for d in l]
        la = itertools.product(*a)
        lb = itertools.product(*b)
        return {a: b for (a, b) in zip(la, lb)}