import logging
import random
from typing import List, Optional
import itertools
import numpy as np

from ..abstract_system import abstract_system
from .controlloop import controlloop


class system(abstract_system):

    def __init__(self, cl: List[controlloop], trap_state=False, name=None):
        if not all([type(i) == controlloop for i in cl]):
            print('All specified controlloops should be of the enumerative type')
            raise ValueError()

        super().__init__(cl)

        self.states = {}
        self.actions = {}
        self.transitions = {}
        self.outputs = {}
        self.output_map = {}
        self._trap_state = trap_state or any([not c._label_split for c in cl])
        self.scheduler = None

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
        inits = (o._initial for o in self.control_loops)
        self.initial = {x for x in itertools.product(*inits)}

        self.transitions = {x: {u: set() for u in self.actions} for x in self.states}
        for xxx in self.states:
            for uuu in self.actions:
                try:
                    if self._trap_state and uuu.count('t') >= 2:
                        self.transitions[xxx][uuu].update({'trap'})
                    else:
                        s = [o.transitions[x][u] for (o, x, u) in zip(self.control_loops, xxx, uuu)]
                        ls = set(itertools.product(*s))
                        self.transitions[xxx][uuu].update(ls)
                except KeyError:
                    continue

        if self._trap_state:
            self.transitions['trap'] = {u: set() for u in self.actions}
            self.states.update({'trap': -1})

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

        if self._trap_state:
            return {k: v for (k, v) in self.states.items() if k != 'trap'}
        else:
            W = {k: v for (k, v) in self.states.items() if isSafe(self.output_map[k])}
            return W

    def safety_game(self, W=None):
        """
        Solve Safety Game for the NFA with safe set W using fixed-point iterations
        :param W: The safe set. If it is not specified, it is first created.
        :return: Solution to the Safety Game
        """
        if self._trap_state:
            F_old = dict()
            F_new = self.states
            it = 1
            while F_old != F_new:
                logging.info(f'Safety Game Iteration: {it}')
                F_old = F_new
                F_new = self.__safety_operator_trap(F_old)
                it += 1

            if F_old == {} or any(x0 not in F_old for x0 in self.initial):
                return None

            return F_old

        else:
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

            if F_old == {} or any(x0 not in F_old for x0 in self.initial):
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
                if len(p) > 0 and all(xp in Z for xp in p):
                    U_c[x].add(u)

        if not any([s._is_part for s in self.control_loops]):
            return U_c, None
        elif convert_blocks and any([s._is_part for s in self.control_loops]):
            U_c_n = {}
            for (b, uuu) in U_c.items():
                if b != 'trap':
                    U_c_n.update({x:uuu for x in itertools.product(*[xx.keys() for xx in self.states[b]])})
            return U_c_n, None
        else:
            # Additionally supply look-up for the blocks
            invBs = [{x:b for (b,xx) in cl.states.items() for x in xx} for cl in self.control_loops]
            return U_c, invBs

    def simulate(self, Ts:float = 0.01, Tmax:float = 1, x0=None, use_scheduler=True, random_inputs=False):
        # Check correct/enough initial conditions
        if x0 is None:
            x0 = [np.random.uniform(low=-4, high=4, size=(cl.abstraction.plant.nx,)) for cl in self.control_loops]
        else:
            if len(x0) != len(self.control_loops):
                print('Supply initial conditions for each control loop.')
                return

            for x0i, cl in zip(x0, self.control_loops):
                if len(x0i) != cl.abstraction.plant.nx:
                    print(f'Initial condition dimension ({len(x0i)}) does not correspond to the expected ({cl.abstraction.plant.nx}).')
                    return

            x0 = [np.array(x) for x in x0]

        # Clip Ts such that it becomes a multiple of h
        t = int(Ts/self.h)
        Ts = t*self.h


        # 3D Matrix storing the evolution of the continuous states over time.
        x = [[np.array(x0i)] for x0i in x0]
        xhat = [[np.array(x0i)] for x0i in x0]
        u_hist = [[] for i in range(0, self.ns)]   # continuous inputs

        # Evolution of the traffic model regions over time
        regions = [[cl.abstraction.region_of_state(x0i)] for (x0i, cl) in zip(x0, self.control_loops)]

        for i in range(0, self.ns):
            print(f'Controlloop {i} starts in region {regions[i][0]}')

        # 3D Matrix storing the evolution of the transitions sytem states over time.
        if self.state2block is None:
            s = [[f"T{'_'.join([str(l) for l in i[0]])}"] for i in regions]
        else:
            b = [self.state2block[j][f"T{'_'.join([str(l) for l in i[0]])}"]  for (i,j) in zip(regions, range(0, self.ns))]
            s = [[b[i]] for i in range(0, self.ns)]
        v = [[[]] for i in range(0, self.ns)]   # inputs (w/t/lw)

        TriggerTimes = [[0] for i in range(0, self.ns)]
        CollisionTimes = {}

        N = int(Tmax/Ts) # Number of samples

        import scipy

        I = [scipy.integrate.quad_vec(lambda s: scipy.linalg.expm(cl.abstraction.plant.A * s), 0, Ts)[0] for cl in self.control_loops]

        for t in range(0, N):
            # Step 1: Update the continuous states
            utemp = [cl.abstraction.controller.K @ xn[-1] for (cl, xn) in zip(self.control_loops, xhat)]
            xn = [scipy.linalg.expm(cl.abstraction.plant.A * Ts) @ xi[-1] + integral @ cl.abstraction.plant.B @ ui
                  for (cl, xi, ui, integral) in zip(self.control_loops, x, utemp, I)]

            for i in range(0, self.ns):
                x[i].append(xn[i])

            for i in range(0, self.ns):
                xhat[i].append(xhat[i][-1])

            for i in range(0, self.ns):
                u_hist[i].append(utemp[i])

            ## Step 2: Check triggering conditions
            # If a scheduler is defined use that
            if self.scheduler is not None and use_scheduler:
                ss = tuple(q[-1] for q in s)
                u_ts = self.scheduler[ss]
                if random_inputs:
                    u_ts = random.choice(list(u_ts))
                else:
                    all_w = tuple('w' for i in range(0, self.ns))
                    if all_w in u_ts:
                        u_ts = all_w
                    else:
                        u_ts = random.choice(list(u_ts))

                for i in range(0, self.ns):
                    if u_ts[i] == 't':
                        reg = self.control_loops[i].abstraction.region_of_state(x[i][-1])
                        si = f"T{'_'.join([str(l) for l in reg])}"
                        if self.state2block is not None:
                            si = self.state2block[i][si]
                        s[i].append(si)
                        xhat[i][-1] = xn[i]
                        regions[i].append(reg)
                        TriggerTimes[i].append(t * Ts)

                    else:
                        # reg = self.control_loops[i].abstraction.region_of_state(x[i][-1])
                        regions[i].append(regions[i][-1])
                        sn = self.control_loops[i].post({s[i][-1]}, u_ts[i])
                        sn = random.choice(list(sn))
                        s[i].append(sn)
                # for
            else:
                triggers = set()
                for i in range(0, self.ns):
                    xx = np.block([x[i][-1].T, xhat[i][-1]])
                    if  xx.T @ self.control_loops[i].abstraction.trigger.Qbar @ xx.T > 0 or (t*Ts - TriggerTimes[i][-1]) >= self.h*self.control_loops[i].kmax:
                        xhat[i][-1] = xn[i]
                        TriggerTimes[i].append(t*Ts)
                        triggers.add(i)

                if len(triggers) > 1:
                    CollisionTimes[t*Ts] = triggers

        import matplotlib.pyplot as plt

        dur = np.arange(0, Ts*N, Ts)
        for i in range(0, self.ns):
            plt.plot(dur, x[i][0:len(dur)], '--')
            plt.gca().set_prop_cycle(None)
            plt.plot(dur, xhat[i][0:len(dur)])
            plt.title(f'Controlloop {i+1}: $x(t)$ and $x_e(t)$.')
            plt.show()

        for i in range(0, self.ns):
            plt.plot(dur, u_hist[i][0:len(dur)])
            plt.title(f'Controlloop {i+1}: $u(t)$.')
            plt.show()

        for i in range(0, self.ns):
            plt.plot(TriggerTimes[i], i*np.ones(len(TriggerTimes[i])), 'x')

        for t, ii in CollisionTimes.items():
            for i in ii:
                plt.plot(t, i, 'd')

        plt.title('Trigger times')
        plt.yticks(range(0, self.ns), [f'Controlloop {i}' for i in range(1, self.ns + 1)])
        plt.show()

        for i in range(0, self.ns):
            plt.plot(dur, regions[i][0:len(dur)])

        plt.title('Traffic Model Regions')
        plt.legend([f'Controlloop {i}' for i in range(1, self.ns + 1)])
        plt.show()



    """ Private Helper Methods """

    def __safety_operator_trap(self, Z:dict):
        F = dict()
        for (x, v) in Z.items():
            if x == 'trap':
                continue
            else:
                for (uk, uv) in self.actions.items():
                    p = self.transitions[x][uk]
                    if len(p) == 0:
                        continue
                    elif any(xp not in Z for xp in p):  # 200x faster
                        continue
                    else:
                        F.update({x: v})

        return F

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
                    elif any(xp not in Z for xp in p):  # 200x faster
                        continue
                    else:
                        F[x] = v

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