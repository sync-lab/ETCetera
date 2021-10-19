import itertools
import logging
import os
import sys
from typing import List
import numpy as np
import random

try:
    import dd.cudd as _bdd
except:
    import dd.autoref as _bdd

from ..abstract_system import abstract_system
from .controlloop import controlloop

from config import save_path


class system(abstract_system):

    def __init__(self, cl: List[controlloop], trap_state=False):

        # First check if all correct controlloop types
        if not all([type(i) == controlloop for i in cl]):
            print('All specified controlloops should be of the bdd type')
            raise ValueError()

        super().__init__(cl)

        # self.bdd = _bdd.BDD()  # BDD manager (empty until compose())
        self.bdd = _bdd.BDD()
        self.bdd.configure(reordering=True)

        # BDD's describing composed system (all False until compose())
        self.tr = self.bdd.false
        self.Q = self.bdd.false
        self.XT = self.bdd.false

        # Boolean Variables for composed system (empty until compose())
        self.xvars = []
        self.yvars = []
        self.uvars = []
        self.bvars = []
        self.cvars = []

        self._trap_state = trap_state or any([not c._label_split for c in cl])

    """ Implementing Abstract Methods"""

    def compose(self):
        """
        Creates the BDDs describing all the NFA's in parallel. If R is True, use the partition systems, otherwise
        use the original systems.
        :return: None
        """

        xvars = []
        uvars = []
        yvars = []
        bvars = []
        cvars = []

        tr = self.bdd.true
        XT = self.bdd.false
        Q = self.bdd.true
        for o in self.control_loops:
            xvars += o.xvars
            uvars += o.uvars
            yvars += o.yvars
            bvars += o.bvars
            cvars += o.cvars
            self.bdd.declare(*o.vars)
            tr = self.bdd.apply('&', tr, o.bdd.copy(o.tr, self.bdd))
            XT = self.bdd.apply('|', XT, o.bdd.copy(o.XT, self.bdd))
            Q = self.bdd.apply('&', Q, o.bdd.copy(o.Q, self.bdd))

        # Modify transitions to incorporate trap state
        if self._trap_state:
            self.bdd.declare('tx')
            self.bdd.declare('ty')
            xvars += ['tx']
            yvars += ['ty']

            tx = self.bdd.add_expr('tx')
            ty = self.bdd.add_expr('ty')

            self._trapy = ty
            self._trapx = tx
            self._trapx_var = 'tx'
            self._trapy_var = 'ty'

            # Add one more bit, which will represent the trap state
            tr = self.bdd.apply('&', ~ty, tr)

            # Create BDD for when two or more inputs are 't'
            two_t_bdd = self.bdd.false
            a = []
            for b in self.control_loops:
                a.append(self.bdd.add_expr(b.enc(b.uvars, 1)))

            for i in range(0, self.ns):
                for j in range(i + 1, self.ns):
                    two_t_bdd = self.bdd.apply('|', two_t_bdd, self.bdd.apply('&', a[i], a[j]))

            tr = self.bdd.apply('&', tr, ~two_t_bdd)

            # Add transitions to trap state
            tr = self.bdd.apply('|', tr, self.bdd.apply('&', two_t_bdd, ty))

        self.tr = tr
        self.Q = Q
        self.XT = XT
        self.xvars = xvars
        self.yvars = yvars
        self.uvars = uvars
        self.bvars = bvars
        self.cvars = cvars
        self.vars = xvars + uvars + yvars + bvars + cvars

    def safe_set(self):
        """
        Creates a BDD function describing the safe set, defined as (x1,...,xn) in W if at most one of the outputs
        of xi is 'T'.
        :param R: Boolean defining whether to use the original systems, or the partitioned systems
        :return: BDD function describing the safe set W
        """
        if len(self.bdd.vars) == 0:
            print("Compose the system before generating the safe set.")
            return None

        if self._trap_state:
            return ~self._trapx

        a = []
        for b in self.control_loops:
            a.append(b.bdd.copy(b.XT, self.bdd))

        W = self.bdd.false
        for i in range(0, self.ns):
            if hasattr(self.control_loops[i], '_is_late'):
                q = self.control_loops[i].bdd.copy(self.control_loops[i]._XT_aux, self.bdd)
                W = self.bdd.apply('|', W, q)

            for j in range(i + 1, self.ns):
                W = self.bdd.apply('|', W, self.bdd.apply('&', a[i], a[j]))

        W = ~W
        # if R:  # Make W function of the blocks
        #     W = self.bdd.exist(self.xvars, self.bdd.apply('&', self.Q, W))
        return W

    def safety_game(self, W: _bdd.Function = None):
        """
        Solves the safety game for given safety set W (expressed as BDD function)
        :param W: BDD function defining the safety set
        :return: BDD function defining the solution Z of the safety game or None if this solution is self.bdd.false
        """

        if W is None:
            W = self.safe_set()

        rename = dict()
        for (i, j) in zip(self.bvars, self.cvars):
            rename.update({i: j})

        Z_new = self.bdd.true
        Z_old = self.bdd.false
        it = 1
        while Z_old != Z_new:
            logging.info(f'Safety Game Iteration: {it}')
            Z_old = Z_new
            Z_r = self.bdd.let(rename, Z_old)
            Z_new = self.__safety_operator(W, Z_r)
            it += 1

        if Z_new == self.bdd.false:
            return None

        return Z_new

    def __safety_operator(self, W: _bdd.Function, Z: _bdd.Function):

        B1 = self.bdd.exist(self.cvars, self.tr)
        B2 = self.bdd.forall(self.cvars, self.bdd.apply('->', self.tr, Z))
        # B2 = ~self.bdd.exist(tvar, ~self.bdd.apply('->', self.tr, Z_r))
        B3 = self.bdd.apply('&', B1, B2)
        Z_new = self.bdd.apply('&', W, self.bdd.exist(self.uvars, B3))

        return Z_new

    def create_controller(self, Z: _bdd.Function, StatesOnlyZ=True, convert_blocks=True):
        """
        Creates a safety controller for the composed system
        :param Z: Solution of some scheduler/controller synthesization game (E.g. a safety game)
        :param StatesOnlyZ: Whether to use only the states in the solution or also all states
                            (So that if the system start in a state outside of Z,
                            it can still find a transition leading to Z (if it exists)).
        :return: BDD describing which inputs can be safely chosen for given state i.e, U_c(x).
        """
        if Z is None:
            W = self.safe_set()
            Z = self.safety_game(W)

        rename = dict()
        for (i, j) in zip(self.bvars, self.cvars):
            rename.update({i: j})

        Z_r = self.bdd.let(rename, Z)
        TC = self.bdd.exist(self.cvars, self.tr)
        Ux = self.bdd.apply('&', TC, self.bdd.forall(self.cvars, self.bdd.apply('->', self.tr, Z_r)))
        if StatesOnlyZ:
            Ux = self.bdd.apply('&', Ux, Z)

        if convert_blocks and any([s._is_part for s in self.control_loops]):
            Ux_n = self.bdd.exist(self.bvars, self.bdd.apply('&', Ux, self.Q))
            return Ux_n, None
        else:
            return Ux, self.Q

    def block_repr(self, X: _bdd.Function):
        """
        Represents given BDD using blocks (b_n, ..., b_0)
        @param X: Input BDD
        @return: BDD
        """
        Xr = self.bdd.exist(self.xvars, self.bdd.apply('&', X, self.Q))
        return Xr

    def state_repr(self, X: _bdd.Function):
        """
        Represents given BDD using states (x_n, ..., x_0)
        @param X: Input BDD
        @return: BDD
        """
        Xr = self.bdd.exist(self.bvars, self.bdd.apply('&', X, self.Q))
        return Xr

    def generate_safety_scheduler(self):
        Ux = None

        if self.ns > 5 or (any([hasattr(x, '_is_late') for x in self.control_loops]) and self.ns > 3):
            Ux, Q = self.gen_safety_scheduler_part()
        else:
            Ux, Q = self.gen_safety_scheduler_basic()

        # Save Ux to a DDDMP file
        fpathsched = os.path.join(save_path, 'scheduler.dddmp')
        self.bdd.dump(fpathsched, roots=[Ux])
        print(f"Saved Scheduler BDD to {fpathsched}")

        fpathsys = os.path.join(save_path, 'transitions.dddmp')
        self.bdd.dump(fpathsys, roots=[self.tr])
        print(f"Saved composed system transitions BDD to {fpathsys}")

        if Q is not None:
            fpathsys = os.path.join(save_path, 'state2block.dddmp')
            self.bdd.dump(fpathsys, roots=[Q])
            print(f"Saved state-block BDD to {fpathsys}")

        return Ux, Q

    def simulate(self, Ts: float = 0.01, Tmax: float = 1, x0=None, use_scheduler=True, random_inputs=False, save_results=False):
        # Check correct/enough initial conditions
        if x0 is None:
            x0 = [np.random.uniform(low=-4, high=4, size=(cl.abstraction.plant.nx,)) for cl in self.control_loops]
        else:
            if len(x0) != len(self.control_loops):
                print('Supply initial conditions for each control loop.')
                return

            for x0i, cl in zip(x0, self.control_loops):
                if len(x0i) != cl.abstraction.plant.nx:
                    print(
                        f'Initial condition dimension ({len(x0i)}) does not correspond to the expected ({cl.abstraction.plant.nx}).')
                    return

            x0 = [np.array(x) for x in x0]

        # Clip Ts such that it becomes a multiple of h
        t = int(Ts / self.h)
        Ts = t * self.h

        # 3D Matrix storing the evolution of the continuous states over time.
        x = [[np.array(x0i)] for x0i in x0]
        xhat = [[np.array(x0i)] for x0i in x0]
        u_hist = [[] for i in range(0, self.ns)]  # continuous inputs

        # Evolution of the traffic model regions over time
        regions = [[cl.abstraction.region_of_state(x0i)] for (x0i, cl) in zip(x0, self.control_loops)]

        for i in range(0, self.ns):
            print(f'Controlloop {i} starts in region {regions[i][0]}')

        # 3D Matrix storing the evolution of the transitions sytem states over time.
        if self.state2block is None:
            s = [{p: q for (i, cl) in zip(regions, self.control_loops) for (p, q) in
                  cl.state2bin(f"T{'_'.join([str(l) for l in i[0]])}").items()}]
        else:
            st = {}
            for (i, cl) in zip(regions, self.control_loops):
                st.update(cl.state2bin(f"T{'_'.join([str(l) for l in i[0]])}"))

            temp = self.bdd.let(st, self.Q)
            bl = self.bdd.pick(temp, self.bvars)
            s = [bl]

        v = [[[]] for i in range(0, self.ns)]  # inputs (w/t/lw)

        TriggerTimes = [[0] for i in range(0, self.ns)]
        CollisionTimes = []

        N = int(Tmax / Ts)  # Number of samples

        import scipy
        I = [scipy.integrate.quad_vec(lambda s: scipy.linalg.expm(cl.abstraction.plant.A * s), 0, Ts)[0] for cl in
             self.control_loops]

        possible_actions = [{p: q for cl in self.control_loops for (p, q) in cl.input2bin('w').items()}]
        for j in range(0, self.ns):
            possible_actions.append({p: q for cl in self.control_loops for (p, q) in cl.input2bin('w').items()})
            possible_actions[-1].update({p: q for (p, q) in self.control_loops[j].input2bin('t').items()})

        rename = {y: x for (y, x) in zip(self.yvars, self.xvars)}

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
                UU = self.bdd.let(s[-1], self.scheduler)
                if random_inputs:
                    random.shuffle(possible_actions)

                for u in possible_actions:
                    if self.bdd.let(u, UU) == self.bdd.false:
                        pass
                    else:
                        temp = self.bdd.let(u, self.bdd.let(s[-1], self.tr))
                        sn = self.bdd.pick(self.bdd.let(rename, temp), self.xvars)
                        s.append(sn)
                        break

                for i in range(0, self.ns):
                    if u[self.control_loops[i].uvars[-1]]:
                        reg = self.control_loops[i].abstraction.region_of_state(x[i][-1])
                        si = self.control_loops[i].state2bin(f"T{'_'.join([str(l) for l in reg])}")
                        s[-1].update(si)
                        xhat[i][-1] = xn[i]
                        regions[i].append(reg)
                        TriggerTimes[i].append(t * Ts)

                    else:
                        reg = self.control_loops[i].abstraction.region_of_state(x[i][-1])
                        regions[i].append(reg)

            else:
                triggers = set()
                for i in range(0, self.ns):
                    xx = np.block([x[i][-1].T, xhat[i][-1]])
                    if xx.T @ self.control_loops[i].abstraction.trigger.Qbar @ xx.T > 0 or (
                            t * Ts - TriggerTimes[i][-1]) >= self.h * self.control_loops[i].kmax:
                        xhat[i][-1] = xn[i]
                        TriggerTimes[i].append(t * Ts)
                        triggers.add(i)

                    reg = self.control_loops[i].abstraction.region_of_state(x[i][-1])
                    regions[i].append(reg)

                if len(triggers) > 1:
                    CollisionTimes[t * Ts] = triggers

        import matplotlib.pyplot as plt

        dur = np.arange(0, Ts * N, Ts)
        for i in range(0, self.ns):
            plt.plot(dur, x[i][0:len(dur)], '--')
            plt.gca().set_prop_cycle(None)
            plt.plot(dur, xhat[i][0:len(dur)])
            plt.title(f'Controlloop {i + 1}: $x(t)$ and $x_e(t)$.')
            plt.show()

        for i in range(0, self.ns):
            plt.plot(dur, u_hist[i][0:len(dur)])
            plt.title(f'Controlloop {i + 1}: $u(t)$.')
            plt.show()

        for i in range(0, self.ns):
            plt.plot(TriggerTimes[i], i * np.ones(len(TriggerTimes[i])), 'x')

        for t, ii in CollisionTimes.items():
            for i in ii:
                plt.plot(t, i, 'dk')

        plt.title('Trigger times')
        plt.yticks(range(0, self.ns), [f'Controlloop {i}' for i in range(1, self.ns + 1)])
        plt.show()

        for i in range(0, self.ns):
            plt.plot(dur, regions[i][0:len(dur)])

        plt.title('Traffic Model Regions')
        plt.legend([f'Controlloop {i}' for i in range(1, self.ns + 1)], loc='upper left')
        plt.show()

        if save_results:
            import csv
            from itertools import zip_longest
            with open('sim.csv', 'w') as f:
                writer = csv.writer(f)
                writer.writerow(
                    ['t', 'x11', 'x12', 'x1hat1', 'x1hat2', 'x21', 'x22', 'x2hat1', 'x2hat2', 'u1', 'u2', 'reg1', 'reg2'])
                for (t, x1, xhat1, x2, xhat2, u1, u2, reg1, reg2) in zip(dur, x[0], xhat[0], x[1], xhat[1], u_hist[0],
                                                                         u_hist[1], regions[0], regions[1]):
                    writer.writerow([t, *x1, *xhat1, *x2, *xhat2, *u1, *u2, *reg1, *reg2])

            with open('simTT1.csv', 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['y', 'TT', 'TTc'])
                for i in zip_longest(TriggerTimes[0], CollisionTimes):
                    writer.writerow([1, *i])

            with open('simTT2.csv', 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['y', 'TT', 'TTc'])
                for i in zip_longest(TriggerTimes[1], CollisionTimes):
                    writer.writerow([-1, *i])