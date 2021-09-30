import logging
from typing import List

try:
    import dd.cudd as _bdd
except:
    import dd.autoref as _bdd

from ..abstract_system import abstract_system
from .controlloop import controlloop


class system(abstract_system):

    def __init__(self, cl: List[controlloop]):

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
        # TC = self.bdd.apply('&', self.tr, self.bdd.forall(tvar, self.bdd.apply('->', self.tr, Z_r)))
        # Ux = self.bdd.exist(tvar, TC)
        Ux = self.bdd.apply('&', TC, self.bdd.forall(self.cvars, self.bdd.apply('->', self.tr, Z_r)))
        # Ux = self.bdd.apply('&', TC, ~self.bdd.exist(tvar, ~self.bdd.apply('->', self.tr, Z_r)))
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
        if self.ns > 5 or (any([hasattr(x, '_is_late') for x in self.control_loops]) and self.ns > 3):
        # if self.ns < 5:
            return self.gen_safety_scheduler_part()
        else:
            return self.gen_safety_scheduler_basic()
