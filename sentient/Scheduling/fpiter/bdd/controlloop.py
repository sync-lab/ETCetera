import math
import sys
import logging
import shortuuid
from functools import cached_property

try:
    import dd.cudd as _bdd

    logging.info("Activated CUDD")
except:
    import dd.autoref as _bdd

from sentient.Abstractions import TrafficModelLinearPETC
from ..enum import controlloop as cltemp


class controlloop:

    def __init__(self, abstraction: TrafficModelLinearPETC, name: str = None, maxLate: int = None,
                 maxLateStates: int = None, ratio: int = 1):
        temp = cltemp(abstraction, maxLate=maxLate, maxLateStates=maxLateStates, ratio=ratio)

        self._h = abstraction.trigger.h

        # Used to differentiate variables later on
        self.name = name or f'cl{shortuuid.uuid()[:3]}'

        self._is_part = False

        # BDD manager
        self.bdd = _bdd.BDD()
        self.bdd.configure(reordering=True)

        # Determine number of boolean Variables needed to represent
        # the states, actions and outputs
        nx = math.ceil(math.log2(len(temp.states)))
        nu = math.ceil(math.log2(len(temp.actions)))

        # Define variables needed to transitions and other sets/relations
        self.xvars = [self.name + 'x{i}'.format(i=i) for i in range(nx - 1, -1, -1)]
        self.uvars = [self.name + 'u{i}'.format(i=i) for i in range(nu - 1, -1, -1)]
        self.yvars = [self.name + 'y{i}'.format(i=i) for i in range(nx - 1, -1, -1)]
        self.bvars = self.xvars.copy()
        self.cvars = self.yvars.copy()
        self.vars = self.xvars + self.uvars + self.yvars + self.bvars + self.cvars

        # Add variables to BDD manager
        self.bdd.declare(*self.vars)

        # Create Expression in BDD manager for the transitions
        # Treat as if every state is its own block
        self._tr_x = self.bdd.false
        for (i, Ux) in temp._transitions.items():
            i_enc = self.enc(self.bvars, temp._states[i])
            for (u, Post) in Ux.items():
                u_enc = self.enc(self.uvars, temp.actions[u])
                for t in Post:
                    t_enc = self.enc(self.cvars, temp._states[t])
                    tr = self.bdd.add_expr(i_enc + '&' + u_enc + '&' + t_enc)
                    self._tr_x = self.bdd.apply('|', self._tr_x, tr)

        # Create BDD for whether x lies in block b
        self._Q_x = self.bdd.true
        # for (x, b) in zip(self.xvars, self.bvars):
        #     self._Q_x = self.bdd.apply('&', self._Q_x, self.bdd.apply('<=>', self.bdd.add_expr(x), self.bdd.add_expr(b)))


        # Create BDD XT for whether state x has output 'T1' or 'T'
        self._XT_x = self.bdd.false
        self.H = dict()
        for (x, y) in temp._output_map.items():
            self.H.update({temp._states[x]: temp._outputs[y]})
            if y in {'T1', 'T'}:
                x_enc = self.enc(self.bvars, temp._states[x])
                self._XT_x = self.bdd.apply('|', self._XT_x, self.bdd.add_expr(x_enc))

        #
        self._tr_b = self.bdd.false
        self._Q_b = self.bdd.false
        self._XT_b = self.bdd.false

        # Check if system is late
        if hasattr(temp, '_is_late'):
            self._is_late = True
            self.maxLate = maxLate
            self.maxLateStates = maxLateStates or max([i for (i,) in abstraction.regions])
            self.ratio = ratio

            # New Variables for aux. system
            nx_aux = math.ceil(math.log2(len(temp._states_aux)))
            self.xvars_aux = [self.name + '_aux_x{i}'.format(i=i) for i in range(nx_aux - 1, -1, -1)]
            self.yvars_aux = [self.name + '_aux_y{i}'.format(i=i) for i in range(nx_aux - 1, -1, -1)]
            self.vars += self.xvars_aux + self.yvars_aux
            self.bdd.declare(*self.xvars_aux)
            self.bdd.declare(*self.yvars_aux)

            # Construct bdd repr. transitions of the aux system
            self._tr_aux = self.bdd.false
            for (i, Ux) in temp._transitions_aux.items():
                i_enc = self.enc(self.xvars_aux, temp._states_aux[i])
                for (u, Post) in Ux.items():
                    u_enc = self.enc(self.uvars, temp.actions[u])
                    for t in Post:
                        t_enc = self.enc(self.yvars_aux, temp._states_aux[t])
                        tr = self.bdd.add_expr(i_enc + '&' + u_enc + '&' + t_enc)
                        self._tr_aux = self.bdd.apply('|', self._tr_aux, tr)


            # Create BDD XT for whether state x in aux. system has output X
            self._XT_aux = self.bdd.false
            for (x, y) in temp._output_map_aux.items():
                if y == 'X':
                    x_enc = self.enc(self.xvars_aux, temp._states_aux[x])
                    self._XT_aux = self.bdd.apply('|', self._XT_aux, self.bdd.add_expr(x_enc))


    @cached_property
    def tr(self):
        if not self._is_part:
            use_tr = self._tr_x
        else:
            use_tr = self._tr_b

        if hasattr(self, '_is_late'):
            return self.bdd.apply('&', use_tr, self._tr_aux)

        return use_tr

    @property
    def Q(self):
        if not self._is_part:
            return self._Q_x
        else:
            return self._Q_b

    @property
    def XT(self):
        if not self._is_part:
            return self._XT_x
        else:
            return self._XT_b

    def _clear_cache(self):
        if 'tr' in self.__dict__:
            del self.__dict__['tr']

    def restore(self):
        self._is_part = False
        self._clear_cache()

    def __copy__(self):
        return self.__deepcopy__()

    def __deepcopy__(self, memodict={}):
        obj = type(self).__new__(self.__class__)

        # First make shallow copy of everything
        obj.__dict__.update(self.__dict__)

        obj.name = f'c_{self.name}'
        obj._h = self._h
        obj.H = self.H.copy()

        # Make new BDD manager and copy the BDDs
        tempbdd = _bdd.BDD()
        obj.bdd = _bdd.BDD()
        obj.bdd.configure(reordering=True)

        # New variables
        obj.xvars = [f'c_{x}' for x in self.xvars]
        obj.uvars = [f'c_{x}' for x in self.uvars]
        obj.yvars = [f'c_{x}' for x in self.yvars]
        obj.bvars = [f'c_{x}' for x in self.bvars]
        obj.cvars = [f'c_{x}' for x in self.cvars]
        obj.vars = obj.xvars + obj.uvars + obj.yvars + obj.bvars + obj.cvars

        if hasattr(self, '_is_late'):
            obj.xvars_aux = [f'c_{x}' for x in self.xvars_aux]
            obj.yvars_aux = [f'c_{x}' for x in self.yvars_aux]
            obj.vars += obj.xvars_aux + obj.yvars_aux

        tempbdd.declare(*obj.vars)
        tempbdd.declare(*self.vars)
        obj.bdd.declare(*obj.vars)
        rename = {i:j for (i,j) in zip(self.vars, obj.vars)}

        for attr in self.__dict__:
            if isinstance(self.__dict__[attr], _bdd.Function):
                tempf = self.bdd.copy(self.__dict__[attr], tempbdd)
                tempf = tempbdd.let(rename, tempf)
                obj.__dict__[attr] = tempbdd.copy(tempf, obj.bdd)

        return obj

    @staticmethod
    def enc(xvars, x):
        """
        Encodes a number as a boolean expression
        :param xvars: List of boolean variables
        :param x: Number to be encoded
        :return: String
        """
        enc = list(controlloop.__enc_bin(x, len(xvars)))

        r = ''
        id = 0
        for i in enc:
            if int(i):
                r += ' & ' + xvars[id]
            else:
                r += ' & !' + xvars[id]

            id += 1
        return r[3:]

    @staticmethod
    def __enc_bin(x, n):
        """
        Encodes x as a binary using at least n binary values
        :param x: Input number
        :param n: Minimum number of values used
        :return: String
        """
        return format(x, '0' + str(n) + 'b')

    def transition_exists(self, i: int, u: int, t: int):
        """
        Check if transition is in the BDD
        :param i: Number of initial state
        :param u: Number of input
        :param t: Number of final state
        :return:
        """

        i_enc = self.enc(self.bvars, i)
        t_enc = self.enc(self.cvars, t)

        u_enc = self.enc(self.uvars, u)
        res = self.bdd.apply('&', self.tr, self.bdd.add_expr(i_enc + ' & ' + u_enc + ' & ' + t_enc))
        if res == self.bdd.false:
            return False
        else:
            return True

    def create_initial_partition(self):
        """
        Partitions the NFA based on the outputs
        :return: New nfa_bdd object
        """
        if self._is_part:
            print("System already partitioned")
            return False

        # Calculate number of blocks needed for the partition
        m = max(self.H.values())
        nb = math.ceil(math.log2(m+1))

        # Boolean variables
        # xvars = self.xvars.copy()
        # uvars = self.uvars.copy()
        # yvars = self.yvars.copy()
        self.bvars = [self.name + 'b{i}'.format(i=i) for i in range(nb - 1, -1, -1)]
        self.cvars = [self.name + 'c{i}'.format(i=i) for i in range(nb - 1, -1, -1)]
        self.vars = self.xvars + self.uvars + self.yvars + self.bvars + self.cvars
        self.bdd.declare(*self.bvars)
        self.bdd.declare(*self.cvars)

        # Determining the blocks of the partition and defining
        # the characteristic function describing them
        self._Q_b = self.bdd.false
        self._XT_b = self.bdd.false
        for (x, b) in self.H.items():
            x_enc = self.enc(self.xvars, x)
            b_enc = self.enc(self.bvars, b)
            x_bdd = self.bdd.add_expr(b_enc)
            q = self.bdd.apply('&', self.bdd.add_expr(x_enc), x_bdd)
            self._Q_b = self.bdd.apply('|', self._Q_b, q)

        self._XT_b = self.bdd.exist(self.xvars, self.bdd.apply('&', self._Q_b, self._XT_x))
            # if b == 0 or b == 1:  # This means that x is a trigger state with output T1 or T
                # self._XT_b = self.bdd.apply('|', self._XT_b, x_bdd)

        # Rename χ_B(b,x) to χ_B(c,y)
        rename = dict()
        for (i, j) in zip(self.xvars + self.bvars, self.yvars + self.cvars):
            rename.update({i: j})

        Qr = self.bdd.let(rename, self._Q_b)

        # Construct new transition function
        QQr = self.bdd.apply('&', self._Q_b, Qr)
        TQQr = self.bdd.apply('&', self._tr_x, QQr)
        self._tr_b = self.bdd.exist(self.xvars + self.yvars, TQQr)
        self._is_part = True
        self._clear_cache()

        return True
        # return nfa_bdd(name, xvars, uvars, yvars, bvars, cvars, bdd, tr, Q, self.H, XT), True

    def block_exists(self, b: dict):
        """
        Checks if there is a block that is equivalent to b
        """

        Xb = self.bdd.false
        for (k, v) in b.items():
            v_enc = self.enc(self.xvars, v)
            Xb = self.bdd.apply('|', Xb, self.bdd.add_expr(v_enc))

        res = self.bdd.exist(self.bvars, self.bdd.forall(self.xvars, self.bdd.apply('<->', self.Q, Xb)))
        if res == self.bdd.true:
            return True
        else:
            print(b)
            return False

    def refine(self):
        if not self._is_part:
            print("Partition system before refining.")
            return False

        bvars = self.bvars.copy()
        cvars = self.cvars.copy()

        Q_part = self._Q_b  # Current partition
        Q_new = self._Q_b   # Will contain new partition

        # To rename x -> y
        rename = {j:k for (j,k) in zip(self.xvars, self.yvars)}

        # Whether the refinement is successfully
        success = False

        # Loop over all the blocks in the current partition (in Q_part)
        # =============================================================================================================
        for i in range(0, 2**len(self.bvars)):

            # Encode i as block
            val = self._encode_as_block(i, self.bvars)
            X_C = self.bdd.let(val, Q_part)
            if X_C == self.bdd.false:
                break
            X_Cr = self.bdd.let(rename, X_C)
            X_preC = self.bdd.exist(self.uvars + self.yvars, self.bdd.apply('&', X_Cr, self._tr_x))

            # Define new variables
            nb = len(bvars)
            bvars.insert(0, self.name + 'b' + str(nb))
            cvars.insert(0, self.name + 'c' + str(nb))
            self.bdd.declare(*[bvars[0], cvars[0]])

            # temp vars
            kvars = [f'k{i}' for i in range(0, nb)]
            self.bdd.declare(*kvars)

            Q_old = Q_new   # current (but not yet final) partition
            Q_new = self.bdd.false

            # Rename Q_old with temp vars
            rename_bk = {b: k for (b, k) in zip(bvars[1:], kvars)}
            Q_old_k = self.bdd.let(rename_bk, Q_old)

            n = 0   # Current new block number

            # Refinement Operator
            # ---------------------------------------------------------------------------------------------------------
            for ii in range(0, 2**len(kvars)):
                val = self._encode_as_block(ii, kvars)
                X_b = self.bdd.let(val, Q_old_k)
                if X_b == self.bdd.false:
                    # print("<:(")
                    break  # Assume that there are no blocks with higher number

                X_1 = self.bdd.apply('&', X_b, X_preC)
                X_2 = self.bdd.apply('&', X_b, ~X_preC)

                if X_1 == self.bdd.false and X_2 == self.bdd.false:
                    print('OEI!')
                    sys.exit()

                elif X_1 == self.bdd.false or X_2 == self.bdd.false:
                    bII = self.bdd.add_expr(self.enc(bvars, n))
                    Q_new = self.bdd.apply('|', Q_new, self.bdd.apply('&', bII, X_b))
                    n += 1
                else:
                    bII = self.bdd.add_expr(self.enc(bvars, n))
                    Q_new = self.bdd.apply('|', Q_new, self.bdd.apply('&', bII, X_1))
                    bII = self.bdd.add_expr(self.enc(bvars, n + 1))
                    Q_new = self.bdd.apply('|', Q_new, self.bdd.apply('&', bII, X_2))
                    n += 2
                    logging.info(f'Refinement: Block {ii} split apart with block {i}')
                    success = True

            # ---------------------------------------------------------------------------------------------------------
            # Check if added b&c-variables are necessary
            if self.bdd.let({bvars[0]: True}, Q_new) == self.bdd.false:
                Q_new = self.bdd.exist([bvars[0]], Q_new)
                bvars.remove(bvars[0])
                cvars.remove(cvars[0])
        # =============================================================================================================
        #     if success and Q_new == Q_old:
        #         print("Success but Old == New")
        #         print(Q_new == self.bdd.false)
        #         print(Q_old == self.bdd.false)
        #
        #         sys.exit()

        if success:
            self.bvars = bvars
            self.cvars = cvars
            self.vars = self.xvars + self.uvars + self.yvars + self.bvars + self.cvars
            rename = {j: k for (j,k) in zip(self.xvars + self.bvars, self.yvars + self.cvars)}
            Q_newr = self.bdd.let(rename, Q_new)
            BT = self.bdd.apply('&', self._tr_x, self.bdd.apply('&', Q_new, Q_newr))
            self._tr_b = self.bdd.exist(self.xvars + self.yvars, BT)
            self._XT_b = self.bdd.exist(self.xvars, self.bdd.apply('&', Q_new, self._tr_x))
            self._Q_b = Q_new

        self._clear_cache()
        return success


    def _encode_as_block(self, i, bvars=None):
        bvars = bvars or self.bvars
        if i >= 2**len(bvars):
            return None
        else:
            c = self.__enc_bin(i, len(bvars))
            val = dict()
            for (j, k) in zip(c, bvars):
                if j == '1':
                    val.update({k: True})
                else:
                    val.update({k: False})
            return val
