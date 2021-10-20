import os
import pickle
import unittest
from unittest.mock import Mock, patch

from config import root_path
from sentient.Abstractions import TrafficModelLinearPETC
from sentient.Scheduling.fpiter import controlloop, system
import sentient.Scheduling.fpiter.enum as schedenum
import sentient.Scheduling.fpiter.bdd as schedbdd

large_sys_file = os.path.join(root_path, 'tests/scheduling/files/_large_linpetc_sys_tr.txt')
traffic1_file = os.path.join(root_path, 'tests/scheduling/files/traffic1.pickle')
traffic1_ref_file = os.path.join(root_path, 'tests/scheduling/files/traffic1_ref.pickle')

sys_refsim_file = os.path.join(root_path, 'tests/scheduling/files/sys_testrefsim.txt')

class TestBDDSystem(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        # large traffic
        cls.traffic_l = Mock(spec=TrafficModelLinearPETC)
        with open(large_sys_file, 'r') as f:
            cls.traffic_l.transitions = eval(f.read())
        cls.traffic_l.regions = {i for (i, _) in cls.traffic_l.transitions}
        cls.traffic_l.trigger = Mock()
        cls.traffic_l.trigger.h = 0.01

    @classmethod
    def tearDownClass(cls) -> None:
        cls.traffic_l = None

    def setUp(self) -> None:
        self.cl = controlloop(self.traffic_l, use_bdd=False)
        self.cl_bdd = controlloop(self.traffic_l, use_bdd=True)
        self.cl_bdd2 = controlloop(self.traffic_l, use_bdd=True)

    def tearDown(self) -> None:
        self.cl = None
        self.cl_bdd = None

    def testRegSchedEquivalent(self):
        S = system([self.cl, self.cl])
        Sbdd = system([self.cl_bdd, self.cl_bdd2])
        Ux, _ = S.gen_safety_scheduler_part(convert_blocks=True)
        Uxbdd = Sbdd.gen_safety_scheduler_basic()
        S.restore_all()
        S.compose()

        self.__compUx(S, Sbdd, Ux, Uxbdd)

    def testRegSchedPartEquivalent(self):
        S = system([self.cl, self.cl])

        Sbdd = system([self.cl_bdd, self.cl_bdd2])
        Ux, _ = S.gen_safety_scheduler_part(convert_blocks=True)
        Uxbdd, _ = Sbdd.gen_safety_scheduler_part(convert_blocks=True)

        S.restore_all()
        S.compose()

        Sbdd.restore_all()
        Sbdd.compose()

        self.__compUx(S, Sbdd, Ux, Uxbdd)

    def testTrapStateSchedEquivalent(self):
        S = system([self.cl, self.cl], trap_state=True)
        Sbdd = system([self.cl_bdd, self.cl_bdd2], trap_state=True)
        Ux, _ = S.gen_safety_scheduler_part(convert_blocks=True)
        Uxbdd = Sbdd.gen_safety_scheduler_basic()
        S.restore_all()
        S.compose()
        self.__compUx(S, Sbdd, Ux, Uxbdd, add_Trap=True)

        S = system([self.cl, self.cl], trap_state=True)
        Sbdd = system([self.cl_bdd, self.cl_bdd2], trap_state=True)
        Ux, _ = S.gen_safety_scheduler_part(convert_blocks=True)
        Uxbdd,_ = Sbdd.gen_safety_scheduler_part(convert_blocks=True)
        S.restore_all()
        S.compose()
        Sbdd.restore_all()
        Sbdd.compose()
        self.__compUx(S, Sbdd, Ux, Uxbdd, add_Trap=True)

    def __compUx(self, S, Sbdd, Ux, Uxbdd, add_Trap=False):


        for (x, uuu) in Ux.items():
            iiencs = [schedbdd.controlloop.enc(a.xvars, b.states[ii]) for (a, b, ii) in
                      zip(Sbdd.control_loops, S.control_loops, x)]
            i_enc = ''
            for i in iiencs:
                i_enc += ' & ' + i

            if add_Trap:
                i_enc += f' & !{Sbdd._trapx_var}'

            i_enc = i_enc[3:]

            for uu in uuu:
                uuencs = [schedbdd.controlloop.enc(a.uvars, b.actions[u]) for (a, b, u) in
                          zip(Sbdd.control_loops, S.control_loops, uu)]
                u_enc = ''
                for i in uuencs:
                    u_enc += ' & ' + i

                res = Sbdd.bdd.apply('&', Uxbdd, Sbdd.bdd.add_expr(i_enc + u_enc))
                self.assertNotEqual(res, Sbdd.bdd.false)



