import os
import unittest
from unittest.mock import Mock, patch

from config import root_path
from sentient.Abstractions import TrafficModelLinearPETC
from sentient.Scheduling.fpiter import controlloop

large_sys_file = os.path.join(root_path, 'tests/scheduling/files/_large_linpetc_sys_tr.txt')

class TestEnumControlLoop(unittest.TestCase):
    longMessage = True

    @classmethod
    def setUpClass(cls) -> None:
        cls.traffic_s = Mock(spec=TrafficModelLinearPETC)
        cls.traffic_s.regions = {(1,), (2,)}
        cls.traffic_s.transitions = {((1,), 1): {(2,), (1,)}, ((2,), 1): {(2,)}, ((2,), 2): {(1,), (2,)}}
        cls.traffic_s.trigger = Mock()
        cls.traffic_s.trigger.h = 0.01

        # medium traffic
        cls.traffic_m = Mock(spec=TrafficModelLinearPETC)
        cls.traffic_m.regions = {(2,), (3,), (4,)}
        cls.traffic_m.transitions = \
            {((2,), 1): {(3,), (4,)}, ((2,), 2): {(2,)},
             ((3,), 1): {(3,), (4,)}, ((3,), 2): {(2,), (3,)}, ((3,), 3): {(2,), (3,)},
             ((4,), 1): {(4,)}, ((4,), 2): {(3,), (4,)}, ((4,), 3): {(2,), (3,), (4,)}, ((4,), 4): {(2,), (3,), (4,)}}
        cls.traffic_m.trigger = Mock()
        cls.traffic_m.trigger.h = 0.01

        # large traffic
        cls.traffic_l = Mock(spec=TrafficModelLinearPETC)
        with open(large_sys_file, 'r') as f:
            cls.traffic_l.transitions = eval(f.read())
        cls.traffic_l.regions = {i for (i, _) in cls.traffic_l.transitions}
        cls.traffic_l.trigger = Mock()
        cls.traffic_l.trigger.h = 0.01

    @classmethod
    def tearDownClass(cls) -> None:
        cls.traffic_s = None
        cls.traffic_m = None
        cls.traffic_l = None

    def setUp(self) -> None:
        self.cl_s = controlloop(self.traffic_s, use_bdd=False)
        self.cl_s_bdd = controlloop(self.traffic_s, use_bdd=True)
        self.cl_m = controlloop(self.traffic_m, use_bdd=False)
        self.cl_m_bdd = controlloop(self.traffic_m, use_bdd=True)
        self.cl_l = controlloop(self.traffic_l, use_bdd=False)
        self.cl_l_bdd = controlloop(self.traffic_l, use_bdd=True)

    def tearDown(self) -> None:
        self.cl_s = None
        self.cl_s_bdd = None
        self.cl_m = None
        self.cl_m_bdd = None
        self.cl_l = None
        self.cl_l_bdd = None

    def testConstruction(self):
        with self.subTest('Medium system'):
            for (s, Ux) in self.cl_m.transitions.items():
                for (u, Post) in Ux.items():
                    for t in Post:
                        self.assertTrue(self.cl_m_bdd.transition_exists(self.cl_m.states[s], self.cl_m.actions[u], self.cl_m.states[t]))
        with self.subTest('Large system'):
            for (s, Ux) in self.cl_l.transitions.items():
                for (u, Post) in Ux.items():
                    for t in Post:
                        self.assertTrue(self.cl_l_bdd.transition_exists(self.cl_l.states[s], self.cl_l.actions[u], self.cl_l.states[t]))

    def testPartitionRefinement(self):
        with self.subTest('Part medium system'):
            self.assertTrue(self.cl_m.create_initial_partition())
            self.assertTrue(self.cl_m_bdd.create_initial_partition())
            for bk, bv in self.cl_m.states.items():
                self.assertTrue(self.cl_m_bdd.block_exists(bv))

        with self.subTest('Ref medium system'):
            self.assertTrue(self.cl_m.refine())
            self.assertTrue(self.cl_m_bdd.refine())
            for bk, bv in self.cl_m.states.items():
                self.assertTrue(self.cl_m_bdd.block_exists(bv))

        with self.subTest('Part large system'):
            self.assertTrue(self.cl_l.create_initial_partition())
            self.assertTrue(self.cl_l_bdd.create_initial_partition())
            for bk, bv in self.cl_l.states.items():
                self.assertTrue(self.cl_l_bdd.block_exists(bv))

        with self.subTest('Ref large system'):
            self.assertTrue(self.cl_l.refine())
            self.assertTrue(self.cl_l_bdd.refine())
            for bk, bv in self.cl_l.states.items():
                self.assertTrue(self.cl_l_bdd.block_exists(bv))

    def testRestore(self):
        tr_x = self.cl_l_bdd.tr
        self.cl_l_bdd.create_initial_partition()
        tr_b = self.cl_l_bdd.tr
        self.assertNotEqual(tr_x, tr_b)
        self.cl_l_bdd.restore()
        new_tr_x = self.cl_l_bdd.tr
        self.assertEqual(tr_x, new_tr_x)
