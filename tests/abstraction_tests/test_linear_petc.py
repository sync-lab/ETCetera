import os
import pickle
import shutil
import unittest
from unittest.mock import Mock, patch

from config import root_path
from sentient.util import construct_linearPETC_traffic_from_file
from sentient.Abstractions import TrafficModelLinearPETC

sys1_file = os.path.join(root_path, 'tests/abstraction_tests/files/sys1.txt')
sys1_corr = os.path.join(root_path, 'tests/abstraction_tests/files/sys1_corr.pickle')
sys1_backup = os.path.join(root_path, 'tests/abstraction_tests/files/sys1_backup.pickle')

sys1_z3_file = os.path.join(root_path, 'tests/abstraction_tests/files/sys1_z3.txt')
sys1_z3_corr = os.path.join(root_path, 'tests/abstraction_tests/files/sys1_z3_corr.pickle')
sys1_z3_backup = os.path.join(root_path, 'tests/abstraction_tests/files/sys1_z3_backup.pickle')

sys2_file = os.path.join(root_path, 'tests/abstraction_tests/files/sys2.txt')
sys2_corr = os.path.join(root_path, 'tests/abstraction_tests/files/sys2_corr.pickle')
sys2_backup = os.path.join(root_path, 'tests/abstraction_tests/files/sys2_backup.pickle')

sys2ref_file = os.path.join(root_path, 'tests/abstraction_tests/files/sys2ref.txt')
sys2ref_corr = os.path.join(root_path, 'tests/abstraction_tests/files/sys2ref_corr.pickle')
sys2ref_backup = os.path.join(root_path, 'tests/abstraction_tests/files/sys2ref_backup.pickle')

sys2ref2_file = os.path.join(root_path, 'tests/abstraction_tests/files/sys2ref2.txt')
sys2ref2_corr = os.path.join(root_path, 'tests/abstraction_tests/files/sys2ref2_corr.pickle')
sys2ref2_backup = os.path.join(root_path, 'tests/abstraction_tests/files/sys2ref2_backup.pickle')

class TestLinearPETC(unittest.TestCase):
    longMessage = True
    maxDiff = None

    def test1(self):
        with open(sys1_corr, 'rb') as f:
            regs, trs = pickle.load(f)

        if os.path.isfile(sys1_backup):
            with open(sys1_backup, 'rb') as f:
                traffic = pickle.load(f)
        else:
            traffic = construct_linearPETC_traffic_from_file(sys1_file)
            with open(sys1_backup, 'wb') as f:
                pickle.dump(traffic, f)

        self.assertSetEqual(regs, traffic.regions)
        self.assertDictEqual(trs, traffic.transitions)

        # self.assertSetEqual(regs, traffic.regions)
        # self.assertDictEqual(trs, traffic.transitions)
        # for t in trs:
        #     with self.subTest(f'(old) Post of {t}'):
        #         self.assertIn(t, traffic.transitions)
        #         self.assertSetEqual(trs[t], traffic.transitions[t])
        #
        # for t in traffic.transitions:
        #     with self.subTest(f'(new) Post of {t}'):
        #         self.assertIn(t, trs)
        #         self.assertSetEqual(trs[t], traffic.transitions[t])




    def test2(self):
        with open(sys2_corr, 'rb') as f:
            regs, trs = pickle.load(f)

        if os.path.isfile(sys2_backup):
            with open(sys2_backup, 'rb') as f:
                traffic = pickle.load(f)
        else:
            traffic = construct_linearPETC_traffic_from_file(sys2_file)
            with open(sys2_backup, 'wb') as f:
                pickle.dump(traffic, f)

        self.assertSetEqual(regs, traffic.regions)
        self.assertDictEqual(trs, traffic.transitions)


    def test2ref(self):
        with open(sys2ref_corr, 'rb') as f:
            regs, trs = pickle.load(f)

        if os.path.isfile(sys2ref_backup):
            with open(sys2ref_backup, 'rb') as f:
                traffic = pickle.load(f)
        else:
            traffic = construct_linearPETC_traffic_from_file(sys2ref_file)
            with open(sys2ref_backup, 'wb') as f:
                pickle.dump(traffic, f)

        self.assertSetEqual(regs, traffic.regions)
        self.assertDictEqual(trs, traffic.transitions)

    # def test2ref2(self):
    #     with open(sys2ref2_corr, 'rb') as f:
    #         regs, trs = pickle.load(f)
    #
    #     if os.path.isfile(sys2ref2_backup):
    #         with open(sys2ref2_backup, 'rb') as f:
    #             traffic = pickle.load(f)
    #     else:
    #         traffic = construct_linearPETC_traffic_from_file(sys2ref2_file)
    #         with open(sys2ref2_backup, 'wb') as f:
    #             pickle.dump(traffic, f)
    #
    #     self.assertSetEqual(regs, traffic.regions)
    #     # self.assertDictEqual(trs, traffic.transitions)
    #     for t in trs:
    #         with self.subTest(f'(old) Post of {t}'):
    #             self.assertIn(t, traffic.transitions)
    #             self.assertSetEqual(trs[t], traffic.transitions[t])
    #
    #     for t in traffic.transitions:
    #         with self.subTest(f'(new) Post of {t}'):
    #             self.assertIn(t, trs)
    #             self.assertSetEqual(trs[t], traffic.transitions[t])