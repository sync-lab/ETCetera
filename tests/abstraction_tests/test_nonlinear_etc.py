import os
import pickle
import shutil
import unittest
from unittest.mock import Mock, patch

import sympy

from config import root_path
from sentient.util import construct_nonlinearETC_traffic_from_file
from sentient.Abstractions import TrafficModelNonlinearETC

hom_sys_file = os.path.join(root_path, 'tests/abstraction_tests/files/nl_homogeneous.txt')
hom_sys_corr = os.path.join(root_path, 'tests/abstraction_tests/files/hom_ex_regs_dict.pickle')
hom_sys_backup = os.path.join(root_path, 'tests/abstraction_tests/files/test_hom_backup.pickle')

nonhom_sys_file_mani = os.path.join(root_path, 'tests/abstraction_tests/files/nl_nonhomogeneous_mani.txt')
nonhom_sys_mani_corr = os.path.join(root_path, 'tests/abstraction_tests/files/nonhom_ex_manifold_dict.pickle')
nonhom_sys_mani_backup = os.path.join(root_path, 'tests/abstraction_tests/files/test_nonhom_mani_backup.pickle')

nonhom_sys_file_grid = os.path.join(root_path, 'tests/abstraction_tests/files/nl_nonhomogeneous_grid.txt')
nonhom_sys_grid_corr = os.path.join(root_path, 'tests/abstraction_tests/files/nonhom_ex_grid_dict.pickle')
nonhom_sys_grid_backup = os.path.join(root_path, 'tests/abstraction_tests/files/test_nonhom_grid_backup.pickle')

pert_sys_file = os.path.join(root_path, 'tests/abstraction_tests/files/nl_perturbed.txt')
pert_sys_corr = os.path.join(root_path, 'tests/abstraction_tests/files/pert_ex_dict.pickle')
pert_sys_backup = os.path.join(root_path, 'tests/abstraction_tests/files/test_pert_grid_backup.pickle')



class TestNonlinearETC(unittest.TestCase):
    longMessage = True

    @classmethod
    def tearDownClass(cls) -> None:
        try:
            shutil.rmtree('./counterexamples')
        except:
            pass
        try:
            shutil.rmtree('./images')
        except:
            pass
        try:
            shutil.rmtree('./outputs')
        except:
            pass


    def testHomogenousExample(self):
        with open(hom_sys_corr, 'rb') as f:
            guar_corr = pickle.load(f)

        if os.path.isfile(hom_sys_backup):
            with open(hom_sys_backup, 'rb') as f:
                traffic = pickle.load(f)
        else:
            traffic = construct_nonlinearETC_traffic_from_file(hom_sys_file)
            with open(hom_sys_backup, 'wb') as f:
                pickle.dump(traffic, f)



        # For every region:
        # 'transitions': [[1, 1], [2, 1], [1, 2], [2, 2]],
        # 'index': [1, 1],
        # 'inner_manifold_time': 0.0013, 'outer_manifold_time': 0.0006,
        # 'timing_lower_bound': 0.0006, 'inner_radius': 1.605783505252552,
        # 'outer_radius': 2.906990526001505, 'contains_origin': False
        for reg in guar_corr:
            with self.subTest(f'Region {reg["index"]}'):
                self.assertTrue(any([
                    x.index == reg['index'] and \
                    set(x.transitions) == set(reg['transitions']) and \
                    reg['inner_manifold_time'] == x.inner_manifold_time and \
                    reg['outer_manifold_time'] == x.outer_manifold_time and \
                    reg['timing_lower_bound'] == x.timing_lower_bound and \
                    abs(reg['inner_radius'] - x.inner_radius) <= 1e-3 and \
                    abs(reg['outer_radius'] - x.outer_radius) <= 1e-3 and \
                    reg['contains_origin'] == x.contains_origin
                    for x in traffic.Regions
                ]))


    def testNonHomogeneousManifold(self):
        with open(nonhom_sys_mani_corr, 'rb') as f:
            guar_corr = pickle.load(f)
        if os.path.isfile(nonhom_sys_mani_backup):
            with open(nonhom_sys_mani_backup, 'rb') as f:
                traffic = pickle.load(f)


            # for x in traffic.Regions:
            #     print('')
            #     print(x.__dict__)
        else:
            traffic = construct_nonlinearETC_traffic_from_file(nonhom_sys_file_mani)
            with open(nonhom_sys_mani_backup, 'wb') as f:
                pickle.dump(traffic, f)

        for reg in guar_corr:
            for x in traffic.Regions:
                if x.index == reg['index']:
                    print(f"Regions supposed to be same: \n{reg},\n{x.__dict__}")

            with self.subTest(f'Region: {reg["index"]}'):
                    self.assertTrue(any([
                        x.index == reg['index'] and \
                        set(str(y) for y in x.transitions) == set(str(y) for y in reg['transitions']) and \
                        abs(reg['timing_lower_bound'] - x.timing_lower_bound) <= 0.01 and \
                        abs(reg['timing_upper_bound'] - x.timing_upper_bound) <= 0.01 and \
                        abs(reg['inner_radius'] - x.inner_radius) <= 1e-3 and \
                        abs(reg['outer_radius'] - x.outer_radius) <= 1e-3 and \
                        reg['inner_manifold_time'] == x.inner_manifold_time and \
                        reg['outer_manifold_time'] == x.outer_manifold_time and \
                        reg['contains_origin'] == x.contains_origin and \
                        self._round_sympy(reg['symbolic_domain_reach'].subs({'y1': 'x2'})) == self._round_sympy(x.symbolic_domain_reach)
                        for x in traffic.Regions
                        ]))

        # for reg in guar_corr:
        #     for x in traffic.Regions:
        #         if x.index == reg['index']:
        #             print(self._round_sympy(reg['symbolic_domain_reach']).subs({'y1': 'x2'}))
        #             print(self._round_sympy(x.symbolic_domain_reach))

    def _round_sympy(self, expr):
        res = expr
        for a in sympy.preorder_traversal(expr):
            if isinstance(a, float):
                res = res.subs(a, round(a, 7))

        return res

    def testNonHomogenousGridExample(self):
        with open(nonhom_sys_grid_corr, 'rb') as f:
            guar_corr = pickle.load(f)
        if os.path.isfile(nonhom_sys_grid_backup):
            with open(nonhom_sys_grid_backup, 'rb') as f:
                traffic = pickle.load(f)
        else:
            traffic = construct_nonlinearETC_traffic_from_file(nonhom_sys_file_grid)
            with open(nonhom_sys_grid_backup, 'wb') as f:
                pickle.dump(traffic, f)


        for reg in guar_corr:
            with self.subTest(f'Region: {reg["index"]}'):
                    self.assertTrue(any([
                        x.index == reg['index'] and \
                        set(x.transitions) == set(reg['transitions']) and \
                        abs(reg['timing_lower_bound'] - x.timing_lower_bound) <= 0.01 and \
                        abs(reg['timing_upper_bound'] - x.timing_upper_bound) <= 0.01 and \
                        reg['center'] == x.center and \
                        reg['region_box'] == x.region_box and \
                        reg['contains_origin'] == x.contains_origin
                        for x in traffic.Regions
                        ]))

    def testPerturbedExample(self):
        with open(pert_sys_corr, 'rb') as f:
            guar_corr = pickle.load(f)


        if os.path.isfile(pert_sys_backup):
            with open(pert_sys_backup, 'rb') as f:
                traffic = pickle.load(f)
        else:
            traffic = construct_nonlinearETC_traffic_from_file(pert_sys_file)
            with open(pert_sys_backup, 'wb') as f:
                pickle.dump(traffic, f)

        for reg in guar_corr:
            with self.subTest(f'Region: {reg["index"]}'):
                    self.assertTrue(any([
                        x.index == reg['index'] and \
                        set(x.transitions) == set(reg['transitions']) and \
                        abs(reg['timing_lower_bound'] - x.timing_lower_bound) <= 0.01 and \
                        abs(reg['timing_upper_bound'] - x.timing_upper_bound) <= 0.01 and \
                        reg['center'] == x.center and \
                        reg['region_box'] == x.region_box and \
                        reg['contains_origin'] == x.contains_origin
                        for x in traffic.Regions
                        ]))
