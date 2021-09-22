import os
import pickle
import unittest
from unittest.mock import Mock, patch

from config import root_path
from sentient.Abstractions import TrafficModelLinearPETC
from sentient.Scheduling.fpiter import controlloop

large_sys_file = os.path.join(root_path, 'tests/scheduling/files/_large_linpetc_sys_tr.txt')
traffic1_file = os.path.join(root_path, 'tests/scheduling/files/traffic1.pickle')
traffic1_ref_file = os.path.join(root_path, 'tests/scheduling/files/traffic1_ref.pickle')

sys_refsim_file = os.path.join(root_path, 'tests/scheduling/files/sys_testrefsim.txt')

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
        cls.traffic_l.regions = {i for (i,_) in cls.traffic_l.transitions}
        cls.traffic_l.trigger = Mock()
        cls.traffic_l.trigger.h = 0.01

    @classmethod
    def tearDownClass(cls) -> None:
        cls.traffic_s = None
        cls.traffic_m = None
        cls.traffic_l = None

    def setUp(self) -> None:
        self.cl_s = controlloop(self.traffic_s, use_bdd=False)
        self.cl_m = controlloop(self.traffic_m, use_bdd=False)
        self.cl_l = controlloop(self.traffic_l, use_bdd=False)

    def tearDown(self) -> None:
        self.cl_s = None
        self.cl_m = None
        self.cl_l = None

    def testConstruction(self):
        with self.subTest('test constr. small cl'):
            expected_regions = {'T1', 'T2', 'W2,1'}
            self.assertEqual(expected_regions, set(self.cl_s.states))

            expected_transitions = {'T1': {'w': set(), 't': {'T1', 'T2'}}, 'T2': {'w': {'W2,1'}, 't': {'T2'}},
                                    'W2,1': {'w': set(), 't': {'T1', 'T2'}}}
            self.assertEqual(expected_transitions, self.cl_s.transitions)

            expected_H = {'T1': 'T1', 'T2': 'T', 'W2,1': 'W1'}
            self.assertEqual(expected_H, self.cl_s.output_map)

        with self.subTest('test constr. med. cl'):
            expected_regions = {'T2', 'T3', 'T4', 'W2,1', 'W3,1', 'W3,2', 'W4,1', 'W4,2', 'W4,3'}
            self.assertEqual(expected_regions, set(self.cl_m.states))

            expected_transitions = {
                'T2': {'w': {'W2,1'}, 't': {'T3', 'T4'}}, 'W2,1': {'w': set(), 't': {'T2'}},
                'T3': {'w': {'W3,1'}, 't': {'T3', 'T4'}}, 'W3,1': {'w': {'W3,2'}, 't': {'T2', 'T3'}},
                'W3,2': {'w': set(), 't': {'T2', 'T3'}}, 'T4': {'w': {'W4,1'}, 't': {'T4'}},
                'W4,1': {'w': {'W4,2'}, 't': {'T3', 'T4'}}, 'W4,2': {'w': {'W4,3'}, 't': {'T2', 'T3', 'T4'}},
                'W4,3': {'w': set(), 't': {'T2', 'T3', 'T4'}}}
            self.assertEqual(expected_transitions, self.cl_m.transitions)

            expected_H = {'T2': 'T', 'T3': 'T', 'T4': 'T', 'W2,1': 'W1', 'W3,1': 'W2',
                          'W3,2': 'W1', 'W4,1':'W3', 'W4,2':'W2', 'W4,3': 'W1'}
            self.assertEqual(expected_H, self.cl_m.output_map)

    def testCorrectStatesAfterPartitioningAndRefinement(self):
        # Trivial partition
        with self.subTest('test part. small cl'):
            self.assertTrue(self.cl_s.create_initial_partition())
            expected_states = {'T1', 'T', 'W1'}
            expected_transitions = {'T1': {'w': set(), 't': {'T1', 'T'}}, 'T': {'w': {'W1'}, 't': {'T'}},
                            'W1': {'w': set(), 't': {'T1', 'T'}}}
            expected_H = {'T1': 'T1', 'T': 'T', 'W1': 'W1'}
            self.assertEqual(expected_states, set(self.cl_s.states))
            self.assertEqual(expected_transitions, self.cl_s.transitions)
            self.assertEqual(expected_H, self.cl_s.output_map)

        with self.subTest('test ref. small cl'):
            self.assertFalse(self.cl_s.refine())

        with self.subTest('test part. med. cl'):
            self.assertTrue(self.cl_m.create_initial_partition())
            expected_states = {'T', 'W1', 'W2', 'W3'}
            expected_transitions = {
                'T': {'w': {'W1', 'W2', 'W3'}, 't': {'T'}}, 'W3': {'w': {'W2'}, 't':{'T'}},
                'W2': {'w': {'W1'}, 't': {'T'}}, 'W1': {'w': set(), 't':{'T'}},
            }
            expected_H = {i:i for i in expected_states}
            self.assertEqual(expected_states, set(self.cl_m.states))
            self.assertEqual(expected_transitions, self.cl_m.transitions)
            self.assertEqual(expected_H, self.cl_m.output_map)

        with self.subTest('test ref. med. cl'):
            self.assertTrue(self.cl_m.refine())

            expected_blocks = [{'T4'},{'T2'},{'T3'},{'W4,1'},{'W4,2', 'W3,1'},{'W2,1', 'W3,2', 'W4,3'}]
            real_blocks = [set(b.keys()) for b in self.cl_m.states.values()]
            self.assertCountEqual(expected_blocks, real_blocks)

            invB = {x: k for (k, v) in self.cl_m.states.items() for x in v}

            og_tr = {
                'T2': {'w': {'W2,1'}, 't': {'T3', 'T4'}}, 'W2,1': {'w': set(), 't': {'T2'}},
                'T3': {'w': {'W3,1'}, 't': {'T3', 'T4'}}, 'W3,1': {'w': {'W3,2'}, 't': {'T2', 'T3'}},
                'W3,2': {'w': set(), 't': {'T2', 'T3'}}, 'T4': {'w': {'W4,1'}, 't': {'T4'}},
                'W4,1': {'w': {'W4,2'}, 't': {'T3', 'T4'}}, 'W4,2': {'w': {'W4,3'}, 't': {'T2', 'T3', 'T4'}},
                'W4,3': {'w': set(), 't': {'T2', 'T3', 'T4'}}}
            for (x, Ux) in og_tr.items():
                for (u, Post) in Ux.items():
                    for y in Post:
                        self.assertIn(invB[y], self.cl_m.transitions[invB[x]][u])

    def testRestore(self):
        states_cpy = self.cl_m.states.copy()
        self.cl_m.create_initial_partition()
        blocks_cpy = self.cl_m.states.copy()
        self.assertNotEqual(states_cpy, blocks_cpy)
        self.cl_m.restore()
        new_states_cpy = self.cl_m.states.copy()
        self.assertEqual(states_cpy, new_states_cpy)

    def testPost(self):
        with self.subTest('Non-part med system'):
            for x in self.cl_m.states:
                post1 = self.cl_m.post({x})
                post2 = {y for (_, Post) in self.cl_m.transitions[x].items() for y in Post}
                self.assertEqual(post1, post2)
                for y in self.cl_m.states:
                    post1 = self.cl_m.post({x,y})
                    post2a = {i for (_, Post) in self.cl_m.transitions[x].items() for i in Post}
                    post2b = {i for (_, Post) in self.cl_m.transitions[y].items() for i in Post}
                    self.assertEqual(post1, post2a.union(post2b))

        with self.subTest('Part med system'):
            self.cl_m.create_initial_partition()
            for x in self.cl_m.states:
                post1 = self.cl_m.post({x})
                post2 = {y for (_, Post) in self.cl_m.transitions[x].items() for y in Post}
                self.assertEqual(post1, post2)
                for y in self.cl_m.states:
                    post1 = self.cl_m.post({x,y})
                    post2a = {i for (_, Post) in self.cl_m.transitions[x].items() for i in Post}
                    post2b = {i for (_, Post) in self.cl_m.transitions[y].items() for i in Post}
                    self.assertEqual(post1, post2a.union(post2b))

        with self.subTest('Non-part large system'):
            for x in self.cl_l.states:
                post1 = self.cl_l.post({x})
                post2 = {y for (_, Post) in self.cl_l.transitions[x].items() for y in Post}
                self.assertEqual(post1, post2)
                for y in self.cl_l.states:
                    post1 = self.cl_l.post({x,y})
                    post2a = {i for (_, Post) in self.cl_l.transitions[x].items() for i in Post}
                    post2b = {i for (_, Post) in self.cl_l.transitions[y].items() for i in Post}
                    self.assertEqual(post1, post2a.union(post2b))

        with self.subTest('Part large system'):
            self.cl_l.create_initial_partition()
            for x in self.cl_l.states:
                post1 = self.cl_l.post({x})
                post2 = {y for (_, Post) in self.cl_l.transitions[x].items() for y in Post}
                self.assertEqual(post1, post2)
                for y in self.cl_l.states:
                    post1 = self.cl_l.post({x,y})
                    post2a = {i for (_, Post) in self.cl_l.transitions[x].items() for i in Post}
                    post2b = {i for (_, Post) in self.cl_l.transitions[y].items() for i in Post}
                    self.assertEqual(post1, post2a.union(post2b))

    def testPostOfPre(self):
        with self.subTest('Medium system'):
            # x should be in the Pre of a subset of its Post for the given input
            for (x, Ux) in self.cl_m.transitions.items():
                for (u, Post) in Ux.items():
                    if Post == set():
                        continue
                    pre = self.cl_m.pre(Post, u)
                    self.assertIn(x, pre)

            # x should also be in the Pre of all the Post
            for x in self.cl_m.states:
                Post = self.cl_m.post({x})
                pre = self.cl_m.pre(Post)
                self.assertIn(x, pre)

        with self.subTest('Medium system part'):
            self.cl_m.create_initial_partition()
            # x should be in the Pre of a subset of its Post for the given input
            for (x, Ux) in self.cl_m.transitions.items():
                for (u, Post) in Ux.items():
                    if Post == set():
                        continue
                    pre = self.cl_m.pre(Post, u)
                    self.assertIn(x, pre)

            # x should also be in the Pre of all the Post
            for x in self.cl_m.states:
                Post = self.cl_m.post({x})
                pre = self.cl_m.pre(Post)
                self.assertIn(x, pre)

        with self.subTest('Large System'):
            # x should be in the Pre of a subset of its Post for the given input
            for (x, Ux) in self.cl_l.transitions.items():
                for (u, Post) in Ux.items():
                    if Post == set():
                        continue
                    pre = self.cl_l.pre(Post, u)
                    self.assertIn(x, pre)

            # x should also be in the Pre of all the Post
            for x in self.cl_l.states:
                Post = self.cl_l.post({x})
                pre = self.cl_l.pre(Post)
                self.assertIn(x, pre)

        with self.subTest('Large System part'):
            self.cl_l.create_initial_partition()
            # x should be in the Pre of a subset of its Post for the given input
            for (x, Ux) in self.cl_l.transitions.items():
                for (u, Post) in Ux.items():
                    if Post == set():
                        continue
                    pre = self.cl_l.pre(Post, u)
                    self.assertIn(x, pre)

            # x should also be in the Pre of all the Post
            for x in self.cl_l.states:
                Post = self.cl_l.post({x})
                pre = self.cl_l.pre(Post)
                self.assertIn(x, pre)

    # def testSupplyInvalidTrafficModel(self):
    #     temps = Mock(spec=TrafficModelLinearPETC)
    #     temps.regions = {(1,1), (2,1), (1,2), (2,2)}
    #     temps.transitions = {((1,1), 1): {(2,2), (1,2)}, ((2,), 1): {(2,2)}, ((2,1), 2): {(1,), (2,1)}}
    #     self.assertRaises(NotImplementedError, controlloop, temps, use_bdd=False)

    # TODO: (More) Lateness tests
    def testConstrLate(self):
        cl_late = controlloop(self.traffic_l, maxLate=1, use_bdd=False)
        self.assertIn('W0', cl_late._outputs)
        self.assertIn('W9,9', cl_late._transitions['W9,8']['lw'])
        self.assertSetEqual(cl_late._transitions['W9,8']['w'], set())

    def testStateGetterLate(self):
        cl_late = controlloop(self.traffic_l, maxLate=1, use_bdd=False)
        self.assertNotEqual(cl_late._states, cl_late.states)
        for x in cl_late._states:
            for xaux in cl_late._states_aux:
                self.assertIn((x, xaux), cl_late.states)

    def testTransitionGetterLate(self):
        cl_late = controlloop(self.traffic_l, maxLate=1, use_bdd=False)
        self.assertNotEqual(cl_late._transitions, cl_late.transitions)

        # Test that every combination of transitions in S and S_aux are in cl_late.transitions
        for (x, Ux) in cl_late._transitions.items():
            for (u, Post) in Ux.items():
                for y in Post:
                    for (xaux, Uaux) in cl_late._transitions_aux.items():
                        for (uaux, Postaux) in Uaux.items():
                            if uaux != u:
                                continue
                            for yaux in Postaux:
                                self.assertIn((y, yaux), cl_late.transitions[(x, xaux)][u])


    # TODO: (More) Tests for constr with refined traffic model
    def testRefinedTrafficIsSimilar(self):
        # with open(traffic1_file, 'rb') as f:
        #     traffic = pickle.load(f)
        # with open(traffic1_ref_file, 'rb') as f:
        #     traffic_ref = pickle.load(f)

        from sentient.util import construct_linearPETC_traffic_from_file
        traffic = construct_linearPETC_traffic_from_file(sys_refsim_file)

        c1 = controlloop(traffic, use_bdd=False)

        traffic.refine()
        c2 = controlloop(traffic, use_bdd=False)

        _, b1 = c1.check_similar(c2) # c1 <= c2 ?
        _, b2 = c2.check_similar(c1) # c2 <= c1 ?
        
        self.assertTrue(b2)
        self.assertFalse(b1)