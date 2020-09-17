# -*- coding: utf-8 -*-

from unittest import TestCase
from unittest.mock import patch

import control_system_abstractions.data.nonlinear_systems_datastructure as data
import control_system_abstractions.parser as parser
import sympy as sp
import numpy as np


class TestParserNonLinearInputData(TestCase):

    def test_hyperbox_states(self):
        from control_system_abstractions.parser.parser_nonlinear_systems import parse_nonlinear
        self.assertTrue(np.allclose(parse_nonlinear('Hyperbox States : [1 2]'),
                                    np.array([[1, 2]], dtype='f'), rtol=1e-05, atol=1e-08))
        self.assertTrue(np.allclose(parse_nonlinear('Hyperbox States : [1 2], '),
                                    np.array([[1, 2]], dtype='f'), rtol=1e-05, atol=1e-08))
        for item in parse_nonlinear('Hyperbox States : [1 2], [1 2], [1 2]'):
            with self.subTest(line=item):
                self.assertTrue(np.allclose(item, np.array([[1, 2]], dtype='f'), rtol=1e-05, atol=1e-08))
        for item in parse_nonlinear('Hyperbox States : [-1 -2 -3 -4], [-1 -2 -3 -4]'):
            with self.subTest(line=item):
                self.assertTrue(np.allclose(item, np.array([[-1, -2, -3, -4]], dtype='f'), rtol=1e-05, atol=1e-08))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox States : [1 2; 3 4]')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox States : 1 2 3 4]')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox States : [1 2 3 4], [')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox States : asdf')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox States : []')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox States :')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox States :   ')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox States ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_hyperbox_disturbances(self):
        from control_system_abstractions.parser.parser_nonlinear_systems import parse_nonlinear
        self.assertTrue(np.allclose(parse_nonlinear('Hyperbox Disturbances : [1 2]'),
                                    np.array([[1, 2]], dtype='f'), rtol=1e-05, atol=1e-08))
        self.assertTrue(np.allclose(parse_nonlinear('Hyperbox Disturbances : [1 2], '),
                                    np.array([[1, 2]], dtype='f'), rtol=1e-05, atol=1e-08))
        for item in parse_nonlinear('Hyperbox Disturbances : [1 2], [1 2], [1 2]'):
            with self.subTest(line=item):
                self.assertTrue(np.allclose(item, np.array([[1, 2]], dtype='f'), rtol=1e-05, atol=1e-08))
        for item in parse_nonlinear('Hyperbox Disturbances : [-1 -2 -3 -4], [-1 -2 -3 -4]'):
            with self.subTest(line=item):
                self.assertTrue(np.allclose(item, np.array([[-1, -2, -3, -4]], dtype='f'), rtol=1e-05, atol=1e-08))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox Disturbances : [1 2; 3 4]')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox Disturbances : 1 2 3 4]')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox Disturbances : [1 2 3 4], [')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox Disturbances : asdf')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox Disturbances : []')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox Disturbances :')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox Disturbances :   ')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Hyperbox Disturbances ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_dynamics(self):
        from control_system_abstractions.parser.parser_nonlinear_systems import parse_nonlinear
        self.assertEqual(parse_nonlinear('Dynamics : x0**2+u0+d0, x1+x0*x2**2+d1, x2*sin(x0)+u1+d2'), [sp.sympify('x0**2+u0+d0'), sp.sympify('x1+x0*x2**2+d1'), sp.sympify('x2*sin(x0)+u1+d2') ])
        self.assertEqual(parse_nonlinear('Dynamics : 1.2, x0**2'), [sp.sympify('1.2'), sp.sympify('x0**2')])
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Dynamics : a0+x0')
        self.assertTrue('Incorrect symbols in expressions' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Dynamics : 1.2. a')
        self.assertTrue('Incorrect symbols in expressions' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Dynamics : x2*sin()+u1+d2')
        self.assertTrue('Incorrect expression' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Dynamics : x2*sin(x0+u1+d2')
        self.assertTrue('Incorrect expression' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Dynamics : gfjg')
        self.assertTrue('Incorrect symbols in expressions' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Dynamics :')
        self.assertTrue('Incorrect expression' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Dynamics :   ')
        self.assertTrue('Incorrect expression' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Dynamics ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_controller(self):
        from control_system_abstractions.parser.parser_nonlinear_systems import parse_nonlinear
        self.assertEqual(parse_nonlinear('Controller : -x0**2 - x0**3, -x2*sin(x0)-x2'), [sp.sympify('-x0**2 - x0**3'), sp.sympify('-x2*sin(x0)-x2')])
        self.assertEqual(parse_nonlinear('Controller : sin(x0)+x1'), [sp.sympify('sin(x0)+x1')])
        self.assertEqual(parse_nonlinear('Controller : 1.2, x0**2'), [sp.sympify('1.2'), sp.sympify('x0**2')])
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Controller : x0+e0, x0+e0')
        self.assertTrue('Incorrect symbols in expressions' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Controller : a0+x0')
        self.assertTrue('Incorrect symbols in expressions' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Controller : 1.2. a')
        self.assertTrue('Incorrect symbols in expressions' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Controller :')
        self.assertTrue('Incorrect expression' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Controller :   ')
        self.assertTrue('Incorrect expression' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Controller ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_triggering_condition(self):
        from control_system_abstractions.parser.parser_nonlinear_systems import parse_nonlinear
        self.assertEqual(parse_nonlinear('Triggering Condition : x0+e0'), sp.sympify('x0+e0'))
        self.assertEqual(parse_nonlinear('Triggering Condition : sin(x0)+e0'), sp.sympify('sin(x0)+e0'))
        self.assertEqual(parse_nonlinear('Triggering Condition : x0**e0'), sp.sympify('x0**e0'))
        self.assertEqual(parse_nonlinear('Triggering Condition : 1.2'), sp.sympify('1.2'))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Triggering Condition : x0+e0, x0+e0')
        self.assertTrue('Only one expression expected' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Triggering Condition : a0+x0')
        self.assertTrue('Incorrect symbols in expressions' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Triggering Condition : 1.2. a')
        self.assertTrue('Incorrect symbols in expressions' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Triggering Condition :')
        self.assertTrue('Incorrect expression' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Triggering Condition :   ')
        self.assertTrue('Incorrect expression' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Triggering Condition ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_lyapunov_function(self):
        from control_system_abstractions.parser.parser_nonlinear_systems import parse_nonlinear
        self.assertEqual(parse_nonlinear('Lyapunov Function : x0'), sp.sympify('x0'))
        self.assertEqual(parse_nonlinear('Lyapunov Function : sin(x0)'), sp.sympify('sin(x0)'))
        self.assertEqual(parse_nonlinear('Lyapunov Function : x0**2'), sp.sympify('x0**2'))
        self.assertEqual(parse_nonlinear('Lyapunov Function : 1.2'), sp.sympify('1.2'))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Lyapunov Function : x0, x1')
        self.assertTrue('Only one expression expected' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Lyapunov Function : e0+x0')
        self.assertTrue('Incorrect symbols in expressions' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Lyapunov Function : 1.2. a')
        self.assertTrue('Incorrect symbols in expressions' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Lyapunov Function :')
        self.assertTrue('Incorrect expression' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Lyapunov Function :   ')
        self.assertTrue('Incorrect expression' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Lyapunov Function ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_triggering_times(self):
        from control_system_abstractions.parser.parser_nonlinear_systems import parse_nonlinear
        self.assertEqual(parse_nonlinear('Triggering Times : 1, 2, 3'), [1.0, 2.0, 3.0])
        self.assertEqual(parse_nonlinear('Triggering Times : 1.2, 2.4, 3.7'), [1.2, 2.4, 3.7])
        self.assertEqual(parse_nonlinear('Triggering Times : 12., 3.7'), [12.0, 3.7])
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Triggering Times : 1.2, a, 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Triggering Times : 1.2,3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Triggering Times : 1.2. a, 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Triggering Times : 1.2; 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Triggering Times : ')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Triggering Times :')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Triggering Times ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_solver_options(self):
        from control_system_abstractions.parser.parser_nonlinear_systems import parse_nonlinear
        self.assertEqual(parse_nonlinear('Solver Options : 1, 2, 3'), [1.0, 2.0, 3.0])
        self.assertEqual(parse_nonlinear('Solver Options : 1.2, 2.4, 3.7'), [1.2, 2.4, 3.7])
        self.assertEqual(parse_nonlinear('Solver Options : 12., 3.7'), [12.0, 3.7])
        self.assertEqual(parse_nonlinear('Solver Options :'), [])
        self.assertEqual(parse_nonlinear('Solver Options :  '), [])
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Solver Options : 1.2, a, 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Solver Options : 1.2,3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Solver Options : 1.2. a, 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Solver Options : 1.2; 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Solver Options ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_linesearch_options(self):
        from control_system_abstractions.parser.parser_nonlinear_systems import parse_nonlinear
        self.assertEqual(parse_nonlinear('Linesearch Options : 1, 2, 3'), [1.0, 2.0, 3.0])
        self.assertEqual(parse_nonlinear('Linesearch Options : 1.2, 2.4, 3.7'), [1.2, 2.4, 3.7])
        self.assertEqual(parse_nonlinear('Linesearch Options : 12., 3.7'), [12.0, 3.7])
        self.assertEqual(parse_nonlinear('Linesearch Options :'), [])
        self.assertEqual(parse_nonlinear('Linesearch Options :  '), [])
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Linesearch Options : 1.2, a, 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Linesearch Options : 1.2,3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Linesearch Options : 1.2. a, 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Linesearch Options : 1.2; 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_nonlinear('Linesearch Options ')
        self.assertTrue('Syntax error' in str(context.exception))