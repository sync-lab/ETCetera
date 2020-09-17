from unittest import TestCase
from unittest.mock import patch
import sympy as sp
import numpy as np


class TestParserNonLinearInputData(TestCase):

    def test_dynamics(self):
        from control_system_abstractions.parser.parser_linear_systems import parse_linear
        self.assertTrue(np.allclose(parse_linear('Dynamics : [1 2; 3 4; 5 6]'), np.array([[1, 2], [3, 4], [5, 6]], dtype='f'), rtol=1e-05, atol=1e-08))
        for item in parse_linear('Dynamics : [1 2; 3 4], [1 2; 3 4]'):
            with self.subTest(line=item):
                self.assertTrue(np.allclose(item, np.array([[1, 2], [3, 4]], dtype='f'), rtol=1e-05, atol=1e-08))
        for item in parse_linear('Dynamics : [-1 -2; -3 -4], [-1 -2; -3 -4]'):
            with self.subTest(line=item):
                self.assertTrue(np.allclose(item, np.array([[-1, -2], [-3, -4]], dtype='f'), rtol=1e-05, atol=1e-08))
        with self.assertRaises(Exception) as context:
            parse_linear('Dynamics : [1 2; 3 4], [1 2; 3 4; 5 6]')
        self.assertTrue('Shape of matrices should be same' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Dynamics : 1 2; 3 4], [1 2; 3 4; 5 6]')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Dynamics : [1 2: 3 4], [1 2; 3 4]')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Dynamics : [1 2; 3 4]. [1 2; 3 4]')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Dynamics : []')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Dynamics :')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Dynamics :   ')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Dynamics ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_controller(self):
        from control_system_abstractions.parser.parser_linear_systems import parse_linear
        self.assertTrue(np.allclose(parse_linear('Controller : [1 2; 3 4; 5 6]'), np.array([[1, 2], [3, 4], [5, 6]], dtype='f'), rtol=1e-05, atol=1e-08))
        self.assertTrue(np.allclose(parse_linear('Controller : [1 2; 3 4]'), np.array([[1, 2], [3, 4]], dtype='f'), rtol=1e-05, atol=1e-08))
        self.assertTrue(np.allclose(parse_linear('Controller : [-1 2; 3.6 4; 5 6]'), np.array([[-1, 2], [3.6, 4], [5, 6]], dtype='f'), rtol=1e-05, atol=1e-08))
        with self.assertRaises(Exception) as context:
            parse_linear('Controller : [1 2; 3 4], [1 2; 3 4]')
        self.assertTrue('More than one matrix specified' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Controller : 1 2; 3 4]')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Controller : [1 2: 3 4]')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Controller : [1 2; 3 a]')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Controller : aaaaa')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Controller : []')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Controller :')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Controller :   ')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Controller ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_triggering_condition(self):
        from control_system_abstractions.parser.parser_linear_systems import parse_linear
        self.assertTrue(np.allclose(parse_linear('Triggering Condition : [1 2; 3 4; 5 6]'), np.array([[1, 2], [3, 4], [5, 6]], dtype='f'), rtol=1e-05, atol=1e-08))
        self.assertTrue(np.allclose(parse_linear('Triggering Condition : [1 2 3; 4 5 6]'), np.array([[1, 2, 3], [4, 5, 6]], dtype='f'), rtol=1e-05, atol=1e-08))
        self.assertTrue(np.allclose(parse_linear('Triggering Condition : [1 2 3; -4 5 6]'), np.array([[1, 2, 3], [-4, 5, 6]], dtype='f'), rtol=1e-05, atol=1e-08))
        self.assertTrue(np.allclose(parse_linear('Triggering Condition : [1 2.3 3; -4 5 6.44]'), np.array([[1, 2.3, 3], [-4, 5, 6.44]], dtype='f'), rtol=1e-05, atol=1e-08))
        self.assertTrue(np.allclose(parse_linear('Triggering Condition : [1 2.3 3 ; -4 5 6.44]'), np.array([[1, 2.3, 3], [-4, 5, 6.44]], dtype='f'), rtol=1e-05, atol=1e-08))
        self.assertTrue(np.allclose(parse_linear('Triggering Condition : [1 2.3 3 ;-4 5 6.44]'), np.array([[1, 2.3, 3], [-4, 5, 6.44]], dtype='f'), rtol=1e-05, atol=1e-08))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Condition : [1 2; 3 4; 5 6], [1 2; 3 4; 5 6]')
        self.assertTrue('More than one matrix specified' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Condition : [1 2; 3 4; 5 a]')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Condition : 1 2; 3 4; 5 6')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Condition : [1 2; 3 4; 5 6')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Condition : [1 2.2.; 3 4; 5 6]')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Condition :')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Condition :   ')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Condition ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_triggering_heartbeat(self):
        from control_system_abstractions.parser.parser_linear_systems import parse_linear
        self.assertEqual(parse_linear('Triggering Heartbeat : 2.5'), float('2.5'))
        self.assertEqual(parse_linear('Triggering Heartbeat : 2'), float('2'))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Heartbeat : 2.5.5')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Heartbeat : [1 2]')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Heartbeat : ahsj')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Heartbeat : 2, 3.5')
        self.assertTrue('More than one scalar value specified' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Heartbeat :')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Heartbeat :   ')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Heartbeat ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_triggering_sampling_time(self):
        from control_system_abstractions.parser.parser_linear_systems import parse_linear
        self.assertEqual(parse_linear('Triggering Sampling Time : 2.5'), float('2.5'))
        self.assertEqual(parse_linear('Triggering Sampling Time : 2'), float('2'))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Sampling Time : 2.5.5')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Sampling Time : [1 2]')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Sampling Time : ahsj')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Sampling Time : 2, 3.5')
        self.assertTrue('More than one scalar value specified' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Sampling Time :')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Sampling Time :   ')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Triggering Sampling Time ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_lyapunov_func(self):
        from control_system_abstractions.parser.parser_linear_systems import parse_linear
        self.assertTrue(np.allclose(parse_linear('Lyapunov Function : [1 2; 3 4]'), np.array([[1, 2], [3, 4]], dtype='f'), rtol=1e-05, atol=1e-08))
        self.assertTrue(np.allclose(parse_linear('Lyapunov Function : [1.0 2.1; 3.2 4.3]'), np.array([[1, 2.1], [3.2, 4.3]], dtype='f'), rtol=1e-05, atol=1e-08))
        self.assertTrue(np.allclose(parse_linear('Lyapunov Function : [1.1 2.2 3.3; 4.4 5.5 6.6; 7.7 8.8 9.9]'), np.array([[1.1, 2.2, 3.3], [4.4, 5.5, 6.6], [7.7, 8.8, 9.9]], dtype='f'), rtol=1e-05, atol=1e-08))
        with self.assertRaises(Exception) as context:
            parse_linear('Lyapunov Function : 2.5.5')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Lyapunov Function : [1 2]')
        self.assertTrue('Matrix is not quadratic' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Lyapunov Function : ahsj')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Lyapunov Function : [1 2; 3 4; 5 6]')
        self.assertTrue('Matrix is not quadratic' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Lyapunov Function : [1 2; 3 4], [1 2; 3 4]')
        self.assertTrue('More than one matrix specified' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Lyapunov Function :   ')
        self.assertTrue('Syntax error for value' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Lyapunov Function ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_solver_options(self):
        from control_system_abstractions.parser.parser_linear_systems import parse_linear
        self.assertEqual(parse_linear('Solver Options : 1, 2, 3'), [1.0, 2.0, 3.0])
        self.assertEqual(parse_linear('Solver Options : 1.2, 2.4, 3.7'), [1.2, 2.4, 3.7])
        self.assertEqual(parse_linear('Solver Options : 12., 3.7'), [12.0, 3.7])
        self.assertEqual(parse_linear('Solver Options :'), [])
        self.assertEqual(parse_linear('Solver Options :  '), [])
        with self.assertRaises(Exception) as context:
            parse_linear('Solver Options : 1.2, a, 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Solver Options : 1.2,3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Solver Options : 1.2. a, 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Solver Options : 1.2; 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Solver Options ')
        self.assertTrue('Syntax error' in str(context.exception))

    def test_linesearch_options(self):
        from control_system_abstractions.parser.parser_linear_systems import parse_linear
        self.assertEqual(parse_linear('Linesearch Options : 1, 2, 3'), [1.0, 2.0, 3.0])
        self.assertEqual(parse_linear('Linesearch Options : 1.2, 2.4, 3.7'), [1.2, 2.4, 3.7])
        self.assertEqual(parse_linear('Linesearch Options : 12., 3.7'), [12.0, 3.7])
        self.assertEqual(parse_linear('Linesearch Options :'), [])
        self.assertEqual(parse_linear('Linesearch Options :  '), [])
        with self.assertRaises(Exception) as context:
            parse_linear('Linesearch Options : 1.2, a, 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Linesearch Options : 1.2,3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Linesearch Options : 1.2. a, 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Linesearch Options : 1.2; 3.7')
        self.assertTrue('Non-numerical values found' in str(context.exception))
        with self.assertRaises(Exception) as context:
            parse_linear('Linesearch Options ')
        self.assertTrue('Syntax error' in str(context.exception))

