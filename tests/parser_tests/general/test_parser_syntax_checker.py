from unittest import TestCase
import sympy as sp


class TestParserGeneral(TestCase):
    def test_check_keyvalue_syntax(self):
        from sentient.util.parsing.syntax_checker import check_keyvalue_syntax
        self.assertEqual(check_keyvalue_syntax(':', '{(.*)}', 'Hyperbox States : {[1 2], [2 3], [3 4]}'), ['[1 2], [2 3], [3 4]'])
        self.assertEqual(check_keyvalue_syntax(':', '{(.*)}', 'Hyperbox States  : {[1 2; 3 4], [2 3]}'), ['[1 2; 3 4], [2 3]'])
        self.assertEqual(check_keyvalue_syntax(':', '{(.*)}', 'Hyperbox States  : {[1; 2], [2; 3], [3; 4]}'), ['[1; 2], [2; 3], [3; 4]'])
        self.assertEqual(check_keyvalue_syntax(' ', '\[(.*)\]', '[1 2]'), ['1', '2'])
        self.assertEqual(check_keyvalue_syntax(' |;', '\[(.*)\]', '[1 2]'), ['1', '2'])
        self.assertEqual(check_keyvalue_syntax(' |;', '\[(.*)\]', '[1; 2]'), ['1', '2'])
        self.assertEqual(check_keyvalue_syntax(' |;', '\[(.*)\]', '[ ]'), [])
        self.assertEqual(check_keyvalue_syntax(' |;', '\[(.*)\]', '[a b]'), ['a', 'b'])
        self.assertEqual(check_keyvalue_syntax(' |;', '\[(.*)\]', '[1;2]'), ['1', '2'])
        self.assertEqual(check_keyvalue_syntax(':', '\[(.*)\]', '[1; 2]'), ['1; 2'])
        with self.assertRaises(Exception) as context:
            check_keyvalue_syntax(' ', '\[(.*)\]', 'asdf')
        self.assertTrue('Syntax error for value on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_keyvalue_syntax(':', '{(.*)}', ': : :')
        self.assertTrue('Syntax error for value on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_keyvalue_syntax(' ', '\[(.*)\]', '[1 2')
        self.assertTrue('Syntax error for value on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_keyvalue_syntax(' ', '\[(.*)\]', '{1 2}')
        self.assertTrue('Syntax error for value on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_keyvalue_syntax(' ', '\[(.*)\]', '1 2')
        self.assertTrue('Syntax error for value on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_keyvalue_syntax(' ', '\[(.*)\]', '')
        self.assertTrue('Syntax error for value on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_keyvalue_syntax(' ', '\[(.*)\]', '  ')
        self.assertTrue('Syntax error for value on line: ' in str(context.exception))

    def test_check_if_numerical_values(self):
        from sentient.util.parsing.syntax_checker import check_if_numerical_values
        with self.assertRaises(Exception) as context:
            check_if_numerical_values(['a', '2'])
        self.assertTrue('Non-numerical values found on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_if_numerical_values(['', '2'])
        self.assertTrue('Non-numerical values found on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_if_numerical_values([' ', '2'])
        self.assertTrue('Non-numerical values found on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_if_numerical_values('aa')
        self.assertTrue('Non-numerical values found on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_if_numerical_values(' ')
        self.assertTrue('Non-numerical values found on line: ' in str(context.exception))

    def test_check_matrix_syntax(self):
        from sentient.util.parsing.syntax_checker import check_matrix_syntax
        self.assertEqual(check_matrix_syntax('[1 2; 3 4; 5 6]'), (3, 2))
        self.assertEqual(check_matrix_syntax('[1 2 3 4]'), (1, 4))
        self.assertEqual(check_matrix_syntax('[1; 2; 3; 4; 5; 6]'), (6, 1))
        self.assertEqual(check_matrix_syntax('[1; -2; 3; +4; -5; 6]'), (6, 1))
        self.assertEqual(check_matrix_syntax('[1 2;3 4]'), (2, 2))
        self.assertEqual(check_matrix_syntax('[1 2 ;3 4]'), (2, 2))
        with self.assertRaises(Exception) as context:
            check_matrix_syntax('[a b; c d; 5 6]')
        self.assertTrue('Some value in matrix definition are not numbers on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_matrix_syntax('1 2; 3 4')
        self.assertTrue('Missing matrix or vector definition on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_matrix_syntax('[1 2: 3 4]')
        self.assertTrue('Some value in matrix definition are not numbers on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_matrix_syntax('[1 2| 3 4]')
        self.assertTrue('Some value in matrix definition are not numbers on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_matrix_syntax('[1 2] 3 4')
        self.assertTrue('Incorrect matrix definition on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_matrix_syntax('[1 2 3 +]')
        self.assertTrue('Some value in matrix definition are not numbers on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_matrix_syntax('[1 2] 3 +]')
        self.assertTrue('Some value in matrix definition are not numbers on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_matrix_syntax('[1 2] 3 -1]')
        self.assertTrue('Some value in matrix definition are not numbers on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_matrix_syntax('[1 2; 3 -1; 2]')
        self.assertTrue('Number of columns does not match in the matrix definition on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_matrix_syntax('[1; 3 -1; 2 0]')
        self.assertTrue('Number of columns does not match in the matrix definition on line: ' in str(context.exception))

    def test_check_symbols_in_exprs(self):
        from sentient.util.parsing.syntax_checker import check_symbols_in_exprs
        self.assertEqual(check_symbols_in_exprs(['u', 'x', 'd'], 'x0**2+u0+d0, x1+x0*x2**2+d1, x2*sin(x0)+u1+d2'), None)
        self.assertEqual(check_symbols_in_exprs(['u', 'x', 'd'], '22*sin(x0)+u1+d2'), None)
        # Test if not allowed chars present in expressions
        with self.assertRaises(Exception) as context:
            check_symbols_in_exprs(['u', 'x'], 'x0**2+u0+d0, x1+x0*x2**2+d1, x2*sin(x0)+u1+d2')
        self.assertTrue('Incorrect symbols in expressions on line: ' in str(context.exception))
        # Test capital letters in expressions
        with self.assertRaises(Exception) as context:
            check_symbols_in_exprs(['u', 'x', 'd'], 'x0**2+u0+D0')
        self.assertTrue('Incorrect symbols in expressions on line: ' in str(context.exception))
        # Test not allowed characters in expressions
        with self.assertRaises(Exception) as context:
            check_symbols_in_exprs(['u', 'x', 'd'], 'x0**2+u0+d0, x1+x0*x2**2+d1, x2*sin(x0)+u1+d2+s0')
        self.assertTrue('Incorrect symbols in expressions on line: ' in str(context.exception))
        # Test if xx like patterns exist even if x is allowed
        with self.assertRaises(Exception) as context:
            check_symbols_in_exprs(['u', 'x', 'd'], 'xx0**2+u0+d0')
        self.assertTrue('Incorrect symbols in expressions on line: ' in str(context.exception))
        # Test if symbol without number exists such as d instead of d0, d1..
        with self.assertRaises(Exception) as context:
            check_symbols_in_exprs(['u', 'x', 'd'], 'x0**2+u0+d')
        self.assertTrue('Incorrect symbols in expressions on line: ' in str(context.exception))
        # Test illegal expressions like sin instead of sin(x0)
        with self.assertRaises(Exception) as context:
            check_symbols_in_exprs(['u', 'x', 'd'], 'x2*sin+u1+d2')
        self.assertTrue('Incorrect expression on line: ' in str(context.exception))
        # Test illegal expressions like sin(x0 instead of sin(x0)
        with self.assertRaises(Exception) as context:
            check_symbols_in_exprs(['u', 'x', 'd'], 'x2*sin(x0+u1+d2')
        self.assertTrue('Incorrect expression on line: ' in str(context.exception))
        # Test illegal expressions like sin() instead of sin(x0)
        with self.assertRaises(Exception) as context:
            check_symbols_in_exprs(['u', 'x', 'd'], 'x2*sin()+u1+d2')
        self.assertTrue('Incorrect expression on line: ' in str(context.exception))
        # Test illegal expressions like sin() instead of sin(x0)
        with self.assertRaises(Exception) as context:
            check_symbols_in_exprs(['u', 'x', 'd'], 'x2*sin(x0)+u1+')
        self.assertTrue('Incorrect expression on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_symbols_in_exprs(['u', 'x', 'd'], '')
        self.assertTrue('Incorrect expression on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_symbols_in_exprs(['u', 'x', 'd'], '  ')
        self.assertTrue('Incorrect expression on line: ' in str(context.exception))

    def test_check_symbolic_expr(self):
        from sentient.util.parsing.syntax_checker import check_symbolic_expr
        self.assertEqual(check_symbolic_expr('x2*sin(x0)+u1+d2'), sp.sympify('x2*sin(x0)+u1+d2'))
        self.assertEqual(check_symbolic_expr('x0**2+u0+d0'), sp.sympify('x0**2+u0+d0'))
        with self.assertRaises(Exception) as context:
            check_symbolic_expr('x0**2+u0+')
        self.assertTrue('Incorrect expression on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_symbolic_expr('x2*sin(x0+u1+d2')
        self.assertTrue('Incorrect expression on line: ' in str(context.exception))
        with self.assertRaises(Exception) as context:
            check_symbolic_expr('x2*sin()+u1+d2')
        self.assertTrue('Incorrect expression on line: ' in str(context.exception))
