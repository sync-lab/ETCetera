import io
import unittest
import unittest.mock as mock
from unittest.mock import patch
from unittest import TestCase
from sentient.util.parsing.parser_nonlinear_systems import parse_from_file


class TestParserFileRead(TestCase):

    @unittest.mock.patch('sys.stdout', new_callable=io.StringIO)
    def assert_stdout(self, file, expected_output, mock_stdout):
        from sentient.util.parsing.parser_nonlinear_systems import parse_from_file
        parse_from_file(file)
        self.assertEqual(mock_stdout.getvalue(), expected_output)

    def test_parse_from_file(self):
        from sentient.util.parsing.parser_nonlinear_systems import parse_from_file
        with self.assertRaises(SystemExit) as context:
            parse_from_file('/Users/gmaddodi/Downloads/control_systems_abstractions/tests/parser_tests/nonlinear_systems_utils/incorrect_hyperbox_states_key.txt')
        self.assertEqual(context.exception.code, None)
        with self.assertRaises(SystemExit) as context:
            parse_from_file('/Users/gmaddodi/Downloads/control_systems_abstractions/tests/parser_tests/nonlinear_systems_utils/incorrect_hyperbox_disturbances_key.txt')
        self.assertEqual(context.exception.code, None)
        with self.assertRaises(SystemExit) as context:
            parse_from_file('/Users/gmaddodi/Downloads/control_systems_abstractions/tests/parser_tests/nonlinear_systems_utils/incorrect_dynamics_key.txt')
        self.assertEqual(context.exception.code, None)
        with self.assertRaises(SystemExit) as context:
            parse_from_file('/Users/gmaddodi/Downloads/control_systems_abstractions/tests/parser_tests/nonlinear_systems_utils/incorrect_controller_key.txt')
        self.assertEqual(context.exception.code, None)
        with self.assertRaises(SystemExit) as context:
            parse_from_file('/Users/gmaddodi/Downloads/control_systems_abstractions/tests/parser_tests/nonlinear_systems_utils/incorrect_triggering_condition_key.txt')
        self.assertEqual(context.exception.code, None)
        with self.assertRaises(SystemExit) as context:
            parse_from_file('/Users/gmaddodi/Downloads/control_systems_abstractions/tests/parser_tests/nonlinear_systems_utils/incorrect_triggering_times_key.txt')
        self.assertEqual(context.exception.code, None)
        with self.assertRaises(SystemExit) as context:
            parse_from_file('/Users/gmaddodi/Downloads/control_systems_abstractions/tests/parser_tests/nonlinear_systems_utils/incorrect_lyapunov_func_key.txt')
        self.assertEqual(context.exception.code, None)
        with self.assertRaises(SystemExit) as context:
            parse_from_file('/Users/gmaddodi/Downloads/control_systems_abstractions/tests/parser_tests/nonlinear_systems_utils/incorrect_solver_options_key.txt')
        self.assertEqual(context.exception.code, None)
        with self.assertRaises(SystemExit) as context:
            parse_from_file('/Users/gmaddodi/Downloads/control_systems_abstractions/tests/parser_tests/nonlinear_systems_utils/incorrect_linesearch_options_key.txt')
        self.assertEqual(context.exception.code, None)


    # Solution one: testing print with @patch
    @patch('sys.stdout', new_callable=io.StringIO)
    def test_foo_one(mock_stdout):
        parse_from_file(
            '/Users/gmaddodi/Downloads/control_systems_abstractions/tests/parser_tests/nonlinear_systems_utils/incorrect_hyperbox_states_key.txt')
        assert mock_stdout.getvalue() == "Incorrect key string on line : 1"

    # Solution two: testing print with with-statement
    def test_foo_two(self):
        with patch('sys.stdout', new=io.StringIO()) as fake_out:
            parse_from_file(
                '/Users/gmaddodi/Downloads/control_systems_abstractions/tests/parser_tests/nonlinear_systems_utils/incorrect_hyperbox_states_key.txt')
            self.assertEqual(fake_out.getvalue(), "Incorrect key string on line : 1\n")


