import unittest
import os
from ETCetera.util import construct_nonlinearETC_traffic_from_file
from ETCetera.exceptions import *
import sympy

test_file_name = 'test.txt'

class TestInputNonlinearETC(unittest.TestCase):

    def setUp(self) -> None:
        self.base_file = {
            'Dynamics': '-x1**3 + x1*x2**2, x1*x2**2 - x2*x1**2 + u1',
            'Controller': '-x2**3 - x1*x2**2',
            'Hyperbox States': '[-2.5 2.5], [-2.5 2.5]',
            'Triggering Condition': 'e1**2 + e2**2 - (0.0127*0.3)**2*(x1**2+x2**2)'
        }
        self.solver_options = 'partition_method=grid'

    def tearDown(self) -> None:
        self.base_file = None
        self.solver_options = None

    @classmethod
    def tearDownClass(cls) -> None:
        # os.remove('test.txt')
        pass

    def _write2file(self):
        with open(test_file_name, 'w') as f:
            for (k,v) in self.base_file.items():
                f.write(f'{k}: {v}\n')
            f.write(f'Solver Options: {self.solver_options}')

    def _testfile(self):
        self._write2file()
        return construct_nonlinearETC_traffic_from_file(test_file_name, CreateAbstraction=False)

    def testBaseFile(self):
        try:
            self._testfile()
        except Exception as e:
            self.fail(f'Base File Raised {e}')

    # First Check If Everything Related To Dynamics is checked correctly
    def testDynamicsNumExprs(self):
        with self.subTest('Too many expressions'):
            self.base_file['Dynamics'] += ', x1**3'
            self.assertRaises(IncorrectNumOfSymbolicExpressionException, self._testfile)
        with self.subTest('Too few expressions'):
            self.base_file['Dynamics'] = 'x1**3 + x1*x2**2'
            self.assertRaises(IncorrectNumOfSymbolicExpressionException, self._testfile)

    def testGridPntsPerDimNum(self):
        with self.subTest('Too many dimensions'):
            self.base_file['Grid Points Per Dimension'] = '[3 3 3]'
            self.assertRaises(IncorrectNumberOfItemsInListException, self._testfile)
        with self.subTest('Too few dimensions'):
            self.base_file['Grid Points Per Dimension'] = '[3]'
            self.assertRaises(IncorrectNumberOfItemsInListException, self._testfile)
        with self.subTest('Not given'):
            self.base_file.pop('Grid Points Per Dimension')
            try:
                traffic = self._testfile()
                self.assertEqual(traffic.grid_points_per_dim, [5, 5])
            except:
                self.fail(f'Not given grid points per dim raises exception. Should be set to default value.')

    def testNrSmallAnglesNum(self):
        self.solver_options = 'partition_method=manifold, manifolds_times=[0.01 0.02]'
        with self.subTest('Too many dimensions'):
            self.solver_options += ', nr_cones_small_angles=[3 3 3]'
            self.assertRaises(IncorrectNumberOfItemsInListException, self._testfile)
        with self.subTest('Too few dimensions'):
            self.solver_options += ', nr_cones_small_angles=[3]'
            self.assertRaises(IncorrectNumberOfItemsInListException, self._testfile)
        with self.subTest('Not given'):
            self.solver_options = 'partition_method=manifold, manifolds_times=[0.01 0.02]'
            self.base_file['Dynamics'] = '-x1**3 + x3*x2**2, x1*x2**2 - x2*x1**2 + u1, -x3**3, -x4**3'
            self.base_file['Triggering Condition'] = 'e1**2 + e2**2 + e3**2 + e4**2 - (0.0127*0.3)**2*(x1**2+x2**2+x3**2+x4**2)'
            self.base_file['Hyperbox States'] = '[-2.5 2.5], [-2.5 2.5], [-2.5 2.5], [-2.5 2.5]'
            try:
                traffic = self._testfile()
                self.assertEqual(traffic.nr_cones_small_angles, [4, 4])
            except:
                self.fail(f'Not given nr_small_angles raises exception or is not equal to [4,4]. Should be set to default value.')

    def testControllerNumExprs(self):
        self.base_file['Dynamics'] = 'x1+u2, x1**2*x2 + x2**3 + u1'
        with self.subTest('Too many expressions'):
            self.base_file['Controller'] = '-x1**2 + x2**2, x2, x1'
            self.assertRaises(IncorrectNumOfSymbolicExpressionException, self._testfile)
        with self.subTest('Too few expressions'):
            self.base_file['Controller'] = '-x1**2 + x2**2'
            self.assertRaises(IncorrectNumOfSymbolicExpressionException, self._testfile)

    def testETCForm(self):
        traffic = self._testfile()
        x1, x2, e1, e2 = sympy.symbols('x1 x2 e1 e2')
        x1dot = -x1**3 + x1*x2**2
        x2dot = x1*x2**2 - x2*x1**2 -(x2+e2)**3 - (x1+e1)*(e2+x2)**2
        self.assertEqual(traffic.Dynamics[0].expand(), x1dot.expand())
        self.assertEqual(traffic.Dynamics[1].expand(), x2dot.expand())
        self.assertEqual(traffic.Dynamics[2].expand(), -x1dot.expand())
        self.assertEqual(traffic.Dynamics[3].expand(), -x2dot.expand())

    def testHomogenizeIfNeeded(self):
        with self.subTest('No hom. needed (base file)'):
            traffic = self._testfile()
            x1,x2,e1,e2 = sympy.symbols('x1 x2 e1 e2')
            x1dot = -x1**3 + x1*x2**2
            x2dot = x1 * x2 ** 2 - x2 * x1 ** 2 - (e2 + x2) ** 3 - (e1 + x1) * (e2 + x2) ** 2
            self.assertEqual(traffic.Dynamics[0].expand(), x1dot.expand())
            self.assertEqual(traffic.Dynamics[1].expand(), x2dot.expand())
            self.assertEqual(traffic.Dynamics[2].expand(), -x1dot.expand())
            self.assertEqual(traffic.Dynamics[3].expand(), -x2dot.expand())

        with self.subTest('Needs hom. with specified deg. 5'):
            self.base_file['Dynamics'] = '-x1, x1 ** 2 * x2 + x2 ** 3 + u1'
            self.base_file['Controller'] = '-x2 - x1 ** 2 * x2 - x1 ** 3'
            self.base_file['Deg. of Homogeneity'] = 5
            traffic = self._testfile()
            x1, x2, e1, e2, w1 = sympy.symbols('x1 x2 e1 e2 w1')
            x1dot = -w1**5.0*x1
            x2dot = w1**3.0*x1**2*x2 + w1**3.0*x2**3 - w1**5.0*(x2+e2) - w1**3.0*(x1+e1)**2*(x2+e2) - w1**3.0*(x1+e1)**3
            self.assertEqual(traffic.Dynamics[0].expand(), x1dot.expand())
            self.assertEqual(traffic.Dynamics[1].expand(), x2dot.expand())
            self.assertEqual(traffic.Dynamics[2].expand(), 0)
            self.assertEqual(traffic.Dynamics[3].expand(), -x1dot.expand())
            self.assertEqual(traffic.Dynamics[4].expand(), -x2dot.expand())
            self.assertEqual(traffic.Dynamics[5].expand(), 0)
            self.assertEqual(traffic.Homogeneity_degree, 5)

        with self.subTest('Needs hom. with default deg (=2)'):
            self.setUp()
            self.base_file['Dynamics'] = '-x1, x1 ** 2 * x2 + x2 ** 3 + u1'
            self.base_file['Controller'] = '-x2 - x1 ** 2 * x2 - x1 ** 3'
            traffic = self._testfile()
            x1, x2, e1, e2, w1 = sympy.symbols('x1 x2 e1 e2 w1')
            x1dot = -w1**2*x1
            x2dot = x1**2*x2 + x2**3 - w1**2*(x2+e2) - (x1+e1)**2*(x2+e2) - (x1+e1)**3
            self.assertEqual(traffic.Dynamics[0].expand(), x1dot.expand())
            self.assertEqual(traffic.Dynamics[1].expand(), x2dot.expand())
            self.assertEqual(traffic.Dynamics[2].expand(), 0)
            self.assertEqual(traffic.Dynamics[3].expand(), -x1dot.expand())
            self.assertEqual(traffic.Dynamics[4].expand(), -x2dot.expand())
            self.assertEqual(traffic.Dynamics[5].expand(), 0)
            self.assertEqual(traffic.Homogeneity_degree, 2)

    def testRequiredInputFields(self):
        with self.subTest('Missing Dynamics'):
            self.setUp()
            self.base_file.pop('Dynamics', None)
            self.assertRaises(IncompleteInputFileException, self._testfile)

        with self.subTest('Missing Hyperbox States'):
            self.setUp()
            self.base_file.pop('Hyperbox States', None)
            self.assertRaises(IncompleteInputFileException, self._testfile)

        with self.subTest('Missing Controller'):
            self.setUp()
            self.base_file.pop('Controller', None)
            self.assertRaises(IncompleteInputFileException, self._testfile)

        with self.subTest('Missing Triggering Condition'):
            self.setUp()
            self.base_file.pop('Triggering Condition', None)
            self.assertRaises(IncompleteInputFileException, self._testfile)

    # def testRequireGridPointsPerDimensionIfGridOrNonhom(self):
    #     with self.subTest('When partition=grid and not given'):
    #         self.assertRaises(IncompleteInputFileException, self._testfile)
    #
    #     with self.subTest('When partition=grid and given'):
    #         try:
    #             self.base_file['Grid Points Per Dimension'] = '[3 3]'
    #             self._testfile()
    #         except Exception as e:
    #             self.fail(f'Thrown exception {e} even when "Grid Points Per Dimension" Specified')
    #
    #     with self.subTest('When system is nonhomogeneous and not given'):
    #         self.setUp()
    #         self.solver_options += ', partition_method=manifold'
    #         self.assertRaises(IncompleteInputFileException, self._testfile)
    #
    #     with self.subTest('When partition=grid and given'):
    #         self.setUp()
    #         try:
    #             self.base_file['Grid Points Per Dimension'] = '[3 3]'
    #             self._testfile()
    #         except Exception as e:
    #             self.fail(f'Thrown exception {e} even when "Grid Points Per Dimension" Specified')

    def testRequireManifoldTimes(self):
        self.solver_options += ', partition_method=manifold'
        with self.subTest('Not given'):
            self.assertRaises(IncompleteInputFileException, self._testfile)

        with self.subTest('Not enough times specified'):
            self.solver_options += ', manifolds_times=[0.0002]'
            self.assertRaises(IncorrectNumberOfItemsInListException, self._testfile)

        with self.subTest('Enough times specified'):
            self.solver_options += ', manifolds_times=[0.0004 0.0008 0.0020]'
            print(self.solver_options)
            try:
                self._testfile()
            except Exception as e:
                self.fail(f'Thrown exception {e} even when enough manifold times Specified')

    def testHypBoxDistNotEmpty(self):
        self.base_file['Dynamics'] = '-x1, x1 ** 2 * x2 + x2 ** 3 + d1 + u1'
        self.assertRaises(IncompleteInputFileException, self._testfile)

    def testAllVariablesSequential(self):
        with self.subTest('All Correct (base file)'):
            try:
                self._testfile()
            except Exception as e:
                self.fail(f'Base file does not have sequential variables')

        with self.subTest('Dynamics variables not sequential 1'):
            self.base_file['Dynamics'] = 'x3 * x5 + u1, x3**2 - x5**2'
            self.assertRaises(ArbitraryVariableNumberingException, self._testfile)
        with self.subTest('Dynamics variables not sequential 2'):
            self.base_file['Dynamics'] = 'x1 * x4, x1**2 - x4**2'
            self.base_file['Controller'] = '-x4**3 - x1*x4**2'
            self.assertRaises(ArbitraryVariableNumberingException, self._testfile)
        with self.subTest('Controller variables not sequential 1'):
            self.base_file['Dynamics'] = 'x1 * x2 + u1, x1**2 - x2**2 + u3'
            self.base_file['Controller'] = '-x1 * x2, -x1**2 - x2**2'
            self.assertRaises(ArbitraryVariableNumberingException, self._testfile)
        with self.subTest('Controller variables not sequential 2'):
            self.base_file['Dynamics'] = 'x1 * x2 + u2, x1**2 - x2**2 + u3'
            self.base_file['Controller'] = '-x1 * x2, -x1**2 - x2**2'
            self.assertRaises(ArbitraryVariableNumberingException, self._testfile)
        with self.subTest('Error Variables not sequential 1'):
            self.base_file['Triggering Condition'] = 'e3**2 + e4**2 - (0.0127*0.3)**2*(x1**2+x2**2)'
            self.assertRaises(ArbitraryVariableNumberingException, self._testfile)
        with self.subTest('Disturbace variables not sequential 1'):
            self.base_file['Dynamics'] = 'x3 *  + d1, x3**2 - x4**2 +d3'
            self.assertRaises(ArbitraryVariableNumberingException, self._testfile)
        with self.subTest('Disturbace variables not sequential 1'):
            self.base_file['Dynamics'] = 'x3 *  + d2, x3**2 - x4**2 +d3'
            self.assertRaises(ArbitraryVariableNumberingException, self._testfile)

    def testNumOfErrorVarEqualToDynVar(self):
        with self.subTest('Equal number (base file)'):
            try:
                self._testfile()
            except Exception as e:
                self.fail(f'Number of error variables not equal to state variables in base file.')
        with self.subTest('Nonequal number'):
            self.base_file['Triggering Condition'] = 'e1**2+e2**2+e3**2 - 0.01*(x1**2+x2**3)'
            self.assertRaises(IncorrectNumberOfVariablesSpecifiedException, self._testfile)