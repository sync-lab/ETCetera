# -*- coding: utf-8 -*-

"""
Created on Sat May 16 14:53:58 2020

@author: gmaddodi
"""

import re
import sympy as sp
import tokenize

from control_system_abstractions.exceptions.parser_exceptions.symbolic_expression_exceptions import \
    ArbitraryVariableNamingException, ArbitraryVariableNumberingException, IncorrectSymbolicExpressionException
from control_system_abstractions.exceptions.parser_exceptions.general_parser_exception import \
    NonnumbericValuesFoundException, EmptyValueException, IncorrectSyntaxException
from control_system_abstractions.exceptions.parser_exceptions.vector_matrix_syntax_exceptions import \
    IncorrectMatrixBoundaryException, NonnumericValueInMatrixException, IncorrectMatrixDefinitionException


def check_keyvalue_syntax(split_exp, search_exp, line):
    """
	The function checks if a string can be split with given delimiter/s and also a given pattern exists in the
    split string. The function raises an error otherwise and ends the program execution.

    Parameters
    ----------
        split_exp : string
            A string of delimites to split.
        search_exp : string
    		A regular expression string specifying the pattern to be present in the string.
        line  : string
            A string to be split and searched for the given split and serach pattern.
        line_num : int
            An interger specifying the line in the input file.

    Returns
    -------
        value : list
            A list of values matching the specified search pattern.
    """
    try:
        value = re.split(split_exp, re.search(search_exp, line).group(1))   # Split the string with delimiter and search for the pattern
        value = list(filter(None, value))   # Remove space and empty characters from the list
        value = list(filter(str.strip, value))
        return value
    except Exception:
        raise IncorrectSyntaxException


def check_if_numerical_values(list_to_check):
    """
	The function checks if a list of values are numerals. The function enusres that the data entered are number and
    characters ot special symbols. Raises an exception and end execution if data are not numbers.

    Parameters
    ----------
        list_to_check : list
            A list of values.
        line_num : int
            An interger specifying the line in the input file.
    """
    # Iterate over all the values of the list, strip and '+' or '-' indicating if the value is a positive or negative
    # number and check if the value is a number
    is_all_numbers = all(item.lstrip('-+').replace('.', '', 1).isdigit() for item in list_to_check)
    # Check values are number or length of list is 0, then invalid data specified
    if (not is_all_numbers) or len(list_to_check) == 0:
        raise NonnumbericValuesFoundException
    else:
        pass


def check_matrix_syntax(matrix_str):
    """
	The function checks if a string can be converted to a matrix given values with a row are separated by ' ' and
    values of columns are sperated by '; '. It also checks if all rows have same number of values. Raises an exception
    and terminates execution otherwise.

    Parameters
    ----------
        matrix_string : string
            A string containing values separated by ' ' and '; '.
        line_num : int
            An interger specifying the line in the input file.

    Returns
    -------
        A tuple containing row and column length to shape a list to a matrix.
    """

    row_len = 0  # Variable to hold length of each row
    try:
        matrix_str_searched = re.search('\[(.*)\]', matrix_str).group(1)   # Search matrix string for '[ .. ]'
    except Exception:
        raise IncorrectMatrixBoundaryException

    # Check if pattern like '[1 2] 3 4' exists
    if not len(re.findall(r'\d+', matrix_str)) == len(re.findall(r'\d+', matrix_str_searched)):
        raise IncorrectMatrixBoundaryException

    # Check all values within [] are numbers
    matrix_split = list(filter(None, re.split(' |;', matrix_str_searched)))
    if not all(item.lstrip('-+').replace('.', '', 1).isdigit() for item in matrix_split):
        raise NonnumericValueInMatrixException

    # Split the matrix string using ';', which indicates a new row and check num of columns for each row
    for row in matrix_str_searched.split(';'):
        if row_len == 0 or len(list(filter(None, row.split(' ')))) == row_len:  # Check if first row or number of digits is same as previous row
            row_len = len(list(filter(None, row.split(' '))))   # Get length of split string
        else:
            raise IncorrectMatrixDefinitionException

    # Calculate the shape of the matrix, i.e (num of rows, num of cols)
    total_len = len((list(filter(None, list(filter(str.strip, re.split(' |;', matrix_str_searched)))))))
    return (int(total_len / row_len), row_len)


def check_symbols_in_exprs(allowed_chars, sym_expr):
    """
	The function checks if a mathematial expression contains only the specified characters as variables.
    The function returns a dictionary containing key as the character and value as list of sympy symbols
    in the expression starting with the character. The function raises an exception if the expression contains
    variables with characters that are not in allowed characters. Raises an exception otherwise and end the execution.

    Parameters
    ----------
        allowed_chars : list of characters
            A list containing alphabetic characters.
        sym_expr : string
            String to be convertable to a sympy expression.
        line_num : int
            An interger specifying the line in the input file.

    Returns
    -------
        Dictionary of keys as specified allowed characters and values as list of variable starting with the character.
    """
    list_sympy_funcs = ['sin', 'cos', 'tan']
    try:
        sp.sympify(sym_expr)  # Check expressions are legit use of sin, cos, ...
    except Exception:
        raise IncorrectSymbolicExpressionException

    for item in re.findall(r'\w+', sym_expr):
        if not item.isdigit() and not re.split('(\d.*)', item)[0] in list_sympy_funcs:  # Substring should not be number or sympy function
            if len(re.split('(\d.*)', item)) == 1 or not (re.split('(\d.*)', item)[0] in allowed_chars):    # If substring is like 'x' or not in allowed chars
                raise ArbitraryVariableNamingException
            elif re.split('(\d.*)', item)[1] == '0':
                raise ArbitraryVariableNumberingException
            else:
                pass


def check_symbolic_expr(sym_expr):
    """
	The function checks if a mathematial expression stiring can be a sympy expression. Returns the expression if
    the string can be converted else raises an error.

    Parameters
    ----------
        sym_expr : string
            String to be convertable to a sympy expression.
        line_num : int
            An interger specifying the line in the input file.

    Returns
    -------
        Sympy expression of the specified string.
    """
    try:
        return sp.sympify(sym_expr)     # Check if expression can be converted to a Sympy expression
    except Exception:
        raise IncorrectSymbolicExpressionException

