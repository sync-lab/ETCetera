class ArbitraryVariableNamingException(Exception):
    """Raised when the variables' character in symbolic expression is not the expected one"""
    pass


class ArbitraryVariableNumberingException(Exception):
    """Raised when the number following the variables' character in symbolic expression is not sequential starting
    from 1"""
    pass


class IncorrectSymbolicExpressionException(Exception):
    """Raised when the symbolic expression is invalid"""
    pass


class IncorrectNumOfSymbolicExpressionException(Exception):
    """Raised when the number of symbolic expressions is not as expected"""
    pass
