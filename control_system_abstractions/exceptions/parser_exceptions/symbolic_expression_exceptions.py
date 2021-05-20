from .general_parser_exception import GenericParsingException


class ArbitraryVariableNamingException(GenericParsingException):
    """Raised when the variables' character in symbolic expression is not the expected one"""
    def __init__(self, found, allowed):
        self.base_msg = f'Invalid Character Symbol Found in Expression. Found: {found}. Allowed: {allowed}. '
        super().__init__(self.base_msg)


# TODO: Replace this by two exceptions:
#       One that is raised if the number after a variable is not in certain set
#       One that is raised if the number are not sequential starting from 1.
class ArbitraryVariableNumberingException(GenericParsingException):
    """Raised when the number following the variables' character in symbolic expression is not sequential starting
    from 1"""
    base_msg = 'Variable character numbering not starting from 1. Temporary error, to be replaced by others.'


class IncorrectSymbolicExpressionException(GenericParsingException):
    """Raised when the symbolic expression is invalid"""
    base_msg = 'Invalid Symbolic Expression Found. '


class IncorrectNumOfSymbolicExpressionException(GenericParsingException):
    """Raised when the number of symbolic expressions is not as expected"""
    def __init__(self, found, exp):
        self.base_msg = f'Invalid Number of Symbolic Expressions. Expected f{exp}, Found: {found}. '
        super.__init__(self.base_msg)
