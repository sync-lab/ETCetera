from ..sentient_base_exception import SentientBaseException

class GenericParsingException(SentientBaseException):
    """ General Parser Exception. """
    base_msg = 'General Parsing Exception Occurred. '

    def __init__(self, msg=None, param=None):
        if msg is None:
            msg = self.base_msg
        if param is not None:
            msg += ','.join(map(str, param))
        super().__init__('Parsing Error: ' + msg)

class IllegalKeyException(GenericParsingException):
    base_msg = 'Illegal Key Specified: '

class IncompleteInputFileException(SentientBaseException):
    def __init__(self, l):
        msg = f'Not enough input fields were specified in the input file. Missing are: {*l,}'
        super().__init__(msg)

class NonnumbericValuesFoundException(GenericParsingException):
    """Raised when non-numeric values found in input"""
    base_msg = 'Found Unexpected Non-Numeric Value. '


class EmptyValueException(GenericParsingException):
    """Raised when the value of key is empty"""
    base_msg = 'Unexpected Empty Value Found. '


class MultipleValuesFoundException(GenericParsingException):
    """Raised when one value is expected but multiple specified"""
    base_msg = 'Found Multiple Values, But Expected Only One. '


class NotPositiveRealNumberException(GenericParsingException):
    """Raised when a negative number is specified when a non-negative real number is expected"""
    base_msg = 'Expected Non-Negative Number But Given Negative Number. '


class IncorrectSyntaxException(GenericParsingException):
    """Raised when the incorrect syntax is encountered"""
    base_msg = 'Incorrect Syntax. '


class MultipleScalarsSpecifiedException(GenericParsingException):
    """Raised when multiple scalar values separated by comma is found when a single value is expected"""
    base_msg = 'Expected Single Scalar, But Found Multiple. '

class IncorrectNumberOfItemsInListException(GenericParsingException):
    def __init__(self, list, exp, found):
        msg = f'Expected {exp} items in list {list}, but found {found}. '
        super().__init__(msg)
# class GenericParsingException(BaseException):
#     """Raised when any generic exceptions occurs"""
#
#     def __init__(self, msg):
#         self.msg = msg
