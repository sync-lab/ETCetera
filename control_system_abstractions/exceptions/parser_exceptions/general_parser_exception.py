class NonnumbericValuesFoundException(Exception):
    """Raised when non-numeric values found in input"""
    pass


class EmptyValueException(Exception):
    """Raised when the value of key is empty"""
    pass


class MultipleValuesFoundException(Exception):
    """Raised when one value is expected but multiple specified"""
    pass


class NotPositiveRealNumberException(Exception):
    """Raised when a negative number is specified when a non-negative real number is expected"""
    pass


class IncorrectSyntaxException(Exception):
    """Raised when the incorrect syntax is encountered"""
    pass


class MultipleScalarsSpecifiedException(Exception):
    """Raised when multiple scalar values separated by comma is found when a single value is expected"""
    pass

class GenericParsingException(Exception):
    """Raised when any generic exceptions occurs"""
    def __init__(self, msg):
        self.msg = msg