class LPGeneralException(Exception):
    """Raised when generic exceptions when using 'LPData' class"""
    def __init__(self, msg):
        self.msg = msg


class LPOptimizationFailedException(Exception):
    """Raised when optimization fails when using LPData class"""
    pass
