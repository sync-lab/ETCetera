class DataObjectGeneralException(Exception):
    """Raised when generic exceptions when using LPData class"""
    def __init__(self, msg):
        self.msg = msg