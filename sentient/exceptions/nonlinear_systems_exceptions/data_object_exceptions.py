class DataObjectGeneralException(Exception):
    """Raised when generic exceptions occurs with 'InputDataStructureNonLinear' class"""
    def __init__(self, msg):
        self.msg = msg