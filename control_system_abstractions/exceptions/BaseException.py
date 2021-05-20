class BaseException(Exception):
    """
    Base Exception Class. This is where every custom exception will derive from.
    """
    def __init__(self, msg=None):
        if msg is None:
            msg = "General SENTIENT Exception Occurred."

        super().__init__(msg)
