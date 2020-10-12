class IncorrectMatrixBoundaryException(Exception):
    """Raised when the matrix boundary of the format [...] is missing"""
    pass


class IncorrectVectorBoundaryException(Exception):
    """Raised when the vector boundary of the format [...] is missing"""
    pass


class NonnumericValueInMatrixException(Exception):
    """Raised when non-numeric values found in a matrix definition"""
    pass


class NonnumericValueInVectorException(Exception):
    """Raised when non-numeric values found in a vector definition"""
    pass


class IncorrectMatrixDefinitionException(Exception):
    """Raised when the matrix does not contain same number of columns for each row"""
    pass

class MatricesUnequalRowsException(Exception):
    """Raised when multiple matrices should supposed to be having same number of rows"""
    pass

class MultipleMatricesSpecifiedException(Exception):
    """Raised when the multiple matrices is specified when a single matrix is expected"""
    pass


class MatrixNotQuadraticException(Exception):
    """Raised when the input matrix not quadratic"""
    pass