from .general_parser_exception import GenericParsingException


class IncorrectMatrixBoundaryException(GenericParsingException):
    """Raised when the matrix boundary of the format [...] is missing"""
    base_msg = 'Missing Or Incorrect Matrix Boundary of Format [...]. '


class IncorrectVectorBoundaryException(GenericParsingException):
    """Raised when the vector boundary of the format [...] is missing"""
    base_msg = 'Missing Or Incorrect Vector Boundary of Format [...]. '


class NonnumericValueInMatrixException(GenericParsingException):
    """Raised when non-numeric values found in a matrix definition"""
    base_msg = 'Found Non-Numeric Values in Matrix Definition. '


class NonnumericValueInVectorException(GenericParsingException):
    """Raised when non-numeric values found in a vector definition"""
    base_msg = 'Found Non-Numeric Values in Vector Definition. '


class IncorrectMatrixDefinitionException(GenericParsingException):
    """Raised when the matrix does not contain same number of columns for each row"""

    def __init__(self, row, row_len, temp_len):
        self.base_msg = f'Row Does Not Have Expected Length: [{row}]. Expected Length: {row_len}, Found: {temp_len}'
        super().__init__(self.base_msg)


class MatricesUnequalRowsException(GenericParsingException):
    """Raised when multiple matrices supposed to be having same number of rows but do not"""

    def __init__(self, matrix, row_len, temp_len):
        self.base_msg = f'Matrix Does Not Have Expected Number of Rows: {matrix}. Expected: {row_len}, Found: {temp_len}. '
        super().__init__(self.base_msg)


class MultipleMatricesSpecifiedException(GenericParsingException):
    """Raised when the multiple matrices is specified when a single matrix is expected"""
    base_msg = 'Expected Single Matrix, Found Multiple. '


class MatrixNotQuadraticException(GenericParsingException):
    """Raised when the input matrix not quadratic"""
    base_msg = 'Input Matrix Is Not Quadratic. '
