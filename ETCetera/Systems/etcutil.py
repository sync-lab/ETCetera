import scipy.linalg as la
import numpy as np

class ETCUtilError(Exception):
    pass


'''Linear algebra'''

def is_positive_definite(A):
    return (la.eigvals(A) > 0).all()


def is_positive_semidefinite(A):
    return (la.eigvals(A) >= 0).all()

# Some utility linear control functions
def worst_eig(t, A, B, K):
    nx, nu = B.shape
    Abar = np.block([[A, B], [np.zeros((nu, nx+nu))]])
    Phibar = la.expm(Abar*t)
    Phip = Phibar[0:nx, 0:nx]
    Gammap = Phibar[0:nx, nx:]
    return max(np.abs(la.eig(Phip + Gammap @ K)[0]))

def normalize_state_to_lyapunov(x, P, V_target):
    """Normalize state x such that x'Px = V_target.

    Parameters
    ----------
    x : np.array
        The sate to be normalized
    P : TYPE
        The Lyapunov matrix
    V_target : TYPE
        The desired Lyapunov function value

    Returns
    -------
    np.array
        The normalized state

    """
    return x * np.sqrt(V_target / (x @ P @ x))