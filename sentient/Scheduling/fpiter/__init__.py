from . import enum
from . import bdd


def controlloop(abstraction, maxLate: int = None, maxLateStates: int = None, ratio: int = 1, use_bdd=True):
    if use_bdd:
        return bdd.controlloop(abstraction, maxLate=maxLate, maxLateStates=maxLateStates, ratio=ratio)
    else:
        return enum.controlloop(abstraction, maxLate=maxLate, maxLateStates=maxLateStates, ratio=ratio)


def system(cl: list):
    if not all(type(x) == type(cl[0]) for x in cl):
        print('Please specify same type of control loops')
        return None
    elif not all(x._h == cl[0]._h for x in cl):
        print('All control loops should share same sampling period')
    else:
        if type(cl[0]) == enum.controlloop:
            return enum.system(cl)
        elif type(cl[0]) == bdd.controlloop:
            return bdd.system(cl)
        else:
            return None
