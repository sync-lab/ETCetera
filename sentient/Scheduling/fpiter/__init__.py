from . import enum
from . import bdd


def controlloop(abstraction, maxLate: int = None, maxLateStates: int = None, ratio: int = 1, use_bdd=True, label_split_T=True,
                init_steps: int = None):
    if use_bdd:
        return bdd.controlloop(abstraction, maxLate=maxLate,
                               maxLateStates=maxLateStates, ratio=ratio,
                               label_split_T=label_split_T, init_steps=init_steps)
    else:
        return enum.controlloop(abstraction, maxLate=maxLate,
                                maxLateStates=maxLateStates, ratio=ratio,
                                label_split_T=label_split_T, init_steps=init_steps)


def system(cl: list, trap_state=False):
    if not all(type(x) == type(cl[0]) for x in cl):
        print('Please specify same type of control loops')
        return None
    elif not all(x.h == cl[0].h for x in cl):
        print('All control loops should share same sampling period')
    else:
        if type(cl[0]) == enum.controlloop:
            return enum.system(cl, trap_state=trap_state)
        elif type(cl[0]) == bdd.controlloop:
            return bdd.system(cl, trap_state=trap_state)
        else:
            return None
