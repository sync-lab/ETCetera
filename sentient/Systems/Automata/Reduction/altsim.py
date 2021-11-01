#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct  8 10:20:35 2021

@author: ggleizer
"""

from itertools import product, combinations, permutations
from collections import defaultdict
from copy import copy,deepcopy
import logging

''' alur style '''


def minimize_alternating_simulation_equivalence(S, H, X0, Hrel=None):
    logging.info('Computing maximal alternating simulation relation')
    R = maximal_alternating_simulation_relation(S, H, Hrel=Hrel)
    logging.info('Obtaining the quotient system')
    SQ, HQ, simulated, RQ, S0, X0Q = make_quotient(S, H, R, X0)
    logging.info('Removing irrational controller actions')
    Sr = remove_controller_actions(SQ, RQ)
    logging.info('Removing irrational environment actions and unreachable'
                 ' states')
    X0r = remove_initial_states(Sr, RQ, X0Q)
    Sr = delete_transitions(Sr, simulated, X0r)
    HQ = {q:y for q,y in HQ.items() if q in Sr}
    return Sr, HQ, X0r


def initial_alternating_sim(H, H2, Hrel=None):
    #print(Hrel)
    if Hrel is None:
        return set((x1,x2) for x1,y1 in H.items() for x2,y2 in H2.items()
                   if y1 == y2)
    else:
        return set((x1,x2) for x1,y1 in H.items() for x2,y2 in H2.items()
                   if (y1,y2) in Hrel)


def is_alternating(xa, xb, R, Sa, Sb):
    # Verify if (xa,xb) satisfy condition 3.
    # It assumes conditions 1 and 2 hold
    for ua, post_ua in Sa[xa].items():
        for ub, post_ub in Sb[xb].items():
            for xbp in post_ub:
                if not any((xap,xbp) in R for xap in post_ua):
                    break  # this ub does not work
            else:
                # This ub worked! Move to the next one
                break # for every ub
        else:
            # No ub worked
            return False
    # If we reach here, then every ua worked
    return True


def maximal_alternating_simulation_relation(S, H, Hrel=None, S2=None, H2=None):
    if S2 is None:
        S2 = S
        H2 = H

    R = initial_alternating_sim(H, H2, Hrel=Hrel)

    while True:
        Rnew = set()
        for (xa, xb) in R:
            if is_alternating(xa, xb, R, S, S2):
                Rnew.add((xa,xb))
        if Rnew == R:
            break
        R = Rnew.copy()

    return R


def make_quotient(S, H, R, X0=None):
    S0 = {x:set() for x in S} #dictionary, key and its corresponding items are equivalent
    for (x,y) in R: #looks for symmetric entries and appends states equivalent to x to the list indexed by x
        if (y,x) in R:
            S0[x].add(y)

    # Freeze sets
    S0 = {x:frozenset(val) for x, val in S0.items()}

    # Create quotient set and output map
    Q = set(S0.values())

    HQ = {q : H[next(x for x in q)] for q in Q}
    if X0:
        X0Q = {S0[x] for x in X0}
    else:
        X0Q = None

    # AQ is the quotient system. Initialize it with quotient states.
    SQ = {q:dict() for q in Q}

    # Populate transitions
    for x, tran in S.items():
        q = S0[x]
        ptran = dict()
        for a, post in tran.items():
            # Get corresponding quotient states
            ptran[a] = {S0[xp] for xp in post}
        SQ[q] = ptran

    # Now create a dictionary of simulated states
    # simulated[q] = set of states that are strictly simulated by states q.

    simulated = {q:set() for q in Q}
    RQ = set()
    for q in Q:
        for (x,y) in R:
            if x in q:
                RQ.add((q, S0[y]))
                if y not in q:
                    simulated[q].add(S0[y])
    return SQ, HQ, simulated, RQ, S0, X0Q


def make_quotient_bisim(S, H, R, X0=None):
    S0 = {x:set() for x in S} #dictionary, key and its corresponding items are equivalent
    for (x,y) in R: #looks for symmetric entries and appends states equivalent to x to the list indexed by x
        if (y,x) in R:
            S0[x].add(y)

    # Freeze sets
    S0 = {x:frozenset(val) for x, val in S0.items()}

    # Create quotient set and output map
    Q = set(S0.values())

    HQ = {q : H[next(x for x in q)] for q in Q}
    if X0:
        X0Q = {S0[x] for x in X0}
    else:
        X0Q = None

    # AQ is the quotient system. Initialize it with quotient states.
    SQ = {q:dict() for q in Q}

    for q in Q:
        candidates = {x: set() for x in q}
        for x in q:
            for u,xp in S[x].items():
                for xpp in xp:
                    candidates[x].add((u,S0[xpp]))
        print(candidates)
        good = set.intersection(*candidates.values())
        ptran = dict()
        for (u,post) in good:
            try:
                ptran[u].add(post)
            except KeyError:
                ptran[u] = {post}
        SQ[q] = ptran


    # Now create a dictionary of simulated states
    # simulated[q] = set of states that are strictly simulated by states q.

    simulated = {q:set() for q in Q}
    RQ = set()
    for q in Q:
        for (x,y) in R:
            if x in q:
                RQ.add((q, S0[y]))
                if y not in q:
                    simulated[q].add(S0[y])
    return SQ, HQ, simulated, RQ, S0, X0Q

def remove_controller_actions(S, R):
    rem_u = {x:set() for x in S}  # Actions marked to be removed
    for x, tran in S.items():
        for ((a, posta),(b, postb)) in permutations(tran.items(),2):
            if a in rem_u[x] or b in rem_u[x]:  # already marked for removal
                continue
            for xb in postb:
                if not any((xa,xb) in R for xa in posta):
                    break  # for this xb, no xa...
            else: # for every xb there is xa!
                rem_u[x].add(a)

    # Now remove actions
    for x, us in rem_u.items():
        for u in us:
            del S[x][u]

    return S


def remove_initial_states(S, R, X0):
    remx0 = set()
    for x1, x2 in permutations(X0, 2):
        if (x1,x2) in R:
            remx0.add(x2)
    return X0 - remx0


def delete_transitions(S, simulated, X0):
    ''' Main steps to remove transitions'''
    l = list(X0)
    visited = set()
    while len(l):
        # Deleting transitions that lead to a state when there is another
        # transition on same label leading to state that simulates it
        # alternatingly
        b = l[0]
        #print("list = " + str(l))
        if b not in visited:
            l.pop(0)
            visited.add(b)
            for label, post_states in S[b].items():
                #print("label = " + str(label))
                #print("post_states = " + str(post_states))
                # Remove simulated states
                for aaa in post_states:
                    S[b][label] = S[b][label] - simulated[aaa]
                #print("Pb = "+str(S[b][label]))
                # Mark remaining post states to visit
                for p in S[b][label]:
                    if p not in visited:
                        l.append(p)
                        # print("append = " + str(p))
        else:
            l.pop(0)

    ''' Delete unreachable states '''
    for aaa in simulated.keys():
        if aaa not in visited:
            S.pop(aaa)

    return S


def stats(S, X0):
    ''' Compute statistics '''
    num_states = len(S.keys())
    num_initial_states = len(X0)
    num_transitions = sum(1 for x,y in S.items()
                          for l,p in y.items()
                          for xp in p)
    return num_states, num_initial_states, num_transitions


if __name__ == '__main__':
    ''' Some test data '''

    B = {'q01' : {'w': {'q02'}, 's': {'q01'}},
     'q02' : {'s': {'q01', 'q11'}},
     'q11' : {'w': {'q12'}, 's': {'q11'}},
     'q12' : {'w': {'q13'}, 's': {'q11'}},
     'q13' : {'s': {'q11'}}
     }

    H = {'q01': 'T', 'q02': 'W', 'q11': 'T', 'q12' : 'W', 'q13': 'W'}


    Borig = {'q01' : {'w': {'q02'}, 's': {'q01'}},
             'q02' : {'w': {'q01', 'q11'}, 's': {'q01', 'q11'}},
             'q11' : {'w': {'q12'}, 's': {'q11'}},
             'q12' : {'w': {'q13'}, 's': {'q11'}},
             'q13' : {'w': {'q11', 'q14'}, 's': {'q11'}},
             'q14' : {'w': {'q11'}, 's': {'q11'}}}

    Horig = {'q01': 'T', 'q02': 'W', 'q11': 'T', 'q12' : 'W', 'q13': 'W',
             'q14': 'W'}

    X0orig = {'q01', 'q11'}

    n, n0, m = stats(Borig, X0orig)
    print(f'Original system: {n} states ({n0} initial) and {m} transitions.')
    print(Borig)

    Sr, HQ, X0r = minimize_alternating_simulation_equivalence(Borig,
                                                              Horig,
                                                              X0orig)

    n, n0, m = stats(Sr, X0r)
    print(f'Minimal system: {n} states ({n0} initial) and {m} transitions.')
    print(Sr)