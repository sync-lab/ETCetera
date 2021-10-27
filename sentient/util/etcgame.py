#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 19 16:40:17 2021

@author: ggleizer
"""

import graph_tool.all as gt
import numpy as np
import scipy.sparse.linalg as sla
import scipy.linalg as la
import time
from tqdm import tqdm

class TrafficGameAutomaton:
    def __init__(self, transition):
        transition = {x:frozenset(y) for x,y in transition.items()}
        G = gt.Graph()

        self.regions = list(set(x[0] for x in transition))
        self.branchpoints = list(set((x[1], y)
                                     for x, y in transition.items()))

        self.nodes = self.regions + self.branchpoints

        # Controllable acations
        Ei = [(self.nodes.index(x[0]), self.nodes.index((x[1], y)))
              for x,y in transition.items()]
        self.UC = range(len(Ei))  # controlled actions

        Ei.extend(set((self.nodes.index((x[1], y)), self.nodes.index(z))
                  for x,y in transition.items() for z in y))
        self.UU = range(len(self.UC), len(Ei))

        G = gt.Graph()
        G.add_edge_list(Ei)

        self.G = G

        # Ranges indicating players
        self.V0 = range(len(self.regions))
        self.V1 = range(len(self.regions),
                        len(self.regions) + len(self.branchpoints))

        # Weights on edges
        w0 = [self.nodes[v][0] for u,v in Ei]  # Player 1: weight is chosen k
        w1 = [self.nodes[u][0] for u,v in Ei]  # Player 2: repeat player 1's weight
        self.w = w0[:len(self.UC)] + w1[len(self.UC):]
        weight = G.new_edge_property('int', self.w)
        self.G.ep['weight'] = weight

        self.fill_plot_props()

    def fill_plot_props(self):
        G = self.G
        player = G.new_vertex_property('int', [0]*len(self.regions) +
                                              [1]*len(self.branchpoints))

        color = G.new_vertex_property('string', ['Red']*len(self.regions) +
                                      ['Aquamarine']*len(self.branchpoints))

        shape = G.new_vertex_property('string', ['circle']*len(self.regions) +
                                      ['square']*len(self.branchpoints))

        w_str = G.new_edge_property('string',
                                    self.w[:len(self.UC)] +
                                    [""]*len(self.UU))

        self.G.vp['player'] = player
        self.G.vp['color'] = color
        self.G.vp['shape'] = shape
        self.G.ep['weight_str'] = w_str

        self.deadline = [x[0] for x in self.regions]
        text = G.new_vertex_property('string', self.deadline +
                                               [""]*len(self.branchpoints))
        self.G.vp['text'] = text

    def plot(self):
        night = time.localtime().tm_hour > 18  # I won't work past midnight
        bg = '#222222' if night else None
        ec = '#dddddd' if night else [0.179, 0.203,0.210, 0.8]
        vc = 'White' if night else 'Black'
        gt.graph_draw(self.G,
                      vertex_text = self.G.vp.text,
                      vertex_fill_color = self.G.vp.color,
                      vertex_shape = self.G.vp.shape,
                      vertex_color = vc,
                      bg_color = bg,
                      edge_color = ec,
                      edge_text = self.G.ep.weight_str,
                      edge_text_color = ec)




# Brim et al (2011) algorithm

def preceq(a, b, MG):
    return b > MG or b >= a

def succeq(a, b, MG):
    return a > MG or a >= b

def prec(a, b, MG):
    return a <= MG and a < b


def circminus(a, b, MG):
    if a <= MG and a - b <= MG:
        return max(a - b, 0)
    else:
        return MG + 1


def arrpreceq(a, b, MG):
    return np.logical_or(b > MG, b >= a)

def arrsucceq(a, b, MG):
    return np.logical_or(a > MG, a >= b)

def arrprec(a, b, MG):
    return np.logical_and(a < b, a <= MG)

def arrcircminus(a, b, MG):
    return np.where(np.logical_and(a <= MG, a - b <= MG),
                    np.maximum(a - b, 0),
                    MG + 1)


#@profile
def solve_energy_game(G, w, V0, V1, f=None):
    W = gt.adjacency(G, w)  # Sparsity is the same as that of the adjacency
    # matrix, even if weight is zero!
    Wc = W.tocsc()  # More efficient for posts (column slicing)
    MG = int(np.sum(np.maximum(-W.min(axis=0).data, 0)))

    #MG = sum(max(0, max(-wu for _, _, wu in G.iter_out_edges(v, [w])))
    #         for v in G.iter_vertices())

    def leq(a, b):
        return preceq(a, b, MG)
    def geq(a, b):
        return preceq(b, a, MG)
    def less(a, b):
        return prec(a, b, MG)

    def minus(a, b):
        return circminus(a, b, MG)

    def arrleq(a, b):
        return arrpreceq(a, b, MG)
    def arrgeq(a, b):
        return arrleq(b, a)
    def arrless(a, b):
        return np.logical_not(arrgeq(a, b))
    def arrminus(a, b):
        return arrcircminus(a, b, MG)

    if f is None:
        L = {v for v in V0
             if all(wu < 0 for _, _, wu in G.iter_out_edges(v, [w]))}
        L.update(v for v in V1
                 if any(wu < 0 for _, _, wu in G.iter_out_edges(v, [w])))
        f = G.new_vp('int', 0)
    else:
        L = {v for v in V0
             if all(less(f[v], minus(f[u], wu))
                    for _, u, wu in G.iter_out_edges(v, [w]))}
        L.update(v for v in V1
                 if any(less(f[v], minus(f[u], wu))
                        for _, u, wu in G.iter_out_edges(v, [w])))

    count = G.new_vp('int', 0)
    for v in V0:
        if v not in L:
            count[v] = sum(1 for _, u, wu in G.iter_out_edges(v, [w])
                           if geq(f[v], minus(f[u], wu)))

    while len(L) > 0:
        v = L.pop()
        intv = int(v)
        old = f[v]
        #wu = Wc[:,intv].data # Very slow
        wu = Wc.data[Wc.indptr[intv]:Wc.indptr[intv+1]]
        #fu = G.get_out_neighbors(G, [f])[:,1]  # Slower
        fu = f.a[Wc.indices[Wc.indptr[intv]:Wc.indptr[intv+1]]]
        vals = arrminus(fu, wu)

        #vals = (minus(f[u], wu) for _, u, wu in G.iter_out_edges(v, [w]))
        # Above: Ultra slow!

        if v in V0:
            fv = np.min(vals)
            f[v] = fv
            #count[v] = sum(1 for _, u, wu in G.iter_out_edges(v, [w])
            #               if geq(f[v], minus(f[u], wu)))
            count[v] = np.count_nonzero(arrgeq(fv, vals))
        elif v in V1:
            fv = np.max(vals)
            f[v] = fv

        fv = f[v]
        for u, _, wu in G.iter_in_edges(v, [w]):
            fu = f[u]
            if less(fu, minus(fv, wu)):
                if u in V0:
                    if geq(fu, minus(old, wu)):
                        count[u] -= 1
                    if count[u] <= 0:
                        L.add(u)
                elif u in V1:
                    L.add(u)

    return f, MG


#@profile
def solve_energy_game_sparse(W, Wc, player, f):
    N = W.shape[0]  # number of vertices

    MG = int(np.sum(np.maximum(-Wc.min(axis=0).data, 0)))

    inL = np.zeros(N, dtype='bool')
    count = np.zeros(N, dtype='int')

    for v in range(N):
        fv = f[v]
        wu = Wc.data[Wc.indptr[v]:Wc.indptr[v+1]]
        posts = Wc.indices[Wc.indptr[v]:Wc.indptr[v+1]]
        fu = f[posts]
        cases = arrprec(fv, arrcircminus(fu, wu, MG), MG)
        if player[v]:  # v in V1
            inL[v] = np.any(cases)
        else:
            v_inL = np.all(cases)
            inL[v] = v_inL
            if not v_inL:
                count[v] = np.sum(np.logical_not(cases))

    while any(inL):
        v = inL.argmax()  # Fast "get first" for bool arrays
        inL[v] = False
        old = f[v]
        #wu = Wc[:,intv].data # Very slow
        wu = Wc.data[Wc.indptr[v]:Wc.indptr[v+1]]
        #fu = G.get_out_neighbors(G, [f])[:,1]  # Slower
        posts = Wc.indices[Wc.indptr[v]:Wc.indptr[v+1]]
        fu = f[posts]
        vals = arrcircminus(fu, wu, MG)

        #vals = (minus(f[u], wu) for _, u, wu in G.iter_out_edges(v, [w]))
        # Above: Ultra slow!

        if player[v]:  # v in V1
            fv = np.max(vals)
            f[v] = fv
        else:
            fv = np.min(vals)
            f[v] = fv
            count[v] = np.count_nonzero(arrsucceq(fv, vals, MG))

        # Now we loop Pre's
        wus = W.data[W.indptr[v]:W.indptr[v+1]]
        pres = W.indices[W.indptr[v]:W.indptr[v+1]]
        fus = f[pres]

        # cases = arrless(fus, arrminus(fv, wus))
        # Ends up being slower

        for u, fu, wu in zip(pres, fus, wus):
            if prec(fu, circminus(fv, wu, MG), MG):
                if player[u]:
                    inL[u] = True
                else:
                    if succeq(fu, circminus(old, wu, MG), MG):
                        count[u] -= 1
                    if count[u] <= 0:
                        inL[u] = True
    return f, MG


def farey(N):
    # Generate the Farey sequence for integer N
    # Based on Problem 4-61 of Concrete Mathematics (Graham, Knuth, Patashnik)
    m = 0
    mm = 1
    n = 1
    nn = N
    FN = [(0,1), (1,N)]
    for x in FN:
        yield x
    while mm < nn:
        fac = (n + N)//nn
        mmm = fac*mm - m
        nnn = fac*nn - n
        yield (mmm, nnn)
        n = nn
        m = mm
        nn = nnn
        mm = mmm


def farey_length(N):
    # https://stackoverflow.com/questions/54513531/farey-sequence-length/54513629#54513629
    dp = dict() # sometimes, dp stands for dynamic programming

    def fl(n):
        if dp.get(n):
            return dp.get(n)
        dp[n] = (n * (n + 3)) // 2 - sum(fl(n // k) for k in range(2, n + 1))
        return dp[n]

    return fl(N)


#@profile
def solve_mean_payoff_game(G, w, V0, V1,
                           both_players_same_weight=False,
                           min_value = None):
    W0 = set()
    W1 = set()
    f = G.new_vp('int', 0)
    Wmax = max(w.a)
    Wmin = min(w.a)
    if min_value is None:
        min_value = Wmin

    # FIXME: remove
    Wmin -= 1

    MG = 1

    # Create player bool value
    player = G.new_vp('bool', [v in V1 for v in G.iter_vertices()])

    # Creating vertex/edge properties to store working arrays
    wp = G.new_ep('int', w.a)
    wprev = G.new_ep('int', 0)
    W = gt.adjacency(G, w)
    Wc = W.tocsc()
    Wp = gt.adjacency(G, wp)
    Wpprev = gt.adjacency(G, wprev)
    Wpc = Wp.tocsc()
    Wpcprev = Wpprev.tocsc()

    fprev = G.new_vp('int', 0)

    # Creating output properties
    nu = G.new_vp('float', 0.0)
    sigma = G.new_vp('object', [set() for v in G.iter_vertices()])

    #
    if both_players_same_weight:
        NV = len(V0)
    else:
        NV = G.num_vertices()

    # For early termination:
    states_without_strategy = len(V0)
    # Main iteration (line 5 of [1])
    num = (Wmax - Wmin) * (farey_length(NV) - 1)
    with tqdm(total=num) as pbar:
        for i in range(Wmin, Wmax):  # no reason to be from -W to W
            iter_farey = farey(NV)
            N, D = next(iter_farey)
            for NN, DD in iter_farey:
                if i + NN/DD < min_value:
                    continue

                fprev.a = f.a[:]
                wprev.a = wp.a[:]  # divided by D
                Wpprev.data = Wp.data[:]
                Wpcprev.data = Wpc.data[:]
                Fprev = (N, D)
                MGprev = MG

                N, D = NN, DD
                wp.a = D*(w.a - i) - N
                Wp.data = D*(W.data - i) - N
                Wpc.data = D*(Wc.data - i) - N  # Faster than another conversion
                np.ceil(D*fprev.a/Fprev[1], f.a, casting='unsafe')
                # f divided by D
                f.a, MG = solve_energy_game_sparse(Wp, Wpc, player, f.a[:])
                for v in G.iter_vertices():
                    if fprev[v] <= MGprev and f[v] > MG:
                        nu[v] = i + Fprev[0]/Fprev[1]
                        if nu[v] >= 0:
                            W0.add(v)
                        else:
                            W1.add(v)
                        if v in V0:  # Add strategy for v
                            post = (u for _, u, wu
                                    in G.iter_out_edges(v, [wprev])
                                    if preceq(circminus(fprev[u], wu, MGprev),
                                              fprev[v], MGprev))
                            sigma[v].update(post)
                            states_without_strategy -= 1

                pbar.update()
                if states_without_strategy == 0:
                    break
            if states_without_strategy == 0:
                    break

    return nu, sigma, W0, W1



if __name__ == '__main__':
    import pickle
    with open('trafficmpg.pickle', 'rb') as f:
        g, wg = pickle.load(f)

    V0 = {1,2,3}
    V1 = {0,4}
    labels = ['x', 'z', 'y', 'w', 'v']
    Ebrim = [(0,1), (1,2), (1,3), (2,0), (2,1), (3,4), (4,1), (4,3)]
    w = [3, -3, 1, 2, 1, -4, 1, 0]
    G = gt.Graph()
    G.add_edge_list(Ebrim)
    ww = G.new_ep('int', w)
    player = G.new_vp('bool', [v in V1 for v in G.iter_vertices()])
    f, MG = solve_energy_game(G, ww, V0, V1)
    print(f.a, MG)

    #nu, sigma, W0, W1 = solve_mean_payoff_game(G, ww, V0, V1)
    #print(nu.a)

    nu, sigma, W0, W1 = solve_mean_payoff_game(g.G, wg, g.V0, g.V1,
                                               True)
    print(nu.a)



