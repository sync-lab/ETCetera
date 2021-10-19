#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 16 14:38:58 2020
"""

import networkx as nx


class TrafficAutomaton:
    def __init__(self, regions):
        self.regions = [r for r in regions]
        Ei = [(i, j) for j, d in enumerate(regions) for i, s in enumerate(regions) if d[:-1] == s[1:]]
        print(Ei)
        self.G = nx.DiGraph()
        self.G.add_edges_from(Ei)
        # prop, hist, attractors = gt.label_components(self.G, attractors=True)
        self.y = None

    def all_behavioral_cycles(self):
        for c in nx.recursive_simple_cycles(self.G):
            new_c = [self.regions[x][0] for x in c]
            yield new_c

    def fill_props(self):
        for x in self.regions:
            self.G._node[1]['name'] = x[0]

    def plot(self):
        nx.draw_networkx(self.G)


if __name__ == '__main__':
    import pickle
    with open('../../saves/traffic1.pickle', 'rb') as f:
        r1 = pickle.load(f)

    import matplotlib.pyplot as plt
    r1.refine()
    print(r1.regions)
    a = TrafficAutomaton(r1.regions)
    a.plot()
    plt.show()
    print(r1.regions)

