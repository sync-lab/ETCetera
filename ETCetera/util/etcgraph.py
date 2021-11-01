#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat May 16 14:53:58 2020

@author: ggleizer
"""

import graph_tool.all as gt
import numpy as np
import scipy.sparse.linalg as sla
import scipy.sparse as sparse
import scipy.linalg as la
from scipy.sparse.linalg.eigen.arpack import ArpackNoConvergence


class TrafficAutomaton:
    r"""
    Weighted automaton (with outputs) representing a traffic model.

    Creates a graph-tool graph based on input regions, which are encoded
    as sequences of numbers (tuples), and builds edges following the domino
    rule. That is, s --> d iff all_but_last(d) is a prefix of
    all_but_first(s). For a region r, its output is r[0] and the weight of
    an edge is r[0] for every r --> r\' in the edge set.

    This class is used to perform operations that are faster or more
    convenient to use a graph object.


    Parameters
    ----------
    regions : set of tuples
        The abstraction state space X.


    Attributes
    ----------

    regions : list of tuples
        The abstraction state space X.
    y : list of ints
        Corresponding outpts
    G : gt.Graph
        The graph-tool graph object representing the automaton. G
        has the edge propery weight and the vertex property y.


    Methods
    -------

    all_behavioral_cycles :
        Iterator on all cycles of the graph (not suitable for large/dense
        graphs).

    generate_components :
        Generate strongly connnected components of the graph.

    minimum_average_cycle :
        Compute the minimum average cycle using Karp\'s algorithm.

    plot :
        Plot the automaton as a graph, with outputs as node labels.

    """

    def __init__(self, data, strat=None):
        self.has_multiple_actions = False
        self.strat = strat

        if type(data) is dict:
            Ei, w = self._init_transition(data)
        else:
            Ei = self._init_regions(data)
            w = None

        # Build graph
        self.G = gt.Graph()
        self.G.add_edge_list(Ei)

        # Add output and weight properties to the graph
        self.fill_props(w)

        # Initialize components and condensation graph variables
        self.comp = None
        self.CG = None

        # Initialize component value list, in case values are later computed
        self.min_v = None
        self.max_v = None
        self.min_vs = None
        self.max_vs = None

    def _init_regions(self, regions):
        # Use a list (instad of set) of regions for indexing
        self.regions = [r for r in regions]

        # Build the list of edges using the domino rule
        Ei = [(i,j) for j,d in enumerate(regions)
              for i,s in enumerate(regions) if d[:len(s)-1] == s[1:len(d)+1]]

        return Ei

    def _init_transition(self, transition):
        regions = set(x[0] for x in transition)
        self.has_multiple_actions = len(transition) > len(regions)

        # Use a list (instad of set) of regions for indexing
        self.regions = [r for r in regions]
        Ei = [(self.regions.index(x[0]),self.regions.index(y))
              for x,post in transition.items() for y in post]

        w = [x[1] for x,post in transition.items() for _ in post]
        return Ei, w

    def all_behavioral_cycles(self):
        for c in gt.all_circuits(self.G):
            new_c = [self.regions[x][0] for x in c]
            yield new_c

    def fill_props(self, w):
        # Create weights
        if w is None:
            if self.strat:
                w = [self.strat[s[0]] for d in self.regions
                     for s in self.regions
                     if d[:len(s)-1] == s[1:len(d)+1]]
            else:
                w = [s[0] for d in self.regions for s in self.regions
                     if d[:len(s)-1] == s[1:len(d)+1]]
        weights = self.G.new_edge_property('short', w)
        self.G.edge_properties['weight'] = weights
        self.w = weights

        # Create outputs for plotting
        try:
            y = self.G.new_vertex_property('int', [x[0] for x in self.regions])
        except TypeError:
            y = self.G.new_vertex_property('int',
                                           [x[0][0] for x in self.regions])
        self.G.vertex_properties['output'] = y

        self.y = y

    def plot(self):
        #self.fill_props()
        if self.has_multiple_actions:
            gt.graph_draw(self.G, vertex_text=self.y, edge_text=self.w)
        else:
            gt.graph_draw(self.G, vertex_text=self.y)

    def plot_condensation_graph(self):
        if self.CG is None:
            self.generate_condensation_graph()
        gt.graph_draw(self.CG, vertex_text=self.CG.vp.value)

    def generate_components(self):
        # Generate strongly connected components
        comp, hist, attr = gt.label_components(self.G, attractors=True)
        self.comp = comp
        return comp, hist, attr

    def generate_condensation_graph(self, max_val=True):
        if self.comp is None:
            self.generate_components()
        self.CG, prop = gt.condensation_graph(self.G, self.comp)[0:2]
        self.comp_map = prop

        if self.max_v is not None and self.min_v is not None and max_val:
            self.max_v = self.max_v[prop.a]
            v = self.max_v
        elif self.max_v is None:
            self.min_v = self.min_v[prop.a]
            v = self.min_v
        elif self.min_v is None:
            self.max_v = self.max_v[prop.a]
            v = self.max_v

        if v is not None:
            values = self.CG.new_vertex_property('float', v)
            self.CG.vertex_properties['value'] = values

    def generate_max_avg_cycle_per_state(self):
        if self.max_v is None:
            self.maximum_average_cycle()
        if self.CG is None:
            self.generate_condensation_graph()

        # Generate maximum average cycle per SCC
        w_comp = max_reachable_node_dag(self.CG, self.max_v)

        # Value of state... first sort back to the order of components
        w_comp = np.array([x[1] for x in sorted(zip(self.comp_map.a, w_comp.a))])
        vs_array = w_comp[self.comp.a]
        self.max_vs = vs_array

    def generate_min_avg_cycle_per_state(self):
        max_v = self.max_v
        max_vs = self.max_vs

        self.G.ep.weight.a = -self.G.ep.weight.a
        self.generate_max_avg_cycle_per_state()
        self.min_v = -self.max_v
        self.min_vs = -self.max_vs
        self.G.ep.weight.a = -self.G.ep.weight.a

        # Recover max values
        self.max_v = max_v
        self.max_vs = max_vs

    def maximum_average_cycle(self):
        min_v = self.min_v

        self.G.ep.weight.a = -self.G.ep.weight.a
        v, b, c, i = self.minimum_average_cycle()
        self.G.ep.weight.a = -self.G.ep.weight.a
        self.max_v = -self.min_v

        # Recover min value if it was computed
        self.min_v = min_v

        try:  # single value
            v = -v
        except TypeError:  # list of values
            v = [-vv for vv in v]

        return v, b, c, i

    def minimum_average_cycle(self):
        """Compute a minimum average cycle of the traffic model.

        Use Karp's algorithm to compute the minimum average cycle
        of our traffic (weighted) automaton.


        Returns
        -------
        val : float
            The LimAvg value of the minimum average cycle
        behavioral_cycle : tuple
            The minimum average cycle, in terms of outputs
        cycle_regions : list
            The minimum average cycle, as a list of regions (states)
        is_simple_cycle: bool
            Whether the cycle is the only cycle in its SCC.
        """

        # This must be done in each strongly connected component
        comp = self.generate_components()[0]

        # Store the component list
        components = set(comp.a)
        if self.strat:
            val = max(max(r[0]) for r in self.regions)+1  # kbar + 1
        else:
            val = max(max(r) for r in self.regions)+1  # kbar + 1

        val_list = []
        full_val_list = []
        cycle_list = []
        cycle_region_list = []
        is_simple_cycle_list = []
        for component in sorted(components):
            # Filter for this component
            prop = self.G.new_vertex_property('bool',
                                              comp.a == component)
            self.G.set_vertex_filter(prop)  # ...
            if self.G.num_edges() > 0:  # not an isolated node
                new_val, new_cycle = karp(self.G)

                new_is_simple_cycle = all(self.G.get_out_degrees(
                    self.G.get_vertices()) == 1)

                # Unfilter
                self.G.set_vertex_filter(None)

                # Get cycle data
                new_cycle_regions = [self.regions[int(x)] for x in new_cycle]
                new_behavioral_cycle = tuple(r[0] for r in new_cycle_regions)

                # Get SCC data
                SCC_regions = {self.regions[int(x)]
                               for x in self.G.iter_vertices() if prop[x]}

                val_list.append(new_val)
                full_val_list.append(new_val)
                cycle_list.append(new_behavioral_cycle)
                cycle_region_list.append(SCC_regions)
                is_simple_cycle_list.append(new_is_simple_cycle)

                if new_val < val:
                    cycle = new_cycle
                    val = new_val
                    cycle_regions = new_cycle_regions
                    behavioral_cycle = new_behavioral_cycle
                    is_simple_cycle = new_is_simple_cycle

            else:  # Unfilter and move on
                self.G.set_vertex_filter(None)
                full_val_list.append(0.0)

        cycle_regions = [self.regions[int(x)] for x in cycle]
        behavioral_cycle = tuple(r[0] for r in cycle_regions)

        self.min_v = np.array(full_val_list)
        return val, behavioral_cycle, cycle_regions, is_simple_cycle


def karp(G: gt.Graph):
    """An implementation of Karp's algorithm for minimum average cycle.

    The algorithm assumes the graph is strongly connected, but does not
    check this for speed reasons.

    From a table of the k-shortest paths from an arbitrary node (here, 0)
    the minimum values is

    val = min_v max_k (dp[v,n] - dp[v,k])/(n-k)

    Parameters
    ----------
    G : gt.Graph
        A strongly connected graph with the <int or float> edge
        property "weight".

    Returns
    -------
    val : float
        The minimum average (np.inf if graph has no cycles)
    cycle : list, or None
        List of vertex indices that compose the minimum cycle. None if
        the graph has no cycles

    """

    # Table of shortest k-paths
    n = G.num_vertices()

    # NEW: using sparse-matrix version of shortest k paths
    #  (7-10x faster)
    w = w = G.ep['weight']
    W = gt.adjacency(G, w)
    dp, bp = shortest_k_paths_sparse(W)


    #val: min_v max_k (dp[v,n] - dp[v,k])/n-k
    val = np.inf
    v_min = None  # Minimizer

    # This loop is needed to store the vertex associated with the minimizer
    # for recovering the cycle.
    for v in range(n):
        # val_v = -np.inf
        # k_min = None
        val_v = max((dp[v,n] - dp[v,k])/(n-k) for k in range(n)
                    if not np.isinf(dp[v,k]))
        # for k in range(n):
        #     val_this = (dp[v,n] - dp[v,k])/(n-k)
        #     if val_this > val_v:
        #         k_min = k
        #         val_v = val_this
        if val_v < val:
            v_min = v
            val = val_v

    # Now recover the minimizer cycle (Chatuverdi and McConnell, 2017)
    if v_min is not None:
        # A loop to detect a cycle in O(n) as suggested in the paper,
        # walking backwards from the minimizer vertex.
        v = v_min
        walk = [v]
        marked = set((v,))
        for k in range(n,-1,-1):
            v = bp[v,k]
            walk.insert(0,v)
            if v in marked:
                break
            marked.add(v)

        cycle = walk[0:walk.index(v,1)]
        # Make list of vertex pointers instead of indices
        v_dict = {i:v for i,v in enumerate(G.vertices())}
        cycle = [v_dict[i] for i in cycle]
    else:
        cycle = None

    return val, cycle


def shortest_k_paths(G: gt.Graph):
    """Compute the shortest k-paths from vertex 0 to any vertex.

    Part of the minimum average cycle algorithm from Karp. The
    dynamic programming recursion is

    dp[k][v] = min_(u,v in E)(dp[k-1][u] + weight(u,v))

    Parameters
    ----------
    G : gt.Graph
        A strongly connected graph with the <int or float> edge
        property "weight".

    Returns
    -------
    dp : np.array
        Element (i,j) contains the minimum weight of the j-path
        from 0 to node i.
    bp : np.array
        Element (i,j) contains the index of the (j-1)-th vertex
        of the minimum j-path from 0 to i (backpointer).

    """

    # s = G.vertex(0)  # source, always 0
    n = G.num_vertices()

    # Initialize tables: all paths with infinite weight, except 0-->0.
    dp = np.zeros((n,n+1))
    dp[:,:] = np.inf
    dp[0,0] = 0
    # Initialize backpointers (-1 for no path).
    bp = -np.ones((n,n+1), dtype='int')  # Backpointers

    v_dict = {v:i for i,v in enumerate(G.vertices())}

    for k in range(1, n+1):
        for e in G.edges():
            u, v = v_dict[e.source()], v_dict[e.target()]
            w = G.ep.weight[e]
            c = dp[u,k-1] + w
            if c < dp[v,k]:
                dp[v,k] = c
                bp[v,k] = u
    return dp, bp


def shortest_k_paths_sparse(W: sparse.csr_matrix):
    """Compute the shortest k-paths from vertex 0 to any vertex.

    Part of the minimum average cycle algorithm from Karp. The
    dynamic programming recursion is

    dp[k][v] = min_(u,v in E)(dp[k-1][u] + weight(u,v))

    Parameters
    ----------
    W : sparse.csr_matrix
        The weighted adjacency matrix of a strongly connected graph.

    Returns
    -------
    dp : np.array
        Element (i,j) contains the minimum weight of the j-path
        from 0 to node i.
    bp : np.array
        Element (i,j) contains the index of the (j-1)-th vertex
        of the minimum j-path from 0 to i (backpointer).

    """

    # s = G.vertex(0)  # source, always 0
    n = W.shape[0]

    # Initialize tables: all paths with infinite weight, except 0-->0.
    dp = np.zeros((n,n+1))
    dp[:,:] = np.inf
    dp[0,0] = 0
    # Initialize backpointers (-1 for no path).
    bp = -np.ones((n,n+1), dtype='int')  # Backpointers

    for k in range(1, n+1):
        # Next for loop is parallelizable
        for v in range(n):  # csr matrix: iterate through posts
            weights_to_v = W.data[W.indptr[v]:W.indptr[v+1]]
            us_to_v = W.indices[W.indptr[v]:W.indptr[v+1]]
            cost_candidates = dp[us_to_v, k-1] + weights_to_v
            iu_min = np.argmin(cost_candidates)
            dp[v, k] = cost_candidates[iu_min]
            bp[v, k] = us_to_v[iu_min]

    return dp, bp


def max_reachable_node_dag(G, w):
    """Compute the value of the maximum reachable node for any node.

    It assumes we have a directed acyclic graph G, and w is the list of
    vertex weights. Returns a list of weights accordingly."""

    class Visitor(gt.DFSVisitor):
        def __init__(self, w):
            self.w = G.new_vertex_property('float',w)
            self.s = 0
        def discover_vertex(self, u):
            self.s = u
        def examine_edge(self, e):
            self.w[e.source()] = max(self.w[e.source()], self.w[e.target()])
        def finish_vertex(self, u):
            self.w[u] = max(self.w[u], self.w[self.s])

    visitor = Visitor(w.copy())
    gt.dfs_search(G, visitor=visitor)
    return visitor.w






















