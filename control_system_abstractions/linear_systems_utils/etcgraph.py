# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-
# """
# Created on Sat May 16 14:53:58 2020
#
# @author: ggleizer
# """
#
# import graph_tool.all as gt
#
#
# class TrafficAutomaton:
#     def __init__(self, regions):
#         self.regions = [r for r in regions]
#         Ei = [(i, j) for j, d in enumerate(regions) for i, s in enumerate(regions) if d[:-1] == s[1:]]
#         print(Ei)
#         self.G = gt.Graph()
#         self.G.add_edge_list(Ei)
#         # prop, hist, attractors = gt.label_components(self.G, attractors=True)
#
#     def all_behavioral_cycles(self):
#         print('')
#         for c in gt.all_circuits(self.G):
#             new_c = [self.regions[x][0] for x in c]
#             yield new_c
#
#     def fill_props(self):
#         print('')
#         y = self.G.new_vertex_property('int', [x[0] for x in self.regions])
#
#         self.y = y
#
#     def plot(self):
#         print('')
#         self.fill_props()
#         gt.graph_draw(self.G, vertex_text=self.y)
