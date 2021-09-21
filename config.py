import os

root_path = os.path.dirname(os.path.abspath(__file__))

save_path = os.path.join(root_path, 'saves')
strat_path = os.path.join(root_path, 'strategy')

smt_path = os.path.join(root_path, 'SMT_files')  # path where aux files are stored #'./SMT_files'#

dreal_path = '/home/ivo/Master_Thesis_Libs/dreal4/bazel-bin/dreal/dreal'
dreach_path = '/home/ivo/Master_Thesis_Libs/dReal-3.16.06.02-linux/bin/dReach'
flowstar_path = '/home/ivo/Master_Thesis_Libs/flowstar-2.0.0/flowstar-2.0.0/flowstar'

VERIFYTA = '/home/ivo/UPPAAL/uppaal64-4.1.20-stratego-7/bin-Linux/verifyta'
