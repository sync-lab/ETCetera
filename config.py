import os

root_path = os.path.dirname(os.path.abspath(__file__))

save_path = os.path.join(root_path, 'saves')
if not os.path.exists(save_path):
    os.mkdir(save_path)

strat_path = os.path.join(root_path, 'strategy')
if not os.path.exists(strat_path):
    os.mkdir(strat_path)

smt_path = os.path.join(root_path, 'SMT_files')  # path where aux files are stored #'./SMT_files'#
if not os.path.exists(smt_path):
    os.mkdir(smt_path)

dreal_path = '<path>/sentient/third-party/dreal4/bazel-bin/dreal/dreal'
dreach_path = '<path>/sentient/third-party/dReal-3.16.06.02-*/bin/dReach'
flowstar_path = '<path>/sentient/third-party/flowstar-2.0.0/flowstar-2.0.0/flowstar'

VERIFYTA = '<path>/uppaal64-4.1.20-stratego-7/bin-Linux/verifyta'


CPU_COUNT = os.cpu_count()-2