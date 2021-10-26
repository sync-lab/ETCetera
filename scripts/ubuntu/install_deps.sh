#!/bin/bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'This script must be run as root' >&2
  exit 1
fi
apt-key adv --keyserver keys.openpgp.org --recv-key 612DEFB798507F25
apt-add-repository "deb [ arch=amd64 ] https://downloads.skewed.de/apt `lsb_release -c -s` main"
apt-get update
apt-get install python3-graph-tool -y --no-install-recommends
apt-get install -y --no-install-recommends $(tr '\n' ' ' <<EOF
gcc
g++
gfortran
libpq-dev
python3.8-dev
python3.8-venv
libgmp3-dev
libmpfr-dev
libgsl0-dev
gnuplot
glpk-doc
glpk-utils
libglpk-dev
flex
bison
ninja-build
graphviz
graphviz-dev
EOF
)
