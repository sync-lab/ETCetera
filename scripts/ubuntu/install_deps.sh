#!/bin/bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo 'This script must be run as root' >&2
  exit 1
fi
apt-get update
apt-get install -y --no-install-recommends $(tr '\n' ' ' <<EOF
gcc
g++
libpq-dev
python3.8-dev
python3.8-pip
python3.8-venv
python3.8-wheel
libgmp3-dev
libmpfr-dev
libgsl0-dev
gnuplot
glpk-doc
glpk-utils
libglpk-dev
flex
bison
EOF
)
