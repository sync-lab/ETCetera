#!/bin/bash
set -euxo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  echo 'This script must NOT be run as root' >&2
  exit 1
fi

brew update
brew install gmp mpfr gsl glpk bison flex gnuplot
