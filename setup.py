# !/usr/bin/python
# coding=utf-8

from setuptools import setup
from setuptools.command.install import install
import subprocess
from sys import platform
import shlex
#import lsb_release
import os
import sys
import glob

# List of dependencies installed via `pip install -e .`
# by virtue of the Setuptools `install_requires` value below.

class CustomInstallCommand(install):
    def run(self):
        if platform == "linux" or platform == "linux2":
            print()
            release = ''
            if os.path.isfile('/etc/lsb-release'):
                lines = open('/etc/lsb-release').read().split('\n')
                for line in lines:
                    if line.split('=')[0] == 'DISTRIB_RELEASE':
                        release = line.split('=')[1]
                    else:
                        print()
            else:
                print()
            subprocess.check_call(["sudo","apt", "update"])
            subprocess.run(shlex.split("sudo apt-get install git curl libgmp3-dev libmpfr-dev libgsl0-dev glpk-doc glpk-utils libglpk-dev bison flex gnuplot"))
            if release == "16.04":
                print()
                subprocess.run(shlex.split("curl -fsSL https://raw.githubusercontent.com/dreal/dreal4/master/setup/ubuntu/16.04/install_prereqs.sh -o /tmp/install_prereqs.sh"))
                subprocess.run(shlex.split("sudo bash /tmp/install_prereqs.sh"))
                subprocess.run(shlex.split("rm -f /tmp/install_prereqs.sh"))
            elif release == "18.04":
                print()
                subprocess.run(shlex.split("curl -fsSL https://raw.githubusercontent.com/dreal/dreal4/master/setup/ubuntu/18.04/install_prereqs.sh -o /tmp/install_prereqs.sh"))
                subprocess.run(shlex.split("sudo bash /tmp/install_prereqs.sh"))
                subprocess.run(shlex.split("rm -f /tmp/install_prereqs.sh"))
            elif release == "20.04":
                print()
                subprocess.run(shlex.split("curl -fsSL https://raw.githubusercontent.com/dreal/dreal4/master/setup/ubuntu/20.04/install_prereqs.sh -o /tmp/install_prereqs.sh"))
                subprocess.run(shlex.split("sudo bash /tmp/install_prereqs.sh"))
                subprocess.run(shlex.split("rm -f /tmp/install_prereqs.sh"))
            else:
                print()
            flowstar_file = ''
            for filename in glob.glob('flowstar*'):
                flowstar_file = filename
            command = "tar -xf " + flowstar_file
            subprocess.run(shlex.split(command))
            command = "make -C " + os.getcwd() + "/" + flowstar_file
            subprocess.run(shlex.split(command))
        # Mac OS X system install scripts
        elif platform == "darwin":
            print()
            if sys.version_info[0] < 3:
                print()
                brew_installed = subprocess.check_call(shlex.split("which -s brew"))
                if brew_installed == 0:
                    print("Installing dependencies...")
                    subprocess.check_call(shlex.split("brew install git curl gmp mpfr gsl glpk bison flex gnuplot"))
                    subprocess.check_call(shlex.split("/usr/bin/curl -fsSL https://raw.githubusercontent.com/dreal/dreal4/master/setup/mac/install_prereqs.sh -o /tmp/install_prereqs.sh"))
                    subprocess.check_call(shlex.split("chmod u+x /tmp/install_prereqs.sh"))
                    subprocess.check_call(shlex.split("sh /tmp/install_prereqs.sh"))
                    flowstar_file = ''
                    for filename in glob.glob('flowstar*'):
                        flowstar_file = filename
                    command = "tar -xf " + flowstar_file
                    subprocess.check_call(shlex.split(command))
                    command = "make -C " + os.getcwd() + "/" + flowstar_file
                    subprocess.check_call(shlex.split(command))
                else:
                    print("Homebrew not installed!")
                    sys.exit(2)
            else:
                print()
                brew_installed = subprocess.run(shlex.split("which -s brew"))
                if brew_installed == 0:
                    print("Installing dependencies...")
                    subprocess.run(shlex.split("brew install git curl gmp mpfr gsl glpk bison flex gnuplot"))
                    subprocess.run(shlex.split("/usr/bin/curl -fsSL https://raw.githubusercontent.com/dreal/dreal4/master/setup/mac/install_prereqs.sh -o /tmp/install_prereqs.sh"))
                    subprocess.run(shlex.split("chmod u+x /tmp/install_prereqs.sh"))
                    subprocess.run(shlex.split("sh /tmp/install_prereqs.sh"))
                    flowstar_file = ''
                    for filename in glob.glob('flowstar*'):
                        flowstar_file = filename
                    command = "tar -xf " + flowstar_file
                    subprocess.run(shlex.split(command))
                    command = "make -C " + os.getcwd() + "/" + flowstar_file
                    subprocess.run(shlex.split(command))
                else:
                    print("Homebrew not installed!")
                    sys.exit(2)
        # Default system case
        else:
            print()
        install.run(self)
        
requires = [
    'dreal',
    'sympy',
    'numpy',
    'setuptools',
    'future',
    'scipy',
    'numba',
    'cvxpy',
    'z3-solver',
    'control',
    'matplotlib',
    'tqdm',
    'joblib',
    'python-igraph',
    'slycot',
    'NetworkX',
    'dreal',
    'pycddlib'
]

setup(
    name='control_system_abstractions',
    cmdclass={
    	'install' : CustomInstallCommand
    },
    install_requires=requires,
)
