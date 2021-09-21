# !/usr/bin/python
# coding=utf-8

"""
Created on Sat May 16 14:53:58 2020

@author: gmaddodi
"""

from contextlib import contextmanager
from setuptools import setup
from setuptools.command.install import install
import subprocess
from sys import platform
import shlex
#import lsb_release
import os
import sys
import glob
import config

# List of dependencies installed via `pip install -e .`
# by virtue of the Setuptools `install_requires` value below.


class CustomInstallCommand(install):
    """
        Custom install class for running post install scripts after pip install all the requirements. The run function
        checks if smt file path, dreal path, dreach path, and flow* paths are set in config.py, if not sets the path to
        third-party directory.
    """
    # user_options = install.user_options + [
    #     ('--cudd', None, None),  # a 'flag' option
    # ]

    def run(self):
        pwd = os.getcwd()
        config_str = ''
        if not config.smt_path:
            config_str = config_str + 'path = ' + pwd + '/resources\n'


        if not config.dreal_path and os.path.isfile(os.getcwd()+'/third-party/dreal4/bazel-bin/dreal/dreal'):
            config_str = config_str + 'dreal_path = ' + pwd + '/third-party/dreal4/bazel-bin/dreal/dreal\n'
        elif config.dreal_path:
            pass
        elif not config.dreal_path and not os.path.isfile(os.getcwd()+'/third-party/dreal4/bazel-bin/dreal/dreal'):
            print('Warning! dreal executable not specified, please specify dreal path in config.py')


        if platform == "linux" or platform == "linux2":
            if not config.dreach_path and os.path.isfile(os.getcwd()+'/third-party/dreach/ubuntu/dreach'):
                config_str = config_str + 'dreach_path = ' + pwd + \
                             '/third-party/dreach/linux/dReach\n'
            elif config.dreach_path:
                pass
            elif not config.dreach_path and not os.path.isfile(os.getcwd()+'/third-party/dreach/ubuntu/dreach'):
                print('Warning! dreach executable not specified, please specify dreach path in config.py')
        elif platform == "darwin":
            if not config.dreach_path and os.path.isfile(os.getcwd() + '/third-party/dreach/mac/dreach'):
                config_str = config_str + 'dreach_path = ' + pwd + \
                             '/third-party/dreach/mac/dReach\n'
            elif config.dreach_path:
                pass
            elif not config.dreach_path and not os.path.isfile(os.getcwd()+'/third-party/dreach/mac/dreach'):
                print('Warning! dreach executable not specified, please specify dreach path in config.py')


        if not config.flowstar_path and os.path.isfile(os.getcwd()+'/third-party/flowstar-2.0.0/flowstar'):
            config_str = config_str + 'flowstar_path = ' + pwd + '/third-party/flowstar-2.0.0/flowstar\n'
        elif config.flowstar_path:
            pass
        elif not config.flowstar_path and not os.path.isfile(os.getcwd()+'/third-party/flowstar-2.0.0/flowstar'):
            print('Warning! Flow* executable not specified, please specify Flow* path in config.py')

        # Write to config.py
        with open("config.py", "w") as config_write:
            config_write.write(config_str)

        install.run(self)
        subprocess.call(['pip', 'install', '-r', 'dd.txt'])#'--install-option="--fetch"', 'install-option="--cudd"'])

        # Check if the OS is Linux or Mac
        #if platform == "linux" or platform == "linux2":
        #    release = ''
        #    if os.path.isfile('/etc/lsb-release'):      # Linux distribution should be Ubuntu 16.04, 18,04, or 20.04
        #        lines = open('/etc/lsb-release').read().split('\n')
        #        for line in lines:
        #            if line.split('=')[0] == 'DISTRIB_RELEASE':
        #                release = line.split('=')[1]
        #            else:
        #                pass
        #    else:
        #        pass

            # Install dependencies
            #subprocess.check_call(["sudo", "apt", "update"])
            #subprocess.run(shlex.split("sudo apt-get install git curl libgmp3-dev libmpfr-dev libgsl0-dev glpk-doc "
            #                          "glpk-utils libglpk-dev bison flex gnuplot"))

        #    if config.dreal_path:
        #        pass
        #    else:
        #        if release == "16.04":
                    #subprocess.run(shlex.split("curl -fsSL https://raw.githubusercontent.com/dreal/dreal4/master/setup/ubuntu/16.04/install_prereqs.sh -o /tmp/install_deps.sh"))
                    #subprocess.run(shlex.split("sudo bash /tmp/install_deps.sh"))
                    #subprocess.run(shlex.split("rm -f /tmp/install_deps.sh"))
        #            with cd(os.getcwd() + '/third-party/dreal4/setup/ubuntu/' + release):
                        #command = "sudo bash " + dreal_path + "/setup/ubuntu/16.04/install_deps.sh"
        #                subprocess.run(shlex.split("sudo bash install_prereqs.sh"))
        #            with cd(os.getcwd() + '/third-party/dreal4/'):
        #                subprocess.run(shlex.split("bazel build //..."))
        #        elif release == "18.04":
        #            with cd(os.getcwd() + '/third-party/dreal4/setup/ubuntu/' + release):
        #                subprocess.run(shlex.split("sudo bash install_prereqs.sh"))
        #            with cd(os.getcwd() + '/third-party/dreal4/'):
        #                subprocess.run(shlex.split("bazel build //..."))
        #        elif release == "20.04":
        #            with cd(os.getcwd() + '/third-party/dreal4/setup/ubuntu/' + release):
        #                subprocess.run(shlex.split("sudo bash install_prereqs.sh"))
        #            with cd(os.getcwd() + '/third-party/dreal4/'):
        #                subprocess.run(shlex.split("bazel build //..."))
        #        else:
        #            print('Cannot install dReal on this Linux distribution.')
        #            #sys.exit()

        #    if config.flowstar_path:
        #        pass
            #else:
                # flowstar_file = ''
                # for filename in glob.glob('flowstar*'):
                #    flowstar_file = filename
                # command = "tar -xf " + flowstar_file
                # subprocess.run(shlex.split(command))
                #with cd(os.getcwd() + '/third-party/flowstar-2.0.0/'):
                #    subprocess.run(shlex.split('make'))

        # Mac OS X system install scripts
        #elif platform == "darwin":
        #    print()
        #    if sys.version_info[0] < 3:
        #        brew_installed = subprocess.check_call(shlex.split("which -s brew"))
        #        if brew_installed == 0:
        #            subprocess.check_call(shlex.split("brew install git curl gmp mpfr gsl glpk bison flex gnuplot"))

        #            if config.dreal_path:
        #                pass
        #            else:
        #                with cd(os.getcwd() + '/third-party/dreal4/setup/mac/'):
        #                    subprocess.check_call(shlex.split("chmod u+x install_prereqs.sh"))
        #                    subprocess.check_call(shlex.split("sh install_deps.sh"))
        #                with cd(os.getcwd() + '/third-party/dreal4/'):
        #                    subprocess.run(shlex.split("bazel build //..."))

        #            if config.flowstar_path:
        #                pass
        #            else:
                        #flowstar_file = ''
                        #for filename in glob.glob('flowstar*'):
                        #    flowstar_file = filename
                        #command = "tar -xf " + flowstar_file
        #                #subprocess.check_call(shlex.split(command))
        #                #command = "make -C " + os.getcwd() + "/" + flowstar_file
        #                #subprocess.check_call(shlex.split(command))
        #                with cd(os.getcwd() + '/third-party/flowstar-2.0.0/'):
        #                    subprocess.check_call(shlex.split('make'))
        #        else:
        #            print("Homebrew not installed. Install Homebrew and retry.")
        #    else:
        #        brew_installed = subprocess.run(shlex.split("which -s brew"))
        #        if brew_installed == 0:
        #            subprocess.run(shlex.split("brew install git curl gmp mpfr gsl glpk bison flex gnuplot"))

        #            if config.dreal_path:
        #                pass
        #            else:
        #                with cd(os.getcwd() + '/third-party/dreal4/setup/mac/'):
        #                    subprocess.run(shlex.split("chmod u+x install_deps.sh"))
        #                    subprocess.run(shlex.split("sh install_deps.sh"))
        #                with cd(os.getcwd() + '/third-party/dreal4/'):
        #                    subprocess.run(shlex.split("bazel build //..."))

        #            if config.flowstar_path:
        #                pass
        #            else:
        #                with cd(os.getcwd() + '/third-party/flowstar-2.0.0/'):
        #                    subprocess.run(shlex.split('make'))
        #       else:
        #            print("Homebrew not installed. Install Homebrew and retry.")
        # Default system case
        #else:
        #    print('Cannot install dReal and Flow* on this OS.')


@contextmanager
def cd(newdir):
    prev_dir = os.getcwd()
    os.chdir(os.path.expanduser(newdir))
    try:
        yield
    finally:
        os.chdir(prev_dir)


requires = [
    'cvxpy',
    'sympy',
    'numpy',
    'scipy',
    'numba',
    'z3-solver',
    'control',
    'matplotlib',
    'tqdm',
    'joblib',
    'python-igraph',
    'NetworkX',
    'pycddlib',
    'pyparsing',
    'cycler',
    'python-dateutil',
    'slycot',
    'pygraphviz',
    'shortuuid',
    'dd>=0.1.1'
]

setup(
    name='sentient',
    cmdclass={
        'install': CustomInstallCommand
    },
    scripts=['pip install dd>=0.1.1 --install-option="--fetch" --install-option="--cudd"'],
    install_requires=requires,
)
