# SENTIENT
Toolchain for scheduling of event-triggered controllers.

To install the project download or clone the project to a local directory.

In project root directory there is scripts folder. There are two subfolders 'mac' and 'ubuntu' 
containing scripts to install dependencies. For Linux, there are three subfolders again '16.04',
'18.04', and '20.04'. Depending on your Ubuntu version run the appropriate install_deps.sh. Scripts 
should be run as sudo user in Linux and normal user on Mac.

Download and copy the third-party tools, dReeal, dReach, and Flow* in the 'third-party' folder in the root project
directory if you want to install them. You can download them from below location:

    dReal - https://github.com/dreal/dreal4
    dReach - https://github.com/dreal/dreal3/releases/tag/v3.16.06.02
    Flow* - https://flowstar.org/dowloads/
    
For example '/{location to project}/control_system_abstractions/third-party/dreal4/'
should contain the extracted contents of the dReal4 project. Similarly for Flow*, copy the source files directly under
the '/third-party/flowstar-2.0.0' directory. Follow the instructions to build the tools. For dReach copy the executable for 
either Linux or Mac directly under the '/third-party/dreach/ubuntu' or '/third-party/dreach/mac'.

Alternatively, if you already have any or all of dReal, dReach, and Flow* setup on you system, update the location
in config.py. For e.g. 'dreal_path = /home/user/dreal4/bazel-bin/dreal/dreal'.

Create a virtual environment to install the Python dependencies:

    export VENV=~/projects/control_systems_abstractions/env
    python3 -m venv $VENV

Build the tool using the following command:
    
    $VENV/bin/pip3 install -vvv .

Run the application using following command:

    python3 etc2pta -h  # For details of input args
    python3 etc2pta -i {path_to_file} -s linear
    python3 etc2pta -i {path_to_file} -s non-linear
    
Alternatively, a GUI based input can be used by running command:

    python3 etc2pta_GUI 
