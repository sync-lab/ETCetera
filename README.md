# SENTIENT
Toolchain for scheduling of event-triggered controllers

To install the project download or clone the project to a local diectory.

Create a virtual environment to install the dependancies and third-party tools:

    export VENV=~/projects/control_systems_abstractions/env
    python3 -m venv $VENV

Download and copy the latest flow* tar file into the project root directory from https://flowstar.org/dowloads/.

Install the application using the following command:
    
    $VENV/bin/pip3 install -vvv .

Run the application using following command:

    python3 etc2pta -h  # For details of input args
    python3 etc2pta -i {path_to_file} -s linear
    python3 etc2pta -i {path_to_file} -s non-linear
