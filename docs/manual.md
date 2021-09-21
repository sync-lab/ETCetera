# SENTIENT 
***
SENTIENT is a tool written in Python for the creation of 
traffic models of Event Triggered Control systems. Additionally, it is able to synthesize 
schedulers (based on the traffic models) for these systems. 

## Table of contents
***
- [Installation](#installation)
- [Quickstart](#quickstart)
- [Detailed Usage](#detailed-usage)
  - [Command Line Interface](#command-line-interface)
- [Theory](#theory)
  - [Linear PETC Traffic Models](#linear-petc-traffic-models)
  - [Nonlinear ETC Traffic Models](#nonlinear-etc-traffic-models)

<!-- toc -->

## Installation
***
The tool requires Python 3.8. If you have it installed check whether `python3` 
points to the correct version by using
```shell
$ python3 --version
```
Most likely this is not the case, so in the rest of this section it is advised to replace 
`python3` by `python3.8` to make sure that installation occurs on Python 3.8.

If Python 3.8 is not yet installed on your machine install it as follows:
```shell
$ sudo add-apt-repository ppa:deadsnakes/ppa
$ sudo apt-get update
$ sudo apt-get install python3.8
```

You can install both using conda and pip. The advantage of using conda over pip 
is that the required packages are binaries already so there is no tinkering and issues with 
installing dependencies written in other languages than Python.
However, currently it can not be installed as a conda package, so it cannot be used as such yet. 
Installing using pip does create a pip package.



### Conda
Installation using conda is quite straightforward. You can create a conda environment 
with the requirements using the given `environment.yml` file:
```shell
$ conda env create --file environment.yml
```
This will automatically create a conda environment named `sentient` and install the dependencies. 
To activate the environment simply run
```shell
$ conda activate sentient
```

### Pip and venv
Before continuing run the installation script which contains the required dependencies for the pip installation. 
The installation scripts are found in the folder scripts and depend on your OS. 
The installation script should be run as root.
To install the tool using pip, it is recommended to do in a virtual environment. 
To create a virtual environment do:
```shell
$ python3 -m venv <path_to_venv>
```
where `<path_to_venv>` is the path to where the virtual environment should be created. 
Activate the virtual environment:
```shell
$ source activate <path_to_venv>/bin/activate
```
Then in the root of the project source folder run
```shell
$ pip install .
```
To install the tool. Add option `-vvv` to view more of the installation process.

## Quickstart 
***
For a quickstart, the command line tool `etc2pta.py` can be used. 
It can generate traffic models for linear PETC systems following [], and for nonlinear 
ETC systems following []. The command line interface is used as follows:

```shell
$ python etc2pta.py <systemtype> <inputfile> [options]
```

Systemtype is either `linear' or 'nonlinear', from which the corresponding abstraction 
algorithm is chosen. Input file is a file containing the details for the ETC system. 
The contents of this file depend on the abstraction algorithm. 
The following shows an example input file for linear PETC systems: ![example_file](linear_ifac2020.jpg) 


For example, running the example `linear_ifac.txt` and outputting is done as:
```shell
$ python etc2pta.py linear examples/linear_ifac.txt --output_file=ex.json
```

# Detailed Usage
***
## Command Line Interface
The command line interface `etc2pta.py` can be used for generating traffic models for 
both linear PETC and nonlinear ETC systems. The command line interface usage is as follows:
```shell
$ python etc2pta.py <systemtype> <inputfile> [options]
```

The argument `systemtype` is either `linear` or `nonlinear` and `inputfile` is the 
path to the input file. If `linear` is specified it will generate a linear PETC traffic model ([], [], []). 
See [Linear PETC Traffic Models](#linear-petc-traffic-models) for the details.  
If `nonlinear` is specified, it will generate a nonlinear ETC traffic model ([], []).
Depending on the systemtype, the contents of the inputfile should be different. 
See [Nonlinear ETC Traffic Models](#nonlinear-etc-traffic-models) for the details.  

### Linear PETC Input Fields
The required input fields are: 
- Dynamics: `Dynamics: A, B` which are the matrices describing the LTI 
  system `dx/dt = Ax + Bu`.
  
- Controller: `Controller: K`, which is the control matrix rendering the LTI system stable(!).
- Triggering Sampling Time: `Triggering Sampling Time: h`, which is the PETC sampling time in seconds.
- Triggering Heartbeat: `Triggering Heartbeat: hmax`, which is the maximum time until the PETC triggers in seconds.
- Triggering Condition: `Triggering Condition: Q`, which is the matrix in the quadratic triggering condition `x^TQx > 0`.

Elements in matrices are seperated with a `,`, rows are seperated with a `;`. E.g.: `[0 1;-2 3]` 

Optional arguments are:
- Lyapunov function: `Lyapunov Function: P`, the positive definite matrix P in the 
  Lyapunov function `V(x) = x^T Px`, which shows that the LTI system is stable. If this is not given,
  it will be generated automatically based on the provided LTI system and controller.
  
- Solver Options: `Solver Options: `
  - `kmaxextra: n`. Compute the transitions for inter-event times larger than `kmax`. Default: `None` 
    (this sets the value to `kmax + max_delay_steps`)
    
  - `cost_computation=b`. Set to `True` if transition costs should be computed. Default: `False`
  - `mu_threshold=f`. If `f >= 0`, tries to remove regions that are geometrically similar. The bigger the number the more regions that are likely removed. Default: `0.0`
  - `min_eig_threshold=f`. The minimum value the smallest eigenvalue of the matrix describing
        a state-space region must have for this region to be considered.
        If bigger than 0.0, it typically results in a maximum uniform
        inter-event time smaller than what would naturally occur, but for a
        very small subset of the state-space. Default: `0.0`
  
  - `reduced_actions=b`. If True, action set is equal to the set of possible inter-event
        times. Default: `False`
  - `early_trigger_only=b`. If True, transitions from a region (i, ...) are computed for inter-
        event times up to i + max_delay_steps. Default: `False`
  - `max_delay_steps=f`. The maximum extra discrete time for which transitions are computed.
        That is, for a region (i, ...), transitions are computed from kmin
        until min(kmaxetra, i + max_delay_steps). Default: `0`
  - `depth=n`. Maximum depth of the bisimulation algorithm. Default: `1`. For `n > 1`, it is recommended to set `solver=z3`.
  - `etc_only=f`. Set to True if early trigger transitions should not be computed.
        This is useful for verification, but not for scheduling, since the
        resulting abstraction becomes autonomous. Default: `False`
  - `end_level=f`: If stop_around_origin is true, specifies the sublevel set
        {x: x'Px <= end_level} from which the bisimulation algorithm stops.
        This assumes that the initial states are normalized to satisfy
        x(0)'Px(0) = 1. Default: `0.01`
  - `solver=sdr|z3`. The solver to be used to compute the transition relations. Default: `sdr`
  - `stop_around_origin=b`. Set to True if the bisimulation algorithm should stop one all states
        are guaranteed to have entered a ball around the origin. Default: `False`
  - `stop_if_omega_bisimilar=b`. Set to True if the bisimulation algorithm should stop if an omega-
        bisimulation is found. Default:`False`
    
  - `symbolic=b`. (Current not yet working). Whether to perform the calculations symbolically. Default: `False`
  - `consider_noise=b`. Not yet Implemented! Whether pure measurement noise is to be considered. Default: `False`

### Nonlinear ETC Input Fields
The dynamics, controller, etc. are expressed and inputted using `sympy` expressions. To make sure no errors occur,
some conventions have been defined. There are a few sets of possible variables:
- State variables: Starting with an `x`.
- Input variables: Starting with an `u`.
- Error variables: Starting with an `e`.
- Homogeneous variable: A single variable `w1`, used for making dynamics homogeneous.
- Disturbance variables: Starting with a `d`. 

By convention, every variables should be numbered sequentially starting from 1: `x1, x2, ...`, `u1, u2, ...`, etc.
The input fields are:
- Dynamics: `Dynamics: x_1dot, x_2dot, ...`, describing the dynamics of the nonlinear system. 
  - The number of used state variables should match the number of given expressions.
  - If one or more input variables are present, the field `Controller` should be filled in as well.
  - If variables `e1, ...` are present in the dynamics, it is assumed that the system is in ETC form,
    and the number of them should equal the number of state variables. If they are not specified,
    the dynamics will automatically be converted into ETC form, but only if controller expression are specified.
    If both error and input variables are present in the dynamics, an error is thrown.
  - If variables `w1, ...` are present in the dynamics, it is assumed that the system is homogenized,
    and the correctness is checked first.
  - If variables `d1, ...` are present in the dynamics, `Hyperbox Disturbances` should be specified as well.
    This will also automatically overwrite the partitioning method to `grid`.
- Controller: `Controller: u1expr, u2expr`, describing the controller. The number of given expression should match
  the number of input variables in the dynamics.    
- Triggering Condition: `Triggering Condition: expr`. The triggering condition. Should contain both `x` and `e` variables.
  
- Homogeneity degree: `Deg. of Homogeneity: n`. The degree of homogeneity of the dynamics. If not specified, will 
  automatically be calculated.

- State space limits: `Hyperbox States: [a1 b1], [a2 b2], ...`. The state space region that is considered during 
  generation of the traffic model, represented by an interval. The number of given intervals should match the number of 
  state variables.    

- Disturbance limits: `Hyperbox Disturbances: [a1 b1], [a2 b2], ...`. The intervals the disturbance variables are limited to.
  Number of intervals should match the number of disturbance variables.
  
- Grid point per dimension: `Grid Points Per Dimension: [n1 n2 ...]`. Number of boxes each dimension is divided into. 
  Number of given grid points should match the number of state variables. Default: 5 for each dimension.
  
- Solver Options: `Solver Options: opt1=arg1, opt2=args,...`. These specify the options for abstraction:
  - `partitioning: grid|manifold`. Choose whether the state space is partitioned by isochronous manifolds, 
    or by gridding. If `manifold` is specified, also the option `manifold_times` should be specified. Default: `grid`
    
  - `manifold_times: [t1, t2, ...]`. The times used for partitioning using isochronous manifolds. Should be specified
    when `partition_method=manifold` and have at least two elements. When `partition_method=grid`, this value is used
    as a reference manifold for timing lower bounds, which then has default: `[1e-4]`
  - `nr_cones_small_angles: [n1, n2, ...]`. The number of divisions for the small angles. When the state space is 
  represented using generalized spherical coordinates, there are `n-2` angle coordinates which run from `0` to `pi`. 
    These are the `small angles`. Default: `[5, ...]`
  - `nr_cones_big_angle: n`. The number of divisions for the big angle. When the state space is 
  represented using generalized spherical coordinates, there is only one angle that runs from `-pi` to `pi`. 
    This is the `big angle`. Default: `None`
    
  - `heartbeat: f`. The maximum trigger time. Default: `0.1`
  - `order_approx: n`. The order to which the isochronous manifold are approximated. Default: `2`
  - `timeout_deltas: f`. The maximum time to calculate each delta. Default: `1000`
  - `precision_deltas: f`. The precision at which the deltas are calculated. Default: `1e-4`
  - `timeout_timing_bounds: f`. The maximum time to calculate upper and lower bounds to the regions. Default: `200`
  - `precision_timing_bounds: f`. Precision to which the upper and lower bounds to the regions are calculated. Default: `1e-3`
  - `timeout_transitions: f`. The maximum calculation time to calculate each transition. Default: `200`
  - `precision_transitions: f`. The precision to which the flowpipe is calculated. Default: `1e-3`




# Theory
***
## Linear PETC Traffic Models


## Nonlinear ETC Traffic Models