***********************
Command Line Interface
***********************

:mod:`etc2pta.py`
=================
The command line interface `etc2pta.py` can be used for generating traffic models for
both linear PETC and nonlinear ETC systems. The command line interface usage is as follows:

.. code-block:: console

   $ python etc2pta.py system_type input_file [options]

positional arguments:

  system_type:         System type.
            Possible values: ``linear``, ``nonlinear``.

  input_file :         Path to input file.
            Contents depend on the value of system_type (see below).

optional arguments:

  -h, --help      Shows help message.

  -o OUTPUT_FILE, --output_file OUTPUT_FILE  Output file, default is out.json if nothing specified. Tries to extract file type from the name if possible.

  --output_type OUTPUT_TYPE
                  Output file type. Will overwrite type generated from ``-o``.
  -v, --verbose         Increase verbosity


.. _linearpetc-cli-label:

Linear PETC
-----------
The required input fields are:

* ``Dynamics: A, B`` which are the matrices describing the LTI system :math:`\dot{x} = Ax + Bu`.
* ``Controller: K``, which is the control matrix rendering the LTI system stable.
* ``Triggering Sampling Time: h``, which is the PETC sampling time in seconds.
* ``Triggering Heartbeat: hmax``, which is the maximum time until the PETC triggers in seconds.
* ``Triggering Condition: Q``, which is the matrix in the quadratic triggering condition :math:`x^TQx > 0`.

Elements in matrices are seperated with a space, rows are seperated with a ``;``. E.g.: :math:`[0 \enspace 1;\enspace -2 \enspace 3]`

Optional arguments are:

* ``Lyapunov Function: P``, the positive definite matrix :math:`P` in the Lyapunov function :math:`V(x) = x^T Px`, which shows that the LTI system is stable. If this is not given, it will be generated automatically based on the provided LTI system and controller.
* Solver Options: ``Solver Options``:

  * ``kmaxextra: n``. Compute the transitions for inter-event times larger than ``kmax``. Default: ``kmax + max_delay_steps``
  * ``cost_computation=b``. Set to ``True`` if transition costs should be computed. Default: ``False``
  * ``mu_threshold=f``. If f > 0, tries to remove regions that are geometrically similar. The bigger the number the more regions that are likely removed. Default: ``0.0``
  * ``min_eig_threshold=f``. The minimum value the smallest eigenvalue of the matrix describing a state-space region must have for this region to be considered. If bigger than 0.0, it typically results in a maximum uniform inter-event time smaller than what would naturally occur, but for a very small subset of the state-space. Default: ``0.0``
  * ``reduced_actions=b``. If True, action set is equal to the set of possible inter-eventtimes. Default: ``False``
  * ``early_trigger_only=b``. If True, transitions from a region (i, ...) are computed for inter-event times up to i + max_delay_steps. Default: ``False``
  * ``max_delay_steps=f``. The maximum extra discrete time for which transitions are computed. That is, for a region :math:`(i, ...)`, transitions are computed from kmin until `min(kmaxetra, i + max_delay_steps)`. Default: ``0``
  * ``depth=n``. Maximum depth of the bisimulation algorithm. For :math:`n > 1`, it is recommended to set ``solver=z3``. Default: ``1``.
  * ``etc_only=f``. Set to True if early trigger transitions should not be computed. This is useful for verification, but not for scheduling, since the resulting abstraction becomes autonomous. Default: ``False``
  * ``end_level=f``: If stop_around_origin is true, specifies the sublevel set :math:`{x: x'Px <= end_level}` from which the bisimulation algorithm stops. This assumes that the initial states are normalized to satisfy :math:`x(0)^T Px(0) = 1`. Default: ``0.01``
  * ``solver=sdr|z3``. The solver to be used to compute the transition relations. ``sdr`` will use (approximate) semi-definite relaxations, ``z3`` will solve exactly. Default: ``sdr``
  * ``stop_around_origin=b``. Set to True if the bisimulation algorithm should stop one all statesare guaranteed to have entered a ball around the origin. Default: ``False``
  * ``stop_if_omega_bisimilar=b``. Set to True if the bisimulation algorithm should stop if an omega-bisimulation is found. Default:``False``

  .. * ``symbolic=b``. (Current not yet working). Whether to perform the calculations symbolically.
  ..      Default: ``False``
  .. * ``consider_noise=b``. Not yet Implemented! Whether pure measurement noise is to be considered.
  ..      Default: ``False``

For example, the file ``examples/linear_ifac2020.txt`` looks as follows::

    Dynamics : [1.38 -0.208 6.715 -5.676; -0.581 -4.29 0 0.675; 1.067 4.273 -6.654 5.893; 0.048 4.273 1.343 -2.104],  [0 0;5.679 0;1.136 3.146;1.136 0]
    Controller : [0.518 -1.973 -0.448 -2.1356;-3.812 -0.0231 -2.7961 1.671]
    Triggering Sampling Time: .01
    Triggering Heartbeat: 0.20
    Triggering Condition: [5.12877179 -0.33664526  3.75378367 -2.69239893 -2.96042877 0.18702515 -2.08543537  1.49577718;
                            -0.33664526  1.20197192 .34159831  1.50316955  0.18702515 -0.77887329 -0.18977684 -0.8350942;
                            3.75378367  0.34159831  2.68689327 -1.33784524 -2.08543537  -0.18977684 -1.60382959  .74324736;
                            -2.69239893  1.50316955 -1.33784524  2.44744618  1.49577718  -0.8350942   0.74324736 -1.47080343;
                            -2.96042877  0.18702515 -2.08543537  1.49577718  0.       0.          0.          0.        ;
                            0.18702515 -0.77887329 -0.18977684 -0.8350942   0.        0.          0.          0.        ;
                            -2.08543537 -0.18977684 -1.60382959  0.74324736  0.       0.          0.          0.        ;
                            1.49577718 -0.8350942   0.74324736 -1.47080343  0.      0.          0.          0.        ]



.. _nonlinearetc-cli-label:

Nonlinear ETC
-------------
The dynamics, controller, etc. are expressed and inputted using ``sympy`` expressions. To make sure no errors occur,
some conventions have been defined. There are a few sets of possible variables:

* State variables: Starting with an ``x``.
* Input variables: Starting with an ``u``.
* Error variables: Starting with an ``e``.
* Homogeneous variable: A single variable ``w1``, used for making dynamics homogeneous.
* Disturbance variables: Starting with a ``d``.

By convention, every variables should be numbered sequentially starting from 1: ``x1, x2, ...``, ``u1, u2, ...``, etc.
The input fields are:

* ``Dynamics: x_1dot, x_2dot, ...``, describing the dynamics of the nonlinear system :math:`\dot{x} = f(x, u)`.

  * The number of used state variables should match the number of given expressions.
  * If one or more input variables are present, the field ``Controller`` should be filled in as well.
  * If variables ``e1, ...`` are present in the dynamics, it is assumed that the system is in ETC form, and the number of them should equal the number of state variables. If they are not specified, the dynamics will automatically be converted into ETC form, but only if controller expression are specified. If both error and input variables are present in the dynamics, an error is thrown.
  * If variables ``w1, ...`` are present in the dynamics, it is assumed that the system is homogenized, and the correctness is checked first.
  * If variables ``d1, ...`` are present in the dynamics, ``Hyperbox Disturbances`` should be specified as well. This will also automatically overwrite the partitioning method to ``grid``.

* ``Controller: u1expr, u2expr``, describing the controller. The number of given expression should match the number of input variables in the dynamics.
* ``Triggering Condition: expr``. The triggering condition. Should contain both ``x`` and ``e`` variables.
* ``Deg. of Homogeneity: n``. The degree of homogeneity of the dynamics. If not specified, will automatically be calculated.
* ``Hyperbox States: [a1 b1], [a2 b2], ...``. The state space region that is considered during generation of the traffic model, represented by an interval. The number of given intervals should match the number of state variables.
* ``Hyperbox Disturbances: [a1 b1], [a2 b2], ...``. The intervals the disturbance variables are limited to. Number of intervals should match the number of disturbance variables.
* ``Grid Points Per Dimension: [n1 n2 ...]``. Number of boxes each dimension is divided into. Number of given grid points should match the number of state variables. Default: ``5`` for each dimension.
* Solver Options: ``Solver Options: opt1=arg1, opt2=args,...``. These specify the options for abstraction:

  * ``partitioning: grid|manifold``. Choose whether the state space is partitioned by isochronous manifolds, or by gridding. If ``manifold`` is specified, also the option ``manifold_times`` should be specified. Default: ``grid``
  * ``manifold_times: [t1, t2, ...]``. The times used for partitioning using isochronous manifolds. Should be specifiedwhen ``partition_method=manifold`` and have at least two elements. When ``partition_method=grid``, this value is used as a reference manifold for timing lower bounds. Default: ``[1e*4]``
  * ``nr_cones_small_angles: [n1, n2, ...]``. The number of divisions for the small angles. When the state space is represented using generalized spherical coordinates, there are ``n-2`` angle coordinates which run from ``0`` to ``pi``. These are the ``small angles``. Default: ``[5, ...]``
  * ``nr_cones_big_angle: n``. The number of divisions for the big angle. When the state space is represented using generalized spherical coordinates, there is only one angle that runs from ``-pi`` to ``pi``. This is the ``big angle``. Default: ``None``
  * ``heartbeat: f``. The maximum trigger time. Default: ``0.1``
  * ``order_approx: n``. The order to which the isochronous manifold are approximated. Default: ``2``
  * ``timeout_deltas: f``. The maximum time to calculate each delta. Default: ``1000``
  * ``precision_deltas: f``. The precision at which the deltas are calculated. Default: ``1e-4``
  * ``timeout_timing_bounds: f``. The maximum time to calculate upper and lower bounds to the regions. Default: ``200``
  * ``precision_timing_bounds: f``. Precision to which the upper and lower bounds to the regions are calculated. Default: ``1e-3``
  * ``timeout_transitions: f``. The maximum calculation time to calculate each transition. Default: ``200``
  * ``precision_transitions: f``. The precision to which the flowpipe is calculated. Default: ``1e-3``

For example, the file ``examples/nl_nonhomogeneous.txt`` looks as follows::

  Hyperbox States: [-2 2], [-2 2]
  Grid Points Per Dimension: [3 3]
  Dynamics : x1*w1**2, x1**2*x2 + x2**3 + u1
  Controller: -x2*w1**2 - x1**2*(x2+e2) - x2**3
  Triggering Condition : e1**2 + e2**2 - 0.01**2*w1**2

  Solver Options : manifolds_times=[0.002 0.0028 0.0038 0.005 0.0065 0.0075], partition_method=manifold, heartbeat=0.021, order_approx=4


