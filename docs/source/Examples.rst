************
Examples
************

The scripts of all the following examples can be found in ``examples/``.

Traffic Model Examples
=======================
Here we discuss examples of how to construct traffic models for linear PETC and nonlinear ETC systems.

Alternative to these approaches, the functions :func:`~sentient.util.construct_linearPETC_traffic_from_file` and :func:`~sentient.util.construct_nonlinearETC_traffic_from_file` can be used to construct the traffic models from a file.

Command Line Interface
-------------------------
You can construct traffic models using the CLI ``etc2pta.py`` by running one of the examples:

.. code-block:: console

    $ python etc2pta.py linear examples/linear_ifac2020.txt --output_file=ex.json
    $ python etc2pta.py nonlinear examples/nl_homogeneous.txt --output_file=hom.json
    $ python etc2pta.py nonlinear examples/nl_nonhomogeneous.txt --output_file=nonhom.json
    $ python etc2pta.py nonlinear examples/nl_disturbances.txt --output_file=dist.json

Linear PETC
------------
In this example, a traffic model of a linear Periodic Event Triggered Control system will be generated.
The system to be abstracted is as follows:

.. math::

    \dot{x}(t) &= \begin{bmatrix}
		1.38 & -0.208 & 6.715 & -5.676 \\
		-0.581 & -4.29 & 0 & 0.675 \\
		1.067 & 4.273 & -6.654 & 5.893 \\
		0.048 & 4.273 & 1.343 & -2.104
	\end{bmatrix} x(t) + \begin{bmatrix}
		0 & 0 \\
		5.679 & 0 \\
		1.136 & 3.146 \\
		1.136 & 0
    \end{bmatrix} u(t), \\
    u(t) &= \begin{bmatrix}
		0.52 & -1.97 & -0.45 & -2.14\\
		-3.81 & -0.023 & -2.80 &  1.67
	    \end{bmatrix}  x(t)

With a sampling time of :math:`h=0.01` and :math:`\bar{k} = 20`. We will use the triggering condition :math:`\Gamma = |e(t)|^2 - \sigma |x(t)|^2 = \zeta^T Q \zeta`, :math:`\sigma = 0.05`.
In the tool, a PETC system is then defined as follows:

.. code-block:: python

    # Define LTI system matrices
    A = np.array([[1.38, -0.208, 6.715, -5.676], [-0.581, -4.29, 0, 0.675], [1.067, 4.273, -6.654, 5.893], [0.048, 4.273, 1.343, -2.104]])
    B = np.array([[0, 0],[5.679, 0], [1.136, 3.146],[1.136, 0]])
    K = np.array([[0.518, -1.973, -0.448, -2.1356], [-3.812, -0.0231, -2.7961, 1.671]])

    # PETC parameters
    h = 0.01
    kmax = 20
    sigma = 0.01

    # Triggering condition
    Qtrigger = np.block([[(1-sigma)*np.eye(4), -np.eye(4)], [-np.eye(4), np.eye(4)]])

    # Construct object representing the PETC system
    import sentient.Systems as systems

    plant = systems.LinearPlant(A, B)
    controller = systems.LinearController(K, h)
    trigger = systems.LinearQuadraticPETC(plant, controller, kmax=kmax, Qbar=Qtrigger)

Then a traffic model is generated as follows. More arguments can be specified as described in :class:`sentient.Abstractions.TrafficModelLinearPETC`.

.. code-block:: python

    import sentient.Abstractions as abstr

    traffic = abstr.TrafficModelLinearPETC(trigger)
    regions, transitions = traffic.create_abstraction()
    # Returns: {(2,), (5,), (4,), (1,), (3,)}, {((2,), 1): {(1,), (2,), (3,)}, ((2,), 2): {(2,), (5,), (4,), (1,), (3,)}, ...}

Alternatively, the regions/transitions can also be accessed separately:

.. code-block:: python

    regions = traffic.regions
    # Returns: {(2,), (5,), (4,), (1,), (3,)}
    transitions = traffic.transitions
    # Returns: {((2,), 1): {(1,), (2,), (3,)}, ((2,), 2): {(2,), (5,), (4,), (1,), (3,)}, ...}

This will instead cause the regions/transitions to be computed on first access (after caches are reset by for instance :func:`~sentient.Abstractions.TrafficModelLinearPETC.refine`).
Use

.. code-block:: python

    region_descriptors = traffic.return_region_descriptors()
    # Returns: {(2,): (x1*(0.0325014371942073*x1 - 0.758236561497541*x2 + 2.28413716988318*x3 - 1.57991935089754*x4) + x2*(-0.758236561497541*x1 + 2.38486724990143*x2 + 0.198562111037632*x3 + 2.68392804714407*x4) + x3*(2.28413716988318*x1 + 0.198562111037632*x2 + 3.0913841666531*x3 - 2.03625414224163*x4) + x4*(-1.57991935089754*x1 + 2.68392804714407*x2 - 2.03625414224163*x3 + 2.14744812716726*x4) <= 0) & (x1*(-34.1084257021831*x1 + 22.2685952023407*x2 - 68.7949413314049*x3 + 47.7543148417454*x4) + x2*(22.2685952023407*x1 - 102.519813292189*x2 - 8.36602721281553*x3 - 82.3989681651887*x4) + x3*(-68.7949413314049*x1 - 8.36602721281553*x2 - 121.315797670082*x3 + 56.7655786995279*x4) + x4*(47.7543148417454*x1 - 82.3989681651887*x2 + 56.7655786995279*x3 - 97.0796601856019*x4) < 0), ... }

to obtain the expressions describing the actual regions.

Finally, the traffic model can be saved for future use:

.. code-block:: python

    # To pickle the object:
    traffic.export('traffic_petc', 'pickle')

    # To save to a .json file:
    traffic.export('traffic_petc', 'json')

The files will be saved to the ``saves`` folder.

Nonlinear ETC
--------------
In this example, a traffic model for a nonhomogeneous nonlinear system will be generated. The dynamics are given by:

.. math::

    \dot{x} = f(x) = \begin{bmatrix} x_1 \\ x_1^2x_2 + x_2^3 + u \end{bmatrix}, \quad u = -x_2 - x_1^2x_2 - x_2^3

With triggering condition:

.. math::

    \Gamma = |e|^2 - |x|^2 * (0.0127*0.3)^2

The system first has to be converted into a ETC form. This is done by:

.. code-block:: python

    import sympy
    import sentient.util as utils

    # Define
    state_vector = x1, x2, e1, e2 = sympy.symbols('x1 x2 e1 e2')

    # Define controller (in etc form)
    u1 = -(x2+e2) - (x1+e1)**2*(x2+e2) - (x2+e2)**3

    # Define dynamics
    x1dot = x1
    x2dot = x1**2*x2 + x2**3 + u1
    dynamics = [x1dot, x2dot, -x1dot, -x2dot]

These dynamics are not yet homogeneous, so they are homogenized (see ...):

.. code-block:: python

    # Make the system homogeneous (with degree 2)
    hom_degree = 2
    dynamics, state_vector = utils.make_homogeneous_etc(dynamics, state_vector, hom_degree)
    dynamics = sympy.Matrix(dynamics)

Then we define the triggering condition and the portion of the state space we want to consider.

.. code-block:: python

    # Triggering condition & other etc.
    trigger = ex**2 + ey**2 - (x1**2+y1**2)*(0.0127*0.3)**2

    # State space limits
    state_space_limits = [[-2.5, 2.5], [-2.5, 2.5]]

And lastly, we define the traffic model (since we homogenized the dynamics, ``homogenization_flag`` should be set to ``True``):

.. code-block:: python

    import sentient.Abstractions as abstr

    traffic = abstr.TrafficModelNonlinearETC(dynamics, hom_degree, trigger, state_vector, homogenization_flag=True, state_space_limits=state_space_limits)
    regions, transitions = traffic.create_abstraction()
    # Result: {'1': 0.003949281693284397, '2': 0.003924684110791467, ...}, {('1', (0.00358211491454367, 0.003949281693284397)): [1, 2, 6, 7], ... }

Now, the state space has been partitioned by gridding (default). To partition the state space by means of manifold, set ``partition_method=manifolds``.
Alternatively, the regions/transitions can also be accessed separately:

.. code-block:: python

    regions = traffic.regions
    # Returns: {'1': 0.003949281693284397, '2': 0.003924684110791467, ...}
    transitions = traffic.transitions
    # Returns: {('1', (0.00358211491454367, 0.003949281693284397)): [1, 2, 6, 7], ... }

This will instead cause the regions/transitions to be computed on first access.
Use

.. code-block:: python

    region_descriptors = traffic.return_region_descriptors()
    # Returns: {'1': (-1.0*x1 <= 2.5) & (1.0*x1 <= -1.5) & (-1.0*x2 <= 2.5) & (1.0*x2 <= -1.5), ...}

to obtain the expressions describing the actual regions.

Finally, the traffic model can be saved for future use:

.. code-block:: python

    # To pickle the object:
    traffic.export('traffic_etc', 'pickle')

    # To save to a .json file:
    traffic.export('traffic_etc', 'json')

The files will be saved to the ``saves`` folder.

Scheduling Examples
=====================

In the two following examples, two identical linear PETC systems are used. These have been computed and saved before hand, and are loaded as follows:

.. code-block:: python

    import sentient.Abstractions as abstr
    traffic_petc = abstr.TrafficModelLinearPETC.from_bytestream_file('traffic_petc.pickle')

To determine which of the scheduling algorithms should be used see ...

Scheduling using NTGAs and UPPAAL Stratego
------------------------------------------------

Here a scheduler is generated by representing the traffic models by TGA and adding a network. Then using `UPPAAL Stratego <https://people.cs.aau.dk/~marius/stratego/>`_, a strategy is generated and automatically parsed.
First both traffic models are converted:

.. code-block:: python

    import sentient.Scheduling.NTGA as sched
    cl1 = sched.controlloop(traffic_petc)
    cl2 = sched.controlloop(traffic_petc)

And a network is defined:

.. code-block:: python

    net = sched.Network()
    nta = sched.NTA(net, [cl1, cl2])

Then a scheduler is generated by:

.. code-block:: python

    nta.generate_strategy(parse_strategy=True)
    # Result: {"('7', '15')": [[[[1, 0]], [[0.07]], [[0, -1], [0, 1], [0, -1], [0, 1]], [[-0.09], [0.0015], [0.018500000000000003], [0.15]], 0], [[[1, 0], [1, -1], [0, 1]], [[0.07], [0], [0.07]], [], [], 0]], ...

This will save the parsed strategy to a file in ``strategy``. The contents of the file are as is discussed in ...



Scheduling by solving safety games
------------------------------------

Similar to before, first both traffic models are converted:

.. code-block:: python

    import sentient.Scheduling.fpiter as sched
    # For the example do not use BDDs to represent the models
    cl1 = sched.controlloop(traffic_petc, use_bdd=False)
    cl2 = sched.controlloop(traffic_petc, use_bdd=False)

These are then combined into a system, and a scheduler is generated:

.. code-block:: python

    S = sched.system([cl1, cl2])
    Ux = S.generate_safety_scheduler() # Scheduler
    # Results: ({('T12', 'W12,1'): {('w', 't'), ('w', 'w'), ('t', 'w')}, ('T12', 'W18,7'): {('w', 't'), ('w', 'w'), ...}, None)

The method :func:`generate_safety_scheduler` will automatically choose the (likely) most efficient algorithm.
To use BDDs, we simply set ``use_bdd=True``, and the rest is the same:

.. code-block:: python

    # Now do use BDDs
    cl1 = sched.controlloop(traffic_petc, use_bdd=True)
    cl2 = sched.controlloop(traffic_petc, use_bdd=True)
    S = sched.system([cl1, cl2])
    Ux = S.generate_safety_scheduler()  # Scheduler
    # Result: BDD representing the boolean function of Ux

To allow late triggers, at least specify ``maxLate``, and optionally ``maxLateStates`` and/or ``ratio``:

.. code-block:: python

    cl1 = sched.controlloop(traffic, use_bdd=True)
    cl2 = sched.controlloop(traffic, use_bdd=True, maxLate=2, ratio=2)
    S = sched.system([cl1, cl2])
    Ux = S.generate_safety_scheduler()  # Scheduler
    # Result: BDD representing the boolean function of Ux
