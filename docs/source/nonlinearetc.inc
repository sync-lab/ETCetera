Traffic Models of Nonlinear ETC Systems
========================================

.. py:class:: TrafficModelNonlinearETC(dynamics, homogeneity, trigger, state, init_cond_symbols=None, dist_param=None, dist_param_domain=None, homogenization_flag=False,  precision_deltas=1e-4, timeout_deltas=1000, partition_method='grid', manifolds_times=None, nr_cones_small_angles=None, nr_cones_big_angle=5, state_space_limits=None, grid_points_per_dim=None, heartbeat=0.1, precision_timing_bounds=1e-3, precision_transitions=1e-3, timeout_timing_bounds=200, timeout_transitions=200, order_approx=2)

    **Traffic model of a nonlinear ETC system.**

    Considering general nonlinear event-triggered systems, this class computes traffic models capturing the triggering behaviour of the closed loop system. This class implements several techniques, depending on the type of system specified:

    - If the system is homogeneous (:math:`f(\lambda x) = \lambda^{\alpha+1}f(x)`), the state space is partitioned into regions delimited by isochronous manifolds and cones (:attr:`partition_method` = ``manifold``) or by hypercubes (:attr:`partition_method` = ``grid``)
    - If the system is nonhomogeneous (but has been homogenized manually by introducing a new variable :math:`w`), the cones are defined by partitioning the state space on the :math:`w=1`-plane (:attr:`partition_method` = ``manifold``) or by hypercubes (:attr:`partition_method` = ``grid``)
    - If the system contains disturbances, the only option is to partition the state space by hypercubes (:attr:`partition_method` = ``grid``).

    :param `~sympy:sympy.matrices.common.MatrixCommon` dynamics: Sympy expression(s) of :math:`f(x)`.
    :param float homogeneity: The degree of homogeneity of the dynamics
    :param `~sympy:sympy.core.basic.Basic` trigger: The sympy expression of the trigger condition :math:`\Gamma(x, e)`.
    :param tuple[`~sympy:sympy.core.symbol.Symbol`] state: Tuple of the state variables present in :attr:`dynamics`.
    :param tuple[`~sympy:sympy.core.symbol.Symbol`] init_cond_symbols: Symbols denoting initial conditions for state variables. Default: ``(a1, a2, \dots)``.
    :param tuple[`~sympy:sympy.core.symbol.Symbol`] dist_param: Tuple of the disturbance variable. **Add these if present in :attr:`dynamics`**. Default: ``None``.
    :param int order_approx: Order of approximation of the triggering function. Default: ``2``.
    :param list[list[int]] dist_param_domain: Interval of each of the disturbance variables (e.g. [[-1,1],[-2,2]]). Length should be equal to the number of disturbance variables. Default: ``None``.
    :param bool homogenization_flag: **Set to ``True`` if the system has been homogenized**. Default: ``False``.
    :param ``grid`` or ``manifold`` partition_method: Whether to partition the state space by a grid or by isochronous manifolds. Default: ``grid``.
    :param list[float] manifolds_times: List of triggering times defining the isochronous manifolds. **This should be set if the partition method ``manifold`` is used**. Default: ``[0.001]``.
    :param list[int] nr_cones_small_angles: For homogeneous systems (to create the cones), the small angles of spherical coordinates (the ones with :math:`\theta_i \in [-\pi/2, \pi/2)`) are divided into this many pieces. The length should be 2 less than the number of state variables.   Default: ``[4, 4, ...]``
    :param int nr_cones_big_angle: For homogeneous systems (to create the cones), the big angle of spherical coordinates (the ones with :math:`\phi \in [-\pi, \pi)`) are divided into this many pieces. Default: ``5``.
    :param list[list[float]] state_space_limits: List of intervals the state variables will lie in (e.g. [[-1,1],[-2,4], [-1.5,3]]). Default: ``[[-1, 1], ...]``.
    :param list[int] grid_points_per_dim:
                                           - When :attr:`partition_method` = ``grid``, specifies in how many pieces each dimension should be cut.
                                           - When :attr:`partition_method` = ``manifold`` **and** :attr:`homogenization_flag` = ``True``, specifies how many partitions of the state space on the w=1-plane into cubes, which then defines the cones.
    :param float heartbeat: Maximum trigger time. Default: ``0.1``.

    :param float precision_deltas: dReal precision for manifold approximation algorithm (calculating :math:`\delta_i`). Default: ``1e-4``.
    :param int timeout_deltas: Timeout for manifold approximation algorithm (calculating :math:`\delta_i`). Default: ``1000``.
    :param float precision_timing_bounds: dReal precision or flowstar remainder for reachability analysis for timing bounds of the regions. Default: ``1e-3``.
    :param float precision_transitions: dReal precision or flowstar remainder for reachability analysis for transitions. Default: ``1e-3``.
    :param float timeout_timing_bounds:  Timeout for reachability analysis for timing bounds. Default: ``200``.
    :param float timeout_transitions:  Timeout for reachability analysis for transitions. Default: ``200``.


    .. py:property:: regions:
        List containing ``Region_Homogeneous`` or ``Region_Grid`` objects. Will generate this regions automatically on first access.

    .. py:method:: create_abstraction():

        Creates the regions and transitions of the traffic model.

        :return: ``self.regions``, ``self.transitions``

.. py:class: Region_Homogeneous:

    Class representing one of the regions of a