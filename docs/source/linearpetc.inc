Traffic Models of Linear PETC Systems
========================================



.. py:currentmodule:: sentient.Abstractions

.. py:class:: TrafficModelLinearPETC(trigger[, kmaxextra=None, cost_computation=False, consider_noise=False, mu_threshold=0.0, min_eig_threshold=0.0, symbolic=False, reduced_actions=False, early_trigger_only=False, max_delay_steps=0, depth=1, etc_only=False, end_level=0.01, solver='sdr', stop_around_origin=False, stop_if_omega_bisimilar=False])

    **Traffic model of a PETC system.**

    Considering a linear quadratic periodic event-triggering system, this
    class computes a finite-state transition system and a (bi)simulation
    relation that captures the traffic patterns of the closed-loop system,
    as well as the changes in such patterns if early triggers are applied.
    The traffic model is essentially a system that, given a sampled state
    :math:`x(t)`, can determine its next discrete inter-event time :math:`k` and the next
    sample :math:`x(t+kh)`, where h is the PETC sampling time.

    This class implements the algorithms of

    * [1]_ for fast and scalable similar models;
    * [2]_ for bisimilar models of either PETC or a modified PETC that
      samples periodically when states get close to the origin;
    * [3]_ for omega-bisimilar models of PETC.

    The resulting transition system is the tuple :math:`(X, U, E, Y, H, P)` where

    * :math:`X` is the set of states. A state is labeled by a tuple :math:`(k_1, k_2, \dots, k_m)`
      which indicates the expected sequence of discrete inter-event times
      from a related concrete state.
    * :math:`U` is the set of actions. These are integers representing the next
      discrete inter-sample time.
    * :math:`E` is the set of transitions, a subset of :math:`X \times U \times X`. A transition
      :math:`(x,u,x')` means that, from state :math:`x`, using input :math:`u` can lead the state
      to :math:`x'`.
    * :math:`Y` is the set of outputs, which are integers.
    * :math:`H : X \to Y` is the output map. :math:`H((k_1, k_2, \dots)) = k_1`.
    * :math:`P: E \to \mathbb{Q}` is the transition price map. Different prices can be used from the structure generated
      by this class.

    .. note::
        Note: Only state-feedback controllers are currently supported.

    :param LinearQuadraticPETC trigger: The ETC control system to be abstracted.
    :param int kmaxextra: Set a number if transitions are to be computed for inter-event times longer than :py:attr:`trigger.kmax`. This can be useful for triggering
        conditions that can be satisfied at a second firing. Default: :py:attr:`trigger.kmax` + :py:attr:`max_delay_steps`.
    :param bool early_trigger_only: If True, transitions from a region ``(i, ...)`` are computed for inter-event times up to ``i + max_delay_steps``. Default: ``False``.
    :param int max_delay_steps: The maximum extra discrete time for which transitions are computed. That is, for a region (i, ...), transitions are computed from ``kmin``
        until ``min(kmaxetra, i + max_delay_steps)```. Default: ``0``.

        .. note::

            if ``max_delay_steps > 0``, early_trigger_only is automatically enabled.

    :param solver: The solver to be used to compute the transition relations:

            * ``sdr``    (Conservative) semi-definite relaxation, which uses CVX.
            * ``z3``     Exact solutions through computational algebra solver Z3.

            Default: ``sdr``
    :param int depth: Maximum depth of the bisimulation algorithm. Default: ``1``, which uses the simulation algorithm from [1]_, with no refinement For greater depths, it is recommended to set solver='z3'.
    :param bool stop_if_omega_bisimilar: Set to True if the bisimulation algorithm should stop if an omega-
        bisimulation is found (thus, using [3]_). Default: ``False``.
    :param bool cost_computation: Set to True if the transition costs should be computed. A transition
        cost from :math:`\mathcal{R}_i` to :math:`\mathcal{R}_j` after sampling instant :math:`k` is the pair
        :math:`(1 + min cost, 1 + max cost)` determined by solving:

        .. math::

            \min / \max \quad &cost(x(k)) - cost(x(0)) \\
                \textrm{s.t.} \quad & x(0)^T Px(0) = 1, \\
                     &x(0) \in \mathcal{R}_i, \\
                     &x(k) = M(k)x(0) \in \mathcal{R}_j \\

        where :math:`cost(x) := x^T Px`, and :math:`P` is the Lyapunov matrix. Default: ``False``.
    :param bool stop_around_origin: Set to True if the bisimulation algorithm should stop once all states are guaranteed to have entered a ball around the origin. Default: ``False``.
    :param float end_level: If stop_around_origin is true, specifies the sublevel set
        :math:`\{x: x^T Px <= end\_level\}` from which the bisimulation algorithm stops.
        This assumes that the initial states are normalized to satisfy
        :math:`x(0)^T Px(0) = 1`, following [2]_. Default: ``0.01``.
    :param bool etc_only: Set to True if early trigger transitions should not be computed.
        This is useful for verification, but not for scheduling, since the
        resulting abstraction becomes autonomous. Default: ``False``.
    :param float mu_threshold: If bigger than ``0.0``, attempts to reduce the number of regions by
        discarding regions that are geometrically similar. The bigger the
        number, the more regions are likely to be discarded. Default: ``0.0``.
    :param float min_eig_threshold: The mimimum value the smallest eigenvalue of the matrix describing
        a state-space region must have for this region to be considered.
        If bigger than ``0.0``, it typically results in a maximum uniform
        inter-event time smaller than what would naturally occur, but for a
        very small subset of the state-space. Default: ``0.0``.
    :param bool reduced_actions: If ``True``, action set is equal to the set of possible inter-event
        times. Default: ``False``.
    :param bool consider_noise: (Future) Whether pure measurement noise is to be considered.


    .. py:attribute:: n

        The data-space dimension (typically the state-space dimension,
        but it can be different for dynamic / output-feedback controllers).

        :type: int

    .. py:attribute:: inputs

        The abstraction input space U.

        :type: set[int]

    .. py:attribute:: K

        The set of possible discrete inter-event times.

        :type: set[int]

    .. py:attribute:: M

        State-to-state transition matrices for each discrete inter-event time,
        such that x(t+kh) = M(k)x(t).

        :type: dict[:attr:`K`, :class:`~numpy:numpy.array`]

    .. py:attribute:: N

        (Future) Transition matrices from state to input and output, such that
        [y(t+kh), u(t+kh)].T = N(k) @ x(t).

        :type: dict[:attr:`K`, :class:`~numpy:numpy.array`]

    .. py:attribute:: Q

        Matrices Q[k] define quadratic forms such that, if
        x(t) @ Q[k] @ x(t) > epsilon, then x(t) will certainly trigger
        before t + kh. These are basic matrices to partition the state-
        space and build the (bi)simulation relations. See [1]_, [2]_.

        :type: dict[:attr:`K`, :class:`~numpy:numpy.array`]

    .. py:attribute:: kmin

        Minimum discrete inter-efent time, which is by default trigger.kmin.
        It can be overwritten to a higher value if the algorithm detects that
        no state can trigger at kmin.

        :type: int

    .. py:attribute:: kmax

        Maximum discrete inter-event time, which is by default trigger.kmax.
        It can be overwritten to a smaller value if the algorithm detects that
        all state would have triggered before kmax.

        :type: int

    .. py:attribute:: kmaxextra

        See Parameters above.

        :type: int


    .. py:attribute:: complete_cost : {}
    .. py:attribute:: probability_region = {}
    .. py:attribute:: probability_transition = {}
    .. py:attribute:: predecessors = {}  # precedence (subset) relation for quadratic forms
    .. py:attribute:: cost = {}  # Lyapunov reduction after a sequence of sampling times

    .. py:property:: regions:

        Set containing the sequence of trigger times corresponding to each region.
        I.e. :math:`\{(k_1, k_2, ..), ...\}`. The length of the sequence is equal to :py:attr:`depth`.

    .. py:property:: transitions:

        Dictionary containing sets of possible next regions for all combinations of region, trigger time pairs.
        I.e. :math:`\{((k_1, \dots), u_1): \{(a_1, \dots), \dots \}, ((l_1, \dots), u_2): \{(b_1, \dots), \dots \} \dots \}`.
        Will be recomputed on access if necessary.

    .. py:method:: create_abstraction():

        Creates the regions and transitions of the traffic model.

        :return: :py:attr:`~.TrafficModelLinearPETC.regions`, :obj:`transitions`

    .. py:method:: generate_regions():

        Generate the regions of the traffic model.

        :return: None

    .. py:method:: refine([, i=1]):

        Increases :attr:`depth` by ``i`` and recomputes the regions.

        :param int i: How many times the regions should be refined. Default: ``1``.
        :return: None

    .. py:method:: return_regions_descriptors():

        Returns a dictionary containing symbolic expressions describing the regions.

        :return: Region descriptors.
        :rtype: dict









.. [1] Gleizer, Gabriel de A., and Manuel Mazo Jr. "Scalable Traffic
   Models for Scheduling of Linear Periodic Event-Triggered Controllers."
   arXiv preprint arXiv:2003.07642 (2020). Presented at IFAC World
   Congress 2020.

.. [2] G. A. Gleizer and M. Mazo Jr., "Towards Traffic Bisimulation of
   Linear Periodic Event-Triggered Controllers," in IEEE Control Systems
   Letters, vol. 5, no. 1, pp. 25-30, Jan. 2021,
   doi: 10.1109/LCSYS.2020.2999177.

.. [3] G. A. Gleizer and M. Mazo Jr. WIP.