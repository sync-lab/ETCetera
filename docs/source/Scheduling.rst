******************
Scheduling
******************
.. py:module:: sentient.Scheduling


Scheduling with TGAs and UPPAAL Stratego
----------------------------------------------

.. py:module:: sentient.Scheduling.NTGA

.. py:currentmodule:: sentient.Scheduling.NTGA

.. py:class:: controlloop(abstraction[, name=None, index=None, initial_location=None, max_delay_steps=0, max_early=None, max_early_triggers=0, delta=None, sync='up', nack='nack', ack='ack', timeout='timeout', down='down', clock='c'])


    Timed Game Automaton represention of a traffic model of a (P)ETC control loop as is discussed in :ref:`theory_ntga_uppaal`.

    :param ~sentient.Abstractions.Abstraction abstraction: Traffic model of the control loop.
    :param str name: Name that this will be referred to (mainly in UPPAAL). Default: `controlloop`.
    :param str or int: Index of the controlloop. This is to make sure no name clash occurs (in UPPAAL). Default: 6 random characters in ``[A-z0-9]``.
    :param initial_location: Known initial location of the traffic model.
    :param int max_delay_steps: Maximum amount of extra samples available for retransmission. Default: ``0``.
    :param int max_early: Maximum amount of samples the control loop can trigger earlier. Default: ``2`` for PETC, ``0.01`` for ETC.
    :param int max_early_triggers: Maximum amount of consecutive early triggers that may occur. Default: ``0``.
    :param float delta: Occupancy time of the communication channel when control loops triggers. Default: ``h`` for PETC, ``0.01`` for ETC.


    .. note::

        Recommended to leave the following parameters at their default value. If you do, make sure they match those defined in :py:class:`~sentient.Scheduling.NTGA.network` and supply them to :py:class:`~sentient.Scheduling.NTGA.nta`.

    :param str sync: Synchronization action for requesting access to the communication channel. Default: ``'up'``.
    :param str ack: Synchronization action to indicate that the communication channel is available. Default: ``'ack'``.
    :param str nack: Synchronization action to indicate that the communication channel is not available. Default: ``'nack'``.
    :param str timout: Synchronization action to indicate that the controlloop timed out.  Default: ``'timeout'``.
    :param str down: Synchronization action to indicate that the control loop is down transmitting. Default: ``'down'``.
    :param str clock: Name for the clock variable. Default: ``'c'``.



.. py:class:: network([, name='Network', index=None, sync='up', ack='ack', nack='nack', timeout='timeout', down='down'])

    Timed Game Automaton representing the network/communication channel shared between multiple :py:class:`~sentient.Scheduling.NTGA.controlloop`. Structure is as is described in :ref:`theory_ntga_uppaal`.

    :param str name: Name of the network. Default: ``Network``.
    :param str or int index: Number of the network. This is to make sure no name clash occurs (in UPPAAL). Default: 6 random characters in ``[A-z0-9]``.

    .. note::

        Recommended to leave the following parameters at their default value. If you do, make sure they match those defined in :py:class:`~sentient.Scheduling.NTGA.controlloop` and supply them to :py:class:`~sentient.Scheduling.NTGA.nta`.

    :param str sync: Synchronization action for requesting access to the communication channel. Default: ``'up'``.
    :param str ack: Synchronization action to indicate that the communication channel is available. Default: ``'ack'``.
    :param str nack: Synchronization action to indicate that the communication channel is not available. Default: ``'nack'``.
    :param str timout: Synchronization action to indicate that the controlloop timed out.  Default: ``'timeout'``.
    :param str down: Synchronization action to indicate that the control loop is down transmitting. Default: ``'down'``.



.. py:class:: nta (net, closed_loops[, synchronization=None])

    Composition of a network TGA and several control loop TGAs as is discussed in :ref:`theory_ntga_uppaal`. This is the final object used to construct the scheduler for the mulitple control loops.

    :param network net: Network to be used in the NTA.
    :param list[controlloop] closed_loops: List of controlloops to be used in the NTA.
    :param list[str] synchronization: List of synchronization actions that are present in the network/control loops. Default:``["up", "down", "timeout", "ack", "nack"]``

        .. note::

            If any of those parameters are changed in :class:`controlloop` or :class:`network`, the whole list with the changed should be supplied.

    .. py:method:: generate_strategy([, parse_strategy=True, delete_files=True])

        Will generate a scheduler for the control loops.

        If ``parse_strategy=False``, the resulting strategy contains lines like::

            State: ( controlloopXnz3NMnednZr.Trans_loc controlloopghGzCHW8GAiy.Ear7 NetworkDu8ujp.InUse_ack ) EarNum=1 controlloopXnz3NMnednZr.to_region=0 controlloopXnz3NMnednZr.from_region=6 controlloopXnz3NMnednZr.count=0 controlloopghGzCHW8GAiy.to_region=0 controlloopghGzCHW8GAiy.from_region=0 controlloopghGzCHW8GAiy.count=0
            When you are in (5<=controlloopghGzCHW8GAiy.c && controlloopXnz3NMnednZr.c<=9 && controlloopghGzCHW8GAiy.c<=7 && controlloopghGzCHW8GAiy.c-controlloopXnz3NMnednZr.c<=-1) || (controlloopXnz3NMnednZr.c==6 && controlloopXnz3NMnednZr.c-controlloopghGzCHW8GAiy.c==-1 && controlloopghGzCHW8GAiy.c==7) || (6<=controlloopXnz3NMnednZr.c && controlloopXnz3NMnednZr.c<=7 && controlloopXnz3NMnednZr.c==controlloopghGzCHW8GAiy.c), take transition NetworkDu8ujp.InUse_ack->NetworkDu8ujp.InUse { true, ack!, 1 }
            controlloopXnz3NMnednZr.Trans_loc->controlloopXnz3NMnednZr.Clk_wait { true, ack?, c := 0, count := 0, from_region := to_region }

        If ``parse_strategy=True``, the resulting strategy will be a dictionary mapping state pairs to clock conditions and which loop to trigger::

            strat = {(r1, r2, ..): [(E1, d1, E2, d2, to_trigger), ...]  , ...}

        E1, d1, E2, d2 are the matrices which represent the clock conditions: :math:`E_1\textbf{c} = d_1 \land E_2\textbf{c} \leq d_2`, where :math:`\textbf{c}` is the vector of clocks. If these conditions are satisfied, ``to_trigger`` contains which control loops (the index in the list) to trigger. If none of the possible conditions of the current state are satisfied, no control loops should trigger.

        :param bool parse_strategy: Whether to parse the strategy resulting from UPPAAL into a more compact and useable form. Default: ``True``.
        :param bool delete_files: Whether to delete all the files (except for the resulting strategy files) after generation is done. Default: ``True``.








Scheduling by solving safety games
-----------------------------------------

.. py:module:: sentient.Scheduling.fpiter

.. py:currentmodule:: sentient.Scheduling.fpiter

.. py:function:: controlloop(abstraction[, use_bdd=True, maxLate: int = None, maxLateStates: int = None, ratio: int = 1])

    Creates a different representation of the PETC traffic model, such that the actions become 'per-sample-based'. In short, to each region :math:`k = (k_1, k_2, \dots)`, new states are associated: :math:`\{T_{k}, W_{k, 1}, \dots, W_{k, k_1-1}\}`. Transitions between these states consist of:

    - :math:`(W_{k, j}, w, W_{k,j+1})`,
    - :math:`(T_i, t, T_j)`, if :math:`\exists (i, 1, j)` in original traffic model.
    - :math:`(W_{k,i}, t, T_j)` if :math:`\exists (k, i, j)` in original traffic model.

    Lastly, the output map is given by: :math:`H(T_1) = T_1`,  :math:`H(T_i) = T` (:math:`i>1`), :math:`H(W_{k,j}) = W_{k_1-j}`.
    In this way, triggering after :math:`j` samples becomes ``w`` (``wait``) for :math:`j-1` samples, and then ``t`` (``trigger``).

    .. note::

        Because of the construction of this representation, only PETC abstractions can be used.

    :param TrafficModelLinearPETC abstraction: The traffic model of a PETC system.
    :param bool use_bdd: Whether to use BDDs to represent the traffic model. Default: ``True``.
    :param int maxLate: The maximum amount of ``wait`` actions (:math:`\Delta`) after the triggering deadline can be performed in a row. Default: ``None``.

        .. warning::

            Allowing late triggers to occur might (likely!) cause instability of the underlying PETC system.

    :param int maxLateStates: The amount of states present in the traffic model (:math:`L`) after the triggering deadline (so states :math:`\{W_{k, k_1}, \dots, W_{k,k_1+L-1}\},` and transitions :math:`(W_{k, k_1}, lw, W_{k, k_1+1}), \dots` are added). Default: As much as is present in :attr:`abstraction`.
    :param int ratio: Amount of ``trigger`` / ``regular-wait`` actions have to be performed to compensate for a ``late-wait`` action (:math:`r`). Default: ``1``.

    :returns: New traffic model representation
    :rtype: :py:class:`enum.controlloop` or :py:class:`bdd.controlloop`

.. py:currentmodule:: sentient.Scheduling.fpiter.enum

.. py:class:: controlloop(abstraction[, maxLate: int = None, maxLateStates: int = None, ratio: int = 1])

    Implementation of the traffic model as described in :ref:`theory_sg` with simple lists/sets/dictionaries.

    .. note::

        This is purely for documentation. Recommended is to construct the control loop using :func:`~sentient.Scheduling.fpiter.controlloop`.

    :param TrafficModelLinearPETC abstraction: The traffic model of a PETC system.
    :param int maxLate: The maximum amount of ``wait`` actions (:math:`\Delta`) after the triggering deadline can be performed in a row. Default: ``None``.

        .. warning::

            Allowing late triggers to occur might (likely!) cause instability of the underlying PETC system.

    :param int maxLateStates: The amount of states present in the traffic model (:math:`L`) after the triggering deadline (so states :math:`\{W_{k, k_1}, \dots, W_{k,k_1+L-1}\},` and transitions :math:`(W_{k, k_1}, lw, W_{k, k_1+1}), \dots` are added). Default: As much as is present in :attr:`abstraction`.
    :param int ratio: Amount of ``trigger`` / ``regular-wait`` actions have to be performed to compensate for a ``late-wait`` action (:math:`r`). Default: ``1``.

    .. py:property:: states

        Dictionary containing state, state index pairs. If the system is partitioned, then will return a dictionary containing pairs of blocks, set of states in the block.
        Additionally, if late triggers are allowed, will automatically compose the states with the auxiliary system.

        :type: dict

    .. py:property:: transitions

        Dictionary of the form :math:`\{x: \{w: Postw(x), t: Post_t(x)\}, \dots \}` which represent the transitions possible in the system.
        If the system is partitioned, then the transitions will be in terms of the blocks.
        Additionally, if late triggers are allowed, will automatically compose the transitions with the auxiliary system.

    .. py:property:: output_map

        Dictionary mapping the states to the outputs (:math:`H : X \to Y`). When the system is partitioned, will map the blocks to outputs. Additionally, if late triggers are allowed, will automatically compose the output map with the auxiliary system.

    .. py:property:: outputs

        Dictionary containing output, output index pairs. If late triggers are allowed, the outputs are composed with the auxiliary system.

    .. py:method:: restore()

        Restores the system to the original from the partitioned system.

    .. py:method:: create_initial_partition()

        Groups together states with the same output into blocks and computes new transitions between these blocks.

        :return: Whether partitioning is successful.
        :rtype: bool

    .. py:method:: refine()

        Splits apart blocks of the current partition by means of the refinement operator:  :math:`\textit{for all  } C \in \Pi \, \textit{ do }: \: \Pi := Refine(\Pi, C) = \bigcup_{B\in\Pi}Refine(B,C)`, where :math:`Refine(B,C) = \{B \cap pre(C), B \setminus pre(C)\}\setminus\{\emptyset\}`.

        :return: Whether refinement is successful.
        :rtype: bool

.. py:currentmodule:: sentient.Scheduling.fpiter.bdd

.. py:class:: controlloop(abstraction[, name: str = None, maxLate: int = None, maxLateStates: int = None, ratio: int = 1])

    Implementation of the traffic model as described in :ref:`theory_sg` with the use of Binary Decision Diagrams (BDDs) and the use of the tool `dd <https://pypi.org/project/dd/>`_.
    This represents the exact same traffic models as in :py:class:`.enum.controlloop`, except everything represented with BDDs (This object is actually constructed by first creating a :class:`.enum.controlloop` and creating the BDDs from that).

    BDDs are constructed by assigning to each state in the system a unique (binary) number. So :math:`enc_x: X \to \mathbb{B}^n` converts states to binary values, and :math:`enc_u` similarly for inputs. A single transition


    Example:
        Suppose we want to get the set with all the states that could lead to themselves. Using set notation this would be:

        .. math::

            A = \{x \in X \mid \exists (x,u,y) \in \longrightarrow: \: x = y\}.

        Which would be represented with boolean functions/BDDs as:

        .. math::

            f_A(x) = \exists u.f_{tr}(x,u,x)

        And in this in turn in code would be:

        .. code-block:: python

            # cl is a `bdd.controlloop` object
            rename = {y:x for (x,y) in zip(cl.xvars, cl.yvars)}
            f_A = cl.bdd.exists(cl.bdd.uvars, cl.bdd.let(rename, cl.tr))


    :param str name: The name used to differentiate between boolean variables. Default: ``cl`` + 3 random characters in ``[A-z0-9]``.
    :param TrafficModelLinearPETC abstraction: The traffic model of a PETC system.
    :param int maxLate: The maximum amount of ``wait`` actions (:math:`\Delta`) after the triggering deadline can be performed in a row. Default: ``None``.

        .. warning::

            Allowing late triggers to occur might (likely!) cause instability of the underlying PETC system.

    :param int maxLateStates: The amount of states present in the traffic model (:math:`L`) after the triggering deadline (so states :math:`\{W_{k, k_1}, \dots, W_{k,k_1+L-1}\},` and transitions :math:`(W_{k, k_1}, lw, W_{k, k_1+1}), \dots` are added). Default: As much as is present in :attr:`abstraction`.
    :param int ratio: Amount of ``trigger`` / ``regular-wait`` actions have to be performed to compensate for a ``late-wait`` action (:math:`r`). Default: ``1``.

    .. py:attribute:: xvars



    .. py:property:: tr

        BDD of the boolean function representing the transitions in the system:

        .. math::

            f_{tr}(x,u,y) := \begin{cases}
                    1, \quad \text{if} \ \exists(a,v,b) \in \longrightarrow: \: x = enc(a) \land u = enc(v) \land y = enc(b),  \\
                    0, \quad \text{otherwise}
                    \end{cases}

        If the system is partitioned, this will be in terms of the blocks variables (:math:`b_i, c_i`) instead.
        Additionally, if late triggers are allowed, this will be the composition with the transitions from the auxiliary system: :math:`f_{tr} \land f_{tr}^{aux}`.

        :Example:

            Suppose we have the transitions: :math:`\{T_2 : \{w: \{W_{2,1}\}, t:\{T_2\}\}, W_{2,1}: \{w: \emptyset, t:\{T_2\}\}\}`, then the boolean function which :attr:`tr` represents is:

            .. math::

                f_{tr}(x_0, u, y_0) = \bar{x}_0u\bar{y}_0 \lor \bar{x}_0uy_0 \lor x_0u\bar{y}_0.

    .. py:property:: Q

        BDD of the boolean function representing the blocks in the system:

        .. math::

            f_{Q}(b, x) := \begin{cases}
                    1, \quad \text{if} \ \exists \, \text{state } p, \text{block } q: \: p \in q \land x = enc(p) \land b = enc(q),  \\
                    0, \quad \text{otherwise}
                    \end{cases}

        In the case that the system is not partitioned, this function simply becomes :math:`f_Q(x,x) = [x \iff x]`.

    .. py:property:: XT

        BDD of the boolean function representing whether a state/block is a transmit state/block:

        .. math::

            f_{XT}(x) := \begin{cases}
                    1, \quad \text{if} \ \exists a: \: x = enc(a) \land H(a) \in \{T, T_1\}, \\
                    0, \quad \text{otherwise}
                    \end{cases}

    .. py:method:: restore

        Restores the system to the original from the partitioned system.

    .. py:method:: create_initial_partition()

        Groups together states with the same output into blocks and computes new transitions between these blocks.

        :return: Whether partitioning is successful.
        :rtype: bool

    .. py:method:: refine()

        Splits apart blocks of the current partition by means of the refinement operator using BDDs:

        :return: Whether refinement is successful.
        :rtype: bool

