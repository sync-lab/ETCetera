******************
Scheduling
******************
.. py:module:: sentient.Scheduling


Scheduling with TGAs and UPPAAL Stratego
=========================================













Scheduling by solving safety games
======================================

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
    :param int float: Amount of ``trigger`` / ``regular-wait`` actions have to be performed to compensate for a ``late-wait`` action (:math:`r`). Default: ``1``.

    :returns: New traffic model representation
    :rtype: :py:class:`enum.controlloop` or :py:class:`bdd.controlloop`

.. py:currentmodule:: sentient.Scheduling.fpiter.enum

.. py:function:: controlloop(abstraction[, maxLate: int = None, maxLateStates: int = None, ratio: int = 1])

.. py:currentmodule:: sentient.Scheduling.fpiter.bdd

.. py:function:: controlloop(abstraction[, name: str = None, maxLate: int = None, maxLateStates: int = None, ratio: int = 1])