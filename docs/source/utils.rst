.. include etc2pta.rst

Utils
======

.. py:module:: sentient.util

.. py:function:: construct_linearPETC_traffic_from_file(file_name)

   Constructs a :class:`TrafficModelLinearPETC` object based on the contents of a file. Performs multiple checks to make sure these contents fulfil the requirements as specified in :ref:`linearpetc-cli-label`.

   :param str file_name: path to file from which to construct the traffic model.
   :return: The traffic model
   :rtype: TrafficModelLinearPETC

.. py:function:: construct_nonlinearETC_traffic_from_file(file_name)

   Constructs a :class:`TrafficModelNonlinearETC` object based on the contents of a file. Performs multiple checks to make sure these contents fulfil the requirements as specified in :ref:`nonlinearetc-cli-label`.

   :param str file_name: path to file from which to construct the traffic model.
   :return: The traffic model
   :rtype: TrafficModelNonlinearETC

.. py:function:: test_homogeneity(exprs, vars)

    Tests whether the autonomous system :math:`\dot{x} = f(x)` is homogeneous. I.e. if for all :math:`x \in \mathbb{R}^n` and any :math:`\lambda > 0: \: f(\lambda x) = \lambda^{\alpha+1}f(x)`. :math:`f` then is homogeneous of degree :math:`\alpha \in \mathbb{R}`.

    :param exprs: Sympy expression(s) of :math:`f(x)`.
    :type exprs: list[:class:`~sympy:sympy.core.basic.Basic`] or :class:`~sympy:sympy.matrices.common.MatrixCommon`
    :param vars: List of sympy variables used in :attr:`exprs`.
    :type vars: list[:class:`~sympy:sympy.core.symbol.Symbol`]
    :return: If homogeneous, :math:`\alpha`; None otherwise.
    :rtype: None or float.

.. py:function:: make_homogeneous(exprs, vars: List[sympy.Symbol], des_hom_degree: int)

    Makes a given autonomous nonlinear system :math:`\dot{x} = f(x)` homogenous with given degree :math:`\alpha`. The resulting system is given by

    .. math::

        \begin{bmatrix} \dot{x} \\ \dot{w}\end{bmatrix} = \begin{bmatrix} w^{\alpha+1} f(w^{-1}(x, \hat{x}) \\ 0\end{bmatrix}.

    :param exprs: Sympy expression(s) of :math:`f(x)`.
    :type exprs: List[:class:`~sympy:sympy.core.basic.Basic`] or :class:`~sympy:sympy.matrices.common.MatrixCommon`
    :param vars: List of sympy variables used in :attr:`exprs`.
    :type vars: List[:class:`~sympy:sympy.core.symbol.Symbol`]
    :param des_hom_degree: Desired degree of homogeneity :math:`\alpha`.
    :type des_hom_degee: float
    :return: New expressions, new symbol vector
    :rtype: tuple[list[:class:`~sympy:sympy.core.basic.Basic`], list[:class:`~sympy:sympy.core.symbol.Symbol`]] or tuple[:class:`~sympy:sympy.matrices.common.MatrixCommon`, list[:class:`~sympy:sympy.core.symbol.Symbol`]]

.. py:function:: make_homogeneous_etc(exprs, vars: List[sympy.Symbol], des_hom_degree: int)

    Makes a given autonomous ETC system :math:`\begin{bmatrix} \dot{x} \\ \dot{\hat{x}} \end{bmatrix} = \begin{bmatrix} f(x, \hat{x}) \\ -f(x, \hat{x}) \end{bmatrix}` homogenous with given degree :math:`\alpha`. The resulting system is given by

    .. math::

        \begin{bmatrix} \dot{x} \\ \dot{w} \\ \dot{\hat{x}} \\ \dot{\hat{w}} \end{bmatrix} = \begin{bmatrix} w^{\alpha+1} f(w^{-1}(x, \hat{x}) \\ 0 \\ - w^{\alpha+1} f(w^{-1}(x, \hat{x}) \\ 0  \end{bmatrix}.

    :param exprs: Sympy expression(s) of :math:`[f(x)^T, -f(x)^T]`.
    :type exprs: List[:class:`~sympy:sympy.core.basic.Basic`] or :class:`~sympy:sympy.matrices.common.MatrixCommon`
    :param vars: List of sympy variables used in :attr:`exprs`.
    :type vars: List[:class:`~sympy:sympy.core.symbol.Symbol`]
    :param des_hom_degree: Desired degree of homogeneity :math:`\alpha`.
    :type des_hom_degee: float
    :return: New expressions, new symbol vector
    :rtype: tuple[list[:class:`~sympy:sympy.core.basic.Basic`], list[:class:`~sympy:sympy.core.symbol.Symbol`]] or tuple[:class:`~sympy:sympy.matrices.common.MatrixCommon`, list[:class:`~sympy:sympy.core.symbol.Symbol`]]

