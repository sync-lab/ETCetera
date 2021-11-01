################
Theory
################

************************
Event-Triggered Control
************************

Linear ETC
----------------------

This section quickly summarizes the most important points from (Heemels2012). For a more detailed overview see (heemels2012).

We start with a LTI system :math:`\dot{x} = Ax + Bu, \, x \in \mathbb{R}^n, \, u \in \mathbb{R}^{n_u}` and feedback law :math:`u = Kx` which renders the closed loop asympotically stable. The most common approach that is used to implement this controller is to periodically recompute this control law. However, in ETC it is only recalculated when the performance is not as required.
For example, suppose the Lyapunov function :math:`V(x) = x^TPx` shows stability of the closed loop (for continuous control): :math:`\dot{V}(x(t)) = -x^T Q x`. Then we can continuously check :math:`\dot{V}(x(t)) \leq -\sigma x^T Q x`, and if it is violated, recalculate/send the control law. This defines the time instants :math:`t_k( , \, k \in \mathbb{N})`, called the triggering times, when this condition is violated.
Define the error: :math:`\epsilon(t) = x(t_k) = x(t), \, \forall t \in [t_k, t_{k+1}[,\, k \in \mathbb{N}`, and subsequently we can rewrite the time evolution of the Lyapunov function: :math:`\dot{V}(x(t)) = -x^T(t)Qx(T) + 2x^T(t)PBKe(t)`. Combining this with the previous condition, we get:

.. math::

    \phi(x, e) := \begin{bmatrix} x^T(t) & \epsilon^T(t) \end{bmatrix} \begin{bmatrix} (\sigma - 1)Q & PBK \\ K^TB^TP & 0 \end{bmatrix} \begin{bmatrix} x(t) \\ \epsilon(t) \end{bmatrix} =: z^T(t)\Psi z(t) \leq 0,

From which the triggering times :math:`t_k` can be defined as the instants when :math:`\phi(z) = z^T(t)\Psi z(t) = 0` holds. For other triggering conditions and stability concerns see ...
The triggering times can be defined by:

.. math::

    t_{k+1} = \inf \{t > t_k \mid z^T(t) \Psi z(t) > 0\}.

Linear PETC
----------------------

Periodic Event Triggered Control (PETC) is almost the same as regular ETC, however this time, the triggering condition is only checked with some period :math:`h`. This means that the condition has to be designed in such a way that the associated performance metric is not violated in between sampling times. Using a similar approach as before, we arrive at the quadratic triggering condition:

.. math::

    \begin{bmatrix} x^T(t) & \hat{x}^T(t) \end{bmatrix} \begin{bmatrix} A_d^TP_dA_d - \beta P_d & A_d^TP_dB_dK \\ (B_dK)^TP_dA_d & (B_dK)^TP_dB_dK \end{bmatrix} \begin{bmatrix} x(t) \\ \hat{x}(t) \end{bmatrix} \leq 0,

where :math:`\hat{x}` is the state currently known to the controller: :math:`\dot{\hat{x}}(t) = 0, \, \hat{x}(t_k^+) = x(t_k)`, :math:`A_d := e^{Ah}`, :math:`B_d := \int_0^h e^{As}Bds` and :math:`P_d` is the matrix for the Lyapunov function which shows stability of the discretized dynamics. The triggering times are multiples of the sampling period: :math:`t_{i+1} - t_i = k_ih, \, k_i \in \mathbb{N}`, and oftentimes an upperbound :math:`\bar{k}h` on the inter-event time is imposed, to make sure the system triggers at least every :math:`\bar{k}` samples.

Nonlinear ETC
----------------------


****************
Traffic Models
****************

The common goal of the two abstraction algorithms is to construct a system which captures the triggering behaviour of a (P)ETC system. For this we define a system: A system is a tuple: :math:`S = (X, X_0, U, \longrightarrow, Y, H)`, where :math:`X` is a set of states, :math:`X_0 \subseteq X` is a set of initial states, :math:`U` is a set of actions, :math:`\longrightarrow \subseteq X \times U \times X` are transitions between states, :math:`Y` is a set of outputs and :math:`H: X \to Y` is a map from the states to the outputs.
These systems can be infinite, but for these traffic models to be useful we would like to construct finite equivalent systems.

Traffic Models for Linear PETC
--------------------------------

This section summarizes the approaches in ~\cite{1,2,3} and is implemented by :class:`~ETCetera.Abstractions.TrafficModelLinearPETC`. First define

.. math::

    M(k) &= e^{Akh} + \int_0^{kh} e^{A\tau}d\tau BK, \\
    N(k) &= \begin{bmatrix} M(k) \\ I \end{bmatrix}^T Q \begin{bmatrix} M(k) \\ I \end{bmatrix}, \\
    \kappa(x) &= \min \{k \in \{1, 2, \dots, \bar{k}\} \mid x^T N(k) x > 0 \lor k = \bar{k}\},

where :math:`Q` is the triggering matrix of the quadratic triggering condition.

The system corresponding to a linear PETC system is: :math:`S = (X, X_0, U, \longrightarrow, Y, H)`, where
 - :math:`X \subseteq \mathbb{R}^n`,
 - :math:`X_0 \subseteq X`,
 - :math:`U = \emptyset`
 - :math:`\longrightarrow = \{(x, x') \subseteq X \times X \mid x' = \xi(h\kappa(x); x) = M(h\kappa(x))x\}`,
 - :math:`Y = \{1, 2, \dots, \bar{k}\}`,
 - :math:`H(x) = \kappa(x)`.

This system is infinite, so the goal is to construct a system :math:`S_{\setminus\mathcal{Q}}` that can generate the same behaviour, but is finite.
First, the state space is partitioned based on the trigger time:

.. math::

    \mathcal{Q}_k = \{x \mid \kappa(x) = k\}.

These sets form the states: :math:`X_{\setminus\mathcal{Q}} = \{\mathcal{Q}_1, \dots \}`. Transitions between these states can be calculated by solving a quadratic constrain satisfaction problem:

.. math::

        \exists & x \in \mathbb{R}^n\\
		\textrm{s.t.} \quad & x^T N(i) x > 0, \\
							& x^T N(i') x \leq 0, \, \forall i' \in \{1, \dots, i-1\}    \\
							& x^T M(i)^T N(j) M(i) x > 0, \\
							& x^T M(i)^T N(j') M(i) x \leq 0, \, \forall j' \in \{1, \dots j-1\}.

This can be solved exactly using ``z3`` (slowish) or semi-definite relaxations (SDR) can be used and solved using ``cvx``. This defines the transitions :math:`\mathcal{E}_{\setminus \mathcal{Q}} := \{(\mathcal{Q}_i, i, \mathcal{Q}_j) \mid \text{satisfying the problem}\}`. We can allow early (late) triggers by replacing :math:`i` by :math:`k` for any :math:`k \in \mathbb{N}, \, k < i` (:math:`k > i`), resulting in more transitions: :math:`(\mathcal{Q}_i, k, \mathcal{Q}_j) \in \mathcal{E}^*`  The traffic model :math:`S_{\setminus\mathcal{Q}}` then consists of:

- :math:`X_{\setminus \mathcal{Q}} = X_{\setminus \mathcal{Q},0} = \{\mathcal{Q}_1, \dots,  \mathcal{Q}_{k_{max}}\}`,
- :math:`\xrightarrow[\setminus \mathcal{Q}]{\quad \quad} := \mathcal{E}_{\setminus \mathcal{Q}} \cup \mathcal{E}^*`,
- :math:`U_{\setminus \mathcal{Q}} = Y_{\setminus \mathcal{Q}} = \{1, \dots, k_{max}\}`,
- :math:`H_{\setminus \mathcal{Q}}(\mathcal{Q}_k) = k`.


Traffic Models for Nonlinear ETC
-------------------------------------

This section summarizes the approach(es) in (....) and is implemented by :class:`~ETCetera.Abstractions.TrafficModelNonlinearETC`. First define

.. math::

    \tau(x) := \inf \{ t > 0 \mid \phi(\xi(t; x), \epsilon(t)) \geq 0 \},

where :math:`\phi(\cdot)` is the triggering condition. The system corresponding to a general nonlinear ETC system is :math:`S = (X, X_0, U, \longrightarrow, Y, H)`, where

 - :math:`X \subseteq \mathbb{R}^n`,
 - :math:`X_0 \subseteq X`,
 - :math:`U = \emptyset`
 - :math:`\longrightarrow = \{(x, x') \subseteq X \times X \mid x' = \xi(\tau(x); x)\}`,
 - :math:`Y \subseteq \mathbb{R}^+`,
 - :math:`H(x) = \tau(x)`.

Again, this system is infinite, so we want to construct a finite system :math:`S_{\setminus\mathcal{Q}}` which (almost) has the same behaviour.
There are two methods for creating the states :math:`X_{\setminus\mathcal{Q}}`. In the first one, a grid is defined on the state space, and the resulting (hyper-)cubes form the states :math:`S_{\setminus\mathcal{Q}}`.

The second one partitions the state space by intersections of cones and isochronous manifolds. For this approach, it is assumed that the ETC system is homogeneous.
A function f is homogeneous of degree :math:`\alpha`, if for all :math:`x \in \mathbb{R}^n` and :math:`\lambda > 0` : :math:`f(\lambda x) = \lambda^{\alpha+1} f(x)`. An ETC system then is homogeneous of degree :math:`\alpha` if :math:`f(x, u(x)) = \bar{f}(x)` is homogeneous of degree :math:`\alpha`.
General nonlinear ETC systems are often not homogeneous, but can be transformed into an equivalent system with an arbitrary degree of homogeneity :math:`\alpha` by introducing a new auxiliary variable :math:`w`:

.. math::

    \begin{bmatrix} \dot{x} \\ \dot{w} \\ \dot{\hat{x}} \\ \dot{\hat{w}} \end{bmatrix} = \begin{bmatrix} w^{\alpha+1} f(w^{-1}(x, \hat{x})) \\ 0 \\ - w^{\alpha+1} f(w^{-1}(x, \hat{x})) \\ 0  \end{bmatrix}.

These dynamics are equivalent to the original one when confined to the :math:`w=1` plane. It is also assumed that the triggering function is homogeneous of degree :math:`\theta \geq 1`. With these assumptions, the following property holds: :math:`\tau(\lambda x) = \lambda^{-\alpha}\tau(x), \quad \lambda > 0`.

An isochronous manifold is a subset of the state space where every point shares the same trigger time: :math:`M_{\tau_*} = \{x \in \mathbb{R}^n : \tau(x) = \tau_*\}`. Now, we can define regions which are enclosed by two isochronous manifolds: :math:`M_{\tau_i}` and :math:`M_{\tau_{i+1}}`, with :math:`\tau_i < \tau_{i+1}`. These can not be derived analytically, however, so inner-approximations of the manifolds :math:`M_{\tau_i}` are calculated. To do this a `conservative approximation` :math:`\mu(x,t)` of the triggering function is constructed, and the inner-approximations can be written as :math:`\underline{M}_{\tau_*} = \{x \in \mathbb{R}^n \mid \mu(x, \tau_*) = 0\}`. The state space is then split into regions enclosed by inner-approximations of isochronous manifolds, defined by a given list of trigger times :math:`\{\underline{\tau}_i\}`: :math:`\mathcal{R}_i := \{x \in \mathbb{R}^n \mid \mu(x,\underline{\tau}_{i+1}) > 0, \mu(x,\underline{\tau}_{i}) \leq 0\}`. However, since these regions are generally quite large, the accuracy of the reachability analysis that has to be performed might be poor. So they are split further by overlaying cones: :math:`\mathcal{C}_j = \{ x \in \mathbb{R}^n \mid E_jx \preceq 0 \}`, resulting in the new regions :math:`\mathcal{R}_{i,j} = \mathcal{R}_i \cap \mathcal{C}_j`.

So now we have a set of regions :math:`\{\mathcal{R}_{i,j}\}` with lower bound on the triggering time :math:`\underline{\tau}_{\mathcal{R}_{i,j}} = \underline{\tau}_i`. However, these regions are very difficult to handle computationally, so to determine the upper bound on the trigger time and transitions between the regions, they are first overapproximated by ball segments: :math:`\hat{\mathcal{R}}_{i,j} = \{x \in \mathcal{C}_j \mid \underline{r}_{i+1,j} \leq |x| \leq \underline{r}_{i,j} \}`. These over-approximated regions are then used to calculate the upper bounds :math:`\overline{\tau}_{\mathcal{R}_{i,j}}` and the transitions that can occur.

The final obtained system :math:`S_{\setminus\mathcal{Q}}` then consists of:
 - :math:`X_{\setminus\mathcal{Q}} = \{\mathcal{R}_{i,j}\}`,
 - :math:`X_{0\setminus\mathcal{Q}} = \{ \mathcal{R}_{i,j} \mid \mathcal{R}_{i,j} \cap X_0 \neq \emptyset\}`,
 - :math:`U_{\setminus\mathcal{Q}} = \emptyset`,
 - :math:`(\mathcal{R}_{i,j}, \mathcal{R}_{a,b}) \in \xrightarrow[\setminus \mathcal{Q}]{\quad \quad} \text{if}: \: \mathcal{X}^f_{[\underline{\tau}_{\mathcal{R}_{i,j}}, \overline{\tau}_{\mathcal{R}_{i,j}}]}(\hat{\mathcal{R}}_{i,j}) \cap \hat{\mathcal{R}}_{a,b} \neq \emptyset`,
 - :math:`Y_{\setminus\mathcal{Q}} \subseteq 2^Y`,
 - :math:`H_{\setminus\mathcal{Q}}(\mathcal{R}_{i,j}) := [\underline{\tau}_{\mathcal{R}_{i,j}}, \overline{\tau}_{\mathcal{R}_{i,j}}]`



****************
Scheduling
****************

One of the uses of these constructed traffic models is to generate schedulers for the (P)ETC systems. The objective of these schedulers is to trigger each of the seperate control loops before or when the triggering condition is satisfied, but preventing a collision of these triggering events within a given timeframe.
This tool implements two ways to construct these schedulers:

- Solving safety games on the traffic models.
- Transform the traffic models into TGAs and generating a strategy using UPPAAL Stratego.

.. _theory_sg:

Safety Game
-------------------

This section summarizes the approach in (....) and implemented by :mod:`~ETCetera.Scheduling.fpiter`.

This approach is based on the fact that in PETC, every sample the triggering condition decides to wait or to trigger. The traffic models are thus converted to represent this `choose wait/trigger every sample` behaviour.
In this way, this approach is currently only applicable to PETC. This approach scales much better than the other approach, and thus is the default for PETC.

The PETC traffic model :math:`S_{\setminus\mathcal{Q}}` generated by :class:`~ETCetera.Abstractions.TrafficModelLinearPETC` is first converted into a different form (in :func:`~ETCetera.Scheduling.fpiter.controlloop`): System :math:`\hat{S} = (\hat{X}, \hat{X}_0, \hat{U}, \xrightarrow[\hat{S}]{}, \hat{Y}, \hat{H})`, where:
 - :math:`\hat{X} = \{T_{k}, W_{k, 1}, \dots, W_{k, k_1-1} \mid k = (k_1, k_2, \dots) \in X_{\setminus\mathcal{Q}} \}`,
 - :math:`\hat{X}_0 = \hat{X}`,
 - :math:`\hat{U} = \{w, t\}`,
 - :math:`\xrightarrow[\hat{S}]{} = \{`
    - :math:`(W_{k, j}, w, W_{k,j+1})`,
    - :math:`(T_i, t, T_j)`, if :math:`\exists (i, 1, j)` in original traffic model.
    - :math:`(W_{k,i}, t, T_j)` if :math:`\exists (k, i, j)` in original traffic model.

    :math:`\}`,
 - :math:`\hat{H}(T_1) = T_1`,  :math:`\hat{H}(T_i) = T` (:math:`i>1`), :math:`\hat{H}(W_{k,j}) = W_{k_1-j}`.

These systems are then combined into a larger system :math:`S_{comp.}` (implemented in :func:`~ETCetera.Scheduling.fpiter.system`), where each element is just the product of the elements of the individual control loops, i.e. :math:`X_{comp.} := \hat{X}_1 \times \hat{X}_2 \times \dots`, etc. This `composed` system represents all the control loops running individually in parallel.
The states in the composed system where two or more of the control loops have an output :math:`T / T_1` represent the event of a trigger collision, which the scheduler thus should avoid. The scheduler will choose an action (`wait/trigger`) for each control loop which will guarantee that these collision states can be avoided.
This will be done by solving a safety game. Define the operator:

.. math::

    F_W(Z) = \{x \in Z \mid x \in W \land \exists u \in U(x) : \: \emptyset \neq Post_u(x) \subseteq Z\}.

Where :math:`W` is the safe set (the states no collision occurs), which in this case is:

.. math::

    W = \{(x_1, \dots, x_n) \in X_{comp.} \, \vert \, \exists_{\leq 1} x_i : \: H_i(x_i) \in \{T_1, T\}\}.

The safety game will iterate over this operator :math:`F_W` (starting with :math:`Z_0 := X_{comp.}`), which will iteratively remove states that are either not safe, or are not guaranteed to lead to a state in :math:`Z_i`:

.. math::

    Z_{sol.} := \lim_{i\to\infty} F_W^i(X_{comp.}).

The scheduler can be generated from this solution by:

.. math::

    U_c(x) = \{u \in U_{comp.} \mid \emptyset \neq Post_{u}(x) \subseteq Z \}.

To make this process (potentially) more efficient, states in the original control loop systems can be grouped together (called partitioning and refining), or the systems can be represented using so called Binary Decision Diagrams (BDDs). For details on this see ~\cite{})

.. _theory_ntga_uppaal:

NTGAs and UPPAAL Stratego
------------------------------------

This section summarizes the approach in (....) and implemented by :mod:`~ETCetera.Scheduling.NTGA`.

This approach is based on the use of Timed Game Automata (TGAs) to design a scheduler. It scales somewhat worse than using a safety game to construct a scheduler, but has the advantage that it has more customization possible and that it can be applied also for general ETC.

.. note::

    The following discusses how a PETC traffic model is converted. However, an almost identical approach can be performed for general ETC traffic models.

Each traffic model is converted into a TGA (implemented in :class:`~ETCetera.Scheduling.NTGA.controlloop`):  :math:`TGA^{CL} = (L^{CL}, l_0^{CL}, Act_c^{CL}, Act_u^{CL}, C^{CL}, E^{CL}, Inv^{CL})`, where:

 - :math:`L^{CL} = \{Transition\_loc, CLK\_wait, Bad \} \cup \{R_k, Ear_k \mid k \in X_{\setminus\mathcal{Q}}\}`,
 - :math:`l_0^{CL} = R_k` such that :math:`x_0 \in R_k`,
 - :math:`Act_c^{CL} = \{ack?, nack?, down!, timeout! \} \cup \{*\}`,
 - :math:`Act_u^{CL} = \{up!\}`,
 - :math:`C^{CL} = \{c\}`,
 - :math:`E^{CL}` is a set containing the following edges:

    - For each of the transitions :math:`(\mathcal{Q}_i, k, \mathcal{Q}_j)` in the traffic model:

        - If :math:`Inv(\mathcal{Q}_i) > k` (Thus early trigger): Add transition :math:`(Ear_i, true, up!, to\_region:=j, Transition\_loc)`
        - Else add transition :math:`(R_i, \underline{\tau}_i \leq c \leq \overline{\tau}_i + N_{max}, up!, (to\_region:=j)\land(earlyCount:=0), Transition\_loc)`
    - :math:`\bigcup_{i=1}^q (R_i, (\underline{d}_i \leq c \leq \overline{d}_i)\land(earlyCount < earlyMax), *, \emptyset, Ear_i)`,
    - :math:`\bigcup_{i=1}^q (Transition\_loc, \{rt\_count < RT\_MAX, from\_region==i\}, nack?, rt\_count := rt\_count+1, R_i)`,
    - :math:`(Transition\_loc, true, ack?, \{c:=0, rt\_count:=0, from\_region:=to\_region\}, CLK\_wait)`,
    - :math:`\bigcup_{i=1}^q (CLK\_wait, \{to\_region == i, c == \Delta\}, down!, c :=0, R_i)`,
    - :math:`(Transition\_loc, rt\_count \geq RT\_MAX, timeout!, \emptyset, Bad)`;
 - :math:`Inv^{CL}(R_k) = \{c \mid c \leq \bar{\tau}_k + N_{max}\}, Inv^{CL}(CLK\_wait) = \{c \mid c \leq \Delta\}`

Where :math:`\Delta` is the time the communication channel is occupied when a trigger occurs, and in case of PETC: :math:`\underline{\tau}_i = \overline{\tau}_i = Inv(\mathcal{Q}_i)` and :math:`\overline{d}_i = \underline{\tau}_i, \underline{d}_i = \overline{d}_i-max\_early`. The other parameters that can be set are :math:`RT\_MAX`: The maximum amount of retransmissions that is allowed and :math:`N_{max}`: The relaxation on the state upper bound to account for these retransmissions and :math:`earlyMax`: the maximum amount of sequential early triggers.

Aside from the TGAs representing the control loops, a TGA representing the network (implemented in :class:`~ETCetera.Scheduling.NTGA.network`) these control loops are part of has to be constructed. In this case it is: :math:`TGA^{net}`, with:
 - :math:`L^{net} = \{Idle, InUse, ACK\_loc, NACK\_loc, Bad\}`,
 - :math:`l^{net}_0 = Idle`,
 - :math:`Act_c^{net} = \{ack!, nack!, down?, timout?\}`,
 - :math:`Act_u^{net} = \{up?\}`,
 - :math:`C^{net} = \emptyset`,
 - :math:`E^{net} = \{(Idle, true, up?, \emptyset, ACK\_loc), (ACK\_loc, true,ack!, \emptyset, InUse)`,
    :math:`(InUse, true, up?, \emptyset, NACK\_loc), (NACK\_loc, true, nack!, \emptyset, InUse)`,
    :math:`(NACK\_loc, true, timeout?, \emptyset, Bad), (InUse, true, timeout?, \emptyset, Bad)\}`.

 - :math:`Inv^{net} = \emptyset` for all locations, since there are no clocks.

The control loops TGAs and the network TGA are then combined into a so-called `Network of Timed (Game) Automata` (implemented in :class:`~ETCetera.Scheduling.NTGA.nta`). In these, all the TAs run in parallel, and either each TA makes a move on its own, or a synchronization action takes place. In the above, the synchronization actions are of the form ``action?`` and ``action!``. For a synchronization to occur both ``?`` and ``!`` with the same action name have to take place at the same time.
The goal of the scheduler is to avoid the occurance of one of the control loops to time-out and for the network to end up in the bad states. This scheduling strategy is generated using `UPPAAL Stratego <https://people.cs.aau.dk/~marius/stratego/>`_ by the command: ``control: A[] not Network.Bad``