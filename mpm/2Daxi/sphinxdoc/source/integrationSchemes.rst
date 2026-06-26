.. _IntegrationSchemes:

Integration schemes
===================

In a ``conf-file``, the integration scheme to be used is set with the keyword ``Integrator`` 
followed by the name of the scheme (defined below). The position, velocity and acceleration 
vectors is noted :math:`\underline{x}`, :math:`\underline{v}` and :math:`\underline{a}`, respectively; 
and for rotations they are :math:`\mathbf{Q}` (a quaternion), :math:`\underline{\omega}` and :math:`\underline{\dot{\omega}}`. 

For all these schemes, the driven bodies with force or moment are updated similarly to the chosen scheme, 
and the driven bodies with velocity imposed (translation or rotation) are updated that way:

.. math::

   \begin{cases}
   \underline{x}_{t+\Delta t} &= \underline{x}_{t} + \underline{v}^\mbox{imposed}_{t} \Delta t \\
   \mathbf{Q}_{t+\Delta t} &= \mathbf{Q}_{t} + \mathbf{\dot{Q}}\left(\underline{\omega}^\mbox{imposed}_{t}\right) \Delta t 
   \end{cases}

Euler scheme (keyword ``Euler``)
--------------------------------

The Euler scheme is a simple and intuitive time integration method, but it has limitations. One of the main limitations is that it is conditionally stable. Therefore, the time step size must be carefully chosen to ensure numerical stability. Additionally, the explicit Euler method is only first-order accurate, meaning that errors in position and velocity tend to accumulate over time, especially for stiff systems. This can lead to inaccuracies in the simulation, especially for long integration times or complex systems.


For a discrete particle in DEM, the equations used to update its position and velocity using the Euler scheme are as follows:


**Position update**


The new position and orientation of a particle, :math:`\underline{x}_{t+\Delta t}` and :math:`\mathbf{Q}_{t+\Delta t}`, at the next time step (:math:`t+\Delta t`) are obtained from its current position and orientation, :math:`\underline{x}_t` and :math:`\mathbf{Q}_{t}`, and its velocity, :math:`\underline{v}_t` and :math:`\mathbf{\dot{Q}}_t`, at the current time step (:math:`t`). The position update equation is given by:

.. math::

   \begin{cases}
   \underline{x}_{t+\Delta t} &= \underline{x}_t + \underline{v}_t \Delta t \\
	 \mathbf{Q}_{t+\Delta t} &= \mathbf{Q}_t + \mathbf{\dot{Q}}(\underline{\omega}_{t}) \Delta t
	 \end{cases}
	 
where :math:`\Delta t` is the time step size, which is a user-defined parameter.


**Velocity update**


The new velocity of a particle, :math:`\underline{v}_{t+\Delta t}` and :math:`\underline{\omega}_{t+\Delta t}`, at the next time step (:math:`t+\Delta t`) is obtained from its current velocity, :math:`\underline{v}_t` and :math:`\underline{\omega}_t`, and the acceleration, :math:`\underline{a}_t` and :math:`\underline{\dot{\omega}}_t`, at the current time step (:math:`t`). The velocity update equation is given by:

.. math::

   \begin{cases}
   \underline{v}_{t+\Delta t}     &= \underline{v}_t + \underline{a}_t \Delta t \\
	 \underline{\omega}_{t+\Delta t} &= \underline{\omega}_t + \underline{\dot{\omega}}_t \Delta t
	 \end{cases}
	 
where :math:`\underline{a}_t` (and :math:`\underline{\dot{\omega}}_t`) is the acceleration calculated from the forces (and moments) acting on the particle at time step :math:`t`.



