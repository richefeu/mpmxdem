.. _Dissipation:

Mixing PIC and FLIP
===================

The MPM is a complex process that can make it hard (or even impossible) to keep a simulation stable. When only elasticity is considered, there's no way to dissipate energy, like irreversible deformations or viscosity. To help with this, a trick can be used: combining PIC and FLIP methods during the time steps of the simulation. PIC is known to reduce energy numerically, while FLIP reduces energy much less.








