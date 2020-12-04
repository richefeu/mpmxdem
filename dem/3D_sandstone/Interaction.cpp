#include "Interaction.hpp"

Interaction::Interaction()
    : i(0), j(0), gap0(0.0), n(), fn(0.0), fn_elas(0.0), fn_bond(0.0), ft(), ft_fric(), ft_bond(), dt_fric(), dt_bond(),
      drot_bond(), mom(), dampn(0.0), dampt(0.0), state(noContactState), D(0.0) {}

Interaction::Interaction(const Interaction &I)
    : i(I.i), j(I.j), gap0(I.gap0), n(I.n), fn(I.fn), fn_elas(I.fn_elas), fn_bond(I.fn_bond), ft(I.ft),
      ft_fric(I.ft_fric), ft_bond(I.ft_bond), dt_fric(I.dt_fric), dt_bond(I.dt_bond), drot_bond(I.drot_bond),
      mom(I.mom), dampn(I.dampn), dampt(I.dampt), state(I.state), D(I.D) {}

Interaction::Interaction(size_t I, size_t J, double Dampn, double Dampt)
    : i(I), j(J), gap0(0.0), n(), fn(0.0), fn_elas(0.0), fn_bond(0.0), ft(), ft_fric(), ft_bond(), dt_fric(), dt_bond(),
      drot_bond(), mom(), dampn(Dampn), dampt(Dampt), state(noContactState), D(0.0) {}
