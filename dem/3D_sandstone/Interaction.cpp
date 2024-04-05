#include "Interaction.hpp"

Interaction::Interaction()
    : i(0),
      j(0),
      gap0(0.0),
      n(),
      fn(0.0),
      fn_elas(0.0),
      fn_bond(0.0),
      ft(),
      ft_fric(),
      ft_bond(),
      dt_fric(),
      dt_bond(),
      drot_bond(),
      drot_fric(),
      mom(),
      mom_fric(),
      mom_bond(),
      dampn(0.0),
      dampt(0.0),
      state(noContactState),
      D(0.0) {}

Interaction::Interaction(size_t I, size_t J, double Dampn, double Dampt)
    : i(I),
      j(J),
      gap0(0.0),
      n(),
      fn(0.0),
      fn_elas(0.0),
      fn_bond(0.0),
      ft(),
      ft_fric(),
      ft_bond(),
      dt_fric(),
      dt_bond(),
      drot_bond(),
      drot_fric(),
      mom(),
      mom_fric(),
      mom_bond(),
      dampn(Dampn),
      dampt(Dampt),
      state(noContactState),
      D(0.0) {}

bool Interaction::operator<(const Interaction& other) const {
  if (i == other.i) {
    return j < other.j;
  }
  return i < other.i;
}
