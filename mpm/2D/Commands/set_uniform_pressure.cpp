#include "set_uniform_pressure.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

void set_uniform_pressure::read(std::istream& is) { is >> pressure; }

void set_uniform_pressure::exec() {
  if (box->MP.empty()) return;

  for (size_t p = 0; p < box->MP.size(); ++p) {
    box->MP[p].stress.yy = -pressure;
    box->MP[p].stress.xx = -pressure;
    box->MP[p].stress.xy = box->MP[p].stress.yx = 0.0;
    box->MP[p].outOfPlaneStress = -pressure;
  }
}
