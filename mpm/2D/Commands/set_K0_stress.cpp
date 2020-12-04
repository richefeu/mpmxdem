#include "set_K0_stress.hpp"

#include <Core/MPMbox.hpp>
#include <Core/MaterialPoint.hpp>

#include <factory.hpp>
static Registrar<Command, set_K0_stress> registrar("set_K0_stress");

void set_K0_stress::read(std::istream& is) { is >> nu >> rho0; }

void set_K0_stress::exec() {
  if (box->MP.empty()) return;
  vec2r ug = box->gravity;
  double g = ug.normalize();

  vec2r surfacePoint = box->MP[0].pos;
  double dmin = box->MP[0].pos * ug;
  double d = dmin;
  for (size_t p = 1; p < box->MP.size(); ++p) {
    d = box->MP[p].pos * ug;
    if (d < dmin) {
      dmin = d;
      surfacePoint = box->MP[p].pos;
    }
  }

  for (size_t p = 0; p < box->MP.size(); ++p) {
    d = (box->MP[p].pos - surfacePoint) * ug;
    box->MP[p].stress.yy = -d * g * rho0;
    box->MP[p].stress.xx = -d * g * rho0 * nu / (1.0 - nu);
    box->MP[p].stress.xy = box->MP[p].stress.yx = 0.0;

    // box->MP[p].prevStress = box->MP[p].stress;
  }
}
