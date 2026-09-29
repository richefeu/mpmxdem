#include "set_K0_stress.hpp"

#include "Core/MPMbox.hpp"
#include "Core/MaterialPoint.hpp"

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

    // Under plane strain the two horizontal directions are equivalent, so the
    // out-of-plane stress takes the same K0 value as sigma_xx. Leaving it at
    // zero -- as this command used to -- starts the computation from a state
    // that is not plane strain at all, and the models that integrate sigma_zz
    // carry that error for the whole run.
    box->MP[p].outOfPlaneStress = box->MP[p].stress.xx;
  }
}
